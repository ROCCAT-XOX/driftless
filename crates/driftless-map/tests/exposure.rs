//! Exposure and white balance: images rendered with known per-image gains (the right camera
//! brighter and bluish, the left one brightening over time) must get those gains back, up to
//! the common factor the fit cannot see, and the views of a point must agree afterwards.

mod common;

use common::*;
use driftless_map::{
    colorize, estimate_exposure, estimate_gains, ColorCamera, ColorConfig, ColorStats,
    ExposureConfig, ImageSource, MapError, Observations, ViewObs,
};
use driftless_traj::nalgebra::{Isometry3, UnitQuaternion};
use driftless_traj::Pose;
use std::sync::OnceLock;

/// Stamps of the image pairs (left and right at the same time).
const STAMPS: usize = 11;

/// The walk, turning slowly about z (0.45 rad/s), so the right camera later sees what the
/// left one saw before.
struct TurningWalk(Shifted<Walk>);

impl Trajectory for TurningWalk {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        let yaw = 0.45 * (t - T0).max(0.0);
        self.0.pose(t) * UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
    }
}

/// A view seen on the optical axis.
fn view(image: u32, rgb: [f32; 3]) -> ViewObs {
    ViewObs {
        image,
        rgb,
        theta: 0.0,
    }
}

fn stamp(i: usize) -> f64 {
    T0 + 0.5 * (i + 1) as f64
}

/// Injected gain of image `i` of `camera` (linear): left 0.8 → 1.2 over the walk, right 1.3×
/// brighter and bluish.
fn injected(camera: usize, i: usize) -> [f64; 3] {
    match camera {
        0 => [0.8 + 0.4 * i as f64 / (STAMPS - 1) as f64; 3],
        _ => [1.3 * 0.9, 1.3, 1.3 * 1.2],
    }
}

fn to_linear(v: u8) -> f64 {
    let c = f64::from(v) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn to_srgb(l: f64) -> u8 {
    let l = l.clamp(0.0, 1.0);
    let c = if l <= 0.003_130_8 {
        12.92 * l
    } else {
        1.055 * l.powf(1.0 / 2.4) - 0.055
    };
    (c * 255.0).round() as u8
}

/// Shot whose rendered pixels are scaled by `gain` in linear light (clipped at white).
fn gained(camera: usize, stamp: f64, gain: [f64; 3]) -> Shot {
    Shot {
        camera,
        stamp,
        edit: Box::new(move |rgb| {
            rgb.chunks(3)
                .flat_map(|px| (0..3).map(move |c| to_srgb(to_linear(px[c]) * gain[c])))
                .collect()
        }),
    }
}

/// The turning walk with injected gains, rendered once for all tests that use it.
fn gained_data() -> &'static (Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>) {
    static DATA: OnceLock<(Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>)> = OnceLock::new();
    DATA.get_or_init(|| {
        let painted = walk_scene();
        let cams = cameras();
        let traj = TurningWalk(walk());
        let shots: Vec<Shot> = (0..STAMPS)
            .flat_map(|i| [0, 1].map(|c| gained(c, stamp(i), injected(c, i))))
            .collect();
        let images = write_bag("gained", &painted, &traj, &cams, &shots);
        (painted, images, poses(&traj, 6.0), cams)
    })
}

fn run(cfg: &ColorConfig) -> (driftless_map::Cloud, ColorStats) {
    let (painted, images, poses, cams) = gained_data();
    let mut cloud = surface_cloud(painted);
    let stats = colorize(&mut cloud, images.clone(), poses, cams, cfg).unwrap();
    (cloud, stats)
}

/// Largest relative error of the fitted gains against the injected ones, in the fit's gauge
/// (the mean log gain per channel is 0); asserts each is within 3 %.
fn assert_gains_recovered(stats: &ColorStats) -> f64 {
    assert_eq!(stats.images_used, 2 * STAMPS, "{stats:?}");
    assert_eq!(stats.gains.len(), 2 * STAMPS);
    let truth: Vec<[f64; 3]> = stats
        .gains
        .iter()
        .map(|g| {
            let camera = usize::from(g.camera == "right");
            let i = (0..STAMPS)
                .position(|i| stamp(i) == g.stamp)
                .expect("known stamp");
            injected(camera, i)
        })
        .collect();
    let mut worst = 0.0f64;
    for c in 0..3 {
        let mean_log = truth.iter().map(|t| t[c].ln()).sum::<f64>() / truth.len() as f64;
        for (g, t) in stats.gains.iter().zip(&truth) {
            let expected = t[c] / mean_log.exp();
            let err = (f64::from(g.gain[c]) / expected - 1.0).abs();
            worst = worst.max(err);
            assert!(
                err < 0.03,
                "{} at {:.2} channel {c}: {} vs {expected:.4}",
                g.camera,
                g.stamp - T0,
                g.gain[c]
            );
        }
    }
    worst
}

#[test]
fn recovers_injected_gains() {
    let (_, stats) = run(&ColorConfig::default());
    let worst = assert_gains_recovered(&stats);
    println!(
        "worst gain error {:.2} %; gains {:.3}…{:.3}; spread {:.2} → {:.2}",
        100.0 * worst,
        stats.gain_min,
        stats.gain_max,
        stats.spread_before,
        stats.spread_after
    );
    assert!(stats.spread_before > 0.0);
    assert!(
        stats.spread_after < 0.25 * stats.spread_before,
        "spread {} → {}",
        stats.spread_before,
        stats.spread_after
    );
    assert_eq!(stats.images_gain_clamped, 0);
}

/// Largest angle of the test cameras (rad): the equidistant fisheye without distortion
/// projects uniquely up to the 100° limit.
const THETA_MAX: f64 = 100.0 * std::f64::consts::PI / 180.0;

/// Injected edge falloff of both cameras.
fn falloff(theta: f64) -> f64 {
    1.0 - 0.5 * (theta / THETA_MAX).powi(2)
}

/// Like [`gained`], with every pixel also darkened by [`falloff`] of its angle from the
/// optical axis.
fn vignetted(camera: usize, stamp: f64, gain: [f64; 3]) -> Shot {
    let f = f64::from(W) / 2.0 / 80f64.to_radians();
    let (cx, cy) = (f64::from(W) / 2.0 - 0.5, f64::from(H) / 2.0 - 0.5);
    Shot {
        camera,
        stamp,
        edit: Box::new(move |rgb| {
            rgb.chunks(3)
                .enumerate()
                .flat_map(|(p, px)| {
                    let (u, v) = ((p as u32 % W) as f64, (p as u32 / W) as f64);
                    let v = falloff((u - cx).hypot(v - cy) / f);
                    (0..3).map(move |c| to_srgb(to_linear(px[c]) * gain[c] * v))
                })
                .collect()
        }),
    }
}

fn vignetted_data() -> &'static (Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>) {
    static DATA: OnceLock<(Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>)> = OnceLock::new();
    DATA.get_or_init(|| {
        let painted = walk_scene();
        let cams = cameras();
        let traj = TurningWalk(walk());
        let shots: Vec<Shot> = (0..STAMPS)
            .flat_map(|i| [0, 1].map(|c| vignetted(c, stamp(i), injected(c, i))))
            .collect();
        let images = write_bag("vignetted", &painted, &traj, &cams, &shots);
        (painted, images, poses(&traj, 6.0), cams)
    })
}

fn run_vignetted(cfg: &ColorConfig) -> (driftless_map::Cloud, ColorStats) {
    let (painted, images, poses, cams) = vignetted_data();
    let mut cloud = surface_cloud(painted);
    let stats = colorize(&mut cloud, images.clone(), poses, cams, cfg).unwrap();
    (cloud, stats)
}

#[test]
fn recovers_injected_vignetting() {
    let (_, stats) = run_vignetted(&ColorConfig::default());
    let v = &stats.vignetting;
    assert_eq!(v.coeffs.len(), 2, "{v:?}");
    let mut worst = 0.0f64;
    for cam in 0..2 {
        for c in 0..3 {
            // Class centres 5°, 15°, …, 75°: the kept views lie below about 80°.
            for class in 0..8 {
                let theta = (10.0 * class as f64 + 5.0).to_radians();
                let fitted = f64::from(v.factor(cam, c, theta as f32));
                let err = (fitted / falloff(theta) - 1.0).abs();
                worst = worst.max(err);
                assert!(
                    err < 0.03,
                    "camera {cam} channel {c} at {:.0}°: {fitted:.4} vs {:.4}; {v:?}",
                    theta.to_degrees(),
                    falloff(theta)
                );
            }
        }
    }
    let gains = assert_gains_recovered(&stats);
    println!(
        "worst falloff error {:.2} %, gain error {:.2} %; residual {:.2} → {:.2} %; {v:?}",
        100.0 * worst,
        100.0 * gains,
        stats.vignette_residual_before,
        stats.vignette_residual_after
    );
    assert!(stats.vignette_residual_before > 5.0, "{stats:?}");
    assert!(
        stats.vignette_residual_after < 0.3 * stats.vignette_residual_before,
        "{stats:?}"
    );
}

/// Deterministic pseudo-random number in [0, 1) from `i`.
fn unit(i: u64) -> f64 {
    let x = i.wrapping_mul(0x9E37_79B9_7F4A_7C15).rotate_left(17) ^ 0xD1B5_4A32_D192_ED03;
    (x.wrapping_mul(0xBF58_476D_1CE4_E5B9) >> 11) as f64 / (1u64 << 53) as f64
}

/// Points seen by images 0 and 1 (camera 0) and 2 and 3 (camera 1) at pseudo-random angles
/// below `theta_max`, with gains 1, 1.2, 0.8 and 0.9 and the falloff `exp(b1 θ² + b2 θ⁴)` of
/// `coeffs` (per camera, same in every channel).
fn synthetic(points: usize, theta_max: f64, coeffs: [[f64; 2]; 2]) -> Observations {
    synthetic_on(vec![0, 0, 1, 1], points, theta_max, coeffs)
}

/// [`synthetic`] with the cameras of the images given (gains 1, 1.2, 0.8, 0.9, 1.1).
fn synthetic_on(
    cameras: Vec<usize>,
    points: usize,
    theta_max: f64,
    coeffs: [[f64; 2]; 2],
) -> Observations {
    let mut obs = Observations::new(cameras.clone());
    let gains = [1.0, 1.2, 0.8, 0.9, 1.1];
    for j in 0..points {
        let a = 0.05 + 0.3 * unit(8 * j as u64);
        let views: Vec<ViewObs> = (0..cameras.len())
            .map(|i| {
                let theta = theta_max * unit(8 * j as u64 + 1 + i as u64);
                let [b1, b2] = coeffs[cameras[i]];
                let t2 = theta * theta;
                let c = (a * gains[i] * (b1 * t2 + b2 * t2 * t2).exp()) as f32;
                ViewObs {
                    image: i as u32,
                    rgb: [c; 3],
                    theta: theta as f32,
                }
            })
            .collect();
        obs.push_point(&views);
    }
    obs
}

#[test]
fn recovers_synthetic_vignetting_from_observations() {
    let truth = [[-0.3, 0.02], [-0.15, -0.01]];
    let free = ExposureConfig {
        smooth: 0.0,
        ..ExposureConfig::default()
    };
    // A single camera too: no point links it to another, its falloff is fitted without the
    // camera offset.
    let (_, single) = estimate_exposure(
        &synthetic_on(vec![0, 0, 0, 0], 3000, 1.4, [truth[0]; 2]),
        &free,
    );
    let [b1, b2] = truth[0];
    for c in 0..3 {
        assert!(
            (single.coeffs[0][c][0] - b1).abs() < 0.01
                && (single.coeffs[0][c][1] - b2).abs() < 0.01,
            "{single:?}"
        );
    }
    let obs = synthetic(3000, 1.4, truth);
    let (gains, v) = estimate_exposure(&obs, &free);
    for (cam, [b1, b2]) in truth.iter().enumerate() {
        for c in 0..3 {
            for step in 0..=14 {
                let theta = 0.1 * step as f64;
                let t2 = theta * theta;
                let expected = (b1 * t2 + b2 * t2 * t2).exp();
                let fitted = f64::from(v.factor(cam, c, theta as f32));
                assert!(
                    (fitted / expected - 1.0).abs() < 0.005,
                    "camera {cam} at {theta}: {fitted} vs {expected}; {v:?}"
                );
            }
        }
    }
    let ratio = |a: usize, b: usize| f64::from(gains[a][1] / gains[b][1]);
    assert!((ratio(1, 0) - 1.2).abs() < 0.005, "{gains:?}");
    assert!((ratio(2, 0) - 0.8).abs() < 0.005, "{gains:?}");
    assert!((ratio(3, 0) - 0.9).abs() < 0.005, "{gains:?}");
}

#[test]
fn vignetting_unobservable_stays_flat() {
    // Every view within 20° of the optical axis: the falloff cannot be told apart from the
    // gains and the albedo, so it stays flat even though the data has one.
    let truth = [[-0.3, 0.0], [-0.3, 0.0]];
    let (_, v) = estimate_exposure(
        &synthetic(3000, 20f64.to_radians(), truth),
        &ExposureConfig::default(),
    );
    assert_eq!(v.coeffs, vec![[[0.0; 2]; 3]; 2]);
    // Wide angles, but camera 1 has fewer than 1,000 observations: only camera 0 is fitted.
    let (_, v) = estimate_exposure(
        &synthetic_on(vec![0, 0, 0, 1, 1], 400, 1.4, truth),
        &ExposureConfig::default(),
    );
    assert_eq!(v.coeffs[1], [[0.0; 2]; 3], "{v:?}");
    assert!(v.coeffs[0].iter().all(|b| b[0] < -0.2), "{v:?}");
    // A camera standing still: every point at the same angle in all images (up to 1e-4 rad),
    // the angles over the whole lens, the colours with 1 % noise. The falloff cannot be told
    // apart from the albedo.
    let mut still = Observations::new(vec![0, 0, 0]);
    for j in 0..3000u64 {
        let a = 0.05 + 0.3 * unit(8 * j);
        let theta = 1.4 * unit(8 * j + 1);
        let views: Vec<ViewObs> = (0..3u64)
            .map(|i| ViewObs {
                image: i as u32,
                rgb: [(a * (1.0 + 0.01 * (unit(8 * j + 2 + i) - 0.5))) as f32; 3],
                theta: (theta + 1e-4 * unit(8 * j + 5 + i)) as f32,
            })
            .collect();
        still.push_point(&views);
    }
    let (_, v) = estimate_exposure(&still, &ExposureConfig::default());
    assert_eq!(v.coeffs, vec![[[0.0; 2]; 3]], "{v:?}");

    // In a whole run, every factor stays within (0, 1].
    let (_, stats) = run(&ColorConfig::default());
    for cam in 0..2 {
        for step in 0..=20 {
            let f = stats.vignetting.factor(cam, 1, 0.08 * step as f32);
            assert!(f > 0.0 && f <= 1.0, "{f}");
        }
    }
}

#[test]
fn vignetting_off_matches_m3_gains() {
    // Without the falloff the gains are those of the plain exposure fit, which is what the
    // fit with falloff gives when no camera has an observable one (all angles 0).
    let truth = [[-0.3, 0.02], [-0.15, -0.01]];
    let obs = synthetic(3000, 1.4, truth);
    let off = ExposureConfig {
        vignetting: false,
        ..ExposureConfig::default()
    };
    let (gains, v) = estimate_exposure(&obs, &off);
    assert_eq!(v.coeffs, vec![[[0.0; 2]; 3]; 2]);
    assert_eq!(estimate_gains(&obs, &off), gains);
    let mut flat = Observations::new(vec![0, 0, 1, 1]);
    let wide = synthetic(3000, 1.4, truth);
    for j in 0..wide.points() {
        let views: Vec<ViewObs> = wide
            .point(j)
            .iter()
            .map(|v| ViewObs { theta: 0.0, ..*v })
            .collect();
        flat.push_point(&views);
    }
    let (flat_gains, flat_v) = estimate_exposure(&flat, &ExposureConfig::default());
    assert_eq!(flat_gains, gains);
    assert_eq!(flat_v.coeffs, vec![[[0.0; 2]; 3]; 2]);

    // In a whole run: flat falloff, the residual is not changed.
    let cfg = ColorConfig {
        exposure: off,
        ..ColorConfig::default()
    };
    let (_, stats) = run_vignetted(&cfg);
    assert!(stats.vignetting.coeffs.iter().all(|c| *c == [[0.0; 2]; 3]));
    assert_eq!(
        stats.vignette_residual_before,
        stats.vignette_residual_after
    );
    assert!(stats.vignette_residual_before > 5.0, "{stats:?}");
}

#[test]
fn gains_at_clamp_are_counted() {
    let max_gain = 1.1;
    let cfg = ColorConfig {
        exposure: ExposureConfig {
            max_gain,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let (_, stats) = run(&cfg);
    let (hi, lo) = (max_gain as f32, (1.0 / max_gain) as f32);
    let at_bound = stats
        .gains
        .iter()
        .filter(|g| g.gain.iter().any(|&x| x == hi || x == lo))
        .count();
    assert!(at_bound > 0, "{stats:?}");
    assert_eq!(stats.images_gain_clamped, at_bound);
    let (_, free) = run(&ColorConfig::default());
    assert_eq!(free.images_gain_clamped, 0);
}

#[test]
fn camera_difference_drops_after_equalisation() {
    // The right camera is 1.3× brighter and bluish: on points both cameras see, its views are
    // systematically brighter. The equalisation must remove most of that offset; the plain
    // spread mixes it with the scatter of the views and reacts less.
    let (_, stats) = run(&ColorConfig::default());
    println!(
        "camera difference {:.2} → {:.2} on {} points; spread {:.2} → {:.2}",
        stats.camera_difference_before,
        stats.camera_difference_after,
        stats.camera_difference_points,
        stats.spread_before,
        stats.spread_after
    );
    assert!(stats.camera_difference_points > 100, "{stats:?}");
    assert!(stats.camera_difference_before > 10.0, "{stats:?}");
    assert!(
        stats.camera_difference_after < 0.2 * stats.camera_difference_before,
        "{stats:?}"
    );
    // Without the equalisation nothing changes.
    let off = ColorConfig {
        exposure: ExposureConfig {
            samples: 0,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let (_, plain) = run(&off);
    assert_eq!(
        plain.camera_difference_before,
        plain.camera_difference_after
    );
    assert_eq!(
        plain.camera_difference_before,
        stats.camera_difference_before
    );
}

#[test]
fn image_offset_drops_after_equalisation() {
    // The left camera brightens from 0.8 to 1.2 over the walk and the right one is brighter:
    // most points are seen by a few images close in time, so the offsets are small, but each
    // image's mean offset must mostly vanish after the equalisation.
    let (_, stats) = run(&ColorConfig::default());
    println!(
        "image offset {:.2} → {:.2}",
        stats.image_offset_before, stats.image_offset_after
    );
    assert!(stats.image_offset_before > 1.0, "{stats:?}");
    assert!(
        stats.image_offset_after < 0.2 * stats.image_offset_before,
        "{stats:?}"
    );
}

#[test]
fn gains_are_clamped() {
    // Image 1 sees everything 100× brighter than image 0: unclamped gains 0.1 and 10.
    let mut obs = Observations::new(vec![0, 0]);
    for j in 0..100 {
        let a = 0.004 + 0.000_05 * j as f32;
        obs.push_point(&[view(0, [a; 3]), view(1, [100.0 * a; 3])]);
    }
    let gains = estimate_gains(&obs, &ExposureConfig::default());
    assert_eq!(gains, vec![[0.25; 3], [4.0; 3]]);
    let tight = ExposureConfig {
        max_gain: 2.0,
        ..ExposureConfig::default()
    };
    assert_eq!(estimate_gains(&obs, &tight), vec![[0.5; 3], [2.0; 3]]);

    // In a whole run every gain stays within the bounds.
    let cfg = ColorConfig {
        exposure: ExposureConfig {
            max_gain: 1.1,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let (_, stats) = run(&cfg);
    assert!(stats.gain_min >= 1.0 / 1.1 - 1e-6, "{stats:?}");
    assert!(stats.gain_max <= 1.1 + 1e-6, "{stats:?}");
    assert!(stats.gain_min < 0.95 && stats.gain_max > 1.05, "{stats:?}");
}

#[test]
fn single_camera_single_image_is_identity() {
    // No point is seen twice: nothing to compare, every gain is 1.
    let mut obs = Observations::new(vec![0]);
    obs.push_point(&[view(0, [0.2, 0.4, 0.6])]);
    assert_eq!(
        estimate_gains(&obs, &ExposureConfig::default()),
        vec![[1.0; 3]]
    );
    assert_eq!(
        estimate_gains(&Observations::new(vec![]), &ExposureConfig::default()),
        Vec::<[f32; 3]>::new()
    );

    let painted = walk_scene();
    let cams = cameras();
    let traj = walk();
    let images = write_bag(
        "single",
        &painted,
        &traj,
        &cams,
        &[gained(1, T0 + 1.0, [1.3, 1.3, 1.6])],
    );
    let poses = poses(&traj, 3.0);
    let colorize_with = |cfg: &ColorConfig| {
        let mut cloud = surface_cloud(&painted);
        let stats = colorize(&mut cloud, images.clone(), &poses, &cams, cfg).unwrap();
        (cloud, stats)
    };
    let (cloud, stats) = colorize_with(&ColorConfig::default());
    assert_eq!(stats.images_used, 1);
    assert_eq!(stats.gains.len(), 1);
    assert_eq!(stats.gains[0].gain, [1.0; 3]);
    assert_eq!((stats.gain_min, stats.gain_max), (1.0, 1.0));
    assert_eq!(stats.spread_before, 0.0);
    assert_eq!(stats.spread_after, 0.0);
    // The same colours as without the equalisation.
    let off = ColorConfig {
        exposure: ExposureConfig {
            samples: 0,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let (plain, plain_stats) = colorize_with(&off);
    assert_eq!(cloud.rgb, plain.rgb);
    assert_eq!(stats, plain_stats);
}

#[test]
fn exposure_off_keeps_gains_at_one() {
    let off = ColorConfig {
        exposure: ExposureConfig {
            samples: 0,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let (_, stats) = run(&off);
    assert!(stats.gains.iter().all(|g| g.gain == [1.0; 3]));
    assert_eq!((stats.gain_min, stats.gain_max), (1.0, 1.0));
    assert_eq!(stats.spread_before, stats.spread_after);
    assert!(stats.spread_before > 0.0);
}

#[test]
fn exposure_is_deterministic() {
    let (a, sa) = run(&ColorConfig::default());
    let (b, sb) = run(&ColorConfig::default());
    assert_eq!(sa, sb);
    assert_eq!(a.rgb, b.rgb);
    assert_eq!(a.colored, b.colored);
    assert!(sa.gain_min < 1.0 && sa.gain_max > 1.0, "{sa:?}");
    let (a, sa) = run_vignetted(&ColorConfig::default());
    let (b, sb) = run_vignetted(&ColorConfig::default());
    assert_eq!(sa, sb);
    assert_eq!(a.rgb, b.rgb);
    assert!(sa.vignetting.coeffs.iter().all(|c| c[0][0] < 0.0), "{sa:?}");
}

#[test]
fn rejects_bad_exposure_config() {
    let (painted, images, poses, cams) = gained_data();
    let mut cloud = surface_cloud(painted);
    let bad = [
        ExposureConfig {
            smooth: -1.0,
            ..ExposureConfig::default()
        },
        ExposureConfig {
            smooth: f64::NAN,
            ..ExposureConfig::default()
        },
        ExposureConfig {
            max_gain: 0.9,
            ..ExposureConfig::default()
        },
        ExposureConfig {
            max_gain: f64::INFINITY,
            ..ExposureConfig::default()
        },
    ];
    for exposure in bad {
        let cfg = ColorConfig {
            exposure,
            ..ColorConfig::default()
        };
        let r = colorize(&mut cloud, images.clone(), poses, cams, &cfg);
        assert!(matches!(r, Err(MapError::Config(_))), "{cfg:?}");
    }
}
