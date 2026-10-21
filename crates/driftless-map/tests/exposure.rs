//! Exposure and white balance: images rendered with known per-image gains (the right camera
//! brighter and bluish, the left one brightening over time) must get those gains back, up to
//! the common factor the fit cannot see, and the views of a point must agree afterwards.

mod common;

use common::*;
use driftless_map::{
    colorize, estimate_gains, ColorCamera, ColorConfig, ColorStats, ExposureConfig, ImageSource,
    MapError, Observations,
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

#[test]
fn recovers_injected_gains() {
    let (_, stats) = run(&ColorConfig::default());
    assert_eq!(stats.images_used, 2 * STAMPS, "{stats:?}");
    assert_eq!(stats.gains.len(), 2 * STAMPS);
    // The true gains in the fit's gauge: the mean log gain per channel is 0.
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
        obs.push_point(&[(0, [a; 3]), (1, [100.0 * a; 3])]);
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
    obs.push_point(&[(0, [0.2, 0.4, 0.6])]);
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
