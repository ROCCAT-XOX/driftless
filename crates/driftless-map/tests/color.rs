//! Colouring from synthetic camera images: a room with coloured walls is rendered (ray cast
//! per pixel) from known camera poses, the images go through JPEG into a bag, and the
//! cloud's points on the surfaces must get the colour of their surface.

mod common;

use common::*;
use driftless_map::{
    colorize, ColorCamera, ColorConfig, ColorStats, ExposureConfig, ImageSource, MapError,
    Vignetting,
};
use driftless_traj::nalgebra::{Isometry3, UnitQuaternion, Vector3};

/// Separable Gaussian blur with standard deviation `sigma` (pixels), edges clamped.
fn blur(rgb: &[u8], sigma: f64) -> Vec<u8> {
    let r = (3.0 * sigma).ceil() as i64;
    let kernel: Vec<f64> = (-r..=r)
        .map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp())
        .collect();
    let norm: f64 = kernel.iter().sum();
    let (w, h) = (i64::from(W), i64::from(H));
    let pass = |src: &[f64], horizontal: bool| -> Vec<f64> {
        let mut out = vec![0.0; src.len()];
        for y in 0..h {
            for x in 0..w {
                for c in 0..3 {
                    let mut s = 0.0;
                    for (k, kv) in kernel.iter().enumerate() {
                        let o = k as i64 - r;
                        let (sx, sy) = if horizontal {
                            ((x + o).clamp(0, w - 1), y)
                        } else {
                            (x, (y + o).clamp(0, h - 1))
                        };
                        s += kv * src[((sy * w + sx) * 3 + c) as usize];
                    }
                    out[((y * w + x) * 3 + c) as usize] = s / norm;
                }
            }
        }
        out
    };
    let src: Vec<f64> = rgb.iter().map(|&v| f64::from(v)).collect();
    pass(&pass(&src, true), false)
        .iter()
        .map(|v| v.round() as u8)
        .collect()
}

#[test]
fn colors_match_rendered_walls() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    let stats = colorize(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
    )
    .unwrap();
    assert_eq!(stats.images_used, 22, "{stats:?}");
    assert_eq!(
        stats.images_bad + stats.images_blurry + stats.images_outside_trajectory,
        0
    );
    let m = color_match(painted, &cloud);
    println!("{stats:?}: {m:?}");
    assert_eq!(m.colored, stats.points_colored);
    assert!(
        m.colored as f64 > 0.5 * cloud.len() as f64,
        "{m:?} of {}",
        cloud.len()
    );
    m.assert_good();
}

#[test]
fn occluded_points_take_no_color_from_occluder() {
    // A box 0.44 m in front of the left camera (standing still); the strip of the +y wall
    // behind it is never visible. Its points (5 cm apart) are 2–3 depth-buffer cells apart
    // in the image, so the box only hides the wall if each point covers its footprint.
    let painted = Painted::new(vec![(aabb([-0.5, 0.5, -1.2], [0.5, 0.9, 0.6]), BOX)]);
    let still = Shifted(Walk {
        still: 100.0,
        speed: 0.0,
    });
    let cams = cameras();
    let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
    let images = write_bag("occluded", &painted, &still, &cams, &shots);
    let poses = poses(&still, 3.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_used, 2);

    let cam = still.pose(T0 + 1.0) * cams[0].cam_in_body;
    let o = cam.translation.vector;
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    let (mut hidden, mut wrong) = (0, 0);
    for i in 0..cloud.len() {
        let p = Vector3::from(cloud.xyz[i].map(f64::from));
        if p.y < 4.99 {
            continue; // only the +y wall
        }
        let to = p - o;
        let in_view = (cam.inverse() * driftless_traj::nalgebra::Point3::from(p))
            .coords
            .normalize()
            .z
            > 0.5;
        if in_view && painted.scene.raycast(&o, &to.normalize()) < to.norm() - 0.05 {
            hidden += 1;
            if colored[i] && close(rgb[i], BOX, 40) {
                wrong += 1;
            }
        }
    }
    println!("{stats:?}: {wrong} of {hidden} hidden wall points in box colour");
    assert!(hidden > 200, "{hidden}");
    assert_eq!(
        wrong, 0,
        "{wrong} of {hidden} hidden wall points in box colour"
    );
    // The box and the visible wall are coloured right.
    color_match(&painted, &cloud).assert_good();
}

/// Inverted colours: images that must not be used.
fn invert(rgb: Vec<u8>) -> Vec<u8> {
    rgb.into_iter().map(|v| 255 - v).collect()
}

/// Brief, fast turn about z (3 rad/s for 0.3 s) on top of the walk.
struct WalkWithTurn(Shifted<Walk>);

const TURN: (f64, f64) = (T0 + 3.1, T0 + 3.4);

impl Trajectory for WalkWithTurn {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        let yaw = 3.0 * (t.clamp(TURN.0, TURN.1) - TURN.0);
        self.0.pose(t) * UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
    }
}

#[test]
fn blurry_and_fast_images_are_skipped() {
    let painted = walk_scene();
    let cams = cameras();
    let traj = WalkWithTurn(walk());
    let mut shots: Vec<Shot> = (1..=11).map(|i| shot(0, T0 + 0.5 * f64::from(i))).collect();
    shots.push(Shot {
        camera: 0,
        stamp: T0 + 2.25,
        edit: Box::new(|rgb| blur(&invert(rgb), 6.0)),
    });
    shots.push(Shot {
        camera: 0,
        stamp: T0 + 3.22,
        edit: Box::new(invert),
    });
    let images = write_bag("blurry", &painted, &traj, &cams, &shots);
    let poses = poses(&traj, 6.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_blurry, 2, "{stats:?}");
    assert_eq!(stats.images_used, 11, "{stats:?}");
    // Had the inverted images been used, their colours would show.
    color_match(&painted, &cloud).assert_good();
}

#[test]
fn images_outside_trajectory_skipped() {
    let painted = walk_scene();
    let cams = cameras();
    let traj = walk();
    // The trajectory covers T0 … T0 + 6; with the +0.03 s offset an image stamped
    // T0 − 0.02 is taken at T0 + 0.01 (inside), T0 − 0.04 at T0 − 0.01 (outside), and
    // T0 + 5.96 at T0 + 5.99 (inside).
    let shots = [
        shot(0, T0 - 1.0),
        shot(0, T0 - 0.04),
        shot(0, T0 - 0.02),
        shot(1, T0 + 3.0),
        shot(1, T0 + 5.96),
        shot(1, T0 + 7.0),
    ];
    let images = write_bag("outside", &painted, &traj, &cams, &shots);
    let poses = poses(&traj, 6.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_outside_trajectory, 3, "{stats:?}");
    assert_eq!(stats.images_used, 3, "{stats:?}");
    assert!(stats.points_colored > 0);
}

#[test]
fn uncolored_points_are_gray_not_black() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    // Points far outside the room, beyond max_dist of every camera.
    let n0 = cloud.len();
    for i in 0..50 {
        cloud.xyz.push([50.0 + i as f32 * 0.1, 0.0, 0.0]);
        cloud.intensity.push(i as f32);
        cloud.t.push(T0);
        cloud.frame.push(0);
    }
    let stats = colorize(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
    )
    .unwrap();
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    assert_eq!(stats.points_colored + stats.points_uncolored, cloud.len());
    for i in n0..cloud.len() {
        assert!(!colored[i]);
        let [r, g, b] = rgb[i];
        assert!(r == g && g == b && r >= 64, "{:?}", rgb[i]);
    }
    for i in (0..cloud.len()).filter(|&i| !colored[i]) {
        assert_ne!(rgb[i], [0, 0, 0]);
    }
    // Without images every point is gray, brighter with higher intensity.
    let empty = ImageSource {
        bags: vec![],
        topics: images.topics.clone(),
    };
    let stats = colorize(&mut cloud, empty, poses, cams, &ColorConfig::default()).unwrap();
    assert_eq!(
        stats,
        ColorStats {
            points_uncolored: cloud.len(),
            gain_min: 1.0,
            gain_max: 1.0,
            vignetting: Vignetting::flat(cams.len(), ExposureConfig::default().max_gain),
            ..ColorStats::default()
        }
    );
    let rgb = cloud.rgb.as_ref().unwrap();
    assert!(cloud.colored.as_ref().unwrap().iter().all(|&c| !c));
    assert!(rgb[n0 + 10][0] < rgb[n0 + 40][0]);
    assert!(rgb
        .iter()
        .all(|c| c[0] == c[1] && c[1] == c[2] && c[0] >= 64));
}

#[test]
fn colorize_is_deterministic() {
    let (painted, images, poses, cams) = walk_data();
    let run = || {
        let mut cloud = surface_cloud(painted);
        let stats = colorize(
            &mut cloud,
            images.clone(),
            poses,
            cams,
            &ColorConfig::default(),
        )
        .unwrap();
        (cloud, stats)
    };
    let (a, sa) = run();
    let (b, sb) = run();
    assert_eq!(sa, sb);
    assert_eq!(a.rgb, b.rgb);
    assert_eq!(a.colored, b.colored);
}

#[test]
fn colorize_rejects_bad_config_and_cameras() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    let bad = [
        ColorConfig {
            downscale: 3,
            ..ColorConfig::default()
        },
        ColorConfig {
            views_kept: 0,
            ..ColorConfig::default()
        },
        ColorConfig {
            max_dist: 0.0,
            ..ColorConfig::default()
        },
        ColorConfig {
            zbuffer_scale: 0,
            ..ColorConfig::default()
        },
        ColorConfig {
            max_angular_rate: f64::NAN,
            ..ColorConfig::default()
        },
        ColorConfig {
            highlight_ratio: -0.1,
            ..ColorConfig::default()
        },
        ColorConfig {
            highlight_abs: f64::NAN,
            ..ColorConfig::default()
        },
        ColorConfig {
            rim_theta: 0.0,
            ..ColorConfig::default()
        },
        ColorConfig {
            view_gap: -1.0,
            ..ColorConfig::default()
        },
    ];
    for cfg in &bad {
        let r = colorize(&mut cloud, images.clone(), poses, cams, cfg);
        assert!(matches!(r, Err(MapError::Config(_))), "{cfg:?}");
    }
    let twice = [cams[0].clone(), cams[0].clone()];
    let r = colorize(
        &mut cloud,
        images.clone(),
        poses,
        &twice,
        &ColorConfig::default(),
    );
    assert!(matches!(r, Err(MapError::Config(_))));
    let missing = ImageSource {
        bags: vec![temp_dir("missing").join("nope.bag")],
        topics: images.topics.clone(),
    };
    let r = colorize(&mut cloud, missing, poses, cams, &ColorConfig::default());
    assert!(matches!(r, Err(MapError::Images(_))));
}

/// The walk with a highlight in every second image of each camera.
fn highlight_data() -> &'static (
    Painted,
    ImageSource,
    Vec<driftless_traj::Pose>,
    Vec<ColorCamera>,
) {
    static DATA: std::sync::OnceLock<(
        Painted,
        ImageSource,
        Vec<driftless_traj::Pose>,
        Vec<ColorCamera>,
    )> = std::sync::OnceLock::new();
    DATA.get_or_init(|| {
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let shots: Vec<Shot> = (1..=11)
            .flat_map(|i| {
                [0, 1].map(|c| Shot {
                    camera: c,
                    stamp: T0 + 0.5 * f64::from(i),
                    edit: if i % 2 == 0 {
                        Box::new(with_highlight)
                    } else {
                        Box::new(|rgb| rgb)
                    },
                })
            })
            .collect();
        let images = write_bag("highlight", &painted, &traj, &cams, &shots);
        (painted, images, poses(&traj, 6.0), cams)
    })
}

/// Coloured floor points at least 10 cm from another colour (see [`Match`]) and the share of
/// them within [`TOL`] of the floor colour.
fn floor_match(painted: &Painted, cloud: &driftless_map::Cloud) -> (usize, f64) {
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    let floor: Vec<usize> = (0..cloud.len())
        .filter(|&i| {
            let p = Vector3::from(cloud.xyz[i].map(f64::from));
            colored[i] && on_floor(painted, &p) && !near_boundary(painted, &p, 0.1)
        })
        .collect();
    let good = floor.iter().filter(|&&i| close(rgb[i], FLOOR, TOL)).count();
    (floor.len(), good as f64 / floor.len().max(1) as f64)
}

#[test]
fn rejects_view_dependent_highlights() {
    let (painted, images, poses, cams) = highlight_data();
    let run = |cfg: &ColorConfig| {
        let mut cloud = surface_cloud(painted);
        let stats = colorize(&mut cloud, images.clone(), poses, cams, cfg).unwrap();
        (floor_match(painted, &cloud), stats)
    };
    // The rule is off by default (experimental).
    assert!(!ColorConfig::default().highlights);
    let ((n, on), stats) = run(&ColorConfig {
        highlights: true,
        ..ColorConfig::default()
    });
    // Without the rule and with neighbouring images kept (as in Meilenstein 3): the spots
    // show. (Views 1 s apart alone already avoid many of them here, since every second image
    // has one; real reflections are not that kind.)
    let ((n_off, off), stats_off) = run(&ColorConfig {
        highlights: false,
        view_gap: 0.0,
        ..ColorConfig::default()
    });
    println!("on: {n} floor points, {on:.3} good, {stats:?}");
    println!("off: {n_off} floor points, {off:.3} good");
    assert!(n > 1000, "{n}");
    assert!(on > 0.95, "{on}");
    assert!(off < 0.85, "{off}");
    assert!(
        stats.views_rejected > 0 && stats.points_with_rejected > 0,
        "{stats:?}"
    );
    assert_eq!(stats_off.views_rejected, 0);
    assert!(
        stats.floor_highlight_share_after < 0.5 * stats.floor_highlight_share_before,
        "{stats:?}"
    );
    // Without the rule the share stays as before: the blend alone keeps the spots.
    assert!(
        stats_off.floor_highlight_share_after > 0.5 * stats_off.floor_highlight_share_before,
        "{stats_off:?}"
    );
}

#[test]
fn few_views_are_never_all_rejected() {
    let painted = walk_scene();
    let cams = cameras();
    let traj = walk();
    let one = [Shot {
        camera: 0,
        stamp: T0 + 2.0,
        edit: Box::new(with_highlight),
    }];
    let two = [
        shot(0, T0 + 2.0),
        Shot {
            camera: 0,
            stamp: T0 + 2.1,
            edit: Box::new(with_highlight),
        },
    ];
    let poses = poses(&traj, 6.0);
    for (name, shots) in [("one-view", &one[..]), ("two-views", &two[..])] {
        let images = write_bag(name, &painted, &traj, &cams, shots);
        let mut colored = Vec::new();
        for highlights in [true, false] {
            let cfg = ColorConfig {
                highlights,
                ..ColorConfig::default()
            };
            let mut cloud = surface_cloud(&painted);
            let stats = colorize(&mut cloud, images.clone(), &poses, &cams, &cfg).unwrap();
            assert_eq!(stats.images_used, shots.len(), "{stats:?}");
            colored.push(cloud.colored.unwrap());
        }
        // Every point seen keeps a colour, however bright its views.
        assert_eq!(colored[0], colored[1], "{name}");
        assert!(colored[0].iter().filter(|&&c| c).count() > 1000, "{name}");
    }
}

/// The walk with every second stamp `gain` times brighter (linear), no exposure fit: the
/// difference stays and shows wherever a point's views change.
fn seam_stats(name: &str, gain: f64) -> ColorStats {
    let painted = walk_scene();
    let cams = cameras();
    let traj = walk();
    let shots: Vec<Shot> = (1..=11)
        .flat_map(|i| {
            let g = if i % 2 == 0 { gain } else { 1.0 };
            [0, 1].map(move |c| Shot {
                camera: c,
                stamp: T0 + 0.5 * f64::from(i),
                edit: Box::new(move |rgb: Vec<u8>| {
                    rgb.into_iter()
                        .map(|v| {
                            let l = (f64::from(v) / 255.0).powf(2.2) * g;
                            (l.min(1.0).powf(1.0 / 2.2) * 255.0).round() as u8
                        })
                        .collect()
                }),
            })
        })
        .collect();
    let images = write_bag(name, &painted, &traj, &cams, &shots);
    let poses = poses(&traj, 6.0);
    let cfg = ColorConfig {
        exposure: ExposureConfig {
            samples: 0,
            ..ExposureConfig::default()
        },
        ..ColorConfig::default()
    };
    let mut cloud = surface_cloud(&painted);
    colorize(&mut cloud, images, &poses, &cams, &cfg).unwrap()
}

#[test]
fn seam_jump_measures_exposure_seams() {
    // Exposure differences between the images show as jumps between neighbours with
    // different main images; with equal exposure there are none.
    let flat = seam_stats("seam-flat", 1.0);
    let grouped = seam_stats("seam-grouped", 1.2);
    println!(
        "seam jump: equal exposure {:.2} -> {:.2}, two groups {:.2} -> {:.2}",
        flat.seam_jump_before,
        flat.seam_jump_after,
        grouped.seam_jump_before,
        grouped.seam_jump_after
    );
    for s in [&flat, &grouped] {
        assert!(s.seam_jump_before.is_finite() && s.seam_jump_after.is_finite());
    }
    assert!(
        flat.seam_jump_before < 0.5 && flat.seam_jump_after < 0.5,
        "{flat:?}"
    );
    assert!(grouped.seam_jump_before > 1.0, "{grouped:?}");
    assert!(grouped.seam_jump_after > 1.0, "{grouped:?}");
}
