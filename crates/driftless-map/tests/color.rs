//! Colouring from synthetic camera images: a room with coloured walls is rendered (ray cast
//! per pixel) from known camera poses, the images go through JPEG into a bag, and the
//! cloud's points on the surfaces must get the colour of their surface.

mod common;

use common::*;
use driftless_map::{colorize, ColorConfig, ColorStats, ImageSource, MapError};
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
            views_per_point: 0,
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
