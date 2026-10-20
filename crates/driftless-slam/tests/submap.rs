mod common;

use common::{body_points, frame, iso, walk_pose};
use driftless_odom::testutil::room_scene;
use driftless_odom::{FrameQuality, OUTPUT_MAP_VOXEL};
use driftless_slam::{Submap, SubmapBuilder, SubmapConfig};

fn build(n: usize, speed: f64) -> Vec<Submap> {
    let mut b = SubmapBuilder::new(SubmapConfig::default());
    for i in 0..n {
        b.push(frame(i, walk_pose(i, speed), FrameQuality::first(), &[]));
    }
    b.finish()
}

fn indices(maps: &[Submap]) -> Vec<usize> {
    maps.iter()
        .flat_map(|m| m.frames.iter().map(|f| f.index))
        .collect()
}

#[test]
fn splits_by_length() {
    // 12 m at 1 m/s, 10 Hz: 120 frames
    let maps = build(120, 1.0);
    assert_eq!(maps.len(), 3);
    assert_eq!(indices(&maps), (0..120).collect::<Vec<_>>());
    for (k, m) in maps.iter().enumerate() {
        assert_eq!(m.id, k);
    }
    assert_eq!(maps[0].frames.len(), 51);
    let first = &maps[0].frames[0].offset;
    assert!(first.translation.vector.norm() < 1e-12 && first.rotation.angle() < 1e-12);
    assert!((maps[1].anchor.translation.x - 5.1).abs() < 1e-9);
}

#[test]
fn splits_by_frame_count() {
    // standing still for 15 s
    let maps = build(150, 0.0);
    assert_eq!(maps.len(), 150usize.div_ceil(100));
    assert_eq!(maps[0].frames.len(), 100);
    assert_eq!(maps[1].frames.len(), 50);
    assert_eq!(indices(&maps), (0..150).collect::<Vec<_>>());
}

#[test]
fn points_round_trip() {
    let scene = room_scene();
    let mut b = SubmapBuilder::new(SubmapConfig::default());
    let mut all = Vec::new();
    for i in 0..10 {
        let pose = walk_pose(i, 0.5);
        let pts = body_points(&scene, &pose, 4.0);
        b.push(frame(i, pose.clone(), FrameQuality::first(), &pts));
        all.push((pose, pts));
    }
    let maps = b.finish();
    assert_eq!(maps.len(), 1);
    let m = &maps[0];
    assert!(!m.points.is_empty());
    let world: Vec<[f32; 3]> = m.world_points(&m.anchor).collect();
    assert_eq!(world.len(), m.points.len());

    // every stored point lies within a voxel of some frame-pose-projected body point,
    // and the voxel grid thins the cloud
    let projected: Vec<_> = all
        .iter()
        .flat_map(|(pose, pts)| {
            let t = iso(pose);
            pts.iter()
                .map(move |p| t * driftless_traj::nalgebra::Point3::from(*p))
        })
        .collect();
    assert!(world.len() < projected.len());
    for w in &world {
        let w = driftless_traj::nalgebra::Point3::new(
            f64::from(w[0]),
            f64::from(w[1]),
            f64::from(w[2]),
        );
        let d = projected
            .iter()
            .map(|q| (q - w).norm())
            .fold(f64::INFINITY, f64::min);
        assert!(
            d < OUTPUT_MAP_VOXEL,
            "stored point {d} m from any source point"
        );
    }

    // moving the anchor moves the world points rigidly
    let shift = driftless_traj::nalgebra::Isometry3::translation(1.0, 2.0, 0.0) * m.anchor;
    for (a, b) in world.iter().zip(m.world_points(&shift)) {
        assert!((b[0] - a[0] - 1.0).abs() < 1e-4 && (b[1] - a[1] - 2.0).abs() < 1e-4);
    }
}

#[test]
fn quality_is_minimum() {
    let mut b = SubmapBuilder::new(SubmapConfig::default());
    let bad = FrameQuality {
        residual_rms: 0.05,
        ..FrameQuality::first()
    };
    for i in 0..5 {
        let q = if i == 2 { bad } else { FrameQuality::first() };
        b.push(frame(i, walk_pose(i, 0.5), q, &[]));
    }
    let maps = b.finish();
    assert_eq!(maps[0].quality(), bad.score());
    assert!(bad.score() < FrameQuality::first().score());
}
