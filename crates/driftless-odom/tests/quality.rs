use driftless_odom::testutil as common;

use common::{aabb, imu_samples, lidar_frames, room_scene, Scene, Trajectory, Walk};
use driftless_odom::{run_odometry, run_odometry_with, FrameOutput, OdomConfig, OdomResult};
use driftless_traj::nalgebra::{Isometry3, Translation3, Vector3};
use driftless_traj::Pose;

/// [`Walk`] starting at `start` instead of the world origin.
struct WalkFrom {
    walk: Walk,
    start: Vector3<f64>,
}

impl Trajectory for WalkFrom {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        Translation3::from(self.start) * self.walk.pose(t)
    }
}

fn walk_run(scene: &Scene, start: [f64; 3], duration: f64, speed: f64) -> OdomResult {
    let walk = WalkFrom {
        walk: Walk { still: 1.0, speed },
        start: Vector3::from(start),
    };
    let frames = lidar_frames(scene, &walk, 0.0, duration, 360, &Isometry3::identity());
    let imu = imu_samples(&walk, -0.5, duration + 0.5, 200.0);
    run_odometry(frames.into_iter(), &imu, &OdomConfig::default())
}

fn room_run(duration: f64) -> OdomResult {
    walk_run(&room_scene(), [0.0; 3], duration, 0.8)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Median degeneracy and score over the frames after the first.
fn medians(r: &OdomResult) -> (f64, f64) {
    let q = &r.quality[1..];
    (
        median(q.iter().map(|q| q.degeneracy).collect()),
        median(q.iter().map(|q| f64::from(q.score())).collect()),
    )
}

/// Corridor without pillars: only the end walls fix the position along it. Both stay within
/// LiDAR range everywhere, so the walk starts in the middle where they are farthest away.
fn featureless_corridor() -> Scene {
    Scene {
        room: aabb([-3.0, -1.6, -1.3], [60.0, 1.6, 1.7]),
        boxes: vec![],
    }
}

#[test]
fn quality_room_is_well_constrained() {
    let r = room_run(6.0);
    let (deg, score) = medians(&r);
    println!("room: median degeneracy {deg:.4}, median score {score:.3}");
    assert!(deg > 0.05, "median degeneracy {deg}");
    assert!(score > 0.8, "median score {score}");
}

#[test]
fn quality_flags_featureless_corridor() {
    let room = room_run(6.0);
    let (room_deg, _) = medians(&room);
    let r = walk_run(&featureless_corridor(), [24.0, 0.0, 0.0], 12.0, 1.0);
    let (deg, score) = medians(&r);
    println!("corridor: median degeneracy {deg:.4}, median score {score:.3}");
    assert!(
        deg < 0.2 * room_deg,
        "median degeneracy {deg} vs room {room_deg}"
    );
    assert!(score < 0.5, "median score {score}");
}

#[test]
fn quality_len_matches_poses() {
    let r = room_run(3.0);
    assert!(!r.poses.is_empty());
    assert_eq!(r.quality.len(), r.poses.len());
    assert_eq!(r.quality[0].score(), 1.0);
    assert!(r.quality.iter().all(|q| (0.0..=1.0).contains(&q.score())));
}

#[test]
fn sink_sees_every_used_frame() {
    let walk = Walk {
        still: 1.0,
        speed: 0.8,
    };
    let frames = lidar_frames(&room_scene(), &walk, 0.0, 3.0, 360, &Isometry3::identity());
    let imu = imu_samples(&walk, -0.5, 3.5, 200.0);
    let mut seen: Vec<(usize, Pose, usize)> = Vec::new();
    let mut sink = |f: FrameOutput| {
        assert_eq!(
            f.intensity.len(),
            f.body_points.len(),
            "one intensity per point"
        );
        seen.push((f.index, f.pose.clone(), f.body_points.len()));
    };
    let r = run_odometry_with(frames.into_iter(), &imu, &OdomConfig::default(), &mut sink);
    assert_eq!(seen.len(), r.poses.len());
    assert!(seen.iter().enumerate().all(|(i, s)| s.0 == i));
    let poses: Vec<Pose> = seen.iter().map(|s| s.1.clone()).collect();
    assert_eq!(poses, r.poses);
    assert!(seen.iter().all(|s| s.2 > 0));
}
