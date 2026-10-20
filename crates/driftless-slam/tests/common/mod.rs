//! Synthetic odometry frames for the submap tests.
#![allow(dead_code)] // each test file uses a different subset

use driftless_odom::testutil::Scene;
use driftless_odom::{FrameOutput, FrameQuality};
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use driftless_traj::Pose;

pub const RATE: f64 = 10.0;

/// Pose at frame `i` of a straight walk along +x at `speed` m/s (still if 0).
pub fn walk_pose(i: usize, speed: f64) -> Pose {
    let t = i as f64 / RATE;
    Pose {
        t,
        p: Vector3::new(speed * t, 0.0, 0.0),
        q: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
    }
}

pub fn iso(p: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::from(p.p), p.q)
}

/// Scene surface points within `range` of the pose, seen from the body frame.
pub fn body_points(scene: &Scene, pose: &Pose, range: f64) -> Vec<Vector3<f64>> {
    let inv = iso(pose).inverse();
    scene
        .surface_samples(0.2, 0.0)
        .into_iter()
        .filter(|w| (w - pose.p).norm() < range)
        .map(|w| inv * driftless_traj::nalgebra::Point3::from(w))
        .map(|p| p.coords)
        .collect()
}

pub fn frame<'a>(
    index: usize,
    pose: Pose,
    quality: FrameQuality,
    pts: &'a [Vector3<f64>],
) -> FrameOutput<'a> {
    FrameOutput {
        index,
        pose,
        quality,
        body_points: pts,
        // The submaps ignore intensity; leaked so the slice lives as long as `pts`.
        intensity: Vec::leak(vec![0.0; pts.len()]),
    }
}
