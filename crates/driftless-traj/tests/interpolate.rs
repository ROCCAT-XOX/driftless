use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
use driftless_traj::{interpolate_pose, Pose};

fn pose(t: f64, x: f64, yaw: f64) -> Pose {
    Pose {
        t,
        p: Vector3::new(x, 2.0 * x, 0.0),
        q: UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
    }
}

#[test]
fn interpolate_pose_midpoint_and_bounds() {
    let poses = [
        pose(1.0, 0.0, 0.0),
        pose(2.0, 1.0, 0.4),
        pose(3.0, 3.0, 1.0),
    ];

    let mid = interpolate_pose(&poses, 1.25).unwrap();
    assert!((mid.translation.vector - Vector3::new(0.25, 0.5, 0.0)).norm() < 1e-12);
    assert!((mid.rotation.angle() - 0.1).abs() < 1e-12);

    let mid = interpolate_pose(&poses, 2.5).unwrap();
    assert!((mid.translation.vector - Vector3::new(2.0, 4.0, 0.0)).norm() < 1e-12);
    assert!((mid.rotation.euler_angles().2 - 0.7).abs() < 1e-12);

    // The ends themselves are inside, anything beyond is not.
    let first = interpolate_pose(&poses, 1.0).unwrap();
    assert!(first.translation.vector.norm() < 1e-12);
    let last = interpolate_pose(&poses, 3.0).unwrap();
    assert!((last.translation.vector - Vector3::new(3.0, 6.0, 0.0)).norm() < 1e-12);
    assert!(interpolate_pose(&poses, 0.999).is_none());
    assert!(interpolate_pose(&poses, 3.001).is_none());
    assert!(interpolate_pose(&poses, f64::NAN).is_none());
    assert!(interpolate_pose(&[], 1.0).is_none());
    assert!(interpolate_pose(&poses[..1], 1.0).is_some());
}
