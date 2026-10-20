use driftless_odom::testutil as common;

use common::{imu_samples, room_scene, Spin, Trajectory, G};
use driftless_io::msg::{ImuSample, LidarFrame, LidarPoint};
use driftless_odom::deskew::{deskew, frame_points};
use driftless_odom::imu_integrate::{propagate, ImuModel, ImuState};
use driftless_traj::nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

fn still_model() -> ImuModel {
    ImuModel {
        gravity: Vector3::new(0.0, 0.0, -G),
        gyro_bias: Vector3::zeros(),
        accel_bias: Vector3::zeros(),
    }
}

#[test]
fn imu_integration_constant_rotation() {
    let imu: Vec<ImuSample> = (0..=200)
        .map(|i| ImuSample {
            stamp: i as f64 / 200.0,
            gyro: [0.0, 0.0, 1.0],
            accel: [0.0, 0.0, G],
        })
        .collect();
    let start = ImuState::at_rest(0.0, UnitQuaternion::identity());
    let states = propagate(&start, &imu, &still_model(), 1.0);
    let end = states.last().unwrap();
    assert_eq!(end.t, 1.0);
    let (_, _, yaw) = end.rot.euler_angles();
    assert!((yaw - 1.0).abs() < 1e-3, "yaw {yaw}");
    assert!(end.pos.norm() < 1e-9, "pos {}", end.pos);
    assert!(end.vel.norm() < 1e-9, "vel {}", end.vel);
}

#[test]
fn deskew_moves_points_by_motion() {
    // 2 rad/s yaw: 0.2 rad during the 0.1 s sweep.
    let s = 50.0;
    let spin = Spin {
        omega: Vector3::new(0.0, 0.0, 2.0),
        t0: s,
    };
    let scene = room_scene();
    let frame = common::lidar_frame(&scene, &spin, s, 360, &Isometry3::identity());
    let imu = imu_samples(&spin, s - 0.05, s + 0.15, 200.0);

    let pts = frame_points(&frame, &Isometry3::identity());
    assert_eq!(pts.points.len(), frame.points.len());
    let start = ImuState::at_rest(pts.t_start, spin.pose(pts.t_start).rotation);
    let motion = propagate(&start, &imu, &still_model(), pts.t_end);
    let out = deskew(&pts.points, &motion, pts.t_end);

    // Reference: the world points seen from the sensor pose at the frame end.
    let end_inv = spin.pose(pts.t_end).inverse();
    let (mut worst, mut worst_raw) = (0.0f64, 0.0f64);
    for (tp, d) in pts.points.iter().zip(&out) {
        let world = spin.pose(tp.t) * Point3::from(tp.p);
        let reference = end_inv * world;
        worst = worst.max((d - reference.coords).norm());
        worst_raw = worst_raw.max((tp.p - reference.coords).norm());
    }
    assert!(worst < 0.01, "deskewed error {worst} m");
    assert!(
        worst_raw > 0.3,
        "test motion too small: raw error {worst_raw} m"
    );
}

#[test]
fn absolute_point_times_are_normalised() {
    let p = |t: f64| LidarPoint {
        xyz: [1.0, 2.0, 3.0],
        intensity: 1.0,
        ring: 0,
        t_rel: t,
    };
    let frame = LidarFrame {
        stamp: 1000.0,
        points: vec![
            p(1000.0),
            p(1000.05),
            p(1000.1),
            p(f64::NAN),
            p(999.99), // before the frame stamp
        ],
        dropped: 0,
    };
    let pts = frame_points(&frame, &Isometry3::identity());
    let t: Vec<f64> = pts.points.iter().map(|x| x.t).collect();
    assert_eq!(t.len(), 4);
    assert!((t[0] - 1000.0).abs() < 1e-9);
    assert!((t[1] - 1000.05).abs() < 1e-9);
    assert!((t[2] - 1000.1).abs() < 1e-9);
    assert!(
        (t[3] - 1000.0).abs() < 1e-9,
        "negative offset clamped to stamp"
    );
    assert_eq!(pts.skipped, 1);
    assert_eq!(pts.clamped, 1);
    assert!((pts.t_start - 1000.0).abs() < 1e-9);
    assert!((pts.t_end - 1000.1).abs() < 1e-9);

    // Relative times stay relative.
    let rel = LidarFrame {
        stamp: 1000.0,
        points: vec![p(0.0), p(0.04), p(-0.001)],
        dropped: 0,
    };
    let pts = frame_points(&rel, &Isometry3::identity());
    let t: Vec<f64> = pts.points.iter().map(|x| x.t).collect();
    assert!((t[1] - 1000.04).abs() < 1e-9);
    assert!((t[2] - 1000.0).abs() < 1e-9);
    assert_eq!((pts.skipped, pts.clamped), (0, 1));
}

#[test]
fn frame_points_apply_lidar_to_imu() {
    let frame = LidarFrame {
        stamp: 5.0,
        points: vec![LidarPoint {
            xyz: [1.0, 0.0, 0.0],
            intensity: 1.0,
            ring: 0,
            t_rel: 0.0,
        }],
        dropped: 0,
    };
    let ext = Isometry3::new(
        Vector3::new(0.1, 0.2, 0.3),
        Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
    );
    let pts = frame_points(&frame, &ext);
    let q = pts.points[0].p;
    assert!((q - Vector3::new(0.1, 1.2, 0.3)).norm() < 1e-9, "{q}");
}
