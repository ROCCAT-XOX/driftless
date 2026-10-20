mod common;

use common::{corridor_scene, imu_samples, lidar_frames, Trajectory, Walk};
use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_io::testdata;
use driftless_odom::{run_odometry, OdomConfig, OdomResult};
use driftless_traj::nalgebra::{Isometry3, Vector3};
use driftless_traj::{ate_rmse, path_length, read_tum, start_end_gap};
use std::time::Instant;

fn corridor_run(duration: f64, lidar_to_imu: Isometry3<f64>) -> (Walk, OdomResult) {
    let walk = Walk {
        still: 1.0,
        speed: 1.0,
    };
    let scene = corridor_scene();
    let frames = lidar_frames(&scene, &walk, 0.0, duration, 360, &lidar_to_imu);
    let imu = imu_samples(&walk, -0.5, duration + 0.5, 200.0);
    let cfg = OdomConfig {
        lidar_to_imu,
        ..OdomConfig::default()
    };
    let r = run_odometry(frames.into_iter(), &imu, &cfg);
    (walk, r)
}

#[test]
fn pipeline_straight_corridor() {
    let (walk, r) = corridor_run(26.0, Isometry3::identity());
    assert_eq!(r.poses.len(), 260);
    let last = r.poses.last().unwrap();
    let truth = walk.pose(last.t).translation.vector;
    assert!(truth.x > 23.0, "test walks {} m", truth.x);
    let dev = (last.p - truth).norm();
    assert!(
        dev < 0.3,
        "end deviation {dev} m (est {}, truth {truth})",
        last.p
    );
    assert!(!r.map_points.is_empty());
}

/// Gyro bias appearing after the static start: only ICP can correct the yaw drift.
#[test]
fn pipeline_corrects_gyro_drift_with_icp() {
    let walk = Walk {
        still: 1.0,
        speed: 1.0,
    };
    let scene = corridor_scene();
    let frames = lidar_frames(&scene, &walk, 0.0, 26.0, 360, &Isometry3::identity());
    let mut imu = imu_samples(&walk, -0.5, 26.5, 200.0);
    for s in imu.iter_mut().filter(|s| s.stamp > 2.0) {
        s.gyro[2] += 0.02; // 1.15°/s
    }
    let r = run_odometry(frames.into_iter(), &imu, &OdomConfig::default());
    let last = r.poses.last().unwrap();
    let truth = walk.pose(last.t);
    let dev = (last.p - truth.translation.vector).norm();
    let yaw = (last.q.inverse() * truth.rotation).angle().to_degrees();
    assert!(dev < 0.3, "end deviation {dev} m");
    assert!(yaw < 0.5, "end rotation error {yaw}°");
}

#[test]
fn pipeline_handles_lidar_to_imu_extrinsic() {
    let ext = Isometry3::new(
        Vector3::new(0.05, -0.03, 0.1),
        Vector3::new(0.0, 0.0, std::f64::consts::PI),
    );
    let (walk, r) = corridor_run(6.0, ext);
    let last = r.poses.last().unwrap();
    let dev = (last.p - walk.pose(last.t).translation.vector).norm();
    assert!(dev < 0.1, "end deviation {dev} m");
}

#[test]
fn odometry_is_deterministic() {
    let (_, a) = corridor_run(4.0, Isometry3::identity());
    let (_, b) = corridor_run(4.0, Isometry3::identity());
    assert_eq!(a.poses, b.poses);
    assert_eq!(a.map_points, b.map_points);
}

#[test]
fn pipeline_survives_disordered_input() {
    let walk = Walk {
        still: 1.0,
        speed: 1.0,
    };
    let scene = corridor_scene();
    let mut frames = lidar_frames(&scene, &walk, 0.0, 3.0, 180, &Isometry3::identity());
    frames.swap(10, 11); // one frame out of order
    frames.insert(5, frames[4].clone()); // duplicate frame
    frames.push(LidarFrame {
        stamp: 99.0,
        points: vec![],
        dropped: 0,
    });
    let mut imu = imu_samples(&walk, -0.5, 3.5, 200.0);
    imu.swap(100, 101);
    let dup = imu[200].clone();
    imu.insert(200, dup);
    imu.drain(300..310); // 50 ms gap
    let r = run_odometry(frames.into_iter(), &imu, &OdomConfig::default());
    assert!(r.poses.windows(2).all(|w| w[0].t < w[1].t));
    assert_eq!(r.stats.frames_skipped, 3);
    assert_eq!(r.stats.imu.duplicates, 1);
    assert!(r.stats.imu.reordered);
    assert!(r.stats.static_start);
    assert!(r.poses.iter().all(|p| p.p.iter().all(|v| v.is_finite())));
}

/// Real recording: `$DRIFTLESS_TESTDATA/Test` (JMK6, 266 s). Reports runtime and accuracy.
#[test]
#[ignore]
fn real_test_runs_under_10_minutes() {
    let root = testdata::path("Test").expect("DRIFTLESS_TESTDATA not set");
    let project = discover_project(&root).unwrap();
    let start = Instant::now();
    let mut imu: Vec<ImuSample> = Vec::new();
    for bag in &project.lidar_bags {
        let mut r = BagReader::open(bag).unwrap();
        for m in r.messages(&["/vanjee_imu_packets"]).map_while(Result::ok) {
            imu.extend(decode_imu(&m.data).ok());
        }
    }
    let bags = project.lidar_bags.clone();
    let (tx, rx) = std::sync::mpsc::sync_channel::<LidarFrame>(8);
    let reader = std::thread::spawn(move || {
        for bag in &bags {
            let mut r = BagReader::open(bag).unwrap();
            for m in r.messages(&["/vanjee_722z"]).map_while(Result::ok) {
                if let Ok(f) = decode_pointcloud2(&m.data) {
                    if tx.send(f).is_err() {
                        return;
                    }
                }
            }
        }
    });
    let r = run_odometry(rx.into_iter(), &imu, &OdomConfig::default());
    reader.join().unwrap();
    let secs = start.elapsed().as_secs_f64();

    let out = std::env::temp_dir().join("driftless-odom-real.tum");
    driftless_traj::write_tum(&out, &r.poses).unwrap();
    println!(
        "runtime {secs:.1} s, {} poses, {} map points",
        r.poses.len(),
        r.map_points.len()
    );
    println!("stats {:?}", r.stats);
    println!(
        "start-end gap {:.3} m, path length {:.1} m, trajectory {}",
        start_end_gap(&r.poses),
        path_length(&r.poses),
        out.display()
    );
    let rs = read_tum(&root.join("shading/trj.txt")).unwrap();
    println!(
        "ATE vs RayStudio trj.txt: {:.3} m",
        ate_rmse(&r.poses, &rs, 0.01).unwrap()
    );
    let glim = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../spikes/glim/out/glim_traj.tum");
    if let Ok(g) = read_tum(&glim) {
        println!(
            "ATE vs GLIM: {:.3} m",
            ate_rmse(&r.poses, &g, 0.01).unwrap()
        );
    }
    assert!(secs < 600.0, "runtime {secs} s");
}
