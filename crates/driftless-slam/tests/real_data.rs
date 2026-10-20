//! SLAM on real scans in `$DRIFTLESS_TESTDATA` (see `docs/testdaten.md`), odometry against
//! loop-closed trajectory. The bounds are the measured values of
//! `docs/ergebnisse/2026-10-20-meilenstein-2.md` with some headroom. Every test skips when its
//! data is missing.
//!
//! `cargo test --release -p driftless-slam -- --ignored --nocapture`

use driftless_io::bag::BagReader;
use driftless_io::msg::{decode_imu, decode_pointcloud2, ImuSample, LidarFrame};
use driftless_io::project::discover_project;
use driftless_io::testdata;
use driftless_odom::OdomConfig;
use driftless_slam::{
    build_submaps, read_cache, save_cache, solve, LoopVerdict, SlamConfig, SlamResult,
};
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use driftless_traj::{align_umeyama, ate_rmse, read_tum, start_end_gap, write_tum, Pose};
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Time matching of the ATE (s), as in the milestone 1 measurements.
const MAX_DT: f64 = 0.05;
/// LiDAR pose in the IMU frame of the Hilti 2022 rig, exactly as passed to `--lidar-to-imu` in
/// `docs/testdaten.md` (so the run matches the command line).
#[allow(clippy::approx_constant)]
const HILTI_LIDAR_TO_IMU: [f64; 7] = [-0.001, -0.00855, 0.055, 0.7071068, -0.7071068, 0.0, 0.0];

/// `$DRIFTLESS_TESTDATA/<rel>` if it exists; otherwise prints why the test is skipped.
fn data(rel: &str) -> Option<PathBuf> {
    let Some(p) = testdata::path(rel) else {
        eprintln!("SKIPPED: {} not set", testdata::ENV_VAR);
        return None;
    };
    if !p.exists() {
        eprintln!("SKIPPED: {} missing", p.display());
        return None;
    }
    Some(p)
}

fn hilti_extrinsic() -> Isometry3<f64> {
    let [x, y, z, qx, qy, qz, qw] = HILTI_LIDAR_TO_IMU;
    Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_quaternion(driftless_traj::nalgebra::Quaternion::new(qw, qx, qy, qz)),
    )
}

/// Topic of the first connection of `msg_type` in the bag.
fn topic(bag: &Path, msg_type: &str) -> String {
    let r = BagReader::open(bag).unwrap();
    r.connections()
        .iter()
        .find(|c| c.msg_type == msg_type)
        .unwrap_or_else(|| panic!("{}: no {msg_type}", bag.display()))
        .topic
        .clone()
}

struct Run {
    result: SlamResult,
    secs: f64,
}

/// The whole SLAM of the scan folder (odometry, submaps, automatic loops), like
/// `driftless slam`: IMU read first, LiDAR frames streamed from a reader thread.
fn slam(folder: &Path, lidar_to_imu: Isometry3<f64>) -> Run {
    let start = Instant::now();
    let cfg = SlamConfig {
        odom: OdomConfig {
            lidar_to_imu,
            ..OdomConfig::default()
        },
        ..SlamConfig::default()
    };
    let bags = discover_project(folder).unwrap().lidar_bags;
    let mut imu: Vec<ImuSample> = Vec::new();
    for bag in &bags {
        let t = topic(bag, "sensor_msgs/Imu");
        let mut r = BagReader::open(bag).unwrap();
        for m in r.messages(&[t.as_str()]).map_while(Result::ok) {
            imu.extend(decode_imu(&m.data).ok());
        }
    }
    let (tx, rx) = std::sync::mpsc::sync_channel::<LidarFrame>(16);
    let reader_bags = bags.clone();
    let reader = std::thread::spawn(move || {
        for bag in &reader_bags {
            let t = topic(bag, "sensor_msgs/PointCloud2");
            let mut r = BagReader::open(bag).unwrap();
            for m in r.messages(&[t.as_str()]).map_while(Result::ok) {
                if let Ok(f) = decode_pointcloud2(&m.data) {
                    if tx.send(f).is_err() {
                        return;
                    }
                }
            }
        }
    });
    let (submaps, _) = build_submaps(rx.into_iter(), &imu, &cfg);
    reader.join().unwrap();
    let odom_anchors = submaps.iter().map(|m| m.anchor).collect();
    let result = solve(submaps, odom_anchors, &[], &cfg).unwrap();
    Run {
        result,
        secs: start.elapsed().as_secs_f64(),
    }
}

/// Prints every loop check and the start-end gap; returns (accepted, rejected).
fn summary(name: &str, run: &Run) -> (usize, usize) {
    let r = &run.result;
    for l in &r.loops {
        println!(
            "{name}: loop {}↔{} inlier {:.3} rms {:.4} degeneracy {:.3} {:?}",
            l.a, l.b, l.inlier_ratio, l.rms, l.degeneracy, l.verdict
        );
    }
    let accepted = r
        .loops
        .iter()
        .filter(|l| l.verdict == LoopVerdict::Accepted)
        .count();
    println!(
        "{name}: {:.1} s, {} submaps, loops {accepted} accepted / {} rejected, \
         start-end {:.3} -> {:.3} m",
        run.secs,
        r.submaps.len(),
        r.loops.len() - accepted,
        start_end_gap(&r.odom_poses()),
        start_end_gap(&r.poses()),
    );
    (accepted, r.loops.len() - accepted)
}

/// ATE of the odometry and of the loop-closed trajectory against `reference`.
fn ate(name: &str, run: &Run, reference: &[Pose]) -> (f64, f64) {
    let odom = ate_rmse(&run.result.odom_poses(), reference, MAX_DT).unwrap();
    let slam = ate_rmse(&run.result.poses(), reference, MAX_DT).unwrap();
    println!("{name}: ATE odometry {odom:.4} m, SLAM {slam:.4} m");
    (odom, slam)
}

fn same_poses(a: &[Pose], b: &[Pose]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| {
            x.t.to_bits() == y.t.to_bits()
                && x.p
                    .iter()
                    .zip(y.p.iter())
                    .all(|(u, v)| u.to_bits() == v.to_bits())
                && x.q
                    .coords
                    .iter()
                    .zip(y.q.coords.iter())
                    .all(|(u, v)| u.to_bits() == v.to_bits())
        })
}

/// Hilti 2022 exp14 (no loop): the SLAM must not spoil the odometry.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn exp14_slam_not_worse() {
    let (Some(folder), Some(gt)) = (
        data("hilti2022/projekt"),
        data("hilti2022/exp14_basement_2_imu.txt"),
    ) else {
        return;
    };
    let run = slam(&folder, hilti_extrinsic());
    summary("exp14", &run);
    let (odom, slam) = ate("exp14", &run, &read_tum(&gt).unwrap());
    assert!(slam <= odom * 1.05, "ATE SLAM {slam} vs odometry {odom}");
    assert!(slam <= 0.074 * 1.2, "ATE SLAM {slam}");
}

/// Hilti 2022 exp04, a round trip that starts and ends on the same control point. The
/// reference is 7 control points (position only); the metric is the error at them after
/// rigid alignment. The odometry stays within a few centimetres here, so the loop closure
/// can only keep that (see the results document): SLAM must not be worse than the odometry.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn loop_sequence_slam_not_worse() {
    let (Some(folder), Some(gt)) = (
        data("hilti2022/projekt-exp04"),
        data("hilti2022/exp04_construction_upper_level.txt"),
    ) else {
        return;
    };
    let gt = read_tum(&gt).unwrap();
    assert_eq!(gt.len(), 7, "control points");
    let run = slam(&folder, hilti_extrinsic());
    summary("exp04", &run);
    let (odom, slam) = ate("exp04", &run, &gt);
    for (name, poses) in [
        ("odometry", run.result.odom_poses()),
        ("SLAM", run.result.poses()),
    ] {
        let t = align_umeyama(&poses, &gt, MAX_DT).unwrap();
        let at = |c: &Pose| {
            let p = poses
                .iter()
                .min_by(|a, b| (a.t - c.t).abs().total_cmp(&(b.t - c.t).abs()))
                .unwrap();
            t * Point3::from(p.p)
        };
        let errors: Vec<String> = gt
            .iter()
            .map(|c| format!("{:.3}", (at(c).coords - c.p).norm()))
            .collect();
        // First and last control point are the same place.
        let gap = (at(&gt[6]) - at(&gt[0])).norm();
        println!(
            "exp04 {name}: control point errors {} m, first↔last control point {gap:.3} m",
            errors.join(" ")
        );
    }
    assert!(slam <= odom * 1.05, "ATE SLAM {slam} vs odometry {odom}");
    assert!(slam <= 0.0553 * 1.2, "ATE SLAM {slam}");
}

/// JMK6 `Test` (266 s) against RayStudio `trj.txt` (a comparison, not ground truth).
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_slam_not_worse_than_odom() {
    let (Some(folder), Some(rs)) = (data("Test"), data("Test/shading/trj.txt")) else {
        return;
    };
    let run = slam(&folder, Isometry3::identity());
    summary("Test", &run);
    let (odom, slam) = ate("Test", &run, &read_tum(&rs).unwrap());
    assert!(slam <= odom * 1.05, "ATE SLAM {slam} vs odometry {odom}");
    assert!(slam <= 0.0183 * 1.2, "ATE SLAM {slam}");
}

/// `--reuse` on JMK6 `Test`: load the cache, solve the graph, build the outputs in well under
/// 10 s, with the same trajectory as the full run.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_reuse_under_10_s() {
    let Some(folder) = data("Test") else {
        return;
    };
    let full = slam(&folder, Isometry3::identity());
    let dir = std::env::temp_dir().join("driftless-slam-real-reuse");
    std::fs::create_dir_all(&dir).unwrap();
    let cache = dir.join("submaps.bin");
    save_cache(
        &cache,
        &full.result.submaps,
        &full.result.odom_anchors,
        "real-data test",
    )
    .unwrap();

    let start = Instant::now();
    let c = read_cache(&cache).unwrap();
    let r = solve(c.submaps, c.odom_anchors, &[], &SlamConfig::default()).unwrap();
    let poses = r.poses();
    write_tum(&dir.join("traj.tum"), &poses).unwrap();
    write_tum(&dir.join("traj_odom.tum"), &r.odom_poses()).unwrap();
    let map = r.map_points();
    let secs = start.elapsed().as_secs_f64();
    println!(
        "Test: full {:.1} s, reuse {secs:.2} s ({} map points)",
        full.secs,
        map.len()
    );
    assert!(same_poses(&poses, &full.result.poses()));
    assert!(secs < 10.0, "reuse {secs} s");
}

/// New JMK6 scan without reference (63 s): SLAM runs and gives the same result twice. Whether
/// the walk ends at its start is unknown, so the start-end gap is only printed.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scan_143415_deterministic() {
    let Some(folder) = data("jmk6-2026-10-20/20261020143415") else {
        return;
    };
    let a = slam(&folder, Isometry3::identity());
    summary("143415", &a);
    let b = slam(&folder, Isometry3::identity());
    assert!(same_poses(&a.result.poses(), &b.result.poses()));
    assert_eq!(a.result.loops, b.result.loops);
}

/// New JMK6 scan without reference (108 s) that ends where it started: SLAM is deterministic
/// and keeps the start-end gap at the measured value.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn jmk6_scan_143623_closes() {
    let Some(folder) = data("jmk6-2026-10-20/20261020143623") else {
        return;
    };
    let a = slam(&folder, Isometry3::identity());
    let (accepted, _) = summary("143623", &a);
    let b = slam(&folder, Isometry3::identity());
    assert!(same_poses(&a.result.poses(), &b.result.poses()));
    assert_eq!(a.result.loops, b.result.loops);
    assert!(accepted > 0);
    let gap = start_end_gap(&a.result.poses());
    assert!(gap <= 0.024 * 1.2 + 0.01, "start-end gap {gap} m");
}
