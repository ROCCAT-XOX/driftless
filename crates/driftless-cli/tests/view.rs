//! `driftless view` with the debug viewer (`--features debug-viewer`): the recordings it
//! writes with `--dry-run`.
#![cfg(feature = "debug-viewer")]

use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_compressed_image, encode_imu, encode_pointcloud2};
use driftless_io::msg::{CompressedImage, ImuSample, LidarPoint};
use predicates::prelude::*;
use std::fs;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-view-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn time(t: f64) -> Time {
    let sec = t.floor();
    Time {
        sec: sec as u32,
        nsec: ((t - sec) * 1e9).round() as u32,
    }
}

fn synthetic_project(name: &str) -> PathBuf {
    let dir = tmp(name);
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/vanjee_722z", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/vanjee_imu_packets", "sensor_msgs/Imu");
    for i in 0..10 {
        let s = 100.0 + i as f64 * 0.1;
        let pts: Vec<LidarPoint> = (0..5)
            .map(|k| LidarPoint {
                xyz: [k as f32 + 1.0, 1.0, 2.0],
                intensity: k as f32 * 10.0,
                ring: 0,
                t_rel: 0.0,
            })
            .collect();
        w.write_message(lc, time(s), &encode_pointcloud2(s, "lidar", &pts));
    }
    for i in 0..200 {
        let s = 100.0 + i as f64 * 0.005;
        let m = ImuSample {
            stamp: s,
            gyro: [0.1, 0.2, 0.3],
            accel: [0.0, 0.0, 9.81],
        };
        w.write_message(ic, time(s), &encode_imu(&m));
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();

    let mut w = BagWriter::new(Compression::None);
    let l = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    for i in 0..10 {
        let s = 100.0 + i as f64 * 0.1;
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: vec![0xff, 0xd8, 0xff, 0xd9],
        };
        w.write_message(l, time(s), &encode_compressed_image(&img));
    }
    fs::write(dir.join("IMAGE_0.bag"), w.finish()).unwrap();
    dir
}

#[test]
fn view_dry_run_writes_rrd() {
    let dir = synthetic_project("dry");
    let traj = dir.join("traj.txt");
    fs::write(
        &traj,
        "100.0 0 0 0 0 0 0 1\n100.5 1 0 0 0 0 0 1\n101.0 2 0 0 0 0 0 1\n",
    )
    .unwrap();
    let rrd = dir.join("out.rrd");
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("view")
        .arg(&dir)
        .arg("--traj")
        .arg(&traj)
        .args(["--every", "2", "--dry-run"])
        .arg(&rrd)
        .assert()
        .success();
    assert!(fs::metadata(&rrd).unwrap().len() > 0);
}

/// Hilti-style topic names; raw `sensor_msgs/Image` topic present (not shown, only warned).
fn hilti_style_project(name: &str, with_lidar: bool) -> PathBuf {
    let dir = tmp(name);
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/hesai/pandar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/alphasense/imu", "sensor_msgs/Imu");
    let cc = w.add_connection(
        "/alphasense/cam0/image_raw/compressed",
        "sensor_msgs/CompressedImage",
    );
    w.add_connection("/alphasense/cam1/image_raw", "sensor_msgs/Image");
    for i in 0..4 {
        let s = 100.0 + i as f64 * 0.1;
        if with_lidar {
            let pts = [LidarPoint {
                xyz: [1.0, 2.0, 3.0],
                intensity: 1.0,
                ring: 0,
                t_rel: 0.0,
            }];
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &pts));
        }
        let m = ImuSample {
            stamp: s,
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81],
        };
        w.write_message(ic, time(s), &encode_imu(&m));
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: vec![0xff, 0xd8, 0xff, 0xd9],
        };
        w.write_message(cc, time(s), &encode_compressed_image(&img));
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
    dir
}

fn view(dir: &std::path::Path, extra: &[&str]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("view")
        .arg(dir)
        .args(extra)
        .arg("--dry-run")
        .arg(dir.join("out.rrd"))
        .assert()
}

#[test]
fn view_shows_non_jmk6_topics() {
    let dir = hilti_style_project("hilti", true);
    view(&dir, &["--every", "1"])
        .success()
        .stdout(predicate::str::contains("4 LiDAR-Frames"))
        .stdout(predicate::str::contains("4 IMU-Messungen"))
        .stdout(predicate::str::contains("4 Kamerabilder"))
        .stderr(predicate::str::contains("sensor_msgs/Image-Topics"))
        .stderr(predicate::str::contains("/alphasense/cam1/image_raw"));
}

#[test]
fn view_without_lidar_topic_is_error_before_opening_output() {
    let dir = hilti_style_project("no-lidar", false);
    // The bag still has the PointCloud2 connection but we drop it by asking for a wrong name.
    view(&dir, &["--lidar-topic", "/nope"])
        .failure()
        .stderr(predicate::str::contains("Fehler"))
        .stderr(predicate::str::contains("/nope"))
        .stderr(predicate::str::contains("/hesai/pandar"));
    assert!(!dir.join("out.rrd").exists());

    let mut w = BagWriter::new(Compression::None);
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
    let m = ImuSample {
        stamp: 1.0,
        gyro: [0.0; 3],
        accel: [0.0, 0.0, 9.81],
    };
    w.write_message(ic, time(1.0), &encode_imu(&m));
    let dir = tmp("really-no-lidar");
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
    view(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("PointCloud2"));
    assert!(!dir.join("out.rrd").exists());
}

#[test]
fn view_bad_trajectory_fails_before_opening_output() {
    let dir = hilti_style_project("bad-traj", true);
    let traj = dir.join("traj.txt");
    fs::write(&traj, "not a trajectory\n").unwrap();
    view(&dir, &["--traj", traj.to_str().unwrap()]).failure();
    assert!(!dir.join("out.rrd").exists());
}

/// A `driftless slam` output folder: 3 submaps (poses every 0.5 s), one accepted, one
/// rejected and one manual position-only loop.
fn slam_output(name: &str) -> PathBuf {
    use driftless_slam::report::{BeforeAfter, LoopReport, SlamReport, SubmapReport};
    let dir = tmp(name);
    let sm = |id: usize, t: f64, quality: f32, x: f64| SubmapReport {
        id,
        t_start: t,
        t_end: t + 1.0,
        quality,
        anchor: [x, 0.5, 0.0],
        odom_anchor: [x, 0.0, 0.0],
    };
    let lp = |a, b, manual, accepted, reason: Option<&str>| LoopReport {
        a,
        b,
        manual,
        accepted,
        reason: reason.map(String::from),
        inlier_ratio: 0.5,
        rms: 0.1,
    };
    let report = SlamReport {
        scan_start: 100.0,
        submaps: vec![
            sm(0, 0.0, 1.0, 0.0),
            sm(1, 2.0, 0.5, 2.0),
            sm(2, 4.0, 0.0, 4.0),
        ],
        loops: vec![
            lp(0, 2, false, true, None),
            lp(0, 1, false, false, Some("zu wenige Übereinstimmungen")),
            lp(1, 2, true, true, Some("nur Position verbunden")),
        ],
        graph: None,
        start_end_gap: BeforeAfter {
            before: 1.0,
            after: 0.1,
        },
        path_length: BeforeAfter {
            before: 5.0,
            after: 5.0,
        },
    };
    fs::write(
        dir.join("slam.json"),
        serde_json::to_string(&report).unwrap(),
    )
    .unwrap();
    let tum = |dy: f64| {
        (0..12)
            .map(|i| {
                format!(
                    "{} {} {dy} 0 0 0 0 1\n",
                    100.0 + i as f64 * 0.5,
                    i as f64 * 0.5
                )
            })
            .collect::<String>()
    };
    fs::write(dir.join("traj.tum"), tum(0.5)).unwrap();
    fs::write(dir.join("traj_odom.tum"), tum(0.0)).unwrap();
    dir
}

#[test]
fn view_slam_logs_quality_and_loops() {
    let dir = hilti_style_project("slam", true);
    let out = slam_output("slam-out");
    view(&dir, &["--slam", out.to_str().unwrap()])
        .success()
        .stdout(predicate::str::contains("Zeitleiste ab_scan_beginn"));
    let rrd = fs::read(dir.join("out.rrd")).unwrap();
    let has = |name: &str| rrd.windows(name.len()).any(|w| w == name.as_bytes());
    for entity in [
        "slam/trajectory",
        "slam/odometry",
        "slam/loops",
        "slam/anchors",
        // the timeline of the correction times
        "ab_scan_beginn",
    ] {
        assert!(has(entity), "{entity} fehlt in der Aufzeichnung");
    }

    // without --slam there is no scan start, so no such timeline
    view(&dir, &[])
        .success()
        .stdout(predicate::str::contains("ab_scan_beginn").not());
    let rrd = fs::read(dir.join("out.rrd")).unwrap();
    assert!(!rrd
        .windows("ab_scan_beginn".len())
        .any(|w| w == b"ab_scan_beginn"));
}

#[test]
fn view_slam_missing_report_errors() {
    let dir = hilti_style_project("slam-missing", true);
    let out = tmp("slam-empty");
    view(&dir, &["--slam", out.to_str().unwrap()])
        .failure()
        .stderr(predicate::str::contains("Fehler"))
        .stderr(predicate::str::contains("slam.json"));
    assert!(!dir.join("out.rrd").exists());
}

/// Real recording; prints runtime. Run with `cargo test -- --ignored --nocapture`.
#[test]
#[ignore]
fn view_dry_run_real_project() {
    let Some(folder) = driftless_io::testdata::path("Test") else {
        eprintln!("DRIFTLESS_TESTDATA nicht gesetzt, übersprungen");
        return;
    };
    let dir = tmp("real");
    let rrd = dir.join("real.rrd");
    let start = std::time::Instant::now();
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("view")
        .arg(&folder)
        .arg("--dry-run")
        .arg(&rrd)
        .assert()
        .success();
    eprintln!(
        "view real: {:.1} s, rrd {} MB",
        start.elapsed().as_secs_f64(),
        fs::metadata(&rrd).unwrap().len() / 1_000_000
    );
    assert!(fs::metadata(&rrd).unwrap().len() > 0);
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn view_cloud_dry_run() {
    use driftless_map::{write_las, Cloud};
    let dir = synthetic_project("cloud");
    let cloud = |rgb: bool| Cloud {
        xyz: (0..10).map(|i| [i as f32, 1.0, 2.0]).collect(),
        intensity: (0..10).map(|i| 20.0 * i as f32).collect(),
        t: vec![100.0; 10],
        frame: vec![0; 10],
        rgb: rgb.then(|| vec![[255, 0, 0]; 10]),
        colored: rgb.then(|| vec![true; 10]),
    };
    write_las(&dir.join("color.las"), &cloud(true)).unwrap();
    write_las(&dir.join("dense.las"), &cloud(false)).unwrap();
    for (file, kind) in [("color.las", "Farbe"), ("dense.las", "Intensität")] {
        let rrd = dir.join(format!("{file}.rrd"));
        Command::cargo_bin("driftless")
            .unwrap()
            .arg("view")
            .arg(&dir)
            .arg("--cloud")
            .arg(dir.join(file))
            .arg("--dry-run")
            .arg(&rrd)
            .assert()
            .success()
            .stdout(predicate::str::contains(format!(
                "Wolke {file}: 10 von 10 Punkten ({kind})"
            )));
        assert!(fs::metadata(&rrd).unwrap().len() > 0);
    }
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("view")
        .arg(&dir)
        .arg("--cloud")
        .arg(dir.join("missing.las"))
        .arg("--dry-run")
        .arg(dir.join("missing.rrd"))
        .assert()
        .failure()
        .stderr(predicate::str::contains("missing.las"));
    assert!(!dir.join("missing.rrd").exists());
}
