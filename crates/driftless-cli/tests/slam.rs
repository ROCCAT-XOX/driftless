use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_imu, encode_pointcloud2};
use driftless_io::msg::{ImuSample, LidarPoint};
use predicates::prelude::*;
use std::fs;
use std::path::{Path, PathBuf};

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-slam-{}-{name}", std::process::id()));
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

/// Static 16-ring scan from inside a 10 x 8 x 3.5 m box room.
fn room_scan() -> Vec<LidarPoint> {
    let (lo, hi) = ([-4.0f64, -3.0, -1.5], [6.0f64, 5.0, 2.0]);
    let mut out = Vec::new();
    for j in 0..360 {
        let az = (j as f64).to_radians();
        for ring in 0..16u16 {
            let el = (-15.0 + 2.0 * f64::from(ring)).to_radians();
            let d = [el.cos() * az.cos(), el.cos() * az.sin(), el.sin()];
            let r = (0..3)
                .filter(|&i| d[i].abs() > 1e-9)
                .map(|i| {
                    if d[i] > 0.0 {
                        hi[i] / d[i]
                    } else {
                        lo[i] / d[i]
                    }
                })
                .fold(f64::INFINITY, f64::min);
            out.push(LidarPoint {
                xyz: [(d[0] * r) as f32, (d[1] * r) as f32, (d[2] * r) as f32],
                intensity: 5.0,
                ring,
                t_rel: 0.1 * j as f64 / 360.0,
            });
        }
    }
    out
}

/// Static room scan for 1.5 s: 15 LiDAR frames on `/lidar`, IMU at 200 Hz on `/imu` (if `imu`).
fn room_bag(dir: &Path, imu: bool) {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
    let scan = room_scan();
    for i in 0..=300 {
        let s = 1000.0 + i as f64 * 0.005;
        if imu {
            let m = ImuSample {
                stamp: s,
                gyro: [0.0; 3],
                accel: [0.0, 0.0, 9.81],
            };
            w.write_message(ic, time(s), &encode_imu(&m));
        }
        if i % 20 == 0 && i < 300 {
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &scan));
        }
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
}

fn slam(dir: &Path, out: &Path) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("slam")
        .arg(dir)
        .arg("-o")
        .arg(out)
        .assert()
}

#[test]
fn slam_writes_all_outputs() {
    let dir = tmp("basic");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out)
        .success()
        .stdout(predicate::str::contains("15 Posen"))
        .stdout(predicate::str::contains("1 Teilkarten"))
        .stdout(predicate::str::contains("keine Schleife gefunden"))
        .stdout(predicate::str::contains("Start↔Ende"))
        .stdout(predicate::str::contains("Laufzeit"));

    let traj = driftless_traj::read_tum(&out.join("traj.tum")).unwrap();
    let odom = driftless_traj::read_tum(&out.join("traj_odom.tum")).unwrap();
    assert_eq!(traj.len(), 15);
    assert_eq!(traj, odom, "single submap: corrected = odometry");
    assert!(
        traj.iter().all(|p| p.p.norm() < 0.02),
        "static sensor drifted"
    );

    let mut las = las::Reader::from_path(out.join("map.las")).unwrap();
    assert!(las.header().number_of_points() > 1000);
    let p = las.read_points(1).unwrap();
    let p = p.points().next().unwrap().unwrap();
    assert!(p.x.abs() < 7.0 && p.y.abs() < 6.0 && p.z.abs() < 3.0);

    let json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(out.join("slam.json")).unwrap()).unwrap();
    let submaps = json["submaps"].as_array().unwrap();
    assert_eq!(submaps.len(), 1);
    assert_eq!(submaps[0]["t_start"], 0.0);
    let t_end = submaps[0]["t_end"].as_f64().unwrap();
    assert!((t_end - 1.4).abs() < 1e-6, "t_end {t_end}");
    assert_eq!(submaps[0]["anchor"].as_array().unwrap().len(), 3);
    assert_eq!(submaps[0]["odom_anchor"], submaps[0]["anchor"]);
    assert!(json["loops"].as_array().unwrap().is_empty());
    assert!(json["graph"].is_null());
    assert!(json["start_end_gap"]["before"].is_number());
    assert!(json["path_length"]["after"].is_number());
}

#[test]
fn slam_is_byte_identical_on_rerun() {
    let dir = tmp("rerun");
    room_bag(&dir, true);
    let (a, b) = (dir.join("a"), dir.join("b"));
    slam(&dir, &a).success();
    slam(&dir, &b).success();
    for f in ["traj.tum", "traj_odom.tum", "slam.json"] {
        assert_eq!(
            fs::read(a.join(f)).unwrap(),
            fs::read(b.join(f)).unwrap(),
            "{f}"
        );
    }
}

#[test]
fn slam_rejects_missing_imu() {
    let dir = tmp("no-imu");
    room_bag(&dir, false);
    slam(&dir, &dir.join("out"))
        .failure()
        .stderr(predicate::str::contains("keine IMU-Messungen"));
    assert!(!dir.join("out/traj.tum").exists());
}
