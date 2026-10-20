use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_imu, encode_pointcloud2};
use driftless_io::msg::{ImuSample, LidarPoint};
use predicates::prelude::*;
use std::fs;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-odom-{}-{name}", std::process::id()));
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

#[test]
fn odom_writes_trajectory_and_map() {
    let dir = tmp("basic");
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/vanjee_722z", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/vanjee_imu_packets", "sensor_msgs/Imu");
    let t0 = 1000.0;
    for i in 0..=300 {
        let s = t0 + i as f64 * 0.005;
        let imu = ImuSample {
            stamp: s,
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81],
        };
        w.write_message(ic, time(s), &encode_imu(&imu));
        if i % 20 == 0 && i < 300 {
            w.write_message(lc, time(s), &encode_pointcloud2(s, "lidar", &room_scan()));
        }
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
    let out = dir.join("out");

    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .assert()
        .success()
        .stdout(predicate::str::contains("15 Posen"));

    let traj = driftless_traj::read_tum(&out.join("traj.tum")).unwrap();
    assert_eq!(traj.len(), 15);
    assert!(
        traj.iter().all(|p| p.p.norm() < 0.02),
        "static sensor drifted"
    );
    let mut las = las::Reader::from_path(out.join("map.las")).unwrap();
    let n = las.header().number_of_points();
    assert!(n > 1000, "map has {n} points");
    let pd = las.read_points(1).unwrap();
    let p = pd.points().next().unwrap().unwrap();
    assert!(p.x.abs() < 7.0 && p.y.abs() < 6.0 && p.z.abs() < 3.0);
}

#[test]
fn odom_missing_folder_fails() {
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["odom", "/nonexistent/driftless", "-o"])
        .arg(tmp("missing").join("out"))
        .assert()
        .failure()
        .stderr(predicate::str::contains("nicht gefunden"));
}
