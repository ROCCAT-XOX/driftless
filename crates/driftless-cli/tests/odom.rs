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

/// Static box-room bag: LiDAR on `lidar_topic`, IMU on `/vanjee_imu_packets`, optionally a
/// second PointCloud2 topic `/other` with a single far point per frame.
fn room_bag(dir: &std::path::Path, lidar_topic: &str, with_other_cloud: bool) {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection(lidar_topic, "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/vanjee_imu_packets", "sensor_msgs/Imu");
    let oc = with_other_cloud.then(|| w.add_connection("/other", "sensor_msgs/PointCloud2"));
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
            if let Some(oc) = oc {
                let far = [LidarPoint {
                    xyz: [30.0, 0.0, 0.0],
                    intensity: 1.0,
                    ring: 0,
                    t_rel: 0.0,
                }];
                w.write_message(oc, time(s + 0.001), &encode_pointcloud2(s, "o", &far));
            }
        }
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
}

fn las_min_x(path: &std::path::Path) -> f64 {
    let mut las = las::Reader::from_path(path).unwrap();
    let data = las.read_all().unwrap();
    data.points()
        .map(|p| p.unwrap().x)
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn odom_applies_lidar_to_imu_extrinsic() {
    let dir = tmp("extrinsic");
    room_bag(&dir, "/vanjee_722z", false);
    let (plain, turned) = (dir.join("plain"), dir.join("turned"));
    let odom = |out: &PathBuf, extra: &[&str]| {
        Command::cargo_bin("driftless")
            .unwrap()
            .arg("odom")
            .arg(&dir)
            .arg("-o")
            .arg(out)
            .args(extra)
            .assert()
            .success()
            .stdout(predicate::str::contains("15 Posen"));
    };
    odom(&plain, &[]);
    // 180° about z (qz = 1): the room x range [-4, 6] becomes [-6, 4].
    odom(
        &turned,
        &["--lidar-to-imu", "0", "0", "0", "0", "0", "1", "0"],
    );
    assert!(las_min_x(&plain.join("map.las")) > -4.5);
    assert!(las_min_x(&turned.join("map.las")) < -5.5);
    let traj = driftless_traj::read_tum(&turned.join("traj.tum")).unwrap();
    assert!(traj.iter().all(|p| p.p.norm() < 0.02));
}

#[test]
fn odom_rejects_malformed_extrinsic() {
    let dir = tmp("bad-extrinsic");
    room_bag(&dir, "/vanjee_722z", false);
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("out"))
        .args(["--lidar-to-imu", "0", "0", "0", "0", "0", "0", "0"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("Quaternion"));
}

#[test]
fn odom_refuses_ambiguous_lidar_topics_and_selects_by_name() {
    let dir = tmp("topics");
    room_bag(&dir, "/hesai/pandar", true);
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("a"))
        .assert()
        .failure()
        .stderr(predicate::str::contains("/hesai/pandar"))
        .stderr(predicate::str::contains("/other"))
        .stderr(predicate::str::contains("--lidar-topic"));
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("b"))
        .args(["--lidar-topic", "/hesai/pandar"])
        .args(["--imu-topic", "/vanjee_imu_packets"])
        .assert()
        .success()
        .stdout(predicate::str::contains("15 Posen"));
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("c"))
        .args(["--lidar-topic", "/nope"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("/nope"));
}

#[test]
fn odom_without_deskew_runs() {
    let dir = tmp("no-deskew");
    room_bag(&dir, "/vanjee_722z", false);
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("out"))
        .arg("--no-deskew")
        .assert()
        .success()
        .stdout(predicate::str::contains("15 Posen"))
        .stdout(predicate::str::contains("ohne Deskew"));
}

/// Small bag: `frames` room scans on `/lidar` (if `lidar`), IMU on `/imu` (if `imu`).
fn small_bag(comp: Compression, per_chunk: usize, lidar: bool, imu: bool) -> Vec<u8> {
    let mut w = BagWriter::new(comp).messages_per_chunk(per_chunk);
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
        if lidar && i % 20 == 0 && i < 300 {
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &scan));
        }
    }
    w.finish()
}

fn odom(dir: &std::path::Path, extra: &[&str]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("odom")
        .arg(dir)
        .arg("-o")
        .arg(dir.join("out"))
        .args(extra)
        .assert()
}

#[test]
fn odom_fails_on_unsupported_compression() {
    let dir = tmp("bz2");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::Labelled("bz2"), 50, true, true),
    )
    .unwrap();
    odom(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("Fehler"))
        .stderr(predicate::str::contains("bz2"));
    assert!(!dir.join("out/traj.tum").exists());
}

#[test]
fn odom_truncated_bag_warns_and_keeps_data() {
    let dir = tmp("truncated");
    let mut bytes = small_bag(Compression::None, 40, true, true);
    bytes.truncate(bytes.len() * 7 / 10);
    fs::write(dir.join("LIDAR_0.bag"), bytes).unwrap();
    odom(&dir, &[])
        .success()
        .stderr(predicate::str::contains(
            "Warnung: LIDAR_0.bag abgeschnitten (",
        ))
        .stderr(predicate::str::contains("neu vom Gerät kopieren"));
    let traj = driftless_traj::read_tum(&dir.join("out/traj.tum")).unwrap();
    assert!(!traj.is_empty() && traj.len() < 15, "{} poses", traj.len());
}

#[test]
fn odom_unknown_topic_names_list_available_ones() {
    let dir = tmp("unknown-topic");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::None, 50, true, true),
    )
    .unwrap();
    odom(&dir, &["--imu-topic", "/nope"])
        .failure()
        .stderr(predicate::str::contains("/nope"))
        .stderr(predicate::str::contains("/imu"));
    odom(&dir, &["--lidar-topic", "/nope"])
        .failure()
        .stderr(predicate::str::contains("/nope"))
        .stderr(predicate::str::contains("/lidar"));
}

#[test]
fn odom_without_imu_fails() {
    let dir = tmp("no-imu");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::None, 50, true, false),
    )
    .unwrap();
    odom(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("keine IMU-Messungen"));
}

#[test]
fn odom_without_lidar_frames_fails() {
    let dir = tmp("no-lidar");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::None, 50, false, true),
    )
    .unwrap();
    odom(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("keine LiDAR-Frames"));
    assert!(!dir.join("out/traj.tum").exists());
}
