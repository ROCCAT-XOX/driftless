use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_imu, encode_pointcloud2};
use driftless_io::msg::{ImuSample, LidarPoint};
use predicates::prelude::*;
use std::fs;
use std::path::PathBuf;

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-glim-{}-{name}",
        std::process::id()
    ));
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

fn point(x: f32, ring: u16, t_rel: f64) -> LidarPoint {
    LidarPoint {
        xyz: [x, 1.0, 2.0],
        intensity: 7.0,
        ring,
        t_rel,
    }
}

fn f64_at(b: &[u8], off: usize) -> f64 {
    f64::from_le_bytes(b[off..off + 8].try_into().unwrap())
}
fn f32_at(b: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(b[off..off + 4].try_into().unwrap())
}
fn u32_at(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(b[off..off + 4].try_into().unwrap())
}

#[test]
fn export_glim_writes_imu_and_lidar_streams() {
    let dir = tmp("basic");
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/vanjee_722z", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/vanjee_imu_packets", "sensor_msgs/Imu");
    for i in 0..3 {
        let s = 100.0 + i as f64 * 0.005;
        let imu = ImuSample {
            stamp: s,
            gyro: [0.1, 0.2, 0.3],
            accel: [0.0, 0.0, 9.81],
        };
        w.write_message(ic, time(s), &encode_imu(&imu));
    }
    for i in 0..2 {
        let s = 100.0 + i as f64 * 0.1;
        let pts = [
            point(1.0, 3, 0.0),
            point(f32::NAN, 4, 0.01),
            point(2.0, 5, 0.05),
        ];
        w.write_message(lc, time(s), &encode_pointcloud2(s, "lidar", &pts));
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
    let out = dir.join("out");

    Command::cargo_bin("driftless")
        .unwrap()
        .args(["export-glim"])
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .assert()
        .success()
        .stdout(predicate::str::contains("2 LiDAR-Frames"))
        .stdout(predicate::str::contains("3 IMU-Messungen"));

    let imu = fs::read(out.join("imu.bin")).unwrap();
    assert_eq!(&imu[..8], b"DLIMU01\0");
    assert_eq!(imu.len(), 8 + 3 * 56);
    assert_eq!(f64_at(&imu, 8), 100.0);
    assert_eq!(f64_at(&imu, 8 + 3 * 8), 9.81); // az
    assert_eq!(f64_at(&imu, 8 + 6 * 8), 0.3); // gz
    assert!((f64_at(&imu, 8 + 2 * 56) - 100.01).abs() < 1e-9);

    let lidar = fs::read(out.join("lidar.bin")).unwrap();
    assert_eq!(&lidar[..8], b"DLPTS01\0");
    // Two frames, NaN point dropped: 2 points of 24 bytes each.
    let frame = 12 + 2 * 24;
    assert_eq!(lidar.len(), 8 + 2 * frame);
    assert_eq!(f64_at(&lidar, 8), 100.0);
    assert_eq!(u32_at(&lidar, 16), 2);
    let p1 = 8 + 12 + 24;
    assert_eq!(f32_at(&lidar, p1), 2.0); // x
    assert_eq!(f32_at(&lidar, p1 + 12), 7.0); // intensity
    assert!((f32_at(&lidar, p1 + 16) - 0.05).abs() < 1e-7); // t_rel
    assert_eq!(u32_at(&lidar, p1 + 20), 5); // ring
    assert!((f64_at(&lidar, 8 + frame) - 100.1).abs() < 1e-9);
}

#[test]
fn export_glim_missing_folder_fails() {
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["export-glim", "/nonexistent/driftless", "-o"])
        .arg(tmp("missing").join("out"))
        .assert()
        .failure()
        .stderr(predicate::str::contains("nicht gefunden"));
}

/// Hesai-style absolute per-point stamps become offsets from the frame stamp; the LiDAR
/// topic can be chosen by name when a bag holds several.
#[test]
fn export_glim_normalises_absolute_point_times_and_selects_topic() {
    let dir = tmp("hesai");
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/hesai/pandar", "sensor_msgs/PointCloud2");
    let oc = w.add_connection("/other", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/alphasense/imu", "sensor_msgs/Imu");
    let s = 1_649_764_528.063_481;
    let imu = ImuSample {
        stamp: s,
        gyro: [0.0; 3],
        accel: [0.0, 9.81, 0.0],
    };
    w.write_message(ic, time(s), &encode_imu(&imu));
    let pts = [point(1.0, 0, s), point(2.0, 1, s + 0.05)];
    w.write_message(lc, time(s), &encode_pointcloud2(s, "PandarXT-32", &pts));
    w.write_message(oc, time(s), &encode_pointcloud2(s, "o", &pts[..1]));
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();

    Command::cargo_bin("driftless")
        .unwrap()
        .args(["export-glim"])
        .arg(&dir)
        .arg("-o")
        .arg(dir.join("a"))
        .assert()
        .failure()
        .stderr(predicate::str::contains("--lidar-topic"));

    let out = dir.join("b");
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["export-glim"])
        .arg(&dir)
        .arg("-o")
        .arg(&out)
        .args(["--lidar-topic", "/hesai/pandar"])
        .assert()
        .success()
        .stdout(predicate::str::contains("1 LiDAR-Frames"));
    let lidar = fs::read(out.join("lidar.bin")).unwrap();
    assert_eq!(lidar.len(), 8 + 12 + 2 * 24);
    assert_eq!(u32_at(&lidar, 16), 2);
    assert_eq!(f32_at(&lidar, 20 + 16), 0.0);
    assert!((f32_at(&lidar, 20 + 24 + 16) - 0.05).abs() < 1e-6);
}

/// Small bag: `n` LiDAR frames on `/lidar` (if `lidar`), `n * 5` IMU samples on `/imu`.
fn small_bag(comp: Compression, per_chunk: usize, lidar: bool, n: usize) -> Vec<u8> {
    let mut w = BagWriter::new(comp).messages_per_chunk(per_chunk);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
    for i in 0..n * 5 {
        let s = 100.0 + i as f64 * 0.01;
        let imu = ImuSample {
            stamp: s,
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81],
        };
        w.write_message(ic, time(s), &encode_imu(&imu));
        if lidar && i % 5 == 0 {
            let pts = [point(1.0, 0, 0.0), point(2.0, 1, 0.01)];
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &pts));
        }
    }
    w.finish()
}

fn export(dir: &std::path::Path, extra: &[&str]) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("export-glim")
        .arg(dir)
        .arg("-o")
        .arg(dir.join("out"))
        .args(extra)
        .assert()
}

#[test]
fn export_glim_fails_on_unsupported_compression() {
    let dir = tmp("bz2");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::Labelled("bz2"), 10, true, 4),
    )
    .unwrap();
    export(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("Fehler"))
        .stderr(predicate::str::contains("bz2"));
}

#[test]
fn export_glim_truncated_bag_warns_and_keeps_data() {
    let dir = tmp("truncated");
    let mut bytes = small_bag(Compression::None, 5, true, 10);
    bytes.truncate(bytes.len() * 7 / 10);
    fs::write(dir.join("LIDAR_0.bag"), bytes).unwrap();
    export(&dir, &[])
        .success()
        .stderr(predicate::str::contains(
            "Warnung: LIDAR_0.bag abgeschnitten (",
        ))
        .stderr(predicate::str::contains("neu vom Gerät kopieren"));
    let lidar = fs::read(dir.join("out/lidar.bin")).unwrap();
    assert!(lidar.len() > 8);
}

#[test]
fn export_glim_unknown_topic_names_are_errors() {
    let dir = tmp("unknown-topic");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::None, 10, true, 2),
    )
    .unwrap();
    export(&dir, &["--lidar-topic", "/nope"])
        .failure()
        .stderr(predicate::str::contains("/nope"))
        .stderr(predicate::str::contains("/lidar"));
    export(&dir, &["--imu-topic", "/nope"])
        .failure()
        .stderr(predicate::str::contains("/nope"))
        .stderr(predicate::str::contains("/imu"));
}

#[test]
fn export_glim_without_lidar_frames_fails() {
    let dir = tmp("no-lidar");
    fs::write(
        dir.join("LIDAR_0.bag"),
        small_bag(Compression::None, 10, false, 2),
    )
    .unwrap();
    export(&dir, &[])
        .failure()
        .stderr(predicate::str::contains("keine LiDAR-Frames"));
}
