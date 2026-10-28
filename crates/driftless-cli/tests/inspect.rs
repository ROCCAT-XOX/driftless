use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_compressed_image, encode_imu, encode_pointcloud2};
use driftless_io::msg::{CompressedImage, ImuSample, LidarPoint};
use driftless_io::testdir::TestDir;
use predicates::prelude::*;
use serde_json::Value;
use std::fs;
use std::path::Path;

fn tmp(name: &str) -> TestDir {
    TestDir::new(format!("driftless-inspect-{}-{name}", std::process::id()))
}

fn time(t: f64) -> Time {
    let sec = t.floor();
    Time {
        sec: sec as u32,
        nsec: ((t - sec) * 1e9).round() as u32,
    }
}

fn point(x: f32) -> LidarPoint {
    LidarPoint {
        xyz: [x, 1.0, 2.0],
        intensity: 1.0,
        ring: 0,
        t_rel: 0.0,
    }
}

fn imu(stamp: f64) -> ImuSample {
    ImuSample {
        stamp,
        gyro: [0.0; 3],
        accel: [0.0, 0.0, 9.81],
    }
}

/// LiDAR 10 Hz and IMU 200 Hz for `secs` seconds; each frame has one NaN point.
/// `lidar_stamps` overrides the frame stamps when given.
fn lidar_bag(secs: f64, lidar_stamps: Option<Vec<f64>>) -> Vec<u8> {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/vanjee_722z", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/vanjee_imu_packets", "sensor_msgs/Imu");
    let stamps = lidar_stamps.unwrap_or_else(|| {
        (0..(secs * 10.0) as usize)
            .map(|i| 100.0 + i as f64 * 0.1)
            .collect()
    });
    for s in stamps {
        let pts = [point(1.0), point(f32::NAN)];
        w.write_message(lc, time(s), &encode_pointcloud2(s, "lidar", &pts));
    }
    for i in 0..(secs * 200.0) as usize {
        let s = 100.0 + i as f64 * 0.005;
        w.write_message(ic, time(s), &encode_imu(&imu(s)));
    }
    w.finish()
}

fn image_bag(secs: f64) -> Vec<u8> {
    let mut w = BagWriter::new(Compression::None);
    let l = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    for i in 0..(secs * 4.0) as usize {
        let s = 100.0 + i as f64 * 0.25;
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: vec![0xff, 0xd8, 0xff, 0xd9],
        };
        w.write_message(l, time(s), &encode_compressed_image(&img));
    }
    w.finish()
}

const CALIB: &str = r#"{"device":{"SN":"SN123","model":"JMK6"},"camera_info":{},"out_put":{}}"#;

fn project(name: &str, lidar: Vec<u8>, image: Option<Vec<u8>>, calib: bool) -> TestDir {
    let dir = tmp(name);
    fs::write(dir.join("LIDAR_0.bag"), lidar).unwrap();
    if let Some(i) = image {
        fs::write(dir.join("IMAGE_0.bag"), i).unwrap();
    }
    if calib {
        fs::create_dir_all(dir.join("calibration")).unwrap();
        fs::write(dir.join("calibration/calib.json"), CALIB).unwrap();
    }
    dir
}

fn inspect_json(dir: &Path) -> Value {
    let out = Command::cargo_bin("driftless")
        .unwrap()
        .args(["inspect", "--json"])
        .arg(dir)
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).unwrap()
}

fn topic<'a>(v: &'a Value, name: &str) -> &'a Value {
    v["topics"]
        .as_array()
        .unwrap()
        .iter()
        .find(|t| t["topic"] == name)
        .unwrap_or_else(|| panic!("topic {name} missing in {v}"))
}

#[test]
fn inspect_reports_counts_and_rates() {
    let dir = project("rates", lidar_bag(10.0, None), Some(image_bag(10.0)), true);
    let v = inspect_json(&dir);
    let lidar = topic(&v, "/vanjee_722z");
    assert_eq!(lidar["count"], 100);
    assert_eq!(lidar["type"], "sensor_msgs/PointCloud2");
    assert!((lidar["rate_hz"].as_f64().unwrap() - 10.0).abs() < 0.1);
    let imu = topic(&v, "/vanjee_imu_packets");
    assert_eq!(imu["count"], 2000);
    assert!((imu["rate_hz"].as_f64().unwrap() - 200.0).abs() < 2.0);
    let cam = topic(&v, "/camera_left/image_raw");
    assert!((cam["rate_hz"].as_f64().unwrap() - 4.0).abs() < 0.04);
    assert_eq!(v["dropped_points"], 100);
    assert_eq!(v["device"]["sn"], "SN123");
    assert!(v["problems"].as_array().unwrap().is_empty());

    Command::cargo_bin("driftless")
        .unwrap()
        .arg("inspect")
        .arg(&dir)
        .assert()
        .success()
        .stdout(predicate::str::contains("SN123"))
        .stdout(predicate::str::contains("/vanjee_722z"))
        .stdout(predicate::str::contains("Hz"));
}

#[test]
fn inspect_flags_non_monotonic() {
    let mut stamps: Vec<f64> = (0..20).map(|i| 100.0 + i as f64 * 0.1).collect();
    stamps.swap(5, 6);
    stamps[12] = 100.0; // jumps back
    let dir = project("nonmono", lidar_bag(1.0, Some(stamps)), None, true);
    let v = inspect_json(&dir);
    assert_eq!(topic(&v, "/vanjee_722z")["non_monotonic"], 2);
    assert_eq!(topic(&v, "/vanjee_imu_packets")["non_monotonic"], 0);

    Command::cargo_bin("driftless")
        .unwrap()
        .arg("inspect")
        .arg(&dir)
        .assert()
        .success()
        .stdout(predicate::str::contains("nicht monoton"));
}

#[test]
fn inspect_lists_problems() {
    let dir = project("problems", lidar_bag(1.0, None), None, false);
    let v = inspect_json(&dir);
    let problems: Vec<&str> = v["problems"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p.as_str().unwrap())
        .collect();
    assert!(
        problems.iter().any(|p| p.contains("IMAGE_*.bag")),
        "{problems:?}"
    );
    assert!(
        problems.iter().any(|p| p.contains("calib.json")),
        "{problems:?}"
    );
    // LiDAR still inspected without images and calibration.
    assert_eq!(topic(&v, "/vanjee_722z")["count"], 10);
    assert!(v["device"].is_null());
}

#[test]
fn inspect_reports_truncated_bag_with_offset() {
    let mut bytes = lidar_bag(2.0, None);
    bytes.truncate(bytes.len() * 2 / 3);
    let dir = project("trunc", bytes, None, true);
    let v = inspect_json(&dir);
    let problems = v["problems"].to_string();
    assert!(problems.contains("abgeschnitten"), "{problems}");
    assert!(problems.contains("Byte"), "{problems}");
}

#[test]
fn inspect_counts_decode_errors_and_continues() {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/vanjee_722z", "sensor_msgs/PointCloud2");
    w.write_message(lc, time(100.0), &[1, 2, 3]);
    w.write_message(
        lc,
        time(100.1),
        &encode_pointcloud2(100.1, "l", &[point(1.0)]),
    );
    let dir = project("decode", w.finish(), None, true);
    let v = inspect_json(&dir);
    let t = topic(&v, "/vanjee_722z");
    assert_eq!(t["count"], 1);
    assert_eq!(t["decode_errors"], 1);
}

#[test]
fn inspect_json_schema() {
    let dir = project("schema", lidar_bag(1.0, None), Some(image_bag(1.0)), true);
    let v = inspect_json(&dir);
    for key in ["device", "topics", "dropped_points", "problems"] {
        assert!(v.get(key).is_some(), "missing {key}");
    }
    for t in v["topics"].as_array().unwrap() {
        for key in [
            "topic",
            "type",
            "count",
            "rate_hz",
            "first",
            "last",
            "non_monotonic",
            "decode_errors",
        ] {
            assert!(t.get(key).is_some(), "missing {key} in {t}");
        }
    }
}

#[test]
fn inspect_missing_folder_fails() {
    Command::cargo_bin("driftless")
        .unwrap()
        .args(["inspect", "/nonexistent/driftless"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("nicht gefunden"));
}

/// Real project: `cargo test -p driftless-cli -- --ignored`.
#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn inspect_real_project() {
    let Some(dir) = driftless_io::testdata::path("Test") else {
        println!("SKIPPED: DRIFTLESS_TESTDATA not set");
        return;
    };
    let v = inspect_json(&dir);
    let near = |x: f64, want: f64| (x - want).abs() <= want * 0.05;
    let rate = |name: &str| topic(&v, name)["rate_hz"].as_f64().unwrap();
    assert!(near(rate("/vanjee_722z"), 10.0));
    assert!(near(rate("/vanjee_imu_packets"), 200.0));
    for cam in ["/camera_left/image_raw", "/camera_right/image_raw"] {
        assert!(near(rate(cam), 3.3), "{cam}: {}", rate(cam));
    }
    assert!(near(v["duration_s"].as_f64().unwrap(), 266.0));
}
