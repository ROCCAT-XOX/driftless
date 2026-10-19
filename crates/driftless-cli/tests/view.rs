use assert_cmd::Command;
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::msg::encode::{encode_compressed_image, encode_imu, encode_pointcloud2};
use driftless_io::msg::{CompressedImage, ImuSample, LidarPoint};
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
