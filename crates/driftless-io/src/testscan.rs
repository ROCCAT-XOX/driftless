//! Synthetic scan folders for tests of the steps (`slam`, `correct`, `map`, …).

use crate::bag::writer::{BagWriter, Compression};
use crate::bag::Time;
use crate::msg::encode::{encode_imu, encode_pointcloud2};
use crate::msg::{ImuSample, LidarPoint};
use std::fs;
use std::path::{Path, PathBuf};

/// Fresh, empty temporary folder for the test `name` (unique per process).
pub fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-cli-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

/// ROS time of `t` seconds.
pub fn time(t: f64) -> Time {
    let sec = t.floor();
    Time {
        sec: sec as u32,
        nsec: ((t - sec) * 1e9).round() as u32,
    }
}

/// Static 16-ring scan from inside a 10 x 8 x 3.5 m box room.
pub fn room_scan() -> Vec<LidarPoint> {
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
pub fn room_bag(dir: &Path, imu: bool) {
    room_bag_secs(dir, imu, 1.5);
}

/// Static room scan for `secs` seconds: a LiDAR frame every 0.1 s, IMU at 200 Hz.
pub fn room_bag_secs(dir: &Path, imu: bool, secs: f64) {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
    let scan = room_scan();
    let n = (secs * 200.0).round() as usize;
    for i in 0..=n {
        let s = 1000.0 + i as f64 * 0.005;
        if imu {
            let m = ImuSample {
                stamp: s,
                gyro: [0.0; 3],
                accel: [0.0, 0.0, 9.81],
            };
            w.write_message(ic, time(s), &encode_imu(&m));
        }
        if i % 20 == 0 && i < n {
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &scan));
        }
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
}
