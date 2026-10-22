//! Synthetic scan folders for tests of the steps (`slam`, `correct`, `map`, …).

use crate::bag::writer::{BagWriter, Compression};
use crate::bag::Time;
use crate::calib::load_calibration;
use crate::camera::Camera;
use crate::msg::encode::encode_compressed_image;
use crate::msg::encode::{encode_imu, encode_pointcloud2};
use crate::msg::CompressedImage;
use crate::msg::{ImuSample, LidarPoint};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
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
    room_bag_at(dir, imu, 1000.0, secs);
}

/// [`room_bag_secs`] starting at bag time `start` (s).
pub fn room_bag_at(dir: &Path, imu: bool, start: f64, secs: f64) {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
    let scan = room_scan();
    let n = (secs * 200.0).round() as usize;
    for i in 0..=n {
        let s = start + i as f64 * 0.005;
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

// A room with camera images and a calibration.

/// The room of `common::room_scan`.
const LO: [f64; 3] = [-4.0, -3.0, -1.5];
const HI: [f64; 3] = [6.0, 5.0, 2.0];
/// Colour of the +x wall, plain (the camera looks at it); the others carry a checkerboard.
pub const WALL_X: [u8; 3] = [200, 200, 60];
pub const IMAGE_W: u16 = 160;
pub const IMAGE_H: u16 = 120;
pub const IMAGES: usize = 5;

/// Camera pose in the LiDAR frame: looking along LiDAR +x, tilted 10° down, offset a bit.
fn lidar_cam() -> Isometry3<f64> {
    let base = UnitQuaternion::from_basis_unchecked(&[
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ]);
    let tilt = UnitQuaternion::from_euler_angles(10f64.to_radians(), 0.0, 0.0);
    Isometry3::from_parts(Translation3::new(0.05, -0.1, 0.03), base * tilt)
}

/// Colour where the ray from `o` along `d` leaves the room.
fn cast(o: &Vector3<f64>, d: &Vector3<f64>) -> [u8; 3] {
    let (mut t, mut axis) = (f64::INFINITY, 0);
    for i in 0..3 {
        if d[i].abs() > 1e-12 {
            let bound = if d[i] > 0.0 { HI[i] } else { LO[i] };
            let ti = (bound - o[i]) / d[i];
            if ti > 0.0 && ti < t {
                (t, axis) = (ti, i);
            }
        }
    }
    if axis == 0 && d[0] > 0.0 {
        return WALL_X;
    }
    let p = o + d * t;
    let checker = (0..3)
        .filter(|&i| i != axis)
        .map(|i| (p[i] * 2.0).floor() as i64)
        .sum::<i64>()
        .rem_euclid(2);
    let base = [[60, 60, 200], [200, 60, 200], [60, 200, 200]][axis];
    base.map(|c: u8| if checker == 0 { c } else { c / 2 })
}

fn write_calib(dir: &Path) {
    let m = lidar_cam().inverse().to_homogeneous();
    let rows: Vec<Vec<f64>> = (0..4)
        .map(|i| (0..4).map(|j| m[(i, j)]).collect())
        .collect();
    let json = serde_json::json!({
        "camera_info": {"left": {
            "K": [55.0, 0.0, 59.5, 0.0, 55.0, 79.5, 0.0, 0.0, 1.0],
            "camera_model": "fisheyeKB",
            "coeff": [0.0, 0.0, 0.0, 0.0],
            "image_width": IMAGE_W,
            "image_height": IMAGE_H
        }},
        "out_put": {"left": {"transform_matrix": rows}}
    });
    fs::create_dir_all(dir.join("calibration")).unwrap();
    fs::write(dir.join("calibration/calib.json"), json.to_string()).unwrap();
}

/// Stored image of the static device (at the origin) through the calibration of `dir` as
/// Driftless reads it.
fn render_from(dir: &Path) -> Vec<u8> {
    let calib = load_calibration(&dir.join("calibration/calib.json")).unwrap();
    let c = &calib.cameras["left"];
    let cam = Camera::from_calib("left", c).unwrap();
    let world_cam = c.lidar_to_cam.unwrap().inverse();
    let o = world_cam.translation.vector;
    let mut rgb = Vec::new();
    for y in 0..cam.height {
        for x in 0..cam.width {
            let px = cam
                .unproject([f64::from(x), f64::from(y)])
                .map_or([0; 3], |d| cast(&o, &(world_cam.rotation * d)));
            rgb.extend(px);
        }
    }
    rgb
}

/// Scan folder: static room (LiDAR + IMU), `IMAGES` camera images, calibration.
pub fn scan_with_images(dir: &Path) {
    room_bag(dir, true);
    write_calib(dir);
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 95)
        .encode(
            &render_from(dir),
            IMAGE_W,
            IMAGE_H,
            jpeg_encoder::ColorType::Rgb,
        )
        .unwrap();
    let mut w = BagWriter::new(Compression::None);
    let ic = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    for k in 0..IMAGES {
        let s = 1000.3 + 0.2 * k as f64;
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: jpeg.clone(),
        };
        w.write_message(ic, time(s), &encode_compressed_image(&img));
    }
    fs::write(dir.join("IMAGE_0.bag"), w.finish()).unwrap();
}
