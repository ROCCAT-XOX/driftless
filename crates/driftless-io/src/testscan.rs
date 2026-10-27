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
    let scan = room_scan();
    static_bag(dir, imu, start, secs, |_| scan.clone());
}

/// Bag of a device standing still for `secs` seconds from bag time `start`: frame `k` is
/// `scan(k)`, one every 0.1 s on `/lidar`, IMU at 200 Hz on `/imu` (if `imu`, level: gravity
/// along +z).
fn static_bag(
    dir: &Path,
    imu: bool,
    start: f64,
    secs: f64,
    scan: impl Fn(usize) -> Vec<LidarPoint>,
) {
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let ic = w.add_connection("/imu", "sensor_msgs/Imu");
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
            w.write_message(lc, time(s), &encode_pointcloud2(s, "l", &scan(i / 20)));
        }
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();
}

// The room of `room_scan` with furniture, scanned from another place.

/// Axis-aligned box `[lo, hi]` in the room frame.
pub type Block = ([f64; 3], [f64; 3]);

/// Two boxes on the floor of the room of [`room_scan`], at places without symmetry: a low
/// one near the −y wall and a high one in the +y half near the −x wall.
pub const FURNITURE: [Block; 2] = [
    ([1.8, -2.6, -1.5], [2.8, -1.8, -0.7]),
    ([-3.2, 2.2, -1.5], [-2.4, 3.4, -0.3]),
];

/// Distance along `d` from `o` (inside the room) to the first surface: a wall, the floor,
/// the ceiling or one of `blocks`.
fn first_hit(o: &Vector3<f64>, d: &Vector3<f64>, blocks: &[Block]) -> f64 {
    let mut r = (0..3)
        .filter(|&i| d[i].abs() > 1e-9)
        .map(|i| {
            let bound = if d[i] > 0.0 { HI[i] } else { LO[i] };
            (bound - o[i]) / d[i]
        })
        .fold(f64::INFINITY, f64::min);
    for (blo, bhi) in blocks {
        // Slab test: entry is the largest near distance, exit the smallest far distance.
        let (mut near, mut far) = (0.0f64, f64::INFINITY);
        for i in 0..3 {
            if d[i].abs() < 1e-12 {
                if o[i] < blo[i] || o[i] > bhi[i] {
                    far = -1.0;
                }
                continue;
            }
            let (a, b) = ((blo[i] - o[i]) / d[i], (bhi[i] - o[i]) / d[i]);
            near = near.max(a.min(b));
            far = far.min(a.max(b));
        }
        if near <= far && near > 0.0 {
            r = r.min(near);
        }
    }
    r
}

/// Frame `k` of a static scan of the room of [`room_scan`] with `blocks`, from a device at
/// `offset` turned by `yaw` (rad) about the vertical, in the device frame: 71 rings from
/// −35° to +35° elevation, every 1° in azimuth, the pattern shifted by `k mod 3` thirds of a
/// degree in azimuth and `(k / 3) mod 5` fifths in elevation. Like a non-repeating scan
/// pattern, 15 frames fill a grid of ⅓° × ⅕°: a cloud as dense as a real handheld scan
/// (a point every ≈ 3 cm at 8 m), so that two scans from different places overlap
/// everywhere and their edges agree.
pub fn room_scan_from(offset: [f64; 3], yaw: f64, blocks: &[Block], k: usize) -> Vec<LidarPoint> {
    const RINGS: u16 = 71;
    const STEPS: usize = 360;
    let az0 = (k % 3) as f64 / 3.0;
    let el0 = ((k / 3) % 5) as f64 / 5.0;
    let o = Vector3::from(offset);
    let turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
    let mut out = Vec::with_capacity(STEPS * usize::from(RINGS));
    for j in 0..STEPS {
        let az = (j as f64 + az0).to_radians();
        for ring in 0..RINGS {
            let el = (-35.0 + f64::from(ring) + el0).to_radians();
            let d = Vector3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            let r = first_hit(&o, &(turn * d), blocks);
            let p = d * r;
            out.push(LidarPoint {
                xyz: [p.x as f32, p.y as f32, p.z as f32],
                intensity: 5.0,
                ring,
                t_rel: 0.1 * j as f64 / STEPS as f64,
            });
        }
    }
    out
}

/// Scan folder: the room of [`room_scan`] with the two boxes of [`FURNITURE`], scanned for
/// 1.5 s by a device standing still at `offset` in the room, turned by `yaw` (rad) about the
/// vertical. The map of the scan is in the device frame, so the pose of this scan in the
/// frame of a scan at the origin with `yaw = 0` is `offset`, `yaw`.
pub fn furnished_room_bag(dir: &Path, imu: bool, offset: [f64; 3], yaw: f64) {
    static_bag(dir, imu, 1000.0, 1.5, |k| {
        room_scan_from(offset, yaw, &FURNITURE, k)
    });
}

/// [`furnished_room_bag`] without the boxes: the bare room, which looks the same turned by
/// 180° about its centre.
pub fn empty_room_bag(dir: &Path, imu: bool, offset: [f64; 3], yaw: f64) {
    static_bag(dir, imu, 1000.0, 1.5, |k| {
        room_scan_from(offset, yaw, &[], k)
    });
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

/// Colour of a box of [`FURNITURE`] in the images of [`furnished_scan_with_images`].
pub const BLOCK_COLOR: [u8; 3] = [220, 120, 40];

/// Colour where the ray from `o` along `d` first meets the room with `blocks`: [`cast`] on
/// a wall, [`BLOCK_COLOR`] on a box.
fn cast_in(o: &Vector3<f64>, d: &Vector3<f64>, blocks: &[Block]) -> [u8; 3] {
    if !blocks.is_empty() && first_hit(o, d, blocks) < first_hit(o, d, &[]) - 1e-9 {
        return BLOCK_COLOR;
    }
    cast(o, d)
}

/// Stored image of the static device at `device` (device → room) in the room with
/// `blocks`, through the calibration of `dir` as Driftless reads it.
fn render_from(dir: &Path, device: &Isometry3<f64>, blocks: &[Block]) -> Vec<u8> {
    let calib = load_calibration(&dir.join("calibration/calib.json")).unwrap();
    let c = &calib.cameras["left"];
    let cam = Camera::from_calib("left", c).unwrap();
    let world_cam = device * c.lidar_to_cam.unwrap().inverse();
    let o = world_cam.translation.vector;
    let mut rgb = Vec::new();
    for y in 0..cam.height {
        for x in 0..cam.width {
            let px = cam
                .unproject([f64::from(x), f64::from(y)])
                .map_or([0; 3], |d| cast_in(&o, &(world_cam.rotation * d), blocks));
            rgb.extend(px);
        }
    }
    rgb
}

/// Writes the calibration and `IMAGES` camera images of the static device at `device` in
/// the room with `blocks` into the scan folder `dir`.
fn write_images(dir: &Path, device: &Isometry3<f64>, blocks: &[Block]) {
    write_calib(dir);
    let mut jpeg = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpeg, 95)
        .encode(
            &render_from(dir, device, blocks),
            u32::from(IMAGE_W),
            u32::from(IMAGE_H),
            image::ExtendedColorType::Rgb8,
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

/// Scan folder: static room (LiDAR + IMU), `IMAGES` camera images, calibration.
pub fn scan_with_images(dir: &Path) {
    room_bag(dir, true);
    write_images(dir, &Isometry3::identity(), &[]);
}

/// [`furnished_room_bag`] with the calibration and `IMAGES` camera images of the device at
/// `offset`, turned by `yaw` (rad): the camera looks along the device's +x, the boxes are
/// [`BLOCK_COLOR`].
pub fn furnished_scan_with_images(dir: &Path, offset: [f64; 3], yaw: f64) {
    furnished_room_bag(dir, true, offset, yaw);
    let device = Isometry3::from_parts(
        Translation3::from(Vector3::from(offset)),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
    );
    write_images(dir, &device, &FURNITURE);
}
