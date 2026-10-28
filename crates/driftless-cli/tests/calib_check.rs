//! `driftless calib-check` on a synthetic scan: a room with pillars, a yawing and moving
//! device, LiDAR frames and camera images rendered from known poses.

mod common;

use assert_cmd::Command;
use common::{room_bag, time, tmp};
use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::calib::{load_calibration_as, CalibReading, ImageRotation, MatrixDirection};
use driftless_io::camera::Camera;
use driftless_io::msg::encode::{encode_compressed_image, encode_pointcloud2};
use driftless_io::msg::{CompressedImage, LidarPoint};
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use driftless_traj::{write_tum, Pose};
use predicates::prelude::*;
use std::fs;
use std::path::Path;

const T0: f64 = 1000.0;
const DURATION: f64 = 6.0;
/// The camera stamps lag the true capture time by this (s).
const TRUE_OFFSET: f64 = 0.05;
const IMAGE_W: u16 = 800;
const IMAGE_H: u16 = 600;

/// Device (LiDAR) pose in the world at `t`: constant yaw rate and velocity.
fn device_pose(t: f64) -> Isometry3<f64> {
    let s = t - T0;
    Isometry3::from_parts(
        Translation3::new(0.15 * s, 0.05 * s, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * s),
    )
}

/// Camera pose in the LiDAR frame: looking along LiDAR +x, tilted 10° down, offset a bit.
fn lidar_cam() -> Isometry3<f64> {
    // Columns: camera x = −y_L, camera y = −z_L, camera z = +x_L.
    let base = UnitQuaternion::from_basis_unchecked(&[
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ]);
    let tilt = UnitQuaternion::from_euler_angles(10f64.to_radians(), 0.0, 0.0);
    Isometry3::from_parts(Translation3::new(0.05, -0.1, 0.03), base * tilt)
}

struct Aabb {
    lo: [f64; 3],
    hi: [f64; 3],
}

/// Room interior plus solid pillars; each surface has its own gray value.
struct Scene {
    room: Aabb,
    pillars: Vec<Aabb>,
}

fn scene() -> Scene {
    let pillar = |x: f64, y: f64, r: f64| Aabb {
        lo: [x - r, y - r, -1.5],
        hi: [x + r, y + r, 2.5],
    };
    Scene {
        room: Aabb {
            lo: [-5.0, -4.0, -1.5],
            hi: [5.0, 4.0, 2.5],
        },
        pillars: vec![
            pillar(2.5, 1.5, 0.25),
            pillar(-2.0, 2.5, 0.3),
            pillar(3.0, -2.0, 0.2),
            pillar(-3.0, -1.5, 0.35),
            pillar(0.5, 3.0, 0.25),
            pillar(0.0, -2.8, 0.3),
            pillar(-1.0, 0.5, 0.15),
        ],
    }
}

impl Scene {
    /// Distance along the unit ray and a gray value of the surface hit.
    fn cast(&self, o: &Vector3<f64>, d: &Vector3<f64>) -> Option<(f64, u8)> {
        // Inside the room: the nearest exit.
        let mut best: Option<(f64, u8)> = None;
        for i in 0..3 {
            if d[i].abs() < 1e-12 {
                continue;
            }
            let bound = if d[i] > 0.0 {
                self.room.hi[i]
            } else {
                self.room.lo[i]
            };
            let gray = 40 + 20 * (2 * i + usize::from(d[i] < 0.0)) as u8;
            let t = (bound - o[i]) / d[i];
            if t > 0.0 && best.is_none_or(|b| t < b.0) {
                best = Some((t, gray));
            }
        }
        for (k, b) in self.pillars.iter().enumerate() {
            let (mut t_in, mut t_out) = (f64::NEG_INFINITY, f64::INFINITY);
            for i in 0..3 {
                if d[i].abs() < 1e-12 {
                    if o[i] < b.lo[i] || o[i] > b.hi[i] {
                        t_in = f64::INFINITY;
                    }
                    continue;
                }
                let (a, c) = ((b.lo[i] - o[i]) / d[i], (b.hi[i] - o[i]) / d[i]);
                t_in = t_in.max(a.min(c));
                t_out = t_out.min(a.max(c));
            }
            if t_in <= t_out && t_in > 0.0 && best.is_none_or(|h| t_in < h.0) {
                best = Some((t_in, 190 + 9 * k as u8));
            }
        }
        best
    }
}

/// 16 rings, 0.5° azimuth steps, swept over 0.1 s; points in the LiDAR frame at their time.
fn lidar_frame(scene: &Scene, stamp: f64) -> Vec<LidarPoint> {
    let mut out = Vec::new();
    for j in 0..720 {
        let t_rel = 0.1 * f64::from(j) / 720.0;
        let pose = device_pose(stamp + t_rel);
        let az = (0.5 * f64::from(j)).to_radians();
        for ring in 0..16u16 {
            let el = (-24.0 + 3.0 * f64::from(ring)).to_radians();
            let d = Vector3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            let o = pose.translation.vector;
            if let Some((r, _)) = scene.cast(&o, &(pose.rotation * d)) {
                let p = d * r;
                out.push(LidarPoint {
                    xyz: [p.x as f32, p.y as f32, p.z as f32],
                    intensity: 1.0,
                    ring,
                    t_rel,
                });
            }
        }
    }
    out
}

/// Writes `calib.json` for one camera "left": fisheye whose `K` fits the image turned 90°
/// counter-clockwise (like the JMK6), and `transform_matrix` = LiDAR → camera.
fn write_calib(dir: &Path) {
    let m = lidar_cam().inverse().to_homogeneous();
    let rows: Vec<Vec<f64>> = (0..4)
        .map(|i| (0..4).map(|j| m[(i, j)]).collect())
        .collect();
    let json = serde_json::json!({
        "camera_info": {"left": {
            "K": [220.0, 0.0, 301.0, 0.0, 221.0, 398.0, 0.0, 0.0, 1.0],
            "camera_model": "fisheyeKB",
            "coeff": [0.01, -0.002, 0.0, 0.0],
            "image_width": IMAGE_W,
            "image_height": IMAGE_H
        }},
        "out_put": {"left": {"transform_matrix": rows}}
    });
    fs::create_dir_all(dir.join("calibration")).unwrap();
    fs::write(dir.join("calibration/calib.json"), json.to_string()).unwrap();
}

/// Renders the stored (landscape) image whose stamp is `stamp`; it was taken at
/// `stamp + TRUE_OFFSET`.
fn render(scene: &Scene, cam: &Camera, stamp: f64) -> Vec<u8> {
    let world_cam = device_pose(stamp + TRUE_OFFSET) * lidar_cam();
    let o = world_cam.translation.vector;
    let (w, h) = (usize::from(IMAGE_W), usize::from(IMAGE_H));
    let mut rgb = Vec::with_capacity(w * h * 3);
    for ys in 0..h {
        for xs in 0..w {
            // Stored pixel → pixel of the image turned 90° counter-clockwise.
            let uv = [ys as f64, (w - 1 - xs) as f64];
            let g = cam
                .unproject(uv)
                .and_then(|d| scene.cast(&o, &(world_cam.rotation * d)))
                .map_or(0, |(_, g)| g);
            rgb.extend([g; 3]);
        }
    }
    rgb
}

fn jpeg(rgb: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    jpeg_encoder::Encoder::new(&mut out, 95)
        .encode(rgb, IMAGE_W, IMAGE_H, jpeg_encoder::ColorType::Rgb)
        .unwrap();
    out
}

/// Scan folder with LiDAR, images and calibration, and a SLAM folder with the true trajectory.
fn synthetic_scan(dir: &Path) {
    let scene = scene();
    write_calib(dir);
    let mut w = BagWriter::new(Compression::None);
    let lc = w.add_connection("/lidar", "sensor_msgs/PointCloud2");
    let frames = (DURATION * 10.0).round() as usize;
    for k in 0..frames {
        let s = T0 + 0.1 * k as f64;
        w.write_message(
            lc,
            time(s),
            &encode_pointcloud2(s, "l", &lidar_frame(&scene, s)),
        );
    }
    fs::write(dir.join("LIDAR_0.bag"), w.finish()).unwrap();

    // The camera model of the turned image: `K` as written, width and height swapped.
    let as_written = CalibReading {
        rotation: ImageRotation::None,
        direction: MatrixDirection::LidarToCam,
    };
    let calib = load_calibration_as(&dir.join("calibration/calib.json"), as_written).unwrap();
    let mut turned = calib.cameras["left"].clone();
    std::mem::swap(&mut turned.width, &mut turned.height);
    let cam = Camera::from_calib("left", &turned).unwrap();
    let mut w = BagWriter::new(Compression::None);
    let ic = w.add_connection("/camera_left/image_raw", "sensor_msgs/CompressedImage");
    for k in 0..16 {
        let s = T0 + 0.7 + 0.3 * f64::from(k);
        let img = CompressedImage {
            stamp: s,
            format: "jpeg".into(),
            data: jpeg(&render(&scene, &cam, s)),
        };
        w.write_message(ic, time(s), &encode_compressed_image(&img));
    }
    fs::write(dir.join("IMAGE_0.bag"), w.finish()).unwrap();

    let poses: Vec<Pose> = (0..=frames)
        .map(|k| {
            let t = T0 + 0.1 * k as f64;
            let p = device_pose(t);
            Pose {
                t,
                p: p.translation.vector,
                q: p.rotation,
            }
        })
        .collect();
    fs::create_dir_all(dir.join("slam")).unwrap();
    write_tum(&dir.join("slam/traj.tum"), &poses).unwrap();
}

fn calib_check(dir: &Path, out: &Path) -> assert_cmd::assert::Assert {
    Command::cargo_bin("driftless")
        .unwrap()
        .arg("calib-check")
        .arg(dir)
        .arg("--slam")
        .arg(dir.join("slam"))
        .arg("-o")
        .arg(out)
        .args(["--images", "4"])
        .assert()
}

#[test]
fn calib_check_finds_synthetic_truth() {
    let dir = tmp("calib-check-truth");
    synthetic_scan(&dir);
    let out = dir.join("out");
    calib_check(&dir, &out)
        .success()
        .stdout(predicate::str::contains("Beste Variante"));

    let text = fs::read_to_string(out.join("calib_check.json")).unwrap();
    let json: serde_json::Value = serde_json::from_str(&text).unwrap();
    let best = &json["summary"]["best"];
    assert_eq!(best["image_rotation"], "ccw90", "{best}");
    assert_eq!(best["direction"], "lidar_to_cam", "{best}");
    assert_eq!(best["time_offset"], TRUE_OFFSET, "{best}");
    assert!(json["summary"]["margin_direction"].as_f64().unwrap() > 0.5);
    assert_eq!(json["images"].as_array().unwrap().len(), 4);
    assert_eq!(json["variants"].as_array().unwrap().len(), 4 * 2 * 41);
    assert_eq!(json["programm"]["version"], env!("CARGO_PKG_VERSION"));
    assert!(
        json["programm"]["commit"].is_string(),
        "{}",
        json["programm"]
    );
    for label in ["best", "worst"] {
        assert!(out.join(format!("overlay_{label}_left_01.png")).is_file());
    }

    // Same input → byte-identical result.
    let again = dir.join("again");
    calib_check(&dir, &again).success();
    assert_eq!(
        fs::read(again.join("calib_check.json")).unwrap(),
        text.as_bytes()
    );
}

#[test]
fn calib_check_without_images_errors() {
    let dir = tmp("calib-check-noimages");
    room_bag(&dir, true);
    calib_check(&dir, &dir.join("out"))
        .failure()
        .stderr(predicate::str::contains("keine Kamerabilder"));
}
