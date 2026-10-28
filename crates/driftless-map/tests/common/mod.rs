//! Synthetic test scenes for colouring: a room with coloured walls is rendered (ray cast
//! per pixel) from known camera poses and the images go through JPEG into a bag.

#![allow(dead_code)] // each test binary uses a different part

use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use driftless_io::msg::encode::encode_compressed_image;
use driftless_io::msg::CompressedImage;
use driftless_io::testdir::{SharedDir, TestDir};
use driftless_map::{Cloud, ColorCamera, ColorConfig, ImageSource};
pub use driftless_odom::testutil::{aabb, Aabb, Scene, Trajectory, Walk};
use driftless_traj::nalgebra::{
    Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3,
};
use driftless_traj::Pose;
use jpeg_encoder::{ColorType, Encoder, SamplingFactor};
use std::sync::OnceLock;

/// All scene times start here (bag and ROS stamps are unsigned).
pub const T0: f64 = 100.0;
/// Full image size; colouring decodes at half size (320 × 240) by default.
pub const W: u32 = 640;
pub const H: u32 = 480;
/// Colour tolerance per channel.
pub const TOL: i32 = 8;

pub const FLOOR: [u8; 3] = [200, 60, 60];
pub const CEILING: [u8; 3] = [60, 200, 60];
pub const WALLS: [[u8; 3]; 4] = [
    [60, 60, 200],  // −x
    [200, 200, 60], // +x
    [200, 60, 200], // −y
    [60, 200, 200], // +y
];
pub const BOX: [u8; 3] = [240, 140, 20];

/// A scene whose surfaces each have one colour.
pub struct Painted {
    pub scene: Scene,
    pub box_colors: Vec<[u8; 3]>,
}

impl Painted {
    pub fn new(boxes: Vec<(Aabb, [u8; 3])>) -> Painted {
        Painted {
            scene: Scene {
                room: aabb([-5.0, -4.0, -1.2], [7.0, 5.0, 1.8]),
                boxes: boxes.iter().map(|b| b.0).collect(),
            },
            box_colors: boxes.iter().map(|b| b.1).collect(),
        }
    }

    /// Colour of the surface point `p`.
    pub fn color_at(&self, p: &Vector3<f64>) -> [u8; 3] {
        let eps = 1e-4;
        for (b, c) in self.scene.boxes.iter().zip(&self.box_colors) {
            if (0..3).all(|i| p[i] >= b.min[i] - eps && p[i] <= b.max[i] + eps) {
                return *c;
            }
        }
        let r = &self.scene.room;
        let faces = [
            (p.x - r.min.x, WALLS[0]),
            (r.max.x - p.x, WALLS[1]),
            (p.y - r.min.y, WALLS[2]),
            (r.max.y - p.y, WALLS[3]),
            (p.z - r.min.z, FLOOR),
            (r.max.z - p.z, CEILING),
        ];
        faces
            .iter()
            .min_by(|a, b| a.0.abs().total_cmp(&b.0.abs()))
            .unwrap()
            .1
    }
}

/// Equidistant fisheye, ±80° across the width, looking along `forward` (body frame) with
/// image-down = body −z.
pub fn camera(name: &str, forward: Vector3<f64>) -> (Camera, Isometry3<f64>) {
    let f = f64::from(W) / 2.0 / 80f64.to_radians();
    let calib = CameraCalib {
        k: Matrix3::new(
            f,
            0.0,
            f64::from(W) / 2.0 - 0.5,
            0.0,
            f,
            f64::from(H) / 2.0 - 0.5,
            0.0,
            0.0,
            1.0,
        ),
        model: CameraModel::FisheyeKB { k: [0.0; 4] },
        width: W,
        height: H,
        lidar_to_cam: None,
    };
    let down = Vector3::new(0.0, 0.0, -1.0);
    let right = down.cross(&forward);
    let rot = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[right, down, forward]));
    let cam_in_body = Isometry3::from_parts(
        Translation3::new(0.02, 0.06 * forward.y, 0.05),
        UnitQuaternion::from_rotation_matrix(&rot),
    );
    (Camera::from_calib(name, &calib).unwrap(), cam_in_body)
}

/// The left (+y) and right (−y) camera, mounted via a LiDAR that is offset from the IMU.
pub fn cameras() -> Vec<ColorCamera> {
    let lidar_to_imu = Isometry3::translation(0.05, 0.0, 0.1);
    [
        ("left", Vector3::new(0.0, 1.0, 0.0)),
        ("right", Vector3::new(0.0, -1.0, 0.0)),
    ]
    .into_iter()
    .map(|(name, fwd)| {
        let (cam, cam_in_body) = camera(name, fwd);
        // What the calibration file holds: LiDAR points into the camera frame.
        let lidar_to_cam = (lidar_to_imu.inverse() * cam_in_body).inverse();
        let c = ColorCamera::new(cam, &lidar_to_cam, &lidar_to_imu);
        assert!((c.cam_in_body.to_homogeneous() - cam_in_body.to_homogeneous()).norm() < 1e-12);
        c
    })
    .collect()
}

/// Ray-cast image of `painted` from camera `cam` at world pose `world_cam`.
pub fn render(painted: &Painted, cam: &Camera, world_cam: &Isometry3<f64>) -> Vec<u8> {
    let o = world_cam.translation.vector;
    let mut rgb = Vec::with_capacity((W * H * 3) as usize);
    for v in 0..H {
        for u in 0..W {
            let c = cam
                .unproject([f64::from(u), f64::from(v)])
                .map_or([0; 3], |dir| {
                    let d = world_cam.rotation * dir;
                    let t = painted.scene.raycast(&o, &d);
                    painted.color_at(&(o + d * t))
                });
            rgb.extend(c);
        }
    }
    rgb
}

pub fn jpeg(rgb: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut enc = Encoder::new(&mut out, 95);
    enc.set_sampling_factor(SamplingFactor::R_4_4_4);
    enc.encode(rgb, W as u16, H as u16, ColorType::Rgb).unwrap();
    out
}

pub fn temp_dir(name: &str) -> TestDir {
    TestDir::new(format!("driftless-color-{}-{name}", std::process::id()))
}

/// An image to put in the bag.
pub struct Shot {
    pub camera: usize,
    pub stamp: f64,
    /// Changes the rendered pixels (to mark images that must not be used).
    pub edit: Box<dyn Fn(Vec<u8>) -> Vec<u8>>,
}

pub fn shot(camera: usize, stamp: f64) -> Shot {
    Shot {
        camera,
        stamp,
        edit: Box::new(|rgb| rgb),
    }
}

/// Renders `shots` (camera pose at the true trajectory time stamp + offset) into one bag,
/// `<name>/IMAGE_0.bag` in a folder shared by the tests of the test program (the images are
/// read from it later, also from data sets kept in a `static`): removed when the program exits
/// ([`SharedDir`]). `name` is one of its own per bag.
pub fn write_bag(
    name: &str,
    painted: &Painted,
    traj: &dyn Trajectory,
    cams: &[ColorCamera],
    shots: &[Shot],
) -> ImageSource {
    static BAGS: OnceLock<SharedDir> = OnceLock::new();
    let dir = BAGS.get_or_init(|| SharedDir::new("driftless-color-", "bilder"));
    let dir = dir.join(name);
    std::fs::create_dir_all(&dir).unwrap();
    let offset = ColorConfig::default().time_offset;
    let mut w = BagWriter::new(Compression::None);
    let conns: Vec<u32> = cams
        .iter()
        .map(|c| {
            let topic = format!("/camera_{}/image_raw", c.camera.name);
            w.add_connection(&topic, "sensor_msgs/CompressedImage")
        })
        .collect();
    for (i, s) in shots.iter().enumerate() {
        let cam = &cams[s.camera];
        let world_cam = traj.pose(s.stamp + offset) * cam.cam_in_body;
        let rgb = (s.edit)(render(painted, &cam.camera, &world_cam));
        let msg = encode_compressed_image(&CompressedImage {
            stamp: s.stamp,
            format: "jpeg".into(),
            data: jpeg(&rgb),
        });
        w.write_message(
            conns[s.camera],
            Time {
                sec: i as u32,
                nsec: 0,
            },
            &msg,
        );
    }
    let path = dir.join("IMAGE_0.bag");
    std::fs::write(&path, w.finish()).unwrap();
    ImageSource {
        bags: vec![path],
        topics: cams
            .iter()
            .map(|c| format!("/camera_{}/image_raw", c.camera.name))
            .collect(),
    }
}

/// Walk shifted to start at [`T0`].
pub struct Shifted<T>(pub T);

impl<T: Trajectory> Trajectory for Shifted<T> {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        self.0.pose(t - T0)
    }
}

pub fn walk() -> Shifted<Walk> {
    Shifted(Walk {
        still: 0.5,
        speed: 0.3,
    })
}

/// Body poses at 10 Hz on `[T0, T0 + duration]`.
pub fn poses(traj: &dyn Trajectory, duration: f64) -> Vec<Pose> {
    let n = (duration * 10.0).round() as usize;
    (0..=n)
        .map(|i| {
            let t = T0 + i as f64 * 0.1;
            let iso = traj.pose(t);
            Pose {
                t,
                p: iso.translation.vector,
                q: iso.rotation,
            }
        })
        .collect()
}

/// Points on every surface of the scene, 5 cm apart, with intensities 0..100.
pub fn surface_cloud(painted: &Painted) -> Cloud {
    let pts = painted.scene.surface_samples(0.05, 0.025);
    let n = pts.len();
    Cloud {
        xyz: pts
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect(),
        intensity: (0..n).map(|i| (i % 101) as f32).collect(),
        t: vec![T0; n],
        frame: vec![0; n],
        rgb: None,
        colored: None,
    }
}

pub fn close(a: [u8; 3], b: [u8; 3], tol: i32) -> bool {
    (0..3).all(|c| (i32::from(a[c]) - i32::from(b[c])).abs() <= tol)
}

/// Whether another colour lies within `r` of surface point `p` along an axis.
pub fn near_boundary(painted: &Painted, p: &Vector3<f64>, r: f64) -> bool {
    let c = painted.color_at(p);
    let room = &painted.scene.room;
    (0..3).any(|k| {
        [-r, r].into_iter().any(|o| {
            let mut q = *p;
            q[k] += o;
            (0..3).all(|j| q[j] >= room.min[j] && q[j] <= room.max[j]) && painted.color_at(&q) != c
        })
    })
}

/// Coloured points and how many of them lie within [`TOL`] of their surface colour.
#[derive(Debug, Default)]
pub struct Match {
    pub colored: usize,
    pub good: usize,
    /// The same for points at least 10 cm from a colour boundary (an image pixel there
    /// covers 2–6 cm, and JPEG and bilinear sampling mix the colours of a boundary).
    pub inner: usize,
    pub inner_good: usize,
}

pub fn color_match(painted: &Painted, cloud: &Cloud) -> Match {
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    let mut m = Match::default();
    for i in (0..cloud.len()).filter(|&i| colored[i]) {
        let p = Vector3::from(cloud.xyz[i].map(f64::from));
        let good = close(rgb[i], painted.color_at(&p), TOL);
        m.colored += 1;
        m.good += usize::from(good);
        if !near_boundary(painted, &p, 0.1) {
            m.inner += 1;
            m.inner_good += usize::from(good);
        }
    }
    m
}

impl Match {
    /// More than 95 % of all coloured points and 99 % of the inner ones match.
    pub fn assert_good(&self) {
        let share = |a: usize, b: usize| a as f64 / b.max(1) as f64;
        assert!(share(self.good, self.colored) > 0.95, "{self:?}");
        assert!(share(self.inner_good, self.inner) > 0.99, "{self:?}");
    }
}

pub fn walk_scene() -> Painted {
    Painted::new(vec![
        (aabb([2.0, 1.5, -1.2], [3.0, 2.5, 0.3]), BOX),
        (aabb([-3.5, -3.0, -1.2], [-2.5, -1.0, 1.8]), [120, 40, 160]),
    ])
}

/// Images every 0.5 s from both cameras during a 6 s walk through the room.
pub fn walk_shots() -> Vec<Shot> {
    (1..=11)
        .flat_map(|i| {
            [
                shot(0, T0 + 0.5 * f64::from(i)),
                shot(1, T0 + 0.5 * f64::from(i)),
            ]
        })
        .collect()
}

/// The walk data set, rendered once for all tests that use it.
pub fn walk_data() -> &'static (Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>) {
    static DATA: OnceLock<(Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>)> = OnceLock::new();
    DATA.get_or_init(|| {
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let images = write_bag("walk", &painted, &traj, &cams, &walk_shots());
        (painted, images, poses(&traj, 6.0), cams)
    })
}

/// Brightness added by [`with_highlight`] (sRGB levels).
pub const HIGHLIGHT: u8 = 120;

/// Width of the highlight's soft edge (pixels).
pub const HIGHLIGHT_EDGE: f64 = 24.0;

/// A bright spot that moves with the camera: +[`HIGHLIGHT`] in every channel inside a disc
/// below the image centre, where the cameras of [`cameras`] see the floor. Its edge fades
/// over [`HIGHLIGHT_EDGE`] like a real highlight's; a hard edge would make these images
/// the sharpest of the scan and so the best views of every point.
pub fn with_highlight(mut rgb: Vec<u8>) -> Vec<u8> {
    let (cx, cy, r) = (f64::from(W) / 2.0, 0.78 * f64::from(H), 0.16 * f64::from(W));
    for v in 0..H {
        for u in 0..W {
            let (du, dv) = (f64::from(u) - cx, f64::from(v) - cy);
            let s = ((r - (du * du + dv * dv).sqrt()) / HIGHLIGHT_EDGE).clamp(0.0, 1.0);
            let add = (f64::from(HIGHLIGHT) * s * s * (3.0 - 2.0 * s)).round() as u8;
            let i = ((v * W + u) * 3) as usize;
            for c in &mut rgb[i..i + 3] {
                *c = c.saturating_add(add);
            }
        }
    }
    rgb
}

/// Whether surface point `p` lies on the floor of `painted`'s room.
pub fn on_floor(painted: &Painted, p: &Vector3<f64>) -> bool {
    (p.z - painted.scene.room.min.z).abs() < 1e-3 && painted.color_at(p) == FLOOR
}
