//! Colouring from synthetic camera images: a room with coloured walls is rendered (ray cast
//! per pixel) from known camera poses, the images go through JPEG into a bag, and the
//! cloud's points on the surfaces must get the colour of their surface.

use driftless_io::bag::writer::{BagWriter, Compression};
use driftless_io::bag::Time;
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use driftless_io::msg::encode::encode_compressed_image;
use driftless_io::msg::CompressedImage;
use driftless_map::{colorize, Cloud, ColorCamera, ColorConfig, ColorStats, ImageSource, MapError};
use driftless_odom::testutil::{aabb, Aabb, Scene, Trajectory, Walk};
use driftless_traj::nalgebra::{
    Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3,
};
use driftless_traj::Pose;
use jpeg_encoder::{ColorType, Encoder, SamplingFactor};
use std::path::PathBuf;
use std::sync::OnceLock;

/// All scene times start here (bag and ROS stamps are unsigned).
const T0: f64 = 100.0;
/// Full image size; colouring decodes at half size (320 × 240) by default.
const W: u32 = 640;
const H: u32 = 480;
/// Colour tolerance per channel.
const TOL: i32 = 8;

const FLOOR: [u8; 3] = [200, 60, 60];
const CEILING: [u8; 3] = [60, 200, 60];
const WALLS: [[u8; 3]; 4] = [
    [60, 60, 200],  // −x
    [200, 200, 60], // +x
    [200, 60, 200], // −y
    [60, 200, 200], // +y
];
const BOX: [u8; 3] = [240, 140, 20];

/// A scene whose surfaces each have one colour.
struct Painted {
    scene: Scene,
    box_colors: Vec<[u8; 3]>,
}

impl Painted {
    fn new(boxes: Vec<(Aabb, [u8; 3])>) -> Painted {
        Painted {
            scene: Scene {
                room: aabb([-5.0, -4.0, -1.2], [7.0, 5.0, 1.8]),
                boxes: boxes.iter().map(|b| b.0).collect(),
            },
            box_colors: boxes.iter().map(|b| b.1).collect(),
        }
    }

    /// Colour of the surface point `p`.
    fn color_at(&self, p: &Vector3<f64>) -> [u8; 3] {
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
fn camera(name: &str, forward: Vector3<f64>) -> (Camera, Isometry3<f64>) {
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
fn cameras() -> Vec<ColorCamera> {
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
fn render(painted: &Painted, cam: &Camera, world_cam: &Isometry3<f64>) -> Vec<u8> {
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

/// Separable Gaussian blur with standard deviation `sigma` (pixels), edges clamped.
fn blur(rgb: &[u8], sigma: f64) -> Vec<u8> {
    let r = (3.0 * sigma).ceil() as i64;
    let kernel: Vec<f64> = (-r..=r)
        .map(|i| (-(i * i) as f64 / (2.0 * sigma * sigma)).exp())
        .collect();
    let norm: f64 = kernel.iter().sum();
    let (w, h) = (i64::from(W), i64::from(H));
    let pass = |src: &[f64], horizontal: bool| -> Vec<f64> {
        let mut out = vec![0.0; src.len()];
        for y in 0..h {
            for x in 0..w {
                for c in 0..3 {
                    let mut s = 0.0;
                    for (k, kv) in kernel.iter().enumerate() {
                        let o = k as i64 - r;
                        let (sx, sy) = if horizontal {
                            ((x + o).clamp(0, w - 1), y)
                        } else {
                            (x, (y + o).clamp(0, h - 1))
                        };
                        s += kv * src[((sy * w + sx) * 3 + c) as usize];
                    }
                    out[((y * w + x) * 3 + c) as usize] = s / norm;
                }
            }
        }
        out
    };
    let src: Vec<f64> = rgb.iter().map(|&v| f64::from(v)).collect();
    pass(&pass(&src, true), false)
        .iter()
        .map(|v| v.round() as u8)
        .collect()
}

fn jpeg(rgb: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut enc = Encoder::new(&mut out, 95);
    enc.set_sampling_factor(SamplingFactor::R_4_4_4);
    enc.encode(rgb, W as u16, H as u16, ColorType::Rgb).unwrap();
    out
}

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-color-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// An image to put in the bag.
struct Shot {
    camera: usize,
    stamp: f64,
    /// Changes the rendered pixels (to mark images that must not be used).
    edit: fn(Vec<u8>) -> Vec<u8>,
}

fn shot(camera: usize, stamp: f64) -> Shot {
    Shot {
        camera,
        stamp,
        edit: |rgb| rgb,
    }
}

/// Renders `shots` (camera pose at the true trajectory time stamp + offset) into one bag.
fn write_bag(
    name: &str,
    painted: &Painted,
    traj: &dyn Trajectory,
    cams: &[ColorCamera],
    shots: &[Shot],
) -> ImageSource {
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
    let path = temp_dir(name).join("IMAGE_0.bag");
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
struct Shifted<T>(T);

impl<T: Trajectory> Trajectory for Shifted<T> {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        self.0.pose(t - T0)
    }
}

fn walk() -> Shifted<Walk> {
    Shifted(Walk {
        still: 0.5,
        speed: 0.3,
    })
}

/// Body poses at 10 Hz on `[T0, T0 + duration]`.
fn poses(traj: &dyn Trajectory, duration: f64) -> Vec<Pose> {
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
fn surface_cloud(painted: &Painted) -> Cloud {
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

fn close(a: [u8; 3], b: [u8; 3], tol: i32) -> bool {
    (0..3).all(|c| (i32::from(a[c]) - i32::from(b[c])).abs() <= tol)
}

/// Whether another colour lies within `r` of surface point `p` along an axis.
fn near_boundary(painted: &Painted, p: &Vector3<f64>, r: f64) -> bool {
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
struct Match {
    colored: usize,
    good: usize,
    /// The same for points at least 10 cm from a colour boundary (an image pixel there
    /// covers 2–6 cm, and JPEG and bilinear sampling mix the colours of a boundary).
    inner: usize,
    inner_good: usize,
}

fn color_match(painted: &Painted, cloud: &Cloud) -> Match {
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
    fn assert_good(&self) {
        let share = |a: usize, b: usize| a as f64 / b.max(1) as f64;
        assert!(share(self.good, self.colored) > 0.95, "{self:?}");
        assert!(share(self.inner_good, self.inner) > 0.99, "{self:?}");
    }
}

fn walk_scene() -> Painted {
    Painted::new(vec![
        (aabb([2.0, 1.5, -1.2], [3.0, 2.5, 0.3]), BOX),
        (aabb([-3.5, -3.0, -1.2], [-2.5, -1.0, 1.8]), [120, 40, 160]),
    ])
}

/// Images every 0.5 s from both cameras during a 6 s walk through the room.
fn walk_shots() -> Vec<Shot> {
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
fn walk_data() -> &'static (Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>) {
    static DATA: OnceLock<(Painted, ImageSource, Vec<Pose>, Vec<ColorCamera>)> = OnceLock::new();
    DATA.get_or_init(|| {
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let images = write_bag("walk", &painted, &traj, &cams, &walk_shots());
        (painted, images, poses(&traj, 6.0), cams)
    })
}

#[test]
fn colors_match_rendered_walls() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    let stats = colorize(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
    )
    .unwrap();
    assert_eq!(stats.images_used, 22, "{stats:?}");
    assert_eq!(
        stats.images_bad + stats.images_blurry + stats.images_outside_trajectory,
        0
    );
    let m = color_match(painted, &cloud);
    println!("{stats:?}: {m:?}");
    assert_eq!(m.colored, stats.points_colored);
    assert!(
        m.colored as f64 > 0.5 * cloud.len() as f64,
        "{m:?} of {}",
        cloud.len()
    );
    m.assert_good();
}

#[test]
fn occluded_points_take_no_color_from_occluder() {
    // A box 0.44 m in front of the left camera (standing still); the strip of the +y wall
    // behind it is never visible. Its points (5 cm apart) are 2–3 depth-buffer cells apart
    // in the image, so the box only hides the wall if each point covers its footprint.
    let painted = Painted::new(vec![(aabb([-0.5, 0.5, -1.2], [0.5, 0.9, 0.6]), BOX)]);
    let still = Shifted(Walk {
        still: 100.0,
        speed: 0.0,
    });
    let cams = cameras();
    let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
    let images = write_bag("occluded", &painted, &still, &cams, &shots);
    let poses = poses(&still, 3.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_used, 2);

    let cam = still.pose(T0 + 1.0) * cams[0].cam_in_body;
    let o = cam.translation.vector;
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    let (mut hidden, mut wrong) = (0, 0);
    for i in 0..cloud.len() {
        let p = Vector3::from(cloud.xyz[i].map(f64::from));
        if p.y < 4.99 {
            continue; // only the +y wall
        }
        let to = p - o;
        let in_view = (cam.inverse() * driftless_traj::nalgebra::Point3::from(p))
            .coords
            .normalize()
            .z
            > 0.5;
        if in_view && painted.scene.raycast(&o, &to.normalize()) < to.norm() - 0.05 {
            hidden += 1;
            if colored[i] && close(rgb[i], BOX, 40) {
                wrong += 1;
            }
        }
    }
    println!("{stats:?}: {wrong} of {hidden} hidden wall points in box colour");
    assert!(hidden > 200, "{hidden}");
    assert_eq!(
        wrong, 0,
        "{wrong} of {hidden} hidden wall points in box colour"
    );
    // The box and the visible wall are coloured right.
    color_match(&painted, &cloud).assert_good();
}

/// Inverted colours: images that must not be used.
fn invert(rgb: Vec<u8>) -> Vec<u8> {
    rgb.into_iter().map(|v| 255 - v).collect()
}

/// Brief, fast turn about z (3 rad/s for 0.3 s) on top of the walk.
struct WalkWithTurn(Shifted<Walk>);

const TURN: (f64, f64) = (T0 + 3.1, T0 + 3.4);

impl Trajectory for WalkWithTurn {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        let yaw = 3.0 * (t.clamp(TURN.0, TURN.1) - TURN.0);
        self.0.pose(t) * UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
    }
}

#[test]
fn blurry_and_fast_images_are_skipped() {
    let painted = walk_scene();
    let cams = cameras();
    let traj = WalkWithTurn(walk());
    let mut shots: Vec<Shot> = (1..=11).map(|i| shot(0, T0 + 0.5 * f64::from(i))).collect();
    shots.push(Shot {
        camera: 0,
        stamp: T0 + 2.25,
        edit: |rgb| blur(&invert(rgb), 6.0),
    });
    shots.push(Shot {
        camera: 0,
        stamp: T0 + 3.22,
        edit: invert,
    });
    let images = write_bag("blurry", &painted, &traj, &cams, &shots);
    let poses = poses(&traj, 6.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_blurry, 2, "{stats:?}");
    assert_eq!(stats.images_used, 11, "{stats:?}");
    // Had the inverted images been used, their colours would show.
    color_match(&painted, &cloud).assert_good();
}

#[test]
fn images_outside_trajectory_skipped() {
    let painted = walk_scene();
    let cams = cameras();
    let traj = walk();
    // The trajectory covers T0 … T0 + 6; with the +0.03 s offset an image stamped
    // T0 − 0.02 is taken at T0 + 0.01 (inside), T0 − 0.04 at T0 − 0.01 (outside), and
    // T0 + 5.96 at T0 + 5.99 (inside).
    let shots = [
        shot(0, T0 - 1.0),
        shot(0, T0 - 0.04),
        shot(0, T0 - 0.02),
        shot(1, T0 + 3.0),
        shot(1, T0 + 5.96),
        shot(1, T0 + 7.0),
    ];
    let images = write_bag("outside", &painted, &traj, &cams, &shots);
    let poses = poses(&traj, 6.0);
    let mut cloud = surface_cloud(&painted);
    let stats = colorize(&mut cloud, images, &poses, &cams, &ColorConfig::default()).unwrap();
    assert_eq!(stats.images_outside_trajectory, 3, "{stats:?}");
    assert_eq!(stats.images_used, 3, "{stats:?}");
    assert!(stats.points_colored > 0);
}

#[test]
fn uncolored_points_are_gray_not_black() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    // Points far outside the room, beyond max_dist of every camera.
    let n0 = cloud.len();
    for i in 0..50 {
        cloud.xyz.push([50.0 + i as f32 * 0.1, 0.0, 0.0]);
        cloud.intensity.push(i as f32);
        cloud.t.push(T0);
        cloud.frame.push(0);
    }
    let stats = colorize(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
    )
    .unwrap();
    let rgb = cloud.rgb.as_ref().unwrap();
    let colored = cloud.colored.as_ref().unwrap();
    assert_eq!(stats.points_colored + stats.points_uncolored, cloud.len());
    for i in n0..cloud.len() {
        assert!(!colored[i]);
        let [r, g, b] = rgb[i];
        assert!(r == g && g == b && r >= 64, "{:?}", rgb[i]);
    }
    for i in (0..cloud.len()).filter(|&i| !colored[i]) {
        assert_ne!(rgb[i], [0, 0, 0]);
    }
    // Without images every point is gray, brighter with higher intensity.
    let empty = ImageSource {
        bags: vec![],
        topics: images.topics.clone(),
    };
    let stats = colorize(&mut cloud, empty, poses, cams, &ColorConfig::default()).unwrap();
    assert_eq!(
        stats,
        ColorStats {
            points_uncolored: cloud.len(),
            ..ColorStats::default()
        }
    );
    let rgb = cloud.rgb.as_ref().unwrap();
    assert!(cloud.colored.as_ref().unwrap().iter().all(|&c| !c));
    assert!(rgb[n0 + 10][0] < rgb[n0 + 40][0]);
    assert!(rgb
        .iter()
        .all(|c| c[0] == c[1] && c[1] == c[2] && c[0] >= 64));
}

#[test]
fn colorize_is_deterministic() {
    let (painted, images, poses, cams) = walk_data();
    let run = || {
        let mut cloud = surface_cloud(painted);
        let stats = colorize(
            &mut cloud,
            images.clone(),
            poses,
            cams,
            &ColorConfig::default(),
        )
        .unwrap();
        (cloud, stats)
    };
    let (a, sa) = run();
    let (b, sb) = run();
    assert_eq!(sa, sb);
    assert_eq!(a.rgb, b.rgb);
    assert_eq!(a.colored, b.colored);
}

#[test]
fn colorize_rejects_bad_config_and_cameras() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    let bad = [
        ColorConfig {
            downscale: 3,
            ..ColorConfig::default()
        },
        ColorConfig {
            views_per_point: 0,
            ..ColorConfig::default()
        },
        ColorConfig {
            max_dist: 0.0,
            ..ColorConfig::default()
        },
        ColorConfig {
            zbuffer_scale: 0,
            ..ColorConfig::default()
        },
        ColorConfig {
            max_angular_rate: f64::NAN,
            ..ColorConfig::default()
        },
    ];
    for cfg in &bad {
        let r = colorize(&mut cloud, images.clone(), poses, cams, cfg);
        assert!(matches!(r, Err(MapError::Config(_))), "{cfg:?}");
    }
    let twice = [cams[0].clone(), cams[0].clone()];
    let r = colorize(
        &mut cloud,
        images.clone(),
        poses,
        &twice,
        &ColorConfig::default(),
    );
    assert!(matches!(r, Err(MapError::Config(_))));
    let missing = ImageSource {
        bags: vec![temp_dir("missing").join("nope.bag")],
        topics: images.topics.clone(),
    };
    let r = colorize(&mut cloud, missing, poses, cams, &ColorConfig::default());
    assert!(matches!(r, Err(MapError::Images(_))));
}
