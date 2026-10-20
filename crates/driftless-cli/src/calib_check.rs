//! `driftless calib-check <scan> --slam <ziel> -o <ordner> [--images N]`: test projection of
//! the LiDAR points into camera images to find the direction of the LiDAR ↔ camera matrix
//! (`out_put.<cam>.transform_matrix`) and the time offset of the camera stamps.
//!
//! Per camera, the sharpest image in each of N equal time bins is used. The LiDAR points of
//! ±[`POINT_WINDOW`] s around each image are put into the world with the corrected trajectory
//! (`traj.tum` of `driftless slam`, interpolated per point, which also deskews them) and
//! projected into the image for every image orientation, both readings of the matrix and
//! every time offset of [`OFFSET_STEPS`]. Score of a variant = mean normalised image gradient
//! at the projected, visible depth-discontinuity points (points whose neighbour along the
//! LiDAR scan line is more than [`DEPTH_JUMP`] further away); the right calibration puts them
//! on image edges. Writes `calib_check.json`
//! and overlay images (PNG) of the best and the worst variant.

use crate::scan_input::{parse_extrinsic, with_lidar_frames};
use crate::topics::file_name;
use driftless_io::bag::BagReader;
use driftless_io::calib::{load_calibration_as, CalibReading, ImageRotation, MatrixDirection};
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, RgbImage};
use driftless_io::msg::LidarFrame;
use driftless_io::project::discover_project;
use driftless_odom::deskew::point_time_base;
use driftless_traj::nalgebra::{Isometry3, Point3, Vector3};
use driftless_traj::{interpolate_pose, read_tum};
use rayon::prelude::*;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

const IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";
/// Time offsets tested: `step / 100` s for every step (−0.20 … +0.20 s).
pub const OFFSET_STEPS: std::ops::RangeInclusive<i32> = -20..=20;
/// LiDAR points within this time (s) of an image are projected into it.
pub const POINT_WINDOW: f64 = 0.5;
/// Points closer than this to the LiDAR (m) are dropped (device, operator's hand).
const MIN_RANGE: f64 = 0.5;
/// Points further than this (m) are dropped.
const MAX_RANGE: f64 = 30.0;
/// Depth jump (m) to a neighbour that makes a point a depth-discontinuity point.
pub const DEPTH_JUMP: f64 = 0.3;
/// Side of a depth-buffer cell in working-resolution pixels.
const DEPTH_CELL: usize = 2;
/// A point is visible if its range is at most `cell minimum · (1 + rel) + abs`.
const VISIBLE_ABS: f64 = 0.05;
const VISIBLE_REL: f64 = 0.02;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag, IMAGE_*.bag und calibration/calib.json)
    scan: PathBuf,
    /// Zielordner von `driftless slam` (liest traj.tum)
    #[arg(long)]
    slam: PathBuf,
    /// Zielordner für calib_check.json und die Überlagerungsbilder
    #[arg(short = 'o', long = "out")]
    out: PathBuf,
    /// Anzahl Bilder je Kamera, gleichmäßig über den Scan verteilt (je Abschnitt das schärfste)
    #[arg(long, default_value_t = 12)]
    images: usize,
    /// LiDAR-Lage im IMU-Rahmen: x y z qx qy qz qw (Standard: Identität, wie bei `slam`)
    #[arg(long, num_args = 7, value_names = ["X", "Y", "Z", "QX", "QY", "QZ", "QW"], allow_negative_numbers = true)]
    lidar_to_imu: Option<Vec<f64>>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
}

/// Every reading of `calib.json` that is tested.
fn readings() -> Vec<CalibReading> {
    let rotations = [
        ImageRotation::None,
        ImageRotation::Cw90,
        ImageRotation::Ccw90,
        ImageRotation::R180,
    ];
    let directions = [MatrixDirection::LidarToCam, MatrixDirection::CamToLidar];
    rotations
        .into_iter()
        .flat_map(|rotation| {
            directions.into_iter().map(move |direction| CalibReading {
                rotation,
                direction,
            })
        })
        .collect()
}

/// One tested calibration: matrix reading and time offset.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Variant {
    /// Index into [`readings`].
    reading: usize,
    step: i32,
}

impl Variant {
    fn offset(self) -> f64 {
        f64::from(self.step) / 100.0
    }
}

fn variants(readings: usize) -> Vec<Variant> {
    (0..readings)
        .flat_map(|reading| OFFSET_STEPS.map(move |step| Variant { reading, step }))
        .collect()
}

/// Downscale factor for decoding: the working resolution keeps about 1000 px width.
fn working_downscale(width: u32) -> u32 {
    match width {
        w if w >= 3200 => 4,
        w if w >= 1600 => 2,
        _ => 1,
    }
}

/// The camera model for images decoded with block averaging by `d` (pixel centres at
/// integer coordinates, partial blocks dropped).
fn scaled_camera(cam: &Camera, d: u32) -> Camera {
    let s = f64::from(d);
    Camera {
        width: cam.width / d,
        height: cam.height / d,
        fx: cam.fx / s,
        fy: cam.fy / s,
        cx: (cam.cx + 0.5) / s - 0.5,
        cy: (cam.cy + 0.5) / s - 0.5,
        ..cam.clone()
    }
}

/// Gray image with the gradient magnitude normalised to mean 1.
struct Gradient {
    width: usize,
    height: usize,
    gray: Vec<u8>,
    grad: Vec<f32>,
}

fn to_gray(img: &RgbImage) -> Vec<u8> {
    img.data
        .as_chunks::<3>()
        .0
        .iter()
        .map(|p| {
            ((77 * u32::from(p[0]) + 150 * u32::from(p[1]) + 29 * u32::from(p[2]) + 128) >> 8) as u8
        })
        .collect()
}

/// 3x3 box blur, edges clamped.
fn box_blur(src: &[f32], w: usize, h: usize) -> Vec<f32> {
    let at = |x: isize, y: isize| {
        let x = x.clamp(0, w as isize - 1) as usize;
        let y = y.clamp(0, h as isize - 1) as usize;
        src[y * w + x]
    };
    let mut out = vec![0.0f32; w * h];
    for y in 0..h as isize {
        for x in 0..w as isize {
            let mut s = 0.0;
            for dy in -1..=1 {
                for dx in -1..=1 {
                    s += at(x + dx, y + dy);
                }
            }
            out[y as usize * w + x as usize] = s / 9.0;
        }
    }
    out
}

/// Sobel gradient magnitude, edges clamped.
fn sobel(src: &[f32], w: usize, h: usize) -> Vec<f32> {
    let at = |x: isize, y: isize| {
        let x = x.clamp(0, w as isize - 1) as usize;
        let y = y.clamp(0, h as isize - 1) as usize;
        src[y * w + x]
    };
    let mut out = vec![0.0f32; w * h];
    for y in 0..h as isize {
        for x in 0..w as isize {
            let gx = at(x + 1, y - 1) + 2.0 * at(x + 1, y) + at(x + 1, y + 1)
                - at(x - 1, y - 1)
                - 2.0 * at(x - 1, y)
                - at(x - 1, y + 1);
            let gy = at(x - 1, y + 1) + 2.0 * at(x, y + 1) + at(x + 1, y + 1)
                - at(x - 1, y - 1)
                - 2.0 * at(x, y - 1)
                - at(x + 1, y - 1);
            out[y as usize * w + x as usize] = gx.hypot(gy);
        }
    }
    out
}

fn mean(v: &[f32]) -> f64 {
    v.iter().map(|&x| f64::from(x)).sum::<f64>() / v.len().max(1) as f64
}

/// Image sharpness: mean Sobel magnitude of the gray image.
fn sharpness(img: &RgbImage) -> f64 {
    let gray: Vec<f32> = to_gray(img).into_iter().map(f32::from).collect();
    mean(&sobel(&gray, img.width as usize, img.height as usize))
}

/// Gray → blur → Sobel → blur, normalised to mean 1 (so images of different contrast count
/// the same; a score of 1 is what randomly placed points get).
fn gradient(img: &RgbImage) -> Gradient {
    let (w, h) = (img.width as usize, img.height as usize);
    let gray = to_gray(img);
    let g: Vec<f32> = gray.iter().map(|&v| f32::from(v)).collect();
    let mut grad = box_blur(&sobel(&box_blur(&g, w, h), w, h), w, h);
    let m = mean(&grad);
    if m > 0.0 {
        grad.iter_mut()
            .for_each(|v| *v = (f64::from(*v) / m) as f32);
    }
    Gradient {
        width: w,
        height: h,
        gray,
        grad,
    }
}

impl Gradient {
    /// Bilinear sample at pixel coordinates (pixel centres at integers).
    fn sample(&self, u: f64, v: f64) -> f64 {
        let (w, h) = (self.width, self.height);
        let x = u.clamp(0.0, (w - 1) as f64);
        let y = v.clamp(0.0, (h - 1) as f64);
        let (x0, y0) = (x.floor() as usize, y.floor() as usize);
        let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
        let (fx, fy) = (x - x0 as f64, y - y0 as f64);
        let g = |x: usize, y: usize| f64::from(self.grad[y * w + x]);
        (g(x0, y0) * (1.0 - fx) + g(x1, y0) * fx) * (1.0 - fy)
            + (g(x0, y1) * (1.0 - fx) + g(x1, y1) * fx) * fy
    }
}

/// A projected point that is visible in the image.
struct ViewPoint {
    uv: [f64; 2],
    range: f64,
    edge: bool,
}

/// Projects world points into a camera at `world_cam` and keeps the visible ones (depth
/// buffer: at most `VISIBLE_*` behind the nearest point of their cell). Pure function of its
/// inputs.
fn project_view(points: &[WorldPoint], cam: &Camera, world_cam: &Isometry3<f64>) -> Vec<ViewPoint> {
    let cam_world = world_cam.inverse();
    let (w, h) = (cam.width as usize, cam.height as usize);
    let gw = w.div_ceil(DEPTH_CELL);
    let gh = h.div_ceil(DEPTH_CELL);
    let projected: Vec<(ViewPoint, usize)> = points
        .iter()
        .filter_map(|wp| {
            let p = wp.p.map(f64::from);
            let pc = cam_world * Point3::new(p[0], p[1], p[2]);
            let uv = cam.project(&pc.coords)?;
            let (x, y) = (uv[0].max(0.0) as usize, uv[1].max(0.0) as usize);
            let cell = (y / DEPTH_CELL).min(gh - 1) * gw + (x / DEPTH_CELL).min(gw - 1);
            let range = pc.coords.norm();
            Some((
                ViewPoint {
                    uv,
                    range,
                    edge: wp.edge,
                },
                cell,
            ))
        })
        .collect();
    let mut depth = vec![f64::INFINITY; gw * gh];
    for (p, c) in &projected {
        if p.range < depth[*c] {
            depth[*c] = p.range;
        }
    }
    projected
        .into_iter()
        .filter(|(p, c)| p.range <= depth[*c] * (1.0 + VISIBLE_REL) + VISIBLE_ABS)
        .map(|(p, _)| p)
        .collect()
}

/// Sum of the normalised gradient at the edge points and their count.
fn score_view(view: &[ViewPoint], grad: &Gradient) -> (f64, u64) {
    view.iter().filter(|p| p.edge).fold((0.0, 0), |(s, n), p| {
        (s + grad.sample(p.uv[0], p.uv[1]), n + 1)
    })
}

/// Gray image with the visible points coloured by range (red near → blue far); edge points
/// drawn as 3x3 squares.
fn overlay(grad: &Gradient, view: &[ViewPoint]) -> Vec<u8> {
    let (w, h) = (grad.width, grad.height);
    let mut rgb: Vec<u8> = grad.gray.iter().flat_map(|&g| [g / 2; 3]).collect();
    for p in view {
        let c = depth_color(p.range);
        let r: isize = if p.edge { 1 } else { 0 };
        let (x, y) = (p.uv[0].round() as isize, p.uv[1].round() as isize);
        for dy in -r..=r {
            for dx in -r..=r {
                let (px, py) = (x + dx, y + dy);
                if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                    let i = (py as usize * w + px as usize) * 3;
                    rgb[i..i + 3].copy_from_slice(&c);
                }
            }
        }
    }
    rgb
}

/// Hue from red (≤ 0.5 m) to blue (≥ 10 m).
fn depth_color(range: f64) -> [u8; 3] {
    let t = ((range - 0.5) / 9.5).clamp(0.0, 1.0);
    let hue = 240.0 * t; // degrees
    let x = 1.0 - ((hue / 60.0) % 2.0 - 1.0).abs();
    let (r, g, b) = match (hue / 60.0) as u32 {
        0 => (1.0, x, 0.0),
        1 => (x, 1.0, 0.0),
        2 => (0.0, 1.0, x),
        _ => (0.0, x, 1.0),
    };
    [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
}

fn write_png(path: &Path, w: usize, h: usize, rgb: &[u8]) -> Result<(), String> {
    let err = |e: &dyn std::fmt::Display| format!("{} nicht schreibbar: {e}", path.display());
    let file = fs::File::create(path).map_err(|e| err(&e))?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w as u32, h as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    let mut writer = enc.write_header().map_err(|e| err(&e))?;
    writer.write_image_data(rgb).map_err(|e| err(&e))?;
    writer.finish().map_err(|e| err(&e))
}

/// An image of the first pass.
struct Candidate {
    /// Position in the image stream.
    index: usize,
    camera: String,
    stamp: f64,
    sharpness: f64,
}

/// Per camera, the sharpest image in each of `n` equal time bins (ties: the earlier one).
fn select_images(candidates: &[Candidate], n: usize) -> Vec<usize> {
    let cameras: BTreeSet<&str> = candidates.iter().map(|c| c.camera.as_str()).collect();
    let mut chosen = Vec::new();
    for cam in cameras {
        let own: Vec<&Candidate> = candidates.iter().filter(|c| c.camera == cam).collect();
        let lo = own.iter().map(|c| c.stamp).fold(f64::INFINITY, f64::min);
        let hi = own
            .iter()
            .map(|c| c.stamp)
            .fold(f64::NEG_INFINITY, f64::max);
        let span = (hi - lo).max(1e-9);
        let mut best: Vec<Option<&Candidate>> = vec![None; n];
        for c in own {
            let bin = (((c.stamp - lo) / span * n as f64) as usize).min(n - 1);
            if best[bin].is_none_or(|b| c.sharpness > b.sharpness) {
                best[bin] = Some(c);
            }
        }
        chosen.extend(best.into_iter().flatten().map(|c| c.index));
    }
    chosen.sort_unstable();
    chosen
}

/// A camera at working resolution under one reading of the calibration.
struct CameraSetup {
    camera: Camera,
    /// Camera pose in the LiDAR frame.
    lidar_cam: Isometry3<f64>,
}

/// A LiDAR point of one frame with its capture time.
struct ScanPoint {
    xyz: Vector3<f64>,
    t: f64,
    /// A scan neighbour (same ring, adjacent in time) is more than [`DEPTH_JUMP`] further away.
    edge: bool,
}

/// The points of a frame within [`MIN_RANGE`]..[`MAX_RANGE`], with absolute times (as
/// [`driftless_odom::deskew::frame_points`] computes them) and depth-discontinuity flags. Neighbours are taken along
/// each ring in time order, so the flags do not depend on the camera; projected into the
/// image, a scan neighbour is also the image neighbour along the scan line.
fn scan_points(frame: &LidarFrame) -> Vec<ScanPoint> {
    let base = point_time_base(frame);
    let mut rings: BTreeMap<u16, Vec<ScanPoint>> = BTreeMap::new();
    for lp in &frame.points {
        if !lp.t_rel.is_finite() {
            continue;
        }
        let xyz = Vector3::new(
            f64::from(lp.xyz[0]),
            f64::from(lp.xyz[1]),
            f64::from(lp.xyz[2]),
        );
        if !(MIN_RANGE..=MAX_RANGE).contains(&xyz.norm()) {
            continue;
        }
        rings.entry(lp.ring).or_default().push(ScanPoint {
            xyz,
            t: frame.stamp + (lp.t_rel - base).max(0.0),
            edge: false,
        });
    }
    let mut out = Vec::new();
    for (_, mut ring) in rings {
        // Stable: points with equal times keep their message order.
        ring.sort_by(|a, b| a.t.total_cmp(&b.t));
        let range: Vec<f64> = ring.iter().map(|p| p.xyz.norm()).collect();
        for (i, p) in ring.iter_mut().enumerate() {
            let farther = |j: usize| range[j] > range[i] + DEPTH_JUMP;
            p.edge = (i > 0 && farther(i - 1)) || (i + 1 < range.len() && farther(i + 1));
        }
        out.extend(ring);
    }
    out
}

/// A LiDAR point in the world frame.
struct WorldPoint {
    p: [f32; 3],
    /// Depth-discontinuity point (see [`ScanPoint::edge`]).
    edge: bool,
}

/// One image used for scoring.
struct Shot {
    camera: String,
    stamp: f64,
    sharpness: f64,
    grad: Gradient,
    points: Vec<WorldPoint>,
}

#[derive(Serialize)]
struct ImageReport {
    camera: String,
    stamp: f64,
    sharpness: f64,
    lidar_points: usize,
}

#[derive(Serialize, Clone)]
struct Score {
    image_rotation: ImageRotation,
    direction: MatrixDirection,
    time_offset: f64,
    score: f64,
    edge_points: u64,
}

#[derive(Serialize)]
struct VariantReport {
    #[serde(flatten)]
    total: Score,
    per_camera: BTreeMap<String, Score>,
}

#[derive(Serialize)]
struct Summary {
    best: Score,
    second_best: Score,
    best_other_direction: Score,
    best_other_rotation: Score,
    worst: Score,
    /// `best.score - second_best.score`
    margin_second: f64,
    /// `best.score - best_other_direction.score`
    margin_direction: f64,
}

#[derive(Serialize)]
struct Report {
    images: Vec<ImageReport>,
    summary: Summary,
    per_camera_best: BTreeMap<String, Score>,
    variants: Vec<VariantReport>,
}

fn make_score(v: Variant, readings: &[CalibReading], (sum, n): (f64, u64)) -> Score {
    Score {
        image_rotation: readings[v.reading].rotation,
        direction: readings[v.reading].direction,
        time_offset: v.offset(),
        score: if n == 0 { 0.0 } else { sum / n as f64 },
        edge_points: n,
    }
}

/// Index of the best score (ties: the first one) among `idx`.
fn best_of(scores: &[Score], idx: impl Iterator<Item = usize>) -> usize {
    idx.fold(None, |best: Option<usize>, i| match best {
        Some(b) if scores[b].score >= scores[i].score => Some(b),
        _ => Some(i),
    })
    .expect("at least one variant")
}

fn summarise(scores: &[Score]) -> (Summary, usize, usize) {
    let all = 0..scores.len();
    let best = best_of(scores, all.clone());
    let second = best_of(scores, all.clone().filter(|&i| i != best));
    let other = best_of(
        scores,
        all.clone()
            .filter(|&i| scores[i].direction != scores[best].direction),
    );
    let other_rotation = best_of(
        scores,
        all.clone()
            .filter(|&i| scores[i].image_rotation != scores[best].image_rotation),
    );
    let worst = all
        .fold(None, |w: Option<usize>, i| match w {
            Some(w) if scores[w].score <= scores[i].score => Some(w),
            _ => Some(i),
        })
        .expect("at least one variant");
    let summary = Summary {
        best: scores[best].clone(),
        second_best: scores[second].clone(),
        best_other_direction: scores[other].clone(),
        best_other_rotation: scores[other_rotation].clone(),
        worst: scores[worst].clone(),
        margin_second: scores[best].score - scores[second].score,
        margin_direction: scores[best].score - scores[other].score,
    };
    (summary, best, worst)
}

/// All `sensor_msgs/CompressedImage` topics of the image bags, sorted.
fn image_topics(bags: &[PathBuf]) -> Result<Vec<String>, String> {
    let mut topics = BTreeSet::new();
    for bag in bags {
        let reader = BagReader::open(bag).map_err(|e| format!("{}: {e}", file_name(bag)))?;
        topics.extend(
            reader
                .connections()
                .iter()
                .filter(|c| c.msg_type == IMAGE_TYPE)
                .map(|c| c.topic.clone()),
        );
    }
    Ok(topics.into_iter().collect())
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if args.images == 0 {
        return Err("--images muss mindestens 1 sein".into());
    }
    let lidar_to_imu = parse_extrinsic(args.lidar_to_imu.as_deref())?;
    let project = discover_project(&args.scan)?;
    if project.image_bags.is_empty() {
        return Err(format!(
            "keine Kamerabilder: {} enthält keine IMAGE_*.bag",
            args.scan.display()
        )
        .into());
    }
    let calib_path = project.calibration.as_ref().ok_or_else(|| {
        format!(
            "keine Kalibrierung: {} fehlt",
            args.scan.join("calibration").join("calib.json").display()
        )
    })?;
    let readings = readings();
    // Camera name → one setup per reading (same order as `readings`).
    let mut cameras: BTreeMap<String, Vec<CameraSetup>> = BTreeMap::new();
    let mut downscale = 1;
    for reading in &readings {
        let calib = load_calibration_as(calib_path, *reading)?;
        for (name, c) in &calib.cameras {
            let Some(lidar_to_cam) = c.lidar_to_cam else {
                if reading == &readings[0] {
                    eprintln!("Warnung: Kamera {name} hat keine transform_matrix, übersprungen");
                }
                continue;
            };
            let d = working_downscale(c.width);
            downscale = downscale.max(d);
            cameras.entry(name.clone()).or_default().push(CameraSetup {
                camera: scaled_camera(&Camera::from_calib(name, c)?, d),
                lidar_cam: lidar_to_cam.inverse(),
            });
        }
    }
    if cameras.is_empty() {
        return Err("keine Kamera mit transform_matrix in calib.json".into());
    }
    let traj_path = args.slam.join("traj.tum");
    let traj =
        read_tum(&traj_path).map_err(|e| format!("Laufbahn {}: {e}", traj_path.display()))?;
    if traj.len() < 2 {
        return Err(format!("Laufbahn {}: weniger als 2 Posen", traj_path.display()).into());
    }
    let topics = image_topics(&project.image_bags)?;
    if topics.is_empty() {
        return Err(format!("keine Kamerabilder ({IMAGE_TYPE}) in den IMAGE_*.bag").into());
    }

    // Pass 1: sharpness of every usable image (the trajectory must cover every offset).
    let margin = f64::from(*OFFSET_STEPS.end()).max(-f64::from(*OFFSET_STEPS.start())) / 100.0;
    let (t_lo, t_hi) = (traj[0].t + margin, traj[traj.len() - 1].t - margin);
    let mut candidates = Vec::new();
    let mut index = 0usize;
    let mut size_warned = BTreeSet::new();
    let stats = for_each_image(&project.image_bags, &topics, downscale, |frame| {
        let i = index;
        index += 1;
        let Ok(frame) = frame else { return true };
        let Some(cam) = cameras.get(&frame.camera).map(|c| &c[0].camera) else {
            return true;
        };
        if (frame.rgb.width, frame.rgb.height) != (cam.width, cam.height) {
            if size_warned.insert(frame.camera.clone()) {
                eprintln!(
                    "Warnung: Kamera {}: Bildgröße {}x{} passt nicht zur Kalibrierung ({}x{} nach Verkleinerung), übersprungen",
                    frame.camera, frame.rgb.width, frame.rgb.height, cam.width, cam.height
                );
            }
            return true;
        }
        if frame.stamp >= t_lo && frame.stamp <= t_hi {
            candidates.push(Candidate {
                index: i,
                camera: frame.camera,
                stamp: frame.stamp,
                sharpness: sharpness(&frame.rgb),
            });
        }
        true
    })?;
    if stats.frames == 0 {
        return Err("keine Kamerabilder lesbar".into());
    }
    if candidates.is_empty() {
        return Err("kein Kamerabild liegt im Zeitraum der Laufbahn".into());
    }
    let chosen = select_images(&candidates, args.images);

    // Pass 2: gradient images of the chosen ones.
    let by_index: BTreeMap<usize, &Candidate> = candidates.iter().map(|c| (c.index, c)).collect();
    let last = *chosen.last().expect("candidates are not empty");
    let mut shots: Vec<Shot> = Vec::with_capacity(chosen.len());
    let mut index = 0usize;
    for_each_image(&project.image_bags, &topics, downscale, |frame| {
        let i = index;
        index += 1;
        if let (Ok(frame), true) = (frame, chosen.binary_search(&i).is_ok()) {
            let c = by_index[&i];
            shots.push(Shot {
                camera: c.camera.clone(),
                stamp: c.stamp,
                sharpness: c.sharpness,
                grad: gradient(&frame.rgb),
                points: Vec::new(),
            });
        }
        i < last
    })?;
    shots.sort_by(|a, b| a.stamp.total_cmp(&b.stamp).then(a.camera.cmp(&b.camera)));

    // LiDAR points of ±POINT_WINDOW around each image, in the world frame.
    let ((), _) = with_lidar_frames(&project.lidar_bags, args.lidar_topic.as_deref(), |frames| {
        for frame in frames {
            let points = scan_points(&frame);
            let (Some(t0), Some(t1)) = (
                points.iter().map(|p| p.t).reduce(f64::min),
                points.iter().map(|p| p.t).reduce(f64::max),
            ) else {
                continue;
            };
            let near: Vec<usize> = (0..shots.len())
                .filter(|&k| {
                    t1 >= shots[k].stamp - POINT_WINDOW && t0 <= shots[k].stamp + POINT_WINDOW
                })
                .collect();
            if near.is_empty() {
                continue;
            }
            for p in &points {
                let Some(pose) = interpolate_pose(&traj, p.t) else {
                    continue;
                };
                let w = pose * lidar_to_imu * Point3::from(p.xyz);
                for &k in &near {
                    if (p.t - shots[k].stamp).abs() <= POINT_WINDOW {
                        shots[k].points.push(WorldPoint {
                            p: [w.x as f32, w.y as f32, w.z as f32],
                            edge: p.edge,
                        });
                    }
                }
            }
        }
    })?;

    // Score every (image, variant) pair; fixed order, so the result is deterministic.
    let vars = variants(readings.len());
    let view_of = |shot: &Shot, v: Variant| -> Option<Vec<ViewPoint>> {
        let setup = &cameras[&shot.camera][v.reading];
        let body = interpolate_pose(&traj, shot.stamp + v.offset())?;
        let world_cam = body * lidar_to_imu * setup.lidar_cam;
        Some(project_view(&shot.points, &setup.camera, &world_cam))
    };
    let per_view: Vec<(f64, u64)> = (0..shots.len() * vars.len())
        .into_par_iter()
        .map(|i| {
            let (shot, v) = (&shots[i / vars.len()], vars[i % vars.len()]);
            view_of(shot, v).map_or((0.0, 0), |view| score_view(&view, &shot.grad))
        })
        .collect();
    let camera_names: BTreeSet<&str> = shots.iter().map(|s| s.camera.as_str()).collect();
    let mut reports = Vec::with_capacity(vars.len());
    let mut totals = Vec::with_capacity(vars.len());
    let mut cam_scores: BTreeMap<&str, Vec<Score>> = BTreeMap::new();
    for (j, &v) in vars.iter().enumerate() {
        let mut total = (0.0, 0u64);
        let mut per_cam: BTreeMap<&str, (f64, u64)> =
            camera_names.iter().map(|&c| (c, (0.0, 0))).collect();
        for (k, shot) in shots.iter().enumerate() {
            let (s, n) = per_view[k * vars.len() + j];
            total = (total.0 + s, total.1 + n);
            let e = per_cam.get_mut(shot.camera.as_str()).expect("known camera");
            *e = (e.0 + s, e.1 + n);
        }
        let per_camera: BTreeMap<String, Score> = per_cam
            .iter()
            .map(|(&c, &sn)| (c.to_string(), make_score(v, &readings, sn)))
            .collect();
        for (&c, &sn) in &per_cam {
            cam_scores
                .entry(c)
                .or_default()
                .push(make_score(v, &readings, sn));
        }
        totals.push(make_score(v, &readings, total));
        reports.push(VariantReport {
            total: make_score(v, &readings, total),
            per_camera,
        });
    }
    let (summary, best, worst) = summarise(&totals);
    let per_camera_best = cam_scores
        .iter()
        .map(|(&c, s)| (c.to_string(), s[best_of(s, 0..s.len())].clone()))
        .collect();

    fs::create_dir_all(&args.out)?;
    let mut counter: BTreeMap<&str, usize> = BTreeMap::new();
    for shot in &shots {
        let n = counter.entry(shot.camera.as_str()).or_default();
        *n += 1;
        for (label, v) in [("best", vars[best]), ("worst", vars[worst])] {
            let view = view_of(shot, v).unwrap_or_default();
            let path = args
                .out
                .join(format!("overlay_{label}_{}_{n:02}.png", shot.camera));
            write_png(
                &path,
                shot.grad.width,
                shot.grad.height,
                &overlay(&shot.grad, &view),
            )?;
        }
    }
    let report = Report {
        images: shots
            .iter()
            .map(|s| ImageReport {
                camera: s.camera.clone(),
                stamp: s.stamp,
                sharpness: s.sharpness,
                lidar_points: s.points.len(),
            })
            .collect(),
        summary,
        per_camera_best,
        variants: reports,
    };
    fs::write(
        args.out.join("calib_check.json"),
        serde_json::to_string_pretty(&report)? + "\n",
    )?;

    let s = &report.summary;
    println!(
        "{} Bilder, {} Varianten geprüft",
        report.images.len(),
        vars.len()
    );
    println!(
        "Beste Variante: Bild {:?}, Matrix {:?}, Zeitversatz {:+.2} s, Güte {:.3} ({} Kantenpunkte)",
        s.best.image_rotation, s.best.direction, s.best.time_offset, s.best.score, s.best.edge_points
    );
    println!(
        "Beste andere Bilddrehung: {:?}, {:?}, {:+.2} s, Güte {:.3}",
        s.best_other_rotation.image_rotation,
        s.best_other_rotation.direction,
        s.best_other_rotation.time_offset,
        s.best_other_rotation.score
    );
    println!(
        "Beste andere Richtung: {:?}, {:+.2} s, Güte {:.3}; Abstand {:.3}",
        s.best_other_direction.direction,
        s.best_other_direction.time_offset,
        s.best_other_direction.score,
        s.margin_direction
    );
    for (c, b) in &report.per_camera_best {
        println!(
            "  Kamera {c}: Bild {:?}, Matrix {:?}, {:+.2} s, Güte {:.3}",
            b.image_rotation, b.direction, b.time_offset, b.score
        );
    }
    println!("→ {}", args.out.display());
    Ok(())
}
