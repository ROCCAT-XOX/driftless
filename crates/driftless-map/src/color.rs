//! Colouring the cloud from the camera images.
//!
//! Two passes over the images, each streamed from the bags one image at a time:
//!
//! 1. **Quality** (images decoded at 1/[`SHARPNESS_DOWNSCALE`]): [`sharpness`] of every
//!    image and the angular rate of the trajectory at its capture time. An image is used if
//!    its capture time lies on the trajectory, its sharpness reaches
//!    [`ColorConfig::min_sharpness`] × the median of the scan and the rate stays at or below
//!    [`ColorConfig::max_angular_rate`].
//! 2. **Colour** (images decoded at 1/[`ColorConfig::downscale`]): per used image, the points
//!    it sees (see [`crate::visibility`]) get a view with weight
//!    `w = cos(incidence) / (1 + d²/4) · edge falloff(θ) · sharpness_norm` and the colour
//!    there (bilinear, in linear RGB). Each point keeps its
//!    [`ColorConfig::views_kept`] best views; its colour is the weighted mean of the best
//!    [`ColorConfig::views_blended`] of them in linear RGB, written as sRGB.
//!
//! Before the blend, each view's colour is divided by its image's exposure and white-balance
//! gain (see [`crate::exposure`]), fitted on the kept views.
//!
//! Points no image sees get a gray from their intensity and `colored = false`.

use crate::clean::spread_axes;
use crate::exposure::{
    apply_gains, camera_difference, estimate_gains, image_offset, sample_observations, view_spread,
    ExposureConfig, ImageGain,
};
use crate::sharpness::sharpness;
use crate::visibility::{visible_points, PointGrid, Seen};
use crate::{Cloud, MapError};
use driftless_io::calib::CAMERA_TIME_OFFSET_S;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, RgbImage};
use driftless_traj::{interpolate_pose, Pose};
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::{Isometry3, Matrix3, Vector3};
use rayon::prelude::*;
use std::num::NonZeroUsize;
use std::path::PathBuf;

/// Images are decoded at this fraction of their size for the sharpness pass.
pub const SHARPNESS_DOWNSCALE: u32 = 4;
/// The angular rate is taken over ±this time around the capture time (s).
pub const RATE_HALF_WINDOW: f64 = 0.05;
/// Up to this angle from the optical axis a view has full weight; beyond it the weight falls
/// linearly to 0 at the camera's `max_theta`.
pub const FULL_WEIGHT_THETA: f64 = 60.0 * std::f64::consts::PI / 180.0;
/// Neighbours used for a point's surface normal.
pub const NORMAL_NEIGHBOURS: usize = 12;
/// Nearest neighbours whose mean distance is a point's spacing (its splat size).
pub const SPACING_NEIGHBOURS: usize = 4;
/// Gray of an uncoloured point with intensity 0; the cloud's 99th intensity percentile and
/// above get 255. Dark returns stay visible and are not mistaken for black surfaces.
pub const GRAY_MIN: u8 = 64;

/// The camera images of a scan: every pass streams them again from the bags.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageSource {
    pub bags: Vec<PathBuf>,
    /// `sensor_msgs/CompressedImage` topics; the camera name comes from the topic
    /// (`/camera_left/image_raw` → `left`).
    pub topics: Vec<String>,
}

/// A camera with its mount on the device.
#[derive(Debug, Clone, PartialEq)]
pub struct ColorCamera {
    /// Intrinsics at full image size.
    pub camera: Camera,
    /// Camera pose in the body (IMU) frame: the camera's world pose at time `t` is
    /// `trajectory(t) · cam_in_body`.
    pub cam_in_body: Isometry3<f64>,
}

impl ColorCamera {
    /// From the calibration's `lidar_to_cam` (LiDAR points into the camera frame) and the
    /// LiDAR mount `lidar_to_imu` (as in the odometry): `cam_in_body = lidar_to_imu ·
    /// lidar_to_cam⁻¹`.
    pub fn new(
        camera: Camera,
        lidar_to_cam: &Isometry3<f64>,
        lidar_to_imu: &Isometry3<f64>,
    ) -> ColorCamera {
        ColorCamera {
            camera,
            cam_in_body: lidar_to_imu * lidar_to_cam.inverse(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ColorConfig {
    /// Images are decoded at 1/`downscale` of their size for colouring (1, 2 or 4).
    pub downscale: u32,
    /// An image stamped `t` was taken at trajectory time `t + time_offset` (s).
    pub time_offset: f64,
    /// Points farther from the camera get no colour from it (m).
    pub max_dist: f64,
    /// Absolute part of the depth tolerance (m); the relative part is
    /// [`crate::visibility::DEPTH_TOLERANCE_REL`] of the depth.
    pub depth_tolerance: f64,
    /// A depth-buffer cell covers `zbuffer_scale` × `zbuffer_scale` pixels of the decoded
    /// image.
    pub zbuffer_scale: u32,
    /// Images with a sharpness below this share of the scan's median sharpness are skipped.
    pub min_sharpness: f64,
    /// Images taken while the device turned faster are skipped (rad/s, from the trajectory).
    pub max_angular_rate: f64,
    /// Views kept per point (the best ones; also what the exposure fit sees).
    pub views_kept: usize,
    /// Views blended into a point's colour: the best `views_blended` of the kept ones.
    pub views_blended: usize,
    /// Exposure and white balance between the images.
    pub exposure: ExposureConfig,
}

impl Default for ColorConfig {
    fn default() -> Self {
        ColorConfig {
            downscale: 2,
            time_offset: CAMERA_TIME_OFFSET_S,
            max_dist: 12.0,
            depth_tolerance: 0.05,
            zbuffer_scale: 4,
            min_sharpness: 0.4,
            max_angular_rate: 1.0,
            views_kept: 3,
            views_blended: 3,
            exposure: ExposureConfig::default(),
        }
    }
}

impl ColorConfig {
    fn check(&self) -> Result<(), MapError> {
        let bad = |msg: &str| Err(MapError::Config(msg.into()));
        if !matches!(self.downscale, 1 | 2 | 4) {
            return bad("Verkleinerung der Bilder muss 1, 2 oder 4 sein");
        }
        if !self.time_offset.is_finite() {
            return bad("Zeitversatz der Kamera muss eine endliche Zahl sein");
        }
        if !(self.max_dist.is_finite() && self.max_dist > 0.0) {
            return bad("Höchstabstand zur Kamera muss größer als 0 sein");
        }
        if !(self.depth_tolerance.is_finite() && self.depth_tolerance >= 0.0) {
            return bad("Tiefentoleranz darf nicht negativ sein");
        }
        if self.zbuffer_scale == 0 {
            return bad("Maßstab des Tiefenpuffers muss mindestens 1 sein");
        }
        if !(self.min_sharpness.is_finite() && self.min_sharpness >= 0.0) {
            return bad("Mindestschärfe darf nicht negativ sein");
        }
        if self.max_angular_rate.is_nan() || self.max_angular_rate <= 0.0 {
            return bad("höchste Drehrate muss größer als 0 sein");
        }
        if self.views_kept == 0 {
            return bad("Ansichten je Punkt müssen mindestens 1 sein");
        }
        if self.views_blended == 0 || self.views_blended > self.views_kept {
            return bad("gemischte Ansichten müssen zwischen 1 und den behaltenen liegen");
        }
        self.exposure.check()
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ColorStats {
    /// Images that passed every check and were projected.
    pub images_used: usize,
    /// Images skipped as blurred: too little sharpness or too fast a turn.
    pub images_blurry: usize,
    /// Images whose capture time lies outside the trajectory.
    pub images_outside_trajectory: usize,
    /// Images that could not be decoded, of an unknown camera or of the wrong size.
    pub images_bad: usize,
    /// Image bags that were cut short; the images before the cut were used.
    pub image_bags_truncated: usize,
    pub points_colored: usize,
    pub points_uncolored: usize,
    /// Gain of every used image, in (capture time, camera name) order.
    pub gains: Vec<ImageGain>,
    /// Smallest and largest gain over all used images and channels (1 without images).
    pub gain_min: f32,
    pub gain_max: f32,
    /// Mean colour spread between the views of a point before and after the equalisation:
    /// standard deviation of the blended views' colours per channel, in sRGB units (0–255),
    /// averaged over the channels and over the points with at least 2 views.
    pub spread_before: f64,
    pub spread_after: f64,
    /// Systematic colour difference between the cameras on the points they both see, before
    /// and after the equalisation: per camera pair the mean difference of the two cameras'
    /// view colours, in sRGB units (0–255), averaged over the channels (absolute value) and
    /// the pairs. It reacts to exposure and white balance, not to the scatter of single views.
    pub camera_difference_before: f64,
    pub camera_difference_after: f64,
    /// Points with views from at least 2 cameras (once per camera pair).
    pub camera_difference_points: usize,
    /// Mean colour offset of the images before and after the equalisation: per image, the
    /// mean difference of its views to their points' mean colour, in sRGB units (0–255),
    /// averaged over the channels (absolute value) and the images (weighted by views). It
    /// reacts to exposure changes over time as well as between the cameras.
    pub image_offset_before: f64,
    pub image_offset_after: f64,
}

/// What the quality pass found for one image of the stream.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Status {
    Bad,
    Outside,
    Blurry,
    Used {
        /// Position in (capture time, camera name) order: breaks ties between equal weights.
        rank: u32,
        /// Sharpness relative to the scan's median, at most 1.
        sharpness_norm: f32,
    },
}

#[derive(Debug, Clone, Copy)]
struct ImageRecord {
    camera: usize,
    stamp: f64,
    status: Status,
}

/// Body rotation rate around `t` (rad/s): rotation angle between the poses at
/// `t ± RATE_HALF_WINDOW` (clamped to the trajectory) over the time between them.
fn angular_rate(poses: &[Pose], t: f64) -> f64 {
    let (Some(first), Some(last)) = (poses.first(), poses.last()) else {
        return 0.0;
    };
    let (lo, hi) = (
        (t - RATE_HALF_WINDOW).max(first.t),
        (t + RATE_HALF_WINDOW).min(last.t),
    );
    match (interpolate_pose(poses, lo), interpolate_pose(poses, hi)) {
        (Some(a), Some(b)) if hi > lo => (a.rotation.inverse() * b.rotation).angle() / (hi - lo),
        _ => 0.0,
    }
}

/// Median of `v` (mean of the two middle values for an even count); `None` if empty.
fn median(v: &[f64]) -> Option<f64> {
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    let n = s.len();
    match n {
        0 => None,
        _ if n % 2 == 1 => Some(s[n / 2]),
        _ => Some(0.5 * (s[n / 2 - 1] + s[n / 2])),
    }
}

/// Camera index of a frame and whether its size fits the camera decoded at `downscale`.
fn frame_camera(frame: &Frame, cams: &[ColorCamera], downscale: u32) -> Option<usize> {
    let c = cams.iter().position(|c| c.camera.name == frame.camera)?;
    let cam = &cams[c].camera;
    ((frame.rgb.width, frame.rgb.height) == (cam.width / downscale, cam.height / downscale))
        .then_some(c)
}

/// What the quality pass measured on one image.
#[derive(Debug, Clone, Copy)]
struct Measured {
    /// `None`: undecodable, unknown camera or wrong size.
    camera: Option<usize>,
    stamp: f64,
    /// (sharpness, angular rate) if the capture time lies on the trajectory.
    quality: Option<(f64, f64)>,
}

/// Pass 1: status of every image of the stream, in stream order.
fn quality_pass(
    images: &ImageSource,
    poses: &[Pose],
    cams: &[ColorCamera],
    cfg: &ColorConfig,
) -> Result<(Vec<ImageRecord>, usize), MapError> {
    let mut found: Vec<Measured> = Vec::new();
    let read = for_each_image(&images.bags, &images.topics, SHARPNESS_DOWNSCALE, |frame| {
        let (camera, stamp, quality) = match frame {
            Err(_) => (None, f64::NAN, None),
            Ok(frame) => match frame_camera(&frame, cams, SHARPNESS_DOWNSCALE) {
                None => (None, frame.stamp, None),
                Some(c) => {
                    let t = frame.stamp + cfg.time_offset;
                    let quality = interpolate_pose(poses, t)
                        .map(|_| (sharpness(&frame.rgb), angular_rate(poses, t)));
                    (Some(c), frame.stamp, quality)
                }
            },
        };
        found.push(Measured {
            camera,
            stamp,
            quality,
        });
        true
    })
    .map_err(|e| MapError::Images(e.to_string()))?;
    let truncated = read.truncated;

    let on_traj: Vec<f64> = found
        .iter()
        .filter_map(|f| f.quality.map(|q| q.0))
        .collect();
    let median = median(&on_traj).unwrap_or(0.0);
    let limit = cfg.min_sharpness * median;
    let mut records: Vec<ImageRecord> = found
        .iter()
        .map(
            |&Measured {
                 camera,
                 stamp,
                 quality,
             }| ImageRecord {
                camera: camera.unwrap_or(usize::MAX),
                stamp,
                status: match (camera, quality) {
                    (None, _) => Status::Bad,
                    (Some(_), None) => Status::Outside,
                    (Some(_), Some((s, rate))) if s < limit || rate > cfg.max_angular_rate => {
                        Status::Blurry
                    }
                    (Some(_), Some((s, _))) => Status::Used {
                        rank: 0,
                        sharpness_norm: if median > 0.0 {
                            (s / median).min(1.0) as f32
                        } else {
                            1.0
                        },
                    },
                },
            },
        )
        .collect();

    // Ranks in (capture time, camera name) order.
    let mut used: Vec<usize> = (0..records.len())
        .filter(|&i| matches!(records[i].status, Status::Used { .. }))
        .collect();
    used.sort_by(|&a, &b| {
        let (ra, rb) = (&records[a], &records[b]);
        ra.stamp
            .total_cmp(&rb.stamp)
            .then_with(|| {
                cams[ra.camera]
                    .camera
                    .name
                    .cmp(&cams[rb.camera].camera.name)
            })
            .then(a.cmp(&b))
    });
    for (r, &i) in used.iter().enumerate() {
        if let Status::Used { rank, .. } = &mut records[i].status {
            *rank = u32::try_from(r).expect("fewer than 2^32 images");
        }
    }
    Ok((records, truncated))
}

/// Surface normal (if the neighbourhood is planar) and spacing (mean distance of the
/// [`SPACING_NEIGHBOURS`] nearest other points; 0 without neighbours) of every point.
fn local_geometry(xyz: &[[f32; 3]]) -> (Vec<Option<Vector3<f32>>>, Vec<f32>) {
    if xyz.len() < 2 {
        return (vec![None; xyz.len()], vec![0.0; xyz.len()]);
    }
    // The tree is built on the cloud's own f32 coordinates: no f64 copy of the cloud.
    let tree: ImmutableKdTree<f32, 3> =
        ImmutableKdTree::new_from_slice(xyz).expect("fewer than 2^32 points");
    let at = |i: u32| Vector3::from(xyz[i as usize].map(f64::from));
    // The point itself comes back with distance 0, hence + 1.
    let k = NonZeroUsize::new((NORMAL_NEIGHBOURS + 1).min(xyz.len())).expect("k > 0");
    xyz.par_iter()
        .map(|p| {
            let nn = tree
                .query(p)
                .nearest_n::<SquaredEuclidean<f32>>(k)
                .execute();
            let others = &nn[1..];
            let m = others.len().min(SPACING_NEIGHBOURS);
            let spacing = others[..m]
                .iter()
                .map(|x| f64::from(x.distance).sqrt())
                .sum::<f64>()
                / m as f64;
            let normal = (nn.len() >= 3)
                .then(|| {
                    let c = nn.len() as f64;
                    let mean = nn.iter().map(|x| at(x.item)).sum::<Vector3<f64>>() / c;
                    let cov = nn
                        .iter()
                        .map(|x| {
                            let d = at(x.item) - mean;
                            d * d.transpose()
                        })
                        .sum::<Matrix3<f64>>()
                        / c;
                    let spread = spread_axes(&cov);
                    (spread[0].0 < 0.1 * spread[1].0).then(|| spread[0].1.cast::<f32>())
                })
                .flatten();
            (normal, spacing as f32)
        })
        .unzip()
}

/// sRGB value → linear, for all 256 values.
fn linear_table() -> [f32; 256] {
    std::array::from_fn(|i| {
        let c = i as f64 / 255.0;
        let l = if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        };
        l as f32
    })
}

/// Linear value → sRGB in 0–255, not rounded.
pub(crate) fn srgb_value(l: f32) -> f64 {
    let l = f64::from(l.clamp(0.0, 1.0));
    let c = if l <= 0.003_130_8 {
        12.92 * l
    } else {
        1.055 * l.powf(1.0 / 2.4) - 0.055
    };
    c * 255.0
}

/// Linear value → sRGB byte.
fn to_srgb(l: f32) -> u8 {
    srgb_value(l).round().clamp(0.0, 255.0) as u8
}

/// Bilinear sample of `img` at pixel `uv` (pixel centres at integers), in linear RGB.
fn sample_linear(img: &RgbImage, uv: [f32; 2], lin: &[f32; 256]) -> [f32; 3] {
    let (w, h) = (img.width as usize, img.height as usize);
    let x = uv[0].clamp(0.0, (w - 1) as f32);
    let y = uv[1].clamp(0.0, (h - 1) as f32);
    let (x0, y0) = (x.floor() as usize, y.floor() as usize);
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (fx, fy) = (x - x0 as f32, y - y0 as f32);
    let px = |x: usize, y: usize, c: usize| lin[img.data[(y * w + x) * 3 + c] as usize];
    std::array::from_fn(|c| {
        (px(x0, y0, c) * (1.0 - fx) + px(x1, y0, c) * fx) * (1.0 - fy)
            + (px(x0, y1, c) * (1.0 - fx) + px(x1, y1, c) * fx) * fy
    })
}

/// Weight of a view: `cos(incidence) / (1 + d²/4) · edge falloff(θ) · sharpness_norm`.
/// Without a normal the incidence counts as head-on.
fn view_weight(
    p: &[f32; 3],
    normal: Option<&Vector3<f32>>,
    cam_pos: &Vector3<f32>,
    seen: &Seen,
    max_theta: f64,
    sharpness_norm: f32,
) -> f32 {
    let d = seen.proj.range;
    let cos_inc = normal.map_or(1.0, |n| {
        let to_cam = (cam_pos - Vector3::from(*p)) / d;
        n.dot(&to_cam).abs()
    });
    let theta = f64::from(seen.proj.theta);
    let falloff = if theta <= FULL_WEIGHT_THETA {
        1.0
    } else if max_theta > FULL_WEIGHT_THETA {
        ((max_theta - theta) / (max_theta - FULL_WEIGHT_THETA)).clamp(0.0, 1.0)
    } else {
        0.0
    };
    cos_inc / (1.0 + d * d / 4.0) * falloff as f32 * sharpness_norm
}

/// One kept view of a point; `weight == 0` marks an empty slot. 16 bytes: the colour and the
/// angle are stored as `u16`. The colour covers linear 0 to [`RGB_RANGE`] (a view divided by
/// an exposure gain of at least 1/4 can exceed 1) in steps of about 6e-5, the angle steps by
/// below 0.003°.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct View {
    pub(crate) weight: f32,
    /// The image's position in (capture time, camera name) order.
    pub(crate) rank: u32,
    /// Linear RGB / [`RGB_RANGE`] × 65535.
    pub(crate) rgb: [u16; 3],
    /// Angle from the optical axis: θ / π × 65535.
    pub(crate) theta: u16,
}

const EMPTY: View = View {
    weight: 0.0,
    rank: u32::MAX,
    rgb: [0; 3],
    theta: 0,
};

/// Largest linear colour a view can hold: the inverse of the smallest exposure gain
/// ([`crate::exposure::ExposureConfig::max_gain`] of 4 at its default).
const RGB_RANGE: f32 = 4.0;

/// `v` (clamped to 0 to `range`) as a `u16` over the full range.
fn pack(v: f32, range: f32) -> u16 {
    (v.clamp(0.0, range) / range * 65535.0).round() as u16
}

impl View {
    /// A view with linear colour `rgb` seen at `theta` (rad) from the optical axis.
    pub(crate) fn new(weight: f32, rank: u32, rgb: [f32; 3], theta: f32) -> View {
        View {
            weight,
            rank,
            rgb: rgb.map(|c| pack(c, RGB_RANGE)),
            theta: pack(theta / std::f32::consts::PI, 1.0),
        }
    }

    /// Linear RGB.
    pub(crate) fn linear(&self) -> [f32; 3] {
        self.rgb.map(|c| f32::from(c) / 65535.0 * RGB_RANGE)
    }

    /// Angle from the optical axis (rad).
    #[allow(dead_code)] // read by the edge falloff fit
    pub(crate) fn theta(&self) -> f32 {
        f32::from(self.theta) / 65535.0 * std::f32::consts::PI
    }

    /// Replaces the colour by linear `rgb`.
    pub(crate) fn set_linear(&mut self, rgb: [f32; 3]) {
        self.rgb = rgb.map(|c| pack(c, RGB_RANGE));
    }

    /// Higher weight first; equal weights: the earlier image (capture time, camera name).
    fn better_than(&self, other: &View) -> bool {
        self.weight > other.weight || (self.weight == other.weight && self.rank < other.rank)
    }
}

/// Inserts `v` into the point's views (best first) if it is among the best.
fn keep_best(slots: &mut [View], v: View) {
    if let Some(j) = slots.iter().position(|s| v.better_than(s)) {
        slots[j..].rotate_right(1);
        slots[j] = v;
    }
}

/// Weighted mean of the best `blended` views in linear RGB as sRGB; `None` without views.
fn blend(slots: &[View], blended: usize) -> Option<[u8; 3]> {
    let (mut sum, mut wsum) = ([0.0f64; 3], 0.0f64);
    for v in slots[..blended.min(slots.len())]
        .iter()
        .filter(|v| v.weight > 0.0)
    {
        let w = f64::from(v.weight);
        wsum += w;
        for (s, c) in sum.iter_mut().zip(v.linear()) {
            *s += w * f64::from(c);
        }
    }
    (wsum > 0.0).then(|| sum.map(|s| to_srgb((s / wsum) as f32)))
}

/// Gray from intensity: [`GRAY_MIN`] at 0, 255 at the cloud's 99th percentile and above.
fn intensity_grays(intensity: &[f32]) -> Vec<u8> {
    let mut sorted: Vec<f32> = intensity
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .collect();
    sorted.sort_by(f32::total_cmp);
    let p99 = sorted
        .get(((sorted.len().max(1) - 1) as f64 * 0.99) as usize)
        .copied()
        .unwrap_or(0.0);
    let span = f32::from(255 - GRAY_MIN);
    intensity
        .iter()
        .map(|&i| {
            let s = if p99 > 0.0 && i.is_finite() {
                (i / p99).clamp(0.0, 1.0)
            } else {
                0.0
            };
            GRAY_MIN + (s * span).round() as u8
        })
        .collect()
}

/// Colours `cloud` (world frame) from the camera images in `images`. `poses` is the body
/// trajectory (e.g. the SLAM `traj.tum`), `cams` every camera with its mount. Sets
/// `cloud.rgb` and `cloud.colored` for every point; points no used image sees get a gray
/// from their intensity and `colored = false`. Same input, same result.
///
/// Fails if `cfg` is invalid, two cameras share a name or the images cannot be read.
pub fn colorize(
    cloud: &mut Cloud,
    images: ImageSource,
    poses: &[Pose],
    cams: &[ColorCamera],
    cfg: &ColorConfig,
) -> Result<ColorStats, MapError> {
    cfg.check()?;
    for (i, c) in cams.iter().enumerate() {
        if cams[..i].iter().any(|o| o.camera.name == c.camera.name) {
            return Err(MapError::Config(format!(
                "Kamera {} ist doppelt angegeben",
                c.camera.name
            )));
        }
    }
    let (records, bags_truncated) = quality_pass(&images, poses, cams, cfg)?;
    let count = |f: fn(&Status) -> bool| records.iter().filter(|r| f(&r.status)).count();
    let mut stats = ColorStats {
        images_used: count(|s| matches!(s, Status::Used { .. })),
        images_blurry: count(|s| *s == Status::Blurry),
        images_outside_trajectory: count(|s| *s == Status::Outside),
        images_bad: count(|s| *s == Status::Bad),
        image_bags_truncated: bags_truncated,
        ..ColorStats::default()
    };

    let k = cfg.views_kept;
    let last_used = records
        .iter()
        .rposition(|r| matches!(r.status, Status::Used { .. }));
    // Allocated after `local_geometry`, whose k-d tree is the memory peak of the setup.
    let mut slots = Vec::new();
    if let (Some(last_used), false) = (last_used, cloud.is_empty()) {
        let (normals, spacing) = local_geometry(&cloud.xyz);
        slots = vec![EMPTY; cloud.len() * k];
        let grid = PointGrid::new(&cloud.xyz);
        let lin = linear_table();
        let scaled: Vec<Camera> = cams
            .iter()
            .map(|c| c.camera.downscaled(cfg.downscale))
            .collect();
        let mut index = 0usize;
        let mut failure: Option<MapError> = None;
        for_each_image(&images.bags, &images.topics, cfg.downscale, |frame| {
            let i = index;
            index += 1;
            let rec = records.get(i).copied();
            let Some(ImageRecord {
                camera,
                stamp,
                status:
                    Status::Used {
                        rank,
                        sharpness_norm,
                    },
            }) = rec
            else {
                return i < last_used;
            };
            let frame = match frame {
                Ok(f)
                    if f.stamp == stamp
                        && frame_camera(&f, cams, cfg.downscale) == Some(camera) =>
                {
                    f
                }
                _ => {
                    failure = Some(MapError::Images(
                        "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
                    ));
                    return false;
                }
            };
            let body = interpolate_pose(poses, stamp + cfg.time_offset)
                .expect("checked in the quality pass");
            let world_cam = body * cams[camera].cam_in_body;
            let cam_pos = world_cam.translation.vector;
            let cam = &scaled[camera];
            let candidates = grid.near(&cam_pos, cfg.max_dist);
            let seen = visible_points(
                &cloud.xyz,
                &spacing,
                &candidates,
                cam,
                &world_cam.inverse(),
                cfg.max_dist,
                cfg.zbuffer_scale,
                cfg.depth_tolerance,
            );
            let cam_pos = cam_pos.cast::<f32>();
            let views: Vec<(u32, View)> = seen
                .par_iter()
                .filter_map(|s| {
                    let p = &cloud.xyz[s.index as usize];
                    let weight = view_weight(
                        p,
                        normals[s.index as usize].as_ref(),
                        &cam_pos,
                        s,
                        cam.max_theta,
                        sharpness_norm,
                    );
                    (weight > 0.0).then(|| {
                        let rgb = sample_linear(&frame.rgb, s.proj.uv, &lin);
                        (s.index, View::new(weight, rank, rgb, s.proj.theta))
                    })
                })
                .collect();
            for (p, v) in views {
                let p = p as usize;
                keep_best(&mut slots[p * k..(p + 1) * k], v);
            }
            i < last_used
        })
        .map_err(|e| MapError::Images(e.to_string()))?;
        if let Some(e) = failure {
            return Err(e);
        }
    }

    slots.resize(cloud.len() * k, EMPTY);
    // Exposure and white balance, fitted on the kept views. Images by rank.
    let mut used: Vec<&ImageRecord> = records
        .iter()
        .filter(|r| matches!(r.status, Status::Used { .. }))
        .collect();
    used.sort_by_key(|r| match r.status {
        Status::Used { rank, .. } => rank,
        _ => unreachable!("only used images"),
    });
    let cameras: Vec<usize> = used.iter().map(|r| r.camera).collect();
    stats.spread_before = view_spread(&slots, k);
    let before = camera_difference(&slots, k, &cameras);
    stats.image_offset_before = image_offset(&slots, k, used.len());
    let gains = if cfg.exposure.samples == 0 {
        vec![[1.0; 3]; used.len()]
    } else {
        let obs = sample_observations(&slots, k, cameras.clone(), cfg.exposure.samples);
        estimate_gains(&obs, &cfg.exposure)
    };
    apply_gains(&mut slots, &gains);
    stats.spread_after = view_spread(&slots, k);
    let after = camera_difference(&slots, k, &cameras);
    stats.image_offset_after = image_offset(&slots, k, used.len());
    stats.camera_difference_before = before.value;
    stats.camera_difference_after = after.value;
    stats.camera_difference_points = before.points;
    let all = gains.iter().flatten();
    stats.gain_min = all.clone().copied().reduce(f32::min).unwrap_or(1.0);
    stats.gain_max = all.copied().reduce(f32::max).unwrap_or(1.0);
    stats.gains = used
        .iter()
        .zip(&gains)
        .map(|(r, g)| ImageGain {
            camera: cams[r.camera].camera.name.clone(),
            stamp: r.stamp,
            gain: *g,
        })
        .collect();

    let grays = intensity_grays(&cloud.intensity);
    let (rgb, colored): (Vec<[u8; 3]>, Vec<bool>) = slots
        .par_chunks(k)
        .zip(grays.par_iter())
        .map(|(s, &g)| blend(s, cfg.views_blended).map_or(([g; 3], false), |c| (c, true)))
        .unzip();
    stats.points_colored = colored.iter().filter(|&&c| c).count();
    stats.points_uncolored = colored.len() - stats.points_colored;
    cloud.rgb = Some(rgb);
    cloud.colored = Some(colored);
    Ok(stats)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn srgb_round_trip_is_exact() {
        let lin = linear_table();
        for v in 0..=255u8 {
            assert_eq!(to_srgb(lin[v as usize]), v);
        }
        // Mid gray in linear light is not 128 in sRGB.
        assert_eq!(to_srgb(0.5), 188);
    }

    #[test]
    fn keep_best_orders_by_weight_then_rank() {
        let v = |weight, rank| View::new(weight, rank, [0.0; 3], 0.0);
        let mut slots = [EMPTY; 3];
        for x in [v(0.5, 4), v(0.9, 7), v(0.5, 2), v(0.1, 0), v(0.5, 3)] {
            keep_best(&mut slots, x);
        }
        assert_eq!(slots, [v(0.9, 7), v(0.5, 2), v(0.5, 3)]);
        // The result does not depend on the insertion order.
        let mut other = [EMPTY; 3];
        for x in [v(0.5, 3), v(0.1, 0), v(0.5, 2), v(0.9, 7), v(0.5, 4)] {
            keep_best(&mut other, x);
        }
        assert_eq!(slots, other);
    }

    #[test]
    fn blend_averages_in_linear_light() {
        let lin = linear_table();
        let view = |c: u8| View::new(1.0, 0, [lin[c as usize]; 3], 0.0);
        // Black and white mix to linear 0.5, i.e. sRGB 188, not 128.
        assert_eq!(blend(&[view(0), view(255), EMPTY], 3), Some([188; 3]));
        assert_eq!(blend(&[EMPTY; 2], 2), None);
        // Only the best `blended` views count.
        assert_eq!(blend(&[view(255), view(0), EMPTY], 1), Some([255; 3]));
    }

    #[test]
    fn view_round_trip() {
        assert_eq!(std::mem::size_of::<View>(), 16);
        let lin = linear_table();
        for (rgb, theta) in [
            ([0.0, 0.5, 1.0], 0.0),
            ([0.123_456, 0.9, 0.000_1], 1.0),
            ([lin[17], lin[200], lin[255]], 2.1),
            ([2.0, -1.0, 0.3], std::f32::consts::PI),
            ([3.9, 1.5, 4.0], 0.5),
            ([9.0, 1.0, 0.0], 0.0),
        ] {
            let v = View::new(0.7, 5, rgb, theta);
            for (a, b) in v.linear().iter().zip(rgb) {
                assert!((a - b.clamp(0.0, RGB_RANGE)).abs() < 1e-4, "{a} vs {b}");
            }
            assert!((v.theta() - theta).abs() < 1e-4, "{} vs {theta}", v.theta());
            assert_eq!((v.weight, v.rank), (0.7, 5));
        }
        // Every sRGB level survives the packing.
        for c in 0..=255u8 {
            let v = View::new(1.0, 0, [lin[c as usize]; 3], 0.0);
            assert_eq!(to_srgb(v.linear()[0]), c);
        }
    }

    #[test]
    fn angular_rate_of_constant_spin() {
        let poses: Vec<Pose> = (0..=20)
            .map(|i| {
                let t = f64::from(i) * 0.1;
                Pose {
                    t,
                    p: Vector3::zeros(),
                    q: nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.8 * t),
                }
            })
            .collect();
        assert!((angular_rate(&poses, 1.0) - 0.8).abs() < 1e-9);
        assert!((angular_rate(&poses, 0.0) - 0.8).abs() < 1e-9); // window clamped
        assert_eq!(angular_rate(&poses[..1], 0.0), 0.0);
    }

    #[test]
    fn grays_scale_with_the_99th_percentile() {
        let intensity: Vec<f32> = (0..=100).map(|i| i as f32).collect();
        let g = intensity_grays(&intensity);
        assert_eq!(g[0], GRAY_MIN);
        assert_eq!(g[99], 255);
        assert_eq!(g[100], 255);
        assert!(g[50] > GRAY_MIN && g[50] < 255);
        assert_eq!(intensity_grays(&[0.0, 0.0]), vec![GRAY_MIN; 2]);
    }
}
