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
//!    [`ColorConfig::views_kept`] best views.
//!
//! Then, per point:
//!
//! 1. Each kept view's colour is divided by its image's exposure and white-balance gain and
//!    by its camera's edge falloff at the view's angle (see [`crate::exposure`]), both fitted
//!    on the kept views.
//! 2. Views beyond [`ColorConfig::rim_theta`] are left out, unless the point has no other.
//! 3. Highlights are rejected: the reference luminance is the second darkest view's (the
//!    darkest's below 3 views); a view brighter than it by more than
//!    [`ColorConfig::highlight_ratio`] and [`ColorConfig::highlight_abs`] is left out.
//!    Highlights are additive and follow the camera over several images; the second darkest
//!    view survives one too dark outlier and up to `views_kept − 2` highlighted views.
//! 4. Soft blend: the best [`ColorConfig::views_blended`] remaining views, each weighted by
//!    its weight minus the next remaining view's (all unchanged if that leaves nothing), in
//!    linear RGB, written as sRGB. The colour then changes smoothly where two neighbouring
//!    points' last blended and first unblended views swap places.
//!
//! Points no image sees get a gray from their intensity and `colored = false`.

use crate::clean::spread_axes;
use crate::exposure::{
    apply_exposure, camera_difference, estimate_exposure, image_offset, sample_observations,
    view_spread, vignette_residual, ExposureConfig, ImageGain, Vignetting,
};
use crate::sharpness::measure;
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
/// Points sampled for the seam metric ([`ColorStats::seam_jump_after`]).
pub const SEAM_SAMPLES: usize = 200_000;
/// Nearest neighbours of a seam sample that are compared with it.
pub const SEAM_NEIGHBOURS: usize = 4;
/// Seam neighbours lie within this many voxels; the voxel is the cloud's median spacing.
pub const SEAM_RADIUS_VOXELS: f64 = 2.0;
/// Seam neighbours' normals differ from the sample's by at most this angle (rad).
pub const SEAM_MAX_NORMAL_ANGLE: f64 = 10.0 * std::f64::consts::PI / 180.0;
/// Floor points of the highlight metric: normal with `|n_z|` above this.
pub const FLOOR_NORMAL_Z: f32 = 0.9;
/// Floor points lie this far below the trajectory's median height (m).
pub const FLOOR_BELOW_TRAJECTORY: f64 = 0.8;
/// Floor cells of the highlight metric (m, in x and y).
pub const FLOOR_CELL: f64 = 0.5;
/// A floor point counts as highlighted above this factor of its cell's median luminance.
pub const FLOOR_HIGHLIGHT: f64 = 1.3;

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
    /// Views blended into a point's colour: the best `views_blended` of the kept ones that
    /// remain after the rim and the highlights are left out.
    pub views_blended: usize,
    /// Reject highlights (see the module docs).
    pub highlights: bool,
    /// A view is a highlight if its luminance exceeds the reference by this share …
    pub highlight_ratio: f64,
    /// … and by this much (linear).
    pub highlight_abs: f64,
    /// Views farther than this from the optical axis (rad) are left out of the blend and of
    /// the highlight reference, unless all of a point's views are.
    pub rim_theta: f64,
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
            views_kept: 6,
            views_blended: 3,
            highlights: true,
            highlight_ratio: 0.3,
            highlight_abs: 0.02,
            rim_theta: 85f64.to_radians(),
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
        if !(self.highlight_ratio.is_finite() && self.highlight_ratio >= 0.0) {
            return bad("Glanzlicht-Verhältnis darf nicht negativ sein");
        }
        if !(self.highlight_abs.is_finite() && self.highlight_abs >= 0.0) {
            return bad("Glanzlicht-Abstand darf nicht negativ sein");
        }
        if !(self.rim_theta.is_finite() && self.rim_theta > 0.0) {
            return bad("Randwinkel muss größer als 0 sein");
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
    /// Used images with a gain at `1/max_gain` or `max_gain` in some channel: the fit wanted
    /// more and these images stay too dark or too bright.
    pub images_gain_clamped: usize,
    /// Edge falloff of every camera (numbered as `cams` of [`colorize`]).
    pub vignetting: Vignetting,
    /// Remaining edge falloff after the gains alone and after gains and falloff (in %, see
    /// `exposure::vignette_residual`): per camera the largest mean luminance residual of a 10°
    /// class of the views' angle from the optical axis, averaged over the cameras.
    pub vignette_residual_before: f64,
    pub vignette_residual_after: f64,
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
    /// Spearman rank correlation between the sharpness and the mean gray value of the images
    /// on the trajectory (0 without at least 2 images or without variation). Close to 0 means
    /// the sharpness does not follow the brightness.
    pub sharpness_brightness_rank_corr: f64,
    /// Views rejected as highlights, and the points with at least one of them.
    pub views_rejected: usize,
    pub points_with_rejected: usize,
    /// Views left out for lying beyond [`ColorConfig::rim_theta`] (points with only such
    /// views keep them and are not counted).
    pub views_rim_excluded: usize,
    /// Share of the floor points (normal `|n_z|` > [`FLOOR_NORMAL_Z`], lower than the
    /// trajectory's median height minus [`FLOOR_BELOW_TRAJECTORY`], coloured) whose linear
    /// luminance exceeds [`FLOOR_HIGHLIGHT`] × the median of their [`FLOOR_CELL`] cell.
    /// Before: the best [`ColorConfig::views_blended`] views blended with their weights,
    /// nothing left out (as in Meilenstein 3); after: the final colour. Same views.
    pub floor_highlight_share_before: f64,
    pub floor_highlight_share_after: f64,
    /// Colour jumps between images: up to [`SEAM_SAMPLES`] planar points, evenly over the
    /// cloud, each with its [`SEAM_NEIGHBOURS`] nearest neighbours within
    /// [`SEAM_RADIUS_VOXELS`] whose normals differ by at most [`SEAM_MAX_NORMAL_ANGLE`]; the
    /// mean absolute luminance difference (sRGB levels) of the pairs whose main images (the
    /// view with the largest blend weight) differ, minus that of the pairs with the same
    /// main image. 0 if either kind is missing. Before and after as for the floor.
    pub seam_jump_before: f64,
    pub seam_jump_after: f64,
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

/// Average ranks (1-based; ties share the mean of their positions) in input order.
fn average_ranks(v: &[f64]) -> Vec<f64> {
    let mut order: Vec<usize> = (0..v.len()).collect();
    order.sort_by(|&a, &b| v[a].total_cmp(&v[b]).then(a.cmp(&b)));
    let mut ranks = vec![0.0; v.len()];
    let mut i = 0;
    while i < order.len() {
        let mut j = i;
        while j + 1 < order.len() && v[order[j + 1]] == v[order[i]] {
            j += 1;
        }
        let rank = 0.5 * ((i + 1 + j + 1) as f64);
        for &o in &order[i..=j] {
            ranks[o] = rank;
        }
        i = j + 1;
    }
    ranks
}

/// Spearman rank correlation (Pearson on average ranks); 0 for fewer than 2 pairs or if one
/// side has no variation.
fn spearman(a: &[f64], b: &[f64]) -> f64 {
    if a.len() != b.len() || a.len() < 2 {
        return 0.0;
    }
    let (ra, rb) = (average_ranks(a), average_ranks(b));
    let n = ra.len() as f64;
    let (ma, mb) = (ra.iter().sum::<f64>() / n, rb.iter().sum::<f64>() / n);
    let (mut cov, mut va, mut vb) = (0.0, 0.0, 0.0);
    for (x, y) in ra.iter().zip(&rb) {
        cov += (x - ma) * (y - mb);
        va += (x - ma) * (x - ma);
        vb += (y - mb) * (y - mb);
    }
    if va == 0.0 || vb == 0.0 {
        0.0
    } else {
        cov / (va * vb).sqrt()
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
    /// (sharpness, angular rate, mean gray) if the capture time lies on the trajectory.
    quality: Option<(f64, f64, f64)>,
}

/// Pass 1: status of every image of the stream, in stream order.
fn quality_pass(
    images: &ImageSource,
    poses: &[Pose],
    cams: &[ColorCamera],
    cfg: &ColorConfig,
) -> Result<(Vec<ImageRecord>, usize, f64), MapError> {
    let mut found: Vec<Measured> = Vec::new();
    let read = for_each_image(&images.bags, &images.topics, SHARPNESS_DOWNSCALE, |frame| {
        let (camera, stamp, quality) = match frame {
            Err(_) => (None, f64::NAN, None),
            Ok(frame) => match frame_camera(&frame, cams, SHARPNESS_DOWNSCALE) {
                None => (None, frame.stamp, None),
                Some(c) => {
                    let t = frame.stamp + cfg.time_offset;
                    let quality = interpolate_pose(poses, t).map(|_| {
                        let (sharp, mean) = measure(&frame.rgb);
                        (sharp, angular_rate(poses, t), mean)
                    });
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
    let brightness: Vec<f64> = found
        .iter()
        .filter_map(|f| f.quality.map(|q| q.2))
        .collect();
    let corr = spearman(&on_traj, &brightness);
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
                    (Some(_), Some((s, rate, _))) if s < limit || rate > cfg.max_angular_rate => {
                        Status::Blurry
                    }
                    (Some(_), Some((s, _, _))) => Status::Used {
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
    Ok((records, truncated, corr))
}

/// A seam sample and its neighbours (see [`ColorStats::seam_jump_after`]); `u32::MAX` marks
/// a missing neighbour.
type SeamSample = (u32, [u32; SEAM_NEIGHBOURS]);

/// Per point the surface normal (if the neighbourhood is planar) and the spacing (mean
/// distance of the [`SPACING_NEIGHBOURS`] nearest other points; 0 without neighbours), and the
/// seam samples with their neighbours.
#[allow(clippy::type_complexity)]
fn local_geometry(xyz: &[[f32; 3]]) -> (Vec<Option<Vector3<f32>>>, Vec<f32>, Vec<SeamSample>) {
    if xyz.len() < 2 {
        return (vec![None; xyz.len()], vec![0.0; xyz.len()], Vec::new());
    }
    // The tree is built on the cloud's own f32 coordinates: no f64 copy of the cloud.
    let tree: ImmutableKdTree<f32, 3> =
        ImmutableKdTree::new_from_slice(xyz).expect("fewer than 2^32 points");
    let at = |i: u32| Vector3::from(xyz[i as usize].map(f64::from));
    // The point itself comes back with distance 0, hence + 1.
    let k = NonZeroUsize::new((NORMAL_NEIGHBOURS + 1).min(xyz.len())).expect("k > 0");
    let (normals, spacing): (Vec<Option<Vector3<f32>>>, Vec<f32>) = xyz
        .par_iter()
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
        .unzip();

    // Seam samples: evenly over the planar points, neighbours on the same plane.
    let voxel = median(&spacing.iter().map(|&s| f64::from(s)).collect::<Vec<_>>()).unwrap_or(0.0);
    let radius2 = (SEAM_RADIUS_VOXELS * voxel).powi(2) as f32;
    let planar: Vec<u32> = (0..xyz.len() as u32)
        .filter(|&i| normals[i as usize].is_some())
        .collect();
    let take = planar.len().min(SEAM_SAMPLES);
    let k = NonZeroUsize::new((SEAM_NEIGHBOURS + 1).min(xyz.len())).expect("k > 0");
    let min_cos = SEAM_MAX_NORMAL_ANGLE.cos() as f32;
    let seam: Vec<SeamSample> = (0..take)
        .into_par_iter()
        .map(|s| {
            let i = planar[s * planar.len() / take];
            let n = normals[i as usize].expect("planar");
            let mut out = [u32::MAX; SEAM_NEIGHBOURS];
            let nn = tree
                .query(&xyz[i as usize])
                .nearest_n::<SquaredEuclidean<f32>>(k)
                .execute();
            let near = nn.iter().filter(|x| x.item != i).take(SEAM_NEIGHBOURS);
            for (o, x) in out.iter_mut().zip(near) {
                let same_plane =
                    normals[x.item as usize].is_some_and(|m| n.dot(&m).abs() >= min_cos);
                if x.distance <= radius2 && same_plane {
                    *o = x.item;
                }
            }
            (i, out)
        })
        .collect();
    (normals, spacing, seam)
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

/// Linear luminance (Rec. 709).
fn luminance(c: [f32; 3]) -> f64 {
    0.2126 * f64::from(c[0]) + 0.7152 * f64::from(c[1]) + 0.0722 * f64::from(c[2])
}

/// Weighted mean of linear colours; `None` without positive weight.
fn weighted_mean(views: impl Iterator<Item = (f64, [f32; 3])>) -> Option<[f32; 3]> {
    let (mut sum, mut wsum) = ([0.0f64; 3], 0.0f64);
    for (w, c) in views {
        wsum += w;
        for (s, c) in sum.iter_mut().zip(c) {
            *s += w * f64::from(c);
        }
    }
    (wsum > 0.0).then(|| sum.map(|s| (s / wsum) as f32))
}

/// What the blend makes of one point's views.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Mix {
    /// Blended linear colour; `None` without views.
    linear: Option<[f32; 3]>,
    /// Image (rank) of the view with the largest blend weight; `u32::MAX` without views.
    main: u32,
    /// Views rejected as highlights.
    rejected: usize,
    /// Views left out beyond the rim.
    rim: usize,
}

impl Mix {
    fn srgb(&self) -> Option<[u8; 3]> {
        self.linear.map(|c| c.map(to_srgb))
    }

    /// Luminance in sRGB levels (0–255).
    fn srgb_luminance(&self) -> Option<f64> {
        self.linear.map(|c| srgb_value(luminance(c) as f32))
    }
}

/// Meilenstein 3's blend: the best `blended` views (sorted best first) with their weights,
/// nothing left out.
fn hard_mix(slots: &[View], blended: usize) -> Mix {
    let top = slots[..blended.min(slots.len())]
        .iter()
        .filter(|v| v.weight > 0.0);
    Mix {
        linear: weighted_mean(top.clone().map(|v| (f64::from(v.weight), v.linear()))),
        main: top.clone().next().map_or(u32::MAX, |v| v.rank),
        rejected: 0,
        rim: 0,
    }
}

/// The colour of a point from its views (sorted best first, exposure applied): rim views
/// and highlights left out, soft blend (see the module docs).
fn mix(slots: &[View], cfg: &ColorConfig) -> Mix {
    let valid = |v: &View| v.weight > 0.0;
    let inside = |v: &View| valid(v) && f64::from(v.theta()) <= cfg.rim_theta;
    let any_inside = slots.iter().any(inside);
    let candidate = |v: &View| if any_inside { inside(v) } else { valid(v) };
    let rim = if any_inside {
        slots.iter().filter(|v| valid(v) && !inside(v)).count()
    } else {
        0
    };
    // Darkest and second darkest candidate.
    let (mut dark, mut second, mut n) = (f64::INFINITY, f64::INFINITY, 0usize);
    for y in slots
        .iter()
        .filter(|v| candidate(v))
        .map(|v| luminance(v.linear()))
    {
        n += 1;
        if y < dark {
            second = dark;
            dark = y;
        } else if y < second {
            second = y;
        }
    }
    let reference = if n >= 3 { second } else { dark };
    let bright = |v: &View| {
        let y = luminance(v.linear());
        cfg.highlights
            && y > reference * (1.0 + cfg.highlight_ratio)
            && y - reference > cfg.highlight_abs
    };
    let rejected = slots.iter().filter(|v| candidate(v) && bright(v)).count();
    let kept = || slots.iter().filter(|v| candidate(v) && !bright(v));
    let b = cfg.views_blended;
    let next = kept().nth(b).map_or(0.0, |v| f64::from(v.weight));
    let soft = kept().take(b).any(|v| f64::from(v.weight) > next);
    let weight = |v: &View| f64::from(v.weight) - if soft { next } else { 0.0 };
    Mix {
        linear: weighted_mean(kept().take(b).map(|v| (weight(v), v.linear()))),
        main: kept().next().map_or(u32::MAX, |v| v.rank),
        rejected,
        rim,
    }
}

/// Share of the points with a luminance above [`FLOOR_HIGHLIGHT`] × their cell's median, from
/// (cell, luminance) entries; sorts them.
fn highlight_share(entries: &mut [([i64; 2], f32)]) -> f64 {
    if entries.is_empty() {
        return 0.0;
    }
    entries.par_sort_unstable_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let mut bright = 0usize;
    for cell in entries.chunk_by(|a, b| a.0 == b.0) {
        let m = cell.len();
        let median = if m % 2 == 1 {
            f64::from(cell[m / 2].1)
        } else {
            0.5 * (f64::from(cell[m / 2 - 1].1) + f64::from(cell[m / 2].1))
        };
        bright += cell
            .iter()
            .filter(|e| f64::from(e.1) > FLOOR_HIGHLIGHT * median)
            .count();
    }
    bright as f64 / entries.len() as f64
}

/// [`ColorStats::floor_highlight_share_before`] and `_after` of the points `floor`.
fn floor_highlight_shares(
    slots: &[View],
    k: usize,
    cfg: &ColorConfig,
    xyz: &[[f32; 3]],
    floor: &[u32],
) -> (f64, f64) {
    let cell = |c: f32| (f64::from(c) / FLOOR_CELL).floor() as i64;
    let entries: Vec<([i64; 2], f32, f32)> = floor
        .par_iter()
        .filter_map(|&i| {
            let i = i as usize;
            let s = &slots[i * k..(i + 1) * k];
            let before = hard_mix(s, cfg.views_blended).linear?;
            let after = mix(s, cfg).linear?;
            let p = xyz[i];
            Some((
                [cell(p[0]), cell(p[1])],
                luminance(before) as f32,
                luminance(after) as f32,
            ))
        })
        .collect();
    let mut column: Vec<([i64; 2], f32)> = entries.iter().map(|e| (e.0, e.1)).collect();
    let before = highlight_share(&mut column);
    // Refilled entry by entry (the cell and the luminance of the same point).
    for (c, e) in column.iter_mut().zip(&entries) {
        *c = (e.0, e.2);
    }
    drop(entries);
    (before, highlight_share(&mut column))
}

/// [`ColorStats::seam_jump_before`] and `_after` over the seam samples.
fn seam_jumps(slots: &[View], k: usize, cfg: &ColorConfig, seam: &[SeamSample]) -> (f64, f64) {
    let point = |i: u32| &slots[i as usize * k..(i as usize + 1) * k];
    // Per sample: (summed |ΔY|, pairs) with the same and with different main images, before
    // and after; summed in sample order.
    let partial: Vec<[(f64, usize); 4]> = seam
        .par_iter()
        .map(|&(i, near)| {
            let mut sums = [(0.0, 0usize); 4];
            let (hb, ha) = (hard_mix(point(i), cfg.views_blended), mix(point(i), cfg));
            for &j in near.iter().filter(|&&j| j != u32::MAX) {
                let (nb, na) = (hard_mix(point(j), cfg.views_blended), mix(point(j), cfg));
                for (slot, (a, b)) in [(0, (hb, nb)), (2, (ha, na))] {
                    if let (Some(ya), Some(yb)) = (a.srgb_luminance(), b.srgb_luminance()) {
                        let e = &mut sums[slot + usize::from(a.main != b.main)];
                        e.0 += (ya - yb).abs();
                        e.1 += 1;
                    }
                }
            }
            sums
        })
        .collect();
    let mut sums = [(0.0, 0usize); 4];
    for p in &partial {
        for (t, p) in sums.iter_mut().zip(p) {
            t.0 += p.0;
            t.1 += p.1;
        }
    }
    let jump = |same: (f64, usize), diff: (f64, usize)| {
        if same.1 == 0 || diff.1 == 0 {
            0.0
        } else {
            diff.0 / diff.1 as f64 - same.0 / same.1 as f64
        }
    };
    (jump(sums[0], sums[1]), jump(sums[2], sums[3]))
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
    let (records, bags_truncated, rank_corr) = quality_pass(&images, poses, cams, cfg)?;
    let count = |f: fn(&Status) -> bool| records.iter().filter(|r| f(&r.status)).count();
    let mut stats = ColorStats {
        images_used: count(|s| matches!(s, Status::Used { .. })),
        images_blurry: count(|s| *s == Status::Blurry),
        images_outside_trajectory: count(|s| *s == Status::Outside),
        images_bad: count(|s| *s == Status::Bad),
        image_bags_truncated: bags_truncated,
        sharpness_brightness_rank_corr: rank_corr,
        ..ColorStats::default()
    };

    let k = cfg.views_kept;
    let last_used = records
        .iter()
        .rposition(|r| matches!(r.status, Status::Used { .. }));
    // Allocated after `local_geometry`, whose k-d tree is the memory peak of the setup.
    let mut slots = Vec::new();
    // Per point: whether its normal is nearly vertical (floor or ceiling).
    let mut horizontal: Vec<bool> = Vec::new();
    let mut seam: Vec<SeamSample> = Vec::new();
    if let (Some(last_used), false) = (last_used, cloud.is_empty()) {
        let (normals, spacing, samples) = local_geometry(&cloud.xyz);
        seam = samples;
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
        horizontal = normals
            .iter()
            .map(|n| n.is_some_and(|n| n.z.abs() > FLOOR_NORMAL_Z))
            .collect();
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
    let (gains, vignetting) = if cfg.exposure.samples == 0 {
        (
            vec![[1.0; 3]; used.len()],
            Vignetting::flat(cams.len(), cfg.exposure.max_gain),
        )
    } else {
        let obs = sample_observations(&slots, k, cameras.clone(), cfg.exposure.samples);
        let (gains, mut vignetting) = estimate_exposure(&obs, &cfg.exposure);
        vignetting.coeffs.resize(cams.len(), [[0.0; 2]; 3]);
        (gains, vignetting)
    };
    let flat = Vignetting::flat(cams.len(), cfg.exposure.max_gain);
    stats.vignette_residual_before = vignette_residual(&slots, k, &gains, &flat, &cameras);
    stats.vignette_residual_after = vignette_residual(&slots, k, &gains, &vignetting, &cameras);
    apply_exposure(&mut slots, &gains, &vignetting, &cameras);
    stats.spread_after = view_spread(&slots, k);
    let after = camera_difference(&slots, k, &cameras);
    stats.image_offset_after = image_offset(&slots, k, used.len());
    stats.camera_difference_before = before.value;
    stats.camera_difference_after = after.value;
    stats.camera_difference_points = before.points;
    let all = gains.iter().flatten();
    stats.gain_min = all.clone().copied().reduce(f32::min).unwrap_or(1.0);
    stats.gain_max = all.copied().reduce(f32::max).unwrap_or(1.0);
    let (hi, lo) = (
        cfg.exposure.max_gain as f32,
        (1.0 / cfg.exposure.max_gain) as f32,
    );
    stats.images_gain_clamped = gains
        .iter()
        .filter(|g| g.iter().any(|&x| x >= hi || x <= lo))
        .count();
    stats.vignetting = vignetting;
    stats.gains = used
        .iter()
        .zip(&gains)
        .map(|(r, g)| ImageGain {
            camera: cams[r.camera].camera.name.clone(),
            stamp: r.stamp,
            gain: *g,
        })
        .collect();

    // Highlights and seams, before (Meilenstein 3's blend) and after.
    let pose_z: Vec<f64> = poses.iter().map(|p| p.p.z).collect();
    let floor_z = median(&pose_z).unwrap_or(0.0) - FLOOR_BELOW_TRAJECTORY;
    let floor: Vec<u32> = (0..horizontal.len() as u32)
        .filter(|&i| horizontal[i as usize] && f64::from(cloud.xyz[i as usize][2]) < floor_z)
        .collect();
    drop(horizontal);
    (
        stats.floor_highlight_share_before,
        stats.floor_highlight_share_after,
    ) = floor_highlight_shares(&slots, k, cfg, &cloud.xyz, &floor);
    drop(floor);
    (stats.seam_jump_before, stats.seam_jump_after) = seam_jumps(&slots, k, cfg, &seam);
    drop(seam);
    (
        stats.views_rejected,
        stats.points_with_rejected,
        stats.views_rim_excluded,
    ) = slots
        .par_chunks(k)
        .map(|s| {
            let m = mix(s, cfg);
            (m.rejected, usize::from(m.rejected > 0), m.rim)
        })
        .reduce(|| (0, 0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2));

    let grays = intensity_grays(&cloud.intensity);
    let (rgb, colored): (Vec<[u8; 3]>, Vec<bool>) = slots
        .par_chunks(k)
        .zip(grays.par_iter())
        .map(|(s, &g)| mix(s, cfg).srgb().map_or(([g; 3], false), |c| (c, true)))
        .unzip();
    stats.points_colored = colored.iter().filter(|&&c| c).count();
    stats.points_uncolored = colored.len() - stats.points_colored;
    cloud.rgb = Some(rgb);
    cloud.colored = Some(colored);
    Ok(stats)
}

#[cfg(test)]
mod tests {
    #[test]
    fn rank_corr_of_brightness_ramp() {
        let ramp: Vec<f64> = (0..20).map(f64::from).collect();
        let rev: Vec<f64> = ramp.iter().rev().copied().collect();
        // Monotone (even non-linear) relations give exactly +1 / -1.
        let cubed: Vec<f64> = ramp.iter().map(|x| x * x * x).collect();
        assert!((super::spearman(&ramp, &cubed) - 1.0).abs() < 1e-12);
        assert!((super::spearman(&ramp, &rev) + 1.0).abs() < 1e-12);
        // Ties get average ranks.
        assert_eq!(
            super::average_ranks(&[5.0, 1.0, 5.0, 5.0, 2.0]),
            vec![4.0, 1.0, 4.0, 4.0, 2.0]
        );
        // Symmetric (non-monotone) relation: 0.
        let sym = [-2.0, -1.0, 0.0, 1.0, 2.0];
        let sq: Vec<f64> = sym.iter().map(|x| x * x).collect();
        assert!(super::spearman(&sym, &sq).abs() < 1e-12);
        // Degenerate input.
        assert_eq!(super::spearman(&[1.0], &[2.0]), 0.0);
        assert_eq!(super::spearman(&[1.0, 1.0, 1.0], &[1.0, 2.0, 3.0]), 0.0);
        assert_eq!(super::spearman(&[], &[]), 0.0);
    }

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
        let blend = |s: &[View], b: usize| hard_mix(s, b).srgb();
        // Black and white mix to linear 0.5, i.e. sRGB 188, not 128.
        assert_eq!(blend(&[view(0), view(255), EMPTY], 3), Some([188; 3]));
        assert_eq!(blend(&[EMPTY; 2], 2), None);
        // Only the best `blended` views count.
        assert_eq!(blend(&[view(255), view(0), EMPTY], 1), Some([255; 3]));
        // Equal weights: the soft blend keeps them.
        let cfg = ColorConfig {
            highlights: false,
            ..ColorConfig::default()
        };
        assert_eq!(
            mix(&[view(0), view(255), EMPTY], &cfg).srgb(),
            Some([188; 3])
        );
    }

    /// A view of linear gray `y` with weight `w` from image `rank` at `theta_deg`.
    fn gray(w: f32, rank: u32, y: f32, theta_deg: f32) -> View {
        View::new(w, rank, [y; 3], theta_deg.to_radians())
    }

    #[test]
    fn highlights_are_rejected_against_the_second_darkest() {
        let cfg = ColorConfig::default();
        // Two of five views are highlights (the best two); the rest show 0.2.
        let s = [
            gray(1.0, 0, 0.6, 0.0),
            gray(0.9, 1, 0.5, 0.0),
            gray(0.8, 2, 0.2, 0.0),
            gray(0.7, 3, 0.21, 0.0),
            gray(0.6, 4, 0.2, 0.0),
            EMPTY,
        ];
        let m = mix(&s, &cfg);
        assert_eq!(m.rejected, 2);
        assert_eq!(m.main, 2);
        assert!((m.linear.unwrap()[0] - 0.2).abs() < 0.01, "{m:?}");
        // Slightly brighter, or bright by little in absolute terms: kept.
        let dim = [gray(1.0, 0, 0.012, 0.0), gray(0.9, 1, 0.01, 0.0), EMPTY];
        assert_eq!(mix(&dim, &cfg).rejected, 0);
        let near = [gray(1.0, 0, 0.25, 0.0), gray(0.9, 1, 0.2, 0.0), EMPTY];
        assert_eq!(mix(&near, &cfg).rejected, 0);
        // Switched off: nothing rejected.
        let off = ColorConfig {
            highlights: false,
            ..ColorConfig::default()
        };
        assert_eq!(mix(&s, &off).rejected, 0);
    }

    #[test]
    fn few_views_are_never_all_rejected() {
        let cfg = ColorConfig::default();
        let one = [gray(1.0, 0, 0.9, 0.0), EMPTY, EMPTY];
        let m = mix(&one, &cfg);
        assert_eq!((m.rejected, m.main), (0, 0));
        assert!(m.linear.is_some());
        // Two views: the darker is the reference and stays.
        let two = [gray(1.0, 0, 0.9, 0.0), gray(0.5, 1, 0.1, 0.0), EMPTY];
        let m = mix(&two, &cfg);
        assert_eq!((m.rejected, m.main), (1, 1));
        assert!((m.linear.unwrap()[0] - 0.1).abs() < 1e-3, "{m:?}");
    }

    #[test]
    fn dark_outlier_does_not_become_reference() {
        let cfg = ColorConfig::default();
        let mut s = [gray(0.5, 0, 0.3, 0.0); 6];
        for (r, v) in s.iter_mut().enumerate() {
            v.rank = r as u32;
            v.weight = 1.0 - 0.1 * r as f32;
        }
        // The best view sees the operator's shadow.
        s[0].set_linear([0.01; 3]);
        assert_eq!(mix(&s, &cfg).rejected, 0);
    }

    #[test]
    fn soft_blend_is_continuous() {
        let cfg = ColorConfig::default();
        let eps = 1e-4;
        // The 3rd and 4th view swap places between two neighbouring points.
        let a = [
            gray(1.0, 0, 0.3, 0.0),
            gray(0.9, 1, 0.3, 0.0),
            View::new(0.5, 2, [0.6, 0.3, 0.3], 0.0),
            View::new(0.5 - eps, 3, [0.3, 0.3, 0.6], 0.0),
            EMPTY,
            EMPTY,
        ];
        let mut b = a;
        b.swap(2, 3);
        b[2].weight = 0.5;
        b[3].weight = 0.5 - eps;
        let (ma, mb) = (mix(&a, &cfg), mix(&b, &cfg));
        let (ca, cb) = (ma.linear.unwrap(), mb.linear.unwrap());
        for c in 0..3 {
            assert!((ca[c] - cb[c]).abs() < 1.0 / 255.0, "{ca:?} vs {cb:?}");
        }
        // The hard cut jumps.
        let (ha, hb) = (
            hard_mix(&a, 3).linear.unwrap(),
            hard_mix(&b, 3).linear.unwrap(),
        );
        assert!((ha[0] - hb[0]).abs() > 0.05, "{ha:?} vs {hb:?}");
    }

    #[test]
    fn rim_views_only_used_when_nothing_else() {
        let cfg = ColorConfig::default();
        // The rim view is the best and the darkest: left out of blend and reference.
        let s = [
            gray(1.0, 0, 0.05, 92.0),
            gray(0.5, 1, 0.3, 30.0),
            gray(0.4, 2, 0.3, 40.0),
            EMPTY,
        ];
        let m = mix(&s, &cfg);
        assert_eq!((m.rim, m.rejected, m.main), (1, 0, 1));
        assert!((m.linear.unwrap()[0] - 0.3).abs() < 1e-3, "{m:?}");
        // Only rim views: used as before, not counted.
        let rim = [
            gray(1.0, 0, 0.05, 92.0),
            gray(0.5, 1, 0.06, 95.0),
            EMPTY,
            EMPTY,
        ];
        let m = mix(&rim, &cfg);
        assert_eq!((m.rim, m.main), (0, 0));
        assert!(m.linear.is_some());
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
