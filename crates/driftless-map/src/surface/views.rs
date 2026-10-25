//! Which camera images show each face of the mesh, and how well: the input of the texture.
//!
//! Two passes over the images, both decoded at 1/[`SHARPNESS_DOWNSCALE`] (the views need only
//! the geometry and a colour sample for the exposure):
//!
//! 1. **Quality**, as for colouring (see [`crate::color`]): sharpness and angular rate decide
//!    which images are used; the used ones are ranked in (capture time, camera name) order.
//! 2. **Views**: per used image a depth buffer of the faces near the camera (cells of
//!    [`ColorConfig::zbuffer_scale`] pixels of the image at 1/[`ColorConfig::downscale`]),
//!    splatted from the centroid and the corners of every face. A face that would cover more
//!    than [`SAMPLE_GAP_CELLS`] cells between its samples gets a barycentric grid of up to
//!    [`MAX_SUBDIVISIONS`] rows instead, so that nothing behind it shows through. The samples
//!    then lie at most [`SAMPLE_GAP_CELLS`] cells apart as long as the face's nearest point is
//!    at least `longest edge · focal / (SAMPLE_GAP_CELLS · MAX_SUBDIVISIONS)` away (`focal` in
//!    cells per radian; with [`SurfaceConfig::max_edge`] 0.3 m, a lens of about 600 px/rad and
//!    `zbuffer_scale` 4: about 0.7 m); nearer, the grid is capped and gaps can open. A face is
//!    seen if its centroid lies within [`ColorConfig::max_dist`], its centroid and all three
//!    corners pass the depth buffer (a partly hidden face is not seen), its normal points
//!    towards the camera and its corners lie in the image within [`ColorConfig::rim_theta`] of
//!    the optical axis. Seen at a grazing angle (beyond about 75°), a face is often hidden by
//!    the nearer part of its own wall in the same depth cell, as in colouring. Its score is
//!    `projected area (pixels at full size) × sharpness_norm × cos(angle to the normal)`,
//!    the area capped at `face area / min_texel²` ([`SurfaceConfig::min_texel`]): the texture
//!    is never finer than that, so a nearer image is no better. The normal here is smoothed
//!    over the neighbourhood ([`NORMAL_SMOOTH_ROUNDS`]; the area scaled by the ratio of the
//!    cosines, within ½ … 2): the normals of a scanned surface scatter by tens of degrees from
//!    face to face, and with the face's own normal neighbouring faces ranked the images
//!    differently and kept different ones (black room: 57 % of a neighbour's views in common,
//!    with the smoothed normal 79 %, and 2.5 times fewer charts). Visibility and facing still
//!    use the face's own normal.
//!    Each face keeps its [`SurfaceConfig::views_per_face`] best views (higher score first;
//!    equal scores: the earlier image).
//!
//! **The carrier** of the device stands next to it and below the camera: a view looking
//! steeply down (horizontal distance below both [`CARRIER_RADIUS`] and the height below the
//! camera, world Z up) at a face that faces up (normal `z` above [`CARRIER_NORMAL_Z`]) and lies
//! more than [`CARRIER_BELOW`] below the camera may show the carrier's legs or shoes instead of
//! the floor. Such a view is tagged ([`FaceView::carrier`]) and ranks as if its score were
//! [`CARRIER_ALTERNATIVE`] of its own. After the pass ([`filter_views`]) the tagged views of a
//! face are dropped if its best untagged view scores at least [`CARRIER_ALTERNATIVE`] of its
//! best tagged one, so the floor along the walking path is textured from farther away where
//! there is a decent view; otherwise they stay. (If the tagged views were pushed out of the
//! slots by untagged ones, each of those scores above that share: the same decision.) Walls,
//! desk edges and steps' risers near the path are not tagged.
//!
//! **Occlusion mask** ([`mask_views`], after the exposure fit): what moves with the device –
//! the carrier's legs, a strap – sits in the same part of a camera's image in every frame, also
//! farther than the carrier rule reaches. Every view of a face with at least
//! [`MASK_JUDGE_VIEWS`] views is judged against the median of them (as in the consistency
//! check below), and counted in its cell of its camera's image ([`MASK_CELLS`]) for its
//! image. The statistic is per image, not per view (one frame shows hundreds of faces in a
//! cell): an image disagrees in a cell if most of its judged views there disagree. A cell
//! with at least [`MASK_MIN_VIEWS`] judged views from at least [`MASK_MIN_IMAGES`] images, of
//! which more than [`MASK_SHARE`] disagree, is masked; the views in masked cells are dropped
//! from faces that have a decent other view (as for the carrier). A camera with more than
//! [`MASK_MAX_CELLS`] such cells is not masked at all (and warned about).
//!
//! Black room (VM, 133 / 134 cells with enough images per camera): camera "right" 31–44 % of
//! the images disagree in the 7 lower-left cells (the carrier), at most 29 % elsewhere (p90
//! 22 %); camera "left" at most 24 % (p90 18 %), nothing masked. Büro: 9 cells (counted per
//! view). The cap is 3–4 times what both scans need. The median can be the carrier: where
//! the carrier is in most views of a face (along the walking path), the clean views count as
//! the outliers, in their cells. In the black room that is the case for 6,092 of 698,527
//! judged faces (0.9 %; 4,450 have at least half their views in masked cells), too few to
//! push a clean cell over [`MASK_SHARE`]; such faces lose their masked views where a decent
//! other view exists. The margin to the next cell (29 % vs. 30 %) is thin; those cells lie next
//! to the carrier's.
//!
//! **Photo consistency**: every view also keeps the mean colour of its face in the image
//! ([`FaceView::colour`]: the centroid and the three points halfway to the corners, bilinear
//! in the image at 1/[`SHARPNESS_DOWNSCALE`], linear mean, as sRGB) and the centroid's angle
//! from the optical axis. After the exposure fit, a face with at least
//! [`CONSISTENCY_MIN_VIEWS`] views compares their colours divided by gain and edge falloff
//! (sRGB). The median is taken over votes, not views: per camera and stretch of
//! [`VOTE_GAP`] of capture time (`⌊stamp / VOTE_GAP⌋`) only the best view votes, since a
//! person standing still for a few frames would otherwise outvote the bare wall. A view further
//! than [`PHOTO_TOLERANCE`] in any channel from the median shows something the mesh does not
//! have (a person walking by, the carrier, a reflection) and is dropped, but only if the votes
//! within the tolerance are a strict majority: with a 2/2 split (or no vote near the median)
//! nothing says which side is right, and all views stay. The views' order is kept; at most
//! [`MAX_VIEWS_PER_FACE`] per face. The exposure fit runs before this check (it needs the
//! gains), so its sample still contains the dropped views.
//!
//! For a port of the pass (GPU): the score uses the smoothed normals (computed once on the
//! CPU, [`NORMAL_SMOOTH_ROUNDS`]). It has to fill [`ViewTable::per_face`] as
//! [`keep_best_view`] does: [`FaceView`] has four fields (`image`, `score`, `colour`, `tag`;
//! 12 bytes), `tag` holds the carrier bit (7) and the centroid's angle from the optical axis
//! quantised to steps of π/127 (bits 0–6, rounded, at most 127); the slot order is
//! [`FaceView::better_than`]: a view before an empty slot, then the higher score with a tagged
//! view's score counted at [`CARRIER_ALTERNATIVE`], then the earlier image; a view goes in
//! only if it beats the last slot. [`filter_views`] then runs on the result as it stands.
//!
//! Exposure and white balance: every `k`-th face (at most [`EXPOSURE_FACES`] and
//! [`crate::ExposureConfig::samples`]) also keeps the colour (bilinear, linear RGB, at its
//! centroid) and the angle from the optical axis of [`SurfaceConfig::views_per_face`] of its
//! views (not of the carrier); from these, [`crate::exposure::estimate_exposure`] fits the
//! gains and the edge falloff as in colouring. The sample is chosen so that it does not hang
//! on rounding: the GPU's scores differ from the CPU's in the last bits, and consecutive
//! images of a face score nearly the same.
//! - Which views a sample face keeps depends on the score only coarsely ([`sample_key`]): the
//!   views of the highest half octaves of score (`⌊2 · log₂ score⌋`) are kept, and within a
//!   half octave a hash of face and image decides (equal: the earlier image). By the exact
//!   score, a ±1e-4 change of the scores swapped views at the last slot, and the gains of
//!   single images moved by up to 4.7 % between CPU and GPU (black room, 2,145 images). By
//!   the hash alone (no score), the fit also took grazing and far views, and the seams before
//!   levelling got 6 % worse (black room `seam_step_before` 26.2 → 27.7).
//! - There are at most as many sample faces as the fit takes, so it takes every one with views
//!   from 2 images. With more, it took every `m / samples`-th of the `m` such faces, and one
//!   face more or less with 2 views shifted the whole selection.
//!
//! Unlike colouring (which fits on [`ColorConfig::views_kept`] views per point, at most one
//! per camera and [`ColorConfig::view_gap`]), the fit sees the
//! [`SurfaceConfig::views_per_face`] best views by half octave of score.

use super::{Mesh, SurfaceConfig};
use crate::color::{
    frame_camera, keep_best, linear_table, quality_pass, sample_linear, srgb_value, ColorCamera,
    ColorConfig, ImageRecord, ImageSource, Status, View, SHARPNESS_DOWNSCALE,
};
use crate::color_gpu::GpuStop;
use crate::exposure::{estimate_exposure, sample_observations, Vignetting};
use crate::visibility::{project_point, DepthBuffer, PointGrid};
use crate::MapError;
use driftless_gpu::GpuContext;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, ImageError, RgbImage};
use driftless_io::progress::Progress;
use driftless_traj::{interpolate_pose, Pose};
use nalgebra::{Isometry3, Point3, Vector3};
use rayon::prelude::*;

/// At most this many faces (every `k`-th) keep the colours of their views for the exposure fit.
pub const EXPOSURE_FACES: usize = 200_000;
/// The depth-buffer samples of a face are meant to lie at most this many cells apart in the
/// image (see the module docs for when the cap [`MAX_SUBDIVISIONS`] prevents it).
pub const SAMPLE_GAP_CELLS: f64 = 2.0;
/// A face is split into at most this many sample rows for the depth buffer.
pub const MAX_SUBDIVISIONS: usize = 32;
/// Rounds of smoothing the normal for the score (face → vertices, area-weighted → face).
pub const NORMAL_SMOOTH_ROUNDS: usize = 2;
/// The smoothed normal's cosine counts as at least this (it can face away at thin parts).
pub(crate) const SMOOTH_COS_MIN: f64 = 0.05;
/// A view of a face whose centroid lies within this horizontal distance (m) of the camera …
pub const CARRIER_RADIUS: f64 = 1.2;
/// … and more than this far below it (m) may show the carrier of the device (see the module
/// docs) …
pub const CARRIER_BELOW: f64 = 0.5;
/// … if the face faces up: its unit normal's `z` is above this (45°: floor, not wall).
pub const CARRIER_NORMAL_Z: f32 = 0.7;
/// A tagged view counts as this share of its score in the slot order, and the tagged views of
/// a face are dropped only if its best untagged view reaches this share of the best tagged one.
pub const CARRIER_ALTERNATIVE: f32 = 0.25;
/// Photo consistency: per camera, one vote per stretch of this much capture time (s).
pub const VOTE_GAP: f64 = 1.0;
/// The occlusion mask divides every camera image into this many cells (columns, rows).
pub const MASK_CELLS: [usize; 2] = [16, 12];
/// A face with at least this many views judges them for the mask (median of all of them).
pub const MASK_JUDGE_VIEWS: usize = 4;
/// A cell is masked if at least this many views were judged in it …
pub const MASK_MIN_VIEWS: u64 = 200;
/// … from at least this many different images (one frame shows hundreds of faces in a cell:
/// a person passing once must not mask it) …
pub const MASK_MIN_IMAGES: usize = 20;
/// … and in more than this share of those images most of the cell's views lay further than
/// [`PHOTO_TOLERANCE`] from their face's median (black room: carrier 31–44 %, elsewhere at
/// most 29 %).
pub const MASK_SHARE: f64 = 0.3;
/// A camera with more masked cells than this (an eighth of [`MASK_CELLS`]) is not masked at
/// all: that many cells point at something else than a carrier (a crowd, monitors, a glossy
/// floor). The black room masks 7, the Büro 9.
pub const MASK_MAX_CELLS: usize = 24;
/// The mask statistics are summed over this many face ranges (one counter table each).
const MASK_RANGES: usize = 16;
/// Most views per face (`SurfaceConfig::views_per_face`); the consistency check works on
/// fixed arrays of this size.
pub const MAX_VIEWS_PER_FACE: usize = 32;
/// [`FaceView::tag`] bit of a view that may show the carrier.
const TAG_CARRIER: u8 = 0x80;
/// [`FaceView::tag`] bits 0–6: the angle from the optical axis in steps of this (rad).
pub(crate) const ANGLE_STEP: f32 = std::f32::consts::PI / 127.0;
/// Faces with at least this many views are checked for photo consistency (see the module
/// docs): with fewer the median is no majority.
pub const CONSISTENCY_MIN_VIEWS: usize = 3;
/// A view whose colour differs from the median of its face's views by more than this in any
/// channel (sRGB levels, after the exposure correction) is dropped. The views of a point
/// spread by 10.6 (black room) and 13.5 (Büro) sRGB levels (standard deviation per channel
/// after the exposure correction, `spread_after` of the colour step): 30 is about 2.5 of
/// those; a person or a shoe differs by 40 and more.
pub const PHOTO_TOLERANCE: f64 = 30.0;

/// The camera images of one scan with its trajectory and cameras.
#[derive(Debug, Clone, PartialEq)]
pub struct TextureSource {
    pub images: ImageSource,
    /// Body trajectory (e.g. the SLAM `traj.tum`).
    pub poses: Vec<Pose>,
    /// Every camera with its mount.
    pub cams: Vec<ColorCamera>,
}

/// One view of a face; `score == 0` marks an empty slot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FaceView {
    /// Rank of the image: index into [`ViewTable::images`].
    pub image: u32,
    pub score: f32,
    /// Mean colour of the face in the image (sRGB, before the exposure correction; see the
    /// module docs).
    pub colour: [u8; 3],
    /// Bit 7 ([`FaceView::carrier`]): the view may show the carrier of the device. Bits 0–6:
    /// the centroid's angle from the optical axis in steps of [`ANGLE_STEP`] (π/127, up to π).
    pub tag: u8,
}

impl FaceView {
    pub(crate) const EMPTY: FaceView = FaceView {
        image: u32::MAX,
        score: 0.0,
        colour: [0; 3],
        tag: 0,
    };

    /// Whether the view may show the carrier of the device (see the module docs).
    pub(crate) fn carrier(&self) -> bool {
        self.tag & TAG_CARRIER != 0
    }

    /// The centroid's angle from the optical axis (rad, in steps of [`ANGLE_STEP`]).
    pub(crate) fn theta(&self) -> f32 {
        f32::from(self.tag & !TAG_CARRIER) * ANGLE_STEP
    }

    /// Its score in the slot order: a [`FaceView::carrier`] view's counts at
    /// [`CARRIER_ALTERNATIVE`].
    fn rank_score(&self) -> f32 {
        if self.carrier() {
            self.score * CARRIER_ALTERNATIVE
        } else {
            self.score
        }
    }

    /// A view before an empty slot; then the higher [`FaceView::rank_score`]; equal: the
    /// earlier image.
    pub(crate) fn better_than(&self, other: &FaceView) -> bool {
        if (self.score > 0.0) != (other.score > 0.0) {
            return self.score > 0.0;
        }
        let (a, b) = (self.rank_score(), other.rank_score());
        a > b || (a == b && self.image < other.image)
    }
}

/// Inserts `v` into a face's views (best first) if it is among the best.
fn keep_best_view(slots: &mut [FaceView], v: FaceView) {
    // Most views are not among the best: one comparison with the last slot.
    if !slots.last().is_some_and(|s| v.better_than(s)) {
        return;
    }
    if let Some(j) = slots.iter().position(|s| v.better_than(s)) {
        slots[j..].rotate_right(1);
        slots[j] = v;
    }
}

/// lowbias32 (C. Wellons): a 32-bit integer hash; the GPU pass has the same.
pub(crate) fn hash32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^ (x >> 16)
}

/// Mantissa bits of √2: an `f32` at or above `2^e · √2` has at least these.
const HALF_OCTAVE_MANTISSA: u32 = 0x3504f3;

/// Weight of image `rank`'s view with `score` in exposure sample `slot` (see the module docs):
/// the score's half octave (`⌊2 · log₂ score⌋`, from the bits, exact) as the exponent and a
/// hash of sample and image as the mantissa. Higher half octaves first; within one, the hash
/// decides, not the last bits of the score. A positive normal `f32`; 0 for a score ≤ 0.
pub(crate) fn sample_key(slot: u32, rank: u32, score: f32) -> f32 {
    if score.is_nan() || score <= 0.0 {
        return 0.0;
    }
    let bits = score.to_bits();
    let octave = ((bits >> 23) & 0xff) as i32 - 127;
    let band = 2 * octave + i32::from(bits & 0x7f_ffff >= HALF_OCTAVE_MANTISSA);
    // Exponent field 1 … 254: half octaves −126 … 127 (scores from about 2⁻⁶³ to 2⁶³).
    let exponent = (band.clamp(-126, 127) + 127) as u32;
    f32::from_bits(exponent << 23 | hash32(rank ^ hash32(slot)) >> 9)
}

/// Inserts image `v.rank`'s view into exposure sample `slot`'s views: [`keep_best`] by
/// [`sample_key`] instead of the score, so that which views a sample keeps does not hang on
/// the last bits of near-equal scores.
fn keep_sample(slots: &mut [View], slot: usize, v: View) {
    let weight = sample_key(slot as u32, v.rank, v.weight);
    keep_best(slots, View { weight, ..v });
}

/// A used image.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct UsedImage {
    /// Index into [`TextureSource::cams`].
    pub camera: usize,
    /// Image stamp (s), as in the bag.
    pub stamp: f64,
    /// Camera pose in the world: `trajectory(stamp + time_offset) · cam_in_body`.
    pub world_from_cam: Isometry3<f64>,
}

/// The views of every face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ViewTable {
    /// The used images by rank: (capture time, camera name) order.
    pub images: Vec<UsedImage>,
    /// [`SurfaceConfig::views_per_face`] views per face, best first; empty slots (score 0)
    /// at the end.
    pub per_face: Vec<FaceView>,
    /// Exposure and white-balance gain of every used image (by rank, linear RGB): divide an
    /// image's colours by it.
    pub gains: Vec<[f32; 3]>,
    /// Edge falloff of every camera (numbered as [`TextureSource::cams`]).
    pub vignetting: Vignetting,
}

impl ViewTable {
    /// Rank of the image of `camera` stamped `stamp`, if it is used.
    pub(crate) fn rank_of(&self, camera: usize, stamp: f64) -> Option<u32> {
        let start = self.images.partition_point(|i| i.stamp < stamp);
        self.images[start..]
            .iter()
            .take_while(|i| i.stamp == stamp)
            .position(|i| i.camera == camera)
            .map(|j| u32::try_from(start + j).expect("fewer than 2^32 images"))
    }
}

/// Per face: centroid, unit normal and longest edge; and the largest distance from a centroid
/// to a corner.
pub(crate) struct Faces {
    pub centroid: Vec<[f32; 3]>,
    pub normal: Vec<[f32; 3]>,
    /// [`NORMAL_SMOOTH_ROUNDS`] rounds smoothed (for the score; zero where it cancels out).
    pub smooth: Vec<[f32; 3]>,
    pub longest: Vec<f32>,
    pub reach: f64,
}

impl Faces {
    fn new(mesh: &Mesh) -> Faces {
        let corners = |f: usize| mesh.indices[f].map(|i| Vector3::from(mesh.positions[i as usize]));
        let (centroid, longest): (Vec<[f32; 3]>, Vec<f32>) = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| {
                let [a, b, c] = corners(f);
                let m = (a + b + c) / 3.0;
                let edge = (b - a).norm().max((c - b).norm()).max((a - c).norm());
                ([m.x, m.y, m.z], edge)
            })
            .unzip();
        let reach = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| {
                let m = Vector3::from(centroid[f]);
                corners(f)
                    .iter()
                    .map(|c| f64::from((c - m).norm()))
                    .fold(0.0, f64::max)
            })
            .reduce(|| 0.0, f64::max);
        let normal: Vec<[f32; 3]> = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| mesh.face_normal(f))
            .collect();
        let area: Vec<f32> = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| mesh.face_area(f))
            .collect();
        let mut smooth = normal.clone();
        for _ in 0..NORMAL_SMOOTH_ROUNDS {
            let mut at_vertex = vec![[0.0f32; 3]; mesh.positions.len()];
            for (f, t) in mesh.indices.iter().enumerate() {
                for &v in t {
                    for k in 0..3 {
                        at_vertex[v as usize][k] += smooth[f][k] * area[f];
                    }
                }
            }
            smooth = mesh
                .indices
                .par_iter()
                .map(|t| {
                    let s: [f32; 3] = std::array::from_fn(|k| {
                        t.iter().map(|&v| at_vertex[v as usize][k]).sum::<f32>()
                    });
                    let l = (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt();
                    if l > 0.0 {
                        s.map(|x| x / l)
                    } else {
                        [0.0; 3]
                    }
                })
                .collect();
        }
        Faces {
            centroid,
            normal,
            smooth,
            longest,
            reach,
        }
    }
}

/// What one image contributes: per seen face its view, and for an exposure sample face also the
/// view with its colour.
type Found = Vec<(u32, FaceView, Option<View>)>;

/// One used image, ready to be looked through.
struct Shot<'a> {
    /// Camera at 1/[`ColorConfig::downscale`] (depth buffer) and at full size (score).
    geometry: &'a Camera,
    full: &'a Camera,
    world_from_cam: Isometry3<f64>,
    rank: u32,
    sharpness_norm: f32,
    /// The image at 1/[`SHARPNESS_DOWNSCALE`].
    rgb: &'a RgbImage,
    /// [`SurfaceConfig::min_texel`].
    min_texel: f64,
}

/// The faces `shot` sees, in face order.
fn image_views(
    mesh: &Mesh,
    faces: &Faces,
    grid: &PointGrid,
    shot: &Shot,
    cfg: &ColorConfig,
    sampled: impl Fn(usize) -> bool + Sync,
    lin: &[f32; 256],
) -> Found {
    let world_to_cam = shot.world_from_cam.inverse();
    let cam_pos = shot.world_from_cam.translation.vector;
    let mut candidates = grid.near(&cam_pos, cfg.max_dist + faces.reach);
    candidates.par_sort_unstable();
    let cam = shot.geometry;
    let buffer = DepthBuffer::new(cam, cfg.zbuffer_scale);
    // Cells per radian, as the depth buffer sizes its splats.
    let focal = cam.fx.max(cam.fy) / f64::from(cfg.zbuffer_scale);
    candidates.par_iter().for_each(|&f| {
        let f = f as usize;
        let [a, b, c] = mesh.indices[f].map(|i| Vector3::from(mesh.positions[i as usize]));
        let longest = faces.longest[f];
        // No point of the face is nearer than this.
        let near = [a, b, c]
            .iter()
            .map(|p| (p.cast::<f64>() - cam_pos).norm())
            .fold(f64::INFINITY, f64::min)
            - f64::from(longest);
        let n = (f64::from(longest) * focal / (near.max(1e-3) * SAMPLE_GAP_CELLS)).ceil();
        let n = (n as usize).clamp(1, MAX_SUBDIVISIONS);
        let spacing = longest / n as f32;
        let splat = |p: Vector3<f32>| {
            if let Some(proj) = project_point(&[p.x, p.y, p.z], cam, &world_to_cam, cfg.max_dist) {
                buffer.splat(&proj, spacing);
            }
        };
        splat(Vector3::from(faces.centroid[f]));
        for i in 0..=n {
            for j in 0..=n - i {
                let (u, v) = (i as f32 / n as f32, j as f32 / n as f32);
                splat(a + (b - a) * u + (c - a) * v);
            }
        }
    });

    // Image coordinates at 1/`downscale` → at 1/SHARPNESS_DOWNSCALE (pixel centres at integers).
    let to_colour = f64::from(cfg.downscale) / f64::from(SHARPNESS_DOWNSCALE);
    candidates
        .par_iter()
        .filter_map(|&f| {
            let f = f as usize;
            let centre = faces.centroid[f];
            let proj = project_point(&centre, cam, &world_to_cam, cfg.max_dist)?;
            if f64::from(proj.theta) >= cfg.rim_theta {
                return None;
            }
            let to_cam = cam_pos - Vector3::from(centre).cast::<f64>();
            let cos = Vector3::from(faces.normal[f]).cast::<f64>().dot(&to_cam) / to_cam.norm();
            if cos.is_nan() || cos <= 0.0 || !buffer.is_visible(&proj, cfg.depth_tolerance) {
                return None;
            }
            let mut uv = [[0.0f64; 2]; 3];
            // The corners in the depth buffer's image (1/`downscale`).
            let mut at = [[0.0f32; 2]; 3];
            for ((uv, at), &i) in uv.iter_mut().zip(&mut at).zip(&mesh.indices[f]) {
                let p = Vector3::from(mesh.positions[i as usize]).cast::<f64>();
                let pc = world_to_cam * Point3::from(p);
                if pc.x.hypot(pc.y).atan2(pc.z) >= cfg.rim_theta {
                    return None;
                }
                *uv = shot.full.project(&pc.coords)?;
                // The corner must not be hidden either (same tolerance as the centroid).
                let corner = project_point(
                    &mesh.positions[i as usize],
                    cam,
                    &world_to_cam,
                    f64::INFINITY,
                )?;
                if !buffer.is_visible(&corner, cfg.depth_tolerance) {
                    return None;
                }
                *at = corner.uv;
            }
            let area = 0.5
                * ((uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1])
                    - (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]))
                    .abs();
            // Finer than `min_texel` is of no use (see the module docs).
            let [a, b, c] =
                mesh.indices[f].map(|i| Vector3::from(mesh.positions[i as usize]).cast::<f64>());
            let cap = 0.5 * (b - a).cross(&(c - a)).norm() / (shot.min_texel * shot.min_texel);
            // The score with the smoothed normal (see the module docs).
            let cos_s = Vector3::from(faces.smooth[f]).cast::<f64>().dot(&to_cam) / to_cam.norm();
            let cos_s = cos_s.max(SMOOTH_COS_MIN);
            let area = area * (cos_s / cos).clamp(0.5, 2.0);
            let score = (area.min(cap) * cos_s) as f32 * shot.sharpness_norm;
            if score.is_nan() || score <= 0.0 {
                return None;
            }
            let (below, across) = (to_cam.z, to_cam.x.hypot(to_cam.y));
            let carrier = faces.normal[f][2] > CARRIER_NORMAL_Z
                && below > CARRIER_BELOW
                && across < CARRIER_RADIUS
                && across < below;
            let to_px = |p: [f32; 2]| p.map(|x| ((f64::from(x) + 0.5) * to_colour - 0.5) as f32);
            let centre_rgb = sample_linear(shot.rgb, to_px(proj.uv), lin);
            // Mean colour: the centroid and the points halfway to the corners (in the image).
            let mut mean = centre_rgb;
            for corner in at {
                let half = [0, 1].map(|k| (proj.uv[k] + corner[k]) / 2.0);
                let rgb = sample_linear(shot.rgb, to_px(half), lin);
                for c in 0..3 {
                    mean[c] += rgb[c];
                }
            }
            let colour = mean.map(|v| srgb_value(v / 4.0).round().clamp(0.0, 255.0) as u8);
            // The carrier does not belong in the exposure fit either.
            let sample = (sampled(f) && !carrier)
                .then(|| View::new(score, shot.rank, centre_rgb, proj.theta));
            let angle = ((proj.theta / ANGLE_STEP).round() as u32).min(0x7f) as u8;
            let view = FaceView {
                image: shot.rank,
                score,
                colour,
                tag: angle | if carrier { TAG_CARRIER } else { 0 },
            };
            Some((f as u32, view, sample))
        })
        .collect()
}

/// Exposure-corrected colour of view `v` (sRGB levels).
fn corrected(
    v: &FaceView,
    images: &[UsedImage],
    gains: &[[f32; 3]],
    vignetting: &Vignetting,
    lin: &[f32; 256],
) -> [f64; 3] {
    let r = v.image as usize;
    let cam = images[r].camera;
    std::array::from_fn(|ch| {
        srgb_value(
            lin[v.colour[ch] as usize] / (gains[r][ch] * vignetting.factor(cam, ch, v.theta())),
        )
    })
}

/// The occlusion mask (see the module docs): learns per camera and image cell the share of
/// views that disagree with their face, and drops the views in cells above [`MASK_SHARE`] from
/// faces that have a decent other view (at least [`CARRIER_ALTERNATIVE`] of the best masked
/// one). `centroids` are the faces' centroids. Returns the masked cells per camera.
pub(crate) fn mask_views(
    per_face: &mut [FaceView],
    k: usize,
    images: &[UsedImage],
    gains: &[[f32; 3]],
    vignetting: &Vignetting,
    cams: &[ColorCamera],
    centroids: &[[f32; 3]],
) -> Vec<MaskResult> {
    let [gx, gy] = MASK_CELLS;
    let lin = linear_table();
    let world_to_cam: Vec<Isometry3<f64>> =
        images.iter().map(|i| i.world_from_cam.inverse()).collect();
    let per_cam = gx * gy;
    // The camera of a view and its cell in the camera's image (row · gx + column).
    let cell = |f: usize, v: &FaceView| -> Option<(usize, usize)> {
        let u = &images[v.image as usize];
        let cam = &cams.get(u.camera)?.camera;
        let pc = world_to_cam[v.image as usize]
            * Point3::from(Vector3::from(centroids[f]).cast::<f64>());
        let uv = cam.project(&pc.coords)?;
        let x = ((uv[0] / f64::from(cam.width)) * gx as f64).clamp(0.0, gx as f64 - 1.0) as usize;
        let y = ((uv[1] / f64::from(cam.height)) * gy as f64).clamp(0.0, gy as f64 - 1.0) as usize;
        Some((u.camera, y * gx + x))
    };
    let n_images = images.len();
    // Learn: per image and cell of its camera the judged views and those far from their
    // face's median, in a fixed number of face ranges (one counter table each, summed in
    // order).
    let faces = per_face.len() / k.max(1);
    let ranges: Vec<std::ops::Range<usize>> = (0..MASK_RANGES)
        .map(|r| faces * r / MASK_RANGES..faces * (r + 1) / MASK_RANGES)
        .collect();
    let tables: Vec<MaskCounts> = ranges
        .par_iter()
        .map(|range| {
            let mut acc = MaskCounts::new(n_images);
            for f in range.clone() {
                let slots = &per_face[f * k..(f + 1) * k];
                let n = slots.iter().take_while(|s| s.score > 0.0).count();
                if n < MASK_JUDGE_VIEWS {
                    continue;
                }
                let mut colours = [[0.0f64; 3]; MAX_VIEWS_PER_FACE];
                for (c, v) in colours.iter_mut().zip(&slots[..n]) {
                    *c = corrected(v, images, gains, vignetting, &lin);
                }
                let median = median_of(&colours[..n]);
                for (v, c) in slots[..n].iter().zip(&colours) {
                    if let Some((_, i)) = cell(f, v) {
                        let e = acc.at(v.image as usize, i);
                        e[0] += 1;
                        e[1] += u32::from(off(c, &median) > PHOTO_TOLERANCE);
                    }
                }
            }
            acc
        })
        .collect();
    let mut counts = MaskCounts::new(n_images);
    for t in &tables {
        counts.add(t);
    }
    drop(tables);
    // Per camera and cell: the images with a judged view there and those whose views there
    // mostly disagree (a frame counts once, however many faces it shows in the cell).
    let mut by_camera: Vec<Vec<usize>> = vec![Vec::new(); cams.len()];
    for (i, u) in images.iter().enumerate() {
        if let Some(list) = by_camera.get_mut(u.camera) {
            list.push(i);
        }
    }
    let stats: Vec<MaskCell> = (0..cams.len() * per_cam)
        .map(|c| {
            let (cam, cell) = (c / per_cam, c % per_cam);
            let mut s = MaskCell {
                views: 0,
                images: 0,
                disagree: 0,
            };
            for &i in &by_camera[cam] {
                let e = counts.image(i)[cell];
                s.views += u64::from(e[0]);
                s.images += usize::from(e[0] > 0);
                s.disagree += usize::from(e[0] > 0 && 2 * e[1] > e[0]);
            }
            s
        })
        .collect();
    drop(counts);
    let mut masked: Vec<bool> = stats.iter().map(MaskCell::masked).collect();
    // A camera with too many masked cells looks at something else than a carrier (many people,
    // monitors, a glossy floor): its mask is not used.
    let mut result = Vec::with_capacity(cams.len());
    for c in 0..cams.len() {
        let mine = &mut masked[c * gx * gy..(c + 1) * gx * gy];
        let count = mine.iter().filter(|&&m| m).count();
        let capped = count > MASK_MAX_CELLS;
        if capped {
            mine.fill(false);
        }
        result.push(MaskResult {
            masked: if capped { 0 } else { count },
            would_mask: count,
        });
    }
    per_face
        .par_chunks_mut(k)
        .enumerate()
        .for_each(|(f, slots)| {
            let n = slots.iter().take_while(|s| s.score > 0.0).count();
            let mut is_masked = [false; MAX_VIEWS_PER_FACE];
            for (m, v) in is_masked.iter_mut().zip(&slots[..n]) {
                *m = cell(f, v).is_some_and(|(cam, i)| masked[cam * per_cam + i]);
            }
            let best = |want: bool| {
                (0..n)
                    .filter(|&i| is_masked[i] == want)
                    .map(|i| slots[i].score)
                    .fold(0.0f32, f32::max)
            };
            let (bad, good) = (best(true), best(false));
            if bad > 0.0 && good >= CARRIER_ALTERNATIVE * bad {
                let mut kept = 0;
                for i in 0..n {
                    if !is_masked[i] {
                        slots[kept] = slots[i];
                        kept += 1;
                    }
                }
                for s in &mut slots[kept..] {
                    *s = FaceView::EMPTY;
                }
            }
        });
    result
}

/// What [`mask_views`] found for one camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MaskResult {
    /// Cells masked.
    pub masked: usize,
    /// Cells that met the rule; more than [`MASK_MAX_CELLS`] and the camera's mask is not used.
    pub would_mask: usize,
}

/// Per image and cell of its camera's image ([`MASK_CELLS`]): the judged views and those
/// far from their face's median.
struct MaskCounts {
    counts: Vec<[u32; 2]>,
}

impl MaskCounts {
    const CELLS: usize = MASK_CELLS[0] * MASK_CELLS[1];

    fn new(images: usize) -> MaskCounts {
        MaskCounts {
            counts: vec![[0; 2]; images * Self::CELLS],
        }
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.counts.len()
    }

    fn at(&mut self, image: usize, cell: usize) -> &mut [u32; 2] {
        &mut self.counts[image * Self::CELLS + cell]
    }

    /// The cells of one image.
    fn image(&self, image: usize) -> &[[u32; 2]] {
        &self.counts[image * Self::CELLS..(image + 1) * Self::CELLS]
    }

    fn add(&mut self, other: &MaskCounts) {
        for (x, y) in self.counts.iter_mut().zip(&other.counts) {
            x[0] += y[0];
            x[1] += y[1];
        }
    }
}

/// The statistics of one cell of a camera's image.
struct MaskCell {
    views: u64,
    images: usize,
    disagree: usize,
}

impl MaskCell {
    fn masked(&self) -> bool {
        self.views >= MASK_MIN_VIEWS
            && self.images >= MASK_MIN_IMAGES
            && self.disagree as f64 > MASK_SHARE * self.images as f64
    }
}

/// Per-channel median of `colours` (the mean of the middle two for an even count).
fn median_of(colours: &[[f64; 3]]) -> [f64; 3] {
    let n = colours.len();
    std::array::from_fn(|ch| {
        let mut x = [0.0f64; MAX_VIEWS_PER_FACE];
        for (x, c) in x.iter_mut().zip(colours) {
            *x = c[ch];
        }
        x[..n].sort_by(f64::total_cmp);
        (x[(n - 1) / 2] + x[n / 2]) / 2.0
    })
}

/// Largest channel difference of `c` from `m` (sRGB levels).
fn off(c: &[f64; 3], m: &[f64; 3]) -> f64 {
    (0..3).map(|ch| (c[ch] - m[ch]).abs()).fold(0.0, f64::max)
}

/// Drops the views of a face that may show the carrier if it has a decent other one, then the
/// views that do not fit the others' colours (see the module docs). `per_face` holds `k`
/// slots per face (at most [`MAX_VIEWS_PER_FACE`]), as [`keep_best_view`] fills them;
/// `images`, `gains` and `vignetting` are the used images and their exposure. The kept views
/// are ordered by score (then image).
pub(crate) fn filter_views(
    per_face: &mut [FaceView],
    k: usize,
    images: &[UsedImage],
    gains: &[[f32; 3]],
    vignetting: &Vignetting,
) {
    assert!(k <= MAX_VIEWS_PER_FACE, "too many views per face");
    let lin = linear_table();
    per_face.par_chunks_mut(k).for_each(|slots| {
        let best = |carrier: bool| {
            slots
                .iter()
                .filter(|s| s.score > 0.0 && s.carrier() == carrier)
                .map(|s| s.score)
                .fold(0.0f32, f32::max)
        };
        let (tagged, untagged) = (best(true), best(false));
        if tagged > 0.0 && untagged >= CARRIER_ALTERNATIVE * tagged {
            for s in slots.iter_mut().filter(|s| s.carrier()) {
                *s = FaceView::EMPTY;
            }
        }
        // By score, empty slots last.
        slots.sort_by(|a, b| {
            (b.score > 0.0)
                .cmp(&(a.score > 0.0))
                .then(b.score.total_cmp(&a.score))
                .then(a.image.cmp(&b.image))
        });
        let n = slots.iter().take_while(|s| s.score > 0.0).count();
        if n < CONSISTENCY_MIN_VIEWS {
            return;
        }
        // Exposure-corrected colours (sRGB levels).
        let mut colours = [[0.0f64; 3]; MAX_VIEWS_PER_FACE];
        for (c, v) in colours.iter_mut().zip(&slots[..n]) {
            let r = v.image as usize;
            let cam = images[r].camera;
            *c = std::array::from_fn(|ch| {
                srgb_value(
                    lin[v.colour[ch] as usize]
                        / (gains[r][ch] * vignetting.factor(cam, ch, v.theta())),
                )
            });
        }
        // Votes: the first (best) view per camera and stretch of VOTE_GAP.
        let stretch = |i: usize| {
            let u = &images[slots[i].image as usize];
            (u.camera, (u.stamp / VOTE_GAP).floor() as i64)
        };
        let mut voter = [false; MAX_VIEWS_PER_FACE];
        for (i, v) in voter[..n].iter_mut().enumerate() {
            *v = (0..i).all(|j| stretch(j) != stretch(i));
        }
        let votes = voter[..n].iter().filter(|&&v| v).count();
        if votes < CONSISTENCY_MIN_VIEWS {
            return;
        }
        let median: [f64; 3] = std::array::from_fn(|ch| {
            let mut x = [0.0f64; MAX_VIEWS_PER_FACE];
            let mut m = 0;
            for i in (0..n).filter(|&i| voter[i]) {
                x[m] = colours[i][ch];
                m += 1;
            }
            x[..m].sort_by(f64::total_cmp);
            (x[(m - 1) / 2] + x[m / 2]) / 2.0
        });
        let near = |i: usize| {
            (0..3)
                .map(|ch| (colours[i][ch] - median[ch]).abs())
                .fold(0.0, f64::max)
                <= PHOTO_TOLERANCE
        };
        let agree = (0..n).filter(|&i| voter[i] && near(i)).count();
        // Only a strict majority of the votes decides; otherwise all views stay.
        if 2 * agree <= votes {
            return;
        }
        let mut kept = 0;
        for i in 0..n {
            if near(i) {
                slots[kept] = slots[i];
                kept += 1;
            }
        }
        for s in &mut slots[kept..] {
            *s = FaceView::EMPTY;
        }
    });
}

/// The views of every face of `mesh` in the images of `src`, and the exposure of the images
/// (see the module docs). Progress: phase „Bilder prüfen“ (the quality pass) and „Ansichten
/// suchen“, one step per image of the stream; a cancel is checked at every image and returns
/// [`MapError::Cancelled`]. Same input, same result.
///
/// Fails if `cfg` is invalid, two cameras share a name or the images cannot be read.
#[cfg(test)]
pub(crate) fn collect_views(
    mesh: &Mesh,
    src: &TextureSource,
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<ViewTable, MapError> {
    collect_views_with(mesh, src, cfg, progress, None)
}

/// [`collect_views`], with `gpu` the pass over the images on the GPU (see
/// [`super::views_gpu`]): the same views up to rounding.
#[cfg(test)]
pub(crate) fn collect_views_with(
    mesh: &Mesh,
    src: &TextureSource,
    cfg: &SurfaceConfig,
    progress: &Progress,
    gpu: Option<&GpuContext>,
) -> Result<ViewTable, MapError> {
    collect_views_on(mesh, src, cfg, progress, gpu.map(|g| (g, None))).map(|(t, _)| t)
}

/// What [`collect_views_on`] reports besides the views.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct ViewsInfo {
    /// The GPU was given but could not start (`--gpu auto`): the CPU looked, for this reason
    /// (German).
    pub gpu_fallback: Option<String>,
    /// Seconds of the quality pass and of the pass over the images (wall clock).
    pub quality_seconds: f64,
    pub views_seconds: f64,
    /// Of `views_seconds`, how long the GPU waited for decoded images.
    pub decode_wait_seconds: f64,
}

/// [`collect_views_with`] with the GPU's batch size (faces per batch; `None`: as large as the
/// device allows), and what the run reports.
pub(crate) fn collect_views_on(
    mesh: &Mesh,
    src: &TextureSource,
    cfg: &SurfaceConfig,
    progress: &Progress,
    gpu: Option<(&GpuContext, Option<usize>)>,
) -> Result<(ViewTable, ViewsInfo), MapError> {
    if progress.is_cancelled() {
        return Err(MapError::Cancelled);
    }
    cfg.check()?;
    let (cams, poses, ccfg) = (&src.cams, &src.poses, &cfg.color);
    for (i, c) in cams.iter().enumerate() {
        if cams[..i].iter().any(|o| o.camera.name == c.camera.name) {
            return Err(MapError::Config(format!(
                "Kamera {} ist doppelt angegeben",
                c.camera.name
            )));
        }
    }
    let mut info = ViewsInfo::default();
    let clock = std::time::Instant::now();
    let (records, _, _) = quality_pass(&src.images, poses, cams, ccfg, progress)?;
    info.quality_seconds = clock.elapsed().as_secs_f64();
    let clock = std::time::Instant::now();

    // The used images by rank, with their world poses.
    let mut images: Vec<Option<UsedImage>> = Vec::new();
    for r in &records {
        if let Status::Used { rank, .. } = r.status {
            let rank = rank as usize;
            if images.len() <= rank {
                images.resize(rank + 1, None);
            }
            let body = interpolate_pose(poses, r.stamp + ccfg.time_offset)
                .expect("checked in the quality pass");
            images[rank] = Some(UsedImage {
                camera: r.camera,
                stamp: r.stamp,
                world_from_cam: body * cams[r.camera].cam_in_body,
            });
        }
    }
    let images: Vec<UsedImage> = images
        .into_iter()
        .map(|i| i.expect("ranks are dense"))
        .collect();

    let k = cfg.views_per_face;
    let face_count = mesh.indices.len();
    // Exposure samples: every `stride`-th face, `slot = face / stride`; at most as many as the
    // fit takes, so that it takes every one with 2 views (see the module docs).
    let most = EXPOSURE_FACES.min(ccfg.exposure.samples).max(1);
    let stride = face_count.div_ceil(most).max(1);
    let sample_faces = if ccfg.exposure.samples > 0 {
        face_count.div_ceil(stride)
    } else {
        0
    };
    let empty = View::new(0.0, u32::MAX, [0.0; 3], 0.0);

    let last_used = records
        .iter()
        .rposition(|r| matches!(r.status, Status::Used { .. }));
    let walked = match (last_used, face_count) {
        (Some(last_used), 1..) => last_used as u64 + 1,
        _ => 0,
    };
    progress.phase("Ansichten suchen", Some(walked));
    let (mut per_face, samples) = if let (Some(last_used), 1..) = (last_used, face_count) {
        let faces = Faces::new(mesh);
        let pass = Pass {
            mesh,
            faces: &faces,
            src,
            cfg,
            records: &records,
            images: &images,
            last_used,
            stride,
            sample_faces,
        };
        let on_gpu = match gpu {
            None => None,
            Some((ctx, batch)) => match super::views_gpu::gpu_views(&pass, ctx, batch, progress) {
                Ok((views, wait)) => {
                    info.decode_wait_seconds = wait;
                    Some(views)
                }
                Err(GpuStop::Fail(e)) => return Err(e),
                Err(GpuStop::Fallback(why)) => {
                    progress.info(format!("Ansichten auf der CPU: {why}"));
                    info.gpu_fallback = Some(why);
                    None
                }
            },
        };
        match on_gpu {
            Some(views) => views,
            None => cpu_views(&pass, progress)?,
        }
    } else {
        (
            vec![FaceView::EMPTY; face_count * k],
            vec![empty; sample_faces * k],
        )
    };
    info.views_seconds = clock.elapsed().as_secs_f64();

    // Exposure and white balance, fitted on the sample faces' kept views (as `colorize`).
    let cameras: Vec<usize> = images.iter().map(|i| i.camera).collect();
    let (gains, vignetting) = if ccfg.exposure.samples == 0 {
        (
            vec![[1.0; 3]; images.len()],
            Vignetting::flat(cams.len(), ccfg.exposure.max_gain),
        )
    } else {
        let obs = sample_observations(&samples, k, cameras.clone(), ccfg.exposure.samples);
        drop(samples);
        let (gains, mut vignetting) = estimate_exposure(&obs, &ccfg.exposure);
        vignetting.coeffs.resize(cams.len(), [[0.0; 2]; 3]);
        (gains, vignetting)
    };
    let centroids: Vec<[f32; 3]> = (0..face_count)
        .into_par_iter()
        .map(|f| {
            let c = mesh.indices[f].map(|i| mesh.positions[i as usize]);
            [0, 1, 2].map(|a| (c[0][a] + c[1][a] + c[2][a]) / 3.0)
        })
        .collect();
    let masked = mask_views(
        &mut per_face,
        k,
        &images,
        &gains,
        &vignetting,
        cams,
        &centroids,
    );
    drop(centroids);
    let all = MASK_CELLS[0] * MASK_CELLS[1];
    for (c, m) in cams.iter().zip(&masked) {
        if m.masked > 0 {
            progress.info(format!(
                "Kamera {}: {} von {all} Bildbereichen zeigen oft etwas, das nicht zur Fläche passt (Träger, Personen)",
                c.camera.name, m.masked
            ));
        } else if m.would_mask > 0 {
            progress.warn(format!(
                "Kamera {}: {} von {all} Bildbereichen passen oft nicht zur Fläche, zu viele für einen Träger; diese Kamera wird nicht maskiert",
                c.camera.name, m.would_mask
            ));
        }
    }
    filter_views(&mut per_face, k, &images, &gains, &vignetting);
    Ok((
        ViewTable {
            images,
            per_face,
            gains,
            vignetting,
        },
        info,
    ))
}

/// The pass over the images: what [`cpu_views`] and the GPU need.
pub(crate) struct Pass<'a> {
    pub mesh: &'a Mesh,
    pub faces: &'a Faces,
    pub src: &'a TextureSource,
    pub cfg: &'a SurfaceConfig,
    pub records: &'a [ImageRecord],
    /// The used images by rank.
    pub images: &'a [UsedImage],
    /// Stream index of the last used image.
    pub last_used: usize,
    /// Every `stride`-th face keeps exposure samples (`sample_faces` of them; none if 0).
    pub stride: usize,
    pub sample_faces: usize,
}

/// The views per face ([`SurfaceConfig::views_per_face`] slots each, as [`keep_best_view`]
/// fills them) and the exposure samples, on the CPU.
fn cpu_views(pass: &Pass, progress: &Progress) -> Result<(Vec<FaceView>, Vec<View>), MapError> {
    let Pass {
        mesh,
        faces,
        src,
        cfg,
        records,
        images,
        last_used,
        stride,
        sample_faces,
    } = *pass;
    let (cams, ccfg) = (&src.cams, &cfg.color);
    let k = cfg.views_per_face;
    let face_count = mesh.indices.len();
    let mut per_face = vec![FaceView::EMPTY; face_count * k];
    let empty = View::new(0.0, u32::MAX, [0.0; 3], 0.0);
    let mut samples = vec![empty; sample_faces * k];
    {
        let grid = PointGrid::new(&faces.centroid);
        let lin = linear_table();
        let geometry: Vec<Camera> = cams
            .iter()
            .map(|c| c.camera.downscaled(ccfg.downscale))
            .collect();
        let sampled = |f: usize| sample_faces > 0 && f.is_multiple_of(stride);
        let mut index = 0usize;
        let mut failure: Option<MapError> = None;
        let mut one = |frame: Result<Frame, ImageError>| {
            let i = index;
            index += 1;
            let Some(&ImageRecord {
                camera,
                stamp,
                status:
                    Status::Used {
                        rank,
                        sharpness_norm,
                    },
            }) = records.get(i)
            else {
                return i < last_used;
            };
            let frame = match frame {
                Ok(f)
                    if f.stamp == stamp
                        && frame_camera(&f, cams, SHARPNESS_DOWNSCALE) == Some(camera) =>
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
            let shot = Shot {
                geometry: &geometry[camera],
                full: &cams[camera].camera,
                world_from_cam: images[rank as usize].world_from_cam,
                rank,
                sharpness_norm,
                rgb: &frame.rgb,
                min_texel: cfg.min_texel,
            };
            for (f, view, colour) in image_views(mesh, faces, &grid, &shot, ccfg, sampled, &lin) {
                let f = f as usize;
                keep_best_view(&mut per_face[f * k..(f + 1) * k], view);
                if let Some(v) = colour {
                    let s = f / stride;
                    keep_sample(&mut samples[s * k..(s + 1) * k], s, v);
                }
            }
            i < last_used
        };
        let mut cancelled = false;
        let read = for_each_image(
            &src.images.bags,
            &src.images.topics,
            SHARPNESS_DOWNSCALE,
            |frame| {
                if progress.is_cancelled() {
                    cancelled = true;
                    return false;
                }
                let go = one(frame);
                progress.advance(1);
                go
            },
        );
        if cancelled {
            return Err(MapError::Cancelled);
        }
        read.map_err(|e| MapError::Images(e.to_string()))?;
        if let Some(e) = failure {
            return Err(e);
        }
    }
    Ok((per_face, samples))
}

#[cfg(test)]
mod tests {
    use super::super::testutil::scene::{
        aabb, cameras, poses, render, shot, surface_cloud, walk, walk_data, walk_scene, write_bag,
        Painted, Shot, Trajectory, T0,
    };
    use super::super::testutil::{blur, centroid, face_near, painted_mesh, Steps};
    use super::*;
    use crate::{colorize, ColorConfig, ExposureConfig};
    use driftless_gpu::GpuContext;
    use nalgebra::{UnitQuaternion, Vector3};

    /// Square size of the test meshes (m).
    const STEP: f64 = 0.1;

    /// The exposure gains from sample slots filled as `cpu_views` fills them: 3,000 points,
    /// each seen by 16 consecutive of 30 images (two cameras) with nearly equal scores, so that
    /// the 8 best by score depend on rounding. Colours: albedo × the image's gain × ±5 % noise.
    fn sampled_gains(perturb: f32) -> Vec<[f32; 3]> {
        let (images, points, seen, k) = (30u32, 3000usize, 16u32, 8usize);
        let mix = |a: u64| {
            let h = a.wrapping_mul(0x9E37_79B9_7F4A_7C15);
            ((h ^ (h >> 29)) >> 11) as f32 / (1u64 << 53) as f32
        };
        let empty = View::new(0.0, u32::MAX, [0.0; 3], 0.0);
        let mut slots = vec![empty; points * k];
        for rank in 0..images {
            let gain = 0.8 + 0.45 * mix(u64::from(rank) + 7);
            for j in 0..points {
                let first = (j as u32 * 7) % (images - seen);
                if !(first..first + seen).contains(&rank) {
                    continue;
                }
                let at = (j as u64) << 8 | u64::from(rank);
                let albedo = 0.1 + 0.5 * mix(j as u64 * 3 + 1);
                let rgb = [0.0, 1.0, 2.0].map(|c| {
                    albedo * gain * (0.95 + 0.1 * mix(at * 4 + c as u64)) * (1.0 - 0.05 * c)
                });
                // Near-equal scores: the order among them is set by the last bits.
                let score = 100.0 * (1.0 + 1e-6 * mix(at * 4 + 3));
                let sign = if mix(at * 5 + 11) < 0.5 { -1.0 } else { 1.0 };
                let score = score * (1.0 + sign * perturb);
                let theta = 0.1 + 0.03 * rank as f32;
                let v = View::new(score, rank, rgb, theta);
                keep_sample(&mut slots[j * k..(j + 1) * k], j, v);
            }
        }
        let cameras = (0..images as usize).map(|i| i % 2).collect();
        let obs = sample_observations(&slots, k, cameras, ExposureConfig::default().samples);
        estimate_exposure(&obs, &ExposureConfig::default()).0
    }

    #[test]
    fn exposure_sample_prefers_higher_half_octaves() {
        // Half octaves: [64, 90.5), [90.5, 128), [128, 181) …
        let key = |rank, score| sample_key(3, rank, score);
        assert!(key(0, 91.0) > key(1, 90.0) && key(1, 91.0) > key(0, 90.0));
        assert!(key(0, 130.0) > key(1, 127.0) && key(1, 130.0) > key(0, 127.0));
        // Within one, the hash decides: one order for the pair, whatever the scores.
        let (a, b) = (key(0, 100.0), key(1, 100.0));
        assert_ne!(a, b);
        assert_eq!(key(0, 95.0) > key(1, 120.0), a > b);
        assert_eq!(key(0, 0.0), 0.0);
        assert!(key(0, 1e-30) > 0.0 && key(0, 1e30).is_finite());
    }

    #[test]
    fn exposure_sample_ignores_near_tie_scores() {
        let plain = sampled_gains(0.0);
        assert!(
            plain.iter().any(|g| (g[0] - 1.0).abs() > 0.02),
            "the fit sees the gains: {plain:?}"
        );
        // Scores off by ±1e-4 (relative): the same samples, the same gains.
        assert_eq!(sampled_gains(1e-4), plain);
        assert_eq!(sampled_gains(-1e-4), plain);
    }

    #[test]
    fn mask_counts_hold_one_row_of_cells_per_image() {
        let cells = MASK_CELLS[0] * MASK_CELLS[1];
        // Seven images: the table does not grow with the number of cameras.
        let mut counts = MaskCounts::new(7);
        assert_eq!(counts.len(), 7 * cells);
        counts.at(6, cells - 1)[0] += 3;
        counts.at(0, 0)[1] += 1;
        assert_eq!(counts.image(6)[cells - 1], [3, 0]);
        assert_eq!(counts.image(0)[0], [0, 1]);
        let mut other = MaskCounts::new(7);
        other.at(6, cells - 1)[0] += 2;
        counts.add(&other);
        assert_eq!(counts.image(6)[cells - 1], [5, 0]);
    }

    fn source(images: &ImageSource, poses: &[Pose], cams: &[ColorCamera]) -> TextureSource {
        TextureSource {
            images: images.clone(),
            poses: poses.to_vec(),
            cams: cams.to_vec(),
        }
    }

    fn views(table: &ViewTable, k: usize, f: usize) -> &[FaceView] {
        &table.per_face[f * k..(f + 1) * k]
    }

    /// Body pose that puts camera `cam` at `world_cam`.
    fn body_for(cam: &ColorCamera, world_cam: Isometry3<f64>) -> Isometry3<f64> {
        world_cam * cam.cam_in_body.inverse()
    }

    #[test]
    fn front_view_beats_grazing() {
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        // A face on the +y wall, seen by the left camera head-on from 1.5 m and at 80° from
        // its normal from 1.5 m.
        let target = Vector3::new(1.0, 5.0, 0.3);
        let f = face_near(&mesh, [1.0, 5.0, 0.3]);
        let rot = cams[0].cam_in_body.rotation;
        let front = Isometry3::from_parts((target - Vector3::new(0.0, 1.5, 0.0)).into(), rot);
        let a = 80f64.to_radians();
        let graze = Isometry3::from_parts(
            (target + 1.5 * Vector3::new(a.sin(), -a.cos(), 0.0)).into(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, a) * rot,
        );
        let traj = Steps(vec![
            (0.0, body_for(&cams[0], front)),
            (T0 + 1.5, body_for(&cams[0], graze)),
        ]);
        let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
        let images = write_bag("views-graze", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let v = views(&table, cfg.views_per_face, f);
        println!("face {f} at {:?}: {v:?}", centroid(&mesh, f));
        // Rank 0 is the frontal image (the earlier one).
        assert_eq!(v[0].image, 0, "{v:?}");
        assert!(v[0].score > 0.0, "{v:?}");
        if let Some(g) = v.iter().find(|x| x.image == 1) {
            assert!(g.score < 0.2 * v[0].score, "{v:?}");
        }
        // At 80° the wall's own nearer stretch in the same depth cell hides the face; with a
        // loose depth tolerance the grazing view comes through and scores far lower
        // (area × cos² 80° ≈ 3 % of the frontal one).
        let loose = SurfaceConfig {
            color: ColorConfig {
                depth_tolerance: 0.5,
                ..cfg.color.clone()
            },
            ..cfg.clone()
        };
        let table = collect_views(&mesh, &src, &loose, &Progress::new()).unwrap();
        let v = views(&table, cfg.views_per_face, f);
        println!("loose: {v:?}");
        assert_eq!(v[0].image, 0, "{v:?}");
        let g = v.iter().find(|x| x.image == 1).expect("grazing view");
        assert!(g.score > 0.0 && g.score < 0.1 * v[0].score, "{v:?}");
    }

    #[test]
    fn occluded_view_dropped() {
        // A pillar between the left camera at the origin and the +y wall; from 2 m to the
        // side the same stretch of wall is in plain view.
        let painted = Painted::new(vec![(
            aabb([-0.2, 2.0, -1.2], [0.2, 2.4, 1.8]),
            [240, 140, 20],
        )]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let traj = Steps(vec![
            (0.0, Isometry3::identity()),
            (T0 + 1.5, Isometry3::translation(2.0, 0.0, 0.0)),
        ]);
        let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
        let images = write_bag("views-occluded", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let k = cfg.views_per_face;
        // The wall behind the pillar, seen from the origin: x within ±0.49 m at every height.
        let hidden: Vec<usize> = (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(&mesh, f);
                c[1] > 4.99 && c[0].abs() < 0.25 && (c[2] - 0.05).abs() < 0.6
            })
            .collect();
        assert!(hidden.len() > 20, "{}", hidden.len());
        for &f in &hidden {
            let v = views(&table, k, f);
            assert!(v.iter().all(|x| x.image != 0), "face {f}: {v:?}");
            assert!(v.iter().any(|x| x.image == 1), "face {f}: {v:?}");
        }
        // The pillar's front is seen from the origin.
        let pillar = face_near(&mesh, [0.0, 2.0, 0.05]);
        assert!(views(&table, k, pillar).iter().any(|x| x.image == 0));
    }

    /// Adds triangle `t` to `mesh`, wound so that its normal points along `normal`; returns
    /// its face index.
    fn add_triangle(mesh: &mut Mesh, t: [[f32; 3]; 3], normal: [f32; 3]) -> usize {
        let base = mesh.positions.len() as u32;
        mesh.positions.extend_from_slice(&t);
        let f = mesh.indices.len();
        mesh.indices.push([base, base + 1, base + 2]);
        let n = mesh.face_normal(f);
        if (0..3).map(|k| n[k] * normal[k]).sum::<f32>() < 0.0 {
            mesh.indices[f] = [base, base + 2, base + 1];
        }
        f
    }

    #[test]
    fn partly_hidden_face_rejected() {
        partly_hidden_case(None);
    }

    #[test]
    fn partly_hidden_face_rejected_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        partly_hidden_case(Some(&ctx));
    }

    fn partly_hidden_case(gpu: Option<&GpuContext>) {
        // A pillar between the left camera at the origin and the +y wall hides the wall for
        // x within about −0.29 … 0.22 m; the depth buffer widens that by its splats and the
        // silhouette rule to about ±0.5 m. One extra wall face has a corner deep in the shadow
        // and its centroid in view (x = 0.7); a second one lies wholly in view.
        let painted = Painted::new(vec![(
            aabb([-0.1, 2.0, -1.2], [0.1, 2.2, 1.8]),
            [240, 140, 20],
        )]);
        let cams = cameras();
        let mut mesh = painted_mesh(&painted, STEP);
        let wall = [0.0, -1.0, 0.0];
        let partly = add_triangle(
            &mut mesh,
            [[0.1, 5.0, 0.0], [1.0, 5.0, 0.0], [1.0, 5.0, 0.4]],
            wall,
        );
        let clear = add_triangle(
            &mut mesh,
            [[0.7, 5.0, 0.0], [1.3, 5.0, 0.0], [1.3, 5.0, 0.4]],
            wall,
        );
        let traj = Steps(vec![(0.0, Isometry3::identity())]);
        let images = write_bag("views-partly", &painted, &traj, &cams, &[shot(0, T0 + 1.0)]);
        let src = source(&images, &poses(&traj, 2.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views_with(&mesh, &src, &cfg, &Progress::new(), gpu).unwrap();
        assert_eq!(table.images.len(), 1);
        let k = cfg.views_per_face;
        // The centroid alone is in view: the grid face around it is seen …
        let around = face_near(&painted_mesh(&painted, STEP), centroid(&mesh, partly));
        assert!(views(&table, k, around).iter().any(|v| v.image == 0));
        assert!(views(&table, k, clear).iter().any(|v| v.image == 0));
        // … but not the face with the hidden corner.
        let v = views(&table, k, partly);
        assert!(v.iter().all(|x| x.image != 0), "{v:?}");
    }

    #[test]
    fn simplified_wall_gets_views() {
        simplified_wall_case(None);
    }

    #[test]
    fn simplified_wall_gets_views_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        simplified_wall_case(Some(&ctx));
    }

    fn simplified_wall_case(gpu: Option<&GpuContext>) {
        // A 10 × 2 m stretch of the +y wall as 2 triangles, and the left camera walking along
        // it 1.5 m away. Only views within 50° of the optical axis count (a stand-in for a wall
        // too long for one image): the big triangles fit into no image, their split pieces do.
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mut wall = Mesh::default();
        let n = [0.0, -1.0, 0.0];
        add_triangle(
            &mut wall,
            [[-4.0, 5.0, -0.8], [6.0, 5.0, -0.8], [6.0, 5.0, 1.2]],
            n,
        );
        add_triangle(
            &mut wall,
            [[-4.0, 5.0, -0.8], [6.0, 5.0, 1.2], [-4.0, 5.0, 1.2]],
            n,
        );
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let xs: Vec<f64> = (0..8).map(|i| -3.5 + 1.4 * f64::from(i)).collect();
        let traj = Steps(
            xs.iter()
                .enumerate()
                .map(|(i, &x)| {
                    let from = if i == 0 { 0.0 } else { T0 + 0.5 + i as f64 };
                    (from, Isometry3::translation(x, 3.5 - cam_y, 0.0))
                })
                .collect(),
        );
        let shots: Vec<Shot> = (0..xs.len())
            .map(|i| shot(0, T0 + 1.0 + i as f64))
            .collect();
        let images = write_bag("views-split", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, xs.len() as f64 + 1.0), &cams);
        let mut cfg = SurfaceConfig::default();
        cfg.color.rim_theta = 50f64.to_radians();
        let seen = |mesh: &Mesh| {
            let table = collect_views_with(mesh, &src, &cfg, &Progress::new(), gpu).unwrap();
            assert_eq!(table.images.len(), xs.len());
            (0..mesh.indices.len())
                .filter(|&f| views(&table, cfg.views_per_face, f)[0].score > 0.0)
                .count()
        };
        assert_eq!(seen(&wall), 0);
        let split = super::super::simplify(&wall, &cfg);
        assert!(split.indices.len() > 100, "{}", split.indices.len());
        assert_eq!(seen(&split), split.indices.len());
    }

    /// The floor faces under "shoes" (a box the mesh does not have) 0.6–1.0 m in front of the
    /// left camera at the origin, 0.75 m above the floor.
    fn under_shoes(mesh: &Mesh) -> Vec<usize> {
        (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(mesh, f);
                c[2] < -1.19 && c[0].abs() < 0.15 && c[1] > 0.75 && c[1] < 1.05
            })
            .collect()
    }

    #[test]
    fn carrier_views_left_out() {
        // The carrier's shoes stand on the floor 0.6–1.0 m in front of the camera, which
        // looks down at them from 1.2 m (near and steep: the best view by far). A second image
        // 1.5–1.9 m away sees the same floor without them (the carrier has walked on).
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let shoes = aabb([-0.25, 0.65, -1.2], [0.25, 1.15, -1.17]);
        let near = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(-0.5, 0.0, 0.0),
        );
        let far = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, -0.8, 0.5),
            UnitQuaternion::from_euler_angles(-0.6, 0.0, 0.0),
        );
        let traj = Steps(vec![(0.0, near), (T0 + 1.5, far)]);
        let cam = cams[0].clone();
        let with_shoes = move |_| {
            let scene = Painted::new(vec![(shoes, [150, 90, 40])]);
            render(&scene, &cam.camera, &(near * cam.cam_in_body))
        };
        let shots = [
            Shot {
                camera: 0,
                stamp: T0 + 1.0,
                edit: Box::new(with_shoes.clone()),
            },
            shot(0, T0 + 2.0),
        ];
        let images = write_bag("views-carrier", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let floor = under_shoes(&mesh);
        assert!(floor.len() > 10, "{}", floor.len());
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let labels = crate::surface::atlas::choose_labels(&mesh, &table, &cfg);
        for &f in &floor {
            let v = views(&table, k, f);
            assert!(v.iter().all(|x| x.image != 0), "face {f}: {v:?}");
            assert!(v.iter().any(|x| x.image == 1), "face {f}: {v:?}");
            assert_eq!(labels[f], Some(1), "face {f}");
        }
        // The wall ahead is far from the carrier: both images keep it.
        let wall = face_near(&mesh, [0.0, 5.0, -0.5]);
        let v = views(&table, k, wall);
        assert!(
            v.iter().any(|x| x.image == 0) && v.iter().any(|x| x.image == 1),
            "{v:?}"
        );
        // Seen from near only, the floor keeps that view: better than none.
        let only = write_bag(
            "views-carrier-only",
            &painted,
            &traj,
            &cams,
            &[Shot {
                camera: 0,
                stamp: T0 + 1.0,
                edit: Box::new(with_shoes),
            }],
        );
        let src = source(&only, &poses(&traj, 3.0), &cams);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        for &f in &floor {
            assert_eq!(views(&table, k, f)[0].image, 0, "face {f}");
        }
    }

    #[test]
    fn inconsistent_view_left_out() {
        // Three images of the +y wall from 1.5 m: head-on at x = 0 (the best view) and from
        // x = ±1 m. In the head-on one somebody stands 0.5 m in front of the wall (a box the
        // mesh does not have): there the wall has another colour than in the other two.
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let body = move |x: f64| Isometry3::translation(x, 3.5 - cam_y, 0.0);
        let traj = Steps(vec![
            (0.0, body(0.0)),
            (T0 + 1.5, body(-1.0)),
            (T0 + 2.5, body(1.0)),
        ]);
        let cam = cams[0].clone();
        let person = aabb([-0.3, 4.4, -0.4], [0.3, 4.6, 0.5]);
        let shots = [
            Shot {
                camera: 0,
                stamp: T0 + 1.0,
                edit: Box::new(move |_| {
                    let scene = Painted::new(vec![(person, [40, 50, 90])]);
                    render(&scene, &cam.camera, &(body(0.0) * cam.cam_in_body))
                }),
            },
            shot(0, T0 + 2.0),
            shot(0, T0 + 3.0),
        ];
        let images = write_bag("views-person", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 4.0), &cams);
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 3, "{:?}", table.images);
        let labels = crate::surface::atlas::choose_labels(&mesh, &table, &cfg);
        // The wall behind the person as the head-on camera sees it (its shadow there is 1.5
        // times the box: x within ±0.45 m, z from −0.55 to 0.8 m).
        let behind: Vec<usize> = (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(&mesh, f);
                c[1] > 4.99 && c[0].abs() < 0.35 && c[2] > -0.45 && c[2] < 0.65
            })
            .collect();
        assert!(behind.len() > 20, "{}", behind.len());
        for &f in &behind {
            let v = views(&table, k, f);
            assert!(v.iter().all(|x| x.image != 0), "face {f}: {v:?}");
            assert!(
                v.iter().filter(|x| x.score > 0.0).count() >= 2,
                "face {f}: {v:?}"
            );
            assert_ne!(labels[f], Some(0), "face {f}");
        }
        // Above the person all three agree: the head-on view stays the best.
        let clear = face_near(&mesh, [0.0, 5.0, 1.2]);
        let v = views(&table, k, clear);
        assert_eq!(v[0].image, 0, "{v:?}");
        assert_eq!(v.iter().filter(|x| x.score > 0.0).count(), 3, "{v:?}");
    }

    #[test]
    fn low_wall_near_path_keeps_near_view() {
        // A wall face 0.3 m above the floor, 0.8 m in front of a camera looking down at it
        // (0.9 m below): near the path, but no floor. A second image sees it from 3 m.
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let near = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 4.2 - cam_y, 0.0),
            UnitQuaternion::from_euler_angles(-0.6, 0.0, 0.0),
        );
        let far = Isometry3::translation(0.0, 2.0 - cam_y, 0.0);
        let traj = Steps(vec![(0.0, near), (T0 + 1.5, far)]);
        let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
        let images = write_bag("views-lowwall", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let f = face_near(&mesh, [0.0, 5.0, -0.9]);
        let v = views(&table, cfg.views_per_face, f);
        assert!(v.iter().any(|x| x.image == 0), "{v:?}");
        assert!(v.iter().any(|x| x.image == 1), "{v:?}");
        assert!(v.iter().all(|x| !x.carrier()), "{v:?}");
    }

    #[test]
    fn carrier_drop_needs_decent_alternative() {
        // A floor face's near view (tagged) and a poor grazing one: the near view stays; with
        // a decent alternative it goes.
        let view = |image: u32, score: f32, tag: u8| FaceView {
            image,
            score,
            colour: [80; 3],
            tag,
        };
        let images: Vec<UsedImage> = (0..3)
            .map(|i| UsedImage {
                camera: 0,
                stamp: f64::from(i),
                world_from_cam: Isometry3::identity(),
            })
            .collect();
        let gains = vec![[1.0f32; 3]; 3];
        let flat = Vignetting::flat(1, 4.0);
        let k = 8;
        let mut slots = vec![FaceView::EMPTY; k];
        keep_best_view(&mut slots, view(0, 100.0, TAG_CARRIER));
        keep_best_view(&mut slots, view(1, 10.0, 0));
        filter_views(&mut slots, k, &images, &gains, &flat);
        let kept: Vec<u32> = slots
            .iter()
            .filter(|v| v.score > 0.0)
            .map(|v| v.image)
            .collect();
        assert_eq!(kept, [0, 1], "{slots:?}");
        let mut slots = vec![FaceView::EMPTY; k];
        keep_best_view(&mut slots, view(0, 100.0, TAG_CARRIER));
        keep_best_view(&mut slots, view(1, 10.0, 0));
        keep_best_view(&mut slots, view(2, 40.0, 0));
        filter_views(&mut slots, k, &images, &gains, &flat);
        let kept: Vec<u32> = slots
            .iter()
            .filter(|v| v.score > 0.0)
            .map(|v| v.image)
            .collect();
        assert_eq!(kept, [2, 1], "{slots:?}");
    }

    #[test]
    fn still_person_in_several_frames_is_outvoted() {
        // Somebody stands 0.5 m in front of the +y wall for four frames within a second; three
        // frames from other times show the bare wall. Per frame the person is the majority
        // (4 of 7); per stretch of VOTE_GAP it is one vote of four.
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let body = move |x: f64| Isometry3::translation(x, 3.5 - cam_y, 0.0);
        let person_xs = [0.0, 0.1, 0.2, 0.3];
        let wall_xs = [(3.0, -1.0), (5.0, 1.0), (7.0, -0.6)];
        let mut stamps: Vec<(f64, f64)> = person_xs
            .iter()
            .enumerate()
            .map(|(i, &x)| (1.0 + 0.25 * i as f64, x))
            .collect();
        stamps.extend(wall_xs);
        // Each pose holds from halfway to the previous frame (poses are sampled at 10 Hz).
        let traj = Steps(
            stamps
                .iter()
                .enumerate()
                .map(|(i, &(t, x))| {
                    let from = if i == 0 {
                        0.0
                    } else {
                        T0 + (t + stamps[i - 1].0) / 2.0
                    };
                    (from, body(x))
                })
                .collect(),
        );
        let person = aabb([-0.3, 4.4, -0.4], [0.3, 4.6, 0.5]);
        let shots: Vec<Shot> = stamps
            .iter()
            .enumerate()
            .map(|(i, &(t, x))| {
                if i < person_xs.len() {
                    let cam = cams[0].clone();
                    Shot {
                        camera: 0,
                        stamp: T0 + t,
                        edit: Box::new(move |_| {
                            let scene = Painted::new(vec![(person, [40, 50, 90])]);
                            render(&scene, &cam.camera, &(body(x) * cam.cam_in_body))
                        }),
                    }
                } else {
                    shot(0, T0 + t)
                }
            })
            .collect();
        let images = write_bag("views-still-person", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 8.0), &cams);
        let mut cfg = SurfaceConfig::default();
        // The bare wall is far less sharp than the frames with the person in front.
        cfg.color.min_sharpness = 0.0;
        let k = cfg.views_per_face;
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 7, "{:?}", table.images);
        let behind: Vec<usize> = (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(&mesh, f);
                c[1] > 4.99 && c[0].abs() < 0.2 && c[2] > -0.45 && c[2] < 0.6
            })
            .collect();
        assert!(behind.len() > 10, "{}", behind.len());
        for &f in &behind {
            let v = views(&table, k, f);
            // Ranks 0–3 are the frames with the person.
            assert!(
                v.iter().all(|x| x.score == 0.0 || x.image >= 4),
                "face {f}: {v:?}"
            );
            assert!(
                v.iter().filter(|x| x.score > 0.0).count() >= 2,
                "face {f}: {v:?}"
            );
        }
    }

    #[test]
    fn noisy_normals_share_candidates() {
        // The walk with an image every 0.1 s from both cameras; the room as 5 cm squares with
        // corners moved by up to ±1 cm (normals off by up to about 20°, as on real meshes).
        // Neighbouring faces should mostly keep the same images: else they cannot share one.
        let painted = walk_scene();
        let cams = cameras();
        let shots: Vec<Shot> = (1..=55)
            .flat_map(|i| [0, 1].map(|c| shot(c, T0 + 0.1 * f64::from(i))))
            .collect();
        let images = write_bag("views-noisy", &painted, &walk(), &cams, &shots);
        let src = source(&images, &poses(&walk(), 6.0), &cams);
        let mut mesh = painted_mesh(&painted, 0.05);
        for (i, p) in mesh.positions.iter_mut().enumerate() {
            for (k, x) in p.iter_mut().enumerate() {
                let mut h = (i as u64 * 3 + k as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
                h ^= h >> 29;
                *x += ((h >> 11) as f64 / (1u64 << 53) as f64 - 0.5) as f32 * 0.02;
            }
        }
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        // Faces sharing an edge.
        let mut by_edge: std::collections::BTreeMap<(u32, u32), Vec<usize>> = Default::default();
        for (f, t) in mesh.indices.iter().enumerate() {
            for e in 0..3 {
                let (a, b) = (t[e], t[(e + 1) % 3]);
                by_edge.entry((a.min(b), a.max(b))).or_default().push(f);
            }
        }
        let set = |f: usize| -> Vec<u32> {
            views(&table, k, f)
                .iter()
                .filter(|v| v.score > 0.0)
                .map(|v| v.image)
                .collect()
        };
        let (mut sum, mut pairs) = (0.0, 0usize);
        for fs in by_edge.values().filter(|fs| fs.len() == 2) {
            let (a, b) = (set(fs[0]), set(fs[1]));
            if a.len() < k || b.len() < k {
                continue;
            }
            let common = a.iter().filter(|x| b.contains(x)).count();
            sum += common as f64 / (a.len() + b.len() - common) as f64;
            pairs += 1;
        }
        let jaccard = sum / pairs as f64;
        println!("{pairs} neighbour pairs with full slots, Jaccard {jaccard:.3}");
        assert!(pairs > 1000, "{pairs}");
        assert!(jaccard > 0.83, "{jaccard}");
    }

    /// Points 1 m in front of camera 1 (at the origin, looking along +z), grouped by the mask
    /// cell they appear in: `(cell column, cell row, point)`.
    fn points_by_cell(cams: &[ColorCamera]) -> Vec<(usize, usize, [f32; 3])> {
        let cam1 = &cams[1].camera;
        (0..1_000_000)
            .filter_map(|i| {
                let p = [
                    -5.0 + 0.01 * f64::from(i % 1000),
                    -5.0 + 0.01 * f64::from(i / 1000),
                    1.0,
                ];
                let uv = cam1.project(&Vector3::from(p))?;
                Some((
                    (uv[0] / f64::from(cam1.width) * MASK_CELLS[0] as f64) as usize,
                    (uv[1] / f64::from(cam1.height) * MASK_CELLS[1] as f64) as usize,
                    p.map(|v| v as f32),
                ))
            })
            .collect()
    }

    /// A hand-made view table for [`mask_views`]: `faces` centroids, five images of camera 0
    /// (elsewhere, every face in its colour) and `frames` images of camera 1 at the origin, of
    /// which every face has three (the best views); `carrier(f)` paints those three in
    /// [`CARRIER_BLOB`]; `weak(f)`: the face's camera-0 views score only 1 % of the others.
    fn mask_table(
        faces: &[[f32; 3]],
        frames: u32,
        carrier: impl Fn(usize) -> bool,
        weak: impl Fn(usize) -> bool,
    ) -> (Vec<FaceView>, Vec<UsedImage>) {
        let images: Vec<UsedImage> = (0..5 + frames)
            .map(|i| UsedImage {
                camera: usize::from(i >= 5),
                stamp: f64::from(i),
                world_from_cam: if i >= 5 {
                    Isometry3::identity()
                } else {
                    Isometry3::translation(f64::from(i), -2.0, 0.0)
                },
            })
            .collect();
        let k = 8;
        let mut per_face = vec![FaceView::EMPTY; faces.len() * k];
        for f in 0..faces.len() {
            let slots = &mut per_face[f * k..(f + 1) * k];
            for j in 0..3u32 {
                let image = 5 + (f as u32 + j * 7) % frames;
                let colour = if carrier(f) { CARRIER_BLOB } else { [90; 3] };
                keep_best_view(
                    slots,
                    FaceView {
                        image,
                        score: 100.0 - j as f32,
                        colour,
                        tag: 0,
                    },
                );
            }
            for image in 0..5u32 {
                let score = if weak(f) { 1.0 } else { 50.0 } - image as f32 * 0.1;
                keep_best_view(
                    slots,
                    FaceView {
                        image,
                        score,
                        colour: [90; 3],
                        tag: 0,
                    },
                );
            }
        }
        (per_face, images)
    }

    fn kept(per_face: &[FaceView], f: usize) -> Vec<u32> {
        let mut v: Vec<u32> = per_face[f * 8..(f + 1) * 8]
            .iter()
            .filter(|v| v.score > 0.0)
            .map(|v| v.image)
            .collect();
        v.sort_unstable();
        v
    }

    /// 200 faces in a cell in the lower left of camera 1's image and 200 in a middle cell.
    fn two_cells(cams: &[ColorCamera]) -> (Vec<[f32; 3]>, Vec<bool>) {
        let mut faces = Vec::new();
        let mut lower_left = Vec::new();
        for (x, y, p) in points_by_cell(cams) {
            let ll = (x, y) == (1, 10);
            let mid = (x, y) == (8, 6);
            let have = lower_left.iter().filter(|&&l| l == ll).count();
            if (ll || mid) && have < 200 {
                faces.push(p);
                lower_left.push(ll);
            }
        }
        assert_eq!(faces.len(), 400);
        (faces, lower_left)
    }

    #[test]
    fn image_region_with_carrier_is_masked() {
        // The carrier in the lower left of 30 frames of camera 1; a tenth of those faces have
        // only weak other views and keep the carrier.
        let cams = cameras();
        let (faces, lower_left) = two_cells(&cams);
        let weak = |f: usize| lower_left[f] && f.is_multiple_of(10);
        let (mut per_face, images) = mask_table(&faces, 30, |f| lower_left[f], weak);
        let gains = vec![[1.0f32; 3]; images.len()];
        let flat = Vignetting::flat(2, 4.0);
        let masked = mask_views(&mut per_face, 8, &images, &gains, &flat, &cams, &faces);
        println!("{masked:?}");
        assert_eq!(masked[0].masked, 0);
        assert!(masked[1].masked >= 1, "{masked:?}");
        for (f, &ll) in lower_left.iter().enumerate() {
            let k = kept(&per_face, f);
            if ll && !weak(f) {
                assert_eq!(k, [0, 1, 2, 3, 4], "face {f}");
            } else {
                assert_eq!(k.len(), 8, "face {f}: {k:?}");
            }
        }
    }

    #[test]
    fn few_frames_are_not_masked() {
        // The same, but only 5 frames of camera 1 show the cells: 600 judged views per cell,
        // but from too few images.
        let cams = cameras();
        let (faces, lower_left) = two_cells(&cams);
        let (mut per_face, images) = mask_table(&faces, 5, |f| lower_left[f], |_| false);
        let before = per_face.clone();
        let gains = vec![[1.0f32; 3]; images.len()];
        let masked = mask_views(
            &mut per_face,
            8,
            &images,
            &gains,
            &Vignetting::flat(2, 4.0),
            &cams,
            &faces,
        );
        assert!(masked.iter().all(|m| m.would_mask == 0), "{masked:?}");
        assert_eq!(per_face, before);
    }

    #[test]
    fn too_many_cells_disable_the_mask() {
        // Disagreement all over camera 1's image: that is no carrier; nothing is masked.
        let cams = cameras();
        let mut per_cell = std::collections::BTreeMap::new();
        let faces: Vec<[f32; 3]> = points_by_cell(&cams)
            .into_iter()
            .filter(|&(x, y, _)| {
                let n = per_cell.entry((x, y)).or_insert(0);
                *n += 1;
                *n <= 80
            })
            .map(|(_, _, p)| p)
            .collect();
        let (mut per_face, images) = mask_table(&faces, 30, |_| true, |_| false);
        let before = per_face.clone();
        let gains = vec![[1.0f32; 3]; images.len()];
        let masked = mask_views(
            &mut per_face,
            8,
            &images,
            &gains,
            &Vignetting::flat(2, 4.0),
            &cams,
            &faces,
        );
        println!("{masked:?}");
        assert!(masked[1].would_mask > MASK_MAX_CELLS, "{masked:?}");
        assert_eq!(masked[1].masked, 0);
        assert_eq!(per_face, before);
    }

    #[test]
    fn static_scene_masks_nothing() {
        // The rendered walk without a carrier: no cell of either camera is masked.
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let mesh = painted_mesh(painted, STEP);
        let cfg = SurfaceConfig::default();
        let mut table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        let centroids: Vec<[f32; 3]> = (0..mesh.indices.len())
            .map(|f| centroid(&mesh, f))
            .collect();
        let masked = mask_views(
            &mut table.per_face,
            cfg.views_per_face,
            &table.images,
            &table.gains,
            &table.vignetting,
            cams,
            &centroids,
        );
        println!("{masked:?}");
        assert!(masked.iter().all(|m| m.would_mask == 0), "{masked:?}");
    }

    const CARRIER_BLOB: [u8; 3] = [20, 30, 120];

    #[test]
    fn blurry_image_never_chosen() {
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let mut shots: Vec<Shot> = (1..=11).map(|i| shot(0, T0 + 0.5 * f64::from(i))).collect();
        let blurry = T0 + 2.25;
        shots.push(Shot {
            camera: 0,
            stamp: blurry,
            edit: Box::new(|rgb| blur(&rgb, 6.0)),
        });
        let images = write_bag("views-blurry", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 6.0), &cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(&painted, STEP);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 11, "{:?}", table.images);
        assert!(table.images.iter().all(|i| i.stamp != blurry));
        let seen = table.per_face.iter().filter(|v| v.score > 0.0);
        assert!(seen.clone().count() > mesh.indices.len() / 4);
        assert!(seen.clone().all(|v| (v.image as usize) < 11));
        // Every used image is someone's view, and ranks follow the capture time.
        for r in 0..11u32 {
            assert!(seen.clone().any(|v| v.image == r), "image {r} unused");
        }
        assert!(table.images.windows(2).all(|w| w[0].stamp < w[1].stamp));
    }

    #[test]
    fn gains_match_colorize() {
        // Every second stamp 1.2 times brighter (linear).
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let shots: Vec<Shot> = (1..=11)
            .flat_map(|i| {
                let g = if i % 2 == 0 { 1.2 } else { 1.0 };
                [0, 1].map(move |c| Shot {
                    camera: c,
                    stamp: T0 + 0.5 * f64::from(i),
                    edit: Box::new(move |rgb: Vec<u8>| {
                        rgb.into_iter()
                            .map(|v| {
                                let l = (f64::from(v) / 255.0).powf(2.2) * g;
                                (l.min(1.0).powf(1.0 / 2.2) * 255.0).round() as u8
                            })
                            .collect()
                    }),
                })
            })
            .collect();
        let images = write_bag("views-gains", &painted, &traj, &cams, &shots);
        let poses = poses(&traj, 6.0);
        let mut cloud = surface_cloud(&painted);
        let stats = colorize(
            &mut cloud,
            images.clone(),
            &poses,
            &cams,
            &ColorConfig::default(),
        )
        .unwrap();
        let src = source(&images, &poses, &cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(&painted, STEP);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.gains.len(), stats.gains.len());
        assert_eq!(table.vignetting.coeffs.len(), cams.len());
        let mut spread = (f32::INFINITY, 0.0f32);
        for (ours, theirs) in table.gains.iter().zip(&stats.gains) {
            for c in 0..3 {
                let r = ours[c] / theirs.gain[c];
                assert!((r - 1.0).abs() < 0.02, "{ours:?} vs {theirs:?}");
                spread = (spread.0.min(ours[c]), spread.1.max(ours[c]));
            }
        }
        // The gains differ between the images, so the comparison means something.
        assert!(spread.1 / spread.0 > 1.1, "{spread:?}");
        // Without equalisation every gain is 1.
        let off = SurfaceConfig {
            color: ColorConfig {
                exposure: ExposureConfig {
                    samples: 0,
                    ..ExposureConfig::default()
                },
                ..cfg.color.clone()
            },
            ..cfg.clone()
        };
        let flat = collect_views(&mesh, &src, &off, &Progress::new()).unwrap();
        assert!(flat.gains.iter().all(|g| *g == [1.0; 3]));
        // The same views, except where the photo-consistency check sees the uncorrected
        // brightness steps.
        let k = cfg.views_per_face;
        let faces = mesh.indices.len();
        let same = (0..faces)
            .filter(|&f| views(&flat, k, f) == views(&table, k, f))
            .count();
        println!("same views without equalisation: {same} of {faces}");
        assert!(same as f64 > 0.95 * faces as f64, "{same} of {faces}");
    }

    #[test]
    fn views_deterministic_and_cancellable() {
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(painted, STEP);
        let a = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        let b = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.per_face.len(), mesh.indices.len() * cfg.views_per_face);
        // Each image's pose is the trajectory's at its capture time.
        let offset = cfg.color.time_offset;
        for (r, i) in a.images.iter().enumerate() {
            assert_eq!(a.rank_of(i.camera, i.stamp), Some(r as u32));
            let want = walk().pose(i.stamp + offset) * cams[i.camera].cam_in_body;
            let d = (i.world_from_cam.translation.vector - want.translation.vector).norm();
            assert!(d < 0.01, "{d}");
        }
        assert_eq!(a.rank_of(0, T0 + 0.25), None);
        for phase in ["Bilder prüfen", "Ansichten suchen"] {
            let p = Progress::new();
            let q = p.clone();
            let p = p.with_observer(move |s| {
                if s.phase == phase && s.done == 3 {
                    q.cancel();
                }
            });
            let r = collect_views(&mesh, &src, &cfg, &p);
            assert!(matches!(r, Err(MapError::Cancelled)), "{phase}: {r:?}");
            assert_eq!(p.snapshot().phase, phase);
        }
    }

    /// A GPU for the tests that fails instead of falling back to the CPU (else a broken
    /// kernel would pass unnoticed).
    fn test_gpu() -> Option<GpuContext> {
        GpuContext::new_for_tests().map(|c| c.with_fallback(false))
    }

    /// `mesh` with every vertex moved by up to ±1 cm (normals off by up to about 20°, as on
    /// real meshes).
    fn jittered(mut mesh: Mesh) -> Mesh {
        for (i, p) in mesh.positions.iter_mut().enumerate() {
            for (k, x) in p.iter_mut().enumerate() {
                let mut h = (i as u64 * 3 + k as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
                h ^= h >> 29;
                *x += ((h >> 11) as f64 / (1u64 << 53) as f64 - 0.5) as f32 * 0.02;
            }
        }
        mesh
    }

    /// How far the views of `gpu` are from those of `cpu`: (faces with a view in either,
    /// faces whose best view is the same image, of those the ones whose score differs by more
    /// than 1 %, largest relative score difference of the same best view, and the ones whose
    /// colour differs by more than one level in a channel).
    fn agreement(cpu: &ViewTable, gpu: &ViewTable, k: usize) -> (usize, usize, usize, f32, usize) {
        assert_eq!(cpu.images, gpu.images);
        assert_eq!(cpu.per_face.len(), gpu.per_face.len());
        let (mut seen, mut same, mut off, mut worst, mut tint) = (0, 0, 0, 0.0f32, 0);
        for (c, g) in cpu.per_face.chunks(k).zip(gpu.per_face.chunks(k)) {
            if c[0].score == 0.0 && g[0].score == 0.0 {
                continue;
            }
            seen += 1;
            if c[0].score > 0.0 && c[0].image == g[0].image {
                same += 1;
                let d = (g[0].score - c[0].score).abs() / c[0].score;
                worst = worst.max(d);
                off += usize::from(d > 0.01);
                tint += usize::from((0..3).any(|ch| c[0].colour[ch].abs_diff(g[0].colour[ch]) > 1));
            }
        }
        (seen, same, off, worst, tint)
    }

    #[test]
    fn views_gpu_match_cpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        // The rendered walk (both cameras): the room as 10 cm squares, and as 5 cm squares
        // with jittered corners (smoothed normals matter).
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        for mesh in [
            painted_mesh(painted, STEP),
            jittered(painted_mesh(painted, 0.05)),
        ] {
            let cpu = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
            let (gpu, info) =
                collect_views_on(&mesh, &src, &cfg, &Progress::new(), Some((&ctx, None))).unwrap();
            assert_eq!(info.gpu_fallback, None);
            let (seen, same, off, worst, tint) = agreement(&cpu, &gpu, k);
            println!(
                "{} faces: {seen} with views, best view the same for {same}, score off > 1 % \
                 for {off} (at most {:.4} %), colour off > 1 level for {tint}",
                mesh.indices.len(),
                100.0 * worst
            );
            assert!(seen > mesh.indices.len() / 4, "{seen}");
            assert!(same * 100 >= seen * 99, "{same} of {seen}");
            assert!(off * 100 <= same, "{off} of {same}");
            assert!(tint * 100 <= same, "{tint} of {same}");
            // The exposure comes from the same samples.
            let gain_off = cpu
                .gains
                .iter()
                .zip(&gpu.gains)
                .flat_map(|(c, g)| (0..3).map(move |ch| (c[ch] / g[ch] - 1.0).abs()))
                .fold(0.0f32, f32::max);
            println!(
                "gains: largest relative difference {:.4} %",
                100.0 * gain_off
            );
            for (c, g) in cpu.gains.iter().zip(&gpu.gains) {
                for ch in 0..3 {
                    assert!((c[ch] / g[ch] - 1.0).abs() < 0.001, "{c:?} vs {g:?}");
                }
            }
            // Same device, same result.
            let again = collect_views_with(&mesh, &src, &cfg, &Progress::new(), Some(&ctx));
            assert_eq!(again.unwrap(), gpu);
        }
    }

    #[test]
    fn gpu_views_in_batches_match_one_batch() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(painted, STEP);
        let run = |batch: Option<usize>| {
            collect_views_on(&mesh, &src, &cfg, &Progress::new(), Some((&ctx, batch)))
                .unwrap()
                .0
        };
        let one = run(None);
        let many = run(Some(777));
        assert!(mesh.indices.len() > 3 * 777, "{}", mesh.indices.len());
        assert_eq!(one, many);
    }

    #[test]
    fn cancel_during_gpu_views() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(painted, STEP);
        let p = Progress::new();
        let q = p.clone();
        let p = p.with_observer(move |s| {
            if s.phase == "Ansichten suchen" && s.done == 2 {
                q.cancel();
            }
        });
        let r = collect_views_with(&mesh, &src, &cfg, &p, Some(&ctx));
        assert!(matches!(r, Err(MapError::Cancelled)), "{r:?}");
        let s = p.snapshot();
        assert_eq!(s.phase, "Ansichten suchen");
        assert_eq!(s.done, 2);
    }

    #[test]
    fn gpu_budget_falls_back_or_fails() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(painted, STEP);
        let cpu = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        // `auto`: the CPU, with the reason.
        let small = ctx.with_budget(1000).with_fallback(true);
        let notes = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = notes.clone();
        let p = Progress::with_log(move |l| {
            if let driftless_io::progress::LogLine::Info(s) = l {
                log.lock().unwrap().push(s.clone());
            }
        });
        let (table, info) = collect_views_on(&mesh, &src, &cfg, &p, Some((&small, None))).unwrap();
        assert_eq!(table, cpu);
        let why = info.gpu_fallback.expect("fallback reason");
        assert!(why.contains("Grafikspeicher"), "{why}");
        let notes = notes.lock().unwrap();
        assert!(
            notes
                .iter()
                .any(|n| n.starts_with("Ansichten auf der CPU:")),
            "{notes:?}"
        );
        // `an`: an error in German.
        let strict = small.with_fallback(false);
        let r = collect_views_with(&mesh, &src, &cfg, &Progress::new(), Some(&strict));
        match r {
            Err(MapError::Gpu(e)) => assert!(e.contains("Grafikspeicher"), "{e}"),
            other => panic!("{other:?}"),
        }
    }
}
