//! Alignment of two point clouds (two scans of the same place) for merging them into one
//! model.
//!
//! [`align_pair`] finds the pose of the source cloud in the frame of the target cloud
//! without a start pose:
//! 1. Both clouds are thinned to [`AlignConfig::voxel`]; normals and FPFH features
//!    ([`crate::features`]) are computed on the thinned points and matched where two
//!    features are each other's nearest neighbour.
//! 2. RANSAC over pairs of matches proposes poses with 4 degrees of freedom (turn about the
//!    vertical, shift): the maps are levelled by gravity, up to the IMU start (≤ 0.6° between
//!    the scans of 22.10.). The best [`AlignConfig::candidates`] different poses are kept.
//!    If the best has fewer than [`MIN_INLIERS`] matches, a grid over the turn about the
//!    vertical proposes them instead: per turn the shifts that most pairs of corner points
//!    agree on, each with a coarse ICP.
//! 3. Every candidate is refined by ICP in 6 degrees of freedom, coarse to fine
//!    ([`crate::loops::register`] at 1.0, 0.5, 0.25 m on the thinned points, then one stage
//!    on [`AlignConfig::fine_voxel`] points), and rated by [`pair_quality`].
//!    If no feature candidate is plausible (see 4.), the grid runs as well.
//! 4. The best candidate is the one with the largest overlap among the plausible ones:
//!    edges agree ([`PairQuality::agreement`] ≥ [`MIN_AGREEMENT`]) and the overlap reaches
//!    [`AlignConfig::min_overlap`]. It is also refined turned by 90°, 180° and 270° about
//!    the vertical through the centre of the source ([`SYMMETRY_TURNS`]; once rated as it
//!    is, once refined), and those join the candidates: a room that looks the same turned
//!    is caught even if no feature match proposed the turned pose. Too little overlap or
//!    agreement, a large residual or tilt, or a different candidate almost as good make the
//!    result "unsicher".
//!
//! Why the agreement: walls, floor and ceiling hold most points, and the shell of a
//! rectangular room fits itself turned by 180°. Two scans that each cover part of a room, or
//! two equal rooms on either side of a corridor, overlap *more* in such a wrong pose than in
//! the right one; only the furniture and other edges tell them apart, and in the wrong pose
//! they land where the other scan saw something else (between its floor and ceiling), or a
//! floor lands on a ceiling.
//!
//! Every step is deterministic: a fixed random seed, parallel work collected in input
//! order, reductions in order. The result does not depend on the number of threads.

use crate::features::{
    corner_strength, fpfh, local_shapes, mean_cov, mutual_matches, spread_axes, Fpfh, LocalShape,
    PointTree,
};
use crate::loops::{register, stage_params};
use driftless_io::progress::Progress;
use driftless_odom::icp::{align, IcpParams, IcpTarget};
use driftless_odom::voxel_downsample;
use driftless_odom::voxel_map::FixedState;
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use rayon::prelude::*;
use std::collections::HashMap;

/// Settings of the alignment.
#[derive(Debug, Clone, PartialEq)]
pub struct AlignConfig {
    /// Voxel of the points for features, RANSAC and the coarse ICP stages (m).
    pub voxel: f64,
    /// Neighbours of a normal.
    pub normal_k: usize,
    /// Radius of the FPFH neighbourhood (m).
    pub feature_radius: f64,
    /// RANSAC samples (pairs of feature matches).
    pub ransac_iters: usize,
    /// A feature match is an inlier of a RANSAC pose within this distance (m); a sampled
    /// pair is dropped if its length or height difference disagrees by more.
    pub inlier_dist: f64,
    /// Different candidates refined by ICP (more than [`DISTINCT_TRANS`] or
    /// [`DISTINCT_ROT_DEG`] apart).
    pub candidates: usize,
    /// Step of the grid over the turn about the vertical (°).
    pub yaw_step_deg: f64,
    /// Voxel of the fine ICP stage and of [`pair_quality`] (m).
    pub fine_voxel: f64,
    /// A point overlaps if a point of the other cloud lies this close (m); also the
    /// correspondence distance of the fine ICP stage.
    pub overlap_dist: f64,
    /// Smallest overlap of a sure alignment (share of the source points).
    pub min_overlap: f64,
    /// Largest median residual of a sure alignment (mm).
    pub max_residual_mm: f64,
    /// Largest tilt of the found pose against the vertical (°).
    pub max_tilt_deg: f64,
    /// A different candidate with at least this share of the best overlap makes the
    /// alignment unsure.
    pub ambiguity: f64,
}

impl Default for AlignConfig {
    fn default() -> Self {
        AlignConfig {
            voxel: 0.10,
            normal_k: 12,
            feature_radius: 0.30,
            ransac_iters: 20_000,
            inlier_dist: 0.10,
            candidates: 5,
            yaw_step_deg: 5.0,
            fine_voxel: 0.02,
            overlap_dist: 0.05,
            min_overlap: 0.15,
            max_residual_mm: 5.0,
            max_tilt_deg: 3.0,
            ambiguity: 0.9,
        }
    }
}

/// How the start pose of the ICP was found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignMethod {
    /// FPFH matches and RANSAC.
    Features,
    /// Grid over the turn about the vertical.
    Grid,
    /// Point pairs marked by hand.
    PointPairs,
}

/// Rating of an alignment; the reasons are for the user (German).
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    Sure,
    Unsure(String),
    Failed(String),
}

impl Verdict {
    /// The word of the report and the user interface.
    pub fn word(&self) -> &'static str {
        match self {
            Verdict::Sure => "sicher",
            Verdict::Unsure(_) => "unsicher",
            Verdict::Failed(_) => "gescheitert",
        }
    }
}

/// Fit of two clouds at a pose ([`pair_quality`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairQuality {
    /// Share of the (thinned) source points with a target point within
    /// [`AlignConfig::overlap_dist`] that is not seen from the other side (a floor laid on a
    /// ceiling does not overlap, see [`COLUMN_MARGIN`]).
    pub overlap: f64,
    /// Median and RMS of the distances of those points to the local target surface (mm);
    /// 0 without overlap.
    pub residual_median_mm: f64,
    pub residual_rms_mm: f64,
    /// Angle between the vertical of the source turned by the pose and the vertical (°).
    pub tilt_deg: f64,
    /// How well the edges agree where both clouds saw the place: of the edge and corner
    /// points of one cloud (surface variation ≥ [`EDGE_VARIATION`] on
    /// [`AlignConfig::voxel`] points) that the other cloud could have seen, the share it
    /// has a point within [`AlignConfig::overlap_dist`] of; the smaller of the two
    /// directions. The other cloud could have seen an edge point if it has one that close,
    /// or if it has points more than [`COLUMN_MARGIN`] below and above it in its
    /// [`COLUMN_CELL`] column (it saw the floor and the ceiling there, so the space between
    /// was in view). A point found within the distance but seen from the other side (one
    /// at the bottom of its column, the other at the top) does not agree. 1 if no edge point
    /// could have been seen (no evidence against the pose), 0 for a failed alignment.
    pub agreement: f64,
}

impl PairQuality {
    const NONE: PairQuality = PairQuality {
        overlap: 0.0,
        residual_median_mm: 0.0,
        residual_rms_mm: 0.0,
        tilt_deg: 0.0,
        agreement: 0.0,
    };
}

/// Result of [`align_pair`] or [`align_with_pairs`].
#[derive(Debug, Clone, PartialEq)]
pub struct Alignment {
    /// Maps source world coordinates to target world coordinates (identity if failed).
    pub pose: Isometry3<f64>,
    pub method: AlignMethod,
    pub quality: PairQuality,
    pub verdict: Verdict,
    /// Pose and overlap of the best other candidate after ICP (more than
    /// [`DISTINCT_TRANS`] or [`DISTINCT_ROT_DEG`] away from `pose`), ranked like the best.
    pub runner_up: Option<(Isometry3<f64>, f64)>,
}

/// Two candidates are different if they put the source centroid this far apart (m) …
pub const DISTINCT_TRANS: f64 = 0.3;
/// … or differ by this turn (°).
pub const DISTINCT_ROT_DEG: f64 = 10.0;
/// Fewer RANSAC inliers of the best candidate switch to the grid.
pub const MIN_INLIERS: usize = 12;
/// Smallest [`PairQuality::agreement`] of a sure alignment. Starting value from the
/// synthetic scenes (right poses 0.92–1.0, wrong ones 0.42–0.77); to be checked on real
/// scans.
pub const MIN_AGREEMENT: f64 = 0.85;
/// Points with at least this surface variation count as edge or corner points.
pub const EDGE_VARIATION: f64 = 0.02;
/// Side of the vertical columns of [`PairQuality::agreement`] (m).
pub const COLUMN_CELL: f64 = 0.25;
/// An edge point lies between the lowest and highest point of a column if it is at least
/// this far from both; a point at most this far from the lowest (highest) is at the
/// bottom (top) of the column (m).
pub const COLUMN_MARGIN: f64 = 0.1;
/// The best candidate is also tried turned by every `360° / SYMMETRY_TURNS` about the
/// vertical through the centre of the source (step 4 of the module documentation).
pub const SYMMETRY_TURNS: usize = 4;
/// Correspondence distances of the coarse ICP stages on the thinned points (m).
const STAGES: [f64; 3] = [1.0, 0.5, 0.25];
/// Neighbours and iterations of the fine ICP stage (it starts within a few centimetres; a
/// candidate that still slides after 10 iterations is a wrong one).
const FINE_KNN: usize = 20;
const FINE_ICP_ITERS: u32 = 10;
/// Neighbours of the local target surface in [`pair_quality`].
const QUALITY_KNN: usize = 8;
/// Points of the fine ICP stage, of the overlap, of the agreement and of the feature
/// matching, at most (thinned evenly in input order, so that a pair of scans of 5–6
/// million points takes seconds, not minutes).
const FINE_ICP_POINTS: usize = 50_000;
const QUALITY_POINTS: usize = 200_000;
const EDGE_POINTS: usize = 50_000;
const FEATURE_POINTS: usize = 50_000;
/// The two matches of a RANSAC sample must lie at least this many `inlier_dist` apart
/// horizontally, or their direction gives no usable turn.
const MIN_SAMPLE_SPAN: f64 = 5.0;
/// Fixed seed of the RANSAC samples.
const RANSAC_SEED: u64 = 0x5eed_a11c_0de5_1234;
/// Points with at least this corner strength ([`corner_strength`]) are corners.
const CORNER_STRENGTH: f64 = 0.1;
/// Corner points per cloud, bin size (m) and peaks (shifts tried per turn) of the shift vote
/// of the grid.
const VOTE_POINTS: usize = 500;
const VOTE_BIN: f64 = 0.5;
const VOTE_PEAKS: usize = 3;
/// Voxel (m), source points at most, correspondence distance (m) and iterations of the
/// coarse ICP of each grid start.
const GRID_VOXEL: f64 = 0.25;
const GRID_POINTS: usize = 10_000;
const GRID_STAGE: f64 = 1.0;
const GRID_ICP_ITERS: u32 = 10;

/// The points with finite coordinates (a NaN would stall the eigen decompositions).
fn finite(mut points: Vec<Vector3<f64>>) -> Vec<Vector3<f64>> {
    points.retain(|p| p.iter().all(|c| c.is_finite()));
    points
}

/// Every `ceil(n / max)`-th item, so that at most `max` remain.
fn thin_evenly<T>(points: Vec<T>, max: usize) -> Vec<T> {
    if points.len() <= max || max == 0 {
        return points;
    }
    let step = points.len().div_ceil(max);
    points.into_iter().step_by(step).collect()
}

/// Splittable 64-bit generator (Steele et al. 2014).
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `0..n` (`n > 0`).
    fn below(&mut self, n: usize) -> usize {
        ((u128::from(self.next()) * n as u128) >> 64) as usize
    }
}

/// Distance (m at the source centroid `c`) and turn (°) between two poses.
fn pose_gap(a: &Isometry3<f64>, b: &Isometry3<f64>, c: &Point3<f64>) -> (f64, f64) {
    let trans = (a * c - b * c).norm();
    let rot = (a.rotation.inverse() * b.rotation).angle().to_degrees();
    (trans, rot)
}

fn distinct(a: &Isometry3<f64>, b: &Isometry3<f64>, c: &Point3<f64>) -> bool {
    let (t, r) = pose_gap(a, b, c);
    t > DISTINCT_TRANS || r > DISTINCT_ROT_DEG
}

/// The first `max` of `ranked` (best first) that differ from all chosen before.
fn pick_distinct(
    ranked: impl IntoIterator<Item = Isometry3<f64>>,
    max: usize,
    c: &Point3<f64>,
) -> Vec<Isometry3<f64>> {
    let mut out: Vec<Isometry3<f64>> = Vec::new();
    for pose in ranked {
        if out.len() >= max {
            break;
        }
        if out.iter().all(|o| distinct(o, &pose, c)) {
            out.push(pose);
        }
    }
    out
}

fn yaw_pose(yaw: f64, t: Vector3<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(t),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
    )
}

/// Thinned points and search structures of one cloud.
struct Cloud {
    /// Points on [`AlignConfig::voxel`] with their shapes.
    coarse: PointTree,
    shapes: Vec<Option<LocalShape>>,
    /// Points on [`AlignConfig::fine_voxel`].
    fine: PointTree,
    /// Indices of the edge and corner points of `coarse`, thinned to [`FEATURE_POINTS`]:
    /// the points whose features are matched (a plane looks the same everywhere).
    keypoints: Vec<usize>,
    /// Edge and corner points of `coarse`, thinned to [`EDGE_POINTS`].
    edges: Vec<Vector3<f64>>,
    /// Corner points of `coarse` (corner strength ≥ [`CORNER_STRENGTH`]), thinned to
    /// [`VOTE_POINTS`].
    corners: Vec<Vector3<f64>>,
    /// Lowest and highest fine point of every [`COLUMN_CELL`] column.
    columns: HashMap<[i64; 2], (f64, f64), FixedState>,
}

fn column_key(p: &Vector3<f64>) -> [i64; 2] {
    // `as` saturates for huge values, which only merges far-away garbage into edge cells.
    [
        (p.x / COLUMN_CELL).floor() as i64,
        (p.y / COLUMN_CELL).floor() as i64,
    ]
}

fn columns(points: &[Vector3<f64>]) -> HashMap<[i64; 2], (f64, f64), FixedState> {
    let mut out: HashMap<[i64; 2], (f64, f64), FixedState> = HashMap::default();
    for p in points {
        let e = out.entry(column_key(p)).or_insert((p.z, p.z));
        e.0 = e.0.min(p.z);
        e.1 = e.1.max(p.z);
    }
    out
}

impl Cloud {
    /// `None` with fewer than 3 points on either voxel.
    fn new(points: &[Vector3<f64>], cfg: &AlignConfig) -> Option<Cloud> {
        let (coarse, fine) = rayon::join(
            || finite(voxel_downsample(points, cfg.voxel)),
            || finite(voxel_downsample(points, cfg.fine_voxel)),
        );
        if coarse.len() < 3 || fine.len() < 3 {
            return None;
        }
        let (coarse, fine) = rayon::join(|| PointTree::new(coarse), || PointTree::new(fine));
        let (coarse, fine) = (coarse?, fine?);
        let shapes = local_shapes(&coarse, cfg.normal_k);
        let edge_idx: Vec<usize> = (0..shapes.len())
            .filter(|&i| shapes[i].is_some_and(|s| s.variation >= EDGE_VARIATION))
            .collect();
        let edges: Vec<Vector3<f64>> = edge_idx.iter().map(|&i| coarse.points()[i]).collect();
        let normals: Vec<Option<Vector3<f64>>> =
            shapes.iter().map(|s| s.map(|s| s.normal)).collect();
        let (columns, strength) = rayon::join(
            || columns(fine.points()),
            || corner_strength(&coarse, &normals, cfg.normal_k),
        );
        let corners: Vec<Vector3<f64>> = coarse
            .points()
            .iter()
            .zip(&strength)
            .filter(|(_, &c)| c >= CORNER_STRENGTH)
            .map(|(p, _)| *p)
            .collect();
        Some(Cloud {
            coarse,
            shapes,
            fine,
            keypoints: thin_evenly(edge_idx, FEATURE_POINTS),
            edges: thin_evenly(edges, EDGE_POINTS),
            corners: thin_evenly(corners, VOTE_POINTS),
            columns,
        })
    }

    /// −1 if `p` lies within [`COLUMN_MARGIN`] of the lowest point of its column (a floor,
    /// seen from above), +1 if of the highest (a ceiling, seen from below), else 0 (also in
    /// a column not taller than twice the margin, where up and down are unknown).
    fn side(&self, p: &Vector3<f64>) -> i8 {
        match self.columns.get(&column_key(p)) {
            Some(&(lo, hi)) if hi - lo > 2.0 * COLUMN_MARGIN => {
                if p.z < lo + COLUMN_MARGIN {
                    -1
                } else if p.z > hi - COLUMN_MARGIN {
                    1
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    /// FPFH of the keypoints (`None` elsewhere).
    fn features(&self, cfg: &AlignConfig) -> Vec<Option<Fpfh>> {
        let normals: Vec<Option<Vector3<f64>>> =
            self.shapes.iter().map(|s| s.map(|s| s.normal)).collect();
        let all = fpfh(&self.coarse, &normals, cfg.feature_radius);
        let mut out = vec![None; all.len()];
        for &i in &self.keypoints {
            out[i] = all[i];
        }
        out
    }
}

/// The two clouds and what [`quality`] measures against.
struct Pair {
    src: Cloud,
    tgt: Cloud,
    /// Source points of the overlap and the residual: fine, thinned to [`QUALITY_POINTS`].
    src_quality: Vec<Vector3<f64>>,
}

impl Pair {
    /// `Err` with the reason (German) if either cloud has too few points.
    fn new(
        source: &[Vector3<f64>],
        target: &[Vector3<f64>],
        cfg: &AlignConfig,
    ) -> Result<Pair, String> {
        if source.is_empty() || target.is_empty() {
            return Err("Punktwolke leer".into());
        }
        let sizes = [
            cfg.voxel,
            cfg.fine_voxel,
            cfg.feature_radius,
            cfg.overlap_dist,
        ];
        if !sizes.iter().all(|&x| x > 0.0 && x.is_finite()) {
            return Err("ungültige Einstellungen".into());
        }
        let (src, tgt) = rayon::join(|| Cloud::new(source, cfg), || Cloud::new(target, cfg));
        let (Some(src), Some(tgt)) = (src, tgt) else {
            return Err("zu wenige Punkte".into());
        };
        let src_quality = thin_evenly(src.fine.points().to_vec(), QUALITY_POINTS);
        Ok(Pair {
            src,
            tgt,
            src_quality,
        })
    }

    /// Centroid of the thinned source (reference point of [`pose_gap`]).
    fn centroid(&self) -> Point3<f64> {
        let pts = self.src.coarse.points();
        Point3::from(pts.iter().sum::<Vector3<f64>>() / pts.len() as f64)
    }
}

/// Edge points `edges` of `own` mapped by `pose` that `other` could have seen, and those of
/// them that it has a point within `overlap_dist` of, seen from the same side (see
/// [`PairQuality::agreement`]).
fn edge_counts(
    edges: &[Vector3<f64>],
    own: &Cloud,
    pose: &Isometry3<f64>,
    other: &Cloud,
    overlap_dist: f64,
) -> (usize, usize) {
    let near2 = overlap_dist * overlap_dist;
    let hits: Vec<(bool, bool)> = edges
        .par_iter()
        .map(|e| {
            let q = (pose * Point3::from(*e)).coords;
            if let Some(&(i, d2)) = other.fine.nearest(&q, 1).first() {
                if d2 < near2 {
                    // Seen from the other side (a floor on a ceiling): seen, not agreeing.
                    let opposite = own.side(e) * other.side(&other.fine.points()[i]) < 0;
                    return (true, !opposite);
                }
            }
            let between = other
                .columns
                .get(&column_key(&q))
                .is_some_and(|&(lo, hi)| q.z > lo + COLUMN_MARGIN && q.z < hi - COLUMN_MARGIN);
            (between, false)
        })
        .collect();
    let seen = hits.iter().filter(|h| h.0).count();
    let agree = hits.iter().filter(|h| h.1).count();
    (seen, agree)
}

/// [`PairQuality::agreement`] from the [`edge_counts`] of both directions.
fn agreement(a: (usize, usize), b: (usize, usize)) -> f64 {
    let share = |(seen, agree): (usize, usize)| {
        if seen == 0 {
            1.0
        } else {
            agree as f64 / seen as f64
        }
    };
    share(a).min(share(b))
}

fn tilt_deg(pose: &Isometry3<f64>) -> f64 {
    let up = pose.rotation * Vector3::z();
    up.z.clamp(-1.0, 1.0).acos().to_degrees()
}

/// Fit of the pair at `pose` (see [`PairQuality`]).
fn quality(p: &Pair, pose: &Isometry3<f64>, cfg: &AlignConfig) -> PairQuality {
    let agreement = agreement(
        edge_counts(&p.src.edges, &p.src, pose, &p.tgt, cfg.overlap_dist),
        edge_counts(
            &p.tgt.edges,
            &p.tgt,
            &pose.inverse(),
            &p.src,
            cfg.overlap_dist,
        ),
    );
    let max_d2 = cfg.overlap_dist * cfg.overlap_dist;
    // Neighbours spread less than this along the second axis lie on a line: no surface.
    let min_spread = (0.25 * cfg.fine_voxel).powi(2);
    let pts = p.tgt.fine.points();
    let residuals: Vec<Option<f64>> = p
        .src_quality
        .par_iter()
        .map(|s| {
            let q = (pose * Point3::from(*s)).coords;
            let nn = p.tgt.fine.nearest(&q, QUALITY_KNN);
            let &(i, d2) = nn.first()?;
            if d2 >= max_d2 || p.src.side(s) * p.tgt.side(&pts[i]) < 0 {
                return None;
            }
            let near: Vec<Vector3<f64>> = nn.iter().map(|&(i, _)| pts[i]).collect();
            if near.len() < 3 {
                return Some(d2.sqrt());
            }
            let (c, cov) = mean_cov(&near);
            let axes = spread_axes(&cov);
            if axes[1].0 < min_spread {
                return Some(d2.sqrt());
            }
            Some(axes[0].1.normalize().dot(&(q - c)).abs().min(d2.sqrt()))
        })
        .collect();
    let mut r: Vec<f64> = residuals.into_iter().flatten().collect();
    if r.is_empty() {
        return PairQuality {
            tilt_deg: tilt_deg(pose),
            agreement,
            ..PairQuality::NONE
        };
    }
    let overlap = r.len() as f64 / p.src_quality.len() as f64;
    r.sort_by(f64::total_cmp);
    let mid = r.len() / 2;
    let median = if r.len().is_multiple_of(2) {
        0.5 * (r[mid - 1] + r[mid])
    } else {
        r[mid]
    };
    let rms = (r.iter().map(|x| x * x).sum::<f64>() / r.len() as f64).sqrt();
    PairQuality {
        overlap,
        residual_median_mm: median * 1e3,
        residual_rms_mm: rms * 1e3,
        tilt_deg: tilt_deg(pose),
        agreement,
    }
}

/// Fit of `source` against `target` at `pose` (source world → target world). Both clouds
/// are thinned to [`AlignConfig::fine_voxel`], the source evenly to at most 200 000 points.
/// Overlap is the share of source points with a target point within
/// [`AlignConfig::overlap_dist`]; their residual is the distance to the plane of the 8
/// nearest target points. All zero (tilt aside) if either cloud has fewer than 3 points.
pub fn pair_quality(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    pose: &Isometry3<f64>,
    cfg: &AlignConfig,
) -> PairQuality {
    match Pair::new(source, target, cfg) {
        Ok(p) => quality(&p, pose, cfg),
        Err(_) => PairQuality {
            tilt_deg: tilt_deg(pose),
            ..PairQuality::NONE
        },
    }
}

/// Kabsch: the rigid pose mapping the first point of each pair onto the second in the
/// least-squares sense. `None` with fewer than 3 pairs, a coordinate that is not finite, or
/// if either side lies almost on a line (the spread across the line is below 2 % of the
/// spread along it, or below 1 mm).
pub fn pose_from_pairs(pairs: &[(Vector3<f64>, Vector3<f64>)]) -> Option<Isometry3<f64>> {
    let finite_pair =
        |p: &(Vector3<f64>, Vector3<f64>)| p.0.iter().chain(p.1.iter()).all(|c| c.is_finite());
    if pairs.len() < 3 || !pairs.iter().all(finite_pair) {
        return None;
    }
    let src: Vec<Vector3<f64>> = pairs.iter().map(|p| p.0).collect();
    let tgt: Vec<Vector3<f64>> = pairs.iter().map(|p| p.1).collect();
    let (cs, cov_s) = mean_cov(&src);
    let (ct, cov_t) = mean_cov(&tgt);
    for cov in [cov_s, cov_t] {
        let axes = spread_axes(&cov);
        let (mid, max) = (axes[1].0.max(0.0).sqrt(), axes[2].0.max(0.0).sqrt());
        if mid < 0.02 * max || mid < 1e-3 {
            return None;
        }
    }
    let h = src.iter().zip(&tgt).fold(Matrix3::zeros(), |acc, (s, t)| {
        acc + (t - ct) * (s - cs).transpose()
    });
    let svd = h.svd(true, true);
    let (u, v_t) = (svd.u?, svd.v_t?);
    // Turn, not mirror: flip the axis of the smallest singular value if needed.
    let d = (u * v_t).determinant().signum();
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| svd.singular_values[a].total_cmp(&svd.singular_values[b]));
    let mut fix = Matrix3::identity();
    fix[(order[0], order[0])] = d;
    let rotation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(u * fix * v_t));
    let t = ct - rotation * cs;
    Some(Isometry3::from_parts(Translation3::from(t), rotation))
}

/// The ICP targets and thinned sources shared by all candidates.
struct Icp {
    coarse_target: IcpTarget,
    fine_target: IcpTarget,
    /// Fine source points, thinned to [`FINE_ICP_POINTS`].
    fine_source: Vec<Vector3<f64>>,
}

impl Icp {
    fn new(p: &Pair) -> Option<Icp> {
        Some(Icp {
            coarse_target: IcpTarget::new(p.tgt.coarse.points().to_vec())?,
            fine_target: IcpTarget::new(p.tgt.fine.points().to_vec())?,
            fine_source: thin_evenly(p.src.fine.points().to_vec(), FINE_ICP_POINTS),
        })
    }
}

/// ICP parameters of the fine stage: correspondences within `overlap_dist`, planes from 20
/// neighbours (on 2 cm points 5 neighbours span too little for the plane tests of the
/// coarse stages), flatter planes accepted (6.5 mm noise on a 5 cm patch), weak prior.
fn fine_params(cfg: &AlignConfig) -> IcpParams {
    IcpParams {
        knn: FINE_KNN,
        plane_tolerance: cfg.overlap_dist,
        min_plane_spread: 0.5 * cfg.fine_voxel,
        max_flatness: 0.1,
        kernel: 0.5 * cfg.overlap_dist,
        max_iter: FINE_ICP_ITERS,
        ..stage_params(cfg.overlap_dist)
    }
}

/// ICP of the source from `init` (coarse stages, then the fine stage) and its quality.
fn refine(
    p: &Pair,
    icp: &Icp,
    init: &Isometry3<f64>,
    cfg: &AlignConfig,
) -> (Isometry3<f64>, PairQuality) {
    let (coarse, _, _) = register(p.src.coarse.points(), &icp.coarse_target, init, &STAGES);
    let fine = align(
        &icp.fine_source,
        &icp.fine_target,
        &coarse,
        &fine_params(cfg),
    )
    .pose;
    (fine, quality(p, &fine, cfg))
}

/// RANSAC pose of the sample `(i, j)` and its inlier count, or `None` if the two matches do
/// not agree in length and height difference or lie too close together.
fn hypothesis(
    matches: &[(Vector3<f64>, Vector3<f64>)],
    i: usize,
    j: usize,
    cfg: &AlignConfig,
) -> Option<(Isometry3<f64>, usize)> {
    let (s1, t1) = matches[i];
    let (s2, t2) = matches[j];
    let (ds, dt) = (s2 - s1, t2 - t1);
    if (ds.norm() - dt.norm()).abs() > cfg.inlier_dist || (ds.z - dt.z).abs() > cfg.inlier_dist {
        return None;
    }
    let span = MIN_SAMPLE_SPAN * cfg.inlier_dist;
    if ds.xy().norm() < span || dt.xy().norm() < span {
        return None;
    }
    let yaw = dt.y.atan2(dt.x) - ds.y.atan2(ds.x);
    let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
    let t = 0.5 * (t1 + t2) - rot * (0.5 * (s1 + s2));
    let max_d2 = cfg.inlier_dist * cfg.inlier_dist;
    let inliers = matches
        .iter()
        .filter(|(s, tg)| (rot * s + t - tg).norm_squared() < max_d2)
        .count();
    Some((yaw_pose(yaw, t), inliers))
}

/// Candidates from feature matches (source point, target point): the best different RANSAC
/// poses, most inliers first, and the inlier count of the best.
fn ransac(
    matches: &[(Vector3<f64>, Vector3<f64>)],
    cfg: &AlignConfig,
    c: &Point3<f64>,
) -> (Vec<Isometry3<f64>>, usize) {
    let m = matches.len();
    if m < 2 {
        return (Vec::new(), 0);
    }
    let mut rng = SplitMix64(RANSAC_SEED);
    let samples: Vec<(usize, usize)> = (0..cfg.ransac_iters)
        .map(|_| {
            let i = rng.below(m);
            let mut j = rng.below(m - 1);
            if j >= i {
                j += 1;
            }
            (i, j)
        })
        .collect();
    let found: Vec<Option<(Isometry3<f64>, usize)>> = samples
        .par_iter()
        .map(|&(i, j)| hypothesis(matches, i, j, cfg))
        .collect();
    let mut ranked: Vec<(usize, Isometry3<f64>, usize)> = found
        .into_iter()
        .enumerate()
        .filter_map(|(k, h)| h.map(|(pose, n)| (k, pose, n)))
        .collect();
    ranked.sort_by(|a, b| b.2.cmp(&a.2).then(a.0.cmp(&b.0)));
    let best = ranked.first().map_or(0, |r| r.2);
    let poses = pick_distinct(ranked.into_iter().map(|r| r.1), cfg.candidates, c);
    (poses, best)
}

/// Shifts that most corner point pairs (`src` turned by `rot`, `tgt`) agree on: the centres
/// of the [`VOTE_PEAKS`] [`VOTE_BIN`] bins with the most votes that are not next to a
/// better one, most votes first. Empty without points.
fn vote_shifts(
    src: &[Vector3<f64>],
    tgt: &[Vector3<f64>],
    rot: &UnitQuaternion<f64>,
) -> Vec<Vector3<f64>> {
    let mut votes: HashMap<[i64; 3], usize, FixedState> = HashMap::default();
    for s in src {
        let r = rot * s;
        for t in tgt {
            let v = (t - r) / VOTE_BIN;
            let key = [v.x.floor() as i64, v.y.floor() as i64, v.z.floor() as i64];
            *votes.entry(key).or_insert(0) += 1;
        }
    }
    // Most votes first, ties to the smaller bin (independent of the map order).
    let mut bins: Vec<([i64; 3], usize)> = votes.into_iter().collect();
    bins.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let mut peaks: Vec<([i64; 3], usize)> = Vec::new();
    for (key, n) in bins {
        if peaks.len() >= VOTE_PEAKS {
            break;
        }
        let next_to = |k: &[i64; 3]| (0..3).all(|i| (k[i] - key[i]).abs() <= 1);
        if !peaks.iter().any(|(k, _)| next_to(k)) {
            peaks.push((key, n));
        }
    }
    peaks
        .into_iter()
        .map(|(k, _)| {
            Vector3::new(k[0] as f64, k[1] as f64, k[2] as f64).add_scalar(0.5) * VOTE_BIN
        })
        .collect()
}

/// Candidates from the grid over the turn about the vertical. For every turn the
/// [`VOTE_PEAKS`] shifts that most pairs of corner points agree on (without corners: the
/// shift that puts the centres of the bounding boxes together), each refined by a coarse
/// ICP on [`GRID_VOXEL`] points. Ranked like the final candidates, but on the thinned
/// edge points and within `inlier_dist`: agreeing edges first, then the most edge points
/// found in the other cloud (both directions).
fn grid(
    p: &Pair,
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    cfg: &AlignConfig,
    c: &Point3<f64>,
) -> Vec<Isometry3<f64>> {
    let src = thin_evenly(finite(voxel_downsample(source, GRID_VOXEL)), GRID_POINTS);
    let Some(tgt) = IcpTarget::new(finite(voxel_downsample(target, GRID_VOXEL))) else {
        return Vec::new();
    };
    let (Some(sc), Some(tc)) = (
        box_centre(p.src.coarse.points()),
        box_centre(p.tgt.coarse.points()),
    ) else {
        return Vec::new();
    };
    let src_edges = thin_evenly(p.src.edges.clone(), VOTE_POINTS);
    let tgt_edges = thin_evenly(p.tgt.edges.clone(), VOTE_POINTS);
    let icp = IcpParams {
        max_iter: GRID_ICP_ITERS,
        ..stage_params(GRID_STAGE)
    };
    let steps = if cfg.yaw_step_deg > 0.0 {
        ((360.0 / cfg.yaw_step_deg).round() as usize).clamp(1, 3600)
    } else {
        1
    };
    let starts: Vec<Isometry3<f64>> = (0..steps)
        .into_par_iter()
        .flat_map_iter(|k| {
            let yaw = (k as f64 * 360.0 / steps as f64).to_radians();
            let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
            let mut shifts = vote_shifts(&p.src.corners, &p.tgt.corners, &rot);
            if shifts.is_empty() {
                shifts.push(tc - rot * sc);
            }
            shifts.into_iter().map(move |shift| yaw_pose(yaw, shift))
        })
        .collect();
    let tried: Vec<(Isometry3<f64>, bool, usize)> = starts
        .par_iter()
        .map(|init| {
            let pose = align(&src, &tgt, init, &icp).pose;
            let a = edge_counts(&src_edges, &p.src, &pose, &p.tgt, cfg.inlier_dist);
            let b = edge_counts(&tgt_edges, &p.tgt, &pose.inverse(), &p.src, cfg.inlier_dist);
            (pose, agreement(a, b) >= MIN_AGREEMENT, a.1 + b.1)
        })
        .collect();
    let mut order: Vec<usize> = (0..tried.len()).collect();
    order.sort_by(|&a, &b| {
        (tried[b].1.cmp(&tried[a].1))
            .then(tried[b].2.cmp(&tried[a].2))
            .then(a.cmp(&b))
    });
    pick_distinct(order.into_iter().map(|k| tried[k].0), cfg.candidates, c)
}

/// Centre of the bounding box.
fn box_centre(points: &[Vector3<f64>]) -> Option<Vector3<f64>> {
    let first = points.first()?;
    let (lo, hi) = points
        .iter()
        .fold((*first, *first), |(lo, hi), p| (lo.inf(p), hi.sup(p)));
    Some(0.5 * (lo + hi))
}

fn failed(method: AlignMethod, reason: impl Into<String>) -> Alignment {
    Alignment {
        pose: Isometry3::identity(),
        method,
        quality: PairQuality::NONE,
        verdict: Verdict::Failed(reason.into()),
        runner_up: None,
    }
}

fn cancelled(method: AlignMethod) -> Alignment {
    failed(method, "abgebrochen")
}

/// Whether the edges of a candidate agree.
fn agrees(q: &PairQuality) -> bool {
    q.agreement >= MIN_AGREEMENT
}

/// Whether a candidate could be sure as far as overlap and edges go; such candidates rank
/// before all others.
fn plausible(q: &PairQuality, cfg: &AlignConfig) -> bool {
    agrees(q) && q.overlap >= cfg.min_overlap
}

/// Verdict of the best candidate: unsure for every threshold it misses (reasons joined by
/// "; "), and for a runner-up (`runner_up`: its quality) that is as plausible and has at
/// least `ambiguity` of its overlap.
fn judge(q: &PairQuality, runner_up: Option<&PairQuality>, cfg: &AlignConfig) -> Verdict {
    let mut why = Vec::new();
    if q.overlap < cfg.min_overlap {
        why.push(format!("Überlappung nur {:.0} %", 100.0 * q.overlap));
    }
    if !agrees(q) {
        why.push(format!("Kanten passen nur zu {:.0} %", 100.0 * q.agreement));
    }
    if q.residual_median_mm > cfg.max_residual_mm {
        why.push(format!("Restfehler {:.1} mm", q.residual_median_mm));
    }
    if q.tilt_deg > cfg.max_tilt_deg {
        why.push(format!("Neigung {:.1}°", q.tilt_deg));
    }
    if let Some(r) = runner_up {
        if plausible(r, cfg) == plausible(q, cfg) && r.overlap >= cfg.ambiguity * q.overlap {
            why.push(format!(
                "zweite Lage fast so gut ({:.0} % gegen {:.0} % Überlappung)",
                100.0 * r.overlap,
                100.0 * q.overlap
            ));
        }
    }
    if why.is_empty() {
        Verdict::Sure
    } else {
        Verdict::Unsure(why.join("; "))
    }
}

/// Order of the candidates, best first: plausible first, then the largest overlap; ties keep
/// the candidate order.
fn rank(refined: &[(Isometry3<f64>, PairQuality, AlignMethod)], cfg: &AlignConfig) -> Vec<usize> {
    let mut order: Vec<usize> = (0..refined.len()).collect();
    order.sort_by(|&a, &b| {
        let (qa, qb) = (&refined[a].1, &refined[b].1);
        plausible(qb, cfg)
            .cmp(&plausible(qa, cfg))
            .then(qb.overlap.total_cmp(&qa.overlap))
            .then(a.cmp(&b))
    });
    order
}

/// Aligns `source` to `target` without a start pose (see the module documentation).
/// `pose` maps source world coordinates to target world coordinates. Returns early with
/// `Failed("abgebrochen")` when `progress` is cancelled; phase and count are left to the
/// caller.
pub fn align_pair(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    cfg: &AlignConfig,
    progress: &Progress,
) -> Alignment {
    if progress.is_cancelled() {
        return cancelled(AlignMethod::Features);
    }
    let p = match Pair::new(source, target, cfg) {
        Ok(p) => p,
        Err(why) => return failed(AlignMethod::Features, why),
    };
    let Some(icp) = Icp::new(&p) else {
        return failed(AlignMethod::Features, "zu wenige Punkte");
    };
    let centroid = p.centroid();
    if progress.is_cancelled() {
        return cancelled(AlignMethod::Features);
    }
    let (src_feat, tgt_feat) = rayon::join(|| p.src.features(cfg), || p.tgt.features(cfg));
    if progress.is_cancelled() {
        return cancelled(AlignMethod::Features);
    }
    let (src_pts, tgt_pts) = (p.src.coarse.points(), p.tgt.coarse.points());
    let matches: Vec<(Vector3<f64>, Vector3<f64>)> = mutual_matches(&src_feat, &tgt_feat)
        .into_iter()
        .map(|(i, j)| (src_pts[i], tgt_pts[j]))
        .collect();
    let (starts, best_inliers) = ransac(&matches, cfg, &centroid);
    // Refined candidates with the method that proposed them.
    let mut refined: Vec<(Isometry3<f64>, PairQuality, AlignMethod)> = Vec::new();
    if best_inliers >= MIN_INLIERS {
        for s in &starts {
            if progress.is_cancelled() {
                return cancelled(AlignMethod::Features);
            }
            let (pose, q) = refine(&p, &icp, s, cfg);
            refined.push((pose, q, AlignMethod::Features));
        }
    }
    // The grid is the second way when the features give nothing that could be sure.
    if !refined.iter().any(|r| plausible(&r.1, cfg)) {
        if progress.is_cancelled() {
            return cancelled(AlignMethod::Grid);
        }
        for s in &grid(&p, source, target, cfg, &centroid) {
            if progress.is_cancelled() {
                return cancelled(AlignMethod::Grid);
            }
            let (pose, q) = refine(&p, &icp, s, cfg);
            refined.push((pose, q, AlignMethod::Grid));
        }
    }
    let Some(&first) = rank(&refined, cfg).first() else {
        return failed(AlignMethod::Grid, "keine Lage gefunden");
    };
    // The best so far turned about the vertical through the centre of the source: a room
    // that looks the same turned (a rectangle by 180°, a square by 90°) fits as well there,
    // and the matches need not have proposed it (a feature and its turned twin match only
    // one of them). As a candidate it makes such a result unsure.
    let (start, _, method) = refined[first];
    if let Some(c) = box_centre(p.src.coarse.points()) {
        for k in 1..SYMMETRY_TURNS {
            if progress.is_cancelled() {
                return cancelled(method);
            }
            let yaw = std::f64::consts::TAU * k as f64 / SYMMETRY_TURNS as f64;
            let turn =
                Translation3::from(c) * yaw_pose(yaw, Vector3::zeros()) * Translation3::from(-c);
            // A source that looks the same turned is already in place there; it is rated
            // as it is, since the ICP can drag it away (towards furniture only the target
            // has, 7–10 cm in the synthetic room of the merge tests). The refined one
            // covers a source that looks almost the same.
            let init = start * turn;
            refined.push((init, quality(&p, &init, cfg), method));
            let (pose, q) = refine(&p, &icp, &init, cfg);
            refined.push((pose, q, method));
        }
    }
    let order = rank(&refined, cfg);
    let best = order[0];
    let (pose, quality, method) = refined[best];
    let runner = order
        .iter()
        .skip(1)
        .map(|&k| &refined[k])
        .find(|r| distinct(&pose, &r.0, &centroid));
    Alignment {
        pose,
        method,
        quality,
        verdict: judge(&quality, runner.map(|r| &r.1), cfg),
        runner_up: runner.map(|r| (r.0, r.1.overlap)),
    }
}

/// Aligns `source` to `target` from point pairs marked by hand (source point, target point):
/// [`pose_from_pairs`] as start, then ICP and rating as in [`align_pair`] with this one
/// candidate and without the ambiguity test (the user chose the place).
pub fn align_with_pairs(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    pairs: &[(Vector3<f64>, Vector3<f64>)],
    cfg: &AlignConfig,
    progress: &Progress,
) -> Alignment {
    let method = AlignMethod::PointPairs;
    if progress.is_cancelled() {
        return cancelled(method);
    }
    let p = match Pair::new(source, target, cfg) {
        Ok(p) => p,
        Err(why) => return failed(method, why),
    };
    let Some(icp) = Icp::new(&p) else {
        return failed(method, "zu wenige Punkte");
    };
    let Some(init) = pose_from_pairs(pairs) else {
        return failed(
            method,
            "mindestens drei Punktpaare nötig, nicht auf einer Linie",
        );
    };
    if progress.is_cancelled() {
        return cancelled(method);
    }
    let (pose, quality) = refine(&p, &icp, &init, cfg);
    Alignment {
        pose,
        method,
        quality,
        verdict: judge(&quality, None, cfg),
        runner_up: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uniform(rng: &mut SplitMix64, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (rng.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn random_point(rng: &mut SplitMix64) -> Vector3<f64> {
        Vector3::new(
            uniform(rng, 0.0, 8.0),
            uniform(rng, 0.0, 5.0),
            uniform(rng, 0.0, 2.8),
        )
    }

    #[test]
    fn ransac_finds_the_pose_among_wrong_matches() {
        let truth = yaw_pose(0.6, Vector3::new(1.0, -2.0, 0.1));
        let mut rng = SplitMix64(3);
        let mut matches = Vec::new();
        for k in 0..440 {
            let s = random_point(&mut rng);
            let t = if k % 11 == 0 {
                // 40 right matches with 2 cm noise.
                let n = Vector3::new(
                    uniform(&mut rng, -0.02, 0.02),
                    uniform(&mut rng, -0.02, 0.02),
                    0.0,
                );
                (truth * Point3::from(s)).coords + n
            } else {
                random_point(&mut rng)
            };
            matches.push((s, t));
        }
        let c = Point3::new(4.0, 2.5, 1.4);
        let (poses, best) = ransac(&matches, &AlignConfig::default(), &c);
        assert!(best >= 40, "{best} inliers");
        let (dt, dr) = pose_gap(&poses[0], &truth, &c);
        assert!(dt < 0.05 && dr < 1.0, "{dt} m {dr}°");
        assert!(poses.len() <= AlignConfig::default().candidates);
        for (i, a) in poses.iter().enumerate() {
            for b in &poses[i + 1..] {
                assert!(distinct(a, b, &c));
            }
        }
    }

    #[test]
    fn ransac_without_matches_gives_nothing() {
        let c = Point3::origin();
        let cfg = AlignConfig::default();
        assert_eq!(ransac(&[], &cfg, &c).1, 0);
        let one = [(Vector3::zeros(), Vector3::zeros())];
        assert!(ransac(&one, &cfg, &c).0.is_empty());
    }

    #[test]
    fn vote_finds_the_shift() {
        let mut rng = SplitMix64(5);
        let src: Vec<Vector3<f64>> = (0..60).map(|_| random_point(&mut rng)).collect();
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.1);
        let shift = Vector3::new(3.3, -1.7, 0.4);
        let tgt: Vec<Vector3<f64>> = src.iter().map(|s| rot * s + shift).collect();
        let peaks = vote_shifts(&src, &tgt, &rot);
        assert!(!peaks.is_empty() && peaks.len() <= VOTE_PEAKS);
        assert!(
            (peaks[0] - shift).amax() <= 0.5 * VOTE_BIN + 1e-9,
            "{:?}",
            peaks[0]
        );
        assert!(vote_shifts(&[], &tgt, &rot).is_empty());
    }

    #[test]
    fn thin_evenly_keeps_at_most_max() {
        let v: Vec<usize> = (0..10).collect();
        assert_eq!(thin_evenly(v.clone(), 4), vec![0, 3, 6, 9]);
        assert_eq!(thin_evenly(v.clone(), 10), v);
        assert_eq!(thin_evenly(v.clone(), 0), v);
    }

    #[test]
    fn floor_and_ceiling_sides() {
        let mut pts = Vec::new();
        for i in 0..50 {
            for j in 0..50 {
                let (x, y) = (i as f64 * 0.02, j as f64 * 0.02);
                pts.push(Vector3::new(x, y, 0.0));
                pts.push(Vector3::new(x, y, 2.5));
            }
        }
        let cloud = Cloud::new(&pts, &AlignConfig::default()).expect("cloud");
        assert_eq!(cloud.side(&Vector3::new(0.5, 0.5, 0.02)), -1);
        assert_eq!(cloud.side(&Vector3::new(0.5, 0.5, 2.48)), 1);
        assert_eq!(cloud.side(&Vector3::new(0.5, 0.5, 1.2)), 0);
        // Outside the scanned columns up and down are unknown.
        assert_eq!(cloud.side(&Vector3::new(5.0, 5.0, 0.0)), 0);
    }

    fn good() -> PairQuality {
        PairQuality {
            overlap: 0.7,
            residual_median_mm: 2.0,
            residual_rms_mm: 3.0,
            tilt_deg: 0.2,
            agreement: 0.98,
        }
    }

    #[test]
    fn judge_names_every_reason() {
        let cfg = AlignConfig::default();
        assert_eq!(judge(&good(), None, &cfg), Verdict::Sure);
        let bad = PairQuality {
            overlap: 0.1,
            residual_median_mm: 7.0,
            tilt_deg: 4.0,
            agreement: 0.5,
            ..good()
        };
        let Verdict::Unsure(why) = judge(&bad, None, &cfg) else {
            panic!("not unsure");
        };
        for word in ["Überlappung", "Kanten", "Restfehler", "Neigung"] {
            assert!(why.contains(word), "{why}");
        }
    }

    #[test]
    fn judge_ambiguity_needs_an_equally_plausible_runner_up() {
        let cfg = AlignConfig::default();
        let twin = good();
        assert!(
            matches!(judge(&good(), Some(&twin), &cfg), Verdict::Unsure(w) if w.contains("zweite Lage"))
        );
        // A turned shell overlaps more but its edges disagree: no ambiguity.
        let shell = PairQuality {
            overlap: 0.9,
            agreement: 0.7,
            ..good()
        };
        assert_eq!(judge(&good(), Some(&shell), &cfg), Verdict::Sure);
        let weaker = PairQuality {
            overlap: 0.5,
            ..good()
        };
        assert_eq!(judge(&good(), Some(&weaker), &cfg), Verdict::Sure);
    }

    #[test]
    fn pose_from_pairs_rejects_non_finite() {
        let p = |x: f64| (Vector3::new(x, 0.0, 0.0), Vector3::new(x, 0.0, 0.0));
        let mut pairs = vec![
            p(0.0),
            p(1.0),
            (Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        ];
        assert!(pose_from_pairs(&pairs).is_some());
        pairs[1].1.x = f64::NAN;
        assert!(pose_from_pairs(&pairs).is_none());
    }
}
