//! Cleaning of the dense cloud: ghosts (people and things that moved) and isolated outliers.
//!
//! Ghosts are found by free-space counting: every sensor ray of the [`RaySource`] is walked
//! through a coarse voxel grid (3D DDA). A voxel that rays keep passing through, much more
//! often than rays end in it, was empty most of the scan; points in it are ghosts. Only voxels
//! that hold cloud points are counted, so memory follows the cloud, not the rays.
//!
//! Two details keep surfaces seen at a flat angle (floors, walls along the path) from being
//! taken for free space; without them the synthetic test room loses 3–5 % of its walls:
//! - A ray ends within its noise: the [`END_MARGIN`] voxels before its end voxel count as
//!   *hit*, not passed. A surface lying along a voxel face splits its points between two
//!   voxel layers, and the rays ending just behind the face then hit the front layer too.
//! - A ray crossing a voxel's surface flatter than [`MIN_INCIDENCE_SIN`] does not pass it:
//!   it runs inside the surface's voxel layer for a long way before it ends on it.
//!
//! Outliers are points whose mean distance to their nearest neighbours is far above that of
//! the cloud (statistical outlier removal).
//!
//! Both are a pure per-element step (one ray, one point; run in parallel) and a reduction
//! that does not depend on the order (integer counters) or runs in point order, so every run
//! gives the same result.

use crate::{Cloud, MapError, RaySource};
use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use std::collections::HashMap;
use std::num::NonZeroUsize;
use std::sync::atomic::{AtomicU32, Ordering};

/// Voxels right before a ray's end voxel that count as hit rather than passed: the end point
/// is noisy and the sensor position is the one at the frame end, so a ray may stop a little
/// short of or beyond the surface it hit.
const END_MARGIN: u32 = 2;

/// Sine of the flattest angle between a ray and a voxel's surface at which the ray still
/// counts as passing the voxel (17.5°). A ray at angle `a` runs about `1 / tan a` voxels
/// inside the voxel layer of a surface before it ends on it, [`END_MARGIN`] of them hit: at
/// this angle about one pass per ray remains, well below [`CleanConfig::ghost_ratio`].
const MIN_INCIDENCE_SIN: f64 = 0.3;

#[derive(Debug, Clone, PartialEq)]
pub struct CleanConfig {
    /// Edge of the voxels for free-space counting (m).
    pub ghost_voxel: f64,
    /// A voxel is free space if rays pass through it at least this many times as often as
    /// rays hit it ...
    pub ghost_ratio: f64,
    /// ... and at least this many times.
    pub min_passes: u32,
    /// Neighbours for the mean neighbour distance of the outlier filter.
    pub outlier_k: usize,
    /// A point is an outlier if its mean neighbour distance lies more than this many
    /// standard deviations above the mean over the cloud.
    pub outlier_sigma: f64,
    /// Remove ghosts in [`clean`].
    pub ghosts: bool,
    /// Remove outliers in [`clean`].
    pub outliers: bool,
}

impl Default for CleanConfig {
    fn default() -> Self {
        CleanConfig {
            ghost_voxel: 0.1,
            ghost_ratio: 3.0,
            min_passes: 5,
            outlier_k: 16,
            outlier_sigma: 2.5,
            ghosts: true,
            outliers: true,
        }
    }
}

impl CleanConfig {
    fn check(&self) -> Result<(), MapError> {
        let bad = |msg: &str| Err(MapError::Config(msg.into()));
        if !(self.ghost_voxel.is_finite() && self.ghost_voxel > 0.0) {
            return bad("Voxelgröße der Geistersuche muss größer als 0 sein");
        }
        if !(self.ghost_ratio.is_finite() && self.ghost_ratio >= 0.0) {
            return bad("Verhältnis der Geistersuche darf nicht negativ sein");
        }
        // With 0, a voxel no ray touches (0 passes ≥ ratio · 0 hits) would be free space.
        if self.min_passes == 0 {
            return bad("Mindestzahl der Durchgänge der Geistersuche muss mindestens 1 sein");
        }
        if self.outlier_k == 0 {
            return bad("Nachbarzahl des Ausreißerfilters muss mindestens 1 sein");
        }
        if !(self.outlier_sigma.is_finite() && self.outlier_sigma >= 0.0) {
            return bad("Schwelle des Ausreißerfilters darf nicht negativ sein");
        }
        Ok(())
    }
}

/// Point counts of [`clean`]: removed by reason (each point counts once, ghost before
/// outlier) and kept.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CleanStats {
    pub ghosts: usize,
    pub outliers: usize,
    pub kept: usize,
}

fn vec3(p: &[f32; 3]) -> Vector3<f64> {
    Vector3::from(p.map(f64::from))
}

fn key(p: &[f32; 3], voxel: f64) -> VoxelKey {
    voxel_key(&vec3(p), voxel)
}

/// Walks the ray `from` → `to` through the grid of edge `voxel` (Amanatides–Woo) and calls
/// `visit(voxel, at_end)` for every voxel it crosses, in order, the end voxel included;
/// `at_end` is true for the end voxel and the [`END_MARGIN`] voxels before it.
///
/// The walk takes exactly as many steps as the end voxel is away (sum over the axes), each
/// on an axis that still has steps left, so it always stops in the end voxel.
fn walk_ray(from: &[f32; 3], to: &[f32; 3], voxel: f64, mut visit: impl FnMut(VoxelKey, bool)) {
    let (start, end) = (key(from, voxel), key(to, voxel));
    let mut cur = start;
    let mut left = [0u32; 3];
    let mut step = [0i32; 3];
    let mut t_max = [f64::INFINITY; 3];
    let mut t_delta = [f64::INFINITY; 3];
    for i in 0..3 {
        left[i] = end[i].abs_diff(start[i]);
        if left[i] == 0 {
            continue;
        }
        // The keys differ, so the coordinates do, in the same direction.
        let (o, d) = (f64::from(from[i]), f64::from(to[i]) - f64::from(from[i]));
        step[i] = if end[i] > start[i] { 1 } else { -1 };
        let boundary = f64::from(start[i] + i32::from(step[i] > 0)) * voxel;
        t_max[i] = (boundary - o) / d;
        t_delta[i] = voxel / d.abs();
    }
    let steps: u32 = left.iter().sum();
    for n in 0..steps {
        visit(cur, n + END_MARGIN >= steps);
        let axis = (0..3)
            .filter(|&i| left[i] > 0)
            .min_by(|&a, &b| t_max[a].total_cmp(&t_max[b]))
            .expect("steps left on some axis");
        cur[axis] += step[axis];
        t_max[axis] += t_delta[axis];
        left[axis] -= 1;
    }
    debug_assert_eq!(cur, end);
    visit(cur, true);
}

/// Principal axes of a covariance matrix with the spread along each, least spread first.
pub(crate) fn spread_axes(cov: &Matrix3<f64>) -> [(f64, Vector3<f64>); 3] {
    // nalgebra 0.33.3 can pair the eigenvalues of a nearly diagonal matrix with the wrong
    // eigenvectors (the vectors themselves are right); the spread along each vector is its
    // true eigenvalue.
    let axes = cov.symmetric_eigen().eigenvectors;
    let mut spread: [(f64, Vector3<f64>); 3] = std::array::from_fn(|i| {
        let v = axes.column(i).into_owned();
        (v.dot(&(cov * v)), v)
    });
    spread.sort_by(|a, b| a.0.total_cmp(&b.0));
    spread
}

/// Point moments of a voxel, relative to its lowest corner.
#[derive(Clone, Copy)]
struct Moments {
    n: u32,
    sum: Vector3<f64>,
    outer: Matrix3<f64>,
}

impl Moments {
    const ZERO: Moments = Moments {
        n: 0,
        sum: Vector3::new(0.0, 0.0, 0.0),
        outer: Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    };

    /// Adds `b`, whose points are relative to a corner `off` away from this one.
    fn add_shifted(&mut self, b: &Moments, off: &Vector3<f64>) {
        let n = f64::from(b.n);
        self.n += b.n;
        self.sum += b.sum + off * n;
        self.outer +=
            b.outer + b.sum * off.transpose() + off * b.sum.transpose() + off * off.transpose() * n;
    }

    /// Axis of least spread, if there are at least 3 points and they lie on a plane (least
    /// spread below a tenth of the middle one).
    fn plane_normal(&self) -> Option<Vector3<f64>> {
        if self.n < 3 {
            return None;
        }
        let c = f64::from(self.n);
        let mean = self.sum / c;
        let cov = self.outer / c - mean * mean.transpose();
        let spread = spread_axes(&cov);
        let (least, middle) = (spread[0].0, spread[1].0);
        (least < 0.1 * middle).then_some(spread[0].1)
    }
}

/// The voxels that hold cloud points.
struct Occupied {
    voxel: f64,
    /// Voxel → slot, numbered in point order.
    slot: HashMap<VoxelKey, u32, FixedState>,
    /// Slot of every point.
    point_slot: Vec<u32>,
    /// Surface normal per slot, from the points of the voxel and its 26 neighbours (so that
    /// a voxel holding only a few points of a surface along its face gets one too).
    normal: Vec<Option<Vector3<f64>>>,
}

impl Occupied {
    fn new(xyz: &[[f32; 3]], voxel: f64) -> Self {
        let mut slot: HashMap<VoxelKey, u32, FixedState> = HashMap::default();
        let mut keys = Vec::new();
        let point_slot: Vec<u32> = xyz
            .iter()
            .map(|p| {
                let k = key(p, voxel);
                *slot.entry(k).or_insert_with(|| {
                    keys.push(k);
                    u32::try_from(keys.len() - 1).expect("fewer than 2^32 voxels")
                })
            })
            .collect();
        let corner = |k: &VoxelKey| Vector3::from(k.map(|c| f64::from(c) * voxel));
        let mut own = vec![Moments::ZERO; keys.len()];
        for (p, &s) in xyz.iter().zip(&point_slot) {
            let m = &mut own[s as usize];
            let d = vec3(p) - corner(&keys[s as usize]);
            m.n += 1;
            m.sum += d;
            m.outer += d * d.transpose();
        }
        let normal = keys
            .par_iter()
            .map(|k| {
                let mut all = Moments::ZERO;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            if let Some(&o) = slot.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                                let off = corner(&[dx, dy, dz]);
                                all.add_shifted(&own[o as usize], &off);
                            }
                        }
                    }
                }
                all.plane_normal()
            })
            .collect();
        Occupied {
            voxel,
            slot,
            point_slot,
            normal,
        }
    }
}

/// Pass and hit counters per occupied voxel.
struct Counts {
    passes: Vec<AtomicU32>,
    hits: Vec<AtomicU32>,
}

/// Counts one ray from `sensor` to `end` into `counts`. Integer additions commute, so the
/// counts do not depend on the order the rays run in.
fn count_ray(sensor: &[f32; 3], end: &[f32; 3], grid: &Occupied, counts: &Counts) {
    let dir = vec3(end) - vec3(sensor);
    let min_dot = MIN_INCIDENCE_SIN * dir.norm();
    walk_ray(sensor, end, grid.voxel, |k, at_end| {
        let Some(&s) = grid.slot.get(&k) else {
            return;
        };
        let s = s as usize;
        if at_end {
            counts.hits[s].fetch_add(1, Ordering::Relaxed);
        } else if grid.normal[s].is_none_or(|n| dir.dot(&n).abs() >= min_dot) {
            counts.passes[s].fetch_add(1, Ordering::Relaxed);
        }
    });
}

/// Ghost flags, one per point: the point's `ghost_voxel` is passed by at least `min_passes`
/// rays and by at least `ghost_ratio` times as many rays as hit it. A ray hits its end voxel
/// and the [`END_MARGIN`] voxels before it, and passes the others unless it crosses their
/// surface flatter than [`MIN_INCIDENCE_SIN`] (see the module documentation).
///
/// # Panics
/// If `cfg` is invalid (see [`clean`]) or `rays` is inconsistent (`frame_start` must have one
/// entry more than `sensor`, rise, and end at `ends.len()`).
pub fn find_ghosts(cloud: &Cloud, rays: &RaySource, cfg: &CleanConfig) -> Vec<bool> {
    cfg.check().expect("valid CleanConfig");
    let fs = &rays.frame_start;
    assert_eq!(
        fs.len(),
        rays.sensor.len() + 1,
        "one frame start per frame, plus the end"
    );
    assert!(fs.windows(2).all(|w| w[0] <= w[1]), "frame starts rise");
    assert_eq!(
        fs.last().copied().unwrap_or(0) as usize,
        rays.ends.len(),
        "last start = rays"
    );

    let grid = Occupied::new(&cloud.xyz, cfg.ghost_voxel);
    let counter = || (0..grid.normal.len()).map(|_| AtomicU32::new(0)).collect();
    let counts = Counts {
        passes: counter(),
        hits: counter(),
    };
    (0..rays.sensor.len()).into_par_iter().for_each(|f| {
        let sensor = &rays.sensor[f];
        for end in &rays.ends[fs[f] as usize..fs[f + 1] as usize] {
            count_ray(sensor, end, &grid, &counts);
        }
    });

    let free: Vec<bool> = counts
        .passes
        .iter()
        .zip(&counts.hits)
        .map(|(p, h)| {
            let (p, h) = (p.load(Ordering::Relaxed), h.load(Ordering::Relaxed));
            p >= cfg.min_passes && f64::from(p) >= cfg.ghost_ratio * f64::from(h)
        })
        .collect();
    grid.point_slot.iter().map(|&s| free[s as usize]).collect()
}

/// Outlier flags for `xyz` (see [`find_outliers`]).
fn outlier_flags(xyz: &[[f32; 3]], k: usize, sigma: f64) -> Vec<bool> {
    // Fewer points than neighbours: no statistics to compare against.
    if xyz.len() <= k {
        return vec![false; xyz.len()];
    }
    let points: Vec<[f64; 3]> = xyz.iter().map(|p| p.map(f64::from)).collect();
    let tree: ImmutableKdTree<f64, 3> =
        ImmutableKdTree::new_from_slice(&points).expect("fewer than 2^32 points");
    // The point itself comes back with distance 0 (or a duplicate of it), hence k + 1.
    let n = NonZeroUsize::new(k + 1).expect("k + 1 > 0");
    let mean_dist: Vec<f64> = points
        .par_iter()
        .map(|p| {
            let nn = tree
                .query(p)
                .nearest_n::<SquaredEuclidean<f64>>(n)
                .execute();
            nn.iter().map(|x| x.distance.sqrt()).sum::<f64>() / k as f64
        })
        .collect();
    let count = mean_dist.len() as f64;
    let mean = mean_dist.iter().sum::<f64>() / count;
    let var = mean_dist.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / count;
    let limit = mean + sigma * var.sqrt();
    mean_dist.iter().map(|&d| d > limit).collect()
}

/// Outlier flags, one per point: the mean distance to the `outlier_k` nearest other points
/// lies more than `outlier_sigma` standard deviations above its mean over the cloud. A cloud
/// of at most `outlier_k` points has no outliers.
///
/// # Panics
/// If `cfg` is invalid (see [`clean`]).
pub fn find_outliers(cloud: &Cloud, cfg: &CleanConfig) -> Vec<bool> {
    cfg.check().expect("valid CleanConfig");
    outlier_flags(&cloud.xyz, cfg.outlier_k, cfg.outlier_sigma)
}

/// Splits `cloud` into (kept, removed). With [`CleanConfig::ghosts`] the ghosts go (see
/// [`find_ghosts`], rays from [`crate::build_dense`]), then with [`CleanConfig::outliers`]
/// the outliers among the remaining points (see [`find_outliers`]): isolated scraps a ghost
/// left behind count as outliers. Both parts keep the point order and all columns.
///
/// Fails if `cfg` is invalid.
///
/// # Panics
/// If `rays` is inconsistent (see [`find_ghosts`]).
pub fn clean(
    cloud: Cloud,
    rays: &RaySource,
    cfg: &CleanConfig,
) -> Result<(Cloud, Cloud, CleanStats), MapError> {
    cfg.check()?;
    let mut keep = if cfg.ghosts {
        find_ghosts(&cloud, rays, cfg).iter().map(|&g| !g).collect()
    } else {
        vec![true; cloud.len()]
    };
    let ghosts = keep.iter().filter(|&&k| !k).count();
    let mut outliers = 0;
    if cfg.outliers {
        let rest: Vec<usize> = (0..cloud.len()).filter(|&i| keep[i]).collect();
        let xyz: Vec<[f32; 3]> = rest.iter().map(|&i| cloud.xyz[i]).collect();
        for (&i, out) in rest
            .iter()
            .zip(outlier_flags(&xyz, cfg.outlier_k, cfg.outlier_sigma))
        {
            if out {
                keep[i] = false;
                outliers += 1;
            }
        }
    }
    let (kept, removed) = cloud.select(&keep);
    let stats = CleanStats {
        ghosts,
        outliers,
        kept: kept.len(),
    };
    Ok((kept, removed, stats))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Voxels the ray passes and the voxels it hits.
    fn walk(from: [f32; 3], to: [f32; 3]) -> (Vec<VoxelKey>, Vec<VoxelKey>) {
        let (mut passed, mut hit) = (Vec::new(), Vec::new());
        walk_ray(&from, &to, 1.0, |k, at_end| {
            if at_end {
                hit.push(k);
            } else {
                passed.push(k);
            }
        });
        (passed, hit)
    }

    #[test]
    fn walk_along_axis_hits_end_and_margin() {
        let (passed, hit) = walk([0.5, 0.5, 0.5], [6.5, 0.5, 0.5]);
        assert_eq!(passed, vec![[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]]);
        assert_eq!(hit, vec![[4, 0, 0], [5, 0, 0], [6, 0, 0]]);
        let (passed, hit) = walk([0.5, 0.5, 0.5], [-2.5, 0.5, 0.5]);
        assert_eq!(passed, vec![[0, 0, 0]]);
        assert_eq!(hit, vec![[-1, 0, 0], [-2, 0, 0], [-3, 0, 0]]);
    }

    #[test]
    fn walk_diagonal_visits_face_neighbours() {
        let (from, to) = ([0.2, 0.1, 0.3], [4.9, 3.7, -2.4]);
        let (passed, hit) = walk(from, to);
        assert_eq!(passed.len(), 4 + 3 + 3 - 2);
        assert_eq!(passed[0], [0, 0, 0]);
        assert_eq!(hit.len(), 3);
        assert_eq!(hit.last(), Some(&[4, 3, -3]));
        let all: Vec<VoxelKey> = passed.iter().chain(&hit).copied().collect();
        // Every step moves one voxel along one axis, and every voxel is really crossed by
        // the segment (slab test with a little slack).
        for w in all.windows(2) {
            let d: i32 = (0..3).map(|i| (w[1][i] - w[0][i]).abs()).sum();
            assert_eq!(d, 1, "{w:?}");
        }
        let (o, e) = (vec3(&from), vec3(&to));
        for k in &all {
            let (mut t0, mut t1) = (0.0f64, 1.0f64);
            for i in 0..3 {
                let d = e[i] - o[i];
                let (a, b) = (
                    (f64::from(k[i]) - o[i]) / d,
                    (f64::from(k[i] + 1) - o[i]) / d,
                );
                t0 = t0.max(a.min(b));
                t1 = t1.min(a.max(b));
            }
            assert!(t0 <= t1 + 1e-6, "{k:?} not on the ray");
        }
    }

    #[test]
    fn walk_inside_one_voxel() {
        let (passed, hit) = walk([0.1, 0.1, 0.1], [0.9, 0.9, 0.9]);
        assert!(passed.is_empty());
        assert_eq!(hit, vec![[0, 0, 0]]);
    }

    /// Floor of points at z = 0.05 (one voxel layer), 2 cm apart.
    fn floor() -> Occupied {
        let mut xyz = Vec::new();
        for i in 0..100 {
            for j in 0..20 {
                xyz.push([0.02 * i as f32 + 0.001, 0.02 * j as f32 + 0.001, 0.05]);
            }
        }
        Occupied::new(&xyz, 0.1)
    }

    fn counts(grid: &Occupied, sensor: [f32; 3], end: [f32; 3]) -> (u32, u32) {
        let counter = || (0..grid.normal.len()).map(|_| AtomicU32::new(0)).collect();
        let c = Counts {
            passes: counter(),
            hits: counter(),
        };
        count_ray(&sensor, &end, grid, &c);
        let sum = |v: &[AtomicU32]| v.iter().map(|x| x.load(Ordering::Relaxed)).sum();
        (sum(&c.passes), sum(&c.hits))
    }

    #[test]
    fn floor_normals_point_up() {
        let grid = floor();
        assert_eq!(grid.normal.len(), 20 * 4);
        for n in &grid.normal {
            let n = n.expect("floor voxels are planar");
            assert!(n.z.abs() > 0.999, "{n:?}");
        }
    }

    #[test]
    fn grazing_ray_does_not_pass_floor() {
        let grid = floor();
        // 5° to the floor, runs through 15 floor voxels in its layer: no passes, 3 hits.
        let (passes, hits) = counts(&grid, [0.05, 0.05, 0.0999], [1.95, 0.05, 0.0001]);
        assert_eq!((passes, hits), (0, 3));
        // Steep ray through the floor layer to below it: passes the voxels it crosses.
        let (passes, hits) = counts(&grid, [0.05, 0.15, 1.0], [0.95, 0.15, -1.0]);
        assert_eq!(hits, 0);
        assert!(passes >= 1, "{passes}");
    }

    #[test]
    fn moments_shift_matches_direct_sum() {
        let pts = [
            [0.31f32, 0.12, 0.05],
            [0.44, 0.19, 0.27],
            [0.18, 0.02, 0.11],
        ];
        let mut direct = Moments::ZERO;
        let mut shifted = Moments::ZERO;
        let off = Vector3::new(0.1, -0.2, 0.3);
        for p in &pts {
            let d = vec3(p);
            direct.n += 1;
            direct.sum += d;
            direct.outer += d * d.transpose();
        }
        let mut local = Moments::ZERO;
        for p in &pts {
            let d = vec3(p) - off;
            local.n += 1;
            local.sum += d;
            local.outer += d * d.transpose();
        }
        shifted.add_shifted(&local, &off);
        assert_eq!(shifted.n, direct.n);
        assert!((shifted.sum - direct.sum).norm() < 1e-12);
        assert!((shifted.outer - direct.outer).norm() < 1e-12);
    }

    #[test]
    fn outliers_small_cloud_none() {
        assert_eq!(outlier_flags(&[[0.0; 3]; 3], 16, 2.5), vec![false; 3]);
    }
}
