//! Thinner walls: every point of a planar neighbourhood is projected onto the local plane.
//!
//! The LiDAR measures a wall as a band of 3–5 cm. Averaging inside the 1 cm voxels of the
//! dense cloud does not make it thinner, the voxels are smaller than the band. Instead each
//! point takes its neighbours within [`SmoothConfig::radius`] (at most
//! [`SmoothConfig::max_neighbours`], evenly spaced in distance rank: the nearest ones of a
//! dense cloud would span a ball no wider than the band), fits a plane through their centroid
//! (least spread axis of their covariance) and, if they lie on a plane (least spread below
//! [`SmoothConfig::planarity`] times the middle one), moves onto it, by at most
//! [`SmoothConfig::max_shift`]. Corners, edges and thin rods have no planar neighbourhood
//! and stay where they are.
//!
//! Every point is moved from the input coordinates, never from already moved ones, and
//! neighbours at the same distance are ordered by their coordinates, so the result does not
//! depend on the order the points run in; the step is a pure function per point.

use crate::clean::{block_moments, spread_axes, vec3, Moments};
use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::Vector3;
use rayon::prelude::*;
use std::collections::HashMap;

/// Refits of the local plane to its inliers (see [`project`]).
const REFITS: usize = 10;

/// A neighbour is an inlier of the local plane within this many times the RMS distance of
/// the neighbours to it.
const INLIER_SIGMAS: f64 = 3.0;

/// Smallest inlier band of the local plane (m), so that a plane without noise keeps its
/// points despite rounding.
const MIN_BAND: f64 = 0.002;

/// Edge of the cells the wall thickness is measured in (m).
const THICKNESS_CELL: f64 = 0.1;

/// Points a cell needs for its thickness to count.
const THICKNESS_MIN_POINTS: u32 = 30;

/// A cell counts as planar for the thickness if the least spread of its points and those of
/// its 26 neighbours is below this share of the middle one (as for the surface normals of
/// [`crate::clean`]). The neighbours are taken in because a cell face cutting through the
/// noise band would halve the thickness measured in each cell.
const THICKNESS_CELL_PLANARITY: f64 = 0.1;

#[derive(Debug, Clone, PartialEq)]
pub struct SmoothConfig {
    /// Project points onto their local plane in [`crate::clean`].
    pub enabled: bool,
    /// Neighbours are the points within this distance (m) ...
    pub radius: f64,
    /// ... at most this many, evenly spaced in distance rank ...
    pub max_neighbours: usize,
    /// ... and at least this many (the point itself included), else the point stays.
    pub min_neighbours: usize,
    /// The neighbours lie on a plane if their least spread is below this share of the
    /// middle one.
    pub planarity: f64,
    /// Largest shift of a point (m).
    pub max_shift: f64,
}

impl Default for SmoothConfig {
    fn default() -> Self {
        SmoothConfig {
            enabled: true,
            radius: 0.10,
            max_neighbours: 48,
            min_neighbours: 10,
            planarity: 0.05,
            max_shift: 0.03,
        }
    }
}

impl SmoothConfig {
    pub(crate) fn check(&self) -> Result<(), String> {
        if !(self.radius.is_finite() && self.radius > 0.0) {
            return Err("Radius der Wandglättung muss größer als 0 sein".into());
        }
        if self.min_neighbours < 3 {
            return Err("Wandglättung braucht mindestens 3 Nachbarn".into());
        }
        if self.max_neighbours < self.min_neighbours {
            return Err(
                "Höchstzahl der Nachbarn der Wandglättung liegt unter der Mindestzahl".into(),
            );
        }
        if !(self.planarity.is_finite() && self.planarity >= 0.0) {
            return Err("Ebenheitsschwelle der Wandglättung darf nicht negativ sein".into());
        }
        if !(self.max_shift.is_finite() && self.max_shift >= 0.0) {
            return Err("Größte Verschiebung der Wandglättung darf nicht negativ sein".into());
        }
        Ok(())
    }
}

/// What [`smooth_surfaces`] did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SmoothStats {
    /// Points that moved.
    pub moved: usize,
    /// Mean shift of the moved points (m).
    pub mean_shift: f64,
    /// Wall thickness before and after (m): median over the planar 10 cm cells with at
    /// least 30 points of the RMS distance to the cell's plane, of the points of the cell
    /// and its 26 neighbours. The cells and their points are taken from the input.
    pub thickness_before: f64,
    pub thickness_after: f64,
}

/// Plane through the mean of `pts` along their least spread: (mean, normal, least spread,
/// middle spread).
fn fit(pts: impl Iterator<Item = Vector3<f64>>) -> (Vector3<f64>, Vector3<f64>, f64, f64) {
    let mut m = Moments::ZERO;
    for d in pts {
        m.add(&d);
    }
    let (mean, cov) = m.mean_cov();
    let spread = spread_axes(&cov);
    (mean, spread[0].1, spread[0].0, spread[1].0)
}

/// New position of point `p` given its neighbours `rel` (relative to `p`, `p` itself
/// included, nearest first; the first `inner` within half the radius): the projection onto
/// their plane, capped at `max_shift`, or `None` if they do not lie on a plane.
///
/// Near an edge a few points of the other surface pass the planarity test and tilt the plane
/// towards them, and a tilted plane keeps them as inliers. The plane the point moves onto is
/// therefore started from the neighbours within half the radius, which lie on the point's
/// own surface, if there are `min_neighbours` of them and they are planar themselves (on a
/// noisy wall they are too few for a good normal; then from all), and fitted again, up to
/// [`REFITS`] times, to the neighbours within [`INLIER_SIGMAS`] times the RMS distance of
/// the last fit (at least [`MIN_BAND`], at most `max_shift`). On a plain wall that comes
/// back to all neighbours.
fn project(rel: &[Vector3<f64>], inner: usize, cfg: &SmoothConfig) -> Option<Vector3<f64>> {
    // Written so that NaN spreads (degenerate neighbourhoods) count as not planar.
    let planar = |least: f64, middle: f64| least < cfg.planarity * middle;
    let mut plane = fit(rel.iter().copied());
    if !planar(plane.2, plane.3) {
        return None;
    }
    if inner >= cfg.min_neighbours {
        let near = fit(rel[..inner].iter().copied());
        if planar(near.2, near.3) {
            plane = near;
        }
    }
    let (mut mean, mut n, mut least, _) = plane;
    let mut used = 0;
    for _ in 0..REFITS {
        let band = (INLIER_SIGMAS * least.max(0.0).sqrt())
            .max(MIN_BAND)
            .min(cfg.max_shift);
        let (m0, n0) = (mean, n);
        let near = |d: &&Vector3<f64>| (*d - m0).dot(&n0).abs() <= band;
        let inliers = rel.iter().filter(near).count();
        if inliers < cfg.min_neighbours {
            return None;
        }
        if inliers == used {
            break;
        }
        (mean, n, least, _) = fit(rel.iter().filter(near).copied());
        used = inliers;
    }
    // `p` is the origin: the shift onto the plane through `mean` is along `n`.
    let shift = n * mean.dot(&n);
    let len = shift.norm();
    Some(if len > cfg.max_shift {
        shift * (cfg.max_shift / len)
    } else {
        shift
    })
}

/// Projects every point of a planar neighbourhood onto its local plane (see the module
/// documentation); the others stay. Returns the new coordinates, one per point in the same
/// order, and what was done. With [`SmoothConfig::enabled`] off the points come back
/// unchanged and the stats are zero.
///
/// # Panics
/// If `cfg` is invalid (see [`crate::clean`]) or there are 2^32 points or more.
pub fn smooth_surfaces(xyz: &[[f32; 3]], cfg: &SmoothConfig) -> (Vec<[f32; 3]>, SmoothStats) {
    cfg.check().expect("valid SmoothConfig");
    if !cfg.enabled || xyz.len() < cfg.min_neighbours {
        return (xyz.to_vec(), SmoothStats::default());
    }
    let tree: ImmutableKdTree<f32, 3> =
        ImmutableKdTree::new_from_slice(xyz).expect("fewer than 2^32 points");
    let r2 = (cfg.radius * cfg.radius) as f32;
    let half = (0.25 * cfg.radius * cfg.radius) as f32;
    let shifts: Vec<Option<[f32; 3]>> = xyz
        .par_iter()
        .map(|p| {
            let mut nn = tree.query(p).within::<SquaredEuclidean<f32>>(r2).execute();
            // Ties in distance are broken by the coordinates, not by the order of the
            // kd-tree, which depends on the input order.
            nn.sort_by(|a, b| {
                a.distance.total_cmp(&b.distance).then_with(|| {
                    let bits = |i: u32| xyz[i as usize].map(f32::to_bits);
                    bits(a.item).cmp(&bits(b.item))
                })
            });
            let found = nn.len();
            if found < cfg.min_neighbours {
                return None;
            }
            let origin = vec3(p);
            // Evenly spaced in distance rank, so that the neighbourhood spans the whole
            // radius.
            let take = found.min(cfg.max_neighbours);
            let picked = (0..take).map(|i| &nn[i * found / take]);
            let rel: Vec<Vector3<f64>> = picked
                .clone()
                .map(|x| vec3(&xyz[x.item as usize]) - origin)
                .collect();
            let inner = picked.take_while(|x| x.distance <= half).count();
            let q = origin + project(&rel, inner, cfg)?;
            let q = [q.x as f32, q.y as f32, q.z as f32];
            (q != *p).then_some(q)
        })
        .collect();
    drop(tree);

    let mut moved = 0;
    let mut total = 0.0;
    let out: Vec<[f32; 3]> = xyz
        .iter()
        .zip(&shifts)
        .map(|(p, s)| match s {
            Some(q) => {
                moved += 1;
                total += (vec3(q) - vec3(p)).norm();
                *q
            }
            None => *p,
        })
        .collect();
    drop(shifts);
    let (thickness_before, thickness_after) = thickness(xyz, &out);
    let stats = SmoothStats {
        moved,
        mean_shift: if moved > 0 { total / moved as f64 } else { 0.0 },
        thickness_before,
        thickness_after,
    };
    (out, stats)
}

/// Median wall thickness of `before` and of `after` (same points, moved) over the cells of
/// `before` (see [`SmoothStats`]); 0 if there is no such cell.
fn thickness(before: &[[f32; 3]], after: &[[f32; 3]]) -> (f64, f64) {
    let mut slot: HashMap<VoxelKey, u32, FixedState> = HashMap::default();
    let mut keys: Vec<VoxelKey> = Vec::new();
    let point_slot: Vec<u32> = before
        .iter()
        .map(|p| {
            let k = voxel_key(&vec3(p), THICKNESS_CELL);
            *slot.entry(k).or_insert_with(|| {
                keys.push(k);
                u32::try_from(keys.len() - 1).expect("fewer than 2^32 cells")
            })
        })
        .collect();
    let corner = |k: &VoxelKey| Vector3::from(k.map(|c| f64::from(c) * THICKNESS_CELL));
    let moments = |pts: &[[f32; 3]]| {
        let mut m = vec![Moments::ZERO; keys.len()];
        for (p, &s) in pts.iter().zip(&point_slot) {
            m[s as usize].add(&(vec3(p) - corner(&keys[s as usize])));
        }
        m
    };
    let (own_b, own_a) = (moments(before), moments(after));
    drop(point_slot);
    // Moments of a cell and its 26 neighbours, relative to the cell's corner.
    let block = |own: &[Moments], k: &VoxelKey| block_moments(&slot, own, k, THICKNESS_CELL);
    let (tb, ta): (Vec<f64>, Vec<f64>) = keys
        .par_iter()
        .enumerate()
        .filter(|&(s, _)| own_b[s].n >= THICKNESS_MIN_POINTS)
        .filter_map(|(_, k)| {
            let b = spread_axes(&block(&own_b, k).mean_cov().1);
            (b[0].0 < THICKNESS_CELL_PLANARITY * b[1].0).then(|| {
                let a = spread_axes(&block(&own_a, k).mean_cov().1);
                // RMS distance to the best plane = square root of the least spread.
                (b[0].0.max(0.0).sqrt(), a[0].0.max(0.0).sqrt())
            })
        })
        .unzip();
    let (mut tb, mut ta) = (tb, ta);
    (median(&mut tb), median(&mut ta))
}

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_unstable_by(f64::total_cmp);
    let m = v.len() / 2;
    if v.len() % 2 == 1 {
        v[m]
    } else {
        0.5 * (v[m - 1] + v[m])
    }
}
