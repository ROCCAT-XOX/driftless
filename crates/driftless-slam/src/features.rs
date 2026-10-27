//! Local shape features of a thinned point cloud for the coarse alignment of two clouds:
//! normals from the nearest neighbours and Fast Point Feature Histograms (FPFH, Rusu et
//! al. 2009, "Fast Point Feature Histograms (FPFH) for 3D Registration"), written from the
//! paper.
//!
//! The clouds carry no viewpoint, so the sign of a normal is arbitrary and differs between
//! two scans of the same wall. The three angles of a point pair are therefore folded so that
//! they do not change when either normal flips: `|cos φ|`, `|cos α|` and `θ ∈ [0, π/2]`.
//! Each of them goes into 11 classes; a histogram has 33 classes, each third summing to 100.
//!
//! Every per-point step runs in parallel and collects in point order, so the results do not
//! depend on the number of threads.

use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use std::num::NonZeroUsize;

/// Classes per angle.
pub const FPFH_BINS: usize = 11;
/// Classes of a histogram (three angles).
pub const FPFH_DIM: usize = 3 * FPFH_BINS;

/// One FPFH.
pub type Fpfh = [f32; FPFH_DIM];

/// Smallest number of neighbours (with a normal) within the feature radius for a histogram;
/// fewer give a histogram of a handful of pairs that matches anything.
pub const MIN_FEATURE_NEIGHBOURS: usize = 5;

/// Principal axes of a covariance matrix with the spread along each, least spread first.
pub(crate) fn spread_axes(cov: &Matrix3<f64>) -> [(f64, Vector3<f64>); 3] {
    // nalgebra 0.33.3 can pair the eigenvalues of a nearly diagonal matrix with the wrong
    // eigenvectors (the vectors themselves are right); the spread along each vector is its
    // true eigenvalue (same workaround as `driftless_map::clean::spread_axes`).
    let axes = cov.symmetric_eigen().eigenvectors;
    let mut spread: [(f64, Vector3<f64>); 3] = std::array::from_fn(|i| {
        let v = axes.column(i).into_owned();
        (v.dot(&(cov * v)), v)
    });
    spread.sort_by(|a, b| a.0.total_cmp(&b.0));
    spread
}

/// Mean and covariance of `pts` (at least one point).
pub(crate) fn mean_cov(pts: &[Vector3<f64>]) -> (Vector3<f64>, Matrix3<f64>) {
    let n = pts.len() as f64;
    let c = pts.iter().sum::<Vector3<f64>>() / n;
    let cov = pts.iter().fold(Matrix3::zeros(), |acc, x| {
        acc + (x - c) * (x - c).transpose()
    }) / n;
    (c, cov)
}

/// k-d tree over a point cloud (owns the points).
pub struct PointTree {
    tree: ImmutableKdTree<f64, 3>,
    points: Vec<Vector3<f64>>,
}

impl PointTree {
    /// `None` for an empty cloud.
    pub fn new(points: Vec<Vector3<f64>>) -> Option<Self> {
        if points.is_empty() {
            return None;
        }
        let flat: Vec<[f64; 3]> = points.iter().map(|p| [p.x, p.y, p.z]).collect();
        let tree = ImmutableKdTree::new_from_slice(&flat).ok()?;
        Some(PointTree { tree, points })
    }

    pub fn points(&self) -> &[Vector3<f64>] {
        &self.points
    }

    /// Indices and squared distances of the `k` nearest points to `q`, nearest first.
    pub fn nearest(&self, q: &Vector3<f64>, k: usize) -> Vec<(usize, f64)> {
        let k = NonZeroUsize::new(k.max(1)).unwrap_or(NonZeroUsize::MIN);
        self.tree
            .query(&[q.x, q.y, q.z])
            .nearest_n::<SquaredEuclidean<f64>>(k)
            .execute()
            .into_iter()
            .map(|n| (n.item as usize, n.distance))
            .collect()
    }

    /// Squared distance from `q` to the nearest point.
    pub fn nearest_d2(&self, q: &Vector3<f64>) -> f64 {
        self.tree
            .query(&[q.x, q.y, q.z])
            .nearest_one::<SquaredEuclidean<f64>>()
            .execute()
            .distance
    }

    /// Indices and squared distances of all points within `radius` of `q`, nearest first.
    pub fn within(&self, q: &Vector3<f64>, radius: f64) -> Vec<(usize, f64)> {
        self.tree
            .query(&[q.x, q.y, q.z])
            .within::<SquaredEuclidean<f64>>(radius * radius)
            .execute()
            .into_iter()
            .map(|n| (n.item as usize, n.distance))
            .collect()
    }
}

/// Shape of the neighbourhood of a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalShape {
    /// Axis of least spread, unit length, arbitrary sign.
    pub normal: Vector3<f64>,
    /// Surface variation: least spread over the sum of the three spreads (0 on a plane,
    /// about 0.05–0.3 on an edge or a corner, at most 1/3).
    pub variation: f64,
}

/// Shape of every point from its `k` nearest neighbours (itself included). `None` if the
/// neighbours lie on a line (the middle spread is below a hundredth of the largest) or
/// there are fewer than 3.
pub fn local_shapes(tree: &PointTree, k: usize) -> Vec<Option<LocalShape>> {
    tree.points
        .par_iter()
        .map(|p| {
            let nn = tree.nearest(p, k.max(3));
            if nn.len() < 3 {
                return None;
            }
            let pts: Vec<Vector3<f64>> = nn.iter().map(|&(i, _)| tree.points[i]).collect();
            let (_, cov) = mean_cov(&pts);
            let axes = spread_axes(&cov);
            if axes[1].0 <= 0.01 * axes[2].0 {
                return None;
            }
            let total = axes[0].0.max(0.0) + axes[1].0 + axes[2].0;
            Some(LocalShape {
                normal: axes[0].1.normalize(),
                variation: axes[0].0.max(0.0) / total,
            })
        })
        .collect()
}

/// Normals of [`local_shapes`].
pub fn normals(tree: &PointTree, k: usize) -> Vec<Option<Vector3<f64>>> {
    local_shapes(tree, k)
        .into_iter()
        .map(|s| s.map(|s| s.normal))
        .collect()
}

/// Corner strength of every point: the smallest eigenvalue of the mean of `n nᵀ` over the
/// normals `n` of its `k` nearest neighbours (itself included; the sign of a normal does
/// not matter). About 0 on a plane and on an edge (the normals take one or two
/// directions), up to 1/3 at a corner of three planes. 0 for points without a normal.
pub fn corner_strength(tree: &PointTree, normals: &[Option<Vector3<f64>>], k: usize) -> Vec<f64> {
    tree.points
        .par_iter()
        .enumerate()
        .map(|(i, p)| {
            if normals[i].is_none() {
                return 0.0;
            }
            let (mut m, mut n) = (Matrix3::zeros(), 0usize);
            for (j, _) in tree.nearest(p, k.max(3)) {
                if let Some(nj) = normals[j] {
                    m += nj * nj.transpose();
                    n += 1;
                }
            }
            if n == 0 {
                return 0.0;
            }
            spread_axes(&(m / n as f64))[0].0.max(0.0)
        })
        .collect()
}

/// Class of `x ∈ [0, 1]`.
fn bin(x: f64) -> usize {
    ((x * FPFH_BINS as f64) as usize).min(FPFH_BINS - 1)
}

/// Folded pair angles of the points `ps`, `pt` with normals `ns`, `nt` as classes of the
/// three thirds; `None` for coincident points or a normal along the line between them.
fn pair_classes(
    ps: &Vector3<f64>,
    ns: &Vector3<f64>,
    pt: &Vector3<f64>,
    nt: &Vector3<f64>,
) -> Option<[usize; 3]> {
    let d = pt - ps;
    let len = d.norm();
    if len < 1e-12 {
        return None;
    }
    let d = d / len;
    // The point whose normal is closer to the line is the source of the Darboux frame
    // (decided on the folded angles, so independent of the normal signs).
    let (u, n2) = if ns.dot(&d).abs() >= nt.dot(&d).abs() {
        (ns, nt)
    } else {
        (nt, ns)
    };
    let v = d.cross(u);
    let vn = v.norm();
    if vn < 1e-9 {
        return None;
    }
    let v = v / vn;
    let w = u.cross(&v);
    let phi = u.dot(&d).abs();
    let alpha = v.dot(n2).abs();
    let theta = w.dot(n2).abs().atan2(u.dot(n2).abs()) / std::f64::consts::FRAC_PI_2;
    Some([bin(phi), FPFH_BINS + bin(alpha), 2 * FPFH_BINS + bin(theta)])
}

/// Neighbours of point `i` within `radius` that have a normal, without `i` itself.
fn neighbours(
    tree: &PointTree,
    normals: &[Option<Vector3<f64>>],
    i: usize,
    radius: f64,
) -> Vec<(usize, f64)> {
    tree.within(&tree.points[i], radius)
        .into_iter()
        .filter(|&(j, _)| j != i && normals[j].is_some())
        .collect()
}

/// FPFH of every point over its neighbours within `radius` (Rusu et al. 2009, eq. 2, with
/// the weights `1 / d²` of the reference implementation and every third normalised to 100).
/// `None` for points without a normal or with fewer than [`MIN_FEATURE_NEIGHBOURS`]
/// neighbours.
pub fn fpfh(tree: &PointTree, normals: &[Option<Vector3<f64>>], radius: f64) -> Vec<Option<Fpfh>> {
    let pts = tree.points();
    // Simplified point feature histogram: the pairs of each point with its neighbours.
    let spfh: Vec<Option<Fpfh>> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let ni = normals[i]?;
            let nb = neighbours(tree, normals, i, radius);
            if nb.len() < MIN_FEATURE_NEIGHBOURS {
                return None;
            }
            let mut h = [0f32; FPFH_DIM];
            let mut used = 0usize;
            for &(j, _) in &nb {
                let Some(nj) = normals[j] else { continue };
                if let Some(c) = pair_classes(&pts[i], &ni, &pts[j], &nj) {
                    for k in c {
                        h[k] += 1.0;
                    }
                    used += 1;
                }
            }
            if used == 0 {
                return None;
            }
            let scale = 100.0 / used as f32;
            h.iter_mut().for_each(|x| *x *= scale);
            Some(h)
        })
        .collect();
    (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let own = spfh[i]?;
            let mut acc = [0f64; FPFH_DIM];
            let mut sum = [0f64; 3];
            for (j, d2) in neighbours(tree, normals, i, radius) {
                let Some(h) = spfh[j] else { continue };
                if d2 <= 0.0 {
                    continue;
                }
                for (k, x) in h.iter().enumerate() {
                    let val = f64::from(*x) / d2;
                    acc[k] += val;
                    sum[k / FPFH_BINS] += val;
                }
            }
            let mut out = own;
            for (k, o) in out.iter_mut().enumerate() {
                let s = sum[k / FPFH_BINS];
                if s > 0.0 {
                    *o += (acc[k] * 100.0 / s) as f32;
                }
            }
            Some(out)
        })
        .collect()
}

/// Valid features with their point indices.
fn valid(features: &[Option<Fpfh>]) -> (Vec<usize>, Vec<Fpfh>) {
    features
        .iter()
        .enumerate()
        .filter_map(|(i, f)| f.map(|f| (i, f)))
        .unzip()
}

/// Index of the nearest feature in `tree` for every feature in `queries`.
fn nearest_feature(tree: &ImmutableKdTree<f32, FPFH_DIM>, queries: &[Fpfh]) -> Vec<usize> {
    queries
        .par_iter()
        .map(|q| {
            tree.query(q)
                .nearest_one::<SquaredEuclidean<f32>>()
                .execute()
                .item as usize
        })
        .collect()
}

/// Pairs `(i, j)` of a source point `i` and a target point `j` whose features are each
/// other's nearest neighbour in feature space, in source order.
pub fn mutual_matches(source: &[Option<Fpfh>], target: &[Option<Fpfh>]) -> Vec<(usize, usize)> {
    let (src_idx, src_f) = valid(source);
    let (tgt_idx, tgt_f) = valid(target);
    if src_f.is_empty() || tgt_f.is_empty() {
        return Vec::new();
    }
    let (Ok(src_tree), Ok(tgt_tree)) = (
        ImmutableKdTree::<f32, FPFH_DIM>::new_from_slice(&src_f),
        ImmutableKdTree::<f32, FPFH_DIM>::new_from_slice(&tgt_f),
    ) else {
        return Vec::new();
    };
    let s_to_t = nearest_feature(&tgt_tree, &src_f);
    let t_to_s = nearest_feature(&src_tree, &tgt_f);
    s_to_t
        .iter()
        .enumerate()
        .filter(|&(a, &b)| t_to_s[b] == a)
        .map(|(a, &b)| (src_idx[a], tgt_idx[b]))
        .collect()
}
