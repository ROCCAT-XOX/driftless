//! Point-to-plane ICP against a k-d tree over the local map.
//!
//! Each iteration is split into a per-point step (nearest neighbours, local plane, residual
//! and Jacobian; independent per point, run in parallel) and a reduction into the 6x6 normal
//! equations, done sequentially in point order so results are bit-identical between runs.
//! The per-point step is the part meant to move to a GPU compute pass later.

use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::{
    Isometry3, Matrix3, Matrix6, Point3, Translation3, UnitQuaternion, Vector3, Vector6,
};
use rayon::prelude::*;
use std::num::NonZeroUsize;

#[derive(Debug, Clone, PartialEq)]
pub struct IcpParams {
    pub max_iter: u32,
    /// Correspondences with a larger point-to-plane residual are ignored (m).
    pub max_corr_dist: f64,
    /// Neighbours used for the local plane.
    pub knn: usize,
    /// Neighbours must lie within this distance of the fitted plane (m).
    pub plane_tolerance: f64,
    /// Neighbours must spread at least this much (std. dev., m) in the plane's second
    /// direction; collinear neighbourhoods (one scan line) give no usable normal.
    pub min_plane_spread: f64,
    /// Largest ratio of the smallest to the middle covariance eigenvalue of a plane.
    pub max_flatness: f64,
    /// Scale of the Cauchy robust kernel (m).
    pub kernel: f64,
    /// Weight of the motion prior (the `init` pose) per rad² and per m², relative to one
    /// point-to-plane term. Keeps directions the geometry does not constrain (long corridor,
    /// few planes) at the IMU prediction instead of letting them wander.
    pub prior_rot: f64,
    pub prior_trans: f64,
    /// Stop when the update is below these (rad, m).
    pub eps_rot: f64,
    pub eps_trans: f64,
}

impl Default for IcpParams {
    fn default() -> Self {
        IcpParams {
            max_iter: 30,
            max_corr_dist: 1.0,
            knn: 5,
            plane_tolerance: 0.1,
            min_plane_spread: 0.03,
            max_flatness: 0.02,
            kernel: 0.1,
            prior_rot: 10.0,
            prior_trans: 0.5,
            eps_rot: 5e-4,
            eps_trans: 1e-3,
        }
    }
}

/// Registration target: map points in a k-d tree.
pub struct IcpTarget {
    tree: ImmutableKdTree<f64, 3>,
    points: Vec<[f64; 3]>,
}

impl IcpTarget {
    /// `None` if there are fewer points than a plane needs.
    pub fn new(points: Vec<Vector3<f64>>) -> Option<Self> {
        if points.len() < 3 {
            return None;
        }
        let points: Vec<[f64; 3]> = points.iter().map(|p| [p.x, p.y, p.z]).collect();
        let tree = ImmutableKdTree::new_from_slice(&points).ok()?;
        Some(IcpTarget { tree, points })
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct IcpResult {
    pub pose: Isometry3<f64>,
    pub iterations: u32,
    pub converged: bool,
    /// Correspondences used in the last iteration.
    pub correspondences: usize,
}

/// Weighted residual row for one source point: Jacobian w.r.t. `[rotation, translation]`
/// (left perturbation in the world frame), residual and robust weight.
struct Term {
    j: Vector6<f64>,
    r: f64,
    w: f64,
}

fn point_term(
    q: &Vector3<f64>,
    target: &IcpTarget,
    p: &IcpParams,
    k: NonZeroUsize,
) -> Option<Term> {
    let nn = target
        .tree
        .query(&[q.x, q.y, q.z])
        .nearest_n::<SquaredEuclidean<f64>>(k)
        .execute();
    if nn.len() < 3 {
        return None;
    }
    let max_d2 = p.max_corr_dist * p.max_corr_dist;
    if nn.iter().any(|n| n.distance > max_d2) {
        return None;
    }
    let pts: Vec<Vector3<f64>> = nn
        .iter()
        .map(|n| Vector3::from(target.points[n.item as usize]))
        .collect();
    let c = pts.iter().sum::<Vector3<f64>>() / pts.len() as f64;
    let cov = pts.iter().fold(Matrix3::zeros(), |acc, x| {
        acc + (x - c) * (x - c).transpose()
    }) / pts.len() as f64;
    let eig = cov.symmetric_eigen();
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    // Neighbours along a single scan line fit any plane through that line: reject.
    if eig.eigenvalues[order[1]] < p.min_plane_spread * p.min_plane_spread {
        return None;
    }
    // Thickness against width: corners and edges are not planes.
    if eig.eigenvalues[order[0]] > p.max_flatness * eig.eigenvalues[order[1]] {
        return None;
    }
    let n = eig.eigenvectors.column(order[0]).into_owned();
    if pts
        .iter()
        .any(|x| n.dot(&(x - c)).abs() > p.plane_tolerance)
    {
        return None;
    }
    let r = n.dot(&(q - c));
    if r.abs() > p.max_corr_dist {
        return None;
    }
    let rot = q.cross(&n);
    let j = Vector6::new(rot.x, rot.y, rot.z, n.x, n.y, n.z);
    let w = 1.0 / (1.0 + (r / p.kernel).powi(2));
    Some(Term { j, r, w })
}

/// Offset of `pose` from `prior` as a left perturbation `[rotation, translation]`.
fn prior_offset(pose: &Isometry3<f64>, prior: &Isometry3<f64>) -> Vector6<f64> {
    let d = pose.rotation * prior.rotation.inverse();
    let r = d.scaled_axis();
    let t = pose.translation.vector - d * prior.translation.vector;
    Vector6::new(r.x, r.y, r.z, t.x, t.y, t.z)
}

/// Aligns `source` (body frame) to `target` (world frame) starting at `init`, which also
/// serves as the motion prior.
pub fn align(
    source: &[Vector3<f64>],
    target: &IcpTarget,
    init: &Isometry3<f64>,
    params: &IcpParams,
) -> IcpResult {
    let k = NonZeroUsize::new(params.knn.max(3)).unwrap_or(NonZeroUsize::MIN);
    let mut pose = *init;
    let mut result = IcpResult {
        pose,
        iterations: 0,
        converged: false,
        correspondences: 0,
    };
    for it in 1..=params.max_iter {
        let terms: Vec<Option<Term>> = source
            .par_iter()
            .map(|s| point_term(&(pose * Point3::from(*s)).coords, target, params, k))
            .collect();
        let (mut h, mut g, mut used) = (Matrix6::<f64>::zeros(), Vector6::<f64>::zeros(), 0);
        for t in terms.iter().flatten() {
            h += t.j * t.j.transpose() * t.w;
            g += t.j * (t.r * t.w);
            used += 1;
        }
        result.iterations = it;
        result.correspondences = used;
        if used < 6 {
            break;
        }
        let e = prior_offset(&pose, init);
        for i in 0..6 {
            let w = if i < 3 {
                params.prior_rot
            } else {
                params.prior_trans
            };
            h[(i, i)] += w;
            g[i] += w * e[i];
        }
        let Some(chol) = h.cholesky() else { break };
        let dx = -chol.solve(&g);
        let d_rot = Vector3::new(dx[0], dx[1], dx[2]);
        let d_trans = Vector3::new(dx[3], dx[4], dx[5]);
        let update = Isometry3::from_parts(
            Translation3::from(d_trans),
            UnitQuaternion::from_scaled_axis(d_rot),
        );
        pose = update * pose;
        result.pose = pose;
        if d_rot.norm() < params.eps_rot && d_trans.norm() < params.eps_trans {
            result.converged = true;
            break;
        }
    }
    result
}
