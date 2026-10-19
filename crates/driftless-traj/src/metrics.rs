//! Rigid alignment (Umeyama/Kabsch, no scale) and trajectory metrics.

use crate::{Pose, TrajError};
use nalgebra::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3};
use std::borrow::Cow;

const MIN_MATCHES: usize = 3;

/// Matched `(est, gt)` positions.
type Pairs = Vec<(Vector3<f64>, Vector3<f64>)>;

fn sorted(poses: &[Pose]) -> Cow<'_, [Pose]> {
    if poses.windows(2).all(|w| w[0].t <= w[1].t) {
        Cow::Borrowed(poses)
    } else {
        let mut v = poses.to_vec();
        v.sort_by(|a, b| a.t.total_cmp(&b.t));
        Cow::Owned(v)
    }
}

/// For each `est` pose the nearest-in-time `gt` pose within `max_dt`.
/// Returns matched position pairs `(est, gt)`.
fn match_positions(est: &[Pose], gt: &[Pose], max_dt: f64) -> Result<Pairs, TrajError> {
    let (est, gt) = (sorted(est), sorted(gt));
    let mut pairs = Vec::new();
    for e in est.iter() {
        let i = gt.partition_point(|g| g.t < e.t);
        let best = [i.checked_sub(1), (i < gt.len()).then_some(i)]
            .into_iter()
            .flatten()
            .min_by(|&a, &b| (gt[a].t - e.t).abs().total_cmp(&(gt[b].t - e.t).abs()));
        if let Some(j) = best {
            if (gt[j].t - e.t).abs() <= max_dt {
                pairs.push((e.p, gt[j].p));
            }
        }
    }
    if pairs.len() < MIN_MATCHES {
        return Err(TrajError::TooFewMatches {
            found: pairs.len(),
            needed: MIN_MATCHES,
            max_dt,
        });
    }
    Ok(pairs)
}

fn align_pairs(pairs: &[(Vector3<f64>, Vector3<f64>)]) -> Isometry3<f64> {
    let n = pairs.len() as f64;
    let me = pairs.iter().map(|p| p.0).sum::<Vector3<f64>>() / n;
    let mg = pairs.iter().map(|p| p.1).sum::<Vector3<f64>>() / n;
    let cov: Matrix3<f64> = pairs
        .iter()
        .map(|(e, g)| (e - me) * (g - mg).transpose())
        .sum();
    let svd = cov.svd(true, true);
    let (u, vt) = (svd.u.unwrap(), svd.v_t.unwrap());
    // R = V * diag(1, 1, det(V U^T)) * U^T: forces det(R) = +1 (no reflection).
    let d = if (vt.transpose() * u.transpose()).determinant() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let r = vt.transpose() * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * u.transpose();
    let rot = UnitQuaternion::from_matrix(&r);
    Isometry3::from_parts(Translation3::from(mg - rot * me), rot)
}

/// Rigid transform that maps `est` onto `gt` (least squares over positions
/// matched by nearest timestamp within `max_dt`). Inputs may differ in rate and length.
pub fn align_umeyama(est: &[Pose], gt: &[Pose], max_dt: f64) -> Result<Isometry3<f64>, TrajError> {
    Ok(align_pairs(&match_positions(est, gt, max_dt)?))
}

/// Absolute trajectory error (RMSE, metres) of `est` against `gt` after alignment.
pub fn ate_rmse(est: &[Pose], gt: &[Pose], max_dt: f64) -> Result<f64, TrajError> {
    let pairs = match_positions(est, gt, max_dt)?;
    let t = align_pairs(&pairs);
    let sum: f64 = pairs
        .iter()
        .map(|(e, g)| (t.transform_point(&Point3::from(*e)).coords - g).norm_squared())
        .sum();
    Ok((sum / pairs.len() as f64).sqrt())
}

/// Distance between first and last pose (metres); 0 for fewer than two poses.
pub fn start_end_gap(est: &[Pose]) -> f64 {
    match (est.first(), est.last()) {
        (Some(a), Some(b)) => (b.p - a.p).norm(),
        _ => 0.0,
    }
}

/// Summed length of the path through all poses in the given order (metres).
pub fn path_length(est: &[Pose]) -> f64 {
    est.windows(2).map(|w| (w[1].p - w[0].p).norm()).sum()
}
