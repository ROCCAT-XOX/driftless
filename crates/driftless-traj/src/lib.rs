//! Trajectories: TUM files, rigid alignment and the yardstick metrics.

mod metrics;
mod tum;

pub use metrics::{align_umeyama, ate_rmse, match_count, path_length, start_end_gap};
pub use nalgebra;
pub use tum::{read_tum, write_tum};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

/// One timestamped pose (seconds, metres).
#[derive(Debug, Clone, PartialEq)]
pub struct Pose {
    pub t: f64,
    pub p: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
}

#[derive(Debug, thiserror::Error)]
pub enum TrajError {
    #[error("{path}: {source}")]
    Io {
        path: String,
        source: std::io::Error,
    },
    #[error("Zeile {line}: {msg}")]
    Parse { line: usize, msg: String },
    #[error("zu wenige zugeordnete Posen: {found} (mindestens {needed} nötig, max. Zeitabstand {max_dt} s)")]
    TooFewMatches {
        found: usize,
        needed: usize,
        max_dt: f64,
    },
}

/// Pose at time `t` in a trajectory sorted by time: position linear, rotation slerp between
/// the two neighbouring poses. `None` before the first or after the last pose (and for an
/// empty trajectory or a non-finite `t`).
pub fn interpolate_pose(poses: &[Pose], t: f64) -> Option<Isometry3<f64>> {
    let (first, last) = (poses.first()?, poses.last()?);
    if !(t >= first.t && t <= last.t) {
        return None;
    }
    let iso = |p: &Pose| Isometry3::from_parts(Translation3::from(p.p), p.q);
    // First pose with time >= t; `t >= first.t` makes it at least 0.
    let i = poses.partition_point(|p| p.t < t);
    let b = &poses[i];
    if i == 0 || b.t == t {
        return Some(iso(b));
    }
    let a = &poses[i - 1];
    let s = (t - a.t) / (b.t - a.t);
    let p = a.p.lerp(&b.p, s);
    // `try_slerp` fails only for (nearly) opposite rotations; fall back to the nearer pose.
    let q =
        a.q.try_slerp(&b.q, s, 1e-12)
            .unwrap_or(if s < 0.5 { a.q } else { b.q });
    Some(Isometry3::from_parts(Translation3::from(p), q))
}
