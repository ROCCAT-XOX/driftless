//! Trajectories: TUM files, rigid alignment and the yardstick metrics.

mod metrics;
mod tum;

pub use metrics::{align_umeyama, ate_rmse, match_count, path_length, start_end_gap};
pub use nalgebra;
pub use tum::{read_tum, write_tum};

use nalgebra::{UnitQuaternion, Vector3};

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
