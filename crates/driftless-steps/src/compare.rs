//! Yardstick between two trajectories: deviation after alignment, start↔end gap, length.

use crate::error::{failed, StepError};
use driftless_traj::{ate_rmse, match_count, path_length, start_end_gap, Pose};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Pair<T> {
    pub a: T,
    pub b: T,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Report {
    pub ate_rmse_m: f64,
    pub matched_poses: usize,
    pub poses: Pair<usize>,
    pub start_end_gap_m: Pair<f64>,
    pub path_length_m: Pair<f64>,
}

/// `max_dt` must be a number ≥ 0.
pub fn check_max_dt(max_dt: f64) -> Result<(), StepError> {
    if !(max_dt.is_finite() && max_dt >= 0.0) {
        return Err("--max-dt muss eine Zahl ≥ 0 sein".into());
    }
    Ok(())
}

/// Compares `a` (aligned to `b`) with the reference `b`; poses are matched up to `max_dt`
/// seconds apart.
pub fn compare(a: &[Pose], b: &[Pose], max_dt: f64) -> Result<Report, StepError> {
    check_max_dt(max_dt)?;
    let ate = ate_rmse(a, b, max_dt).map_err(failed)?;
    Ok(Report {
        ate_rmse_m: ate,
        matched_poses: match_count(a, b, max_dt),
        poses: Pair {
            a: a.len(),
            b: b.len(),
        },
        start_end_gap_m: Pair {
            a: start_end_gap(a),
            b: start_end_gap(b),
        },
        path_length_m: Pair {
            a: path_length(a),
            b: path_length(b),
        },
    })
}

/// The report as `driftless compare` prints it without `--json`.
pub fn render_text(r: &Report) -> String {
    format!(
        "Abweichung (ATE, RMSE): {:.3} m\nStart↔Ende a/b: {:.3} m / {:.3} m\nLänge a/b: {:.3} m / \
         {:.3} m\nZugeordnete Posen: {}\n",
        r.ate_rmse_m,
        r.start_end_gap_m.a,
        r.start_end_gap_m.b,
        r.path_length_m.a,
        r.path_length_m.b,
        r.matched_poses
    )
}
