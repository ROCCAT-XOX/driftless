//! `driftless compare <a.tum> <b.tum> [--max-dt S] [--json]`: yardstick between two trajectories.

use driftless_traj::{ate_rmse, path_length, read_tum, start_end_gap};
use serde::Serialize;
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Trajektorie a (TUM-Format), wird an b ausgerichtet
    a: PathBuf,
    /// Trajektorie b (TUM-Format), Referenz
    b: PathBuf,
    /// Größter Zeitabstand (Sekunden) für die Zuordnung von Posen
    #[arg(long, default_value_t = 0.01)]
    max_dt: f64,
    /// Ausgabe als JSON
    #[arg(long)]
    json: bool,
}

#[derive(Serialize, Debug)]
pub struct Pair {
    pub a: f64,
    pub b: f64,
}

#[derive(Serialize, Debug)]
pub struct Report {
    pub ate_rmse_m: f64,
    pub matched_poses: usize,
    pub poses: Pair,
    pub start_end_gap_m: Pair,
    pub path_length_m: Pair,
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if !(args.max_dt.is_finite() && args.max_dt >= 0.0) {
        return Err("--max-dt muss eine Zahl ≥ 0 sein".into());
    }
    let a = read_tum(&args.a)?;
    let b = read_tum(&args.b)?;
    let ate = ate_rmse(&a, &b, args.max_dt)?;
    let matched = count_matches(&a, &b, args.max_dt);
    let r = Report {
        ate_rmse_m: ate,
        matched_poses: matched,
        poses: Pair {
            a: a.len() as f64,
            b: b.len() as f64,
        },
        start_end_gap_m: Pair {
            a: start_end_gap(&a),
            b: start_end_gap(&b),
        },
        path_length_m: Pair {
            a: path_length(&a),
            b: path_length(&b),
        },
    };
    if args.json {
        println!("{}", serde_json::to_string_pretty(&r)?);
    } else {
        println!("Abweichung (ATE, RMSE): {:.3} m", r.ate_rmse_m);
        println!(
            "Start↔Ende a/b: {:.3} m / {:.3} m",
            r.start_end_gap_m.a, r.start_end_gap_m.b
        );
        println!(
            "Länge a/b: {:.3} m / {:.3} m",
            r.path_length_m.a, r.path_length_m.b
        );
        println!("Zugeordnete Posen: {}", r.matched_poses);
    }
    Ok(())
}

/// Number of `a` poses that have a `b` pose within `max_dt` (same rule as the metric).
fn count_matches(a: &[driftless_traj::Pose], b: &[driftless_traj::Pose], max_dt: f64) -> usize {
    let mut bt: Vec<f64> = b.iter().map(|p| p.t).collect();
    bt.sort_by(f64::total_cmp);
    a.iter()
        .filter(|e| {
            let i = bt.partition_point(|&t| t < e.t);
            let lo = i.checked_sub(1).map(|j| (bt[j] - e.t).abs());
            let hi = bt.get(i).map(|t| (t - e.t).abs());
            lo.into_iter().chain(hi).fold(f64::INFINITY, f64::min) <= max_dt
        })
        .count()
}
