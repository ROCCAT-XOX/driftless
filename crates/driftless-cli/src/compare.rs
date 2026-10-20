//! `driftless compare <a.tum> <b.tum> [--max-dt S] [--json]`: yardstick between two trajectories.

use driftless_traj::{ate_rmse, match_count, path_length, read_tum, start_end_gap};
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
pub struct Pair<T> {
    pub a: T,
    pub b: T,
}

#[derive(Serialize, Debug)]
pub struct Report {
    pub ate_rmse_m: f64,
    pub matched_poses: usize,
    pub poses: Pair<usize>,
    pub start_end_gap_m: Pair<f64>,
    pub path_length_m: Pair<f64>,
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if !(args.max_dt.is_finite() && args.max_dt >= 0.0) {
        return Err("--max-dt muss eine Zahl ≥ 0 sein".into());
    }
    let a = read_tum(&args.a)?;
    let b = read_tum(&args.b)?;
    let ate = ate_rmse(&a, &b, args.max_dt)?;
    let matched = match_count(&a, &b, args.max_dt);
    let r = Report {
        ate_rmse_m: ate,
        matched_poses: matched,
        poses: Pair {
            a: a.len(),
            b: b.len(),
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
