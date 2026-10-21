//! `driftless compare <a.tum> <b.tum> [--max-dt S] [--json]`: yardstick between two trajectories.

use driftless_steps::compare::{check_max_dt, compare, render_text};
use driftless_traj::read_tum;
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

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    check_max_dt(args.max_dt)?;
    let a = read_tum(&args.a)?;
    let b = read_tum(&args.b)?;
    let r = compare(&a, &b, args.max_dt)?;
    if args.json {
        println!("{}", serde_json::to_string_pretty(&r)?);
    } else {
        print!("{}", render_text(&r));
    }
    Ok(())
}
