//! `driftless`: command line entry point. Each subcommand lives in its own module.

mod compare;
mod export_glim;
mod inspect;
mod odom;
mod view;

use clap::{Parser, Subcommand};
use std::process::ExitCode;

#[derive(Parser)]
#[command(name = "driftless", version, about = "Driftless: Handheld-LiDAR-Scans")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Bericht über einen Scan-Ordner: Gerät, Topics, Raten, Probleme
    Inspect(inspect::Args),
    /// Vergleicht zwei Trajektorien (TUM): Abweichung, Start↔Ende, Länge
    Compare(compare::Args),
    /// Fehlersuche-Viewer (rerun): Punkte, IMU, Bilder, Trajektorie
    View(view::Args),
    /// Spike: LiDAR- und IMU-Daten als Binärstrom für den GLIM-Treiber (spikes/glim)
    ExportGlim(export_glim::Args),
    /// LiDAR-IMU-Odometrie (Rust-Grundlinie): traj.tum und map.las
    Odom(odom::Args),
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Inspect(args) => inspect::run(&args),
        Command::Compare(args) => compare::run(&args),
        Command::View(args) => view::run(&args),
        Command::ExportGlim(args) => export_glim::run(&args),
        Command::Odom(args) => odom::run(&args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Fehler: {e}");
            ExitCode::FAILURE
        }
    }
}
