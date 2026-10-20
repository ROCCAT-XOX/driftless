//! `driftless`: command line entry point. Each subcommand lives in its own module.

mod calib_check;
mod compare;
mod correct;
mod export_glim;
mod inspect;
mod map;
mod odom;
mod scan_input;
mod slam;
mod topics;
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
    /// SLAM mit Schleifenschluss: traj.tum, traj_odom.tum, map.las, slam.json, submaps.bin
    Slam(slam::Args),
    /// Manuelle Korrekturen eines SLAM-Laufs (corrections.toml): auflisten, Schleife
    /// eintragen, ab- und einschalten
    Correct(correct::Args),
    /// Prüft die Kamera-Kalibrierung per Test-Projektion: Richtung der LiDAR↔Kamera-Matrix
    /// und Zeitversatz der Bilder (calib_check.json, Überlagerungsbilder)
    CalibCheck(calib_check::Args),
    /// Karte eines SLAM-Laufs: dichte Wolke (dense.las), bereinigt (clean.las, removed.las),
    /// eingefärbt (color.las, color.ply) und map.json; jeder Schritt einzeln startbar
    Map(map::Args),
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Inspect(args) => inspect::run(&args),
        Command::Compare(args) => compare::run(&args),
        Command::View(args) => view::run(&args),
        Command::ExportGlim(args) => export_glim::run(&args),
        Command::Odom(args) => odom::run(&args),
        Command::Slam(args) => slam::run(&args),
        Command::Correct(args) => correct::run(&args),
        Command::CalibCheck(args) => calib_check::run(&args),
        Command::Map(args) => map::run(&args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Fehler: {e}");
            ExitCode::FAILURE
        }
    }
}
