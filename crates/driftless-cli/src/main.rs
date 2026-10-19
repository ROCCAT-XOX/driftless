//! `driftless`: command line entry point. Each subcommand lives in its own module.

mod compare;
mod inspect;
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
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Inspect(args) => inspect::run(&args),
        Command::Compare(args) => compare::run(&args),
        Command::View(args) => view::run(&args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Fehler: {e}");
            ExitCode::FAILURE
        }
    }
}
