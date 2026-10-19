//! `driftless`: command line entry point. Each subcommand lives in its own module.

mod inspect;

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
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Inspect(args) => inspect::run(&args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Fehler: {e}");
            ExitCode::FAILURE
        }
    }
}
