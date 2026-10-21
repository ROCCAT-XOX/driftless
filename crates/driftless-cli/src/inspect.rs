//! `driftless inspect <ordner> [--json]`: report about a scan folder.

use driftless_steps::inspect::{inspect, render_text};
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Ausgabe als JSON
    #[arg(long)]
    json: bool,
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let report = inspect(&args.folder, &crate::console())?;
    if args.json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print!("{}", render_text(&report));
    }
    Ok(())
}
