//! `driftless correct <ziel> …`: manual corrections of a SLAM run in `<ziel>/corrections.toml`.
//! Entries are only appended or switched off and on, never removed; `driftless slam --reuse`
//! applies them in seconds.

use driftless_steps::correct;
use driftless_steps::slam::CORRECTIONS_FILE;
use std::path::{Path, PathBuf};

#[derive(clap::Args)]
pub struct Args {
    /// Zielordner eines `driftless slam`-Laufs (dort liegt corrections.toml)
    target: PathBuf,
    #[command(subcommand)]
    action: Action,
}

#[derive(clap::Subcommand)]
enum Action {
    /// Alle Korrekturen nummeriert mit Status
    List,
    /// Manuelle Schleife: die Stellen zu zwei Zeiten (Sekunden ab Scan-Beginn, wie in slam.json
    /// und im Viewer auf der Zeitleiste ab_scan_beginn) sind derselbe Ort
    #[command(allow_negative_numbers = true)]
    Loop {
        /// Erste Zeit (s ab Scan-Beginn)
        a: f64,
        /// Zweite Zeit (s ab Scan-Beginn)
        b: f64,
        /// Notiz zum Eintrag
        #[arg(long, default_value = "")]
        note: String,
    },
    /// Eintrag abschalten (bleibt in der Datei stehen)
    Disable {
        /// Nummer aus `list`
        nr: usize,
    },
    /// Abgeschalteten Eintrag wieder einschalten
    Enable {
        /// Nummer aus `list`
        nr: usize,
    },
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    match &args.action {
        Action::List => {
            let c = correct::load(&args.target)?;
            print!(
                "{}",
                correct::render_list(&c, &args.target.join(CORRECTIONS_FILE))
            );
        }
        Action::Loop { a, b, note } => {
            let nr = correct::add_loop(&args.target, *a, *b, note)?;
            println!("{}", correct::added_message(nr, *a, *b));
            print_reuse_hint(&args.target);
        }
        Action::Disable { nr } | Action::Enable { nr } => {
            let enable = matches!(args.action, Action::Enable { .. });
            let changed = correct::set_enabled(&args.target, *nr, enable)?;
            println!("{}", correct::switched_message(*nr, enable, changed));
            if changed {
                print_reuse_hint(&args.target);
            }
        }
    }
    Ok(())
}

fn print_reuse_hint(target: &Path) {
    println!(
        "Neu rechnen: driftless slam <scan-ordner> -o {} --reuse",
        target.display()
    );
}
