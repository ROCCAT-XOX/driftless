//! `driftless`: command line entry point. Each subcommand lives in its own module.

mod calib_check;
mod compare;
mod correct;
mod export;
mod export_glim;
mod inspect;
mod map;
mod merge;
mod odom;
mod slam;
mod surface;
// The rerun viewer only with `--features debug-viewer`; else `view` says where to look.
#[cfg(feature = "debug-viewer")]
mod view;
#[cfg(not(feature = "debug-viewer"))]
#[path = "view_off.rs"]
mod view;

use clap::{Parser, Subcommand};
use driftless_io::progress::{LogLine, Progress};
use driftless_steps::StepError;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

/// The system allocator keeps the per-image buffers of `map` mapped (peak 9.9 GB on a 10 min
/// scan); mimalloc returns them (3 GB).
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

/// `driftless --version`: the version and the commit it was built from (`build.rs`).
const LONG_VERSION: &str = concat!(
    env!("CARGO_PKG_VERSION"),
    " (",
    env!("DRIFTLESS_COMMIT"),
    ")"
);

#[derive(Parser)]
#[command(
    name = "driftless",
    version,
    long_version = LONG_VERSION,
    about = "Driftless: Handheld-LiDAR-Scans"
)]
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
    #[cfg_attr(
        feature = "debug-viewer",
        doc = "Fehlersuche-Viewer (rerun): Punkte, IMU, Bilder, Trajektorie"
    )]
    #[cfg_attr(
        not(feature = "debug-viewer"),
        doc = "Fehlersuche-Viewer: nicht enthalten (bauen mit --features debug-viewer); \
               Wolken zeigt die Desktop-App"
    )]
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
    /// Karte eines SLAM-Laufs: dichte Wolke, bereinigt (removed.las), eingefärbt (color.las,
    /// ohne Farbe clean.las) und map.json; jeder Schritt einzeln startbar, --alle-dateien
    /// schreibt jeden als LAS und color.ply dazu
    Map(map::Args),
    /// Fotoähnliche Oberfläche einer Karte oder eines Modells (--model): oberflaeche.glb und
    /// oberflaeche.json, mit --obj auch oberflaeche.obj (mit .mtl und Texturseiten)
    Surface(surface::Args),
    /// Führt mehrere Scans eines Orts zu einem Modell zusammen: modell.las und modell.json
    /// (Lage und Güte je Scan), mit --ply auch modell.ply
    Merge(merge::Args),
    /// Exportiert Karte, Modell, Oberfläche, Laufbahn oder eine Punktwolken-Datei (LAS, LAZ,
    /// E57, PLY, PCD, glb, OBJ, TUM, CSV)
    Export(export::Args),
}

impl Command {
    /// Whether it runs a step that stops cleanly when its progress is cancelled: Ctrl+C
    /// cancels it ([`stop_on_ctrl_c`]). The others end at once, as without a handler.
    fn runs_a_step(&self) -> bool {
        matches!(
            self,
            Command::Slam(_)
                | Command::Map(_)
                | Command::Surface(_)
                | Command::Merge(_)
                | Command::Export(_)
        )
    }
}

/// Exit code of a step stopped with Ctrl+C: 128 + SIGINT, as a shell reports a program that
/// Ctrl+C ended.
const EXIT_CANCELLED: u8 = 130;

/// The progress of the step this command runs (all clones of one): Ctrl+C cancels it.
static CONSOLE: OnceLock<Progress> = OnceLock::new();

/// Progress of a step on the console: messages to stdout, warnings as „Warnung: …“ to stderr.
fn console() -> Progress {
    CONSOLE
        .get_or_init(|| {
            halt_for_tests(Progress::with_log(|line| match line {
                LogLine::Info(s) => println!("{s}"),
                LogLine::Warning(s) => eprintln!("Warnung: {s}"),
            }))
        })
        .clone()
}

/// Ctrl+C cancels the step through [`console`]: it stops at its next check and cleans up as
/// on any cancel (a step that is writing its files finishes them). A second Ctrl+C ends the
/// program at once; the `.tmp` files left then are removed by the next run of the step.
fn stop_on_ctrl_c() {
    let progress = console();
    let pressed = AtomicBool::new(false);
    let installed = ctrlc::set_handler(move || {
        if pressed.swap(true, Ordering::SeqCst) {
            eprintln!(
                "Strg+C zum zweiten Mal: sofort beendet; halb geschriebene Dateien (*.tmp) \
                 entfernt der nächste Lauf"
            );
            std::process::exit(EXIT_CANCELLED.into());
        }
        eprintln!(
            "Strg+C: wird abgebrochen, sobald der Schritt anhalten kann – noch einmal Strg+C \
             beendet sofort"
        );
        progress.cancel();
    });
    if let Err(e) = installed {
        eprintln!("Warnung: Strg+C lässt sich nicht abfangen ({e}), es beendet sofort");
    }
}

/// For the Ctrl+C tests, in debug builds only: with `DRIFTLESS_TEST_HALT=<phase>` the step
/// says so on stderr when that phase starts and waits there until it is cancelled (with
/// `DRIFTLESS_TEST_HALT_STUR` set: for good).
#[cfg(debug_assertions)]
fn halt_for_tests(progress: Progress) -> Progress {
    let Ok(phase) = std::env::var("DRIFTLESS_TEST_HALT") else {
        return progress;
    };
    let stubborn = std::env::var_os("DRIFTLESS_TEST_HALT_STUR").is_some();
    let handle = progress.clone();
    progress.with_observer(move |s| {
        if s.phase == phase && s.done == 0 {
            eprintln!("Test: hält in Phase {phase}");
            while stubborn || !handle.is_cancelled() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }
    })
}

#[cfg(not(debug_assertions))]
fn halt_for_tests(progress: Progress) -> Progress {
    progress
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    if cli.command.runs_a_step() {
        stop_on_ctrl_c();
    }
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
        Command::Surface(args) => surface::run(&args),
        Command::Merge(args) => merge::run(&args),
        Command::Export(args) => export::run(&args),
    };
    let cancelled = CONSOLE.get().is_some_and(Progress::is_cancelled);
    match result.map_err(StepError::from) {
        Ok(()) => {
            if cancelled {
                eprintln!(
                    "Strg+C kam zu spät (der Schritt schrieb schon): das Ergebnis ist \
                     vollständig geschrieben"
                );
            }
            ExitCode::SUCCESS
        }
        Err(StepError::Cancelled) => {
            eprintln!(
                "Abgebrochen mit Strg+C: nichts Halbes geschrieben; denselben Befehl erneut \
                 starten, um fertig zu rechnen"
            );
            ExitCode::from(EXIT_CANCELLED)
        }
        Err(e) => {
            eprintln!("Fehler: {e}");
            ExitCode::FAILURE
        }
    }
}
