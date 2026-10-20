//! `driftless correct <ziel> …`: manual corrections of a SLAM run in `<ziel>/corrections.toml`.
//! Entries are only appended or switched off and on, never removed; `driftless slam --reuse`
//! applies them in seconds.

use crate::slam::{CACHE_FILE, CORRECTIONS_FILE};
use driftless_slam::{match_manual_loops, read_cache, CacheError, Corrections, ManualLoop};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

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
    /// und im Viewer) sind derselbe Ort
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
    if !args.target.is_dir() {
        return Err(format!(
            "Zielordner {} fehlt; zuerst `driftless slam <ordner> -o {}` laufen lassen",
            args.target.display(),
            args.target.display()
        )
        .into());
    }
    let path = args.target.join(CORRECTIONS_FILE);
    let mut c = Corrections::load(&path)?;
    match &args.action {
        Action::List => list(&c, &path),
        Action::Loop { a, b, note } => {
            check_new_loop(&args.target, &c, *a, *b)?;
            let nr = c.add_loop(*a, *b, note, &utc_now());
            c.save(&path)?;
            println!("Eintrag {nr} angehängt: Schleife {a} s ↔ {b} s");
            print_reuse_hint(&args.target);
        }
        Action::Disable { nr } | Action::Enable { nr } => {
            let enable = matches!(args.action, Action::Enable { .. });
            let state = if enable { "aktiv" } else { "deaktiviert" };
            if c.set_enabled(*nr, enable)? {
                c.save(&path)?;
                println!("Eintrag {nr} ist jetzt {state}");
                print_reuse_hint(&args.target);
            } else {
                println!("Eintrag {nr} ist schon {state}; nichts geändert");
            }
        }
    }
    Ok(())
}

fn list(c: &Corrections, path: &Path) {
    if c.loops.is_empty() {
        println!("keine Korrekturen in {}", path.display());
        return;
    }
    println!("Nr  Status       Art       a (s)      b (s)  hinzugefügt           Notiz");
    for (i, l) in c.loops.iter().enumerate() {
        println!(
            "{:<3} {:<12} {:<8} {:>6.1} {:>10.1}  {:<21} {}",
            i + 1,
            if l.enabled { "aktiv" } else { "deaktiviert" },
            "Schleife",
            l.a,
            l.b,
            l.added,
            l.note
        );
    }
}

/// Checks a new loop together with the active ones against the submaps of `submaps.bin`
/// (times inside the scan, two different submaps, no second loop between the same pair).
/// Without a cache only the times themselves and exact repeats are checked.
fn check_new_loop(
    target: &Path,
    c: &Corrections,
    a: f64,
    b: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    for t in [a, b] {
        if !t.is_finite() || t < 0.0 {
            return Err(format!("Zeit {t} s liegt außerhalb des Scans (ab 0 s)").into());
        }
    }
    let mut loops = c.manual_loops();
    let new = ManualLoop { a_t: a, b_t: b };
    if let Some(old) = loops
        .iter()
        .find(|l| (l.a_t, l.b_t) == (a, b) || (l.a_t, l.b_t) == (b, a))
    {
        return Err(format!(
            "doppelte Schleife: {} s ↔ {} s ist schon eingetragen",
            old.a_t, old.b_t
        )
        .into());
    }
    loops.push(new);
    match read_cache(&target.join(CACHE_FILE)) {
        Ok(cache) => {
            match_manual_loops(&cache.submaps, &loops)?;
            Ok(())
        }
        Err(CacheError::Missing(_)) => Ok(()),
        Err(e) => Err(e.into()),
    }
}

fn print_reuse_hint(target: &Path) {
    println!(
        "Neu rechnen: driftless slam <scan-ordner> -o {} --reuse",
        target.display()
    );
}

/// Current time as RFC 3339 in UTC, to the second.
fn utc_now() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let (days, rest) = (secs / 86_400, secs % 86_400);
    // Civil date from days since 1970-01-01 (H. Hinnant's algorithm).
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rest / 3600,
        rest % 3600 / 60,
        rest % 60
    )
}
