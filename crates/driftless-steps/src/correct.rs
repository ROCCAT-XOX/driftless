//! Manual corrections of a SLAM run in `<lauf>/corrections.toml`. Entries are only appended
//! or switched off and on, never removed; a reuse of the run applies them in seconds.

use crate::error::{failed, StepError};
use crate::slam::{CACHE_FILE, CORRECTIONS_FILE};
use driftless_slam::{match_manual_loops, read_cache, CacheError, Corrections, ManualLoop};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

/// The corrections of the run folder `run`; the folder must exist.
pub fn load(run: &Path) -> Result<Corrections, StepError> {
    if !run.is_dir() {
        return Err(format!(
            "Zielordner {} fehlt; zuerst `driftless slam <ordner> -o {}` laufen lassen",
            run.display(),
            run.display()
        )
        .into());
    }
    Ok(Corrections::load(&run.join(CORRECTIONS_FILE))?)
}

/// Checks a new loop together with the active ones against the submaps of `submaps.bin`
/// (times inside the scan, two different submaps, no second loop between the same pair).
/// Without a cache only the times themselves and exact repeats are checked.
pub fn check_new_loop(run: &Path, c: &Corrections, a: f64, b: f64) -> Result<(), StepError> {
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
    match read_cache(&run.join(CACHE_FILE)) {
        Ok(cache) => {
            match_manual_loops(&cache.submaps, &loops).map_err(failed)?;
            Ok(())
        }
        Err(CacheError::Missing(_)) => Ok(()),
        Err(e) => Err(failed(e)),
    }
}

/// Checks and appends a manual loop between the places at times `a` and `b` (seconds since
/// the start of the scan) and saves the file; returns its number (from 1).
pub fn add_loop(run: &Path, a: f64, b: f64, note: &str) -> Result<usize, StepError> {
    add_loop_at(run, a, b, note, &utc_now())
}

/// [`add_loop`] with the time stamp `added` (for tests).
#[doc(hidden)]
pub fn add_loop_at(
    run: &Path,
    a: f64,
    b: f64,
    note: &str,
    added: &str,
) -> Result<usize, StepError> {
    let mut c = load(run)?;
    check_new_loop(run, &c, a, b)?;
    let nr = c.add_loop(a, b, note, added);
    c.save(&run.join(CORRECTIONS_FILE))?;
    Ok(nr)
}

/// Switches entry `nr` (from 1) on or off and saves the file; returns whether it changed.
/// Switching an entry back on is checked like a new loop against the active ones.
pub fn set_enabled(run: &Path, nr: usize, enabled: bool) -> Result<bool, StepError> {
    let mut c = load(run)?;
    if let Some(l) = c
        .loops
        .get(nr.wrapping_sub(1))
        .filter(|l| enabled && !l.enabled)
    {
        check_new_loop(run, &c, l.a, l.b)
            .map_err(|e| format!("Eintrag {nr} nicht eingeschaltet: {e}"))?;
    }
    let changed = c.set_enabled(nr, enabled)?;
    if changed {
        c.save(&run.join(CORRECTIONS_FILE))?;
    }
    Ok(changed)
}

/// The corrections as `driftless correct <lauf> list` prints them (`path`: their file).
pub fn render_list(c: &Corrections, path: &Path) -> String {
    if c.loops.is_empty() {
        return format!("keine Korrekturen in {}\n", path.display());
    }
    let mut s =
        String::from("Nr  Status       Art       a (s)      b (s)  hinzugefügt           Notiz\n");
    for (i, l) in c.loops.iter().enumerate() {
        s += &format!(
            "{:<3} {:<12} {:<8} {:>6.1} {:>10.1}  {:<21} {}\n",
            i + 1,
            if l.enabled { "aktiv" } else { "deaktiviert" },
            "Schleife",
            l.a,
            l.b,
            l.added,
            l.note
        );
    }
    s
}

/// Current time as RFC 3339 in UTC, to the second.
pub fn utc_now() -> String {
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
