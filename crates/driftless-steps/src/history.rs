//! `verlauf/` of a SLAM run folder: after every run of the desktop program a numbered state
//! `verlauf/NNN/` with copies of `traj.tum`, `slam.json`, `corrections.toml` and a
//! `stand.toml`:
//!
//! ```toml
//! nr = 3
//! art = "neu gerechnet"
//! erstellt = "2026-10-21T14:03:12Z"
//! sekunden = 2.4
//! ```
//!
//! States are only added, never changed or removed. `erstellt` and `sekunden` are the only
//! values that differ between two equal runs.

use crate::correct::utc_now;
use crate::error::{failed, StepError};
use crate::slam::CORRECTIONS_FILE;
use driftless_slam::Corrections;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub const HISTORY_DIR: &str = "verlauf";
const STATE_FILE: &str = "stand.toml";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HistoryKind {
    /// A full run.
    Full,
    /// Recomputed from `submaps.bin`.
    Reuse,
    /// A result the history does not hold (a run made without history, or recomputed
    /// without it, e.g. by `driftless slam`), kept before a reuse of the desktop program
    /// replaces it.
    Imported,
}

impl HistoryKind {
    /// The `art` in `stand.toml`.
    pub fn label(self) -> &'static str {
        match self {
            HistoryKind::Full => "voll",
            HistoryKind::Reuse => "neu gerechnet",
            HistoryKind::Imported => "übernommen",
        }
    }

    fn from_label(s: &str) -> Option<Self> {
        [HistoryKind::Full, HistoryKind::Reuse, HistoryKind::Imported]
            .into_iter()
            .find(|k| k.label() == s)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HistoryEntry {
    pub nr: u32,
    pub dir: PathBuf,
    pub kind: HistoryKind,
    /// RFC 3339, UTC.
    pub created: String,
    /// Runtime of the run (s), rounded to ms.
    pub seconds: f64,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct State {
    nr: u32,
    art: String,
    erstellt: String,
    sekunden: f64,
}

fn state_dir(out: &Path, nr: u32) -> PathBuf {
    out.join(HISTORY_DIR).join(format!("{nr:03}"))
}

/// Numbers of all folders in `verlauf/`, readable or not.
fn numbers(out: &Path) -> Vec<u32> {
    let Ok(entries) = fs::read_dir(out.join(HISTORY_DIR)) else {
        return Vec::new();
    };
    entries
        .filter_map(Result::ok)
        .filter(|e| e.path().is_dir())
        .filter_map(|e| e.file_name().to_str()?.parse().ok())
        .collect()
}

/// Records the current result of the run folder `out` as a new state and returns its folder.
/// The number is one more than the largest present; `stand.toml` is written last, so an
/// interrupted copy is never listed. A missing `slam.json` is left out of the state.
pub fn record(out: &Path, kind: HistoryKind, seconds: f64) -> Result<PathBuf, StepError> {
    record_with(out, kind, seconds, None)
}

/// [`record`] with `corrections` stored instead of a copy of the current file.
pub(crate) fn record_with(
    out: &Path,
    kind: HistoryKind,
    seconds: f64,
    corrections: Option<&Corrections>,
) -> Result<PathBuf, StepError> {
    let nr = numbers(out).into_iter().max().unwrap_or(0) + 1;
    let dir = state_dir(out, nr);
    let fail = |e: &dyn std::fmt::Display| format!("Stand {} nicht schreibbar: {e}", dir.display());
    fs::create_dir_all(&dir).map_err(|e| fail(&e))?;
    let copy = |file: &str| {
        fs::copy(out.join(file), dir.join(file)).map_err(|e| fail(&format!("{file}: {e}")))
    };
    copy("traj.tum")?;
    if out.join("slam.json").exists() {
        copy("slam.json")?;
    }
    let path = out.join(CORRECTIONS_FILE);
    match corrections {
        Some(c) => c.save(&dir.join(CORRECTIONS_FILE))?,
        None if path.exists() => {
            copy(CORRECTIONS_FILE)?;
        }
        None => Corrections::default().save(&dir.join(CORRECTIONS_FILE))?,
    }
    let state = State {
        nr,
        art: kind.label().to_string(),
        erstellt: utc_now(),
        sekunden: (seconds * 1000.0).round() / 1000.0,
    };
    let text = toml::to_string(&state).map_err(|e| fail(&e))?;
    driftless_io::atomic_write(&dir.join(STATE_FILE), text.as_bytes()).map_err(|e| fail(&e))?;
    Ok(dir)
}

/// The readable states of `out`, by number; folders without a valid `stand.toml` are
/// skipped.
pub fn list(out: &Path) -> Vec<HistoryEntry> {
    let mut entries: Vec<HistoryEntry> = numbers(out)
        .into_iter()
        .filter_map(|nr| {
            let dir = state_dir(out, nr);
            let text = fs::read_to_string(dir.join(STATE_FILE)).ok()?;
            let state: State = toml::from_str(&text).ok()?;
            Some(HistoryEntry {
                nr,
                kind: HistoryKind::from_label(&state.art)?,
                created: state.erstellt,
                seconds: state.sekunden,
                dir,
            })
        })
        .collect();
    entries.sort_by_key(|e| e.nr);
    entries
}

/// Whether the newest readable state of `out` has the current `traj.tum` (byte-equal). False
/// without states, or when `traj.tum` was replaced since (e.g. by `driftless slam --reuse`).
pub fn holds_current(out: &Path) -> bool {
    let Some(newest) = list(out).pop() else {
        return false;
    };
    match (
        fs::read(newest.dir.join("traj.tum")),
        fs::read(out.join("traj.tum")),
    ) {
        (Ok(then), Ok(now)) => then == now,
        _ => false,
    }
}

/// Switches the corrections of `out` back to state `nr`: entry i gets the switch it had in
/// that state, entries added after it are switched off, and the automatic loops switched off
/// are those of the state (its reject entries). No manual loop is removed. Returns the number
/// of entries that changed (each reject added or removed counts).
pub fn restore_flags(out: &Path, nr: u32) -> Result<usize, StepError> {
    let entry = list(out).into_iter().find(|e| e.nr == nr).ok_or_else(|| {
        format!(
            "Stand {nr} nicht gefunden in {}",
            out.join(HISTORY_DIR).display()
        )
    })?;
    let then = Corrections::load(&entry.dir.join(CORRECTIONS_FILE))?;
    let path = out.join(CORRECTIONS_FILE);
    let mut now = Corrections::load(&path)?;
    let mut changed = 0;
    for (i, l) in now.loops.iter_mut().enumerate() {
        let enabled = then.loops.get(i).is_some_and(|t| t.enabled);
        if l.enabled != enabled {
            l.enabled = enabled;
            changed += 1;
        }
    }
    if now.rejects != then.rejects {
        let gone = now.rejects.iter().filter(|r| !then.rejects.contains(r));
        let back = then.rejects.iter().filter(|r| !now.rejects.contains(r));
        changed += gone.count() + back.count();
        now.rejects = then.rejects.clone();
    }
    if changed > 0 {
        now.save(&path).map_err(failed)?;
    }
    Ok(changed)
}
