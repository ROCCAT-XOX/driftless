//! `corrections.toml`: the user's manual corrections of a SLAM run. Entries are only ever
//! appended or switched off and on again, never removed, so every correction can be undone
//! and the history stays visible.
//!
//! ```toml
//! [[loop]]
//! a = 12.3
//! b = 250.1
//! enabled = true
//! note = "Treppenhaus"
//! added = "2026-10-20T14:03:12Z"
//! ```
//!
//! Times are seconds since the start of the scan ([`crate::scan_start`]).

use crate::loops::ManualLoop;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io;
use std::path::Path;

/// Comment at the top of the file.
const HEADER: &str = "# Driftless-Korrekturen. Zeiten in Sekunden ab Scan-Beginn.\n\
# Bearbeiten mit `driftless correct`; Einträge werden nur angehängt oder deaktiviert.\n\n";

/// Unknown keys (e.g. a mistyped hand edit) are an error instead of being dropped silently.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Corrections {
    #[serde(rename = "loop", default)]
    pub loops: Vec<LoopEntry>,
}

/// One manual loop: the places at times `a` and `b` are the same.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoopEntry {
    pub a: f64,
    pub b: f64,
    pub enabled: bool,
    #[serde(default)]
    pub note: String,
    /// When the entry was added (RFC 3339, UTC).
    #[serde(default)]
    pub added: String,
}

impl Corrections {
    /// Reads `path`; a missing file means no corrections yet.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = match fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(e) => return Err(format!("{}: {e}", path.display())),
        };
        toml::from_str(&text).map_err(|e| format!("{} nicht lesbar: {e}", path.display()))
    }

    /// Writes `path`. The file is written next to it first and then renamed, so an
    /// interrupted write never leaves half a file behind.
    pub fn save(&self, path: &Path) -> Result<(), String> {
        let fail = |e: &dyn std::fmt::Display| format!("{} nicht schreibbar: {e}", path.display());
        let body = toml::to_string(self).map_err(|e| fail(&e))?;
        driftless_io::atomic_write(path, format!("{HEADER}{body}").as_bytes()).map_err(|e| fail(&e))
    }

    /// Appends an active loop and returns its number (1-based, as `driftless correct list`
    /// shows it).
    pub fn add_loop(&mut self, a: f64, b: f64, note: &str, added: &str) -> usize {
        self.loops.push(LoopEntry {
            a,
            b,
            enabled: true,
            note: note.to_string(),
            added: added.to_string(),
        });
        self.loops.len()
    }

    /// Switches entry `nr` (1-based) on or off; the entry itself stays. Returns whether it
    /// changed.
    pub fn set_enabled(&mut self, nr: usize, enabled: bool) -> Result<bool, String> {
        let n = self.loops.len();
        let entry = nr
            .checked_sub(1)
            .and_then(|i| self.loops.get_mut(i))
            .ok_or_else(|| match n {
                0 => format!("kein Eintrag {nr}: es gibt noch keine Korrekturen"),
                _ => format!("kein Eintrag {nr} (vorhanden: 1 bis {n})"),
            })?;
        let changed = entry.enabled != enabled;
        entry.enabled = enabled;
        Ok(changed)
    }

    /// The active loops, in file order.
    pub fn manual_loops(&self) -> Vec<ManualLoop> {
        self.loops
            .iter()
            .filter(|l| l.enabled)
            .map(|l| ManualLoop { a_t: l.a, b_t: l.b })
            .collect()
    }
}
