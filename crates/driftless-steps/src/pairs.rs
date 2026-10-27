//! `punktpaare.toml`: point pairs marked by hand between two scans of a model, in the models
//! folder of a project. Like `corrections.toml` ([`driftless_slam::Corrections`]), entries
//! are only ever appended or switched off and on again, never removed.
//!
//! ```toml
//! [[paar]]
//! a = "scans/raum-1/driftless"
//! b = "scans/raum-2/driftless"
//! points_a = [[0.1, 2.0, 0.5], [3.2, 2.1, 0.4], [1.5, 0.2, 2.3]]
//! points_b = [[1.9, 0.3, 0.5], [2.0, 3.4, 0.4], [0.1, 1.7, 2.3]]
//! enabled = true
//! note = "Türrahmen und Ecke"
//! added = "2026-10-27T14:03:12Z"
//! ```
//!
//! `a` and `b` are map folders, relative to the project with `/` as in `project.toml`
//! (absolute if outside it); [`PointPairs::resolved`] makes them paths of the file system
//! for [`crate::merge::MergeRequest::pairs`]. A switched-on entry replaces the automatic
//! alignment of its two scans in the merge.

use crate::StepError;
use driftless_slam::align::pose_from_pairs;
use driftless_traj::nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// File of the point pairs, in `<projekt>/modelle/`.
pub const POINT_PAIRS_FILE: &str = "punktpaare.toml";

/// Fewest pairs of an entry.
pub const MIN_POINT_PAIRS: usize = 3;

/// Comment at the top of the file.
const HEADER: &str = "# Driftless-Punktpaare: gleiche Stellen in zwei Scans, Punkte in Metern im \
Rahmen des jeweiligen Scans.\n# Pfade relativ zum Projekt; Einträge werden nur angehängt oder \
abgeschaltet.\n\n";

/// Points marked by hand on two scans that show the same places.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PointPairEntry {
    /// Map folders of the two scans.
    pub a: PathBuf,
    pub b: PathBuf,
    /// The points, pair by pair (at least [`MIN_POINT_PAIRS`]), each in the world frame of
    /// its own scan.
    pub points_a: Vec<[f64; 3]>,
    pub points_b: Vec<[f64; 3]>,
    /// Switched off entries are kept but not used.
    pub enabled: bool,
    #[serde(default)]
    pub note: String,
    /// When the entry was added (RFC 3339, UTC).
    #[serde(default)]
    pub added: String,
}

impl PointPairEntry {
    /// The pairs as (point in `a`, point in `b`).
    pub fn pairs(&self) -> Vec<(Vector3<f64>, Vector3<f64>)> {
        self.points_a
            .iter()
            .zip(&self.points_b)
            .map(|(p, q)| (Vector3::from(*p), Vector3::from(*q)))
            .collect()
    }

    /// Whether the entry can be used at all: at least [`MIN_POINT_PAIRS`] pairs, as many
    /// points in `a` as in `b`, all finite. The message is German.
    pub fn check(&self) -> Result<(), String> {
        let (na, nb) = (self.points_a.len(), self.points_b.len());
        if na != nb {
            return Err(format!(
                "Punktpaare: gleich viele Punkte in beiden Scans nötig ({na} und {nb})"
            ));
        }
        if na < MIN_POINT_PAIRS {
            return Err(format!(
                "Punktpaare: mindestens drei Paare nötig, nicht {na}"
            ));
        }
        let finite = self
            .points_a
            .iter()
            .chain(&self.points_b)
            .flatten()
            .all(|c| c.is_finite());
        if !finite {
            return Err("Punktpaare: ungültige Koordinate".into());
        }
        Ok(())
    }
}

/// The entries of `punktpaare.toml`, in file order. Unknown keys (e.g. a mistyped hand edit)
/// are an error instead of being dropped silently.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PointPairs {
    #[serde(rename = "paar", default)]
    pub entries: Vec<PointPairEntry>,
}

impl PointPairs {
    /// Reads `path`; a missing file means no pairs yet. An entry that cannot be used
    /// ([`PointPairEntry::check`]) is an error naming its number (1-based).
    pub fn load(path: &Path) -> Result<Self, StepError> {
        let text = match fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Self::default()),
            Err(e) => return Err(format!("{}: {e}", path.display()).into()),
        };
        let pairs: Self =
            toml::from_str(&text).map_err(|e| format!("{} nicht lesbar: {e}", path.display()))?;
        for (k, e) in pairs.entries.iter().enumerate() {
            e.check()
                .map_err(|m| format!("{}, Eintrag {}: {m}", path.display(), k + 1))?;
        }
        Ok(pairs)
    }

    /// Writes `path` through a file next to it, so an interrupted write never leaves half a
    /// file behind.
    pub fn save(&self, path: &Path) -> Result<(), StepError> {
        let fail = |e: &dyn std::fmt::Display| format!("{} nicht schreibbar: {e}", path.display());
        let body = toml::to_string(self).map_err(|e| fail(&e))?;
        driftless_io::atomic_write(path, format!("{HEADER}{body}").as_bytes())
            .map_err(|e| StepError::Failed(fail(&e)))
    }

    /// Appends `e` and returns its index (0-based, for [`Self::set_enabled`]). Refused with a
    /// German message, adding nothing: an entry that cannot be used
    /// ([`PointPairEntry::check`]), the same folder as `a` and `b`, or points on a line (or
    /// all in one place) in either scan, which fix no turn.
    pub fn add(&mut self, e: PointPairEntry) -> Result<usize, StepError> {
        e.check()?;
        if e.a == e.b {
            return Err("Punktpaare: zwei verschiedene Scans wählen".into());
        }
        if pose_from_pairs(&e.pairs()).is_none() {
            return Err(
                "Punktpaare: die Punkte liegen auf einer Linie – weiter auseinander wählen".into(),
            );
        }
        self.entries.push(e);
        Ok(self.entries.len() - 1)
    }

    /// Switches entry `index` (0-based) on or off; the entry itself stays.
    pub fn set_enabled(&mut self, index: usize, enabled: bool) -> Result<(), StepError> {
        let n = self.entries.len();
        let entry = self.entries.get_mut(index).ok_or_else(|| match n {
            0 => format!("kein Punktpaar-Eintrag {index}: es gibt noch keine"),
            _ => format!(
                "kein Punktpaar-Eintrag {index} (vorhanden: 0 bis {})",
                n - 1
            ),
        })?;
        entry.enabled = enabled;
        Ok(())
    }

    /// The entries with `a` and `b` as paths of the file system: relative ones (with `/` or
    /// `\`) below `base`, absolute ones as written ([`stored_path`]).
    pub fn resolved(&self, base: &Path) -> Vec<PointPairEntry> {
        let resolve = |p: &Path| stored_path(base, &p.to_string_lossy());
        self.entries
            .iter()
            .map(|e| PointPairEntry {
                a: resolve(&e.a),
                b: resolve(&e.b),
                ..e.clone()
            })
            .collect()
    }
}

/// Absolute on Windows: `C:\…`, `C:/…` or `\\server\…` (also read on another system, where
/// such a path is just not found instead of being taken as relative).
fn windows_absolute(text: &str) -> bool {
    let b = text.as_bytes();
    let drive =
        b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'/' | b'\\');
    drive || text.starts_with("\\\\") || text.starts_with("//")
}

/// A path from a project file (`project.toml`, `punktpaare.toml`): absolute as written,
/// relative (with `/` or `\`) to `base`.
pub fn stored_path(base: &Path, text: &str) -> PathBuf {
    let p = Path::new(text);
    if p.is_absolute() || windows_absolute(text) {
        return p.to_path_buf();
    }
    let mut out = base.to_path_buf();
    for part in text
        .split(['/', '\\'])
        .filter(|s| !s.is_empty() && *s != ".")
    {
        out.push(part);
    }
    out
}
