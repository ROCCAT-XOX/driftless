//! What a job cut off left behind (a crash, a force quit, a power cut): a run, map, surface or
//! model folder without its final report (`slam.json`, `map.json`, `oberflaeche.glb`,
//! `modell.json`), or a stale `<name>.tmp` beside a file being written. A cancelled or failed
//! job removes its new folder itself ([`crate::project::StepDir`]); these are what remains when
//! the program had no chance to. „Daten“ lists them as „unvollständig“, „Aufräumen“ moves them
//! to the trash.
//!
//! What a job may still write is left alone: a folder a running job marks busy ([`BUSY_FILE`],
//! touched every [`BUSY_BEAT`]; this window's, another window's), and what was written to within
//! [`QUIET`] (a job of the command line, which does not mark its folders; see [`settled`]).

use crate::disk::folder_size;
use crate::project::SURFACE_NUMBERED;
use crate::project::{numbered, MAP_PREFIX, MODELS_DIR, MODEL_PREFIX, RUN_PREFIX, SCANS_DIR};
use crate::surface::SURFACE_GLB;
use crate::ui::text;
use driftless_steps::map::REPORT as MAP_REPORT;
use driftless_steps::merge::MODEL_REPORT;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

/// The mark of a folder a job writes into right now: it names the process.
pub const BUSY_FILE: &str = ".in-arbeit";
/// How often a running job touches its [`BUSY_FILE`].
pub const BUSY_BEAT: Duration = Duration::from_secs(10);
/// A [`BUSY_FILE`] not touched for this long belongs to a job that was cut off.
pub const BUSY_STALE: Duration = Duration::from_secs(60);
/// What was written to within this time is not offered for „Aufräumen“ yet.
pub const QUIET: Duration = Duration::from_secs(120);

/// The final report of a run folder.
const SLAM_REPORT: &str = "slam.json";

/// What a leftover is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeftoverKind {
    Run,
    Map,
    Surface,
    Model,
    /// A `<name>.tmp` in a folder that is complete otherwise.
    Temp,
}

/// A folder or file a job cut off left behind.
#[derive(Debug, Clone, PartialEq)]
pub struct Leftover {
    pub path: PathBuf,
    pub kind: LeftoverKind,
    /// How „Daten“ names it: „SLAM 002“, „Karte 002 (SLAM 001)“, „clean.bin.tmp (Karte 001)“.
    pub label: String,
    pub bytes: u64,
    /// When it was last written: for a folder the newest of it and what lies directly in it.
    pub modified: SystemTime,
}

/// Whether `leftover` has lain untouched for `quiet` at `now` (zero: at once).
pub fn settled(leftover: &Leftover, quiet: Duration, now: SystemTime) -> bool {
    quiet.is_zero()
        || now
            .duration_since(leftover.modified)
            .is_ok_and(|age| age >= quiet)
}

/// The leftovers in the project folder `dir` with the scans `scan_ids`: incomplete runs, maps
/// and surfaces under `scans/<id>`, incomplete models and their surfaces under `modelle`, and
/// stale `.tmp` files in all of these and in the project folder itself. Folders a running job
/// marks busy are left out with what lies in them. Reads folders: call it off the UI thread.
pub fn find(dir: &Path, scan_ids: &[String]) -> Vec<Leftover> {
    let mut out = Vec::new();
    for id in scan_ids {
        for (_, digits, run) in numbered(&dir.join(SCANS_DIR).join(id), RUN_PREFIX) {
            let run_label = format!("SLAM {digits}");
            if busy(&run) {
                continue;
            }
            if !run.join(SLAM_REPORT).is_file() {
                out.push(folder(LeftoverKind::Run, &run, run_label));
                continue;
            }
            temps(&run, Some(&run_label), &mut out);
            for (_, digits, map) in numbered(&run, MAP_PREFIX) {
                let map_label = format!("Karte {digits}");
                if busy(&map) {
                    continue;
                }
                if !map.join(MAP_REPORT).is_file() {
                    let label = format!("{map_label} ({run_label})");
                    out.push(folder(LeftoverKind::Map, &map, label));
                    continue;
                }
                temps(&map, Some(&map_label), &mut out);
                surfaces(&map, &map_label, &mut out);
            }
        }
    }
    let models = dir.join(MODELS_DIR);
    for (_, digits, model) in numbered(&models, MODEL_PREFIX) {
        let label = text::model_label(&digits);
        if busy(&model) {
            continue;
        }
        if !model.join(MODEL_REPORT).is_file() {
            out.push(folder(LeftoverKind::Model, &model, label));
            continue;
        }
        temps(&model, Some(&label), &mut out);
        surfaces(&model, &label, &mut out);
    }
    temps(&models, None, &mut out);
    temps(dir, None, &mut out);
    out
}

/// The surfaces `oberflaeche-NNN` of the map or model folder `dir` (named `parent`): one
/// without its glb whole, else its stale `.tmp` files.
fn surfaces(dir: &Path, parent: &str, out: &mut Vec<Leftover>) {
    for (_, digits, surface) in numbered(dir, SURFACE_NUMBERED) {
        let label = text::surface_label(&digits);
        if busy(&surface) {
            continue;
        }
        if surface.join(SURFACE_GLB).is_file() {
            temps(&surface, Some(&label), out);
        } else {
            let label = format!("{label} ({parent})");
            out.push(folder(LeftoverKind::Surface, &surface, label));
        }
    }
}

/// The `<name>.tmp` files directly in `dir`, named with `parent` if given.
fn temps(dir: &Path, parent: Option<&str>, out: &mut Vec<Leftover>) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    let mut found: Vec<Leftover> = entries
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_str()?.to_string();
            let meta = e.metadata().ok().filter(|m| m.is_file())?;
            name.ends_with(".tmp").then(|| Leftover {
                path: e.path(),
                kind: LeftoverKind::Temp,
                label: match parent {
                    Some(parent) => format!("{name} ({parent})"),
                    None => name,
                },
                bytes: meta.len(),
                modified: meta.modified().unwrap_or(SystemTime::UNIX_EPOCH),
            })
        })
        .collect();
    found.sort_by(|a, b| a.path.cmp(&b.path));
    out.extend(found);
}

/// The incomplete folder `dir` as a leftover.
fn folder(kind: LeftoverKind, dir: &Path, label: String) -> Leftover {
    Leftover {
        path: dir.to_path_buf(),
        kind,
        label,
        bytes: folder_size(dir),
        modified: newest(dir),
    }
}

/// When the folder `dir` or what lies directly in it was last written.
fn newest(dir: &Path) -> SystemTime {
    let own = fs::metadata(dir).and_then(|m| m.modified());
    let entries = fs::read_dir(dir).into_iter().flatten().flatten();
    let inside = entries.filter_map(|e| e.metadata().and_then(|m| m.modified()).ok());
    own.into_iter()
        .chain(inside)
        .max()
        .unwrap_or(SystemTime::UNIX_EPOCH)
}

/// Whether a running job marks the folder `dir` busy: its [`BUSY_FILE`] was touched within
/// [`BUSY_STALE`].
fn busy(dir: &Path) -> bool {
    let touched = fs::metadata(dir.join(BUSY_FILE)).and_then(|m| m.modified());
    touched.is_ok_and(|t| {
        SystemTime::now()
            .duration_since(t)
            .map_or(true, |age| age < BUSY_STALE)
    })
}
