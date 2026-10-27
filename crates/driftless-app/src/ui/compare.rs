//! Before and after on the right: two trajectories of the scan („Vergleichen“) – of every run
//! the current one, its odometry, the one before its last recompute and its states – compared
//! as `driftless compare` does (A aligned to B, poses matched within [`MAX_DT`]); A is drawn
//! orange over the run of B. Two map folders of the scan side by side („Karten vergleichen“):
//! the settings and numbers of their `map.json`, differing values highlighted. And for a
//! selected state of the history: compare it with the current trajectory, or restore it
//! ([`Action::RestoreHistory`]: the switches of the corrections as in that state, then a
//! recompute – a new state, the old ones stay).
//!
//! The panel only asks ([`Action::Compare`], [`Action::CompareMaps`]); the program reads and
//! computes on its helper runner and puts the answer into [`CompareState`].

use super::status::count;
use super::{text, Action, AppState, ScanNode, Selection};
use crate::project::{run_label, RunInfo};
use driftless_steps::compare::{self, Report};
use driftless_steps::history::HISTORY_DIR;
use driftless_steps::StepError;
use driftless_traj::nalgebra::Isometry3;
use driftless_traj::{align_umeyama, read_tum, Pose};
use eframe::egui::{self, RichText, Ui};
use serde_json::Value;
use std::path::{Path, PathBuf};

/// Largest time between two matched poses (s), the default of `driftless compare`.
pub const MAX_DT: f64 = 0.01;

/// A trajectory of a run folder.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TrajRef {
    /// `traj.tum`.
    Current(PathBuf),
    /// `traj_odom.tum`, before loop closure.
    Odometry(PathBuf),
    /// `traj.vorher.tum`, before the last recompute.
    Previous(PathBuf),
    /// `verlauf/NNN/traj.tum`.
    History { run: PathBuf, nr: u32 },
}

impl TrajRef {
    /// The run folder.
    pub fn run(&self) -> &Path {
        match self {
            TrajRef::Current(r) | TrajRef::Odometry(r) | TrajRef::Previous(r) => r,
            TrajRef::History { run, .. } => run,
        }
    }

    /// The TUM file.
    pub fn path(&self) -> PathBuf {
        match self {
            TrajRef::Current(r) => r.join("traj.tum"),
            TrajRef::Odometry(r) => r.join("traj_odom.tum"),
            TrajRef::Previous(r) => r.join("traj.vorher.tum"),
            TrajRef::History { run, nr } => run
                .join(HISTORY_DIR)
                .join(format!("{nr:03}"))
                .join("traj.tum"),
        }
    }

    /// „SLAM 001 aktuell“, „… Odometrie“, „… vorher“, „… Stand 003“.
    pub fn label(&self) -> String {
        let what = match self {
            TrajRef::Current(_) => "aktuell".to_string(),
            TrajRef::Odometry(_) => "Odometrie".to_string(),
            TrajRef::Previous(_) => "vorher".to_string(),
            TrajRef::History { nr, .. } => text::state_name(*nr),
        };
        format!("{} {what}", run_label(self.run()))
    }
}

/// Two trajectories compared.
#[derive(Debug, Clone)]
pub struct Comparison {
    pub a: TrajRef,
    pub b: TrajRef,
    pub report: Report,
    /// Moves A onto B (as for the deviation).
    pub align: Isometry3<f64>,
    /// The poses of A as read (for drawing them moved by [`Comparison::align`]).
    pub a_poses: Vec<Pose>,
    /// The poses of B as read.
    pub b_poses: Vec<Pose>,
}

fn read(r: &TrajRef) -> Result<Vec<Pose>, StepError> {
    read_tum(&r.path()).map_err(|e| StepError::Failed(format!("{}: {e}", r.label())))
}

/// Compares `a` (aligned to `b`) with `b`, as `driftless compare a b` does.
pub fn compare_refs(a: &TrajRef, b: &TrajRef) -> Result<Comparison, StepError> {
    let (pa, pb) = (read(a)?, read(b)?);
    let report = compare::compare(&pa, &pb, MAX_DT)?;
    let align = align_umeyama(&pa, &pb, MAX_DT).map_err(|e| StepError::Failed(e.to_string()))?;
    Ok(Comparison {
        a: a.clone(),
        b: b.clone(),
        report,
        align,
        a_poses: pa,
        b_poses: pb,
    })
}

/// The trajectories of one run, as far as its files exist.
pub fn refs_of_run(run: &RunInfo) -> Vec<TrajRef> {
    let dir = &run.dir;
    let mut refs = Vec::new();
    if run.has_result {
        refs.push(TrajRef::Current(dir.clone()));
    }
    if run.has_odometry {
        refs.push(TrajRef::Odometry(dir.clone()));
    }
    if run.has_previous {
        refs.push(TrajRef::Previous(dir.clone()));
    }
    refs.extend(run.history.iter().map(|e| TrajRef::History {
        run: dir.clone(),
        nr: e.nr,
    }));
    refs
}

/// All trajectories of the scan, run by run.
pub fn refs_of_scan(scan: &ScanNode) -> Vec<TrajRef> {
    scan.runs.iter().flat_map(refs_of_run).collect()
}

/// What a run is compared with first, always with its current trajectory: after a recompute
/// the trajectory before it, else the odometry, without one the state before the last; `None`
/// without a current trajectory or anything to compare it with.
pub fn default_refs(run: &RunInfo) -> Option<(TrajRef, TrajRef)> {
    if !run.has_result {
        return None;
    }
    let dir = run.dir.clone();
    let a = if run.has_previous {
        TrajRef::Previous(dir.clone())
    } else if run.has_odometry {
        TrajRef::Odometry(dir.clone())
    } else {
        let before_last = run.history.iter().rev().nth(1)?;
        TrajRef::History {
            run: dir.clone(),
            nr: before_last.nr,
        }
    };
    Some((a, TrajRef::Current(dir)))
}

/// Whether a row of the map comparison is highlighted: its values differ, and it is no
/// runtime (those differ on every run).
pub fn highlighted(label: &str, a: &str, b: &str) -> bool {
    a != b
        && ![
            text::MAP_SECONDS_DENSE,
            text::MAP_SECONDS_CLEAN,
            text::MAP_SECONDS_COLOR,
            text::MAP_SECONDS_TOTAL,
        ]
        .contains(&label)
}

/// The rows of the comparison table: label and value, as `driftless compare` prints them.
pub fn report_rows(r: &Report) -> Vec<(&'static str, String)> {
    let pair =
        |a: f64, b: f64| format!("{} / {}", text::compare_metres(a), text::compare_metres(b));
    vec![
        (text::COMPARE_ATE, text::compare_metres(r.ate_rmse_m)),
        (
            text::COMPARE_GAP,
            pair(r.start_end_gap_m.a, r.start_end_gap_m.b),
        ),
        (
            text::COMPARE_LENGTH,
            pair(r.path_length_m.a, r.path_length_m.b),
        ),
        (text::COMPARE_MATCHED, r.matched_poses.to_string()),
    ]
}

/// Two map folders side by side.
#[derive(Debug, Clone, PartialEq)]
pub struct MapComparison {
    pub a: PathBuf,
    pub b: PathBuf,
    /// [`map_rows`] of their `map.json`.
    pub rows: Vec<(String, String, String)>,
}

/// How a value of `map.json` is shown.
#[derive(Clone, Copy)]
enum Shown {
    /// „an“ / „aus“.
    Switch,
    /// „1.234.567“.
    Count,
    /// „0,012 m“.
    Metres,
    /// „0,575“.
    Number,
    /// „27,6 s“.
    Seconds,
    /// „1/2“.
    Downscale,
}

/// Every row of the map comparison: label, path in `map.json`, how it is shown.
const MAP_ROWS: [(&str, &[&str], Shown); 24] = [
    (text::VOXEL, &["settings", "dense", "voxel"], Shown::Number),
    (
        text::GHOSTS,
        &["settings", "clean", "ghosts"],
        Shown::Switch,
    ),
    (
        text::OUTLIERS,
        &["settings", "clean", "outliers"],
        Shown::Switch,
    ),
    (
        text::THROUGH,
        &["settings", "clean", "through"],
        Shown::Switch,
    ),
    (
        text::SMOOTH,
        &["settings", "clean", "smooth", "enabled"],
        Shown::Switch,
    ),
    (
        text::HIGHLIGHTS,
        &["settings", "color", "highlights"],
        Shown::Switch,
    ),
    (
        text::VIGNETTING,
        &["settings", "color", "exposure", "vignetting"],
        Shown::Switch,
    ),
    (
        text::DOWNSCALE,
        &["settings", "color", "downscale"],
        Shown::Downscale,
    ),
    (
        text::MAP_POINTS_DENSE,
        &["steps", "dense", "stats", "points"],
        Shown::Count,
    ),
    (
        text::MAP_POINTS_KEPT,
        &["steps", "clean", "stats", "kept"],
        Shown::Count,
    ),
    (
        text::MAP_REMOVED_GHOSTS,
        &["steps", "clean", "stats", "ghosts"],
        Shown::Count,
    ),
    (
        text::MAP_REMOVED_OUTLIERS,
        &["steps", "clean", "stats", "outliers"],
        Shown::Count,
    ),
    (
        text::MAP_REMOVED_THROUGH,
        &["steps", "clean", "stats", "through"],
        Shown::Count,
    ),
    (
        text::MAP_THICKNESS_BEFORE,
        &["steps", "clean", "stats", "smooth", "thickness_before"],
        Shown::Metres,
    ),
    (
        text::MAP_THICKNESS_AFTER,
        &["steps", "clean", "stats", "smooth", "thickness_after"],
        Shown::Metres,
    ),
    (
        text::MAP_POINTS_COLORED,
        &["steps", "color", "stats", "points_colored"],
        Shown::Count,
    ),
    (
        text::MAP_CAMERA_BEFORE,
        &["steps", "color", "stats", "camera_difference_before"],
        Shown::Number,
    ),
    (
        text::MAP_CAMERA_AFTER,
        &["steps", "color", "stats", "camera_difference_after"],
        Shown::Number,
    ),
    (
        text::MAP_SEAM_BEFORE,
        &["steps", "color", "stats", "seam_jump_before"],
        Shown::Number,
    ),
    (
        text::MAP_SEAM_AFTER,
        &["steps", "color", "stats", "seam_jump_after"],
        Shown::Number,
    ),
    (
        text::MAP_SECONDS_DENSE,
        &["steps", "dense", "seconds"],
        Shown::Seconds,
    ),
    (
        text::MAP_SECONDS_CLEAN,
        &["steps", "clean", "seconds"],
        Shown::Seconds,
    ),
    (
        text::MAP_SECONDS_COLOR,
        &["steps", "color", "seconds"],
        Shown::Seconds,
    ),
    (text::MAP_SECONDS_TOTAL, &["seconds_total"], Shown::Seconds),
];

fn map_value(json: &Value, path: &[&str], shown: Shown) -> String {
    let v = path.iter().fold(json, |v, key| &v[*key]);
    let value = match shown {
        Shown::Switch => v
            .as_bool()
            .map(|on| (if on { text::MAP_ON } else { text::MAP_OFF }).to_string()),
        Shown::Count => v.as_u64().map(count),
        Shown::Metres => v.as_f64().map(text::compare_metres),
        Shown::Number => v.as_f64().map(|x| text::decimal(x, 3)),
        Shown::Seconds => v.as_f64().map(|s| format!("{} s", text::decimal(s, 1))),
        Shown::Downscale => v.as_u64().map(|d| {
            text::DOWNSCALE_CHOICES
                .iter()
                .find(|(c, _)| u64::from(*c) == d)
                .map_or_else(|| format!("1/{d}"), |(_, l)| l.to_string())
        }),
    };
    value.unwrap_or_else(|| text::MISSING.to_string())
}

/// The settings and numbers of two `map.json` side by side: label (German), value of A,
/// value of B; a value that is missing (a step that did not run, an older file) is „–“.
pub fn map_rows(a: &Value, b: &Value) -> Vec<(String, String, String)> {
    MAP_ROWS
        .iter()
        .map(|(label, path, shown)| {
            (
                label.to_string(),
                map_value(a, path, *shown),
                map_value(b, path, *shown),
            )
        })
        .collect()
}

fn read_map(dir: &Path) -> Result<Value, StepError> {
    let file = dir.join("map.json");
    let text = std::fs::read_to_string(&file)
        .map_err(|e| StepError::Failed(format!("{} nicht lesbar: {e}", file.display())))?;
    serde_json::from_str(&text)
        .map_err(|e| StepError::Failed(format!("{} ungültig: {e}", file.display())))
}

/// Reads the `map.json` of the map folders `a` and `b` and puts them side by side.
pub fn compare_maps(a: &Path, b: &Path) -> Result<MapComparison, StepError> {
    let (ja, jb) = (read_map(a)?, read_map(b)?);
    Ok(MapComparison {
        a: a.to_path_buf(),
        b: b.to_path_buf(),
        rows: map_rows(&ja, &jb),
    })
}

/// What the comparison panels show.
#[derive(Debug, Clone, Default)]
pub struct CompareState {
    /// The run the choice was preset for (the run drawn in the view).
    pub run: Option<PathBuf>,
    pub a: Option<TrajRef>,
    pub b: Option<TrajRef>,
    /// The last comparison (or why it failed); `None` while it is being computed.
    pub result: Option<Result<Comparison, String>>,
    pub map_a: Option<PathBuf>,
    pub map_b: Option<PathBuf>,
    /// The last map comparison; `None` while it is being read.
    pub maps: Option<Result<MapComparison, String>>,
}

impl CompareState {
    /// The comparison of the chosen A and B, once computed.
    pub fn current(&self) -> Option<&Comparison> {
        match &self.result {
            Some(Ok(c)) if Some(&c.a) == self.a.as_ref() && Some(&c.b) == self.b.as_ref() => {
                Some(c)
            }
            _ => None,
        }
    }
}

/// A choice among `options`; the new one when it was changed.
pub(crate) fn choose<T: Clone + PartialEq>(
    ui: &mut Ui,
    id: &str,
    label: &str,
    current: Option<&T>,
    options: &[(T, String)],
) -> Option<T> {
    let mut picked = None;
    ui.horizontal(|ui| {
        let shown = ui.label(label);
        let text = current
            .and_then(|c| options.iter().find(|(o, _)| o == c))
            .map_or(text::CHOOSE.to_string(), |(_, l)| l.clone());
        egui::ComboBox::from_id_salt(id)
            .selected_text(text)
            .truncate()
            .show_ui(ui, |ui| {
                for (o, l) in options {
                    if ui.selectable_label(current == Some(o), l).clicked() && current != Some(o) {
                        picked = Some(o.clone());
                    }
                }
            })
            .response
            .labelled_by(shown.id);
    });
    picked
}

fn error_label(ui: &mut Ui, message: &str) {
    let color = ui.visuals().error_fg_color;
    ui.label(RichText::new(message).color(color));
}

/// The trajectory comparison of the scan of the selected run (without its heading).
pub fn show(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let (scan, run) = state.selected_run()?;
    let c = &state.compare;
    let mut action = None;
    ui.weak(text::COMPARE_HINT);
    let options: Vec<(TrajRef, String)> = refs_of_scan(scan)
        .into_iter()
        .map(|r| {
            let l = r.label();
            (r, l)
        })
        .collect();
    let a = choose(ui, "vergleich_a", text::COMPARE_A, c.a.as_ref(), &options);
    let b = choose(ui, "vergleich_b", text::COMPARE_B, c.b.as_ref(), &options);
    if a.is_some() || b.is_some() {
        if let (Some(a), Some(b)) = (a.or(c.a.clone()), b.or(c.b.clone())) {
            action = Some(Action::Compare { a, b });
        }
    }
    let (Some(_), Some(cb)) = (&c.a, &c.b) else {
        return action;
    };
    match (&c.result, c.current()) {
        (_, Some(done)) => {
            egui::Grid::new("vergleich")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for (label, value) in report_rows(&done.report) {
                        ui.label(label);
                        ui.label(value);
                        ui.end_row();
                    }
                });
            if cb.run() != run.dir {
                ui.weak(text::compare_not_drawn(&run_label(cb.run())));
            }
        }
        (Some(Err(e)), None) => error_label(ui, e),
        _ => {
            ui.weak(text::COMPARING);
        }
    }
    action
}

/// The map folders of the scan with their names („SLAM 001 · Karte 002“).
pub fn maps_of_scan(scan: &ScanNode) -> Vec<(PathBuf, String)> {
    scan.runs
        .iter()
        .flat_map(|r| {
            r.maps
                .iter()
                .map(|m| (m.dir.clone(), format!("{} · {}", r.label, m.label)))
        })
        .collect()
}

/// The map comparison of the scan of the selected run (without its heading).
pub fn show_maps(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let (scan, _) = state.selected_run()?;
    let c = &state.compare;
    let options = maps_of_scan(scan);
    if options.len() < 2 {
        ui.weak(text::MAPS_NEED_TWO);
        return None;
    }
    let mut action = None;
    let a = choose(ui, "karte_a", text::MAP_A, c.map_a.as_ref(), &options);
    let b = choose(ui, "karte_b", text::MAP_B, c.map_b.as_ref(), &options);
    if a.is_some() || b.is_some() {
        if let (Some(a), Some(b)) = (a.or(c.map_a.clone()), b.or(c.map_b.clone())) {
            action = Some(Action::CompareMaps { a, b });
        }
    }
    let (Some(ma), Some(mb)) = (&c.map_a, &c.map_b) else {
        return action;
    };
    match &c.maps {
        Some(Ok(m)) if m.a == *ma && m.b == *mb => {
            let warn = ui.visuals().warn_fg_color;
            egui::Grid::new("karten_vergleich")
                .num_columns(3)
                .striped(true)
                .show(ui, |ui| {
                    for head in text::MAP_COLUMNS {
                        ui.strong(head);
                    }
                    ui.end_row();
                    for (label, va, vb) in &m.rows {
                        let differs = highlighted(label, va, vb);
                        let cell = |s: &str| {
                            let t = RichText::new(s);
                            if differs {
                                t.color(warn).strong()
                            } else {
                                t
                            }
                        };
                        ui.label(cell(label));
                        ui.label(cell(va));
                        ui.label(cell(vb));
                        ui.end_row();
                    }
                });
        }
        Some(Err(e)) => error_label(ui, e),
        _ => {
            ui.weak(text::MAPS_COMPARING);
        }
    }
    action
}

/// Why the selected state cannot be restored now: a step runs, a change of the corrections
/// is being written, the scan cannot be read or the run has no `submaps.bin`.
fn restore_blocked(state: &AppState, scan: &ScanNode, run: &RunInfo) -> Option<String> {
    if let Some(job) = &state.job {
        return Some(text::busy(&job.label));
    }
    if let crate::project::ScanState::Missing(why) = &scan.state {
        return Some(why.clone());
    }
    super::correct::writing(state).or_else(|| (!run.has_cache).then(|| text::NO_CACHE.into()))
}

/// The selected state of a run's history: what it is, and the buttons „Vergleichen mit
/// aktuell“ and „Diesen Stand wiederherstellen“. Nothing unless a state is selected.
pub fn show_state(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let Selection::History { run: dir, nr } = &state.selection else {
        return None;
    };
    let (scan, run) = state.selected_run()?;
    let entry = run.history.iter().find(|e| e.nr == *nr)?;
    let mut action = None;
    ui.strong(text::state_name(*nr));
    ui.label(text::history_entry(
        entry,
        state.history_gaps.get(&entry.dir).copied(),
    ));
    if ui.button(text::COMPARE_WITH_CURRENT).clicked() {
        action = Some(Action::Compare {
            a: TrajRef::History {
                run: dir.clone(),
                nr: *nr,
            },
            b: TrajRef::Current(dir.clone()),
        });
    }
    let blocked = restore_blocked(state, scan, run);
    let button = ui
        .add_enabled(blocked.is_none(), egui::Button::new(text::RESTORE_STATE))
        .on_hover_text(text::RESTORE_HINT);
    if button.clicked() {
        action = Some(Action::RestoreHistory {
            run: dir.clone(),
            nr: *nr,
        });
    }
    if let Some(why) = blocked {
        button.on_disabled_hover_text(why);
    }
    action
}
