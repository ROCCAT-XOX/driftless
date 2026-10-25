//! Step 1 „Daten“, as in the design: the heading with the project and its number of scans,
//! a card per scan with its status (neu, wartet, rechnet, fertig ✓, veraltet), „+ Scan
//! hinzufügen“, the precision as a three-way switch (it sets only the voxel size of the map),
//! the progress of the running job, and at the bottom the main button: „Alle rechnen“ queues
//! every new or stale scan ([`Action::ComputeAllMaps`]), „Rechnen“ only the selected scan.
//!
//! The runs and maps of a scan unfold below its card while it, or something of it, is
//! selected. Everything else of the former panel lies under „▸ Erweitert“: the map switches
//! (with the voxel size as a number), the layers „Teilkarten“ and „Verworfene Kandidaten“, the
//! steps of the selection („Prüfen“, „SLAM rechnen“, „Neu rechnen (aus Teilkarten)“, „Karte
//! rechnen“) with their state, the corrections ([`correct`]), the comparisons ([`compare`]),
//! what the steps are computed with („Eigenschaften“); the clicked point shows in the view.
//! Every step button is locked while a step runs and while the scan cannot be read; selecting
//! and viewing stay possible.

use super::{
    badge, below_lines, blocked, card, error_label, fraction, heading, hint, job_progress,
    main_button, no_project, pinned, quiet_button, section, segments, state_label, step_row, tree,
    StepState,
};
use crate::project::{MapInfo, RunInfo, ScanState};
use crate::ui::status::count;
use crate::ui::theme::{icon, DARK};
use crate::ui::{compare, correct, text, Action, AppState, Queued, ScanNode, Selection, Step};
use driftless_steps::history::HistoryKind;
use driftless_steps::map::{MapSettings, MapStep};
use driftless_steps::slam::SlamSettings;
use eframe::egui::{self, RichText, Ui};

/// The LiDAR pose a switched-on pose field starts with: no offset, no rotation.
const IDENTITY_POSE: [f64; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];

/// The precision of the map: only its voxel size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Precision {
    Schnell,
    /// Today's default of `driftless map`.
    Genau,
    Maximal,
    /// A voxel size of no level, set under „Erweitert“.
    Eigene,
}

/// The levels of the switch with their voxel size (m) and label.
const LEVELS: [(Precision, f64, &str); 3] = [
    (Precision::Schnell, 0.02, text::PRECISION_FAST),
    (Precision::Genau, 0.01, text::PRECISION_EXACT),
    (Precision::Maximal, 0.005, text::PRECISION_MAX),
];

/// The voxel size of a level; `None` for [`Precision::Eigene`].
pub fn voxel_of(p: Precision) -> Option<f64> {
    LEVELS.iter().find(|(l, ..)| *l == p).map(|(_, v, _)| *v)
}

/// The level of a voxel size, [`Precision::Eigene`] if it is none.
pub fn precision_of(voxel: f64) -> Precision {
    LEVELS
        .iter()
        .find(|(_, v, _)| (v - voxel).abs() < 1e-9)
        .map_or(Precision::Eigene, |(p, ..)| *p)
}

/// Where a scan stands, as its card shows it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScanStatus {
    /// No map yet.
    New,
    /// Waits in the queue of „Alle rechnen“.
    Queued,
    /// A step of it runs, with its progress (0…1) if the step knows its total.
    Running(Option<f32>),
    /// Its newest run with a result has a map that still fits.
    Done,
    /// It has maps, but none fits its newest run any more.
    Stale,
    /// Its folder cannot be read.
    Missing,
}

/// The status of `scan`: a running step or the queue first, then its maps. A map counts as
/// fitting until the background check ([`AppState::map_status`]) says it is stale.
pub fn scan_status(state: &AppState, scan: &ScanNode) -> ScanStatus {
    if let ScanState::Missing(_) = scan.state {
        return ScanStatus::Missing;
    }
    let running = state.running.as_ref().and_then(|a| state.scan_of(a));
    if running.is_some_and(|s| s.id == scan.id) {
        return ScanStatus::Running(state.job.as_ref().and_then(fraction));
    }
    let path = state.project.as_ref().and_then(|p| p.scan(&scan.id));
    let queued = |e: &crate::project::ScanEntry| {
        state
            .queue
            .iter()
            .any(|q| matches!(q, Queued::Scan(p) if *p == e.path))
    };
    if path.is_some_and(queued) {
        return ScanStatus::Queued;
    }
    let Some(newest) = scan.runs.iter().rev().find(|r| r.has_result) else {
        return ScanStatus::New;
    };
    let maps = mapped(newest);
    let fits = |m: &&MapInfo| !state.map_status.get(&m.dir).is_some_and(|s| s.stale);
    if maps.iter().any(fits) {
        ScanStatus::Done
    } else if !maps.is_empty() || scan.runs.iter().any(|r| !mapped(r).is_empty()) {
        ScanStatus::Stale
    } else {
        ScanStatus::New
    }
}

/// The maps of `run` with recorded switches (a folder without `map.json` is no map yet).
fn mapped(run: &RunInfo) -> Vec<&MapInfo> {
    run.maps.iter().filter(|m| m.settings.is_some()).collect()
}

/// What the steps act on.
enum Target {
    None,
    Scan(ScanNode),
    /// A run, and the map if one of it is selected.
    Run {
        scan: ScanNode,
        run: Box<RunInfo>,
        map: Option<MapInfo>,
    },
}

fn target(state: &AppState) -> Target {
    if let Selection::Scan(id) = &state.selection {
        return state
            .scan_node(id)
            .map_or(Target::None, |s| Target::Scan(s.clone()));
    }
    let Some((scan, run)) = state.selected_run() else {
        return Target::None;
    };
    let map = state.selected_map().cloned();
    Target::Run {
        scan: scan.clone(),
        run: Box::new(run.clone()),
        map,
    }
}

/// Opens or closes „Erweitert“ (as its toggle does; for tests and pictures).
pub fn set_advanced(ctx: &egui::Context, open: bool) {
    super::set_advanced(ctx, Step::Daten, open);
}

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_DATA);
    let Some(project) = &state.project else {
        no_project(ui, &mut action);
        return action;
    };
    ui.weak(text::project_line(&project.name, state.scans.len()))
        .on_hover_text(project.dir.display().to_string());
    if state.scans.is_empty() {
        hint(ui, text::EMPTY_NO_SCANS);
        if main_button(ui, text::ADD_SCAN, None, text::COMPUTE_HINT) {
            action = Some(Action::PickAddScan);
        }
        return action;
    }
    // the main button and „Erweitert“ stay at the bottom, the rest scrolls above them
    pinned(
        ui,
        Step::Daten,
        &mut (state, &mut action),
        |ui, (state, action), advanced| {
            ui.add_space(4.0);
            for scan in &state.scans {
                scan_card(ui, state, scan, action);
            }
            if add_scan_link(ui) {
                **action = Some(Action::PickAddScan);
            }
            ui.add_space(6.0);
            precision_switch(ui, state);
            job_progress(ui, state);
            if advanced {
                ui.separator();
                if let Some(a) = advanced_content(ui, state) {
                    **action = Some(a);
                }
            }
        },
        |ui, (state, action)| compute_buttons(ui, state, action),
    );
    action
}

/// The card of a scan: its name and status; a click selects it (again: deselects). While it
/// or something of it is selected, its runs and maps unfold below it.
fn scan_card(ui: &mut Ui, state: &AppState, scan: &ScanNode, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Scan(scan.id.clone());
    let status = scan_status(state, scan);
    let folder = state
        .project
        .as_ref()
        .and_then(|p| p.scan(&scan.id))
        .map(|e| e.path.display().to_string())
        .unwrap_or_default();
    let (name, hover) = match &scan.state {
        ScanState::Ready => (
            RichText::new(&scan.name),
            format!("{}\n{folder}", scan.name),
        ),
        ScanState::Missing(reason) => {
            let label = text::scan_missing(&scan.name);
            let hover = format!("{label}\n{reason}");
            (RichText::new(label).color(DARK.text_dim), hover)
        }
    };
    let r = card(ui, ("scan", &scan.id), selected, name, &hover, |ui| {
        status_label(ui, status);
    });
    if r.clicked() {
        let selection = if selected {
            Selection::None
        } else {
            Selection::Scan(scan.id.clone())
        };
        *action = Some(Action::Select(selection));
    }
    let open = selected || state.selected_run().is_some_and(|(s, _)| s.id == scan.id);
    if open {
        ui.indent(("scan_laeufe", &scan.id), |ui| {
            if scan.runs.is_empty() {
                ui.weak(text::NO_RUNS);
            }
            for run in &scan.runs {
                tree::run_node(ui, state, run, action);
            }
            if ui.small_button(text::ADD_RUN).clicked() {
                *action = Some(Action::PickAddRun(scan.id.clone()));
            }
        });
    }
}

/// The status at the right of a card.
fn status_label(ui: &mut Ui, status: ScanStatus) {
    let (symbol, word, color) = match status {
        ScanStatus::New => (None, text::STATUS_NEW, DARK.text_dim),
        ScanStatus::Queued => (None, text::STATUS_QUEUED, DARK.text_dim),
        ScanStatus::Running(_) => (Some(icon::RUNNING), text::STATUS_RUNNING, DARK.warn),
        ScanStatus::Done => (Some(icon::CHECK), text::STATUS_DONE, DARK.ok),
        ScanStatus::Stale => (Some(icon::STALE), text::STATUS_STALE, DARK.warn),
        // the name says it („… (fehlt)“)
        ScanStatus::Missing => return,
    };
    badge(ui, symbol, word, color);
}

/// „+ Scan hinzufügen …“, quiet under the cards. True when clicked.
fn add_scan_link(ui: &mut Ui) -> bool {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        ui.label(RichText::new(icon::ADD).color(DARK.text_dim));
        let link =
            egui::Button::new(RichText::new(text::ADD_SCAN).color(DARK.text_dim)).frame(false);
        ui.add(link).clicked()
    })
    .inner
}

/// „Genauigkeit“: Schnell / Genau / Maximal, which set the voxel size of the next map; the
/// memory hint under „Maximal“, the value under „Eigene“.
fn precision_switch(ui: &mut Ui, state: &mut AppState) {
    ui.label(RichText::new(text::PRECISION).strong());
    let voxel = &mut state.map_settings.switches.voxel;
    let choices = LEVELS.iter().map(|(_, value, label)| (*value, *label));
    segments(ui, choices, voxel, |v| Some(text::precision_hover(v)));
    match precision_of(*voxel) {
        Precision::Maximal => {
            let warn = RichText::new(text::PRECISION_MAX_HINT).color(DARK.warn);
            ui.add(egui::Label::new(warn).wrap());
        }
        Precision::Eigene => {
            ui.weak(text::precision_custom(*voxel));
        }
        _ => {}
    }
}

/// The main button – „Rechnen“ with a scan selected, else „Alle rechnen“ – and, with a scan
/// selected, „Alle rechnen“ quietly below it. Locked while a step runs, while the scan cannot
/// be read, for invalid map settings, and with nothing to compute; the reason shows on hover,
/// and below the button unless the field shows it already (the progress of the running step,
/// „(fehlt)“ on the card).
fn compute_buttons(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let busy = state.job.as_ref().map(|j| text::busy(&j.label));
    let invalid = state.map_settings.check().err().map(|e| e.to_string());
    // until the maps are checked a stale one counts as fitting: „Alle rechnen“ then waits
    // for the check instead of saying that all is done
    let pending = state.map_status_pending;
    let nothing =
        (!pending && state.scans_to_compute().is_empty()).then(|| text::ALL_COMPUTED.to_string());
    let selected = match &state.selection {
        Selection::Scan(id) => state.scan_node(id),
        _ => None,
    };
    // colouring a scan without colour fails: lock before the click, not at that scan of the
    // queue
    let no_color = |scan: &ScanNode| {
        (state.map_settings.step == MapStep::Color)
            .then(|| scan.no_color.clone())
            .flatten()
    };
    let to_compute =
        |s: &&ScanNode| matches!(scan_status(state, s), ScanStatus::New | ScanStatus::Stale);
    let all_invalid = invalid
        .clone()
        .or_else(|| state.scans.iter().filter(to_compute).find_map(no_color));
    let all_locked = busy.clone().or(all_invalid.clone()).or(nothing.clone());
    let below = match selected {
        Some(scan) => {
            let invalid = invalid.clone().or_else(|| no_color(scan));
            let locked = blocked(state, scan).or(invalid.clone());
            if main_button(ui, text::COMPUTE_ONE, locked, text::COMPUTE_HINT) {
                *action = Some(Action::ComputeMap(scan.id.clone()));
            }
            if quiet_button(ui, text::COMPUTE_ALL, all_locked) {
                *action = Some(Action::ComputeAllMaps);
            }
            busy.is_none()
                .then_some(invalid)
                .flatten()
                .map(|e| text::cannot_compute(&e))
        }
        None => {
            if main_button(ui, text::COMPUTE_ALL, all_locked, text::COMPUTE_HINT) {
                *action = Some(Action::ComputeAllMaps);
            }
            match (busy, all_invalid) {
                (Some(_), _) => None,
                (None, Some(e)) => Some(text::cannot_compute(&e)),
                (None, None) if pending => Some(text::MAP_STATUS_PENDING.to_string()),
                (None, None) => nothing.clone(),
            }
        }
    };
    let mut lines: Vec<String> = below.into_iter().collect();
    // all done, but with another precision than the one chosen now: how to get it
    if selected.is_none() && nothing.is_some() && other_precision(state) {
        lines.push(text::RECOMPUTE_HINT.to_string());
    }
    below_lines(ui, lines);
}

/// Whether a computed scan has no fitting map of the voxel size chosen now.
fn other_precision(state: &AppState) -> bool {
    let voxel = state.map_settings.switches.voxel;
    state.scans.iter().any(|scan| {
        scan_status(state, scan) == ScanStatus::Done
            && scan
                .runs
                .iter()
                .rev()
                .find(|r| r.has_result)
                .is_some_and(|run| {
                    !mapped(run).iter().any(|m| {
                        m.settings
                            .as_ref()
                            .is_some_and(|s| (s.voxel - voxel).abs() < 1e-9)
                    })
                })
    })
}

/// Under „Erweitert“: the map switches, two layers, and for the selection its steps,
/// corrections, comparisons and settings (the clicked point shows in the view's info).
fn advanced_content(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    let no_color = match target(state) {
        Target::Scan(scan) | Target::Run { scan, .. } => scan.no_color,
        Target::None => None,
    };
    section(ui, text::MAP_SETTINGS, |ui| {
        map_settings_edit(ui, &mut state.map_settings, no_color.as_deref());
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(text::LAYERS);
        ui.checkbox(&mut state.view.show_submaps, text::LAYER_SUBMAPS);
        ui.checkbox(&mut state.view.show_rejected, text::LAYER_REJECTED);
    });
    ui.separator();
    if let Some(a) = steps_and_properties(ui, state) {
        action = Some(a);
    }
    action
}

/// The steps of the selection, its corrections and comparisons, and their settings.
fn steps_and_properties(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    let target = target(state);
    ui.strong(text::PANEL_STEPS);
    match &target {
        Target::None => {
            ui.weak(text::SELECT_FOR_STEPS);
        }
        Target::Scan(scan) => scan_steps(ui, state, scan, &mut action),
        Target::Run { scan, run, map } => {
            run_steps(ui, state, scan, run, map.as_ref(), &mut action);
            if matches!(state.selection, Selection::History { .. }) {
                ui.separator();
                section(ui, text::HISTORY, |ui| {
                    if let Some(a) = compare::show_state(ui, state) {
                        action = Some(a);
                    }
                });
            }
            ui.separator();
            section(ui, text::CORRECTIONS, |ui| {
                if let Some(a) = correct::show(ui, state) {
                    action = Some(a);
                }
            });
            ui.separator();
            section(ui, text::COMPARE, |ui| {
                if let Some(a) = compare::show(ui, state) {
                    action = Some(a);
                }
            });
            // only with something to compare
            if compare::maps_of_scan(scan).len() >= 2 {
                ui.separator();
                section(ui, text::MAPS_COMPARE, |ui| {
                    if let Some(a) = compare::show_maps(ui, state) {
                        action = Some(a);
                    }
                });
            }
        }
    }
    ui.separator();
    section(ui, text::PANEL_PROPERTIES, |ui| match &target {
        Target::None => {}
        Target::Scan(scan) => {
            slam_settings_edit(ui, &mut state.slam_settings);
            inspect_table(ui, state, &scan.id);
        }
        Target::Run { run, map, .. } => {
            slam_settings_shown(ui, &run.settings);
            if let Some(map) = map {
                ui.separator();
                map_switches_shown(ui, map, &mut state.map_settings);
            }
        }
    });
    action
}

fn scan_steps(ui: &mut Ui, state: &AppState, scan: &ScanNode, action: &mut Option<Action>) {
    let blocked = blocked(state, scan);
    let inspected = match &state.inspect {
        Some(r) if r.scan == scan.id => StepState::Done(Some(r.seconds)),
        _ => StepState::NotComputed,
    };
    if step_row(ui, text::STEP_INSPECT, blocked.clone(), inspected) {
        *action = Some(Action::Inspect(scan.id.clone()));
    }
    // the newest run with a result, with the runtime of its last full run
    let slam = scan
        .runs
        .iter()
        .rev()
        .find(|r| r.has_result)
        .map_or(StepState::NotComputed, |r| {
            let full = r.history.iter().rev().find(|e| e.kind == HistoryKind::Full);
            StepState::Done(full.map(|e| e.seconds))
        });
    let invalid = state.slam_settings.config().err().map(|e| e.to_string());
    if step_row(ui, text::STEP_SLAM, blocked.or(invalid), slam) {
        *action = Some(Action::RunSlam(scan.id.clone()));
    }
}

fn run_steps(
    ui: &mut Ui,
    state: &AppState,
    scan: &ScanNode,
    run: &RunInfo,
    map: Option<&MapInfo>,
    action: &mut Option<Action>,
) {
    let blocked = blocked(state, scan);
    let reused = run
        .history
        .iter()
        .rev()
        .find(|e| e.kind == HistoryKind::Reuse)
        .map_or(StepState::NotReused, |e| StepState::Done(Some(e.seconds)));
    let no_cache = (!run.has_cache).then(|| text::NO_CACHE.to_string());
    let writing = correct::writing(state);
    if step_row(
        ui,
        text::STEP_REUSE,
        blocked.clone().or(writing).or(no_cache),
        reused,
    ) {
        *action = Some(Action::Reuse(run.dir.clone()));
    }
    let no_color = (state.map_settings.step == MapStep::Color)
        .then(|| scan.no_color.clone())
        .flatten();
    let invalid = state
        .map_settings
        .check()
        .err()
        .map(|e| e.to_string())
        .or(no_color);
    let mapped = map_state(state, run, &state.map_settings);
    if step_row(ui, text::STEP_MAP, blocked.clone().or(invalid), mapped) {
        *action = Some(Action::RunMap(run.dir.clone()));
    }
    if let Some(map) = map {
        ui.horizontal(|ui| {
            ui.label(text::THIS_MAP);
            state_label(ui, own_map_state(state, map));
        });
    }
}

/// The state of a map of `run` with `settings`: done if a folder with these switches still
/// fits the run, stale if all such folders no longer fit.
fn map_state(state: &AppState, run: &RunInfo, settings: &MapSettings) -> StepState {
    let mut same = run
        .maps
        .iter()
        .rev()
        .filter(|m| m.settings.as_ref() == Some(&settings.switches))
        .peekable();
    if same.peek().is_none() {
        return StepState::NotComputed;
    }
    same.map(|m| own_map_state(state, m))
        .find(|s| *s != StepState::Stale)
        .unwrap_or(StepState::Stale)
}

fn own_map_state(state: &AppState, map: &MapInfo) -> StepState {
    match state.map_status.get(&map.dir) {
        Some(s) if s.stale => StepState::Stale,
        Some(s) if map.settings.is_some() => StepState::Done(s.seconds),
        _ if map.settings.is_some() => StepState::Done(None),
        _ => StepState::NotComputed,
    }
}

/// The settings of the next SLAM run.
fn slam_settings_edit(ui: &mut Ui, settings: &mut SlamSettings) {
    ui.strong(text::SLAM_SETTINGS);
    ui.weak(text::SLAM_SETTINGS_NEW);
    let mut with_pose = settings.lidar_to_imu.is_some();
    if ui.checkbox(&mut with_pose, text::LIDAR_POSE).changed() {
        settings.lidar_to_imu = with_pose.then_some(IDENTITY_POSE);
    }
    if let Some(pose) = &mut settings.lidar_to_imu {
        egui::Grid::new("lidar_lage").num_columns(2).show(ui, |ui| {
            for (part, value) in text::LIDAR_POSE_PARTS.iter().zip(pose.iter_mut()) {
                ui.label(*part);
                ui.add(egui::DragValue::new(value).speed(0.001).max_decimals(6));
                ui.end_row();
            }
        });
    }
    egui::Grid::new("topics").num_columns(2).show(ui, |ui| {
        for (what, topic) in [
            (text::LIDAR_TOPIC, &mut settings.lidar_topic),
            (text::IMU_TOPIC, &mut settings.imu_topic),
        ] {
            ui.label(what);
            let mut s = topic.clone().unwrap_or_default();
            let edit = egui::TextEdit::singleline(&mut s).hint_text(text::TOPIC_HINT);
            if ui.add(edit).changed() {
                *topic = (!s.trim().is_empty()).then(|| s.trim().to_string());
            }
            ui.end_row();
        }
    });
    if let Err(e) = settings.config() {
        error_label(ui, &e.to_string());
    }
}

/// The settings a run was computed with (they belong to its folder and are not changed).
fn slam_settings_shown(ui: &mut Ui, settings: &SlamSettings) {
    ui.strong(text::SLAM_SETTINGS);
    ui.weak(text::SLAM_SETTINGS_OF_RUN);
    match &settings.lidar_to_imu {
        None => {
            ui.label(text::LIDAR_POSE_NONE);
        }
        Some(pose) => {
            for (part, value) in text::LIDAR_POSE_PARTS.iter().zip(pose) {
                ui.label(text::pose_part(part, *value));
            }
        }
    }
    ui.label(text::topic_of_run(
        text::LIDAR_TOPIC,
        settings.lidar_topic.as_deref(),
    ));
    ui.label(text::topic_of_run(
        text::IMU_TOPIC,
        settings.imu_topic.as_deref(),
    ));
}

/// The switches recorded for a selected map, with a button that takes them over for the next
/// map (the step stays).
fn map_switches_shown(ui: &mut Ui, map: &MapInfo, settings: &mut MapSettings) {
    ui.strong(text::MAP_SWITCHES);
    let Some(s) = &map.settings else {
        ui.weak(text::MAP_SWITCHES_UNKNOWN);
        return;
    };
    for (label, on) in [
        (text::GHOSTS, s.ghosts),
        (text::OUTLIERS, s.outliers),
        (text::THROUGH, s.through),
        (text::SMOOTH, s.smooth),
        (text::HIGHLIGHTS, s.highlights),
        (text::VIGNETTING, s.vignetting),
    ] {
        ui.label(text::switch_value(label, on));
    }
    ui.label(text::voxel_value(s.voxel));
    ui.label(text::downscale_value(s.downscale));
    if ui.button(text::USE_AS_TEMPLATE).clicked() {
        settings.switches = s.clone();
    }
}

/// Step and switches of the next map, as `driftless map` takes them. `no_color`: why the scan
/// cannot be coloured (step „Einfärben“ is then locked, „Alles“ skips colouring).
fn map_settings_edit(ui: &mut Ui, settings: &mut MapSettings, no_color: Option<&str>) {
    ui.strong(text::MAP_SETTINGS);
    ui.horizontal_wrapped(|ui| {
        ui.label(text::MAP_STEP);
        for (step, label) in [
            (MapStep::Dense, text::STEP_DENSE),
            (MapStep::Clean, text::STEP_CLEAN),
            (MapStep::Color, text::STEP_COLOR),
            (MapStep::All, text::STEP_ALL),
        ] {
            let locked = (step == MapStep::Color).then_some(no_color).flatten();
            let r = ui.add_enabled(
                locked.is_none(),
                egui::Button::selectable(settings.step == step, label),
            );
            if r.clicked() {
                settings.step = step;
            }
            if let Some(why) = locked {
                r.on_disabled_hover_text(why);
            }
        }
    });
    if let (MapStep::All, Some(why)) = (settings.step, no_color) {
        let warn = ui.visuals().warn_fg_color;
        ui.label(RichText::new(text::color_skipped(why)).color(warn));
    }
    let s = &mut settings.switches;
    ui.checkbox(&mut s.ghosts, text::GHOSTS);
    ui.checkbox(&mut s.outliers, text::OUTLIERS);
    ui.checkbox(&mut s.through, text::THROUGH);
    ui.checkbox(&mut s.smooth, text::SMOOTH);
    ui.checkbox(&mut s.highlights, text::HIGHLIGHTS);
    ui.checkbox(&mut s.vignetting, text::VIGNETTING);
    ui.horizontal(|ui| {
        ui.label(text::VOXEL);
        ui.add(
            egui::DragValue::new(&mut s.voxel)
                .speed(0.001)
                .max_decimals(3),
        );
    });
    ui.horizontal(|ui| {
        ui.label(text::DOWNSCALE);
        for (value, label) in text::DOWNSCALE_CHOICES {
            ui.selectable_value(&mut s.downscale, value, label);
        }
    });
    if let Err(e) = settings.check() {
        error_label(ui, &e.to_string());
    }
}

/// The last check of the scan `id`, if it is of this scan.
fn inspect_table(ui: &mut Ui, state: &AppState, id: &str) {
    let Some(result) = state.inspect.as_ref().filter(|r| r.scan == id) else {
        return;
    };
    let r = &result.report;
    ui.separator();
    ui.strong(text::INSPECT_RESULT);
    egui::Grid::new("pruefbericht")
        .num_columns(2)
        .show(ui, |ui| {
            ui.label(text::DEVICE);
            let device = r.device.as_ref();
            ui.label(text::device(
                device.and_then(|d| d.model.as_deref()),
                device.and_then(|d| d.sn.as_deref()),
            ));
            ui.end_row();
            ui.label(text::DURATION);
            ui.label(text::duration(r.duration_s));
            ui.end_row();
        });
    ui.label(text::TOPICS);
    egui::Grid::new("topics_bericht")
        .num_columns(4)
        .striped(true)
        .show(ui, |ui| {
            for head in [
                text::TOPIC,
                text::TOPIC_TYPE,
                text::TOPIC_COUNT,
                text::TOPIC_RATE,
            ] {
                ui.strong(head);
            }
            ui.end_row();
            for t in &r.topics {
                ui.label(&t.topic);
                ui.label(&t.msg_type);
                ui.label(count(t.count));
                ui.label(text::rate(t.rate_hz));
                ui.end_row();
            }
        });
    ui.label(text::PROBLEMS);
    if r.problems.is_empty() {
        ui.weak(text::NO_PROBLEMS);
    }
    for p in &r.problems {
        let warn = ui.visuals().warn_fg_color;
        ui.label(RichText::new(p).color(warn));
    }
}
