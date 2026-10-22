//! The right panel: the steps of the selection with their state („Schritte“), and below
//! them what the steps are computed with („Eigenschaften“).
//!
//! A scan offers „Prüfen“ and „SLAM rechnen“ (a new run with the SLAM settings shown here);
//! a run, or a map, cloud or state of it, offers „Neu rechnen (aus Teilkarten)“ and „Karte
//! rechnen“ (with the map settings, every switch of `driftless map`). Every step button is
//! locked while a step runs and while the scan cannot be read; selecting and viewing stay
//! possible.
//!
//! A run also shows its manual corrections ([`correct`]), a selected state of its history and
//! the comparisons of trajectories and maps of its scan ([`compare`]). Below them, always:
//! how the 3D view draws („Ansicht“) and the point clicked in it.

use super::status::count;
use super::view3d::{color_modes, effective_color_mode, PointInfo};
use super::{compare, correct};
use super::{text, Action, AppState, CloudSummary, ScanNode, Selection, ViewSettings};
use crate::camera::CameraMode;
use crate::project::{MapInfo, RunInfo, ScanState};
use crate::render::ColorMode;
use driftless_steps::history::HistoryKind;
use driftless_steps::map::{MapSettings, MapStep};
use driftless_steps::slam::SlamSettings;
use eframe::egui::{self, RichText, Ui};

/// The LiDAR pose a switched-on pose field starts with: no offset, no rotation.
const IDENTITY_POSE: [f64; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];

/// How far a step is.
#[derive(Debug, Clone, Copy, PartialEq)]
enum StepState {
    NotComputed,
    /// A run that was never recomputed from its submaps.
    NotReused,
    /// With its runtime (s), if known.
    Done(Option<f64>),
    /// There is a result, but it no longer fits the run.
    Stale,
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
    let map = match &state.selection {
        Selection::Map(dir) => run.maps.iter().find(|m| m.dir == *dir).cloned(),
        _ => None,
    };
    Target::Run {
        scan: scan.clone(),
        run: Box::new(run.clone()),
        map,
    }
}

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    // a solid bar that is always there: the panel is longer than the window
    ui.style_mut().spacing.scroll = egui::style::ScrollStyle::solid();
    egui::ScrollArea::vertical()
        .auto_shrink(false)
        .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysVisible)
        .show(ui, |ui| {
            let target = target(state);
            ui.heading(text::PANEL_STEPS);
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
            // the view first: it is used most while choosing places
            section(ui, text::VIEW_SETTINGS, |ui| view_section(ui, state));
            ui.separator();
            section(ui, text::PANEL_PROPERTIES, |ui| match &target {
                Target::None => {}
                Target::Scan(scan) => {
                    slam_settings_edit(ui, &mut state.slam_settings);
                    inspect_table(ui, state, &scan.id);
                }
                Target::Run { scan, run, map } => {
                    slam_settings_shown(ui, &run.settings);
                    if let Some(map) = map {
                        ui.separator();
                        map_switches_shown(ui, map, &mut state.map_settings);
                    }
                    ui.separator();
                    map_settings_edit(ui, &mut state.map_settings, scan.no_color.as_deref());
                }
            });
        });
    action
}

/// A section that can be folded away (open at first; egui keeps the choice).
fn section(ui: &mut Ui, title: &str, body: impl FnOnce(&mut Ui)) {
    egui::CollapsingHeader::new(RichText::new(title).heading())
        .id_salt(title)
        .default_open(true)
        .show(ui, body);
}

/// How the 3D view draws, the height cut, the numbers of the last frame and the point
/// clicked in it.
fn view_section(ui: &mut Ui, state: &mut AppState) {
    let has_rgb = state.cloud.as_ref().is_none_or(|c| c.has_rgb);
    view_settings_edit(ui, &mut state.view, has_rgb);
    if state.cloud.as_ref().is_some_and(|c| !c.quality_known) {
        ui.weak(text::QUALITY_NONE_HINT);
    }
    if let Some(cloud) = &state.cloud {
        cut_edit(ui, &mut state.view.cut, cloud);
    }
    if let Some(stats) = &state.view_stats {
        ui.label(text::drawn_stats(
            stats.drawn,
            stats.total,
            stats.frame_ms,
            stats.gpu,
        ));
    }
    ui.weak(text::FRAME_ALL_HINT);
    ui.separator();
    point_shown(ui, state.picked.as_ref());
}

/// How the 3D view draws. `has_rgb`: whether the loaded cloud has colours (offers the
/// colour mode „Farbe“).
fn view_settings_edit(ui: &mut Ui, view: &mut ViewSettings, has_rgb: bool) {
    let shown = effective_color_mode(view.color_mode, has_rgb);
    ui.horizontal_wrapped(|ui| {
        ui.label(text::COLOR_MODE);
        for mode in color_modes(has_rgb) {
            let label = match mode {
                ColorMode::Rgb => text::MODE_RGB,
                ColorMode::Intensity => text::MODE_INTENSITY,
                ColorMode::Quality => text::MODE_QUALITY,
            };
            if ui.selectable_label(shown == mode, label).clicked() {
                view.color_mode = mode;
            }
        }
    });
    ui.horizontal(|ui| {
        ui.label(text::POINT_SIZE);
        ui.add(
            egui::Slider::new(&mut view.point_px, 1.0..=10.0)
                .step_by(0.5)
                .custom_formatter(|v, _| text::decimal(v, 1))
                .custom_parser(parse_decimal),
        );
        ui.checkbox(&mut view.round, text::ROUND_POINTS);
    });
    ui.horizontal(|ui| {
        ui.label(text::BUDGET);
        let (lo, hi) = (*text::BUDGET_RANGE.start(), *text::BUDGET_RANGE.end());
        let mut millions = view.budget as f64 / 1e6;
        let slider = egui::Slider::new(&mut millions, lo as f64 / 1e6..=hi as f64 / 1e6)
            .step_by(0.5)
            .custom_formatter(|v, _| format!("{} Mio.", text::decimal(v, 1)))
            .custom_parser(|s| parse_decimal(s.trim_end_matches("Mio.").trim()));
        if ui.add(slider).changed() {
            view.budget = ((millions * 1e6).round() as usize).clamp(lo, hi);
        }
    });
    ui.horizontal(|ui| {
        ui.label(text::CAMERA_MODE);
        ui.selectable_value(&mut view.camera_mode, CameraMode::Orbit, text::ORBIT);
        ui.selectable_value(&mut view.camera_mode, CameraMode::Fly, text::FLY);
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(text::LAYERS);
        ui.checkbox(&mut view.show_trajectory, text::LAYER_TRAJECTORY);
        ui.checkbox(&mut view.show_odometry, text::LAYER_ODOMETRY);
        ui.checkbox(&mut view.show_loops, text::LAYER_LOOPS);
        ui.checkbox(&mut view.show_rejected, text::LAYER_REJECTED);
        ui.checkbox(&mut view.show_submaps, text::LAYER_SUBMAPS);
        ui.checkbox(&mut view.show_compare, text::LAYER_COMPARE);
    });
}

/// The height cut: z min / z max over the cloud's z range, „Decke ausblenden“ (z max just
/// below the estimated ceiling) and „Zurücksetzen“.
fn cut_edit(ui: &mut Ui, cut: &mut Option<[f32; 2]>, cloud: &CloudSummary) {
    let [lo, hi] = cloud.z_range;
    ui.strong(text::CUT).on_hover_text(text::CUT_HINT);
    let mut range = cut.unwrap_or([lo, hi]);
    let mut changed = false;
    for (label, k) in [(text::CUT_Z_MIN, 0), (text::CUT_Z_MAX, 1)] {
        ui.horizontal(|ui| {
            ui.label(label);
            let slider = egui::Slider::new(&mut range[k], lo..=hi.max(lo))
                .step_by(0.05)
                .custom_formatter(|v, _| text::height(v as f32))
                .custom_parser(|s| parse_decimal(s.trim_end_matches('m').trim()));
            changed |= ui.add(slider).changed();
        });
    }
    if changed {
        // the other end follows, so that the range never turns over
        if range[0] > range[1] {
            range = [range[0].min(range[1]), range[0].max(range[1])];
        }
        *cut = Some(range);
    }
    ui.horizontal(|ui| {
        let button = ui.add_enabled(
            cloud.ceiling.is_some(),
            egui::Button::new(text::HIDE_CEILING),
        );
        if button.clicked() {
            if let Some(ceiling) = cloud.ceiling {
                *cut = Some(below_ceiling(range, ceiling, lo));
            }
        }
        button.on_disabled_hover_text(text::NO_CEILING);
        if ui
            .add_enabled(cut.is_some(), egui::Button::new(text::CUT_RESET))
            .clicked()
        {
            *cut = None;
        }
    });
}

/// The cut of „Decke ausblenden“: z max at the ceiling (not below the cloud's lowest point
/// `lo`), z min kept but never above z max, so that the range never turns over.
fn below_ceiling(range: [f32; 2], ceiling: f32, lo: f32) -> [f32; 2] {
    let top = ceiling.max(lo);
    [range[0].min(top), top]
}

/// A number with a decimal comma or point.
fn parse_decimal(s: &str) -> Option<f64> {
    s.trim().replace(',', ".").parse().ok()
}

/// The point clicked in the 3D view.
fn point_shown(ui: &mut Ui, picked: Option<&PointInfo>) {
    ui.strong(text::POINT);
    let Some(info) = picked else {
        ui.weak(text::NO_POINT);
        return;
    };
    egui::Grid::new("punkt").num_columns(2).show(ui, |ui| {
        for (label, value) in point_lines(info) {
            ui.label(label);
            ui.label(value);
            ui.end_row();
        }
    });
}

/// The rows of the point info: label and value.
pub fn point_lines(info: &PointInfo) -> Vec<(&'static str, String)> {
    let [x, y, z] = info.position;
    let mut lines = vec![
        (text::POINT_X, text::metres(x)),
        (text::POINT_Y, text::metres(y)),
        (text::POINT_Z, text::metres(z)),
        (
            text::POINT_TIME,
            format!("{} s", text::decimal(f64::from(info.t), 2)),
        ),
        (text::POINT_INTENSITY, info.intensity.to_string()),
        (
            text::POINT_COLOR,
            info.rgb.map_or_else(
                || text::NO_COLOR.to_string(),
                |[r, g, b]| format!("{r} / {g} / {b}"),
            ),
        ),
        (
            text::POINT_QUALITY,
            info.quality.map_or_else(
                || text::QUALITY_UNKNOWN.to_string(),
                |q| text::decimal(f64::from(q) / 255.0, 2),
            ),
        ),
    ];
    if let Some(code) = info.removed {
        let reason = text::removal_reason(code).map_or_else(|| code.to_string(), str::to_string);
        lines.push((text::POINT_REMOVED, reason));
    }
    lines
}

/// Why no step can start now: another step runs, or the scan cannot be read.
fn blocked(state: &AppState, scan: &ScanNode) -> Option<String> {
    if let Some(job) = &state.job {
        return Some(text::busy(&job.label));
    }
    match &scan.state {
        ScanState::Missing(why) => Some(why.clone()),
        ScanState::Ready => None,
    }
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
    if step_row(ui, text::STEP_MAP, blocked.or(invalid), mapped) {
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

/// A step button with its state; locked with the reason as tooltip. True when clicked.
fn step_row(ui: &mut Ui, label: &str, blocked: Option<String>, state: StepState) -> bool {
    ui.horizontal(|ui| {
        let button = ui.add_enabled(blocked.is_none(), egui::Button::new(label));
        let button = match blocked {
            Some(why) => button.on_disabled_hover_text(why),
            None => button,
        };
        state_label(ui, state);
        button.clicked()
    })
    .inner
}

fn state_label(ui: &mut Ui, state: StepState) {
    match state {
        StepState::NotComputed => ui.weak(text::NOT_COMPUTED),
        StepState::NotReused => ui.weak(text::NOT_REUSED),
        StepState::Done(seconds) => ui.label(text::step_done(seconds)),
        StepState::Stale => {
            let warn = ui.visuals().warn_fg_color;
            ui.label(RichText::new(text::STALE).color(warn))
        }
    };
}

fn error_label(ui: &mut Ui, message: &str) {
    let color = ui.visuals().error_fg_color;
    ui.label(RichText::new(message).color(color));
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hiding_the_ceiling_never_turns_the_range_over() {
        assert_eq!(below_ceiling([0.5, 4.0], 2.5, 0.0), [0.5, 2.5]);
        // z min above the ceiling follows it down
        assert_eq!(below_ceiling([3.0, 4.0], 2.5, 0.0), [2.5, 2.5]);
        // a ceiling below the cloud is taken as its lowest point
        assert_eq!(below_ceiling([0.5, 4.0], -1.0, 0.0), [0.0, 0.0]);
    }
}
