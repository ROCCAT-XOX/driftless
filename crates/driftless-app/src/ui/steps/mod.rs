//! The field of the active step, next to the rail: [`daten`], [`bilder`] or [`planung`]
//! ([`show`] picks it by [`AppState::step`]). Each step starts with its heading; without a
//! project, or without what the step needs, it explains the next move and offers a button for
//! it instead of an empty field.
//!
//! Shared here: the state of a step button („nicht gerechnet“, „fertig (12,3 s)“, „veraltet“),
//! why a step is locked, and the clicked point.

pub mod bilder;
pub mod daten;
pub mod planung;
mod tree;

use super::view3d::PointInfo;
use super::{text, theme, Action, AppState, ScanNode, Step};
use crate::project::ScanState;
use eframe::egui::{self, RichText, Ui};

/// Shows the field of [`AppState::step`].
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    match state.step {
        Step::Daten => daten::show(ui, state),
        Step::Bilder => bilder::show(ui, state),
        Step::Planung => planung::show(ui, state),
    }
}

/// The heading of a step: larger than everything below it in the field.
fn heading(ui: &mut Ui, title: &str) {
    ui.label(RichText::new(title).size(theme::HEADING_SIZE).strong());
}

/// Without a project: what to do, and the two buttons for it.
fn no_project(ui: &mut Ui, action: &mut Option<Action>) {
    hint(ui, text::EMPTY_NO_PROJECT);
    if ui.button(text::OPEN_PROJECT).clicked() {
        *action = Some(Action::PickOpenProject);
    }
    if ui.button(text::NEW_PROJECT).clicked() {
        *action = Some(Action::PickNewProject);
    }
}

/// A muted hint that wraps in the field.
fn hint(ui: &mut Ui, message: &str) {
    ui.add(egui::Label::new(RichText::new(message).weak()).wrap());
}

/// How far a step is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum StepState {
    NotComputed,
    /// A run that was never recomputed from its submaps.
    NotReused,
    /// With its runtime (s), if known.
    Done(Option<f64>),
    /// There is a result, but it no longer fits the run.
    Stale,
    /// There is a surface, but it no longer fits its map.
    StaleSurface,
}

/// A section that can be folded away (open at first; egui keeps the choice). Its title is
/// body-sized, below the heading of the step.
pub(super) fn section(ui: &mut Ui, title: &str, body: impl FnOnce(&mut Ui)) {
    egui::CollapsingHeader::new(RichText::new(title).strong())
        .id_salt(title)
        .default_open(true)
        .show(ui, body);
}

/// A number with a decimal comma or point.
pub(crate) fn parse_decimal(s: &str) -> Option<f64> {
    s.trim().replace(',', ".").parse().ok()
}

/// The point clicked in the 3D view.
pub(super) fn point_shown(ui: &mut Ui, picked: Option<&PointInfo>) {
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
pub(super) fn blocked(state: &AppState, scan: &ScanNode) -> Option<String> {
    if let Some(job) = &state.job {
        return Some(text::busy(&job.label));
    }
    match &scan.state {
        ScanState::Missing(why) => Some(why.clone()),
        ScanState::Ready => None,
    }
}

/// A step button with its state; locked with the reason as tooltip. True when clicked.
pub(super) fn step_row(
    ui: &mut Ui,
    label: &str,
    blocked: Option<String>,
    state: StepState,
) -> bool {
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

pub(super) fn state_label(ui: &mut Ui, state: StepState) {
    match state {
        StepState::NotComputed => ui.weak(text::NOT_COMPUTED),
        StepState::NotReused => ui.weak(text::NOT_REUSED),
        StepState::Done(seconds) => ui.label(text::step_done(seconds)),
        StepState::Stale | StepState::StaleSurface => {
            let warn = ui.visuals().warn_fg_color;
            let what = match state {
                StepState::Stale => text::STALE,
                _ => text::SURFACE_STALE,
            };
            ui.label(RichText::new(what).color(warn))
        }
    };
}

pub(super) fn error_label(ui: &mut Ui, message: &str) {
    let color = ui.visuals().error_fg_color;
    ui.label(RichText::new(message).color(color));
}
