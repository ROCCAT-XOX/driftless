//! The field of the active step, next to the rail: [`daten`], [`bilder`] or [`planung`]
//! ([`show`] picks it by [`AppState::step`]). Each step starts with its heading; without a
//! project, or without what the step needs, it explains the next move and offers a button for
//! it instead of an empty field.
//!
//! Shared here: the frame of „Daten“ and „Bilder“ (a list that scrolls above the main button
//! and „▸ Erweitert“), cards with a status, the segment switch, the progress of the running
//! job, the state of a step button („nicht gerechnet“, „fertig (12,3 s)“, „veraltet“), why a
//! step is locked, and the clicked point.

pub mod bilder;
pub mod daten;
pub mod planung;
mod tree;

use super::theme::{icon, DARK};
use super::view3d::PointInfo;
use super::{text, theme, Action, AppState, ScanNode, Step};
use crate::jobs::JobStatus;
use crate::project::ScanState;
use eframe::egui::{self, Align, Color32, Layout, Response, RichText, Sense, Ui};

/// Height of the main button (points).
const MAIN_BUTTON_HEIGHT: f32 = 34.0;
/// Height of the line in a card (points).
const CARD_ROW_HEIGHT: f32 = 18.0;
/// Height of a segment of a switch (points).
const SEGMENT_HEIGHT: f32 = 26.0;
/// The list keeps at least this height in a very small window (points).
const MIN_LIST_HEIGHT: f32 = 120.0;

/// Shows the field of [`AppState::step`].
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    match state.step {
        Step::Daten => daten::show(ui, state),
        Step::Bilder => bilder::show(ui, state),
        Step::Planung => planung::show(ui, state),
    }
}

/// Whether „Erweitert“ of `step` is open (kept in egui's memory).
fn advanced_id(step: Step) -> egui::Id {
    egui::Id::new(("erweitert", step))
}

/// Opens or closes „Erweitert“ of `step` (as its toggle does; for tests and pictures).
pub fn set_advanced(ctx: &egui::Context, step: Step, open: bool) {
    ctx.data_mut(|d| d.insert_persisted(advanced_id(step), open));
}

/// The body of the field of `step` below its heading: `list` scrolls, told whether
/// „Erweitert“ is open (its content then goes at the end of the list); `bottom` (the main
/// button) and the toggle „▸ Erweitert“ stay at the bottom. `cx` is handed to both.
fn pinned<C>(
    ui: &mut Ui,
    step: Step,
    cx: &mut C,
    list: impl FnOnce(&mut Ui, &mut C, bool),
    bottom: impl FnOnce(&mut Ui, &mut C),
) {
    let advanced_id = advanced_id(step);
    let mut advanced = ui.data_mut(|d| *d.get_persisted_mut_or(advanced_id, false));
    // The scroll area leaves the height the bottom took in the last frame.
    let bottom_id = egui::Id::new(("unten_hoehe", step));
    let height = ui.data(|d| d.get_temp::<f32>(bottom_id)).unwrap_or(90.0);
    // a solid bar that is always there: the field can be longer than the window
    ui.style_mut().spacing.scroll = egui::style::ScrollStyle::solid();
    egui::ScrollArea::vertical()
        .id_salt(("liste", step))
        .auto_shrink(false)
        .max_height((ui.available_height() - height).max(MIN_LIST_HEIGHT))
        .show(ui, |ui| list(ui, cx, advanced));
    let top = ui.cursor().top();
    ui.add_space(8.0);
    bottom(ui, cx);
    if advanced_toggle(ui, advanced) {
        advanced = !advanced;
        ui.data_mut(|d| d.insert_persisted(advanced_id, advanced));
    }
    let used = ui.min_rect().bottom() - top;
    if (used - height).abs() > 0.5 {
        ui.data_mut(|d| d.insert_temp(bottom_id, used));
        ui.ctx().request_repaint();
    }
}

/// A card: its name (cut with „…“) and, at the right, what `badge` adds (right to left).
/// Returns the response of a click on the whole card, with `hover` as its tooltip.
fn card(
    ui: &mut Ui,
    salt: impl std::hash::Hash + std::fmt::Debug,
    selected: bool,
    name: RichText,
    hover: &str,
    badge: impl FnOnce(&mut Ui),
) -> Response {
    let card = theme::card_frame(selected).show(ui, |ui| {
        let row = egui::vec2(ui.available_width(), CARD_ROW_HEIGHT);
        ui.allocate_ui_with_layout(row, Layout::right_to_left(Align::Center), |ui| {
            badge(ui);
            ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                ui.add(egui::Label::new(name).truncate().selectable(false));
            });
        });
    });
    let id = ui.id().with(("karte", salt));
    ui.interact(card.response.rect, id, Sense::click())
        .on_hover_cursor(egui::CursorIcon::PointingHand)
        .on_hover_text(hover)
}

/// The status at the right of a card: word and symbol (added right to left).
fn badge(ui: &mut Ui, symbol: Option<&str>, word: &str, color: Color32) {
    ui.spacing_mut().item_spacing.x = 4.0;
    ui.label(RichText::new(word).color(color));
    if let Some(symbol) = symbol {
        ui.label(RichText::new(symbol).color(color));
    }
}

/// A switch of segments over the field's width, each a value of `value` with its label;
/// `hover` gives a segment's tooltip. Values within 1e-9 count as the same.
fn segments<'a>(
    ui: &mut Ui,
    choices: impl ExactSizeIterator<Item = (f64, &'a str)>,
    value: &mut f64,
    hover: impl Fn(f64) -> Option<String>,
) {
    theme::segment_frame().show(ui, |ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        let width = ui.available_width() / choices.len() as f32;
        ui.horizontal(|ui| {
            for (choice, label) in choices {
                let on = (choice - *value).abs() < 1e-9;
                let (fill, color) = if on {
                    (DARK.surface_hi, DARK.on_accent)
                } else {
                    (Color32::TRANSPARENT, DARK.text_dim)
                };
                let segment = egui::Button::new(RichText::new(label).color(color))
                    .fill(fill)
                    .corner_radius(egui::CornerRadius::same(theme::CARD_RADIUS - 2))
                    .min_size(egui::vec2(width, SEGMENT_HEIGHT));
                let r = ui.add(segment);
                let r = match hover(choice) {
                    Some(h) => r.on_hover_text(h),
                    None => r,
                };
                if r.clicked() {
                    *value = choice;
                }
            }
        });
    });
}

/// How far a job is (0…1), if it knows its total.
fn fraction(job: &JobStatus) -> Option<f32> {
    let p = &job.progress;
    p.total.map(|total| {
        if total == 0 {
            0.0
        } else {
            (p.done as f64 / total as f64).clamp(0.0, 1.0) as f32
        }
    })
}

/// The running step: a thin bar (if it knows its total) and „Scan 13:30 · Karte rechnen 62 %“.
fn job_progress(ui: &mut Ui, state: &AppState) {
    let Some(job) = &state.job else {
        return;
    };
    let f = fraction(job);
    ui.add_space(4.0);
    // without a total only the line (the status bar has the spinner)
    if let Some(f) = f {
        ui.add(
            egui::ProgressBar::new(f)
                .desired_height(6.0)
                .fill(DARK.accent),
        );
    }
    let scan = state.running.as_ref().and_then(|a| state.scan_of(a));
    let line = text::job_of_scan(scan.map(|s| s.name.as_str()), &job.label, f);
    ui.add(egui::Label::new(RichText::new(line).color(DARK.text_dim)).truncate());
}

/// A wide button in the accent colour; locked with the reason as tooltip, `hover` otherwise.
/// True when clicked.
fn main_button(ui: &mut Ui, label: &str, locked: Option<String>, hover: &str) -> bool {
    let button =
        egui::Button::new(RichText::new(label).strong().color(DARK.on_accent)).fill(DARK.accent);
    let size = egui::vec2(ui.available_width(), MAIN_BUTTON_HEIGHT);
    // add_sized centres the label
    let r = ui
        .add_enabled_ui(locked.is_none(), |ui| ui.add_sized(size, button))
        .inner
        .on_hover_text(hover);
    let r = match locked {
        Some(why) => r.on_disabled_hover_text(why),
        None => r,
    };
    r.clicked()
}

/// A quiet centred button below the main button („Alle rechnen“ beside „Rechnen“); locked
/// with the reason as tooltip. True when clicked.
fn quiet_button(ui: &mut Ui, label: &str, locked: Option<String>) -> bool {
    let r = ui.add_enabled_ui(locked.is_none(), |ui| {
        ui.vertical_centered(|ui| {
            let label = RichText::new(label).color(DARK.text_dim);
            ui.add(egui::Button::new(label).frame(false))
        })
        .inner
    });
    let r = match locked {
        Some(why) => r.inner.on_disabled_hover_text(why),
        None => r.inner,
    };
    r.clicked()
}

/// Muted lines centred below the main button.
fn below_lines(ui: &mut Ui, lines: impl IntoIterator<Item = String>) {
    for line in lines {
        ui.vertical_centered(|ui| {
            ui.add(egui::Label::new(RichText::new(line).color(DARK.text_dim)).wrap());
        });
    }
}

/// „▸ Erweitert“ / „▾ Erweitert“, centred. True when clicked.
fn advanced_toggle(ui: &mut Ui, open: bool) -> bool {
    ui.vertical_centered(|ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            let symbol = if open { icon::UNFOLDED } else { icon::FOLDED };
            let label =
                egui::Button::new(RichText::new(text::ADVANCED).color(DARK.text_dim)).frame(false);
            // centre the pair by hand: vertical_centered centres single widgets only
            let width = 14.0
                + 4.0
                + ui.fonts_mut(|f| {
                    f.layout_no_wrap(
                        text::ADVANCED.into(),
                        egui::TextStyle::Button.resolve(ui.style()),
                        DARK.text_dim,
                    )
                    .size()
                    .x
                });
            ui.add_space(((ui.available_width() - width) / 2.0).max(0.0));
            ui.label(RichText::new(symbol).color(DARK.text_dim));
            ui.add(label).clicked()
        })
        .inner
    })
    .inner
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
