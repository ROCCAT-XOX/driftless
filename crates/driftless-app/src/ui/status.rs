//! The status bar at the bottom, one slim line: at the left „Protokoll (n)“, which opens the
//! log above the line, and the last message with „Schließen“; at the right the running step
//! or, without one, the cloud or surface being loaded (progress bar or spinner without a
//! total, phase, runtime, „Abbrechen“).

use super::theme::{icon, DARK};
use super::{text, Action, AppState, UiMessage};
use crate::jobs::JobStatus;
use driftless_io::progress::LogLine;
use eframe::egui::{self, Align, Layout, RichText, Ui};

/// Width of the progress bar (points).
const BAR_WIDTH: f32 = 160.0;
/// Height of the open log (points).
const LOG_HEIGHT: f32 = 160.0;

pub fn show(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let mut action = None;
    let open_id = egui::Id::new("protokoll_offen");
    let mut open = ui.data_mut(|d| *d.get_persisted_mut_or(open_id, false));
    if open {
        log(ui, state);
        ui.separator();
    }
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 6.0;
        if log_toggle(ui, open, state.log.len()) {
            open = !open;
            ui.data_mut(|d| d.insert_persisted(open_id, open));
        }
        ui.separator();
        // the job at the right first, so that the message gets what is left (cut with „…“)
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            // A step comes first; loading runs beside it and is shown when no step runs.
            if let Some(job) = state.job.as_ref().or(state.loading.as_ref()) {
                job_line(ui, job, &mut action);
                ui.separator();
            }
            let last = state.messages.last();
            if last.is_some() && ui.small_button(text::DISMISS).clicked() {
                action = Some(Action::DismissMessages);
            }
            ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                message_line(ui, last);
            });
        });
    });
    action
}

/// „▸ Protokoll (n)“. True when clicked.
fn log_toggle(ui: &mut Ui, open: bool, lines: usize) -> bool {
    let symbol = if open { icon::UNFOLDED } else { icon::FOLDED };
    ui.label(RichText::new(symbol).color(DARK.text_dim));
    let label = RichText::new(text::log_header(lines)).color(DARK.text_dim);
    ui.add(egui::Button::new(label).frame(false)).clicked()
}

/// The lines of the log, newest at the bottom.
fn log(ui: &mut Ui, state: &AppState) {
    // Only the visible lines are laid out: the log holds up to 10 000.
    let row = ui.text_style_height(&egui::TextStyle::Body);
    egui::ScrollArea::vertical()
        .max_height(LOG_HEIGHT)
        .auto_shrink([false, true])
        .stick_to_bottom(true)
        .show_rows(ui, row, state.log.len(), |ui, rows| {
            for line in &state.log[rows] {
                let label = match line {
                    LogLine::Info(t) => egui::Label::new(t),
                    LogLine::Warning(t) => egui::Label::new(RichText::new(t).color(DARK.warn)),
                };
                // one row per line, so that the row height holds
                ui.add(label.truncate());
            }
        });
}

/// The job, added right to left: „Abbrechen“, runtime, count, phase, bar.
fn job_line(ui: &mut Ui, job: &JobStatus, action: &mut Option<Action>) {
    if job.cancelling {
        ui.add_enabled(false, egui::Button::new(text::CANCELLING));
    } else if ui.button(text::CANCEL).clicked() {
        *action = Some(Action::Cancel);
    }
    ui.label(RichText::new(text::duration(job.seconds)).color(DARK.text_dim));
    let p = &job.progress;
    let what = if p.phase.is_empty() {
        job.label.clone()
    } else {
        format!("{}: {}", job.label, p.phase)
    };
    match p.total {
        Some(total) => {
            ui.label(format!("{} / {}", count(p.done), count(total)));
            ui.label(what);
            let fraction = if total == 0 {
                0.0
            } else {
                (p.done as f64 / total as f64).clamp(0.0, 1.0) as f32
            };
            ui.add(
                egui::ProgressBar::new(fraction)
                    .desired_width(BAR_WIDTH)
                    .desired_height(6.0)
                    .fill(DARK.accent),
            );
        }
        None => {
            if p.done > 0 {
                ui.label(count(p.done));
            }
            ui.label(what);
            ui.add(egui::Spinner::new());
        }
    }
}

/// The last message (cut with „…“), else „Bereit“.
fn message_line(ui: &mut Ui, last: Option<&UiMessage>) {
    let line = match last {
        None => RichText::new(text::READY).color(DARK.text_dim),
        Some(UiMessage::Info(t)) => RichText::new(t),
        Some(UiMessage::Error(t)) => RichText::new(t).color(DARK.error),
    };
    ui.add(egui::Label::new(line).truncate());
}

/// `1234567` → „1.234.567“.
pub fn count(n: u64) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push('.');
        }
        out.push(c);
    }
    out
}
