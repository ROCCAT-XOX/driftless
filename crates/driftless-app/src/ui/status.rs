//! The status bar at the bottom: the running step (phase, progress bar or spinner without a
//! total, runtime, „Abbrechen“) or the last message, and the log, which opens on a click.

use super::{text, Action, AppState, UiMessage};
use crate::jobs::JobStatus;
use driftless_io::progress::LogLine;
use eframe::egui::{self, Color32, RichText, Ui};

/// Width of the progress bar (points).
const BAR_WIDTH: f32 = 220.0;
/// Height of the open log (points).
const LOG_HEIGHT: f32 = 160.0;
const ERROR_COLOR: Color32 = Color32::from_rgb(0xe0, 0x4b, 0x4b);

pub fn show(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let mut action = None;
    ui.horizontal(|ui| match &state.job {
        Some(job) => job_line(ui, job, &mut action),
        None => message_line(ui, state.messages.last(), &mut action),
    });
    let header = format!("{} ({})", text::LOG, state.log.len());
    egui::CollapsingHeader::new(header)
        .id_salt("protokoll")
        .default_open(false)
        .show(ui, |ui| {
            egui::ScrollArea::vertical()
                .max_height(LOG_HEIGHT)
                .auto_shrink([false, true])
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    let warn = ui.visuals().warn_fg_color;
                    for line in &state.log {
                        match line {
                            LogLine::Info(t) => ui.label(t),
                            LogLine::Warning(t) => ui.label(RichText::new(t).color(warn)),
                        };
                    }
                });
        });
    action
}

fn job_line(ui: &mut Ui, job: &JobStatus, action: &mut Option<Action>) {
    let p = &job.progress;
    let what = if p.phase.is_empty() {
        job.label.clone()
    } else {
        format!("{}: {}", job.label, p.phase)
    };
    match p.total {
        Some(total) => {
            let fraction = if total == 0 {
                0.0
            } else {
                (p.done as f64 / total as f64).clamp(0.0, 1.0) as f32
            };
            ui.add(egui::ProgressBar::new(fraction).desired_width(BAR_WIDTH));
            ui.label(what);
            ui.label(format!("{} / {}", count(p.done), count(total)));
        }
        None => {
            ui.add(egui::Spinner::new());
            ui.label(what);
            if p.done > 0 {
                ui.label(count(p.done));
            }
        }
    }
    ui.label(duration(job.seconds));
    if job.cancelling {
        ui.add_enabled(false, egui::Button::new(text::CANCELLING));
    } else if ui.button(text::CANCEL).clicked() {
        *action = Some(Action::Cancel);
    }
}

fn message_line(ui: &mut Ui, last: Option<&UiMessage>, action: &mut Option<Action>) {
    match last {
        None => {
            ui.label(text::READY);
        }
        Some(m) => {
            match m {
                UiMessage::Info(t) => ui.label(t),
                UiMessage::Error(t) => ui.label(RichText::new(t).color(ERROR_COLOR)),
            };
            if ui.small_button(text::DISMISS).clicked() {
                *action = Some(Action::DismissMessages);
            }
        }
    }
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

/// „12 s“ below a minute, else „3:05 min“ (seconds cut off, not rounded).
pub fn duration(seconds: f64) -> String {
    let s = seconds.max(0.0) as u64;
    if s < 60 {
        format!("{s} s")
    } else {
        format!("{}:{:02} min", s / 60, s % 60)
    }
}
