//! Step 3 „Planung“: the measurements of the tool „Messen“ (not saved yet), and a card that
//! says what comes with Baustein 5 (floor plan, measures, DWG), below the heading as in
//! „Daten“ and „Bilder“.

use super::{heading, hint, no_project};
use crate::ui::theme::{self, icon, DARK};
use crate::ui::{text, Action, AppState};
use eframe::egui::{self, Align, Layout, RichText, Ui};

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_PLAN);
    if state.project.is_none() {
        no_project(ui, &mut action);
    }
    ui.add_space(4.0);
    measurements(ui, state);
    if state.project.is_none() {
        return action;
    }
    ui.add_space(4.0);
    theme::card_frame(false).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.label(
            RichText::new(icon::PLAN)
                .size(theme::HEADING_SIZE)
                .color(DARK.text_dim),
        );
        ui.add(egui::Label::new(RichText::new(text::PLAN_COMING).color(DARK.text_dim)).wrap());
    });
    action
}

/// „Messungen“: a card per measurement with its distance, or how to make one; „Messungen
/// löschen“ below them.
fn measurements(ui: &mut Ui, state: &mut AppState) {
    ui.strong(text::MEASUREMENTS);
    if state.measurements.is_empty() {
        hint(ui, text::NO_MEASUREMENTS);
        return;
    }
    for (i, m) in state.measurements.iter().enumerate() {
        theme::card_frame(false).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(RichText::new(icon::MEASURE).color(DARK.measure));
                ui.label(text::measurement_name(i + 1));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.label(RichText::new(text::meters(m.distance())).strong());
                });
            });
        });
    }
    hint(ui, text::MEASUREMENTS_NOT_SAVED);
    if ui.button(text::CLEAR_MEASUREMENTS).clicked() {
        state.measurements.clear();
        state.measure_a = None;
    }
}
