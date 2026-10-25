//! Step 3 „Planung“: floor plan, measures and DWG come with Baustein 5; until then the field
//! says so in a card, below the heading as in „Daten“ and „Bilder“.

use super::{heading, no_project};
use crate::ui::theme::{self, icon, DARK};
use crate::ui::{text, Action, AppState};
use eframe::egui::{self, RichText, Ui};

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_PLAN);
    if state.project.is_none() {
        no_project(ui, &mut action);
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
