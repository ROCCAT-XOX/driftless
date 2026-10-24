//! Step 3 „Planung“: floor plan, measures and DWG come with Baustein 5; until then the field
//! says so.

use super::{heading, hint, no_project};
use crate::ui::{text, Action, AppState};
use eframe::egui::Ui;

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_PLAN);
    if state.project.is_none() {
        no_project(ui, &mut action);
        return action;
    }
    hint(ui, text::PLAN_COMING);
    action
}
