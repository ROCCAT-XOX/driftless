//! The questions before a change of the project, each in a small window in the middle of the
//! view ([`AppState::ask`]): „Scan umbenennen“ with the name to type, „Aus Projekt entfernen“
//! with what stays (the raw data and the results), and „Löschen“ with the size of the folder,
//! what lies in it and what depends on it, and why it cannot go now if it cannot. Esc or
//! „Abbrechen“ closes a question without a change.

use super::theme::DARK;
use super::{text, Action, AppState, Ask, DeleteQuestion};
use eframe::egui::{self, Align2, Context, RichText, Ui};

/// Width of a question window (points).
const WIDTH: f32 = 380.0;

/// The window of the question asked, if any. Returns what was answered.
pub fn show(ctx: &Context, state: &mut AppState) -> Option<Action> {
    let ask = state.ask.clone()?;
    let mut action = None;
    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        return Some(Action::CloseAsk);
    }
    let title = match &ask {
        Ask::RenameScan { .. } => text::RENAME_TITLE,
        Ask::RemoveScan { .. } => text::REMOVE_TITLE,
        Ask::Delete(_) => text::DELETE_TITLE,
    };
    egui::Window::new(title)
        .collapsible(false)
        .resizable(false)
        .anchor(Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
        .fixed_size(egui::vec2(WIDTH, 0.0))
        .show(ctx, |ui| {
            action = match &ask {
                Ask::RenameScan { id, name } => rename(ui, state, id, name),
                Ask::RemoveScan { id } => remove(ui, state, id),
                Ask::Delete(question) => delete(ui, state, question),
            };
        });
    action
}

/// „Abbrechen“ and the answer button (locked with the reason as tooltip). Returns which was
/// clicked: `Some(true)` the answer.
fn buttons(ui: &mut Ui, answer: &str, locked: Option<&str>) -> Option<bool> {
    let mut clicked = None;
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        let r = ui.add_enabled(
            locked.is_none(),
            egui::Button::new(RichText::new(answer).strong()),
        );
        if r.clicked() {
            clicked = Some(true);
        }
        if let Some(why) = locked {
            r.on_disabled_hover_text(why);
        }
        if ui.button(text::ASK_CANCEL).clicked() {
            clicked = Some(false);
        }
    });
    clicked
}

/// A paragraph that wraps in the window.
fn paragraph(ui: &mut Ui, what: &str) {
    ui.add(egui::Label::new(what).wrap());
}

fn rename(ui: &mut Ui, state: &mut AppState, id: &str, name: &str) -> Option<Action> {
    let mut typed = name.to_string();
    let enter = ui
        .horizontal(|ui| {
            let label = ui.label(text::SCAN_NAME);
            let edit = ui
                .add(egui::TextEdit::singleline(&mut typed).desired_width(WIDTH - 80.0))
                .labelled_by(label.id);
            edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))
        })
        .inner;
    if typed != name {
        state.ask = Some(Ask::RenameScan {
            id: id.to_string(),
            name: typed.clone(),
        });
    }
    let empty = typed.trim().is_empty().then_some(text::SCAN_NAME_EMPTY);
    let busy = state.scan_busy(id);
    let locked = busy.as_deref().or(empty);
    let answer = buttons(ui, text::RENAME_CONFIRM, locked);
    match (answer, enter && locked.is_none()) {
        (Some(true), _) | (_, true) => Some(Action::RenameScan {
            id: id.to_string(),
            name: typed,
        }),
        (Some(false), _) => Some(Action::CloseAsk),
        _ => None,
    }
}

fn remove(ui: &mut Ui, state: &AppState, id: &str) -> Option<Action> {
    let (Some(project), Some(scan)) = (
        state.project.as_ref(),
        state.project.as_ref().and_then(|p| p.scan(id)),
    ) else {
        return Some(Action::CloseAsk);
    };
    let results = project.dir.join(crate::project::SCANS_DIR).join(id);
    paragraph(
        ui,
        &text::remove_scan_question(&scan.name, &scan.path, &results),
    );
    let busy = state.scan_busy(id);
    match buttons(ui, text::REMOVE_CONFIRM, busy.as_deref()) {
        Some(true) => Some(Action::RemoveScan(id.to_string())),
        Some(false) => Some(Action::CloseAsk),
        None => None,
    }
}

fn delete(ui: &mut Ui, state: &AppState, q: &DeleteQuestion) -> Option<Action> {
    paragraph(ui, &text::delete_question(&q.label, q.bytes));
    if !q.contains.is_empty() {
        paragraph(ui, &text::delete_contains(&q.contains));
    }
    if !q.used_by.is_empty() {
        let used = RichText::new(text::delete_used_by(&q.used_by)).color(DARK.warn);
        ui.add(egui::Label::new(used).wrap());
    }
    // asked again each frame: what is shown or computed may change while the question is open
    let computing = state.delete_computing(&q.dir);
    let shown = state.delete_shown(&q.dir);
    let mut unshow = false;
    if let Some(why) = computing.as_ref().or(shown.as_ref()) {
        let why = RichText::new(why).color(DARK.warn);
        ui.add(egui::Label::new(why).wrap());
        // what is only shown can be taken out of the view, so the folder can go
        unshow = computing.is_none() && ui.button(text::UNSHOW).clicked();
    }
    if unshow {
        return Some(Action::Unshow(q.dir.clone()));
    }
    let blocked = computing.or(shown);
    match buttons(ui, text::DELETE_CONFIRM, blocked.as_deref()) {
        Some(true) => Some(Action::Delete(q.dir.clone())),
        Some(false) => Some(Action::CloseAsk),
        None => None,
    }
}
