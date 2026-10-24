//! The rail at the left edge: ☰ (the project menu) at the top, the three steps 1 Daten ·
//! 2 Bilder · 3 Planung below it, ⚙ (the settings window) at the bottom. The active step is
//! filled with the accent colour.

use super::theme::{icon, DARK, RAIL_RADIUS};
use super::{text, Action, AppState, Selection, Step};
use eframe::egui::{
    self, Align, Align2, Color32, CornerRadius, FontId, Layout, RectAlign, Response, Sense, Ui,
    WidgetInfo, WidgetType,
};

/// Width of the rail (points), its frame included.
pub const WIDTH: f32 = 60.0;
/// Size of a button of the rail (points).
const BUTTON: egui::Vec2 = egui::vec2(44.0, 44.0);
const ICON_SIZE: f32 = 18.0;
const CAPTION_SIZE: f32 = 10.0;

/// The steps in the rail: step, symbol, name.
const STEPS: [(Step, &str, &str); 3] = [
    (Step::Daten, icon::DATA, text::STEP_DATA),
    (Step::Bilder, icon::IMAGES, text::STEP_IMAGES),
    (Step::Planung, icon::PLAN, text::STEP_PLAN),
];

/// Shows the rail. A step is asked for as [`Action::SetStep`] (it also changes the view);
/// ⚙ opens and closes the settings window itself.
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    ui.with_layout(Layout::top_down(Align::Center), |ui| {
        ui.spacing_mut().item_spacing.y = 12.0;
        let menu = button(ui, icon::MENU, None, text::MENU_PROJECT, false);
        egui::Popup::menu(&menu)
            .align(RectAlign::RIGHT_START)
            .show(|ui| project_menu(ui, state, &mut action));
        for (step, symbol, name) in STEPS {
            if button(ui, symbol, Some(name), name, state.step == step).clicked() {
                action = Some(Action::SetStep(step));
            }
        }
        ui.with_layout(Layout::bottom_up(Align::Center), |ui| {
            let open = state.settings_window.open;
            if button(ui, icon::SETTINGS, None, text::SETTINGS, open).clicked() {
                state.settings_window.open = !open;
            }
        });
    });
    action
}

/// The entries of ☰: what the old menu bar „Projekt“ had, and „Lauf hinzufügen“ for the scan
/// of the selection.
fn project_menu(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let has_project = state.project.is_some();
    if ui.button(text::NEW_PROJECT).clicked() {
        *action = Some(Action::PickNewProject);
    }
    if ui.button(text::OPEN_PROJECT).clicked() {
        *action = Some(Action::PickOpenProject);
    }
    let r = ui.add_enabled(has_project, egui::Button::new(text::ADD_SCAN));
    if r.clicked() {
        *action = Some(Action::PickAddScan);
    }
    r.on_disabled_hover_text(text::ADD_SCAN_NEEDS_PROJECT);
    let scan = selected_scan(state);
    let r = ui.add_enabled(scan.is_some(), egui::Button::new(text::ADD_RUN));
    if r.clicked() {
        if let Some(id) = scan {
            *action = Some(Action::PickAddRun(id));
        }
    }
    r.on_disabled_hover_text(text::ADD_RUN_NEEDS_SCAN);
    if ui.button(text::OPEN_SURFACE).clicked() {
        *action = Some(Action::PickOpenSurface);
    }
}

/// The scan of the selection: the scan itself, or the scan of a selected run, map, cloud,
/// surface or state.
fn selected_scan(state: &AppState) -> Option<String> {
    match &state.selection {
        Selection::Scan(id) => state.scan_node(id).map(|s| s.id.clone()),
        _ => state.selected_run().map(|(s, _)| s.id.clone()),
    }
}

/// A square button: the symbol, a caption below it if given; `name` for screen readers, and
/// as tooltip of a button without caption.
fn button(ui: &mut Ui, symbol: &str, caption: Option<&str>, name: &str, on: bool) -> Response {
    let (rect, response) = ui.allocate_exact_size(BUTTON, Sense::click());
    let enabled = ui.is_enabled();
    response.widget_info(|| WidgetInfo::selected(WidgetType::Button, enabled, on, name));
    if ui.is_rect_visible(rect) {
        let p = DARK;
        let fill = if on {
            p.accent
        } else if response.hovered() {
            p.surface_hi
        } else {
            p.surface
        };
        let fg = if on {
            Color32::WHITE
        } else if response.hovered() {
            p.text
        } else {
            p.text_dim
        };
        let painter = ui.painter();
        painter.rect_filled(rect, CornerRadius::same(RAIL_RADIUS), fill);
        let icon_at = match caption {
            Some(_) => rect.center() - egui::vec2(0.0, 7.0),
            None => rect.center(),
        };
        let font = FontId::proportional(ICON_SIZE);
        painter.text(icon_at, Align2::CENTER_CENTER, symbol, font, fg);
        if let Some(caption) = caption {
            let at = rect.center_bottom() - egui::vec2(0.0, 6.0);
            let font = FontId::proportional(CAPTION_SIZE);
            painter.text(at, Align2::CENTER_BOTTOM, caption, font, fg);
        }
    }
    match caption {
        Some(_) => response,
        None => response.on_hover_text(name),
    }
}
