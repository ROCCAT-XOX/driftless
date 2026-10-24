//! Step 2 „Bilder“: the maps of the project with their surfaces („Oberfläche 001“, a stale one
//! marked), and for the chosen map „Oberfläche rechnen“ with its settings (voxel size 1, 1,5 or
//! 2 cm and „ohne Fotos“ of `driftless surface`). Clicking a surface shows it.

use super::StepState;
use super::{blocked, heading, hint, no_project, state_label, step_row, tree};
use crate::project::{MapInfo, SurfaceInfo, SurfaceSettings};
use crate::ui::{text, Action, AppState, Selection, Step};
use eframe::egui::{self, Ui};

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_IMAGES);
    if state.project.is_none() {
        no_project(ui, &mut action);
        return action;
    }
    let has_maps = state
        .scans
        .iter()
        .flat_map(|s| &s.runs)
        .any(|r| !r.maps.is_empty());
    if !has_maps {
        hint(ui, text::EMPTY_NO_MAPS);
        if ui.button(text::GO_TO_DATA).clicked() {
            action = Some(Action::SetStep(Step::Daten));
        }
        return action;
    }
    egui::ScrollArea::vertical()
        .auto_shrink(false)
        .show(ui, |ui| {
            ui.strong(text::MAPS_AND_SURFACES);
            map_list(ui, state, &mut action);
            ui.separator();
            surface_step(ui, state, &mut action);
        });
    action
}

/// Every map of the project, its surfaces below it.
fn map_list(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    for scan in &state.scans {
        for run in &scan.runs {
            for map in &run.maps {
                let selected = state.selection == Selection::Map(map.dir.clone());
                let name = text::map_of(&scan.name, &run.label, &map.label);
                let hover = format!("{name}\n{}", map.dir.display());
                if tree::entry(ui, selected, name, &hover).clicked() {
                    *action = Some(Action::Select(Selection::Map(map.dir.clone())));
                }
                for surface in &map.surfaces {
                    tree::surface_leaf(ui, state, surface, action);
                }
            }
        }
    }
}

/// „Oberfläche rechnen“ for the map of the selection, with the settings of the next surface.
fn surface_step(ui: &mut Ui, state: &mut AppState, action: &mut Option<Action>) {
    let Some(map) = state.selected_map().cloned() else {
        hint(ui, text::SELECT_MAP_FOR_SURFACE);
        return;
    };
    let blocked = state
        .map(&map.dir)
        .and_then(|(scan, _, _)| blocked(state, scan));
    let settings = &state.surface_settings;
    let invalid = settings
        .config
        .check()
        .err()
        .map(|e| e.to_string())
        .or_else(|| {
            map.settings
                .is_none()
                .then(|| text::SURFACE_NEEDS_MAP.to_string())
        });
    let surfaced = surface_state(state, &map, settings);
    if step_row(ui, text::STEP_SURFACE, blocked.or(invalid), surfaced) {
        *action = Some(Action::RunSurface(map.dir.clone()));
    }
    if let Selection::Surface(dir) = &state.selection {
        if let Some(surface) = map.surfaces.iter().find(|s| s.dir == *dir) {
            ui.horizontal(|ui| {
                ui.label(text::THIS_SURFACE);
                state_label(ui, own_surface_state(state, surface));
            });
        }
    }
    ui.separator();
    surface_settings_edit(ui, &mut state.surface_settings);
}

/// The state of a surface of `map` with `settings`: done if one made with them still fits,
/// stale if all such ones no longer fit.
fn surface_state(state: &AppState, map: &MapInfo, settings: &SurfaceSettings) -> StepState {
    let mut same = map
        .surfaces
        .iter()
        .rev()
        .filter(|s| s.settings.as_ref() == Some(settings))
        .peekable();
    if same.peek().is_none() {
        return StepState::NotComputed;
    }
    same.map(|s| own_surface_state(state, s))
        .find(|s| *s != StepState::StaleSurface)
        .unwrap_or(StepState::StaleSurface)
}

fn own_surface_state(state: &AppState, surface: &SurfaceInfo) -> StepState {
    if surface.stale {
        return StepState::StaleSurface;
    }
    StepState::Done(state.map_status.get(&surface.dir).and_then(|s| s.seconds))
}

/// The settings of the next surface: the voxel size (1, 1,5 or 2 cm) and „ohne Fotos“.
fn surface_settings_edit(ui: &mut Ui, settings: &mut SurfaceSettings) {
    ui.strong(text::SURFACE_SETTINGS);
    ui.weak(text::SURFACE_SETTINGS_HINT);
    ui.horizontal(|ui| {
        ui.label(text::SURFACE_VOXEL)
            .on_hover_text(text::SURFACE_VOXEL_HINT);
        for (voxel, label) in text::SURFACE_VOXEL_CHOICES {
            ui.selectable_value(&mut settings.config.voxel, voxel, label);
        }
    });
    let mut no_photos = !settings.texture;
    if ui
        .checkbox(&mut no_photos, text::SURFACE_NO_PHOTOS)
        .changed()
    {
        settings.texture = !no_photos;
    }
}
