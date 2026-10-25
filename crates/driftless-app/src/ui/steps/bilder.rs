//! Step 2 „Bilder“, in the form of „Daten“: the heading with the number of maps and
//! surfaces, a card per map with its status (neu, wartet, rechnet, fertig ✓, veraltet, Karte
//! veraltet, Karte noch nicht fertig, Scan fehlt) and its surfaces below it („Oberfläche 001“, a stale one marked; a click shows
//! it), the fineness of the next surface as a three-way switch (1 / 1,5 / 2 cm), the progress
//! of the running job, and at the bottom the main button: „Alle Oberflächen rechnen“ queues
//! every map without a current surface ([`Action::ComputeAllSurfaces`]), „Oberfläche
//! rechnen“ only the selected map. „ohne Fotos“ lies under „▸ Erweitert“.

use super::{
    badge, below_lines, blocked, card, fraction, heading, hint, job_progress, main_button,
    no_project, pinned, quiet_button, segments, state_label, tree, StepState,
};
use crate::project::{MapInfo, ScanState, SurfaceInfo, SurfaceSettings};
use crate::ui::theme::{icon, DARK};
use crate::ui::{text, Action, AppState, Queued, Selection, Step};
use eframe::egui::{self, RichText, Ui};

/// Where the surfaces of a map stand, as its card shows it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceStatus {
    /// No surface yet.
    New,
    /// Waits in the queue of „Alle Oberflächen rechnen“.
    Queued,
    /// Its surface is being computed, with its progress (0…1) if the step knows its total.
    Running(Option<f32>),
    /// One of its surfaces still fits it.
    Done,
    /// It has surfaces, but none fits any more.
    Stale,
    /// The map itself no longer fits its run: it is computed anew in „Daten“ first, „Alle
    /// Oberflächen rechnen“ leaves it out.
    MapStale,
    /// The map has no `map.json` yet (still being computed, or failed): left out as well.
    MapNotReady,
    /// The folder of its scan cannot be read: left out as well.
    ScanMissing,
}

/// The status of the surfaces of `map`: a running step or the queue first, then its scan and
/// the map, then its surfaces. A map or surface counts as fitting until the background check
/// ([`AppState::map_status`]) says it is stale.
pub fn surface_status(state: &AppState, map: &MapInfo) -> SurfaceStatus {
    if state.running.as_ref() == Some(&Action::RunSurface(map.dir.clone())) {
        return SurfaceStatus::Running(state.job.as_ref().and_then(fraction));
    }
    if state.queue.contains(&Queued::Surface(map.dir.clone())) {
        return SurfaceStatus::Queued;
    }
    let scan_missing = state
        .map(&map.dir)
        .is_some_and(|(scan, _, _)| matches!(scan.state, ScanState::Missing(_)));
    let map_stale = state.map_status.get(&map.dir).is_some_and(|s| s.stale);
    if scan_missing {
        SurfaceStatus::ScanMissing
    } else if map.settings.is_none() {
        SurfaceStatus::MapNotReady
    } else if map_stale {
        SurfaceStatus::MapStale
    } else if map.surfaces.iter().any(|s| !s.stale) {
        SurfaceStatus::Done
    } else if !map.surfaces.is_empty() {
        SurfaceStatus::Stale
    } else {
        SurfaceStatus::New
    }
}

/// Opens or closes „Erweitert“ (as its toggle does; for tests and pictures).
pub fn set_advanced(ctx: &egui::Context, open: bool) {
    super::set_advanced(ctx, Step::Bilder, open);
}

/// Every map of the project, in the order of the list.
fn maps(state: &AppState) -> impl Iterator<Item = (&str, &str, &MapInfo)> {
    state.scans.iter().flat_map(|scan| {
        scan.runs.iter().flat_map(move |run| {
            run.maps
                .iter()
                .map(move |m| (scan.name.as_str(), run.label.as_str(), m))
        })
    })
}

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_IMAGES);
    if state.project.is_none() {
        no_project(ui, &mut action);
        return action;
    }
    let count = maps(state).count();
    if count == 0 {
        hint(ui, text::EMPTY_NO_MAPS);
        if ui.button(text::GO_TO_DATA).clicked() {
            action = Some(Action::SetStep(Step::Daten));
        }
        return action;
    }
    let surfaces = maps(state).map(|(_, _, m)| m.surfaces.len()).sum();
    ui.weak(text::surfaces_line(count, surfaces));
    // the main button and „Erweitert“ stay at the bottom, the rest scrolls above them
    pinned(
        ui,
        Step::Bilder,
        &mut (state, &mut action),
        |ui, (state, action), advanced| {
            ui.add_space(4.0);
            map_list(ui, state, action);
            ui.add_space(6.0);
            fineness_switch(ui, &mut state.surface_settings);
            selection_state(ui, state);
            job_progress(ui, state);
            if advanced {
                ui.separator();
                advanced_content(ui, &mut state.surface_settings);
            }
        },
        |ui, (state, action)| compute_buttons(ui, state, action),
    );
    action
}

/// A card per map, its surfaces below it. A click on a card selects the map (again:
/// deselects), a click on a surface selects and shows it.
fn map_list(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    for (scan, run, map) in maps(state) {
        let selected = state.selection == Selection::Map(map.dir.clone());
        let name = text::map_of(scan, run, &map.label);
        let status = surface_status(state, map);
        let why = match status {
            SurfaceStatus::MapStale => Some(text::STATUS_MAP_STALE_HINT.to_string()),
            SurfaceStatus::MapNotReady => Some(text::SURFACE_NEEDS_MAP.to_string()),
            SurfaceStatus::ScanMissing => state.map(&map.dir).and_then(|(scan, _, _)| match &scan
                .state
            {
                ScanState::Missing(why) => Some(why.clone()),
                ScanState::Ready => None,
            }),
            _ => None,
        };
        let hover = match why {
            Some(why) => format!("{name}\n{why}\n{}", map.dir.display()),
            None => format!("{name}\n{}", map.dir.display()),
        };
        let r = card(
            ui,
            ("karte", &map.dir),
            selected,
            RichText::new(name),
            &hover,
            |ui| status_label(ui, status),
        );
        if r.clicked() {
            let selection = if selected {
                Selection::None
            } else {
                Selection::Map(map.dir.clone())
            };
            *action = Some(Action::Select(selection));
        }
        if !map.surfaces.is_empty() {
            ui.indent(("oberflaechen", &map.dir), |ui| {
                for surface in &map.surfaces {
                    tree::surface_leaf(ui, state, surface, action);
                }
            });
        }
    }
}

/// The status at the right of a card.
fn status_label(ui: &mut Ui, status: SurfaceStatus) {
    let (symbol, word, color) = match status {
        SurfaceStatus::New => (None, text::STATUS_NEW, DARK.text_dim),
        SurfaceStatus::Queued => (None, text::STATUS_QUEUED, DARK.text_dim),
        SurfaceStatus::Running(_) => (Some(icon::RUNNING), text::STATUS_RUNNING, DARK.warn),
        SurfaceStatus::Done => (Some(icon::CHECK), text::STATUS_DONE, DARK.ok),
        SurfaceStatus::Stale => (Some(icon::STALE), text::STATUS_STALE, DARK.warn),
        SurfaceStatus::MapStale => (Some(icon::STALE), text::STATUS_MAP_STALE, DARK.text_dim),
        SurfaceStatus::MapNotReady => (None, text::STATUS_MAP_NOT_READY, DARK.text_dim),
        SurfaceStatus::ScanMissing => (Some(icon::STALE), text::STATUS_SCAN_MISSING, DARK.text_dim),
    };
    badge(ui, symbol, word, color);
}

/// „Feinheit“: 1 / 1,5 / 2 cm, the voxel size of the next surface; the memory hint at 1 cm,
/// the value of a size that is none of them („Eigene“).
fn fineness_switch(ui: &mut Ui, settings: &mut SurfaceSettings) {
    ui.label(RichText::new(text::SURFACE_FINENESS).strong())
        .on_hover_text(text::SURFACE_VOXEL_HINT);
    let voxel = &mut settings.config.voxel;
    let choices = text::SURFACE_VOXEL_CHOICES.into_iter();
    segments(ui, choices, voxel, |_| None);
    let finest = text::SURFACE_VOXEL_CHOICES[0].0;
    let known = text::SURFACE_VOXEL_CHOICES
        .iter()
        .any(|(v, _)| (v - *voxel).abs() < 1e-9);
    if (*voxel - finest).abs() < 1e-9 {
        let warn = RichText::new(text::SURFACE_FINE_HINT).color(DARK.warn);
        ui.add(egui::Label::new(warn).wrap());
    } else if !known {
        ui.weak(text::precision_custom(*voxel));
    }
}

/// For a selected map: the state of its surface with the settings chosen now; for a
/// selected surface also its own state.
fn selection_state(ui: &mut Ui, state: &AppState) {
    let Some(map) = state.selected_map() else {
        return;
    };
    ui.add_space(4.0);
    // wrapped: a long state („veraltet – …“) must not widen the field
    ui.horizontal_wrapped(|ui| {
        ui.label(text::WITH_THIS_FINENESS);
        state_label(ui, surface_state(state, map, &state.surface_settings));
    });
    if let Selection::Surface(dir) = &state.selection {
        if let Some(surface) = map.surfaces.iter().find(|s| s.dir == *dir) {
            ui.horizontal_wrapped(|ui| {
                ui.label(text::THIS_SURFACE);
                state_label(ui, own_surface_state(state, surface));
            });
        }
    }
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

/// The main button – „Oberfläche rechnen“ with a map (or a surface of it) selected, else
/// „Alle Oberflächen rechnen“ – and, with a map selected, „Alle Oberflächen rechnen“ quietly
/// below it. Locked while a step runs, while the scan cannot be read, for invalid settings,
/// for a map without `map.json` or that no longer fits its run (its surface would be stale at
/// once), and with nothing to compute; the reason shows on hover, and below the button unless
/// the field shows it already (the progress of the running step). The maps „Alle Oberflächen
/// rechnen“ leaves out because they wait for „Daten“ are named below it.
fn compute_buttons(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let busy = state.job.as_ref().map(|j| text::busy(&j.label));
    let invalid = state
        .surface_settings
        .config
        .check()
        .err()
        .map(|e| e.to_string());
    // until the maps are checked a stale surface counts as fitting: the button then waits
    // for the check instead of saying that all is done
    let pending = state.map_status_pending;
    let waiting = state.maps_waiting_for_data();
    let wait = (!pending && waiting > 0).then(|| text::maps_wait_for_data(waiting));
    let nothing = (!pending && state.maps_to_surface().is_empty()).then(|| {
        wait.clone()
            .unwrap_or_else(|| text::ALL_SURFACED.to_string())
    });
    let all_locked = busy.clone().or(invalid.clone()).or(nothing.clone());
    let selected = state.selected_map();
    let below = match selected {
        Some(map) => {
            let no_map = map
                .settings
                .is_none()
                .then(|| text::SURFACE_NEEDS_MAP.to_string());
            let scan_blocked = state
                .map(&map.dir)
                .and_then(|(scan, _, _)| blocked(state, scan));
            let map_stale = (surface_status(state, map) == SurfaceStatus::MapStale)
                .then(|| text::STATUS_MAP_STALE_HINT.to_string());
            let locked = scan_blocked
                .or(invalid.clone())
                .or(no_map.clone())
                .or(map_stale.clone());
            if main_button(ui, text::STEP_SURFACE, locked, text::COMPUTE_SURFACE_HINT) {
                *action = Some(Action::RunSurface(map.dir.clone()));
            }
            if quiet_button(ui, text::COMPUTE_ALL_SURFACES, all_locked) {
                *action = Some(Action::ComputeAllSurfaces);
            }
            if busy.is_some() {
                None
            } else {
                invalid
                    .or(no_map)
                    .map(|e| text::cannot_compute(&e))
                    .or(map_stale)
            }
        }
        None => {
            let hover = text::COMPUTE_SURFACES_HINT;
            if main_button(ui, text::COMPUTE_ALL_SURFACES, all_locked, hover) {
                *action = Some(Action::ComputeAllSurfaces);
            }
            match (busy, invalid) {
                (Some(_), _) => None,
                (None, Some(e)) => Some(text::cannot_compute(&e)),
                (None, None) if pending => Some(text::MAP_STATUS_PENDING.to_string()),
                // „all done“, or the maps that wait for „Daten“
                (None, None) => nothing.clone().or(wait),
            }
        }
    };
    let mut lines: Vec<String> = below.into_iter().collect();
    // all done, but with another fineness than the one chosen now: how to get it
    if selected.is_none() && nothing.is_some() && other_fineness(state) {
        lines.push(text::SURFACE_RECOMPUTE_HINT.to_string());
    }
    below_lines(ui, lines);
}

/// Whether a map with a current surface has none of the voxel size chosen now.
fn other_fineness(state: &AppState) -> bool {
    let voxel = state.surface_settings.config.voxel;
    maps(state).any(|(_, _, map)| {
        surface_status(state, map) == SurfaceStatus::Done
            && !map.surfaces.iter().any(|s| {
                !s.stale
                    && s.settings
                        .as_ref()
                        .is_some_and(|s| (s.config.voxel - voxel).abs() < 1e-9)
            })
    })
}

/// Under „Erweitert“: „ohne Fotos“, for the next surface.
fn advanced_content(ui: &mut Ui, settings: &mut SurfaceSettings) {
    ui.strong(text::SURFACE_SETTINGS);
    ui.weak(text::SURFACE_SETTINGS_HINT);
    let mut no_photos = !settings.texture;
    if ui
        .checkbox(&mut no_photos, text::SURFACE_NO_PHOTOS)
        .changed()
    {
        settings.texture = !no_photos;
    }
}
