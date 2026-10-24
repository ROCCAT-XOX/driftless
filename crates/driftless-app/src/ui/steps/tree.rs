//! The project tree of „Daten“: scans → runs („SLAM 001“ …) → maps → clouds and surfaces
//! („Oberfläche 001“, a stale one with „veraltet – Karte oder Lauf neu gerechnet“), and per run
//! its history („Verlauf“: each state with number, kind, time and start↔end gap). Names too
//! long for the field end in „…“; the full name and the folder show on hover.

use crate::project::{MapInfo, RunInfo, ScanState, SurfaceInfo};
use crate::ui::{text, Action, AppState, ScanNode, Selection};
use eframe::egui::{self, collapsing_header::CollapsingState, Id, Response, RichText, Ui};
use std::path::Path;

/// The scans of the project with everything below them.
pub fn show(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    for scan in &state.scans {
        scan_node(ui, state, scan, action);
    }
}

fn scan_node(ui: &mut Ui, state: &AppState, scan: &ScanNode, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Scan(scan.id.clone());
    let header = branch(
        ui,
        Id::new(("scan", &scan.id)),
        |ui| match &scan.state {
            ScanState::Ready => entry(ui, selected, scan.name.clone(), &scan.name),
            ScanState::Missing(reason) => {
                let label = text::scan_missing(&scan.name);
                let weak = ui.visuals().weak_text_color();
                let hover = format!("{label}\n{reason}");
                entry(ui, selected, RichText::new(label).color(weak), &hover)
            }
        },
        |ui| {
            if scan.runs.is_empty() {
                ui.weak(text::NO_RUNS);
            }
            for run in &scan.runs {
                run_node(ui, state, run, action);
            }
            if ui.small_button(text::ADD_RUN).clicked() {
                *action = Some(Action::PickAddRun(scan.id.clone()));
            }
        },
    );
    if header.clicked() {
        *action = Some(Action::Select(Selection::Scan(scan.id.clone())));
    }
}

fn run_node(ui: &mut Ui, state: &AppState, run: &RunInfo, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Run(run.dir.clone());
    let header = |ui: &mut Ui| {
        let hover = format!("{}\n{}", run.label, run.dir.display());
        entry(ui, selected, run.label.clone(), &hover)
    };
    let header = if run.cloud.is_none() && run.maps.is_empty() && run.history.is_empty() {
        leaf(ui, header)
    } else {
        branch(ui, Id::new(("run", &run.dir)), header, |ui| {
            if let Some(cloud) = &run.cloud {
                cloud_leaf(ui, state, cloud, action);
            }
            for map in &run.maps {
                map_node(ui, state, map, action);
            }
            if !run.history.is_empty() {
                history_node(ui, state, run, action);
            }
        })
    };
    if header.clicked() {
        *action = Some(Action::Select(Selection::Run(run.dir.clone())));
    }
}

/// „Verlauf“ with the states of the run, folded at first; the newest last.
fn history_node(ui: &mut Ui, state: &AppState, run: &RunInfo, action: &mut Option<Action>) {
    egui::CollapsingHeader::new(text::HISTORY)
        .id_salt(("verlauf", &run.dir))
        .default_open(false)
        .show(ui, |ui| {
            for e in &run.history {
                let selection = Selection::History {
                    run: run.dir.clone(),
                    nr: e.nr,
                };
                let name = text::history_entry(e, state.history_gaps.get(&e.dir).copied());
                let hover = format!("{name}\n{}", e.dir.display());
                let r = leaf(ui, |ui| {
                    entry(ui, state.selection == selection, name.clone(), &hover)
                });
                if r.clicked() {
                    *action = Some(Action::Select(selection));
                }
            }
        });
}

fn map_node(ui: &mut Ui, state: &AppState, map: &MapInfo, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Map(map.dir.clone());
    let header = |ui: &mut Ui| {
        let hover = format!("{}\n{}", map.label, map.dir.display());
        entry(ui, selected, map.label.clone(), &hover)
    };
    let header = if map.clouds.is_empty() && map.surfaces.is_empty() {
        leaf(ui, header)
    } else {
        branch(ui, Id::new(("map", &map.dir)), header, |ui| {
            for cloud in &map.clouds {
                cloud_leaf(ui, state, cloud, action);
            }
            for surface in &map.surfaces {
                surface_leaf(ui, state, surface, action);
            }
        })
    };
    if header.clicked() {
        *action = Some(Action::Select(Selection::Map(map.dir.clone())));
    }
}

fn cloud_leaf(ui: &mut Ui, state: &AppState, cloud: &Path, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Cloud(cloud.to_path_buf());
    let name = cloud.file_name().map_or_else(
        || cloud.display().to_string(),
        |n| n.to_string_lossy().into(),
    );
    let r = leaf(ui, |ui| {
        entry(ui, selected, name, &cloud.display().to_string())
    });
    if r.clicked() {
        *action = Some(Action::Select(Selection::Cloud(cloud.to_path_buf())));
    }
}

pub(super) fn surface_leaf(
    ui: &mut Ui,
    state: &AppState,
    surface: &SurfaceInfo,
    action: &mut Option<Action>,
) {
    let selected = state.selection == Selection::Surface(surface.dir.clone());
    let r = leaf(ui, |ui| {
        if surface.stale {
            let name = text::surface_stale(&surface.label);
            let hover = format!("{name}\n{}", surface.dir.display());
            let warn = ui.visuals().warn_fg_color;
            entry(ui, selected, RichText::new(name).color(warn), &hover)
        } else {
            let hover = format!("{}\n{}", surface.label, surface.dir.display());
            entry(ui, selected, surface.label.clone(), &hover)
        }
    });
    if r.clicked() {
        *action = Some(Action::Select(Selection::Surface(surface.dir.clone())));
    }
}

/// A selectable name, cut with „…“ where the panel ends, the full text on hover.
pub(super) fn entry(
    ui: &mut Ui,
    selected: bool,
    name: impl Into<egui::WidgetText>,
    hover: &str,
) -> Response {
    ui.add(egui::Button::selectable(selected, name).truncate())
        .on_hover_text(hover)
}

/// An entry with children, open by default; returns the header's response.
fn branch(
    ui: &mut Ui,
    id: Id,
    header: impl FnOnce(&mut Ui) -> Response,
    body: impl FnOnce(&mut Ui),
) -> Response {
    let (_, header, _) = CollapsingState::load_with_default_open(ui.ctx(), id, true)
        .show_header(ui, header)
        .body(body);
    header.inner
}

/// An entry without children, aligned with the headers of [`branch`].
pub(super) fn leaf(ui: &mut Ui, entry: impl FnOnce(&mut Ui) -> Response) -> Response {
    ui.horizontal(|ui| {
        ui.add_space(ui.spacing().indent);
        entry(ui)
    })
    .inner
}
