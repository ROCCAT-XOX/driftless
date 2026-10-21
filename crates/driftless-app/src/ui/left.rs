//! The left panel: the project tree, project → scans → runs („SLAM 001“ …) → maps → clouds.

use super::{text, Action, AppState, ScanNode, Selection};
use crate::project::{MapInfo, RunInfo, ScanState};
use eframe::egui::{self, collapsing_header::CollapsingState, Id, Response, RichText, Ui};
use std::path::Path;

pub fn show(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let mut action = None;
    ui.heading(text::PANEL_PROJECT);
    let Some(project) = &state.project else {
        ui.label(text::NO_PROJECT);
        if ui.button(text::NEW_PROJECT).clicked() {
            action = Some(Action::PickNewProject);
        }
        if ui.button(text::OPEN_PROJECT).clicked() {
            action = Some(Action::PickOpenProject);
        }
        return action;
    };
    ui.label(RichText::new(&project.name).strong())
        .on_hover_text(project.dir.display().to_string());
    if ui.button(text::ADD_SCAN).clicked() {
        action = Some(Action::PickAddScan);
    }
    ui.separator();
    egui::ScrollArea::both().auto_shrink(false).show(ui, |ui| {
        if state.scans.is_empty() {
            ui.weak(text::NO_SCANS);
        }
        for scan in &state.scans {
            scan_node(ui, state, scan, &mut action);
        }
    });
    action
}

fn scan_node(ui: &mut Ui, state: &AppState, scan: &ScanNode, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Scan(scan.id.clone());
    let header = branch(
        ui,
        Id::new(("scan", &scan.id)),
        |ui| match &scan.state {
            ScanState::Ready => ui.selectable_label(selected, &scan.name),
            ScanState::Missing(reason) => {
                let label = text::scan_missing(&scan.name);
                let weak = ui.visuals().weak_text_color();
                ui.selectable_label(selected, RichText::new(label).color(weak))
                    .on_hover_text(reason)
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
        let r = ui.selectable_label(selected, &run.label);
        r.on_hover_text(run.dir.display().to_string())
    };
    let header = if run.cloud.is_none() && run.maps.is_empty() {
        leaf(ui, header)
    } else {
        branch(ui, Id::new(("run", &run.dir)), header, |ui| {
            if let Some(cloud) = &run.cloud {
                cloud_leaf(ui, state, cloud, action);
            }
            for map in &run.maps {
                map_node(ui, state, map, action);
            }
        })
    };
    if header.clicked() {
        *action = Some(Action::Select(Selection::Run(run.dir.clone())));
    }
}

fn map_node(ui: &mut Ui, state: &AppState, map: &MapInfo, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Map(map.dir.clone());
    let header = |ui: &mut Ui| {
        ui.selectable_label(selected, &map.label)
            .on_hover_text(map.dir.display().to_string())
    };
    let header = if map.clouds.is_empty() {
        leaf(ui, header)
    } else {
        branch(ui, Id::new(("map", &map.dir)), header, |ui| {
            for cloud in &map.clouds {
                cloud_leaf(ui, state, cloud, action);
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
        ui.selectable_label(selected, name)
            .on_hover_text(cloud.display().to_string())
    });
    if r.clicked() {
        *action = Some(Action::Select(Selection::Cloud(cloud.to_path_buf())));
    }
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
fn leaf(ui: &mut Ui, entry: impl FnOnce(&mut Ui) -> Response) -> Response {
    ui.horizontal(|ui| {
        ui.add_space(ui.spacing().indent);
        entry(ui)
    })
    .inner
}
