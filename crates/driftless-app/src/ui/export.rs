//! The window „Exportieren“: what (the source picked in the tree, or what the view shows), the
//! format (those of its kind, LAZ, glb or CSV first), „Ziel wählen …“ and „Abbrechen“.
//!
//! „Ziel wählen …“ opens the save dialog of the system ([`Action::PickExportTarget`]), which
//! asks itself before replacing the file chosen. The other files the export would replace (the
//! `.mtl` and the pages of an OBJ, or the file chosen once its extension was added) are asked
//! about here, by name ([`target_picked`]); only „Ersetzen“ exports over them. The export then
//! runs as a step ([`Action::RunExport`], with progress and „Abbrechen“ in the status bar),
//! also without a project.

use super::{text, Action, AppState};
use driftless_steps::export::{
    check_export, existing_targets, ExportFormat, ExportKind, ExportRequest, ExportSource,
};
use driftless_steps::merge::MODEL_LAS;
use eframe::egui::{self, Context, Ui};
use std::path::{Path, PathBuf};

/// Width of the window (points).
const WIDTH: f32 = 340.0;
/// The mesh of a surface folder; the view shows a surface by this file.
const SURFACE_GLB: &str = "oberflaeche.glb";
/// What the export of a map reads: the first of these there.
const MAP_CLOUDS: [&str; 2] = ["color.las", "clean.las"];
/// What the export of a run reads.
const RUN_TRAJ: &str = "traj.tum";

/// The window „Exportieren“ while it is open.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportDialog {
    /// Was: the source and its name („Karte 001 – color.las“, „Oberfläche 001“ …).
    pub choices: Vec<(ExportSource, String)>,
    /// Index into [`ExportDialog::choices`].
    pub chosen: usize,
    /// Of the kind of the chosen source.
    pub format: ExportFormat,
    /// The question „Ersetzen?“, while it is asked.
    pub replace: Option<ReplaceQuestion>,
}

/// „Diese Dateien gibt es schon: … Ersetzen?“
#[derive(Debug, Clone, PartialEq)]
pub struct ReplaceQuestion {
    /// The export „Ersetzen“ starts (with [`ExportRequest::overwrite`]).
    pub request: ExportRequest,
    /// The files it would replace that the save dialog did not ask about.
    pub files: Vec<PathBuf>,
}

/// The format offered first: LAZ for points, glb for a mesh, CSV for a trajectory.
pub fn default_format(kind: ExportKind) -> ExportFormat {
    match kind {
        ExportKind::Points => ExportFormat::Laz,
        ExportKind::Mesh => ExportFormat::Glb,
        ExportKind::Trajectory => ExportFormat::Csv,
    }
}

impl ExportDialog {
    /// The window with `choices`, the first chosen; `None` without any.
    pub fn new(choices: Vec<(ExportSource, String)>) -> Option<Self> {
        let kind = choices.first()?.0.kind();
        Some(ExportDialog {
            choices,
            chosen: 0,
            format: default_format(kind),
            replace: None,
        })
    }

    /// The chosen source.
    pub fn source(&self) -> &ExportSource {
        &self.choices[self.chosen.min(self.choices.len() - 1)].0
    }

    /// The formats of the chosen source's kind, in the order of [`ExportFormat::ALL`].
    pub fn formats(&self) -> Vec<ExportFormat> {
        let kind = self.source().kind();
        ExportFormat::ALL
            .into_iter()
            .filter(|f| f.kind() == kind)
            .collect()
    }

    /// Chooses source `k`; a source of another kind starts with its first format.
    fn choose(&mut self, k: usize) {
        let kind = self.source().kind();
        self.chosen = k;
        if self.source().kind() != kind {
            self.format = default_format(self.source().kind());
        }
        self.replace = None;
    }
}

/// The name of `source` in the window: as the tree names it („Karte 001“, „Modell 001“,
/// „SLAM 001“, „Oberfläche 001“), with the file that is read („– color.las“); a file the tree
/// does not know by its name.
pub fn source_name(state: &AppState, source: &ExportSource) -> String {
    match source {
        ExportSource::Map(d) => {
            let map = state.map(d).map(|(_, _, m)| m);
            let label = map.map_or_else(|| folder(d), |m| m.label.clone());
            let cloud = MAP_CLOUDS.into_iter().find(|c| match map {
                Some(m) => m.clouds.contains(&d.join(c)),
                None => d.join(c).is_file(),
            });
            match cloud {
                Some(c) => text::export_choice(&label, c),
                None => label,
            }
        }
        ExportSource::Model(d) => {
            let label = state
                .model(d)
                .map_or_else(|| folder(d), |m| m.label.clone());
            text::export_choice(&label, MODEL_LAS)
        }
        ExportSource::Surface(d) | ExportSource::EditedSurface(d) => {
            surface_label(state, d).unwrap_or_else(|| folder(d))
        }
        ExportSource::Run(d) => {
            let label = state
                .run(d)
                .map_or_else(|| folder(d), |(_, r)| r.label.clone());
            text::export_choice(&label, RUN_TRAJ)
        }
        ExportSource::PointFile(f) => {
            let owner = state.scans.iter().flat_map(|s| &s.runs).find_map(|r| {
                if r.cloud.as_ref() == Some(f) {
                    return Some(r.label.clone());
                }
                r.maps
                    .iter()
                    .find(|m| m.clouds.contains(f))
                    .map(|m| m.label.clone())
            });
            match owner {
                Some(label) => text::export_choice(&label, &folder(f)),
                None => folder(f),
            }
        }
        ExportSource::MeshFile(f) => folder(f),
    }
}

/// The label of the surface folder `dir` in the tree (of a map or of a model).
fn surface_label(state: &AppState, dir: &Path) -> Option<String> {
    let maps = state
        .scans
        .iter()
        .flat_map(|s| &s.runs)
        .flat_map(|r| &r.maps)
        .flat_map(|m| &m.surfaces);
    let models = state.models.iter().flat_map(|m| &m.surfaces);
    maps.chain(models)
        .find(|s| s.dir == dir)
        .map(|s| s.label.clone())
}

/// The name of a file or folder, else all of the path.
fn folder(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// What „Exportieren …“ in ☰ offers: the cloud the view shows ([`ExportSource::detect`], so
/// that `modell.las` is the model and keeps the names of its scans) and the surface (its folder
/// for an `oberflaeche.glb`), with their names. Empty while neither is shown, or for a
/// synthetic cloud.
pub fn shown_sources(state: &AppState) -> Vec<(ExportSource, String)> {
    let cloud = state
        .cloud
        .as_ref()
        .and_then(|c| ExportSource::detect(&c.path).ok())
        .filter(|s| s.kind() == ExportKind::Points);
    let surface = state.surface.as_ref().and_then(|s| shown_surface(&s.path));
    cloud
        .into_iter()
        .chain(surface)
        .map(|s| {
            let name = source_name(state, &s);
            (s, name)
        })
        .collect()
}

/// The source of the surface file the view shows: the surface folder of an
/// `oberflaeche.glb`, else the file.
fn shown_surface(path: &Path) -> Option<ExportSource> {
    let folder = path
        .parent()
        .filter(|_| path.file_name().is_some_and(|n| n == SURFACE_GLB))
        .and_then(|dir| ExportSource::detect_as(dir, ExportKind::Mesh).ok());
    folder
        .into_iter()
        .chain(ExportSource::detect(path).ok())
        .find(|s| s.kind() == ExportKind::Mesh)
}

/// The save dialog gave `picked`: the request for it, with the format's extension added if it
/// has another or none. Refused (`Err`) if the format does not fit or a target is a file of
/// the source ([`check_export`]). The file the dialog gave back it has asked about itself;
/// if the export would replace others, the window asks first ([`ExportDialog::replace`],
/// `Ok(None)`), else the export can start (`Ok(Some(RunExport))`, replacing what the dialog
/// asked about). Nothing without an open window.
pub fn target_picked(state: &mut AppState, picked: PathBuf) -> Result<Option<Action>, String> {
    let Some(dialog) = state.export.as_mut() else {
        return Ok(None);
    };
    dialog.replace = None;
    let format = dialog.format;
    let mut request = ExportRequest {
        source: dialog.source().clone(),
        out: with_extension(&picked, format),
        format,
        overwrite: false,
    };
    check_export(&request).map_err(|e| e.to_string())?;
    let existing = existing_targets(&request);
    let files: Vec<PathBuf> = existing.iter().filter(|p| **p != picked).cloned().collect();
    request.overwrite = !existing.is_empty();
    if files.is_empty() {
        return Ok(Some(Action::RunExport(request)));
    }
    dialog.replace = Some(ReplaceQuestion { request, files });
    Ok(None)
}

/// `path` if its extension is that of `format` (any case), else with it added.
fn with_extension(path: &Path, format: ExportFormat) -> PathBuf {
    if ExportFormat::for_path(path, format.kind()) == Some(format) {
        return path.to_path_buf();
    }
    let mut name = path.as_os_str().to_os_string();
    name.push(".");
    name.push(format.extension());
    name.into()
}

/// Shows the window while [`AppState::export`] is open. „Abbrechen“ and its × close it,
/// „Abbrechen“ of the question only the question. „Ziel wählen …“ is locked while a step runs.
pub fn show(ctx: &Context, state: &mut AppState) -> Option<Action> {
    let busy = state.job.as_ref().map(|j| j.label.clone());
    let dialog = state.export.as_mut()?;
    if dialog.choices.is_empty() {
        // nothing to export (made by hand): nothing to show
        state.export = None;
        return None;
    }
    let mut action = None;
    let mut open = true;
    let mut close = false;
    egui::Window::new(text::EXPORT_TITLE)
        .id(egui::Id::new("exportieren"))
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .default_width(WIDTH)
        .show(ctx, |ui| match dialog.replace.clone() {
            Some(question) => {
                ui.label(text::export_replace_question(&question.files));
                ui.horizontal(|ui| {
                    if ui.button(text::EXPORT_REPLACE).clicked() {
                        action = Some(Action::ConfirmExport(question.request));
                    }
                    if ui.button(text::CANCEL).clicked() {
                        dialog.replace = None;
                    }
                });
            }
            None => {
                egui::Grid::new("export_wahl")
                    .num_columns(2)
                    .show(ui, |ui| {
                        let label = ui.label(text::EXPORT_WHAT);
                        what_choice(ui, dialog, label.id);
                        ui.end_row();
                        let label = ui.label(text::EXPORT_FORMAT);
                        format_choice(ui, dialog, label.id);
                        ui.end_row();
                    });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    let pick = ui.add_enabled(busy.is_none(), egui::Button::new(text::EXPORT_PICK));
                    if pick.clicked() {
                        action = Some(Action::PickExportTarget);
                    }
                    if let Some(label) = &busy {
                        pick.on_disabled_hover_text(text::busy(label));
                    }
                    if ui.button(text::CANCEL).clicked() {
                        close = true;
                    }
                });
            }
        });
    if !open || close {
        state.export = None;
    }
    action
}

/// Was: the name of the one source, or a choice of several.
fn what_choice(ui: &mut Ui, dialog: &mut ExportDialog, label: egui::Id) {
    if dialog.choices.len() == 1 {
        ui.label(&dialog.choices[0].1);
        return;
    }
    let mut chosen = dialog.chosen;
    egui::ComboBox::from_id_salt("export_was")
        .selected_text(
            dialog.choices[chosen.min(dialog.choices.len() - 1)]
                .1
                .as_str(),
        )
        .show_ui(ui, |ui| {
            for (k, (_, name)) in dialog.choices.iter().enumerate() {
                ui.selectable_value(&mut chosen, k, name);
            }
        })
        .response
        .labelled_by(label);
    if chosen != dialog.chosen {
        dialog.choose(chosen);
    }
}

/// Format: the formats of the chosen source's kind.
fn format_choice(ui: &mut Ui, dialog: &mut ExportDialog, label: egui::Id) {
    let formats = dialog.formats();
    egui::ComboBox::from_id_salt("export_format")
        .selected_text(dialog.format.label())
        .show_ui(ui, |ui| {
            for f in formats {
                ui.selectable_value(&mut dialog.format, f, f.label());
            }
        })
        .response
        .labelled_by(label);
}
