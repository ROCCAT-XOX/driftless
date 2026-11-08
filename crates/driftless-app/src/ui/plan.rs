//! The plan shown in the field „Planung“: its state, reading it from its folder, and the part of
//! the field that belongs to it (the layers with their numbers, „Draufsicht“, the hints of the
//! plan and „DXF …“). The surface it is drawn on is [`super::plan_view`].
//!
//! What the view draws is the [`Drawing`] of the DXF export, made with the layers as they are
//! switched here ([`refresh_drawing`]), so the plan and its DXF cannot differ. Only the plan
//! frame is drawn here; the placement „Wie die Punktwolke“ is a choice of the export.

use super::plan_view::PlanCamera;
use super::theme::{icon, DARK};
use super::{text, Action, AppState, Selection, Step};
use driftless_export::dxf::Drawing;
use driftless_steps::export::ExportSource;
use driftless_steps::objects::{ObjectKind, Objects};
use driftless_steps::plan::{read_plan, PlanReport, TOP_VIEW_FILE};
use driftless_steps::plan_drawing::{
    opening_counts, plan_drawing, PlanExport, PlanPlacement, LAYERS,
};
use driftless_steps::StepError;
use eframe::egui::{self, RichText, Ui};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// The layer with the note that it is set by hand for now.
const PIPE_LAYER: &str = "Rohr";
/// Height of a row of the layer list (points).
const LAYER_ROW: f32 = 18.0;

/// What the central area of „Planung“ shows while a plan is there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlanMode {
    /// The plan, in 2D.
    #[default]
    Plan,
    /// The 3D view, as in the other steps.
    View3d,
}

/// A plan as read from its folder.
#[derive(Debug, Clone)]
pub struct ShownPlan {
    pub dir: PathBuf,
    pub report: PlanReport,
    /// The objects of its map or model: they belong to the map, so a plan computed anew keeps
    /// them.
    pub objects: Objects,
    /// In the frame of the plan, with the layers as switched ([`refresh_drawing`]).
    pub drawing: Drawing,
    /// The picture from above; `None` without a readable file.
    pub top_view: Option<egui::ColorImage>,
}

/// The field „Planung“ and the plan shown.
#[derive(Debug, Clone)]
pub struct PlanState {
    /// The map or model folder of the plan computed or shown last.
    pub owner: Option<PathBuf>,
    pub shown: Option<Arc<ShownPlan>>,
    pub mode: PlanMode,
    /// The layers switched off; they stay off for the next plan and go into its DXF.
    pub hidden: BTreeSet<String>,
    /// Draw the picture from above under the plan.
    pub top_view: bool,
    /// „Mehrere Stockwerke gefunden“: the map or model asked about and the storeys to choose.
    pub storeys: Option<(PathBuf, Vec<driftless_plan::Storey>)>,
    /// The object marked in the plan.
    pub selected_object: Option<u32>,
    pub camera: PlanCamera,
}

impl Default for PlanState {
    fn default() -> Self {
        PlanState {
            owner: None,
            shown: None,
            mode: PlanMode::default(),
            hidden: BTreeSet::new(),
            top_view: true,
            storeys: None,
            selected_object: None,
            camera: PlanCamera::default(),
        }
    }
}

/// Whether the central area shows the plan: in „Planung“, in the mode Plan, once there is one.
pub fn on_screen(state: &AppState) -> bool {
    state.step == Step::Planung && state.plan.mode == PlanMode::Plan && state.plan.shown.is_some()
}

/// Whether the switch „Plan · 3D“ is there: in „Planung“, once there is a plan.
pub fn switch_shown(state: &AppState) -> bool {
    state.step == Step::Planung && state.plan.shown.is_some()
}

/// The folder of the map or model a plan folder lies in.
fn owner_of(dir: &Path) -> &Path {
    dir.parent().unwrap_or(dir)
}

/// Reads the plan in `dir` with the objects of its map or model. An objects file that cannot be
/// read fails it, as it fails the export: a plan does not silently lose objects. The picture is
/// optional. Off the UI thread: it decodes a PNG.
pub fn load(dir: &Path) -> Result<ShownPlan, StepError> {
    let report = read_plan(dir)?;
    let objects = Objects::load(owner_of(dir))?;
    let drawing = plan_drawing(&report, &objects, &PlanExport::default());
    Ok(ShownPlan {
        dir: dir.to_path_buf(),
        report,
        objects,
        drawing,
        top_view: read_top_view(&dir.join(TOP_VIEW_FILE)),
    })
}

fn read_top_view(path: &Path) -> Option<egui::ColorImage> {
    let bytes = fs::read(path).ok()?;
    let image = image::load_from_memory_with_format(&bytes, image::ImageFormat::Png)
        .ok()?
        .to_rgba8();
    let size = [image.width() as usize, image.height() as usize];
    Some(egui::ColorImage::from_rgba_unmultiplied(
        size,
        image.as_raw(),
    ))
}

/// Shows `plan`, read in the background. Another plan than the one shown is fitted to the view
/// at its first frame, the plan shown read again stays where it is.
pub fn set_shown(state: &mut PlanState, plan: ShownPlan) {
    let same = state.shown.as_ref().is_some_and(|s| s.dir == plan.dir);
    if !same {
        state.camera = PlanCamera::default();
        state.selected_object = None;
    }
    state.owner = Some(owner_of(&plan.dir).to_path_buf());
    state.shown = Some(Arc::new(plan));
    refresh_drawing(state);
}

/// Draws the plan shown anew: after a change of its objects or layers.
pub fn refresh_drawing(state: &mut PlanState) {
    let hidden: Vec<String> = state.hidden.iter().cloned().collect();
    let Some(shown) = &mut state.shown else {
        return;
    };
    let export = PlanExport {
        placement: PlanPlacement::Walls,
        hidden,
    };
    let drawing = plan_drawing(&shown.report, &shown.objects, &export);
    Arc::make_mut(shown).drawing = drawing;
}

/// The plan shown takes `objects`, the ones of its map or model as they are now.
pub fn set_objects(state: &mut PlanState, objects: Objects) {
    if let Some(shown) = &mut state.shown {
        Arc::make_mut(shown).objects = objects;
    }
    refresh_drawing(state);
}

/// The colour of the layer `name` in the plan and in the 3D view (of the objects drawn there).
pub fn layer_colour(name: &str) -> egui::Color32 {
    let index = LAYERS.iter().find(|&&(n, _)| n == name).map_or(0, |l| l.1);
    super::plan_view::layer_colour(index)
}

/// Switches a layer of the plan shown (and of the plans shown after it).
pub fn set_layer(state: &mut PlanState, layer: &str, on: bool) {
    if on {
        state.hidden.remove(layer);
    } else {
        state.hidden.insert(layer.to_string());
    }
    refresh_drawing(state);
}

/// A plan that is no longer in the tree (deleted, here or by another window) is shown no more.
pub(super) fn sync_with_tree(state: &mut AppState) {
    let Some(dir) = state.plan.shown.as_ref().map(|s| s.dir.clone()) else {
        return;
    };
    if state.all_plans().any(|(_, p)| p.dir == dir) {
        return;
    }
    state.plan.shown = None;
    state.plan.selected_object = None;
    if state.selection == Selection::Plan(dir) {
        state.selection = Selection::None;
    }
}

/// Takes the plan out of the view if it lies in the folder `dir` („Aus der Ansicht nehmen“).
pub(super) fn unshow(state: &mut AppState, dir: &Path) {
    if state
        .plan
        .shown
        .as_ref()
        .is_some_and(|s| s.dir.starts_with(dir))
    {
        state.plan.shown = None;
        state.plan.selected_object = None;
    }
}

/// What the export of the plan in `dir` reads: the layers hidden while it is the plan shown.
pub fn export_source(state: &PlanState, dir: &Path) -> ExportSource {
    let shown = state.shown.as_ref().is_some_and(|s| s.dir == dir);
    ExportSource::Plan {
        dir: dir.to_path_buf(),
        export: PlanExport {
            placement: PlanPlacement::default(),
            hidden: if shown {
                state.hidden.iter().cloned().collect()
            } else {
                Vec::new()
            },
        },
    }
}

/// The layers in the order of [`LAYERS`] with what lies on them: wall faces, openings of a
/// kind (with the doors and windows set by hand, each once, as the plan draws them), rooms,
/// dimensions and objects of a kind, not the lines they are drawn with.
pub fn layer_counts(shown: &ShownPlan) -> Vec<(&'static str, usize)> {
    let plan = &shown.report.plan;
    let [doors, windows, unclear] = opening_counts(plan, &shown.objects);
    let object = |kind: ObjectKind| {
        shown
            .objects
            .all()
            .iter()
            .filter(|o| o.kind == kind)
            .count()
    };
    // in the order of LAYERS
    let counts = [
        plan.faces.len(),
        doors,
        windows,
        unclear,
        plan.rooms.len(),
        plan.dims.len(),
        object(ObjectKind::Measure),
        object(ObjectKind::Socket),
        object(ObjectKind::Connection),
        object(ObjectKind::Cable),
        object(ObjectKind::Pipe),
    ];
    LAYERS.iter().map(|&(name, _)| name).zip(counts).collect()
}

/// The part of the field for the plan shown: the layers, „Draufsicht“ and the hints of the plan.
/// Nothing while none is shown.
pub fn panel(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let Some(shown) = &state.plan.shown else {
        return;
    };
    ui.add_space(6.0);
    ui.strong(text::PLAN_LAYERS);
    ui.scope(|ui| {
        // eleven layers: closer together than the rows of the other fields
        ui.spacing_mut().interact_size.y = LAYER_ROW;
        ui.spacing_mut().item_spacing.y = 2.0;
        for (name, count) in layer_counts(shown) {
            ui.horizontal(|ui| {
                let mut on = !state.plan.hidden.contains(name);
                if ui
                    .checkbox(&mut on, text::plan_layer(name, count))
                    .changed()
                {
                    *action = Some(Action::SetPlanLayer {
                        layer: name.to_string(),
                        on,
                    });
                }
                if name == PIPE_LAYER {
                    ui.weak(text::PLAN_PIPE_HINT);
                }
            });
        }
    });
    ui.add_space(4.0);
    let mut top = state.plan.top_view;
    if ui.checkbox(&mut top, text::PLAN_TOP_VIEW).changed() {
        *action = Some(Action::SetTopView(top));
    }
    if shown.top_view.is_none() {
        ui.weak(text::PLAN_TOP_VIEW_MISSING);
    }
    if !shown.report.plan.hints.is_empty() {
        ui.add_space(4.0);
        ui.strong(text::PLAN_HINTS);
        for hint in &shown.report.plan.hints {
            let line = format!("{} {hint}", icon::HINT);
            ui.add(egui::Label::new(RichText::new(line).color(DARK.text_dim)).wrap());
        }
    }
}

/// „DXF …“ of the plan shown; nothing while none is shown.
pub fn dxf_button(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    if state.plan.shown.is_none() {
        return;
    }
    ui.add_space(4.0);
    if ui
        .button(text::PLAN_DXF)
        .on_hover_text(text::PLAN_DXF_HINT)
        .clicked()
    {
        *action = Some(Action::OpenPlanExport);
    }
}
