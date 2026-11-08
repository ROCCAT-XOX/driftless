//! Step 3 „Planung“, in the form of „Bilder“: a card per map and model with its status (nicht
//! gerechnet, rechnet, fertig ✓, veraltet) and its floor plans below it, „Mehrere Stockwerke
//! gefunden“ with a button per storey, what belongs to the plan shown ([`plan::panel`]: layers,
//! „Draufsicht“, hints, „DXF …“) and the measurements of the tool „Messen“ (not saved yet). At
//! the bottom the main button „Grundriss rechnen“ for the map or model chosen; with only one in
//! the project that one. A plan needs the map or model only, so raw data that are gone lock
//! nothing here.

use super::{
    badge, below_lines, card, fraction, heading, hint, job_progress, main_button, no_project,
    pinned_with, tree,
};
use crate::project::{MapInfo, ModelInfo, PlanInfo};
use crate::ui::theme::{self, icon, DARK};
use crate::ui::{plan, text, Action, AppState, Selection, Step};
use eframe::egui::{self, Align, Layout, RichText, Ui};
use std::path::{Path, PathBuf};

/// Height kept below the scrolling list of measurements for the hint and „Messungen löschen“
/// (points), and the least height of the list.
const BELOW_LIST: f32 = 120.0;
const MIN_LIST: f32 = 80.0;

/// Where the plans of a map or model stand, as its card shows it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanStatus {
    /// No plan yet.
    New,
    /// One is being computed, with its progress (0…1) if the step knows its total.
    Running(Option<f32>),
    /// One of its plans still fits.
    Done,
    /// It has plans, but none fits any more.
    Stale,
}

fn status(state: &AppState, owner: &Path, plans: &[PlanInfo]) -> PlanStatus {
    if matches!(&state.running, Some(Action::RunPlan { owner: d, .. }) if d == owner) {
        return PlanStatus::Running(state.job.as_ref().and_then(fraction));
    }
    if plans.iter().any(|p| !p.stale) {
        PlanStatus::Done
    } else if plans.is_empty() {
        PlanStatus::New
    } else {
        PlanStatus::Stale
    }
}

/// Every map of the project, in the order of the list, with its name in the field.
fn maps(state: &AppState) -> impl Iterator<Item = (String, &MapInfo)> {
    state.scans.iter().flat_map(|scan| {
        scan.runs.iter().flat_map(move |run| {
            run.maps
                .iter()
                .map(move |m| (text::map_of(&scan.name, &run.label, &m.label), m))
        })
    })
}

/// How many maps and models the field lists.
fn owner_count(state: &AppState) -> usize {
    maps(state).count() + state.models.len()
}

pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    heading(ui, text::STEP_PLAN);
    if state.project.is_none() {
        no_project(ui, &mut action);
    } else if owner_count(state) == 0 {
        hint(ui, text::EMPTY_NO_MAPS);
        if ui.button(text::GO_TO_DATA).clicked() {
            action = Some(Action::SetStep(Step::Daten));
        }
    } else {
        ui.weak(text::plans_line(state.all_plans().count()));
        pinned_with(
            ui,
            Step::Planung,
            false,
            &mut (state, &mut action),
            |ui, (state, action), _| {
                ui.add_space(4.0);
                owner_list(ui, state, action);
                storey_choice(ui, state, action);
                plan::panel(ui, state, action);
                job_progress(ui, state);
                ui.add_space(6.0);
                measurements(ui, state, false);
            },
            |ui, (state, action)| compute_button(ui, state, action),
        );
        return action;
    }
    ui.add_space(4.0);
    measurements(ui, state, true);
    action
}

/// A card per map and model, its plans below it. A click on a card chooses it (again: lets it
/// go), a click on a plan shows it.
fn owner_list(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    for (name, map) in maps(state) {
        let selection = Selection::Map(map.dir.clone());
        owner_card(ui, state, &name, &map.dir, &map.plans, selection, action);
    }
    for model in &state.models {
        let name = text::model_card(&model.label, model.report.members.len());
        let selection = Selection::Model(model.dir.clone());
        owner_card(
            ui,
            state,
            &name,
            &model.dir,
            &model.plans,
            selection,
            action,
        );
    }
}

fn owner_card(
    ui: &mut Ui,
    state: &AppState,
    name: &str,
    dir: &Path,
    plans: &[PlanInfo],
    selection: Selection,
    action: &mut Option<Action>,
) {
    let selected = state.selection == selection;
    let hover = format!("{name}\n{}", dir.display());
    let plan_status = status(state, dir, plans);
    let r = card(
        ui,
        ("grundriss", dir),
        selected,
        RichText::new(name),
        &hover,
        |ui| status_label(ui, plan_status),
    );
    if r.clicked() {
        *action = Some(Action::Select(if selected {
            Selection::None
        } else {
            selection
        }));
    }
    if !plans.is_empty() {
        ui.indent(("grundrisse", dir), |ui| {
            for plan in plans {
                tree::plan_leaf(ui, state, plan, action);
            }
        });
    }
}

/// The status at the right of a card.
fn status_label(ui: &mut Ui, status: PlanStatus) {
    let (symbol, word, color) = match status {
        PlanStatus::New => (None, text::NOT_COMPUTED, DARK.text_dim),
        PlanStatus::Running(_) => (Some(icon::RUNNING), text::STATUS_RUNNING, DARK.warn),
        PlanStatus::Done => (Some(icon::CHECK), text::STATUS_DONE, DARK.ok),
        PlanStatus::Stale => (Some(icon::STALE), text::STATUS_STALE, DARK.warn),
    };
    badge(ui, symbol, word, color);
}

/// „Mehrere Stockwerke gefunden“: a button per storey, once a map with several was computed.
fn storey_choice(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let Some((owner, storeys)) = &state.plan.storeys else {
        return;
    };
    ui.add_space(6.0);
    ui.strong(text::PLAN_SEVERAL_STOREYS);
    for (i, storey) in storeys.iter().enumerate() {
        if ui.button(text::storey_line(storey)).clicked() {
            *action = Some(Action::RunPlan {
                owner: owner.clone(),
                storey: Some(i + 1),
            });
        }
    }
}

/// The map or model „Grundriss rechnen“ works on: the one chosen (or whose plan or surface is
/// chosen), else the one of the last plan, else the only one of the project.
fn compute_target(state: &AppState) -> Option<PathBuf> {
    let chosen = state
        .selected_map()
        .map(|m| m.dir.clone())
        .or_else(|| state.selected_model().map(|m| m.dir.clone()));
    let known = |dir: &PathBuf| state.map(dir).is_some() || state.model(dir).is_some();
    chosen
        .or_else(|| state.plan.owner.clone().filter(known))
        .or_else(|| match (owner_count(state), maps(state).next()) {
            (1, Some((_, map))) => Some(map.dir.clone()),
            (1, None) => state.models.first().map(|m: &ModelInfo| m.dir.clone()),
            _ => None,
        })
}

/// The main button: locked while a step runs, without a chosen map or model and for a map that
/// was not finished; why shows on hover, and below it unless the field says so already.
fn compute_button(ui: &mut Ui, state: &AppState, action: &mut Option<Action>) {
    let busy = state.job.as_ref().map(|j| text::busy(&j.label));
    let target = compute_target(state);
    let incomplete = target
        .as_deref()
        .and_then(|dir| state.map(dir))
        .is_some_and(|(_, _, map)| map.incomplete);
    let why = busy
        .clone()
        .or_else(|| target.is_none().then(|| text::PLAN_PICK_OWNER.to_string()))
        .or_else(|| incomplete.then(|| text::PLAN_MAP_INCOMPLETE.to_string()));
    if main_button(
        ui,
        text::STEP_FLOOR_PLAN,
        why.clone(),
        text::PLAN_COMPUTE_HINT,
    ) {
        if let Some(owner) = target {
            *action = Some(Action::RunPlan {
                owner,
                storey: None,
            });
        }
    }
    let below = if busy.is_some() { None } else { why };
    below_lines(ui, below);
}

/// „Messungen“: a card per measurement with its distance, or how to make one; „Messungen
/// löschen“ below them. `scroll`: the field does not scroll itself (no map to list), so the
/// cards scroll here and the hint and the button stay in the field.
fn measurements(ui: &mut Ui, state: &mut AppState, scroll: bool) {
    ui.strong(text::MEASUREMENTS);
    if state.measurements.is_empty() {
        hint(ui, text::NO_MEASUREMENTS);
        return;
    }
    let cards = |ui: &mut Ui| {
        for (i, m) in state.measurements.iter().enumerate() {
            theme::card_frame(false).show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(RichText::new(icon::MEASURE).color(DARK.measure));
                    ui.label(text::measurement_name(i + 1));
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.label(RichText::new(text::meters(m.distance())).strong());
                    });
                });
            });
        }
    };
    if scroll {
        egui::ScrollArea::vertical()
            .id_salt("messungen")
            .max_height((ui.available_height() - BELOW_LIST).max(MIN_LIST))
            .show(ui, cards);
    } else {
        cards(ui);
    }
    hint(ui, text::MEASUREMENTS_NOT_SAVED);
    if ui.button(text::CLEAR_MEASUREMENTS).clicked() {
        state.measurements.clear();
        state.measure_a = None;
    }
}
