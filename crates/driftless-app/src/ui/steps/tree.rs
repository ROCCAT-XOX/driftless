//! What „Daten“ shows below the card of a scan: its runs („SLAM 001“ …) → maps → clouds and
//! surfaces („Oberfläche 001“, a stale one with „veraltet – Karte oder Lauf neu gerechnet“; a
//! hint of its report small below it, „Hinweis: Stellenweise offen“, in full on hover), and
//! per run its history („Verlauf“: each state with number, kind, time and start↔end gap). Below
//! the scans the models („Modell 001“): per scan of a model its number, name and how surely it
//! was placed (✓ sicher, ⚠ unsicher, ✗ gescheitert, the reason on hover) with overlap and
//! residual („Punktpaare“ before them if placed by hand, the fit of the pairs and the wall
//! residual – or „Wände nicht messbar“ – on hover), under an unsure or failed one „Von Hand
//! ausrichten“ ([`Action::StartPairs`]); then the point pairs of the model („Punktpaare“, each
//! with its switch, as the loops of a run; one the merge cannot use marked „nicht nutzbar“, why
//! on hover), `modell.las` and the surfaces of the model. Names too long for the field end in „…“;
//! the full name and the folder show on hover. A run, a map, a model and a surface have
//! „Exportieren …“ in their context menu (right click, [`Action::OpenExport`]).

use super::{badge, model_stale_hint};
use crate::project::{MapInfo, ModelInfo, RunInfo, SurfaceInfo};
use crate::render::SCAN_COLORS;
use crate::ui::theme::{icon, DARK};
use crate::ui::{text, Action, AppState, Selection};
use driftless_steps::export::ExportSource;
use driftless_steps::merge::{ModelMember, MODEL_LAS};
use eframe::egui::{
    self, collapsing_header::CollapsingState, Align, Color32, Id, Layout, Response, RichText,
    Sense, Ui,
};
use std::path::{Path, PathBuf};

/// Height of the line of a scan of a model (points).
const MEMBER_ROW_HEIGHT: f32 = 18.0;
/// Diameter of the colour dot of a scan of a model (points).
const DOT: f32 = 8.0;

/// A run of a scan with its cloud, maps and history, below the scan's card in „Daten“.
pub(super) fn run_node(ui: &mut Ui, state: &AppState, run: &RunInfo, action: &mut Option<Action>) {
    let selected = state.selection == Selection::Run(run.dir.clone());
    let label = sized(state, &run.label, &run.dir);
    let header = |ui: &mut Ui| {
        let hover = format!("{label}\n{}", run.dir.display());
        entry(ui, selected, label.clone(), &hover)
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
    // a run of the project goes to the trash; one the CLI made and was added by hand is only
    // taken out (its folder is the user's)
    let added = |id: &str| {
        let entry = state.project.as_ref().and_then(|p| p.scan(id));
        entry.is_some_and(|e| e.extra_runs.contains(&run.dir))
    };
    let manage = match state.run(&run.dir) {
        Some((scan, _)) if added(&scan.id) => Manage::RemoveRun {
            scan: scan.id.clone(),
            run: run.dir.clone(),
        },
        _ => Manage::Delete(run.dir.clone()),
    };
    entry_menu(
        &header,
        state,
        || ExportSource::Run(run.dir.clone()),
        manage,
        action,
    );
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
    let label = sized(state, &map.label, &map.dir);
    let header = |ui: &mut Ui| {
        let hover = format!("{label}\n{}", map.dir.display());
        entry(ui, selected, label.clone(), &hover)
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
    // a map the CLI wrote into the run folder is the run folder
    let in_run = state.run(&map.dir).is_some();
    let manage = if in_run {
        Manage::Locked(text::MAP_IN_RUN.to_string())
    } else {
        Manage::Delete(map.dir.clone())
    };
    entry_menu(
        &header,
        state,
        || ExportSource::Map(map.dir.clone()),
        manage,
        action,
    );
    if header.clicked() {
        *action = Some(Action::Select(Selection::Map(map.dir.clone())));
    }
}

/// A model with its scans, its cloud and its surfaces, below the scan cards in „Daten“.
pub(super) fn model_node(
    ui: &mut Ui,
    state: &AppState,
    model: &ModelInfo,
    action: &mut Option<Action>,
) {
    let selected = state.selection == Selection::Model(model.dir.clone());
    let stale = state.map_status.get(&model.dir).is_some_and(|s| s.stale);
    let label = sized(state, &model.label, &model.dir);
    let header = |ui: &mut Ui| {
        if stale {
            let name = if model.other_version() {
                text::model_stale_version(&label)
            } else {
                text::model_stale(&label)
            };
            let hover = format!(
                "{name}\n{}\n{}",
                model_stale_hint(model),
                model.dir.display()
            );
            let warn = ui.visuals().warn_fg_color;
            entry(ui, selected, RichText::new(name).color(warn), &hover)
        } else {
            let hover = format!("{label}\n{}", model.dir.display());
            entry(ui, selected, label.clone(), &hover)
        }
    };
    let header = branch(ui, Id::new(("model", &model.dir)), header, |ui| {
        for (k, member) in model.report.members.iter().enumerate() {
            member_rows(ui, state, k, member);
            if member.placed_against.is_some() && member.verdict != "sicher" {
                let r = ui.horizontal(|ui| {
                    ui.add_space(ui.spacing().indent + DOT + ui.spacing().item_spacing.x);
                    ui.small_button(text::PAIRS_START)
                        .on_hover_text(text::PAIRS_START_HINT)
                });
                if r.inner.clicked() {
                    *action = Some(Action::StartPairs {
                        model: model.dir.clone(),
                        member: k,
                    });
                }
            }
        }
        pair_list(ui, state, model, action);
        cloud_leaf(ui, state, &model.dir.join(MODEL_LAS), action);
        for surface in &model.surfaces {
            surface_leaf(ui, state, surface, action);
        }
    });
    let manage = Manage::Delete(model.dir.clone());
    entry_menu(
        &header,
        state,
        || ExportSource::Model(model.dir.clone()),
        manage,
        action,
    );
    if header.clicked() {
        *action = Some(Action::Select(Selection::Model(model.dir.clone())));
    }
}

/// Scan `k` of a model: its colour in „je Scan“, number and name, and at the right how surely
/// it was placed („Bezug“ for the reference); below it overlap and residual, and whether its
/// points were left out. The reason and what the numbers mean show on hover.
fn member_rows(ui: &mut Ui, state: &AppState, k: usize, m: &ModelMember) {
    let name = text::model_member(k + 1, &state.scan_name(&m.scan));
    let reference = m.placed_against.is_none();
    let (symbol, word, color) = match m.verdict.as_str() {
        _ if reference => (None, text::MERGE_REFERENCE, DARK.text_dim),
        "sicher" => (Some(icon::CHECK), text::VERDICT_SURE, DARK.ok),
        "unsicher" => (Some(icon::UNSURE), text::VERDICT_UNSURE, DARK.warn),
        _ => (Some(icon::FAILED), text::VERDICT_FAILED, DARK.error),
    };
    let mut hover = format!("{name}\n{}", m.scan.display());
    if let Some(why) = &m.reason {
        hover += &format!("\n{word}: {why}");
    }
    if !m.included {
        hover += &format!("\n{}", text::NOT_IN_MODEL);
        if m.verdict == "unsicher" {
            hover += &format!(" – {}", text::NOT_IN_MODEL_UNSURE);
        }
    }
    if let Some(rms) = m.pair_rms_mm {
        hover += &format!("\n{}", text::member_pairs_fit(rms));
    }
    if !reference {
        let walls = m
            .wall_residual_mm
            .map_or_else(|| text::WALLS_UNMEASURABLE.to_string(), text::member_walls);
        hover += &format!("\n{walls}\n{}", text::VERDICT_HINT);
    }
    let [r, g, b] = SCAN_COLORS[k % SCAN_COLORS.len()];
    let row = ui
        .horizontal(|ui| {
            ui.add_space(ui.spacing().indent);
            let size = egui::vec2(ui.available_width(), MEMBER_ROW_HEIGHT);
            ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                badge(ui, symbol, word, color);
                ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                    let (dot, _) = ui.allocate_exact_size(egui::vec2(DOT, DOT), Sense::hover());
                    ui.painter()
                        .circle_filled(dot.center(), DOT / 2.0, Color32::from_rgb(r, g, b));
                    ui.add(egui::Label::new(name.clone()).truncate().selectable(false));
                });
            });
        })
        .response;
    let mut rect = row.rect;
    let mut line = |ui: &mut Ui, what: String, color: Color32| {
        let r = ui.horizontal(|ui| {
            ui.add_space(ui.spacing().indent + DOT + ui.spacing().item_spacing.x);
            ui.add(egui::Label::new(RichText::new(what).color(color)).wrap());
        });
        rect = rect.union(r.response.rect);
    };
    if !reference {
        let numbers = if m.method == "punktpaare" {
            text::member_numbers_pairs(m.overlap, m.residual_median_mm)
        } else {
            text::member_numbers(m.overlap, m.residual_median_mm)
        };
        line(ui, numbers, DARK.text_dim);
    }
    if !m.included {
        line(ui, text::NOT_IN_MODEL.to_string(), DARK.warn);
    }
    let id = ui.id().with(("modell_scan", k, &m.scan));
    ui.interact(rect, id, Sense::hover()).on_hover_text(hover);
}

/// „Punktpaare“: the point pairs that name two scans of `model`, each with its switch
/// ([`Action::SetPairEnabled`], locked while a switch is written), its scans and how many
/// points it has; its note and when it was added on hover. Nothing without pairs.
fn pair_list(ui: &mut Ui, state: &AppState, model: &ModelInfo, action: &mut Option<Action>) {
    let pairs: Vec<_> = state.pairs_of(model).collect();
    if pairs.is_empty() {
        return;
    }
    egui::CollapsingHeader::new(text::PAIRS_LIST)
        .id_salt(("punktpaare", &model.dir))
        .default_open(true)
        .show(ui, |ui| {
            for (index, e, [a, b]) in pairs {
                let nr = index + 1;
                ui.horizontal(|ui| {
                    let mut on = e.enabled;
                    let can = !state.pairs_writing;
                    let switch = ui.add_enabled(can, egui::Checkbox::without_text(&mut on));
                    switch.widget_info(|| {
                        egui::WidgetInfo::selected(
                            egui::WidgetType::Checkbox,
                            can,
                            on,
                            text::pair_switch(nr),
                        )
                    });
                    if switch.changed() {
                        *action = Some(Action::SetPairEnabled { index, enabled: on });
                    }
                    switch.on_hover_text(text::PAIR_SWITCH_HINT);
                    let mut row = text::pair_row(nr, a + 1, b + 1, e.points_a.len());
                    let unusable = e.check().err();
                    let color = match (&unusable, e.enabled) {
                        (Some(_), _) => DARK.warn,
                        (None, true) => DARK.text,
                        (None, false) => DARK.text_dim,
                    };
                    if unusable.is_some() {
                        row += &format!(" · {}", text::PAIR_UNUSABLE);
                    }
                    let hover = [
                        unusable.as_deref().unwrap_or(""),
                        e.note.as_str(),
                        e.added.as_str(),
                    ]
                    .into_iter()
                    .filter(|t| !t.is_empty())
                    .collect::<Vec<_>>()
                    .join("\n");
                    let label = ui.add(
                        egui::Label::new(RichText::new(row).color(color))
                            .truncate()
                            .selectable(false),
                    );
                    if !hover.is_empty() {
                        label.on_hover_text(hover);
                    }
                });
            }
        });
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
    let label = sized(state, &surface.label, &surface.dir);
    // its hints for the user in full on hover, below the folder
    let hover = |name: &str| {
        let mut hover = format!("{name}\n{}", surface.dir.display());
        for hint in &surface.hints {
            hover += &format!("\n\n{hint}");
        }
        hover
    };
    let r = leaf(ui, |ui| {
        if surface.stale {
            let name = text::surface_stale(&label);
            let warn = ui.visuals().warn_fg_color;
            entry(
                ui,
                selected,
                RichText::new(&name).color(warn),
                &hover(&name),
            )
        } else {
            entry(ui, selected, label.clone(), &hover(&label))
        }
    });
    for hint in &surface.hints {
        hint_line(ui, hint);
    }
    let manage = Manage::Delete(surface.dir.clone());
    entry_menu(
        &r,
        state,
        || ExportSource::Surface(surface.dir.clone()),
        manage,
        action,
    );
    if r.clicked() {
        *action = Some(Action::Select(Selection::Surface(surface.dir.clone())));
    }
}

/// A hint of a surface for the user below its entry: small and muted (a hint, not an error),
/// only its title ([`text::surface_hint_short`]), the whole hint on hover.
fn hint_line(ui: &mut Ui, hint: &str) {
    ui.horizontal(|ui| {
        ui.add_space(ui.spacing().indent + ui.spacing().button_padding.x);
        let line = format!("{} {}", icon::HINT, text::surface_hint_short(hint));
        let line = RichText::new(line).small().color(DARK.text_dim);
        ui.add(egui::Label::new(line).wrap()).on_hover_text(hint);
    });
}

/// What the context menu of an entry offers besides „Exportieren …“.
enum Manage {
    /// „Löschen …“ of the folder (to the trash), locked while it is shown or computed.
    Delete(PathBuf),
    /// „Löschen …“ locked for good, with the reason.
    Locked(String),
    /// „Aus Projekt entfernen“ of a run the CLI made: only the entry.
    RemoveRun { scan: String, run: PathBuf },
}

/// The context menu (right click) of `entry`: „Exportieren …“ opens the window for `source`;
/// then „Löschen …“ (asks first, [`Action::AskDelete`]; locked with the reason while a step
/// works on the folder or on something depending on it) or „Aus Projekt entfernen“.
fn entry_menu(
    entry: &Response,
    state: &AppState,
    source: impl FnOnce() -> ExportSource,
    manage: Manage,
    action: &mut Option<Action>,
) {
    entry.context_menu(|ui| {
        if ui.button(text::EXPORT).clicked() {
            *action = Some(Action::OpenExport(source()));
        }
        ui.separator();
        match manage {
            Manage::Delete(dir) => {
                // what is only shown, the question offers to take out of the view
                let blocked = state.delete_computing(&dir);
                let r = ui.add_enabled(blocked.is_none(), egui::Button::new(text::DELETE));
                if r.clicked() {
                    *action = Some(Action::AskDelete(dir));
                }
                if let Some(why) = blocked {
                    r.on_disabled_hover_text(why);
                }
            }
            Manage::Locked(why) => {
                ui.add_enabled(false, egui::Button::new(text::DELETE))
                    .on_disabled_hover_text(why);
            }
            Manage::RemoveRun { scan, run } => {
                if ui.button(text::REMOVE_RUN).clicked() {
                    *action = Some(Action::RemoveRun { scan, run });
                }
            }
        }
    });
}

/// The name of an entry with the size of its folder, once measured („Karte 001 · 640 MB“).
fn sized(state: &AppState, label: &str, dir: &Path) -> String {
    match state.folder_size(dir) {
        Some(bytes) => text::with_size(label, bytes),
        None => label.to_string(),
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
