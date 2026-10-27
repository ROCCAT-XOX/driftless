//! The field „Scans zusammenführen“ in „Daten“: a tick per scan and, for a ticked one, which
//! of its maps to take (as the comparisons choose a run, see [`crate::ui::compare`]; preset
//! is the newest map of its newest run), „Auch unsichere übernehmen“, and „Zusammenführen“,
//! which is locked until two scans are ticked. The first ticked scan of the list is the
//! reference; its frame is the frame of the model.
//!
//! A scan can be merged once a map of it is finished: it has cleaned points (`clean.las`, or
//! `color.las` made from them) and still fits its run ([`AppState::map_status`]).

use super::main_button;
use crate::project::{MapInfo, MergePick};
use crate::ui::theme::DARK;
use crate::ui::{compare, text, Action, AppState, ScanNode};
use eframe::egui::{self, RichText, Ui};
use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

/// The clouds of a map that tell it has cleaned points (`clean.bin` beside them).
const CLEANED: [&str; 2] = ["clean.las", "color.las"];

/// What the field shows and what was chosen in it; kept while it is closed, so a merge that
/// failed can be started again as it was.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MergeState {
    pub open: bool,
    /// The ticked scans (ids).
    pub ticked: BTreeSet<String>,
    /// The map chosen per scan (id → map folder); one no longer offered is the preset again.
    pub chosen: BTreeMap<String, PathBuf>,
    pub include_unsure: bool,
    /// Ticks were preset (the first opening ticks every scan that can be merged).
    preset: bool,
}

/// A map that can be merged: finished, with cleaned points, still fitting its run.
pub fn mergeable(state: &AppState, map: &MapInfo) -> bool {
    let cleaned = map.clouds.iter().any(|c| {
        c.file_name()
            .is_some_and(|n| CLEANED.iter().any(|f| n == *f))
    });
    map.settings.is_some() && cleaned && !state.map_status.get(&map.dir).is_some_and(|s| s.stale)
}

/// The maps of `scan` that can be merged, as (run folder, map folder, „SLAM 001 · Karte 001“),
/// in the order of the tree: the last is the newest map of the newest run.
pub fn options(state: &AppState, scan: &ScanNode) -> Vec<((PathBuf, PathBuf), String)> {
    scan.runs
        .iter()
        .flat_map(|run| {
            run.maps
                .iter()
                .filter(|m| mergeable(state, m))
                .map(move |m| {
                    let label = format!("{} · {}", run.label, m.label);
                    ((run.dir.clone(), m.dir.clone()), label)
                })
        })
        .collect()
}

/// How many scans have a map that can be merged: from two on „Zusammenführen …“ shows.
pub fn mergeable_scans(state: &AppState) -> usize {
    state
        .scans
        .iter()
        .filter(|s| {
            s.runs
                .iter()
                .flat_map(|r| &r.maps)
                .any(|m| mergeable(state, m))
        })
        .count()
}

impl MergeState {
    /// Opens the field; the first time every scan that can be merged is ticked.
    pub fn open(&mut self, state: &AppState) {
        if !self.preset {
            self.ticked = state
                .scans
                .iter()
                .filter(|s| !options(state, s).is_empty())
                .map(|s| s.id.clone())
                .collect();
            self.preset = true;
        }
        self.open = true;
    }

    /// The map of `scan` to merge: the chosen one if it is still offered, else the newest.
    fn pick_of(&self, state: &AppState, scan: &ScanNode) -> Option<(PathBuf, PathBuf)> {
        let options = options(state, scan);
        let chosen = self.chosen.get(&scan.id);
        options
            .iter()
            .find(|((_, map), _)| Some(map) == chosen)
            .or(options.last())
            .map(|(o, _)| o.clone())
    }

    /// The ticked scans that can be merged, in the order of the list, with their maps; the
    /// first is the reference.
    pub fn picks(&self, state: &AppState) -> Vec<MergePick> {
        state
            .scans
            .iter()
            .filter(|s| self.ticked.contains(&s.id))
            .filter_map(|s| {
                let (run, map) = self.pick_of(state, s)?;
                Some(MergePick {
                    scan_id: s.id.clone(),
                    run,
                    map,
                })
            })
            .collect()
    }
}

/// The field, in a card. Returns [`Action::RunMerge`] when started; „Schließen“ closes it.
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let mut action = None;
    let mut merge = std::mem::take(&mut state.merge);
    crate::ui::theme::card_frame(true).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.strong(text::MERGE_TITLE);
        ui.add(egui::Label::new(RichText::new(text::MERGE_HINT).color(DARK.text_dim)).wrap());
        ui.add_space(4.0);
        let reference = merge.picks(state).first().map(|p| p.scan_id.clone());
        for scan in &state.scans {
            scan_row(ui, state, &mut merge, scan, reference.as_deref());
        }
        ui.add_space(4.0);
        ui.checkbox(&mut merge.include_unsure, text::MERGE_UNSURE)
            .on_hover_text(text::MERGE_UNSURE_HINT);
        ui.add_space(4.0);
        let picks = merge.picks(state);
        let busy = state.job.as_ref().map(|j| text::busy(&j.label));
        let few = (picks.len() < 2).then(|| text::MERGE_NEEDS_TWO.to_string());
        if main_button(ui, text::MERGE_START, busy.or(few), text::MERGE_START_HINT) {
            action = Some(Action::RunMerge {
                picks,
                include_unsure: merge.include_unsure,
            });
        }
        ui.vertical_centered(|ui| {
            let close = RichText::new(text::MERGE_CLOSE).color(DARK.text_dim);
            if ui.add(egui::Button::new(close).frame(false)).clicked() {
                merge.open = false;
            }
        });
    });
    state.merge = merge;
    action
}

/// A scan in the field: its tick (locked without a finished map), „Bezug“ beside the
/// reference, and for a ticked one the choice of its map.
fn scan_row(
    ui: &mut Ui,
    state: &AppState,
    merge: &mut MergeState,
    scan: &ScanNode,
    reference: Option<&str>,
) {
    let options = options(state, scan);
    let mut on = merge.ticked.contains(&scan.id) && !options.is_empty();
    ui.horizontal(|ui| {
        let r = ui.add_enabled(
            !options.is_empty(),
            egui::Checkbox::new(&mut on, &scan.name),
        );
        let r = r.on_disabled_hover_text(text::MERGE_NO_MAP);
        if r.changed() {
            if on {
                merge.ticked.insert(scan.id.clone());
            } else {
                merge.ticked.remove(&scan.id);
            }
        }
        if reference == Some(scan.id.as_str()) {
            ui.label(RichText::new(text::MERGE_REFERENCE).color(DARK.accent));
        }
    });
    if !on {
        return;
    }
    let current = merge.pick_of(state, scan);
    let id = format!("zusammenfuehren-{}", scan.id);
    let picked = compare::choose(ui, &id, text::MERGE_MAP, current.as_ref(), &options);
    if let Some((_, map)) = picked {
        merge.chosen.insert(scan.id.clone(), map);
    }
}

/// „Zusammenführen …“ under the scan cards, once two scans can be merged and the field is
/// closed. True when clicked.
pub(super) fn open_button(ui: &mut Ui, state: &AppState) -> bool {
    if state.merge.open || mergeable_scans(state) < 2 {
        return false;
    }
    ui.add_space(4.0);
    ui.button(text::MERGE_OPEN)
        .on_hover_text(text::MERGE_OPEN_HINT)
        .clicked()
}
