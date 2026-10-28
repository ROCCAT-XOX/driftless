//! The manual corrections of a run on the right („Korrekturen“): two places of the
//! trajectory – clicked in the 3D view in the mode „Stellen wählen“ or typed as seconds since
//! the start of the scan – become a loop in `corrections.toml`, the same file as
//! `driftless correct` writes. Below, every entry as `driftless correct <lauf> list` shows it,
//! each with its switch, and after a change the hint to recompute the run.
//!
//! Below them the automatic loops of the run ([`AutoLoop`], from its `slam.json`), each with a
//! switch „verwenden“: off writes a `[[reject]]` entry and the program recomputes the run from
//! its submaps at once; on removes it again.
//!
//! The panel only asks ([`Action::AddLoop`], [`Action::SetLoopEnabled`],
//! [`Action::SetLoopUsed`], [`Action::Reuse`]); the program checks and writes in the
//! background (the check reads `submaps.bin`) and puts the answer of the check, unchanged,
//! into [`CorrectState::message`].

use super::{text, Action, AppState, UiMessage};
use crate::overlay::RunOverlay;
use driftless_slam::{Corrections, SlamReport, VETOED};
use eframe::egui::{self, RichText, Ui};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// An automatic loop of the run, as „Korrekturen“ lists it: in the pose graph, or left out
/// because it was switched off by hand.
#[derive(Debug, Clone, PartialEq)]
pub struct AutoLoop {
    /// Its two submaps (`a < b`) and their starts (seconds since the start of the scan).
    pub a: usize,
    pub b: usize,
    pub a_t: f64,
    pub b_t: f64,
    /// In the pose graph of the last computation (else switched off by hand then).
    pub used: bool,
    /// The share of points with a match (`inlier_ratio`).
    pub overlap: f64,
}

/// The automatic loops of `report` that can be switched: the accepted ones and the ones
/// switched off by hand, by their submaps.
pub fn auto_loops(report: &SlamReport) -> Vec<AutoLoop> {
    let start = |s: usize| report.submaps.get(s).map(|m| m.t_start);
    let mut out: Vec<AutoLoop> = report
        .loops
        .iter()
        .filter(|l| !l.manual && (l.accepted || l.reason.as_deref() == Some(VETOED)))
        .filter_map(|l| {
            let (a, b) = (l.a.min(l.b), l.a.max(l.b));
            Some(AutoLoop {
                a,
                b,
                a_t: start(a)?,
                b_t: start(b)?,
                used: l.accepted,
                overlap: l.inlier_ratio,
            })
        })
        .collect();
    out.sort_by_key(|l| (l.a, l.b));
    out.dedup_by_key(|l| (l.a, l.b));
    out
}

/// The pair of submaps (`a < b`) the times `a` and `b` (seconds since the start of the scan)
/// fall into, by the starts of the submaps; `None` for both in one.
pub fn submap_pair(starts: &[f64], a: f64, b: f64) -> Option<(usize, usize)> {
    let of = |t: f64| starts.partition_point(|&s| s <= t + 1e-6).checked_sub(1);
    let (x, y) = (of(a)?, of(b)?);
    (x != y).then(|| (x.min(y), x.max(y)))
}

/// One of the two places of a loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Place {
    #[default]
    A,
    B,
}

/// The time of a place: as typed, and the value read from it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlaceField {
    /// Seconds since the start of the scan; `None` while empty or not a number.
    pub t: Option<f64>,
    pub text: String,
}

impl PlaceField {
    fn set(&mut self, t: f64) {
        self.t = Some(t);
        self.text = text::place_time(t);
    }
}

/// What the corrections panel shows of the run the view draws.
#[derive(Debug, Clone, Default)]
pub struct CorrectState {
    /// The run the places and the list belong to.
    pub run: Option<PathBuf>,
    pub a: PlaceField,
    pub b: PlaceField,
    pub note: String,
    /// Clicks on the trajectory set the places („Stellen wählen“).
    pub picking: bool,
    /// The place the next click sets.
    pub next: Place,
    /// `corrections.toml` as last read; `None` while it is being read.
    pub corrections: Option<Result<Corrections, String>>,
    /// The answer to the last change (the check's message on a refusal).
    pub message: Option<UiMessage>,
    /// A change is being written.
    pub busy: bool,
    /// Runs whose corrections changed since they were last computed (in this session).
    pub changed: BTreeSet<PathBuf>,
    /// How many changes were written per run (in this session).
    pub writes: BTreeMap<PathBuf, u64>,
    /// The run of the reuse that runs, with its count of [`CorrectState::writes`] at the start:
    /// a change written later was not applied.
    pub reuse_writes: Option<(PathBuf, u64)>,
    /// The automatic loops of the run ([`auto_loops`]), as its `slam.json` was last read.
    pub auto_loops: Vec<AutoLoop>,
    /// The starts of the submaps of the run (s), to tell which loop a reject names.
    pub submap_starts: Vec<f64>,
}

impl CorrectState {
    /// Shows the run `run` from now on; the places, note and list of another run are
    /// dropped. True if the run changed.
    pub fn for_run(&mut self, run: Option<&Path>) -> bool {
        if self.run.as_deref() == run {
            return false;
        }
        *self = CorrectState {
            run: run.map(Path::to_path_buf),
            changed: std::mem::take(&mut self.changed),
            writes: std::mem::take(&mut self.writes),
            reuse_writes: self.reuse_writes.take(),
            ..CorrectState::default()
        };
        true
    }

    /// A pose at `t` (seconds since the start of the scan) was clicked: sets the next place,
    /// A then B; after B the mode ends.
    pub fn place(&mut self, t: f64) {
        match self.next {
            Place::A => {
                self.a.set(t);
                self.next = Place::B;
            }
            Place::B => {
                self.b.set(t);
                self.next = Place::A;
                self.picking = false;
            }
        }
    }

    /// The times of both places, once both are set.
    pub fn places(&self) -> Option<(f64, f64)> {
        Some((self.a.t?, self.b.t?))
    }

    /// The changes written for `run` so far.
    pub fn writes_of(&self, run: &Path) -> u64 {
        self.writes.get(run).copied().unwrap_or(0)
    }

    /// A step on `run` ended: whether it was computed with the corrections as they are now
    /// (no change written since a reuse of it started).
    pub fn computed_current(&mut self, run: &Path) -> bool {
        match self.reuse_writes.take() {
            Some((r, n)) if r == run => n == self.writes_of(run),
            _ => true,
        }
    }

    /// After a loop was written: the places and the note start empty again.
    pub fn clear_places(&mut self) {
        self.a = PlaceField::default();
        self.b = PlaceField::default();
        self.note.clear();
        self.next = Place::A;
    }
}

/// What the helper job that reads a run hands back.
#[derive(Debug)]
pub struct RunLoaded {
    pub run: PathBuf,
    /// The overlay, relative to the first pose (the program moves it to the cloud's origin).
    pub overlay: Result<RunOverlay, String>,
    pub corrections: Result<Corrections, String>,
}

/// What the helper job that changed `corrections.toml` hands back.
#[derive(Debug)]
pub struct Corrected {
    pub run: PathBuf,
    /// For the panel.
    pub message: String,
    /// Whether the file changed.
    pub changed: bool,
    /// A loop was added (the places are then cleared).
    pub added: bool,
    /// The file as now on disk.
    pub corrections: Corrections,
}

/// A typed time: a decimal number with comma or point.
pub fn parse_time(s: &str) -> Option<f64> {
    s.trim()
        .replace(',', ".")
        .parse::<f64>()
        .ok()
        .filter(|t| t.is_finite())
}

/// The status column of an entry, as `driftless correct list` writes it.
pub fn status(enabled: bool) -> &'static str {
    driftless_steps::correct::status_text(enabled)
}

/// Why the corrections of `run` cannot be changed now: a step of this run is running (it
/// read them at its start, and its state is recorded with those).
pub fn locked(state: &AppState, run: &Path) -> Option<String> {
    let of_run = matches!(
        &state.running,
        Some(Action::Reuse(d) | Action::RunMap(d) | Action::RestoreHistory { run: d, .. })
            if d == run
    );
    state
        .job
        .as_ref()
        .filter(|_| of_run)
        .map(|j| text::busy(&j.label))
}

/// Why the run cannot be recomputed now: a change of the corrections is still being written
/// (the recompute would read the file before it).
pub fn writing(state: &AppState) -> Option<String> {
    state
        .correct
        .busy
        .then(|| text::CORRECTION_BUSY.to_string())
}

/// The panel of the selected run (without its heading); nothing without one.
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let (dir, has_cache) = state
        .selected_run()
        .map(|(_, r)| (r.dir.clone(), r.has_cache))?;
    let mut action = None;
    ui.weak(text::CORRECTIONS_HINT);
    let job = state.job.as_ref().map(|j| text::busy(&j.label));
    let locked = locked(state, &dir);
    let writing = writing(state);
    let trajectory_hidden = !state.view.show_trajectory;
    let c = &mut state.correct;
    if c.run.as_deref() != Some(dir.as_path()) {
        ui.weak(text::CORRECTIONS_READING);
        return None;
    }

    let toggle = ui
        .selectable_label(c.picking, text::PICK_PLACES)
        .on_hover_text(text::PICK_PLACES_HINT);
    if toggle.clicked() {
        c.picking = !c.picking;
        c.next = Place::A;
    }
    if c.picking && trajectory_hidden {
        let warn = ui.visuals().warn_fg_color;
        ui.label(RichText::new(text::PICK_NEEDS_TRAJECTORY).color(warn));
    } else if c.picking {
        ui.weak(match c.next {
            Place::A => text::NEXT_PLACE_A,
            Place::B => text::NEXT_PLACE_B,
        });
    }
    egui::Grid::new("stellen").num_columns(2).show(ui, |ui| {
        for (label, field) in [(text::PLACE_A, &mut c.a), (text::PLACE_B, &mut c.b)] {
            let shown = ui.label(label);
            let edit = egui::TextEdit::singleline(&mut field.text)
                .hint_text(text::PLACE_HINT)
                .desired_width(80.0);
            if ui.add(edit).labelled_by(shown.id).changed() {
                field.t = parse_time(&field.text);
            }
            ui.end_row();
        }
        let shown = ui.label(text::NOTE);
        ui.add(egui::TextEdit::singleline(&mut c.note).desired_width(160.0))
            .labelled_by(shown.id);
        ui.end_row();
    });
    let loaded = matches!(c.corrections, Some(Ok(_)));
    let places = c.places();
    let add = ui.add_enabled(
        places.is_some() && loaded && !c.busy && locked.is_none(),
        egui::Button::new(text::ADD_LOOP),
    );
    if let Some(why) = &locked {
        add.clone().on_disabled_hover_text(why);
    } else if places.is_none() {
        add.clone()
            .on_disabled_hover_text(text::ADD_LOOP_NEEDS_PLACES);
    }
    if add.clicked() {
        if let Some((a, b)) = places {
            action = Some(Action::AddLoop {
                run: dir.clone(),
                a,
                b,
                note: c.note.trim().to_string(),
            });
        }
    }
    if c.busy {
        ui.weak(text::CORRECTION_BUSY);
    }
    match &c.message {
        Some(UiMessage::Error(e)) => {
            let color = ui.visuals().error_fg_color;
            ui.label(RichText::new(e).color(color));
        }
        Some(UiMessage::Warning(m)) => {
            let color = ui.visuals().warn_fg_color;
            ui.label(RichText::new(m).color(color));
        }
        Some(UiMessage::Info(m)) => {
            ui.label(m);
        }
        None => {}
    }
    if c.changed.contains(&dir) {
        let warn = ui.visuals().warn_fg_color;
        ui.label(RichText::new(text::CHANGED_RECOMPUTE).color(warn));
        let blocked = job
            .or(writing)
            .or((!has_cache).then(|| text::NO_CACHE.to_string()));
        let button = ui.add_enabled(blocked.is_none(), egui::Button::new(text::RECOMPUTE_NOW));
        if button.clicked() {
            action = Some(Action::Reuse(dir.clone()));
        }
        if let Some(why) = blocked {
            button.on_disabled_hover_text(why);
        }
    }
    let can_switch = !c.busy && locked.is_none();
    match &c.corrections {
        None => {
            ui.weak(text::CORRECTIONS_READING);
        }
        Some(Err(e)) => {
            let color = ui.visuals().error_fg_color;
            ui.label(RichText::new(e).color(color));
        }
        Some(Ok(list)) if list.loops.is_empty() => {
            ui.weak(text::NO_CORRECTIONS);
        }
        Some(Ok(list)) => {
            egui::Grid::new("korrekturen")
                .num_columns(6)
                .striped(true)
                .show(ui, |ui| {
                    for head in text::CORRECTION_COLUMNS {
                        ui.strong(head);
                    }
                    ui.end_row();
                    for (i, l) in list.loops.iter().enumerate() {
                        let nr = i + 1;
                        let mut on = l.enabled;
                        ui.label(nr.to_string());
                        ui.horizontal(|ui| {
                            let switch =
                                ui.add_enabled(can_switch, egui::Checkbox::without_text(&mut on));
                            switch.widget_info(|| {
                                egui::WidgetInfo::selected(
                                    egui::WidgetType::Checkbox,
                                    can_switch,
                                    on,
                                    text::loop_switch(nr),
                                )
                            });
                            if switch.changed() {
                                action = Some(Action::SetLoopEnabled {
                                    run: dir.clone(),
                                    nr,
                                    enabled: on,
                                });
                            }
                            let switch = switch.on_hover_text(text::LOOP_SWITCH_HINT);
                            if let Some(why) = &locked {
                                switch.on_disabled_hover_text(why);
                            }
                            ui.label(status(l.enabled));
                        });
                        ui.label(text::place_time(l.a));
                        ui.label(text::place_time(l.b));
                        ui.label(&l.added);
                        ui.label(&l.note);
                        ui.end_row();
                    }
                });
        }
    }
    let no_cache = (!has_cache).then(|| text::NO_CACHE.to_string());
    let blocked = locked.or(no_cache);
    if let Some(a) = auto_loop_list(ui, c, &dir, can_switch, blocked) {
        action = Some(a);
    }
    action
}

/// „Automatische Schleifen“: each automatic loop of the run with its switch „verwenden“ (off
/// where `corrections.toml` rejects it), its submaps, their starts and its overlap; a reject
/// whose loop the last computation did not list follows with its times. Switching asks for
/// [`Action::SetLoopUsed`]; locked while a change is written, a step of the run runs, or the
/// run cannot be recomputed (`blocked`, the reason on hover).
fn auto_loop_list(
    ui: &mut Ui,
    c: &CorrectState,
    run: &Path,
    can_switch: bool,
    blocked: Option<String>,
) -> Option<Action> {
    let mut action = None;
    ui.add_space(6.0);
    ui.strong(text::AUTO_LOOPS);
    ui.add(egui::Label::new(RichText::new(text::AUTO_LOOPS_HINT).weak()).wrap());
    let rejects = match &c.corrections {
        Some(Ok(list)) => list.rejects.clone(),
        _ => Vec::new(),
    };
    let rejected: BTreeSet<(usize, usize)> = rejects
        .iter()
        .filter_map(|r| submap_pair(&c.submap_starts, r.a, r.b))
        .collect();
    /// A row: the submaps (if known), their starts, the overlap (if listed) and the switch.
    struct Row {
        pair: Option<(usize, usize)>,
        a_t: f64,
        b_t: f64,
        overlap: Option<f64>,
        on: bool,
    }
    let mut rows: Vec<Row> = c
        .auto_loops
        .iter()
        .map(|l| Row {
            pair: Some((l.a, l.b)),
            a_t: l.a_t,
            b_t: l.b_t,
            overlap: Some(l.overlap),
            on: !rejected.contains(&(l.a, l.b)),
        })
        .collect();
    for r in &rejects {
        let pair = submap_pair(&c.submap_starts, r.a, r.b);
        if !c.auto_loops.iter().any(|l| Some((l.a, l.b)) == pair) {
            rows.push(Row {
                pair,
                a_t: r.a,
                b_t: r.b,
                overlap: None,
                on: false,
            });
        }
    }
    if rows.is_empty() {
        ui.weak(text::NO_AUTO_LOOPS);
        return None;
    }
    let enabled = can_switch && blocked.is_none();
    egui::Grid::new("automatische_schleifen")
        .num_columns(4)
        .striped(true)
        .show(ui, |ui| {
            for head in text::AUTO_LOOP_COLUMNS {
                ui.strong(head);
            }
            ui.end_row();
            for Row {
                pair,
                a_t,
                b_t,
                overlap,
                on,
            } in rows
            {
                let mut now = on;
                let switch = ui.add_enabled(enabled, egui::Checkbox::without_text(&mut now));
                let name = pair.map_or_else(
                    || text::auto_loop_times(a_t, b_t),
                    |(a, b)| text::auto_loop_switch(a, b),
                );
                switch.widget_info(|| {
                    egui::WidgetInfo::selected(egui::WidgetType::Checkbox, enabled, now, &name)
                });
                if switch.changed() {
                    action = Some(Action::SetLoopUsed {
                        run: run.to_path_buf(),
                        a: a_t,
                        b: b_t,
                        used: now,
                    });
                }
                let switch = switch.on_hover_text(text::AUTO_LOOP_SWITCH_HINT);
                if let Some(why) = &blocked {
                    switch.on_disabled_hover_text(why);
                }
                ui.label(pair.map_or_else(String::new, |(a, b)| text::auto_loop_pair(a, b)));
                ui.label(text::auto_loop_times(a_t, b_t));
                ui.label(overlap.map_or_else(String::new, text::percent));
                ui.end_row();
            }
        });
    action
}
