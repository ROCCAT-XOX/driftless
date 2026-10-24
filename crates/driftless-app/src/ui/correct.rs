//! The manual corrections of a run on the right („Korrekturen“): two places of the
//! trajectory – clicked in the 3D view in the mode „Stellen wählen“ or typed as seconds since
//! the start of the scan – become a loop in `corrections.toml`, the same file as
//! `driftless correct` writes. Below, every entry as `driftless correct <lauf> list` shows it,
//! each with its switch, and after a change the hint to recompute the run.
//!
//! The panel only asks ([`Action::AddLoop`], [`Action::SetLoopEnabled`], [`Action::Reuse`]);
//! the program checks and writes in the background (the check reads `submaps.bin`) and puts
//! the answer of the check, unchanged, into [`CorrectState::message`].

use super::{text, Action, AppState, UiMessage};
use crate::overlay::RunOverlay;
use driftless_slam::Corrections;
use eframe::egui::{self, RichText, Ui};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

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
    action
}
