//! Manual corrections in the window, without a GPU, on a synthetic run with `submaps.bin`
//! (12 s standing still: submap 0 from 0 to 9.9 s, submap 1 from 10 s): two places picked on
//! the trajectory become a loop in `corrections.toml`, written as `driftless correct` writes
//! it (`driftless_steps::correct`), refusals of its check appear unchanged, entries are
//! switched, and the run is recomputed.

use driftless_app::jobs::JobStatus;
use driftless_app::project::canonical;
use driftless_app::ui::correct::{parse_time, CorrectState, Place};
use driftless_app::ui::{steps, text, Action, AppState, DriftlessApp, Selection, UiMessage};
use driftless_io::progress::ProgressSnapshot;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_slam::Corrections;
use driftless_steps::correct;
use driftless_steps::history::{self, HistoryKind};
use driftless_steps::slam::CORRECTIONS_FILE;
use eframe::egui;
use egui::accesskit::{Role, Toggled};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

const ADDED: &str = "2026-10-21T12:00:00Z";

type H = Harness<'static, DriftlessApp>;

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            _ => None,
        })
        .collect()
}

/// Ticks until no step and no background work is left.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        app.tick();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "background work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A project with the scan `halle` and one SLAM run of it; returns the app and the run.
fn app_with_run(dir: &Path) -> (DriftlessApp, PathBuf) {
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 12.0);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    app.apply(Action::AddScan(scan));
    app.apply(Action::RunSlam("halle".into()));
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    let run = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/halle/slam-001");
    assert_eq!(app.state().selection, Selection::Run(run.clone()));
    (app, run)
}

/// The window of `app` (no GPU: the view shows only its note).
fn harness(app: DriftlessApp) -> H {
    let mut h = Harness::builder()
        .with_size([1400.0, 1800.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    // the corrections and comparisons lie under „Erweitert“
    driftless_app::ui::steps::daten::set_advanced(&h.ctx, true);
    frames_until(&mut h, |app| {
        let c = &app.state().correct;
        app.overlay().is_some() && c.corrections.is_some() && !app.wants_repaint_while_busy()
    });
    h
}

/// Chooses the run again in the tree, so that the program reads what the CLI wrote.
fn reselect(app: &mut DriftlessApp, run: &Path) {
    app.apply(Action::Select(Selection::Run(run.to_path_buf())));
}

/// Frames until `cond` holds.
fn frames_until(h: &mut H, cond: impl Fn(&DriftlessApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        h.step();
        if cond(h.state()) {
            h.step();
            return;
        }
        assert!(Instant::now() < deadline, "condition not reached");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Frames until the change of the corrections is written.
fn wait_written(h: &mut H) {
    frames_until(h, |app| !app.state().correct.busy);
}

/// Moves the pointer onto `label` first, so that a tooltip from the last click is gone.
fn click(h: &mut H, label: &str) {
    h.get_by_label(label).hover();
    h.step();
    h.get_by_label(label).click();
    h.step();
}

/// Types `value` into the time field `label`.
fn type_time(h: &mut H, label: &str, value: &str) {
    let field = h
        .get_all_by_label(label)
        .find(|n| n.accesskit_node().role() == Role::TextInput)
        .expect("time field");
    field.focus();
    h.step();
    let field = h
        .get_all_by_label(label)
        .find(|n| n.accesskit_node().role() == Role::TextInput)
        .unwrap();
    field.type_text(value);
    h.step();
}

#[test]
fn places_fill_a_then_b_and_parse_typed_times() {
    let mut c = CorrectState {
        picking: true,
        ..CorrectState::default()
    };
    c.place(1.04);
    assert_eq!(
        (c.a.t, c.a.text.as_str(), c.next),
        (Some(1.04), "1,0", Place::B)
    );
    assert!(c.picking && c.places().is_none());
    c.place(11.26);
    assert_eq!((c.b.t, c.b.text.as_str()), (Some(11.26), "11,3"));
    assert!(!c.picking, "the mode ends after B");
    assert_eq!(c.places(), Some((1.04, 11.26)));
    assert_eq!(parse_time(" 11,5 "), Some(11.5));
    assert_eq!(parse_time("3.25"), Some(3.25));
    assert_eq!(parse_time("elf"), None);
    assert_eq!(parse_time("inf"), None);
    // another run: places gone, the changed runs stay
    c.changed.insert(PathBuf::from("/lauf"));
    assert!(c.for_run(Some(Path::new("/anderer"))));
    assert_eq!(c.places(), None);
    assert!(c.changed.contains(Path::new("/lauf")));
    assert!(!c.for_run(Some(Path::new("/anderer"))));
}

#[test]
fn pick_two_places_adds_loop() {
    let scratch = tmp("ui-korrektur-klick");
    let (app, run) = app_with_run(&scratch);
    let mut h = harness(app);
    let overlay = h.state().overlay().expect("trajectory of the run").clone();
    assert_eq!(overlay.run, run);
    let (pa, pb) = (
        overlay.pose_at(1.0).unwrap(),
        overlay.pose_at(11.0).unwrap(),
    );
    let (ta, tb) = (overlay.time_of(pa), overlay.time_of(pb));

    click(&mut h, text::PICK_PLACES);
    assert!(h.state().state().correct.picking);
    h.get_by_label(text::NEXT_PLACE_A);
    // what a click on the trajectory in the view does
    h.state_mut().pose_picked(pa);
    h.step();
    h.get_by_label(text::NEXT_PLACE_B);
    h.state_mut().pose_picked(pb);
    h.step();
    let c = &h.state().state().correct;
    assert_eq!(c.places(), Some((ta, tb)));
    assert!(!c.picking);
    assert_eq!(c.a.text, text::place_time(ta));

    type_time(&mut h, text::NOTE, "Flur");
    assert_eq!(h.state().state().correct.note, "Flur");
    click(&mut h, text::ADD_LOOP);
    wait_written(&mut h);

    let c = correct::load(&run).unwrap();
    assert_eq!(c.loops.len(), 1);
    let entry = &c.loops[0];
    assert_eq!((entry.a, entry.b, entry.enabled), (ta, tb, true));
    assert_eq!(entry.note, "Flur");
    // as `driftless correct <lauf> list` shows it
    let list = correct::render_list(&c, &run.join(CORRECTIONS_FILE));
    let row = list.lines().nth(1).unwrap();
    assert!(row.starts_with("1   aktiv"), "{row}");
    assert!(row.contains(&format!("{ta:.1}")) && row.contains(&format!("{tb:.1}")));
    // in the panel: the message, the entry, the hint to recompute; places cleared
    // the arrow of the message drawn by the icon font
    h.get_by_label(&text::shown(&correct::added_message(1, ta, tb)));
    h.get_by_label(&text::place_time(ta));
    h.get_by_label(&entry.added);
    h.get_by_label(text::CHANGED_RECOMPUTE);
    assert_eq!(h.state().state().correct.places(), None);
}

#[test]
fn invalid_loop_shows_error_and_writes_nothing() {
    let scratch = tmp("ui-korrektur-falsch");
    let (app, run) = app_with_run(&scratch);
    // one loop already there, written as the CLI writes it
    correct::add_loop_at(&run, 1.0, 11.0, "", ADDED).unwrap();
    let before = fs::read(run.join(CORRECTIONS_FILE)).unwrap();
    let mut app = app;
    reselect(&mut app, &run);
    let mut h = harness(app);

    // typed by hand: both in submap 0, after the end, the same loop again, and the same
    // pair of submaps at other times
    for (a, b) in [
        ("1,0", "2,0"),
        ("1,0", "500"),
        ("1,0", "11,0"),
        ("2,5", "11,5"),
    ] {
        let c = &mut h.state_mut().state_mut().correct;
        c.message = None;
        c.a.text.clear();
        c.b.text.clear();
        type_time(&mut h, text::PLACE_A, a);
        type_time(&mut h, text::PLACE_B, b);
        let (ta, tb) = (parse_time(a).unwrap(), parse_time(b).unwrap());
        assert_eq!(h.state().state().correct.places(), Some((ta, tb)));
        click(&mut h, text::ADD_LOOP);
        wait_written(&mut h);

        // the message of the check, as shown (its arrow drawn by the icon font)
        let expected = correct::check_new_loop(&run, &correct::load(&run).unwrap(), ta, tb)
            .unwrap_err()
            .to_string();
        let expected = text::shown(&text::german(&expected)).into_owned();
        assert_eq!(
            h.state().state().correct.message,
            Some(UiMessage::Error(expected.clone())),
            "{a} ↔ {b}"
        );
        h.get_by_label(&expected);
        assert_eq!(fs::read(run.join(CORRECTIONS_FILE)).unwrap(), before);
        // nothing to recompute, the places stay for correcting them
        assert!(h.query_by_label(text::CHANGED_RECOMPUTE).is_none());
        assert_eq!(h.state().state().correct.places(), Some((ta, tb)));
    }
}

#[test]
fn toggle_entry_and_reuse() {
    let scratch = tmp("ui-korrektur-schalter");
    let (app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "Treppe", ADDED).unwrap();
    let mut app = app;
    reselect(&mut app, &run);
    let mut h = harness(app);
    assert!(h.query_by_label(text::CHANGED_RECOMPUTE).is_none());

    let states_before = history::list(&run).len();
    // switch entry 1 off
    click(&mut h, &text::loop_switch(1));
    wait_written(&mut h);
    assert!(!correct::load(&run).unwrap().loops[0].enabled);
    h.get_by_label(&correct::switched_message(1, false, true));
    h.get_by_label(correct::status_text(false));
    h.get_by_label(text::CHANGED_RECOMPUTE);

    // recompute: the reuse of this run starts
    click(&mut h, text::RECOMPUTE_NOW);
    assert_eq!(h.state().state().running, Some(Action::Reuse(run.clone())));
    frames_until(&mut h, |app| {
        app.state().job.is_none() && !app.wants_repaint_while_busy()
    });
    assert_eq!(errors(h.state()), Vec::<String>::new());
    assert!(run.join("traj.vorher.tum").is_file(), "recomputed");
    // recorded as a state of kind reuse, with the entry switched off
    let states = history::list(&run);
    let last = states.last().unwrap();
    assert_eq!(states.len(), states_before + 1);
    assert_eq!(last.kind, HistoryKind::Reuse);
    let then = Corrections::load(&last.dir.join(CORRECTIONS_FILE)).unwrap();
    assert!(!then.loops[0].enabled);
    // computed with the corrections as they are: no hint any more
    assert!(h.query_by_label(text::CHANGED_RECOMPUTE).is_none());

    // and on again: the check passes, the entry is active
    click(&mut h, &text::loop_switch(1));
    wait_written(&mut h);
    assert!(correct::load(&run).unwrap().loops[0].enabled);
    h.get_by_label(text::CHANGED_RECOMPUTE);
}

#[test]
fn cli_written_corrections_are_listed() {
    let scratch = tmp("ui-korrektur-cli");
    let (app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "Treppe", ADDED).unwrap();
    correct::set_enabled(&run, 1, false).unwrap();
    correct::add_loop_at(&run, 2.25, 10.75, "Flur Süd", "2026-10-21T12:30:00Z").unwrap();
    let mut app = app;
    reselect(&mut app, &run);
    let h = harness(app);

    let c = correct::load(&run).unwrap();
    let list = correct::render_list(&c, &run.join(CORRECTIONS_FILE));
    let rows: Vec<&str> = list.lines().skip(1).collect();
    assert_eq!(rows.len(), 2);
    for head in text::CORRECTION_COLUMNS {
        assert!(h.get_all_by_label(head).count() >= 1, "{head}");
    }
    for (row, entry) in rows.iter().zip(&c.loops) {
        // Nr Status Art a b hinzugefügt Notiz…
        let cells: Vec<&str> = row.split_whitespace().collect();
        let (nr, status, a, b, added) = (cells[0], cells[1], cells[3], cells[4], cells[5]);
        let note = cells[6..].join(" ");
        h.get_by_label(nr);
        assert!(h.get_all_by_label(status).count() >= 1, "{status}");
        // the same numbers, with a decimal comma
        for (cli, panel) in [(a, entry.a), (b, entry.b)] {
            assert_eq!(cli.replace('.', ","), text::place_time(panel));
            h.get_by_label(&text::place_time(panel));
        }
        h.get_by_label(added);
        h.get_by_label(&note);
    }
    // the switches show the state of the file
    let toggled = |nr| {
        h.get_by_label(&text::loop_switch(nr))
            .accesskit_node()
            .toggled()
    };
    assert_eq!(toggled(1), Some(Toggled::False));
    assert_eq!(toggled(2), Some(Toggled::True));
}

/// While a step of the run is going, its corrections cannot be changed: the step read them at
/// its start and its state is recorded with those.
#[test]
fn corrections_locked_while_a_step_of_the_run_runs() {
    let scratch = tmp("ui-korrektur-gesperrt");
    let (app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "", ADDED).unwrap();
    let mut app = app;
    reselect(&mut app, &run);
    let mut state = harness(app).state().state().clone();
    state.correct.place(2.0);
    state.correct.place(11.5);
    // as the program shows a running reuse of this run
    state.running = Some(Action::Reuse(run.clone()));
    state.job = Some(JobStatus {
        label: text::STEP_REUSE.into(),
        progress: ProgressSnapshot::default(),
        seconds: 1.0,
        cancelling: false,
    });
    let mut h = Harness::builder()
        .with_size([400.0, 1600.0])
        .build_ui_state(
            |ui, state: &mut AppState| {
                steps::show(ui, state);
            },
            state.clone(),
        );
    driftless_app::ui::steps::daten::set_advanced(&h.ctx, true);
    h.run();
    let disabled = |h: &Harness<'_, AppState>, label: &str| {
        h.get_by_label(label).accesskit_node().is_disabled()
    };
    assert!(disabled(&h, text::ADD_LOOP));
    assert!(disabled(&h, &text::loop_switch(1)));
    h.get_by_label(text::ADD_LOOP).hover();
    h.run();
    h.get_by_label(&text::busy(text::STEP_REUSE));

    // a step of another run locks nothing here
    h.state_mut().running = Some(Action::Reuse(PathBuf::from("/anderer/lauf")));
    h.run();
    assert!(!disabled(&h, text::ADD_LOOP));
    assert!(!disabled(&h, &text::loop_switch(1)));

    // choosing places with the trajectory hidden: the panel says what to do
    h.state_mut().correct.picking = true;
    h.state_mut().view.show_trajectory = false;
    h.run();
    h.get_by_label(text::PICK_NEEDS_TRAJECTORY);
}

/// A change still being written blocks recomputing (the reuse would read the file before
/// it); a change written after a reuse started keeps the hint, it was not applied.
#[test]
fn recompute_waits_for_writes_and_later_writes_keep_the_hint() {
    let scratch = tmp("ui-korrektur-wettlauf");
    let (app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "", ADDED).unwrap();
    let mut app = app;
    reselect(&mut app, &run);
    let mut h = harness(app);

    // a write queued: both recompute buttons locked, a reuse refused
    h.state_mut().apply(Action::SetLoopEnabled {
        run: run.clone(),
        nr: 1,
        enabled: false,
    });
    assert!(h.state().state().correct.busy);
    h.state_mut().apply(Action::Reuse(run.clone()));
    assert_eq!(h.state().state().running, None, "reuse refused");
    assert_eq!(
        errors(h.state()).last().map(String::as_str),
        Some(text::CORRECTION_BUSY)
    );
    h.step();
    if h.state().state().correct.busy {
        let disabled = h
            .get_by_label(text::STEP_REUSE)
            .accesskit_node()
            .is_disabled();
        assert!(disabled, "„Neu rechnen“ locked while writing");
    }
    wait_written(&mut h);
    h.get_by_label(text::CHANGED_RECOMPUTE);

    // a reuse starts, then a change lands (as from a write that was under way)
    h.state_mut().apply(Action::Reuse(run.clone()));
    assert_eq!(h.state().state().running, Some(Action::Reuse(run.clone())));
    h.state_mut().apply(Action::SetLoopEnabled {
        run: run.clone(),
        nr: 1,
        enabled: true,
    });
    frames_until(&mut h, |app| {
        app.state().job.is_none() && !app.wants_repaint_while_busy()
    });
    assert!(correct::load(&run).unwrap().loops[0].enabled);
    // not applied by that reuse: the hint stays
    assert!(h.state().state().correct.changed.contains(&run));
    h.get_by_label(text::CHANGED_RECOMPUTE);

    // a reuse without a later write clears it
    h.state_mut().apply(Action::Reuse(run.clone()));
    frames_until(&mut h, |app| {
        app.state().job.is_none() && !app.wants_repaint_while_busy()
    });
    assert!(h.query_by_label(text::CHANGED_RECOMPUTE).is_none());
}

/// The count taken at the start of a reuse decides whether it ran with the corrections as
/// they are now (a write landing while it ran, before it ended, keeps the hint).
#[test]
fn reuse_with_a_write_since_its_start_is_not_current() {
    let run = PathBuf::from("/lauf");
    let mut c = CorrectState::default();
    c.reuse_writes = Some((run.clone(), c.writes_of(&run)));
    *c.writes.entry(run.clone()).or_default() += 1;
    assert!(!c.computed_current(&run));
    assert!(c.reuse_writes.is_none(), "taken");
    c.reuse_writes = Some((run.clone(), c.writes_of(&run)));
    assert!(c.computed_current(&run));
    // no reuse recorded (a full run): current
    assert!(c.computed_current(&run));
}

/// The legend of the quality colours: shown while the trajectory is drawn in them (not while
/// a comparison draws it in one colour), or the anchors of the submaps, or the points in the
/// colour mode „Güte“; its three rows say in German what green, yellow and red mean.
#[test]
fn quality_legend_follows_what_is_drawn() {
    let scratch = tmp("ui-guete-legende");
    let (app, _run) = app_with_run(&scratch);
    let mut h = harness(app);
    // whether the window placed the legend in the last frame
    let placed = |h: &mut H| {
        h.step();
        h.state().layout().legend.is_some()
    };
    // by default the comparison „Odometrie / aktuell“ draws the run in one colour
    assert!(h.state().overlay().is_some_and(|o| !o.compare.is_empty()));
    assert!(!placed(&mut h));
    h.state_mut().state_mut().view.show_compare = false;
    assert!(placed(&mut h));
    // the rows, once the legend has faded in
    frames_until(&mut h, |_| true);
    let deadline = Instant::now() + Duration::from_secs(10);
    while h.query_by_label(text::QUALITY_BAD).is_none() {
        assert!(Instant::now() < deadline, "legend rows not shown");
        h.step();
    }
    for row in [text::QUALITY_LEGEND, text::QUALITY_GOOD, text::QUALITY_WEAK] {
        h.get_by_label(row);
    }
    // its place: in the view, at the left, above the bottom
    let rect = h.state().layout().legend.expect("legend placed");
    let view = h.state().layout().view;
    assert!(view.contains_rect(rect), "{rect:?} in {view:?}");
    assert!(rect.left() < view.center().x && rect.bottom() > view.center().y);
    // without the trajectory: gone; the anchors of the submaps bring it back
    h.state_mut().state_mut().view.show_trajectory = false;
    assert!(!placed(&mut h));
    h.state_mut().state_mut().view.show_submaps = true;
    assert!(placed(&mut h));
    h.state_mut().state_mut().view.show_submaps = false;
    // the points in the colour mode „Güte“ (the synthetic cloud carries qualities)
    h.state_mut().load_synthetic(2_000);
    // the run's own map.las is shown already: wait for the synthetic cloud itself
    let synthetic = std::path::PathBuf::from(text::synthetic_name(2_000));
    frames_until(&mut h, |app| {
        app.cloud().is_some_and(|c| c.path == synthetic)
    });
    h.state_mut().state_mut().view.color_mode = driftless_app::render::ColorMode::Rgb;
    assert!(!placed(&mut h));
    h.state_mut().state_mut().view.color_mode = driftless_app::render::ColorMode::Quality;
    assert!(placed(&mut h));
}
