//! The veto of automatic loops in „Korrekturen“ (decision 8 of track U), in the window without
//! a GPU: a run of two laps round the synthetic room closes automatic loops; each is listed
//! with a switch „verwenden“. Switching one off writes a `[[reject]]` into `corrections.toml`
//! and recomputes the run from its submaps at once (the pose graph only); the loop is then
//! left out („von Hand abgeschaltet“). Switching it on again removes the entry. A
//! `corrections.toml` written before the veto existed still reads.

use driftless_app::project::canonical;
use driftless_app::ui::{steps, text, Action, DriftlessApp, Selection, UiMessage};
use driftless_io::testscan::tmp;
use driftless_odom::testutil::{room_scene, walk_bag, Circle};
use driftless_slam::{Corrections, SlamReport, VETOED};
use driftless_steps::slam::CORRECTIONS_FILE;
use eframe::egui;
use egui::accesskit::Toggled;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

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

/// Frames until `cond` holds and no background work is left.
fn frames_until(h: &mut H, what: &str, cond: impl Fn(&DriftlessApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        h.step();
        let app = h.state();
        if cond(app) && app.state().job.is_none() && !app.wants_repaint_while_busy() {
            h.step();
            return;
        }
        assert!(Instant::now() < deadline, "{what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A project with a scan of two laps and its SLAM run (computed in the window), the run
/// selected and „Erweitert“ open; with `old` a `corrections.toml` as written before the veto
/// existed (one manual loop, no rejects) is put into the run first.
fn window_with_loops(name: &str, old: bool) -> (H, PathBuf) {
    let dir = tmp(name);
    let scan = dir.join("runde");
    let walk = Circle {
        still: 1.0,
        radius: 2.0,
        speed: 1.0,
        laps: 2.0,
    };
    walk_bag(&scan, &room_scene(), &walk, walk.duration(), 360, 1000.0);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Runde".into(),
    });
    app.apply(Action::AddScan(scan));
    let mut h = Harness::builder()
        .with_size([1400.0, 2400.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    frames_until(&mut h, "scan not added", |app| {
        !app.state().scans.is_empty()
    });
    h.state_mut().apply(Action::RunSlam("runde".into()));
    frames_until(&mut h, "SLAM did not end", |app| {
        app.state().running.is_none()
    });
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let run = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/runde/slam-001");
    if old {
        fs::write(
            run.join(CORRECTIONS_FILE),
            "# Driftless-Korrekturen. Zeiten in Sekunden ab Scan-Beginn.\n\n[[loop]]\na = 1.5\n\
             b = 27.5\nenabled = false\nnote = \"alt\"\nadded = \"2026-10-20T10:00:00Z\"\n",
        )
        .unwrap();
    }
    steps::daten::set_advanced(&h.ctx, true);
    h.state_mut()
        .apply(Action::Select(Selection::Run(run.clone())));
    frames_until(&mut h, "run not loaded", |app| {
        app.overlay().is_some() && app.state().correct.corrections.is_some()
    });
    (h, run)
}

fn report(run: &Path) -> SlamReport {
    serde_json::from_str(&fs::read_to_string(run.join("slam.json")).unwrap()).unwrap()
}

/// The accepted automatic loops of the run, by submaps.
fn accepted(run: &Path) -> Vec<(usize, usize)> {
    report(run)
        .loops
        .iter()
        .filter(|l| !l.manual && l.accepted)
        .map(|l| (l.a, l.b))
        .collect()
}

fn switch_on(h: &H, a: usize, b: usize) -> bool {
    let name = text::auto_loop_switch(a, b);
    let node = h.get_by_label(&name);
    node.accesskit_node().toggled() == Some(Toggled::True)
}

#[test]
fn switching_an_automatic_loop_off_and_on() {
    let (mut h, run) = window_with_loops("veto-schalter", false);
    let loops = accepted(&run);
    assert!(!loops.is_empty(), "the walk closes automatic loops");
    // every automatic loop is listed with its switch, on
    h.get_by_label(text::AUTO_LOOPS);
    for &(a, b) in &loops {
        assert!(switch_on(&h, a, b), "{a}–{b}");
    }
    let (a, b) = loops[0];
    let starts = |r: &SlamReport| (r.submaps[a].t_start, r.submaps[b].t_start);
    let times = starts(&report(&run));

    // off: a reject with the start times of the two submaps, and the run recomputed at once
    h.get_by_label(&text::auto_loop_switch(a, b)).click();
    frames_until(&mut h, "reject not written", |app| {
        matches!(app.state().running, Some(Action::Reuse(_)))
            || !app.state().correct.busy && app.state().correct.reuse_writes.is_none()
    });
    frames_until(&mut h, "recompute did not end", |app| {
        app.state().running.is_none()
    });
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let c = Corrections::load(&run.join(CORRECTIONS_FILE)).unwrap();
    assert_eq!(c.rejects.len(), 1);
    assert_eq!((c.rejects[0].a, c.rejects[0].b), times);
    let r = report(&run);
    let line = r
        .loops
        .iter()
        .find(|l| (l.a, l.b) == (a, b) && !l.manual)
        .unwrap();
    assert!(!line.accepted);
    assert_eq!(line.reason.as_deref(), Some(VETOED));
    // a new state of the history holds it
    let states = driftless_steps::history::list(&run);
    assert_eq!(states.len(), 2, "{states:?}");
    // still listed, switched off; the others still on
    frames_until(&mut h, "panel not updated", |app| {
        app.state()
            .correct
            .auto_loops
            .iter()
            .any(|l| (l.a, l.b) == (a, b) && !l.used)
    });
    assert!(!switch_on(&h, a, b));
    for &(x, y) in &loops[1..] {
        assert!(switch_on(&h, x, y), "{x}–{y}");
    }
    assert!(
        h.query_by_label(text::CHANGED_RECOMPUTE).is_none(),
        "recomputed already"
    );

    // on again: the entry goes, the loop is used again
    h.get_by_label(&text::auto_loop_switch(a, b)).click();
    frames_until(&mut h, "reject not removed", |app| {
        matches!(app.state().running, Some(Action::Reuse(_)))
            || !app.state().correct.busy && app.state().correct.reuse_writes.is_none()
    });
    frames_until(&mut h, "recompute did not end", |app| {
        app.state().running.is_none()
            && app
                .state()
                .correct
                .auto_loops
                .iter()
                .any(|l| (l.a, l.b) == (a, b) && l.used)
    });
    assert!(Corrections::load(&run.join(CORRECTIONS_FILE))
        .unwrap()
        .rejects
        .is_empty());
    assert!(accepted(&run).contains(&(a, b)));
    assert!(switch_on(&h, a, b));
}

/// A `corrections.toml` of before the veto (manual loops only) still reads: its manual loop is
/// listed as before, the automatic loops all on.
#[test]
fn old_corrections_without_rejects_still_read() {
    let (h, run) = window_with_loops("veto-alte-datei", true);
    let c = &h.state().state().correct;
    let corrections = c.corrections.clone().unwrap().unwrap();
    assert_eq!(corrections.loops.len(), 1);
    assert!(corrections.rejects.is_empty());
    h.get_by_label("alt");
    let loops = accepted(&run);
    assert!(!loops.is_empty());
    for (a, b) in loops {
        assert!(switch_on(&h, a, b), "{a}–{b}");
    }
}
