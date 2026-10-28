//! Before and after: two trajectories of a scan compared as `driftless compare` does (a
//! aligned to b), the presets after a recompute and without one, the states of a run's
//! history in the tree, restoring a state (a new state, the old ones stay), the error of
//! trajectories without common times, and two `map.json` side by side.

use driftless_app::overlay::{RunOverlay, COMPARE_B_COLOR, COMPARE_COLOR};
use driftless_app::project::{canonical, Project, RunInfo};
use driftless_app::ui::compare::{
    compare_maps, compare_refs, default_refs, highlighted, map_rows, refs_of_scan, report_rows,
    TrajRef,
};
use driftless_app::ui::{actions, text, Action, DriftlessApp, Selection, UiMessage, ViewSettings};
use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_slam::Corrections;
use driftless_steps::compare::compare;
use driftless_steps::correct;
use driftless_steps::history::{self, HistoryKind};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings, CORRECTIONS_FILE};
use driftless_traj::nalgebra::Vector3;
use driftless_traj::{read_tum, write_tum};
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use serde_json::json;
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

/// A project with the scan `halle` (12 s standing still) and one SLAM run of it.
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

/// The window of `app` (no GPU: the view shows only its note), once everything is read.
fn harness(app: DriftlessApp) -> H {
    let mut h = Harness::builder()
        .with_size([1400.0, 2400.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    // the corrections and comparisons lie under „Erweitert“
    driftless_app::ui::steps::daten::set_advanced(&h.ctx, true);
    frames_until(&mut h, |app| {
        app.overlay().is_some() && !app.wants_repaint_while_busy()
    });
    h
}

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

fn idle(h: &mut H) {
    frames_until(h, |app| {
        app.state().job.is_none() && !app.wants_repaint_while_busy()
    });
}

/// The comparison the panel shows: its references and report.
fn shown(app: &DriftlessApp) -> (TrajRef, TrajRef, driftless_steps::compare::Report) {
    let c = &app.state().compare;
    let result = c
        .result
        .as_ref()
        .expect("compared")
        .as_ref()
        .expect("no error");
    assert_eq!(Some(&result.a), c.a.as_ref());
    assert_eq!(Some(&result.b), c.b.as_ref());
    (result.a.clone(), result.b.clone(), result.report.clone())
}

/// What `driftless compare a b --json` prints, as the steps compute it.
fn cli_report(a: &Path, b: &Path) -> driftless_steps::compare::Report {
    compare(&read_tum(a).unwrap(), &read_tum(b).unwrap(), 0.01).unwrap()
}

/// Every row of the comparison table is in the panel.
fn table_shown(h: &H, report: &driftless_steps::compare::Report) {
    for (label, value) in report_rows(report) {
        h.get_by_label(label);
        assert!(
            h.get_all_by_label(&value).next().is_some(),
            "{label}: {value}"
        );
    }
}

/// The text shown in the choice `label`.
fn chosen(h: &H, label: &str) -> String {
    let combo = h
        .get_all_by_label(label)
        .find(|n| n.accesskit_node().role() == Role::ComboBox)
        .expect("choice");
    combo
        .accesskit_node()
        .value()
        .unwrap_or_default()
        .to_string()
}

#[test]
fn previous_vs_current_after_reuse() {
    let scratch = tmp("vergleich-vorher");
    let (mut app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "Treppe", ADDED).unwrap();
    app.apply(Action::Reuse(run.clone()));
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert!(run.join("traj.vorher.tum").is_file());
    let mut h = harness(app);
    idle(&mut h);
    let (a, b, report) = shown(h.state());
    assert_eq!(a, TrajRef::Previous(run.clone()));
    assert_eq!(b, TrajRef::Current(run.clone()));
    let expected = cli_report(&run.join("traj.vorher.tum"), &run.join("traj.tum"));
    assert_eq!(report, expected);
    // as `driftless compare` serialises it
    assert_eq!(
        serde_json::to_value(&report).unwrap(),
        serde_json::to_value(&expected).unwrap()
    );
    table_shown(&h, &report);
    assert_eq!(chosen(&h, text::COMPARE_A), "SLAM 001 vorher");
    assert_eq!(chosen(&h, text::COMPARE_B), "SLAM 001 aktuell");
}

#[test]
fn odometry_vs_current_default_without_previous() {
    let scratch = tmp("vergleich-odometrie");
    let (app, run) = app_with_run(&scratch);
    let mut h = harness(app);
    idle(&mut h);
    let (a, b, report) = shown(h.state());
    assert_eq!(a, TrajRef::Odometry(run.clone()));
    assert_eq!(b, TrajRef::Current(run.clone()));
    assert_eq!(
        report,
        cli_report(&run.join("traj_odom.tum"), &run.join("traj.tum"))
    );
    table_shown(&h, &report);
    // the run in the view carries a, aligned to b, for the orange line
    let o = h.state().overlay().unwrap();
    assert_eq!(o.compare.len(), report.poses.a);
    let lines = o.lines(&ViewSettings::default(), &[]).0;
    let orange = lines
        .iter()
        .filter(|v| v.rgba & 0xff_ffff == rgb(COMPARE_COLOR))
        .count();
    assert_eq!(orange, 2 * (report.poses.a - 1));
    // B in one neutral colour instead of the quality colours, with the legend
    let neutral = lines
        .iter()
        .filter(|v| v.rgba & 0xff_ffff == rgb(COMPARE_B_COLOR))
        .count();
    assert_eq!(neutral, 2 * (report.poses.b - 1));
    assert_eq!(
        o.compare_legend,
        Some([
            text::legend("A", "SLAM 001 Odometrie"),
            text::legend("B", "SLAM 001 aktuell")
        ])
    );
    let off = ViewSettings {
        show_compare: false,
        ..ViewSettings::default()
    };
    assert!(o
        .lines(&off, &[])
        .0
        .iter()
        .all(|v| v.rgba & 0xff_ffff != rgb(COMPARE_COLOR)
            && v.rgba & 0xff_ffff != rgb(COMPARE_B_COLOR)));
}

fn rgb(c: [u8; 3]) -> u32 {
    u32::from(c[0]) | u32::from(c[1]) << 8 | u32::from(c[2]) << 16
}

#[test]
fn history_refs_resolve() {
    let run = PathBuf::from("projekt/scans/halle/slam-001");
    let refs = [
        (
            TrajRef::Current(run.clone()),
            "traj.tum",
            "SLAM 001 aktuell",
        ),
        (
            TrajRef::Odometry(run.clone()),
            "traj_odom.tum",
            "SLAM 001 Odometrie",
        ),
        (
            TrajRef::Previous(run.clone()),
            "traj.vorher.tum",
            "SLAM 001 vorher",
        ),
        (
            TrajRef::History {
                run: run.clone(),
                nr: 3,
            },
            "verlauf/003/traj.tum",
            "SLAM 001 Stand 003",
        ),
    ];
    for (r, file, label) in refs {
        assert_eq!(r.path(), run.join(file));
        assert_eq!(r.label(), label);
        assert_eq!(r.run(), run.as_path());
    }
    // a run added from the CLI keeps its folder name
    let cli = TrajRef::Current(PathBuf::from("/daten/ergebnisse"));
    assert_eq!(cli.label(), "ergebnisse aktuell");

    // all trajectories of the scan: per run current, odometry, previous, states
    let scratch = tmp("vergleich-verlauf");
    let (mut app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "", ADDED).unwrap();
    app.apply(Action::Reuse(run.clone()));
    settle(&mut app);
    let scan = app.state().scan_node("halle").unwrap().clone();
    let scan = &scan;
    assert_eq!(
        refs_of_scan(scan),
        vec![
            TrajRef::Current(run.clone()),
            TrajRef::Odometry(run.clone()),
            TrajRef::Previous(run.clone()),
            TrajRef::History {
                run: run.clone(),
                nr: 1
            },
            TrajRef::History {
                run: run.clone(),
                nr: 2
            },
        ]
    );
    for r in refs_of_scan(scan) {
        assert!(r.path().is_file(), "{}", r.path().display());
    }
    let info = &scan.runs[0];
    assert_eq!(
        default_refs(info),
        Some((
            TrajRef::Previous(run.clone()),
            TrajRef::Current(run.clone())
        ))
    );
    // without traj.vorher.tum and traj_odom.tum: the state before the last
    let bare = RunInfo {
        has_previous: false,
        has_odometry: false,
        ..info.clone()
    };
    assert_eq!(
        default_refs(&bare),
        Some((
            TrajRef::History {
                run: run.clone(),
                nr: 1
            },
            TrajRef::Current(run.clone())
        ))
    );
    let single = RunInfo {
        history: info.history[..1].to_vec(),
        ..bare.clone()
    };
    assert_eq!(default_refs(&single), None);
    let none = RunInfo {
        has_result: false,
        ..info.clone()
    };
    assert_eq!(default_refs(&none), None);

    // the states in the tree, with kind and start↔end gap of their slam.json
    let mut h = harness(app);
    idle(&mut h);
    h.get_by_label(text::HISTORY).click();
    h.step();
    h.step();
    for e in history::list(&run) {
        let gap = h.state().state().history_gaps.get(&e.dir).copied();
        assert!(gap.is_some(), "Start↔Ende of state {}", e.nr);
        h.get_by_label(&text::history_entry(&e, gap));
    }
    // choosing one: compare it with the current trajectory
    h.state_mut().apply(Action::Select(Selection::History {
        run: run.clone(),
        nr: 1,
    }));
    h.step();
    h.get_by_label(text::COMPARE_WITH_CURRENT).click();
    idle(&mut h);
    let (a, b, report) = shown(h.state());
    assert_eq!(
        a,
        TrajRef::History {
            run: run.clone(),
            nr: 1
        }
    );
    assert_eq!(b, TrajRef::Current(run.clone()));
    assert_eq!(
        report,
        cli_report(&run.join("verlauf/001/traj.tum"), &run.join("traj.tum"))
    );
}

#[test]
fn restore_history_creates_new_state() {
    let scratch = tmp("vergleich-wiederherstellen");
    let (mut app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "Treppe", ADDED).unwrap();
    app.apply(Action::Reuse(run.clone()));
    settle(&mut app);
    assert_eq!(history::list(&run).len(), 2);
    let one = run.join("verlauf/001");
    let then = Corrections::load(&one.join(CORRECTIONS_FILE)).unwrap();
    assert!(then.loops.is_empty());
    assert!(correct::load(&run).unwrap().loops[0].enabled);

    // not while a change of the corrections is being written
    app.state_mut().correct.busy = true;
    let mut runner = driftless_app::jobs::JobRunner::new(Arc::new(|| {}));
    let restore = Action::RestoreHistory {
        run: run.clone(),
        nr: 1,
    };
    assert_eq!(
        actions::start_step(app.state_mut(), &mut runner, &restore),
        Err(text::CORRECTION_BUSY.to_string())
    );
    app.state_mut().correct.busy = false;

    let mut h = harness(app);
    h.state_mut().apply(Action::Select(Selection::History {
        run: run.clone(),
        nr: 1,
    }));
    h.step();
    h.get_by_label(text::RESTORE_STATE).click();
    h.step();
    assert_eq!(h.state().state().running, Some(restore.clone()));
    // the corrections are locked while it runs
    assert!(driftless_app::ui::correct::locked(h.state().state(), &run).is_some());
    idle(&mut h);
    assert_eq!(errors(h.state()), Vec::<String>::new());

    // switches as in state 1 (the later entry off, nothing removed), a new state 3
    let now = correct::load(&run).unwrap();
    assert_eq!(now.loops.len(), 1);
    assert!(!now.loops[0].enabled);
    let states = history::list(&run);
    assert_eq!(
        states.iter().map(|e| e.nr).collect::<Vec<_>>(),
        vec![1, 2, 3]
    );
    assert_eq!(states[2].kind, HistoryKind::Reuse);
    assert_eq!(
        fs::read(run.join("verlauf/003/traj.tum")).unwrap(),
        fs::read(one.join("traj.tum")).unwrap()
    );
    assert_eq!(
        fs::read(run.join("traj.tum")).unwrap(),
        fs::read(one.join("traj.tum")).unwrap()
    );
    // the older states are unchanged
    assert!(run.join("verlauf/002/traj.tum").is_file());
}

#[test]
fn compare_mismatched_trajectories_reports_error() {
    let scratch = tmp("vergleich-fehler");
    let (app, run) = app_with_run(&scratch);
    // a trajectory 1000 s later: no pose has a partner within 0.01 s
    let mut late = read_tum(&run.join("traj.tum")).unwrap();
    for p in &mut late {
        p.t += 1000.0;
        p.p += Vector3::new(1.0, 0.0, 0.0);
    }
    write_tum(&run.join("traj.vorher.tum"), &late).unwrap();
    let err = compare_refs(
        &TrajRef::Previous(run.clone()),
        &TrajRef::Current(run.clone()),
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("zu wenige zugeordnete Posen"), "{err}");

    let mut h = harness(app);
    idle(&mut h);
    h.state_mut().apply(Action::Compare {
        a: TrajRef::Previous(run.clone()),
        b: TrajRef::Current(run.clone()),
    });
    idle(&mut h);
    let c = &h.state().state().compare;
    let shown_err = c.result.as_ref().unwrap().as_ref().unwrap_err();
    assert_eq!(*shown_err, err);
    h.get_by_label(&err);
    // no orange line for a failed comparison
    assert!(h.state().overlay().unwrap().compare.is_empty());
}

/// A `map.json` as `driftless map` writes it, with steps `steps` and `--no-smooth` if
/// `smooth` is false.
fn map_json(smooth: bool, steps: &[&str]) -> serde_json::Value {
    let mut all = serde_json::Map::new();
    for s in steps {
        let entry = match *s {
            "dense" => json!({"seconds": 27.5, "stats": {"points": 15_604_354}}),
            "clean" => json!({"seconds": 8.25, "stats": {
                "kept": 14_976_105, "ghosts": 397_239, "outliers": 231_010, "through": 0,
                "smooth": {"thickness_before": 0.031, "thickness_after": 0.012}}}),
            _ => json!({"seconds": 302.0, "stats": {"points_colored": 14_573_540,
                "camera_difference_before": 3.33, "camera_difference_after": 0.575,
                "seam_jump_before": 2.0, "seam_jump_after": 1.0}}),
        };
        all.insert((*s).to_string(), entry);
    }
    json!({
        "settings": {
            "dense": {"voxel": 0.05},
            "clean": {"ghosts": true, "outliers": true, "through": true,
                      "smooth": {"enabled": smooth}},
            "color": {"downscale": 2, "highlights": false,
                      "exposure": {"vignetting": true}},
        },
        "steps": all,
        "seconds_total": 338.0,
    })
}

#[test]
fn map_rows_side_by_side() {
    let a = map_json(true, &["dense", "clean", "color"]);
    let b = map_json(false, &["dense", "clean"]);
    let rows = map_rows(&a, &b);
    let row = |label: &str| {
        rows.iter()
            .find(|r| r.0 == label)
            .unwrap_or_else(|| panic!("row {label}: {rows:?}"))
            .clone()
    };
    assert_eq!(
        row(text::SMOOTH),
        (text::SMOOTH.into(), "an".into(), "aus".into())
    );
    assert_eq!(
        row(text::GHOSTS),
        (text::GHOSTS.into(), "an".into(), "an".into())
    );
    // a step missing in b
    let colored = row(text::MAP_POINTS_COLORED);
    assert_eq!(colored.1, "14.573.540");
    assert_eq!(colored.2, "–");
    let ghosts = row(text::MAP_REMOVED_GHOSTS);
    assert_eq!(ghosts.1, ghosts.2);
    assert_eq!(row(text::MAP_THICKNESS_AFTER).1, "0,012 m");
    assert_eq!(row(text::MAP_SECONDS_TOTAL).1, "338,0 s");
    // read from two map folders; a folder without map.json is a German error
    let dir = tmp("vergleich-karten");
    for (name, json) in [("karte-001", &a), ("karte-002", &b)] {
        fs::create_dir_all(dir.join(name)).unwrap();
        fs::write(dir.join(name).join("map.json"), json.to_string()).unwrap();
    }
    let m = compare_maps(&dir.join("karte-001"), &dir.join("karte-002")).unwrap();
    assert_eq!(m.rows, rows);
    let err = compare_maps(&dir.join("karte-001"), &dir.join("fehlt"))
        .unwrap_err()
        .to_string();
    assert!(err.contains("nicht lesbar"), "{err}");
    // runtimes always differ: not highlighted; real differences are
    assert!(highlighted(text::SMOOTH, "an", "aus"));
    assert!(!highlighted(text::GHOSTS, "an", "an"));
    let total = row(text::MAP_SECONDS_TOTAL);
    assert!(!highlighted(&total.0, "338,0 s", "12,0 s"));
    // nothing at all: every value „–“
    let empty = map_rows(&json!({}), &json!(null));
    assert_eq!(empty.len(), rows.len());
    assert!(empty.iter().all(|r| r.1 == "–" && r.2 == "–"));
}

#[test]
fn overlay_compare_follows_origin() {
    let scratch = tmp("vergleich-ursprung");
    let (app, run) = app_with_run(&scratch);
    let h = harness(app);
    let c = compare_refs(
        &TrajRef::Odometry(run.clone()),
        &TrajRef::Current(run.clone()),
    )
    .unwrap();
    let mut o: RunOverlay = h.state().overlay().unwrap().clone();
    o.set_comparison(Some(&c));
    o.set_origin(Some([1.0, 2.0, 3.0]));
    let p = c.align * driftless_traj::nalgebra::Point3::from(c.a_poses[0].p);
    let rel = o.compare_positions[0];
    assert!((f64::from(rel[0]) - (p.x - 1.0)).abs() < 1e-4);
    assert!((f64::from(rel[2]) - (p.z - 3.0)).abs() < 1e-4);
    o.set_comparison(None);
    assert!(o.compare.is_empty() && o.compare_positions.is_empty());
}

/// The files of `dir` and their bytes (`verlauf/` included).
fn files(dir: &Path) -> std::collections::BTreeMap<PathBuf, Vec<u8>> {
    let mut out = std::collections::BTreeMap::new();
    let mut todo = vec![dir.to_path_buf()];
    while let Some(d) = todo.pop() {
        for e in fs::read_dir(&d).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                todo.push(p);
            } else {
                out.insert(p.clone(), fs::read(&p).unwrap());
            }
        }
    }
    out
}

#[test]
fn failed_restore_writes_the_corrections_back() {
    let scratch = tmp("vergleich-rueckschreiben");
    let (mut app, run) = app_with_run(&scratch);
    correct::add_loop_at(&run, 1.0, 11.0, "Treppe", ADDED).unwrap();
    app.apply(Action::Reuse(run.clone()));
    settle(&mut app);
    let before = fs::read(run.join(CORRECTIONS_FILE)).unwrap();
    // the recompute after the switches were restored fails: a broken submaps.bin
    fs::write(run.join("submaps.bin"), b"kaputt").unwrap();
    app.apply(Action::RestoreHistory {
        run: run.clone(),
        nr: 1,
    });
    settle(&mut app);
    let errors = errors(&app);
    assert!(
        errors.iter().any(|e| e.starts_with(text::STEP_RESTORE)),
        "{errors:?}"
    );
    assert_eq!(fs::read(run.join(CORRECTIONS_FILE)).unwrap(), before);
    assert_eq!(history::list(&run).len(), 2, "no new state");
    // the panel shows the file as it is on disk again
    let shown = app.state().correct.corrections.clone().unwrap().unwrap();
    assert_eq!(shown, correct::load(&run).unwrap());
    assert!(shown.loops[0].enabled);
}

/// A project with the scan `halle` and a run made by `driftless slam` (no history) with
/// `settings`, added to the scan.
fn app_with_cli_run(dir: &Path, settings: SlamSettings) -> (DriftlessApp, PathBuf) {
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 12.0);
    let out = dir.join("cli-lauf");
    let req = SlamRequest {
        scan: scan.clone(),
        out: out.clone(),
        settings,
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let mut p = Project::create(&dir.join("projekt"), "Halle").unwrap();
    let id = p.add_scan(&scan).unwrap();
    p.add_run(&id, &out).unwrap();
    p.save().unwrap();
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::OpenProject(dir.join("projekt")));
    let out = canonical(&out).unwrap();
    app.apply(Action::Select(Selection::Run(out.clone())));
    settle(&mut app);
    (app, out)
}

#[test]
fn restore_imported_state_of_a_cli_run() {
    let scratch = tmp("vergleich-cli");
    let (mut app, run) = app_with_cli_run(&scratch, SlamSettings::default());
    let cli_traj = fs::read(run.join("traj.tum")).unwrap();
    // added after the run was computed: not part of the state kept as „übernommen“
    correct::add_loop_at(&run, 1.0, 11.0, "", "2099-01-01T00:00:00Z").unwrap();
    app.apply(Action::Reuse(run.clone()));
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    let states = history::list(&run);
    assert_eq!(
        states.iter().map(|e| e.kind).collect::<Vec<_>>(),
        vec![HistoryKind::Imported, HistoryKind::Reuse]
    );
    assert_eq!(
        fs::read(run.join("verlauf/001/traj.tum")).unwrap(),
        cli_traj
    );

    app.apply(Action::RestoreHistory {
        run: run.clone(),
        nr: 1,
    });
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert!(!correct::load(&run).unwrap().loops[0].enabled);
    assert_eq!(history::list(&run).len(), 3);
    assert_eq!(
        fs::read(run.join("verlauf/003/traj.tum")).unwrap(),
        cli_traj
    );
    assert_eq!(fs::read(run.join("traj.tum")).unwrap(), cli_traj);
}

#[test]
fn restore_of_a_cli_run_with_other_settings_changes_nothing() {
    let settings = SlamSettings {
        lidar_topic: Some("/lidar".into()),
        ..SlamSettings::default()
    };
    let scratch = tmp("vergleich-cli-andere");
    let (mut app, run) = app_with_cli_run(&scratch, settings);
    correct::add_loop_at(&run, 1.0, 11.0, "", ADDED).unwrap();
    // a state kept earlier (as a reuse with these settings on the command line would)
    history::record(&run, HistoryKind::Imported, 0.0).unwrap();
    correct::set_enabled(&run, 1, false).unwrap();
    app.state_mut().refresh();
    let before = files(&run);
    app.apply(Action::RestoreHistory {
        run: run.clone(),
        nr: 1,
    });
    settle(&mut app);
    let errors = errors(&app);
    assert!(
        errors
            .iter()
            .any(|e| e.contains("der Lauf wurde mit anderen SLAM-Einstellungen")),
        "{errors:?}"
    );
    assert_eq!(files(&run), before, "nothing changed on disk");
}

#[test]
fn comparison_follows_a_recomputed_run() {
    let scratch = tmp("vergleich-zwei-laeufe");
    let (mut app, one) = app_with_run(&scratch);
    app.apply(Action::RunSlam("halle".into()));
    settle(&mut app);
    let two = one.with_file_name("slam-002");
    assert_eq!(app.state().selection, Selection::Run(two.clone()));
    app.apply(Action::Compare {
        a: TrajRef::Current(one.clone()),
        b: TrajRef::Current(two.clone()),
    });
    settle(&mut app);
    assert!(app.state().compare.current().is_some());
    // recompute the other run: the numbers shown are of the files as they are now
    correct::add_loop_at(&one, 1.0, 11.0, "", ADDED).unwrap();
    app.apply(Action::Reuse(one.clone()));
    settle(&mut app);
    let c = app.state().compare.current().expect("compared again");
    assert_eq!(
        c.report,
        cli_report(&c.a.path(), &c.b.path()),
        "{} / {}",
        c.a.label(),
        c.b.label()
    );
    assert!(c.a.run() == one || c.b.run() == one);
}

#[test]
fn history_times_are_local() {
    let created = "2026-10-21T14:03:12Z";
    let local = chrono::DateTime::parse_from_rfc3339(created)
        .unwrap()
        .with_timezone(&chrono::Local)
        .format("%d.%m.%Y %H:%M")
        .to_string();
    assert_eq!(text::stamp(created), local);
    assert_eq!(text::stamp("kaputt"), "kaputt");
}
