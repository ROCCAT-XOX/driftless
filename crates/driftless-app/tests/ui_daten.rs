//! Step 1 „Daten“: the scan list with its status, the precision switch, „Alle rechnen“ with
//! its queue (one job at a time), a job that keeps running across a change of step, and
//! „Erweitert“ with the switches, corrections and comparisons.

use driftless_app::project::{MapInfo, MapStatus, Project, RunInfo, ScanEntry, ScanState};
use driftless_app::ui::steps::daten::{precision_of, scan_status, voxel_of, Precision, ScanStatus};
use driftless_app::ui::{steps, text, Action, AppState, DriftlessApp, ScanNode, Selection, Step};
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::map::MapSwitches;
use driftless_steps::slam::SlamSettings;
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, (AppState, Option<Action>)>;

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

/// A harness that shows the field „Daten“ of `state` and keeps the last action.
fn panel(state: AppState) -> H {
    let mut h = Harness::builder()
        .with_size([320.0, 1200.0])
        .build_ui_state(
            |ui, (state, action): &mut (AppState, Option<Action>)| {
                if let Some(a) = steps::show(ui, state) {
                    *action = Some(a);
                }
            },
            (state, None),
        );
    h.run();
    h
}

/// Clicks `label` and returns the action it gave.
fn click(h: &mut H, label: &str) -> Option<Action> {
    h.state_mut().1 = None;
    h.get_by_label(label).click();
    h.run();
    h.state_mut().1.take()
}

fn run(dir: &str, maps: Vec<MapInfo>) -> RunInfo {
    RunInfo {
        dir: PathBuf::from(dir),
        label: "SLAM 001".into(),
        settings: SlamSettings::default(),
        has_result: true,
        has_cache: true,
        has_previous: false,
        has_odometry: true,
        cloud: None,
        maps,
        history: Vec::new(),
    }
}

fn map(dir: &str) -> MapInfo {
    MapInfo {
        dir: PathBuf::from(dir),
        label: "Karte 001".into(),
        settings: Some(MapSwitches::default()),
        clouds: Vec::new(),
        surfaces: Vec::new(),
    }
}

/// A project „Schwarzer Raum“ with three scans (not read from disk): „Scan 13:25“ not
/// computed, „Scan 13:30“ with a fitting map, „Scan 13:35“ whose map no longer fits.
fn three_scans() -> AppState {
    let ids = ["scan-1325", "scan-1330", "scan-1335"];
    let names = ["Scan 13:25", "Scan 13:30", "Scan 13:35"];
    let runs = [
        Vec::new(),
        vec![run(
            "/p/scans/scan-1330/slam-001",
            vec![map("/p/scans/scan-1330/slam-001/karte-001")],
        )],
        vec![run(
            "/p/scans/scan-1335/slam-001",
            vec![map("/p/scans/scan-1335/slam-001/karte-001")],
        )],
    ];
    let mut state = AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Schwarzer Raum".into(),
            scans: ids
                .iter()
                .zip(names)
                .map(|(id, name)| ScanEntry {
                    id: (*id).into(),
                    path: PathBuf::from(format!("/daten/{id}")),
                    name: name.into(),
                    extra_runs: Vec::new(),
                })
                .collect(),
        }),
        scans: ids
            .iter()
            .zip(names)
            .zip(runs)
            .map(|((id, name), runs)| ScanNode {
                id: (*id).into(),
                name: name.into(),
                state: ScanState::Ready,
                no_color: None,
                runs,
            })
            .collect(),
        ..AppState::default()
    };
    let fresh = MapStatus {
        seconds: Some(10.0),
        stale: false,
    };
    let stale = MapStatus {
        seconds: Some(10.0),
        stale: true,
    };
    state.set_map_status(
        [
            ("/p/scans/scan-1330/slam-001/karte-001".into(), fresh),
            ("/p/scans/scan-1335/slam-001/karte-001".into(), stale),
        ]
        .into(),
    );
    state
}

#[test]
fn precision_maps_to_voxel() {
    assert_eq!(voxel_of(Precision::Schnell), Some(0.02));
    assert_eq!(voxel_of(Precision::Genau), Some(0.01));
    assert_eq!(voxel_of(Precision::Maximal), Some(0.005));
    assert_eq!(voxel_of(Precision::Eigene), None);
    assert_eq!(precision_of(0.01), Precision::Genau);
    assert_eq!(precision_of(0.02), Precision::Schnell);
    assert_eq!(precision_of(0.005), Precision::Maximal);
    assert_eq!(precision_of(0.013), Precision::Eigene);
    // „Genau“ is today's default: results do not change
    assert_eq!(precision_of(MapSwitches::default().voxel), Precision::Genau);
}

#[test]
fn scan_list_shows_status() {
    let state = three_scans();
    let status: Vec<ScanStatus> = state.scans.iter().map(|s| scan_status(&state, s)).collect();
    assert_eq!(
        status,
        [ScanStatus::New, ScanStatus::Done, ScanStatus::Stale]
    );
    let h = panel(state);
    h.get_by_label(text::STEP_DATA);
    h.get_by_label(&text::project_line("Schwarzer Raum", 3));
    for (name, status) in [
        ("Scan 13:25", text::STATUS_NEW),
        ("Scan 13:30", text::STATUS_DONE),
        ("Scan 13:35", text::STATUS_STALE),
    ] {
        let name = h.get_by_label(name).rect();
        // the status stands on the row of its scan
        let row = h
            .query_all_by_label(status)
            .find(|s| (s.rect().center().y - name.center().y).abs() < 6.0);
        assert!(row.is_some(), "{status} not next to its scan");
    }
    h.get_by_label(text::ADD_SCAN);
    // the heading stands out over everything below it
    let heading = h.get_by_label(text::STEP_DATA).rect().height();
    let section = h.get_by_label(text::PRECISION).rect().height();
    assert!(heading > section, "heading {heading} ≤ section {section}");
}

#[test]
fn main_button_computes_all_or_the_selected_scan() {
    let mut h = panel(three_scans());
    assert_eq!(
        click(&mut h, text::COMPUTE_ALL),
        Some(Action::ComputeAllMaps)
    );
    // a click on a scan selects it; the main button then computes only that one
    assert_eq!(
        click(&mut h, "Scan 13:35"),
        Some(Action::Select(Selection::Scan("scan-1335".into())))
    );
    h.state_mut().0.selection = Selection::Scan("scan-1335".into());
    h.run();
    assert_eq!(
        click(&mut h, text::COMPUTE_ONE),
        Some(Action::ComputeMap("scan-1335".into()))
    );
    // „Alle rechnen“ stays at hand
    assert_eq!(
        click(&mut h, text::COMPUTE_ALL),
        Some(Action::ComputeAllMaps)
    );
    // the selected scan shows its runs and maps below it
    h.get_by_label("SLAM 001");
    h.get_by_label("Karte 001");
    h.get_by_label(text::ADD_RUN);
}

#[test]
fn nothing_to_compute_says_so() {
    let mut state = three_scans();
    state.scans.truncate(2);
    state.scans.remove(0);
    let mut h = panel(state);
    h.get_by_label(text::ALL_COMPUTED);
    assert_eq!(click(&mut h, text::COMPUTE_ALL), None);
}

#[test]
fn maximal_shows_memory_hint() {
    let mut h = panel(three_scans());
    h.get_by_label(text::PRECISION);
    assert!(h.query_by_label(text::PRECISION_MAX_HINT).is_none());
    click(&mut h, text::PRECISION_MAX);
    assert_eq!(h.state().0.map_settings.switches.voxel, 0.005);
    h.get_by_label(text::PRECISION_MAX_HINT);
    click(&mut h, text::PRECISION_FAST);
    assert_eq!(h.state().0.map_settings.switches.voxel, 0.02);
    assert!(h.query_by_label(text::PRECISION_MAX_HINT).is_none());
    // a value of no level (from „Erweitert“) shows „Eigene“
    h.state_mut().0.map_settings.switches.voxel = 0.013;
    h.run();
    h.get_by_label(&text::precision_custom(0.013));
    // precision only sets the voxel size
    assert_eq!(
        MapSwitches {
            voxel: MapSwitches::default().voxel,
            ..h.state().0.map_settings.switches.clone()
        },
        MapSwitches::default()
    );
}

#[test]
fn advanced_holds_switches_compare_correct() {
    let mut state = three_scans();
    state.selection = Selection::Run("/p/scans/scan-1330/slam-001".into());
    let mut h = panel(state);
    for label in [
        text::GHOSTS,
        text::VOXEL,
        text::DOWNSCALE,
        text::CORRECTIONS,
        text::COMPARE,
        text::STEP_REUSE,
        text::LAYER_SUBMAPS,
        text::LAYER_REJECTED,
    ] {
        assert!(
            h.query_by_label(label).is_none(),
            "{label} before „Erweitert“"
        );
    }
    click(&mut h, text::ADVANCED);
    for label in [
        text::GHOSTS,
        text::OUTLIERS,
        text::VOXEL,
        text::DOWNSCALE,
        text::CORRECTIONS,
        text::COMPARE,
        text::STEP_REUSE,
        text::STEP_MAP,
        text::LAYER_SUBMAPS,
        text::LAYER_REJECTED,
        text::POINT,
    ] {
        h.get_by_label(label);
    }
    // and they work
    let ghosts = h.state().0.map_settings.switches.ghosts;
    click(&mut h, text::GHOSTS);
    assert_eq!(h.state().0.map_settings.switches.ghosts, !ghosts);
    click(&mut h, text::LAYER_SUBMAPS);
    assert!(h.state().0.view.show_submaps);
    assert_eq!(
        click(&mut h, text::STEP_MAP),
        Some(Action::RunMap("/p/scans/scan-1330/slam-001".into()))
    );
    // closed again: gone
    click(&mut h, text::ADVANCED);
    assert!(h.query_by_label(text::GHOSTS).is_none());
}

/// Ticks like frames do until no step and no background work is left; returns each step
/// action seen running, in order.
fn settle(app: &mut DriftlessApp) -> Vec<Action> {
    let mut seen: Vec<Action> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        app.tick();
        if let Some(a) = &app.state().running {
            if seen.last() != Some(a) {
                seen.push(a.clone());
            }
        }
        if !app.wants_repaint_while_busy() {
            return seen;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            driftless_app::ui::UiMessage::Error(e) => Some(e.clone()),
            driftless_app::ui::UiMessage::Info(_) => None,
        })
        .collect()
}

fn add_scan(app: &mut DriftlessApp, dir: &Path, name: &str) -> String {
    let scan = dir.join(name);
    std::fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.5);
    app.apply(Action::AddScan(scan));
    app.state()
        .project
        .as_ref()
        .unwrap()
        .scans
        .last()
        .unwrap()
        .id
        .clone()
}

fn scan_path(app: &DriftlessApp, id: &str) -> PathBuf {
    app.state()
        .project
        .as_ref()
        .unwrap()
        .scan(id)
        .unwrap()
        .path
        .clone()
}

fn newest_run(app: &DriftlessApp, id: &str) -> PathBuf {
    app.state()
        .scan_node(id)
        .unwrap()
        .runs
        .last()
        .unwrap()
        .dir
        .clone()
}

fn status_of(app: &DriftlessApp, id: &str) -> ScanStatus {
    scan_status(app.state(), app.state().scan_node(id).unwrap())
}

#[test]
fn compute_all_queues_new_and_stale() {
    let dir = tmp("ui-daten-alle");
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    let a = add_scan(&mut app, &dir, "a");
    let b = add_scan(&mut app, &dir, "b");
    let c = add_scan(&mut app, &dir, "c");
    // b: computed; c: computed, then SLAM again (its map belongs to the older run)
    for (id, again) in [(&b, false), (&c, true)] {
        app.apply(Action::RunSlam(id.clone()));
        settle(&mut app);
        app.apply(Action::RunMap(newest_run(&app, id)));
        settle(&mut app);
        if again {
            app.apply(Action::RunSlam(id.clone()));
            settle(&mut app);
        }
    }
    assert_eq!(errors(&app), Vec::<String>::new());
    assert_eq!(status_of(&app, &a), ScanStatus::New);
    assert_eq!(status_of(&app, &b), ScanStatus::Done);
    assert_eq!(status_of(&app, &c), ScanStatus::Stale);

    app.apply(Action::ComputeAllMaps);
    // exactly the new and the stale ones, in the order of the list; one job runs
    let queue: Vec<PathBuf> = app.state().queue.iter().cloned().collect();
    assert_eq!(queue, [scan_path(&app, &a), scan_path(&app, &c)]);
    assert_eq!(app.state().running, Some(Action::RunSlam(a.clone())));
    assert!(matches!(status_of(&app, &a), ScanStatus::Running(_)));
    assert_eq!(status_of(&app, &c), ScanStatus::Queued);
    let c_run = newest_run(&app, &c);
    let seen = settle(&mut app);
    let a_run = newest_run(&app, &a);
    // one after the other: SLAM of a, then its map, then the map of c's newest run
    assert_eq!(
        seen,
        [
            Action::RunSlam(a.clone()),
            Action::RunMap(a_run),
            Action::RunMap(c_run),
        ]
    );
    assert_eq!(errors(&app), Vec::<String>::new());
    assert!(app.state().queue.is_empty());
    for id in [&a, &b, &c] {
        assert_eq!(status_of(&app, id), ScanStatus::Done, "{id}");
    }

    // nothing left: nothing starts
    app.apply(Action::ComputeAllMaps);
    assert!(app.state().queue.is_empty());
    assert!(app.state().running.is_none());

    // „Abbrechen“ empties the queue: the next scan never starts
    let d = add_scan(&mut app, &dir, "d");
    let e = add_scan(&mut app, &dir, "e");
    app.apply(Action::ComputeAllMaps);
    assert_eq!(app.state().queue.len(), 2);
    assert_eq!(app.state().running, Some(Action::RunSlam(d)));
    app.apply(Action::Cancel);
    assert!(app.state().queue.is_empty());
    let seen = settle(&mut app);
    assert!(seen.len() <= 1, "{seen:?}");
    assert!(app.state().scan_node(&e).unwrap().runs.is_empty());
    assert!(app.state().running.is_none());
}

#[test]
fn compute_one_scan_runs_only_that_one() {
    let dir = tmp("ui-daten-einer");
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    let a = add_scan(&mut app, &dir, "a");
    let b = add_scan(&mut app, &dir, "b");
    app.apply(Action::ComputeMap(b.clone()));
    let queue: Vec<PathBuf> = app.state().queue.iter().cloned().collect();
    assert_eq!(queue, [scan_path(&app, &b)]);
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert_eq!(status_of(&app, &a), ScanStatus::New);
    assert_eq!(status_of(&app, &b), ScanStatus::Done);
}

#[test]
fn job_survives_step_switch() {
    let mut app = app();
    app.start_job(text::STEP_MAP, |p| {
        p.phase("Dichte Wolke", Some(100));
        p.advance(40);
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(driftless_steps::StepError::Cancelled)
    })
    .unwrap();
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run_steps(3);
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.run_steps(3);
    assert_eq!(h.state().state().step, Step::Bilder);
    // still running, with its progress and „Abbrechen“ in the status bar
    assert!(h.state().state().job.is_some());
    h.get_by_role(Role::ProgressIndicator);
    h.get_by_label_contains(text::STEP_MAP);
    h.get_by_label(text::CANCEL).click();
    let deadline = Instant::now() + Duration::from_secs(10);
    while h.state().state().job.is_some() {
        assert!(Instant::now() < deadline, "job did not stop");
        std::thread::sleep(Duration::from_millis(5));
        h.step();
    }
}
