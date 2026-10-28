//! Step 1 „Daten“: the scan list with its status, the precision switch, „Alle rechnen“ with
//! its queue (one job at a time), a job that keeps running across a change of step, and
//! „Erweitert“ with the switches, corrections and comparisons.

use driftless_app::project::{
    MapInfo, MapStatus, MergePick, ModelInfo, Project, RunInfo, ScanEntry, ScanState, SurfaceInfo,
};
use driftless_app::render::ColorMode;
use driftless_app::ui::steps::daten::{precision_of, scan_status, voxel_of, Precision, ScanStatus};
use driftless_app::ui::{
    steps, text, Action, AppState, DriftlessApp, Queued, ScanNode, Selection, Step,
};
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::map::MapSwitches;
use driftless_steps::merge::{ModelMember, ModelReport, MODEL_LAS, MODEL_REPORT};
use driftless_steps::slam::SlamSettings;
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::{NodeT, Queryable};
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
                    bags: Vec::new(),
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
            _ => None,
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
    let queue: Vec<Queued> = app.state().queue.iter().cloned().collect();
    assert_eq!(
        queue,
        [
            Queued::Scan(scan_path(&app, &a)),
            Queued::Scan(scan_path(&app, &c))
        ]
    );
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
    // it starts once the new scans' maps are checked
    tick_until(&mut app, |a| a.state().running.is_some());
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
    let queue: Vec<Queued> = app.state().queue.iter().cloned().collect();
    assert_eq!(queue, [Queued::Scan(scan_path(&app, &b))]);
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

/// Ticks until `cond` holds.
fn tick_until(app: &mut DriftlessApp, cond: impl Fn(&DriftlessApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(60);
    while !cond(app) {
        assert!(Instant::now() < deadline, "condition not reached");
        std::thread::sleep(Duration::from_millis(2));
        app.tick();
    }
}

/// An app with a new project in `dir` and the new scans `names`, map statuses known.
fn project_with(dir: &Path, names: &[&str]) -> (DriftlessApp, Vec<String>) {
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    let ids = names.iter().map(|n| add_scan(&mut app, dir, n)).collect();
    settle(&mut app);
    (app, ids)
}

/// The `slam-NNN` folders of the scan `id` on disk.
fn run_dirs(project: &Path, id: &str) -> usize {
    std::fs::read_dir(project.join("scans").join(id))
        .map(|d| {
            d.filter_map(Result::ok)
                .filter(|e| e.file_name().to_string_lossy().starts_with("slam-"))
                .count()
        })
        .unwrap_or(0)
}

#[test]
fn closing_empties_the_queue() {
    let dir = tmp("ui-daten-schliessen");
    let (mut app, ids) = project_with(&dir, &["a", "b"]);
    let project = app.state().project.as_ref().unwrap().dir.clone();
    app.apply(Action::ComputeAllMaps);
    assert_eq!(app.state().queue.len(), 2);
    // a step runs: the close is put off, and nothing of the queue follows
    assert!(!app.close_requested());
    assert!(app.state().queue.is_empty());
    tick_until(&mut app, |a| a.state().running.is_none());
    settle(&mut app);
    assert!(app.take_close());
    assert_eq!(run_dirs(&project, &ids[1]), 0);
}

#[test]
fn project_switch_empties_the_queue() {
    let dir = tmp("ui-daten-wechsel");
    let (mut app, ids) = project_with(&dir, &["a", "b"]);
    let old = app.state().project.as_ref().unwrap().dir.clone();
    app.apply(Action::ComputeAllMaps);
    assert_eq!(app.state().queue.len(), 2);
    app.apply(Action::NewProject {
        dir: dir.join("anderes"),
        name: "Anderes".into(),
    });
    assert!(app.state().queue.is_empty());
    settle(&mut app);
    assert_eq!(app.state().project.as_ref().unwrap().name, "Anderes");
    assert!(app.state().queue.is_empty());
    assert!(app.state().running.is_none());
    // the second scan of the old project never started
    assert_eq!(run_dirs(&old, &ids[1]), 0);
}

#[test]
fn failed_step_stops_the_queue() {
    // Chosen: a failing scan stops „Alle rechnen“ with its error and empties the queue, so
    // that nothing runs on unseen after a problem.
    let dir = tmp("ui-daten-fehler");
    let (mut app, ids) = project_with(&dir, &["a", "b", "c"]);
    app.apply(Action::ComputeAllMaps);
    assert_eq!(app.state().queue.len(), 3);
    // b's folder goes while a is computed
    std::fs::remove_dir_all(dir.join("b")).unwrap();
    settle(&mut app);
    assert_eq!(status_of(&app, &ids[0]), ScanStatus::Done);
    assert!(!errors(&app).is_empty(), "no error shown");
    assert!(app.state().queue.is_empty());
    assert_eq!(status_of(&app, &ids[2]), ScanStatus::New);
    assert!(app.state().scan_node(&ids[2]).unwrap().runs.is_empty());
}

#[test]
fn queue_keeps_the_settings_it_started_with() {
    let dir = tmp("ui-daten-schnappschuss");
    let (mut app, ids) = project_with(&dir, &["a", "b"]);
    app.apply(Action::ComputeAllMaps);
    // changed while the first scan is computed: the queue does not take it
    app.state_mut().map_settings.switches.voxel = 0.02;
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    for id in &ids {
        let node = app.state().scan_node(id).unwrap();
        let map = node.runs.last().unwrap().maps.last().unwrap();
        assert_eq!(map.settings.as_ref().unwrap().voxel, 0.01, "{id}");
    }
    assert_eq!(app.state().map_settings.switches.voxel, 0.02);
}

#[test]
fn compute_all_waits_for_the_map_check() {
    let dir = tmp("ui-daten-pruefung");
    let (mut app, _) = project_with(&dir, &["a"]);
    // a new scan: the tree changed, its maps are being checked
    let b = add_scan(&mut app, &dir, "b");
    app.apply(Action::ComputeAllMaps);
    tick_until(&mut app, |a| a.state().running.is_some());
    let queue: Vec<Queued> = app.state().queue.iter().cloned().collect();
    assert_eq!(queue.len(), 2);
    assert_eq!(queue[1], Queued::Scan(scan_path(&app, &b)));
    settle(&mut app);

    // the field says so instead of „Alle Scans sind gerechnet“
    let mut state = three_scans();
    state.scans.truncate(2);
    state.scans.remove(0);
    state.map_status_pending = true;
    let h = panel(state);
    h.get_by_label(text::MAP_STATUS_PENDING);
    assert!(h.query_by_label(text::ALL_COMPUTED).is_none());
}

#[test]
fn new_precision_with_all_done_says_what_to_do() {
    let mut state = three_scans();
    state.scans.truncate(2);
    state.scans.remove(0);
    let mut h = panel(state);
    h.get_by_label(text::ALL_COMPUTED);
    assert!(h.query_by_label(text::RECOMPUTE_HINT).is_none());
    click(&mut h, text::PRECISION_MAX);
    h.get_by_label(text::RECOMPUTE_HINT);
    click(&mut h, text::PRECISION_EXACT);
    assert!(h.query_by_label(text::RECOMPUTE_HINT).is_none());
}

#[test]
fn no_colour_locks_compute_when_colouring() {
    use driftless_steps::map::MapStep;
    let why = "Keine Kamerabilder im Scan";
    let mut state = three_scans();
    state.scans[2].no_color = Some(why.into());
    state.map_settings.step = MapStep::Color;
    let mut h = panel(state);
    // „Alle rechnen“ would stop at that scan: locked, with the reason
    assert_eq!(click(&mut h, text::COMPUTE_ALL), None);
    h.get_by_label(&text::cannot_compute(why));
    // the scan alone too; a scan with colour can still be computed alone
    h.state_mut().0.selection = Selection::Scan("scan-1335".into());
    h.run();
    assert_eq!(click(&mut h, text::COMPUTE_ONE), None);
    h.state_mut().0.selection = Selection::Scan("scan-1325".into());
    h.run();
    assert_eq!(
        click(&mut h, text::COMPUTE_ONE),
        Some(Action::ComputeMap("scan-1325".into()))
    );
    // „Alles“ skips the colours with a warning: nothing locked
    h.state_mut().0.map_settings.step = MapStep::All;
    h.state_mut().0.selection = Selection::None;
    h.run();
    assert_eq!(
        click(&mut h, text::COMPUTE_ALL),
        Some(Action::ComputeAllMaps)
    );
}

/// [`map`] with a cleaned cloud: a map that scans can be merged from.
fn clean_map(dir: &str) -> MapInfo {
    MapInfo {
        clouds: vec![PathBuf::from(dir).join("clean.las")],
        ..map(dir)
    }
}

const MAP_1330: &str = "/p/scans/scan-1330/slam-001/karte-001";
const MAP_1335: &str = "/p/scans/scan-1335/slam-001/karte-001";
const MAP_1335_2: &str = "/p/scans/scan-1335/slam-002/karte-001";

fn fresh() -> MapStatus {
    MapStatus {
        seconds: Some(10.0),
        stale: false,
    }
}

/// [`three_scans`] with cleaned maps: „Scan 13:30“ with one run, „Scan 13:35“ with a second
/// run „SLAM 002“ whose map fits (the older one no longer does); „Scan 13:25“ has none.
fn mergeable() -> AppState {
    let mut state = three_scans();
    state.scans[1].runs[0].maps[0] = clean_map(MAP_1330);
    state.scans[2].runs[0].maps[0] = clean_map(MAP_1335);
    let mut second = run("/p/scans/scan-1335/slam-002", vec![clean_map(MAP_1335_2)]);
    second.label = "SLAM 002".into();
    state.scans[2].runs.push(second);
    let stale = MapStatus {
        seconds: Some(10.0),
        stale: true,
    };
    state.set_map_status(
        [
            (MAP_1330.into(), fresh()),
            (MAP_1335.into(), stale),
            (MAP_1335_2.into(), fresh()),
        ]
        .into(),
    );
    state
}

#[test]
fn merge_button_needs_two_maps() {
    // „Scan 13:30“ has a map, the map of „Scan 13:35“ no longer fits: one scan to merge
    let mut state = three_scans();
    state.scans[1].runs[0].maps[0] = clean_map(MAP_1330);
    state.scans[2].runs[0].maps[0] = clean_map(MAP_1335);
    let h = panel(state.clone());
    assert!(h.query_by_label(text::MERGE_OPEN).is_none());
    // a fitting map without cleaned points (only the dense step) does not count either
    let mut dense = state.clone();
    dense.scans[2].runs[0].maps[0] = map(MAP_1335);
    dense.set_map_status([(MAP_1330.into(), fresh()), (MAP_1335.into(), fresh())].into());
    let h = panel(dense);
    assert!(h.query_by_label(text::MERGE_OPEN).is_none());
    // both fit: two scans
    state.set_map_status([(MAP_1330.into(), fresh()), (MAP_1335.into(), fresh())].into());
    let mut h = panel(state);
    assert_eq!(click(&mut h, text::MERGE_OPEN), Some(Action::OpenMerge));
}

#[test]
fn merge_dialog_start_needs_two_ticks() {
    let mut state = mergeable();
    state.open_merge();
    let mut h = panel(state);
    assert!(
        h.query_by_label(text::MERGE_OPEN).is_none(),
        "the field replaces the button"
    );
    let checkbox = |h: &H, name: &str| {
        h.get_by_role_and_label(Role::CheckBox, name)
            .accesskit_node()
            .is_disabled()
    };
    // a scan without a map cannot be ticked; the others are, the first is the reference
    assert!(checkbox(&h, "Scan 13:25"));
    assert!(!checkbox(&h, "Scan 13:30"));
    assert_eq!(h.get_all_by_label(text::MERGE_REFERENCE).count(), 1);
    // the newest map of the newest run is preset
    h.get_by_value("SLAM 002 · Karte 001");
    h.get_by_value("SLAM 001 · Karte 001");
    let start_locked = |h: &H| {
        h.get_by_label(text::MERGE_START)
            .accesskit_node()
            .is_disabled()
    };
    assert!(!start_locked(&h));
    let want = |include_unsure| Action::RunMerge {
        picks: vec![
            MergePick {
                scan_id: "scan-1330".into(),
                run: "/p/scans/scan-1330/slam-001".into(),
                map: MAP_1330.into(),
            },
            MergePick {
                scan_id: "scan-1335".into(),
                run: "/p/scans/scan-1335/slam-002".into(),
                map: MAP_1335_2.into(),
            },
        ],
        include_unsure,
    };
    assert_eq!(click(&mut h, text::MERGE_START), Some(want(false)));
    // one tick: locked
    h.get_by_role_and_label(Role::CheckBox, "Scan 13:30")
        .click();
    h.run();
    assert!(start_locked(&h));
    assert!(h.state().0.merge.picks(&h.state().0).len() == 1);
    // the reference is now the first ticked one
    h.get_by_role_and_label(Role::CheckBox, "Scan 13:30")
        .click();
    h.run();
    h.get_by_label(text::MERGE_UNSURE).click();
    h.run();
    assert_eq!(click(&mut h, text::MERGE_START), Some(want(true)));
    // closed: the button again
    click(&mut h, text::MERGE_CLOSE);
    assert!(!h.state().0.merge.open);
    h.get_by_label(text::MERGE_OPEN);
}

/// At the width of the program's field (250 points), the map choice of a ticked scan takes
/// the width of the field, so that „SLAM 002 · Karte 001“ is not cut.
#[test]
fn merge_dialog_shows_the_whole_map_name() {
    let mut state = mergeable();
    state.open_merge();
    let mut h = Harness::builder()
        .with_size([250.0, 1200.0])
        .build_ui_state(
            |ui, state: &mut AppState| {
                steps::show(ui, state);
            },
            state,
        );
    h.run();
    for name in ["SLAM 001 · Karte 001", "SLAM 002 · Karte 001"] {
        let combo = h.get_by_value(name).rect();
        assert!(combo.width() > 170.0, "{name}: {combo:?}");
        assert!(combo.right() <= 250.0, "{name}: {combo:?}");
    }
}

fn member(scan: &str, verdict: &str, included: bool) -> ModelMember {
    let run = PathBuf::from(format!("/p/scans/{scan}/slam-001"));
    ModelMember {
        scan: PathBuf::from(format!("/daten/{scan}")),
        scan_rel: None,
        slam_dir: run.clone(),
        slam_dir_rel: None,
        map_dir: run.join("karte-001"),
        map_dir_rel: None,
        slam: SlamSettings::default(),
        transform: [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        placed_against: (verdict != "bezug").then_some(0),
        method: if verdict == "bezug" {
            "bezug"
        } else {
            "merkmale"
        }
        .into(),
        verdict: if verdict == "bezug" {
            "sicher"
        } else {
            verdict
        }
        .into(),
        reason: (verdict == "unsicher").then(|| "zwei Lagen passen fast gleich gut".into()),
        overlap: 0.98,
        residual_median_mm: 1.84,
        residual_rms_mm: 2.5,
        agreement: 0.95,
        wall_residual_mm: 1.9,
        pair_rms_mm: None,
        included,
        color_gain: [1.0; 3],
        points: if included { 1000 } else { 0 },
        clean_id: None,
    }
}

fn report(members: Vec<ModelMember>) -> ModelReport {
    ModelReport {
        version: 1,
        input_id: "0".into(),
        points: members.iter().map(|m| m.points).sum(),
        members,
        seconds_total: 12.0,
        peak_rss_bytes: 0,
        include_unsure: false,
        warnings: Vec::new(),
    }
}

const MODEL: &str = "/p/modelle/modell-001";

/// [`three_scans`] with „Modell 001“ of all three: 13:25 the reference, 13:30 sure, 13:35
/// unsure and left out; with a surface.
fn with_model() -> AppState {
    let mut state = three_scans();
    state.models = vec![ModelInfo {
        dir: MODEL.into(),
        label: "Modell 001".into(),
        report: report(vec![
            member("scan-1325", "bezug", true),
            member("scan-1330", "sicher", true),
            member("scan-1335", "unsicher", false),
        ]),
        surfaces: vec![SurfaceInfo {
            dir: format!("{MODEL}/oberflaeche-001").into(),
            label: "Oberfläche 001".into(),
            stale: false,
            settings: None,
        }],
    }];
    state
}

#[test]
fn model_node_shows_verdicts() {
    let mut h = panel(with_model());
    h.get_by_label("Modell 001");
    // per scan its name (from the project), the verdict, overlap and residual
    for name in ["1 · Scan 13:25", "2 · Scan 13:30", "3 · Scan 13:35"] {
        h.get_by_label(name);
    }
    h.get_by_label(text::MERGE_REFERENCE);
    h.get_by_label(text::VERDICT_SURE);
    h.get_by_label(text::VERDICT_UNSURE);
    assert_eq!(
        h.get_all_by_label(&text::member_numbers(0.98, 1.84))
            .count(),
        2
    );
    assert_eq!(
        text::member_numbers(0.98, 1.84),
        "Überlappung 98 % · Restversatz 1,8 mm"
    );
    h.get_by_label(text::NOT_IN_MODEL);
    h.get_by_label(MODEL_LAS);
    h.get_by_label("Oberfläche 001");
    assert_eq!(
        click(&mut h, "Modell 001"),
        Some(Action::Select(Selection::Model(MODEL.into())))
    );
    assert_eq!(
        click(&mut h, MODEL_LAS),
        Some(Action::Select(Selection::Cloud(
            PathBuf::from(MODEL).join(MODEL_LAS)
        )))
    );
    // a failed scan
    let mut state = with_model();
    state.models[0].report.members[2].verdict = "gescheitert".into();
    let h = panel(state);
    h.get_by_label(text::VERDICT_FAILED);
    // a model whose map was computed anew since
    let mut state = with_model();
    state.set_map_status(
        [(
            PathBuf::from(MODEL),
            MapStatus {
                seconds: None,
                stale: true,
            },
        )]
        .into(),
    );
    let h = panel(state);
    h.get_by_label(&text::model_stale("Modell 001"));
}

#[test]
fn selecting_model_loads_it_coloured_by_scan() {
    use driftless_map::{write_las_sources, Cloud};
    let dir = tmp("app-model-select");
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    let model = app
        .state()
        .project
        .as_ref()
        .unwrap()
        .dir
        .join("modelle")
        .join("modell-001");
    std::fs::create_dir_all(&model).unwrap();
    let n = 200;
    let cloud = Cloud {
        xyz: (0..n).map(|i| [i as f32 * 0.01, 0.0, 1.0]).collect(),
        intensity: vec![0.5; n],
        t: vec![0.0; n],
        frame: vec![0; n],
        rgb: None,
        colored: None,
    };
    let sources: Vec<u16> = (0..n).map(|i| (i % 2) as u16).collect();
    write_las_sources(&model.join(MODEL_LAS), &cloud, &sources).unwrap();
    let r = report(vec![
        member("a", "bezug", true),
        member("b", "sicher", true),
    ]);
    std::fs::write(model.join(MODEL_REPORT), serde_json::to_string(&r).unwrap()).unwrap();
    // the tree is read again when the project is opened again
    let project = app.state().project.as_ref().unwrap().dir.clone();
    app.apply(Action::OpenProject(project));
    settle(&mut app);
    let models = &app.state().models;
    assert_eq!(models.len(), 1);
    assert_eq!(models[0].dir, model);
    app.state_mut().view.color_mode = ColorMode::Rgb;
    app.apply(Action::Select(Selection::Model(model.clone())));
    settle(&mut app);
    let cloud = app.state().cloud.as_ref().expect("model loaded");
    assert_eq!(cloud.path, model.join(MODEL_LAS));
    assert!(cloud.has_scans);
    assert_eq!(app.state().view.color_mode, ColorMode::Scan);
    // switched once: another mode chosen afterwards stays
    app.state_mut().view.color_mode = ColorMode::Intensity;
    settle(&mut app);
    assert_eq!(app.state().view.color_mode, ColorMode::Intensity);
}
