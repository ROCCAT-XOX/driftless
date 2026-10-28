//! The steps started from the right panel, without a window, on a synthetic scan: new run
//! folders, reuse with history, map folders per set of switches, refusals.

use driftless_app::jobs::{JobOutput, JobRunner};
use driftless_app::project::{canonical, Project};
use driftless_app::ui::actions::start_step;
use driftless_app::ui::{text, Action, AppState, DriftlessApp, Selection, UiMessage};
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::history::{self, HistoryKind};
use driftless_steps::map::MapStep;
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::mpsc::channel;
use std::sync::Arc;
use std::time::{Duration, Instant};

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

/// An app with the project `dir/projekt` and its scan `halle` (1.5 s, static) selected.
fn app_with_scan(dir: &Path) -> DriftlessApp {
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.5);
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    app.apply(Action::AddScan(scan));
    assert_eq!(errors(&app), Vec::<String>::new());
    // the tree is read in the background
    settle(&mut app);
    app
}

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

/// Applies `action` and ticks like frames do until the step has ended; fails on an error.
fn run_step(app: &mut DriftlessApp, action: Action) {
    app.apply(action.clone());
    assert_eq!(errors(app), Vec::<String>::new(), "{action:?}");
    let deadline = Instant::now() + Duration::from_secs(120);
    while app.state().job.is_some() || app.wants_repaint_while_busy() {
        assert!(Instant::now() < deadline, "step did not end");
        std::thread::sleep(Duration::from_millis(5));
        app.tick();
    }
    assert_eq!(errors(app), Vec::<String>::new(), "{action:?}");
    assert!(app.state().running.is_none());
}

/// Ticks until no step and no background work is left.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        app.tick();
        if !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "background work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn runs_dir(dir: &Path) -> PathBuf {
    canonical(&dir.join("projekt")).unwrap().join("scans/halle")
}

/// Files directly in `dir` (name → contents).
fn files(dir: &Path) -> BTreeMap<String, Vec<u8>> {
    fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.is_file())
        .map(|p| {
            let name = p.file_name().unwrap().to_string_lossy().into_owned();
            (name, fs::read(&p).unwrap())
        })
        .collect()
}

fn names(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    v.sort();
    v
}

const RUN_FILES: [&str; 7] = [
    "lauf.toml",
    "map.las",
    "slam.json",
    "submaps.bin",
    "traj.tum",
    "traj_odom.tum",
    "verlauf",
];

#[test]
fn slam_action_creates_new_run_dir() {
    let dir = tmp("aktion-slam");
    let mut app = app_with_scan(&dir);
    run_step(&mut app, Action::RunSlam("halle".into()));
    let first = runs_dir(&dir).join("slam-001");
    assert_eq!(app.state().selection, Selection::Run(first.clone()));
    let before = files(&first);
    run_step(&mut app, Action::RunSlam("halle".into()));
    let second = runs_dir(&dir).join("slam-002");
    assert_eq!(app.state().selection, Selection::Run(second.clone()));
    assert_eq!(names(&runs_dir(&dir)), ["slam-001", "slam-002"]);
    for run in [&first, &second] {
        assert_eq!(names(run), RUN_FILES, "{}", run.display());
        let h = history::list(run);
        assert_eq!(h.len(), 1);
        assert_eq!(h[0].kind, HistoryKind::Full);
    }
    assert_eq!(files(&first), before, "the first run is unchanged");
    // both runs in the tree, with their runtime
    let runs = &app.state().scans[0].runs;
    assert_eq!(runs.len(), 2);
    assert!(runs.iter().all(|r| r.has_result && r.has_cache));
    // the job ended with a message „SLAM rechnen fertig (…)“
    assert!(app.state().messages.iter().any(|m| matches!(
        m,
        UiMessage::Info(t) if t.starts_with(&format!("{} {}", text::STEP_SLAM, text::JOB_DONE))
    )));
}

#[test]
fn reuse_action_records_history() {
    let dir = tmp("aktion-reuse");
    let mut app = app_with_scan(&dir);
    run_step(&mut app, Action::RunSlam("halle".into()));
    let run = runs_dir(&dir).join("slam-001");
    let traj = fs::read(run.join("traj.tum")).unwrap();
    run_step(&mut app, Action::Reuse(run.clone()));
    assert_eq!(app.state().selection, Selection::Run(run.clone()));
    let kinds: Vec<HistoryKind> = history::list(&run).iter().map(|e| e.kind).collect();
    assert_eq!(kinds, [HistoryKind::Full, HistoryKind::Reuse]);
    assert_eq!(fs::read(run.join("traj.vorher.tum")).unwrap(), traj);
    let info = &app.state().scans[0].runs[0];
    assert!(info.has_previous);
    assert_eq!(info.history.len(), 2);

    // without submaps.bin: refused with the reason, nothing started
    fs::remove_file(run.join("submaps.bin")).unwrap();
    app.apply(Action::OpenProject(dir.join("projekt")));
    settle(&mut app);
    app.apply(Action::Reuse(run.clone()));
    assert_eq!(errors(&app), [text::NO_CACHE]);
    assert!(app.state().job.is_none());
}

#[test]
fn map_action_with_new_settings_creates_new_map_dir() {
    let dir = tmp("aktion-karte");
    let mut app = app_with_scan(&dir);
    run_step(&mut app, Action::RunSlam("halle".into()));
    let run = runs_dir(&dir).join("slam-001");
    // after a run its cloud is shown, after a map the map's best cloud
    let shown = |app: &DriftlessApp| app.cloud().map(|c| c.path.clone());
    assert_eq!(shown(&app), Some(run.join("map.las")));
    app.state_mut().map_settings.step = MapStep::Dense;
    run_step(&mut app, Action::RunMap(run.clone()));
    let first = run.join("karte-001");
    assert_eq!(app.state().selection, Selection::Map(first.clone()));
    assert_eq!(shown(&app), Some(first.join("dense.las")));
    assert!(first.join("dense.las").is_file());
    assert!(first.join("map.json").is_file());
    // the same settings again: the same folder
    run_step(&mut app, Action::RunMap(run.clone()));
    assert_eq!(app.state().selection, Selection::Map(first.clone()));
    assert!(!run.join("karte-002").exists());
    // other switches: a new folder, the first one unchanged
    let before = files(&first);
    app.state_mut().map_settings.switches.voxel = 0.1;
    run_step(&mut app, Action::RunMap(run.clone()));
    let second = run.join("karte-002");
    assert_eq!(app.state().selection, Selection::Map(second.clone()));
    assert!(second.join("dense.las").is_file());
    assert_eq!(files(&first), before);
    let status = &app.state().map_status;
    assert!(!status[&first].stale && !status[&second].stale);
    assert!(status[&second].seconds.is_some());

    // a changed trajectory makes both maps stale
    let mut traj = fs::read(run.join("traj.tum")).unwrap();
    traj.extend_from_slice(b"\n");
    fs::write(run.join("traj.tum"), traj).unwrap();
    app.apply(Action::OpenProject(dir.join("projekt")));
    // the reading again leaves the statuses to the background: the old ones stay until then
    assert!(!app.state().map_status[&first].stale);
    settle(&mut app);
    let status = &app.state().map_status;
    assert!(status[&first].stale && status[&second].stale);

    // an invalid voxel size: refused with the CLI message, no folder made
    app.state_mut().map_settings.switches.voxel = 0.0;
    app.apply(Action::RunMap(run.clone()));
    assert_eq!(errors(&app), ["--voxel: Voxelgröße muss größer als 0 sein"]);
    assert!(app.state().job.is_none());
    assert!(!run.join("karte-003").exists());
}

#[test]
fn steps_refused_while_a_step_runs() {
    let dir = tmp("aktion-belegt");
    let mut app = app_with_scan(&dir);
    let (tx, rx) = channel::<()>();
    app.start_job("warten", move |_| {
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    app.apply(Action::RunSlam("halle".into()));
    assert_eq!(errors(&app), [text::busy("warten")]);
    assert!(!runs_dir(&dir).exists(), "no run folder while busy");
    tx.send(()).unwrap();
}

#[test]
fn start_step_checks_without_a_window() {
    let dir = tmp("aktion-direkt");
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.5);
    let mut project = Project::create(&dir.join("projekt"), "P").unwrap();
    project.add_scan(&scan).unwrap();
    project.save().unwrap();
    let mut state = AppState::default();
    state.set_project(project);
    let mut runner = JobRunner::new(Arc::new(|| {}));

    // unknown scan and run, a non-step action
    let e = start_step(&mut state, &mut runner, &Action::Inspect("x".into())).unwrap_err();
    assert!(e.contains(text::UNKNOWN_SCAN), "{e}");
    let e = start_step(
        &mut state,
        &mut runner,
        &Action::RunMap("/kein/lauf".into()),
    );
    assert!(e.unwrap_err().contains(text::UNKNOWN_RUN));
    assert!(start_step(&mut state, &mut runner, &Action::Cancel).is_err());
    assert!(!runner.is_busy());

    // inspect: the step is known while it runs
    let inspect = Action::Inspect("halle".into());
    start_step(&mut state, &mut runner, &inspect).unwrap();
    assert_eq!(state.running, Some(inspect));
    let deadline = Instant::now() + Duration::from_secs(60);
    let finished = loop {
        if let Some(f) = runner.poll() {
            break f;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(5));
    };
    assert_eq!(finished.label, text::STEP_INSPECT);
    assert!(matches!(finished.result, Ok(JobOutput::Inspect(_))));

    // a missing scan folder: refused with its reason
    fs::rename(&scan, dir.join("weg")).unwrap();
    state.refresh();
    let e = start_step(&mut state, &mut runner, &Action::RunSlam("halle".into()));
    assert!(e.unwrap_err().contains("nicht gefunden"));
    assert!(!runner.is_busy());
}

#[test]
fn inspect_result_reaches_state() {
    let dir = tmp("aktion-pruefen");
    let mut app = app_with_scan(&dir);
    run_step(&mut app, Action::Inspect("halle".into()));
    let result = app.state().inspect.clone().expect("inspected");
    assert_eq!(result.scan, "halle");
    assert!(!result.report.topics.is_empty());
}

#[test]
fn refresh_leaves_map_status_to_the_background() {
    let dir = tmp("aktion-status");
    let mut app = app_with_scan(&dir);
    run_step(&mut app, Action::RunSlam("halle".into()));
    let run = runs_dir(&dir).join("slam-001");
    app.state_mut().map_settings.step = MapStep::Dense;
    run_step(&mut app, Action::RunMap(run.clone()));
    let map = run.join("karte-001");
    assert!(app.state().map_status.contains_key(&map));

    // a fresh state: refresh reads the tree only, no map status
    let mut state = AppState::default();
    state.set_project(Project::open(&dir.join("projekt")).unwrap());
    assert_eq!(state.scans[0].runs[0].maps.len(), 1);
    assert!(state.map_status.is_empty());
    let version = state.tree_version;
    state.refresh();
    assert_eq!(state.tree_version, version + 1);
    // the job computes it
    let status = state.map_status_job()();
    assert!(!status[&map].stale);
    assert!(status[&map].seconds.is_some());
}

#[test]
fn scan_without_images_cannot_be_coloured() {
    let dir = tmp("aktion-farbe");
    let mut app = app_with_scan(&dir);
    let why = app.state().scans[0].no_color.clone().expect("no images");
    assert!(why.starts_with("keine Kamerabilder"), "{why}");
    run_step(&mut app, Action::RunSlam("halle".into()));
    let run = runs_dir(&dir).join("slam-001");
    app.state_mut().map_settings.step = MapStep::Color;
    app.apply(Action::RunMap(run.clone()));
    assert_eq!(errors(&app), [why]);
    assert!(app.state().job.is_none());
    assert!(!run.join("karte-001").exists());
}

/// `dir/lauf/traj.tum` with two poses, and the request to export it as CSV to `dir/<name>`.
fn traj_export(dir: &Path, name: &str) -> driftless_steps::export::ExportRequest {
    use driftless_steps::export::{ExportFormat, ExportRequest, ExportSource};
    let run = dir.join("lauf");
    fs::create_dir_all(&run).unwrap();
    fs::write(
        run.join("traj.tum"),
        "0.0 0 0 0 0 0 0 1\n1.0 1 2 3 0 0 0 1\n",
    )
    .unwrap();
    ExportRequest {
        source: ExportSource::Run(run),
        out: dir.join(name),
        format: ExportFormat::Csv,
        overwrite: false,
    }
}

#[test]
fn run_export_without_project_starts() {
    let dir = tmp("aktion-export-ohne-projekt");
    let req = traj_export(&dir, "Büro Lauf.csv");
    let mut state = AppState::default();
    let mut runner = JobRunner::new(Arc::new(|| {}));
    let action = Action::RunExport(req.clone());
    start_step(&mut state, &mut runner, &action).unwrap();
    assert_eq!(state.running, Some(action.clone()));
    assert_eq!(state.job.as_ref().unwrap().label, text::EXPORT_TITLE);
    let deadline = Instant::now() + Duration::from_secs(30);
    let finished = loop {
        if let Some(f) = runner.poll() {
            break f;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(5));
    };
    assert!(matches!(finished.result, Ok(JobOutput::Exported(_))));

    // the program: the message names the file, nothing else changes
    let mut app = app();
    let req = traj_export(&dir, "zweite.csv");
    run_step(&mut app, Action::RunExport(req.clone()));
    assert!(req.out.is_file());
    assert!(app.state().project.is_none());
    let last = app.state().messages.last().cloned();
    assert!(
        matches!(&last, Some(UiMessage::Info(m)) if m.starts_with("Exportiert: zweite.csv (2 Lagen, ")),
        "{last:?}"
    );
    // a second time without „Ersetzen“: refused by the step, the file stays
    let before = fs::read(&req.out).unwrap();
    app.apply(Action::RunExport(req.clone()));
    settle(&mut app);
    let e = errors(&app);
    assert_eq!(e.len(), 1, "{e:?}");
    assert!(e[0].contains("gibt es schon"), "{e:?}");
    assert_eq!(fs::read(&req.out).unwrap(), before);
}

#[test]
fn run_export_while_busy_refused() {
    let dir = tmp("aktion-export-belegt");
    let req = traj_export(&dir, "x.csv");
    let mut app = app();
    let (tx, rx) = channel::<()>();
    app.start_job("warten", move |_| {
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    app.apply(Action::OpenExport(req.source.clone()));
    app.apply(Action::RunExport(req.clone()));
    assert_eq!(errors(&app), [text::busy("warten")]);
    assert!(!req.out.exists());
    // the window stays, to try again later
    assert!(app.state().export.is_some());
    tx.send(()).unwrap();
}
