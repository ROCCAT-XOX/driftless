//! The program's actions without a window: opening projects, adding scans, jobs, switching
//! projects while a step runs.

use driftless_app::jobs::JobOutput;
use driftless_app::project::{canonical, Project};
use driftless_app::ui::{parse_args, Action, DriftlessApp, Selection, UiMessage};
use driftless_io::progress::LogLine;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::StepError;
use std::fs;
use std::path::PathBuf;
use std::sync::mpsc::channel;
use std::sync::Arc;
use std::time::{Duration, Instant};

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

/// Ticks the app like frames do until `cond` holds.
fn tick_until(app: &mut DriftlessApp, mut cond: impl FnMut(&DriftlessApp) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        app.tick();
        if cond(app) {
            return;
        }
        assert!(Instant::now() < deadline, "condition not reached");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            UiMessage::Info(_) => None,
        })
        .collect()
}

#[test]
fn new_project_add_scan_shows_tree() {
    let dir = tmp("ui-neu");
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 2.0);
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    assert_eq!(errors(&app), Vec::<String>::new());
    app.apply(Action::AddScan(scan.clone()));
    assert_eq!(errors(&app), Vec::<String>::new());
    let s = app.state();
    assert_eq!(s.project.as_ref().unwrap().name, "Halle");
    assert_eq!(s.scans.len(), 1);
    assert_eq!(s.scans[0].id, "halle");
    assert_eq!(s.selection, Selection::Scan("halle".into()));
    // written to disk
    let back = Project::open(&dir.join("projekt")).unwrap();
    assert_eq!(back.scans.len(), 1);

    // the same scan again: German error, project unchanged
    app.apply(Action::AddScan(scan));
    assert_eq!(errors(&app).len(), 1, "{:?}", errors(&app));
    assert!(errors(&app)[0].contains("schon im Projekt"));
    assert_eq!(app.state().scans.len(), 1);
}

#[test]
fn open_missing_project_reports_error() {
    let dir = tmp("ui-kein-projekt");
    let mut app = app();
    app.apply(Action::OpenProject(dir));
    assert!(app.state().project.is_none());
    let e = errors(&app);
    assert_eq!(e.len(), 1);
    assert!(e[0].starts_with("Kein Projekt"), "{e:?}");
    app.apply(Action::DismissMessages);
    assert!(app.state().messages.is_empty());
}

#[test]
fn job_progress_log_and_result_reach_state() {
    let mut app = app();
    let (tx, rx) = channel::<()>();
    let (reached_tx, reached) = channel::<()>();
    app.start_job("zählen", move |p| {
        p.phase("Zählen", Some(10));
        p.advance(3);
        p.warn("Achtung");
        reached_tx.send(()).unwrap();
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    reached.recv().unwrap();
    app.tick();
    let job = app.state().job.clone().unwrap();
    assert_eq!((job.progress.done, job.progress.total), (3, Some(10)));
    assert_eq!(app.state().log, vec![LogLine::Warning("Achtung".into())]);
    assert!(app.wants_repaint_while_busy());
    tx.send(()).unwrap();
    tick_until(&mut app, |a| a.state().job.is_none());
    assert!(!app.wants_repaint_while_busy());
    assert!(matches!(
        app.state().messages.last(),
        Some(UiMessage::Info(m)) if m.starts_with("zählen fertig")
    ));
}

#[test]
fn cancel_action_reaches_job() {
    let mut app = app();
    app.start_job("warten", |p| {
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(StepError::Cancelled)
    })
    .unwrap();
    app.apply(Action::Cancel);
    app.tick();
    assert!(app.state().job.as_ref().is_none_or(|j| j.cancelling));
    tick_until(&mut app, |a| a.state().job.is_none());
    assert!(matches!(
        app.state().messages.last(),
        Some(UiMessage::Info(m)) if m == "warten abgebrochen"
    ));
}

#[test]
fn switching_project_cancels_job() {
    let dir = tmp("ui-wechsel");
    let (a, b) = (dir.join("a"), dir.join("b"));
    Project::create(&a, "A").unwrap();
    Project::create(&b, "B").unwrap();
    let mut app = app();
    app.apply(Action::OpenProject(a));
    assert_eq!(app.state().project.as_ref().unwrap().name, "A");

    let (started_tx, started) = channel::<()>();
    app.start_job("läuft in A", move |p| {
        started_tx.send(()).unwrap();
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(1));
        }
        // a last unit of work: the switch must wait for it
        std::thread::sleep(Duration::from_millis(50));
        Err(StepError::Cancelled)
    })
    .unwrap();
    started.recv().unwrap();

    app.apply(Action::OpenProject(b.clone()));
    app.tick();
    // the job is asked to stop; until it has, the old project stays (it may still write)
    if let Some(job) = &app.state().job {
        assert!(job.cancelling);
        assert_eq!(app.state().project.as_ref().unwrap().name, "A");
    }
    // a new step cannot start in between
    assert!(app
        .start_job("dazwischen", |_| Ok(JobOutput::Done))
        .is_err());

    tick_until(&mut app, |a| a.state().job.is_none());
    let p = app.state().project.as_ref().unwrap();
    assert_eq!(p.name, "B");
    assert_eq!(p.dir, canonical(&b).unwrap());
    assert_eq!(errors(&app), Vec::<String>::new());
}

#[test]
fn args_take_one_project_folder() {
    let args = |v: &[&str]| parse_args(v.iter().map(|s| s.to_string()));
    assert_eq!(args(&[]), Ok(None));
    assert_eq!(args(&["/p"]), Ok(Some(PathBuf::from("/p"))));
    assert!(args(&["/p", "/q"]).is_err());
    assert!(args(&["--synthetic", "5"]).is_err());
}
