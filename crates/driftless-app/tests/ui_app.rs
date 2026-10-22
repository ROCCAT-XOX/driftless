//! The program's actions without a window: opening projects, adding scans, jobs, switching
//! projects while a step runs.

use driftless_app::jobs::JobOutput;
use driftless_app::project::{canonical, Project};
use driftless_app::ui::{parse_args, Action, Args, DriftlessApp, Selection, UiMessage};
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

/// A job that runs until it is cancelled and then holds until `release` is sent: the
/// state in between can be checked without a race.
fn held_job(app: &mut DriftlessApp, label: &str) -> std::sync::mpsc::Sender<()> {
    let (started_tx, started) = channel::<()>();
    let (release_tx, release) = channel::<()>();
    app.start_job(label.to_string(), move |p| {
        started_tx.send(()).unwrap();
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(1));
        }
        // the last unit of work: the switch must wait for it
        let _ = release.recv();
        Err(StepError::Cancelled)
    })
    .unwrap();
    started.recv().unwrap();
    release_tx
}

/// Two empty projects `A` and `B`, `A` open.
fn two_projects(name: &str) -> (DriftlessApp, PathBuf, PathBuf) {
    let dir = tmp(name);
    let (a, b) = (dir.join("a"), dir.join("b"));
    Project::create(&a, "A").unwrap();
    Project::create(&b, "B").unwrap();
    let mut app = app();
    app.apply(Action::OpenProject(a.clone()));
    assert_eq!(app.state().project.as_ref().unwrap().name, "A");
    (app, a, b)
}

#[test]
fn switching_project_cancels_job() {
    let (mut app, _, b) = two_projects("ui-wechsel");
    let release = held_job(&mut app, "läuft in A");

    app.apply(Action::OpenProject(b.clone()));
    app.tick();
    // the job is asked to stop; until it has, the old project stays (it may still write)
    let job = app.state().job.clone().expect("job still running");
    assert!(job.cancelling);
    assert_eq!(app.state().project.as_ref().unwrap().name, "A");
    // a new step cannot start in between
    assert!(app
        .start_job("dazwischen", |_| Ok(JobOutput::Done))
        .is_err());

    release.send(()).unwrap();
    tick_until(&mut app, |a| a.state().job.is_none());
    let p = app.state().project.as_ref().unwrap();
    assert_eq!(p.name, "B");
    assert_eq!(p.dir, canonical(&b).unwrap());
    assert_eq!(errors(&app), Vec::<String>::new());
}

#[test]
fn reopening_open_project_keeps_job() {
    let (mut app, a, _) = two_projects("ui-wieder");
    let release = held_job(&mut app, "läuft in A");
    // the same folder, written differently
    app.apply(Action::OpenProject(a.join(".")));
    app.tick();
    assert!(!app.state().job.as_ref().unwrap().cancelling);
    assert_eq!(app.state().project.as_ref().unwrap().name, "A");
    app.apply(Action::Cancel);
    release.send(()).unwrap();
    tick_until(&mut app, |a| a.state().job.is_none());
    assert_eq!(app.state().project.as_ref().unwrap().name, "A");
}

#[test]
fn closing_while_busy_waits_for_cancel() {
    let (mut app, _, _) = two_projects("ui-schliessen");
    let release = held_job(&mut app, "läuft");
    assert!(!app.close_requested(), "close must be put off");
    app.tick();
    assert!(app.state().job.as_ref().unwrap().cancelling);
    assert!(!app.take_close());
    // asking again changes nothing
    assert!(!app.close_requested());
    release.send(()).unwrap();
    tick_until(&mut app, |a| a.state().job.is_none());
    assert!(app.take_close());
    assert!(!app.take_close(), "only once");
    // without a job the window closes at once
    assert!(app.close_requested());
}

#[test]
fn window_close_is_cancelled_then_sent() {
    use eframe::egui::{ViewportCommand, ViewportEvent, ViewportId};
    use egui_kittest::Harness;

    let (mut app, _, _) = two_projects("ui-fenster");
    let release = held_job(&mut app, "läuft");
    let mut h = Harness::builder()
        .with_size([1000.0, 700.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    let commands = |h: &Harness<'_, DriftlessApp>| {
        h.output().viewport_output[&ViewportId::ROOT]
            .commands
            .clone()
    };
    h.input_mut()
        .viewports
        .entry(ViewportId::ROOT)
        .or_default()
        .events
        .push(ViewportEvent::Close);
    h.step();
    assert!(commands(&h).contains(&ViewportCommand::CancelClose));
    assert!(!commands(&h).contains(&ViewportCommand::Close));
    assert!(h.state().state().job.as_ref().unwrap().cancelling);

    release.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        h.step();
        if commands(&h).contains(&ViewportCommand::Close) {
            break;
        }
        assert!(Instant::now() < deadline, "window was not closed");
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(h.state().state().job.is_none());
}

#[test]
fn gone_last_project_is_reported_once() {
    let dir = tmp("ui-letztes");
    let mut app = app();
    app.restore_last(Some(dir.join("weg").to_str().unwrap()));
    let e = errors(&app);
    assert_eq!(e.len(), 1);
    assert!(
        e[0].starts_with("Zuletzt geöffnetes Projekt nicht lesbar"),
        "{e:?}"
    );
    // forgotten on the next save
    assert_eq!(app.last_to_store(), Some(String::new()));

    // next start: the empty value is no project, no message
    let mut app = self::app();
    app.restore_last(Some(""));
    assert!(app.state().messages.is_empty());
    assert_eq!(app.last_to_store(), None);

    // an open project is stored
    let p = dir.join("p");
    Project::create(&p, "P").unwrap();
    app.restore_last(Some(p.to_str().unwrap()));
    let stored = app.last_to_store().unwrap();
    assert_eq!(PathBuf::from(stored), canonical(&p).unwrap());
}

#[test]
fn messages_are_capped() {
    let mut app = app();
    for _ in 0..250 {
        app.apply(Action::Inspect("x".into()));
    }
    assert_eq!(app.state().messages.len(), 100);
}

#[test]
fn args_take_one_project_folder() {
    let args = |v: &[&str]| parse_args(v.iter().map(|s| s.to_string()));
    let project = |p: &str| Args {
        project: Some(PathBuf::from(p)),
        synthetic: None,
    };
    assert_eq!(args(&[]), Ok(Args::default()));
    assert_eq!(args(&["/p"]), Ok(project("/p")));
    assert!(args(&["/p", "/q"]).is_err());
    assert!(args(&["--other"]).is_err());
}

#[test]
fn args_take_synthetic_count() {
    let args = |v: &[&str]| parse_args(v.iter().map(|s| s.to_string()));
    let synthetic = |n| Args {
        project: None,
        synthetic: Some(n),
    };
    assert_eq!(args(&["--synthetic", "5"]), Ok(synthetic(5)));
    assert_eq!(
        args(&["--synthetic", "50_000_000"]),
        Ok(synthetic(50_000_000))
    );
    assert_eq!(
        args(&["/p", "--synthetic", "1000"]),
        Ok(Args {
            project: Some(PathBuf::from("/p")),
            synthetic: Some(1000),
        })
    );
    for bad in [
        &["--synthetic"][..],
        &["--synthetic", "0"],
        &["--synthetic", "viele"],
        &["--synthetic", "5.5"],
        &["--synthetic", "50.000.000"],
        &["--synthetic", "-5"],
        &["--synthetic", "_5"],
        &["--synthetic", "+5"],
    ] {
        assert!(args(bad).is_err(), "{bad:?}");
    }
}

/// `--synthetic` generates the cloud in the background; the window stays usable and the
/// cloud appears when ready, with its levels of detail.
#[test]
fn synthetic_cloud_loads_in_background() {
    let mut app = app();
    app.load_synthetic(200_000);
    assert!(app.is_loading());
    assert!(app.cloud().is_none());
    tick_until(&mut app, |a| a.cloud().is_some());
    let cloud = app.cloud().unwrap();
    assert_eq!(cloud.points.len(), 200_000);
    assert_eq!(cloud.tree.vertices.len(), 200_000);
    let summary = app.state().cloud.clone().unwrap();
    assert_eq!(summary.points, 200_000);
    assert!(summary.has_rgb);
    assert!(app.state().loading.is_none());
    assert_eq!(errors(&app), Vec::<String>::new());
}

/// A cloud selected in the tree is loaded with the quality of its run; a second selection
/// while the first loads replaces it.
#[test]
fn selecting_a_cloud_loads_it() {
    let dir = tmp("ui-wolke");
    let write = |name: &str, n: usize| {
        let path = dir.join(name);
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(1).unwrap();
        let mut w = las::Writer::from_path(&path, builder.into_header().unwrap()).unwrap();
        for i in 0..n {
            w.write_point(las::Point {
                x: i as f64 * 0.01,
                y: 1.0,
                z: 2.0,
                intensity: 1000,
                gps_time: Some(i as f64),
                ..Default::default()
            })
            .unwrap();
        }
        w.close().unwrap();
        path
    };
    let (a, b) = (write("a.las", 300_000), write("b.las", 1_000));
    let mut app = app();
    app.apply(Action::Select(Selection::Cloud(a)));
    assert!(app.is_loading());
    app.apply(Action::Select(Selection::Cloud(b.clone())));
    tick_until(&mut app, |a| !a.is_loading());
    assert_eq!(errors(&app), Vec::<String>::new());
    let cloud = app.cloud().unwrap();
    assert_eq!(cloud.path, b);
    assert_eq!(cloud.points.len(), 1_000);
    // selecting the shown cloud again does not load it again
    app.apply(Action::Select(Selection::Cloud(b)));
    assert!(!app.is_loading());
}

/// A, then C (cancels A), then A again: A is loaded after all, not left cancelled.
#[test]
fn reselecting_a_cancelled_cloud_loads_it() {
    let dir = tmp("ui-wolke-zurueck");
    let write = |name: &str, n: usize| {
        let path = dir.join(name);
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(1).unwrap();
        let mut w = las::Writer::from_path(&path, builder.into_header().unwrap()).unwrap();
        for i in 0..n {
            w.write_point(las::Point {
                x: i as f64 * 0.01,
                y: 1.0,
                z: 2.0,
                gps_time: Some(i as f64),
                ..Default::default()
            })
            .unwrap();
        }
        w.close().unwrap();
        path
    };
    let (a, c) = (write("a.las", 2_000_000), write("c.las", 1_000));
    let mut app = app();
    app.apply(Action::Select(Selection::Cloud(a.clone())));
    app.apply(Action::Select(Selection::Cloud(c)));
    app.apply(Action::Select(Selection::Cloud(a.clone())));
    tick_until(&mut app, |app| !app.is_loading());
    assert_eq!(errors(&app), Vec::<String>::new());
    assert_eq!(app.cloud().map(|c| c.path.clone()), Some(a));
}
