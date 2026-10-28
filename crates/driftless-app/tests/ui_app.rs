//! The program's actions without a window: opening projects, adding scans, jobs, switching
//! projects while a step runs.

use driftless_app::jobs::JobOutput;
use driftless_app::project::{canonical, Project};
use driftless_app::ui::{
    parse_args, text, Action, Args, DriftlessApp, LogKind, Selection, UiMessage,
};
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_steps::StepError;
use egui_kittest::kittest::Queryable;
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

/// Ticks until no step and no background work is left (the tree is read in the background).
fn settle(app: &mut DriftlessApp) {
    tick_until(app, |a| !a.wants_repaint_while_busy());
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
    // saved at once; the tree is read in the background
    assert_eq!(app.state().project.as_ref().unwrap().scans.len(), 1);
    assert!(app.state().tree_pending);
    settle(&mut app);
    assert!(!app.state().tree_pending);
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
    app.apply(Action::OpenProject(dir.clone()));
    // read in the background: the field says so meanwhile
    assert_eq!(app.state().opening, Some(dir));
    settle(&mut app);
    assert_eq!(app.state().opening, None);
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
    let log = &app.state().log;
    assert_eq!(log.len(), 1, "{log:?}");
    assert_eq!(
        (log[0].kind, log[0].text.as_str()),
        (LogKind::Warning, "Achtung")
    );
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
    settle(&mut app);
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
    // then B is read and opened
    settle(&mut app);
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
    settle(&mut app);
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
    settle(&mut app);
    let stored = app.last_to_store().unwrap();
    assert_eq!(PathBuf::from(stored), canonical(&p).unwrap());
}

#[test]
fn messages_are_capped() {
    let mut app = app();
    // 250 different messages („Scan nicht im Projekt: x7“)
    for i in 0..250 {
        app.apply(Action::ComputeMap(format!("x{i}")));
    }
    assert_eq!(app.state().messages.len(), 100);
    // the same message again and again stands once
    app.apply(Action::DismissMessages);
    for _ in 0..5 {
        app.apply(Action::Inspect("x".into()));
    }
    assert_eq!(app.state().messages.len(), 1);
}

#[test]
fn args_take_one_project_folder() {
    let args = |v: &[&str]| parse_args(v.iter().map(|s| s.to_string()));
    let project = |p: &str| Args {
        project: Some(PathBuf::from(p)),
        ..Args::default()
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
        synthetic: Some(n),
        ..Args::default()
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
            ..Args::default()
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

#[test]
fn parse_args_version() {
    let args = |v: &[&str]| parse_args(v.iter().map(|s| s.to_string()));
    let version = Args {
        version: true,
        ..Args::default()
    };
    assert_eq!(args(&["--version"]), Ok(version));
    assert!(!args(&[]).unwrap().version);
    assert!(!args(&["/p"]).unwrap().version);
    // not a project folder
    assert_eq!(args(&["--version"]).unwrap().project, None);
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

/// A file dropped onto the window, as the windowing backend hands it over.
#[derive(Debug)]
struct Dropped(PathBuf);

impl eframe::egui::DroppedFile for Dropped {
    fn path(&self) -> &std::path::Path {
        &self.0
    }

    fn bytes(&self) -> Result<Vec<u8>, String> {
        fs::read(&self.0).map_err(|e| e.to_string())
    }
}

fn window(app: DriftlessApp) -> egui_kittest::Harness<'static, DriftlessApp> {
    egui_kittest::Harness::builder()
        .with_size([1000.0, 700.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app)
}

fn drop_files(h: &mut egui_kittest::Harness<'_, DriftlessApp>, paths: &[PathBuf]) {
    h.input_mut().dropped_files = paths
        .iter()
        .map(|p| Arc::new(Dropped(p.clone())) as eframe::egui::DroppedFileHandle)
        .collect();
    h.step();
}

/// A LAS or PLY dropped onto the window is shown, without a project; of several files the
/// first one that can be opened.
#[test]
fn dropped_las_loads_cloud() {
    let dir = tmp("ui-hineinziehen");
    let las = dir.join("wolke.las");
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(1).unwrap();
    let mut w = las::Writer::from_path(&las, builder.into_header().unwrap()).unwrap();
    for i in 0..500 {
        w.write_point(las::Point {
            x: f64::from(i) * 0.01,
            y: 1.0,
            z: 2.0,
            gps_time: Some(f64::from(i)),
            ..Default::default()
        })
        .unwrap();
    }
    w.close().unwrap();
    let mut h = window(app());
    drop_files(&mut h, &[dir.join("notiz.txt"), las.clone()]);
    assert!(h.state().is_loading());
    let deadline = Instant::now() + Duration::from_secs(30);
    while h.state().cloud().is_none() {
        assert!(Instant::now() < deadline, "cloud not loaded");
        std::thread::sleep(Duration::from_millis(2));
        h.step();
    }
    assert_eq!(h.state().cloud().unwrap().path, las);
    assert_eq!(h.state().cloud().unwrap().points.len(), 500);
    assert!(h.state().state().project.is_none());
    assert_eq!(errors(h.state()), Vec::<String>::new());

    // a PLY the same way
    let ply = dir.join("modell.PLY");
    fs::write(
        &ply,
        "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n\
         property float z\nend_header\n0 0 0\n1 1 1\n",
    )
    .unwrap();
    drop_files(&mut h, std::slice::from_ref(&ply));
    let deadline = Instant::now() + Duration::from_secs(30);
    while h.state().cloud().is_none_or(|c| c.path != ply) {
        assert!(Instant::now() < deadline, "PLY not loaded");
        std::thread::sleep(Duration::from_millis(2));
        h.step();
    }
    assert_eq!(h.state().cloud().unwrap().points.len(), 2);
    assert_eq!(errors(h.state()), Vec::<String>::new());

    // a surface starts its load
    drop_files(&mut h, &[dir.join("oberflaeche.glb")]);
    assert!(h.state().is_loading_surface());
}

#[test]
fn dropped_unknown_shows_hint() {
    let dir = tmp("ui-hineinziehen-fremd");
    let mut h = window(app());
    // while a file hovers over the window
    h.input_mut().hovered_files = vec![eframe::egui::HoveredFile {
        path: Some(dir.join("notiz.txt")),
        mime: String::new(),
    }];
    h.step();
    h.step();
    h.get_by_label(text::DROP_HERE);
    h.input_mut().hovered_files.clear();
    drop_files(&mut h, &[dir.join("notiz.txt"), dir.join("bild.jpg")]);
    h.step();
    assert!(h.query_by_label(text::DROP_HERE).is_none());
    assert!(!h.state().is_loading());
    assert!(!h.state().is_loading_surface());
    let shown: Vec<_> = h.state().state().messages.clone();
    let hint = text::drop_unsupported("notiz.txt");
    assert_eq!(
        hint,
        "Dateiformat nicht unterstützt: notiz.txt (LAS, LAZ, E57, PLY, PCD, GLB oder OBJ)"
    );
    assert!(
        shown
            .iter()
            .any(|m| matches!(m, UiMessage::Info(t) if *t == hint)),
        "{shown:?}"
    );
}

/// Waits (frames of `h`) until `done` holds.
fn step_until(
    h: &mut egui_kittest::Harness<'_, DriftlessApp>,
    what: &str,
    mut done: impl FnMut(&DriftlessApp) -> bool,
) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !done(h.state()) {
        assert!(Instant::now() < deadline, "{what}");
        std::thread::sleep(Duration::from_millis(2));
        h.step();
    }
}

/// An E57 dropped onto the window is shown as a cloud of its scans; a file the reader
/// refuses gives its message in the status bar.
#[test]
fn dropped_e57_loads_cloud() {
    use driftless_export::{write_points, PointFormat, Points, ScanInfo};
    let dir = tmp("ui-hineinziehen-e57");
    let e57 = dir.join("Büro Scan.E57");
    let n = 300;
    let points = Points {
        xyz: (0..n).map(|i| [i as f64 * 0.01, 1.0, 2.0]).collect(),
        source: Some((0..n).map(|i| (i % 2) as u16).collect()),
        scans: ["Erdgeschoss", "Keller"]
            .iter()
            .enumerate()
            .map(|(k, name)| ScanInfo {
                source: k as u16,
                name: (*name).into(),
                pose: nalgebra::Isometry3::identity(),
            })
            .collect(),
        ..Points::default()
    };
    write_points(
        &e57,
        PointFormat::E57,
        &points,
        &driftless_io::progress::Progress::new(),
    )
    .unwrap();
    let mut h = window(app());
    drop_files(&mut h, &[dir.join("notiz.txt"), e57.clone()]);
    assert!(h.state().is_loading());
    step_until(&mut h, "E57 not loaded", |a| a.cloud().is_some());
    let cloud = h.state().cloud().unwrap();
    assert_eq!(cloud.path, e57);
    assert_eq!(cloud.points.len(), n);
    assert!(!cloud.points.source.is_empty());
    assert!(h.state().state().project.is_none());
    assert_eq!(errors(h.state()), Vec::<String>::new());

    // a compressed PCD: the reader's message
    let pcd = dir.join("gepackt.pcd");
    fs::write(
        &pcd,
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\n\
         POINTS 1\nDATA binary_compressed\n",
    )
    .unwrap();
    drop_files(&mut h, std::slice::from_ref(&pcd));
    step_until(&mut h, "no message", |a| !errors(a).is_empty());
    let shown = errors(h.state());
    assert!(
        shown
            .iter()
            .any(|e| e.contains("gepackt.pcd") && e.contains("„binary_compressed“")),
        "{shown:?}"
    );
}

/// An OBJ dropped onto the window is shown as a surface; a texture it names but that is
/// missing gives a warning in the log.
#[test]
fn dropped_obj_loads_surface() {
    use driftless_export::mesh::{write_obj, ImageKind, Mesh, MeshPage, PageImage};
    let dir = tmp("ui-hineinziehen-obj");
    let obj = dir.join("Netz.obj");
    let jpeg = {
        let img = image::RgbImage::from_pixel(2, 2, image::Rgb([10, 200, 10]));
        let mut out = std::io::Cursor::new(Vec::new());
        img.write_to(&mut out, image::ImageFormat::Jpeg).unwrap();
        out.into_inner()
    };
    let page = |z: f64, image: Option<PageImage>| MeshPage {
        positions: vec![[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]],
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2]],
        image,
        color: [90, 90, 90],
    };
    let image = PageImage {
        kind: ImageKind::Jpeg,
        bytes: jpeg.into(),
    };
    let mesh = Mesh {
        pages: vec![page(0.0, Some(image.clone())), page(2.0, Some(image))],
    };
    write_obj(&obj, &mesh, &driftless_io::progress::Progress::new()).unwrap();
    fs::remove_file(dir.join("Netz_seite_01.jpg")).unwrap();
    let mut h = window(app());
    drop_files(&mut h, &[dir.join("notiz.txt"), obj.clone()]);
    assert!(h.state().is_loading_surface());
    step_until(&mut h, "OBJ not loaded", |a| a.state().surface.is_some());
    let surface = h.state().state().surface.clone().unwrap();
    assert_eq!(surface.path, obj);
    assert_eq!((surface.triangles, surface.pages), (2, 2));
    assert_eq!(errors(h.state()), Vec::<String>::new());
    let log = &h.state().state().log;
    assert!(
        log.iter().any(|l| l.kind == LogKind::Warning
            && l.text.contains("Textur fehlt")
            && l.text.contains("Netz_seite_01.jpg")),
        "{log:?}"
    );
}
