//! The first hour (decision 5 of track U): a click on a map shows it (in „Daten“ its best
//! cloud, in „Bilder“ its newest fitting surface or else its cloud), and the hints and tooltips
//! say what is there: without a project the view asks for one, „Scan hinzufügen …“ says what it
//! wants, one point is „1 Punkt“, no internal building-block numbers, and „Lauf hinzufügen …“
//! (a command-line matter) lives in ☰ only.

use driftless_app::project::canonical;
use driftless_app::ui::{text, Action, DriftlessApp, Selection, Step, UiMessage};
use driftless_io::testscan::{room_bag_secs, tmp};
use egui_kittest::kittest::Queryable;
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

fn harness() -> H {
    let app = DriftlessApp::new(Arc::new(|| {}));
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    settle(&mut h);
    h
}

/// Frames until no step and no background work is left.
fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            h.step();
            return;
        }
        assert!(Instant::now() < deadline, "background work did not end");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn run(h: &mut H, action: Action) {
    h.state_mut().apply(action);
    settle(h);
}

/// The cloud the view shows.
fn cloud(h: &H) -> Option<PathBuf> {
    h.state().state().cloud.as_ref().map(|c| c.path.clone())
}

/// The surface the view shows.
fn surface(h: &H) -> Option<PathBuf> {
    h.state().state().surface.as_ref().map(|s| s.path.clone())
}

/// A project with the scan `halle` and its map, computed in the window.
fn project_with_map(dir: &Path) -> (H, PathBuf) {
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.5);
    let mut h = harness();
    run(
        &mut h,
        Action::NewProject {
            dir: dir.join("projekt"),
            name: "Halle".into(),
        },
    );
    run(&mut h, Action::AddScan(raw));
    h.state_mut().state_mut().map_settings.step = driftless_steps::map::MapStep::Clean;
    run(&mut h, Action::ComputeMap("halle".into()));
    let map = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/halle/slam-001/karte-001");
    assert!(map.join("clean.las").is_file());
    assert_eq!(errors(h.state()), Vec::<String>::new());
    (h, map)
}

#[test]
fn a_map_click_shows_it() {
    let dir = tmp("erste-stunde-karte");
    let (mut h, map) = project_with_map(&dir);
    // „Daten“: the map's entry in the tree shows its cloud
    run(&mut h, Action::Unshow(map.clone()));
    run(&mut h, Action::Select(Selection::Scan("halle".into())));
    assert_eq!(cloud(&h), None);
    let size = h.state().state().folder_size(&map).unwrap();
    h.get_by_label(&text::with_size("Karte 001", size)).click();
    settle(&mut h);
    assert_eq!(h.state().state().selection, Selection::Map(map.clone()));
    assert_eq!(cloud(&h), Some(map.join("clean.las")));

    // „Bilder“: the map's card shows its cloud while it has no surface …
    run(&mut h, Action::Unshow(map.clone()));
    run(&mut h, Action::SetStep(Step::Bilder));
    let card = text::map_of("halle", "SLAM 001", "Karte 001");
    h.get_by_label(&card).click();
    settle(&mut h);
    assert_eq!(cloud(&h), Some(map.join("clean.las")));
    assert_eq!(surface(&h), None);
    // … and its newest fitting surface once it has one
    h.state_mut().state_mut().surface_settings.texture = false;
    h.state_mut().state_mut().surface_settings.config.voxel = 0.05;
    run(&mut h, Action::RunSurface(map.clone()));
    run(&mut h, Action::Unshow(map.clone()));
    assert_eq!(surface(&h), None);
    h.get_by_label(&card).click();
    settle(&mut h);
    assert_eq!(
        surface(&h),
        Some(map.join("oberflaeche-001/oberflaeche.glb"))
    );
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

#[test]
fn hints_fit_what_is_there() {
    // no project: the view asks for one; no surface hint in „Bilder“
    let mut h = harness();
    h.get_by_label(text::VIEW_NO_PROJECT);
    assert!(h.query_by_label(text::VIEW_EMPTY).is_none());
    run(&mut h, Action::SetStep(Step::Bilder));
    assert!(h.query_by_label(text::NO_SURFACE_YET).is_none());
    h.get_by_label(text::VIEW_NO_PROJECT);

    // an empty project: „Scan hinzufügen …“ says what it wants
    let dir = tmp("erste-stunde-hinweise");
    run(
        &mut h,
        Action::NewProject {
            dir: dir.join("projekt"),
            name: "Halle".into(),
        },
    );
    run(&mut h, Action::SetStep(Step::Daten));
    h.get_by_label(text::ADD_SCAN).hover();
    for _ in 0..60 {
        h.step();
    }
    h.get_by_label(text::ADD_SCAN_HINT);
    assert!(h.query_by_label(text::COMPUTE_HINT).is_none());
    // a project without maps: „Bilder“ says so in the field, the view has no surface hint
    run(&mut h, Action::SetStep(Step::Bilder));
    assert!(h.query_by_label(text::NO_SURFACE_YET).is_none());

    // „Lauf hinzufügen …“ only in ☰, not under every scan
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.0);
    run(&mut h, Action::SetStep(Step::Daten));
    run(&mut h, Action::AddScan(raw));
    run(&mut h, Action::Select(Selection::Scan("halle".into())));
    h.get_by_label(text::NO_RUNS);
    assert!(h.query_by_label(text::ADD_RUN).is_none());
}

#[test]
fn texts_without_jargon_and_with_the_singular() {
    assert!(
        !text::PLAN_COMING.contains("Baustein"),
        "{}",
        text::PLAN_COMING
    );
    assert!(
        !text::MEASUREMENTS_NOT_SAVED.contains("Baustein"),
        "{}",
        text::MEASUREMENTS_NOT_SAVED
    );
    let one = text::cloud_loaded(Path::new("/p/eins.las"), 1, 0.0, 0.0);
    assert!(one.starts_with("eins.las geladen: 1 Punkt ("), "{one}");
    let many = text::cloud_loaded(Path::new("/p/viele.las"), 2, 0.0, 0.0);
    assert!(many.starts_with("viele.las geladen: 2 Punkte ("), "{many}");
    assert_eq!(text::points(1), "1 Punkt");
    assert_eq!(text::points(15_600_000), "15,6 Mio. Punkte");
}
