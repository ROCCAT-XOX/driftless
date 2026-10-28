//! The surface in the project, without a window: the step „Oberfläche rechnen“ on a synthetic
//! scan, its folder in the tree, the note on a stale surface and the key `O` of the view.

use driftless_app::project::{canonical, run_surface_job, SurfaceSettings};
use driftless_app::render::ShowMode;
use driftless_app::ui::view3d::effective_show;
use driftless_app::ui::{
    text, Action, AppState, CloudSummary, DriftlessApp, Selection, Step, SurfaceSummary, UiMessage,
};
use driftless_gpu::GpuPref;
use driftless_io::images::RgbImage;
use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_map::surface::{encode_pages, write_glb, SurfaceConfig, TexturePage, TexturedMesh};
use driftless_steps::map::MapStep;
use driftless_steps::StepError;
use eframe::egui;
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

/// Frames until no step and no background work is left.
fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            break;
        }
        assert!(Instant::now() < deadline, "step did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// Applies `action` and runs frames until it ended.
fn run(h: &mut H, action: Action) {
    h.state_mut().apply(action.clone());
    assert_eq!(errors(h.state()), Vec::<String>::new(), "{action:?}");
    settle(h);
}

/// Coarse settings: the synthetic room has a point every 10 cm or so.
fn small_config() -> SurfaceConfig {
    SurfaceConfig {
        voxel: 0.15,
        min_weight: 1,
        min_component_faces: 10,
        page_size: 256,
        simplify_error: 0.01,
        ..SurfaceConfig::default()
    }
}

/// The whole window of a project `dir/projekt` with the scan `halle` (1.5 s, static), a SLAM
/// run and a cleaned map of it, the map selected. Returns the harness, the run and the map.
fn project_with_map(dir: &Path) -> (H, PathBuf, PathBuf) {
    let scan = dir.join("halle");
    fs::create_dir_all(&scan).unwrap();
    room_bag_secs(&scan, true, 1.5);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    app.apply(Action::AddScan(scan));
    app.state_mut().map_settings.step = MapStep::Clean;
    app.state_mut().surface_settings = SurfaceSettings {
        config: small_config(),
        texture: true,
    };
    let mut h = Harness::builder()
        .with_size([1400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    run(&mut h, Action::RunSlam("halle".into()));
    let run_dir = canonical(&dir.join("projekt"))
        .unwrap()
        .join("scans/halle/slam-001");
    run(&mut h, Action::RunMap(run_dir.clone()));
    let map = run_dir.join("karte-001");
    assert_eq!(h.state().state().selection, Selection::Map(map.clone()));
    (h, run_dir, map)
}

/// The surfaces of the map `map` in the tree: (folder, label, stale).
fn surfaces(state: &AppState, map: &Path) -> Vec<(PathBuf, String, bool)> {
    state
        .scans
        .iter()
        .flat_map(|s| &s.runs)
        .flat_map(|r| &r.maps)
        .filter(|m| m.dir == map)
        .flat_map(|m| &m.surfaces)
        .map(|s| (s.dir.clone(), s.label.clone(), s.stale))
        .collect()
}

#[test]
fn surface_button_runs_step() {
    let dir = tmp("ui-oberflaeche");
    let (mut h, _, map) = project_with_map(&dir);
    // „Oberfläche rechnen“ is in the step „Bilder“
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.step();
    h.get_by_label(text::STEP_SURFACE).click();
    h.step();
    assert!(h.state().state().job.is_some(), "the step did not start");
    settle(&mut h);
    let surface = map.join("oberflaeche-001");
    assert!(surface.join("oberflaeche.glb").is_file());
    assert!(surface.join("oberflaeche.json").is_file());
    // the new surface is selected, listed under its map and shown
    let state = h.state().state();
    assert_eq!(state.selection, Selection::Surface(surface.clone()));
    assert_eq!(
        surfaces(state, &map),
        [(surface.clone(), "Oberfläche 001".to_string(), false)]
    );
    let shown = state.surface.as_ref().expect("surface loaded");
    assert_eq!(shown.path, surface.join("oberflaeche.glb"));
    assert!(shown.triangles > 0);
    h.get_by_label("Oberfläche 001");
    h.get_by_label(text::THIS_SURFACE);

    // the same settings again: no second folder, the surface fits
    run(&mut h, Action::RunSurface(map.clone()));
    assert!(!map.join("oberflaeche-002").exists());
    // other settings: a new folder
    h.state_mut().state_mut().surface_settings.texture = false;
    run(&mut h, Action::RunSurface(map.clone()));
    assert!(map.join("oberflaeche-002/oberflaeche.glb").is_file());
}

#[test]
fn stale_surface_marked() {
    let dir = tmp("ui-oberflaeche-veraltet");
    let (mut h, run_dir, map) = project_with_map(&dir);
    run(&mut h, Action::RunSurface(map.clone()));
    let surface = map.join("oberflaeche-001");
    assert!(!surfaces(h.state().state(), &map)[0].2);
    h.state_mut()
        .apply(Action::Select(Selection::Map(map.clone())));
    h.step();
    assert!(h
        .query_by_label(&text::surface_stale("Oberfläche 001"))
        .is_none());

    // the run is computed anew; on the static synthetic scan its trajectory may come out
    // the same, so it is changed as well: the map, and with it the surface, no longer fit
    run(&mut h, Action::Reuse(run_dir.clone()));
    let mut traj = fs::read(run_dir.join("traj.tum")).unwrap();
    traj.extend_from_slice(b"\n");
    fs::write(run_dir.join("traj.tum"), traj).unwrap();
    run(&mut h, Action::OpenProject(dir.join("projekt")));
    assert_eq!(
        surfaces(h.state().state(), &map),
        [(surface, "Oberfläche 001".to_string(), true)]
    );
    h.get_by_label(&text::surface_stale("Oberfläche 001"));
}

fn summaries(state: &mut AppState) {
    state.cloud = Some(CloudSummary {
        path: "wolke.las".into(),
        points: 10,
        has_rgb: true,
        has_scans: false,
        quality_known: false,
        z_range: [0.0, 1.0],
        ceiling: None,
        point_m: 0.015,
    });
    state.surface = Some(SurfaceSummary {
        path: "oberflaeche.glb".into(),
        triangles: 712_218,
        pages: 12,
        origin: [0.0; 3],
        z_range: [0.0, 1.0],
    });
}

#[test]
fn toggle_cycles_show_mode() {
    assert_eq!(ShowMode::default(), ShowMode::Surface);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    summaries(app.state_mut());
    let mut h = Harness::builder()
        .with_size([1400.0, 1000.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.step();
    let show = |h: &H| h.state().state().view.show;
    assert_eq!(show(&h), ShowMode::Surface);
    for expected in [ShowMode::Cloud, ShowMode::Both, ShowMode::Surface] {
        h.key_press(egui::Key::O);
        h.step();
        assert_eq!(show(&h), expected);
    }
    // without a cloud only the surface can be shown: the key does nothing
    h.state_mut().state_mut().cloud = None;
    h.key_press(egui::Key::O);
    h.step();
    assert_eq!(show(&h), ShowMode::Surface);
}

#[test]
fn surface_of_an_older_program_is_stale() {
    let dir = tmp("ui-oberflaeche-alt");
    let (mut h, _, map) = project_with_map(&dir);
    run(&mut h, Action::RunSurface(map.clone()));
    // a report as an earlier version wrote it: a setting it did not know yet
    let report = map.join("oberflaeche-001/oberflaeche.json");
    let text = fs::read_to_string(&report).unwrap();
    assert!(text.contains("\"max_hole_perimeter\""));
    fs::write(
        &report,
        text.replace("\"max_hole_perimeter\"", "\"max_hole_edges\""),
    )
    .unwrap();
    run(&mut h, Action::OpenProject(dir.join("projekt")));
    assert!(surfaces(h.state().state(), &map)[0].2, "not marked stale");
}

#[test]
fn surface_without_report_is_stale() {
    let dir = tmp("ui-oberflaeche-ohne-bericht");
    let (mut h, _, map) = project_with_map(&dir);
    run(&mut h, Action::RunSurface(map.clone()));
    // a glb without its json: the step was killed between the two, or the folder copied
    fs::remove_file(map.join("oberflaeche-001/oberflaeche.json")).unwrap();
    run(&mut h, Action::OpenProject(dir.join("projekt")));
    assert!(surfaces(h.state().state(), &map)[0].2, "not marked stale");
}

#[test]
fn tree_marks_stale_after_a_step() {
    let dir = tmp("ui-oberflaeche-nach-schritt");
    let (mut h, run_dir, map) = project_with_map(&dir);
    run(&mut h, Action::RunSurface(map.clone()));
    assert!(!surfaces(h.state().state(), &map)[0].2);
    // the trajectory changes; the next step reads the tree again and the background check
    // marks the surface, without opening the project anew
    let mut traj = fs::read(run_dir.join("traj.tum")).unwrap();
    traj.extend_from_slice(b"\n");
    fs::write(run_dir.join("traj.tum"), traj).unwrap();
    run(&mut h, Action::Inspect("halle".into()));
    assert!(surfaces(h.state().state(), &map)[0].2, "not marked stale");
    h.get_by_label(&text::surface_stale("Oberfläche 001"));
}

#[test]
fn cancelled_surface_leaves_no_folder() {
    let dir = tmp("ui-oberflaeche-abbruch");
    let (h, _, map) = project_with_map(&dir);
    let state = h.state().state();
    let (scan, run, info) = state.map(&map).unwrap();
    let scan_dir = state
        .project
        .as_ref()
        .unwrap()
        .scan(&scan.id)
        .unwrap()
        .path
        .clone();
    let progress = Progress::new();
    progress.cancel();
    let settings = state.surface_settings.clone();
    let e = run_surface_job(
        run,
        &scan_dir,
        info,
        &settings,
        GpuPref::default(),
        &progress,
    )
    .unwrap_err();
    assert!(matches!(e, StepError::Cancelled), "{e}");
    assert!(!map.join("oberflaeche-001").exists());
}

/// Writes a one-page glb of `n` triangles to `path`.
fn small_glb(path: &Path, n: usize) {
    let mut tex = TexturedMesh::default();
    let positions: Vec<[f32; 3]> = (0..n * 3)
        .map(|i| [i as f32, (i % 3) as f32, 0.0])
        .collect();
    tex.pages = vec![TexturePage {
        uvs: vec![[0.5; 2]; positions.len()],
        indices: (0..n as u32)
            .map(|t| [3 * t, 3 * t + 1, 3 * t + 2])
            .collect(),
        positions,
        image: RgbImage {
            width: 4,
            height: 4,
            data: vec![128; 48],
        },
    }];
    write_glb(path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
}

#[test]
fn changed_surface_file_loads_again() {
    let dir = tmp("ui-oberflaeche-neu-laden");
    let glb = dir.join("oberflaeche.glb");
    small_glb(&glb, 1);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    let wait = |app: &mut DriftlessApp| {
        let deadline = Instant::now() + Duration::from_secs(30);
        while app.is_loading_surface() {
            app.tick();
            assert!(Instant::now() < deadline, "surface not loaded");
            std::thread::sleep(Duration::from_millis(2));
        }
    };
    let triangles = |app: &DriftlessApp| app.state().surface.as_ref().map(|s| s.triangles);
    app.apply(Action::LoadSurface(glb.clone()));
    wait(&mut app);
    assert_eq!(triangles(&app), Some(1));
    // unchanged: not loaded again
    app.apply(Action::LoadSurface(glb.clone()));
    assert!(!app.is_loading_surface());
    // written anew (as `driftless surface -o` into the same folder does): loaded again
    small_glb(&glb, 2);
    app.apply(Action::LoadSurface(glb.clone()));
    assert!(app.is_loading_surface());
    wait(&mut app);
    assert_eq!(triangles(&app), Some(2));
    // asked for again from the menu: loaded even unchanged
    app.apply(Action::ReloadSurface(glb.clone()));
    assert!(app.is_loading_surface());
    wait(&mut app);
}

/// A LAS file of `n` points at `path`.
fn small_las(path: &Path, n: usize) {
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    let mut builder = las::Builder::from((1, 2));
    builder.point_format = las::point::Format::new(1).unwrap();
    let mut w = las::Writer::from_path(path, builder.into_header().unwrap()).unwrap();
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
}

#[test]
fn cloud_of_another_map_drops_surface() {
    let dir = tmp("ui-oberflaeche-andere-karte");
    let surface = dir.join("karte-001/oberflaeche-001");
    fs::create_dir_all(&surface).unwrap();
    small_glb(&surface.join("oberflaeche.glb"), 2);
    let (own, other) = (
        dir.join("karte-001/color.las"),
        dir.join("karte-002/color.las"),
    );
    small_las(&own, 100);
    small_las(&other, 100);
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    let wait = |app: &mut DriftlessApp| {
        let deadline = Instant::now() + Duration::from_secs(30);
        while app.is_loading_surface() || app.is_loading() {
            app.tick();
            assert!(Instant::now() < deadline, "not loaded");
            std::thread::sleep(Duration::from_millis(2));
        }
    };
    let drawn = |app: &DriftlessApp| {
        effective_show(
            app.state().view.show,
            app.view().has_surface(),
            app.cloud().is_some(),
        )
    };
    app.apply(Action::LoadSurface(surface.clone()));
    wait(&mut app);
    assert!(app.state().surface.is_some());
    // a cloud of the surface's map: both stay, the picked cloud is what is drawn
    app.apply(Action::Select(Selection::Cloud(own)));
    wait(&mut app);
    assert!(app.state().surface.is_some() && app.view().has_surface());
    assert_eq!(drawn(&app), ShowMode::Cloud);
    // a cloud of another map: the surface goes, the cloud is drawn
    app.state_mut().view.show = ShowMode::Surface;
    app.apply(Action::Select(Selection::Cloud(other)));
    wait(&mut app);
    assert!(app.state().surface.is_none());
    assert!(!app.view().has_surface());
    assert_eq!(drawn(&app), ShowMode::Cloud);
    // another project: the surface goes too
    app.apply(Action::LoadSurface(surface));
    wait(&mut app);
    assert!(app.view().has_surface());
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Neu".into(),
    });
    assert!(app.state().surface.is_none());
    assert!(!app.view().has_surface());
}
