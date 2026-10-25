//! Pictures of the whole window for looking at by eye (colours, levels of detail, layout):
//! a synthetic cloud and, if `$DRIFTLESS_TESTDATA` has it, the colour cloud of the
//! Büro-Scan, with its run drawn over it, two places picked for a loop, and the odometry
//! compared with the corrected trajectory (nothing is written into the test data: the project
//! lives in `$DRIFTLESS_SHOTS`). Written as PNG into `$DRIFTLESS_SHOTS` (skipped without it);
//! never commit them. Run with
//!
//! ```text
//! DRIFTLESS_SHOTS=/tmp/shots cargo test -p driftless-app --release \
//!     --test ui_screenshots -- --ignored --nocapture
//! ```

use driftless_app::camera::CameraMode;
use driftless_app::project::{canonical, Project};
use driftless_app::render::{headless_device, ColorMode};
use driftless_app::ui::{text, Action, DriftlessApp, Selection, Step};
use eframe::egui;
use egui_kittest::kittest::Queryable;
use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
use egui_kittest::Harness;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

const SIZE: [f32; 2] = [1400.0, 900.0];

fn harness(app: DriftlessApp, rs: egui_wgpu::RenderState) -> Harness<'static, DriftlessApp> {
    Harness::builder()
        .with_size(SIZE)
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app)
}

/// Frames until the load and the upload have ended and the view has drawn.
fn wait_loaded(h: &mut Harness<'_, DriftlessApp>) {
    let deadline = Instant::now() + Duration::from_secs(600);
    while h.state().is_loading() || h.state().cloud().is_none() || h.state().view().is_uploading() {
        h.step();
        assert!(Instant::now() < deadline, "cloud not loaded");
        std::thread::sleep(Duration::from_millis(20));
    }
    for _ in 0..3 {
        h.step();
    }
}

fn shot(h: &mut Harness<'_, DriftlessApp>, dir: &Path, name: &str) {
    h.step();
    let image = h.render().expect("render");
    let path = dir.join(format!("{name}.png"));
    image.save(&path).expect("save png");
    let stats = h.state().state().view_stats;
    eprintln!("{} ({stats:?})", path.display());
}

/// A click at `pos` (points) without moving.
fn click(h: &mut Harness<'_, DriftlessApp>, pos: egui::Pos2) {
    let button = |pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: egui::Modifiers::NONE,
    };
    h.event(egui::Event::PointerMoved(pos));
    h.step();
    h.event(button(true));
    h.step();
    h.event(button(false));
    h.step();
}

#[test]
#[ignore = "writes pictures; needs $DRIFTLESS_SHOTS and a GPU"]
fn ui_screenshots() {
    let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_SHOTS nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();

    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.load_synthetic(3_000_000);
    let mut h = harness(app, rs);
    wait_loaded(&mut h);
    shot(&mut h, &dir, "synthetisch_farbe");
    h.state_mut().state_mut().view.color_mode = ColorMode::Intensity;
    shot(&mut h, &dir, "synthetisch_intensitaet");
    h.state_mut().state_mut().view.color_mode = ColorMode::Quality;
    shot(&mut h, &dir, "synthetisch_guete");
    // coarse levels: a small budget shows the samples of the upper nodes
    let view = &mut h.state_mut().state_mut().view;
    view.color_mode = ColorMode::Rgb;
    view.budget = 1_000_000;
    view.point_px = 1.0;
    shot(&mut h, &dir, "synthetisch_budget_1mio");
    let view = &mut h.state_mut().state_mut().view;
    view.budget = 6_000_000;
    view.point_px = 2.0;
    view.camera_mode = CameraMode::Fly;
    // a click into the middle of the view picks a point
    click(&mut h, egui::pos2(SIZE[0] * 0.45, SIZE[1] * 0.5));
    shot(&mut h, &dir, "synthetisch_punkt");

    let Some(data) = std::env::var_os("DRIFTLESS_TESTDATA").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_TESTDATA nicht gesetzt, echte Wolke übersprungen");
        return;
    };
    let run = data.join("ergebnisse-m5/buero");
    let path = run.join("karte/color.las");
    if !path.exists() {
        eprintln!("{} fehlt, echte Wolke übersprungen", path.display());
        return;
    }
    let view = &mut h.state_mut().state_mut().view;
    view.camera_mode = CameraMode::Orbit;
    h.state_mut().apply(Action::LoadCloud {
        path,
        run: Some(run.clone()),
    });
    wait_loaded(&mut h);
    shot(&mut h, &dir, "buero_farbe");
    h.state_mut().state_mut().view.color_mode = ColorMode::Quality;
    shot(&mut h, &dir, "buero_guete");
    h.state_mut().state_mut().view.color_mode = ColorMode::Intensity;
    shot(&mut h, &dir, "buero_intensitaet");
    let view = &mut h.state_mut().state_mut().view;
    view.color_mode = ColorMode::Rgb;
    view.budget = 1_000_000;
    shot(&mut h, &dir, "buero_budget_1mio");
    // from above with the ceiling cut away: the rooms instead of the roofs
    let summary = h.state().state().cloud.clone().unwrap();
    let ceiling = summary.ceiling.expect("Decke erkannt");
    eprintln!("z {:?}, Decke bei {ceiling:.2} m", summary.z_range);
    let view = &mut h.state_mut().state_mut().view;
    view.budget = 6_000_000;
    view.cut = Some([summary.z_range[0], ceiling]);
    let camera = h.state_mut().view_mut().camera_mut();
    camera.pitch = -89f32.to_radians();
    camera.distance *= 0.8;
    shot(&mut h, &dir, "buero_ohne_decke");

    // the run in a project: the corrections panel, the trajectory by quality, the loops, the
    // anchors and two picked places (the start and the pose of the end nearest to it)
    let scan = data.join("jmk6-2026-10-20/20261020173822");
    if !scan.is_dir() {
        eprintln!("{} fehlt, Korrektur-Bild übersprungen", scan.display());
        return;
    }
    let project_dir = dir.join("projekt-korrektur");
    let _ = std::fs::remove_dir_all(&project_dir);
    let mut project = Project::create(&project_dir, "Büro").unwrap();
    let id = project.add_scan(&scan).unwrap();
    project.add_run(&id, &run).unwrap();
    project.save().unwrap();
    h.state_mut().apply(Action::OpenProject(project_dir));
    let run = canonical(&run).unwrap();
    h.state_mut()
        .apply(Action::Select(Selection::Run(run.clone())));
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().overlay().is_none_or(|o| o.run != run)
        || h.state().state().correct.corrections.is_none()
    {
        h.step();
        assert!(Instant::now() < deadline, "run not read");
        std::thread::sleep(Duration::from_millis(20));
    }
    let overlay = h.state().overlay().unwrap().clone();
    let start = overlay.positions[0];
    let half = overlay.poses.len() / 2;
    let end = (half..overlay.poses.len())
        .min_by(|&i, &j| {
            let d = |k: usize| {
                let p = overlay.positions[k];
                (0..3).map(|a| (p[a] - start[a]).powi(2)).sum::<f32>()
            };
            d(i).total_cmp(&d(j))
        })
        .unwrap();
    eprintln!(
        "Stellen: {:.1} s und {:.1} s",
        overlay.time_of(0),
        overlay.time_of(end)
    );
    let view = &mut h.state_mut().state_mut().view;
    view.show_submaps = true;
    view.show_odometry = true;
    h.state_mut().state_mut().correct.picking = true;
    h.state_mut().pose_picked(0);
    h.state_mut().pose_picked(end);
    shot(&mut h, &dir, "buero_korrektur");

    // before and after: the odometry (A, orange, aligned) over the corrected trajectory (B),
    // with the numbers of the comparison; the sections above folded away
    let app = h.state_mut();
    app.state_mut().correct.picking = false;
    app.state_mut().correct.clear_places();
    let view = &mut app.state_mut().view;
    view.show_submaps = false;
    view.show_odometry = false;
    view.show_loops = false;
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().state().compare.current().is_none()
        || h.state().overlay().is_none_or(|o| o.compare.is_empty())
    {
        h.step();
        let c = &h.state().state().compare;
        assert!(
            Instant::now() < deadline,
            "comparison not ready: {:?} {:?} {:?}",
            c.a,
            c.b,
            c.result.as_ref().map(|r| r.as_ref().err())
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    let c = h.state().state().compare.current().unwrap();
    eprintln!(
        "{} gegen {}:\n{}",
        c.a.label(),
        c.b.label(),
        driftless_steps::compare::render_text(&c.report)
    );
    // „Korrekturen“ lies under „Erweitert“: open „Erweitert“, then fold „Korrekturen“ to leave
    // room for the comparison
    driftless_app::ui::steps::daten::set_advanced(&h.ctx, true);
    h.step();
    h.get_by_label(text::CORRECTIONS).click();
    h.step();
    let camera = h.state_mut().view_mut().camera_mut();
    camera.pitch = -89f32.to_radians();
    shot(&mut h, &dir, "buero_vorher_nachher");
}

/// The surface of `$DRIFTLESS_SURFACE` (a `.glb` or a surface folder) alone, with the
/// ceiling cut away, as `app_oberflaeche.png` in `$DRIFTLESS_SHOTS`. Run with
///
/// ```text
/// DRIFTLESS_SHOTS=… DRIFTLESS_SURFACE=…/oberflaeche-001 cargo test -p driftless-app \
///     --release --test ui_screenshots surface_screenshot -- --ignored --nocapture
/// ```
#[test]
#[ignore = "writes a picture; needs $DRIFTLESS_SHOTS, $DRIFTLESS_SURFACE and a GPU"]
fn surface_screenshot() {
    let (Some(dir), Some(surface)) = (
        std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_SURFACE").map(PathBuf::from),
    ) else {
        eprintln!("DRIFTLESS_SHOTS oder DRIFTLESS_SURFACE nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::LoadSurface(surface));
    let mut h = harness(app, rs);
    let deadline = Instant::now() + Duration::from_secs(300);
    while h.state().is_loading_surface() {
        h.step();
        assert!(Instant::now() < deadline, "surface not loaded");
        std::thread::sleep(Duration::from_millis(50));
    }
    let summary = h.state().state().surface.clone().expect("surface loaded");
    eprintln!("{summary:?}");
    // looking in from above: the ceiling cut away
    let [lo, hi] = summary.z_range;
    h.state_mut().state_mut().view.cut = Some([lo, lo + (hi - lo) * 0.6]);
    for _ in 0..3 {
        h.step();
    }
    shot(&mut h, &dir, "app_oberflaeche");
}

/// The surfaces of a scan in a project, as `app_projekt_oberflaeche.png` in
/// `$DRIFTLESS_SHOTS`: `$DRIFTLESS_SCAN` is a scan folder whose run `driftless/` (made with
/// the CLI) holds a map and surfaces `oberflaeche…`. The project lives in `$DRIFTLESS_SHOTS`;
/// nothing is written into the scan. Checks the tree (stale surfaces marked), the key `O` and
/// the loading of a surface from the tree. Run with
///
/// ```text
/// DRIFTLESS_SHOTS=… DRIFTLESS_SCAN=…/20261022133554 cargo test -p driftless-app --release \
///     --test ui_screenshots project_surface_screenshot -- --ignored --nocapture
/// ```
#[test]
#[ignore = "writes a picture; needs $DRIFTLESS_SHOTS, $DRIFTLESS_SCAN and a GPU"]
fn project_surface_screenshot() {
    let (Some(dir), Some(scan)) = (
        std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_SCAN").map(PathBuf::from),
    ) else {
        eprintln!("DRIFTLESS_SHOTS oder DRIFTLESS_SCAN nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    let project_dir = dir.join("projekt-oberflaeche");
    let _ = std::fs::remove_dir_all(&project_dir);
    let mut project = Project::create(&project_dir, "Schwarzer Raum").unwrap();
    let id = project.add_scan(&scan).unwrap();
    project.add_run(&id, &scan.join("driftless")).unwrap();
    project.save().unwrap();

    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::OpenProject(project_dir));
    let mut h = harness(app, rs);
    // the statuses of maps and surfaces come from the background
    let deadline = Instant::now() + Duration::from_secs(300);
    while h.state().wants_repaint_while_busy() {
        h.step();
        assert!(Instant::now() < deadline, "statuses not read");
        std::thread::sleep(Duration::from_millis(50));
    }
    let maps: Vec<_> = h.state().state().scans[0]
        .runs
        .iter()
        .flat_map(|r| r.maps.clone())
        .collect();
    let map = maps.first().expect("a map in the run");
    for s in &map.surfaces {
        eprintln!("{} veraltet: {} ({})", s.label, s.stale, s.dir.display());
    }
    let surface = map
        .surfaces
        .iter()
        .rev()
        .find(|s| s.label.ends_with("2cm"))
        .or(map.surfaces.last())
        .expect("a surface in the map")
        .clone();
    let label = if surface.stale {
        text::surface_stale(&surface.label)
    } else {
        surface.label.clone()
    };
    // the cloud of the map first, then the surface from the tree
    if let Some(cloud) = map.clouds.first() {
        h.state_mut()
            .apply(Action::Select(Selection::Cloud(cloud.clone())));
        wait_loaded(&mut h);
    }
    h.get_by_label(&label).click();
    h.step();
    let deadline = Instant::now() + Duration::from_secs(300);
    while h.state().is_loading_surface() || h.state().view().is_uploading_surface() {
        h.step();
        assert!(Instant::now() < deadline, "surface not loaded");
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(
        h.state().state().selection,
        Selection::Surface(surface.dir.clone())
    );
    let summary = h.state().state().surface.clone().expect("surface loaded");
    eprintln!("{summary:?}");
    // O: Oberfläche → Punktwolke → beides → Oberfläche
    for _ in 0..3 {
        h.key_press(egui::Key::O);
        h.step();
        eprintln!("Anzeige: {:?}", h.state().state().view.show);
    }
    // the cut is over the cloud while one is shown (its frame), else over the surface
    let cloud = h.state().state().cloud.clone();
    let [lo, hi] = cloud.as_ref().map_or(summary.z_range, |c| c.z_range);
    let top = cloud
        .and_then(|c| c.ceiling)
        .unwrap_or(lo + (hi - lo) * 0.6);
    h.state_mut().state_mut().view.cut = Some([lo, top]);
    for _ in 0..3 {
        h.step();
    }
    shot(&mut h, &dir, "app_projekt_oberflaeche");
}

/// The layout with rail, field „Daten“ and view: a project with one small synthetic scan and a
/// synthetic cloud, as `layout.png` in `$DRIFTLESS_SHOTS`. Run with
///
/// ```text
/// DRIFTLESS_SHOTS=… cargo test -p driftless-app --test ui_screenshots layout_screenshot \
///     -- --ignored --nocapture
/// ```
#[test]
#[ignore = "writes a picture; needs $DRIFTLESS_SHOTS and a GPU"]
fn layout_screenshot() {
    let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_SHOTS nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();
    let work = dir.join("projekt-aufbau");
    let _ = std::fs::remove_dir_all(&work);
    let scan = work.join("scan-1335");
    std::fs::create_dir_all(&scan).unwrap();
    driftless_io::testscan::room_bag_secs(&scan, true, 1.0);
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::NewProject {
        dir: work.join("projekt"),
        name: "Schwarzer Raum".into(),
    });
    app.apply(Action::AddScan(scan));
    app.load_synthetic(1_000_000);
    let mut h = harness(app, rs);
    wait_loaded(&mut h);
    shot(&mut h, &dir, "layout");
}

/// The field „Daten“ as in the design: project „Schwarzer Raum“ with three small synthetic
/// scans – one computed („fertig“), one whose map is being computed (a job held at 62 %), one
/// new – over a synthetic cloud, as `daten.png` in `$DRIFTLESS_SHOTS`.
#[test]
#[ignore = "writes a picture; needs $DRIFTLESS_SHOTS and a GPU"]
fn daten_screenshot() {
    let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_SHOTS nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();
    let work = dir.join("projekt-daten");
    let _ = std::fs::remove_dir_all(&work);
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::NewProject {
        dir: work.join("projekt"),
        name: "Schwarzer Raum".into(),
    });
    let mut ids = Vec::new();
    for name in ["Scan 13:35", "Scan 13:30", "Scan 13:25"] {
        let scan = work.join(name.replace([' ', ':'], "-"));
        std::fs::create_dir_all(&scan).unwrap();
        driftless_io::testscan::room_bag_secs(&scan, true, 1.0);
        app.apply(Action::AddScan(scan));
        ids.push(
            app.state()
                .project
                .as_ref()
                .unwrap()
                .scans
                .last()
                .unwrap()
                .id
                .clone(),
        );
    }
    let settle = |app: &mut DriftlessApp| {
        let deadline = Instant::now() + Duration::from_secs(120);
        while app.wants_repaint_while_busy() {
            assert!(Instant::now() < deadline, "work did not end");
            std::thread::sleep(Duration::from_millis(5));
            app.tick();
        }
    };
    // 13:35 computed, 13:30 with a run whose map is being computed
    app.apply(Action::ComputeMap(ids[0].clone()));
    settle(&mut app);
    app.apply(Action::RunSlam(ids[1].clone()));
    settle(&mut app);
    // the shown names as in the design (folder names cannot hold „:“ everywhere)
    let mut project = app.state().project.clone().unwrap();
    for (scan, name) in project
        .scans
        .iter_mut()
        .zip(["Scan 13:35", "Scan 13:30", "Scan 13:25"])
    {
        scan.name = name.into();
    }
    app.state_mut().set_project(project);
    let run = app.state().scan_node(&ids[1]).unwrap().runs[0].dir.clone();
    app.start_job(text::STEP_MAP, |p| {
        p.phase("Dichte Wolke", Some(100));
        p.advance(62);
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(driftless_steps::StepError::Cancelled)
    })
    .unwrap();
    app.state_mut().running = Some(Action::RunMap(run));
    app.load_synthetic(1_000_000);
    let mut h = harness(app, rs);
    wait_loaded(&mut h);
    shot(&mut h, &dir, "daten");
    h.state_mut().apply(Action::Cancel);
}

/// The field „Bilder“ as in the design: project „Schwarzer Raum“ with three small synthetic
/// scans and their maps – one with a surface („fertig“, shown in the view), one whose surface
/// is being computed (a job held at 40 %), one without – as `bilder.png` in
/// `$DRIFTLESS_SHOTS`.
#[test]
#[ignore = "writes a picture; needs $DRIFTLESS_SHOTS and a GPU"]
fn bilder_screenshot() {
    let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_SHOTS nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();
    let work = dir.join("projekt-bilder");
    let _ = std::fs::remove_dir_all(&work);
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::NewProject {
        dir: work.join("projekt"),
        name: "Schwarzer Raum".into(),
    });
    let names = ["Scan 13:35", "Scan 13:30", "Scan 13:25"];
    let mut ids = Vec::new();
    for name in names {
        let scan = work.join(name.replace([' ', ':'], "-"));
        std::fs::create_dir_all(&scan).unwrap();
        driftless_io::testscan::room_bag_secs(&scan, true, 1.0);
        app.apply(Action::AddScan(scan));
        ids.push(
            app.state()
                .project
                .as_ref()
                .unwrap()
                .scans
                .last()
                .unwrap()
                .id
                .clone(),
        );
    }
    let settle = |app: &mut DriftlessApp| {
        let deadline = Instant::now() + Duration::from_secs(120);
        while app.wants_repaint_while_busy() {
            assert!(Instant::now() < deadline, "work did not end");
            std::thread::sleep(Duration::from_millis(5));
            app.tick();
        }
    };
    app.state_mut().map_settings.step = driftless_steps::map::MapStep::Clean;
    app.apply(Action::ComputeAllMaps);
    settle(&mut app);
    let map_of = |app: &DriftlessApp, id: &str| {
        let node = app.state().scan_node(id).unwrap();
        node.runs.last().unwrap().maps.last().unwrap().dir.clone()
    };
    // 13:35 with a surface, coarse enough for the synthetic room
    app.state_mut().surface_settings = driftless_app::project::SurfaceSettings {
        config: driftless_map::surface::SurfaceConfig {
            voxel: 0.15,
            min_weight: 1,
            min_component_faces: 10,
            page_size: 256,
            simplify_error: 0.01,
            ..Default::default()
        },
        texture: true,
    };
    app.apply(Action::RunSurface(map_of(&app, &ids[0])));
    settle(&mut app);
    // the shown names as in the design (folder names cannot hold „:“ everywhere)
    let mut project = app.state().project.clone().unwrap();
    for (scan, name) in project.scans.iter_mut().zip(names) {
        scan.name = name.into();
    }
    app.state_mut().set_project(project);
    app.apply(Action::SetStep(Step::Bilder));
    let surface = app.state().scan_node(&ids[0]).unwrap().runs[0].maps[0].surfaces[0]
        .dir
        .clone();
    app.apply(Action::Select(Selection::Surface(surface)));
    settle(&mut app);
    app.apply(Action::Select(Selection::None));
    app.state_mut().surface_settings = Default::default();
    app.state_mut().surface_settings.config.voxel = 0.015;
    // 13:30: its surface is being computed
    let map = map_of(&app, &ids[1]);
    app.start_job(text::STEP_SURFACE, |p| {
        p.phase("Oberfläche", Some(100));
        p.advance(40);
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(5));
        }
        Err(driftless_steps::StepError::Cancelled)
    })
    .unwrap();
    app.state_mut().running = Some(Action::RunSurface(map));
    let mut h = harness(app, rs);
    for _ in 0..10 {
        h.step();
        std::thread::sleep(Duration::from_millis(20));
    }
    shot(&mut h, &dir, "bilder");
    h.state_mut().apply(Action::Cancel);
}

/// Points in world size against fixed pixels: the GPU time of the synthetic cloud of
/// `--synthetic 6_000_000` in both modes, from afar and close to a wall, and – with
/// `$DRIFTLESS_CLOUD`, a colour cloud of a map folder – `flecken_pixel.png` and
/// `flecken_meter.png` close to a wall, in `$DRIFTLESS_SHOTS`. Run with
///
/// ```text
/// DRIFTLESS_SHOTS=… DRIFTLESS_CLOUD=…/color.las cargo test -p driftless-app --release \
///     --test ui_screenshots spot_screenshots -- --ignored --nocapture
/// ```
#[test]
#[ignore = "writes pictures; needs $DRIFTLESS_SHOTS and a GPU"]
fn spot_screenshots() {
    use driftless_app::render::{with_timestamps, PointSize};
    let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_SHOTS nicht gesetzt, Test übersprungen");
        return;
    };
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    std::fs::create_dir_all(&dir).unwrap();
    // with timestamps, as the program asks for them: the view reports the GPU time
    let setup = match default_wgpu_setup() {
        egui_wgpu::WgpuSetup::CreateNew(s) => egui_wgpu::WgpuSetup::CreateNew(with_timestamps(s)),
        other => other,
    };
    let rs = create_render_state(setup, egui_wgpu::RendererOptions::PREDICTABLE);
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.load_synthetic(6_000_000);
    let mut h = harness(app, rs);
    wait_loaded(&mut h);

    // The median GPU time of frames that each turn the camera a little (so that every step
    // draws anew).
    let median_ms = |h: &mut Harness<'_, DriftlessApp>, size: PointSize| {
        h.state_mut().state_mut().view.point_size = size;
        let mut times = Vec::new();
        for i in 0..80 {
            h.state_mut().view_mut().camera_mut().yaw += if i % 2 == 0 { 1e-3 } else { -1e-3 };
            h.step();
            std::thread::sleep(Duration::from_millis(5));
            if i >= 10 && h.state().view().gpu_timed() {
                times.push(h.state().view().frame_ms());
            }
        }
        times.sort_by(f32::total_cmp);
        let drawn = h.state().view().drawn_points();
        (times.get(times.len() / 2).copied(), drawn)
    };
    for place in ["fern", "nah"] {
        if place == "nah" {
            to_wall(&mut h, 1.0);
        }
        let pixel = median_ms(&mut h, PointSize::Pixel);
        let world = median_ms(&mut h, PointSize::World);
        let pixel_again = median_ms(&mut h, PointSize::Pixel);
        eprintln!(
            "synthetisch 6 Mio., {place}: Pixel {:?} ms ({} Punkte), Meter {:?} ms ({} Punkte), \
             Pixel wieder {:?} ms",
            pixel.0, pixel.1, world.0, world.1, pixel_again.0
        );
    }

    let Some(cloud) = std::env::var_os("DRIFTLESS_CLOUD").map(PathBuf::from) else {
        eprintln!("DRIFTLESS_CLOUD nicht gesetzt, echte Wolke übersprungen");
        return;
    };
    h.state_mut().apply(Action::LoadCloud {
        path: cloud,
        run: None,
    });
    wait_loaded(&mut h);
    eprintln!(
        "Fleckgröße der Karte: {} m",
        h.state().cloud().unwrap().point_m
    );
    to_wall(&mut h, 1.2);
    h.state_mut().state_mut().view.point_size = PointSize::Pixel;
    shot(&mut h, &dir, "flecken_pixel");
    h.state_mut().state_mut().view.point_size = PointSize::World;
    shot(&mut h, &dir, "flecken_meter");
}

/// Orbits around the nearest point straight ahead (+x) of the middle of the cloud, at
/// `distance` metres, looking horizontally at it: close to a wall.
fn to_wall(h: &mut Harness<'_, DriftlessApp>, distance: f32) {
    let points = &h.state().cloud().expect("cloud").points;
    let (min, max) = points.bounds();
    let centre: [f32; 3] = std::array::from_fn(|k| (min[k] + max[k]) / 2.0);
    // 1.5 m above the floor (the lowest 2 % of the heights: stray points below it do not
    // count), not the middle of the bounds
    let mut heights: Vec<f32> = points.xyz.iter().step_by(97).map(|p| p[2]).collect();
    heights.sort_by(f32::total_cmp);
    let floor = heights.get(heights.len() / 50).copied().unwrap_or(min[2]);
    let z = (floor + 1.5).min(max[2]);
    let hit = points
        .xyz
        .iter()
        .filter(|p| p[0] > centre[0] && (p[1] - centre[1]).abs() < 0.05 && (p[2] - z).abs() < 0.05)
        .map(|p| p[0])
        .min_by(f32::total_cmp)
        .unwrap_or(max[0]);
    eprintln!("Wand bei x = {hit:.2} m (Mitte {centre:?}, z = {z:.2})");
    let camera = h.state_mut().view_mut().camera_mut();
    camera.target = [hit, centre[1], z].into();
    camera.yaw = 0.0;
    camera.pitch = 0.0;
    camera.distance = distance;
    for _ in 0..3 {
        h.step();
    }
}
