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
use driftless_app::ui::{text, Action, DriftlessApp, Selection};
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
    for section in [text::CORRECTIONS, text::VIEW_SETTINGS] {
        h.get_by_label(section).click();
        h.step();
    }
    let camera = h.state_mut().view_mut().camera_mut();
    camera.pitch = -89f32.to_radians();
    shot(&mut h, &dir, "buero_vorher_nachher");
}
