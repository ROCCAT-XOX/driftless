//! Pictures of the whole window for looking at by eye (colours, levels of detail, layout):
//! a synthetic cloud and, if `$DRIFTLESS_TESTDATA` has it, the colour cloud of the
//! Büro-Scan. Written as PNG into `$DRIFTLESS_SHOTS` (skipped without it); never commit
//! them. Run with
//! `DRIFTLESS_SHOTS=/tmp/shots cargo test -p driftless-app --release --test ui_screenshots -- --ignored --nocapture`.

use driftless_app::camera::CameraMode;
use driftless_app::render::{headless_device, ColorMode};
use driftless_app::ui::{Action, DriftlessApp};
use eframe::egui;
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

/// Frames until the load has ended and the view has drawn.
fn wait_loaded(h: &mut Harness<'_, DriftlessApp>) {
    let deadline = Instant::now() + Duration::from_secs(600);
    while h.state().is_loading() || h.state().cloud().is_none() {
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
    let run = data.join("ergebnisse-buero");
    let path = run.join("color.las");
    if !path.exists() {
        eprintln!("{} fehlt, echte Wolke übersprungen", path.display());
        return;
    }
    let view = &mut h.state_mut().state_mut().view;
    view.camera_mode = CameraMode::Orbit;
    h.state_mut().apply(Action::LoadCloud {
        path,
        run: Some(run),
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
}
