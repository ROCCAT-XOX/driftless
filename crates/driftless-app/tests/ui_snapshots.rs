//! Picture tests of the three steps (`snapshot_daten`, `snapshot_bilder`, `snapshot_planung`):
//! the whole window at 1400 × 900 and scale 1 with a synthetic scene – a project of small
//! synthetic scans and the seeded synthetic cloud – compared with the references in
//! `tests/snapshots/`. Only on macOS (the references are made there; fonts and the GPU round
//! differently elsewhere), and only with a GPU adapter (else the test says it was skipped).
//!
//! The 3D view is checked by probes (the cloud drawn, the measurement line) and then painted
//! over, except its floating bars; the numbers of the last frame (the frame time changes from
//! run to run) are painted over too. What is compared is the layout around the view. New references after a wanted change of the layout:
//!
//! ```text
//! UPDATE_SNAPSHOTS=1 cargo test -p driftless-app --test ui_snapshots
//! ```
#![cfg(target_os = "macos")]

use driftless_app::render::headless_device;
use driftless_app::ui::tools::Measurement;
use driftless_app::ui::{Action, DriftlessApp, Step};
use eframe::egui;
use egui_kittest::kittest::{By, Queryable};
use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
use egui_kittest::{Harness, SnapshotOptions};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

const SIZE: [f32; 2] = [1400.0, 900.0];
/// Points of the synthetic cloud: enough to fill the view, quick to build.
const CLOUD_POINTS: usize = 100_000;
/// How much a pixel may differ (YIQ distance) before it counts, and how many may: room for
/// the rounding of text edges and of the points, not for a moved panel or button.
const THRESHOLD: f32 = 1.5;
const MAX_FAILED_PIXELS: usize = 400;

/// Says that the running test was skipped, and why, past the test harness's capture.
fn skipped(why: &str) {
    use std::io::Write;
    let name = std::thread::current().name().unwrap_or("?").to_string();
    let _ = writeln!(std::io::stderr(), "ÜBERSPRUNGEN {name}: {why}");
}

/// A fixed work folder for `name` (the same on every run, so that nothing shown depends on
/// the process), emptied.
fn work(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join("driftless-snapshots").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Ticks the app until its background work has ended.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(120);
    while app.wants_repaint_while_busy() {
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
        app.tick();
    }
}

/// The app with a GPU and the project „Schwarzer Raum“: two small synthetic scans
/// „Scan 13:35“ with a computed map and „Scan 13:30“ without, and the synthetic cloud in
/// the view. `None` without a GPU adapter.
fn scene(name: &str) -> Option<Harness<'static, DriftlessApp>> {
    if headless_device().is_none() {
        skipped("kein GPU-Adapter");
        return None;
    }
    let dir = work(name);
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Schwarzer Raum".into(),
    });
    let names = ["Scan 13:35", "Scan 13:30"];
    for name in names {
        let scan = dir.join(name.replace([' ', ':'], "-"));
        std::fs::create_dir_all(&scan).unwrap();
        driftless_io::testscan::room_bag_secs(&scan, true, 1.0);
        app.apply(Action::AddScan(scan));
    }
    settle(&mut app);
    let first = app.state().project.as_ref().unwrap().scans[0].id.clone();
    app.state_mut().map_settings.step = driftless_steps::map::MapStep::Clean;
    app.apply(Action::ComputeMap(first));
    settle(&mut app);
    // the shown names (folder names cannot hold „:“ everywhere)
    let mut project = app.state().project.clone().unwrap();
    for (scan, name) in project.scans.iter_mut().zip(names) {
        scan.name = name.into();
    }
    app.state_mut().set_project(project);
    // no message of the computation in the status bar: it holds a duration
    app.state_mut().log.clear();
    app.load_synthetic(CLOUD_POINTS);
    let mut h = Harness::builder()
        .with_size(SIZE)
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    let deadline = Instant::now() + Duration::from_secs(120);
    while h.state().is_loading() || h.state().cloud().is_none() || h.state().view().is_uploading() {
        h.step();
        assert!(Instant::now() < deadline, "cloud not loaded");
        std::thread::sleep(Duration::from_millis(10));
    }
    // no message of the load in the status bar: it holds its durations
    h.state_mut().state_mut().messages.clear();
    h.event(egui::Event::PointerGone);
    Some(h)
}

/// What the 3D view must show before it is painted over.
#[derive(Clone, Copy, PartialEq)]
enum Probe {
    /// The cloud: the view is not one flat background.
    Cloud,
    /// The cloud and a measurement line in its colour.
    Measurement,
}

/// The pixels of `rect` (points) in `image`, clipped to it.
fn pixels(
    image: &image::RgbaImage,
    rect: egui::Rect,
    ppp: f32,
) -> impl Iterator<Item = (u32, u32)> {
    let (x0, y0) = (
        (rect.min.x * ppp).max(0.0) as u32,
        (rect.min.y * ppp).max(0.0) as u32,
    );
    let x1 = ((rect.max.x * ppp) as u32).min(image.width());
    let y1 = ((rect.max.y * ppp) as u32).min(image.height());
    (y0..y1).flat_map(move |y| (x0..x1).map(move |x| (x, y)))
}

/// Renders the window, checks the 3D view with `probe`, then paints over the view (all but
/// the floating bars) and the numbers of the last frame, and compares the rest – rail, step
/// field, status bar and the bars – with the reference `name`. The view itself is the part
/// that a GPU or driver rounds differently, and noise there would bloat the references; the
/// render tests check its pixels.
fn check(h: &mut Harness<'_, DriftlessApp>, name: &str, probe: Probe) {
    use driftless_app::ui::theme::DARK;
    for _ in 0..4 {
        h.step();
    }
    let ppp = h.ctx.pixels_per_point();
    let frame_lines: Vec<egui::Rect> = h
        .query_all(By::new().label_contains(" ms "))
        .map(|node| node.rect().expand(2.0))
        .collect();
    assert!(!frame_lines.is_empty(), "no frame line in the info");
    let layout = *h.state().layout();
    let bars: Vec<egui::Rect> = [layout.info, layout.show, layout.tools, layout.cut]
        .into_iter()
        .flatten()
        .collect();
    let mut image = h.render().expect("render");

    // the probes: the cloud was drawn (a spread of brightness), the measurement line is there
    let luma: Vec<f32> = pixels(&image, layout.view, ppp)
        .map(|(x, y)| {
            let p = image.get_pixel(x, y).0;
            0.3 * f32::from(p[0]) + 0.59 * f32::from(p[1]) + 0.11 * f32::from(p[2])
        })
        .collect();
    let mean = luma.iter().sum::<f32>() / luma.len() as f32;
    let spread = (luma.iter().map(|l| (l - mean).powi(2)).sum::<f32>() / luma.len() as f32).sqrt();
    assert!(spread > 20.0, "the view looks empty (spread {spread:.1})");
    if probe == Probe::Measurement {
        let [r, g, b, _] = DARK.measure.to_array();
        let near = |p: [u8; 4]| {
            (i32::from(p[0]) - i32::from(r)).abs()
                + (i32::from(p[1]) - i32::from(g)).abs()
                + (i32::from(p[2]) - i32::from(b)).abs()
                < 30
        };
        let line = pixels(&image, layout.view, ppp)
            .filter(|&(x, y)| near(image.get_pixel(x, y).0))
            .count();
        assert!(
            line > 100,
            "no measurement line in the view ({line} pixels)"
        );
    }

    let fill = image::Rgba(DARK.bg.to_array());
    let in_bar = |x: u32, y: u32| {
        let p = egui::pos2(x as f32 / ppp, y as f32 / ppp);
        bars.iter().any(|b| b.contains(p))
    };
    let view: Vec<(u32, u32)> = pixels(&image, layout.view, ppp)
        .filter(|&(x, y)| !in_bar(x, y))
        .collect();
    for (x, y) in view {
        image.put_pixel(x, y, fill);
    }
    let mask = image::Rgba(DARK.surface_hi.to_array());
    for rect in frame_lines {
        let at: Vec<(u32, u32)> = pixels(&image, rect, ppp).collect();
        for (x, y) in at {
            image.put_pixel(x, y, mask);
        }
    }
    let options = SnapshotOptions::new()
        .threshold(THRESHOLD)
        .max_failed_pixels(MAX_FAILED_PIXELS);
    egui_kittest::image_snapshot_options(&image, name, &options);
}

#[test]
fn snapshot_daten() {
    let Some(mut h) = scene("daten") else {
        return;
    };
    check(&mut h, "snapshot_daten", Probe::Cloud);
}

#[test]
fn snapshot_bilder() {
    let Some(mut h) = scene("bilder") else {
        return;
    };
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    check(&mut h, "snapshot_bilder", Probe::Cloud);
}

#[test]
fn snapshot_planung() {
    let Some(mut h) = scene("planung") else {
        return;
    };
    h.state_mut().apply(Action::SetStep(Step::Planung));
    // two measurements across the cloud, in its own frame
    let origin = h.state().cloud().unwrap().tree.origin;
    let (min, max) = h.state().cloud().unwrap().points.bounds();
    let at = |fx: f32, fy: f32| {
        let p = [
            min[0] + (max[0] - min[0]) * fx,
            min[1] + (max[1] - min[1]) * fy,
            (min[2] + max[2]) / 2.0,
        ];
        std::array::from_fn(|k| origin[k] + f64::from(p[k]))
    };
    let measurements = &mut h.state_mut().state_mut().measurements;
    measurements.push(Measurement {
        a: at(0.2, 0.3),
        b: at(0.8, 0.3),
    });
    measurements.push(Measurement {
        a: at(0.5, 0.2),
        b: at(0.5, 0.8),
    });
    check(&mut h, "snapshot_planung", Probe::Measurement);
}
