//! The floating bars over the 3D view – the info at the top left, the switch Punkte · Fläche ·
//! beides at the top right, the tools at the bottom centre and the cut above them – measuring
//! between two points, and setting point pairs by hand („Von Hand ausrichten“).

use driftless_app::pick::{project, relative_to};
use driftless_app::project::{ModelInfo, Project, ScanEntry, ScanState};
use driftless_app::render::{headless_device, ShowMode};
use driftless_app::ui::tools::{self, Measurement, PairSession, Tool};
use driftless_app::ui::view3d::PointInfo;
use driftless_app::ui::{
    steps, text, Action, AppState, CloudSummary, DriftlessApp, ScanNode, Selection, Step,
    SurfaceSummary,
};
use driftless_io::testscan::tmp;
use driftless_map::{write_las, Cloud};
use driftless_steps::merge::{ModelMember, ModelReport, MODEL_LAS, MODEL_REPORT};
use driftless_steps::pairs::{stored_text, PointPairEntry, PointPairs, POINT_PAIRS_FILE};
use driftless_steps::slam::SlamSettings;
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use eframe::egui::{
    self,
    accesskit::{Role, Toggled},
};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// The tools of the bar, in order.
const TOOLS: [&str; 5] = [
    text::TOOL_ORBIT,
    text::TOOL_PAN,
    text::TOOL_PICK,
    text::TOOL_MEASURE,
    text::TOOL_CUT,
];

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

fn window(app: DriftlessApp, size: [f32; 2]) -> Harness<'static, DriftlessApp> {
    let mut h = Harness::builder()
        .with_size(size)
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run();
    h
}

fn cloud_summary() -> CloudSummary {
    CloudSummary {
        path: PathBuf::from("/k/color.las"),
        points: 6_000_000,
        has_rgb: true,
        has_scans: false,
        quality_known: false,
        z_range: [-0.5, 3.0],
        ceiling: Some(2.8),
        point_m: 0.015,
    }
}

fn surface_summary() -> SurfaceSummary {
    SurfaceSummary {
        path: PathBuf::from("oberflaeche.glb"),
        triangles: 712_218,
        pages: 12,
        origin: [0.0; 3],
        z_range: [0.0, 3.0],
    }
}

/// The labels of the tool buttons that are on.
fn tools_on(h: &Harness<'_, DriftlessApp>) -> Vec<&'static str> {
    TOOLS
        .into_iter()
        .filter(|label| {
            let node = h.get_by_role_and_label(Role::Button, label);
            node.accesskit_node().toggled() == Some(Toggled::True)
        })
        .collect()
}

fn click_button(h: &mut Harness<'_, DriftlessApp>, label: &str) {
    h.get_by_role_and_label(Role::Button, label).click();
    h.run();
}

#[test]
fn exactly_one_tool_active() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    let mut h = window(app, [1400.0, 900.0]);
    assert_eq!(h.state().state().tool, Tool::Orbit);
    assert_eq!(tools_on(&h), vec![text::TOOL_ORBIT]);
    click_button(&mut h, text::TOOL_MEASURE);
    assert_eq!(h.state().state().tool, Tool::Measure);
    assert_eq!(tools_on(&h), vec![text::TOOL_MEASURE]);
    click_button(&mut h, text::TOOL_CUT);
    assert_eq!(h.state().state().tool, Tool::Cut);
    assert_eq!(tools_on(&h), vec![text::TOOL_CUT]);
    // „Alles“ is a button, no tool: the tool stays
    let fit = h.get_by_role_and_label(Role::Button, text::FIT_ALL);
    assert_eq!(fit.accesskit_node().toggled(), None);
    click_button(&mut h, text::FIT_ALL);
    assert_eq!(h.state().state().tool, Tool::Cut);
    assert_eq!(tools_on(&h), vec![text::TOOL_CUT]);
}

#[test]
fn measurement_distance() {
    let m = Measurement {
        a: [512_000.0, 5_400_000.0, 100.0],
        b: [512_001.2, 5_400_001.6, 100.0],
    };
    assert!((m.distance() - 2.0).abs() < 1e-9);
    assert_eq!(text::meters(2.0), "2,00 m");
    assert_eq!(text::meters(12.345), "12,35 m");
}

/// A floor plate of 4 × 2 m with a point every 25 cm, as `color.las` in a folder of the
/// running test (tests run in parallel, and each LAS goes through its own `.tmp`): a click
/// hits exactly one point. Returns the LAS path.
fn plate() -> PathBuf {
    let mut xyz = Vec::new();
    for i in 0..=16 {
        for j in 0..=8 {
            xyz.push([500_000.0 + i as f32 * 0.25, 400.0 + j as f32 * 0.25, 10.0]);
        }
    }
    let n = xyz.len();
    let cloud = Cloud {
        xyz,
        intensity: vec![100.0; n],
        t: vec![0.0; n],
        frame: vec![0; n],
        rgb: Some(vec![[200, 200, 200]; n]),
        colored: Some(vec![true; n]),
    };
    let test = std::thread::current()
        .name()
        .unwrap_or("?")
        .replace(':', "_");
    let path = tmp(&format!("ui-tools-platte-{test}")).join("color.las");
    write_las(&path, &cloud).unwrap();
    path
}

/// Says that the running test was skipped, and why. Written past the test harness's capture
/// (straight to stderr, not with `eprintln!`), so that a run without a GPU – in CI, say –
/// shows which tests did not test anything instead of passing in silence.
fn skipped(why: &str) {
    use std::io::Write;
    let name = std::thread::current().name().unwrap_or("?").to_string();
    let _ = writeln!(std::io::stderr(), "ÜBERSPRUNGEN {name}: {why}");
}

/// The app with the plate loaded and drawn by a GPU, or `None` without an adapter.
fn plate_window() -> Option<Harness<'static, DriftlessApp>> {
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    if headless_device().is_none() {
        skipped("kein GPU-Adapter");
        return None;
    }
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = app().with_render_state(rs.clone());
    app.apply(Action::LoadCloud {
        path: plate(),
        run: None,
    });
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().is_loading() || h.state().cloud().is_none() || h.state().view().is_uploading() {
        h.step();
        assert!(Instant::now() < deadline, "cloud not loaded");
        std::thread::sleep(Duration::from_millis(10));
    }
    for _ in 0..3 {
        h.step();
    }
    Some(h)
}

/// Where the absolute point `p` lies on the screen (points).
fn on_screen(h: &Harness<'_, DriftlessApp>, p: [f64; 3]) -> egui::Pos2 {
    let app = h.state();
    let view = app.layout().view;
    let origin = app.cloud().unwrap().tree.origin;
    let vp = app.view().camera().view_proj(view.width() / view.height());
    let q = project(&vp, relative_to(origin, p), [view.width(), view.height()]).unwrap();
    view.min + egui::vec2(q[0], q[1])
}

/// A click at `pos` without moving.
fn click_at(h: &mut Harness<'_, DriftlessApp>, pos: egui::Pos2) {
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
    h.step();
}

const A: [f64; 3] = [500_000.5, 401.0, 10.0];
const B: [f64; 3] = [500_002.5, 401.0, 10.0];

#[test]
fn measure_two_points_gives_distance() {
    let Some(mut h) = plate_window() else {
        return;
    };
    click_button(&mut h, text::TOOL_MEASURE);
    let (a, b) = (on_screen(&h, A), on_screen(&h, B));
    click_at(&mut h, a);
    assert!(h.state().state().measurements.is_empty());
    click_at(&mut h, b);
    let measurements = &h.state().state().measurements;
    assert_eq!(measurements.len(), 1, "{measurements:?}");
    let d = measurements[0].distance();
    assert!((d - 2.0).abs() <= 0.02, "distance {d}");
    // in the view …
    h.get_by_label(&text::meters(2.0));
    // … and in the field „Planung“
    h.state_mut().apply(Action::SetStep(Step::Planung));
    h.run();
    h.get_by_label(&text::measurement_name(1));
    assert_eq!(h.query_all_by_label(&text::meters(2.0)).count(), 2);
    // a click with the tool „Punkt wählen“ shows the point, in any step
    click_button(&mut h, text::TOOL_PICK);
    click_at(&mut h, a);
    let picked = h.state().state().picked.clone().expect("picked point");
    for (got, want) in picked.position.iter().zip(A) {
        assert!((got - want).abs() < 0.01, "{picked:?}");
    }
    h.get_by_label(text::POINT);
    assert_eq!(h.state().state().measurements.len(), 1);
}

#[test]
fn escape_cancels_measure() {
    let Some(mut h) = plate_window() else {
        return;
    };
    click_button(&mut h, text::TOOL_MEASURE);
    let (a, b) = (on_screen(&h, A), on_screen(&h, B));
    click_at(&mut h, a);
    assert!(h.state().state().measure_a.is_some());
    h.key_press(egui::Key::Escape);
    h.step();
    assert!(h.state().state().measure_a.is_none());
    // the next click starts anew
    click_at(&mut h, b);
    assert!(h.state().state().measurements.is_empty());
    assert!(h.state().state().measure_a.is_some());
}

#[test]
fn cut_tool_shows_sliders() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    let mut h = window(app, [1400.0, 900.0]);
    assert!(h.query_by_label(text::CUT_MIN).is_none());
    // not in ⚙ any more either
    click_button(&mut h, text::SETTINGS);
    assert!(h.query_by_label(text::CUT_MIN).is_none());
    assert!(h.query_by_label(text::HIDE_CEILING).is_none());
    click_button(&mut h, text::SETTINGS);

    click_button(&mut h, text::TOOL_CUT);
    h.get_by_label(text::CUT_MIN);
    h.get_by_label(text::CUT_MAX);
    assert_eq!(h.state().state().view.cut, None);
    h.get_by_label(text::CUT_MIN).click();
    h.run();
    let cut = h.state().state().view.cut.expect("cut set by the slider");
    assert!(cut[0] > -0.5 && cut[0] <= cut[1], "{cut:?}");
    click_button(&mut h, text::HIDE_CEILING);
    assert_eq!(h.state().state().view.cut.map(|c| c[1]), Some(2.8));
    click_button(&mut h, text::CUT_RESET);
    assert_eq!(h.state().state().view.cut, None);
    // another tool: the cut stays, its bar goes
    click_button(&mut h, text::TOOL_ORBIT);
    assert!(h.query_by_label(text::CUT_MIN).is_none());
}

#[test]
fn info_and_show_switch() {
    // a cloud alone: its points, no switch
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    let mut h = window(app, [1400.0, 900.0]);
    h.get_by_label(&text::info_cloud(6_000_000));
    assert_eq!(text::info_cloud(6_000_000), "Punktwolke · 6,0 Mio. Punkte");
    assert!(h.query_by_label(text::SHOW_SURFACE).is_none());
    // and a surface: the switch at the top right, `O` too
    h.state_mut().state_mut().surface = Some(surface_summary());
    h.state_mut().state_mut().view.show = ShowMode::Surface;
    h.run();
    assert_eq!(text::info_surface(712_218), "Oberfläche · 712.218 Dreiecke");
    h.get_by_label(&text::info_surface(712_218));
    h.get_by_role_and_label(Role::Button, text::SHOW_CLOUD)
        .click();
    h.run();
    assert_eq!(h.state().state().view.show, ShowMode::Cloud);
    h.get_by_label(&text::info_cloud(6_000_000));
    h.key_press(egui::Key::O);
    h.run();
    assert_eq!(h.state().state().view.show, ShowMode::Both);
    // the switch is not in ⚙ any more
    click_button(&mut h, text::SETTINGS);
    assert_eq!(h.query_all_by_label(text::SHOW_BOTH).count(), 1);
    // the numbers of the last frame stand in the info
    use driftless_app::ui::ViewStats;
    h.state_mut().state_mut().view_stats = Some(ViewStats {
        drawn: 5_900_000,
        total: 6_000_000,
        triangles: None,
        frame_ms: 8.44,
        gpu: true,
    });
    h.run();
    let line = text::frame_line(Some(5_900_000), 8.44, true);
    assert_eq!(line, "gezeichnet 5,9 Mio. · 8,4 ms GPU");
    h.get_by_label(&line);
    assert_eq!(text::frame_line(None, 8.44, false), "8,4 ms je Bild");
}

fn point() -> PointInfo {
    PointInfo {
        index: 3,
        position: [512_345.678, 5_400_000.25, 97.5],
        t: 12.5,
        intensity: 77,
        rgb: Some([200, 120, 40]),
        quality: None,
        removed: None,
        scan: None,
    }
}

#[test]
fn picked_point_shows_in_every_step() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    app.state_mut().picked = Some(point());
    let mut h = window(app, [1400.0, 900.0]);
    for step in [Step::Daten, Step::Bilder, Step::Planung] {
        h.state_mut().apply(Action::SetStep(step));
        h.run();
        h.get_by_label(text::POINT);
        h.get_by_label("512.345,678 m");
    }
    click_button(&mut h, text::CLOSE_POINT);
    assert_eq!(h.state().state().picked, None);
    assert!(h.query_by_label(text::POINT).is_none());
}

/// Whether `a` and `b` share more than an edge.
fn overlap(a: egui::Rect, b: egui::Rect) -> bool {
    let x = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
    let y = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
    x > 0.5 && y > 0.5
}

#[test]
fn small_window_no_overlap() {
    let name: String = "Scan Halle Nord, Erdgeschoss, Achse A bis F, zweiter Durchgang mit Kamera"
        .chars()
        .chain(std::iter::repeat('x'))
        .take(80)
        .collect();
    assert_eq!(name.chars().count(), 80);
    let mut app = app();
    *app.state_mut() = AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: vec![ScanEntry {
                id: "s".into(),
                path: PathBuf::from("/daten/s"),
                name: name.clone(),
                extra_runs: Vec::new(),
                bags: Vec::new(),
            }],
        }),
        scans: vec![ScanNode {
            id: "s".into(),
            name: name.clone(),
            state: ScanState::Ready,
            no_color: None,
            runs: Vec::new(),
        }],
        selection: Selection::Scan("s".into()),
        cloud: Some(cloud_summary()),
        surface: Some(surface_summary()),
        picked: Some(point()),
        tool: Tool::Cut,
        ..AppState::default()
    };
    let h = window(app, [1024.0, 640.0]);
    let l = h.state().layout();
    let floats = [
        ("Info", l.info),
        ("Ansicht", l.show),
        ("Werkzeuge", l.tools),
        ("Schnitt", l.cut),
    ];
    let mut all = vec![("Leiste", l.rail), ("Schrittfeld", l.step)];
    for (what, rect) in floats {
        let rect = rect.unwrap_or_else(|| panic!("{what} not shown"));
        assert!(
            l.view.contains_rect(rect),
            "{what} {rect:?} outside {:?}",
            l.view
        );
        all.push((what, rect));
    }
    for (i, (a, ra)) in all.iter().enumerate() {
        for (b, rb) in &all[i + 1..] {
            assert!(!overlap(*ra, *rb), "{a} {ra:?} overlaps {b} {rb:?}");
        }
    }
    // the name is cut with „…“: narrower than written out, inside the field
    let label = h.get_by_label(&name).rect();
    let font = egui::TextStyle::Body.resolve(&h.ctx.global_style());
    let full = h.ctx.fonts_mut(|f| {
        f.layout_no_wrap(name.clone(), font, egui::Color32::WHITE)
            .size()
            .x
    });
    assert!(label.width() < full - 20.0, "{} vs {full}", label.width());
    assert!(label.max.x <= l.step.max.x, "{label:?} vs {:?}", l.step);
}

#[test]
fn surface_alone_shows_triangles() {
    use driftless_app::ui::ViewStats;
    let mut app = app();
    app.state_mut().surface = Some(surface_summary());
    app.state_mut().view_stats = Some(ViewStats {
        drawn: 0,
        total: 0,
        triangles: Some(712_218),
        frame_ms: 8.44,
        gpu: true,
    });
    let mut h = window(app, [1400.0, 900.0]);
    h.get_by_label(&text::info_surface(712_218));
    h.get_by_label(&text::frame_line(None, 8.44, true));
    // without a cloud only the surface can be shown
    let points = h.get_by_role_and_label(Role::Button, text::SHOW_CLOUD);
    assert!(points.accesskit_node().is_disabled());
    points.click();
    h.run();
    assert_eq!(h.state().state().view.show, ShowMode::default());
}

/// A drag with the left button from `from` by `by` (points), in small moves.
fn drag(h: &mut Harness<'_, DriftlessApp>, from: egui::Pos2, by: egui::Vec2) {
    let button = |pos, pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: egui::Modifiers::NONE,
    };
    h.event(egui::Event::PointerMoved(from));
    h.step();
    h.event(button(from, true));
    h.step();
    let mut pos = from;
    for _ in 0..10 {
        pos += by / 10.0;
        h.event(egui::Event::PointerMoved(pos));
        h.step();
    }
    h.event(button(pos, false));
    h.step();
    h.step();
}

#[test]
fn left_drag_orbits_or_pans_by_tool() {
    let Some(mut h) = plate_window() else {
        return;
    };
    let middle = h.state().layout().view.center();
    // „Drehen“: the left button turns the camera about its target, which stays
    assert_eq!(h.state().state().tool, Tool::Orbit);
    let before = h.state().view().camera().clone();
    drag(&mut h, middle, egui::vec2(120.0, 0.0));
    let after = h.state().view().camera().clone();
    assert!(
        (after.yaw - before.yaw).abs() > 0.05,
        "{before:?} → {after:?}"
    );
    assert!(
        (after.target - before.target).norm() < 1e-4,
        "{before:?} → {after:?}"
    );
    // „Verschieben“: the left button moves the target, the direction stays
    click_button(&mut h, text::TOOL_PAN);
    let before = h.state().view().camera().clone();
    drag(&mut h, middle, egui::vec2(120.0, 0.0));
    let after = h.state().view().camera().clone();
    assert!(
        (after.target - before.target).norm() > 0.05,
        "{before:?} → {after:?}"
    );
    assert_eq!(after.yaw, before.yaw);
    assert_eq!(after.pitch, before.pitch);
    // a drag is no click: nothing was picked or measured
    assert_eq!(h.state().state().picked, None);
}

/// The app with a small text field of its own in a window next to it: the text field takes
/// the keyboard when focused.
fn window_with_text_field(app: DriftlessApp) -> Harness<'static, DriftlessApp> {
    let mut note = String::new();
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(
            move |ui, app: &mut DriftlessApp| {
                app.show(ui);
                egui::Window::new("Notiz")
                    .default_pos([700.0, 300.0])
                    .show(ui.ctx(), |ui| ui.text_edit_singleline(&mut note));
            },
            app,
        );
    h.run();
    h
}

#[test]
fn keys_free_unless_a_text_field_has_focus() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    app.state_mut().surface = Some(surface_summary());
    app.state_mut().tool = Tool::Measure;
    app.state_mut().measure_a = Some(A);
    let mut h = window_with_text_field(app);
    let shown = h.state().state().view.show;
    // a text field with the keyboard: `O` and Esc are its
    h.get_by_role(Role::TextInput).focus();
    h.run();
    assert!(h.ctx.text_edit_focused());
    h.key_press(egui::Key::O);
    h.run();
    assert_eq!(h.state().state().view.show, shown);
    h.key_press(egui::Key::Escape);
    h.run();
    assert_eq!(h.state().state().measure_a, Some(A));
    // a button with the keyboard (after Tab, say) holds neither
    h.get_by_role_and_label(Role::Button, text::FIT_ALL).focus();
    h.run();
    assert!(h.ctx.memory(|m| m.focused()).is_some());
    assert!(!h.ctx.text_edit_focused());
    h.key_press(egui::Key::O);
    h.run();
    assert_eq!(h.state().state().view.show, shown.next());
    h.key_press(egui::Key::Escape);
    h.run();
    assert_eq!(h.state().state().measure_a, None);
    assert_eq!(h.state().state().tool, Tool::Measure);
}

#[test]
fn active_cut_shows_in_info() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    let mut h = window(app, [1400.0, 900.0]);
    let line = text::cut_active([-0.5, 2.8]);
    assert_eq!(line, "Schnitt aktiv: z -0,50 bis 2,80 m");
    assert!(h.query_by_label(&line).is_none());
    h.state_mut().state_mut().view.cut = Some([-0.5, 2.8]);
    h.run();
    h.get_by_label(&line);
    // with the tool „Schnitt“ its bar shows the cut instead
    click_button(&mut h, text::TOOL_CUT);
    assert!(h.query_by_label(&line).is_none());
    click_button(&mut h, text::TOOL_ORBIT);
    h.get_by_label(&line);
    click_button(&mut h, text::TOOL_CUT);
    click_button(&mut h, text::CUT_RESET);
    click_button(&mut h, text::TOOL_ORBIT);
    assert!(h.query_by_label(&line).is_none());
}

#[test]
fn picking_places_ends_outside_daten_and_with_point_tools() {
    use driftless_app::ui::tools;
    let mut app = app();
    app.state_mut().correct.picking = true;
    // leaving „Daten“ ends it
    app.apply(Action::SetStep(Step::Planung));
    assert!(!app.state().correct.picking);
    // in „Daten“, „Messen“ and „Punkt wählen“ end it; turning the view does not
    app.apply(Action::SetStep(Step::Daten));
    for (tool, ends) in [
        (Tool::Pan, false),
        (Tool::Measure, true),
        (Tool::Orbit, false),
        (Tool::Pick, true),
    ] {
        tools::select(app.state_mut(), Tool::Orbit);
        app.state_mut().correct.picking = true;
        tools::select(app.state_mut(), tool);
        assert_eq!(app.state().correct.picking, !ends, "{tool:?}");
    }
}

#[test]
fn many_measurements_scroll_in_planung() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    app.state_mut().step = Step::Planung;
    for i in 0..40 {
        app.state_mut().measurements.push(Measurement {
            a: A,
            b: [A[0] + 1.0 + f64::from(i), A[1], A[2]],
        });
    }
    let h = window(app, [1024.0, 640.0]);
    // „Messungen löschen“ stays in the field, the list scrolls above it
    let field = h.state().layout().step;
    let clear = h.get_by_label(text::CLEAR_MEASUREMENTS).rect();
    assert!(
        clear.max.y <= field.max.y.min(640.0),
        "{clear:?} below {field:?}"
    );
    h.get_by_label(&text::measurement_name(1));
}

#[test]
fn measuring_over_the_surface_says_it_uses_the_cloud() {
    let mut app = app();
    app.state_mut().cloud = Some(cloud_summary());
    app.state_mut().surface = Some(surface_summary());
    app.state_mut().view.show = ShowMode::Surface;
    app.state_mut().tool = Tool::Measure;
    let mut h = window(app, [1400.0, 900.0]);
    h.get_by_label(text::PICKS_ON_CLOUD);
    // with the points shown, or the view turned, it says nothing of it
    h.state_mut().state_mut().view.show = ShowMode::Both;
    h.run();
    assert!(h.query_by_label(text::PICKS_ON_CLOUD).is_none());
    h.state_mut().state_mut().view.show = ShowMode::Surface;
    h.state_mut().state_mut().tool = Tool::Orbit;
    h.run();
    assert!(h.query_by_label(text::PICKS_ON_CLOUD).is_none());
}

/// `T_1` of the model of the pair tests: turned by 90° about z, then moved.
fn t_second() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(2.0, 1.0, 0.5),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90f64.to_radians()),
    )
}

/// `pose` as `modell.json` stores it: 4 × 4, row by row.
fn rows(pose: &Isometry3<f64>) -> [f64; 16] {
    let h = pose.to_homogeneous();
    std::array::from_fn(|k| h[(k / 4, k % 4)])
}

/// Scan `nr` of a model, its map in `<root>/<name>/karte`.
fn model_member(root: &Path, name: &str, pose: &Isometry3<f64>, verdict: &str) -> ModelMember {
    let run = root.join(name);
    ModelMember {
        scan: PathBuf::from(format!("/daten/{name}")),
        scan_rel: None,
        slam_dir: run.clone(),
        slam_dir_rel: None,
        map_dir: run.join("karte"),
        map_dir_rel: None,
        slam: SlamSettings::default(),
        transform: rows(pose),
        placed_against: (verdict != "bezug").then_some(0),
        method: if verdict == "bezug" {
            "bezug"
        } else {
            "merkmale"
        }
        .into(),
        verdict: if verdict == "bezug" {
            "sicher"
        } else {
            verdict
        }
        .into(),
        reason: None,
        overlap: 0.5,
        residual_median_mm: 3.0,
        residual_rms_mm: 4.0,
        agreement: 0.9,
        wall_residual_mm: Some(3.0),
        pair_rms_mm: None,
        included: verdict != "unsicher",
        color_gain: [1.0; 3],
        points: 100,
        clean_id: None,
    }
}

/// „Modell 001“ in `dir` of three scans: a (the reference), b (sure, at [`t_second`]) and c
/// (unsure, left out).
fn pair_model(dir: &Path, root: &Path) -> ModelInfo {
    let members = vec![
        model_member(root, "a", &Isometry3::identity(), "bezug"),
        model_member(root, "b", &t_second(), "sicher"),
        model_member(root, "c", &Isometry3::identity(), "unsicher"),
    ];
    ModelInfo {
        dir: dir.to_path_buf(),
        label: "Modell 001".into(),
        report: ModelReport {
            version: 1,
            input_id: "0".into(),
            points: 200,
            members,
            seconds_total: 3.0,
            peak_rss_bytes: 0,
            include_unsure: false,
            warnings: Vec::new(),
        },
        surfaces: Vec::new(),
    }
}

/// The shift of the scan in the pair cloud of the tests.
const SHIFT: [f64; 3] = [7.5, 0.0, 0.0];

/// A project with one scan and [`pair_model`], pairs being set for scan c, its cloud shown.
fn pair_state() -> AppState {
    let model = pair_model(Path::new("/p/modelle/modell-001"), Path::new("/p/scans"));
    let mut session = PairSession::new(&model, 2, "3 · c".into()).unwrap();
    session.shift = Some(SHIFT);
    let mut cloud = cloud_summary();
    cloud.path = session.cloud.clone();
    cloud.has_scans = true;
    AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: vec![ScanEntry {
                id: "s".into(),
                path: PathBuf::from("/daten/s"),
                name: "s".into(),
                extra_runs: Vec::new(),
                bags: Vec::new(),
            }],
        }),
        scans: vec![ScanNode {
            id: "s".into(),
            name: "s".into(),
            state: ScanState::Ready,
            no_color: None,
            runs: Vec::new(),
        }],
        models: vec![model],
        cloud: Some(cloud),
        pairs: Some(session),
        tool: Tool::Pairs,
        ..AppState::default()
    }
}

/// A click of the tool on `position` (absolute, as shown), a point of scan `scan`.
fn hit(position: [f64; 3], scan: u16) -> PointInfo {
    PointInfo {
        position,
        scan: Some(scan),
        ..point()
    }
}

type Panel = Harness<'static, (AppState, Option<Action>)>;

/// The field „Daten“ of `state` at the width of the program's field, keeping the last action.
fn daten_panel(state: AppState) -> Panel {
    let mut h = Harness::builder()
        .with_size([250.0, 1200.0])
        .build_ui_state(
            |ui, (state, action): &mut (AppState, Option<Action>)| {
                if let Some(a) = steps::show(ui, state) {
                    *action = Some(a);
                }
            },
            (state, None),
        );
    h.run();
    h
}

fn locked(h: &Panel, label: &str) -> bool {
    h.get_by_label(label).accesskit_node().is_disabled()
}

#[test]
fn pair_tool_alternates_and_needs_three() {
    // in the window: the tool „Punktpaare“ is in the bar (and on), the info says what to click
    let mut app = app();
    *app.state_mut() = pair_state();
    let mut h = window(app, [1400.0, 900.0]);
    let pairs = h.get_by_role_and_label(Role::Button, text::TOOL_PAIRS);
    assert_eq!(pairs.accesskit_node().toggled(), Some(Toggled::True));
    assert!(h.query_all_by_label(&text::pair_next(1, true)).count() >= 1);
    // another tool: the session stays, the tool can be chosen again
    click_button(&mut h, text::TOOL_ORBIT);
    click_button(&mut h, text::TOOL_PAIRS);
    assert_eq!(h.state().state().tool, Tool::Pairs);
    // without a session the tool is not offered
    tools::end_pairs(h.state_mut().state_mut());
    h.run();
    assert!(h.query_by_label(text::TOOL_PAIRS).is_none());
    assert_eq!(h.state().state().tool, Tool::Orbit);

    let mut h = daten_panel(pair_state());
    h.get_by_label(&text::pairs_subject("3 · c", "Modell 001"));
    let click = |h: &mut Panel, p: [f64; 3], scan: u16| {
        tools::point_clicked(&mut h.state_mut().0, hit(p, scan));
        h.run();
    };
    let session = |h: &Panel| h.state().0.pairs.clone().unwrap();
    h.get_by_label(&text::pair_next(1, true));
    assert!(locked(&h, text::PAIRS_APPLY));
    // a point of the scan first: refused, the model is wanted
    click(&mut h, [10.0, 0.0, 0.0], 2);
    assert!(session(&h).in_scan.is_empty());
    h.get_by_label(text::PAIRS_WANTS_MODEL);
    click(&mut h, [0.0, 0.0, 0.0], 0);
    h.get_by_label(&text::pair_next(1, false));
    // now a point of the model is refused
    click(&mut h, [1.0, 0.0, 0.0], 1);
    h.get_by_label(text::PAIRS_WANTS_SCAN);
    assert_eq!(session(&h).in_model.len(), 1);
    click(&mut h, [7.5, 0.0, 0.0], 2);
    h.get_by_label(&text::pairs_set(1));
    h.get_by_label(&text::pair_next(2, true));
    for (m, q) in [
        ([1.0, 0.0, 0.0], [8.5, 0.0, 0.0]),
        ([0.0, 1.0, 0.0], [7.5, 1.0, 0.0]),
    ] {
        assert!(locked(&h, text::PAIRS_APPLY), "fewer than three pairs");
        click(&mut h, m, 0);
        click(&mut h, q, 2);
    }
    h.get_by_label(&text::pairs_set(3));
    assert!(!locked(&h, text::PAIRS_APPLY));
    // a place in the model without its place in the scan locks again; „zurück“ drops it
    click(&mut h, [0.0, 0.0, 1.0], 0);
    assert!(locked(&h, text::PAIRS_APPLY));
    h.get_by_label(text::PAIRS_UNDO).click();
    h.run();
    assert_eq!(session(&h).in_model.len(), 3);
    assert!(!locked(&h, text::PAIRS_APPLY));
    h.state_mut().1 = None;
    h.get_by_label(text::PAIRS_APPLY).click();
    h.run();
    let Some(Action::ApplyPairs { model, entry }) = h.state_mut().1.take() else {
        panic!("no ApplyPairs");
    };
    assert_eq!(model, PathBuf::from("/p/modelle/modell-001"));
    assert_eq!((entry.points_a.len(), entry.points_b.len()), (3, 3));
    assert!(entry.enabled);
    // „Abbrechen“ ends it
    h.get_by_label(text::PAIRS_CANCEL).click();
    h.run();
    assert!(h.state().0.pairs.is_none());
    assert!(h.query_by_label(text::PAIRS_APPLY).is_none());
}

#[test]
fn pair_points_mapped_back_to_member_frames() {
    let root = Path::new("/p/scans");
    let model = pair_model(Path::new("/p/modelle/modell-001"), root);
    let mut s = PairSession::new(&model, 2, "3 · c".into()).unwrap();
    assert!(
        PairSession::new(&model, 0, "1 · a".into()).is_none(),
        "not the reference"
    );
    assert!(PairSession::new(&model, 3, "4".into()).is_none());
    assert!(s.entry().is_err(), "not shown yet");
    s.shift = Some(SHIFT);
    // places on scan b in the model frame, and where b has them
    let in_b = [[0.5, 0.0, 0.0], [1.0, 2.0, 0.0], [0.0, 1.0, 1.5]];
    let in_model = in_b.map(|p| {
        let q = t_second() * Point3::from(p);
        [q.x, q.y, q.z]
    });
    // the same places in c (its own frame), shown shifted
    let in_c = [[3.0, 4.0, 0.25], [1.0, 1.0, 1.0], [2.0, 0.5, 2.0]];
    for k in 0..3 {
        s.click(&hit(in_model[k], 1));
        s.click(&hit([0, 1, 2].map(|i| in_c[k][i] + SHIFT[i]), 2));
    }
    let e: PointPairEntry = s.entry().unwrap();
    assert_eq!(e.a, root.join("b").join("karte"));
    assert_eq!(e.b, root.join("c").join("karte"));
    let close = |a: &[[f64; 3]], b: &[[f64; 3]]| {
        a.iter()
            .zip(b)
            .all(|(p, q)| (0..3).all(|k| (p[k] - q[k]).abs() < 1e-9))
    };
    assert!(close(&e.points_a, &in_b), "{:?}", e.points_a);
    assert!(close(&e.points_b, &in_c), "{:?}", e.points_b);
    assert!(e.added.is_empty() && e.enabled);
    assert_eq!(e.note, text::pairs_subject("3 · c", "Modell 001"));

    // most places on the reference: all of them in its frame (identity), the one on b too
    let mut s = PairSession::new(&model, 2, "3 · c".into()).unwrap();
    s.shift = Some(SHIFT);
    let places = [
        ([0.0, 0.0, 0.0], 0),
        ([4.0, 0.0, 0.0], 1),
        ([0.0, 3.0, 0.0], 0),
    ];
    for (p, scan) in places {
        s.click(&hit(p, scan));
        s.click(&hit([9.0, 9.0, 9.0], 2));
    }
    let e = s.entry().unwrap();
    assert_eq!(e.a, root.join("a").join("karte"));
    assert!(close(&e.points_a, &places.map(|(p, _)| p)));
    assert!(close(&e.points_b, &[[1.5, 9.0, 9.0]; 3]));
}

/// A project on disk with [`pair_model`] (its maps inside the project, `modell.las` with
/// points of a and b, a `clean.las` for c) and `punktpaare.toml` with two entries between a
/// and b, the second switched off; opened in the program.
fn pair_project(name: &str) -> (DriftlessApp, PathBuf) {
    let dir = tmp(name);
    let mut app = app();
    app.apply(Action::NewProject {
        dir: dir.join("projekt"),
        name: "Halle".into(),
    });
    let project = app.state().project.as_ref().unwrap().dir.clone();
    let model_dir = project.join("modelle").join("modell-001");
    let model = pair_model(&model_dir, &project.join("scans"));
    for m in &model.report.members {
        std::fs::create_dir_all(&m.map_dir).unwrap();
    }
    std::fs::create_dir_all(&model_dir).unwrap();
    let report = serde_json::to_string(&model.report).unwrap();
    std::fs::write(model_dir.join(MODEL_REPORT), report).unwrap();
    let n = 40;
    let cloud = |x0: f32| Cloud {
        xyz: (0..n)
            .map(|i| [x0 + (i % 10) as f32 * 0.1, (i / 10) as f32 * 0.1, 1.0])
            .collect(),
        intensity: vec![0.5; n],
        t: vec![0.0; n],
        frame: vec![0; n],
        rgb: None,
        colored: None,
    };
    let sources: Vec<u16> = (0..n).map(|i| (i % 2) as u16).collect();
    driftless_map::write_las_sources(&model_dir.join(MODEL_LAS), &cloud(0.0), &sources).unwrap();
    write_las(
        &model.report.members[2].map_dir.join("clean.las"),
        &cloud(-3.0),
    )
    .unwrap();
    let models = project.join("modelle");
    let rel =
        |k: usize| PathBuf::from(stored_text(&models, &model.report.members[k].map_dir).unwrap());
    let points = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut pairs = PointPairs::default();
    for enabled in [true, false] {
        pairs
            .add(PointPairEntry {
                a: rel(0),
                b: rel(1),
                points_a: points.clone(),
                points_b: points.clone(),
                enabled,
                note: "Ecken".into(),
                added: "2026-10-27T10:00:00Z".into(),
                unknown: Default::default(),
            })
            .unwrap();
    }
    pairs.save(&models.join(POINT_PAIRS_FILE)).unwrap();
    app.apply(Action::OpenProject(project.clone()));
    (app, project)
}

/// Ticks the program until its work has ended.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        app.tick();
        if !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn pair_toggle_saves() {
    let (mut app, project) = pair_project("app-pair-toggle");
    settle(&mut app);
    let state = app.state();
    assert_eq!(state.point_pairs.len(), 2, "{:?}", state.point_pairs_error);
    let model = state.models[0].clone();
    let listed: Vec<(usize, [usize; 2])> =
        state.pairs_of(&model).map(|(k, _, ab)| (k, ab)).collect();
    assert_eq!(listed, [(0, [0, 1]), (1, [0, 1])]);
    // the list under the model with a switch each (the field needs a scan to show)
    let mut shown = state.clone();
    shown.scans = pair_state().scans;
    let mut h = daten_panel(shown);
    h.get_by_label(text::PAIRS_LIST);
    h.get_by_label(&text::pair_row(1, 1, 2, 3));
    let switch = |h: &Panel, nr: usize| {
        h.get_by_role_and_label(Role::CheckBox, &text::pair_switch(nr))
            .accesskit_node()
            .toggled()
    };
    assert_eq!(switch(&h, 1), Some(Toggled::True));
    assert_eq!(switch(&h, 2), Some(Toggled::False));
    // under the unsure scan „Von Hand ausrichten“, not under the others
    assert_eq!(h.get_all_by_label(text::PAIRS_START).count(), 1);
    h.state_mut().1 = None;
    h.get_by_role_and_label(Role::CheckBox, &text::pair_switch(1))
        .click();
    h.run();
    let action = h.state_mut().1.take().unwrap();
    assert_eq!(
        action,
        Action::SetPairEnabled {
            index: 0,
            enabled: false
        }
    );
    // written by the helper: the entry stays, switched off; the other is untouched
    app.apply(action);
    assert!(app.state().pairs_writing);
    settle(&mut app);
    assert!(!app.state().pairs_writing);
    let file = project.join("modelle").join(POINT_PAIRS_FILE);
    let saved = PointPairs::load(&file).unwrap();
    assert_eq!(saved.entries.len(), 2);
    assert!(!saved.entries[0].enabled && !saved.entries[1].enabled);
    assert_eq!(saved.entries[0].note, "Ecken");
    let text = std::fs::read_to_string(&file).unwrap();
    assert!(text.contains("a = \"../scans/a/karte\""), "{text}");
    assert!(!app.state().point_pairs[0].enabled);
    assert!(app
        .state()
        .messages
        .iter()
        .any(|m| matches!(m, driftless_app::ui::UiMessage::Info(t) if *t == text::pair_switched(1, false))));
    // on again
    app.apply(Action::SetPairEnabled {
        index: 0,
        enabled: true,
    });
    settle(&mut app);
    assert!(PointPairs::load(&file).unwrap().entries[0].enabled);
}

#[test]
fn start_pairs_shows_model_and_scan() {
    let (mut app, _) = pair_project("app-pair-start");
    settle(&mut app);
    let model = app.state().models[0].dir.clone();
    app.state_mut().view.color_mode = driftless_app::render::ColorMode::Rgb;
    app.apply(Action::StartPairs {
        model: model.clone(),
        member: 2,
    });
    assert_eq!(app.state().tool, Tool::Pairs);
    settle(&mut app);
    let state = app.state();
    let session = state.pairs.as_ref().expect("pairs being set");
    let shift = session.shift.expect("shown");
    assert_eq!(
        state.view.color_mode,
        driftless_app::render::ColorMode::Scan
    );
    assert_eq!(state.selection, Selection::Model(model));
    let cloud = app.cloud().unwrap();
    assert_eq!(cloud.path, session.cloud);
    // a and b of the model, c beside it: 1 m beyond the model's end (x 0…0,9 m)
    assert_eq!(cloud.points.len(), 80);
    let c: Vec<usize> = (0..80).filter(|&i| cloud.points.source[i] == 2).collect();
    assert_eq!(c.len(), 40);
    let x = |i: usize| cloud.points.origin[0] + f64::from(cloud.points.xyz[i][0]);
    let start = c.iter().map(|&i| x(i)).fold(f64::INFINITY, f64::min);
    assert!((start - 1.9).abs() < 1e-3, "{start}");
    assert!((start - shift[0] - -3.0).abs() < 1e-3, "{shift:?}");
    // a scan without a cloud: the session ends with the reason
    app.apply(Action::StartPairs {
        model: app.state().models[0].dir.clone(),
        member: 1,
    });
    settle(&mut app);
    assert!(app.state().pairs.is_none());
    assert!(app.state().messages.iter().any(
        |m| matches!(m, driftless_app::ui::UiMessage::Error(t) if t.contains(text::PAIRS_NO_CLOUD))
    ));
    // another cloud ends it
    app.apply(Action::StartPairs {
        model: app.state().models[0].dir.clone(),
        member: 2,
    });
    settle(&mut app);
    assert!(app.state().pairs.as_ref().unwrap().shift.is_some());
    let las = app.state().models[0].dir.join(MODEL_LAS);
    app.apply(Action::Select(Selection::Cloud(las)));
    settle(&mut app);
    assert!(app.state().pairs.is_none());
    assert_eq!(app.state().tool, Tool::Orbit);
}
