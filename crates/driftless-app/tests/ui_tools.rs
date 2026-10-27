//! The floating bars over the 3D view – the info at the top left, the switch Punkte · Fläche ·
//! beides at the top right, the tools at the bottom centre and the cut above them – and
//! measuring between two points.

use driftless_app::pick::{project, relative_to};
use driftless_app::project::{Project, ScanEntry, ScanState};
use driftless_app::render::{headless_device, ShowMode};
use driftless_app::ui::tools::{Measurement, Tool};
use driftless_app::ui::view3d::PointInfo;
use driftless_app::ui::{
    text, Action, AppState, CloudSummary, DriftlessApp, ScanNode, Selection, Step, SurfaceSummary,
};
use driftless_io::testscan::tmp;
use driftless_map::{write_las, Cloud};
use eframe::egui::{
    self,
    accesskit::{Role, Toggled},
};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::path::PathBuf;
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

/// A floor plate of 4 × 2 m with a point every 25 cm, as `color.las` in its own folder: a
/// click hits exactly one point. Returns the LAS path.
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
    let path = tmp("ui-tools-platte").join("color.las");
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
