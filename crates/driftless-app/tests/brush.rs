//! The tool „Pinsel“ in the whole window: painting a Stelle on the surface of a map, erasing
//! with ⌥/Alt (from the Stelle under the pointer; an erasing stroke that takes nothing is
//! none), Shift and the right button still moving the view, taking back with ⌘Z (also after a
//! restart, and with any tool), Esc left to a question over the view, the list „Markierte
//! Stellen“ in „Bilder“ with „Löschen“, a damaged `stellen.toml` named, and the tool off without
//! a surface. The window draws with a GPU like `tests/ui_tools.rs` (without an adapter those
//! tests say they were skipped); the project is made by hand: a scan, a run with a map and a
//! surface that is a wall of 4 × 2.5 m. Without a window: how long a frame of painting into a
//! large Stelle takes (a measurement, ignored by default).

use driftless_app::pick::{project, relative_to};
use driftless_app::project::{
    canonical, MapInfo, Project, RunInfo, ScanEntry, ScanState, SurfaceInfo, SurfaceSettings,
};
use driftless_app::render::headless_device;
use driftless_app::render::mesh::mark_color;
use driftless_app::ui::brush::ACTIVE_TINT;
use driftless_app::ui::tools::Tool;
use driftless_app::ui::{
    steps, text, Action, AppState, CloudSummary, DriftlessApp, ScanNode, Step, SurfaceSummary,
    UiMessage,
};
use driftless_export::mesh::read_mesh;
use driftless_io::images::RgbImage;
use driftless_io::progress::Progress;
use driftless_io::testdir::TestDir;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_map::surface::edit::{select_faces, FaceBvh, SelectDab, WeldedSurface};
use driftless_map::surface::{encode_pages, write_glb, TexturePage, TexturedMesh};
use driftless_steps::map::MapSwitches;
use driftless_steps::marks::{Marks, MARKS_FILE};
use driftless_steps::slam::SlamSettings;
use eframe::egui::{self, accesskit::Role, Modifiers, PointerButton};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

/// The wall: x from 0 to 4 m, z from 0 to 2.5 m, at y = 1 m, in cells of 10 cm.
const WALL_Y: f64 = 1.0;
/// A stroke across the middle of the wall, 1.6 m long.
const FROM: [f64; 3] = [1.2, WALL_Y, 1.25];
const TO: [f64; 3] = [2.8, WALL_Y, 1.25];
/// A place on the wall far from the stroke.
const AWAY: [f64; 3] = [0.4, WALL_Y, 2.2];

/// Says that the running test was skipped, and why, past the test harness's capture.
fn skipped(why: &str) {
    use std::io::Write;
    let name = std::thread::current().name().unwrap_or("?").to_string();
    let _ = writeln!(std::io::stderr(), "ÜBERSPRUNGEN {name}: {why}");
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

/// A file `name` in `dir` (made with its folders) with a few bytes.
fn file(dir: &Path, name: &str) {
    fs::create_dir_all(dir).unwrap();
    fs::write(dir.join(name), b"{}").unwrap();
}

/// Writes the wall as a surface glb to `path`: one page, grey.
fn wall_glb(path: &Path) {
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    let (nx, nz) = (40u32, 25u32);
    let mut positions = Vec::new();
    let mut uvs = Vec::new();
    for k in 0..=nz {
        for i in 0..=nx {
            let (x, z) = (f64::from(i) * 0.1, f64::from(k) * 0.1);
            positions.push([x as f32, WALL_Y as f32, z as f32]);
            uvs.push([(x / 4.0) as f32, (1.0 - z / 2.5) as f32]);
        }
    }
    let at = |i: u32, k: u32| k * (nx + 1) + i;
    let mut indices = Vec::new();
    for k in 0..nz {
        for i in 0..nx {
            let (a, b, c, d) = (at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
            indices.push([a, b, c]);
            indices.push([a, c, d]);
        }
    }
    let mut tex = TexturedMesh::default();
    tex.pages = vec![TexturePage {
        positions,
        uvs,
        indices,
        image: RgbImage {
            width: 64,
            height: 64,
            data: vec![128; 64 * 64 * 3],
        },
    }];
    write_glb(path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
}

/// A project with the scan `halle` and a run `slam-001` with a map `karte-001` and the wall
/// as its surface `oberflaeche-001`, as complete folders would be (the reports here files of
/// a few bytes). Returns the project, the map and the surface folder.
fn wall_project(dir: &Path) -> (PathBuf, PathBuf, PathBuf) {
    let raw = dir.join("halle");
    fs::create_dir_all(&raw).unwrap();
    room_bag_secs(&raw, true, 1.0);
    let project = dir.join("projekt");
    let mut p = Project::create(&project, "Halle").unwrap();
    p.add_scan(&raw).unwrap();
    p.save().unwrap();
    let project = canonical(&project).unwrap();
    let run = project.join("scans/halle/slam-001");
    file(&run, "slam.json");
    file(&run, "traj.tum");
    let map = run.join("karte-001");
    file(&map, "map.json");
    let surface = map.join("oberflaeche-001");
    wall_glb(&surface.join("oberflaeche.glb"));
    file(&surface, "oberflaeche.json");
    (project, map, surface)
}

/// Frames until no step and no background work is left, without errors.
fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            break;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// The window with a GPU on the project in `project`, in „Bilder“, with the surface in
/// `surface` shown and framed; `None` without an adapter.
fn open(project: &Path, surface: &Path) -> Option<H> {
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    if headless_device().is_none() {
        skipped("kein GPU-Adapter");
        return None;
    }
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::OpenProject(project.to_path_buf()));
    // frames as a display makes them: two clicks in a row are a double click
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .with_pixels_per_point(1.0)
        .with_step_dt(1.0 / 60.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    settle(&mut h);
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.state_mut()
        .apply(Action::LoadSurface(surface.to_path_buf()));
    settle(&mut h);
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().state().surface.is_none() || h.state().view().is_uploading_surface() {
        h.step();
        assert!(Instant::now() < deadline, "surface not shown");
        std::thread::sleep(Duration::from_millis(5));
    }
    for _ in 0..3 {
        h.step();
    }
    Some(h)
}

/// A new project with the wall in a folder of the running test, and the window on it.
fn wall_window(name: &str) -> Option<(TestDir, PathBuf, PathBuf, H)> {
    let dir = tmp(name);
    let (project, map, surface) = wall_project(&dir);
    let h = open(&project, &surface)?;
    Some((dir, project, map, h))
}

/// Where the absolute point `p` lies on the screen (points; the scale is 1). Without a cloud
/// the view draws in the frame of the surface.
fn on_screen(h: &H, p: [f64; 3]) -> egui::Pos2 {
    let app = h.state();
    let view = app.layout().view;
    let origin = app.state().surface.as_ref().expect("surface").origin;
    let vp = app.view().camera().view_proj(view.width() / view.height());
    let q = project(&vp, relative_to(origin, p), [view.width(), view.height()]).unwrap();
    let pos = view.min + egui::vec2(q[0], q[1]);
    assert!(view.contains(pos), "{p:?} at {pos:?} outside {view:?}");
    pos
}

/// A drag with `button` from `from` to `to` (points) in small moves, `modifiers` held.
fn drag(h: &mut H, from: egui::Pos2, to: egui::Pos2, button: PointerButton, modifiers: Modifiers) {
    let press = |pos, pressed| egui::Event::PointerButton {
        pos,
        button,
        pressed,
        modifiers,
    };
    h.event(egui::Event::ModifiersChanged(modifiers));
    h.event(egui::Event::PointerMoved(from));
    h.step();
    h.event(press(from, true));
    h.step();
    for k in 1..=10 {
        h.event(egui::Event::PointerMoved(
            from + (to - from) * k as f32 / 10.0,
        ));
        h.step();
    }
    h.event(press(to, false));
    h.step();
    h.event(egui::Event::ModifiersChanged(Modifiers::NONE));
    h.step();
}

/// A stroke of the brush across the wall from `from` to `to` (absolute), with `modifiers`.
fn stroke(h: &mut H, from: [f64; 3], to: [f64; 3], modifiers: Modifiers) {
    let (a, b) = (on_screen(h, from), on_screen(h, to));
    drag(h, a, b, PointerButton::Primary, modifiers);
    settle(h);
}

/// Clicks the tool `name` in the bar.
fn choose(h: &mut H, name: &str) {
    h.get_by_role_and_label(Role::Button, name).click();
    h.step();
    h.step();
}

fn choose_brush(h: &mut H) {
    choose(h, text::TOOL_BRUSH);
    assert_eq!(h.state().state().tool, Tool::Brush);
}

/// The Stellen of the map in `map` as saved.
fn saved(map: &Path) -> Marks {
    Marks::load(map).unwrap()
}

/// How many faces of the wall the dabs of Stelle `nr` in `map` cover, as the touch-up selects
/// them.
fn covered(map: &Path, nr: u32) -> usize {
    let glb = map.join("oberflaeche-001/oberflaeche.glb");
    let mesh = read_mesh(&glb, &Progress::new()).unwrap();
    let s = WeldedSurface::from_mesh(&mesh);
    let bvh = FaceBvh::build(&s);
    let marks = saved(map);
    let mark = marks.marks.iter().find(|m| m.nr == nr).expect("mark");
    let dabs: Vec<SelectDab> = mark
        .dabs
        .iter()
        .map(|d| SelectDab {
            centre: d.centre,
            normal: d.normal,
            radius: f64::from(d.radius),
            erase: d.erase,
        })
        .collect();
    select_faces(&s, &bvh, &dabs).len()
}

/// The colour of the window at `pos` (points; the scale is 1), as rendered now.
fn pixel(h: &mut H, pos: egui::Pos2) -> [u8; 3] {
    let image = h.render().expect("render");
    let p = image
        .get_pixel(pos.x.round() as u32, pos.y.round() as u32)
        .0;
    [p[0], p[1], p[2]]
}

#[test]
fn paint_creates_mark_and_file() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-malen") else {
        return;
    };
    assert!(!map.join(MARKS_FILE).exists());
    let away = on_screen(&h, AWAY);
    let wall = pixel(&mut h, away);
    choose_brush(&mut h);
    // the info says what the left button does now
    h.get_by_label(&text::brush_hint(None));
    // the pointer over the wall: the ring of the brush lies on it, its middle stays free
    h.event(egui::Event::PointerMoved(away));
    h.step();
    h.step();
    let image = h.render().expect("render");
    let bright = |p: egui::Pos2| {
        let (x, y) = (p.x.round() as i64, p.y.round() as i64);
        (-1..=1).any(|dy| {
            (-1..=1).any(|dx| {
                let q = image.get_pixel((x + dx) as u32, (y + dy) as u32).0;
                q[..3].iter().all(|&c| c > 220)
            })
        })
    };
    let on_ring = (0..16)
        .filter(|k| {
            let t = std::f64::consts::TAU * f64::from(*k) / 16.0;
            let p = [AWAY[0] + 0.2 * t.cos(), WALL_Y, AWAY[2] + 0.2 * t.sin()];
            bright(on_screen(&h, p))
        })
        .count();
    assert!(on_ring >= 14, "ring at {on_ring} of 16 places");
    let centre = image.get_pixel(away.x as u32, away.y as u32).0;
    assert!(centre[..3].iter().all(|&c| c < 200), "{centre:?}");
    stroke(&mut h, FROM, TO, Modifiers::NONE);

    // one Stelle with all dabs of the stroke, saved in the folder of the map
    let marks = saved(&map);
    assert_eq!(marks.marks.len(), 1, "{marks:?}");
    let mark = &marks.marks[0];
    assert!(mark.dabs.len() >= 3, "{} dabs", mark.dabs.len());
    assert!(mark
        .dabs
        .iter()
        .all(|d| !d.erase && d.seq == mark.dabs[0].seq));
    assert!(mark.dabs.iter().all(|d| (d.radius - 0.2).abs() < 1e-6));
    // a dab every quarter of the radius along the way: 1.6 m / 5 cm
    let n = mark.dabs.len();
    assert!((25..=45).contains(&n), "{n} dabs");
    for d in &mark.dabs {
        assert!((d.centre[1] - WALL_Y).abs() < 1e-6, "{d:?}");
        assert!(d.normal[1].abs() > 0.99, "{d:?}");
    }
    // it is listed in „Bilder“ and painted into from now on
    h.get_by_label("Stelle 1 · Wand");
    h.get_by_label(text::MARKS_LIST);
    assert_eq!(h.state().state().brush.active, Some(1));
    h.get_by_label(&text::brush_hint(Some(1)));
    // and marked in the view: tinted on the stroke, the wall as it was away from it
    for _ in 0..3 {
        h.step();
    }
    let middle = on_screen(&h, [2.0, WALL_Y, 1.25]);
    let marked = pixel(&mut h, middle);
    // the colour of Stelle 1 over the grey wall, chosen: stronger than the others
    let [r, g, b, _] = mark_color(1);
    let a = f32::from(ACTIVE_TINT) / 255.0;
    for (k, c) in [r, g, b].into_iter().enumerate() {
        let want = a * f32::from(c) + (1.0 - a) * f32::from(wall[k]);
        assert!(
            (f32::from(marked[k]) - want).abs() <= 4.0,
            "not marked: {marked:?} vs wall {wall:?}"
        );
    }
    let now = pixel(&mut h, away);
    for k in 0..3 {
        assert!(now[k].abs_diff(wall[k]) <= 2, "{now:?} vs {wall:?}");
    }
    // a second stroke paints into the same Stelle
    stroke(
        &mut h,
        [1.2, WALL_Y, 0.6],
        [2.0, WALL_Y, 0.6],
        Modifiers::NONE,
    );
    let marks = saved(&map);
    assert_eq!(marks.marks.len(), 1, "{marks:?}");
    assert!(marks.marks[0].dabs.len() > n);
    // Esc chooses no Stelle: the next stroke begins a new one
    h.key_press(egui::Key::Escape);
    h.step();
    assert_eq!(h.state().state().brush.active, None);
    stroke(
        &mut h,
        [1.2, WALL_Y, 2.0],
        [2.0, WALL_Y, 2.0],
        Modifiers::NONE,
    );
    let marks = saved(&map);
    assert_eq!(marks.marks.len(), 2, "{marks:?}");
    h.get_by_label("Stelle 2 · Wand");
    assert_eq!(h.state().state().brush.active, Some(2));
}

#[test]
fn alt_erases() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-radieren") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    let before = covered(&map, 1);
    assert!(before > 50, "{before} faces");
    // ⌥/Alt over the first half takes those faces out again, as a stroke of its own
    stroke(&mut h, FROM, [2.0, WALL_Y, 1.25], Modifiers::ALT);
    let marks = saved(&map);
    assert_eq!(marks.marks.len(), 1, "{marks:?}");
    let mark = &marks.marks[0];
    let erasing: Vec<_> = mark.dabs.iter().filter(|d| d.erase).collect();
    assert!(erasing.len() >= 3, "{} erasing dabs", erasing.len());
    assert!(erasing.iter().all(|d| d.seq > mark.dabs[0].seq));
    let after = covered(&map, 1);
    assert!(
        after < before * 3 / 4 && after > 0,
        "{before} → {after} faces"
    );
    // without a Stelle chosen, erasing where none is begins nothing
    h.key_press(egui::Key::Escape);
    h.step();
    stroke(&mut h, AWAY, [0.8, WALL_Y, 2.2], Modifiers::ALT);
    assert_eq!(saved(&map), marks);
}

#[test]
fn shift_pans_not_paints() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-shift") else {
        return;
    };
    choose_brush(&mut h);
    let before = h.state().view().camera().clone();
    let (a, b) = (on_screen(&h, FROM), on_screen(&h, TO));
    drag(&mut h, a, b, PointerButton::Primary, Modifiers::SHIFT);
    settle(&mut h);
    let after = h.state().view().camera().clone();
    assert!(
        (after.target - before.target).norm() > 0.05,
        "{before:?} → {after:?}"
    );
    assert_eq!((after.yaw, after.pitch), (before.yaw, before.pitch));
    assert!(saved(&map).marks.is_empty());
    // the middle button pans as well
    let before = after;
    let middle = h.state().layout().view.center();
    drag(
        &mut h,
        middle,
        middle + egui::vec2(0.0, 80.0),
        PointerButton::Middle,
        Modifiers::NONE,
    );
    let after = h.state().view().camera().clone();
    assert!((after.target - before.target).norm() > 0.05);
    assert_eq!(after.yaw, before.yaw);
    assert!(saved(&map).marks.is_empty());
}

#[test]
fn right_drag_orbits() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-rechts") else {
        return;
    };
    choose_brush(&mut h);
    let before = h.state().view().camera().clone();
    let middle = h.state().layout().view.center();
    drag(
        &mut h,
        middle,
        middle + egui::vec2(120.0, 0.0),
        PointerButton::Secondary,
        Modifiers::NONE,
    );
    settle(&mut h);
    let after = h.state().view().camera().clone();
    assert!(
        (after.yaw - before.yaw).abs() > 0.05,
        "{before:?} → {after:?}"
    );
    assert!(
        (after.target - before.target).norm() < 1e-4,
        "{before:?} → {after:?}"
    );
    assert!(saved(&map).marks.is_empty());
    // a double click into the empty frames nothing while the brush is chosen (it paints);
    // the view moved first, so that framing everything would show
    drag(
        &mut h,
        middle,
        middle + egui::vec2(0.0, 150.0),
        PointerButton::Middle,
        Modifiers::NONE,
    );
    let corner = h.state().layout().view.left_bottom() + egui::vec2(40.0, -60.0);
    let camera = h.state().view().camera().clone();
    double_click(&mut h, corner);
    assert_eq!(*h.state().view().camera(), camera);
    // with another tool the same double click frames everything again
    choose(&mut h, text::TOOL_ORBIT);
    double_click(&mut h, corner);
    assert_ne!(*h.state().view().camera(), camera);
}

/// A double click at `pos`, once earlier clicks are long enough ago not to make it a triple
/// click (the frames are 1/60 s).
fn double_click(h: &mut H, pos: egui::Pos2) {
    h.run_steps(60);
    h.event(egui::Event::PointerMoved(pos));
    h.step();
    for pressed in [true, false, true, false] {
        h.event(egui::Event::PointerButton {
            pos,
            button: PointerButton::Primary,
            pressed,
            modifiers: Modifiers::NONE,
        });
        h.step();
    }
    h.run_steps(3);
}

fn undo_keys(h: &mut H) {
    h.key_press_modifiers(Modifiers::COMMAND, egui::Key::Z);
    h.step();
    settle(h);
}

#[test]
fn undo_and_restart() {
    let Some((dir, project, map, mut h)) = wall_window("pinsel-zurueck") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    assert_eq!(saved(&map).marks.len(), 1);
    undo_keys(&mut h);
    assert!(saved(&map).marks.is_empty(), "{:?}", saved(&map));
    assert!(h.query_by_label("Stelle 1 · Wand").is_none());
    assert_eq!(h.state().state().brush.active, None);

    // a stroke, the app closed and started again: ⌘Z takes it back
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    assert_eq!(saved(&map).marks.len(), 1);
    drop(h);
    let surface = map.join("oberflaeche-001");
    let mut h = open(&project, &surface).expect("GPU");
    h.get_by_label("Stelle 2 · Wand");
    choose_brush(&mut h);
    undo_keys(&mut h);
    assert!(saved(&map).marks.is_empty(), "{:?}", saved(&map));
    assert!(h.query_by_label("Stelle 2 · Wand").is_none());
    // ↶ in the bar is the same, with nothing left to take back it is off
    let undo = h.get_by_role_and_label(Role::Button, text::BRUSH_UNDO);
    assert!(undo.accesskit_node().is_disabled());
    drop(dir);
}

#[test]
fn delete_from_list() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-loeschen") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    h.key_press(egui::Key::Escape);
    h.step();
    stroke(
        &mut h,
        [1.2, WALL_Y, 2.0],
        [2.0, WALL_Y, 2.0],
        Modifiers::NONE,
    );
    assert_eq!(saved(&map).marks.len(), 2);
    // a click shows the Stelle and chooses it
    let camera = h.state().view().camera().clone();
    h.get_by_label("Stelle 1 · Wand").click();
    for _ in 0..3 {
        h.step();
    }
    assert_eq!(h.state().state().brush.active, Some(1));
    let framed = h.state().view().camera().clone();
    assert_ne!(framed, camera);
    // looking straight at the wall (along +y), the Stelle in the middle
    let dir = framed.dir();
    assert!(dir.y > 0.99, "{dir:?}");
    // right click „Löschen“: gone from the file and the list, the other one stays
    h.get_by_label("Stelle 1 · Wand").click_secondary();
    for _ in 0..4 {
        h.step();
    }
    h.get_by_label(text::MARK_DELETE).click();
    for _ in 0..4 {
        h.step();
    }
    settle(&mut h);
    let marks = saved(&map);
    assert_eq!(
        marks.marks.iter().map(|m| m.nr).collect::<Vec<_>>(),
        vec![2]
    );
    assert!(h.query_by_label("Stelle 1 · Wand").is_none());
    h.get_by_label("Stelle 2 · Wand");
    assert_eq!(h.state().state().brush.active, None);
}

/// ⌘Z takes the Stellen back with any tool, not only with the brush: „Löschen“ in the list
/// works with any tool too.
#[test]
fn undo_with_any_tool() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-zurueck-werkzeug") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    assert_eq!(saved(&map).marks.len(), 1);
    choose(&mut h, text::TOOL_ORBIT);
    undo_keys(&mut h);
    assert!(saved(&map).marks.is_empty(), "{:?}", saved(&map));
    assert!(h.query_by_label("Stelle 1 · Wand").is_none());
}

/// ⌥ over another Stelle than the chosen one erases from the one under the pointer (and
/// chooses it); an erasing stroke that takes nothing is dropped, so ⌘Z never takes back a
/// stroke that did nothing.
#[test]
fn erase_where_the_pointer_is() {
    let Some((_dir, _project, map, mut h)) = wall_window("pinsel-radieren-andere") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    h.key_press(egui::Key::Escape);
    h.step();
    stroke(
        &mut h,
        [1.2, WALL_Y, 2.0],
        [2.0, WALL_Y, 2.0],
        Modifiers::NONE,
    );
    assert_eq!(h.state().state().brush.active, Some(2));
    let (first, second) = (covered(&map, 1), saved(&map).marks[1].clone());
    // over Stelle 1 while Stelle 2 is chosen
    stroke(&mut h, FROM, [2.0, WALL_Y, 1.25], Modifiers::ALT);
    let marks = saved(&map);
    assert_eq!(marks.marks[1], second, "Stelle 2 unchanged");
    let after = covered(&map, 1);
    assert!(
        after < first * 3 / 4 && after > 0,
        "{first} → {after} faces"
    );
    assert_eq!(h.state().state().brush.active, Some(1));
    // where no Stelle is, with Stelle 1 chosen: nothing taken, nothing saved
    stroke(
        &mut h,
        [0.3, WALL_Y, 0.4],
        [0.6, WALL_Y, 0.4],
        Modifiers::ALT,
    );
    assert_eq!(saved(&map), marks);
    // ⌘Z takes back the erasing over Stelle 1, the last stroke that did something
    undo_keys(&mut h);
    assert_eq!(covered(&map, 1), first);
}

/// Esc meant for another window – a question over the view – leaves the chosen Stelle alone;
/// Esc of its own chooses none.
#[test]
fn esc_for_a_question_keeps_the_stelle() {
    let Some((_dir, _project, _map, mut h)) = wall_window("pinsel-esc-frage") else {
        return;
    };
    choose_brush(&mut h);
    stroke(&mut h, FROM, TO, Modifiers::NONE);
    assert_eq!(h.state().state().brush.active, Some(1));
    h.state_mut().state_mut().ask = Some(driftless_app::ui::Ask::RemoveScan { id: "halle".into() });
    h.step();
    h.step();
    h.key_press(egui::Key::Escape);
    h.step();
    assert_eq!(h.state().state().ask, None);
    assert_eq!(h.state().state().brush.active, Some(1));
    h.key_press(egui::Key::Escape);
    h.step();
    assert_eq!(h.state().state().brush.active, None);
}

/// A `stellen.toml` that cannot be read says where, in plain words, as the other files of the
/// project do ([`text::damaged_toml`]), not with the reader's own text.
#[test]
fn damaged_marks_file_says_where() {
    let dir = tmp("pinsel-kaputt");
    let file = dir.join(MARKS_FILE);
    let content = "[[stelle]]\nnr = \"eins\"\n";
    fs::write(&file, content).unwrap();
    let e = toml::from_str::<Marks>(content).unwrap_err();
    let read = driftless_app::ui::brush::read_marks([dir.to_path_buf()]);
    assert_eq!(
        read.get(&dir.to_path_buf()),
        Some(&Err(text::damaged_toml(&file, content, &e)))
    );
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

const MAP: &str = "/p/scans/s/slam-001/karte-001";

/// A project (not read from disk) with one map and its surface, in „Bilder“.
fn state_with_map() -> AppState {
    let surface = SurfaceInfo {
        dir: PathBuf::from(format!("{MAP}/oberflaeche-001")),
        label: "Oberfläche 001".into(),
        stale: false,
        settings: Some(SurfaceSettings::default()),
        hints: Vec::new(),
    };
    let map = MapInfo {
        dir: PathBuf::from(MAP),
        label: "Karte 001".into(),
        settings: Some(MapSwitches::default()),
        clouds: Vec::new(),
        surfaces: vec![surface],
        incomplete: false,
    };
    let run = RunInfo {
        dir: PathBuf::from("/p/scans/s/slam-001"),
        label: "SLAM 001".into(),
        settings: SlamSettings::default(),
        has_result: true,
        has_cache: false,
        has_previous: false,
        has_odometry: false,
        cloud: None,
        maps: vec![map],
        history: Vec::new(),
        incomplete: false,
    };
    AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: vec![ScanEntry {
                id: "s".into(),
                path: PathBuf::from("/daten/s"),
                name: "Scan s".into(),
                extra_runs: Vec::new(),
                bags: Vec::new(),
            }],
        }),
        scans: vec![ScanNode {
            id: "s".into(),
            name: "Scan s".into(),
            state: ScanState::Ready,
            no_color: None,
            runs: vec![run],
        }],
        step: Step::Bilder,
        ..AppState::default()
    }
}

fn surface_summary(path: &str) -> SurfaceSummary {
    SurfaceSummary {
        path: PathBuf::from(path),
        triangles: 2000,
        pages: 1,
        origin: [2.0, 1.0, 1.0],
        z_range: [-1.0, 1.5],
    }
}

fn window(state: AppState) -> H {
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    *app.state_mut() = state;
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run();
    h
}

/// Hovers the button „Pinsel“ (off) and finds `hint` in its tooltip; a click chooses nothing.
fn brush_off_with(h: &mut H, hint: &str) {
    let button = h.get_by_role_and_label(Role::Button, text::TOOL_BRUSH);
    assert!(button.accesskit_node().is_disabled(), "{hint}");
    button.hover();
    h.run();
    h.get_by_label(hint);
    h.get_by_role_and_label(Role::Button, text::TOOL_BRUSH)
        .click();
    h.run();
    assert_eq!(h.state().state().tool, Tool::Orbit);
}

#[test]
fn brush_off_without_surface() {
    // a cloud alone: nothing to paint on
    let mut state = state_with_map();
    state.cloud = Some(cloud_summary());
    let mut h = window(state);
    brush_off_with(&mut h, text::BRUSH_NEEDS_SURFACE);
    // a surface that is no surface of a map or model of the project (a file opened)
    h.state_mut().state_mut().surface = Some(surface_summary("/irgendwo/wand.glb"));
    h.run();
    brush_off_with(&mut h, text::BRUSH_NOT_IN_PROJECT);
    // the surface of the map, but only the points shown
    let glb = format!("{MAP}/oberflaeche-001/oberflaeche.glb");
    h.state_mut().state_mut().surface = Some(surface_summary(&glb));
    h.state_mut().state_mut().view.show = driftless_app::render::ShowMode::Cloud;
    h.run();
    brush_off_with(&mut h, text::BRUSH_NEEDS_SHOWN);
    // shown: the tool can be chosen; its bar holds the size and ↶
    h.state_mut().state_mut().view.show = driftless_app::render::ShowMode::Surface;
    h.run();
    let button = h.get_by_role_and_label(Role::Button, text::TOOL_BRUSH);
    assert!(!button.accesskit_node().is_disabled());
    button.click();
    h.run();
    assert_eq!(h.state().state().tool, Tool::Brush);
    h.get_by_label(text::BRUSH_SIZE);
    assert_eq!(h.state().state().brush.radius, 0.2);
    assert_eq!(text::centimetres(0.2), "20 cm");
    h.get_by_role_and_label(Role::Button, text::BRUSH_UNDO);
    // the surface goes: the tool goes back to „Drehen“
    h.state_mut().state_mut().surface = None;
    h.run();
    assert_eq!(h.state().state().tool, Tool::Orbit);
    assert!(h.query_by_label(text::BRUSH_SIZE).is_none());
}

/// The panel „Bilder“ alone, with the Stellen of the map whose surface is shown.
fn panel(state: AppState) -> Harness<'static, (AppState, Option<Action>)> {
    let mut h = Harness::builder().with_size([320.0, 700.0]).build_ui_state(
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

#[test]
fn list_of_marks() {
    use driftless_steps::marks::{DabInput, SmoothLevel, StepInput};
    let dab = |normal: [f32; 3]| DabInput {
        centre: [1.0, 1.0, 1.0],
        normal,
        radius: 0.2,
        erase: false,
    };
    let mut marks = Marks::default();
    let wall = marks.paint(None, &[dab([0.0, -1.0, 0.0])]);
    let floor = marks.paint(None, &[dab([0.0, 0.0, 1.0])]);
    marks
        .add_step(
            wall,
            StepInput::Smooth {
                level: SmoothLevel::Strong,
            },
        )
        .unwrap();
    let mut state = state_with_map();
    state.marks.insert(PathBuf::from(MAP), Ok(marks.clone()));
    // without the surface of the map shown, no list
    let h = panel(state.clone());
    assert!(h.query_by_label(text::MARKS_LIST).is_none());
    let glb = format!("{MAP}/oberflaeche-001/oberflaeche.glb");
    state.surface = Some(surface_summary(&glb));
    let mut h = panel(state);
    h.get_by_label(text::MARKS_LIST);
    h.get_by_label("Stelle 1 · Wand");
    h.get_by_label("Stelle 2 · Boden");
    // the last step below a Stelle
    h.get_by_label(&text::mark_step(&marks.marks[0].steps[0]));
    assert_eq!(
        text::mark_step(&marks.marks[0].steps[0]),
        "geglättet „stark“"
    );
    // a click shows it, on the chosen one again: none chosen
    h.get_by_label("Stelle 2 · Boden").click();
    h.run();
    assert_eq!(h.state().1, Some(Action::FrameMark(floor)));
    h.state_mut().0.brush.active = Some(floor);
    h.state_mut().1 = None;
    h.run();
    h.get_by_label("Stelle 2 · Boden").click();
    h.run();
    assert_eq!(h.state().1, Some(Action::SelectMark(None)));
    // right click „Löschen“
    h.state_mut().1 = None;
    h.get_by_label("Stelle 1 · Wand").click_secondary();
    h.run();
    h.get_by_label(text::MARK_DELETE).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::DeleteMark(wall)));
    // a file that cannot be read says why
    let mut state = h.state().0.clone();
    state.marks.insert(
        PathBuf::from(MAP),
        Err("stellen.toml nicht lesbar: kaputt".into()),
    );
    let h = panel(state);
    h.get_by_label("stellen.toml nicht lesbar: kaputt");
}

/// A floor of `w` × `d` m at z = 0 in square cells of `cell` m, two triangles each: one page
/// without an image.
fn floor_mesh(w: f64, d: f64, cell: f64) -> driftless_export::mesh::Mesh {
    let (nx, ny) = ((w / cell).round() as u32, (d / cell).round() as u32);
    let mut positions = Vec::new();
    for j in 0..=ny {
        for i in 0..=nx {
            positions.push([f64::from(i) * cell, f64::from(j) * cell, 0.0]);
        }
    }
    let at = |i: u32, j: u32| j * (nx + 1) + i;
    let mut indices = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let (a, b, c, e) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            indices.push([a, b, c]);
            indices.push([a, c, e]);
        }
    }
    driftless_export::mesh::Mesh {
        pages: vec![driftless_export::mesh::MeshPage {
            positions,
            uvs: Vec::new(),
            indices,
            image: None,
            color: [128, 128, 128],
        }],
    }
}

/// Painting into a Stelle of 30 m² on a floor as dense as the surface of the Büro (3,200 faces
/// per m²: cells of 2.5 cm), with the brush of 20 cm and the rows of the Stelle 35 cm apart, as
/// R3 measured it: a frame that lays dabs takes less than 10 ms (the brush and the marks for the
/// view; with a GPU also writing them to it). The release selects the whole Stelle anew.
#[test]
#[ignore = "Messung, im Release: cargo test --release -p driftless-app --test brush -- --ignored timing_paint_large_stelle --nocapture"]
fn timing_paint_large_stelle() {
    use driftless_app::camera::Camera;
    use driftless_app::surface::SurfacePick;
    use driftless_app::ui::brush::Brush;
    use driftless_app::ui::view3d::BrushInput;
    use driftless_steps::marks::DabInput;
    let mesh = floor_mesh(6.0, 6.0, 0.025);
    let faces: usize = mesh.pages[0].indices.len();
    let pick = Arc::new(SurfacePick::new(&mesh));
    let origin = [3.0, 3.0, 0.0];
    let glb = PathBuf::from(format!("{MAP}/oberflaeche-001/oberflaeche.glb"));
    let mut state = state_with_map();
    state.surface = Some(SurfaceSummary {
        path: glb.clone(),
        triangles: faces,
        pages: 1,
        origin,
        z_range: [0.0, 0.0],
    });
    state.tool = Tool::Brush;
    // the Stelle: 16 rows 35 cm apart over 5.5 × 5.5 m, a dab every 5 cm as the brush lays them
    let mut marks = Marks::default();
    let mut nr = None;
    for row in 0..16 {
        let y = 0.25 + 0.35 * f64::from(row);
        let dabs: Vec<DabInput> = (0..=110)
            .map(|i| DabInput {
                centre: [0.25 + 0.05 * f64::from(i), y, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: 0.2,
                erase: false,
            })
            .collect();
        nr = Some(marks.paint(nr, &dabs));
    }
    let dabs: usize = marks.marks[0].dabs.len();
    state.marks.insert(PathBuf::from(MAP), Ok(marks));
    state.brush.active = nr;
    let mut brush = Brush::default();
    brush.set_surface(Some((glb, origin, pick)));
    let t = Instant::now();
    let marks = brush.sync(&mut state).expect("the marks");
    let first = t.elapsed();
    assert!(marks.len() > 90_000, "{} faces marked", marks.len());
    let gpu = headless_device().map(|(device, queue, _)| {
        let mesh = driftless_app::render::mesh::MeshRenderer::new(
            &device,
            driftless_app::ui::view3d::VIEW_FORMAT,
        );
        (device, queue, mesh)
    });
    let mut gpu = gpu;
    if gpu.is_none() {
        skipped("kein GPU-Adapter: ohne Schreiben auf die Grafikkarte gemessen");
    }

    // the camera from above, the floor filling the view
    let viewport = [1400.0f32, 900.0];
    let mut camera = Camera::default();
    camera.frame_facing([0.0; 3], [0.0, 0.0, 1.0], 3.5, viewport[0] / viewport[1]);
    let vp = camera.view_proj(viewport[0] / viewport[1]);
    let px = |p: [f64; 3]| {
        let q = project(&vp, relative_to(origin, p), viewport).unwrap();
        [q[0], q[1]]
    };
    // in the Stelle, beside it (every dab adds new faces), quickly, and erasing in it
    let cases = [
        ("in der Stelle, 5 cm je Bild", 2.9, 0.05, false),
        ("daneben, 5 cm je Bild", 5.85, 0.05, false),
        ("in der Stelle, 20 cm je Bild", 2.2, 0.20, false),
        ("radieren, 5 cm je Bild", 1.5, 0.05, true),
    ];
    for (what, y, step, erase) in cases {
        let mut cpu = Vec::new();
        let mut all = Vec::new();
        let frames = (4.0 / step) as usize;
        let mut release = None;
        for k in 0..=frames + 1 {
            let x = 1.0 + step * k.min(frames) as f64;
            let at = px([x, y, 0.0]);
            let input = BrushInput {
                viewport,
                hover: Some(at),
                pressed: k == 0,
                erase,
                path: vec![at],
                released: k == frames + 1,
            };
            let t = Instant::now();
            let stroke = brush.input(&mut state, &input, &camera, origin, false);
            let tris = brush.take_marks(state.brush.active, true);
            let brushed = t.elapsed();
            if let (Some(tris), Some((device, queue, mesh))) = (&tris, &mut gpu) {
                mesh.set_marks(device, queue, tris);
                queue.submit(std::iter::empty());
            }
            let total = t.elapsed();
            if k == frames + 1 {
                release = Some(total);
                let Some(Action::Paint { owner, mark, dabs }) = stroke else {
                    panic!("the stroke as one change")
                };
                // what the window does with it: the Stelle in the state, and it stays chosen
                let marks = state.marks.get_mut(&owner).unwrap().as_mut().unwrap();
                marks.paint(mark, &dabs);
                brush.sync(&mut state);
            } else if k > 0 {
                cpu.push(brushed.as_secs_f64() * 1e3);
                all.push(total.as_secs_f64() * 1e3);
            }
        }
        let stat = |v: &mut Vec<f64>| {
            v.sort_by(f64::total_cmp);
            (v[v.len() / 2], v[v.len() * 9 / 10], v[v.len() - 1])
        };
        let (cpu_med, cpu_p90, cpu_max) = stat(&mut cpu);
        let (med, p90, max) = stat(&mut all);
        eprintln!(
            "Pinsel auf 30 m² ({faces} Flächen, Stelle {dabs} Tupfer, {} Threads), {what}: \
             Pinsel+Markierung Median {cpu_med:.2} ms, 90 % {cpu_p90:.2} ms, max {cpu_max:.2} ms; \
             mit Grafikkarte Median {med:.2} ms, 90 % {p90:.2} ms, max {max:.2} ms; \
             Loslassen {:.1} ms; erste Auswahl {:.1} ms",
            rayon::current_num_threads(),
            release.unwrap().as_secs_f64() * 1e3,
            first.as_secs_f64() * 1e3,
        );
        assert!(
            med < 10.0 && p90 < 10.0,
            "{what}: Median {med:.2} ms, 90 % {p90:.2} ms"
        );
    }
}
