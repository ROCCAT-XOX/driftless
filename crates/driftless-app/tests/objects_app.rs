//! The tool „Objekt“ and the measurements kept, in the whole window: sockets, connections,
//! cables and pipes set in the 3D view, saved in `objekte.toml` of the map or model; the tool
//! off without a map or model of the project; what ⌘Z takes back with it and with the brush;
//! the objects in the plan and its DXF, also before a plan and through a new one; the list in
//! „Planung“; the measurements saved with the map, and only in memory on a cloud from
//! elsewhere. The project is made by hand. Most tests carry the clicks out as the view hands
//! them over; three look at the window with a GPU (without an adapter they say they were
//! skipped): a click on the cloud, a click on a surface alone, and a double click.

use driftless_app::pick::{project, relative_to};
use driftless_app::project::canonical;
use driftless_app::render::headless_device;
use driftless_app::ui::objects;
use driftless_app::ui::tools::{self, Tool};
use driftless_app::ui::view3d::PointInfo;
use driftless_app::ui::{
    text, Action, CloudSummary, DriftlessApp, Step, SurfaceSummary, UiMessage,
};
use driftless_export::testutil::write_map_folder;
use driftless_io::images::RgbImage;
use driftless_io::testscan::tmp;
use driftless_map::surface::{encode_pages, write_glb, TexturePage, TexturedMesh};
use driftless_plan::testutil::{Scene, TestData};
use driftless_steps::export::{ExportFormat, ExportRequest};
use driftless_steps::marks::{DabInput, Marks};
use driftless_steps::objects::{
    ConnectionKind, ObjectInput, ObjectKind, Objects, OBJECTS_FILE, SOURCE_HAND,
};
use eframe::egui::{self, accesskit::Role, Key, Modifiers, PointerButton};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

const A: [f64; 3] = [500_000.5, 401.0, 10.0];
const B: [f64; 3] = [500_002.5, 401.0, 10.0];
const C: [f64; 3] = [500_002.5, 402.0, 10.0];

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

fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            break;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
    h.step();
    h.step();
}

fn run(h: &mut H, action: Action) {
    h.state_mut().apply(action.clone());
    settle(h);
    assert_eq!(errors(h.state()), Vec::<String>::new(), "{action:?}");
}

/// A plate of 4 × 2 m with a point every 25 cm: a click hits exactly one point.
fn plate() -> Vec<[f64; 3]> {
    let mut xyz = Vec::new();
    for i in 0..=16 {
        for j in 0..=8 {
            xyz.push([
                500_000.0 + f64::from(i) * 0.25,
                400.0 + f64::from(j) * 0.25,
                10.0,
            ]);
        }
    }
    xyz
}

/// The project in `dir` with the scan `halle` (its raw data do not exist), a run and the map
/// `karte-001` of the points. Returns the folder of the map.
fn write_project(dir: &Path, xyz: &[[f64; 3]], poses: &[[f64; 3]]) -> PathBuf {
    fs::create_dir_all(dir).unwrap();
    let toml = "format = 1\nname = \"Halle\"\n\n[[scan]]\nid = \"halle\"\n\
                path = \"rohdaten/halle\"\nname = \"Halle\"\n";
    fs::write(dir.join("project.toml"), toml).unwrap();
    let run = dir.join("scans/halle/slam-001");
    let map = run.join("karte-001");
    write_map_folder(&map, xyz, poses);
    fs::write(run.join("slam.json"), "{}").unwrap();
    fs::copy(map.join("traj.tum"), run.join("traj.tum")).unwrap();
    map
}

/// The map folder as the program knows it.
fn known(project: &Path, map: &Path) -> PathBuf {
    canonical(project)
        .unwrap()
        .join(map.strip_prefix(project).unwrap())
}

fn app_window(app: DriftlessApp) -> H {
    Harness::builder()
        .with_size([1400.0, 900.0])
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app)
}

/// The window on the project in `dir`, in „Planung“.
fn open(dir: &Path) -> H {
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::OpenProject(dir.to_path_buf()));
    let mut h = app_window(app);
    settle(&mut h);
    run(&mut h, Action::SetStep(Step::Planung));
    h
}

/// A new project with the plate as the cloud of its map, shown. Returns the window, the project
/// and the map.
fn plate_project(dir: &Path) -> (H, PathBuf, PathBuf) {
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let mut h = open(&project);
    let map = known(&project, &map);
    run(
        &mut h,
        Action::LoadCloud {
            path: map.join("color.las"),
            run: None,
        },
    );
    (h, project, map)
}

fn choose(h: &mut H, tool: Tool) {
    tools::select(h.state_mut().state_mut(), tool);
    settle(h);
}

/// A click of the tool on `at`, as the view hands it over.
fn click(h: &mut H, at: [f64; 3]) {
    let action = objects::point_clicked(h.state_mut().state_mut(), at);
    if let Some(action) = action {
        run(h, action);
    }
}

fn key(h: &mut H, key: Key) {
    h.key_press(key);
    h.step();
    settle(h);
}

fn undo_keys(h: &mut H) {
    h.key_press_modifiers(Modifiers::COMMAND, Key::Z);
    h.step();
    settle(h);
}

fn saved(map: &Path) -> Objects {
    Objects::load(map).unwrap()
}

fn input(kind: ObjectKind, points: Vec<[f64; 3]>) -> ObjectInput {
    ObjectInput {
        kind,
        points,
        name: String::new(),
        note: String::new(),
        connection: None,
        diameter_mm: None,
        properties: Default::default(),
    }
}

/// „Punkt wählen“ of the point `position`, as the view hands it over.
fn point(position: [f64; 3]) -> PointInfo {
    PointInfo {
        index: 0,
        position,
        t: 0.0,
        intensity: 0,
        rgb: None,
        quality: None,
        removed: None,
        scan: None,
    }
}

/// Measures from `a` to `b` with the tool „Messen“, as the view hands the clicks over.
fn measure(h: &mut H, a: [f64; 3], b: [f64; 3]) {
    let state = h.state_mut().state_mut();
    tools::select(state, Tool::Measure);
    tools::point_clicked(state, point(a));
    tools::point_clicked(state, point(b));
    let action = objects::measured(state);
    if let Some(action) = action {
        run(h, action);
    }
    settle(h);
}

/// A surface folder of the map (no glb in it: only what the tree and the tools ask about), as
/// the surface shown alone.
fn show_surface_of(h: &mut H, map: &Path) {
    let surface = map.join("oberflaeche-001");
    fs::create_dir_all(&surface).unwrap();
    fs::write(surface.join("oberflaeche.json"), "{}").unwrap();
    h.state_mut().state_mut().refresh();
    h.state_mut().state_mut().surface = Some(SurfaceSummary {
        path: surface.join("oberflaeche.glb"),
        triangles: 2,
        pages: 1,
        origin: [0.0; 3],
        z_range: [0.0, 2.0],
    });
    settle(h);
}

/// Paints one Stelle on the surface of `map`.
fn paint(h: &mut H, map: &Path) {
    run(
        h,
        Action::Paint {
            owner: map.to_path_buf(),
            mark: None,
            dabs: vec![DabInput {
                centre: [1.0, 1.0, 1.0],
                normal: [0.0, -1.0, 0.0],
                radius: 0.2,
                erase: false,
            }],
        },
    );
}

// ─── setting objects ─────────────────────────────────────────────────────────────────────────

#[test]
fn socket_with_one_click() {
    let dir = tmp("objekt-steckdose");
    let (mut h, _, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    assert_eq!(h.state().state().tool, Tool::Object);
    click(&mut h, A);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1);
    let o = &saved.all()[0];
    assert_eq!(o.kind, ObjectKind::Socket);
    assert_eq!(o.points, vec![A]);
    assert_eq!(o.source, SOURCE_HAND);
    assert_eq!(o.nr, 1);
    // shown at once in the list of the field
    h.get_by_label("Steckdose 1");
    assert!(h.state().state().object_tool.line.is_empty());
}

#[test]
fn connection_kind_is_saved() {
    let dir = tmp("objekt-anschluss");
    let (mut h, _, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    let tool = &mut h.state_mut().state_mut().object_tool;
    tool.kind = ObjectKind::Connection;
    tool.connection = ConnectionKind::Water;
    click(&mut h, B);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1);
    assert_eq!(saved.all()[0].kind, ObjectKind::Connection);
    assert_eq!(saved.all()[0].connection, Some(ConnectionKind::Water));
}

#[test]
fn cable_with_clicks_and_enter() {
    let dir = tmp("objekt-kabel");
    let (mut h, _, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    h.state_mut().state_mut().object_tool.kind = ObjectKind::Cable;
    // the view can be seen after a tool was chosen
    click(&mut h, A);
    key(&mut h, Key::Enter);
    assert!(saved(&map).all().is_empty(), "one point makes no cable");
    assert!(
        h.state().state().object_tool.message.is_some(),
        "says why not"
    );
    click(&mut h, B);
    click(&mut h, C);
    // the same point again is no new point
    click(&mut h, C);
    assert_eq!(h.state().state().object_tool.line, vec![A, B, C]);
    assert!(saved(&map).all().is_empty(), "not before it is finished");
    key(&mut h, Key::Enter);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1);
    assert_eq!(saved.all()[0].kind, ObjectKind::Cable);
    assert_eq!(saved.all()[0].points, vec![A, B, C]);
    assert!(h.state().state().object_tool.line.is_empty());
}

#[test]
fn pipe_needs_a_diameter() {
    let dir = tmp("objekt-rohr");
    let (mut h, _, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    let tool = &mut h.state_mut().state_mut().object_tool;
    tool.kind = ObjectKind::Pipe;
    tool.diameter_mm = 0.0;
    click(&mut h, A);
    click(&mut h, B);
    key(&mut h, Key::Enter);
    assert!(saved(&map).all().is_empty());
    // the line stays, and the message says why
    assert_eq!(h.state().state().object_tool.line, vec![A, B]);
    assert!(h.state().state().object_tool.message.is_some());
    h.state_mut().state_mut().object_tool.diameter_mm = 40.0;
    key(&mut h, Key::Enter);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1);
    assert_eq!(saved.all()[0].kind, ObjectKind::Pipe);
    assert_eq!(saved.all()[0].diameter_mm, Some(40.0));
}

#[test]
fn backspace_and_esc() {
    let dir = tmp("objekt-tasten");
    let (mut h, _, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    h.state_mut().state_mut().object_tool.kind = ObjectKind::Cable;
    click(&mut h, A);
    click(&mut h, B);
    click(&mut h, C);
    key(&mut h, Key::Backspace);
    assert_eq!(h.state().state().object_tool.line, vec![A, B]);
    key(&mut h, Key::Escape);
    assert!(h.state().state().object_tool.line.is_empty());
    assert_eq!(h.state().state().tool, Tool::Object);
    assert!(saved(&map).all().is_empty());
    // another tool drops a line under way too
    click(&mut h, A);
    choose(&mut h, Tool::Orbit);
    assert!(h.state().state().object_tool.line.is_empty());
}

// ─── the clicks of the view (with a GPU) ─────────────────────────────────────────────────────

fn gpu_window(project: &Path) -> Option<H> {
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
    Some(h)
}

/// The window with a GPU on a project with the plate as the cloud of its map, shown.
fn plate_gpu(dir: &Path) -> Option<(H, PathBuf)> {
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let map = known(&project, &map);
    let mut h = gpu_window(&project)?;
    run(
        &mut h,
        Action::LoadCloud {
            path: map.join("color.las"),
            run: None,
        },
    );
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().state().cloud.is_none() || h.state().view().is_uploading() {
        h.step();
        assert!(Instant::now() < deadline, "cloud not shown");
        std::thread::sleep(Duration::from_millis(5));
    }
    for _ in 0..3 {
        h.step();
    }
    Some((h, map))
}

fn on_screen(h: &H, p: [f64; 3], origin: [f64; 3]) -> egui::Pos2 {
    let app = h.state();
    let view = app.layout().view;
    let vp = app.view().camera().view_proj(view.width() / view.height());
    let q = project(&vp, relative_to(origin, p), [view.width(), view.height()]).unwrap();
    view.min + egui::vec2(q[0], q[1])
}

fn cloud_origin(h: &H) -> [f64; 3] {
    h.state().cloud().unwrap().tree.origin
}

fn click_at(h: &mut H, pos: egui::Pos2) {
    let button = |pressed| egui::Event::PointerButton {
        pos,
        button: PointerButton::Primary,
        pressed,
        modifiers: Modifiers::NONE,
    };
    h.run_steps(30);
    h.event(egui::Event::PointerMoved(pos));
    h.step();
    h.event(button(true));
    h.step();
    h.event(button(false));
    h.step();
    h.step();
}

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

fn close(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() <= eps)
}

#[test]
fn cloud_is_hit_like_measuring() {
    let dir = tmp("objekt-wolke");
    let Some((mut h, map)) = plate_gpu(&dir) else {
        return;
    };
    choose(&mut h, Tool::Object);
    let at = on_screen(&h, A, cloud_origin(&h));
    click_at(&mut h, at);
    settle(&mut h);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1, "{:?}", saved.all());
    assert!(
        close(saved.all()[0].points[0], A, 0.01),
        "{:?}",
        saved.all()
    );
}

#[test]
fn double_click_ends_without_a_double_point() {
    let dir = tmp("objekt-doppelklick");
    let Some((mut h, map)) = plate_gpu(&dir) else {
        return;
    };
    choose(&mut h, Tool::Object);
    h.state_mut().state_mut().object_tool.kind = ObjectKind::Cable;
    let origin = cloud_origin(&h);
    let (a, b) = (on_screen(&h, A, origin), on_screen(&h, B, origin));
    click_at(&mut h, a);
    let camera = h.state().view().camera().clone();
    double_click(&mut h, b);
    settle(&mut h);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1, "{:?}", saved.all());
    let points = &saved.all()[0].points;
    assert_eq!(points.len(), 2, "no double point: {points:?}");
    assert!(close(points[0], A, 0.01) && close(points[1], B, 0.01));
    assert!(h.state().state().object_tool.line.is_empty());
    // the double click ended the line, it framed nothing
    assert_eq!(*h.state().view().camera(), camera);
}

#[test]
fn show_in_3d_goes_to_the_object() {
    let dir = tmp("objekt-zeigen");
    let Some((mut h, map)) = plate_gpu(&dir) else {
        return;
    };
    run(
        &mut h,
        Action::AddObject {
            owner: map,
            input: input(ObjectKind::Socket, vec![A]),
        },
    );
    run(&mut h, Action::FrameObject(1));
    h.step();
    h.step();
    assert_eq!(h.state().state().plan.selected_object, Some(1));
    let at = on_screen(&h, A, cloud_origin(&h));
    let centre = h.state().layout().view.center();
    assert!(at.distance(centre) < 2.0, "{at:?} vs {centre:?}");
}

#[test]
fn objects_show_in_3d_in_planung_or_with_the_tool_by_layer() {
    let dir = tmp("objekt-in-3d");
    let Some((mut h, map)) = plate_gpu(&dir) else {
        return;
    };
    let count = |h: &H| h.query_all_by_label("Steckdose 1").count();
    run(&mut h, Action::SetStep(Step::Planung));
    run(
        &mut h,
        Action::AddObject {
            owner: map,
            input: input(ObjectKind::Socket, vec![A]),
        },
    );
    assert_eq!(count(&h), 2, "in the list and in the view");
    // a layer off: not in the view, still in the list
    run(
        &mut h,
        Action::SetPlanLayer {
            layer: "Steckdose".into(),
            on: false,
        },
    );
    assert_eq!(count(&h), 1);
    run(
        &mut h,
        Action::SetPlanLayer {
            layer: "Steckdose".into(),
            on: true,
        },
    );
    // in another step only with the tool chosen
    run(&mut h, Action::SetStep(Step::Daten));
    assert_eq!(count(&h), 0);
    choose(&mut h, Tool::Object);
    assert_eq!(count(&h), 1);
}

/// Writes a wall of 4 × 2.5 m at y = 1 m as a surface glb to `path`, one page, grey.
fn wall_glb(path: &Path) {
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    let (nx, nz) = (40u32, 25u32);
    let mut positions = Vec::new();
    let mut uvs = Vec::new();
    for k in 0..=nz {
        for i in 0..=nx {
            let (x, z) = (f64::from(i) * 0.1, f64::from(k) * 0.1);
            positions.push([x as f32, 1.0f32, z as f32]);
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

#[test]
fn surface_only_is_hit_by_ray() {
    let dir = tmp("objekt-flaeche");
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let surface = map.join("oberflaeche-001");
    wall_glb(&surface.join("oberflaeche.glb"));
    fs::write(surface.join("oberflaeche.json"), "{}").unwrap();
    let map = known(&project, &map);
    let surface = map.join("oberflaeche-001");
    let Some(mut h) = gpu_window(&project) else {
        return;
    };
    run(&mut h, Action::SetStep(Step::Bilder));
    run(&mut h, Action::LoadSurface(surface.clone()));
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().state().surface.is_none() || h.state().view().is_uploading_surface() {
        h.step();
        assert!(Instant::now() < deadline, "surface not shown");
        std::thread::sleep(Duration::from_millis(5));
    }
    for _ in 0..3 {
        h.step();
    }
    choose(&mut h, Tool::Object);
    assert_eq!(h.state().state().tool, Tool::Object);
    let on_wall = [2.0, 1.0, 1.25];
    let origin = h.state().state().surface.as_ref().unwrap().origin;
    let at = on_screen(&h, on_wall, origin);
    click_at(&mut h, at);
    settle(&mut h);
    let saved = saved(&map);
    assert_eq!(saved.all().len(), 1, "{:?}", saved.all());
    assert!(
        close(saved.all()[0].points[0], on_wall, 0.03),
        "{:?}",
        saved.all()
    );
    assert_eq!(h.query_all_by_label("Steckdose 1").count(), 1);
}

// ─── off without a map or model ──────────────────────────────────────────────────────────────

fn foreign_cloud() -> CloudSummary {
    CloudSummary {
        path: PathBuf::from("/woanders/color.las"),
        points: 100,
        has_rgb: false,
        has_scans: false,
        quality_known: false,
        z_range: [0.0, 2.0],
        ceiling: None,
        point_m: 0.02,
    }
}

#[test]
fn off_without_owner_with_hint() {
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.state_mut().cloud = Some(foreign_cloud());
    let mut h = app_window(app);
    h.run();
    let button = h.get_by_role_and_label(Role::Button, text::TOOL_OBJECT);
    assert!(button.accesskit_node().is_disabled());
    button.hover();
    h.run();
    h.get_by_label(text::OBJECTS_NEED_OWNER);
    h.get_by_role_and_label(Role::Button, text::TOOL_OBJECT)
        .click();
    h.run();
    assert_eq!(h.state().state().tool, Tool::Orbit);
    assert_eq!(objects::view_owner(h.state().state()), None);
    // a click carried out anyway sets nothing, and says why
    let state = h.state_mut().state_mut();
    state.tool = Tool::Object;
    assert_eq!(objects::point_clicked(state, A), None);
    assert_eq!(
        state.object_tool.message.as_deref(),
        Some(text::OBJECTS_NEED_OWNER)
    );
}

// ─── ⌘Z ──────────────────────────────────────────────────────────────────────────────────────

#[test]
fn undo_with_object_tool_leaves_stellen_alone() {
    let dir = tmp("objekt-zurueck-objekt");
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let mut h = open(&project);
    let map = known(&project, &map);
    show_surface_of(&mut h, &map);
    paint(&mut h, &map);
    assert_eq!(Marks::load(&map).unwrap().marks.len(), 1);
    choose(&mut h, Tool::Object);
    assert_eq!(h.state().state().tool, Tool::Object);
    click(&mut h, A);
    assert_eq!(saved(&map).all().len(), 1);
    undo_keys(&mut h);
    assert!(saved(&map).all().is_empty());
    assert_eq!(
        Marks::load(&map).unwrap().marks.len(),
        1,
        "the Stelle stays"
    );
    // nothing left to take back of the objects: ⌘Z leaves the Stelle alone too
    undo_keys(&mut h);
    assert_eq!(Marks::load(&map).unwrap().marks.len(), 1);
}

#[test]
fn undo_with_brush_leaves_objects_alone() {
    let dir = tmp("objekt-zurueck-pinsel");
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let mut h = open(&project);
    let map = known(&project, &map);
    show_surface_of(&mut h, &map);
    paint(&mut h, &map);
    choose(&mut h, Tool::Object);
    click(&mut h, A);
    assert_eq!(saved(&map).all().len(), 1);
    choose(&mut h, Tool::Brush);
    assert_eq!(h.state().state().tool, Tool::Brush);
    undo_keys(&mut h);
    assert!(Marks::load(&map).unwrap().marks.is_empty());
    assert_eq!(saved(&map).all().len(), 1, "the object stays");
}

#[test]
fn undo_after_restart() {
    let dir = tmp("objekt-neustart");
    let (mut h, project, map) = plate_project(&dir);
    choose(&mut h, Tool::Object);
    click(&mut h, A);
    assert_eq!(saved(&map).all().len(), 1);
    drop(h);
    let mut h = open(&project);
    run(
        &mut h,
        Action::LoadCloud {
            path: map.join("color.las"),
            run: None,
        },
    );
    h.get_by_label("Steckdose 1");
    choose(&mut h, Tool::Object);
    let undo = h.get_by_role_and_label(Role::Button, text::BRUSH_UNDO);
    assert!(!undo.accesskit_node().is_disabled());
    undo_keys(&mut h);
    assert!(saved(&map).all().is_empty());
    assert!(h.query_by_label("Steckdose 1").is_none());
}

// ─── the plan ────────────────────────────────────────────────────────────────────────────────

const FLOOR: f64 = -1.3;
const HEIGHT: f64 = 2.6;
const LOOP: [[f64; 3]; 4] = [
    [1.0, 1.0, 0.0],
    [4.0, 1.0, 0.0],
    [4.0, 3.0, 0.0],
    [1.0, 3.0, 0.0],
];

fn one_room() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        Scene::new()
            .spacing(0.015)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, HEIGHT)
            .walk(&LOOP)
            .build()
    })
}

fn room_project(dir: &Path) -> (H, PathBuf, PathBuf) {
    let project = dir.join("projekt");
    let data = one_room();
    let map = write_project(&project, &data.xyz, &data.poses);
    let mut h = open(&project);
    let map = known(&project, &map);
    run(
        &mut h,
        Action::LoadCloud {
            path: map.join("color.las"),
            run: None,
        },
    );
    (h, project, map)
}

fn compute(h: &mut H, map: &Path) -> PathBuf {
    run(
        h,
        Action::RunPlan {
            owner: map.to_path_buf(),
            storey: None,
        },
    );
    h.state()
        .state()
        .plan
        .shown
        .as_ref()
        .expect("a plan is shown")
        .dir
        .clone()
}

/// The trajectory of the map changed: its plan no longer fits.
fn make_stale(h: &mut H, map: &Path) {
    let traj = "0.0 1.0 1.0 0.0 0.0 0.0 0.0 1.0\n0.1 1.5 1.0 0.0 0.0 0.0 0.0 1.0\n";
    fs::write(map.join("traj.tum"), traj).unwrap();
    h.state_mut().state_mut().refresh();
    settle(h);
}

#[test]
fn objects_show_in_plan_and_dxf() {
    let dir = tmp("objekt-plan-dxf");
    let (mut h, _, map) = room_project(&dir);
    let plan = compute(&mut h, &map);
    h.get_by_label("Steckdose (0)");
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Socket, vec![[1.0, 1.0, 0.3]]),
        },
    );
    // in the plan at once
    h.get_by_label("Steckdose (1)");
    let shown = h.state().state().plan.shown.clone().unwrap();
    assert_eq!(shown.objects.all().len(), 1);
    assert!(shown
        .drawing
        .entities
        .iter()
        .any(|e| e.layer == "Steckdose"));
    let out = dir.join("plan.dxf");
    let source = driftless_app::ui::plan::export_source(&h.state().state().plan, &plan);
    run(
        &mut h,
        Action::RunExport(ExportRequest {
            source,
            out: out.clone(),
            format: ExportFormat::Dxf,
            overwrite: false,
        }),
    );
    let written = String::from_utf8_lossy(&fs::read(&out).unwrap()).replace("\r\n", "\n");
    assert!(written.contains("\nSteckdose\n"), "layer in the DXF");
    assert!(written.contains("INSERT"));
}

#[test]
fn objects_before_a_plan_stay_and_appear_later() {
    let dir = tmp("objekt-vor-plan");
    let (mut h, _, map) = room_project(&dir);
    choose(&mut h, Tool::Object);
    click(&mut h, [1.0, 1.0, 0.3]);
    assert_eq!(saved(&map).all().len(), 1);
    h.get_by_label("Steckdose 1");
    let plan = compute(&mut h, &map);
    assert_eq!(
        h.state().state().plan.shown.as_ref().unwrap().dir,
        plan.clone()
    );
    h.get_by_label("Steckdose (1)");
    h.get_by_label("Steckdose 1");
}

#[test]
fn objects_survive_a_new_plan() {
    let dir = tmp("objekt-neuer-plan");
    let (mut h, _, map) = room_project(&dir);
    let first = compute(&mut h, &map);
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Socket, vec![[1.0, 1.0, 0.3]]),
        },
    );
    make_stale(&mut h, &map);
    let second = compute(&mut h, &map);
    assert_ne!(first, second);
    assert_eq!(saved(&map).all().len(), 1);
    h.get_by_label("Steckdose (1)");
}

#[test]
fn objects_of_a_stale_plan_stay_listed() {
    let dir = tmp("objekt-veraltet");
    let (mut h, _, map) = room_project(&dir);
    compute(&mut h, &map);
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Cable, vec![[1.0, 1.0, 0.3], [2.0, 1.0, 0.3]]),
        },
    );
    make_stale(&mut h, &map);
    // the entry of the list; the plan names the cable too
    h.get_by_role_and_label(Role::Button, "Kabel 1");
    h.get_by_label("Kabel (1)");
}

#[test]
fn rename_note_and_delete_from_list() {
    let dir = tmp("objekt-liste");
    let (mut h, _, map) = plate_project(&dir);
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Socket, vec![A]),
        },
    );
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Socket, vec![B]),
        },
    );
    // a click marks it, a click again lets it go
    h.get_by_label("Steckdose 2").click();
    h.run_steps(3);
    assert_eq!(h.state().state().plan.selected_object, Some(2));
    h.get_by_label("Steckdose 2").click();
    h.run_steps(3);
    assert_eq!(h.state().state().plan.selected_object, None);

    let menu = |h: &mut H, label: &str, entry: &str| {
        h.get_by_label(label).click_secondary();
        h.run_steps(4);
        h.get_by_label(entry).click();
        h.run_steps(4);
    };
    menu(&mut h, "Steckdose 1", text::OBJECT_RENAME);
    h.get_by_label(text::OBJECT_NAME).focus();
    h.step();
    h.get_by_label(text::OBJECT_NAME).type_text("Küche");
    h.step();
    h.get_by_label(text::RENAME_CONFIRM).click();
    settle(&mut h);
    assert_eq!(saved(&map).get(1).unwrap().name, "Küche");
    assert_eq!(saved(&map).get(1).unwrap().label(), "Steckdose 1 · Küche");
    h.get_by_label("Steckdose 1 · Küche");

    menu(&mut h, "Steckdose 1 · Küche", text::OBJECT_NOTE);
    h.get_by_label(text::OBJECT_NOTE_FIELD).focus();
    h.step();
    h.get_by_label(text::OBJECT_NOTE_FIELD)
        .type_text("hinter dem Schrank");
    h.step();
    h.get_by_label(text::OBJECT_NOTE_CONFIRM).click();
    settle(&mut h);
    assert_eq!(saved(&map).get(1).unwrap().note, "hinter dem Schrank");
    assert_eq!(saved(&map).all().len(), 2);

    menu(&mut h, "Steckdose 2", text::OBJECT_DELETE);
    settle(&mut h);
    assert_eq!(saved(&map).all().len(), 1);
    assert!(saved(&map).get(2).is_none());
    assert!(h.query_by_label("Steckdose 2").is_none());
    // the name is one change, the note one: ↶ takes the delete back first
    h.state_mut().apply(Action::UndoObject);
    settle(&mut h);
    assert!(saved(&map).get(2).is_some());
}

// ─── measurements ────────────────────────────────────────────────────────────────────────────

#[test]
fn measurement_is_saved_and_back_after_restart() {
    let dir = tmp("objekt-messung");
    let (mut h, project, map) = plate_project(&dir);
    measure(&mut h, A, B);
    let saved_now = saved(&map);
    assert_eq!(saved_now.all().len(), 1);
    assert_eq!(saved_now.all()[0].kind, ObjectKind::Measure);
    assert_eq!(saved_now.all()[0].points, vec![A, B]);
    assert_eq!(h.state().state().measurements.len(), 1);
    h.get_by_label(text::MEASUREMENTS_SAVED);
    // a socket lies beside it and does not count as a measurement
    run(
        &mut h,
        Action::AddObject {
            owner: map.clone(),
            input: input(ObjectKind::Socket, vec![C]),
        },
    );
    assert_eq!(h.state().state().measurements.len(), 1);
    drop(h);

    let mut h = open(&project);
    run(
        &mut h,
        Action::LoadCloud {
            path: map.join("color.las"),
            run: None,
        },
    );
    let state = h.state().state();
    assert_eq!(state.measurements.len(), 1);
    assert_eq!(state.measurements[0].a, A);
    assert_eq!(state.measurements[0].b, B);
    h.get_by_label(&text::measurement_name(1));
    h.get_by_label(&text::meters(2.0));
}

#[test]
fn foreign_cloud_measurement_stays_in_memory() {
    let dir = tmp("objekt-fremd");
    let project = dir.join("projekt");
    let map = write_project(&project, &plate(), &[[500_002.0, 401.0, 11.0]]);
    let mut h = open(&project);
    let map = known(&project, &map);
    h.state_mut().state_mut().cloud = Some(foreign_cloud());
    h.step();
    assert_eq!(objects::view_owner(h.state().state()), None);
    measure(&mut h, A, B);
    assert_eq!(h.state().state().measurements.len(), 1);
    assert!(!map.join(OBJECTS_FILE).exists(), "nothing written");
    h.get_by_label(text::MEASUREMENTS_NOT_SAVED);
    // the map of the project shown: its measurements (none), the other ones wait …
    h.state_mut().state_mut().cloud = Some(CloudSummary {
        path: map.join("color.las"),
        ..foreign_cloud()
    });
    settle(&mut h);
    assert_eq!(objects::view_owner(h.state().state()), Some(map.clone()));
    assert_eq!(h.state().state().measurements.len(), 0);
    // … and are back with the cloud from elsewhere
    h.state_mut().state_mut().cloud = Some(foreign_cloud());
    settle(&mut h);
    assert_eq!(h.state().state().measurements.len(), 1);
}

#[test]
fn clear_measurements_is_one_undo() {
    let dir = tmp("objekt-messungen-loeschen");
    let (mut h, _, map) = plate_project(&dir);
    measure(&mut h, A, B);
    measure(&mut h, B, C);
    assert_eq!(saved(&map).all().len(), 2);
    assert_eq!(h.state().state().measurements.len(), 2);
    h.get_by_role_and_label(Role::Button, text::CLEAR_MEASUREMENTS)
        .click();
    h.run_steps(3);
    settle(&mut h);
    assert!(saved(&map).all().is_empty());
    assert!(h.state().state().measurements.is_empty());
    run(&mut h, Action::UndoObject);
    assert_eq!(saved(&map).all().len(), 2);
    assert_eq!(h.state().state().measurements.len(), 2);
}
