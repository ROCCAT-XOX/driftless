//! „Melden“ in the whole window: ⇧⇧ opens the dialog (holding Shift never does), a report is
//! saved as a folder – with the picture of the window where a GPU draws it, taken before the
//! dialog opened –, Esc and „Abbrechen“ close it without saving; ☰ → „Meldungen …“ lists the
//! reports with what was answered in their files, and „Zur Ansicht“ goes back to the project,
//! selection and camera of a report; „Melden …“ in the bar of the brush attaches the Stellen
//! and names the one chosen. Every report goes to a folder of the test, never to the app's own.
//!
//! The project is made by hand like `tests/brush.rs`: a scan, a run with a SLAM report and a
//! trajectory (so that selecting the surface draws the run without an error), a map and a
//! surface that is a wall of 4 × 2.5 m. Without a GPU adapter the parts that need one say they
//! were skipped and the rest runs without.

use driftless_app::camera::{Camera, CameraMode, CameraState};
use driftless_app::feedback::{
    self, Feedback, FeedbackCamera, FeedbackKind, FeedbackProject, FeedbackSelection, LOG_FILE,
    REPORT_FILE, SCREENSHOT_FILE,
};
use driftless_app::logfile::{self, Level, LogFile};
use driftless_app::project::{canonical, Project};
use driftless_app::render::headless_device;
use driftless_app::ui::about;
use driftless_app::ui::tools::Tool;
use driftless_app::ui::{text, Action, DriftlessApp, Selection, Step, UiMessage};
use driftless_io::images::RgbImage;
use driftless_io::testscan::{room_bag_secs, tmp};
use driftless_map::surface::{encode_pages, write_glb, TexturePage, TexturedMesh};
use driftless_slam::report::{BeforeAfter, SubmapReport};
use driftless_slam::SlamReport;
use driftless_steps::marks::{DabInput, Marks, MARKS_FILE};
use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
use driftless_traj::{write_tum, Pose};
use eframe::egui::{
    self,
    accesskit::{Role, Toggled},
    Event, Key, Modifiers,
};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

const SIZE: [f32; 2] = [1400.0, 900.0];
/// Frames this far apart (s): the taps of ⇧⇧ are a few frames long.
const STEP: f32 = 0.05;
/// The wall: x from 0 to 4 m, z from 0 to 2.5 m, at y = 1 m, in cells of 10 cm.
const WALL_Y: f64 = 1.0;

/// Says that the running test (or a part of it) was skipped, and why, past the capture.
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

/// The app writing its reports into `reports`.
fn app(reports: &Path) -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {})).with_feedback_dir(Some(reports.to_path_buf()))
}

/// The window on `app` without a GPU: the view says so, „Melden“ opens without a picture.
fn window(app: DriftlessApp) -> H {
    let mut h = Harness::builder()
        .with_size(SIZE)
        .with_step_dt(STEP)
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run_ok();
    h
}

/// The window on `app` drawn by a GPU; `None` without an adapter.
fn gpu_window(app: DriftlessApp) -> Option<H> {
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    if headless_device().is_none() {
        skipped("kein GPU-Adapter: ohne Bild des Fensters");
        return None;
    }
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let app = app.with_render_state(rs.clone());
    let mut h = Harness::builder()
        .with_size(SIZE)
        .with_pixels_per_point(1.0)
        .with_step_dt(STEP)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    h.run_ok();
    Some(h)
}

fn shift(pressed: bool) -> Event {
    Event::Key {
        key: Key::ShiftLeft,
        physical_key: Some(Key::ShiftLeft),
        pressed,
        repeat: false,
        modifiers: if pressed {
            Modifiers::SHIFT
        } else {
            Modifiers::NONE
        },
    }
}

/// ⇧⇧: two taps on Shift, each 0.05 s down, 0.15 s between (a frame per event).
fn double_shift(h: &mut H) {
    for _ in 0..2 {
        h.key_press_modifiers(Modifiers::SHIFT, Key::ShiftLeft);
    }
    h.step();
    h.run_ok();
}

/// Shift held for `frames` frames, then let go.
fn hold_shift(h: &mut H, frames: usize) {
    h.event(Event::ModifiersChanged(Modifiers::SHIFT));
    h.event(shift(true));
    h.step();
    h.run_steps(frames);
    h.event(shift(false));
    h.event(Event::ModifiersChanged(Modifiers::NONE));
    h.step();
}

/// A click with Shift held into the middle of the window, in one frame each.
fn shift_click(h: &mut H) {
    let pos = egui::pos2(SIZE[0] / 2.0, SIZE[1] / 2.0);
    let button = |pressed| Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: Modifiers::SHIFT,
    };
    h.event(Event::ModifiersChanged(Modifiers::SHIFT));
    h.event(shift(true));
    h.event(button(true));
    h.event(button(false));
    h.event(shift(false));
    h.event(Event::ModifiersChanged(Modifiers::NONE));
    h.step();
}

/// The capital `c` typed as a keyboard sends it: Shift down, the key with its text, Shift up.
fn type_capital(h: &mut H, c: char, key: Key) {
    let key_event = |pressed| Event::Key {
        key,
        physical_key: Some(key),
        pressed,
        repeat: false,
        modifiers: Modifiers::SHIFT,
    };
    h.event(Event::ModifiersChanged(Modifiers::SHIFT));
    h.event(shift(true));
    h.event(key_event(true));
    h.event(Event::Text(c.to_string()));
    h.event(key_event(false));
    h.event(shift(false));
    h.event(Event::ModifiersChanged(Modifiers::NONE));
    h.step();
}

/// Types `text` into the text field of „Melden“, a capital first if it starts with one.
fn type_report(h: &mut H, text: &str) {
    h.get_by_role(Role::MultilineTextInput).focus();
    h.step();
    let mut chars = text.chars();
    let first = chars.next().unwrap();
    if first.is_uppercase() {
        let key = Key::from_name(&first.to_string()).expect("a letter key");
        type_capital(h, first, key);
        h.get_by_role(Role::MultilineTextInput)
            .type_text(chars.as_str());
    } else {
        h.get_by_role(Role::MultilineTextInput).type_text(text);
    }
    h.step();
    h.run_ok();
}

fn click(h: &mut H, role: Role, label: &str) {
    h.get_by_role_and_label(role, label).click();
    h.step();
    h.run_ok();
}

fn toggled(h: &H, label: &str) -> Option<Toggled> {
    h.get_by_label(label).accesskit_node().toggled()
}

fn disabled(h: &H, role: Role, label: &str) -> bool {
    h.get_by_role_and_label(role, label)
        .accesskit_node()
        .is_disabled()
}

/// Frames until `reports` holds `n` reports and no background work is left; the reports.
fn saved(h: &mut H, reports: &Path, n: usize) -> Vec<(PathBuf, Feedback)> {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        h.step();
        let all = feedback::read_all(reports);
        if all.len() >= n && !h.state().wants_repaint_while_busy() {
            assert_eq!(all.len(), n, "{all:?}");
            return all;
        }
        assert!(Instant::now() < deadline, "report not saved");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Frames until no step and no background work is left.
fn settle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        h.step();
        let app = h.state();
        if app.state().job.is_none() && !app.wants_repaint_while_busy() {
            break;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
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

/// A run in front of the wall: a SLAM report with one submap and three poses.
fn run_files(run: &Path) {
    fs::create_dir_all(run).unwrap();
    let start = 1_700_000_000.0;
    let report = SlamReport {
        scan_start: start,
        submaps: vec![SubmapReport {
            id: 0,
            t_start: 0.0,
            t_end: 3.0,
            quality: 1.0,
            anchor: [1.0, -1.0, 1.2],
            odom_anchor: [1.0, -1.0, 1.2],
        }],
        loops: Vec::new(),
        graph: None,
        start_end_gap: BeforeAfter {
            before: 0.0,
            after: 0.0,
        },
        path_length: BeforeAfter {
            before: 2.0,
            after: 2.0,
        },
    };
    fs::write(
        run.join("slam.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    let poses: Vec<Pose> = (0..3)
        .map(|k| Pose {
            t: start + f64::from(k),
            p: Vector3::new(1.0 + f64::from(k), -1.0, 1.2),
            q: UnitQuaternion::identity(),
        })
        .collect();
    write_tum(&run.join("traj.tum"), &poses).unwrap();
    write_tum(&run.join("traj_odom.tum"), &poses).unwrap();
}

/// A project „Halle“ with the scan `halle` and a run `slam-001` with a map `karte-001` and the
/// wall as its surface `oberflaeche-001`. Returns the project, the map and the surface folder.
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
    run_files(&run);
    let map = run.join("karte-001");
    file(&map, "map.json");
    let surface = map.join("oberflaeche-001");
    wall_glb(&surface.join("oberflaeche.glb"));
    file(&surface, "oberflaeche.json");
    (project, map, surface)
}

/// One Stelle on the wall, seen from the run.
fn wall_marks() -> Marks {
    let mut marks = Marks::default();
    let dab = DabInput {
        centre: [2.0, WALL_Y, 1.25],
        normal: [0.0, -1.0, 0.0],
        radius: 0.2,
        erase: false,
    };
    marks.paint(None, &[dab]);
    marks
}

/// Whether the view of `h` still waits for the surface: to be loaded, and with a GPU to be up.
fn surface_waits(h: &H, gpu: bool) -> bool {
    h.state().state().surface.is_none() || (gpu && h.state().view().is_uploading_surface())
}

/// Opens `project` in `h` (without a GPU), shows its surface in „Bilder“ (selected, so the run
/// is drawn).
fn show_surface(h: &mut H, project: &Path, surface: &Path) {
    h.state_mut()
        .apply(Action::OpenProject(project.to_path_buf()));
    settle(h);
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.state_mut()
        .apply(Action::Select(Selection::Surface(surface.to_path_buf())));
    let deadline = Instant::now() + Duration::from_secs(60);
    while surface_waits(h, false) {
        h.step();
        assert!(Instant::now() < deadline, "surface not shown");
        std::thread::sleep(Duration::from_millis(5));
    }
    settle(h);
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

/// How many pixels of `a` and `b` differ by more than a little.
fn differing(a: &egui::ColorImage, b: &image::RgbaImage) -> usize {
    assert_eq!(a.size, [b.width() as usize, b.height() as usize]);
    a.pixels
        .iter()
        .zip(b.pixels())
        .filter(|(p, q)| {
            let p = p.to_srgba_unmultiplied();
            (0..4).any(|k| p[k].abs_diff(q.0[k]) > 8)
        })
        .count()
}

#[test]
fn dialog_saves() {
    let dir = tmp("melden-dialog");
    let reports = dir.join("Meldungen");
    let log = Arc::new(LogFile::start(&dir.join("logs"), &about::version_line()).unwrap());
    log.write(Level::Warning, "Bild 3 übersprungen: zu unscharf");
    let mut h = window(app(&reports).with_log_file(log.clone()));
    // holding Shift (panning with Shift, flying faster) opens nothing, also twice in a row;
    // neither do Shift + click twice, nor two capital letters typed in a row
    hold_shift(&mut h, 10);
    hold_shift(&mut h, 10);
    for _ in 0..2 {
        shift_click(&mut h);
    }
    type_capital(&mut h, 'A', Key::A);
    type_capital(&mut h, 'B', Key::B);
    h.run_ok();
    assert!(h.state().state().feedback_dialog.is_none());
    // ⇧⇧ opens „Melden“; without a GPU at once and without a picture
    double_shift(&mut h);
    let dialog = h.state().state().feedback_dialog.as_ref().expect("Melden");
    assert!(dialog.image.is_none());
    h.get_by_role_and_label(Role::Label, text::FEEDBACK_TITLE);
    assert_eq!(toggled(&h, text::FEEDBACK_BUG), Some(Toggled::True));
    assert_eq!(toggled(&h, text::FEEDBACK_WISH), Some(Toggled::False));
    // without a picture and without Stellen there is nothing to tick
    assert_eq!(
        toggled(&h, text::FEEDBACK_ATTACH_IMAGE),
        Some(Toggled::False)
    );
    assert!(disabled(&h, Role::CheckBox, text::FEEDBACK_ATTACH_IMAGE));
    assert!(disabled(
        &h,
        Role::CheckBox,
        &text::feedback_attach_marks(0)
    ));
    // „Melden“ waits for a text
    assert!(disabled(&h, Role::Button, text::FEEDBACK_SEND));
    // a text with a capital letter typed with Shift: it is in the text, nothing opens again
    type_report(&mut h, "Decke flimmert beim Drehen");
    let dialog = h.state().state().feedback_dialog.as_ref().expect("Melden");
    assert_eq!(dialog.text, "Decke flimmert beim Drehen");
    assert!(!disabled(&h, Role::Button, text::FEEDBACK_SEND));
    click(&mut h, Role::Button, text::FEEDBACK_WISH);
    assert_eq!(toggled(&h, text::FEEDBACK_WISH), Some(Toggled::True));
    // „Melden“: a folder with the report and the last lines of the log file
    click(&mut h, Role::Button, text::FEEDBACK_SEND);
    assert!(h.state().state().feedback_dialog.is_none());
    let all = saved(&mut h, &reports, 1);
    let (folder, f) = &all[0];
    assert!(folder
        .file_name()
        .unwrap()
        .to_string_lossy()
        .ends_with("_wunsch"));
    assert_eq!(f.kind, FeedbackKind::Wish);
    assert_eq!(f.text, "Decke flimmert beim Drehen");
    assert_eq!(f.status, feedback::OPEN);
    assert_eq!(f.answer, "");
    assert_eq!(f.version, about::version_line());
    assert_eq!(f.system, logfile::os_text());
    assert!(
        chrono::DateTime::parse_from_rfc3339(&f.created).is_ok(),
        "{}",
        f.created
    );
    assert_eq!(f.project, None);
    assert_eq!(f.camera, None);
    assert!(folder.join(REPORT_FILE).is_file());
    assert!(!folder.join(SCREENSHOT_FILE).exists());
    let lines = fs::read_to_string(folder.join(LOG_FILE)).unwrap();
    assert!(
        lines.contains("Bild 3 übersprungen: zu unscharf"),
        "{lines}"
    );
    assert!(lines.lines().next().unwrap().contains(" Start "), "{lines}");
    let said: Vec<&str> = h
        .state()
        .state()
        .messages
        .iter()
        .map(|m| m.text())
        .collect();
    assert!(said.contains(&text::FEEDBACK_SAVED), "{said:?}");
    assert_eq!(errors(h.state()), Vec::<String>::new());
    // Esc closes without saving, so does „Abbrechen“
    double_shift(&mut h);
    type_report(&mut h, "verworfen");
    h.key_press(Key::Escape);
    h.step();
    h.run_ok();
    assert!(h.state().state().feedback_dialog.is_none());
    double_shift(&mut h);
    assert!(h.state().state().feedback_dialog.is_some());
    click(&mut h, Role::Button, text::CANCEL);
    assert!(h.state().state().feedback_dialog.is_none());
    settle(&mut h);
    assert_eq!(feedback::read_all(&reports).len(), 1);

    // with a GPU: the picture of the window as it was before the dialog opened
    let reports = dir.join("Meldungen-Bild");
    let Some(mut h) = gpu_window(app(&reports)) else {
        return;
    };
    let before = h.render().unwrap();
    // ⇧⇧ asks for the picture; the dialog opens only in the frame it comes in (eframe paints it
    // with the frame after the one that asked, so the dialog must not be open then)
    h.key_press_modifiers(Modifiers::SHIFT, Key::ShiftLeft);
    h.event(Event::ModifiersChanged(Modifiers::SHIFT));
    h.event(shift(true));
    h.event(shift(false));
    h.step();
    assert!(
        h.state().state().feedback_dialog.is_none(),
        "open before the picture came"
    );
    h.step();
    assert!(
        h.state().state().feedback_dialog.is_some(),
        "the picture came"
    );
    h.event(Event::ModifiersChanged(Modifiers::NONE));
    h.step();
    let image = h
        .state()
        .state()
        .feedback_dialog
        .as_ref()
        .unwrap()
        .image
        .clone();
    let image = image.expect("the picture of the window");
    let total = image.pixels.len();
    assert!(
        differing(&image, &before) * 200 < total,
        "not the window as before"
    );
    h.run_ok();
    let shown = h.render().unwrap();
    assert!(
        differing(&image, &shown) * 5 > total,
        "the picture shows the dialog"
    );
    assert_eq!(
        toggled(&h, text::FEEDBACK_ATTACH_IMAGE),
        Some(Toggled::True)
    );
    type_report(&mut h, "Mit Bild");
    click(&mut h, Role::Button, text::FEEDBACK_SEND);
    let all = saved(&mut h, &reports, 1);
    let (folder, f) = &all[0];
    let png = fs::read(folder.join(SCREENSHOT_FILE)).unwrap();
    let decoded = image::load_from_memory(&png).unwrap().to_rgba8();
    assert_eq!(
        [decoded.width() as usize, decoded.height() as usize],
        image.size
    );
    assert!(!f.graphics.is_empty(), "{f:?}");
}

#[test]
fn list_shows_answer_and_goes_back() {
    let dir = tmp("melden-liste");
    let (project, _map, surface) = wall_project(&dir);
    let reports = dir.join("Meldungen");
    // a report about the wall, seen from in front; Claude answers it in its file
    let camera = Camera {
        target: Vector3::new(2.0, WALL_Y as f32, 1.25),
        yaw: 90f32.to_radians(),
        pitch: (-5f32).to_radians(),
        distance: 3.0,
        ..Camera::default()
    };
    let state = camera.state([0.0; 3]);
    let mut f = Feedback::new(FeedbackKind::Bug);
    f.text = "Wand rauscht".into();
    f.created = "2026-10-29T09:52:00+01:00".into();
    f.project = Some(FeedbackProject::new("Halle", &project));
    f.step = Some("Bilder".into());
    f.selection = Some(FeedbackSelection::new(
        "oberflaeche",
        "Oberfläche 001",
        Some(surface.clone()),
    ));
    f.camera = Some(FeedbackCamera::new(state.clone()));
    let folder = feedback::write(&reports, &f, None, None, &[]).unwrap();
    let answer = |stand: &str| {
        let path = folder.join(REPORT_FILE);
        let text: String = fs::read_to_string(&path)
            .unwrap()
            .lines()
            .map(|line| {
                if line.starts_with("stand = ") {
                    format!("stand = \"{stand}\"\n")
                } else if line.starts_with("antwort = ") {
                    "antwort = \"Wände werden als ganze Fläche geglättet\"\n".to_string()
                } else {
                    format!("{line}\n")
                }
            })
            .collect();
        fs::write(&path, text).unwrap();
    };
    answer("erledigt in 0.2.0");
    // the window without a project: ☰ → „Meldungen …“
    let app = app(&reports);
    let (mut h, gpu) = match gpu_window(app) {
        Some(h) => (h, true),
        None => (window(self::app(&reports)), false),
    };
    let open_list = |h: &mut H| {
        click(h, Role::Button, text::MENU_PROJECT);
        click(h, Role::Button, text::REPORTS);
        let deadline = Instant::now() + Duration::from_secs(10);
        while h
            .state()
            .state()
            .feedback_list
            .as_ref()
            .is_some_and(|l| l.reading)
        {
            h.step();
            assert!(Instant::now() < deadline, "list not read");
            std::thread::sleep(Duration::from_millis(5));
        }
        h.run_ok();
    };
    open_list(&mut h);
    h.get_by_label(text::REPORTS_TITLE);
    h.get_by_label(text::FEEDBACK_BUG);
    h.get_by_label(&text::report_time(&f.created));
    h.get_by_label(&text::report_project("Halle"));
    h.get_by_label("Wand rauscht");
    h.get_by_label("erledigt in 0.2.0");
    h.get_by_label("Wände werden als ganze Fläche geglättet");
    // answered again while the list is closed: it reads the files anew when it opens
    h.get_by_role_and_label(Role::Button, "Close window")
        .click();
    h.step();
    h.run_ok();
    assert!(h.state().state().feedback_list.is_none());
    answer("erledigt in 0.2.1");
    open_list(&mut h);
    h.get_by_label("erledigt in 0.2.1");
    assert!(h.query_by_label("erledigt in 0.2.0").is_none());
    // „Zur Ansicht“: the project opens, its surface is selected and shown in „Bilder“, and the
    // camera is where it was – also after the surface went up to the GPU and framed itself
    click(&mut h, Role::Button, text::REPORT_TO_VIEW);
    assert!(h.state().state().feedback_list.is_none());
    let deadline = Instant::now() + Duration::from_secs(60);
    while h.state().report_view_pending() || surface_waits(&h, gpu) {
        h.step();
        assert!(
            Instant::now() < deadline,
            "the view of the report did not come back"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    settle(&mut h);
    h.run_steps(5);
    let app = h.state();
    assert_eq!(errors(app), Vec::<String>::new());
    assert_eq!(app.state().project.as_ref().unwrap().dir, project);
    assert_eq!(app.state().selection, Selection::Surface(surface.clone()));
    assert_eq!(app.state().step, Step::Bilder);
    assert_eq!(
        app.state().surface.as_ref().unwrap().path,
        surface.join("oberflaeche.glb")
    );
    assert!(app.overlay().is_some(), "the run of the surface is drawn");
    assert_eq!(gpu, app.view().view_origin().is_some());
    let origin = app
        .view()
        .view_origin()
        .or(app.state().surface.as_ref().map(|s| s.origin))
        .unwrap();
    let now = app.view().camera().state(origin);
    assert_eq!(now.mode, CameraMode::Orbit);
    assert_eq!(app.state().view.camera_mode, CameraMode::Orbit);
    let gap = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt();
    assert!(gap(now.target, state.target) < 1e-3, "{now:?} ≠ {state:?}");
    assert!(gap(now.eye, state.eye) < 1e-3, "{now:?} ≠ {state:?}");
    assert!(
        (now.yaw_deg - state.yaw_deg).abs() < 0.01,
        "{now:?} ≠ {state:?}"
    );
    assert!(
        (now.pitch_deg - state.pitch_deg).abs() < 0.01,
        "{now:?} ≠ {state:?}"
    );
}

#[test]
fn brush_bar_reports_mark() {
    let dir = tmp("melden-stelle");
    let (project, map, surface) = wall_project(&dir);
    wall_marks().save(&map).unwrap();
    let marks = Marks::load(&map).unwrap();
    let reports = dir.join("Meldungen");
    let mut h = window(app(&reports));
    show_surface(&mut h, &project, &surface);
    // the brush with Stelle 1 chosen: „Melden …“ in its bar
    click(&mut h, Role::Button, text::TOOL_BRUSH);
    assert_eq!(h.state().state().tool, Tool::Brush);
    h.state_mut().apply(Action::SelectMark(Some(1)));
    h.run_ok();
    click(&mut h, Role::Button, text::FEEDBACK_BUTTON);
    assert!(h.state().state().feedback_dialog.is_some());
    // „Markierte Stellen anhängen (1)“ is ticked, and the Stelle is named
    let attach = text::feedback_attach_marks(1);
    assert_eq!(toggled(&h, &attach), Some(Toggled::True));
    h.get_by_label(&text::feedback_about_mark("Stelle 1 · Wand"));
    type_report(&mut h, "Hier rauscht es");
    click(&mut h, Role::Button, text::FEEDBACK_SEND);
    let all = saved(&mut h, &reports, 1);
    let (folder, f) = &all[0];
    assert_eq!(f.mark, Some(1));
    assert_eq!(Marks::load(folder).unwrap(), marks);
    let p = f.project.as_ref().expect("project");
    assert_eq!(
        (p.name.as_str(), p.dir.as_path()),
        ("Halle", project.as_path())
    );
    let s = f.selection.as_ref().expect("selection");
    assert_eq!(s.kind, "oberflaeche");
    assert_eq!(s.path.as_deref(), Some(surface.as_path()));
    assert!(s.text.contains("Oberfläche 001"), "{s:?}");
    assert_eq!(f.step.as_deref(), Some("Bilder"));
    // without a GPU the view drew nothing: no camera to keep
    assert_eq!(f.camera, None);
    // ⇧⇧ about the surface: the Stellen ticked too, but no Stelle named; unticked, they stay out
    double_shift(&mut h);
    assert_eq!(toggled(&h, &attach), Some(Toggled::True));
    assert!(h
        .query_by_label(&text::feedback_about_mark("Stelle 1 · Wand"))
        .is_none());
    click(&mut h, Role::CheckBox, &attach);
    assert_eq!(toggled(&h, &attach), Some(Toggled::False));
    type_report(&mut h, "ohne Stellen");
    click(&mut h, Role::Button, text::FEEDBACK_SEND);
    let all = saved(&mut h, &reports, 2);
    let (folder, f) = &all[0];
    assert_eq!(f.text, "ohne Stellen");
    assert_eq!(f.mark, None);
    assert!(!folder.join(MARKS_FILE).exists());
    assert_eq!(errors(h.state()), Vec::<String>::new());
}

#[test]
fn menu_lists_reports() {
    use driftless_app::ui::{rail, AppState};
    let mut h = Harness::builder().with_size([320.0, 700.0]).build_ui_state(
        |ui, (state, action): &mut (AppState, Option<Action>)| {
            if let Some(a) = rail::show(ui, state) {
                *action = Some(a);
            }
        },
        (AppState::default(), None),
    );
    h.run();
    h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
        .click();
    h.run();
    h.get_by_label(text::REPORTS).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::OpenReports));
}

/// The camera kept with a report is the one of the view, in absolute coordinates.
#[test]
fn camera_state_kept_in_the_file() {
    let state = CameraState {
        mode: CameraMode::Fly,
        target: [512_002.0, 5_400_001.0, 101.25],
        eye: [512_002.0, 5_399_998.0, 101.5],
        yaw_deg: 90.0,
        pitch_deg: -4.76,
        distance_m: 3.01,
        fov_y_deg: 60.0,
    };
    let dir = tmp("melden-kamera");
    let mut f = Feedback::new(FeedbackKind::Wish);
    f.created = "2026-10-29T10:00:00+01:00".into();
    f.camera = Some(FeedbackCamera::new(state.clone()));
    let folder = feedback::write(&dir, &f, None, None, &[]).unwrap();
    let toml = fs::read_to_string(folder.join(REPORT_FILE)).unwrap();
    assert!(toml.contains("[kamera]"), "{toml}");
    assert!(
        toml.contains("ziel = [512002.0, 5400001.0, 101.25]"),
        "{toml}"
    );
    let back = feedback::read_all(&dir);
    assert_eq!(back[0].1.camera.as_ref().unwrap().state, state);
}
