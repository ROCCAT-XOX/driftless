//! The touch-up of Stellen in the program, without a window: a Stelle painted
//! ([`Action::Paint`]), then smoothed or its images baked anew as a background step with its
//! progress; a refused step taken back out of `stellen.toml`; „Bilder neu einspielen“ off while
//! the raw data are gone; a new surface of the map taking the touch-up over; the export holding
//! it; „Aufräumen“ leaving it; ↶ on a step computing the touch-up anew; a new map starting
//! without Stellen.
//!
//! The project is made once (`scan_with_images` outside the project folder, a run, a map and a
//! coarse surface with images, as the program computes them); every test works on a copy.

use driftless_app::project::{
    canonical, run_map_job, run_slam_job, run_surface_job, MapInfo, Project, RunInfo, ScanEntry,
    ScanState, SlamJob, SurfaceInfo, SurfaceSettings,
};
use driftless_app::ui::tools::Tool;
use driftless_app::ui::{
    brush, export, text, Action, AppState, DriftlessApp, ScanNode, Step, SurfaceSummary, UiMessage,
};
use driftless_export::mesh::{read_mesh, Mesh};
use driftless_gpu::GpuPref;
use driftless_io::progress::Progress;
use driftless_io::testdir::{SharedDir, TestDir};
use driftless_io::testscan::{scan_with_images, shared, tmp};
use driftless_map::surface::edit::{dab_normal, FaceBvh, WeldedSurface, PATCHES_FILE};
use driftless_map::surface::SurfaceConfig;
use driftless_steps::export::{ExportFormat, ExportRequest, ExportSource};
use driftless_steps::map::{MapSettings, MapSwitches};
use driftless_steps::marks::{DabInput, EditStep, Marks, SmoothLevel, MARKS_FILE};
use driftless_steps::slam::SlamSettings;
use driftless_steps::surface::texture_sources;
use driftless_steps::touchup::{edited_mesh, touchup_fits, touchup_without_raw_data, TOUCHUP_DIR};
use eframe::egui::accesskit::Toggled;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

/// Where the device stood: the rays of the brush start here.
const EYE: [f64; 3] = [0.0, 0.0, 0.0];
const PROJECT: &str = "Abnahme";

/// Coarse settings: the synthetic room has a point every 10 cm or so.
fn small_config(voxel: f64) -> SurfaceConfig {
    SurfaceConfig {
        voxel,
        min_weight: 1,
        min_component_faces: 10,
        page_size: 256,
        simplify_error: 0.01,
        ..SurfaceConfig::default()
    }
}

fn surface_settings(voxel: f64) -> SurfaceSettings {
    SurfaceSettings {
        config: small_config(voxel),
        texture: true,
    }
}

/// `<dir>/Rohdaten/raum` (a scan with camera images) and `<dir>/Abnahme` with its run, map and
/// a surface with images, made once for all tests of this file.
fn template() -> &'static Path {
    static DIR: OnceLock<SharedDir> = OnceLock::new();
    DIR.get_or_init(|| {
        let dir = shared("nacharbeit-app-vorlage");
        let raw = dir.join("Rohdaten").join("raum");
        fs::create_dir_all(&raw).unwrap();
        scan_with_images(&raw);
        let mut p = Project::create(&dir.join(PROJECT), "Abnahme").unwrap();
        let id = p.add_scan(&raw).unwrap();
        p.save().unwrap();
        let full = SlamJob::Full(SlamSettings::default());
        let run = run_slam_job(&p, &id, &full, &Progress::new()).unwrap();
        let info = |p: &Project| {
            p.runs(&id)
                .into_iter()
                .find(|r| r.dir == run.run_dir)
                .unwrap()
        };
        run_map_job(&info(&p), &raw, &MapSettings::default(), &Progress::new()).unwrap();
        let run_info = info(&p);
        let done = run_surface_job(
            &run_info,
            &raw,
            &run_info.maps[0],
            &surface_settings(0.15),
            GpuPref::Off,
            &Progress::new(),
        )
        .unwrap();
        let report = &done.report;
        assert!(
            report["stats"]["texture"]["faces_textured"]
                .as_u64()
                .unwrap()
                > 0,
            "the surface has images: {report}"
        );
        dir
    })
    .path()
}

/// Copies the folder `from` with everything in it to `to`.
fn copy_dir(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for e in fs::read_dir(from).unwrap() {
        let e = e.unwrap();
        let target = to.join(e.file_name());
        if e.file_type().unwrap().is_dir() {
            copy_dir(&e.path(), &target);
        } else {
            fs::copy(e.path(), &target).unwrap();
        }
    }
}

/// Points the absolute paths of `project.toml` and of every `lauf.toml` below `to`, a copy of
/// `from`, into the copy: it finds its own raw data, not those of `from`.
fn own_paths(from: &Path, to: &Path) {
    fn rewrite(v: &mut toml::Value, from: &[PathBuf], to: &Path) {
        match v {
            toml::Value::String(text) => {
                let path = PathBuf::from(text.as_str());
                if let Some(rest) = from.iter().find_map(|f| path.strip_prefix(f).ok()) {
                    *text = to.join(rest).to_string_lossy().into_owned();
                }
            }
            toml::Value::Array(items) => items.iter_mut().for_each(|i| rewrite(i, from, to)),
            toml::Value::Table(t) => t.iter_mut().for_each(|(_, i)| rewrite(i, from, to)),
            _ => {}
        }
    }
    fn walk(dir: &Path, from: &[PathBuf], to: &Path) {
        for e in fs::read_dir(dir).unwrap() {
            let path = e.unwrap().path();
            if path.is_dir() {
                walk(&path, from, to);
            } else if path
                .file_name()
                .is_some_and(|n| n == "project.toml" || n == "lauf.toml")
            {
                let mut v: toml::Value =
                    toml::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
                rewrite(&mut v, from, to);
                fs::write(&path, toml::to_string(&v).unwrap()).unwrap();
            }
        }
    }
    let from = [from.to_path_buf(), canonical(from).unwrap()];
    walk(to, &from, &canonical(to).unwrap());
}

/// Ticks the program until its work has ended.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        app.tick();
        if !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
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

/// Every message of the status bar, of any kind.
fn said(app: &DriftlessApp) -> Vec<String> {
    let messages = app.state().messages.iter();
    messages.map(|m| m.text().to_string()).collect()
}

fn no_errors(app: &DriftlessApp) {
    assert_eq!(errors(app), Vec::<String>::new(), "{:?}", said(app));
}

/// A copy of [`template`] of a test with the program on it: the project open, in „Bilder“,
/// its surface shown.
struct Scene {
    dir: TestDir,
    project: PathBuf,
    map: PathBuf,
    surface: PathBuf,
    app: DriftlessApp,
}

impl Scene {
    fn new(name: &str) -> Scene {
        let dir = tmp(name);
        copy_dir(template(), &dir);
        own_paths(template(), &dir);
        let project = canonical(&dir.join(PROJECT)).unwrap();
        let mut app = DriftlessApp::new(Arc::new(|| {})).with_trash(gone);
        app.apply(Action::OpenProject(project.clone()));
        settle(&mut app);
        no_errors(&app);
        app.state_mut().gpu = GpuPref::Off;
        app.state_mut().surface_settings = surface_settings(0.15);
        app.apply(Action::SetStep(Step::Bilder));
        // as the tree names them
        let map = app.state().scans[0].runs[0].maps[0].clone();
        let surface = map.surfaces[0].dir.clone();
        app.apply(Action::LoadSurface(surface.clone()));
        settle(&mut app);
        no_errors(&app);
        let shown = app.state().surface.as_ref().map(|s| s.path.clone());
        assert_eq!(shown, Some(surface.join("oberflaeche.glb")));
        assert_eq!(brush::blocked(app.state()), None);
        Scene {
            dir,
            project,
            map: map.dir,
            surface,
            app,
        }
    }

    fn marks(&self) -> Marks {
        Marks::load(&self.map).unwrap()
    }

    /// Paints `dabs` as one stroke into a new Stelle and waits until it is saved; its number.
    fn paint(&mut self, dabs: Vec<DabInput>) -> u32 {
        let before: BTreeSet<u32> = self.marks().marks.iter().map(|m| m.nr).collect();
        self.app.apply(Action::Paint {
            owner: self.map.clone(),
            mark: None,
            dabs,
        });
        settle(&mut self.app);
        no_errors(&self.app);
        let nr = self
            .marks()
            .marks
            .iter()
            .map(|m| m.nr)
            .find(|nr| !before.contains(nr))
            .expect("a new Stelle");
        assert_eq!(self.app.state().brush.active, Some(nr));
        nr
    }

    /// Asks for `action` and waits until it is done.
    fn run(&mut self, action: Action) {
        self.app.apply(action);
        settle(&mut self.app);
    }

    fn cache(&self) -> PathBuf {
        self.surface.join(TOUCHUP_DIR)
    }

    fn patches(&self) -> Vec<u8> {
        fs::read(self.cache().join(PATCHES_FILE)).unwrap()
    }
}

/// The trash of a test: what goes there is gone.
fn gone(path: &Path) -> Result<(), String> {
    let removed = if path.is_dir() {
        fs::remove_dir_all(path)
    } else {
        fs::remove_file(path)
    };
    removed.map_err(|e| e.to_string())
}

/// Dabs of `radius` where the rays from [`EYE`] towards `targets` meet the surface in
/// `surface_dir`, each with the normal the brush gives it.
fn dabs(surface_dir: &Path, targets: &[[f64; 3]], radius: f64) -> Vec<DabInput> {
    let mesh = read_mesh(&surface_dir.join("oberflaeche.glb"), &Progress::new()).unwrap();
    let s = WeldedSurface::from_mesh(&mesh);
    let bvh = FaceBvh::build(&s);
    targets
        .iter()
        .map(|t| {
            let dir = [t[0] - EYE[0], t[1] - EYE[1], t[2] - EYE[2]];
            let hit = bvh
                .raycast(&s, EYE, dir, None)
                .unwrap_or_else(|| panic!("the ray towards {t:?} hits the surface"));
            DabInput {
                centre: hit.pos,
                normal: dab_normal(&s, &bvh, &hit, radius),
                radius: radius as f32,
                erase: false,
            }
        })
        .collect()
}

/// A Stelle on the +x wall, the plain one the camera looks at: about 1.5 × 0.7 m around
/// (6, 1, 0).
fn front_wall(surface_dir: &Path) -> Vec<DabInput> {
    let targets: Vec<[f64; 3]> = (0..5)
        .map(|i| [6.0, 0.6 + 0.2 * f64::from(i), 0.0])
        .collect();
    let dabs = dabs(surface_dir, &targets, 0.35);
    for d in &dabs {
        assert!(d.normal[0] < -0.9, "faces the device: {d:?}");
    }
    dabs
}

/// A Stelle across the corner of the −x and the −y wall: not flat.
fn corner(surface_dir: &Path) -> Vec<DabInput> {
    let dabs = dabs(surface_dir, &[[-4.0, -2.6, 0.0], [-3.6, -3.0, 0.0]], 0.45);
    assert!(
        dabs[0].normal[0] > 0.9 && dabs[1].normal[1] > 0.9,
        "{dabs:?}"
    );
    dabs
}

/// The places the triangles of `mesh` use, by their bits.
fn used(mesh: &Mesh) -> BTreeSet<[u64; 3]> {
    mesh.pages
        .iter()
        .flat_map(|p| {
            p.indices
                .iter()
                .flatten()
                .map(|&i| p.positions[i as usize].map(|c| (c + 0.0).to_bits()))
        })
        .collect()
}

fn glb(dir: &Path) -> Mesh {
    read_mesh(&dir.join("oberflaeche.glb"), &Progress::new()).unwrap()
}

#[test]
fn smooth_runs_and_shows() {
    let mut scene = Scene::new("nacharbeit-app-glaetten");
    let nr = scene.paint(front_wall(&scene.surface));
    // painting alone needs no touch-up
    assert!(!scene.cache().exists());
    assert!(!scene.app.view().has_edit());

    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    // the step is in the file, the touch-up computed as a step of its own and kept
    let marks = scene.marks();
    assert!(matches!(
        marks.marks[0].steps[..],
        [EditStep::Smooth {
            level: SmoothLevel::Strong,
            ..
        }]
    ));
    assert!(touchup_fits(&scene.surface, &marks));
    assert!(scene.cache().join(PATCHES_FILE).is_file());
    let log: Vec<&str> = scene
        .app
        .state()
        .log
        .iter()
        .map(|l| l.text.as_str())
        .collect();
    assert!(
        log.iter().any(|l| l.starts_with("Nacharbeit: 1 Schritt")),
        "{log:?}"
    );
    let told = said(&scene.app);
    assert!(
        told.iter()
            .any(|m| m.contains(&format!("Stelle {nr}")) && m.contains("geglättet")),
        "{told:?}"
    );
    // the view holds the touched-up state and shows it: „Nachher“
    assert!(scene.app.view().has_edit());
    assert!(scene.app.view().showing_edit());
    assert!(scene.app.state().touchup.after);
    assert!(scene.app.state().touchup.shown.is_some());
    assert!(scene.app.state().touchup.edited);

    // „Vorher“ shows the surface as computed, without loading anything
    scene.app.apply(Action::ShowEdit(false));
    assert!(!scene.app.view().showing_edit());
    assert!(!scene.app.state().touchup.after);
    assert!(scene.app.view().has_edit());
    scene.app.apply(Action::ShowEdit(true));
    assert!(scene.app.view().showing_edit());

    // „Melden …“ of the bar asks for the dialog (filled by the next step of the plan)
    scene.app.apply(Action::OpenFeedback { mark: Some(nr) });
    no_errors(&scene.app);

    // after a restart the touch-up is there again: the loader gives it with the surface, no
    // touch-up runs for it
    let bytes = scene.patches();
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::OpenProject(scene.project.clone()));
    settle(&mut app);
    app.apply(Action::LoadSurface(scene.surface.clone()));
    let deadline = Instant::now() + Duration::from_secs(60);
    while app.state().surface.is_none() {
        app.tick();
        assert!(Instant::now() < deadline, "surface not loaded");
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(app.view().has_edit());
    assert!(app.state().running.is_none());
    assert!(app.state().touchup.waiting.is_empty());
    settle(&mut app);
    no_errors(&app);
    assert_eq!(app.state().touchup.shown, scene.app.state().touchup.shown);
    assert_eq!(scene.patches(), bytes);
}

#[test]
fn cancel_takes_the_step_back() {
    let mut scene = Scene::new("nacharbeit-app-abbrechen");
    let nr = scene.paint(front_wall(&scene.surface));
    scene.app.apply(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    // „Abbrechen“ as soon as the touch-up runs, with its progress in the status bar
    let deadline = Instant::now() + Duration::from_secs(60);
    while !matches!(scene.app.state().running, Some(Action::Touchup { .. })) {
        scene.app.tick();
        assert!(Instant::now() < deadline, "the touch-up did not start");
        std::thread::sleep(Duration::from_millis(1));
    }
    let job = scene.app.state().job.clone().expect("its progress");
    assert_eq!(job.label, text::STEP_TOUCHUP);
    scene.run(Action::Cancel);
    no_errors(&scene.app);
    let marks = scene.marks();
    let told = said(&scene.app);
    if told.contains(&text::job_cancelled(text::STEP_TOUCHUP)) {
        // stopped: the step goes again, nothing of it is kept or shown
        assert!(told.contains(&text::touchup_taken_back(nr)), "{told:?}");
        assert!(marks.marks[0].steps.is_empty(), "{marks:?}");
        assert!(!scene.cache().exists());
        assert!(!scene.app.view().has_edit());
    } else {
        // done before the cancel came: as asked
        eprintln!("cancel_takes_the_step_back: the touch-up ended before the cancel");
        assert_eq!(marks.marks[0].steps.len(), 1);
        assert!(scene.app.view().has_edit());
    }
    assert!(touchup_fits(&scene.surface, &marks));
}

/// A wall of 2 × 2 m at y = 1 m in cells of 25 cm, facing −y, one page without an image.
fn wall_mesh() -> Mesh {
    let n = 8u32;
    let mut positions = Vec::new();
    for k in 0..=n {
        for i in 0..=n {
            positions.push([f64::from(i) * 0.25, 1.0, f64::from(k) * 0.25]);
        }
    }
    let at = |i: u32, k: u32| k * (n + 1) + i;
    let mut indices = Vec::new();
    for k in 0..n {
        for i in 0..n {
            let (a, b, c, d) = (at(i, k), at(i + 1, k), at(i + 1, k + 1), at(i, k + 1));
            indices.push([a, b, c]);
            indices.push([a, c, d]);
        }
    }
    Mesh {
        pages: vec![driftless_export::mesh::MeshPage {
            positions,
            uvs: Vec::new(),
            indices,
            image: None,
            color: [128, 128, 128],
        }],
    }
}

#[test]
fn marks_lie_on_the_state_drawn() {
    use driftless_app::surface::SurfacePick;
    use driftless_app::ui::brush::Brush;
    let mesh = wall_mesh();
    let pick = Arc::new(SurfacePick::new(&mesh));
    // Stelle 1 of [`crafted`] lies on the wall around (1, 1, 1)
    let mut state = crafted();
    let glb = state.surface.as_ref().unwrap().path.clone();
    let mut brush = Brush::default();
    brush.set_surface(Some((glb, [0.0; 3], Arc::clone(&pick))));
    let corners = |tris: &[driftless_app::render::mesh::MarkTriangle]| -> Vec<[f32; 3]> {
        tris.iter().flat_map(|t| t.pos).collect()
    };
    let computed = brush.sync(&mut state).expect("the marks");
    assert!(!computed.is_empty());
    assert!(corners(&computed).iter().all(|p| p[1] == 1.0));
    // the touch-up moved the corner in the middle of the Stelle 10 cm out
    let v = pick
        .surface
        .positions()
        .iter()
        .position(|p| *p == [1.0, 1.0, 1.0])
        .unwrap() as u32;
    brush.set_moved(&[(v, [1.0, 0.9, 1.0])]);
    let after = brush.take_marks(Some(1)).expect("the marks anew");
    assert!(corners(&after).contains(&[1.0, 0.9, 1.0]));
    assert_eq!(after.len(), computed.len());
    // „Vorher“: where they were computed; „Nachher“ again: moved
    brush.set_after(false);
    let before = brush.take_marks(Some(1)).expect("the marks anew");
    assert_eq!(before, computed);
    brush.set_after(true);
    assert_eq!(brush.take_marks(Some(1)), Some(after));
    // nothing changed: nothing to hand over
    brush.set_after(true);
    assert_eq!(brush.take_marks(Some(1)), None);
}

#[test]
fn refused_flat_is_taken_back() {
    let mut scene = Scene::new("nacharbeit-app-abgelehnt");
    let nr = scene.paint(corner(&scene.surface));
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Flat,
    });
    no_errors(&scene.app);
    let told = said(&scene.app);
    assert!(
        told.iter()
            .any(|m| m.contains(&format!("Stelle {nr}")) && m.contains("nicht eben genug")),
        "{told:?}"
    );
    // the step is out of the file again, the Stelle stays
    let marks = scene.marks();
    assert_eq!(marks.marks.len(), 1);
    assert!(!marks.marks[0].dabs.is_empty());
    assert!(marks.marks[0].steps.is_empty(), "{marks:?}");
    let shown = scene.app.state().marks.get(&scene.map).cloned();
    assert_eq!(shown, Some(Ok(marks.clone())));
    // nothing changed on the surface: no touch-up kept, none shown
    assert!(!scene.cache().exists());
    assert!(!scene.app.view().has_edit());
    assert!(touchup_fits(&scene.surface, &marks));
}

#[test]
fn no_raw_data_disables_retexture() {
    let mut scene = Scene::new("nacharbeit-app-ohne-rohdaten");
    let nr = scene.paint(front_wall(&scene.surface));
    assert_eq!(brush::retexture_blocked(scene.app.state()), None);

    // the drive with the raw data unplugged, before the program read the tree again: the
    // touch-up finds no images, says so, and takes the step back out
    let raw = scene.dir.join("Rohdaten");
    let away = scene.dir.join("Rohdaten-weg");
    fs::rename(&raw, &away).unwrap();
    assert_eq!(brush::retexture_blocked(scene.app.state()), None);
    scene.run(Action::Retexture { mark: nr });
    no_errors(&scene.app);
    assert_eq!(
        said(&scene.app).last().map(String::as_str),
        Some(text::RETEXTURE_NO_RAW_DATA)
    );
    assert!(scene.marks().marks[0].steps.is_empty());
    assert!(!scene.cache().exists());
    assert!(!scene.app.view().has_edit());

    // once the tree is read again, the button is off
    scene.run(Action::OpenProject(scene.project.clone()));
    assert_eq!(
        brush::retexture_blocked(scene.app.state()).as_deref(),
        Some(text::RETEXTURE_NO_RAW_DATA)
    );
    // asked for anyway: refused at once, nothing written
    let before = fs::read(scene.map.join(MARKS_FILE)).unwrap();
    scene.app.apply(Action::Retexture { mark: nr });
    assert_eq!(
        said(&scene.app).last().map(String::as_str),
        Some(text::RETEXTURE_NO_RAW_DATA)
    );
    assert!(scene.app.state().touchup.waiting.is_empty());
    settle(&mut scene.app);
    assert_eq!(fs::read(scene.map.join(MARKS_FILE)).unwrap(), before);
    assert!(!scene.cache().exists());

    // smoothing works without them; the images stay as they were, which a hint says
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    let marks = scene.marks();
    assert_eq!(marks.marks[0].steps.len(), 1);
    assert!(touchup_fits(&scene.surface, &marks));
    assert!(touchup_without_raw_data(&scene.surface));
    assert!(scene.app.view().has_edit());
    assert!(said(&scene.app)
        .iter()
        .any(|m| m.contains("Rohdaten fehlen")));
    let without = scene.patches();

    // back again: the button is on, and showing the surface bakes the images of the
    // smoothed Stelle anew
    fs::rename(&away, &raw).unwrap();
    scene.run(Action::OpenProject(scene.project.clone()));
    assert_eq!(brush::retexture_blocked(scene.app.state()), None);
    scene.run(Action::ReloadSurface(scene.surface.clone()));
    no_errors(&scene.app);
    assert!(!touchup_without_raw_data(&scene.surface));
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    assert_ne!(scene.patches(), without);
    assert!(scene.app.view().has_edit());

    // and now „Bilder neu einspielen“ runs: the next variant, said with how many there are
    scene.run(Action::Retexture { mark: nr });
    no_errors(&scene.app);
    let marks = scene.marks();
    assert!(matches!(
        marks.marks[0].steps[..],
        [
            EditStep::Smooth { .. },
            EditStep::Retexture { variant: 1, .. }
        ]
    ));
    assert!(touchup_fits(&scene.surface, &marks));
    let told = said(&scene.app);
    assert!(
        told.iter().any(|m| m.contains("Bilder neu eingespielt")
            && m.contains("Variante")
            && m.contains(" von ")),
        "{told:?}"
    );
}

#[test]
fn new_surface_gets_touchup() {
    let mut scene = Scene::new("nacharbeit-app-neue-oberflaeche");
    let nr = scene.paint(front_wall(&scene.surface));
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);

    // another fineness: a new surface of the same map, which takes the Stellen over; its
    // touch-up is queued with the end of the step, before (whether) the surface is shown
    scene.app.state_mut().surface_settings = surface_settings(0.2);
    scene.app.apply(Action::RunSurface(scene.map.clone()));
    let second = scene.map.join("oberflaeche-002");
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        scene.app.tick();
        let state = scene.app.state();
        if matches!(&state.running, Some(Action::Touchup { surface, .. }) if *surface == second) {
            let shown = state.surface.as_ref().map(|s| s.path.clone());
            assert_eq!(shown, Some(scene.surface.join("oberflaeche.glb")));
            break;
        }
        assert!(Instant::now() < deadline, "no touch-up of the new surface");
        std::thread::sleep(Duration::from_millis(1));
    }
    settle(&mut scene.app);
    no_errors(&scene.app);
    let told = said(&scene.app);
    assert!(
        told.contains(&text::touchup_applied("Oberfläche 002")),
        "{told:?}"
    );
    assert!(second.join("oberflaeche.json").is_file());
    let marks = scene.marks();
    assert!(touchup_fits(&second, &marks));
    assert!(second.join(TOUCHUP_DIR).join(PATCHES_FILE).is_file());
    let edited = edited_mesh(&second, &Progress::new()).unwrap().unwrap();
    assert!(
        !used(&edited).is_subset(&used(&glb(&second))),
        "moved points"
    );
    // it is shown, touched up; the first one still fits too
    let shown = scene.app.state().surface.as_ref().map(|s| s.path.clone());
    assert_eq!(shown, Some(second.join("oberflaeche.glb")));
    assert!(scene.app.view().has_edit());
    assert!(touchup_fits(&scene.surface, &marks));
}

#[test]
fn export_contains_touchup() {
    let mut scene = Scene::new("nacharbeit-app-export");
    let nr = scene.paint(front_wall(&scene.surface));
    // without a step the shown surface is exported as computed
    let sources: Vec<ExportSource> = export::shown_sources(scene.app.state())
        .into_iter()
        .map(|(s, _)| s)
        .collect();
    assert_eq!(sources, [ExportSource::Surface(scene.surface.clone())]);

    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    let shown = export::shown_sources(scene.app.state());
    let [(source, name)] = &shown[..] else {
        panic!("{shown:?}")
    };
    assert_eq!(*source, ExportSource::EditedSurface(scene.surface.clone()));
    assert!(name.contains("Nacharbeit"), "{name}");
    // as the tree offers it
    assert_eq!(
        export::surface_source(scene.app.state(), &scene.surface),
        ExportSource::EditedSurface(scene.surface.clone())
    );

    let out = scene.dir.join("ausgabe.glb");
    scene.run(Action::RunExport(ExportRequest {
        source: source.clone(),
        out: out.clone(),
        format: ExportFormat::Glb,
        overwrite: false,
    }));
    no_errors(&scene.app);
    let exported = read_mesh(&out, &Progress::new()).unwrap();
    let edited = edited_mesh(&scene.surface, &Progress::new())
        .unwrap()
        .unwrap();
    assert_eq!(used(&exported), used(&edited));
    assert_eq!(exported.pages.len(), edited.pages.len());
    assert_ne!(used(&exported), used(&glb(&scene.surface)));
}

#[test]
fn cleanup_keeps_marks() {
    let mut scene = Scene::new("nacharbeit-app-aufraeumen");
    let nr = scene.paint(front_wall(&scene.surface));
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    let marks_file = scene.map.join(MARKS_FILE);
    let (marks_before, patches_before) = (fs::read(&marks_file).unwrap(), scene.patches());
    // something a job cut off left beside them
    let left = scene.surface.join("oberflaeche.glb.tmp");
    fs::write(&left, b"halb").unwrap();
    scene.app.state_mut().leftover_quiet = Some(Duration::ZERO);
    scene.run(Action::OpenProject(scene.project.clone()));
    let offered: Vec<PathBuf> = scene
        .app
        .state()
        .leftovers_shown()
        .iter()
        .map(|l| l.path.clone())
        .collect();
    assert_eq!(offered, std::slice::from_ref(&left));

    scene.run(Action::CleanUp);
    no_errors(&scene.app);
    assert!(!left.exists());
    assert_eq!(fs::read(&marks_file).unwrap(), marks_before);
    assert_eq!(scene.patches(), patches_before);
    assert!(touchup_fits(&scene.surface, &scene.marks()));
}

#[test]
fn undo_step_recomputes() {
    let mut scene = Scene::new("nacharbeit-app-zuruecknehmen");
    let nr = scene.paint(front_wall(&scene.surface));
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    let strong = scene.patches();
    let shown = scene.app.state().touchup.shown;
    assert!(shown.is_some());
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Light,
    });
    no_errors(&scene.app);
    assert_ne!(scene.patches(), strong);
    assert_ne!(scene.app.state().touchup.shown, shown);

    // ⌘Z takes the step back: the touch-up is that of the strong smoothing again, shown
    scene.run(Action::UndoMark);
    no_errors(&scene.app);
    let marks = scene.marks();
    assert_eq!(marks.marks[0].steps.len(), 1);
    assert!(touchup_fits(&scene.surface, &marks));
    assert_eq!(scene.patches(), strong);
    assert_eq!(scene.app.state().touchup.shown, shown);

    // „Löschen“ takes the Stelle and its touch-up away …
    scene.run(Action::DeleteMark(nr));
    no_errors(&scene.app);
    assert!(scene.marks().marks.is_empty());
    assert!(!scene.cache().exists());
    assert!(!scene.app.view().has_edit());
    // … and ⌘Z brings both back
    scene.run(Action::UndoMark);
    no_errors(&scene.app);
    let marks = scene.marks();
    assert_eq!(marks.marks.len(), 1);
    assert_eq!(marks.marks[0].steps.len(), 1);
    assert!(touchup_fits(&scene.surface, &marks));
    assert_eq!(scene.patches(), strong);
    assert!(scene.app.view().has_edit());
}

/// The touch-up of the surface shown takes it as the loader read it: with its glb unreadable
/// (its size and time kept) smoothing and „Bilder neu“ give the same patches as on a readable
/// one, and the view gets the touched-up state. The window keeps the index of the scan's
/// images from the first step that baked them.
#[test]
fn touchup_takes_the_loaded_surface() {
    let mut want = Scene::new("nacharbeit-app-vorbereitet-soll");
    let nr = want.paint(front_wall(&want.surface));
    want.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    want.run(Action::Retexture { mark: nr });
    no_errors(&want.app);
    let bytes = want.patches();

    let mut scene = Scene::new("nacharbeit-app-vorbereitet");
    let sources = texture_sources(&scene.surface).unwrap().unwrap();
    let images = &sources[0].images;
    let nr = scene.paint(front_wall(&scene.surface));
    assert!(!scene
        .app
        .image_indices()
        .holds(&images.bags, &images.topics));
    let glb = scene.surface.join("oberflaeche.glb");
    let meta = fs::metadata(&glb).unwrap();
    fs::write(&glb, vec![0u8; meta.len() as usize]).unwrap();
    fs::File::options()
        .write(true)
        .open(&glb)
        .unwrap()
        .set_modified(meta.modified().unwrap())
        .unwrap();
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    assert!(scene.app.view().has_edit());
    assert!(scene
        .app
        .image_indices()
        .holds(&images.bags, &images.topics));
    scene.run(Action::Retexture { mark: nr });
    no_errors(&scene.app);
    assert_eq!(scene.patches(), bytes);
    let log: Vec<&str> = scene
        .app
        .state()
        .log
        .iter()
        .map(|l| l.text.as_str())
        .collect();
    assert!(
        log.iter().any(|l| l.starts_with("Bilder der Stelle: ")),
        "{log:?}"
    );
}

#[test]
fn new_map_starts_empty() {
    let mut scene = Scene::new("nacharbeit-app-neue-karte");
    let nr = scene.paint(front_wall(&scene.surface));
    scene.run(Action::Smooth {
        mark: nr,
        level: SmoothLevel::Strong,
    });
    no_errors(&scene.app);
    let before = fs::read(scene.map.join(MARKS_FILE)).unwrap();

    // another precision: a new map of the run, which begins without Stellen
    let voxel = scene.app.state().map_settings.switches.voxel;
    scene.app.state_mut().map_settings.switches.voxel = voxel * 2.0;
    let run = scene.app.state().scans[0].runs[0].dir.clone();
    scene.run(Action::RunMap(run.clone()));
    no_errors(&scene.app);
    let maps: Vec<PathBuf> = scene.app.state().scans[0].runs[0]
        .maps
        .iter()
        .map(|m| m.dir.clone())
        .collect();
    let second = run.join("karte-002");
    assert_eq!(maps, [scene.map.clone(), second.clone()]);
    assert!(!second.join(MARKS_FILE).exists());
    assert!(!scene.app.state().marks.contains_key(&second));
    // the old one keeps its own
    assert_eq!(fs::read(scene.map.join(MARKS_FILE)).unwrap(), before);
    assert!(scene.app.state().marks.contains_key(&scene.map));
    assert!(touchup_fits(&scene.surface, &scene.marks()));
}

/// The map of [`crafted`], not on disk.
const MAP: &str = "/p/scans/s/slam-001/karte-001";

/// A project (not read from disk) with one map and its surface shown, Stelle 1 painted on it and
/// chosen, the tool „Pinsel“.
fn crafted() -> AppState {
    let surface = PathBuf::from(format!("{MAP}/oberflaeche-001"));
    let map = MapInfo {
        dir: PathBuf::from(MAP),
        label: "Karte 001".into(),
        settings: Some(MapSwitches::default()),
        clouds: Vec::new(),
        surfaces: vec![SurfaceInfo {
            dir: surface.clone(),
            label: "Oberfläche 001".into(),
            stale: false,
            settings: Some(SurfaceSettings::default()),
            hints: Vec::new(),
        }],
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
    let mut marks = Marks::default();
    let dab = DabInput {
        centre: [1.0, 1.0, 1.0],
        normal: [0.0, -1.0, 0.0],
        radius: 0.2,
        erase: false,
    };
    let nr = marks.paint(None, &[dab]);
    let mut state = AppState {
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
        surface: Some(SurfaceSummary {
            path: surface.join("oberflaeche.glb"),
            triangles: 2000,
            pages: 1,
            origin: [0.0; 3],
            z_range: [0.0, 2.5],
        }),
        tool: Tool::Brush,
        ..AppState::default()
    };
    state.marks.insert(PathBuf::from(MAP), Ok(marks));
    state.brush.active = Some(nr);
    state
}

/// The bar of the brush alone, over a view filling the window; what it asked for last.
fn bar(state: AppState) -> Harness<'static, (AppState, Option<Action>)> {
    let mut h = Harness::builder().with_size([900.0, 300.0]).build_ui_state(
        |ui, (state, action): &mut (AppState, Option<Action>)| {
            let ctx = ui.ctx().clone();
            let view = ui.max_rect();
            let (_, asked) = brush::bar(&ctx, view, view.bottom() - 8.0, state);
            if asked.is_some() {
                *action = asked;
            }
        },
        (state, None),
    );
    h.run();
    h
}

/// Whether the button `name` of the bar is on.
fn on(h: &Harness<'_, (AppState, Option<Action>)>, name: &str) -> bool {
    !h.get_by_label(name).accesskit_node().is_disabled()
}

/// Clicks the button `name` of the bar: what it asks for.
fn click(h: &mut Harness<'_, (AppState, Option<Action>)>, name: &str) -> Option<Action> {
    h.state_mut().1 = None;
    h.get_by_label(name).click();
    h.run();
    h.state().1.clone()
}

/// Hovers the button `name` (off) and finds `why` in its tooltip.
fn off_with(h: &mut Harness<'_, (AppState, Option<Action>)>, name: &str, why: &str) {
    assert!(!on(h, name), "{name}");
    h.get_by_label(name).hover();
    h.run();
    h.get_by_label(why);
}

#[test]
fn bar_offers_the_steps() {
    let mut h = bar(crafted());
    let steps = [
        (text::SMOOTH_LIGHT_NAME, SmoothLevel::Light),
        (text::SMOOTH_STRONG_NAME, SmoothLevel::Strong),
        (text::SMOOTH_FLAT_NAME, SmoothLevel::Flat),
    ];
    for (name, level) in steps {
        assert_eq!(click(&mut h, name), Some(Action::Smooth { mark: 1, level }));
    }
    assert_eq!(
        click(&mut h, text::RETEXTURE),
        Some(Action::Retexture { mark: 1 })
    );
    assert_eq!(
        click(&mut h, text::FEEDBACK_BUTTON),
        Some(Action::OpenFeedback { mark: Some(1) })
    );
    assert!(on(&h, text::BRUSH_UNDO));
    // no touch-up yet: nothing to compare
    off_with(&mut h, text::EDIT_AFTER, text::EDIT_NONE);
    assert!(!on(&h, text::EDIT_BEFORE));

    // with a touch-up shown: „Nachher“ is drawn, „Vorher“ switches
    let mut state = crafted();
    state.touchup.shown = Some(7);
    state.touchup.edited = true;
    let mut h = bar(state);
    let toggled = |h: &Harness<'_, (AppState, Option<Action>)>, name: &str| {
        h.get_by_label(name).accesskit_node().toggled()
    };
    assert_eq!(toggled(&h, text::EDIT_AFTER), Some(Toggled::True));
    assert_eq!(toggled(&h, text::EDIT_BEFORE), Some(Toggled::False));
    assert_eq!(
        click(&mut h, text::EDIT_BEFORE),
        Some(Action::ShowEdit(false))
    );

    // no Stelle chosen: the steps wait for one; „Melden …“ reports none
    let mut state = crafted();
    state.brush.active = None;
    let mut h = bar(state);
    off_with(&mut h, text::SMOOTH_STRONG_NAME, text::STEPS_NEED_MARK);
    assert!(!on(&h, text::RETEXTURE));
    assert_eq!(
        click(&mut h, text::FEEDBACK_BUTTON),
        Some(Action::OpenFeedback { mark: None })
    );

    // the raw data gone: smoothing still works, the images not
    let mut state = crafted();
    state.scans[0].state = ScanState::Missing("Scan-Ordner nicht gefunden".into());
    let mut h = bar(state);
    assert!(on(&h, text::SMOOTH_FLAT_NAME));
    off_with(&mut h, text::RETEXTURE, text::RETEXTURE_NO_RAW_DATA);

    // while the touch-up of the surface runs or waits, the steps wait for it
    let surface = PathBuf::from(format!("{MAP}/oberflaeche-001"));
    let mut state = crafted();
    state.touchup.waiting.push_back((surface.clone(), None));
    let mut h = bar(state);
    off_with(&mut h, text::SMOOTH_LIGHT_NAME, text::TOUCHUP_RUNS);
    off_with(&mut h, text::RETEXTURE, text::TOUCHUP_RUNS);
    let mut state = crafted();
    state.running = Some(Action::Touchup {
        surface,
        step: None,
    });
    let h = bar(state);
    assert!(!on(&h, text::SMOOTH_STRONG_NAME));
}
