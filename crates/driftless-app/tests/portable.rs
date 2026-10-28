//! Projects that are renamed, moved or copied: the model still fits, its surface and „Von
//! Hand ausrichten“ still find the maps, a copy works on its own maps; a model written before
//! the relative paths is taken onto the project folder it lies in now.
//!
//! The project has two synthetic scans of one furnished room outside its folder
//! (`<dir>/Rohdaten/a`, `b`), a run and a map of each in the project, and a model of both.
//! It is made once; every test works on a copy of the whole folder.

use driftless_app::project::{
    canonical, model_status, run_map_job, run_merge_job, run_model_surface_job, run_slam_job,
    MergePick, ModelInfo, Project, SlamJob, SurfaceSettings,
};
use driftless_app::ui::{text, Action, DriftlessApp, UiMessage};
use driftless_gpu::GpuPref;
use driftless_io::progress::Progress;
use driftless_io::testdir::{SharedDir, TestDir};
use driftless_io::testscan::{furnished_room_bag, shared, tmp};
use driftless_map::surface::SurfaceConfig;
use driftless_steps::map::{MapSettings, CLEAN_BIN};
use driftless_steps::merge::MODEL_REPORT;
use driftless_steps::slam::SlamSettings;
use std::fs;
use std::path::Path;
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

const PROJECT: &str = "Halle 2026";

/// `<dir>/Rohdaten/{a,b}` and `<dir>/Halle 2026` with a run, a map and the model of both, made
/// once for all tests of this file.
fn template() -> &'static Path {
    static DIR: OnceLock<SharedDir> = OnceLock::new();
    DIR.get_or_init(|| {
        let dir = shared("portabel-vorlage");
        let raw = dir.join("Rohdaten");
        let (a, b) = (raw.join("a"), raw.join("b"));
        for d in [&a, &b] {
            fs::create_dir_all(d).unwrap();
        }
        furnished_room_bag(&a, true, [0.0; 3], 0.0);
        furnished_room_bag(&b, true, [0.8, 0.5, 0.0], 30f64.to_radians());
        let mut p = Project::create(&dir.join(PROJECT), "Halle").unwrap();
        for scan in [&a, &b] {
            p.add_scan(scan).unwrap();
        }
        p.save().unwrap();
        let mut picks = Vec::new();
        for s in p.scans.clone() {
            let full = SlamJob::Full(SlamSettings::default());
            let run = run_slam_job(&p, &s.id, &full, &Progress::new()).unwrap();
            let info = p
                .runs(&s.id)
                .into_iter()
                .find(|r| r.dir == run.run_dir)
                .unwrap();
            let map =
                run_map_job(&info, &s.path, &MapSettings::default(), &Progress::new()).unwrap();
            picks.push(MergePick {
                scan_id: s.id.clone(),
                run: run.run_dir,
                map: map.dir,
            });
        }
        run_merge_job(&p, &picks, false, &Progress::new()).unwrap();
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

/// A fresh copy of [`template`] as `<tmp>/<name>`.
fn copy(name: &str) -> TestDir {
    let dir = tmp(name);
    copy_dir(template(), &dir);
    dir
}

/// The only model of the project in `dir`, as the program lists it.
fn the_model(dir: &Path) -> (Project, ModelInfo) {
    let p = Project::open(dir).unwrap();
    let mut models = p.models();
    assert_eq!(models.len(), 1);
    (p, models.remove(0))
}

/// Every folder the model reads lies below `dir` (the scans outside it are not checked).
fn reads_below(model: &ModelInfo, dir: &Path) {
    let dir = canonical(dir).unwrap();
    for m in &model.report.members {
        for p in [&m.slam_dir, &m.map_dir] {
            assert!(
                p.starts_with(&dir),
                "{} not below {}",
                p.display(),
                dir.display()
            );
            assert!(p.is_dir(), "{}", p.display());
        }
    }
}

/// A small, gray surface of the model: computed, not reused.
fn surface_of(model: &ModelInfo) {
    let settings = SurfaceSettings {
        config: SurfaceConfig {
            voxel: 0.05,
            min_weight: 1,
            min_component_faces: 10,
            page_size: 256,
            simplify_error: 0.01,
            ..SurfaceConfig::default()
        },
        texture: false,
    };
    let done = run_model_surface_job(model, &settings, GpuPref::Off, &Progress::new()).unwrap();
    assert!(!done.reused);
    assert!(done.dir.join("oberflaeche.glb").is_file());
}

/// „Von Hand ausrichten“ of the second scan in the program: the session starts and the scan's
/// cloud is loaded, no error.
fn pairs_start(dir: &Path) {
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::OpenProject(dir.to_path_buf()));
    settle(&mut app);
    let model = app.state().models[0].dir.clone();
    app.apply(Action::StartPairs { model, member: 1 });
    settle(&mut app);
    let errors: Vec<&String> = app
        .state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e),
            _ => None,
        })
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
    assert!(!errors.iter().any(|e| e.contains(text::PAIRS_NO_CLOUD)));
    let session = app.state().pairs.as_ref().expect("pairs being set");
    assert!(session.shift.is_some(), "the scan is shown");
    assert!(app.cloud().is_some());
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

/// The renamed project: its model still fits, its surface and „Von Hand ausrichten“ find the
/// maps in the renamed folder. Then the project and the raw data are moved together: the
/// scans are found through their relative paths alone.
#[test]
fn renamed_project_keeps_its_model() {
    let dir = copy("portabel-umbenannt");
    let renamed = dir.join("Halle 2026 (abgeschlossen)");
    fs::rename(dir.join(PROJECT), &renamed).unwrap();
    let (_, model) = the_model(&renamed);
    reads_below(&model, &renamed);
    assert!(!model_status(&model).stale, "the model still fits its maps");
    surface_of(&model);
    pairs_start(&renamed);

    // Project and raw data moved together to another folder: the old paths are all gone.
    let moved = tmp("portabel-verschoben");
    fs::remove_dir_all(&moved).unwrap();
    fs::rename(&dir, &moved).unwrap();
    let project = moved.join("Halle 2026 (abgeschlossen)");
    let (p, model) = the_model(&project);
    let raw = canonical(&moved.join("Rohdaten")).unwrap();
    assert_eq!(p.scans[0].path, raw.join("a"));
    assert_eq!(p.scans[1].path, raw.join("b"));
    reads_below(&model, &project);
    for m in &model.report.members {
        assert!(m.scan.starts_with(&raw), "{}", m.scan.display());
    }
    assert!(!model_status(&model).stale);
    pairs_start(&project);
}

/// A copy next to the original works on its own maps, even when the original's no longer
/// fit.
#[test]
fn copied_project_uses_its_own_maps() {
    let dir = copy("portabel-kopie");
    let copy = dir.join("Halle 2026 Kopie");
    copy_dir(&dir.join(PROJECT), &copy);
    // The original's runs change, so its maps no longer fit (a model reading them would be
    // stale), and its stage files are broken.
    let (_, original) = the_model(&dir.join(PROJECT));
    for m in &original.report.members {
        let traj = m.slam_dir.join("traj.tum");
        let text = fs::read_to_string(&traj).unwrap();
        fs::write(&traj, format!("{text}# geändert\n")).unwrap();
        fs::write(m.map_dir.join(CLEAN_BIN), b"kaputt").unwrap();
    }
    assert!(model_status(&original).stale);

    let (_, model) = the_model(&copy);
    reads_below(&model, &copy);
    assert!(!model_status(&model).stale, "the copy reads its own maps");
    surface_of(&model);
    pairs_start(&copy);
}

/// A model written before the relative paths (only absolute ones in `modell.json`) of a
/// project renamed since: its folders are taken in the project folder it lies in now.
#[test]
fn old_model_is_rebased_after_a_move() {
    let dir = copy("portabel-alt");
    let project = dir.join(PROJECT);
    let (_, model) = the_model(&project);
    let file = model.dir.join(MODEL_REPORT);
    let mut json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&file).unwrap()).unwrap();
    for m in json["members"].as_array_mut().unwrap() {
        let m = m.as_object_mut().unwrap();
        for key in ["scan_rel", "slam_dir_rel", "map_dir_rel"] {
            assert!(m.remove(key).is_some(), "{key}");
        }
    }
    fs::write(&file, serde_json::to_string_pretty(&json).unwrap()).unwrap();

    let renamed = dir.join("Halle alt");
    fs::rename(&project, &renamed).unwrap();
    let (_, model) = the_model(&renamed);
    reads_below(&model, &renamed);
    assert!(!model_status(&model).stale);
    surface_of(&model);
    pairs_start(&renamed);
}

/// The paths as written: in `modell.json` relative to the model folder next to the absolute
/// ones, in `project.toml` the scans outside the project relative to it as well.
#[test]
fn relative_paths_are_written() {
    let dir = template();
    let project = canonical(&dir.join(PROJECT)).unwrap();
    let (p, model) = the_model(&project);
    let file = model.dir.join(MODEL_REPORT);
    let json: serde_json::Value = serde_json::from_str(&fs::read_to_string(file).unwrap()).unwrap();
    for (k, scan) in p.scans.iter().enumerate() {
        let m = &json["members"][k];
        let run = format!("../../scans/{}/slam-001", scan.id);
        assert_eq!(m["slam_dir_rel"], run.as_str());
        assert_eq!(m["map_dir_rel"], format!("{run}/karte-001").as_str());
        assert_eq!(
            m["scan_rel"],
            format!("../../../Rohdaten/{}", scan.id).as_str()
        );
        assert_eq!(
            Path::new(m["map_dir"].as_str().unwrap()),
            project
                .join("scans")
                .join(&scan.id)
                .join("slam-001")
                .join("karte-001")
        );
    }
    let toml: toml::Table =
        toml::from_str(&fs::read_to_string(project.join("project.toml")).unwrap()).unwrap();
    let scan = &toml["scan"][0];
    assert_eq!(
        Path::new(scan["path"].as_str().unwrap()),
        canonical(&dir.join("Rohdaten").join("a")).unwrap()
    );
    assert_eq!(scan["path_rel"].as_str(), Some("../Rohdaten/a"));
}
