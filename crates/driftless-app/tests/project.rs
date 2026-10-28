//! The project model on synthetic scans: `project.toml`, run folders, map folders, CLI
//! folders added as runs, missing scan folders, models merged from several scans.

use driftless_app::leftovers;
use driftless_app::project::{
    map_dir_for, model_picks, model_status, model_surface_status, point_pairs, restore_job,
    run_map_job, run_merge_job, run_model_surface_job, run_pairs_merge_job, run_slam_job,
    set_point_pair_enabled, MergePick, Project, ScanState, SlamJob, SurfaceSettings,
};
use driftless_app::ui::read_tree;
use driftless_gpu::GpuPref;
use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testdir::SharedDir;
use driftless_io::testscan::{furnished_room_bag, room_bag_at, room_bag_secs, shared, tmp};
use driftless_map::surface::SurfaceConfig;
use driftless_steps::correct;
use driftless_steps::history::HistoryKind;
use driftless_steps::map::{
    map_input_for, run_map, MapInput, MapRequest, MapSettings, MapStep, MapSwitches,
};
use driftless_steps::merge::{
    member_transform, read_model, MERGE_VERSION, MODEL_LAS, MODEL_REPORT,
};
use driftless_steps::pairs::{PointPairEntry, PointPairs, POINT_PAIRS_FILE};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::StepError;
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};

fn canon(p: &Path) -> PathBuf {
    driftless_app::project::canonical(p).unwrap()
}

/// A scan folder `name` under `dir` with a static room scan of `secs` seconds.
fn scan(dir: &Path, name: &str, secs: f64) -> PathBuf {
    let s = dir.join(name);
    fs::create_dir_all(&s).unwrap();
    room_bag_secs(&s, true, secs);
    s
}

/// A run folder with just a `slam.json` that starts with the scans of [`scan`].
fn fake_run(dir: &Path) {
    fs::create_dir_all(dir).unwrap();
    fs::write(
        dir.join("slam.json"),
        r#"{"scan_start": 1000.0, "submaps": []}"#,
    )
    .unwrap();
}

/// Writes `dir/project.toml` by hand with the scans `(id, path)` (TOML literal strings).
fn hand_written(dir: &Path, scans: &[(&str, &str)]) {
    fs::create_dir_all(dir).unwrap();
    let mut text = "format = 1\nname = 'P'\n".to_string();
    for (id, path) in scans {
        text += &format!("\n[[scan]]\nid = '{id}'\npath = '{path}'\nname = 'S'\n");
    }
    fs::write(dir.join("project.toml"), text).unwrap();
}

/// A run as `driftless slam <scan> -o <out>` makes it (no history, no `lauf.toml`).
fn cli_slam(scan: &Path, out: &Path, reuse: bool) {
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
}

/// Files directly in `dir` (name → contents).
fn files(dir: &Path) -> BTreeMap<String, Vec<u8>> {
    fs::read_dir(dir)
        .map(|entries| {
            entries
                .map(|e| e.unwrap().path())
                .filter(|p| p.is_file())
                .map(|p| {
                    let name = p.file_name().unwrap().to_string_lossy().into_owned();
                    (name, fs::read(&p).unwrap())
                })
                .collect()
        })
        .unwrap_or_default()
}

fn names(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    v.sort();
    v
}

fn settings(step: MapStep) -> MapSettings {
    MapSettings {
        step,
        ..MapSettings::default()
    }
}

/// A project in `dir/projekt` with one scan of `secs` seconds and one full run (`slam-001`).
fn project_with_run(dir: &Path, secs: f64) -> (Project, String) {
    let raw = scan(dir, "rohdaten", secs);
    let mut p = Project::create(&dir.join("projekt"), "Test").unwrap();
    let id = p.add_scan(&raw).unwrap();
    p.save().unwrap();
    run_slam_job(
        &p,
        &id,
        &SlamJob::Full(SlamSettings::default()),
        &Progress::new(),
    )
    .unwrap();
    (p, id)
}

#[test]
fn create_open_round_trip() {
    let dir = tmp("app-round-trip");
    let raw = scan(&dir, "buero", 0.5);
    let mut p = Project::create(&dir.join("projekt"), "Büro").unwrap();
    assert_eq!(p.name, "Büro");
    assert!(p.scans.is_empty());
    let id = p.add_scan(&raw).unwrap();
    assert_eq!(id, "buero");
    p.scans[0].name = "Büro, Erdgeschoss".into();
    p.save().unwrap();

    let q = Project::open(&dir.join("projekt")).unwrap();
    assert_eq!(q, p);
    assert_eq!(q.dir, canon(&dir.join("projekt")));
    assert_eq!(q.scans[0].path, canon(&raw));
    assert_eq!(q.scans[0].name, "Büro, Erdgeschoss");
    let text = fs::read_to_string(dir.join("projekt/project.toml")).unwrap();
    let t: toml::Table = toml::from_str(&text).unwrap();
    assert_eq!(t["format"].as_integer(), Some(1));
    assert_eq!(t["name"].as_str(), Some("Büro"));
    assert_eq!(t["scan"][0]["id"].as_str(), Some("buero"));
    assert!(!dir.join("projekt/project.toml.tmp").exists());

    // a folder without a project
    let err = Project::open(&dir).unwrap_err().to_string();
    assert!(err.contains("project.toml"), "{err}");
}

#[test]
fn create_refuses_existing_project() {
    let dir = tmp("app-create-twice");
    Project::create(&dir.join("p"), "eins").unwrap();
    let err = Project::create(&dir.join("p"), "zwei").unwrap_err();
    assert!(
        err.to_string().starts_with("Hier ist schon ein Projekt"),
        "{err}"
    );
    assert_eq!(Project::open(&dir.join("p")).unwrap().name, "eins");

    // a folder with other files is not taken
    fs::create_dir_all(dir.join("voll")).unwrap();
    fs::write(dir.join("voll/notiz.txt"), "x").unwrap();
    let err = Project::create(&dir.join("voll"), "drei").unwrap_err();
    assert!(err.to_string().contains("nicht leer"), "{err}");
    assert!(!dir.join("voll/project.toml").exists());

    // an empty one is
    fs::create_dir_all(dir.join("leer")).unwrap();
    Project::create(&dir.join("leer"), "vier").unwrap();
}

#[test]
fn scan_ids_are_unique() {
    let dir = tmp("app-scan-ids");
    let a = scan(&dir.join("a"), "scan", 0.5);
    let b = scan(&dir.join("b"), "scan", 0.5);
    let c = scan(&dir.join("c"), "scan", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    assert_eq!(p.add_scan(&a).unwrap(), "scan");
    assert_eq!(p.add_scan(&b).unwrap(), "scan-2");
    assert_eq!(p.add_scan(&c).unwrap(), "scan-3");

    // a folder without LiDAR bags is no scan
    fs::create_dir_all(dir.join("leer")).unwrap();
    let err = p.add_scan(&dir.join("leer")).unwrap_err().to_string();
    assert!(err.contains("LIDAR_"), "{err}");
    assert_eq!(p.scans.len(), 3);
}

#[test]
fn same_scan_twice_is_refused() {
    let dir = tmp("app-scan-twice");
    let a = scan(&dir, "scan", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    p.add_scan(&a).unwrap();
    // also through another spelling of the same folder
    let err = p.add_scan(&dir.join("p/../scan")).unwrap_err().to_string();
    assert!(err.contains("schon im Projekt"), "{err}");
    assert_eq!(p.scans.len(), 1);
}

#[test]
fn runs_are_numbered_and_sorted() {
    let dir = tmp("app-runs-sorted");
    let raw = scan(&dir, "scan", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id = p.add_scan(&raw).unwrap();
    let base = dir.join("p/scans").join(&id);
    for n in ["slam-010", "slam-002", "slam-001", "notizen"] {
        fs::create_dir_all(base.join(n)).unwrap();
    }
    // a CLI folder outside the project
    let cli = dir.join("cli-lauf");
    fs::create_dir_all(&cli).unwrap();
    fake_run(&cli);
    p.add_run(&id, &cli).unwrap();

    let labels: Vec<String> = p.runs(&id).into_iter().map(|r| r.label).collect();
    assert_eq!(labels, ["SLAM 001", "SLAM 002", "SLAM 010", "cli-lauf"]);
    let run = &p.runs(&id)[1];
    assert_eq!(run.dir, canon(&base.join("slam-002")));
    assert!(!run.has_result && !run.has_cache && !run.has_previous);
    assert_eq!(run.settings, SlamSettings::default());

    // the next run gets the next number, with its settings in lauf.toml
    let s = SlamSettings {
        lidar_topic: Some("/lidar".into()),
        lidar_to_imu: Some([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        imu_topic: None,
    };
    let new = p.new_run_dir(&id, &s).unwrap();
    assert_eq!(new.path(), canon(&base).join("slam-011"));
    assert!(new.is_new());
    let path = new.keep();
    let t: toml::Table =
        toml::from_str(&fs::read_to_string(path.join("lauf.toml")).unwrap()).unwrap();
    assert!(t["erstellt"].as_str().unwrap().ends_with('Z'));
    assert_eq!(t["scan"].as_str(), Some(canon(&raw).to_str().unwrap()));
    assert_eq!(t["slam"]["lidar_topic"].as_str(), Some("/lidar"));
    let runs = p.runs(&id);
    assert_eq!(runs[3].label, "SLAM 011");
    assert_eq!(runs[3].settings, s);

    // a run folder that is not kept is removed again
    let dropped = p.new_run_dir(&id, &s).unwrap();
    let dropped_path = dropped.path().to_path_buf();
    assert!(dropped_path.ends_with("slam-012") && dropped_path.exists());
    drop(dropped);
    assert!(!dropped_path.exists());
}

#[test]
fn cli_folder_can_be_added_as_run() {
    let dir = tmp("app-cli-run");
    let raw = scan(&dir, "scan", 1.5);
    let out = dir.join("cli-ausgabe");
    cli_slam(&raw, &out, false);
    // `driftless map <scan> --slam <out>` writes into the SLAM folder
    let req = MapRequest {
        input: MapInput {
            scan: raw.clone(),
            slam_dir: out.clone(),
            slam: SlamSettings::default(),
            map: MapSettings::default(),
        },
        out_dir: out.clone(),
        all_files: false,
    };
    run_map(&req, &Progress::new()).unwrap();

    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id = p.add_scan(&raw).unwrap();
    p.add_run(&id, &out).unwrap();
    let err = p.add_run(&id, &out).unwrap_err().to_string();
    assert!(err.contains("schon im Projekt"), "{err}");
    let err = p.add_run(&id, &raw).unwrap_err().to_string();
    assert!(err.contains("slam.json"), "{err}");
    let err = p.add_run("gibt-es-nicht", &out).unwrap_err().to_string();
    assert!(err.contains("gibt-es-nicht"), "{err}");
    p.save().unwrap();

    let p = Project::open(&dir.join("p")).unwrap();
    let runs = p.runs(&id);
    assert_eq!(runs.len(), 1);
    let run = &runs[0];
    assert_eq!(run.label, "cli-ausgabe");
    assert_eq!(run.settings, SlamSettings::default());
    assert!(run.has_result && run.has_cache && !run.has_previous);
    assert_eq!(run.cloud, Some(canon(&out).join("map.las")));
    assert!(run.history.is_empty());
    assert_eq!(run.maps.len(), 1);
    let map = &run.maps[0];
    assert_eq!(map.label, "Karte (im Lauf-Ordner)");
    assert_eq!(map.dir, run.dir);
    assert_eq!(map.settings, Some(MapSwitches::default()));
    // no camera images: no color.las, the cleaned cloud to look at, and the removed points
    let clouds: Vec<String> = map
        .clouds
        .iter()
        .map(|c| c.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(clouds, ["clean.las", "removed.las"]);
}

#[test]
fn reuse_of_added_cli_run_keeps_its_trajectory() {
    // two submaps, so a manual loop moves the trajectory
    let dir = tmp("app-cli-reuse");
    let raw = scan(&dir, "scan", 12.0);
    let out = dir.join("cli");
    cli_slam(&raw, &out, false);
    let traj = fs::read(out.join("traj.tum")).unwrap();
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id = p.add_scan(&raw).unwrap();
    p.add_run(&id, &out).unwrap();
    correct::add_loop(&out, 1.0, 11.0, "").unwrap();

    let run = p.runs(&id).remove(0);
    let done = run_slam_job(&p, &id, &SlamJob::Reuse(run), &Progress::new()).unwrap();
    assert_eq!(done.run_dir, canon(&out));
    let run = p.runs(&id).remove(0);
    let kinds: Vec<HistoryKind> = run.history.iter().map(|h| h.kind).collect();
    assert_eq!(kinds, [HistoryKind::Imported, HistoryKind::Reuse]);
    assert_eq!(fs::read(run.history[0].dir.join("traj.tum")).unwrap(), traj);
    assert_eq!(fs::read(out.join("traj.vorher.tum")).unwrap(), traj);
    assert!(run.has_previous);
}

#[test]
fn slam_job_makes_a_new_run_folder_each_time() {
    let dir = tmp("app-slam-new");
    let (p, id) = project_with_run(&dir, 1.5);
    let first = p.runs(&id).remove(0);
    let before = files(&first.dir);
    let done = run_slam_job(
        &p,
        &id,
        &SlamJob::Full(SlamSettings::default()),
        &Progress::new(),
    )
    .unwrap();
    assert!(done.run_dir.ends_with("slam-002"));
    assert_eq!(files(&first.dir), before, "slam-001 untouched");
    let runs = p.runs(&id);
    assert_eq!(runs.len(), 2);
    for r in &runs {
        assert!(r.has_result && r.has_cache, "{}", r.label);
        assert!(r.dir.join("lauf.toml").exists());
        let kinds: Vec<HistoryKind> = r.history.iter().map(|h| h.kind).collect();
        assert_eq!(kinds, [HistoryKind::Full]);
    }
}

#[test]
fn cancelled_jobs_leave_no_folder() {
    let dir = tmp("app-cancel");
    let (p, id) = project_with_run(&dir, 1.5);
    let base = p.dir.join("scans").join(&id);
    let p2 = Progress::new();
    let handle = p2.clone();
    let p2 = p2.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "LiDAR lesen" && s.done >= 3 {
            handle.cancel();
        }
    });
    let err = run_slam_job(&p, &id, &SlamJob::Full(SlamSettings::default()), &p2).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(names(&base), ["slam-001"]);

    // a map into a new folder, cancelled: the folder goes again
    let run = p.runs(&id).remove(0);
    let scan_dir = p.scans[0].path.clone();
    let p3 = Progress::new();
    let handle = p3.clone();
    let p3 = p3.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "LiDAR lesen" && s.done >= 3 {
            handle.cancel();
        }
    });
    let err = run_map_job(&run, &scan_dir, &settings(MapStep::All), &p3).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(!run.dir.join("karte-001").exists());
    // a failing one too (no colouring possible without images)
    let err = run_map_job(&run, &scan_dir, &settings(MapStep::Color), &Progress::new());
    assert!(err.is_err());
    assert!(!run.dir.join("karte-001").exists());
}

#[test]
fn missing_scan_folder_is_reported_not_fatal() {
    let dir = tmp("app-missing-scan");
    let (p, id) = project_with_run(&dir, 1.5);
    assert_eq!(p.scan_state(&id), ScanState::Ready);
    let raw = p.scans[0].path.clone();
    fs::remove_dir_all(&raw).unwrap();

    let p = Project::open(&p.dir).unwrap();
    match p.scan_state(&id) {
        ScanState::Missing(why) => {
            // the reason first, then the folder
            assert!(why.starts_with("Scan-Ordner nicht gefunden: "), "{why}");
            assert!(why.contains("rohdaten"), "{why}");
        }
        ScanState::Ready => panic!("scan folder is gone"),
    }
    let runs = p.runs(&id);
    assert_eq!(runs.len(), 1);
    assert!(runs[0].has_result && runs[0].has_cache);
    assert_eq!(runs[0].history.len(), 1);
    assert!(matches!(p.scan_state("unbekannt"), ScanState::Missing(_)));
}

#[test]
fn run_without_cache_cannot_reuse() {
    let dir = tmp("app-no-cache");
    let (p, id) = project_with_run(&dir, 1.5);
    let run = p.runs(&id).remove(0);
    assert!(run.has_cache);
    fs::remove_file(run.dir.join("submaps.bin")).unwrap();
    let run = p.runs(&id).remove(0);
    assert!(!run.has_cache);
    assert!(run.has_result);
}

#[test]
fn map_dir_reused_when_settings_and_traj_fit() {
    let dir = tmp("app-map-fits");
    let (p, id) = project_with_run(&dir, 1.5);
    let run = p.runs(&id).remove(0);
    let scan_dir = p.scans[0].path.clone();
    let done = run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-001"));
    let k1 = files(&done.dir);

    // the same map again: current, nothing computed or written, the same folder
    let run = p.runs(&id).remove(0);
    let done = run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-001"));
    assert_eq!(files(&done.dir), k1);

    // map_dir_for on its own: the current folder, nothing created
    let input = MapInput {
        scan: scan_dir.clone(),
        slam_dir: run.dir.clone(),
        slam: run.settings.clone(),
        map: settings(MapStep::All),
    };
    let target = map_dir_for(&run, &input, &Progress::new()).unwrap();
    assert!(!target.is_new());
    assert_eq!(target.path(), run.dir.join("karte-001"));
    drop(target);
    assert!(run.dir.join("karte-001/map.json").exists(), "kept on drop");

    // same switches, another step: it writes, so never into the map's own folder – a new one,
    // with the stage it takes over linked from the old one, which stays as it was
    let done = run_map_job(&run, &scan_dir, &settings(MapStep::Clean), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-002"));
    assert_eq!(done.report["steps"]["dense"]["reused"], true);
    assert_eq!(
        fs::read(done.dir.join("dense.bin")).unwrap(),
        k1["dense.bin"]
    );
    assert_eq!(files(&run.dir.join("karte-001")), k1, "karte-001 untouched");
    let run = p.runs(&id).remove(0);
    let labels: Vec<&str> = run.maps.iter().map(|m| m.label.as_str()).collect();
    assert_eq!(labels, ["Karte 001", "Karte 002"]);
    assert_eq!(run.maps[0].settings, Some(MapSwitches::default()));
}

/// A map whose files no longer fit it all (here `dense.bin`, deleted to save space) is
/// computed anew into a new folder, never into its own: cancelled in the clean step, the old
/// map keeps its report and its surface and stays in the tree, and nothing is offered for
/// „Aufräumen“ (the review's probe of I1). Computed to its end, the new map has a folder of
/// its own as well.
#[test]
fn cancelled_map_recompute_keeps_the_map() {
    let dir = tmp("app-map-recompute-cancel");
    let (p, id) = project_with_run(&dir, 1.5);
    let scan_dir = p.scans[0].path.clone();
    let run = p.runs(&id).remove(0);
    run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    let k1 = run.dir.join("karte-001");
    let surface = k1.join("oberflaeche-001");
    fs::create_dir_all(&surface).unwrap();
    fs::write(surface.join("oberflaeche.glb"), b"glb").unwrap();
    fs::write(surface.join("oberflaeche.json"), b"{}").unwrap();
    fs::remove_file(k1.join("dense.bin")).unwrap();
    let before = files(&k1);

    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Geister" {
            handle.cancel();
        }
    });
    let run = p.runs(&id).remove(0);
    let err = run_map_job(&run, &scan_dir, &settings(MapStep::All), &progress).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(files(&k1), before, "karte-001 untouched");
    assert_eq!(files(&surface).len(), 2, "its surface too");
    assert!(
        !run.dir.join("karte-002").exists(),
        "the new folder goes again"
    );
    let tree = read_tree(&p);
    let run_node = &tree.scans[0].runs[0];
    let maps: Vec<&Path> = run_node.maps.iter().map(|m| m.dir.as_path()).collect();
    assert_eq!(maps, [k1.as_path()]);
    assert!(!run_node.maps[0].incomplete);
    assert_eq!(run_node.maps[0].surfaces.len(), 1);
    assert_eq!(
        leftovers::find(&p.dir, std::slice::from_ref(&id)),
        Vec::new()
    );

    let done = run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-002"));
    assert_eq!(files(&k1), before, "karte-001 untouched");
}

/// A reuse of a run whose `map.las` cannot be written (as on a full disk) puts every file of
/// the run back: it keeps its report, its trajectory and its history, stays in the tree with
/// its map, and nothing is offered for „Aufräumen“ (the review's probe of I1).
#[test]
fn failed_reuse_keeps_the_run() {
    let dir = tmp("app-reuse-fails");
    let (p, id) = project_with_run(&dir, 1.5);
    let scan_dir = p.scans[0].path.clone();
    let run = p.runs(&id).remove(0);
    run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    let run = p.runs(&id).remove(0);
    let before = files(&run.dir);
    let states = run.history.len();
    // a folder in the way of the temporary file of the new map.las: its write fails
    let blocked = run.dir.join("map.las.tmp");
    fs::create_dir_all(blocked.join("x")).unwrap();
    let err = run_slam_job(&p, &id, &SlamJob::Reuse(run.clone()), &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("wie vorher"), "{err}");
    fs::remove_dir_all(&blocked).unwrap();
    assert_eq!(files(&run.dir), before);
    let tree = read_tree(&p);
    let node = &tree.scans[0].runs[0];
    assert!(node.has_result && !node.incomplete);
    assert_eq!(node.maps.len(), 1);
    assert_eq!(node.history.len(), states);
    assert_eq!(leftovers::find(&p.dir, &[id]), Vec::new());
}

/// A reuse and „Diesen Stand wiederherstellen“ write into the run's own folder: while they
/// run, the folder carries the busy mark, so another window neither offers it for „Aufräumen“
/// nor deletes it; afterwards the mark is gone.
#[test]
fn jobs_in_the_run_folder_mark_it_busy() {
    let dir = tmp("app-reuse-busy");
    let (p, id) = project_with_run(&dir, 1.5);
    let run = p.runs(&id).remove(0);
    let mark = run.dir.join(leftovers::BUSY_FILE);
    let watching = |seen: Arc<AtomicBool>| {
        let mark = mark.clone();
        Progress::new().with_observer(move |s: &ProgressSnapshot| {
            if s.phase == "Schleifen schließen" && mark.is_file() {
                seen.store(true, Ordering::SeqCst);
            }
        })
    };
    let seen = Arc::new(AtomicBool::new(false));
    run_slam_job(
        &p,
        &id,
        &SlamJob::Reuse(run.clone()),
        &watching(seen.clone()),
    )
    .unwrap();
    assert!(seen.load(Ordering::SeqCst), "marked while it ran");
    assert!(!mark.exists(), "the mark goes afterwards");

    let seen = Arc::new(AtomicBool::new(false));
    let run = p.runs(&id).remove(0);
    restore_job(&p, &id, &run, 1, &watching(seen.clone())).unwrap();
    assert!(seen.load(Ordering::SeqCst), "marked while it ran");
    assert!(!mark.exists(), "the mark goes afterwards");
}

#[test]
fn new_map_dir_on_changed_settings_copies_bins() {
    let dir = tmp("app-map-new");
    let (p, id) = project_with_run(&dir, 1.5);
    let scan_dir = p.scans[0].path.clone();
    let run = p.runs(&id).remove(0);
    run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
    let k1 = run.dir.join("karte-001");
    let before = files(&k1);

    // other cleaning: a new folder; dense.bin still fits and is taken over, clean.bin not
    let mut s = settings(MapStep::Clean);
    s.switches.ghosts = false;
    let run = p.runs(&id).remove(0);
    let done = run_map_job(&run, &scan_dir, &s, &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-002"));
    assert_eq!(done.report["steps"]["dense"]["reused"], true);
    assert_eq!(done.report["steps"]["clean"]["reused"], false);
    assert_eq!(
        fs::read(done.dir.join("dense.bin")).unwrap(),
        before["dense.bin"]
    );
    assert_ne!(
        fs::read(done.dir.join("clean.bin")).unwrap(),
        before["clean.bin"]
    );
    assert_eq!(files(&k1), before, "karte-001 untouched");

    // other voxel size: dense.bin does not fit, nothing is taken over
    let mut s = settings(MapStep::Dense);
    s.switches.voxel *= 2.0;
    let run = p.runs(&id).remove(0);
    let input = MapInput {
        scan: scan_dir.clone(),
        slam_dir: run.dir.clone(),
        slam: run.settings.clone(),
        map: MapSettings {
            step: MapStep::Clean,
            ..s.clone()
        },
    };
    let target = map_dir_for(&run, &input, &Progress::new()).unwrap();
    assert!(target.is_new());
    assert_eq!(target.path(), run.dir.join("karte-003"));
    // only the mark of the job that writes into it
    assert_eq!(
        names(target.path()),
        [driftless_app::leftovers::BUSY_FILE],
        "{:?}",
        names(target.path())
    );
    drop(target);
    assert!(!run.dir.join("karte-003").exists());

    let run = p.runs(&id).remove(0);
    let labels: Vec<&str> = run.maps.iter().map(|m| m.label.as_str()).collect();
    assert_eq!(labels, ["Karte 001", "Karte 002"]);
    let ghosts_off = MapSwitches {
        ghosts: false,
        ..MapSwitches::default()
    };
    assert_eq!(run.maps[1].settings, Some(ghosts_off));
}

#[test]
fn new_map_dir_after_reuse() {
    // two submaps, so a manual loop moves the trajectory
    let dir = tmp("app-map-reuse");
    let (p, id) = project_with_run(&dir, 12.0);
    let scan_dir = p.scans[0].path.clone();
    let run = p.runs(&id).remove(0);
    run_map_job(&run, &scan_dir, &settings(MapStep::Dense), &Progress::new()).unwrap();
    let k1 = run.dir.join("karte-001");
    let before = files(&k1);

    let traj = fs::read(run.dir.join("traj.tum")).unwrap();
    correct::add_loop(&run.dir, 1.0, 11.0, "").unwrap();
    run_slam_job(&p, &id, &SlamJob::Reuse(run), &Progress::new()).unwrap();
    let run = p.runs(&id).remove(0);
    assert_ne!(fs::read(run.dir.join("traj.tum")).unwrap(), traj);
    assert!(run.has_previous);

    // dense.bin belongs to the old trajectory: not taken over
    let done = run_map_job(&run, &scan_dir, &settings(MapStep::Clean), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-002"));
    assert_eq!(done.report["steps"]["dense"]["reused"], false);
    assert_eq!(files(&k1), before, "karte-001 untouched");
}

#[test]
fn relative_and_windows_style_paths() {
    let dir = tmp("app-paths");
    let pdir = dir.join("p");
    let mut p = Project::create(&pdir, "P").unwrap();
    let inside = scan(&pdir.join("rohdaten"), "innen", 0.5);
    let outside = scan(&dir, "aussen", 0.5);
    let a = p.add_scan(&inside).unwrap();
    let b = p.add_scan(&outside).unwrap();
    let cli = pdir.join("cli").join("lauf");
    fs::create_dir_all(&cli).unwrap();
    fake_run(&cli);
    p.add_run(&a, &cli).unwrap();
    p.save().unwrap();

    let text = fs::read_to_string(pdir.join("project.toml")).unwrap();
    let t: toml::Table = toml::from_str(&text).unwrap();
    assert_eq!(t["scan"][0]["path"].as_str(), Some("rohdaten/innen"));
    assert_eq!(t["scan"][0]["extra_runs"][0].as_str(), Some("cli/lauf"));
    let abs = t["scan"][1]["path"].as_str().unwrap();
    assert!(Path::new(abs).is_absolute(), "{abs}");
    assert_eq!(Path::new(abs), canon(&outside));

    // written on Windows: backslashes in relative paths
    let text = text
        .replace("\"rohdaten/innen\"", "'rohdaten\\innen'")
        .replace("\"cli/lauf\"", "'cli\\lauf'");
    fs::write(pdir.join("project.toml"), text).unwrap();
    let q = Project::open(&pdir).unwrap();
    assert_eq!(q.scans[0].path, canon(&inside));
    assert_eq!(q.scans[0].extra_runs, [canon(&cli)]);
    assert_eq!(q.scan_state(&a), ScanState::Ready);
    assert_eq!(q.scan_state(&b), ScanState::Ready);

    // the whole project moved: relative paths move along
    let moved = dir.join("verschoben");
    fs::rename(&pdir, &moved).unwrap();
    let r = Project::open(&moved).unwrap();
    assert_eq!(
        r.scans[0].path,
        canon(&moved).join("rohdaten").join("innen")
    );
    assert_eq!(r.scan_state(&a), ScanState::Ready);
}

/// A scan and a run outside the project: absolute, and relative to the project beside it
/// (also in `lauf.toml`). The relative path is read first where it exists, else the absolute
/// one; a list of relative run paths that does not match the runs is ignored.
#[test]
fn outside_paths_are_kept_relative_too() {
    let dir = tmp("app-paths-outside");
    let pdir = dir.join("projekte").join("p");
    let mut p = Project::create(&pdir, "P").unwrap();
    let outside = scan(&dir.join("rohdaten"), "aussen", 0.5);
    let id = p.add_scan(&outside).unwrap();
    let cli = dir.join("cli").join("lauf");
    fake_run(&cli);
    p.add_run(&id, &cli).unwrap();
    p.save().unwrap();
    let text = fs::read_to_string(pdir.join("project.toml")).unwrap();
    let t: toml::Table = toml::from_str(&text).unwrap();
    let s = &t["scan"][0];
    assert_eq!(Path::new(s["path"].as_str().unwrap()), canon(&outside));
    assert_eq!(s["path_rel"].as_str(), Some("../../rohdaten/aussen"));
    assert_eq!(Path::new(s["extra_runs"][0].as_str().unwrap()), canon(&cli));
    assert_eq!(s["extra_runs_rel"][0].as_str(), Some("../../cli/lauf"));
    let run = p.new_run_dir(&id, &SlamSettings::default()).unwrap();
    let lauf: toml::Table =
        toml::from_str(&fs::read_to_string(run.path().join("lauf.toml")).unwrap()).unwrap();
    assert_eq!(Path::new(lauf["scan"].as_str().unwrap()), canon(&outside));
    assert_eq!(lauf["scan_rel"].as_str(), Some("../../rohdaten/aussen"));
    drop(run);

    // Everything moved together: only the relative paths lead to the folders.
    let moved = tmp("app-paths-outside-moved");
    fs::remove_dir_all(&moved).unwrap();
    fs::rename(&dir, &moved).unwrap();
    let q = Project::open(&moved.join("projekte").join("p")).unwrap();
    assert_eq!(
        q.scans[0].path,
        canon(&moved.join("rohdaten").join("aussen"))
    );
    assert_eq!(
        q.scans[0].extra_runs,
        [canon(&moved.join("cli").join("lauf"))]
    );
    assert_eq!(q.scan_state(&id), ScanState::Ready);
    // Saved again: the paths of the new place.
    q.save().unwrap();
    let r = Project::open(&moved.join("projekte").join("p")).unwrap();
    assert_eq!(r.scans, q.scans);

    // The project alone moved elsewhere: the relative paths lead nowhere, the absolute ones
    // are taken.
    let alone = moved.join("woanders").join("tiefer").join("p");
    fs::create_dir_all(alone.parent().unwrap()).unwrap();
    fs::rename(moved.join("projekte").join("p"), &alone).unwrap();
    let r = Project::open(&alone).unwrap();
    assert_eq!(r.scans[0].path, q.scans[0].path);
    assert_eq!(r.scans[0].extra_runs, q.scans[0].extra_runs);

    // A hand edit left another number of relative run paths than runs: they are not used,
    // although the first would lead to a folder (another one).
    let file = alone.join("project.toml");
    let mut t: toml::Table = toml::from_str(&fs::read_to_string(&file).unwrap()).unwrap();
    let wrong = toml::Value::Array(vec!["../../../rohdaten/aussen".into(), "".into()]);
    t["scan"].as_array_mut().unwrap()[0]
        .as_table_mut()
        .unwrap()
        .insert("extra_runs_rel".into(), wrong);
    fs::write(&file, toml::to_string(&t).unwrap()).unwrap();
    assert!(alone.join("../../../rohdaten/aussen").is_dir());
    assert_eq!(
        Project::open(&alone).unwrap().scans[0].extra_runs,
        q.scans[0].extra_runs
    );
}

#[test]
fn bad_scan_ids_are_refused() {
    let dir = tmp("app-bad-ids");
    let bad = [
        "", ".", "..", "../x", "a/b", "a\\b", "/abs", "C:\\x", "c:", "a:b", "x/..", ".. ", "...",
        "scan.", "scan ", "NUL", "con.txt", "Com1",
    ];
    for (n, id) in bad.iter().enumerate() {
        let pdir = dir.join(format!("p{n}"));
        hand_written(&pdir, &[(id, "rohdaten")]);
        let err = Project::open(&pdir).unwrap_err().to_string();
        assert!(
            err.contains("Scan-Kennung") && err.contains(&format!("„{id}“")),
            "{id}: {err}"
        );
    }
    hand_written(&dir.join("gut"), &[("scan-1", "rohdaten")]);
    Project::open(&dir.join("gut")).unwrap();
}

#[cfg(unix)]
#[test]
fn add_scan_refuses_a_folder_name_that_is_no_id() {
    let dir = tmp("app-bad-folder-name");
    let raw = scan(&dir, "a:b", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let err = p.add_scan(&raw).unwrap_err().to_string();
    assert!(
        err.contains("Scan-Kennung") && err.contains("„a:b“"),
        "{err}"
    );
    assert!(p.scans.is_empty());
}

#[test]
fn runs_stay_inside_the_project() {
    let dir = tmp("app-runs-guard");
    let raw = scan(&dir, "scan", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    p.add_scan(&raw).unwrap();
    // `scans/..` would be the project folder itself
    fs::create_dir_all(dir.join("p/slam-001")).unwrap();
    p.scans[0].id = "..".into();
    assert!(p.runs("..").is_empty());
    let err = p
        .new_run_dir("..", &SlamSettings::default())
        .unwrap_err()
        .to_string();
    assert!(err.contains("Scan-Kennung"), "{err}");
    assert!(!dir.join("p/slam-002").exists());
    assert!(!dir.join("slam-001").exists());
    let err = p.save().unwrap_err().to_string();
    assert!(err.contains("Scan-Kennung"), "{err}");
}

#[test]
fn windows_absolute_path_is_not_joined_to_the_project() {
    let dir = tmp("app-windows-abs");
    let pdir = dir.join("p");
    for path in ["C:\\Daten\\Scan", "\\\\server\\freigabe\\scan"] {
        hand_written(&pdir, &[("scan", path)]);
        let p = Project::open(&pdir).unwrap();
        assert_eq!(p.scans[0].path, PathBuf::from(path));
        assert!(!p.scans[0].path.starts_with(&p.dir), "{path}");
        assert!(matches!(p.scan_state("scan"), ScanState::Missing(_)));
    }
}

#[test]
fn scan_ids_are_unique_ignoring_case() {
    let dir = tmp("app-ids-case");
    let a = scan(&dir.join("x"), "Buero", 0.5);
    let b = scan(&dir.join("y"), "buero", 0.5);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    assert_eq!(p.add_scan(&a).unwrap(), "Buero");
    assert_eq!(p.add_scan(&b).unwrap(), "buero-2");

    hand_written(&dir.join("q"), &[("Scan", "a"), ("scan", "b")]);
    let err = Project::open(&dir.join("q")).unwrap_err().to_string();
    assert!(err.contains("„scan“") && err.contains("mehrfach"), "{err}");
}

#[test]
fn reuse_of_cli_run_with_other_settings_explains_why() {
    let dir = tmp("app-cli-other-settings");
    let raw = scan(&dir, "scan", 1.5);
    let out = dir.join("cli");
    let req = SlamRequest {
        scan: raw.clone(),
        out: out.clone(),
        settings: SlamSettings {
            lidar_topic: Some("/lidar".into()),
            ..SlamSettings::default()
        },
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let before = files(&out);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id = p.add_scan(&raw).unwrap();
    p.add_run(&id, &out).unwrap();

    let run = p.runs(&id).remove(0);
    let err = run_slam_job(&p, &id, &SlamJob::Reuse(run), &Progress::new())
        .unwrap_err()
        .to_string();
    assert!(
        err.starts_with("Nicht neu rechenbar: der Lauf wurde mit anderen SLAM-Einstellungen"),
        "{err}"
    );
    // what to do in the window, no command line
    assert!(err.contains("„SLAM rechnen“"), "{err}");
    assert!(!err.contains("driftless "), "{err}");
    assert_eq!(files(&out), before);
    assert!(!out.join("verlauf").exists());
}

/// A map of an added CLI run computed with other than the default settings is refused like
/// a reuse, instead of being made with the default odometry settings.
#[test]
fn map_of_cli_run_with_other_settings_is_refused() {
    let dir = tmp("app-cli-other-settings-map");
    let raw = scan(&dir, "scan", 1.5);
    let out = dir.join("cli");
    let req = SlamRequest {
        scan: raw.clone(),
        out: out.clone(),
        settings: SlamSettings {
            lidar_to_imu: Some([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            ..SlamSettings::default()
        },
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let before = files(&out);
    let entries = names(&out);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id = p.add_scan(&raw).unwrap();
    p.add_run(&id, &out).unwrap();

    let run = p.runs(&id).remove(0);
    let err = run_map_job(&run, &raw, &settings(MapStep::All), &Progress::new())
        .unwrap_err()
        .to_string();
    assert!(
        err.starts_with("Karte nicht rechenbar: der Lauf wurde mit anderen SLAM-Einstellungen"),
        "{err}"
    );
    assert!(!err.contains("driftless "), "{err}");
    assert_eq!(files(&out), before);
    assert_eq!(names(&out), entries, "no map folder");

    // with the default settings the map is made
    let plain = dir.join("cli-plain");
    cli_slam(&raw, &plain, false);
    p.add_run(&id, &plain).unwrap();
    let run = p
        .runs(&id)
        .into_iter()
        .find(|r| r.dir == canon(&plain))
        .unwrap();
    run_map_job(&run, &raw, &settings(MapStep::All), &Progress::new()).unwrap();
}

/// A correction that does not fit: the app's message says how to switch it off in the
/// program, not with CLI commands.
#[test]
fn bad_correction_hint_points_to_the_panel() {
    let dir = tmp("app-bad-correction");
    let (p, id) = project_with_run(&dir, 1.5);
    let run = p.runs(&id).remove(0);
    fs::write(
        run.dir.join("corrections.toml"),
        "[[loop]]\na = 0.5\nb = 30.0\nenabled = true\n",
    )
    .unwrap();
    let err = run_slam_job(&p, &id, &SlamJob::Reuse(run), &Progress::new()).unwrap_err();
    assert!(matches!(err, StepError::BadCorrection(_)), "{err:?}");
    let shown = driftless_app::ui::text::step_error(&err);
    assert!(shown.contains("außerhalb des Scans"), "{shown}");
    assert!(shown.contains("„Korrekturen“"), "{shown}");
    assert!(!shown.contains("driftless"), "{shown}");
}

#[test]
fn add_run_checks_the_scan() {
    let dir = tmp("app-run-scan");
    let a = scan(&dir, "a", 1.5);
    let b = dir.join("b");
    fs::create_dir_all(&b).unwrap();
    room_bag_at(&b, true, 5000.0, 1.5);
    let lauf = dir.join("lauf-a");
    cli_slam(&a, &lauf, false);
    let mut p = Project::create(&dir.join("p"), "P").unwrap();
    let id_a = p.add_scan(&a).unwrap();
    let id_b = p.add_scan(&b).unwrap();
    let err = p.add_run(&id_b, &lauf).unwrap_err().to_string();
    assert!(err.starts_with("Lauf gehört nicht zu diesem Scan"), "{err}");
    assert!(p.scans[1].extra_runs.is_empty());
    p.add_run(&id_a, &lauf).unwrap();
}

/// Two synthetic scans of one furnished room – B from (0.8, 0.5, 0) turned by 30° – each with
/// a run and a cleaned map in `<scan>/driftless` as the CLI makes them. Made once for the
/// merge tests, which only read them.
struct MergeScans {
    /// Their folder, shared by the tests of this file.
    _root: SharedDir,
    a: PathBuf,
    b: PathBuf,
}

fn merge_scans() -> &'static MergeScans {
    static SCANS: OnceLock<MergeScans> = OnceLock::new();
    SCANS.get_or_init(|| {
        let root = shared("app-merge-scans");
        let (a, b) = (root.join("a"), root.join("b"));
        for d in [&a, &b] {
            fs::create_dir_all(d).unwrap();
        }
        furnished_room_bag(&a, true, [0.0; 3], 0.0);
        furnished_room_bag(&b, true, [0.8, 0.5, 0.0], 30f64.to_radians());
        for d in [&a, &b] {
            let out = d.join("driftless");
            cli_slam(d, &out, false);
            let input = map_input_for(d, &out, &out, SlamSettings::default());
            run_map(
                &MapRequest {
                    input,
                    out_dir: out,
                    all_files: false,
                },
                &Progress::new(),
            )
            .unwrap();
        }
        MergeScans {
            a: canon(&a),
            b: canon(&b),
            _root: root,
        }
    })
}

/// A project in `dir/projekt` with both scans of [`merge_scans`], their runs added.
fn merge_project(dir: &Path) -> Project {
    let s = merge_scans();
    let mut p = Project::create(&dir.join("projekt"), "Modell").unwrap();
    for scan in [&s.a, &s.b] {
        let id = p.add_scan(scan).unwrap();
        p.add_run(&id, &scan.join("driftless")).unwrap();
    }
    p.save().unwrap();
    p
}

/// Every scan of `p` with the map in the folder of its run.
fn picks(p: &Project) -> Vec<MergePick> {
    p.scans
        .iter()
        .map(|s| {
            let run = p.runs(&s.id).remove(0);
            assert_eq!(run.maps.len(), 1, "{}", run.label);
            MergePick {
                scan_id: s.id.clone(),
                map: run.maps[0].dir.clone(),
                run: run.dir,
            }
        })
        .collect()
}

fn models_dir(p: &Project) -> PathBuf {
    p.dir.join("modelle")
}

/// „Ordner neu zuordnen …“ reaches the models (M2 of the final review): `modell.json` keeps
/// the scan folder of the merge, the project the one it has now. A model of a scan assigned a
/// new folder takes it from the project: the report its surface request is made of names the new
/// folder (the photos come from there), the scan is found in the project, and whether its map
/// changed is told from there.
#[test]
fn relinked_scan_reaches_the_model() {
    let dir = tmp("app-merge-relink");
    let (a, b) = (dir.join("a"), dir.join("b"));
    for d in [&a, &b] {
        fs::create_dir_all(d).unwrap();
    }
    furnished_room_bag(&a, true, [0.0; 3], 0.0);
    furnished_room_bag(&b, true, [0.8, 0.5, 0.0], 30f64.to_radians());
    let mut p = Project::create(&dir.join("projekt"), "Modell").unwrap();
    let ids: Vec<String> = [&a, &b].iter().map(|s| p.add_scan(s).unwrap()).collect();
    p.save().unwrap();
    let mut picks = Vec::new();
    for id in &ids {
        let full = SlamJob::Full(SlamSettings::default());
        let done = run_slam_job(&p, id, &full, &Progress::new()).unwrap();
        let run = p.runs(id).remove(0);
        let scan_dir = p.scan(id).unwrap().path.clone();
        let map = run_map_job(&run, &scan_dir, &settings(MapStep::All), &Progress::new()).unwrap();
        picks.push(MergePick {
            scan_id: id.clone(),
            run: done.run_dir,
            map: map.dir,
        });
    }
    run_merge_job(&p, &picks, false, &Progress::new()).unwrap();

    // the raw data of `a` moves to another drive; the scan is assigned its new folder
    let moved = dir.join("anderes-laufwerk").join("a");
    fs::create_dir_all(moved.parent().unwrap()).unwrap();
    fs::rename(&a, &moved).unwrap();
    assert!(p.relink_scan(&ids[0], &moved).unwrap());
    p.save().unwrap();
    let p = Project::open(&p.dir).unwrap();
    let model = p.models().remove(0);
    assert_eq!(model.report.members[0].scan, canon(&moved));
    assert_eq!(model.report.members[1].scan, canon(&b));
    assert_eq!(model_picks(&p, &model.report).unwrap().len(), 2);
    assert!(!model_status(&model).stale);

    // the scan's bags change in the new folder: the model is stale (from the old folder,
    // which is gone, it could not be told at all)
    let bag = moved.join("LIDAR_0.bag");
    let mut bytes = fs::read(&bag).unwrap();
    bytes.extend_from_slice(&[0; 16]);
    fs::write(&bag, bytes).unwrap();
    let model = p.models().remove(0);
    assert!(model_status(&model).stale);
}

#[test]
fn merge_job_creates_model_dir() {
    let dir = tmp("app-merge-job");
    let p = merge_project(&dir);
    let picks = picks(&p);
    let first = run_merge_job(&p, &picks, false, &Progress::new()).unwrap();
    assert_eq!(first, models_dir(&p).join("modell-001"));
    assert!(first.join(MODEL_REPORT).is_file());
    assert!(first.join(MODEL_LAS).is_file());
    let before = files(&first);
    // nothing is overwritten: a second merge is a new folder
    let second = run_merge_job(&p, &picks, false, &Progress::new()).unwrap();
    assert_eq!(second, models_dir(&p).join("modell-002"));
    assert_eq!(files(&first), before, "modell-001 untouched");
    // one scan only: refused, no folder
    let err = run_merge_job(&p, &picks[..1], false, &Progress::new()).unwrap_err();
    assert!(
        err.to_string().to_lowercase().contains("mindestens zwei"),
        "{err}"
    );
    assert_eq!(names(&models_dir(&p)), ["modell-001", "modell-002"]);
    // a scan not in the project
    let mut wrong = picks.clone();
    wrong[1].scan_id = "gibt-es-nicht".into();
    assert!(run_merge_job(&p, &wrong, false, &Progress::new()).is_err());
    assert_eq!(names(&models_dir(&p)), ["modell-001", "modell-002"]);
}

#[test]
fn cancelled_merge_job_leaves_no_dir() {
    let dir = tmp("app-merge-cancel");
    let p = merge_project(&dir);
    let picks = picks(&p);
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Ausrichten" {
            handle.cancel();
        }
    });
    let err = run_merge_job(&p, &picks, false, &progress).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    let left: Vec<String> = fs::read_dir(models_dir(&p))
        .map(|e| {
            e.map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect()
        })
        .unwrap_or_default();
    assert!(left.is_empty(), "{left:?}");
    // a failing merge too: the same run twice
    let twice = vec![picks[0].clone(), picks[0].clone()];
    assert!(run_merge_job(&p, &twice, false, &Progress::new()).is_err());
    assert!(p.models().is_empty());
    assert!(!models_dir(&p).join("modell-001").exists());
}

#[test]
fn models_listed_with_verdicts() {
    let dir = tmp("app-merge-list");
    let p = merge_project(&dir);
    assert!(p.models().is_empty());
    let made = run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    // a folder without a report (a merge still running, or stopped) and others are no model
    fs::create_dir_all(models_dir(&p).join("modell-002")).unwrap();
    fs::create_dir_all(models_dir(&p).join("anderes")).unwrap();
    let models = p.models();
    assert_eq!(models.len(), 1);
    let m = &models[0];
    assert_eq!(m.dir, made);
    assert_eq!(m.label, "Modell 001");
    assert!(m.surfaces.is_empty());
    let verdicts: Vec<&str> = m
        .report
        .members
        .iter()
        .map(|x| x.verdict.as_str())
        .collect();
    assert_eq!(verdicts, ["sicher", "sicher"]);
    assert_eq!(m.report.members[0].method, "bezug");
    assert!(m.report.members.iter().all(|x| x.included));
    assert_eq!(m.report.members[1].scan, merge_scans().b);
    assert!(!model_status(m).stale);
}

/// A model merged by an older version of the merge (`version` = [`MERGE_VERSION`] − 1) is
/// stale, also while a scan of it cannot be read (its maps cannot be told then, its version
/// can), and so are its surfaces.
#[test]
fn model_of_an_older_merge_is_stale() {
    let dir = tmp("app-merge-version");
    let p = merge_project(&dir);
    let made = run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    let current = p.models().remove(0);
    assert!(!current.other_version());
    assert!(!model_status(&current).stale);
    let file = made.join(MODEL_REPORT);
    let mut json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&file).unwrap()).unwrap();
    json["version"] = (MERGE_VERSION - 1).into();
    fs::write(&file, json.to_string()).unwrap();
    let old = p.models().remove(0);
    assert_eq!(old.report.version, MERGE_VERSION - 1);
    assert!(old.other_version());
    assert!(model_status(&old).stale);
    // a scan of it cannot be read: a changed map could not be told, the version still is
    let mut unplugged = old.clone();
    unplugged.report.members[1].scan = dir.join("abgesteckt");
    assert!(model_status(&unplugged).stale);
    let mut current_unplugged = current.clone();
    current_unplugged.report.members[1].scan = dir.join("abgesteckt");
    assert!(!model_status(&current_unplugged).stale);
    // a surface with a readable report: stale with the model, not without it
    let surface_dir = made.join("oberflaeche-001");
    fs::create_dir_all(&surface_dir).unwrap();
    fs::write(surface_dir.join("oberflaeche.json"), "{}").unwrap();
    let surface = driftless_app::project::SurfaceInfo {
        dir: surface_dir,
        label: "Oberfläche 001".into(),
        stale: false,
        settings: None,
        hints: Vec::new(),
    };
    assert!(model_surface_status(&unplugged, &surface).stale);
    assert!(!model_surface_status(&current_unplugged, &surface).stale);
}

/// The hints for the user in a surface report (`hinweise`) are read with the surface; a report
/// without them (written before the field) reads as no hints.
#[test]
fn surface_hints_are_read() {
    let dir = tmp("app-surface-hints");
    let p = merge_project(&dir);
    let model = run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    let hint = "Stellenweise offen: An 23 % der dünn gemessenen Stellen lag kein Messpunkt.";
    for (name, report) in [
        ("oberflaeche-001", serde_json::json!({ "hinweise": [hint] })),
        (
            "oberflaeche-002",
            serde_json::json!({ "seconds_total": 1.0 }),
        ),
        ("oberflaeche-003", serde_json::json!({ "hinweise": [] })),
    ] {
        let surface = model.join(name);
        fs::create_dir_all(&surface).unwrap();
        fs::write(surface.join("oberflaeche.glb"), b"glb").unwrap();
        fs::write(surface.join("oberflaeche.json"), report.to_string()).unwrap();
    }
    let surfaces = &p.models()[0].surfaces;
    let hints: Vec<&[String]> = surfaces.iter().map(|s| s.hints.as_slice()).collect();
    assert_eq!(hints, [&[hint.to_string()][..], &[], &[]]);
}

#[test]
fn model_surface_job_reuses_a_fitting_surface() {
    let dir = tmp("app-merge-surface");
    let p = merge_project(&dir);
    run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    let model = p.models().remove(0);
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
    let done = run_model_surface_job(&model, &settings, GpuPref::Off, &Progress::new()).unwrap();
    assert!(!done.reused);
    assert_eq!(done.dir, model.dir.join("oberflaeche-001"));
    let model = p.models().remove(0);
    assert_eq!(model.surfaces.len(), 1);
    assert_eq!(model.surfaces[0].label, "Oberfläche 001");
    assert_eq!(model.surfaces[0].settings.as_ref(), Some(&settings));
    let status = model_surface_status(&model, &model.surfaces[0]);
    assert!(!status.stale && status.seconds.is_some(), "{status:?}");
    // the same again: kept, nothing computed
    let again = run_model_surface_job(&model, &settings, GpuPref::Off, &Progress::new()).unwrap();
    assert!(again.reused);
    assert_eq!(again.dir, done.dir);
    // with other settings, cancelled: no new folder
    let other = SurfaceSettings {
        config: SurfaceConfig {
            voxel: 0.04,
            ..settings.config.clone()
        },
        texture: false,
    };
    let progress = Progress::new();
    progress.cancel();
    let err = run_model_surface_job(&model, &other, GpuPref::Off, &progress).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(!model.dir.join("oberflaeche-002").exists());
    // a surface without its report is stale
    let mut broken = model.surfaces[0].clone();
    broken.dir = model.dir.join("oberflaeche-009");
    assert!(model_surface_status(&model, &broken).stale);
}

/// „Übernehmen und neu zusammenführen“: the pairs are added to `modelle/punktpaare.toml` and
/// the same scans are merged again into the next model, the scan placed by the pairs. A
/// switched-off entry no longer counts; an entry that cannot be used adds nothing.
#[test]
fn pairs_rerun_creates_next_model() {
    use driftless_traj::nalgebra::Point3;
    let dir = tmp("app-merge-pairs");
    let p = merge_project(&dir);
    let first = run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    let model = p.models().remove(0);
    let members = &model.report.members;
    assert_eq!(model_picks(&p, &model.report).unwrap(), picks(&p));
    let before = files(&first);
    // three places of the room in the model frame (= frame of A), and where B sees them
    let in_model = [[0.5, 0.5, 0.3], [2.5, 0.8, 1.2], [1.0, 2.5, 2.0]];
    let t_b = member_transform(&members[1]);
    let in_b = in_model.map(|q| {
        let r = t_b.inverse() * Point3::from(q);
        [r.x, r.y, r.z]
    });
    let entry = PointPairEntry {
        a: members[0].map_dir.clone(),
        b: members[1].map_dir.clone(),
        points_a: in_model.to_vec(),
        points_b: in_b.to_vec(),
        enabled: true,
        note: "Ecken".into(),
        added: String::new(),
        unknown: Default::default(),
    };
    let second = run_pairs_merge_job(&p, &model, entry.clone(), &Progress::new()).unwrap();
    assert_eq!(second, models_dir(&p).join("modell-002"));
    assert_eq!(files(&first), before, "modell-001 untouched");
    let file = models_dir(&p).join(POINT_PAIRS_FILE);
    let saved = PointPairs::load(&file).unwrap();
    assert_eq!(saved.entries.len(), 1);
    // the scans lie outside the project: absolute
    assert_eq!(saved.entries[0].a, members[0].map_dir);
    assert!(!saved.entries[0].added.is_empty());
    assert_eq!(point_pairs(&p).unwrap()[0].b, members[1].map_dir);
    let report = read_model(&second).unwrap();
    let dirs = |r: &driftless_steps::merge::ModelReport| -> Vec<PathBuf> {
        r.members.iter().map(|m| m.map_dir.clone()).collect()
    };
    assert_eq!(
        dirs(&report),
        dirs(&model.report),
        "same scans, same reference"
    );
    assert_eq!(report.members[1].method, "punktpaare");
    assert_eq!(report.members[1].verdict, "sicher");

    // switched off: the next merge aligns B by itself again
    set_point_pair_enabled(&p, 0, false).unwrap();
    assert!(!PointPairs::load(&file).unwrap().entries[0].enabled);
    let third = run_merge_job(&p, &picks(&p), false, &Progress::new()).unwrap();
    assert_ne!(read_model(&third).unwrap().members[1].method, "punktpaare");
    assert!(set_point_pair_enabled(&p, 5, true).is_err());

    // points on a line fix no turn: refused, nothing saved, no new model
    let mut line = entry;
    line.points_a = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    line.points_b = line.points_a.clone();
    let err = run_pairs_merge_job(&p, &model, line, &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("Linie"), "{err}");
    assert_eq!(PointPairs::load(&file).unwrap().entries.len(), 1);
    assert_eq!(p.models().len(), 3);
    assert!(!report.include_unsure, "merged without unsure ones");

    // „Auch unsichere übernehmen“ is stored in the model, and the rerun with point pairs keeps
    // it although no member is unsure
    let fourth = run_merge_job(&p, &picks(&p), true, &Progress::new()).unwrap();
    let unsure = read_model(&fourth).unwrap();
    assert!(unsure.include_unsure);
    assert!(unsure.members.iter().all(|m| m.verdict != "unsicher"));
    let model = p
        .models()
        .into_iter()
        .find(|m| m.dir == fourth)
        .expect("the fourth model");
    let fifth = run_pairs_merge_job(&p, &model, entry_again(&model), &Progress::new()).unwrap();
    assert!(read_model(&fifth).unwrap().include_unsure);
}

/// A point pair entry between the first two members of `model` (the same room corners as
/// above, by its transforms).
fn entry_again(model: &driftless_app::project::ModelInfo) -> PointPairEntry {
    use driftless_traj::nalgebra::Point3;
    let members = &model.report.members;
    let in_model = [[0.5, 0.5, 0.3], [2.5, 0.8, 1.2], [1.0, 2.5, 2.0]];
    let t_b = member_transform(&members[1]);
    PointPairEntry {
        a: members[0].map_dir.clone(),
        b: members[1].map_dir.clone(),
        points_a: in_model.to_vec(),
        points_b: in_model
            .map(|q| {
                let r = t_b.inverse() * Point3::from(q);
                [r.x, r.y, r.z]
            })
            .to_vec(),
        enabled: true,
        note: "Ecken".into(),
        added: String::new(),
        unknown: Default::default(),
    }
}
