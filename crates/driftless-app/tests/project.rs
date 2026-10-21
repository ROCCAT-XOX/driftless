//! The project model on synthetic scans: `project.toml`, run folders, map folders, CLI
//! folders added as runs, missing scan folders.

use driftless_app::project::{map_dir_for, run_map_job, run_slam_job, Project, ScanState, SlamJob};
use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag_at, room_bag_secs, tmp};
use driftless_steps::correct;
use driftless_steps::history::HistoryKind;
use driftless_steps::map::{run_map, MapInput, MapRequest, MapSettings, MapStep, MapSwitches};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::StepError;
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

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
        err.to_string().starts_with("Projekt existiert schon"),
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
    // no camera images: no color.las
    let clouds: Vec<String> = map
        .clouds
        .iter()
        .map(|c| c.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(clouds, ["clean.las", "dense.las", "removed.las"]);
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
            assert!(why.starts_with("Scan-Ordner "), "{why}");
            assert!(why.ends_with(" nicht gefunden"), "{why}");
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
    let dense = fs::read(done.dir.join("dense.bin")).unwrap();

    // same switches, another step: the same folder
    let run = p.runs(&id).remove(0);
    let done = run_map_job(&run, &scan_dir, &settings(MapStep::Clean), &Progress::new()).unwrap();
    assert_eq!(done.dir, run.dir.join("karte-001"));
    assert_eq!(done.report["steps"]["dense"]["reused"], true);
    assert_eq!(fs::read(done.dir.join("dense.bin")).unwrap(), dense);
    let run = p.runs(&id).remove(0);
    assert_eq!(run.maps.len(), 1);
    assert_eq!(run.maps[0].label, "Karte 001");
    assert_eq!(run.maps[0].settings, Some(MapSwitches::default()));

    // map_dir_for on its own: the fitting folder, nothing created
    let input = MapInput {
        scan: scan_dir.clone(),
        slam_dir: run.dir.clone(),
        slam: run.settings.clone(),
        map: settings(MapStep::Color),
    };
    let target = map_dir_for(&run, &input, &Progress::new()).unwrap();
    assert!(!target.is_new());
    assert_eq!(target.path(), run.dir.join("karte-001"));
    drop(target);
    assert!(run.dir.join("karte-001/map.json").exists(), "kept on drop");
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
    assert!(
        names(target.path()).is_empty(),
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
        err.starts_with("Lauf wurde mit anderen Einstellungen gerechnet"),
        "{err}"
    );
    assert!(err.contains("--reuse"), "{err}");
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
        err.starts_with("Lauf wurde mit anderen Einstellungen gerechnet"),
        "{err}"
    );
    assert!(err.contains("Karte"), "{err}");
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
    assert!(err.contains("gehört nicht zum Scan"), "{err}");
    assert!(p.scans[1].extra_runs.is_empty());
    p.add_run(&id_a, &lauf).unwrap();
}
