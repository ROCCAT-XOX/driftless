//! `run_map` and `folder_fits` on synthetic scans (no camera images: colouring is skipped).

use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag, room_bag_secs, tmp};
use driftless_steps::map::{
    clean_stage_id, dense_stage_id, fitting_stages, folder_fits, run_map, MapInput, MapRequest,
    MapSettings, MapStep, CLEAN_BIN, DENSE_BIN,
};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::{correct, StepError};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

fn slam(scan: &Path, out: &Path, reuse: bool) {
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
}

fn input(scan: &Path, slam_dir: &Path, step: MapStep) -> MapInput {
    MapInput {
        scan: scan.to_path_buf(),
        slam_dir: slam_dir.to_path_buf(),
        slam: SlamSettings::default(),
        map: MapSettings {
            step,
            ..MapSettings::default()
        },
    }
}

fn request(scan: &Path, slam_dir: &Path, out_dir: &Path, step: MapStep) -> MapRequest {
    MapRequest {
        input: input(scan, slam_dir, step),
        out_dir: out_dir.to_path_buf(),
    }
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

#[test]
fn map_cancelled_writes_nothing() {
    let dir = tmp("steps-map-cancel");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    run_map(&request(&dir, &out, &out, MapStep::Dense), &Progress::new()).unwrap();
    let before = files(&out);
    assert!(before.contains_key("dense.las") && before.contains_key("dense.bin"));

    // the clean step takes dense.bin and is cancelled in its first part
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Geister" {
            handle.cancel();
        }
    });
    let err = run_map(&request(&dir, &out, &out, MapStep::Clean), &p).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    for f in ["clean.las", "removed.las", "clean.bin"] {
        assert!(!out.join(f).exists(), "{f} written");
    }
    assert_eq!(files(&out), before, "dense.*, map.json unchanged");

    // cancelled while the frames are read: the dense step writes nothing either
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "LiDAR lesen" && s.done >= 3 {
            handle.cancel();
        }
    });
    let err = run_map(&request(&dir, &out, &out, MapStep::All), &p).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(files(&out), before);
}

#[test]
fn map_out_dir_separate_from_slam_dir() {
    let dir = tmp("steps-map-out-dir");
    room_bag(&dir, true);
    let slam_dir = dir.join("slam");
    slam(&dir, &slam_dir, false);
    let slam_files = files(&slam_dir);
    let map_dir = dir.join("karte");

    let p = Progress::new();
    let report = run_map(&request(&dir, &slam_dir, &map_dir, MapStep::All), &p).unwrap();
    assert_eq!(files(&slam_dir), slam_files, "SLAM folder untouched");
    let names: Vec<String> = files(&map_dir).into_keys().collect();
    assert_eq!(
        names,
        [
            "clean.bin",
            "clean.las",
            "dense.bin",
            "dense.las",
            "map.json",
            "removed.las"
        ]
    );
    let json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(map_dir.join("map.json")).unwrap()).unwrap();
    assert_eq!(json, report);
    assert_eq!(report["steps"]["dense"]["reused"], false);
    assert!(
        report["steps"]["clean"]["stats"]["dense_quality"]["patches"].is_number(),
        "{report}"
    );
    assert!(report["steps"]["color"]["skipped"].is_string(), "{report}");
    // no images: a warning, as on the console
    assert!(p.take_log().iter().any(|l| matches!(
        l,
        driftless_io::progress::LogLine::Warning(w) if w.starts_with("nicht eingefärbt, ")
    )));
}

#[test]
fn folder_fits_detects_new_traj() {
    // two submaps, so a manual loop moves the trajectory
    let dir = tmp("steps-map-fits");
    room_bag_secs(&dir, true, 12.0);
    let slam_dir = dir.join("slam");
    slam(&dir, &slam_dir, false);
    let map_dir = dir.join("karte");
    assert!(!folder_fits(
        &input(&dir, &slam_dir, MapStep::All),
        &map_dir
    ));
    run_map(
        &request(&dir, &slam_dir, &map_dir, MapStep::Dense),
        &Progress::new(),
    )
    .unwrap();

    // the step does not matter, the settings do
    for step in [MapStep::Dense, MapStep::Clean, MapStep::All] {
        assert!(folder_fits(&input(&dir, &slam_dir, step), &map_dir));
    }
    let mut other = input(&dir, &slam_dir, MapStep::All);
    other.map.switches.ghosts = false;
    assert!(!folder_fits(&other, &map_dir));
    let mut other = input(&dir, &slam_dir, MapStep::All);
    other.map.switches.voxel *= 2.0;
    assert!(!folder_fits(&other, &map_dir));

    let traj = fs::read(slam_dir.join("traj.tum")).unwrap();
    correct::add_loop(&slam_dir, 1.0, 11.0, "").unwrap();
    slam(&dir, &slam_dir, true);
    assert_ne!(fs::read(slam_dir.join("traj.tum")).unwrap(), traj);
    assert!(!folder_fits(
        &input(&dir, &slam_dir, MapStep::All),
        &map_dir
    ));
}

#[test]
fn fitting_stages_check_the_input() {
    let dir = tmp("steps-map-stages");
    room_bag(&dir, true);
    let slam_dir = dir.join("slam");
    slam(&dir, &slam_dir, false);
    let map_dir = dir.join("karte");
    let all = input(&dir, &slam_dir, MapStep::All);
    assert!(fitting_stages(&all, &map_dir).is_empty());
    run_map(
        &request(&dir, &slam_dir, &map_dir, MapStep::All),
        &Progress::new(),
    )
    .unwrap();
    assert_eq!(fitting_stages(&all, &map_dir), [DENSE_BIN, CLEAN_BIN]);

    // other cleaning: only the dense stage fits
    let mut other = input(&dir, &slam_dir, MapStep::All);
    other.map.switches.smooth = false;
    assert_eq!(fitting_stages(&other, &map_dir), [DENSE_BIN]);
    // other voxel size: neither
    let mut other = input(&dir, &slam_dir, MapStep::All);
    other.map.switches.voxel *= 2.0;
    assert!(fitting_stages(&other, &map_dir).is_empty());
    // colour switches do not matter for the stages
    let mut other = input(&dir, &slam_dir, MapStep::All);
    other.map.switches.downscale = 4;
    assert_eq!(fitting_stages(&other, &map_dir), [DENSE_BIN, CLEAN_BIN]);
}

/// No stage id holds the program version: a new release keeps every map that still fits.
/// Each holds the version of its step instead.
#[test]
fn stage_ids_hold_no_program_version() {
    let dir = tmp("steps-map-ids");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let input = input(&dir, &out, MapStep::All);
    let dense = dense_stage_id(&input).unwrap();
    let clean = clean_stage_id(&input).unwrap();
    for id in [&dense, &clean] {
        assert!(!id.contains(env!("CARGO_PKG_VERSION")), "{id}");
        assert!(!id.contains("driftless"), "{id}");
    }
    assert!(dense.starts_with("map dense v1\n"), "{dense}");
    assert!(clean.starts_with(&dense), "{clean}");
    assert!(clean.contains("clean v3 "), "{clean}");
}

#[test]
fn settings_check_names_the_option() {
    let mut s = MapSettings::default();
    assert_eq!(s.check(), Ok(()));
    s.switches.voxel = 0.0;
    assert_eq!(
        s.check().unwrap_err().to_string(),
        "--voxel: Voxelgröße muss größer als 0 sein"
    );
    let mut s = MapSettings::default();
    s.switches.downscale = 3;
    assert_eq!(
        s.check().unwrap_err().to_string(),
        "--downscale muss 1, 2 oder 4 sein"
    );
}
