//! `run_map` and `folder_fits` on synthetic scans (no camera images: colouring is skipped).

use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag, room_bag_secs, tmp};
use driftless_steps::map::{
    clean_stage_id, current_map, dense_stage_id, fitting_stages, folder_fits, recompute_map,
    run_map, MapInput, MapRequest, MapSettings, MapStep, CLEAN_BIN, DENSE_BIN,
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
        all_files: false,
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
    // No images: the cleaned cloud is the one to look at; no `dense.las`.
    let names: Vec<String> = files(&map_dir).into_keys().collect();
    assert_eq!(
        names,
        [
            "clean.bin",
            "clean.las",
            "dense.bin",
            "map.json",
            "removed.las"
        ]
    );
    assert_eq!(
        report["steps"]["dense"]["files"],
        serde_json::json!(["dense.bin"])
    );
    assert_eq!(
        report["steps"]["clean"]["files"],
        serde_json::json!(["clean.las", "removed.las", "clean.bin"])
    );
    let json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(map_dir.join("map.json")).unwrap()).unwrap();
    assert_eq!(json, report);
    assert_eq!(
        report["programm"],
        serde_json::json!({
            "version": env!("CARGO_PKG_VERSION"),
            "commit": driftless_steps::PROGRAM.commit,
        })
    );
    assert_eq!(report["steps"]["dense"]["reused"], false);
    assert!(
        report["steps"]["clean"]["stats"]["dense_quality"]["patches"].is_number(),
        "{report}"
    );
    assert!(report["steps"]["color"]["skipped"].is_string(), "{report}");
    // no images: a warning, as on the console; the reason first, then what to do
    assert!(p.take_log().iter().any(|l| matches!(
        l,
        driftless_io::progress::LogLine::Warning(w) if w.starts_with("keine Kamerabilder: ")
            && w.contains(" – Karte nicht eingefärbt (clean.las statt color.las); ")
            && w.ends_with("die Karte neu rechnen")
    )));
}

/// With `all_files` every step writes its cloud as LAS, as before the first release.
#[test]
fn all_files_writes_every_cloud() {
    let dir = tmp("steps-map-all-files");
    room_bag(&dir, true);
    let slam_dir = dir.join("slam");
    slam(&dir, &slam_dir, false);
    let map_dir = dir.join("karte");
    let req = MapRequest {
        all_files: true,
        ..request(&dir, &slam_dir, &map_dir, MapStep::All)
    };
    let report = run_map(&req, &Progress::new()).unwrap();
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
    assert_eq!(
        report["steps"]["dense"]["files"],
        serde_json::json!(["dense.las", "dense.bin"])
    );
    // The same stages as without: the switch is part of no id.
    let plain = dir.join("karte-ohne");
    let again = run_map(
        &request(&dir, &slam_dir, &plain, MapStep::All),
        &Progress::new(),
    )
    .unwrap();
    for step in ["dense", "clean"] {
        assert_eq!(
            again["steps"][step]["input_id"],
            report["steps"][step]["input_id"]
        );
    }
    for f in ["dense.bin", "clean.bin", "clean.las", "removed.las"] {
        assert_eq!(
            fs::read(map_dir.join(f)).unwrap(),
            fs::read(plain.join(f)).unwrap()
        );
    }
}

/// A cloud an earlier run wrote is listed as current only while it fits: the same input keeps
/// it, another trajectory makes it stale (warned, not deleted), also when its step ran again
/// without writing it.
#[test]
fn clouds_of_an_earlier_run_fit_only_the_same_input() {
    let dir = tmp("steps-map-earlier-clouds");
    room_bag(&dir, true);
    let slam_dir = dir.join("slam");
    slam(&dir, &slam_dir, false);
    let all = MapRequest {
        all_files: true,
        ..request(&dir, &slam_dir, &slam_dir, MapStep::All)
    };
    run_map(&all, &Progress::new()).unwrap();
    // The same input without the switch: `dense.las` still fits.
    let plain = request(&dir, &slam_dir, &slam_dir, MapStep::All);
    let report = run_map(&plain, &Progress::new()).unwrap();
    assert_eq!(report["stale"], serde_json::json!([]));
    assert_eq!(
        report["steps"]["dense"]["files"],
        serde_json::json!(["dense.las", "dense.bin"])
    );
    // Another trajectory: the dense step runs anew without writing `dense.las`.
    let traj = fs::read_to_string(slam_dir.join("traj.tum")).unwrap();
    fs::write(slam_dir.join("traj.tum"), format!("{traj}# geändert\n")).unwrap();
    let p = Progress::new();
    let report = run_map(&plain, &p).unwrap();
    assert_eq!(report["stale"], serde_json::json!(["dense.las"]));
    assert_eq!(
        report["steps"]["dense"]["files"],
        serde_json::json!(["dense.bin"])
    );
    assert!(p.take_log().iter().any(|l| matches!(
        l,
        driftless_io::progress::LogLine::Warning(w)
            if w.starts_with("dense.las stammt aus einem früheren Lauf")
    )));
    assert!(slam_dir.join("dense.las").is_file(), "not deleted");
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

/// A killed run leaves `<name>.tmp` files; the next map removes those of its own files at
/// its start, and only those: in the SLAM folder (where the CLI writes the map) a `.tmp` of
/// the SLAM run or of a surface stays.
#[test]
fn stale_tmp_files_of_the_map_are_removed() {
    let dir = tmp("steps-map-tmp");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let own = [
        "dense.las.tmp",
        "dense.bin.tmp",
        "clean.las.tmp",
        "removed.las.tmp",
        "clean.bin.tmp",
        "color.las.tmp",
        "color.ply.tmp",
        "map.json.tmp",
    ];
    let foreign = ["traj.tum.tmp", "map.las.tmp", "slam.json.tmp", "notiz.tmp"];
    for f in own.iter().chain(&foreign) {
        fs::write(out.join(f), b"halb").unwrap();
    }
    run_map(&request(&dir, &out, &out, MapStep::All), &Progress::new()).unwrap();
    for f in own {
        assert!(!out.join(f).exists(), "{f} left");
    }
    for f in foreign {
        assert!(out.join(f).exists(), "{f} removed");
    }
}

/// `map.json` goes before the first file of a map is written and comes back last: a map
/// that stops in between (here: `removed.las` cannot be replaced, after `dense.bin` was
/// written) leaves no report and does not fit, also not the input of the old report.
#[test]
fn report_goes_before_the_first_write() {
    let dir = tmp("steps-map-report-first");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let all = input(&dir, &out, MapStep::All);
    run_map(&request(&dir, &out, &out, MapStep::All), &Progress::new()).unwrap();
    assert!(folder_fits(&all, &out));
    fs::remove_file(out.join("removed.las")).unwrap();
    fs::create_dir_all(out.join("removed.las").join("x")).unwrap();
    let mut coarse = request(&dir, &out, &out, MapStep::All);
    coarse.input.map.switches.voxel *= 2.0;
    let err = run_map(&coarse, &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("removed.las"), "{err}");
    assert!(!out.join("map.json").exists(), "the old report stayed");
    assert!(!folder_fits(&all, &out));
    assert!(!folder_fits(&coarse.input, &out));
    assert!(!files(&out).keys().any(|f| f.ends_with(".tmp")));
}

/// A stage of another run next to the report of an earlier one (what a run of an older
/// version left when it was killed between its files) does not fit; a stage that is missing
/// is no reason (`dense.bin` may be deleted to save space).
#[test]
fn stage_of_another_run_next_to_an_old_report_does_not_fit() {
    let dir = tmp("steps-map-mixed");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let all = input(&dir, &out, MapStep::All);
    run_map(&request(&dir, &out, &out, MapStep::All), &Progress::new()).unwrap();
    let mut coarse = request(&dir, &out, &dir.join("grob"), MapStep::All);
    coarse.input.map.switches.voxel *= 2.0;
    run_map(&coarse, &Progress::new()).unwrap();
    let keep = fs::read(out.join(DENSE_BIN)).unwrap();
    fs::copy(dir.join("grob").join(DENSE_BIN), out.join(DENSE_BIN)).unwrap();
    assert!(!folder_fits(&all, &out), "dense.bin of another run");
    fs::write(out.join(DENSE_BIN), &keep).unwrap();
    assert!(folder_fits(&all, &out));
    fs::copy(dir.join("grob").join(CLEAN_BIN), out.join(CLEAN_BIN)).unwrap();
    assert!(!folder_fits(&all, &out), "clean.bin of another run");
    fs::remove_file(out.join(CLEAN_BIN)).unwrap();
    fs::remove_file(out.join(DENSE_BIN)).unwrap();
    assert!(folder_fits(&all, &out), "missing stages are no reason");
}

/// `programm` in `map.json` only says which build made the map: a map of another build, or
/// one written before the field, still fits.
#[test]
fn program_is_part_of_no_id() {
    let dir = tmp("steps-map-programm");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let all = input(&dir, &out, MapStep::All);
    run_map(&request(&dir, &out, &out, MapStep::All), &Progress::new()).unwrap();
    assert!(folder_fits(&all, &out));
    let path = out.join("map.json");
    let mut json: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
    json["programm"] = serde_json::json!({"version": "0.0.1", "commit": "anderer"});
    fs::write(&path, json.to_string()).unwrap();
    assert!(folder_fits(&all, &out), "another build");
    json.as_object_mut().unwrap().remove("programm");
    fs::write(&path, json.to_string()).unwrap();
    assert!(folder_fits(&all, &out), "written before the field");
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

/// A whole map that is current already is not computed again: nothing is written and the
/// step says so. Any change computes it: other switches, another trajectory, a file of the
/// report missing, more files wanted, another step; [`recompute_map`] computes it anyway.
#[test]
fn repeated_map_is_current() {
    use driftless_io::progress::LogLine;
    let dir = tmp("steps-map-aktuell");
    room_bag(&dir, true);
    let out = dir.join("out");
    slam(&dir, &out, false);
    let req = request(&dir, &out, &out, MapStep::All);
    assert!(current_map(&req).is_none(), "no map yet");
    let first = run_map(&req, &Progress::new()).unwrap();
    let before = files(&out);
    let p = Progress::new();
    assert_eq!(run_map(&req, &p).unwrap(), first);
    assert_eq!(files(&out), before, "nothing written");
    let log = p.take_log();
    assert!(
        matches!(&log[..], [LogLine::Info(i)] if i.starts_with("Karte ist aktuell: ")),
        "{log:?}"
    );
    // Computed again when not current.
    let computes = |req: &MapRequest| {
        let p = Progress::new();
        run_map(req, &p).unwrap();
        p.take_log()
            .iter()
            .any(|l| matches!(l, LogLine::Info(i) if i.starts_with("Dicht: ")))
    };
    let mut other = req.clone();
    other.input.map.switches.outliers = false;
    assert!(current_map(&other).is_none(), "other switches");
    let mut gpu_off = req.clone();
    gpu_off.input.map.switches.gpu = driftless_gpu::GpuPref::Off;
    assert!(
        current_map(&gpu_off).is_some(),
        "the GPU gives an equivalent map"
    );
    let all_files = MapRequest {
        all_files: true,
        ..req.clone()
    };
    assert!(current_map(&all_files).is_none(), "more files wanted");
    assert!(current_map(&request(&dir, &out, &out, MapStep::Clean)).is_none());
    // No images: `clean.las` is the cloud to look at.
    fs::remove_file(out.join("clean.las")).unwrap();
    assert!(current_map(&req).is_none(), "a file of the report missing");
    assert!(computes(&req));
    assert!(current_map(&req).is_some());
    let traj = fs::read_to_string(out.join("traj.tum")).unwrap();
    fs::write(out.join("traj.tum"), format!("{traj}# geändert\n")).unwrap();
    assert!(current_map(&req).is_none(), "another trajectory");
    assert!(computes(&req));
    // `recompute_map` computes it although it is current.
    let p = Progress::new();
    recompute_map(&req, &p).unwrap();
    assert!(p
        .take_log()
        .iter()
        .any(|l| matches!(l, LogLine::Info(i) if i.starts_with("Dicht: "))));
}
