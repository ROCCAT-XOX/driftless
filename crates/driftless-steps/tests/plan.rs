//! `run_plan` on synthetic rooms: a map folder or a model made by hand, without any raw data.

use driftless_export::testutil::write_map_folder;
use driftless_export::{write_points, PointFormat, Points};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_plan::testutil::{Scene, TestData};
use driftless_steps::merge::{
    read_model, ModelMember, ModelReport, MERGE_VERSION, MODEL_LAS, MODEL_REPORT,
};
use driftless_steps::plan::{
    plan_fits, read_plan, run_plan, PlanOutcome, PlanReport, PlanRequest, PlanSettings, PlanSource,
    PLAN_REPORT, PLAN_VERSION, TOP_VIEW_FILE,
};
use driftless_steps::slam::SlamSettings;
use driftless_steps::StepError;
use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
use driftless_traj::{write_tum, Pose};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

const FLOOR: f64 = -1.3;
const HEIGHT: f64 = 2.6;
/// Coarser than a real scan, fine enough for the search of the walls: the tests write and read
/// the cloud as LAS.
const SPACING: f64 = 0.015;

const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
/// A scan's world → the model: a quarter turn about Z, then a shift. Exact numbers, no angle.
const TURNED: [f64; 16] = [
    0.0, -1.0, 0.0, 30.0, 1.0, 0.0, 0.0, -7.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 0.0, 1.0,
];

/// A point of the model in the world of a scan moved by [`TURNED`].
fn in_turned_scan(p: [f64; 3]) -> [f64; 3] {
    [p[1] + 7.0, 30.0 - p[0], p[2] - 5.0]
}

const LOOP: [[f64; 3]; 4] = [
    [1.0, 1.0, 0.0],
    [4.0, 1.0, 0.0],
    [4.0, 3.0, 0.0],
    [1.0, 3.0, 0.0],
];

/// One room of 5 × 4 m and a walk around in it.
fn one_room() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        Scene::new()
            .spacing(SPACING)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, HEIGHT)
            .walk(&LOOP)
            .build()
    })
}

/// The same room twice, one storey above the other, with a walk on each.
fn two_storeys() -> &'static TestData {
    static DATA: OnceLock<TestData> = OnceLock::new();
    DATA.get_or_init(|| {
        let upper = LOOP.map(|p| [p[0], p[1], 2.9]);
        Scene::new()
            .spacing(SPACING)
            .room([0.0, 0.0], [5.0, 4.0], FLOOR, HEIGHT)
            .room([0.0, 0.0], [5.0, 4.0], 1.6, HEIGHT)
            .walk(&LOOP)
            .walk(&upper)
            .build()
    })
}

fn map_request(dir: &Path) -> PlanRequest {
    PlanRequest {
        source: PlanSource::Map(dir.to_path_buf()),
        model: None,
        out_dir: dir.join("grundriss"),
        settings: PlanSettings::default(),
    }
}

fn run(req: &PlanRequest) -> PlanReport {
    match run_plan(req, &Progress::new()).unwrap() {
        PlanOutcome::Done(report) => *report,
        PlanOutcome::SeveralStoreys(found) => panic!("mehrere Stockwerke: {found:?}"),
    }
}

fn write_traj(dir: &Path, poses: &[[f64; 3]]) {
    fs::create_dir_all(dir).unwrap();
    let poses: Vec<Pose> = poses
        .iter()
        .enumerate()
        .map(|(i, p)| Pose {
            t: i as f64 * 0.1,
            p: Vector3::from(*p),
            q: UnitQuaternion::identity(),
        })
        .collect();
    write_tum(&dir.join("traj.tum"), &poses).unwrap();
}

fn write_cloud(path: &Path, xyz: &[[f64; 3]]) {
    let points = Points {
        xyz: xyz.to_vec(),
        rgb: None,
        intensity: None,
        source: None,
        time: None,
        flags: None,
        scans: Vec::new(),
    };
    write_points(path, PointFormat::Las, &points, &Progress::new()).unwrap();
}

/// A scan of a model: only its run folder `<root>/<name>` exists, found relative to the model
/// folder `<root>/modell`. Its raw data and map are nowhere.
fn member(name: &str, transform: [f64; 16], included: bool) -> ModelMember {
    let nowhere = PathBuf::from("/gibt/es/nicht").join(name);
    ModelMember {
        scan: nowhere.clone(),
        scan_rel: None,
        slam_dir: nowhere.clone(),
        slam_dir_rel: Some(format!("../{name}")),
        map_dir: nowhere,
        map_dir_rel: None,
        slam: SlamSettings::default(),
        transform,
        placed_against: None,
        method: "bezug".into(),
        verdict: "sicher".into(),
        reason: None,
        overlap: 1.0,
        residual_median_mm: 0.0,
        residual_rms_mm: 0.0,
        agreement: 1.0,
        wall_residual_mm: None,
        pair_rms_mm: None,
        included,
        color_gain: [1.0; 3],
        points: 0,
        clean_id: Some(format!("clean-{name}")),
    }
}

fn model_report(members: Vec<ModelMember>) -> ModelReport {
    ModelReport {
        version: MERGE_VERSION,
        input_id: "00000000000000bb".into(),
        members,
        points: 0,
        seconds_total: 0.0,
        peak_rss_bytes: 0,
        include_unsure: false,
        warnings: Vec::new(),
    }
}

/// Writes `<root>/modell` with `xyz` and `report`; the request of its plan.
fn write_model(root: &Path, xyz: &[[f64; 3]], report: &ModelReport) -> PlanRequest {
    let dir = root.join("modell");
    fs::create_dir_all(&dir).unwrap();
    write_cloud(&dir.join(MODEL_LAS), xyz);
    write_model_report(&dir, report);
    PlanRequest {
        source: PlanSource::Model(dir.clone()),
        model: None,
        out_dir: dir.join("grundriss"),
        settings: PlanSettings::default(),
    }
}

fn write_model_report(dir: &Path, report: &ModelReport) {
    fs::write(
        dir.join(MODEL_REPORT),
        serde_json::to_string_pretty(report).unwrap(),
    )
    .unwrap();
}

/// A model of [`one_room`] from two scans, with the walk split between them.
fn small_model(root: &Path, change: impl FnOnce(&mut ModelReport)) -> PlanRequest {
    let data = one_room();
    let (first, second) = data.poses.split_at(data.poses.len() / 2);
    write_traj(&root.join("a"), first);
    write_traj(&root.join("b"), second);
    let mut report = model_report(vec![
        member("a", IDENTITY, true),
        member("b", IDENTITY, true),
    ]);
    change(&mut report);
    write_model(root, &data.xyz, &report)
}

fn copy_dir(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for entry in fs::read_dir(from).unwrap().flatten() {
        let target = to.join(entry.file_name());
        if entry.file_type().unwrap().is_dir() {
            copy_dir(&entry.path(), &target);
        } else {
            fs::copy(entry.path(), target).unwrap();
        }
    }
}

#[test]
fn map_gives_plan_and_files() {
    let dir = tmp("plan-map");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);
    let report = run(&req);

    for f in [PLAN_REPORT, TOP_VIEW_FILE] {
        assert!(req.out_dir.join(f).is_file(), "{f} fehlt");
    }
    let back = read_plan(&req.out_dir).unwrap();
    assert_eq!(back, report);
    assert_eq!(back.version, PLAN_VERSION);
    assert_eq!(back.plan.rooms.len(), 1, "{:?}", back.plan.rooms);
    assert!((back.plan.rooms[0].area - 20.0).abs() < 0.5);
    assert_eq!(back.points, data.xyz.len() as u64);
    assert_eq!(back.files, [TOP_VIEW_FILE, PLAN_REPORT]);
    let picture = req.out_dir.join(TOP_VIEW_FILE);
    assert_eq!(back.bytes.len(), 1);
    assert_eq!(
        back.bytes[TOP_VIEW_FILE],
        fs::metadata(&picture).unwrap().len()
    );
    assert!(plan_fits(&req));

    let text = fs::read_to_string(req.out_dir.join(PLAN_REPORT)).unwrap();
    let json: Value = serde_json::from_str(&text).unwrap();
    assert!(json["programm"]["version"].is_string());

    let file = fs::File::open(&picture).unwrap();
    let reader = png::Decoder::new(std::io::BufReader::new(file))
        .read_info()
        .unwrap();
    let info = reader.info();
    assert_eq!(
        (info.width, info.height),
        (back.top_view.width, back.top_view.height)
    );
    assert_eq!(info.color_type, png::ColorType::Rgba);
    assert!(back.top_view.cell > 0.0);
}

#[test]
fn trajectory_is_found_one_folder_up() {
    let dir = tmp("plan-up");
    let data = one_room();
    let map = dir.join("karte-001");
    write_map_folder(&map, &data.xyz, &data.poses);
    fs::rename(map.join("traj.tum"), dir.join("traj.tum")).unwrap();
    let req = map_request(&map);
    assert_eq!(run(&req).plan.rooms.len(), 1);
    assert!(plan_fits(&req));
}

#[test]
fn clean_las_serves_without_color_las() {
    let dir = tmp("plan-clean");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);
    fs::rename(dir.join("color.las"), dir.join("clean.las")).unwrap();
    assert_eq!(run(&req).plan.rooms.len(), 1);
    assert!(plan_fits(&req));
}

#[test]
fn what_is_missing_is_named() {
    let dir = tmp("plan-missing");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);

    fs::rename(dir.join("traj.tum"), dir.join("anders.tum")).unwrap();
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("Laufbahn") && e.contains("traj.tum"), "{e}");
    fs::rename(dir.join("anders.tum"), dir.join("traj.tum")).unwrap();

    fs::rename(dir.join("color.las"), dir.join("anders.las")).unwrap();
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("color.las") && e.contains("clean.las"), "{e}");
    fs::rename(dir.join("anders.las"), dir.join("color.las")).unwrap();

    fs::write(dir.join("map.json"), "{\"steps\":{}}").unwrap();
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("bereinigt"), "{e}");
    assert!(!req.out_dir.exists());
}

#[test]
fn source_is_detected() {
    let dir = tmp("plan-detect");
    let map = dir.join("karte");
    let model = dir.join("modell");
    let other = dir.join("anderes");
    for d in [&map, &model, &other] {
        fs::create_dir_all(d).unwrap();
    }
    fs::write(map.join("map.json"), "{}").unwrap();
    fs::write(model.join(MODEL_REPORT), "{}").unwrap();

    assert_eq!(
        PlanSource::detect(&map).unwrap(),
        PlanSource::Map(map.clone())
    );
    assert_eq!(
        PlanSource::detect(&model).unwrap(),
        PlanSource::Model(model.clone())
    );
    assert_eq!(PlanSource::detect(&model).unwrap().dir(), model.as_path());
    let e = PlanSource::detect(&other).unwrap_err().to_string();
    assert!(e.contains("weder eine Karte"), "{e}");
    assert!(PlanSource::detect(&dir.join("fehlt")).is_err());
}

#[test]
fn model_gives_plan() {
    let root = tmp("plan-model");
    // Rooms a and b share a door, room c stands apart.
    let data = Scene::new()
        .spacing(SPACING)
        .room([0.0, 0.0], [4.0, 4.0], FLOOR, HEIGHT)
        .door(1, 1.5, 0.9, 2.05)
        .room([4.12, 0.0], [8.12, 4.0], FLOOR, HEIGHT)
        .door(3, 1.6, 0.9, 2.05)
        .room([20.0, 0.0], [24.0, 4.0], FLOOR, HEIGHT)
        .build();
    let walk = |x: f64| {
        Scene::new()
            .walk(&[
                [x + 1.0, 1.0, 0.0],
                [x + 3.0, 1.0, 0.0],
                [x + 3.0, 3.0, 0.0],
                [x + 1.0, 3.0, 0.0],
            ])
            .build()
            .poses
    };
    write_traj(&root.join("a"), &walk(0.0));
    // Only with its `transform` is this walk in room b: as it stands, it is far from any room.
    let in_scan: Vec<[f64; 3]> = walk(4.12).into_iter().map(in_turned_scan).collect();
    write_traj(&root.join("b"), &in_scan);
    write_traj(&root.join("c"), &walk(20.0));

    let report = model_report(vec![
        member("a", IDENTITY, true),
        member("b", TURNED, true),
        member("c", IDENTITY, false),
    ]);
    let req = write_model(&root, &data.xyz, &report);
    let done = run(&req);
    assert_eq!(done.plan.rooms.len(), 2, "{:?}", done.plan.rooms);
    assert!(plan_fits(&req));

    // The plan lies in the frame of the model.
    for room in &done.plan.rooms {
        for corner in &room.outline {
            let p = done.plan.frame.to_map([corner[0], corner[1], 0.0]);
            assert!((-0.05..=8.17).contains(&p[0]), "{p:?}");
            assert!((-0.05..=4.05).contains(&p[1]), "{p:?}");
            assert!((p[2] - FLOOR).abs() < 0.02, "{p:?}");
        }
    }

    // The scan left out adds no poses: with it, its room is found too.
    let mut with_c = report.clone();
    with_c.members[2].included = true;
    write_model_report(req.source.dir(), &with_c);
    assert!(!plan_fits(&req), "a scan more makes it stale");
    assert_eq!(run(&req).plan.rooms.len(), 3);

    // The model as the caller read it counts like the file.
    let read = read_model(req.source.dir()).unwrap();
    let given = PlanRequest {
        model: Some(read),
        ..req.clone()
    };
    assert!(plan_fits(&given));
}

#[test]
fn traj_or_cloud_or_storey_change_makes_it_stale() {
    let dir = tmp("plan-stale");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);
    run(&req);
    assert!(plan_fits(&req));

    // Another setting.
    let mut other = req.clone();
    other.settings.storey = Some(1);
    assert!(!plan_fits(&other));

    // Another trajectory.
    let traj = dir.join("traj.tum");
    let kept = fs::read(&traj).unwrap();
    fs::write(&traj, [kept.as_slice(), b"# noch eine Zeile\n"].concat()).unwrap();
    assert!(!plan_fits(&req));
    fs::write(&traj, &kept).unwrap();
    assert!(plan_fits(&req));

    // Another cloud: another size, or another file.
    let cloud = dir.join("color.las");
    let kept = fs::read(&cloud).unwrap();
    write_cloud(&cloud, &data.xyz[..data.xyz.len() - 1]);
    assert!(!plan_fits(&req));
    fs::write(&cloud, &kept).unwrap();
    assert!(plan_fits(&req));
    fs::rename(&cloud, dir.join("clean.las")).unwrap();
    assert!(!plan_fits(&req));
    fs::rename(dir.join("clean.las"), &cloud).unwrap();
    assert!(plan_fits(&req));

    // A map cleaned anew.
    let report = dir.join("map.json");
    let kept = fs::read(&report).unwrap();
    fs::write(&report, "{\"steps\":{\"clean\":{\"input_id\":\"bb\"}}}").unwrap();
    assert!(!plan_fits(&req));
    fs::write(&report, &kept).unwrap();
    assert!(plan_fits(&req));

    // The picture is not the one of the report.
    let picture = req.out_dir.join(TOP_VIEW_FILE);
    let kept = fs::read(&picture).unwrap();
    fs::write(&picture, b"anderes Bild").unwrap();
    assert!(!plan_fits(&req));
    fs::remove_file(&picture).unwrap();
    assert!(!plan_fits(&req));
    fs::write(&picture, &kept).unwrap();
    assert!(plan_fits(&req));

    // A folder without its report never counts.
    fs::remove_file(req.out_dir.join(PLAN_REPORT)).unwrap();
    assert!(!plan_fits(&req));
}

#[test]
fn a_change_in_the_model_makes_it_stale() {
    let root = tmp("plan-model-stale");
    let req = small_model(&root, |_| {});
    run(&req);
    assert!(plan_fits(&req));
    let dir = req.source.dir().to_path_buf();
    let report = read_model(&dir).unwrap();

    // The trajectory of a scan.
    let traj = root.join("b/traj.tum");
    let kept = fs::read(&traj).unwrap();
    fs::write(&traj, [kept.as_slice(), b"# noch eine Zeile\n"].concat()).unwrap();
    assert!(!plan_fits(&req));
    fs::write(&traj, &kept).unwrap();
    assert!(plan_fits(&req));

    // The map of a scan computed anew, the model merged anew, a scan left out.
    let changes: [fn(&mut ModelReport); 3] = [
        |m| m.members[0].clean_id = Some("neu".into()),
        |m| m.input_id = "anders".into(),
        |m| m.members[1].included = false,
    ];
    for change in changes {
        let mut changed = report.clone();
        change(&mut changed);
        write_model_report(&dir, &changed);
        assert!(!plan_fits(&req));
    }
    write_model_report(&dir, &report);
    assert!(plan_fits(&req));

    // The cloud of the model.
    write_cloud(&dir.join(MODEL_LAS), &one_room().xyz[1..]);
    assert!(!plan_fits(&req));
}

#[test]
fn model_of_another_version_is_refused() {
    let root = tmp("plan-model-version");
    let req = small_model(&root, |m| m.version = MERGE_VERSION + 1);
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("Modell veraltet"), "{e}");
    assert!(!req.out_dir.exists());
    assert!(!plan_fits(&req));
}

#[test]
fn model_without_a_scan_or_a_trajectory_says_so() {
    let root = tmp("plan-model-empty");
    let req = small_model(&root, |m| {
        m.members.iter_mut().for_each(|s| s.included = false);
    });
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("keinen Scan"), "{e}");

    let root = tmp("plan-model-no-traj");
    let req = small_model(&root, |_| {});
    fs::remove_file(root.join("b/traj.tum")).unwrap();
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("Laufbahn") && e.contains("b"), "{e}");
    assert!(!req.out_dir.exists());
}

#[test]
fn plan_needs_no_raw_data() {
    let dir = tmp("plan-no-raw");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    assert_eq!(fs::read_dir(&*dir).unwrap().count(), 3, "only the map");
    let req = map_request(&dir);
    assert_eq!(run(&req).plan.rooms.len(), 1);
    assert!(plan_fits(&req));

    let root = tmp("plan-no-raw-model");
    let req = small_model(&root, |_| {});
    let read = read_model(req.source.dir()).unwrap();
    assert!(read.members.iter().all(|m| !m.scan.exists()));
    assert!(read.members.iter().all(|m| !m.map_dir.exists()));
    assert_eq!(run(&req).plan.rooms.len(), 1);
    assert!(plan_fits(&req));
}

#[test]
fn several_storeys_write_nothing() {
    let dir = tmp("plan-storeys");
    let data = two_storeys();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);
    let found = match run_plan(&req, &Progress::new()).unwrap() {
        PlanOutcome::SeveralStoreys(found) => found,
        PlanOutcome::Done(_) => panic!("ein Stockwerk geraten"),
    };
    assert_eq!(found.len(), 2, "{found:?}");
    assert!(!req.out_dir.exists(), "nichts geschrieben");
    assert!(!plan_fits(&req));

    let mut upper = req.clone();
    upper.settings.storey = Some(2);
    let report = run(&upper);
    assert!((report.plan.storey.floor_z - found[1].floor_z).abs() < 1e-3);
    assert_eq!(report.settings.storey, Some(2));
    assert!(plan_fits(&upper));
    assert!(!plan_fits(&req), "another setting");

    upper.settings.storey = Some(3);
    let e = run_plan(&upper, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("Stockwerk 3"), "{e}");
}

#[test]
fn no_plan_is_a_clear_error_and_no_report() {
    let dir = tmp("plan-empty");
    write_map_folder(&dir, &[], &[[1.0, 1.0, 0.0]]);
    let req = map_request(&dir);
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("zu wenige Punkte"), "{e}");
    assert!(!req.out_dir.exists(), "kein halber Ordner");
    assert!(!plan_fits(&req));

    let dir = tmp("plan-only-a-floor");
    let only_a_floor = Scene::new()
        .spacing(0.02)
        .slab([0.0, 0.0], [5.0, 4.0], FLOOR)
        .walk(&LOOP)
        .build();
    write_map_folder(&dir, &only_a_floor.xyz, &only_a_floor.poses);
    let req = map_request(&dir);
    let e = run_plan(&req, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains("Keine Wände"), "{e}");
    assert!(!req.out_dir.exists(), "kein halber Ordner");
}

#[test]
fn cancel_leaves_no_report() {
    let dir = tmp("plan-cancel");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);

    let cancelled = Progress::new();
    cancelled.cancel();
    assert_eq!(run_plan(&req, &cancelled), Err(StepError::Cancelled));
    assert!(!req.out_dir.exists());

    // In the middle of the computation: a plan of an earlier run stays as it was.
    run(&req);
    let report = req.out_dir.join(PLAN_REPORT);
    let before = fs::read(&report).unwrap();
    let progress = Progress::new();
    let handle = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == "Räume" {
            handle.cancel();
        }
    });
    assert_eq!(run_plan(&req, &progress), Err(StepError::Cancelled));
    assert_eq!(fs::read(&report).unwrap(), before);
    assert!(plan_fits(&req));
}

#[test]
fn killed_run_is_cleaned_up() {
    let dir = tmp("plan-killed");
    let data = one_room();
    write_map_folder(&dir, &data.xyz, &data.poses);
    let req = map_request(&dir);
    fs::create_dir_all(&req.out_dir).unwrap();
    for f in ["draufsicht.png.tmp", "grundriss.json.tmp", "fremd.las.tmp"] {
        fs::write(req.out_dir.join(f), b"halb").unwrap();
    }
    run(&req);
    assert!(!req.out_dir.join("draufsicht.png.tmp").exists());
    assert!(!req.out_dir.join("grundriss.json.tmp").exists());
    assert!(req.out_dir.join("fremd.las.tmp").exists(), "nicht unsere");
    assert!(plan_fits(&req));
}

#[test]
fn report_survives_a_moved_project() {
    let root = tmp("plan-moved");
    let data = one_room();
    let old = root.join("alt");
    write_map_folder(&old.join("karte-001"), &data.xyz, &data.poses);
    let report = run(&map_request(&old.join("karte-001")));
    copy_dir(&old, &root.join("neu"));
    fs::remove_dir_all(&old).unwrap();

    let moved = map_request(&root.join("neu/karte-001"));
    assert_eq!(read_plan(&moved.out_dir).unwrap(), report);
    assert!(plan_fits(&moved));

    // A model finds the runs of its scans relative to its own folder.
    let root = tmp("plan-moved-model");
    let old = root.join("alt");
    let req = small_model(&old, |_| {});
    let report = run(&req);
    copy_dir(&old, &root.join("neu"));
    fs::remove_dir_all(&old).unwrap();

    let dir = root.join("neu/modell");
    let moved = PlanRequest {
        source: PlanSource::Model(dir.clone()),
        model: None,
        out_dir: dir.join("grundriss"),
        settings: PlanSettings::default(),
    };
    assert_eq!(read_plan(&moved.out_dir).unwrap(), report);
    assert!(plan_fits(&moved));
}
