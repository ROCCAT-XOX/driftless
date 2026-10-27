//! `run_merge` on synthetic scans of one room with two boxes: scan A at the origin, scan B
//! from (0.8, 0.5, 0) turned by 30°, scan C of the bare room from its centre (it fits turned
//! by 180° as well). A has a `color.las` coloured by the place in the room, B and C have none.

use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{empty_room_bag, furnished_room_bag, tmp};
use driftless_map::{load_stage, write_las, Cloud};
use driftless_slam::align::AlignConfig;
use driftless_steps::map::{clean_stage_id, map_input_for, run_map, MapRequest, CLEAN_BIN};
use driftless_steps::merge::{
    member_transform, merge_input_id, read_model, run_merge, MergeMember, MergeRequest,
    ModelReport, MODEL_LAS, MODEL_PLY, MODEL_REPORT,
};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::StepError;
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// Pose of scan B in the frame of scan A.
fn truth_b() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(0.8, 0.5, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 30f64.to_radians()),
    )
}

/// Pose of scan C (bare room, from its centre) in the frame of scan A.
fn truth_c() -> Isometry3<f64> {
    Isometry3::translation(1.0, 1.0, 0.0)
}

/// Colour of a point of the room (room frame = frame of scan A): a checkerboard of 0.5 m.
fn room_color(p: &Point3<f64>, brightness: f64) -> [u8; 3] {
    let cell = (p.x * 2.0).floor() + (p.y * 2.0).floor() + (p.z * 2.0).floor();
    let base = if (cell as i64).rem_euclid(2) == 0 {
        [200.0, 150.0, 100.0]
    } else {
        [100.0, 75.0, 50.0]
    };
    base.map(|c| (c * brightness).round() as u8)
}

struct Scene {
    a: PathBuf,
    b: PathBuf,
    c: PathBuf,
}

/// Scan folder with SLAM and map in `<scan>/driftless`.
fn prepare(scan: &Path) {
    let out = scan.join("driftless");
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: out.clone(),
        settings: SlamSettings::default(),
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let input = map_input_for(scan, &out, &out, SlamSettings::default());
    run_map(
        &MapRequest {
            input,
            out_dir: out,
        },
        &Progress::new(),
    )
    .unwrap();
}

/// The cleaned cloud of the map of `scan` in `map_dir`.
fn clean_cloud(scan: &Path, map_dir: &Path) -> Cloud {
    let slam_dir = scan.join("driftless");
    let input = map_input_for(scan, &slam_dir, map_dir, SlamSettings::default());
    let id = clean_stage_id(&input).unwrap();
    load_stage(&map_dir.join(CLEAN_BIN), &id).unwrap().0
}

/// Writes `color.las` into `map_dir`: the cleaned cloud of `scan` coloured by
/// [`room_color`], its points taken to the room frame by `pose`.
fn write_color(scan: &Path, map_dir: &Path, pose: &Isometry3<f64>, brightness: f64) {
    let mut cloud = clean_cloud(scan, map_dir);
    let rgb = cloud
        .xyz
        .iter()
        .map(|p| {
            let p = Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            room_color(&(pose * p), brightness)
        })
        .collect();
    cloud.rgb = Some(rgb);
    cloud.colored = Some(vec![true; cloud.len()]);
    write_las(&map_dir.join("color.las"), &cloud).unwrap();
}

fn scene() -> &'static Scene {
    static SCENE: OnceLock<Scene> = OnceLock::new();
    SCENE.get_or_init(|| {
        let root = tmp("steps-merge");
        let (a, b, c) = (root.join("a"), root.join("b"), root.join("c"));
        for d in [&a, &b, &c] {
            fs::create_dir_all(d).unwrap();
        }
        furnished_room_bag(&a, true, [0.0; 3], 0.0);
        furnished_room_bag(&b, true, [0.8, 0.5, 0.0], 30f64.to_radians());
        empty_room_bag(&c, true, [1.0, 1.0, 0.0], 0.0);
        for d in [&a, &b, &c] {
            prepare(d);
        }
        write_color(&a, &a.join("driftless"), &Isometry3::identity(), 1.0);
        Scene { a, b, c }
    })
}

fn member(scan: &Path) -> MergeMember {
    let run = scan.join("driftless");
    MergeMember {
        scan: scan.to_path_buf(),
        slam_dir: run.clone(),
        map_dir: run,
        slam: SlamSettings::default(),
    }
}

fn request(members: Vec<MergeMember>, out: &Path) -> MergeRequest {
    MergeRequest {
        members,
        out_dir: out.to_path_buf(),
        align: AlignConfig::default(),
        voxel: 0.01,
        smooth: true,
        include_unsure: false,
        pairs: Vec::new(),
    }
}

/// Fresh output folder below the scene.
fn out_dir(name: &str) -> PathBuf {
    let dir = scene().a.parent().unwrap().join("out").join(name);
    let _ = fs::remove_dir_all(&dir);
    dir
}

/// Per point of `modell.las`: (point source id, user data = 1 if uncoloured).
fn las_points(dir: &Path) -> Vec<(u16, u8)> {
    let mut r = las::Reader::from_path(dir.join(MODEL_LAS)).unwrap();
    let n = r.header().number_of_points();
    r.read_points(n)
        .unwrap()
        .points()
        .map(|p| {
            let p = p.unwrap();
            (p.point_source_id, p.user_data)
        })
        .collect()
}

fn sources(dir: &Path) -> BTreeSet<u16> {
    las_points(dir).into_iter().map(|(s, _)| s).collect()
}

fn pose_error(est: &Isometry3<f64>, truth: &Isometry3<f64>) -> (f64, f64) {
    // At a point in the middle of the room, as seen from the merged scan.
    let c = truth.inverse() * Point3::new(1.0, 1.0, 0.0);
    let trans = (est * c - truth * c).norm();
    let rot = (est.rotation.inverse() * truth.rotation)
        .angle()
        .to_degrees();
    (trans, rot)
}

fn describe(r: &ModelReport) -> String {
    serde_json::to_string_pretty(&r.members).unwrap()
}

#[test]
fn merges_two_scans() {
    let s = scene();
    let out = out_dir("zwei");
    let req = request(vec![member(&s.a), member(&s.b)], &out);
    let report = run_merge(&req, &Progress::new()).unwrap();
    let b = &report.members[1];
    assert_eq!(b.verdict, "sicher", "{}", describe(&report));
    assert_eq!(b.placed_against, Some(0));
    assert!(b.included);
    assert_eq!(report.members[0].method, "bezug");
    assert_eq!(member_transform(&report.members[0]), Isometry3::identity());
    let (dt, dr) = pose_error(&member_transform(b), &truth_b());
    assert!(dt < 0.02 && dr < 0.5, "error {dt:.4} m {dr:.3}°");

    assert_eq!(sources(&out), BTreeSet::from([0, 1]));
    let points: u64 = report.members.iter().map(|m| m.points).sum();
    assert_eq!(points, report.points);
    assert_eq!(las_points(&out).len() as u64, report.points);
    let ply = fs::read(out.join(MODEL_PLY)).unwrap();
    let head = String::from_utf8_lossy(&ply[..400.min(ply.len())]).into_owned();
    assert!(head.contains("property ushort scan"), "{head}");

    let back = read_model(&out).unwrap();
    assert_eq!(back.input_id, report.input_id);
    assert_eq!(back.input_id, merge_input_id(&req).unwrap());
    assert_eq!(back.members.len(), 2);
    assert_eq!(back.members[1].transform, b.transform);
    assert_eq!(back.points, report.points);
    assert!(!out.join(format!("{MODEL_LAS}.tmp")).exists());

    // The same request gives the same files.
    let again = out_dir("zwei-nochmal");
    run_merge(&request(req.members.clone(), &again), &Progress::new()).unwrap();
    for f in [MODEL_LAS, MODEL_PLY] {
        assert!(
            fs::read(out.join(f)).unwrap() == fs::read(again.join(f)).unwrap(),
            "{f} differs between two runs"
        );
    }

    // A folder that already holds a model is not overwritten.
    let err = run_merge(&req, &Progress::new()).unwrap_err();
    assert!(
        matches!(&err, StepError::Failed(m) if m.contains("schon ein Modell")),
        "{err}"
    );
}

#[test]
fn member_without_color_merges() {
    let s = scene();
    let out = out_dir("ohne-farbe");
    let report = run_merge(
        &request(vec![member(&s.a), member(&s.b)], &out),
        &Progress::new(),
    )
    .unwrap();
    assert!(report.members[1].included, "{}", describe(&report));
    assert_eq!(report.members[1].color_gain, [1.0; 3]);
    let points = las_points(&out);
    let of = |src: u16| points.iter().filter(move |p| p.0 == src);
    assert!(of(1).count() > 1000, "{} points of B", of(1).count());
    assert!(of(1).all(|p| p.1 == 1), "points of B are marked uncoloured");
    assert!(of(0).all(|p| p.1 == 0), "points of A keep their colour");
    let mut r = las::Reader::from_path(out.join(MODEL_LAS)).unwrap();
    assert!(r.header().point_format().has_color);
    let first = r.read_points(1).unwrap();
    assert!(first.points().next().unwrap().unwrap().color.is_some());
}

#[test]
fn color_gain_evens_out_members() {
    let s = scene();
    // B coloured like A but 20 % darker, in a copy of its map folder.
    let dark = s.b.parent().unwrap().join("b-dunkel");
    let _ = fs::remove_dir_all(&dark);
    fs::create_dir_all(&dark).unwrap();
    for f in [CLEAN_BIN, "map.json"] {
        fs::copy(s.b.join("driftless").join(f), dark.join(f)).unwrap();
    }
    write_color(&s.b, &dark, &truth_b(), 0.8);
    let out = out_dir("farbangleich");
    let mut b = member(&s.b);
    b.map_dir = dark;
    let report = run_merge(&request(vec![member(&s.a), b], &out), &Progress::new()).unwrap();
    let gain = report.members[1].color_gain;
    assert!(
        gain.iter().all(|g| (g - 1.25).abs() < 0.05),
        "gain {gain:?}"
    );
    assert_eq!(report.members[0].color_gain, [1.0; 3]);
    let points = las_points(&out);
    assert!(points.iter().any(|p| p.0 == 1 && p.1 == 0), "B is coloured");
}

#[test]
fn refuses_bad_members() {
    let s = scene();
    let out = out_dir("abgelehnt");
    let failed = |req: &MergeRequest| match run_merge(req, &Progress::new()) {
        Err(StepError::Failed(m)) => m,
        other => panic!("expected a failure, got {other:?}"),
    };

    let m = failed(&request(vec![member(&s.a)], &out));
    assert!(m.contains("mindestens zwei"), "{m}");

    let m = failed(&request(vec![member(&s.a), member(&s.a)], &out));
    assert!(m.contains("doppelt"), "{m}");

    // A map folder without clean.bin.
    let empty = s.a.parent().unwrap().join("leer");
    fs::create_dir_all(&empty).unwrap();
    let mut b = member(&s.b);
    b.map_dir = empty.clone();
    let m = failed(&request(vec![member(&s.a), b], &out));
    assert!(m.contains("Karte fehlt oder ist veraltet"), "{m}");
    assert!(m.contains(&empty.display().to_string()), "{m}");

    assert!(!out.join(MODEL_REPORT).exists());
    assert!(!out.join(MODEL_LAS).exists());
}

#[test]
fn cancel_leaves_no_report() {
    let s = scene();
    let out = out_dir("abgebrochen");
    let req = request(vec![member(&s.a), member(&s.b)], &out);
    let p = Progress::new();
    p.cancel();
    assert_eq!(run_merge(&req, &p).unwrap_err(), StepError::Cancelled);
    for f in [MODEL_REPORT, MODEL_LAS, MODEL_PLY] {
        assert!(!out.join(f).exists(), "{f} written");
    }

    // Cancelled while aligning: nothing either.
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Ausrichten" {
            handle.cancel();
        }
    });
    assert_eq!(run_merge(&req, &p).unwrap_err(), StepError::Cancelled);
    for f in [MODEL_REPORT, MODEL_LAS, MODEL_PLY] {
        assert!(!out.join(f).exists(), "{f} written");
    }
}

#[test]
fn same_input_same_id() {
    let s = scene();
    let out = out_dir("kennung");
    let req = request(vec![member(&s.a), member(&s.b)], &out);
    let id = merge_input_id(&req).unwrap();
    assert_eq!(id, merge_input_id(&req.clone()).unwrap());
    let coarser = MergeRequest {
        voxel: 0.02,
        ..req.clone()
    };
    assert_ne!(id, merge_input_id(&coarser).unwrap());
    let swapped = request(vec![member(&s.b), member(&s.a)], &out);
    assert_ne!(id, merge_input_id(&swapped).unwrap());
}

#[test]
fn unsure_member_left_out() {
    let s = scene();
    let out = out_dir("unsicher");
    let members = vec![member(&s.a), member(&s.b), member(&s.c)];
    let report = run_merge(&request(members.clone(), &out), &Progress::new()).unwrap();
    let c = &report.members[2];
    assert_eq!(c.verdict, "unsicher", "{}", describe(&report));
    assert!(c.reason.as_deref().is_some_and(|r| !r.is_empty()));
    assert!(!c.included);
    assert_eq!(c.points, 0);
    assert!(report.members[1].included);
    assert_eq!(sources(&out), BTreeSet::from([0, 1]));

    let out = out_dir("unsicher-dabei");
    let req = MergeRequest {
        include_unsure: true,
        ..request(members, &out)
    };
    let report = run_merge(&req, &Progress::new()).unwrap();
    let c = &report.members[2];
    assert_eq!(c.verdict, "unsicher", "{}", describe(&report));
    assert!(c.included);
    assert!(c.points > 0);
    assert_eq!(sources(&out), BTreeSet::from([0, 1, 2]));
    // Either of the two poses of the bare room (right, or turned by 180° about its centre).
    let turned = Isometry3::translation(1.0, 1.0, 0.0)
        * Isometry3::rotation(Vector3::z() * std::f64::consts::PI)
        * Isometry3::translation(-1.0, -1.0, 0.0)
        * truth_c();
    let est = member_transform(c);
    let (dt, dr) = pose_error(&est, &truth_c());
    let (tt, tr) = pose_error(&est, &turned);
    assert!(
        (dt < 0.05 && dr < 1.0) || (tt < 0.05 && tr < 1.0),
        "C at neither pose: {dt:.3} m {dr:.2}° / {tt:.3} m {tr:.2}°"
    );
}
