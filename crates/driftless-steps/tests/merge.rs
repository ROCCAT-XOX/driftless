//! `run_merge` on synthetic scans of one room with two boxes: scan A at the origin, scan B
//! from (0.8, 0.5, 0) turned by 30°, scan C of the bare room from its centre (it fits turned
//! by 180° as well). A has a `color.las` coloured by the place in the room, B and C have none.
//! Point pairs marked by hand place C.

use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{empty_room_bag, furnished_room_bag, tmp};
use driftless_map::{load_stage, write_las, Cloud};
use driftless_slam::align::AlignConfig;
use driftless_steps::map::{clean_stage_id, map_input_for, run_map, MapRequest, CLEAN_BIN};
use driftless_steps::merge::{
    member_transform, merge_input_id, read_model, run_merge, MergeMember, MergeRequest,
    ModelMember, ModelReport, MODEL_LAS, MODEL_PLY, MODEL_REPORT,
};
use driftless_steps::pairs::{PointPairEntry, PointPairs, POINT_PAIRS_FILE};
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
    // Every folder absolute and resolved (in its plain form on Windows), and relative to the
    // model folder beside it.
    let canon = |p: &Path| driftless_io::paths::canonical(p).unwrap();
    for (m, scan) in back.members.iter().zip([&s.a, &s.b]) {
        let name = scan.file_name().unwrap().to_str().unwrap();
        assert_eq!(m.scan, canon(scan));
        assert_eq!(m.map_dir, canon(&scan.join("driftless")));
        assert_eq!(
            m.scan_rel.as_deref(),
            Some(format!("../../{name}").as_str())
        );
        let run = format!("../../{name}/driftless");
        assert_eq!(m.slam_dir_rel.as_deref(), Some(run.as_str()));
        assert_eq!(m.map_dir_rel.as_deref(), Some(run.as_str()));
    }
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

/// A member of a hand-written model with the folders `abs` and, relative to the model
/// folder, `rel`.
fn written_member(abs: &Path, rel: Option<&str>) -> ModelMember {
    ModelMember {
        scan: abs.to_path_buf(),
        scan_rel: rel.map(String::from),
        slam_dir: abs.to_path_buf(),
        slam_dir_rel: rel.map(String::from),
        map_dir: abs.to_path_buf(),
        map_dir_rel: rel.map(String::from),
        slam: SlamSettings::default(),
        transform: std::array::from_fn(|k| if k % 5 == 0 { 1.0 } else { 0.0 }),
        placed_against: None,
        method: "bezug".into(),
        verdict: "sicher".into(),
        reason: None,
        overlap: 1.0,
        residual_median_mm: 0.0,
        residual_rms_mm: 0.0,
        agreement: 1.0,
        wall_residual_mm: 0.0,
        pair_rms_mm: None,
        included: true,
        color_gain: [1.0; 3],
        points: 1,
        clean_id: None,
    }
}

/// `read_model` takes a relative folder (below the model folder) where it exists, also when
/// the absolute one exists too (a copy of a project), else the absolute one; a model written
/// before the relative paths reads as written.
#[test]
fn read_model_takes_the_relative_folder_first() {
    let dir = driftless_io::paths::canonical(&tmp("modell-relativ")).unwrap();
    let old = dir.join("alt").join("scans").join("a");
    let new = dir.join("neu").join("scans").join("a");
    let model = dir.join("neu").join("modelle").join("modell-001");
    for d in [&old, &new, &model] {
        fs::create_dir_all(d).unwrap();
    }
    let write = |members: Vec<ModelMember>| {
        let report = ModelReport {
            version: 2,
            input_id: "0".into(),
            members,
            points: 1,
            seconds_total: 0.0,
            peak_rss_bytes: 0,
            include_unsure: false,
            warnings: Vec::new(),
        };
        let text = serde_json::to_string_pretty(&report).unwrap();
        fs::write(model.join(MODEL_REPORT), text).unwrap();
    };
    write(vec![written_member(&old, Some("../../scans/a"))]);
    let m = read_model(&model).unwrap().members.remove(0);
    assert_eq!((&m.scan, &m.slam_dir, &m.map_dir), (&new, &new, &new));
    assert_eq!(
        m.map_dir_rel.as_deref(),
        Some("../../scans/a"),
        "kept as read"
    );
    // The relative folder is gone: the absolute one.
    fs::remove_dir_all(&new).unwrap();
    assert_eq!(read_model(&model).unwrap().members[0].map_dir, old);
    // Written before the relative paths: no keys at all, read as written.
    write(vec![written_member(&old, None)]);
    let text = fs::read_to_string(model.join(MODEL_REPORT)).unwrap();
    let mut json: serde_json::Value = serde_json::from_str(&text).unwrap();
    for key in ["scan_rel", "slam_dir_rel", "map_dir_rel"] {
        json["members"][0].as_object_mut().unwrap().remove(key);
    }
    json.as_object_mut().unwrap().remove("warnings");
    fs::write(model.join(MODEL_REPORT), json.to_string()).unwrap();
    let back = read_model(&model).unwrap();
    assert_eq!(back.members[0].map_dir, old);
    assert!(back.members[0].map_dir_rel.is_none() && back.warnings.is_empty());
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

/// Three places seen by scan C, in its own frame and in the frame of scan A.
fn pairs_c_in_a() -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let in_c = vec![[-0.8, -0.6, 0.3], [0.9, -0.4, 1.2], [0.3, 0.9, 0.6]];
    let in_a = in_c
        .iter()
        .map(|p| {
            let q = truth_c() * Point3::from(*p);
            [q.x, q.y, q.z]
        })
        .collect();
    (in_c, in_a)
}

fn pair_entry(
    a: &Path,
    b: &Path,
    points_a: Vec<[f64; 3]>,
    points_b: Vec<[f64; 3]>,
) -> PointPairEntry {
    PointPairEntry {
        a: a.to_path_buf(),
        b: b.to_path_buf(),
        points_a,
        points_b,
        enabled: true,
        note: String::new(),
        added: "2026-10-27T12:00:00Z".into(),
        unknown: Default::default(),
    }
}

#[test]
fn pairs_rescue_unsure_member() {
    let s = scene();
    let out = out_dir("punktpaare");
    let (in_c, in_a) = pairs_c_in_a();
    let mut req = request(vec![member(&s.a), member(&s.b), member(&s.c)], &out);
    // Marked from A to C: the merge, aligning C to A, turns them round.
    req.pairs = vec![pair_entry(
        &s.a.join("driftless"),
        &s.c.join("driftless"),
        in_a,
        in_c,
    )];
    let report = run_merge(&req, &Progress::new()).unwrap();
    let c = &report.members[2];
    assert_eq!(c.verdict, "sicher", "{}", describe(&report));
    assert_eq!(c.method, "punktpaare");
    assert_eq!(c.placed_against, Some(0));
    assert!(c.included && c.points > 0);
    let (dt, dr) = pose_error(&member_transform(c), &truth_c());
    assert!(dt < 0.02 && dr < 0.5, "error {dt:.4} m {dr:.3}°");
    assert_eq!(report.members[1].verdict, "sicher");
    assert_ne!(report.members[1].method, "punktpaare");
    assert_eq!(sources(&out), BTreeSet::from([0, 1, 2]));
    assert_eq!(
        read_model(&out).unwrap().input_id,
        merge_input_id(&req).unwrap()
    );
}

#[test]
fn disabled_pairs_are_ignored() {
    let s = scene();
    let out = out_dir("punktpaare-aus");
    let plain = request(vec![member(&s.a), member(&s.b), member(&s.c)], &out);
    let (in_c, in_a) = pairs_c_in_a();
    // Wrong by 2 m: used, they would put C elsewhere.
    let wrong: Vec<[f64; 3]> = in_a.iter().map(|p| [p[0] + 2.0, p[1], p[2]]).collect();
    let mut off = pair_entry(
        &s.c.join("driftless"),
        &s.a.join("driftless"),
        in_c.clone(),
        wrong,
    );
    off.enabled = false;
    // Switched on, but for a scan that is not in the model.
    let elsewhere = pair_entry(&s.c.join("driftless"), &s.a.join("anderswo"), in_c, in_a);
    let req = MergeRequest {
        pairs: vec![off, elsewhere],
        ..plain.clone()
    };
    let id = merge_input_id(&req).unwrap();
    assert_eq!(id, merge_input_id(&plain).unwrap());
    let p = Progress::new();
    let report = run_merge(&req, &p).unwrap();
    let c = &report.members[2];
    assert_eq!(c.verdict, "unsicher", "{}", describe(&report));
    assert_ne!(c.method, "punktpaare");
    assert!(!c.included);
    assert_eq!(report.input_id, id);
    // The switched-on entry that names no two of the maps is not skipped silently: a warning
    // in the log and in `modell.json`; the switched-off one gives none.
    let logged: Vec<String> = p
        .take_log()
        .into_iter()
        .filter_map(|l| match l {
            driftless_io::progress::LogLine::Warning(w) => Some(w),
            driftless_io::progress::LogLine::Info(_) => None,
        })
        .collect();
    let expected = format!(
        "Punktpaare Eintrag 2: {} und {} sind nicht zwei der gewählten Karten, nicht genutzt",
        s.c.join("driftless").display(),
        s.a.join("anderswo").display()
    );
    assert_eq!(report.warnings, std::slice::from_ref(&expected));
    assert!(logged.contains(&expected), "{logged:?}");
    assert_eq!(read_model(&out).unwrap().warnings, report.warnings);

    // Switched on, the pairs count.
    let mut on = req.clone();
    on.pairs[0].enabled = true;
    assert_ne!(merge_input_id(&on).unwrap(), id);
}

#[test]
fn pairs_round_trip_and_reject_two_points() {
    let dir = tmp("punktpaare-datei");
    let path = dir.join("modelle").join(POINT_PAIRS_FILE);
    assert_eq!(PointPairs::load(&path).unwrap(), PointPairs::default());

    let (in_c, in_a) = pairs_c_in_a();
    let (a, b) = (
        Path::new("scans/a/driftless"),
        Path::new("scans/c/driftless"),
    );
    let mut pairs = PointPairs::default();
    let mut first = pair_entry(a, b, in_a.clone(), in_c.clone());
    first.note = "Ecken".into();
    assert_eq!(pairs.add(first).unwrap(), 0);
    assert_eq!(
        pairs
            .add(pair_entry(b, a, in_c.clone(), in_a.clone()))
            .unwrap(),
        1
    );
    pairs.set_enabled(0, false).unwrap();
    assert!(pairs.set_enabled(2, true).is_err());

    fs::create_dir_all(path.parent().unwrap()).unwrap();
    pairs.save(&path).unwrap();
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.contains("[[paar]]"), "{text}");
    assert!(text.contains("a = \"scans/a/driftless\""), "{text}");
    let back = PointPairs::load(&path).unwrap();
    assert_eq!(back, pairs);
    assert!(!back.entries[0].enabled && back.entries[1].enabled);

    // Relative to a base folder, with `/` or `\`; absolute paths stay.
    let resolved = back.resolved(&dir);
    assert_eq!(resolved[0].a, dir.join("scans").join("a").join("driftless"));
    let mut abs = back.clone();
    abs.entries[0].b = dir.join("woanders");
    abs.entries[1].a = PathBuf::from("./scans\\c//driftless");
    let resolved = abs.resolved(&dir);
    assert_eq!(resolved[0].b, dir.join("woanders"));
    assert_eq!(resolved[1].a, dir.join("scans").join("c").join("driftless"));

    let rejected = |e: PointPairEntry, what: &str| {
        let mut p = back.clone();
        match p.add(e) {
            Err(StepError::Failed(m)) => assert!(m.contains(what), "{m}"),
            other => panic!("expected a failure, got {other:?}"),
        }
        assert_eq!(p.entries.len(), 2, "nothing added");
    };
    rejected(
        pair_entry(a, b, in_a[..2].to_vec(), in_c[..2].to_vec()),
        "mindestens drei",
    );
    rejected(
        pair_entry(a, b, in_a.clone(), in_c[..2].to_vec()),
        "gleich viele",
    );
    let line = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    rejected(pair_entry(a, b, line.clone(), line), "Linie");
    rejected(
        pair_entry(a, a, in_a.clone(), in_c.clone()),
        "zwei verschiedene",
    );

    // A file that is no TOML, or a value of the wrong type, is an error, not an empty list.
    fs::write(&path, "[[paar]]\na = 1\n").unwrap();
    assert!(PointPairs::load(&path).is_err());
    fs::write(&path, "[[paar\n").unwrap();
    assert!(PointPairs::load(&path).is_err());
    // An entry that cannot be used (a hand edit) is read beside the good one and written back
    // as it was, unknown keys included; only switching it changes it.
    pairs.save(&path).unwrap();
    let mut text = fs::read_to_string(&path).unwrap();
    text += "\n[[paar]]\na = \"x\"\nb = \"y\"\npoints_a = [[0, 0, 0]]\npoints_b = []\n\
             enabeld = true\n";
    fs::write(&path, &text).unwrap();
    let mut loaded = PointPairs::load(&path).unwrap();
    assert_eq!(loaded.entries.len(), 3);
    assert_eq!(&loaded.entries[..2], &pairs.entries[..]);
    let odd = loaded.entries[2].clone();
    assert!(!odd.enabled, "a missing switch reads as off");
    assert!(odd.check().unwrap_err().contains("enabeld"));
    let mut no_typo = odd.clone();
    no_typo.unknown.clear();
    assert!(no_typo.check().unwrap_err().contains("gleich viele"));
    loaded.set_enabled(2, true).unwrap();
    loaded.save(&path).unwrap();
    let again = PointPairs::load(&path).unwrap();
    assert_eq!(again.entries[2].unknown, odd.unknown);
    assert!(again.entries[2].enabled);
    assert_eq!(again.entries[2].points_a, vec![[0.0; 3]]);
}

/// What the app writes into `<projekt>/modelle/punktpaare.toml`: a map folder of the project
/// relative to the folder of the file (`../scans/…`), one outside it absolute; reading the
/// file as `driftless merge --punktpaare` does (relative to the file) gives the same folders.
#[test]
fn stored_text_round_trips_with_file_resolution() {
    use driftless_steps::pairs::stored_text;
    let dir = tmp("punktpaare-pfade");
    let project = fs::canonicalize(&dir).unwrap().join("projekt");
    let models = project.join("modelle");
    let inside = project
        .join("scans")
        .join("raum-1")
        .join("slam-001")
        .join("karte-001");
    let below = models.join("eigene-karte");
    let outside = fs::canonicalize(&dir)
        .unwrap()
        .join("woanders")
        .join("karte");
    for d in [&models, &inside, &below, &outside] {
        fs::create_dir_all(d).unwrap();
    }
    assert_eq!(
        stored_text(&models, &inside).unwrap(),
        "../scans/raum-1/slam-001/karte-001"
    );
    assert_eq!(stored_text(&models, &below).unwrap(), "eigene-karte");
    let abs = stored_text(&models, &outside).unwrap();
    assert_eq!(Path::new(&abs), outside);

    let (in_c, in_a) = pairs_c_in_a();
    let mut pairs = PointPairs::default();
    for (a, b) in [(&inside, &outside), (&below, &inside)] {
        let e = pair_entry(
            Path::new(&stored_text(&models, a).unwrap()),
            Path::new(&stored_text(&models, b).unwrap()),
            in_a.clone(),
            in_c.clone(),
        );
        pairs.add(e).unwrap();
    }
    let file = models.join(POINT_PAIRS_FILE);
    pairs.save(&file).unwrap();
    let text = fs::read_to_string(&file).unwrap();
    assert!(
        text.contains("a = \"../scans/raum-1/slam-001/karte-001\""),
        "{text}"
    );
    let back = PointPairs::load(&file)
        .unwrap()
        .resolved(file.parent().unwrap());
    // `..` is taken off in the text: no part `..` is left for the file system to resolve
    // (below a verbatim Windows path it would be taken literally).
    assert_eq!(back[0].a, inside);
    assert!(back
        .iter()
        .flat_map(|e| [&e.a, &e.b])
        .all(|p| !p.components().any(|c| c == std::path::Component::ParentDir)));
    let canon = |p: &Path| fs::canonicalize(p).unwrap();
    assert_eq!(canon(&back[0].a), inside);
    assert_eq!(canon(&back[0].b), outside);
    assert_eq!(canon(&back[1].a), below);
    assert_eq!(canon(&back[1].b), inside);
}

/// A project on a Windows share (`\\localhost\C$\…`): the map folders of `punktpaare.toml`,
/// relative to `modelle/` with `..`, resolve below the share and match the members' folders
/// as resolving gives them (in their plain form, without `\\?\UNC\`). Skipped where the
/// share cannot be read (it needs an administrator, as on the CI runner).
#[cfg(windows)]
#[test]
fn pairs_match_members_on_a_share() {
    use driftless_io::paths::canonical;
    use driftless_steps::merge::pair_members;
    let local = canonical(&tmp("punktpaare-freigabe")).unwrap();
    let text = local.to_str().unwrap().to_string();
    let Some(rest) = text
        .get(3..)
        .filter(|_| text[..3].eq_ignore_ascii_case("C:\\"))
    else {
        eprintln!("übersprungen: {text} liegt nicht auf C:");
        return;
    };
    let share = PathBuf::from(format!(r"\\localhost\C$\{rest}"));
    if fs::metadata(&share).is_err() {
        eprintln!("übersprungen: {} nicht lesbar", share.display());
        return;
    }
    let project = share.join("projekt");
    let models = project.join("modelle");
    let maps: Vec<PathBuf> = ["a", "b"]
        .iter()
        .map(|s| {
            project
                .join("scans")
                .join(s)
                .join("slam-001")
                .join("karte-001")
        })
        .collect();
    for d in maps.iter().chain([&models]) {
        fs::create_dir_all(d).unwrap();
    }
    let members: Vec<MergeMember> = maps
        .iter()
        .map(|m| MergeMember {
            map_dir: canonical(m).unwrap(),
            ..member(m)
        })
        .collect();
    for m in &members {
        let shown = m.map_dir.to_string_lossy().into_owned();
        assert!(!shown.starts_with(r"\\?\"), "{shown}");
    }
    let req = request(members, &models.join("modell-001"));
    let (in_c, in_a) = pairs_c_in_a();
    let mut pairs = PointPairs::default();
    pairs
        .add(pair_entry(
            Path::new("../scans/a/slam-001/karte-001"),
            Path::new("../scans/b/slam-001/karte-001"),
            in_a,
            in_c,
        ))
        .unwrap();
    for base in [canonical(&models).unwrap(), models.clone()] {
        let resolved = pairs.resolved(&base);
        assert_eq!(pair_members(&req, &resolved[0]), Some((0, 1)), "{base:?}");
    }
}
