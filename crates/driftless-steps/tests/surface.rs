//! `run_surface` on the synthetic room scan (with camera images where noted), and
//! `run_model_surface` on models of two scans of the furnished room.

use driftless_io::calib::load_calibration;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image_where, Wanted};
use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use driftless_io::testdir::{SharedDir, TestDir};
use driftless_io::testscan::{
    furnished_room_bag, furnished_scan_with_images, room_bag, scan_with_images, shared, tmp, IMAGES,
};
use driftless_map::surface::{SurfaceConfig, TextureSource};
use driftless_map::{ColorCamera, ImageSource};
use driftless_slam::align::AlignConfig;
use driftless_steps::map::{map_input_for, run_map, MapInput, MapRequest, MapSettings, MapStep};
use driftless_steps::merge::{
    member_transform, read_model, run_merge, MergeMember, MergeRequest, MERGE_VERSION, MODEL_REPORT,
};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::surface::{
    model_surface_fits, run_model_surface, run_surface, surface_fits, texture_sources,
    ModelSurfaceRequest, NoSources, SurfaceRequest, TextureSources, SURFACE_REPORT,
};
use driftless_steps::StepError;
use driftless_traj::nalgebra::Isometry3;
use driftless_traj::{read_tum, Pose};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

fn slam(scan: &Path, out: &Path) {
    let req = SlamRequest {
        scan: scan.to_path_buf(),
        out: out.to_path_buf(),
        settings: SlamSettings::default(),
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
}

fn map_input(scan: &Path, dir: &Path) -> MapInput {
    MapInput {
        scan: scan.to_path_buf(),
        slam_dir: dir.to_path_buf(),
        slam: SlamSettings::default(),
        map: MapSettings {
            step: MapStep::Clean,
            ..MapSettings::default()
        },
    }
}

/// Coarse settings: the synthetic room has a point every 10 cm or so.
fn small_config() -> SurfaceConfig {
    SurfaceConfig {
        voxel: 0.15,
        min_weight: 1,
        min_component_faces: 10,
        page_size: 256,
        simplify_error: 0.01,
        ..SurfaceConfig::default()
    }
}

/// Scan, SLAM and clean map in one folder; returns (scan, folder).
fn prepared(name: &str, images: bool) -> (TestDir, PathBuf) {
    let dir = tmp(name);
    if images {
        scan_with_images(&dir);
    } else {
        room_bag(&dir, true);
    }
    let out = dir.join("out");
    slam(&dir, &out);
    let req = MapRequest {
        input: map_input(&dir, &out),
        out_dir: out.clone(),
        all_files: false,
    };
    run_map(&req, &Progress::new()).unwrap();
    (dir, out)
}

fn request(scan: &Path, map_dir: &Path, out_dir: &Path) -> SurfaceRequest {
    SurfaceRequest {
        map: map_input(scan, map_dir),
        map_dir: map_dir.to_path_buf(),
        out_dir: out_dir.to_path_buf(),
        config: small_config(),
        texture: true,
        gpu: driftless_gpu::GpuPref::default(),
        obj: false,
    }
}

fn warnings(p: &Progress) -> Vec<String> {
    p.take_log()
        .into_iter()
        .filter_map(|l| match l {
            LogLine::Warning(w) => Some(w),
            LogLine::Info(_) => None,
        })
        .collect()
}

fn names(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = fs::read_dir(dir)
        .map(|e| {
            e.map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
                .collect()
        })
        .unwrap_or_default();
    v.sort();
    v
}

#[test]
fn surface_end_to_end() {
    let (dir, out) = prepared("steps-surface-e2e", true);
    let dest = out.join("oberflaeche");
    let req = SurfaceRequest {
        obj: true,
        ..request(&dir, &out, &dest)
    };
    assert!(!surface_fits(&req));
    let report = run_surface(&req, &Progress::new()).unwrap();
    for f in [
        "oberflaeche.glb",
        "oberflaeche.obj",
        "oberflaeche.mtl",
        SURFACE_REPORT,
    ] {
        assert!(dest.join(f).is_file(), "{f} missing");
    }
    assert!(dest.join("oberflaeche_seite_00.jpg").is_file());
    assert!(
        report["stats"]["texture"]["faces_textured"]
            .as_u64()
            .unwrap()
            > 0,
        "{report}"
    );
    assert!(
        report["stats"]["texture"]["islands_absorbed"].is_u64(),
        "{report}"
    );
    assert!(
        report["stats"]["texture"]["gray_filled"].is_u64(),
        "{report}"
    );
    let share = report["stats"]["texture"]["untextured_area_share"]
        .as_f64()
        .unwrap();
    assert!((0.0..1.0).contains(&share), "{report}");
    let per_m = report["stats"]["seams"]["seam_step_per_m"]
        .as_f64()
        .unwrap();
    assert!(per_m >= 0.0, "{report}");
    assert_eq!(report["axes"], "glb: Y oben; obj: Z oben");
    assert_eq!(report["input_id"].as_str().unwrap().len(), 16);
    assert!(report["stats"]["seconds"]["tsdf"].is_number());
    assert_eq!(report["settings"]["coarse_fallback"], true);
    assert!(report["stats"]["cells_from_coarse"].is_u64(), "{report}");
    assert!(report["stats"]["faces_unsupported"].is_u64(), "{report}");
    assert_eq!(report["settings"]["max_fill_gap"], 0.015);
    assert!(report["files"].as_array().unwrap().len() >= 4);
    let on_disk: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(dest.join(SURFACE_REPORT)).unwrap()).unwrap();
    assert_eq!(on_disk, report);
    assert_eq!(report["programm"], program());
    // Hints for the app (German): one when many faces of the coarse grid were dropped.
    let s = &report["stats"];
    let dropped = s["faces_unsupported"].as_f64().unwrap();
    let open = dropped / (dropped + s["faces_final"].as_f64().unwrap());
    assert_eq!(
        report["hinweise"].as_array().unwrap().is_empty(),
        open < 0.15,
        "{report}"
    );
    assert!(surface_fits(&req));
}

/// `programm` of the reports: version and commit of this build.
fn program() -> Value {
    serde_json::json!({
        "version": env!("CARGO_PKG_VERSION"),
        "commit": driftless_steps::PROGRAM.commit,
    })
}

/// Without `obj` a surface is its glb and its report; an OBJ of an earlier run is listed as
/// stale, not deleted. The OBJ is part of no id.
#[test]
fn surface_writes_only_the_glb_by_default() {
    let (dir, out) = prepared("steps-surface-glb", true);
    let dest = out.join("oberflaeche");
    let req = request(&dir, &out, &dest);
    let report = run_surface(&req, &Progress::new()).unwrap();
    assert_eq!(names(&dest), ["oberflaeche.glb", SURFACE_REPORT]);
    assert_eq!(
        report["files"],
        serde_json::json!(["oberflaeche.glb", SURFACE_REPORT])
    );
    let with_obj = SurfaceRequest {
        obj: true,
        ..req.clone()
    };
    assert!(surface_fits(&with_obj), "the OBJ changes no id");
    run_surface(&with_obj, &Progress::new()).unwrap();
    assert!(dest.join("oberflaeche.obj").is_file() && dest.join("oberflaeche.mtl").is_file());
    let p = Progress::new();
    let again = run_surface(&req, &p).unwrap();
    let stale: Vec<&str> = again["stale"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    assert!(
        stale.contains(&"oberflaeche.obj")
            && stale.contains(&"oberflaeche.mtl")
            && stale.contains(&"oberflaeche_seite_00.jpg"),
        "{stale:?}"
    );
    assert!(dest.join("oberflaeche.obj").is_file(), "not deleted");
    assert!(warnings(&p).iter().any(|w| w.contains("früheren Lauf")));
}

#[test]
fn stale_map_is_refused() {
    let (dir, out) = prepared("steps-surface-stale", false);
    let traj = fs::read_to_string(out.join("traj.tum")).unwrap();
    let shifted: String = traj
        .lines()
        .map(|l| {
            if l.starts_with('#') {
                return format!("{l}\n");
            }
            let mut v: Vec<String> = l.split(' ').map(str::to_string).collect();
            v[1] = format!("{:.9}", v[1].parse::<f64>().unwrap() + 0.01);
            v.join(" ") + "\n"
        })
        .collect();
    fs::write(out.join("traj.tum"), shifted).unwrap();
    let dest = out.join("oberflaeche");
    let err = run_surface(&request(&dir, &out, &dest), &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("veraltet"), "{err}");
    assert!(!dest.exists(), "nothing written");

    // no clean.bin at all
    let (dir, out) = prepared("steps-surface-missing", false);
    fs::remove_file(out.join("clean.bin")).unwrap();
    let err = run_surface(&request(&dir, &out, &out.join("o")), &Progress::new()).unwrap_err();
    assert!(
        err.to_string().contains("Karte fehlt oder ist veraltet"),
        "{err}"
    );
}

#[test]
fn surface_without_dense_bin_still_succeeds() {
    let (dir, out) = prepared("steps-surface-dense", false);
    assert!(out.join("dense.bin").is_file(), "the map step writes it");
    let with = run_surface(&request(&dir, &out, &out.join("with")), &Progress::new()).unwrap();
    assert!(
        with["stats"]["accuracy_dense"]["walls"]["median_mm"].is_number(),
        "{with}"
    );
    fs::remove_file(out.join("dense.bin")).unwrap();
    let p = Progress::new();
    let without = run_surface(&request(&dir, &out, &out.join("without")), &p).unwrap();
    assert!(without["stats"]["accuracy_dense"].is_null(), "{without}");
    let infos: Vec<String> = p
        .take_log()
        .into_iter()
        .filter_map(|l| match l {
            LogLine::Info(i) => Some(i),
            LogLine::Warning(_) => None,
        })
        .collect();
    assert_eq!(
        infos.iter().filter(|i| i.contains("dense.bin")).count(),
        1,
        "{infos:?}"
    );
}

#[test]
fn surface_without_images_is_gray_with_warning() {
    let (dir, out) = prepared("steps-surface-gray", false);
    let dest = out.join("oberflaeche");
    let p = Progress::new();
    let report = run_surface(&request(&dir, &out, &dest), &p).unwrap();
    let w = warnings(&p);
    assert!(w.iter().any(|w| w.contains("keine Kamerabilder")), "{w:?}");
    let t = &report["stats"]["texture"];
    assert_eq!(t["faces_textured"], 0);
    assert!(t["faces_untextured"].as_u64().unwrap() > 0, "{report}");
    assert_eq!(t["untextured_area_share"], 1.0);
    assert_eq!(t["gray_filled"], 0);
    assert!(dest.join("oberflaeche.glb").is_file());
}

#[test]
fn no_texture_skips_the_images() {
    let (dir, out) = prepared("steps-surface-notex", true);
    let dest = out.join("oberflaeche");
    let mut req = request(&dir, &out, &dest);
    req.texture = false;
    let p = Progress::new();
    let report = run_surface(&req, &p).unwrap();
    assert!(warnings(&p).is_empty());
    assert_eq!(report["stats"]["texture"]["faces_textured"], 0);
    // another input id than with textures
    let with = request(&dir, &out, &dest);
    assert!(!surface_fits(&with));
}

#[test]
fn cancel_writes_nothing() {
    let (dir, out) = prepared("steps-surface-cancel", false);
    let dest = out.join("oberflaeche");
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Oberfläche: Genauigkeit" {
            handle.cancel();
        }
    });
    let before = names(&out);
    let err = run_surface(&request(&dir, &out, &dest), &p).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(!dest.exists());
    assert_eq!(names(&out), before);
}

/// A cancel once the files are being written comes too late: the step writes the whole
/// surface (glb, OBJ with its pages, report) rather than a new glb next to an old OBJ.
#[test]
fn cancel_while_writing_completes_surface() {
    let (dir, out) = prepared("steps-surface-cancel-writing", false);
    let dest = out.join("oberflaeche");
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == driftless_export::PHASE_WRITE {
            handle.cancel();
        }
    });
    let report = run_surface(&request(&dir, &out, &dest), &p).unwrap();
    for name in report["files"].as_array().unwrap() {
        let name = name.as_str().unwrap();
        assert!(dest.join(name).is_file(), "{name}");
    }
    assert!(!names(&dest).iter().any(|n| n.ends_with(".tmp")));
}

#[test]
fn surface_fits_after_run() {
    let (dir, out) = prepared("steps-surface-fits", false);
    let dest = out.join("oberflaeche");
    let req = request(&dir, &out, &dest);
    assert!(!surface_fits(&req));
    run_surface(&req, &Progress::new()).unwrap();
    assert!(surface_fits(&req));
    let mut other = request(&dir, &out, &dest);
    other.config.voxel = 0.2;
    assert!(!surface_fits(&other), "other settings");
    // `programm` is part of no id: a surface of another build, or one written before the
    // field, still fits.
    let path = dest.join(SURFACE_REPORT);
    let mut report: Value = serde_json::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
    report["programm"] = serde_json::json!({"version": "0.0.1", "commit": "anderer"});
    fs::write(&path, report.to_string()).unwrap();
    assert!(surface_fits(&req), "another build");
    report.as_object_mut().unwrap().remove("programm");
    fs::write(&path, report.to_string()).unwrap();
    assert!(surface_fits(&req), "written before the field");
    let traj = fs::read_to_string(out.join("traj.tum")).unwrap();
    fs::write(out.join("traj.tum"), format!("{traj}# changed\n")).unwrap();
    assert!(!surface_fits(&req), "other trajectory");
}

/// A killed run leaves `<name>.tmp` files; the next surface into the folder removes those of
/// its own files at its start, and only those.
#[test]
fn stale_tmp_files_of_the_surface_are_removed() {
    let (dir, out) = prepared("steps-surface-tmp", false);
    let dest = out.join("oberflaeche");
    fs::create_dir_all(&dest).unwrap();
    let own = [
        "oberflaeche.glb.tmp",
        "oberflaeche.obj.tmp",
        "oberflaeche.mtl.tmp",
        "oberflaeche_seite_00.jpg.tmp",
        "oberflaeche_seite_12.png.tmp",
        SURFACE_REPORT_TMP,
    ];
    let foreign = [
        "oberflaeche_seite_xy.jpg.tmp",
        "modell.las.tmp",
        "notiz.tmp",
    ];
    for f in own.iter().chain(&foreign) {
        fs::write(dest.join(f), b"halb").unwrap();
    }
    run_surface(&request(&dir, &out, &dest), &Progress::new()).unwrap();
    for f in own {
        assert!(!dest.join(f).exists(), "{f} left");
    }
    for f in foreign {
        assert!(dest.join(f).exists(), "{f} removed");
    }
}

const SURFACE_REPORT_TMP: &str = "oberflaeche.json.tmp";

/// What a kill of an earlier version could leave: the glb of a new run (other settings) next
/// to the OBJ and the report of the old one. The report names the sizes of its files, so the
/// folder does not fit the old request (nor the new one).
#[test]
fn new_glb_next_to_an_old_report_does_not_fit() {
    let (dir, out) = prepared("steps-surface-mixed", true);
    let dest = out.join("oberflaeche");
    let old = SurfaceRequest {
        obj: true,
        ..request(&dir, &out, &dest)
    };
    let report = run_surface(&old, &Progress::new()).unwrap();
    assert!(surface_fits(&old));
    let glb = fs::read(dest.join("oberflaeche.glb")).unwrap();
    assert_eq!(report["bytes"]["oberflaeche.glb"], glb.len() as u64);
    assert!(report["bytes"]["oberflaeche.obj"].as_u64().unwrap() > 0);
    let new = SurfaceRequest {
        config: SurfaceConfig {
            voxel: 0.3,
            ..small_config()
        },
        obj: false,
        ..request(&dir, &out, &out.join("neu"))
    };
    run_surface(&new, &Progress::new()).unwrap();
    fs::copy(
        out.join("neu").join("oberflaeche.glb"),
        dest.join("oberflaeche.glb"),
    )
    .unwrap();
    assert!(!surface_fits(&old), "new glb, old OBJ and report");
    assert!(!surface_fits(&SurfaceRequest {
        out_dir: dest.clone(),
        ..new.clone()
    }));
    fs::write(dest.join("oberflaeche.glb"), &glb).unwrap();
    assert!(surface_fits(&old));
    fs::remove_file(dest.join("oberflaeche_seite_00.jpg")).unwrap();
    assert!(!surface_fits(&old), "a page of the report is missing");
}

/// `oberflaeche.json` goes before the first file of a surface is written and comes back
/// last: a surface that stops in between (here: the OBJ cannot be replaced, after the new
/// glb was written) leaves no report and does not fit.
#[test]
fn report_goes_before_the_first_write() {
    let (dir, out) = prepared("steps-surface-report-first", false);
    let dest = out.join("oberflaeche");
    let req = request(&dir, &out, &dest);
    run_surface(&req, &Progress::new()).unwrap();
    assert!(surface_fits(&req));
    fs::create_dir_all(dest.join("oberflaeche.obj").join("x")).unwrap();
    let with_obj = SurfaceRequest {
        obj: true,
        ..req.clone()
    };
    let err = run_surface(&with_obj, &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("oberflaeche.obj"), "{err}");
    assert!(!dest.join(SURFACE_REPORT).exists(), "the old report stayed");
    assert!(dest.join("oberflaeche.glb").is_file());
    assert!(!surface_fits(&req) && !surface_fits(&with_obj));
    assert!(!names(&dest).iter().any(|n| n.ends_with(".tmp")));
}

#[test]
fn old_pages_are_listed_as_stale_not_deleted() {
    let (dir, out) = prepared("steps-surface-stale-pages", false);
    let dest = out.join("oberflaeche");
    let req = request(&dir, &out, &dest);
    let first = run_surface(&req, &Progress::new()).unwrap();
    assert_eq!(first["stale"], serde_json::json!([]));
    fs::write(dest.join("oberflaeche_seite_07.jpg"), b"old").unwrap();
    let p = Progress::new();
    let report = run_surface(&req, &p).unwrap();
    assert_eq!(
        report["stale"],
        serde_json::json!(["oberflaeche_seite_07.jpg"])
    );
    assert!(
        dest.join("oberflaeche_seite_07.jpg").is_file(),
        "not deleted"
    );
    assert!(warnings(&p).iter().any(|w| w.contains("früheren Lauf")));
    assert!(!dest.join("oberflaeche.json.tmp").exists());
}

// Surfaces of models: scan A of the furnished room from the origin, scan B from
// (0.8, 0.5, 0) turned by 30°, both with camera images; scan C with the LiDAR data of B,
// without images.

struct ModelScene {
    /// Their folder, shared by the tests of this file.
    _root: SharedDir,
    a: PathBuf,
    /// Model of A (reference) and B.
    both: PathBuf,
    /// Model of A (reference) and C: the points of `both`, the images of A only.
    images_of_a: PathBuf,
    /// Model of C (reference, no images) and A.
    one_imageless: PathBuf,
}

/// SLAM and clean map of `scan` in `<scan>/driftless`.
fn prepare_member(scan: &Path) {
    let out = scan.join("driftless");
    slam(scan, &out);
    let mut input = map_input_for(scan, &out, &out, SlamSettings::default());
    input.map.step = MapStep::Clean;
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

fn merge_member(scan: &Path) -> MergeMember {
    let run = scan.join("driftless");
    MergeMember {
        scan: scan.to_path_buf(),
        slam_dir: run.clone(),
        map_dir: run,
        slam: SlamSettings::default(),
    }
}

fn merged(scans: &[&Path], out: &Path) -> PathBuf {
    let req = MergeRequest {
        members: scans.iter().map(|s| merge_member(s)).collect(),
        out_dir: out.to_path_buf(),
        align: AlignConfig::default(),
        voxel: 0.01,
        smooth: true,
        include_unsure: false,
        pairs: Vec::new(),
        ply: false,
    };
    let report = run_merge(&req, &Progress::new()).unwrap();
    assert!(
        report.members.iter().all(|m| m.included),
        "{}",
        serde_json::to_string_pretty(&report.members).unwrap()
    );
    out.to_path_buf()
}

fn model_scene() -> &'static ModelScene {
    static SCENE: OnceLock<ModelScene> = OnceLock::new();
    SCENE.get_or_init(|| {
        let root = shared("steps-model-surface");
        let (a, b, c) = (
            root.join("raum-a"),
            root.join("raum-b"),
            root.join("raum-c"),
        );
        for d in [&a, &b, &c] {
            fs::create_dir_all(d).unwrap();
        }
        let yaw = 30f64.to_radians();
        furnished_scan_with_images(&a, [0.0; 3], 0.0);
        furnished_scan_with_images(&b, [0.8, 0.5, 0.0], yaw);
        furnished_room_bag(&c, true, [0.8, 0.5, 0.0], yaw);
        for d in [&a, &b, &c] {
            prepare_member(d);
        }
        let both = merged(&[&a, &b], &root.join("modell-001"));
        let images_of_a = merged(&[&a, &c], &root.join("modell-002"));
        let one_imageless = merged(&[&c, &a], &root.join("modell-003"));
        ModelScene {
            _root: root,
            a,
            both,
            images_of_a,
            one_imageless,
        }
    })
}

fn model_request(model: &Path, out: &Path) -> ModelSurfaceRequest {
    ModelSurfaceRequest {
        model_dir: model.to_path_buf(),
        model: None,
        out_dir: out.to_path_buf(),
        config: small_config(),
        texture: true,
        gpu: driftless_gpu::GpuPref::default(),
        obj: false,
    }
}

/// Fresh output folder next to the models.
fn model_out(name: &str) -> PathBuf {
    let dir = model_scene().a.parent().unwrap().join("out").join(name);
    let _ = fs::remove_dir_all(&dir);
    dir
}

fn untextured(report: &Value) -> u64 {
    report["stats"]["texture"]["faces_untextured"]
        .as_u64()
        .unwrap()
}

#[test]
fn model_surface_textures_both_scans() {
    let s = model_scene();
    // The same points with the images of one scan.
    let single = run_model_surface(
        &model_request(&s.images_of_a, &model_out("bilder-von-a")),
        &Progress::new(),
    )
    .unwrap();

    let dest = s.both.join("oberflaeche");
    let _ = fs::remove_dir_all(&dest);
    let p = Progress::new();
    let report = run_model_surface(&model_request(&s.both, &dest), &p).unwrap();
    assert!(warnings(&p).is_empty(), "{:?}", warnings(&p));
    for f in ["oberflaeche.glb", SURFACE_REPORT] {
        assert!(dest.join(f).is_file(), "{f} missing");
    }
    assert!(
        !dest.join("oberflaeche.obj").exists(),
        "the OBJ only on request"
    );
    assert_eq!(
        report["model"]["members"],
        serde_json::json!(["raum-a", "raum-b"])
    );
    assert_eq!(report["model"]["dir"], s.both.display().to_string());
    assert!(report["stats"]["accuracy_dense"].is_null(), "{report}");
    assert!(
        report["stats"]["accuracy"]["all"]["vertices"]
            .as_u64()
            .unwrap()
            > 0
    );
    assert_eq!(
        report["stats"]["faces_final"], single["stats"]["faces_final"],
        "the same points give the same mesh"
    );
    assert!(
        untextured(&report) < untextured(&single),
        "both scans {} vs. images of one {}",
        report["stats"]["texture"],
        single["stats"]["texture"]
    );
    let on_disk: Value =
        serde_json::from_str(&fs::read_to_string(dest.join(SURFACE_REPORT)).unwrap()).unwrap();
    assert_eq!(on_disk, report);
    assert_eq!(report["programm"], program());
}

#[test]
fn model_surface_with_one_imageless_member() {
    let s = model_scene();
    let dest = model_out("ohne-bilder");
    let p = Progress::new();
    let report = run_model_surface(&model_request(&s.one_imageless, &dest), &p).unwrap();
    let w = warnings(&p);
    assert!(
        w.iter()
            .any(|w| w.contains("raum-c") && w.contains("keine Kamerabilder")),
        "{w:?}"
    );
    assert_eq!(
        report["model"]["members"],
        serde_json::json!(["raum-c", "raum-a"])
    );
    let t = &report["stats"]["texture"];
    assert!(t["faces_textured"].as_u64().unwrap() > 0, "{report}");
    assert!(dest.join("oberflaeche.glb").is_file());
}

#[test]
fn model_surface_fits_after_run() {
    let s = model_scene();
    let dest = model_out("passt");
    let req = model_request(&s.both, &dest);
    assert!(!model_surface_fits(&req));
    run_model_surface(&req, &Progress::new()).unwrap();
    assert!(model_surface_fits(&req));
    let mut other = model_request(&s.both, &dest);
    other.config.voxel = 0.2;
    assert!(!model_surface_fits(&other), "other settings");
    let mut gray = model_request(&s.both, &dest);
    gray.texture = false;
    assert!(!model_surface_fits(&gray), "without texture");
    // A glb other than the one of the report (a kill between the files of two runs).
    let glb = dest.join("oberflaeche.glb");
    let bytes = fs::read(&glb).unwrap();
    fs::write(&glb, &bytes[..bytes.len() / 2]).unwrap();
    assert!(!model_surface_fits(&req), "glb of another run");
    fs::write(&glb, &bytes).unwrap();
    assert!(model_surface_fits(&req));
}

/// A copy of `modell.json` of the model of A and B in a new folder, changed by `edit`.
fn edited_model(name: &str, edit: impl FnOnce(&mut Value)) -> PathBuf {
    let dir = model_out(name);
    fs::create_dir_all(&dir).unwrap();
    let text = fs::read_to_string(model_scene().both.join(MODEL_REPORT)).unwrap();
    let mut model: Value = serde_json::from_str(&text).unwrap();
    edit(&mut model);
    fs::write(dir.join(MODEL_REPORT), model.to_string()).unwrap();
    dir
}

#[test]
fn model_surface_takes_only_included_members() {
    let model = edited_model("nur-a", |m| m["members"][1]["included"] = false.into());
    let dest = model.join("oberflaeche");
    let mut req = model_request(&model, &dest);
    req.texture = false;
    let report = run_model_surface(&req, &Progress::new()).unwrap();
    assert_eq!(report["model"]["members"], serde_json::json!(["raum-a"]));
}

#[test]
fn model_surface_refuses_a_changed_map_or_no_model() {
    let model = edited_model("veraltet", |m| {
        m["members"][1]["clean_id"] = "0000000000000000".into();
    });
    let dest = model.join("oberflaeche");
    let req = model_request(&model, &dest);
    let err = run_model_surface(&req, &Progress::new()).unwrap_err();
    assert!(
        err.to_string().contains("raum-b") && err.to_string().contains("neu zusammenführen"),
        "{err}"
    );
    assert!(!dest.exists(), "nothing written");
    assert!(!model_surface_fits(&req));

    let empty = model_out("kein-modell");
    let err =
        run_model_surface(&model_request(&empty, &empty.join("o")), &Progress::new()).unwrap_err();
    assert!(err.to_string().contains("modell.json"), "{err}");
}

/// A model merged by another version of the merge is stale like one whose map was computed
/// anew: `surface --model` refuses it, and a surface made while it was current no longer fits.
#[test]
fn model_surface_refuses_another_merge_version() {
    let model = edited_model("version", |_| {});
    let dest = model.join("oberflaeche");
    let mut req = model_request(&model, &dest);
    req.texture = false;
    run_model_surface(&req, &Progress::new()).unwrap();
    assert!(model_surface_fits(&req));
    for (version, which) in [
        (MERGE_VERSION - 1, "älteren"),
        (MERGE_VERSION + 1, "neueren"),
    ] {
        let file = model.join(MODEL_REPORT);
        let mut json: Value = serde_json::from_str(&fs::read_to_string(&file).unwrap()).unwrap();
        json["version"] = version.into();
        fs::write(&file, json.to_string()).unwrap();
        assert!(!model_surface_fits(&req), "v{version}");
        let other = model.join(format!("oberflaeche-v{version}"));
        let mut again = model_request(&model, &other);
        again.texture = false;
        let err = run_model_surface(&again, &Progress::new()).unwrap_err();
        let text = err.to_string();
        assert!(
            text.contains(&format!("{which} Programmversion"))
                && text.contains(&format!("v{version}"))
                && text.contains("neu zusammenführen"),
            "{text}"
        );
        assert!(!other.exists(), "nothing written");
    }
}

#[test]
fn model_surface_cancel_writes_nothing() {
    let s = model_scene();
    let dest = model_out("abbruch");
    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.phase == "Oberfläche: Genauigkeit" {
            handle.cancel();
        }
    });
    let err = run_model_surface(&model_request(&s.both, &dest), &p).unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert!(!dest.exists());
}

// The camera images of a surface for the touch-up („Bilder neu einspielen“): the same
// sources as the surface step, found from the surface folder alone.

/// The images of `src` as the stream has them: (camera, stamp), without decoding any.
fn image_list(src: &TextureSource) -> Vec<(String, f64)> {
    let mut list = Vec::new();
    for_each_image_where(
        &src.images.bags,
        &src.images.topics,
        1,
        |h| {
            list.push((h.camera.clone(), h.stamp));
            Wanted::No
        },
        |_| true,
    )
    .unwrap();
    list
}

/// `path` resolved, for comparing paths given another way.
fn resolved(path: &Path) -> PathBuf {
    fs::canonicalize(path).unwrap()
}

/// The sources of the surface in `surface`; panics without.
fn sources_of(surface: &Path) -> Vec<TextureSource> {
    match texture_sources(surface).unwrap() {
        TextureSources::Found(sources) => sources,
        other => panic!("keine Rohdaten für {}: {other:?}", surface.display()),
    }
}

/// `t · pose`, as the model surface moves the poses of a scan.
fn moved(t: &Isometry3<f64>, p: &Pose) -> Pose {
    Pose {
        t: p.t,
        p: t.rotation * p.p + t.translation.vector,
        q: t.rotation * p.q,
    }
}

/// The source the surface step makes of `scan` (with the trajectory `poses`): its image bag
/// and topic, its camera from the calibration (LiDAR mount: the default settings' identity).
fn expected_source(scan: &Path, poses: Vec<Pose>) -> TextureSource {
    let calib = load_calibration(&scan.join("calibration/calib.json")).unwrap();
    let c = &calib.cameras["left"];
    let camera = Camera::from_calib("left", c).unwrap();
    TextureSource {
        images: ImageSource {
            bags: vec![resolved(&scan.join("IMAGE_0.bag"))],
            topics: vec!["/camera_left/image_raw".to_string()],
        },
        poses,
        cams: vec![ColorCamera::new(
            camera,
            &c.lidar_to_cam.unwrap(),
            &Isometry3::identity(),
        )],
    }
}

/// `src` with its bags resolved (the folders of a project are found resolved or not).
fn with_resolved_bags(mut src: TextureSource) -> TextureSource {
    src.images.bags = src.images.bags.iter().map(|b| resolved(b)).collect();
    src
}

/// A scan with images in a project as the program lays it out: the scan in
/// `<projekt>/rohdaten/raum`, its run `scans/raum/slam-001` with `lauf.toml` (the scan relative
/// to the project), the map `karte-001` and the surface `oberflaeche-001` in it, and
/// `project.toml`. Returns (temporary folder, project, surface folder).
fn project_with_surface(name: &str) -> (TestDir, PathBuf, PathBuf) {
    let dir = tmp(name);
    let project = dir.join("projekt");
    let scan = project.join("rohdaten").join("raum");
    fs::create_dir_all(&scan).unwrap();
    scan_with_images(&scan);
    fs::write(
        project.join("project.toml"),
        "format = 1\nname = \"P\"\n\n[[scan]]\nid = \"raum\"\npath = \"rohdaten/raum\"\n\
         name = \"Raum\"\n",
    )
    .unwrap();
    let run = project.join("scans").join("raum").join("slam-001");
    slam(&scan, &run);
    fs::write(
        run.join("lauf.toml"),
        "scan = \"rohdaten/raum\"\nerstellt = \"2026-10-29T08:00:00Z\"\n\n[slam]\n",
    )
    .unwrap();
    let map_dir = run.join("karte-001");
    let input = MapInput {
        scan: scan.clone(),
        slam_dir: run.clone(),
        slam: SlamSettings::default(),
        map: MapSettings {
            step: MapStep::Clean,
            ..MapSettings::default()
        },
    };
    run_map(
        &MapRequest {
            input: input.clone(),
            out_dir: map_dir.clone(),
            all_files: false,
        },
        &Progress::new(),
    )
    .unwrap();
    let surface = map_dir.join("oberflaeche-001");
    let report = run_surface(
        &SurfaceRequest {
            map: input,
            map_dir,
            out_dir: surface.clone(),
            config: small_config(),
            texture: true,
            gpu: driftless_gpu::GpuPref::Off,
            obj: false,
        },
        &Progress::new(),
    )
    .unwrap();
    assert!(
        report["stats"]["texture"]["faces_textured"]
            .as_u64()
            .unwrap()
            > 0,
        "the surface step used the images: {report}"
    );
    (dir, project, surface)
}

#[test]
fn texture_sources_match_surface_step() {
    let (dir, project, surface) = project_with_surface("steps-texture-sources");
    let scan = project.join("rohdaten").join("raum");
    let run = project.join("scans").join("raum").join("slam-001");
    let poses = read_tum(&run.join("traj.tum")).unwrap();
    let sources = sources_of(&surface);
    assert_eq!(sources.len(), 1);
    let src = with_resolved_bags(sources[0].clone());
    assert_eq!(src, expected_source(&scan, poses.clone()));
    // Every image of the scan, of its one camera.
    let images = image_list(&src);
    assert_eq!(images.len(), IMAGES, "{images:?}");
    assert!(images.iter().all(|(c, _)| c == "left"), "{images:?}");

    // The project moved with its raw data: found in the new place.
    let moved_to = dir.join("verschoben");
    fs::rename(&project, &moved_to).unwrap();
    let surface = moved_to.join(surface.strip_prefix(&project).unwrap());
    let scan = moved_to.join("rohdaten").join("raum");
    let src = with_resolved_bags(sources_of(&surface).remove(0));
    assert_eq!(src, expected_source(&scan, poses.clone()));

    // The raw data gone (a drive unplugged, the folder renamed): none, no error.
    let elsewhere = moved_to.join("rohdaten").join("raum-woanders");
    fs::rename(&scan, &elsewhere).unwrap();
    assert!(matches!(
        texture_sources(&surface).unwrap(),
        TextureSources::Unavailable(NoSources::ScanFolder { .. })
    ));

    // The scan given its new folder in the project (`project.toml`, as „Ordner neu zuordnen“
    // writes it; `lauf.toml` keeps the old one): found there.
    let file = moved_to.join("project.toml");
    let text = fs::read_to_string(&file).unwrap();
    fs::write(
        &file,
        text.replace("rohdaten/raum", "rohdaten/raum-woanders"),
    )
    .unwrap();
    let src = with_resolved_bags(sources_of(&surface).remove(0));
    assert_eq!(src, expected_source(&elsewhere, poses));

    // A surface computed without images has none either.
    let gray = surface.with_file_name("oberflaeche-002");
    fs::create_dir_all(&gray).unwrap();
    let mut report: Value =
        serde_json::from_str(&fs::read_to_string(surface.join(SURFACE_REPORT)).unwrap()).unwrap();
    report["settings"]["texture"] = false.into();
    fs::write(gray.join(SURFACE_REPORT), report.to_string()).unwrap();
    assert_eq!(
        texture_sources(&gray).unwrap(),
        TextureSources::Found(Vec::new())
    );

    // No surface in the folder: an error that says so.
    let err = texture_sources(&moved_to.join("scans")).unwrap_err();
    assert!(err.to_string().contains(SURFACE_REPORT), "{err}");
}

#[test]
fn texture_sources_of_a_model() {
    let s = model_scene();
    // A surface of the model of A and B outside its folder: the report leads to the model.
    let dest = model_out("quellen");
    run_model_surface(&model_request(&s.both, &dest), &Progress::new()).unwrap();
    let model = read_model(&s.both).unwrap();
    let sources = sources_of(&dest);
    assert_eq!(sources.len(), 2, "one per scan with images");
    for (src, m) in sources.into_iter().zip(&model.members) {
        let own = read_tum(&m.slam_dir.join("traj.tum")).unwrap();
        let t = member_transform(m);
        let poses = own.iter().map(|p| moved(&t, p)).collect();
        assert_eq!(
            with_resolved_bags(src.clone()),
            expected_source(&m.scan, poses)
        );
        assert_eq!(image_list(&src).len(), IMAGES);
    }

    // C has no images: only A's.
    let dest = model_out("quellen-ohne-bilder");
    run_model_surface(&model_request(&s.one_imageless, &dest), &Progress::new()).unwrap();
    let model = read_model(&s.one_imageless).unwrap();
    let a = &model.members[1];
    let poses = read_tum(&a.slam_dir.join("traj.tum"))
        .unwrap()
        .iter()
        .map(|p| moved(&member_transform(a), p))
        .collect();
    let sources = sources_of(&dest);
    assert_eq!(sources.len(), 1);
    assert_eq!(
        with_resolved_bags(sources[0].clone()),
        expected_source(&a.scan, poses)
    );

    // The raw data of a scan in the model gone: none for the model.
    let missing = edited_model("quellen-fehlen", |m| {
        m["members"][1]["scan"] = "/gibt/es/nicht".into();
        m["members"][1]["scan_rel"] = Value::Null;
    });
    let surface = missing.join("oberflaeche");
    fs::create_dir_all(&surface).unwrap();
    fs::copy(dest.join(SURFACE_REPORT), surface.join(SURFACE_REPORT)).unwrap();
    let mut report: Value =
        serde_json::from_str(&fs::read_to_string(surface.join(SURFACE_REPORT)).unwrap()).unwrap();
    report["model"]["dir"] = missing.display().to_string().into();
    report["model"]["dir_rel"] = "..".into();
    fs::write(surface.join(SURFACE_REPORT), report.to_string()).unwrap();
    assert!(matches!(
        texture_sources(&surface).unwrap(),
        TextureSources::Unavailable(NoSources::ScanFolder { .. })
    ));
}

/// Moves every position of the trajectory `traj` by `dx` m along x, as a corrected trajectory
/// computed anew in the same folder does („Schleife eintragen → Jetzt neu rechnen“); returns
/// the text it had.
fn shift_traj(traj: &Path, dx: f64) -> String {
    let text = fs::read_to_string(traj).unwrap();
    let shifted: String = text
        .lines()
        .map(|l| {
            if l.starts_with('#') || l.trim().is_empty() {
                return format!("{l}\n");
            }
            let mut f: Vec<String> = l.split_whitespace().map(String::from).collect();
            let x: f64 = f[1].parse().unwrap();
            f[1] = format!("{}", x + dx);
            format!("{}\n", f.join(" "))
        })
        .collect();
    fs::write(traj, shifted).unwrap();
    text
}

/// What is not there of the raw data of a map's surface is said by name: the calibration, the
/// image bag, the scan folder, the scan of a run of the command line; raw data that are there
/// but not those of the surface (the trajectory computed anew in its folder) make it stale.
/// Each back as it was: found again.
#[test]
fn texture_sources_say_what_is_missing() {
    let (dir, project, surface) = project_with_surface("steps-quellen-fehlen");
    let scan = fs::canonicalize(project.join("rohdaten").join("raum")).unwrap();
    let missing = |surface: &Path, what: &str| match texture_sources(surface).unwrap() {
        TextureSources::Unavailable(why) => {
            assert!(why.raw_data_missing(), "{what}: {why:?}");
            why
        }
        found => panic!("{what}: {found:?}"),
    };

    let calib = scan.join("calibration").join("calib.json");
    fs::rename(&calib, scan.join("calib.weg")).unwrap();
    let why = missing(&surface, "calibration");
    assert_eq!(
        why.to_string(),
        format!(
            "Rohdaten fehlen – keine Kalibrierung: {} fehlt",
            calib.display()
        )
    );
    assert!(matches!(&why, NoSources::Images { scan: s, .. } if *s == scan));
    fs::rename(scan.join("calib.weg"), &calib).unwrap();
    assert_eq!(sources_of(&surface).len(), 1);

    let bag = scan.join("IMAGE_0.bag");
    fs::rename(&bag, scan.join("IMAGE_0.weg")).unwrap();
    let why = missing(&surface, "image bag");
    assert_eq!(
        why.to_string(),
        format!(
            "Rohdaten fehlen – keine Kamerabilder: {} enthält keine IMAGE_*.bag",
            scan.display()
        )
    );
    fs::rename(scan.join("IMAGE_0.weg"), &bag).unwrap();
    assert_eq!(sources_of(&surface).len(), 1);

    let away = scan.with_file_name("raum-weg");
    fs::rename(&scan, &away).unwrap();
    let why = missing(&surface, "scan folder");
    assert_eq!(
        why,
        NoSources::ScanFolder {
            folder: scan.clone(),
            why: format!("Scan-Ordner {} nicht gefunden", scan.display()),
        }
    );
    assert_eq!(
        why.to_string(),
        format!(
            "Rohdaten fehlen – Scan-Ordner {} nicht gefunden",
            scan.display()
        )
    );
    fs::rename(&away, &scan).unwrap();
    assert_eq!(sources_of(&surface).len(), 1);

    // the trajectory computed anew in its folder: the surface does not fit it any more
    let run = project.join("scans").join("raum").join("slam-001");
    let text = shift_traj(&run.join("traj.tum"), 0.3);
    let stale = texture_sources(&surface).unwrap();
    assert_eq!(stale, TextureSources::Unavailable(NoSources::SurfaceStale));
    assert!(!NoSources::SurfaceStale.raw_data_missing());
    assert_eq!(
        NoSources::SurfaceStale.to_string(),
        "Oberfläche veraltet – Bilder nur auf einer aktuellen Oberfläche neu einspielbar"
    );
    fs::write(run.join("traj.tum"), text).unwrap();
    assert_eq!(sources_of(&surface).len(), 1);

    // a run of the command line added to the project from elsewhere: it names no scan
    let cli = dir.join("cli").join("lauf");
    fs::create_dir_all(cli.parent().unwrap()).unwrap();
    fs::rename(&run, &cli).unwrap();
    fs::remove_file(cli.join("lauf.toml")).unwrap();
    let surface = cli.join("karte-001").join("oberflaeche-001");
    let cli = fs::canonicalize(&cli).unwrap();
    let why = missing(&surface, "run of the command line");
    assert_eq!(why, NoSources::CommandLineRun(cli.clone()));
    assert_eq!(
        why.to_string(),
        format!(
            "Rohdaten nicht gefunden – Lauf der Kommandozeile: zum Lauf {} ist kein Scan-Ordner \
             bekannt",
            cli.display()
        )
    );
}

/// The same for a model's surface: a scan of it short of its image bag makes the whole model
/// short (never a part of the sources), a scan or run folder not found is named, an old model
/// of a moved project says so, and a scan whose trajectory was computed anew makes it stale.
#[test]
fn texture_sources_of_a_model_say_what_is_missing() {
    let s = model_scene();
    // a copy of A, B and their model for this test alone, the model's absolute folders
    // pointing nowhere: only its relative ones lead to the copy
    let root = fs::canonicalize(s.a.parent().unwrap()).unwrap();
    let dir = tmp("steps-quellen-modell-fehlt");
    for name in ["raum-a", "raum-b"] {
        copy_dir(&root.join(name), &dir.join(name));
    }
    let model = dir.join("modell");
    fs::create_dir_all(&model).unwrap();
    let text = fs::read_to_string(s.both.join(MODEL_REPORT)).unwrap();
    let mut report: Value = serde_json::from_str(&text).unwrap();
    for m in report["members"].as_array_mut().unwrap() {
        for key in ["scan", "slam_dir", "map_dir"] {
            let name = Path::new(m[key].as_str().unwrap())
                .strip_prefix(&root)
                .unwrap()
                .to_path_buf();
            m[key] = Path::new("/frueher")
                .join(name)
                .display()
                .to_string()
                .into();
        }
    }
    fs::write(model.join(MODEL_REPORT), report.to_string()).unwrap();
    let surface = model.join("oberflaeche-001");
    run_model_surface(&model_request(&model, &surface), &Progress::new()).unwrap();
    assert_eq!(sources_of(&surface).len(), 2);
    let (a, b) = (
        fs::canonicalize(dir.join("raum-a")).unwrap(),
        fs::canonicalize(dir.join("raum-b")).unwrap(),
    );

    // B without its images: none at all, B named
    fs::rename(b.join("IMAGE_0.bag"), b.join("IMAGE_0.weg")).unwrap();
    let why = texture_sources(&surface).unwrap();
    assert_eq!(
        why,
        TextureSources::Unavailable(NoSources::Images {
            scan: b.clone(),
            why: format!(
                "keine Kamerabilder: {} enthält keine IMAGE_*.bag",
                b.display()
            ),
        })
    );
    fs::rename(b.join("IMAGE_0.weg"), b.join("IMAGE_0.bag")).unwrap();
    assert_eq!(sources_of(&surface).len(), 2);

    // the run folder of B gone
    let run = b.join("driftless");
    fs::rename(&run, b.join("driftless-weg")).unwrap();
    assert_eq!(
        texture_sources(&surface).unwrap(),
        TextureSources::Unavailable(NoSources::RunFolder(PathBuf::from(
            "/frueher/raum-b/driftless"
        )))
    );
    fs::rename(b.join("driftless-weg"), &run).unwrap();

    // the scan folder of B gone (with its run in it)
    fs::rename(&b, b.with_file_name("raum-b-weg")).unwrap();
    let why = texture_sources(&surface).unwrap();
    assert_eq!(
        why,
        TextureSources::Unavailable(NoSources::ScanFolder {
            folder: PathBuf::from("/frueher/raum-b"),
            why: "Scan-Ordner /frueher/raum-b nicht gefunden".into(),
        })
    );
    fs::rename(b.with_file_name("raum-b-weg"), &b).unwrap();
    assert_eq!(sources_of(&surface).len(), 2);

    // the trajectory of A computed anew in its folder: the map of A no longer fits the model
    let traj = a.join("driftless").join("traj.tum");
    let text = shift_traj(&traj, 0.3);
    assert_eq!(
        texture_sources(&surface).unwrap(),
        TextureSources::Unavailable(NoSources::SurfaceStale)
    );
    fs::write(&traj, text).unwrap();
    assert_eq!(sources_of(&surface).len(), 2);

    // an old model, of the folders of its scans only the absolute ones, at the old place of
    // the project – its scans elsewhere, still there
    for m in report["members"].as_array_mut().unwrap() {
        for key in ["scan_rel", "slam_dir_rel", "map_dir_rel"] {
            m.as_object_mut().unwrap().remove(key);
        }
        let name = Path::new(m["scan"].as_str().unwrap()).file_name().unwrap();
        m["scan"] = dir.join(name).display().to_string().into();
    }
    fs::write(model.join(MODEL_REPORT), report.to_string()).unwrap();
    let why = texture_sources(&surface).unwrap();
    let model = fs::canonicalize(&model).unwrap();
    assert_eq!(
        why,
        TextureSources::Unavailable(NoSources::OldModel(model.clone()))
    );
    assert_eq!(
        why.unavailable().unwrap().to_string(),
        format!(
            "Rohdaten nicht gefunden – altes Modell: {} kennt die Ordner seiner Scans nur am \
             früheren Ort des Projekts",
            model.display()
        )
    );
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
