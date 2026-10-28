//! `run_surface` on the synthetic room scan (with camera images where noted), and
//! `run_model_surface` on models of two scans of the furnished room.

use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use driftless_io::testscan::{
    furnished_room_bag, furnished_scan_with_images, room_bag, scan_with_images, tmp,
};
use driftless_map::surface::SurfaceConfig;
use driftless_slam::align::AlignConfig;
use driftless_steps::map::{map_input_for, run_map, MapInput, MapRequest, MapSettings, MapStep};
use driftless_steps::merge::{run_merge, MergeMember, MergeRequest, MODEL_REPORT};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::surface::{
    model_surface_fits, run_model_surface, run_surface, surface_fits, ModelSurfaceRequest,
    SurfaceRequest, SURFACE_REPORT,
};
use driftless_steps::StepError;
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
fn prepared(name: &str, images: bool) -> (PathBuf, PathBuf) {
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
    let req = request(&dir, &out, &dest);
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
    assert!(surface_fits(&req));
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
    let traj = fs::read_to_string(out.join("traj.tum")).unwrap();
    fs::write(out.join("traj.tum"), format!("{traj}# changed\n")).unwrap();
    assert!(!surface_fits(&req), "other trajectory");
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
        let root = tmp("steps-model-surface");
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
    for f in ["oberflaeche.glb", "oberflaeche.obj", SURFACE_REPORT] {
        assert!(dest.join(f).is_file(), "{f} missing");
    }
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
