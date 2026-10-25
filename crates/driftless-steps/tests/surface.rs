//! `run_surface` on the synthetic room scan (with camera images where noted).

use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag, scan_with_images, tmp};
use driftless_map::surface::SurfaceConfig;
use driftless_steps::map::{run_map, MapInput, MapRequest, MapSettings, MapStep};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::surface::{run_surface, surface_fits, SurfaceRequest, SURFACE_REPORT};
use driftless_steps::StepError;
use std::fs;
use std::path::{Path, PathBuf};

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
    assert_eq!(report["axes"], "glb: Y oben; obj: Z oben");
    assert_eq!(report["input_id"].as_str().unwrap().len(), 16);
    assert!(report["stats"]["seconds"]["tsdf"].is_number());
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
