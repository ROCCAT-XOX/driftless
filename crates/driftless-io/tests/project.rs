use driftless_io::calib::{load_calibration, CalibError, CameraModel};
use driftless_io::project::{discover_project, ProjectError};
use std::fs;
use std::path::{Path, PathBuf};

fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/calib_synthetic.json")
}

/// Fresh empty directory under the system temp dir.
fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-project-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn touch(root: &Path, rel: &str) {
    let p = root.join(rel);
    fs::create_dir_all(p.parent().unwrap()).unwrap();
    fs::write(p, b"x").unwrap();
}

#[test]
fn parses_two_fisheye_cameras() {
    let c = load_calibration(&fixture()).unwrap();
    assert_eq!(c.device_sn.as_deref(), Some("TEST0000001"));
    assert_eq!(c.device_model.as_deref(), Some("TESTMODEL"));
    let names: Vec<_> = c.cameras.keys().cloned().collect();
    assert_eq!(names, ["left", "right"]);
    let left = &c.cameras["left"];
    assert_eq!(
        left.model,
        CameraModel::FisheyeKB {
            k: [0.01, -0.02, 0.003, -0.0004]
        }
    );
    assert_eq!((left.width, left.height), (640, 480));
    assert_eq!(left.k[(0, 0)], 400.5);
    assert_eq!(left.k[(0, 2)], 320.25);
    assert_eq!(left.k[(1, 1)], 401.5);
    assert_eq!(left.k[(2, 2)], 1.0);
    let m = left.lidar_to_cam_raw.unwrap();
    assert_eq!(m[(0, 3)], 4.0);
    assert_eq!(m[(1, 0)], 5.0);
    assert_eq!(m[(3, 3)], 1.0);
    // camera without out_put entry
    assert!(c.cameras["right"].lidar_to_cam_raw.is_none());
}

#[test]
fn unknown_camera_model_is_error() {
    let dir = tmp("badmodel");
    let text = fs::read_to_string(fixture())
        .unwrap()
        .replacen("fisheyeKB", "mystery", 1);
    fs::write(dir.join("calib.json"), text).unwrap();
    assert!(matches!(
        load_calibration(&dir.join("calib.json")),
        Err(CalibError::UnknownModel(m)) if m == "mystery"
    ));
}

#[test]
fn full_project_has_no_problems_and_sorts_naturally() {
    let dir = tmp("full");
    for f in [
        "LIDAR_10.bag",
        "LIDAR_2.bag",
        "LIDAR_1.BAG",
        "IMAGE_0.bag",
        "calibration/calib.json",
        "notes.txt",
        "lidar_x.bag",
    ] {
        touch(&dir, f);
    }
    let p = discover_project(&dir).unwrap();
    let names = |v: &[PathBuf]| -> Vec<String> {
        v.iter()
            .map(|p| p.file_name().unwrap().to_string_lossy().into_owned())
            .collect()
    };
    assert_eq!(
        names(&p.lidar_bags),
        ["LIDAR_1.BAG", "LIDAR_2.bag", "LIDAR_10.bag"]
    );
    assert_eq!(names(&p.image_bags), ["IMAGE_0.bag"]);
    assert!(p.calibration.is_some());
    assert!(p.problems.is_empty(), "{:?}", p.problems);
}

#[test]
fn missing_camera_bag_is_problem() {
    let dir = tmp("noimage");
    touch(&dir, "LIDAR_0.bag");
    touch(&dir, "calibration/calib.json");
    let p = discover_project(&dir).unwrap();
    assert_eq!(p.lidar_bags.len(), 1);
    assert!(p.image_bags.is_empty());
    assert_eq!(p.problems, ["Kamera-Aufnahme (IMAGE_*.bag) fehlt"]);
}

#[test]
fn missing_calib_is_problem() {
    let dir = tmp("nocalib");
    touch(&dir, "LIDAR_0.bag");
    touch(&dir, "IMAGE_0.bag");
    let p = discover_project(&dir).unwrap();
    assert!(p.calibration.is_none());
    assert_eq!(p.problems, ["Kalibrierung (calibration/calib.json) fehlt"]);
}

#[test]
fn empty_dir_is_error() {
    let dir = tmp("empty");
    assert!(matches!(
        discover_project(&dir),
        Err(ProjectError::NoLidarBag(_))
    ));
}

#[test]
fn missing_or_non_directory_root_is_error() {
    let dir = tmp("root");
    assert!(matches!(
        discover_project(&dir.join("nope")),
        Err(ProjectError::NotFound(_))
    ));
    touch(&dir, "file.txt");
    assert!(matches!(
        discover_project(&dir.join("file.txt")),
        Err(ProjectError::NotADirectory(_))
    ));
}

#[test]
#[ignore = "needs real data in $DRIFTLESS_TESTDATA"]
fn real_project_jmk6() {
    let Some(root) = driftless_io::testdata::path("Test") else {
        eprintln!("SKIPPED: DRIFTLESS_TESTDATA not set");
        return;
    };
    let p = discover_project(&root).unwrap();
    assert!(p.problems.is_empty(), "{:?}", p.problems);
    assert!(!p.lidar_bags.is_empty() && !p.image_bags.is_empty());
    let c = load_calibration(p.calibration.as_ref().unwrap()).unwrap();
    assert_eq!(c.device_sn.as_deref(), Some("JMK6001330"));
    assert_eq!(c.device_model.as_deref(), Some("JMK6"));
    assert!(!c.cameras.is_empty());
}
