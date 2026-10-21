use driftless_io::calib::{
    load_calibration, load_calibration_as, CalibError, CalibReading, CameraModel, ImageRotation,
    MatrixDirection, CALIB_READING,
};
use driftless_io::camera::Camera;
use driftless_io::project::{discover_project, ProjectError};
use nalgebra::Point3;
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

/// `K` and `transform_matrix` exactly as written in the file.
const AS_WRITTEN: CalibReading = CalibReading {
    rotation: ImageRotation::None,
    direction: MatrixDirection::LidarToCam,
};

#[test]
fn parses_two_fisheye_cameras() {
    let c = load_calibration_as(&fixture(), AS_WRITTEN).unwrap();
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
    let m = left.lidar_to_cam.unwrap().to_homogeneous();
    assert!((m[(0, 3)] - 0.1).abs() < 1e-12);
    assert!((m[(1, 2)] + 1.0).abs() < 1e-12);
    assert!((m[(2, 0)] - 1.0).abs() < 1e-12);
    assert_eq!(m[(3, 3)], 1.0);
    // camera without out_put entry
    assert!(c.cameras["right"].lidar_to_cam.is_none());
}

/// Every reading must give the projection of the image `K` refers to, turned back into the
/// stored image; the measured reading is what `load_calibration` returns.
#[test]
fn readings_turn_k_and_matrix_into_the_stored_image() {
    let raw = load_calibration_as(&fixture(), AS_WRITTEN).unwrap();
    let raw = &raw.cameras["left"];
    let (w1, h1) = (f64::from(raw.width) - 1.0, f64::from(raw.height) - 1.0);
    for rotation in [
        ImageRotation::None,
        ImageRotation::Cw90,
        ImageRotation::Ccw90,
        ImageRotation::R180,
    ] {
        for direction in [MatrixDirection::LidarToCam, MatrixDirection::CamToLidar] {
            let reading = CalibReading {
                rotation,
                direction,
            };
            let c = load_calibration_as(&fixture(), reading).unwrap();
            let c = &c.cameras["left"];
            assert_eq!((c.width, c.height), (raw.width, raw.height));
            let stored = Camera::from_calib("s", c).unwrap();
            // The turned image: `K` as written, axes swapped for 90° turns.
            let mut turned = raw.clone();
            if matches!(rotation, ImageRotation::Cw90 | ImageRotation::Ccw90) {
                std::mem::swap(&mut turned.width, &mut turned.height);
            }
            let turned = Camera::from_calib("t", &turned).unwrap();
            let m = raw.lidar_to_cam.unwrap();
            let m = match direction {
                MatrixDirection::LidarToCam => m,
                MatrixDirection::CamToLidar => m.inverse(),
            };
            let mut checked = 0;
            for i in 0..1000 {
                // Spread over the sphere (golden angle), 3 m away.
                let f = f64::from(i);
                let z = 1.0 - 2.0 * (f + 0.5) / 1000.0;
                let (r, phi) = ((1.0 - z * z).sqrt(), f * 2.399963);
                let p = Point3::new(3.0 * r * phi.cos(), 3.0 * r * phi.sin(), 3.0 * z);
                let Some([x, y]) = turned.project(&(m * p).coords) else {
                    continue;
                };
                let expected = match rotation {
                    ImageRotation::None => [x, y],
                    ImageRotation::Cw90 => [y, h1 - x],
                    ImageRotation::Ccw90 => [w1 - y, x],
                    ImageRotation::R180 => [w1 - x, h1 - y],
                };
                let Some(got) = stored.project(&(c.lidar_to_cam.unwrap() * p).coords) else {
                    // Turning moves the half-open pixel bounds by up to one pixel.
                    let near_border = expected[0] < 0.0
                        || expected[1] < 0.0
                        || expected[0] > w1
                        || expected[1] > h1;
                    assert!(near_border, "{reading:?}: {expected:?} not projected");
                    continue;
                };
                assert!(
                    (got[0] - expected[0]).abs() < 1e-9 && (got[1] - expected[1]).abs() < 1e-9,
                    "{reading:?}: {got:?} vs {expected:?}"
                );
                checked += 1;
            }
            assert!(checked > 20, "{reading:?}: only {checked} points inside");
        }
    }
    assert_eq!(
        load_calibration(&fixture()).unwrap(),
        load_calibration_as(&fixture(), CALIB_READING).unwrap()
    );
}

#[test]
fn non_rigid_transform_matrix_is_error() {
    let dir = tmp("nonrigid");
    let text = fs::read_to_string(fixture()).unwrap().replacen(
        "[0.0, -1.0, 0.0, 0.1]",
        "[0.0, -2.0, 0.0, 0.1]",
        1,
    );
    fs::write(dir.join("calib.json"), text).unwrap();
    assert!(matches!(
        load_calibration(&dir.join("calib.json")),
        Err(CalibError::Malformed(m)) if m.contains("not rigid")
    ));
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

#[cfg(unix)]
#[test]
fn symlinked_bags_are_found() {
    let dir = tmp("symlink");
    let store = tmp("symlink-store");
    touch(&store, "a.bag");
    touch(&store, "b.bag");
    std::os::unix::fs::symlink(store.join("a.bag"), dir.join("LIDAR_0.bag")).unwrap();
    std::os::unix::fs::symlink(store.join("b.bag"), dir.join("IMAGE_0.bag")).unwrap();
    let p = discover_project(&dir).unwrap();
    assert_eq!(p.lidar_bags.len(), 1);
    assert_eq!(p.image_bags.len(), 1);
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
    assert_eq!(c.device_model.as_deref(), Some("JMK6"));
    assert!(c.device_sn.as_deref().is_some_and(|sn| !sn.is_empty()));
    assert!(!c.cameras.is_empty());
}
