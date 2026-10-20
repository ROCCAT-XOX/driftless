use driftless_io::calib::{load_calibration, CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use nalgebra::Vector3;
use std::path::Path;

/// Numerically stable angle between two vectors.
fn angle(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a.cross(b).norm().atan2(a.dot(b))
}

fn cameras() -> Vec<Camera> {
    let calib = load_calibration(Path::new("tests/fixtures/calib_synthetic.json")).unwrap();
    calib
        .cameras
        .iter()
        .map(|(n, c)| Camera::from_calib(n, c).unwrap())
        .collect()
}

#[test]
fn kb_project_unproject_round_trip() {
    for cam in cameras() {
        assert!(cam.max_theta >= 90f64.to_radians(), "{}", cam.name);
        let mut checked = 0;
        for i in 0..1000 {
            let theta = 90f64.to_radians() * (i as f64 + 0.5) / 1000.0;
            let phi = i as f64 * 2.399963; // golden angle: spreads azimuth
            let dir = Vector3::new(
                theta.sin() * phi.cos(),
                theta.sin() * phi.sin(),
                theta.cos(),
            );
            // The synthetic image (640x480) is smaller than the field of view; skip misses.
            let Some(uv) = cam.project(&(dir * 2.5)) else {
                continue;
            };
            let back = cam.unproject(uv).expect("unproject");
            let err = angle(&back, &dir);
            assert!(err < 1e-9, "{} theta {theta}: error {err}", cam.name);
            checked += 1;
        }
        assert!(
            checked > 100,
            "too few directions inside the image: {checked}"
        );
    }
}

/// Same round trip with an image large enough to contain the whole 90 degree field of view.
#[test]
fn kb_round_trip_full_field_of_view() {
    for mut cam in cameras() {
        cam.width = 4000;
        cam.height = 4000;
        cam.cx = 2000.0;
        cam.cy = 2000.0;
        cam.fx = 1.0;
        cam.fy = 1.0;
        let mut checked = 0;
        for i in 0..1000 {
            let theta = 90f64.to_radians() * (i as f64) / 999.0;
            let phi = i as f64 * 2.399963;
            let dir = Vector3::new(
                theta.sin() * phi.cos(),
                theta.sin() * phi.sin(),
                theta.cos(),
            );
            let uv = cam.project(&dir).expect("inside image");
            let back = cam.unproject(uv).expect("unproject");
            assert!(angle(&back, &dir) < 1e-9);
            checked += 1;
        }
        assert_eq!(checked, 1000);
    }
}

#[test]
fn kb_center_maps_to_principal_point() {
    for cam in cameras() {
        let uv = cam.project(&Vector3::new(0.0, 0.0, 3.0)).unwrap();
        assert_eq!(uv, [cam.cx, cam.cy]);
        let ray = cam.unproject([cam.cx, cam.cy]).unwrap();
        assert!((ray - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-15);
    }
}

#[test]
fn kb_rejects_behind_and_beyond_max_theta() {
    let cam = cameras().remove(0);
    assert!(cam.project(&Vector3::new(0.0, 0.0, -1.0)).is_none());
    assert!(cam.project(&Vector3::new(1.0, 0.0, -0.1)).is_none());
    let beyond = cam.max_theta + 0.01;
    assert!(cam
        .project(&Vector3::new(beyond.sin(), 0.0, beyond.cos()))
        .is_none());
    // Outside the image.
    assert!(cam.project(&Vector3::new(10.0, 0.0, 0.1)).is_none());
    assert!(cam.unproject([f64::NAN, 0.0]).is_none());
}

#[test]
fn max_theta_stops_at_monotonicity_limit() {
    // k1 strongly negative: θd peaks well below 100 degrees.
    let calib = CameraCalib {
        k: nalgebra::Matrix3::new(100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0),
        model: CameraModel::FisheyeKB {
            k: [-0.2, 0.0, 0.0, 0.0],
        },
        width: 100,
        height: 100,
        lidar_to_cam_raw: None,
    };
    let cam = Camera::from_calib("x", &calib).unwrap();
    // 1 + 3 k1 θ² = 0 -> θ = sqrt(1/0.6)
    assert!((cam.max_theta - (1.0f64 / 0.6).sqrt()).abs() < 1e-9);
}

#[test]
fn pinhole_round_trip() {
    let calib = CameraCalib {
        k: nalgebra::Matrix3::new(500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
        model: CameraModel::Pinhole,
        width: 640,
        height: 480,
        lidar_to_cam_raw: None,
    };
    let cam = Camera::from_calib("p", &calib).unwrap();
    let dir = Vector3::new(0.1, -0.2, 1.0).normalize();
    let uv = cam.project(&dir).unwrap();
    assert!((cam.unproject(uv).unwrap() - dir).norm() < 1e-12);
    assert!(cam.project(&Vector3::new(0.0, 0.0, -1.0)).is_none());
}
