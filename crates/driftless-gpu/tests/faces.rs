//! Bad input to the face views on the GPU is an error, never a panic; the budget counts
//! the staged image uploads.

use driftless_gpu::{
    faces_bytes, guarded, views_bytes, CarrierRule, FaceData, FaceImage, FaceParams, FaceViews,
    GpuCamera, GpuContext, VisParams,
};
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use nalgebra::{Isometry3, Matrix3};

fn gpu() -> Option<GpuContext> {
    let ctx = GpuContext::new_for_tests();
    if ctx.is_none() {
        eprintln!("übersprungen: kein Grafikadapter vorhanden");
    }
    ctx
}

fn camera() -> GpuCamera {
    let calib = CameraCalib {
        k: Matrix3::new(40.0, 0.0, 31.5, 0.0, 40.0, 23.5, 0.0, 0.0, 1.0),
        model: CameraModel::Pinhole,
        width: 64,
        height: 48,
        lidar_to_cam: None,
    };
    GpuCamera::from_camera(&Camera::from_calib("test", &calib).unwrap())
}

fn params(k: usize, zbuffer_scale: u32) -> FaceParams {
    FaceParams {
        vis: VisParams {
            zbuffer_scale,
            ..VisParams::default()
        },
        rim_theta: 1.5,
        min_texel: 0.002,
        views_per_face: k,
        sample_stride: None,
        sample_gap_cells: 1.5,
        max_subdivisions: 8,
        smooth_cos_min: 0.05,
        carrier: CarrierRule {
            radius: 1.0,
            below: 1.0,
            normal_z: 0.9,
            alternative: 0.25,
        },
        angle_step: std::f32::consts::PI / 127.0,
        reach: 0.1,
    }
}

fn face(_: usize) -> FaceData {
    FaceData {
        corners: [[0.0, 0.0, 2.0], [0.1, 0.0, 2.0], [0.0, 0.1, 2.0]],
        centroid: [0.033, 0.033, 2.0],
        longest: 0.15,
        normal: [0.0, 0.0, -1.0],
        smooth: [0.0, 0.0, -1.0],
    }
}

fn image(rgb: &[u8], size: (u32, u32)) -> FaceImage<'_> {
    FaceImage {
        geometry: camera(),
        full: camera(),
        world_from_cam: Isometry3::identity(),
        rgb,
        rgb_size: size,
        to_colour: 1.0,
        rank: 0,
        sharpness: 1.0,
    }
}

#[test]
fn bad_input_is_an_error_not_a_panic() {
    let Some(ctx) = gpu() else { return };
    let lin = [0.5f32; 256];
    // No views per face.
    let none = guarded(&ctx, || {
        FaceViews::new(&ctx, 4, face, &lin, params(0, 1), 4, 0..4)
    })
    .unwrap();
    let err = none.err().expect("k = 0 is refused").to_string();
    assert!(err.contains("Grafikkarte"), "{err}");

    let mut views = guarded(&ctx, || {
        FaceViews::new(&ctx, 4, face, &lin, params(2, 1), 4, 0..4)
    })
    .unwrap()
    .unwrap();
    views.probe().unwrap();
    // An image whose bytes do not match its size, and an empty one.
    let short = vec![0u8; 10];
    let err = views.add_image(&image(&short, (64, 48))).unwrap_err();
    assert!(err.to_string().contains("Grafikkarte"), "{err}");
    let err = views.add_image(&image(&[], (0, 0))).unwrap_err();
    assert!(err.to_string().contains("Grafikkarte"), "{err}");
    // A good image still works afterwards.
    let rgb = vec![128u8; 64 * 48 * 3];
    views.add_image(&image(&rgb, (64, 48))).unwrap();
    assert!(views
        .read_views(|s| s)
        .unwrap()
        .iter()
        .any(|s| s.score > 0.0));

    // A depth buffer of scale 0.
    let mut zero = guarded(&ctx, || {
        FaceViews::new(&ctx, 4, face, &lin, params(2, 0), 4, 0..4)
    })
    .unwrap()
    .unwrap();
    let err = zero.add_image(&image(&rgb, (64, 48))).unwrap_err();
    assert!(err.to_string().contains("Grafikkarte"), "{err}");
}

#[test]
fn budget_counts_staged_image_uploads() {
    // The image buffer, one regrown buffer and every image staged for a submission in
    // flight (three) count; the depth buffer twice (regrowth).
    let image = 1_000_000;
    assert_eq!(
        faces_bytes(0, 1, 0, image, 0) - faces_bytes(0, 1, 0, 0, 0),
        5 * image as u64
    );
    assert_eq!(
        faces_bytes(0, 1, 0, 0, 1000) - faces_bytes(0, 1, 0, 0, 0),
        8000
    );
    assert_eq!(
        views_bytes(0, 1, image, 0) - views_bytes(0, 1, 0, 0),
        5 * image as u64
    );
    assert_eq!(views_bytes(0, 1, 0, 1000) - views_bytes(0, 1, 0, 0), 8000);
}
