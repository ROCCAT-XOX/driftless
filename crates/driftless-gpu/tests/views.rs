//! Failures of the view selection on the GPU are errors, never panics.

use driftless_gpu::{
    guarded, GpuCamera, GpuCloud, GpuContext, ImageViews, ViewImage, ViewParams, VisParams,
};
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use nalgebra::{Isometry3, Matrix3, Vector3};

fn gpu() -> Option<GpuContext> {
    let ctx = GpuContext::new_for_tests();
    if ctx.is_none() {
        eprintln!("übersprungen: kein Grafikadapter vorhanden");
    }
    ctx
}

fn camera() -> Camera {
    let calib = CameraCalib {
        k: Matrix3::new(40.0, 0.0, 31.5, 0.0, 40.0, 23.5, 0.0, 0.0, 1.0),
        model: CameraModel::Pinhole,
        width: 64,
        height: 48,
        lidar_to_cam: None,
    };
    Camera::from_calib("test", &calib).unwrap()
}

fn params() -> ViewParams {
    ViewParams {
        vis: VisParams::default(),
        views_kept: 2,
        full_weight_theta: 1.0,
        stretch: None,
    }
}

#[test]
fn lost_device_is_an_error_not_a_panic() {
    let Some(ctx) = gpu() else { return };
    let xyz: Vec<[f32; 3]> = (0..100).map(|i| [0.01 * i as f32, 0.0, 2.0]).collect();
    let normals = vec![Some(Vector3::new(0.0, 0.0, -1.0)); xyz.len()];
    let spacing = vec![0.01; xyz.len()];
    let cloud = guarded(&ctx, || GpuCloud::new(&ctx, &xyz, &normals, &spacing))
        .unwrap()
        .unwrap();
    let lin = [0.5f32; 256];
    let mut views = guarded(&ctx, || ImageViews::new(&ctx, &cloud, &lin, params())).unwrap();
    views.probe().unwrap();
    let rgb = vec![128u8; 64 * 48 * 3];
    let img = ViewImage {
        cam: GpuCamera::from_camera(&camera()),
        world_from_cam: Isometry3::identity(),
        rgb: &rgb,
        rank: 0,
        sharpness: 1.0,
    };
    views.add_image(&img).unwrap();
    assert!(views.read(|s| s).unwrap().iter().any(|s| s.weight > 0.0));

    // The device goes away (as when the Win11 VM takes the card): errors in German.
    ctx.device().destroy();
    let added = views.add_image(&img);
    let read = views.read(|s| s);
    let err = added
        .err()
        .or(read.err())
        .expect("an error after the device is gone");
    let text = err.to_string();
    eprintln!("nach destroy: {text}");
    assert!(text.contains("Grafikkarte"), "{text}");
}

#[test]
fn budget_is_checked_before_allocating() {
    let Some(ctx) = gpu() else { return };
    let need = driftless_gpu::views_bytes(1_000_000, 6, 960 * 540 * 3, 240 * 135);
    assert!(need > 128 * 1_000_000, "{need}");
    assert!(ctx.check_budget(need).is_ok(), "default budget 4 GB");
    let small = ctx.with_budget(1000);
    let err = small.check_budget(need).unwrap_err().to_string();
    assert!(err.contains("Grafikspeicher"), "{err}");
    // The spec's 4 GB per room scan is the ceiling, whatever the device offers.
    let huge = driftless_gpu::views_bytes(40_000_000, 6, 0, 0);
    assert!(gpu().unwrap().check_budget(huge).is_err());
}

#[test]
fn image_of_wrong_size_is_an_error_not_a_panic() {
    let Some(ctx) = gpu() else { return };
    let xyz: Vec<[f32; 3]> = (0..10).map(|i| [0.01 * i as f32, 0.0, 2.0]).collect();
    let normals = vec![Some(Vector3::new(0.0, 0.0, -1.0)); xyz.len()];
    let spacing = vec![0.01; xyz.len()];
    let cloud = guarded(&ctx, || GpuCloud::new(&ctx, &xyz, &normals, &spacing))
        .unwrap()
        .unwrap();
    let lin = [0.5f32; 256];
    let mut views = guarded(&ctx, || ImageViews::new(&ctx, &cloud, &lin, params())).unwrap();
    views.probe().unwrap();
    let rgb = vec![128u8; 10];
    let err = views
        .add_image(&ViewImage {
            cam: GpuCamera::from_camera(&camera()),
            world_from_cam: Isometry3::identity(),
            rgb: &rgb,
            rank: 0,
            sharpness: 1.0,
        })
        .unwrap_err();
    assert!(err.to_string().contains("Grafikkarte"), "{err}");
}
