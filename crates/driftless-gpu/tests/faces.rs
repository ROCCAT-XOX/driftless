//! Bad input to the face views on the GPU is an error, never a panic; the budget counts
//! the staged image uploads.

use driftless_gpu::{
    faces_bytes, faces_pass_bytes, guarded, views_bytes, CarrierRule, FaceData, FaceImage,
    FaceOccluders, FaceParams, FaceSlot, FaceViews, GpuCamera, GpuContext, VisParams,
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
        occluder_gap: 0.02,
        occluded_score: 0.04,
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
        FaceViews::new(
            &ctx,
            4,
            face,
            &lin,
            params(0, 1),
            4,
            0..4,
            FaceOccluders::default(),
        )
    })
    .unwrap();
    let err = none.err().expect("k = 0 is refused").to_string();
    assert!(err.contains("Grafikkarte"), "{err}");

    let mut views = guarded(&ctx, || {
        FaceViews::new(
            &ctx,
            4,
            face,
            &lin,
            params(2, 1),
            4,
            0..4,
            FaceOccluders::default(),
        )
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
        FaceViews::new(
            &ctx,
            4,
            face,
            &lin,
            params(2, 0),
            4,
            0..4,
            FaceOccluders::default(),
        )
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
        faces_bytes(0, 1, 0, image, 0, 0) - faces_bytes(0, 1, 0, 0, 0, 0),
        5 * image as u64
    );
    assert_eq!(
        faces_bytes(0, 1, 0, 0, 1000, 0) - faces_bytes(0, 1, 0, 0, 0, 0),
        8000
    );
    assert_eq!(
        views_bytes(0, 1, image, 0) - views_bytes(0, 1, 0, 0),
        5 * image as u64
    );
    assert_eq!(views_bytes(0, 1, 0, 1000) - views_bytes(0, 1, 0, 0), 8000);
}

#[test]
fn budget_counts_the_occluders_and_their_depth_buffer() {
    // 16 B per occluder and the second depth buffer twice (regrowth), in every pass.
    let (cells, occluders) = (1000, 500);
    let none = faces_pass_bytes(0, 0, 1, 0, 0, cells, 0);
    assert_eq!(
        faces_pass_bytes(0, 0, 1, 0, 0, cells, occluders) - none,
        500 * 16 + 8000
    );
    assert_eq!(
        faces_bytes(0, 1, 0, 0, cells, occluders) - faces_bytes(0, 1, 0, 0, cells, 0),
        500 * 16 + 8000
    );
    // Without occluders no second depth buffer is counted: only the first one's 8000.
    assert_eq!(none - faces_pass_bytes(0, 0, 1, 0, 0, 0, 0), 8000);
}

/// The best view of [`face`] from the origin with `occluders` of `spacing`.
fn best_view(ctx: &GpuContext, points: &[[f32; 3]], spacing: f32) -> FaceSlot {
    let lin = [0.5f32; 256];
    let occluders = FaceOccluders { points, spacing };
    let mut views = guarded(ctx, || {
        FaceViews::new(ctx, 1, face, &lin, params(2, 1), 1, 0..1, occluders)
    })
    .unwrap()
    .unwrap();
    views.probe().unwrap();
    let rgb = vec![128u8; 64 * 48 * 3];
    views.add_image(&image(&rgb, (64, 48))).unwrap();
    views.read_views(|s| s).unwrap()[0]
}

/// `p` moved towards the camera at the origin to `share` of its distance: the same pixel.
fn on_the_ray(p: [f32; 3], share: f32) -> [f32; 3] {
    p.map(|v| v * share)
}

#[test]
fn every_one_of_the_seven_samples_counts() {
    let Some(ctx) = gpu() else { return };
    let plain = best_view(&ctx, &[], 0.0);
    assert!(plain.score > 0.0 && plain.tag & 0x80 == 0, "{plain:?}");
    let f = face(0);
    let [a, b, c] = f.corners;
    let half = |p: [f32; 3], q: [f32; 3]| [0, 1, 2].map(|i| (p[i] + q[i]) * 0.5);
    for s in [f.centroid, a, b, c, half(a, b), half(b, c), half(c, a)] {
        // One cell (the footprint of such a small spacing), in front of this sample alone.
        let hit = best_view(&ctx, &[on_the_ray(s, 0.5)], 0.0005);
        assert!(hit.tag & 0x80 != 0, "{s:?}: {hit:?}");
        assert!(
            (hit.score / plain.score - 0.04).abs() < 1e-3,
            "{s:?}: {} of {}",
            hit.score,
            plain.score
        );
        assert_eq!(hit.tag & 0x7f, plain.tag & 0x7f, "{s:?}");
        assert_eq!(hit.colour, plain.colour, "{s:?}");
    }
}

#[test]
fn occluders_need_the_gap_and_their_own_cell() {
    let Some(ctx) = gpu() else { return };
    let plain = best_view(&ctx, &[], 0.0);
    let centre = face(0).centroid;
    let range = centre.map(|v| v * v).iter().sum::<f32>().sqrt();
    let at = |gap: f32| on_the_ray(centre, 1.0 - gap / range);
    // Less than 2 cm in front of the face, and behind it: the view stays as it was.
    assert_eq!(best_view(&ctx, &[at(0.01)], 0.0005), plain);
    assert_eq!(best_view(&ctx, &[at(-0.1)], 0.0005), plain);
    // More than that: hidden.
    assert!(best_view(&ctx, &[at(0.03)], 0.0005).tag & 0x80 != 0);
    // Out of the sight of every sample of the face (they lie in cells 32 ... 34, 24 ... 26):
    // the view stays, whatever its range.
    let aside = [(36.0 - 31.5) / 40.0, (25.0 - 23.5) / 40.0, 1.0];
    assert_eq!(best_view(&ctx, &[aside], 0.0005), plain);
    // With the footprint of a larger spacing (3 cells around it) the same point covers the
    // samples at cell 34 and hides the face.
    let wide = best_view(&ctx, &[aside], 0.15);
    assert!(wide.tag & 0x80 != 0, "{wide:?}");
}

#[test]
fn occluders_beyond_the_reach_change_nothing() {
    let Some(ctx) = gpu() else { return };
    let plain = best_view(&ctx, &[], 0.0);
    // Behind the camera, and beyond the farthest distance a view reaches.
    for p in [[0.0, 0.0, -1.0], [0.0, 0.0, 200.0]] {
        assert_eq!(best_view(&ctx, &[p], 0.05), plain, "{p:?}");
    }
}

#[test]
fn too_many_occluders_are_an_error() {
    let Some(ctx) = gpu() else { return };
    let lin = [0.5f32; 256];
    let points = vec![[0.0f32; 3]; 65535 * 64 + 1];
    let occluders = FaceOccluders {
        points: &points,
        spacing: 0.02,
    };
    let refused = guarded(&ctx, || {
        FaceViews::new(&ctx, 1, face, &lin, params(2, 1), 1, 0..1, occluders)
    })
    .unwrap();
    let err = refused.err().expect("refused").to_string();
    assert!(err.contains("Verdecker"), "{err}");
}
