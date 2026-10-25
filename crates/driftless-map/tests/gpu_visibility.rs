//! GPU depth buffer and visibility (`driftless-gpu`) against the CPU reference
//! [`driftless_map::visibility::visible_points`].

mod common;

use common::*;
use driftless_gpu::{GpuCamera, GpuCloud, GpuContext, ImagePass, VisParams};
use driftless_io::calib::CameraModel;
use driftless_io::camera::Camera;
use driftless_map::visibility::{visible_points, DEPTH_TOLERANCE_REL, MAX_SPLAT, SILHOUETTE_REL};
use driftless_map::ColorConfig;
use driftless_traj::interpolate_pose;
use driftless_traj::nalgebra::{Isometry3, Point3, Vector3};
use std::time::Instant;

/// A GPU context (software adapters allowed), or `None` with a note when there is none.
fn gpu() -> Option<GpuContext> {
    match GpuContext::new_for_tests() {
        Some(ctx) => {
            eprintln!("GPU-Test auf {:?}", ctx.info());
            Some(ctx)
        }
        None => {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            None
        }
    }
}

#[test]
fn vis_params_mirror_color_config() {
    let cfg = ColorConfig::default();
    let p = VisParams::default();
    assert_eq!(p.max_dist, cfg.max_dist);
    assert_eq!(p.depth_tolerance, cfg.depth_tolerance);
    assert_eq!(p.zbuffer_scale, cfg.zbuffer_scale);
    assert_eq!(p.depth_tolerance_rel, DEPTH_TOLERANCE_REL);
    assert_eq!(p.silhouette_rel, SILHOUETTE_REL);
    assert_eq!(p.max_splat, MAX_SPLAT);
}

/// Pseudo-random spacings in `[lo, hi)` (fixed seed), so splat sizes vary from point to point.
fn spacings(n: usize, lo: f32, hi: f32) -> Vec<f32> {
    let mut s = 0x9e37_79b9_7f4a_7c15u64;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            lo + (hi - lo) * ((s >> 40) as f32 / (1u64 << 24) as f32)
        })
        .collect()
}

/// Camera (decoded size) and world pose of every image of the colouring tests' walk.
fn walk_images() -> Vec<(Camera, Isometry3<f64>)> {
    let cfg = ColorConfig::default();
    let cams = cameras();
    let poses = poses(&walk(), 6.0);
    walk_shots()
        .iter()
        .map(|s| {
            let c = &cams[s.camera];
            let body = interpolate_pose(&poses, s.stamp + cfg.time_offset).unwrap();
            (c.camera.downscaled(cfg.downscale), body * c.cam_in_body)
        })
        .collect()
}

/// CPU visibility per point of `xyz` (all points are candidates: the grid only narrows them
/// down to the ones within `max_dist`, which the projection checks again).
fn cpu_visible(
    xyz: &[[f32; 3]],
    spacing: &[f32],
    cam: &Camera,
    world_cam: &Isometry3<f64>,
    p: &VisParams,
) -> Vec<bool> {
    let all: Vec<u32> = (0..xyz.len() as u32).collect();
    let mut seen = vec![false; xyz.len()];
    for s in visible_points(
        xyz,
        spacing,
        &all,
        cam,
        &world_cam.inverse(),
        p.max_dist,
        p.zbuffer_scale,
        p.depth_tolerance,
    ) {
        seen[s.index as usize] = true;
    }
    seen
}

/// Whether `p` lies at a depth edge of the image: its 3 × 3 cells in `depth` (the GPU's
/// buffer) hold depths more than the relative tolerance apart, or it projects within 0.01 px
/// of a cell border.
fn at_depth_edge(
    p: &[f32; 3],
    cam: &Camera,
    world_cam: &Isometry3<f64>,
    depth: &[f32],
    scale: u32,
) -> bool {
    let pc = world_cam.inverse() * Point3::from(Vector3::from(p.map(f64::from)));
    let Some(uv) = cam.project(&pc.coords) else {
        return true; // at the rim of the lens model or the image
    };
    let (w, h) = (
        cam.width.div_ceil(scale) as i64,
        cam.height.div_ceil(scale) as i64,
    );
    let s = f64::from(scale);
    let near_border = uv.iter().any(|&v| {
        let f = ((v + 0.5) / s).fract();
        f.min(1.0 - f) * s < 0.01
    });
    let (cx, cy) = (
        (((uv[0] + 0.5) / s) as i64).clamp(0, w - 1),
        (((uv[1] + 0.5) / s) as i64).clamp(0, h - 1),
    );
    let mut lo = f32::INFINITY;
    let mut hi = 0f32;
    for y in (cy - 1).max(0)..=(cy + 1).min(h - 1) {
        for x in (cx - 1).max(0)..=(cx + 1).min(w - 1) {
            let d = depth[(y * w + x) as usize];
            lo = lo.min(d);
            hi = hi.max(d);
        }
    }
    near_border || f64::from(hi) > f64::from(lo) * (1.0 + DEPTH_TOLERANCE_REL)
}

#[test]
fn visible_set_matches_cpu() {
    let Some(ctx) = gpu() else { return };
    let painted = walk_scene();
    let cloud = surface_cloud(&painted);
    let spacing = spacings(cloud.len(), 0.03, 0.15);
    let normals = vec![None; cloud.len()];
    // Small batches: the cloud spans several, as a large scan does.
    let gpu_cloud =
        GpuCloud::with_batch_points(&ctx, &cloud.xyz, &normals, &spacing, 50_000).unwrap();
    assert!(gpu_cloud.batch_count() > 1);
    let mut pass = ImagePass::new(&ctx, &gpu_cloud);
    let params = VisParams::default();
    let (mut total, mut differ) = (0usize, 0usize);
    for (n, (cam, world_cam)) in walk_images().iter().enumerate() {
        let cpu = cpu_visible(&cloud.xyz, &spacing, cam, world_cam, &params);
        let v = pass.depth_and_visibility(&GpuCamera::from_camera(cam), world_cam, &params);
        let gpu = v.read();
        let depth = v.read_depth();
        let seen = cpu.iter().filter(|&&s| s).count();
        let mut d = 0;
        for i in (0..cpu.len()).filter(|&i| cpu[i] != gpu[i]) {
            d += 1;
            assert!(
                at_depth_edge(&cloud.xyz[i], cam, world_cam, &depth, params.zbuffer_scale),
                "Bild {n}: Punkt {i} {:?} weicht ab (CPU {}), liegt an keiner Tiefenkante",
                cloud.xyz[i],
                cpu[i]
            );
        }
        assert!(seen > 1000, "Bild {n}: nur {seen} Punkte sichtbar");
        assert!(
            d * 1000 <= seen,
            "Bild {n}: {d} von {seen} sichtbaren Punkten weichen ab"
        );
        total += seen;
        differ += d;
    }
    println!("{differ} von {total} sichtbaren Punkten weichen ab");
}

/// The CPU unit cases of `visibility.rs`: a nearer point hides one behind, a close point's
/// splat covers its footprint, a point beside an occluder's silhouette is not visible, a
/// slanted surface does not hide its neighbours.
#[test]
fn small_cases_match_cpu() {
    let Some(ctx) = gpu() else { return };
    let cam = Camera {
        name: "test".into(),
        width: 100,
        height: 80,
        fx: 50.0,
        fy: 50.0,
        cx: 49.5,
        cy: 39.5,
        model: CameraModel::Pinhole,
        max_theta: std::f64::consts::FRAC_PI_2,
    };
    let cases: [(&[[f32; 3]], &[f32]); 5] = [
        (
            &[
                [0.0, 0.0, 2.0],
                [0.0, 0.0, 4.0],
                [0.0, 0.0, 2.03],
                [1.0, 0.0, 4.0],
            ],
            &[0.0; 4],
        ),
        (&[[0.0, 0.0, 0.5], [0.64, 0.0, 4.0]], &[0.0, 0.0]),
        (&[[0.0, 0.0, 0.5], [0.64, 0.0, 4.0]], &[0.2, 0.0]),
        (&[[0.0, 0.0, 0.5], [0.32, 0.0, 4.0]], &[0.0, 0.0]),
        (&[[0.0, 0.0, 3.6], [0.32, 0.0, 4.0]], &[0.0, 0.0]),
    ];
    let expected = [
        vec![true, false, true, true],
        vec![true, true],
        vec![true, false],
        vec![true, false],
        vec![true, true],
    ];
    for (max_dist, far) in [(12.0, false), (3.0, true)] {
        let params = VisParams {
            max_dist,
            ..VisParams::default()
        };
        for ((xyz, spacing), want) in cases.iter().zip(&expected) {
            let cpu = cpu_visible(xyz, spacing, &cam, &Isometry3::identity(), &params);
            let cloud = GpuCloud::new(&ctx, xyz, &vec![None; xyz.len()], spacing).unwrap();
            let gpu = ImagePass::new(&ctx, &cloud)
                .depth_and_visibility(
                    &GpuCamera::from_camera(&cam),
                    &Isometry3::identity(),
                    &params,
                )
                .read();
            if !far {
                assert_eq!(&cpu, want, "{xyz:?}");
            }
            assert_eq!(gpu, cpu, "{xyz:?} max_dist {max_dist}");
        }
    }
}

/// Time per image, GPU against CPU, on a cloud of about a million points
/// (`cargo test --release -p driftless-map --test gpu_visibility -- --ignored --nocapture`).
#[test]
#[ignore = "Messung"]
fn timing_one_million_points() {
    let Some(ctx) = gpu() else { return };
    let painted = walk_scene();
    let xyz: Vec<[f32; 3]> = painted
        .scene
        .surface_samples(0.0185, 0.009)
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let spacing = vec![0.0185; xyz.len()];
    let normals = vec![None; xyz.len()];
    let images = walk_images();
    let params = VisParams::default();
    let t = Instant::now();
    let cloud = GpuCloud::new(&ctx, &xyz, &normals, &spacing).unwrap();
    let upload = t.elapsed();
    let mut pass = ImagePass::new(&ctx, &cloud);
    let gpu_run = |pass: &mut ImagePass, i: usize| {
        let (cam, pose) = &images[i % images.len()];
        pass.depth_and_visibility(&GpuCamera::from_camera(cam), pose, &params)
            .read()
    };
    gpu_run(&mut pass, 0); // warm-up: pipelines, buffers
    let t = Instant::now();
    let mut gpu_seen = 0;
    for i in 0..images.len() {
        gpu_seen += gpu_run(&mut pass, i).iter().filter(|&&s| s).count();
    }
    let gpu = t.elapsed() / images.len() as u32;
    let t = Instant::now();
    let mut cpu_seen = 0;
    for (cam, pose) in &images {
        cpu_seen += cpu_visible(&xyz, &spacing, cam, pose, &params)
            .iter()
            .filter(|&&s| s)
            .count();
    }
    let cpu = t.elapsed() / images.len() as u32;
    println!(
        "{} Punkte, {} Bilder ({}×{}), Hochladen {upload:?}: GPU {gpu:?} je Bild ({gpu_seen} \
         sichtbar), CPU {cpu:?} je Bild ({cpu_seen} sichtbar), {:.1}×, Gerät {:?}",
        xyz.len(),
        images.len(),
        images[0].0.width,
        images[0].0.height,
        cpu.as_secs_f64() / gpu.as_secs_f64(),
        ctx.info()
    );
}
