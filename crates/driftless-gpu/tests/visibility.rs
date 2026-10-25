//! GPU depth buffer and visibility on small synthetic scenes (the comparison with the CPU
//! code lives in `driftless-map/tests/gpu_visibility.rs`).

use driftless_gpu::{GpuCamera, GpuCloud, GpuContext, ImagePass, VisParams};
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

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

/// Pinhole camera, 200 × 160 pixels, ±45° across the width, looking along +z.
fn pinhole() -> Camera {
    let calib = CameraCalib {
        k: Matrix3::new(100.0, 0.0, 99.5, 0.0, 100.0, 79.5, 0.0, 0.0, 1.0),
        model: CameraModel::Pinhole,
        width: 200,
        height: 160,
        lidar_to_cam: None,
    };
    Camera::from_calib("test", &calib).unwrap()
}

/// Wall 4 × 4 m at z = 5 and a pillar 0.4 m wide, as tall as the wall, at z = 2 (front face
/// only), both sampled every 2 cm. Returns the points and how many belong to the wall.
fn pillar_scene() -> (Vec<[f32; 3]>, usize) {
    let mut xyz = Vec::new();
    let n = 200;
    for i in 0..=n {
        for j in 0..=n {
            let x = -2.0 + 4.0 * i as f32 / n as f32;
            let y = -2.0 + 4.0 * j as f32 / n as f32;
            xyz.push([x, y, 5.0]);
        }
    }
    let wall = xyz.len();
    for i in 0..=20 {
        for j in 0..=n {
            let x = -0.2 + 0.4 * i as f32 / 20.0;
            let y = -2.0 + 4.0 * j as f32 / n as f32;
            xyz.push([x, y, 2.0]);
        }
    }
    (xyz, wall)
}

fn upload(ctx: &GpuContext, xyz: &[[f32; 3]], batch_points: Option<usize>) -> GpuCloud {
    let normals = vec![None; xyz.len()];
    let spacing = vec![0.02; xyz.len()];
    match batch_points {
        Some(b) => GpuCloud::with_batch_points(ctx, xyz, &normals, &spacing, b).unwrap(),
        None => GpuCloud::new(ctx, xyz, &normals, &spacing).unwrap(),
    }
}

#[test]
fn occluder_hides_wall() {
    let Some(ctx) = gpu() else { return };
    let (xyz, wall) = pillar_scene();
    let cloud = upload(&ctx, &xyz, None);
    let mut pass = ImagePass::new(&ctx, &cloud);
    let seen = pass
        .depth_and_visibility(
            &GpuCamera::from_camera(&pinhole()),
            &Isometry3::identity(),
            &VisParams::default(),
        )
        .read();
    assert_eq!(seen.len(), xyz.len());
    // The pillar's shadow on the wall covers |x| < 0.5 (0.2 · 5 / 2); keep two cells clear
    // of its edge (a cell is 4 px = 0.2 m on the wall), where the silhouette test decides.
    let (mut hidden, mut open) = (0, 0);
    for (p, &s) in xyz[..wall].iter().zip(&seen) {
        if p[0].abs() < 0.4 {
            assert!(!s, "wall point {p:?} behind the pillar is visible");
            hidden += 1;
        } else if p[0].abs() > 1.0 {
            assert!(s, "open wall point {p:?} is not visible");
            open += 1;
        }
    }
    assert!(hidden > 5000 && open > 15000, "{hidden} {open}");
    // The pillar is in front of everything around it: all of it inside the image (rows
    // v = 79.5 + 50·y in [0, 160)) is seen.
    for (p, &s) in xyz[wall..].iter().zip(&seen[wall..]) {
        assert_eq!(s, p[1] > -1.59 && p[1] < 1.61, "pillar point {p:?}");
    }
}

#[test]
fn far_points_and_points_behind_are_not_seen() {
    let Some(ctx) = gpu() else { return };
    let xyz = [
        [0.0, 0.0, 5.0],
        [0.0, 0.0, -5.0],
        [0.0, 0.0, 13.0],
        [0.0, 0.0, 0.0],
    ];
    let cloud = upload(&ctx, &xyz, None);
    let mut pass = ImagePass::new(&ctx, &cloud);
    let seen = pass
        .depth_and_visibility(
            &GpuCamera::from_camera(&pinhole()),
            &Isometry3::identity(),
            &VisParams::default(),
        )
        .read();
    assert_eq!(seen, vec![true, false, false, false]);
}

/// Pseudo-random points in a box in front of the camera with random spacings.
fn random_cloud(n: usize) -> (Vec<[f32; 3]>, Vec<f32>) {
    let mut s = 0x2545_f491_4f6c_dd1du64;
    let mut next = || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f32 / (1u64 << 53) as f32
    };
    let xyz = (0..n)
        .map(|_| [next() * 8.0 - 4.0, next() * 6.0 - 3.0, next() * 10.0 + 0.3])
        .collect();
    let spacing = (0..n).map(|_| next() * 0.2).collect();
    (xyz, spacing)
}

#[test]
fn deterministic_on_same_device() {
    let Some(ctx) = gpu() else { return };
    let (xyz, spacing) = random_cloud(300_000);
    let normals = vec![None; xyz.len()];
    let cam = GpuCamera::from_camera(&pinhole());
    let pose = Isometry3::from_parts(
        Translation3::new(0.1, -0.2, 0.05),
        UnitQuaternion::from_euler_angles(0.05, -0.1, 0.02),
    );
    let run = || {
        let cloud = GpuCloud::new(&ctx, &xyz, &normals, &spacing).unwrap();
        let mut pass = ImagePass::new(&ctx, &cloud);
        let v = pass.depth_and_visibility(&cam, &pose, &VisParams::default());
        let depth: Vec<u32> = v.read_depth().iter().map(|d| d.to_bits()).collect();
        (v.read_bits(), depth)
    };
    let (bits, depth) = run();
    assert!(bits.iter().any(|&b| b != 0));
    assert!(depth.iter().any(|&d| d != f32::INFINITY.to_bits()));
    for _ in 0..3 {
        assert_eq!(run(), (bits.clone(), depth.clone()));
    }
}

#[test]
fn batches_give_the_same_result_as_one() {
    let Some(ctx) = gpu() else { return };
    let (xyz, spacing) = random_cloud(100_003);
    let normals: Vec<Option<Vector3<f32>>> = (0..xyz.len())
        .map(|i| (i % 3 == 0).then(|| Vector3::new(0.0, 0.0, -1.0)))
        .collect();
    let cam = GpuCamera::from_camera(&pinhole());
    let pose = Isometry3::translation(0.0, 0.1, -0.5);
    let read = |cloud: &GpuCloud| {
        let mut pass = ImagePass::new(&ctx, cloud);
        let v = pass.depth_and_visibility(&cam, &pose, &VisParams::default());
        (v.read(), v.read_depth())
    };
    let one = GpuCloud::new(&ctx, &xyz, &normals, &spacing).unwrap();
    assert_eq!(one.batch_count(), 1);
    let many = GpuCloud::with_batch_points(&ctx, &xyz, &normals, &spacing, 7_777).unwrap();
    assert_eq!(many.batch_count(), 100_003usize.div_ceil(7_777));
    assert_eq!(many.len(), xyz.len());
    let (a, b) = (read(&one), read(&many));
    let seen = a.0.iter().filter(|&&s| s).count();
    assert!(seen > 100, "{seen}");
    assert!(a == b, "batched result differs");
}

#[test]
fn pass_is_reusable_across_images() {
    let Some(ctx) = gpu() else { return };
    let (xyz, wall) = pillar_scene();
    let cloud = upload(&ctx, &xyz, Some(10_000));
    let mut pass = ImagePass::new(&ctx, &cloud);
    let cam = GpuCamera::from_camera(&pinhole());
    let small = GpuCamera::from_camera(&pinhole().downscaled(2));
    let params = VisParams::default();
    let first = pass
        .depth_and_visibility(&cam, &Isometry3::identity(), &params)
        .read();
    // Seen from behind the wall nothing is visible ...
    let behind = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 10.0),
        UnitQuaternion::from_euler_angles(0.0, std::f64::consts::PI, 0.0),
    );
    let back = pass.depth_and_visibility(&small, &behind, &params).read();
    assert!(
        back[wall..].iter().all(|&s| !s),
        "pillar seen through the wall"
    );
    assert!(back[..wall].iter().filter(|&&s| s).count() > 10_000);
    // ... and the first image again gives the same as before.
    let again = pass
        .depth_and_visibility(&cam, &Isometry3::identity(), &params)
        .read();
    assert_eq!(first, again);
}

/// Kilometres from the world origin the result is the same as near it: the cloud is
/// centred in f64 before it goes to f32.
#[test]
fn far_from_origin_like_near() {
    let Some(ctx) = gpu() else { return };
    let (xyz, _) = pillar_scene();
    let shift = Vector3::new(30_000.0, -20_000.0, 50.0);
    let far_xyz: Vec<[f32; 3]> = xyz
        .iter()
        .map(|p| [0, 1, 2].map(|k| (f64::from(p[k]) + shift[k]) as f32))
        .collect();
    // The same points (rounded as far from the origin) back near it: exact in f32.
    let near_xyz: Vec<[f32; 3]> = far_xyz
        .iter()
        .map(|p| [0, 1, 2].map(|k| (f64::from(p[k]) - shift[k]) as f32))
        .collect();
    let cam = GpuCamera::from_camera(&pinhole());
    let pose = Isometry3::from_parts(
        Translation3::new(0.03, -0.01, 0.2),
        UnitQuaternion::from_euler_angles(0.02, 0.03, -0.01),
    );
    let params = VisParams::default();
    let run = |xyz: &[[f32; 3]], pose: &Isometry3<f64>| {
        let cloud = upload(&ctx, xyz, None);
        ImagePass::new(&ctx, &cloud)
            .depth_and_visibility(&cam, pose, &params)
            .read()
    };
    let near = run(&near_xyz, &pose);
    let far = run(
        &far_xyz,
        &(Isometry3::from(Translation3::from(shift)) * pose),
    );
    assert!(near.iter().filter(|&&s| s).count() > 20_000);
    let differ = near.iter().zip(&far).filter(|(a, b)| a != b).count();
    assert_eq!(differ, 0, "{differ} of {} differ", near.len());
}

#[test]
fn empty_cloud() {
    let Some(ctx) = gpu() else { return };
    let cloud = upload(&ctx, &[], None);
    assert!(cloud.is_empty());
    let mut pass = ImagePass::new(&ctx, &cloud);
    let v = pass.depth_and_visibility(
        &GpuCamera::from_camera(&pinhole()),
        &Isometry3::identity(),
        &VisParams::default(),
    );
    assert!(v.read().is_empty());
    assert!(v.read_depth().iter().all(|d| d.is_infinite()));
}
