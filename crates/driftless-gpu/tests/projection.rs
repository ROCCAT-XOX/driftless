//! GPU projection against the CPU camera model.

use driftless_gpu::{project_points, GpuCamera, GpuContext};
use driftless_io::calib::{CameraCalib, CameraModel};
use driftless_io::camera::Camera;
use nalgebra::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3};

/// Pixel tolerance between GPU (f32) and CPU (f64).
const TOL_PX: f64 = 1e-2;

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

fn camera(model: CameraModel, w: u32, h: u32, f: f64) -> Camera {
    let calib = CameraCalib {
        k: Matrix3::new(
            f,
            0.0,
            (f64::from(w) - 1.0) / 2.0 + 0.3,
            0.0,
            f * 1.01,
            (f64::from(h) - 1.0) / 2.0 - 0.2,
            0.0,
            0.0,
            1.0,
        ),
        model,
        width: w,
        height: h,
        lidar_to_cam: None,
    };
    Camera::from_calib("test", &calib).unwrap()
}

/// Wide fisheye (`max_theta` = 100°), fisheye whose `max_theta` is limited by the
/// distortion, and a pinhole camera.
fn cameras() -> Vec<Camera> {
    let wide = camera(
        CameraModel::FisheyeKB {
            k: [0.05, -0.01, 0.002, -0.0002],
        },
        1400,
        1400,
        320.0,
    );
    let limited = camera(
        CameraModel::FisheyeKB {
            k: [-0.2, 0.01, 0.0, 0.0],
        },
        1000,
        800,
        400.0,
    );
    assert!(
        limited.max_theta < 95f64.to_radians(),
        "{}",
        limited.max_theta
    );
    vec![wide, limited, camera(CameraModel::Pinhole, 640, 480, 400.0)]
}

fn world_from_cam() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(1.2, -0.7, 0.4),
        UnitQuaternion::from_euler_angles(0.3, -1.1, 2.0),
    )
}

/// Deterministic uniform numbers in [0, 1).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// World points (as f32) of the camera-frame points, and the CPU projection of exactly
/// those f32 points.
fn world_and_cpu(
    cam: &Camera,
    pose: &Isometry3<f64>,
    p_cam: &[Vector3<f64>],
) -> (Vec<[f32; 3]>, Vec<Option<[f64; 2]>>) {
    let xyz: Vec<[f32; 3]> = p_cam
        .iter()
        .map(|p| {
            let w = pose * Point3::from(*p);
            [w.x as f32, w.y as f32, w.z as f32]
        })
        .collect();
    let cpu = xyz
        .iter()
        .map(|w| {
            let w = Point3::new(f64::from(w[0]), f64::from(w[1]), f64::from(w[2]));
            cam.project(&(pose.inverse() * w).coords)
        })
        .collect();
    (xyz, cpu)
}

fn theta(p: &Vector3<f64>) -> f64 {
    p.x.hypot(p.y).atan2(p.z)
}

fn near_border(cam: &Camera, uv: [f64; 2]) -> bool {
    uv[0].abs() < TOL_PX
        || uv[1].abs() < TOL_PX
        || (uv[0] - f64::from(cam.width)).abs() < TOL_PX
        || (uv[1] - f64::from(cam.height)).abs() < TOL_PX
}

#[test]
fn projection_matches_cpu() {
    let Some(ctx) = gpu() else { return };
    let pose = world_from_cam();
    let mut rng = Lcg(42);
    for cam in cameras() {
        let p_cam: Vec<Vector3<f64>> = (0..10_000)
            .map(|_| {
                Vector3::new(
                    rng.next() * 8.0 - 4.0,
                    rng.next() * 8.0 - 4.0,
                    rng.next() * 4.0 + 0.02,
                )
            })
            .collect();
        let (xyz, cpu) = world_and_cpu(&cam, &pose, &p_cam);
        let gpu = project_points(&ctx, &GpuCamera::from_camera(&cam), &pose, &xyz);
        assert_eq!(gpu.len(), xyz.len());
        let (mut valid, mut max_err, mut borderline) = (0, 0f64, 0);
        for (i, (c, g)) in cpu.iter().zip(&gpu).enumerate() {
            match (c, g) {
                (Some(c), Some(g)) => {
                    valid += 1;
                    let err = (c[0] - f64::from(g[0])).hypot(c[1] - f64::from(g[1]));
                    max_err = max_err.max(err);
                    assert!(
                        err <= TOL_PX,
                        "{}: Punkt {i}: CPU {c:?}, GPU {g:?}",
                        cam.name
                    );
                }
                (None, None) => {}
                (c, g) => {
                    // Only allowed right at the image border or at `max_theta`.
                    let uv = c.unwrap_or_else(|| g.unwrap().map(f64::from));
                    let at_rim = (theta(&p_cam[i]) - cam.max_theta).abs() < 1e-5;
                    assert!(
                        near_border(&cam, uv) || at_rim,
                        "Punkt {i}: CPU {c:?}, GPU {g:?}"
                    );
                    borderline += 1;
                }
            }
        }
        eprintln!(
            "{:?}: {valid} gültig, größte Abweichung {max_err:.2e} px, {borderline} am Rand",
            cam.model
        );
        assert!(valid >= 1000, "nur {valid} gültige Punkte");
        assert!(borderline <= 5, "{borderline} Abweichungen am Rand");
    }
}

#[test]
fn projection_matches_cpu_at_rim() {
    let Some(ctx) = gpu() else { return };
    let pose = world_from_cam();
    for cam in cameras() {
        let mut p_cam = Vec::new();
        let mut expect_valid = Vec::new();
        for a in 0..36 {
            let phi = f64::from(a) * 10f64.to_radians();
            for dist in [0.5, 3.0] {
                let dir =
                    |t: f64| dist * Vector3::new(t.sin() * phi.cos(), t.sin() * phi.sin(), t.cos());
                if matches!(cam.model, CameraModel::FisheyeKB { .. }) {
                    p_cam.push(dir(cam.max_theta - 1e-4));
                    expect_valid.push(true);
                    p_cam.push(dir(cam.max_theta + 1e-4));
                    expect_valid.push(false);
                }
                // Behind the camera.
                p_cam.push(dir(std::f64::consts::PI - 0.01));
                expect_valid.push(false);
                p_cam.push(dir(cam.max_theta.max(std::f64::consts::FRAC_PI_2) + 0.2));
                expect_valid.push(false);
            }
        }
        p_cam.push(Vector3::new(0.0, 0.0, 2.0));
        expect_valid.push(true);
        p_cam.push(Vector3::new(1e-3, 0.0, -2.0));
        expect_valid.push(false);
        let (xyz, cpu) = world_and_cpu(&cam, &pose, &p_cam);
        let gpu = project_points(&ctx, &GpuCamera::from_camera(&cam), &pose, &xyz);
        for (i, ((c, g), e)) in cpu.iter().zip(&gpu).zip(&expect_valid).enumerate() {
            assert_eq!(
                c.is_some(),
                *e,
                "{}: CPU Punkt {i} {:?}",
                cam.name,
                p_cam[i]
            );
            assert_eq!(
                g.is_some(),
                c.is_some(),
                "{:?}: Punkt {i} θ={} max={}: CPU {c:?}, GPU {g:?}",
                cam.model,
                theta(&p_cam[i]),
                cam.max_theta
            );
            if let (Some(c), Some(g)) = (c, g) {
                let err = (c[0] - f64::from(g[0])).hypot(c[1] - f64::from(g[1]));
                assert!(err <= TOL_PX, "Punkt {i}: CPU {c:?}, GPU {g:?}");
            }
        }
    }
}

#[test]
fn projection_spans_several_dispatches() {
    let Some(ctx) = gpu() else { return };
    let cam = &cameras()[0];
    let pose = world_from_cam();
    let p_cam: Vec<Vector3<f64>> = (0..7)
        .map(|i| Vector3::new(f64::from(i) * 0.3 - 1.0, 0.2, 1.5))
        .chain([Vector3::new(0.0, 0.0, -1.0)])
        .collect();
    let (base, cpu) = world_and_cpu(cam, &pose, &p_cam);
    // More points than one dispatch takes (2^20), not a multiple of the pattern.
    let n = (1 << 20) + 13;
    let xyz: Vec<[f32; 3]> = base.iter().copied().cycle().take(n).collect();
    let gpu = project_points(&ctx, &GpuCamera::from_camera(cam), &pose, &xyz);
    assert_eq!(gpu.len(), n);
    for (i, g) in gpu.iter().enumerate() {
        let c = cpu[i % cpu.len()];
        assert_eq!(g.is_some(), c.is_some(), "Punkt {i}");
        if let (Some(c), Some(g)) = (c, g) {
            let err = (c[0] - f64::from(g[0])).hypot(c[1] - f64::from(g[1]));
            assert!(err <= TOL_PX, "Punkt {i}: CPU {c:?}, GPU {g:?}");
        }
    }
    assert!(project_points(&ctx, &GpuCamera::from_camera(cam), &pose, &[]).is_empty());
}
