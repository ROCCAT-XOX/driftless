use driftless_io::msg::{ImuSample, LidarFrame, LidarPoint};
use driftless_map::{build_dense, write_las, write_ply, Cloud, DenseConfig, MapError, RaySource};
use driftless_odom::testutil::{imu_samples, lidar_frames, room_scene, Aabb, Trajectory, Walk};
use driftless_odom::{run_odometry, OdomConfig};
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, Vector3};
use driftless_traj::Pose;
use std::path::PathBuf;

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("driftless-map-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

struct RoomScan {
    frames: Vec<LidarFrame>,
    imu: Vec<ImuSample>,
    cfg: OdomConfig,
    /// True body pose at the end of every frame the odometry uses.
    truth: Vec<Pose>,
}

fn pose(t: f64, iso: &Isometry3<f64>) -> Pose {
    Pose {
        t,
        p: iso.translation.vector,
        q: iso.rotation,
    }
}

/// Walk through the synthetic room with a slightly offset LiDAR; the "corrected" poses are
/// the true poses at the odometry's frame times.
fn room_scan(duration: f64) -> RoomScan {
    let walk = Walk {
        still: 1.0,
        speed: 0.5,
    };
    let lidar_to_imu = Isometry3::translation(0.05, 0.0, 0.1);
    let frames = lidar_frames(&room_scene(), &walk, 0.0, duration, 360, &lidar_to_imu);
    let imu = imu_samples(&walk, -0.5, duration + 0.5, 200.0);
    let cfg = OdomConfig {
        lidar_to_imu,
        ..OdomConfig::default()
    };
    let odom = run_odometry(frames.clone().into_iter(), &imu, &cfg);
    let truth = odom
        .poses
        .iter()
        .map(|p| pose(p.t, &walk.pose(p.t)))
        .collect();
    RoomScan {
        frames,
        imu,
        cfg,
        truth,
    }
}

fn dense(scan: &RoomScan, corrected: &[Pose]) -> Result<(Cloud, RaySource), MapError> {
    build_dense(
        scan.frames.clone().into_iter(),
        &scan.imu,
        &scan.cfg,
        corrected,
        &DenseConfig::default(),
    )
    .map(|(cloud, rays, _)| (cloud, rays))
}

/// Distance from `p` to the nearest surface of the box (inside or outside).
fn box_distance(b: &Aabb, p: &Vector3<f64>) -> f64 {
    let outside = Vector3::from_fn(|i, _| (b.min[i] - p[i]).max(p[i] - b.max[i]).max(0.0));
    if outside.norm() > 0.0 {
        return outside.norm();
    }
    (0..3)
        .map(|i| (p[i] - b.min[i]).min(b.max[i] - p[i]))
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn dense_reproduces_room_geometry() {
    let scan = room_scan(6.0);
    let (cloud, _, stats) = build_dense(
        scan.frames.clone().into_iter(),
        &scan.imu,
        &scan.cfg,
        &scan.truth,
        &DenseConfig::default(),
    )
    .unwrap();
    assert_eq!(stats.frames, scan.truth.len());
    assert!(cloud.len() > 20_000, "only {} points", cloud.len());
    assert_eq!(stats.points, cloud.len());
    for col in [cloud.intensity.len(), cloud.t.len(), cloud.frame.len()] {
        assert_eq!(col, cloud.xyz.len());
    }
    assert!(cloud.rgb.is_none() && cloud.colored.is_none());
    assert!(cloud.intensity.iter().all(|&i| i == 10.0));
    for (&f, &t) in cloud.frame.iter().zip(&cloud.t) {
        assert_eq!(t, scan.truth[f as usize].t, "time = frame time");
    }

    let scene = room_scene();
    let near = cloud
        .xyz
        .iter()
        .filter(|p| {
            let p = Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            let d = scene
                .boxes
                .iter()
                .map(|b| box_distance(b, &p))
                .fold(box_distance(&scene.room, &p), f64::min);
            d < 0.03
        })
        .count();
    let share = near as f64 / cloud.len() as f64;
    assert!(share >= 0.95, "only {:.1} % within 3 cm", 100.0 * share);
}

#[test]
fn dense_ray_source_per_frame() {
    let scan = room_scan(2.0);
    let (cloud, rays) = dense(&scan, &scan.truth).unwrap();
    let frames = scan.truth.len();
    assert_eq!(rays.sensor.len(), frames);
    assert_eq!(rays.frame_start.len(), frames + 1);
    assert_eq!(rays.frame_start[0], 0);
    assert_eq!(*rays.frame_start.last().unwrap() as usize, rays.ends.len());
    assert!(rays.frame_start.windows(2).all(|w| w[0] <= w[1]));
    // 16 rings x 360 steps per frame, every 4th point; all inside the 30 m range.
    let per_frame = (rays.frame_start[2] - rays.frame_start[1]) as usize;
    assert_eq!(per_frame, 16 * 360 / 4);
    assert!(cloud.len() > per_frame);
    // The sensor sits at the corrected pose times the LiDAR mount.
    for (s, p) in rays.sensor.iter().zip(&scan.truth) {
        let iso = Isometry3::from_parts(Translation3::from(p.p), p.q);
        let want = iso * Point3::from(scan.cfg.lidar_to_imu.translation.vector);
        let got = Vector3::new(f64::from(s[0]), f64::from(s[1]), f64::from(s[2]));
        assert!((got - want.coords).norm() < 1e-5);
    }
}

fn frame(stamp: f64, xyz: [f32; 3], intensity: f32) -> LidarFrame {
    LidarFrame {
        stamp,
        points: vec![LidarPoint {
            xyz,
            intensity,
            ring: 0,
            t_rel: 0.0,
        }],
        dropped: 0,
    }
}

/// Four frames of a still device see the same 1 cm voxel from 3 m and (corrected pose moved
/// 1 m closer) from 2 m: the nearer point wins, a tie keeps the earlier frame.
#[test]
fn dense_voxel_keeps_nearest() {
    let still = Walk {
        still: 100.0,
        speed: 0.0,
    };
    let imu = imu_samples(&still, 0.0, 3.0, 200.0);
    let p = 0.011f32;
    let frames = vec![
        frame(1.5, [3.011, p, p], 1.0),
        frame(1.6, [2.013, p, p], 2.0),
        frame(1.7, [3.015, p, p], 3.0),
        frame(1.8, [2.013, p, p], 4.0),
    ];
    let corrected: Vec<Pose> = [(1.5, 0.0), (1.6, 1.0), (1.7, 0.0), (1.8, 1.0)]
        .iter()
        .map(|&(t, x)| pose(t, &Isometry3::translation(x, 0.0, 0.0)))
        .collect();
    let (cloud, rays, stats) = build_dense(
        frames.into_iter(),
        &imu,
        &OdomConfig::default(),
        &corrected,
        &DenseConfig::default(),
    )
    .unwrap();
    assert_eq!(stats.frames, 4);
    assert_eq!(cloud.len(), 1);
    assert_eq!(cloud.intensity, vec![2.0]);
    assert_eq!(cloud.frame, vec![1]);
    assert_eq!(cloud.t, vec![1.6]);
    assert!((cloud.xyz[0][0] - 3.013).abs() < 1e-5);
    // Frame f takes points f mod 4, f mod 4 + 4, ...: of these one-point frames only frame 0
    // gives a ray.
    assert_eq!(rays.ends.len(), 1);
    assert_eq!(rays.frame_start, vec![0, 1, 1, 1, 1]);
}

/// With `ray_step` 4, frame f takes points f mod 4, f mod 4 + 4, ...: a step that shares a
/// factor with the ring count does not miss the same rings in every frame.
#[test]
fn dense_ray_subset_rotates() {
    let still = Walk {
        still: 100.0,
        speed: 0.0,
    };
    let imu = imu_samples(&still, 0.0, 3.0, 200.0);
    let point = |k: usize| [2.0 + 0.5 * k as f32, 0.3, 0.1];
    let frames: Vec<LidarFrame> = (0..5)
        .map(|f| LidarFrame {
            stamp: 1.5 + 0.1 * f as f64,
            points: (0..4)
                .map(|k| LidarPoint {
                    xyz: point(k),
                    intensity: 1.0,
                    ring: k as u16,
                    t_rel: 0.0,
                })
                .collect(),
            dropped: 0,
        })
        .collect();
    let corrected: Vec<Pose> = (0..5)
        .map(|f| pose(1.5 + 0.1 * f as f64, &Isometry3::identity()))
        .collect();
    let (_, rays, _) = build_dense(
        frames.into_iter(),
        &imu,
        &OdomConfig::default(),
        &corrected,
        &DenseConfig::default(),
    )
    .unwrap();
    assert_eq!(rays.frame_start, vec![0, 1, 2, 3, 4, 5]);
    for (f, end) in rays.ends.iter().enumerate() {
        let want = point(f % 4);
        assert!(
            (0..3).all(|a| (end[a] - want[a]).abs() < 1e-4),
            "frame {f}: {end:?}"
        );
    }
}

/// Every dense point keeps the intensity of its own return, also when the odometry and the
/// dense range filter drop points around it (bad time, too close, too far).
#[test]
fn dense_keeps_intensity_of_each_return() {
    let still = Walk {
        still: 100.0,
        speed: 0.0,
    };
    let imu = imu_samples(&still, 0.0, 3.0, 200.0);
    let lp = |xyz: [f32; 3], intensity: f32, t_rel: f64| LidarPoint {
        xyz,
        intensity,
        ring: 0,
        t_rel,
    };
    // Kept returns: distinct voxels, intensity 1, 2, 3, ... in order.
    let kept = |frame: usize, k: usize| {
        let i = (frame * 10 + k) as f32;
        ([1.0 + 0.1 * i, 0.5 + 0.03 * i, 0.2], 1.0 + i)
    };
    let mut frames = Vec::new();
    for f in 0..3 {
        let mut points = vec![
            lp([2.0, 0.0, 0.0], 99.0, f64::NAN), // no valid time: dropped by the odometry
            lp([0.2, 0.0, 0.0], 98.0, 0.0),      // inside the odometry's min range
        ];
        for k in 0..6 {
            let (xyz, i) = kept(f, k);
            points.push(lp(xyz, i, 0.0));
            if k == 2 {
                points.push(lp([40.0, 1.0, 0.0], 97.0, 0.0)); // beyond the dense max range
            }
        }
        frames.push(LidarFrame {
            stamp: 1.5 + 0.1 * f as f64,
            points,
            dropped: 0,
        });
    }
    let corrected: Vec<Pose> = (0..3)
        .map(|f| pose(1.5 + 0.1 * f as f64, &Isometry3::identity()))
        .collect();
    let (cloud, _, _) = build_dense(
        frames.into_iter(),
        &imu,
        &OdomConfig::default(),
        &corrected,
        &DenseConfig::default(),
    )
    .unwrap();
    assert_eq!(cloud.len(), 18);
    for f in 0..3 {
        for k in 0..6 {
            let (xyz, want) = kept(f, k);
            let j = cloud
                .xyz
                .iter()
                .position(|p| (0..3).all(|a| (p[a] - xyz[a]).abs() < 1e-4))
                .unwrap_or_else(|| panic!("return {f}/{k} missing"));
            assert_eq!(cloud.intensity[j], want, "return {f}/{k}");
            assert_eq!(cloud.frame[j], f as u32);
        }
    }
}

#[test]
fn dense_rejects_pose_count_mismatch() {
    let scan = room_scan(2.0);
    let short = &scan.truth[..scan.truth.len() - 1];
    let msg = dense(&scan, short).unwrap_err().to_string();
    assert!(msg.contains("Laufbahn passt nicht zum Scan"), "{msg}");
    assert!(
        msg.contains(&format!(
            "{} Posen, aber {} Frames",
            short.len(),
            scan.truth.len()
        )),
        "{msg}"
    );
    assert!(msg.contains("driftless slam"), "{msg}");

    // Same count, but from another scan: the times do not match.
    let shifted: Vec<Pose> = scan
        .truth
        .iter()
        .map(|p| Pose {
            t: p.t + 0.5,
            ..p.clone()
        })
        .collect();
    let msg = dense(&scan, &shifted).unwrap_err().to_string();
    assert!(msg.contains("Laufbahn passt nicht zum Scan"), "{msg}");
}

#[test]
fn dense_rejects_bad_config() {
    let scan = room_scan(1.5);
    for cfg in [
        DenseConfig {
            voxel: 0.0,
            ..DenseConfig::default()
        },
        DenseConfig {
            ray_step: 0,
            ..DenseConfig::default()
        },
        DenseConfig {
            min_range: 40.0,
            ..DenseConfig::default()
        },
    ] {
        let r = build_dense(
            scan.frames.clone().into_iter(),
            &scan.imu,
            &scan.cfg,
            &scan.truth,
            &cfg,
        );
        assert!(matches!(r, Err(MapError::Config(_))));
    }
}

#[test]
fn dense_is_deterministic() {
    let scan = room_scan(3.0);
    let (a, ra) = dense(&scan, &scan.truth).unwrap();
    let (b, rb) = dense(&scan, &scan.truth).unwrap();
    assert_eq!(a, b);
    assert_eq!(ra, rb);
    let dir = temp_dir("determinism");
    for (name, cloud) in [("a", &a), ("b", &b)] {
        write_las(&dir.join(format!("{name}.las")), cloud).unwrap();
        write_ply(&dir.join(format!("{name}.ply")), cloud).unwrap();
    }
    for ext in ["las", "ply"] {
        let read = |n: &str| std::fs::read(dir.join(format!("{n}.{ext}"))).unwrap();
        assert!(read("a") == read("b"), "{ext} differs between runs");
    }
    std::fs::remove_dir_all(&dir).unwrap();
}
