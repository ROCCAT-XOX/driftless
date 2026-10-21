use driftless_io::msg::{ImuSample, LidarFrame, LidarPoint};
use driftless_map::{
    build_dense, clean, find_ghosts, find_outliers, write_removed_las, CleanConfig, CleanStats,
    Cloud, DenseConfig, MapError, RaySource, RemovalReason, SmoothConfig, SmoothStats,
};
use driftless_odom::testutil::{aabb, imu_samples, room_scene, Aabb, Scene, Trajectory, Walk};
use driftless_odom::{run_odometry, OdomConfig};
use driftless_traj::nalgebra::{Isometry3, Vector3};
use driftless_traj::Pose;

fn pose(t: f64, iso: &Isometry3<f64>) -> Pose {
    Pose {
        t,
        p: iso.translation.vector,
        q: iso.rotation,
    }
}

fn v(p: &[f32; 3]) -> Vector3<f64> {
    Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
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

/// Whether `p` lies inside the box grown by `margin` on every side.
fn within(b: &Aabb, p: &Vector3<f64>, margin: f64) -> bool {
    (0..3).all(|i| p[i] >= b.min[i] - margin && p[i] <= b.max[i] + margin)
}

/// Distance from `p` to the nearest surface of the static scene.
fn scene_distance(scene: &Scene, p: &Vector3<f64>) -> f64 {
    scene
        .boxes
        .iter()
        .map(|b| box_distance(b, p))
        .fold(box_distance(&scene.room, p), f64::min)
}

/// A scene that changes over time: static geometry plus boxes placed by `moving(t)`.
struct World<'a> {
    fixed: Scene,
    moving: &'a dyn Fn(f64) -> Vec<Aabb>,
}

impl World<'_> {
    fn at(&self, t: f64) -> Scene {
        let mut boxes = self.fixed.boxes.clone();
        boxes.extend((self.moving)(t));
        Scene {
            room: self.fixed.room,
            boxes,
        }
    }
}

/// 16-ring spinning LiDAR like `testutil::lidar_frame`, but every azimuth step sees the
/// world at its own capture time.
fn lidar_frame(
    world: &World,
    traj: &dyn Trajectory,
    stamp: f64,
    az_steps: usize,
    mount: &Isometry3<f64>,
) -> LidarFrame {
    let mut points = Vec::with_capacity(16 * az_steps);
    for j in 0..az_steps {
        let t_rel = 0.1 * j as f64 / az_steps as f64;
        let scene = world.at(stamp + t_rel);
        let sensor = traj.pose(stamp + t_rel) * mount;
        let az = 2.0 * std::f64::consts::PI * j as f64 / az_steps as f64;
        for ring in 0..16u16 {
            let el = (-15.0 + 2.0 * f64::from(ring)).to_radians();
            let d_s = Vector3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            let r = scene.raycast(&sensor.translation.vector, &(sensor.rotation * d_s));
            if r.is_finite() && r < 100.0 {
                let p = d_s * r;
                points.push(LidarPoint {
                    xyz: [p.x as f32, p.y as f32, p.z as f32],
                    intensity: 10.0,
                    ring,
                    t_rel,
                });
            }
        }
    }
    LidarFrame {
        stamp,
        points,
        dropped: 0,
    }
}

/// Dense cloud and rays of a walk through `world`, placed with the true poses.
fn scan(world: &World, walk: &Walk, duration: f64) -> (Cloud, RaySource) {
    let mount = Isometry3::translation(0.05, 0.0, 0.1);
    let n = (duration * 10.0).floor() as usize;
    let frames: Vec<LidarFrame> = (0..n)
        .map(|i| lidar_frame(world, walk, i as f64 * 0.1, 360, &mount))
        .collect();
    let imu: Vec<ImuSample> = imu_samples(walk, -0.5, duration + 0.5, 200.0);
    let cfg = OdomConfig {
        lidar_to_imu: mount,
        ..OdomConfig::default()
    };
    let odom = run_odometry(frames.clone().into_iter(), &imu, &cfg);
    let truth: Vec<Pose> = odom
        .poses
        .iter()
        .map(|p| pose(p.t, &walk.pose(p.t)))
        .collect();
    let (cloud, rays, _) = build_dense(
        frames.into_iter(),
        &imu,
        &cfg,
        &truth,
        &DenseConfig::default(),
    )
    .unwrap();
    (cloud, rays)
}

const WALK: Walk = Walk {
    still: 1.0,
    speed: 0.3,
};

/// Person-sized box 4.5 m ahead that walks 3 m sideways (y −3 → 0) between 1 s and 6 s.
fn person(t: f64) -> Vec<Aabb> {
    let y = -3.0 + 3.0 * ((t - 1.0) / 5.0).clamp(0.0, 1.0);
    vec![aabb([4.5, y, -1.2], [5.0, y + 0.5, 0.5])]
}

fn moving_scan() -> (Cloud, RaySource) {
    let world = World {
        fixed: room_scene(),
        moving: &person,
    };
    scan(&world, &WALK, 8.0)
}

fn only_ghosts() -> CleanConfig {
    CleanConfig {
        outliers: false,
        through: false,
        ..CleanConfig::default()
    }
}

#[test]
fn removes_person_walking_through() {
    let (cloud, rays) = moving_scan();
    let ghost = find_ghosts(&cloud, &rays, &CleanConfig::default());
    assert_eq!(ghost.len(), cloud.len());

    let fixed = room_scene();
    // Where the box ever was, and where it stays at the end.
    let swept = aabb([4.5, -3.0, -1.2], [5.0, 0.5, 0.5]);
    let last = person(100.0)[0];
    let (mut old, mut old_removed, mut wall, mut wall_removed) = (0, 0, 0, 0);
    for (p, &g) in cloud.xyz.iter().zip(&ghost) {
        let p = v(p);
        let static_d = scene_distance(&fixed, &p);
        if static_d < 0.03 {
            wall += 1;
            wall_removed += usize::from(g);
        } else if static_d > 0.05 && within(&swept, &p, 0.03) && !within(&last, &p, 0.05) {
            old += 1;
            old_removed += usize::from(g);
        }
    }
    assert!(old > 500, "only {old} points at the box's old positions");
    let old_share = old_removed as f64 / old as f64;
    assert!(
        old_share > 0.9,
        "only {:.1} % of {old} old box points removed",
        100.0 * old_share
    );
    let wall_share = wall_removed as f64 / wall as f64;
    assert!(
        wall_share < 0.01,
        "{:.2} % of {wall} wall points removed",
        100.0 * wall_share
    );
}

#[test]
fn keeps_wall_seen_once_behind_occluder() {
    // A pillar 1.5 m ahead hides the far wall (x = 7) behind it; the walk's sideways sway
    // uncovers strips of that wall for a few frames only.
    let mut fixed = room_scene();
    fixed.boxes.push(aabb([2.0, -0.4, -1.2], [2.3, 0.4, 1.8]));
    let none = |_: f64| Vec::new();
    let world = World {
        fixed,
        moving: &none,
    };
    let (cloud, rays) = scan(&world, &WALK, 6.0);
    let ghost = find_ghosts(&cloud, &rays, &CleanConfig::default());

    let scene = world.at(0.0);
    let (mut shadow, mut shadow_removed, mut surface, mut surface_removed) = (0, 0, 0, 0);
    for (p, &g) in cloud.xyz.iter().zip(&ghost) {
        let p = v(p);
        if scene_distance(&scene, &p) < 0.03 {
            surface += 1;
            surface_removed += usize::from(g);
            if (p.x - 7.0).abs() < 0.03 && p.y.abs() < 1.0 {
                shadow += 1;
                shadow_removed += usize::from(g);
            }
        }
    }
    assert!(shadow > 20, "only {shadow} wall points behind the pillar");
    assert_eq!(
        shadow_removed, 0,
        "of {shadow} wall points behind the pillar"
    );
    assert!(
        (surface_removed as f64) < 0.01 * surface as f64,
        "{surface_removed} of {surface} surface points removed"
    );
}

/// Deterministic pseudo-random numbers in [0, 1).
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

#[test]
fn removes_isolated_outliers() {
    let none = |_: f64| Vec::new();
    let world = World {
        fixed: room_scene(),
        moving: &none,
    };
    let (mut cloud, _) = scan(&world, &WALK, 6.0);
    let scene = world.at(0.0);
    let real = cloud.len();

    // Single points in the air, at least 0.5 m from any surface.
    let mut rng = Lcg(7);
    let mut added = 0;
    while added < 60 {
        let p = Vector3::new(
            -5.0 + 12.0 * rng.next(),
            -4.0 + 9.0 * rng.next(),
            -1.2 + 3.0 * rng.next(),
        );
        let near_box = scene.boxes.iter().any(|b| within(b, &p, 0.5));
        if near_box || box_distance(&scene.room, &p) < 0.5 {
            continue;
        }
        cloud.xyz.push([p.x as f32, p.y as f32, p.z as f32]);
        cloud.intensity.push(0.0);
        cloud.t.push(0.0);
        cloud.frame.push(0);
        added += 1;
    }
    let outlier = find_outliers(&cloud, &CleanConfig::default());
    assert_eq!(outlier.len(), cloud.len());
    assert!(
        outlier[real..].iter().all(|&o| o),
        "every point in the air is removed"
    );

    // Points on box edges and room corners (two faces within 3 cm) stay.
    let near_faces = |p: &Vector3<f64>| {
        let mut faces = 0;
        for b in scene.boxes.iter().chain([&scene.room]) {
            for i in 0..3 {
                let on_span = (0..3)
                    .filter(|&j| j != i)
                    .all(|j| p[j] >= b.min[j] - 0.03 && p[j] <= b.max[j] + 0.03);
                for side in [b.min[i], b.max[i]] {
                    if on_span && (p[i] - side).abs() < 0.03 {
                        faces += 1;
                    }
                }
            }
        }
        faces
    };
    let (mut edge, mut edge_removed) = (0, 0);
    for (p, &o) in cloud.xyz[..real].iter().zip(&outlier) {
        if near_faces(&v(p)) >= 2 {
            edge += 1;
            edge_removed += usize::from(o);
        }
    }
    assert!(edge > 500, "only {edge} edge points");
    assert!(
        (edge_removed as f64) < 0.01 * edge as f64,
        "{edge_removed} of {edge} edge points removed"
    );
    let real_removed = outlier[..real].iter().filter(|&&o| o).count();
    assert!(
        (real_removed as f64) < 0.01 * real as f64,
        "{real_removed} of {real} surface points removed"
    );
}

#[test]
fn clean_is_deterministic() {
    let (cloud, rays) = moving_scan();
    let a = clean(cloud.clone(), &rays, &CleanConfig::default()).unwrap();
    let b = clean(cloud, &rays, &CleanConfig::default()).unwrap();
    assert!(a.3.ghosts > 0 && a.3.kept > 0);
    assert_eq!(a, b);
}

/// Points as sortable bit patterns, to compare clouds as sets.
fn sorted_bits(clouds: &[&Cloud]) -> Vec<([u32; 3], u32, u32)> {
    let mut out: Vec<_> = clouds
        .iter()
        .flat_map(|c| {
            c.xyz
                .iter()
                .zip(&c.intensity)
                .zip(&c.frame)
                .map(|((p, i), &f)| (p.map(f32::to_bits), i.to_bits(), f))
        })
        .collect();
    out.sort_unstable();
    out
}

#[test]
fn removed_plus_kept_equals_input() {
    let (mut cloud, rays) = moving_scan();
    cloud.rgb = Some((0..cloud.len()).map(|i| [i as u8, 0, 0]).collect());
    cloud.colored = Some((0..cloud.len()).map(|i| i % 3 == 0).collect());
    // Without the smoothing, which moves the kept points.
    let unsmoothed = CleanConfig {
        smooth: SmoothConfig {
            enabled: false,
            ..SmoothConfig::default()
        },
        ..CleanConfig::default()
    };
    let (kept, removed, reasons, stats) = clean(cloud.clone(), &rays, &unsmoothed).unwrap();
    assert_eq!(kept.len() + removed.len(), cloud.len());
    assert_eq!(
        stats,
        CleanStats {
            ghosts: stats.ghosts,
            through: stats.through,
            outliers: stats.outliers,
            kept: kept.len(),
            smooth: SmoothStats::default(),
        }
    );
    assert_eq!(stats.ghosts + stats.through + stats.outliers, removed.len());
    assert_eq!(reasons.len(), removed.len());
    assert_eq!(
        reasons
            .iter()
            .filter(|&&r| r == RemovalReason::Ghost)
            .count(),
        stats.ghosts
    );
    assert_eq!(
        reasons
            .iter()
            .filter(|&&r| r == RemovalReason::Outlier)
            .count(),
        stats.outliers
    );
    assert!(stats.ghosts > 0 && stats.outliers > 0);
    assert_eq!(sorted_bits(&[&kept, &removed]), sorted_bits(&[&cloud]));
    for c in [&kept, &removed] {
        assert_eq!(c.rgb.as_ref().unwrap().len(), c.len());
        assert_eq!(c.colored.as_ref().unwrap().len(), c.len());
    }

    // With the smoothing the same points go, as they were; the kept ones only move.
    let (kept_s, removed_s, reasons_s, stats_s) =
        clean(cloud.clone(), &rays, &CleanConfig::default()).unwrap();
    assert_eq!((removed_s, reasons_s), (removed, reasons));
    assert_eq!(kept_s.len(), kept.len());
    assert_eq!(kept_s.frame, kept.frame);
    assert!(stats_s.smooth.moved > 0);

    // Both filters off: everything stays.
    let off = CleanConfig {
        ghosts: false,
        outliers: false,
        through: false,
        ..unsmoothed.clone()
    };
    let (kept, removed, reasons, stats) = clean(cloud.clone(), &rays, &off).unwrap();
    assert_eq!(kept, cloud);
    assert!(removed.is_empty() && reasons.is_empty());
    assert_eq!(
        (stats.ghosts, stats.through, stats.outliers, stats.kept),
        (0, 0, 0, cloud.len())
    );

    // Ghosts only: the same ghosts as `find_ghosts`.
    let (_, removed, _, stats) = clean(cloud.clone(), &rays, &only_ghosts()).unwrap();
    let ghosts = find_ghosts(&cloud, &rays, &CleanConfig::default());
    assert_eq!(removed.len(), ghosts.iter().filter(|&&g| g).count());
    assert_eq!(stats.outliers, 0);
}

#[test]
fn clean_rejects_bad_config() {
    let cloud = Cloud::default();
    let rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    for cfg in [
        CleanConfig {
            ghost_voxel: 0.0,
            ..CleanConfig::default()
        },
        CleanConfig {
            ghost_ratio: f64::NAN,
            ..CleanConfig::default()
        },
        CleanConfig {
            // 0 would make every voxel no ray touches a ghost
            min_passes: 0,
            ..CleanConfig::default()
        },
        CleanConfig {
            outlier_k: 0,
            ..CleanConfig::default()
        },
        CleanConfig {
            outlier_sigma: -1.0,
            ..CleanConfig::default()
        },
    ] {
        let r = clean(cloud.clone(), &rays, &cfg);
        assert!(matches!(r, Err(MapError::Config(_))), "{cfg:?}");
    }
    let (kept, removed, _, stats) = clean(cloud, &rays, &CleanConfig::default()).unwrap();
    assert!(kept.is_empty() && removed.is_empty());
    assert_eq!(stats, CleanStats::default());
}

#[test]
fn removed_las_carries_reason() {
    let (cloud, rays) = moving_scan();
    let (_, removed, reasons, _) = clean(cloud, &rays, &CleanConfig::default()).unwrap();
    let dir = std::env::temp_dir().join(format!("driftless-removed-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("removed.las");
    write_removed_las(&path, &removed, &reasons).unwrap();
    let mut r = las::Reader::from_path(&path).unwrap();
    let got: Vec<u8> = r
        .read_all()
        .unwrap()
        .points()
        .map(|p| p.unwrap().user_data)
        .collect();
    let want: Vec<u8> = reasons.iter().map(|&r| r as u8).collect();
    assert_eq!(got, want);
    assert!(got.contains(&1) && got.contains(&2));
    std::fs::remove_dir_all(&dir).unwrap();
}
