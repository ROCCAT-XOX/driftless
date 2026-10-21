//! Points measured through windows: a scene of plates (room with a window band, boxes
//! outside, an open door into a second room), scanned with an ideal spinning LiDAR.

use driftless_map::{clean, find_through, CleanConfig, Cloud, RaySource, RemovalReason};
use driftless_odom::testutil::{aabb, Aabb, Scene};
use driftless_traj::nalgebra::Vector3;

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

/// Floor, ceiling and the walls shared by all scenes: room 1 is x 0–6, y 0–5, room 2 is
/// x 0–6, y 5.2–9, floor at z = 0, ceiling at 3 m. The wall between them (y 5–5.2) and the
/// window wall of room 1 (x 6–6.2) are added by the scenes.
fn shell() -> Vec<Aabb> {
    vec![
        aabb([-0.2, -0.2, -0.2], [6.2, 9.2, 0.0]), // floor
        aabb([-0.2, -0.2, 3.0], [6.2, 9.2, 3.2]),  // ceiling
        aabb([-0.2, -0.2, 0.0], [0.0, 9.2, 3.0]),  // x = 0 wall
        aabb([-0.2, -0.2, 0.0], [6.2, 0.0, 3.0]),  // y = 0 wall
        aabb([6.0, 5.2, 0.0], [6.2, 9.2, 3.0]),    // x = 6 wall of room 2
        aabb([-0.2, 9.0, 0.0], [6.2, 9.2, 3.0]),   // y = 9 wall
        aabb([1.0, 3.5, 0.0], [2.0, 4.5, 0.75]),   // table
        aabb([0.0, 1.0, 0.0], [0.6, 2.5, 2.0]),    // cabinet
    ]
}

/// Window band in the x = 6 wall of room 1: parapet up to 0.9 m, lintel from 2.1 m, open
/// for y 0.5–4.5; outside ground and boxes 8–15 m from the origin.
fn window_wall_and_outside() -> Vec<Aabb> {
    vec![
        aabb([6.0, 0.0, 0.0], [6.2, 5.0, 0.9]),      // parapet
        aabb([6.0, 0.0, 2.1], [6.2, 5.0, 3.0]),      // lintel
        aabb([6.0, -0.2, 0.9], [6.2, 0.5, 2.1]),     // jamb
        aabb([6.0, 4.5, 0.9], [6.2, 5.2, 2.1]),      // jamb
        aabb([6.2, -10.0, -0.2], [20.0, 15.0, 0.0]), // ground outside
        aabb([8.0, 1.0, 0.0], [9.0, 2.0, 2.0]),
        aabb([11.0, 3.0, 0.0], [12.0, 4.5, 2.5]),
        aabb([14.0, -1.0, 0.0], [15.0, 1.5, 3.0]),
    ]
}

/// A scene from plates; the "room" is far away, rays reaching it are lost.
fn scene(boxes: Vec<Aabb>) -> Scene {
    Scene {
        room: aabb([-1000.0, -1000.0, -1000.0], [1000.0, 1000.0, 1000.0]),
        boxes,
    }
}

/// Room 1 with the window band, and an open door (x 2–3, up to 2.1 m) into room 2.
fn door_scene() -> Scene {
    let mut b = shell();
    b.extend(window_wall_and_outside());
    b.extend([
        aabb([-0.2, 5.0, 0.0], [2.0, 5.2, 3.0]),
        aabb([3.0, 5.0, 0.0], [6.2, 5.2, 3.0]),
        aabb([2.0, 5.0, 2.1], [3.0, 5.2, 3.0]), // door lintel
    ]);
    scene(b)
}

/// Room 1 with the window band; the wall to room 2 has a glass band (x 0.5–2.5, 0.9–2.1 m)
/// and a door (x 4–5).
fn glass_scene() -> Scene {
    let mut b = shell();
    b.extend(window_wall_and_outside());
    b.extend([
        aabb([-0.2, 5.0, 0.0], [0.5, 5.2, 3.0]),
        aabb([0.5, 5.0, 0.0], [2.5, 5.2, 0.9]), // parapet
        aabb([0.5, 5.0, 2.1], [2.5, 5.2, 3.0]), // lintel
        aabb([2.5, 5.0, 0.0], [4.0, 5.2, 3.0]),
        aabb([4.0, 5.0, 2.1], [5.0, 5.2, 3.0]), // door lintel
        aabb([5.0, 5.0, 0.0], [6.2, 5.2, 3.0]),
    ]);
    scene(b)
}

/// Sensor positions, one per frame, along the polyline `path` (x, y) at 1.2 m height.
fn along(path: &[[f64; 2]], frames: usize) -> Vec<Vector3<f64>> {
    let seg: Vec<f64> = path
        .windows(2)
        .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
        .collect();
    let total: f64 = seg.iter().sum();
    (0..frames)
        .map(|f| {
            let mut s = total * f as f64 / (frames - 1) as f64;
            let mut i = 0;
            while i + 1 < seg.len() && s > seg[i] {
                s -= seg[i];
                i += 1;
            }
            let a = (s / seg[i]).min(1.0);
            let (p, q) = (path[i], path[i + 1]);
            Vector3::new(
                p[0] + a * (q[0] - p[0]),
                p[1] + a * (q[1] - p[1]),
                1.2 + 0.03 * (f as f64).sin(),
            )
        })
        .collect()
}

/// Cloud (every hit, 3 mm range noise) and rays (every third hit) of an ideal LiDAR at the
/// `sensors`: 1° azimuth steps, 23 rings from −55° to +55°, rotated a little per frame.
fn scan(scene: &Scene, sensors: &[Vector3<f64>]) -> (Cloud, RaySource) {
    let mut rng = Lcg(11);
    let mut cloud = Cloud::default();
    let mut rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    for (f, o) in sensors.iter().enumerate() {
        let spin = 0.37 * f as f64;
        let mut n = 0;
        for j in 0..360 {
            let az = (j as f64 + spin).to_radians();
            for ring in 0..23 {
                let el = (-55.0 + 5.0 * ring as f64 + 0.5 * spin.sin()).to_radians();
                let d = Vector3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
                let r = scene.raycast(o, &d);
                if !(r.is_finite() && r < 100.0) {
                    continue;
                }
                let p = o + d * (r + 0.006 * (rng.next() - 0.5));
                let p = [p.x as f32, p.y as f32, p.z as f32];
                cloud.xyz.push(p);
                cloud.intensity.push(10.0);
                cloud.t.push(0.1 * f as f64);
                cloud.frame.push(f as u32);
                if n % 3 == 0 {
                    rays.ends.push(p);
                }
                n += 1;
            }
        }
        rays.sensor.push([o.x as f32, o.y as f32, o.z as f32]);
        rays.frame_start.push(rays.ends.len() as u32);
    }
    (cloud, rays)
}

/// Removed share (and count) of the points `which` selects.
fn share(cloud: &Cloud, flags: &[bool], which: impl Fn(&[f32; 3]) -> bool) -> (f64, usize) {
    let (mut n, mut removed) = (0usize, 0usize);
    for (p, &f) in cloud.xyz.iter().zip(flags) {
        if which(p) {
            n += 1;
            removed += usize::from(f);
        }
    }
    (removed as f64 / n.max(1) as f64, n)
}

/// Walk through room 1 only.
fn room1_walk() -> Vec<Vector3<f64>> {
    along(&[[1.5, 1.0], [4.5, 3.5], [2.5, 2.5]], 40)
}

fn outside(p: &[f32; 3]) -> bool {
    p[0] > 6.4
}

fn in_room1(p: &[f32; 3]) -> bool {
    p[0] < 6.1 && p[1] < 5.1
}

fn in_room2(p: &[f32; 3]) -> bool {
    p[0] < 6.1 && p[1] > 5.1
}

#[test]
fn removes_points_seen_through_window() {
    let (cloud, rays) = scan(&door_scene(), &room1_walk());
    let through = find_through(&cloud, &rays, &CleanConfig::default());
    assert_eq!(through.len(), cloud.len());
    let (out, n_out) = share(&cloud, &through, outside);
    let (room, n_room) = share(&cloud, &through, in_room1);
    assert!(
        n_out > 3000 && n_room > 50_000,
        "{n_out} outside, {n_room} room"
    );
    assert!(
        out > 0.9,
        "only {:.1} % of {n_out} outside points removed",
        100.0 * out
    );
    assert!(
        room < 0.005,
        "{:.2} % of {n_room} room points removed",
        100.0 * room
    );

    // In `clean`, with their own reason; off, nothing is removed for it.
    let (_, removed, reasons, stats) =
        clean(cloud.clone(), &rays, &CleanConfig::default()).unwrap();
    assert!(stats.through > 0);
    assert_eq!(
        reasons
            .iter()
            .filter(|&&r| r == RemovalReason::Through)
            .count(),
        stats.through
    );
    assert_eq!(stats.ghosts + stats.through + stats.outliers, removed.len());
    assert_eq!(RemovalReason::Through as u8, 3);
    let off = CleanConfig {
        through: false,
        ..CleanConfig::default()
    };
    let (_, _, reasons, stats) = clean(cloud, &rays, &off).unwrap();
    assert_eq!(stats.through, 0);
    assert!(!reasons.contains(&RemovalReason::Through));
}

#[test]
fn keeps_room_seen_through_door() {
    let (cloud, rays) = scan(&door_scene(), &room1_walk());
    let through = find_through(&cloud, &rays, &CleanConfig::default());
    let (r2, n) = share(&cloud, &through, in_room2);
    assert!(n > 2000, "only {n} points of room 2");
    assert!(
        r2 < 0.01,
        "{:.2} % of {n} points of room 2 removed",
        100.0 * r2
    );
}

#[test]
fn keeps_room_behind_glass_once_entered() {
    // Through room 1, then through the door into room 2 and along it.
    let walk = along(
        &[[1.5, 1.5], [4.5, 3.0], [4.5, 5.1], [4.5, 7.5], [1.5, 7.5]],
        60,
    );
    let (cloud, rays) = scan(&glass_scene(), &walk);
    let through = find_through(&cloud, &rays, &CleanConfig::default());
    let (r2, n2) = share(&cloud, &through, in_room2);
    let (r1, n1) = share(&cloud, &through, in_room1);
    assert!(n2 > 20_000, "only {n2} points of room 2");
    assert!(
        r2 < 0.01,
        "{:.2} % of {n2} points of room 2 removed",
        100.0 * r2
    );
    assert!(
        r1 < 0.01,
        "{:.2} % of {n1} points of room 1 removed",
        100.0 * r1
    );
    // The room is seen through the glass before it is entered.
    let before: Vec<bool> = cloud
        .frame
        .iter()
        .zip(&cloud.xyz)
        .map(|(&f, p)| f < 20 && in_room2(p))
        .collect();
    assert!(before.iter().filter(|&&b| b).count() > 1000);
}

#[test]
fn through_is_deterministic() {
    let (cloud, rays) = scan(&door_scene(), &room1_walk());
    let a = find_through(&cloud, &rays, &CleanConfig::default());
    let b = find_through(&cloud, &rays, &CleanConfig::default());
    assert!(a.iter().any(|&t| t));
    assert_eq!(a, b);
    let a = clean(cloud.clone(), &rays, &CleanConfig::default()).unwrap();
    let b = clean(cloud, &rays, &CleanConfig::default()).unwrap();
    assert_eq!(a, b);
}

#[test]
fn rejects_bad_through_config() {
    let cloud = Cloud::default();
    let rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    for cfg in [
        CleanConfig {
            window_reach: 0.0,
            ..CleanConfig::default()
        },
        CleanConfig {
            through_beyond: f64::NAN,
            ..CleanConfig::default()
        },
    ] {
        assert!(clean(cloud.clone(), &rays, &cfg).is_err(), "{cfg:?}");
    }
}
