//! Picking points of the cloud and poses of the trajectory, in physical pixels.

use driftless_app::camera::Camera;
use driftless_app::cloud::PointSet;
use driftless_app::lod::{build_octree, build_octree_with, Octree};
use driftless_app::pick::{pick_point, pick_pose, project, to_physical};
use driftless_app::synthetic::Rng;
use driftless_io::progress::Progress;
use nalgebra::Vector3;

const VIEWPORT: [f32; 2] = [1600.0, 1200.0];
const ASPECT: f32 = 1600.0 / 1200.0;
const CENTRE: [f32; 2] = [800.0, 600.0];

/// Eye at the origin, looking along +x.
fn along_x() -> Camera {
    Camera {
        target: Vector3::new(10.0, 0.0, 0.0),
        yaw: 0.0,
        pitch: 0.0,
        distance: 10.0,
        ..Camera::default()
    }
}

fn tree_of(xyz: Vec<[f32; 3]>) -> Octree {
    build_octree(&PointSet::from_xyz([0.0; 3], xyz), &Progress::new()).unwrap()
}

fn all_nodes(tree: &Octree) -> Vec<u32> {
    (0..tree.nodes.len() as u32).collect()
}

fn vertex_of(tree: &Octree, source: u32) -> u32 {
    tree.source.iter().position(|&s| s == source).unwrap() as u32
}

#[test]
fn picks_front_point() {
    // Eye at (6, 0, 0) looking along −x; both points on the central ray.
    let cam = Camera {
        target: Vector3::new(-4.0, 0.0, 0.0),
        yaw: std::f32::consts::PI,
        pitch: 0.0,
        distance: 10.0,
        ..Camera::default()
    };
    assert!((cam.eye_position() - Vector3::new(6.0, 0.0, 0.0)).norm() < 1e-5);
    let tree = tree_of(vec![[4.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
    // The far point comes first in the octree, so vertex order alone cannot give the answer.
    assert_eq!(
        tree.vertices[0].pos,
        [2.0, 0.0, 0.0],
        "precondition: far point first"
    );
    assert_eq!(tree.vertices[1].pos, [4.0, 0.0, 0.0]);
    let got = pick_point(&tree, &all_nodes(&tree), &cam, VIEWPORT, CENTRE, 4.0);
    assert_eq!(got, Some(1));
}

#[test]
fn pick_outside_radius_is_none() {
    let cam = along_x();
    let tree = tree_of(vec![[10.0, 0.0, 0.0]]);
    let drawn = all_nodes(&tree);
    let q = project(&cam.view_proj(ASPECT), [10.0, 0.0, 0.0], VIEWPORT).unwrap();
    assert!((q[0] - CENTRE[0]).abs() < 0.01 && (q[1] - CENTRE[1]).abs() < 0.01);
    assert_eq!(
        pick_point(&tree, &drawn, &cam, VIEWPORT, [806.0, 600.0], 5.0),
        None
    );
    assert_eq!(
        pick_point(&tree, &drawn, &cam, VIEWPORT, [804.0, 603.0], 5.0),
        Some(0)
    );
    // Nodes not drawn are not picked.
    assert_eq!(pick_point(&tree, &[], &cam, VIEWPORT, CENTRE, 5.0), None);
    // Nor are points behind the camera or before the near plane.
    let tree = tree_of(vec![[-10.0, 0.0, 0.0], [0.01, 0.0, 0.0]]);
    assert_eq!(
        pick_point(&tree, &all_nodes(&tree), &cam, VIEWPORT, CENTRE, 50.0),
        None
    );
}

#[test]
fn pick_is_deterministic_on_ties() {
    let cam = along_x();
    // Two points at exactly the same depth, symmetric about the central ray, among noise
    // that is out of reach of the click.
    let mut rng = Rng::new(5);
    let mut xyz: Vec<[f32; 3]> = (0..5000)
        .map(|_| {
            [
                rng.next_f32() * 40.0 - 20.0,
                5.0 + rng.next_f32() * 20.0,
                rng.next_f32(),
            ]
        })
        .collect();
    xyz.push([10.0, 0.01, 0.0]);
    xyz.push([10.0, -0.01, 0.0]);
    let tree = build_octree_with(
        &PointSet::from_xyz([0.0; 3], xyz.clone()),
        64,
        &Progress::new(),
    )
    .unwrap();
    let a = vertex_of(&tree, 5000);
    let b = vertex_of(&tree, 5001);
    let mut drawn = all_nodes(&tree);
    drawn.reverse();
    let got = pick_point(&tree, &drawn, &cam, VIEWPORT, CENTRE, 6.0);
    assert_eq!(got, Some(a.min(b)));
    drawn.reverse();
    assert_eq!(pick_point(&tree, &drawn, &cam, VIEWPORT, CENTRE, 6.0), got);
}

/// The node culling by the cone around the ray only skips work: the result is that of
/// testing every drawn vertex.
#[test]
fn pick_matches_brute_force() {
    let mut rng = Rng::new(11);
    let xyz: Vec<[f32; 3]> = (0..200_000)
        .map(|_| {
            [
                rng.next_f32() * 60.0 - 30.0,
                rng.next_f32() * 60.0 - 30.0,
                rng.next_f32() * 6.0 - 3.0,
            ]
        })
        .collect();
    let tree = tree_of(xyz);
    assert!(tree.nodes.len() > 8);
    let mut cam = Camera {
        target: Vector3::new(3.0, -2.0, 0.0),
        distance: 25.0,
        ..Camera::default()
    };
    cam.orbit(210.0, -40.0);
    let vp = cam.view_proj(ASPECT);
    let drawn = all_nodes(&tree);
    let mut hits = 0;
    for _ in 0..40 {
        let pixel = [rng.next_f32() * VIEWPORT[0], rng.next_f32() * VIEWPORT[1]];
        let radius = 1.0 + rng.next_f32() * 6.0;
        let mut best: Option<(f32, u32)> = None;
        for (i, v) in tree.vertices.iter().enumerate() {
            let Some(q) = project(&vp, v.pos, VIEWPORT) else {
                continue;
            };
            let d2 = (q[0] - pixel[0]).powi(2) + (q[1] - pixel[1]).powi(2);
            if q[2] > 1.0 || d2 > radius * radius {
                continue;
            }
            if best.is_none_or(|(z, _)| q[2] > z) {
                best = Some((q[2], i as u32));
            }
        }
        let expected = best.map(|(_, i)| i);
        hits += usize::from(expected.is_some());
        assert_eq!(
            pick_point(&tree, &drawn, &cam, VIEWPORT, pixel, radius),
            expected,
            "pixel {pixel:?} radius {radius}"
        );
    }
    assert!(hits > 10, "only {hits} clicks hit a point");
}

#[test]
fn pick_pose_nearest_on_screen() {
    let cam = along_x();
    let vp = cam.view_proj(ASPECT);
    // Pixels per metre at 10 m: 600 / tan(30°) / 10.
    let ppm = 600.0 / 30f32.to_radians().tan() / 10.0;
    let off = 3.0 / ppm; // 3 px at 10 m
    let positions = [
        [10.0, -off, 0.0], // 3 px right of the centre
        [5.0, 0.0, 0.0],   // on the centre, nearer
        [10.0, 0.0, 0.0],  // on the centre, farther
        [10.0, 0.0, 0.0],  // same as before: the smaller index wins
    ];
    assert_eq!(pick_pose(&positions, &vp, VIEWPORT, CENTRE, 8.0), Some(1));
    assert_eq!(
        pick_pose(&positions[2..], &vp, VIEWPORT, CENTRE, 8.0),
        Some(0)
    );
    let q0 = project(&vp, positions[0], VIEWPORT).unwrap();
    assert!((q0[0] - 803.0).abs() < 0.01, "{q0:?}");
    // Nearest on screen beats nearest to the camera.
    assert_eq!(
        pick_pose(&positions, &vp, VIEWPORT, [803.0, 600.0], 8.0),
        Some(0)
    );
    assert_eq!(
        pick_pose(&positions, &vp, VIEWPORT, [820.0, 600.0], 8.0),
        None
    );
    assert_eq!(pick_pose(&[], &vp, VIEWPORT, CENTRE, 8.0), None);
    assert_eq!(
        pick_pose(&[[-5.0, 0.0, 0.0]], &vp, VIEWPORT, CENTRE, 8.0),
        None
    );
}

#[test]
fn pick_uses_physical_pixels() {
    // A panel of 800 × 600 points at 2 physical pixels per point, at (100, 40) in the window.
    let ppp = 2.0;
    let rect_min = [100.0, 40.0];
    let viewport_px = [800.0 * ppp, 600.0 * ppp];
    let mut cam = Camera {
        target: Vector3::new(0.0, 0.0, 0.0),
        distance: 20.0,
        ..Camera::default()
    };
    cam.orbit(-60.0, 15.0);
    let p = [3.0, -2.0, 1.0];
    let tree = tree_of(vec![p]);
    let drawn = all_nodes(&tree);
    let q = project(
        &cam.view_proj(viewport_px[0] / viewport_px[1]),
        p,
        viewport_px,
    )
    .unwrap();
    // Far enough from the top-left corner that unconverted coordinates miss.
    assert!(q[0] > 300.0 && q[1] > 300.0, "{q:?}");
    // Where the mouse is, in egui points.
    let mouse = [rect_min[0] + q[0] / ppp, rect_min[1] + q[1] / ppp];
    let hit = to_physical(mouse, rect_min, ppp);
    assert!((hit[0] - q[0]).abs() < 1e-3 && (hit[1] - q[1]).abs() < 1e-3);
    assert_eq!(
        pick_point(&tree, &drawn, &cam, viewport_px, hit, 4.0),
        Some(0)
    );
    let raw = [mouse[0] - rect_min[0], mouse[1] - rect_min[1]];
    assert_eq!(pick_point(&tree, &drawn, &cam, viewport_px, raw, 4.0), None);
    assert_eq!(
        pick_point(&tree, &drawn, &cam, viewport_px, mouse, 4.0),
        None
    );
}
