//! Loop search and verification on synthetic submaps built straight from scene surfaces.

use driftless_odom::testutil::{corridor_scene, room_scene, Scene};
use driftless_odom::{voxel_downsample, FrameQuality, OUTPUT_MAP_VOXEL};
use driftless_slam::{
    find_candidates, verify, LoopCandidate, LoopCheck, LoopConfig, LoopVerdict, Submap, SubmapFrame,
};
use driftless_traj::nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

/// Frame spacing along the path (m).
const STEP: f64 = 0.25;

fn pose(x: f64, y: f64, yaw: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, y, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
    )
}

/// Surface samples within `range` of any of `views` that are not hidden behind another
/// surface, in the frame of `anchor`, one per output voxel.
fn visible_points(
    scene: &Scene,
    views: &[Isometry3<f64>],
    range: f64,
    anchor: &Isometry3<f64>,
) -> Vec<[f32; 3]> {
    let samples = scene.surface_samples(0.1, 0.05);
    let inv = anchor.inverse();
    let mut pts = Vec::new();
    for v in views {
        let o = v.translation.vector;
        for w in &samples {
            let dist = (w - o).norm();
            if dist < range && dist > 1e-6 && scene.raycast(&o, &((w - o) / dist)) > dist - 0.01 {
                pts.push((inv * Point3::from(*w)).coords);
            }
        }
    }
    voxel_downsample(&pts, OUTPUT_MAP_VOXEL)
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect()
}

/// A submap whose frames are `poses` (true poses, the first is the anchor) and whose points
/// are what is visible within `range` of every fourth frame.
fn submap(id: usize, scene: &Scene, poses: &[Isometry3<f64>], range: f64) -> Submap {
    let anchor = poses[0];
    let frames = poses
        .iter()
        .enumerate()
        .map(|(i, p)| SubmapFrame {
            index: id * 1000 + i,
            t: (id * 1000 + i) as f64 * 0.1,
            offset: anchor.inverse() * p,
            quality: FrameQuality::first(),
        })
        .collect();
    let views: Vec<_> = poses.iter().step_by(4).copied().collect();
    Submap {
        id,
        anchor,
        frames,
        points: visible_points(scene, &views, range, &anchor),
    }
}

/// Rectangle around both boxes of the room, walked counter-clockwise from its lower left
/// corner, heading along the path.
const RECT: [[f64; 2]; 4] = [[-4.3, -3.5], [4.5, -3.5], [4.5, 4.2], [-4.3, 4.2]];

fn rect_perimeter() -> f64 {
    (0..4)
        .map(|i| {
            let (p, q) = (RECT[i], RECT[(i + 1) % 4]);
            ((q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2)).sqrt()
        })
        .sum()
}

fn rect_pose(s: f64) -> Isometry3<f64> {
    let mut s = s.rem_euclid(rect_perimeter());
    for i in 0..4 {
        let (p, q) = (RECT[i], RECT[(i + 1) % 4]);
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let len = (dx * dx + dy * dy).sqrt();
        if s <= len || i == 3 {
            let f = s / len;
            return pose(p[0] + f * dx, p[1] + f * dy, dy.atan2(dx));
        }
        s -= len;
    }
    unreachable!()
}

/// Six submaps once around the rectangle plus a seventh that starts again at the start.
fn rectangle_walk() -> (Vec<Submap>, Vec<f64>) {
    let scene = room_scene();
    let per = rect_perimeter();
    let len = per / 6.0;
    let n_frames = (len / STEP).round() as usize;
    let mut maps = Vec::new();
    let mut starts = Vec::new();
    for k in 0..7 {
        let s0 = k as f64 * len;
        let poses: Vec<_> = (0..n_frames)
            .map(|i| rect_pose(s0 + i as f64 * len / n_frames as f64))
            .collect();
        maps.push(submap(k, &scene, &poses, 15.0));
        starts.push(s0);
    }
    (maps, starts)
}

/// Odometry anchors with drift growing along the path to `trans` (m) and `yaw` (rad) at
/// `total` metres.
fn drifted(
    maps: &[Submap],
    starts: &[f64],
    total: f64,
    trans: Vector3<f64>,
    yaw: f64,
) -> Vec<Isometry3<f64>> {
    maps.iter()
        .zip(starts)
        .map(|(m, s)| {
            let f = s / total;
            Isometry3::from_parts(
                Translation3::from(trans * f),
                UnitQuaternion::from_euler_angles(0.0, 0.0, yaw * f),
            ) * m.anchor
        })
        .collect()
}

fn true_relative(maps: &[Submap], a: usize, b: usize) -> Isometry3<f64> {
    maps[a].anchor.inverse() * maps[b].anchor
}

/// Translation (m) and rotation (deg) error of `got` against `want`.
fn error(got: &Isometry3<f64>, want: &Isometry3<f64>) -> (f64, f64) {
    let d = want.inverse() * got;
    (d.translation.vector.norm(), d.rotation.angle().to_degrees())
}

fn describe(c: &LoopCheck, want: &Isometry3<f64>) -> String {
    let (dt, dr) = error(&c.relative, want);
    format!(
        "{}↔{}: {:?} inlier {:.3} rms {:.4} deg {:.4} err {:.4} m {:.3}°",
        c.a, c.b, c.verdict, c.inlier_ratio, c.rms, c.degeneracy, dt, dr
    )
}

#[test]
fn finds_loop_after_rectangle() {
    let (maps, starts) = rectangle_walk();
    let total = rect_perimeter();
    let anchors = drifted(
        &maps,
        &starts,
        total,
        Vector3::new(0.3, -0.25, 0.08),
        1f64.to_radians(),
    );
    let end_drift = (anchors[6].translation.vector - maps[6].anchor.translation.vector).norm();
    assert!(end_drift > 0.4, "drift at the end {end_drift}");

    let cfg = LoopConfig::default();
    let cands = find_candidates(&maps, &anchors, &cfg);
    assert!(cands.contains(&LoopCandidate { a: 0, b: 6 }), "{cands:?}");
    for c in &cands {
        let check = verify(&maps, &anchors, c, &cfg);
        let want = true_relative(&maps, c.a, c.b);
        eprintln!("{}", describe(&check, &want));
        if check.verdict == LoopVerdict::Accepted {
            let (dt, dr) = error(&check.relative, &want);
            assert!(dt < 0.03 && dr < 0.5, "{}", describe(&check, &want));
        }
        if (c.a, c.b) == (0, 6) {
            assert_eq!(
                check.verdict,
                LoopVerdict::Accepted,
                "{}",
                describe(&check, &want)
            );
            assert!(!check.manual);
        }
    }
}

/// Straight walk along the corridor from `x0`, `n` frames.
fn corridor_poses(x0: f64, n: usize) -> Vec<Isometry3<f64>> {
    (0..n)
        .map(|i| pose(x0 + i as f64 * STEP, 0.0, 0.0))
        .collect()
}

#[test]
fn no_candidates_on_straight_walk() {
    let scene = corridor_scene();
    // 28 m in submaps of 5 m, anchors exact
    let maps: Vec<_> = (0..6)
        .map(|k| submap(k, &scene, &corridor_poses(-2.0 + 5.0 * k as f64, 20), 0.0))
        .collect();
    let anchors: Vec<_> = maps.iter().map(|m| m.anchor).collect();
    assert!(find_candidates(&maps, &anchors, &LoopConfig::default()).is_empty());
}

#[test]
fn rejects_repetitive_corridor() {
    // Two short submaps one pillar period (3 m) apart, away from the crate and the end
    // walls, so the corridor looks the same from both. Their anchors claim the same place.
    let scene = corridor_scene();
    let maps = vec![
        submap(0, &scene, &corridor_poses(19.5, 4), 6.0),
        submap(1, &scene, &corridor_poses(22.5, 4), 6.0),
    ];
    let anchors = vec![maps[0].anchor, maps[0].anchor];
    let check = verify(
        &maps,
        &anchors,
        &LoopCandidate { a: 0, b: 1 },
        &LoopConfig::default(),
    );
    let want = true_relative(&maps, 0, 1);
    eprintln!("{}", describe(&check, &want));
    let (dt, _) = error(&check.relative, &want);
    assert!(
        check.verdict != LoopVerdict::Accepted || dt < 0.1,
        "{}",
        describe(&check, &want)
    );
}

#[test]
fn rejects_low_overlap() {
    // 12 m apart in the corridor with 7 m range: only a short stretch is seen by both.
    let scene = corridor_scene();
    let maps = vec![
        submap(0, &scene, &corridor_poses(2.0, 4), 7.0),
        submap(1, &scene, &corridor_poses(14.0, 4), 7.0),
    ];
    let anchors: Vec<_> = maps.iter().map(|m| m.anchor).collect();
    let check = verify(
        &maps,
        &anchors,
        &LoopCandidate { a: 0, b: 1 },
        &LoopConfig::default(),
    );
    eprintln!("{}", describe(&check, &true_relative(&maps, 0, 1)));
    assert_eq!(check.verdict, LoopVerdict::Rejected("zu wenig Überlappung"));
}

#[test]
fn candidates_are_sorted_and_deterministic() {
    let (maps, starts) = rectangle_walk();
    let anchors = drifted(
        &maps,
        &starts,
        rect_perimeter(),
        Vector3::new(0.3, -0.25, 0.08),
        0.0,
    );
    let cfg = LoopConfig::default();
    let cands = find_candidates(&maps, &anchors, &cfg);
    assert!(!cands.is_empty());
    assert_eq!(cands, find_candidates(&maps, &anchors, &cfg));
    for w in cands.windows(2) {
        assert!((w[0].b, w[0].a) < (w[1].b, w[1].a), "{cands:?}");
    }
    for c in &cands {
        assert!(c.b - c.a >= cfg.min_gap);
        assert!(cands.iter().filter(|o| o.b == c.b).count() <= cfg.max_candidates);
    }

    // With one candidate per submap it is the nearest anchor.
    let one = LoopConfig {
        max_candidates: 1,
        ..cfg.clone()
    };
    for c in find_candidates(&maps, &anchors, &one) {
        let d = |a: usize| (anchors[c.b].translation.vector - anchors[a].translation.vector).norm();
        for o in cands.iter().filter(|o| o.b == c.b) {
            assert!(d(c.a) <= d(o.a));
        }
    }

    let c = LoopCandidate { a: 0, b: 6 };
    assert_eq!(
        verify(&maps, &anchors, &c, &cfg),
        verify(&maps, &anchors, &c, &cfg)
    );
}
