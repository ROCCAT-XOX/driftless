//! Alignment of two point clouds on synthetic rooms: features, RANSAC, ICP and the verdict.
//!
//! The scenes are rooms of walls, floor and ceiling sampled every 2 cm with 3 mm noise, with
//! boxes and a column at places without symmetry. `source` is a part of the scene seen from
//! a frame at a known pose `T`, so that `target ≈ T · source`.

use driftless_io::progress::Progress;
use driftless_slam::align::{
    align_pair, align_with_pairs, pair_quality, pose_from_pairs, AlignConfig, AlignMethod,
    Alignment, Verdict,
};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

/// Sample spacing on every surface (m).
const SPACING: f64 = 0.02;
/// Noise of every sample, standard deviation per axis (m).
const NOISE: f64 = 0.003;

/// Deterministic pseudo-random numbers.
struct Lcg(u64);

impl Lcg {
    /// Uniform in [0, 1).
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Normal with standard deviation `sigma` (Box–Muller).
    fn gauss(&mut self, sigma: f64) -> f64 {
        let (u, v) = (self.next().max(1e-12), self.next());
        sigma * (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// Rectangle with corner `o` and edges `u`, `v`.
#[derive(Clone, Copy)]
struct Rect {
    o: Vector3<f64>,
    u: Vector3<f64>,
    v: Vector3<f64>,
}

fn v3(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

fn rect(o: Vector3<f64>, u: Vector3<f64>, v: Vector3<f64>) -> Rect {
    Rect { o, u, v }
}

/// Floor, ceiling and the four walls of the room `[x0, x0 + w] × [y0, y0 + d] × [0, h]`.
fn room(x0: f64, y0: f64, w: f64, d: f64, h: f64) -> Vec<Rect> {
    let mut out = vec![
        rect(v3(x0, y0, 0.0), v3(w, 0.0, 0.0), v3(0.0, d, 0.0)),
        rect(v3(x0, y0, h), v3(w, 0.0, 0.0), v3(0.0, d, 0.0)),
    ];
    out.extend(walls(x0, y0, w, d, h));
    out
}

/// The four walls of the room of [`room`].
fn walls(x0: f64, y0: f64, w: f64, d: f64, h: f64) -> Vec<Rect> {
    vec![
        rect(v3(x0, y0, 0.0), v3(w, 0.0, 0.0), v3(0.0, 0.0, h)),
        rect(v3(x0, y0 + d, 0.0), v3(w, 0.0, 0.0), v3(0.0, 0.0, h)),
        rect(v3(x0, y0, 0.0), v3(0.0, d, 0.0), v3(0.0, 0.0, h)),
        rect(v3(x0 + w, y0, 0.0), v3(0.0, d, 0.0), v3(0.0, 0.0, h)),
    ]
}

/// The four sides and the top of a box standing on the floor.
fn crate_box(x0: f64, y0: f64, w: f64, d: f64, h: f64) -> Vec<Rect> {
    let mut out = walls(x0, y0, w, d, h);
    out.push(rect(v3(x0, y0, h), v3(w, 0.0, 0.0), v3(0.0, d, 0.0)));
    out
}

/// Noisy samples every [`SPACING`] on `rects`, keeping those for which `keep` holds.
fn sample(rects: &[Rect], keep: impl Fn(&Vector3<f64>) -> bool, seed: u64) -> Vec<Vector3<f64>> {
    let mut rng = Lcg(seed);
    let mut out = Vec::new();
    for r in rects {
        let nu = (r.u.norm() / SPACING).round().max(1.0) as usize;
        let nv = (r.v.norm() / SPACING).round().max(1.0) as usize;
        for i in 0..=nu {
            for j in 0..=nv {
                let p = r.o + r.u * (i as f64 / nu as f64) + r.v * (j as f64 / nv as f64);
                if keep(&p) {
                    let n = v3(rng.gauss(NOISE), rng.gauss(NOISE), rng.gauss(NOISE));
                    out.push(p + n);
                }
            }
        }
    }
    out
}

/// Room 8 × 5 × 2.8 m with three boxes and a column.
fn main_room() -> Vec<Rect> {
    let mut s = room(0.0, 0.0, 8.0, 5.0, 2.8);
    s.extend(crate_box(2.5, 0.6, 0.8, 0.6, 0.8));
    s.extend(crate_box(4.4, 3.0, 1.2, 0.6, 0.5));
    s.extend(crate_box(6.6, 1.5, 0.6, 1.0, 1.2));
    s.extend(walls(3.8, 2.2, 0.4, 0.4, 2.8));
    s
}

fn transform(pose: &Isometry3<f64>, pts: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    pts.iter()
        .map(|p| (pose * Point3::from(*p)).coords)
        .collect()
}

/// Pose of the source frame in the target frame.
fn truth(tilt_deg: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(1.2, -0.8, 0.05),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 37f64.to_radians())
            * UnitQuaternion::from_euler_angles(tilt_deg.to_radians(), 0.0, 0.0),
    )
}

/// Target: the part `x ≤ 6` of the main room in the world frame. Source: the part `x ≥ 2`
/// in the frame at `pose`.
fn room_pair(pose: &Isometry3<f64>) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
    let scene = main_room();
    let target = sample(&scene, |p| p.x <= 6.0, 1);
    let world = sample(&scene, |p| p.x >= 2.0, 2);
    (transform(&pose.inverse(), &world), target)
}

/// Translation error at the centroid of `source` (m) and rotation error (°) of `est`.
fn pose_error(est: &Isometry3<f64>, truth: &Isometry3<f64>, source: &[Vector3<f64>]) -> (f64, f64) {
    let c = Point3::from(source.iter().sum::<Vector3<f64>>() / source.len() as f64);
    let trans = (est * c - truth * c).norm();
    let rot = (est.rotation.inverse() * truth.rotation)
        .angle()
        .to_degrees();
    (trans, rot)
}

fn describe(a: &Alignment) -> String {
    format!(
        "{:?} {:?} overlap {:.3} agreement {:.3} residual {:.2} mm tilt {:.2}° runner-up {:?}",
        a.verdict,
        a.method,
        a.quality.overlap,
        a.quality.agreement,
        a.quality.residual_median_mm,
        a.quality.tilt_deg,
        a.runner_up.as_ref().map(|r| r.1)
    )
}

fn run(source: &[Vector3<f64>], target: &[Vector3<f64>]) -> Alignment {
    let t = std::time::Instant::now();
    let a = align_pair(source, target, &AlignConfig::default(), &Progress::new());
    eprintln!(
        "align_pair {:.2} s: {}",
        t.elapsed().as_secs_f64(),
        describe(&a)
    );
    a
}

#[test]
fn aligns_rotated_room() {
    let pose = truth(0.0);
    let (source, target) = room_pair(&pose);
    let a = run(&source, &target);
    let (dt, dr) = pose_error(&a.pose, &pose, &source);
    assert!(dt < 0.01 && dr < 0.3, "error {dt:.4} m {dr:.3}°");
    assert_eq!(a.verdict, Verdict::Sure, "{}", describe(&a));
    assert!(a.quality.overlap > 0.5, "{}", describe(&a));
}

#[test]
fn aligns_with_small_tilt() {
    let pose = truth(0.6);
    let (source, target) = room_pair(&pose);
    let a = run(&source, &target);
    let (dt, dr) = pose_error(&a.pose, &pose, &source);
    assert!(dt < 0.01 && dr < 0.3, "error {dt:.4} m {dr:.3}°");
    assert_eq!(a.verdict, Verdict::Sure, "{}", describe(&a));
}

#[test]
fn symmetric_box_is_unsure() {
    let scene = room(0.0, 0.0, 6.0, 4.0, 2.5);
    let target = sample(&scene, |_| true, 3);
    // The same box turned by 180° about its centre: both turns fit equally well.
    let turn = Isometry3::from_parts(
        Translation3::new(3.0, 2.0, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
    ) * Translation3::new(-3.0, -2.0, 0.0);
    let source = transform(&turn, &sample(&scene, |_| true, 4));
    let a = run(&source, &target);
    assert_ne!(a.verdict, Verdict::Sure, "{}", describe(&a));
}

/// Room A (8 × 5 m), a corridor (3 × 1.5 m) through a door in its wall `x = 8`, and room B
/// (8 × 5 m) behind the corridor. Returns (room A, corridor, room B).
fn suite() -> (Vec<Rect>, Vec<Rect>, Vec<Rect>) {
    let (h, door_y0, door_y1, door_h) = (2.8, 1.75, 3.25, 2.1);
    // A wall `x = x0` from y 0 to 5 with the door opening (the lintel belongs to the
    // corridor, which sees it from both sides).
    let door_wall = |x0: f64| {
        vec![
            rect(v3(x0, 0.0, 0.0), v3(0.0, door_y0, 0.0), v3(0.0, 0.0, h)),
            rect(
                v3(x0, door_y1, 0.0),
                v3(0.0, 5.0 - door_y1, 0.0),
                v3(0.0, 0.0, h),
            ),
        ]
    };
    let room_without = |x0: f64, door_x: f64| {
        let mut s = vec![
            rect(v3(x0, 0.0, 0.0), v3(8.0, 0.0, 0.0), v3(0.0, 5.0, 0.0)),
            rect(v3(x0, 0.0, h), v3(8.0, 0.0, 0.0), v3(0.0, 5.0, 0.0)),
            rect(v3(x0, 0.0, 0.0), v3(8.0, 0.0, 0.0), v3(0.0, 0.0, h)),
            rect(v3(x0, 5.0, 0.0), v3(8.0, 0.0, 0.0), v3(0.0, 0.0, h)),
        ];
        let other = if door_x == x0 { x0 + 8.0 } else { x0 };
        s.push(rect(
            v3(other, 0.0, 0.0),
            v3(0.0, 5.0, 0.0),
            v3(0.0, 0.0, h),
        ));
        s.extend(door_wall(door_x));
        s
    };
    let mut a = room_without(0.0, 8.0);
    a.extend(crate_box(2.5, 0.6, 0.8, 0.6, 0.8));
    a.extend(crate_box(4.4, 3.0, 1.2, 0.6, 0.5));
    a.extend(walls(1.6, 3.4, 0.4, 0.4, h));
    let w = door_y1 - door_y0;
    let mut corridor = vec![
        rect(v3(8.0, door_y0, 0.0), v3(3.0, 0.0, 0.0), v3(0.0, w, 0.0)),
        rect(v3(8.0, door_y0, h), v3(3.0, 0.0, 0.0), v3(0.0, w, 0.0)),
        rect(v3(8.0, door_y0, 0.0), v3(3.0, 0.0, 0.0), v3(0.0, 0.0, h)),
        rect(v3(8.0, door_y1, 0.0), v3(3.0, 0.0, 0.0), v3(0.0, 0.0, h)),
        rect(
            v3(8.0, door_y0, door_h),
            v3(0.0, w, 0.0),
            v3(0.0, 0.0, h - door_h),
        ),
        rect(
            v3(11.0, door_y0, door_h),
            v3(0.0, w, 0.0),
            v3(0.0, 0.0, h - door_h),
        ),
    ];
    corridor.extend(crate_box(9.0, door_y0, 0.8, 0.4, 0.9));
    let mut b = room_without(11.0, 11.0);
    b.extend(crate_box(13.0, 3.5, 1.5, 1.1, 0.9));
    b.extend(crate_box(17.6, 0.4, 1.0, 0.6, 0.6));
    b.extend(walls(16.3, 1.0, 0.4, 0.4, h));
    (a, corridor, b)
}

#[test]
fn disjoint_rooms_are_not_sure() {
    let (a, _, _) = suite();
    let target = sample(&a, |_| true, 5);
    // A different room: other size and height, other furniture.
    let mut other = room(0.0, 0.0, 6.0, 7.0, 2.6);
    other.extend(crate_box(0.8, 4.9, 1.4, 0.7, 0.9));
    other.extend(crate_box(3.9, 1.1, 0.7, 0.7, 1.1));
    other.extend(walls(4.6, 5.2, 0.3, 0.3, 2.6));
    let source = transform(&truth(0.0).inverse(), &sample(&other, |_| true, 6));
    let a = run(&source, &target);
    assert_ne!(a.verdict, Verdict::Sure, "{}", describe(&a));
}

#[test]
fn corridor_overlap_aligns() {
    let (a, corridor, b) = suite();
    let target_rects: Vec<Rect> = a.iter().chain(&corridor).copied().collect();
    let source_rects: Vec<Rect> = corridor.iter().chain(&b).copied().collect();
    let target = sample(&target_rects, |_| true, 7);
    let world = sample(&source_rects, |_| true, 8);
    let shared = sample(&corridor, |_| true, 9).len() as f64 / world.len() as f64;
    eprintln!("shared part {:.1} % of the source points", 100.0 * shared);
    let pose = truth(0.0);
    let source = transform(&pose.inverse(), &world);
    let al = run(&source, &target);
    let (dt, dr) = pose_error(&al.pose, &pose, &source);
    eprintln!("error {dt:.4} m {dr:.3}°");
    if al.verdict == Verdict::Sure {
        assert!(dt < 0.02, "sure with error {dt:.4} m: {}", describe(&al));
    }
}

#[test]
fn pose_from_pairs_recovers_transform() {
    let pose = truth(4.0);
    let src = [
        v3(0.3, -1.2, 0.4),
        v3(2.5, 0.7, 1.9),
        v3(-1.4, 3.1, 0.2),
        v3(0.9, 0.2, 2.6),
    ];
    let pairs: Vec<_> = src
        .iter()
        .map(|s| (*s, (pose * Point3::from(*s)).coords))
        .collect();
    let est = pose_from_pairs(&pairs).expect("pose");
    assert!((est.translation.vector - pose.translation.vector).norm() < 1e-9);
    assert!((est.rotation.inverse() * pose.rotation).angle() < 1e-9);
    // Three pairs are enough.
    assert!(pose_from_pairs(&pairs[..3]).is_some());
    assert!(pose_from_pairs(&pairs[..2]).is_none());
}

#[test]
fn pose_from_pairs_rejects_collinear() {
    let pairs: Vec<_> = [0.0, 1.0, 2.5]
        .iter()
        .map(|&s| {
            let p = v3(1.0, 2.0, 0.5) + v3(0.6, -0.3, 0.1) * s;
            (p, p + v3(0.4, 0.0, 0.0))
        })
        .collect();
    assert!(pose_from_pairs(&pairs).is_none());
}

#[test]
fn align_with_pairs_refines() {
    let pose = truth(0.0);
    let (source, target) = room_pair(&pose);
    // Box corners and the column seen in both parts, each source mark 5 cm off.
    let marks = [
        (v3(2.5, 0.6, 0.8), v3(0.05, 0.0, 0.0)),
        (v3(5.6, 3.6, 0.5), v3(0.0, -0.05, 0.0)),
        (v3(3.8, 2.2, 2.8), v3(0.0, 0.0, 0.05)),
    ];
    let inv = pose.inverse();
    let pairs: Vec<_> = marks
        .iter()
        .map(|(w, e)| ((inv * Point3::from(w + e)).coords, *w))
        .collect();
    let a = align_with_pairs(
        &source,
        &target,
        &pairs,
        &AlignConfig::default(),
        &Progress::new(),
    );
    let (dt, dr) = pose_error(&a.pose, &pose, &source);
    assert!(dt < 0.01 && dr < 0.3, "error {dt:.4} m {dr:.3}°");
    assert_eq!(a.method, AlignMethod::PointPairs);
    assert!(a.runner_up.is_none());
}

#[test]
fn deterministic_across_threads() {
    // The grid path (features find nothing in this room).
    let (source, target) = room_pair(&truth(0.0));
    assert_same_in_pools(&source, &target, AlignMethod::Grid);
}

#[test]
fn deterministic_features_across_threads() {
    let (source, target) = cluttered_pair(&truth(0.0));
    assert_same_in_pools(&source, &target, AlignMethod::Features);
}

/// The same alignment with 1 and with 4 threads, by `method`.
fn assert_same_in_pools(source: &[Vector3<f64>], target: &[Vector3<f64>], method: AlignMethod) {
    let in_pool = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool")
            .install(|| run(source, target))
    };
    let one = in_pool(1);
    let four = in_pool(4);
    assert_eq!(one.method, method);
    assert_eq!(one, four);
}

#[test]
fn empty_inputs_fail() {
    let (source, target) = room_pair(&truth(0.0));
    let cfg = AlignConfig::default();
    let p = Progress::new();
    for a in [
        align_pair(&[], &target, &cfg, &p),
        align_pair(&source, &[], &cfg, &p),
        align_pair(&source[..2], &target[..2], &cfg, &p),
        align_with_pairs(&[], &target, &[], &cfg, &p),
    ] {
        assert!(matches!(a.verdict, Verdict::Failed(_)), "{}", describe(&a));
        assert_eq!(a.verdict.word(), "gescheitert");
    }
    let q = pair_quality(&[], &target, &Isometry3::identity(), &cfg);
    assert_eq!(q.overlap, 0.0);
}

#[test]
fn cancelled_alignment_fails() {
    let (source, target) = room_pair(&truth(0.0));
    let p = Progress::new();
    p.cancel();
    let a = align_pair(&source, &target, &AlignConfig::default(), &p);
    assert_eq!(a.verdict, Verdict::Failed("abgebrochen".into()));
}

#[test]
fn verdict_words() {
    assert_eq!(Verdict::Sure.word(), "sicher");
    assert_eq!(Verdict::Unsure("x".into()).word(), "unsicher");
    assert_eq!(Verdict::Failed("x".into()).word(), "gescheitert");
}

/// Box of `w × d × h` at `(x, y)` turned by `yaw` about its corner.
fn turned_box(x: f64, y: f64, yaw: f64, w: f64, d: f64, h: f64) -> Vec<Rect> {
    let (u, v) = (
        v3(yaw.cos(), yaw.sin(), 0.0) * w,
        v3(-yaw.sin(), yaw.cos(), 0.0) * d,
    );
    let (o, up) = (v3(x, y, 0.0), v3(0.0, 0.0, h));
    vec![
        rect(o, u, up),
        rect(o + v, u, up),
        rect(o, v, up),
        rect(o + u, v, up),
        rect(o + up, u, v),
    ]
}

/// Room with 14 boxes of random size, place and turn: corners at many angles, so that the
/// features find the pose (the boxes of the other scenes are all parallel to the walls,
/// and every corner looks like every other).
/// Target: the part `x ≤ 6` of the cluttered room, source: the part `x ≥ 2` in the frame
/// at `pose`.
fn cluttered_pair(pose: &Isometry3<f64>) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
    let mut scene = room(0.0, 0.0, 8.0, 5.0, 2.8);
    let mut rng = Lcg(77);
    for _ in 0..14 {
        let (x, y) = (0.5 + 6.5 * rng.next(), 0.5 + 3.5 * rng.next());
        let yaw = rng.next() * 3.0;
        let (w, d, h) = (
            0.3 + 0.9 * rng.next(),
            0.3 + 0.6 * rng.next(),
            0.3 + 1.2 * rng.next(),
        );
        scene.extend(turned_box(x, y, yaw, w, d, h));
    }
    let target = sample(&scene, |p| p.x <= 6.0, 11);
    let world = sample(&scene, |p| p.x >= 2.0, 12);
    (transform(&pose.inverse(), &world), target)
}

#[test]
fn cluttered_room_aligns_by_features() {
    let pose = truth(0.0);
    let (source, target) = cluttered_pair(&pose);
    let a = run(&source, &target);
    let (dt, dr) = pose_error(&a.pose, &pose, &source);
    assert!(dt < 0.01 && dr < 0.3, "error {dt:.4} m {dr:.3}°");
    assert_eq!(a.method, AlignMethod::Features, "{}", describe(&a));
    assert_eq!(a.verdict, Verdict::Sure, "{}", describe(&a));
}
