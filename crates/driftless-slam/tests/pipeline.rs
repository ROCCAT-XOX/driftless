//! Whole SLAM runs on synthetic scans, and the loop handling of the pose graph on its own.

use driftless_odom::testutil::{
    corridor_scene, imu_samples, lidar_frames, room_scene, Trajectory, Walk,
};
use driftless_odom::OdomConfig;
use driftless_slam::{
    close_loops, run_slam, solve, LoopCheck, LoopConfig, LoopVerdict, ManualLoop, SlamConfig,
    SlamReport, SlamResult, Submap,
};
use driftless_traj::nalgebra::{Isometry3, Matrix6, Translation3, UnitQuaternion, Vector3};
use driftless_traj::{start_end_gap, Pose};
use std::f64::consts::{FRAC_PI_2, PI};

/// Seconds the IMU runs before the first and after the last LiDAR frame.
const IMU_MARGIN: f64 = 0.5;

fn run(
    traj: &dyn Trajectory,
    scene: &driftless_odom::testutil::Scene,
    duration: f64,
    gyro_bias_z: f64,
) -> SlamResult {
    let frames = lidar_frames(scene, traj, 0.0, duration, 360, &Isometry3::identity());
    let mut imu = imu_samples(traj, -IMU_MARGIN, duration + IMU_MARGIN, 200.0);
    for s in imu.iter_mut().filter(|s| s.stamp > 2.0) {
        s.gyro[2] += gyro_bias_z;
    }
    let (r, stats) = run_slam(frames.into_iter(), &imu, &SlamConfig::default());
    assert_eq!(stats.frames_used, r.poses().len());
    r
}

fn max_pose_diff(a: &[Pose], b: &[Pose]) -> f64 {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b)
        .map(|(x, y)| {
            assert_eq!(x.t, y.t);
            (x.p - y.p).norm().max(x.q.angle_to(&y.q))
        })
        .fold(0.0, f64::max)
}

#[test]
fn slam_without_loop_equals_odometry() {
    let walk = Walk {
        still: 1.0,
        speed: 1.0,
    };
    let scene = corridor_scene();
    let duration = 22.0;
    let r = run(&walk, &scene, duration, 0.0);
    assert!(r.submaps.len() >= 4, "{} submaps", r.submaps.len());
    assert!(
        r.loops.iter().all(|c| c.verdict != LoopVerdict::Accepted),
        "{:?}",
        r.loops
    );
    assert!(max_pose_diff(&r.poses(), &r.odom_poses()) < 1e-6);

    // odom_poses() reproduces the plain odometry
    let frames = lidar_frames(&scene, &walk, 0.0, duration, 360, &Isometry3::identity());
    let imu = imu_samples(&walk, -IMU_MARGIN, duration + IMU_MARGIN, 200.0);
    let odom = driftless_odom::run_odometry(frames.into_iter(), &imu, &OdomConfig::default());
    assert!(max_pose_diff(&r.odom_poses(), &odom.poses) < 1e-9);
}

#[test]
fn slam_single_submap_skips_graph() {
    let walk = Walk {
        still: 1.0,
        speed: 1.0,
    };
    let r = run(&walk, &corridor_scene(), 3.0, 0.0);
    assert_eq!(r.submaps.len(), 1);
    assert!(r.graph.is_none());
    assert!(r.loops.is_empty());
    assert_eq!(r.anchors, r.odom_anchors);
    assert_eq!(r.poses(), r.odom_poses());
    assert!(!r.map_points().is_empty());
}

/// Two laps on a circle of radius 2 m around the room centre (clear of both boxes),
/// counter-clockwise, heading along the path. Still at the start and at the end, so the
/// trajectory ends where it started.
struct Circle {
    still: f64,
    radius: f64,
    speed: f64,
    laps: f64,
}

/// Time to reach full speed and to stop again (s).
const RAMP: f64 = 2.0;

impl Circle {
    fn length(&self) -> f64 {
        2.0 * PI * self.radius * self.laps
    }

    /// Distance covered within `tau` seconds of a smooth-step speed ramp.
    fn ramp(&self, tau: f64) -> f64 {
        let s = (tau / RAMP).clamp(0.0, 1.0);
        self.speed * RAMP * (s * s * s - 0.5 * s * s * s * s)
    }

    /// Seconds of motion: both ramps cover half of their duration at full speed.
    fn moving(&self) -> f64 {
        self.length() / self.speed + RAMP
    }

    fn duration(&self) -> f64 {
        2.0 * self.still + self.moving()
    }

    /// Path length covered at time `t`.
    fn s(&self, t: f64) -> f64 {
        let tau = t - self.still;
        let m = self.moving();
        if tau <= 0.0 {
            0.0
        } else if tau < RAMP {
            self.ramp(tau)
        } else if tau < m - RAMP {
            self.speed * (0.5 * RAMP + tau - RAMP)
        } else if tau < m {
            self.length() - self.ramp(m - tau)
        } else {
            self.length()
        }
    }
}

impl Trajectory for Circle {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        let theta = self.s(t) / self.radius;
        Isometry3::from_parts(
            Translation3::new(self.radius * theta.cos(), self.radius * theta.sin(), 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, theta + FRAC_PI_2),
        )
    }
}

fn circle() -> Circle {
    Circle {
        still: 1.0,
        radius: 2.0,
        speed: 1.0,
        laps: 3.0,
    }
}

/// Two laps in the room with a gyro bias appearing after the static start. ICP against the
/// room holds the odometry within millimetres; the bias is five times that of the odometry
/// test (`pipeline_corrects_gyro_drift_with_icp`) so that there is some drift left to remove.
fn room_loop() -> SlamResult {
    let c = circle();
    run(&c, &room_scene(), c.duration(), 0.1)
}

#[test]
fn slam_closes_synthetic_loop() {
    let c = circle();
    assert!(
        (c.pose(0.0).translation.vector - c.pose(c.duration()).translation.vector).norm() < 1e-9
    );
    let r = room_loop();
    let accepted = r
        .loops
        .iter()
        .filter(|l| l.verdict == LoopVerdict::Accepted)
        .count();
    let report = SlamReport::new(&r);
    eprintln!(
        "{} submaps, {accepted}/{} loops accepted, gap {:.4} -> {:.4} m, graph {:?}",
        r.submaps.len(),
        r.loops.len(),
        report.start_end_gap.before,
        report.start_end_gap.after,
        r.graph
    );
    assert!(accepted >= 1, "{:?}", r.loops);
    assert!(r.graph.is_some());
    let (before, after) = (start_end_gap(&r.odom_poses()), start_end_gap(&r.poses()));
    assert_eq!(report.start_end_gap.before, before);
    assert_eq!(report.start_end_gap.after, after);
    assert!(after < before, "start-end gap {before} -> {after} m");
    assert!(after < 0.05, "start-end gap after {after} m");
}

/// The submaps of `r` with odometry drift injected into their anchors, growing along the
/// submaps to 0.33 m and 3° yaw (about the start) at the last one.
fn with_injected_drift(r: &SlamResult) -> (Vec<Submap>, Vec<Isometry3<f64>>) {
    let n = r.submaps.len();
    let (trans, yaw) = (Vector3::new(0.25, -0.2, 0.05), 3f64.to_radians());
    let mut submaps = r.submaps.clone();
    for (k, m) in submaps.iter_mut().enumerate() {
        let f = k as f64 / (n - 1) as f64;
        let drift = Isometry3::from_parts(
            Translation3::from(trans * f),
            UnitQuaternion::from_euler_angles(0.0, 0.0, yaw * f),
        );
        m.anchor = drift * m.anchor;
    }
    let anchors = submaps.iter().map(|m| m.anchor).collect();
    (submaps, anchors)
}

/// With the automatic search switched off (search radius 0) the drifted room loop keeps its
/// start-end gap; one manual loop between the still phases at start and end closes it. The
/// first and the last submap share only part of the room (inlier ratio 0.34, below the
/// automatic 0.5), which `manual_min_inlier_ratio` (0.25) allows for.
#[test]
fn manual_loop_fixes_missed_loop() {
    let r = room_loop();
    let (submaps, anchors) = with_injected_drift(&r);
    let cfg = SlamConfig {
        loops: LoopConfig {
            base_radius: 0.0,
            radius_per_m: 0.0,
            ..LoopConfig::default()
        },
        ..SlamConfig::default()
    };
    let auto = solve(submaps.clone(), anchors.clone(), &[], &cfg).unwrap();
    assert!(auto.loops.is_empty(), "{:?}", auto.loops);
    let before = start_end_gap(&auto.poses());
    assert!(before > 0.25, "injected drift too small: {before} m");

    let end = SlamReport::new(&auto).submaps.last().unwrap().t_end;
    let m = ManualLoop {
        a_t: 0.5,
        b_t: end - 0.5,
    };
    let d = solve(submaps, anchors, &[m], &cfg).unwrap();
    for l in &d.loops {
        eprintln!(
            "{}↔{} {:?} inlier {:.3} rms {:.4}",
            l.a, l.b, l.verdict, l.inlier_ratio, l.rms
        );
    }
    let after = start_end_gap(&d.poses());
    eprintln!("start-end gap {before:.4} -> {after:.4} m");
    assert_eq!(d.loops.len(), 1);
    assert!(d.loops[0].manual);
    assert_eq!(d.loops[0].verdict, LoopVerdict::Accepted);
    assert_eq!((d.loops[0].a, d.loops[0].b), (0, d.submaps.len() - 1));
    assert!(after < 0.05, "start-end gap {before} -> {after} m");
}

/// The room loop with additional odometry drift injected into the submap anchors, growing
/// along the submaps to `trans` and `yaw` (about the start) at the last one. Submap 0 (mostly
/// the static start) finds no loop of its own, so the drift of its odometry edge (1/7 of the
/// total) cannot be removed; everything after it is tied together by the loops.
#[test]
fn solve_removes_injected_drift() {
    let r = room_loop();
    let n = r.submaps.len();
    let (submaps, anchors) = with_injected_drift(&r);
    let d = solve(submaps, anchors, &[], &SlamConfig::default()).unwrap();
    let (before, after) = (start_end_gap(&d.odom_poses()), start_end_gap(&d.poses()));
    for l in &d.loops {
        eprintln!(
            "{}↔{} {:?} inlier {:.3}",
            l.a, l.b, l.verdict, l.inlier_ratio
        );
    }
    eprintln!("start-end gap {before:.4} -> {after:.4} m");
    assert!(before > 0.25, "injected drift too small: {before} m");
    assert!(after < 0.25 * before, "start-end gap {before} -> {after} m");
    assert!(d
        .loops
        .iter()
        .any(|l| l.verdict == LoopVerdict::Accepted && l.b == n - 1));
}

#[test]
fn slam_is_deterministic() {
    let (a, b) = (room_loop(), room_loop());
    assert_eq!(format!("{:?}", a.poses()), format!("{:?}", b.poses()));
    assert_eq!(a.loops, b.loops);
    assert_eq!(a.map_points(), b.map_points());
    let json = |r: &SlamResult| serde_json::to_string(&SlamReport::new(r)).unwrap();
    assert_eq!(json(&a), json(&b));
}

#[test]
fn report_lists_submaps_and_loops() {
    let r = room_loop();
    let rep = SlamReport::new(&r);
    let t0 = r.submaps[0].frames[0].t;
    assert_eq!(rep.submaps.len(), r.submaps.len());
    assert_eq!(rep.submaps[0].t_start, 0.0);
    for (s, m) in rep.submaps.iter().zip(&r.submaps) {
        assert_eq!(s.id, m.id);
        assert_eq!(s.t_end, m.frames.last().unwrap().t - t0);
        assert_eq!(s.quality, m.quality());
        assert_eq!(s.odom_anchor[0], r.odom_anchors[m.id].translation.x);
        assert_eq!(s.anchor[1], r.anchors[m.id].translation.y);
    }
    assert_eq!(rep.loops.len(), r.loops.len());
    for (l, c) in rep.loops.iter().zip(&r.loops) {
        assert_eq!((l.a, l.b, l.manual), (c.a, c.b, c.manual));
        assert_eq!(l.accepted, c.verdict == LoopVerdict::Accepted);
        assert_eq!(l.reason.is_none(), l.accepted);
    }
    let g = rep.graph.as_ref().unwrap();
    assert!(g.final_cost <= g.initial_cost);
    let v: serde_json::Value = serde_json::to_value(&rep).unwrap();
    for key in ["submaps", "loops", "graph", "start_end_gap", "path_length"] {
        assert!(v.get(key).is_some(), "{key} missing");
    }
}

// --- loop handling of the pose graph (Ruling C) ---

fn planar(x: f64, y: f64, yaw: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, y, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
    )
}

/// 40 anchors evenly along a 10 m x 4 m rectangle, heading along the side.
fn rectangle_truth() -> Vec<Isometry3<f64>> {
    let (w, h) = (10.0, 4.0);
    let perimeter = 2.0 * (w + h);
    (0..40)
        .map(|i| {
            let s = perimeter * i as f64 / 40.0;
            if s < w {
                planar(s, 0.0, 0.0)
            } else if s < w + h {
                planar(w, s - w, FRAC_PI_2)
            } else if s < 2.0 * w + h {
                planar(w - (s - w - h), h, PI)
            } else {
                planar(0.0, h - (s - 2.0 * w - h), -FRAC_PI_2)
            }
        })
        .collect()
}

/// Odometry anchors with 0.5° yaw error per submap.
fn drifted(truth: &[Isometry3<f64>]) -> Vec<Isometry3<f64>> {
    let yaw_error = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5f64.to_radians());
    let mut out = vec![truth[0]];
    for i in 0..truth.len() - 1 {
        let mut z = truth[i].inverse() * truth[i + 1];
        z.rotation = yaw_error * z.rotation;
        out.push(out[i] * z);
    }
    out
}

fn accepted(a: usize, b: usize, relative: Isometry3<f64>) -> LoopCheck {
    LoopCheck {
        a,
        b,
        relative,
        inlier_ratio: 1.0,
        rms: 0.0,
        degeneracy: 0.2,
        verdict: LoopVerdict::Accepted,
        manual: false,
        marked: None,
    }
}

fn max_error(nodes: &[Isometry3<f64>], truth: &[Isometry3<f64>]) -> f64 {
    nodes
        .iter()
        .zip(truth)
        .map(|(n, t)| (n.translation.vector - t.translation.vector).norm())
        .fold(0.0, f64::max)
}

/// The same graph optimised with the final kernel alone (Cauchy c = 1 from the odometry).
fn cauchy_only(odom: &[Isometry3<f64>], loops: &[LoopCheck]) -> Vec<Isometry3<f64>> {
    use driftless_graph::{optimize, Edge, PoseGraph};
    let info = Matrix6::from_diagonal(&driftless_traj::nalgebra::Vector6::new(
        1e4, 1e4, 1e4, 1e3, 1e3, 1e3,
    ));
    let mut edges: Vec<Edge> = (0..odom.len() - 1)
        .map(|i| Edge {
            from: i,
            to: i + 1,
            measurement: odom[i].inverse() * odom[i + 1],
            information: info,
            robust: None,
        })
        .collect();
    edges.extend(loops.iter().map(|l| Edge {
        from: l.a,
        to: l.b,
        measurement: l.relative,
        information: info,
        robust: Some(1.0),
    }));
    let mut g = PoseGraph {
        nodes: odom.to_vec(),
        edges,
        fixed: 0,
    };
    optimize(&mut g, 100);
    g.nodes
}

#[test]
fn true_loop_closes_under_large_drift() {
    let truth = rectangle_truth();
    let odom = drifted(&truth);
    let before = max_error(&odom, &truth);
    let mut loops = vec![accepted(0, 39, truth[0].inverse() * truth[39])];

    let plain = max_error(&cauchy_only(&odom, &loops), &truth);
    let quality = vec![1.0; odom.len() - 1];
    let (anchors, report) = close_loops(&odom, &quality, &mut loops);
    let after = max_error(&anchors, &truth);
    eprintln!("max error: odometry {before:.3}, cauchy only {plain:.3}, close_loops {after:.3} m");

    assert!(before > 1.0, "drift too small to test: {before} m");
    assert!(
        plain > 0.5 * before,
        "Cauchy alone already closes: {plain} m"
    );
    assert_eq!(loops[0].verdict, LoopVerdict::Accepted);
    assert!(report.is_some());
    assert!(after < 0.1 * before, "max error {before} -> {after} m");
    assert_eq!(anchors[0], odom[0], "fixed anchor moved");
}

#[test]
fn false_loop_is_rejected_as_inconsistent() {
    let truth = rectangle_truth();
    let odom = drifted(&truth);
    let before = max_error(&odom, &truth);
    let mut wrong = truth[10].inverse() * truth[30];
    wrong.translation.vector += Vector3::new(3.0, 0.0, 0.0);
    let mut loops = vec![
        accepted(0, 39, truth[0].inverse() * truth[39]),
        accepted(10, 30, wrong),
    ];
    let quality = vec![1.0; odom.len() - 1];
    let (anchors, _) = close_loops(&odom, &quality, &mut loops);
    let after = max_error(&anchors, &truth);
    eprintln!("max error: odometry {before:.3}, with false loop {after:.3} m");

    assert_eq!(loops[0].verdict, LoopVerdict::Accepted);
    assert_eq!(
        loops[1].verdict,
        LoopVerdict::Rejected("widerspricht anderen Schleifen")
    );
    assert!(after < 0.1 * before, "max error {before} -> {after} m");

    // Without the false loop the result is the same.
    let mut only_true = vec![loops[0].clone()];
    let (clean, _) = close_loops(&odom, &quality, &mut only_true);
    assert_eq!(anchors, clean);
}

#[test]
fn no_accepted_loop_keeps_odometry() {
    let truth = rectangle_truth();
    let odom = drifted(&truth);
    let mut rejected = accepted(0, 39, truth[0].inverse() * truth[39]);
    rejected.verdict = LoopVerdict::Rejected("Restfehler zu groß");
    let mut loops = vec![rejected];
    let (anchors, report) = close_loops(&odom, &vec![1.0; odom.len() - 1], &mut loops);
    assert_eq!(anchors, odom);
    assert!(report.is_none());
}
