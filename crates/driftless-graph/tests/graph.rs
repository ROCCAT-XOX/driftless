use driftless_graph::nalgebra::{Isometry3, Matrix6, Translation3, UnitQuaternion, Vector3};
use driftless_graph::{optimize, Edge, PoseGraph};

/// Isometry from a planar position and heading.
fn planar(x: f64, y: f64, yaw: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, y, 0.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
    )
}

/// Odometry information: 1 cm / 0.01 rad standard deviation.
const ODOM_INFO: f64 = 1e4;
/// Loop edges carry 100x the odometry information.
const LOOP_INFO: f64 = 100.0 * ODOM_INFO;

fn edge(from: usize, to: usize, measurement: Isometry3<f64>, information: f64) -> Edge {
    Edge {
        from,
        to,
        measurement,
        information: Matrix6::identity() * information,
        robust: None,
    }
}

/// Ground truth: 40 poses evenly spaced along a 4 m x 10 m rectangle, heading along the side.
fn rectangle_truth() -> Vec<Isometry3<f64>> {
    let (w, h) = (10.0, 4.0);
    let perimeter = 2.0 * (w + h);
    (0..40)
        .map(|i| {
            let s = perimeter * i as f64 / 40.0;
            if s < w {
                planar(s, 0.0, 0.0)
            } else if s < w + h {
                planar(w, s - w, std::f64::consts::FRAC_PI_2)
            } else if s < 2.0 * w + h {
                planar(w - (s - w - h), h, std::f64::consts::PI)
            } else {
                planar(0.0, h - (s - 2.0 * w - h), -std::f64::consts::FRAC_PI_2)
            }
        })
        .collect()
}

/// Rectangle graph: odometry with 0.5 deg yaw error per step, nodes initialised by chaining
/// the odometry, and one loop edge last -> first with the true measurement and 100x information.
fn drifted_rectangle() -> (PoseGraph, Vec<Isometry3<f64>>) {
    let truth = rectangle_truth();
    let yaw_error = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.5f64.to_radians());
    let mut nodes = vec![truth[0]];
    let mut edges = Vec::new();
    for i in 0..truth.len() - 1 {
        let mut z = truth[i].inverse() * truth[i + 1];
        z.rotation = yaw_error * z.rotation;
        nodes.push(nodes[i] * z);
        edges.push(edge(i, i + 1, z, ODOM_INFO));
    }
    let last = truth.len() - 1;
    edges.push(edge(last, 0, truth[last].inverse() * truth[0], LOOP_INFO));
    (
        PoseGraph {
            nodes,
            edges,
            fixed: 0,
        },
        truth,
    )
}

fn position_errors(nodes: &[Isometry3<f64>], truth: &[Isometry3<f64>]) -> Vec<f64> {
    nodes
        .iter()
        .zip(truth)
        .map(|(n, t)| (n.translation.vector - t.translation.vector).norm())
        .collect()
}

fn max(v: &[f64]) -> f64 {
    v.iter().copied().fold(0.0, f64::max)
}

fn rms(v: &[f64]) -> f64 {
    (v.iter().map(|e| e * e).sum::<f64>() / v.len() as f64).sqrt()
}

#[test]
fn chain_without_loop_stays_put() {
    let truth: Vec<Isometry3<f64>> = (0..20)
        .map(|i| {
            let a = i as f64 * 0.1;
            Isometry3::from_parts(
                Translation3::new(i as f64, 0.3 * a.sin(), 0.1 * a),
                UnitQuaternion::from_euler_angles(0.05 * a, -0.02 * a, a),
            )
        })
        .collect();
    let edges = (0..19)
        .map(|i| edge(i, i + 1, truth[i].inverse() * truth[i + 1], 1.0))
        .collect();
    let mut graph = PoseGraph {
        nodes: truth.clone(),
        edges,
        fixed: 0,
    };
    let report = optimize(&mut graph, 50);
    assert!(report.initial_cost < 1e-20, "{report:?}");
    assert!(report.final_cost < 1e-20, "{report:?}");
    assert!(report.converged);
    for (n, t) in graph.nodes.iter().zip(&truth) {
        assert!((n.translation.vector - t.translation.vector).norm() <= 1e-9);
        assert!(n.rotation.angle_to(&t.rotation) <= 1e-9);
    }
}

#[test]
fn loop_closure_removes_injected_drift() {
    let (mut graph, truth) = drifted_rectangle();
    let before = position_errors(&graph.nodes, &truth);
    let report = optimize(&mut graph, 100);
    let after = position_errors(&graph.nodes, &truth);
    assert!(report.converged, "{report:?}");
    assert!(report.final_cost < report.initial_cost);
    assert_eq!(report.edge_cost.len(), graph.edges.len());
    assert!(max(&before) > 1.0, "drift too small to test: {before:?}");
    // Aggregate over all nodes: RMS and maximum must both drop below 10 %.
    assert!(
        rms(&after) < 0.1 * rms(&before),
        "rms {} -> {}",
        rms(&before),
        rms(&after)
    );
    assert!(
        max(&after) < 0.1 * max(&before),
        "max {} -> {}",
        max(&before),
        max(&after)
    );
    assert!(after[39] < 0.05, "last node {} m off", after[39]);
    assert_eq!(graph.nodes[0], truth[0], "fixed node moved");
}

/// Wrong loop edge 10 <-> 30 whose measurement is shifted by 3 m.
fn false_loop(truth: &[Isometry3<f64>], robust: Option<f64>) -> Edge {
    let mut z = truth[10].inverse() * truth[30];
    z.translation.vector += Vector3::new(3.0, 0.0, 0.0);
    Edge {
        robust,
        ..edge(10, 30, z, LOOP_INFO)
    }
}

#[test]
fn cauchy_limits_false_loop() {
    let (mut robust_graph, truth) = drifted_rectangle();
    robust_graph.edges.push(false_loop(&truth, Some(1.0)));
    let mut plain_graph = robust_graph.clone();
    plain_graph.edges.last_mut().unwrap().robust = None;

    let report = optimize(&mut robust_graph, 100);
    let robust_err = max(&position_errors(&robust_graph.nodes, &truth));
    optimize(&mut plain_graph, 100);
    let plain_err = max(&position_errors(&plain_graph.nodes, &truth));

    assert!(report.converged, "{report:?}");
    assert!(robust_err < 0.3, "robust: max error {robust_err} m");
    assert!(plain_err > 1.0, "plain: max error {plain_err} m");
}

#[test]
fn optimize_is_deterministic() {
    let (mut a, truth) = drifted_rectangle();
    a.edges.push(false_loop(&truth, Some(1.0)));
    let mut b = a.clone();
    let ra = optimize(&mut a, 100);
    let rb = optimize(&mut b, 100);
    assert_eq!(format!("{:?}", a.nodes), format!("{:?}", b.nodes));
    for (x, y) in a.nodes.iter().zip(&b.nodes) {
        let (x, y) = (x.to_homogeneous(), y.to_homogeneous());
        assert!(x
            .iter()
            .zip(y.iter())
            .all(|(p, q)| p.to_bits() == q.to_bits()));
    }
    assert_eq!(ra.iterations, rb.iterations);
    assert_eq!(ra.final_cost.to_bits(), rb.final_cost.to_bits());
    let bits = |r: &driftless_graph::OptimizeReport| -> Vec<u64> {
        r.edge_cost.iter().map(|c| c.to_bits()).collect()
    };
    assert_eq!(bits(&ra), bits(&rb));
}

#[test]
fn single_node_graph_is_noop() {
    let node = planar(1.0, 2.0, 0.3);
    let mut graph = PoseGraph {
        nodes: vec![node],
        edges: Vec::new(),
        fixed: 0,
    };
    let report = optimize(&mut graph, 10);
    assert_eq!(graph.nodes, vec![node]);
    assert_eq!(report.iterations, 0);
    assert!(report.converged);
    assert_eq!(report.initial_cost, 0.0);
    assert_eq!(report.final_cost, 0.0);
    assert!(report.edge_cost.is_empty());
}
