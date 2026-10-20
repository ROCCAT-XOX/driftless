//! Levenberg-Marquardt on the pose graph.
//!
//! Each node `i` (except [`PoseGraph::fixed`]) gets a right perturbation `T_i · Exp(δ_i)`.
//! Edge Jacobians come from central differences, the normal equations are assembled block by
//! block in edge-list order into a sparse matrix and solved with a sparse Cholesky
//! factorisation. Robust edges use iteratively reweighted least squares with a Cauchy kernel.

use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector, Isometry3, Matrix6, Vector6};
use nalgebra_sparse::factorization::CscCholesky;
use nalgebra_sparse::{CooMatrix, CscMatrix};

use crate::se3::{exp, residual};

/// Step of the central differences for the edge Jacobians.
const JACOBIAN_STEP: f64 = 1e-6;
/// Initial Levenberg-Marquardt damping and its bounds.
const LAMBDA_INIT: f64 = 1e-4;
const LAMBDA_MIN: f64 = 1e-12;
const LAMBDA_MAX: f64 = 1e12;
/// Floor for the damped diagonal, so nodes without constraints stay solvable.
const DIAGONAL_FLOOR: f64 = 1e-12;
/// Convergence: largest gradient entry, largest step entry, relative cost decrease.
const GRADIENT_TOL: f64 = 1e-10;
const STEP_TOL: f64 = 1e-10;
const COST_TOL: f64 = 1e-12;

/// Relative measurement between two nodes.
#[derive(Debug, Clone)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    /// Expected `T_from⁻¹ · T_to`.
    pub measurement: Isometry3<f64>,
    /// Information matrix (inverse covariance) of the residual `[rotation; translation]`.
    pub information: Matrix6<f64>,
    /// `Some(c)`: Cauchy kernel of width `c` on the Mahalanobis norm `sqrt(eᵀΩe)`.
    pub robust: Option<f64>,
}

/// Absolute node poses and the relative edges between them.
#[derive(Debug, Clone)]
pub struct PoseGraph {
    pub nodes: Vec<Isometry3<f64>>,
    pub edges: Vec<Edge>,
    /// Index of the node that stays fixed (gauge); usually 0.
    pub fixed: usize,
}

/// Outcome of [`optimize`].
#[derive(Debug, Clone)]
pub struct OptimizeReport {
    /// Levenberg-Marquardt trials (accepted and rejected steps).
    pub iterations: u32,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub converged: bool,
    /// Cost of each edge after the optimisation, in edge-list order.
    pub edge_cost: Vec<f64>,
}

/// Optimises the node poses in place with at most `max_iter` Levenberg-Marquardt trials.
///
/// Cost per edge is `½ eᵀΩe`, or `½ c² ln(1 + eᵀΩe / c²)` with a Cauchy kernel of width `c`.
///
/// # Panics
/// If an edge or `fixed` refers to a node that does not exist (with at least one node).
pub fn optimize(graph: &mut PoseGraph, max_iter: u32) -> OptimizeReport {
    let n = graph.nodes.len();
    if n > 0 {
        assert!(
            graph.fixed < n,
            "fixed node {} out of range ({n} nodes)",
            graph.fixed
        );
    }
    for (k, e) in graph.edges.iter().enumerate() {
        assert!(
            e.from < n && e.to < n,
            "edge {k} ({} -> {}) out of range ({n} nodes)",
            e.from,
            e.to
        );
    }

    let initial_cost = total_cost(graph);
    let mut cost = initial_cost;
    let mut iterations = 0;
    let mut converged = false;
    let mut lambda = LAMBDA_INIT;
    let dim = 6 * n.saturating_sub(1);

    'outer: while dim > 0 && !graph.edges.is_empty() {
        let system = linearize(graph);
        if system.gradient.amax() <= GRADIENT_TOL {
            converged = true;
            break;
        }
        loop {
            if iterations >= max_iter {
                break 'outer;
            }
            iterations += 1;
            let accepted = solve(&system, lambda, dim).map(|step| {
                let previous = graph.nodes.clone();
                apply(graph, &step);
                let new_cost = total_cost(graph);
                if new_cost < cost {
                    (step, new_cost)
                } else {
                    graph.nodes = previous;
                    (step, f64::INFINITY)
                }
            });
            match accepted {
                Some((step, new_cost)) if new_cost.is_finite() => {
                    let decrease = (cost - new_cost) / cost;
                    cost = new_cost;
                    lambda = (lambda / 10.0).max(LAMBDA_MIN);
                    if step.amax() <= STEP_TOL || decrease <= COST_TOL {
                        converged = true;
                        break 'outer;
                    }
                    continue 'outer;
                }
                _ => {
                    lambda *= 10.0;
                    if lambda > LAMBDA_MAX {
                        break 'outer;
                    }
                }
            }
        }
    }
    if dim == 0 || graph.edges.is_empty() {
        converged = true;
    }

    let edge_cost = graph.edges.iter().map(|e| edge_cost(graph, e).0).collect();
    OptimizeReport {
        iterations,
        initial_cost,
        final_cost: cost,
        converged,
        edge_cost,
    }
}

/// Linearised normal equations `H δ = -g` around the current poses.
struct System {
    /// 6x6 blocks of `H` keyed by (row variable, column variable), both triangles.
    blocks: BTreeMap<(usize, usize), Matrix6<f64>>,
    gradient: DVector<f64>,
}

/// First variable index of `node`, `None` for the fixed node.
fn variable(graph: &PoseGraph, node: usize) -> Option<usize> {
    use std::cmp::Ordering;
    match node.cmp(&graph.fixed) {
        Ordering::Less => Some(6 * node),
        Ordering::Equal => None,
        Ordering::Greater => Some(6 * (node - 1)),
    }
}

/// Cost and IRLS weight of one edge at the current poses.
fn edge_cost(graph: &PoseGraph, edge: &Edge) -> (f64, f64) {
    let e = residual(
        &graph.nodes[edge.from],
        &graph.nodes[edge.to],
        &edge.measurement,
    );
    robust_cost((e.transpose() * edge.information * e)[0], edge.robust)
}

/// `(cost, weight)` for the squared Mahalanobis norm `s`.
fn robust_cost(s: f64, robust: Option<f64>) -> (f64, f64) {
    match robust {
        None => (0.5 * s, 1.0),
        Some(c) => {
            let c2 = c * c;
            (0.5 * c2 * (s / c2).ln_1p(), 1.0 / (1.0 + s / c2))
        }
    }
}

fn total_cost(graph: &PoseGraph) -> f64 {
    graph.edges.iter().map(|e| edge_cost(graph, e).0).sum()
}

/// Jacobian of the edge residual with respect to the right perturbation of `from` (or `to`).
fn jacobian(
    edge: &Edge,
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
    wrt_from: bool,
) -> Matrix6<f64> {
    let mut j = Matrix6::zeros();
    for k in 0..6 {
        let mut d = Vector6::zeros();
        d[k] = JACOBIAN_STEP;
        let (plus, minus) = if wrt_from {
            (
                residual(&(from * exp(&d)), to, &edge.measurement),
                residual(&(from * exp(&-d)), to, &edge.measurement),
            )
        } else {
            (
                residual(from, &(to * exp(&d)), &edge.measurement),
                residual(from, &(to * exp(&-d)), &edge.measurement),
            )
        };
        j.set_column(k, &((plus - minus) / (2.0 * JACOBIAN_STEP)));
    }
    j
}

fn linearize(graph: &PoseGraph) -> System {
    let dim = 6 * (graph.nodes.len() - 1);
    let mut blocks: BTreeMap<(usize, usize), Matrix6<f64>> = BTreeMap::new();
    let mut gradient = DVector::zeros(dim);
    for edge in &graph.edges {
        let (from, to) = (&graph.nodes[edge.from], &graph.nodes[edge.to]);
        let e = residual(from, to, &edge.measurement);
        let (_, w) = edge_cost(graph, edge);
        let omega = edge.information * w;
        let vars = [
            (variable(graph, edge.from), jacobian(edge, from, to, true)),
            (variable(graph, edge.to), jacobian(edge, from, to, false)),
        ];
        for (vi, ji) in &vars {
            let Some(vi) = *vi else { continue };
            let jt_omega = ji.transpose() * omega;
            let mut g = gradient.fixed_rows_mut::<6>(vi);
            g += jt_omega * e;
            for (vj, jj) in &vars {
                let Some(vj) = *vj else { continue };
                *blocks.entry((vi, vj)).or_insert_with(Matrix6::zeros) += jt_omega * jj;
            }
        }
    }
    System { blocks, gradient }
}

/// Solves `(H + λ diag(H)) δ = -g`; `None` if the factorisation fails.
fn solve(system: &System, lambda: f64, dim: usize) -> Option<DVector<f64>> {
    let mut coo = CooMatrix::new(dim, dim);
    for (&(vi, vj), block) in &system.blocks {
        for c in 0..6 {
            for r in 0..6 {
                let mut v = block[(r, c)];
                if vi == vj && r == c {
                    v += lambda * v.max(DIAGONAL_FLOOR);
                }
                coo.push(vi + r, vj + c, v);
            }
        }
    }
    // Variables without any block (isolated nodes) still need a diagonal entry.
    for i in (0..dim).step_by(6) {
        if !system.blocks.contains_key(&(i, i)) {
            for r in 0..6 {
                coo.push(i + r, i + r, lambda * DIAGONAL_FLOOR);
            }
        }
    }
    let h = CscMatrix::from(&coo);
    let chol = CscCholesky::factor(&h).ok()?;
    let rhs = DMatrix::from_column_slice(dim, 1, (-&system.gradient).as_slice());
    let step = chol.solve(&rhs);
    let step = DVector::from_column_slice(step.as_slice());
    step.iter().all(|v| v.is_finite()).then_some(step)
}

/// `T_i ← T_i · Exp(δ_i)` for every free node.
fn apply(graph: &mut PoseGraph, step: &DVector<f64>) {
    for i in 0..graph.nodes.len() {
        if let Some(v) = variable(graph, i) {
            let d: Vector6<f64> = step.fixed_rows::<6>(v).into_owned();
            graph.nodes[i] *= exp(&d);
        }
    }
}
