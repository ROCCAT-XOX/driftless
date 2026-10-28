//! The SLAM pipeline: the odometry is cut into submaps, loops between submaps are searched
//! and verified, and a pose graph over the submap anchors closes the accepted loops. The
//! points inside a submap never move; only the anchors do.

use crate::loops::{
    find_candidates, match_manual_loops, match_rejected_loops, position_only_relative, verify,
    verify_manual, LoopCheck, LoopConfig, LoopVerdict, ManualLoop, ManualLoopError, ManualMatch,
    RejectedLoop,
};
use crate::submap::{Submap, SubmapBuilder, SubmapConfig};
use driftless_graph::se3::residual;
use driftless_graph::{optimize, Edge, OptimizeReport, PoseGraph};
use driftless_io::msg::{ImuSample, LidarFrame};
use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use driftless_odom::{run_odometry_with, OdomConfig, OdomStats, OUTPUT_MAP_VOXEL};
use driftless_traj::Pose;
use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use std::collections::HashSet;

/// Information of an edge residual: rotation (per rad²) and translation (per m²).
const ROT_INFO: f64 = 1e4;
const TRANS_INFO: f64 = 1e3;
/// Translation information (per m²) of a manual loop whose ICP failed; its rotation is free.
/// As stiff as a rotation-wise accepted loop: the user states that the places are the same.
const POSITION_ONLY_INFO: f64 = 1e4;
/// The measurement of a position-only loop is recomputed until it moves less than this (m)…
const POSITION_ONLY_TOL: f64 = 1e-3;
/// …or this many optimisations have run.
const POSITION_ONLY_MAX_SOLVES: u32 = 10;
/// Odometry edges keep at least this share of their information, however bad the submap.
const MIN_ODOM_WEIGHT: f64 = 0.05;
/// Cauchy width of the loop edges. Wide (nearly quadratic within 100 σ), so a loop pulls in
/// a large drift; a tight kernel (c = 1) caps a loop's cost at a few units and drops true
/// loops once spreading the drift over a long odometry chain costs more than that. Wrong
/// loops are caught by the consistency check ([`MAX_LOOP_RESIDUAL`]) instead.
const LOOP_ROBUST: f64 = 100.0;
/// Largest whitened residual `sqrt(eᵀΩe)` of an accepted loop after the optimisation.
const MAX_LOOP_RESIDUAL: f64 = 3.0;
/// Levenberg-Marquardt trials per optimisation.
const MAX_ITER: u32 = 100;
/// Verdict of a loop the optimised graph does not fit.
pub const INCONSISTENT: &str = "widerspricht anderen Schleifen";
/// Verdict of an automatic loop the user switched off (`[[reject]]` of `corrections.toml`).
pub const VETOED: &str = "von Hand abgeschaltet";

#[derive(Debug, Clone)]
pub struct SlamConfig {
    pub odom: OdomConfig,
    pub submap: SubmapConfig,
    pub loops: LoopConfig,
    /// Rounds of loop search → verification → optimisation; each round searches from the
    /// anchors of the previous one.
    pub rounds: u32,
}

impl Default for SlamConfig {
    fn default() -> Self {
        SlamConfig {
            odom: OdomConfig::default(),
            submap: SubmapConfig::default(),
            loops: LoopConfig::default(),
            rounds: 2,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SlamResult {
    pub submaps: Vec<Submap>,
    /// Optimised anchor per submap (index = id).
    pub anchors: Vec<Isometry3<f64>>,
    /// Odometry anchor per submap.
    pub odom_anchors: Vec<Isometry3<f64>>,
    /// Every verified loop candidate, sorted by `(b, a)`.
    pub loops: Vec<LoopCheck>,
    /// `None` if nothing was optimised (fewer than two submaps or no accepted loop); the
    /// anchors are then the odometry anchors.
    pub graph: Option<OptimizeReport>,
}

impl SlamResult {
    /// Corrected pose of every frame: its submap's optimised anchor times its offset.
    pub fn poses(&self) -> Vec<Pose> {
        frame_poses(&self.submaps, &self.anchors)
    }

    /// Odometry pose of every frame (before loop closure).
    pub fn odom_poses(&self) -> Vec<Pose> {
        frame_poses(&self.submaps, &self.odom_anchors)
    }

    /// Points of all submaps at the optimised anchors, one per [`OUTPUT_MAP_VOXEL`] voxel, in
    /// submap order.
    pub fn map_points(&self) -> Vec<[f32; 3]> {
        let mut seen: HashSet<VoxelKey, FixedState> = HashSet::default();
        let mut out = Vec::new();
        for (m, anchor) in self.submaps.iter().zip(&self.anchors) {
            for p in m.world_points(anchor) {
                let v = Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
                if seen.insert(voxel_key(&v, OUTPUT_MAP_VOXEL)) {
                    out.push(p);
                }
            }
        }
        out
    }
}

fn frame_poses(submaps: &[Submap], anchors: &[Isometry3<f64>]) -> Vec<Pose> {
    submaps
        .iter()
        .zip(anchors)
        .flat_map(|(m, anchor)| {
            m.frames.iter().map(move |f| {
                let pose = anchor * f.offset;
                Pose {
                    t: f.t,
                    p: pose.translation.vector,
                    q: pose.rotation,
                }
            })
        })
        .collect()
}

/// Odometry and submaps in one pass over the frames: the input of [`solve`]. Returns the
/// submaps and the odometry statistics.
pub fn build_submaps(
    frames: impl Iterator<Item = LidarFrame>,
    imu: &[ImuSample],
    cfg: &SlamConfig,
) -> (Vec<Submap>, OdomStats) {
    let mut builder = SubmapBuilder::new(cfg.submap.clone());
    let odom = run_odometry_with(frames, imu, &cfg.odom, &mut |f| builder.push(f));
    (builder.finish(), odom.stats)
}

/// Odometry, submaps and automatic loop closure in one pass over the frames. Returns the
/// result and the odometry statistics.
pub fn run_slam(
    frames: impl Iterator<Item = LidarFrame>,
    imu: &[ImuSample],
    cfg: &SlamConfig,
) -> (SlamResult, OdomStats) {
    let (submaps, stats) = build_submaps(frames, imu, cfg);
    let odom_anchors = submaps.iter().map(|m| m.anchor).collect();
    (solve_matched(submaps, odom_anchors, &[], &[], cfg), stats)
}

/// The graph part of the pipeline on finished submaps: `cfg.rounds` rounds of loop search
/// and verification from the current anchors, each followed by [`close_loops`] from the
/// odometry anchors. A round that optimises nothing ends the search (the next one would
/// see the same anchors).
///
/// `manual` are the loops the user marked (times since [`crate::scan_start`]). They are
/// verified in every round like the automatic ones ([`verify_manual`]), join the graph
/// without robust kernel and are never rejected as inconsistent; a pair of submaps with a
/// manual loop gets no automatic one. Fails if a manual loop does not match the submaps
/// ([`match_manual_loops`]).
pub fn solve(
    submaps: Vec<Submap>,
    odom_anchors: Vec<Isometry3<f64>>,
    manual: &[ManualLoop],
    cfg: &SlamConfig,
) -> Result<SlamResult, ManualLoopError> {
    solve_with(submaps, odom_anchors, manual, &[], cfg)
}

/// [`solve`] with the automatic loops the user switched off (`rejected`, the `[[reject]]`
/// entries of `corrections.toml`): such a loop is verified as before, so the report lists it
/// with its numbers, but gets the verdict `Rejected(`[`VETOED`]`)` and so stays out of the
/// pose graph. Manual loops are never touched by it. Fails also if a rejected loop does not
/// name two different submaps ([`match_rejected_loops`]).
pub fn solve_with(
    submaps: Vec<Submap>,
    odom_anchors: Vec<Isometry3<f64>>,
    manual: &[ManualLoop],
    rejected: &[RejectedLoop],
    cfg: &SlamConfig,
) -> Result<SlamResult, ManualLoopError> {
    let manual = match_manual_loops(&submaps, manual)?;
    let vetoed = match_rejected_loops(&submaps, rejected)?;
    Ok(solve_matched(submaps, odom_anchors, &manual, &vetoed, cfg))
}

fn solve_matched(
    submaps: Vec<Submap>,
    odom_anchors: Vec<Isometry3<f64>>,
    manual: &[ManualMatch],
    vetoed: &[(usize, usize)],
    cfg: &SlamConfig,
) -> SlamResult {
    assert_eq!(submaps.len(), odom_anchors.len(), "one anchor per submap");
    let quality = odometry_quality(&submaps);
    let mut anchors = odom_anchors.clone();
    let mut loops: Vec<LoopCheck> = Vec::new();
    let mut graph = None;
    for _ in 0..cfg.rounds {
        let candidates = find_candidates(&submaps, &anchors, &cfg.loops)
            .into_iter()
            .filter(|c| !manual.iter().any(|m| (m.a, m.b) == (c.a, c.b)));
        let checks = candidates
            .map(|c| {
                let mut check = verify(&submaps, &anchors, &c, &cfg.loops);
                // switched off by hand: verified for the report, never in the graph
                if vetoed.contains(&(check.a, check.b)) {
                    check.verdict = LoopVerdict::Rejected(VETOED);
                }
                check
            })
            .chain(
                manual
                    .iter()
                    .map(|m| verify_manual(&submaps, &anchors, m, &cfg.loops)),
            )
            .collect::<Vec<_>>();
        for check in checks {
            match loops
                .iter_mut()
                .find(|l| (l.a, l.b, l.manual) == (check.a, check.b, check.manual))
            {
                Some(l) => *l = check,
                None => loops.push(check),
            }
        }
        loops.sort_by_key(|l| (l.b, l.a, l.manual));
        (anchors, graph) = close_loops(&odom_anchors, &quality, &mut loops);
        if graph.is_none() {
            break;
        }
    }
    SlamResult {
        submaps,
        anchors,
        odom_anchors,
        loops,
        graph,
    }
}

/// Trust in the odometry edge from submap `s` to `s + 1`: the worse of submap `s` (its
/// frames make up the step) and the first frame of `s + 1` (its pose is the next anchor).
fn odometry_quality(submaps: &[Submap]) -> Vec<f32> {
    submaps
        .windows(2)
        .map(|w| w[0].quality().min(w[1].frames[0].quality.score()))
        .collect()
}

fn information(weight: f64) -> Matrix6<f64> {
    Matrix6::from_diagonal(&Vector6::new(
        ROT_INFO, ROT_INFO, ROT_INFO, TRANS_INFO, TRANS_INFO, TRANS_INFO,
    )) * weight
}

/// The graph edge of a loop that takes part in the optimisation, `None` for a rejected one.
/// Automatic loops get the wide Cauchy kernel, manual ones none (the user's decision);
/// a [`LoopVerdict::PositionOnly`] loop constrains the translation only.
fn loop_edge(l: &LoopCheck) -> Option<Edge> {
    let information = match l.verdict {
        LoopVerdict::Accepted => information(1.0),
        LoopVerdict::PositionOnly(_) => Matrix6::from_diagonal(&Vector6::new(
            0.0,
            0.0,
            0.0,
            POSITION_ONLY_INFO,
            POSITION_ONLY_INFO,
            POSITION_ONLY_INFO,
        )),
        LoopVerdict::Rejected(_) => return None,
    };
    Some(Edge {
        from: l.a,
        to: l.b,
        measurement: l.relative,
        information,
        robust: (!l.manual).then_some(LOOP_ROBUST),
    })
}

/// Pose graph over the anchors with the odometry edges (`odom_quality[s]` = trust in the
/// edge `s → s + 1`) and the accepted loops in `loops`, optimised from the odometry anchors.
/// Afterwards the accepted automatic loop with the largest whitened residual `sqrt(eᵀΩe)`
/// above [`MAX_LOOP_RESIDUAL`] is marked `Rejected(INCONSISTENT)` and the graph is solved
/// again from the odometry without it, until every remaining loop fits. Removing only the
/// worst one per pass keeps true loops that a wrong one had bent out of shape. Manual loops
/// (accepted or position only) always stay.
///
/// Returns the anchors and the report of the last optimisation; without an accepted loop the
/// odometry anchors and `None`.
pub fn close_loops(
    odom_anchors: &[Isometry3<f64>],
    odom_quality: &[f32],
    loops: &mut [LoopCheck],
) -> (Vec<Isometry3<f64>>, Option<OptimizeReport>) {
    assert_eq!(
        odom_quality.len(),
        odom_anchors.len().saturating_sub(1),
        "one quality per odometry edge"
    );
    let odom_edges: Vec<Edge> = odom_quality
        .iter()
        .enumerate()
        .map(|(s, &q)| Edge {
            from: s,
            to: s + 1,
            measurement: odom_anchors[s].inverse() * odom_anchors[s + 1],
            information: information(f64::from(q).max(MIN_ODOM_WEIGHT)),
            robust: None,
        })
        .collect();
    loop {
        let (active, edges): (Vec<usize>, Vec<Edge>) = loops
            .iter()
            .enumerate()
            .filter_map(|(k, l)| loop_edge(l).map(|e| (k, e)))
            .unzip();
        if active.is_empty() || odom_anchors.len() < 2 {
            return (odom_anchors.to_vec(), None);
        }
        let mut graph = PoseGraph {
            nodes: odom_anchors.to_vec(),
            edges: odom_edges.clone(),
            fixed: 0,
        };
        graph.edges.extend(edges);
        let report = optimize_position_only(&mut graph, odom_edges.len(), &active, loops);

        let worst = active
            .iter()
            .zip(&graph.edges[odom_edges.len()..])
            .filter(|(&k, _)| !loops[k].manual)
            .map(|(&k, e)| {
                let r = residual(&graph.nodes[e.from], &graph.nodes[e.to], &e.measurement);
                ((r.transpose() * e.information * r)[0].sqrt(), k)
            })
            .filter(|&(r, _)| r.is_nan() || r > MAX_LOOP_RESIDUAL)
            .max_by(|x, y| x.0.total_cmp(&y.0).then(y.1.cmp(&x.1)));
        match worst {
            Some((_, k)) => loops[k].verdict = LoopVerdict::Rejected(INCONSISTENT),
            None => return (graph.nodes, Some(report)),
        }
    }
}

/// Optimises `graph` (loop edge `j` = `graph.edges[first_loop + j]` belongs to
/// `loops[active[j]]`). A position-only loop must bring its two marked frames together, but
/// where anchor `b` has to sit for that depends on the relative rotation of the two anchors,
/// which the optimisation changes. So its measurement is recomputed from the optimised
/// anchors ([`position_only_relative`]) and the graph optimised again from there, until no
/// measurement moves by [`POSITION_ONLY_TOL`] or after [`POSITION_ONLY_MAX_SOLVES`]
/// optimisations. The final measurements are written back to `relative`. The report spans
/// all optimisations (initial cost of the first, final cost of the last).
fn optimize_position_only(
    graph: &mut PoseGraph,
    first_loop: usize,
    active: &[usize],
    loops: &mut [LoopCheck],
) -> OptimizeReport {
    let mut report = optimize(graph, MAX_ITER);
    for _ in 1..POSITION_ONLY_MAX_SOLVES {
        let mut moved = 0.0f64;
        for (j, &k) in active.iter().enumerate() {
            let (LoopVerdict::PositionOnly(_), Some(marked)) = (&loops[k].verdict, loops[k].marked)
            else {
                continue;
            };
            let e = &mut graph.edges[first_loop + j];
            let rotation = (graph.nodes[e.from].inverse() * graph.nodes[e.to]).rotation;
            let z = position_only_relative(&marked, &rotation);
            moved = moved.max((z.translation.vector - e.measurement.translation.vector).norm());
            e.measurement = z;
            loops[k].relative = z;
        }
        if moved < POSITION_ONLY_TOL {
            break;
        }
        let next = optimize(graph, MAX_ITER);
        report = OptimizeReport {
            iterations: report.iterations + next.iterations,
            initial_cost: report.initial_cost,
            ..next
        };
    }
    report
}
