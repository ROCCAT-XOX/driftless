//! Loop closure candidates between submaps and their ICP verification.
//!
//! [`find_candidates`] proposes pairs of submaps whose anchors lie close together compared
//! with the drift the odometry may have collected on the way from one to the other.
//! [`verify`] registers the points of the later submap against the earlier one and accepts
//! the loop only if the alignment overlaps enough, fits tightly, is well constrained and
//! comes out the same when the roles are swapped.

use crate::report::scan_start;
use crate::submap::Submap;
use driftless_odom::icp::{align, IcpParams, IcpResult, IcpTarget};
use driftless_odom::{voxel_downsample, DEGENERACY_GOOD};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

/// Thresholds of the loop search and verification.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopConfig {
    /// Smallest id difference `b − a` of a candidate (neighbouring submaps overlap anyway).
    pub min_gap: usize,
    /// Search radius around an anchor without any path in between (m).
    pub base_radius: f64,
    /// Additional search radius per metre of path between the two submaps (assumed drift).
    pub radius_per_m: f64,
    /// Candidates per later submap `b`, nearest first.
    pub max_candidates: usize,
    /// Voxel size the source points are thinned to before ICP (m).
    pub icp_voxel: f64,
    /// Smallest share of source points with a correspondence at the final pose.
    pub min_inlier_ratio: f64,
    /// The same for a manual loop: the user vouches for the place, and the two submaps
    /// usually cover it from different stretches of the walk.
    pub manual_min_inlier_ratio: f64,
    /// Largest point-to-plane RMS at the final pose (m).
    pub max_rms: f64,
    /// Smallest [`IcpResult::degeneracy`] at the final pose.
    pub min_degeneracy: f64,
}

impl Default for LoopConfig {
    fn default() -> Self {
        LoopConfig {
            min_gap: 3,
            base_radius: 3.0,
            radius_per_m: 0.05,
            max_candidates: 3,
            icp_voxel: 0.25,
            min_inlier_ratio: 0.5,
            manual_min_inlier_ratio: 0.25,
            max_rms: 0.05,
            // The weakest position direction must be at least partly fixed by the geometry.
            min_degeneracy: 0.3 * DEGENERACY_GOOD,
        }
    }
}

/// A pair of submap ids (`a < b`) that may show the same place.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopCandidate {
    pub a: usize,
    pub b: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LoopVerdict {
    Accepted,
    /// Reason for the user (German).
    Rejected(&'static str),
    /// A manual loop whose ICP failed for the given reason (German): it only ties the two
    /// positions together ([`POSITION_ONLY`]).
    PositionOnly(&'static str),
}

/// Warning for a manual loop whose ICP failed.
pub const POSITION_ONLY: &str = "Stelle passt nicht sauber, nur Position übernommen";

/// Outcome of [`verify`].
#[derive(Debug, Clone, PartialEq)]
pub struct LoopCheck {
    pub a: usize,
    pub b: usize,
    /// Pose of the anchor of `b` in the anchor frame of `a`.
    pub relative: Isometry3<f64>,
    /// Metrics of the final ICP pose `relative`.
    pub inlier_ratio: f64,
    pub rms: f64,
    pub degeneracy: f64,
    pub verdict: LoopVerdict,
    /// Set by the user instead of found automatically.
    pub manual: bool,
    /// Manual loops: positions of the two marked frames in the anchor frames of `a` and `b`.
    pub marked: Option<[Vector3<f64>; 2]>,
}

/// A loop the user marks by hand: the places at times `a_t` and `b_t` (seconds since the
/// start of the scan, [`crate::scan_start`]) are the same.
#[derive(Debug, Clone, PartialEq)]
pub struct ManualLoop {
    pub a_t: f64,
    pub b_t: f64,
}

/// A [`ManualLoop`] matched to the submaps: the frame nearest in time to each of its two
/// times, ordered so that `a < b`.
#[derive(Debug, Clone, PartialEq)]
pub struct ManualMatch {
    pub a: usize,
    pub b: usize,
    /// Index into `submaps[a].frames` and `submaps[b].frames`.
    pub frame_a: usize,
    pub frame_b: usize,
}

/// Why manual loops cannot be used; the messages are for the user (German).
#[derive(Debug, Clone, PartialEq)]
pub enum ManualLoopError {
    /// `t` lies outside `0..=end` (seconds since the start of the scan).
    OutsideScan { t: f64, end: f64 },
    /// Both times fall into the same submap.
    SameSubmap { a_t: f64, b_t: f64, submap: usize },
    /// Two manual loops join the same pair of submaps.
    Duplicate {
        first: ManualLoop,
        second: ManualLoop,
        a: usize,
        b: usize,
    },
}

impl std::fmt::Display for ManualLoopError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ManualLoopError::OutsideScan { t, end } => write!(
                f,
                "Zeit {t} s liegt außerhalb des Scans (0 bis {end:.1} s ab Scan-Beginn)"
            ),
            ManualLoopError::SameSubmap { a_t, b_t, submap } => write!(
                f,
                "Schleife {a_t} s ↔ {b_t} s: beide Zeiten liegen in derselben Teilkarte \
                 {submap}; eine Schleife verbindet zwei verschiedene Stellen des Rundgangs"
            ),
            ManualLoopError::Duplicate {
                first,
                second,
                a,
                b,
            } => write!(
                f,
                "doppelte Schleife: {} s ↔ {} s und {} s ↔ {} s verbinden beide die \
                 Teilkarten {a} und {b}; eine davon deaktivieren",
                first.a_t, first.b_t, second.a_t, second.b_t
            ),
        }
    }
}

impl std::error::Error for ManualLoopError {}

/// Submap and frame index of the frame nearest in time to `t` (seconds since the start of
/// the scan); earlier frames win ties.
fn locate(submaps: &[Submap], t: f64) -> Result<(usize, usize), ManualLoopError> {
    let t0 = scan_start(submaps);
    let end = submaps
        .last()
        .and_then(|m| m.frames.last())
        .map_or(0.0, |f| f.t - t0);
    if submaps.is_empty() || !(0.0..=end).contains(&t) {
        return Err(ManualLoopError::OutsideScan { t, end });
    }
    let abs = t0 + t;
    let mut best = (f64::INFINITY, 0, 0);
    for (s, m) in submaps.iter().enumerate() {
        for (i, fr) in m.frames.iter().enumerate() {
            let d = (fr.t - abs).abs();
            if d < best.0 {
                best = (d, s, i);
            }
        }
    }
    Ok((best.1, best.2))
}

/// Matches every manual loop to its two submaps. Fails on a time outside the scan, on both
/// times in the same submap and on two loops between the same pair of submaps.
pub fn match_manual_loops(
    submaps: &[Submap],
    manual: &[ManualLoop],
) -> Result<Vec<ManualMatch>, ManualLoopError> {
    let mut out: Vec<ManualMatch> = Vec::with_capacity(manual.len());
    for (k, l) in manual.iter().enumerate() {
        let (sa, fa) = locate(submaps, l.a_t)?;
        let (sb, fb) = locate(submaps, l.b_t)?;
        if sa == sb {
            return Err(ManualLoopError::SameSubmap {
                a_t: l.a_t,
                b_t: l.b_t,
                submap: sa,
            });
        }
        let m = if sa < sb {
            ManualMatch {
                a: sa,
                b: sb,
                frame_a: fa,
                frame_b: fb,
            }
        } else {
            ManualMatch {
                a: sb,
                b: sa,
                frame_a: fb,
                frame_b: fa,
            }
        };
        if let Some(j) = out.iter().position(|o| (o.a, o.b) == (m.a, m.b)) {
            return Err(ManualLoopError::Duplicate {
                first: manual[j].clone(),
                second: manual[k].clone(),
                a: m.a,
                b: m.b,
            });
        }
        out.push(m);
    }
    Ok(out)
}

/// Coarse-to-fine correspondence distances of the verification ICP (m).
const STAGES: [f64; 3] = [1.0, 0.5, 0.25];
/// The same for a manual loop: the user marks the place only roughly, so the first stage
/// searches up to 2 m.
const MANUAL_STAGES: [f64; 4] = [2.0, 1.0, 0.5, 0.25];
/// Largest difference between the two registration directions (m, rad).
const SWAP_MAX_TRANS: f64 = 0.05;
const SWAP_MAX_ROT: f64 = 1.0 * std::f64::consts::PI / 180.0;

/// ICP parameters of one verification stage. The prior is weak (1/1000 of the odometry
/// default): the anchors may be off by the whole drift, so the start pose must not hold the
/// solution back; it only keeps the normal equations solvable.
fn stage_params(max_corr_dist: f64) -> IcpParams {
    let d = IcpParams::default();
    IcpParams {
        max_corr_dist,
        prior_rot: d.prior_rot * 1e-3,
        prior_trans: d.prior_trans * 1e-3,
        ..d
    }
}

/// Path length (m) of the odometry from the anchor of submap 0 to the anchor of each
/// submap: the frame-to-frame distances inside every submap plus the step from its last
/// frame to the next anchor, both from the odometry anchors and frame offsets stored in the
/// submaps (independent of the anchors being optimised).
fn path_to_anchor(submaps: &[Submap]) -> Vec<f64> {
    let mut out = Vec::with_capacity(submaps.len());
    let mut s = 0.0;
    for (k, m) in submaps.iter().enumerate() {
        out.push(s);
        s += m
            .frames
            .windows(2)
            .map(|w| (w[1].offset.translation.vector - w[0].offset.translation.vector).norm())
            .sum::<f64>();
        if let (Some(next), Some(last)) = (submaps.get(k + 1), m.frames.last()) {
            let end = (m.anchor * last.offset).translation.vector;
            s += (next.anchor.translation.vector - end).norm();
        }
    }
    out
}

/// Loop candidates: pairs with `b − a ≥ min_gap` whose anchors (`anchors`, one per submap,
/// index = id) are closer than `base_radius + radius_per_m × path length between them`. For
/// each `b` at most `max_candidates`, the nearest; sorted by `(b, a)`.
pub fn find_candidates(
    submaps: &[Submap],
    anchors: &[Isometry3<f64>],
    cfg: &LoopConfig,
) -> Vec<LoopCandidate> {
    assert_eq!(submaps.len(), anchors.len(), "one anchor per submap");
    let path = path_to_anchor(submaps);
    let gap = cfg.min_gap.max(1);
    let mut out = Vec::new();
    for b in gap..submaps.len() {
        let mut near: Vec<(f64, usize)> = (0..=b - gap)
            .filter_map(|a| {
                let d = (anchors[b].translation.vector - anchors[a].translation.vector).norm();
                let radius = cfg.base_radius + cfg.radius_per_m * (path[b] - path[a]);
                (d < radius).then_some((d, a))
            })
            .collect();
        near.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        near.truncate(cfg.max_candidates);
        near.sort_by_key(|&(_, a)| a);
        out.extend(near.into_iter().map(|(_, a)| LoopCandidate { a, b }));
    }
    out
}

fn to_f64(points: &[[f32; 3]]) -> Vec<Vector3<f64>> {
    points
        .iter()
        .map(|p| Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2])))
        .collect()
}

/// Coarse-to-fine ICP of `source` against `target` from `init`. Returns the final pose, its
/// convergence in the finest stage and the metrics evaluated at that pose.
fn register(
    source: &[Vector3<f64>],
    target: &IcpTarget,
    init: &Isometry3<f64>,
    stages: &[f64],
) -> (Isometry3<f64>, bool, IcpResult) {
    let mut pose = *init;
    let mut converged = false;
    for &d in stages {
        let r = align(source, target, &pose, &stage_params(d));
        pose = r.pose;
        converged = r.converged;
    }
    // `align` reports the metrics of the pose before its last update; one more iteration
    // from the final pose measures the pose itself.
    let fine = IcpParams {
        max_iter: 1,
        ..stage_params(stages[stages.len() - 1])
    };
    let at_pose = align(source, target, &pose, &fine);
    (pose, converged, at_pose)
}

/// Verifies candidate `c` by registering the points of `b` (thinned to `icp_voxel`) against
/// those of `a`, starting from `anchors[a]⁻¹ · anchors[b]`, and cross-checks the result by
/// registering `a` against `b`.
pub fn verify(
    submaps: &[Submap],
    anchors: &[Isometry3<f64>],
    c: &LoopCandidate,
    cfg: &LoopConfig,
) -> LoopCheck {
    let init = anchors[c.a].inverse() * anchors[c.b];
    check_pair(submaps, c.a, c.b, &init, &STAGES, cfg.min_inlier_ratio, cfg)
}

/// Verifies a manual loop like [`verify`], but starting from the two marked frames at the
/// same position (each keeps the rotation of the current anchors), with a first ICP stage
/// of 2 m and `manual_min_inlier_ratio`. A failed ICP gives [`LoopVerdict::PositionOnly`]
/// with `relative` = that start ([`position_only_relative`]), so the loop still ties the
/// marked positions together. `manual` and `marked` are set.
pub fn verify_manual(
    submaps: &[Submap],
    anchors: &[Isometry3<f64>],
    m: &ManualMatch,
    cfg: &LoopConfig,
) -> LoopCheck {
    let marked = [
        submaps[m.a].frames[m.frame_a].offset.translation.vector,
        submaps[m.b].frames[m.frame_b].offset.translation.vector,
    ];
    let current = anchors[m.a].inverse() * anchors[m.b];
    let init = position_only_relative(&marked, &current.rotation);
    let mut check = check_pair(
        submaps,
        m.a,
        m.b,
        &init,
        &MANUAL_STAGES,
        cfg.manual_min_inlier_ratio,
        cfg,
    );
    check.manual = true;
    check.marked = Some(marked);
    if let LoopVerdict::Rejected(why) = check.verdict {
        check.verdict = LoopVerdict::PositionOnly(why);
        check.relative = init;
    }
    check
}

/// Pose of the anchor of `b` in the anchor frame of `a` that puts the marked frames
/// (`marked`, positions in their anchor frames) at the same place for the relative rotation
/// `rotation`: translation `marked[0] − rotation · marked[1]`.
pub fn position_only_relative(
    marked: &[Vector3<f64>; 2],
    rotation: &UnitQuaternion<f64>,
) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(marked[0] - rotation * marked[1]),
        *rotation,
    )
}

/// Registers the points of `b` against those of `a` from `init` (pose of the anchor of `b`
/// in the anchor frame of `a`) through `stages` and checks the result.
fn check_pair(
    submaps: &[Submap],
    a: usize,
    b: usize,
    init: &Isometry3<f64>,
    stages: &[f64],
    min_inlier_ratio: f64,
    cfg: &LoopConfig,
) -> LoopCheck {
    let mut check = LoopCheck {
        a,
        b,
        relative: *init,
        inlier_ratio: 0.0,
        rms: 0.0,
        degeneracy: 0.0,
        verdict: LoopVerdict::Rejected("zu wenig Überlappung"),
        manual: false,
        marked: None,
    };
    let pts_a = to_f64(&submaps[a].points);
    let pts_b = to_f64(&submaps[b].points);
    let src_b = voxel_downsample(&pts_b, cfg.icp_voxel);
    let src_a = voxel_downsample(&pts_a, cfg.icp_voxel);
    let (Some(target_a), Some(target_b)) = (IcpTarget::new(pts_a), IcpTarget::new(pts_b)) else {
        return check;
    };

    let (relative, converged, m) = register(&src_b, &target_a, init, stages);
    check.relative = relative;
    check.inlier_ratio = m.inlier_ratio;
    check.rms = m.residual_rms;
    check.degeneracy = m.degeneracy;
    check.verdict = if m.inlier_ratio < min_inlier_ratio {
        LoopVerdict::Rejected("zu wenig Überlappung")
    } else if !converged {
        LoopVerdict::Rejected("ICP nicht konvergiert")
    } else if m.residual_rms > cfg.max_rms {
        LoopVerdict::Rejected("Restfehler zu groß")
    } else if m.degeneracy < cfg.min_degeneracy {
        LoopVerdict::Rejected("schlecht bestimmt")
    } else {
        let (swapped, _, _) = register(&src_a, &target_b, &init.inverse(), stages);
        let diff = relative * swapped;
        if diff.translation.vector.norm() > SWAP_MAX_TRANS || diff.rotation.angle() > SWAP_MAX_ROT {
            LoopVerdict::Rejected("nicht eindeutig")
        } else {
            LoopVerdict::Accepted
        }
    };
    check
}
