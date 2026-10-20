//! `slam.json`: what the SLAM run did, for the user and the viewer. Times are seconds since
//! the start of the scan ([`scan_start`]).

use crate::loops::{LoopVerdict, POSITION_ONLY};
use crate::pipeline::SlamResult;
use crate::submap::Submap;
use driftless_traj::{path_length, start_end_gap};
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SlamReport {
    pub submaps: Vec<SubmapReport>,
    /// Every verified loop candidate, sorted by `(b, a)`.
    pub loops: Vec<LoopReport>,
    /// `None` if nothing was optimised.
    pub graph: Option<GraphReport>,
    /// Distance between the first and the last pose (m).
    pub start_end_gap: BeforeAfter,
    pub path_length: BeforeAfter,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubmapReport {
    pub id: usize,
    /// Time of the first and the last frame.
    pub t_start: f64,
    pub t_end: f64,
    /// [`Submap::quality`] (1 = good).
    pub quality: f32,
    /// Optimised anchor position.
    pub anchor: [f64; 3],
    /// Odometry anchor position.
    pub odom_anchor: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoopReport {
    pub a: usize,
    pub b: usize,
    pub manual: bool,
    /// The loop takes part in the pose graph (also a manual loop that only ties the
    /// positions together).
    pub accepted: bool,
    /// Why the loop was rejected, or the warning of a manual loop that only ties the
    /// positions together (German); `None` if fully accepted.
    pub reason: Option<String>,
    pub inlier_ratio: f64,
    pub rms: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraphReport {
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: u32,
}

/// A metric of the odometry (`before`) and of the loop-closed trajectory (`after`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeforeAfter {
    pub before: f64,
    pub after: f64,
}

/// Start of the scan: time of the first pose (end of the first LiDAR frame that produced a
/// pose); 0 without submaps. Derived from the submaps alone, so it stays the same when they
/// are reloaded.
pub fn scan_start(submaps: &[Submap]) -> f64 {
    submaps
        .first()
        .and_then(|m| m.frames.first())
        .map_or(0.0, |f| f.t)
}

fn position(a: &Isometry3<f64>) -> [f64; 3] {
    let v = a.translation.vector;
    [v.x, v.y, v.z]
}

impl SlamReport {
    pub fn new(r: &SlamResult) -> Self {
        let t0 = scan_start(&r.submaps);
        let submaps = r
            .submaps
            .iter()
            .zip(r.anchors.iter().zip(&r.odom_anchors))
            .map(|(m, (anchor, odom))| SubmapReport {
                id: m.id,
                t_start: m.frames.first().map_or(t0, |f| f.t) - t0,
                t_end: m.frames.last().map_or(t0, |f| f.t) - t0,
                quality: m.quality(),
                anchor: position(anchor),
                odom_anchor: position(odom),
            })
            .collect();
        let loops = r
            .loops
            .iter()
            .map(|c| LoopReport {
                a: c.a,
                b: c.b,
                manual: c.manual,
                accepted: !matches!(c.verdict, LoopVerdict::Rejected(_)),
                reason: match c.verdict {
                    LoopVerdict::Accepted => None,
                    LoopVerdict::Rejected(why) => Some(why.to_string()),
                    LoopVerdict::PositionOnly(why) => Some(format!("{POSITION_ONLY} ({why})")),
                },
                inlier_ratio: c.inlier_ratio,
                rms: c.rms,
            })
            .collect();
        let (before, after) = (r.odom_poses(), r.poses());
        SlamReport {
            submaps,
            loops,
            graph: r.graph.as_ref().map(|g| GraphReport {
                initial_cost: g.initial_cost,
                final_cost: g.final_cost,
                iterations: g.iterations,
            }),
            start_end_gap: BeforeAfter {
                before: start_end_gap(&before),
                after: start_end_gap(&after),
            },
            path_length: BeforeAfter {
                before: path_length(&before),
                after: path_length(&after),
            },
        }
    }
}
