//! Per-frame registration quality: how much a frame's pose can be trusted.

use crate::icp::IcpResult;
use driftless_traj::Pose;
use nalgebra::Vector3;

/// [`FrameQuality::degeneracy`] at which the weakest position direction counts as fully
/// fixed. Derived from the synthetic room (`room_scene`, walls, floor and ceiling in every
/// direction): walking frames there reach 0.18–0.20, the first frames against a one-scan
/// map about 0.05; a pillar-free corridor gives about 0.01. Roughly the share of
/// correspondences whose plane normal points along the weakest direction.
pub const DEGENERACY_GOOD: f64 = 0.15;
/// Correspondences per source point at which overlap counts as full.
const INLIER_RATIO_GOOD: f64 = 0.6;
/// Residual RMS at which the score drops to zero (m).
const RESIDUAL_RMS_BAD: f64 = 0.1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameQuality {
    /// Weighted point-to-plane residual RMS of the last ICP iteration (m).
    pub residual_rms: f64,
    /// Correspondences per source point.
    pub inlier_ratio: f64,
    /// Smallest eigenvalue of the translation block of the ICP Hessian per correspondence.
    pub degeneracy: f64,
    /// ICP found enough correspondences; otherwise the IMU prediction was kept.
    pub icp_ok: bool,
}

impl FrameQuality {
    /// The first frame defines the map and is exact by definition.
    pub fn first() -> Self {
        FrameQuality {
            residual_rms: 0.0,
            inlier_ratio: 1.0,
            degeneracy: DEGENERACY_GOOD,
            icp_ok: true,
        }
    }

    pub fn from_icp(r: &IcpResult, icp_ok: bool) -> Self {
        FrameQuality {
            residual_rms: r.residual_rms,
            inlier_ratio: r.inlier_ratio,
            degeneracy: r.degeneracy,
            icp_ok,
        }
    }

    /// Trust in `[0, 1]` (1 = good): product of overlap, constraint and residual factors,
    /// zero if ICP failed.
    pub fn score(&self) -> f32 {
        if !self.icp_ok {
            return 0.0;
        }
        let unit = |x: f64| if x.is_nan() { 0.0 } else { x.clamp(0.0, 1.0) };
        let s = unit(self.inlier_ratio / INLIER_RATIO_GOOD)
            * unit(self.degeneracy / DEGENERACY_GOOD)
            * unit(1.0 - self.residual_rms / RESIDUAL_RMS_BAD);
        s as f32
    }
}

/// One used frame as handed to the sink of [`crate::run_odometry_with`].
#[derive(Debug, Clone)]
pub struct FrameOutput<'a> {
    /// Position in [`crate::OdomResult::poses`].
    pub index: usize,
    pub pose: Pose,
    pub quality: FrameQuality,
    /// Deskewed points in the body frame at the frame end, before downsampling.
    pub body_points: &'a [Vector3<f64>],
    /// Intensity of each entry of `body_points`.
    pub intensity: &'a [f32],
}
