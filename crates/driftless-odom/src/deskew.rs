//! Per-point capture times and motion compensation ("deskew") of one LiDAR sweep.

use crate::imu_integrate::ImuState;
use driftless_io::msg::LidarFrame;
use nalgebra::{Isometry3, Point3, Translation3, Vector3};

/// Per-point times above this (seconds) are taken as absolute stamps, not offsets.
pub const ABSOLUTE_TIME_THRESHOLD: f64 = 1.0;

/// A point in the body (IMU) frame at its absolute capture time `t`.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedPoint {
    pub p: Vector3<f64>,
    pub t: f64,
}

/// One sweep in the body frame with normalised point times.
#[derive(Debug, Clone, PartialEq)]
pub struct FramePoints {
    pub points: Vec<TimedPoint>,
    /// Earliest and latest point time (both `stamp` for an empty frame).
    pub t_start: f64,
    pub t_end: f64,
    /// Points dropped because their time was NaN/Inf.
    pub skipped: usize,
    /// Points whose time lay before the frame stamp; moved to the stamp.
    pub clamped: usize,
}

/// Value to subtract from the per-point time field to get an offset from `frame.stamp`:
/// `frame.stamp` if the field holds absolute stamps (largest finite value above
/// [`ABSOLUTE_TIME_THRESHOLD`]), else 0.
pub fn point_time_base(frame: &LidarFrame) -> f64 {
    let max_t = frame
        .points
        .iter()
        .map(|p| p.t_rel)
        .filter(|t| t.is_finite())
        .fold(f64::NEG_INFINITY, f64::max);
    if max_t > ABSOLUTE_TIME_THRESHOLD {
        frame.stamp
    } else {
        0.0
    }
}

/// Converts a frame to body-frame points with absolute times.
///
/// Point times are offsets from `frame.stamp` (JMK6/Vanjee). If the largest one exceeds
/// [`ABSOLUTE_TIME_THRESHOLD`], the field holds absolute stamps (e.g. Hesai) and the frame
/// stamp is subtracted first. Offsets below zero are clamped to zero.
pub fn frame_points(frame: &LidarFrame, lidar_to_imu: &Isometry3<f64>) -> FramePoints {
    let base = point_time_base(frame);
    let (mut skipped, mut clamped) = (0, 0);
    let mut points = Vec::with_capacity(frame.points.len());
    let (mut t_start, mut t_end) = (f64::INFINITY, f64::NEG_INFINITY);
    for lp in &frame.points {
        if !lp.t_rel.is_finite() {
            skipped += 1;
            continue;
        }
        let mut off = lp.t_rel - base;
        if off < 0.0 {
            off = 0.0;
            clamped += 1;
        }
        let t = frame.stamp + off;
        t_start = t_start.min(t);
        t_end = t_end.max(t);
        let raw = Point3::new(
            f64::from(lp.xyz[0]),
            f64::from(lp.xyz[1]),
            f64::from(lp.xyz[2]),
        );
        points.push(TimedPoint {
            p: (lidar_to_imu * raw).coords,
            t,
        });
    }
    if points.is_empty() {
        (t_start, t_end) = (frame.stamp, frame.stamp);
    }
    FramePoints {
        points,
        t_start,
        t_end,
        skipped,
        clamped,
    }
}

/// Body pose at `t`, interpolated in `motion` (non-empty, sorted by time; held at the ends).
pub fn pose_at(motion: &[ImuState], t: f64) -> Isometry3<f64> {
    let i = motion.partition_point(|s| s.t < t);
    if i == 0 {
        return motion[0].pose();
    }
    let Some(b) = motion.get(i) else {
        return motion[motion.len() - 1].pose();
    };
    let a = &motion[i - 1];
    let span = b.t - a.t;
    let w = if span > 0.0 { (t - a.t) / span } else { 0.0 };
    Isometry3::from_parts(
        Translation3::from(a.pos.lerp(&b.pos, w)),
        a.rot.try_slerp(&b.rot, w, 1e-9).unwrap_or(a.rot), // None only for a 180° step
    )
}

/// Moves every point into the body frame at `t_ref` using the body motion `motion`
/// (e.g. from [`crate::imu_integrate::propagate`]). Without motion the points stay as they are.
pub fn deskew(points: &[TimedPoint], motion: &[ImuState], t_ref: f64) -> Vec<Vector3<f64>> {
    if motion.is_empty() {
        return points.iter().map(|tp| tp.p).collect();
    }
    let ref_inv = pose_at(motion, t_ref).inverse();
    points
        .iter()
        .map(|tp| (ref_inv * pose_at(motion, tp.t) * Point3::from(tp.p)).coords)
        .collect()
}
