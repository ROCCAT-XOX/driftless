//! IMU preparation, static initialisation and strapdown propagation.
//!
//! Conventions: the world frame is gravity aligned with +z up; `rot` maps body to world.
//! Accelerometers measure specific force `f = R^T (a - g)`, so at rest `|f| ≈ 9.81` pointing up.

use driftless_io::msg::ImuSample;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

/// Body state at time `t` (world from body).
#[derive(Debug, Clone, PartialEq)]
pub struct ImuState {
    pub t: f64,
    pub rot: UnitQuaternion<f64>,
    pub pos: Vector3<f64>,
    pub vel: Vector3<f64>,
}

impl ImuState {
    /// Standing still at the origin with orientation `rot`.
    pub fn at_rest(t: f64, rot: UnitQuaternion<f64>) -> Self {
        ImuState {
            t,
            rot,
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
        }
    }

    pub fn pose(&self) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::from(self.pos), self.rot)
    }
}

/// Gravity (world frame, m/s²) and sensor biases (body frame).
#[derive(Debug, Clone, PartialEq)]
pub struct ImuModel {
    pub gravity: Vector3<f64>,
    pub gyro_bias: Vector3<f64>,
    pub accel_bias: Vector3<f64>,
}

/// What [`sanitise_imu`] changed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImuCleanup {
    /// Input was not in time order and had to be sorted.
    pub reordered: bool,
    /// Samples dropped because another sample had the same stamp.
    pub duplicates: usize,
    /// Samples dropped because a stamp or value was NaN/Inf.
    pub non_finite: usize,
    /// Gaps longer than [`GAP_WARN_S`]; propagation interpolates across them.
    pub gaps: usize,
}

/// Gaps longer than this are counted in [`ImuCleanup::gaps`] (nominal spacing is 5 ms).
pub const GAP_WARN_S: f64 = 0.02;

/// Sorts by stamp, drops non-finite samples and duplicate stamps (first one wins).
pub fn sanitise_imu(imu: &[ImuSample]) -> (Vec<ImuSample>, ImuCleanup) {
    let mut c = ImuCleanup::default();
    let finite =
        |s: &ImuSample| s.stamp.is_finite() && s.gyro.iter().chain(&s.accel).all(|v| v.is_finite());
    let mut out: Vec<ImuSample> = imu.iter().filter(|s| finite(s)).cloned().collect();
    c.non_finite = imu.len() - out.len();
    if out.windows(2).any(|w| w[1].stamp < w[0].stamp) {
        c.reordered = true;
        out.sort_by(|a, b| a.stamp.total_cmp(&b.stamp)); // stable: first duplicate stays first
    }
    let before = out.len();
    out.dedup_by(|b, a| b.stamp == a.stamp);
    c.duplicates = before - out.len();
    c.gaps = out
        .windows(2)
        .filter(|w| w[1].stamp - w[0].stamp > GAP_WARN_S)
        .count();
    (out, c)
}

/// Result of [`estimate_initialisation`].
#[derive(Debug, Clone, PartialEq)]
pub struct ImuInit {
    /// Gravity-aligned start orientation (yaw = 0).
    pub rot: UnitQuaternion<f64>,
    pub model: ImuModel,
    /// The window looked static (gyro and accelerometer quiet).
    pub is_static: bool,
    pub samples: usize,
}

/// Mean gyro below this (rad/s) and accel-norm spread below [`STATIC_ACCEL_STD`] count as static.
pub const STATIC_GYRO: f64 = 0.05;
pub const STATIC_ACCEL_STD: f64 = 0.3;

/// Estimates start orientation, gravity and gyro bias from the first `window` seconds of
/// (sanitised) IMU data, assuming the device is held still there.
///
/// If the window is not static, the gyro bias is set to zero (the mean rate would be real
/// motion) and gravity comes from the mean accelerometer reading, which then includes motion:
/// the start tilt can be off by a few degrees and the odometry has to absorb it.
/// Returns `None` without IMU data.
pub fn estimate_initialisation(imu: &[ImuSample], window: f64) -> Option<ImuInit> {
    let t0 = imu.first()?.stamp;
    let win: Vec<&ImuSample> = imu.iter().take_while(|s| s.stamp <= t0 + window).collect();
    let n = win.len() as f64;
    let mean = |f: &dyn Fn(&ImuSample) -> [f64; 3]| {
        win.iter()
            .fold(Vector3::zeros(), |acc, s| acc + Vector3::from(f(s)))
            / n
    };
    let gyro = mean(&|s| s.gyro);
    let accel = mean(&|s| s.accel);
    let norms: Vec<f64> = win.iter().map(|s| Vector3::from(s.accel).norm()).collect();
    let mean_norm = norms.iter().sum::<f64>() / n;
    let std = (norms.iter().map(|v| (v - mean_norm).powi(2)).sum::<f64>() / n).sqrt();
    let is_static = gyro.norm() < STATIC_GYRO && std < STATIC_ACCEL_STD;
    let g = accel.norm();
    if g.is_nan() || g <= 1.0 {
        return None; // free fall or garbage: no usable gravity direction
    }
    let rot = UnitQuaternion::rotation_between(&accel, &Vector3::z()).unwrap_or_else(|| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    });
    Some(ImuInit {
        rot,
        model: ImuModel {
            gravity: Vector3::new(0.0, 0.0, -g),
            gyro_bias: if is_static { gyro } else { Vector3::zeros() },
            accel_bias: Vector3::zeros(),
        },
        is_static,
        samples: win.len(),
    })
}

/// Gyro and accelerometer reading at `t`, linearly interpolated (held at the ends).
/// `None` without IMU data.
fn measurement(imu: &[ImuSample], t: f64) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let i = imu.partition_point(|s| s.stamp < t);
    let pick = |s: &ImuSample| (Vector3::from(s.gyro), Vector3::from(s.accel));
    if i == 0 {
        return imu.first().map(pick);
    }
    let Some(b) = imu.get(i) else {
        return imu.last().map(pick);
    };
    let a = &imu[i - 1];
    let span = b.stamp - a.stamp;
    let w = if span > 0.0 {
        (t - a.stamp) / span
    } else {
        0.0
    };
    let (ga, aa) = pick(a);
    let (gb, ab) = pick(b);
    Some((ga.lerp(&gb, w), aa.lerp(&ab, w)))
}

/// Propagates `start` to `t_end` with sanitised IMU samples (sorted by stamp).
///
/// Returns `start`, one state per IMU sample inside `(start.t, t_end)` and the state at
/// `t_end`. Measurements are interpolated linearly, so gaps are bridged. Without IMU data the
/// motion continues at constant velocity and orientation.
pub fn propagate(
    start: &ImuState,
    imu: &[ImuSample],
    model: &ImuModel,
    t_end: f64,
) -> Vec<ImuState> {
    let first = imu.partition_point(|s| s.stamp <= start.t);
    let mut times: Vec<f64> = imu[first..]
        .iter()
        .map(|s| s.stamp)
        .take_while(|&t| t < t_end)
        .collect();
    if t_end > start.t {
        times.push(t_end);
    }
    let mut states = Vec::with_capacity(times.len() + 1);
    states.push(start.clone());
    let mut cur = start.clone();
    let mut m_prev = measurement(imu, cur.t);
    for t in times {
        let dt = t - cur.t;
        let m_next = measurement(imu, t);
        cur = match (m_prev, m_next) {
            (Some((g0, a0)), Some((g1, a1))) => {
                let omega = (g0 + g1) * 0.5 - model.gyro_bias;
                let f = (a0 + a1) * 0.5 - model.accel_bias;
                let half = cur.rot * UnitQuaternion::from_scaled_axis(omega * (0.5 * dt));
                let acc = half * f + model.gravity;
                ImuState {
                    t,
                    rot: cur.rot * UnitQuaternion::from_scaled_axis(omega * dt),
                    pos: cur.pos + cur.vel * dt + acc * (0.5 * dt * dt),
                    vel: cur.vel + acc * dt,
                }
            }
            _ => ImuState {
                t,
                rot: cur.rot,
                pos: cur.pos + cur.vel * dt,
                vel: cur.vel,
            },
        };
        m_prev = m_next;
        states.push(cur.clone());
    }
    states
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(t: f64, gz: f64) -> ImuSample {
        ImuSample {
            stamp: t,
            gyro: [0.0, 0.0, gz],
            accel: [0.0, 0.0, 9.81],
        }
    }

    #[test]
    fn sanitise_sorts_drops_duplicates_and_nan_and_counts_gaps() {
        let mut nan = s(0.02, 0.0);
        nan.accel[1] = f64::NAN;
        let input = vec![
            s(0.0, 1.0),
            s(0.01, 2.0),
            s(0.005, 3.0),
            s(0.01, 4.0),
            nan,
            s(0.1, 5.0),
        ];
        let (out, c) = sanitise_imu(&input);
        let t: Vec<f64> = out.iter().map(|x| x.stamp).collect();
        assert_eq!(t, vec![0.0, 0.005, 0.01, 0.1]);
        assert_eq!(out[2].gyro[2], 2.0, "first of the duplicates is kept");
        assert!(c.reordered);
        assert_eq!((c.duplicates, c.non_finite, c.gaps), (1, 1, 1));
    }

    #[test]
    fn initialisation_levels_tilted_device_and_takes_gyro_bias() {
        let tilt = UnitQuaternion::from_euler_angles(0.5, -0.2, 0.0);
        let f = tilt.inverse() * Vector3::new(0.0, 0.0, 9.8);
        let imu: Vec<ImuSample> = (0..400)
            .map(|i| ImuSample {
                stamp: i as f64 * 0.005,
                gyro: [0.01, -0.02, 0.003],
                accel: f.into(),
            })
            .collect();
        let init = estimate_initialisation(&imu, 1.0).unwrap();
        assert!(init.is_static);
        assert_eq!(init.samples, 201);
        assert!((init.rot * f - Vector3::new(0.0, 0.0, 9.8)).norm() < 1e-9);
        assert!((init.model.gyro_bias - Vector3::new(0.01, -0.02, 0.003)).norm() < 1e-12);
        assert!((init.model.gravity.z + 9.8).abs() < 1e-9);
    }

    #[test]
    fn moving_start_is_flagged_and_keeps_zero_gyro_bias() {
        let imu: Vec<ImuSample> = (0..400).map(|i| s(i as f64 * 0.005, 0.5)).collect();
        let init = estimate_initialisation(&imu, 1.0).unwrap();
        assert!(!init.is_static);
        assert_eq!(init.model.gyro_bias, Vector3::zeros());
    }

    #[test]
    fn propagation_without_imu_keeps_velocity() {
        let mut start = ImuState::at_rest(1.0, UnitQuaternion::identity());
        start.vel = Vector3::new(1.0, 0.0, 0.0);
        let model = ImuModel {
            gravity: Vector3::new(0.0, 0.0, -9.81),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
        };
        let out = propagate(&start, &[], &model, 1.5);
        assert_eq!(out.len(), 2);
        assert!((out[1].pos.x - 0.5).abs() < 1e-12);
    }
}
