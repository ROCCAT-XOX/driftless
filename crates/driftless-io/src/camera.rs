//! Camera model: projection and unprojection for Kannala-Brandt fisheye and pinhole lenses.

use crate::calib::{CalibError, CameraCalib, CameraModel};
use nalgebra::Vector3;
use std::f64::consts::PI;

/// Upper limit for `max_theta` (100 degrees).
const MAX_THETA_LIMIT: f64 = 100.0 * PI / 180.0;
const NEWTON_ITERATIONS: usize = 20;
const NEWTON_TOLERANCE: f64 = 1e-14;

/// A calibrated camera: intrinsics plus lens model.
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    pub name: String,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub model: CameraModel,
    /// Largest angle from the optical axis (rad) that still projects uniquely.
    pub max_theta: f64,
}

/// `θd(θ) = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)`.
fn distorted(k: &[f64; 4], theta: f64) -> f64 {
    let t2 = theta * theta;
    theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))
}

/// `dθd/dθ`.
fn distorted_derivative(k: &[f64; 4], theta: f64) -> f64 {
    let t2 = theta * theta;
    1.0 + t2 * (3.0 * k[0] + t2 * (5.0 * k[1] + t2 * (7.0 * k[2] + t2 * 9.0 * k[3])))
}

/// Largest θ (up to 100 degrees) up to which `θd` still increases strictly.
fn kb_max_theta(k: &[f64; 4]) -> f64 {
    const STEPS: usize = 1000;
    let step = MAX_THETA_LIMIT / STEPS as f64;
    for i in 1..=STEPS {
        let hi = step * i as f64;
        if distorted_derivative(k, hi) <= 0.0 {
            // Bisect the sign change of the derivative between `hi - step` and `hi`.
            let (mut lo, mut hi) = (hi - step, hi);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if distorted_derivative(k, mid) > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            return lo;
        }
    }
    MAX_THETA_LIMIT
}

impl Camera {
    pub fn from_calib(name: &str, calib: &CameraCalib) -> Result<Camera, CalibError> {
        let (fx, fy) = (calib.k[(0, 0)], calib.k[(1, 1)]);
        let (cx, cy) = (calib.k[(0, 2)], calib.k[(1, 2)]);
        if !(fx > 0.0 && fy > 0.0 && cx.is_finite() && cy.is_finite()) {
            return Err(CalibError::Malformed(format!(
                "camera `{name}`: focal lengths must be positive and finite"
            )));
        }
        let max_theta = match &calib.model {
            CameraModel::FisheyeKB { k } => kb_max_theta(k),
            CameraModel::Pinhole => PI / 2.0,
        };
        Ok(Camera {
            name: name.to_string(),
            width: calib.width,
            height: calib.height,
            fx,
            fy,
            cx,
            cy,
            model: calib.model.clone(),
            max_theta,
        })
    }

    /// Projects a point in the camera frame to pixel coordinates.
    /// `None` if the point is beyond `max_theta` or falls outside the image.
    pub fn project(&self, p_cam: &Vector3<f64>) -> Option<[f64; 2]> {
        let r = p_cam.x.hypot(p_cam.y);
        let (u, v) = match &self.model {
            CameraModel::FisheyeKB { k } => {
                let theta = r.atan2(p_cam.z);
                if theta > self.max_theta {
                    return None;
                }
                if r == 0.0 {
                    (self.cx, self.cy)
                } else {
                    let scale = distorted(k, theta) / r;
                    (
                        self.fx * scale * p_cam.x + self.cx,
                        self.fy * scale * p_cam.y + self.cy,
                    )
                }
            }
            CameraModel::Pinhole => {
                if p_cam.z <= 0.0 {
                    return None;
                }
                (
                    self.fx * p_cam.x / p_cam.z + self.cx,
                    self.fy * p_cam.y / p_cam.z + self.cy,
                )
            }
        };
        let inside = u >= 0.0 && v >= 0.0 && u < self.width as f64 && v < self.height as f64;
        inside.then_some([u, v])
    }

    /// Unit viewing ray for a pixel. `None` if the pixel lies outside the lens model's range.
    pub fn unproject(&self, uv: [f64; 2]) -> Option<Vector3<f64>> {
        let du = (uv[0] - self.cx) / self.fx;
        let dv = (uv[1] - self.cy) / self.fy;
        let rd = du.hypot(dv);
        match &self.model {
            CameraModel::FisheyeKB { k } => {
                if !rd.is_finite() {
                    return None;
                }
                if rd == 0.0 {
                    return Some(Vector3::new(0.0, 0.0, 1.0));
                }
                if rd > distorted(k, self.max_theta) {
                    return None;
                }
                // Newton on θd(θ) = rd.
                let mut theta = rd.min(self.max_theta);
                let mut converged = false;
                for _ in 0..NEWTON_ITERATIONS {
                    let step = (distorted(k, theta) - rd) / distorted_derivative(k, theta);
                    theta -= step;
                    if step.abs() < NEWTON_TOLERANCE {
                        converged = true;
                        break;
                    }
                }
                if !converged || !(0.0..=self.max_theta).contains(&theta) {
                    return None;
                }
                let s = theta.sin() / rd;
                Some(Vector3::new(s * du, s * dv, theta.cos()))
            }
            CameraModel::Pinhole => Some(Vector3::new(du, dv, 1.0).normalize()),
        }
    }
}
