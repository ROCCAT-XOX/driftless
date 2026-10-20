//! SE(3) helpers: the exponential map used for the right perturbation `T · Exp(δ)` and the
//! edge residual. Tangent vectors are ordered `[rotation; translation]` as in `icp.rs`.

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3, Vector6};

/// Below this rotation angle the Taylor series replaces the closed-form `V` coefficients.
const SMALL_ANGLE: f64 = 1e-6;

/// Exponential map of `delta = [ω; v]` onto SE(3).
pub fn exp(delta: &Vector6<f64>) -> Isometry3<f64> {
    let omega = Vector3::new(delta[0], delta[1], delta[2]);
    let v = Vector3::new(delta[3], delta[4], delta[5]);
    let theta2 = omega.norm_squared();
    let theta = theta2.sqrt();
    let w = omega.cross_matrix();
    // V = I + (1 - cos θ)/θ² W + (θ - sin θ)/θ³ W²
    let (a, b) = if theta < SMALL_ANGLE {
        (0.5 - theta2 / 24.0, 1.0 / 6.0 - theta2 / 120.0)
    } else {
        (
            (1.0 - theta.cos()) / theta2,
            (theta - theta.sin()) / (theta2 * theta),
        )
    };
    let v_mat = Matrix3::identity() + w * a + w * w * b;
    Isometry3::from_parts(
        Translation3::from(v_mat * v),
        UnitQuaternion::from_scaled_axis(omega),
    )
}

/// Residual of an edge: `E = Z⁻¹ · T_from⁻¹ · T_to`, `e = [rot(E).scaled_axis; trans(E)]`.
/// Zero when the two poses agree with the measurement `Z`.
pub fn residual(
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
    measurement: &Isometry3<f64>,
) -> Vector6<f64> {
    let e = measurement.inverse() * from.inverse() * to;
    let r = e.rotation.scaled_axis();
    let t = e.translation.vector;
    Vector6::new(r.x, r.y, r.z, t.x, t.y, t.z)
}
