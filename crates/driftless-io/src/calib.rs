//! Camera calibration from `calibration/calib.json`.
//!
//! How the file is read was measured by test projection (`driftless calib-check`, decision
//! record `docs/entscheidungen/0002-kamera-kalibrierung.md`): `K` describes the stored image
//! turned 90° counter-clockwise ([`CALIB_IMAGE_ROTATION`]), and `out_put.<cam>.transform_matrix`
//! maps LiDAR-frame points into that camera's frame ([`CALIB_MATRIX_DIRECTION`]).
//! [`load_calibration`] applies both, so its intrinsics and `lidar_to_cam` refer to the
//! stored image as decoded.

use nalgebra::{Isometry3, Matrix3, Matrix4, Rotation3, Translation3, UnitQuaternion};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

/// Errors produced while loading a calibration file.
#[derive(Debug, thiserror::Error)]
pub enum CalibError {
    #[error("Kalibrierung nicht lesbar: {0}")]
    Io(#[from] std::io::Error),
    #[error("calib.json ist beschädigt: {0}")]
    Json(#[from] serde_json::Error),
    #[error("unbekanntes Kameramodell „{0}“")]
    UnknownModel(String),
    #[error("Kalibrierung ist beschädigt: {0}")]
    Malformed(String),
}

/// Lens model of a camera.
#[derive(Debug, Clone, PartialEq)]
pub enum CameraModel {
    /// Kannala-Brandt fisheye with four distortion coefficients.
    FisheyeKB {
        k: [f64; 4],
    },
    Pinhole,
}

/// Camera time offset (s): an image stamped `t` was taken at `t + CAMERA_TIME_OFFSET_S` on the
/// LiDAR clock. Measured by test projection (ADR 0002).
pub const CAMERA_TIME_OFFSET_S: f64 = 0.03;

/// How a stored image must be turned to get the image `K` of `calib.json` refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageRotation {
    None,
    /// Turned 90° clockwise.
    Cw90,
    /// Turned 90° counter-clockwise.
    Ccw90,
    R180,
}

/// What `out_put.<cam>.transform_matrix` maps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MatrixDirection {
    /// LiDAR-frame points into the camera frame.
    LidarToCam,
    /// Camera-frame points into the LiDAR frame.
    CamToLidar,
}

/// Measured reading of `K` for the JMK6 (ADR 0002).
pub const CALIB_IMAGE_ROTATION: ImageRotation = ImageRotation::Ccw90;
/// Measured meaning of `transform_matrix` for the JMK6 (ADR 0002).
pub const CALIB_MATRIX_DIRECTION: MatrixDirection = MatrixDirection::LidarToCam;

/// Intrinsics and extrinsics of one camera, for the stored (decoded) image: pixel `(0, 0)`
/// is the centre of the first stored pixel, x right, y down.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraCalib {
    pub k: Matrix3<f64>,
    pub model: CameraModel,
    pub width: u32,
    pub height: u32,
    /// Maps LiDAR-frame points into the camera frame (`p_cam = lidar_to_cam · p_lidar`), the
    /// camera frame being x right and y down along the stored image, z forward. `None` if
    /// the file has no `transform_matrix` for this camera. Reading per ADR 0002.
    pub lidar_to_cam: Option<Isometry3<f64>>,
}

/// Calibration of a device, keyed by camera name (e.g. `left`, `right`).
#[derive(Debug, Clone, PartialEq)]
pub struct Calibration {
    pub device_sn: Option<String>,
    pub device_model: Option<String>,
    pub cameras: BTreeMap<String, CameraCalib>,
}

#[derive(Deserialize)]
struct RawFile {
    #[serde(default)]
    camera_info: BTreeMap<String, RawCamera>,
    device: Option<RawDevice>,
    #[serde(default)]
    out_put: BTreeMap<String, RawOutput>,
}

#[derive(Deserialize)]
struct RawDevice {
    #[serde(rename = "SN")]
    sn: Option<String>,
    model: Option<String>,
}

#[derive(Deserialize)]
struct RawCamera {
    #[serde(rename = "K")]
    k: Vec<f64>,
    camera_model: String,
    #[serde(default)]
    coeff: Vec<f64>,
    image_width: u32,
    image_height: u32,
}

#[derive(Deserialize)]
struct RawOutput {
    transform_matrix: Option<Vec<Vec<f64>>>,
}

/// `transform_matrix` as an isometry: the rotation part must be orthonormal (tolerance 1e-3)
/// with determinant +1 and the last row `0 0 0 1`.
fn matrix_isometry(m: &Matrix4<f64>) -> Option<Isometry3<f64>> {
    let r: Matrix3<f64> = m.fixed_view::<3, 3>(0, 0).into();
    let last_row = [m[(3, 0)], m[(3, 1)], m[(3, 2)], m[(3, 3)] - 1.0];
    let rigid = m.iter().all(|v| v.is_finite())
        && last_row.iter().all(|v| v.abs() < 1e-9)
        && (r.transpose() * r - Matrix3::identity()).norm() < 1e-3
        && r.determinant() > 0.0;
    rigid.then(|| {
        Isometry3::from_parts(
            Translation3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&r)),
        )
    })
}

/// Turns intrinsics `k` of the image turned by `rotation` and the transform into that
/// camera's frame into those of the stored image (`width` × `height`). The lens model is
/// radially symmetric, so only `K` and the camera axes change. `None` for a rotated reading
/// with skew in `K`.
fn to_stored_image(
    k: &Matrix3<f64>,
    lidar_to_cam: Option<Isometry3<f64>>,
    rotation: ImageRotation,
    width: u32,
    height: u32,
) -> Option<(Matrix3<f64>, Option<Isometry3<f64>>)> {
    let (fx, fy, cx, cy) = (k[(0, 0)], k[(1, 1)], k[(0, 2)], k[(1, 2)]);
    let (w1, h1) = (f64::from(width) - 1.0, f64::from(height) - 1.0);
    // Stored camera axes in the turned camera's frame (about the optical axis), and the
    // stored intrinsics: e.g. Ccw90: stored pixel = (w − 1 − y', x') of turned pixel (x', y').
    let (angle, fx_s, fy_s, cx_s, cy_s) = match rotation {
        ImageRotation::None => return Some((*k, lidar_to_cam)),
        ImageRotation::Ccw90 => (std::f64::consts::FRAC_PI_2, fy, fx, w1 - cy, cx),
        ImageRotation::Cw90 => (-std::f64::consts::FRAC_PI_2, fy, fx, cy, h1 - cx),
        ImageRotation::R180 => (std::f64::consts::PI, fx, fy, w1 - cx, h1 - cy),
    };
    if k[(0, 1)] != 0.0 {
        return None;
    }
    let turn = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), angle);
    let k_s = Matrix3::new(fx_s, 0.0, cx_s, 0.0, fy_s, cy_s, 0.0, 0.0, 1.0);
    Some((
        k_s,
        lidar_to_cam.map(|t| Isometry3::from_parts(Default::default(), turn) * t),
    ))
}

fn parse_camera(
    name: &str,
    cam: &RawCamera,
    out: Option<&RawOutput>,
    reading: CalibReading,
) -> Result<CameraCalib, CalibError> {
    let bad = |what: &str| CalibError::Malformed(format!("camera `{name}`: {what}"));
    if cam.k.len() != 9 {
        return Err(bad("K must have 9 numbers"));
    }
    let model = match cam.camera_model.as_str() {
        "fisheyeKB" => {
            let k: [f64; 4] = cam
                .coeff
                .as_slice()
                .try_into()
                .map_err(|_| bad("coeff must have 4 numbers for fisheyeKB"))?;
            CameraModel::FisheyeKB { k }
        }
        "pinhole" => CameraModel::Pinhole,
        other => return Err(CalibError::UnknownModel(other.to_string())),
    };
    let matrix = match out.and_then(|o| o.transform_matrix.as_ref()) {
        None => None,
        Some(rows) => {
            if rows.len() != 4 || rows.iter().any(|r| r.len() != 4) {
                return Err(bad("transform_matrix must be 4x4"));
            }
            let m = Matrix4::from_fn(|i, j| rows[i][j]);
            Some(matrix_isometry(&m).ok_or_else(|| bad("transform_matrix is not rigid"))?)
        }
    };
    let lidar_to_cam = matrix.map(|m| match reading.direction {
        MatrixDirection::LidarToCam => m,
        MatrixDirection::CamToLidar => m.inverse(),
    });
    let (k, lidar_to_cam) = to_stored_image(
        &Matrix3::from_row_slice(&cam.k),
        lidar_to_cam,
        reading.rotation,
        cam.image_width,
        cam.image_height,
    )
    .ok_or_else(|| bad("K with skew cannot be turned"))?;
    Ok(CameraCalib {
        k,
        model,
        width: cam.image_width,
        height: cam.image_height,
        lidar_to_cam,
    })
}

/// How `calib.json` is interpreted (see [`load_calibration_as`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibReading {
    pub rotation: ImageRotation,
    pub direction: MatrixDirection,
}

/// The measured reading (ADR 0002).
pub const CALIB_READING: CalibReading = CalibReading {
    rotation: CALIB_IMAGE_ROTATION,
    direction: CALIB_MATRIX_DIRECTION,
};

/// Loads `calib.json` with the measured reading ([`CALIB_READING`]). Unknown fields are
/// ignored.
pub fn load_calibration(path: &Path) -> Result<Calibration, CalibError> {
    load_calibration_as(path, CALIB_READING)
}

/// Loads `calib.json` with an explicit reading of `K` and `transform_matrix` (for testing the
/// alternatives, `driftless calib-check`). The image size stays that of the file, which is
/// the stored image's.
pub fn load_calibration_as(path: &Path, reading: CalibReading) -> Result<Calibration, CalibError> {
    let raw: RawFile = serde_json::from_slice(&std::fs::read(path)?)?;
    let mut cameras = BTreeMap::new();
    for (name, cam) in &raw.camera_info {
        cameras.insert(
            name.clone(),
            parse_camera(name, cam, raw.out_put.get(name), reading)?,
        );
    }
    Ok(Calibration {
        device_sn: raw.device.as_ref().and_then(|d| d.sn.clone()),
        device_model: raw.device.and_then(|d| d.model),
        cameras,
    })
}
