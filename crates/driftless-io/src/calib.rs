//! Camera calibration from `calibration/calib.json`.

use nalgebra::{Matrix3, Matrix4};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

/// Errors produced while loading a calibration file.
#[derive(Debug, thiserror::Error)]
pub enum CalibError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("invalid calib.json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("unknown camera model `{0}`")]
    UnknownModel(String),
    #[error("malformed calibration: {0}")]
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

/// Intrinsics and the raw LiDAR/camera transform of one camera.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraCalib {
    pub k: Matrix3<f64>,
    pub model: CameraModel,
    pub width: u32,
    pub height: u32,
    /// `out_put.<cam>.transform_matrix` as stored (row i = matrix row i).
    /// The direction is not yet verified, hence the `_raw` suffix.
    pub lidar_to_cam_raw: Option<Matrix4<f64>>,
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

fn parse_camera(
    name: &str,
    cam: &RawCamera,
    out: Option<&RawOutput>,
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
    let lidar_to_cam_raw = match out.and_then(|o| o.transform_matrix.as_ref()) {
        None => None,
        Some(rows) => {
            if rows.len() != 4 || rows.iter().any(|r| r.len() != 4) {
                return Err(bad("transform_matrix must be 4x4"));
            }
            Some(Matrix4::from_fn(|i, j| rows[i][j]))
        }
    };
    Ok(CameraCalib {
        k: Matrix3::from_row_slice(&cam.k),
        model,
        width: cam.image_width,
        height: cam.image_height,
        lidar_to_cam_raw,
    })
}

/// Loads `calib.json`. Unknown fields are ignored.
pub fn load_calibration(path: &Path) -> Result<Calibration, CalibError> {
    let raw: RawFile = serde_json::from_slice(&std::fs::read(path)?)?;
    let mut cameras = BTreeMap::new();
    for (name, cam) in &raw.camera_info {
        cameras.insert(
            name.clone(),
            parse_camera(name, cam, raw.out_put.get(name))?,
        );
    }
    Ok(Calibration {
        device_sn: raw.device.as_ref().and_then(|d| d.sn.clone()),
        device_model: raw.device.and_then(|d| d.model),
        cameras,
    })
}
