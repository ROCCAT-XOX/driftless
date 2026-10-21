//! Dense point cloud from the loop-closed trajectory, and its export.
//!
//! [`build_dense`] runs the (deterministic) odometry again over the same frames, so every
//! deskewed sweep comes back in the body frame, and places it with the SLAM-corrected pose of
//! that frame. One point is kept per voxel: the one seen from closest, since near points are
//! the most accurate. [`clean`] removes ghosts (things that moved) and isolated outliers;
//! [`colorize`] colours it from the camera images (visibility by depth buffer, image quality,
//! exposure and white balance between the images, weighted blend of the best views);
//! [`write_las`] and [`write_ply`] write the [`Cloud`]; [`save_stage`] and [`load_stage`] keep
//! it between the steps at full precision.

mod clean;
mod cloud;
mod color;
mod dense;
mod export;
mod exposure;
mod sharpness;
mod stage;
pub mod visibility;

pub use clean::{clean, find_ghosts, find_outliers, CleanConfig, CleanStats};
pub use cloud::Cloud;
pub use color::{colorize, ColorCamera, ColorConfig, ColorStats, ImageSource};
pub use dense::{build_dense, DenseConfig, DenseStats, RaySource};
pub use export::{write_las, write_ply, LAS_SCALE};
pub use exposure::{estimate_gains, ExposureConfig, ImageGain, Observations, DARK, SATURATED};
pub use sharpness::sharpness;
pub use stage::{content_hash, load_stage, save_stage, StageError};

#[derive(Debug, thiserror::Error)]
pub enum MapError {
    #[error(
        "Laufbahn passt nicht zum Scan: {poses} Posen, aber {frames} Frames — `driftless slam` \
         mit denselben Einstellungen neu laufen lassen"
    )]
    PoseCount { poses: usize, frames: usize },
    #[error(
        "Laufbahn passt nicht zum Scan: Pose {index} hat die Zeit {pose_t:.6} s, der Frame \
         {frame_t:.6} s — `driftless slam` mit denselben Einstellungen neu laufen lassen"
    )]
    PoseTime {
        index: usize,
        pose_t: f64,
        frame_t: f64,
    },
    #[error("Kamerabilder: {0}")]
    Images(String),
    #[error("ungültige Einstellung: {0}")]
    Config(String),
    #[error("{path}: {source}")]
    Io {
        path: String,
        source: std::io::Error,
    },
    #[error("{path}: {source}")]
    Las { path: String, source: las::Error },
}
