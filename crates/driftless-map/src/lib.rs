//! Dense point cloud from the loop-closed trajectory, and its export.
//!
//! [`build_dense`] runs the (deterministic) odometry again over the same frames, so every
//! deskewed sweep comes back in the body frame, and places it with the SLAM-corrected pose of
//! that frame. One point is kept per voxel: the one seen from closest, since near points are
//! the most accurate. [`clean`] removes ghosts (things that moved) and isolated outliers;
//! [`write_las`] and [`write_ply`] write the [`Cloud`].

mod clean;
mod cloud;
mod dense;
mod export;

pub use clean::{clean, find_ghosts, find_outliers, CleanConfig, CleanStats};
pub use cloud::Cloud;
pub use dense::{build_dense, DenseConfig, DenseStats, RaySource};
pub use export::{write_las, write_ply};

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
