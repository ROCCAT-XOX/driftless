//! Odometry and SLAM.
//!
//! Milestone 1 baseline: loosely coupled LiDAR-IMU odometry without loop closure.
//! [`run_odometry`] predicts each sweep with the IMU ([`imu_integrate`]), removes the motion
//! blur ([`deskew`]), downsamples ([`voxel_map`]) and registers it point-to-plane against a
//! local voxel map ([`icp`]). The heavy inner loop (neighbour search and per-point normal
//! equation terms in `icp`) is a pure per-point map followed by a fixed-order reduction,
//! so it can move to a GPU compute pass without changing the results' structure.

pub mod deskew;
pub mod icp;
pub mod imu_integrate;
pub mod pipeline;
pub mod quality;
#[cfg(feature = "test-util")]
pub mod testutil;
pub mod voxel_map;

pub use pipeline::{
    run_odometry, run_odometry_with, OdomConfig, OdomResult, OdomStats, OUTPUT_MAP_VOXEL,
};
pub use quality::{FrameOutput, FrameQuality, DEGENERACY_GOOD};
pub use voxel_map::voxel_downsample;
