//! SE(3) pose graph optimisation.
//!
//! Nodes are absolute poses (later: submap anchors), edges relative measurements between two
//! nodes (odometry between consecutive submaps, loop closures). [`optimize`] runs
//! Levenberg-Marquardt on the sparse normal equations; loop edges can carry a Cauchy kernel
//! so a wrong closure cannot pull the whole trajectory apart.

pub mod optimize;
pub mod se3;

pub use nalgebra;
pub use optimize::{optimize, Edge, OptimizeReport, PoseGraph};
