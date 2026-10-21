//! File input/output: bags, trajectories, point clouds.

pub mod bag;
pub mod calib;
pub mod camera;
pub mod images;
pub mod msg;
pub mod progress;
pub mod project;
pub mod testdata;
#[cfg(any(test, feature = "test-util"))]
pub mod testscan;
