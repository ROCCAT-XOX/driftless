//! SLAM on top of the odometry: submaps, loop detection and the pose graph over submap
//! anchors.

pub mod submap;

pub use submap::{Submap, SubmapBuilder, SubmapConfig, SubmapFrame};
