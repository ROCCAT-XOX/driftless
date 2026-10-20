//! SLAM on top of the odometry: submaps, loop detection and the pose graph over submap
//! anchors.

pub mod loops;
pub mod submap;

pub use loops::{find_candidates, verify, LoopCandidate, LoopCheck, LoopConfig, LoopVerdict};
pub use submap::{Submap, SubmapBuilder, SubmapConfig, SubmapFrame};
