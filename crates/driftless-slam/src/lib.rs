//! SLAM on top of the odometry: submaps, loop detection and the pose graph over submap
//! anchors.

pub mod loops;
pub mod pipeline;
pub mod report;
pub mod submap;

pub use loops::{
    find_candidates, verify, LoopCandidate, LoopCheck, LoopConfig, LoopVerdict, ManualLoop,
};
pub use pipeline::{close_loops, run_slam, solve, SlamConfig, SlamResult};
pub use report::{scan_start, SlamReport};
pub use submap::{Submap, SubmapBuilder, SubmapConfig, SubmapFrame};
