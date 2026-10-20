//! SLAM on top of the odometry: submaps, loop detection and the pose graph over submap
//! anchors.

pub mod cache;
pub mod corrections;
pub mod loops;
pub mod pipeline;
pub mod report;
pub mod submap;

pub use cache::{load_cache, read_cache, save_cache, CacheContents, CacheError};
pub use corrections::{Corrections, LoopEntry};
pub use loops::{
    find_candidates, match_manual_loops, position_only_relative, verify, verify_manual,
    LoopCandidate, LoopCheck, LoopConfig, LoopVerdict, ManualLoop, ManualLoopError, ManualMatch,
    POSITION_ONLY,
};
pub use pipeline::{build_submaps, close_loops, run_slam, solve, SlamConfig, SlamResult};
pub use report::{scan_start, SlamReport};
pub use submap::{Submap, SubmapBuilder, SubmapConfig, SubmapFrame};
