//! Streaming reader for ROS1 bag files (format 2.0).
//!
//! Chunks are read and decompressed one at a time; the index is not required,
//! so unindexed and truncated recordings stay readable.

mod reader;
mod record;
#[cfg(any(test, feature = "test-util"))]
pub mod writer;

pub use reader::{BagReader, Connection, RawMessage};
pub use record::Time;

/// Errors produced while reading a bag.
#[derive(Debug, thiserror::Error)]
pub enum BagError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("not a ROS1 bag (missing `#ROSBAG V2.0` magic)")]
    NotABag,
    #[error("bag is truncated: record at byte offset {offset} is incomplete")]
    Truncated { offset: u64 },
    #[error("unsupported chunk compression `{0}`")]
    UnsupportedCompression(String),
    #[error("malformed bag: {0}")]
    Malformed(String),
}
