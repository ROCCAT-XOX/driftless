//! Streaming reader for ROS1 bag files (format 2.0).
//!
//! Chunks are read and decompressed one at a time; the index is not required,
//! so unindexed and truncated recordings stay readable. [`BagReader::message_places`] finds
//! where the messages lie without reading the chunks whole, and
//! [`BagReader::read_message`] reads one message at its place.

mod places;
mod reader;
mod record;
#[cfg(any(test, feature = "test-util"))]
pub mod writer;

pub use places::{ChunkCompression, MessagePlace, PlacedMessage, Places};
pub use reader::{BagReader, Connection, RawMessage};
pub use record::Time;

/// Errors produced while reading a bag.
#[derive(Debug, thiserror::Error)]
pub enum BagError {
    #[error("Bag-Datei nicht lesbar: {0}")]
    Io(#[from] std::io::Error),
    #[error("keine ROS1-Bag-Datei (Kennung `#ROSBAG V2.0` fehlt)")]
    NotABag,
    #[error("Bag-Datei abgeschnitten: Eintrag bei Byte {offset} unvollständig")]
    Truncated { offset: u64 },
    #[error("Kompression „{0}“ der Bag-Datei nicht unterstützt")]
    UnsupportedCompression(String),
    #[error("Bag-Datei ist beschädigt: {0}")]
    Malformed(String),
}
