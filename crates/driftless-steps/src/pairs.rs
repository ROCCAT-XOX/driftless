//! Point pairs marked by hand between two scans of a model (`punktpaare.toml` in the models
//! folder of a project). Only the entry type for now; the merge does not use them yet.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// File of the point pairs, in `<projekt>/modelle/`.
pub const POINT_PAIRS_FILE: &str = "punktpaare.toml";

/// Points marked by hand on two scans that show the same places.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PointPairEntry {
    /// Map folders of the two scans.
    pub a: PathBuf,
    pub b: PathBuf,
    /// The points, pair by pair (at least 3), each in the world frame of its own scan.
    pub points_a: Vec<[f64; 3]>,
    pub points_b: Vec<[f64; 3]>,
    /// Switched off entries are kept but not used.
    pub enabled: bool,
    pub note: String,
    /// When the entry was added.
    pub added: String,
}
