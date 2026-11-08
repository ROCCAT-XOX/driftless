//! Floor plans from a point cloud: storeys, walls, openings, rooms.
//!
//! The input is the cleaned cloud of a map or model and its trajectory, in metres with Z up.
//! Everything here is deterministic on one system: no random numbers, sorts with a tie rule,
//! no order taken from a hash map.

mod grid;
pub mod levels;
pub mod openings;
mod sight;
#[cfg(any(test, feature = "test-util"))]
pub mod testutil;
pub mod walls;

pub use levels::{
    find_storeys, level_for, storey_points, Leveling, Storey, StoreyPoints, StoreySearch,
};
pub use openings::{find_openings, Opening, OpeningKind};
pub use walls::{find_walls, WallFace, WallPair, Walls};

/// The cloud and the trajectory a plan is computed from (metres, Z up).
#[derive(Debug, Clone, Copy)]
pub struct PlanInput<'a> {
    pub xyz: &'a [[f64; 3]],
    /// One colour per point, if the cloud has colours.
    pub rgb: Option<&'a [[u16; 3]]>,
    pub poses: &'a [[f64; 3]],
}

/// Settings of a plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanConfig {
    /// Edge of a cell of the top-down grid (metres).
    pub cell: f64,
    /// Which storey to draw, counted from 1 (lowest first); `None` if the map has only one.
    pub storey: Option<usize>,
}

impl Default for PlanConfig {
    fn default() -> Self {
        PlanConfig {
            cell: 0.02,
            storey: None,
        }
    }
}

/// Why no plan came out of a cloud.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PlanError {
    /// The step was cancelled through its [`driftless_io::progress::Progress`].
    #[error("abgebrochen")]
    Cancelled,
    #[error("Die Punktwolke hat zu wenige Punkte für einen Grundriss.")]
    TooFewPoints,
    #[error("Es gibt keine Laufbahn, ohne sie lässt sich kein Stockwerk bestimmen.")]
    NoPoses,
    #[error("Kein Boden gefunden: Unter der Laufbahn liegt keine große ebene Fläche.")]
    NoFloor,
    #[error("Keine Wände gefunden: Aus diesen Punkten entsteht kein Grundriss.")]
    NoWalls,
    #[error("Stockwerk {asked} gibt es nicht, gefunden wurden {found}.")]
    NoSuchStorey { asked: usize, found: usize },
}
