//! Point clouds, surfaces and trajectories in open formats: one writer and one reader per
//! format, used by the export step, the desktop program and the older map writers alike.
//!
//! Coordinates are metres in the frame of the map or the model, Z up. Colours are 16 bit
//! inside; every file is written to `<name>.tmp` next to its target and renamed at the end
//! ([`driftless_io::TempPath`]), so an error or a cancel never leaves half a file behind.

pub mod e57;
pub mod las;
pub mod mesh;
pub mod pcd;
pub mod ply;
mod points;
#[cfg(any(test, feature = "test-util"))]
pub mod testutil;
pub mod traj;

pub use points::{
    bounds_of, needs_double, read_points, write_points, Layout, PointFormat, PointSink,
    PointSource, Points, ReadEnd, ReadStart, ScanInfo, SINK_BLOCK,
};

/// Points per block when reading and writing: progress and the cancel check once per block.
pub const BLOCK: usize = 1 << 20;

/// From this absolute coordinate (m) on, PLY and PCD store xyz as `double`: `float` keeps
/// about 0.5 mm there.
pub const MAX_FLOAT_COORD: f64 = 8192.0;

/// Progress phase of every reader; its total is the number of points (or pages).
pub const PHASE_READ: &str = "Lesen";
/// Progress phase of every writer.
pub const PHASE_WRITE: &str = "Schreiben";

/// Why a file could not be written or read. The messages are German, for the user.
#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("{path}: {source}")]
    Io {
        path: String,
        source: std::io::Error,
    },
    /// The content does not fit the format (German message).
    #[error("{path}: {msg}")]
    Format { path: String, msg: String },
    /// Stopped through [`driftless_io::progress::Progress::cancel`].
    #[error("abgebrochen")]
    Cancelled,
}

impl ExportError {
    pub(crate) fn io(path: &std::path::Path, source: std::io::Error) -> ExportError {
        ExportError::Io {
            path: path.display().to_string(),
            source,
        }
    }

    pub(crate) fn format(path: &std::path::Path, msg: impl Into<String>) -> ExportError {
        ExportError::Format {
            path: path.display().to_string(),
            msg: msg.into(),
        }
    }
}

/// `Err(Cancelled)` once the export was asked to stop.
pub(crate) fn check_cancel(progress: &driftless_io::progress::Progress) -> Result<(), ExportError> {
    if progress.is_cancelled() {
        Err(ExportError::Cancelled)
    } else {
        Ok(())
    }
}
