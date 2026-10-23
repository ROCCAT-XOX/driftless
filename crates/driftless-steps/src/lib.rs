//! The steps of Driftless – inspect a scan, SLAM, manual corrections, map, surface, compare – as
//! functions that the command line and the desktop program both call. Every step reports to
//! a [`driftless_io::progress::Progress`] (phases, counts, the lines the CLI prints) and
//! stops at its next check once that is cancelled.

pub mod compare;
pub mod correct;
mod error;
pub mod history;
pub mod inspect;
pub mod map;
pub mod overlay;
pub mod scan;
pub mod slam;
pub mod surface;

pub use error::StepError;
