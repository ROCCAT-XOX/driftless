//! The error of a step: cancelled, or failed with a German message for the user.

use driftless_map::MapError;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StepError {
    /// Stopped through [`driftless_io::progress::Progress::cancel`].
    Cancelled,
    /// Failed; the message is shown as it is (the CLI prefixes "Fehler: ").
    Failed(String),
    /// A manual correction could not be applied; `submaps.bin` is written, so a reuse works
    /// once the entry is switched off. The message has no hint how: the CLI and the desktop
    /// program each add their own.
    BadCorrection(String),
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StepError::Cancelled => f.write_str("abgebrochen"),
            StepError::Failed(m) | StepError::BadCorrection(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for StepError {}

impl From<String> for StepError {
    fn from(m: String) -> Self {
        StepError::Failed(m)
    }
}

impl From<&str> for StepError {
    fn from(m: &str) -> Self {
        StepError::Failed(m.to_string())
    }
}

impl From<MapError> for StepError {
    fn from(e: MapError) -> Self {
        match e {
            MapError::Cancelled => StepError::Cancelled,
            e => StepError::Failed(e.to_string()),
        }
    }
}

/// Keeps a cancel that travelled through a boxed error.
impl From<Box<dyn std::error::Error>> for StepError {
    fn from(e: Box<dyn std::error::Error>) -> Self {
        if let Some(s) = e.downcast_ref::<StepError>() {
            return s.clone();
        }
        if let Some(MapError::Cancelled) = e.downcast_ref::<MapError>() {
            return StepError::Cancelled;
        }
        StepError::Failed(e.to_string())
    }
}

/// `Err(Cancelled)` once the step was asked to stop.
pub(crate) fn check_cancel(progress: &driftless_io::progress::Progress) -> Result<(), StepError> {
    if progress.is_cancelled() {
        Err(StepError::Cancelled)
    } else {
        Ok(())
    }
}

/// Turns any displayable error into a failure with its message.
pub(crate) fn failed(e: impl fmt::Display) -> StepError {
    StepError::Failed(e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_and_cancel_survive_conversions() {
        assert_eq!(StepError::Cancelled.to_string(), "abgebrochen");
        assert_eq!(StepError::from("kaputt").to_string(), "kaputt");
        assert_eq!(StepError::from(MapError::Cancelled), StepError::Cancelled);
        let boxed: Box<dyn std::error::Error> = Box::new(MapError::Cancelled);
        assert_eq!(StepError::from(boxed), StepError::Cancelled);
        let boxed: Box<dyn std::error::Error> = Box::new(StepError::Cancelled);
        assert_eq!(StepError::from(boxed), StepError::Cancelled);
        let boxed: Box<dyn std::error::Error> = "nicht lesbar".into();
        assert_eq!(
            StepError::from(boxed),
            StepError::Failed("nicht lesbar".into())
        );
    }
}
