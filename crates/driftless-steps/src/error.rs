//! The error of a step: cancelled, or failed with a German message for the user.

use driftless_export::ExportError;
use driftless_map::surface::edit::EditError;
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

impl From<ExportError> for StepError {
    fn from(e: ExportError) -> Self {
        match e {
            ExportError::Cancelled => StepError::Cancelled,
            e => StepError::Failed(e.to_string()),
        }
    }
}

/// A touch-up step that could not run: cancelled, or failed with its German text (a step
/// that only refused, such as [`EditError::NotFlat`], is no error of the touch-up step and is
/// told apart before).
impl From<EditError> for StepError {
    fn from(e: EditError) -> Self {
        match e {
            EditError::Cancelled => StepError::Cancelled,
            EditError::Map(e) => StepError::from(e),
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
        if let Some(ExportError::Cancelled) = e.downcast_ref::<ExportError>() {
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
        assert_eq!(
            StepError::from(ExportError::Cancelled),
            StepError::Cancelled
        );
        let boxed: Box<dyn std::error::Error> = Box::new(ExportError::Cancelled);
        assert_eq!(StepError::from(boxed), StepError::Cancelled);
        let e = ExportError::Format {
            path: "a.las".into(),
            msg: "kaputt".into(),
        };
        assert_eq!(
            StepError::from(e),
            StepError::Failed("a.las: kaputt".into())
        );
        let boxed: Box<dyn std::error::Error> = Box::new(StepError::Cancelled);
        assert_eq!(StepError::from(boxed), StepError::Cancelled);
        assert_eq!(StepError::from(EditError::Cancelled), StepError::Cancelled);
        assert_eq!(
            StepError::from(EditError::Map(MapError::Cancelled)),
            StepError::Cancelled
        );
        assert_eq!(
            StepError::from(EditError::NoImages),
            StepError::Failed("Kein Bild zeigt diese Stelle; sie bleibt, wie sie ist.".into())
        );
        let boxed: Box<dyn std::error::Error> = "nicht lesbar".into();
        assert_eq!(
            StepError::from(boxed),
            StepError::Failed("nicht lesbar".into())
        );
    }
}
