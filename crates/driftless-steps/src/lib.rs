//! The steps of Driftless – inspect a scan, SLAM, manual corrections, map, surface, merge,
//! compare – as functions that the command line and the desktop program both call. Every
//! step reports to a [`driftless_io::progress::Progress`] (phases, counts, the lines the CLI
//! prints) and stops at its next check once that is cancelled.

pub mod compare;
pub mod correct;
mod error;
pub mod export;
pub mod history;
pub mod inspect;
pub mod map;
pub mod merge;
pub mod overlay;
pub mod pairs;
pub mod scan;
pub mod slam;
pub mod surface;

pub use error::StepError;

use serde::Serialize;

/// The program that made a result: version and commit (`packaging/build_commit.rs`). Every
/// step report (`slam.json`, `map.json`, `oberflaeche.json`, `modell.json`) names it under
/// [`PROGRAM_KEY`]. Informational only: part of no input id, so a result of another build
/// that fits still counts as current, and a report written before the field still reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Program {
    pub version: &'static str,
    pub commit: &'static str,
}

/// This build.
pub const PROGRAM: Program = Program {
    version: env!("CARGO_PKG_VERSION"),
    commit: env!("DRIFTLESS_COMMIT"),
};

/// Key of [`PROGRAM`] in a report.
pub const PROGRAM_KEY: &str = "programm";

/// A report with [`PROGRAM`] added under [`PROGRAM_KEY`], after its own fields.
#[derive(Serialize)]
struct WithProgram<'a, T: Serialize> {
    #[serde(flatten)]
    report: &'a T,
    programm: Program,
}

/// The text of a report file: `report` with [`PROGRAM`], pretty, with a final newline.
pub(crate) fn report_text<T: Serialize>(report: &T) -> Result<String, StepError> {
    let text = serde_json::to_string_pretty(&WithProgram {
        report,
        programm: PROGRAM,
    })
    .map_err(error::failed)?;
    Ok(text + "\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_is_this_build() {
        assert_eq!(PROGRAM.version, env!("CARGO_PKG_VERSION"));
        assert!(!PROGRAM.commit.is_empty());
        assert!(!PROGRAM.commit.contains(char::is_whitespace));
    }

    #[test]
    fn report_text_adds_the_program_last() {
        #[derive(Serialize)]
        struct R {
            a: u32,
        }
        let text = report_text(&R { a: 1 }).unwrap();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(
            value,
            serde_json::json!({"a": 1, "programm": {
                "version": PROGRAM.version,
                "commit": PROGRAM.commit,
            }})
        );
        assert!(text.ends_with("}\n"));
        assert!(text.find("\"a\"").unwrap() < text.find(PROGRAM_KEY).unwrap());
    }
}
