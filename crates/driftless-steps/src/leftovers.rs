//! What an interrupted run leaves behind, and how a step keeps it from counting as current.
//!
//! A step writes each of its files as `<name>.tmp` and renames it at the end
//! ([`driftless_io::TempPath`]), its report last. A hard kill (a power loss, a crash, a
//! forced quit, a second Ctrl+C) skips the removal of the `.tmp` and can stop the step
//! between two of its files. Therefore:
//!
//! - at its start a step removes the `.tmp` of the files it writes ([`remove_stale_tmp`]),
//!   and only those: the `.tmp` of another step or program in the same folder stays;
//! - before it writes its first file it removes its report ([`remove_report`]), the file the
//!   `*_fits` checks read, and writes it again last: a folder without its report never
//!   counts as current.

use crate::error::StepError;
use driftless_io::progress::Progress;
use std::fs;
use std::io;
use std::path::Path;

/// Removes from `dir` every `<name>.tmp` with `owns(name)` (a file, not a folder): what a
/// killed run of the step left. Says in the log what went; a file that cannot be removed
/// stays (writing it again fails then with a message).
pub(crate) fn remove_stale_tmp(dir: &Path, owns: impl Fn(&str) -> bool, progress: &Progress) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    let mut removed = Vec::new();
    let mut bytes = 0;
    for e in entries.flatten() {
        let name = e.file_name();
        let Some(name) = name.to_str() else {
            continue;
        };
        let ours = name.strip_suffix(".tmp").is_some_and(&owns);
        if !ours || !e.file_type().is_ok_and(|t| t.is_file()) {
            continue;
        }
        let size = e.metadata().map_or(0, |m| m.len());
        if fs::remove_file(e.path()).is_ok() {
            removed.push(name.to_string());
            bytes += size;
        }
    }
    if !removed.is_empty() {
        removed.sort();
        progress.info(format!(
            "Reste eines abgebrochenen Laufs entfernt: {} ({:.1} MB)",
            removed.join(", "),
            bytes as f64 / 1e6
        ));
    }
}

/// Removes the report `path` of a step before the step writes its first file (it writes
/// it again last). Fails if it is there and cannot be removed: then nothing may be
/// written, or the old report would describe new files.
pub(crate) fn remove_report(path: &Path) -> Result<(), StepError> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!(
            "Bericht {} nicht entfernbar ({e}); nichts geschrieben, damit er keine neuen \
             Dateien beschreibt",
            path.display()
        )
        .into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_io::progress::LogLine;

    #[test]
    fn removes_only_own_tmp_files() {
        let dir = std::env::temp_dir().join(format!("driftless-leftovers-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(dir.join("ordner.las.tmp")).unwrap();
        for f in ["wolke.las.tmp", "wolke.las", "fremd.las.tmp", "wolke.tmp"] {
            fs::write(dir.join(f), b"halb").unwrap();
        }
        let p = Progress::new();
        remove_stale_tmp(&dir, |n| n == "wolke.las" || n == "ordner.las", &p);
        assert!(!dir.join("wolke.las.tmp").exists());
        assert!(dir.join("wolke.las").exists(), "the file itself stays");
        assert!(dir.join("fremd.las.tmp").exists());
        assert!(dir.join("wolke.tmp").exists());
        assert!(
            dir.join("ordner.las.tmp").is_dir(),
            "a folder is no leftover"
        );
        assert_eq!(
            p.take_log(),
            [LogLine::Info(
                "Reste eines abgebrochenen Laufs entfernt: wolke.las.tmp (0.0 MB)".into()
            )]
        );
        // Nothing to remove: nothing said; a missing folder is fine.
        remove_stale_tmp(&dir, |n| n == "wolke.las", &p);
        remove_stale_tmp(&dir.join("fehlt"), |_| true, &p);
        assert!(p.take_log().is_empty());

        remove_report(&dir.join("wolke.las")).unwrap();
        assert!(!dir.join("wolke.las").exists());
        remove_report(&dir.join("wolke.las")).unwrap();
        // A folder in the way cannot be removed as a file.
        let err = remove_report(&dir.join("ordner.las.tmp")).unwrap_err();
        assert!(err.to_string().contains("nicht entfernbar"), "{err}");
        fs::remove_dir_all(&dir).unwrap();
    }
}
