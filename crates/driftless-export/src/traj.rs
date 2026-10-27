//! Trajectories: TUM (as `driftless slam` writes it, through [`driftless_traj::write_tum`])
//! and CSV for spreadsheets.
//!
//! CSV: header `zeit;x;y;z;qx;qy;qz;qw`, one line per pose with the same numbers as TUM
//! (`{:.9}`), `;` between them and `.` as the decimal point (German spreadsheets take `;`
//! as the separator; the point keeps the numbers as they are in TUM), `\n` at the end of each
//! line.

use crate::ExportError;
use driftless_io::TempPath;
use driftless_traj::{write_tum, Pose, TrajError};
use std::fmt::Write as _;
use std::path::Path;

/// A file format for trajectories.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrajFormat {
    Tum,
    Csv,
}

/// Header line of a CSV trajectory.
pub const CSV_HEADER: &str = "zeit;x;y;z;qx;qy;qz;qw";

/// Writes `poses` as `format` through `<name>.tmp` next to `path`.
pub fn write_traj(path: &Path, poses: &[Pose], format: TrajFormat) -> Result<(), ExportError> {
    let temp = TempPath::new(path);
    match format {
        TrajFormat::Tum => write_tum(temp.tmp(), poses).map_err(|e| match e {
            TrajError::Io { source, .. } => ExportError::io(path, source),
            other => ExportError::format(path, other.to_string()),
        })?,
        TrajFormat::Csv => {
            let mut s = String::with_capacity(16 + poses.len() * 100);
            s.push_str(CSV_HEADER);
            s.push('\n');
            for p in poses {
                let q = p.q.quaternion();
                let _ = writeln!(
                    s,
                    "{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9}",
                    p.t, p.p.x, p.p.y, p.p.z, q.i, q.j, q.k, q.w
                );
            }
            std::fs::write(temp.tmp(), s).map_err(|e| ExportError::io(path, e))?;
        }
    }
    temp.commit().map_err(|e| ExportError::io(path, e))
}
