//! Trajectories: TUM (as `driftless slam` writes it, through [`driftless_traj::write_tum`])
//! and CSV for spreadsheets.
//!
//! CSV, as German Excel reads it: header `stempel;zeit_s;x;y;z;qx;qy;qz;qw`, one line per
//! pose, `;` between the fields and a decimal comma, `\n` at the end of each line. `stempel` is
//! the stamp of the pose as in TUM (the clock of the bags, `{:.9}`), `zeit_s` the seconds
//! since the first pose (`{:.3}`), the rest the numbers of TUM (`{:.9}`).

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
pub const CSV_HEADER: &str = "stempel;zeit_s;x;y;z;qx;qy;qz;qw";

/// `v` with `digits` decimals and a decimal comma.
fn comma(v: f64, digits: usize) -> String {
    format!("{v:.digits$}").replace('.', ",")
}

/// Writes `poses` as `format` through `<name>.tmp` next to `path`.
pub fn write_traj(path: &Path, poses: &[Pose], format: TrajFormat) -> Result<(), ExportError> {
    let temp = TempPath::new(path);
    match format {
        TrajFormat::Tum => write_tum(temp.tmp(), poses).map_err(|e| match e {
            TrajError::Io { source, .. } => ExportError::io(path, source),
            other => ExportError::format(path, other.to_string()),
        })?,
        TrajFormat::Csv => {
            let mut s = String::with_capacity(40 + poses.len() * 110);
            s.push_str(CSV_HEADER);
            s.push('\n');
            let t0 = poses.first().map_or(0.0, |p| p.t);
            for p in poses {
                let q = p.q.quaternion();
                let rest = [p.p.x, p.p.y, p.p.z, q.i, q.j, q.k, q.w].map(|v| comma(v, 9));
                let _ = writeln!(
                    s,
                    "{};{};{}",
                    comma(p.t, 9),
                    comma(p.t - t0, 3),
                    rest.join(";")
                );
            }
            std::fs::write(temp.tmp(), s).map_err(|e| ExportError::io(path, e))?;
        }
    }
    temp.commit().map_err(|e| ExportError::io(path, e))
}
