//! TUM trajectory format: `timestamp tx ty tz qx qy qz qw`.

use crate::{Pose, TrajError};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::fmt::Write as _;
use std::path::Path;

fn parse_line(path: &Path, line: usize, text: &str) -> Result<Pose, TrajError> {
    let err = |msg: String| TrajError::Parse {
        path: path.display().to_string(),
        line,
        msg,
    };
    let fields: Vec<&str> = text.split_whitespace().collect();
    if fields.len() != 8 {
        return Err(err(format!("8 Felder erwartet, {} gefunden", fields.len())));
    }
    let mut v = [0.0f64; 8];
    for (i, f) in fields.iter().enumerate() {
        v[i] = f
            .parse::<f64>()
            .ok()
            .filter(|x| x.is_finite())
            .ok_or_else(|| err(format!("ungültige Zahl '{f}' in Feld {}", i + 1)))?;
    }
    let q = Quaternion::new(v[7], v[4], v[5], v[6]); // w, x, y, z
    if q.norm() < 1e-12 {
        return Err(err("Quaternion ist null".into()));
    }
    Ok(Pose {
        t: v[0],
        p: Vector3::new(v[1], v[2], v[3]),
        q: UnitQuaternion::from_quaternion(q),
    })
}

pub fn read_tum(path: &Path) -> Result<Vec<Pose>, TrajError> {
    let text = std::fs::read_to_string(path).map_err(|source| TrajError::Io {
        path: path.display().to_string(),
        source,
    })?;
    let mut out = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let l = raw.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        out.push(parse_line(path, i + 1, l)?);
    }
    Ok(out)
}

/// Writes `poses` to `path` through `<name>.tmp` next to it, renamed at the end: an
/// interruption leaves the old file whole, never half a trajectory.
pub fn write_tum(path: &Path, poses: &[Pose]) -> Result<(), TrajError> {
    let mut s = String::from("# timestamp tx ty tz qx qy qz qw\n");
    for p in poses {
        let q = p.q.quaternion();
        let _ = writeln!(
            s,
            "{:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
            p.t, p.p.x, p.p.y, p.p.z, q.i, q.j, q.k, q.w
        );
    }
    driftless_io::atomic_write(path, s.as_bytes()).map_err(|source| TrajError::Io {
        path: path.display().to_string(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn pose(t: f64) -> Pose {
        Pose {
            t,
            p: Vector3::new(t, 0.0, 0.0),
            q: UnitQuaternion::identity(),
        }
    }

    /// The file is replaced as a whole: a second name of the old file (a hard link) keeps the
    /// old trajectory, no `.tmp` is left, and a write that fails leaves none either.
    #[test]
    fn write_tum_replaces_the_file_as_a_whole() {
        let dir = std::env::temp_dir().join(format!("driftless-traj-tum-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("traj.tum");
        write_tum(&path, &[pose(1.0)]).unwrap();
        let link = dir.join("traj.alt.tum");
        let _ = fs::remove_file(&link);
        fs::hard_link(&path, &link).unwrap();
        let before = fs::read(&link).unwrap();
        write_tum(&path, &[pose(2.0), pose(3.0)]).unwrap();
        assert_eq!(
            fs::read(&link).unwrap(),
            before,
            "written into the old file"
        );
        let back = read_tum(&path).unwrap();
        assert_eq!(back.iter().map(|p| p.t).collect::<Vec<_>>(), [2.0, 3.0]);
        assert!(!dir.join("traj.tum.tmp").exists());

        // The target is a folder with something in it: the rename fails.
        let blocked = dir.join("ordner.tum");
        fs::create_dir_all(blocked.join("x")).unwrap();
        assert!(matches!(
            write_tum(&blocked, &[pose(1.0)]),
            Err(TrajError::Io { .. })
        ));
        assert!(!dir.join("ordner.tum.tmp").exists());
        fs::remove_dir_all(&dir).unwrap();
    }
}
