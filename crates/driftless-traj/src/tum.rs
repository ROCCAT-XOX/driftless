//! TUM trajectory format: `timestamp tx ty tz qx qy qz qw`.

use crate::{Pose, TrajError};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::fmt::Write as _;
use std::path::Path;

fn parse_line(line: usize, text: &str) -> Result<Pose, TrajError> {
    let err = |msg: String| TrajError::Parse { line, msg };
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
        out.push(parse_line(i + 1, l)?);
    }
    Ok(out)
}

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
    std::fs::write(path, s).map_err(|source| TrajError::Io {
        path: path.display().to_string(),
        source,
    })
}
