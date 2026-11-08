//! Synthetic clouds for tests: deterministic for a seed, no scan data.

use crate::{write_points, Layout, PointFormat, Points, ScanInfo};
use driftless_io::progress::Progress;
use driftless_traj::{write_tum, Pose};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::fs;
use std::path::Path;

/// splitmix64: a small, fixed pseudo-random sequence.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn u16(&mut self) -> u16 {
        (self.next() >> 48) as u16
    }
}

/// Number of scans of a synthetic cloud with the `source` column.
pub const SYNTHETIC_SCANS: u16 = 3;

/// `n` points in a room of 20 × 10 × 3 m around (0, 0, 1.5) with the columns of `layout`:
/// full 16-bit colours and intensities, scans 0..[`SYNTHETIC_SCANS`] with one [`ScanInfo`]
/// each, times from 1.7e9 s, flags 0–3. The same `seed` gives the same cloud.
pub fn synthetic_points(n: usize, seed: u64, layout: Layout) -> Points {
    let mut rng = Rng(seed);
    let xyz = (0..n)
        .map(|_| {
            [
                rng.unit() * 20.0 - 10.0,
                rng.unit() * 10.0 - 5.0,
                rng.unit() * 3.0,
            ]
        })
        .collect();
    let rgb = layout
        .rgb
        .then(|| (0..n).map(|_| [rng.u16(), rng.u16(), rng.u16()]).collect());
    let intensity = layout
        .intensity
        .then(|| (0..n).map(|_| rng.u16()).collect());
    let source = layout.source.then(|| {
        (0..n)
            .map(|_| (rng.next() % u64::from(SYNTHETIC_SCANS)) as u16)
            .collect()
    });
    let time = layout
        .time
        .then(|| (0..n).map(|i| 1.7e9 + i as f64 * 1e-4).collect());
    let flags = layout
        .flags
        .then(|| (0..n).map(|_| (rng.next() % 4) as u8).collect());
    let scans = if layout.source {
        (0..SYNTHETIC_SCANS)
            .map(|s| ScanInfo {
                source: s,
                name: format!("Scan {}", s + 1),
                pose: Isometry3::from_parts(
                    Translation3::new(f64::from(s) * 2.0, -1.0, 0.25),
                    UnitQuaternion::from_euler_angles(0.0, 0.0, f64::from(s) * 0.5),
                ),
            })
            .collect()
    } else {
        Vec::new()
    };
    Points {
        xyz,
        rgb,
        intensity,
        source,
        time,
        flags,
        scans,
    }
}

/// A map folder as the floor plan reads it, for the tests of the step, the command line and
/// the app: `color.las` of the points `xyz` (all one colour), a `map.json` that holds only
/// `steps.clean.input_id` and a `traj.tum` with a level pose per entry of `poses`, 0.1 s apart.
///
/// # Panics
/// If a file cannot be written.
pub fn write_map_folder(dir: &Path, xyz: &[[f64; 3]], poses: &[[f64; 3]]) {
    fs::create_dir_all(dir).expect("test folder");
    let points = Points {
        xyz: xyz.to_vec(),
        rgb: Some(vec![[40_000, 30_000, 20_000]; xyz.len()]),
        intensity: None,
        source: None,
        time: None,
        flags: None,
        scans: Vec::new(),
    };
    write_points(
        &dir.join("color.las"),
        PointFormat::Las,
        &points,
        &Progress::new(),
    )
    .expect("color.las");
    fs::write(
        dir.join("map.json"),
        "{\"steps\":{\"clean\":{\"input_id\":\"00000000000000aa\"}}}\n",
    )
    .expect("map.json");
    let poses: Vec<Pose> = poses
        .iter()
        .enumerate()
        .map(|(i, p)| Pose {
            t: i as f64 * 0.1,
            p: Vector3::from(*p),
            q: UnitQuaternion::identity(),
        })
        .collect();
    write_tum(&dir.join("traj.tum"), &poses).expect("traj.tum");
}
