//! E57 point clouds (ASTM E2807) through the `e57` crate: a writer with one `data3D` entry
//! per scan, its name and its pose, and a reader for every entry of a file.
//!
//! The writer stores the points of a scan in that scan's own frame (`pose⁻¹ · p`, xyz as
//! `double`) and the pose next to them, so that programs which show scans separately put each
//! one where it belongs. Colour is `0–255` when every value came from 8 bit (a multiple of
//! 257), else `0–65535`; intensity is `0–65535`. There is no creation date, and the GUIDs
//! come from the file name, so the same cloud gives the same bytes.
//!
//! The reader applies the poses, turns spherical coordinates into cartesian ones and skips
//! points without a position (invalid, or only a direction). Images are not read.

use crate::{
    check_cancel, ExportError, PointSource, Points, ScanInfo, BLOCK, PHASE_READ, PHASE_WRITE,
};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use e57::{
    CartesianCoordinate, ColorLimits, E57Reader, E57Writer, IntensityLimits, Record,
    RecordDataType, RecordName, RecordValue,
};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use std::fs::{File, OpenOptions};
use std::io::BufReader;
use std::path::Path;

/// Buffer of the file reader (bytes).
const READ_BUFFER: usize = 1 << 16;
/// Scan number of the file itself in [`guid`].
const FILE_GUID: u32 = u32::MAX;

/// One `data3D` entry to write: the points `order[start..start + len]` (or `start..start +
/// len` of the source without an order).
struct Entry {
    name: String,
    pose: Isometry3<f64>,
    start: usize,
    len: usize,
}

/// Writes `src` as E57 through `<name>.tmp` next to `path`. Progress: phase [`PHASE_WRITE`]
/// counting points, a cancel check per [`BLOCK`]; an error or a cancel leaves `path` as it was
/// and no `.tmp`.
///
/// One `data3D` entry per scan:
/// - with a scan column and [`PointSource::scans`], one per [`ScanInfo`] (its name and pose),
///   then one per scan number without a `ScanInfo` (`Scan <n+1>`, identity pose);
/// - with a scan column and no `scans`, one per scan number that occurs (`Scan <n+1>`);
/// - without a scan column, one entry: the only [`ScanInfo`] if there is exactly one, else
///   the file name without extension at the identity pose.
///
/// The points of an entry keep the order of the source. Times and flags are not written.
pub fn write_e57(
    path: &Path,
    src: &dyn PointSource,
    progress: &Progress,
) -> Result<(), ExportError> {
    check_cancel(progress)?;
    let n = src.len();
    if u32::try_from(n).is_err() {
        return Err(ExportError::format(
            path,
            format!("zu viele Punkte für E57 ({n}, höchstens {})", u32::MAX),
        ));
    }
    let layout = src.layout();
    let mut eight_bit = true;
    for i in 0..n {
        let p = src.xyz(i);
        if !p.iter().all(|v| v.is_finite()) {
            return Err(ExportError::format(
                path,
                format!("Punkt {} hat keine gültige Lage ({p:?})", i + 1),
            ));
        }
        if layout.rgb && eight_bit {
            eight_bit = src.rgb(i).iter().all(|c| c % 257 == 0);
        }
    }
    let file_name = path
        .file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let (entries, order) = plan(src, &stem);

    let colour_max: i64 = if eight_bit { 255 } else { 65_535 };
    let mut prototype = [
        RecordName::CartesianX,
        RecordName::CartesianY,
        RecordName::CartesianZ,
    ]
    .map(|name| Record {
        name,
        data_type: RecordDataType::Double {
            min: None,
            max: None,
        },
    })
    .to_vec();
    if layout.rgb {
        for name in [
            RecordName::ColorRed,
            RecordName::ColorGreen,
            RecordName::ColorBlue,
        ] {
            prototype.push(Record {
                name,
                data_type: RecordDataType::Integer {
                    min: 0,
                    max: colour_max,
                },
            });
        }
    }
    if layout.intensity {
        prototype.push(Record {
            name: RecordName::Intensity,
            data_type: RecordDataType::Integer {
                min: 0,
                max: 65_535,
            },
        });
    }

    let temp = TempPath::new(path);
    let io = |e| ExportError::io(path, e);
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(temp.tmp())
        .map_err(io)?;
    // Declared after `temp`, so it is dropped (and the file closed) before the `.tmp` is
    // removed on an error.
    let mut writer = E57Writer::new(file, &guid(&file_name, n as u64, FILE_GUID))
        .map_err(|e| failed(path, e))?;
    writer.set_creation(None);
    progress.phase(PHASE_WRITE, Some(n as u64));
    for (k, entry) in entries.iter().enumerate() {
        check_cancel(progress)?;
        let id = guid(&file_name, entry.len as u64, k as u32);
        let mut pc = writer
            .add_pointcloud(&id, prototype.clone())
            .map_err(|e| failed(path, e))?;
        pc.set_name(Some(cdata_safe(&entry.name)));
        pc.set_transform(Some(transform(&entry.pose)));
        if layout.rgb {
            let (min, max) = (
                Some(RecordValue::Integer(0)),
                Some(RecordValue::Integer(colour_max)),
            );
            pc.set_color_limits(Some(ColorLimits {
                red_min: min.clone(),
                red_max: max.clone(),
                green_min: min.clone(),
                green_max: max.clone(),
                blue_min: min,
                blue_max: max,
            }));
        }
        if layout.intensity {
            pc.set_intensity_limits(Some(IntensityLimits {
                intensity_min: Some(RecordValue::Integer(0)),
                intensity_max: Some(RecordValue::Integer(65_535)),
            }));
        }
        let inverse = entry.pose.inverse();
        let end = entry.start + entry.len;
        for block in (entry.start..end).step_by(BLOCK) {
            check_cancel(progress)?;
            let block_end = (block + BLOCK).min(end);
            for at in block..block_end {
                let i = order.as_ref().map_or(at, |o| o[at] as usize);
                let local = inverse * Point3::from(src.xyz(i));
                let mut values = Vec::with_capacity(prototype.len());
                values.extend(local.coords.iter().map(|&v| RecordValue::Double(v)));
                if layout.rgb {
                    values.extend(src.rgb(i).map(|c| {
                        RecordValue::Integer(if eight_bit {
                            i64::from(c / 257)
                        } else {
                            i64::from(c)
                        })
                    }));
                }
                if layout.intensity {
                    let v = (f64::from(src.intensity(i)) * 257.0).round();
                    values.push(RecordValue::Integer(v.clamp(0.0, 65_535.0) as i64));
                }
                pc.add_point(values).map_err(|e| failed(path, e))?;
            }
            progress.advance((block_end - block) as u64);
        }
        pc.finalize().map_err(|e| failed(path, e))?;
    }
    check_cancel(progress)?;
    writer.finalize().map_err(|e| failed(path, e))?;
    drop(writer);
    check_cancel(progress)?;
    temp.commit().map_err(io)
}

/// The entries to write and, with a scan column, the point indices grouped by entry.
fn plan(src: &dyn PointSource, stem: &str) -> (Vec<Entry>, Option<Vec<u32>>) {
    let n = src.len();
    let scans = src.scans();
    if !src.layout().source {
        let (name, pose) = match scans {
            [one] => (one.name.clone(), one.pose),
            _ => (stem.to_owned(), Isometry3::identity()),
        };
        let entry = Entry {
            name,
            pose,
            start: 0,
            len: n,
        };
        return (vec![entry], None);
    }

    // Scan number → entry; the listed scans first (a number listed twice: the first one).
    const NONE: u32 = u32::MAX;
    let mut slot = vec![NONE; 1 << 16];
    let mut entries = Vec::new();
    for info in scans {
        if slot[usize::from(info.source)] == NONE {
            slot[usize::from(info.source)] = entries.len() as u32;
            entries.push(Entry {
                name: info.name.clone(),
                pose: info.pose,
                start: 0,
                len: 0,
            });
        }
    }
    let mut seen = vec![false; 1 << 16];
    for i in 0..n {
        seen[usize::from(src.source(i))] = true;
    }
    for (s, _) in seen.iter().enumerate().filter(|(_, &seen)| seen) {
        if slot[s] == NONE {
            slot[s] = entries.len() as u32;
            entries.push(Entry {
                name: format!("Scan {}", s + 1),
                pose: Isometry3::identity(),
                start: 0,
                len: 0,
            });
        }
    }
    drop(seen);

    for i in 0..n {
        entries[slot[usize::from(src.source(i))] as usize].len += 1;
    }
    let mut next = Vec::with_capacity(entries.len());
    let mut start = 0;
    for entry in &mut entries {
        entry.start = start;
        next.push(start);
        start += entry.len;
    }
    let mut order = vec![0u32; n];
    for i in 0..n {
        let k = slot[usize::from(src.source(i))] as usize;
        order[next[k]] = i as u32;
        next[k] += 1;
    }
    (entries, Some(order))
}

/// FNV-1a, 64 bit.
fn fnv1a64(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
    })
}

/// A GUID `xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx` from the file name, a point count and a scan
/// number ([`FILE_GUID`] for the file): FNV-1a 64 of `name ‖ 0 ‖ count ‖ scan` (little
/// endian) for the first 64 bits, FNV-1a 64 of the same bytes followed by the first hash for
/// the last 64.
fn guid(file_name: &str, count: u64, scan: u32) -> String {
    let mut key = Vec::with_capacity(file_name.len() + 21);
    key.extend(file_name.as_bytes());
    key.push(0);
    key.extend(count.to_le_bytes());
    key.extend(scan.to_le_bytes());
    let hi = fnv1a64(&key);
    key.extend(hi.to_le_bytes());
    let lo = fnv1a64(&key);
    let s = format!("{:016x}{:016x}", hi, lo);
    format!(
        "{}-{}-{}-{}-{}",
        &s[0..8],
        &s[8..12],
        &s[12..16],
        &s[16..20],
        &s[20..32]
    )
}

/// The library writes names as CDATA: `]]>` would end it early.
fn cdata_safe(name: &str) -> String {
    name.replace("]]>", "]] >")
}

fn transform(pose: &Isometry3<f64>) -> e57::Transform {
    let q = pose.rotation.quaternion();
    let t = pose.translation.vector;
    e57::Transform {
        rotation: e57::Quaternion {
            w: q.w,
            x: q.i,
            y: q.j,
            z: q.k,
        },
        translation: e57::Translation {
            x: t.x,
            y: t.y,
            z: t.z,
        },
    }
}

/// A failed write: an I/O error as such, anything else as a format error.
fn failed(path: &Path, e: e57::Error) -> ExportError {
    match e {
        e57::Error::Write {
            desc,
            source: Some(source),
        }
        | e57::Error::Read {
            desc,
            source: Some(source),
        } => match source.downcast::<std::io::Error>() {
            Ok(io) => ExportError::io(path, *io),
            Err(source) => ExportError::format(path, format!("E57: {desc}: {source}")),
        },
        other => ExportError::format(path, format!("E57: {other}")),
    }
}

/// Reads every `data3D` entry of an E57 file into one [`Points`], one after the other, in
/// blocks of [`BLOCK`] points (phase [`PHASE_READ`], total = all records, a cancel check per
/// block).
///
/// - The pose of each entry is applied; spherical coordinates become cartesian. Points that
///   are invalid or only a direction are skipped. An entry with records but no point with a
///   position is an error: `E57: Scan „<name>“ hat keine kartesischen Punkte`.
/// - Colour and intensity (normalised to 0–1 by their limits) become `round(v × 65535)`;
///   intensity is not turned into colour. A point without one of them gets 0 when the column
///   exists for another entry.
/// - [`Points::source`] is the index of the entry, `None` for a file with one entry;
///   [`Points::scans`] has the name (else `Scan <n+1>`) and the pose of each entry.
/// - No times, no flags; images are not read (a warning through `progress`).
///
/// Errors: [`ExportError::Io`] for a file that cannot be opened, [`ExportError::Format`] for
/// anything that is not a readable E57 file – never a panic.
pub fn read_e57(path: &Path, progress: &Progress) -> Result<Points, ExportError> {
    check_cancel(progress)?;
    let file = File::open(path).map_err(|e| ExportError::io(path, e))?;
    let len = file.metadata().map_err(|e| ExportError::io(path, e))?.len();
    // The library checks what it reads; this only turns a missed case into a message.
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        read(path, file, len, progress)
    }))
    .unwrap_or_else(|_| {
        Err(ExportError::format(
            path,
            "E57-Datei ist beschädigt (unerwarteter Inhalt)",
        ))
    })
}

fn read(path: &Path, file: File, len: u64, progress: &Progress) -> Result<Points, ExportError> {
    let mut reader = E57Reader::new(BufReader::with_capacity(READ_BUFFER, file))
        .map_err(|e| ExportError::format(path, format!("keine lesbare E57-Datei ({e})")))?;
    let broken =
        |e: e57::Error| ExportError::format(path, format!("E57-Datei ist beschädigt ({e})"));
    let clouds = reader.pointclouds();
    if clouds.len() > 1 << 16 {
        return Err(ExportError::format(
            path,
            format!(
                "E57 mit {} Scans: höchstens 65536 werden gelesen",
                clouds.len()
            ),
        ));
    }
    match reader.images().len() {
        0 => {}
        1 => progress.warn("E57: das Bild in der Datei wird nicht gelesen"),
        k => progress.warn(format!(
            "E57: die {k} Bilder in der Datei werden nicht gelesen"
        )),
    }

    let total = clouds
        .iter()
        .fold(0u64, |sum, pc| sum.saturating_add(pc.records));
    // A header can claim any count: reserve no more than the file could hold.
    let reserve = total.min(len / 4) as usize;
    let mut xyz = Vec::with_capacity(reserve);
    let mut rgb = clouds
        .iter()
        .any(|pc| pc.has_color())
        .then(|| Vec::with_capacity(reserve));
    let mut intensity = clouds
        .iter()
        .any(|pc| pc.has_intensity())
        .then(|| Vec::with_capacity(reserve));
    let several = clouds.len() > 1;
    let mut source = several.then(|| Vec::with_capacity(reserve));
    let mut scans = Vec::with_capacity(clouds.len());
    progress.phase(PHASE_READ, Some(total));

    for (k, pc) in clouds.iter().enumerate() {
        check_cancel(progress)?;
        let name = pc
            .name
            .clone()
            .filter(|n| !n.trim().is_empty())
            .unwrap_or_else(|| format!("Scan {}", k + 1));
        let pose = match &pc.transform {
            None => Isometry3::identity(),
            Some(t) => pose_of(t).ok_or_else(|| {
                ExportError::format(path, format!("E57: Scan „{name}“ hat eine ungültige Lage"))
            })?,
        };
        if pc.records > 0 {
            let mut points = reader.pointcloud_simple(pc).map_err(broken)?;
            points.apply_pose(true);
            points.spherical_to_cartesian(true);
            points.intensity_to_color(false);
            let mut valid = 0u64;
            let mut pending = 0u64;
            for p in points {
                let p = p.map_err(broken)?;
                pending += 1;
                if pending == BLOCK as u64 {
                    progress.advance(pending);
                    pending = 0;
                    check_cancel(progress)?;
                }
                let CartesianCoordinate::Valid { x, y, z } = p.cartesian else {
                    continue;
                };
                if !(x.is_finite() && y.is_finite() && z.is_finite()) {
                    continue;
                }
                valid += 1;
                xyz.push([x, y, z]);
                if let Some(rgb) = &mut rgb {
                    rgb.push(
                        p.color
                            .map_or([0; 3], |c| [c.red, c.green, c.blue].map(sixteen_bit)),
                    );
                }
                if let Some(intensity) = &mut intensity {
                    intensity.push(p.intensity.map_or(0, sixteen_bit));
                }
                if let Some(source) = &mut source {
                    source.push(k as u16);
                }
            }
            progress.advance(pending);
            if valid == 0 {
                return Err(ExportError::format(
                    path,
                    format!("E57: Scan „{name}“ hat keine kartesischen Punkte"),
                ));
            }
        }
        scans.push(ScanInfo {
            source: k as u16,
            name,
            pose,
        });
    }
    Ok(Points {
        xyz,
        rgb,
        intensity,
        source,
        time: None,
        flags: None,
        scans,
    })
}

/// A value normalised to 0–1 as 16 bit.
fn sixteen_bit(v: f32) -> u16 {
    (f64::from(v) * 65_535.0).round().clamp(0.0, 65_535.0) as u16
}

/// The pose of an entry; `None` for a rotation that is no rotation (zero or not finite).
fn pose_of(t: &e57::Transform) -> Option<Isometry3<f64>> {
    let r = &t.rotation;
    let q = Quaternion::new(r.w, r.x, r.y, r.z);
    let v = [t.translation.x, t.translation.y, t.translation.z];
    let finite = q.coords.iter().chain(&v).all(|c| c.is_finite());
    if !finite || q.norm() < 1e-9 {
        return None;
    }
    Some(Isometry3::from_parts(
        Translation3::new(v[0], v[1], v[2]),
        UnitQuaternion::from_quaternion(q),
    ))
}
