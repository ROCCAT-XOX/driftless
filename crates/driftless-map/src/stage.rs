//! Intermediate results between the steps of `driftless map` (`dense.bin`, `clean.bin`): the
//! cloud columns the later steps need, at full precision, and optionally the [`RaySource`]
//! the cleaning needs. LAS stays the export: it rounds coordinates to [`crate::LAS_SCALE`]
//! and intensity to 16 bits and has no column for the frame index, so a cloud read back from
//! it would not give the same result as the one in memory.
//!
//! Little-endian layout:
//!
//! ```text
//! "DLMS"  u32 version  u64 input hash
//! u64 point count   per point: f32 x y z  f32 intensity  f64 t  u32 frame
//! u8 rays (0 or 1)  if 1:  u64 frame count  per frame: f32 x y z (sensor)
//!                          per frame + 1: u32 frame start
//!                          u64 ray count    per ray: f32 x y z (end)
//! ```
//!
//! Colours are not stored: the stages hold the cloud before colouring. Every count is checked
//! against the bytes left before anything is allocated; coordinates must be finite and within
//! [`MAX_COORD`], intensities and times finite, frame starts rise from 0 to the ray count.

use crate::{Cloud, RaySource};
use std::fmt;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

const MAGIC: &[u8; 4] = b"DLMS";
const VERSION: u32 = 1;
const POINT_BYTES: usize = 3 * 4 + 4 + 8 + 4;
const XYZ_BYTES: usize = 3 * 4;
/// Largest absolute coordinate (m); anything larger is damage.
pub const MAX_COORD: f32 = 1e6;
/// Records parsed per read while loading.
const CHUNK: usize = 1 << 16;

#[derive(Debug)]
pub enum StageError {
    /// The file does not exist.
    Missing(PathBuf),
    Io(io::Error),
    /// Wrong magic.
    NotAStage,
    WrongVersion(u32),
    Truncated,
    /// Written for another input or other settings.
    OtherInput,
    /// Readable but inconsistent (reason in German).
    Invalid(&'static str),
}

impl fmt::Display for StageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StageError::Missing(p) => write!(f, "{} fehlt", p.display()),
            StageError::Io(e) => write!(f, "nicht lesbar: {e}"),
            StageError::NotAStage => write!(f, "keine Driftless-Zwischenstufe"),
            StageError::WrongVersion(v) => write!(f, "Version {v}, erwartet {VERSION}"),
            StageError::Truncated => write!(f, "abgeschnitten"),
            StageError::OtherInput => {
                write!(
                    f,
                    "gehört zu einer anderen Eingabe oder anderen Einstellungen"
                )
            }
            StageError::Invalid(why) => write!(f, "beschädigt ({why})"),
        }
    }
}

impl std::error::Error for StageError {}

/// FNV-1a: a stable 64-bit hash, for input ids and the files they name.
pub fn content_hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
    })
}

fn write_xyz(w: &mut impl Write, p: &[f32; 3]) -> io::Result<()> {
    for v in p {
        w.write_all(&v.to_le_bytes())?;
    }
    Ok(())
}

fn write_body(
    w: &mut impl Write,
    input_id: &str,
    cloud: &Cloud,
    rays: Option<&RaySource>,
) -> io::Result<()> {
    w.write_all(MAGIC)?;
    w.write_all(&VERSION.to_le_bytes())?;
    w.write_all(&content_hash(input_id.as_bytes()).to_le_bytes())?;
    w.write_all(&(cloud.len() as u64).to_le_bytes())?;
    for i in 0..cloud.len() {
        write_xyz(w, &cloud.xyz[i])?;
        w.write_all(&cloud.intensity[i].to_le_bytes())?;
        w.write_all(&cloud.t[i].to_le_bytes())?;
        w.write_all(&cloud.frame[i].to_le_bytes())?;
    }
    match rays {
        None => w.write_all(&[0]),
        Some(r) => {
            w.write_all(&[1])?;
            w.write_all(&(r.sensor.len() as u64).to_le_bytes())?;
            for p in &r.sensor {
                write_xyz(w, p)?;
            }
            for s in &r.frame_start {
                w.write_all(&s.to_le_bytes())?;
            }
            w.write_all(&(r.ends.len() as u64).to_le_bytes())?;
            for p in &r.ends {
                write_xyz(w, p)?;
            }
            Ok(())
        }
    }
}

/// Writes `cloud` (without colours) and `rays` for the input `input_id`. The file is written
/// next to `path` first and then renamed, so an interrupted write never leaves half a file.
///
/// # Panics
/// If `rays` is given and does not have one frame start more than sensors.
pub fn save_stage(
    path: &Path,
    input_id: &str,
    cloud: &Cloud,
    rays: Option<&RaySource>,
) -> io::Result<()> {
    if let Some(r) = rays {
        assert_eq!(
            r.frame_start.len(),
            r.sensor.len() + 1,
            "one start per frame, plus the end"
        );
    }
    let tmp = path.with_extension("bin.tmp");
    let mut w = BufWriter::new(File::create(&tmp)?);
    write_body(&mut w, input_id, cloud, rays)?;
    w.into_inner()
        .map_err(io::IntoInnerError::into_error)?
        .sync_all()?;
    fs::rename(&tmp, path)
}

/// Streams the file, knowing how many bytes are left.
struct Reader {
    inner: BufReader<File>,
    left: u64,
    buf: Vec<u8>,
}

impl Reader {
    fn bytes<const N: usize>(&mut self) -> Result<[u8; N], StageError> {
        let mut a = [0u8; N];
        if self.left < N as u64 {
            return Err(StageError::Truncated);
        }
        self.inner.read_exact(&mut a).map_err(StageError::Io)?;
        self.left -= N as u64;
        Ok(a)
    }

    fn u64(&mut self) -> Result<u64, StageError> {
        Ok(u64::from_le_bytes(self.bytes()?))
    }

    /// A count of `record`-byte records that must fit into the bytes left.
    fn count(&mut self, record: usize) -> Result<usize, StageError> {
        let n = self.u64()?;
        match n.checked_mul(record as u64) {
            Some(b) if b <= self.left => usize::try_from(n).map_err(|_| StageError::Truncated),
            _ => Err(StageError::Truncated),
        }
    }

    /// Calls `f` with each of `n` records of `size` bytes, read in chunks.
    fn records(
        &mut self,
        n: usize,
        size: usize,
        mut f: impl FnMut(&[u8]) -> Result<(), StageError>,
    ) -> Result<(), StageError> {
        let mut done = 0;
        while done < n {
            let k = (n - done).min(CHUNK);
            self.buf.resize(k * size, 0);
            self.inner
                .read_exact(&mut self.buf)
                .map_err(StageError::Io)?;
            self.left = self.left.saturating_sub((k * size) as u64);
            for rec in self.buf.chunks_exact(size) {
                f(rec)?;
            }
            done += k;
        }
        Ok(())
    }
}

fn f32_at(b: &[u8], at: usize) -> f32 {
    f32::from_le_bytes(b[at..at + 4].try_into().expect("4 bytes"))
}

fn xyz_at(b: &[u8], at: usize, what: &'static str) -> Result<[f32; 3], StageError> {
    let p = [f32_at(b, at), f32_at(b, at + 4), f32_at(b, at + 8)];
    if p.iter().all(|v| v.is_finite() && v.abs() <= MAX_COORD) {
        Ok(p)
    } else {
        Err(StageError::Invalid(what))
    }
}

/// Reads a stage and checks that it belongs to `input_id`. Returns the cloud (without
/// colours) and the rays, if the stage has them.
pub fn load_stage(path: &Path, input_id: &str) -> Result<(Cloud, Option<RaySource>), StageError> {
    let file = File::open(path).map_err(|e| match e.kind() {
        io::ErrorKind::NotFound => StageError::Missing(path.to_path_buf()),
        _ => StageError::Io(e),
    })?;
    let left = file.metadata().map_err(StageError::Io)?.len();
    let mut r = Reader {
        inner: BufReader::new(file),
        left,
        buf: Vec::new(),
    };
    if &r.bytes::<4>()? != MAGIC {
        return Err(StageError::NotAStage);
    }
    let version = u32::from_le_bytes(r.bytes()?);
    if version != VERSION {
        return Err(StageError::WrongVersion(version));
    }
    if r.u64()? != content_hash(input_id.as_bytes()) {
        return Err(StageError::OtherInput);
    }

    let n = r.count(POINT_BYTES)?;
    let mut cloud = Cloud {
        xyz: Vec::with_capacity(n),
        intensity: Vec::with_capacity(n),
        t: Vec::with_capacity(n),
        frame: Vec::with_capacity(n),
        rgb: None,
        colored: None,
    };
    r.records(n, POINT_BYTES, |b| {
        let intensity = f32_at(b, 12);
        let t = f64::from_le_bytes(b[16..24].try_into().expect("8 bytes"));
        if !intensity.is_finite() {
            return Err(StageError::Invalid("ungültige Intensität"));
        }
        if !t.is_finite() {
            return Err(StageError::Invalid("ungültige Zeit"));
        }
        cloud.xyz.push(xyz_at(b, 0, "ungültiger Punkt")?);
        cloud.intensity.push(intensity);
        cloud.t.push(t);
        cloud
            .frame
            .push(u32::from_le_bytes(b[24..28].try_into().expect("4 bytes")));
        Ok(())
    })?;

    let rays = match r.bytes::<1>()? {
        [0] => None,
        [1] => {
            let frames = r.count(XYZ_BYTES + 4)?;
            let mut rays = RaySource {
                sensor: Vec::with_capacity(frames),
                ends: Vec::new(),
                frame_start: Vec::with_capacity(frames + 1),
            };
            r.records(frames, XYZ_BYTES, |b| {
                rays.sensor.push(xyz_at(b, 0, "ungültige Sensorlage")?);
                Ok(())
            })?;
            r.records(frames + 1, 4, |b| {
                let s = u32::from_le_bytes(b.try_into().expect("4 bytes"));
                let prev = rays.frame_start.last().copied();
                if prev.map_or(s != 0, |p| s < p) {
                    return Err(StageError::Invalid("Strahl-Anfänge steigen nicht von 0 an"));
                }
                rays.frame_start.push(s);
                Ok(())
            })?;
            let ends = r.count(XYZ_BYTES)?;
            if rays.frame_start.last().map(|&s| s as usize) != Some(ends) {
                return Err(StageError::Invalid("Strahlzahl passt nicht zu den Frames"));
            }
            rays.ends.reserve_exact(ends);
            r.records(ends, XYZ_BYTES, |b| {
                rays.ends.push(xyz_at(b, 0, "ungültiger Strahl")?);
                Ok(())
            })?;
            Some(rays)
        }
        _ => return Err(StageError::Invalid("unbekannte Kennung der Strahlen")),
    };
    if r.left != 0 {
        return Err(StageError::Invalid("Daten nach dem Ende"));
    }
    Ok((cloud, rays))
}
