//! `submaps.bin`: the submaps of a SLAM run with their odometry anchors, so that
//! `driftless slam --reuse` can solve the pose graph again without running the odometry.
//!
//! Little-endian layout:
//!
//! ```text
//! "DLSM"  u32 version  u64 input hash  f64 scan start  u64 submap count
//! per submap:  u64 id  iso anchor  iso odometry anchor
//!              u64 frame count  per frame: u64 index  f64 t  iso offset
//!                                          f64 residual_rms  f64 inlier_ratio
//!                                          f64 degeneracy  u8 icp_ok
//!              u64 point count  per point: f32 x y z
//! iso = f64 x y z  f64 qi qj qk qw
//! ```
//!
//! Every count is checked against the bytes left before anything is allocated, and the
//! loaded submaps are validated so that [`crate::solve`] can work on them without panicking.

use crate::report::scan_start;
use crate::submap::{Submap, SubmapFrame};
use driftless_odom::FrameQuality;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::fmt;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const MAGIC: &[u8; 4] = b"DLSM";
const VERSION: u32 = 1;
const ISO_BYTES: usize = 7 * 8;
const FRAME_BYTES: usize = 8 + 8 + ISO_BYTES + 3 * 8 + 1;
const POINT_BYTES: usize = 3 * 4;
/// Id, two anchors and the two counts of an empty submap.
const SUBMAP_MIN_BYTES: usize = 8 + 2 * ISO_BYTES + 8 + 8;
/// Largest deviation of a stored quaternion from unit length.
const MAX_QUAT_NORM_ERROR: f64 = 1e-6;
/// Largest absolute coordinate (m) of a position, offset or point. Far beyond any scan; a
/// larger value is damage and would only produce inf and NaN in the pose graph.
const MAX_COORD: f64 = 1e6;

const HINT: &str = "`driftless slam` ohne `--reuse` neu laufen lassen";

#[derive(Debug)]
pub enum CacheError {
    /// The file does not exist.
    Missing(PathBuf),
    Io(io::Error),
    /// Wrong magic.
    NotACache,
    WrongVersion(u32),
    Truncated,
    /// Written for another scan or other settings.
    OtherInput,
    /// Readable but inconsistent (reason in German).
    Invalid(&'static str),
}

impl fmt::Display for CacheError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CacheError::Missing(p) => write!(f, "kein Zwischenspeicher {}", p.display()),
            CacheError::Io(e) => write!(f, "Zwischenspeicher nicht lesbar: {e}"),
            CacheError::NotACache => write!(f, "Datei ist kein Driftless-Zwischenspeicher"),
            CacheError::WrongVersion(v) => {
                write!(f, "Zwischenspeicher hat Version {v}, erwartet {VERSION}")
            }
            CacheError::Truncated => write!(f, "Zwischenspeicher ist abgeschnitten"),
            CacheError::OtherInput => write!(
                f,
                "Zwischenspeicher gehört zu einem anderen Scan oder anderen Einstellungen"
            ),
            CacheError::Invalid(why) => write!(f, "Zwischenspeicher ist beschädigt ({why})"),
        }?;
        write!(f, "; {HINT}")
    }
}

impl std::error::Error for CacheError {}

/// Everything `submaps.bin` holds.
#[derive(Debug, Clone)]
pub struct CacheContents {
    /// Hash of the input id the cache was written for.
    pub input_hash: u64,
    /// [`scan_start`] of the submaps: the reference of all user-facing times.
    pub scan_start: f64,
    pub submaps: Vec<Submap>,
    pub odom_anchors: Vec<Isometry3<f64>>,
}

/// FNV-1a: a stable 64-bit hash of the input id (file names, sizes, settings).
fn input_hash(input_id: &str) -> u64 {
    input_id.bytes().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
    })
}

fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_iso(out: &mut Vec<u8>, iso: &Isometry3<f64>) {
    let t = iso.translation.vector;
    let q = iso.rotation.quaternion();
    for v in [t.x, t.y, t.z, q.i, q.j, q.k, q.w] {
        put_f64(out, v);
    }
}

/// Writes the cache for `submaps` (with their odometry anchors) of the input `input_id`.
/// The file is written next to `path` first and then renamed, so an interrupted write never
/// leaves a half cache behind.
pub fn save_cache(
    path: &Path,
    submaps: &[Submap],
    odom_anchors: &[Isometry3<f64>],
    input_id: &str,
) -> io::Result<()> {
    assert_eq!(submaps.len(), odom_anchors.len(), "one anchor per submap");
    let mut out = Vec::new();
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    put_u64(&mut out, input_hash(input_id));
    put_f64(&mut out, scan_start(submaps));
    put_u64(&mut out, submaps.len() as u64);
    for (m, odom) in submaps.iter().zip(odom_anchors) {
        put_u64(&mut out, m.id as u64);
        put_iso(&mut out, &m.anchor);
        put_iso(&mut out, odom);
        put_u64(&mut out, m.frames.len() as u64);
        for f in &m.frames {
            put_u64(&mut out, f.index as u64);
            put_f64(&mut out, f.t);
            put_iso(&mut out, &f.offset);
            put_f64(&mut out, f.quality.residual_rms);
            put_f64(&mut out, f.quality.inlier_ratio);
            put_f64(&mut out, f.quality.degeneracy);
            out.push(u8::from(f.quality.icp_ok));
        }
        put_u64(&mut out, m.points.len() as u64);
        for p in &m.points {
            for v in p {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
    }
    let tmp = path.with_extension("bin.tmp");
    fs::write(&tmp, &out)?;
    fs::rename(&tmp, path)
}

/// Reads the cache and checks that it belongs to `input_id`.
pub fn load_cache(
    path: &Path,
    input_id: &str,
) -> Result<(Vec<Submap>, Vec<Isometry3<f64>>), CacheError> {
    let c = read_cache(path)?;
    if c.input_hash != input_hash(input_id) {
        return Err(CacheError::OtherInput);
    }
    Ok((c.submaps, c.odom_anchors))
}

/// Reads and validates the cache without checking which input it belongs to (for looking
/// up times, e.g. by `driftless correct`).
pub fn read_cache(path: &Path) -> Result<CacheContents, CacheError> {
    let bytes = fs::read(path).map_err(|e| match e.kind() {
        io::ErrorKind::NotFound => CacheError::Missing(path.to_path_buf()),
        _ => CacheError::Io(e),
    })?;
    parse(&bytes)
}

struct Reader<'a> {
    rest: &'a [u8],
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], CacheError> {
        if self.rest.len() < n {
            return Err(CacheError::Truncated);
        }
        let (head, tail) = self.rest.split_at(n);
        self.rest = tail;
        Ok(head)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], CacheError> {
        let mut a = [0u8; N];
        a.copy_from_slice(self.take(N)?);
        Ok(a)
    }

    fn u64(&mut self) -> Result<u64, CacheError> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    fn f64(&mut self) -> Result<f64, CacheError> {
        Ok(f64::from_le_bytes(self.array()?))
    }

    fn finite(&mut self, what: &'static str) -> Result<f64, CacheError> {
        let v = self.f64()?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(CacheError::Invalid(what))
        }
    }

    /// A finite coordinate within [`MAX_COORD`].
    fn coord(&mut self, what: &'static str) -> Result<f64, CacheError> {
        let v = self.finite(what)?;
        if v.abs() <= MAX_COORD {
            Ok(v)
        } else {
            Err(CacheError::Invalid(what))
        }
    }

    /// A count of records of at least `record` bytes each that must fit into the rest.
    fn count(&mut self, record: usize) -> Result<usize, CacheError> {
        let n = self.u64()?;
        let n = usize::try_from(n).map_err(|_| CacheError::Truncated)?;
        match n.checked_mul(record) {
            Some(bytes) if bytes <= self.rest.len() => Ok(n),
            _ => Err(CacheError::Truncated),
        }
    }

    fn iso(&mut self) -> Result<Isometry3<f64>, CacheError> {
        const WHAT: &str = "ungültige Pose";
        let mut v = [0.0; 7];
        for (i, x) in v.iter_mut().enumerate() {
            *x = if i < 3 {
                self.coord(WHAT)?
            } else {
                self.finite(WHAT)?
            };
        }
        let q = Quaternion::new(v[6], v[3], v[4], v[5]);
        if (q.norm() - 1.0).abs() > MAX_QUAT_NORM_ERROR {
            return Err(CacheError::Invalid(WHAT));
        }
        // Stored unit quaternions are taken as they are, so the round trip is bit-exact.
        Ok(Isometry3::from_parts(
            Translation3::new(v[0], v[1], v[2]),
            UnitQuaternion::new_unchecked(q),
        ))
    }
}

fn parse(bytes: &[u8]) -> Result<CacheContents, CacheError> {
    let mut r = Reader { rest: bytes };
    if r.array::<4>()? != *MAGIC {
        return Err(CacheError::NotACache);
    }
    let version = u32::from_le_bytes(r.array()?);
    if version != VERSION {
        return Err(CacheError::WrongVersion(version));
    }
    let input_hash = r.u64()?;
    let stored_start = r.f64()?;
    let n = r.count(SUBMAP_MIN_BYTES)?;
    let mut submaps = Vec::with_capacity(n);
    let mut odom_anchors = Vec::with_capacity(n);
    for k in 0..n {
        let id = r.u64()?;
        if id != k as u64 {
            return Err(CacheError::Invalid("Teilkarten nicht in Reihenfolge"));
        }
        let anchor = r.iso()?;
        odom_anchors.push(r.iso()?);
        let nf = r.count(FRAME_BYTES)?;
        if nf == 0 {
            return Err(CacheError::Invalid("Teilkarte ohne Frames"));
        }
        let mut frames = Vec::with_capacity(nf);
        for _ in 0..nf {
            let index = usize::try_from(r.u64()?)
                .map_err(|_| CacheError::Invalid("ungültiger Frame-Index"))?;
            let t = r.finite("ungültige Zeit")?;
            let offset = r.iso()?;
            let quality = FrameQuality {
                residual_rms: r.finite("ungültige Güte")?,
                inlier_ratio: r.finite("ungültige Güte")?,
                degeneracy: r.finite("ungültige Güte")?,
                icp_ok: match r.array::<1>()?[0] {
                    0 => false,
                    1 => true,
                    _ => return Err(CacheError::Invalid("ungültige Güte")),
                },
            };
            frames.push(SubmapFrame {
                index,
                t,
                offset,
                quality,
            });
        }
        let np = r.count(POINT_BYTES)?;
        let mut points = Vec::with_capacity(np);
        for _ in 0..np {
            let p = [
                f32::from_le_bytes(r.array()?),
                f32::from_le_bytes(r.array()?),
                f32::from_le_bytes(r.array()?),
            ];
            if !p
                .iter()
                .all(|v| v.is_finite() && f64::from(v.abs()) <= MAX_COORD)
            {
                return Err(CacheError::Invalid("ungültiger Punkt"));
            }
            points.push(p);
        }
        submaps.push(Submap {
            id: k,
            anchor,
            frames,
            points,
        });
    }
    if !r.rest.is_empty() {
        return Err(CacheError::Invalid("überzählige Daten am Ende"));
    }
    if stored_start.to_bits() != scan_start(&submaps).to_bits() {
        return Err(CacheError::Invalid(
            "Scan-Beginn passt nicht zu den Teilkarten",
        ));
    }
    Ok(CacheContents {
        input_hash,
        scan_start: stored_start,
        submaps,
        odom_anchors,
    })
}
