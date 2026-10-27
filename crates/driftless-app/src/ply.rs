//! A reader for PLY point clouds (`format ascii 1.0`, `binary_little_endian 1.0` and
//! `binary_big_endian 1.0`) into a [`PointSet`].
//!
//! Of the element `vertex` it takes `x y z` (any scalar type, usually `float` or `double`),
//! the colour `red green blue` (`uchar`, or a float in 0–1 or 0–255), `intensity` or
//! `scalar_intensity`, `scan` (the scan of a merged model) and `colored` (0 = without
//! measured colour, as [`driftless_map::write_ply`] writes it). Other properties and other
//! elements (e.g. `face`) are skipped, elements after `vertex` are not read at all.
//!
//! A file that does not fit its header – a cut header, an unknown type, more points than the
//! file can hold – is an error, never a panic. No buffer is sized by a count of the header
//! alone: the counts of the elements read are checked against the size of the file first.

use crate::cloud::{PointSet, MID_GRAY, PHASE_LOAD};
use driftless_io::progress::Progress;
use driftless_steps::StepError;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read};
use std::path::Path;

/// Points read per batch (one cancel check and one progress step each).
const BATCH: u64 = 1 << 20;
/// Longest header (bytes).
const MAX_HEADER: u64 = 1 << 20;
/// Longest value of an ASCII file (bytes).
const MAX_TOKEN: usize = 64;
/// Characters of a header line quoted in an error.
const QUOTE: usize = 40;
/// Buffer of the file reader (bytes).
const READ_BUFFER: usize = 1 << 16;

/// Reads a PLY file in batches of 2²⁰ points (phase [`PHASE_LOAD`], total from the header,
/// a cancel check per batch). The origin is the first point in whole metres; points with a
/// coordinate that is not finite are left out. Float colours are 0–1 (×255), unless one of
/// them is above 1: then the file holds 0–255 (as some programs write it) and they are taken
/// as they are. Intensities in 0–255 are taken as they are,
/// floats in 0–1 ×255, larger ones scaled so that the brightest is 255; without any (or
/// all 0) the points are [`MID_GRAY`]. Without point times the time is 0 and the quality
/// unknown (255). A `scan` of only one value gives no [`PointSet::source`].
///
/// Errors are `StepError::Failed("PLY nicht lesbar: <file>: <reason>")`.
pub fn load_ply(path: &Path, progress: &Progress) -> Result<PointSet, StepError> {
    read(path, progress).map_err(|e| match e {
        Fail::Cancelled => StepError::Cancelled,
        Fail::Bad(m) => StepError::Failed(format!("PLY nicht lesbar: {}: {m}", path.display())),
    })
}

/// Why a read stopped.
#[derive(Debug)]
enum Fail {
    Cancelled,
    /// The reason, German.
    Bad(String),
}

/// The file ended before the elements of its header.
const SHORT: &str = "Datei endet früher, als die Anzahl im Kopf sagt";

impl From<io::Error> for Fail {
    fn from(e: io::Error) -> Self {
        match e.kind() {
            io::ErrorKind::UnexpectedEof => Fail::Bad(SHORT.into()),
            _ => Fail::Bad(e.to_string()),
        }
    }
}

fn bad<T>(m: impl Into<String>) -> Result<T, Fail> {
    Err(Fail::Bad(m.into()))
}

/// The first [`QUOTE`] characters of `s`.
fn quote(s: &str) -> String {
    s.chars().take(QUOTE).collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(name: &str) -> Option<Scalar> {
        Some(match name {
            "char" | "int8" => Scalar::I8,
            "uchar" | "uint8" => Scalar::U8,
            "short" | "int16" => Scalar::I16,
            "ushort" | "uint16" => Scalar::U16,
            "int" | "int32" => Scalar::I32,
            "uint" | "uint32" => Scalar::U32,
            "float" | "float32" => Scalar::F32,
            "double" | "float64" => Scalar::F64,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Scalar::I8 | Scalar::U8 => 1,
            Scalar::I16 | Scalar::U16 => 2,
            Scalar::I32 | Scalar::U32 | Scalar::F32 => 4,
            Scalar::F64 => 8,
        }
    }

    fn is_float(self) -> bool {
        matches!(self, Scalar::F32 | Scalar::F64)
    }

    /// The value of the first [`Scalar::size`] bytes of `b`.
    fn decode(self, b: &[u8], big: bool) -> f64 {
        macro_rules! num {
            ($t:ty, $n:expr) => {{
                let mut a = [0u8; $n];
                a.copy_from_slice(&b[..$n]);
                (if big {
                    <$t>::from_be_bytes(a)
                } else {
                    <$t>::from_le_bytes(a)
                }) as f64
            }};
        }
        match self {
            Scalar::I8 => f64::from(b[0] as i8),
            Scalar::U8 => f64::from(b[0]),
            Scalar::I16 => num!(i16, 2),
            Scalar::U16 => num!(u16, 2),
            Scalar::I32 => num!(i32, 4),
            Scalar::U32 => num!(u32, 4),
            Scalar::F32 => num!(f32, 4),
            Scalar::F64 => num!(f64, 8),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Scalar(Scalar),
    /// A count, then that many items.
    List {
        count: Scalar,
        item: Scalar,
    },
}

#[derive(Debug, Clone)]
struct Property {
    name: String,
    kind: Kind,
}

#[derive(Debug, Clone)]
struct Element {
    name: String,
    count: u64,
    props: Vec<Property>,
}

impl Element {
    /// Bytes of a binary row without lists; `None` with a list.
    fn stride(&self) -> Option<usize> {
        self.props
            .iter()
            .map(|p| match p.kind {
                Kind::Scalar(s) => Some(s.size()),
                Kind::List { .. } => None,
            })
            .sum()
    }

    /// The fewest bytes a binary row can have (an empty list is its count).
    fn min_binary_row(&self) -> u128 {
        self.props
            .iter()
            .map(|p| match p.kind {
                Kind::Scalar(s) | Kind::List { count: s, .. } => s.size() as u128,
            })
            .sum()
    }

    /// The first scalar property `name`, in any case (CloudCompare writes
    /// `scalar_Intensity`).
    fn scalar(&self, name: &str) -> Option<(usize, Scalar)> {
        self.props
            .iter()
            .enumerate()
            .find_map(|(i, p)| match p.kind {
                Kind::Scalar(s) if p.name.eq_ignore_ascii_case(name) => Some((i, s)),
                _ => None,
            })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Format {
    Ascii,
    Binary { big: bool },
}

#[derive(Debug)]
struct Header {
    format: Format,
    elements: Vec<Element>,
    /// Bytes up to and with the line `end_header`.
    len: u64,
}

/// Reads the header up to its line `end_header` (at most [`MAX_HEADER`] bytes); `r` then
/// stands at the first byte of the data.
fn parse_header(r: &mut impl BufRead) -> Result<Header, Fail> {
    let mut line = Vec::new();
    let mut len = 0u64;
    let mut format = None;
    let mut elements: Vec<Element> = Vec::new();
    let mut first = true;
    loop {
        line.clear();
        let n = r
            .by_ref()
            .take(MAX_HEADER - len + 1)
            .read_until(b'\n', &mut line)?;
        len += n as u64;
        if first {
            first = false;
            if line.trim_ascii() != b"ply" {
                return bad("keine PLY-Datei");
            }
            continue;
        }
        if len > MAX_HEADER {
            return bad("Kopf länger als 1 MiB");
        }
        if line.last() != Some(&b'\n') {
            return bad("Kopf ohne „end_header“");
        }
        let Ok(text) = std::str::from_utf8(&line) else {
            return bad("Kopf ist kein Text");
        };
        let words: Vec<&str> = text.split_ascii_whitespace().collect();
        let invalid = || bad(format!("Kopfzeile „{}“ ungültig", quote(text.trim())));
        match words.as_slice() {
            [] => {}
            ["comment" | "obj_info", ..] => {}
            ["format", f, version] => {
                format = Some(match *f {
                    "ascii" => Format::Ascii,
                    "binary_little_endian" => Format::Binary { big: false },
                    "binary_big_endian" => Format::Binary { big: true },
                    _ => return bad(format!("Format „{}“ unbekannt", quote(f))),
                });
                if *version != "1.0" {
                    return bad(format!("Version „{}“ nicht unterstützt", quote(version)));
                }
            }
            ["element", name, count] => {
                let Ok(count) = count.parse::<u64>() else {
                    return bad(format!("Anzahl „{}“ ungültig", quote(count)));
                };
                elements.push(Element {
                    name: (*name).to_string(),
                    count,
                    props: Vec::new(),
                });
            }
            ["property", ty, name] => {
                let s = scalar_type(ty)?;
                push_property(&mut elements, name, Kind::Scalar(s))?;
            }
            ["property", "list", count, item, name] => {
                let count = scalar_type(count)?;
                if count.is_float() {
                    return invalid();
                }
                let item = scalar_type(item)?;
                push_property(&mut elements, name, Kind::List { count, item })?;
            }
            ["end_header"] => break,
            _ => return invalid(),
        }
    }
    let Some(format) = format else {
        return bad("Kopf ohne Format");
    };
    Ok(Header {
        format,
        elements,
        len,
    })
}

fn scalar_type(name: &str) -> Result<Scalar, Fail> {
    Scalar::parse(name).ok_or_else(|| Fail::Bad(format!("Typ „{}“ unbekannt", quote(name))))
}

fn push_property(elements: &mut [Element], name: &str, kind: Kind) -> Result<(), Fail> {
    let Some(e) = elements.last_mut() else {
        return bad("Eigenschaft vor dem ersten Element");
    };
    e.props.push(Property {
        name: name.to_string(),
        kind,
    });
    Ok(())
}

/// Reads the rows of the elements, binary or ASCII.
enum Rows {
    Binary {
        r: BufReader<File>,
        big: bool,
        /// Bytes read or skipped since the start of the file.
        pos: u64,
        /// Size of the file.
        len: u64,
        buf: Vec<u8>,
    },
    Ascii {
        r: BufReader<File>,
        token: Vec<u8>,
    },
}

impl Rows {
    /// The next row of `e`: the value of each scalar property in `out` (one entry per
    /// property; lists are skipped and leave theirs).
    fn row(&mut self, e: &Element, stride: Option<usize>, out: &mut [f64]) -> Result<(), Fail> {
        match self {
            Rows::Binary {
                r,
                big,
                pos,
                len,
                buf,
            } => {
                let big = *big;
                if let Some(stride) = stride {
                    buf.resize(stride, 0);
                    r.read_exact(buf)?;
                    *pos += stride as u64;
                    let mut at = 0;
                    for (p, o) in e.props.iter().zip(out.iter_mut()) {
                        if let Kind::Scalar(s) = p.kind {
                            *o = s.decode(&buf[at..], big);
                            at += s.size();
                        }
                    }
                    return Ok(());
                }
                let mut b = [0u8; 8];
                for (p, o) in e.props.iter().zip(out.iter_mut()) {
                    match p.kind {
                        Kind::Scalar(s) => {
                            r.read_exact(&mut b[..s.size()])?;
                            *pos += s.size() as u64;
                            *o = s.decode(&b, big);
                        }
                        Kind::List { count, item } => {
                            r.read_exact(&mut b[..count.size()])?;
                            *pos += count.size() as u64;
                            let n = list_len(count.decode(&b, big))?;
                            let skip = n * item.size() as u64;
                            *pos += skip;
                            if *pos > *len {
                                return bad(SHORT);
                            }
                            // within the file, so within i64
                            r.seek_relative(skip as i64)?;
                        }
                    }
                }
                Ok(())
            }
            Rows::Ascii { r, token } => {
                for (p, o) in e.props.iter().zip(out.iter_mut()) {
                    let v = number(r, token)?;
                    match p.kind {
                        Kind::Scalar(_) => *o = v,
                        Kind::List { .. } => {
                            for _ in 0..list_len(v)? {
                                next_token(r, token)?;
                            }
                        }
                    }
                }
                Ok(())
            }
        }
    }

    /// Skips the `e.count` rows of an element that is not read.
    fn skip(&mut self, e: &Element, progress: &Progress) -> Result<(), Fail> {
        if e.props.is_empty() {
            return Ok(());
        }
        let stride = e.stride();
        if let (Rows::Binary { r, pos, len, .. }, Some(stride)) = (&mut *self, stride) {
            // checked against the file size before
            let skip = e.count * stride as u64;
            *pos += skip;
            if *pos > *len {
                return bad(SHORT);
            }
            r.seek_relative(skip as i64)?;
            return Ok(());
        }
        let mut out = vec![0.0; e.props.len()];
        for i in 0..e.count {
            if i % BATCH == 0 && progress.is_cancelled() {
                return Err(Fail::Cancelled);
            }
            self.row(e, stride, &mut out)?;
        }
        Ok(())
    }
}

/// A list length: a whole number ≥ 0.
fn list_len(v: f64) -> Result<u64, Fail> {
    if v >= 0.0 && v.fract() == 0.0 && v < u32::MAX as f64 * 2.0 {
        Ok(v as u64)
    } else {
        bad(format!("Listenlänge „{v}“ ungültig"))
    }
}

/// The next whitespace-separated value of an ASCII file, in `token`.
fn next_token(r: &mut impl BufRead, token: &mut Vec<u8>) -> Result<(), Fail> {
    token.clear();
    loop {
        let buf = r.fill_buf()?;
        if buf.is_empty() {
            break;
        }
        let mut used = 0;
        let mut done = false;
        for &b in buf {
            used += 1;
            if b.is_ascii_whitespace() {
                if !token.is_empty() {
                    done = true;
                    break;
                }
            } else if token.len() == MAX_TOKEN {
                return bad("Wert länger als 64 Zeichen");
            } else {
                token.push(b);
            }
        }
        r.consume(used);
        if done {
            break;
        }
    }
    if token.is_empty() {
        bad(SHORT)
    } else {
        Ok(())
    }
}

fn number(r: &mut impl BufRead, token: &mut Vec<u8>) -> Result<f64, Fail> {
    next_token(r, token)?;
    let text = String::from_utf8_lossy(token);
    text.parse::<f64>()
        .or_else(|_| bad(format!("Wert „{}“ ist keine Zahl", quote(&text))))
}

/// A colour channel of type `s` as 0–255: floats are 0–1 (×`float_scale`: 255, or 1 for a
/// file of float colours in 0–255), `ushort` 16 bit, other types 0–255 (clamped).
fn channel(v: f64, s: Scalar, float_scale: f64) -> u8 {
    let v = match s {
        _ if s.is_float() => (v * float_scale).round(),
        // the high byte, as LAS colours are read
        Scalar::U16 => (v / 256.0).floor(),
        _ => v,
    };
    // NaN becomes 0
    v.clamp(0.0, 255.0) as u8
}

/// The intensities as 0–255: float ones up to 1 ×255, up to 255 as they are, larger ones
/// scaled so that the brightest is 255; none or all 0 is [`MID_GRAY`].
fn intensities(raw: &[f32], float: bool) -> Vec<u8> {
    let max = raw
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0f32, |a, &b| a.max(b));
    if max <= 0.0 {
        return vec![MID_GRAY; raw.len()];
    }
    let scale = if float && max <= 1.0 {
        255.0
    } else if max <= 255.0 {
        1.0
    } else {
        255.0 / max
    };
    raw.iter()
        .map(|&v| {
            if v.is_finite() {
                (v * scale).round().clamp(0.0, 255.0) as u8
            } else {
                0
            }
        })
        .collect()
}

fn read(path: &Path, progress: &Progress) -> Result<PointSet, Fail> {
    let file = File::open(path)?;
    let file_len = file.metadata()?.len();
    let mut r = BufReader::with_capacity(READ_BUFFER, file);
    let header = parse_header(&mut r)?;
    let Some(vi) = header.elements.iter().position(|e| e.name == "vertex") else {
        return bad("keine Punkte („element vertex“ fehlt)");
    };
    let vertex = &header.elements[vi];
    let (Some(x), Some(y), Some(z)) = (vertex.scalar("x"), vertex.scalar("y"), vertex.scalar("z"))
    else {
        return bad("Punkte ohne x, y oder z");
    };
    let color = match (
        vertex.scalar("red"),
        vertex.scalar("green"),
        vertex.scalar("blue"),
    ) {
        (Some(r), Some(g), Some(b)) => Some([r, g, b]),
        _ => None,
    };
    let intensity = vertex
        .scalar("intensity")
        .or_else(|| vertex.scalar("scalar_intensity"));
    let scan = vertex.scalar("scan");
    let colored = vertex.scalar("colored");

    // The elements up to the vertices must fit into the rest of the file.
    let rest = u128::from(file_len.saturating_sub(header.len));
    let elements = &header.elements[..=vi];
    let need: u128 = match header.format {
        Format::Binary { .. } => elements
            .iter()
            .map(|e| u128::from(e.count) * e.min_binary_row())
            .sum(),
        // a character per value and a separator between two values
        Format::Ascii => elements
            .iter()
            .map(|e| u128::from(e.count) * 2 * e.props.len() as u128)
            .sum::<u128>()
            .saturating_sub(1),
    };
    if need > rest {
        return bad("Anzahl der Punkte passt nicht zur Dateigröße");
    }

    let mut rows = match header.format {
        Format::Binary { big } => Rows::Binary {
            r,
            big,
            pos: header.len,
            len: file_len,
            buf: Vec::new(),
        },
        Format::Ascii => Rows::Ascii {
            r,
            token: Vec::with_capacity(MAX_TOKEN),
        },
    };
    for e in &header.elements[..vi] {
        rows.skip(e, progress)?;
    }

    let total = vertex.count;
    // bounded by the file size above
    let capacity = total as usize;
    let mut out = PointSet {
        origin: [0.0; 3],
        xyz: Vec::with_capacity(capacity),
        rgb: color.map(|_| Vec::with_capacity(capacity)),
        intensity: Vec::new(),
        t: Vec::new(),
        quality: Vec::new(),
        quality_known: false,
        flags: Vec::with_capacity(capacity),
        source: Vec::with_capacity(if scan.is_some() { capacity } else { 0 }),
    };
    let mut raw_intensity: Vec<f32> =
        Vec::with_capacity(if intensity.is_some() { capacity } else { 0 });
    // Float colours are kept as read until the range is known (0–1 or 0–255).
    let float_color = color.filter(|c| c.iter().any(|(_, s)| s.is_float()));
    let mut raw_rgb: Vec<[f32; 3]> =
        Vec::with_capacity(if float_color.is_some() { capacity } else { 0 });
    let mut origin = None;
    let stride = vertex.stride();
    let mut row = vec![0.0f64; vertex.props.len()];
    progress.phase(PHASE_LOAD, Some(total));
    let mut done = 0u64;
    loop {
        if progress.is_cancelled() {
            return Err(Fail::Cancelled);
        }
        if done == total {
            break;
        }
        let n = BATCH.min(total - done);
        for _ in 0..n {
            rows.row(vertex, stride, &mut row)?;
            let p = [row[x.0], row[y.0], row[z.0]];
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let o = *origin.get_or_insert_with(|| p.map(f64::round));
            out.xyz.push([0, 1, 2].map(|k| (p[k] - o[k]) as f32));
            if let Some(c) = float_color {
                raw_rgb.push(c.map(|(i, _)| row[i] as f32));
            } else if let (Some(rgb), Some(c)) = (&mut out.rgb, color) {
                rgb.push(c.map(|(i, s)| channel(row[i], s, 255.0)));
            }
            if let Some((i, _)) = intensity {
                raw_intensity.push(row[i] as f32);
            }
            if let Some((i, _)) = scan {
                out.source
                    .push(row[i].clamp(0.0, f64::from(u16::MAX)) as u16);
            }
            out.flags
                .push(u8::from(colored.is_some_and(|(i, _)| row[i] == 0.0)));
        }
        done += n;
        progress.advance(n);
    }
    let n = out.xyz.len();
    out.origin = origin.unwrap_or([0.0; 3]);
    if let (Some(rgb), Some(c)) = (&mut out.rgb, float_color) {
        // a float colour above 1 anywhere: the file holds 0–255
        let above_one = raw_rgb
            .iter()
            .any(|v| v.iter().zip(&c).any(|(x, (_, s))| s.is_float() && *x > 1.0));
        let scale = if above_one { 1.0 } else { 255.0 };
        rgb.extend(
            raw_rgb
                .iter()
                .map(|v| std::array::from_fn(|k| channel(f64::from(v[k]), c[k].1, scale))),
        );
    }
    out.intensity = match intensity {
        Some((_, s)) => intensities(&raw_intensity, s.is_float()),
        None => vec![MID_GRAY; n],
    };
    out.t = vec![0.0; n];
    out.quality = vec![255; n];
    if out.source.windows(2).all(|w| w[0] == w[1]) {
        out.source = Vec::new();
    }
    Ok(out)
}
