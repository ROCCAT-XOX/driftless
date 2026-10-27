//! PLY point clouds: a writer (binary little-endian, the layout of `color.ply` and
//! `modell.ply`) and a reader for `format ascii 1.0`, `binary_little_endian 1.0` and
//! `binary_big_endian 1.0`.
//!
//! Of the element `vertex` the reader takes `x y z` (any scalar type, usually `float` or
//! `double`), the colour `red green blue` (`uchar`, `ushort`, or a float in 0–1 or 0–255),
//! `intensity` or `scalar_intensity`, `scan` (the scan of a merged model) and `colored`
//! (0 = without measured colour, as [`write_ply`] writes it). Other properties and other
//! elements (e.g. `face`) are skipped, elements after `vertex` are not read at all.
//!
//! A file that does not fit its header – a cut header, an unknown type, more points than the
//! file can hold – is an error, never a panic. No buffer is sized by a count of the header
//! alone: the counts of the elements read are checked against the size of the file first.

use crate::{
    check_cancel, needs_double, ExportError, PointSource, Points, BLOCK, PHASE_READ, PHASE_WRITE,
};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

/// Longest header (bytes).
const MAX_HEADER: u64 = 1 << 20;
/// Longest value of an ASCII file (bytes).
pub(crate) const MAX_TOKEN: usize = 64;
/// Characters of a header line quoted in an error.
const QUOTE: usize = 40;
/// Buffer of the file reader (bytes).
pub(crate) const READ_BUFFER: usize = 1 << 16;

/// Writes `src` as binary little-endian PLY through `<name>.tmp` next to `path`. Progress:
/// phase [`PHASE_WRITE`] counting points, a cancel check per [`BLOCK`]; an error or a cancel
/// leaves `path` as it was and no `.tmp`.
///
/// Properties of `vertex`, in this order and only those of the layout:
/// - `x y z`: `float`, or `double` once a coordinate reaches [`crate::MAX_FLOAT_COORD`];
/// - `intensity`: `float` 0–255;
/// - `red green blue`: `uchar`, `round(v / 257)`;
/// - `colored`: `uchar`, 1 where the flags are 0 (measured colour), else 0;
/// - `scan`: `ushort`, the scan number.
pub fn write_ply(
    path: &Path,
    src: &dyn PointSource,
    progress: &Progress,
) -> Result<(), ExportError> {
    check_cancel(progress)?;
    let layout = src.layout();
    let double = needs_double(src);
    let coord = if double { "double" } else { "float" };
    let mut header = format!(
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\n\
         property {coord} x\nproperty {coord} y\nproperty {coord} z\n",
        src.len()
    );
    if layout.intensity {
        header += "property float intensity\n";
    }
    if layout.rgb {
        header += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    if layout.flags {
        header += "property uchar colored\n";
    }
    if layout.source {
        header += "property ushort scan\n";
    }
    header += "end_header\n";

    let temp = TempPath::new(path);
    let io = |e| ExportError::io(path, e);
    let file = File::create(temp.tmp()).map_err(io)?;
    let mut w = BufWriter::new(file);
    w.write_all(header.as_bytes()).map_err(io)?;
    let n = src.len();
    progress.phase(PHASE_WRITE, Some(n as u64));
    // 3 × 8 (xyz) + 4 + 3 + 1 + 2
    let mut row = Vec::with_capacity(34);
    for start in (0..n).step_by(BLOCK) {
        check_cancel(progress)?;
        let end = (start + BLOCK).min(n);
        for i in start..end {
            row.clear();
            for v in src.xyz(i) {
                if double {
                    row.extend(v.to_le_bytes());
                } else {
                    row.extend((v as f32).to_le_bytes());
                }
            }
            if layout.intensity {
                row.extend(src.intensity(i).to_le_bytes());
            }
            if layout.rgb {
                row.extend(src.rgb(i).map(to_8bit));
            }
            if layout.flags {
                row.push(u8::from(src.flags(i) == 0));
            }
            if layout.source {
                row.extend(src.source(i).to_le_bytes());
            }
            w.write_all(&row).map_err(io)?;
        }
        progress.advance((end - start) as u64);
    }
    check_cancel(progress)?;
    let file = w.into_inner().map_err(|e| io(e.into_error()))?;
    drop(file);
    temp.commit().map_err(io)
}

/// A 16-bit colour channel as 8 bit: `round(v / 257)`.
pub(crate) fn to_8bit(v: u16) -> u8 {
    // (v + 128) / 257 rounds; at most 65663 / 257 = 255
    ((u32::from(v) + 128) / 257) as u8
}

/// Reads a PLY file into [`Points`] in blocks of [`BLOCK`] points (phase [`PHASE_READ`],
/// total from the header, a cancel check per block). Points with a coordinate that is not
/// finite are left out.
///
/// - Colour: `uchar` (and other integers) 0–255 ×257, `ushort` as 16 bit; float colours are
///   0–1 (×255), unless one of them is above 1: then the file holds 0–255 (as some programs
///   write it) and they are taken as they are; then ×257.
/// - Intensity: 0–255 is taken as it is, floats in 0–1 ×255, larger ones are scaled so that
///   the brightest is 255; then ×257.
/// - `scan` becomes [`Points::source`], `colored` = 0 becomes flag 1 (no measured colour).
/// - No times, no scans.
///
/// Errors: [`ExportError::Format`] with `PLY nicht lesbar: <reason>`, [`ExportError::Io`]
/// for a file that cannot be opened.
pub fn read_ply(path: &Path, progress: &Progress) -> Result<Points, ExportError> {
    check_cancel(progress)?;
    read(path, progress).map_err(|e| match e {
        Fail::Cancelled => ExportError::Cancelled,
        Fail::Io(e) => ExportError::io(path, e),
        Fail::Bad(m) => ExportError::format(path, format!("PLY nicht lesbar: {m}")),
    })
}

/// Why a read stopped.
#[derive(Debug)]
pub(crate) enum Fail {
    Cancelled,
    /// The file could not be read.
    Io(io::Error),
    /// The reason, German.
    Bad(String),
}

/// The file ended before the elements of its header.
pub(crate) const SHORT: &str = "Datei endet früher, als die Anzahl im Kopf sagt";

impl From<io::Error> for Fail {
    fn from(e: io::Error) -> Self {
        match e.kind() {
            io::ErrorKind::UnexpectedEof => Fail::Bad(SHORT.into()),
            _ => Fail::Io(e),
        }
    }
}

pub(crate) fn bad<T>(m: impl Into<String>) -> Result<T, Fail> {
    Err(Fail::Bad(m.into()))
}

/// The first [`QUOTE`] characters of `s`.
pub(crate) fn quote(s: &str) -> String {
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
            if i % BLOCK as u64 == 0 && progress.is_cancelled() {
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
pub(crate) fn next_token(r: &mut impl BufRead, token: &mut Vec<u8>) -> Result<(), Fail> {
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

/// The next value of an ASCII file as a number.
pub(crate) fn number(r: &mut impl BufRead, token: &mut Vec<u8>) -> Result<f64, Fail> {
    next_token(r, token)?;
    let text = String::from_utf8_lossy(token);
    text.parse::<f64>()
        .or_else(|_| bad(format!("Wert „{}“ ist keine Zahl", quote(&text))))
}

/// A colour channel of type `s` as 16 bit: floats are 0–1 (×`float_scale`: 255, or 1 for a
/// file of float colours in 0–255), `ushort` 16 bit, other types 0–255 (clamped); 8-bit
/// values are widened ×257.
fn channel(v: f64, s: Scalar, float_scale: f64) -> u16 {
    // NaN becomes 0
    let eight = |v: f64| (v.clamp(0.0, 255.0) as u16) * 257;
    match s {
        _ if s.is_float() => eight((v * float_scale).round()),
        Scalar::U16 => v.clamp(0.0, 65535.0) as u16,
        _ => eight(v),
    }
}

/// The intensities as 16 bit: float ones up to 1 ×255, up to 255 as they are, larger ones
/// scaled so that the brightest is 255, then ×257 and rounded (so Driftless' own float
/// 0–255, `v / 257`, comes back exactly); all 0 (or none finite above 0) stays 0.
pub(crate) fn intensities(raw: &[f32], float: bool) -> Vec<u16> {
    let max = raw
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0f32, |a, &b| a.max(b));
    if max <= 0.0 {
        return vec![0; raw.len()];
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
                (f64::from(v) * f64::from(scale) * 257.0)
                    .round()
                    .clamp(0.0, 65535.0) as u16
            } else {
                0
            }
        })
        .collect()
}

fn read(path: &Path, progress: &Progress) -> Result<Points, Fail> {
    let file = File::open(path).map_err(Fail::Io)?;
    let file_len = file.metadata().map_err(Fail::Io)?.len();
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
    let with = |on: bool| if on { capacity } else { 0 };
    // Float colours are kept as read until the range is known (0–1 or 0–255).
    let float_color = color.filter(|c| c.iter().any(|(_, s)| s.is_float()));
    let mut rgb: Vec<[u16; 3]> = Vec::with_capacity(with(color.is_some() && float_color.is_none()));
    let mut raw_rgb: Vec<[f32; 3]> = Vec::with_capacity(with(float_color.is_some()));
    let mut raw_intensity: Vec<f32> = Vec::with_capacity(with(intensity.is_some()));
    let mut out = Points {
        xyz: Vec::with_capacity(capacity),
        source: scan.map(|_| Vec::with_capacity(capacity)),
        flags: colored.map(|_| Vec::with_capacity(capacity)),
        ..Points::default()
    };
    let stride = vertex.stride();
    let mut row = vec![0.0f64; vertex.props.len()];
    progress.phase(PHASE_READ, Some(total));
    let mut done = 0u64;
    loop {
        if progress.is_cancelled() {
            return Err(Fail::Cancelled);
        }
        if done == total {
            break;
        }
        let n = (BLOCK as u64).min(total - done);
        for _ in 0..n {
            rows.row(vertex, stride, &mut row)?;
            let p = [row[x.0], row[y.0], row[z.0]];
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            out.xyz.push(p);
            if let Some(c) = float_color {
                raw_rgb.push(c.map(|(i, _)| row[i] as f32));
            } else if let Some(c) = color {
                rgb.push(c.map(|(i, s)| channel(row[i], s, 255.0)));
            }
            if let Some((i, _)) = intensity {
                raw_intensity.push(row[i] as f32);
            }
            if let (Some(source), Some((i, _))) = (&mut out.source, scan) {
                source.push(row[i].clamp(0.0, f64::from(u16::MAX)) as u16);
            }
            if let (Some(flags), Some((i, _))) = (&mut out.flags, colored) {
                flags.push(u8::from(row[i] == 0.0));
            }
        }
        done += n;
        progress.advance(n);
    }
    if let Some(c) = float_color {
        // a float colour above 1 anywhere: the file holds 0–255
        let above_one = raw_rgb
            .iter()
            .any(|v| v.iter().zip(&c).any(|(x, (_, s))| s.is_float() && *x > 1.0));
        let scale = if above_one { 1.0 } else { 255.0 };
        rgb = raw_rgb
            .iter()
            .map(|v| std::array::from_fn(|k| channel(f64::from(v[k]), c[k].1, scale)))
            .collect();
    }
    out.rgb = color.map(|_| rgb);
    out.intensity = intensity.map(|(_, s)| intensities(&raw_intensity, s.is_float()));
    Ok(out)
}
