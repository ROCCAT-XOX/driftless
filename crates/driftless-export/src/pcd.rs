//! PCD point clouds (the format of PCL): a writer with a fixed header (`DATA binary`) and a
//! reader for `DATA ascii` and `DATA binary`.
//!
//! The reader takes the fields `x y z` (F 4 or F 8), the colour packed in `rgb`/`rgba` (4
//! bytes `0x00RRGGBB`/`0xAARRGGBB`, stored as F or U as PCL does) or in three fields `r g b`,
//! `intensity` (any type) and `scan`; other fields are skipped, also those with `COUNT` > 1.
//! Points with a coordinate that is not finite (the gaps of an organised cloud) are left out.
//! `binary_compressed` is refused with a hint how to convert the file.
//!
//! As for PLY, a file that does not fit its header is an error, never a panic, and no buffer
//! is sized by the header alone.

use crate::ply::{
    bad, intensities, next_token, number, quote, to_8bit, Fail, MAX_TOKEN, READ_BUFFER,
};
use crate::{
    check_cancel, needs_double, ExportError, PointSource, Points, BLOCK, PHASE_READ, PHASE_WRITE,
};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

/// Longest header (bytes).
const MAX_HEADER: u64 = 1 << 20;
/// Longest binary row of a point (bytes).
const MAX_ROW: usize = 1 << 20;

/// The message for `DATA binary_compressed` (without the usual lead).
const COMPRESSED: &str = "PCD „binary_compressed“ wird nicht unterstützt – in CloudCompare oder \
                          PCL als „binary“ speichern";

/// Writes `src` as binary PCD 0.7 through `<name>.tmp` next to `path`. Progress: phase
/// [`PHASE_WRITE`] counting points, a cancel check per [`BLOCK`]; an error or a cancel leaves
/// `path` as it was and no `.tmp`.
///
/// Fields, in this order and only those of the layout:
/// - `x y z`: F 4, or F 8 once a coordinate reaches [`crate::MAX_FLOAT_COORD`];
/// - `rgb`: F 4 with the bits `0x00RRGGBB` (as PCL packs it), each `round(v / 257)`;
/// - `intensity`: F 4, 0–255;
/// - `scan`: U 2, the scan number.
///
/// The header is always the same for the same layout and count: `WIDTH n`, `HEIGHT 1`,
/// `VIEWPOINT 0 0 0 1 0 0 0`, `POINTS n`, `DATA binary`. Times and flags are not stored.
pub fn write_pcd(
    path: &Path,
    src: &dyn PointSource,
    progress: &Progress,
) -> Result<(), ExportError> {
    check_cancel(progress)?;
    let layout = src.layout();
    let double = needs_double(src);
    let n = src.len();
    let coord = if double { "8" } else { "4" };
    let mut fields = vec![("x", coord, "F"), ("y", coord, "F"), ("z", coord, "F")];
    if layout.rgb {
        fields.push(("rgb", "4", "F"));
    }
    if layout.intensity {
        fields.push(("intensity", "4", "F"));
    }
    if layout.source {
        fields.push(("scan", "2", "U"));
    }
    let join = |pick: usize| {
        let words: Vec<&str> = fields.iter().map(|f| [f.0, f.1, f.2][pick]).collect();
        words.join(" ")
    };
    let header = format!(
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {}\nSIZE {}\n\
         TYPE {}\nCOUNT {}\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\n\
         DATA binary\n",
        join(0),
        join(1),
        join(2),
        vec!["1"; fields.len()].join(" "),
    );

    let temp = TempPath::new(path);
    let io = |e| ExportError::io(path, e);
    let file = File::create(temp.tmp()).map_err(io)?;
    let mut w = BufWriter::new(file);
    w.write_all(header.as_bytes()).map_err(io)?;
    progress.phase(PHASE_WRITE, Some(n as u64));
    // 3 × 8 (xyz) + 4 + 4 + 2
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
            if layout.rgb {
                let [r, g, b] = src.rgb(i).map(|v| u32::from(to_8bit(v)));
                row.extend((r << 16 | g << 8 | b).to_le_bytes());
            }
            if layout.intensity {
                row.extend(src.intensity(i).to_le_bytes());
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

/// Reads a PCD file into [`Points`] in blocks of [`BLOCK`] points (phase [`PHASE_READ`],
/// total = `POINTS`, a cancel check per block).
///
/// - Colour: 8 bit ×257; packed `rgb`/`rgba` from its 4 bytes, `r g b` clamped to 0–255.
/// - Intensity as for PLY: 0–255 is taken as it is, floats in 0–1 ×255, larger ones scaled
///   so that the brightest is 255; then ×257.
/// - `scan` becomes [`Points::source`]. No times, no flags, no scans.
///
/// Errors: [`ExportError::Format`] with `PCD nicht lesbar: <reason>` (or the hint for
/// `binary_compressed`), [`ExportError::Io`] for a file that cannot be opened.
pub fn read_pcd(path: &Path, progress: &Progress) -> Result<Points, ExportError> {
    check_cancel(progress)?;
    read(path, progress).map_err(|e| match e {
        Fail::Cancelled => ExportError::Cancelled,
        Fail::Io(e) => ExportError::io(path, e),
        Fail::Bad(m) if m == COMPRESSED => ExportError::format(path, m),
        Fail::Bad(m) => ExportError::format(path, format!("PCD nicht lesbar: {m}")),
    })
}

/// The type of a field: `F`, `I` or `U`, with its size in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ty {
    F,
    I,
    U,
}

#[derive(Debug, Clone)]
struct Field {
    name: String,
    ty: Ty,
    size: usize,
    count: usize,
    /// Bytes before it in a binary row.
    offset: usize,
}

impl Field {
    /// The value of a binary row `b` (little-endian), of its first item.
    fn decode(&self, b: &[u8]) -> f64 {
        let b = &b[self.offset..self.offset + self.size];
        macro_rules! num {
            ($t:ty, $n:expr) => {{
                let mut a = [0u8; $n];
                a.copy_from_slice(&b[..$n]);
                <$t>::from_le_bytes(a) as f64
            }};
        }
        match (self.ty, self.size) {
            (Ty::I, 1) => f64::from(b[0] as i8),
            (Ty::U, 1) => f64::from(b[0]),
            (Ty::I, 2) => num!(i16, 2),
            (Ty::U, 2) => num!(u16, 2),
            (Ty::I, 4) => num!(i32, 4),
            (Ty::U, 4) => num!(u32, 4),
            (Ty::F, 4) => num!(f32, 4),
            (Ty::I, _) => num!(i64, 8),
            (Ty::U, _) => num!(u64, 8),
            (Ty::F, _) => num!(f64, 8),
        }
    }

    /// The 4 bytes of a packed colour in a binary row.
    fn bits(&self, b: &[u8]) -> u32 {
        let mut a = [0u8; 4];
        a.copy_from_slice(&b[self.offset..self.offset + 4]);
        u32::from_le_bytes(a)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Data {
    Ascii,
    Binary,
}

#[derive(Debug)]
struct Header {
    fields: Vec<Field>,
    points: u64,
    data: Data,
    /// Bytes up to and with the line `DATA …`.
    len: u64,
}

/// The numbers of a header line, or an error naming the line.
fn numbers<T: std::str::FromStr>(words: &[&str], line: &str) -> Result<Vec<T>, Fail> {
    words
        .iter()
        .map(|w| w.parse::<T>())
        .collect::<Result<Vec<T>, _>>()
        .or_else(|_| bad(format!("Kopfzeile „{}“ ungültig", quote(line))))
}

/// Reads the header up to its line `DATA …` (at most [`MAX_HEADER`] bytes); `r` then stands
/// at the first byte of the data.
fn parse_header(r: &mut impl BufRead) -> Result<Header, Fail> {
    let mut line = Vec::new();
    let mut len = 0u64;
    let mut names: Option<Vec<String>> = None;
    let mut sizes: Option<Vec<usize>> = None;
    let mut types: Option<Vec<String>> = None;
    let mut counts: Option<Vec<usize>> = None;
    let mut width: Option<u64> = None;
    let mut height: Option<u64> = None;
    let mut points: Option<u64> = None;
    let data = loop {
        line.clear();
        let n = r
            .by_ref()
            .take(MAX_HEADER - len + 1)
            .read_until(b'\n', &mut line)?;
        len += n as u64;
        if len > MAX_HEADER {
            return bad("Kopf länger als 1 MiB");
        }
        if line.last() != Some(&b'\n') {
            return bad("Kopf ohne „DATA“");
        }
        let Ok(text) = std::str::from_utf8(&line) else {
            return bad("Kopf ist kein Text");
        };
        let text = text.trim();
        if text.is_empty() || text.starts_with('#') {
            continue;
        }
        let words: Vec<&str> = text.split_ascii_whitespace().collect();
        let (key, rest) = (words[0].to_ascii_uppercase(), &words[1..]);
        let one = |rest: &[&str]| -> Result<u64, Fail> {
            match numbers::<u64>(rest, text)?.as_slice() {
                [v] => Ok(*v),
                _ => bad(format!("Kopfzeile „{}“ ungültig", quote(text))),
            }
        };
        match key.as_str() {
            "VERSION" | "VIEWPOINT" => {}
            "FIELDS" => names = Some(rest.iter().map(|s| s.to_string()).collect()),
            "SIZE" => sizes = Some(numbers(rest, text)?),
            "TYPE" => types = Some(rest.iter().map(|s| s.to_string()).collect()),
            "COUNT" => counts = Some(numbers(rest, text)?),
            "WIDTH" => width = Some(one(rest)?),
            "HEIGHT" => height = Some(one(rest)?),
            "POINTS" => points = Some(one(rest)?),
            "DATA" => match rest {
                ["ascii"] => break Data::Ascii,
                ["binary"] => break Data::Binary,
                ["binary_compressed"] => return bad(COMPRESSED),
                _ => return bad(format!("Daten „{}“ unbekannt", quote(&rest.join(" ")))),
            },
            _ => return bad(format!("Kopfzeile „{}“ ungültig", quote(text))),
        }
    };

    let Some(names) = names.filter(|n| !n.is_empty()) else {
        return bad("Kopf ohne „FIELDS“");
    };
    let k = names.len();
    let (Some(sizes), Some(types)) = (sizes, types) else {
        return bad("Kopf ohne „SIZE“ oder „TYPE“");
    };
    let counts = counts.unwrap_or_else(|| vec![1; k]);
    if sizes.len() != k || types.len() != k || counts.len() != k {
        return bad("„SIZE“, „TYPE“ und „COUNT“ passen nicht zu „FIELDS“");
    }
    let mut fields = Vec::with_capacity(k);
    let mut offset = 0usize;
    for (((name, size), ty), count) in names.into_iter().zip(sizes).zip(types).zip(counts) {
        let ty = match ty.as_str() {
            "F" if size == 4 || size == 8 => Ty::F,
            "I" if matches!(size, 1 | 2 | 4 | 8) => Ty::I,
            "U" if matches!(size, 1 | 2 | 4 | 8) => Ty::U,
            _ => {
                return bad(format!(
                    "Feld „{}“: Typ „{} {size}“ unbekannt",
                    quote(&name),
                    quote(&ty)
                ))
            }
        };
        if count == 0 || count > 1 << 16 {
            return bad(format!("Feld „{}“: COUNT {count} ungültig", quote(&name)));
        }
        fields.push(Field {
            name,
            ty,
            size,
            count,
            offset,
        });
        offset += size * count;
        // also keeps the sum far from an overflow
        if offset > MAX_ROW {
            return bad("Zeile länger als 1 MiB");
        }
    }
    let Some(width) = width else {
        return bad("Kopf ohne „WIDTH“");
    };
    let Some(cells) = width.checked_mul(height.unwrap_or(1)) else {
        return bad("„WIDTH“ × „HEIGHT“ zu groß");
    };
    let points = points.unwrap_or(cells);
    if points != cells {
        return bad("„POINTS“ passt nicht zu „WIDTH“ × „HEIGHT“");
    }
    Ok(Header {
        fields,
        points,
        data,
        len,
    })
}

/// How the fields of a row map to the columns of [`Points`].
struct Columns {
    xyz: [usize; 3],
    /// A packed colour (`rgb`/`rgba`).
    packed: Option<usize>,
    /// Three fields `r g b`.
    separate: Option<[usize; 3]>,
    intensity: Option<usize>,
    scan: Option<usize>,
}

impl Columns {
    fn of(fields: &[Field]) -> Result<Columns, Fail> {
        let find = |name: &str| fields.iter().position(|f| f.name == name);
        let single = |i: Option<usize>| -> Result<Option<usize>, Fail> {
            match i {
                Some(i) if fields[i].count != 1 => bad(format!(
                    "Feld „{}“ mit COUNT {} statt 1",
                    quote(&fields[i].name),
                    fields[i].count
                )),
                _ => Ok(i),
            }
        };
        let (Some(x), Some(y), Some(z)) = (find("x"), find("y"), find("z")) else {
            return bad("Punkte ohne x, y oder z");
        };
        for i in [x, y, z] {
            single(Some(i))?;
            if fields[i].ty != Ty::F {
                return bad(format!(
                    "Feld „{}“ muss vom Typ F 4 oder F 8 sein",
                    quote(&fields[i].name)
                ));
            }
        }
        let packed = single(find("rgb").or_else(|| find("rgba")))?;
        if let Some(i) = packed.filter(|&i| fields[i].size != 4) {
            return bad(format!(
                "Feld „{}“ muss 4 Bytes groß sein",
                quote(&fields[i].name)
            ));
        }
        let separate = match (find("r"), find("g"), find("b")) {
            (Some(r), Some(g), Some(b)) if packed.is_none() => {
                for i in [r, g, b] {
                    single(Some(i))?;
                }
                Some([r, g, b])
            }
            _ => None,
        };
        Ok(Columns {
            xyz: [x, y, z],
            packed,
            separate,
            intensity: single(find("intensity"))?,
            scan: single(find("scan"))?,
        })
    }
}

/// A packed colour `0x??RRGGBB` as 16 bit.
fn unpack(bits: u32) -> [u16; 3] {
    [bits >> 16, bits >> 8, bits].map(|v| (v & 0xff) as u16 * 257)
}

/// An 8-bit colour channel of a foreign file as 16 bit (rounded, clamped, NaN → 0).
fn channel(v: f64) -> u16 {
    (v.round().clamp(0.0, 255.0) as u16) * 257
}

/// The packed colour of an ASCII value: PCL writes it as a whole number even for type F,
/// other writers print the float.
fn packed_text(token: &[u8]) -> Result<u32, Fail> {
    let text = String::from_utf8_lossy(token);
    if let Ok(v) = text.parse::<i64>() {
        // the 4 bytes, also of a negative I 4
        return Ok(v as u32);
    }
    text.parse::<f32>()
        .map(f32::to_bits)
        .or_else(|_| bad(format!("Wert „{}“ ist keine Zahl", quote(&text))))
}

fn read(path: &Path, progress: &Progress) -> Result<Points, Fail> {
    let file = File::open(path).map_err(Fail::Io)?;
    let file_len = file.metadata().map_err(Fail::Io)?.len();
    let mut r = BufReader::with_capacity(READ_BUFFER, file);
    let header = parse_header(&mut r)?;
    let fields = &header.fields;
    let cols = Columns::of(fields)?;
    let stride: usize = fields.iter().map(|f| f.size * f.count).sum();
    let values: usize = fields.iter().map(|f| f.count).sum();

    // The points must fit into the rest of the file.
    let rest = u128::from(file_len.saturating_sub(header.len));
    let total = header.points;
    let need = match header.data {
        Data::Binary => u128::from(total) * stride as u128,
        // a character per value and a separator between two values
        Data::Ascii => (u128::from(total) * 2 * values as u128).saturating_sub(1),
    };
    if need > rest {
        return bad("Anzahl der Punkte passt nicht zur Dateigröße");
    }

    // bounded by the file size above
    let capacity = total as usize;
    let with = |on: bool| if on { capacity } else { 0 };
    let has_rgb = cols.packed.is_some() || cols.separate.is_some();
    let mut out = Points {
        xyz: Vec::with_capacity(capacity),
        rgb: has_rgb.then(|| Vec::with_capacity(capacity)),
        source: cols.scan.map(|_| Vec::with_capacity(capacity)),
        ..Points::default()
    };
    let mut raw_intensity: Vec<f32> = Vec::with_capacity(with(cols.intensity.is_some()));
    // One value per field (the first item), the packed colour as its bits.
    let mut row = vec![0.0f64; fields.len()];
    let mut bits = 0u32;
    let mut buf = vec![
        0u8;
        if header.data == Data::Binary {
            stride
        } else {
            0
        }
    ];
    let mut token = Vec::with_capacity(MAX_TOKEN);
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
            match header.data {
                Data::Binary => {
                    r.read_exact(&mut buf)?;
                    for (f, v) in fields.iter().zip(row.iter_mut()) {
                        *v = f.decode(&buf);
                    }
                    if let Some(i) = cols.packed {
                        bits = fields[i].bits(&buf);
                    }
                }
                Data::Ascii => {
                    for (i, f) in fields.iter().enumerate() {
                        for item in 0..f.count {
                            if item > 0 {
                                next_token(&mut r, &mut token)?;
                            } else if Some(i) == cols.packed {
                                next_token(&mut r, &mut token)?;
                                bits = packed_text(&token)?;
                            } else {
                                row[i] = number(&mut r, &mut token)?;
                            }
                        }
                    }
                }
            }
            let p = cols.xyz.map(|i| row[i]);
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            out.xyz.push(p);
            if let Some(rgb) = &mut out.rgb {
                rgb.push(match cols.separate {
                    Some(c) => c.map(|i| channel(row[i])),
                    None => unpack(bits),
                });
            }
            if let Some(i) = cols.intensity {
                raw_intensity.push(row[i] as f32);
            }
            if let (Some(source), Some(i)) = (&mut out.source, cols.scan) {
                source.push(row[i].clamp(0.0, f64::from(u16::MAX)) as u16);
            }
        }
        done += n;
        progress.advance(n);
    }
    out.intensity = cols
        .intensity
        .map(|i| intensities(&raw_intensity, fields[i].ty == Ty::F));
    Ok(out)
}
