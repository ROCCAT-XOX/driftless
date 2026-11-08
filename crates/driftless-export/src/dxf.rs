//! AutoCAD 2000 text DXF (AC1015) for the floor plan, written without a library.
//!
//! A [`Drawing`] holds layers, blocks and entities in millimetres; [`dxf_bytes`] turns it into
//! the bytes of the file. Sections, tables and objects follow what `ezdxf.new("R2000")` writes,
//! reduced to what the format needs. The same drawing gives the same bytes: there are no time
//! stamps and the handles are counted in the order of writing.
//!
//! Strings are Windows-1252. Other characters up to U+FFFF become `\U+XXXX`, anything above `?`;
//! line breaks and control characters become spaces, so that each group stays two lines. Numbers
//! have three decimals and never an exponent. Every line ends in `\n`, on every system.
//!
//! What no CAD program could open is refused before anything is written: bad names, references
//! to layers or blocks that do not exist, numbers that are not finite, circles without a radius.
//!
//! Attributes: the first one of a block is visible next to the symbol, the others are hidden
//! and stand below it, one per line, for whoever switches them on in the CAD program.

use crate::{check_cancel, ExportError, PHASE_WRITE};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use std::collections::BTreeSet;
use std::path::Path;

/// Application name of the extended data.
const APP_ID: &str = "DRIFTLESS";
/// Line type of everything dashed: 100 mm line, 50 mm gap.
const DASHED: &str = "GESTRICHELT";
const STYLE: &str = "Standard";
/// Extended data strings are limited to 255 bytes.
const MAX_XDATA: usize = 255;
/// The limit of the format for one line.
const MAX_LINE: usize = 2049;
/// Layer and block names, as AutoCAD 2000 allows them.
const MAX_NAME: usize = 255;
/// Size and place of the attributes, relative to the insertion point (mm).
const ATTRIBUTE_HEIGHT: f64 = 100.0;
const ATTRIBUTE_X: f64 = 120.0;
const ATTRIBUTE_LINE: f64 = 140.0;
/// Width to height of the opening view.
const VIEW_ASPECT: f64 = 1.34;

/// A whole drawing; coordinates in millimetres.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Drawing {
    pub layers: Vec<Layer>,
    pub blocks: Vec<Block>,
    pub entities: Vec<Entity>,
    /// Comment lines at the top of the file.
    pub comments: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Layer {
    pub name: String,
    /// AutoCAD colour index, 1 to 255.
    pub color: u8,
    /// A layer that is off is still written, with a negative colour.
    pub on: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    Line {
        a: [f64; 2],
        b: [f64; 2],
        dashed: bool,
    },
    Polyline {
        points: Vec<[f64; 2]>,
        closed: bool,
        dashed: bool,
    },
    Circle {
        centre: [f64; 2],
        radius: f64,
    },
    Text {
        at: [f64; 2],
        height: f64,
        rotation_deg: f64,
        value: String,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Block {
    pub name: String,
    pub shapes: Vec<Shape>,
    /// Tags, one `ATTDEF` each. An `INSERT` may only use these.
    pub attributes: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum EntityKind {
    Shape(Shape),
    Insert {
        block: String,
        at: [f64; 2],
        /// Tag and value. A tag of the block that is left out gets an empty value.
        attributes: Vec<(String, String)>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Entity {
    pub layer: String,
    pub kind: EntityKind,
    /// Strings of the extended data under `DRIFTLESS`, cut to 255 bytes.
    pub xdata: Vec<String>,
}

/// Writes `drawing` through `<name>.tmp` next to `path`, at height `elevation_mm`.
pub fn write_dxf(
    path: &Path,
    drawing: &Drawing,
    elevation_mm: f64,
    progress: &Progress,
) -> Result<(), ExportError> {
    check_cancel(progress)?;
    progress.phase(PHASE_WRITE, Some(1));
    let bytes = dxf_bytes(drawing, elevation_mm).map_err(|msg| ExportError::format(path, msg))?;
    let temp = TempPath::new(path);
    std::fs::write(temp.tmp(), bytes).map_err(|e| ExportError::io(path, e))?;
    check_cancel(progress)?;
    temp.commit().map_err(|e| ExportError::io(path, e))?;
    progress.advance(1);
    Ok(())
}

/// The file for `drawing`; every entity outside the blocks stands at height `elevation_mm`.
/// The error is a German sentence for the user.
pub fn dxf_bytes(drawing: &Drawing, elevation_mm: f64) -> Result<Vec<u8>, String> {
    check(drawing, elevation_mm)?;

    let mut ids = Ids { next: 1 };
    let fixed = Fixed::new(&mut ids);
    let records: Vec<u64> = drawing.blocks.iter().map(|_| ids.take()).collect();
    let bounds = extents(drawing);

    // Everything after the header first: the header names the next free handle.
    let mut body = Out::default();
    classes(&mut body);
    tables(&mut body, &mut ids, &fixed, drawing, &records, bounds);
    blocks(&mut body, &mut ids, &fixed, drawing, &records);
    entities(&mut body, &mut ids, &fixed, drawing, elevation_mm);
    objects(&mut body, &fixed);

    let mut out = Out::default();
    for comment in &drawing.comments {
        out.text(999, comment);
    }
    header(&mut out, bounds, elevation_mm, ids.next);
    out.bytes.extend_from_slice(&body.bytes);
    out.pair(0, "EOF");
    Ok(out.bytes)
}

/// Line breaks and control characters, which must not reach the file.
fn is_break(c: char) -> bool {
    c.is_control() || matches!(c, '\u{2028}' | '\u{2029}')
}

fn is_finite(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

fn check_name(kind: &str, name: &str) -> Result<(), String> {
    let forbidden = |c: char| {
        is_break(c)
            || c as u32 > 0xFFFF
            || matches!(
                c,
                '<' | '>' | '/' | '\\' | '"' | ':' | ';' | '?' | '*' | '|' | '=' | '`'
            )
    };
    if name.is_empty() || name.chars().count() > MAX_NAME || name.chars().any(forbidden) {
        return Err(format!("{kind}-Name nicht erlaubt: „{name}“"));
    }
    Ok(())
}

fn check_tag(tag: &str) -> Result<(), String> {
    let forbidden = |c: char| is_break(c) || c == ' ' || c == '!' || c as u32 > 0xFFFF;
    if tag.is_empty() || tag.chars().count() > MAX_NAME || tag.chars().any(forbidden) {
        return Err(format!("Attribut-Name nicht erlaubt: „{tag}“"));
    }
    Ok(())
}

fn check_shape(shape: &Shape) -> Result<(), String> {
    let (fine, what) = match shape {
        Shape::Line { a, b, .. } => (is_finite(a) && is_finite(b), "Linie"),
        Shape::Polyline { points, .. } => (
            points.len() >= 2 && points.iter().all(|p| is_finite(p)),
            "Linienzug",
        ),
        Shape::Circle { centre, radius } => (
            is_finite(centre) && radius.is_finite() && *radius > 0.0,
            "Kreis",
        ),
        Shape::Text {
            at,
            height,
            rotation_deg,
            ..
        } => (
            is_finite(at) && is_finite(&[*height, *rotation_deg]) && *height > 0.0,
            "Text",
        ),
    };
    if fine {
        Ok(())
    } else {
        Err(format!("{what} lässt sich nicht zeichnen"))
    }
}

/// Inserts the name into `seen`; names differing only in case count as the same, as in CAD.
fn unique(kind: &str, seen: &mut BTreeSet<String>, name: &str) -> Result<(), String> {
    if seen.insert(name.to_lowercase()) {
        Ok(())
    } else {
        Err(format!("{kind} doppelt: „{name}“"))
    }
}

/// Everything that would make the file unusable, as a sentence.
fn check(drawing: &Drawing, elevation_mm: f64) -> Result<(), String> {
    if !elevation_mm.is_finite() {
        return Err("Die Höhe der Zeichnung ist keine Zahl".to_string());
    }
    let mut seen = BTreeSet::from(["0".to_string()]);
    for layer in &drawing.layers {
        check_name("Layer", &layer.name)?;
        unique("Layer", &mut seen, &layer.name)?;
        if layer.color == 0 {
            return Err(format!("Layer „{}“ hat keine Farbe", layer.name));
        }
    }
    let mut seen = BTreeSet::new();
    for block in &drawing.blocks {
        check_name("Block", &block.name)?;
        unique("Block", &mut seen, &block.name)?;
        block.shapes.iter().try_for_each(check_shape)?;
        let mut tags = BTreeSet::new();
        for tag in &block.attributes {
            check_tag(tag)?;
            unique("Attribut", &mut tags, tag)?;
        }
    }
    for entity in &drawing.entities {
        if entity.layer != "0" && !drawing.layers.iter().any(|l| l.name == entity.layer) {
            return Err(format!("Layer „{}“ ist nicht angelegt", entity.layer));
        }
        match &entity.kind {
            EntityKind::Shape(shape) => check_shape(shape)?,
            EntityKind::Insert {
                block,
                at,
                attributes,
            } => {
                let Some(found) = drawing.blocks.iter().find(|b| b.name == *block) else {
                    return Err(format!("Block „{block}“ ist nicht angelegt"));
                };
                if !is_finite(at) {
                    return Err(format!("Block „{block}“ hat keine gültige Lage"));
                }
                let mut tags = BTreeSet::new();
                for (tag, _) in attributes {
                    unique("Attribut", &mut tags, tag)?;
                    if !found.attributes.contains(tag) {
                        return Err(format!("Block „{block}“ hat kein Attribut „{tag}“"));
                    }
                }
            }
        }
    }
    Ok(())
}

/// The next free handle. 0 stands for "no owner".
struct Ids {
    next: u64,
}

impl Ids {
    fn take(&mut self) -> u64 {
        let handle = self.next;
        self.next += 1;
        handle
    }
}

/// Handles that are referred to before the object is written.
struct Fixed {
    vport: u64,
    ltype: u64,
    layer: u64,
    style: u64,
    view: u64,
    ucs: u64,
    appid: u64,
    dimstyle: u64,
    block_record: u64,
    model: u64,
    paper: u64,
    model_layout: u64,
    paper_layout: u64,
    root: u64,
    group: u64,
    layouts: u64,
    mline_styles: u64,
    plot_settings: u64,
    plot_styles: u64,
    placeholder: u64,
    mline_style: u64,
}

impl Fixed {
    fn new(ids: &mut Ids) -> Fixed {
        Fixed {
            vport: ids.take(),
            ltype: ids.take(),
            layer: ids.take(),
            style: ids.take(),
            view: ids.take(),
            ucs: ids.take(),
            appid: ids.take(),
            dimstyle: ids.take(),
            block_record: ids.take(),
            model: ids.take(),
            paper: ids.take(),
            model_layout: ids.take(),
            paper_layout: ids.take(),
            root: ids.take(),
            group: ids.take(),
            layouts: ids.take(),
            mline_styles: ids.take(),
            plot_settings: ids.take(),
            plot_styles: ids.take(),
            placeholder: ids.take(),
            mline_style: ids.take(),
        }
    }
}

/// Text as Windows-1252, cut to at most `max` bytes, never inside an escape.
fn encode(value: &str, max: usize) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(value.len().min(max));
    for c in value.chars() {
        let before = bytes.len();
        if is_break(c) {
            bytes.push(b' ');
        } else if let Some(byte) = windows_1252(c) {
            bytes.push(byte);
        } else if c as u32 <= 0xFFFF {
            bytes.extend_from_slice(format!("\\U+{:04X}", c as u32).as_bytes());
        } else {
            bytes.push(b'?');
        }
        if bytes.len() > max {
            bytes.truncate(before);
            break;
        }
    }
    bytes
}

/// The byte of `c` in Windows-1252, `None` if the code page does not have it.
fn windows_1252(c: char) -> Option<u8> {
    /// What the code page puts at 0x80 to 0x9F instead of the control characters of Latin-1.
    const HIGH: [(char, u8); 27] = [
        ('\u{20AC}', 0x80),
        ('\u{201A}', 0x82),
        ('\u{0192}', 0x83),
        ('\u{201E}', 0x84),
        ('\u{2026}', 0x85),
        ('\u{2020}', 0x86),
        ('\u{2021}', 0x87),
        ('\u{02C6}', 0x88),
        ('\u{2030}', 0x89),
        ('\u{0160}', 0x8A),
        ('\u{2039}', 0x8B),
        ('\u{0152}', 0x8C),
        ('\u{017D}', 0x8E),
        ('\u{2018}', 0x91),
        ('\u{2019}', 0x92),
        ('\u{201C}', 0x93),
        ('\u{201D}', 0x94),
        ('\u{2022}', 0x95),
        ('\u{2013}', 0x96),
        ('\u{2014}', 0x97),
        ('\u{02DC}', 0x98),
        ('\u{2122}', 0x99),
        ('\u{0161}', 0x9A),
        ('\u{203A}', 0x9B),
        ('\u{0153}', 0x9C),
        ('\u{017E}', 0x9E),
        ('\u{0178}', 0x9F),
    ];
    match c as u32 {
        code @ (0x20..=0x7E | 0xA0..=0xFF) => Some(code as u8),
        _ => HIGH.iter().find(|&&(k, _)| k == c).map(|&(_, byte)| byte),
    }
}

/// The file, group by group: a right-aligned code on one line, the value on the next.
#[derive(Default)]
struct Out {
    bytes: Vec<u8>,
}

impl Out {
    fn raw(&mut self, code: u16, value: &[u8]) {
        self.bytes
            .extend_from_slice(format!("{code:>3}\n").as_bytes());
        self.bytes.extend_from_slice(value);
        self.bytes.push(b'\n');
    }

    /// A fixed value in plain ASCII.
    fn pair(&mut self, code: u16, value: &str) {
        self.raw(code, value.as_bytes());
    }

    fn pairs(&mut self, pairs: &[(u16, &str)]) {
        for (code, value) in pairs {
            self.pair(*code, value);
        }
    }

    /// A value from outside: encoded and cut to the length of a line.
    fn text(&mut self, code: u16, value: &str) {
        self.raw(code, &encode(value, MAX_LINE));
    }

    fn num(&mut self, code: u16, value: f64) {
        let text = format!("{value:.3}");
        self.pair(code, if text == "-0.000" { "0.000" } else { &text });
    }

    fn int(&mut self, code: u16, value: i64) {
        self.pair(code, &value.to_string());
    }

    fn handle(&mut self, code: u16, handle: u64) {
        self.pair(code, &format!("{handle:X}"));
    }

    /// x, y and z under `code`, `code + 10`, `code + 20`.
    fn point(&mut self, code: u16, x: f64, y: f64, z: f64) {
        self.num(code, x);
        self.num(code + 10, y);
        self.num(code + 20, z);
    }

    fn point2(&mut self, code: u16, p: [f64; 2]) {
        self.num(code, p[0]);
        self.num(code + 10, p[1]);
    }

    fn section(&mut self, name: &str) {
        self.pair(0, "SECTION");
        self.pair(2, name);
    }

    fn end_section(&mut self) {
        self.pair(0, "ENDSEC");
    }

    /// Type, handle and owner.
    fn object(&mut self, kind: &str, handle: u64, owner: u64) {
        self.pair(0, kind);
        self.handle(5, handle);
        self.handle(330, owner);
    }

    /// The start of an entity; returns its handle.
    fn entity(&mut self, ids: &mut Ids, kind: &str, owner: u64, layer: &str, dashed: bool) -> u64 {
        let handle = ids.take();
        self.object(kind, handle, owner);
        self.pair(100, "AcDbEntity");
        self.text(8, layer);
        if dashed {
            self.pair(6, DASHED);
        }
        handle
    }

    fn table(&mut self, name: &str, handle: u64, entries: usize) {
        self.pair(0, "TABLE");
        self.pair(2, name);
        self.handle(5, handle);
        self.handle(330, 0);
        self.pair(100, "AcDbSymbolTable");
        self.int(70, entries as i64);
    }

    /// An entry of a symbol table, up to its name.
    fn record(&mut self, kind: &str, handle: u64, table: u64, subclass: &str, name: &str) {
        self.object(kind, handle, table);
        self.pair(100, "AcDbSymbolTableRecord");
        self.pair(100, subclass);
        self.text(2, name);
    }

    fn end_table(&mut self) {
        self.pair(0, "ENDTAB");
    }

    /// A dictionary owned by `owner`, with its entries.
    fn dictionary(&mut self, handle: u64, owner: u64, entries: &[(&str, u64)]) {
        self.object("DICTIONARY", handle, owner);
        self.pair(100, "AcDbDictionary");
        self.int(281, 1);
        self.entries(entries);
    }

    fn entries(&mut self, entries: &[(&str, u64)]) {
        for (name, handle) in entries {
            self.pair(3, name);
            self.handle(350, *handle);
        }
    }
}

/// Smallest and largest corner of everything drawn.
type Bounds = Option<([f64; 2], [f64; 2])>;

fn grow(bounds: &mut Bounds, x: f64, y: f64) {
    let (min, max) = bounds.get_or_insert(([x, y], [x, y]));
    *min = [min[0].min(x), min[1].min(y)];
    *max = [max[0].max(x), max[1].max(y)];
}

/// Adds `shape`, moved by `at`.
fn grow_by_shape(bounds: &mut Bounds, shape: &Shape, at: [f64; 2]) {
    let mut point = |p: &[f64; 2]| grow(bounds, p[0] + at[0], p[1] + at[1]);
    match shape {
        Shape::Line { a, b, .. } => {
            point(a);
            point(b);
        }
        Shape::Polyline { points, .. } => points.iter().for_each(point),
        Shape::Circle { centre, radius } => {
            point(&[centre[0] - radius, centre[1] - radius]);
            point(&[centre[0] + radius, centre[1] + radius]);
        }
        Shape::Text { at, .. } => point(at),
    }
}

fn extents(drawing: &Drawing) -> Bounds {
    let mut bounds = None;
    for entity in &drawing.entities {
        match &entity.kind {
            EntityKind::Shape(shape) => grow_by_shape(&mut bounds, shape, [0.0, 0.0]),
            EntityKind::Insert { block, at, .. } => {
                grow(&mut bounds, at[0], at[1]);
                if let Some(found) = drawing.blocks.iter().find(|b| b.name == *block) {
                    for shape in &found.shapes {
                        grow_by_shape(&mut bounds, shape, *at);
                    }
                }
            }
        }
    }
    bounds
}

fn header(out: &mut Out, bounds: Bounds, elevation_mm: f64, handle_seed: u64) {
    let (ext_min, ext_max) = bounds.unwrap_or(([0.0; 2], [0.0; 2]));
    let (lim_min, lim_max) = bounds.unwrap_or(([0.0; 2], [420.0, 297.0]));

    out.section("HEADER");
    out.pairs(&[
        (9, "$ACADVER"),
        (1, "AC1015"),
        (9, "$DWGCODEPAGE"),
        (3, "ANSI_1252"),
        (9, "$INSBASE"),
    ]);
    out.point(10, 0.0, 0.0, 0.0);
    out.pair(9, "$EXTMIN");
    out.point(10, ext_min[0], ext_min[1], elevation_mm);
    out.pair(9, "$EXTMAX");
    out.point(10, ext_max[0], ext_max[1], elevation_mm);
    out.pair(9, "$LIMMIN");
    out.point2(10, lim_min);
    out.pair(9, "$LIMMAX");
    out.point2(10, lim_max);
    out.pairs(&[
        (9, "$LTSCALE"),
        (40, "1.000"),
        (9, "$TEXTSTYLE"),
        (7, STYLE),
        (9, "$CLAYER"),
        (8, "0"),
        (9, "$DIMSTYLE"),
        (2, STYLE),
        (9, "$LUNITS"),
        (70, "2"),
        (9, "$AUNITS"),
        (70, "0"),
        (9, "$HANDSEED"),
    ]);
    out.handle(5, handle_seed);
    out.pairs(&[
        (9, "$MEASUREMENT"),
        (70, "1"),
        (9, "$LWDISPLAY"),
        (290, "0"),
        (9, "$INSUNITS"),
        (70, "4"),
        (9, "$EXTNAMES"),
        (290, "1"),
    ]);
    out.end_section();
}

fn classes(out: &mut Out) {
    out.section("CLASSES");
    for (record, class) in [
        ("ACDBDICTIONARYWDFLT", "AcDbDictionaryWithDefault"),
        ("ACDBPLACEHOLDER", "AcDbPlaceHolder"),
        ("LAYOUT", "AcDbLayout"),
    ] {
        out.pair(0, "CLASS");
        out.pair(1, record);
        out.pair(2, class);
        out.pair(3, "ObjectDBX Classes");
        out.pairs(&[(90, "0"), (280, "0"), (281, "0")]);
    }
    out.end_section();
}

fn tables(
    out: &mut Out,
    ids: &mut Ids,
    fixed: &Fixed,
    drawing: &Drawing,
    records: &[u64],
    bounds: Bounds,
) {
    out.section("TABLES");

    // The view the file opens with shows the whole drawing.
    let (min, max) = bounds.unwrap_or(([0.0; 2], [420.0, 297.0]));
    let centre = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0];
    let height = ((max[1] - min[1]).max((max[0] - min[0]) / VIEW_ASPECT) * 1.1).max(1.0);
    out.table("VPORT", fixed.vport, 1);
    out.record(
        "VPORT",
        ids.take(),
        fixed.vport,
        "AcDbViewportTableRecord",
        "*Active",
    );
    out.pairs(&[
        (70, "0"),
        (10, "0.000"),
        (20, "0.000"),
        (11, "1.000"),
        (21, "1.000"),
    ]);
    out.point2(12, centre);
    out.pairs(&[
        (13, "0.000"),
        (23, "0.000"),
        (14, "0.500"),
        (24, "0.500"),
        (15, "0.500"),
        (25, "0.500"),
        (16, "0.000"),
        (26, "0.000"),
        (36, "1.000"),
        (17, "0.000"),
        (27, "0.000"),
        (37, "0.000"),
    ]);
    out.num(40, height);
    out.num(41, VIEW_ASPECT);
    out.pairs(&[
        (42, "50.000"),
        (43, "0.000"),
        (44, "0.000"),
        (50, "0.000"),
        (51, "0.000"),
        (71, "0"),
        (72, "1000"),
        (73, "1"),
        (74, "3"),
        (75, "0"),
        (76, "0"),
        (77, "0"),
        (78, "0"),
        (281, "0"),
        (65, "0"),
        (146, "0.000"),
    ]);
    out.end_table();

    out.table("LTYPE", fixed.ltype, 4);
    for name in ["ByBlock", "ByLayer", "Continuous"] {
        out.record(
            "LTYPE",
            ids.take(),
            fixed.ltype,
            "AcDbLinetypeTableRecord",
            name,
        );
        out.pairs(&[(70, "0"), (3, ""), (72, "65"), (73, "0"), (40, "0.000")]);
    }
    out.record(
        "LTYPE",
        ids.take(),
        fixed.ltype,
        "AcDbLinetypeTableRecord",
        DASHED,
    );
    out.pairs(&[
        (70, "0"),
        (3, "Gestrichelt __ __ __"),
        (72, "65"),
        (73, "2"),
        (40, "150.000"),
        (49, "100.000"),
        (74, "0"),
        (49, "-50.000"),
        (74, "0"),
    ]);
    out.end_table();

    out.table("LAYER", fixed.layer, drawing.layers.len() + 1);
    let zero = Layer {
        name: "0".to_string(),
        color: 7,
        on: true,
    };
    for layer in std::iter::once(&zero).chain(&drawing.layers) {
        out.record(
            "LAYER",
            ids.take(),
            fixed.layer,
            "AcDbLayerTableRecord",
            &layer.name,
        );
        out.int(70, 0);
        let color = i64::from(layer.color);
        out.int(62, if layer.on { color } else { -color });
        out.pair(6, "Continuous");
        out.int(370, -3);
        out.handle(390, fixed.placeholder);
    }
    out.end_table();

    out.table("STYLE", fixed.style, 1);
    out.record(
        "STYLE",
        ids.take(),
        fixed.style,
        "AcDbTextStyleTableRecord",
        STYLE,
    );
    out.pairs(&[
        (70, "0"),
        (40, "0.000"),
        (41, "1.000"),
        (50, "0.000"),
        (71, "0"),
        (42, "2.500"),
        (3, "arial.ttf"),
        (4, ""),
    ]);
    out.end_table();

    out.table("VIEW", fixed.view, 0);
    out.end_table();
    out.table("UCS", fixed.ucs, 0);
    out.end_table();

    out.table("APPID", fixed.appid, 2);
    for name in ["ACAD", APP_ID] {
        out.record(
            "APPID",
            ids.take(),
            fixed.appid,
            "AcDbRegAppTableRecord",
            name,
        );
        out.int(70, 0);
    }
    out.end_table();

    out.table("DIMSTYLE", fixed.dimstyle, 1);
    out.pair(100, "AcDbDimStyleTable");
    // DIMSTYLE alone has its handle in group 105.
    out.pair(0, "DIMSTYLE");
    out.handle(105, ids.take());
    out.handle(330, fixed.dimstyle);
    out.pairs(&[
        (100, "AcDbSymbolTableRecord"),
        (100, "AcDbDimStyleTableRecord"),
        (2, STYLE),
        (70, "0"),
        (3, ""),
        (4, ""),
        (40, "1.000"),
        (41, "2.500"),
        (42, "0.625"),
        (43, "3.750"),
        (44, "1.250"),
        (45, "0.000"),
        (46, "0.000"),
        (47, "0.000"),
        (48, "0.000"),
        (140, "2.500"),
        (141, "2.500"),
        (142, "0.000"),
        (143, "0.03937007874"),
        (144, "1.000"),
        (145, "0.000"),
        (146, "1.000"),
        (147, "0.625"),
        (148, "0.000"),
        (71, "0"),
        (72, "0"),
        (73, "0"),
        (74, "0"),
        (75, "0"),
        (76, "0"),
        (77, "1"),
        (78, "8"),
        (79, "3"),
        (170, "0"),
        (171, "3"),
        (172, "1"),
        (173, "0"),
        (174, "0"),
        (175, "0"),
        (176, "0"),
        (177, "0"),
        (178, "0"),
        (179, "2"),
        (271, "2"),
        (272, "2"),
        (273, "2"),
        (274, "3"),
        (275, "0"),
        (276, "0"),
        (277, "2"),
        (278, "44"),
        (279, "0"),
        (280, "0"),
        (281, "0"),
        (282, "0"),
        (283, "0"),
        (284, "8"),
        (285, "0"),
        (286, "0"),
        (288, "0"),
        (289, "3"),
        (371, "-2"),
        (372, "-2"),
    ]);
    out.end_table();

    out.table("BLOCK_RECORD", fixed.block_record, drawing.blocks.len() + 2);
    let spaces = [
        ("*Model_Space", fixed.model, fixed.model_layout),
        ("*Paper_Space", fixed.paper, fixed.paper_layout),
    ];
    let own = drawing
        .blocks
        .iter()
        .zip(records)
        .map(|(block, record)| (block.name.as_str(), *record, 0));
    for (name, record, layout) in spaces.into_iter().chain(own) {
        out.record(
            "BLOCK_RECORD",
            record,
            fixed.block_record,
            "AcDbBlockTableRecord",
            name,
        );
        out.handle(340, layout);
    }
    out.end_table();

    out.end_section();
}

fn block_begin(out: &mut Out, ids: &mut Ids, record: u64, name: &str, has_attributes: bool) {
    out.entity(ids, "BLOCK", record, "0", false);
    out.pair(100, "AcDbBlockBegin");
    out.text(2, name);
    out.int(70, if has_attributes { 2 } else { 0 });
    out.point(10, 0.0, 0.0, 0.0);
    out.text(3, name);
    out.pair(1, "");
}

fn block_end(out: &mut Out, ids: &mut Ids, record: u64) {
    out.entity(ids, "ENDBLK", record, "0", false);
    out.pair(100, "AcDbBlockEnd");
}

/// Row `index` of the attributes, relative to the insertion point.
fn attribute_offset(index: usize) -> [f64; 2] {
    [ATTRIBUTE_X, -(index as f64) * ATTRIBUTE_LINE]
}

/// Group 70 of attribute `index`: the first is visible, the others are hidden.
fn attribute_flags(index: usize) -> i64 {
    i64::from(index != 0)
}

fn blocks(out: &mut Out, ids: &mut Ids, fixed: &Fixed, drawing: &Drawing, records: &[u64]) {
    out.section("BLOCKS");
    for (name, record) in [("*Model_Space", fixed.model), ("*Paper_Space", fixed.paper)] {
        block_begin(out, ids, record, name, false);
        block_end(out, ids, record);
    }
    for (block, &record) in drawing.blocks.iter().zip(records) {
        block_begin(out, ids, record, &block.name, !block.attributes.is_empty());
        for shape in &block.shapes {
            write_shape(out, ids, record, "0", shape, 0.0);
        }
        for (index, tag) in block.attributes.iter().enumerate() {
            let offset = attribute_offset(index);
            out.entity(ids, "ATTDEF", record, "0", false);
            out.pair(100, "AcDbText");
            out.point(10, offset[0], offset[1], 0.0);
            out.num(40, ATTRIBUTE_HEIGHT);
            out.pair(1, "");
            out.pair(100, "AcDbAttributeDefinition");
            out.text(3, tag);
            out.text(2, tag);
            out.int(70, attribute_flags(index));
        }
        block_end(out, ids, record);
    }
    out.end_section();
}

fn write_shape(out: &mut Out, ids: &mut Ids, owner: u64, layer: &str, shape: &Shape, z: f64) {
    match shape {
        Shape::Line { a, b, dashed } => {
            out.entity(ids, "LINE", owner, layer, *dashed);
            out.pair(100, "AcDbLine");
            out.point(10, a[0], a[1], z);
            out.point(11, b[0], b[1], z);
        }
        Shape::Polyline {
            points,
            closed,
            dashed,
        } => {
            out.entity(ids, "LWPOLYLINE", owner, layer, *dashed);
            out.pair(100, "AcDbPolyline");
            out.int(90, points.len() as i64);
            out.int(70, i64::from(*closed));
            out.num(38, z);
            for p in points {
                out.point2(10, *p);
            }
        }
        Shape::Circle { centre, radius } => {
            out.entity(ids, "CIRCLE", owner, layer, false);
            out.pair(100, "AcDbCircle");
            out.point(10, centre[0], centre[1], z);
            out.num(40, *radius);
        }
        Shape::Text {
            at,
            height,
            rotation_deg,
            value,
        } => {
            out.entity(ids, "TEXT", owner, layer, false);
            out.pair(100, "AcDbText");
            out.point(10, at[0], at[1], z);
            out.num(40, *height);
            out.text(1, value);
            out.num(50, *rotation_deg);
            out.pair(7, STYLE);
            out.pair(100, "AcDbText");
        }
    }
}

fn write_xdata(out: &mut Out, xdata: &[String]) {
    if xdata.is_empty() {
        return;
    }
    out.pair(1001, APP_ID);
    for value in xdata {
        out.raw(1000, &encode(value, MAX_XDATA));
    }
}

fn entities(out: &mut Out, ids: &mut Ids, fixed: &Fixed, drawing: &Drawing, z: f64) {
    out.section("ENTITIES");
    for entity in &drawing.entities {
        let layer = entity.layer.as_str();
        match &entity.kind {
            EntityKind::Shape(shape) => {
                write_shape(out, ids, fixed.model, layer, shape, z);
                write_xdata(out, &entity.xdata);
            }
            EntityKind::Insert {
                block,
                at,
                attributes,
            } => {
                let tags = drawing
                    .blocks
                    .iter()
                    .find(|b| b.name == *block)
                    .map_or(&[][..], |b| b.attributes.as_slice());
                let insert = out.entity(ids, "INSERT", fixed.model, layer, false);
                out.pair(100, "AcDbBlockReference");
                if !tags.is_empty() {
                    out.int(66, 1);
                }
                out.text(2, block);
                out.point(10, at[0], at[1], z);
                write_xdata(out, &entity.xdata);
                for (index, tag) in tags.iter().enumerate() {
                    let value = attributes
                        .iter()
                        .find(|(t, _)| t == tag)
                        .map_or("", |(_, v)| v.as_str());
                    let offset = attribute_offset(index);
                    out.entity(ids, "ATTRIB", insert, layer, false);
                    out.pair(100, "AcDbText");
                    out.point(10, at[0] + offset[0], at[1] + offset[1], z);
                    out.num(40, ATTRIBUTE_HEIGHT);
                    out.text(1, value);
                    out.pair(100, "AcDbAttribute");
                    out.text(2, tag);
                    out.int(70, attribute_flags(index));
                }
                if !tags.is_empty() {
                    out.entity(ids, "SEQEND", insert, layer, false);
                }
            }
        }
    }
    out.end_section();
}

/// A layout with the settings of a blank A3 sheet.
fn layout(out: &mut Out, handle: u64, owner: u64, name: &str, flags: &str, tab: &str, record: u64) {
    out.object("LAYOUT", handle, owner);
    out.pairs(&[
        (100, "AcDbPlotSettings"),
        (1, ""),
        (4, "A3"),
        (6, ""),
        (40, "7.500"),
        (41, "20.000"),
        (42, "7.500"),
        (43, "20.000"),
        (44, "420.000"),
        (45, "297.000"),
        (46, "0.000"),
        (47, "0.000"),
        (48, "0.000"),
        (49, "0.000"),
        (140, "0.000"),
        (141, "0.000"),
        (142, "1.000"),
        (143, "1.000"),
        (70, flags),
        (72, "1"),
        (73, "0"),
        (74, "5"),
        (7, ""),
        (75, "16"),
        (76, "0"),
        (77, "2"),
        (78, "300"),
        (147, "1.000"),
        (148, "0.000"),
        (149, "0.000"),
        (100, "AcDbLayout"),
        (1, name),
        (70, "1"),
        (71, tab),
        (10, "0.000"),
        (20, "0.000"),
        (11, "420.000"),
        (21, "297.000"),
        (12, "0.000"),
        (22, "0.000"),
        (32, "0.000"),
        (14, "100000000000000000000.000"),
        (24, "100000000000000000000.000"),
        (34, "100000000000000000000.000"),
        (15, "-100000000000000000000.000"),
        (25, "-100000000000000000000.000"),
        (35, "-100000000000000000000.000"),
        (146, "0.000"),
        (13, "0.000"),
        (23, "0.000"),
        (33, "0.000"),
        (16, "1.000"),
        (26, "0.000"),
        (36, "0.000"),
        (17, "0.000"),
        (27, "1.000"),
        (37, "0.000"),
        (76, "1"),
    ]);
    out.handle(330, record);
}

fn objects(out: &mut Out, fixed: &Fixed) {
    out.section("OBJECTS");
    out.dictionary(
        fixed.root,
        0,
        &[
            ("ACAD_GROUP", fixed.group),
            ("ACAD_LAYOUT", fixed.layouts),
            ("ACAD_MLINESTYLE", fixed.mline_styles),
            ("ACAD_PLOTSETTINGS", fixed.plot_settings),
            ("ACAD_PLOTSTYLENAME", fixed.plot_styles),
        ],
    );
    out.dictionary(fixed.group, fixed.root, &[]);
    out.dictionary(
        fixed.layouts,
        fixed.root,
        &[
            ("Model", fixed.model_layout),
            ("Layout1", fixed.paper_layout),
        ],
    );
    out.dictionary(
        fixed.mline_styles,
        fixed.root,
        &[(STYLE, fixed.mline_style)],
    );
    out.dictionary(fixed.plot_settings, fixed.root, &[]);

    out.object("ACDBDICTIONARYWDFLT", fixed.plot_styles, fixed.root);
    out.pair(100, "AcDbDictionary");
    out.int(281, 1);
    out.entries(&[("Normal", fixed.placeholder)]);
    out.pair(100, "AcDbDictionaryWithDefault");
    out.handle(340, fixed.placeholder);

    // The one object without a subclass, in AutoCAD's files as well.
    out.object("ACDBPLACEHOLDER", fixed.placeholder, fixed.plot_styles);

    layout(
        out,
        fixed.model_layout,
        fixed.layouts,
        "Model",
        "1024",
        "0",
        fixed.model,
    );
    layout(
        out,
        fixed.paper_layout,
        fixed.layouts,
        "Layout1",
        "0",
        "1",
        fixed.paper,
    );

    out.object("MLINESTYLE", fixed.mline_style, fixed.mline_styles);
    out.pairs(&[
        (100, "AcDbMlineStyle"),
        (2, STYLE),
        (70, "0"),
        (3, ""),
        (62, "256"),
        (51, "90.000"),
        (52, "90.000"),
        (71, "2"),
        (49, "0.500"),
        (62, "256"),
        (6, "BYLAYER"),
        (49, "-0.500"),
        (62, "256"),
        (6, "BYLAYER"),
    ]);
    out.end_section();
}
