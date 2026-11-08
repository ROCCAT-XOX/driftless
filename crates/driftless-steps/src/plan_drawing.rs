//! A floor plan and the objects of its map or model as one [`Drawing`], in millimetres: what
//! the DXF export writes and the plan view shows, so that the two cannot differ.
//!
//! [`PlanPlacement::Walls`] draws in the frame of the plan (main direction = x, lower left
//! corner = (0, 0), floor = 0); [`PlanPlacement::AsCloud`] where the points of the map lie
//! ([`PlanFrame::to_map`]), texts turned with it. Objects are stored in the map, so they lie
//! right in both, however far outside the walls; they are drawn flat, their height above the
//! floor is an attribute.
//!
//! An opening in a wall with two faces stands twice in the plan: both faces are cut, its symbol
//! and text are drawn once ([`FloorPlan::distinct_openings`]). Every layer is there even when
//! nothing is on it, one hidden in the program is written but off.

use crate::objects::{Object, ObjectKind, Objects};
use crate::plan::PlanReport;
use crate::PROGRAM;
use driftless_export::dxf::{Block, Drawing, Entity, EntityKind, Layer, Shape};
use driftless_plan::{metres, FloorPlan, Opening, OpeningKind, PlanFrame, WallFace};
use std::collections::{BTreeMap, BTreeSet};

const WALL: &str = "Wand";
const DOOR: &str = "Tür";
const WINDOW: &str = "Fenster";
const UNCLEAR: &str = "Öffnung-unklar";
const ROOM: &str = "Raum";
const DIMENSION: &str = "Bemaßung";
const MEASURE: &str = "Messung";
const SOCKET: &str = "Steckdose";
const CONNECTION: &str = "Anschluss";
const CABLE: &str = "Kabel";
const PIPE: &str = "Rohr";

/// The layers of a plan with their colour (AutoCAD colour index), in this order.
pub const LAYERS: [(&str, u8); 11] = [
    (WALL, 7),
    (DOOR, 30),
    (WINDOW, 4),
    (UNCLEAR, 8),
    (ROOM, 3),
    (DIMENSION, 2),
    (MEASURE, 6),
    (SOCKET, 1),
    (CONNECTION, 200),
    (CABLE, 5),
    (PIPE, 34),
];

/// Attributes of the blocks; the first one stands visibly next to the symbol.
const BLOCK_TAGS: [&str; 5] = ["NR", "NAME", "ART", "HOEHE", "NOTIZ"];
/// What the extended data of a cable or pipe always holds, which a free property cannot be.
const LINE_TAGS: [&str; 5] = ["NR", "NAME", "NOTIZ", "QUELLE", "DURCHMESSER_MM"];
/// Longest tag made from the key of a free property, before a number that makes it unique.
const MAX_TAG: usize = 64;
/// A key of which nothing is left for a tag.
const UNNAMED_TAG: &str = "EIGENSCHAFT";

/// Height of the texts (mm).
const TEXT_HEIGHT: f64 = 120.0;
/// Distance between the lines of a room label (mm).
const TEXT_LINE: f64 = 170.0;
/// Distance of a text from the line it belongs to (mm).
const TEXT_GAP: f64 = 60.0;
/// Rough width of a character in text heights: a text can only be centred by estimate.
const CHAR_WIDTH: f64 = 0.6;
/// The most of an object's name that goes into a text of the plan (characters); the whole name
/// is in its attributes.
const MAX_NAME_IN_TEXT: usize = 40;
/// Thickness of a wall measured from one side only (metres).
const DEFAULT_THICKNESS: f64 = 0.12;
/// Distance of the two lines of a window (metres).
const WINDOW_LINES: f64 = 0.06;
/// A piece of wall shorter than this is not drawn (metres).
const MIN_PIECE: f64 = 0.001;
/// Half a tick at the end of a dimension, per axis (mm): at 45° to the line, 100 mm long in all.
const TICK: f64 = 35.355;
/// Half the edge of the symbol of a block (mm).
const SYMBOL: f64 = 75.0;

/// Where a plan is drawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PlanPlacement {
    /// In the frame of the plan.
    #[default]
    Walls,
    /// Where the points of the map lie, at the height of the floor.
    AsCloud,
}

/// How a plan is exported.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PlanExport {
    pub placement: PlanPlacement,
    /// Layers written but off ([`LAYERS`] by name).
    pub hidden: Vec<String>,
}

/// The height the drawing stands at (mm): 0 in the frame of the plan, the floor where the points lie.
pub fn elevation_mm(report: &PlanReport, export: &PlanExport) -> f64 {
    match export.placement {
        PlanPlacement::Walls => 0.0,
        PlanPlacement::AsCloud => report.plan.frame.origin[2] * 1000.0,
    }
}

/// The drawing of `report` with `objects`, as they are now.
pub fn plan_drawing(report: &PlanReport, objects: &Objects, export: &PlanExport) -> Drawing {
    let plan = &report.plan;
    let tags = Tags::new(objects);
    let mut sheet = Sheet {
        frame: &plan.frame,
        as_cloud: export.placement == PlanPlacement::AsCloud,
        entities: Vec::new(),
    };
    sheet.walls(plan);
    sheet.openings(plan);
    sheet.rooms(plan);
    sheet.dimensions(plan);
    sheet.objects(objects, &tags);
    Drawing {
        layers: LAYERS
            .iter()
            .map(|&(name, color)| Layer {
                name: name.to_string(),
                color,
                on: !export.hidden.iter().any(|h| h == name),
            })
            .collect(),
        blocks: tags.blocks(),
        entities: sheet.entities,
        comments: comments(&plan.frame, export.placement),
    }
}

fn comments(frame: &PlanFrame, placement: PlanPlacement) -> Vec<String> {
    let lies = match placement {
        PlanPlacement::Walls => {
            "an den Wänden (Hauptrichtung = x-Achse, linke untere Ecke = (0, 0), Boden = 0)"
        }
        PlanPlacement::AsCloud => "wie die Punktwolke (Lage der LAS-Datei, Höhe des Bodens)",
    };
    let turn = decimal(frame.axis[1].atan2(frame.axis[0]).to_degrees(), 2);
    let [x, y, z] = frame.origin.map(|v| decimal(v, 3));
    vec![
        format!(
            "Programm: Driftless, Version {} ({})",
            PROGRAM.version, PROGRAM.commit
        ),
        format!("Lage: {lies}; Plan zur Karte: Drehung {turn}°, Verschiebung {x} / {y} / {z} m"),
        "Rohre: von Hand gesetzt, automatische Erkennung folgt".to_string(),
    ]
}

/// A number with a decimal comma.
fn decimal(value: f64, places: usize) -> String {
    format!("{value:.places$}").replace('.', ",")
}

/// A length without its unit: „1,20“.
fn number(x: f64) -> String {
    metres(x).trim_end_matches(" m").to_string()
}

fn unit(a: [f64; 2], b: [f64; 2]) -> Option<[f64; 2]> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    (len > 0.0).then(|| [dx / len, dy / len])
}

fn middle(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

fn left_of(d: [f64; 2]) -> [f64; 2] {
    [-d[1], d[0]]
}

/// `d`, or the opposite: the way a text runs so that it is not upside down.
fn reading(d: [f64; 2]) -> [f64; 2] {
    if d[0] < -1e-9 || (d[0].abs() <= 1e-9 && d[1] < 0.0) {
        [-d[0], -d[1]]
    } else {
        d
    }
}

/// A text centred on `centre`, running along `along`.
fn centred(centre: [f64; 2], along: [f64; 2], value: String) -> Shape {
    let up = left_of(along);
    let half = CHAR_WIDTH * TEXT_HEIGHT * value.chars().count() as f64 / 2.0;
    Shape::Text {
        at: [
            centre[0] - along[0] * half - up[0] * TEXT_HEIGHT / 2.0,
            centre[1] - along[1] * half - up[1] * TEXT_HEIGHT / 2.0,
        ],
        height: TEXT_HEIGHT,
        rotation_deg: along[1].atan2(along[0]).to_degrees(),
        value,
    }
}

/// A text beside the middle of a line: running along it, readable, on the side `side`.
fn beside(mid: [f64; 2], along: [f64; 2], side: [f64; 2], value: String) -> Shape {
    let read = reading(along);
    let up = left_of(read);
    let sign = if up[0] * side[0] + up[1] * side[1] >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let off = sign * (TEXT_GAP + TEXT_HEIGHT / 2.0);
    centred([mid[0] + up[0] * off, mid[1] + up[1] * off], read, value)
}

/// A wall face in the frame of the plan (metres).
struct Wall {
    a: [f64; 2],
    dir: [f64; 2],
    inward: [f64; 2],
    len: f64,
}

impl Wall {
    fn of(face: &WallFace) -> Option<Wall> {
        let dir = unit(face.a, face.b)?;
        Some(Wall {
            a: face.a,
            dir,
            inward: face.inward,
            len: (face.b[0] - face.a[0]).hypot(face.b[1] - face.a[1]),
        })
    }

    /// The point `along` metres from `a`, `depth` metres into the wall (negative: into the room).
    fn at(&self, along: f64, depth: f64) -> [f64; 2] {
        [
            self.a[0] + self.dir[0] * along - self.inward[0] * depth,
            self.a[1] + self.dir[1] * along - self.inward[1] * depth,
        ]
    }

    /// How far along this face the point `p` lies.
    fn along(&self, p: [f64; 2]) -> f64 {
        (p[0] - self.a[0]) * self.dir[0] + (p[1] - self.a[1]) * self.dir[1]
    }
}

/// The thickness of the wall of `face` at `along` on it: from the pair that reaches there.
fn thickness(plan: &FloorPlan, index: usize, wall: &Wall, along: f64) -> f64 {
    plan.pairs
        .iter()
        .find_map(|pair| {
            let other = match (pair.front == index, pair.back == index) {
                (true, _) => pair.back,
                (_, true) => pair.front,
                _ => return None,
            };
            let other = plan.faces.get(other)?;
            let (t0, t1) = (wall.along(other.a), wall.along(other.b));
            (t0.min(t1) <= along && along <= t0.max(t1)).then_some(pair.thickness)
        })
        .unwrap_or(DEFAULT_THICKNESS)
}

/// The size of an opening: „0,90 × 2,05 m“.
fn size(width: f64, o: &Opening) -> String {
    format!("{} × {}", number(width), metres(o.head - o.sill))
}

/// The entities being drawn.
struct Sheet<'a> {
    frame: &'a PlanFrame,
    as_cloud: bool,
    entities: Vec<Entity>,
}

impl Sheet<'_> {
    /// A point of the plan (metres) in the drawing (mm).
    fn plan(&self, p: [f64; 2]) -> [f64; 2] {
        if self.as_cloud {
            let m = self.frame.to_map([p[0], p[1], 0.0]);
            [m[0] * 1000.0, m[1] * 1000.0]
        } else {
            [p[0] * 1000.0, p[1] * 1000.0]
        }
    }

    /// A point of the map (metres) in the drawing (mm).
    fn map(&self, p: [f64; 3]) -> [f64; 2] {
        if self.as_cloud {
            [p[0] * 1000.0, p[1] * 1000.0]
        } else {
            let q = self.frame.to_plan(p);
            [q[0] * 1000.0, q[1] * 1000.0]
        }
    }

    fn add(&mut self, layer: &str, shape: Shape) {
        self.entities.push(Entity {
            layer: layer.to_string(),
            kind: EntityKind::Shape(shape),
            xdata: Vec::new(),
        });
    }

    fn line(&mut self, layer: &str, a: [f64; 2], b: [f64; 2], dashed: bool) {
        self.add(layer, Shape::Line { a, b, dashed });
    }

    /// The faces without their openings. Both entries of an opening in a wall with two faces
    /// cut their face.
    fn walls(&mut self, plan: &FloorPlan) {
        for (index, face) in plan.faces.iter().enumerate() {
            let Some(wall) = Wall::of(face) else {
                continue;
            };
            let mut gaps: Vec<(f64, f64)> = plan
                .openings
                .iter()
                .filter(|o| o.face == index)
                .map(|o| (o.from.max(0.0), o.to.min(wall.len)))
                .filter(|(from, to)| to > from)
                .collect();
            gaps.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
            let mut start = 0.0_f64;
            for (from, to) in gaps {
                self.wall_piece(&wall, start, from);
                start = start.max(to);
            }
            self.wall_piece(&wall, start, wall.len);
        }
    }

    fn wall_piece(&mut self, wall: &Wall, from: f64, to: f64) {
        if to - from < MIN_PIECE {
            return;
        }
        let points = vec![self.plan(wall.at(from, 0.0)), self.plan(wall.at(to, 0.0))];
        self.add(
            WALL,
            Shape::Polyline {
                points,
                closed: false,
                dashed: false,
            },
        );
    }

    /// End caps, symbol and text, once per real opening.
    fn openings(&mut self, plan: &FloorPlan) {
        for index in plan.distinct_openings() {
            let o = &plan.openings[index];
            let Some(wall) = plan.faces.get(o.face).and_then(Wall::of) else {
                continue;
            };
            let (from, to) = (o.from.max(0.0), o.to.min(wall.len));
            if to <= from {
                continue;
            }
            let mid = (from + to) / 2.0;
            let depth = thickness(plan, o.face, &wall, mid);
            for along in [from, to] {
                let (a, b) = (
                    self.plan(wall.at(along, 0.0)),
                    self.plan(wall.at(along, depth)),
                );
                self.line(WALL, a, b, false);
                if o.kind == OpeningKind::Door {
                    self.line(DOOR, a, b, false);
                }
            }
            let (a, b) = (self.plan(wall.at(from, 0.0)), self.plan(wall.at(to, 0.0)));
            let (layer, text) = match o.kind {
                OpeningKind::Door => {
                    self.line(DOOR, a, b, true);
                    (DOOR, format!("Tür {}", size(to - from, o)))
                }
                OpeningKind::Window => {
                    for side in [-0.5, 0.5] {
                        let at = depth / 2.0 + side * WINDOW_LINES;
                        let (p, q) = (self.plan(wall.at(from, at)), self.plan(wall.at(to, at)));
                        self.line(WINDOW, p, q, false);
                    }
                    let brh = metres(o.sill);
                    (WINDOW, format!("Fenster {}, BRH {brh}", size(to - from, o)))
                }
                OpeningKind::Unclear => {
                    self.line(UNCLEAR, a, b, true);
                    (UNCLEAR, o.note.clone())
                }
            };
            let Some(along) = unit(a, b) else {
                continue;
            };
            let (centre, room) = (self.plan(wall.at(mid, 0.0)), self.plan(wall.at(mid, -1.0)));
            let side = unit(centre, room).unwrap_or(left_of(along));
            self.add(layer, beside(middle(a, b), along, side, text));
        }
    }

    /// Outline, number and name, area and height, the texts turned with the frame.
    fn rooms(&mut self, plan: &FloorPlan) {
        let along = unit(self.plan([0.0, 0.0]), self.plan([1.0, 0.0])).unwrap_or([1.0, 0.0]);
        let along = reading(along);
        let up = left_of(along);
        for room in &plan.rooms {
            if room.outline.len() >= 2 {
                let points = room.outline.iter().map(|p| self.plan(*p)).collect();
                self.add(
                    ROOM,
                    Shape::Polyline {
                        points,
                        closed: true,
                        dashed: false,
                    },
                );
            }
            let mut text = vec![
                if room.open {
                    format!("Raum {} (offen)", room.nr)
                } else {
                    format!("Raum {}", room.nr)
                },
                format!("{} m²", number(room.area)),
            ];
            if let Some(height) = room.height {
                text.push(format!("Höhe {}", metres(height)));
            }
            let centre = self.plan(room.label_at);
            let top = (text.len() - 1) as f64 / 2.0;
            for (row, value) in text.into_iter().enumerate() {
                let off = (top - row as f64) * TEXT_LINE;
                let at = [centre[0] + up[0] * off, centre[1] + up[1] * off];
                self.add(ROOM, centred(at, along, value));
            }
        }
    }

    /// Line, a tick at both ends, text.
    fn dimensions(&mut self, plan: &FloorPlan) {
        for dim in &plan.dims {
            let (a, b) = (self.plan(dim.line_a), self.plan(dim.line_b));
            let Some(along) = unit(a, b) else {
                continue;
            };
            self.line(DIMENSION, a, b, false);
            let tick = [(along[0] - along[1]) * TICK, (along[0] + along[1]) * TICK];
            for p in [a, b] {
                let (from, to) = (
                    [p[0] - tick[0], p[1] - tick[1]],
                    [p[0] + tick[0], p[1] + tick[1]],
                );
                self.line(DIMENSION, from, to, false);
            }
            self.add(
                DIMENSION,
                beside(
                    middle(a, b),
                    along,
                    left_of(reading(along)),
                    dim.text.clone(),
                ),
            );
        }
    }

    /// Every object on its layer, in the order added.
    fn objects(&mut self, objects: &Objects, tags: &Tags) {
        for o in objects.all() {
            match o.kind {
                ObjectKind::Socket => self.point_object(o, SOCKET, o.kind.label(), &tags.socket),
                ObjectKind::Connection => {
                    let art = o.connection.map_or("", |c| c.label());
                    self.point_object(o, CONNECTION, art, &tags.connection);
                }
                ObjectKind::Cable => self.line_object(o, CABLE, &tags.line),
                ObjectKind::Pipe => self.line_object(o, PIPE, &tags.line),
                ObjectKind::Measure => self.measurement(o),
            }
        }
    }

    fn point_object(
        &mut self,
        o: &Object,
        block: &str,
        art: &str,
        tags: &BTreeMap<String, String>,
    ) {
        let [p] = o.points[..] else {
            return;
        };
        let height = (self.frame.to_plan(p)[2] * 1000.0).round() as i64;
        let mut attributes: Vec<(String, String)> = [
            ("NR", o.nr.to_string()),
            ("NAME", o.name.clone()),
            ("ART", art.to_string()),
            ("HOEHE", height.to_string()),
            ("NOTIZ", o.note.clone()),
        ]
        .map(|(tag, value)| (tag.to_string(), value))
        .into();
        attributes.extend(
            o.properties
                .iter()
                .filter_map(|(key, value)| Some((tags.get(key)?.clone(), value.clone()))),
        );
        let at = self.map(p);
        self.entities.push(Entity {
            layer: block.to_string(),
            kind: EntityKind::Insert {
                block: block.to_string(),
                at,
                attributes,
            },
            xdata: Vec::new(),
        });
    }

    fn line_object(&mut self, o: &Object, layer: &str, tags: &BTreeMap<String, String>) {
        let points: Vec<[f64; 2]> = o.points.iter().map(|p| self.map(*p)).collect();
        let mut xdata = vec![
            format!("NR={}", o.nr),
            format!("NAME={}", o.name),
            format!("NOTIZ={}", o.note),
            format!("QUELLE={}", o.source),
        ];
        xdata.extend(
            o.properties
                .iter()
                .filter_map(|(key, value)| Some(format!("{}={value}", tags.get(key)?))),
        );
        let mut text = title(o);
        if let Some(mm) = o.diameter_mm {
            xdata.push(format!("DURCHMESSER_MM={mm}"));
            text += &format!(" · Ø {} mm", format!("{mm}").replace('.', ","));
        }
        let longest = points.windows(2).max_by(|p, q| {
            let len = |s: &[[f64; 2]]| (s[1][0] - s[0][0]).hypot(s[1][1] - s[0][1]);
            len(p).total_cmp(&len(q))
        });
        let label = longest.map(|s| {
            let along = unit(s[0], s[1]).unwrap_or([1.0, 0.0]);
            beside(middle(s[0], s[1]), along, left_of(reading(along)), text)
        });
        self.entities.push(Entity {
            layer: layer.to_string(),
            kind: EntityKind::Shape(Shape::Polyline {
                points,
                closed: false,
                dashed: false,
            }),
            xdata,
        });
        if let Some(label) = label {
            self.add(layer, label);
        }
    }

    /// Line and text; the length is the measured one, also of a line not flat in the plan.
    fn measurement(&mut self, o: &Object) {
        let [p, q] = o.points[..] else {
            return;
        };
        let (a, b) = (self.map(p), self.map(q));
        self.line(MEASURE, a, b, false);
        let distance = (0..3).map(|i| (q[i] - p[i]).powi(2)).sum::<f64>().sqrt();
        let name = short_name(o);
        let text = if name.is_empty() {
            metres(distance)
        } else {
            format!("{name} · {}", metres(distance))
        };
        let along = unit(a, b).unwrap_or([1.0, 0.0]);
        let side = left_of(reading(along));
        self.add(MEASURE, beside(middle(a, b), along, side, text));
    }
}

/// The name of an object as far as it goes into a text of the plan.
fn short_name(o: &Object) -> String {
    let name = o.name.trim();
    if name.chars().count() <= MAX_NAME_IN_TEXT {
        return name.to_string();
    }
    let mut short: String = name.chars().take(MAX_NAME_IN_TEXT - 1).collect();
    short.push('…');
    short
}

/// „Rohr 3“, with a name „Rohr 3 · Heizung“.
fn title(o: &Object) -> String {
    let label = format!("{} {}", o.kind.label(), o.nr);
    match short_name(o).as_str() {
        "" => label,
        name => format!("{label} · {name}"),
    }
}

/// The tag for a key: capitals, umlauts written out, only `A-Z`, `0-9` and `_`.
fn tag_of(key: &str) -> String {
    let mut tag = String::new();
    for c in key.chars().flat_map(char::to_uppercase) {
        match c {
            'Ä' => tag.push_str("AE"),
            'Ö' => tag.push_str("OE"),
            'Ü' => tag.push_str("UE"),
            'A'..='Z' | '0'..='9' => tag.push(c),
            _ if tag.is_empty() || tag.ends_with('_') => {}
            _ => tag.push('_'),
        }
    }
    tag.truncate(MAX_TAG);
    match tag.trim_end_matches('_') {
        "" => UNNAMED_TAG.to_string(),
        tag => tag.to_string(),
    }
}

/// Key → tag of the free properties of `objects`, each tag once and none of `reserved`: a key
/// that would give a tag already taken gets a number. Sorted by key.
fn property_tags<'a>(
    objects: impl Iterator<Item = &'a Object>,
    reserved: &[&str],
) -> BTreeMap<String, String> {
    let keys: BTreeSet<&String> = objects.flat_map(|o| o.properties.keys()).collect();
    let mut taken: BTreeSet<String> = reserved.iter().map(|t| t.to_string()).collect();
    let mut tags = BTreeMap::new();
    for key in keys {
        let base = tag_of(key);
        let mut tag = base.clone();
        let mut number = 2;
        while !taken.insert(tag.clone()) {
            tag = format!("{base}_{number}");
            number += 1;
        }
        tags.insert(key.clone(), tag);
    }
    tags
}

/// The tags of the free properties, per block and for the cables and pipes.
struct Tags {
    socket: BTreeMap<String, String>,
    connection: BTreeMap<String, String>,
    line: BTreeMap<String, String>,
}

impl Tags {
    fn new(objects: &Objects) -> Tags {
        let of = |kinds: &[ObjectKind], reserved: &[&str]| {
            property_tags(
                objects.all().iter().filter(|o| kinds.contains(&o.kind)),
                reserved,
            )
        };
        Tags {
            socket: of(&[ObjectKind::Socket], &BLOCK_TAGS),
            connection: of(&[ObjectKind::Connection], &BLOCK_TAGS),
            line: of(&[ObjectKind::Cable, ObjectKind::Pipe], &LINE_TAGS),
        }
    }

    fn blocks(&self) -> Vec<Block> {
        vec![
            block(SOCKET, socket_symbol(), &self.socket),
            block(CONNECTION, connection_symbol(), &self.connection),
        ]
    }
}

fn block(name: &str, shapes: Vec<Shape>, tags: &BTreeMap<String, String>) -> Block {
    let mut attributes: Vec<String> = BLOCK_TAGS.iter().map(|t| t.to_string()).collect();
    attributes.extend(tags.values().cloned());
    Block {
        name: name.to_string(),
        shapes,
        attributes,
    }
}

fn line_shape(a: [f64; 2], b: [f64; 2]) -> Shape {
    Shape::Line {
        a,
        b,
        dashed: false,
    }
}

/// A circle with two contacts.
fn socket_symbol() -> Vec<Shape> {
    let pin = SYMBOL / 3.0;
    vec![
        Shape::Circle {
            centre: [0.0, 0.0],
            radius: SYMBOL,
        },
        line_shape([-pin, -pin], [-pin, pin]),
        line_shape([pin, -pin], [pin, pin]),
    ]
}

/// A square with its diagonals.
fn connection_symbol() -> Vec<Shape> {
    let s = SYMBOL;
    vec![
        Shape::Polyline {
            points: vec![[-s, -s], [s, -s], [s, s], [-s, s]],
            closed: true,
            dashed: false,
        },
        line_shape([-s, -s], [s, s]),
        line_shape([-s, s], [s, -s]),
    ]
}
