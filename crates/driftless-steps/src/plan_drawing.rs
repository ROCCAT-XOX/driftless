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
//! nothing is on it, one hidden in the program is written but off. Texts never lie on top of
//! each other: one that finds no free place near its line is left out.
//!
//! A door or window set by hand is drawn like one the plan found: near a wall of the plan it is
//! laid on that wall, cuts it (both faces of a wall with two) and takes the place of the
//! openings it overlaps; the hand wins. Near no wall it is drawn where it was set, as a line
//! with its text, so that nothing set is lost. The plan itself – rooms, dimensions, what was
//! found – stays as computed.

use crate::objects::{Object, ObjectKind, Objects};
use crate::plan::PlanReport;
use crate::PROGRAM;
use driftless_export::dxf::{Block, Drawing, Entity, EntityKind, Layer, Shape};
use driftless_plan::{metres, FloorPlan, Opening, OpeningKind, PlanFrame, Room, WallFace};
use std::borrow::Cow;
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
/// What the extended data of a cable, pipe, door or window always holds, which a free property
/// cannot be.
const LINE_TAGS: [&str; 8] = [
    "NR",
    "NAME",
    "ART",
    "HOEHE",
    "NOTIZ",
    "QUELLE",
    "DURCHMESSER_MM",
    "BRUESTUNG",
];
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
/// What an unclear opening says in the plan; the reason is in its extended data.
const UNCLEAR_TEXT: &str = "unklar";
/// Space kept around a text (mm).
const TEXT_MARGIN: f64 = 30.0;
/// Rows on each side of a line in which a text looks for a free place.
const TEXT_TRIES: usize = 4;
/// A piece of wall shorter than this is not drawn (metres).
const MIN_PIECE: f64 = 0.001;
/// Half a tick at the end of a dimension, per axis (mm): at 45° to the line, 100 mm long in all.
const TICK: f64 = 35.355;
/// Half the edge of the symbol of a block (mm).
const SYMBOL: f64 = 75.0;
/// How near a wall both edges of a door or window set by hand must lie to go on it (metres).
const HAND_REACH: f64 = 0.30;
/// The least of its width it must run along the wall, not across (a cosine, about 45°).
const HAND_ALONG: f64 = 0.7;
/// The least of its width on the wall, and not past its end.
const HAND_ON_WALL: f64 = 0.5;
/// An opening found gives way to one set by hand it overlaps by at least this (metres); less
/// is the inaccuracy of the clicks and leaves both.
const HAND_OVERLAP: f64 = 0.05;
/// How far under the floor or over the ceiling a door or window may lie and still go on a wall
/// of this storey (metres); further it is on another storey.
const HAND_STOREY_SLACK: f64 = 0.5;
/// The top of a storey without a flat ceiling, for that (metres).
const HAND_NO_CEILING: f64 = 4.0;

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
    let placed = place_openings(plan, objects);
    let mut sheet = Sheet {
        frame: &plan.frame,
        as_cloud: export.placement == PlanPlacement::AsCloud,
        entities: Vec::new(),
        taken: Vec::new(),
    };
    sheet.walls(&placed.plan);
    // The names of the rooms stand where they stand; the other texts make way for them.
    sheet.reserve_room_labels(plan);
    sheet.openings(&placed.plan, &placed.on_wall, &tags.line);
    sheet.rooms(plan);
    sheet.dimensions(plan);
    sheet.objects(objects, &tags);
    sheet.far_openings(&placed.far, &tags.line);
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

/// The doors and windows set by hand among the openings of a plan.
struct Placed<'a> {
    /// The plan with those near a wall among its openings, in place of the ones they overlap;
    /// the plan as computed without any.
    plan: Cow<'a, FloorPlan>,
    /// Index into the openings of [`Self::plan`] → the object that opening is.
    on_wall: BTreeMap<usize, &'a Object>,
    /// Those near no wall of this plan: drawn where they were set.
    far: Vec<&'a Object>,
}

/// Lays the doors and windows of `objects` on the walls of `plan`, as [`Placed`] says.
fn place_openings<'a>(plan: &'a FloorPlan, objects: &'a Objects) -> Placed<'a> {
    let mut near = Vec::new();
    let mut far = Vec::new();
    for object in objects.all().iter().filter(|o| o.kind.is_opening()) {
        match lay_on_wall(plan, object) {
            Some(opening) => near.push((object, opening)),
            None => far.push(object),
        }
    }
    if near.is_empty() {
        return Placed {
            plan: Cow::Borrowed(plan),
            on_wall: BTreeMap::new(),
            far,
        };
    }
    let mut merged = plan.clone();
    merged
        .openings
        .retain(|old| !near.iter().any(|(_, new)| overlaps(plan, old, new)));
    let mut on_wall = BTreeMap::new();
    for (object, opening) in near {
        on_wall.insert(merged.openings.len(), object);
        merged.openings.push(opening);
    }
    Placed {
        plan: Cow::Owned(merged),
        on_wall,
        far,
    }
}

/// `object` as an opening of the wall it lies at: the nearest face both its edges are within
/// [`HAND_REACH`] of, along it and on it, on this storey. `None` if there is none.
fn lay_on_wall(plan: &FloorPlan, object: &Object) -> Option<Opening> {
    let [p, q] = object.points[..] else {
        return None;
    };
    let (p, q) = (plan.frame.to_plan(p), plan.frame.to_plan(q));
    let z = (p[2] + q[2]) / 2.0;
    let top = plan
        .storey
        .ceiling_z
        .map_or(HAND_NO_CEILING, |ceiling| ceiling - plan.storey.floor_z);
    if z < -HAND_STOREY_SLACK || z > top + HAND_STOREY_SLACK {
        return None;
    }
    let (p, q) = ([p[0], p[1]], [q[0], q[1]]);
    let width = (q[0] - p[0]).hypot(q[1] - p[1]);
    let mut nearest: Option<(f64, usize, f64, f64)> = None;
    for (index, face) in plan.faces.iter().enumerate() {
        let Some(wall) = Wall::of(face) else {
            continue;
        };
        let (t0, t1) = (wall.along(p), wall.along(q));
        let (from, to) = (t0.min(t1), t0.max(t1));
        let distance = wall.distance(p).max(wall.distance(q));
        let on_wall = to.min(wall.len) - from.max(0.0);
        if distance <= HAND_REACH
            && to - from >= HAND_ALONG * width
            && on_wall >= HAND_ON_WALL * (to - from)
            && nearest.is_none_or(|n| distance < n.0)
        {
            nearest = Some((distance, index, from, to));
        }
    }
    let (_, face, from, to) = nearest?;
    let (kind, sill) = match object.kind {
        ObjectKind::Door => (OpeningKind::Door, 0.0),
        _ => (OpeningKind::Window, object.sill_m.unwrap_or(0.0)),
    };
    Some(Opening {
        kind,
        face,
        from,
        to,
        sill,
        head: sill + object.height_m.unwrap_or(0.0),
        crossed: false,
        note: String::new(),
    })
}

/// Whether the opening `new` set by hand covers the opening `old` of the plan: on the same face
/// or the other face of the wall, along it by at least [`HAND_OVERLAP`].
fn overlaps(plan: &FloorPlan, old: &Opening, new: &Opening) -> bool {
    let paired = old.face == new.face
        || plan.pairs.iter().any(|pair| {
            (pair.front, pair.back) == (old.face, new.face)
                || (pair.back, pair.front) == (old.face, new.face)
        });
    if !paired {
        return false;
    }
    let (Some(wall), Some(other)) = (
        plan.faces.get(new.face).and_then(Wall::of),
        plan.faces.get(old.face).and_then(Wall::of),
    ) else {
        return false;
    };
    let (t0, t1) = (
        wall.along(other.at(old.from, 0.0)),
        wall.along(other.at(old.to, 0.0)),
    );
    t0.max(t1).min(new.to) - t0.min(t1).max(new.from) >= HAND_OVERLAP
}

/// How many doors, windows and unclear openings the plan shows with these objects: those it
/// found, without the ones a door or window set by hand replaces, and the ones set by hand.
pub fn opening_counts(plan: &FloorPlan, objects: &Objects) -> [usize; 3] {
    let placed = place_openings(plan, objects);
    let drawn: &FloorPlan = &placed.plan;
    let distinct = drawn.distinct_openings();
    let found = |kind: OpeningKind| {
        distinct
            .iter()
            .filter(|&&i| drawn.openings[i].kind == kind)
            .count()
    };
    let far = |kind: ObjectKind| placed.far.iter().filter(|o| o.kind == kind).count();
    [
        found(OpeningKind::Door) + far(ObjectKind::Door),
        found(OpeningKind::Window) + far(ObjectKind::Window),
        found(OpeningKind::Unclear),
    ]
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

/// The lines of the label of a room.
fn room_text(room: &Room) -> Vec<String> {
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
    text
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

    /// How far off the line of this face the point `p` lies.
    fn distance(&self, p: [f64; 2]) -> f64 {
        ((p[0] - self.a[0]) * self.dir[1] - (p[1] - self.a[1]) * self.dir[0]).abs()
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

/// The doors and windows of the faces paired with `index`, on this face: where a partner has
/// one that this face does not know of. Unclear openings stay on their own face.
fn partner_gaps(plan: &FloorPlan, index: usize, wall: &Wall) -> Vec<(f64, f64)> {
    let mut gaps = Vec::new();
    for pair in &plan.pairs {
        let partner = match (pair.front == index, pair.back == index) {
            (true, _) => pair.back,
            (_, true) => pair.front,
            _ => continue,
        };
        let Some(other) = plan.faces.get(partner).and_then(Wall::of) else {
            continue;
        };
        for o in plan
            .openings
            .iter()
            .filter(|o| o.face == partner && o.kind != OpeningKind::Unclear)
        {
            let (t0, t1) = (
                wall.along(other.at(o.from, 0.0)),
                wall.along(other.at(o.to, 0.0)),
            );
            let (from, to) = (t0.min(t1).max(0.0), t0.max(t1).min(wall.len));
            if to > from {
                gaps.push((from, to));
            }
        }
    }
    gaps
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
    /// Rectangles (min x, min y, max x, max y) of the texts placed so far, in the drawing (mm).
    taken: Vec<[f64; 4]>,
}

/// The rectangle a text covers, in the drawing (mm), with a small margin.
fn text_box(centre: [f64; 2], along: [f64; 2], value: &str) -> [f64; 4] {
    let half = CHAR_WIDTH * TEXT_HEIGHT * value.chars().count() as f64 / 2.0 + TEXT_MARGIN;
    let up = left_of(along);
    let high = TEXT_HEIGHT / 2.0 + TEXT_MARGIN;
    let reach = [
        along[0].abs() * half + up[0].abs() * high,
        along[1].abs() * half + up[1].abs() * high,
    ];
    [
        centre[0] - reach[0],
        centre[1] - reach[1],
        centre[0] + reach[0],
        centre[1] + reach[1],
    ]
}

fn boxes_meet(a: &[f64; 4], b: &[f64; 4]) -> bool {
    a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3]
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

    /// Takes the place of a text, if it is free.
    fn claim(&mut self, centre: [f64; 2], along: [f64; 2], value: &str) -> bool {
        let wanted = text_box(centre, along, value);
        if self.taken.iter().any(|t| boxes_meet(t, &wanted)) {
            return false;
        }
        self.taken.push(wanted);
        true
    }

    /// A text beside the middle of a line, on the side `side` or the other one and further off
    /// until it meets no other text. `None` if there is no free place: no text is better than
    /// two on top of each other.
    fn free_beside(
        &mut self,
        mid: [f64; 2],
        along: [f64; 2],
        side: [f64; 2],
        value: String,
    ) -> Option<Shape> {
        let read = reading(along);
        let up = left_of(read);
        let first = if up[0] * side[0] + up[1] * side[1] >= 0.0 {
            1.0
        } else {
            -1.0
        };
        for row in 0..TEXT_TRIES {
            for sign in [first, -first] {
                let off = sign * (TEXT_GAP + TEXT_HEIGHT / 2.0 + row as f64 * TEXT_LINE);
                let centre = [mid[0] + up[0] * off, mid[1] + up[1] * off];
                if self.claim(centre, read, &value) {
                    return Some(centred(centre, read, value));
                }
            }
        }
        None
    }

    /// Reserves the places of the names of the rooms.
    fn reserve_room_labels(&mut self, plan: &FloorPlan) {
        let along =
            reading(unit(self.plan([0.0, 0.0]), self.plan([1.0, 0.0])).unwrap_or([1.0, 0.0]));
        let up = left_of(along);
        for room in &plan.rooms {
            let text = room_text(room);
            let centre = self.plan(room.label_at);
            let top = (text.len() - 1) as f64 / 2.0;
            for (row, value) in text.iter().enumerate() {
                let off = (top - row as f64) * TEXT_LINE;
                let at = [centre[0] + up[0] * off, centre[1] + up[1] * off];
                self.taken.push(text_box(at, along, value));
            }
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

    /// A line with extended data.
    fn line_with(
        &mut self,
        layer: &str,
        a: [f64; 2],
        b: [f64; 2],
        dashed: bool,
        xdata: Vec<String>,
    ) {
        self.entities.push(Entity {
            layer: layer.to_string(),
            kind: EntityKind::Shape(Shape::Line { a, b, dashed }),
            xdata,
        });
    }

    /// The faces without their openings. Both entries of an opening in a wall with two faces
    /// cut their face; a door or window found from one side only cuts the other face as well.
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
            gaps.extend(partner_gaps(plan, index, &wall));
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

    /// End caps, symbol and text, once per real opening. One set by hand (`hand` by the index
    /// of the opening) keeps the width it was set with, also past the end of the wall, and has
    /// its own text and extended data.
    fn openings(
        &mut self,
        plan: &FloorPlan,
        hand: &BTreeMap<usize, &Object>,
        tags: &BTreeMap<String, String>,
    ) {
        for index in plan.distinct_openings() {
            let o = &plan.openings[index];
            let Some(wall) = plan.faces.get(o.face).and_then(Wall::of) else {
                continue;
            };
            let object = hand.get(&index).copied();
            let (from, to) = match object {
                Some(_) => (o.from, o.to),
                None => (o.from.max(0.0), o.to.min(wall.len)),
            };
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
            let (hand_text, xdata) = match object {
                Some(object) => (
                    Some(opening_text(object, to - from)),
                    opening_xdata(object, tags),
                ),
                None => (None, Vec::new()),
            };
            let (layer, text) = match o.kind {
                OpeningKind::Door => {
                    self.line_with(DOOR, a, b, true, xdata);
                    let text = hand_text.unwrap_or_else(|| format!("Tür {}", size(to - from, o)));
                    (DOOR, text)
                }
                OpeningKind::Window => {
                    for side in [-0.5, 0.5] {
                        let at = depth / 2.0 + side * WINDOW_LINES;
                        let (p, q) = (self.plan(wall.at(from, at)), self.plan(wall.at(to, at)));
                        self.line_with(WINDOW, p, q, false, xdata.clone());
                    }
                    let brh = metres(o.sill);
                    let found = || format!("Fenster {}, BRH {brh}", size(to - from, o));
                    (WINDOW, hand_text.unwrap_or_else(found))
                }
                OpeningKind::Unclear => {
                    // The reason stands in the extended data, the plan only says „unklar“.
                    self.entities.push(Entity {
                        layer: UNCLEAR.to_string(),
                        kind: EntityKind::Shape(Shape::Line { a, b, dashed: true }),
                        xdata: vec![format!("GRUND={}", o.note)],
                    });
                    (UNCLEAR, UNCLEAR_TEXT.to_string())
                }
            };
            let Some(along) = unit(a, b) else {
                continue;
            };
            let (centre, room) = (self.plan(wall.at(mid, 0.0)), self.plan(wall.at(mid, -1.0)));
            let side = unit(centre, room).unwrap_or(left_of(along));
            if let Some(label) = self.free_beside(middle(a, b), along, side, text) {
                self.add(layer, label);
            }
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
            let text = room_text(room);
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
            let side = left_of(reading(along));
            if let Some(label) = self.free_beside(middle(a, b), along, side, dim.text.clone()) {
                self.add(DIMENSION, label);
            }
        }
    }

    /// The doors and windows set by hand near no wall: a line where they were set, with their
    /// text and data, in the plan and in both placements.
    fn far_openings(&mut self, far: &[&Object], tags: &BTreeMap<String, String>) {
        for o in far {
            let [p, q] = o.points[..] else {
                continue;
            };
            let (a, b) = (self.map(p), self.map(q));
            let (pp, qp) = (self.frame.to_plan(p), self.frame.to_plan(q));
            let width = (qp[0] - pp[0]).hypot(qp[1] - pp[1]);
            let layer = if o.kind == ObjectKind::Door {
                DOOR
            } else {
                WINDOW
            };
            let along = unit(a, b).unwrap_or([1.0, 0.0]);
            let side = left_of(reading(along));
            let text = opening_text(o, width);
            let label = self.free_beside(middle(a, b), along, side, text);
            self.entities.push(Entity {
                layer: layer.to_string(),
                kind: EntityKind::Shape(Shape::Polyline {
                    points: vec![a, b],
                    closed: false,
                    dashed: false,
                }),
                xdata: opening_xdata(o, tags),
            });
            if let Some(label) = label {
                self.add(layer, label);
            }
        }
    }

    /// Every object on its layer, in the order added. Doors and windows are not here: those on
    /// a wall are among the openings, the others are [`Self::far_openings`].
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
                ObjectKind::Door | ObjectKind::Window => {}
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
        // Height above the floor (mm), as a span where the line rises or falls.
        let heights: Vec<i64> = o
            .points
            .iter()
            .map(|p| (self.frame.to_plan(*p)[2] * 1000.0).round() as i64)
            .collect();
        let (low, high) = (
            heights.iter().min().copied().unwrap_or(0),
            heights.iter().max().copied().unwrap_or(0),
        );
        let height = if low == high {
            low.to_string()
        } else {
            format!("{low}..{high}")
        };
        let mut xdata = vec![
            format!("NR={}", o.nr),
            format!("NAME={}", o.name),
            format!("ART={}", o.kind.label()),
            format!("HOEHE={height}"),
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

/// The text of a door or window set by hand, of width `width` (metres): „Tür 0,90 × 2,00 m“,
/// „Fenster 1,20 m“ without a known height, the sill of a window as „BRH 0,90 m“ and a name
/// after them, as with a measurement.
fn opening_text(o: &Object, width: f64) -> String {
    let mut text = match o.height_m {
        Some(height) => format!("{} {} × {}", o.kind.label(), number(width), metres(height)),
        None => format!("{} {}", o.kind.label(), metres(width)),
    };
    if let Some(sill) = o.sill_m.filter(|_| o.kind == ObjectKind::Window) {
        text += &format!(", BRH {}", metres(sill));
    }
    let name = short_name(o);
    if !name.is_empty() {
        text += &format!(" · {name}");
    }
    text
}

/// The extended data of a door or window set by hand (heights in millimetres, empty if not
/// known), as those of a cable.
fn opening_xdata(o: &Object, tags: &BTreeMap<String, String>) -> Vec<String> {
    let mm = |metres: f64| ((metres * 1000.0).round() as i64).to_string();
    let mut xdata = vec![
        format!("NR={}", o.nr),
        format!("NAME={}", o.name),
        format!("ART={}", o.kind.label()),
        format!("HOEHE={}", o.height_m.map_or(String::new(), mm)),
        format!("NOTIZ={}", o.note),
        format!("QUELLE={}", o.source),
    ];
    xdata.extend(
        o.properties
            .iter()
            .filter_map(|(key, value)| Some(format!("{}={value}", tags.get(key)?))),
    );
    if let Some(sill) = o.sill_m.filter(|_| o.kind == ObjectKind::Window) {
        xdata.push(format!("BRUESTUNG={}", mm(sill)));
    }
    xdata
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

/// The tags of the free properties, per block and for the cables, pipes, doors and windows.
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
            line: of(
                &[
                    ObjectKind::Cable,
                    ObjectKind::Pipe,
                    ObjectKind::Door,
                    ObjectKind::Window,
                ],
                &LINE_TAGS,
            ),
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
