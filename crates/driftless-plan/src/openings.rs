//! Openings: doors, windows and what cannot be told.
//!
//! Every wall face is looked at from the front: an image of 4 cm cells, length × height, from
//! the points directly on it. A hole in the image can be an opening, or something stands in
//! front of it. That is decided by the line of sight from the trajectory ([`SightGrid`]): where
//! the view is free and still no point lies, the wall really is open. Where the view is blocked
//! the wall simply runs on. What the program cannot be sure about is reported as unclear.
//!
//! An opening has wall above it. A gap that is open up to the ceiling, between two walls that
//! only happen to lie in line, is no opening at all: the face is cut there, and what is on both
//! sides of it stays one room.

use driftless_io::progress::Progress;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::levels::check;
use crate::sight::SightGrid;
use crate::walls::cut_faces;
use crate::{PlanError, StoreyPoints, WallFace, Walls};

/// Edge of a cell of the wall image (metres).
const CELL: f64 = 0.04;
/// Points this close to the face are on it (metres).
const BAND: f64 = 0.03;
/// A cell is empty below this share of the median of the occupied cells.
const EMPTY_SHARE: f64 = 0.15;
/// Rows at the floor and at the ceiling that hold the points of those surfaces: not looked at.
const EDGE_ROWS: usize = 2;
/// Without a ceiling the image ends here (metres), as the wall band does.
const NO_CEILING_TOP: f64 = 2.2;
/// A hole needs this many cells each way to be looked at.
const MIN_HOLE_CELLS: u32 = 5;
/// Bigger holes are tested on a coarser lattice: cells per hole at most.
const MAX_TESTED_CELLS: f64 = 4096.0;

/// Line of sight: poses at most, thinned to this much way between them (metres).
const MAX_POSES: usize = 2000;
const POSE_SPACING: f64 = 0.5;
/// Longest line of sight (metres).
const MAX_SIGHT: f64 = 12.0;
/// cos 75°: the steepest angle between the line of sight and the normal of the face.
const MIN_COS: f64 = 0.258_819_045_102_520_74;
/// The line of sight ends this far in front of the wall (metres) ...
const WALL_CLEARANCE: f64 = 0.08;
/// ... and leaves out this much at its start, where the scanner and its carrier are.
const SIGHT_START: f64 = 0.15;
/// A hole, a row or a column is open from this share of its cells with a free view.
const OPEN_SHARE: f64 = 0.5;

/// A step of the trajectory longer than this is a jump, not a walk through a wall (metres).
const MAX_WALK_STEP: f64 = 1.0;
/// A crossing counts this close beside an opening (metres).
const CROSSING_MARGIN: f64 = 0.05;

/// A door: lower edge at most, height at least, width (metres).
const DOOR_SILL: f64 = 0.10;
const DOOR_HEIGHT: f64 = 1.7;
const DOOR_WIDTH: (f64, f64) = (0.55, 2.6);
/// Points count from this far to this far behind a door (metres).
const BEHIND: (f64, f64) = (0.10, 3.0);
/// Lines through the opening that have to meet a point behind it.
const MIN_BEHIND_LINES: u32 = 16;
/// Lattice of those lines and step along them; distance to the sides of the opening (metres).
const BEHIND_STEP: f64 = 0.05;
const BEHIND_DEPTH_STEP: f64 = 0.025;
const BEHIND_SIDE: f64 = 0.08;

/// A window or an unclear opening is at least this big each way (metres).
const MIN_SIZE: f64 = 0.25;
/// A window starts at least this high and has wall below on this share of its width.
const WINDOW_SILL: f64 = 0.20;
const WALL_BELOW: f64 = 0.5;
/// A door or a window fills at least this share of its bounding rectangle.
const MIN_FILL: f64 = 0.6;
/// A hole at the ceiling over this share of the face is the end of the wall.
const WALL_END_SHARE: f64 = 0.9;
/// Wall over an opening of a storey without a ceiling: on this share of its width.
const LINTEL_SHARE: f64 = 0.5;

/// A closed door: depth behind the face, height and width (metres), filled share of the
/// rectangle; a cell is behind the face when at most this share of its points is on it.
const RECESS: (f64, f64) = (0.015, 0.10);
const CLOSED_HEIGHT: (f64, f64) = (1.8, 2.4);
const CLOSED_WIDTH: (f64, f64) = (0.6, 1.3);
const CLOSED_FILL: f64 = 0.7;
const RECESS_ON_FACE: f64 = 0.3;

/// Reach of the raster for the line of sight below and above the storey (metres).
const RASTER_BELOW: f64 = 0.15;
const RASTER_ABOVE: f64 = 0.15;
/// Without a ceiling the points of a storey end here (metres).
const NO_CEILING_POINTS: f64 = 3.5;
/// Edge of the bins that say which faces a point can belong to (metres).
const BIN: f64 = 0.5;

const OPEN_TO_FLOOR: &str = "Bis zum Boden offen, nichts dahinter: Tür oder bodentiefes Fenster?";
const CLOSED_DOOR: &str = "Tür geschlossen?";
const NEITHER: &str = "Öffnung in Größe oder Form weder Tür noch Fenster";
const HIDDEN_BELOW: &str = "Unten verdeckt: Fenster oder Tür?";

/// What an opening is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OpeningKind {
    Door,
    Window,
    Unclear,
}

/// An opening in a wall face.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Opening {
    pub kind: OpeningKind,
    /// Index into [`Walls::faces`].
    pub face: usize,
    /// Metres from `a` of the face.
    pub from: f64,
    pub to: f64,
    /// Metres above the floor.
    pub sill: f64,
    pub head: f64,
    /// The trajectory went through it.
    pub crossed: bool,
    /// In German; for [`OpeningKind::Unclear`] the reason.
    pub note: String,
}

/// Finds the openings of all wall faces, sorted by face and position along it. A face that is
/// open up to the ceiling in a stretch is cut there (in `walls`): it runs on neither as a door
/// nor as a border of the rooms.
pub fn find_openings(
    points: &StoreyPoints,
    walls: &mut Walls,
    progress: &Progress,
) -> Result<Vec<Opening>, PlanError> {
    progress.phase("Öffnungen", Some(walls.faces.len() as u64 + 2));
    check(progress)?;
    if points.poses.is_empty() {
        return Err(PlanError::NoPoses);
    }
    let top = points.height.map_or(NO_CEILING_TOP, |h| h as f64);
    let planes: Vec<Option<Plane>> = walls.faces.iter().map(Plane::new).collect();
    let images = wall_images(points, &planes, top);
    progress.advance(1);
    check(progress)?;

    let ctx = Context {
        poses: thinned_poses(&points.poses),
        walk: &points.poses,
        grid: sight_grid(points, walls),
        above_image: points.height.is_none().then_some(NO_CEILING_POINTS),
    };
    progress.advance(1);
    check(progress)?;

    let per_face: Vec<FaceResult> = planes
        .par_iter()
        .zip(images.par_iter())
        .enumerate()
        .map(|(index, (plane, image))| {
            check(progress)?;
            let found = match plane {
                Some(plane) => FaceWork::new(index, *plane, image, &ctx).openings(),
                None => FaceResult::default(),
            };
            progress.advance(1);
            Ok(found)
        })
        .collect::<Result<_, PlanError>>()?;

    let passages: Vec<Vec<(f64, f64)>> = per_face.iter().map(|f| f.passages.clone()).collect();
    let mut found: Vec<Opening> = per_face.into_iter().flat_map(|f| f.openings).collect();
    let cut = cut_faces(walls, &passages);
    found.retain_mut(|o| cut.place(o));
    found.sort_by(|a, b| {
        a.face
            .cmp(&b.face)
            .then(a.from.total_cmp(&b.from))
            .then(a.to.total_cmp(&b.to))
            .then(a.sill.total_cmp(&b.sill))
    });
    Ok(found)
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn median(values: &mut [f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let middle = values.len() / 2;
    Some(*values.select_nth_unstable_by(middle, f64::total_cmp).1)
}

/// A face as a plane: `u` along it from `a`, `s` towards the room.
#[derive(Debug, Clone, Copy)]
struct Plane {
    a: [f64; 2],
    d: [f64; 2],
    n: [f64; 2],
    len: f64,
}

impl Plane {
    fn new(face: &WallFace) -> Option<Plane> {
        let d = [face.b[0] - face.a[0], face.b[1] - face.a[1]];
        let len = d[0].hypot(d[1]);
        (len > 0.0).then(|| Plane {
            a: face.a,
            d: [d[0] / len, d[1] / len],
            n: face.inward,
            len,
        })
    }

    fn u(&self, p: [f64; 2]) -> f64 {
        dot(self.d, [p[0] - self.a[0], p[1] - self.a[1]])
    }

    fn s(&self, p: [f64; 2]) -> f64 {
        dot(self.n, [p[0] - self.a[0], p[1] - self.a[1]])
    }

    fn point(&self, u: f64, s: f64, z: f64) -> [f64; 3] {
        [
            self.a[0] + self.d[0] * u + self.n[0] * s,
            self.a[1] + self.d[1] * u + self.n[1] * s,
            z,
        ]
    }

    /// Distance from `p` to the segment of the face.
    fn distance(&self, p: [f64; 2]) -> f64 {
        let u = self.u(p).clamp(0.0, self.len);
        let q = self.point(u, 0.0, 0.0);
        (p[0] - q[0]).hypot(p[1] - q[1])
    }
}

/// What a cell of a wall image holds.
#[derive(Debug, Clone, Copy)]
struct ImageCell {
    /// Points on the face (within `BAND`).
    on: u32,
    /// Points from `RECESS.1` behind the face to `BAND` in front of it ...
    near: u32,
    /// ... and how many of them are not behind it.
    level: u32,
    /// Extent of the points on the face.
    u_min: f32,
    u_max: f32,
    z_min: f32,
    z_max: f32,
}

impl ImageCell {
    const EMPTY: ImageCell = ImageCell {
        on: 0,
        near: 0,
        level: 0,
        u_min: f32::INFINITY,
        u_max: f32::NEG_INFINITY,
        z_min: f32::INFINITY,
        z_max: f32::NEG_INFINITY,
    };
}

/// The view of a face from the front. Row 0 is at the floor.
#[derive(Debug)]
struct WallImage {
    cols: usize,
    rows: usize,
    top: f64,
    cells: Vec<ImageCell>,
}

impl WallImage {
    fn new(len: f64, top: f64) -> WallImage {
        let cols = ((len / CELL).ceil() as usize).max(1);
        let rows = ((top / CELL).round() as usize).max(1);
        WallImage {
            cols,
            rows,
            top,
            cells: vec![ImageCell::EMPTY; cols * rows],
        }
    }

    fn at(&self, col: usize, row: usize) -> &ImageCell {
        &self.cells[row * self.cols + col]
    }

    fn add(&mut self, u: f64, s: f64, z: f64) {
        let (col, row) = ((u / CELL) as usize, (z / CELL) as usize);
        if col >= self.cols || row >= self.rows {
            return;
        }
        let cell = &mut self.cells[row * self.cols + col];
        cell.near += 1;
        if s >= -RECESS.0 {
            cell.level += 1;
        }
        if s.abs() <= BAND {
            cell.on += 1;
            cell.u_min = cell.u_min.min(u as f32);
            cell.u_max = cell.u_max.max(u as f32);
            cell.z_min = cell.z_min.min(z as f32);
            cell.z_max = cell.z_max.max(z as f32);
        }
    }

    /// The rows that are looked at.
    fn measured(&self) -> std::ops::Range<usize> {
        EDGE_ROWS..self.rows.saturating_sub(EDGE_ROWS)
    }

    /// Points below this count mean empty; `None` if the face has no points at all.
    fn threshold(&self) -> Option<f64> {
        let mut counts: Vec<f64> = self
            .measured()
            .flat_map(|row| (0..self.cols).map(move |col| (col, row)))
            .map(|(col, row)| self.at(col, row).on as f64)
            .filter(|&on| on > 0.0)
            .collect();
        median(&mut counts).map(|m| m * EMPTY_SHARE)
    }
}

/// Bins of the top view that say which faces a point can be near.
struct FaceBins {
    min: [f64; 2],
    nx: usize,
    ny: usize,
    bins: Vec<Vec<u32>>,
}

impl FaceBins {
    fn new(planes: &[Option<Plane>]) -> Option<FaceBins> {
        let margin = RECESS.1 + BIN;
        let ends = planes
            .iter()
            .flatten()
            .flat_map(|p| [p.point(0.0, 0.0, 0.0), p.point(p.len, 0.0, 0.0)]);
        let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
        for e in ends {
            for i in 0..2 {
                min[i] = min[i].min(e[i] - margin);
                max[i] = max[i].max(e[i] + margin);
            }
        }
        if min[0] > max[0] {
            return None;
        }
        let nx = ((max[0] - min[0]) / BIN).ceil() as usize;
        let ny = ((max[1] - min[1]) / BIN).ceil() as usize;
        let mut bins = vec![Vec::new(); nx * ny];
        let reach = RECESS.1 + BIN * std::f64::consts::SQRT_2 / 2.0;
        for (index, plane) in planes.iter().enumerate() {
            let Some(plane) = plane else { continue };
            let (a, b) = (plane.point(0.0, 0.0, 0.0), plane.point(plane.len, 0.0, 0.0));
            let lo = |i: usize| (a[i].min(b[i]) - margin - min[i]) / BIN;
            let hi = |i: usize| (a[i].max(b[i]) + margin - min[i]) / BIN;
            let x0 = lo(0).floor().max(0.0) as usize;
            let y0 = lo(1).floor().max(0.0) as usize;
            for y in y0..(hi(1).ceil() as usize).min(ny) {
                for x in x0..(hi(0).ceil() as usize).min(nx) {
                    let centre = [
                        min[0] + (x as f64 + 0.5) * BIN,
                        min[1] + (y as f64 + 0.5) * BIN,
                    ];
                    if plane.distance(centre) <= reach {
                        bins[y * nx + x].push(index as u32);
                    }
                }
            }
        }
        Some(FaceBins { min, nx, ny, bins })
    }

    fn faces_at(&self, x: f64, y: f64) -> &[u32] {
        let cx = ((x - self.min[0]) / BIN).floor();
        let cy = ((y - self.min[1]) / BIN).floor();
        if cx >= 0.0 && cy >= 0.0 && cx < self.nx as f64 && cy < self.ny as f64 {
            &self.bins[cy as usize * self.nx + cx as usize]
        } else {
            &[]
        }
    }
}

/// The wall images of all faces (an empty image for a face of no length).
fn wall_images(points: &StoreyPoints, planes: &[Option<Plane>], top: f64) -> Vec<WallImage> {
    let mut images: Vec<WallImage> = planes
        .iter()
        .map(|p| WallImage::new(p.map_or(0.0, |p| p.len), top))
        .collect();
    let Some(bins) = FaceBins::new(planes) else {
        return images;
    };
    for p in &points.xyz {
        let z = p[2] as f64;
        if !(z >= 0.0 && z < top) {
            continue;
        }
        let xy = [p[0] as f64, p[1] as f64];
        for &face in bins.faces_at(xy[0], xy[1]) {
            let Some(plane) = planes[face as usize] else {
                continue;
            };
            let s = plane.s(xy);
            if !(-RECESS.1..=BAND).contains(&s) {
                continue;
            }
            let u = plane.u(xy);
            if (0.0..plane.len).contains(&u) {
                images[face as usize].add(u, s, z);
            }
        }
    }
    images
}

/// The poses at least `POSE_SPACING` of way apart, then evenly down to `MAX_POSES`.
fn thinned_poses(poses: &[[f32; 3]]) -> Vec<[f64; 3]> {
    let mut kept: Vec<[f64; 3]> = Vec::new();
    let mut way = 0.0;
    let mut last: Option<[f64; 3]> = None;
    for pose in poses {
        let p = [pose[0] as f64, pose[1] as f64, pose[2] as f64];
        match last {
            None => kept.push(p),
            Some(q) => {
                let d = [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
                way += (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                if way >= POSE_SPACING {
                    kept.push(p);
                    way = 0.0;
                }
            }
        }
        last = Some(p);
    }
    if kept.len() > MAX_POSES {
        let n = kept.len();
        kept = (0..MAX_POSES).map(|i| kept[i * n / MAX_POSES]).collect();
    }
    kept
}

/// The raster reaches `MAX_SIGHT` around the faces: no line of sight leaves that.
fn sight_grid(points: &StoreyPoints, walls: &Walls) -> SightGrid {
    let (mut low, mut high) = ([f64::MAX; 3], [f64::MIN; 3]);
    for end in walls.faces.iter().flat_map(|f| [f.a, f.b]) {
        for i in 0..2 {
            low[i] = low[i].min(end[i] - MAX_SIGHT - 0.5);
            high[i] = high[i].max(end[i] + MAX_SIGHT + 0.5);
        }
    }
    low[2] = -RASTER_BELOW;
    high[2] = points.height.map_or(NO_CEILING_POINTS, |h| h as f64) + RASTER_ABOVE;
    SightGrid::build(&points.xyz, low, high)
}

/// What all faces share.
struct Context<'a> {
    poses: Vec<[f64; 3]>,
    walk: &'a [[f32; 3]],
    grid: SightGrid,
    /// Without a ceiling the image ends low: how far above it to look for wall (metres).
    above_image: Option<f64>,
}

/// What one face yields.
#[derive(Debug, Default)]
struct FaceResult {
    openings: Vec<Opening>,
    /// Stretches that are open up to the ceiling (metres along the face), to cut the face at.
    passages: Vec<(f64, f64)>,
}

/// A connected group of cells of a wall image.
#[derive(Debug, Clone)]
struct Region {
    /// (column, row), row by row.
    cells: Vec<(u32, u32)>,
    cols: (u32, u32),
    rows: (u32, u32),
}

impl Region {
    fn new(cells: Vec<(u32, u32)>) -> Option<Region> {
        let first = *cells.first()?;
        let (mut cols, mut rows) = ((first.0, first.0), (first.1, first.1));
        for &(c, r) in &cells {
            cols = (cols.0.min(c), cols.1.max(c));
            rows = (rows.0.min(r), rows.1.max(r));
        }
        Some(Region { cells, cols, rows })
    }

    fn col_count(&self) -> u32 {
        self.cols.1 - self.cols.0 + 1
    }

    fn row_count(&self) -> u32 {
        self.rows.1 - self.rows.0 + 1
    }

    /// The share of the bounding rectangle that the region fills.
    fn fill(&self) -> f64 {
        self.cells.len() as f64 / (self.col_count() as f64 * self.row_count() as f64)
    }
}

/// Closes the mask by one cell: it fills gaps up to two cells wide and keeps the outline.
fn close(mask: &[bool], cols: usize, rows: usize, outside: bool) -> Vec<bool> {
    let spread = |source: &[bool], any: bool| -> Vec<bool> {
        let mut out = vec![false; source.len()];
        for r in 0..rows {
            for c in 0..cols {
                let test = |dc: isize, dr: isize| {
                    let (x, y) = (c as isize + dc, r as isize + dr);
                    if x < 0 || y < 0 || x >= cols as isize || y >= rows as isize {
                        outside
                    } else {
                        source[y as usize * cols + x as usize]
                    }
                };
                let mut hits = (-1..=1).flat_map(|dr| (-1..=1).map(move |dc| (dc, dr)));
                out[r * cols + c] = if any {
                    hits.any(|(dc, dr)| test(dc, dr))
                } else {
                    hits.all(|(dc, dr)| test(dc, dr))
                };
            }
        }
        out
    };
    spread(&spread(mask, true), false)
}

/// The connected groups (four neighbours) of the set cells in `measured` rows.
fn regions(mask: &[bool], cols: usize, measured: std::ops::Range<usize>) -> Vec<Region> {
    let mut seen = vec![false; mask.len()];
    let mut found = Vec::new();
    let mut stack = Vec::new();
    for row in measured.clone() {
        for col in 0..cols {
            if !mask[row * cols + col] || seen[row * cols + col] {
                continue;
            }
            seen[row * cols + col] = true;
            stack.push((col, row));
            let mut cells = Vec::new();
            while let Some((c, r)) = stack.pop() {
                cells.push((c as u32, r as u32));
                let mut visit = |x: usize, y: usize| {
                    if x < cols
                        && measured.contains(&y)
                        && mask[y * cols + x]
                        && !seen[y * cols + x]
                    {
                        seen[y * cols + x] = true;
                        stack.push((x, y));
                    }
                };
                visit(c.wrapping_sub(1), r);
                visit(c + 1, r);
                visit(c, r.wrapping_sub(1));
                visit(c, r + 1);
            }
            cells.sort_unstable_by_key(|&(c, r)| (r, c));
            found.extend(Region::new(cells));
        }
    }
    found
}

/// Where an opening is: along the face and in height (metres).
#[derive(Debug, Clone, Copy)]
struct Extent {
    from: f64,
    to: f64,
    sill: f64,
    head: f64,
}

impl Extent {
    fn width(&self) -> f64 {
        self.to - self.from
    }

    fn height(&self) -> f64 {
        self.head - self.sill
    }

    fn door_sized(&self) -> bool {
        self.sill <= DOOR_SILL
            && self.height() >= DOOR_HEIGHT
            && (DOOR_WIDTH.0..=DOOR_WIDTH.1).contains(&self.width())
    }
}

/// What was measured below an opening.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Below {
    /// Wall, on half of its width at least.
    Wall,
    /// No wall, but something stands in front of it.
    Hidden,
    /// Neither.
    Open,
}

/// The work on one face.
struct FaceWork<'a> {
    index: usize,
    plane: Plane,
    image: &'a WallImage,
    ctx: &'a Context<'a>,
    crossings: Vec<f64>,
}

impl<'a> FaceWork<'a> {
    fn new(index: usize, plane: Plane, image: &'a WallImage, ctx: &'a Context<'a>) -> Self {
        FaceWork {
            index,
            plane,
            image,
            ctx,
            crossings: crossings(&plane, ctx.walk),
        }
    }

    fn opening(&self, kind: OpeningKind, at: Extent, note: &str) -> Opening {
        Opening {
            kind,
            face: self.index,
            from: at.from,
            to: at.to,
            sill: at.sill,
            head: at.head,
            crossed: self.crossed(at),
            note: note.to_string(),
        }
    }

    fn crossed(&self, at: Extent) -> bool {
        self.crossings
            .iter()
            .any(|&u| u >= at.from - CROSSING_MARGIN && u <= at.to + CROSSING_MARGIN)
    }

    fn openings(&self) -> FaceResult {
        let image = self.image;
        let Some(threshold) = image.threshold() else {
            return FaceResult::default();
        };
        let (cols, rows) = (image.cols, image.rows);
        let measured = image.measured();
        let mut solid: Vec<bool> = (0..rows)
            .flat_map(|row| (0..cols).map(move |col| (col, row)))
            .map(|(col, row)| {
                !measured.contains(&row) || {
                    let on = image.at(col, row).on;
                    on > 0 && on as f64 >= threshold
                }
            })
            .collect();
        solid = close(&solid, cols, rows, true);

        let mut found = FaceResult::default();
        for door in self.closed_doors(threshold) {
            for r in door.rows.0..=door.rows.1 {
                for c in door.cols.0..=door.cols.1 {
                    solid[r as usize * cols + c as usize] = true;
                }
            }
            let at = self.measure(&door.cells);
            let kind = if self.crossed(at) {
                OpeningKind::Door
            } else {
                OpeningKind::Unclear
            };
            let note = if kind == OpeningKind::Door {
                ""
            } else {
                CLOSED_DOOR
            };
            found.openings.push(self.opening(kind, at, note));
        }

        let empty: Vec<bool> = solid.iter().map(|s| !s).collect();
        for hole in regions(&empty, cols, measured) {
            if hole.col_count() >= MIN_HOLE_CELLS && hole.row_count() >= MIN_HOLE_CELLS {
                self.classify(&hole, &solid, &mut found);
            }
        }
        found
    }

    /// Rectangles of door size that lie a little behind the face.
    fn closed_doors(&self, threshold: f64) -> Vec<Region> {
        let image = self.image;
        let (cols, rows) = (image.cols, image.rows);
        let measured = image.measured();
        let behind: Vec<bool> = (0..rows)
            .flat_map(|row| (0..cols).map(move |col| (col, row)))
            .map(|(col, row)| {
                let cell = image.at(col, row);
                measured.contains(&row)
                    && cell.near > 0
                    && cell.near as f64 >= threshold
                    && cell.level as f64 <= cell.near as f64 * RECESS_ON_FACE
            })
            .collect();
        let behind = close(&behind, cols, rows, false);
        regions(&behind, cols, measured)
            .into_iter()
            .filter(|region| {
                let at = self.measure(&region.cells);
                region.fill() >= CLOSED_FILL
                    && at.sill <= DOOR_SILL
                    && (CLOSED_HEIGHT.0..=CLOSED_HEIGHT.1).contains(&at.height())
                    && (CLOSED_WIDTH.0..=CLOSED_WIDTH.1).contains(&at.width())
            })
            .collect()
    }

    /// Whether no wall was measured over `at`, up to the ceiling.
    fn open_to_top(&self, at: Extent) -> bool {
        at.head >= self.image.top && !self.wall_above_image(at)
    }

    /// Whether wall was measured above the top of the image: that is a storey without a ceiling,
    /// where the image ends low and a door can reach its top.
    fn wall_above_image(&self, at: Extent) -> bool {
        let Some(end) = self.ctx.above_image else {
            return false;
        };
        let columns = ((at.to - at.from) / BEHIND_STEP).floor() as u32;
        let levels = ((end - self.image.top) / BEHIND_STEP).ceil() as u32;
        let walled = (0..columns)
            .filter(|&i| {
                let u = at.from + (i as f64 + 0.5) * BEHIND_STEP;
                (0..levels).any(|k| {
                    let z = self.image.top + (k as f64 + 0.5) * BEHIND_STEP;
                    [-0.02, 0.0, 0.02]
                        .iter()
                        .any(|&s| self.ctx.grid.occupied(self.plane.point(u, s, z)))
                })
            })
            .count();
        columns > 0 && walled as f64 >= LINTEL_SHARE * columns as f64
    }

    /// A gap one can walk through, without wall above it: the space between two walls.
    fn is_passage(&self, at: Extent) -> bool {
        at.sill <= DOOR_SILL && at.width() >= DOOR_WIDTH.0 && self.open_to_top(at)
    }

    /// Decides what a hole in the image is.
    fn classify(&self, hole: &Region, solid: &[bool], found: &mut FaceResult) {
        let whole = self.measure(&hole.cells);
        if whole.head >= self.image.top && whole.width() >= WALL_END_SHARE * self.plane.len {
            return;
        }
        // The walk went through it and there is wall above it: a door, however much of it is
        // hidden. With no wall above it, it may be the space between two walls: see below.
        if whole.door_sized() && self.crossed(whole) && !self.open_to_top(whole) {
            found
                .openings
                .push(self.opening(OpeningKind::Door, whole, ""));
            return;
        }
        let free = self.free_cells(hole);
        for part in open_parts(hole, &free) {
            let at = self.measure(&part.cells);
            if at.width() < MIN_SIZE || at.height() < MIN_SIZE {
                continue;
            }
            let rectangular = part.fill() >= MIN_FILL;
            let reached = self.crossed(at) || self.has_points_behind(at);
            // Open up to the ceiling and one can get through: the wall stops here.
            if rectangular && reached && self.is_passage(at) {
                found.passages.push((at.from, at.to));
                continue;
            }
            let below = self.below(&part, hole, &free, solid);
            let (kind, note) = if rectangular && at.door_sized() && reached {
                (OpeningKind::Door, "")
            } else if rectangular && at.sill >= WINDOW_SILL && below == Below::Wall {
                (OpeningKind::Window, "")
            } else if rectangular && at.sill >= WINDOW_SILL && below == Below::Hidden {
                // Something stands in front of its lower part: a parapet may be there or not.
                (OpeningKind::Unclear, HIDDEN_BELOW)
            } else if rectangular && at.door_sized() {
                (OpeningKind::Unclear, OPEN_TO_FLOOR)
            } else {
                (OpeningKind::Unclear, NEITHER)
            };
            found.openings.push(self.opening(kind, at, note));
        }
    }

    /// For every cell of the hole whether a pose has a free view of it.
    fn free_cells(&self, hole: &Region) -> Vec<bool> {
        let poses = self.poses_for(hole);
        let stride = ((hole.cells.len() as f64 / MAX_TESTED_CELLS).sqrt().ceil() as u32).max(1);
        let macro_cols = (hole.col_count().div_ceil(stride)) as usize;
        let macro_rows = (hole.row_count().div_ceil(stride)) as usize;
        let mut cache: Vec<Option<bool>> = vec![None; macro_cols * macro_rows];
        hole.cells
            .iter()
            .map(|&(c, r)| {
                let key = ((c - hole.cols.0) / stride) as usize
                    + macro_cols * ((r - hole.rows.0) / stride) as usize;
                *cache[key].get_or_insert_with(|| self.cell_is_free(c, r, &poses))
            })
            .collect()
    }

    /// The poses on the room side that can reach the hole, nearest first.
    fn poses_for(&self, hole: &Region) -> Vec<[f64; 3]> {
        let mid_u = (hole.cols.0 + hole.cols.1 + 1) as f64 / 2.0 * CELL;
        let mid_z = (hole.rows.0 + hole.rows.1 + 1) as f64 / 2.0 * CELL;
        let centre = self.plane.point(mid_u, 0.0, mid_z);
        let half_diagonal =
            (hole.col_count() as f64 * CELL).hypot(hole.row_count() as f64 * CELL) / 2.0;
        let mut near: Vec<(f64, usize)> = self
            .ctx
            .poses
            .iter()
            .enumerate()
            .filter(|(_, p)| self.plane.s([p[0], p[1]]) > WALL_CLEARANCE + SIGHT_START)
            .filter_map(|(i, p)| {
                let d = [p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]];
                let distance = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                (distance <= MAX_SIGHT + half_diagonal).then_some((distance, i))
            })
            .collect();
        near.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        near.iter().map(|&(_, i)| self.ctx.poses[i]).collect()
    }

    fn cell_is_free(&self, col: u32, row: u32, poses: &[[f64; 3]]) -> bool {
        let u = ((col as f64 + 0.5) * CELL).min(self.plane.len);
        let z = (row as f64 + 0.5) * CELL;
        let target = self.plane.point(u, 0.0, z);
        poses.iter().any(|p| {
            let s = self.plane.s([p[0], p[1]]);
            let d = [p[0] - target[0], p[1] - target[1], p[2] - target[2]];
            let distance = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            if distance > MAX_SIGHT || s < MIN_COS * distance {
                return false;
            }
            let t = WALL_CLEARANCE / s;
            let end = [
                target[0] + d[0] * t,
                target[1] + d[1] * t,
                target[2] + d[2] * t,
            ];
            self.ctx.grid.free(*p, end, SIGHT_START)
        })
    }

    /// Whether points lie behind the opening.
    fn has_points_behind(&self, at: Extent) -> bool {
        let from = at.from + BEHIND_SIDE;
        let to = at.to - BEHIND_SIDE;
        let along = ((to - from) / BEHIND_STEP).floor().max(0.0) as u32;
        let up = (at.height() / BEHIND_STEP).floor() as u32;
        let depths = ((BEHIND.1 - BEHIND.0) / BEHIND_DEPTH_STEP).floor() as u32;
        let mut hits = 0;
        for i in 0..=along {
            let u = from + i as f64 * BEHIND_STEP;
            for j in 0..up {
                let z = at.sill + (j as f64 + 0.5) * BEHIND_STEP;
                let hit = (0..=depths).any(|k| {
                    let depth = BEHIND.0 + k as f64 * BEHIND_DEPTH_STEP;
                    self.ctx.grid.occupied(self.plane.point(u, -depth, z))
                });
                if hit {
                    hits += 1;
                    if hits >= MIN_BEHIND_LINES {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// What is measured below the part of the hole, on half of its width at least.
    fn below(&self, part: &Region, hole: &Region, free: &[bool], solid: &[bool]) -> Below {
        let image = self.image;
        let lowest = EDGE_ROWS as u32;
        let width = part.col_count() as usize;
        let mut wall = vec![false; width];
        for c in part.cols.0..=part.cols.1 {
            wall[(c - part.cols.0) as usize] =
                (lowest..part.rows.0).any(|r| solid[r as usize * image.cols + c as usize]);
        }
        let mut hidden = wall.clone();
        for (&(c, r), &open) in hole.cells.iter().zip(free) {
            if !open && r < part.rows.0 && (part.cols.0..=part.cols.1).contains(&c) {
                hidden[(c - part.cols.0) as usize] = true;
            }
        }
        let enough = |marks: &[bool]| {
            marks.iter().filter(|&&s| s).count() as f64 >= WALL_BELOW * width as f64
        };
        if enough(&wall) {
            Below::Wall
        } else if enough(&hidden) {
            Below::Hidden
        } else {
            Below::Open
        }
    }

    /// The extent of a group of cells, taken from the points around it.
    fn measure(&self, cells: &[(u32, u32)]) -> Extent {
        let image = self.image;
        let first = cells.first().copied().unwrap_or((0, 0));
        let (mut cmin, mut cmax, mut rmin, mut rmax) = (first.0, first.0, first.1, first.1);
        for &(c, r) in cells {
            cmin = cmin.min(c);
            cmax = cmax.max(c);
            rmin = rmin.min(r);
            rmax = rmax.max(r);
        }
        let nr = (rmax - rmin + 1) as usize;
        let nc = (cmax - cmin + 1) as usize;
        let mut row_ends = vec![(u32::MAX, 0u32); nr];
        let mut col_ends = vec![(u32::MAX, 0u32); nc];
        for &(c, r) in cells {
            let e = &mut row_ends[(r - rmin) as usize];
            *e = (e.0.min(c), e.1.max(c));
            let e = &mut col_ends[(c - cmin) as usize];
            *e = (e.0.min(r), e.1.max(r));
        }

        let mut lefts = Vec::new();
        let mut rights = Vec::new();
        for (i, &(first, last)) in row_ends.iter().enumerate() {
            if first == u32::MAX {
                continue;
            }
            let row = rmin as usize + i;
            if first > 0 && image.at(first as usize - 1, row).on > 0 {
                lefts.push(image.at(first as usize - 1, row).u_max as f64);
            }
            if (last as usize + 1) < image.cols && image.at(last as usize + 1, row).on > 0 {
                rights.push(image.at(last as usize + 1, row).u_min as f64);
            }
        }
        let mut below = Vec::new();
        let mut above = Vec::new();
        for (i, &(first, last)) in col_ends.iter().enumerate() {
            if first == u32::MAX {
                continue;
            }
            let col = cmin as usize + i;
            if first > 0 && image.at(col, first as usize - 1).on > 0 {
                below.push(image.at(col, first as usize - 1).z_max as f64);
            }
            if (last as usize + 1) < image.rows && image.at(col, last as usize + 1).on > 0 {
                above.push(image.at(col, last as usize + 1).z_min as f64);
            }
        }

        let measured = image.measured();
        let touches_floor = rmin as usize <= measured.start;
        let touches_top = rmax as usize + 1 >= measured.end;
        let from = median(&mut lefts).unwrap_or(cmin as f64 * CELL).max(0.0);
        let to = median(&mut rights)
            .unwrap_or((cmax + 1) as f64 * CELL)
            .min(self.plane.len);
        let sill = if touches_floor {
            0.0
        } else {
            median(&mut below).unwrap_or(rmin as f64 * CELL)
        };
        let head = if touches_top {
            image.top
        } else {
            median(&mut above).unwrap_or((rmax + 1) as f64 * CELL)
        };
        Extent {
            from,
            to,
            sill,
            head,
        }
    }
}

/// The parts of a hole that are open: columns with a free view, cut to the rows with one.
fn open_parts(hole: &Region, free: &[bool]) -> Vec<Region> {
    let col_of = |c: u32| (c - hole.cols.0) as usize;
    let mut per_col = vec![(0u32, 0u32); hole.col_count() as usize];
    for (&(c, _), &open) in hole.cells.iter().zip(free) {
        per_col[col_of(c)].0 += 1;
        per_col[col_of(c)].1 += u32::from(open);
    }
    let open_col =
        |c: usize| per_col[c].0 > 0 && per_col[c].1 as f64 >= OPEN_SHARE * per_col[c].0 as f64;

    let mut parts = Vec::new();
    let mut c = 0;
    while c < per_col.len() {
        if !open_col(c) {
            c += 1;
            continue;
        }
        let start = c;
        while c < per_col.len() && open_col(c) {
            c += 1;
        }
        let (first, last) = (hole.cols.0 + start as u32, hole.cols.0 + c as u32 - 1);
        parts.extend(open_rows(hole, free, first, last));
    }
    parts
}

/// The cells of the columns `first..=last`, without the blocked rows at the bottom and top.
fn open_rows(hole: &Region, free: &[bool], first: u32, last: u32) -> Option<Region> {
    let row_of = |r: u32| (r - hole.rows.0) as usize;
    let in_columns = |c: u32| (first..=last).contains(&c);
    let mut per_row = vec![(0u32, 0u32); hole.row_count() as usize];
    for (&(c, r), &open) in hole.cells.iter().zip(free) {
        if in_columns(c) {
            per_row[row_of(r)].0 += 1;
            per_row[row_of(r)].1 += u32::from(open);
        }
    }
    let open_row = |r: &(u32, u32)| r.0 > 0 && r.1 as f64 >= OPEN_SHARE * r.0 as f64;
    let low = hole.rows.0 + per_row.iter().position(open_row)? as u32;
    let high = hole.rows.0 + per_row.iter().rposition(open_row)? as u32;

    let (mut cells, mut open) = (Vec::new(), 0usize);
    for (&(c, r), &is_open) in hole.cells.iter().zip(free) {
        if in_columns(c) && (low..=high).contains(&r) {
            cells.push((c, r));
            open += usize::from(is_open);
        }
    }
    (open as f64 >= OPEN_SHARE * cells.len() as f64)
        .then(|| Region::new(cells))
        .flatten()
}

/// Where the trajectory goes through the plane of the face, as positions along it.
fn crossings(plane: &Plane, walk: &[[f32; 3]]) -> Vec<f64> {
    let flat = |p: &[f32; 3]| [p[0] as f64, p[1] as f64];
    walk.windows(2)
        .filter_map(|pair| {
            let (p, q) = (flat(&pair[0]), flat(&pair[1]));
            let (sp, sq) = (plane.s(p), plane.s(q));
            if (sp > 0.0) == (sq > 0.0) || (q[0] - p[0]).hypot(q[1] - p[1]) > MAX_WALK_STEP {
                return None;
            }
            let t = sp / (sp - sq);
            Some(plane.u([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::Scene;
    use crate::{find_storeys, find_walls, storey_points, Leveling, PlanConfig};

    const FLOOR: f64 = -1.3;
    const WIDE: f64 = 8.3;
    const DEEP: f64 = 7.9;
    const OPEN_TO_FLOOR: &str =
        "Bis zum Boden offen, nichts dahinter: Tür oder bodentiefes Fenster?";
    const CLOSED: &str = "Tür geschlossen?";
    const WRONG_SHAPE: &str = "Öffnung in Größe oder Form weder Tür noch Fenster";

    fn walk() -> [[f64; 3]; 5] {
        [
            [1.0, 1.0, 0.0],
            [7.0, 1.0, 0.0],
            [7.0, 6.5, 0.0],
            [1.0, 6.5, 0.0],
            [1.0, 1.0, 0.0],
        ]
    }

    fn room() -> Scene {
        Scene::new()
            .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
            .walk(&walk())
    }

    /// A far wall behind wall 0, seen through its openings.
    fn back_wall(scene: Scene) -> Scene {
        scene.behind([1.0, -2.5, FLOOR], [7.0, -2.5, FLOOR + 2.6])
    }

    fn storey_of(scene: Scene) -> StoreyPoints {
        let data = scene.build();
        let input = data.input();
        let found = find_storeys(&input, &Progress::new()).unwrap();
        storey_points(&input, &found.storeys[0], &Leveling::NONE)
    }

    fn openings_of(scene: Scene) -> Vec<Opening> {
        let points = storey_of(scene);
        let mut walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        find_openings(&points, &mut walls, &Progress::new()).unwrap()
    }

    fn of_kind(openings: &[Opening], kind: OpeningKind) -> Vec<&Opening> {
        openings.iter().filter(|o| o.kind == kind).collect()
    }

    fn width(o: &Opening) -> f64 {
        o.to - o.from
    }

    fn assert_near(what: &str, got: f64, expected: f64, tol: f64) {
        assert!(
            (got - expected).abs() <= tol,
            "{what}: {got} statt {expected}"
        );
    }

    #[test]
    fn door_found() {
        let found = openings_of(back_wall(room().door(0, 3.0, 0.90, 2.05)));
        assert_eq!(found.len(), 1, "{found:?}");
        let door = &found[0];
        assert_eq!(door.kind, OpeningKind::Door, "{door:?}");
        assert_near("Breite", width(door), 0.90, 0.03);
        assert_near("Von", door.from, 3.0, 0.03);
        assert_near("Höhe", door.head - door.sill, 2.05, 0.03);
        assert!(door.sill <= 0.03, "{door:?}");
        assert!(!door.crossed, "{door:?}");
    }

    #[test]
    fn walkthrough_makes_a_door() {
        let scene = room().door(0, 3.0, 0.90, 2.05).walk(&[
            [3.45, 1.0, 0.0],
            [3.45, -1.5, 0.0],
            [3.45, 1.0, 0.0],
        ]);
        let found = openings_of(scene);
        let doors = of_kind(&found, OpeningKind::Door);
        assert_eq!(doors.len(), 1, "{found:?}");
        assert!(doors[0].crossed, "{:?}", doors[0]);
    }

    #[test]
    fn window_found() {
        let found = openings_of(room().window(0, 3.0, 1.20, 0.90, 1.30));
        assert_eq!(found.len(), 1, "{found:?}");
        let window = &found[0];
        assert_eq!(window.kind, OpeningKind::Window, "{window:?}");
        assert_near("Breite", width(window), 1.20, 0.03);
        assert_near("Brüstung", window.sill, 0.90, 0.03);
        assert_near("Sturz", window.head, 2.20, 0.03);
    }

    #[test]
    fn cabinet_is_no_door() {
        let points = storey_of(room().block([3.0, 0.0, FLOOR], [4.0, 0.6, FLOOR + 2.1]));
        let mut walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        let found = find_openings(&points, &mut walls, &Progress::new()).unwrap();
        assert!(found.is_empty(), "{found:?}");
        // Wall 0 still runs through behind the cabinet.
        let y = -points.origin[1];
        assert!(walls.faces.iter().any(|f| {
            (f.a[1] - y).abs() < 0.02
                && (f.b[1] - y).abs() < 0.02
                && (f.b[0] - f.a[0]).abs() > WIDE - 0.05
        }));
    }

    #[test]
    fn open_to_floor_alone_is_unclear() {
        let found = openings_of(room().door(0, 3.0, 0.90, 2.05));
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Unclear);
        assert_eq!(found[0].note, OPEN_TO_FLOOR);
    }

    #[test]
    fn closed_door_is_unclear() {
        let found = openings_of(room().closed_door(0, 3.0, 0.90, 2.05, 0.04));
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Unclear);
        assert_eq!(found[0].note, CLOSED);
        assert_near("Breite", width(&found[0]), 0.90, 0.05);
    }

    #[test]
    fn very_wide_gap_is_unclear() {
        let found = openings_of(room().door(0, 2.5, 3.0, 2.2));
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Unclear);
        assert_eq!(found[0].note, WRONG_SHAPE);
        assert_near("Breite", width(&found[0]), 3.0, 0.05);
    }

    #[test]
    fn small_hole_is_nothing() {
        let found = openings_of(room().window(0, 3.0, 0.15, 1.0, 0.15));
        assert!(found.is_empty(), "{found:?}");
    }

    #[test]
    fn two_openings_in_order() {
        let found = openings_of(back_wall(
            room().window(0, 5.5, 1.0, 0.9, 1.2).door(0, 1.5, 0.9, 2.05),
        ));
        assert_eq!(found.len(), 2, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Door);
        assert_eq!(found[1].kind, OpeningKind::Window);
        assert_eq!(found[0].face, found[1].face);
        assert!(found[0].from < found[1].from);
    }

    #[test]
    fn noise_keeps_the_window() {
        let found = openings_of(room().window(0, 3.0, 1.20, 0.90, 1.30).noise(0.005));
        let windows = of_kind(&found, OpeningKind::Window);
        assert_eq!(windows.len(), 1, "{found:?}");
        assert_near("Breite", width(windows[0]), 1.20, 0.05);
        assert_near("Brüstung", windows[0].sill, 0.90, 0.05);
    }

    #[test]
    fn same_input_same_result() {
        let scene = || {
            back_wall(room().door(0, 3.0, 0.9, 2.05))
                .noise(0.005)
                .spacing(0.02)
        };
        assert_eq!(openings_of(scene()), openings_of(scene()));
    }

    #[test]
    fn rotated_room_keeps_door_and_window() {
        let found = openings_of(
            back_wall(room().door(0, 1.5, 0.9, 2.05).window(0, 5.0, 1.2, 0.9, 1.3)).yaw(23.0),
        );
        assert_eq!(found.len(), 2, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Door, "{found:?}");
        assert_eq!(found[1].kind, OpeningKind::Window, "{found:?}");
        assert_near("Tür", width(&found[0]), 0.9, 0.04);
        assert_near("Fenster", width(&found[1]), 1.2, 0.04);
    }

    #[test]
    fn door_at_a_corner_is_found() {
        let found = openings_of(
            room()
                .door(0, 0.10, 0.9, 2.05)
                .behind([-0.5, -2.5, FLOOR], [2.5, -2.5, FLOOR + 2.6]),
        );
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Door, "{found:?}");
        assert_near("Breite", width(&found[0]), 0.9, 0.04);
    }

    #[test]
    fn cupboard_beside_a_door_does_not_widen_it() {
        let found = openings_of(back_wall(
            room()
                .door(0, 3.0, 0.9, 2.05)
                .block([3.9, 0.0, FLOOR], [5.4, 0.5, FLOOR + 2.1]),
        ));
        let doors = of_kind(&found, OpeningKind::Door);
        assert_eq!(doors.len(), 1, "{found:?}");
        assert_near("Von", doors[0].from, 3.0, 0.04);
        assert!(doors[0].to < 4.0, "{:?}", doors[0]);
    }

    #[test]
    fn window_above_a_sideboard_is_a_window() {
        let found = openings_of(
            room()
                .window(0, 3.0, 1.2, 0.9, 1.3)
                .block([2.8, 0.0, FLOOR], [4.4, 0.4, FLOOR + 0.85]),
        );
        let windows = of_kind(&found, OpeningKind::Window);
        assert_eq!(windows.len(), 1, "{found:?}");
        assert_near("Breite", width(windows[0]), 1.2, 0.05);
        assert_near("Brüstung", windows[0].sill, 0.9, 0.08);
    }

    #[test]
    fn a_door_behind_a_low_sideboard_is_unclear_not_a_window() {
        // An unwalked door, a sideboard 12 cm in front of it, no parapet measured.
        let scene = Scene::new()
            .spacing(0.02)
            .room([0.0, 0.0], [8.0, 6.0], FLOOR, 2.6)
            .door(0, 3.0, 0.9, 2.05)
            .block([2.7, 0.12, FLOOR], [4.2, 0.6, FLOOR + 0.9])
            .behind([1.0, -2.5, FLOOR], [7.0, -2.5, FLOOR + 2.6])
            .walk(&[
                [1.0, 2.0, 0.0],
                [7.0, 2.0, 0.0],
                [7.0, 5.0, 0.0],
                [1.0, 5.0, 0.0],
                [1.0, 2.0, 0.0],
            ]);
        let found = openings_of(scene);
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Unclear, "{found:?}");
        assert_eq!(found[0].note, HIDDEN_BELOW, "{found:?}");
    }

    #[test]
    fn walking_through_a_blocked_door_is_enough() {
        let scene = room()
            .door(0, 3.0, 0.9, 2.05)
            .block([3.0, 0.3, FLOOR], [3.9, 0.8, FLOOR + 1.6])
            .walk(&[[3.45, 1.2, 0.0], [3.45, -1.5, 0.0]]);
        let found = openings_of(scene);
        let doors = of_kind(&found, OpeningKind::Door);
        assert_eq!(doors.len(), 1, "{found:?}");
        assert!(doors[0].crossed);
        assert_near("Breite", width(doors[0]), 0.9, 0.04);
    }

    #[test]
    fn slightly_recessed_door_is_seen_too() {
        let found = openings_of(room().closed_door(0, 3.0, 0.9, 2.05, 0.02));
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].note, CLOSED);
    }

    #[test]
    fn a_closed_door_walked_through_is_a_door() {
        let scene = room()
            .closed_door(0, 3.0, 0.9, 2.05, 0.04)
            .walk(&[[3.45, 1.2, 0.0], [3.45, -1.5, 0.0]]);
        let found = openings_of(scene);
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].kind, OpeningKind::Door);
        assert!(found[0].crossed);
    }

    #[test]
    fn hidden_recess_in_a_thin_wall_is_no_door() {
        // Two sheets 8 cm apart: the back one lies in the depth of a recess, but the face is there.
        let found = openings_of(room().behind([3.0, -0.08, FLOOR], [3.9, -0.08, FLOOR + 2.05]));
        assert!(found.is_empty(), "{found:?}");
    }

    #[test]
    fn a_wall_ending_below_the_ceiling_is_no_opening() {
        // Free-standing: its top edge is 40 cm below the ceiling on the whole length.
        let found = openings_of(room().block([2.0, 3.0, FLOOR], [6.0, 3.1, FLOOR + 2.2]));
        assert!(found.is_empty(), "{found:?}");
    }

    #[test]
    fn room_without_ceiling_keeps_its_door() {
        let found = openings_of(back_wall(room().open_top().door(0, 3.0, 0.9, 2.0)));
        let doors = of_kind(&found, OpeningKind::Door);
        assert_eq!(doors.len(), 1, "{found:?}");
    }

    #[test]
    fn door_in_a_shared_wall_is_on_both_faces() {
        let scene = Scene::new()
            .room([0.0, 0.0], [4.0, 4.0], FLOOR, 2.6)
            .door(1, 1.5, 0.9, 2.05)
            .room([4.2, 0.0], [8.2, 4.0], FLOOR, 2.6)
            .door(3, 1.6, 0.9, 2.05)
            .walk(&[
                [1.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
                [3.0, 1.95, 0.0],
                [5.2, 1.95, 0.0],
                [7.0, 1.95, 0.0],
                [7.0, 3.0, 0.0],
            ]);
        let found = openings_of(scene);
        let doors = of_kind(&found, OpeningKind::Door);
        assert_eq!(doors.len(), 2, "{found:?}");
        assert_ne!(doors[0].face, doors[1].face);
        assert!(doors.iter().all(|d| d.crossed), "{found:?}");
    }

    #[test]
    fn no_poses_no_openings() {
        let mut points = storey_of(room().spacing(0.05));
        let mut walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        points.poses.clear();
        assert_eq!(
            find_openings(&points, &mut walls, &Progress::new()),
            Err(PlanError::NoPoses)
        );
    }

    #[test]
    fn cancel_stops() {
        let points = storey_of(room().spacing(0.05));
        let mut walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        let progress = Progress::new();
        progress.cancel();
        assert_eq!(
            find_openings(&points, &mut walls, &progress),
            Err(PlanError::Cancelled)
        );
    }
}
