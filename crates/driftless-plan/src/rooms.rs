//! Rooms: the floor the trajectory walked on, cut at the walls.
//!
//! On a grid of 4 cm a cell is free where floor was measured. Furniture hides the floor, so
//! gaps between measured floor, and between floor and wall, are closed. Wall faces (they run
//! on through doors and unclear openings) and the inside of a wall with two faces are the
//! borders. A fill from the poses gives one area per room. Its outline is simplified and laid
//! on the wall lines, and the corners are the crossings of those lines. Everything here is in
//! the frame of the plan.

use driftless_io::progress::Progress;
use serde::{Deserialize, Serialize};

use crate::grid::{robust_extent, Band};
use crate::levels::check;
use crate::{PlanError, StoreyPoints, WallFace, WallPair};

/// Edge of a cell of the room grid (metres).
const CELL: f64 = 0.04;
/// Points this close to the floor height are floor (metres).
const FLOOR_BAND: f64 = 0.04;
/// Furniture: gaps up to this width are closed (metres).
const CLOSE_GAP: f64 = 1.5;
/// A room has at least this area (m²).
const MIN_AREA: f64 = 1.0;
/// Tolerance of the simplified outline (metres).
const SIMPLIFY: f64 = 0.05;
/// An outline edge lies on a parallel wall this close (metres).
const WALL_REACH: f64 = 0.12;
/// Outline edges shorter than this are dropped: steps and cut corners (metres).
const MIN_STEP: f64 = 0.10;
/// A room is open when more than this share of its perimeter has no wall.
const OPEN_SHARE: f64 = 0.30;
/// A room can take a wall line if they overlap this much (metres), or by half the edge.
const MIN_OVERLAP: f64 = 0.30;
/// Longest side of the grid; beyond it the cell grows.
const MAX_SIDE: usize = 8192;
/// Empty cells around the measured area: wider than the closing reaches, which otherwise
/// would run out of the grid and not come back.
const MARGIN_CELLS: f64 = 21.0;
/// sin 15°: an outline edge runs along a wall up to this angle.
const ALONG_SIN: f64 = 0.258_819_045_102_520_74;
/// sin 3°: neighbouring outline edges closer in direction have no useful crossing.
const SAME_DIRECTION_SIN: f64 = 0.052_335_956_242_943_83;
/// A corner stays this close to the outline on the grid (metres).
const CORNER_REACH: f64 = 0.30;
/// Ceiling of a room: points from this share of the storey height, up to this far above it.
const CEILING_FROM: f64 = 0.75;
const CEILING_ABOVE: f64 = 0.10;
const HEIGHT_BIN: f64 = 0.01;
/// The height of a room needs this many ceiling points per cell, else the storey's counts.
const CEILING_POINTS_PER_CELL: f64 = 0.2;

/// A room of the plan: outline counterclockwise, starting at its smallest x, then y.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Room {
    pub nr: u32,
    pub outline: Vec<[f64; 2]>,
    /// m²
    pub area: f64,
    /// Clear height, if the storey has a ceiling.
    pub height: Option<f64>,
    /// More than 30 % of the perimeter has no wall.
    pub open: bool,
    pub label_at: [f64; 2],
}

/// Finds the rooms of a storey. The points and the faces are in the frame of the plan.
pub(crate) fn find_rooms(
    points: &StoreyPoints,
    faces: &[WallFace],
    pairs: &[WallPair],
    progress: &Progress,
) -> Result<Vec<Room>, PlanError> {
    let Some(raster) = Raster::new(points, faces) else {
        return Ok(Vec::new());
    };
    let walls = raster.walls(faces, pairs);
    let mut solid = raster.floor(points);
    for (cell, wall) in solid.iter_mut().zip(&walls) {
        *cell |= *wall;
    }
    check(progress)?;
    let radius = (CLOSE_GAP / 2.0 / raster.cell).ceil() as usize;
    let closed = close(&solid, raster.nx, raster.ny, radius);
    let free: Vec<bool> = closed.iter().zip(&walls).map(|(c, w)| *c && !*w).collect();
    check(progress)?;

    let (label, regions) = flood(&raster, &free, &points.poses);
    let heights = room_heights(points, &raster, &label, &regions);
    let mut paired = vec![false; faces.len()];
    for pair in pairs {
        paired[pair.front] = true;
        paired[pair.back] = true;
    }

    let mut rooms = Vec::new();
    for (region, height) in regions.iter().zip(heights) {
        check(progress)?;
        if region.cells as f64 * raster.cell * raster.cell >= MIN_AREA {
            rooms.extend(room_of(&raster, &label, region, faces, &paired, height));
        }
    }
    number_rooms(&mut rooms);
    Ok(rooms)
}

/// By area, the largest first; on a tie by the smallest x, then y.
fn number_rooms(rooms: &mut [Room]) {
    rooms.sort_by(|a, b| {
        b.area
            .total_cmp(&a.area)
            .then(a.outline[0][0].total_cmp(&b.outline[0][0]))
            .then(a.outline[0][1].total_cmp(&b.outline[0][1]))
    });
    for (index, room) in rooms.iter_mut().enumerate() {
        room.nr = index as u32 + 1;
    }
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn along(p: [f64; 2], d: [f64; 2], t: f64) -> [f64; 2] {
    [p[0] + d[0] * t, p[1] + d[1] * t]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn middle(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

/// Direction and length of the way from `a` to `b`; `None` without a length.
fn direction(a: [f64; 2], b: [f64; 2]) -> Option<([f64; 2], f64)> {
    let len = distance(a, b);
    (len > 1e-9).then(|| ([(b[0] - a[0]) / len, (b[1] - a[1]) / len], len))
}

/// The grid of the rooms; cells are numbered row by row.
#[derive(Debug, Clone, Copy)]
struct Raster {
    origin: [f64; 2],
    cell: f64,
    nx: usize,
    ny: usize,
}

impl Raster {
    /// A grid over the floor and the walls; `None` if no floor was measured.
    fn new(points: &StoreyPoints, faces: &[WallFace]) -> Option<Raster> {
        let band = Band {
            low: -FLOOR_BAND,
            high: FLOOR_BAND,
        };
        let (mut min, mut max) = robust_extent(&points.xyz, band)?;
        for p in faces.iter().flat_map(|f| [f.a, f.b]) {
            min = [min[0].min(p[0]), min[1].min(p[1])];
            max = [max[0].max(p[0]), max[1].max(p[1])];
        }
        let side = (max[0] - min[0]).max(max[1] - min[1]);
        let cell = CELL.max(side / (MAX_SIDE as f64 - 2.0 * MARGIN_CELLS));
        let margin = MARGIN_CELLS * cell;
        let origin = [min[0] - margin, min[1] - margin];
        let nx = (((max[0] + margin - origin[0]) / cell).ceil() as usize).clamp(1, MAX_SIDE);
        let ny = (((max[1] + margin - origin[1]) / cell).ceil() as usize).clamp(1, MAX_SIDE);
        Some(Raster {
            origin,
            cell,
            nx,
            ny,
        })
    }

    fn index(&self, p: [f64; 2]) -> Option<usize> {
        let (x, y) = (
            (p[0] - self.origin[0]) / self.cell,
            (p[1] - self.origin[1]) / self.cell,
        );
        if !(x >= 0.0 && y >= 0.0) {
            return None;
        }
        let (i, j) = (x as usize, y as usize);
        (i < self.nx && j < self.ny).then_some(j * self.nx + i)
    }

    /// The corner of cell `(i, j)` towards the origin.
    fn corner(&self, i: usize, j: usize) -> [f64; 2] {
        [
            self.origin[0] + i as f64 * self.cell,
            self.origin[1] + j as f64 * self.cell,
        ]
    }

    /// Cells with a floor point.
    fn floor(&self, points: &StoreyPoints) -> Vec<bool> {
        let mut mask = vec![false; self.nx * self.ny];
        for p in &points.xyz {
            if (p[2] as f64).abs() <= FLOOR_BAND {
                if let Some(c) = self.index([p[0] as f64, p[1] as f64]) {
                    mask[c] = true;
                }
            }
        }
        mask
    }

    /// Cells on a wall face or inside a wall with two faces.
    fn walls(&self, faces: &[WallFace], pairs: &[WallPair]) -> Vec<bool> {
        let mut mask = vec![false; self.nx * self.ny];
        for face in faces {
            self.paint(&mut mask, face.a, face.b);
        }
        for pair in pairs {
            let (front, back) = (&faces[pair.front], &faces[pair.back]);
            let Some((d, len)) = direction(front.a, front.b) else {
                continue;
            };
            let (t0, t1) = (dot(d, sub(back.a, front.a)), dot(d, sub(back.b, front.a)));
            let (from, to) = (t0.min(t1).max(0.0), t0.max(t1).min(len));
            if to <= from {
                continue;
            }
            let layers = (pair.thickness / (self.cell / 2.0)).ceil() as usize;
            for k in 0..=layers {
                let depth = -pair.thickness * k as f64 / layers.max(1) as f64;
                let a = along(along(front.a, d, from), front.inward, depth);
                let b = along(along(front.a, d, to), front.inward, depth);
                self.paint(&mut mask, a, b);
            }
        }
        mask
    }

    /// Marks the cells of a line, one cell beyond both ends: gaps at corners stay shut.
    fn paint(&self, mask: &mut [bool], a: [f64; 2], b: [f64; 2]) {
        let Some((d, len)) = direction(a, b) else {
            return;
        };
        let step = self.cell / 2.0;
        let count = ((len + 2.0 * self.cell) / step).ceil() as usize;
        for k in 0..=count {
            if let Some(c) = self.index(along(a, d, k as f64 * step - self.cell)) {
                mask[c] = true;
            }
        }
    }
}

/// One direction of a dilation: `lines` lines of `len` cells, the cell `k` of line `l` being
/// `l * across + k * step`. A cell is set up to `radius` cells from a set one.
fn dilate_along(
    mask: &[bool],
    lines: usize,
    len: usize,
    step: usize,
    across: usize,
    radius: usize,
) -> Vec<bool> {
    let mut out = vec![false; mask.len()];
    for line in 0..lines {
        let at = |k: usize| line * across + k * step;
        let mut gap = usize::MAX;
        for k in 0..len {
            gap = if mask[at(k)] {
                0
            } else {
                gap.saturating_add(1)
            };
            if gap <= radius {
                out[at(k)] = true;
            }
        }
        gap = usize::MAX;
        for k in (0..len).rev() {
            gap = if mask[at(k)] {
                0
            } else {
                gap.saturating_add(1)
            };
            if gap <= radius {
                out[at(k)] = true;
            }
        }
    }
    out
}

/// Dilation with a square of `2 * radius + 1` cells.
fn dilate(mask: &[bool], nx: usize, ny: usize, radius: usize) -> Vec<bool> {
    let rows = dilate_along(mask, ny, nx, 1, nx, radius);
    dilate_along(&rows, nx, ny, nx, 1, radius)
}

/// Morphological closing with a square of `2 * radius + 1` cells.
fn close(mask: &[bool], nx: usize, ny: usize, radius: usize) -> Vec<bool> {
    let grown = dilate(mask, nx, ny, radius);
    let outside: Vec<bool> = grown.iter().map(|c| !c).collect();
    dilate(&outside, nx, ny, radius)
        .iter()
        .map(|c| !c)
        .collect()
}

/// An area reached from the poses (`id` in the labels is its index + 1).
#[derive(Debug, Clone, Copy)]
struct Region {
    id: u32,
    /// Lowest row, then leftmost cell of the area.
    first: usize,
    cells: usize,
    low: [usize; 2],
    high: [usize; 2],
}

/// Fills the free cells from every pose (4 neighbours).
fn flood(raster: &Raster, free: &[bool], poses: &[[f32; 3]]) -> (Vec<u32>, Vec<Region>) {
    let nx = raster.nx;
    let mut label = vec![0u32; free.len()];
    let mut regions: Vec<Region> = Vec::new();
    let mut stack = Vec::new();
    for pose in poses {
        let Some(seed) = raster.index([pose[0] as f64, pose[1] as f64]) else {
            continue;
        };
        if !free[seed] || label[seed] != 0 {
            continue;
        }
        let id = regions.len() as u32 + 1;
        let mut region = Region {
            id,
            first: seed,
            cells: 0,
            low: [nx, raster.ny],
            high: [0, 0],
        };
        label[seed] = id;
        stack.push(seed);
        while let Some(c) = stack.pop() {
            region.cells += 1;
            region.first = region.first.min(c);
            let (i, j) = (c % nx, c / nx);
            region.low = [region.low[0].min(i), region.low[1].min(j)];
            region.high = [region.high[0].max(i), region.high[1].max(j)];
            let mut visit = |next: usize| {
                if free[next] && label[next] == 0 {
                    label[next] = id;
                    stack.push(next);
                }
            };
            if i > 0 {
                visit(c - 1);
            }
            if i + 1 < nx {
                visit(c + 1);
            }
            if j > 0 {
                visit(c - nx);
            }
            if j + 1 < raster.ny {
                visit(c + nx);
            }
        }
        regions.push(region);
    }
    (label, regions)
}

/// The clear height of every region: the median of the ceiling points above it.
fn room_heights(
    points: &StoreyPoints,
    raster: &Raster,
    label: &[u32],
    regions: &[Region],
) -> Vec<Option<f64>> {
    let Some(storey) = points.height.map(|h| h as f64) else {
        return vec![None; regions.len()];
    };
    let low = storey * CEILING_FROM;
    let high = storey + CEILING_ABOVE;
    let bins = ((high - low) / HEIGHT_BIN).ceil() as usize + 1;
    let mut histogram = vec![0u32; bins * regions.len()];
    for p in &points.xyz {
        let z = p[2] as f64;
        if z < low || z > high {
            continue;
        }
        if let Some(c) = raster.index([p[0] as f64, p[1] as f64]) {
            if label[c] != 0 {
                let bin = (((z - low) / HEIGHT_BIN) as usize).min(bins - 1);
                histogram[(label[c] - 1) as usize * bins + bin] += 1;
            }
        }
    }
    regions
        .iter()
        .map(|region| {
            let counts = &histogram[(region.id - 1) as usize * bins..region.id as usize * bins];
            let total: u64 = counts.iter().map(|&c| c as u64).sum();
            if (total as f64) < region.cells as f64 * CEILING_POINTS_PER_CELL {
                return Some(storey);
            }
            let mut seen = 0u64;
            for (bin, &count) in counts.iter().enumerate() {
                seen += count as u64;
                if seen * 2 >= total {
                    return Some(low + (bin as f64 + 0.5) * HEIGHT_BIN);
                }
            }
            Some(storey)
        })
        .collect()
}

/// The room of an area; `None` if its outline comes out smaller than a room.
fn room_of(
    raster: &Raster,
    label: &[u32],
    region: &Region,
    faces: &[WallFace],
    paired: &[bool],
    height: Option<f64>,
) -> Option<Room> {
    let contour = outer_contour(raster, label, region);
    let rough: Vec<[f64; 2]> = contour.iter().map(|c| raster.corner(c[0], c[1])).collect();
    let simple = simplify(&rough, SIMPLIFY);
    let mut outline = lay_on_walls(&simple, faces, paired).unwrap_or(simple);
    let area = polygon_area(&outline);
    if area < MIN_AREA {
        return None;
    }
    let open = wall_gap_share(&outline, faces, paired) > OPEN_SHARE;
    let first = (0..outline.len())
        .min_by(|&a, &b| {
            outline[a][0]
                .total_cmp(&outline[b][0])
                .then(outline[a][1].total_cmp(&outline[b][1]))
        })
        .unwrap_or(0);
    outline.rotate_left(first);
    Some(Room {
        nr: 0,
        outline,
        area,
        height,
        open,
        label_at: label_position(raster, label, region),
    })
}

/// The corners of the outer contour of a region, counterclockwise, on the grid corners.
fn outer_contour(raster: &Raster, label: &[u32], region: &Region) -> Vec<[usize; 2]> {
    const STEP: [[i64; 2]; 4] = [[1, 0], [0, 1], [-1, 0], [0, -1]];
    let (nx, ny) = (raster.nx as i64, raster.ny as i64);
    let inside = |c: [i64; 2]| {
        c[0] >= 0
            && c[1] >= 0
            && c[0] < nx
            && c[1] < ny
            && label[(c[1] * nx + c[0]) as usize] == region.id
    };
    // The cells ahead of a corner on the left and on the right of direction `d`.
    let ahead = |x: i64, y: i64, d: usize| match d {
        0 => ([x, y], [x, y - 1]),
        1 => ([x - 1, y], [x, y]),
        2 => ([x - 1, y - 1], [x - 1, y]),
        _ => ([x, y - 1], [x - 1, y - 1]),
    };
    let usable = |x: i64, y: i64, d: usize| {
        let (left, right) = ahead(x, y, d);
        inside(left) && !inside(right)
    };

    let start = [
        (region.first % raster.nx) as i64,
        (region.first / raster.nx) as i64,
    ];
    let [mut x, mut y] = start;
    let mut d = 0;
    let mut corners = vec![[start[0] as usize, start[1] as usize]];
    let limit = region.cells * 4 + 4;
    for _ in 0..limit {
        x += STEP[d][0];
        y += STEP[d][1];
        // Left first: a cell touching only at a corner is not connected.
        let Some(next) = [1, 0, 3, 2]
            .iter()
            .map(|turn| (d + turn) % 4)
            .find(|&candidate| usable(x, y, candidate))
        else {
            break;
        };
        if [x, y] == start && next == 0 {
            break;
        }
        if next != d {
            corners.push([x as usize, y as usize]);
        }
        d = next;
    }
    corners
}

/// Distance of `p` from the line through `a` and `b`.
fn line_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    match direction(a, b) {
        Some((d, _)) => cross(d, sub(p, a)).abs(),
        None => distance(p, a),
    }
}

/// Douglas–Peucker for a closed line. The first corner is kept, as is the one farthest from it.
fn simplify(points: &[[f64; 2]], tolerance: f64) -> Vec<[f64; 2]> {
    let n = points.len();
    if n < 4 {
        return points.to_vec();
    }
    let far = (1..n)
        .max_by(|&a, &b| distance(points[0], points[a]).total_cmp(&distance(points[0], points[b])))
        .unwrap_or(1);
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[far] = true;
    let mut stack = vec![(0, far), (far, n)];
    while let Some((from, to)) = stack.pop() {
        let (a, b) = (points[from], points[to % n]);
        let mut worst = (tolerance, None);
        for (k, &p) in points.iter().enumerate().take(to).skip(from + 1) {
            let off = line_distance(p, a, b);
            if off > worst.0 {
                worst = (off, Some(k));
            }
        }
        if let (_, Some(k)) = worst {
            keep[k] = true;
            stack.push((from, k));
            stack.push((k, to));
        }
    }
    points
        .iter()
        .zip(&keep)
        .filter(|(_, kept)| **kept)
        .map(|(p, _)| *p)
        .collect()
}

/// An outline edge as a line, with the corner of the grid where it starts.
#[derive(Debug, Clone, Copy)]
struct Edge {
    at: [f64; 2],
    d: [f64; 2],
    on_wall: bool,
    start: [f64; 2],
}

/// A wall running along an outline edge.
#[derive(Debug, Clone, Copy)]
struct WallNear {
    face: usize,
    wrong_side: bool,
    distance: f64,
    /// The part of the edge beside the wall, metres from its start.
    from: f64,
    to: f64,
}

/// The walls along the edge from `p` to `q` (the room on its left). A face turned away only
/// counts if it has no other side: then it is the wall of the rooms on both of its sides.
fn walls_along(p: [f64; 2], q: [f64; 2], faces: &[WallFace], paired: &[bool]) -> Vec<WallNear> {
    let Some((d, len)) = direction(p, q) else {
        return Vec::new();
    };
    let left = [-d[1], d[0]];
    let mid = middle(p, q);
    let mut near = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        let Some((fd, _)) = direction(face.a, face.b) else {
            continue;
        };
        let wrong_side = dot(face.inward, left) <= 0.0;
        if cross(d, fd).abs() > ALONG_SIN || (wrong_side && paired[index]) {
            continue;
        }
        let off = dot(face.inward, sub(mid, face.a)).abs();
        if off > WALL_REACH {
            continue;
        }
        let (t0, t1) = (dot(d, sub(face.a, p)), dot(d, sub(face.b, p)));
        let (from, to) = (t0.min(t1).max(0.0), t0.max(t1).min(len));
        if to > from {
            near.push(WallNear {
                face: index,
                wrong_side,
                distance: off,
                from,
                to,
            });
        }
    }
    near
}

/// The edge from `p` to `q`, on its wall if there is one.
fn edge_on_wall(p: [f64; 2], q: [f64; 2], faces: &[WallFace], paired: &[bool]) -> Option<Edge> {
    let (d, len) = direction(p, q)?;
    let needed = MIN_OVERLAP.min(len / 2.0);
    let wall = walls_along(p, q, faces, paired)
        .into_iter()
        .filter(|w| w.to - w.from >= needed)
        .min_by(|a, b| {
            (a.wrong_side as u8)
                .cmp(&(b.wrong_side as u8))
                .then(a.distance.total_cmp(&b.distance))
                .then(a.face.cmp(&b.face))
        });
    Some(match wall {
        Some(wall) => {
            let face = &faces[wall.face];
            let (fd, _) = direction(face.a, face.b)?;
            let d = if dot(fd, d) >= 0.0 {
                fd
            } else {
                [-fd[0], -fd[1]]
            };
            Edge {
                at: face.a,
                d,
                on_wall: true,
                start: p,
            }
        }
        None => Edge {
            at: middle(p, q),
            d,
            on_wall: false,
            start: p,
        },
    })
}

/// Where two neighbouring edges meet. Nearly parallel lines, or a crossing far from the
/// outline on the grid, give no corner: then the corner on the grid stays.
fn corner(before: &Edge, after: &Edge) -> [f64; 2] {
    let turn = cross(before.d, after.d);
    if turn.abs() < SAME_DIRECTION_SIN {
        return after.start;
    }
    let t = cross(sub(after.at, before.at), after.d) / turn;
    let crossing = along(before.at, before.d, t);
    if distance(crossing, after.start) > CORNER_REACH {
        after.start
    } else {
        crossing
    }
}

/// Lays the simplified outline on the walls. Edges shorter than `MIN_STEP` fall away, also
/// when they got that short on the walls; parallel neighbours become one edge.
fn lay_on_walls(
    outline: &[[f64; 2]],
    faces: &[WallFace],
    paired: &[bool],
) -> Option<Vec<[f64; 2]>> {
    let n = outline.len();
    let mut edges: Vec<Edge> = (0..n)
        .filter_map(|i| edge_on_wall(outline[i], outline[(i + 1) % n], faces, paired))
        .collect();
    loop {
        let m = edges.len();
        if m < 3 {
            return None;
        }
        let corners: Vec<[f64; 2]> = (0..m)
            .map(|i| corner(&edges[(i + m - 1) % m], &edges[i]))
            .collect();
        let lengths: Vec<f64> = (0..m)
            .map(|i| distance(corners[i], corners[(i + 1) % m]))
            .collect();

        let shortest = (0..m)
            .filter(|&i| lengths[i] < MIN_STEP)
            .min_by(|&a, &b| lengths[a].total_cmp(&lengths[b]));
        if let Some(i) = shortest {
            let removed = edges.remove(i);
            let next = i % edges.len();
            edges[next].start = middle(removed.start, edges[next].start);
            continue;
        }

        let same =
            (0..m).find(|&i| cross(edges[i].d, edges[(i + 1) % m].d).abs() < SAME_DIRECTION_SIN);
        if let Some(i) = same {
            let j = (i + 1) % m;
            let keep_second = match (edges[i].on_wall, edges[j].on_wall) {
                (false, true) => true,
                (true, false) => false,
                _ => lengths[j] > lengths[i],
            };
            if keep_second {
                edges[j].start = edges[i].start;
                edges.remove(i);
            } else {
                edges.remove(j);
            }
            continue;
        }
        return Some(corners);
    }
}

/// Area of a polygon (m²).
fn polygon_area(outline: &[[f64; 2]]) -> f64 {
    let n = outline.len();
    let twice: f64 = (0..n)
        .map(|i| cross(outline[i], outline[(i + 1) % n]))
        .sum();
    twice.abs() / 2.0
}

/// The share of the perimeter that has no wall beside it. Doors and unclear openings are part
/// of the faces, so they count as wall.
fn wall_gap_share(outline: &[[f64; 2]], faces: &[WallFace], paired: &[bool]) -> f64 {
    let n = outline.len();
    let mut perimeter = 0.0;
    let mut covered = 0.0;
    for i in 0..n {
        let (p, q) = (outline[i], outline[(i + 1) % n]);
        perimeter += distance(p, q);
        let mut spans: Vec<(f64, f64)> = walls_along(p, q, faces, paired)
            .iter()
            .map(|w| (w.from, w.to))
            .collect();
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut end = 0.0;
        for (from, to) in spans {
            let from = from.max(end);
            if to > from {
                covered += to - from;
                end = to;
            }
        }
    }
    if perimeter > 0.0 {
        1.0 - covered / perimeter
    } else {
        1.0
    }
}

/// The cell farthest from the edge of the area; on a tie the one nearest to its middle.
fn label_position(raster: &Raster, label: &[u32], region: &Region) -> [f64; 2] {
    let (w, h) = (
        region.high[0] - region.low[0] + 1,
        region.high[1] - region.low[1] + 1,
    );
    let inside = |x: usize, y: usize| {
        label[(region.low[1] + y) * raster.nx + region.low[0] + x] == region.id
    };
    let at = |x: usize, y: usize| y * w + x;
    let mut clearance = vec![0u16; w * h];
    let mut sum = [0.0f64; 2];
    for y in 0..h {
        for x in 0..w {
            if inside(x, y) {
                let left = if x > 0 { clearance[at(x - 1, y)] } else { 0 };
                let below = if y > 0 { clearance[at(x, y - 1)] } else { 0 };
                clearance[at(x, y)] = left.min(below).saturating_add(1);
                sum = [sum[0] + x as f64, sum[1] + y as f64];
            }
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            if inside(x, y) {
                let right = if x + 1 < w {
                    clearance[at(x + 1, y)]
                } else {
                    0
                };
                let above = if y + 1 < h {
                    clearance[at(x, y + 1)]
                } else {
                    0
                };
                let around = right.min(above).saturating_add(1);
                clearance[at(x, y)] = clearance[at(x, y)].min(around);
            }
        }
    }
    let centre = [sum[0] / region.cells as f64, sum[1] / region.cells as f64];
    let off = |x: usize, y: usize| distance([x as f64, y as f64], centre);
    let mut best = (0, 0);
    let mut best_clearance = 0;
    for y in 0..h {
        for x in 0..w {
            let here = clearance[at(x, y)];
            if here > best_clearance
                || (here == best_clearance && here > 0 && off(x, y) < off(best.0, best.1))
            {
                best = (x, y);
                best_clearance = here;
            }
        }
    }
    let corner = raster.corner(region.low[0] + best.0, region.low[1] + best.1);
    [corner[0] + raster.cell / 2.0, corner[1] + raster.cell / 2.0]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raster(nx: usize, ny: usize) -> Raster {
        Raster {
            origin: [0.0, 0.0],
            cell: 1.0,
            nx,
            ny,
        }
    }

    fn room(area: f64, first: [f64; 2]) -> Room {
        Room {
            nr: 0,
            outline: vec![first],
            area,
            height: None,
            open: false,
            label_at: first,
        }
    }

    /// Labels from rows of text, top row first: `#` is the region with id 1.
    fn labels(rows: &[&str]) -> (Raster, Vec<u32>, Region) {
        let (nx, ny) = (rows[0].len(), rows.len());
        let mut label = vec![0; nx * ny];
        let mut region = Region {
            id: 1,
            first: usize::MAX,
            cells: 0,
            low: [nx, ny],
            high: [0, 0],
        };
        for (r, row) in rows.iter().enumerate() {
            let j = ny - 1 - r;
            for (i, c) in row.bytes().enumerate() {
                if c == b'#' {
                    let index = j * nx + i;
                    label[index] = 1;
                    region.first = region.first.min(index);
                    region.cells += 1;
                    region.low = [region.low[0].min(i), region.low[1].min(j)];
                    region.high = [region.high[0].max(i), region.high[1].max(j)];
                }
            }
        }
        (raster(nx, ny), label, region)
    }

    fn contour(rows: &[&str]) -> Vec<[usize; 2]> {
        let (raster, label, region) = labels(rows);
        outer_contour(&raster, &label, &region)
    }

    #[test]
    fn equal_areas_go_by_x_then_y() {
        let mut rooms = vec![
            room(5.0, [3.0, 1.0]),
            room(9.0, [8.0, 8.0]),
            room(5.0, [2.0, 7.0]),
            room(5.0, [2.0, 3.0]),
        ];
        number_rooms(&mut rooms);
        let order: Vec<(u32, [f64; 2])> = rooms.iter().map(|r| (r.nr, r.outline[0])).collect();
        assert_eq!(
            order,
            vec![
                (1, [8.0, 8.0]),
                (2, [2.0, 3.0]),
                (3, [2.0, 7.0]),
                (4, [3.0, 1.0])
            ]
        );
    }

    #[test]
    fn contour_of_a_block_runs_counterclockwise() {
        assert_eq!(
            contour(&["....", ".##.", ".##.", "...."]),
            vec![[1, 1], [3, 1], [3, 3], [1, 3]]
        );
    }

    #[test]
    fn contour_leaves_out_a_hole() {
        assert_eq!(
            contour(&["###", "#.#", "###"]),
            vec![[0, 0], [3, 0], [3, 3], [0, 3]]
        );
    }

    #[test]
    fn cells_touching_at_a_corner_are_not_connected() {
        let rows = ["#..", ".#.", "..."];
        let (raster, mut label, region) = labels(&rows);
        // The upper cell is another region: the lower one is traced alone.
        label[2 * raster.nx] = 2;
        let region = Region {
            cells: 1,
            first: raster.nx + 1,
            low: [1, 1],
            high: [1, 1],
            ..region
        };
        assert_eq!(
            outer_contour(&raster, &label, &region),
            vec![[1, 1], [2, 1], [2, 2], [1, 2]]
        );
    }

    #[test]
    fn a_pinch_is_passed_without_crossing() {
        // Connected around the pinch at the middle of the second row from the top.
        let rows = ["##.", "#.#", "###"];
        let found = contour(&rows);
        assert_eq!(found[0], [0, 0]);
        assert!(found.contains(&[2, 3]), "{found:?}");
        assert!(found.contains(&[2, 1]), "{found:?}");
    }

    #[test]
    fn simplify_removes_a_small_step_and_keeps_corners() {
        let points = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 0.04],
            [8.0, 0.04],
            [8.0, 8.0],
            [0.0, 8.0],
        ];
        let simple = simplify(&points, 0.05);
        assert_eq!(simple.len(), 4, "{simple:?}");
    }

    #[test]
    fn closing_fills_narrow_gaps_only() {
        let (nx, ny) = (20, 3);
        let mut mask = vec![true; nx * ny];
        // A gap of 4 cells and one of 7 across the whole height.
        for j in 0..ny {
            for i in (3..7).chain(10..17) {
                mask[j * nx + i] = false;
            }
        }
        let closed = close(&mask, nx, ny, 2);
        assert!(closed[nx + 5], "kleine Lücke");
        assert!(!closed[nx + 13], "große Lücke");
    }

    fn face(a: [f64; 2], b: [f64; 2], inward: [f64; 2]) -> WallFace {
        WallFace {
            a,
            b,
            inward,
            seen_m: distance(a, b),
        }
    }

    #[test]
    fn a_step_below_ten_centimetres_goes_and_corners_are_crossings() {
        // Two long pieces of one wall, 6 cm apart, and walls on the three other sides.
        let faces = [
            face([0.0, 0.0], [4.0, 0.0], [0.0, 1.0]),
            face([4.0, 0.06], [8.0, 0.06], [0.0, 1.0]),
            face([8.0, 0.0], [8.0, 5.0], [-1.0, 0.0]),
            face([8.0, 5.0], [0.0, 5.0], [0.0, -1.0]),
            face([0.0, 5.0], [0.0, 0.0], [1.0, 0.0]),
        ];
        let outline = [
            [0.03, 0.03],
            [4.01, 0.03],
            [4.01, 0.09],
            [7.97, 0.09],
            [7.97, 4.97],
            [0.03, 4.97],
        ];
        let laid = lay_on_walls(&outline, &faces, &[false; 5]).unwrap();
        assert_eq!(laid.len(), 4, "{laid:?}");
        assert!(
            laid.contains(&[0.0, 5.0]) && laid.contains(&[8.0, 5.0]),
            "{laid:?}"
        );
        assert!(laid
            .iter()
            .any(|p| (p[1] - 0.0).abs() < 1e-9 || (p[1] - 0.06).abs() < 1e-9));
    }

    #[test]
    fn a_face_of_the_other_room_is_not_taken() {
        let faces = [
            face([0.0, 0.0], [4.0, 0.0], [0.0, 1.0]),
            face([4.0, -0.08], [0.0, -0.08], [0.0, -1.0]),
        ];
        let near = walls_along([0.0, 0.03], [4.0, 0.03], &faces, &[true, true]);
        assert_eq!(near.len(), 1, "{near:?}");
        assert_eq!(near[0].face, 0);
        // A wall measured from one side is the wall of the room behind it too.
        let near = walls_along([4.0, -0.03], [0.0, -0.03], &faces[..1], &[false]);
        assert_eq!(near.len(), 1, "{near:?}");
        assert!(near[0].wrong_side);
    }

    #[test]
    fn gaps_count_against_the_perimeter() {
        let faces = [face([0.0, 0.0], [4.0, 0.0], [0.0, 1.0])];
        let outline = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let share = wall_gap_share(&outline, &faces, &[false]);
        assert!((share - 0.75).abs() < 1e-9, "{share}");
    }
}
