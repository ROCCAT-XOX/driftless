//! Walls: where the measured wall surfaces are, seen from above.
//!
//! The points of the wall band go on a grid that says how much of the room height is occupied
//! ([`crate::grid`]). The main direction is the angle at which the occupied cells form the
//! sharpest stripes. Wall lines are searched along it and at right angles to it, then in what
//! is left over at any angle. The distance of a line comes from the points themselves (median),
//! which is more exact than the grid. Pieces of one line become one face across gaps up to 3 m;
//! whether such a gap is a door is decided later. The side of the room comes from where the
//! trajectory can see the face from.

use std::cmp::Reverse;

use driftless_io::progress::Progress;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::grid::{Band, CellClass, HeightGrid};
use crate::levels::check;
use crate::{Opening, PlanConfig, PlanError, StoreyPoints};

/// The wall band starts this far above the floor ...
const FLOOR_MARGIN: f64 = 0.15;
/// ... and ends this far below the ceiling, or at `NO_CEILING_TOP` without one (metres).
const CEILING_MARGIN: f64 = 0.15;
const NO_CEILING_TOP: f64 = 2.2;

/// Main direction: coarse steps over 90°, then fine steps around the best (degrees).
const COARSE_STEP: f64 = 0.5;
const COARSE_ANGLES: usize = 180;
const FINE_STEP: f64 = 0.1;
const FINE_ANGLES: i32 = 5;
/// Slanted walls: whole degrees over 180°.
const SLANT_ANGLES: u32 = 180;
const MAX_SLANTED: usize = 128;

/// A line needs this much strong wall (metres) ...
const MIN_SUPPORT: f64 = 0.5;
/// ... in pieces of at least this length (metres); walls crossing it are shorter.
const SUPPORT_PIECE: f64 = 0.10;
/// Points this close to a line are on it (metres).
const BAND: f64 = 0.04;
const SETTLE_ROUNDS: usize = 4;
/// Lines closer than this are the same line (metres): nearer than the thinnest wall
/// ([`PAIR_THICKNESS`]), so they cannot be two surfaces. A door leaf set back a little would
/// otherwise see the wall's own cells and become a second line along the whole wall.
const SAME_LINE: f64 = 0.05;
/// sin 1°: lines closer in direction are parallel.
const PARALLEL_SIN: f64 = 0.017_452_406_437_283_51;
/// sin 2°: lines closer than this to a main direction are laid on it.
const SNAP_SIN: f64 = 0.03489949670250097;
/// sin 10°: lines closer in direction have no useful crossing.
const CROSSING_SIN: f64 = 0.17364817766693033;
/// A piece leaving a line sideways by this much between its first and last quarter (metres).
const MAX_DRIFT: f64 = 0.03;

/// Gaps up to this length stay inside a piece (metres).
const BRIDGE: f64 = 0.25;
/// Pieces of a line with gaps up to this length are one face (metres).
const DOOR_GAP: f64 = 3.0;
/// Measured faces below this length are dropped (metres).
const MIN_FACE: f64 = 0.30;
/// Pieces below this length are specks (metres).
const MIN_PIECE: f64 = 0.06;
/// Tolerance when asking whether a wall reaches a position (metres).
const REACH_MARGIN: f64 = 0.10;

/// Two ends this close to the crossing of their lines form a corner (metres).
const CORNER: f64 = 0.25;
/// Closing a corner leaves at least this much of a face (metres).
const MIN_CLOSED: f64 = 0.10;

/// Sight from the trajectory: poses at most, rays per pose, start and reach (metres).
const MAX_POSES: usize = 400;
const RAYS: usize = 720;
const RAY_START: f64 = 0.15;
const RAY_LENGTH: f64 = 30.0;

/// Distance of the two faces of one wall (metres).
const PAIR_THICKNESS: (f64, f64) = (0.05, 0.60);
/// The two faces overlap on this share of the shorter one.
const PAIR_OVERLAP: f64 = 0.5;
/// A face may have several partners; another partner may overlap this share of a new stretch.
const PAIR_SHARED: f64 = 0.5;

/// A measured wall surface in the frame of [`StoreyPoints`], seen from above.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WallFace {
    pub a: [f64; 2],
    pub b: [f64; 2],
    /// Unit vector into the room: on the left when going from `a` to `b`.
    pub inward: [f64; 2],
    /// The length really measured (without door gaps), in metres.
    pub seen_m: f64,
}

/// Two faces that are the two sides of one wall (indices into [`Walls::faces`]).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WallPair {
    pub front: usize,
    pub back: usize,
    pub thickness: f64,
}

/// The walls of a storey.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Walls {
    /// Main direction as a unit vector, folded onto the first quadrant.
    pub axis: [f64; 2],
    pub faces: Vec<WallFace>,
    pub pairs: Vec<WallPair>,
    /// Edge of the grid cell used (metres).
    pub cell: f64,
}

/// Upper end of the wall band above the floor (metres).
pub(crate) fn band_top(height: Option<f32>) -> f64 {
    height.map_or(NO_CEILING_TOP, |h| h as f64 - CEILING_MARGIN)
}

/// Finds the wall faces of a storey.
pub fn find_walls(
    points: &StoreyPoints,
    cfg: &PlanConfig,
    progress: &Progress,
) -> Result<Walls, PlanError> {
    progress.phase("Wände", Some(5));
    check(progress)?;
    if points.poses.is_empty() {
        return Err(PlanError::NoPoses);
    }
    let band = Band {
        low: FLOOR_MARGIN,
        high: band_top(points.height),
    };
    let grid = HeightGrid::build(&points.xyz, band, cfg.cell).ok_or(PlanError::NoWalls)?;
    progress.advance(1);
    check(progress)?;

    let ctx = Search::new(&grid, points);
    if !ctx.cells.iter().any(|c| c.strong) {
        return Err(PlanError::NoWalls);
    }
    let axis = main_direction(&ctx);
    progress.advance(1);
    check(progress)?;

    let along = Frame::new(axis);
    let across = Frame::new(along.n);
    let all: Vec<u32> = (0..ctx.cells.len() as u32).collect();
    let mut lines = lines_along(&ctx, along, &all);
    lines.extend(lines_along(&ctx, across, &all));
    progress.advance(1);
    check(progress)?;

    slanted_lines(&ctx, [along, across], &mut lines, progress)?;
    progress.advance(1);
    check(progress)?;

    let mut drafts = drafts_of(&ctx, &lines);
    if drafts.is_empty() {
        return Err(PlanError::NoWalls);
    }
    close_corners(&mut drafts);
    let inward = inward_sides(&ctx, &drafts);
    let faces: Vec<WallFace> = drafts
        .iter()
        .zip(&inward)
        .map(|(draft, &positive)| draft.face(positive))
        .collect();
    let pairs = find_pairs(&faces);
    progress.advance(1);
    Ok(Walls {
        axis,
        faces,
        pairs,
        cell: grid.cell(),
    })
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

fn median(values: &mut [f64]) -> f64 {
    let middle = values.len() / 2;
    *values.select_nth_unstable_by(middle, f64::total_cmp).1
}

/// A direction and its normal, which is `(-y, x)`: a right angle without an angle function.
#[derive(Debug, Clone, Copy)]
struct Frame {
    d: [f64; 2],
    n: [f64; 2],
}

impl Frame {
    fn new(d: [f64; 2]) -> Frame {
        Frame {
            d,
            n: [-d[1], d[0]],
        }
    }

    fn u(&self, p: [f64; 2]) -> f64 {
        dot(self.d, p)
    }

    fn v(&self, p: [f64; 2]) -> f64 {
        dot(self.n, p)
    }

    fn at(&self, u: f64, v: f64) -> [f64; 2] {
        [u * self.d[0] + v * self.n[0], u * self.d[1] + v * self.n[1]]
    }
}

/// A cell that counts as wall.
#[derive(Debug, Clone, Copy)]
struct WallCell {
    at: [f64; 2],
    index: u32,
    strong: bool,
}

/// What every step of the search needs.
struct Search<'a> {
    grid: &'a HeightGrid,
    points: &'a StoreyPoints,
    cells: Vec<WallCell>,
    wall: Vec<bool>,
    cell: f64,
    /// Lengths in cells: the widest gap inside a piece, the shortest piece that adds to the
    /// support, and the support a line needs.
    link: i32,
    support_piece: u32,
    need: u32,
    /// Cell centres are this far from the line at most when their points are within `BAND`.
    band_cells: f64,
}

impl<'a> Search<'a> {
    fn new(grid: &'a HeightGrid, points: &'a StoreyPoints) -> Search<'a> {
        let class = grid.classify();
        let cells = class
            .iter()
            .enumerate()
            .filter(|(_, c)| **c != CellClass::Empty)
            .map(|(index, c)| WallCell {
                at: grid.centre(index),
                index: index as u32,
                strong: *c == CellClass::Strong,
            })
            .collect();
        let cell = grid.cell();
        Search {
            grid,
            points,
            cells,
            wall: class.iter().map(|c| *c != CellClass::Empty).collect(),
            cell,
            link: (BRIDGE / cell).floor() as i32 + 1,
            support_piece: (SUPPORT_PIECE / cell).ceil() as u32,
            need: (MIN_SUPPORT / cell).ceil() as u32,
            band_cells: BAND + 0.75 * cell,
        }
    }

    /// Positions of the points of `cells` within `BAND` of the line at `v`.
    fn points_near(&self, frame: Frame, v: f64, cells: &[Placed]) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for c in cells {
            for &i in self
                .grid
                .points_of(self.cells[c.cell as usize].index as usize)
            {
                let p = self.points.xyz[i as usize];
                let q = [p[0] as f64, p[1] as f64];
                if (frame.v(q) - v).abs() <= BAND {
                    out.push(q);
                }
            }
        }
        out
    }
}

/// The angle in [0°, 90°) at which the strong cells form the sharpest stripes, as a unit vector.
fn main_direction(ctx: &Search) -> [f64; 2] {
    let strong: Vec<[f64; 2]> = ctx
        .cells
        .iter()
        .filter(|c| c.strong)
        .map(|c| c.at)
        .collect();
    let count = strong.len() as f64;
    let centre = [
        strong.iter().map(|p| p[0]).sum::<f64>() / count,
        strong.iter().map(|p| p[1]).sum::<f64>() / count,
    ];
    let radius = strong
        .iter()
        .map(|&p| distance(p, centre))
        .fold(0.0, f64::max);
    let bins = (radius * 2.0 / ctx.cell) as usize + 3;
    let sharpness = |deg: f64| -> u64 {
        let (s, c) = deg.to_radians().sin_cos();
        let mut along = vec![0u64; bins];
        let mut across = vec![0u64; bins];
        for p in &strong {
            let (dx, dy) = (p[0] - centre[0], p[1] - centre[1]);
            along[(((c * dx + s * dy + radius) / ctx.cell) as usize).min(bins - 1)] += 1;
            across[(((-s * dx + c * dy + radius) / ctx.cell) as usize).min(bins - 1)] += 1;
        }
        along.iter().chain(&across).map(|&h| h * h).sum()
    };
    let best = |angles: Vec<f64>| -> f64 {
        let scores: Vec<u64> = angles.par_iter().map(|&deg| sharpness(deg)).collect();
        let winner = (0..angles.len())
            .max_by_key(|&i| (scores[i], Reverse(i)))
            .expect("the list of angles is not empty");
        angles[winner]
    };
    let coarse = best((0..COARSE_ANGLES).map(|i| i as f64 * COARSE_STEP).collect());
    let fine = best(
        (-FINE_ANGLES..=FINE_ANGLES)
            .map(|i| coarse + i as f64 * FINE_STEP)
            .collect(),
    );
    let (s, c) = fine.rem_euclid(90.0).to_radians().sin_cos();
    [c, s]
}

/// A wall cell seen along a direction.
#[derive(Debug, Clone, Copy)]
struct Placed {
    v_bin: i32,
    u_bin: i32,
    v: f64,
    u: f64,
    cell: u32,
    strong: bool,
}

/// Cells sorted by their distance from a line through the origin (bins of one cell).
struct Profile<'a> {
    ctx: &'a Search<'a>,
    frame: Frame,
    v_min: f64,
    placed: Vec<Placed>,
    /// Cells of bin `b`: `placed[starts[b]..starts[b + 1]]`.
    starts: Vec<u32>,
}

impl<'a> Profile<'a> {
    fn new(ctx: &'a Search<'a>, frame: Frame, live: &[u32]) -> Profile<'a> {
        let cells = &ctx.cells;
        let (mut u_min, mut v_min) = (f64::MAX, f64::MAX);
        for &c in live {
            u_min = u_min.min(frame.u(cells[c as usize].at));
            v_min = v_min.min(frame.v(cells[c as usize].at));
        }
        let mut placed: Vec<Placed> = live
            .iter()
            .map(|&c| {
                let cell = &cells[c as usize];
                let (u, v) = (frame.u(cell.at), frame.v(cell.at));
                Placed {
                    // Cell centres in the middle of a bin: rounding cannot move them across.
                    v_bin: ((v - v_min) / ctx.cell + 0.5) as i32,
                    u_bin: ((u - u_min) / ctx.cell + 0.5) as i32,
                    v,
                    u,
                    cell: c,
                    strong: cell.strong,
                }
            })
            .collect();
        placed.sort_unstable_by_key(|p| (p.v_bin, p.u_bin, p.cell));
        let bins = placed.last().map_or(0, |p| p.v_bin as usize + 1);
        let mut starts = vec![0u32; bins + 1];
        for p in &placed {
            starts[p.v_bin as usize + 1] += 1;
        }
        for b in 0..bins {
            starts[b + 1] += starts[b];
        }
        Profile {
            ctx,
            frame,
            v_min,
            placed,
            starts,
        }
    }

    fn bin(&self, b: usize) -> &[Placed] {
        &self.placed[self.starts[b] as usize..self.starts[b + 1] as usize]
    }

    fn bins(&self) -> usize {
        self.starts.len() - 1
    }

    fn bin_centre(&self, b: usize) -> f64 {
        self.v_min + b as f64 * self.ctx.cell
    }

    fn bin_of(&self, v: f64) -> i64 {
        ((v - self.v_min) / self.ctx.cell + 0.5).floor() as i64
    }

    /// For every bin, the strong length (in cells) of the pieces around it, three bins wide.
    fn supports(&self) -> Vec<u32> {
        (0..self.bins())
            .into_par_iter()
            .map_init(Vec::new, |buffer: &mut Vec<(i32, bool)>, b| {
                buffer.clear();
                for w in b.saturating_sub(1)..=(b + 1).min(self.bins() - 1) {
                    buffer.extend(self.bin(w).iter().map(|p| (p.u_bin, p.strong)));
                }
                buffer.sort_unstable();
                let mut total = 0;
                for run in buffer.chunk_by(|a, b| b.0 - a.0 <= self.ctx.link) {
                    let strong = run
                        .chunk_by(|a, b| a.0 == b.0)
                        .filter(|same| same.iter().any(|e| e.1))
                        .count() as u32;
                    if strong >= self.ctx.support_piece {
                        total += strong;
                    }
                }
                total
            })
            .collect()
    }

    /// The cells near the line at `v`, along the line.
    fn band(&self, v: f64) -> Vec<Placed> {
        let reach = self.ctx.band_cells;
        let first = self.bin_of(v - reach).max(0) as usize;
        let last = self.bin_of(v + reach).min(self.bins() as i64 - 1);
        let mut band: Vec<Placed> = (first as i64..=last)
            .flat_map(|b| self.bin(b as usize))
            .filter(|p| (p.v - v).abs() <= reach)
            .copied()
            .collect();
        band.sort_unstable_by_key(|p| (p.u_bin, p.cell));
        band
    }
}

/// Adjacent cells of a line, with gaps up to `BRIDGE`.
#[derive(Debug, Clone)]
struct Piece {
    start: f64,
    end: f64,
    /// Cells along the line that are occupied, and how many of them by strong cells.
    seen: u32,
    strong: u32,
    range: std::ops::Range<usize>,
}

/// The pieces of cells sorted along the line.
fn pieces_of(ctx: &Search, band: &[Placed]) -> Vec<Piece> {
    let mut pieces = Vec::new();
    let mut at = 0;
    for run in band.chunk_by(|a, b| b.u_bin - a.u_bin <= ctx.link) {
        let mut seen = 0;
        let mut strong = 0;
        for same in run.chunk_by(|a, b| a.u_bin == b.u_bin) {
            seen += 1;
            if same.iter().any(|p| p.strong) {
                strong += 1;
            }
        }
        let start = run.iter().map(|p| p.u).fold(f64::MAX, f64::min);
        let end = run.iter().map(|p| p.u).fold(f64::MIN, f64::max);
        pieces.push(Piece {
            start: start - ctx.cell / 2.0,
            end: end + ctx.cell / 2.0,
            seen,
            strong,
            range: at..at + run.len(),
        });
        at += run.len();
    }
    pieces
}

fn has_support(ctx: &Search, pieces: &[Piece]) -> bool {
    let strong: u32 = pieces
        .iter()
        .filter(|p| p.strong >= ctx.support_piece)
        .map(|p| p.strong)
        .sum();
    strong >= ctx.need
}

/// A line that has wall on it.
#[derive(Debug, Clone)]
struct Line {
    frame: Frame,
    v: f64,
    /// The cells of the line, along it.
    band: Vec<Placed>,
    pieces: Vec<Piece>,
}

impl Line {
    fn new(ctx: &Search, frame: Frame, v: f64, band: Vec<Placed>) -> Option<Line> {
        let pieces = pieces_of(ctx, &band);
        has_support(ctx, &pieces).then_some(Line {
            frame,
            v,
            band,
            pieces,
        })
    }

    /// Whether the wall of this line reaches the position `u` along it.
    fn reaches(&self, ctx: &Search, u: f64) -> bool {
        let long = (MIN_FACE / ctx.cell).ceil() as u32;
        self.pieces
            .iter()
            .any(|p| p.seen >= long && u >= p.start - REACH_MARGIN && u <= p.end + REACH_MARGIN)
    }

    fn same_as(&self, frame: Frame, v: f64) -> bool {
        let turn = dot(self.frame.d, frame.d);
        cross(self.frame.d, frame.d).abs() <= PARALLEL_SIN
            && (self.v - v * turn.signum()).abs() <= SAME_LINE
    }
}

/// The line near `v`: its distance is the median of the points, not a grid position.
fn settle(profile: &Profile, start: f64) -> Option<(f64, Vec<Placed>)> {
    let ctx = profile.ctx;
    let mut v = start;
    let mut band = profile.band(v);
    for _ in 0..SETTLE_ROUNDS {
        let mut offsets: Vec<f64> = ctx
            .points_near(profile.frame, v, &band)
            .iter()
            .map(|&q| profile.frame.v(q))
            .collect();
        if offsets.is_empty() {
            return None;
        }
        let moved = median(&mut offsets);
        let change = (moved - v).abs();
        v = moved;
        band = profile.band(v);
        if change < 1e-4 {
            break;
        }
    }
    Some((v, band))
}

/// Whether a piece leaves the line sideways: then it is a wall at a slight angle, which
/// would come out as a stair of short lines along this direction.
fn drifts(ctx: &Search, band: &[Placed], pieces: &[Piece]) -> bool {
    let Some(longest) = pieces.iter().max_by_key(|p| p.seen) else {
        return false;
    };
    if longest.seen < ctx.need {
        return false;
    }
    let cells = &band[longest.range.clone()];
    let quarter = cells.len() / 4;
    let side = |part: &[Placed]| median(&mut part.iter().map(|p| p.v).collect::<Vec<_>>());
    let drift = (side(&cells[..quarter]) - side(&cells[cells.len() - quarter..])).abs();
    drift >= MAX_DRIFT.max(1.5 * ctx.cell)
}

/// The lines along one direction, by the position across it.
fn lines_along(ctx: &Search, frame: Frame, live: &[u32]) -> Vec<Line> {
    let profile = Profile::new(ctx, frame, live);
    let supports = profile.supports();
    let mut order: Vec<usize> = (0..supports.len())
        .filter(|&b| supports[b] >= ctx.need)
        .collect();
    order.sort_unstable_by_key(|&b| (Reverse(supports[b]), b));

    let radius = (BAND / ctx.cell).ceil() as i64;
    let mut taken = vec![false; supports.len()];
    let mut lines: Vec<Line> = Vec::new();
    for b in order {
        if taken[b] {
            continue;
        }
        taken[b] = true;
        let Some((v, band)) = settle(&profile, profile.bin_centre(b)) else {
            continue;
        };
        let centre = profile.bin_of(v);
        for t in (centre - radius).max(0)..=(centre + radius).min(supports.len() as i64 - 1) {
            taken[t as usize] = true;
        }
        if lines.iter().any(|l| l.same_as(frame, v)) {
            continue;
        }
        if let Some(line) = Line::new(ctx, frame, v, band) {
            if !drifts(ctx, &line.band, &line.pieces) {
                lines.push(line);
            }
        }
    }
    lines.sort_by(|a, b| a.v.total_cmp(&b.v));
    lines
}

/// The direction of a line of points, by the sign of `near` (total least squares).
fn fit_direction(points: &[[f64; 2]], near: [f64; 2]) -> [f64; 2] {
    let count = points.len() as f64;
    let mean = [
        points.iter().map(|p| p[0]).sum::<f64>() / count,
        points.iter().map(|p| p[1]).sum::<f64>() / count,
    ];
    let (mut xx, mut xy, mut yy) = (0.0, 0.0, 0.0);
    for p in points {
        let (x, y) = (p[0] - mean[0], p[1] - mean[1]);
        xx += x * x;
        xy += x * y;
        yy += y * y;
    }
    let root = ((xx - yy) * (xx - yy) / 4.0 + xy * xy).sqrt();
    let largest = (xx + yy) / 2.0 + root;
    // Eigenvector of the larger eigenvalue; of the two formulas, the better conditioned one.
    let d = if xx >= yy {
        [largest - yy, xy]
    } else {
        [xy, largest - xx]
    };
    let len = d[0].hypot(d[1]);
    if len < 1e-12 {
        return near;
    }
    let d = [d[0] / len, d[1] / len];
    if dot(d, near) < 0.0 {
        [-d[0], -d[1]]
    } else {
        d
    }
}

/// The direction of `axes` within 2° of `d`, with the orientation of `d`.
fn snapped(d: [f64; 2], axes: [Frame; 2]) -> Option<[f64; 2]> {
    axes.iter().find_map(|axis| {
        (cross(axis.d, d).abs() <= SNAP_SIN).then(|| {
            if dot(axis.d, d) >= 0.0 {
                axis.d
            } else {
                [-axis.d[0], -axis.d[1]]
            }
        })
    })
}

/// Lines in the cells no straight line along the main directions has taken.
fn slanted_lines(
    ctx: &Search,
    axes: [Frame; 2],
    lines: &mut Vec<Line>,
    progress: &Progress,
) -> Result<(), PlanError> {
    let mut used = vec![false; ctx.cells.len()];
    for line in lines.iter() {
        for p in &line.band {
            used[p.cell as usize] = true;
        }
    }
    let mut live: Vec<u32> = (0..ctx.cells.len() as u32)
        .filter(|&c| !used[c as usize])
        .collect();

    for _ in 0..MAX_SLANTED {
        check(progress)?;
        let strong: Vec<u32> = live
            .iter()
            .copied()
            .filter(|&c| ctx.cells[c as usize].strong)
            .collect();
        if strong.len() < ctx.need as usize {
            break;
        }
        let best = (0..SLANT_ANGLES)
            .into_par_iter()
            .filter_map(|deg| {
                let (s, c) = (deg as f64).to_radians().sin_cos();
                let frame = Frame::new([c, s]);
                let profile = Profile::new(ctx, frame, &strong);
                let supports = profile.supports();
                let bin = (0..supports.len()).max_by_key(|&b| (supports[b], Reverse(b)))?;
                (supports[bin] >= ctx.need).then(|| (supports[bin], deg, profile.bin_centre(bin)))
            })
            .collect::<Vec<_>>()
            .into_iter()
            .max_by_key(|&(support, deg, _)| (support, Reverse(deg)));
        let Some((_, deg, v)) = best else {
            break;
        };
        let (s, c) = (deg as f64).to_radians().sin_cos();
        let (line, spent) = fit_slanted(ctx, Frame::new([c, s]), v, &live, axes);
        if let Some(line) = line {
            if !lines.iter().any(|l| l.same_as(line.frame, line.v)) {
                lines.push(line);
            }
        }
        if spent.is_empty() {
            break;
        }
        for &c in &spent {
            used[c as usize] = true;
        }
        live.retain(|&c| !used[c as usize]);
    }
    Ok(())
}

/// Follows a candidate to the direction and distance of its points. Returns the line, if the
/// candidate holds, and the cells that are used up by it.
fn fit_slanted(
    ctx: &Search,
    start: Frame,
    start_v: f64,
    live: &[u32],
    axes: [Frame; 2],
) -> (Option<Line>, Vec<u32>) {
    let mut frame = start;
    let mut v = start_v;
    for _ in 0..SETTLE_ROUNDS {
        let profile = Profile::new(ctx, frame, live);
        let near = ctx.points_near(frame, v, &profile.band(v));
        if near.len() < 2 {
            return (None, Vec::new());
        }
        frame = Frame::new(fit_direction(&near, frame.d));
        let mut offsets: Vec<f64> = near.iter().map(|&q| frame.v(q)).collect();
        v = median(&mut offsets);
    }
    let profile = Profile::new(ctx, frame, live);
    let band = profile.band(v);
    let spent: Vec<u32> = band.iter().map(|p| p.cell).collect();

    let line = match snapped(frame.d, axes) {
        Some(d) => {
            // Near a main direction: laid on it, with the cells of the slanted fit.
            let near = ctx.points_near(frame, v, &band);
            let on_axis = Frame::new(d);
            let mut offsets: Vec<f64> = near.iter().map(|&q| on_axis.v(q)).collect();
            let v = median(&mut offsets);
            let mut placed = Profile::new(ctx, on_axis, &spent).placed;
            placed.sort_unstable_by_key(|p| (p.u_bin, p.cell));
            Line::new(ctx, on_axis, v, placed)
        }
        None => Line::new(ctx, frame, v, band),
    };
    (line, spent)
}

/// A wall face before it knows its room: along `frame.d`, from `start` to `end`.
#[derive(Debug, Clone)]
struct Draft {
    frame: Frame,
    v: f64,
    start: f64,
    end: f64,
    seen_m: f64,
    cells: Vec<u32>,
}

impl Draft {
    fn point(&self, at_end: bool) -> [f64; 2] {
        self.frame
            .at(if at_end { self.end } else { self.start }, self.v)
    }

    fn put(&mut self, at_end: bool, p: [f64; 2]) {
        let u = self.frame.u(p);
        if at_end {
            self.end = u;
        } else {
            self.start = u;
        }
    }

    fn length_with(&self, at_end: bool, p: [f64; 2]) -> f64 {
        let u = self.frame.u(p);
        if at_end {
            u - self.start
        } else {
            self.end - u
        }
    }

    /// Where the lines of two faces cross; `None` for faces near to parallel.
    fn crossing(&self, other: &Draft) -> Option<[f64; 2]> {
        line_crossing(self.frame, self.v, other.frame, other.v)
    }

    fn face(&self, inward_positive: bool) -> WallFace {
        let (first, second) = (
            self.frame.at(self.start, self.v),
            self.frame.at(self.end, self.v),
        );
        let (a, b, inward) = if inward_positive {
            (first, second, self.frame.n)
        } else {
            (second, first, [-self.frame.n[0], -self.frame.n[1]])
        };
        WallFace {
            a,
            b,
            inward,
            seen_m: self.seen_m,
        }
    }
}

fn line_crossing(a: Frame, av: f64, b: Frame, bv: f64) -> Option<[f64; 2]> {
    let turn = cross(a.d, b.d);
    if turn.abs() < CROSSING_SIN {
        return None;
    }
    // On the line of `a` at position u: v in the frame of `b` must be `bv`.
    let u = (bv - av * dot(a.n, b.n)) / dot(a.d, b.n);
    Some(a.at(u, av))
}

/// The cuts on a line where another wall crosses it: there its cells say nothing about this
/// line, and a wall merely crossing its extension would otherwise lengthen it.
fn crossing_cuts(ctx: &Search, lines: &[Line], index: usize) -> Vec<(f64, f64)> {
    let line = &lines[index];
    let mut cuts = Vec::new();
    for (other_index, other) in lines.iter().enumerate() {
        if other_index == index {
            continue;
        }
        let Some(at) = line_crossing(line.frame, line.v, other.frame, other.v) else {
            continue;
        };
        if !other.reaches(ctx, other.frame.u(at)) {
            continue;
        }
        let half = ctx.band_cells / cross(line.frame.d, other.frame.d).abs();
        let u = line.frame.u(at);
        cuts.push((u - half, u + half));
    }
    cuts
}

/// Turns the lines into faces. Pieces of one line with up to `DOOR_GAP` between them are one
/// face: whether a gap is a door is decided later.
fn drafts_of(ctx: &Search, lines: &[Line]) -> Vec<Draft> {
    let min_piece = (MIN_PIECE / ctx.cell).ceil() as u32;
    let mut drafts = Vec::new();
    for (index, line) in lines.iter().enumerate() {
        let cuts = crossing_cuts(ctx, lines, index);
        let kept: Vec<Placed> = line
            .band
            .iter()
            .filter(|p| !cuts.iter().any(|&(from, to)| p.u >= from && p.u <= to))
            .copied()
            .collect();
        let pieces: Vec<Piece> = pieces_of(ctx, &kept)
            .into_iter()
            .filter(|p| p.seen >= min_piece)
            .collect();
        for group in pieces.chunk_by(|a, b| b.start - a.end <= DOOR_GAP) {
            let (start, end) = (group[0].start, group[group.len() - 1].end);
            let inside = |p: &&Placed| p.u >= start - ctx.cell && p.u <= end + ctx.cell;
            let seen = line
                .band
                .iter()
                .filter(inside)
                .collect::<Vec<_>>()
                .chunk_by(|a, b| a.u_bin == b.u_bin)
                .count() as f64
                * ctx.cell;
            if seen < MIN_FACE {
                continue;
            }
            drafts.push(Draft {
                frame: line.frame,
                v: line.v,
                start,
                end,
                seen_m: seen,
                cells: line.band.iter().filter(inside).map(|p| p.cell).collect(),
            });
        }
    }
    drafts
}

/// Closes corners: two face ends within `CORNER` of the crossing of their lines end there.
/// Afterwards free ends run onto a face that passes by (a wall meeting another one).
fn close_corners(drafts: &mut [Draft]) {
    struct Corner {
        gap: f64,
        i: usize,
        end_i: bool,
        j: usize,
        end_j: bool,
        at: [f64; 2],
    }
    let mut corners = Vec::new();
    for i in 0..drafts.len() {
        for j in i + 1..drafts.len() {
            let Some(at) = drafts[i].crossing(&drafts[j]) else {
                continue;
            };
            for end_i in [false, true] {
                let gap_i = distance(drafts[i].point(end_i), at);
                for end_j in [false, true] {
                    let gap_j = distance(drafts[j].point(end_j), at);
                    if gap_i <= CORNER && gap_j <= CORNER {
                        corners.push(Corner {
                            gap: gap_i + gap_j,
                            i,
                            end_i,
                            j,
                            end_j,
                            at,
                        });
                    }
                }
            }
        }
    }
    corners.sort_by(|a, b| {
        a.gap
            .total_cmp(&b.gap)
            .then_with(|| (a.i, a.end_i, a.j, a.end_j).cmp(&(b.i, b.end_i, b.j, b.end_j)))
    });

    let mut free = vec![[true; 2]; drafts.len()];
    for c in &corners {
        if !free[c.i][c.end_i as usize] || !free[c.j][c.end_j as usize] {
            continue;
        }
        if drafts[c.i].length_with(c.end_i, c.at) < MIN_CLOSED
            || drafts[c.j].length_with(c.end_j, c.at) < MIN_CLOSED
        {
            continue;
        }
        free[c.i][c.end_i as usize] = false;
        free[c.j][c.end_j as usize] = false;
        drafts[c.i].put(c.end_i, c.at);
        drafts[c.j].put(c.end_j, c.at);
    }

    for i in 0..drafts.len() {
        for end in [false, true] {
            if !free[i][end as usize] {
                continue;
            }
            let point = drafts[i].point(end);
            let mut nearest: Option<(f64, [f64; 2])> = None;
            for (j, other) in drafts.iter().enumerate() {
                if i == j {
                    continue;
                }
                let Some(at) = drafts[i].crossing(other) else {
                    continue;
                };
                let gap = distance(point, at);
                let u = other.frame.u(at);
                if gap <= CORNER
                    && u >= other.start - REACH_MARGIN
                    && u <= other.end + REACH_MARGIN
                    && nearest.is_none_or(|n| gap < n.0)
                {
                    nearest = Some((gap, at));
                }
            }
            if let Some((_, at)) = nearest {
                if drafts[i].length_with(end, at) >= MIN_CLOSED {
                    drafts[i].put(end, at);
                }
            }
        }
    }
}

/// For every draft whether its room lies on the side of `frame.n`: the side from which the
/// trajectory sees most of the face. Counting poses alone would fail at a wall between two
/// rooms, whose bigger room is on the wrong side for one of its two faces.
fn inward_sides(ctx: &Search, drafts: &[Draft]) -> Vec<bool> {
    // Cell -> face; a cell near two faces belongs to the nearer one.
    let mut owners: Vec<(u32, f64, u32)> = Vec::new();
    for (index, draft) in drafts.iter().enumerate() {
        for &c in &draft.cells {
            let cell = &ctx.cells[c as usize];
            let off = (draft.frame.v(cell.at) - draft.v).abs();
            owners.push((cell.index, off, index as u32));
        }
    }
    owners.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.2.cmp(&b.2)));
    owners.dedup_by_key(|o| o.0);
    let owner = |cell: u32| -> Option<u32> {
        owners
            .binary_search_by_key(&cell, |o| o.0)
            .ok()
            .map(|at| owners[at].2)
    };

    let stride = (ctx.points.poses.len() / MAX_POSES).max(1);
    let poses: Vec<[f64; 2]> = ctx
        .points
        .poses
        .iter()
        .step_by(stride)
        .map(|p| [p[0] as f64, p[1] as f64])
        .collect();
    let rays: Vec<[f64; 2]> = (0..RAYS)
        .map(|k| {
            let (s, c) = (k as f64 * 360.0 / RAYS as f64).to_radians().sin_cos();
            [c, s]
        })
        .collect();

    let mut hits: Vec<(u32, bool, u32)> = poses
        .par_iter()
        .flat_map_iter(|&pose| {
            let mut out = Vec::new();
            for ray in &rays {
                let Some(cell) = first_wall(ctx, pose, *ray) else {
                    continue;
                };
                if let Some(face) = owner(cell) {
                    let draft = &drafts[face as usize];
                    out.push((face, draft.frame.v(pose) >= draft.v, cell));
                }
            }
            out
        })
        .collect();
    hits.par_sort_unstable();
    hits.dedup();

    let mut seen = vec![[0u32; 2]; drafts.len()];
    for &(face, positive, _) in &hits {
        seen[face as usize][positive as usize] += 1;
    }
    drafts
        .iter()
        .zip(&seen)
        .map(|(draft, sides)| {
            if sides[0] != sides[1] {
                return sides[1] > sides[0];
            }
            // Not seen at all: the side with more poses.
            let above = poses
                .iter()
                .filter(|&&p| draft.frame.v(p) >= draft.v)
                .count();
            above * 2 >= poses.len()
        })
        .collect()
}

/// The first wall cell a ray from `pose` meets.
fn first_wall(ctx: &Search, pose: [f64; 2], ray: [f64; 2]) -> Option<u32> {
    let step = ctx.cell / 2.0;
    let mut t = RAY_START;
    while t < RAY_LENGTH {
        let p = [pose[0] + ray[0] * t, pose[1] + ray[1] * t];
        if let Some(index) = ctx.grid.index_at(p) {
            if ctx.wall[index] {
                return Some(index as u32);
            }
        }
        t += step;
    }
    None
}

/// What became of the faces cut by [`cut_faces`].
#[derive(Debug)]
pub(crate) struct CutFaces {
    /// For every old face its pieces, in order.
    pieces: Vec<Vec<CutPiece>>,
}

/// A piece of an old face.
#[derive(Debug, Clone, Copy)]
struct CutPiece {
    /// Index of the new face.
    face: usize,
    /// The start of the new face on the old one (metres).
    a_on_old: f64,
    /// The stretch of the old face it was cut from (metres).
    from: f64,
    to: f64,
}

impl CutFaces {
    /// Moves an opening onto the piece that holds it; `false` if there is none any more.
    pub(crate) fn place(&self, opening: &mut Opening) -> bool {
        let middle = (opening.from + opening.to) / 2.0;
        let piece = self.pieces[opening.face]
            .iter()
            .find(|p| middle >= p.from && middle <= p.to);
        match piece {
            Some(p) => {
                opening.face = p.face;
                opening.from = (opening.from - p.a_on_old).max(0.0);
                opening.to -= p.a_on_old;
                true
            }
            None => false,
        }
    }
}

/// The new end of a face where it was cut: runs onto the face behind it, if one is that close.
fn close_cut_end(faces: &mut [WallFace], i: usize, at_b: bool) {
    let face = faces[i];
    let Some((d, len)) = direction(face.a, face.b) else {
        return;
    };
    let end = if at_b { face.b } else { face.a };
    let outward = if at_b { d } else { [-d[0], -d[1]] };
    let mut nearest: Option<(f64, usize, [f64; 2])> = None;
    for (j, other) in faces.iter().enumerate() {
        let Some((e, other_len)) = direction(other.a, other.b) else {
            continue;
        };
        if i == j || cross(d, e).abs() < CROSSING_SIN || dot(other.inward, outward) >= 0.0 {
            continue;
        }
        let t = cross([other.a[0] - end[0], other.a[1] - end[1]], e) / cross(d, e);
        let at = [end[0] + d[0] * t, end[1] + d[1] * t];
        let u = dot(e, [at[0] - other.a[0], at[1] - other.a[1]]);
        let gap = t.abs();
        if gap <= CORNER
            && u >= -REACH_MARGIN
            && u <= other_len + REACH_MARGIN
            && nearest.is_none_or(|n| (gap, j) < (n.0, n.1))
        {
            nearest = Some((gap, j, at));
        }
    }
    let Some((_, _, at)) = nearest else {
        return;
    };
    let kept = if at_b { face.a } else { face.b };
    if distance(kept, at) < MIN_CLOSED || distance(kept, at) > len + CORNER {
        return;
    }
    if at_b {
        faces[i].b = at;
    } else {
        faces[i].a = at;
    }
}

/// Cuts every face at its stretches in `cuts` (metres from `a`), closes the new ends onto what
/// runs behind them and pairs the faces anew. The faces keep their order, the pieces of one in
/// a row; pieces shorter than `MIN_FACE` go.
pub(crate) fn cut_faces(walls: &mut Walls, cuts: &[Vec<(f64, f64)>]) -> CutFaces {
    let old_faces = std::mem::take(&mut walls.faces);
    let mut new_faces: Vec<WallFace> = Vec::with_capacity(old_faces.len());
    // New face -> (old face, from, to, cut at a, cut at b).
    let mut origins: Vec<(usize, f64, f64, bool, bool)> = Vec::with_capacity(old_faces.len());
    for (index, (face, stretches)) in old_faces.iter().zip(cuts).enumerate() {
        let Some((d, len)) = direction(face.a, face.b).filter(|_| !stretches.is_empty()) else {
            new_faces.push(*face);
            origins.push((index, 0.0, distance(face.a, face.b), false, false));
            continue;
        };
        let mut sorted = stretches.clone();
        sorted.sort_by(|p, q| p.0.total_cmp(&q.0).then(p.1.total_cmp(&q.1)));
        let mut pieces: Vec<(f64, f64, bool, bool)> = Vec::new();
        let (mut start, mut cut_at_start) = (0.0_f64, false);
        for (from, to) in sorted {
            if from > start {
                pieces.push((start, from.min(len), cut_at_start, true));
            }
            start = start.max(to);
            cut_at_start = true;
        }
        if start < len {
            pieces.push((start, len, cut_at_start, false));
        }
        let kept: f64 = pieces.iter().map(|p| p.1 - p.0).sum();
        for (from, to, cut_a, cut_b) in pieces {
            let at = |t: f64| [face.a[0] + d[0] * t, face.a[1] + d[1] * t];
            let piece_len = to - from;
            new_faces.push(WallFace {
                a: at(from),
                b: at(to),
                inward: face.inward,
                seen_m: if kept > 0.0 {
                    (face.seen_m * piece_len / kept).min(piece_len)
                } else {
                    0.0
                },
            });
            origins.push((index, from, to, cut_a, cut_b));
        }
    }

    let ends: Vec<(usize, bool)> = origins
        .iter()
        .enumerate()
        .flat_map(|(i, o)| [(o.3, (i, false)), (o.4, (i, true))])
        .filter(|(cut, _)| *cut)
        .map(|(_, end)| end)
        .collect();
    for (i, at_b) in ends {
        close_cut_end(&mut new_faces, i, at_b);
    }

    let mut pieces: Vec<Vec<CutPiece>> = vec![Vec::new(); old_faces.len()];
    let mut faces: Vec<WallFace> = Vec::with_capacity(new_faces.len());
    for (face, (old, from, to, cut_a, cut_b)) in new_faces.into_iter().zip(origins) {
        if (cut_a || cut_b) && distance(face.a, face.b) < MIN_FACE {
            continue;
        }
        let original = &old_faces[old];
        let a_on_old = direction(original.a, original.b).map_or(0.0, |(d, _)| {
            dot(d, [face.a[0] - original.a[0], face.a[1] - original.a[1]])
        });
        pieces[old].push(CutPiece {
            face: faces.len(),
            a_on_old,
            from,
            to,
        });
        faces.push(face);
    }
    walls.pairs = find_pairs(&faces);
    walls.faces = faces;
    CutFaces { pieces }
}

fn direction(a: [f64; 2], b: [f64; 2]) -> Option<([f64; 2], f64)> {
    let len = distance(a, b);
    (len > 0.0).then(|| ([(b[0] - a[0]) / len, (b[1] - a[1]) / len], len))
}

/// Pairs of faces that are the two sides of one wall. A long face can have several partners, one
/// per stretch, when the other side consists of faces lying at different depths. The longest
/// common stretch goes first, then the thinnest: a door leaf set back in one of the faces must
/// not take the stretch of the wall's real partner.
fn find_pairs(faces: &[WallFace]) -> Vec<WallPair> {
    let unit = |f: &WallFace| {
        let len = distance(f.a, f.b);
        ([(f.b[0] - f.a[0]) / len, (f.b[1] - f.a[1]) / len], len)
    };
    let middle = |f: &WallFace| [(f.a[0] + f.b[0]) / 2.0, (f.a[1] + f.b[1]) / 2.0];
    let along = |p: [f64; 2], f: &WallFace, d: [f64; 2]| dot(d, [p[0] - f.a[0], p[1] - f.a[1]]);
    let above = |p: [f64; 2], f: &WallFace| dot(f.inward, [p[0] - f.a[0], p[1] - f.a[1]]);

    let mut found = Vec::new();
    for (i, fi) in faces.iter().enumerate() {
        let (di, len_i) = unit(fi);
        for (j, fj) in faces.iter().enumerate().skip(i + 1) {
            let (dj, len_j) = unit(fj);
            if cross(di, dj).abs() > PARALLEL_SIN || dot(fi.inward, fj.inward) >= 0.0 {
                continue;
            }
            let (behind_i, behind_j) = (above(middle(fj), fi), above(middle(fi), fj));
            if behind_i >= 0.0 || behind_j >= 0.0 {
                continue;
            }
            let thickness = -(behind_i + behind_j) / 2.0;
            if !(PAIR_THICKNESS.0..=PAIR_THICKNESS.1).contains(&thickness) {
                continue;
            }
            // The stretch where they overlap, on each of them.
            let (from, to) = (along(fj.a, fi, di), along(fj.b, fi, di));
            let stretch_i = (to.min(from).max(0.0), to.max(from).min(len_i));
            let overlap = stretch_i.1 - stretch_i.0;
            if overlap >= PAIR_OVERLAP * len_i.min(len_j) {
                let (from, to) = (along(fi.a, fj, dj), along(fi.b, fj, dj));
                let stretch_j = (to.min(from).max(0.0), to.max(from).min(len_j));
                found.push((
                    overlap,
                    stretch_i,
                    stretch_j,
                    WallPair {
                        front: i,
                        back: j,
                        thickness,
                    },
                ));
            }
        }
    }
    found.sort_by(|a, b| {
        b.0.total_cmp(&a.0)
            .then(a.3.thickness.total_cmp(&b.3.thickness))
            .then((a.3.front, a.3.back).cmp(&(b.3.front, b.3.back)))
    });
    let mut taken: Vec<Vec<(f64, f64)>> = vec![Vec::new(); faces.len()];
    found
        .into_iter()
        .filter_map(|(_, stretch_i, stretch_j, pair)| {
            let free = |taken: &[(f64, f64)], stretch: (f64, f64)| {
                let used: f64 = taken
                    .iter()
                    .map(|t| (t.1.min(stretch.1) - t.0.max(stretch.0)).max(0.0))
                    .sum();
                used <= PAIR_SHARED * (stretch.1 - stretch.0)
            };
            if free(&taken[pair.front], stretch_i) && free(&taken[pair.back], stretch_j) {
                taken[pair.front].push(stretch_i);
                taken[pair.back].push(stretch_j);
                Some(pair)
            } else {
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::Scene;
    use crate::{find_storeys, storey_points, Leveling};

    const FLOOR: f64 = -1.3;
    const WIDE: f64 = 8.3;
    const DEEP: f64 = 7.9;

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

    fn storey_of(scene: Scene) -> StoreyPoints {
        let data = scene.build();
        let input = data.input();
        let found = find_storeys(&input, &Progress::new()).unwrap();
        storey_points(&input, &found.storeys[0], &Leveling::NONE)
    }

    fn walls_of(scene: Scene) -> Walls {
        find_walls(&storey_of(scene), &PlanConfig::default(), &Progress::new()).unwrap()
    }

    fn length(f: &WallFace) -> f64 {
        (f.b[0] - f.a[0]).hypot(f.b[1] - f.a[1])
    }

    fn direction(f: &WallFace) -> [f64; 2] {
        let len = length(f);
        [(f.b[0] - f.a[0]) / len, (f.b[1] - f.a[1]) / len]
    }

    fn middle(f: &WallFace) -> [f64; 2] {
        [(f.a[0] + f.b[0]) / 2.0, (f.a[1] + f.b[1]) / 2.0]
    }

    fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
        a[0] * b[0] + a[1] * b[1]
    }

    fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
        a[0] * b[1] - a[1] * b[0]
    }

    fn assert_lengths(walls: &Walls, expected: &[f64], tol: f64) {
        let mut got: Vec<f64> = walls.faces.iter().map(length).collect();
        got.sort_by(f64::total_cmp);
        assert_eq!(got.len(), expected.len(), "{got:?}");
        for (g, e) in got.iter().zip(expected) {
            assert!((g - e).abs() <= tol, "{got:?} statt {expected:?}");
        }
    }

    /// The nearest other face end to each face end, as the index of the face and the distance.
    fn neighbours(walls: &Walls) -> Vec<(usize, f64)> {
        let ends: Vec<(usize, [f64; 2])> = walls
            .faces
            .iter()
            .enumerate()
            .flat_map(|(i, f)| [(i, f.a), (i, f.b)])
            .collect();
        ends.iter()
            .map(|&(i, p)| {
                ends.iter()
                    .filter(|e| e.0 != i)
                    .map(|&(j, q)| (j, (p[0] - q[0]).hypot(p[1] - q[1])))
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap()
            })
            .collect()
    }

    fn assert_closed(walls: &Walls, tol: f64) {
        for (j, gap) in neighbours(walls) {
            assert!(gap <= tol, "Ecke an Fläche {j} offen: {gap} m");
        }
    }

    /// Every face end lies on another face (a corner or a wall meeting another one).
    fn assert_touching(walls: &Walls, tol: f64) {
        let on_segment = |p: [f64; 2], f: &WallFace| {
            let d = direction(f);
            let t = dot(d, [p[0] - f.a[0], p[1] - f.a[1]]).clamp(0.0, length(f));
            (p[0] - (f.a[0] + d[0] * t)).hypot(p[1] - (f.a[1] + d[1] * t))
        };
        for (i, f) in walls.faces.iter().enumerate() {
            for end in [f.a, f.b] {
                let gap = walls
                    .faces
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .map(|(_, g)| on_segment(end, g))
                    .fold(f64::MAX, f64::min);
                assert!(gap <= tol, "Ende von Fläche {i} offen: {gap} m");
            }
        }
    }

    #[test]
    fn box_room() {
        let walls = walls_of(room());
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
        assert_eq!(walls.cell, 0.02);
    }

    #[test]
    fn rotated_room_keeps_right_angles() {
        let walls = walls_of(room().yaw(23.0));
        let angle = walls.axis[1].atan2(walls.axis[0]).to_degrees();
        assert!((angle - 23.0).abs() <= 0.2, "{angle}");
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
        for (i, f) in walls.faces.iter().enumerate() {
            for g in &walls.faces[i + 1..] {
                let d = dot(direction(f), direction(g)).abs();
                assert!(d < 1e-9 || (d - 1.0).abs() < 1e-9, "{d}");
            }
        }
    }

    #[test]
    fn l_shaped_room() {
        let corners = [
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 3.0],
            [3.0, 3.0],
            [3.0, 6.0],
            [0.0, 6.0],
        ];
        let way = [
            [1.0, 1.0, 0.0],
            [5.0, 1.0, 0.0],
            [5.0, 2.0, 0.0],
            [1.5, 2.0, 0.0],
            [1.5, 5.0, 0.0],
        ];
        let walls = walls_of(Scene::new().polygon(&corners, FLOOR, 2.6).walk(&way));
        assert_lengths(&walls, &[3.0, 3.0, 3.0, 3.0, 6.0, 6.0], 0.01);
        assert_closed(&walls, 0.001);
    }

    /// Two rooms next to each other, a 12 cm wall between them, a door through it.
    fn two_rooms() -> Scene {
        Scene::new()
            .room([0.0, 0.0], [4.0, 4.0], FLOOR, 2.6)
            .door(1, 1.5, 0.9, 2.05)
            .room([4.12, 0.0], [8.12, 4.0], FLOOR, 2.6)
            .door(3, 1.6, 0.9, 2.05)
            .walk(&[
                [1.0, 1.0, 0.0],
                [1.0, 3.0, 0.0],
                [2.0, 1.95, 0.0],
                [6.0, 1.95, 0.0],
                [7.0, 3.0, 0.0],
                [7.0, 1.0, 0.0],
            ])
    }

    #[test]
    fn shared_wall_has_thickness() {
        let walls = walls_of(two_rooms());
        assert_eq!(walls.pairs.len(), 1, "{:?}", walls.faces);
        let pair = walls.pairs[0];
        assert!((pair.thickness - 0.12).abs() <= 0.01, "{}", pair.thickness);
        // The two faces look away from each other.
        let (front, back) = (&walls.faces[pair.front], &walls.faces[pair.back]);
        assert!(dot(front.inward, back.inward) < -0.99);
    }

    #[test]
    fn furniture_is_no_wall() {
        let points = storey_of(
            room()
                .block([2.0, 2.0, FLOOR], [3.2, 3.0, FLOOR + 1.2])
                .block([5.0, 4.0, FLOOR], [5.8, 6.0, FLOOR + 1.2]),
        );
        let walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.02);
        for f in &walls.faces {
            let m = middle(f);
            let (x, y) = (m[0] + points.origin[0], m[1] + points.origin[1]);
            let inside = x > 0.3 && x < WIDE - 0.3 && y > 0.3 && y < DEEP - 0.3;
            assert!(!inside, "Möbel als Wand: {f:?}");
        }
    }

    #[test]
    fn low_cupboard_at_the_wall_is_no_wall() {
        let walls = walls_of(room().block([3.0, 0.0, FLOOR], [4.5, 0.6, FLOOR + 1.2]));
        assert_eq!(walls.faces.len(), 4, "{:?}", walls.faces);
    }

    fn slanted_room() -> Scene {
        // The wall from (2.3094, 4) to (0, 0) runs at 60° to the x axis.
        let corners = [[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [2.3094, 4.0]];
        Scene::new().polygon(&corners, FLOOR, 2.6).walk(&[
            [2.0, 1.0, 0.0],
            [5.0, 1.0, 0.0],
            [5.0, 3.0, 0.0],
            [3.0, 3.0, 0.0],
        ])
    }

    #[test]
    fn slanted_wall_stays_slanted() {
        let walls = walls_of(slanted_room());
        assert_eq!(walls.faces.len(), 4, "{:?}", walls.faces);
        assert_closed(&walls, 0.001);
        let slanted = walls
            .faces
            .iter()
            .find(|f| direction(f)[0].abs() > 0.1 && direction(f)[1].abs() > 0.1)
            .expect("schräge Wand fehlt");
        let (s, c) = 60f64.to_radians().sin_cos();
        let off = cross(direction(slanted), [c, s]).abs();
        assert!(off < 0.5f64.to_radians().sin(), "{off}");
        let len = length(slanted);
        assert!((len - 4.0 / s).abs() <= 0.03, "{len}");
    }

    #[test]
    fn nearly_aligned_wall_is_laid_on_the_main_direction() {
        // The east wall leans by atan(0.16 / 6) = 1.53°.
        let corners = [[0.0, 0.0], [8.0, 0.0], [8.16, 6.0], [0.0, 6.0]];
        let way = [
            [1.0, 1.0, 0.0],
            [7.0, 1.0, 0.0],
            [7.0, 5.0, 0.0],
            [1.0, 5.0, 0.0],
        ];
        let walls = walls_of(Scene::new().polygon(&corners, FLOOR, 2.6).walk(&way));
        assert_eq!(walls.faces.len(), 4, "{:?}", walls.faces);
        assert_closed(&walls, 0.001);
        for f in &walls.faces {
            let along = cross(direction(f), walls.axis).abs();
            let across = dot(direction(f), walls.axis).abs();
            assert!(along < 1e-9 || across < 1e-9, "{f:?}");
        }
    }

    #[test]
    fn door_gap_stays_one_face() {
        let walls = walls_of(room().door(0, 2.0, 0.9, 2.05));
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
        let south = walls
            .faces
            .iter()
            .filter(|f| (length(f) - WIDE).abs() < 0.05)
            .map(|f| f.seen_m)
            .fold(f64::MAX, f64::min);
        assert!((south - (WIDE - 0.9)).abs() < 0.2, "{south}");
    }

    #[test]
    fn door_beside_a_corner_keeps_the_corner() {
        let walls = walls_of(room().door(0, 0.15, 0.9, 2.05));
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
    }

    #[test]
    fn window_does_not_cut_the_wall() {
        let walls = walls_of(room().window(1, 2.0, 1.4, 0.9, 1.3));
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
        for f in &walls.faces {
            assert!(length(f) - f.seen_m < 0.2, "{f:?}");
        }
    }

    #[test]
    fn wall_in_line_with_another_room_does_not_stretch_it() {
        // A sheet crossing the line of the south wall, 1.5 m beyond its end.
        let walls =
            walls_of(room().behind([WIDE + 1.5, -2.0, FLOOR], [WIDE + 1.5, 2.0, FLOOR + 2.6]));
        let east = walls
            .faces
            .iter()
            .map(length)
            .filter(|l| (l - 4.0).abs() < 0.1)
            .count();
        assert_eq!(east, 1, "{:?}", walls.faces);
        assert_eq!(walls.faces.len(), 5, "{:?}", walls.faces);
        assert_lengths(&walls, &[4.0, DEEP, DEEP, WIDE, WIDE], 0.02);
    }

    #[test]
    fn noisy_and_sparse() {
        let walls = walls_of(room().noise(0.005).spacing(0.02));
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.02);
    }

    #[test]
    fn inward_points_to_the_walk() {
        let points = storey_of(room());
        let walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        assert_eq!(walls.faces.len(), 4);
        let centre = [WIDE / 2.0 - points.origin[0], DEEP / 2.0 - points.origin[1]];
        for f in &walls.faces {
            let m = middle(f);
            assert!(dot(f.inward, [centre[0] - m[0], centre[1] - m[1]]) > 0.0);
            // The room lies on the left of a → b.
            let d = direction(f);
            assert!((f.inward[0] + d[1]).abs() < 1e-12 && (f.inward[1] - d[0]).abs() < 1e-12);
        }
    }

    #[test]
    fn inward_of_a_shared_wall_points_to_its_room() {
        let points = storey_of(two_rooms());
        let walls = find_walls(&points, &PlanConfig::default(), &Progress::new()).unwrap();
        for f in &walls.faces {
            if direction(f)[1].abs() < 0.9 {
                continue;
            }
            let x = middle(f)[0] + points.origin[0];
            let expected = if x < 2.0 || (4.05..7.0).contains(&x) {
                1.0
            } else {
                -1.0
            };
            assert_eq!(f.inward[0].signum(), expected, "{f:?}");
        }
    }

    #[test]
    fn rotated_and_noisy_rooms_stay_clean() {
        let turned = walls_of(two_rooms().noise(0.005).spacing(0.02).yaw(37.3));
        assert_eq!(turned.faces.len(), 6, "{:?}", turned.faces);
        assert_eq!(turned.pairs.len(), 1, "{:?}", turned.pairs);
        assert!((turned.pairs[0].thickness - 0.12).abs() <= 0.015);
        assert_touching(&turned, 0.001);
        let angle = turned.axis[1].atan2(turned.axis[0]).to_degrees();
        assert!((angle - 37.3).abs() <= 0.2, "{angle}");

        let lean = walls_of(slanted_room().yaw(15.0).noise(0.003));
        assert_eq!(lean.faces.len(), 4, "{:?}", lean.faces);
        assert_closed(&lean, 0.001);
    }

    #[test]
    fn room_without_ceiling_uses_the_lower_band() {
        let walls = walls_of(
            Scene::new()
                .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
                .open_top()
                .walk(&walk()),
        );
        assert_lengths(&walls, &[DEEP, DEEP, WIDE, WIDE], 0.01);
        assert_closed(&walls, 0.001);
    }

    #[test]
    fn same_input_same_result() {
        let scene = || room().door(0, 2.0, 0.9, 2.05).noise(0.005).spacing(0.02);
        assert_eq!(walls_of(scene()), walls_of(scene()));
    }

    #[test]
    fn cancel_stops() {
        let points = storey_of(room().spacing(0.05));
        let progress = Progress::new();
        progress.cancel();
        assert_eq!(
            find_walls(&points, &PlanConfig::default(), &progress),
            Err(PlanError::Cancelled)
        );
    }
}
