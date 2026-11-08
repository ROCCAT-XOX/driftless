//! Storeys: where the floor and the ceiling are.
//!
//! For every height step (1 cm) the horizontal area with points is counted, in columns of
//! 10 cm. A column only counts where its points form a short run in height, so that walls
//! (long runs) do not count and floor and ceiling stand out. Neighbouring height steps with
//! area form one surface; a slightly tilted floor is one surface over a few steps.
//!
//! The trajectory says where somebody stood: the floor is the lowest big surface 0.5 to 2.2 m
//! below a group of poses, the ceiling the lowest big surface above it. Surfaces are compared
//! within their window, so that a table is no floor and a lamp no ceiling.

use driftless_io::progress::Progress;
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{PlanError, PlanInput};

/// Height step of the area profile (metres).
const Z_STEP: f64 = 0.01;
/// Edge of a column (metres).
const COLUMN: f64 = 0.10;
const COLUMN_AREA: f64 = COLUMN * COLUMN;
/// Height steps of one run may be this far apart (a sparse wall still is one run).
const RUN_GAP: u32 = 10;
/// A run spanning more height steps than this is vertical, not a surface.
const RUN_FLAT: u32 = 20;
/// A height step belongs to a surface from this many columns (0.25 m²) ...
const SURFACE_COLUMNS: u32 = 25;
/// ... and this share of the fullest step of the surface: a surface ends at a valley.
const SURFACE_PEAK_SHARE: f64 = 0.05;
/// A run counts if this many of the 8 neighbouring columns have a flat run this near in height
/// (steps): a real surface is a patch, a stray point stands alone.
const SUPPORT_COLUMNS: u32 = 2;
const SUPPORT_REACH: u32 = 3;
const MIN_POINTS: usize = 1000;

/// Floor window below the median of a group of poses (metres).
const FLOOR_BELOW: (f64, f64) = (0.5, 2.2);
/// A floor or ceiling has at least this area (m²) ...
const MIN_AREA: f64 = 1.5;
/// ... and at least this share of the largest surface in its window.
const BIG_SHARE: f64 = 0.40;
/// The ceiling starts this far above the poses and ends this far above the floor (metres).
const CEILING_ABOVE_POSES: f64 = 0.1;
const CEILING_MAX_ABOVE_FLOOR: f64 = 15.0;
/// A ceiling may spread this much more in height than the floor (else: a roof slope).
const CEILING_EXTRA_SPREAD: f64 = 0.30;
/// Floors closer than this are one storey (metres).
const MIN_STOREY_DISTANCE: f64 = 1.0;
/// A step in the floor, in metres.
const STEP_RANGE: (f64, f64) = (0.05, 0.50);
/// Surfaces are compared by their area within this many height steps: real surfaces hold most
/// of their columns there, a band of clutter all over the height only little of them.
const CORE_STEPS: usize = 10;
/// A surface spreading over more height than this is no step (metres).
const STEP_MAX_SPREAD: f64 = 0.30;
/// A surface below the lowest floor is worth a hint from this depth and share of its area.
const BELOW_DEPTH: f64 = 1.0;
const BELOW_SHARE: f64 = 0.25;

/// Height bin of the trajectory (metres).
const WALK_BIN: f64 = 0.10;
/// A bin belongs to a group of poses from this share of the poses ...
const WALK_BIN_SHARE: f64 = 0.015;
/// ... and a group counts from this share.
const WALK_GROUP_SHARE: f64 = 0.10;

/// Half-widths of the bands around the floor plane, one per round of the fit (metres).
const LEVEL_BANDS: [f64; 4] = [0.30, 0.10, 0.05, 0.03];
const LEVEL_MAX_SPREAD: f64 = 0.10;
const LEVEL_MIN_COLUMNS: usize = 20;
/// Tilts below this are left alone (degrees).
const MIN_TILT_DEG: f64 = 0.3;

/// Without a ceiling, the points of a storey end this far above the floor (metres).
const NO_CEILING_TOP: f64 = 3.5;
/// The points of a storey reach this far below the floor and above the ceiling (metres).
const STOREY_MARGIN: f64 = 0.10;

/// A storey of a map (heights in the frame of the map).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Storey {
    pub floor_z: f64,
    /// `None` if there is no flat ceiling (roof slope, open hall).
    pub ceiling_z: Option<f64>,
    /// Share of the trajectory on this storey, 0 to 1.
    pub pose_share: f64,
    /// Measured floor area (m²).
    pub floor_area: f64,
}

/// A rotation about `pivot` that makes the floor level.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Leveling {
    /// Unit normal of the floor, pointing up.
    pub normal: [f64; 3],
    pub pivot: [f64; 3],
    pub tilt_deg: f64,
}

impl Leveling {
    /// The cloud stays as it is.
    pub const NONE: Leveling = Leveling {
        normal: [0.0, 0.0, 1.0],
        pivot: [0.0, 0.0, 0.0],
        tilt_deg: 0.0,
    };

    /// The rotation taking `normal` to +Z, without an angle function; `None` for [`Self::NONE`].
    fn rotation(&self) -> Option<Matrix3<f64>> {
        if *self == Leveling::NONE {
            return None;
        }
        let n = Vector3::from(self.normal);
        let skew = n.cross(&Vector3::z()).cross_matrix();
        Some(Matrix3::identity() + skew + skew * skew / (1.0 + n.z))
    }

    /// From the frame of the map into the levelled frame.
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        match self.rotation() {
            Some(r) => turn(&r, p, self.pivot),
            None => p,
        }
    }

    /// From the levelled frame back into the frame of the map.
    pub fn undo(&self, p: [f64; 3]) -> [f64; 3] {
        match self.rotation() {
            Some(r) => turn(&r.transpose(), p, self.pivot),
            None => p,
        }
    }
}

fn turn(r: &Matrix3<f64>, p: [f64; 3], pivot: [f64; 3]) -> [f64; 3] {
    let c = Vector3::from(pivot);
    (r * (Vector3::from(p) - c) + c).into()
}

/// What [`find_storeys`] found.
#[derive(Debug, Clone, PartialEq)]
pub struct StoreySearch {
    /// Ascending by `floor_z`.
    pub storeys: Vec<Storey>,
    pub hints: Vec<String>,
}

/// The points and poses of one storey, levelled, relative to `origin` (z above the floor).
#[derive(Debug, Clone, PartialEq)]
pub struct StoreyPoints {
    pub xyz: Vec<[f32; 3]>,
    pub rgb: Option<Vec<[u16; 3]>>,
    pub poses: Vec<[f32; 3]>,
    /// In the levelled frame: `xyz + origin` is the levelled position.
    pub origin: [f64; 3],
    pub leveling: Leveling,
    /// Clear height, if the storey has a ceiling.
    pub height: Option<f32>,
}

/// A flat surface: neighbouring height steps with area.
#[derive(Debug, Clone, Copy)]
struct Surface {
    /// Lower and upper edge (metres).
    low: f64,
    high: f64,
    /// Height of its area's centre (metres).
    z: f64,
    /// Columns it covers, each counted once (m²).
    area: f64,
    /// Columns within its fullest [`CORE_STEPS`] height steps, each counted once (m²).
    core: f64,
}

/// A height at which many poses are.
#[derive(Debug, Clone, Copy)]
struct WalkGroup {
    z: f64,
    count: usize,
}

pub(crate) fn check(progress: &Progress) -> Result<(), PlanError> {
    if progress.is_cancelled() {
        Err(PlanError::Cancelled)
    } else {
        Ok(())
    }
}

fn is_finite(p: &[f64; 3]) -> bool {
    p.iter().all(|c| c.is_finite())
}

/// Finds the storeys of a cloud from the height of its flat surfaces and the trajectory.
pub fn find_storeys(input: &PlanInput, progress: &Progress) -> Result<StoreySearch, PlanError> {
    progress.phase("Stockwerke", Some(3));
    check(progress)?;
    if input.xyz.len() < MIN_POINTS {
        return Err(PlanError::TooFewPoints);
    }
    let total = input.poses.iter().filter(|p| is_finite(p)).count();
    if total == 0 {
        return Err(PlanError::NoPoses);
    }
    let surfaces = flat_surfaces(input.xyz, progress)?;
    progress.advance(1);
    check(progress)?;

    let groups = walk_groups(input.poses);

    // From below, so that the ceiling of a lower storey is known when the floor above it is
    // searched: seen from above, it lies in the window of the upper floor.
    let mut ceilings: Vec<usize> = Vec::new();
    let mut storeys: Vec<Found> = Vec::new();
    for group in &groups {
        let Some(floor) = find_floor(&surfaces, group, &ceilings) else {
            continue;
        };
        let ceiling = find_ceiling(&surfaces, group, floor);
        if let Some(c) = ceiling {
            ceilings.push(c);
        }
        storeys.push(Found {
            floor,
            ceiling,
            poses: group.count,
        });
    }
    progress.advance(1);
    check(progress)?;

    let storeys = merge_close_floors(storeys, &surfaces);
    if storeys.is_empty() {
        return Err(PlanError::NoFloor);
    }
    let floors: Vec<usize> = storeys.iter().map(|s| s.floor).collect();
    let mut hints = Vec::new();
    let mut result = Vec::with_capacity(storeys.len());
    for (nr, found) in storeys.iter().enumerate() {
        let label = (storeys.len() > 1).then_some(nr + 1);
        step_hints(
            &surfaces,
            found.floor,
            &floors,
            &ceilings,
            label,
            &mut hints,
        );
        result.push(Storey {
            floor_z: surface_height(input.xyz, &surfaces[found.floor]),
            ceiling_z: found
                .ceiling
                .map(|c| surface_height(input.xyz, &surfaces[c])),
            pose_share: found.poses as f64 / total as f64,
            floor_area: surfaces[found.floor].area,
        });
        check(progress)?;
    }
    below_hint(&surfaces, storeys[0].floor, &mut hints);
    progress.advance(1);
    Ok(StoreySearch {
        storeys: result,
        hints,
    })
}

/// A storey while searching: indices into the surfaces.
#[derive(Debug, Clone, Copy)]
struct Found {
    floor: usize,
    ceiling: Option<usize>,
    poses: usize,
}

/// The flat surfaces of the cloud, ascending by height.
fn flat_surfaces(xyz: &[[f64; 3]], progress: &Progress) -> Result<Vec<Surface>, PlanError> {
    let min = xyz
        .par_iter()
        .filter(|p| is_finite(p))
        .copied()
        .reduce_with(|a, b| [a[0].min(b[0]), a[1].min(b[1]), a[2].min(b[2])])
        .ok_or(PlanError::TooFewPoints)?;

    // Column in the upper bits, height step in the lower ones: sorted, the steps of a column
    // lie next to each other, ascending.
    const COLUMN_MAX: u64 = (1 << 21) - 1;
    const STEP_MAX: u64 = (1 << 22) - 1;
    let mut keys: Vec<u64> = xyz
        .par_iter()
        .filter(|p| is_finite(p))
        .map(|p| {
            let x = (((p[0] - min[0]) / COLUMN) as u64).min(COLUMN_MAX);
            let y = (((p[1] - min[1]) / COLUMN) as u64).min(COLUMN_MAX);
            let z = (((p[2] - min[2]) / Z_STEP) as u64).min(STEP_MAX);
            (x << 43) | (y << 22) | z
        })
        .collect();
    if keys.len() < MIN_POINTS {
        return Err(PlanError::TooFewPoints);
    }
    check(progress)?;
    keys.par_sort_unstable();
    keys.dedup();
    check(progress)?;

    let column_of = |key: u64| key >> 22;
    let step_of = |key: u64| (key & STEP_MAX) as u32;
    let steps = keys.iter().map(|&k| step_of(k)).max().unwrap_or(0) as usize + 1;

    // The flat runs of every column, ascending; only those with neighbours count.
    let mut runs: Vec<FlatRun> = Vec::new();
    let mut key_at = 0;
    for column in keys.chunk_by(|a, b| column_of(*a) == column_of(*b)) {
        let id = column_of(column[0]);
        for run in column.chunk_by(|a, b| step_of(*b) - step_of(*a) <= RUN_GAP) {
            let (first, last) = (step_of(run[0]), step_of(run[run.len() - 1]));
            if last - first <= RUN_FLAT {
                runs.push(FlatRun {
                    column: id,
                    first,
                    last,
                    keys: (key_at as u32, (key_at + run.len()) as u32),
                });
            }
            key_at += run.len();
        }
    }
    check(progress)?;
    let supported: Vec<bool> = runs.par_iter().map(|run| has_support(run, &runs)).collect();
    check(progress)?;

    // Calls `f(column, step)` for every occupied step of a counting run.
    let for_flat_steps = |f: &mut dyn FnMut(u64, u32)| {
        for (run, _) in runs.iter().zip(&supported).filter(|(_, ok)| **ok) {
            for &key in &keys[run.keys.0 as usize..run.keys.1 as usize] {
                f(run.column, step_of(key));
            }
        }
    };

    let mut profile = vec![0u32; steps];
    for_flat_steps(&mut |_, step| profile[step as usize] += 1);

    // Neighbouring steps with area form one surface; valleys separate surfaces.
    let mut spans: Vec<(usize, usize)> = Vec::new();
    let mut step = 0;
    while step < steps {
        if profile[step] < SURFACE_COLUMNS {
            step += 1;
            continue;
        }
        let first = step;
        while step < steps && profile[step] >= SURFACE_COLUMNS {
            step += 1;
        }
        split_at_valleys(&profile, first, step - 1, &mut spans);
    }
    spans.sort_unstable();

    const NO_SURFACE: u32 = u32::MAX;
    let mut surface_of = vec![NO_SURFACE; steps];
    for (index, &(first, last)) in spans.iter().enumerate() {
        surface_of[first..=last].fill(index as u32);
    }

    // The fullest `CORE_STEPS` steps of every surface, first and last step.
    let core_of: Vec<(usize, usize)> = spans
        .iter()
        .map(|&(first, last)| fullest_steps(&profile, first, last))
        .collect();

    // One column can have several steps of a surface: count it once.
    let mut last_column = vec![u64::MAX; spans.len()];
    let mut last_core = vec![u64::MAX; spans.len()];
    let mut columns = vec![0u64; spans.len()];
    let mut core_columns = vec![0u64; spans.len()];
    for_flat_steps(&mut |id, step| {
        let surface = surface_of[step as usize];
        if surface == NO_SURFACE {
            return;
        }
        let surface = surface as usize;
        if last_column[surface] != id {
            last_column[surface] = id;
            columns[surface] += 1;
        }
        let (from, to) = core_of[surface];
        if (from..=to).contains(&(step as usize)) && last_core[surface] != id {
            last_core[surface] = id;
            core_columns[surface] += 1;
        }
    });

    let step_z = |step: usize| min[2] + step as f64 * Z_STEP;
    Ok(spans
        .iter()
        .zip(columns.iter().zip(&core_columns))
        .map(|(&(first, last), (&count, &core))| {
            let bins = &profile[first..=last];
            let weight: f64 = bins.iter().map(|&c| c as f64).sum();
            let moment: f64 = bins
                .iter()
                .enumerate()
                .map(|(i, &c)| c as f64 * (first + i) as f64)
                .sum();
            Surface {
                low: step_z(first),
                high: step_z(last + 1),
                z: min[2] + (moment / weight + 0.5) * Z_STEP,
                area: count as f64 * COLUMN_AREA,
                core: core as f64 * COLUMN_AREA,
            }
        })
        .collect())
}

/// A short run of height steps with points in one column.
#[derive(Debug, Clone)]
struct FlatRun {
    column: u64,
    first: u32,
    last: u32,
    /// Its entries in the sorted keys: start, end.
    keys: (u32, u32),
}

/// Whether enough neighbouring columns have a flat run near this one in height. `runs` is
/// sorted by column.
fn has_support(run: &FlatRun, runs: &[FlatRun]) -> bool {
    const COORD: u64 = (1 << 21) - 1;
    let (x, y) = ((run.column >> 21) as i64, (run.column & COORD) as i64);
    let near = |other: &FlatRun| {
        other.first <= run.last + SUPPORT_REACH && run.first <= other.last + SUPPORT_REACH
    };
    let mut support = 0;
    for dy in -1..=1 {
        for dx in -1..=1 {
            let (nx, ny) = (x + dx, y + dy);
            if (dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx as u64 > COORD || ny as u64 > COORD {
                continue;
            }
            let id = ((nx as u64) << 21) | ny as u64;
            let at = runs.partition_point(|r| r.column < id);
            if runs[at..].iter().take_while(|r| r.column == id).any(near) {
                support += 1;
                if support >= SUPPORT_COLUMNS {
                    return true;
                }
            }
        }
    }
    false
}

/// The first and last step of the `CORE_STEPS` fullest neighbouring steps of a surface.
fn fullest_steps(profile: &[u32], first: usize, last: usize) -> (usize, usize) {
    let bins = &profile[first..=last];
    let width = CORE_STEPS.min(bins.len());
    let mut best = (0u64, 0usize);
    let mut sum: u64 = bins[..width].iter().map(|&c| c as u64).sum();
    best.0 = sum;
    for start in 1..=bins.len() - width {
        sum += bins[start + width - 1] as u64;
        sum -= bins[start - 1] as u64;
        if sum > best.0 {
            best = (sum, start);
        }
    }
    (first + best.1, first + best.1 + width - 1)
}

/// Splits the stretch `first..=last` of steps with area into surfaces: a surface is the fullest
/// step and what goes on from it while it stays above [`SURFACE_PEAK_SHARE`] of that step (and
/// [`SURFACE_COLUMNS`]). What is left on both sides is split in the same way. So a thin band of
/// stray points between floor and ceiling does not join them into one surface.
fn split_at_valleys(profile: &[u32], first: usize, last: usize, spans: &mut Vec<(usize, usize)>) {
    let mut todo = vec![(first, last)];
    while let Some((low, high)) = todo.pop() {
        let mut peak = low;
        for step in low..=high {
            if profile[step] > profile[peak] {
                peak = step;
            }
        }
        let keep = ((profile[peak] as f64 * SURFACE_PEAK_SHARE).ceil() as u32).max(SURFACE_COLUMNS);
        let (mut from, mut to) = (peak, peak);
        while from > low && profile[from - 1] >= keep {
            from -= 1;
        }
        while to < high && profile[to + 1] >= keep {
            to += 1;
        }
        spans.push((from, to));
        if from > low {
            todo.push((low, from - 1));
        }
        if to < high {
            todo.push((to + 1, high));
        }
    }
}

/// The median height of the points in the surface: more exact than its height steps.
fn surface_height(xyz: &[[f64; 3]], surface: &Surface) -> f64 {
    let mut z: Vec<f64> = xyz
        .par_iter()
        .filter(|p| is_finite(p))
        .map(|p| p[2])
        .filter(|z| (surface.low..surface.high).contains(z))
        .collect();
    if z.is_empty() {
        return surface.z;
    }
    let middle = z.len() / 2;
    *z.select_nth_unstable_by(middle, f64::total_cmp).1
}

/// Groups the poses by height. A group is a stretch of height bins with many poses, so that
/// poses on a staircase between two storeys do not join them.
fn walk_groups(poses: &[[f64; 3]]) -> Vec<WalkGroup> {
    let mut z: Vec<f64> = poses
        .iter()
        .filter(|p| is_finite(p))
        .map(|p| p[2])
        .collect();
    if z.is_empty() {
        return Vec::new();
    }
    z.sort_unstable_by(f64::total_cmp);
    let bin_of = |v: f64| ((v - z[0]) / WALK_BIN) as u64;
    let enough = ((z.len() as f64 * WALK_BIN_SHARE).ceil() as usize).max(1);

    // (bin, first index, end index) of the full bins, ascending.
    let mut full: Vec<(u64, usize, usize)> = Vec::new();
    let mut start = 0;
    for bin in z.chunk_by(|a, b| bin_of(*a) == bin_of(*b)) {
        if bin.len() >= enough {
            full.push((bin_of(bin[0]), start, start + bin.len()));
        }
        start += bin.len();
    }

    let mut groups = Vec::new();
    for stretch in full.chunk_by(|a, b| b.0 == a.0 + 1) {
        let (begin, end) = (stretch[0].1, stretch[stretch.len() - 1].2);
        let count: usize = stretch.iter().map(|s| s.2 - s.1).sum();
        if count as f64 >= z.len() as f64 * WALK_GROUP_SHARE {
            groups.push(WalkGroup {
                z: z[(begin + end) / 2],
                count,
            });
        }
    }
    groups
}

/// Index of the lowest big surface among `candidates` (ascending by height).
fn lowest_big(surfaces: &[Surface], candidates: &[usize]) -> Option<usize> {
    let largest = candidates
        .iter()
        .map(|&i| surfaces[i].core)
        .fold(0.0, f64::max);
    candidates
        .iter()
        .copied()
        .find(|&i| surfaces[i].area >= MIN_AREA && surfaces[i].core >= BIG_SHARE * largest)
}

fn find_floor(surfaces: &[Surface], group: &WalkGroup, ceilings: &[usize]) -> Option<usize> {
    let window = (group.z - FLOOR_BELOW.1)..=(group.z - FLOOR_BELOW.0);
    let all: Vec<usize> = (0..surfaces.len())
        .filter(|&i| window.contains(&surfaces[i].z))
        .collect();
    let free: Vec<usize> = all
        .iter()
        .copied()
        .filter(|i| !ceilings.contains(i))
        .collect();
    // A storey's ceiling can only be the next floor if nothing else is there (the underside
    // of the slab was not measured).
    lowest_big(surfaces, &free).or_else(|| lowest_big(surfaces, &all))
}

fn find_ceiling(surfaces: &[Surface], group: &WalkGroup, floor: usize) -> Option<usize> {
    let floor = surfaces[floor];
    let spread = floor.high - floor.low + CEILING_EXTRA_SPREAD;
    let window = (group.z + CEILING_ABOVE_POSES)..=(floor.z + CEILING_MAX_ABOVE_FLOOR);
    let candidates: Vec<usize> = (0..surfaces.len())
        .filter(|&i| {
            window.contains(&surfaces[i].z) && surfaces[i].high - surfaces[i].low <= spread
        })
        .collect();
    lowest_big(surfaces, &candidates)
}

/// One storey per floor at least [`MIN_STOREY_DISTANCE`] apart, ascending. Of floors closer
/// than that, the larger one stays (the other one comes back as a hint on a step), and the
/// storey keeps the poses of both.
fn merge_close_floors(mut found: Vec<Found>, surfaces: &[Surface]) -> Vec<Found> {
    found.sort_by(|a, b| surfaces[a.floor].z.total_cmp(&surfaces[b.floor].z));
    let mut merged: Vec<Found> = Vec::new();
    for next in found {
        let Some(last) = merged.last_mut() else {
            merged.push(next);
            continue;
        };
        let (a, b) = (surfaces[last.floor], surfaces[next.floor]);
        if b.z - a.z >= MIN_STOREY_DISTANCE {
            merged.push(next);
            continue;
        }
        let poses = last.poses + next.poses;
        if b.area > a.area {
            *last = next;
        }
        last.poses = poses;
    }
    merged
}

/// Hints on surfaces a little above or below the floor. `label` is the number of the storey,
/// if there are several.
fn step_hints(
    surfaces: &[Surface],
    floor: usize,
    floors: &[usize],
    ceilings: &[usize],
    label: Option<usize>,
    hints: &mut Vec<String>,
) {
    let z = surfaces[floor].z;
    for (i, surface) in surfaces.iter().enumerate() {
        let height = (surface.z - z).abs();
        if floors.contains(&i)
            || ceilings.contains(&i)
            || surface.core < MIN_AREA
            || surface.high - surface.low > STEP_MAX_SPREAD
            || !(STEP_RANGE.0..=STEP_RANGE.1).contains(&height)
        {
            continue;
        }
        let prefix = label.map_or(String::new(), |nr| format!("Stockwerk {nr}: "));
        let side = if surface.z > z { "höher" } else { "tiefer" };
        hints.push(format!(
            "{prefix}Absatz im Boden: {} m² liegen {} m {side} als der Boden.",
            comma(surface.area, 1),
            comma(height, 2),
        ));
    }
}

/// A big surface far below the lowest floor: maybe a storey nobody walked on.
fn below_hint(surfaces: &[Surface], lowest_floor: usize, hints: &mut Vec<String>) {
    let floor = surfaces[lowest_floor];
    let below = surfaces
        .iter()
        .filter(|s| s.z < floor.z - BELOW_DEPTH && s.area >= BELOW_SHARE * floor.area)
        .max_by(|a, b| a.area.total_cmp(&b.area));
    if let Some(surface) = below {
        hints.push(format!(
            "Eine große Fläche ({} m²) liegt {} m tiefer als der Boden, dort gibt es keine \
             Laufbahn. Vielleicht ein weiteres Stockwerk?",
            comma(surface.area, 0),
            comma(floor.z - surface.z, 2)
        ));
    }
}

/// A number with a decimal comma.
pub(crate) fn comma(value: f64, digits: usize) -> String {
    format!("{value:.digits$}").replace('.', ",")
}

/// The rotation that makes the floor of `storey` level ([`Leveling::NONE`] below 0.3°).
pub fn level_for(input: &PlanInput, storey: &Storey) -> Leveling {
    let mut normal = Vector3::z();
    let mut pivot = Vector3::new(0.0, 0.0, storey.floor_z);
    for band in LEVEL_BANDS {
        match fit_floor(input.xyz, normal, pivot, band) {
            Some((n, p)) => {
                normal = n;
                pivot = p;
            }
            None => return Leveling::NONE,
        }
    }
    let tilt_deg = normal.xy().norm().atan2(normal.z).to_degrees();
    if tilt_deg < MIN_TILT_DEG {
        return Leveling::NONE;
    }
    Leveling {
        normal: normal.into(),
        pivot: pivot.into(),
        tilt_deg,
    }
}

/// Fits a plane through the columns near the current one: every column counts once, however
/// close the scanner was, and a column with points at very different heights (foot of a wall,
/// step) does not count.
fn fit_floor(
    xyz: &[[f64; 3]],
    normal: Vector3<f64>,
    pivot: Vector3<f64>,
    band: f64,
) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let mut near: Vec<(u64, [f64; 3])> = xyz
        .par_iter()
        .filter(|p| is_finite(p))
        .filter(|p| (Vector3::from(**p) - pivot).dot(&normal).abs() <= band)
        .map(|p| {
            let x = (p[0] / COLUMN).floor() as i64 as u32 as u64;
            let y = (p[1] / COLUMN).floor() as i64 as u32 as u64;
            ((x << 32) | y, *p)
        })
        .collect();
    // Stable, so that the sums below add up in the same order every time.
    near.par_sort_by_key(|entry| entry.0);

    let max_spread = (band * 1.5).min(LEVEL_MAX_SPREAD);
    let mut centres: Vec<Vector3<f64>> = Vec::new();
    for column in near.chunk_by(|a, b| a.0 == b.0) {
        let mut sum = Vector3::zeros();
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for (_, p) in column {
            let p = Vector3::from(*p);
            let d = (p - pivot).dot(&normal);
            low = low.min(d);
            high = high.max(d);
            sum += p;
        }
        if high - low <= max_spread {
            centres.push(sum / column.len() as f64);
        }
    }
    if centres.len() < LEVEL_MIN_COLUMNS {
        return None;
    }

    let centre = centres.iter().sum::<Vector3<f64>>() / centres.len() as f64;
    let mut covariance = Matrix3::zeros();
    for c in &centres {
        let d = c - centre;
        covariance += d * d.transpose();
    }
    let eigen = SymmetricEigen::new(covariance);
    let smallest = eigen.eigenvalues.imin();
    let mut n: Vector3<f64> = eigen.eigenvectors.column(smallest).into_owned();
    if n.z < 0.0 {
        n = -n;
    }
    Some((n.normalize(), centre))
}

/// The points and poses of one storey: points from 10 cm below the floor to 10 cm above the
/// ceiling (without one: up to 3.5 m above the floor), poses between floor and ceiling.
pub fn storey_points(input: &PlanInput, storey: &Storey, leveling: &Leveling) -> StoreyPoints {
    let rotation = leveling.rotation();
    let place = |p: [f64; 3]| match &rotation {
        Some(r) => turn(r, p, leveling.pivot),
        None => p,
    };
    // Levelling turns about a point on the floor plane, so the plane ends up at its height.
    let floor = if rotation.is_some() {
        leveling.pivot[2]
    } else {
        storey.floor_z
    };
    let height = storey.ceiling_z.map(|ceiling| ceiling - storey.floor_z);
    let top = floor + height.unwrap_or(NO_CEILING_TOP);
    let upper = if height.is_some() {
        top + STOREY_MARGIN
    } else {
        top
    };
    let band = (floor - STOREY_MARGIN)..=upper;

    let poses: Vec<[f64; 3]> = input
        .poses
        .iter()
        .map(|p| place(*p))
        .filter(|p| p[2] > floor && p[2] < top)
        .collect();
    let count = poses.len().max(1) as f64;
    let origin = [
        poses.iter().map(|p| p[0]).sum::<f64>() / count,
        poses.iter().map(|p| p[1]).sum::<f64>() / count,
        floor,
    ];
    let relative = |p: [f64; 3]| {
        [
            (p[0] - origin[0]) as f32,
            (p[1] - origin[1]) as f32,
            (p[2] - origin[2]) as f32,
        ]
    };

    let xyz: Vec<[f32; 3]> = input
        .xyz
        .par_iter()
        .filter_map(|p| {
            let q = place(*p);
            band.contains(&q[2]).then(|| relative(q))
        })
        .collect();
    // Two passes instead of one list of pairs: the colours would cost the memory twice.
    let rgb = input
        .rgb
        .filter(|rgb| rgb.len() == input.xyz.len())
        .map(|rgb| {
            input
                .xyz
                .par_iter()
                .zip(rgb.par_iter())
                .filter_map(|(p, c)| band.contains(&place(*p)[2]).then_some(*c))
                .collect()
        });

    StoreyPoints {
        xyz,
        rgb,
        poses: poses.iter().map(|p| relative(*p)).collect(),
        origin,
        leveling: *leveling,
        height: height.map(|h| h as f32),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{Scene, TestData};

    const FLOOR: f64 = -1.3;
    const CEILING: f64 = 1.3;
    const SIZE: [f64; 2] = [8.3, 7.9];
    const LOOP_Z0: [[f64; 3]; 4] = [
        [1.0, 1.0, 0.0],
        [7.0, 1.0, 0.0],
        [7.0, 7.0, 0.0],
        [1.0, 7.0, 0.0],
    ];

    fn scene() -> Scene {
        Scene::new()
            .room([0.0, 0.0], SIZE, FLOOR, 2.6)
            .walk(&LOOP_Z0)
    }

    fn search(data: &TestData) -> StoreySearch {
        find_storeys(&data.input(), &Progress::new()).unwrap()
    }

    fn two_storeys_data() -> TestData {
        let upper = LOOP_Z0.map(|p| [p[0], p[1], 2.9]);
        Scene::new()
            .room([0.0, 0.0], SIZE, FLOOR, 2.6)
            .room([0.0, 0.0], SIZE, 1.6, 2.6)
            .walk(&LOOP_Z0)
            .walk(&upper)
            .build()
    }

    #[test]
    fn one_room() {
        let found = search(&scene().build());
        assert_eq!(found.storeys.len(), 1);
        let storey = found.storeys[0];
        assert!((storey.floor_z - FLOOR).abs() < 0.01, "{}", storey.floor_z);
        assert!((storey.ceiling_z.unwrap() - CEILING).abs() < 0.01);
        assert!(storey.floor_area > 50.0, "{}", storey.floor_area);
        assert!((storey.pose_share - 1.0).abs() < 1e-9);
        assert!(found.hints.is_empty(), "{:?}", found.hints);
    }

    #[test]
    fn table_is_no_floor() {
        let found = search(&scene().slab([2.0, 2.0], [4.0, 3.0], FLOOR + 0.75).build());
        assert_eq!(found.storeys.len(), 1);
        assert!((found.storeys[0].floor_z - FLOOR).abs() < 0.01);
    }

    #[test]
    fn two_storeys() {
        let found = search(&two_storeys_data());
        assert_eq!(found.storeys.len(), 2, "{:?}", found);
        let (low, high) = (found.storeys[0], found.storeys[1]);
        assert!((low.floor_z - FLOOR).abs() < 0.01);
        assert!((high.floor_z - 1.6).abs() < 0.01, "{}", high.floor_z);
        assert!((low.ceiling_z.unwrap() - CEILING).abs() < 0.01);
        assert!((high.ceiling_z.unwrap() - 4.2).abs() < 0.01);
        assert!((low.pose_share - 0.5).abs() < 0.1);
        assert!((high.pose_share - 0.5).abs() < 0.1);
    }

    #[test]
    fn big_area_below_is_a_hint() {
        let found = search(&scene().slab([1.0, 1.0], [6.0, 5.0], FLOOR - 3.0).build());
        assert_eq!(found.storeys.len(), 1);
        assert!((found.storeys[0].floor_z - FLOOR).abs() < 0.01);
        assert!(
            found.hints.iter().any(|h| h.contains("tiefer")),
            "{:?}",
            found.hints
        );
    }

    #[test]
    fn step_in_floor_is_a_hint() {
        let found = search(&scene().slab([2.0, 2.0], [4.0, 4.0], FLOOR + 0.2).build());
        assert_eq!(found.storeys.len(), 1);
        assert!((found.storeys[0].floor_z - FLOOR).abs() < 0.01);
        assert!(
            found
                .hints
                .iter()
                .any(|h| h.contains("Absatz") && h.contains("höher")),
            "{:?}",
            found.hints
        );
    }

    #[test]
    fn tilt_is_taken_out() {
        let data = scene().tilt(1.0).build();
        let input = data.input();
        let storey = search(&data).storeys[0];
        let leveling = level_for(&input, &storey);
        assert!((leveling.tilt_deg - 1.0).abs() < 0.05, "{:?}", leveling);

        let points = storey_points(&input, &storey, &leveling);
        let mut floor = 0;
        for p in &points.xyz {
            let (x, y) = (
                p[0] as f64 + points.origin[0],
                p[1] as f64 + points.origin[1],
            );
            if (1.0..7.3).contains(&x) && (1.0..6.9).contains(&y) && p[2].abs() < 0.05 {
                assert!(p[2].abs() <= 0.005, "{}", p[2]);
                floor += 1;
            }
        }
        assert!(floor > 100_000, "{floor}");
    }

    #[test]
    fn small_tilt_is_kept() {
        let data = scene().tilt(0.1).build();
        let storey = search(&data).storeys[0];
        assert_eq!(level_for(&data.input(), &storey), Leveling::NONE);
    }

    #[test]
    fn leveling_turns_the_normal_up_and_back() {
        let n = [0.03, -0.02, (1.0f64 - 0.03 * 0.03 - 0.02 * 0.02).sqrt()];
        let pivot = [4.0, 3.0, -1.0];
        let leveling = Leveling {
            normal: n,
            pivot,
            tilt_deg: 2.0,
        };
        let top = leveling.apply([pivot[0] + n[0], pivot[1] + n[1], pivot[2] + n[2]]);
        assert!((top[0] - pivot[0]).abs() < 1e-12);
        assert!((top[1] - pivot[1]).abs() < 1e-12);
        assert!((top[2] - pivot[2] - 1.0).abs() < 1e-12);

        let p = [7.5, -2.0, 1.25];
        let back = leveling.undo(leveling.apply(p));
        for (a, b) in back.iter().zip(p) {
            assert!((a - b).abs() < 1e-12);
        }
        assert_eq!(Leveling::NONE.apply(p), p);
        assert_eq!(Leveling::NONE.undo(p), p);
    }

    /// Scatters `n` points evenly in the box from `min` to `max`, from a fixed generator.
    fn scatter(data: &mut TestData, n: usize, min: [f64; 3], max: [f64; 3]) {
        let mut seed: u64 = 0xACE1_0000_0000_0001;
        let mut next = || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (seed >> 40) as f64 / (1u64 << 24) as f64
        };
        for _ in 0..n {
            data.xyz.push([
                min[0] + (max[0] - min[0]) * next(),
                min[1] + (max[1] - min[1]) * next(),
                min[2] + (max[2] - min[2]) * next(),
            ]);
        }
    }

    /// Floor and ceiling of the plain room, whatever was scattered.
    fn assert_floor_and_ceiling(data: &TestData) {
        let found = search(data);
        assert_eq!(found.storeys.len(), 1, "{found:?}");
        let storey = found.storeys[0];
        assert!((storey.floor_z - FLOOR).abs() < 0.02, "{found:?}");
        let ceiling = storey.ceiling_z.expect("Decke");
        assert!((ceiling - CEILING).abs() < 0.02, "{found:?}");
    }

    #[test]
    fn scattered_points_at_room_height_do_not_join_floor_and_ceiling() {
        let base = scene().spacing(0.03).build();
        assert_floor_and_ceiling(&base);
        let high = FLOOR + 2.6;
        // About a room, far outside and inside the room itself.
        for (min, max) in [
            ([-80.0, -80.0, FLOOR], [88.0, 88.0, high]),
            ([20.0, 20.0, FLOOR], [60.0, 60.0, high]),
            ([0.0, 0.0, FLOOR], [8.3, 7.9, high]),
        ] {
            let mut data = base.clone();
            scatter(&mut data, 12_000, min, max);
            assert_floor_and_ceiling(&data);
        }
    }

    #[test]
    fn scattered_points_over_part_of_the_height_leave_the_ceiling() {
        let base = scene().spacing(0.03).build();
        let high = FLOOR + 2.6;
        for (from, to) in [(FLOOR + 1.8, high), (FLOOR, FLOOR + 1.0)] {
            for (min, max) in [
                ([20.0, 20.0, from], [80.0, 80.0, to]),
                ([0.0, 0.0, from], [8.3, 7.9, to]),
            ] {
                let mut data = base.clone();
                scatter(&mut data, 6_000, min, max);
                assert_floor_and_ceiling(&data);
            }
        }
    }

    #[test]
    fn a_low_ceiling_and_a_step_survive_scattered_points() {
        // A low ceiling over 40 % of the room and a step of 20 cm in the floor.
        let base = scene()
            .spacing(0.03)
            .slab([0.5, 0.5], [6.5, 5.5], FLOOR + 2.2)
            .slab([2.0, 2.0], [4.0, 4.0], FLOOR + 0.2)
            .build();
        let mut data = base.clone();
        scatter(
            &mut data,
            12_000,
            [0.0, 0.0, FLOOR],
            [8.3, 7.9, FLOOR + 2.6],
        );
        let found = search(&data);
        assert_eq!(found.storeys.len(), 1, "{found:?}");
        let storey = found.storeys[0];
        assert!((storey.floor_z - FLOOR).abs() < 0.02, "{found:?}");
        assert!(
            (storey.ceiling_z.expect("Decke") - (FLOOR + 2.2)).abs() < 0.02,
            "{found:?}"
        );
        assert!(
            found.hints.iter().any(|h| h.contains("Absatz")),
            "{:?}",
            found.hints
        );
    }

    /// Boxes of 30 × 30 × 15 cm at random places around a storey, their lower side at a random
    /// height of `low` above the floor.
    fn add_clutter(
        mut scene: Scene,
        count: usize,
        min: [f64; 2],
        max: [f64; 2],
        low: f64,
    ) -> Scene {
        let mut seed: u64 = 0xBEEF_0000_0000_0001;
        let mut next = || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (seed >> 40) as f64 / (1u64 << 24) as f64
        };
        for _ in 0..count {
            let (x, y) = (
                min[0] + (max[0] - min[0]) * next(),
                min[1] + (max[1] - min[1]) * next(),
            );
            let z = FLOOR + 0.1 + low * next();
            scene = scene.block([x, y, z], [x + 0.3, y + 0.3, z + 0.15]);
        }
        scene
    }

    #[test]
    fn clutter_low_above_the_floor_does_not_beat_a_small_floor() {
        // Over 60 cm of height, so the band of clutter lies in the search window of the floor
        // and has more columns than the floor of the small room.
        let way = [
            [1.0, 1.0, 0.0],
            [2.5, 1.0, 0.0],
            [2.5, 3.0, 0.0],
            [1.0, 3.0, 0.0],
        ];
        let small = Scene::new()
            .spacing(0.02)
            .room([0.0, 0.0], [3.5, 4.0], FLOOR, 2.6)
            .walk(&way);
        let data = add_clutter(small, 300, [-8.0, -8.0], [12.0, 12.0], 0.6).build();
        let found = search(&data);
        assert_eq!(found.storeys.len(), 1, "{found:?}");
        let storey = found.storeys[0];
        assert!((storey.floor_z - FLOOR).abs() < 0.02, "{found:?}");
        assert!(
            (storey.ceiling_z.expect("Decke") - CEILING).abs() < 0.02,
            "{found:?}"
        );
    }

    #[test]
    fn stray_points_alone_are_no_surface() {
        // The whole volume filled with strays only: no floor comes out of them.
        let mut data = Scene::new().walk(&LOOP_Z0).build();
        scatter(
            &mut data,
            30_000,
            [-5.0, -5.0, FLOOR],
            [13.0, 13.0, FLOOR + 2.6],
        );
        assert_eq!(
            find_storeys(&data.input(), &Progress::new()),
            Err(PlanError::NoFloor)
        );
    }

    #[test]
    fn no_ceiling() {
        let found = search(&scene().open_top().build());
        assert_eq!(found.storeys.len(), 1);
        assert_eq!(found.storeys[0].ceiling_z, None);
    }

    #[test]
    fn only_walls_is_no_floor() {
        let (x, y) = (SIZE[0], SIZE[1]);
        let data = Scene::new()
            .behind([0.0, 0.0, FLOOR], [x, 0.0, CEILING])
            .behind([0.0, y, FLOOR], [x, y, CEILING])
            .behind([0.0, 0.0, FLOOR], [0.0, y, CEILING])
            .behind([x, 0.0, FLOOR], [x, y, CEILING])
            .walk(&LOOP_Z0)
            .build();
        let result = find_storeys(&data.input(), &Progress::new());
        assert_eq!(result, Err(PlanError::NoFloor));
    }

    #[test]
    fn no_poses() {
        let data = Scene::new().room([0.0, 0.0], SIZE, FLOOR, 2.6).build();
        let result = find_storeys(&data.input(), &Progress::new());
        assert_eq!(result, Err(PlanError::NoPoses));
    }

    #[test]
    fn too_few_points() {
        let data = Scene::new()
            .slab([0.0, 0.0], [0.05, 0.05], FLOOR)
            .walk(&LOOP_Z0)
            .build();
        let result = find_storeys(&data.input(), &Progress::new());
        assert_eq!(result, Err(PlanError::TooFewPoints));
    }

    #[test]
    fn storey_points_only_this_storey() {
        let data = two_storeys_data();
        let input = data.input();
        let found = search(&data);

        let low = storey_points(&input, &found.storeys[0], &Leveling::NONE);
        assert!(!low.xyz.is_empty());
        assert!(low.xyz.iter().all(|p| (-0.1..=2.7).contains(&p[2])));
        assert_eq!(low.poses.len(), data.poses.len() / 2);
        assert!(low.poses.iter().all(|p| (p[2] as f64 + FLOOR).abs() < 0.01));
        assert!((low.height.unwrap() - 2.6).abs() < 0.01);
        let floor_points = low.xyz.iter().filter(|p| p[2].abs() < 0.01).count();
        assert!(floor_points > 600_000, "{floor_points}");

        let high = storey_points(&input, &found.storeys[1], &Leveling::NONE);
        assert_eq!(high.poses.len(), data.poses.len() / 2);
        assert!(high.xyz.iter().all(|p| (-0.1..=2.7).contains(&p[2])));
        assert!((high.origin[2] - 1.6).abs() < 0.01);
    }

    #[test]
    fn storey_without_ceiling_ends_at_three_and_a_half_metres() {
        let data = scene()
            .open_top()
            .slab([1.0, 1.0], [2.0, 2.0], FLOOR + 4.0)
            .build();
        let storey = Storey {
            floor_z: FLOOR,
            ceiling_z: None,
            pose_share: 1.0,
            floor_area: 60.0,
        };
        let points = storey_points(&data.input(), &storey, &Leveling::NONE);
        assert_eq!(points.height, None);
        assert!(!points.xyz.is_empty());
        assert!(points.xyz.iter().all(|p| p[2] <= 3.5));
    }

    #[test]
    fn colours_stay_with_their_points() {
        let data = two_storeys_data();
        let rgb: Vec<[u16; 3]> = data
            .xyz
            .iter()
            .map(|p| [if p[2] < 1.4 { 1 } else { 2 }; 3])
            .collect();
        let input = PlanInput {
            xyz: &data.xyz,
            rgb: Some(&rgb),
            poses: &data.poses,
        };
        let storey = search(&data).storeys[0];
        let points = storey_points(&input, &storey, &Leveling::NONE);
        let colours = points.rgb.unwrap();
        assert_eq!(colours.len(), points.xyz.len());
        assert!(colours.iter().all(|c| c[0] == 1));
    }

    #[test]
    fn same_input_same_result() {
        let data = scene().noise(0.005).tilt(1.0).build();
        let input = data.input();
        let a = search(&data);
        let b = search(&data);
        assert_eq!(a, b);
        let leveling = level_for(&input, &a.storeys[0]);
        assert_eq!(leveling, level_for(&input, &b.storeys[0]));
        assert_eq!(
            storey_points(&input, &a.storeys[0], &leveling),
            storey_points(&input, &b.storeys[0], &leveling)
        );
    }

    #[test]
    fn cancel_stops() {
        let data = scene().build();
        let before = Progress::new();
        before.cancel();
        let result = find_storeys(&data.input(), &before);
        assert_eq!(result, Err(PlanError::Cancelled));

        // Cancelled while it runs, at the first report.
        let canceller = Progress::new();
        let handle = canceller.clone();
        let during = canceller.with_observer(move |_| handle.cancel());
        let result = find_storeys(&data.input(), &during);
        assert_eq!(result, Err(PlanError::Cancelled));
    }
}
