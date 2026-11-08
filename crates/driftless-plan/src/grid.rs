//! The top-down grid of the wall band: per cell a mask of the occupied height slices.
//!
//! A cell does not count points but how much of the room height is occupied. A wall fills
//! almost all of it, a table or a low cupboard only the lower part. The masks of the 3×3
//! neighbourhood are combined, so that a sparse or slanted wall still fills its height.

use rayon::prelude::*;

/// Height slices of a cell, one bit each.
const SLICES: u32 = 64;
/// A cell with at least this many occupied slices (55 %) is strong ...
const STRONG_SLICES: u32 = 36;
/// ... and from this many (25 %) weak: it only counts when it hangs on a strong cell.
const WEAK_SLICES: u32 = 16;
/// Raster deciding the extent of the grid (metres).
const EXTENT_CELL: f64 = 0.5;
/// Fewer points than this in an extent cell are stray points.
const EXTENT_MIN_POINTS: usize = 20;
/// A group of extent cells cut off from the rest needs this share of the points.
const EXTENT_MIN_SHARE: f64 = 0.01;
/// Longest side of the grid; beyond it the cell grows.
const MAX_SIDE: usize = 8192;
/// Smallest cell (metres): guards against a nonsensical setting.
const MIN_CELL: f64 = 0.005;

/// The height range of the wall points (metres above the floor).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Band {
    pub low: f64,
    pub high: f64,
}

impl Band {
    fn contains(&self, z: f64) -> bool {
        z >= self.low && z < self.high
    }

    fn slice(&self, z: f64) -> u32 {
        (((z - self.low) / (self.high - self.low) * SLICES as f64) as u32).min(SLICES - 1)
    }
}

/// What a cell is after the threshold with connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CellClass {
    Empty,
    Weak,
    Strong,
}

/// See the module description. Cells are numbered row by row.
#[derive(Debug)]
pub(crate) struct HeightGrid {
    cell: f64,
    min: [f64; 2],
    nx: usize,
    ny: usize,
    masks: Vec<u64>,
    /// Points of cell `c`: `points[first[c]..first[c + 1]]`.
    first: Vec<u32>,
    points: Vec<u32>,
}

/// The area holding the points of the band. Only cells of a coarse raster with enough points
/// count, and a group of such cells, cut off from the rest, only with a real share of the
/// points. So a stray point or a patch of reflections 80 m away does not make the grid grow.
fn robust_extent(xyz: &[[f32; 3]], band: Band) -> Option<([f64; 2], [f64; 2])> {
    const OFFSET: i64 = 1 << 31;
    let mut keys: Vec<u64> = xyz
        .par_iter()
        .filter(|p| p.iter().all(|c| c.is_finite()) && band.contains(p[2] as f64))
        .map(|p| {
            let ix = (p[0] as f64 / EXTENT_CELL).floor() as i32;
            let iy = (p[1] as f64 / EXTENT_CELL).floor() as i32;
            (((ix as i64 + OFFSET) as u64) << 32) | (iy as i64 + OFFSET) as u64
        })
        .collect();
    keys.par_sort_unstable();

    // Sorted by (x, y), so that neighbours can be found by a binary search.
    let cells: Vec<([i64; 2], usize)> = keys
        .chunk_by(|a, b| a == b)
        .filter(|run| run.len() >= EXTENT_MIN_POINTS)
        .map(|run| {
            let (ix, iy) = (run[0] >> 32, run[0] & 0xFFFF_FFFF);
            ([ix as i64 - OFFSET, iy as i64 - OFFSET], run.len())
        })
        .collect();
    let total: usize = cells.iter().map(|c| c.1).sum();

    let mut min = [i64::MAX; 2];
    let mut max = [i64::MIN; 2];
    let mut visited = vec![false; cells.len()];
    let mut group = Vec::new();
    for start in 0..cells.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        group.clear();
        group.push(start);
        let mut next = 0;
        while next < group.len() {
            let [x, y] = cells[group[next]].0;
            next += 1;
            for key in (x - 1..=x + 1).flat_map(|nx| (y - 1..=y + 1).map(move |ny| [nx, ny])) {
                if let Ok(found) = cells.binary_search_by_key(&key, |c| c.0) {
                    if !visited[found] {
                        visited[found] = true;
                        group.push(found);
                    }
                }
            }
        }
        let points: usize = group.iter().map(|&c| cells[c].1).sum();
        if points as f64 >= total as f64 * EXTENT_MIN_SHARE {
            for &c in &group {
                let [x, y] = cells[c].0;
                min = [min[0].min(x), min[1].min(y)];
                max = [max[0].max(x), max[1].max(y)];
            }
        }
    }
    (min[0] <= max[0]).then(|| {
        (
            [min[0] as f64 * EXTENT_CELL, min[1] as f64 * EXTENT_CELL],
            [
                (max[0] + 1) as f64 * EXTENT_CELL,
                (max[1] + 1) as f64 * EXTENT_CELL,
            ],
        )
    })
}

impl HeightGrid {
    /// The grid of the points in `band`; `None` if there are not enough of them.
    pub(crate) fn build(xyz: &[[f32; 3]], band: Band, cell: f64) -> Option<HeightGrid> {
        let (min, max) = robust_extent(xyz, band)?;
        let size = [max[0] - min[0], max[1] - min[1]];
        let cell = cell
            .max(MIN_CELL)
            .max(size[0].max(size[1]) / MAX_SIDE as f64);
        let nx = ((size[0] / cell).ceil() as usize).clamp(1, MAX_SIDE);
        let ny = ((size[1] / cell).ceil() as usize).clamp(1, MAX_SIDE);

        let place = |p: &[f32; 3]| -> Option<(usize, u32)> {
            let z = p[2] as f64;
            if !band.contains(z) {
                return None;
            }
            let (x, y) = ((p[0] as f64 - min[0]) / cell, (p[1] as f64 - min[1]) / cell);
            if !(x >= 0.0 && y >= 0.0) {
                return None;
            }
            let (i, j) = (x as usize, y as usize);
            (i < nx && j < ny).then(|| (j * nx + i, band.slice(z)))
        };

        let mut masks = vec![0u64; nx * ny];
        let mut first = vec![0u32; nx * ny + 1];
        for p in xyz {
            if let Some((c, slice)) = place(p) {
                masks[c] |= 1 << slice;
                first[c + 1] += 1;
            }
        }
        for c in 0..nx * ny {
            first[c + 1] += first[c];
        }
        let mut next = first.clone();
        let mut points = vec![0u32; first[nx * ny] as usize];
        for (i, p) in xyz.iter().enumerate() {
            if let Some((c, _)) = place(p) {
                points[next[c] as usize] = i as u32;
                next[c] += 1;
            }
        }
        Some(HeightGrid {
            cell,
            min,
            nx,
            ny,
            masks,
            first,
            points,
        })
    }

    pub(crate) fn cell(&self) -> f64 {
        self.cell
    }

    /// Number of the cell holding a position, if it is in the grid.
    pub(crate) fn index_at(&self, p: [f64; 2]) -> Option<usize> {
        let (x, y) = (
            (p[0] - self.min[0]) / self.cell,
            (p[1] - self.min[1]) / self.cell,
        );
        if !(x >= 0.0 && y >= 0.0) {
            return None;
        }
        let (i, j) = (x as usize, y as usize);
        (i < self.nx && j < self.ny).then_some(j * self.nx + i)
    }

    pub(crate) fn centre(&self, index: usize) -> [f64; 2] {
        let (i, j) = (index % self.nx, index / self.nx);
        [
            self.min[0] + (i as f64 + 0.5) * self.cell,
            self.min[1] + (j as f64 + 0.5) * self.cell,
        ]
    }

    /// Indices of the points in a cell.
    pub(crate) fn points_of(&self, index: usize) -> &[u32] {
        &self.points[self.first[index] as usize..self.first[index + 1] as usize]
    }

    /// Slices occupied in the 3×3 neighbourhood of a cell.
    fn neighbourhood(&self, i: usize, j: usize) -> u32 {
        let mut mask = 0u64;
        for y in j.saturating_sub(1)..=(j + 1).min(self.ny - 1) {
            for x in i.saturating_sub(1)..=(i + 1).min(self.nx - 1) {
                mask |= self.masks[y * self.nx + x];
            }
        }
        mask.count_ones()
    }

    /// Strong and weak cells. A cell needs points of its own; a weak cell only stays when it
    /// is connected to a strong one (8-neighbourhood).
    pub(crate) fn classify(&self) -> Vec<CellClass> {
        let mut class: Vec<CellClass> = (0..self.nx * self.ny)
            .into_par_iter()
            .map(|c| {
                if self.masks[c] == 0 {
                    return CellClass::Empty;
                }
                match self.neighbourhood(c % self.nx, c / self.nx) {
                    n if n >= STRONG_SLICES => CellClass::Strong,
                    n if n >= WEAK_SLICES => CellClass::Weak,
                    _ => CellClass::Empty,
                }
            })
            .collect();

        let mut reached = vec![false; class.len()];
        let mut stack: Vec<usize> = (0..class.len())
            .filter(|&c| class[c] == CellClass::Strong)
            .collect();
        while let Some(c) = stack.pop() {
            let (i, j) = (c % self.nx, c / self.nx);
            for y in j.saturating_sub(1)..=(j + 1).min(self.ny - 1) {
                for x in i.saturating_sub(1)..=(i + 1).min(self.nx - 1) {
                    let other = y * self.nx + x;
                    if class[other] == CellClass::Weak && !reached[other] {
                        reached[other] = true;
                        stack.push(other);
                    }
                }
            }
        }
        for (c, reached) in class.iter_mut().zip(&reached) {
            if *c == CellClass::Weak && !reached {
                *c = CellClass::Empty;
            }
        }
        class
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BAND: Band = Band {
        low: 0.15,
        high: 2.45,
    };

    /// A vertical strip of points at `(x, y)` from `z0` to `z1`, a point every centimetre.
    fn strip(out: &mut Vec<[f32; 3]>, x: f32, y: f32, z0: f32, z1: f32) {
        let n = ((z1 - z0) / 0.01) as usize;
        out.extend((0..n).map(|k| [x, y, z0 + k as f32 * 0.01]));
    }

    /// A full-height wall along x at y = 1, from x = 0 to `len`.
    fn wall(out: &mut Vec<[f32; 3]>, len: f32) {
        let n = (len / 0.02) as usize;
        for i in 0..n {
            strip(out, 0.01 + i as f32 * 0.02, 1.01, 0.0, 2.6);
        }
    }

    #[test]
    fn a_mask_holds_the_occupied_slices() {
        let mut xyz = Vec::new();
        strip(&mut xyz, 0.03, 0.03, 0.15, 1.3);
        strip(&mut xyz, 0.45, 0.45, 0.15, 2.45);
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        let low = grid.masks[grid.index_at([0.03, 0.03]).unwrap()];
        assert_eq!(low.count_ones(), 32);
        assert_eq!(low.trailing_ones(), 32);
        assert_eq!(grid.masks[grid.index_at([0.45, 0.45]).unwrap()], u64::MAX);
        assert_eq!(
            grid.points_of(grid.index_at([0.45, 0.45]).unwrap()).len(),
            230
        );
    }

    #[test]
    fn points_outside_the_band_do_not_count() {
        let mut xyz = Vec::new();
        strip(&mut xyz, 0.03, 0.03, 0.0, 0.15);
        strip(&mut xyz, 0.03, 0.03, 2.45, 2.6);
        strip(&mut xyz, 0.45, 0.45, 0.2, 0.5);
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        assert_eq!(grid.masks[grid.index_at([0.03, 0.03]).unwrap()], 0);
    }

    #[test]
    fn wall_is_strong_and_low_furniture_is_not() {
        let mut xyz = Vec::new();
        wall(&mut xyz, 2.0);
        // A box 1 m high, away from the wall.
        for i in 0..40 {
            strip(&mut xyz, 0.5 + i as f32 * 0.02, 0.2, 0.0, 1.0);
        }
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        let class = grid.classify();
        assert_eq!(
            class[grid.index_at([1.01, 1.01]).unwrap()],
            CellClass::Strong
        );
        assert_eq!(class[grid.index_at([0.9, 0.2]).unwrap()], CellClass::Empty);
    }

    #[test]
    fn weak_cell_stays_when_it_hangs_on_a_strong_one() {
        let mut xyz = Vec::new();
        wall(&mut xyz, 2.0);
        // Half height, in line with the wall and touching it (a window in the wall).
        for i in 0..40 {
            strip(&mut xyz, 2.01 + i as f32 * 0.02, 1.01, 0.0, 1.2);
        }
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        let class = grid.classify();
        assert_eq!(class[grid.index_at([2.6, 1.01]).unwrap()], CellClass::Weak);
    }

    #[test]
    fn cell_without_own_points_is_empty_even_beside_a_wall() {
        let mut xyz = Vec::new();
        wall(&mut xyz, 2.0);
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        assert_eq!(
            grid.classify()[grid.index_at([1.01, 1.03]).unwrap()],
            CellClass::Empty
        );
    }

    #[test]
    fn stray_points_do_not_grow_the_grid() {
        let mut xyz = Vec::new();
        wall(&mut xyz, 10.0);
        // A few reflections, sparse or as a small dense patch.
        xyz.extend((0..200).map(|i| [80.0 + i as f32, 80.0, 1.0]));
        xyz.extend((0..300).map(|i| [-80.0 + i as f32 * 0.02, 60.0, 1.0]));
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        assert!(grid.nx <= 550, "{}", grid.nx);
        assert!(grid.ny <= 100, "{}", grid.ny);
        assert_eq!(grid.cell(), 0.02);
    }

    #[test]
    fn huge_extent_coarsens_the_cell() {
        let mut xyz = Vec::new();
        wall(&mut xyz, 2.0);
        let far = xyz
            .iter()
            .map(|p| [p[0] + 300.0, p[1], p[2]])
            .collect::<Vec<_>>();
        xyz.extend(far);
        let grid = HeightGrid::build(&xyz, BAND, 0.02).unwrap();
        assert!(grid.cell() > 0.02);
        assert!(grid.nx <= MAX_SIDE && grid.ny <= MAX_SIDE);
    }

    #[test]
    fn no_points_no_grid() {
        assert!(HeightGrid::build(&[], BAND, 0.02).is_none());
        assert!(HeightGrid::build(&[[0.0, 0.0, 1.0]], BAND, 0.02).is_none());
    }
}
