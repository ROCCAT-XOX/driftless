//! Line of sight: a bit raster of the occupied space and a walk along a line through it.
//!
//! Every point marks the cell it falls in. A line is free when it passes no marked cell, in the
//! order of the cells it runs through (no steps that could jump over a cell).

/// Edge of a cell (metres).
const CELL: f64 = 0.05;
/// Most bits of a raster (1 GiB); beyond it the cell grows.
const MAX_BITS: usize = 1 << 33;

/// See the module description. The space outside the raster counts as free.
#[derive(Debug)]
pub(crate) struct SightGrid {
    cell: f64,
    origin: [f64; 3],
    dims: [usize; 3],
    bits: Vec<u64>,
}

impl SightGrid {
    /// A raster of the box from `low` to `high`; points outside it are left out.
    pub(crate) fn build(xyz: &[[f32; 3]], low: [f64; 3], high: [f64; 3]) -> SightGrid {
        let mut cell = CELL;
        let dims = loop {
            let dims = [0, 1, 2].map(|i| (((high[i] - low[i]) / cell).ceil() as usize).max(1));
            let bits = dims[0].saturating_mul(dims[1]).saturating_mul(dims[2]);
            if bits <= MAX_BITS {
                break dims;
            }
            cell *= 2.0;
        };
        let words = (dims[0] * dims[1] * dims[2]).div_ceil(64);
        let mut grid = SightGrid {
            cell,
            origin: low,
            dims,
            bits: vec![0; words],
        };
        for p in xyz {
            let at = [p[0] as f64, p[1] as f64, p[2] as f64];
            if let Some(index) = grid.index(grid.voxel_of(at)) {
                grid.bits[index / 64] |= 1 << (index % 64);
            }
        }
        grid
    }

    /// Edge of a cell really used (metres).
    #[cfg(test)]
    pub(crate) fn cell(&self) -> f64 {
        self.cell
    }

    fn voxel_of(&self, p: [f64; 3]) -> [f64; 3] {
        [0, 1, 2].map(|i| ((p[i] - self.origin[i]) / self.cell).floor())
    }

    fn index(&self, voxel: [f64; 3]) -> Option<usize> {
        let inside = (0..3).all(|i| voxel[i] >= 0.0 && voxel[i] < self.dims[i] as f64);
        inside.then(|| {
            voxel[0] as usize
                + self.dims[0] * (voxel[1] as usize + self.dims[1] * voxel[2] as usize)
        })
    }

    /// Whether the cell at `p` has a point.
    pub(crate) fn occupied(&self, p: [f64; 3]) -> bool {
        self.occupied_voxel(self.voxel_of(p))
    }

    fn occupied_voxel(&self, voxel: [f64; 3]) -> bool {
        self.index(voxel)
            .is_some_and(|index| self.bits[index / 64] >> (index % 64) & 1 == 1)
    }

    /// Whether the line from `from` to `to` is free, leaving out its first `skip` metres.
    pub(crate) fn free(&self, from: [f64; 3], to: [f64; 3], skip: f64) -> bool {
        let d = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
        let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if length <= skip {
            return true;
        }
        let first = skip / length;
        let start = [0, 1, 2].map(|i| (from[i] + d[i] * first - self.origin[i]) / self.cell);
        let end = [0, 1, 2].map(|i| (to[i] - self.origin[i]) / self.cell);
        let way = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];

        let mut voxel = start.map(f64::floor);
        let mut step = [0.0; 3];
        let mut next = [f64::INFINITY; 3];
        let mut delta = [f64::INFINITY; 3];
        for i in 0..3 {
            if way[i] > 0.0 {
                step[i] = 1.0;
                next[i] = (voxel[i] + 1.0 - start[i]) / way[i];
                delta[i] = 1.0 / way[i];
            } else if way[i] < 0.0 {
                step[i] = -1.0;
                next[i] = (voxel[i] - start[i]) / way[i];
                delta[i] = -1.0 / way[i];
            }
        }
        loop {
            if self.occupied_voxel(voxel) {
                return false;
            }
            let axis = (1..3).fold(0, |best, i| if next[i] < next[best] { i } else { best });
            if next[axis] > 1.0 {
                return true;
            }
            voxel[axis] += step[axis];
            next[axis] += delta[axis];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOW: [f64; 3] = [-1.0, -1.0, -1.0];
    const HIGH: [f64; 3] = [5.0, 5.0, 3.0];

    #[test]
    fn a_point_blocks_the_line_through_its_cell() {
        let grid = SightGrid::build(&[[2.0, 1.0, 1.0]], LOW, HIGH);
        assert!(!grid.free([0.0, 1.0, 1.0], [4.0, 1.0, 1.0], 0.0));
        assert!(grid.free([0.0, 2.0, 1.0], [4.0, 2.0, 1.0], 0.0));
        assert!(grid.occupied([2.0, 1.0, 1.0]));
        assert!(!grid.occupied([2.0, 2.0, 1.0]));
    }

    #[test]
    fn a_slanted_line_meets_the_cell_in_every_direction() {
        let grid = SightGrid::build(&[[2.0, 2.0, 1.0]], LOW, HIGH);
        let centre = [2.025, 2.025, 1.025];
        for (dx, dy, dz) in [
            (1.0, 1.0, 0.5),
            (-1.0, 1.0, -0.5),
            (1.0, -0.3, 0.2),
            (-0.2, -1.0, 0.7),
        ] {
            let from = [centre[0] - dx, centre[1] - dy, centre[2] - dz];
            let to = [centre[0] + dx, centre[1] + dy, centre[2] + dz];
            assert!(!grid.free(from, to, 0.0), "{dx} {dy} {dz}");
        }
    }

    #[test]
    fn the_line_ends_where_it_should() {
        let grid = SightGrid::build(&[[2.0, 1.0, 1.0]], LOW, HIGH);
        assert!(grid.free([0.0, 1.0, 1.0], [1.9, 1.0, 1.0], 0.0));
        assert!(!grid.free([0.0, 1.0, 1.0], [2.02, 1.0, 1.0], 0.0));
    }

    #[test]
    fn the_start_of_the_line_is_left_out() {
        let grid = SightGrid::build(&[[0.1, 1.0, 1.0]], LOW, HIGH);
        assert!(!grid.free([0.0, 1.0, 1.0], [2.0, 1.0, 1.0], 0.0));
        assert!(grid.free([0.0, 1.0, 1.0], [2.0, 1.0, 1.0], 0.20));
        assert!(grid.free([0.0, 1.0, 1.0], [0.1, 1.0, 1.0], 0.20));
    }

    #[test]
    fn outside_the_raster_is_free() {
        let grid = SightGrid::build(&[[9.0, 1.0, 1.0]], LOW, HIGH);
        assert!(!grid.occupied([9.0, 1.0, 1.0]));
        assert!(grid.free([-3.0, 1.0, 1.0], [12.0, 1.0, 1.0], 0.0));
    }

    #[test]
    fn a_huge_box_makes_the_cell_grow() {
        let grid = SightGrid::build(&[], [0.0; 3], [2000.0, 2000.0, 5.0]);
        assert!(grid.cell() > CELL);
        assert!(grid.dims[0] * grid.dims[1] * grid.dims[2] <= MAX_BITS);
    }
}
