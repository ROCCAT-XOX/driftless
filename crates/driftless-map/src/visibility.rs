//! Which cloud points an image sees: candidates near the camera from a coarse grid, then a
//! depth buffer.
//!
//! Per image the work is a pure per-point map (project a point), an order-independent
//! reduction (minimum depth per buffer cell, an integer `fetch_min` on the bits of a positive
//! `f32`) and a second pure per-point map (compare with the buffer), so the result is the
//! same in every run and the steps can later move to a GPU compute shader unchanged.

use driftless_io::camera::Camera;
use nalgebra::{Isometry3, Point3, Vector3};
use rayon::prelude::*;
use std::sync::atomic::{AtomicU32, Ordering};

/// Edge of the coarse grid cells used to find the points near a camera (m).
pub const GRID_CELL: f64 = 1.0;
/// Relative part of the depth tolerance: a point is visible if its range is at most the
/// buffer depth plus [`crate::ColorConfig::depth_tolerance`] plus this share of the depth.
pub const DEPTH_TOLERANCE_REL: f64 = 0.02;
/// A point whose neighbouring depth cell holds something nearer than its depth divided by
/// `1 + SILHOUETTE_REL` (minus the absolute tolerance) lies on an occluder's silhouette and
/// counts as not visible. Surfaces seen at a slant change depth far less from one cell to
/// the next, except at grazing angles where their views carry little weight anyway.
pub const SILHOUETTE_REL: f64 = 0.2;
/// Largest splat half-size in depth-buffer cells (a splat covers up to (2·4+1)² cells).
pub const MAX_SPLAT: i32 = 4;

/// The cloud's points sorted into cubic cells of [`GRID_CELL`], cells in key order.
pub(crate) struct PointGrid {
    /// Cell key, and the range of `order` holding its points.
    cells: Vec<([i32; 3], u32, u32)>,
    /// Point indices, grouped by cell, ascending within a cell.
    order: Vec<u32>,
}

fn cell_key(p: &[f32; 3]) -> [i32; 3] {
    p.map(|v| (f64::from(v) / GRID_CELL).floor() as i32)
}

impl PointGrid {
    pub(crate) fn new(xyz: &[[f32; 3]]) -> PointGrid {
        let mut keyed: Vec<([i32; 3], u32)> = xyz
            .iter()
            .enumerate()
            .map(|(i, p)| {
                (
                    cell_key(p),
                    u32::try_from(i).expect("fewer than 2^32 points"),
                )
            })
            .collect();
        keyed.par_sort_unstable();
        let mut cells: Vec<([i32; 3], u32, u32)> = Vec::new();
        for (n, (key, _)) in keyed.iter().enumerate() {
            let n = n as u32;
            match cells.last_mut() {
                Some(c) if c.0 == *key => c.2 = n + 1,
                _ => cells.push((*key, n, n + 1)),
            }
        }
        PointGrid {
            cells,
            order: keyed.into_iter().map(|(_, i)| i).collect(),
        }
    }

    /// Indices of the points in every cell that comes within `radius` of `centre` (a
    /// superset of the points within `radius`), in a fixed order.
    pub(crate) fn near(&self, centre: &Vector3<f64>, radius: f64) -> Vec<u32> {
        let r2 = radius * radius;
        let mut out = Vec::new();
        for &(key, start, end) in &self.cells {
            // Squared distance from the centre to the cell's box.
            let d2: f64 = (0..3)
                .map(|k| {
                    let lo = f64::from(key[k]) * GRID_CELL;
                    let d = (lo - centre[k]).max(centre[k] - lo - GRID_CELL).max(0.0);
                    d * d
                })
                .sum();
            if d2 <= r2 {
                out.extend_from_slice(&self.order[start as usize..end as usize]);
            }
        }
        out
    }
}

/// A cloud point projected into an image.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projection {
    /// Pixel coordinates in the image (pixel centres at integers).
    pub uv: [f32; 2],
    /// Distance from the camera centre (m).
    pub range: f32,
    /// Angle from the optical axis (rad).
    pub theta: f32,
}

/// Projects world point `p` into `cam` (pose inverse `world_to_cam`); `None` beyond
/// `max_dist`, outside the lens model or outside the image.
pub(crate) fn project_point(
    p: &[f32; 3],
    cam: &Camera,
    world_to_cam: &Isometry3<f64>,
    max_dist: f64,
) -> Option<Projection> {
    let pc = world_to_cam * Point3::from(Vector3::from(p.map(f64::from)));
    let range = pc.coords.norm();
    if range > max_dist || range == 0.0 {
        return None;
    }
    let uv = cam.project(&pc.coords)?;
    Some(Projection {
        uv: [uv[0] as f32, uv[1] as f32],
        range: range as f32,
        theta: pc.x.hypot(pc.y).atan2(pc.z) as f32,
    })
}

/// Minimum depth per cell of an image's depth buffer (`scale` × `scale` image pixels a
/// cell). Depths are positive `f32`, whose bit patterns order like the values, so the
/// minimum is an integer `fetch_min` and does not depend on the order of the writes.
///
/// It may hold only a window of the cells ([`DepthBuffer::window`]): writes outside it are
/// left out, and only cells inside it may be read; there it holds what the whole buffer holds.
pub(crate) struct DepthBuffer {
    width: usize,
    height: usize,
    scale: f32,
    /// Focal length in cells per radian (the larger of fx, fy), for the splat size.
    focal: f32,
    /// The cells held: columns `x0 .. x0 + cols`, rows `y0 .. y0 + rows`.
    x0: i32,
    y0: i32,
    cols: usize,
    rows: usize,
    depth: Vec<AtomicU32>,
}

impl DepthBuffer {
    pub(crate) fn new(cam: &Camera, scale: u32) -> DepthBuffer {
        let width = cam.width.div_ceil(scale) as usize;
        let height = cam.height.div_ceil(scale) as usize;
        DepthBuffer::window(cam, scale, [0, 0], [width as i32 - 1, height as i32 - 1])
    }

    /// The buffer of [`DepthBuffer::new`], holding only the cells from `lo` to `hi` (column,
    /// row; both included, cut to the image).
    pub(crate) fn window(cam: &Camera, scale: u32, lo: [i32; 2], hi: [i32; 2]) -> DepthBuffer {
        let width = cam.width.div_ceil(scale) as usize;
        let height = cam.height.div_ceil(scale) as usize;
        let x0 = lo[0].clamp(0, width as i32 - 1);
        let y0 = lo[1].clamp(0, height as i32 - 1);
        let x1 = hi[0].clamp(x0, width as i32 - 1);
        let y1 = hi[1].clamp(y0, height as i32 - 1);
        let (cols, rows) = ((x1 - x0 + 1) as usize, (y1 - y0 + 1) as usize);
        DepthBuffer {
            width,
            height,
            scale: scale as f32,
            focal: (cam.fx.max(cam.fy) / f64::from(scale)) as f32,
            x0,
            y0,
            cols,
            rows,
            depth: (0..cols * rows)
                .map(|_| AtomicU32::new(f32::INFINITY.to_bits()))
                .collect(),
        }
    }

    /// The cells it holds.
    pub(crate) fn cells(&self) -> usize {
        self.cols * self.rows
    }

    /// Cell holding pixel `uv`.
    fn cell(&self, uv: [f32; 2]) -> (i32, i32) {
        let c = |v: f32, n: usize| (((v + 0.5) / self.scale) as i32).clamp(0, n as i32 - 1);
        (c(uv[0], self.width), c(uv[1], self.height))
    }

    /// The cell of [`Self::cell`] in a buffer of `cam` with cells of `scale` pixels, before one
    /// is made.
    pub(crate) fn cell_of(cam: &Camera, scale: u32, uv: [f32; 2]) -> (i32, i32) {
        let n = |len: u32| len.div_ceil(scale) as i32;
        let c = |v: f32, n: i32| (((v + 0.5) / scale as f32) as i32).clamp(0, n - 1);
        (c(uv[0], n(cam.width)), c(uv[1], n(cam.height)))
    }

    /// Where cell (`x`, `y`) of the window is kept.
    fn at(&self, x: i32, y: i32) -> &AtomicU32 {
        debug_assert!(
            x >= self.x0
                && y >= self.y0
                && ((x - self.x0) as usize) < self.cols
                && ((y - self.y0) as usize) < self.rows,
            "cell ({x}, {y}) outside the window"
        );
        &self.depth[(y - self.y0) as usize * self.cols + (x - self.x0) as usize]
    }

    /// Splat half-size (cells) of a point with neighbour spacing `spacing` (m): its footprint
    /// in the image, so that a surface seen from close leaves no holes in the buffer through
    /// which points behind it would count as visible.
    fn half_size(&self, p: &Projection, spacing: f32) -> i32 {
        let cells = spacing * self.focal / p.range;
        ((0.5 * cells - 0.5).ceil() as i32).clamp(0, MAX_SPLAT)
    }

    /// Writes the point's depth into the cells of its footprint (keeping the minimum), as far
    /// as they are in the window.
    pub(crate) fn splat(&self, p: &Projection, spacing: f32) {
        let (cx, cy) = self.cell(p.uv);
        let r = self.half_size(p, spacing);
        let bits = p.range.to_bits();
        let (x1, y1) = (
            self.x0 + self.cols as i32 - 1,
            self.y0 + self.rows as i32 - 1,
        );
        for y in (cy - r).max(self.y0)..=(cy + r).min(y1) {
            for x in (cx - r).max(self.x0)..=(cx + r).min(x1) {
                self.at(x, y).fetch_min(bits, Ordering::Relaxed);
            }
        }
    }

    /// The point is at most `tol_abs` + [`DEPTH_TOLERANCE_REL`] of the depth behind the
    /// nearest depth of its cell, and no neighbouring cell holds something more than
    /// `tol_abs` + [`SILHOUETTE_REL`] of the depth nearer (it would lie on the silhouette of
    /// an occluder, whose colour the image blurs into it).
    pub(crate) fn is_visible(&self, p: &Projection, tol_abs: f64) -> bool {
        let (cx, cy) = self.cell(p.uv);
        let range = f64::from(p.range);
        let depth =
            |x: i32, y: i32| f64::from(f32::from_bits(self.at(x, y).load(Ordering::Relaxed)));
        if range > depth(cx, cy) * (1.0 + DEPTH_TOLERANCE_REL) + tol_abs {
            return false;
        }
        let ys = (cy - 1).max(0)..=(cy + 1).min(self.height as i32 - 1);
        ys.flat_map(|y| {
            ((cx - 1).max(0)..=(cx + 1).min(self.width as i32 - 1)).map(move |x| (x, y))
        })
        .all(|(x, y)| range <= depth(x, y) * (1.0 + SILHOUETTE_REL) + tol_abs)
    }
}

/// A point an image sees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    pub index: u32,
    pub proj: Projection,
}

/// The points among `candidates` that `cam` at pose inverse `world_to_cam` sees: within
/// `max_dist`, inside the image and at most the depth tolerance behind the depth buffer
/// (cells of `zbuffer_scale` pixels, each point splatted with its neighbour spacing). In
/// candidate order. Public as the reference of the GPU path (`driftless-gpu`).
#[allow(clippy::too_many_arguments)]
pub fn visible_points(
    xyz: &[[f32; 3]],
    spacing: &[f32],
    candidates: &[u32],
    cam: &Camera,
    world_to_cam: &Isometry3<f64>,
    max_dist: f64,
    zbuffer_scale: u32,
    tol_abs: f64,
) -> Vec<Seen> {
    let projected: Vec<Option<Projection>> = candidates
        .par_iter()
        .map(|&i| project_point(&xyz[i as usize], cam, world_to_cam, max_dist))
        .collect();
    let buffer = DepthBuffer::new(cam, zbuffer_scale);
    projected
        .par_iter()
        .zip(candidates.par_iter())
        .for_each(|(p, &i)| {
            if let Some(p) = p {
                buffer.splat(p, spacing[i as usize]);
            }
        });
    projected
        .par_iter()
        .zip(candidates.par_iter())
        .filter_map(|(p, &index)| {
            let proj = (*p)?;
            buffer
                .is_visible(&proj, tol_abs)
                .then_some(Seen { index, proj })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_io::calib::CameraModel;

    fn pinhole() -> Camera {
        Camera {
            name: "test".into(),
            width: 100,
            height: 80,
            fx: 50.0,
            fy: 50.0,
            cx: 49.5,
            cy: 39.5,
            model: CameraModel::Pinhole,
            max_theta: std::f64::consts::FRAC_PI_2,
        }
    }

    #[test]
    fn grid_finds_points_of_cells_in_reach() {
        let xyz = [
            [0.5, 0.5, 0.5],
            [5.5, 0.5, 0.5],
            [1.5, 0.2, 0.1],
            [-0.5, 0.5, 0.5],
        ];
        let grid = PointGrid::new(&xyz);
        let near = grid.near(&Vector3::new(0.5, 0.5, 0.5), 1.0);
        assert_eq!(near, vec![3, 0, 2]); // cells in key order: x = −1, 0, 1
        assert_eq!(grid.near(&Vector3::new(5.0, 0.0, 0.0), 0.1), vec![1]);
        assert!(grid.near(&Vector3::new(20.0, 0.0, 0.0), 2.0).is_empty());
    }

    #[test]
    fn nearer_point_hides_the_one_behind() {
        // Camera at the origin looking along +z; a point 2 m ahead hides one 4 m ahead on
        // the same ray, but not one 2.03 m ahead (within the tolerance).
        let xyz = [
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 2.03],
            [1.0, 0.0, 4.0],
        ];
        let seen = visible_points(
            &xyz,
            &[0.0; 4],
            &[0, 1, 2, 3],
            &pinhole(),
            &Isometry3::identity(),
            12.0,
            4,
            0.05,
        );
        let idx: Vec<u32> = seen.iter().map(|s| s.index).collect();
        assert_eq!(idx, vec![0, 2, 3]);
        assert!((seen[0].proj.range - 2.0).abs() < 1e-6);
        assert!(seen[0].proj.theta.abs() < 1e-6);
        // Beyond max_dist nothing is seen.
        let far = visible_points(
            &xyz,
            &[0.0; 4],
            &[0, 1, 2, 3],
            &pinhole(),
            &Isometry3::identity(),
            3.0,
            4,
            0.05,
        );
        assert_eq!(far.len(), 2);
    }

    #[test]
    fn close_point_splats_its_footprint() {
        // Spacing 0.2 m at 0.5 m: 0.2 · 50 / 4 / 0.5 = 5 cells → half-size 2, so it hides a
        // point behind it two cells to the side (out of reach of the silhouette check),
        // which a single-cell splat would not.
        let xyz = [[0.0, 0.0, 0.5], [0.64, 0.0, 4.0]];
        let seen = |spacing: f32| visible_count(&xyz, &[spacing, 0.0]);
        assert_eq!(cell_offset(&xyz[1]), 2);
        assert_eq!(seen(0.0), 2);
        assert_eq!(seen(0.2), 1);
    }

    #[test]
    fn point_on_an_occluder_silhouette_is_not_visible() {
        // One cell beside a much nearer point: the image blurs the occluder into it.
        let xyz = [[0.0, 0.0, 0.5], [0.32, 0.0, 4.0]];
        assert_eq!(cell_offset(&xyz[1]), 1);
        assert_eq!(visible_count(&xyz, &[0.0, 0.0]), 1);
        // A slanted surface (depth 10 % less one cell over) does not hide its neighbours.
        let slant = [[0.0, 0.0, 3.6], [0.32, 0.0, 4.0]];
        assert_eq!(visible_count(&slant, &[0.0, 0.0]), 2);
    }

    /// Depth-buffer cells between the image centre and the projection of `p` along x.
    fn cell_offset(p: &[f32; 3]) -> i32 {
        let cam = pinhole();
        let proj = project_point(p, &cam, &Isometry3::identity(), 12.0).unwrap();
        let buffer = DepthBuffer::new(&cam, 4);
        buffer.cell(proj.uv).0 - buffer.cell([49.5, 39.5]).0
    }

    fn visible_count(xyz: &[[f32; 3]], spacing: &[f32]) -> usize {
        let candidates: Vec<u32> = (0..xyz.len() as u32).collect();
        let cam = pinhole();
        visible_points(
            xyz,
            spacing,
            &candidates,
            &cam,
            &Isometry3::identity(),
            12.0,
            4,
            0.05,
        )
        .len()
    }
}
