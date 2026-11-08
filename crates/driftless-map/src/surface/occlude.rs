//! Parts that were measured but have no surface hide what lies behind them from the images
//! (measure `verdeckung` of [`NoiseConfig`](super::NoiseConfig)).
//!
//! A pipe, a ladder or a cable is too thin for the mesh, but the camera sees it: without a
//! face of its own it would be painted onto the wall behind it. So the views must know about
//! the points that lie away from every face.
//!
//! - **Finding them** ([`find_occluders`], after the mesh is simplified): the points further
//!   than `max(`[`OCCLUDER_FAR_VOXELS`]` voxels, `[`OCCLUDER_FAR_MIN`]`)` from every face are
//!   collected in cells of one voxel. A cell with at least [`OCCLUDER_MIN_POINTS`] of them is
//!   an occluder, at its centre; fewer may be stray points. Touching cells (also across corners)
//!   form groups, and a group whose longest side is shorter than [`OCCLUDER_MIN_EXTENT`] is
//!   left out: door handles and crumbs stay in the image as they always did. At most
//!   [`OCCLUDER_MAX`] occluders remain, thinned by a coarser grid so that they stay evenly
//!   spread ([`Occluders::spacing`] says how coarse).
//! - **Their effect** ([`super::views`]): per image a second depth buffer holds only the
//!   occluders ([`occluder_buffer`]), with the cells of the faces' buffer. A view of a face is
//!   occluded if one of 7 samples of the face (centroid, corners, middles of the edges) lies
//!   more than [`OCCLUDER_GAP`] behind an occluder of its own cell ([`face_occluded`]).
//!   An occluded view is not struck off: it counts [`OCCLUDED_SCORE`] of its score and carries
//!   the mark of a view that may show the carrier, so that it goes as soon as another image
//!   shows the face and stays if none does (no new gray faces, no new rule in the filter). It
//!   does not count for the exposure either.
//! - **„Bilder neu“** ([`super::edit`]): the surface step leaves them next to the surface
//!   ([`OCCLUDERS_FILE`], [`Occluders::write`]) and [`super::views::windowed_views`] heeds them
//!   the same way, the depth buffer again only where it is read.
//!
//! Sorted cells and an integer grid make the result independent of the order of the points.

use super::simplify::far_points;
use super::views::Shot;
use super::{Mesh, SurfaceConfig};
use crate::color::ColorConfig;
use crate::visibility::{project_point, DepthBuffer, PointGrid, Projection};
use crate::MapError;
use driftless_io::camera::Camera;
use nalgebra::Isometry3;
use rayon::prelude::*;
use std::path::Path;

/// A face hides behind an occluder that is more than this nearer (m).
pub const OCCLUDER_GAP: f64 = 0.02;
/// The share of its score an occluded view keeps.
pub const OCCLUDED_SCORE: f32 = 0.04;
/// Groups of occluders whose longest side is shorter than this (m) are left out.
pub const OCCLUDER_MIN_EXTENT: f64 = 0.30;
/// Occluders kept at most.
pub const OCCLUDER_MAX: usize = 1_500_000;
/// A point counts as having no face from this many voxels away ...
pub const OCCLUDER_FAR_VOXELS: f64 = 1.5;
/// ... and at least from this distance (m).
pub const OCCLUDER_FAR_MIN: f64 = 0.02;
/// Points a cell needs to be an occluder.
pub const OCCLUDER_MIN_POINTS: usize = 3;
/// The occluders next to a surface, for „Bilder neu“ ([`Occluders::write`]).
pub const OCCLUDERS_FILE: &str = "verdecker.bin";
/// The first bytes of [`OCCLUDERS_FILE`].
const FILE_ID: &[u8; 8] = b"DLVERDK1";

/// The 13 neighbouring cells after a cell in key order: the other 13 see it from their side.
const FORWARD: [[i32; 3]; 13] = [
    [0, 0, 1],
    [0, 1, -1],
    [0, 1, 0],
    [0, 1, 1],
    [1, -1, -1],
    [1, -1, 0],
    [1, -1, 1],
    [1, 0, -1],
    [1, 0, 0],
    [1, 0, 1],
    [1, 1, -1],
    [1, 1, 0],
    [1, 1, 1],
];

/// What hides the surface from the images: see the module docs.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Occluders {
    /// In a fixed order, independent of the order of the points.
    pub points: Vec<[f32; 3]>,
    /// Distance between neighbouring occluders (m): one voxel, more if they were thinned.
    pub spacing: f32,
}

impl Occluders {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Writes them to `path`, atomically (a run killed meanwhile leaves the file as it was): the
    /// bytes of [`FILE_ID`], [`Occluders::spacing`] (`f32`), their number (`u64`), the points
    /// (three `f32` each), all little-endian.
    pub fn write(&self, path: &Path) -> Result<(), MapError> {
        let mut bytes = Vec::with_capacity(FILE_ID.len() + 12 + 12 * self.points.len());
        bytes.extend_from_slice(FILE_ID);
        bytes.extend_from_slice(&self.spacing.to_le_bytes());
        bytes.extend_from_slice(&(self.points.len() as u64).to_le_bytes());
        for v in self.points.iter().flatten() {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        driftless_io::atomic_write(path, &bytes).map_err(|source| MapError::Io {
            path: path.display().to_string(),
            source,
        })
    }

    /// The occluders [`Occluders::write`] wrote to `path`; `None` if there is no such file or it
    /// is not one of them: another start, another size than the number of points says, a number
    /// that is not finite, points without a spacing above 0.
    pub fn read(path: &Path) -> Option<Occluders> {
        let bytes = std::fs::read(path).ok()?;
        let rest = bytes.strip_prefix(FILE_ID.as_slice())?;
        let (spacing, rest) = rest.split_first_chunk::<4>()?;
        let (count, rest) = rest.split_first_chunk::<8>()?;
        let spacing = f32::from_le_bytes(*spacing);
        let count = u64::from_le_bytes(*count);
        let ok_spacing = spacing.is_finite() && (spacing > 0.0 || (count == 0 && spacing == 0.0));
        if !ok_spacing || count.checked_mul(12)? != rest.len() as u64 {
            return None;
        }
        let points: Vec<[f32; 3]> = rest
            .as_chunks::<12>()
            .0
            .iter()
            .map(|p| [0, 4, 8].map(|k| f32::from_le_bytes([p[k], p[k + 1], p[k + 2], p[k + 3]])))
            .collect();
        points
            .iter()
            .flatten()
            .all(|v| v.is_finite())
            .then_some(Occluders { points, spacing })
    }
}

/// What of the points `xyz` has no face in `mesh` and is large enough to hide something: see the
/// module docs.
pub fn find_occluders(mesh: &Mesh, xyz: &[[f32; 3]], cfg: &SurfaceConfig) -> Occluders {
    let far = (OCCLUDER_FAR_VOXELS * cfg.voxel).max(OCCLUDER_FAR_MIN);
    let cell = |p: &[f32; 3]| p.map(|v| (f64::from(v) / cfg.voxel).floor() as i32);
    let mut keys: Vec<[i32; 3]> = far_points(xyz, |_| true, mesh, far)
        .into_par_iter()
        .map(|i| cell(&xyz[i as usize]))
        .collect();
    keys.par_sort_unstable();
    let cells: Vec<[i32; 3]> = keys
        .chunk_by(|a, b| a == b)
        .filter(|c| c.len() >= OCCLUDER_MIN_POINTS)
        .map(|c| c[0])
        .collect();
    drop(keys);
    let (cells, every) = thinned(long_groups(cells, cfg.voxel), OCCLUDER_MAX);
    Occluders {
        points: cells
            .par_iter()
            .map(|c| c.map(|k| ((f64::from(k) + 0.5) * cfg.voxel) as f32))
            .collect(),
        spacing: (f64::from(every) * cfg.voxel) as f32,
    }
}

/// The cells of `cells` (sorted, without repeats) in groups of touching cells whose longest side
/// is at least [`OCCLUDER_MIN_EXTENT`]; the order stays.
fn long_groups(cells: Vec<[i32; 3]>, voxel: f64) -> Vec<[i32; 3]> {
    fn root(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            let up = parent[parent[i as usize] as usize];
            parent[i as usize] = up;
            i = up;
        }
        i
    }
    let mut parent: Vec<u32> =
        (0..u32::try_from(cells.len()).expect("fewer than 2^32 cells")).collect();
    // A step to the same neighbour keeps the key order, so one walk per step finds them all.
    for d in FORWARD {
        let mut j = 0;
        for (i, c) in cells.iter().enumerate() {
            let want = [c[0] + d[0], c[1] + d[1], c[2] + d[2]];
            while cells.get(j).is_some_and(|o| *o < want) {
                j += 1;
            }
            if cells.get(j) == Some(&want) {
                let (a, b) = (root(&mut parent, i as u32), root(&mut parent, j as u32));
                parent[a.max(b) as usize] = a.min(b);
            }
        }
    }
    let (mut lo, mut hi) = (cells.clone(), cells.clone());
    let roots: Vec<u32> = (0..cells.len() as u32)
        .map(|i| root(&mut parent, i))
        .collect();
    for (c, &r) in cells.iter().zip(&roots) {
        let r = r as usize;
        for k in 0..3 {
            lo[r][k] = lo[r][k].min(c[k]);
            hi[r][k] = hi[r][k].max(c[k]);
        }
    }
    cells
        .iter()
        .zip(&roots)
        .filter(|&(_, &r)| {
            let r = r as usize;
            let cells = (0..3).map(|k| hi[r][k] - lo[r][k] + 1).max().unwrap_or(0);
            f64::from(cells) * voxel >= OCCLUDER_MIN_EXTENT
        })
        .map(|(c, _)| *c)
        .collect()
}

/// At most `max` of `cells` (sorted), and how many cells apart they are: as they are, or the
/// first cell of every cell of the finest coarser grid (in steps of whole cells) that leaves few
/// enough.
fn thinned(cells: Vec<[i32; 3]>, max: usize) -> (Vec<[i32; 3]>, i32) {
    if cells.len() <= max {
        return (cells, 1);
    }
    let mut every = 2;
    loop {
        let mut keyed: Vec<([i32; 3], [i32; 3])> = cells
            .par_iter()
            .map(|&c| (c.map(|k| k.div_euclid(every)), c))
            .collect();
        keyed.par_sort_unstable();
        keyed.dedup_by_key(|e| e.0);
        if keyed.len() <= max {
            return (keyed.into_iter().map(|e| e.1).collect(), every);
        }
        every += 1;
    }
}

/// The depth buffer of the occluders near the camera of `shot`, with the cells of the faces'
/// buffer in [`super::views::image_views`]; `None` if there are none (`grid` holds
/// `occluders.points`). With `window` (the least and the greatest cell, as
/// [`DepthBuffer::window`]) it holds only those cells; there it holds what the whole buffer
/// holds.
pub(crate) fn occluder_buffer(
    occluders: &Occluders,
    grid: &PointGrid,
    shot: &Shot,
    cfg: &ColorConfig,
    window: Option<([i32; 2], [i32; 2])>,
) -> Option<DepthBuffer> {
    let near = grid.near(&shot.world_from_cam.translation.vector, cfg.max_dist);
    if near.is_empty() {
        return None;
    }
    let world_to_cam = shot.world_from_cam.inverse();
    let buffer = match window {
        Some((lo, hi)) => DepthBuffer::window(shot.geometry, cfg.zbuffer_scale, lo, hi),
        None => DepthBuffer::new(shot.geometry, cfg.zbuffer_scale),
    };
    near.par_iter().for_each(|&i| {
        let p = &occluders.points[i as usize];
        if let Some(proj) = project_point(p, shot.geometry, &world_to_cam, cfg.max_dist) {
            buffer.splat(&proj, occluders.spacing);
        }
    });
    Some(buffer)
}

/// The 6 samples of a face with these `corners` besides its centroid: the corners and the
/// middles of the edges.
pub(crate) fn side_samples(corners: [[f32; 3]; 3]) -> [[f32; 3]; 6] {
    let middle = |k: usize| {
        let (a, b) = (corners[k], corners[(k + 1) % 3]);
        [0, 1, 2].map(|i| (a[i] + b[i]) * 0.5)
    };
    [
        corners[0],
        middle(0),
        corners[1],
        middle(1),
        corners[2],
        middle(2),
    ]
}

/// Whether an occluder of `buffer` hides one of the 7 samples of a face with these `corners`:
/// its centroid (`centre`, projected), the corners, the middles of the edges.
pub(crate) fn face_occluded(
    buffer: &DepthBuffer,
    corners: [[f32; 3]; 3],
    centre: &Projection,
    cam: &Camera,
    world_to_cam: &Isometry3<f64>,
) -> bool {
    let hidden = |p: &[f32; 3]| {
        project_point(p, cam, world_to_cam, f64::INFINITY)
            .is_some_and(|q| buffer.is_behind(&q, OCCLUDER_GAP))
    };
    buffer.is_behind(centre, OCCLUDER_GAP) || side_samples(corners).iter().any(hidden)
}

#[cfg(test)]
mod tests {
    use super::super::edit::{retexture_region, FaceBvh, SurfacePatch, WeldedSurface};
    use super::super::tests::face_colour;
    use super::super::testutil::{centroid, face_near, grid, scene, test_gpu, Lcg, Steps};
    use super::super::views::{
        all_cameras, collect_views, collect_views_on, image_views, windowed_views, CornerFaces,
        Faces, TextureSource, ViewTable,
    };
    use super::super::{encode_pages, texture_mesh_with, NoiseConfig, SurfaceStats};
    use super::*;
    use crate::color::{linear_table, sample_linear, srgb_value};
    use driftless_export::mesh::Mesh as PageMesh;
    use driftless_gpu::GpuContext;
    use driftless_io::images::{decode_jpeg, index_images};
    use driftless_io::progress::Progress;
    use driftless_io::testdir::TestDir;
    use driftless_traj::interpolate_pose;
    use nalgebra::Isometry3;
    use std::collections::BTreeMap;
    use std::fs;
    use std::sync::Arc;

    /// The rod of [`rod_source`]: its middle (x, m) and half its width. It stands 0.5 m in front
    /// of the +y wall (y 4.46 … 4.54), from z −1.1 to 1.7.
    const ROD_X: f64 = 0.025;
    const ROD_HALF: f64 = 0.04;
    const ROD: [u8; 3] = [40, 35, 30];
    /// The colour of the +y wall of the painted room.
    const WALL: [u8; 3] = scene::WALLS[3];

    fn config() -> SurfaceConfig {
        SurfaceConfig {
            min_texel: 0.01,
            page_size: 1024,
            noise: NoiseConfig::OFF,
            ..SurfaceConfig::default()
        }
    }

    /// The +y wall of the painted room (y = 5, facing −y) as squares of 5 cm.
    fn wall() -> Mesh {
        let (positions, indices) = grid([100, 60], |i, j| {
            [-2.5 + 0.05 * i as f64, 5.0, -1.2 + 0.05 * j as f64]
        });
        Mesh {
            positions: positions.iter().map(|p| p.map(|c| c as f32)).collect(),
            indices,
        }
    }

    /// A sheet of points at `y`, x and z in ±`half`, every 5 mm.
    fn sheet(y: f32, half: [f32; 2]) -> Vec<[f32; 3]> {
        let n = half.map(|h| (h / 0.005) as i32);
        (-n[0]..=n[0])
            .flat_map(|i| (-n[1]..=n[1]).map(move |j| [i as f32 * 0.005, y, j as f32 * 0.005]))
            .collect()
    }

    /// Points through a box of the rod's cross-section from `z0` to `z1`, every centimetre.
    fn rod_points(z0: f64, z1: f64) -> Vec<[f32; 3]> {
        let along = ((z1 - z0) / 0.01).round() as i32;
        let across = (2.0 * ROD_HALF / 0.01).round() as i32;
        let mut xyz = Vec::new();
        for i in 0..=across {
            for j in 0..=across {
                for k in 0..=along {
                    xyz.push([
                        (ROD_X - ROD_HALF + 0.01 * f64::from(i)) as f32,
                        (4.46 + 0.01 * f64::from(j)) as f32,
                        (z0 + 0.01 * f64::from(k)) as f32,
                    ]);
                }
            }
        }
        xyz
    }

    /// The whole rod.
    fn full_rod() -> Vec<[f32; 3]> {
        rod_points(-1.1, 1.7)
    }

    /// The images of the camera `left` looking at the wall from 1.5 m at `places` (x, m), one a
    /// second; the images show a thin rod in front of the wall that the mesh does not have. From
    /// each place the rod hides another stripe of the wall.
    fn rod_source(name: &str, places: &[f64]) -> TextureSource {
        let painted = scene::Painted::new(vec![(
            scene::aabb(
                [ROD_X - ROD_HALF, 4.46, -1.1],
                [ROD_X + ROD_HALF, 4.54, 1.7],
            ),
            ROD,
        )]);
        let cam = scene::cameras()[0].clone();
        let cam_y = cam.cam_in_body.translation.vector.y;
        let traj = Steps(
            places
                .iter()
                .enumerate()
                .map(|(i, &x)| {
                    let from = if i == 0 {
                        0.0
                    } else {
                        scene::T0 + 0.5 + i as f64
                    };
                    (from, Isometry3::translation(x, 3.5 - cam_y, 0.0))
                })
                .collect(),
        );
        let shots: Vec<scene::Shot> = (0..places.len())
            .map(|i| scene::shot(0, scene::T0 + 1.0 + i as f64))
            .collect();
        TextureSource {
            images: scene::write_bag(name, &painted, &traj, std::slice::from_ref(&cam), &shots),
            poses: scene::poses(&traj, 6.0),
            cams: vec![cam],
        }
    }

    /// The faces of `mesh` that the rod hides completely, seen from the first place of `src`.
    fn in_the_rods_shadow(mesh: &Mesh, src: &TextureSource) -> Vec<usize> {
        let first = src.poses[0].p + src.poses[0].q * src.cams[0].cam_in_body.translation.vector;
        // From the camera through the rod's middle (y 4.5) onto the wall (y 5).
        let mid = first.x + (ROD_X - first.x) * (5.0 - first.y) / (4.5 - first.y);
        let faces: Vec<usize> = (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(mesh, f).map(f64::from);
                (c[0] - mid).abs() < 0.02 && c[2].abs() < 0.5
            })
            .collect();
        assert!(faces.len() > 10, "{}", faces.len());
        faces
    }

    fn views(mesh: &Mesh, occluders: &Occluders, src: &TextureSource) -> ViewTable {
        views_on(mesh, occluders, src, None)
    }

    fn views_on(
        mesh: &Mesh,
        occluders: &Occluders,
        src: &TextureSource,
        gpu: Option<&GpuContext>,
    ) -> ViewTable {
        collect_views_on(
            mesh,
            occluders,
            std::slice::from_ref(src),
            &config(),
            &Progress::new(),
            gpu.map(|g| (g, None)),
        )
        .unwrap()
        .0
    }

    /// How far `colour` is from the colour of the wall (sRGB levels, any channel).
    fn off_the_wall(colour: [u8; 3]) -> i32 {
        (0..3)
            .map(|c| (i32::from(colour[c]) - i32::from(WALL[c])).abs())
            .max()
            .unwrap()
    }

    #[test]
    fn rod_without_surface_is_found() {
        let cfg = config();
        let mesh = wall();
        let mut xyz = sheet(5.0, [1.0, 0.5]);
        xyz.extend(full_rod());
        let found = find_occluders(&mesh, &xyz, &cfg);
        assert!(!found.is_empty());
        assert_eq!(found.spacing, cfg.voxel as f32);
        // All of them on the rod (the cell centres lie within a voxel of the box), along its
        // whole length.
        for p in &found.points {
            assert!(
                (f64::from(p[0]) - ROD_X).abs() < ROD_HALF + 0.02
                    && (f64::from(p[1]) - 4.5).abs() < 0.06
                    && (-1.12..=1.72).contains(&f64::from(p[2])),
                "{p:?}"
            );
        }
        let z = |take: fn(f32, f32) -> f32| found.points.iter().map(|p| p[2]).reduce(take).unwrap();
        assert!(z(f32::min) < -1.0 && z(f32::max) > 1.6);
    }

    #[test]
    fn points_on_the_surface_are_not_occluders() {
        let cfg = config();
        let mesh = wall();
        // On the wall, 1 cm and 2.5 cm in front of it (1.5 voxels are 3 cm).
        for y in [5.0, 4.99, 4.975] {
            let found = find_occluders(&mesh, &sheet(y, [0.5, 0.2]), &cfg);
            assert!(found.is_empty(), "{y}: {}", found.points.len());
        }
        // Beyond the distance they count.
        assert!(!find_occluders(&mesh, &sheet(4.95, [0.5, 0.2]), &cfg).is_empty());
        // Without a mesh, nothing has a face.
        assert!(!find_occluders(&Mesh::default(), &sheet(5.0, [0.5, 0.2]), &cfg).is_empty());
    }

    #[test]
    fn group_under_30_cm_is_left_out() {
        let cfg = config();
        let mesh = wall();
        assert!(find_occluders(&mesh, &rod_points(0.0, 0.2), &cfg).is_empty());
        assert!(!find_occluders(&mesh, &rod_points(0.0, 0.4), &cfg).is_empty());
        // Two short pieces that do not touch are two groups: too short together as well.
        let mut two = rod_points(0.0, 0.2);
        two.extend(rod_points(0.5, 0.7));
        assert!(find_occluders(&mesh, &two, &cfg).is_empty());
        // Touching across a corner is touching: 16 cells (32 cm) along the diagonal.
        let diagonal: Vec<[f32; 3]> = (0..16)
            .flat_map(|i| {
                [0.25, 0.5, 0.75].map(|s| {
                    let v = ((f64::from(i) + s) * cfg.voxel) as f32;
                    [v, v, v]
                })
            })
            .collect();
        let found = find_occluders(&Mesh::default(), &diagonal, &cfg);
        assert_eq!(found.points.len(), 16);
    }

    #[test]
    fn cell_needs_three_points() {
        let cfg = config();
        // A line of 25 cells (50 cm) with `n` points in each.
        let line = |n: usize| -> Vec<[f32; 3]> {
            (0..25)
                .flat_map(|i| {
                    (0..n).map(move |k| {
                        let x = (i as f64 + (k as f64 + 0.5) / n as f64) * 0.02;
                        [x as f32, 4.51, 0.01]
                    })
                })
                .collect()
        };
        assert!(find_occluders(&wall(), &line(2), &cfg).is_empty());
        assert_eq!(find_occluders(&wall(), &line(3), &cfg).points.len(), 25);
    }

    #[test]
    fn at_most_the_limit_evenly_thinned() {
        // 120³ cells of 2 cm with 3 points each, more than the limit; no face anywhere.
        let cfg = config();
        let at = |k: i32, s: f64| ((f64::from(k) + s) * cfg.voxel) as f32;
        let xyz: Vec<[f32; 3]> = (0..120)
            .flat_map(|x| {
                (0..120).flat_map(move |y| {
                    (0..120).flat_map(move |z| {
                        [0.25, 0.5, 0.75].map(|s| [at(x, s), at(y, s), at(z, s)])
                    })
                })
            })
            .collect();
        let found = find_occluders(&Mesh::default(), &xyz, &cfg);
        assert!(found.points.len() <= OCCLUDER_MAX, "{}", found.points.len());
        // Every second cell along each axis: one in eight remains.
        assert_eq!(found.points.len(), 60 * 60 * 60);
        assert_eq!(found.spacing, (2.0 * cfg.voxel) as f32);
        // Spread over the whole block.
        let high = found.points.iter().filter(|p| p[0] > 1.2).count();
        assert_eq!(high, found.points.len() / 2);
    }

    #[test]
    fn same_result_in_another_order() {
        let cfg = config();
        let mesh = wall();
        let mut xyz = sheet(5.0, [1.0, 0.5]);
        xyz.extend(full_rod());
        let found = find_occluders(&mesh, &xyz, &cfg);
        let mut rng = Lcg::new(7);
        for i in (1..xyz.len()).rev() {
            xyz.swap(i, (rng.unit() * (i + 1) as f64) as usize);
        }
        assert_eq!(find_occluders(&mesh, &xyz, &cfg), found);
    }

    #[test]
    fn rod_is_not_painted_on_wall() {
        rod_case("occlude-rod", None);
    }

    #[test]
    fn rod_is_not_painted_on_wall_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        rod_case("occlude-rod-gpu", Some(&ctx));
    }

    fn rod_case(bag: &str, gpu: Option<&GpuContext>) {
        // Head-on from x = 0 the rod hides a stripe of the wall in the best image; the second
        // image, from the side, shows the wall there. With two views the photo consistency
        // cannot tell which is right; only the occluders can.
        let mesh = wall();
        let src = rod_source(bag, &[0.0, 1.0]);
        let cfg = config();
        let sources = std::slice::from_ref(&src);
        let occluders = find_occluders(&mesh, &full_rod(), &cfg);
        assert!(!occluders.is_empty());
        let run = |occ: &Occluders| {
            let mut stats = SurfaceStats::default();
            let tex =
                texture_mesh_with(&mesh, occ, sources, &cfg, &Progress::new(), &mut stats, gpu)
                    .unwrap();
            (tex, stats)
        };
        let (plain, stats) = run(&Occluders::default());
        assert_eq!(stats.occluders, 0);
        let (seen, stats) = run(&occluders);
        assert_eq!(stats.occluders, occluders.points.len());
        for f in in_the_rods_shadow(&mesh, &src) {
            let (before, after) = (face_colour(&plain, f), face_colour(&seen, f));
            assert!(off_the_wall(before) > 100, "face {f}: {before:?}");
            assert!(off_the_wall(after) <= 3 * scene::TOL, "face {f}: {after:?}");
        }
    }

    #[test]
    fn occluded_view_is_tagged_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let mesh = wall();
        let occluders = find_occluders(&mesh, &full_rod(), &config());
        let k = config().views_per_face;
        let faces = mesh.indices.len();
        // The views of a face that remain, in order, and which of them are marked.
        let marked = |t: &ViewTable, f: usize| -> Vec<(u32, bool)> {
            t.per_face[f * k..(f + 1) * k]
                .iter()
                .filter(|v| v.score > 0.0)
                .map(|v| (v.image, v.carrier()))
                .collect()
        };
        // One image: the marked views stay. Two: those with another view are gone again.
        for places in [&[0.0][..], &[0.0, 1.0][..]] {
            let bag = format!("occlude-tagged-gpu-{}", places.len());
            let src = rod_source(&bag, places);
            let cpu = views(&mesh, &occluders, &src);
            let gpu = views_on(&mesh, &occluders, &src, Some(&ctx));
            let tagged = (0..faces)
                .filter(|&f| marked(&cpu, f).iter().any(|v| v.1))
                .count();
            let differ = (0..faces)
                .filter(|&f| marked(&cpu, f) != marked(&gpu, f))
                .count();
            println!(
                "{} Bilder: {tagged} markiert, {differ} anders",
                places.len()
            );
            // Rounding may move a sample at the edge of the stripe into the next cell.
            assert!(differ * 200 <= faces, "{differ} of {faces}");
            if let [_] = places {
                assert!(tagged > 10, "{tagged}");
                let plain = views_on(&mesh, &Occluders::default(), &src, Some(&ctx));
                for f in in_the_rods_shadow(&mesh, &src) {
                    let (was, is) = (plain.per_face[f * k], gpu.per_face[f * k]);
                    assert!(
                        is.score > 0.0 && is.carrier() && !was.carrier(),
                        "face {f}: {is:?}"
                    );
                    let ratio = is.score / was.score;
                    assert!((ratio - OCCLUDED_SCORE).abs() < 1e-3, "face {f}: {ratio}");
                }
            }
        }
    }

    #[test]
    fn occluders_in_several_passes_match_one_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let mesh = wall();
        let src = rod_source("occlude-passes-gpu", &[0.0, 1.0]);
        let cfg = config();
        let occluders = find_occluders(&mesh, &full_rod(), &cfg);
        let run = |ctx: &GpuContext| {
            collect_views_on(
                &mesh,
                &occluders,
                std::slice::from_ref(&src),
                &cfg,
                &Progress::new(),
                Some((ctx, None)),
            )
            .unwrap()
        };
        let (one, info) = run(&ctx);
        assert_eq!(info.gpu_passes, 1);
        let (image, depth) = super::super::views_gpu::stream_sizes(
            &all_cameras(std::slice::from_ref(&src)),
            &cfg.color,
        );
        // Room for the slots of a quarter of the faces, and for all occluders in every pass.
        let count = mesh.indices.len();
        let k = cfg.views_per_face;
        let n = occluders.points.len();
        let budget = driftless_gpu::faces_pass_bytes(count, count / 4, k, 0, image, depth, n);
        let (many, info) = run(&ctx.with_budget(budget));
        assert!(info.gpu_passes >= 4, "{}", info.gpu_passes);
        assert!(one == many);
    }

    #[test]
    fn only_occluded_views_stay() {
        // One image: the stripe behind the rod has no other. It keeps it, the view counts
        // little and carries the mark.
        let mesh = wall();
        let src = rod_source("occlude-only", &[0.0]);
        let occluders = find_occluders(&mesh, &full_rod(), &config());
        let (plain, table) = (
            views(&mesh, &Occluders::default(), &src),
            views(&mesh, &occluders, &src),
        );
        let k = config().views_per_face;
        let shadow = in_the_rods_shadow(&mesh, &src);
        for &f in &shadow {
            let (was, is) = (plain.per_face[f * k], table.per_face[f * k]);
            assert!(
                is.score > 0.0 && is.carrier() && !was.carrier(),
                "face {f}: {is:?}"
            );
            let ratio = is.score / was.score;
            assert!((ratio - OCCLUDED_SCORE).abs() < 1e-3, "face {f}: {ratio}");
        }
        let cfg = config();
        let mut stats = SurfaceStats::default();
        let tex = texture_mesh_with(
            &mesh,
            &occluders,
            std::slice::from_ref(&src),
            &cfg,
            &Progress::new(),
            &mut stats,
            None,
        )
        .unwrap();
        for &f in &shadow {
            assert!(tex.textured[f], "face {f} went gray");
        }
    }

    #[test]
    fn file_gives_the_same_points() {
        let dir = TestDir::new(format!("driftless-occluders-{}", std::process::id()));
        let path = dir.join(OCCLUDERS_FILE);
        let found = find_occluders(&wall(), &full_rod(), &config());
        assert!(!found.is_empty());
        found.write(&path).unwrap();
        assert!(Occluders::read(&path).as_ref() == Some(&found));
        // Writing again replaces it; none of them is one as well.
        let few = Occluders {
            points: vec![[1.0, -2.5, 0.125]],
            spacing: 0.04,
        };
        few.write(&path).unwrap();
        assert_eq!(Occluders::read(&path).as_ref(), Some(&few));
        Occluders::default().write(&path).unwrap();
        assert_eq!(Occluders::read(&path), Some(Occluders::default()));
    }

    #[test]
    fn broken_file_is_none() {
        let dir = TestDir::new(format!("driftless-occluders-broken-{}", std::process::id()));
        let path = dir.join(OCCLUDERS_FILE);
        assert_eq!(Occluders::read(&path), None, "no file");
        let good = Occluders {
            points: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            spacing: 0.02,
        };
        good.write(&path).unwrap();
        let bytes = fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 20 + 2 * 12);
        let read = |b: &[u8]| {
            fs::write(&path, b).unwrap();
            Occluders::read(&path)
        };
        assert_eq!(read(&bytes).as_ref(), Some(&good));
        assert_eq!(read(&[]), None, "empty");
        assert_eq!(read(&bytes[..10]), None, "shorter than its head");
        assert_eq!(read(&bytes[..bytes.len() - 1]), None, "a byte short");
        let mut longer = bytes.clone();
        longer.push(0);
        assert_eq!(read(&longer), None, "a byte too many");
        let mut other = bytes.clone();
        other[0] ^= 0xff;
        assert_eq!(read(&other), None, "not ours");
        let mut count = bytes.clone();
        count[12..20].copy_from_slice(&3u64.to_le_bytes());
        assert_eq!(read(&count), None, "more than it holds");
        count[12..20].copy_from_slice(&u64::MAX.to_le_bytes());
        assert_eq!(read(&count), None, "absurdly many");
        let mut point = bytes.clone();
        point[24..28].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(read(&point), None, "no number");
        for spacing in [0.0f32, -1.0, f32::INFINITY] {
            let mut bad = bytes.clone();
            bad[8..12].copy_from_slice(&spacing.to_le_bytes());
            assert_eq!(read(&bad), None, "spacing {spacing}");
        }
    }

    #[test]
    fn windowed_views_heed_occluders_like_image_views() {
        let mesh = wall();
        let cfg = config();
        let ccfg = &cfg.color;
        let src = rod_source("occlude-window", &[0.0]);
        let occluders = find_occluders(&mesh, &full_rod(), &cfg);
        let grid = PointGrid::new(&occluders.points);
        let geometry = Faces::new(&mesh);
        let all: Vec<u32> = (0..mesh.indices.len() as u32).collect();
        // A Stelle across the rod's shadow; every face of the wall splats.
        let view: Vec<u32> = all
            .iter()
            .copied()
            .filter(|&f| {
                let c = centroid(&mesh, f as usize);
                (-0.7..0.7).contains(&c[0]) && (-0.4..0.4).contains(&c[2])
            })
            .collect();
        let positions: Vec<[f64; 3]> = mesh.positions.iter().map(|p| p.map(f64::from)).collect();
        let corners = CornerFaces {
            positions: &positions,
            faces: &mesh.indices,
        };
        let cam = &src.cams[0];
        let body = interpolate_pose(&src.poses, scene::T0 + 1.0).unwrap();
        let shot = Shot {
            geometry: &cam.camera.downscaled(ccfg.downscale),
            full: &cam.camera,
            world_from_cam: body * cam.cam_in_body,
            rank: 0,
            sharpness_norm: 1.0,
            rgb: None,
            min_texel: cfg.min_texel,
        };
        let cover = occluder_buffer(&occluders, &grid, &shot, ccfg, None);
        assert!(cover.is_some());
        let lin = linear_table();
        let whole = image_views(
            &mesh,
            &geometry,
            &all,
            &view,
            &shot,
            cover.as_ref(),
            ccfg,
            |_| false,
            &lin,
        );
        assert!(whole.len() > 20, "{}", whole.len());
        let found = Some((&occluders, &grid));
        for cutout in [false, true] {
            let (views, _) = windowed_views(
                &mesh, &geometry, &view, corners, &all, found, &shot, ccfg, cutout,
            );
            assert_eq!(views.len(), whole.len(), "cut out: {cutout}");
            let hidden = views.iter().filter(|v| v.2).count();
            assert!(hidden > 10, "cut out: {cutout}, {hidden}");
            assert!(hidden < views.len(), "cut out: {cutout}");
            for ((f, v, h), (g, w, _)) in views.iter().zip(&whole) {
                assert_eq!((f, v), (g, w), "cut out: {cutout}");
                assert_eq!(*h, v.carrier(), "cut out: {cutout}, face {f}");
            }
        }
        // Nothing to hide: as without.
        let (none, _) = windowed_views(
            &mesh, &geometry, &view, corners, &all, None, &shot, ccfg, true,
        );
        assert_eq!(none.len(), whole.len());
        assert!(none.iter().all(|v| !v.2 && !v.1.carrier()));
    }

    /// The surface of `wall` as the surface step leaves it: textured from `src` with `occluders`,
    /// as pages and welded.
    fn textured_wall(
        mesh: &Mesh,
        occluders: &Occluders,
        src: &TextureSource,
        cfg: &SurfaceConfig,
    ) -> (PageMesh, WeldedSurface) {
        let mut stats = SurfaceStats::default();
        let tex = texture_mesh_with(
            mesh,
            occluders,
            std::slice::from_ref(src),
            cfg,
            &Progress::new(),
            &mut stats,
            None,
        )
        .unwrap();
        let jpegs = encode_pages(&tex, cfg.jpeg_quality).unwrap();
        let pages = tex.to_mesh(&jpegs).unwrap();
        let welded = WeldedSurface::from_mesh(&pages);
        (pages, welded)
    }

    /// The colour of every face on the new pages of `patch` (at the middle of its triangle in the
    /// texture), by its number in the surface.
    fn patch_colours(patch: &SurfacePatch) -> BTreeMap<u32, [u8; 3]> {
        let lin = linear_table();
        let mut colours = BTreeMap::new();
        for page in &patch.pages {
            let image = decode_jpeg(&page.jpeg, 1).unwrap();
            for (tri, &f) in page.indices.iter().zip(&page.faces) {
                let uv =
                    [0, 1].map(|k| tri.iter().map(|&v| page.uvs[v as usize][k]).sum::<f32>() / 3.0);
                let px = [
                    uv[0] * image.width as f32 - 0.5,
                    uv[1] * image.height as f32 - 0.5,
                ];
                let colour = sample_linear(&image, px, &lin).map(srgb_value);
                colours.insert(f, colour.map(|v| v.round().clamp(0.0, 255.0) as u8));
            }
        }
        colours
    }

    /// The faces of the Stelle `spot` of `s` the rod hides completely seen from the camera at
    /// `place` (x, m), as [`in_the_rods_shadow`].
    fn shadow_on(s: &WeldedSurface, spot: &[u32], place: f64) -> Vec<u32> {
        let mid = place + (ROD_X - place) * (5.0 - 3.5) / (4.5 - 3.5);
        spot.iter()
            .copied()
            .filter(|&f| {
                let c = s.face_centroid(f);
                (c[0] - mid).abs() < 0.02 && c[2].abs() < 0.5
            })
            .collect()
    }

    /// A Stelle on the wall that holds the rod's shadow from both of [`retexture_case`]'s
    /// places: x from −0.7 to 0.3, z within ±0.55.
    fn spot_around_the_shadows(s: &WeldedSurface) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                (-0.7..0.3).contains(&c[0]) && c[2].abs() < 0.55
            })
            .collect()
    }

    #[test]
    fn retexture_keeps_rod_off_the_wall() {
        // Head-on the rod hides one stripe of the Stelle, from the side another; each image
        // shows the wall where the other shows the rod.
        let mesh = wall();
        let cfg = config();
        let src = rod_source("occlude-retexture", &[0.0, 1.0]);
        let occluders = find_occluders(&mesh, &full_rod(), &cfg);
        assert!(!occluders.is_empty());
        let (pages, s) = textured_wall(&mesh, &occluders, &src, &cfg);
        let bvh = FaceBvh::build(&s);
        let spot = spot_around_the_shadows(&s);
        let sources = std::slice::from_ref(&src);
        let indices = vec![Arc::new(
            index_images(&src.images.bags, &src.images.topics, &Progress::new()).unwrap(),
        )];
        let run = |occ: &Occluders, variant: u32| {
            retexture_region(
                &s,
                &bvh,
                &pages,
                &spot,
                occ,
                sources,
                &indices,
                variant,
                &cfg,
                &Progress::new(),
            )
            .unwrap()
        };
        let mut shadow = shadow_on(&s, &spot, 0.0);
        let first = shadow.len();
        shadow.extend(shadow_on(&s, &spot, 1.0));
        assert!(first > 10 && shadow.len() > first + 10, "{}", shadow.len());
        // Without them every variant's image brings its stripe of the rod along.
        let mut with_rod = 0;
        for variant in 0..2 {
            let colours = patch_colours(&run(&Occluders::default(), variant).patch);
            with_rod += shadow
                .iter()
                .filter(|f| off_the_wall(colours[f]) > 100)
                .count();
        }
        assert!(with_rod > 10, "{with_rod}");
        // With them no image paints it, and every face of the shadows still has an image.
        for variant in 0..2 {
            let done = run(&occluders, variant);
            let colours = patch_colours(&done.patch);
            for f in &shadow {
                let off = off_the_wall(colours[f]);
                assert!(off <= 3 * scene::TOL, "variant {variant}, face {f}: {off}");
            }
            assert_eq!(done.patch.removed_faces.len(), spot.len(), "{variant}");
        }
    }

    #[test]
    fn retexture_keeps_the_only_view() {
        // One image: the stripe behind the rod has no other, so it stays (no new gray faces),
        // though the image shows the rod there.
        let mesh = wall();
        let cfg = config();
        let src = rod_source("occlude-retexture-one", &[0.0]);
        let occluders = find_occluders(&mesh, &full_rod(), &cfg);
        let (pages, s) = textured_wall(&mesh, &occluders, &src, &cfg);
        let bvh = FaceBvh::build(&s);
        let spot = spot_around_the_shadows(&s);
        let sources = std::slice::from_ref(&src);
        let indices = vec![Arc::new(
            index_images(&src.images.bags, &src.images.topics, &Progress::new()).unwrap(),
        )];
        let done = retexture_region(
            &s,
            &bvh,
            &pages,
            &spot,
            &occluders,
            sources,
            &indices,
            0,
            &cfg,
            &Progress::new(),
        )
        .unwrap();
        let shadow = shadow_on(&s, &spot, 0.0);
        assert!(shadow.len() > 10, "{}", shadow.len());
        let colours = patch_colours(&done.patch);
        for f in &shadow {
            assert!(off_the_wall(colours[f]) > 100, "face {f}: {:?}", colours[f]);
        }
    }

    /// Occluders in a patch at `y` around the middle of the wall.
    fn patch_at(y: f32) -> Occluders {
        let voxel = config().voxel as f32;
        Occluders {
            points: (-20..=20)
                .flat_map(|i| (-15..=15).map(move |j| [i as f32 * voxel, y, j as f32 * voxel]))
                .collect(),
            spacing: voxel,
        }
    }

    #[test]
    fn face_in_front_of_occluder_is_seen() {
        let mesh = wall();
        let src = rod_source("occlude-front", &[0.0]);
        let plain = views(&mesh, &Occluders::default(), &src);
        // Behind the wall, and less than the gap in front of it: nothing changes.
        for y in [5.5, 4.99] {
            assert!(
                views(&mesh, &patch_at(y), &src).per_face == plain.per_face,
                "{y}"
            );
        }
        // In front of it by more than the gap: the faces behind the patch are occluded.
        let hidden = views(&mesh, &patch_at(4.9), &src);
        let k = config().views_per_face;
        let f = face_near(&mesh, [0.1, 5.0, 0.0]);
        assert!(
            hidden.per_face[f * k].carrier(),
            "{:?}",
            hidden.per_face[f * k]
        );
        assert!(!plain.per_face[f * k].carrier());
    }

    #[test]
    fn no_occluders_same_views() {
        let mesh = wall();
        let src = rod_source("occlude-none", &[0.0, 1.0]);
        let cfg = config();
        let plain =
            collect_views(&mesh, std::slice::from_ref(&src), &cfg, &Progress::new()).unwrap();
        assert!(views(&mesh, &Occluders::default(), &src) == plain);
        // Behind the camera (a buffer, but nothing in it reaches the faces) and out of its reach
        // (no buffer at all).
        let voxel = cfg.voxel as f32;
        for at in [[0.0, 2.0, 0.0], [0.0, 5.0, 40.0]] {
            let far = Occluders {
                points: vec![at],
                spacing: voxel,
            };
            assert!(views(&mesh, &far, &src) == plain, "{at:?}");
        }
    }

    #[test]
    fn no_occluders_same_views_gpu() {
        let Some(ctx) = test_gpu() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let mesh = wall();
        let src = rod_source("occlude-none-gpu", &[0.0, 1.0]);
        let plain = views_on(&mesh, &Occluders::default(), &src, Some(&ctx));
        assert!(plain.per_face.iter().any(|v| v.score > 0.0));
        assert!(plain.per_face.iter().all(|v| !v.carrier()));
        // The pass over the occluders runs and finds no cell a face looks at.
        let voxel = config().voxel as f32;
        for at in [[0.0, 2.0, 0.0], [0.0, 5.0, 40.0]] {
            let far = Occluders {
                points: vec![at],
                spacing: voxel,
            };
            assert!(views_on(&mesh, &far, &src, Some(&ctx)) == plain, "{at:?}");
        }
    }
}
