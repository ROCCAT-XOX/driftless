//! Puts the raw mesh onto the measured points (measure `anlegen` of [`NoiseConfig`](super::NoiseConfig)).
//!
//! The TSDF averages the band of the LiDAR over voxels and leaves its noise in the mesh as
//! small bumps. Every vertex of the raw mesh therefore looks at the cleaned points around it
//! and, if they lie on a plane, moves onto that plane along its normal. The plane is the
//! one the points are smoothed with ([`project`]): at least 10 neighbours, 5 % planarity, the
//! inner plane first, outliers out.
//!
//! - The points are taken within [`RADIUS_VOXELS`] voxels, at least [`MIN_RADIUS`]; with too
//!   few, once within twice that.
//! - A vertex more than one voxel from the plane stays: something else is going on there,
//!   an edge for example.
//! - A vertex without a planar neighbourhood (edges, thin parts) stays. As with the points,
//!   a narrow seam along edges is left as it is.
//! - A face that the moves would turn by more than 60° keeps its three vertices where they
//!   were, and so on for the faces that this turns in turn.
//!
//! Every vertex is moved from the input positions, never from already moved ones, and
//! neighbours at the same distance are ordered by their coordinates, so the result does not
//! depend on the order of the points or the threads. Only `+ − × ÷` and `√` on the way.

use super::{Mesh, SurfaceConfig};
use crate::clean::vec3;
use crate::smooth::{project, SmoothConfig};
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use nalgebra::Vector3;
use rayon::prelude::*;

/// What [`snap_to_points`] did.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SnapStats {
    /// Vertices of the raw mesh with a face.
    pub vertices: usize,
    /// Of them: moved onto the plane of their points.
    pub moved: usize,
    /// Stayed: too few points around them, or these do not lie on a plane.
    pub no_plane: usize,
    /// Stayed: more than one voxel from the plane.
    pub too_far: usize,
    /// Would have moved, but a face would have turned by more than 60°.
    pub reverted: usize,
    /// Mean shift of the moved vertices (mm).
    pub mean_shift_mm: f64,
}

/// Radius of the neighbourhood of a vertex, in voxels ...
const RADIUS_VOXELS: f64 = 2.5;

/// ... and at least this (m).
const MIN_RADIUS: f64 = 0.03;

/// A face has turned too far if the cosine of the angle between its normals is below this
/// (60°).
const MAX_TURN_COS: f64 = 0.5;

/// What the points say about one vertex.
#[derive(Clone, Copy)]
enum Fate {
    /// It goes here.
    To([f32; 3]),
    /// It is on the plane already, to the precision of its coordinates.
    Stays,
    NoPlane,
    TooFar,
}

/// Moves the vertices of `mesh` onto the plane of the points of `xyz` around them (see the
/// module documentation); `tree` is the k-d tree of `xyz`. Vertices without a face stay.
pub(crate) fn snap_to_points(
    mesh: &mut Mesh,
    tree: &ImmutableKdTree<f32, 3>,
    xyz: &[[f32; 3]],
    cfg: &SurfaceConfig,
) -> SnapStats {
    let radius = (RADIUS_VOXELS * cfg.voxel).max(MIN_RADIUS);
    let mut used = vec![false; mesh.positions.len()];
    for &v in mesh.indices.iter().flatten() {
        used[v as usize] = true;
    }
    let fate = |p: &[f32; 3]| -> Fate {
        let Some(shift) = plane_shift(tree, xyz, p, radius, true) else {
            return Fate::NoPlane;
        };
        if shift.norm() > cfg.voxel {
            return Fate::TooFar;
        }
        let q = vec3(p) + shift;
        let q = [q.x as f32, q.y as f32, q.z as f32];
        if q == *p {
            Fate::Stays
        } else {
            Fate::To(q)
        }
    };
    let fates: Vec<Option<Fate>> = mesh
        .positions
        .par_iter()
        .zip(&used)
        .map(|(p, &u)| u.then(|| fate(p)))
        .collect();

    let mut stats = SnapStats::default();
    let mut new = mesh.positions.clone();
    for (q, f) in new.iter_mut().zip(&fates) {
        let Some(f) = f else { continue };
        stats.vertices += 1;
        match f {
            Fate::To(to) => *q = *to,
            Fate::Stays => {}
            Fate::NoPlane => stats.no_plane += 1,
            Fate::TooFar => stats.too_far += 1,
        }
    }
    drop(fates);

    // Taking a vertex back can turn the next face, hence until nothing turns any more. Every
    // round takes back at least one vertex.
    let old = &mesh.positions;
    loop {
        let turned_faces: Vec<usize> = mesh
            .indices
            .par_iter()
            .enumerate()
            .filter(|(_, t)| turned(old, &new, t))
            .map(|(f, _)| f)
            .collect();
        if turned_faces.is_empty() {
            break;
        }
        for f in turned_faces {
            for v in mesh.indices[f] {
                let v = v as usize;
                if new[v] != old[v] {
                    new[v] = old[v];
                    stats.reverted += 1;
                }
            }
        }
    }

    let mut total_mm = 0.0;
    for (a, b) in old.iter().zip(&new) {
        if a != b {
            stats.moved += 1;
            total_mm += (vec3(b) - vec3(a)).norm() * 1000.0;
        }
    }
    if stats.moved > 0 {
        stats.mean_shift_mm = total_mm / stats.moved as f64;
    }
    mesh.positions = new;
    stats
}

/// Whether a move from `old` to `new` turns the face `t` by more than 60°, or collapses it; a
/// face without area before has no direction to turn away from.
fn turned(old: &[[f32; 3]], new: &[[f32; 3]], t: &[u32; 3]) -> bool {
    if t.iter().all(|&v| old[v as usize] == new[v as usize]) {
        return false;
    }
    let cross = |pos: &[[f32; 3]]| {
        let [a, b, c] = t.map(|v| vec3(&pos[v as usize]));
        (b - a).cross(&(c - a))
    };
    let (before, after) = (cross(old), cross(new));
    let (lb, la) = (before.norm(), after.norm());
    lb > 0.0 && (la == 0.0 || before.dot(&after) < MAX_TURN_COS * lb * la)
}

/// The shift of `p` onto the plane of the points of `xyz` within `radius` (see [`project`]), or
/// `None` with too few points or if they do not lie on a plane. With `widen`, too few points
/// are looked for once more within twice the radius.
fn plane_shift(
    tree: &ImmutableKdTree<f32, 3>,
    xyz: &[[f32; 3]],
    p: &[f32; 3],
    radius: f64,
    widen: bool,
) -> Option<Vector3<f64>> {
    let base = SmoothConfig::default();
    let mut r = radius;
    let mut nn = within(tree, xyz, p, r);
    if nn.len() < base.min_neighbours && widen {
        r *= 2.0;
        nn = within(tree, xyz, p, r);
    }
    if nn.len() < base.min_neighbours {
        return None;
    }
    // The cap of `project` must not hide a vertex that is too far away: the plane passes
    // within `r` of `p`, so a limit of `r` never bites.
    let cfg = SmoothConfig {
        radius: r,
        max_shift: r,
        ..base
    };
    // As for the points: evenly spaced in distance rank, so that the neighbourhood spans the
    // whole radius.
    let found = nn.len();
    let take = found.min(cfg.max_neighbours);
    let picked = (0..take).map(|i| nn[i * found / take]);
    let origin = vec3(p);
    let rel: Vec<Vector3<f64>> = picked
        .clone()
        .map(|(_, i)| vec3(&xyz[i as usize]) - origin)
        .collect();
    let half = (0.25 * r * r) as f32;
    let inner = picked.take_while(|&(d, _)| d <= half).count();
    project(&rel, inner, &cfg)
}

/// The points of `xyz` within `radius` of `p` as (squared distance, index), nearest first; ties
/// in distance by their coordinates, not by the order of the k-d tree, which depends on the
/// order of the input.
fn within(
    tree: &ImmutableKdTree<f32, 3>,
    xyz: &[[f32; 3]],
    p: &[f32; 3],
    radius: f64,
) -> Vec<(f32, u32)> {
    let r2 = (radius * radius) as f32;
    let mut nn: Vec<(f32, u32)> = tree
        .query(p)
        .within::<SquaredEuclidean<f32>>(r2)
        .execute()
        .iter()
        .map(|x| (x.distance, x.item))
        .collect();
    nn.sort_unstable_by(|a, b| {
        a.0.total_cmp(&b.0).then_with(|| {
            let bits = |i: u32| xyz[i as usize].map(f32::to_bits);
            bits(a.1).cmp(&bits(b.1))
        })
    });
    nn
}

/// Distance (m) of `p` to the plane of the points of `xyz` within `radius`; `None` with too few
/// points or if they do not lie on a plane.
pub(super) fn distance_to_plane(
    tree: &ImmutableKdTree<f32, 3>,
    xyz: &[[f32; 3]],
    p: &[f32; 3],
    radius: f64,
) -> Option<f64> {
    plane_shift(tree, xyz, p, radius, false).map(|s| s.norm())
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{noise, open_wall};
    use super::*;

    fn tree(xyz: &[[f32; 3]]) -> ImmutableKdTree<f32, 3> {
        ImmutableKdTree::new_from_slice(xyz).unwrap()
    }

    fn cfg(voxel: f64) -> SurfaceConfig {
        SurfaceConfig {
            voxel,
            ..SurfaceConfig::default()
        }
    }

    /// Points on the plane `x = x0`, a grid of `step` (m) over `y, z` in `[lo, hi]`.
    fn plane_points(x0: f32, lo: f32, hi: f32, step: f32) -> Vec<[f32; 3]> {
        let n = ((hi - lo) / step).round() as i32;
        let mut xyz = Vec::new();
        for iy in 0..=n {
            for iz in 0..=n {
                xyz.push([x0, lo + iy as f32 * step, lo + iz as f32 * step]);
            }
        }
        xyz
    }

    /// [`open_wall`] of `n` squares of 1 cm, every vertex at `x`.
    fn wall_at(n: usize, x: f32) -> Mesh {
        let mut mesh = open_wall(n, 0.01);
        for p in &mut mesh.positions {
            p[0] = x;
        }
        mesh
    }

    #[test]
    fn vertices_move_onto_the_plane() {
        let xyz = plane_points(0.0, -0.05, 0.13, 0.005);
        let mut mesh = wall_at(8, 0.004);
        let before = mesh.indices.clone();
        let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(0.01));
        assert_eq!(s.vertices, 81);
        assert_eq!(s.moved, 81, "{s:?}");
        assert_eq!(s.no_plane + s.too_far + s.reverted, 0, "{s:?}");
        assert!((s.mean_shift_mm - 4.0).abs() < 0.1, "{s:?}");
        assert!(mesh.positions.iter().all(|p| p[0].abs() < 1e-4));
        assert_eq!(mesh.indices, before);
    }

    #[test]
    fn far_vertex_stays() {
        let xyz = plane_points(0.0, -0.05, 0.13, 0.005);
        let mut mesh = wall_at(8, 0.002);
        // The middle vertex of the 9 × 9: 1.5 voxels off the plane.
        let far = 9 * 4 + 4;
        mesh.positions[far][0] = 0.015;
        let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(0.01));
        assert_eq!(s.too_far, 1, "{s:?}");
        assert_eq!(s.moved, 80, "{s:?}");
        assert_eq!(mesh.positions[far][0], 0.015);
    }

    #[test]
    fn too_few_points_stays() {
        let xyz = [
            [0.0f32, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, 0.0, 0.01],
            [0.0, 0.01, 0.01],
            [0.0, 0.02, 0.02],
        ];
        let mut mesh = wall_at(4, 0.003);
        let before = mesh.clone();
        let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(0.01));
        assert_eq!(s.vertices, 25);
        assert_eq!(s.no_plane, 25, "{s:?}");
        assert_eq!(s.moved, 0, "{s:?}");
        assert_eq!(mesh, before);
    }

    #[test]
    fn too_few_points_look_twice_as_far() {
        // A point every 2.5 cm: about 4 within the 3 cm of a 1 cm voxel, 18 within 6 cm.
        let xyz = plane_points(0.0, -0.1, 0.2, 0.025);
        let mut mesh = wall_at(4, 0.003);
        let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(0.01));
        assert_eq!(s.moved, 25, "{s:?}");
        assert!(mesh.positions.iter().all(|p| p[0].abs() < 1e-4));
    }

    #[test]
    fn thin_plate_stays() {
        // Two sheets of points 2 cm apart, a sheet of the mesh 1 mm inside each.
        for voxel in [0.01, 0.02] {
            let mut xyz = plane_points(0.0, -0.05, 0.13, 0.005);
            xyz.extend(plane_points(0.02, -0.05, 0.13, 0.005));
            let mut mesh = wall_at(8, 0.001);
            let back = wall_at(8, 0.019);
            let offset = mesh.positions.len() as u32;
            mesh.positions.extend(&back.positions);
            mesh.indices
                .extend(back.indices.iter().map(|t| t.map(|v| v + offset)));
            let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(voxel));
            println!("voxel {voxel}: {s:?}");
            let (front, rear) = mesh.positions.split_at(offset as usize);
            assert!(front.iter().all(|p| p[0].abs() < 0.0015), "{voxel}");
            assert!(rear.iter().all(|p| (p[0] - 0.02).abs() < 0.0015), "{voxel}");
        }
    }

    #[test]
    fn flipped_face_reverted() {
        // Points on x = 0. The first face stands almost upright on them (normal about +y) and
        // would lie flat after the move: it keeps its vertices. The second face shares the
        // vertex `a` and lies parallel to the plane; with `a` back where it was, its other two
        // vertices alone would tip it over, so they stay as well.
        let xyz = plane_points(0.0, -0.05, 0.05, 0.005);
        let mut mesh = Mesh {
            positions: vec![
                [0.008, 0.0, 0.0],
                [-0.008, 0.002, 0.0],
                [0.001, 0.001, 0.01],
                [0.008, 0.004, 0.0],
                [0.008, 0.0, 0.004],
            ],
            indices: vec![[0, 1, 2], [0, 3, 4]],
        };
        let before = mesh.clone();
        let s = snap_to_points(&mut mesh, &tree(&xyz), &xyz, &cfg(0.01));
        assert_eq!(s.reverted, 5, "{s:?}");
        assert_eq!(s.moved, 0, "{s:?}");
        assert_eq!(s.mean_shift_mm, 0.0);
        assert_eq!(mesh, before);
    }

    #[test]
    fn snap_is_deterministic() {
        let mut xyz = plane_points(0.0, -0.05, 0.25, 0.006);
        for (i, p) in xyz.iter_mut().enumerate() {
            p[0] += (0.002 * noise(i as u64)) as f32;
        }
        let run = |points: &[[f32; 3]]| {
            let mut mesh = wall_at(20, 0.001);
            let s = snap_to_points(&mut mesh, &tree(points), points, &cfg(0.01));
            (mesh, s)
        };
        let (mesh, stats) = run(&xyz);
        assert!(stats.moved > 0, "{stats:?}");
        let reversed: Vec<[f32; 3]> = xyz.iter().rev().copied().collect();
        let mut rotated = xyz.clone();
        rotated.rotate_left(xyz.len() / 3);
        for other in [&reversed, &rotated] {
            let (m, s) = run(other);
            assert_eq!(m, mesh);
            assert_eq!(s, stats);
        }
    }
}
