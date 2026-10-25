//! Marching cubes over the observed cells of a [`FusedSdf`].
//!
//! A cell is the cube between the centres of 8 neighbouring voxels, named by its lowest voxel.
//! It yields triangles only if all 8 corners are observed: a corner without measurements would
//! otherwise get a made-up sign and produce faces on the back of the band. A corner's value is
//! a pure function of its key (fine or, where the fine voxel is weak, coarse), so neighbouring
//! cells agree on it. Vertices sit on cell edges and are shared by edge id, so neighbouring
//! cells join without cracks. Blocks are visited in sorted order, which makes the output
//! deterministic.

use super::mc_tables::{CORNERS, EDGE_CORNERS, TRIANGLES};
use super::FusedSdf;
use std::collections::HashMap;

/// Voxels per block edge (see [`FusedSdf::blocks`]).
const BLOCK: i32 = 8;

/// An indexed triangle mesh; vertices are shared between neighbouring triangles.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<[u32; 3]>,
}

impl Mesh {
    /// Cross product of two edges of face `f` (twice the area, normal direction).
    fn cross(&self, f: usize) -> [f32; 3] {
        let [a, b, c] = self.indices[f].map(|i| self.positions[i as usize]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ]
    }

    /// Unit normal of face `f` (counter-clockwise winding); zero for a degenerate face.
    pub fn face_normal(&self, f: usize) -> [f32; 3] {
        let n = self.cross(f);
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 0.0 {
            n.map(|x| x / len)
        } else {
            [0.0; 3]
        }
    }

    /// Area of face `f` (m²).
    pub fn face_area(&self, f: usize) -> f32 {
        let n = self.cross(f);
        0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
    }
}

/// The mesh of the zero crossing of `sdf`, normals towards the side with positive sdf (the
/// sensor side), and the number of cells with faces that took a corner from the coarse grid.
/// [`FusedSdf::sample`] already applies `min_weight`.
pub fn extract(sdf: &FusedSdf) -> (Mesh, usize) {
    let voxel = sdf.voxel();
    let mut mesh = Mesh::default();
    let mut from_coarse = 0;
    // (lower voxel of the edge, axis) -> vertex index
    let mut vertex_of: HashMap<([i32; 3], u8), u32> = HashMap::new();
    // The voxels the cells of a block touch (the block and one more layer above), each
    // sampled once: a coarse value costs 8 lookups and every voxel is a corner of 8 cells.
    const SIDE: i32 = BLOCK + 1;
    let at = |l: [i32; 3]| (l[0] + SIDE * (l[1] + SIDE * l[2])) as usize;
    let mut corners: Vec<Option<(f32, bool)>> = vec![None; (SIDE * SIDE * SIDE) as usize];
    for block in sdf.blocks() {
        let origin = block.map(|b| b * BLOCK);
        for lz in 0..SIDE {
            for ly in 0..SIDE {
                for lx in 0..SIDE {
                    let key = [origin[0] + lx, origin[1] + ly, origin[2] + lz];
                    corners[at([lx, ly, lz])] = sdf.sample_tagged(key);
                }
            }
        }
        for lz in 0..BLOCK {
            for ly in 0..BLOCK {
                for lx in 0..BLOCK {
                    let values = CORNERS.map(|o| corners[at([lx + o[0], ly + o[1], lz + o[2]])]);
                    let cell = [origin[0] + lx, origin[1] + ly, origin[2] + lz];
                    if cell_triangles(values, voxel, cell, &mut vertex_of, &mut mesh) {
                        from_coarse += 1;
                    }
                }
            }
        }
    }
    (mesh, from_coarse)
}

/// Adds the faces of `cell` from the values of its corners (in [`CORNERS`] order; `None` =
/// unobserved, and the cell has no faces); true if it has faces and a corner from the coarse
/// grid.
fn cell_triangles(
    values: [Option<(f32, bool)>; 8],
    voxel: f64,
    cell: [i32; 3],
    vertex_of: &mut HashMap<([i32; 3], u8), u32>,
    mesh: &mut Mesh,
) -> bool {
    let mut sdf = [0.0f32; 8];
    let mut case = 0usize;
    let mut coarse = false;
    for (i, v) in values.into_iter().enumerate() {
        let Some((s, c)) = v else {
            return false;
        };
        sdf[i] = s;
        coarse |= c;
        if s < 0.0 {
            case |= 1 << i;
        }
    }
    let tris = &TRIANGLES[case];
    if tris[0] < 0 {
        return false;
    }
    let mut edge_vertex = [u32::MAX; 12];
    for t in tris.chunks(3).take_while(|t| t[0] >= 0) {
        let mut face = [0u32; 3];
        for (slot, &e) in face.iter_mut().zip(t) {
            let e = e as usize;
            if edge_vertex[e] == u32::MAX {
                edge_vertex[e] = edge_point(cell, voxel, e, &sdf, vertex_of, mesh);
            }
            *slot = edge_vertex[e];
        }
        mesh.indices.push(face);
    }
    coarse
}

/// The vertex on edge `e` of `cell`, created on first use.
fn edge_point(
    cell: [i32; 3],
    voxel: f64,
    e: usize,
    sdf: &[f32; 8],
    vertex_of: &mut HashMap<([i32; 3], u8), u32>,
    mesh: &mut Mesh,
) -> u32 {
    let [a, b] = EDGE_CORNERS[e];
    let (ca, cb) = (CORNERS[a], CORNERS[b]);
    // Edges join corners that differ in exactly one axis; the id is the lower corner and axis.
    let axis = (0..3).position(|i| ca[i] != cb[i]).unwrap_or(0);
    let lower = if ca[axis] < cb[axis] { ca } else { cb };
    let id = (
        [cell[0] + lower[0], cell[1] + lower[1], cell[2] + lower[2]],
        axis as u8,
    );
    *vertex_of.entry(id).or_insert_with(|| {
        // Both ends have opposite signs, so the difference is not zero.
        let t = f64::from(sdf[a]) / (f64::from(sdf[a]) - f64::from(sdf[b]));
        let p = [0, 1, 2].map(|i| {
            let at = f64::from(cell[i] + ca[i]) + 0.5;
            let along = f64::from(cb[i] - ca[i]);
            ((at + t * along) * voxel) as f32
        });
        debug_assert!(
            mesh.positions.len() < u32::MAX as usize,
            "too many vertices"
        );
        mesh.positions.push(p);
        (mesh.positions.len() - 1) as u32
    })
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{box_room, plane_x, wall_x};
    use super::super::SurfaceConfig;
    use super::*;
    use driftless_io::progress::Progress;
    use std::collections::HashMap;

    fn mesh_of(scan: &super::super::testutil::Scan) -> Mesh {
        let cfg = SurfaceConfig::default();
        let t = FusedSdf::integrate(&scan.input(), &cfg, &Progress::new()).unwrap();
        extract(&t).0
    }

    #[test]
    fn box_room_is_closed_inside() {
        let scan = box_room([0.0; 3], [1.5, 1.5, 1.25], 0.004);
        let m = mesh_of(&scan);
        assert!(m.indices.len() > 10_000);
        let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
        for f in &m.indices {
            for i in 0..3 {
                let (a, b) = (f[i], f[(i + 1) % 3]);
                *edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let bad = edges.values().filter(|&&n| n != 2).count();
        assert_eq!(
            bad,
            0,
            "{bad} of {} edges are not shared by 2 faces",
            edges.len()
        );
        let mut outward = 0;
        for (i, f) in m.indices.iter().enumerate() {
            let n = m.face_normal(i);
            let c = f.map(|v| m.positions[v as usize]);
            let cen = [0, 1, 2].map(|a| (c[0][a] + c[1][a] + c[2][a]) / 3.0);
            // Zero-area faces (a wall exactly on the voxel centres) have no direction;
            // cleanup drops them.
            if n != [0.0; 3] && n[0] * cen[0] + n[1] * cen[1] + n[2] * cen[2] >= 0.0 {
                outward += 1;
            }
        }
        assert_eq!(outward, 0, "{outward} faces point away from the sensor");
    }

    #[test]
    fn back_of_band_has_no_faces() {
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.004);
        let m = mesh_of(&scan);
        assert!(!m.indices.is_empty());
        for p in &m.positions {
            assert!((p[0] - 2.0).abs() <= 0.01 + 1e-4, "vertex at x = {}", p[0]);
        }
    }

    #[test]
    fn fallback_mesh_has_no_cracks() {
        let cfg = SurfaceConfig {
            voxel: 0.01,
            ..SurfaceConfig::default()
        };
        let half = 0.5f32;
        // The sparse wall of `sparse_wall_closes_with_coarse_fallback` (a point every 7 mm),
        // and a wall of one point per 1 cm voxel column (all from the coarse grid) in 10 cm
        // stripes with 4, where fine and coarse corners meet.
        for (step, stripes) in [(0.007, false), (0.01, true)] {
            let dense = |y: f64, _| stripes && (y / 0.1).floor().rem_euclid(2.0) == 0.0;
            let scan = wall_x(2.003, f64::from(half), step, dense);
            let sdf = FusedSdf::integrate(&scan.input(), &cfg, &Progress::new()).unwrap();
            let (m, from_coarse) = extract(&sdf);
            assert!(from_coarse > 0, "stripes {stripes}");
            let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
            for f in &m.indices {
                for i in 0..3 {
                    let (a, b) = (f[i], f[(i + 1) % 3]);
                    *edges.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
            let inside = |v: u32| {
                let p = m.positions[v as usize];
                p[1].abs() < half - 0.05 && p[2].abs() < half - 0.05
            };
            let interior: Vec<u32> = edges
                .iter()
                .filter(|((a, b), _)| inside(*a) && inside(*b))
                .map(|(_, &n)| n)
                .collect();
            assert!(interior.len() > 10_000, "{}", interior.len());
            let bad = interior.iter().filter(|&&n| n != 2).count();
            assert_eq!(
                bad, 0,
                "stripes {stripes}: {bad} interior edges not shared by 2 faces"
            );
            for p in &m.positions {
                assert!((p[0] - 2.003).abs() < 1e-3, "vertex at x = {}", p[0]);
            }
        }
    }

    #[test]
    fn extract_deterministic() {
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.004);
        assert_eq!(mesh_of(&scan), mesh_of(&scan));
    }
}
