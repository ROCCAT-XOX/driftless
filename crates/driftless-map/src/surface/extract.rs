//! Marching cubes over the observed cells of a [`Tsdf`].
//!
//! A cell is the cube between the centres of 8 neighbouring voxels, named by its lowest voxel.
//! It yields triangles only if all 8 corners are observed: a corner without measurements would
//! otherwise get a made-up sign and produce faces on the back of the band. Vertices sit on
//! cell edges and are shared by edge id, so neighbouring cells join without cracks. Blocks are
//! visited in sorted order, which makes the output deterministic.

use super::mc_tables::{CORNERS, EDGE_CORNERS, TRIANGLES};
use super::{SurfaceConfig, Tsdf};
use std::collections::HashMap;

/// Voxels per block edge (see [`Tsdf::blocks`]).
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

/// The mesh of the zero crossing of `tsdf`, normals towards the side with positive sdf (the
/// sensor side). `Tsdf::sample` already applies `min_weight`, so `cfg` is not read here.
pub fn extract(tsdf: &Tsdf, cfg: &SurfaceConfig) -> Mesh {
    let _ = cfg;
    let voxel = tsdf.voxel();
    let mut mesh = Mesh::default();
    // (lower voxel of the edge, axis) -> vertex index
    let mut vertex_of: HashMap<([i32; 3], u8), u32> = HashMap::new();
    for block in tsdf.blocks() {
        for lz in 0..BLOCK {
            for ly in 0..BLOCK {
                for lx in 0..BLOCK {
                    let cell = [
                        block[0] * BLOCK + lx,
                        block[1] * BLOCK + ly,
                        block[2] * BLOCK + lz,
                    ];
                    cell_triangles(tsdf, voxel, cell, &mut vertex_of, &mut mesh);
                }
            }
        }
    }
    mesh
}

fn cell_triangles(
    tsdf: &Tsdf,
    voxel: f64,
    cell: [i32; 3],
    vertex_of: &mut HashMap<([i32; 3], u8), u32>,
    mesh: &mut Mesh,
) {
    let mut sdf = [0.0f32; 8];
    let mut case = 0usize;
    for (i, off) in CORNERS.iter().enumerate() {
        let key = [cell[0] + off[0], cell[1] + off[1], cell[2] + off[2]];
        match tsdf.sample(key) {
            Some((s, _)) => sdf[i] = s,
            None => return,
        }
        if sdf[i] < 0.0 {
            case |= 1 << i;
        }
    }
    let tris = &TRIANGLES[case];
    if tris[0] < 0 {
        return;
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
    use super::super::testutil::{box_room, plane_x};
    use super::*;
    use driftless_io::progress::Progress;
    use std::collections::HashMap;

    fn mesh_of(scan: &super::super::testutil::Scan) -> Mesh {
        let cfg = SurfaceConfig::default();
        let t = Tsdf::integrate(&scan.input(), &cfg, &Progress::new()).unwrap();
        extract(&t, &cfg)
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
    fn extract_deterministic() {
        let scan = plane_x([0.0; 3], 2.0, 0.3, 0.004);
        assert_eq!(mesh_of(&scan), mesh_of(&scan));
    }
}
