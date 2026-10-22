//! Synthetic scans shared by the surface tests.

use super::SurfaceInput;

/// An owned [`SurfaceInput`].
pub(crate) struct Scan {
    pub xyz: Vec<[f32; 3]>,
    pub sensor_of_point: Vec<u32>,
    pub sensors: Vec<[f32; 3]>,
}

impl Scan {
    pub fn input(&self) -> SurfaceInput<'_> {
        SurfaceInput {
            xyz: &self.xyz,
            sensor_of_point: &self.sensor_of_point,
            sensors: &self.sensors,
        }
    }
}

/// Rays from `origin` to the plane `x = x0`, hitting it on a regular grid of `step` (m)
/// over `|y − origin.y|, |z − origin.z| ≤ half`.
pub(crate) fn plane_x(origin: [f32; 3], x0: f32, half: f32, step: f32) -> Scan {
    let n = (half / step).round() as i32;
    let mut xyz = Vec::new();
    for iy in -n..=n {
        for iz in -n..=n {
            xyz.push([
                x0,
                origin[1] + iy as f32 * step,
                origin[2] + iz as f32 * step,
            ]);
        }
    }
    Scan {
        sensor_of_point: vec![0; xyz.len()],
        xyz,
        sensors: vec![origin],
    }
}

/// A closed box room `[-hx, hx] × [-hy, hy] × [-hz, hz]` around `origin` (the sensor), every
/// wall sampled on a regular grid of `step` (m).
pub(crate) fn box_room(origin: [f32; 3], half: [f32; 3], step: f32) -> Scan {
    let mut xyz = Vec::new();
    let count = |h: f32| (h / step).round() as i32;
    for axis in 0..3 {
        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
        for sign in [-1.0f32, 1.0] {
            for ia in -count(half[a])..=count(half[a]) {
                for ib in -count(half[b])..=count(half[b]) {
                    let mut p = origin;
                    p[axis] += sign * half[axis];
                    p[a] += ia as f32 * step;
                    p[b] += ib as f32 * step;
                    xyz.push(p);
                }
            }
        }
    }
    Scan {
        sensor_of_point: vec![0; xyz.len()],
        xyz,
        sensors: vec![origin],
    }
}

// ---- Mesh fixtures (built directly, no TSDF) -------------------------------------------

use super::Mesh;
use std::collections::{BTreeMap, HashMap};

/// A closed slab `[0, depth] × [0, n·step]²` whose front face (`x = depth`, normals `+x`) is a
/// grid of `step` (m) squares. The squares `hole.0 .. hole.0 + hole.1` in both directions are
/// left out of the front face, which makes one hole of `4 · hole.1` boundary edges. Closed on
/// all other sides, so the hole is the only boundary loop.
pub(crate) fn wall_with_hole(n: usize, step: f32, hole: Option<(usize, usize)>) -> Mesh {
    let depth = 2usize;
    let size = [depth, n, n];
    let mut mesh = Mesh::default();
    let mut vertex: HashMap<[usize; 3], u32> = HashMap::new();
    let mut vid = |mesh: &mut Mesh, p: [usize; 3]| -> u32 {
        *vertex.entry(p).or_insert_with(|| {
            mesh.positions
                .push([p[0] as f32 * step, p[1] as f32 * step, p[2] as f32 * step]);
            (mesh.positions.len() - 1) as u32
        })
    };
    for axis in 0..3 {
        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
        for high in [false, true] {
            for ia in 0..size[a] {
                for ib in 0..size[b] {
                    if axis == 0 && high {
                        if let Some((h0, hn)) = hole {
                            if (h0..h0 + hn).contains(&ia) && (h0..h0 + hn).contains(&ib) {
                                continue;
                            }
                        }
                    }
                    let corner = |da: usize, db: usize| {
                        let mut p = [0usize; 3];
                        p[axis] = if high { size[axis] } else { 0 };
                        p[a] = ia + da;
                        p[b] = ib + db;
                        p
                    };
                    let q = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)]
                        .map(|p| vid(&mut mesh, p));
                    for tri in [[q[0], q[1], q[3]], [q[1], q[2], q[3]]] {
                        let f = mesh.indices.len();
                        mesh.indices.push(tri);
                        // Outward: +axis on the high side, -axis on the low side.
                        let want = if high { 1.0 } else { -1.0 };
                        if mesh.face_normal(f)[axis] * want < 0.0 {
                            mesh.indices[f] = [tri[0], tri[2], tri[1]];
                        }
                    }
                }
            }
        }
    }
    mesh
}

/// An open flat wall (`x = 0`, normals `+x`) of `n × n` squares of `step` (m) in the y-z plane.
pub(crate) fn open_wall(n: usize, step: f32) -> Mesh {
    let mut mesh = Mesh::default();
    for iy in 0..=n {
        for iz in 0..=n {
            mesh.positions
                .push([0.0, iy as f32 * step, iz as f32 * step]);
        }
    }
    let id = |iy: usize, iz: usize| (iy * (n + 1) + iz) as u32;
    for iy in 0..n {
        for iz in 0..n {
            mesh.indices
                .push([id(iy, iz), id(iy + 1, iz), id(iy, iz + 1)]);
            mesh.indices
                .push([id(iy + 1, iz), id(iy + 1, iz + 1), id(iy, iz + 1)]);
        }
    }
    mesh
}

/// Index of the vertex at `p` (fixture helper; linear search).
fn vertex_at(mesh: &Mesh, p: [f32; 3]) -> u32 {
    mesh.positions
        .iter()
        .position(|q| (0..3).all(|k| (q[k] - p[k]).abs() < 1e-6))
        .expect("Eckpunkt nicht im Netz") as u32
}

/// An `n × n` open wall with a thin rod growing out of it: a square tube of 2 × 2 `step`
/// squares and `len` `step`-long segments, closed by a cap. The tube is stitched into a 2 × 2
/// square hole of the wall with shared vertices, so wall and rod are connected by edges.
/// Returns the mesh and the number of rod faces (they are the last ones).
pub(crate) fn rod_on_wall(n: usize, step: f32, len: usize) -> (Mesh, usize) {
    let c = n / 2;
    let mut mesh = open_wall(n, step);
    // Remove the four wall squares (8 faces) that the rod replaces.
    let removed: Vec<[u32; 3]> = {
        let id = |iy: usize, iz: usize| (iy * (n + 1) + iz) as u32;
        let mut gone = Vec::new();
        for iy in c..c + 2 {
            for iz in c..c + 2 {
                gone.push([id(iy, iz), id(iy + 1, iz), id(iy, iz + 1)]);
                gone.push([id(iy + 1, iz), id(iy + 1, iz + 1), id(iy, iz + 1)]);
            }
        }
        gone
    };
    mesh.indices.retain(|f| !removed.contains(f));
    let wall_faces = mesh.indices.len();
    // Ring around the hole, counter-clockwise seen from +x (y towards z).
    let ring_yz: Vec<(usize, usize)> = vec![
        (c, c),
        (c + 1, c),
        (c + 2, c),
        (c + 2, c + 1),
        (c + 2, c + 2),
        (c + 1, c + 2),
        (c, c + 2),
        (c, c + 1),
    ];
    let mut rings: Vec<Vec<u32>> = vec![ring_yz
        .iter()
        .map(|&(y, z)| vertex_at(&mesh, [0.0, y as f32 * step, z as f32 * step]))
        .collect()];
    for level in 1..=len {
        let x = level as f32 * step;
        let ring = ring_yz
            .iter()
            .map(|&(y, z)| {
                mesh.positions.push([x, y as f32 * step, z as f32 * step]);
                (mesh.positions.len() - 1) as u32
            })
            .collect();
        rings.push(ring);
    }
    for w in rings.windows(2) {
        let k = w[0].len();
        for i in 0..k {
            let j = (i + 1) % k;
            mesh.indices.push([w[0][i], w[0][j], w[1][i]]);
            mesh.indices.push([w[0][j], w[1][j], w[1][i]]);
        }
    }
    let top = rings.last().unwrap().clone();
    let mid = [
        len as f32 * step,
        (c + 1) as f32 * step,
        (c + 1) as f32 * step,
    ];
    mesh.positions.push(mid);
    let m = (mesh.positions.len() - 1) as u32;
    for i in 0..top.len() {
        mesh.indices.push([top[i], top[(i + 1) % top.len()], m]);
    }
    let rod = mesh.indices.len() - wall_faces;
    (mesh, rod)
}

/// A closed icosahedron (20 faces, outward normals) of radius `r` around `centre`.
pub(crate) fn speck(centre: [f32; 3], r: f32) -> Mesh {
    let t = (1.0 + 5f32.sqrt()) / 2.0;
    let raw = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let scale = r / (1.0 + t * t).sqrt();
    let positions = raw
        .iter()
        .map(|p| [0, 1, 2].map(|k| centre[k] + p[k] * scale))
        .collect();
    let indices = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    Mesh { positions, indices }
}

/// Appends `other` to `mesh` (indices shifted).
pub(crate) fn append(mesh: &mut Mesh, other: &Mesh) {
    let base = mesh.positions.len() as u32;
    mesh.positions.extend_from_slice(&other.positions);
    mesh.indices
        .extend(other.indices.iter().map(|f| f.map(|i| i + base)));
}

/// Directed edges used more than once, and boundary edges (no opposite edge).
/// A consistently wound mesh has no repeated directed edge.
pub(crate) fn edge_report(mesh: &Mesh) -> (usize, usize) {
    let mut count: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for f in &mesh.indices {
        for k in 0..3 {
            *count.entry((f[k], f[(k + 1) % 3])).or_default() += 1;
        }
    }
    let repeated = count.values().filter(|&&c| c > 1).count();
    let boundary = count
        .keys()
        .filter(|&&(a, b)| !count.contains_key(&(b, a)))
        .count();
    (repeated, boundary)
}
