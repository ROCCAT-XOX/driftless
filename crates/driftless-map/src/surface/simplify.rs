//! Triangle reduction: flat walls become a few big triangles, the shape stays within a tight error.
//! Then edges longer than [`texture_edge`] (a few voxels, at most [`SurfaceConfig::max_edge`])
//! are split, so that every face fits into a camera image and is small enough for the texture's
//! seam levelling, and slivers on flat parts are turned into rounder faces by
//! edge flips ([`improve_triangles`]). Finally every vertex of the input is checked against the
//! result ([`far_vertices`]); those too far away are locked and the reduction runs again.

use super::{Mesh, SurfaceConfig};
use meshopt::{SimplifyOptions, VertexDataAdapter};
use rayon::prelude::*;
use std::collections::HashMap;

/// Rounds of edge splitting at most; each round halves every long edge, so 40 rounds reach
/// from kilometres to below a millimetre.
const MAX_SPLIT_ROUNDS: usize = 40;

/// Reductions after the first at most; each locks the vertices of the input the one before
/// left further than [`SurfaceConfig::simplify_error`] from the result.
const MAX_LOCK_ROUNDS: usize = 4;

/// Edges of the textured mesh are at most this many voxels long, but may always reach
/// [`TEXTURE_EDGE_MIN`], and never more than [`SurfaceConfig::max_edge`] (the bound for coarse
/// voxels); see [`texture_edge`]. The seam levelling has one gain per vertex and the labels are
/// per face, so large flat triangles (a merged model's floor, fused from several scans and
/// simplified further than a single scan's) cannot follow the brightness that changes from
/// image to image and show polygonal blotches.
///
/// Black room model (three scans), with 16 views per face and the gray fill: at 2 cm
/// `seam_step` 21.72 → 15.64 with 3 voxels (6 cm), 17.73 with 4 (8 cm); `.glb` 35 → 50 MB
/// (42 with 4). At 1 cm 23.31 → 19.24 with 4 cm, 17.14 with 3 cm, but `.glb` 83 → 104 MB
/// against 133 MB and simplification 123 against 194 s; 4 cm already beats the single scan
/// as it was (20.64).
pub const TEXTURE_EDGE_VOXELS: f64 = 3.0;

/// Texture edges may always be this long (m), however fine the voxel (see
/// [`TEXTURE_EDGE_VOXELS`]).
pub const TEXTURE_EDGE_MIN: f64 = 0.04;

/// The longest edge of the textured mesh for `cfg` (m): [`TEXTURE_EDGE_VOXELS`] voxels, at
/// least [`TEXTURE_EDGE_MIN`], at most [`SurfaceConfig::max_edge`].
pub fn texture_edge(cfg: &SurfaceConfig) -> f64 {
    cfg.max_edge
        .min((TEXTURE_EDGE_VOXELS * cfg.voxel).max(TEXTURE_EDGE_MIN))
}

/// Reduces the triangles of `mesh` with meshoptimizer.
///
/// The vertices of `mesh` stay within `cfg.simplify_error` (m) of the result (see below), and the
/// vertices on the border of the mesh stay where they are, so open edges keep their length.
/// Vertices no face uses after the reduction are dropped; the order of the rest is kept. Every
/// edge longer than [`texture_edge`] is split at its midpoint (see [`split_long_edges`]), slivers are flipped (see [`improve_triangles`]).
///
/// meshoptimizer's error limit alone does not give that bound: its error is a weighted mean of
/// the squared distances to the planes merged into a vertex, so where a thin part (a rod, a
/// table leg) meets a large flat one, the many flat planes outweigh the few planes of the thin
/// part, and a corner of it may collapse into the flat part although that moves the surface by
/// the whole width of the thin part. Which collapse wins such a near tie depends on float
/// rounding, i.e. on the compiler and the platform (FMA on arm64, none with MSVC). So the
/// result is checked: the vertices of `mesh` too far from it are locked, and the reduction
/// runs again from the start, at most [`MAX_LOCK_ROUNDS`] times. On a real 2-cm scan (1 M
/// vertices) the first reduction left 2.7 % of the vertices beyond the limit, 25 of them by
/// 1–5 cm; after the last round 0.1 % remain, most within 3 mm, none beyond 1 cm.
pub fn simplify(mesh: &Mesh, cfg: &SurfaceConfig) -> Mesh {
    if mesh.indices.is_empty() {
        return Mesh::default();
    }
    let flat: Vec<u32> = mesh.indices.iter().flatten().copied().collect();
    let bytes: &[u8] = positions_as_bytes(&mesh.positions);
    let adapter = VertexDataAdapter::new(bytes, 12, 0).expect("Eckpunkte sind drei f32 je Punkt");
    let mut lock = vec![false; mesh.positions.len()];
    let mut round = 0;
    let max_edge = texture_edge(cfg);
    loop {
        // Target count 0: reduce as far as the error allows. The error is absolute (m), which
        // is the same as the relative error of the brief scaled by the extent of the mesh.
        let reduced = meshopt::simplify_with_locks(
            &flat,
            &adapter,
            &lock,
            0,
            cfg.simplify_error as f32,
            SimplifyOptions::LockBorder | SimplifyOptions::ErrorAbsolute,
            None,
        );
        let mut out = Mesh {
            positions: mesh.positions.clone(),
            indices: reduced.as_chunks::<3>().0.to_vec(),
        };
        drop_unused(&mut out);
        split_long_edges(&mut out, max_edge);
        improve_triangles(&mut out, MAX_FLIP_DEVIATION * cfg.simplify_error, max_edge);
        if round == MAX_LOCK_ROUNDS {
            return out;
        }
        let far = far_vertices(mesh, &out, cfg.simplify_error);
        if far.is_empty() {
            return out;
        }
        for v in far {
            lock[v as usize] = true;
        }
        round += 1;
    }
}

/// The vertices of `from` (those a face uses) further than `max` (m) from every face of `to`,
/// in ascending order. A grid of cells finds the faces near a vertex: each face is listed in
/// every cell its bounding box, grown by `max`, touches.
///
/// The cells are as large as the mean edge of `to` (at least `2 · max`), and the grid is one
/// sorted list of (cell, face), 16 bytes per entry and a few entries per face. Cells of an
/// eighth of the longest edge in a hash map of lists were tiny once the edges were split to
/// 4 cm (5-mm cells, dozens per face, each list its own allocation): on the merged black room
/// at 1 cm (2.9 M faces) the process grew to 6.7 GB in this function, now to 1.9 GB.
pub(crate) fn far_vertices(from: &Mesh, to: &Mesh, max: f64) -> Vec<u32> {
    let up = |p: [f32; 3]| p.map(f64::from);
    let tri = |f: usize| to.indices[f].map(|i| up(to.positions[i as usize]));
    let used = || {
        let mut used = vec![false; from.positions.len()];
        for &v in from.indices.iter().flatten() {
            used[v as usize] = true;
        }
        used
    };
    if to.indices.is_empty() {
        let used = used();
        return (0..from.positions.len() as u32)
            .filter(|&v| used[v as usize])
            .collect();
    }
    let length: f64 = (0..to.indices.len())
        .into_par_iter()
        .map(|f| {
            let t = tri(f);
            (0..3)
                .map(|k| dot(sub(t[k], t[(k + 1) % 3]), sub(t[k], t[(k + 1) % 3])).sqrt())
                .sum::<f64>()
        })
        .sum();
    let mean_edge = length / (3 * to.indices.len()) as f64;
    let (lo_all, hi_all) = to.positions.iter().fold(
        ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
        |(lo, hi), p| {
            (
                [0, 1, 2].map(|k| lo[k].min(f64::from(p[k]))),
                [0, 1, 2].map(|k| hi[k].max(f64::from(p[k]))),
            )
        },
    );
    // Cell coordinates of 21 bits per axis, packed into one key; coarser cells if the mesh is
    // too large for that.
    const BITS: u32 = 21;
    let span = (0..3)
        .map(|k| hi_all[k] - lo_all[k] + 4.0 * max)
        .fold(0.0, f64::max);
    let cell = mean_edge
        .max(2.0 * max)
        .max(1e-3)
        .max(span / f64::from((1u32 << BITS) - 4));
    let origin = lo_all.map(|x| x - max - cell);
    let coord = |x: f64, k: usize| ((x - origin[k]) / cell).floor() as i64;
    let key = |c: [i64; 3]| -> Option<u64> {
        let limit = 1i64 << BITS;
        c.iter()
            .all(|&x| (0..limit).contains(&x))
            .then(|| (c[0] as u64) << (2 * BITS) | (c[1] as u64) << BITS | c[2] as u64)
    };
    let mut grid: Vec<(u64, u32)> = (0..to.indices.len())
        .into_par_iter()
        .flat_map_iter(|f| {
            let t = tri(f);
            let lo = [0, 1, 2].map(|k| coord(t[0][k].min(t[1][k]).min(t[2][k]) - max, k));
            let hi = [0, 1, 2].map(|k| coord(t[0][k].max(t[1][k]).max(t[2][k]) + max, k));
            (lo[0]..=hi[0]).flat_map(move |x| {
                (lo[1]..=hi[1]).flat_map(move |y| {
                    (lo[2]..=hi[2])
                        .map(move |z| (key([x, y, z]).expect("cell within the grid"), f as u32))
                })
            })
        })
        .collect();
    grid.par_sort_unstable();
    let used = used();
    let max_sq = max * max;
    (0..from.positions.len() as u32)
        .into_par_iter()
        .filter(|&v| {
            if !used[v as usize] {
                return false;
            }
            let p = up(from.positions[v as usize]);
            let Some(k) = key([0, 1, 2].map(|i| coord(p[i], i))) else {
                return true;
            };
            let first = grid.partition_point(|e| e.0 < k);
            let near = grid[first..]
                .iter()
                .take_while(|e| e.0 == k)
                .any(|&(_, f)| point_triangle_sq(p, tri(f as usize)) <= max_sq);
            !near
        })
        .collect()
}

/// Squared distance from `p` to the triangle `tri` (Ericson, Real-Time Collision Detection).
fn point_triangle_sq(p: [f64; 3], tri: [[f64; 3]; 3]) -> f64 {
    let [a, b, c] = tri;
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let closest = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let (d3, d4) = (dot(ab, bp), dot(ac, bp));
        let cp = sub(p, c);
        let (d5, d6) = (dot(ab, cp), dot(ac, cp));
        let (vc, vb, va) = (d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4);
        let at = |u: f64, v: f64| {
            [
                a[0] + ab[0] * u + ac[0] * v,
                a[1] + ab[1] * u + ac[1] * v,
                a[2] + ab[2] * u + ac[2] * v,
            ]
        };
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            at(d1 / (d1 - d3), 0.0)
        } else if d6 >= 0.0 && d5 <= d6 {
            c
        } else if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            at(0.0, d2 / (d2 - d6))
        } else if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            [
                b[0] + (c[0] - b[0]) * w,
                b[1] + (c[1] - b[1]) * w,
                b[2] + (c[2] - b[2]) * w,
            ]
        } else {
            let denom = 1.0 / (va + vb + vc);
            at(vb * denom, vc * denom)
        }
    };
    let d = sub(p, closest);
    dot(d, d)
}

/// A flip may move the surface by at most this share of [`SurfaceConfig::simplify_error`].
const MAX_FLIP_DEVIATION: f64 = 0.5;
/// Two faces are flipped only if their normals differ by less than this (rad, 20°).
const MAX_FLIP_BEND: f64 = 0.349;
/// Rounds of flipping at most.
const MAX_FLIP_ROUNDS: usize = 8;
/// A flip must raise the smallest angle of the pair by more than this (rad).
const MIN_FLIP_GAIN: f64 = 1e-3;

/// Smallest angle (rad) of the triangle `t`; zero if it is degenerate.
pub(crate) fn min_angle(t: [[f64; 3]; 3]) -> f64 {
    let angle = |o: [f64; 3], a: [f64; 3], b: [f64; 3]| {
        let u = sub(a, o);
        let v = sub(b, o);
        let c = cross(u, v);
        dot(c, c).sqrt().atan2(dot(u, v))
    };
    angle(t[0], t[1], t[2])
        .min(angle(t[1], t[2], t[0]))
        .min(angle(t[2], t[0], t[1]))
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Replaces the shared edge of two neighbouring faces by the other diagonal of their quad
/// wherever that raises the smallest angle of the pair: slivers left by the reduction (thin
/// faces that later take their colour from another image than their neighbours) become
/// rounder. Only nearly flat pairs are flipped (normals within [`MAX_FLIP_BEND`], the fourth
/// corner within `max_dev` (m) of the plane of the other three), the new edge is at most
/// `max_edge` long and must not exist yet, and both new faces keep the winding. The vertices,
/// the boundary and the orientation stay as they were. Faces are visited in order, so the
/// result is deterministic. Returns the number of flips.
pub(crate) fn improve_triangles(mesh: &mut Mesh, max_dev: f64, max_edge: f64) -> usize {
    let mut face_of: HashMap<(u32, u32), u32> = HashMap::with_capacity(mesh.indices.len() * 3);
    // Directed edges used by more than one face (non-manifold input): never flipped.
    let mut repeated: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    for (f, t) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            let e = (t[k], t[(k + 1) % 3]);
            if face_of.insert(e, f as u32).is_some() {
                repeated.insert(e);
            }
        }
    }
    let touches_repeated = |t: [u32; 3]| {
        (0..3).any(|k| {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            repeated.contains(&(a, b)) || repeated.contains(&(b, a))
        })
    };
    let at = |m: &Mesh, v: u32| m.positions[v as usize].map(f64::from);
    let unit = |n: [f64; 3]| {
        let l = dot(n, n).sqrt();
        (l > 0.0).then(|| n.map(|x| x / l))
    };
    let mut flips = 0;
    for _ in 0..MAX_FLIP_ROUNDS {
        let mut round = 0;
        for f in 0..mesh.indices.len() {
            for k in 0..3 {
                let t = mesh.indices[f];
                let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                let Some(&g) = face_of.get(&(b, a)) else {
                    continue;
                };
                let g = g as usize;
                if g == f
                    || (!repeated.is_empty()
                        && (touches_repeated(mesh.indices[f]) || touches_repeated(mesh.indices[g])))
                {
                    continue;
                }
                let Some(&d) = mesh.indices[g].iter().find(|&&v| v != a && v != b) else {
                    continue;
                };
                if d == c || face_of.contains_key(&(c, d)) || face_of.contains_key(&(d, c)) {
                    continue;
                }
                let (pa, pb, pc, pd) = (at(mesh, a), at(mesh, b), at(mesh, c), at(mesh, d));
                let (Some(nf), Some(ng)) = (
                    unit(cross(sub(pb, pa), sub(pc, pa))),
                    unit(cross(sub(pa, pb), sub(pd, pb))),
                ) else {
                    continue;
                };
                if dot(nf, ng) < MAX_FLIP_BEND.cos() || dot(sub(pd, pa), nf).abs() > max_dev {
                    continue;
                }
                let diag = sub(pd, pc);
                if dot(diag, diag).sqrt() > max_edge {
                    continue;
                }
                // New faces a d c and d b c (the quad runs a -> d -> b -> c).
                let (Some(n1), Some(n2)) = (
                    unit(cross(sub(pd, pa), sub(pc, pa))),
                    unit(cross(sub(pb, pd), sub(pc, pd))),
                ) else {
                    continue;
                };
                let mean = [0, 1, 2].map(|i| nf[i] + ng[i]);
                if dot(n1, mean) <= 0.0 || dot(n2, mean) <= 0.0 || dot(n1, n2) < MAX_FLIP_BEND.cos()
                {
                    continue;
                }
                let old = min_angle([pa, pb, pc]).min(min_angle([pb, pa, pd]));
                let new = min_angle([pa, pd, pc]).min(min_angle([pd, pb, pc]));
                if new <= old + MIN_FLIP_GAIN {
                    continue;
                }
                mesh.indices[f] = [a, d, c];
                mesh.indices[g] = [d, b, c];
                face_of.remove(&(a, b));
                face_of.remove(&(b, a));
                for (e, h) in [
                    ((a, d), f),
                    ((d, c), f),
                    ((c, a), f),
                    ((d, b), g),
                    ((b, c), g),
                    ((c, d), g),
                ] {
                    face_of.insert(e, h as u32);
                }
                round += 1;
                break; // face f changed: its other edges are looked at in the next round
            }
        }
        flips += round;
        if round == 0 {
            break;
        }
    }
    flips
}

/// Splits every edge longer than `max_edge` (m) until none is left. Per round every long edge
/// gets one midpoint, shared by all faces on it, and each face is cut by the template for its
/// number of split edges (1: two faces through the opposite corner; 2: the corner triangle and
/// the rest cut along its shorter diagonal; 3: four faces). Faces keep their winding, the
/// midpoints lie on the old edges, so the surface, its boundary and its orientation stay as
/// they were. New vertices are appended in face order: deterministic.
pub(crate) fn split_long_edges(mesh: &mut Mesh, max_edge: f64) {
    let key = |a: u32, b: u32| (a.min(b), a.max(b));
    for _ in 0..MAX_SPLIT_ROUNDS {
        let len = |m: &Mesh, a: u32, b: u32| {
            let (p, q) = (m.positions[a as usize], m.positions[b as usize]);
            (0..3)
                .map(|k| (f64::from(p[k]) - f64::from(q[k])).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        // Midpoint of every long edge, created in face order.
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        for fi in 0..mesh.indices.len() {
            let f = mesh.indices[fi];
            for k in 0..3 {
                let (a, b) = (f[k], f[(k + 1) % 3]);
                if len(mesh, a, b) > max_edge && !mid.contains_key(&key(a, b)) {
                    let (p, q) = (mesh.positions[a as usize], mesh.positions[b as usize]);
                    let m = [0, 1, 2].map(|k| ((f64::from(p[k]) + f64::from(q[k])) / 2.0) as f32);
                    mesh.positions.push(m);
                    let id = u32::try_from(mesh.positions.len() - 1).expect("fewer than 2^32");
                    mid.insert(key(a, b), id);
                }
            }
        }
        if mid.is_empty() {
            return;
        }
        let mut out = Vec::with_capacity(mesh.indices.len() * 2);
        for &f in &mesh.indices {
            let m = [0, 1, 2].map(|k| mid.get(&key(f[k], f[(k + 1) % 3])).copied());
            match m.iter().filter(|x| x.is_some()).count() {
                0 => out.push(f),
                1 => {
                    // Rotate so that edge 0 (v0 → v1) is the split one.
                    let r = m.iter().position(Option::is_some).expect("one split edge");
                    let v = [f[r], f[(r + 1) % 3], f[(r + 2) % 3]];
                    let m0 = m[r].expect("split");
                    out.push([v[0], m0, v[2]]);
                    out.push([m0, v[1], v[2]]);
                }
                2 => {
                    // Rotate so that edge 2 (v2 → v0) is the whole one.
                    let r = (m.iter().position(Option::is_none).expect("one whole edge") + 1) % 3;
                    let v = [f[r], f[(r + 1) % 3], f[(r + 2) % 3]];
                    let (m0, m1) = (m[r].expect("split"), m[(r + 1) % 3].expect("split"));
                    out.push([m0, v[1], m1]);
                    // The rest is the quad v0, m0, m1, v2: cut along its shorter diagonal.
                    if len(mesh, v[0], m1) <= len(mesh, m0, v[2]) {
                        out.push([v[0], m0, m1]);
                        out.push([v[0], m1, v[2]]);
                    } else {
                        out.push([v[0], m0, v[2]]);
                        out.push([m0, m1, v[2]]);
                    }
                }
                _ => {
                    let [m0, m1, m2] = m.map(|x| x.expect("split"));
                    out.push([f[0], m0, m2]);
                    out.push([m0, f[1], m1]);
                    out.push([m2, m1, f[2]]);
                    out.push([m0, m1, m2]);
                }
            }
        }
        mesh.indices = out;
    }
}

fn positions_as_bytes(p: &[[f32; 3]]) -> &[u8] {
    // SAFETY: `[f32; 3]` has no padding and every bit pattern is a valid `u8`.
    unsafe { std::slice::from_raw_parts(p.as_ptr().cast::<u8>(), std::mem::size_of_val(p)) }
}

/// Drops vertices no face uses, keeping the order of the rest.
fn drop_unused(mesh: &mut Mesh) {
    let mut new_index = vec![u32::MAX; mesh.positions.len()];
    for f in &mesh.indices {
        for &i in f {
            new_index[i as usize] = 0;
        }
    }
    let mut next = 0u32;
    let mut positions = Vec::new();
    for (i, slot) in new_index.iter_mut().enumerate() {
        if *slot == 0 {
            *slot = next;
            next += 1;
            positions.push(mesh.positions[i]);
        }
    }
    for f in &mut mesh.indices {
        *f = f.map(|i| new_index[i as usize]);
    }
    mesh.positions = positions;
}

#[cfg(test)]
mod tests {
    use super::super::testutil::*;
    use super::*;

    fn cfg() -> SurfaceConfig {
        SurfaceConfig::default()
    }

    /// [`cfg`] with a voxel so coarse that `max_edge` alone limits the edges.
    fn coarse(max_edge: f64) -> SurfaceConfig {
        SurfaceConfig {
            voxel: max_edge / TEXTURE_EDGE_VOXELS * 2.0,
            max_edge,
            ..cfg()
        }
    }

    /// A flat wall `x = 0.5` of `nx × ny` squares of `step` (m), normals `+x`.
    fn grid_wall(nx: usize, ny: usize, step: f32) -> Mesh {
        let mut mesh = Mesh::default();
        for iy in 0..=nx {
            for iz in 0..=ny {
                mesh.positions
                    .push([0.5, iy as f32 * step, iz as f32 * step]);
            }
        }
        let id = |iy: usize, iz: usize| (iy * (ny + 1) + iz) as u32;
        for iy in 0..nx {
            for iz in 0..ny {
                mesh.indices
                    .push([id(iy, iz), id(iy + 1, iz), id(iy, iz + 1)]);
                mesh.indices
                    .push([id(iy + 1, iz), id(iy + 1, iz + 1), id(iy, iz + 1)]);
            }
        }
        mesh
    }

    /// Largest distance from a vertex of `from` to the surface of `to` (m).
    fn one_sided(from: &Mesh, to: &Mesh) -> f64 {
        let up = |p: [f32; 3]| p.map(f64::from);
        let tris: Vec<[[f64; 3]; 3]> = to
            .indices
            .iter()
            .map(|f| f.map(|i| up(to.positions[i as usize])))
            .collect();
        // Only vertices a face uses: fixtures may leave orphans behind.
        let used: std::collections::BTreeSet<u32> =
            from.indices.iter().flatten().copied().collect();
        used.iter()
            .map(|&i| {
                let p = from.positions[i as usize];
                tris.iter()
                    .map(|&t| point_triangle_sq(up(p), t).sqrt())
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn flat_wall_gets_few_triangles() {
        let mesh = grid_wall(300, 250, 0.01); // 3 × 2.5 m
                                              // The reduction alone; the split of long edges is checked in `long_edges_are_split`.
        let out = simplify(&mesh, &coarse(100.0));
        assert!(
            out.indices.len() * 100 < mesh.indices.len(),
            "{} von {} Dreiecken übrig",
            out.indices.len(),
            mesh.indices.len()
        );
        assert!(out.positions.iter().all(|p| (p[0] - 0.5).abs() <= 0.001));
        assert!(out.positions.len() < mesh.positions.len());
        // Every vertex is used.
        let mut used = vec![false; out.positions.len()];
        out.indices
            .iter()
            .flatten()
            .for_each(|&i| used[i as usize] = true);
        assert!(used.iter().all(|&u| u));
    }

    #[test]
    fn simplify_keeps_rod() {
        let (mesh, _) = rod_on_wall(40, 0.01, 30);
        let out = simplify(&mesh, &cfg());
        assert!(out.indices.len() < mesh.indices.len());
        let hausdorff = one_sided(&out, &mesh).max(one_sided(&mesh, &out));
        assert!(hausdorff <= 0.002, "Hausdorff-Abstand {hausdorff} m");
        // The tip of the rod is still there.
        let tip = out.positions.iter().map(|p| p[0]).fold(0.0, f32::max);
        assert!(tip >= 0.3 - 0.002);
    }

    /// meshoptimizer's error is a weighted mean over the planes merged into a vertex, not a
    /// largest distance: where a thin rod meets a large wall, the wall planes outweigh the rod
    /// planes and a corner of the rod's foot can collapse into the wall although that moves the
    /// rod by its whole width (1–2 cm against the 2-mm limit). Which of these rods are hit
    /// depends on float rounding (FMA or not), so on the platform: check many.
    #[test]
    fn simplify_keeps_rods_within_error() {
        let err = cfg().simplify_error;
        for n in [40, 50, 60] {
            for len in [10, 20, 25, 30, 35] {
                let (mesh, _) = rod_on_wall(n, 0.01, len);
                let out = simplify(&mesh, &cfg());
                let hausdorff = one_sided(&out, &mesh).max(one_sided(&mesh, &out));
                assert!(
                    hausdorff <= err,
                    "Wand {n}, Stab {len}: Hausdorff-Abstand {hausdorff} m"
                );
            }
        }
    }

    #[test]
    fn border_unchanged() {
        let mesh = wall_with_hole(150, 0.01, Some((25, 100)));
        let before = edge_report(&mesh);
        assert_eq!(before, (0, 400));
        let out = simplify(&mesh, &cfg());
        assert_eq!(edge_report(&out), before);
        assert!(out.indices.len() < mesh.indices.len());
    }

    /// Longest edge of `mesh` (m).
    fn longest_edge(mesh: &Mesh) -> f64 {
        mesh.indices
            .iter()
            .flat_map(|f| {
                (0..3).map(move |k| {
                    let (p, q) = (
                        mesh.positions[f[k] as usize],
                        mesh.positions[f[(k + 1) % 3] as usize],
                    );
                    (0..3)
                        .map(|i| (f64::from(p[i]) - f64::from(q[i])).powi(2))
                        .sum::<f64>()
                        .sqrt()
                })
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn long_edges_are_split() {
        let mesh = grid_wall(300, 250, 0.01); // 3 × 2.5 m
        let unsplit = simplify(&mesh, &coarse(100.0));
        assert!(longest_edge(&unsplit) > 1.0, "{}", longest_edge(&unsplit));
        let max_edge = cfg().max_edge;
        let out = simplify(&mesh, &coarse(max_edge));
        assert!(longest_edge(&out) <= max_edge, "{}", longest_edge(&out));
        assert!(out.indices.len() > unsplit.indices.len());
        // The fans from the locked 1-cm border to the inside are split too, which costs most
        // of the faces; still far fewer than before the reduction.
        println!("{} -> {} faces", unsplit.indices.len(), out.indices.len());
        assert!(out.indices.len() * 4 < mesh.indices.len());
        // Still closed where it was, open where it was, consistently wound, on the plane.
        assert_eq!(edge_report(&out), edge_report(&unsplit));
        assert_eq!(edge_report(&out).0, 0);
        assert!(out.positions.iter().all(|p| (p[0] - 0.5).abs() <= 0.001));
        assert!((0..out.indices.len()).all(|f| out.face_normal(f)[0] > 0.99));
        // The area is unchanged.
        let area = |m: &Mesh| {
            (0..m.indices.len())
                .map(|f| f64::from(m.face_area(f)))
                .sum::<f64>()
        };
        assert!((area(&out) - area(&unsplit)).abs() < 1e-4, "{}", area(&out));
        // A closed slab with long edges stays closed.
        let mut slab = wall_with_hole(4, 1.0, None);
        let before = edge_report(&slab);
        split_long_edges(&mut slab, 0.3);
        assert_eq!(edge_report(&slab), before);
        assert_eq!(before, (0, 0));
        assert!(longest_edge(&slab) <= 0.3);
        // Deterministic.
        assert_eq!(simplify(&mesh, &coarse(max_edge)), out);
    }

    #[test]
    fn max_edge_is_capped_by_the_voxel() {
        let at = |voxel: f64| texture_edge(&SurfaceConfig { voxel, ..cfg() });
        assert_eq!(at(0.02), TEXTURE_EDGE_VOXELS * 0.02);
        assert_eq!(at(0.01), TEXTURE_EDGE_MIN);
        assert_eq!(at(0.005), TEXTURE_EDGE_MIN);
        assert_eq!(at(1.0), cfg().max_edge);
        let mesh = grid_wall(300, 250, 0.01); // 3 × 2.5 m
        for voxel in [0.01, 0.02] {
            let cfg = SurfaceConfig { voxel, ..cfg() };
            let edge = texture_edge(&cfg);
            assert!(edge < cfg.max_edge);
            let out = simplify(&mesh, &cfg);
            let longest = longest_edge(&out);
            assert!(longest <= edge, "{voxel}: {longest}");
            assert!(longest > 0.5 * edge, "{voxel}: {longest}");
            assert!(out.positions.iter().all(|p| (p[0] - 0.5).abs() <= 0.001));
        }
        // A coarse voxel: `max_edge` is the bound.
        let out = simplify(&mesh, &coarse(0.3));
        assert!(longest_edge(&out) <= 0.3);
        assert!(longest_edge(&out) > texture_edge(&cfg()));
    }

    /// [`far_vertices`] against a search over all faces.
    #[test]
    fn far_vertices_match_brute_force() {
        let (from, _) = rod_on_wall(30, 0.01, 20);
        let to = simplify(&from, &cfg());
        let up = |p: [f32; 3]| p.map(f64::from);
        // Shifted a little, so that some vertices are near and some far.
        let mut moved = to.clone();
        for (i, p) in moved.positions.iter_mut().enumerate() {
            p[0] += 0.0005 * (i % 7) as f32;
        }
        // Only vertices a face uses: the fixture leaves orphans behind.
        let used: std::collections::BTreeSet<u32> =
            from.indices.iter().flatten().copied().collect();
        for max in [0.0005, 0.001, 0.002, 0.005] {
            let want: Vec<u32> = (0..from.positions.len() as u32)
                .filter(|v| used.contains(v))
                .filter(|&v| {
                    let p = up(from.positions[v as usize]);
                    moved.indices.iter().all(|f| {
                        point_triangle_sq(p, f.map(|i| up(moved.positions[i as usize]))) > max * max
                    })
                })
                .collect();
            assert_eq!(far_vertices(&from, &moved, max), want, "{max}");
        }
        assert!(!far_vertices(&from, &moved, 0.0005).is_empty());
        // Nothing to be near to: all used vertices are far.
        assert_eq!(
            far_vertices(&from, &Mesh::default(), 0.002).len(),
            used.len()
        );
    }

    /// Smallest angle of any face of `mesh` (rad).
    fn min_angle_of(mesh: &Mesh) -> f64 {
        mesh.indices
            .iter()
            .map(|f| min_angle(f.map(|i| mesh.positions[i as usize].map(f64::from))))
            .fold(f64::INFINITY, f64::min)
    }

    /// A flat wall `x = 0.5` of parallelograms (sides 10 cm along y and (8, 2) cm), each cut
    /// along its long diagonal: slivers.
    fn sheared_wall(n: usize) -> Mesh {
        let mut mesh = Mesh::default();
        for j in 0..=n {
            for i in 0..=n {
                let (y, z) = (i as f32 * 0.1 + j as f32 * 0.08, j as f32 * 0.02);
                mesh.positions.push([0.5, y, z]);
            }
        }
        let id = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        for j in 0..n {
            for i in 0..n {
                mesh.indices
                    .push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                mesh.indices
                    .push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        mesh
    }

    #[test]
    fn slivers_on_flat_wall_are_flipped() {
        let mut mesh = sheared_wall(10);
        let before = mesh.clone();
        let worst = min_angle_of(&mesh);
        assert!(worst.to_degrees() < 7.0, "{}", worst.to_degrees());
        let flips = improve_triangles(&mut mesh, 0.001, 0.3);
        assert!(flips >= 100, "{flips}");
        // Every long diagonal is replaced by the short one: no angle under 10°.
        assert!(
            min_angle_of(&mesh).to_degrees() > 10.0,
            "{}",
            min_angle_of(&mesh).to_degrees()
        );
        assert_eq!(mesh.positions, before.positions);
        assert_eq!(edge_report(&mesh), edge_report(&before));
        assert!((0..mesh.indices.len()).all(|f| mesh.face_normal(f)[0] > 0.99));
        let area = |m: &Mesh| {
            (0..m.indices.len())
                .map(|f| f64::from(m.face_area(f)))
                .sum::<f64>()
        };
        assert!((area(&mesh) - area(&before)).abs() < 1e-6);
        // Deterministic.
        let mut again = before.clone();
        improve_triangles(&mut again, 0.001, 0.3);
        assert_eq!(again, mesh);
    }

    #[test]
    fn edges_and_folds_are_not_flipped() {
        // A closed slab: square cells (no better diagonal) and 90° edges.
        let mut slab = wall_with_hole(6, 0.1, None);
        let before = slab.clone();
        assert_eq!(improve_triangles(&mut slab, 0.001, 0.3), 0);
        assert_eq!(slab, before);
        // A sliver pair folded by 1 cm: flipping would move the surface by more than 1 mm.
        let mut fold = sheared_wall(1);
        fold.positions[2][0] += 0.01;
        let before = fold.clone();
        assert_eq!(improve_triangles(&mut fold, 0.001, 0.3), 0);
        assert_eq!(fold, before);
    }

    #[test]
    fn non_manifold_pair_is_not_flipped() {
        // The sliver pair of a sheared cell plus a copy of its first face: the edge a -> b is
        // used twice, so the map of edges is ambiguous there.
        let mut mesh = sheared_wall(1);
        mesh.indices.push(mesh.indices[0]);
        let before = mesh.clone();
        assert_eq!(improve_triangles(&mut mesh, 0.001, 0.3), 0);
        assert_eq!(mesh, before);
    }

    #[test]
    fn deterministic_and_empty() {
        let mesh = grid_wall(40, 30, 0.01);
        let (a, b) = (simplify(&mesh, &cfg()), simplify(&mesh, &cfg()));
        assert_eq!((a.positions, a.indices), (b.positions, b.indices));
        assert!(simplify(&Mesh::default(), &cfg()).indices.is_empty());
    }
}
