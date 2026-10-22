//! Cleanup of a mesh: drop degenerate faces and small floating components, close small holes.
//!
//! Components are connected through shared edges, so a thin rod that is stitched to a wall
//! stays, while a speck that floats free is removed. A hole is a loop of boundary edges
//! (directed edges without an opposite partner). A loop of at most
//! [`SurfaceConfig::max_hole_perimeter`] is closed if the cap leans like the faces around it (a
//! hole in a surface, not the rim of an open object or the end of a pipe): the loop is
//! projected onto its plane and triangulated there without new vertices, the faces running
//! against the boundary edges, which keeps the winding consistent with the neighbourhood. A
//! flat hole is closed flat. All iteration is over sorted containers (the loops in parallel,
//! collected in order), so the output is deterministic.

use super::{Mesh, SurfaceConfig};
use std::collections::{BTreeMap, BTreeSet};

/// A boundary loop is a hole in a surface (and gets filled) only if the cap it would get
/// leans the same way as the faces around it: the area-weighted mean of cos(angle between cap
/// normal and adjacent face normal) must exceed this. A hole in a wall gives about 1, the outer
/// rim of a flat open patch about -1 and the end of a pipe about 0, so the latter two stay
/// open (unseen areas and open objects are not invented).
const MIN_CAP_ALIGNMENT: f32 = 0.5;

/// A corner of another boundary loop counts as an island of a hole if it lies inside the hole
/// and within this distance of the hole's plane (m): about a voxel. A loop on the far side of
/// a wall (a hole through it) is further away and does not block.
pub const ISLAND_PLANE_TOLERANCE: f64 = 0.03;

/// Twice the area times the normal of the triangle `a b c`.
fn cross3(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [0, 1, 2].map(|k| b[k] - a[k]);
    let v = [0, 1, 2].map(|k| c[k] - a[k]);
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CleanupStats {
    /// Components removed for having fewer than `min_component_faces` faces.
    pub components_removed: usize,
    /// Faces removed: degenerate ones and those of removed components.
    pub faces_removed: usize,
    /// Boundary loops closed.
    pub holes_filled: usize,
    /// Boundary loops left open: rim longer than [`SurfaceConfig::max_hole_perimeter`].
    pub holes_too_large: usize,
    /// Left open: the cap would not lean like the faces around it (rim of an open piece,
    /// end of a pipe; see [`MIN_CAP_ALIGNMENT`]).
    pub holes_rim: usize,
    /// Left open: not a clean loop, more than [`MAX_CAP_VERTICES`] corners, no flat
    /// triangulation, another boundary loop inside it (an island), or a cap edge exists
    /// already.
    pub holes_untriangulable: usize,
}

impl CleanupStats {
    /// All boundary loops left open.
    pub fn holes_left(&self) -> usize {
        self.holes_too_large + self.holes_rim + self.holes_untriangulable
    }
}

/// Loops with more corners are left open (ear clipping is cubic in the corners).
pub const MAX_CAP_VERTICES: usize = 400;

pub fn cleanup(mesh: &mut Mesh, cfg: &SurfaceConfig) -> CleanupStats {
    let mut stats = CleanupStats::default();
    drop_degenerate(mesh, &mut stats);
    drop_small_components(mesh, cfg.min_component_faces, &mut stats);
    fill_holes(mesh, cfg.max_hole_perimeter, &mut stats);
    compact(mesh);
    stats
}

/// Removes faces with a repeated vertex or without area.
fn drop_degenerate(mesh: &mut Mesh, stats: &mut CleanupStats) {
    let keep: Vec<bool> = (0..mesh.indices.len())
        .map(|f| {
            let [a, b, c] = mesh.indices[f];
            a != b && b != c && a != c && mesh.face_area(f) > 0.0
        })
        .collect();
    stats.faces_removed += retain_faces(mesh, &keep);
}

/// Keeps the faces with `keep[f]`; returns how many were dropped.
fn retain_faces(mesh: &mut Mesh, keep: &[bool]) -> usize {
    let before = mesh.indices.len();
    let mut it = keep.iter();
    mesh.indices.retain(|_| *it.next().unwrap());
    before - mesh.indices.len()
}

fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Removes components (faces joined by shared edges) with fewer than `min_faces` faces.
fn drop_small_components(mesh: &mut Mesh, min_faces: usize, stats: &mut CleanupStats) {
    let n = mesh.indices.len();
    let mut parent: Vec<usize> = (0..n).collect();
    let mut edges: Vec<((u32, u32), usize)> = Vec::with_capacity(n * 3);
    for (f, tri) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            edges.push(((a.min(b), a.max(b)), f));
        }
    }
    edges.sort_unstable();
    for w in edges.windows(2) {
        if w[0].0 == w[1].0 {
            let (ra, rb) = (find(&mut parent, w[0].1), find(&mut parent, w[1].1));
            if ra != rb {
                parent[ra.max(rb)] = ra.min(rb);
            }
        }
    }
    let root: Vec<usize> = (0..n).map(|f| find(&mut parent, f)).collect();
    let mut size: BTreeMap<usize, usize> = BTreeMap::new();
    for &r in &root {
        *size.entry(r).or_default() += 1;
    }
    let small: BTreeSet<usize> = size
        .iter()
        .filter(|&(_, &s)| s < min_faces)
        .map(|(&r, _)| r)
        .collect();
    if small.is_empty() {
        return;
    }
    let keep: Vec<bool> = root.iter().map(|r| !small.contains(r)).collect();
    stats.components_removed += small.len();
    stats.faces_removed += retain_faces(mesh, &keep);
}

/// Length of the loop `lp` (m).
fn perimeter(positions: &[[f32; 3]], lp: &[u32]) -> f64 {
    (0..lp.len())
        .map(|i| {
            let (p, q) = (
                positions[lp[i] as usize],
                positions[lp[(i + 1) % lp.len()] as usize],
            );
            (0..3)
                .map(|k| (f64::from(p[k]) - f64::from(q[k])).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .sum()
}

/// Why a boundary loop stays open (see [`CleanupStats`]).
enum Open {
    TooLarge,
    Rim,
    Untriangulable,
}

/// Closes boundary loops of at most `max_perimeter` (m) that are holes in a surface (see
/// [`MIN_CAP_ALIGNMENT`]) with [`cap_faces`]; loops it cannot triangulate stay open.
fn fill_holes(mesh: &mut Mesh, max_perimeter: f64, stats: &mut CleanupStats) {
    use rayon::prelude::*;
    // One sorted array rather than a tree: a single allocation, given back when dropped.
    let mut directed: Vec<(u32, u32)> = mesh
        .indices
        .iter()
        .flat_map(|t| (0..3).map(move |k| (t[k], t[(k + 1) % 3])))
        .collect();
    directed.par_sort_unstable();
    directed.dedup();
    // Boundary edges a -> b of the existing faces, grouped by their start vertex.
    let mut out: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if directed.binary_search(&(b, a)).is_err() {
            out.entry(a).or_default().push(b);
        }
    }
    // Whether the mesh has an edge between a and b (either direction).
    let has_edge = |a: u32, b: u32| {
        directed.binary_search(&(a, b)).is_ok() || directed.binary_search(&(b, a)).is_ok()
    };
    let mut face_of: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for (f, t) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            let e = (t[k], t[(k + 1) % 3]);
            if out.get(&e.0).is_some_and(|bs| bs.contains(&e.1)) {
                face_of.entry(e).or_insert(f);
            }
        }
    }
    let mut used: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut loops: Vec<Vec<u32>> = Vec::new();
    let starts: Vec<(u32, u32)> = out
        .iter()
        .flat_map(|(&a, bs)| bs.iter().map(move |&b| (a, b)))
        .collect();
    for (a, b) in starts {
        if used.contains(&(a, b)) {
            continue;
        }
        // Walk the boundary; a vertex met twice closes a sub-loop (pinch point).
        let mut path = vec![a];
        let mut cur = a;
        loop {
            let next = out
                .get(&cur)
                .and_then(|bs| bs.iter().copied().find(|&b| !used.contains(&(cur, b))));
            let Some(next) = next else {
                if path.len() > 1 {
                    stats.holes_untriangulable += 1; // dead end: not a clean loop
                }
                break;
            };
            used.insert((cur, next));
            if let Some(i) = path.iter().position(|&v| v == next) {
                loops.push(path[i..].to_vec());
                path.truncate(i + 1);
            } else {
                path.push(next);
            }
            cur = next;
        }
    }
    let pos = |v: u32| mesh.positions[v as usize];
    // Every loop corner by x, for the island test.
    let mut corners: Vec<(f32, u32, u32)> = loops
        .iter()
        .enumerate()
        .flat_map(|(l, lp)| lp.iter().map(move |&v| (0.0, l as u32, v)))
        .collect();
    for c in &mut corners {
        c.0 = mesh.positions[c.2 as usize][0];
    }
    corners.par_sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then((a.1, a.2).cmp(&(b.1, b.2))));
    let caps: Vec<Result<Vec<[u32; 3]>, Open>> = loops
        .par_iter()
        .enumerate()
        .map(|(l, lp)| {
            let n = lp.len();
            if perimeter(&mesh.positions, lp) > max_perimeter {
                return Err(Open::TooLarge);
            }
            if !(3..=MAX_CAP_VERTICES).contains(&n) {
                return Err(Open::Untriangulable);
            }
            // The cap runs against the loop: its area vector (Newell) from the reversed loop,
            // in f64 and relative to the first corner (far from the origin, absolute f32
            // coordinates would drown a small hole in rounding).
            let o = pos(lp[0]).map(f64::from);
            let rel = |v: u32| {
                let p = pos(v);
                [0, 1, 2].map(|k| f64::from(p[k]) - o[k])
            };
            let mut cap = [0.0f64; 3];
            for i in 0..n {
                let (a, b) = (rel(lp[(i + 1) % n]), rel(lp[i]));
                let x = [
                    a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0],
                ];
                cap = [0, 1, 2].map(|k| cap[k] + x[k]);
            }
            let cap_len = (cap[0] * cap[0] + cap[1] * cap[1] + cap[2] * cap[2]).sqrt();
            // Area-weighted mean of cos(cap, adjacent face).
            let (mut align, mut area) = (0.0f64, 0.0f64);
            for i in 0..n {
                if let Some(&f) = face_of.get(&(lp[i], lp[(i + 1) % n])) {
                    let [a, b, d] = mesh.indices[f].map(pos);
                    let x = cross3(a, b, d).map(f64::from);
                    align += (cap[0] * x[0] + cap[1] * x[1] + cap[2] * x[2])
                        / cap_len.max(f64::MIN_POSITIVE);
                    area += (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
                }
            }
            if !(cap_len > 0.0 && area > 0.0 && align / area > f64::from(MIN_CAP_ALIGNMENT)) {
                return Err(Open::Rim);
            }
            // Corners of other loops within the loop's box: candidates for an island.
            let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
            for &v in lp {
                let p = pos(v);
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            let margin = ISLAND_PLANE_TOLERANCE as f32;
            let start = corners.partition_point(|c| c.0 < lo[0] - margin);
            let others: Vec<u32> = corners[start..]
                .iter()
                .take_while(|c| c.0 <= hi[0] + margin)
                .filter(|c| c.1 != l as u32 && !lp.contains(&c.2))
                .map(|c| c.2)
                .filter(|&v| {
                    let p = pos(v);
                    (1..3).all(|k| p[k] >= lo[k] - margin && p[k] <= hi[k] + margin)
                })
                .collect();
            let rev: Vec<u32> = lp.iter().rev().copied().collect();
            cap_faces(&mesh.positions, &rev, &others, &has_edge).ok_or(Open::Untriangulable)
        })
        .collect();
    // Diagonals of the caps taken so far: two caps must not share one either.
    let mut taken: BTreeSet<(u32, u32)> = BTreeSet::new();
    for cap in caps {
        let cap = cap.and_then(|faces| {
            let mut diagonals: Vec<(u32, u32)> = faces
                .iter()
                .flat_map(|f| (0..3).map(move |k| (f[k], f[(k + 1) % 3])))
                .filter(|&(a, b)| directed.binary_search(&(b, a)).is_err())
                .map(|(a, b)| (a.min(b), a.max(b)))
                .collect();
            diagonals.sort_unstable();
            diagonals.dedup();
            if diagonals.iter().any(|d| taken.contains(d)) {
                return Err(Open::Untriangulable);
            }
            taken.extend(diagonals);
            Ok(faces)
        });
        match cap {
            Ok(faces) => {
                stats.holes_filled += 1;
                mesh.indices.extend(faces);
            }
            Err(Open::TooLarge) => stats.holes_too_large += 1,
            Err(Open::Rim) => stats.holes_rim += 1,
            Err(Open::Untriangulable) => stats.holes_untriangulable += 1,
        }
    }
}

/// Triangles that close the polygon `poly` (mesh vertices, in the winding the cap gets),
/// without new vertices: the polygon is projected onto its plane (normal from Newell's
/// area vector, the best plane through a closed loop) and cut by ear clipping, always taking
/// the ear with the largest smallest angle. A flat hole is closed flat. `None` if the
/// projection has no ear left (it overlaps itself), a face would turn against the plane
/// normal, a corner of `others` (other boundary loops) lies inside the polygon within
/// [`ISLAND_PLANE_TOLERANCE`] of its plane (an island: the cap would cover it twice), or every
/// remaining ear would need a diagonal for which `has_edge` is true (the edge exists already;
/// a third face on it would make the mesh non-manifold).
pub(crate) fn cap_faces(
    positions: &[[f32; 3]],
    poly: &[u32],
    others: &[u32],
    has_edge: &dyn Fn(u32, u32) -> bool,
) -> Option<Vec<[u32; 3]>> {
    let n = poly.len();
    if !(3..=MAX_CAP_VERTICES).contains(&n) {
        return None;
    }
    // Relative to the first corner: no cancellation far from the origin.
    let o = positions[poly[0] as usize].map(f64::from);
    let at = |v: u32| {
        let p = positions[v as usize];
        [0, 1, 2].map(|k| f64::from(p[k]) - o[k])
    };
    let p: Vec<[f64; 3]> = poly.iter().map(|&v| at(v)).collect();
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let mut normal = [0.0f64; 3];
    for i in 0..n {
        let x = cross(p[i], p[(i + 1) % n]);
        normal = [0, 1, 2].map(|k| normal[k] + x[k]);
    }
    let len = dot(normal, normal).sqrt();
    if !len.is_normal() {
        return None;
    }
    let nrm = normal.map(|x| x / len);
    // Any axis not along the normal gives the first in-plane direction.
    let axis = if nrm[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = cross(axis, nrm);
    let u = u.map(|x| x / dot(u, u).sqrt());
    let v = cross(nrm, u);
    // Counter-clockwise in (u, v): the area vector points along the normal.
    let q: Vec<[f64; 2]> = p.iter().map(|&x| [dot(x, u), dot(x, v)]).collect();
    let orient = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    // Scale of the polygon, for the tolerance of "convex".
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for x in &q {
        lo = [lo[0].min(x[0]), lo[1].min(x[1])];
        hi = [hi[0].max(x[0]), hi[1].max(x[1])];
    }
    let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-9);
    let eps = 1e-12 * extent * extent;
    // An island: a corner of another loop inside the polygon, near its plane.
    for &w in others {
        let x = at(w);
        if dot(x, nrm).abs() > ISLAND_PLANE_TOLERANCE {
            continue;
        }
        let x = [dot(x, u), dot(x, v)];
        let mut inside = false;
        for i in 0..n {
            let (a, b) = (q[i], q[(i + 1) % n]);
            if (a[1] > x[1]) != (b[1] > x[1])
                && x[0] < a[0] + (x[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            {
                inside = !inside;
            }
        }
        if inside {
            return None;
        }
    }
    let min_angle = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        let ang = |o: [f64; 2], x: [f64; 2], y: [f64; 2]| {
            let (dx, dy) = ([x[0] - o[0], x[1] - o[1]], [y[0] - o[0], y[1] - o[1]]);
            (dx[0] * dy[1] - dx[1] * dy[0])
                .abs()
                .atan2(dx[0] * dy[0] + dx[1] * dy[1])
        };
        ang(a, b, c).min(ang(b, c, a)).min(ang(c, a, b))
    };
    let mut rem: Vec<usize> = (0..n).collect();
    let mut faces = Vec::with_capacity(n - 2);
    while rem.len() > 3 {
        let m = rem.len();
        let mut best: Option<(f64, usize)> = None;
        for i in 0..m {
            let (ia, ib, ic) = (rem[(i + m - 1) % m], rem[i], rem[(i + 1) % m]);
            let (a, b, c) = (q[ia], q[ib], q[ic]);
            if orient(a, b, c) <= eps {
                continue; // reflex or straight
            }
            if m > 3 && has_edge(poly[ia], poly[ic]) {
                continue; // the diagonal exists already
            }
            // No other corner inside or on the ear (corners at the ear's own points excepted).
            let blocked = rem.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let x = q[j];
                if x == a || x == b || x == c {
                    return false;
                }
                orient(a, b, x) >= 0.0 && orient(b, c, x) >= 0.0 && orient(c, a, x) >= 0.0
            });
            if blocked {
                continue;
            }
            let score = min_angle(a, b, c);
            if best.is_none_or(|(s, _)| score > s) {
                best = Some((score, i));
            }
        }
        let (_, i) = best?;
        let m = rem.len();
        faces.push([rem[(i + m - 1) % m], rem[i], rem[(i + 1) % m]]);
        rem.remove(i);
    }
    if orient(q[rem[0]], q[rem[1]], q[rem[2]]) <= eps {
        return None;
    }
    faces.push([rem[0], rem[1], rem[2]]);
    // No face may turn against the plane in 3D either.
    for f in &faces {
        let x = cross(sub(p[f[1]], p[f[0]]), sub(p[f[2]], p[f[0]]));
        if dot(x, nrm) <= 0.0 {
            return None;
        }
    }
    Some(faces.into_iter().map(|f| f.map(|i| poly[i])).collect())
}

/// Drops vertices no face uses, keeping the order of the rest.
fn compact(mesh: &mut Mesh) {
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

    #[test]
    fn removes_floating_speck() {
        let mut mesh = open_wall(40, 0.01);
        let wall_faces = mesh.indices.len();
        append(&mut mesh, &speck([1.0, 0.2, 0.2], 0.02));
        assert_eq!(mesh.indices.len(), wall_faces + 20);
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!(stats.components_removed, 1);
        assert_eq!(stats.faces_removed, 20);
        assert_eq!(mesh.indices.len(), wall_faces);
        assert_eq!(mesh.positions.len(), 41 * 41, "Eckpunkte des Teilchens weg");
        assert!(mesh.positions.iter().all(|p| p[0] == 0.0));
    }

    #[test]
    fn keeps_thin_rod_attached_to_wall() {
        let (mut mesh, rod) = rod_on_wall(40, 0.01, 5);
        assert!(rod < cfg().min_component_faces, "Stab allein wäre zu klein");
        let faces = mesh.indices.len();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!(stats.components_removed, 0);
        assert_eq!(stats.faces_removed, 0);
        assert_eq!(stats.holes_filled, 0, "Stabkappe ist geschlossen");
        assert_eq!(mesh.indices.len(), faces);
        assert!(mesh.positions.iter().any(|p| p[0] > 0.04), "Stab noch da");
    }

    #[test]
    fn fills_small_hole() {
        let mut mesh = wall_with_hole(40, 0.01, Some((15, 10))); // 10 cm hole
        assert_eq!(edge_report(&mesh), (0, 40));
        let faces = mesh.indices.len();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (1, 0));
        assert_eq!(mesh.indices.len(), faces + 38);
        assert_flat_fill(&mesh, faces, 0.01);
    }

    /// The front of `mesh` (`x = 2 · step`, from [`wall_with_hole`]) after cleanup: every
    /// face added by the fill faces +x, lies in the plane (±2 mm) and has area; the mesh is
    /// closed and no directed edge is used twice.
    fn assert_flat_fill(mesh: &Mesh, faces_before: usize, step: f32) {
        assert_eq!(
            edge_report(mesh),
            (0, 0),
            "geschlossen, keine Kante doppelt"
        );
        assert!(mesh.indices.len() > faces_before);
        for f in faces_before..mesh.indices.len() {
            assert!(
                mesh.face_normal(f)[0] > 0.99,
                "verdreht: {:?}",
                mesh.face_normal(f)
            );
            assert!(mesh.face_area(f) > 0.0);
            for v in mesh.indices[f] {
                let x = mesh.positions[v as usize][0];
                assert!((x - 2.0 * step).abs() <= 0.002, "x = {x}");
            }
        }
    }

    #[test]
    fn fills_40cm_hole_flat_at_2cm() {
        // 20 × 20 squares of 2 cm: 40 cm, 80 edges, perimeter 1.6 m.
        let mut mesh = wall_with_hole(40, 0.02, Some((10, 20)));
        assert_eq!(edge_report(&mesh), (0, 80));
        let (faces, vertices) = (mesh.indices.len(), mesh.positions.len());
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (1, 0));
        assert_flat_fill(&mesh, faces, 0.02);
        // Triangulated from the rim alone: no new vertices, n - 2 faces.
        assert_eq!(mesh.positions.len(), vertices);
        assert_eq!(mesh.indices.len(), faces + 78);
    }

    #[test]
    fn fills_u_shaped_hole_without_twisted_faces() {
        // A U of 2 cm squares (86 edges, perimeter 1.72 m) whose centroid lies in its notch: a fan from
        // the centroid would turn some faces over.
        let mut mesh = wall_with_shaped_hole(30, 0.02, |a, b| {
            (6..22).contains(&a) && (6..22).contains(&b) && !((11..17).contains(&a) && b >= 11)
        });
        assert_eq!(edge_report(&mesh), (0, 86));
        let faces = mesh.indices.len();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (1, 0));
        assert_flat_fill(&mesh, faces, 0.02);
    }

    #[test]
    fn hole_over_2m_perimeter_stays_open() {
        // 26 squares of 2 cm: perimeter 2.08 m.
        let mut mesh = wall_with_hole(50, 0.02, Some((10, 26)));
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (0, 1));
        assert_eq!(mesh, before);
    }

    #[test]
    fn open_patch_rim_1_5m_stays_open() {
        // 19 squares of 2 cm: a rim of 1.52 m, short enough to fill, but it is a rim.
        let mut mesh = open_wall(19, 0.02);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (0, 1));
        assert_eq!(mesh, before);
    }

    /// `mesh` moved by `d` (m).
    fn moved(mut mesh: Mesh, d: [f32; 3]) -> Mesh {
        for p in &mut mesh.positions {
            *p = [0, 1, 2].map(|k| p[k] + d[k]);
        }
        mesh
    }

    const FAR: [f32; 3] = [25.0, -30.0, 3.0];

    #[test]
    fn far_from_origin_hole_fills_and_rim_stays_open() {
        let mut mesh = moved(wall_with_hole(40, 0.02, Some((10, 20))), FAR);
        let faces = mesh.indices.len();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!(
            (stats.holes_filled, stats.holes_left()),
            (1, 0),
            "{stats:?}"
        );
        assert_eq!(edge_report(&mesh), (0, 0));
        for f in faces..mesh.indices.len() {
            assert!(mesh.face_normal(f)[0] > 0.99);
        }
        let mut rim = moved(open_wall(19, 0.02), FAR);
        let before = rim.clone();
        let stats = cleanup(&mut rim, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_rim), (0, 1), "{stats:?}");
        assert_eq!(rim, before);
    }

    /// A 2 cm open wall with the two faces of one cell removed (a 4-edge hole a b c d) and a
    /// closed two-sided fin on the cell diagonal `diag` (an edge that exists already).
    fn wall_with_fin(diag: usize) -> Mesh {
        let n = 10;
        let mut mesh = open_wall(n, 0.02);
        let id = |iy: usize, iz: usize| (iy * (n + 1) + iz) as u32;
        let cell = [id(4, 4), id(5, 4), id(5, 5), id(4, 5)];
        mesh.indices.retain(|f| !f.iter().all(|v| cell.contains(v)));
        let (a, c) = (cell[diag], cell[diag + 2]);
        mesh.positions.push([0.05, 0.09, 0.09]);
        let x = (mesh.positions.len() - 1) as u32;
        mesh.indices.push([a, c, x]);
        mesh.indices.push([c, a, x]);
        mesh
    }

    #[test]
    fn cap_does_not_reuse_existing_edge() {
        let c = SurfaceConfig {
            min_component_faces: 1,
            ..cfg()
        };
        for diag in 0..2 {
            let mut mesh = wall_with_fin(diag);
            assert_eq!(edge_report(&mesh).0, 0);
            let stats = cleanup(&mut mesh, &c);
            assert_eq!(stats.holes_filled, 1, "{diag}: {stats:?}");
            assert_eq!(stats.holes_rim, 1, "{diag}: outer rim");
            assert_eq!(edge_report(&mesh).0, 0, "{diag}: Kante doppelt");
            // No edge carries more than two faces.
            let mut count: BTreeMap<(u32, u32), usize> = BTreeMap::new();
            for f in &mesh.indices {
                for k in 0..3 {
                    let (a, b) = (f[k], f[(k + 1) % 3]);
                    *count.entry((a.min(b), a.max(b))).or_default() += 1;
                }
            }
            assert!(count.values().all(|&c| c <= 2), "{diag}");
        }
    }

    #[test]
    fn hole_around_island_stays_open() {
        // A 16 × 16 hole of 2 cm squares (1.28 m rim) with a 6 × 6 island left in its middle:
        // capping the outer loop would lay a second layer over the island.
        let mut mesh = wall_with_shaped_hole(30, 0.02, |a, b| {
            (6..22).contains(&a)
                && (6..22).contains(&b)
                && !((11..17).contains(&a) && (11..17).contains(&b))
        });
        let c = SurfaceConfig {
            min_component_faces: 1,
            ..cfg()
        };
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &c);
        assert_eq!(stats.holes_filled, 0, "{stats:?}");
        assert_eq!(stats.holes_rim, 1, "island rim: {stats:?}");
        assert_eq!(stats.holes_untriangulable, 1, "outer loop: {stats:?}");
        assert_eq!(mesh.indices.len(), before.indices.len());
    }

    #[test]
    fn counts_why_holes_stay_open() {
        let mut mesh = wall_with_hole(50, 0.02, Some((10, 26)));
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_too_large, stats.holes_rim), (1, 0));
        let mut mesh = open_wall(19, 0.02);
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_too_large, stats.holes_rim), (0, 1));
        assert_eq!(stats.holes_left(), 1);
    }

    #[test]
    fn large_hole_stays_open() {
        let mut mesh = wall_with_hole(150, 0.01, Some((25, 100))); // 1 m hole
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (0, 1));
        assert_eq!(mesh.indices.len(), before.indices.len());
        assert_eq!(edge_report(&mesh), (0, 400));
    }

    #[test]
    fn open_patch_rim_stays_open() {
        let mut mesh = open_wall(12, 0.01); // 288 faces, 48-edge rim
        assert!(mesh.indices.len() >= cfg().min_component_faces);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (0, 1));
        assert_eq!(mesh, before);
    }

    #[test]
    fn pipe_end_stays_open() {
        let mut mesh = open_tube(13, 0.01); // 208 faces, two 8-edge ends
        assert!(mesh.indices.len() >= cfg().min_component_faces);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left()), (0, 2));
        assert_eq!(mesh, before);
    }

    #[test]
    fn fills_triangular_hole() {
        let mut mesh = speck([0.0; 3], 0.05);
        mesh.indices.remove(0);
        let c = SurfaceConfig {
            min_component_faces: 1,
            ..cfg()
        };
        let stats = cleanup(&mut mesh, &c);
        assert_eq!((stats.holes_filled, stats.holes_left()), (1, 0));
        assert_eq!(mesh.indices.len(), 20);
        assert_eq!(mesh.positions.len(), 12);
        assert_eq!(edge_report(&mesh), (0, 0));
    }

    #[test]
    fn drops_degenerate_faces() {
        let mut mesh = open_wall(40, 0.01);
        let faces = mesh.indices.len();
        mesh.indices.push([0, 0, 1]); // repeated index
        mesh.positions.push([0.0, 0.0, 0.0]); // duplicate of vertex 0 -> zero area
        let dup = (mesh.positions.len() - 1) as u32;
        mesh.indices.push([0, dup, 1]);
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!(stats.faces_removed, 2);
        assert_eq!(mesh.indices.len(), faces);
        assert_eq!(stats.components_removed, 0);
    }

    #[test]
    fn deterministic() {
        let mut a = wall_with_hole(40, 0.01, Some((15, 10)));
        append(&mut a, &speck([1.0, 0.0, 0.0], 0.02));
        let mut b = a.clone();
        cleanup(&mut a, &cfg());
        cleanup(&mut b, &cfg());
        assert_eq!(a, b);
    }
}
