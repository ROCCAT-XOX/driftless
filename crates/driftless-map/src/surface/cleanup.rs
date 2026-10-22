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

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn len3(a: [f32; 3]) -> f32 {
    dot3(a, a).sqrt()
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CleanupStats {
    /// Components removed for having fewer than `min_component_faces` faces.
    pub components_removed: usize,
    /// Faces removed: degenerate ones and those of removed components.
    pub faces_removed: usize,
    /// Boundary loops closed.
    pub holes_filled: usize,
    /// Boundary loops left open (too large, or not a clean loop).
    pub holes_left: usize,
}

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

/// Closes boundary loops of at most `max_perimeter` (m) that are holes in a surface (see
/// [`MIN_CAP_ALIGNMENT`]) with [`cap_faces`]; loops it cannot triangulate stay open.
fn fill_holes(mesh: &mut Mesh, max_perimeter: f64, stats: &mut CleanupStats) {
    use rayon::prelude::*;
    let directed: BTreeSet<(u32, u32)> = mesh
        .indices
        .iter()
        .flat_map(|t| (0..3).map(move |k| (t[k], t[(k + 1) % 3])))
        .collect();
    // Boundary edges a -> b of the existing faces, grouped by their start vertex.
    let mut out: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            out.entry(a).or_default().push(b);
        }
    }
    drop(directed);
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
                    stats.holes_left += 1; // dead end: not a clean loop
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
    let caps: Vec<Option<Vec<[u32; 3]>>> = loops
        .par_iter()
        .map(|lp| {
            let n = lp.len();
            if n < 3 || perimeter(&mesh.positions, lp) > max_perimeter {
                return None;
            }
            // The cap runs against the loop: its area vector (Newell) from the reversed loop.
            let mut cap = [0.0f32; 3];
            for i in 0..n {
                let x = cross3([0.0; 3], pos(lp[(i + 1) % n]), pos(lp[i]));
                cap = [0, 1, 2].map(|k| cap[k] + x[k]);
            }
            let cap_len = len3(cap);
            // Area-weighted mean of cos(cap, adjacent face).
            let (mut align, mut area) = (0.0f32, 0.0f32);
            for i in 0..n {
                if let Some(&f) = face_of.get(&(lp[i], lp[(i + 1) % n])) {
                    let [a, b, d] = mesh.indices[f].map(pos);
                    let x = cross3(a, b, d);
                    align += dot3(cap, x) / cap_len.max(f32::MIN_POSITIVE);
                    area += len3(x);
                }
            }
            if !(cap_len > 0.0 && area > 0.0 && align / area > MIN_CAP_ALIGNMENT) {
                return None;
            }
            let rev: Vec<u32> = lp.iter().rev().copied().collect();
            cap_faces(&mesh.positions, &rev)
        })
        .collect();
    for cap in caps {
        match cap {
            Some(faces) => {
                stats.holes_filled += 1;
                mesh.indices.extend(faces);
            }
            None => stats.holes_left += 1,
        }
    }
}

/// Triangles that close the polygon `poly` (mesh vertices, in the winding the cap gets),
/// without new vertices: the polygon is projected onto its plane (normal from Newell's
/// area vector, the best plane through a closed loop) and cut by ear clipping, always taking
/// the ear with the largest smallest angle. A flat hole is closed flat. `None` if the
/// projection has no ear left (it overlaps itself) or a face would turn against the plane
/// normal.
pub(crate) fn cap_faces(positions: &[[f32; 3]], poly: &[u32]) -> Option<Vec<[u32; 3]>> {
    let n = poly.len();
    if n < 3 {
        return None;
    }
    let p: Vec<[f64; 3]> = poly
        .iter()
        .map(|&v| positions[v as usize].map(f64::from))
        .collect();
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
        assert_eq!((stats.holes_filled, stats.holes_left), (1, 0));
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
        assert_eq!((stats.holes_filled, stats.holes_left), (1, 0));
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
        assert_eq!((stats.holes_filled, stats.holes_left), (1, 0));
        assert_flat_fill(&mesh, faces, 0.02);
    }

    #[test]
    fn hole_over_2m_perimeter_stays_open() {
        // 26 squares of 2 cm: perimeter 2.08 m.
        let mut mesh = wall_with_hole(50, 0.02, Some((10, 26)));
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left), (0, 1));
        assert_eq!(mesh, before);
    }

    #[test]
    fn open_patch_rim_1_5m_stays_open() {
        // 19 squares of 2 cm: a rim of 1.52 m, short enough to fill, but it is a rim.
        let mut mesh = open_wall(19, 0.02);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left), (0, 1));
        assert_eq!(mesh, before);
    }

    #[test]
    fn large_hole_stays_open() {
        let mut mesh = wall_with_hole(150, 0.01, Some((25, 100))); // 1 m hole
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left), (0, 1));
        assert_eq!(mesh.indices.len(), before.indices.len());
        assert_eq!(edge_report(&mesh), (0, 400));
    }

    #[test]
    fn open_patch_rim_stays_open() {
        let mut mesh = open_wall(12, 0.01); // 288 faces, 48-edge rim
        assert!(mesh.indices.len() >= cfg().min_component_faces);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left), (0, 1));
        assert_eq!(mesh, before);
    }

    #[test]
    fn pipe_end_stays_open() {
        let mut mesh = open_tube(13, 0.01); // 208 faces, two 8-edge ends
        assert!(mesh.indices.len() >= cfg().min_component_faces);
        let before = mesh.clone();
        let stats = cleanup(&mut mesh, &cfg());
        assert_eq!((stats.holes_filled, stats.holes_left), (0, 2));
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
        assert_eq!((stats.holes_filled, stats.holes_left), (1, 0));
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
