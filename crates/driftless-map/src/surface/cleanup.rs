//! Cleanup of a mesh: drop degenerate faces and small floating components, close small holes.
//!
//! Components are connected through shared edges, so a thin rod that is stitched to a wall
//! stays, while a speck that floats free is removed. A hole is a loop of boundary edges
//! (directed edges without an opposite partner); it is closed by a fan from its centroid
//! whose faces run against the boundary edges, which keeps the winding consistent with the
//! neighbourhood, but only if the cap leans like the faces around the loop (a hole in a
//! surface, not the rim of an open object). All iteration is over sorted containers, so the output is deterministic.

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
    fill_holes(mesh, cfg.max_hole_edges, &mut stats);
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

/// Closes boundary loops of at most `max_edges` edges with a fan from the centroid.
fn fill_holes(mesh: &mut Mesh, max_edges: usize, stats: &mut CleanupStats) {
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
    let mut face_of: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for (f, t) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            face_of.entry((t[k], t[(k + 1) % 3])).or_insert(f);
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
    for lp in loops {
        if lp.len() < 3 || lp.len() > max_edges {
            stats.holes_left += 1;
            continue;
        }
        let n = lp.len();
        let mut c = [0.0f32; 3];
        for &v in &lp {
            let p = mesh.positions[v as usize];
            c = [0, 1, 2].map(|k| c[k] + p[k] / n as f32);
        }
        let pos = |v: u32| mesh.positions[v as usize];
        // Cap normal from the fan winding (area-weighted sum), against the loop's own edges.
        let mut cap = [0.0f32; 3];
        for i in 0..n {
            let x = cross3(pos(lp[(i + 1) % n]), pos(lp[i]), c);
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
            stats.holes_left += 1;
            continue;
        }
        stats.holes_filled += 1;
        if n == 3 {
            // The existing faces run lp[0] -> lp[1] -> lp[2]; close against them.
            mesh.indices.push([lp[0], lp[2], lp[1]]);
            continue;
        }
        mesh.positions.push(c);
        let centre = (mesh.positions.len() - 1) as u32;
        for i in 0..n {
            mesh.indices.push([lp[(i + 1) % n], lp[i], centre]);
        }
    }
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
        assert_eq!(mesh.indices.len(), faces + 40);
        assert_eq!(edge_report(&mesh), (0, 0), "geschlossen, Umlauf stimmig");
        // The new faces face the same way as the wall around them (+x).
        for f in faces..mesh.indices.len() {
            assert!(mesh.face_normal(f)[0] > 0.99);
        }
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
