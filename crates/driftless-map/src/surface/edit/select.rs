//! Stellen as faces: which faces of a [`WeldedSurface`] a dab of the brush covers.
//!
//! A dab is a sphere (centre, radius) with a normal: the side of the surface it was painted
//! on. It covers the faces that are connected over shared edges, starting at the face nearest
//! to its centre, whose closest point lies within the radius and whose normal is less than 60°
//! from the dab's (dot product above [`MIN_NORMAL_DOT`]); the start is the nearest such face.
//! So a dab stays on what it was painted on: the back of a thin wall lies in the sphere, but it
//! faces the other way and is joined to the front only round the edge; at a corner the other
//! wall is joined, but faces 90° away; an object in front of a wall is not joined to it. A
//! rough wall keeps no holes: its faces scatter by tens of degrees, well within 60°.
//!
//! The dabs of a Stelle apply in their order: a painting dab adds its faces, an erasing one
//! takes them out again. So the last dab that covers a face decides.
//!
//! Then [`select_faces`] closes the holes the Stelle encloses. Where the surface is very
//! noisy, single faces inside a dab are more than 60° off (or cut off from the start by such
//! faces); left out, they would hold their corners in place and leave dimples when the Stelle
//! is smoothed, just where it is noisiest. A hole is a group of faces, connected over edges,
//! that no dab took and that lie in the sphere of a painting dab (the last dab whose sphere
//! holds them paints). It joins the Stelle if it is enclosed: every face it shares an edge
//! with is selected, at least one, and none of its edges is an open edge of the surface. The
//! back of a thin wall, the other wall at a corner and an object in front of a wall touch
//! faces outside the spheres and stay out; so do the faces an erasing dab took, and with them
//! an erased spot.

use super::{dot, sub, unit, FaceBvh, RayHit, WeldedSurface};
use rayon::prelude::*;

/// A face counts for a dab if the dot product of their normals is above this: they are less
/// than 60° apart.
pub const MIN_NORMAL_DOT: f64 = 0.5;

/// One dab of the brush, in the frame of the surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelectDab {
    pub centre: [f64; 3],
    /// The side it was painted on ([`dab_normal`]). Without a direction (zero) the normals of
    /// the faces do not matter.
    pub normal: [f32; 3],
    /// Metres.
    pub radius: f64,
    /// Takes its faces out instead of adding them.
    pub erase: bool,
}

/// The normal for a dab of `radius` at `hit`: the mean normal of the faces within half the
/// radius, weighted by area, of those less than 60° from the hit face's normal (the back of a
/// thin wall or the other wall at a corner must not tilt it); then once more, with that mean
/// in place of the hit face's normal (one odd face at the hit must not tilt it either). The
/// hit face's normal if no face counts; zero if the hit face has no area either.
pub fn dab_normal(s: &WeldedSurface, bvh: &FaceBvh, hit: &RayHit, radius: f64) -> [f32; 3] {
    let near = bvh.faces_within(s, hit.pos, radius / 2.0);
    let mean = |towards: [f64; 3]| {
        let mut sum = [0.0; 3];
        for &f in &near {
            if dot(s.face_normal(f), towards) > MIN_NORMAL_DOT {
                // Normal times twice the area.
                for (sum, c) in sum.iter_mut().zip(s.face_cross(f)) {
                    *sum += c;
                }
            }
        }
        unit(sum)
    };
    let own = s.face_normal(hit.face);
    let first = mean(own).unwrap_or(own);
    mean(first).unwrap_or(first).map(|c| c as f32)
}

/// Slack on the reach of a face's neighbours (m), far above the rounding of a distance.
const REACH_MARGIN: f64 = 1e-6;

/// The faces one dab covers (see the module docs), sorted. Nothing for a radius below zero or
/// not a number, or a centre that is not finite. These are the faces of the dab alone:
/// [`select_faces`] also closes the holes the dabs of a Stelle enclose.
pub fn dab_faces(s: &WeldedSurface, bvh: &FaceBvh, dab: &SelectDab) -> Vec<u32> {
    reach(s, bvh, dab).taken
}

/// The faces the dabs of a Stelle cover, in their order: painting dabs add their faces,
/// erasing dabs take them out; then the holes the Stelle encloses are closed (see the module
/// docs). Sorted, every face once. The dabs are looked at in parallel.
pub fn select_faces(s: &WeldedSurface, bvh: &FaceBvh, dabs: &[SelectDab]) -> Vec<u32> {
    let reaches: Vec<Reach> = dabs.par_iter().map(|d| reach(s, bvh, d)).collect();
    // (face, dab) for every dab taking a face: the last one decides.
    let mut by_face: Vec<(u32, u32)> = reaches
        .iter()
        .enumerate()
        .flat_map(|(d, r)| r.taken.iter().map(move |&f| (f, d as u32)))
        .collect();
    by_face.sort_unstable();
    let (mut selected, mut taken) = (Vec::new(), Vec::new());
    for same in by_face.chunk_by(|x, y| x.0 == y.0) {
        let (face, last) = same[same.len() - 1];
        taken.push(face);
        if !dabs[last as usize].erase {
            selected.push(face);
        }
    }
    let holes = enclosed_holes(s, bvh, dabs, &reaches, &selected, &taken);
    if holes.is_empty() {
        return selected;
    }
    selected.extend(holes);
    selected.sort_unstable();
    selected
}

/// What one dab reaches: the faces in its sphere (closest point within the radius) with their
/// squared distances, by face, and the faces it takes, sorted.
struct Reach {
    near: Vec<(u32, f64)>,
    taken: Vec<u32>,
}

fn reach(s: &WeldedSurface, bvh: &FaceBvh, dab: &SelectDab) -> Reach {
    let near = bvh.faces_within_dist2(s, dab.centre, dab.radius);
    let normal = unit(dab.normal.map(f64::from));
    let faces: Vec<(u32, f64)> = near
        .iter()
        .copied()
        .filter(|&(f, _)| normal.is_none_or(|n| dot(s.face_normal(f), n) > MIN_NORMAL_DOT))
        .collect();
    // The nearest face starts; at the same distance the smaller one.
    let Some(start) = (0..faces.len()).min_by(|&i, &j| {
        let (a, b) = (faces[i], faces[j]);
        a.1.total_cmp(&b.1).then(a.0.cmp(&b.0))
    }) else {
        return Reach {
            near,
            taken: Vec::new(),
        };
    };
    // Faces with the same two corners on an edge are joined.
    let ids: Vec<u32> = faces.iter().map(|&(f, _)| f).collect();
    let edges = edges_of(s, &ids);
    let mut parent: Vec<u32> = (0..faces.len() as u32).collect();
    for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        for e in &same[1..] {
            join(&mut parent, same[0].2, e.2);
        }
    }
    let root = find(&mut parent, start as u32);
    let taken = (0..faces.len())
        .filter(|&i| find(&mut parent, i as u32) == root)
        .map(|i| faces[i].0)
        .collect();
    Reach { near, taken }
}

/// The holes the Stelle `selected` encloses (see the module docs), sorted. `taken` are the
/// faces any dab took, sorted.
fn enclosed_holes(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    dabs: &[SelectDab],
    reaches: &[Reach],
    selected: &[u32],
    taken: &[u32],
) -> Vec<u32> {
    // Candidates: faces no dab took whose last dab holding them in its sphere paints.
    let mut holding: Vec<(u32, u32)> = reaches
        .iter()
        .enumerate()
        .flat_map(|(d, r)| r.near.iter().map(move |&(f, _)| (f, d as u32)))
        .collect();
    holding.sort_unstable();
    let candidates: Vec<u32> = holding
        .chunk_by(|x, y| x.0 == y.0)
        .filter_map(|same| {
            let (face, last) = same[same.len() - 1];
            let free = !dabs[last as usize].erase && taken.binary_search(&face).is_err();
            free.then_some(face)
        })
        .collect();
    if candidates.is_empty() {
        return candidates;
    }
    let candidate = |f: u32| candidates.binary_search(&f).ok();

    // Every face sharing an edge with a candidate `g` in the sphere of dab `d` lies within
    // `d.radius` plus the longest edge of `g` of its centre: the shared edge is part of `g`.
    // So these spheres, around the painting dabs holding candidates, hold all neighbours.
    let around: Vec<Vec<u32>> = dabs
        .par_iter()
        .zip(reaches)
        .map(|(d, r)| {
            let longest = r
                .near
                .iter()
                .filter(|&&(f, _)| candidate(f).is_some())
                .map(|&(f, _)| longest_edge(s, f))
                .fold(f64::NEG_INFINITY, f64::max);
            if d.erase || longest == f64::NEG_INFINITY {
                return Vec::new();
            }
            bvh.faces_within(s, d.centre, d.radius + longest + REACH_MARGIN)
        })
        .collect();
    let mut around: Vec<u32> = around.concat();
    around.sort_unstable();
    around.dedup();
    let edges = edges_of(s, &around);
    let face = |e: &(u32, u32, u32)| around[e.2 as usize];

    // The candidates by groups connected over edges.
    let mut parent: Vec<u32> = (0..candidates.len() as u32).collect();
    for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let mut members = same.iter().filter_map(|e| candidate(face(e)));
        if let Some(first) = members.next() {
            for other in members {
                join(&mut parent, first as u32, other as u32);
            }
        }
    }
    // Per group: does it touch the Stelle (`touches`), or something else or an open edge of
    // the surface (`leaks`)?
    let mut touches = vec![false; candidates.len()];
    let mut leaks = vec![false; candidates.len()];
    for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let mut groups: Vec<u32> = same
            .iter()
            .filter_map(|e| candidate(face(e)))
            .map(|i| find(&mut parent, i as u32))
            .collect();
        if groups.is_empty() {
            continue;
        }
        groups.sort_unstable();
        groups.dedup();
        let (mut stelle, mut other) = (false, same.len() == 1);
        for e in same.iter().filter(|e| candidate(face(e)).is_none()) {
            if selected.binary_search(&face(e)).is_ok() {
                stelle = true;
            } else {
                other = true;
            }
        }
        for g in groups {
            touches[g as usize] |= stelle;
            leaks[g as usize] |= other;
        }
    }
    (0..candidates.len())
        .filter(|&i| {
            let g = find(&mut parent, i as u32) as usize;
            touches[g] && !leaks[g]
        })
        .map(|i| candidates[i])
        .collect()
}

/// The edges of `faces`, sorted: `(a, b, i)` with the welded corners `a < b` of an edge of
/// `faces[i]`; an edge whose corners are one vertex is left out.
fn edges_of(s: &WeldedSurface, faces: &[u32]) -> Vec<(u32, u32, u32)> {
    let mut edges = Vec::with_capacity(3 * faces.len());
    for (i, &f) in faces.iter().enumerate() {
        let t = s.faces()[f as usize];
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if a != b {
                edges.push((a.min(b), a.max(b), i as u32));
            }
        }
    }
    edges.sort_unstable();
    edges
}

/// The longest edge of a face (m): its size.
fn longest_edge(s: &WeldedSurface, f: u32) -> f64 {
    let [a, b, c] = s.face_corners(f);
    [(a, b), (b, c), (c, a)]
        .map(|(p, q)| dot(sub(p, q), sub(p, q)).sqrt())
        .into_iter()
        .fold(0.0, f64::max)
}

/// The root of `x`'s set (path halving).
fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        let up = parent[parent[x as usize] as usize];
        parent[x as usize] = up;
        x = up;
    }
    x
}

/// Joins the sets of `a` and `b`; the smaller root stays.
fn join(parent: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    parent[ra.max(rb) as usize] = ra.min(rb);
}

#[cfg(test)]
mod tests {
    use super::super::bvh::point_triangle_dist2;
    use super::super::{dot, sub};
    use super::*;
    use crate::surface::testutil::{concat, grid, paged, speck, wall_with_hole, Lcg};

    /// A dab where the ray `o + t·d` hits, with the normal from [`dab_normal`].
    fn dab_at(
        s: &WeldedSurface,
        bvh: &FaceBvh,
        o: [f64; 3],
        d: [f64; 3],
        radius: f64,
        erase: bool,
    ) -> SelectDab {
        let hit = bvh
            .raycast(s, o, d, None)
            .expect("the ray hits the surface");
        SelectDab {
            centre: hit.pos,
            normal: dab_normal(s, bvh, &hit, radius),
            radius,
            erase,
        }
    }

    fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
        let v = sub(a, b);
        dot(v, v).sqrt()
    }

    /// Angle of a face's normal to `+x` (degrees).
    fn tilt(s: &WeldedSurface, f: u32) -> f64 {
        s.face_normal(f)[0].clamp(-1.0, 1.0).acos().to_degrees()
    }

    /// The wall `x ≈ 0`, `0 ≤ y, z ≤ n·step`, normals `+x`, every vertex moved by `+a` or
    /// `−a` along x at random: neighbouring faces tilt by up to `atan(2√2·a / step)`.
    fn noisy_wall(n: usize, step: f64, a: f64, seed: u64) -> WeldedSurface {
        let mut rng = Lcg::new(seed);
        let (positions, indices) = grid([n, n], |i, j| {
            let x = if rng.unit() < 0.5 { -a } else { a };
            [x, i as f64 * step, j as f64 * step]
        });
        WeldedSurface::from_mesh(&paged(&positions, &indices, 17, 3))
    }

    /// Up to 30° of noise on the normals, with a dab of 20 cm: at most 2 % of the faces
    /// whose centroid lies within 18 cm of the dab are left out.
    #[test]
    fn selection_on_noisy_wall_has_no_holes() {
        let step = 0.02;
        let a = step * 30f64.to_radians().tan() / (2.0 * 2f64.sqrt());
        let s = noisy_wall(100, step, a, 3);
        let tilts: Vec<f64> = (0..s.face_count() as u32).map(|f| tilt(&s, f)).collect();
        let max = tilts.iter().copied().fold(0.0, f64::max);
        let steep = tilts.iter().filter(|&&t| t > 29.9).count();
        assert!(max < 30.001 && max > 29.9, "{max}");
        assert!(steep * 5 > tilts.len(), "{steep} of {}", tilts.len());
        let bvh = FaceBvh::build(&s);
        for (y, z) in [
            (1.0, 1.0),
            (0.5, 0.5),
            (1.5, 0.7),
            (0.31, 1.69),
            (1.234, 0.567),
        ] {
            let dab = dab_at(&s, &bvh, [1.0, y, z], [-1.0, 0.0, 0.0], 0.2, false);
            assert!(dab.normal[0] > 0.99, "{:?}", dab.normal);
            let chosen = select_faces(&s, &bvh, &[dab]);
            let inner: Vec<u32> = (0..s.face_count() as u32)
                .filter(|&f| dist(s.face_centroid(f), dab.centre) <= 0.18)
                .collect();
            let missed = inner
                .iter()
                .filter(|f| chosen.binary_search(f).is_err())
                .count();
            assert!(inner.len() > 400, "{}", inner.len());
            assert!(
                missed * 50 <= inner.len(),
                "({y}, {z}): {missed} of {} missed",
                inner.len()
            );
            for &f in &chosen {
                assert!(point_triangle_dist2(dab.centre, s.face_corners(f)) <= 0.2 * 0.2);
            }
        }
    }

    /// A slab 5 cm thick: dabs on its front never reach the back, not even 5 cm below the top
    /// or 4 cm from the side, where the sphere reaches round the edge. Two walls at 90°: a dab
    /// 5 cm from the corner stays on its wall. A panel in front of a wall, facing the same way:
    /// a dab stays on the one it hit.
    #[test]
    fn selection_stays_on_its_side() {
        // Front x = 0.05 (normals +x), back x = 0 (normals −x), 2 × 2 m.
        let slab = wall_with_hole(80, 0.025, None);
        let positions: Vec<[f64; 3]> = slab.positions.iter().map(|p| p.map(f64::from)).collect();
        let s = WeldedSurface::from_mesh(&paged(&positions, &slab.indices, 11, 4));
        let bvh = FaceBvh::build(&s);
        for (y, z) in [(1.0, 1.0), (1.03, 1.95), (0.04, 0.5)] {
            let dab = dab_at(&s, &bvh, [1.0, y, z], [-1.0, 0.0, 0.0], 0.2, false);
            assert!(dab.normal[0] > 0.999, "{:?}", dab.normal);
            let chosen = select_faces(&s, &bvh, &[dab]);
            assert!(chosen.len() > 100, "{}", chosen.len());
            for &f in &chosen {
                let n = s.face_normal(f);
                assert!(n[0] > 0.999, "({y}, {z}): face {f} faces {n:?}");
            }
            // The back is in the sphere, though.
            let back = bvh
                .faces_within(&s, dab.centre, 0.2)
                .into_iter()
                .filter(|&f| s.face_normal(f)[0] < -0.999)
                .count();
            assert!(back > 100, "{back}");
        }

        // Wall A: x = 0 (normals +x); wall B: y = 0 (normals +y); 2 × 2 m, the line x = y = 0
        // shared.
        let a = grid([50, 50], |i, j| [0.0, i as f64 * 0.04, j as f64 * 0.04]);
        let b = grid([50, 50], |i, j| [j as f64 * 0.04, 0.0, i as f64 * 0.04]);
        let (positions, indices) = concat(&[a, b]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 9, 3));
        assert_eq!(s.positions().len(), positions.len() - 51);
        let bvh = FaceBvh::build(&s);
        for (o, d, axis) in [
            ([1.0, 0.05, 1.0], [-1.0, 0.0, 0.0], 0),
            ([0.05, 1.0, 1.0], [0.0, -1.0, 0.0], 1),
        ] {
            let dab = dab_at(&s, &bvh, o, d, 0.2, false);
            assert!(f64::from(dab.normal[axis]) > 0.999, "{:?}", dab.normal);
            let chosen = select_faces(&s, &bvh, &[dab]);
            for &f in &chosen {
                assert!(s.face_normal(f)[axis] > 0.999, "face {f} of the other wall");
            }
            // The faces along the corner on its own wall are in.
            let along = chosen
                .iter()
                .filter(|&&f| s.face_centroid(f)[1 - axis] < 0.04)
                .count();
            assert!(along >= 10, "{along}");
            let other = bvh
                .faces_within(&s, dab.centre, 0.2)
                .into_iter()
                .filter(|&f| s.face_normal(f)[1 - axis] > 0.999)
                .count();
            assert!(other > 20, "{other}");
        }

        // A panel 3 cm in front of a wall, facing the same way, not joined to it: a dab on
        // the panel stays on the panel, a dab on the wall beside it stays on the wall.
        let wall = grid([50, 50], |i, j| [0.0, i as f64 * 0.04, j as f64 * 0.04]);
        let panel = grid([10, 10], |i, j| {
            [0.03, 0.85 + i as f64 * 0.03, 0.85 + j as f64 * 0.03]
        });
        let (positions, indices) = concat(&[wall, panel]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 9, 2));
        let bvh = FaceBvh::build(&s);
        for (y, on_panel) in [(1.0, true), (1.3, false)] {
            let dab = dab_at(&s, &bvh, [1.0, y, 1.0], [-1.0, 0.0, 0.0], 0.2, false);
            let chosen = select_faces(&s, &bvh, &[dab]);
            assert!(chosen.len() > 100, "{}", chosen.len());
            for &f in &chosen {
                assert_eq!(s.face_centroid(f)[0] > 0.02, on_panel, "face {f}");
            }
            let near = bvh.faces_within(&s, dab.centre, 0.2);
            let both = near
                .iter()
                .filter(|&&f| s.face_centroid(f)[0] > 0.02)
                .count();
            assert!(both > 10 && both < near.len(), "{both} of {}", near.len());
        }
    }

    /// Erasing dabs take their faces out again, in the order of the dabs.
    #[test]
    fn erase_removes() {
        let (positions, indices) = grid([100, 100], |i, j| [0.0, i as f64 * 0.02, j as f64 * 0.02]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 13, 2));
        let bvh = FaceBvh::build(&s);
        let dab = |y: f64, r: f64, erase: bool| SelectDab {
            centre: [0.0, y, 1.0],
            normal: [1.0, 0.0, 0.0],
            radius: r,
            erase,
        };
        let paint = dab(1.0, 0.2, false);
        let hole = dab(1.0, 0.1, true);
        let all = select_faces(&s, &bvh, &[paint]);
        let gone = dab_faces(&s, &bvh, &hole);
        assert!(gone.len() > 100 && all.len() > 3 * gone.len());
        let rest: Vec<u32> = all
            .iter()
            .copied()
            .filter(|f| gone.binary_search(f).is_err())
            .collect();
        assert_eq!(select_faces(&s, &bvh, &[paint, hole]), rest);
        // Erasing first, painting after: all stays. Painting into the hole fills it again.
        assert_eq!(select_faces(&s, &bvh, &[hole, paint]), all);
        let fill = dab(1.0, 0.1, false);
        assert_eq!(select_faces(&s, &bvh, &[paint, hole, fill]), all);
        // An erasing dab alone selects nothing; a larger one takes everything.
        assert!(select_faces(&s, &bvh, &[hole]).is_empty());
        assert!(select_faces(&s, &bvh, &[paint, dab(1.0, 0.3, true)]).is_empty());
        // Two dabs, the second erased: the first stays as it was.
        let other = dab(1.5, 0.1, false);
        let first = select_faces(&s, &bvh, &[other]);
        let both = select_faces(&s, &bvh, &[other, paint, dab(1.0, 0.21, true)]);
        assert_eq!(both, first);
        assert!(select_faces(&s, &bvh, &[]).is_empty());
    }

    /// Weld, tree, dab normals and a stroke of 46 dabs (every fifth erasing) with 1 thread, 8
    /// threads and the default pool: the same bytes.
    #[test]
    fn selection_is_deterministic() {
        let step = 0.02;
        let a = step * 30f64.to_radians().tan() / (2.0 * 2f64.sqrt());
        let mut rng = Lcg::new(11);
        let (positions, indices) = grid([100, 100], |i, j| {
            let x = if rng.unit() < 0.5 { -a } else { a };
            [x, i as f64 * step, j as f64 * step]
        });
        let mesh = paged(&positions, &indices, 17, 3);
        let run = || {
            let s = WeldedSurface::from_mesh(&mesh);
            let bvh = FaceBvh::build(&s);
            let dabs: Vec<SelectDab> = (0..46)
                .map(|i| {
                    let p = 0.2 + i as f64 * 0.035;
                    let r = if i % 5 == 4 { 0.08 } else { 0.2 };
                    dab_at(&s, &bvh, [1.0, p, p], [-1.0, 0.0, 0.0], r, i % 5 == 4)
                })
                .collect();
            let chosen = select_faces(&s, &bvh, &dabs);
            let normals: Vec<[u32; 3]> = dabs.iter().map(|d| d.normal.map(f32::to_bits)).collect();
            (s.digest(), bvh.digest(), normals, chosen)
        };
        let pool = |threads| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
        };
        let one = pool(1).install(run);
        let many = pool(8).install(run);
        let default = run();
        assert!(one.3.len() > 1000, "{}", one.3.len());
        assert_eq!(one, many);
        assert_eq!(one, default);
    }

    /// A dab without a direction selects by distance alone; one off the surface or with a
    /// radius that is no number selects nothing.
    #[test]
    fn odd_dabs() {
        let (positions, indices) = grid([20, 20], |i, j| [0.0, i as f64 * 0.05, j as f64 * 0.05]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 7, 2));
        let bvh = FaceBvh::build(&s);
        let dab = |normal: [f32; 3], centre: [f64; 3], radius: f64| SelectDab {
            centre,
            normal,
            radius,
            erase: false,
        };
        let plain = dab_faces(&s, &bvh, &dab([1.0, 0.0, 0.0], [0.0, 0.5, 0.5], 0.2));
        assert!(plain.len() > 50);
        assert_eq!(
            dab_faces(&s, &bvh, &dab([0.0; 3], [0.0, 0.5, 0.5], 0.2)),
            plain
        );
        assert_eq!(
            dab_faces(&s, &bvh, &dab([2.0, 0.0, 0.0], [0.0, 0.5, 0.5], 0.2)),
            plain
        );
        // Facing the other way: no face of this wall agrees.
        assert!(dab_faces(&s, &bvh, &dab([-1.0, 0.0, 0.0], [0.0, 0.5, 0.5], 0.2)).is_empty());
        assert!(dab_faces(&s, &bvh, &dab([1.0, 0.0, 0.0], [5.0, 0.5, 0.5], 0.2)).is_empty());
        assert!(dab_faces(&s, &bvh, &dab([1.0, 0.0, 0.0], [0.0, 0.5, 0.5], f64::NAN)).is_empty());
        assert!(dab_faces(&s, &bvh, &dab([1.0, 0.0, 0.0], [0.0, 0.5, 0.5], -1.0)).is_empty());
    }

    /// The faces sharing an edge with each face (all edges, sorted).
    fn edge_neighbours(s: &WeldedSurface) -> Vec<Vec<u32>> {
        let mut edges: Vec<(u32, u32, u32)> = Vec::new();
        for (f, t) in s.faces().iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                edges.push((a.min(b), a.max(b), f as u32));
            }
        }
        edges.sort_unstable();
        let mut out = vec![Vec::new(); s.face_count()];
        for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
            for a in same {
                out[a.2 as usize].extend(same.iter().filter(|b| b.2 != a.2).map(|b| b.2));
            }
        }
        out
    }

    /// The faces not in `chosen` with their centroid within `r` of `centre` whose three edge
    /// neighbours are all in `chosen`.
    fn inner_holes(
        s: &WeldedSurface,
        nbrs: &[Vec<u32>],
        chosen: &[u32],
        centre: [f64; 3],
        r: f64,
    ) -> Vec<u32> {
        let has = |f: &u32| chosen.binary_search(f).is_ok();
        (0..s.face_count() as u32)
            .filter(|f| !has(f) && dist(s.face_centroid(*f), centre) <= r)
            .filter(|&f| nbrs[f as usize].len() == 3 && nbrs[f as usize].iter().all(has))
            .collect()
    }

    /// A wall `x ≈ 0` of 2 × 2 m whose vertices are moved along x by up to `±a` at random,
    /// `a` such that an edge tilts by up to `edge_deg`.
    fn rough_wall(edge_deg: f64, seed: u64) -> WeldedSurface {
        let step = 0.02;
        let a = step * edge_deg.to_radians().tan() / 2.0;
        let mut rng = Lcg::new(seed);
        let (positions, indices) = grid([100, 100], |i, j| {
            [rng.range(-a, a), i as f64 * step, j as f64 * step]
        });
        WeldedSurface::from_mesh(&paged(&positions, &indices, 17, 3))
    }

    /// Normals scattered by about ±55° (edges tilt by up to 55°, faces by up to 64°): the
    /// normal gate leaves single faces inside a dab out. After all dabs no unselected face with
    /// all three edge neighbours selected is left within 2 cm inside the radius, and closing
    /// only adds faces within the radius.
    #[test]
    fn noisy_selection_has_no_inner_holes() {
        let s = rough_wall(55.0, 17);
        let bvh = FaceBvh::build(&s);
        let nbrs = edge_neighbours(&s);
        let mut before = 0;
        for (y, z) in [
            (1.0, 1.0),
            (0.5, 0.5),
            (1.5, 0.7),
            (0.31, 1.69),
            (1.234, 0.567),
        ] {
            let dab = dab_at(&s, &bvh, [1.0, y, z], [-1.0, 0.0, 0.0], 0.2, false);
            let raw = dab_faces(&s, &bvh, &dab);
            let chosen = select_faces(&s, &bvh, &[dab]);
            before += inner_holes(&s, &nbrs, &raw, dab.centre, 0.18).len();
            let left = inner_holes(&s, &nbrs, &chosen, dab.centre, 0.18);
            assert!(left.is_empty(), "({y}, {z}): holes {left:?}");
            assert!(raw.iter().all(|f| chosen.binary_search(f).is_ok()));
            for &f in &chosen {
                assert!(point_triangle_dist2(dab.centre, s.face_corners(f)) <= 0.2 * 0.2);
            }
        }
        assert!(before >= 5, "the gate left only {before} holes");
    }

    /// Holes are closed only where the Stelle encloses them. The reveal of a window opening
    /// meets the Stelle on one side and ends in an open edge of the surface on the other; a
    /// small closed object in front of the wall touches no face of the Stelle. Both lie wholly
    /// inside their dab and stay out.
    #[test]
    fn closing_needs_an_enclosed_hole() {
        let at = |k: usize| k as f64 * 0.02;
        // The wall x = 0 (normals +x), 2 × 2 m, with an opening at 0.9 ≤ y, z ≤ 1.1 ...
        let (positions, mut indices) = grid([100, 100], |i, j| [0.0, at(i), at(j)]);
        indices.retain(|t| {
            let c = |k: usize| t.iter().map(|&v| positions[v as usize][k]).sum::<f64>() / 3.0;
            !((0.9..1.1).contains(&c(1)) && (0.9..1.1).contains(&c(2)))
        });
        let mut parts = vec![(positions, indices)];
        // ... and its reveal, 4 cm deep, open at the back: x = 0 joins the wall's corners.
        for side in [45, 55] {
            parts.push(grid([10, 2], |i, k| [-at(k), at(45 + i), at(side)]));
            parts.push(grid([10, 2], |j, k| [-at(k), at(side), at(45 + j)]));
        }
        // A closed object 2 cm across, 1 cm in front of the wall.
        let object = speck([0.03, 0.5, 0.5], 0.02);
        parts.push((
            object.positions.iter().map(|p| p.map(f64::from)).collect(),
            object.indices.clone(),
        ));
        let (positions, indices) = concat(&parts);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 13, 3));
        let bvh = FaceBvh::build(&s);
        let dab = |centre: [f64; 3]| SelectDab {
            centre,
            normal: [1.0, 0.0, 0.0],
            radius: 0.2,
            erase: false,
        };
        for (centre, part) in [([0.0, 1.0, 1.0], "reveal"), ([0.0, 0.55, 0.5], "object")] {
            let near = bvh.faces_within(&s, centre, 0.2);
            let off = near
                .iter()
                .filter(|&&f| s.face_centroid(f)[0] != 0.0)
                .count();
            assert!(off >= 20, "{part}: {off} faces off the wall");
            let chosen = select_faces(&s, &bvh, &[dab(centre)]);
            assert!(chosen.len() > 300, "{part}: {}", chosen.len());
            for &f in &chosen {
                assert_eq!(
                    s.face_centroid(f)[0],
                    0.0,
                    "{part}: face {f} is off the wall"
                );
            }
        }
    }

    /// Painting, then erasing a small spot in the middle: the spot stays open. On a flat wall
    /// the painted faces enclose it, so closing must leave out the faces an erasing dab took;
    /// on a rough wall nothing inside the erasing dab's sphere is added; and painting the back
    /// of a thin wall behind the spot (that dab's sphere reaches through to it) brings none of
    /// its faces back.
    #[test]
    fn erased_hole_stays_open() {
        let (positions, indices) = grid([100, 100], |i, j| [0.0, i as f64 * 0.02, j as f64 * 0.02]);
        let flat = WeldedSurface::from_mesh(&paged(&positions, &indices, 13, 2));
        for (s, enclosed) in [(flat, true), (rough_wall(55.0, 23), false)] {
            let bvh = FaceBvh::build(&s);
            let paint = dab_at(&s, &bvh, [1.0, 1.0, 1.0], [-1.0, 0.0, 0.0], 0.2, false);
            let erase = dab_at(&s, &bvh, [1.0, 1.01, 0.99], [-1.0, 0.0, 0.0], 0.06, true);
            let painted = dab_faces(&s, &bvh, &paint);
            let gone = dab_faces(&s, &bvh, &erase);
            assert!(gone.len() > 20, "{}", gone.len());
            let chosen = select_faces(&s, &bvh, &[paint, erase]);
            for f in &gone {
                assert!(chosen.binary_search(f).is_err(), "erased face {f} is back");
            }
            for f in bvh.faces_within(&s, erase.centre, erase.radius) {
                if chosen.binary_search(&f).is_ok() {
                    assert!(painted.binary_search(&f).is_ok(), "face {f} was added");
                }
            }
            if enclosed {
                // Every face next to the erased spot is selected.
                let nbrs = edge_neighbours(&s);
                for &f in &gone {
                    for n in &nbrs[f as usize] {
                        let inside = gone.binary_search(n).is_ok();
                        assert!(inside || chosen.binary_search(n).is_ok(), "{n}");
                    }
                }
            }
        }

        // The slab 5 cm thick: front x = 0.05 (normals +x), back x = 0 (normals −x).
        let slab = wall_with_hole(80, 0.025, None);
        let positions: Vec<[f64; 3]> = slab.positions.iter().map(|p| p.map(f64::from)).collect();
        let s = WeldedSurface::from_mesh(&paged(&positions, &slab.indices, 11, 4));
        let bvh = FaceBvh::build(&s);
        let front = dab_at(&s, &bvh, [1.0, 1.0, 1.0], [-1.0, 0.0, 0.0], 0.2, false);
        let erase = dab_at(&s, &bvh, [1.0, 1.01, 0.99], [-1.0, 0.0, 0.0], 0.06, true);
        let back = dab_at(&s, &bvh, [-1.0, 1.0, 1.0], [1.0, 0.0, 0.0], 0.2, false);
        let gone = dab_faces(&s, &bvh, &erase);
        let behind = dab_faces(&s, &bvh, &back);
        assert!(gone.len() > 10 && behind.len() > 100);
        let reach = bvh.faces_within(&s, back.centre, back.radius);
        assert!(gone.iter().all(|f| reach.binary_search(f).is_ok()));
        let chosen = select_faces(&s, &bvh, &[front, erase, back]);
        for f in &gone {
            assert!(chosen.binary_search(f).is_err(), "erased face {f} is back");
        }
        assert!(behind.iter().all(|f| chosen.binary_search(f).is_ok()));
    }

    /// A bump: one vertex 4 cm out of a flat wall, so its six faces tilt by more than 60°. A
    /// dab leaves them out and closing takes them in. A small erasing dab on the bump takes
    /// nothing (the six fail its normal gate as well) but keeps the hole open: the last dab
    /// whose sphere holds them erases.
    #[test]
    fn erasing_over_a_hole_keeps_it_open() {
        let (positions, indices) = grid([100, 100], |i, j| {
            let x = if (i, j) == (50, 50) { 0.04 } else { 0.0 };
            [x, i as f64 * 0.02, j as f64 * 0.02]
        });
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 13, 2));
        let bvh = FaceBvh::build(&s);
        let bump: Vec<u32> = (0..s.face_count() as u32)
            .filter(|&f| s.face_normal(f)[0] < MIN_NORMAL_DOT)
            .collect();
        assert_eq!(bump.len(), 6);
        let dab = |centre: [f64; 3], radius: f64, erase: bool| SelectDab {
            centre,
            normal: [1.0, 0.0, 0.0],
            radius,
            erase,
        };
        let paint = dab([0.0, 1.05, 1.0], 0.2, false);
        let raw = dab_faces(&s, &bvh, &paint);
        assert!(bump.iter().all(|f| raw.binary_search(f).is_err()));
        let closed = select_faces(&s, &bvh, &[paint]);
        assert!(bump.iter().all(|f| closed.binary_search(f).is_ok()));
        assert_eq!(closed.len(), raw.len() + bump.len());

        let erase = dab([0.04, 1.0, 1.0], 0.005, true);
        assert_eq!(bvh.faces_within(&s, erase.centre, erase.radius), bump);
        assert!(dab_faces(&s, &bvh, &erase).is_empty());
        assert_eq!(select_faces(&s, &bvh, &[paint, erase]), raw);
    }
}
