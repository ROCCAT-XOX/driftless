//! Smoothing a Stelle: „leicht“, „stark“ and „ganz eben“.
//!
//! [`smooth_region`] moves the welded vertices of a Stelle's faces; which vertex belongs to
//! which face stays, and so does everything outside the Stelle.
//!
//! - **Border.** A vertex the Stelle shares with a face outside it never moves: the faces
//!   outside keep their place and the surface stays closed. Inside, a vertex takes the share
//!   `w` of its shift, growing linearly with its distance to the nearest border vertex from 0
//!   at the border to 1 at 10 cm: the Stelle goes over into its surroundings without an edge
//!   or a step. A vertex at `p` with the target `t` moves to `p + w · (t − p)`.
//! - **„Leicht“ and „stark“** are Taubin's λ|μ smoothing over the 1-ring (uniform weights, the
//!   border held): a step λ = 0.5 towards the mean of the neighbours, then one of μ = −0.505
//!   back. It takes out the bumps from vertex to vertex and keeps the shape: what varies more
//!   slowly than about every 25 edges keeps its size (1/λ + 1/μ = 0.02). „Leicht“ runs 5
//!   rounds and halves bumps about 4 edges long; „stark“ runs 150 rounds, halves bumps about 9
//!   edges long and takes out those up to 6. Noise from vertex to vertex goes down by about
//!   half and four fifths. (With μ further from −λ the band that keeps its size is wider, but
//!   it grows a little every round: by 14 % over 150 rounds at μ = −0.53.) A vertex on an open
//!   edge of the surface (a hole, the end of the scan) only follows its neighbours along that
//!   edge: the edge is smoothed along itself and keeps its place.
//! - **„Ganz eben“** puts the vertices onto the plane through all vertices of the Stelle (the
//!   least spread of their covariance). If the Stelle does not lie on a plane, it refuses and
//!   nothing moves ([`EditError::NotFlat`]): the vertices are farther than 3 cm (RMS) from the
//!   plane, or their least spread is above 5 % of the middle one (as the planarity of the
//!   cloud smoothing, `SmoothConfig::planarity`).
//!
//! The new positions are rounded to f32, as the glb holds them; a vertex whose shift rounds
//! away has not moved. The smoothing reaches as far in edges on every surface, so on a
//! coarser surface it reaches further in metres.
//!
//! Deterministic: f64, a fixed number of rounds, every vertex of a step from the positions of
//! the step before (so rayon may work on the vertices in any order), sums in a fixed order,
//! sorts on unique keys, no hash map. The plane comes from `spread_axes`, nalgebra's symmetric
//! eigen decomposition: only `+ − × ÷` and `√`, which give the same bits on every machine.

use super::{dot, sub, EditError, WeldedSurface};
use crate::clean::spread_axes;
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;

/// Distance from the border of a Stelle (m) from which on a vertex takes its full shift.
const RAMP: f64 = 0.10;

/// Taubin's step towards the mean of the neighbours ...
const LAMBDA: f64 = 0.5;

/// ... and back, a little further: what varies slowly (1/λ + 1/μ = 0.02) keeps its size.
const MU: f64 = -0.505;

/// Rounds of λ and μ for „leicht“.
const LIGHT_ROUNDS: usize = 5;

/// Rounds of λ and μ for „stark“.
const STRONG_ROUNDS: usize = 150;

/// „Ganz eben“ refuses a Stelle whose vertices lie farther than this from their plane (RMS,
/// m) ...
const FLAT_MAX_RMS: f64 = 0.03;

/// ... or whose least spread is above this share of the middle one.
const FLAT_MAX_SPREAD: f64 = 0.05;

/// Vertices per piece of work for rayon.
const CHUNK: usize = 1024;

/// How a Stelle is smoothed (the steps crate's `SmoothLevel` becomes this).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Smoothing {
    /// „leicht“: the roughness from vertex to vertex.
    Light,
    /// „stark“: bumps of a few edges, too.
    Strong,
    /// „ganz eben“: onto one plane, for walls, floor and ceiling.
    Flat,
}

/// Smooths the Stelle `faces` of `s` the way `how` says (see the module docs): the vertices
/// that move and where to, sorted by vertex. `faces` may come in any order; a face named twice
/// counts once.
///
/// # Errors
/// [`EditError::NoFaces`] without a face; [`EditError::NotFlat`] if `how` is
/// [`Smoothing::Flat`] and the Stelle does not lie on a plane.
///
/// # Panics
/// If a face is not one of `s`.
pub fn smooth_region(
    s: &WeldedSurface,
    faces: &[u32],
    how: Smoothing,
) -> Result<Vec<(u32, [f64; 3])>, EditError> {
    let region = Region::new(s, faces)?;
    let targets = match how {
        Smoothing::Light => region.taubin(LIGHT_ROUNDS),
        Smoothing::Strong => region.taubin(STRONG_ROUNDS),
        Smoothing::Flat => region.plane()?,
    };
    Ok(region.moves(&targets))
}

/// The vertices of a Stelle, with the neighbours they are smoothed towards.
struct Region {
    /// The welded vertices of the Stelle's faces, sorted; the region's index of a vertex is
    /// its place here.
    verts: Vec<u32>,
    /// Their positions.
    pos: Vec<[f64; 3]>,
    /// Whether a face outside the Stelle uses the vertex too: it stays.
    border: Vec<bool>,
    /// The neighbours vertex `i` is smoothed towards are `nbrs[start[i]..start[i + 1]]`
    /// (indices, ascending): along the open edges of the surface if it lies on one, else all
    /// of its 1-ring; none for a border vertex.
    start: Vec<u32>,
    nbrs: Vec<u32>,
}

impl Region {
    fn new(s: &WeldedSurface, faces: &[u32]) -> Result<Region, EditError> {
        let mut faces = faces.to_vec();
        faces.sort_unstable();
        faces.dedup();
        let Some(&last) = faces.last() else {
            return Err(EditError::NoFaces);
        };
        assert!(
            (last as usize) < s.face_count(),
            "face {last} of {}",
            s.face_count()
        );
        let corners = s.faces();
        let mut verts: Vec<u32> = faces.iter().flat_map(|&f| corners[f as usize]).collect();
        verts.sort_unstable();
        verts.dedup();
        let pos: Vec<[f64; 3]> = verts.iter().map(|&v| s.positions()[v as usize]).collect();

        // The border: vertices of the Stelle that a face outside it uses too.
        let mut in_stelle = vec![false; corners.len()];
        for &f in &faces {
            in_stelle[f as usize] = true;
        }
        let mut of_stelle = vec![false; s.positions().len()];
        for &v in &verts {
            of_stelle[v as usize] = true;
        }
        let mut shared = vec![false; s.positions().len()];
        for (tri, &own) in corners.iter().zip(&in_stelle) {
            if !own {
                for &v in tri {
                    if of_stelle[v as usize] {
                        shared[v as usize] = true;
                    }
                }
            }
        }
        let border: Vec<bool> = verts.iter().map(|&v| shared[v as usize]).collect();

        // Every edge of every face once, as (smaller, larger) region index. A vertex off the
        // border has all its faces in the Stelle, so for its edges the faces counted here
        // are all faces: an edge of one face only is an open edge of the surface.
        let index = |v: u32| verts.binary_search(&v).expect("a corner of the Stelle") as u32;
        let mut edges: Vec<(u32, u32)> = Vec::with_capacity(3 * faces.len());
        for &f in &faces {
            let [a, b, c] = corners[f as usize].map(index);
            let mut sides = [(a, b), (b, c), (c, a)].map(|(x, y)| (x.min(y), x.max(y)));
            sides.sort_unstable();
            for (k, &(x, y)) in sides.iter().enumerate() {
                // A face with a corner twice names one edge twice and one from a corner to
                // itself.
                if x != y && (k == 0 || sides[k - 1] != (x, y)) {
                    edges.push((x, y));
                }
            }
        }
        edges.sort_unstable();
        // (vertex, neighbour, open), both ways round.
        let mut halves: Vec<(u32, u32, bool)> = Vec::with_capacity(2 * edges.len());
        for same in edges.chunk_by(|x, y| x == y) {
            let ((x, y), open) = (same[0], same.len() == 1);
            halves.push((x, y, open));
            halves.push((y, x, open));
        }
        halves.sort_unstable();

        let mut start = Vec::with_capacity(verts.len() + 1);
        let mut nbrs = Vec::with_capacity(halves.len());
        start.push(0);
        let mut by_vertex = halves.chunk_by(|x, y| x.0 == y.0).peekable();
        for i in 0..verts.len() as u32 {
            let Some(own) = by_vertex.next_if(|own| own[0].0 == i) else {
                start.push(nbrs.len() as u32);
                continue;
            };
            if !border[i as usize] {
                let on_open_edge = own.iter().any(|h| h.2);
                let along = own.iter().filter(|h| h.2 || !on_open_edge);
                nbrs.extend(along.map(|h| h.1));
            }
            start.push(nbrs.len() as u32);
        }
        Ok(Region {
            verts,
            pos,
            border,
            start,
            nbrs,
        })
    }

    /// Where Taubin's smoothing takes every vertex in `rounds` rounds of λ and μ, the border
    /// held.
    fn taubin(&self, rounds: usize) -> Vec<[f64; 3]> {
        let mut now = self.pos.clone();
        let mut next = self.pos.clone();
        for _ in 0..rounds {
            for factor in [LAMBDA, MU] {
                next.par_iter_mut()
                    .enumerate()
                    .with_min_len(CHUNK)
                    .for_each(|(i, out)| {
                        let nbrs = &self.nbrs[self.start[i] as usize..self.start[i + 1] as usize];
                        let p = now[i];
                        if nbrs.is_empty() {
                            *out = p;
                            return;
                        }
                        // The mean offset to the neighbours.
                        let mut sum = [0.0; 3];
                        for &j in nbrs {
                            for (sum, d) in sum.iter_mut().zip(sub(now[j as usize], p)) {
                                *sum += d;
                            }
                        }
                        let n = nbrs.len() as f64;
                        *out = [0, 1, 2].map(|k| p[k] + factor * (sum[k] / n));
                    });
                std::mem::swap(&mut now, &mut next);
            }
        }
        now
    }

    /// The vertices projected onto the plane through all of them, or [`EditError::NotFlat`]
    /// if they do not lie on a plane (see the module docs).
    fn plane(&self) -> Result<Vec<[f64; 3]>, EditError> {
        let n = self.pos.len() as f64;
        let mut mean = [0.0; 3];
        for p in &self.pos {
            for (m, c) in mean.iter_mut().zip(p) {
                *m += c;
            }
        }
        let mean = mean.map(|m| m / n);
        let mut cov = Matrix3::zeros();
        for &p in &self.pos {
            let d = Vector3::from(sub(p, mean));
            cov += d * d.transpose();
        }
        let [least, middle, _] = spread_axes(&(cov / n));
        // The spread along the unit normal is the mean squared distance to the plane.
        let rms = least.0.max(0.0).sqrt();
        if !(rms <= FLAT_MAX_RMS && least.0 <= FLAT_MAX_SPREAD * middle.0) {
            return Err(EditError::NotFlat { rms_m: rms });
        }
        let normal: [f64; 3] = least.1.into();
        Ok(self
            .pos
            .iter()
            .map(|&p| {
                let h = dot(sub(p, mean), normal);
                [0, 1, 2].map(|k| p[k] - h * normal[k])
            })
            .collect())
    }

    /// Per vertex the share of its shift it takes: 0 on the border, then growing linearly
    /// with the distance to the nearest border vertex, 1 from [`RAMP`] on.
    fn weights(&self) -> Vec<f64> {
        // The border vertices by cubes of RAMP: those within RAMP of a vertex lie in its cube
        // or one of the 26 around it.
        let cube = |p: &[f64; 3]| p.map(|c| (c / RAMP).floor() as i64);
        let mut cubes: Vec<([i64; 3], u32)> = (0..self.pos.len())
            .filter(|&i| self.border[i])
            .map(|i| (cube(&self.pos[i]), i as u32))
            .collect();
        cubes.sort_unstable();
        self.pos
            .par_iter()
            .zip(&self.border)
            .with_min_len(CHUNK)
            .map(|(p, &border)| {
                if border {
                    return 0.0;
                }
                let c = cube(p);
                let mut nearest = RAMP * RAMP;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            let key = [c[0] + dx, c[1] + dy, c[2] + dz];
                            let from = cubes.partition_point(|e| e.0 < key);
                            for &(_, j) in cubes[from..].iter().take_while(|e| e.0 == key) {
                                let d = sub(*p, self.pos[j as usize]);
                                nearest = nearest.min(dot(d, d));
                            }
                        }
                    }
                }
                nearest.sqrt() / RAMP
            })
            .collect()
    }

    /// The vertices off the border that `targets` move, with their new positions (rounded to
    /// f32), sorted by vertex.
    fn moves(&self, targets: &[[f64; 3]]) -> Vec<(u32, [f64; 3])> {
        let weights = self.weights();
        (0..self.verts.len())
            .filter(|&i| !self.border[i])
            .filter_map(|i| {
                let (p, t, w) = (self.pos[i], targets[i], weights[i]);
                let q = [0, 1, 2].map(|k| f64::from((p[k] + w * (t[k] - p[k])) as f32));
                (q != p).then_some((self.verts[i], q))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::super::{dot, sub};
    use super::*;
    use crate::surface::testutil::{concat, grid, paged, Lcg};
    use driftless_export::mesh::Mesh;

    use Smoothing::{Flat, Light, Strong};

    /// Squares of the noisy floor (m).
    const STEP: f64 = 0.01;
    /// Noise of the noisy floor (m).
    const SIGMA: f64 = 0.005;
    /// Where the shift is full (m from the border), as the module promises.
    const RAMP: f64 = 0.10;

    /// `p` as the glb holds it (f32), like every surface the touch-up gets.
    fn f32_exact(p: [f64; 3]) -> [f64; 3] {
        p.map(|c| f64::from(c as f32))
    }

    fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
        let d = sub(a, b);
        dot(d, d).sqrt()
    }

    /// The floor z = 0 over 1 × 1 m in squares of 1 cm, every vertex lifted by noise of
    /// σ = 5 mm, paged as the surface step does it (charts of 37 faces on 5 pages).
    fn noisy_floor(seed: u64) -> (Mesh, WeldedSurface) {
        let mut rng = Lcg::new(seed);
        let (positions, indices) = grid([100, 100], |i, j| {
            f32_exact([i as f64 * STEP, j as f64 * STEP, SIGMA * rng.normal()])
        });
        let mesh = paged(&positions, &indices, 37, 5);
        let s = WeldedSurface::from_mesh(&mesh);
        (mesh, s)
    }

    /// The faces whose centroid lies within `r` of `centre`.
    fn near(s: &WeldedSurface, centre: [f64; 3], r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| dist(s.face_centroid(f), centre) <= r)
            .collect()
    }

    /// The vertices of `faces`, and of those the border: the ones a face outside uses too.
    fn region(s: &WeldedSurface, faces: &[u32]) -> (Vec<u32>, Vec<u32>) {
        let mut inside = vec![false; s.face_count()];
        for &f in faces {
            inside[f as usize] = true;
        }
        let mut verts: Vec<u32> = faces.iter().flat_map(|&f| s.faces()[f as usize]).collect();
        verts.sort_unstable();
        verts.dedup();
        let mut border: Vec<u32> = (0..s.face_count())
            .filter(|&f| !inside[f])
            .flat_map(|f| s.faces()[f])
            .filter(|v| verts.binary_search(v).is_ok())
            .collect();
        border.sort_unstable();
        border.dedup();
        (verts, border)
    }

    /// Distance from `p` to the nearest vertex of `border` (infinite without a border).
    fn to_border(s: &WeldedSurface, border: &[u32], p: [f64; 3]) -> f64 {
        border
            .iter()
            .map(|&v| dist(s.positions()[v as usize], p))
            .fold(f64::INFINITY, f64::min)
    }

    /// The positions of all vertices after `moved`.
    fn positions_after(s: &WeldedSurface, moved: &[(u32, [f64; 3])]) -> Vec<[f64; 3]> {
        let mut pos = s.positions().to_vec();
        for &(v, p) in moved {
            pos[v as usize] = p;
        }
        pos
    }

    /// RMS of z over the vertices of the disk r = 0.3 m in the middle of `noisy_floor(seed)`
    /// at least 0.15 m from its border, before and after smoothing the disk `how`.
    fn calming(seed: u64, how: Smoothing) -> (f64, f64) {
        let (_, s) = noisy_floor(seed);
        let faces = near(&s, [0.5, 0.5, 0.0], 0.3);
        let (verts, border) = region(&s, &faces);
        let inner: Vec<u32> = verts
            .iter()
            .copied()
            .filter(|&v| to_border(&s, &border, s.positions()[v as usize]) >= 0.15)
            .collect();
        assert!(inner.len() > 500, "{}", inner.len());
        let moved = smooth_region(&s, &faces, how).unwrap();
        let rms = |pos: &[[f64; 3]]| {
            let sum: f64 = inner.iter().map(|&v| pos[v as usize][2].powi(2)).sum();
            (sum / inner.len() as f64).sqrt()
        };
        let (before, after) = (rms(s.positions()), rms(&positions_after(&s, &moved)));
        assert!((0.9 * SIGMA..1.1 * SIGMA).contains(&before), "{before}");
        println!(
            "{how:?}, seed {seed}: RMS {:.3} mm → {:.3} mm ({:.1} % less)",
            1e3 * before,
            1e3 * after,
            100.0 * (1.0 - after / before)
        );
        (before, after)
    }

    #[test]
    fn light_calms() {
        for seed in 1..=3 {
            let (before, after) = calming(seed, Light);
            assert!(after <= 0.6 * before, "seed {seed}: {before} → {after}");
        }
    }

    #[test]
    fn strong_calms_more() {
        for seed in 1..=3 {
            let (before, after) = calming(seed, Strong);
            assert!(after <= 0.3 * before, "seed {seed}: {before} → {after}");
            assert!(after < calming(seed, Light).1, "seed {seed}");
        }
    }

    #[test]
    fn flat_is_flat() {
        for seed in 1..=3 {
            let (_, after) = calming(seed, Flat);
            assert!(after <= 0.0005, "seed {seed}: {after}");
        }
    }

    /// Only vertices of the Stelle move, and none on its border; every vertex once, sorted.
    #[test]
    fn border_and_outside_untouched() {
        let (_, s) = noisy_floor(4);
        let faces = near(&s, [0.5, 0.5, 0.0], 0.3);
        let (verts, border) = region(&s, &faces);
        assert!(border.len() > 100, "{}", border.len());
        for how in [Light, Strong, Flat] {
            let moved = smooth_region(&s, &faces, how).unwrap();
            assert!(moved.len() > verts.len() / 2, "{how:?}: {}", moved.len());
            assert!(moved.windows(2).all(|w| w[0].0 < w[1].0), "{how:?}");
            for &(v, p) in &moved {
                assert!(verts.binary_search(&v).is_ok(), "{how:?}: {v} is outside");
                assert!(
                    border.binary_search(&v).is_err(),
                    "{how:?}: {v} is on the border"
                );
                assert_ne!(p, s.positions()[v as usize], "{how:?}: {v} did not move");
                assert_eq!(p, f32_exact(p), "{how:?}: {v} not as the glb holds it");
            }
        }
    }

    /// The shift fades out towards the border, so there is no step: „ganz eben“ moves a vertex
    /// at a distance d from the border by at most min(d / 10 cm, 1) of its distance to the
    /// plane; „leicht“ and „stark“ move the vertices within 2 cm of the border by less than a
    /// quarter of what they move those 10 cm in and more.
    #[test]
    fn border_is_soft() {
        let (_, s) = noisy_floor(5);
        let faces = near(&s, [0.5, 0.5, 0.0], 0.3);
        let (_, border) = region(&s, &faces);
        let flat = smooth_region(&s, &faces, Flat).unwrap();
        for &(v, p) in &flat {
            let old = s.positions()[v as usize];
            let d = to_border(&s, &border, old);
            // The fitted plane lies within a millimetre of z = 0.
            let most = (d / RAMP).min(1.0) * (old[2].abs() + 0.001) + 1e-6;
            assert!(dist(p, old) <= most, "{v} at {d} m: {}", dist(p, old));
        }
        for how in [Light, Strong] {
            let moved = smooth_region(&s, &faces, how).unwrap();
            let (mut near_border, mut inside) = (Vec::new(), Vec::new());
            for &(v, p) in &moved {
                let old = s.positions()[v as usize];
                let d = to_border(&s, &border, old);
                if d < 0.02 {
                    near_border.push(dist(p, old));
                } else if d >= RAMP {
                    inside.push(dist(p, old));
                }
            }
            let mean = |x: &[f64]| x.iter().sum::<f64>() / x.len() as f64;
            assert!(near_border.len() > 100 && inside.len() > 1000, "{how:?}");
            let (edge, full) = (mean(&near_border), mean(&inside));
            assert!(
                edge < 0.25 * full,
                "{how:?}: {edge} near the border, {full} inside"
            );
        }
    }

    /// The moves are per welded vertex: given to every copy on the pages, the surface welds
    /// into the same vertices and faces as before, at the new places.
    #[test]
    fn stays_closed_across_pages() {
        let (mesh, s) = noisy_floor(6);
        let faces = near(&s, [0.5, 0.5, 0.0], 0.3);
        for how in [Light, Strong, Flat] {
            let moved = smooth_region(&s, &faces, how).unwrap();
            // Many moved vertices lie on a chart border, with copies on several pages.
            let shared = moved
                .iter()
                .filter(|&&(v, _)| {
                    let c = s.copies(v);
                    c.iter().any(|x| x.0 != c[0].0)
                })
                .count();
            assert!(shared > 100, "{how:?}: {shared}");
            let mut edited = mesh.clone();
            for &(v, p) in &moved {
                for &(page, i) in s.copies(v) {
                    edited.pages[page as usize].positions[i as usize] = p;
                }
            }
            let again = WeldedSurface::from_mesh(&edited);
            assert_eq!(again.faces(), s.faces(), "{how:?}");
            assert_eq!(
                again.positions(),
                &positions_after(&s, &moved)[..],
                "{how:?}"
            );
        }
    }

    /// A Stelle at the open edge of the surface (the end of the scan, a hole): the edge is
    /// smoothed along itself and stays where it is.
    #[test]
    fn open_edge_keeps_its_place() {
        let (_, s) = noisy_floor(7);
        // Half a disk at the edge y = 0 of the floor.
        let faces = near(&s, [0.5, 0.0, 0.0], 0.3);
        let (verts, border) = region(&s, &faces);
        let on_edge = verts
            .iter()
            .filter(|&&v| s.positions()[v as usize][1] == 0.0)
            .filter(|v| border.binary_search(v).is_err())
            .count();
        assert!(on_edge > 30, "{on_edge}");
        for how in [Light, Strong] {
            let moved = smooth_region(&s, &faces, how).unwrap();
            let mut seen = 0;
            for &(v, p) in &moved {
                let old = s.positions()[v as usize];
                if old[1] == 0.0 {
                    seen += 1;
                    assert_eq!(p[1], 0.0, "{how:?}: {v} left the edge");
                    assert!((p[0] - old[0]).abs() < 1e-6, "{how:?}: {v} slid along it");
                }
            }
            assert!(2 * seen > on_edge, "{how:?}: {seen} of {on_edge}");
        }
    }

    /// „Leicht“ and „stark“ take out the noise, not the shape: a wave of 5 cm over 1 m
    /// without noise stays within half a millimetre.
    #[test]
    fn keeps_the_shape() {
        let (positions, indices) = grid([100, 100], |i, j| {
            let x = i as f64 * STEP;
            let z = 0.05 * (2.0 * std::f64::consts::PI * x).sin();
            f32_exact([x, j as f64 * STEP, z])
        });
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 37, 5));
        let faces = near(&s, [0.5, 0.5, 0.0], 0.3);
        for how in [Light, Strong] {
            let moved = smooth_region(&s, &faces, how).unwrap();
            let worst = moved
                .iter()
                .map(|&(v, p)| dist(p, s.positions()[v as usize]))
                .fold(0.0, f64::max);
            assert!(worst < 0.0005, "{how:?}: {worst}");
        }
    }

    /// Two walls at 90°, 1 × 1 m in squares of 2 cm, joined along the corner x = y = 0.
    fn corner() -> WeldedSurface {
        let a = grid([50, 50], |i, j| {
            f32_exact([0.0, i as f64 * 0.02, j as f64 * 0.02])
        });
        let b = grid([50, 50], |i, j| {
            f32_exact([j as f64 * 0.02, 0.0, i as f64 * 0.02])
        });
        let (positions, indices) = concat(&[a, b]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 23, 3));
        assert_eq!(s.positions().len(), positions.len() - 51);
        s
    }

    /// Over a corner „ganz eben“ refuses and moves nothing; „leicht“ smooths.
    #[test]
    fn corner_is_not_flat() {
        let s = corner();
        let faces = near(&s, [0.0, 0.0, 0.5], 0.3);
        match smooth_region(&s, &faces, Flat) {
            Err(EditError::NotFlat { rms_m }) => assert!(rms_m > 0.03, "{rms_m}"),
            other => panic!("{other:?}"),
        }
        let light = smooth_region(&s, &faces, Light).unwrap();
        assert!(!light.is_empty());
    }

    /// „Ganz eben“ refuses when the vertices lie too far from their plane (a bowed wall) or
    /// do not lie on a plane at all (a small corner, close to its plane in centimetres);
    /// a slightly bowed wall is flat enough.
    #[test]
    fn flat_needs_a_plane() {
        let s = corner();
        let faces = near(&s, [0.0, 0.0, 0.5], 0.08);
        match smooth_region(&s, &faces, Flat) {
            Err(EditError::NotFlat { rms_m }) => assert!(rms_m < 0.03, "{rms_m}"),
            other => panic!("{other:?}"),
        }
        // A wall of 4 × 4 m in squares of 5 cm, bowed around a cylinder of radius `r`.
        let bowed = |r: f64| {
            let (positions, indices) = grid([80, 80], |i, j| {
                let x = -2.0 + i as f64 * 0.05;
                f32_exact([x, j as f64 * 0.05, x * x / (2.0 * r)])
            });
            let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 31, 2));
            let faces: Vec<u32> = (0..s.face_count() as u32).collect();
            smooth_region(&s, &faces, Flat)
        };
        // Sagitta 20 cm: about 6 cm from the plane.
        match bowed(10.0) {
            Err(EditError::NotFlat { rms_m }) => assert!(rms_m > 0.05, "{rms_m}"),
            other => panic!("{other:?}"),
        }
        // Sagitta 4 cm: about 1.2 cm, flat enough. The whole surface is the Stelle, without
        // border: every vertex takes its full shift onto the plane.
        let moved = bowed(50.0).unwrap();
        assert!(moved.len() > 6000, "{}", moved.len());
    }

    /// Time for a Stelle of 1 m² and one of 10 m² on a noisy surface of Büro size: 1818²
    /// squares of 3.5 cm (6.6 M triangles, 3.3 M vertices, 63.6 m wide) on 28 pages.
    /// `cargo test -p driftless-map --release smooth:: -- --ignored --nocapture`
    #[test]
    #[ignore = "Messung"]
    fn timing_buero_size() {
        use std::time::Instant;
        let (n, step) = (1818, 0.035);
        let mut rng = Lcg::new(1);
        let (positions, indices) = grid([n, n], |i, j| {
            f32_exact([i as f64 * step, j as f64 * step, SIGMA * rng.normal()])
        });
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 18, 28));
        assert_eq!(s.face_count(), 2 * n * n);
        let middle = n as f64 * step / 2.0;
        for area in [1.0, 10.0] {
            let r = (area / std::f64::consts::PI).sqrt();
            let faces = near(&s, [middle, middle, 0.0], r);
            for how in [Light, Strong, Flat] {
                let t = Instant::now();
                let moved = smooth_region(&s, &faces, how).unwrap();
                println!(
                    "{area} m² ({} Flächen), {how:?}: {:?}, {} Punkte verschoben, {} Fäden",
                    faces.len(),
                    t.elapsed(),
                    moved.len(),
                    rayon::current_num_threads()
                );
            }
        }
    }

    /// The same Stelle gives the same bytes: twice, with 1 and 8 threads, with the faces in
    /// another order and every face twice (the Stelle reaches the open edge of the floor,
    /// where a face counted twice would hide the edge).
    #[test]
    fn deterministic() {
        let (_, s) = noisy_floor(8);
        let faces = near(&s, [0.45, 0.1, 0.0], 0.3);
        let mut shuffled: Vec<u32> = faces.iter().rev().copied().collect();
        shuffled.extend_from_slice(&faces);
        let pool = |threads| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
        };
        let bits = |moved: Vec<(u32, [f64; 3])>| -> Vec<(u32, [u64; 3])> {
            moved
                .into_iter()
                .map(|(v, p)| (v, p.map(f64::to_bits)))
                .collect()
        };
        for how in [Light, Strong, Flat] {
            let one = bits(pool(1).install(|| smooth_region(&s, &faces, how).unwrap()));
            let many = bits(pool(8).install(|| smooth_region(&s, &shuffled, how).unwrap()));
            let again = bits(smooth_region(&s, &faces, how).unwrap());
            assert!(one.len() > 1000, "{how:?}: {}", one.len());
            assert_eq!(one, many, "{how:?}");
            assert_eq!(one, again, "{how:?}");
        }
        // The refusal too.
        let s = corner();
        let faces = near(&s, [0.0, 0.0, 0.5], 0.3);
        let rms = || match smooth_region(&s, &faces, Flat) {
            Err(EditError::NotFlat { rms_m }) => rms_m.to_bits(),
            other => panic!("{other:?}"),
        };
        assert_eq!(pool(1).install(rms), pool(8).install(rms));
    }

    #[test]
    fn empty_selection() {
        let (_, s) = noisy_floor(9);
        for how in [Light, Strong, Flat] {
            assert!(matches!(
                smooth_region(&s, &[], how),
                Err(EditError::NoFaces)
            ));
            let empty = WeldedSurface::default();
            assert!(matches!(
                smooth_region(&empty, &[], how),
                Err(EditError::NoFaces)
            ));
        }
    }
}
