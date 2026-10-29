//! The faces of a [`WeldedSurface`] in a bounding volume hierarchy: the face a ray hits
//! first, and the faces near a point.
//!
//! A linear BVH (LBVH), sorted like the view's octree: every face's centroid gets a Morton
//! code (21 bits per axis over the cube around the surface), and the faces are sorted by
//! (code, face), which is unique. The sorted list is split top-down at the highest bit in
//! which the first and the last code of a part differ (in the middle where they are all
//! equal), until a part has at most [`LEAF_FACES`] faces; so the parts follow the cells of an
//! octree and a jump of the Morton curve never ends up inside one box. The boxes are computed
//! bottom up and kept as `f32` relative to the centre of the surface, widened by
//! [`BOX_MARGIN`] and rounded outwards: a box always holds its faces, whatever the rounding of
//! a query. The tree depends only on the faces, so it comes out the same, byte for byte, with
//! any number of threads.
//!
//! Both queries test every face whose box they reach and skip a box only where no face in it
//! can count, so they give what testing every face would give. A ray hits a face from either
//! side (the view draws both); the nearest hit wins, at the same distance the smaller face.

use super::{cross, dot, sub, WeldedSurface};
use rayon::prelude::*;

/// Faces per leaf at most.
pub const LEAF_FACES: usize = 8;
/// Bits per axis of the Morton code (63 bits in all).
const MORTON_BITS: u32 = 21;
/// Boxes reach this far beyond their faces (m): far more than the rounding of a query in
/// `f64`, far less than a face.
const BOX_MARGIN: f64 = 1e-6;
/// Smallest edge of the Morton cube (m), so that a single point still has one.
const MIN_CUBE: f64 = 1e-3;

/// A box of the tree.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Node {
    /// Corners of the box, relative to [`FaceBvh::origin`].
    lo: [f32; 3],
    hi: [f32; 3],
    /// Leaf: its first face in [`FaceBvh::order`]. Inner node: its first child; the second
    /// one follows it.
    start: u32,
    /// Leaf: its number of faces (1 to [`LEAF_FACES`]). Inner node: 0.
    count: u32,
}

/// Where a ray first meets the surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub face: u32,
    /// `pos = origin + t · dir` (metres for a `dir` of length 1).
    pub t: f64,
    pub pos: [f64; 3],
}

/// The faces of a [`WeldedSurface`] in a tree of boxes (see the module docs). It holds no
/// geometry: every query takes the surface it was built for.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FaceBvh {
    /// The centre of the surface's bounds; the boxes are relative to it.
    origin: [f64; 3],
    /// The root first; every node before its children, which are next to each other.
    nodes: Vec<Node>,
    /// The faces in Morton order; every leaf is a range of it.
    order: Vec<u32>,
}

impl FaceBvh {
    /// Builds the tree over the faces of `surface` (see the module docs).
    pub fn build(surface: &WeldedSurface) -> FaceBvh {
        let s = surface;
        let n = s.face_count();
        if n == 0 {
            return FaceBvh::default();
        }
        assert!(n < (u32::MAX / 2) as usize, "{n} faces are too many");

        // The bounds; +0.0 for −0.0, so min and max do not depend on the order.
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in s.positions() {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k] + 0.0);
                hi[k] = hi[k].max(p[k] + 0.0);
            }
        }
        let origin = [0, 1, 2].map(|k| (lo[k] + hi[k]) / 2.0);

        // Morton order of the centroids, on the grid of the cube at the lower bounds.
        let size = (0..3).map(|k| hi[k] - lo[k]).fold(MIN_CUBE, f64::max);
        let scale = f64::from(1u32 << MORTON_BITS) / size;
        let top = (1u64 << MORTON_BITS) - 1;
        let mut keys: Vec<(u64, u32)> = (0..n as u32)
            .into_par_iter()
            .map(|f| {
                let c = s.face_centroid(f);
                let q = [0, 1, 2].map(|k| {
                    let cell = ((c[k] - lo[k]) * scale).floor().max(0.0);
                    (cell as u64).min(top)
                });
                (spread(q[0]) | spread(q[1]) << 1 | spread(q[2]) << 2, f)
            })
            .collect();
        // Unique keys: any sort gives the same order.
        keys.par_sort_unstable();

        // The tree, top-down; children are added as a pair.
        let mut nodes = vec![Node::default()];
        let mut parts = vec![(0usize, 0usize, n)];
        while let Some((node, a, b)) = parts.pop() {
            if b - a <= LEAF_FACES {
                nodes[node].start = a as u32;
                nodes[node].count = (b - a) as u32;
                continue;
            }
            let m = a + split(&keys[a..b]);
            let first = nodes.len();
            nodes[node].start = first as u32;
            nodes.extend([Node::default(); 2]);
            parts.push((first + 1, m, b));
            parts.push((first, a, m));
        }
        let order: Vec<u32> = keys.iter().map(|k| k.1).collect();
        drop(keys);

        // Leaf boxes (each from its own faces only), then the inner nodes bottom up: children
        // come after their parent.
        let (positions, faces) = (s.positions(), s.faces());
        nodes
            .par_iter_mut()
            .filter(|node| node.count > 0)
            .for_each(|node| {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                let leaf = &order[node.start as usize..(node.start + node.count) as usize];
                for &f in leaf {
                    for v in faces[f as usize] {
                        let p = positions[v as usize];
                        for k in 0..3 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                    }
                }
                node.lo = [0, 1, 2].map(|k| down(lo[k] - origin[k] - BOX_MARGIN));
                node.hi = [0, 1, 2].map(|k| up(hi[k] - origin[k] + BOX_MARGIN));
            });
        for i in (0..nodes.len()).rev() {
            if nodes[i].count == 0 {
                let c = nodes[i].start as usize;
                let (l, r) = (nodes[c], nodes[c + 1]);
                nodes[i].lo = [0, 1, 2].map(|k| l.lo[k].min(r.lo[k]));
                nodes[i].hi = [0, 1, 2].map(|k| l.hi[k].max(r.hi[k]));
            }
        }
        // Kept for the whole session: no room to grow.
        nodes.shrink_to_fit();
        FaceBvh {
            origin,
            nodes,
            order,
        }
    }

    /// The face the ray `origin + t·dir` (`t > 0`) hits first, from either side; the smaller
    /// face at the same `t`. With `z_range = Some([lo, hi])` only a hit with `lo ≤ z ≤ hi`
    /// counts, as the view's height cut draws it: the ray passes through the others. `None`
    /// if nothing is hit, if `dir` is zero, `origin` or `dir` not finite, or the cut empty or
    /// not a number.
    pub fn raycast(
        &self,
        s: &WeldedSurface,
        origin: [f64; 3],
        dir: [f64; 3],
        z_range: Option<[f64; 2]>,
    ) -> Option<RayHit> {
        let [z_lo, z_hi] = z_range.unwrap_or([f64::NEG_INFINITY, f64::INFINITY]);
        let finite = origin.iter().chain(&dir).all(|c| c.is_finite());
        let cut_empty = z_lo.is_nan() || z_hi.is_nan() || z_lo > z_hi;
        if self.nodes.is_empty() || !finite || dir == [0.0; 3] || cut_empty {
            return None;
        }
        // The ray and the cut in the frame of the boxes.
        let ray = Ray {
            o: sub(origin, self.origin),
            d: dir,
            inv: dir.map(|c| 1.0 / c),
            cut: [z_lo - self.origin[2], z_hi - self.origin[2]],
        };
        let mut best: Option<(f64, u32)> = None;
        let mut stack: Vec<(u32, f64)> = Vec::with_capacity(64);
        if let Some(t) = self.enter(0, &ray) {
            stack.push((0, t));
        }
        while let Some((node, t_in)) = stack.pop() {
            if best.is_some_and(|(t, _)| t_in > t) {
                continue;
            }
            let node = self.nodes[node as usize];
            if node.count > 0 {
                let leaf = &self.order[node.start as usize..(node.start + node.count) as usize];
                for &f in leaf {
                    let Some(t) = ray_triangle(origin, dir, s.face_corners(f)) else {
                        continue;
                    };
                    let z = origin[2] + t * dir[2];
                    if z < z_lo || z > z_hi {
                        continue;
                    }
                    if best.is_none_or(|(bt, bf)| t < bt || (t == bt && f < bf)) {
                        best = Some((t, f));
                    }
                }
                continue;
            }
            let (a, b) = (node.start, node.start + 1);
            // The nearer child goes on the stack last, so it is looked at first.
            match (self.enter(a, &ray), self.enter(b, &ray)) {
                (Some(ta), Some(tb)) if ta <= tb => stack.extend([(b, tb), (a, ta)]),
                (Some(ta), Some(tb)) => stack.extend([(a, ta), (b, tb)]),
                (Some(ta), None) => stack.push((a, ta)),
                (None, Some(tb)) => stack.push((b, tb)),
                (None, None) => {}
            }
        }
        best.map(|(t, face)| RayHit {
            face,
            t,
            pos: [0, 1, 2].map(|k| origin[k] + t * dir[k]),
        })
    }

    /// The faces whose closest point lies within `radius` of `centre`, sorted. Nothing for a
    /// radius below zero or not a number, or a centre that is not finite.
    pub fn faces_within(&self, s: &WeldedSurface, centre: [f64; 3], radius: f64) -> Vec<u32> {
        self.faces_within_dist2(s, centre, radius)
            .into_iter()
            .map(|(f, _)| f)
            .collect()
    }

    /// [`faces_within`](Self::faces_within) with the squared distance of every face.
    pub(super) fn faces_within_dist2(
        &self,
        s: &WeldedSurface,
        centre: [f64; 3],
        radius: f64,
    ) -> Vec<(u32, f64)> {
        let mut out = Vec::new();
        let finite = centre.iter().all(|c| c.is_finite());
        if self.nodes.is_empty() || !finite || radius.is_nan() || radius < 0.0 {
            return out;
        }
        let r2 = radius * radius;
        let c = sub(centre, self.origin);
        let mut stack: Vec<u32> = Vec::with_capacity(64);
        stack.push(0);
        while let Some(node) = stack.pop() {
            let node = self.nodes[node as usize];
            let gap = |k: usize| {
                let d = (f64::from(node.lo[k]) - c[k])
                    .max(c[k] - f64::from(node.hi[k]))
                    .max(0.0);
                d * d
            };
            if gap(0) + gap(1) + gap(2) > r2 {
                continue;
            }
            if node.count == 0 {
                stack.extend([node.start + 1, node.start]);
                continue;
            }
            for &f in &self.order[node.start as usize..(node.start + node.count) as usize] {
                let d2 = point_triangle_dist2(centre, s.face_corners(f));
                if d2 <= r2 {
                    out.push((f, d2));
                }
            }
        }
        out.sort_unstable_by_key(|&(f, _)| f);
        out
    }

    /// The faces that `keep_face` keeps, of the boxes that `keep_box` lets through, sorted.
    /// `keep_box` gets the lower and upper corner of a box (absolute, a little larger than its
    /// faces); a box it refuses is skipped with everything in it, so it must let through every
    /// box that can hold a face `keep_face` keeps.
    #[cfg_attr(
        not(test),
        expect(
            dead_code,
            reason = "used by the second part of „Bilder neu einspielen“"
        )
    )]
    pub(super) fn faces_where(
        &self,
        keep_box: impl Fn([f64; 3], [f64; 3]) -> bool,
        mut keep_face: impl FnMut(u32) -> bool,
    ) -> Vec<u32> {
        let mut out = Vec::new();
        if self.nodes.is_empty() {
            return out;
        }
        let mut stack: Vec<u32> = Vec::with_capacity(64);
        stack.push(0);
        while let Some(node) = stack.pop() {
            let node = self.nodes[node as usize];
            let lo = [0, 1, 2].map(|k| self.origin[k] + f64::from(node.lo[k]));
            let hi = [0, 1, 2].map(|k| self.origin[k] + f64::from(node.hi[k]));
            if !keep_box(lo, hi) {
                continue;
            }
            if node.count == 0 {
                stack.extend([node.start + 1, node.start]);
                continue;
            }
            for &f in &self.order[node.start as usize..(node.start + node.count) as usize] {
                if keep_face(f) {
                    out.push(f);
                }
            }
        }
        out.sort_unstable();
        out
    }

    /// Where the ray enters the box of `node` (`t ≥ 0`), or `None` if it misses the box or
    /// the box lies wholly outside the height cut.
    fn enter(&self, node: u32, ray: &Ray) -> Option<f64> {
        let node = &self.nodes[node as usize];
        if f64::from(node.hi[2]) < ray.cut[0] || f64::from(node.lo[2]) > ray.cut[1] {
            return None;
        }
        let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
        for k in 0..3 {
            let (lo, hi) = (f64::from(node.lo[k]), f64::from(node.hi[k]));
            if ray.d[k] == 0.0 {
                if ray.o[k] < lo || ray.o[k] > hi {
                    return None;
                }
                continue;
            }
            let (a, b) = ((lo - ray.o[k]) * ray.inv[k], (hi - ray.o[k]) * ray.inv[k]);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
        (t0 <= t1).then_some(t0)
    }

    /// FNV-1a over every byte (tests: the same tree, byte for byte).
    #[cfg(test)]
    pub(super) fn digest(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |bytes: &[u8]| {
            for &b in bytes {
                h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
            }
        };
        for c in self.origin {
            eat(&c.to_bits().to_le_bytes());
        }
        for node in &self.nodes {
            for c in node.lo.iter().chain(&node.hi) {
                eat(&c.to_bits().to_le_bytes());
            }
            eat(&node.start.to_le_bytes());
            eat(&node.count.to_le_bytes());
        }
        for f in &self.order {
            eat(&f.to_le_bytes());
        }
        h
    }
}

/// A ray in the frame of the boxes: `o + t·d`, `inv = 1 / d`; `cut` is the height cut.
struct Ray {
    o: [f64; 3],
    d: [f64; 3],
    inv: [f64; 3],
    cut: [f64; 2],
}

/// Spreads the lowest 21 bits of `v` to every third bit.
fn spread(v: u64) -> u64 {
    let mut x = v & 0x1f_ffff;
    x = (x | x << 32) & 0x001f_0000_0000_ffff;
    x = (x | x << 16) & 0x001f_0000_ff00_00ff;
    x = (x | x << 8) & 0x100f_00f0_0f00_f00f;
    x = (x | x << 4) & 0x10c3_0c30_c30c_30c3;
    x = (x | x << 2) & 0x1249_2492_4924_9249;
    x
}

/// Where to split a sorted part of more than one face: before the first code with the highest
/// bit set in which its first and last codes differ (all codes between agree above it), or in
/// the middle if all codes are equal.
fn split(keys: &[(u64, u32)]) -> usize {
    let (first, last) = (keys[0].0, keys[keys.len() - 1].0);
    if first == last {
        return keys.len() / 2;
    }
    let bit = 63 - (first ^ last).leading_zeros();
    keys.partition_point(|k| (k.0 >> bit) & 1 == 0)
}

/// `v` as `f32`, rounded down.
fn down(v: f64) -> f32 {
    let f = v as f32;
    if f64::from(f) > v {
        f.next_down()
    } else {
        f
    }
}

/// `v` as `f32`, rounded up.
fn up(v: f64) -> f32 {
    let f = v as f32;
    if f64::from(f) < v {
        f.next_up()
    } else {
        f
    }
}

/// Where the ray `o + t·d` crosses the triangle from either side (Möller–Trumbore), with
/// `t > 0`; `None` if it misses, runs along the triangle's plane or the triangle has no area.
pub(super) fn ray_triangle(o: [f64; 3], d: [f64; 3], [a, b, c]: [[f64; 3]; 3]) -> Option<f64> {
    let (e1, e2) = (sub(b, a), sub(c, a));
    let p = cross(d, e2);
    let det = dot(e1, p);
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub(o, a);
    let u = dot(s, p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(d, q) * inv;
    if v.is_nan() || v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = dot(e2, q) * inv;
    (t > 0.0).then_some(t)
}

/// Squared distance from `p` to the closest point of the triangle, by the region `p` projects
/// to (Ericson, Real-Time Collision Detection, 5.1.5); a triangle without area counts as its
/// three edges.
pub(super) fn point_triangle_dist2(p: [f64; 3], [a, b, c]: [[f64; 3]; 3]) -> f64 {
    let (ab, ac) = (sub(b, a), sub(c, a));
    let edges = || {
        segment_dist2(p, a, b)
            .min(segment_dist2(p, b, c))
            .min(segment_dist2(p, c, a))
    };
    if cross(ab, ac) == [0.0; 3] {
        return edges();
    }
    let ap = sub(p, a);
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return dot(ap, ap);
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return dot(bp, bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return segment_dist2(p, a, b);
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return dot(cp, cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return segment_dist2(p, a, c);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 >= d3 && d5 >= d6 {
        return segment_dist2(p, b, c);
    }
    let sum = va + vb + vc;
    if sum <= 0.0 || !sum.is_finite() {
        return edges();
    }
    let (v, w) = (vb / sum, vc / sum);
    let q = [0, 1, 2].map(|k| a[k] + ab[k] * v + ac[k] * w);
    let pq = sub(p, q);
    dot(pq, pq)
}

/// Squared distance from `p` to the segment `a`–`b`.
fn segment_dist2(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let (ab, ap) = (sub(b, a), sub(p, a));
    let len2 = dot(ab, ab);
    let t = if len2 > 0.0 {
        (dot(ap, ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q = sub(p, [0, 1, 2].map(|k| a[k] + ab[k] * t));
    dot(q, q)
}

#[cfg(test)]
mod tests {
    use super::super::{dot, sub, unit};
    use super::*;
    use crate::surface::testutil::{concat, grid, paged, Lcg, Shared};

    /// A wavy grid of `n × n` squares of `step` (m) with its corner at `origin`, normals up.
    fn wavy(n: usize, step: f64, origin: [f64; 3]) -> Shared {
        grid([n, n], |i, j| {
            let (x, y) = (i as f64 * step, j as f64 * step);
            let z = 0.25 * (1.3 * x).sin() * (0.9 * y).cos() + 0.04 * (7.0 * x + 3.0 * y).sin();
            [origin[0] + x, origin[1] + y, origin[2] + z]
        })
    }

    /// A direction uniform on the sphere (rejection sampling, no trigonometry).
    fn direction(rng: &mut Lcg) -> [f64; 3] {
        loop {
            let v = [0, 1, 2].map(|_| rng.range(-1.0, 1.0));
            let n2 = dot(v, v);
            if n2 > 1e-4 && n2 <= 1.0 {
                return unit(v).unwrap();
            }
        }
    }

    /// The nearest hit over all faces in face order (equal distance: the smaller face).
    fn brute_ray(
        s: &WeldedSurface,
        o: [f64; 3],
        d: [f64; 3],
        z_range: Option<[f64; 2]>,
    ) -> Option<(u32, f64)> {
        let [lo, hi] = z_range.unwrap_or([f64::NEG_INFINITY, f64::INFINITY]);
        let mut best: Option<(u32, f64)> = None;
        for f in 0..s.face_count() as u32 {
            let Some(t) = ray_triangle(o, d, s.face_corners(f)) else {
                continue;
            };
            let z = o[2] + t * d[2];
            if z < lo || z > hi {
                continue;
            }
            if best.is_none_or(|(_, bt)| t < bt) {
                best = Some((f, t));
            }
        }
        best
    }

    fn brute_within(s: &WeldedSurface, c: [f64; 3], r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| point_triangle_dist2(c, s.face_corners(f)) <= r * r)
            .collect()
    }

    /// 2,000 random rays and 2,000 random spheres on a wavy grid of 20,000 triangles at
    /// (5e5, 5e6, 300) m: the tree finds what testing every face finds. Every fourth ray aims
    /// at a vertex (several faces at the same distance), every third has a height cut.
    #[test]
    fn bvh_matches_brute_force() {
        let origin = [5e5, 5e6, 300.0];
        let (positions, indices) = wavy(100, 0.05, origin);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 13, 3));
        assert_eq!(s.face_count(), 20_000);
        let bvh = FaceBvh::build(&s);
        let mut rng = Lcg::new(1);
        let vertex = |rng: &mut Lcg| positions[(rng.unit() * positions.len() as f64) as usize];
        let (mut hits, mut cut) = (0, 0);
        for i in 0..2000 {
            let o = [
                origin[0] + rng.range(-1.0, 6.0),
                origin[1] + rng.range(-1.0, 6.0),
                origin[2] + rng.range(-1.0, 1.0),
            ];
            let d = if i % 4 == 0 {
                unit(sub(vertex(&mut rng), o)).unwrap()
            } else {
                direction(&mut rng)
            };
            let z_range = (i % 3 == 0).then(|| {
                let z = origin[2] + rng.range(-0.3, 0.3);
                [z - 0.1, z + 0.1]
            });
            let want = brute_ray(&s, o, d, z_range);
            let got = bvh.raycast(&s, o, d, z_range);
            assert_eq!(
                got.map(|h| (h.face, h.t.to_bits())),
                want.map(|(f, t)| (f, t.to_bits())),
                "ray {i}: {o:?} {d:?} {z_range:?}"
            );
            if let Some(h) = got {
                assert_eq!(h.pos, [0, 1, 2].map(|k| o[k] + h.t * d[k]));
                hits += 1;
                cut += usize::from(z_range.is_some());
            }
        }
        assert!(hits > 500 && cut > 50, "{hits} hits, {cut} with a cut");

        let mut found = 0;
        for i in 0..2000 {
            let c = if i % 4 == 0 {
                vertex(&mut rng)
            } else {
                [
                    origin[0] + rng.range(-0.5, 5.5),
                    origin[1] + rng.range(-0.5, 5.5),
                    origin[2] + rng.range(-0.5, 0.5),
                ]
            };
            let r = rng.range(0.0, 0.4);
            let want = brute_within(&s, c, r);
            assert_eq!(bvh.faces_within(&s, c, r), want, "sphere {i}: {c:?} {r}");
            found += usize::from(!want.is_empty());
        }
        assert!(found > 800, "{found} of 2000 spheres reach a face");
    }

    /// A floor at z = 0 and a ceiling at z = 2.5: a hit outside the height cut does not
    /// count, the ray goes on to the next face.
    #[test]
    fn raycast_skips_cut() {
        let floor = grid([8, 8], |i, j| {
            [i as f64 * 0.5 - 2.0, j as f64 * 0.5 - 2.0, 0.0]
        });
        let ceiling = grid([8, 8], |i, j| {
            [i as f64 * 0.5 - 2.0, j as f64 * 0.5 - 2.0, 2.5]
        });
        let (positions, indices) = concat(&[floor, ceiling]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 10, 2));
        let bvh = FaceBvh::build(&s);
        let down = |range| bvh.raycast(&s, [0.1, 0.2, 3.0], [0.0, 0.0, -1.0], range);
        let up = |range| bvh.raycast(&s, [0.1, 0.2, -1.0], [0.0, 0.0, 1.0], range);
        let z = |hit: Option<RayHit>| hit.map(|h| h.pos[2]);
        assert_eq!(z(down(None)), Some(2.5));
        assert_eq!(down(None).unwrap().t, 0.5);
        // The ceiling is above the cut: the ray passes it and hits the floor.
        assert_eq!(z(down(Some([-1.0, 2.0]))), Some(0.0));
        assert_eq!(down(Some([-1.0, 2.0])).unwrap().t, 3.0);
        assert_eq!(z(up(None)), Some(0.0));
        // The floor is below the cut.
        assert_eq!(z(up(Some([0.5, 3.0]))), Some(2.5));
        assert_eq!(z(up(Some([0.5, 2.0]))), None);
        // The cut holds its ends.
        assert_eq!(z(down(Some([0.0, 2.5]))), Some(2.5));
        assert_eq!(z(up(Some([0.0, 2.5]))), Some(0.0));
        // An empty or unusable cut lets nothing through.
        assert_eq!(z(down(Some([1.0, 0.5]))), None);
        assert_eq!(z(down(Some([f64::NAN, 3.0]))), None);
        // Rays that miss, run along the floor, have no direction or start beyond the surface.
        assert_eq!(
            z(bvh.raycast(&s, [0.1, 0.2, 3.0], [0.0, 0.0, 1.0], None)),
            None
        );
        assert_eq!(
            z(bvh.raycast(&s, [-5.0, 0.2, 0.0], [1.0, 0.0, 0.0], None)),
            None
        );
        assert_eq!(z(bvh.raycast(&s, [0.1, 0.2, 1.0], [0.0; 3], None)), None);
        assert_eq!(
            z(bvh.raycast(&s, [0.1, 0.2, 9.0], [0.0, 0.0, 1.0], None)),
            None
        );
        assert_eq!(
            z(bvh.raycast(&s, [0.1, 0.2, f64::NAN], [0.0, 0.0, -1.0], None)),
            None
        );
    }

    #[test]
    fn empty_and_degenerate_faces() {
        let empty = WeldedSurface::default();
        let bvh = FaceBvh::build(&empty);
        assert_eq!(bvh.raycast(&empty, [0.0; 3], [1.0, 0.0, 0.0], None), None);
        assert!(bvh.faces_within(&empty, [0.0; 3], 10.0).is_empty());

        // Faces without area (a repeated corner, corners on a line, one point) among good ones:
        // rays pass them, distances reach them, nothing breaks.
        let (mut positions, mut indices) = grid([3, 3], |i, j| [i as f64, j as f64, 0.0]);
        let base = positions.len() as u32;
        positions.extend([[0.5, 0.5, 1.0], [1.5, 0.5, 1.0], [2.5, 0.5, 1.0]]);
        indices.extend([
            [base, base + 1, base + 2],
            [base, base + 1, base + 1],
            [base, base, base],
        ]);
        let s = WeldedSurface::from_mesh(&paged(&positions, &indices, 4, 2));
        let bvh = FaceBvh::build(&s);
        let mut rng = Lcg::new(5);
        for i in 0..300 {
            let o = [0, 1, 2].map(|_| rng.range(-0.5, 3.5));
            let d = direction(&mut rng);
            let want = brute_ray(&s, o, d, None);
            let got = bvh.raycast(&s, o, d, None).map(|h| (h.face, h.t));
            assert_eq!(got, want, "ray {i}");
            let r = rng.range(0.0, 1.0);
            assert_eq!(
                bvh.faces_within(&s, o, r),
                brute_within(&s, o, r),
                "sphere {i}"
            );
        }
        // Straight down onto the line of degenerate faces: the floor is hit.
        let hit = bvh
            .raycast(&s, [1.5, 0.5, 2.0], [0.0, 0.0, -1.0], None)
            .unwrap();
        assert_eq!(hit.pos[2], 0.0);
        // Right next to the line, the degenerate faces count by their distance.
        let near = bvh.faces_within(&s, [1.5, 0.5, 1.001], 0.01);
        assert_eq!(near.len(), 2, "{near:?}");
        let p = bvh.faces_within(&s, [0.5, 0.5, 1.0], 0.0);
        assert_eq!(p.len(), 3, "{p:?}");
    }

    #[test]
    fn distance_to_triangle() {
        let tri = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]];
        let d2 = |p| point_triangle_dist2(p, tri);
        assert_eq!(d2([0.5, 0.5, 3.0]), 9.0); // above the inside
        assert_eq!(d2([-1.0, -1.0, 0.0]), 2.0); // beyond a corner
        assert_eq!(d2([1.0, -2.0, 0.0]), 4.0); // beside an edge
        assert_eq!(d2([2.0, 2.0, 0.0]), 2.0); // beside the long edge
        assert_eq!(d2([0.0, 0.0, 0.0]), 0.0);
        // A degenerate triangle is its longest segment.
        let line = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0]];
        assert!((point_triangle_dist2([2.0, 1.0, 0.0], line) - 1.0).abs() < 1e-12);
        assert!((point_triangle_dist2([4.0, 0.0, 0.0], line) - 1.0).abs() < 1e-12);
        let repeated = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        assert!((point_triangle_dist2([1.0, 0.5, 0.0], repeated) - 0.25).abs() < 1e-12);
        let dot_tri = [[1.0, 1.0, 1.0]; 3];
        assert_eq!(point_triangle_dist2([1.0, 1.0, 3.0], dot_tri), 4.0);
    }

    #[test]
    fn ray_hits_both_sides() {
        let tri = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        assert_eq!(
            ray_triangle([0.2, 0.2, 1.0], [0.0, 0.0, -1.0], tri),
            Some(1.0)
        );
        assert_eq!(
            ray_triangle([0.2, 0.2, -2.0], [0.0, 0.0, 1.0], tri),
            Some(2.0)
        );
        // Behind the start, beside the triangle, along its plane: no hit.
        assert_eq!(ray_triangle([0.2, 0.2, 1.0], [0.0, 0.0, 1.0], tri), None);
        assert_eq!(ray_triangle([0.8, 0.8, 1.0], [0.0, 0.0, -1.0], tri), None);
        assert_eq!(ray_triangle([-1.0, 0.2, 0.0], [1.0, 0.0, 0.0], tri), None);
        assert_eq!(
            ray_triangle([0.2, 0.2, 1.0], [0.0, 0.0, f64::NAN], tri),
            None
        );
    }

    /// `from_mesh` and `build` at the size of the Büro surface, and the time per ray and per
    /// dab there: a wavy grid of 1818² squares of 3.5 cm (6.61 M triangles, 3.3 M vertices,
    /// 63.6 m wide), its faces in charts of 3 × 3 squares (18 faces, 16 corners) dealt to 28
    /// pages: 5.9 M corners on the pages (Büro: 6.64 M triangles, 5.4 M corners, 28 pages).
    /// `cargo test -p driftless-map --release edit:: -- --ignored --nocapture`
    #[test]
    #[ignore = "Messung"]
    fn timing_buero_size() {
        use super::super::{dab_faces, dab_normal, SelectDab};
        use std::time::Instant;
        let (n, step) = (1818, 0.035);
        let mesh = {
            let (positions, rows) = wavy(n, step, [0.0; 3]);
            let mut charts = Vec::with_capacity(rows.len());
            for ti in (0..n).step_by(3) {
                for tj in (0..n).step_by(3) {
                    for i in ti..ti + 3 {
                        for j in tj..tj + 3 {
                            let f = 2 * (i * n + j);
                            charts.extend_from_slice(&rows[f..f + 2]);
                        }
                    }
                }
            }
            paged(&positions, &charts, 18, 28)
        };
        let corners: usize = mesh.pages.iter().map(|p| p.positions.len()).sum();

        let t = Instant::now();
        let s = WeldedSurface::from_mesh(&mesh);
        let weld = t.elapsed();
        let t = Instant::now();
        let bvh = FaceBvh::build(&s);
        let build = t.elapsed();
        assert_eq!(s.face_count(), 2 * n * n);

        let mb = |bytes: usize| bytes as f64 / 1e6;
        let vertices = s.positions().len();
        let welded = mb(vertices * 24 + s.face_count() * 12 + (vertices + 1) * 4 + corners * 8);
        let tree = mb(bvh.nodes.len() * size_of::<Node>() + bvh.order.len() * 4);
        let leaves = bvh.nodes.iter().filter(|node| node.count > 0).count();

        // Rays from 1.5 m above, tilted by up to 45°.
        let side = n as f64 * step;
        let mut rng = Lcg::new(3);
        let rays: Vec<([f64; 3], [f64; 3])> = (0..10_000)
            .map(|_| {
                let o = [rng.range(1.0, side - 1.0), rng.range(1.0, side - 1.0), 1.5];
                let d = unit([rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), -1.0]).unwrap();
                (o, d)
            })
            .collect();
        let t = Instant::now();
        let hits: Vec<RayHit> = rays
            .iter()
            .filter_map(|&(o, d)| bvh.raycast(&s, o, d, None))
            .collect();
        let per_ray = t.elapsed() / rays.len() as u32;
        assert!(hits.len() > 9_000, "{}", hits.len());
        // A dab at each of the first 1,000 hits: its normal and its faces.
        let dabs = |radius: f64| {
            let t = Instant::now();
            let mut faces = 0;
            for hit in &hits[..1000] {
                let normal = dab_normal(&s, &bvh, hit, radius);
                let dab = SelectDab {
                    centre: hit.pos,
                    normal,
                    radius,
                    erase: false,
                };
                faces += dab_faces(&s, &bvh, &dab).len();
            }
            (t.elapsed() / 1000, faces / 1000)
        };
        let (dab_small, faces_small) = dabs(0.2);
        let (dab_large, faces_large) = dabs(1.0);
        println!(
            "{} Dreiecke, {corners} Ecken auf {} Seiten → {vertices} Punkte, {} Fäden\n\
             from_mesh {weld:?}, build {build:?}\n\
             Speicher: verschweißt {welded:.0} MB, Baum {tree:.0} MB ({} Knoten, {leaves} \
             Blätter, {:.2} Flächen je Blatt)\n\
             Strahl {per_ray:?} ({} von {} treffen); Tupfer 20 cm {dab_small:?} \
             ({faces_small} Flächen), 1 m {dab_large:?} ({faces_large} Flächen)",
            s.face_count(),
            mesh.pages.len(),
            rayon::current_num_threads(),
            bvh.nodes.len(),
            s.face_count() as f64 / leaves as f64,
            hits.len(),
            rays.len(),
        );
    }
}
