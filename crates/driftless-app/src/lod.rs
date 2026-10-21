//! Levels of detail for the 3D view: an octree whose inner nodes hold a thinned sample (one
//! point per cell of a 64³ grid over the node) and whose leaves hold the rest, and the
//! choice of nodes to draw for a camera within a point budget.
//!
//! Every point is in exactly one node (no copies), so the vertices of all nodes together are
//! the cloud once. The build is deterministic: the same points give the same tree, byte for
//! byte, whatever the number of threads.

use crate::cloud::PointSet;
use bytemuck::Zeroable;
use driftless_io::progress::Progress;
use driftless_steps::StepError;
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::ops::Range;

/// A node with more points than this is split (unless it is at [`MAX_DEPTH`]).
pub const LEAF_MAX: usize = 32_768;
/// An inner node keeps one point per cell of a grid of 2^`NODE_GRID_BITS` cells per axis.
pub const NODE_GRID_BITS: u32 = 6;
/// Most vertices in one GPU block (20 B each: 80 MiB).
pub const CHUNK_POINTS: u32 = 4_194_304;
/// Depth of the deepest nodes; their leaves keep all their points (e.g. many equal points).
pub const MAX_DEPTH: u32 = 15;
/// No child in this octant.
pub const NO_NODE: u32 = u32::MAX;
/// Phase name while the octree is built.
pub const PHASE_BUILD: &str = "Detailstufen bauen";

/// Bits per axis of the Morton code (63 bits in all).
const MORTON_BITS: u32 = 21;
/// Smallest edge of the root cube (m), so that a single point still has a cube.
const MIN_CUBE: f32 = 0.001;

/// A point as the GPU reads it.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PointVertex {
    /// Relative to [`Octree::origin`].
    pub pos: [f32; 3],
    /// `r | g << 8 | b << 16 | intensity << 24`; without colours r = g = b = intensity.
    pub rgbi: u32,
    /// `quality | flags << 8`.
    pub extra: u32,
}

/// A cube of the octree and the vertices it owns.
#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    pub min: [f32; 3],
    /// Edge length of the cube.
    pub size: f32,
    /// Distance between the points of an inner node's sample: `size / 64`.
    pub spacing: f32,
    /// The node's vertices are `first..first + count`.
    pub first: u32,
    pub count: u32,
    /// Index per octant `x | y << 1 | z << 2`, or [`NO_NODE`].
    pub children: [u32; 8],
    pub depth: u8,
}

impl Node {
    pub fn max(&self) -> [f32; 3] {
        self.min.map(|m| m + self.size)
    }

    pub fn centre(&self) -> [f32; 3] {
        self.min.map(|m| m + self.size / 2.0)
    }

    pub fn is_leaf(&self) -> bool {
        self.children.iter().all(|&c| c == NO_NODE)
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Octree {
    pub origin: [f64; 3],
    /// Breadth-first; 0 is the root.
    pub nodes: Vec<Node>,
    /// The vertices of each node together, nodes in the order of `nodes`.
    pub vertices: Vec<PointVertex>,
    /// Vertex → index of the point in the [`PointSet`].
    pub source: Vec<u32>,
    /// Vertex ranges of at most the block limit, cut at node boundaries; a node larger than
    /// the limit fills blocks of its own (the renderer draws it per block).
    pub chunks: Vec<Range<u32>>,
}

/// Builds the octree with blocks of at most [`CHUNK_POINTS`] vertices.
pub fn build_octree(points: &PointSet, progress: &Progress) -> Result<Octree, StepError> {
    build_octree_with(points, CHUNK_POINTS, progress)
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

/// Sort key: Morton code, then point index (unique, so any sort gives the same order).
type Key = (u64, u32);

/// A node during the build: its points are `keys[offset..offset + own]` (sample or leaf),
/// its children follow.
struct Temp {
    depth: u32,
    /// Integer position of the cube on the grid of its depth.
    cell: [u64; 3],
    offset: usize,
    own: usize,
    children: Vec<(usize, Temp)>,
}

/// Like [`build_octree`] with another block limit (small blocks for renderer tests).
///
/// The root is the smallest cube at the minimum of the bounds (edge ≥ 1 mm); a point's
/// position on its 2²¹ grid is `floor((p − min) / size · 2²¹)` per axis, computed in `f64`.
/// The points are sorted by (Morton code, index). A node with at most [`LEAF_MAX`] points
/// or at [`MAX_DEPTH`] keeps all of them; otherwise it keeps one point of every cell of its
/// 64³ grid (the one with the smallest hash of its index) and hands the
/// rest, still sorted, to its eight octants.
#[doc(hidden)]
pub fn build_octree_with(
    points: &PointSet,
    chunk_points: u32,
    progress: &Progress,
) -> Result<Octree, StepError> {
    assert!(chunk_points > 0, "blocks hold at least one vertex");
    let n = points.len();
    let n32 =
        u32::try_from(n).map_err(|_| StepError::Failed("Wolke hat zu viele Punkte".into()))?;
    progress.phase(PHASE_BUILD, Some(n as u64));
    check(progress)?;

    let (min, max) = points.bounds();
    let size = (0..3).map(|k| max[k] - min[k]).fold(MIN_CUBE, f32::max);
    let scale = f64::from(1u32 << MORTON_BITS) / f64::from(size);
    let top = (1u64 << MORTON_BITS) - 1;
    let mut keys: Vec<Key> = points
        .xyz
        .par_iter()
        .enumerate()
        .map(|(i, p)| {
            let q = [0, 1, 2].map(|k| {
                let f = (f64::from(p[k]) - f64::from(min[k])) * scale;
                (f.floor().max(0.0) as u64).min(top)
            });
            (
                spread(q[0]) | spread(q[1]) << 1 | spread(q[2]) << 2,
                i as u32,
            )
        })
        .collect();
    keys.par_sort_unstable();
    check(progress)?;

    let root = build_node(&mut keys, 0, 0, [0; 3], progress)?;
    check(progress)?;

    // Number the nodes breadth-first.
    let mut order: Vec<&Temp> = vec![&root];
    let mut nodes = Vec::new();
    let mut first = 0u32;
    let mut i = 0;
    while i < order.len() {
        let t = order[i];
        let mut children = [NO_NODE; 8];
        for (octant, c) in &t.children {
            children[*octant] = order.len() as u32;
            order.push(c);
        }
        let cells = f64::from(1u32 << t.depth);
        let edge = f64::from(size) / cells;
        let count = t.own as u32;
        nodes.push(Node {
            min: [0, 1, 2].map(|k| (f64::from(min[k]) + t.cell[k] as f64 * edge) as f32),
            size: edge as f32,
            spacing: (edge / f64::from(1u32 << NODE_GRID_BITS)) as f32,
            first,
            count,
            children,
            depth: t.depth as u8,
        });
        first += count;
        i += 1;
    }
    debug_assert_eq!(first, n32);

    // Vertices in that order, each node's slice filled in parallel.
    let mut source = vec![0u32; n];
    let mut vertices = vec![PointVertex::zeroed(); n];
    let mut jobs = Vec::with_capacity(order.len());
    let (mut src_rest, mut vtx_rest) = (&mut source[..], &mut vertices[..]);
    for t in &order {
        let (s, sr) = src_rest.split_at_mut(t.own);
        let (v, vr) = vtx_rest.split_at_mut(t.own);
        jobs.push((&keys[t.offset..t.offset + t.own], s, v));
        (src_rest, vtx_rest) = (sr, vr);
    }
    jobs.into_par_iter().for_each(|(k, s, v)| {
        for ((key, s), v) in k.iter().zip(s).zip(v) {
            *s = key.1;
            *v = vertex(points, key.1 as usize);
        }
    });
    drop(keys);

    Ok(Octree {
        origin: points.origin,
        chunks: chunks(&nodes, chunk_points),
        nodes,
        vertices,
        source,
    })
}

fn check(progress: &Progress) -> Result<(), StepError> {
    if progress.is_cancelled() {
        Err(StepError::Cancelled)
    } else {
        Ok(())
    }
}

fn vertex(points: &PointSet, i: usize) -> PointVertex {
    let it = u32::from(points.intensity[i]);
    let [r, g, b] = points.rgb.as_ref().map_or([it; 3], |c| c[i].map(u32::from));
    PointVertex {
        pos: points.xyz[i],
        rgbi: r | g << 8 | b << 16 | it << 24,
        extra: u32::from(points.quality[i]) | u32::from(points.flags[i]) << 8,
    }
}

/// The node at `depth` and grid position `cell` over its sorted `keys`, which start at
/// `offset` in the whole key array. Rearranges `keys`: the node's own points first (sample
/// codes are no longer needed and set to 0), then its octants in order.
fn build_node(
    keys: &mut [Key],
    offset: usize,
    depth: u32,
    cell: [u64; 3],
    progress: &Progress,
) -> Result<Temp, StepError> {
    check(progress)?;
    let n = keys.len();
    if n <= LEAF_MAX || depth == MAX_DEPTH {
        progress.advance(n as u64);
        return Ok(Temp {
            depth,
            cell,
            offset,
            own: n,
            children: Vec::new(),
        });
    }
    // Sample: one point of each run of equal 3·(depth + 6)-bit prefix (a cell), the one
    // with the smallest hash of its index. Not the first in Morton order: that one lies
    // near the low corner of its cell, so coarse levels would look like a lattice. Runs are
    // walked backwards, moving the rest to the back in order; writes never pass reads.
    let shift = 3 * (MORTON_BITS - depth - NODE_GRID_BITS);
    let mut sample = Vec::new();
    let mut w = n;
    let mut end = n;
    while end > 0 {
        let cell = keys[end - 1].0 >> shift;
        let mut start = end - 1;
        while start > 0 && keys[start - 1].0 >> shift == cell {
            start -= 1;
        }
        let pick = (start..end)
            .min_by_key(|&j| (sample_hash(keys[j].1), keys[j].1))
            .unwrap_or(start);
        sample.push(keys[pick].1);
        for r in (start..end).rev() {
            if r != pick {
                w -= 1;
                keys[w] = keys[r];
            }
        }
        end = start;
    }
    let own = sample.len();
    debug_assert_eq!(w, own);
    for (slot, &idx) in keys[..own].iter_mut().zip(sample.iter().rev()) {
        *slot = (0, idx);
    }
    drop(sample);
    progress.advance(own as u64);

    // Octants of the rest, by the next 3 bits.
    let octant_shift = 3 * (MORTON_BITS - depth - 1);
    let (_, mut rest) = keys.split_at_mut(own);
    let mut at = offset + own;
    let mut parts = Vec::new();
    for octant in 0..8u64 {
        let len = rest.partition_point(|k| (k.0 >> octant_shift) & 7 <= octant);
        let (part, tail) = rest.split_at_mut(len);
        if len > 0 {
            let child = [0, 1, 2].map(|k| cell[k] * 2 + ((octant >> k) & 1));
            parts.push((octant as usize, part, at, child));
        }
        at += len;
        rest = tail;
    }
    let children = parts
        .into_par_iter()
        .map(|(octant, part, at, child)| {
            build_node(part, at, depth + 1, child, progress).map(|t| (octant, t))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Temp {
        depth,
        cell,
        offset,
        own,
        children,
    })
}

/// Which point of a cell an inner node keeps: a fixed mix of the point index (the same on
/// every platform and for any thread count).
fn sample_hash(index: u32) -> u32 {
    let mut x = index.wrapping_mul(0x9E37_79B9);
    x ^= x >> 16;
    x = x.wrapping_mul(0x85EB_CA6B);
    x ^= x >> 13;
    x = x.wrapping_mul(0xC2B2_AE35);
    x ^ (x >> 16)
}

/// Blocks of at most `limit` vertices: nodes are added while they fit; a node larger than
/// the limit gets blocks of its own.
fn chunks(nodes: &[Node], limit: u32) -> Vec<Range<u32>> {
    let mut out = Vec::new();
    let mut current = 0..0;
    for node in nodes.iter().filter(|n| n.count > 0) {
        let end = node.first + node.count;
        if node.count > limit {
            if !current.is_empty() {
                out.push(current.clone());
            }
            let mut start = node.first;
            while start < end {
                let stop = end.min(start + limit);
                out.push(start..stop);
                start = stop;
            }
            current = end..end;
        } else if end - current.start > limit {
            out.push(current.clone());
            current = node.first..end;
        } else {
            current.end = end;
        }
    }
    if !current.is_empty() {
        out.push(current);
    }
    out
}

/// The view volume: five planes `nx·x + ny·y + nz·z + d`, inside where ≥ 0 (left, right,
/// bottom, top, near; no far plane).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frustum {
    pub planes: [[f32; 4]; 5],
}

impl Frustum {
    /// Whether the box may touch the volume (conservative: `true` near corners).
    pub fn intersects_aabb(&self, min: [f32; 3], max: [f32; 3]) -> bool {
        self.planes.iter().all(|p| {
            let corner = [0, 1, 2].map(|k| if p[k] >= 0.0 { max[k] } else { min[k] });
            p[0] * corner[0] + p[1] * corner[1] + p[2] * corner[2] + p[3] >= 0.0
        })
    }
}

/// What [`select_nodes`] needs to know about the camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewInfo {
    pub frustum: Frustum,
    pub eye: [f32; 3],
    pub near: f32,
    /// Viewport height in pixels over the vertical field of view.
    pub pixels_per_radian: f32,
    /// A node's children are only considered while its point spacing on screen is larger.
    pub min_spacing_px: f32,
}

/// Point spacing of `node` on screen (pixels), at the nearest point of its cube: a large
/// node next to or around the eye counts as near, not as far as its centre.
fn screen_spacing(node: &Node, view: &ViewInfo) -> f32 {
    let max = node.max();
    let d = (0..3)
        .map(|k| (view.eye[k].clamp(node.min[k], max[k]) - view.eye[k]).powi(2))
        .sum::<f32>()
        .sqrt();
    node.spacing * view.pixels_per_radian / d.max(view.near)
}

/// Heap entry: the largest screen spacing first, on a tie the smaller node index.
struct Candidate {
    spacing: f32,
    node: u32,
}

impl PartialEq for Candidate {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Candidate {}

impl PartialOrd for Candidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Candidate {
    fn cmp(&self, other: &Self) -> Ordering {
        self.spacing
            .total_cmp(&other.spacing)
            .then_with(|| other.node.cmp(&self.node))
    }
}

/// The nodes to draw, at most `budget` points in all: starting at the root, always the node
/// with the largest point spacing on screen next, as long as it fits the budget (the first
/// that does not ends the choice, except a leaf over [`LEAF_MAX`] points, which is only
/// skipped). Only nodes in the view volume, only children of chosen
/// nodes, and those only while the parent's spacing on screen exceeds `min_spacing_px`.
pub fn select_nodes(tree: &Octree, view: &ViewInfo, budget: usize) -> Vec<u32> {
    let mut out = Vec::new();
    let Some(root) = tree.nodes.first() else {
        return out;
    };
    let mut heap = BinaryHeap::new();
    if view.frustum.intersects_aabb(root.min, root.max()) {
        heap.push(Candidate {
            spacing: screen_spacing(root, view),
            node: 0,
        });
    }
    let mut used = 0usize;
    while let Some(Candidate { spacing, node }) = heap.pop() {
        let n = &tree.nodes[node as usize];
        let count = n.count as usize;
        if used + count > budget {
            // An oversized leaf (many equal points at MAX_DEPTH) is skipped, so that it
            // does not end the choice for the whole frame; any other node ends it.
            if n.is_leaf() && count > LEAF_MAX {
                continue;
            }
            break;
        }
        used += count;
        out.push(node);
        if spacing > view.min_spacing_px {
            for &c in n.children.iter().filter(|&&c| c != NO_NODE) {
                let child = &tree.nodes[c as usize];
                if view.frustum.intersects_aabb(child.min, child.max()) {
                    heap.push(Candidate {
                        spacing: screen_spacing(child, view),
                        node: c,
                    });
                }
            }
        }
    }
    out
}
