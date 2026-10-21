//! The octree with levels of detail and the choice of nodes to draw.

use driftless_app::cloud::PointSet;
use driftless_app::lod::{
    build_octree, build_octree_with, select_nodes, Frustum, Octree, ViewInfo, LEAF_MAX, MAX_DEPTH,
    NODE_GRID_BITS, NO_NODE,
};
use driftless_app::synthetic::{synthetic_cloud, Rng};
use driftless_io::progress::Progress;
use std::collections::HashSet;

const MORTON_BITS: u32 = 21;

fn random_points(n: usize, seed: u64, extent: [f32; 3]) -> PointSet {
    let mut rng = Rng::new(seed);
    let xyz = (0..n).map(|_| extent.map(|e| rng.next_f32() * e)).collect();
    PointSet::from_xyz([0.0; 3], xyz)
}

fn build(points: &PointSet) -> Octree {
    build_octree(points, &Progress::new()).unwrap()
}

/// Integer position of a point on the 2²¹ grid of the root cube, as the octree quantises it.
fn grid(tree: &Octree, p: [f32; 3]) -> [u64; 3] {
    let root = &tree.nodes[0];
    let max = (1u64 << MORTON_BITS) - 1;
    [0, 1, 2].map(|k| {
        let f = (f64::from(p[k]) - f64::from(root.min[k])) / f64::from(root.size)
            * (1u64 << MORTON_BITS) as f64;
        (f.floor().max(0.0) as u64).min(max)
    })
}

fn is_leaf(tree: &Octree, i: usize) -> bool {
    tree.nodes[i].children.iter().all(|&c| c == NO_NODE)
}

fn parents(tree: &Octree) -> Vec<u32> {
    let mut parent = vec![NO_NODE; tree.nodes.len()];
    for (i, n) in tree.nodes.iter().enumerate() {
        for &c in &n.children {
            if c != NO_NODE {
                parent[c as usize] = i as u32;
            }
        }
    }
    parent
}

fn total_count(tree: &Octree, ids: &[u32]) -> usize {
    ids.iter()
        .map(|&i| tree.nodes[i as usize].count as usize)
        .sum()
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    v.map(|x| x / l)
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// A perspective view from `eye` towards `target` (z up), 60° vertical, aspect 4:3,
/// 1000 pixels high.
fn view(eye: [f32; 3], target: [f32; 3], min_spacing_px: f32) -> ViewInfo {
    view_near(eye, target, min_spacing_px, 0.05)
}

fn view_near(eye: [f32; 3], target: [f32; 3], min_spacing_px: f32, near: f32) -> ViewInfo {
    let fov_y = 60f32.to_radians();
    let aspect = 4.0 / 3.0;
    let f = normalize([0, 1, 2].map(|k| target[k] - eye[k]));
    let r = normalize(cross(f, [0.0, 0.0, 1.0]));
    let u = cross(r, f);
    let hy = fov_y / 2.0;
    let hx = (hy.tan() * aspect).atan();
    let side = |axis: [f32; 3], sign: f32, h: f32| {
        [0, 1, 2].map(|k| sign * axis[k] * h.cos() + f[k] * h.sin())
    };
    let plane = |n: [f32; 3], d: f32| [n[0], n[1], n[2], d];
    let normals = [
        side(r, 1.0, hx),
        side(r, -1.0, hx),
        side(u, 1.0, hy),
        side(u, -1.0, hy),
    ];
    let mut planes = [[0.0; 4]; 5];
    for (p, n) in planes.iter_mut().zip(normals) {
        *p = plane(n, -dot(n, eye));
    }
    planes[4] = plane(f, -dot(f, eye) - near);
    ViewInfo {
        frustum: Frustum { planes },
        eye,
        near,
        pixels_per_radian: 1000.0 / fov_y,
        min_spacing_px,
    }
}

#[test]
fn every_point_exactly_once() {
    let points = random_points(1_000_000, 7, [50.0, 20.0, 3.0]);
    let tree = build(&points);
    assert_eq!(tree.vertices.len(), points.len());
    let mut seen = tree.source.clone();
    seen.sort_unstable();
    assert!(seen.iter().enumerate().all(|(i, &s)| s as usize == i));
    let sum: usize = tree.nodes.iter().map(|n| n.count as usize).sum();
    assert_eq!(sum, points.len());
    // Nodes in breadth-first order own consecutive vertex ranges.
    let mut next = 0;
    for n in &tree.nodes {
        assert_eq!(n.first, next);
        next += n.count;
    }
    assert!(tree.nodes.len() > 8, "{} nodes", tree.nodes.len());
}

#[test]
fn points_inside_their_node() {
    let points = random_points(300_000, 11, [30.0, 10.0, 4.0]);
    let tree = build(&points);
    for (i, n) in tree.nodes.iter().enumerate() {
        assert!((n.spacing - n.size / 64.0).abs() <= n.size * 1e-6);
        let cell_bits = MORTON_BITS - u32::from(n.depth);
        let cell = (n.count > 0).then(|| {
            let p = tree.vertices[n.first as usize].pos;
            grid(&tree, p).map(|g| g >> cell_bits)
        });
        for v in n.first..n.first + n.count {
            let p = tree.vertices[v as usize].pos;
            assert_eq!(p, points.xyz[tree.source[v as usize] as usize]);
            let eps = n.size * 1e-4;
            for k in 0..3 {
                assert!(
                    p[k] >= n.min[k] - eps && p[k] <= n.min[k] + n.size + eps,
                    "node {i} axis {k}: {p:?} outside {:?} + {}",
                    n.min,
                    n.size
                );
            }
            // The same cell on the Morton grid.
            assert_eq!(
                Some(grid(&tree, p).map(|g| g >> cell_bits)),
                cell,
                "node {i}"
            );
        }
        for (k, &c) in n.children.iter().enumerate() {
            if c != NO_NODE {
                let child = &tree.nodes[c as usize];
                assert_eq!(child.depth, n.depth + 1);
                assert!(c as usize > i, "breadth-first: child after parent");
                let off = [k & 1, (k >> 1) & 1, k >> 2];
                for (a, o) in off.iter().enumerate() {
                    let want = n.min[a] + *o as f32 * n.size / 2.0;
                    assert!((child.min[a] - want).abs() <= n.size * 1e-5);
                }
            }
        }
    }
}

#[test]
fn inner_nodes_hold_one_point_per_cell() {
    let points = random_points(500_000, 13, [20.0, 20.0, 5.0]);
    let tree = build(&points);
    let mut inner = 0;
    for (i, n) in tree.nodes.iter().enumerate() {
        if is_leaf(&tree, i) {
            continue;
        }
        inner += 1;
        let shift = MORTON_BITS - u32::from(n.depth) - NODE_GRID_BITS;
        let mut cells = HashSet::new();
        for v in n.first..n.first + n.count {
            let g = grid(&tree, tree.vertices[v as usize].pos).map(|g| g >> shift);
            assert!(cells.insert(g), "node {i}: two points in cell {g:?}");
        }
        assert!(n.count as usize <= 1 << (3 * NODE_GRID_BITS));
        // Every occupied cell of the node is represented: the points of its children fall
        // into cells it already holds.
        let mut stack: Vec<u32> = n
            .children
            .iter()
            .copied()
            .filter(|&c| c != NO_NODE)
            .collect();
        while let Some(c) = stack.pop() {
            let c = &tree.nodes[c as usize];
            for v in c.first..c.first + c.count {
                let g = grid(&tree, tree.vertices[v as usize].pos).map(|g| g >> shift);
                assert!(cells.contains(&g), "node {i}: cell {g:?} without sample");
            }
            stack.extend(c.children.iter().copied().filter(|&c| c != NO_NODE));
        }
    }
    assert!(inner > 0);
}

#[test]
fn leaves_are_small() {
    let points = synthetic_cloud(1_000_000, 3);
    let tree = build(&points);
    for (i, n) in tree.nodes.iter().enumerate() {
        if is_leaf(&tree, i) {
            assert!(
                n.count as usize <= LEAF_MAX || u32::from(n.depth) == MAX_DEPTH,
                "leaf {i}: {} points at depth {}",
                n.count,
                n.depth
            );
        }
        assert!(u32::from(n.depth) <= MAX_DEPTH);
    }
}

/// Blocks are cut greedily at node boundaries; a node larger than the limit fills blocks of
/// its own.
fn check_chunks(tree: &Octree, limit: u32) {
    let n = tree.vertices.len() as u32;
    let mut starts = HashSet::new();
    for node in &tree.nodes {
        starts.insert(node.first);
    }
    starts.insert(n);
    let mut at = 0;
    for c in &tree.chunks {
        assert_eq!(c.start, at, "blocks are contiguous");
        assert!(c.end > c.start, "no empty block");
        assert!(c.end - c.start <= limit, "block {c:?} over {limit}");
        for edge in [c.start, c.end] {
            if !starts.contains(&edge) {
                let owner = tree
                    .nodes
                    .iter()
                    .find(|nd| nd.first < edge && edge < nd.first + nd.count)
                    .unwrap();
                assert!(owner.count > limit, "cut inside a node that fits: {c:?}");
            }
        }
        at = c.end;
    }
    assert_eq!(at, n, "blocks cover all vertices");
}

#[test]
fn chunks_respect_limit_and_node_boundaries() {
    let points = random_points(400_000, 17, [40.0, 10.0, 3.0]);
    for limit in [4, 1000, 50_000, 300_000] {
        let tree = build_octree_with(&points, limit, &Progress::new()).unwrap();
        check_chunks(&tree, limit);
    }
    let tree = build(&points);
    assert_eq!(tree.chunks, vec![0..points.len() as u32]);
    // A leaf of identical points larger than the block limit is split as well.
    let same = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]; 100_000]);
    let tree = build_octree_with(&same, 10_000, &Progress::new()).unwrap();
    check_chunks(&tree, 10_000);
}

fn same_tree(a: &Octree, b: &Octree) {
    assert_eq!(a.origin, b.origin);
    assert_eq!(a.nodes, b.nodes);
    assert_eq!(
        bytemuck::cast_slice::<_, u8>(&a.vertices),
        bytemuck::cast_slice::<_, u8>(&b.vertices)
    );
    assert_eq!(a.source, b.source);
    assert_eq!(a.chunks, b.chunks);
}

#[test]
fn octree_is_deterministic() {
    let points = synthetic_cloud(600_000, 5);
    let pool = |threads| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
    };
    let one = pool(1).install(|| build(&points));
    let many = pool(8).install(|| build(&points));
    let default = build(&points);
    same_tree(&one, &many);
    same_tree(&one, &default);
}

#[test]
fn degenerate_clouds_build() {
    let empty = PointSet::from_xyz([0.0; 3], Vec::new());
    let tree = build(&empty);
    assert_eq!(tree.nodes.len(), 1);
    assert_eq!(tree.nodes[0].count, 0);
    assert!(tree.vertices.is_empty() && tree.source.is_empty() && tree.chunks.is_empty());
    assert!(tree.nodes[0].size > 0.0);

    let one = PointSet::from_xyz([512_000.0, 5_400_000.0, 300.0], vec![[0.5, -0.25, 1.0]]);
    let tree = build(&one);
    assert_eq!(tree.origin, one.origin);
    assert_eq!(tree.nodes.len(), 1);
    assert_eq!(tree.source, vec![0]);
    assert_eq!(tree.vertices[0].pos, [0.5, -0.25, 1.0]);
    assert!(tree.nodes[0].size >= 0.001);

    let same = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]; 100_000]);
    let tree = build(&same);
    let mut seen = tree.source.clone();
    seen.sort_unstable();
    assert!(seen.iter().enumerate().all(|(i, &s)| s as usize == i));
    let deepest = tree.nodes.iter().map(|n| u32::from(n.depth)).max().unwrap();
    assert!(deepest <= MAX_DEPTH);

    // Two clusters far apart and one point between them.
    let mut xyz = vec![[0.0, 0.0, 0.0]; 40_000];
    xyz.extend(vec![[100.0, 100.0, 100.0]; 40_000]);
    xyz.push([50.0, 50.0, 50.0]);
    let tree = build(&PointSet::from_xyz([0.0; 3], xyz));
    assert_eq!(tree.source.len(), 80_001);
}

#[test]
fn build_cancelled_returns_cancelled() {
    let points = random_points(200_000, 19, [10.0, 10.0, 3.0]);
    let progress = Progress::new();
    progress.cancel();
    let r = build_octree(&points, &progress);
    assert_eq!(r.err(), Some(driftless_steps::StepError::Cancelled));
}

#[test]
fn build_reports_phase() {
    let points = random_points(100_000, 23, [10.0, 10.0, 3.0]);
    let progress = Progress::new();
    build_octree(&points, &progress).unwrap();
    let s = progress.snapshot();
    assert_eq!(s.phase, "Detailstufen bauen");
    assert_eq!(s.total, Some(100_000));
    assert_eq!(s.done, 100_000);
}

#[test]
fn vertex_packs_colour_intensity_quality_and_flags() {
    let mut points = PointSet::from_xyz([0.0; 3], vec![[0.0; 3], [1.0, 1.0, 1.0]]);
    points.intensity = vec![10, 20];
    points.quality = vec![30, 40];
    points.flags = vec![1, 0];
    let gray = build(&points);
    for (v, &s) in gray.vertices.iter().zip(&gray.source) {
        let s = s as usize;
        let i = u32::from(points.intensity[s]);
        assert_eq!(v.rgbi, i | i << 8 | i << 16 | i << 24);
        assert_eq!(
            v.extra,
            u32::from(points.quality[s]) | u32::from(points.flags[s]) << 8
        );
    }
    points.rgb = Some(vec![[1, 2, 3], [4, 5, 6]]);
    let color = build(&points);
    for (v, &s) in color.vertices.iter().zip(&color.source) {
        let s = s as usize;
        let [r, g, b] = points.rgb.as_ref().unwrap()[s].map(u32::from);
        let i = u32::from(points.intensity[s]);
        assert_eq!(v.rgbi, r | g << 8 | b << 16 | i << 24);
    }
    assert_eq!(std::mem::size_of_val(&color.vertices[0]), 20);
}

#[test]
fn frustum_tests_boxes() {
    let v = view([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.0);
    let f = &v.frustum;
    assert!(f.intersects_aabb([5.0, -1.0, -1.0], [6.0, 1.0, 1.0]));
    assert!(
        f.intersects_aabb([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]),
        "eye inside"
    );
    assert!(
        !f.intersects_aabb([-6.0, -1.0, -1.0], [-5.0, 1.0, 1.0]),
        "behind"
    );
    assert!(
        !f.intersects_aabb([5.0, 20.0, -1.0], [6.0, 21.0, 1.0]),
        "left of view"
    );
    assert!(
        f.intersects_aabb([1e6, -1.0, -1.0], [1e6 + 1.0, 1.0, 1.0]),
        "far is open"
    );
}

#[test]
fn selection_respects_budget() {
    let points = synthetic_cloud(1_000_000, 1);
    let tree = build(&points);
    let (min, max) = points.bounds();
    let centre = [0, 1, 2].map(|k| (min[k] + max[k]) / 2.0);
    let eye = [centre[0] - 40.0, centre[1] - 40.0, centre[2] + 30.0];
    for budget in [0, 1, 50_000, 300_000] {
        let ids = select_nodes(&tree, &view(eye, centre, 1.0), budget);
        assert!(total_count(&tree, &ids) <= budget);
        let unique: HashSet<_> = ids.iter().collect();
        assert_eq!(unique.len(), ids.len());
    }
    assert!(!select_nodes(&tree, &view(eye, centre, 1.0), 300_000).is_empty());
}

#[test]
fn selection_is_ancestor_closed() {
    let points = synthetic_cloud(1_000_000, 2);
    let tree = build(&points);
    let parent = parents(&tree);
    let (min, max) = points.bounds();
    let eye = [
        min[0] + 1.0,
        (min[1] + max[1]) / 2.0,
        (min[2] + max[2]) / 2.0,
    ];
    let ids = select_nodes(&tree, &view(eye, max, 0.5), 400_000);
    assert!(ids.len() > 1);
    let set: HashSet<u32> = ids.iter().copied().collect();
    for &i in &ids {
        if i != 0 {
            assert!(set.contains(&parent[i as usize]), "node {i} without parent");
        }
    }
}

#[test]
fn near_nodes_refined_first() {
    // Long corridor 120 x 2 x 2.5 m: floor, ceiling and both walls.
    let mut rng = Rng::new(42);
    let xyz: Vec<[f32; 3]> = (0..1_200_000)
        .map(|i| {
            let x = rng.next_f32() * 120.0;
            let a = rng.next_f32();
            match i % 4 {
                0 => [x, a * 2.0, 0.0],
                1 => [x, a * 2.0, 2.5],
                2 => [x, 0.0, a * 2.5],
                _ => [x, 2.0, a * 2.5],
            }
        })
        .collect();
    let tree = build(&PointSet::from_xyz([0.0; 3], xyz));
    let ids = select_nodes(
        &tree,
        &view([0.2, 1.0, 1.25], [120.0, 1.0, 1.25], 1.0),
        150_000,
    );
    let depth_where = |near: bool| {
        ids.iter()
            .map(|&i| &tree.nodes[i as usize])
            .filter(|n| {
                let cx = n.min[0] + n.size / 2.0;
                if near {
                    cx < 15.0
                } else {
                    cx > 80.0
                }
            })
            .map(|n| n.depth)
            .max()
            .unwrap_or(0)
    };
    let (near, far) = (depth_where(true), depth_where(false));
    assert!(near > far, "near depth {near}, far depth {far}");
}

#[test]
fn large_budget_draws_everything() {
    let points = synthetic_cloud(300_000, 9);
    let tree = build(&points);
    let (min, max) = points.bounds();
    let centre = [0, 1, 2].map(|k| (min[k] + max[k]) / 2.0);
    let size = (0..3).map(|k| max[k] - min[k]).fold(0.0, f32::max);
    let eye = [centre[0], centre[1] - 3.0 * size, centre[2] + 2.0 * size];
    let ids = select_nodes(&tree, &view(eye, centre, 0.0), points.len());
    assert_eq!(total_count(&tree, &ids), points.len());
    assert_eq!(ids.len(), tree.nodes.len());
}

#[test]
fn outside_frustum_not_selected() {
    let points = random_points(200_000, 29, [10.0, 10.0, 10.0]);
    let tree = build(&points);
    // Looking away from the cloud.
    let away = view([-10.0, 5.0, 5.0], [-20.0, 5.0, 5.0], 0.1);
    assert!(select_nodes(&tree, &away, 1_000_000).is_empty());
    // Looking at one corner from inside: only nodes the view touches, not all of them.
    let corner = view([5.0, 5.0, 5.0], [10.0, 10.0, 10.0], 0.0);
    let ids = select_nodes(&tree, &corner, 1_000_000);
    assert!(!ids.is_empty() && ids.len() < tree.nodes.len());
    for &i in &ids {
        let n = &tree.nodes[i as usize];
        let max = [0, 1, 2].map(|k| n.min[k] + n.size);
        assert!(corner.frustum.intersects_aabb(n.min, max), "node {i}");
    }
}

#[test]
fn refinement_stops_at_min_spacing() {
    let points = synthetic_cloud(500_000, 4);
    let tree = build(&points);
    let parent = parents(&tree);
    let (min, max) = points.bounds();
    let centre = [0, 1, 2].map(|k| (min[k] + max[k]) / 2.0);
    let eye = [centre[0], centre[1] - 20.0, centre[2] + 10.0];
    // Never refine: only the root.
    assert_eq!(
        select_nodes(&tree, &view(eye, centre, 1e9), points.len()),
        vec![0]
    );
    let v = view(eye, centre, 2.0);
    let ids = select_nodes(&tree, &v, points.len());
    let on_screen = |i: u32| {
        let n = &tree.nodes[i as usize];
        let c = [0, 1, 2].map(|k| n.min[k] + n.size / 2.0);
        let d = dot(
            [0, 1, 2].map(|k| c[k] - eye[k]),
            [0, 1, 2].map(|k| c[k] - eye[k]),
        )
        .sqrt();
        n.spacing * v.pixels_per_radian / d.max(v.near)
    };
    assert!(ids.len() > 1 && ids.len() < tree.nodes.len());
    for &i in &ids[1..] {
        assert!(on_screen(parent[i as usize]) > 2.0, "node {i}");
    }
}

/// A huge leaf of equal points right at the eye comes up first but does not fit the budget:
/// it is skipped and everything else in view is still drawn.
#[test]
fn oversized_leaf_does_not_end_selection() {
    let mut points = random_points(200_000, 31, [1000.0, 1000.0, 1000.0]);
    points.xyz.extend(vec![[999.0, 999.0, 999.0]; 1_000_000]);
    let points = PointSet::from_xyz([0.0; 3], points.xyz);
    let tree = build(&points);
    let huge = tree
        .nodes
        .iter()
        .position(|n| n.count as usize > LEAF_MAX && n.children.iter().all(|&c| c == NO_NODE))
        .expect("an oversized leaf") as u32;
    assert_eq!(u32::from(tree.nodes[huge as usize].depth), MAX_DEPTH);
    // Eye right behind the leaf, looking through it: the leaf has the largest spacing on
    // screen of all nodes.
    let leaf = &tree.nodes[huge as usize];
    let eye = leaf.max().map(|m| m + 1e-4);
    let v = view_near(eye, [500.0, 500.0, 500.0], 0.0, 1e-5);
    let first = select_nodes(&tree, &v, usize::MAX);
    assert!(first.contains(&huge), "leaf in view");
    let ids = select_nodes(&tree, &v, 500_000);
    assert!(!ids.contains(&huge));
    let chosen: HashSet<u32> = ids.iter().copied().collect();
    let want: HashSet<u32> = (0..tree.nodes.len() as u32)
        .filter(|&i| i != huge)
        .filter(|&i| {
            let n = &tree.nodes[i as usize];
            v.frustum.intersects_aabb(n.min, n.max())
        })
        .collect();
    assert_eq!(chosen, want);
    assert!(total_count(&tree, &ids) <= 500_000);
}
