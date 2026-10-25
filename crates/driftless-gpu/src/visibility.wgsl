// Depth buffer and per-point visibility of one image (appended to camera.wgsl).
// Mirrors `driftless_map::visibility::{project_point, DepthBuffer}`.

struct Vis {
    cam: Camera,
    // Rows of cam_from_world, world relative to the cloud's origin (rotation | translation).
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
    // Depth-buffer size in cells.
    depth_w: u32,
    depth_h: u32,
    // Pixels per cell edge.
    scale: f32,
    // Focal length in cells per radian (the larger of fx, fy over `scale`).
    focal: f32,
    max_dist: f32,
    tol_abs: f32,
    tol_rel: f32,
    sil_rel: f32,
    max_splat: i32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> vis: Vis;
// Per point x, y, z (relative to the origin) and its spacing (m).
@group(0) @binding(1) var<storage, read> points: array<vec4<f32>>;
// Per cell the bitwise complement of the minimum range's f32 bits; 0 = empty. The complement
// turns the minimum into an atomicMax and lets a zero fill clear the buffer.
@group(0) @binding(2) var<storage, read_write> depth: array<atomic<u32>>;
// One bit per point of the batch: seen by the image.
@group(0) @binding(3) var<storage, read_write> visible: array<atomic<u32>>;

struct Proj {
    cell: vec2<i32>,
    range: f32,
    ok: bool,
}

// Projection of point `p` into the depth buffer; not ok beyond `max_dist`, at the camera
// centre, outside the lens model or outside the image.
fn vis_project(p: vec3<f32>) -> Proj {
    var out: Proj;
    out.ok = false;
    let w = vec4<f32>(p, 1.0);
    let pc = vec3<f32>(dot(vis.row0, w), dot(vis.row1, w), dot(vis.row2, w));
    let range = length(pc);
    if range > vis.max_dist || range == 0.0 {
        return out;
    }
    let q = camera_project(vis.cam, pc);
    if q.z < 0.5 {
        return out;
    }
    let max_cell = vec2<i32>(i32(vis.depth_w) - 1, i32(vis.depth_h) - 1);
    out.cell = clamp(vec2<i32>((q.xy + 0.5) / vis.scale), vec2<i32>(0), max_cell);
    out.range = range;
    out.ok = true;
    return out;
}

// Minimum range of cell (x, y); `ok` false where the cell is empty (infinitely far).
fn vis_depth(x: i32, y: i32) -> vec2<f32> {
    let stored = atomicLoad(&depth[u32(y) * vis.depth_w + u32(x)]);
    if stored == 0u {
        return vec2<f32>(0.0, 0.0);
    }
    return vec2<f32>(bitcast<f32>(~stored), 1.0);
}

@compute @workgroup_size(256)
fn splat_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= arrayLength(&points) {
        return;
    }
    let p = points[i];
    let pr = vis_project(p.xyz);
    if !pr.ok {
        return;
    }
    // Footprint of the point in cells (its spacing seen at its range), half-size clamped.
    let cells = p.w * vis.focal / pr.range;
    let r = i32(clamp(ceil(0.5 * cells - 0.5), 0.0, f32(vis.max_splat)));
    let bits = ~bitcast<u32>(pr.range);
    let y0 = max(pr.cell.y - r, 0);
    let y1 = min(pr.cell.y + r, i32(vis.depth_h) - 1);
    let x0 = max(pr.cell.x - r, 0);
    let x1 = min(pr.cell.x + r, i32(vis.depth_w) - 1);
    for (var y = y0; y <= y1; y++) {
        for (var x = x0; x <= x1; x++) {
            atomicMax(&depth[u32(y) * vis.depth_w + u32(x)], bits);
        }
    }
}

@compute @workgroup_size(256)
fn visible_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= arrayLength(&points) {
        return;
    }
    let pr = vis_project(points[i].xyz);
    if !pr.ok {
        return;
    }
    let own = vis_depth(pr.cell.x, pr.cell.y);
    if own.y > 0.5 && pr.range > own.x * (1.0 + vis.tol_rel) + vis.tol_abs {
        return;
    }
    // No neighbouring cell holds an occluder whose silhouette the point lies on.
    let y0 = max(pr.cell.y - 1, 0);
    let y1 = min(pr.cell.y + 1, i32(vis.depth_h) - 1);
    let x0 = max(pr.cell.x - 1, 0);
    let x1 = min(pr.cell.x + 1, i32(vis.depth_w) - 1);
    for (var y = y0; y <= y1; y++) {
        for (var x = x0; x <= x1; x++) {
            let d = vis_depth(x, y);
            if d.y > 0.5 && pr.range > d.x * (1.0 + vis.sil_rel) + vis.tol_abs {
                return;
            }
        }
    }
    atomicOr(&visible[i / 32u], 1u << (i % 32u));
}
