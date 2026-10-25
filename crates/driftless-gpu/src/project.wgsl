// Projects world points into one camera (appended to camera.wgsl).

struct Params {
    cam: Camera,
    // Rows of cam_from_world (rotation | translation).
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
    count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
// x, y, z per point.
@group(0) @binding(1) var<storage, read> points: array<f32>;
// Pixel coordinates; (-1, -1) for points that do not project.
@group(0) @binding(2) var<storage, read_write> out_uv: array<vec2<f32>>;

@compute @workgroup_size(64)
fn project_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.count {
        return;
    }
    let w = vec4<f32>(points[3u * i], points[3u * i + 1u], points[3u * i + 2u], 1.0);
    let p = vec3<f32>(dot(params.row0, w), dot(params.row1, w), dot(params.row2, w));
    let q = camera_project(params.cam, p);
    out_uv[i] = select(vec2<f32>(-1.0), q.xy, q.z > 0.5);
}
