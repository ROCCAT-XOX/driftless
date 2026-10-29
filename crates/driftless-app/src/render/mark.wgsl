// The marked Stellen over the surface: copies of their triangles (three corners each, not
// indexed), tinted in the Stelle's colour and outlined along its rim.

// The same block as in mesh.wgsl (`MeshUniforms` in mesh.rs).
struct Mesh {
    view_proj: mat4x4<f32>,
    // Added to every position: the surface's origin minus the view's (xyz).
    offset: vec4<f32>,
    // Height cut: fragments with z outside x..y are not drawn.
    z_range: vec4<f32>,
    // x: width of the outline (physical pixels).
    mark: vec4<f32>,
};

@group(0) @binding(0) var<uniform> mesh: Mesh;

struct Out {
    // Invariant and computed exactly as in mesh.wgsl: the copy has the depth of the triangle
    // it lies on bit for bit, so it passes `GreaterEqual` where that one is seen.
    @builtin(position) @invariant clip: vec4<f32>,
    // Barycentric coordinates (1 at the corner, 0 at the other two), linear on the screen so
    // that their gradients give pixels.
    @location(0) @interpolate(linear) bary: vec3<f32>,
    // Bit k: edge k (corner k to corner k + 1) lies on the rim; bit 3 + k: corner k does.
    @location(1) @interpolate(flat) rim: u32,
    // The Stelle's colour; alpha: its tint.
    @location(2) @interpolate(flat) color: vec4<f32>,
    @location(3) z: f32,
};

@vertex
fn vs_main(
    @builtin(vertex_index) index: u32,
    @location(0) pos: vec3<f32>,
    @location(1) rim: u32,
    @location(2) color: vec4<f32>,
) -> Out {
    let p = pos + mesh.offset.xyz;
    var out: Out;
    out.clip = mesh.view_proj * vec4<f32>(p, 1.0);
    let corner = index % 3u;
    out.bary = vec3<f32>(f32(corner == 0u), f32(corner == 1u), f32(corner == 2u));
    out.rim = rim;
    out.color = color;
    out.z = p.z;
    return out;
}

// `d` where `bit` of `rim` is set, else far away.
fn on_rim(d: f32, rim: u32, bit: u32) -> f32 {
    return select(1e30, d, (rim & bit) != 0u);
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    // Derivatives before the cut: they need uniform control flow.
    let dx = dpdx(in.bary);
    let dy = dpdy(in.bary);
    if in.z < mesh.z_range.x || in.z > mesh.z_range.y {
        discard;
    }
    // Pixels to the line of each edge: edge k lies opposite corner k + 2, whose coordinate is
    // 0 along it.
    let to_line = in.bary / max(sqrt(dx * dx + dy * dy), vec3<f32>(1e-20));
    // Pixels to each corner: the offsets on the screen from corner 0 to corners 1 and 2 are
    // the inverse of the gradients of their coordinates (a triangle seen edge-on has none).
    let det = dx.y * dy.z - dx.z * dy.y;
    let inv = select(0.0, 1.0 / det, abs(det) > 1e-30);
    let e1 = vec2<f32>(dy.z, -dx.z) * inv;
    let e2 = vec2<f32>(-dy.y, dx.y) * inv;
    let from0 = in.bary.y * e1 + in.bary.z * e2;
    let to_corner = vec3<f32>(length(from0), length(from0 - e1), length(from0 - e2));
    let d = min(
        min(on_rim(to_line.z, in.rim, 1u), min(on_rim(to_line.x, in.rim, 2u), on_rim(to_line.y, in.rim, 4u))),
        min(on_rim(to_corner.x, in.rim, 8u), min(on_rim(to_corner.y, in.rim, 16u), on_rim(to_corner.z, in.rim, 32u))),
    );
    // The outline: opaque up to half a pixel before its width, fading out over one pixel.
    let line = clamp(mesh.mark.x + 0.5 - d, 0.0, 1.0);
    return vec4<f32>(in.color.rgb, mix(in.color.a, 1.0, line));
}
