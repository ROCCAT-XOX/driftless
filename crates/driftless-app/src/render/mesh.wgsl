// The textured surface: unlit triangles showing their photo page as it is.

struct Mesh {
    view_proj: mat4x4<f32>,
    // Added to every position: the surface's origin minus the view's (xyz).
    offset: vec4<f32>,
    // Height cut: fragments with z outside x..y are not drawn.
    z_range: vec4<f32>,
};

@group(0) @binding(0) var<uniform> mesh: Mesh;
@group(1) @binding(0) var page: texture_2d<f32>;
@group(1) @binding(1) var page_sampler: sampler;

struct Out {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) z: f32,
};

@vertex
fn vs_main(@location(0) pos: vec3<f32>, @location(1) uv: vec2<f32>) -> Out {
    let p = pos + mesh.offset.xyz;
    var out: Out;
    out.clip = mesh.view_proj * vec4<f32>(p, 1.0);
    out.uv = uv;
    out.z = p.z;
    return out;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    // Sampled before the cut: textureSample needs uniform control flow.
    let color = textureSample(page, page_sampler, in.uv);
    if in.z < mesh.z_range.x || in.z > mesh.z_range.y {
        discard;
    }
    return vec4<f32>(color.rgb, 1.0);
}
