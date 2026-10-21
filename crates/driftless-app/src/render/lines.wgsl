// Lines of one pixel, coloured per vertex.

struct Frame {
    view_proj: mat4x4<f32>,
    viewport_px: vec2<f32>,
    point_px: f32,
    color_mode: u32,
    round: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0) var<uniform> frame: Frame;

struct Out {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(@location(0) pos: vec3<f32>, @location(1) rgba: u32) -> Out {
    var out: Out;
    out.clip = frame.view_proj * vec4<f32>(pos, 1.0);
    out.color = unpack4x8unorm(rgba);
    return out;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    return in.color;
}
