// Points as screen-aligned quads: one instance per point, four corners as a triangle strip.

struct Frame {
    view_proj: mat4x4<f32>,
    viewport_px: vec2<f32>,
    point_px: f32,
    // 0 = RGB, 1 = intensity, 2 = quality (see `ColorMode`).
    color_mode: u32,
    // 1 = discard outside the circle.
    round: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0) var<uniform> frame: Frame;

struct Out {
    @builtin(position) clip: vec4<f32>,
    // The corner in −1..1 on both axes.
    @location(0) corner: vec2<f32>,
    @location(1) color: vec3<f32>,
};

// The quality ramp of `driftless_steps::overlay::quality_color`: red (0) via yellow (0.5)
// to green (1).
fn quality_color(q: f32) -> vec3<f32> {
    if q < 0.5 {
        return vec3<f32>(1.0, q * 2.0, 0.0);
    }
    return vec3<f32>(2.0 - q * 2.0, 1.0, 0.0);
}

fn byte(v: u32, shift: u32) -> f32 {
    return f32((v >> shift) & 0xffu) / 255.0;
}

@vertex
fn vs_main(
    @builtin(vertex_index) corner_index: u32,
    @location(0) pos: vec3<f32>,
    @location(1) rgbi: u32,
    @location(2) extra: u32,
) -> Out {
    var out: Out;
    let corner = vec2<f32>(f32(corner_index & 1u), f32(corner_index >> 1u)) * 2.0 - 1.0;
    out.corner = corner;
    let clip = frame.view_proj * vec4<f32>(pos, 1.0);
    if clip.w <= 0.0 {
        // Behind the eye: outside the clip volume.
        out.clip = vec4<f32>(0.0, 0.0, -1.0, 1.0);
    } else {
        // Corners ±0.5 point across, in clip space: point_px / viewport_px · 2 · w.
        let offset = corner * 0.5 * frame.point_px / frame.viewport_px * 2.0 * clip.w;
        out.clip = vec4<f32>(clip.xy + offset, clip.zw);
    }
    switch frame.color_mode {
        case 1u: {
            out.color = vec3<f32>(byte(rgbi, 24u));
        }
        case 2u: {
            out.color = quality_color(byte(extra, 0u));
        }
        default: {
            out.color = vec3<f32>(byte(rgbi, 0u), byte(rgbi, 8u), byte(rgbi, 16u));
        }
    }
    return out;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    if frame.round != 0u && dot(in.corner, in.corner) > 1.0 {
        discard;
    }
    return vec4<f32>(in.color, 1.0);
}
