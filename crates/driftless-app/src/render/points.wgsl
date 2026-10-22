// Points as screen-aligned quads: one instance per point, four corners as a triangle strip.

struct Frame {
    view_proj: mat4x4<f32>,
    viewport_px: vec2<f32>,
    point_px: f32,
    // 0 = RGB, 1 = intensity, 2 = quality (see `ColorMode`).
    color_mode: u32,
    // 1 = discard outside the circle.
    round: u32,
    // 0 = the cloud has no submap quality: quality mode shows neutral gray.
    quality_known: u32,
    // Height cut: points with z outside z_min..z_max are not drawn.
    z_min: f32,
    z_max: f32,
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

fn corner_of(corner_index: u32) -> vec2<f32> {
    return vec2<f32>(f32(corner_index & 1u), f32(corner_index >> 1u)) * 2.0 - 1.0;
}

// The corner of a quad of `size_px` across around `pos`; outside the clip volume behind
// the eye or when `hidden`.
fn quad(corner: vec2<f32>, pos: vec3<f32>, size_px: f32, hidden: bool) -> vec4<f32> {
    let clip = frame.view_proj * vec4<f32>(pos, 1.0);
    if clip.w <= 0.0 || hidden {
        return vec4<f32>(0.0, 0.0, -1.0, 1.0);
    }
    // Corners ±0.5 point across, in clip space: size_px / viewport_px · 2 · w.
    let offset = corner * 0.5 * size_px / frame.viewport_px * 2.0 * clip.w;
    return vec4<f32>(clip.xy + offset, clip.zw);
}

// Markers: always in their colour, never cut; `extra` is the size in percent of
// `point_px` (the marker size), 0 = 100 %.
@vertex
fn vs_marker(
    @builtin(vertex_index) corner_index: u32,
    @location(0) pos: vec3<f32>,
    @location(1) rgbi: u32,
    @location(2) extra: u32,
) -> Out {
    var out: Out;
    let corner = corner_of(corner_index);
    out.corner = corner;
    var scale = 1.0;
    if extra != 0u {
        scale = f32(extra) / 100.0;
    }
    out.clip = quad(corner, pos, frame.point_px * scale, false);
    out.color = vec3<f32>(byte(rgbi, 0u), byte(rgbi, 8u), byte(rgbi, 16u));
    return out;
}

@vertex
fn vs_main(
    @builtin(vertex_index) corner_index: u32,
    @location(0) pos: vec3<f32>,
    @location(1) rgbi: u32,
    @location(2) extra: u32,
) -> Out {
    var out: Out;
    let corner = corner_of(corner_index);
    out.corner = corner;
    // Cut away: outside the clip volume.
    let cut = pos.z < frame.z_min || pos.z > frame.z_max;
    out.clip = quad(corner, pos, frame.point_px, cut);
    switch frame.color_mode {
        case 1u: {
            out.color = vec3<f32>(byte(rgbi, 24u));
        }
        case 2u: {
            if frame.quality_known != 0u {
                out.color = quality_color(byte(extra, 0u));
            } else {
                out.color = vec3<f32>(0.5);
            }
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
