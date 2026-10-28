// Points as screen-aligned quads: one instance per point, four corners as a triangle strip.

struct Frame {
    view_proj: mat4x4<f32>,
    viewport_px: vec2<f32>,
    point_px: f32,
    // 0 = RGB, 1 = intensity, 2 = quality, 3 = scan (see `ColorMode`).
    color_mode: u32,
    // 1 = discard outside the circle.
    round: u32,
    // 0 = the cloud has no submap quality: quality mode shows neutral gray.
    quality_known: u32,
    // Height cut: points with z outside z_min..z_max are not drawn.
    z_min: f32,
    z_max: f32,
    // 0 = point_px across, 1 = point_m across in the world (see `PointSize`).
    point_size: u32,
    point_m: f32,
    // Smallest and largest diameter in pixels in world size.
    px_min: f32,
    px_max: f32,
};

@group(0) @binding(0) var<uniform> frame: Frame;

struct Out {
    @builtin(position) clip: vec4<f32>,
    // The corner in −1..1 on both axes.
    @location(0) corner: vec2<f32>,
    @location(1) color: vec3<f32>,
};

// The quality ramp of `driftless_steps::overlay::quality_color`: red at QUALITY_RED and below,
// green at QUALITY_GREEN and above, logarithmic between them (yellow halfway).
const QUALITY_RED: f32 = 0.01;
const QUALITY_GREEN: f32 = 0.25;
fn quality_color(q: f32) -> vec3<f32> {
    var t = 0.0;
    if q > QUALITY_RED {
        t = clamp(log(q / QUALITY_RED) / log(QUALITY_GREEN / QUALITY_RED), 0.0, 1.0);
    }
    if t < 0.5 {
        return vec3<f32>(1.0, t * 2.0, 0.0);
    }
    return vec3<f32>(2.0 - t * 2.0, 1.0, 0.0);
}

// The colour of scan `scan` (bits 16–31 of `extra`): `SCAN_COLORS` of `render/mod.rs`,
// modulo 8.
fn scan_color(scan: u32) -> vec3<f32> {
    var colors = array<vec3<f32>, 8>(
        vec3<f32>(230.0, 159.0, 0.0),
        vec3<f32>(86.0, 180.0, 233.0),
        vec3<f32>(0.0, 158.0, 115.0),
        vec3<f32>(240.0, 228.0, 66.0),
        vec3<f32>(0.0, 114.0, 178.0),
        vec3<f32>(213.0, 94.0, 0.0),
        vec3<f32>(204.0, 121.0, 167.0),
        vec3<f32>(200.0, 200.0, 200.0),
    );
    return colors[scan % 8u] / 255.0;
}

fn byte(v: u32, shift: u32) -> f32 {
    return f32((v >> shift) & 0xffu) / 255.0;
}

fn corner_of(corner_index: u32) -> vec2<f32> {
    return vec2<f32>(f32(corner_index & 1u), f32(corner_index >> 1u)) * 2.0 - 1.0;
}

// The corner of a quad of `size_px` across around the point at `clip`; outside the clip
// volume behind the eye or when `hidden`.
fn quad_at(corner: vec2<f32>, clip: vec4<f32>, size_px: f32, hidden: bool) -> vec4<f32> {
    if clip.w <= 0.0 || hidden {
        return vec4<f32>(0.0, 0.0, -1.0, 1.0);
    }
    // Corners ±0.5 point across, in clip space: size_px / viewport_px · 2 · w.
    let offset = corner * 0.5 * size_px / frame.viewport_px * 2.0 * clip.w;
    return vec4<f32>(clip.xy + offset, clip.zw);
}

fn quad(corner: vec2<f32>, pos: vec3<f32>, size_px: f32, hidden: bool) -> vec4<f32> {
    return quad_at(corner, frame.view_proj * vec4<f32>(pos, 1.0), size_px, hidden);
}

// The diameter in pixels of a point at clip-space depth `w`: fixed, or `point_m` seen from
// `w` metres, within px_min..px_max. The focal length in pixels is the length of the
// second row of view_proj (cot(fov/2) for a rotation without scale) times half the height.
fn size_px(w: f32) -> f32 {
    if frame.point_size == 0u {
        return frame.point_px;
    }
    let row_y = vec3<f32>(frame.view_proj[0][1], frame.view_proj[1][1], frame.view_proj[2][1]);
    let focal_px = length(row_y) * 0.5 * frame.viewport_px.y;
    return clamp(frame.point_m * focal_px / max(w, 1e-6), frame.px_min, frame.px_max);
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
    let clip = frame.view_proj * vec4<f32>(pos, 1.0);
    out.clip = quad_at(corner, clip, size_px(clip.w), cut);
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
        case 3u: {
            out.color = scan_color(extra >> 16u);
        }
        default: {
            out.color = vec3<f32>(byte(rgbi, 0u), byte(rgbi, 8u), byte(rgbi, 16u));
        }
    }
    return out;
}

// Darkening at the very edge of a world-size spot (factor 1 − RIM_DARKEN), so that
// neighbouring spots stay apart; it starts at RIM_START of the radius.
const RIM_START: f32 = 0.8;
const RIM_DARKEN: f32 = 0.12;

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    let r2 = dot(in.corner, in.corner);
    if frame.point_size == 0u {
        if frame.round != 0u && r2 > 1.0 {
            discard;
        }
        return vec4<f32>(in.color, 1.0);
    }
    // World size: always a round spot, opaque (no blending: the depth buffer stays exact).
    if r2 > 1.0 {
        discard;
    }
    let rim = smoothstep(RIM_START * RIM_START, 1.0, r2);
    return vec4<f32>(in.color * (1.0 - RIM_DARKEN * rim), 1.0);
}
