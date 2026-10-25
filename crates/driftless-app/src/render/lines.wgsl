// Line segments as screen-space quads: one instance per segment, four corners as a
// triangle strip, `width` points wide (times `point_px`, the pixels per point), with the ends
// extended by half the width so that the segments of a polyline join without gaps.

struct Frame {
    view_proj: mat4x4<f32>,
    viewport_px: vec2<f32>,
    // Pixels per point of the width.
    point_px: f32,
    color_mode: u32,
    round: u32,
    // Used by the points only (see points.wgsl).
    quality_known: u32,
    z_min: f32,
    z_max: f32,
    // Used by the points only (see points.wgsl).
    point_size: u32,
    point_m: f32,
    px_min: f32,
    px_max: f32,
};

@group(0) @binding(0) var<uniform> frame: Frame;

struct Out {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

// Ends in front of the eye: an end behind it is moved along the segment to just in front.
const NEAR_W: f32 = 1e-4;

@vertex
fn vs_main(
    @builtin(vertex_index) corner: u32,
    @location(0) a: vec3<f32>,
    @location(1) b: vec3<f32>,
    @location(2) rgba: u32,
    @location(3) width: f32,
) -> Out {
    var out: Out;
    out.color = unpack4x8unorm(rgba);
    var ca = frame.view_proj * vec4<f32>(a, 1.0);
    var cb = frame.view_proj * vec4<f32>(b, 1.0);
    if ca.w < NEAR_W && cb.w < NEAR_W {
        out.clip = vec4<f32>(0.0, 0.0, -1.0, 1.0);
        return out;
    }
    if ca.w < NEAR_W {
        ca = mix(ca, cb, (NEAR_W - ca.w) / (cb.w - ca.w));
    } else if cb.w < NEAR_W {
        cb = mix(cb, ca, (NEAR_W - cb.w) / (ca.w - cb.w));
    }
    let half = frame.viewport_px * 0.5;
    let sa = ca.xy / ca.w * half;
    let sb = cb.xy / cb.w * half;
    var dir = vec2<f32>(1.0, 0.0);
    if distance(sa, sb) > 1e-6 {
        dir = normalize(sb - sa);
    }
    let normal = vec2<f32>(-dir.y, dir.x);
    let w = width * frame.point_px * 0.5;
    let end = corner & 1u;
    let side = f32(corner >> 1u) * 2.0 - 1.0;
    var c = ca;
    var along = -1.0;
    if end == 1u {
        c = cb;
        along = 1.0;
    }
    let offset_px = normal * side * w + dir * along * w;
    out.clip = vec4<f32>(c.xy + offset_px / half * c.w, c.zw);
    return out;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    return in.color;
}
