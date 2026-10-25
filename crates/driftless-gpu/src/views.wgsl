// The best views of every point (appended to camera.wgsl), one image at a time. Mirrors the
// per-image part of `driftless_map::color::colorize_with`: weight (`view_weight`), colour
// (`sample_linear`, `View::new`) and `keep_best_apart`.
//
// Each invocation owns one point and the image's dispatch runs after the previous image's,
// so the slots need no atomics: the kept views are exactly those of the CPU rule, whatever
// the scheduling.

struct Views {
    cam: Camera,
    // Rows of cam_from_world, world relative to the cloud's origin (rotation | translation).
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
    // Camera centre relative to the cloud's origin (w unused).
    cam_pos: vec4<f32>,
    // Size of the decoded image (pixels).
    img_w: u32,
    img_h: u32,
    // The image's position in (capture time, camera name) order.
    rank: u32,
    views_kept: u32,
    // Sharpness relative to the scan's median, at most 1.
    sharpness: f32,
    // Up to this angle from the optical axis a view has full weight (rad).
    full_theta: f32,
    // 1: at most one view per camera and stretch of capture time (`stretch` by rank).
    use_stretch: u32,
    _pad0: u32,
}

@group(0) @binding(0) var<uniform> u: Views;
// Per point x, y, z (relative to the origin) and its spacing (m).
@group(0) @binding(1) var<storage, read> points: array<vec4<f32>>;
// Per point the unit normal and 1, or all 0 without normal.
@group(0) @binding(2) var<storage, read> normals: array<vec4<f32>>;
// One bit per point of the batch: seen by the image (from visibility.wgsl).
@group(0) @binding(3) var<storage, read> visible: array<u32>;
// views_kept slots per point, best first: weight bits, !rank (0 = empty), r | g << 16,
// b | theta << 16 (colours and angle packed like `View`). All zero is an empty slot.
@group(0) @binding(4) var<storage, read_write> slots: array<vec4<u32>>;
// The decoded image, RGB8 row by row, four bytes per word (little end first).
@group(0) @binding(5) var<storage, read> image: array<u32>;
// sRGB byte → linear, 256 entries (`linear_table`).
@group(0) @binding(6) var<storage, read> lin: array<f32>;
// Per rank the id of the image's (camera, ⌊t / view_gap⌋) stretch.
@group(0) @binding(7) var<storage, read> stretch: array<u32>;

const PI: f32 = 3.14159265358979323846;
// Largest linear colour a view holds (`RGB_RANGE`).
const RGB_RANGE: f32 = 4.0;

// `v` clamped to 0 to `range` as a 16-bit value over the full range, rounded half up like
// Rust's `f32::round` on non-negative values.
fn pack(v: f32, range: f32) -> u32 {
    return u32(floor(clamp(v, 0.0, range) / range * 65535.0 + 0.5));
}

// Linear value of channel `c` of pixel (x, y).
fn px(x: u32, y: u32, c: u32) -> f32 {
    let b = (y * u.img_w + x) * 3u + c;
    let byte = (image[b / 4u] >> ((b % 4u) * 8u)) & 0xffu;
    return lin[byte];
}

// Bilinear sample at pixel `uv` (pixel centres at integers), in linear RGB.
fn sample_linear(uv: vec2<f32>) -> vec3<f32> {
    let x = clamp(uv.x, 0.0, f32(u.img_w - 1u));
    let y = clamp(uv.y, 0.0, f32(u.img_h - 1u));
    let x0 = u32(floor(x));
    let y0 = u32(floor(y));
    let x1 = min(x0 + 1u, u.img_w - 1u);
    let y1 = min(y0 + 1u, u.img_h - 1u);
    let fx = x - f32(x0);
    let fy = y - f32(y0);
    var out: vec3<f32>;
    for (var c = 0u; c < 3u; c++) {
        out[c] = (px(x0, y0, c) * (1.0 - fx) + px(x1, y0, c) * fx) * (1.0 - fy)
            + (px(x0, y1, c) * (1.0 - fx) + px(x1, y1, c) * fx) * fy;
    }
    return out;
}

fn weight_of(s: vec4<u32>) -> f32 {
    return bitcast<f32>(s.x);
}

fn rank_of(s: vec4<u32>) -> u32 {
    return ~s.y;
}

// Higher weight first; equal weights: the earlier image (`View::better_than`).
fn better(a: vec4<u32>, b: vec4<u32>) -> bool {
    let wa = weight_of(a);
    let wb = weight_of(b);
    return wa > wb || (wa == wb && rank_of(a) < rank_of(b));
}

@compute @workgroup_size(256)
fn views_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= arrayLength(&points) {
        return;
    }
    if ((visible[i / 32u] >> (i % 32u)) & 1u) == 0u {
        return;
    }
    let p = points[i].xyz;
    let w4 = vec4<f32>(p, 1.0);
    let pc = vec3<f32>(dot(u.row0, w4), dot(u.row1, w4), dot(u.row2, w4));
    let range = length(pc);
    let q = camera_project(u.cam, pc);
    if q.z < 0.5 || range == 0.0 {
        return;
    }
    let theta = atan2(length(pc.xy), pc.z);

    // Weight: cos(incidence) / (1 + d²/4) · edge falloff(θ) · sharpness.
    let n = normals[i];
    var cos_inc = 1.0;
    if n.w > 0.5 {
        let to_cam = (u.cam_pos.xyz - p) / range;
        cos_inc = abs(dot(n.xyz, to_cam));
    }
    var falloff = 0.0;
    if theta <= u.full_theta {
        falloff = 1.0;
    } else if u.cam.max_theta > u.full_theta {
        falloff = clamp((u.cam.max_theta - theta) / (u.cam.max_theta - u.full_theta), 0.0, 1.0);
    }
    let weight = cos_inc / (1.0 + range * range / 4.0) * falloff * u.sharpness;
    if !(weight > 0.0) {
        return;
    }
    let rgb = sample_linear(q.xy);
    let v = vec4<u32>(
        bitcast<u32>(weight),
        ~u.rank,
        pack(rgb.x, RGB_RANGE) | (pack(rgb.y, RGB_RANGE) << 16u),
        pack(rgb.z, RGB_RANGE) | (pack(theta / PI, 1.0) << 16u),
    );

    let k = u.views_kept;
    let base = i * k;
    // At most one view per stretch: a better view replaces its stretch's kept view (which
    // leaves the slots, the rest moving up), a worse one is dropped.
    if u.use_stretch == 1u {
        let own = stretch[u.rank];
        for (var j = 0u; j < k; j++) {
            let s = slots[base + j];
            if weight_of(s) > 0.0 && stretch[rank_of(s)] == own {
                if !better(v, s) {
                    return;
                }
                for (var m = j; m + 1u < k; m++) {
                    slots[base + m] = slots[base + m + 1u];
                }
                slots[base + k - 1u] = vec4<u32>(0u);
                break;
            }
        }
    }
    // Insert among the best (`keep_best`).
    for (var j = 0u; j < k; j++) {
        if better(v, slots[base + j]) {
            for (var m = k - 1u; m > j; m--) {
                slots[base + m] = slots[base + m - 1u];
            }
            slots[base + j] = v;
            return;
        }
    }
}
