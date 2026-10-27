// The views of every face of a mesh (appended to camera.wgsl), one image at a time. Mirrors
// the per-image part of `driftless_map::surface::views`: the depth buffer splatted from each
// face's centroid and a barycentric grid of samples (`image_views`), the view of each face
// (centroid and corners visible, score, carrier tag, mean colour, angle) and its insertion
// into the face's slots (`keep_best_view`), plus the exposure samples (`keep_best`).
//
// Each invocation owns one face and the image's views dispatch runs after the previous
// image's, so the slots need no atomics; the depth buffer is a minimum per cell (an integer
// atomic on the bits of the positive range), independent of the order of the writes.

struct Params {
    // The camera of the depth buffer (at 1/downscale) and at full size (score).
    geo: Camera,
    full: Camera,
    // Rows of cam_from_world, world relative to the mesh's origin (rotation | translation).
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
    // Camera centre relative to the origin; w: faces whose centroid is farther are skipped
    // (they lie wholly beyond max_dist).
    cam_pos: vec4<f32>,
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
    // Most sample rows of a face, and the meant gap between samples (cells).
    max_sub: u32,
    gap_cells: f32,
    rim_theta: f32,
    min_texel2: f32,
    smooth_cos_min: f32,
    sharpness: f32,
    rank: u32,
    // Slots per face.
    k: u32,
    // Size of the decoded image (pixels) and the factor from depth-buffer pixels to it.
    img_w: u32,
    img_h: u32,
    to_colour: f32,
    // Every `stride`-th face keeps exposure samples; 0: none.
    stride: u32,
    carrier_nz: f32,
    carrier_below: f32,
    carrier_radius: f32,
    carrier_alt: f32,
    angle_step: f32,
    _pad0: u32,
    _pad1: u32,
}

struct Batch {
    // Index of the batch's first face in the mesh, and its face count.
    start: u32,
    len: u32,
    // Index of its first face in `faces`, and of the first exposure sample in `samples`
    // (the views pass keeps the views of one range of the faces; see `FaceViews::keep`).
    first: u32,
    sample_base: u32,
}

struct Face {
    // Corners relative to the origin (w unused).
    a: vec4<f32>,
    b: vec4<f32>,
    c: vec4<f32>,
    // Centroid relative to the origin; w: longest edge (m).
    centroid: vec4<f32>,
    // Unit normal, and the smoothed normal of the score (w unused).
    normal: vec4<f32>,
    smoothed: vec4<f32>,
}

@group(0) @binding(0) var<uniform> u: Params;
@group(0) @binding(1) var<uniform> batch: Batch;
@group(0) @binding(2) var<storage, read> faces: array<Face>;
// Per cell the bitwise complement of the minimum range's f32 bits; 0 = empty.
@group(0) @binding(3) var<storage, read_write> depth: array<atomic<u32>>;
// k slots per face of the batch (none in the depth pass), three words each: score bits,
// !image (0 = empty), r | g << 8 | b << 16 | tag << 24. All zero is an empty slot.
@group(0) @binding(4) var<storage, read_write> slots: array<u32>;
// k exposure samples per sampled face of the kept range (from sample `batch.sample_base`):
// weight bits, !rank, r | g << 16, b | theta << 16 (as `View`). All zero is empty.
@group(0) @binding(5) var<storage, read_write> samples: array<vec4<u32>>;
// The decoded image, RGB8 row by row, four bytes per word (little end first).
@group(0) @binding(6) var<storage, read> image: array<u32>;
// sRGB byte → linear, 256 entries (`linear_table`).
@group(0) @binding(7) var<storage, read> lin: array<f32>;

const PI: f32 = 3.14159265358979323846;
// Largest linear colour a sample holds (`RGB_RANGE`).
const RGB_RANGE: f32 = 4.0;
const TAG_CARRIER: u32 = 0x80u;

fn cam_from_local(p: vec3<f32>) -> vec3<f32> {
    let w = vec4<f32>(p, 1.0);
    return vec3<f32>(dot(u.row0, w), dot(u.row1, w), dot(u.row2, w));
}

// Depth-buffer cell of pixel `uv` of the geometry camera.
fn cell_of(uv: vec2<f32>) -> vec2<i32> {
    let max_cell = vec2<i32>(i32(u.depth_w) - 1, i32(u.depth_h) - 1);
    return clamp(vec2<i32>((uv + 0.5) / u.scale), vec2<i32>(0), max_cell);
}

// Minimum range of cell (x, y); `.y` 0 where the cell is empty (infinitely far).
fn depth_at(x: i32, y: i32) -> vec2<f32> {
    let stored = atomicLoad(&depth[u32(y) * u.depth_w + u32(x)]);
    if stored == 0u {
        return vec2<f32>(0.0, 0.0);
    }
    return vec2<f32>(bitcast<f32>(~stored), 1.0);
}

// Writes the range of local point `p` into the cells of its footprint (`DepthBuffer::splat`
// after `project_point` with `max_dist`).
fn splat_point(p: vec3<f32>, spacing: f32) {
    let pc = cam_from_local(p);
    let range = length(pc);
    if range > u.max_dist || range == 0.0 {
        return;
    }
    let q = camera_project(u.geo, pc);
    if q.z < 0.5 {
        return;
    }
    let cell = cell_of(q.xy);
    let cells = spacing * u.focal / range;
    var r = 0;
    if cells == cells {
        r = i32(clamp(ceil(0.5 * cells - 0.5), 0.0, f32(u.max_splat)));
    }
    let bits = ~bitcast<u32>(range);
    let y0 = max(cell.y - r, 0);
    let y1 = min(cell.y + r, i32(u.depth_h) - 1);
    let x0 = max(cell.x - r, 0);
    let x1 = min(cell.x + r, i32(u.depth_w) - 1);
    for (var y = y0; y <= y1; y++) {
        for (var x = x0; x <= x1; x++) {
            atomicMax(&depth[u32(y) * u.depth_w + u32(x)], bits);
        }
    }
}

// `DepthBuffer::is_visible` for a point at pixel `uv` and range `range`.
fn visible(uv: vec2<f32>, range: f32) -> bool {
    let cell = cell_of(uv);
    let own = depth_at(cell.x, cell.y);
    if own.y > 0.5 && range > own.x * (1.0 + u.tol_rel) + u.tol_abs {
        return false;
    }
    let y0 = max(cell.y - 1, 0);
    let y1 = min(cell.y + 1, i32(u.depth_h) - 1);
    let x0 = max(cell.x - 1, 0);
    let x1 = min(cell.x + 1, i32(u.depth_w) - 1);
    for (var y = y0; y <= y1; y++) {
        for (var x = x0; x <= x1; x++) {
            let d = depth_at(x, y);
            if d.y > 0.5 && range > d.x * (1.0 + u.sil_rel) + u.tol_abs {
                return false;
            }
        }
    }
    return true;
}

// Whether the face's centroid is too far from the camera for any of its points to count.
fn too_far(f: Face) -> bool {
    return !(distance(f.centroid.xyz, u.cam_pos.xyz) <= u.cam_pos.w);
}

@compute @workgroup_size(64)
fn splat_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= batch.len {
        return;
    }
    let f = faces[batch.first + i];
    if too_far(f) {
        return;
    }
    let a = f.a.xyz;
    let b = f.b.xyz;
    let c = f.c.xyz;
    let cam = u.cam_pos.xyz;
    let longest = f.centroid.w;
    // No point of the face is nearer than this; the rows keep the samples about
    // `gap_cells` apart (at most `max_sub` rows).
    let near = min(min(distance(a, cam), distance(b, cam)), distance(c, cam)) - longest;
    let rows = ceil(longest * u.focal / (max(near, 1e-3) * u.gap_cells));
    var n = 1u;
    if rows >= f32(u.max_sub) {
        n = u.max_sub;
    } else if rows >= 1.0 {
        n = u32(rows);
    }
    let spacing = longest / f32(n);
    splat_point(f.centroid.xyz, spacing);
    for (var si = 0u; si <= n; si++) {
        for (var sj = 0u; sj <= n - si; sj++) {
            let s = f32(si) / f32(n);
            let t = f32(sj) / f32(n);
            splat_point(a + (b - a) * s + (c - a) * t, spacing);
        }
    }
}

// Linear value of channel `ch` of pixel (x, y).
fn px(x: u32, y: u32, ch: u32) -> f32 {
    let at = (y * u.img_w + x) * 3u + ch;
    let byte = (image[at / 4u] >> ((at % 4u) * 8u)) & 0xffu;
    return lin[byte];
}

// Bilinear sample at pixel `uv` of the decoded image (pixel centres at integers), linear RGB.
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
    for (var ch = 0u; ch < 3u; ch++) {
        out[ch] = (px(x0, y0, ch) * (1.0 - fx) + px(x1, y0, ch) * fx) * (1.0 - fy)
            + (px(x0, y1, ch) * (1.0 - fx) + px(x1, y1, ch) * fx) * fy;
    }
    return out;
}

// Depth-buffer pixel → pixel of the decoded image.
fn to_px(p: vec2<f32>) -> vec2<f32> {
    return (p + 0.5) * u.to_colour - 0.5;
}

// Linear value → sRGB level 0–255 (not rounded), `srgb_value`.
fn srgb_level(l: f32) -> f32 {
    let x = clamp(l, 0.0, 1.0);
    var c: f32;
    if x <= 0.0031308 {
        c = 12.92 * x;
    } else {
        c = 1.055 * pow(x, 1.0 / 2.4) - 0.055;
    }
    return c * 255.0;
}

// `v` clamped to 0 to `range` as a 16-bit value over the full range, rounded half up.
fn pack16(v: f32, range: f32) -> u32 {
    return u32(floor(clamp(v, 0.0, range) / range * 65535.0 + 0.5));
}

// `FaceView::better_than`: a view before an empty slot, then the higher score (a carrier
// view's counted at `carrier_alt`), equal: the earlier image.
fn face_better(sa: f32, ia: u32, ta: u32, sb: f32, ib: u32, tb: u32) -> bool {
    if (sa > 0.0) != (sb > 0.0) {
        return sa > 0.0;
    }
    var ra = sa;
    if (ta & TAG_CARRIER) != 0u {
        ra = sa * u.carrier_alt;
    }
    var rb = sb;
    if (tb & TAG_CARRIER) != 0u {
        rb = sb * u.carrier_alt;
    }
    return ra > rb || (ra == rb && ia < ib);
}

// Whether the view (score, image, tag) beats the slot at word `base`.
fn slot_beaten(base: u32, score: f32, img: u32, tag: u32) -> bool {
    let s = bitcast<f32>(slots[base]);
    let im = ~slots[base + 1u];
    let t = slots[base + 2u] >> 24u;
    return face_better(score, img, tag, s, im, t);
}

// `keep_best_view` on the slots of local face `i`.
fn keep_view(i: u32, score: f32, colour: u32, tag: u32) {
    let k = u.k;
    let first = i * k * 3u;
    let img = u.rank;
    if !slot_beaten(first + (k - 1u) * 3u, score, img, tag) {
        return;
    }
    for (var j = 0u; j < k; j++) {
        if slot_beaten(first + j * 3u, score, img, tag) {
            for (var m = k - 1u; m > j; m--) {
                let dst = first + m * 3u;
                let src = dst - 3u;
                slots[dst] = slots[src];
                slots[dst + 1u] = slots[src + 1u];
                slots[dst + 2u] = slots[src + 2u];
            }
            let at = first + j * 3u;
            slots[at] = bitcast<u32>(score);
            slots[at + 1u] = ~img;
            slots[at + 2u] = colour | (tag << 24u);
            return;
        }
    }
}

// lowbias32, as `hash32` in driftless-map.
fn hash32(v: u32) -> u32 {
    var x = v;
    x ^= x >> 16u;
    x *= 0x7feb352du;
    x ^= x >> 15u;
    x *= 0x846ca68bu;
    return x ^ (x >> 16u);
}

// Weight of image `rank`'s view with `score` (> 0) in exposure sample `s` (`sample_key`): the
// score's half octave as the exponent, a hash of sample and image as the mantissa.
fn sample_key(s: u32, rank: u32, score: f32) -> f32 {
    let bits = bitcast<u32>(score);
    let octave = i32((bits >> 23u) & 0xffu) - 127;
    var band = 2 * octave;
    if (bits & 0x7fffffu) >= 0x3504f3u {
        band += 1;
    }
    let exponent = u32(clamp(band, -126, 127) + 127);
    return bitcast<f32>((exponent << 23u) | (hash32(rank ^ hash32(s)) >> 9u));
}

// `keep_best` on exposure sample `s`: higher weight first, equal: the earlier image.
fn keep_sample(s: u32, v: vec4<u32>) {
    let k = u.k;
    let first = s * k;
    let w = bitcast<f32>(v.x);
    for (var j = 0u; j < k; j++) {
        let o = samples[first + j];
        let wo = bitcast<f32>(o.x);
        if w > wo || (w == wo && ~v.y < ~o.y) {
            for (var m = k - 1u; m > j; m--) {
                samples[first + m] = samples[first + m - 1u];
            }
            samples[first + j] = v;
            return;
        }
    }
}

@compute @workgroup_size(64)
fn views_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= batch.len {
        return;
    }
    let f = faces[batch.first + i];
    if too_far(f) {
        return;
    }
    // The centroid: within reach, in the image, inside the rim, facing the camera, visible.
    let centre = f.centroid.xyz;
    let pc = cam_from_local(centre);
    let range = length(pc);
    if range > u.max_dist || range == 0.0 {
        return;
    }
    let q = camera_project(u.geo, pc);
    if q.z < 0.5 {
        return;
    }
    let theta = atan2(length(pc.xy), pc.z);
    if theta >= u.rim_theta {
        return;
    }
    let to_cam = u.cam_pos.xyz - centre;
    let dist = length(to_cam);
    let cos_n = dot(f.normal.xyz, to_cam) / dist;
    if !(cos_n > 0.0) || !visible(q.xy, range) {
        return;
    }
    // The corners: inside the rim, in the full image and the depth buffer's, visible.
    var full: array<vec2<f32>, 3>;
    var at: array<vec2<f32>, 3>;
    for (var m = 0u; m < 3u; m++) {
        var p = f.a.xyz;
        if m == 1u {
            p = f.b.xyz;
        } else if m == 2u {
            p = f.c.xyz;
        }
        let cc = cam_from_local(p);
        if atan2(length(cc.xy), cc.z) >= u.rim_theta {
            return;
        }
        let qf = camera_project(u.full, cc);
        if qf.z < 0.5 {
            return;
        }
        full[m] = qf.xy;
        let rc = length(cc);
        if rc == 0.0 {
            return;
        }
        let qg = camera_project(u.geo, cc);
        if qg.z < 0.5 || !visible(qg.xy, rc) {
            return;
        }
        at[m] = qg.xy;
    }
    // Score: projected area (full-size pixels), capped at the face area over min_texel²,
    // scaled to the smoothed normal, times its cosine and the sharpness.
    let d1 = full[1] - full[0];
    let d2 = full[2] - full[0];
    var area = 0.5 * abs(d1.x * d2.y - d2.x * d1.y);
    let cap = 0.5 * length(cross(f.b.xyz - f.a.xyz, f.c.xyz - f.a.xyz)) / u.min_texel2;
    let cos_s = max(dot(f.smoothed.xyz, to_cam) / dist, u.smooth_cos_min);
    area = area * clamp(cos_s / cos_n, 0.5, 2.0);
    let score = min(area, cap) * cos_s * u.sharpness;
    if !(score > 0.0) {
        return;
    }
    let across = length(to_cam.xy);
    let carrier = f.normal.z > u.carrier_nz && to_cam.z > u.carrier_below
        && across < u.carrier_radius && across < to_cam.z;
    // Mean colour: the centroid and the points halfway to the corners.
    let centre_rgb = sample_linear(to_px(q.xy));
    var mean = centre_rgb;
    for (var m = 0u; m < 3u; m++) {
        mean += sample_linear(to_px((q.xy + at[m]) / 2.0));
    }
    var colour = 0u;
    for (var ch = 0u; ch < 3u; ch++) {
        let level = clamp(floor(srgb_level(mean[ch] / 4.0) + 0.5), 0.0, 255.0);
        colour |= u32(level) << (8u * ch);
    }
    let angle = min(u32(floor(theta / u.angle_step + 0.5)), 0x7fu);
    var tag = angle;
    if carrier {
        tag |= TAG_CARRIER;
    }
    keep_view(i, score, colour, tag);
    // The exposure sample (not of the carrier).
    let face = batch.start + i;
    if u.stride > 0u && face % u.stride == 0u && !carrier {
        let s = face / u.stride;
        let v = vec4<u32>(
            bitcast<u32>(sample_key(s, u.rank, score)),
            ~u.rank,
            pack16(centre_rgb.x, RGB_RANGE) | (pack16(centre_rgb.y, RGB_RANGE) << 16u),
            pack16(centre_rgb.z, RGB_RANGE) | (pack16(theta / PI, 1.0) << 16u),
        );
        keep_sample(s - batch.sample_base, v);
    }
}
