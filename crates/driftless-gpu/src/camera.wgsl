// Camera model of `driftless_io::camera::Camera` in f32: Kannala-Brandt fisheye or pinhole.
// Layout matches `driftless_gpu::GpuCamera` (48 bytes).
struct Camera {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    // Kannala-Brandt k1..k4.
    k: vec4<f32>,
    width: u32,
    height: u32,
    max_theta: f32,
    model: u32,
}

const CAMERA_MODEL_FISHEYE_KB: u32 = 0u;
const CAMERA_MODEL_PINHOLE: u32 = 1u;

// Pixel coordinates of the camera-frame point `p` in `.xy`; `.z` is 1 if the point projects
// into the image and 0 if it lies beyond `max_theta`, behind a pinhole camera or outside.
fn camera_project(cam: Camera, p: vec3<f32>) -> vec3<f32> {
    var uv: vec2<f32>;
    if cam.model == CAMERA_MODEL_PINHOLE {
        if p.z <= 0.0 {
            return vec3<f32>(0.0);
        }
        uv = vec2<f32>(cam.fx * p.x / p.z + cam.cx, cam.fy * p.y / p.z + cam.cy);
    } else {
        let r = length(p.xy);
        if r == 0.0 {
            // On the optical axis: theta is 0 in front, pi behind.
            if p.z < 0.0 {
                return vec3<f32>(0.0);
            }
            uv = vec2<f32>(cam.cx, cam.cy);
        } else {
            let theta = atan2(r, p.z);
            if theta > cam.max_theta {
                return vec3<f32>(0.0);
            }
            let t2 = theta * theta;
            let k = cam.k;
            let theta_d = theta * (1.0 + t2 * (k.x + t2 * (k.y + t2 * (k.z + t2 * k.w))));
            let scale = theta_d / r;
            uv = vec2<f32>(cam.fx * scale * p.x + cam.cx, cam.fy * scale * p.y + cam.cy);
        }
    }
    let inside = uv.x >= 0.0 && uv.y >= 0.0 && uv.x < f32(cam.width) && uv.y < f32(cam.height);
    return vec3<f32>(uv, select(0.0, 1.0, inside));
}
