//! The camera of the 3D view: orbiting a target or flying freely, in a z-up world, with an
//! infinite reverse-Z projection (depth 1 at the near plane, towards 0 at infinity).
//!
//! All pixel values are physical pixels with the origin at the top left of the view; the
//! interface converts egui points with [`crate::pick::to_physical`].
//!
//! Not to be confused with `driftless_io::camera::Camera` (a colour camera of the scanner);
//! import that one under another name where both are needed.

use crate::lod::{Frustum, ViewInfo};
use nalgebra::{Matrix4, Point3, Vector3};

/// Turn per pixel of mouse movement when orbiting or looking around (degrees).
pub const DEG_PER_PX: f32 = 0.3;
/// The pitch stays within ± this (degrees), so that the view never points straight up/down.
pub const PITCH_LIMIT_DEG: f32 = 89.0;
/// One step of the mouse wheel multiplies the distance by this.
pub const ZOOM_FACTOR: f32 = 0.85;
/// The distance to the target never gets smaller than this (m).
pub const MIN_DISTANCE: f32 = 0.1;
/// Flying with Shift is this much faster.
pub const FAST_FACTOR: f32 = 4.0;
/// Pitch after [`Camera::frame_bounds`]: from above at an angle (degrees).
const FRAME_PITCH_DEG: f32 = -35.0;
/// Room around the bounds after [`Camera::frame_bounds`].
const FRAME_MARGIN: f32 = 1.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CameraMode {
    /// Around [`Camera::target`] at [`Camera::distance`].
    Orbit,
    /// From [`Camera::eye`].
    Fly,
}

/// The view direction is given by `yaw` (about z, 0 = +x, counter-clockwise) and `pitch`
/// (above the horizontal). In orbit mode the eye is `target − dir · distance`; in fly mode
/// it is `eye`, and `target` is kept `dir · distance` in front of it, so that switching
/// modes keeps the view.
#[derive(Debug, Clone, PartialEq)]
pub struct Camera {
    pub mode: CameraMode,
    pub target: Vector3<f32>,
    /// Radians.
    pub yaw: f32,
    /// Radians, within ±[`PITCH_LIMIT_DEG`].
    pub pitch: f32,
    /// Metres from the eye to the target.
    pub distance: f32,
    /// The eye in fly mode.
    pub eye: Vector3<f32>,
    /// Vertical field of view (radians).
    pub fov_y: f32,
    /// Distance of the near plane (m).
    pub near: f32,
}

impl Default for Camera {
    fn default() -> Self {
        let mut cam = Camera {
            mode: CameraMode::Orbit,
            target: Vector3::zeros(),
            yaw: 45f32.to_radians(),
            pitch: -30f32.to_radians(),
            distance: 20.0,
            eye: Vector3::zeros(),
            fov_y: 60f32.to_radians(),
            near: 0.05,
        };
        cam.sync();
        cam
    }
}

/// A ray in world coordinates; `dir` has length 1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    pub origin: Vector3<f32>,
    pub dir: Vector3<f32>,
}

/// Which fly keys are held (W/S, A/D, E/Q).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlyKeys {
    pub forward: bool,
    pub back: bool,
    pub left: bool,
    pub right: bool,
    pub up: bool,
    pub down: bool,
}

/// The motion for [`Camera::fly`] from the held keys: `[forward, right, up]` of length 1
/// (or 0 when nothing or only opposite keys are held) and the speed factor (Shift:
/// [`FAST_FACTOR`]).
pub fn fly_motion(keys: FlyKeys, shift: bool) -> ([f32; 3], f32) {
    let axis = |plus: bool, minus: bool| f32::from(u8::from(plus)) - f32::from(u8::from(minus));
    let v = Vector3::new(
        axis(keys.forward, keys.back),
        axis(keys.right, keys.left),
        axis(keys.up, keys.down),
    );
    let n = v.norm();
    let v = if n > 0.0 { v / n } else { v };
    (v.into(), if shift { FAST_FACTOR } else { 1.0 })
}

/// Whether both sides are finite and positive (an egui panel can be 0 × 0, e.g. collapsed).
pub fn valid_viewport(viewport_px: [f32; 2]) -> bool {
    viewport_px.iter().all(|v| v.is_finite() && *v > 0.0)
}

/// The aspect ratio, or 1 when it is not finite and positive.
fn safe_aspect(aspect: f32) -> f32 {
    if aspect.is_finite() && aspect > 0.0 {
        aspect
    } else {
        1.0
    }
}

fn clamp_pitch(pitch: f32) -> f32 {
    let limit = PITCH_LIMIT_DEG.to_radians();
    pitch.clamp(-limit, limit)
}

impl Camera {
    /// Unit view direction.
    pub fn dir(&self) -> Vector3<f32> {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vector3::new(cp * cy, cp * sy, sp)
    }

    /// Unit vectors to the right of and up from the view direction (right is horizontal).
    fn right_up(&self) -> (Vector3<f32>, Vector3<f32>) {
        let f = self.dir();
        let r = f.cross(&Vector3::z()).normalize();
        (r, r.cross(&f))
    }

    pub fn eye_position(&self) -> Vector3<f32> {
        match self.mode {
            CameraMode::Orbit => self.target - self.dir() * self.distance,
            CameraMode::Fly => self.eye,
        }
    }

    /// Makes the field the current mode does not use agree with the one it does.
    fn sync(&mut self) {
        match self.mode {
            CameraMode::Orbit => self.eye = self.target - self.dir() * self.distance,
            CameraMode::Fly => self.target = self.eye + self.dir() * self.distance,
        }
    }

    pub fn view(&self) -> Matrix4<f32> {
        let eye = Point3::from(self.eye_position());
        Matrix4::look_at_rh(&eye, &(eye + self.dir()), &Vector3::z())
    }

    /// Infinite reverse-Z perspective for a depth range 0..1: depth 1 at `near`, → 0 far.
    /// An aspect ratio that is not finite and positive is taken as 1.
    pub fn proj(&self, aspect: f32) -> Matrix4<f32> {
        let aspect = safe_aspect(aspect);
        let f = 1.0 / (self.fov_y / 2.0).tan();
        #[rustfmt::skip]
        let m = Matrix4::new(
            f / aspect, 0.0, 0.0, 0.0,
            0.0, f, 0.0, 0.0,
            0.0, 0.0, 0.0, self.near,
            0.0, 0.0, -1.0, 0.0,
        );
        m
    }

    pub fn view_proj(&self, aspect: f32) -> Matrix4<f32> {
        self.proj(aspect) * self.view()
    }

    /// Left, right, bottom, top and near plane of the view, from the rows of the
    /// view-projection matrix, normalised.
    pub fn frustum(&self, aspect: f32) -> Frustum {
        let m = self.view_proj(aspect);
        let row = |i: usize| m.row(i).transpose();
        let (r0, r1, r2, r3) = (row(0), row(1), row(2), row(3));
        let planes = [r3 + r0, r3 - r0, r3 + r1, r3 - r1, r3 - r2].map(|p| {
            let n = p.xyz().norm();
            [p.x / n, p.y / n, p.z / n, p.w / n]
        });
        Frustum { planes }
    }

    /// For a degenerate viewport (see [`valid_viewport`]) the frustum is that of a square
    /// view and the pixels per radian are 0.
    pub fn view_info(&self, viewport_px: [f32; 2], min_spacing_px: f32) -> ViewInfo {
        let valid = valid_viewport(viewport_px);
        ViewInfo {
            frustum: self.frustum(if valid {
                viewport_px[0] / viewport_px[1]
            } else {
                1.0
            }),
            eye: self.eye_position().into(),
            near: self.near,
            pixels_per_radian: if valid {
                viewport_px[1] / self.fov_y
            } else {
                0.0
            },
            min_spacing_px,
        }
    }

    /// Turns the view by the mouse movement; the pitch is clamped.
    fn turn(&mut self, dx_px: f32, dy_px: f32) {
        let k = DEG_PER_PX.to_radians();
        self.yaw = (self.yaw - dx_px * k).rem_euclid(std::f32::consts::TAU);
        self.pitch = clamp_pitch(self.pitch - dy_px * k);
    }

    /// Around the target (in fly mode: around the eye, like [`Camera::look`]); the scene
    /// follows the mouse.
    pub fn orbit(&mut self, dx_px: f32, dy_px: f32) {
        self.turn(dx_px, dy_px);
        self.sync();
    }

    /// Turns the view about the eye: right/down moves the view right/down.
    pub fn look(&mut self, dx_px: f32, dy_px: f32) {
        let eye = self.eye_position();
        self.turn(dx_px, dy_px);
        self.eye = eye;
        self.target = eye + self.dir() * self.distance;
    }

    /// Moves eye and target sideways so that the point at the target stays under the mouse.
    /// Does nothing for a degenerate viewport (see [`valid_viewport`]).
    pub fn pan(&mut self, dx_px: f32, dy_px: f32, viewport_px: [f32; 2]) {
        if !valid_viewport(viewport_px) {
            return;
        }
        let metres_per_px = 2.0 * self.distance * (self.fov_y / 2.0).tan() / viewport_px[1];
        let (r, u) = self.right_up();
        let delta = (u * dy_px - r * dx_px) * metres_per_px;
        self.target += delta;
        self.eye += delta;
    }

    /// Positive steps come closer. In orbit mode the distance shrinks by [`ZOOM_FACTOR`]
    /// per step down to [`MIN_DISTANCE`]; in fly mode the eye moves forward by as much as
    /// the distance would shrink.
    pub fn zoom(&mut self, steps: f32) {
        let scaled = self.distance * ZOOM_FACTOR.powf(steps);
        match self.mode {
            CameraMode::Orbit => self.distance = scaled.max(MIN_DISTANCE),
            CameraMode::Fly => self.eye += self.dir() * (self.distance - scaled),
        }
        self.sync();
    }

    /// Moves along the view (`forward`), horizontally to the right and along world z, by
    /// `speed · dt` metres per unit; in orbit mode the target moves along.
    pub fn fly(&mut self, forward: f32, right: f32, up: f32, dt: f32, speed: f32) {
        let (r, _) = self.right_up();
        let delta = (self.dir() * forward + r * right + Vector3::z() * up) * (speed * dt);
        self.target += delta;
        self.eye += delta;
    }

    /// Changes the mode; eye and view direction stay as they are.
    pub fn set_mode(&mut self, mode: CameraMode) {
        if mode == self.mode {
            return;
        }
        self.eye = self.eye_position();
        self.mode = mode;
        self.sync();
    }

    /// Looks at the box from above at an angle (keeping the yaw), far enough that all of it
    /// is in an image of this aspect ratio (width / height; not finite and positive: 1).
    /// Bounds that are not finite or inverted (an empty cloud) leave the camera as it is.
    pub fn frame_bounds(&mut self, min: [f32; 3], max: [f32; 3], aspect: f32) {
        let finite = min.iter().chain(&max).all(|v| v.is_finite());
        if !finite || (0..3).any(|k| min[k] > max[k]) {
            return;
        }
        let min = Vector3::from(min);
        let max = Vector3::from(max);
        let radius = ((max - min).norm() / 2.0).max(MIN_DISTANCE);
        // A sphere of this radius fits in the narrower of the two fields of view.
        let half_y = self.fov_y / 2.0;
        let half_x = (half_y.tan() * safe_aspect(aspect)).atan();
        let half = half_y.min(half_x);
        self.target = (min + max) / 2.0;
        self.pitch = FRAME_PITCH_DEG.to_radians();
        self.distance = (radius / half.sin() * FRAME_MARGIN).max(MIN_DISTANCE);
        self.eye = self.target - self.dir() * self.distance;
    }

    /// The ray from the eye through a pixel of the view; for a degenerate viewport (see
    /// [`valid_viewport`]) the view direction.
    pub fn ray(&self, pixel_px: [f32; 2], viewport_px: [f32; 2]) -> Ray {
        if !valid_viewport(viewport_px) {
            return Ray {
                origin: self.eye_position(),
                dir: self.dir(),
            };
        }
        let t = (self.fov_y / 2.0).tan();
        let aspect = viewport_px[0] / viewport_px[1];
        let x = 2.0 * pixel_px[0] / viewport_px[0] - 1.0;
        let y = 1.0 - 2.0 * pixel_px[1] / viewport_px[1];
        let (r, u) = self.right_up();
        let dir = (self.dir() + r * (x * t * aspect) + u * (y * t)).normalize();
        Ray {
            origin: self.eye_position(),
            dir,
        }
    }
}
