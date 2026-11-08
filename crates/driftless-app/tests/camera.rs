//! Camera maths: orbit and fly, reverse-Z projection, rays and the view volume.

use driftless_app::camera::{fly_motion, Camera, CameraMode, CameraState, FlyKeys, FAST_FACTOR};
use driftless_app::pick::project;
use nalgebra::Vector3;

const VIEWPORT: [f32; 2] = [1600.0, 1200.0];
const ASPECT: f32 = 1600.0 / 1200.0;

fn px(cam: &Camera, p: Vector3<f32>) -> [f32; 3] {
    project(&cam.view_proj(ASPECT), p.into(), VIEWPORT).expect("in front of the camera")
}

fn view_dir(cam: &Camera) -> Vector3<f32> {
    // The ray through the centre of the image is the viewing direction.
    cam.ray([VIEWPORT[0] / 2.0, VIEWPORT[1] / 2.0], VIEWPORT)
        .dir
}

fn close(a: Vector3<f32>, b: Vector3<f32>, tol: f32) -> bool {
    (a - b).norm() <= tol
}

#[test]
fn default_values() {
    let cam = Camera::default();
    assert_eq!(cam.mode, CameraMode::Orbit);
    assert!((cam.fov_y - 60f32.to_radians()).abs() < 1e-6);
    assert!((cam.near - 0.05).abs() < 1e-7);
    assert!(cam.pitch < 0.0, "default looks down");
}

#[test]
fn orbit_keeps_target_in_centre() {
    let mut cam = Camera {
        target: Vector3::new(12.0, -3.0, 1.5),
        distance: 25.0,
        ..Camera::default()
    };
    for (dx, dy) in [
        (0.0, 0.0),
        (37.0, 0.0),
        (-500.0, 80.0),
        (13.0, -400.0),
        (900.0, 900.0),
    ] {
        cam.orbit(dx, dy);
        let p = px(&cam, cam.target);
        assert!(
            (p[0] - VIEWPORT[0] / 2.0).abs() <= 0.5 && (p[1] - VIEWPORT[1] / 2.0).abs() <= 0.5,
            "after orbit({dx}, {dy}): {p:?}"
        );
        assert!(((cam.eye_position() - cam.target).norm() - 25.0).abs() < 1e-3);
    }
}

#[test]
fn orbit_turns_by_0_3_degrees_per_pixel() {
    let mut cam = Camera::default();
    let (yaw, pitch) = (cam.yaw, cam.pitch);
    cam.orbit(10.0, 0.0);
    assert!(((cam.yaw - yaw).abs() - 3f32.to_radians()).abs() < 1e-5);
    cam.orbit(0.0, 10.0);
    assert!(((cam.pitch - pitch).abs() - 3f32.to_radians()).abs() < 1e-5);
}

#[test]
fn reverse_z_depth_range() {
    let cam = Camera {
        target: Vector3::new(1.0, 2.0, 3.0),
        ..Camera::default()
    };
    let eye = cam.eye_position();
    let dir = view_dir(&cam);
    let depth = |d: f32| px(&cam, eye + dir * d)[2];
    assert!((depth(cam.near) - 1.0).abs() < 1e-4, "{}", depth(cam.near));
    assert!(
        depth(1000.0) < 1e-4 && depth(1000.0) > 0.0,
        "{}",
        depth(1000.0)
    );
    let mut last = f32::INFINITY;
    for i in 0..200 {
        let d = cam.near * 1.1f32.powi(i);
        let z = depth(d);
        assert!(z < last, "not falling at {d} m: {z} ≥ {last}");
        last = z;
    }
    // Behind the camera there is no projection.
    assert!(project(&cam.view_proj(ASPECT), (eye - dir).into(), VIEWPORT).is_none());
}

#[test]
fn ray_hits_projected_point() {
    let cam = Camera {
        target: Vector3::new(5.0, 5.0, 0.0),
        distance: 10.0,
        ..Camera::default()
    };
    let eye = cam.eye_position();
    // A point 10 m in front, off the centre of the image.
    let dir = view_dir(&cam);
    let side = dir.cross(&Vector3::z()).normalize();
    for (s, u) in [(0.0, 0.0), (3.0, 1.0), (-4.0, -2.0), (1.0, 3.0)] {
        let p = eye + dir * 10.0 + side * s + Vector3::z() * u;
        let q = px(&cam, p);
        let ray = cam.ray([q[0], q[1]], VIEWPORT);
        assert!((ray.origin - eye).norm() < 1e-4);
        assert!((ray.dir.norm() - 1.0).abs() < 1e-5);
        let v = p - ray.origin;
        let off = (v - ray.dir * v.dot(&ray.dir)).norm();
        assert!(off < 0.001, "{off} m off for ({s}, {u})");
    }
}

#[test]
fn pan_keeps_point_under_cursor() {
    for mode in [CameraMode::Orbit, CameraMode::Fly] {
        let mut cam = Camera {
            target: Vector3::new(-2.0, 7.0, 1.0),
            distance: 18.0,
            ..Camera::default()
        };
        cam.set_mode(mode);
        let point = cam.target;
        let before = px(&cam, point);
        let (dx, dy) = (120.0, -75.0);
        cam.pan(dx, dy, VIEWPORT);
        let after = px(&cam, point);
        assert!(
            (after[0] - before[0] - dx).abs() < 0.5 && (after[1] - before[1] - dy).abs() < 0.5,
            "{mode:?}: {before:?} → {after:?}"
        );
    }
}

#[test]
fn zoom_never_passes_target() {
    let mut cam = Camera {
        distance: 10.0,
        ..Camera::default()
    };
    cam.zoom(1.0);
    assert!((cam.distance - 8.5).abs() < 1e-4);
    cam.zoom(-1.0);
    assert!((cam.distance - 10.0).abs() < 1e-4);
    for _ in 0..200 {
        cam.zoom(3.0);
        assert!(cam.distance >= 0.1);
        let to_target = cam.target - cam.eye_position();
        assert!(
            to_target.dot(&view_dir(&cam)) > 0.0,
            "target behind the eye"
        );
    }
    assert!((cam.distance - 0.1).abs() < 1e-6);
}

#[test]
fn mode_switch_keeps_view() {
    let mut cam = Camera {
        target: Vector3::new(3.0, 4.0, 2.0),
        distance: 12.0,
        ..Camera::default()
    };
    cam.orbit(140.0, -30.0);
    let (eye, dir, vp) = (cam.eye_position(), view_dir(&cam), cam.view_proj(ASPECT));
    cam.set_mode(CameraMode::Fly);
    assert_eq!(cam.mode, CameraMode::Fly);
    assert!(close(cam.eye_position(), eye, 1e-4));
    assert!(close(view_dir(&cam), dir, 1e-5));
    cam.look(55.0, 20.0);
    cam.fly(1.0, 0.5, 0.2, 0.5, 3.0);
    let (eye, dir) = (cam.eye_position(), view_dir(&cam));
    cam.set_mode(CameraMode::Orbit);
    assert!(close(cam.eye_position(), eye, 1e-4));
    assert!(close(view_dir(&cam), dir, 1e-5));
    // Switching to the mode it is in changes nothing.
    let before = cam.clone();
    cam.set_mode(CameraMode::Orbit);
    assert_eq!(cam, before);
    assert!(
        (cam.view_proj(ASPECT) - vp).norm() > 1e-3,
        "the test moved the camera"
    );
}

#[test]
fn look_turns_about_the_eye() {
    let mut cam = Camera::default();
    let eye = cam.eye_position();
    cam.look(80.0, -40.0);
    assert!(close(cam.eye_position(), eye, 1e-4));
    // Looking right turns the view to the right of where it was.
    let mut cam = Camera {
        yaw: 0.0,
        pitch: 0.0,
        ..Camera::default()
    };
    cam.look(10.0, 0.0);
    assert!(view_dir(&cam).y < 0.0, "x forward: right is −y");
}

#[test]
fn fly_moves_along_view() {
    let mut cam = Camera::default();
    cam.set_mode(CameraMode::Fly);
    cam.orbit(30.0, 25.0);
    let (eye, dir) = (cam.eye_position(), view_dir(&cam));
    cam.fly(1.0, 0.0, 0.0, 0.5, 4.0);
    assert!(close(cam.eye_position(), eye + dir * 2.0, 1e-4));
    assert!(close(view_dir(&cam), dir, 1e-6));
    // Right is horizontal and to the right of the view; up is world z.
    let eye = cam.eye_position();
    cam.fly(0.0, 1.0, 0.0, 1.0, 1.0);
    let right = cam.eye_position() - eye;
    assert!(right.z.abs() < 1e-6 && (right.norm() - 1.0).abs() < 1e-5);
    assert!(dir.cross(&right).z < 0.0, "right of the view direction");
    let eye = cam.eye_position();
    cam.fly(0.0, 0.0, 1.0, 1.0, 1.0);
    assert!(close(cam.eye_position(), eye + Vector3::z(), 1e-5));
    // In orbit mode the target travels along.
    let mut cam = Camera::default();
    let (eye, target, dir) = (cam.eye_position(), cam.target, view_dir(&cam));
    cam.fly(1.0, 0.0, 0.0, 1.0, 2.0);
    assert!(close(cam.eye_position(), eye + dir * 2.0, 1e-4));
    assert!(close(cam.target, target + dir * 2.0, 1e-4));
}

#[test]
fn fly_keys_map_to_motion() {
    let none = FlyKeys::default();
    assert_eq!(fly_motion(none, false), ([0.0; 3], 1.0));
    let w = FlyKeys {
        forward: true,
        ..none
    };
    assert_eq!(fly_motion(w, false), ([1.0, 0.0, 0.0], 1.0));
    assert_eq!(fly_motion(w, true), ([1.0, 0.0, 0.0], FAST_FACTOR));
    let s_d_q = FlyKeys {
        back: true,
        right: true,
        down: true,
        ..none
    };
    let (m, _) = fly_motion(s_d_q, false);
    let l = 1.0 / 3f32.sqrt();
    assert!((m[0] + l).abs() < 1e-6 && (m[1] - l).abs() < 1e-6 && (m[2] + l).abs() < 1e-6);
    // Opposite keys cancel.
    let a_d_e = FlyKeys {
        left: true,
        right: true,
        up: true,
        ..none
    };
    assert_eq!(fly_motion(a_d_e, false), ([0.0, 0.0, 1.0], 1.0));
}

#[test]
fn frame_bounds_shows_all_corners() {
    // Landscape, portrait and square viewports.
    for viewport in [VIEWPORT, [600.0, 1200.0], [300.0, 1500.0], [900.0, 900.0]] {
        let aspect = viewport[0] / viewport[1];
        for (min, max) in [
            ([0.0, 0.0, 0.0], [50.0, 20.0, 3.0]),
            ([-500.0, 100.0, -2.0], [-480.0, 400.0, 30.0]),
            ([1.0, 1.0, 1.0], [1.0, 1.0, 1.0]),
        ] {
            for mode in [CameraMode::Orbit, CameraMode::Fly] {
                let mut cam = Camera::default();
                cam.set_mode(mode);
                cam.frame_bounds(min, max, aspect);
                assert!(cam.pitch < -10f32.to_radians(), "from above");
                let vp = cam.view_proj(aspect);
                for c in 0..8 {
                    let p = [0, 1, 2].map(|k| if c >> k & 1 == 1 { max[k] } else { min[k] });
                    let q = project(&vp, p, viewport).expect("in front");
                    assert!(
                        (0.0..=viewport[0]).contains(&q[0])
                            && (0.0..=viewport[1]).contains(&q[1])
                            && q[2] > 0.0
                            && q[2] <= 1.0,
                        "{viewport:?} {mode:?} corner {p:?} at {q:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn frame_facing_looks_straight_at_a_place() {
    for viewport in [VIEWPORT, [600.0, 1200.0], [900.0, 900.0]] {
        let aspect = viewport[0] / viewport[1];
        // a wall facing −y, a floor, a ceiling, a slope
        for normal in [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.6, 0.0, 0.8],
        ] {
            for mode in [CameraMode::Orbit, CameraMode::Fly] {
                let mut cam = Camera::default();
                cam.set_mode(mode);
                let centre = [3.0, -2.0, 1.5];
                cam.frame_facing(centre, normal, 0.5, aspect);
                // along the opposite of the normal, as far as the pitch allows
                let n = Vector3::from(normal).normalize();
                assert!(cam.dir().dot(&-n) > 0.9998, "{normal:?} {:?}", cam.dir());
                assert!(close(view_dir(&cam), cam.dir(), 1e-4));
                // the place in the middle of the image, on the side of the normal
                let q = project(&cam.view_proj(aspect), centre, viewport).expect("in front");
                assert!((q[0] - viewport[0] / 2.0).abs() < 0.5, "{q:?}");
                assert!((q[1] - viewport[1] / 2.0).abs() < 0.5, "{q:?}");
                let eye = cam.eye_position() - Vector3::from(centre);
                assert!(eye.dot(&n) > 0.0, "{eye:?}");
                // the sphere of the radius fits: a point on it beside the centre is in the image
                let side = cam.dir().cross(&Vector3::z());
                let side = if side.norm() > 1e-3 {
                    side.normalize()
                } else {
                    Vector3::x()
                };
                for s in [-0.5, 0.5] {
                    let p = Vector3::from(centre) + side * s;
                    let q = project(&cam.view_proj(aspect), p.into(), viewport).unwrap();
                    assert!((0.0..=viewport[0]).contains(&q[0]), "{q:?}");
                }
            }
        }
    }
    // straight down keeps the yaw; no direction keeps the view; no place changes nothing
    let mut cam = Camera::default();
    let yaw = cam.yaw;
    cam.frame_facing([0.0; 3], [0.0, 0.0, 1.0], 1.0, ASPECT);
    assert_eq!(cam.yaw, yaw);
    let (yaw, pitch) = (cam.yaw, cam.pitch);
    cam.frame_facing([1.0; 3], [0.0; 3], 1.0, ASPECT);
    assert_eq!((cam.yaw, cam.pitch), (yaw, pitch));
    assert!(close(cam.target, Vector3::from([1.0; 3]), 1e-6));
    let before = cam.clone();
    cam.frame_facing([f32::NAN, 0.0, 0.0], [0.0, -1.0, 0.0], 1.0, ASPECT);
    assert_eq!(cam, before);
    cam.frame_facing([0.0; 3], [0.0, -1.0, 0.0], f32::NAN, f32::NAN);
    assert!(cam.distance.is_finite() && cam.distance > 0.0);
}

#[test]
fn frame_bounds_ignores_empty_bounds() {
    let mut cam = Camera::default();
    cam.orbit(40.0, 10.0);
    let before = cam.clone();
    // The bounds of an empty cloud, NaN, an inverted box, and a bad aspect ratio.
    for (min, max) in [
        ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]),
        ([f32::NAN, 0.0, 0.0], [1.0; 3]),
        ([0.0; 3], [1.0, -1.0, 1.0]),
    ] {
        cam.frame_bounds(min, max, ASPECT);
        assert_eq!(cam, before, "{min:?} {max:?}");
    }
    // A bad aspect ratio is taken as 1.
    cam.frame_bounds([0.0; 3], [10.0; 3], f32::NAN);
    assert!(cam.distance.is_finite() && cam.target.iter().all(|v| v.is_finite()));
}

#[test]
fn degenerate_viewport_keeps_camera_finite() {
    for viewport in [
        [800.0, 0.0],
        [0.0, 0.0],
        [0.0, 600.0],
        [f32::NAN, 600.0],
        [800.0, f32::INFINITY],
    ] {
        for mode in [CameraMode::Orbit, CameraMode::Fly] {
            let mut cam = Camera::default();
            cam.set_mode(mode);
            let before = cam.clone();
            cam.pan(30.0, -20.0, viewport);
            assert_eq!(cam, before, "pan in {viewport:?}");
            let ray = cam.ray([10.0, 10.0], viewport);
            assert!(ray
                .dir
                .iter()
                .chain(ray.origin.iter())
                .all(|v| v.is_finite()));
            let info = cam.view_info(viewport, 1.0);
            assert!(info.frustum.planes.iter().flatten().all(|v| v.is_finite()));
            assert!(info.pixels_per_radian.is_finite());
            let aspect = viewport[0] / viewport[1];
            assert!(cam.view_proj(aspect).iter().all(|v| v.is_finite()));
            // Afterwards the camera still works.
            cam.pan(30.0, -20.0, VIEWPORT);
            assert!(cam.eye_position().iter().all(|v| v.is_finite()));
        }
    }
}

#[test]
fn frustum_culls_behind_and_beside() {
    let cam = Camera {
        target: Vector3::new(10.0, 0.0, 0.0),
        yaw: 0.0,
        pitch: 0.0,
        distance: 10.0,
        ..Camera::default()
    };
    // Eye at the origin, looking along +x.
    assert!(close(cam.eye_position(), Vector3::zeros(), 1e-5));
    let f = cam.frustum(ASPECT);
    let cube = |c: [f32; 3]| (c.map(|v| v - 0.5), c.map(|v| v + 0.5));
    let inside = |c: [f32; 3]| {
        let (a, b) = cube(c);
        f.intersects_aabb(a, b)
    };
    assert!(inside([10.0, 0.0, 0.0]));
    assert!(inside([100.0, 50.0, 30.0]));
    assert!(!inside([-10.0, 0.0, 0.0]), "behind");
    assert!(!inside([10.0, 40.0, 0.0]), "left");
    assert!(!inside([10.0, -40.0, 0.0]), "right");
    assert!(!inside([10.0, 0.0, 30.0]), "above");
    assert!(!inside([10.0, 0.0, -30.0]), "below");
    // Only the near plane removes a box just in front of the eye.
    assert!(!f.intersects_aabb([0.01, -0.001, -0.001], [0.02, 0.001, 0.001]));
    let info = cam.view_info(VIEWPORT, 1.5);
    assert_eq!(info.frustum, f);
    assert_eq!(info.eye, [0.0, 0.0, 0.0]);
    assert_eq!(info.near, cam.near);
    assert!((info.pixels_per_radian - 1200.0 / 60f32.to_radians()).abs() < 1e-2);
    assert_eq!(info.min_spacing_px, 1.5);
}

#[test]
fn pitch_is_clamped() {
    let limit = 89f32.to_radians() + 1e-6;
    let mut cam = Camera::default();
    cam.orbit(0.0, 100_000.0);
    assert!(cam.pitch.abs() <= limit && cam.pitch < 0.0);
    cam.orbit(0.0, -100_000.0);
    assert!(cam.pitch.abs() <= limit && cam.pitch > 0.0);
    cam.look(0.0, 100_000.0);
    assert!(cam.pitch.abs() <= limit);
    // Even straight from above the view matrix stays finite.
    assert!(cam.view_proj(ASPECT).iter().all(|v| v.is_finite()));
}

#[test]
fn ray_at_the_pole_is_finite() {
    for pitch in [89f32.to_radians(), -89f32.to_radians()] {
        for mode in [CameraMode::Orbit, CameraMode::Fly] {
            let mut cam = Camera {
                pitch,
                ..Camera::default()
            };
            cam.set_mode(mode);
            cam.orbit(0.0, 0.0);
            assert!((cam.pitch - pitch).abs() < 1e-6);
            let vp = cam.view_proj(ASPECT);
            assert!(vp.iter().all(|v| v.is_finite()));
            for pixel in [
                [0.0, 0.0],
                [VIEWPORT[0], VIEWPORT[1]],
                [800.0, 600.0],
                [1600.0, 0.0],
            ] {
                let ray = cam.ray(pixel, VIEWPORT);
                assert!(ray.dir.iter().all(|v| v.is_finite()), "{pixel:?}");
                assert!((ray.dir.norm() - 1.0).abs() < 1e-5);
                // The ray goes back through its pixel.
                let q = project(&vp, (ray.origin + ray.dir * 10.0).into(), VIEWPORT).unwrap();
                assert!((q[0] - pixel[0]).abs() < 0.5 && (q[1] - pixel[1]).abs() < 0.5);
            }
            let p = px(&cam, cam.target);
            assert!((p[0] - 800.0).abs() <= 0.5 && (p[1] - 600.0).abs() <= 0.5);
        }
    }
}

#[test]
fn zoom_in_fly_mode_moves_forward() {
    let mut cam = Camera {
        distance: 10.0,
        ..Camera::default()
    };
    cam.set_mode(CameraMode::Fly);
    let (eye, dir) = (cam.eye_position(), view_dir(&cam));
    cam.zoom(1.0);
    assert!(close(cam.eye_position(), eye + dir * 1.5, 1e-4));
    assert!((cam.distance - 10.0).abs() < 1e-6, "distance unchanged");
    assert!(close(view_dir(&cam), dir, 1e-6));
    assert!(close(cam.target, cam.eye_position() + dir * 10.0, 1e-4));
    cam.zoom(-1.0);
    assert!(close(
        cam.eye_position(),
        eye + dir * (1.5 - 10.0 * (1.0 / 0.85 - 1.0)),
        1e-4
    ));
    // Unlike orbiting, flying is not stopped by a target.
    for _ in 0..50 {
        cam.zoom(3.0);
    }
    assert!((cam.eye_position() - eye).dot(&dir) > 20.0);
}

/// The camera kept in absolute coordinates ([`CameraState`]) comes back where it was, also in
/// the frame of another origin, in both modes; a state that is not finite changes nothing.
#[test]
fn camera_state_is_absolute_and_comes_back() {
    let from = [512_000.0, 5_400_000.0, 100.0];
    let to = [512_003.0, 5_399_998.0, 99.0];
    for mode in [CameraMode::Orbit, CameraMode::Fly] {
        let mut cam = Camera {
            target: Vector3::new(2.25, 1.5, 1.25),
            yaw: 100f32.to_radians(),
            pitch: (-12.5f32).to_radians(),
            distance: 3.5,
            fov_y: 50f32.to_radians(),
            ..Camera::default()
        };
        cam.set_mode(mode);
        let state = cam.state(from);
        assert_eq!(state.mode, mode);
        let abs = |v: Vector3<f32>, o: [f64; 3]| [0, 1, 2].map(|k| o[k] + f64::from(v[k]));
        for k in 0..3 {
            assert!((state.target[k] - abs(cam.target, from)[k]).abs() < 1e-6);
            assert!((state.eye[k] - abs(cam.eye_position(), from)[k]).abs() < 1e-6);
        }
        assert!((state.yaw_deg - 100.0).abs() < 1e-4, "{state:?}");
        assert!((state.pitch_deg + 12.5).abs() < 1e-4, "{state:?}");
        assert!((state.distance_m - 3.5).abs() < 1e-6);
        assert!((state.fov_y_deg - 50.0).abs() < 1e-4);
        // into a camera that draws in the frame of `to`
        let mut back = Camera::default();
        back.restore(&state, to);
        assert_eq!(back.mode, mode);
        let again = back.state(to);
        for k in 0..3 {
            assert!(
                (again.target[k] - state.target[k]).abs() < 1e-3,
                "{again:?}"
            );
            assert!((again.eye[k] - state.eye[k]).abs() < 1e-3, "{again:?}");
        }
        assert!((again.yaw_deg - state.yaw_deg).abs() < 0.01);
        assert!((again.pitch_deg - state.pitch_deg).abs() < 0.01);
        assert!((again.distance_m - state.distance_m).abs() < 1e-4);
        assert!((again.fov_y_deg - state.fov_y_deg).abs() < 1e-4);
        assert!(close(view_dir(&back), view_dir(&cam), 1e-5));
        // a state that is not finite (a file edited by hand) leaves the camera as it is
        let before = back.clone();
        let mut bad: CameraState = state.clone();
        bad.target[1] = f64::NAN;
        back.restore(&bad, to);
        assert_eq!(back, before);
        let mut bad = state.clone();
        bad.distance_m = f64::INFINITY;
        back.restore(&bad, to);
        assert_eq!(back, before);
        // a pitch beyond the limit is held at it
        let mut steep = state.clone();
        steep.pitch_deg = -120.0;
        back.restore(&steep, to);
        assert!((back.pitch.to_degrees() + 89.0).abs() < 1e-3);
    }
}
