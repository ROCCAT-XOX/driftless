//! Synthetic scenes, trajectories and sensors for the odometry tests.
#![allow(dead_code)] // each test file uses a different subset

use driftless_io::msg::{ImuSample, LidarFrame, LidarPoint};
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

pub const G: f64 = 9.81;

/// Axis-aligned box.
#[derive(Clone, Copy)]
pub struct Aabb {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

pub fn aabb(min: [f64; 3], max: [f64; 3]) -> Aabb {
    Aabb {
        min: Vector3::from(min),
        max: Vector3::from(max),
    }
}

/// A room seen from inside plus solid boxes inside it.
pub struct Scene {
    pub room: Aabb,
    pub boxes: Vec<Aabb>,
}

impl Scene {
    /// Distance along the unit ray `d` from `o` (inside the room) to the first surface.
    pub fn raycast(&self, o: &Vector3<f64>, d: &Vector3<f64>) -> f64 {
        let mut best = f64::INFINITY;
        for i in 0..3 {
            if d[i].abs() > 1e-12 {
                let bound = if d[i] > 0.0 {
                    self.room.max[i]
                } else {
                    self.room.min[i]
                };
                best = best.min((bound - o[i]) / d[i]);
            }
        }
        for b in &self.boxes {
            let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
            let mut hit = true;
            for i in 0..3 {
                if d[i].abs() < 1e-12 {
                    if o[i] < b.min[i] || o[i] > b.max[i] {
                        hit = false;
                        break;
                    }
                } else {
                    let ta = (b.min[i] - o[i]) / d[i];
                    let tb = (b.max[i] - o[i]) / d[i];
                    t0 = t0.max(ta.min(tb));
                    t1 = t1.min(ta.max(tb));
                }
            }
            if hit && t0 <= t1 && t0 > 1e-6 {
                best = best.min(t0);
            }
        }
        best
    }

    /// Regular samples on all visible surfaces (room walls and box faces), spacing `step`.
    pub fn surface_samples(&self, step: f64, offset: f64) -> Vec<Vector3<f64>> {
        let mut out = Vec::new();
        let mut faces = |b: &Aabb| {
            for axis in 0..3 {
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                for side in [b.min[axis], b.max[axis]] {
                    let mut a = b.min[u] + offset;
                    while a <= b.max[u] {
                        let mut c = b.min[v] + offset;
                        while c <= b.max[v] {
                            let mut p = Vector3::zeros();
                            p[axis] = side;
                            p[u] = a;
                            p[v] = c;
                            out.push(p);
                            c += step;
                        }
                        a += step;
                    }
                }
            }
        };
        faces(&self.room);
        for b in &self.boxes {
            faces(b);
        }
        out
    }
}

/// Rectangular room 12 x 9 x 3 m with two boxes, origin 1.2 m above the floor.
pub fn room_scene() -> Scene {
    Scene {
        room: aabb([-5.0, -4.0, -1.2], [7.0, 5.0, 1.8]),
        boxes: vec![
            aabb([2.0, 1.5, -1.2], [3.0, 2.5, 0.3]),
            aabb([-3.5, -3.0, -1.2], [-2.5, -1.0, 1.8]),
        ],
    }
}

/// Corridor 33 m long, 3.2 m wide, 3 m high, with pillars on alternating walls every 3 m.
pub fn corridor_scene() -> Scene {
    let mut boxes = Vec::new();
    let mut x = 0.5;
    let mut left = true;
    while x < 29.0 {
        let (y0, y1) = if left { (1.2, 1.6) } else { (-1.6, -1.2) };
        boxes.push(aabb([x, y0, -1.3], [x + 0.4, y1, 1.7]));
        x += 1.5;
        left = !left;
    }
    boxes.push(aabb([10.0, -0.9, -1.3], [10.6, -0.5, -0.5])); // crate on the floor
    Scene {
        room: aabb([-3.0, -1.6, -1.3], [30.0, 1.6, 1.7]),
        boxes,
    }
}

/// Smooth 0 → 1 ramp over `[0, d]`.
fn smooth(t: f64, d: f64) -> f64 {
    let s = (t / d).clamp(0.0, 1.0);
    s * s * (3.0 - 2.0 * s)
}

/// Body pose (world from body) as a function of time.
pub trait Trajectory {
    fn pose(&self, t: f64) -> Isometry3<f64>;
}

/// Still for `still` seconds, then accelerates to `speed` along +x within 2 s and keeps it,
/// with gentle yaw/roll/pitch sway, lateral sway and walking bob.
pub struct Walk {
    pub still: f64,
    pub speed: f64,
}

impl Walk {
    fn x(&self, t: f64) -> f64 {
        // integral of speed * smooth(tau, 2) for tau in [0, t]
        let tau = (t - self.still).max(0.0);
        let ramp = 2.0;
        if tau < ramp {
            let s = tau / ramp;
            self.speed * ramp * (s * s * s - 0.5 * s * s * s * s)
        } else {
            self.speed * (0.5 * ramp + (tau - ramp))
        }
    }
}

impl Trajectory for Walk {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        let tau = (t - self.still).max(0.0);
        let env = smooth(tau, 2.0);
        let p = Vector3::new(
            self.x(t),
            env * 0.3 * (0.4 * tau).sin(),
            env * 0.03 * (2.0 * std::f64::consts::PI * 1.8 * tau).sin(),
        );
        let yaw = env * 0.25 * (0.7 * tau).sin();
        let roll = env * 0.05 * (1.3 * tau).sin();
        let pitch = env * 0.04 * (1.1 * tau).cos() - env * 0.04;
        Isometry3::from_parts(
            Translation3::from(p),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        )
    }
}

/// Constant angular velocity (rad/s, body frame) about the origin, no translation.
pub struct Spin {
    pub omega: Vector3<f64>,
    pub t0: f64,
}

impl Trajectory for Spin {
    fn pose(&self, t: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_scaled_axis(self.omega * (t - self.t0)),
        )
    }
}

/// Ideal IMU at `rate` Hz on `[t0, t1]` (numerical derivatives of the trajectory).
pub fn imu_samples(traj: &dyn Trajectory, t0: f64, t1: f64, rate: f64) -> Vec<ImuSample> {
    let h = 1e-4;
    let n = ((t1 - t0) * rate).round() as usize;
    (0..=n)
        .map(|i| {
            let t = t0 + i as f64 / rate;
            let (a, b, c) = (traj.pose(t - h), traj.pose(t), traj.pose(t + h));
            let acc_w = (c.translation.vector - 2.0 * b.translation.vector + a.translation.vector)
                / (h * h);
            let omega_b = (a.rotation.inverse() * c.rotation).scaled_axis() / (2.0 * h);
            let f_b = b.rotation.inverse() * (acc_w + Vector3::new(0.0, 0.0, G));
            ImuSample {
                stamp: t,
                gyro: omega_b.into(),
                accel: f_b.into(),
            }
        })
        .collect()
}

/// 16-ring spinning LiDAR (±15°), `az_steps` per revolution, 0.1 s sweep starting at `stamp`.
/// Points are in the sensor frame at their own capture time; `lidar_in_body` is the mount.
pub fn lidar_frame(
    scene: &Scene,
    traj: &dyn Trajectory,
    stamp: f64,
    az_steps: usize,
    lidar_in_body: &Isometry3<f64>,
) -> LidarFrame {
    let mut points = Vec::with_capacity(16 * az_steps);
    for j in 0..az_steps {
        let t_rel = 0.1 * j as f64 / az_steps as f64;
        let sensor = traj.pose(stamp + t_rel) * lidar_in_body;
        let az = 2.0 * std::f64::consts::PI * j as f64 / az_steps as f64;
        for ring in 0..16u16 {
            let el = (-15.0 + 2.0 * f64::from(ring)).to_radians();
            let d_s = Vector3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            let d_w = sensor.rotation * d_s;
            let r = scene.raycast(&sensor.translation.vector, &d_w);
            if r.is_finite() && r < 100.0 {
                let p = d_s * r;
                points.push(LidarPoint {
                    xyz: [p.x as f32, p.y as f32, p.z as f32],
                    intensity: 10.0,
                    ring,
                    t_rel,
                });
            }
        }
    }
    LidarFrame {
        stamp,
        points,
        dropped: 0,
    }
}

/// Frames at 10 Hz with stamps `t0, t0 + 0.1, …` while `stamp < t1`.
pub fn lidar_frames(
    scene: &Scene,
    traj: &dyn Trajectory,
    t0: f64,
    t1: f64,
    az_steps: usize,
    lidar_in_body: &Isometry3<f64>,
) -> Vec<LidarFrame> {
    let n = ((t1 - t0) * 10.0).floor() as usize;
    (0..n)
        .map(|i| lidar_frame(scene, traj, t0 + i as f64 * 0.1, az_steps, lidar_in_body))
        .collect()
}
