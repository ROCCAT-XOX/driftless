//! Synthetic scans shared by the surface tests.

use super::SurfaceInput;

/// An owned [`SurfaceInput`].
pub(crate) struct Scan {
    pub xyz: Vec<[f32; 3]>,
    pub sensor_of_point: Vec<u32>,
    pub sensors: Vec<[f32; 3]>,
}

impl Scan {
    pub fn input(&self) -> SurfaceInput<'_> {
        SurfaceInput {
            xyz: &self.xyz,
            sensor_of_point: &self.sensor_of_point,
            sensors: &self.sensors,
        }
    }
}

/// Rays from `origin` to the plane `x = x0`, hitting it on a regular grid of `step` (m)
/// over `|y − origin.y|, |z − origin.z| ≤ half`.
pub(crate) fn plane_x(origin: [f32; 3], x0: f32, half: f32, step: f32) -> Scan {
    let n = (half / step).round() as i32;
    let mut xyz = Vec::new();
    for iy in -n..=n {
        for iz in -n..=n {
            xyz.push([
                x0,
                origin[1] + iy as f32 * step,
                origin[2] + iz as f32 * step,
            ]);
        }
    }
    Scan {
        sensor_of_point: vec![0; xyz.len()],
        xyz,
        sensors: vec![origin],
    }
}

/// A closed box room `[-hx, hx] × [-hy, hy] × [-hz, hz]` around `origin` (the sensor), every
/// wall sampled on a regular grid of `step` (m).
pub(crate) fn box_room(origin: [f32; 3], half: [f32; 3], step: f32) -> Scan {
    let mut xyz = Vec::new();
    let count = |h: f32| (h / step).round() as i32;
    for axis in 0..3 {
        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
        for sign in [-1.0f32, 1.0] {
            for ia in -count(half[a])..=count(half[a]) {
                for ib in -count(half[b])..=count(half[b]) {
                    let mut p = origin;
                    p[axis] += sign * half[axis];
                    p[a] += ia as f32 * step;
                    p[b] += ib as f32 * step;
                    xyz.push(p);
                }
            }
        }
    }
    Scan {
        sensor_of_point: vec![0; xyz.len()],
        xyz,
        sensors: vec![origin],
    }
}
