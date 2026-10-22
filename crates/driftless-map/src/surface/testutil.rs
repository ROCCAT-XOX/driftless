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
