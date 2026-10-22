//! Photo-like surface (baustein 2): a TSDF from the cleaned points, then mesh, texture, glb/obj.
//!
//! This step holds the shared types and the [`Tsdf`], a truncated signed distance field that
//! the cleaned points and the positions they were measured from are fused into.

mod cleanup;
mod extract;
mod mc_tables;
mod simplify;
mod tsdf;
// Wired into the surface step by the texture baking.
#[allow(dead_code)]
mod views;

#[cfg(test)]
pub(crate) mod testutil;

pub use cleanup::{cleanup, CleanupStats};
pub use extract::{extract, Mesh};
pub use simplify::simplify;
pub use tsdf::Tsdf;
pub use views::TextureSource;

use crate::{ColorConfig, MapError};
use driftless_traj::Pose;
use nalgebra::Isometry3;

/// Points with the positions they were measured from.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceInput<'a> {
    /// Points in the world frame (m).
    pub xyz: &'a [[f32; 3]],
    /// Point `i` was measured from `sensors[sensor_of_point[i]]`.
    pub sensor_of_point: &'a [u32],
    /// LiDAR positions in the world frame (m). For a map: one per frame, from
    /// [`sensors_for_frames`]; several scans are these lists appended.
    pub sensors: &'a [[f32; 3]],
}

/// The LiDAR position of every frame: `pose(frame_end) · lidar_to_imu`.
pub fn sensors_for_frames(poses: &[Pose], lidar_to_imu: &Isometry3<f64>) -> Vec<[f32; 3]> {
    let mount = lidar_to_imu.translation.vector;
    poses
        .iter()
        .map(|p| {
            let w = p.p + p.q * mount;
            [w.x as f32, w.y as f32, w.z as f32]
        })
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceConfig {
    /// Edge length of a TSDF voxel (m).
    pub voxel: f64,
    /// The TSDF band reaches this many voxels in front of and behind a surface.
    pub truncation_voxels: f32,
    /// A voxel needs this many measurements to count as observed.
    pub min_weight: u32,
    /// Mesh components with fewer faces are dropped.
    pub min_component_faces: usize,
    /// Holes with at most this many boundary edges are filled.
    pub max_hole_edges: usize,
    /// Largest distance simplification may move the surface (m).
    pub simplify_error: f64,
    /// After simplification, edges longer than this are split (m): a face must fit into one
    /// camera image to get a texture.
    pub max_edge: f64,
    /// Smallest texel size on the surface (m).
    pub min_texel: f64,
    /// Edge of a texture page (pixels).
    pub page_size: u32,
    /// Camera views kept per face.
    pub views_per_face: usize,
    /// Weight of the smoothing step of the texture blend (0 to 1).
    pub smooth_weight: f32,
    /// JPEG quality of the texture pages (1 to 100).
    pub jpeg_quality: u8,
    /// Camera selection and exposure for the texture; images are used at full size.
    pub color: ColorConfig,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        SurfaceConfig {
            voxel: 0.01,
            truncation_voxels: 4.0,
            min_weight: 2,
            min_component_faces: 200,
            max_hole_edges: 64,
            simplify_error: 0.002,
            max_edge: 0.3,
            min_texel: 0.002,
            page_size: 4096,
            views_per_face: 4,
            smooth_weight: 0.5,
            jpeg_quality: 90,
            color: ColorConfig {
                downscale: 1,
                ..ColorConfig::default()
            },
        }
    }
}

impl SurfaceConfig {
    pub fn check(&self) -> Result<(), MapError> {
        let bad = |msg: &str| Err(MapError::Config(msg.into()));
        if !(self.voxel.is_finite() && self.voxel > 0.0) {
            return bad("Voxelgröße der Oberfläche muss größer als 0 sein");
        }
        if !(self.truncation_voxels.is_finite() && self.truncation_voxels >= 1.0) {
            return bad("Bandbreite der Oberfläche muss mindestens 1 Voxel sein");
        }
        if self.min_weight == 0 {
            return bad("Mindestgewicht eines Voxels muss mindestens 1 sein");
        }
        if !(self.simplify_error.is_finite() && self.simplify_error >= 0.0) {
            return bad("Vereinfachungsfehler darf nicht negativ sein");
        }
        if !(self.max_edge.is_finite() && self.max_edge > 0.0) {
            return bad("längste Kante der Oberfläche muss größer als 0 sein");
        }
        if !(self.min_texel.is_finite() && self.min_texel > 0.0) {
            return bad("kleinste Texelgröße muss größer als 0 sein");
        }
        if self.page_size == 0 {
            return bad("Texturseite muss mindestens 1 Pixel breit sein");
        }
        if self.views_per_face == 0 {
            return bad("Ansichten je Fläche müssen mindestens 1 sein");
        }
        if !(0.0..=1.0).contains(&self.smooth_weight) {
            return bad("Glättungsgewicht muss zwischen 0 und 1 liegen");
        }
        if !(1..=100).contains(&self.jpeg_quality) {
            return bad("JPEG-Qualität muss zwischen 1 und 100 liegen");
        }
        self.color.check()
    }
}
