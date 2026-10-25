//! Compute on the GPU via wgpu (Vulkan, Metal, DX12) with the CPU as reference and fallback.
//!
//! [`GpuContext`] finds a usable adapter ([`GpuPref`] decides whether one is required);
//! [`project_points`] runs the fisheye camera model of [`driftless_io::camera::Camera`] as a
//! WGSL compute kernel. [`CAMERA_WGSL`] holds the camera model for other kernels.
//! [`GpuCloud`] keeps a cloud on the GPU and [`ImagePass`] finds the points an image sees
//! (depth buffer and visibility like `driftless_map::visibility`); [`ImageViews`] keeps the
//! best views of every point over a stream of images (the colouring of `driftless_map`);
//! [`FaceViews`] does the same for the faces of a mesh (the surface of `driftless_map`).

mod device;
mod faces;
mod project;
mod views;
mod visibility;

pub use device::{
    CpuFallback, GpuChoice, GpuContext, GpuError, GpuInfo, GpuPref, GpuRunError, BUDGET_ENV,
    MAX_GPU_BYTES,
};
pub use faces::{
    faces_batch, faces_bytes, local_origin, CarrierRule, FaceData, FaceImage, FaceParams, FaceSlot,
    FaceViews,
};
pub use project::{project_points, GpuCamera, CAMERA_WGSL};
pub use views::{
    guarded, views_batch_points, views_bytes, ImageViews, ViewImage, ViewParams, ViewSlot,
};
pub use visibility::{Batch, GpuCloud, ImagePass, VisParams, Visible, MAX_BATCH_BYTES};
pub use wgpu;
