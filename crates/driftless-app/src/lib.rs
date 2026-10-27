//! The desktop program of Driftless: the project model, how a project with its scans, SLAM
//! runs and maps lives on disk ([`project`]), background jobs ([`jobs`]), the 3D view: a
//! loaded cloud ([`cloud`], PLY read through [`driftless_export`]), its levels of detail
//! ([`lod`]), the camera ([`camera`]), picking ([`pick`]), the run drawn over it
//! ([`overlay`]), a textured surface ([`surface`]) and the GPU renderer ([`render`]), and the
//! window with its panels ([`ui`]).

pub mod camera;
pub mod cloud;
pub mod jobs;
pub mod lod;
pub mod overlay;
pub mod pick;
pub mod project;
pub mod render;
pub mod surface;
pub mod synthetic;
pub mod ui;
