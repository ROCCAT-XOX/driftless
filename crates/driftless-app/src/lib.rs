//! The desktop program of Driftless: the project model, how a project with its scans, SLAM
//! runs and maps lives on disk ([`project`]), background jobs ([`jobs`]), the 3D view: a
//! loaded cloud ([`cloud`]), its levels of detail ([`lod`]), the camera ([`camera`]), picking
//! ([`pick`]) and the GPU renderer ([`render`]), and the window with its panels ([`ui`]).

pub mod camera;
pub mod cloud;
pub mod jobs;
pub mod lod;
pub mod pick;
pub mod project;
pub mod render;
pub mod synthetic;
pub mod ui;
