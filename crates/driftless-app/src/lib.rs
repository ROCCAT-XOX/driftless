//! The desktop program of Driftless. So far the project model: how a project with its
//! scans, SLAM runs and maps lives on disk ([`project`]), background jobs ([`jobs`]), and
//! the CPU side of the 3D view: a loaded cloud ([`cloud`]) and its levels of detail
//! ([`lod`]).

pub mod cloud;
pub mod jobs;
pub mod lod;
pub mod project;
pub mod synthetic;
