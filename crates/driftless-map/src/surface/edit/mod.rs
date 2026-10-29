//! Touch-up of a computed surface (Nacharbeit): the Stellen painted on it, found as faces.
//!
//! The computed `oberflaeche.glb` stays as it is. Its pages do not share vertices (every
//! chart has its own copies), so the surface is welded first: [`WeldedSurface`] joins the
//! copies by their bits, as the PLY export does, and numbers the faces by their running index
//! over the pages. [`FaceBvh`] finds the face a ray hits first and the faces near a point;
//! [`select_faces`] turns the dabs of a Stelle into the faces they cover.
//!
//! Everything here runs on the CPU and is deterministic: the same surface and dabs give the
//! same faces, byte for byte, on every machine and with any number of threads (rayon only for
//! ordered maps and work that does not depend on the order, sorts only on unique keys, no hash
//! map order). The app (picking and highlighting while painting) and the touch-up step use
//! the same code, so they see the same faces.

mod bvh;
mod select;
mod weld;

pub use bvh::{FaceBvh, RayHit, LEAF_FACES};
pub use select::{dab_faces, dab_normal, select_faces, SelectDab, MIN_NORMAL_DOT};
pub use weld::WeldedSurface;

/// Version of the touch-up computation. A change that alters what a step gives (the faces a
/// dab selects, how they are smoothed or re-textured) raises it, so cached touch-ups are
/// computed anew.
pub const EDIT_VERSION: u32 = 1;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `v` scaled to length 1; `None` if it has no direction (zero or not finite).
fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = dot(v, v).sqrt();
    (len > 0.0 && len.is_finite()).then(|| v.map(|c| c / len))
}
