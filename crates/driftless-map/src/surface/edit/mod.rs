//! Touch-up of a computed surface (Nacharbeit): the Stellen painted on it, found as faces.
//!
//! The computed `oberflaeche.glb` stays as it is. Its pages do not share vertices (every
//! chart has its own copies), so the surface is welded first: [`WeldedSurface`] joins the
//! copies by their bits, as the PLY export does, and numbers the faces by their running index
//! over the pages. [`FaceBvh`] finds the face a ray hits first and the faces near a point;
//! [`select_faces`] turns the dabs of a Stelle into the faces they cover; [`smooth_region`]
//! moves the vertices of a Stelle („Glätten“); [`retexture_region`] bakes a Stelle anew from the
//! camera images onto pages of its own („Bilder neu einspielen“). Every step gives a
//! [`SurfacePatch`]:
//! [`apply_patches`] makes the touched-up surface from the computed one and the patches,
//! [`page_edits`] says the view which of its pages changed, and [`write_patches`] /
//! [`read_patches`] keep the patches next to the surface.
//!
//! Everything here runs on the CPU and is deterministic: the same surface, dabs and steps give
//! the same faces and positions, byte for byte, on every machine and with any number of
//! threads (rayon only for ordered maps and work that does not depend on the order, sorts only
//! on unique keys, no hash map order). The app (picking and highlighting while painting) and
//! the touch-up step use the same code, so they see the same faces.

mod bvh;
mod patch;
mod retexture;
mod select;
mod smooth;
mod weld;

pub use bvh::{FaceBvh, RayHit, LEAF_FACES};
pub use patch::{
    apply_patches, page_edits, read_patches, write_patches, PageEdit, PatchPage, SurfacePatch,
    PATCHES_FILE,
};
pub use retexture::{retexture_region, RetextureDone};
pub use select::{dab_faces, dab_normal, select_faces, SelectDab, MAX_HOLE_AREA, MIN_NORMAL_DOT};
pub use smooth::{smooth_region, Smoothing};
pub use weld::WeldedSurface;

use crate::MapError;
use std::path::PathBuf;

/// Version of the touch-up computation. A change that alters what a step gives (the faces a
/// dab selects, how they are smoothed or re-textured) raises it, so cached touch-ups are
/// computed anew.
pub const EDIT_VERSION: u32 = 1;

/// Why a touch-up step left the surface as it was. The texts are for the app: German, plain.
#[derive(Debug, thiserror::Error)]
pub enum EditError {
    /// „Ganz eben“ on a Stelle that does not lie on a plane; nothing moved. `rms_m`: the RMS
    /// distance of its vertices to the plane through them (m).
    #[error(
        "Diese Stelle ist nicht eben genug für „ganz eben“ (Abweichung {} cm).",
        centimetres(.rms_m)
    )]
    NotFlat { rms_m: f64 },
    /// The Stelle covers no face of the surface.
    #[error("Die Stelle liegt nicht auf dieser Oberfläche.")]
    NoFaces,
    /// „Bilder neu einspielen“: no camera image shows the Stelle (or none that does could be
    /// read); it keeps its texture.
    #[error("Kein Bild zeigt diese Stelle; sie bleibt, wie sie ist.")]
    NoImages,
    /// „Bilder neu einspielen“: around the Stelle there is no texture the new images could be
    /// matched to (only unseen, gray faces, or none the images show) and no edge to such a
    /// texture; it keeps its texture.
    #[error(
        "Um diese Stelle gibt es keine Textur, an die sich neue Bilder angleichen lassen; \
         sie bleibt, wie sie ist."
    )]
    NoReference,
    /// The step was cancelled through its [`driftless_io::progress::Progress`].
    #[error("abgebrochen")]
    Cancelled,
    /// The patches could not be stored ([`write_patches`]); `path` is the file or its folder.
    #[error("{} nicht schreibbar: {source}", .path.display())]
    Unwritable {
        path: PathBuf,
        source: std::io::Error,
    },
    /// The stored patches are missing, damaged or of another version ([`read_patches`]):
    /// `why` says which (German).
    #[error("{} nicht lesbar: {why}", .path.display())]
    Unreadable { path: PathBuf, why: String },
    /// Reading, writing or computing failed (German, from [`MapError`]).
    #[error(transparent)]
    Map(MapError),
}

impl From<MapError> for EditError {
    fn from(e: MapError) -> Self {
        match e {
            MapError::Cancelled => EditError::Cancelled,
            e => EditError::Map(e),
        }
    }
}

/// Metres as centimetres with one decimal and a decimal comma: „4,2“.
fn centimetres(m: &f64) -> String {
    format!("{:.1}", 100.0 * m).replace('.', ",")
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_texts() {
        assert_eq!(
            EditError::NotFlat { rms_m: 0.042 }.to_string(),
            "Diese Stelle ist nicht eben genug für „ganz eben“ (Abweichung 4,2 cm)."
        );
        assert_eq!(
            EditError::NotFlat { rms_m: 0.0349 }.to_string(),
            "Diese Stelle ist nicht eben genug für „ganz eben“ (Abweichung 3,5 cm)."
        );
        assert_eq!(
            EditError::NoFaces.to_string(),
            "Die Stelle liegt nicht auf dieser Oberfläche."
        );
        assert_eq!(EditError::Cancelled.to_string(), "abgebrochen");
        assert_eq!(
            EditError::NoImages.to_string(),
            "Kein Bild zeigt diese Stelle; sie bleibt, wie sie ist."
        );
        assert_eq!(
            EditError::NoReference.to_string(),
            "Um diese Stelle gibt es keine Textur, an die sich neue Bilder angleichen lassen; \
             sie bleibt, wie sie ist."
        );
        let path = PathBuf::from("nacharbeit").join(PATCHES_FILE);
        let e = EditError::Unwritable {
            path: path.clone(),
            source: std::io::Error::other("Platte voll"),
        };
        assert_eq!(
            e.to_string(),
            format!("{} nicht schreibbar: Platte voll", path.display())
        );
        let e = EditError::Unreadable {
            path: path.clone(),
            why: "beschädigt (Prüfsumme stimmt nicht)".into(),
        };
        assert_eq!(
            e.to_string(),
            format!(
                "{} nicht lesbar: beschädigt (Prüfsumme stimmt nicht)",
                path.display()
            )
        );
        // A cancelled computation stays a cancel, everything else keeps its text.
        assert!(matches!(
            EditError::from(MapError::Cancelled),
            EditError::Cancelled
        ));
        let e = EditError::from(MapError::Config("Radius".into()));
        assert!(matches!(e, EditError::Map(MapError::Config(_))));
        assert_eq!(e.to_string(), "ungültige Einstellung: Radius");
    }
}
