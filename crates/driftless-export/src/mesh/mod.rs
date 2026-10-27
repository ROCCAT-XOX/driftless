//! Textured meshes (surfaces): glb and OBJ written and read, and a PLY mesh with one colour
//! per corner.
//!
//! A [`Mesh`] is split into pages as the surface step makes them: each page has its own
//! corners, triangles and either an encoded image (JPEG or PNG) or a plain colour. Positions
//! are metres in the frame of the map or the model, Z up; texture coordinates have `v`
//! pointing down (row 0 of the image at `v = 0`). Each format converts at its border:
//! - glb (glTF 2.0) has Y up: `(x, y, z)` is stored as `(x, z, −y)` and read back as
//!   `(x, −z, y)`; texture coordinates are unchanged (glTF has `v` down too).
//! - OBJ stays Z up (Blender: "Z up"); its texture coordinates have `v` up, so `1 − v`.
//! - The PLY mesh welds corners with bit-identical positions across pages and gives each
//!   one the mean colour of the texture at its texture coordinates.

mod glb;
mod obj;
mod ply;

pub use glb::write_glb;
pub use obj::{obj_files, write_obj};
pub use ply::write_ply_mesh;

use crate::ExportError;
use driftless_io::progress::Progress;
use std::path::Path;

/// `asset.generator` of every glb.
pub const GENERATOR: &str = "Driftless";

/// Encoding of a page image.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImageKind {
    Jpeg,
    Png,
}

impl ImageKind {
    /// The usual extension, without the dot.
    pub fn extension(self) -> &'static str {
        match self {
            ImageKind::Jpeg => "jpg",
            ImageKind::Png => "png",
        }
    }

    /// The media type in glTF.
    pub fn mime_type(self) -> &'static str {
        match self {
            ImageKind::Jpeg => "image/jpeg",
            ImageKind::Png => "image/png",
        }
    }

    /// The kind of encoded image `bytes`, from their signature; `None` for others.
    pub fn sniff(bytes: &[u8]) -> Option<ImageKind> {
        if bytes.starts_with(&[0xFF, 0xD8, 0xFF]) {
            Some(ImageKind::Jpeg)
        } else if bytes.starts_with(b"\x89PNG\r\n\x1a\n") {
            Some(ImageKind::Png)
        } else {
            None
        }
    }

    fn image_format(self) -> image::ImageFormat {
        match self {
            ImageKind::Jpeg => image::ImageFormat::Jpeg,
            ImageKind::Png => image::ImageFormat::Png,
        }
    }
}

/// An encoded page image, kept as it is in the file.
#[derive(Clone, Debug, PartialEq)]
pub struct PageImage {
    pub kind: ImageKind,
    pub bytes: Vec<u8>,
}

/// One page of a [`Mesh`]: its corners, triangles and image or colour.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshPage {
    /// Z up, metres.
    pub positions: Vec<[f64; 3]>,
    /// Texture coordinates, one per position or none; `v` points down (row 0 at `v = 0`).
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<[u32; 3]>,
    pub image: Option<PageImage>,
    /// Without an image: the colour of the page (sRGB).
    pub color: [u8; 3],
}

/// A textured mesh, split by page.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub pages: Vec<MeshPage>,
}

impl Mesh {
    /// Number of triangles of all pages.
    pub fn triangles(&self) -> usize {
        self.pages.iter().map(|p| p.indices.len()).sum()
    }
}

/// The mesh formats Driftless writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshFormat {
    Glb,
    Obj,
    PlyMesh,
}

impl MeshFormat {
    /// The format of a file name's extension (any case); `.ply` is the PLY mesh here.
    pub fn from_path(path: &Path) -> Option<Self> {
        let ext = path.extension()?.to_str()?.to_ascii_lowercase();
        match ext.as_str() {
            "glb" => Some(MeshFormat::Glb),
            "obj" => Some(MeshFormat::Obj),
            "ply" => Some(MeshFormat::PlyMesh),
            _ => None,
        }
    }

    /// The usual extension, without the dot.
    pub fn extension(self) -> &'static str {
        match self {
            MeshFormat::Glb => "glb",
            MeshFormat::Obj => "obj",
            MeshFormat::PlyMesh => "ply",
        }
    }
}

/// Reads a `.glb` or `.obj` (by extension). Problems that leave the mesh usable, such as a
/// missing OBJ texture, are warnings on `progress`.
pub fn read_mesh(path: &Path, progress: &Progress) -> Result<Mesh, ExportError> {
    match MeshFormat::from_path(path) {
        Some(MeshFormat::Glb) => glb::read_glb(path, progress),
        Some(MeshFormat::Obj) => obj::read_obj(path, progress),
        _ => Err(ExportError::format(
            path,
            "unbekanntes Netz-Format (lesbar sind .glb und .obj)",
        )),
    }
}

/// Gray of a page whose material or texture is missing.
pub(crate) const MISSING_GRAY: [u8; 3] = [128; 3];

/// `seite_NN`: the name of page `p` (material, image).
pub(crate) fn page_name(p: usize) -> String {
    format!("seite_{p:02}")
}

/// Fails unless every page is consistent: finite positions, one uv per position or none
/// (none only without an image), triangles within the page's corners.
pub(crate) fn check_mesh(path: &Path, mesh: &Mesh) -> Result<(), ExportError> {
    for (p, page) in mesh.pages.iter().enumerate() {
        let n = page.positions.len();
        let fail = |msg: String| Err(ExportError::format(path, format!("Seite {p}: {msg}")));
        if u32::try_from(n).is_err() {
            return fail(format!("{n} Ecken sind zu viele"));
        }
        if !page.uvs.is_empty() && page.uvs.len() != n {
            return fail(format!(
                "{} Texturkoordinaten für {n} Ecken",
                page.uvs.len()
            ));
        }
        if page.image.is_some() && page.uvs.is_empty() && n > 0 {
            return fail("Bild ohne Texturkoordinaten".into());
        }
        if let Some(i) = page
            .positions
            .iter()
            .position(|v| !v.iter().all(|c| c.is_finite()))
        {
            return fail(format!("Ecke {i} hat keine gültige Lage"));
        }
        if let Some((t, &i)) = page
            .indices
            .iter()
            .enumerate()
            .find_map(|(t, tri)| tri.iter().find(|&&i| i as usize >= n).map(|i| (t, i)))
        {
            return fail(format!("Dreieck {t} verweist auf Ecke {i}, es gibt {n}"));
        }
    }
    Ok(())
}
