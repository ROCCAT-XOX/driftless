//! A surface loaded for the 3D view: the `oberflaeche.glb` of `driftless surface`, or any
//! `.glb` or `.obj` ([`driftless_export::mesh::read_mesh`]).
//!
//! The reader gives Z-up positions in metres (glb is turned back from Y up, node transforms
//! applied). They are made relative to an origin in whole metres, chosen like the cloud's
//! (centre of the bounds, rounded), so that `f32` keeps millimetres; the view moves the
//! surface into the frame of a cloud shown with it
//! ([`crate::render::mesh::MeshRenderer::set_view_origin`]), so both line up exactly.
//!
//! Every page of the mesh stays a page: its corners, its triangle indices and its image
//! (JPEG or PNG, decoded to RGBA). A page without an image gets a 1 × 1 texture in its colour.
//! The mipmaps of every page are made here too, in the loader's thread, so that the UI thread
//! only copies them to the GPU.

use crate::render::mesh::mip_levels;
pub use crate::render::mesh::MeshVertex;
use crate::ui::text;
use driftless_export::mesh::{read_mesh, ImageKind, MeshPage};
use driftless_export::ExportError;
use driftless_io::progress::Progress;
use driftless_steps::StepError;
use image::RgbaImage;
use rayon::prelude::*;
use std::path::{Path, PathBuf};

/// Phase name while the pages of a surface are decoded.
pub const PHASE_LOAD_SURFACE: &str = "Oberfläche laden";
/// The glb in a surface folder.
pub const SURFACE_GLB: &str = "oberflaeche.glb";
/// Texture coordinate of the corners of a page without any (a plain colour).
const NO_UV: [f32; 2] = [0.5, 0.5];

/// A surface ready for the GPU.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LoadedSurface {
    /// Whole metres; the positions are relative to it.
    pub origin: [f64; 3],
    /// Per page: corners, triangle indices (three per triangle), texture.
    pub pages: Vec<(Vec<MeshVertex>, Vec<u32>, RgbaImage)>,
    /// Per page the halvings of its texture down to 1 × 1 ([`mip_levels`]); a page without
    /// them gets them made when it is uploaded.
    pub mips: Vec<Vec<RgbaImage>>,
}

impl LoadedSurface {
    /// Number of triangles of all pages.
    pub fn triangles(&self) -> usize {
        self.pages.iter().map(|p| p.1.len() / 3).sum()
    }

    /// Per-axis minimum and maximum of the corners (relative), `None` without any.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut corners = self.pages.iter().flat_map(|p| &p.0).map(|v| v.pos);
        let first = corners.next()?;
        Some(corners.fold((first, first), |(lo, hi), p| {
            (
                [0, 1, 2].map(|k| lo[k].min(p[k])),
                [0, 1, 2].map(|k| hi[k].max(p[k])),
            )
        }))
    }
}

/// The file to load for `path`: a folder means its [`SURFACE_GLB`].
pub fn surface_glb(path: &Path) -> PathBuf {
    if path.is_dir() {
        path.join(SURFACE_GLB)
    } else {
        path.to_path_buf()
    }
}

/// Reads the surface at `path`: a `.glb` or `.obj`, or the [`SURFACE_GLB`] of a surface
/// folder. Reading reports phase [`driftless_export::PHASE_READ`]; then the pages are decoded
/// in parallel (phase [`PHASE_LOAD_SURFACE`], one step per page, a cancel check each). Problems
/// that leave the surface usable (a missing OBJ texture) are warnings on `progress`; errors
/// carry the reader's German message and name the file.
pub fn load_mesh(path: &Path, progress: &Progress) -> Result<LoadedSurface, StepError> {
    let path = surface_glb(path);
    let mesh = read_mesh(&path, progress).map_err(|e| match e {
        ExportError::Io { path, source } if source.kind() == std::io::ErrorKind::NotFound => {
            StepError::Failed(format!("{path}: {}", text::SURFACE_FILE_MISSING))
        }
        e => e.into(),
    })?;
    let origin = origin_of(mesh.pages.iter().flat_map(|p| &p.positions));
    progress.phase(PHASE_LOAD_SURFACE, Some(mesh.pages.len() as u64));
    let fail = |e: String| StepError::Failed(format!("{}: {e}", path.display()));
    let pages = mesh
        .pages
        .into_par_iter()
        .enumerate()
        .map(|(p, page)| {
            if progress.is_cancelled() {
                return Err(StepError::Cancelled);
            }
            let page = loaded_page(page, origin).map_err(|e| fail(text::bad_texture(p, &e)))?;
            progress.advance(1);
            Ok(page)
        })
        .collect::<Result<Vec<_>, StepError>>()?;
    let (pages, mips) = pages.into_iter().unzip();
    Ok(LoadedSurface {
        origin,
        pages,
        mips,
    })
}

/// One page for the GPU, with its mipmaps: corners relative to `origin`, the image decoded
/// (or 1 × 1 in the page's colour).
#[allow(clippy::type_complexity)]
fn loaded_page(
    page: MeshPage,
    origin: [f64; 3],
) -> Result<((Vec<MeshVertex>, Vec<u32>, RgbaImage), Vec<RgbaImage>), String> {
    let image = match &page.image {
        None => {
            let [r, g, b] = page.color;
            RgbaImage::from_pixel(1, 1, image::Rgba([r, g, b, 255]))
        }
        Some(img) => {
            let format = match img.kind {
                ImageKind::Jpeg => image::ImageFormat::Jpeg,
                ImageKind::Png => image::ImageFormat::Png,
            };
            image::load_from_memory_with_format(&img.bytes, format)
                .map_err(|e| e.to_string())?
                .to_rgba8()
        }
    };
    let mips = mip_levels(&image);
    let uv = |i: usize| page.uvs.get(i).copied().unwrap_or(NO_UV);
    let vertices = page
        .positions
        .iter()
        .enumerate()
        .map(|(i, p)| MeshVertex {
            pos: [0, 1, 2].map(|k| (p[k] - origin[k]) as f32),
            uv: uv(i),
        })
        .collect();
    let indices = page.indices.iter().flatten().copied().collect();
    Ok(((vertices, indices, image), mips))
}

/// Centre of the bounds of `points` in whole metres; 0 without points.
fn origin_of<'a>(points: impl Iterator<Item = &'a [f64; 3]>) -> [f64; 3] {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    if lo[0].is_finite() {
        [0, 1, 2].map(|k| ((lo[k] + hi[k]) / 2.0).round())
    } else {
        [0.0; 3]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_is_rounded_centre() {
        let pts = [[10.2, -3.0, 0.4], [12.0, -1.0, 2.6]];
        assert_eq!(origin_of(pts.iter()), [11.0, -2.0, 2.0]);
        assert_eq!(origin_of([].iter()), [0.0; 3]);
    }
}
