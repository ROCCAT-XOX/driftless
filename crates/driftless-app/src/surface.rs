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
//!
//! Beside the pages the loader welds the surface into one mesh and puts its faces into a tree
//! ([`SurfacePick`]): what the brush hits and which faces a Stelle covers, the same faces the
//! touch-up computes with. It is built from the file as read, in parallel to decoding the pages.
//!
//! [`mesh_edit`] turns what the touch-up changes on a surface (`page_edits` of
//! `driftless-map`) into the touched-up state the view draws beside the computed one
//! ([`crate::render::mesh::MeshRenderer::set_edit`]), made the same way.

pub use crate::render::mesh::MeshVertex;
use crate::render::mesh::{mip_levels, MeshEdit};
use crate::ui::text;
use driftless_export::mesh::{read_mesh, ImageKind, Mesh, MeshPage};
use driftless_export::ExportError;
use driftless_io::progress::Progress;
use driftless_map::surface::edit::{FaceBvh, PageEdit, WeldedSurface};
use driftless_steps::StepError;
use image::RgbaImage;
use rayon::prelude::*;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// Phase name while the pages of a surface are decoded.
pub const PHASE_LOAD_SURFACE: &str = "Oberfläche laden";
/// The glb in a surface folder.
pub const SURFACE_GLB: &str = "oberflaeche.glb";
/// Texture coordinate of the corners of a page without any (a plain colour).
const NO_UV: [f32; 2] = [0.5, 0.5];
/// Most pixels the page images of one surface decode to together: 64 per byte of image data
/// in the file (photos as JPEG hold far fewer), at least [`PIXELS_FLOOR`] (2^29, 32 pages of
/// 4096²). Decoded they take 4 bytes each, a third more with the mipmaps.
const PIXELS_PER_IMAGE_BYTE: u64 = 64;
const PIXELS_FLOOR: u64 = 1 << 29;

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
    /// What the brush hits on it; the window keeps it while the surface is shown (the view
    /// drops the pages once they are on the GPU). `None` where it was not built (a surface made
    /// by hand).
    pub pick: Option<Arc<SurfacePick>>,
}

/// The surface as the brush sees it: its pages welded into one mesh in its own frame
/// (absolute, f64, as read from the file) and its faces in a tree, built as the touch-up
/// builds them (`driftless_map::surface::edit`), so a Stelle covers the same faces in the view
/// as in the touch-up. About 60 bytes per corner of the pages: some 320 MB for a surface of
/// 6.6 million triangles.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfacePick {
    pub surface: WeldedSurface,
    pub bvh: FaceBvh,
}

impl SurfacePick {
    /// Welds the pages of `mesh` and builds the tree of its faces.
    pub fn new(mesh: &Mesh) -> SurfacePick {
        let surface = WeldedSurface::from_mesh(mesh);
        let bvh = FaceBvh::build(&surface);
        SurfacePick { surface, bvh }
    }
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
/// in parallel (phase [`PHASE_LOAD_SURFACE`], one step per page, a cancel check each), and
/// beside them its [`SurfacePick`] is built (one step more). Problems that leave the surface
/// usable (a missing OBJ texture) are warnings on `progress`; errors carry the reader's German
/// message and name the file.
pub fn load_mesh(path: &Path, progress: &Progress) -> Result<LoadedSurface, StepError> {
    let path = surface_glb(path);
    let mesh = read_mesh(&path, progress).map_err(|e| match e {
        ExportError::Io { path, source } if source.kind() == std::io::ErrorKind::NotFound => {
            StepError::Failed(format!("{path}: {}", text::SURFACE_FILE_MISSING))
        }
        e => e.into(),
    })?;
    let origin = origin_of(mesh.pages.iter().flat_map(|p| &p.positions));
    progress.phase(PHASE_LOAD_SURFACE, Some(mesh.pages.len() as u64 + 1));
    let fail = |e: String| StepError::Failed(format!("{}: {e}", path.display()));
    check_pixels(&mesh.pages).map_err(fail)?;
    // the welding is one thread's work: it runs while the pages are decoded
    let (pick, pages) = rayon::join(
        || {
            let pick = SurfacePick::new(&mesh);
            progress.advance(1);
            pick
        },
        || {
            mesh.pages
                .par_iter()
                .enumerate()
                .map(|(p, page)| {
                    if progress.is_cancelled() {
                        return Err(StepError::Cancelled);
                    }
                    let page =
                        loaded_page(page, origin).map_err(|e| fail(text::bad_texture(p, &e)))?;
                    progress.advance(1);
                    Ok(page)
                })
                .collect::<Result<Vec<_>, StepError>>()
        },
    );
    let (pages, mips) = pages?.into_iter().unzip();
    Ok(LoadedSurface {
        origin,
        pages,
        mips,
        pick: Some(Arc::new(pick)),
    })
}

/// Refuses pages whose images claim more pixels together than the images' bytes can
/// plausibly hold (see [`PIXELS_PER_IMAGE_BYTE`]), read from their headers before anything is
/// decoded: a small file must not decode to gigabytes, several pages at once. A header that
/// cannot be read is left to the decoder and its message.
fn check_pixels(pages: &[MeshPage]) -> Result<(), String> {
    let (mut pixels, mut bytes) = (0u64, 0u64);
    for img in pages.iter().filter_map(|p| p.image.as_ref()) {
        bytes += img.bytes.len() as u64;
        let format = match img.kind {
            ImageKind::Jpeg => image::ImageFormat::Jpeg,
            ImageKind::Png => image::ImageFormat::Png,
        };
        let reader = image::ImageReader::with_format(std::io::Cursor::new(&img.bytes[..]), format);
        if let Ok((w, h)) = reader.into_dimensions() {
            pixels = pixels.saturating_add(u64::from(w) * u64::from(h));
        }
    }
    let max = bytes
        .saturating_mul(PIXELS_PER_IMAGE_BYTE)
        .max(PIXELS_FLOOR);
    if pixels > max {
        return Err(text::textures_too_large(pixels, max));
    }
    Ok(())
}

/// One page for the GPU, with its mipmaps: corners relative to `origin`, the image decoded
/// (or 1 × 1 in the page's colour).
#[allow(clippy::type_complexity)]
fn loaded_page(
    page: &MeshPage,
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
    let vertices = vertices(&page.positions, &page.uvs, origin);
    let indices = page.indices.iter().flatten().copied().collect();
    Ok(((vertices, indices, image), mips))
}

/// Corners for the GPU: `positions` relative to `origin` (subtracted in f64, then cast), each
/// with its texture coordinate of `uvs` ([`NO_UV`] without).
fn vertices(positions: &[[f64; 3]], uvs: &[[f32; 2]], origin: [f64; 3]) -> Vec<MeshVertex> {
    positions
        .iter()
        .enumerate()
        .map(|(i, p)| MeshVertex {
            pos: [0, 1, 2].map(|k| (p[k] - origin[k]) as f32),
            uv: uvs.get(i).copied().unwrap_or(NO_UV),
        })
        .collect()
}

/// What the touch-up changes on the surface `mesh` (as read from its file, loaded with
/// `origin`, its [`LoadedSurface::origin`]), for
/// [`crate::render::mesh::MeshRenderer::set_edit`]: `edits` and `added` are what
/// `driftless_map::surface::edit::page_edits` gives for `mesh`. A changed page gets all its
/// corners anew if some moved – made like [`load_mesh`] makes them, with the page's texture
/// coordinates, so a mark made from the moved places lies on them bit for bit – and its
/// triangles without those taken off. The added pages are decoded with their mipmaps, in
/// parallel: call it off the UI thread.
///
/// # Errors
/// The message of [`text::bad_texture`] if the image of an added page cannot be read (counted
/// as in the touched-up surface, after the pages of `mesh`).
///
/// # Panics
/// If an edit names a page `mesh` does not have (edits of another surface).
pub fn mesh_edit(
    origin: [f64; 3],
    mesh: &Mesh,
    edits: Vec<PageEdit>,
    added: Vec<MeshPage>,
) -> Result<MeshEdit, String> {
    let pages = edits
        .into_iter()
        .map(|edit| {
            let page = &mesh.pages[edit.page as usize];
            let vertices = edit
                .positions
                .map(|positions| vertices(&positions, &page.uvs, origin));
            let indices = (!edit.removed.is_empty()).then(|| {
                page.indices
                    .iter()
                    .enumerate()
                    .filter(|&(t, _)| edit.removed.binary_search(&(t as u32)).is_err())
                    .flat_map(|(_, tri)| *tri)
                    .collect()
            });
            (edit.page, vertices, indices)
        })
        .collect();
    let first = mesh.pages.len();
    let (extra, extra_mips) = added
        .par_iter()
        .enumerate()
        .map(|(i, page)| loaded_page(page, origin).map_err(|e| text::bad_texture(first + i, &e)))
        .collect::<Result<Vec<_>, String>>()?
        .into_iter()
        .unzip();
    Ok(MeshEdit {
        pages,
        extra,
        extra_mips,
    })
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
