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
//! touch-up computes with. It is built from the file as read, in parallel to decoding the pages,
//! and keeps the file as read: a touch-up of the surface shown takes all of it instead of
//! reading and welding the glb again ([`TouchupJob::prepared`]).
//!
//! [`mesh_edit`] turns what the touch-up changes on a surface (`page_edits` of
//! `driftless-map`) into the touched-up state the view draws beside the computed one
//! ([`crate::render::mesh::MeshRenderer::set_edit`]), made the same way.
//!
//! The touch-up of a surface of a map or model („Nacharbeit“, `driftless_steps::touchup`): the
//! loader looks at its cache with the surface ([`LoadedSurface::touchup`]) – fitting, the view
//! gets the touched-up state at once ([`SurfaceEdit`]); not fitting (or computed without the raw
//! data that are back), the window computes it as a step ([`run_touchup_job`], which also makes
//! the touched-up state for the view).

pub use crate::render::mesh::MeshVertex;
use crate::render::mesh::{mip_levels, MeshEdit};
use crate::ui::text;
use driftless_export::mesh::{read_mesh, ImageKind, Mesh, MeshPage};
use driftless_export::ExportError;
use driftless_io::progress::Progress;
use driftless_map::content_hash;
use driftless_map::surface::edit::{read_patches, FaceBvh, PageEdit, WeldedSurface, PATCHES_FILE};
use driftless_steps::marks::Marks;
use driftless_steps::surface::{texture_sources, SURFACE_REPORT};
use driftless_steps::touchup::{
    edited_pages, run_touchup, touchup_fits, touchup_without_raw_data, FileStamp, PreparedSurface,
    StepOutcome, TouchupDone, TouchupPrepared, TouchupRequest, TOUCHUP_DIR,
};
use driftless_steps::StepError;
use image::RgbaImage;
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// Phase name while the pages of a surface are decoded.
pub const PHASE_LOAD_SURFACE: &str = "Oberfläche laden";
/// Phase name while the loader makes the touched-up state of a surface for the view …
pub const PHASE_LOAD_TOUCHUP: &str = "Nacharbeit laden";
/// … and while the touch-up step makes it.
pub const PHASE_TOUCHUP_VIEW: &str = "Nacharbeit: Ansicht vorbereiten";
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
    /// Its touch-up, for a surface folder whose map or model has Stellen with steps; `None`
    /// without (or where `stellen.toml` cannot be read: the brush says why).
    pub touchup: Option<LoadedTouchup>,
}

/// The touch-up of a surface as the loader found it ([`LoadedSurface::touchup`]).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LoadedTouchup {
    /// It is to be computed: its cache does not fit the Stellen, or was computed without the
    /// raw data that are back now.
    pub stale: bool,
    /// The touched-up state for the view, if the cache fits.
    pub edit: Option<SurfaceEdit>,
}

/// The touched-up state of a surface for the view, made from the cache of its touch-up.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceEdit {
    /// The key of the patches it was made from ([`patches_key`]): the same key, the same state.
    pub key: u64,
    /// For [`crate::render::mesh::MeshRenderer::set_edit`].
    pub edit: MeshEdit,
    /// Where the touch-up moved welded vertices, sorted by vertex: the marks of „Nachher“ lie
    /// there.
    pub moved: Vec<(u32, [f64; 3])>,
}

impl SurfaceEdit {
    /// Whether it changes nothing on the surface (its steps were all refused here, or skipped).
    pub fn is_empty(&self) -> bool {
        self.edit.pages.is_empty() && self.edit.extra.is_empty() && self.moved.is_empty()
    }
}

/// The surface as the brush sees it: its pages welded into one mesh in its own frame
/// (absolute, f64, as read from the file) and its faces in a tree, built as the touch-up
/// builds them (`driftless_map::surface::edit`), so a Stelle covers the same faces in the view
/// as in the touch-up. About 60 bytes per corner of the pages: some 320 MB for a surface of
/// 6.6 million triangles; with the file as read some 360 MB more.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfacePick {
    pub surface: Arc<WeldedSurface>,
    pub bvh: Arc<FaceBvh>,
    /// The glb as read and how the file was then: a touch-up of the surface takes it with the
    /// welded surface and the tree instead of reading the glb again
    /// ([`SurfacePick::prepared`]). `None` for a surface not read from its file here.
    pub read: Option<(FileStamp, Arc<Mesh>)>,
}

impl SurfacePick {
    /// Welds the pages of `mesh` and builds the tree of its faces.
    pub fn new(mesh: &Mesh) -> SurfacePick {
        let surface = WeldedSurface::from_mesh(mesh);
        let bvh = FaceBvh::build(&surface);
        SurfacePick {
            surface: Arc::new(surface),
            bvh: Arc::new(bvh),
            read: None,
        }
    }

    /// [`SurfacePick::new`] of `mesh`, read from the file whose stamp was `glb` before it was
    /// read, keeping both for a touch-up ([`SurfacePick::prepared`]).
    pub fn from_file(glb: Option<FileStamp>, mesh: Arc<Mesh>) -> SurfacePick {
        SurfacePick {
            read: glb.map(|g| (g, Arc::clone(&mesh))),
            ..SurfacePick::new(&mesh)
        }
    }

    /// The surface as a touch-up takes it instead of reading its glb ([`PreparedSurface`]), if
    /// it was read from a file here.
    pub fn prepared(&self) -> Option<PreparedSurface> {
        let (glb, mesh) = self.read.as_ref()?;
        Some(PreparedSurface::new(
            glb.clone(),
            Arc::clone(mesh),
            Arc::clone(&self.surface),
            Arc::clone(&self.bvh),
        ))
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
    // Before it is read: a file changed meanwhile does not fit the stamp afterwards.
    let stamp = FileStamp::of(&path);
    let mesh = read_mesh(&path, progress).map_err(|e| match e {
        ExportError::Io { path, source } if source.kind() == std::io::ErrorKind::NotFound => {
            StepError::Failed(format!("{path}: {}", text::SURFACE_FILE_MISSING))
        }
        e => e.into(),
    })?;
    let mesh = Arc::new(mesh);
    let origin = origin_of(mesh.pages.iter().flat_map(|p| &p.positions));
    progress.phase(PHASE_LOAD_SURFACE, Some(mesh.pages.len() as u64 + 1));
    let fail = |e: String| StepError::Failed(format!("{}: {e}", path.display()));
    check_pixels(&mesh.pages).map_err(fail)?;
    // the welding is one thread's work: it runs while the pages are decoded
    let (pick, pages) = rayon::join(
        || {
            let pick = SurfacePick::from_file(stamp, Arc::clone(&mesh));
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
    let touchup = loaded_touchup(&path, origin, &mesh, &pick.surface, progress);
    Ok(LoadedSurface {
        origin,
        pages,
        mips,
        pick: Some(Arc::new(pick)),
        touchup,
    })
}

/// The touch-up of the surface at `glb` (read as `mesh`, welded as `s`, loaded with `origin`),
/// if it is the glb of a surface folder whose map or model – the folder above – has Stellen
/// with steps ([`LoadedSurface::touchup`]). A cache that cannot be made into the touched-up
/// state is computed anew.
fn loaded_touchup(
    glb: &Path,
    origin: [f64; 3],
    mesh: &Mesh,
    s: &WeldedSurface,
    progress: &Progress,
) -> Option<LoadedTouchup> {
    let dir = glb.parent()?;
    if glb.file_name()? != SURFACE_GLB || !dir.join(SURFACE_REPORT).is_file() {
        return None;
    }
    let marks = Marks::load(dir.parent()?).ok()?;
    if !marks.marks.iter().any(|m| !m.steps.is_empty()) {
        return None;
    }
    if !touchup_fits(dir, &marks) {
        return Some(LoadedTouchup {
            stale: true,
            edit: None,
        });
    }
    // computed without the camera images, which are here now: computed anew with them
    let raw_back = touchup_without_raw_data(dir) && matches!(texture_sources(dir), Ok(Some(_)));
    progress.phase(PHASE_LOAD_TOUCHUP, None);
    let edit = surface_edit(dir, origin, mesh, s).unwrap_or_else(|e| {
        progress.warn(e);
        None
    });
    Some(LoadedTouchup {
        stale: raw_back || edit.is_none(),
        edit,
    })
}

/// The key of the patches in the cache of the touch-up of the surface folder `dir`: the hash of
/// their file; `None` without one.
pub fn patches_key(dir: &Path) -> Option<u64> {
    let bytes = std::fs::read(dir.join(TOUCHUP_DIR).join(PATCHES_FILE)).ok()?;
    Some(content_hash(&bytes))
}

/// The touched-up state of the surface folder `dir` for the view, from the cache of its
/// touch-up, if it fits the Stellen: `mesh` is its glb as read, `s` welded, `origin` what it was
/// loaded with. Decodes the pages the touch-up added: call it off the UI thread.
fn surface_edit(
    dir: &Path,
    origin: [f64; 3],
    mesh: &Mesh,
    s: &WeldedSurface,
) -> Result<Option<SurfaceEdit>, String> {
    let Some(key) = patches_key(dir) else {
        return Ok(None);
    };
    let Some((edits, added)) = edited_pages(dir, s, mesh).map_err(|e| e.to_string())? else {
        return Ok(None);
    };
    let patches = read_patches(&dir.join(TOUCHUP_DIR)).map_err(|e| e.to_string())?;
    // the later patch wins, as when they are applied in order
    let vertices = s.positions().len();
    let moved: BTreeMap<u32, [f64; 3]> = patches
        .iter()
        .flat_map(|p| &p.moved)
        .filter(|(v, _)| (*v as usize) < vertices)
        .copied()
        .collect();
    Ok(Some(SurfaceEdit {
        key,
        edit: mesh_edit(origin, mesh, edits, added)?,
        moved: moved.into_iter().collect(),
    }))
}

/// The touch-up of a surface as a step ([`run_touchup_job`]).
#[derive(Debug, Clone)]
pub struct TouchupJob {
    /// The surface folder; its map or model is the folder it lies in.
    pub surface: PathBuf,
    /// The step just added (Stelle, seq): what came of it is told
    /// ([`TouchupOutput::step`]).
    pub step: Option<(u32, u64)>,
    /// For the surface shown: the touched-up state for the view is made as well.
    pub view: Option<TouchupView>,
    /// What the window has at hand: the surface shown as its loader read it
    /// ([`SurfacePick::prepared`]) and the indices of the camera images it keeps.
    pub prepared: TouchupPrepared,
}

/// The surface shown, as a touch-up of it sees the view ([`TouchupJob::view`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TouchupView {
    /// The load it was shown with ([`crate::ui::brush::TouchupState::load`]).
    pub load: u64,
    /// The origin it was loaded with.
    pub origin: [f64; 3],
    /// The key of the touched-up state the view holds ([`SurfaceEdit::key`]).
    pub shown: Option<u64>,
}

/// What the view should hold of a touch-up ([`TouchupOutput::view`]).
#[derive(Debug)]
pub enum ViewEdit {
    /// What it holds is current.
    Keep,
    /// This touched-up state, or none.
    Set(Option<SurfaceEdit>),
}

/// What [`run_touchup_job`] did.
#[derive(Debug)]
pub struct TouchupOutput {
    /// The surface folder.
    pub surface: PathBuf,
    pub done: TouchupDone,
    /// The camera images of the surface were found (the raw data are there).
    pub raw: bool,
    /// The surface got its touch-up now: it had no cache before, it has one now (a new surface
    /// taking the Stellen over, the first step).
    pub first: bool,
    /// What came of [`TouchupJob::step`] (the Stelle's newest step if the seq is not found).
    pub step: Option<StepOutcome>,
    /// For the surface shown: the load it is for, and what the view should hold.
    pub view: Option<(u64, ViewEdit)>,
}

/// Computes the touch-up of the surface folder `job.surface`: the steps of the Stellen of its
/// map or model replayed with the camera images if the raw data are there
/// ([`run_touchup`], with its progress and cancel, taking what [`TouchupJob::prepared`] holds);
/// for the surface shown then the touched-up state for the view (from the surface as prepared,
/// else reading its glb anew), unless the view holds it already. A cancel after the cache is
/// written is not taken any more: the step is done.
pub fn run_touchup_job(job: &TouchupJob, progress: &Progress) -> Result<TouchupOutput, StepError> {
    let dir = &job.surface;
    let owner = dir
        .parent()
        .ok_or_else(|| text::not_in_tree(dir))?
        .to_path_buf();
    let sources = match texture_sources(dir) {
        Ok(sources) => sources,
        Err(e) => {
            progress.warn(text::raw_data_unreadable(&text::step_error(&e)));
            None
        }
    };
    let raw = sources.is_some();
    let before = patches_key(dir);
    let req = TouchupRequest {
        surface_dir: dir.clone(),
        owner_dir: owner,
        sources,
        prepared: job.prepared.clone(),
    };
    let done = run_touchup(&req, progress)?;
    let after = patches_key(dir);
    let step = job.step.and_then(|(mark, seq)| {
        let of_mark = || done.outcomes.iter().filter(move |o| o.mark == mark);
        of_mark()
            .find(|o| o.seq == seq)
            .or_else(|| of_mark().next_back())
            .cloned()
    });
    let view = match job.view {
        None => None,
        Some(v) if v.shown == after => Some((v.load, ViewEdit::Keep)),
        Some(v) if after.is_none() => Some((v.load, ViewEdit::Set(None))),
        Some(v) => {
            progress.phase(PHASE_TOUCHUP_VIEW, None);
            // the cache is written: the view gets it, also if cancelled meanwhile
            let glb = dir.join(SURFACE_GLB);
            let edit = match job.prepared.surface.as_ref().filter(|p| p.fits(&glb)) {
                Some(p) => surface_edit(dir, v.origin, &p.mesh, &p.welded),
                None => {
                    let mesh = read_mesh(&glb, &Progress::new())?;
                    let s = WeldedSurface::from_mesh(&mesh);
                    surface_edit(dir, v.origin, &mesh, &s)
                }
            }
            .map_err(StepError::Failed)?;
            Some((v.load, ViewEdit::Set(edit)))
        }
    };
    Ok(TouchupOutput {
        surface: dir.clone(),
        done,
        raw,
        first: before.is_none() && after.is_some(),
        step,
        view,
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
