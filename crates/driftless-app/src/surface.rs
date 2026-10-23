//! A surface (`oberflaeche.glb` of `driftless surface`) loaded for the 3D view.
//!
//! The glb is glTF: Y up, a Driftless point `(x, y, z)` stored as `(x, z, −y)`. Reading
//! turns it back to Z up. Positions are made relative to an origin in whole metres, chosen
//! like the cloud's (centre of the bounds, rounded), so that `f32` keeps millimetres; the
//! view moves the surface into the frame of a cloud shown with it
//! ([`crate::render::mesh::MeshRenderer::set_view_origin`]), so both line up exactly.
//!
//! Every primitive becomes a page: its corners, its triangle indices and its base colour
//! texture (JPEG, decoded to RGBA). A primitive without a texture gets a 1 × 1 gray page.
//! Node transforms are not applied (the writer stores none).

pub use crate::render::mesh::MeshVertex;
use driftless_io::progress::Progress;
use driftless_steps::StepError;
use image::RgbaImage;
use rayon::prelude::*;
use std::path::{Path, PathBuf};

/// Phase name while a surface is read.
pub const PHASE_LOAD_SURFACE: &str = "Oberfläche laden";
/// The glb in a surface folder.
pub const SURFACE_GLB: &str = "oberflaeche.glb";
/// Colour of a page without texture.
const GRAY: [u8; 4] = [128, 128, 128, 255];

/// A surface ready for the GPU.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LoadedSurface {
    /// Whole metres; the positions are relative to it.
    pub origin: [f64; 3],
    /// Per page: corners, triangle indices (three per triangle), texture.
    pub pages: Vec<(Vec<MeshVertex>, Vec<u32>, RgbaImage)>,
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

/// The glb to load for `path`: a folder means its [`SURFACE_GLB`].
pub fn surface_glb(path: &Path) -> PathBuf {
    if path.is_dir() {
        path.join(SURFACE_GLB)
    } else {
        path.to_path_buf()
    }
}

/// glTF `(x, y, z)` (Y up) → Driftless `(x, −z, y)` (Z up), the inverse of the writer's.
fn from_gltf(v: [f32; 3]) -> [f64; 3] {
    [f64::from(v[0]), -f64::from(v[2]), f64::from(v[1])]
}

/// Reads the glb at `path` (phase [`PHASE_LOAD_SURFACE`], one step per primitive, a cancel
/// check each). See the module docs.
pub fn load_glb(path: &Path, progress: &Progress) -> Result<LoadedSurface, StepError> {
    let fail = |e: String| StepError::Failed(format!("{}: {e}", path.display()));
    let bytes = std::fs::read(path).map_err(|e| fail(e.to_string()))?;
    let gltf = gltf::Gltf::from_slice(&bytes).map_err(|e| fail(e.to_string()))?;
    let blob = gltf.blob.as_deref();
    let buffer = |b: gltf::Buffer<'_>| match b.source() {
        gltf::buffer::Source::Bin => blob,
        gltf::buffer::Source::Uri(_) => None,
    };
    let primitives: Vec<gltf::Primitive<'_>> = gltf.meshes().flat_map(|m| m.primitives()).collect();
    progress.phase(PHASE_LOAD_SURFACE, Some(primitives.len() as u64 + 1));

    // Geometry first (absolute, f64), then the origin from all of it.
    struct Raw<'a> {
        positions: Vec<[f64; 3]>,
        uvs: Vec<[f32; 2]>,
        indices: Vec<u32>,
        image: Option<(&'a [u8], String)>,
    }
    let mut raws = Vec::with_capacity(primitives.len());
    for prim in &primitives {
        if progress.is_cancelled() {
            return Err(StepError::Cancelled);
        }
        if prim.mode() != gltf::mesh::Mode::Triangles {
            return Err(fail(format!(
                "Grundform {:?} statt Dreiecken wird nicht unterstützt",
                prim.mode()
            )));
        }
        let reader = prim.reader(buffer);
        let positions: Vec<[f64; 3]> = reader
            .read_positions()
            .ok_or_else(|| fail("Fläche ohne Positionen".into()))?
            .map(from_gltf)
            .collect();
        let uvs: Vec<[f32; 2]> = match reader.read_tex_coords(0) {
            Some(t) => t.into_f32().collect(),
            None => vec![[0.5; 2]; positions.len()],
        };
        let indices: Vec<u32> = match reader.read_indices() {
            Some(i) => i.into_u32().collect(),
            None => (0..positions.len() as u32).collect(),
        };
        if uvs.len() != positions.len()
            || !indices.len().is_multiple_of(3)
            || indices.iter().any(|&i| i as usize >= positions.len())
        {
            return Err(fail("Fläche mit unstimmigen Ecken oder Dreiecken".into()));
        }
        let texture = prim
            .material()
            .pbr_metallic_roughness()
            .base_color_texture()
            .map(|info| info.texture().source().source());
        let image = match texture {
            None => None,
            Some(gltf::image::Source::View { view, mime_type }) => {
                let data = buffer(view.buffer())
                    .ok_or_else(|| fail("Textur liegt nicht in der Datei".into()))?;
                let range = view.offset()..view.offset() + view.length();
                let bytes = data
                    .get(range)
                    .ok_or_else(|| fail("Textur außerhalb der Daten".into()))?;
                Some((bytes, mime_type.to_string()))
            }
            Some(gltf::image::Source::Uri { .. }) => {
                return Err(fail(
                    "Textur als eigene Datei wird nicht unterstützt".into(),
                ));
            }
        };
        raws.push(Raw {
            positions,
            uvs,
            indices,
            image,
        });
        progress.advance(1);
    }
    let origin = origin_of(raws.iter().flat_map(|r| &r.positions));

    // The pages are decoded in parallel (each JPEG on its own).
    let pages = raws
        .into_par_iter()
        .map(|raw| {
            if progress.is_cancelled() {
                return Err(StepError::Cancelled);
            }
            let image = match raw.image {
                None => RgbaImage::from_pixel(1, 1, image::Rgba(GRAY)),
                Some((bytes, mime)) => {
                    let format = match mime.as_str() {
                        "image/jpeg" => image::ImageFormat::Jpeg,
                        other => {
                            return Err(fail(format!("Texturformat {other} nicht unterstützt")))
                        }
                    };
                    image::load_from_memory_with_format(bytes, format)
                        .map_err(|e| fail(format!("Textur: {e}")))?
                        .to_rgba8()
                }
            };
            let vertices = raw
                .positions
                .iter()
                .zip(&raw.uvs)
                .map(|(p, &uv)| MeshVertex {
                    pos: [0, 1, 2].map(|k| (p[k] - origin[k]) as f32),
                    uv,
                })
                .collect();
            Ok((vertices, raw.indices, image))
        })
        .collect::<Result<Vec<_>, StepError>>()?;
    progress.advance(1);
    Ok(LoadedSurface { origin, pages })
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

    #[test]
    fn axes_back_to_z_up() {
        // the writer stores (x, y, z) as (x, z, −y)
        assert_eq!(from_gltf([1.0, 3.0, -2.0]), [1.0, 2.0, 3.0]);
    }
}
