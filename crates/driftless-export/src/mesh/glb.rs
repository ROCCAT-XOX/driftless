//! glb (binary glTF 2.0): writer (moved from `driftless-map/src/surface/write.rs`, bytes
//! unchanged) and reader for own and simple foreign files.
//!
//! Written: metres, Y up — a Driftless point `(x, y, z)` (Z up) becomes `(x, z, −y)`, a
//! rotation, so the winding and the front sides stay. One mesh with one primitive per
//! non-empty page (positions, `TEXCOORD_0` if the page has uvs, `u32` indices), one
//! material per page with `KHR_materials_unlit` (the photos are already lit): with an image
//! `baseColorTexture` with the page embedded as JPEG or PNG, without one
//! `baseColorFactor = colour / 255`. glTF texture coordinates start at the top left with `v`
//! down, as ours, so they are written unchanged. Deterministic: fixed generator string, no
//! time, fixed key order.
//!
//! Read: the nodes of the scene with their world matrices (without a scene every mesh as it
//! is), one page per primitive; only triangles, only data inside the file (no separate
//! `.bin` or image files), images JPEG or PNG.

use super::{check_mesh, ImageKind, Mesh, MeshPage, PageImage, GENERATOR};
use crate::{check_cancel, ExportError, PHASE_READ};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use gltf_json as json;
use json::validation::{Checked, USize64};
use nalgebra::{Matrix4, Vector4};
use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// At most this many primitives are read (a node tree that reuses nodes could otherwise
/// ask for unbounded copies).
const MAX_INSTANCES: usize = 1 << 16;
/// At most this many node visits, for the same reason.
const MAX_VISITS: usize = 1 << 20;
/// Bytes of corners and triangles all primitives of a glb may decode to: this many per byte
/// of the file (instancing a mesh many times costs nothing in the file), at least
/// [`DECODED_FLOOR`] and at most [`DECODED_MAX`].
const DECODED_PER_FILE_BYTE: u64 = 16;
const DECODED_FLOOR: u64 = 256 << 20;
const DECODED_MAX: u64 = 8 << 30;
/// Decoded bytes per corner (f64 position, f32 uv) and per index (u32, then in a triangle).
const BYTES_PER_CORNER: u64 = 24 + 8;
const BYTES_PER_INDEX: u64 = 4 + 4;

/// Writes `mesh` as one binary glTF through `<name>.tmp` next to `path` (see the module
/// docs). Pages without triangles are left out. A cancel through `progress` is checked per
/// page and before the rename; it leaves `path` as it was and no `.tmp`.
pub fn write_glb(path: &Path, mesh: &Mesh, progress: &Progress) -> Result<(), ExportError> {
    check_cancel(progress)?;
    check_mesh(path, mesh)?;
    let pages: Vec<&MeshPage> = mesh
        .pages
        .iter()
        .filter(|p| !p.indices.is_empty())
        .collect();

    // Layout of the binary chunk: per page positions, uvs, indices, then the images; every
    // part starts at a multiple of 4.
    let mut root = json::Root {
        asset: json::Asset {
            generator: Some(GENERATOR.into()),
            ..Default::default()
        },
        extensions_used: vec!["KHR_materials_unlit".into()],
        ..Default::default()
    };
    let mut offset = 0u64;
    let mut view = |root: &mut json::Root, len: u64, target: Option<json::buffer::Target>| {
        let v = root.push(json::buffer::View {
            buffer: json::Index::new(0),
            byte_length: USize64(len),
            byte_offset: Some(USize64(offset)),
            byte_stride: None,
            target: target.map(Checked::Valid),
            extensions: None,
            extras: Default::default(),
        });
        offset += padded(len);
        v
    };
    let sampler = pages.iter().any(|p| p.image.is_some()).then(|| {
        root.push(json::texture::Sampler {
            mag_filter: Some(Checked::Valid(json::texture::MagFilter::Linear)),
            min_filter: Some(Checked::Valid(json::texture::MinFilter::LinearMipmapLinear)),
            wrap_s: Checked::Valid(json::texture::WrappingMode::ClampToEdge),
            wrap_t: Checked::Valid(json::texture::WrappingMode::ClampToEdge),
            ..Default::default()
        })
    });
    let mut primitives = Vec::new();
    for page in &pages {
        let n = page.positions.len() as u64;
        let (lo, hi) = bounds(page.positions.iter().map(|&v| to_gltf(v)));
        let pos_view = view(&mut root, n * 12, Some(json::buffer::Target::ArrayBuffer));
        let uv_view = (!page.uvs.is_empty())
            .then(|| view(&mut root, n * 8, Some(json::buffer::Target::ArrayBuffer)));
        let idx_view = view(
            &mut root,
            page.indices.len() as u64 * 12,
            Some(json::buffer::Target::ElementArrayBuffer),
        );
        let accessor = |root: &mut json::Root,
                        view,
                        count: u64,
                        ty,
                        component,
                        minmax: Option<(json::Value, json::Value)>| {
            let (min, max) = minmax.unzip();
            root.push(json::Accessor {
                buffer_view: Some(view),
                byte_offset: None,
                count: USize64(count),
                component_type: Checked::Valid(json::accessor::GenericComponentType(component)),
                extensions: None,
                extras: Default::default(),
                type_: Checked::Valid(ty),
                min,
                max,
                normalized: false,
                sparse: None,
            })
        };
        let pos = accessor(
            &mut root,
            pos_view,
            n,
            json::accessor::Type::Vec3,
            json::accessor::ComponentType::F32,
            Some((
                json::Value::from(lo.to_vec()),
                json::Value::from(hi.to_vec()),
            )),
        );
        let uv = uv_view.map(|uv_view| {
            accessor(
                &mut root,
                uv_view,
                n,
                json::accessor::Type::Vec2,
                json::accessor::ComponentType::F32,
                None,
            )
        });
        let idx = accessor(
            &mut root,
            idx_view,
            page.indices.len() as u64 * 3,
            json::accessor::Type::Scalar,
            json::accessor::ComponentType::U32,
            None,
        );
        primitives.push((pos, uv, idx));
    }
    let mut gltf_mesh = json::Mesh {
        extensions: None,
        extras: Default::default(),
        primitives: Vec::new(),
        weights: None,
    };
    for ((pos, uv, idx), page) in primitives.into_iter().zip(&pages) {
        let unlit = Some(json::extensions::material::Material {
            unlit: Some(json::extensions::material::Unlit {}),
        });
        let material = match &page.image {
            Some(image) => {
                let image_view = view(&mut root, image.bytes.len() as u64, None);
                let image = root.push(json::Image {
                    buffer_view: Some(image_view),
                    mime_type: Some(json::image::MimeType(image.kind.mime_type().into())),
                    uri: None,
                    extensions: None,
                    extras: Default::default(),
                });
                let texture = root.push(json::Texture {
                    sampler,
                    source: image,
                    extensions: None,
                    extras: Default::default(),
                });
                json::Material {
                    pbr_metallic_roughness: json::material::PbrMetallicRoughness {
                        base_color_texture: Some(json::texture::Info {
                            index: texture,
                            tex_coord: 0,
                            extensions: None,
                            extras: Default::default(),
                        }),
                        metallic_factor: json::material::StrengthFactor(0.0),
                        roughness_factor: json::material::StrengthFactor(1.0),
                        ..Default::default()
                    },
                    extensions: unlit,
                    ..Default::default()
                }
            }
            None => {
                let [r, g, b] = page.color.map(|c| f32::from(c) / 255.0);
                json::Material {
                    pbr_metallic_roughness: json::material::PbrMetallicRoughness {
                        base_color_factor: json::material::PbrBaseColorFactor([r, g, b, 1.0]),
                        metallic_factor: json::material::StrengthFactor(0.0),
                        roughness_factor: json::material::StrengthFactor(1.0),
                        ..Default::default()
                    },
                    extensions: unlit,
                    ..Default::default()
                }
            }
        };
        let material = root.push(material);
        let mut attributes =
            BTreeMap::from([(Checked::Valid(json::mesh::Semantic::Positions), pos)]);
        if let Some(uv) = uv {
            attributes.insert(Checked::Valid(json::mesh::Semantic::TexCoords(0)), uv);
        }
        gltf_mesh.primitives.push(json::mesh::Primitive {
            attributes,
            extensions: None,
            extras: Default::default(),
            indices: Some(idx),
            material: Some(material),
            mode: Checked::Valid(json::mesh::Mode::Triangles),
            targets: None,
        });
    }
    let bin_len = offset;
    if !gltf_mesh.primitives.is_empty() {
        let gltf_mesh = root.push(gltf_mesh);
        let node = root.push(json::Node {
            mesh: Some(gltf_mesh),
            ..Default::default()
        });
        let scene = root.push(json::Scene {
            extensions: None,
            extras: Default::default(),
            nodes: vec![node],
        });
        root.scene = Some(scene);
    }
    if bin_len > 0 {
        root.push(json::Buffer {
            byte_length: USize64(bin_len),
            uri: None,
            extensions: None,
            extras: Default::default(),
        });
    }
    let mut text = root
        .to_vec()
        .map_err(|e| ExportError::format(path, format!("glTF-Beschreibung: {e}")))?;
    while text.len() % 4 != 0 {
        text.push(b' ');
    }

    let total = 12 + 8 + text.len() as u64 + if bin_len > 0 { 8 + bin_len } else { 0 };
    let total = u32::try_from(total).map_err(|_| {
        ExportError::format(path, "Oberfläche ist für eine glb-Datei zu groß (> 4 GB)")
    })?;
    write_atomic(path, progress, |w| {
        w.write_all(b"glTF")?;
        w.write_all(&2u32.to_le_bytes())?;
        w.write_all(&total.to_le_bytes())?;
        w.write_all(&(text.len() as u32).to_le_bytes())?;
        w.write_all(b"JSON")?;
        w.write_all(&text)?;
        if bin_len == 0 {
            return Ok(());
        }
        w.write_all(&(bin_len as u32).to_le_bytes())?;
        w.write_all(b"BIN\0")?;
        let mut written = 0u64;
        let mut part = |w: &mut dyn Write, bytes: &mut dyn Iterator<Item = [u8; 4]>| {
            let mut n = 0u64;
            for b in bytes {
                w.write_all(&b)?;
                n += 4;
            }
            written += n;
            Ok::<u64, io::Error>(n)
        };
        for page in &pages {
            stop_if_cancelled(progress)?;
            let pos = page.positions.iter().flat_map(|&v| to_gltf(v));
            part(w, &mut pos.map(f32::to_le_bytes))?;
            let uv = page.uvs.iter().flatten();
            part(w, &mut uv.map(|x| x.to_le_bytes()))?;
            let idx = page.indices.iter().flatten();
            part(w, &mut idx.map(|x| x.to_le_bytes()))?;
        }
        for image in pages.iter().filter_map(|p| p.image.as_ref()) {
            let bytes = &image.bytes;
            w.write_all(bytes)?;
            let pad = padded(bytes.len() as u64) - bytes.len() as u64;
            w.write_all(&[0u8; 3][..pad as usize])?;
            written += padded(bytes.len() as u64);
        }
        debug_assert_eq!(written, bin_len);
        Ok(())
    })
}

/// Driftless `(x, y, z)` (Z up) → glTF `(x, z, −y)` (Y up), as `f32`.
fn to_gltf(v: [f64; 3]) -> [f32; 3] {
    [v[0] as f32, v[2] as f32, -(v[1] as f32)]
}

/// glTF `(x, y, z)` (Y up) → Driftless `(x, −z, y)` (Z up), the inverse of [`to_gltf`].
fn from_gltf(v: [f64; 3]) -> [f64; 3] {
    [v[0], -v[2], v[1]]
}

fn bounds(points: impl Iterator<Item = [f32; 3]>) -> ([f32; 3], [f32; 3]) {
    points.fold(
        ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]),
        |(lo, hi), p| {
            (
                [0, 1, 2].map(|k| lo[k].min(p[k])),
                [0, 1, 2].map(|k| hi[k].max(p[k])),
            )
        },
    )
}

/// `len` rounded up to a multiple of 4.
fn padded(len: u64) -> u64 {
    len.div_ceil(4) * 4
}

/// Writes through `write` to `<path>.tmp` and renames it to `path` when complete and not
/// cancelled; on an error or a cancel the `.tmp` is removed.
pub(crate) fn write_atomic(
    path: &Path,
    progress: &Progress,
    write: impl FnOnce(&mut BufWriter<File>) -> io::Result<()>,
) -> Result<(), ExportError> {
    let temp = write_temp(path, write)?;
    check_cancel(progress)?;
    temp.commit().map_err(|e| ExportError::io(path, e))
}

/// What a writing closure returns once the export was asked to stop; [`write_temp`] turns it
/// into [`ExportError::Cancelled`].
#[derive(Debug)]
struct Stopped;

impl std::fmt::Display for Stopped {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("abgebrochen")
    }
}

impl std::error::Error for Stopped {}

/// Inside a writing closure: `Err` once `progress` was cancelled.
pub(crate) fn stop_if_cancelled(progress: &Progress) -> io::Result<()> {
    if progress.is_cancelled() {
        Err(io::Error::other(Stopped))
    } else {
        Ok(())
    }
}

/// Writes through `write` to `<path>.tmp` and closes it, without the rename.
pub(crate) fn write_temp(
    path: &Path,
    write: impl FnOnce(&mut BufWriter<File>) -> io::Result<()>,
) -> Result<TempPath, ExportError> {
    let io = |e| ExportError::io(path, e);
    let temp = TempPath::new(path);
    let mut w = BufWriter::new(File::create(temp.tmp()).map_err(io)?);
    write(&mut w).map_err(|e: io::Error| {
        if e.get_ref().is_some_and(|inner| inner.is::<Stopped>()) {
            ExportError::Cancelled
        } else {
            io(e)
        }
    })?;
    let file = w.into_inner().map_err(|e| io(e.into_error()))?;
    drop(file);
    Ok(temp)
}

/// Reads a glb (see the module docs). Progress: phase [`PHASE_READ`], one step per
/// primitive, a cancel check each.
pub(crate) fn read_glb(path: &Path, progress: &Progress) -> Result<Mesh, ExportError> {
    let fail = |msg: String| ExportError::format(path, msg);
    let bad = |e: gltf::Error| fail(format!("keine gültige glb-Datei ({e})"));
    let bytes = std::fs::read(path).map_err(|e| ExportError::io(path, e))?;
    let gltf = gltf::Gltf::from_slice_without_validation(&bytes).map_err(bad)?;
    // Before the validation: it would only say "unsupported extension".
    for ext in gltf.extensions_required() {
        match ext {
            "KHR_draco_mesh_compression" => {
                return Err(fail(
                    "glTF mit Draco-Kompression wird nicht unterstützt".into(),
                ))
            }
            "EXT_meshopt_compression" => {
                return Err(fail(
                    "glTF mit meshopt-Kompression wird nicht unterstützt".into(),
                ))
            }
            _ => {}
        }
    }
    if gltf.scenes().len() > 1 {
        return Err(fail(
            "glTF mit mehreren Szenen wird nicht unterstützt".into(),
        ));
    }
    let gltf::Gltf { document, blob } = gltf;
    let document = gltf::Document::from_json(document.into_json()).map_err(bad)?;
    // A buffer in a file of its own would otherwise be taken as a primitive without data.
    if let Some(uri) = document.buffers().find_map(|b| match b.source() {
        gltf::buffer::Source::Uri(uri) => Some(uri.to_string()),
        gltf::buffer::Source::Bin => None,
    }) {
        return Err(fail(format!(
            "Daten liegen in einer eigenen Datei ({uri}); nur eine glb mit allen Daten in der \
             Datei wird gelesen (wie `driftless surface` sie schreibt)"
        )));
    }

    // Every primitive with the world matrix of its node.
    let mut instances: Vec<(gltf::Primitive<'_>, Matrix4<f64>)> = Vec::new();
    let too_many = || {
        fail(format!(
            "glTF mit mehr als {MAX_INSTANCES} Flächen wird nicht unterstützt"
        ))
    };
    let scene = document
        .default_scene()
        .or_else(|| document.scenes().next());
    match scene {
        Some(scene) => {
            // Depth-first with an explicit stack; a node deeper than there are nodes means a
            // cycle.
            let limit = document.nodes().len();
            let mut stack: Vec<(gltf::Node<'_>, Matrix4<f64>, usize)> =
                scene.nodes().map(|n| (n, Matrix4::identity(), 0)).collect();
            stack.reverse();
            let mut visits = 0usize;
            while let Some((node, parent, depth)) = stack.pop() {
                visits += 1;
                if visits > MAX_VISITS {
                    return Err(too_many());
                }
                if depth > limit {
                    return Err(fail("glTF-Knoten bilden einen Kreis".into()));
                }
                let m = node.transform().matrix();
                // gltf gives columns.
                let local = Matrix4::from_fn(|r, c| f64::from(m[c][r]));
                let world = parent * local;
                if let Some(mesh) = node.mesh() {
                    add(&mut instances, mesh, world).map_err(|_| too_many())?;
                }
                let children: Vec<_> = node.children().collect();
                for child in children.into_iter().rev() {
                    stack.push((child, world, depth + 1));
                }
            }
        }
        None => {
            for mesh in document.meshes() {
                add(&mut instances, mesh, Matrix4::identity()).map_err(|_| too_many())?;
            }
        }
    }

    progress.phase(PHASE_READ, Some(instances.len() as u64));
    let blob = blob.as_deref();
    let limit = (bytes.len() as u64)
        .saturating_mul(DECODED_PER_FILE_BYTE)
        .clamp(DECODED_FLOOR, DECODED_MAX);
    let mut decoded = 0u64;
    // The image of each glTF image index, read once and shared by its pages.
    let mut images: HashMap<usize, PageImage> = HashMap::new();
    let mut pages = Vec::with_capacity(instances.len());
    for (prim, world) in &instances {
        check_cancel(progress)?;
        // Before anything of it is allocated.
        decoded = decoded.saturating_add(decoded_bytes(prim));
        if decoded > limit {
            return Err(fail(format!(
                "glTF ergibt mehr als {} MB Flächendaten (eine Fläche wird zu oft verwendet) – \
                 wird nicht gelesen",
                limit >> 20
            )));
        }
        // The gltf reader asserts on inconsistent accessors in places; that must never take
        // the program down.
        let page = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            read_primitive(prim, world, blob, &mut images)
        }))
        .unwrap_or_else(|_| Err("keine gültige glb-Datei (unerwarteter Inhalt)".into()))
        .map_err(fail)?;
        pages.push(page);
        progress.advance(1);
    }
    Ok(Mesh { pages })
}

/// Adds the primitives of `mesh` at `world`; `Err` past [`MAX_INSTANCES`].
fn add<'a>(
    instances: &mut Vec<(gltf::Primitive<'a>, Matrix4<f64>)>,
    mesh: gltf::Mesh<'a>,
    world: Matrix4<f64>,
) -> Result<(), ()> {
    for prim in mesh.primitives() {
        if instances.len() >= MAX_INSTANCES {
            return Err(());
        }
        instances.push((prim, world));
    }
    Ok(())
}

/// What reading `prim` allocates for its corners and triangles, from the counts its
/// accessors claim.
fn decoded_bytes(prim: &gltf::Primitive<'_>) -> u64 {
    let corners = prim
        .get(&gltf::Semantic::Positions)
        .map_or(0, |a| a.count() as u64);
    let indices = prim.indices().map_or(corners, |a| a.count() as u64);
    corners
        .saturating_mul(BYTES_PER_CORNER)
        .saturating_add(indices.saturating_mul(BYTES_PER_INDEX))
}

/// One primitive as a page, its positions through `world` (glTF frame) and then to Z up.
/// Its image comes from `images` if another page has read it already.
fn read_primitive(
    prim: &gltf::Primitive<'_>,
    world: &Matrix4<f64>,
    blob: Option<&[u8]>,
    images: &mut HashMap<usize, PageImage>,
) -> Result<MeshPage, String> {
    use gltf::accessor::{DataType, Dimensions};
    if prim.mode() != gltf::mesh::Mode::Triangles {
        return Err(format!(
            "Grundform {:?} statt Dreiecken wird nicht unterstützt",
            prim.mode()
        ));
    }
    let inconsistent = || "Fläche mit unstimmigen Ecken oder Dreiecken".to_string();
    let buffer = |b: gltf::Buffer<'_>| match b.source() {
        gltf::buffer::Source::Bin => blob,
        gltf::buffer::Source::Uri(_) => None,
    };
    let reader = prim.reader(buffer);
    let pos = prim
        .get(&gltf::Semantic::Positions)
        .ok_or("Fläche ohne Positionen")?;
    if pos.dimensions() != Dimensions::Vec3 || pos.data_type() != DataType::F32 {
        return Err(inconsistent());
    }
    let identity = *world == Matrix4::identity();
    let mut positions: Vec<[f64; 3]> = if pos.count() == 0 {
        Vec::new()
    } else {
        reader
            .read_positions()
            .ok_or_else(inconsistent)?
            .map(|p| {
                let p = p.map(f64::from);
                if identity {
                    // Exactly as stored, −0.0 included, so a glb reads back to the same bytes.
                    return from_gltf(p);
                }
                let v = world * Vector4::new(p[0], p[1], p[2], 1.0);
                from_gltf([v.x, v.y, v.z])
            })
            .collect()
    };
    if positions.len() != pos.count() {
        return Err(inconsistent());
    }
    if positions.iter().any(|p| !p.iter().all(|c| c.is_finite())) {
        return Err("Fläche mit ungültigen Lagen".into());
    }
    let n = positions.len();

    let material = prim.material();
    let pbr = material.pbr_metallic_roughness();
    let factor = pbr.base_color_factor();
    let color = [0, 1, 2].map(|k| (factor[k].clamp(0.0, 1.0) * 255.0).round() as u8);
    let (image, set) = match pbr.base_color_texture() {
        None => (None, 0),
        Some(info) => match info.texture().source().source() {
            _ if images.contains_key(&info.texture().source().index()) => {
                let image = images[&info.texture().source().index()].clone();
                (Some(image), info.tex_coord())
            }
            gltf::image::Source::View { view, mime_type } => {
                let kind = match mime_type {
                    "image/jpeg" => ImageKind::Jpeg,
                    "image/png" => ImageKind::Png,
                    other => return Err(format!("Texturformat {other} nicht unterstützt")),
                };
                let data = buffer(view.buffer()).ok_or("Textur liegt nicht in der Datei")?;
                let bytes = data
                    .get(view.offset()..view.offset() + view.length())
                    .ok_or("Textur außerhalb der Daten")?;
                let image = PageImage {
                    kind,
                    bytes: bytes.into(),
                };
                images.insert(info.texture().source().index(), image.clone());
                (Some(image), info.tex_coord())
            }
            gltf::image::Source::Uri { .. } => {
                return Err("Textur als eigene Datei wird nicht unterstützt".into())
            }
        },
    };
    let mut uvs: Vec<[f32; 2]> = match prim.get(&gltf::Semantic::TexCoords(set)) {
        Some(acc) if acc.count() > 0 => {
            let ok = acc.dimensions() == Dimensions::Vec2
                && matches!(
                    acc.data_type(),
                    DataType::F32 | DataType::U8 | DataType::U16
                );
            if !ok {
                return Err(inconsistent());
            }
            reader
                .read_tex_coords(set)
                .ok_or_else(inconsistent)?
                .into_f32()
                .collect()
        }
        _ => Vec::new(),
    };
    if !uvs.is_empty() && uvs.len() != n {
        return Err(inconsistent());
    }
    if image.is_some() && uvs.is_empty() {
        uvs = vec![[0.5; 2]; n];
    }
    let flat: Vec<u32> = match prim.indices() {
        Some(acc) => {
            let ok = acc.dimensions() == Dimensions::Scalar
                && matches!(
                    acc.data_type(),
                    DataType::U8 | DataType::U16 | DataType::U32
                );
            if !ok {
                return Err(inconsistent());
            }
            if acc.count() == 0 {
                Vec::new()
            } else {
                reader
                    .read_indices()
                    .ok_or_else(inconsistent)?
                    .into_u32()
                    .collect()
            }
        }
        None => (0..n as u32).collect(),
    };
    if !flat.len().is_multiple_of(3) || flat.iter().any(|&i| i as usize >= n) {
        return Err(inconsistent());
    }
    let mut indices: Vec<[u32; 3]> = flat.as_chunks::<3>().0.to_vec();
    // A mirroring node turns the triangles inside out; swap two corners to keep the fronts.
    if world.fixed_view::<3, 3>(0, 0).determinant() < 0.0 {
        for t in &mut indices {
            t.swap(1, 2);
        }
    }
    positions.shrink_to_fit();
    Ok(MeshPage {
        positions,
        uvs,
        indices,
        image,
        color,
    })
}
