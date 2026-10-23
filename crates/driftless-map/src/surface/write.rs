//! Writing a [`TexturedMesh`] as `.glb` and as `.obj` with `.mtl` and JPEG pages.
//!
//! **glb** (glTF 2.0, binary): metres, Y up — a Driftless point `(x, y, z)` (Z up) becomes
//! `(x, z, −y)`, a rotation, so the winding and the front sides stay. One mesh with one
//! primitive per non-empty page (positions, `TEXCOORD_0`, `u32` indices), one material per
//! page: `baseColorTexture` with the page as embedded JPEG and `KHR_materials_unlit` (the
//! photos are already lit). glTF texture coordinates start at the top left with `v` down,
//! as ours, so they are written unchanged.
//!
//! **obj**: stays Z up (Blender: import with "Z up"); texture coordinates with `v` up, so
//! `1 − v` is written. One material `seite_NN` per page with its JPEG `stem_seite_NN.jpg`.
//!
//! Both are deterministic: fixed generator string, no time, fixed key and file order, floats
//! in their shortest round-trip form. Every file goes to `<name>.tmp` first and is renamed
//! when complete, so an interrupted write leaves no half file under the real name. Pages are
//! encoded one at a time; only the compressed JPEGs are held together.

use super::TexturedMesh;
use crate::MapError;
use gltf_json as json;
use image::codecs::jpeg::JpegEncoder;
use image::ExtendedColorType;
use json::validation::{Checked, USize64};
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

/// `asset.generator` of every glb.
pub const GENERATOR: &str = "Driftless";

/// Writes `tex` as one binary glTF (see the module docs).
pub fn write_glb(path: &Path, tex: &TexturedMesh, jpeg_quality: u8) -> Result<(), MapError> {
    let pages: Vec<usize> = (0..tex.pages.len())
        .filter(|&p| !tex.pages[p].indices.is_empty())
        .collect();
    let jpegs: Vec<Vec<u8>> = pages
        .iter()
        .map(|&p| encode_jpeg(&tex.pages[p].image, jpeg_quality))
        .collect::<Result<_, _>>()?;

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
    let sampler = root.push(json::texture::Sampler {
        mag_filter: Some(Checked::Valid(json::texture::MagFilter::Linear)),
        min_filter: Some(Checked::Valid(json::texture::MinFilter::LinearMipmapLinear)),
        wrap_s: Checked::Valid(json::texture::WrappingMode::ClampToEdge),
        wrap_t: Checked::Valid(json::texture::WrappingMode::ClampToEdge),
        ..Default::default()
    });
    let mut primitives = Vec::new();
    for &p in &pages {
        let page = &tex.pages[p];
        let n = page.positions.len() as u64;
        let (lo, hi) = bounds(page.positions.iter().map(|&v| to_gltf(v)));
        let pos_view = view(&mut root, n * 12, Some(json::buffer::Target::ArrayBuffer));
        let uv_view = view(&mut root, n * 8, Some(json::buffer::Target::ArrayBuffer));
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
        let uv = accessor(
            &mut root,
            uv_view,
            n,
            json::accessor::Type::Vec2,
            json::accessor::ComponentType::F32,
            None,
        );
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
    let mut mesh = json::Mesh {
        extensions: None,
        extras: Default::default(),
        primitives: Vec::new(),
        weights: None,
    };
    for ((pos, uv, idx), jpeg) in primitives.into_iter().zip(&jpegs) {
        let image_view = view(&mut root, jpeg.len() as u64, None);
        let image = root.push(json::Image {
            buffer_view: Some(image_view),
            mime_type: Some(json::image::MimeType("image/jpeg".into())),
            uri: None,
            extensions: None,
            extras: Default::default(),
        });
        let texture = root.push(json::Texture {
            sampler: Some(sampler),
            source: image,
            extensions: None,
            extras: Default::default(),
        });
        let material = root.push(json::Material {
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
            extensions: Some(json::extensions::material::Material {
                unlit: Some(json::extensions::material::Unlit {}),
            }),
            ..Default::default()
        });
        mesh.primitives.push(json::mesh::Primitive {
            attributes: BTreeMap::from([
                (Checked::Valid(json::mesh::Semantic::Positions), pos),
                (Checked::Valid(json::mesh::Semantic::TexCoords(0)), uv),
            ]),
            extensions: None,
            extras: Default::default(),
            indices: Some(idx),
            material: Some(material),
            mode: Checked::Valid(json::mesh::Mode::Triangles),
            targets: None,
        });
    }
    let bin_len = offset;
    if !mesh.primitives.is_empty() {
        let mesh = root.push(mesh);
        let node = root.push(json::Node {
            mesh: Some(mesh),
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
        .map_err(|e| MapError::Config(format!("glTF-Beschreibung: {e}")))?;
    while text.len() % 4 != 0 {
        text.push(b' ');
    }

    let total = 12 + 8 + text.len() as u64 + if bin_len > 0 { 8 + bin_len } else { 0 };
    let total = u32::try_from(total).map_err(|_| {
        MapError::Config("Oberfläche ist für eine glb-Datei zu groß (> 4 GB)".into())
    })?;
    write_atomic(path, |w| {
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
        for &p in &pages {
            let page = &tex.pages[p];
            let pos = page.positions.iter().flat_map(|&v| to_gltf(v));
            part(w, &mut pos.map(f32::to_le_bytes))?;
            let uv = page.uvs.iter().flatten();
            part(w, &mut uv.map(|x| x.to_le_bytes()))?;
            let idx = page.indices.iter().flatten();
            part(w, &mut idx.map(|x| x.to_le_bytes()))?;
        }
        for jpeg in &jpegs {
            w.write_all(jpeg)?;
            let pad = padded(jpeg.len() as u64) - jpeg.len() as u64;
            w.write_all(&[0u8; 3][..pad as usize])?;
            written += padded(jpeg.len() as u64);
        }
        debug_assert_eq!(written, bin_len);
        Ok(())
    })
}

/// Writes `tex` as `stem.obj`, `stem.mtl` and one `stem_seite_NN.jpg` per page into `dir`
/// (see the module docs). The `.obj` comes last: once it is there, the rest is too.
pub fn write_obj(
    dir: &Path,
    stem: &str,
    tex: &TexturedMesh,
    jpeg_quality: u8,
) -> Result<(), MapError> {
    let jpg = |p: usize| format!("{stem}_{}.jpg", page_name(p));
    for (p, page) in tex.pages.iter().enumerate() {
        let bytes = encode_jpeg(&page.image, jpeg_quality)?;
        write_atomic(&dir.join(jpg(p)), |w| w.write_all(&bytes))?;
    }
    write_atomic(&dir.join(format!("{stem}.mtl")), |w| {
        writeln!(
            w,
            "# Driftless surface: one material per texture page, unlit photos"
        )?;
        for p in 0..tex.pages.len() {
            writeln!(w, "\nnewmtl {}", page_name(p))?;
            writeln!(w, "Ka 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 0")?;
            writeln!(w, "map_Kd {}", jpg(p))?;
        }
        Ok(())
    })?;
    write_atomic(&dir.join(format!("{stem}.obj")), |w| {
        // OBJ keeps Driftless' axes: Z up, metres (import into Blender with "Z up").
        writeln!(w, "# Driftless surface: metres, Z up")?;
        writeln!(w, "mtllib {stem}.mtl")?;
        writeln!(w, "o oberflaeche")?;
        let mut base = 1usize;
        for (p, page) in tex.pages.iter().enumerate() {
            for v in &page.positions {
                writeln!(w, "v {} {} {}", v[0], v[1], v[2])?;
            }
            for t in &page.uvs {
                // OBJ texture coordinates have v up.
                writeln!(w, "vt {} {}", t[0], 1.0 - t[1])?;
            }
            writeln!(w, "usemtl {}", page_name(p))?;
            for tri in &page.indices {
                let [a, b, c] = tri.map(|i| i as usize + base);
                writeln!(w, "f {a}/{a} {b}/{b} {c}/{c}")?;
            }
            base += page.positions.len();
        }
        Ok(())
    })
}

/// `seite_NN`: the name of page `p` (material, image).
fn page_name(p: usize) -> String {
    format!("seite_{p:02}")
}

/// Driftless `(x, y, z)` (Z up) → glTF `(x, z, −y)` (Y up).
fn to_gltf(v: [f32; 3]) -> [f32; 3] {
    [v[0], v[2], -v[1]]
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

/// Baseline JPEG of `image` (`image` crate encoder: pure Rust, MIT/Apache, deterministic,
/// encodes from the borrowed pixels without a copy).
fn encode_jpeg(image: &driftless_io::images::RgbImage, quality: u8) -> Result<Vec<u8>, MapError> {
    let (w, h) = (image.width, image.height);
    if image.data.len() as u64 != u64::from(w) * u64::from(h) * 3 {
        return Err(MapError::Config(format!(
            "Texturseite {w} × {h} hat {} statt {} Bytes",
            image.data.len(),
            u64::from(w) * u64::from(h) * 3
        )));
    }
    let mut out = Vec::new();
    JpegEncoder::new_with_quality(&mut out, quality)
        .encode(&image.data, w, h, ExtendedColorType::Rgb8)
        .map_err(|e| MapError::Config(format!("Textur als JPEG: {e}")))?;
    Ok(out)
}

/// Writes through `write` to `<path>.tmp` and renames it to `path` when complete; on an
/// error the `.tmp` is removed.
fn write_atomic(
    path: &Path,
    write: impl FnOnce(&mut BufWriter<File>) -> io::Result<()>,
) -> Result<(), MapError> {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".tmp");
    let tmp: PathBuf = path.with_file_name(name);
    let result = (|| {
        let mut w = BufWriter::new(File::create(&tmp)?);
        write(&mut w)?;
        w.into_inner().map_err(|e| e.into_error())?.sync_all()?;
        fs::rename(&tmp, path)
    })();
    result.map_err(|source| {
        let _ = fs::remove_file(&tmp);
        MapError::Io {
            path: path.display().to_string(),
            source,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::super::testutil::*;
    use super::*;
    use driftless_io::images::decode_jpeg;
    use std::fs;
    use std::path::PathBuf;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "driftless-surface-write-{}-{name}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The triangles (glTF positions, uvs) of every primitive, and its decoded texture.
    type Prim = (Vec<[[f32; 3]; 3]>, Vec<[[f32; 2]; 3]>, (u32, u32, Vec<u8>));

    fn read_glb(path: &Path) -> Vec<Prim> {
        let bytes = fs::read(path).unwrap();
        let gltf = gltf::Gltf::from_slice(&bytes).unwrap();
        let blob = gltf.blob.as_deref().expect("binary chunk");
        assert!(gltf.extensions_used().any(|e| e == "KHR_materials_unlit"));
        let mesh = gltf.meshes().next().expect("one mesh");
        assert_eq!(gltf.meshes().count(), 1);
        mesh.primitives()
            .map(|p| {
                let r = p.reader(|_| Some(blob));
                let pos: Vec<[f32; 3]> = r.read_positions().unwrap().collect();
                let uv: Vec<[f32; 2]> = r.read_tex_coords(0).unwrap().into_f32().collect();
                let idx: Vec<u32> = r.read_indices().unwrap().into_u32().collect();
                let tris = idx
                    .chunks(3)
                    .map(|t| [0, 1, 2].map(|k| pos[t[k] as usize]))
                    .collect();
                let uvs = idx
                    .chunks(3)
                    .map(|t| [0, 1, 2].map(|k| uv[t[k] as usize]))
                    .collect();
                let mat = p.material();
                assert!(mat.unlit());
                let info = mat.pbr_metallic_roughness().base_color_texture().unwrap();
                let gltf::image::Source::View { view, mime_type } =
                    info.texture().source().source()
                else {
                    panic!("image not embedded");
                };
                assert_eq!(mime_type, "image/jpeg");
                let jpg = &blob[view.offset()..view.offset() + view.length()];
                let img = decode_jpeg(jpg, 1).expect("texture decodes");
                (tris, uvs, (img.width, img.height, img.data))
            })
            .collect()
    }

    fn close(a: [f32; 3], b: [f32; 3]) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() <= 1e-6)
    }

    #[test]
    fn glb_roundtrip() {
        let tex = floor_and_ceiling();
        let dir = temp_dir("roundtrip");
        let path = dir.join("oberflaeche.glb");
        write_glb(&path, &tex, 90).unwrap();
        assert!(!dir.join("oberflaeche.glb.tmp").exists());
        let prims = read_glb(&path);
        assert_eq!(prims.len(), tex.pages.len());
        for (prim, page) in prims.iter().zip(&tex.pages) {
            assert_eq!(prim.0.len(), page.indices.len());
            for (t, tri) in page.indices.iter().enumerate() {
                for (k, &i) in tri.iter().enumerate() {
                    let [x, y, z] = page.positions[i as usize];
                    assert!(close(prim.0[t][k], [x, z, -y]), "{:?}", prim.0[t][k]);
                    // glTF texcoords: origin top left, v down, as ours.
                    assert_eq!(prim.1[t][k], page.uvs[i as usize]);
                }
            }
            let (w, h, data) = &prim.2;
            assert_eq!((*w, *h), (page.image.width, page.image.height));
            // Row 1 is the top colour, the last row the bottom colour (JPEG ± a little).
            let px = |row: u32| &data[(row * w * 3 + w / 2 * 3) as usize..][..3];
            let near = |a: &[u8], b: &[u8]| a.iter().zip(b).all(|(x, y)| x.abs_diff(*y) < 25);
            assert!(near(px(1), &page.image.data[..3]), "{:?}", px(1));
            let last = page.image.data.len() - 3;
            assert!(near(px(h - 2), &page.image.data[last..]), "{:?}", px(h - 2));
        }
    }

    #[test]
    fn glb_is_y_up() {
        let tex = floor_and_ceiling();
        let dir = temp_dir("yup");
        let path = dir.join("a.glb");
        write_glb(&path, &tex, 80).unwrap();
        let prims = read_glb(&path);
        let ys = |p: &Prim| -> Vec<f32> { p.0.iter().flatten().map(|v| v[1]).collect() };
        assert!(
            ys(&prims[0]).iter().all(|&y| y.abs() < 1e-6),
            "floor at y = 0"
        );
        assert!(
            ys(&prims[1]).iter().all(|&y| (y - 2.5).abs() < 1e-6),
            "ceiling at y = 2.5"
        );
        // Driftless +y (3 m deep room) becomes glTF −z.
        let zs: Vec<f32> = prims[0].0.iter().flatten().map(|v| v[2]).collect();
        assert!(zs.iter().all(|&z| (-3.0..=0.0).contains(&z)));
        assert!(zs.iter().any(|&z| z < -2.9));
    }

    #[test]
    fn obj_files_complete() {
        let tex = floor_and_ceiling();
        let dir = temp_dir("obj");
        write_obj(&dir, "oberflaeche", &tex, 90).unwrap();
        let obj = fs::read_to_string(dir.join("oberflaeche.obj")).unwrap();
        let mtl = fs::read_to_string(dir.join("oberflaeche.mtl")).unwrap();
        assert!(obj.lines().any(|l| l == "mtllib oberflaeche.mtl"));
        for p in 0..tex.pages.len() {
            let name = format!("seite_{p:02}");
            let jpg = format!("oberflaeche_{name}.jpg");
            let img = decode_jpeg(&fs::read(dir.join(&jpg)).unwrap(), 1).unwrap();
            assert_eq!(img.width, tex.pages[p].image.width);
            assert!(obj.lines().any(|l| l == format!("usemtl {name}")));
            assert!(mtl.lines().any(|l| l == format!("newmtl {name}")));
            assert!(mtl.lines().any(|l| l == format!("map_Kd {jpg}")));
        }
        let entries: Vec<String> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        assert_eq!(entries.len(), 2 + tex.pages.len(), "{entries:?}");
        assert!(!entries.iter().any(|e| e.ends_with(".tmp")));

        // Vertices (Z up, unchanged), texcoords with v up, faces pointing at both.
        let v: Vec<[f32; 3]> = obj
            .lines()
            .filter_map(|l| l.strip_prefix("v "))
            .map(|l| {
                let n: Vec<f32> = l.split(' ').map(|x| x.parse().unwrap()).collect();
                [n[0], n[1], n[2]]
            })
            .collect();
        let vt: Vec<[f32; 2]> = obj
            .lines()
            .filter_map(|l| l.strip_prefix("vt "))
            .map(|l| {
                let n: Vec<f32> = l.split(' ').map(|x| x.parse().unwrap()).collect();
                [n[0], n[1]]
            })
            .collect();
        let faces: Vec<[(usize, usize); 3]> = obj
            .lines()
            .filter_map(|l| l.strip_prefix("f "))
            .map(|l| {
                let c: Vec<(usize, usize)> = l
                    .split(' ')
                    .map(|c| {
                        let (a, b) = c.split_once('/').unwrap();
                        (a.parse().unwrap(), b.parse().unwrap())
                    })
                    .collect();
                [c[0], c[1], c[2]]
            })
            .collect();
        let total: usize = tex.pages.iter().map(|p| p.indices.len()).sum();
        assert_eq!(faces.len(), total);
        let mut f = 0;
        for page in &tex.pages {
            for tri in &page.indices {
                for k in 0..3 {
                    let (vi, ti) = faces[f][k];
                    assert_eq!(v[vi - 1], page.positions[tri[k] as usize]);
                    let [u, w] = page.uvs[tri[k] as usize];
                    assert_eq!(vt[ti - 1], [u, 1.0 - w]);
                }
                f += 1;
            }
        }
    }

    #[test]
    fn byte_identical_twice() {
        let tex = floor_and_ceiling();
        let (a, b) = (temp_dir("same-a"), temp_dir("same-b"));
        for d in [&a, &b] {
            write_glb(&d.join("x.glb"), &tex, 90).unwrap();
            write_obj(d, "x", &tex, 90).unwrap();
        }
        for name in [
            "x.glb",
            "x.obj",
            "x.mtl",
            "x_seite_00.jpg",
            "x_seite_01.jpg",
        ] {
            let (x, y) = (
                fs::read(a.join(name)).unwrap(),
                fs::read(b.join(name)).unwrap(),
            );
            assert!(!x.is_empty(), "{name}");
            assert!(x == y, "{name} differs");
        }
    }
}
