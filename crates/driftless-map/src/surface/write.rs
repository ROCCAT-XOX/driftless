//! Writing a [`TexturedMesh`] as `.glb` and as `.obj` with `.mtl` and JPEG pages: thin
//! wrappers that turn it into a [`driftless_export::mesh::Mesh`] ([`TexturedMesh::to_mesh`])
//! and hand it to the writers of `driftless-export`, which describe the formats.
//!
//! The pages are encoded once ([`encode_pages`]) for both files; only the compressed JPEGs
//! are held.

use super::TexturedMesh;
use crate::MapError;
use driftless_export::mesh::{self, ImageKind, Mesh, MeshPage, PageImage};
use image::codecs::jpeg::JpegEncoder;
use image::ExtendedColorType;
use std::path::Path;

/// The texture pages of a [`TexturedMesh`] as JPEG, encoded once for the glb and the obj.
#[derive(Debug, Clone, PartialEq)]
pub struct PageJpegs {
    jpegs: Vec<Vec<u8>>,
}

impl PageJpegs {
    /// Number of pages.
    pub fn len(&self) -> usize {
        self.jpegs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.jpegs.is_empty()
    }

    /// The JPEG of page `p`.
    pub fn page(&self, p: usize) -> &[u8] {
        &self.jpegs[p]
    }

    /// Fails if these are not the pages of `tex`.
    fn check(&self, tex: &TexturedMesh) -> Result<(), MapError> {
        if self.jpegs.len() != tex.pages.len() {
            return Err(MapError::Config(format!(
                "{} JPEG-Seiten für {} Texturseiten",
                self.jpegs.len(),
                tex.pages.len()
            )));
        }
        Ok(())
    }
}

/// Encodes every page of `tex` as baseline JPEG of `jpeg_quality`.
pub fn encode_pages(tex: &TexturedMesh, jpeg_quality: u8) -> Result<PageJpegs, MapError> {
    let jpegs = tex
        .pages
        .iter()
        .map(|page| encode_jpeg(&page.image, jpeg_quality))
        .collect::<Result<_, _>>()?;
    Ok(PageJpegs { jpegs })
}

impl TexturedMesh {
    /// The mesh with its encoded pages `jpegs`, for the writers of `driftless-export`:
    /// positions as `f64`, every page with its JPEG.
    pub fn to_mesh(&self, jpegs: &PageJpegs) -> Result<Mesh, MapError> {
        jpegs.check(self)?;
        let pages = self
            .pages
            .iter()
            .enumerate()
            .map(|(p, page)| MeshPage {
                positions: page.positions.iter().map(|v| v.map(f64::from)).collect(),
                uvs: page.uvs.clone(),
                indices: page.indices.clone(),
                image: Some(PageImage {
                    kind: ImageKind::Jpeg,
                    bytes: jpegs.page(p).into(),
                }),
                color: [255; 3],
            })
            .collect();
        Ok(Mesh { pages })
    }
}

/// Writes `tex` with its encoded pages `jpegs` as one binary glTF (Y up, see
/// [`driftless_export::mesh::write_glb`]).
pub fn write_glb(path: &Path, tex: &TexturedMesh, jpegs: &PageJpegs) -> Result<(), MapError> {
    Ok(mesh::write_glb(
        path,
        &tex.to_mesh(jpegs)?,
        &driftless_io::progress::Progress::new(),
    )?)
}

/// Writes `tex` as `stem.obj`, `stem.mtl` and its encoded pages `jpegs` as one
/// `stem_seite_NN.jpg` per page into `dir` (see [`driftless_export::mesh::write_obj`]). The
/// `.obj` comes last: once it is there, the rest is too.
pub fn write_obj(
    dir: &Path,
    stem: &str,
    tex: &TexturedMesh,
    jpegs: &PageJpegs,
) -> Result<(), MapError> {
    let path = dir.join(format!("{stem}.obj"));
    Ok(mesh::write_obj(
        &path,
        &tex.to_mesh(jpegs)?,
        &driftless_io::progress::Progress::new(),
    )?)
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

#[cfg(test)]
mod tests {
    use super::super::testutil::*;
    use super::*;
    use driftless_io::images::decode_jpeg;
    use driftless_io::testdir::TestDir;
    use std::fs;

    fn temp_dir(name: &str) -> TestDir {
        TestDir::new(format!(
            "driftless-surface-write-{}-{name}",
            std::process::id()
        ))
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
        write_glb(&path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
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
        write_glb(&path, &tex, &encode_pages(&tex, 80).unwrap()).unwrap();
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
        write_obj(&dir, "oberflaeche", &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
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
            let pages = encode_pages(&tex, 90).unwrap();
            write_glb(&d.join("x.glb"), &tex, &pages).unwrap();
            write_obj(d, "x", &tex, &pages).unwrap();
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

    /// The bytes of today's glb, obj and mtl, pinned before the writers moved to
    /// `driftless-export`: the move must not change a single byte.
    #[test]
    fn files_unchanged_by_move() {
        let tex = floor_and_ceiling();
        let dir = temp_dir("unchanged");
        let pages = encode_pages(&tex, 90).unwrap();
        write_glb(&dir.join("x.glb"), &tex, &pages).unwrap();
        write_obj(&dir, "x", &tex, &pages).unwrap();
        let hash = |name: &str| crate::content_hash(&fs::read(dir.join(name)).unwrap());
        let got = [hash("x.glb"), hash("x.obj"), hash("x.mtl")];
        assert_eq!(
            got,
            [0xacdf49bc5448d4b8, 0x5aab2ed88a5d7333, 0xe8f7b8df59fdd85e],
            "{got:#018x?}"
        );
    }

    #[test]
    fn pages_are_encoded_once_for_glb_and_obj() {
        let tex = floor_and_ceiling();
        let pages = encode_pages(&tex, 90).unwrap();
        assert_eq!(pages.len(), tex.pages.len());
        let dir = temp_dir("once");
        write_glb(&dir.join("x.glb"), &tex, &pages).unwrap();
        write_obj(&dir, "x", &tex, &pages).unwrap();
        // The jpg files and the glb's embedded images are the very same bytes.
        let glb = fs::read(dir.join("x.glb")).unwrap();
        for p in 0..tex.pages.len() {
            let jpg = fs::read(dir.join(format!("x_seite_{p:02}.jpg"))).unwrap();
            assert_eq!(jpg, pages.page(p));
            assert!(glb.windows(jpg.len()).any(|w| w == jpg), "page {p}");
        }
        // Pages of another mesh are refused.
        let mut fewer = tex.clone();
        fewer.pages.pop();
        let other = encode_pages(&fewer, 90).unwrap();
        assert!(write_glb(&dir.join("y.glb"), &tex, &other).is_err());
        assert!(write_obj(&dir, "y", &tex, &other).is_err());
        assert!(!dir.join("y.glb").exists() && !dir.join("y.obj").exists());
    }
}
