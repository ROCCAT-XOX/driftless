//! Meshes: glb and OBJ written and read back (own and foreign files), and the PLY mesh with
//! one colour per corner averaged from the texture.

use driftless_export::mesh::{
    obj_files, read_mesh, write_glb, write_obj, write_ply_mesh, ImageKind, Mesh, MeshFormat,
    MeshPage, PageImage, GENERATOR,
};
use driftless_export::{ExportError, PHASE_WRITE};
use driftless_io::progress::{LogLine, Progress};
use image::codecs::jpeg::JpegEncoder;
use image::codecs::png::PngEncoder;
use image::{ExtendedColorType, ImageEncoder};
use std::path::{Path, PathBuf};

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "driftless-export-mesh-{}-{name}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Names of the files in `dir`, sorted.
fn entries(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    names.sort();
    names
}

/// `w × h` RGB pixels: the left half `left`, the right half `right`.
fn halves(w: u32, h: u32, left: [u8; 3], right: [u8; 3]) -> Vec<u8> {
    (0..h)
        .flat_map(|_| (0..w).flat_map(move |x| if x < w / 2 { left } else { right }))
        .collect()
}

fn png(w: u32, h: u32, rgb: &[u8]) -> PageImage {
    let mut bytes = Vec::new();
    PngEncoder::new(&mut bytes)
        .write_image(rgb, w, h, ExtendedColorType::Rgb8)
        .unwrap();
    PageImage {
        kind: ImageKind::Png,
        bytes: bytes.into(),
    }
}

fn jpeg(w: u32, h: u32, rgb: &[u8]) -> PageImage {
    let mut bytes = Vec::new();
    JpegEncoder::new_with_quality(&mut bytes, 90)
        .encode(rgb, w, h, ExtendedColorType::Rgb8)
        .unwrap();
    PageImage {
        kind: ImageKind::Jpeg,
        bytes: bytes.into(),
    }
}

/// A rectangle `corners` (counter-clockwise seen from its front) as two triangles, uvs
/// over the whole page (v down: the first two corners at v = 1, the bottom row).
fn quad(corners: [[f64; 3]; 4], image: Option<PageImage>, color: [u8; 3]) -> MeshPage {
    MeshPage {
        positions: corners.to_vec(),
        uvs: vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        indices: vec![[0, 1, 2], [0, 2, 3]],
        image,
        color,
    }
}

/// Floor (JPEG), wall (PNG) and ceiling (no image, a colour), plus an empty page.
fn small_mesh() -> Mesh {
    let floor = quad(
        [
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 3.0, 0.0],
            [0.0, 3.0, 0.0],
        ],
        Some(jpeg(16, 16, &halves(16, 16, [200, 30, 30], [30, 30, 200]))),
        [255; 3],
    );
    let wall = quad(
        [
            [0.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [0.0, 3.0, 2.5],
            [0.0, 0.0, 2.5],
        ],
        Some(png(8, 4, &halves(8, 4, [10, 250, 10], [250, 250, 10]))),
        [255; 3],
    );
    let ceiling = quad(
        [
            [0.0, 0.0, 2.5],
            [0.0, 3.0, 2.5],
            [4.0, 3.0, 2.5],
            [4.0, 0.0, 2.5],
        ],
        None,
        [90, 120, 150],
    );
    Mesh {
        pages: vec![floor, wall, ceiling],
    }
}

/// The corners of a triangle and their uvs.
type Corners = ([[f64; 3]; 3], [[f32; 2]; 3]);

/// The corners of every triangle of `page`, and their uvs.
fn triangles(page: &MeshPage) -> Vec<Corners> {
    page.indices
        .iter()
        .map(|t| {
            (
                t.map(|i| page.positions[i as usize]),
                t.map(|i| page.uvs[i as usize]),
            )
        })
        .collect()
}

fn format_msg(e: ExportError) -> String {
    match e {
        ExportError::Format { msg, .. } => msg,
        other => panic!("expected a format error, got {other:?}"),
    }
}

/// A binary glTF of `json` and the binary chunk `bin`.
fn glb_bytes(json: &str, bin: &[u8]) -> Vec<u8> {
    let mut text = json.as_bytes().to_vec();
    while !text.len().is_multiple_of(4) {
        text.push(b' ');
    }
    let mut bin = bin.to_vec();
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }
    let total = 12 + 8 + text.len() + if bin.is_empty() { 0 } else { 8 + bin.len() };
    let mut out = Vec::new();
    out.extend(b"glTF");
    out.extend(2u32.to_le_bytes());
    out.extend((total as u32).to_le_bytes());
    out.extend((text.len() as u32).to_le_bytes());
    out.extend(b"JSON");
    out.extend(&text);
    if !bin.is_empty() {
        out.extend((bin.len() as u32).to_le_bytes());
        out.extend(b"BIN\0");
        out.extend(&bin);
    }
    out
}

/// One triangle (glTF frame) in a buffer: 3 positions, then 3 `u16` indices.
fn triangle_bin() -> Vec<u8> {
    let mut bin = Vec::new();
    for p in [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
        for c in p {
            bin.extend(c.to_le_bytes());
        }
    }
    for i in [0u16, 1, 2] {
        bin.extend(i.to_le_bytes());
    }
    bin
}

/// glTF JSON with the triangle of [`triangle_bin`] as mesh 0; `extra` goes into the root
/// object (scenes, nodes, extensions).
fn triangle_json(extra: &str, mode: u32) -> String {
    format!(
        r#"{{"asset":{{"version":"2.0","generator":"fremd"}},
        "buffers":[{{"byteLength":44}}],
        "bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":36}},
                       {{"buffer":0,"byteOffset":36,"byteLength":6}}],
        "accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3",
                        "min":[0,0,0],"max":[1,1,0]}},
                     {{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}}],
        "meshes":[{{"primitives":[{{"attributes":{{"POSITION":0}},"indices":1,"mode":{mode}}}]}}]
        {extra}}}"#
    )
}

/// The warnings `progress` has logged since the last call.
fn warnings(progress: &Progress) -> Vec<String> {
    progress
        .take_log()
        .into_iter()
        .filter_map(|l| match l {
            LogLine::Warning(w) => Some(w),
            LogLine::Info(_) => None,
        })
        .collect()
}

// ---------------------------------------------------------------------------------------
// glb

#[test]
fn glb_read_write_identical() {
    let dir = temp_dir("glb-identical");
    let mut mesh = small_mesh();
    mesh.pages.insert(1, MeshPage::default()); // empty: left out of the glb
    let (a, b) = (dir.join("a.glb"), dir.join("b.glb"));
    write_glb(&a, &mesh, &Progress::new()).unwrap();
    assert!(!dir.join("a.glb.tmp").exists());
    let read = read_mesh(&a, &Progress::new()).unwrap();
    // The empty page is not in the file; the others come back unchanged (positions are
    // representable in f32, the axis swap is exact).
    mesh.pages.remove(1);
    assert_eq!(read, mesh);
    assert_eq!(read.triangles(), 6);
    write_glb(&b, &read, &Progress::new()).unwrap();
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());

    let bytes = std::fs::read(&a).unwrap();
    let gltf = gltf::Gltf::from_slice(&bytes).unwrap();
    assert_eq!(gltf.as_json().asset.generator.as_deref(), Some(GENERATOR));
    // The ceiling has no texture, only its colour.
    let ceiling = gltf.meshes().next().unwrap().primitives().nth(2).unwrap();
    let pbr = ceiling.material().pbr_metallic_roughness();
    assert!(pbr.base_color_texture().is_none());
    let f = pbr.base_color_factor();
    assert_eq!(f[..3], [90.0 / 255.0, 120.0 / 255.0, 150.0 / 255.0]);

    // An empty mesh is a valid glb and reads back empty.
    let empty = dir.join("leer.glb");
    write_glb(&empty, &Mesh::default(), &Progress::new()).unwrap();
    gltf::Gltf::from_slice(&std::fs::read(&empty).unwrap()).unwrap();
    assert_eq!(
        read_mesh(&empty, &Progress::new()).unwrap(),
        Mesh::default()
    );
}

#[test]
fn glb_node_transform_applied() {
    let dir = temp_dir("glb-node");
    let bin = triangle_bin();
    // Parent moves by (0, 2, 0) in the glTF frame (2 m up), the child rotates 90° about
    // glTF +y and carries the mesh.
    let s = std::f32::consts::FRAC_1_SQRT_2;
    let json = triangle_json(
        &format!(
            r#","scene":0,"scenes":[{{"nodes":[0]}}],
            "nodes":[{{"translation":[0,2,0],"children":[1]}},
                     {{"rotation":[0,{s},0,{s}],"mesh":0}}]"#
        ),
        4,
    );
    let path = dir.join("fremd.glb");
    std::fs::write(&path, glb_bytes(&json, &bin)).unwrap();
    let mesh = read_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 1);
    let page = &mesh.pages[0];
    assert_eq!(page.indices, vec![[0, 1, 2]]);
    assert!(page.image.is_none());
    // glTF (1, 0, 0) turned about +y by 90° is (0, 0, −1), raised to (0, 2, −1); in
    // Driftless (x, −z, y) = (0, 1, 2). glTF (0, 1, 0) is (0, 3, 0) → (0, 0, 3).
    let want = [[0.0, 0.0, 2.0], [0.0, 1.0, 2.0], [0.0, 0.0, 3.0]];
    for (got, want) in page.positions.iter().zip(want) {
        for k in 0..3 {
            assert!((got[k] - want[k]).abs() < 1e-6, "{got:?} vs {want:?}");
        }
    }
    // Without texture coordinates and texture the page has no uvs and is plain white.
    assert!(page.uvs.is_empty());
    assert_eq!(page.color, [255; 3]);

    // A node used twice gives the mesh twice; without a scene the meshes are taken as they
    // are.
    let twice = triangle_json(
        r#","scene":0,"scenes":[{"nodes":[0,1]}],
            "nodes":[{"mesh":0},{"translation":[5,0,0],"mesh":0}]"#,
        4,
    );
    std::fs::write(&path, glb_bytes(&twice, &bin)).unwrap();
    let mesh = read_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 2);
    assert_eq!(mesh.pages[1].positions[1], [6.0, 0.0, 0.0]);
    let bare = triangle_json(r#","nodes":[{"translation":[0,9,0],"mesh":0}]"#, 4);
    std::fs::write(&path, glb_bytes(&bare, &bin)).unwrap();
    let mesh = read_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 1);
    assert_eq!(mesh.pages[0].positions[2], [0.0, 0.0, 1.0]);
}

/// The JSON (parsed) and the binary chunk of a glb.
fn glb_parts(bytes: &[u8]) -> (serde_json::Value, Vec<u8>) {
    let u32_at = |at: usize| u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as usize;
    let json_len = u32_at(12);
    let json = serde_json::from_slice(&bytes[20..20 + json_len]).unwrap();
    let bin_len = u32_at(20 + json_len);
    let bin = bytes[28 + json_len..28 + json_len + bin_len].to_vec();
    (json, bin)
}

/// A mesh used by many nodes: the image is held once for all its pages, and the corners and
/// triangles all instances would give are bounded by the size of the file.
#[test]
fn glb_instances_share_images_and_are_bounded() {
    let dir = temp_dir("glb-instances");
    // A 100 × 100 grid with a PNG.
    let side = 100usize;
    let mut page = MeshPage {
        image: Some(png(2, 2, &[50; 12])),
        color: [255; 3],
        ..Default::default()
    };
    for y in 0..side {
        for x in 0..side {
            page.positions.push([x as f64 * 0.01, y as f64 * 0.01, 0.0]);
            page.uvs
                .push([x as f32 / side as f32, y as f32 / side as f32]);
        }
    }
    for y in 0..side as u32 - 1 {
        for x in 0..side as u32 - 1 {
            let i = y * side as u32 + x;
            let s = side as u32;
            page.indices.push([i, i + 1, i + s]);
            page.indices.push([i + 1, i + s + 1, i + s]);
        }
    }
    let path = dir.join("viele.glb");
    write_glb(&path, &Mesh { pages: vec![page] }, &Progress::new()).unwrap();
    let (json, bin) = glb_parts(&std::fs::read(&path).unwrap());
    let instanced = |k: usize| {
        let mut json = json.clone();
        json["nodes"] = (0..k)
            .map(|i| serde_json::json!({"mesh": 0, "translation": [i as f64, 0.0, 0.0]}))
            .collect();
        json["scenes"] = serde_json::json!([{"nodes": (0..k).collect::<Vec<_>>()}]);
        json["scene"] = 0.into();
        glb_bytes(&json.to_string(), &bin)
    };

    std::fs::write(&path, instanced(3)).unwrap();
    let mesh = read_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 3);
    let bytes = |p: usize| mesh.pages[p].image.as_ref().unwrap().bytes.clone();
    assert!(
        std::sync::Arc::ptr_eq(&bytes(0), &bytes(2)),
        "one copy of the image"
    );
    let moved = mesh.pages[2].positions[1];
    assert!(
        (moved[0] - 2.01).abs() < 1e-6 && moved[1] == 0.0,
        "{moved:?}"
    );

    // 400 instances would decode to about 300 MB from a file of about 400 KB.
    let many = instanced(400);
    assert!(many.len() < 1 << 20);
    std::fs::write(&path, many).unwrap();
    let msg = format_msg(read_mesh(&path, &Progress::new()).unwrap_err());
    assert!(
        msg.starts_with("glTF ergibt mehr als 256 MB Flächendaten"),
        "{msg}"
    );
}

#[test]
fn glb_png_texture() {
    let dir = temp_dir("glb-png");
    let page = quad(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ],
        Some(png(4, 4, &halves(4, 4, [1, 2, 3], [4, 5, 6]))),
        [255; 3],
    );
    let mesh = Mesh { pages: vec![page] };
    let path = dir.join("png.glb");
    write_glb(&path, &mesh, &Progress::new()).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let gltf = gltf::Gltf::from_slice(&bytes).unwrap();
    let img = gltf.images().next().unwrap();
    let gltf::image::Source::View { mime_type, .. } = img.source() else {
        panic!("image not embedded");
    };
    assert_eq!(mime_type, "image/png");
    let read = read_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(read.pages[0].image, mesh.pages[0].image);
    assert_eq!(read, mesh);
}

#[test]
fn glb_multiple_scenes_refused() {
    let dir = temp_dir("glb-scenes");
    let path = dir.join("zwei.glb");
    let json = triangle_json(
        r#","scene":0,"scenes":[{"nodes":[0]},{"nodes":[0]}],"nodes":[{"mesh":0}]"#,
        4,
    );
    std::fs::write(&path, glb_bytes(&json, &triangle_bin())).unwrap();
    let e = read_mesh(&path, &Progress::new()).unwrap_err();
    assert_eq!(
        format_msg(e),
        "glTF mit mehreren Szenen wird nicht unterstützt"
    );
}

#[test]
fn glb_draco_refused() {
    let dir = temp_dir("glb-draco");
    let path = dir.join("draco.glb");
    for (ext, want) in [
        (
            "KHR_draco_mesh_compression",
            "glTF mit Draco-Kompression wird nicht unterstützt",
        ),
        (
            "EXT_meshopt_compression",
            "glTF mit meshopt-Kompression wird nicht unterstützt",
        ),
    ] {
        let json = triangle_json(
            &format!(r#","extensionsUsed":["{ext}"],"extensionsRequired":["{ext}"]"#),
            4,
        );
        std::fs::write(&path, glb_bytes(&json, &triangle_bin())).unwrap();
        let e = read_mesh(&path, &Progress::new()).unwrap_err();
        assert_eq!(format_msg(e), want);
    }
}

#[test]
fn glb_foreign_oddities_are_clear_errors() {
    let dir = temp_dir("glb-odd");
    let path = dir.join("odd.glb");
    let check = |bytes: Vec<u8>, part: &str| {
        std::fs::write(&path, bytes).unwrap();
        let msg = format_msg(read_mesh(&path, &Progress::new()).unwrap_err());
        assert!(msg.contains(part), "{msg:?} lacks {part:?}");
    };
    // Lines instead of triangles.
    check(
        glb_bytes(&triangle_json("", 1), &triangle_bin()),
        "statt Dreiecken wird nicht unterstützt",
    );
    // Data in a file of its own.
    let external = triangle_json("", 4).replace(
        r#""buffers":[{"byteLength":44}]"#,
        r#""buffers":[{"byteLength":44,"uri":"daten.bin"}]"#,
    );
    check(
        glb_bytes(&external, &[]),
        "Daten liegen in einer eigenen Datei (daten.bin)",
    );
    // An image as a file of its own.
    let image_uri = triangle_json(
        r#","images":[{"uri":"bild.png"}],"textures":[{"source":0}],
           "materials":[{"pbrMetallicRoughness":{"baseColorTexture":{"index":0}}}]"#,
        4,
    )
    .replace(r#""mode":4"#, r#""mode":4,"material":0"#);
    check(
        glb_bytes(&image_uri, &triangle_bin()),
        "Textur als eigene Datei wird nicht unterstützt",
    );
    // Not a glb at all, and a broken one.
    check(b"kein glTF".to_vec(), "keine gültige glb-Datei");
    let mut cut = glb_bytes(&triangle_json("", 4), &triangle_bin());
    cut.truncate(cut.len() - 10);
    check(cut, "keine gültige glb-Datei");
    // An index out of range.
    let mut bin = triangle_bin();
    let n = bin.len();
    bin[n - 2..].copy_from_slice(&7u16.to_le_bytes());
    check(
        glb_bytes(&triangle_json("", 4), &bin),
        "unstimmigen Ecken oder Dreiecken",
    );
    // A node that is its own child.
    let cycle = triangle_json(
        r#","scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"children":[0],"mesh":0}]"#,
        4,
    );
    std::fs::write(&path, glb_bytes(&cycle, &triangle_bin())).unwrap();
    let msg = format_msg(read_mesh(&path, &Progress::new()).unwrap_err());
    assert!(
        msg.contains("Kreis") || msg.starts_with("keine gültige glb-Datei"),
        "{msg:?}"
    );
    // A missing file is an I/O error naming it.
    let e = read_mesh(&dir.join("fehlt.glb"), &Progress::new()).unwrap_err();
    assert!(matches!(e, ExportError::Io { .. }), "{e:?}");
    assert!(e.to_string().contains("fehlt.glb"));
    // An unknown extension.
    let e = read_mesh(&dir.join("x.stl"), &Progress::new()).unwrap_err();
    assert!(format_msg(e).contains(".glb"));
}

// ---------------------------------------------------------------------------------------
// OBJ

#[test]
fn obj_round_trip() {
    let dir = temp_dir("obj-round").join("Büro Scan");
    std::fs::create_dir_all(&dir).unwrap();
    let mut mesh = small_mesh();
    // A coordinate f32 cannot hold is written in full.
    mesh.pages[2].positions[3] = [500_000.123_456_789, 5_400_000.987_654_321, 2.5];
    let path = dir.join("Prüfung ä.obj");
    let files = obj_files(&path, &mesh);
    let names: Vec<String> = files
        .iter()
        .map(|p| p.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(
        names,
        [
            "Prüfung ä.obj",
            "Prüfung ä.mtl",
            "Prüfung ä_seite_00.jpg",
            "Prüfung ä_seite_01.png",
        ]
    );
    assert!(files.iter().all(|f| f.parent() == Some(dir.as_path())));
    write_obj(&path, &mesh, &Progress::new()).unwrap();
    let mut want = names.clone();
    want.sort();
    assert_eq!(entries(&dir), want, "no .tmp, nothing else");

    let mtl = std::fs::read_to_string(dir.join("Prüfung ä.mtl")).unwrap();
    assert!(mtl.contains("newmtl seite_02\n"));
    assert!(mtl.contains("map_Kd Prüfung ä_seite_01.png\n"));
    let obj = std::fs::read_to_string(&path).unwrap();
    assert!(obj.contains("v 500000.123456789 5400000.987654321 2.5\n"));
    assert!(obj.contains("v 4 3 0\n"));

    let progress = Progress::new();
    let read = read_mesh(&path, &progress).unwrap();
    assert!(warnings(&progress).is_empty(), "{:?}", warnings(&progress));
    assert_eq!(read.pages.len(), 3);
    for (got, want) in read.pages.iter().zip(&mesh.pages) {
        assert_eq!(got.image, want.image, "page image by name, bytes unchanged");
        assert_eq!(got.color, want.color);
        let (a, b) = (triangles(got), triangles(want));
        assert_eq!(a.len(), b.len());
        for ((pa, ua), (pb, ub)) in a.iter().zip(&b) {
            assert_eq!(pa, pb);
            for k in 0..3 {
                for c in 0..2 {
                    assert!((ua[k][c] - ub[k][c]).abs() < 1e-6, "{ua:?} vs {ub:?}");
                }
            }
        }
    }

    // Written twice, the same bytes.
    let again = temp_dir("obj-round-again");
    write_obj(&again.join("Prüfung ä.obj"), &mesh, &Progress::new()).unwrap();
    for name in &names {
        assert_eq!(
            std::fs::read(dir.join(name)).unwrap(),
            std::fs::read(again.join(name)).unwrap(),
            "{name}"
        );
    }
}

#[test]
fn obj_failed_write_leaves_no_tmp() {
    let dir = temp_dir("obj-fail");
    let mesh = small_mesh();
    // The .mtl's name is taken by a folder: renaming onto it fails after the images were
    // written as .tmp.
    let path = dir.join("x.obj");
    std::fs::create_dir_all(dir.join("x.mtl")).unwrap();
    std::fs::write(&path, b"alt").unwrap();
    assert!(write_obj(&path, &mesh, &Progress::new()).is_err());
    assert!(
        !entries(&dir).iter().any(|e| e.ends_with(".tmp")),
        "{:?}",
        entries(&dir)
    );
    assert_eq!(std::fs::read(&path).unwrap(), b"alt", "the .obj comes last");
    // An index past the corners is refused before anything is written.
    let mut bad = small_mesh();
    bad.pages[0].indices[0][2] = 9;
    let other = dir.join("y.obj");
    assert!(write_obj(&other, &bad, &Progress::new()).is_err());
    assert!(write_glb(&dir.join("y.glb"), &bad, &Progress::new()).is_err());
    assert!(!entries(&dir).iter().any(|e| e.starts_with('y')));
}

/// „Abbrechen“ while a glb or OBJ is written: the old files stay, no `.tmp` is left, and the
/// writer says it was cancelled (it no longer reports success after a cancel).
#[test]
fn mesh_write_cancel_keeps_old_files() {
    let dir = temp_dir("mesh-cancel");
    let mesh = small_mesh();
    let obj = dir.join("x.obj");
    let glb = dir.join("x.glb");
    std::fs::write(&obj, b"alt").unwrap();
    std::fs::write(&glb, b"alt").unwrap();
    let progress = Progress::new();
    progress.cancel();
    assert!(matches!(
        write_obj(&obj, &mesh, &progress),
        Err(ExportError::Cancelled)
    ));
    assert!(matches!(
        write_glb(&glb, &mesh, &progress),
        Err(ExportError::Cancelled)
    ));
    assert_eq!(entries(&dir), ["x.glb", "x.obj"]);
    assert_eq!(std::fs::read(&obj).unwrap(), b"alt");
    assert_eq!(std::fs::read(&glb).unwrap(), b"alt");
    std::fs::remove_dir_all(dir).unwrap();
}

/// „Abbrechen“ while the pages are written (after the first page, and after the last one,
/// just before the rename): the writer says it was cancelled, no `.tmp` is left and every old
/// file – the glb, or the OBJ with its `.mtl` and page images – keeps its bytes.
#[test]
fn mesh_cancel_while_writing_keeps_old_files() {
    let mesh = small_mesh();
    let old = [
        "x.glb",
        "x.mtl",
        "x.obj",
        "x_seite_00.jpg",
        "x_seite_01.png",
    ];
    for glb in [true, false] {
        for at in [1, 3] {
            let dir = temp_dir(&format!("mesh-cancel-mid-{glb}-{at}"));
            for name in old {
                std::fs::write(dir.join(name), b"alt").unwrap();
            }
            let progress = Progress::new();
            let handle = progress.clone();
            let progress = progress.with_observer(move |s| {
                if s.phase == PHASE_WRITE && s.done == at {
                    handle.cancel();
                }
            });
            let result = if glb {
                write_glb(&dir.join("x.glb"), &mesh, &progress)
            } else {
                write_obj(&dir.join("x.obj"), &mesh, &progress)
            };
            assert!(
                matches!(result, Err(ExportError::Cancelled)),
                "glb {glb}, page {at}: {result:?}"
            );
            assert_eq!(entries(&dir), old, "glb {glb}, page {at}");
            for name in old {
                assert_eq!(std::fs::read(dir.join(name)).unwrap(), b"alt", "{name}");
            }
            std::fs::remove_dir_all(dir).unwrap();
        }
    }
}

#[test]
fn obj_foreign_quads_negative_indices_missing_texture() {
    let dir = temp_dir("obj-foreign");
    let obj = "# fremd\r\n\
        mtllib fremd.mtl\r\n\
        o Wand\n\
        v 0 0 0\n\
        v 1 0 0\n\
        v 1 1 0\n\
        v 0 1 0\n\
        vt 0 0\n\
        vt 1 0\n\
        vt 1 1\n\
        vt 0 1\n\
        vn 0 0 1\n\
        g wand\n\
        s off\n\
        usemtl wand\n\
        f -4/-4/-1 -3/-3/-1 -2/-2/-1 -1/-1/-1\n\
        usemtl boden\n\
        v 0 0 -1\n\
        f 1//1 2//1 5//1\n\
        usemtl wand\n\
        f\t1/1\t3/3\t4/4\n";
    std::fs::write(dir.join("fremd.obj"), obj).unwrap();
    std::fs::write(
        dir.join("fremd.mtl"),
        "newmtl wand\nKd 0.5 0.5 0.5\nmap_Kd -s 1 1 1 bilder\\fehlt.jpg\n\
         newmtl boden\nKd 1 0 0\n",
    )
    .unwrap();
    let progress = Progress::new();
    let mesh = read_mesh(&dir.join("fremd.obj"), &progress).unwrap();
    assert_eq!(warnings(&progress), ["Textur fehlt: bilder\\fehlt.jpg"]);
    assert_eq!(mesh.pages.len(), 2, "one page per material");
    let wand = &mesh.pages[0];
    assert_eq!(wand.color, [128; 3], "missing texture: gray");
    assert!(wand.image.is_none());
    // The quad is a fan of two triangles, then one more triangle of the same material.
    let tris = triangles(wand);
    assert_eq!(tris.len(), 3);
    assert_eq!(
        tris[0].0,
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]
    );
    assert_eq!(
        tris[1].0,
        [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
    );
    // OBJ v up → Driftless v down.
    assert_eq!(tris[0].1, [[0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]);
    assert_eq!(tris[2].0[2], [0.0, 1.0, 0.0]);
    assert_eq!(wand.positions.len(), 4, "corners shared within a page");
    let boden = &mesh.pages[1];
    assert_eq!(boden.color, [255, 0, 0]);
    assert!(boden.uvs.is_empty());
    assert_eq!(boden.positions[2], [0.0, 0.0, -1.0]);

    // Without the .mtl every page is gray, with one warning; a material-less file has one
    // gray page.
    std::fs::remove_file(dir.join("fremd.mtl")).unwrap();
    let progress = Progress::new();
    let mesh = read_mesh(&dir.join("fremd.obj"), &progress).unwrap();
    assert_eq!(warnings(&progress), ["Textur fehlt: fremd.mtl"]);
    assert!(mesh.pages.iter().all(|p| p.color == [128; 3]));
    std::fs::write(
        dir.join("nackt.obj"),
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .unwrap();
    let mesh = read_mesh(&dir.join("nackt.obj"), &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 1);
    assert_eq!(mesh.pages[0].color, [128; 3]);
    assert_eq!(mesh.triangles(), 1);

    // A PNG texture next to the .mtl is read as PNG, with spaces in the names.
    let image = png(2, 2, &[9; 12]);
    std::fs::write(dir.join("mit bild.png"), &image.bytes).unwrap();
    std::fs::write(
        dir.join("mit bild.mtl"),
        "newmtl a b\nmap_Kd mit bild.png\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("mit bild.obj"),
        "mtllib mit bild.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\n\
         usemtl a b\nf 1/1 2/2 3/3\n",
    )
    .unwrap();
    let mesh = read_mesh(&dir.join("mit bild.obj"), &Progress::new()).unwrap();
    assert_eq!(mesh.pages[0].image, Some(image));
}

/// Names in `mtllib` and `map_Kd` of a foreign OBJ never reach outside the OBJ's folder:
/// absolute names, `..`, Windows drive and share names (a share would be contacted) and links
/// that point outside are not read, with their own warning.
#[test]
fn obj_names_stay_inside_the_folder() {
    let dir = temp_dir("obj-escape");
    let inner = dir.join("innen");
    std::fs::create_dir_all(inner.join("bilder")).unwrap();
    // Outside the OBJ's folder: a green material and its image.
    let secret = png(2, 2, &[7; 12]);
    std::fs::write(dir.join("geheim.png"), &secret.bytes).unwrap();
    std::fs::write(dir.join("geheim.mtl"), "newmtl a\nKd 0 1 0\n").unwrap();
    let face = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl a\nf 1/1 2/2 3/3\n";
    let outside_mtl = dir.join("geheim.mtl").display().to_string();
    for lib in [
        "../geheim.mtl",
        "..\\geheim.mtl",
        outside_mtl.as_str(),
        "/etc/passwd",
    ] {
        std::fs::write(inner.join("x.obj"), format!("mtllib {lib}\n{face}")).unwrap();
        let progress = Progress::new();
        let mesh = read_mesh(&inner.join("x.obj"), &progress).unwrap();
        assert_eq!(
            warnings(&progress),
            [format!(
                "Materialdatei außerhalb des Ordners wird nicht gelesen: {lib}"
            )],
            "{lib}"
        );
        assert_eq!(mesh.pages[0].color, [128; 3], "{lib}");
    }

    // A material inside, its image named outside.
    let outside_png = dir.join("geheim.png").display().to_string();
    #[cfg_attr(not(unix), allow(unused_mut))]
    let mut names = vec![
        "../geheim.png".to_string(),
        "bilder/../../geheim.png".to_string(),
        "\\\\server\\share\\x.png".to_string(),
        "//server/share/x.png".to_string(),
        "C:\\geheim.png".to_string(),
        "C:geheim.png".to_string(),
        outside_png,
    ];
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(dir.join("geheim.png"), inner.join("bilder/link.png")).unwrap();
        names.push("bilder/link.png".to_string());
    }
    for name in &names {
        std::fs::write(inner.join("x.mtl"), format!("newmtl a\nmap_Kd {name}\n")).unwrap();
        std::fs::write(inner.join("x.obj"), format!("mtllib x.mtl\n{face}")).unwrap();
        let progress = Progress::new();
        let mesh = read_mesh(&inner.join("x.obj"), &progress).unwrap();
        assert_eq!(
            warnings(&progress),
            [format!(
                "Textur außerhalb des Ordners wird nicht gelesen: {name}"
            )],
            "{name}"
        );
        assert!(mesh.pages[0].image.is_none(), "{name}");
    }
    // Inside but not there: missing, as before.
    std::fs::write(inner.join("x.mtl"), "newmtl a\nmap_Kd bilder/fehlt.png\n").unwrap();
    let progress = Progress::new();
    read_mesh(&inner.join("x.obj"), &progress).unwrap();
    assert_eq!(warnings(&progress), ["Textur fehlt: bilder/fehlt.png"]);

    // Inside the folder (also below it, also with `\\`) it is read.
    std::fs::write(inner.join("bilder/gut.png"), &secret.bytes).unwrap();
    for name in ["bilder/gut.png", "bilder\\gut.png", "./bilder/gut.png"] {
        std::fs::write(inner.join("x.mtl"), format!("newmtl a\nmap_Kd {name}\n")).unwrap();
        let progress = Progress::new();
        let mesh = read_mesh(&inner.join("x.obj"), &progress).unwrap();
        assert!(warnings(&progress).is_empty(), "{name}");
        assert_eq!(mesh.pages[0].image.as_ref(), Some(&secret), "{name}");
    }
}

#[test]
fn obj_broken_lines_are_clear_errors() {
    let dir = temp_dir("obj-broken");
    let path = dir.join("kaputt.obj");
    for (text, part) in [
        ("v 0 0\n", "Zeile 1"),
        ("v 0 0 x\n", "Zeile 1"),
        ("v 0 0 0\nf 1 2 3\n", "Zeile 2"),
        ("v 0 0 0\nv 1 0 0\nf 1 2\n", "Zeile 3"),
        ("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/4 2/4 3/4\n", "Zeile 4"),
        ("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n", "Zeile 4"),
    ] {
        std::fs::write(&path, text).unwrap();
        let msg = format_msg(read_mesh(&path, &Progress::new()).unwrap_err());
        assert!(msg.contains(part), "{text:?}: {msg:?}");
        assert!(msg.starts_with("OBJ nicht lesbar"), "{msg:?}");
    }
}

// ---------------------------------------------------------------------------------------
// PLY mesh

/// A binary little-endian PLY mesh as [`write_ply_mesh`] writes it: header, corners
/// (xyz, rgb), faces.
type PlyMesh = (String, Vec<([f64; 3], [u8; 3])>, Vec<[i32; 3]>);

fn parse_ply_mesh(bytes: &[u8]) -> PlyMesh {
    let end = b"end_header\n";
    let n = bytes.windows(end.len()).position(|w| w == end).unwrap() + end.len();
    let header = std::str::from_utf8(&bytes[..n]).unwrap().to_string();
    let count = |name: &str| -> usize {
        header
            .lines()
            .find_map(|l| l.strip_prefix(&format!("element {name} ")))
            .unwrap()
            .parse()
            .unwrap()
    };
    let double = header.contains("property double x");
    let (nv, nf) = (count("vertex"), count("face"));
    let mut at = n;
    let mut take = |k: usize| {
        let s = &bytes[at..at + k];
        at += k;
        s
    };
    let mut vertices = Vec::new();
    for _ in 0..nv {
        let p = [0, 1, 2].map(|_| {
            if double {
                f64::from_le_bytes(take(8).try_into().unwrap())
            } else {
                f64::from(f32::from_le_bytes(take(4).try_into().unwrap()))
            }
        });
        let c = take(3);
        vertices.push((p, [c[0], c[1], c[2]]));
    }
    let mut faces = Vec::new();
    for _ in 0..nf {
        assert_eq!(take(1), [3]);
        faces.push([0, 1, 2].map(|_| i32::from_le_bytes(take(4).try_into().unwrap())));
    }
    assert_eq!(at, bytes.len(), "nothing after the faces");
    (header, vertices, faces)
}

fn near(a: [u8; 3], b: [u8; 3], tol: u8) -> bool {
    (0..3).all(|k| a[k].abs_diff(b[k]) <= tol)
}

#[test]
fn ply_mesh_vertex_colors_from_texture() {
    let dir = temp_dir("ply-colors");
    let (red, blue) = ([255, 0, 0], [0, 0, 255]);
    // Left half red, right half blue; the left corners (u = 0) red, the right ones blue.
    for image in [
        png(16, 16, &halves(16, 16, red, blue)),
        jpeg(16, 16, &halves(16, 16, red, blue)),
    ] {
        let page = quad(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            Some(image),
            [255; 3],
        );
        let path = dir.join("farbe.ply");
        write_ply_mesh(&path, &Mesh { pages: vec![page] }, &Progress::new()).unwrap();
        let (_, vertices, _) = parse_ply_mesh(&std::fs::read(&path).unwrap());
        assert_eq!(vertices.len(), 4);
        for (p, c) in vertices {
            let want = if p[0] == 0.0 { red } else { blue };
            assert!(near(c, want, 2), "{p:?}: {c:?}");
        }
    }

    // A corner shared by a red page and a blue page gets the mean of both.
    let one = |color: [u8; 3], x: f64| {
        let mut page = quad(
            [
                [x, 0.0, 0.0],
                [x + 1.0, 0.0, 0.0],
                [x + 1.0, 1.0, 0.0],
                [x, 1.0, 0.0],
            ],
            Some(png(2, 2, &[color, color, color, color].concat())),
            [255; 3],
        );
        page.uvs = vec![[0.25, 0.25]; 4];
        page
    };
    let mesh = Mesh {
        pages: vec![one(red, 0.0), one(blue, 1.0)],
    };
    let path = dir.join("mittel.ply");
    write_ply_mesh(&path, &mesh, &Progress::new()).unwrap();
    let (_, vertices, _) = parse_ply_mesh(&std::fs::read(&path).unwrap());
    assert_eq!(vertices.len(), 6);
    for (p, c) in vertices {
        let want = match p[0] {
            0.0 => red,
            1.0 => [128, 0, 128],
            _ => blue,
        };
        assert!(near(c, want, 1), "{p:?}: {c:?}");
    }
}

#[test]
fn ply_mesh_untextured_uses_page_color() {
    let dir = temp_dir("ply-untextured");
    let mut mesh = small_mesh();
    for page in &mut mesh.pages {
        page.image = None;
        page.color = [10, 20, 30];
    }
    mesh.pages[0].uvs.clear();
    let path = dir.join("grund.ply");
    write_ply_mesh(&path, &mesh, &Progress::new()).unwrap();
    let (_, vertices, faces) = parse_ply_mesh(&std::fs::read(&path).unwrap());
    assert!(vertices.iter().all(|(_, c)| *c == [10, 20, 30]));
    assert_eq!(faces.len(), 6);
}

#[test]
fn ply_mesh_header_counts() {
    let dir = temp_dir("ply-header");
    // Floor and wall share the edge (0,0,0)–(0,3,0); the ceiling shares (0,0,2.5),
    // (0,3,2.5) with the wall: 4 + 2 + 2 = 8 corners after welding, 6 faces. An empty page
    // and a corner no triangle uses add nothing.
    let mut mesh = small_mesh();
    mesh.pages.push(MeshPage::default());
    mesh.pages[2].positions.push([9.0, 9.0, 9.0]);
    mesh.pages[2].uvs.push([0.0, 0.0]);
    let path = dir.join("netz.ply");
    let progress = Progress::new();
    write_ply_mesh(&path, &mesh, &progress).unwrap();
    let bytes = std::fs::read(&path).unwrap();
    let (header, vertices, faces) = parse_ply_mesh(&bytes);
    assert_eq!(
        header,
        "ply\nformat binary_little_endian 1.0\nelement vertex 8\n\
         property float x\nproperty float y\nproperty float z\n\
         property uchar red\nproperty uchar green\nproperty uchar blue\n\
         element face 6\nproperty list uchar int vertex_indices\nend_header\n"
    );
    assert_eq!(bytes.len(), header.len() + 8 * 15 + 6 * 13);
    assert_eq!(faces.len(), mesh.triangles());
    // Every face refers to the welded corners at the original positions.
    let all: Vec<[[f64; 3]; 3]> = mesh
        .pages
        .iter()
        .flat_map(|p| triangles(p).into_iter().map(|t| t.0))
        .collect();
    for (f, want) in faces.iter().zip(&all) {
        assert_eq!(f.map(|i| vertices[i as usize].0), *want);
    }
    let snap = progress.snapshot();
    assert_eq!(snap.phase, PHASE_WRITE);
    assert_eq!(snap.total, Some(4), "one step per page");

    // Far from the origin: double; empty: a valid file without corners.
    let mut far = small_mesh();
    far.pages[0].positions[0] = [500_000.0, 5_400_000.0, 312.0];
    write_ply_mesh(&path, &far, &Progress::new()).unwrap();
    let (header, vertices, _) = parse_ply_mesh(&std::fs::read(&path).unwrap());
    assert!(header.contains("property double x\n"));
    assert!(vertices
        .iter()
        .any(|v| v.0 == [500_000.0, 5_400_000.0, 312.0]));
    write_ply_mesh(&path, &Mesh::default(), &Progress::new()).unwrap();
    let (header, vertices, faces) = parse_ply_mesh(&std::fs::read(&path).unwrap());
    assert!(header.contains("element vertex 0\n") && header.contains("element face 0\n"));
    assert!(vertices.is_empty() && faces.is_empty());

    // A cancel leaves the old file and no .tmp.
    let before = std::fs::read(&path).unwrap();
    let cancelled = Progress::new();
    cancelled.cancel();
    let e = write_ply_mesh(&path, &mesh, &cancelled).unwrap_err();
    assert!(matches!(e, ExportError::Cancelled));
    assert_eq!(std::fs::read(&path).unwrap(), before);
    assert_eq!(entries(&dir), ["netz.ply"]);
}

#[test]
fn mesh_formats_follow_the_extension() {
    assert_eq!(
        MeshFormat::from_path(Path::new("a.GLB")),
        Some(MeshFormat::Glb)
    );
    assert_eq!(
        MeshFormat::from_path(Path::new("a.obj")),
        Some(MeshFormat::Obj)
    );
    assert_eq!(
        MeshFormat::from_path(Path::new("a.ply")),
        Some(MeshFormat::PlyMesh)
    );
    assert_eq!(MeshFormat::from_path(Path::new("a.stl")), None);
    assert_eq!(MeshFormat::Glb.extension(), "glb");
    assert_eq!(MeshFormat::Obj.extension(), "obj");
    assert_eq!(MeshFormat::PlyMesh.extension(), "ply");
    assert_eq!(small_mesh().triangles(), 6);
}
