//! The textured surface: drawn offscreen into 64 × 64 pixels and read back (without a GPU
//! adapter those tests are skipped), and the glb written by `driftless-map`, OBJ and foreign
//! glb read back.

use driftless_app::camera::{Camera, CameraMode};
use driftless_app::cloud::PointSet;
use driftless_app::lod::{build_octree, Octree};
use driftless_app::render::mesh::{MeshRenderer, MeshVertex};
use driftless_app::render::{
    headless_device, ColorMode, FrameParams, PointRenderer, PointSize, RenderTarget, ShowMode,
    NO_CUT,
};
use driftless_app::surface::{load_mesh, LoadedSurface};
use driftless_io::images::RgbImage;
use driftless_io::progress::Progress;
use driftless_io::testdir::TestDir;
use driftless_map::surface::{encode_pages, write_glb, TexturePage, TexturedMesh};
use egui_wgpu::wgpu;
use image::RgbaImage;
use std::fs;

const SIZE: u32 = 64;
const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const BACKGROUND: [f32; 4] = [0.1, 0.1, 0.1, 1.0];
const RED: [u8; 3] = [255, 0, 0];
const BLUE: [u8; 3] = [0, 0, 255];
const GREEN: [u8; 3] = [0, 255, 0];

struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

fn gpu() -> Option<Gpu> {
    let Some((device, queue, info)) = headless_device() else {
        skipped("kein GPU-Adapter");
        return None;
    };
    eprintln!("GPU-Adapter: {} ({:?})", info.name, info.backend);
    Some(Gpu { device, queue })
}

fn camera(eye: [f32; 3], yaw: f32, pitch: f32) -> Camera {
    Camera {
        mode: CameraMode::Fly,
        eye: eye.into(),
        yaw,
        pitch,
        ..Camera::default()
    }
}

fn frame(cam: &Camera, show: ShowMode, z_range: [f32; 2]) -> FrameParams {
    FrameParams {
        view_proj: cam.view_proj(1.0).into(),
        viewport_px: [SIZE as f32; 2],
        point_px: 8.0,
        marker_px: 16.0,
        line_scale: 1.0,
        color_mode: ColorMode::Rgb,
        round: false,
        background: BACKGROUND,
        quality_known: true,
        z_range,
        show,
        point_size: PointSize::Pixel,
        point_m: 0.05,
        point_px_range: [1.0, 1000.0],
    }
}

/// A rectangle of one colour: `corner` and the two edges `a` and `b` from it.
fn quad(
    corner: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    rgb: [u8; 3],
) -> (Vec<MeshVertex>, Vec<u32>, RgbaImage) {
    let at = |s: f32, t: f32| [0, 1, 2].map(|k| corner[k] + s * a[k] + t * b[k]);
    let vertices = vec![
        MeshVertex {
            pos: at(0.0, 0.0),
            uv: [0.0, 0.0],
        },
        MeshVertex {
            pos: at(1.0, 0.0),
            uv: [1.0, 0.0],
        },
        MeshVertex {
            pos: at(1.0, 1.0),
            uv: [1.0, 1.0],
        },
        MeshVertex {
            pos: at(0.0, 1.0),
            uv: [0.0, 1.0],
        },
    ];
    let image = RgbaImage::from_pixel(8, 8, image::Rgba([rgb[0], rgb[1], rgb[2], 255]));
    (vertices, vec![0, 1, 2, 0, 2, 3], image)
}

struct Image(Vec<[u8; 4]>);

impl Image {
    fn centre(&self) -> [u8; 4] {
        self.0[(SIZE / 2 * SIZE + SIZE / 2) as usize]
    }
}

fn near(pixel: [u8; 4], rgb: [u8; 3], tol: u8) -> bool {
    (0..3).all(|k| pixel[k].abs_diff(rgb[k]) <= tol)
}

/// Draws `surface` (and the cloud `tree`) with `frame` and reads the image back.
fn render(gpu: &Gpu, surface: &LoadedSurface, tree: &Octree, frame: &FrameParams) -> Image {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let target = RenderTarget::new(device, [SIZE; 2], FORMAT);
    let mut points = PointRenderer::new(device, queue, FORMAT);
    points.upload_cloud(device, queue, tree);
    let mut mesh = MeshRenderer::new(device, FORMAT);
    mesh.upload(device, queue, surface);
    let drawn: Vec<u32> = (0..tree.nodes.len() as u32).collect();
    points.render_with(device, queue, &target, frame, tree, &drawn, Some(&mesh));
    read_back(gpu, &target)
}

fn read_back(gpu: &Gpu, target: &RenderTarget) -> Image {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let row = SIZE * 4;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Rücklesen"),
        size: u64::from(row * SIZE),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &target.color,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row),
                rows_per_image: Some(SIZE),
            },
        },
        wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
    );
    queue.submit([encoder.finish()]);
    let slice = buffer.slice(..);
    slice.map_async(wgpu::MapMode::Read, |r| r.expect("Puffer lesbar"));
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    let bytes = slice.get_mapped_range().expect("Puffer abgebildet");
    Image(bytemuck::cast_slice::<u8, [u8; 4]>(&bytes).to_vec())
}

fn no_cloud() -> Octree {
    Octree::default()
}

#[test]
fn mesh_renders_texture() {
    let Some(gpu) = gpu() else { return };
    // A red wall 2 m in front of the camera (looking along +x), 4 m × 4 m.
    let surface = LoadedSurface {
        origin: [0.0; 3],
        pages: vec![quad(
            [2.0, -2.0, -2.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, 4.0],
            RED,
        )],
        ..LoadedSurface::default()
    };
    let cam = camera([0.0; 3], 0.0, 0.0);
    let c = render(
        &gpu,
        &surface,
        &no_cloud(),
        &frame(&cam, ShowMode::Surface, NO_CUT),
    )
    .centre();
    eprintln!("Mitte {c:?}");
    assert!(near(c, RED, 2), "centre {c:?}");
    // „Punktwolke“ alone does not draw the surface.
    let c = render(
        &gpu,
        &surface,
        &no_cloud(),
        &frame(&cam, ShowMode::Cloud, NO_CUT),
    )
    .centre();
    assert!(near(c, [26; 3], 1), "centre {c:?}");
}

#[test]
fn z_cut_hides_ceiling() {
    let Some(gpu) = gpu() else { return };
    // Looking down from 5 m: a red ceiling at 3 m above a blue floor at 0 m.
    let big = |z, rgb| quad([-20.0, -20.0, z], [40.0, 0.0, 0.0], [0.0, 40.0, 0.0], rgb);
    let surface = LoadedSurface {
        origin: [0.0; 3],
        pages: vec![big(0.0, BLUE), big(3.0, RED)],
        ..LoadedSurface::default()
    };
    let cam = camera([0.0, 0.0, 5.0], 0.0, -1.4);
    let shown = |z_range| {
        render(
            &gpu,
            &surface,
            &no_cloud(),
            &frame(&cam, ShowMode::Surface, z_range),
        )
        .centre()
    };
    let c = shown(NO_CUT);
    assert!(near(c, RED, 2), "uncut: {c:?}");
    let c = shown([-1.0, 2.5]);
    assert!(near(c, BLUE, 2), "ceiling cut away: {c:?}");
}

#[test]
fn surface_and_points_share_depth() {
    let Some(gpu) = gpu() else { return };
    // A red wall at 3 m; a green point at 2 m in front of it, a blue one at 4 m behind it.
    let surface = LoadedSurface {
        origin: [0.0; 3],
        pages: vec![quad(
            [3.0, -2.0, -2.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, 4.0],
            RED,
        )],
        ..LoadedSurface::default()
    };
    let cloud = |x: f32, rgb| {
        let mut p = PointSet::from_xyz([0.0; 3], vec![[x, 0.0, 0.0]]);
        p.rgb = Some(vec![rgb]);
        build_octree(&p, &Progress::new()).unwrap()
    };
    let cam = camera([0.0; 3], 0.0, 0.0);
    let both = frame(&cam, ShowMode::Both, NO_CUT);
    let c = render(&gpu, &surface, &cloud(2.0, GREEN), &both).centre();
    assert!(near(c, GREEN, 2), "point in front: {c:?}");
    let c = render(&gpu, &surface, &cloud(4.0, BLUE), &both).centre();
    assert!(near(c, RED, 2), "point behind: {c:?}");
    // only the points: the one behind the wall shows
    let c = render(
        &gpu,
        &surface,
        &cloud(4.0, BLUE),
        &frame(&cam, ShowMode::Cloud, NO_CUT),
    );
    assert!(near(c.centre(), BLUE, 2), "{:?}", c.centre());
}

fn temp_dir(name: &str) -> TestDir {
    TestDir::new(format!("driftless-app-mesh-{}-{name}", std::process::id()))
}

/// A page of `n × n` texels in one colour.
fn page_image(n: u32, rgb: [u8; 3]) -> RgbImage {
    RgbImage {
        width: n,
        height: n,
        data: rgb.repeat((n * n) as usize),
    }
}

#[test]
fn load_mesh_matches_written_glb() {
    // Far from 0 like real coordinates, so that the origin matters.
    let base = [1000.25f32, -2000.5, 7.0];
    let at = |x: f32, y: f32, z: f32| [base[0] + x, base[1] + y, base[2] + z];
    let mut tex = TexturedMesh::default();
    tex.pages = vec![
        // floor: two triangles
        TexturePage {
            positions: vec![
                at(0.0, 0.0, 0.0),
                at(4.0, 0.0, 0.0),
                at(4.0, 3.0, 0.0),
                at(0.0, 3.0, 0.0),
            ],
            uvs: vec![[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9]],
            indices: vec![[0, 1, 2], [0, 2, 3]],
            image: page_image(16, RED),
        },
        // ceiling: one triangle
        TexturePage {
            positions: vec![at(0.0, 0.0, 2.5), at(4.0, 3.0, 2.5), at(4.0, 0.0, 2.5)],
            uvs: vec![[0.2, 0.2], [0.8, 0.8], [0.8, 0.2]],
            indices: vec![[0, 1, 2]],
            image: page_image(16, BLUE),
        },
    ];
    let dir = temp_dir("roundtrip");
    let path = dir.join("oberflaeche.glb");
    write_glb(&path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();

    let loaded = load_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(loaded.pages.len(), tex.pages.len());
    let tris = |s: &LoadedSurface| s.pages.iter().map(|p| p.1.len() / 3).sum::<usize>();
    assert_eq!(tris(&loaded), 3);
    // the origin in whole metres, the positions relative to it small
    assert!(
        loaded.origin.iter().all(|v| v.fract() == 0.0),
        "{:?}",
        loaded.origin
    );
    for ((verts, idx, image), page) in loaded.pages.iter().zip(&tex.pages) {
        assert_eq!(idx.len(), page.indices.len() * 3);
        for (t, tri) in page.indices.iter().enumerate() {
            for k in 0..3 {
                let v = &verts[idx[t * 3 + k] as usize];
                let want = page.positions[tri[k] as usize];
                for (a, &w) in want.iter().enumerate() {
                    let got = loaded.origin[a] + f64::from(v.pos[a]);
                    assert!((got - f64::from(w)).abs() <= 1e-5, "{got} vs {w}");
                    assert!(v.pos[a].abs() < 10.0, "relative: {:?}", v.pos);
                }
                assert_eq!(v.uv, page.uvs[tri[k] as usize]);
            }
        }
        assert_eq!(image.dimensions(), (16, 16));
        let px = image.get_pixel(8, 8).0;
        let rgb = &page.image.data[..3];
        assert!((0..3).all(|k| px[k].abs_diff(rgb[k]) < 25), "{px:?}");
        assert_eq!(px[3], 255);
    }
}

#[test]
fn load_mesh_builds_mips() {
    let mut tex = TexturedMesh::default();
    tex.pages = vec![TexturePage {
        positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2]],
        image: page_image(16, GREEN),
    }];
    let dir = temp_dir("mips");
    let path = dir.join("oberflaeche.glb");
    write_glb(&path, &tex, &encode_pages(&tex, 90).unwrap()).unwrap();
    let loaded = load_mesh(&path, &Progress::new()).unwrap();
    // made in the loader (off the UI thread): only copied when uploaded
    assert_eq!(loaded.mips.len(), 1);
    let sizes: Vec<_> = loaded.mips[0].iter().map(|l| l.dimensions()).collect();
    assert_eq!(sizes, [(8, 8), (4, 4), (2, 2), (1, 1)]);
}

fn load_error(name: &str, bytes: &[u8]) -> String {
    let dir = temp_dir(name);
    let path = dir.join("oberflaeche.glb");
    fs::write(&path, bytes).unwrap();
    let e = load_mesh(&path, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains(&path.display().to_string()), "{e}");
    e
}

#[test]
fn missing_glb_is_a_german_error() {
    let dir = temp_dir("fehlt");
    let path = dir.join("oberflaeche.glb");
    let e = load_mesh(&path, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains(&path.display().to_string()), "{e}");
    assert!(e.contains("nicht gefunden"), "{e}");
}

#[test]
fn broken_glb_is_a_german_error() {
    let e = load_error("kaputt", b"glTF kein echtes glb");
    assert!(e.contains("keine gültige glb-Datei"), "{e}");
    // cut off in the middle: also refused, not read halfway
    let mut tex = TexturedMesh::default();
    tex.pages = vec![TexturePage {
        positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2]],
        image: page_image(16, GREEN),
    }];
    let dir = temp_dir("abgeschnitten");
    write_glb(
        &dir.join("ganz.glb"),
        &tex,
        &encode_pages(&tex, 90).unwrap(),
    )
    .unwrap();
    let whole = fs::read(dir.join("ganz.glb")).unwrap();
    let e = load_error("abgeschnitten-2", &whole[..whole.len() / 2]);
    assert!(e.contains("keine gültige glb-Datei"), "{e}");
}

#[test]
fn external_buffer_is_a_german_error() {
    // a glTF whose data lie in a separate .bin (not what `driftless surface` writes)
    let json = r#"{"asset":{"version":"2.0"},
        "buffers":[{"uri":"oberflaeche.bin","byteLength":36}],
        "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36}],
        "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3",
                      "min":[0,0,0],"max":[1,1,0]}],
        "meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}]}"#;
    let e = load_error("extern", json.as_bytes());
    assert!(
        e.contains("Daten liegen in einer eigenen Datei (oberflaeche.bin)"),
        "{e}"
    );
}

/// Draws what `mesh` holds and reads the centre back.
fn centre_of(gpu: &Gpu, mesh: &MeshRenderer, frame: &FrameParams) -> [u8; 4] {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let target = RenderTarget::new(device, [SIZE; 2], FORMAT);
    let mut points = PointRenderer::new(device, queue, FORMAT);
    let tree = no_cloud();
    points.upload_cloud(device, queue, &tree);
    points.render_with(device, queue, &target, frame, &tree, &[], Some(mesh));
    read_back(gpu, &target).centre()
}

#[test]
fn upload_in_steps_keeps_old_surface() {
    let Some(gpu) = gpu() else { return };
    let (device, queue) = (&gpu.device, &gpu.queue);
    let wall = |rgb| quad([2.0, -2.0, -2.0], [0.0, 4.0, 0.0], [0.0, 0.0, 4.0], rgb);
    let red = LoadedSurface {
        pages: vec![wall(RED)],
        ..LoadedSurface::default()
    };
    // two pages: the blue wall and a small one off to the side
    let side = quad([2.0, 5.0, 5.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1], GREEN);
    let blue = LoadedSurface {
        pages: vec![wall(BLUE), side],
        ..LoadedSurface::default()
    };
    let cam = camera([0.0; 3], 0.0, 0.0);
    let f = frame(&cam, ShowMode::Surface, NO_CUT);
    let mut mesh = MeshRenderer::new(device, FORMAT);
    mesh.upload(device, queue, &red);
    assert!(near(centre_of(&gpu, &mesh, &f), RED, 2));
    // a page per frame: until the last one is up, the old surface stays drawn
    mesh.begin_upload(blue.origin);
    assert!(mesh.upload_page(device, queue, &blue, 0) > 0);
    assert!(near(centre_of(&gpu, &mesh, &f), RED, 2));
    mesh.upload_page(device, queue, &blue, 1);
    mesh.finish_upload();
    assert!(near(centre_of(&gpu, &mesh, &f), BLUE, 2));
}

/// Says that the running test was skipped, and why. Written past the test harness's capture
/// (straight to stderr, not with `eprintln!`), so that a run without a GPU – in CI, say –
/// shows which tests did not test anything instead of passing in silence.
fn skipped(why: &str) {
    use std::io::Write;
    let name = std::thread::current().name().unwrap_or("?").to_string();
    let _ = writeln!(std::io::stderr(), "ÜBERSPRUNGEN {name}: {why}");
}

/// `w × h` texels of `rgb`, encoded as PNG.
fn png(w: u32, h: u32, rgb: [u8; 3]) -> Vec<u8> {
    let img = image::RgbImage::from_pixel(w, h, image::Rgb(rgb));
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Png).unwrap();
    out.into_inner()
}

/// An OBJ with a PNG page and a page of plain colour: the PNG is decoded, the colour page
/// becomes 1 × 1 in its colour; a missing texture gives a gray page and a warning.
#[test]
fn loads_obj_with_png_page() {
    use driftless_export::mesh::{write_obj, ImageKind, Mesh, MeshPage, PageImage};
    let base = [500_000.0, 5_400_000.0, 300.0];
    let at = |x: f64, y: f64, z: f64| [base[0] + x, base[1] + y, base[2] + z];
    let mesh = Mesh {
        pages: vec![
            MeshPage {
                positions: vec![at(0.0, 0.0, 0.0), at(2.0, 0.0, 0.0), at(0.0, 1.0, 0.0)],
                uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                indices: vec![[0, 1, 2]],
                image: Some(PageImage {
                    kind: ImageKind::Png,
                    bytes: png(4, 2, GREEN).into(),
                }),
                color: [255; 3],
            },
            MeshPage {
                positions: vec![at(0.0, 0.0, 2.0), at(1.0, 0.0, 2.0), at(0.0, 1.0, 2.0)],
                uvs: Vec::new(),
                indices: vec![[0, 1, 2]],
                image: None,
                color: [200, 10, 30],
            },
        ],
    };
    let dir = temp_dir("obj").inside("Büro Scan");
    let path = dir.join("Prüfung ä.obj");
    write_obj(&path, &mesh, &Progress::new()).unwrap();
    let loaded = load_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(loaded.pages.len(), 2);
    assert_eq!(loaded.triangles(), 2);
    // origin: rounded centre of x 0…2, y 0…1, z 0…2 around `base`
    assert_eq!(loaded.origin, [500_001.0, 5_400_001.0, 301.0]);
    for ((verts, idx, _), page) in loaded.pages.iter().zip(&mesh.pages) {
        assert_eq!(idx, &[0, 1, 2]);
        for (v, want) in verts.iter().zip(&page.positions) {
            for (k, w) in want.iter().enumerate() {
                let got = loaded.origin[k] + f64::from(v.pos[k]);
                assert!((got - w).abs() <= 1e-6, "{got} vs {w}");
            }
        }
    }
    let (_, _, image) = &loaded.pages[0];
    assert_eq!(image.dimensions(), (4, 2));
    assert_eq!(image.get_pixel(1, 1).0, [0, 255, 0, 255]);
    assert_eq!(loaded.pages[0].0[1].uv, [1.0, 0.0]);
    let (_, _, colour) = &loaded.pages[1];
    assert_eq!(colour.dimensions(), (1, 1));
    assert_eq!(colour.get_pixel(0, 0).0, [200, 10, 30, 255]);
    assert_eq!(loaded.mips.len(), 2);
    let sizes: Vec<_> = loaded.mips[0].iter().map(|l| l.dimensions()).collect();
    assert_eq!(sizes, [(2, 1), (1, 1)]);

    // the image is gone: a gray page, and the reader's warning
    fs::remove_file(dir.join("Prüfung ä_seite_00.png")).unwrap();
    let progress = Progress::new();
    let loaded = load_mesh(&path, &progress).unwrap();
    assert_eq!(loaded.pages[0].2.dimensions(), (1, 1));
    assert_eq!(loaded.pages[0].2.get_pixel(0, 0).0, [128, 128, 128, 255]);
    let log = progress.take_log();
    assert!(
        log.iter().any(|l| matches!(
            l,
            driftless_io::progress::LogLine::Warning(w) if w.contains("Textur fehlt")
        )),
        "{log:?}"
    );
}

/// A glb as foreign programs write it: binary chunk `bin`, JSON `json`.
fn glb_bytes(json: &str, bin: &[u8]) -> Vec<u8> {
    let mut text = json.as_bytes().to_vec();
    while !text.len().is_multiple_of(4) {
        text.push(b' ');
    }
    let mut bin = bin.to_vec();
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }
    let total = 12 + 8 + text.len() + 8 + bin.len();
    let mut out = Vec::new();
    out.extend(b"glTF");
    out.extend(2u32.to_le_bytes());
    out.extend((total as u32).to_le_bytes());
    out.extend((text.len() as u32).to_le_bytes());
    out.extend(b"JSON");
    out.extend(&text);
    out.extend((bin.len() as u32).to_le_bytes());
    out.extend(b"BIN\0");
    out.extend(&bin);
    out
}

/// glTF JSON with one triangle (glTF (0,0,0), (1,0,0), (0,1,0)) as mesh 0 and `extra` in the
/// root object; its buffer is [`triangle_bin`].
fn triangle_json(extra: &str) -> String {
    format!(
        r#"{{"asset":{{"version":"2.0","generator":"fremd"}},
        "buffers":[{{"byteLength":44}}],
        "bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":36}},
                       {{"buffer":0,"byteOffset":36,"byteLength":6}}],
        "accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3",
                        "min":[0,0,0],"max":[1,1,0]}},
                     {{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}}],
        "meshes":[{{"primitives":[{{"attributes":{{"POSITION":0}},"indices":1}}]}}]
        {extra}}}"#
    )
}

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

/// A foreign glb whose mesh hangs under moved and turned nodes: the view shows it where the
/// nodes put it (Z up), the page without texture white.
#[test]
fn loads_glb_with_node_transform() {
    let s = std::f32::consts::FRAC_1_SQRT_2;
    // parent 2 m up (glTF +y) and 100 m along x, child turned 90° about glTF +y
    let json = triangle_json(&format!(
        r#","scene":0,"scenes":[{{"nodes":[0]}}],
        "nodes":[{{"translation":[100,2,0],"children":[1]}},
                 {{"rotation":[0,{s},0,{s}],"mesh":0}}]"#
    ));
    let dir = temp_dir("glb-knoten");
    let path = dir.join("fremd.glb");
    fs::write(&path, glb_bytes(&json, &triangle_bin())).unwrap();
    let loaded = load_mesh(&path, &Progress::new()).unwrap();
    assert_eq!(loaded.pages.len(), 1);
    // glTF (1,0,0) turned → (0,0,−1), moved → (100,2,−1) = Driftless (100, 1, 2); glTF
    // (0,1,0) → (100,3,0) = (100, 0, 3)
    let want = [[100.0, 0.0, 2.0], [100.0, 1.0, 2.0], [100.0, 0.0, 3.0]];
    assert!(loaded.origin.iter().all(|v| v.fract() == 0.0));
    assert_eq!(loaded.origin[0], 100.0);
    let (verts, idx, image) = &loaded.pages[0];
    assert_eq!(idx, &[0, 1, 2]);
    for (v, want) in verts.iter().zip(want) {
        for (k, w) in want.iter().enumerate() {
            let got = loaded.origin[k] + f64::from(v.pos[k]);
            assert!((got - w).abs() < 1e-5, "{got} vs {w}");
        }
    }
    assert_eq!(image.dimensions(), (1, 1));
    assert_eq!(image.get_pixel(0, 0).0, [255; 4]);
}

/// A glb compressed with Draco is not read: the reader's German message, naming the file.
#[test]
fn draco_glb_shows_message() {
    let json = triangle_json(
        r#","extensionsUsed":["KHR_draco_mesh_compression"],
        "extensionsRequired":["KHR_draco_mesh_compression"]"#,
    );
    let e = load_error("draco", &glb_bytes(&json, &triangle_bin()));
    assert!(
        e.contains("glTF mit Draco-Kompression wird nicht unterstützt"),
        "{e}"
    );
}

/// CRC-32 (as in every PNG chunk).
fn crc32(data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |crc, &b| {
        (0..8).fold(crc ^ u32::from(b), |c, _| {
            if c & 1 == 1 {
                (c >> 1) ^ 0xEDB8_8320
            } else {
                c >> 1
            }
        })
    })
}

/// A PNG of a few bytes whose header claims `w × h` pixels (a stub of image data).
fn png_claiming(w: u32, h: u32) -> Vec<u8> {
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut chunk = |kind: &[u8], data: &[u8]| {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut body = kind.to_vec();
        body.extend_from_slice(data);
        out.extend_from_slice(&body);
        out.extend_from_slice(&crc32(&body).to_be_bytes());
    };
    let mut ihdr = w.to_be_bytes().to_vec();
    ihdr.extend_from_slice(&h.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(b"IHDR", &ihdr);
    chunk(b"IDAT", &[0x78, 0x9c]);
    chunk(b"IEND", &[]);
    out
}

/// A small file whose pages claim more pixels together than it can plausibly hold is refused
/// before any page is decoded (each page alone would be within the decoder's limit).
#[test]
fn load_mesh_refuses_oversized_textures() {
    use driftless_export::mesh::{write_glb, ImageKind, Mesh, MeshPage, PageImage};
    let page = |z: f64| MeshPage {
        positions: vec![[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]],
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2]],
        image: Some(PageImage {
            kind: ImageKind::Png,
            bytes: png_claiming(11_000, 11_000).into(),
        }),
        color: [255; 3],
    };
    let mesh = Mesh {
        pages: (0..5).map(|k| page(f64::from(k))).collect(),
    };
    let dir = temp_dir("textures-too-large");
    let path = dir.join("gross.glb");
    write_glb(&path, &mesh, &Progress::new()).unwrap();
    let start = std::time::Instant::now();
    let err = load_mesh(&path, &Progress::new()).unwrap_err().to_string();
    assert!(start.elapsed().as_secs_f64() < 1.0, "{:?}", start.elapsed());
    assert!(
        err.contains("Texturen zu groß: 605 Mio. Pixel, höchstens 536 Mio."),
        "{err}"
    );
}
