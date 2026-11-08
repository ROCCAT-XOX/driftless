//! The textured surface: drawn offscreen into 64 × 64 pixels and read back (without a GPU
//! adapter those tests are skipped), and the glb written by `driftless-map`, OBJ and foreign
//! glb read back. Also the marked Stellen over it and the touched-up state („Nachher“).

use driftless_app::camera::{Camera, CameraMode};
use driftless_app::cloud::PointSet;
use driftless_app::lod::{build_octree, Octree};
use driftless_app::pick::project;
use driftless_app::render::mesh::{
    mark_color, mark_triangles, MarkTriangle, MeshEdit, MeshRenderer, MeshVertex, MARK_OUTLINE_PT,
};
use driftless_app::render::{
    headless_device, ColorMode, FrameParams, PointRenderer, PointSize, RenderTarget, ShowMode,
    NO_CUT,
};
use driftless_app::surface::{load_mesh, mesh_edit, LoadedSurface};
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
const GRAY: [u8; 3] = [128, 128, 128];
/// [`BACKGROUND`] as bytes.
const BACKGROUND_RGB: [u8; 3] = [26, 26, 26];

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
        self.at(SIZE / 2, SIZE / 2)
    }

    /// Pixel `x`, `y` from the top left.
    fn at(&self, x: u32, y: u32) -> [u8; 4] {
        self.0[(y * SIZE + x) as usize]
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

/// Draws what `mesh` holds (without a cloud) and reads the image back.
fn draw(gpu: &Gpu, mesh: &MeshRenderer, frame: &FrameParams) -> Image {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let target = RenderTarget::new(device, [SIZE; 2], FORMAT);
    let mut points = PointRenderer::new(device, queue, FORMAT);
    let tree = no_cloud();
    points.upload_cloud(device, queue, &tree);
    points.render_with(device, queue, &target, frame, &tree, &[], Some(mesh));
    read_back(gpu, &target)
}

/// Draws what `mesh` holds and reads the centre back.
fn centre_of(gpu: &Gpu, mesh: &MeshRenderer, frame: &FrameParams) -> [u8; 4] {
    draw(gpu, mesh, frame).centre()
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

// --- The marked Stellen and the touched-up state („Vorher/Nachher“) ---

/// A rectangle of one colour as `n × n` cells of two triangles each: `corner` and the two
/// edges `a` and `b` from it. Cell (i, j) – `i` along `a`, `j` along `b` – holds the triangles
/// `2 (j n + i)` and `2 (j n + i) + 1`, cornered like those of [`quad`].
fn grid(
    corner: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    n: u32,
    rgb: [u8; 3],
) -> (Vec<MeshVertex>, Vec<u32>, RgbaImage) {
    let at = |s: f32, t: f32| [0, 1, 2].map(|k| corner[k] + s * a[k] + t * b[k]);
    let mut vertices = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            let (s, t) = (i as f32 / n as f32, j as f32 / n as f32);
            vertices.push(MeshVertex {
                pos: at(s, t),
                uv: [s, t],
            });
        }
    }
    let v = |i: u32, j: u32| j * (n + 1) + i;
    let mut indices = Vec::new();
    for j in 0..n {
        for i in 0..n {
            indices.extend([v(i, j), v(i + 1, j), v(i + 1, j + 1)]);
            indices.extend([v(i, j), v(i + 1, j + 1), v(i, j + 1)]);
        }
    }
    let image = RgbaImage::from_pixel(8, 8, image::Rgba([rgb[0], rgb[1], rgb[2], 255]));
    (vertices, indices, image)
}

/// The triangles of the cells (i, j) of a [`grid`] of `n × n` with `i` and `j` in `range`.
fn cells(n: u32, range: std::ops::Range<u32>) -> Vec<usize> {
    let mut out = Vec::new();
    for j in range.clone() {
        for i in range.clone() {
            let cell = (j * n + i) as usize;
            out.extend([2 * cell, 2 * cell + 1]);
        }
    }
    out
}

/// Copies of the triangles `tris` of `page` as one Stelle in `color`.
fn marks_of(
    page: &(Vec<MeshVertex>, Vec<u32>, RgbaImage),
    tris: &[usize],
    color: [u8; 4],
) -> Vec<MarkTriangle> {
    let corners: Vec<[u32; 3]> = tris
        .iter()
        .map(|&t| [page.1[3 * t], page.1[3 * t + 1], page.1[3 * t + 2]])
        .collect();
    mark_triangles(&corners, |v| page.0[v as usize].pos, color)
}

/// A camera at `eye` looking at `target`.
fn look_at(eye: [f32; 3], target: [f32; 3]) -> Camera {
    let d = [0, 1, 2].map(|k| target[k] - eye[k]);
    camera(eye, d[1].atan2(d[0]), d[2].atan2(d[0].hypot(d[1])))
}

/// `top` with alpha `a` (of 255) over `under`.
fn over(top: [u8; 3], a: u8, under: [u8; 3]) -> [u8; 3] {
    let a = f32::from(a) / 255.0;
    [0, 1, 2].map(|k| (f32::from(top[k]) * a + f32::from(under[k]) * (1.0 - a)).round() as u8)
}

/// How far `got` lies on the way from `from` to `to`: 0 at `from`, 1 at `to`.
fn share(got: [u8; 4], from: [u8; 3], to: [u8; 3]) -> f32 {
    let (mut along, mut length) = (0.0, 0.0);
    for k in 0..3 {
        let span = f32::from(to[k]) - f32::from(from[k]);
        along += (f32::from(got[k]) - f32::from(from[k])) * span;
        length += span * span;
    }
    along / length
}

/// Pixels from `p` to the rim of the convex polygon `poly` (on the screen, either winding):
/// positive inside, negative outside.
fn inside_by(poly: &[[f32; 2]], p: [f32; 2]) -> f32 {
    let n = poly.len();
    let edge = |k: usize| (poly[k], poly[(k + 1) % n]);
    let area: f32 = (0..n)
        .map(|k| {
            let (a, b) = edge(k);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    (0..n)
        .map(|k| {
            let (a, b) = edge(k);
            let e = [b[0] - a[0], b[1] - a[1]];
            area.signum() * (e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])) / e[0].hypot(e[1])
        })
        .fold(f32::INFINITY, f32::min)
}

/// A Stelle on a wall is tinted in its colour and outlined at its rim, and around it nothing
/// changes. Its triangles are copies drawn over the wall's with the same depth: straight on and
/// at a slant (where the depth changes fast across every triangle) every pixel inside is
/// tinted, the plain wall shows through nowhere. A plate in front of it hides it.
#[test]
fn mark_tints_and_outlines() {
    let Some(gpu) = gpu() else { return };
    let (device, queue) = (&gpu.device, &gpu.queue);
    // A red wall 2 m ahead of the eye at 0 (looking along +x), 4 × 4 m in cells of 0.5 m from
    // the left (+y) and the bottom; the Stelle is its middle 2 × 2 m.
    let wall = grid([2.0, 2.0, -2.0], [0.0, -4.0, 0.0], [0.0, 0.0, 4.0], 8, RED);
    let color = mark_color(2);
    let marks = marks_of(&wall, &cells(8, 2..6), color);
    assert_eq!(marks.len(), 32);
    let at = |i: u32, j: u32| [2.0, 2.0 - 0.5 * i as f32, -2.0 + 0.5 * j as f32];
    let rim = [at(2, 2), at(6, 2), at(6, 6), at(2, 6)];
    let rim_corners: Vec<[f32; 3]> = (2..=6)
        .flat_map(|i| (2..=6).map(move |j| (i, j)))
        .filter(|&(i, j)| i == 2 || i == 6 || j == 2 || j == 6)
        .map(|(i, j)| at(i, j))
        .collect();
    let mut mesh = MeshRenderer::new(device, FORMAT);
    mesh.upload(
        device,
        queue,
        &LoadedSurface {
            pages: vec![wall.clone()],
            ..LoadedSurface::default()
        },
    );
    let rgb = [color[0], color[1], color[2]];
    let tint = over(rgb, color[3], RED);
    for (name, cam) in [
        ("gerade", camera([0.0; 3], 0.0, 0.0)),
        ("schräg", look_at([0.3, -2.0, 1.2], [2.0, 0.0, 0.0])),
    ] {
        let straight = name == "gerade";
        let f = frame(&cam, ShowMode::Surface, NO_CUT);
        mesh.set_marks(device, queue, &[]);
        let plain = draw(&gpu, &mesh, &f);
        mesh.set_marks(device, queue, &marks);
        let marked = draw(&gpu, &mesh, &f);
        let vp = cam.view_proj(1.0);
        let screen = |p: [f32; 3]| {
            let q = project(&vp, p, [SIZE as f32; 2]).unwrap();
            [q[0], q[1]]
        };
        let poly = rim.map(screen);
        assert!(
            poly.iter()
                .flatten()
                .all(|&c| c > 2.0 && c < SIZE as f32 - 2.0),
            "{name}: {poly:?}"
        );
        let corners: Vec<[f32; 2]> = rim_corners.iter().map(|&p| screen(p)).collect();
        let (mut inner, mut edge, mut outer) = (0, 0, 0);
        let mut wrong = Vec::new();
        for y in 0..SIZE {
            for x in 0..SIZE {
                let p = [x as f32 + 0.5, y as f32 + 0.5];
                let d = inside_by(&poly, p);
                let (got, was) = (marked.at(x, y), plain.at(x, y));
                if d < -1.0 {
                    // around the Stelle: as without it
                    outer += 1;
                    if got != was {
                        wrong.push(("außen", x, y, got));
                    }
                } else if d > MARK_OUTLINE_PT + 2.0 {
                    // inside: tinted everywhere
                    inner += 1;
                    if !near(got, tint, 3) {
                        wrong.push(("innen", x, y, got));
                    }
                } else if straight && d > 0.2 && d < MARK_OUTLINE_PT - 0.7 {
                    // at the rim: the outline, opaque in the Stelle's colour; next to a corner
                    // on the rim it may thin a little
                    edge += 1;
                    let at_corner = corners
                        .iter()
                        .any(|c| (c[0] - p[0]).hypot(c[1] - p[1]) < MARK_OUTLINE_PT + 0.5);
                    let ok = if at_corner {
                        share(got, tint, rgb) > 0.6
                    } else {
                        near(got, rgb, 3)
                    };
                    if !ok {
                        wrong.push(("Rand", x, y, got));
                    }
                }
            }
        }
        eprintln!(
            "{name}: Mitte {:?}, Rand (32, 5) {:?}, außen (32, 2) {:?}, ohne Stelle {:?}; \
             geprüft {inner} innen, {edge} am Rand, {outer} außen",
            marked.centre(),
            marked.at(SIZE / 2, 5),
            marked.at(SIZE / 2, 2),
            plain.centre(),
        );
        assert!(
            wrong.is_empty(),
            "{name}: {} Pixel falsch, z. B. {:?}",
            wrong.len(),
            &wrong[..wrong.len().min(8)]
        );
        assert!(
            inner > 300 && outer > 100,
            "{name}: {inner} innen, {outer} außen"
        );
        if straight {
            assert!(edge > 100, "{edge} am Rand");
        }
    }

    // A new surface drops the marks of the one before; a plate 0.5 m in front of the
    // Stelle's middle hides the mark behind it.
    let plate = quad([1.5, 0.25, -0.25], [0.0, -0.5, 0.0], [0.0, 0.0, 0.5], GREEN);
    mesh.upload(
        device,
        queue,
        &LoadedSurface {
            pages: vec![wall, plate],
            ..LoadedSurface::default()
        },
    );
    let f = frame(&camera([0.0; 3], 0.0, 0.0), ShowMode::Surface, NO_CUT);
    let img = draw(&gpu, &mesh, &f);
    assert!(
        near(img.at(SIZE / 2, 16), RED, 2),
        "{:?}",
        img.at(SIZE / 2, 16)
    );
    mesh.set_marks(device, queue, &marks);
    let img = draw(&gpu, &mesh, &f);
    assert!(near(img.centre(), GREEN, 2), "plate {:?}", img.centre());
    assert!(
        near(img.at(SIZE / 2, 16), tint, 3),
        "{:?}",
        img.at(SIZE / 2, 16)
    );
}

/// The height cut takes the mark away with the surface under it (else it would float where the
/// cut-away wall was), and „Punktwolke“ draws neither.
#[test]
fn marks_follow_height_cut() {
    let Some(gpu) = gpu() else { return };
    let (device, queue) = (&gpu.device, &gpu.queue);
    // the whole red wall 2 m ahead is the Stelle; the cut keeps what lies below 0 m
    let wall = grid([2.0, 2.0, -2.0], [0.0, -4.0, 0.0], [0.0, 0.0, 4.0], 2, RED);
    let color = mark_color(1);
    let marks = marks_of(&wall, &cells(2, 0..2), color);
    let mut mesh = MeshRenderer::new(device, FORMAT);
    mesh.upload(
        device,
        queue,
        &LoadedSurface {
            pages: vec![wall],
            ..LoadedSurface::default()
        },
    );
    mesh.set_marks(device, queue, &marks);
    let tint = over([color[0], color[1], color[2]], color[3], RED);
    let cam = camera([0.0; 3], 0.0, 0.0);
    let uncut = draw(&gpu, &mesh, &frame(&cam, ShowMode::Surface, NO_CUT));
    let cut = draw(&gpu, &mesh, &frame(&cam, ShowMode::Surface, [-10.0, 0.0]));
    let (above, below) = ((SIZE / 2, 16), (SIZE / 2, 48));
    assert!(
        near(uncut.at(above.0, above.1), tint, 3),
        "{:?}",
        uncut.at(above.0, above.1)
    );
    assert!(
        near(cut.at(above.0, above.1), BACKGROUND_RGB, 1),
        "cut away: {:?}",
        cut.at(above.0, above.1)
    );
    assert!(
        near(cut.at(below.0, below.1), tint, 3),
        "{:?}",
        cut.at(below.0, below.1)
    );
    let points = draw(&gpu, &mesh, &frame(&cam, ShowMode::Cloud, NO_CUT));
    assert!(
        near(points.centre(), BACKGROUND_RGB, 1),
        "{:?}",
        points.centre()
    );
}

/// `w × h` texels of `rgb`, encoded as JPEG.
fn jpeg(w: u32, h: u32, rgb: [u8; 3]) -> Vec<u8> {
    let img = image::RgbImage::from_pixel(w, h, image::Rgb(rgb));
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Jpeg).unwrap();
    out.into_inner()
}

/// A computed surface and one step of touch-up on it, written and read back as the app does.
struct EditCase {
    /// The computed surface and the touched-up one (`apply_patches`), each read from its glb.
    before: LoadedSurface,
    after: LoadedSurface,
    /// What the touch-up changes, for the view ([`mesh_edit`] of `page_edits`).
    edit: MeshEdit,
}

/// The computed surface: a red page (upper left) and a green one (lower left) 3 m ahead of the
/// absolute 0, a gray wall 7 m ahead behind everything (it fixes the bounds, so the touched-up
/// surface has the same origin). The step moves the red page 1.7 m to the right („Glätten“)
/// and puts the green page's second triangle on a blue page of its own („Bilder neu“).
fn edit_case(name: &str) -> EditCase {
    use driftless_export::mesh::{write_glb, ImageKind, Mesh, MeshPage, PageImage};
    use driftless_map::surface::edit::{
        apply_patches, page_edits, PatchPage, SurfacePatch, WeldedSurface,
    };
    let page = |corner: [f64; 3], a: [f64; 3], b: [f64; 3], rgb| MeshPage {
        positions: [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
            .iter()
            .map(|[s, t]| [0, 1, 2].map(|k| corner[k] + s * a[k] + t * b[k]))
            .collect(),
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2], [0, 2, 3]],
        image: Some(PageImage {
            kind: ImageKind::Png,
            bytes: png(4, 4, rgb).into(),
        }),
        color: [255; 3],
    };
    let (left, up) = ([0.0, -1.3, 0.0], [0.0, 0.0, 1.3]);
    let mesh = Mesh {
        pages: vec![
            page([3.0, 1.5, 0.2], left, up, RED),
            page([3.0, 1.5, -1.5], left, up, GREEN),
            page([7.0, 7.0, -7.0], [0.0, -14.0, 0.0], [0.0, 0.0, 14.0], GRAY),
        ],
    };
    let s = WeldedSurface::from_mesh(&mesh);
    let moved = (0..s.positions().len() as u32)
        .filter(|&v| s.copies(v).iter().any(|&(page, _)| page == 0))
        .map(|v| {
            let p = s.positions()[v as usize];
            (v, [p[0], p[1] - 1.7, p[2]])
        })
        .collect();
    let face = 3;
    assert_eq!(s.face_origin(face), (1, 1));
    let patch = SurfacePatch {
        moved,
        removed_faces: vec![face],
        pages: vec![PatchPage {
            positions: s.faces()[face as usize]
                .map(|v| s.positions()[v as usize])
                .to_vec(),
            uvs: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            indices: vec![[0, 1, 2]],
            faces: vec![face],
            jpeg: jpeg(8, 8, BLUE).into(),
        }],
    };
    let (edits, added) = page_edits(&s, &mesh, std::slice::from_ref(&patch));
    let touched = apply_patches(&mesh, &s, &[patch]);
    let dir = temp_dir(name);
    let read = |file: &str, m: &Mesh| {
        let path = dir.join(file);
        write_glb(&path, m, &Progress::new()).unwrap();
        load_mesh(&path, &Progress::new()).unwrap()
    };
    let before = read("vorher.glb", &mesh);
    let after = read("nachher.glb", &touched);
    assert_eq!(before.origin, after.origin);
    let edit = mesh_edit(before.origin, &mesh, edits, added).unwrap();
    EditCase {
        before,
        after,
        edit,
    }
}

/// What the view swaps in is the touched-up surface: the corners of the moved page, the
/// triangles of the page that lost one and the added page are those of `apply_patches`, read
/// back as the app reads a surface.
#[test]
fn mesh_edit_matches_touched_up_surface() {
    let EditCase {
        before,
        after,
        edit,
    } = edit_case("bearbeitung");
    assert_eq!(edit.pages.len(), 2, "{:?}", edit.pages);
    // the red page: its corners moved, its triangles as they were
    let (page, vertices, indices) = &edit.pages[0];
    assert_eq!(*page, 0);
    assert_eq!(vertices.as_ref(), Some(&after.pages[0].0));
    assert_ne!(after.pages[0].0, before.pages[0].0);
    assert_eq!(*indices, None);
    // the green page: its corners as they were, its second triangle gone
    let (page, vertices, indices) = &edit.pages[1];
    assert_eq!((*page, vertices), (1, &None));
    assert_eq!(indices.as_deref(), Some(&before.pages[1].1[..3]));
    assert_eq!(indices.as_ref(), Some(&after.pages[1].1));
    // the gray wall is untouched; the blue page is added, decoded with its mipmaps
    assert_eq!(after.pages[2], before.pages[2]);
    assert_eq!(after.pages.len(), 4);
    assert_eq!(edit.extra, vec![after.pages[3].clone()]);
    assert_eq!(edit.extra_mips, vec![after.mips[3].clone()]);
}

/// „Vorher/Nachher“: with the touched-up state given, the view draws the computed surface or
/// the touched-up one – the same image as the touched-up surface uploaded as a surface of its
/// own – and switching between them uploads nothing.
#[test]
fn edit_toggle_swaps_without_upload() {
    let Some(gpu) = gpu() else { return };
    let (device, queue) = (&gpu.device, &gpu.queue);
    let EditCase {
        before,
        after,
        edit,
    } = edit_case("vorher-nachher");
    // the eye at the absolute 0, looking along +x
    let o = before.origin;
    let rel = |p: [f64; 3]| [0, 1, 2].map(|k| (p[k] - o[k]) as f32);
    let cam = camera(rel([0.0; 3]), 0.0, 0.0);
    let f = frame(&cam, ShowMode::Surface, NO_CUT);
    let vp = cam.view_proj(1.0);
    let pixel = |p: [f64; 3]| {
        let q = project(&vp, rel(p), [SIZE as f32; 2]).unwrap();
        (q[0] as u32, q[1] as u32)
    };
    // the middles of the red page before and after its move, and of the green page's triangles
    let red_before = pixel([3.0, 0.85, 0.85]);
    let red_after = pixel([3.0, -0.85, 0.85]);
    let kept = pixel([3.0, 0.633, -1.067]);
    let redone = pixel([3.0, 1.067, -0.633]);
    let check = |img: &Image, touched: bool| {
        let px = |(x, y)| img.at(x, y);
        let (red, gray) = if touched {
            (red_after, red_before)
        } else {
            (red_before, red_after)
        };
        assert!(near(px(red), RED, 2), "{touched}: {:?}", px(red));
        assert!(near(px(gray), GRAY, 2), "{touched}: {:?}", px(gray));
        assert!(near(px(kept), GREEN, 2), "{touched}: {:?}", px(kept));
        let want = if touched { BLUE } else { GREEN };
        assert!(near(px(redone), want, 12), "{touched}: {:?}", px(redone));
    };

    let mut touched = MeshRenderer::new(device, FORMAT);
    touched.upload(device, queue, &after);
    let touched_image = draw(&gpu, &touched, &f);
    check(&touched_image, true);

    let mut mesh = MeshRenderer::new(device, FORMAT);
    mesh.upload(device, queue, &before);
    let computed_image = draw(&gpu, &mesh, &f);
    check(&computed_image, false);
    let surface_bytes = mesh.uploaded_bytes();
    assert!(surface_bytes > 0);
    mesh.set_edit(device, queue, Some(edit.clone()));
    let with_edit = mesh.uploaded_bytes();
    assert!(with_edit > surface_bytes, "{with_edit} vs {surface_bytes}");
    eprintln!(
        "hochgeladen: Oberfläche {surface_bytes} B, Bearbeitung {} B",
        with_edit - surface_bytes
    );

    // „Vorher“ is the default
    assert!(draw(&gpu, &mesh, &f).0 == computed_image.0);
    for _ in 0..2 {
        mesh.show_edit(true);
        let img = draw(&gpu, &mesh, &f);
        check(&img, true);
        assert!(
            img.0 == touched_image.0,
            "Nachher = nachgearbeitete Oberfläche"
        );
        mesh.show_edit(false);
        let img = draw(&gpu, &mesh, &f);
        check(&img, false);
        assert!(img.0 == computed_image.0, "Vorher = gerechnete Oberfläche");
    }
    assert_eq!(
        mesh.uploaded_bytes(),
        with_edit,
        "Umschalten lädt nichts hoch"
    );

    // without the touched-up state „Nachher“ draws the computed surface
    mesh.show_edit(true);
    mesh.set_edit(device, queue, None);
    assert!(draw(&gpu, &mesh, &f).0 == computed_image.0);
    // a new surface drops the touched-up state of the one before
    mesh.set_edit(device, queue, Some(edit));
    assert!(draw(&gpu, &mesh, &f).0 == touched_image.0);
    mesh.upload(device, queue, &before);
    assert!(draw(&gpu, &mesh, &f).0 == computed_image.0);
}

/// The 3D view draws the marks and the touched-up state as soon as they are given – also when
/// they come before the surface is up – and switches between „Vorher“ and „Nachher“: a frame is
/// drawn anew only when its key changes, so each of them must be in it.
#[test]
fn view_redraws_for_marks_and_edit() {
    use driftless_app::ui::view3d::View3d;
    use driftless_app::ui::ViewSettings;
    use eframe::egui;
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    use egui_kittest::Harness;
    if headless_device().is_none() {
        skipped("kein GPU-Adapter");
        return;
    }
    // A red floor of 4 × 4 m (the view looks down on it), all of it one Stelle; „Nachher“ the
    // same floor on a blue page of its own, the red page without triangles.
    let floor = |rgb| grid([-2.0, -2.0, 0.0], [4.0, 0.0, 0.0], [0.0, 4.0, 0.0], 4, rgb);
    let color = mark_color(3);
    let marks = marks_of(&floor(RED), &cells(4, 0..4), color);
    let edit = MeshEdit {
        pages: vec![(0, None, Some(Vec::new()))],
        extra: vec![floor(BLUE)],
        ..MeshEdit::default()
    };
    let rgb = [color[0], color[1], color[2]];
    let (red_tint, blue_tint) = (over(rgb, color[3], RED), over(rgb, color[3], BLUE));
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let r = rs.clone();
    let mut h = Harness::builder()
        .with_size([64.0, 64.0])
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(
            move |ui, view: &mut View3d| {
                egui::CentralPanel::no_frame().show(ui, |ui| {
                    view.show(ui, &r, None, None, &ViewSettings::default());
                });
            },
            View3d::default(),
        );
    fn centre(h: &mut Harness<'_, View3d>) -> [u8; 4] {
        h.run();
        h.render().unwrap().get_pixel(32, 32).0
    }
    // given right after the surface, before it is up: they wait for it
    h.state_mut().set_surface(Some(LoadedSurface {
        pages: vec![floor(RED)],
        ..LoadedSurface::default()
    }));
    h.state_mut().set_marks(marks.clone());
    let c = centre(&mut h);
    assert!(near(c, red_tint, 3), "marked {c:?}");
    h.state_mut().set_marks(Vec::new());
    let c = centre(&mut h);
    assert!(near(c, RED, 2), "unmarked {c:?}");
    h.state_mut().set_marks(marks);
    h.state_mut().set_edit(Some(edit));
    let c = centre(&mut h);
    assert!(near(c, red_tint, 3), "Vorher {c:?}");
    h.state_mut().show_edit(true);
    let c = centre(&mut h);
    assert!(near(c, blue_tint, 3), "Nachher {c:?}");
    h.state_mut().show_edit(false);
    let c = centre(&mut h);
    assert!(near(c, red_tint, 3), "Vorher again {c:?}");
    h.state_mut().show_edit(true);
    h.state_mut().set_edit(None);
    let c = centre(&mut h);
    assert!(near(c, red_tint, 3), "no touch-up {c:?}");
}
