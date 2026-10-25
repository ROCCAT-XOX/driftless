//! The textured surface: drawn offscreen into 64 × 64 pixels and read back (without a GPU
//! adapter those tests are skipped), and the glb written by `driftless-map` read back.

use driftless_app::camera::{Camera, CameraMode};
use driftless_app::cloud::PointSet;
use driftless_app::lod::{build_octree, Octree};
use driftless_app::render::mesh::{MeshRenderer, MeshVertex};
use driftless_app::render::{
    headless_device, ColorMode, FrameParams, PointRenderer, PointSize, RenderTarget, ShowMode,
    NO_CUT,
};
use driftless_app::surface::{load_glb, LoadedSurface};
use driftless_io::images::RgbImage;
use driftless_io::progress::Progress;
use driftless_map::surface::{write_glb, TexturePage, TexturedMesh};
use egui_wgpu::wgpu;
use image::RgbaImage;
use std::fs;
use std::path::PathBuf;

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

fn temp_dir(name: &str) -> PathBuf {
    let dir =
        std::env::temp_dir().join(format!("driftless-app-mesh-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
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
fn load_glb_matches_written() {
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
    write_glb(&path, &tex, 90).unwrap();

    let loaded = load_glb(&path, &Progress::new()).unwrap();
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
fn load_glb_builds_mips() {
    let mut tex = TexturedMesh::default();
    tex.pages = vec![TexturePage {
        positions: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        indices: vec![[0, 1, 2]],
        image: page_image(16, GREEN),
    }];
    let dir = temp_dir("mips");
    let path = dir.join("oberflaeche.glb");
    write_glb(&path, &tex, 90).unwrap();
    let loaded = load_glb(&path, &Progress::new()).unwrap();
    // made in the loader (off the UI thread): only copied when uploaded
    assert_eq!(loaded.mips.len(), 1);
    let sizes: Vec<_> = loaded.mips[0].iter().map(|l| l.dimensions()).collect();
    assert_eq!(sizes, [(8, 8), (4, 4), (2, 2), (1, 1)]);
}

fn load_error(name: &str, bytes: &[u8]) -> String {
    let dir = temp_dir(name);
    let path = dir.join("oberflaeche.glb");
    fs::write(&path, bytes).unwrap();
    let e = load_glb(&path, &Progress::new()).unwrap_err().to_string();
    assert!(e.contains(&path.display().to_string()), "{e}");
    e
}

#[test]
fn missing_glb_is_a_german_error() {
    let path = temp_dir("fehlt").join("oberflaeche.glb");
    let e = load_glb(&path, &Progress::new()).unwrap_err().to_string();
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
    write_glb(&dir.join("ganz.glb"), &tex, 90).unwrap();
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
