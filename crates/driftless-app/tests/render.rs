//! The wgpu renderer, drawn offscreen into 64 × 64 pixels and read back. Without a GPU
//! adapter (e.g. a CI runner without Metal) every test is skipped.

use driftless_app::camera::{Camera, CameraMode};
use driftless_app::cloud::PointSet;
use driftless_app::lod::{build_octree, build_octree_with, Octree, PointVertex, LEAF_MAX};
use driftless_app::pick::project;
use driftless_app::render::{
    headless_device, ColorMode, FrameParams, LineVertex, PointRenderer, PointSize, RenderTarget,
    ShowMode, NO_CUT,
};
use driftless_app::synthetic::Rng;
use driftless_io::progress::Progress;
use driftless_steps::overlay::quality_color;
use egui_wgpu::wgpu;

const SIZE: u32 = 64;
const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const BACKGROUND: [f32; 4] = [0.1, 0.1, 0.1, 1.0];
const RED: [u8; 3] = [255, 0, 0];
const BLUE: [u8; 3] = [0, 0, 255];

struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

/// The device, or `None` (and a note) when there is no adapter.
fn gpu() -> Option<Gpu> {
    let Some((device, queue, info)) = headless_device() else {
        skipped("kein GPU-Adapter");
        return None;
    };
    eprintln!("GPU-Adapter: {} ({:?})", info.name, info.backend);
    Some(Gpu { device, queue })
}

/// A camera at `eye` looking horizontally along the yaw (radians, 0 = +x).
fn camera(eye: [f32; 3], yaw: f32) -> Camera {
    Camera {
        mode: CameraMode::Fly,
        eye: eye.into(),
        yaw,
        pitch: 0.0,
        ..Camera::default()
    }
}

fn frame(cam: &Camera, mode: ColorMode) -> FrameParams {
    FrameParams {
        view_proj: cam.view_proj(1.0).into(),
        viewport_px: [SIZE as f32; 2],
        point_px: 8.0,
        marker_px: 16.0,
        line_scale: 1.0,
        color_mode: mode,
        round: false,
        background: BACKGROUND,
        quality_known: true,
        z_range: NO_CUT,
        show: ShowMode::Cloud,
        // the tests below this helper measure fixed pixel sizes
        point_size: PointSize::Pixel,
        point_m: 0.05,
        point_px_range: [1.0, 1000.0],
    }
}

fn cloud(xyz: Vec<[f32; 3]>, rgb: Vec<[u8; 3]>) -> PointSet {
    let mut points = PointSet::from_xyz([0.0; 3], xyz);
    points.rgb = Some(rgb);
    points
}

fn tree(points: &PointSet) -> Octree {
    build_octree(points, &Progress::new()).unwrap()
}

/// Pixels as RGBA, row by row from the top.
struct Image(Vec<[u8; 4]>);

impl Image {
    fn at(&self, x: u32, y: u32) -> [u8; 4] {
        self.0[(y * SIZE + x) as usize]
    }

    fn centre(&self) -> [u8; 4] {
        self.at(SIZE / 2, SIZE / 2)
    }
}

/// Draws all nodes of `tree` (and the overlay) and reads the image back.
fn render(
    gpu: &Gpu,
    tree: &Octree,
    frame: &FrameParams,
    lines: &[LineVertex],
    markers: &[PointVertex],
) -> Image {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let target = RenderTarget::new(device, [SIZE; 2], FORMAT);
    let mut renderer = PointRenderer::new(device, queue, FORMAT);
    renderer.upload_cloud(device, queue, tree);
    renderer.set_overlay(device, lines, markers);
    let drawn: Vec<u32> = (0..tree.nodes.len() as u32).collect();
    renderer.render(device, queue, &target, frame, tree, &drawn);
    read_back(gpu, &target)
}

fn read_back(gpu: &Gpu, target: &RenderTarget) -> Image {
    let (device, queue) = (&gpu.device, &gpu.queue);
    let row = SIZE * 4; // 256 B, the copy alignment
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

fn background() -> [u8; 3] {
    BACKGROUND.map(|c| (c * 255.0).round() as u8)[..3]
        .try_into()
        .unwrap()
}

fn near(pixel: [u8; 4], rgb: [u8; 3], tol: u8) -> bool {
    (0..3).all(|k| pixel[k].abs_diff(rgb[k]) <= tol)
}

fn is_background(pixel: [u8; 4]) -> bool {
    near(pixel, background(), 1)
}

#[test]
fn renders_point_at_centre() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[]);
    let c = image.centre();
    eprintln!("Mitte {c:?}, Ecke {:?}", image.at(0, 0));
    assert!(c[0] > 200 && c[1] < 50 && c[2] < 50, "centre {c:?}");
    assert!(is_background(image.at(0, 0)), "corner {:?}", image.at(0, 0));
    // 8 px across: 4 px from the centre is still the point, 6 px is not.
    assert!(near(image.at(SIZE / 2 + 3, SIZE / 2), RED, 2));
    assert!(is_background(image.at(SIZE / 2 + 6, SIZE / 2)));
}

#[test]
fn nearer_point_wins() {
    let Some(gpu) = gpu() else { return };
    // The red point is 2 m and the blue one 4 m from the eye, on the same line of sight.
    // The vertex order is the Morton order from the minimum of the bounds, so looking along
    // −x the blue point (at the minimum) comes first, looking along +x the red one. Only
    // the second case needs the depth test (the farther point is drawn last); both must
    // show red.
    for (eye, yaw, first) in [
        ([6.0, 0.0, 0.0], std::f32::consts::PI, BLUE),
        ([0.0, 0.0, 0.0], 0.0, RED),
    ] {
        let dir = if yaw == 0.0 { 1.0 } else { -1.0 };
        let red = [eye[0] + 2.0 * dir, 0.0, 0.0];
        let blue = [eye[0] + 4.0 * dir, 0.0, 0.0];
        let tree = tree(&cloud(vec![blue, red], vec![BLUE, RED]));
        let colour = |v: &PointVertex| [0, 8, 16].map(|s| (v.rgbi >> s) as u8);
        assert_eq!(colour(&tree.vertices[0]), first, "vertex order as intended");
        let cam = camera(eye, yaw);
        let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[]);
        let c = image.centre();
        eprintln!("zuerst {first:?}: Mitte {c:?}");
        assert!(near(c, RED, 2), "first {first:?}: centre {c:?}");
    }
}

#[test]
fn intensity_mode_is_gray() {
    let Some(gpu) = gpu() else { return };
    let mut points = cloud(vec![[2.0, 0.0, 0.0]], vec![RED]);
    points.intensity = vec![100];
    let tree = tree(&points);
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Intensity), &[], &[]);
    let c = image.centre();
    eprintln!("Mitte {c:?}");
    assert!(near(c, [100; 3], 1), "centre {c:?}");
}

#[test]
fn rgb_mode_without_colours_shows_intensity() {
    let Some(gpu) = gpu() else { return };
    let mut points = PointSet::from_xyz([0.0; 3], vec![[2.0, 0.0, 0.0]]);
    points.intensity = vec![180];
    let tree = tree(&points);
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[]);
    assert!(near(image.centre(), [180; 3], 1), "{:?}", image.centre());
}

#[test]
fn quality_mode_uses_ramp() {
    let Some(gpu) = gpu() else { return };
    let cam = camera([0.0; 3], 0.0);
    for q in [0u8, 64, 128, 191, 255] {
        let mut points = cloud(vec![[2.0, 0.0, 0.0]], vec![BLUE]);
        points.quality = vec![q];
        let tree = tree(&points);
        let image = render(&gpu, &tree, &frame(&cam, ColorMode::Quality), &[], &[]);
        let expected = quality_color(f32::from(q) / 255.0);
        let c = image.centre();
        eprintln!("Güte {q}: Mitte {c:?}, erwartet {expected:?}");
        assert!(near(c, expected, 1), "quality {q}: {c:?} vs {expected:?}");
    }
    assert_eq!(quality_color(0.0), RED);
    assert_eq!(quality_color(1.0), [0, 255, 0]);
}

#[test]
fn nothing_outside_frustum() {
    let Some(gpu) = gpu() else { return };
    // Behind the camera, and one far beside the view.
    let tree = tree(&cloud(
        vec![[-2.0, 0.0, 0.0], [2.0, 50.0, 0.0]],
        vec![RED, RED],
    ));
    let cam = camera([0.0; 3], 0.0);
    let mut f = frame(&cam, ColorMode::Rgb);
    f.point_px = 32.0;
    let image = render(&gpu, &tree, &f, &[], &[]);
    let drawn = image.0.iter().filter(|p| !is_background(**p)).count();
    assert_eq!(drawn, 0, "pixels not background");
}

#[test]
fn empty_cloud_renders_background() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    assert!(tree.chunks.is_empty());
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[]);
    assert!(image.0.iter().all(|p| is_background(*p)));
}

#[test]
fn lines_draw_over_background() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    // r | g << 8 | b << 16 | a << 24
    let green = 0xff00_ff00;
    // A vertical line slightly right of the centre, so that it lies inside one column.
    let lines = [
        LineVertex {
            pos: [2.0, -0.01, -1.0],
            rgba: green,
            width: 1.0,
        },
        LineVertex {
            pos: [2.0, -0.01, 1.0],
            rgba: green,
            width: 1.0,
        },
    ];
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &lines, &[]);
    let column: Vec<u32> = (SIZE / 2 - 2..SIZE / 2 + 2)
        .filter(|&x| near(image.at(x, SIZE / 2), [0, 255, 0], 1))
        .collect();
    eprintln!("Linie in Spalte {column:?}");
    assert!((1..=2).contains(&column.len()), "a 1 px line");
    assert!(is_background(image.at(0, 0)));
}

/// A segment with one end behind the camera is cut at the eye: only the visible part is
/// drawn, along its true direction on screen, no smeared triangles.
#[test]
fn line_through_the_eye_is_clipped() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    // below the eye, from 4 m ahead to 4 m behind: on screen from below the centre down
    let line = [[4.0, 0.0, -0.5], [-4.0, 0.0, -0.5]].map(|pos| LineVertex {
        pos,
        rgba: 0xff00_ff00,
        width: 2.0,
    });
    let cam = camera([0.0; 3], 0.0);
    let f = frame(&cam, ColorMode::Rgb);
    let image = render(&gpu, &tree, &f, &line, &[]);
    let far = project(
        &nalgebra::Matrix4::from(f.view_proj),
        line[0].pos,
        [SIZE as f32; 2],
    )
    .unwrap();
    let green: Vec<(u32, u32)> = (0..SIZE)
        .flat_map(|y| (0..SIZE).map(move |x| (x, y)))
        .filter(|&(x, y)| near(image.at(x, y), [0, 255, 0], 1))
        .collect();
    eprintln!("ferne Ende {far:?}, {} Pixel", green.len());
    assert!(!green.is_empty(), "the visible part is drawn");
    let centre = SIZE as f32 / 2.0;
    for &(x, y) in &green {
        assert!((x as f32 + 0.5 - centre).abs() <= 2.0, "column {x}");
        assert!(y as f32 + 0.5 >= far[1] - 2.0, "row {y} above the far end");
    }
    // it reaches the bottom edge (the part next to the eye)
    assert!(green.iter().any(|&(_, y)| y == SIZE - 1));
}

/// Lines are quads of their width in points times the pixels per point.
#[test]
fn line_width_scales_with_pixels_per_point() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    let line = |width| {
        [[2.0, 0.0, -1.0], [2.0, 0.0, 1.0]].map(|pos| LineVertex {
            pos,
            rgba: 0xff00_ff00,
            width,
        })
    };
    let cam = camera([0.0; 3], 0.0);
    let width_px = |scale: f32| {
        let mut f = frame(&cam, ColorMode::Rgb);
        f.line_scale = scale;
        let image = render(&gpu, &tree, &f, &line(2.5), &[]);
        (0..SIZE)
            .filter(|&x| near(image.at(x, SIZE / 2), [0, 255, 0], 1))
            .count()
    };
    let (one, two) = (width_px(1.0), width_px(2.0));
    eprintln!("2,5 pt: {one} px bei 1×, {two} px bei 2×");
    assert!((2..=3).contains(&one), "{one}");
    assert!((4..=6).contains(&two), "{two}");
}

#[test]
fn lines_are_hidden_behind_nearer_points() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    let lines = [
        LineVertex {
            pos: [4.0, -0.02, -1.0],
            rgba: 0xffff_ffff,
            width: 1.0,
        },
        LineVertex {
            pos: [4.0, -0.02, 1.0],
            rgba: 0xffff_ffff,
            width: 1.0,
        },
    ];
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &lines, &[]);
    assert!(near(image.centre(), RED, 2), "{:?}", image.centre());
    // Above the point the line is visible.
    let top = (SIZE / 2 - 2..SIZE / 2 + 2).any(|x| near(image.at(x, SIZE / 2 - 10), [255; 3], 1));
    assert!(top, "line visible outside the point");
}

#[test]
fn markers_use_marker_size_and_rgb() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    let white = PointVertex {
        pos: [2.0, 0.0, 0.0],
        rgbi: 0x00ff_ffff, // intensity 0: shown white even in intensity mode
        extra: 0,
    };
    let cam = camera([0.0; 3], 0.0);
    let mut f = frame(&cam, ColorMode::Intensity);
    f.point_px = 2.0;
    f.marker_px = 16.0;
    let image = render(&gpu, &tree, &f, &[], &[white]);
    // 16 px across: 6 px from the centre is still the marker.
    let c = image.at(SIZE / 2 + 6, SIZE / 2);
    eprintln!("Marker {c:?}");
    assert!(near(c, [255; 3], 1), "{c:?}");
    assert!(is_background(image.at(SIZE / 2 + 10, SIZE / 2)));
}

/// Markers lie on top: a marker behind a nearer point is still drawn.
#[test]
fn markers_draw_over_nearer_points() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    let marker = PointVertex {
        pos: [6.0, 0.0, 0.0],
        rgbi: 0x00ff_ffff,
        extra: 0,
    };
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[marker]);
    let c = image.centre();
    assert!(near(c, [255; 3], 1), "{c:?}");
}

/// `extra` of a marker scales its size in percent of `marker_px` (0 = 100 %).
#[test]
fn marker_size_from_extra() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&PointSet::default());
    let marker = |extra| PointVertex {
        pos: [2.0, 0.0, 0.0],
        rgbi: 0x00ff_ffff,
        extra,
    };
    let cam = camera([0.0; 3], 0.0);
    let mut f = frame(&cam, ColorMode::Rgb);
    f.marker_px = 16.0;
    // 50 %: 8 px across, so 6 px from the centre is background, 2 px is the marker
    let image = render(&gpu, &tree, &f, &[], &[marker(50)]);
    assert!(near(image.at(SIZE / 2 + 2, SIZE / 2), [255; 3], 1));
    assert!(is_background(image.at(SIZE / 2 + 6, SIZE / 2)));
    // 100 %
    let image = render(&gpu, &tree, &f, &[], &[marker(100)]);
    assert!(near(image.at(SIZE / 2 + 6, SIZE / 2), [255; 3], 1));
}

#[test]
fn round_points_leave_corners_free() {
    let Some(gpu) = gpu() else { return };
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    let cam = camera([0.0; 3], 0.0);
    let mut f = frame(&cam, ColorMode::Rgb);
    f.point_px = 16.0;
    let square = render(&gpu, &tree, &f, &[], &[]);
    f.round = true;
    let round = render(&gpu, &tree, &f, &[], &[]);
    // Pixel centre 6.5 px right of and below the image centre: inside the square (8 px),
    // outside the circle (9.2 px).
    let (x, y) = (SIZE / 2 + 6, SIZE / 2 + 6);
    assert!(near(square.at(x, y), RED, 2));
    assert!(is_background(round.at(x, y)), "{:?}", round.at(x, y));
    assert!(near(round.centre(), RED, 2));
}

#[test]
fn chunks_beyond_first_render() {
    let Some(gpu) = gpu() else { return };
    // More than LEAF_MAX points behind the camera, so that there are several nodes, and a
    // red point in front of it; blocks of 4 vertices.
    let mut rng = Rng::new(7);
    let mut xyz: Vec<[f32; 3]> = (0..LEAF_MAX + 100)
        .map(|_| {
            [
                -10.0 + rng.next_f32() * 5.0,
                rng.next_f32() * 5.0 - 2.5,
                rng.next_f32() * 5.0 - 2.5,
            ]
        })
        .collect();
    let mut rgb = vec![[128u8; 3]; xyz.len()];
    xyz.push([2.0, 0.0, 0.0]);
    rgb.push(RED);
    let red_index = (xyz.len() - 1) as u32;
    let points = cloud(xyz, rgb);
    let tree = build_octree_with(&points, 4, &Progress::new()).unwrap();
    assert!(tree.nodes.len() > 1, "several nodes");
    assert!(tree.chunks.iter().all(|c| c.len() <= 4));
    let vertex = tree.source.iter().position(|&s| s == red_index).unwrap() as u32;
    let block = tree
        .chunks
        .iter()
        .position(|c| c.contains(&vertex))
        .unwrap();
    eprintln!("{} Blöcke, roter Punkt in Block {block}", tree.chunks.len());
    assert!(block > 0, "red point beyond the first block");
    let cam = camera([0.0; 3], 0.0);
    let image = render(&gpu, &tree, &frame(&cam, ColorMode::Rgb), &[], &[]);
    assert!(near(image.centre(), RED, 2), "{:?}", image.centre());
}

#[test]
fn quality_mode_unknown_is_gray() {
    let Some(gpu) = gpu() else { return };
    let cam = camera([0.0; 3], 0.0);
    let mut points = cloud(vec![[2.0, 0.0, 0.0]], vec![BLUE]);
    points.quality = vec![0];
    let tree = tree(&points);
    let f = FrameParams {
        quality_known: false,
        ..frame(&cam, ColorMode::Quality)
    };
    let c = render(&gpu, &tree, &f, &[], &[]).centre();
    assert!(near(c, [128; 3], 2), "{c:?}");
}

#[test]
fn height_cut_hides_points_not_markers() {
    let Some(gpu) = gpu() else { return };
    let cam = camera([0.0; 3], 0.0);
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    let cut = |z_range| FrameParams {
        z_range,
        ..frame(&cam, ColorMode::Rgb)
    };
    // the point (z = 0) inside the cut: drawn; outside: not
    assert!(near(
        render(&gpu, &tree, &cut([-1.0, 1.0]), &[], &[]).centre(),
        RED,
        2
    ));
    assert!(is_background(
        render(&gpu, &tree, &cut([0.5, 1.0]), &[], &[]).centre()
    ));
    assert!(is_background(
        render(&gpu, &tree, &cut([-1.0, -0.5]), &[], &[]).centre()
    ));
    // a marker at the same place stays
    let marker = PointVertex {
        pos: [2.0, 0.0, 0.0],
        rgbi: 0xff00_ff00,
        extra: 0,
    };
    let c = render(&gpu, &tree, &cut([0.5, 1.0]), &[], &[marker]).centre();
    assert!(near(c, [0, 255, 0], 2), "{c:?}");
}

/// With timestamp queries the renderer reports the GPU time of a frame, without waiting.
#[test]
fn gpu_time_comes_back() {
    use driftless_app::render::with_timestamps;
    if gpu().is_none() {
        return;
    }
    let setup = match egui_kittest::wgpu::default_wgpu_setup() {
        egui_wgpu::WgpuSetup::CreateNew(s) => egui_wgpu::WgpuSetup::CreateNew(with_timestamps(s)),
        other => other,
    };
    let rs =
        egui_kittest::wgpu::create_render_state(setup, egui_wgpu::RendererOptions::PREDICTABLE);
    let mut renderer = PointRenderer::new(&rs.device, &rs.queue, FORMAT);
    if !renderer.has_gpu_timer() {
        skipped("keine GPU-Zeitstempel");
        return;
    }
    let tree = tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]));
    renderer.upload_cloud(&rs.device, &rs.queue, &tree);
    let target = RenderTarget::new(&rs.device, [SIZE; 2], FORMAT);
    let f = frame(&camera([0.0; 3], 0.0), ColorMode::Rgb);
    let mut got = None;
    for _ in 0..200 {
        renderer.render(&rs.device, &rs.queue, &target, &f, &tree, &[0]);
        std::thread::sleep(std::time::Duration::from_millis(5));
        got = renderer.gpu_ms(&rs.device);
        if got.is_some() {
            break;
        }
    }
    let ms = got.expect("GPU time");
    eprintln!("GPU {ms} ms");
    assert!(ms > 0.0 && ms < 1000.0);
}

/// How many pixels of the centre row are not background.
fn centre_row_width(image: &Image) -> usize {
    (0..SIZE)
        .filter(|&x| !is_background(image.at(x, SIZE / 2)))
        .count()
}

/// A frame in world size: diameter `point_m`, at least `min` and at most `max` px.
fn world_frame(cam: &Camera, point_m: f32, range: [f32; 2]) -> FrameParams {
    FrameParams {
        point_size: PointSize::World,
        point_m,
        point_px: range[0],
        point_px_range: range,
        ..frame(cam, ColorMode::Rgb)
    }
}

/// In world size a point four times as far is a quarter as wide on screen.
#[test]
fn world_size_shrinks_with_distance() {
    let Some(gpu) = gpu() else { return };
    // A narrow field of view, so that a 5 cm spot is many pixels across at 1 m.
    let cam = Camera {
        fov_y: 10f32.to_radians(),
        ..camera([0.0; 3], 0.0)
    };
    let width = |d: f32| {
        let tree = tree(&cloud(vec![[d, 0.0, 0.0]], vec![RED]));
        let f = world_frame(&cam, 0.05, [1.0, 1000.0]);
        centre_row_width(&render(&gpu, &tree, &f, &[], &[]))
    };
    let (near, far) = (width(1.0), width(4.0));
    eprintln!("5 cm: {near} px bei 1 m, {far} px bei 4 m");
    assert!(far >= 2, "far point visible: {far}");
    assert!(
        (near as f32 / 4.0 - far as f32).abs() <= 1.0,
        "near {near} ≈ 4 × far {far}"
    );
}

/// Very near spots stay at most `max` px wide, very far ones at least `min` px.
#[test]
fn world_size_clamped() {
    let Some(gpu) = gpu() else { return };
    // an even minimum: a round 4 px spot at the image centre covers 4 pixel centres of a row
    let (min, max) = (4.0, 24.0);
    // the near plane in front of the point, so that it is drawn
    let cam = Camera {
        near: 0.01,
        ..camera([0.0; 3], 0.0)
    };
    let width = |d: f32| {
        let tree = tree(&cloud(vec![[d, 0.0, 0.0]], vec![RED]));
        let f = world_frame(&cam, 0.05, [min, max]);
        centre_row_width(&render(&gpu, &tree, &f, &[], &[]))
    };
    let (near, far) = (width(0.05), width(200.0));
    eprintln!("5 cm: {near} px bei 0,05 m, {far} px bei 200 m");
    // unclamped ≈ 55 px across
    assert!(near as f32 <= max, "near {near} > {max}");
    assert!(near as f32 >= max - 2.0, "near {near}: drawn at the limit");
    assert!(far as f32 >= min, "far {far} < {min}");
    assert!(far as f32 <= min + 1.0, "far {far}");
}

/// In pixel mode the world size is ignored: the image is the same pixel for pixel, and a
/// square 8 px point covers exactly the 8 × 8 pixels around the centre, as before.
#[test]
fn pixel_mode_unchanged() {
    let Some(gpu) = gpu() else { return };
    let cam = camera([0.0; 3], 0.0);
    let tree = tree(&cloud(
        vec![[2.0, 0.0, 0.0], [4.0, 0.5, 0.3], [8.0, -1.0, -0.5]],
        vec![RED, BLUE, [0, 255, 0]],
    ));
    for round in [false, true] {
        let a = FrameParams {
            round,
            ..frame(&cam, ColorMode::Rgb)
        };
        let b = FrameParams {
            point_m: 5.0,
            point_px_range: [30.0, 40.0],
            ..a
        };
        let (ia, ib) = (
            render(&gpu, &tree, &a, &[], &[]),
            render(&gpu, &tree, &b, &[], &[]),
        );
        assert!(
            ia.0 == ib.0,
            "round {round}: pixel mode depends on the world size"
        );
    }
    let only_red = tree_of_one();
    let image = render(&gpu, &only_red, &frame(&cam, ColorMode::Rgb), &[], &[]);
    for y in 0..SIZE {
        for x in 0..SIZE {
            let inside = (SIZE / 2 - 4..SIZE / 2 + 4).contains(&x)
                && (SIZE / 2 - 4..SIZE / 2 + 4).contains(&y);
            let p = image.at(x, y);
            if inside {
                assert!(near(p, RED, 0), "({x}, {y}) {p:?}");
            } else {
                assert!(is_background(p), "({x}, {y}) {p:?}");
            }
        }
    }
}

fn tree_of_one() -> Octree {
    tree(&cloud(vec![[2.0, 0.0, 0.0]], vec![RED]))
}

/// The spot size comes from the map's grid: `dense.voxel` × 1.5, else 1.5 cm.
#[test]
fn point_m_from_map() {
    use driftless_app::cloud::{point_m_of_map, LoadedCloud, DEFAULT_POINT_M};
    let dir = std::env::temp_dir().join(format!("driftless-app-point-m-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    assert_eq!(DEFAULT_POINT_M, 0.015);
    assert_eq!(point_m_of_map(&dir), 0.015, "without map.json");
    std::fs::write(dir.join("map.json"), "kein json").unwrap();
    assert_eq!(point_m_of_map(&dir), 0.015, "unreadable map.json");
    std::fs::write(
        dir.join("map.json"),
        r#"{"settings": {"dense": {"voxel": 0.02}}}"#,
    )
    .unwrap();
    let m = point_m_of_map(&dir);
    assert!((m - 0.03).abs() < 1e-6, "{m}");
    // a loaded cloud of that map takes it
    let points = cloud(vec![[2.0, 0.0, 0.0]], vec![RED]);
    let loaded = LoadedCloud::build(dir.join("color.las"), points, 0.0, &Progress::new()).unwrap();
    assert!((loaded.point_m - 0.03).abs() < 1e-6, "{}", loaded.point_m);
    std::fs::remove_dir_all(&dir).unwrap();
}

/// Says that the running test was skipped, and why. Written past the test harness's capture
/// (straight to stderr, not with `eprintln!`), so that a run without a GPU – in CI, say –
/// shows which tests did not test anything instead of passing in silence.
fn skipped(why: &str) {
    use std::io::Write;
    let name = std::thread::current().name().unwrap_or("?").to_string();
    let _ = writeln!(std::io::stderr(), "ÜBERSPRUNGEN {name}: {why}");
}
