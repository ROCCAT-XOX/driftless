//! Real-data comparison images and hole figures of surfaces against their clouds (ignored; needs
//! the results of a `driftless map` / `surface` run, see tools/qualitaet/README.md).
//! Env: QA_A_SURF (folder with oberflaeche.glb), QA_A_CLOUD (color.las), QA_TRAJ (traj.tum),
//! QA_OUT (folder for the images), optional QA_B_SURF + QA_B_CLOUD (a second run, shown next to
//! the first), QA_PREFIX, QA_VIEWS ("name:ex,ey,ez:tx,ty,tz:fov;...").
//! Prints one line per view and run: `LOECHER <view> <percent>` (run A) and `LOECHER_B <view>
//! <percent>` (run B): the share of pixels with cloud points that show no surface.
//! cargo test --release -p driftless-steps --test quality_real -- --ignored --nocapture

use driftless_io::images::{decode_jpeg, RgbImage};
use driftless_map::surface::{render_preview, TexturePage, TexturedMesh};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

const BACKGROUND: [u8; 3] = [40, 40, 40];

type Cloud = Vec<([f32; 3], [u8; 3])>;

/// A surface and the cloud it was built from.
struct Run {
    mesh: TexturedMesh,
    cloud: Cloud,
}

fn env(k: &str) -> String {
    std::env::var(k).unwrap_or_else(|_| panic!("{k} fehlt"))
}

fn parse3(s: &str) -> [f32; 3] {
    let v: Vec<f32> = s.split(',').map(|x| x.trim().parse().unwrap()).collect();
    [v[0], v[1], v[2]]
}

fn load_run(surf: &str, cloud: &str) -> Run {
    Run {
        mesh: read_glb(&Path::new(&env(surf)).join("oberflaeche.glb")),
        cloud: read_las(Path::new(&env(cloud))),
    }
}

#[test]
#[ignore = "real data: needs the results of a quality run"]
fn quality_images() {
    let out = PathBuf::from(env("QA_OUT"));
    fs::create_dir_all(&out).unwrap();
    let prefix = std::env::var("QA_PREFIX").unwrap_or_default();
    let mut runs = vec![load_run("QA_A_SURF", "QA_A_CLOUD")];
    if std::env::var("QA_B_SURF").is_ok() {
        runs.push(load_run("QA_B_SURF", "QA_B_CLOUD"));
    }
    let traj = read_traj(Path::new(&env("QA_TRAJ")));
    let reference = &runs[runs.len() - 1].cloud;
    println!("{} Läufe, Wolke: {} Punkte", runs.len(), reference.len());
    let mean_z = traj.iter().map(|p| p[2]).sum::<f32>() / traj.len() as f32;
    let cut = mean_z + 0.6;
    let (lo, hi) = bounds(reference, cut);
    let centre = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, mean_z];
    println!("Wolke bis {cut:.2} m: {lo:?} … {hi:?}, Mitte {centre:?}");
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]) * 1.1;
    let height = 40.0;
    let fov = 2.0 * (span / 2.0 / (height - (cut - lo[2]))).atan().to_degrees();
    let eye = [centre[0], centre[1], cut + height];
    let low: Vec<Run> = runs
        .iter()
        .map(|r| Run {
            mesh: cut_mesh(&r.mesh, cut),
            cloud: r.cloud.iter().filter(|p| p.0[2] <= cut).copied().collect(),
        })
        .collect();
    save_view(
        &out,
        &format!("{prefix}oben"),
        "oben",
        &low,
        eye,
        [centre[0], centre[1], lo[2]],
        fov,
        [900, 900],
    );
    for k in 0..8 {
        let a = k as f32 * std::f32::consts::FRAC_PI_4;
        let target = [centre[0] + a.cos(), centre[1] + a.sin(), centre[2]];
        let name = format!("rundblick_{:03}", k * 45);
        save_view(
            &out,
            &format!("{prefix}{name}"),
            &name,
            &runs,
            centre,
            target,
            90.0,
            [640, 480],
        );
    }
    if let Ok(views) = std::env::var("QA_VIEWS") {
        for v in views.split(';').filter(|s| !s.trim().is_empty()) {
            let p: Vec<&str> = v.split(':').collect();
            save_view(
                &out,
                &format!("{prefix}{}", p[0]),
                p[0],
                &runs,
                parse3(p[1]),
                parse3(p[2]),
                p[3].parse().unwrap(),
                [800, 600],
            );
        }
    }
}

/// Image grid: one column per run, top the surface, bottom the points. Prints the hole figure
/// of every run for this view.
#[allow(clippy::too_many_arguments)]
fn save_view(
    out: &Path,
    file: &str,
    view: &str,
    runs: &[Run],
    eye: [f32; 3],
    target: [f32; 3],
    fov: f32,
    size: [u32; 2],
) {
    let surfaces: Vec<RgbImage> = runs
        .iter()
        .map(|r| render_preview(&r.mesh, eye, target, fov, size))
        .collect();
    let points: Vec<RgbImage> = runs
        .iter()
        .map(|r| render_points(&r.cloud, eye, target, fov, size))
        .collect();
    let gap = 8u32;
    let cols = runs.len() as u32;
    let w = size[0] * cols + gap * (cols - 1);
    let h = size[1] * 2 + gap;
    let mut data = vec![255u8; (w * h * 3) as usize];
    for (k, img) in surfaces.iter().chain(&points).enumerate() {
        let ox = (k as u32 % cols) * (size[0] + gap);
        let oy = (k as u32 / cols) * (size[1] + gap);
        let row = size[0] as usize * 3;
        for y in 0..size[1] as usize {
            let dst = ((oy as usize + y) * w as usize + ox as usize) * 3;
            data[dst..dst + row].copy_from_slice(&img.data[y * row..(y + 1) * row]);
        }
    }
    write_png(&out.join(format!("{file}.png")), w, h, &data);
    let bg = |img: &RgbImage, i: usize| img.data[i * 3..i * 3 + 3] == BACKGROUND;
    let n = (size[0] * size[1]) as usize;
    for (k, (s, c)) in surfaces.iter().zip(&points).enumerate() {
        let covered = (0..n).filter(|&i| !bg(c, i)).count();
        let holes = (0..n).filter(|&i| !bg(c, i) && bg(s, i)).count();
        let tag = if k == 0 { "LOECHER" } else { "LOECHER_B" };
        println!(
            "{tag} {view} {:.1}",
            100.0 * holes as f64 / covered.max(1) as f64
        );
    }
}

fn write_png(path: &Path, w: u32, h: u32, rgb: &[u8]) {
    let file = fs::File::create(path).unwrap();
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    let mut writer = enc.write_header().unwrap();
    writer.write_image_data(rgb).unwrap();
    writer.finish().unwrap();
}

/// The mesh without the triangles that reach above `z`.
fn cut_mesh(mesh: &TexturedMesh, z: f32) -> TexturedMesh {
    let mut out = mesh.clone();
    for page in &mut out.pages {
        let pos = &page.positions;
        page.indices
            .retain(|t| t.iter().all(|&i| pos[i as usize][2] <= z));
    }
    out
}

/// x/y/z range of the points at or below `z`.
fn bounds(cloud: &[([f32; 3], [u8; 3])], z: f32) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for (p, _) in cloud.iter().filter(|p| p.0[2] <= z) {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

/// The points as small squares, nearest wins, with the camera of `render_preview` (pinhole,
/// Z up, square pixels, vertical field of view). A point covers about 6 mm.
fn render_points(
    cloud: &[([f32; 3], [u8; 3])],
    eye: [f32; 3],
    target: [f32; 3],
    fov: f32,
    size: [u32; 2],
) -> RgbImage {
    let (w, h) = (size[0] as usize, size[1] as usize);
    let mut data = BACKGROUND.repeat(w * h);
    let mut depth = vec![f32::MAX; w * h];
    let forward = norm(sub(target, eye));
    let mut right = cross(forward, [0.0, 0.0, 1.0]);
    if dot(right, right) < 1e-12 {
        right = cross(forward, [0.0, 1.0, 0.0]);
    }
    let right = norm(right);
    let up = cross(right, forward);
    let focal = h as f32 / 2.0 / (fov.to_radians() / 2.0).tan();
    let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
    for (p, rgb) in cloud {
        let d = sub(*p, eye);
        let z = dot(d, forward);
        if z < 0.01 {
            continue;
        }
        let x = cx + focal * dot(d, right) / z;
        let y = cy - focal * dot(d, up) / z;
        let r = (focal * 0.003 / z).clamp(0.5, 6.0);
        let (x0, x1) = ((x - r).floor().max(0.0), (x + r).ceil().min(w as f32));
        let (y0, y1) = ((y - r).floor().max(0.0), (y + r).ceil().min(h as f32));
        for py in y0 as usize..y1.max(0.0) as usize {
            for px in x0 as usize..x1.max(0.0) as usize {
                let i = py * w + px;
                if z < depth[i] {
                    depth[i] = z;
                    data[i * 3..i * 3 + 3].copy_from_slice(rgb);
                }
            }
        }
    }
    RgbImage {
        width: size[0],
        height: size[1],
        data,
    }
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f32; 3]) -> [f32; 3] {
    let l = dot(a, a).sqrt();
    a.map(|x| x / l)
}

/// Positions of a TUM trajectory.
fn read_traj(path: &Path) -> Vec<[f32; 3]> {
    fs::read_to_string(path)
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let v: Vec<f32> = l
                .split_whitespace()
                .skip(1)
                .take(3)
                .map(|s| s.parse().unwrap())
                .collect();
            [v[0], v[1], v[2]]
        })
        .collect()
}

/// Points and colours (8 bit) of a LAS file.
fn read_las(path: &Path) -> Vec<([f32; 3], [u8; 3])> {
    let data = las::Reader::from_path(path).unwrap().read_all().unwrap();
    let rgb = data.rgb().expect("color.las ohne Farbe");
    data.x()
        .zip(data.y())
        .zip(data.z())
        .zip(rgb)
        .map(|(((x, y), z), (r, g, b))| {
            (
                [x as f32, y as f32, z as f32],
                [r, g, b].map(|v| (v / 257) as u8),
            )
        })
        .collect()
}

/// Reads a glb written by `write_glb` back into pages (Driftless frame, Z up).
fn read_glb(path: &Path) -> TexturedMesh {
    let bytes = fs::read(path).unwrap();
    assert_eq!(&bytes[0..4], b"glTF");
    let u32_at = |o: usize| u32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as usize;
    let json_len = u32_at(12);
    let json: Value = serde_json::from_slice(&bytes[20..20 + json_len]).unwrap();
    let bin_start = 20 + json_len + 8;
    let bin = &bytes[bin_start..bin_start + u32_at(20 + json_len)];
    let view = |i: usize| {
        let v = &json["bufferViews"][i];
        let off = v["byteOffset"].as_u64().unwrap_or(0) as usize;
        &bin[off..off + v["byteLength"].as_u64().unwrap() as usize]
    };
    let accessor = |i: usize| view(json["accessors"][i]["bufferView"].as_u64().unwrap() as usize);
    let floats = |b: &[u8]| -> Vec<f32> {
        b.as_chunks::<4>()
            .0
            .iter()
            .map(|&c| f32::from_le_bytes(c))
            .collect()
    };
    let mut pages = Vec::new();
    for prim in json["meshes"][0]["primitives"].as_array().unwrap() {
        let pos = floats(accessor(
            prim["attributes"]["POSITION"].as_u64().unwrap() as usize
        ));
        let uv = floats(accessor(
            prim["attributes"]["TEXCOORD_0"].as_u64().unwrap() as usize
        ));
        let idx: Vec<u32> = accessor(prim["indices"].as_u64().unwrap() as usize)
            .as_chunks::<4>()
            .0
            .iter()
            .map(|&c| u32::from_le_bytes(c))
            .collect();
        let material = &json["materials"][prim["material"].as_u64().unwrap() as usize];
        let texture = material["pbrMetallicRoughness"]["baseColorTexture"]["index"]
            .as_u64()
            .unwrap() as usize;
        let image = json["textures"][texture]["source"].as_u64().unwrap() as usize;
        let jpeg = view(json["images"][image]["bufferView"].as_u64().unwrap() as usize);
        pages.push(TexturePage {
            // glb (x, y, z) Y up = Driftless (x, −z, y).
            positions: pos
                .as_chunks::<3>()
                .0
                .iter()
                .map(|c| [c[0], -c[2], c[1]])
                .collect(),
            uvs: uv.as_chunks::<2>().0.to_vec(),
            indices: idx.as_chunks::<3>().0.to_vec(),
            image: decode_jpeg(jpeg, 1).unwrap(),
        });
    }
    let mut mesh = TexturedMesh::default();
    mesh.pages = pages;
    mesh
}
