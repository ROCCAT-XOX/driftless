//! The surface on real scans (all `#[ignore]`, need `DRIFTLESS_TESTDATA`).
//!
//! The surface itself is computed beforehand with the release CLI (≈ 3 min, see
//! `docs/ergebnisse/2026-10-22-baustein-2.md`):
//!
//! ```sh
//! S=$DRIFTLESS_TESTDATA/jmk6-2026-10-22/20261022133554
//! driftless surface $S --slam $S/driftless -o $S/driftless/oberflaeche-b2
//! ```
//!
//! `surface_real_report_meets_targets` checks the written report and file sizes against the
//! targets; `surface_real_comparison_images` reads the glb back and renders the same
//! viewpoints for the surface and for the coloured point cloud (`color.las`, drawn as
//! points) side by side into `$DRIFTLESS_TESTDATA/ergebnisse-b2/` (or `DRIFTLESS_B2_OUT`).
//!
//! ```sh
//! cargo test --release -p driftless-steps --test surface_real -- --ignored --nocapture
//! ```

use driftless_io::images::{decode_jpeg, RgbImage};
use driftless_map::surface::{render_preview, TexturePage, TexturedMesh};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

/// The scan of the black room (the longest of the three).
const BLACK_ROOM: &str = "jmk6-2026-10-22/20261022133554";
/// Output folder of the surface inside the scan folder; `DRIFTLESS_B2_SURFACE` picks another
/// one (e.g. `oberflaeche-b2-2cm` for a run with `--voxel 0.02`), whose images then get its
/// name as prefix.
fn surface_dir() -> (String, String) {
    match std::env::var("DRIFTLESS_B2_SURFACE") {
        Ok(name) => (format!("driftless/{name}"), format!("b2_{name}_")),
        Err(_) => ("driftless/oberflaeche-b2".into(), "b2_".into()),
    }
}
/// Background of the point images, as in `render_preview`.
const BACKGROUND: [u8; 3] = [40, 40, 40];

fn scan_dir() -> Option<PathBuf> {
    let dir = driftless_io::testdata::path(BLACK_ROOM)?;
    let surface = dir.join(surface_dir().0);
    if surface.join("oberflaeche.json").exists() {
        Some(dir)
    } else {
        eprintln!(
            "übersprungen: {} fehlt (zuerst `driftless surface` rechnen)",
            surface.display()
        );
        None
    }
}

fn out_dir() -> PathBuf {
    let dir = std::env::var_os("DRIFTLESS_B2_OUT")
        .map(PathBuf::from)
        .or_else(|| driftless_io::testdata::path("ergebnisse-b2"))
        .expect("DRIFTLESS_TESTDATA");
    fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
#[ignore = "echte Daten: DRIFTLESS_TESTDATA, vorher `driftless surface`"]
fn surface_real_report_meets_targets() {
    let Some(scan) = scan_dir() else { return };
    let dir = scan.join(surface_dir().0);
    let report: Value =
        serde_json::from_slice(&fs::read(dir.join("oberflaeche.json")).unwrap()).unwrap();
    println!(
        "{}",
        serde_json::to_string_pretty(&report["stats"]).unwrap()
    );
    let stats = &report["stats"];
    let glb = fs::metadata(dir.join("oberflaeche.glb")).unwrap().len();
    let seconds: f64 = report["seconds_total"].as_f64().unwrap();
    let rss = report["peak_rss_bytes"].as_f64().unwrap_or(0.0);
    let acc = |part: &str, key: &str| stats["accuracy"][part][key].as_f64().unwrap();
    for part in ["all", "walls", "edges"] {
        println!(
            "Genauigkeit {part}: {} Ecken, Median {:.2} / Mittel {:.2} / 95 % {:.2} mm",
            stats["accuracy"][part]["vertices"],
            acc(part, "median_mm"),
            acc(part, "mean_mm"),
            acc(part, "p95_mm")
        );
    }
    println!(
        "glb {:.1} MB, {seconds:.0} s, RAM {:.2} GB",
        glb as f64 / 1e6,
        rss / 1e9
    );
    assert!(glb < 300_000_000, "glb {glb} B");
    assert!(seconds < 600.0, "{seconds} s");
    assert!(rss < 4e9, "{rss} B");
    // The target of the plan on the flat walls; corners and edges of a 2 cm grid are cut
    // off by marching cubes and are reported, not checked.
    let walls = acc("walls", "median_mm");
    assert!(walls <= 3.0, "{walls} mm");
}

#[test]
#[ignore = "echte Daten: DRIFTLESS_TESTDATA, vorher `driftless surface`"]
fn surface_real_comparison_images() {
    let Some(scan) = scan_dir() else { return };
    let out = out_dir();
    let (surface, prefix) = surface_dir();
    let mesh = read_glb(&scan.join(surface).join("oberflaeche.glb"));
    let cloud = read_las(&scan.join("driftless/color.las"));
    let traj = read_traj(&scan.join("driftless/traj.tum"));
    let faces: usize = mesh.pages.iter().map(|p| p.indices.len()).sum();
    println!(
        "glb: {} Seiten, {faces} Dreiecke; Wolke {} Punkte; Laufbahn {} Lagen",
        mesh.pages.len(),
        cloud.len(),
        traj.len()
    );

    // Overview from above, everything higher than 0.6 m above the mean scanner height cut
    // away (the ceiling and the upper walls).
    let mean_z = traj.iter().map(|p| p[2]).sum::<f32>() / traj.len() as f32;
    let cut = mean_z + 0.6;
    let (lo, hi) = bounds(&cloud, cut);
    let centre = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, mean_z];
    println!("Wolke bis {cut:.2} m: {lo:?} … {hi:?}, Mitte {centre:?}");
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]) * 1.1;
    let height = 40.0;
    let fov = 2.0 * (span / 2.0 / (height - (cut - lo[2]))).atan().to_degrees();
    let eye = [centre[0], centre[1], cut + height];
    let low = cut_mesh(&mesh, cut);
    let low_cloud: Vec<_> = cloud.iter().filter(|p| p.0[2] <= cut).copied().collect();
    save_pair(
        &out.join(format!("{prefix}oben.png")),
        &low,
        &low_cloud,
        eye,
        [centre[0], centre[1], lo[2]],
        fov,
        [900, 900],
    );

    // Look-around from the middle of the room at scanner height, 8 directions.
    for k in 0..8 {
        let a = k as f32 * std::f32::consts::FRAC_PI_4;
        let target = [centre[0] + a.cos(), centre[1] + a.sin(), centre[2]];
        save_pair(
            &out.join(format!("{prefix}rundblick_{:03}.png", k * 45)),
            &mesh,
            &cloud,
            centre,
            target,
            90.0,
            [640, 480],
        );
    }

    for (name, eye, target, fov) in VIEWS {
        save_pair(
            &out.join(format!("{prefix}{name}.png")),
            &mesh,
            &cloud,
            *eye,
            *target,
            *fov,
            [800, 600],
        );
    }
}

/// Named viewpoints in the black room (world frame of the map, Z up): eye, target, field of
/// view (°). Chosen from the overview and the look-around: the room spans x −6.9…1.4 m,
/// y −6.8…1.1 m, the floor is at z ≈ −1.5 m, the scanner was carried at z ≈ −0.1 m.
const VIEWS: &[(&str, [f32; 3], [f32; 3], f32)] = &[
    // Wall y ≈ 1.1: the emergency-exit sign above the double door, close and whole.
    ("notausgang", [-2.58, -0.9, 0.3], [-2.58, 1.07, 1.2], 50.0),
    (
        "notausgang_tuer",
        [-2.6, -1.0, 0.0],
        [-2.6, 1.07, -0.4],
        75.0,
    ),
    // Wall x ≈ −6.9: the single door.
    ("tuer", [-4.95, -2.88, -0.2], [-6.95, -2.88, -0.5], 75.0),
    // Wall x ≈ 1.4: the vertical light bars.
    ("leuchtstaebe", [-0.6, -2.88, 0.0], [1.4, -2.88, -0.1], 75.0),
    // Wall y ≈ −6.8: the ladder (and more light bars).
    ("leiter", [-2.38, -4.6, 0.3], [-2.38, -6.84, 0.3], 80.0),
];

fn save_pair(
    path: &Path,
    mesh: &TexturedMesh,
    cloud: &[([f32; 3], [u8; 3])],
    eye: [f32; 3],
    target: [f32; 3],
    fov: f32,
    size: [u32; 2],
) {
    let a = render_preview(mesh, eye, target, fov, size);
    let b = render_points(cloud, eye, target, fov, size);
    let gap = 8;
    let w = size[0] * 2 + gap;
    let mut data = vec![255u8; (w * size[1] * 3) as usize];
    for y in 0..size[1] as usize {
        let row = size[0] as usize * 3;
        let dst = y * w as usize * 3;
        data[dst..dst + row].copy_from_slice(&a.data[y * row..(y + 1) * row]);
        let dst = dst + (size[0] + gap) as usize * 3;
        data[dst..dst + row].copy_from_slice(&b.data[y * row..(y + 1) * row]);
    }
    write_png(path, w, size[1], &data);
    // Holes: pixels where the cloud shows something and the surface nothing.
    let bg = |img: &RgbImage, i: usize| img.data[i * 3..i * 3 + 3] == BACKGROUND;
    let n = (size[0] * size[1]) as usize;
    let covered = (0..n).filter(|&i| !bg(&b, i)).count();
    let holes = (0..n).filter(|&i| !bg(&b, i) && bg(&a, i)).count();
    println!(
        "{}: Löcher {:.1} % der Pixel mit Punkten",
        path.display(),
        100.0 * holes as f64 / covered.max(1) as f64
    );
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
