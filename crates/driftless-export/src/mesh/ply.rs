//! PLY mesh: binary little-endian, one colour per corner.
//!
//! `element vertex` with `x y z` (`float`, or `double` once a coordinate reaches
//! [`crate::MAX_FLOAT_COORD`]) and `red green blue` (`uchar`); `element face` with
//! `property list uchar int vertex_indices`, one face per triangle. Corners with
//! bit-identical positions are welded across all pages (only corners a triangle uses are
//! written). The colour of a corner is the mean of the texture, sampled bilinearly, at every
//! texture coordinate it has on any page; a page without an image gives its colour.

use super::{check_mesh, Mesh};
use crate::{check_cancel, ExportError, MAX_FLOAT_COORD, PHASE_WRITE};
use driftless_io::progress::Progress;
use driftless_io::TempPath;
use image::RgbImage;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Writes `mesh` as PLY mesh through `<name>.tmp` next to `path` (see the module docs).
/// Progress: phase [`PHASE_WRITE`], one step per page (each image is decoded once), a
/// cancel check each; an error or a cancel leaves `path` as it was and no `.tmp`.
pub fn write_ply_mesh(path: &Path, mesh: &Mesh, progress: &Progress) -> Result<(), ExportError> {
    check_cancel(progress)?;
    check_mesh(path, mesh)?;

    // Weld: per page the welded corner of each of its corners (u32::MAX: unused).
    let mut welded: HashMap<[u64; 3], u32> = HashMap::new();
    let mut corners: Vec<[f64; 3]> = Vec::new();
    let mut maps: Vec<Vec<u32>> = Vec::with_capacity(mesh.pages.len());
    for page in &mesh.pages {
        let mut map = vec![u32::MAX; page.positions.len()];
        for &i in page.indices.iter().flatten() {
            let slot = &mut map[i as usize];
            if *slot != u32::MAX {
                continue;
            }
            let p = page.positions[i as usize];
            // +0.0 and −0.0 are the same place.
            let key = p.map(|c| (c + 0.0).to_bits());
            let next = corners.len();
            let id = *welded.entry(key).or_insert_with(|| next as u32);
            if id as usize == next {
                if i32::try_from(next).is_err() {
                    return Err(ExportError::format(
                        path,
                        "Netz hat zu viele Ecken für PLY (> 2³¹)",
                    ));
                }
                corners.push(p);
            }
            *slot = id;
        }
        maps.push(map);
    }
    drop(welded);

    // Colours: sums per welded corner, page by page.
    progress.phase(PHASE_WRITE, Some(mesh.pages.len() as u64));
    let mut sums = vec![([0f64; 3], 0u32); corners.len()];
    for (p, (page, map)) in mesh.pages.iter().zip(&maps).enumerate() {
        check_cancel(progress)?;
        let image = match &page.image {
            Some(img) if map.iter().any(|&m| m != u32::MAX) => {
                let bad = |e: String| {
                    ExportError::format(path, format!("Seite {p}: Bild nicht lesbar ({e})"))
                };
                let decoded =
                    image::load_from_memory_with_format(&img.bytes, img.kind.image_format())
                        .map_err(|e| bad(e.to_string()))?
                        .to_rgb8();
                if decoded.width() == 0 || decoded.height() == 0 {
                    return Err(bad("keine Pixel".into()));
                }
                Some(decoded)
            }
            _ => None,
        };
        for (i, &id) in map.iter().enumerate() {
            if id == u32::MAX {
                continue;
            }
            let c = match &image {
                Some(img) => sample(img, page.uvs[i]),
                None => page.color.map(f64::from),
            };
            let s = &mut sums[id as usize];
            for (sum, c) in s.0.iter_mut().zip(c) {
                *sum += c;
            }
            s.1 += 1;
        }
        progress.advance(1);
    }

    let double = corners
        .iter()
        .any(|p| p.iter().any(|c| c.abs() >= MAX_FLOAT_COORD));
    let coord = if double { "double" } else { "float" };
    let header = format!(
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\n\
         property {coord} x\nproperty {coord} y\nproperty {coord} z\n\
         property uchar red\nproperty uchar green\nproperty uchar blue\n\
         element face {}\nproperty list uchar int vertex_indices\nend_header\n",
        corners.len(),
        mesh.triangles()
    );

    check_cancel(progress)?;
    let io = |e| ExportError::io(path, e);
    let temp = TempPath::new(path);
    let mut w = BufWriter::new(File::create(temp.tmp()).map_err(io)?);
    w.write_all(header.as_bytes()).map_err(io)?;
    let mut row = Vec::with_capacity(27);
    for (p, (sum, n)) in corners.iter().zip(&sums) {
        row.clear();
        for c in p {
            if double {
                row.extend(c.to_le_bytes());
            } else {
                row.extend((*c as f32).to_le_bytes());
            }
        }
        let n = f64::from((*n).max(1));
        row.extend(sum.map(|s| (s / n).round().clamp(0.0, 255.0) as u8));
        w.write_all(&row).map_err(io)?;
    }
    for (page, map) in mesh.pages.iter().zip(&maps) {
        check_cancel(progress)?;
        for tri in &page.indices {
            row.clear();
            row.push(3);
            for &i in tri {
                row.extend((map[i as usize] as i32).to_le_bytes());
            }
            w.write_all(&row).map_err(io)?;
        }
    }
    check_cancel(progress)?;
    let file = w.into_inner().map_err(|e| io(e.into_error()))?;
    drop(file);
    temp.commit().map_err(io)
}

/// The colour of `img` at `uv` (`v` down), bilinear between the texel centres, clamped at
/// the edges.
fn sample(img: &RgbImage, uv: [f32; 2]) -> [f64; 3] {
    let (w, h) = img.dimensions();
    let at = |t: f32, n: u32| {
        let t = if t.is_finite() { f64::from(t) } else { 0.5 };
        (t * f64::from(n) - 0.5).clamp(0.0, f64::from(n - 1))
    };
    let (x, y) = (at(uv[0], w), at(uv[1], h));
    let (x0, y0) = (x.floor() as u32, y.floor() as u32);
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (fx, fy) = (x - f64::from(x0), y - f64::from(y0));
    let px = |x, y| img.get_pixel(x, y).0.map(f64::from);
    let (a, b, c, d) = (px(x0, y0), px(x1, y0), px(x0, y1), px(x1, y1));
    [0, 1, 2].map(|k| {
        let top = a[k] + (b[k] - a[k]) * fx;
        let bottom = c[k] + (d[k] - c[k]) * fx;
        top + (bottom - top) * fy
    })
}
