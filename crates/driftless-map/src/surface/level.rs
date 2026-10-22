//! Seam levelling: evens out the brightness jumps where charts of different images meet.
//!
//! Every chart gets an additive correction in linear RGB per mesh vertex:
//!
//! 1. **Vertex colours.** A chart's colour at a vertex is the mean of the
//!    (2·[`VERTEX_SAMPLE_RADIUS`] + 1)² texels of its rectangle around the vertex's texture
//!    coordinate (linear RGB). The chart owns its rectangle, so no other chart's texels are read.
//! 2. **Border vertices** are vertices of several charts. Each of those charts is pulled to their
//!    mean colour there: for two charts, half of the difference each, in opposite directions.
//! 3. **Interior vertices** start at zero and take [`JACOBI_ROUNDS`] Jacobi rounds of the mean
//!    correction of their edge neighbours within the chart; border vertices stay fixed. The
//!    correction thus fades from the seams into the chart.
//! 4. **Texels.** Every texel of a chart triangle ([`for_each_texel`], first face in face order
//!    wins on shared edges) gets the barycentric mix of its corners' corrections. Then the rest
//!    of the chart's rectangle is filled anew with [`fill_rest`] (mask: the triangle texels), so
//!    no old colour stays in the border. A chart without any correction is left alone.
//!
//! Unseen (gray) faces belong to no chart: they neither give nor take a correction.
//!
//! The seam measure ([`SeamStats`]) follows `seam_jumps` of the point colouring: every mesh
//! edge between two textured faces is sampled in both faces just inside the edge
//! ([`EDGE_SAMPLE`]); the mean absolute luminance difference (sRGB levels) of the pairs from
//! different charts, minus that of the pairs within one chart. 0 if either kind is missing.
//!
//! Memory: per chart its vertices with their corrections (≈ 40 bytes per vertex and chart),
//! per mesh edge between textured faces 16 bytes, and per corrected chart one copy of its
//! rectangle while it is being written.

use super::atlas::{fill_rest, for_each_texel, TexturePage};
use super::{Mesh, TexturedMesh};
use crate::color::{linear_table, luminance, sample_linear, srgb_value};
use rayon::prelude::*;

/// Rounds of Jacobi smoothing that carry the seam corrections into a chart.
pub const JACOBI_ROUNDS: usize = 64;
/// A chart's colour at a vertex: the mean of the texels within this many texels (per axis).
pub const VERTEX_SAMPLE_RADIUS: i64 = 1;
/// Where the seam measure samples a face next to one of its edges: barycentric weights of the
/// edge's two corners and of the opposite corner.
pub const EDGE_SAMPLE: [f64; 3] = [0.45, 0.45, 0.1];
/// "No chart".
const NONE: u32 = u32::MAX;

/// Mean sRGB jump along chart borders before and after levelling (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SeamStats {
    pub seam_jump_before: f64,
    pub seam_jump_after: f64,
}

/// A mesh edge between two textured faces: the faces and the edge's two vertices.
type EdgePair = [u32; 4];

/// A chart's vertices: (mesh vertex, page vertex), ascending by mesh vertex.
type ChartVertices = Vec<(u32, u32)>;

/// Levels the seams between charts of `tex` (see the module docs). `labels` (the image per
/// face from `choose_labels`) must fit the mesh; the charts in `tex` already carry them.
/// Same input, byte-identical pages.
pub(crate) fn level_seams(
    mesh: &Mesh,
    labels: &[Option<u32>],
    tex: &mut TexturedMesh,
) -> SeamStats {
    let n = mesh.indices.len();
    assert_eq!(labels.len(), n, "labels do not fit the mesh");
    assert_eq!(tex.face_ref.len(), n, "texture does not fit the mesh");
    let lin = linear_table();
    let mut chart_of = vec![NONE; n];
    for (c, chart) in tex.charts.iter().enumerate() {
        for &f in &chart.faces {
            chart_of[f as usize] = c as u32;
        }
    }
    let edges = edge_pairs(mesh, &chart_of);
    let seam_jump_before = seam_jump(mesh, tex, &chart_of, &edges, &lin);

    // 1. Every chart's vertices and its colour at each.
    let verts: Vec<ChartVertices> = tex
        .charts
        .par_iter()
        .map(|chart| chart_vertices(mesh, tex, &chart.faces))
        .collect();
    let colours: Vec<Vec<[f64; 3]>> = tex
        .charts
        .par_iter()
        .zip(&verts)
        .map(|(chart, vs)| {
            let page = &tex.pages[chart.page as usize];
            vs.iter()
                .map(|&(_, pv)| vertex_colour(page, chart.rect, page.uvs[pv as usize], &lin))
                .collect()
        })
        .collect();

    // 2. Border vertices: (mesh vertex, chart, local index), grouped by mesh vertex.
    let mut entries: Vec<(u32, u32, u32)> = verts
        .iter()
        .enumerate()
        .flat_map(|(c, vs)| {
            vs.iter()
                .enumerate()
                .map(move |(l, &(v, _))| (v, c as u32, l as u32))
        })
        .collect();
    entries.par_sort_unstable();
    let mut fixed: Vec<Vec<(u32, [f64; 3])>> = vec![Vec::new(); tex.charts.len()];
    let mut i = 0;
    while i < entries.len() {
        let mut j = i + 1;
        while j < entries.len() && entries[j].0 == entries[i].0 {
            j += 1;
        }
        if j - i >= 2 {
            let group = &entries[i..j];
            let mut mean = [0.0; 3];
            for &(_, c, l) in group {
                let col = colours[c as usize][l as usize];
                for k in 0..3 {
                    mean[k] += col[k];
                }
            }
            let mean = mean.map(|v| v / group.len() as f64);
            for &(_, c, l) in group {
                let col = colours[c as usize][l as usize];
                fixed[c as usize].push((l, [0, 1, 2].map(|k| mean[k] - col[k])));
            }
        }
        i = j;
    }
    drop(entries);
    drop(colours);

    // 3. Corrections per chart vertex; `None` for a chart without any.
    let corrections: Vec<Option<Vec<[f64; 3]>>> = tex
        .charts
        .par_iter()
        .zip(&verts)
        .zip(&fixed)
        .map(|((chart, vs), fx)| {
            if fx.iter().all(|(_, d)| d.iter().all(|&x| x == 0.0)) {
                return None;
            }
            Some(smooth(mesh, &chart.faces, vs, fx))
        })
        .collect();
    drop(fixed);

    // 4. Texels, page by page (pages are independent; charts in order within a page).
    let charts = &tex.charts;
    let face_ref = &tex.face_ref;
    tex.pages.par_iter_mut().enumerate().for_each(|(p, page)| {
        for (c, chart) in charts.iter().enumerate() {
            if chart.page as usize != p {
                continue;
            }
            if let Some(delta) = &corrections[c] {
                apply(
                    mesh,
                    page,
                    chart.rect,
                    &chart.faces,
                    face_ref,
                    &verts[c],
                    delta,
                    &lin,
                );
            }
        }
    });

    let seam_jump_after = seam_jump(mesh, tex, &chart_of, &edges, &lin);
    SeamStats {
        seam_jump_before,
        seam_jump_after,
    }
}

/// Every mesh edge between two textured faces (each unordered face pair of an edge once),
/// in edge order.
fn edge_pairs(mesh: &Mesh, chart_of: &[u32]) -> Vec<EdgePair> {
    let mut edges: Vec<(u32, u32, u32)> = mesh
        .indices
        .iter()
        .enumerate()
        .filter(|&(f, _)| chart_of[f] != NONE)
        .flat_map(|(f, t)| {
            (0..3).map(move |k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                (a.min(b), a.max(b), f as u32)
            })
        })
        .collect();
    edges.par_sort_unstable();
    let mut pairs = Vec::new();
    let mut i = 0;
    while i < edges.len() {
        let mut j = i + 1;
        while j < edges.len() && edges[j].0 == edges[i].0 && edges[j].1 == edges[i].1 {
            j += 1;
        }
        for (x, a) in edges[i..j].iter().enumerate() {
            for b in &edges[i + x + 1..j] {
                if a.2 != b.2 {
                    pairs.push([a.2, b.2, a.0, a.1]);
                }
            }
        }
        i = j;
    }
    pairs
}

/// The seam measure over `edges` (see the module docs).
fn seam_jump(
    mesh: &Mesh,
    tex: &TexturedMesh,
    chart_of: &[u32],
    edges: &[EdgePair],
    lin: &[f32; 256],
) -> f64 {
    let jumps: Vec<(f64, bool)> = edges
        .par_iter()
        .map(|&[f, g, a, b]| {
            let ya = edge_luminance(mesh, tex, f, a, b, lin);
            let yb = edge_luminance(mesh, tex, g, a, b, lin);
            (
                (ya - yb).abs(),
                chart_of[f as usize] != chart_of[g as usize],
            )
        })
        .collect();
    let mut sums = [(0.0, 0usize); 2];
    for &(d, across) in &jumps {
        let s = &mut sums[usize::from(across)];
        s.0 += d;
        s.1 += 1;
    }
    let [same, diff] = sums;
    if same.1 == 0 || diff.1 == 0 {
        0.0
    } else {
        diff.0 / diff.1 as f64 - same.0 / same.1 as f64
    }
}

/// Luminance (sRGB levels) of face `f`'s texture at [`EDGE_SAMPLE`] next to its edge `a`–`b`.
fn edge_luminance(
    mesh: &Mesh,
    tex: &TexturedMesh,
    f: u32,
    a: u32,
    b: u32,
    lin: &[f32; 256],
) -> f64 {
    let (p, t) = tex.face_ref[f as usize];
    let page = &tex.pages[p as usize];
    let tri = page.indices[t as usize];
    let corners = mesh.indices[f as usize];
    let mut uv = [0.0f64; 2];
    for k in 0..3 {
        let w = if corners[k] == a {
            EDGE_SAMPLE[0]
        } else if corners[k] == b {
            EDGE_SAMPLE[1]
        } else {
            EDGE_SAMPLE[2]
        };
        let q = page.uvs[tri[k] as usize];
        uv[0] += w * f64::from(q[0]);
        uv[1] += w * f64::from(q[1]);
    }
    // Pixel coordinates with centres at integers.
    let px = [
        (uv[0] * f64::from(page.image.width) - 0.5) as f32,
        (uv[1] * f64::from(page.image.height) - 0.5) as f32,
    ];
    srgb_value(luminance(sample_linear(&page.image, px, lin)) as f32)
}

/// A chart's vertices (see [`ChartVertices`]).
fn chart_vertices(mesh: &Mesh, tex: &TexturedMesh, faces: &[u32]) -> ChartVertices {
    let mut vs: ChartVertices = faces
        .iter()
        .flat_map(|&f| {
            let (p, t) = tex.face_ref[f as usize];
            let tri = tex.pages[p as usize].indices[t as usize];
            let corners = mesh.indices[f as usize];
            (0..3).map(move |k| (corners[k], tri[k]))
        })
        .collect();
    vs.sort_unstable();
    vs.dedup_by_key(|v| v.0);
    vs
}

/// Local index of mesh vertex `v` in `vs`.
fn local(vs: &ChartVertices, v: u32) -> usize {
    vs.binary_search_by_key(&v, |x| x.0)
        .expect("vertex of a chart face")
}

/// Mean linear colour of the texels of `rect` within [`VERTEX_SAMPLE_RADIUS`] of `uv`.
fn vertex_colour(page: &TexturePage, rect: [u32; 4], uv: [f32; 2], lin: &[f32; 256]) -> [f64; 3] {
    let width = page.image.width;
    let centre = [
        (f64::from(uv[0]) * f64::from(width)).floor() as i64,
        (f64::from(uv[1]) * f64::from(page.image.height)).floor() as i64,
    ];
    let [x, y, w, h] = rect.map(i64::from);
    let (mut sum, mut count) = ([0.0f64; 3], 0u32);
    let r = VERTEX_SAMPLE_RADIUS;
    for j in (centre[1] - r).max(y)..=(centre[1] + r).min(y + h - 1) {
        for i in (centre[0] - r).max(x)..=(centre[0] + r).min(x + w - 1) {
            let o = ((j as usize) * width as usize + i as usize) * 3;
            for c in 0..3 {
                sum[c] += f64::from(lin[page.image.data[o + c] as usize]);
            }
            count += 1;
        }
    }
    if count == 0 {
        return [0.0; 3];
    }
    sum.map(|s| s / f64::from(count))
}

/// The corrections of a chart's vertices: `fixed` at border vertices, [`JACOBI_ROUNDS`] Jacobi
/// rounds over the chart's edges elsewhere.
fn smooth(
    mesh: &Mesh,
    faces: &[u32],
    vs: &ChartVertices,
    fixed: &[(u32, [f64; 3])],
) -> Vec<[f64; 3]> {
    let m = vs.len();
    let mut pairs: Vec<(u32, u32)> = faces
        .iter()
        .flat_map(|&f| {
            let t = mesh.indices[f as usize].map(|v| local(vs, v) as u32);
            (0..3).flat_map(move |k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                [(a, b), (b, a)]
            })
        })
        .collect();
    pairs.sort_unstable();
    pairs.dedup();
    let mut start = vec![0usize; m + 1];
    for &(a, _) in &pairs {
        start[a as usize + 1] += 1;
    }
    for v in 0..m {
        start[v + 1] += start[v];
    }
    let mut is_fixed = vec![false; m];
    let mut delta = vec![[0.0f64; 3]; m];
    for &(l, d) in fixed {
        is_fixed[l as usize] = true;
        delta[l as usize] = d;
    }
    let mut next = delta.clone();
    for _ in 0..JACOBI_ROUNDS {
        for v in 0..m {
            if is_fixed[v] || start[v] == start[v + 1] {
                continue;
            }
            let around = &pairs[start[v]..start[v + 1]];
            let mut sum = [0.0; 3];
            for &(_, u) in around {
                for k in 0..3 {
                    sum[k] += delta[u as usize][k];
                }
            }
            next[v] = sum.map(|s| s / around.len() as f64);
        }
        std::mem::swap(&mut delta, &mut next);
        next.copy_from_slice(&delta);
    }
    delta
}

/// Adds the corrections `delta` (per vertex of `vs`) to the triangle texels of a chart and
/// fills the rest of its rectangle anew.
#[allow(clippy::too_many_arguments)]
fn apply(
    mesh: &Mesh,
    page: &mut TexturePage,
    rect: [u32; 4],
    faces: &[u32],
    face_ref: &[(u32, u32)],
    vs: &ChartVertices,
    delta: &[[f64; 3]],
    lin: &[f32; 256],
) {
    let [x, y, w, h] = rect;
    let (wu, hu) = (w as usize, h as usize);
    let size = [page.image.width, page.image.height].map(f64::from);
    let stride = page.image.width as usize;
    let mut buf = vec![0u8; wu * hu * 3];
    for r in 0..hu {
        let o = ((y as usize + r) * stride + x as usize) * 3;
        buf[r * wu * 3..(r + 1) * wu * 3].copy_from_slice(&page.image.data[o..o + wu * 3]);
    }
    let mut done = vec![false; wu * hu];
    for &f in faces {
        let (_, t) = face_ref[f as usize];
        let tri = page.indices[t as usize];
        let corners = [0, 1, 2].map(|k| {
            let uv = page.uvs[tri[k] as usize];
            [
                f64::from(uv[0]) * size[0] - f64::from(x),
                f64::from(uv[1]) * size[1] - f64::from(y),
            ]
        });
        let d = mesh.indices[f as usize].map(|v| delta[local(vs, v)]);
        for_each_texel(corners, w, h, |i, j, bary| {
            let at = j as usize * wu + i as usize;
            if done[at] {
                return;
            }
            done[at] = true;
            for c in 0..3 {
                let add = (0..3).map(|k| bary[k] * d[k][c]).sum::<f64>();
                let l = f64::from(lin[buf[at * 3 + c] as usize]) + add;
                buf[at * 3 + c] = srgb_value(l as f32).round().clamp(0.0, 255.0) as u8;
            }
        });
    }
    fill_rest(&mut buf, &mut done, wu, hu);
    for r in 0..hu {
        let o = ((y as usize + r) * stride + x as usize) * 3;
        page.image.data[o..o + wu * 3].copy_from_slice(&buf[r * wu * 3..(r + 1) * wu * 3]);
    }
}

#[cfg(test)]
mod tests {
    use super::super::atlas::{ChartInfo, TexturePage, MARGIN, UNSEEN_GRAY};
    use super::super::testutil::open_wall;
    use super::*;
    use crate::color::{linear_table, srgb_value};
    use driftless_io::images::RgbImage;

    /// Wall cells per side, and texels per cell edge.
    const CELLS: usize = 12;
    const TEXELS: u32 = 4;
    const PAGE: u32 = 128;

    /// An `open_wall` of [`CELLS`]² cells, split at `iy = CELLS / 2` into two charts on one
    /// page, laid out as `bake` would (shared page vertices per chart, [`MARGIN`] border).
    /// Chart 0 (lower `iy`) is filled with `left`, chart 1 with `right`; the rest of the page
    /// is [`UNSEEN_GRAY`].
    fn two_charts(left: [u8; 3], right: [u8; 3]) -> (Mesh, Vec<Option<u32>>, TexturedMesh) {
        let mesh = open_wall(CELLS, 0.1);
        let half = CELLS / 2;
        let mut page = TexturePage {
            positions: Vec::new(),
            uvs: Vec::new(),
            indices: Vec::new(),
            image: RgbImage {
                width: PAGE,
                height: PAGE,
                data: vec![UNSEEN_GRAY; (PAGE * PAGE * 3) as usize],
            },
        };
        let n = mesh.indices.len();
        let mut tex = TexturedMesh {
            pages: Vec::new(),
            face_ref: vec![(0, 0); n],
            textured: vec![true; n],
            charts: Vec::new(),
        };
        let labels: Vec<Option<u32>> = (0..n)
            .map(|f| Some(u32::from(f / 2 / CELLS >= half)))
            .collect();
        let w = half as u32 * TEXELS + 1 + 2 * MARGIN;
        let h = CELLS as u32 * TEXELS + 1 + 2 * MARGIN;
        for (c, colour) in [left, right].into_iter().enumerate() {
            let origin = [c as u32 * (w + 1), 0];
            let mut local = vec![u32::MAX; mesh.positions.len()];
            let faces: Vec<u32> = (0..n as u32)
                .filter(|&f| labels[f as usize] == Some(c as u32))
                .collect();
            for &f in &faces {
                let mut tri = [0u32; 3];
                for (k, &v) in mesh.indices[f as usize].iter().enumerate() {
                    if local[v as usize] == u32::MAX {
                        local[v as usize] = page.positions.len() as u32;
                        let (iy, iz) = (v as usize / (CELLS + 1), v as usize % (CELLS + 1));
                        let tx = origin[0] as f32
                            + (MARGIN as f32 + 0.5)
                            + ((iy - c * half) as u32 * TEXELS) as f32;
                        let ty =
                            origin[1] as f32 + (MARGIN as f32 + 0.5) + (iz as u32 * TEXELS) as f32;
                        page.positions.push(mesh.positions[v as usize]);
                        page.uvs.push([tx / PAGE as f32, ty / PAGE as f32]);
                    }
                    tri[k] = local[v as usize];
                }
                tex.face_ref[f as usize] = (0, page.indices.len() as u32);
                page.indices.push(tri);
            }
            for y in origin[1]..origin[1] + h {
                for x in origin[0]..origin[0] + w {
                    let o = ((y * PAGE + x) * 3) as usize;
                    page.image.data[o..o + 3].copy_from_slice(&colour);
                }
            }
            tex.charts.push(ChartInfo {
                page: 0,
                rect: [origin[0], origin[1], w, h],
                faces,
            });
        }
        tex.pages.push(page);
        (mesh, labels, tex)
    }

    /// `rgb` (sRGB) brightened by `factor` in linear light.
    fn brighter(rgb: [u8; 3], factor: f32) -> [u8; 3] {
        let lin = linear_table();
        rgb.map(|v| srgb_value(lin[v as usize] * factor).round() as u8)
    }

    fn texel(tex: &TexturedMesh, x: u32, y: u32) -> [u8; 3] {
        let o = ((y * PAGE + x) * 3) as usize;
        let d = &tex.pages[0].image.data;
        [d[o], d[o + 1], d[o + 2]]
    }

    #[test]
    fn seam_jump_drops() {
        let left = [120, 110, 100];
        let right = brighter(left, 1.2);
        let (mesh, labels, mut tex) = two_charts(left, right);
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(
            stats.seam_jump_before > 5.0,
            "Sprung vorher {}",
            stats.seam_jump_before
        );
        assert!(
            stats.seam_jump_after < 0.3 * stats.seam_jump_before,
            "Sprung nachher {} (vorher {})",
            stats.seam_jump_after,
            stats.seam_jump_before
        );
        // Both sides of the seam meet in the middle, and the border texels of chart 1 next to
        // the seam (refilled, owned by no triangle) carry the corrected colour, not the old one.
        let [x1, _, _, h] = tex.charts[1].rect;
        let mid = h / 2;
        let lin = linear_table();
        let y = |rgb: [u8; 3]| crate::color::luminance(rgb.map(|v| lin[v as usize]));
        let (yl, yr) = (y(left), y(right));
        for x in [x1, x1 + MARGIN, x1 + MARGIN + 1] {
            let got = y(texel(&tex, x, mid));
            assert!(
                (got - yr).abs() > 0.3 * (yr - yl),
                "Texel ({x}, {mid}) noch alt: {got} (links {yl}, rechts {yr})"
            );
            assert!(got > yl && got < yr, "Texel ({x}, {mid}): {got}");
        }
        // Texels of no chart stay gray.
        let [x0, _, w0, _] = tex.charts[0].rect;
        assert_eq!(texel(&tex, x0 + w0, 0), [UNSEEN_GRAY; 3]);
        // Deterministic.
        let (_, _, mut again) = two_charts(left, right);
        assert_eq!(level_seams(&mesh, &labels, &mut again), stats);
        assert_eq!(again, tex);
    }

    #[test]
    fn uniform_untouched() {
        let colour = [90, 140, 200];
        // Two charts of the same colour meet without a seam.
        let (mesh, labels, mut tex) = two_charts(colour, colour);
        let before = tex.clone();
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert_eq!(tex, before);
        assert!(stats.seam_jump_before.abs() < 1e-9, "{stats:?}");
        assert!(stats.seam_jump_after.abs() < 1e-9, "{stats:?}");
    }
}
