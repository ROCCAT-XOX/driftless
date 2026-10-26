//! Texture atlas: one camera image per face, charts, texture pages.
//!
//! 1. **Labels** ([`choose_labels`]): every face starts with its best view; then
//!    [`ICM_ROUNDS`] rounds of ICM visit the faces in index order and give each the candidate
//!    image (one of its kept views) of least cost `−sqrt(score / best score of the face) +
//!    smooth_weight · (edge neighbours with another image)`, so that neighbours share an image
//!    where the images are about equally good (the square root: the ratio of the linear
//!    resolutions, as the score grows with the projected area). Faces without a view keep no
//!    label. Then small charts are merged ([`merge_small_charts`]): a chart of fewer than
//!    [`MERGE_FACES`] faces takes the image of a neighbouring chart (the one with the most
//!    shared edges first) if that image sees all its faces with at least [`MERGE_MIN_SCORE`] of
//!    their best score. ICM alone cannot do this: inside a small chart every face agrees with
//!    its neighbours. Each chart costs a border on the page and a seam around it.
//!    Then islands ([`absorb_islands`]): a chart of fewer than [`ISLAND_FACES`] faces takes the
//!    image of the neighbouring chart with the most shared edges whose camera sees all its faces
//!    by geometry alone (inside the image rim, within the colour range, facing the camera; no
//!    depth test, no view scores), where [`merge_small_charts`] finds no image with a recorded
//!    view of them all. In the dark room at 1 cm this cut the charts from 192,456 to 79,750
//!    and halved the seam probes.
//! 2. **Charts** ([`bake`]): connected faces (over shared edges) with the same label. A chart is
//!    the image region its faces project to (full resolution, [`Camera::project`]): the
//!    smallest rectangle around it ([`best_axis`]: one side along an edge of the region's convex
//!    hull, the long side first), scaled by `s ≤ 1` where the image is finer than
//!    [`SurfaceConfig::min_texel`] on the surface (`s = texel / min_texel`, texel =
//!    `sqrt(surface area / image area)` of the chart) or the rectangle would not fit a page,
//!    plus a border of [`MARGIN`] texels on every side. A chart that fills less than
//!    [`MIN_CHART_FILL`] of its rectangle is cut at quantiles of its face centroids where the parts
//!    need fewer texels than the whole ([`split_chart`]); [`TextureStats::fill_ratio`] and
//!    [`TextureStats::texels_total`] show how well the rectangles are used.
//! 3. **Packing** ([`pack`]): shelves on pages of `page_size²`, charts sorted by height (then
//!    width, then index). The gray block of the unseen faces ([`GRAY_BLOCK`]², filled with
//!    [`UNSEEN_GRAY`]) comes first, at the origin of page 0.
//! 4. **Filling**: a second pass over the images at 1/[`ColorConfig::downscale`]. Every texel
//!    within half a texel of a chart triangle ([`for_each_texel`]) gets the bilinear image
//!    colour at its spot (a scaled-down chart: the mean of up to [`MAX_SUPERSAMPLE`]² samples
//!    over the texel's footprint), divided (linear RGB) by the image's gain and the camera's edge
//!    falloff at the texel's angle from the optical axis (interpolated over the triangle),
//!    as sRGB. Then [`fill_rest`]: [`MARGIN`] rounds of dilation, and every texel of the
//!    rectangle still empty gets the chart's mean colour, so no gray bleeds into a chart at
//!    coarser mip levels. Texels of no chart stay [`UNSEEN_GRAY`].
//!
//! Unseen faces have no intensity (the mesh carries none), so they get one neutral gray:
//! [`UNSEEN_GRAY`], the gray `colorize` gives a point of half the cloud's 99th-percentile
//! intensity (the middle of [`GRAY_MIN`] … 255).
//!
//! Texture coordinates: `uv` in 0…1 with `v` pointing down (row 0 of the page image at
//! `v = 0`); texel `(i, j)` covers `[i, i+1) × [j, j+1)` in `uv · page_size` and has its
//! centre at `(i + ½, j + ½)`. Within a chart the map from texel to image coordinates is
//! affine: a rotation onto the rectangle's sides (`axis`), then `min + (texel centre − MARGIN −
//! ½) / s` along them.
//!
//! For later steps (seam levelling): `TexturedMesh::face_ref[f]` is face `f`'s triangle
//! (page, index in `pages[page].indices`); `TexturedMesh::textured[f]` says whether an image
//! textures it (an unseen face's triangle sits, with zero texture area, at the centre of the
//! gray block on page 0); `TexturedMesh::charts` lists every chart's page, rectangle (border
//! included, owned by the chart alone) and faces. The texels of a face are those
//! [`for_each_texel`] visits for its corners `uv · page_size`; after changing them, redo the
//! rest of the chart rectangle with [`fill_rest`] (mask: the texels of its faces).
//!
//! Memory: the pages (3 bytes per texel), the per-face projection (≈ 40 bytes per face), and
//! per image only its charts' rectangles; images are streamed one at a time.
//!
//! [`ColorConfig::downscale`]: crate::ColorConfig::downscale

use super::views::{FaceView, TextureSource, ViewTable};
use super::{Mesh, SurfaceConfig};
use crate::color::{frame_camera, image_count, linear_table, sample_linear, srgb_value, GRAY_MIN};
use crate::exposure::Vignetting;
use crate::MapError;
#[cfg(doc)]
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, RgbImage};
use driftless_io::progress::Progress;
use nalgebra::{Isometry3, Point3, Vector3};
use rayon::prelude::*;

/// Rounds of ICM over the face labels.
pub const ICM_ROUNDS: usize = 5;
/// Charts of fewer faces are merged into a neighbouring chart where its image sees them all.
pub const MERGE_FACES: usize = 64;
/// … with a view of at least this share of each face's best score.
pub const MERGE_MIN_SCORE: f32 = 0.25;
/// Rounds of merging.
pub const MERGE_ROUNDS: usize = 4;
/// A chart of fewer faces is an island (see [`absorb_islands`]).
pub const ISLAND_FACES: usize = 8;
/// Border around every chart (texels), filled by dilation.
pub const MARGIN: u32 = 2;
/// Edge of the gray block of the unseen faces (texels), at the origin of page 0.
pub const GRAY_BLOCK: u32 = 8;
/// Gray of unseen faces and of texels no chart covers: the middle of [`GRAY_MIN`] … 255.
pub const UNSEEN_GRAY: u8 = GRAY_MIN + (255 - GRAY_MIN) / 2;
/// A texel of a scaled-down chart averages at most this many samples per axis.
pub const MAX_SUPERSAMPLE: u32 = 4;
/// A chart of several faces whose triangles cover less than this share of its rectangle
/// (border left out) is cut in two, and so on (see [`split_chart`]); a cut is kept where the
/// parts together need fewer texels than the whole, borders included. An L-shaped or patchy
/// chart would otherwise take its whole bounding box on the page, while splitting a small
/// chart only adds borders.
pub const MIN_CHART_FILL: f64 = 0.95;
/// A chart is cut at one of this many − 1 quantiles of its face centroids along either side
/// of its rectangle (see [`split_chart`]).
pub const SPLIT_CUTS: usize = 16;
/// The charts of one image are filled in batches of about this many texels (4 bytes each while
/// they are filled).
pub const FILL_BATCH_TEXELS: u64 = 16 << 20;
/// "No chart" / "not yet assigned".
const NONE: u32 = u32::MAX;

/// One texture page: its triangles (own vertex list) and its image.
#[derive(Debug, Clone, PartialEq)]
pub struct TexturePage {
    /// World positions (m).
    pub positions: Vec<[f32; 3]>,
    /// Texture coordinates, 0…1, `v` pointing down (row 0 of `image` at `v = 0`).
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<[u32; 3]>,
    /// `page_size²`, sRGB.
    pub image: RgbImage,
}

/// A mesh with its texture, split by texture page.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TexturedMesh {
    pub pages: Vec<TexturePage>,
    /// Per face of the input mesh: (page, triangle index within that page). Unseen faces
    /// point at their triangle in page 0, whose corners all lie at the gray block's centre.
    pub(crate) face_ref: Vec<(u32, u32)>,
    /// Per face of the input mesh: whether an image textures it (else it is gray).
    pub(crate) textured: Vec<bool>,
    /// Every chart: its page, rectangle and faces.
    pub(crate) charts: Vec<ChartInfo>,
}

/// A chart on its page.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ChartInfo {
    pub page: u32,
    /// `[x, y, w, h]` in texels, the border of [`MARGIN`] texels included. The chart owns
    /// every texel of it.
    pub rect: [u32; 4],
    /// Its faces (indices into the input mesh), ascending.
    pub faces: Vec<u32>,
}

/// What baking did.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TextureStats {
    pub faces_textured: usize,
    /// Faces no image shows: gray.
    pub faces_untextured: usize,
    pub charts: usize,
    pub pages: usize,
    /// Median over the textured faces of the texel size on the surface (mm).
    pub texel_median_mm: f64,
    /// Texels of all chart rectangles (borders included).
    pub texels_total: u64,
    /// Share of those texels that triangles cover (before the border is filled).
    pub fill_ratio: f64,
    /// Tiny label islands (fewer than `ISLAND_FACES` faces) merged into a neighbouring chart
    /// before baking; set by `texture_mesh`, 0 from `bake`.
    pub islands_absorbed: usize,
}

/// Edge neighbours of every face (faces sharing an edge, either winding), in CSR form.
struct Neighbours {
    start: Vec<u32>,
    list: Vec<u32>,
}

impl Neighbours {
    fn new(mesh: &Mesh) -> Neighbours {
        let mut edges: Vec<(u32, u32, u32)> = mesh
            .indices
            .iter()
            .enumerate()
            .flat_map(|(f, t)| {
                (0..3).map(move |k| {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    (a.min(b), a.max(b), f as u32)
                })
            })
            .collect();
        edges.par_sort_unstable();
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        let mut i = 0;
        while i < edges.len() {
            let mut j = i + 1;
            while j < edges.len() && edges[j].0 == edges[i].0 && edges[j].1 == edges[i].1 {
                j += 1;
            }
            for a in &edges[i..j] {
                for b in &edges[i..j] {
                    if a.2 != b.2 {
                        pairs.push((a.2, b.2));
                    }
                }
            }
            i = j;
        }
        drop(edges);
        pairs.par_sort_unstable();
        pairs.dedup();
        let n = mesh.indices.len();
        let mut start = vec![0u32; n + 1];
        for &(a, _) in &pairs {
            start[a as usize + 1] += 1;
        }
        for f in 0..n {
            start[f + 1] += start[f];
        }
        Neighbours {
            start,
            list: pairs.iter().map(|p| p.1).collect(),
        }
    }

    fn of(&self, f: usize) -> &[u32] {
        &self.list[self.start[f] as usize..self.start[f + 1] as usize]
    }
}

/// One image (by rank) per face, or `None` if no image shows it (see the module docs).
/// Deterministic: faces are visited in index order, ties go to the better view.
pub(crate) fn choose_labels(
    mesh: &Mesh,
    views: &ViewTable,
    cfg: &SurfaceConfig,
) -> Vec<Option<u32>> {
    let k = cfg.views_per_face;
    let n = mesh.indices.len();
    assert_eq!(views.per_face.len(), n * k, "views do not fit the mesh");
    let slots = |f: usize| &views.per_face[f * k..(f + 1) * k];
    let mut labels: Vec<Option<u32>> = (0..n)
        .map(|f| {
            let v = slots(f)[0];
            (v.score > 0.0).then_some(v.image)
        })
        .collect();
    if cfg.smooth_weight == 0.0 {
        return labels;
    }
    let nb = Neighbours::new(mesh);
    for _ in 0..ICM_ROUNDS {
        let mut changed = false;
        for f in 0..n {
            let s = slots(f);
            let best = s[0].score;
            if best <= 0.0 {
                continue;
            }
            let mut pick = (f32::INFINITY, labels[f]);
            for v in s.iter().take_while(|v| v.score > 0.0) {
                let differ = nb
                    .of(f)
                    .iter()
                    .filter(|&&g| labels[g as usize] != Some(v.image))
                    .count();
                let cost = -(v.score / best).sqrt() + cfg.smooth_weight * differ as f32;
                if cost < pick.0 {
                    pick = (cost, Some(v.image));
                }
            }
            if pick.1 != labels[f] {
                labels[f] = pick.1;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    merge_small_charts(&mut labels, &nb, slots);
    labels
}

/// Connected faces (over edges) with the same label: per face its chart (`NONE` without a
/// label) and the faces of every chart (CSR: `start`, `faces`; ascending within a chart),
/// the charts in the order of their first face.
fn connected(
    nb: &Neighbours,
    n: usize,
    label: impl Fn(usize) -> Option<u32>,
) -> (Vec<u32>, Vec<usize>, Vec<u32>) {
    let mut chart_of = vec![NONE; n];
    let mut faces: Vec<u32> = Vec::new();
    let mut start: Vec<usize> = vec![0];
    let mut stack: Vec<u32> = Vec::new();
    for f in 0..n {
        let Some(l) = label(f) else {
            continue;
        };
        if chart_of[f] != NONE {
            continue;
        }
        let c = (start.len() - 1) as u32;
        chart_of[f] = c;
        stack.push(f as u32);
        let first = faces.len();
        while let Some(g) = stack.pop() {
            faces.push(g);
            for &h in nb.of(g as usize) {
                let h_ = h as usize;
                if chart_of[h_] == NONE && label(h_) == Some(l) {
                    chart_of[h_] = c;
                    stack.push(h);
                }
            }
        }
        faces[first..].sort_unstable();
        start.push(faces.len());
    }
    (chart_of, start, faces)
}

/// Merges charts of fewer than [`MERGE_FACES`] faces into a neighbouring chart (see the
/// module docs): smallest first (then in face order), into the neighbouring image with the
/// most shared edges whose views of all the chart's faces score at least [`MERGE_MIN_SCORE`]
/// of their best; up to [`MERGE_ROUNDS`] rounds.
fn merge_small_charts<'a>(
    labels: &mut [Option<u32>],
    nb: &Neighbours,
    slots: impl Fn(usize) -> &'a [FaceView],
) {
    let n = labels.len();
    let mut around: Vec<u32> = Vec::new();
    for _ in 0..MERGE_ROUNDS {
        let (chart_of, start, faces) = connected(nb, n, |f| labels[f]);
        let size = |c: usize| start[c + 1] - start[c];
        let mut small: Vec<usize> = (0..start.len() - 1)
            .filter(|&c| size(c) < MERGE_FACES)
            .collect();
        small.sort_by_key(|&c| (size(c), c));
        let mut changed = false;
        for c in small {
            let own = &faces[start[c]..start[c + 1]];
            let label = labels[own[0] as usize];
            // The neighbouring images, by shared edges (most first, then the lower rank).
            around.clear();
            for &f in own {
                for &g in nb.of(f as usize) {
                    if chart_of[g as usize] != c as u32 {
                        if let Some(l) = labels[g as usize].filter(|&l| Some(l) != label) {
                            around.push(l);
                        }
                    }
                }
            }
            around.sort_unstable();
            let mut counted: Vec<(usize, u32)> = around
                .chunk_by(|a, b| a == b)
                .map(|run| (run.len(), run[0]))
                .collect();
            counted.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
            let sees = |f: u32, l: u32| {
                let s = slots(f as usize);
                s.iter().any(|v| {
                    v.score > 0.0 && v.image == l && v.score >= MERGE_MIN_SCORE * s[0].score
                })
            };
            if let Some(&(_, l)) = counted
                .iter()
                .find(|&&(_, l)| own.iter().all(|&f| sees(f, l)))
            {
                for &f in own {
                    labels[f as usize] = Some(l);
                }
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

/// Places rectangles `sizes` (`[w, h]`, each at most `page` on a side) on pages of `page²`:
/// shelves filled left to right, the rectangles sorted by height, then width (both
/// descending), then index. With `keep_first`, rectangle 0 comes first, at the origin of
/// page 0. A shelf grows to its tallest rectangle while it is the page's last one. Returns
/// per rectangle (page, top-left corner), and the number of pages.
fn pack(sizes: &[[u32; 2]], page: u32, keep_first: bool) -> (Vec<(u32, [u32; 2])>, u32) {
    let skip = usize::from(keep_first && !sizes.is_empty());
    let mut order: Vec<usize> = (skip..sizes.len()).collect();
    order.sort_by(|&a, &b| {
        sizes[b][1]
            .cmp(&sizes[a][1])
            .then(sizes[b][0].cmp(&sizes[a][0]))
            .then(a.cmp(&b))
    });
    let mut at = vec![(0, [0, 0]); sizes.len()];
    let (mut pg, mut x, mut y, mut shelf) = (0u32, 0u32, 0u32, 0u32);
    for i in (0..skip).chain(order) {
        let [w, h] = sizes[i];
        assert!(w <= page && h <= page, "rectangle larger than a page");
        if x + w > page || y + shelf.max(h) > page {
            y += shelf;
            x = 0;
            shelf = 0;
            if y + h > page {
                pg += 1;
                y = 0;
            }
        }
        at[i] = (pg, [x, y]);
        x += w;
        shelf = shelf.max(h);
    }
    let pages = if sizes.is_empty() { 0 } else { pg + 1 };
    (at, pages)
}

/// Islands take the image of their surroundings: a chart of fewer than [`ISLAND_FACES`] faces
/// (typically a face no view reached, or one ICM left alone between its neighbours) takes the
/// image of the neighbouring chart with the most shared edges (then the lowest image index)
/// whose camera sees every one of its faces geometrically: all corners project into the image
/// within [`ColorConfig::rim_theta`] of the optical axis, the centroid is within
/// [`ColorConfig::max_dist`] of the camera, and the area-weighted summed normal of the island and
/// of the faces around it (other charts) faces the camera. There is no depth test and no
/// per-face facing test (the views are not read). Charts are taken in chart order, from the
/// labels as they stood at the start of the round; up to [`MERGE_ROUNDS`] rounds, ending at a
/// round that changes nothing. Returns the number of islands taken over all rounds.
///
/// [`ColorConfig::rim_theta`]: crate::ColorConfig::rim_theta
/// [`ColorConfig::max_dist`]: crate::ColorConfig::max_dist
pub(crate) fn absorb_islands(
    mesh: &Mesh,
    views: &ViewTable,
    cams: &[crate::ColorCamera],
    cfg: &SurfaceConfig,
    labels: &mut [Option<u32>],
) -> usize {
    let n = labels.len();
    let nb = Neighbours::new(mesh);
    let w2c: Vec<Isometry3<f64>> = views
        .images
        .iter()
        .map(|i| i.world_from_cam.inverse())
        .collect();
    let mut total = 0;
    for _ in 0..MERGE_ROUNDS {
        let (chart_of, start, faces) = connected(&nb, n, |f| labels[f]);
        let small: Vec<usize> = (0..start.len() - 1)
            .filter(|&c| start[c + 1] - start[c] < ISLAND_FACES)
            .collect();
        let picks: Vec<Option<(usize, u32)>> = small
            .par_iter()
            .map(|&c| {
                let own = &faces[start[c]..start[c + 1]];
                let label = labels[own[0] as usize];
                let mut around: Vec<u32> = Vec::new();
                let mut normal = [0.0f64; 3];
                let mut add = |f: usize| {
                    let nn = mesh.face_normal(f);
                    let a = f64::from(mesh.face_area(f));
                    for k in 0..3 {
                        normal[k] += f64::from(nn[k]) * a;
                    }
                };
                for &f in own {
                    add(f as usize);
                    for &g in nb.of(f as usize) {
                        if chart_of[g as usize] != c as u32 {
                            add(g as usize);
                            if let Some(l) = labels[g as usize].filter(|&l| Some(l) != label) {
                                around.push(l);
                            }
                        }
                    }
                }
                around.sort_unstable();
                let mut counted: Vec<(usize, u32)> = around
                    .chunk_by(|a, b| a == b)
                    .map(|r| (r.len(), r[0]))
                    .collect();
                counted.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
                let sees = |l: u32| {
                    let img = &views.images[l as usize];
                    let Some(cam) = cams.get(img.camera) else {
                        return false;
                    };
                    let eye = img.world_from_cam.translation.vector;
                    own.iter().all(|&f| {
                        let tri = mesh.indices[f as usize];
                        let mut centre = Vector3::zeros();
                        for &i in &tri {
                            let w = Vector3::from(mesh.positions[i as usize]).cast::<f64>();
                            centre += w / 3.0;
                            let pc = w2c[l as usize] * Point3::from(w);
                            if pc.coords.x.hypot(pc.coords.y).atan2(pc.coords.z)
                                > cfg.color.rim_theta
                                || cam.camera.project(&pc.coords).is_none()
                            {
                                return false;
                            }
                        }
                        let d = eye - centre;
                        d.norm() <= cfg.color.max_dist
                            && normal[0] * d.x + normal[1] * d.y + normal[2] * d.z > 0.0
                    })
                };
                counted
                    .iter()
                    .find(|&&(_, l)| sees(l))
                    .map(|&(_, l)| (c, l))
            })
            .collect();
        let mut changed = 0;
        for (c, l) in picks.into_iter().flatten() {
            for &f in &faces[start[c]..start[c + 1]] {
                labels[f as usize] = Some(l);
            }
            changed += 1;
        }
        total += changed;
        if changed == 0 {
            break;
        }
    }
    total
}

/// Calls `f(i, j, barycentric)` for every texel `(i, j)` of a `width × height` grid whose
/// centre `(i + ½, j + ½)` lies within half a texel of the triangle `corners` (texel
/// coordinates, see the module docs) and within its bounding box widened by half a texel.
/// The barycentric coordinates are clamped to the triangle. A degenerate triangle covers the
/// texels around its bounding box, with equal weights.
pub(crate) fn for_each_texel(
    corners: [[f64; 2]; 3],
    width: u32,
    height: u32,
    mut f: impl FnMut(u32, u32, [f64; 3]),
) {
    let lo = |k: usize| corners.iter().map(|c| c[k]).fold(f64::INFINITY, f64::min) - 0.5;
    let hi = |k: usize| {
        corners
            .iter()
            .map(|c| c[k])
            .fold(f64::NEG_INFINITY, f64::max)
            + 0.5
    };
    let (x0, x1, y0, y1) = (lo(0), hi(0), lo(1), hi(1));
    if !(x0.is_finite() && x1.is_finite() && y0.is_finite() && y1.is_finite()) {
        return;
    }
    // Texels whose centre lies in [x0, x1] × [y0, y1].
    let range = |a: f64, b: f64, n: u32| {
        let first = (a - 0.5).ceil().max(0.0);
        let last = (b - 0.5).floor().min(f64::from(n) - 1.0);
        (first as i64)..=(last as i64)
    };
    let [a, b, c] = corners;
    let cross = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        (q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1])
    };
    let area = cross(a, b, c);
    let edges = [(b, c), (c, a), (a, b)];
    let lengths = edges.map(|(p, q)| (q[0] - p[0]).hypot(q[1] - p[1]));
    let degenerate = area.abs() <= 1e-9 * lengths.iter().fold(1.0, |m: f64, &l| m.max(l * l));
    for j in range(y0, y1, height) {
        for i in range(x0, x1, width) {
            let q = [i as f64 + 0.5, j as f64 + 0.5];
            let bary = if degenerate {
                [1.0 / 3.0; 3]
            } else {
                // Signed distance to each edge, positive inside.
                let mut l = [0.0; 3];
                let mut inside = true;
                for (k, &(p, r)) in edges.iter().enumerate() {
                    let e = cross(p, r, q) / area;
                    l[k] = e;
                    inside &= e * area.abs() / lengths[k].max(1e-12) >= -0.5;
                }
                if !inside {
                    continue;
                }
                let l = l.map(|v| v.max(0.0));
                let sum: f64 = l.iter().sum();
                if sum > 0.0 {
                    l.map(|v| v / sum)
                } else {
                    [1.0 / 3.0; 3]
                }
            };
            f(i as u32, j as u32, bary);
        }
    }
}

/// A labelled face seen in its image: corners in image pixels (full size, centres at
/// integers) and their angles from the optical axis (rad).
#[derive(Debug, Clone, Copy)]
struct Projected {
    uv: [[f32; 2]; 3],
    theta: [f32; 3],
}

/// A chart: faces of one image, connected by edges.
#[derive(Debug, Clone)]
struct Chart {
    rank: u32,
    /// The rectangle's first side in the image (unit vector, see [`turn`]).
    axis: [f64; 2],
    /// Smallest coordinates of its corners along the rectangle's sides (pixels, full size).
    min: [f64; 2],
    /// Texels per image pixel (≤ 1).
    scale: f64,
    /// Rectangle on the page, border included.
    size: [u32; 2],
    page: u32,
    origin: [u32; 2],
}

impl Chart {
    /// Image coordinates → texel coordinates within the chart's rectangle.
    fn to_texel(&self, uv: [f32; 2]) -> [f64; 2] {
        let m = f64::from(MARGIN) + 0.5;
        let q = turn(self.axis, uv.map(f64::from));
        [0, 1].map(|k| m + (q[k] - self.min[k]) * self.scale)
    }

    /// Coordinates along the rectangle's sides (pixels) → image coordinates.
    fn to_image(&self, q: [f64; 2]) -> [f64; 2] {
        let u = self.axis;
        [u[0] * q[0] - u[1] * q[1], u[1] * q[0] + u[0] * q[1]]
    }
}

/// Bakes the texture: charts, pages and their images (see the module docs). Progress: phase
/// „Textur backen“, one step per image of the stream; a cancel is checked at every image and
/// returns [`MapError::Cancelled`]. Same input, byte-identical pages.
///
/// Fails if `cfg` is invalid, `labels` does not fit the mesh or the images cannot be read
/// or have changed since the views were found.
pub(crate) fn bake(
    mesh: &Mesh,
    labels: &[Option<u32>],
    views: &ViewTable,
    src: &TextureSource,
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<(TexturedMesh, TextureStats), MapError> {
    if progress.is_cancelled() {
        return Err(MapError::Cancelled);
    }
    cfg.check()?;
    let n = mesh.indices.len();
    if labels.len() != n {
        return Err(MapError::Config(format!(
            "{} Bildzuordnungen für {n} Dreiecke",
            labels.len()
        )));
    }
    let page_size = cfg.page_size;

    // Every labelled face in its image; a face whose corner does not project stays gray.
    let world_to_cam: Vec<Isometry3<f64>> = views
        .images
        .iter()
        .map(|i| i.world_from_cam.inverse())
        .collect();
    let proj: Vec<Option<Projected>> = (0..n)
        .into_par_iter()
        .map(|f| {
            let r = labels[f]? as usize;
            let cam = &src.cams.get(views.images.get(r)?.camera)?.camera;
            let mut p = Projected {
                uv: [[0.0; 2]; 3],
                theta: [0.0; 3],
            };
            for (k, &i) in mesh.indices[f].iter().enumerate() {
                let w = Vector3::from(mesh.positions[i as usize]).cast::<f64>();
                let pc = world_to_cam[r] * Point3::from(w);
                let uv = cam.project(&pc.coords)?;
                p.uv[k] = [uv[0] as f32, uv[1] as f32];
                p.theta[k] = pc.x.hypot(pc.y).atan2(pc.z) as f32;
            }
            Some(p)
        })
        .collect();

    // Charts: connected faces with the same label, in face order.
    let nb = Neighbours::new(mesh);
    let (chart_of, chart_start, chart_faces) = connected(&nb, n, |f| proj[f].and(labels[f]));
    drop((nb, chart_of));
    let chart_rank: Vec<u32> = chart_start[..chart_start.len() - 1]
        .iter()
        .map(|&i| labels[chart_faces[i] as usize].expect("projected faces have a label"))
        .collect();

    // Texel scale of every connected chart (`min_texel`), then poorly filled charts split
    // into parts; the parts are the charts from here on.
    let groups: Vec<(u32, f64, Vec<Part>)> = (0..chart_rank.len())
        .into_par_iter()
        .map(|c| {
            let faces = &chart_faces[chart_start[c]..chart_start[c + 1]];
            let (mut surface, mut image) = (0.0f64, 0.0f64);
            for &f in faces {
                surface += f64::from(mesh.face_area(f as usize));
                image += pixel_area(&proj[f as usize].expect("chart faces are projected").uv);
            }
            let texel = if image > 0.0 {
                (surface / image).sqrt()
            } else {
                0.0
            };
            let scale = if texel > 0.0 && texel < cfg.min_texel {
                texel / cfg.min_texel
            } else {
                1.0
            };
            (
                chart_rank[c],
                scale,
                split_chart(faces.to_vec(), &proj, scale),
            )
        })
        .collect();
    drop(chart_faces);
    let mut part_faces: Vec<u32> = Vec::new();
    let mut part_start: Vec<usize> = vec![0];
    let mut part_of: Vec<(u32, f64, [f64; 2])> = Vec::new();
    for (rank, scale, parts) in groups {
        for (part, axis) in parts {
            part_faces.extend(part);
            part_start.push(part_faces.len());
            part_of.push((rank, scale, axis));
        }
    }
    let charts_n = part_of.len();
    let faces_of = |c: usize| &part_faces[part_start[c]..part_start[c + 1]];

    // Chart rectangles: image bounding box, scaled to `min_texel` and to fit a page.
    let room = f64::from(page_size - 2 * MARGIN - 1);
    let mut charts: Vec<Chart> = (0..charts_n)
        .into_par_iter()
        .map(|c| {
            let (rank, mut scale, axis) = part_of[c];
            let (lo, hi) = bounds(faces_of(c), &proj, axis);
            let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]);
            if extent * scale > room {
                scale = room / extent;
            }
            let core = |k: usize| {
                let c = ((hi[k] - lo[k]) * scale + 1.0).ceil().max(1.0);
                (c as u32).min(page_size - 2 * MARGIN)
            };
            Chart {
                rank,
                axis,
                min: lo,
                scale,
                size: [core(0) + 2 * MARGIN, core(1) + 2 * MARGIN],
                page: 0,
                origin: [0, 0],
            }
        })
        .collect();

    // Packing; the gray block first if any face is unseen.
    let unseen = proj.iter().filter(|p| p.is_none()).count();
    let gray = usize::from(unseen > 0);
    let sizes: Vec<[u32; 2]> = (0..gray)
        .map(|_| [GRAY_BLOCK; 2])
        .chain(charts.iter().map(|c| c.size))
        .collect();
    let (at, page_count) = pack(&sizes, page_size, gray == 1);
    for (chart, &(page, origin)) in charts.iter_mut().zip(&at[gray..]) {
        chart.page = page;
        chart.origin = origin;
    }

    // Page geometry: the unseen faces first (page 0), then every chart's faces.
    let side = page_size as usize;
    let mut pages: Vec<TexturePage> = (0..page_count)
        .map(|_| TexturePage {
            positions: Vec::new(),
            uvs: Vec::new(),
            indices: Vec::new(),
            image: RgbImage {
                width: page_size,
                height: page_size,
                data: vec![UNSEEN_GRAY; side * side * 3],
            },
        })
        .collect();
    let mut face_ref = vec![(0u32, 0u32); n];
    let to_uv = |p: f64| (p / f64::from(page_size)) as f32;
    if unseen > 0 {
        let page = &mut pages[0];
        let centre = to_uv(f64::from(GRAY_BLOCK) / 2.0);
        let mut vertex = vec![NONE; mesh.positions.len()];
        for f in (0..n).filter(|&f| proj[f].is_none()) {
            let tri = mesh.indices[f].map(|i| {
                let v = &mut vertex[i as usize];
                if *v == NONE {
                    *v = page.positions.len() as u32;
                    page.positions.push(mesh.positions[i as usize]);
                    page.uvs.push([centre; 2]);
                }
                *v
            });
            face_ref[f] = (0, page.indices.len() as u32);
            page.indices.push(tri);
        }
    }
    let mut texel_mm: Vec<f64> = Vec::new();
    for (c, chart) in charts.iter().enumerate() {
        let page = &mut pages[chart.page as usize];
        let faces = faces_of(c);
        let mut verts: Vec<u32> = faces
            .iter()
            .flat_map(|&f| mesh.indices[f as usize])
            .collect();
        verts.sort_unstable();
        verts.dedup();
        let base = page.positions.len();
        page.positions
            .extend(verts.iter().map(|&v| mesh.positions[v as usize]));
        page.uvs.resize(base + verts.len(), [0.0; 2]);
        for &f in faces {
            let f = f as usize;
            let p = proj[f].expect("chart faces are projected");
            let mut tri = [0u32; 3];
            for (k, corner) in tri.iter_mut().enumerate() {
                let v = base
                    + verts
                        .binary_search(&mesh.indices[f][k])
                        .expect("corner of the chart");
                let t = chart.to_texel(p.uv[k]);
                page.uvs[v] = [0, 1].map(|a| to_uv(f64::from(chart.origin[a]) + t[a]));
                *corner = v as u32;
            }
            face_ref[f] = (chart.page, page.indices.len() as u32);
            page.indices.push(tri);
            let texels = pixel_area(&p.uv) * chart.scale * chart.scale;
            if texels > 0.0 {
                texel_mm.push((f64::from(mesh.face_area(f)) / texels).sqrt() * 1000.0);
            }
        }
    }

    // Charts of every image (by rank).
    let mut by_rank: Vec<Vec<u32>> = vec![Vec::new(); views.images.len()];
    for (c, chart) in charts.iter().enumerate() {
        by_rank[chart.rank as usize].push(c as u32);
    }
    let mut remaining = by_rank.iter().filter(|c| !c.is_empty()).count();
    let total = if remaining > 0 {
        image_count(&src.images)
    } else {
        Some(0)
    };
    progress.phase("Textur backen", total);
    let mut covered = 0u64;
    if remaining > 0 {
        let d = cfg.color.downscale;
        let lin = linear_table();
        let mut cancelled = false;
        // Images that could not be decoded, and images of a known camera at the wrong size.
        let (mut unreadable, mut wrong_size) = (0usize, 0usize);
        let read = for_each_image(&src.images.bags, &src.images.topics, d, |frame| {
            if progress.is_cancelled() {
                cancelled = true;
                return false;
            }
            progress.advance(1);
            let Ok(frame) = frame else {
                unreadable += 1;
                return true;
            };
            let Some(camera) = frame_camera(&frame, &src.cams, d) else {
                if src.cams.iter().any(|c| c.camera.name == frame.camera) {
                    wrong_size += 1;
                }
                return true;
            };
            let Some(rank) = views.rank_of(camera, frame.stamp) else {
                return true;
            };
            let todo = std::mem::take(&mut by_rank[rank as usize]);
            if todo.is_empty() {
                return true;
            }
            let shot = Shot {
                rgb: &frame.rgb,
                downscale: d,
                gain: views.gains[rank as usize],
                vignetting: &views.vignetting,
                camera,
                lin: &lin,
            };
            // In batches of at most FILL_BATCH_TEXELS, so that the large charts of one image
            // are not all held at once.
            let mut from = 0;
            while from < todo.len() {
                let mut to = from;
                let mut texels = 0u64;
                while to < todo.len() && (to == from || texels < FILL_BATCH_TEXELS) {
                    let size = charts[todo[to] as usize].size;
                    texels += u64::from(size[0]) * u64::from(size[1]);
                    to += 1;
                }
                let batch = &todo[from..to];
                let filled: Vec<(Vec<u8>, u64)> = batch
                    .par_iter()
                    .map(|&c| fill_chart(&charts[c as usize], faces_of(c as usize), &proj, &shot))
                    .collect();
                for (&c, (buf, inside)) in batch.iter().zip(&filled) {
                    covered += inside;
                    let chart = &charts[c as usize];
                    let image = &mut pages[chart.page as usize].image.data;
                    let row = chart.size[0] as usize * 3;
                    for (y, src_row) in buf.chunks_exact(row).enumerate() {
                        let start =
                            ((chart.origin[1] as usize + y) * side + chart.origin[0] as usize) * 3;
                        image[start..start + row].copy_from_slice(src_row);
                    }
                }
                from = to;
            }
            remaining -= 1;
            remaining > 0
        });
        if cancelled {
            return Err(MapError::Cancelled);
        }
        read.map_err(|e| MapError::Images(e.to_string()))?;
        if remaining > 0 && unreadable + wrong_size > 0 {
            return Err(MapError::Images(format!(
                "{remaining} für die Textur nötige Bilder fehlen: {unreadable} Bilder \
                 unlesbar, {wrong_size} in falscher Größe"
            )));
        }
        if remaining > 0 {
            return Err(MapError::Images(
                "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
            ));
        }
        // The stream stops after the last needed image: the phase is complete.
        if let Some(total) = total {
            let done = progress.snapshot().done;
            progress.advance(total.saturating_sub(done));
        }
    }

    texel_mm.par_sort_unstable_by(f64::total_cmp);
    let texels_total: u64 = charts
        .iter()
        .map(|c| u64::from(c.size[0]) * u64::from(c.size[1]))
        .sum();
    let textured: Vec<bool> = proj.iter().map(Option::is_some).collect();
    let infos: Vec<ChartInfo> = charts
        .iter()
        .enumerate()
        .map(|(c, chart)| ChartInfo {
            page: chart.page,
            rect: [
                chart.origin[0],
                chart.origin[1],
                chart.size[0],
                chart.size[1],
            ],
            faces: faces_of(c).to_vec(),
        })
        .collect();
    let stats = TextureStats {
        faces_textured: n - unseen,
        faces_untextured: unseen,
        charts: charts_n,
        pages: pages.len(),
        texel_median_mm: texel_mm.get(texel_mm.len() / 2).copied().unwrap_or(0.0),
        texels_total,
        fill_ratio: if texels_total > 0 {
            covered as f64 / texels_total as f64
        } else {
            0.0
        },
        islands_absorbed: 0,
    };
    Ok((
        TexturedMesh {
            pages,
            face_ref,
            textured,
            charts: infos,
        },
        stats,
    ))
}

/// Image point `p` in the frame of a rectangle whose first side runs along `axis` (unit
/// vector): `(axis · p, axis × p)`.
fn turn(axis: [f64; 2], p: [f64; 2]) -> [f64; 2] {
    [
        axis[0] * p[0] + axis[1] * p[1],
        axis[0] * p[1] - axis[1] * p[0],
    ]
}

/// Smallest and largest coordinates of the corners of `faces` along the sides of a rectangle
/// whose first side runs along `axis` (see [`turn`]).
fn bounds(faces: &[u32], proj: &[Option<Projected>], axis: [f64; 2]) -> ([f64; 2], [f64; 2]) {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for &f in faces {
        for uv in proj[f as usize].expect("chart faces are projected").uv {
            let q = turn(axis, uv.map(f64::from));
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
    }
    (lo, hi)
}

/// The image corners of `faces`.
fn corners(faces: &[u32], proj: &[Option<Projected>]) -> Vec<[f64; 2]> {
    faces
        .iter()
        .flat_map(|&f| proj[f as usize].expect("chart faces are projected").uv)
        .map(|uv| uv.map(f64::from))
        .collect()
}

/// A part of a chart: its faces (ascending) and the first side of its rectangle.
type Part = (Vec<u32>, [f64; 2]);

/// Splits a chart (`faces` ascending, texel scale `scale`) into the parts that need the
/// fewest texels, borders included (see [`MIN_CHART_FILL`]). Each part's rectangle is the
/// smallest one around its corners in the image ([`best_axis`]); a part that fills less than
/// [`MIN_CHART_FILL`] of it is cut in two where the halves' rectangles need the fewest texels
/// (among [`SPLIT_CUTS`] − 1 quantiles of the face centroids along either side), each half
/// again, down to single faces, and a cut is kept where its parts together need fewer texels
/// than the whole (black room: fill 0.55 with the median cut only, 0.61 with this). Equal
/// centroids are ordered by face index; the parts come in a fixed order, each ascending.
fn split_chart(faces: Vec<u32>, proj: &[Option<Projected>], scale: f64) -> Vec<Part> {
    split_best(faces, proj, scale).1
}

/// [`split_chart`]: the texels of the best split of `faces` and its parts.
fn split_best(faces: Vec<u32>, proj: &[Option<Projected>], scale: f64) -> (f64, Vec<Part>) {
    let axis = best_axis(corners(&faces, proj));
    let (lo, hi) = bounds(&faces, proj, axis);
    let core = [0, 1].map(|k| ((hi[k] - lo[k]) * scale + 1.0).ceil());
    let whole = (core[0] + 2.0 * f64::from(MARGIN)) * (core[1] + 2.0 * f64::from(MARGIN));
    let area: f64 = faces
        .iter()
        .map(|&f| pixel_area(&proj[f as usize].expect("projected").uv))
        .sum();
    if faces.len() <= 1 || area * scale * scale >= MIN_CHART_FILL * core[0] * core[1] {
        return (whole, vec![(faces, axis)]);
    }
    // The cut: at one of SPLIT_CUTS − 1 quantiles of the face centroids along either side of
    // the rectangle, the one whose two halves need the fewest texels as they stand.
    let rect = |fs: &[u32]| {
        let ax = best_axis(corners(fs, proj));
        let (lo, hi) = bounds(fs, proj, ax);
        let c = [0, 1].map(|k| ((hi[k] - lo[k]) * scale + 1.0).ceil());
        (c[0] + 2.0 * f64::from(MARGIN)) * (c[1] + 2.0 * f64::from(MARGIN))
    };
    let mut cut: Option<(f64, Vec<u32>, Vec<u32>)> = None;
    for side in 0..2 {
        let mut keyed: Vec<(f64, u32)> = faces
            .iter()
            .map(|&f| {
                let uv = proj[f as usize].expect("projected").uv;
                let c = [0, 1].map(|k| f64::from(uv[0][k] + uv[1][k] + uv[2][k]) / 3.0);
                (turn(axis, c)[side], f)
            })
            .collect();
        keyed.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for q in 1..SPLIT_CUTS {
            let mid = (keyed.len() * q / SPLIT_CUTS).clamp(1, keyed.len() - 1);
            let half = |s: &[(f64, u32)]| {
                let mut v: Vec<u32> = s.iter().map(|k| k.1).collect();
                v.sort_unstable();
                v
            };
            let (low, high) = (half(&keyed[..mid]), half(&keyed[mid..]));
            let cost = rect(&low) + rect(&high);
            if cut.as_ref().is_none_or(|c| cost < c.0) {
                cut = Some((cost, low, high));
            }
        }
    }
    let (_, low, high) = cut.expect("at least two faces");
    let (a, mut parts) = split_best(low, proj, scale);
    let (b, rest) = split_best(high, proj, scale);
    if a + b < whole {
        parts.extend(rest);
        (a + b, parts)
    } else {
        // Undo: the whole chart, ascending again.
        let mut all: Vec<u32> = parts.into_iter().chain(rest).flat_map(|(f, _)| f).collect();
        all.sort_unstable();
        (whole, vec![(all, axis)])
    }
}

/// Convex hull (counter-clockwise, no collinear points) of `pts` (Andrew's monotone chain).
fn convex_hull(mut pts: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    pts.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut hull: Vec<[f64; 2]> = Vec::with_capacity(pts.len() + 1);
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &[f64; 2]>> = if pass == 0 {
            Box::new(pts.iter())
        } else {
            Box::new(pts.iter().rev())
        };
        for &p in iter {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// Direction (unit vector) of the long side of the smallest rectangle around `pts`: one side of
/// it lies on an edge of their convex hull. `[1, 0]` (the image axes) if nothing is smaller.
fn best_axis(pts: Vec<[f64; 2]>) -> [f64; 2] {
    let hull = convex_hull(pts);
    let extent = |u: [f64; 2]| {
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in &hull {
            let q = [u[0] * p[0] + u[1] * p[1], u[0] * p[1] - u[1] * p[0]];
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
        [hi[0] - lo[0], hi[1] - lo[1]]
    };
    // Texels of the core rectangle, as `rect_texels` counts them.
    let cost = |e: [f64; 2]| (e[0] + 1.0) * (e[1] + 1.0);
    let mut best = ([1.0, 0.0], cost(extent([1.0, 0.0])));
    for i in 0..hull.len() {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0].hypot(d[1]);
        if l == 0.0 {
            continue;
        }
        let u = [d[0] / l, d[1] / l];
        let e = extent(u);
        // The long side along the first axis.
        let u = if e[1] > e[0] { [-u[1], u[0]] } else { u };
        let c = cost(e);
        if c < best.1 * (1.0 - 1e-9) {
            best = (u, c);
        }
    }
    best.0
}

/// Area of a triangle in image pixels.
fn pixel_area(uv: &[[f32; 2]; 3]) -> f64 {
    let [a, b, c] = uv.map(|p| p.map(f64::from));
    0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs()
}

/// One decoded image and its exposure.
struct Shot<'a> {
    /// At 1/`downscale`.
    rgb: &'a RgbImage,
    downscale: u32,
    gain: [f32; 3],
    vignetting: &'a Vignetting,
    camera: usize,
    lin: &'a [f32; 256],
}

/// The chart's rectangle (row-major sRGB, border included) from `shot`, and how many of its
/// texels the triangles cover.
fn fill_chart(
    chart: &Chart,
    faces: &[u32],
    proj: &[Option<Projected>],
    shot: &Shot,
) -> (Vec<u8>, u64) {
    let [w, h] = chart.size;
    let (wu, hu) = (w as usize, h as usize);
    let mut buf = vec![UNSEEN_GRAY; wu * hu * 3];
    let mut filled = vec![false; wu * hu];
    let m = f64::from(MARGIN);
    let d = f64::from(shot.downscale);
    // Samples per axis and texel: about one per image pixel (decoded), at most MAX_SUPERSAMPLE.
    let grid = ((d * chart.scale).recip().ceil() as u32).clamp(1, MAX_SUPERSAMPLE);
    for &f in faces {
        let p = proj[f as usize].expect("chart faces are projected");
        let corners = p.uv.map(|uv| chart.to_texel(uv));
        for_each_texel(corners, w, h, |i, j, bary| {
            // The texel centre along the rectangle's sides (image pixels, full size).
            let full = [
                chart.min[0] + (f64::from(i) - m) / chart.scale,
                chart.min[1] + (f64::from(j) - m) / chart.scale,
            ];
            // A scaled-down chart averages a grid over the texel's footprint.
            let mut rgb = [0.0f32; 3];
            for a in 0..grid {
                for b in 0..grid {
                    let off = [a, b].map(|t| (f64::from(t) + 0.5) / f64::from(grid) - 0.5);
                    let at = chart.to_image([0, 1].map(|k| full[k] + off[k] / chart.scale));
                    let uv = at.map(|x| ((x + 0.5) / d - 0.5) as f32);
                    let s = sample_linear(shot.rgb, uv, shot.lin);
                    for c in 0..3 {
                        rgb[c] += s[c];
                    }
                }
            }
            let rgb = rgb.map(|v| v / (grid * grid) as f32);
            let theta = (0..3).map(|k| bary[k] * f64::from(p.theta[k])).sum::<f64>() as f32;
            let o = (j as usize * wu + i as usize) * 3;
            for c in 0..3 {
                let l = rgb[c] / (shot.gain[c] * shot.vignetting.factor(shot.camera, c, theta));
                buf[o + c] = srgb_value(l).round().clamp(0.0, 255.0) as u8;
            }
            filled[j as usize * wu + i as usize] = true;
        });
    }
    let covered = filled.iter().filter(|&&f| f).count() as u64;
    fill_rest(&mut buf, &mut filled, wu, hu);
    (buf, covered)
}

/// Fills the texels of a chart rectangle (`w × h`, row-major sRGB) that `filled` marks as
/// empty: [`MARGIN`] rounds of dilation, then the mean colour of the filled texels (so that
/// no gray bleeds into the chart at coarser mip levels). Unchanged if nothing is filled.
pub(crate) fn fill_rest(buf: &mut [u8], filled: &mut [bool], w: usize, h: usize) {
    dilate(buf, filled, w, h, MARGIN);
    let (mut sum, mut count) = ([0u64; 3], 0u64);
    for (i, _) in filled.iter().enumerate().filter(|(_, &f)| f) {
        for c in 0..3 {
            sum[c] += u64::from(buf[i * 3 + c]);
        }
        count += 1;
    }
    if count == 0 || count == filled.len() as u64 {
        return;
    }
    let mean = sum.map(|v| ((v + count / 2) / count) as u8);
    for (i, f) in filled.iter_mut().enumerate() {
        if !*f {
            buf[i * 3..i * 3 + 3].copy_from_slice(&mean);
            *f = true;
        }
    }
}

/// `rounds` times: every unfilled texel next to a filled one (8-neighbourhood) gets their
/// mean colour.
fn dilate(buf: &mut [u8], filled: &mut [bool], w: usize, h: usize, rounds: u32) {
    for _ in 0..rounds {
        let before = filled.to_vec();
        let mut grown: Vec<(usize, [u8; 3])> = Vec::new();
        for y in 0..h {
            for x in 0..w {
                if before[y * w + x] {
                    continue;
                }
                let (mut sum, mut count) = ([0u32; 3], 0u32);
                for ny in y.saturating_sub(1)..=(y + 1).min(h - 1) {
                    for nx in x.saturating_sub(1)..=(x + 1).min(w - 1) {
                        let i = ny * w + nx;
                        if before[i] {
                            for c in 0..3 {
                                sum[c] += u32::from(buf[i * 3 + c]);
                            }
                            count += 1;
                        }
                    }
                }
                if count > 0 {
                    grown.push((y * w + x, sum.map(|s| ((s + count / 2) / count) as u8)));
                }
            }
        }
        if grown.is_empty() {
            break;
        }
        for (i, rgb) in grown {
            buf[i * 3..i * 3 + 3].copy_from_slice(&rgb);
            filled[i] = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testutil::open_wall;
    use super::super::testutil::scene::{cameras, poses, write_bag, Painted, Shot, H, T0, W};
    use super::super::testutil::Steps;
    use super::super::views::{collect_views, FaceView, UsedImage};
    use super::*;
    use crate::exposure::Vignetting;
    use crate::ColorCamera;
    use driftless_io::camera::Camera;
    use nalgebra::Isometry3;

    /// Edge of a checkerboard field on the +y wall (m).
    const FIELD: f64 = 0.25;
    const FIELD_A: [u8; 3] = [40, 60, 210];
    const FIELD_B: [u8; 3] = [230, 200, 40];
    /// The painted room's +y wall, and its −y wall.
    const FRONT_Y: f32 = 5.0;
    const BACK_Y: f32 = -4.0;
    /// Colour tolerance (sRGB steps).
    const TOL: i32 = 8;

    fn checker(x: f64, z: f64) -> [u8; 3] {
        let i = (x / FIELD).floor() as i64 + (z / FIELD).floor() as i64;
        if i.rem_euclid(2) == 0 {
            FIELD_A
        } else {
            FIELD_B
        }
    }

    /// The painted room with a checkerboard on the +y wall, seen by `cam` at `world_cam`.
    fn render_checker(cam: &Camera, world_cam: &Isometry3<f64>) -> Vec<u8> {
        let painted = Painted::new(vec![]);
        let o = world_cam.translation.vector;
        let mut rgb = Vec::with_capacity((W * H * 3) as usize);
        for v in 0..H {
            for u in 0..W {
                let c = cam
                    .unproject([f64::from(u), f64::from(v)])
                    .map_or([0; 3], |dir| {
                        let d = world_cam.rotation * dir;
                        let p = o + d * painted.scene.raycast(&o, &d);
                        if (p.y - f64::from(FRONT_Y)).abs() < 1e-6 {
                            checker(p.x, p.z)
                        } else {
                            painted.color_at(&p)
                        }
                    });
                rgb.extend(c);
            }
        }
        rgb
    }

    /// A grid of `step` squares in the plane `y`, over `x` and `z`, normals along `ny` (±1).
    fn patch(y: f32, x: [f32; 2], z: [f32; 2], step: f32, ny: f32) -> Mesh {
        let (nx, nz) = (
            ((x[1] - x[0]) / step).round() as u32,
            ((z[1] - z[0]) / step).round() as u32,
        );
        let mut mesh = Mesh::default();
        for i in 0..=nx {
            for j in 0..=nz {
                mesh.positions
                    .push([x[0] + i as f32 * step, y, z[0] + j as f32 * step]);
            }
        }
        let id = |i: u32, j: u32| i * (nz + 1) + j;
        for i in 0..nx {
            for j in 0..nz {
                for tri in [
                    [id(i, j), id(i + 1, j), id(i + 1, j + 1)],
                    [id(i, j), id(i + 1, j + 1), id(i, j + 1)],
                ] {
                    let f = mesh.indices.len();
                    mesh.indices.push(tri);
                    if mesh.face_normal(f)[1] * ny < 0.0 {
                        mesh.indices[f] = [tri[0], tri[2], tri[1]];
                    }
                }
            }
        }
        mesh
    }

    /// The front patch (x ±1 m, z −0.5 … 1 m on the +y wall, facing the camera) and, with
    /// `back`, a patch on the −y wall behind the camera; the left camera 1.5 m from the +y
    /// wall at each of `xs`. Returns the mesh, the source and the number of front faces
    /// (they come first).
    fn fixture(name: &str, xs: &[f64], back: bool) -> (Mesh, TextureSource, usize) {
        let mut mesh = patch(FRONT_Y, [-1.0, 1.0], [-0.5, 1.0], 0.1, -1.0);
        let front = mesh.indices.len();
        if back {
            let b = patch(BACK_Y, [-0.5, 0.5], [-0.5, 0.5], 0.1, 1.0);
            super::super::testutil::append(&mut mesh, &b);
        }
        let cams = cameras();
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let body = |x: f64| Isometry3::translation(x, 3.5 - cam_y, 0.0);
        let traj = Steps(
            xs.iter()
                .enumerate()
                .map(|(i, &x)| {
                    let from = if i == 0 { 0.0 } else { T0 + 0.5 + i as f64 };
                    (from, body(x))
                })
                .collect(),
        );
        let shots: Vec<Shot> = xs
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                let cam: ColorCamera = cams[0].clone();
                let world_cam = body(x) * cam.cam_in_body;
                Shot {
                    camera: 0,
                    stamp: T0 + 1.0 + i as f64,
                    edit: Box::new(move |_| render_checker(&cam.camera, &world_cam)),
                }
            })
            .collect();
        let images = write_bag(name, &Painted::new(vec![]), &traj, &cams, &shots);
        let src = TextureSource {
            images,
            poses: poses(&traj, xs.len() as f64 + 1.0),
            cams,
        };
        (mesh, src, front)
    }

    /// Texel colour at `uv` of `page`.
    fn texel(page: &TexturePage, uv: [f32; 2]) -> [u8; 3] {
        let (w, h) = (page.image.width, page.image.height);
        let x = ((uv[0] * w as f32).floor() as u32).min(w - 1);
        let y = ((uv[1] * h as f32).floor() as u32).min(h - 1);
        let i = ((y * w + x) * 3) as usize;
        [0, 1, 2].map(|c| page.image.data[i + c])
    }

    /// Checks the baked colour at the centre of every checkerboard field of the front patch;
    /// asserts all 48 fields were on it.
    fn assert_checker(mesh: &Mesh, tm: &TexturedMesh, front: usize) {
        assert_eq!(check_fields(mesh, tm, front), 48);
    }

    /// Checks the baked colour at the centre of every checkerboard field that lies on one of
    /// the first `front` faces; returns how many did.
    fn check_fields(mesh: &Mesh, tm: &TexturedMesh, front: usize) -> usize {
        let mut checked = 0;
        for ix in 0..8 {
            for iz in 0..6 {
                let (x, z) = (
                    -1.0 + FIELD * (ix as f64 + 0.5),
                    -0.5 + FIELD * (iz as f64 + 0.5),
                );
                // The front face containing (x, z), with the barycentric coordinates there.
                let found = (0..front).find_map(|f| {
                    let c = mesh.indices[f].map(|i| mesh.positions[i as usize]);
                    let p = c.map(|p| [f64::from(p[0]), f64::from(p[2])]);
                    let area = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
                        (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
                    };
                    let q = [x, z];
                    let all = area(p[0], p[1], p[2]);
                    let l = [
                        area(q, p[1], p[2]) / all,
                        area(p[0], q, p[2]) / all,
                        area(p[0], p[1], q) / all,
                    ];
                    l.iter().all(|&v| v >= -1e-9).then_some((f, l))
                });
                let Some((f, bary)) = found else {
                    continue;
                };
                let (p, t) = tm.face_ref[f];
                let page = &tm.pages[p as usize];
                let tri = page.indices[t as usize];
                let uv = [0, 1].map(|k| {
                    (0..3)
                        .map(|c| bary[c] as f32 * page.uvs[tri[c] as usize][k])
                        .sum::<f32>()
                });
                let got = texel(page, uv);
                let want = checker(x, z);
                for c in 0..3 {
                    assert!(
                        (i32::from(got[c]) - i32::from(want[c])).abs() <= TOL,
                        "field ({x}, {z}) face {f}: {got:?} vs {want:?}"
                    );
                }
                checked += 1;
            }
        }
        checked
    }

    /// Keeps the faces of `mesh` whose centroid passes `keep`.
    fn keep_faces(mesh: &mut Mesh, keep: impl Fn([f32; 3]) -> bool) {
        let m = mesh.clone();
        mesh.indices.retain(|t| {
            keep(
                [0, 1, 2].map(|k| t.iter().map(|&i| m.positions[i as usize][k]).sum::<f32>() / 3.0),
            )
        });
    }

    /// Each face in exactly one chart, rectangles on their pages.
    fn assert_charts(tm: &TexturedMesh, page_size: u32) {
        let mut owner = vec![usize::MAX; tm.face_ref.len()];
        for (c, chart) in tm.charts.iter().enumerate() {
            let [x, y, w, h] = chart.rect;
            assert!(x + w <= page_size && y + h <= page_size, "{:?}", chart.rect);
            assert!((chart.page as usize) < tm.pages.len());
            for &f in &chart.faces {
                assert_eq!(owner[f as usize], usize::MAX, "face {f} in two charts");
                owner[f as usize] = c;
                assert!(tm.textured[f as usize]);
                assert_eq!(tm.face_ref[f as usize].0, chart.page);
            }
        }
        for (f, &o) in owner.iter().enumerate() {
            assert_eq!(o != usize::MAX, tm.textured[f], "face {f}");
        }
    }

    fn assert_pages_sane(tm: &TexturedMesh, page_size: u32, faces: usize) {
        assert_eq!(tm.face_ref.len(), faces);
        for page in &tm.pages {
            assert_eq!(
                (page.image.width, page.image.height),
                (page_size, page_size)
            );
            assert_eq!(page.image.data.len(), (page_size * page_size * 3) as usize);
            assert_eq!(page.positions.len(), page.uvs.len());
            assert!(page
                .uvs
                .iter()
                .all(|uv| uv.iter().all(|&v| (0.0..=1.0).contains(&v))));
        }
        // Every face is exactly one page triangle with its own corners.
        let mut seen: Vec<Vec<bool>> = tm
            .pages
            .iter()
            .map(|p| vec![false; p.indices.len()])
            .collect();
        for &(p, t) in &tm.face_ref {
            assert!(!seen[p as usize][t as usize], "page {p} triangle {t} twice");
            seen[p as usize][t as usize] = true;
        }
        assert!(seen.iter().flatten().all(|&s| s));
    }

    #[test]
    fn neighbours_share_image() {
        // A wall seen by two images about equally well: image 1 scores up to 4.5 % above
        // image 0 on a quarter of the faces (scattered), up to 1.5 % below on the rest.
        let mesh = open_wall(40, 0.1);
        let n = mesh.indices.len();
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let hash = |f: usize| {
            let mut x = (f as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            x ^= x >> 31;
            x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
            x ^= x >> 29;
            (x >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut per_face = vec![FaceView::EMPTY; n * k];
        for f in 0..n {
            let one = (1.0 + 0.06 * (hash(f) - 0.75)) as f32;
            let mut v = [
                FaceView {
                    image: 0,
                    score: 1.0,
                    colour: [0; 3],
                    tag: 0,
                },
                FaceView {
                    image: 1,
                    score: one,
                    colour: [0; 3],
                    tag: 0,
                },
            ];
            if one > 1.0 {
                v.swap(0, 1);
            }
            per_face[f * k..f * k + 2].copy_from_slice(&v);
        }
        let image = |stamp: f64| UsedImage {
            camera: 0,
            stamp,
            world_from_cam: Isometry3::identity(),
        };
        let views = ViewTable {
            images: vec![image(1.0), image(2.0)],
            per_face,
            gains: vec![[1.0; 3]; 2],
            vignetting: Vignetting::flat(1, 4.0),
        };
        let share = |labels: &[Option<u32>], image: u32| {
            labels.iter().filter(|&&l| l == Some(image)).count() as f64 / n as f64
        };
        let best: Vec<Option<u32>> = (0..n).map(|f| Some(views.per_face[f * k].image)).collect();
        let before = share(&best, 0);
        assert!(before > 0.6 && before < 0.9, "{before}");
        let labels = choose_labels(&mesh, &views, &cfg);
        assert_eq!(labels.len(), n);
        let after = share(&labels, 0).max(share(&labels, 1));
        println!("image 0 before ICM {before:.3}, majority after {after:.3}");
        assert!(after >= 0.95, "{after}");
        // Without smoothing every face keeps its best image.
        let plain = SurfaceConfig {
            smooth_weight: 0.0,
            ..cfg.clone()
        };
        assert_eq!(choose_labels(&mesh, &views, &plain), best);
    }

    #[test]
    fn checkerboard_survives_bake() {
        let (mesh, src, front) = fixture("atlas-checker", &[0.0], false);
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        assert!(labels.iter().all(Option::is_some));
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        let snap = progress.snapshot();
        assert_eq!(snap.phase, "Textur backen");
        assert_eq!(Some(snap.done), snap.total);
        assert_eq!(stats.faces_textured, front);
        assert_eq!(stats.faces_untextured, 0);
        assert_charts(&tm, cfg.page_size);
        assert!(
            stats.fill_ratio > 0.5 && stats.fill_ratio <= 1.0,
            "{stats:?}"
        );
        let [_, _, w, h] = tm.charts[0].rect;
        assert_eq!(stats.texels_total, u64::from(w * h));
        assert_eq!(stats.charts, 1);
        assert_eq!(stats.pages, 1);
        // 1.5 m from a 229 px/rad lens: about 6.5 mm per pixel head-on.
        assert!(
            stats.texel_median_mm > 4.0 && stats.texel_median_mm < 12.0,
            "{stats:?}"
        );
        assert_pages_sane(&tm, cfg.page_size, mesh.indices.len());
        assert_checker(&mesh, &tm, front);
        // Same input, byte-identical pages.
        let (again, _) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        assert_eq!(again, tm);
        // Coarser than the image: the chart is scaled down to 15 mm texels and averaged.
        let coarse = SurfaceConfig {
            min_texel: 0.015,
            ..cfg.clone()
        };
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &coarse, &progress).unwrap();
        assert!((stats.texel_median_mm - 15.0).abs() < 0.5, "{stats:?}");
        assert_checker(&mesh, &tm, front);
    }

    #[test]
    fn unseen_faces_get_gray_block() {
        let (mesh, src, front) = fixture("atlas-unseen", &[0.0], true);
        let back = mesh.indices.len() - front;
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        assert!(labels[front..].iter().all(Option::is_none));
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        assert_eq!(stats.faces_untextured, back);
        assert_eq!(stats.faces_textured, front);
        assert_charts(&tm, cfg.page_size);
        assert!(tm.textured[..front].iter().all(|&t| t));
        assert!(tm.textured[front..].iter().all(|&t| !t));
        assert_pages_sane(&tm, cfg.page_size, mesh.indices.len());
        let block = GRAY_BLOCK as f32 / cfg.page_size as f32;
        for f in front..mesh.indices.len() {
            let (p, t) = tm.face_ref[f];
            assert_eq!(p, 0);
            let page = &tm.pages[0];
            let tri = page.indices[t as usize];
            for (c, &v) in tri.iter().enumerate() {
                let uv = page.uvs[v as usize];
                assert!(
                    uv[0] > 0.0 && uv[0] < block && uv[1] > 0.0 && uv[1] < block,
                    "{uv:?}"
                );
                assert_eq!(texel(page, uv), [UNSEEN_GRAY; 3]);
                // The page triangle has the face's corners.
                assert_eq!(
                    page.positions[v as usize],
                    mesh.positions[mesh.indices[f][c] as usize]
                );
            }
        }
        // The whole block is gray.
        let w = cfg.page_size as usize;
        for y in 0..GRAY_BLOCK as usize {
            for x in 0..GRAY_BLOCK as usize {
                let i = (y * w + x) * 3;
                assert_eq!(&tm.pages[0].image.data[i..i + 3], &[UNSEEN_GRAY; 3]);
            }
        }
        assert_checker(&mesh, &tm, front);
    }

    #[test]
    fn pages_respect_size() {
        // Packing alone: many rectangles on pages of 64², the first one kept at the origin.
        let page = 64;
        let mut sizes = vec![[GRAY_BLOCK; 2]];
        let mut x = 12345u32;
        for _ in 0..400 {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let w = 1 + (x >> 16) % 40;
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let h = 1 + (x >> 16) % 40;
            sizes.push([w, h]);
        }
        sizes.push([page, page]);
        let (at, pages) = pack(&sizes, page, true);
        assert_eq!(at.len(), sizes.len());
        assert_eq!(at[0], (0, [0, 0]));
        assert!(pages > 10, "{pages}");
        for (i, (&(p, o), s)) in at.iter().zip(&sizes).enumerate() {
            assert!(p < pages);
            assert!(
                o[0] + s[0] <= page && o[1] + s[1] <= page,
                "{i}: {o:?} {s:?}"
            );
            for (&(q, u), t) in at[..i].iter().zip(&sizes) {
                let apart = p != q
                    || o[0] + s[0] <= u[0]
                    || u[0] + t[0] <= o[0]
                    || o[1] + s[1] <= u[1]
                    || u[1] + t[1] <= o[1];
                assert!(apart, "{i} overlaps: {o:?} {s:?} / {u:?} {t:?}");
            }
        }
        assert_eq!(pack(&sizes, page, true), (at, pages));

        // Baking: two images, every face alternately from one of them (many small charts),
        // on pages of 64².
        let (mesh, src, front) = fixture("atlas-pages", &[-0.15, 0.15], false);
        let cfg = SurfaceConfig {
            page_size: page,
            ..SurfaceConfig::default()
        };
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        assert_eq!(views.images.len(), 2);
        let k = cfg.views_per_face;
        let labels: Vec<Option<u32>> = (0..front)
            .map(|f| {
                let v = &views.per_face[f * k..(f + 1) * k];
                let want = (f % 2) as u32;
                assert!(v.iter().any(|x| x.image == want), "face {f}: {v:?}");
                Some(want)
            })
            .collect();
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        assert!(stats.charts > 100, "{stats:?}");
        assert!(
            stats.pages > 1 && stats.pages == tm.pages.len(),
            "{stats:?}"
        );
        assert_eq!(stats.faces_textured, front);
        assert_pages_sane(&tm, page, front);
        assert_charts(&tm, page);
        assert_checker(&mesh, &tm, front);
    }

    #[test]
    fn poorly_filled_chart_is_split() {
        // An L of 0.3 m wide arms along the left and the bottom of the front patch, one image:
        // one connected chart covering about a third of its bounding box.
        let (mut mesh, src, _) = fixture("atlas-split", &[0.0], false);
        keep_faces(&mut mesh, |c| c[0] < -0.7 || c[2] < -0.2);
        let faces = mesh.indices.len();
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        assert!(labels.iter().all(|&l| l == Some(0)));
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        assert!(stats.charts > 1, "{stats:?}");
        assert!(stats.fill_ratio >= 0.5, "{stats:?}");
        assert_eq!(stats.faces_textured, faces);
        assert_pages_sane(&tm, cfg.page_size, faces);
        assert_charts(&tm, cfg.page_size);
        assert!(check_fields(&mesh, &tm, faces) >= 10);
        // Deterministic.
        let (again, _) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        assert_eq!(again, tm);
    }

    #[test]
    fn chart_rectangle_has_no_gray() {
        // The front patch without its top-right corner (0.4 × 0.4 m): one concave chart,
        // well enough filled not to be split, with a corner far from every triangle.
        let (mut mesh, src, _) = fixture("atlas-notch", &[0.0], false);
        keep_faces(&mut mesh, |c| !(c[0] > 0.6 && c[2] > 0.6));
        let faces = mesh.indices.len();
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        assert_eq!(stats.charts, 1);
        assert!(stats.fill_ratio < 0.95, "{stats:?}");
        assert_charts(&tm, cfg.page_size);
        let chart = &tm.charts[0];
        let [x, y, w, h] = chart.rect;
        let image = &tm.pages[chart.page as usize].image;
        for j in y..y + h {
            for i in x..x + w {
                let o = ((j * image.width + i) * 3) as usize;
                assert_ne!(&image.data[o..o + 3], &[UNSEEN_GRAY; 3], "texel ({i}, {j})");
            }
        }
        assert!(check_fields(&mesh, &tm, faces) >= 40);
    }

    #[test]
    fn lopsided_chart_is_cut_where_it_pays() {
        // An L with a tall arm (0.3 × 1.5 m) and a low foot (1.7 × 0.2 m): most faces are in
        // the arm, so the median cut along the long side falls inside the arm and leaves an L.
        let (mut mesh, src, _) = fixture("atlas-lopsided", &[0.0], false);
        keep_faces(&mut mesh, |c| c[0] < -0.7 || c[2] < -0.3);
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        assert_charts(&tm, cfg.page_size);
        assert!(stats.fill_ratio > 0.75, "{stats:?}");
        assert!(check_fields(&mesh, &tm, mesh.indices.len()) >= 10);
    }

    /// The views of a wall with one image everywhere and `patch` (cells 8–11 in both
    /// directions of `open_wall(20, 0.1)`, 32 faces) best seen by image 1 and by image 0 at
    /// `second` of that.
    fn patch_views(mesh: &Mesh, k: usize, second: f32) -> ViewTable {
        let n = mesh.indices.len();
        let mut per_face = vec![FaceView::EMPTY; n * k];
        let view = |image: u32, score: f32| FaceView {
            image,
            score,
            colour: [0; 3],
            tag: 0,
        };
        for f in 0..n {
            let c = super::super::testutil::centroid(mesh, f);
            let inside = |x: f32| (0.8..1.2).contains(&x);
            per_face[f * k] = view(0, 1.0);
            if inside(c[1]) && inside(c[2]) {
                per_face[f * k] = view(1, 1.0);
                per_face[f * k + 1] = view(0, second);
            }
        }
        let image = |stamp: f64| UsedImage {
            camera: 0,
            stamp,
            world_from_cam: Isometry3::identity(),
        };
        ViewTable {
            images: vec![image(1.0), image(2.0)],
            per_face,
            gains: vec![[1.0; 3]; 2],
            vignetting: Vignetting::flat(1, 4.0),
        }
    }

    #[test]
    fn small_chart_merges_into_neighbour() {
        // A patch of 32 faces with its own best image inside a wall of image 0: every patch
        // face agrees with its neighbours, so ICM keeps it; image 0 sees it at half the score,
        // so it is merged.
        let mesh = open_wall(20, 0.1);
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let in_patch = |f: usize| {
            let c = super::super::testutil::centroid(&mesh, f);
            (0.8..1.2).contains(&c[1]) && (0.8..1.2).contains(&c[2])
        };
        let patch = (0..mesh.indices.len()).filter(|&f| in_patch(f)).count();
        assert_eq!(patch, 32);
        let labels = choose_labels(&mesh, &patch_views(&mesh, k, 0.5), &cfg);
        assert!(labels.iter().all(|&l| l == Some(0)), "{labels:?}");
        // Seen by image 0 at a tenth only (below MERGE_MIN_SCORE): the patch keeps image 1.
        let labels = choose_labels(&mesh, &patch_views(&mesh, k, 0.1), &cfg);
        for (f, &l) in labels.iter().enumerate() {
            assert_eq!(l, Some(u32::from(in_patch(f))), "face {f}");
        }
    }

    #[test]
    fn labels_form_large_charts() {
        // The walk through the painted room with an image every 0.1 s from both cameras (110),
        // the room as 0.1 m squares with corners moved by up to ±4 mm (noisy normals): before
        // this round 0.28 charts per textured face filling 0.35 of their rectangles.
        use super::super::testutil::scene::{shot, walk, walk_scene};
        let painted = walk_scene();
        let cams = cameras();
        let shots: Vec<Shot> = (1..=55)
            .flat_map(|i| [0, 1].map(|c| shot(c, T0 + 0.1 * f64::from(i))))
            .collect();
        let images = write_bag("atlas-walk", &painted, &walk(), &cams, &shots);
        let src = TextureSource {
            images,
            poses: poses(&walk(), 6.0),
            cams,
        };
        let mut mesh = super::super::testutil::painted_mesh(&painted, 0.1);
        for (i, p) in mesh.positions.iter_mut().enumerate() {
            for (k, x) in p.iter_mut().enumerate() {
                let mut h = (i as u64 * 3 + k as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
                h ^= h >> 29;
                *x += ((h >> 11) as f64 / (1u64 << 53) as f64 - 0.5) as f32 * 0.008;
            }
        }
        let cfg = SurfaceConfig {
            page_size: 1024,
            ..SurfaceConfig::default()
        };
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{} faces: {stats:?}", mesh.indices.len());
        assert_charts(&tm, cfg.page_size);
        assert_pages_sane(&tm, cfg.page_size, mesh.indices.len());
        let per_face = stats.charts as f64 / stats.faces_textured as f64;
        assert!(per_face < 0.12, "{per_face} charts per face: {stats:?}");
        assert!(stats.fill_ratio > 0.45, "{stats:?}");
    }

    #[test]
    fn turned_chart_is_tight() {
        // The front patch seen with the camera rolled by 30°: its image is a turned rectangle,
        // whose axis-aligned box it would fill to about half.
        let mesh = patch(FRONT_Y, [-1.0, 1.0], [-0.5, 1.0], 0.1, -1.0);
        let faces = mesh.indices.len();
        let cams = cameras();
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let body = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 3.5 - cam_y, 0.0),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 30f64.to_radians(), 0.0),
        );
        let traj = Steps(vec![(0.0, body)]);
        let cam: ColorCamera = cams[0].clone();
        let world_cam = body * cam.cam_in_body;
        let shots = [Shot {
            camera: 0,
            stamp: T0 + 1.0,
            edit: Box::new(move |_| render_checker(&cam.camera, &world_cam)),
        }];
        let images = write_bag("atlas-turned", &Painted::new(vec![]), &traj, &cams, &shots);
        let src = TextureSource {
            images,
            poses: poses(&traj, 2.0),
            cams,
        };
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        assert!(labels.iter().all(|&l| l == Some(0)));
        let (tm, stats) = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        println!("{stats:?}");
        assert_eq!(stats.charts, 1, "{stats:?}");
        assert!(stats.fill_ratio > 0.75, "{stats:?}");
        assert_charts(&tm, cfg.page_size);
        assert!(check_fields(&mesh, &tm, faces) >= 40);
    }

    #[test]
    fn bake_needs_no_views_per_face() {
        // `texture_mesh` frees the views per face before the pages are allocated.
        let (mesh, src, _) = fixture("atlas-noviews", &[-0.15, 0.15], false);
        let cfg = SurfaceConfig::default();
        let progress = Progress::new();
        let mut views = collect_views(&mesh, &src, &cfg, &progress).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        let with = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        views.per_face = Vec::new();
        let without = bake(&mesh, &labels, &views, &src, &cfg, &progress).unwrap();
        assert_eq!(with, without);
    }

    #[test]
    fn bake_cancellable_and_checked() {
        let (mesh, src, _) = fixture("atlas-cancel", &[0.0], false);
        let cfg = SurfaceConfig::default();
        let views = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        let labels = choose_labels(&mesh, &views, &cfg);
        let p = Progress::new();
        let q = p.clone();
        let p = p.with_observer(move |s| {
            if s.phase == "Textur backen" {
                q.cancel();
            }
        });
        let r = bake(&mesh, &labels, &views, &src, &cfg, &p);
        assert!(matches!(r, Err(MapError::Cancelled)), "{r:?}");
        // Labels that do not fit the mesh are refused.
        let r = bake(&mesh, &labels[1..], &views, &src, &cfg, &Progress::new());
        assert!(matches!(r, Err(MapError::Config(_))), "{r:?}");
        // A page too small for the gray block and a chart's border.
        let tiny = SurfaceConfig {
            page_size: 8,
            ..cfg.clone()
        };
        assert!(tiny.check().is_err());
        // A needed image of the wrong size has its own message.
        let mut resized = src.clone();
        resized.cams[0].camera.width *= 2;
        let r = bake(&mesh, &labels, &views, &resized, &cfg, &Progress::new());
        match r {
            Err(MapError::Images(m)) => assert!(m.contains("falscher Größe"), "{m}"),
            other => panic!("{other:?}"),
        }
        // Images that went missing since the views were found.
        let mut gone = src.clone();
        gone.images.topics = vec!["/camera_nichts/image_raw".into()];
        let r = bake(&mesh, &labels, &views, &gone, &cfg, &Progress::new());
        assert!(matches!(r, Err(MapError::Images(_))), "{r:?}");
    }

    /// A 100 × 80 pinhole camera (90° wide) for the island tests.
    fn island_camera() -> ColorCamera {
        ColorCamera {
            camera: Camera {
                name: "test".into(),
                width: 100,
                height: 80,
                fx: 50.0,
                fy: 50.0,
                cx: 49.5,
                cy: 39.5,
                model: driftless_io::calib::CameraModel::Pinhole,
                max_theta: std::f64::consts::FRAC_PI_2,
            },
            cam_in_body: Isometry3::identity(),
        }
    }

    /// `open_wall(20, 0.1)` is the plane x = 0 over y, z in 0…2, normals along +x. A pose 3 m
    /// in front of its middle looking at it (`yaw` = −90°), or behind it looking at its back.
    fn wall_pose(x: f64, yaw: f64, roll: f64) -> Isometry3<f64> {
        use nalgebra::{Translation3, UnitQuaternion};
        let r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw);
        Isometry3::from_parts(Translation3::new(x, 1.0, 1.0), r)
    }

    fn island_views(poses: &[Isometry3<f64>], cameras: &[usize]) -> ViewTable {
        ViewTable {
            images: poses
                .iter()
                .zip(cameras)
                .enumerate()
                .map(|(i, (p, &camera))| UsedImage {
                    camera,
                    stamp: i as f64,
                    world_from_cam: *p,
                })
                .collect(),
            per_face: Vec::new(),
            gains: vec![[1.0; 3]; poses.len()],
            vignetting: Vignetting::flat(1, 4.0),
        }
    }

    /// Face `lower`/`upper` of the cell (`iy`, `iz`) of `open_wall(20, _)`.
    fn cell_face(iy: usize, iz: usize, upper: bool) -> usize {
        2 * (iy * 20 + iz) + usize::from(upper)
    }

    fn in_front() -> Isometry3<f64> {
        wall_pose(3.0, -std::f64::consts::FRAC_PI_2, 0.0)
    }

    #[test]
    fn island_without_view_takes_surrounding_image() {
        let mesh = open_wall(20, 0.1);
        let views = island_views(&[in_front(), in_front()], &[0, 0]);
        let mut labels = vec![Some(0); mesh.indices.len()];
        let mid = cell_face(10, 10, false);
        labels[mid] = Some(1);
        labels[0] = None;
        labels[1] = None;
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels);
        assert_eq!(n, 1);
        assert_eq!(labels[mid], Some(0));
        assert_eq!((labels[0], labels[1]), (None, None));
    }

    #[test]
    fn large_chart_stays() {
        let mesh = open_wall(20, 0.1);
        let views = island_views(&[in_front(), in_front()], &[0, 0]);
        let mut labels = vec![Some(0); mesh.indices.len()];
        for (iy, iz) in [(10, 10), (10, 11), (11, 10), (11, 11)] {
            labels[cell_face(iy, iz, false)] = Some(1);
            labels[cell_face(iy, iz, true)] = Some(1);
        }
        assert_eq!(ISLAND_FACES, 8);
        let before = labels.clone();
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels);
        assert_eq!(n, 0);
        assert_eq!(labels, before);
    }

    /// One island (label 1) in a wall of image 0, seen by the poses `poses` (images 0, 1).
    fn lone_island(poses: [Isometry3<f64>; 2]) -> usize {
        let mesh = open_wall(20, 0.1);
        let views = island_views(&poses, &[0, 0]);
        let mut labels = vec![Some(0); mesh.indices.len()];
        labels[cell_face(10, 10, false)] = Some(1);
        let cfg = SurfaceConfig::default();
        absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels)
    }

    #[test]
    fn island_outside_image_stays() {
        // Image 0 turned 90° about z: the wall lies at 90° from its optical axis.
        let turned = wall_pose(
            3.0,
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
        );
        assert_eq!(lone_island([in_front(), in_front()]), 1, "control");
        assert_eq!(lone_island([turned, in_front()]), 0);
    }

    #[test]
    fn island_facing_away_stays() {
        let behind = wall_pose(-3.0, std::f64::consts::FRAC_PI_2, 0.0);
        assert_eq!(lone_island([in_front(), in_front()]), 1, "control");
        assert_eq!(lone_island([behind, in_front()]), 0);
    }

    #[test]
    fn island_without_neighbour_image_stays() {
        let mesh = open_wall(20, 0.1);
        let views = island_views(&[in_front(), in_front()], &[0, 0]);
        let mut labels = vec![None; mesh.indices.len()];
        labels[cell_face(10, 10, false)] = Some(1);
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels);
        assert_eq!(n, 0);
        assert_eq!(labels[cell_face(10, 10, false)], Some(1));
    }

    #[test]
    fn island_takes_image_with_most_shared_edges() {
        // Image 1 on the side z < 1, image 0 beyond; the face (label 2) at the border has two
        // neighbours of image 1 and one of image 0.
        let mesh = open_wall(20, 0.1);
        let views = island_views(&[in_front(), in_front(), in_front()], &[0, 0, 0]);
        let mut labels: Vec<Option<u32>> = (0..mesh.indices.len())
            .map(|f| {
                Some(u32::from(
                    super::super::testutil::centroid(&mesh, f)[2] < 1.0,
                ))
            })
            .collect();
        let f = cell_face(10, 9, true);
        labels[f] = Some(2);
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels);
        assert_eq!(n, 1);
        assert_eq!(labels[f], Some(1));
    }

    #[test]
    fn unknown_camera_is_skipped() {
        let mesh = open_wall(20, 0.1);
        let views = island_views(&[in_front(), in_front()], &[0, 5]);
        // The island (image 0) lies in image 1, whose camera does not exist.
        let mut labels = vec![Some(1); mesh.indices.len()];
        labels[cell_face(10, 10, false)] = Some(0);
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&mesh, &views, &[island_camera()], &cfg, &mut labels);
        assert_eq!(n, 0);
    }

    #[test]
    fn empty_input_absorbs_nothing() {
        let views = island_views(&[], &[]);
        let cfg = SurfaceConfig::default();
        let n = absorb_islands(&Mesh::default(), &views, &[], &cfg, &mut []);
        assert_eq!(n, 0);
    }
}
