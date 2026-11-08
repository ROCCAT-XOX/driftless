//! Image noise damped per chart (M3): after baking and before the seams are levelled, an
//! edge-preserving filter (a guided filter) runs over the texture, every chart for itself and
//! never across the border of a chart.
//!
//! - Worked in luminance and two colour components (YCoCg on sRGB levels, `Y = (R + 2G + B) / 4`,
//!   `Co = (R − B) / 2`, `Cg = (2G − R − B) / 4`, exactly invertible, basic arithmetic only).
//! - The noise of a chart: `σ = 1.4826 · median |N ∗ Y| / 6` with Immerkær's 3×3 operator
//!   `N` over the texels of its triangles whose full 3×3 surroundings lie in them. A chart
//!   with fewer than [`MIN_NOISE_SAMPLES`] such texels takes the median of its image (over the
//!   charts of the image with an estimate of their own, weighted by their texels), and without
//!   such a chart the median of its page (all charts with any sample, weighted by samples).
//! - Luminance: window 5×5, guided by itself, `ε = (1.5 σ)²`. Colour: window 9×9, guided by the
//!   filtered luminance, `ε = (2 σ)²`.
//! - Strength: below `σ` = [`SIGMA_OFF`] the chart stays as it is (not a byte changes), from
//!   there to [`SIGMA_FULL`] the mix of filtered and original rises from 0 to 1. Bright, clean
//!   photos therefore stay practically untouched.
//! - The filter sees the texels of the chart's triangles only, and the rest of its rectangle is
//!   filled anew from them afterwards ([`fill_rest`]), as for the seams.
//!
//! A chart is cut into tiles of at most [`TILE`]² texels; each tile reads [`HALO`] texels
//! around itself, so the cut leaves no trace (all the filter's windows are within the halo)
//! and the memory stays bounded: per thread about 12 MB, per chart a copy of its rectangle
//! while it is written. Everything is basic arithmetic in a fixed order: same input, same bytes
//! on every machine.
//!
//! „Bilder neu“ passes its Stelle through the same filter ([`denoise_charts`]); without that the
//! new patch would be grainier than the surface around it.

use super::atlas::{fill_rest, for_each_texel, ChartInfo, FILL_BATCH_TEXELS};
use super::{Mesh, TexturedMesh};
use driftless_io::images::RgbImage;
use rayon::prelude::*;
use std::collections::BTreeMap;

/// A chart needs this many texels with a full 3×3 neighbourhood for an estimate of its own.
pub const MIN_NOISE_SAMPLES: u64 = 256;
/// Noise (standard deviation, sRGB levels) below which a chart stays as it is.
pub const SIGMA_OFF: f64 = 1.0;
/// Noise from which on a chart is filtered in full.
pub const SIGMA_FULL: f64 = 3.0;
/// Radius of the luminance window (5×5).
pub const LUMA_RADIUS: usize = 2;
/// Radius of the colour window (9×9).
pub const CHROMA_RADIUS: usize = 4;
/// `ε = (this · σ)²` for the luminance.
pub const LUMA_EPS: f64 = 1.5;
/// `ε = (this · σ)²` for the colour.
pub const CHROMA_EPS: f64 = 2.0;
/// Largest side of a tile (texels).
pub const TILE: usize = 512;
/// Texels a tile reads beyond its side: the colour of a texel depends on the filtered luminance
/// within `2 · CHROMA_RADIUS` of it, and that on the luminance `2 · LUMA_RADIUS` further on.
pub const HALO: usize = 2 * CHROMA_RADIUS + 2 * LUMA_RADIUS;
/// Bins of the histogram of `|N ∗ 4Y|` (the weights of `N` add up to ±8, `4Y` is at most 1020).
const BINS: usize = 8 * 1020 + 1;
/// From the median of `|N ∗ Y|` to the standard deviation of Gaussian noise: `N` has norm 6.
const SIGMA_PER_MEDIAN: f64 = 1.4826 / 6.0;

/// The image noise of a texture before and after [`denoise_charts`]: standard deviation
/// (sRGB levels, luminance), the median over the charts with an estimate of their own weighted
/// by their texels; 0 without such a chart.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct DenoiseStats {
    pub noise_before: f64,
    pub noise_after: f64,
}

/// What [`measure_charts`] found of a chart.
#[derive(Debug, Clone, Copy)]
struct Measured {
    /// Estimate from its own texels.
    sigma: f64,
    /// Texels with a full 3×3 neighbourhood in the chart.
    samples: u64,
    /// Texels of its triangles.
    texels: u64,
}

impl Measured {
    fn has_own_estimate(&self) -> bool {
        self.samples >= MIN_NOISE_SAMPLES
    }
}

/// Damps the image noise of every chart of `tex` for itself (see the module docs).
/// `image_of(c)`: the photo of chart `c`, for the charts too small for an estimate of their own.
/// Same input, byte-identical pages.
///
/// # Panics
/// If `tex` does not fit the mesh.
pub(crate) fn denoise_charts(
    mesh: &Mesh,
    tex: &mut TexturedMesh,
    image_of: impl Fn(usize) -> u32 + Sync,
) -> DenoiseStats {
    assert_eq!(
        tex.face_ref.len(),
        mesh.indices.len(),
        "texture does not fit the mesh"
    );
    let measured = measure_charts(tex);
    let sigma = chart_sigmas(tex, &measured, &image_of);
    let todo: Vec<usize> = (0..tex.charts.len())
        .filter(|&c| strength(sigma[c]) > 0.0)
        .collect();
    let mut after: Vec<f64> = measured.iter().map(|m| m.sigma).collect();
    // In batches, like the filling of the charts, so that not every copy is held at once.
    let mut from = 0;
    while from < todo.len() {
        let mut to = from;
        let mut texels = 0u64;
        while to < todo.len() && (to == from || texels < FILL_BATCH_TEXELS) {
            let [_, _, w, h] = tex.charts[todo[to]].rect;
            texels += u64::from(w) * u64::from(h);
            to += 1;
        }
        let batch = &todo[from..to];
        let shared: &TexturedMesh = tex;
        let filtered: Vec<(Vec<u8>, f64)> = batch
            .par_iter()
            .map(|&c| filter_chart(shared, &shared.charts[c], sigma[c], TILE))
            .collect();
        for (&c, (buf, noise)) in batch.iter().zip(filtered) {
            after[c] = noise;
            let chart = &tex.charts[c];
            let [x, y, w, _] = chart.rect.map(|v| v as usize);
            let image = &mut tex.pages[chart.page as usize].image;
            let stride = image.width as usize;
            for (r, row) in buf.chunks_exact(w * 3).enumerate() {
                let o = ((y + r) * stride + x) * 3;
                image.data[o..o + w * 3].copy_from_slice(row);
            }
        }
        from = to;
    }
    DenoiseStats {
        noise_before: noise_figure(&measured, |c| measured[c].sigma),
        noise_after: noise_figure(&measured, |c| after[c]),
    }
}

/// The noise of `tex` as [`denoise_charts`] reports it, without changing anything: for a run with
/// the filter off.
pub(crate) fn noise_of(tex: &TexturedMesh) -> f64 {
    let measured = measure_charts(tex);
    noise_figure(&measured, |c| measured[c].sigma)
}

/// The weighted median of the charts' noise (see [`DenoiseStats`]).
fn noise_figure(measured: &[Measured], sigma: impl Fn(usize) -> f64) -> f64 {
    let values = (0..measured.len())
        .filter(|&c| measured[c].has_own_estimate())
        .map(|c| (sigma(c), measured[c].texels))
        .collect();
    weighted_median(values).unwrap_or(0.0)
}

/// How much of the filtered image a chart with noise `sigma` takes (0 … 1).
fn strength(sigma: f64) -> f64 {
    ((sigma - SIGMA_OFF) / (SIGMA_FULL - SIGMA_OFF)).clamp(0.0, 1.0)
}

fn measure_charts(tex: &TexturedMesh) -> Vec<Measured> {
    tex.charts
        .par_iter()
        .map(|chart| {
            let mask = chart_mask(tex, chart);
            let image = &tex.pages[chart.page as usize].image;
            let [x, y, w, h] = chart.rect.map(|v| v as usize);
            let (sigma, samples) =
                estimate_noise(&image.data, image.width as usize, [x, y, w, h], &mask);
            Measured {
                sigma,
                samples,
                texels: mask.iter().filter(|&&m| m).count() as u64,
            }
        })
        .collect()
}

/// The noise of every chart: its own estimate, else the median of its image, else of its page,
/// else 0 (see the module docs).
fn chart_sigmas(
    tex: &TexturedMesh,
    measured: &[Measured],
    image_of: &(impl Fn(usize) -> u32 + Sync),
) -> Vec<f64> {
    let mut by_image: BTreeMap<u32, Vec<(f64, u64)>> = BTreeMap::new();
    let mut by_page: Vec<Vec<(f64, u64)>> = vec![Vec::new(); tex.pages.len()];
    for (c, m) in measured.iter().enumerate() {
        if m.has_own_estimate() {
            by_image
                .entry(image_of(c))
                .or_default()
                .push((m.sigma, m.texels));
        }
        if m.samples > 0 {
            by_page[tex.charts[c].page as usize].push((m.sigma, m.samples));
        }
    }
    let image_sigma: BTreeMap<u32, f64> = by_image
        .into_iter()
        .filter_map(|(image, values)| weighted_median(values).map(|s| (image, s)))
        .collect();
    let page_sigma: Vec<Option<f64>> = by_page.into_iter().map(weighted_median).collect();
    measured
        .iter()
        .enumerate()
        .map(|(c, m)| {
            if m.has_own_estimate() {
                m.sigma
            } else {
                image_sigma
                    .get(&image_of(c))
                    .copied()
                    .or(page_sigma[tex.charts[c].page as usize])
                    .unwrap_or(0.0)
            }
        })
        .collect()
}

/// The median of `values` (value, weight), `None` without any. Among equal values the weight
/// decides the order, so the result does not depend on the input order.
fn weighted_median(mut values: Vec<(f64, u64)>) -> Option<f64> {
    values.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let total: u64 = values.iter().map(|v| v.1).sum();
    let mut seen = 0u64;
    for &(value, weight) in &values {
        seen += weight;
        if seen * 2 >= total {
            return Some(value);
        }
    }
    values.last().map(|v| v.0)
}

/// Which texels of its rectangle (row-major) the triangles of `chart` cover.
fn chart_mask(tex: &TexturedMesh, chart: &ChartInfo) -> Vec<bool> {
    let [x, y, w, h] = chart.rect;
    let wu = w as usize;
    let page = &tex.pages[chart.page as usize];
    let size = [page.image.width, page.image.height].map(f64::from);
    let mut mask = vec![false; wu * h as usize];
    for &f in &chart.faces {
        let (_, t) = tex.face_ref[f as usize];
        let tri = page.indices[t as usize];
        let corners = [0, 1, 2].map(|k| {
            let uv = page.uvs[tri[k] as usize];
            [
                f64::from(uv[0]) * size[0] - f64::from(x),
                f64::from(uv[1]) * size[1] - f64::from(y),
            ]
        });
        for_each_texel(corners, w, h, |i, j, _| {
            mask[j as usize * wu + i as usize] = true;
        });
    }
    mask
}

/// `4Y = R + 2G + B` of the row `r` of the rectangle.
fn luma_row(data: &[u8], stride: usize, [x, y, w, _]: [usize; 4], r: usize, out: &mut [i32]) {
    let start = ((y + r) * stride + x) * 3;
    for (o, p) in out
        .iter_mut()
        .zip(data[start..start + w * 3].as_chunks::<3>().0)
    {
        *o = i32::from(p[0]) + 2 * i32::from(p[1]) + i32::from(p[2]);
    }
}

/// The noise of the rectangle `[x, y, w, h]` of `data` (`stride` texels per row, sRGB): the
/// estimate over the texels whose full 3×3 neighbourhood lies in `mask`, and how many there are
/// (see the module docs); 0 without any. An exact median, from a histogram of `|N ∗ 4Y|`
/// (integers).
fn estimate_noise(data: &[u8], stride: usize, rect: [usize; 4], mask: &[bool]) -> (f64, u64) {
    let [_, _, w, h] = rect;
    if w < 3 || h < 3 {
        return (0.0, 0);
    }
    let mut hist = vec![0u32; BINS];
    let mut rows: [Vec<i32>; 3] = std::array::from_fn(|_| vec![0; w]);
    for (r, row) in rows.iter_mut().enumerate() {
        luma_row(data, stride, rect, r, row);
    }
    let mut samples = 0u64;
    for y in 1..h - 1 {
        if y > 1 {
            rows.rotate_left(1);
            luma_row(data, stride, rect, y + 1, &mut rows[2]);
        }
        let [t, m, b] = &rows;
        let at = |dy: usize, x: usize| mask[(y + dy - 1) * w + x];
        for x in 1..w - 1 {
            if (0..3).all(|d| at(d, x - 1) && at(d, x) && at(d, x + 1)) {
                let n = t[x - 1] - 2 * t[x] + t[x + 1] - 2 * (m[x - 1] - 2 * m[x] + m[x + 1])
                    + b[x - 1]
                    - 2 * b[x]
                    + b[x + 1];
                hist[n.unsigned_abs() as usize] += 1;
                samples += 1;
            }
        }
    }
    if samples == 0 {
        return (0.0, 0);
    }
    let target = samples / 2;
    let mut seen = 0u64;
    for (v, &count) in hist.iter().enumerate() {
        seen += u64::from(count);
        if seen > target {
            return (SIGMA_PER_MEDIAN * v as f64 / 4.0, samples);
        }
    }
    (0.0, samples)
}

/// Where a chart is filtered from: its page (read only), rectangle and mask, noise and strength.
struct Source<'a> {
    image: &'a RgbImage,
    rect: [usize; 4],
    mask: &'a [bool],
    sigma: f64,
    mix: f32,
}

/// The rectangle of `chart` after the filter (row-major sRGB, the rest filled anew) and the
/// noise of its texels afterwards, in tiles of at most `tile` texels on a side.
fn filter_chart(tex: &TexturedMesh, chart: &ChartInfo, sigma: f64, tile: usize) -> (Vec<u8>, f64) {
    let mask = chart_mask(tex, chart);
    let image = &tex.pages[chart.page as usize].image;
    let rect = chart.rect.map(|v| v as usize);
    let [x, y, w, h] = rect;
    let stride = image.width as usize;
    let mut out = vec![0u8; w * h * 3];
    for (r, row) in out.chunks_exact_mut(w * 3).enumerate() {
        let o = ((y + r) * stride + x) * 3;
        row.copy_from_slice(&image.data[o..o + w * 3]);
    }
    let source = Source {
        image,
        rect,
        mask: &mask,
        sigma,
        mix: strength(sigma) as f32,
    };
    out.par_chunks_mut(w * 3 * tile)
        .enumerate()
        .for_each(|(band, rows)| {
            for x0 in (0..w).step_by(tile) {
                filter_tile(&source, [x0, band * tile], tile, rows);
            }
        });
    let (noise, _) = estimate_noise(&out, w, [0, 0, w, h], &mask);
    let mut filled = mask;
    fill_rest(&mut out, &mut filled, w, h);
    (out, noise)
}

/// Filters the tile at `[x0, y0]` (of the rectangle, at most `tile` on a side) into `band`,
/// the tile's rows of the chart's output, from a copy of its surroundings ([`HALO`] texels on
/// every side, as far as the rectangle goes).
fn filter_tile(source: &Source, [x0, y0]: [usize; 2], tile: usize, band: &mut [u8]) {
    let [rx, ry, w, h] = source.rect;
    let (x1, y1) = ((x0 + tile).min(w), (y0 + tile).min(h));
    let (bx0, by0) = (x0.saturating_sub(HALO), y0.saturating_sub(HALO));
    let (bx1, by1) = ((x1 + HALO).min(w), (y1 + HALO).min(h));
    let (bw, bh) = (bx1 - bx0, by1 - by0);
    let at = |x: usize, y: usize| (y - by0) * bw + (x - bx0);
    let stride = source.image.width as usize;
    let data = &source.image.data;

    let mut luma = vec![0f32; bw * bh];
    let mut co = vec![0f32; bw * bh];
    let mut cg = vec![0f32; bw * bh];
    let mut mask = vec![false; bw * bh];
    for y in by0..by1 {
        for x in bx0..bx1 {
            if !source.mask[y * w + x] {
                continue;
            }
            let o = ((ry + y) * stride + rx + x) * 3;
            let [r, g, b] = [data[o], data[o + 1], data[o + 2]].map(f32::from);
            let k = at(x, y);
            luma[k] = (r + 2.0 * g + b) / 4.0;
            co[k] = (r - b) / 2.0;
            cg[k] = (2.0 * g - r - b) / 4.0;
            mask[k] = true;
        }
    }

    let eps = |factor: f64| {
        let e = factor * source.sigma;
        e * e
    };
    let dims = (bw, bh);
    let filtered_luma = guided(&mask, dims, &luma, &[&luma], LUMA_RADIUS, eps(LUMA_EPS))
        .pop()
        .expect("one channel");
    let chroma = guided(
        &mask,
        dims,
        &filtered_luma,
        &[&co, &cg],
        CHROMA_RADIUS,
        eps(CHROMA_EPS),
    );

    let mix = source.mix;
    let blend = |a: f32, b: f32| a + mix * (b - a);
    for y in y0..y1 {
        for x in x0..x1 {
            if !source.mask[y * w + x] {
                continue;
            }
            let k = at(x, y);
            let l = blend(luma[k], filtered_luma[k]);
            let (c, g) = (blend(co[k], chroma[0][k]), blend(cg[k], chroma[1][k]));
            let t = l - g;
            let rgb = [t + c, l + g, t - c];
            let o = ((y - y0) * w + x) * 3;
            for (byte, v) in band[o..o + 3].iter_mut().zip(rgb) {
                *byte = v.round().clamp(0.0, 255.0) as u8;
            }
        }
    }
}

/// The guided filter over a `w × h` field, on the texels of `mask` only: for each `inputs[k]`,
/// `q_i = mean(a) · guide_i + mean(b)` with, per window of `(2r + 1)²` around a mask texel,
/// `a = cov(guide, input) / (var(guide) + eps)` and `b = mean(input) − a · mean(guide)`, the means
/// over the mask texels of the window. Texels outside the mask are 0.
fn guided(
    mask: &[bool],
    (w, h): (usize, usize),
    guide: &[f32],
    inputs: &[&[f32]],
    r: usize,
    eps: f64,
) -> Vec<Vec<f32>> {
    let n = inputs.len();
    let mut ab = vec![0f32; w * h * 2 * n];
    for_each_window(
        (w, h),
        r,
        3 + 2 * n,
        |i, v| {
            v.fill(0.0);
            if mask[i] {
                let g = f64::from(guide[i]);
                v[0] = 1.0;
                v[1] = g;
                v[2] = g * g;
                for (k, input) in inputs.iter().enumerate() {
                    let p = f64::from(input[i]);
                    v[3 + 2 * k] = p;
                    v[4 + 2 * k] = g * p;
                }
            }
        },
        |i, s| {
            if !mask[i] {
                return;
            }
            let count = s[0];
            let mean_g = s[1] / count;
            let var = s[2] / count - mean_g * mean_g;
            for k in 0..n {
                let mean_p = s[3 + 2 * k] / count;
                let cov = s[4 + 2 * k] / count - mean_g * mean_p;
                let a = cov / (var + eps);
                ab[i * 2 * n + 2 * k] = a as f32;
                ab[i * 2 * n + 2 * k + 1] = (mean_p - a * mean_g) as f32;
            }
        },
    );
    let mut out = vec![vec![0f32; w * h]; n];
    for_each_window(
        (w, h),
        r,
        1 + 2 * n,
        |i, v| {
            v.fill(0.0);
            if mask[i] {
                v[0] = 1.0;
                for (slot, &x) in v[1..].iter_mut().zip(&ab[i * 2 * n..(i + 1) * 2 * n]) {
                    *slot = f64::from(x);
                }
            }
        },
        |i, s| {
            if mask[i] {
                let g = f64::from(guide[i]);
                for (k, channel) in out.iter_mut().enumerate() {
                    channel[i] = ((s[1 + 2 * k] * g + s[2 + 2 * k]) / s[0]) as f32;
                }
            }
        },
    );
    out
}

/// Calls `out(i, sums)` for every texel `i` (row-major) of a `w × h` field with the sums over the
/// window of `(2r + 1)²` around it, cut off at the field's edge, of the `fields` values `value`
/// gives for a texel. Running sums: one per column, one over the row, so the cost does not depend
/// on `r`.
fn for_each_window(
    (w, h): (usize, usize),
    r: usize,
    fields: usize,
    value: impl Fn(usize, &mut [f64]),
    mut out: impl FnMut(usize, &[f64]),
) {
    let mut columns = vec![0f64; w * fields];
    let mut v = vec![0f64; fields];
    let mut add_row = |columns: &mut [f64], row: usize, sign: f64| {
        for (x, column) in columns.chunks_exact_mut(fields).enumerate() {
            value(row * w + x, &mut v);
            for (c, x) in column.iter_mut().zip(&v) {
                *c += sign * x;
            }
        }
    };
    for row in 0..(r + 1).min(h) {
        add_row(&mut columns, row, 1.0);
    }
    let mut sum = vec![0f64; fields];
    for y in 0..h {
        if y > 0 {
            if y + r < h {
                add_row(&mut columns, y + r, 1.0);
            }
            if y > r {
                add_row(&mut columns, y - r - 1, -1.0);
            }
        }
        sum.fill(0.0);
        for column in columns.chunks_exact(fields).take(r + 1) {
            for (s, c) in sum.iter_mut().zip(column) {
                *s += c;
            }
        }
        for x in 0..w {
            out(y * w + x, &sum);
            if x + r + 1 < w {
                for (s, c) in sum.iter_mut().zip(&columns[(x + r + 1) * fields..]) {
                    *s += c;
                }
            }
            if x >= r {
                for (s, c) in sum.iter_mut().zip(&columns[(x - r) * fields..]) {
                    *s -= c;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::atlas::{fill_rest, ChartInfo, TexturePage, GRAY_BLOCK, MARGIN, UNSEEN_GRAY};
    use super::super::testutil::Lcg;
    use super::*;
    use driftless_io::images::RgbImage;

    /// Charts on one page of `size²` texels, laid out as `bake` leaves them: every chart a square
    /// of `side²` texels (two triangles that cover exactly those texels) with [`MARGIN`] around
    /// it, the [`GRAY_BLOCK`] at the origin, the rest of the page [`UNSEEN_GRAY`].
    struct Page {
        mesh: Mesh,
        tex: TexturedMesh,
        images: Vec<u32>,
    }

    impl Page {
        fn new(size: u32) -> Page {
            Page {
                mesh: Mesh::default(),
                tex: TexturedMesh {
                    pages: vec![TexturePage {
                        positions: Vec::new(),
                        uvs: Vec::new(),
                        indices: Vec::new(),
                        image: RgbImage {
                            width: size,
                            height: size,
                            data: vec![UNSEEN_GRAY; (size * size * 3) as usize],
                        },
                    }],
                    face_ref: Vec::new(),
                    textured: Vec::new(),
                    charts: Vec::new(),
                },
                images: Vec::new(),
            }
        }

        /// A chart at `origin` (its rectangle's corner) from the image `image`; its index.
        fn chart(&mut self, origin: [u32; 2], side: u32, image: u32) -> usize {
            self.chart_on(0, origin, side, image)
        }

        fn chart_on(&mut self, p: usize, origin: [u32; 2], side: u32, image: u32) -> usize {
            let size = self.tex.pages[p].image.width as f32;
            let page = &mut self.tex.pages[p];
            let base = page.positions.len() as u32;
            let m = MARGIN as f32 + 0.5;
            let (x, y) = (origin[0] as f32 + m, origin[1] as f32 + m);
            let s = side as f32 - 1.0;
            for (dx, dy) in [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)] {
                page.positions.push([x + dx, y + dy, 0.0]);
                page.uvs.push([(x + dx) / size, (y + dy) / size]);
            }
            let mesh_base = self.mesh.positions.len() as u32;
            self.mesh
                .positions
                .extend(page.positions[base as usize..].iter().copied());
            let mut faces = Vec::new();
            for tri in [[0, 1, 2], [0, 2, 3]] {
                let f = self.mesh.indices.len() as u32;
                self.mesh.indices.push(tri.map(|k: u32| mesh_base + k));
                self.tex
                    .face_ref
                    .push((p as u32, page.indices.len() as u32));
                self.tex.textured.push(true);
                page.indices.push(tri.map(|k: u32| base + k));
                faces.push(f);
            }
            let w = side + 2 * MARGIN;
            self.tex.charts.push(ChartInfo {
                page: p as u32,
                rect: [origin[0], origin[1], w, w],
                faces,
            });
            self.images.push(image);
            self.tex.charts.len() - 1
        }

        /// The texels of chart `c`'s square from `colour(x, y)` (within the square), the rest
        /// of its rectangle as `bake` fills it.
        fn paint(&mut self, c: usize, mut colour: impl FnMut(u32, u32) -> [u8; 3]) {
            let chart = &self.tex.charts[c];
            let [x0, y0, w, h] = chart.rect;
            let image = &mut self.tex.pages[chart.page as usize].image;
            let side = image.width;
            let (wu, hu) = (w as usize, h as usize);
            let mut buf = vec![UNSEEN_GRAY; wu * hu * 3];
            let mut filled = vec![false; wu * hu];
            for j in MARGIN..h - MARGIN {
                for i in MARGIN..w - MARGIN {
                    let at = j as usize * wu + i as usize;
                    buf[at * 3..at * 3 + 3].copy_from_slice(&colour(i - MARGIN, j - MARGIN));
                    filled[at] = true;
                }
            }
            fill_rest(&mut buf, &mut filled, wu, hu);
            for r in 0..hu {
                let o = ((y0 as usize + r) * side as usize + x0 as usize) * 3;
                image.data[o..o + wu * 3].copy_from_slice(&buf[r * wu * 3..(r + 1) * wu * 3]);
            }
        }

        /// The colours of chart `c`'s square, row by row.
        fn inner(&self, c: usize) -> Vec<[u8; 3]> {
            let chart = &self.tex.charts[c];
            let [x0, y0, w, h] = chart.rect;
            let image = &self.tex.pages[chart.page as usize].image;
            let mut out = Vec::new();
            for y in y0 + MARGIN..y0 + h - MARGIN {
                for x in x0 + MARGIN..x0 + w - MARGIN {
                    out.push(texel(image, x, y));
                }
            }
            out
        }

        fn denoise(&mut self) -> DenoiseStats {
            let images = self.images.clone();
            denoise_charts(&self.mesh, &mut self.tex, |c| images[c])
        }

        fn image(&self) -> &RgbImage {
            &self.tex.pages[0].image
        }
    }

    fn texel(image: &RgbImage, x: u32, y: u32) -> [u8; 3] {
        let o = ((y * image.width + x) * 3) as usize;
        [image.data[o], image.data[o + 1], image.data[o + 2]]
    }

    fn level(v: f64) -> u8 {
        v.round().clamp(0.0, 255.0) as u8
    }

    /// `base` plus the same noise of standard deviation `sigma` on all three channels.
    fn gray_noise(base: [u8; 3], sigma: f64, rng: &mut Lcg) -> [u8; 3] {
        let n = sigma * rng.normal();
        base.map(|v| level(f64::from(v) + n))
    }

    /// `base` plus independent noise of standard deviation `sigma` on every channel.
    fn colour_noise(base: [u8; 3], sigma: f64, rng: &mut Lcg) -> [u8; 3] {
        base.map(|v| level(f64::from(v) + sigma * rng.normal()))
    }

    fn luma(c: [u8; 3]) -> f64 {
        (f64::from(c[0]) + 2.0 * f64::from(c[1]) + f64::from(c[2])) / 4.0
    }

    fn co(c: [u8; 3]) -> f64 {
        (f64::from(c[0]) - f64::from(c[2])) / 2.0
    }

    fn cg(c: [u8; 3]) -> f64 {
        (2.0 * f64::from(c[1]) - f64::from(c[0]) - f64::from(c[2])) / 4.0
    }

    fn deviation(values: &[f64]) -> f64 {
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        (values.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / values.len() as f64).sqrt()
    }

    fn deviation_of(colours: &[[u8; 3]], f: impl Fn([u8; 3]) -> f64) -> f64 {
        deviation(&colours.iter().map(|&c| f(c)).collect::<Vec<_>>())
    }

    const FLAT: [u8; 3] = [120, 110, 100];

    /// One chart of `side²` texels on a page of its own, painted by `colour`.
    fn one_chart(side: u32, colour: impl FnMut(u32, u32) -> [u8; 3]) -> Page {
        let mut page = Page::new(side + 64);
        let c = page.chart([GRAY_BLOCK, 0], side, 0);
        page.paint(c, colour);
        page
    }

    #[test]
    fn flat_noise_goes_down() {
        let mut rng = Lcg::new(1);
        let mut page = one_chart(96, |_, _| gray_noise(FLAT, 6.0, &mut rng));
        let before = deviation_of(&page.inner(0), luma);
        let stats = page.denoise();
        let after = deviation_of(&page.inner(0), luma);
        println!("{before:.2} -> {after:.2}, {stats:?}");
        assert!((before - 6.0).abs() < 0.6, "{before}");
        assert!(after <= 0.5 * before, "{before} -> {after}");
        // The figures say the same: about the noise that was added, then about half of it.
        assert!((stats.noise_before - 6.0).abs() < 1.0, "{stats:?}");
        assert!(stats.noise_after <= 0.5 * stats.noise_before, "{stats:?}");
    }

    #[test]
    fn edges_stay_sharp() {
        // Stripes two texels wide, 40 levels apart, under noise that switches the filter fully on.
        let (dark, light) = ([100u8; 3], [140u8; 3]);
        let mut rng = Lcg::new(2);
        let mut page = one_chart(96, |x, _| {
            gray_noise(if (x / 2) % 2 == 0 { dark } else { light }, 4.0, &mut rng)
        });
        let contrast = |page: &Page| {
            let inner = page.inner(0);
            let (mut sum, mut count) = ([0.0f64; 2], [0.0f64; 2]);
            for (i, &c) in inner.iter().enumerate() {
                let (x, y) = (i % 96, i / 96);
                // Away from the ends of the stripes.
                if (8..88).contains(&x) && (8..88).contains(&y) {
                    let k = (x / 2) % 2;
                    sum[k] += luma(c);
                    count[k] += 1.0;
                }
            }
            sum[1] / count[1] - sum[0] / count[0]
        };
        let before = contrast(&page);
        let stats = page.denoise();
        let after = contrast(&page);
        println!("{before:.2} -> {after:.2}, {stats:?}");
        assert!(stats.noise_after < stats.noise_before, "{stats:?}");
        assert!(after >= 0.8 * 40.0, "contrast {before} -> {after}");
    }

    #[test]
    fn chroma_noise_goes_down_more() {
        let mut rng = Lcg::new(3);
        let mut page = one_chart(96, |_, _| colour_noise(FLAT, 6.0, &mut rng));
        let (y0, co0, cg0) = {
            let inner = page.inner(0);
            (
                deviation_of(&inner, luma),
                deviation_of(&inner, co),
                deviation_of(&inner, cg),
            )
        };
        page.denoise();
        let inner = page.inner(0);
        let (y1, co1, cg1) = (
            deviation_of(&inner, luma),
            deviation_of(&inner, co),
            deviation_of(&inner, cg),
        );
        println!("Y {y0:.2} -> {y1:.2}, Co {co0:.2} -> {co1:.2}, Cg {cg0:.2} -> {cg1:.2}");
        assert!(y1 < y0, "{y0} -> {y1}");
        assert!(co1 / co0 < y1 / y0, "Co {co0} -> {co1}, Y {y0} -> {y1}");
        assert!(cg1 / cg0 < y1 / y0, "Cg {cg0} -> {cg1}, Y {y0} -> {y1}");
    }

    #[test]
    fn clean_chart_untouched() {
        // A clean photo: a gentle gradient and noise well below one level.
        let mut rng = Lcg::new(4);
        let mut page = one_chart(96, |x, y| {
            let base = 100.0 + 0.2 * f64::from(x) + 0.1 * f64::from(y);
            [level(base + 0.4 * rng.normal()); 3]
        });
        let before = page.image().clone();
        let stats = page.denoise();
        assert_eq!(page.image(), &before);
        assert!(stats.noise_before < 1.0, "{stats:?}");
        assert_eq!(stats.noise_after, stats.noise_before);
    }

    #[test]
    fn charts_do_not_mix() {
        // Two charts side by side in clearly different colours, the rectangles touching.
        let side = 96;
        let mut page = Page::new(256);
        let w = side + 2 * MARGIN;
        let a = page.chart([GRAY_BLOCK, 0], side, 0);
        let b = page.chart([GRAY_BLOCK + w, 0], side, 1);
        let (red, blue) = ([200u8, 40, 40], [40u8, 40, 200]);
        let mut rng = Lcg::new(5);
        page.paint(a, |_, _| colour_noise(red, 6.0, &mut rng));
        page.paint(b, |_, _| colour_noise(blue, 6.0, &mut rng));
        let before = page.image().clone();
        page.denoise();
        let image = page.image();
        assert_ne!(image, &before, "nothing was filtered");
        let in_rect = |x: u32, y: u32, r: [u32; 4]| {
            x >= r[0] && x < r[0] + r[2] && y >= r[1] && y < r[1] + r[3]
        };
        let (ra, rb) = (page.tex.charts[a].rect, page.tex.charts[b].rect);
        for y in 0..image.height {
            for x in 0..image.width {
                let c = texel(image, x, y);
                if in_rect(x, y, ra) {
                    // Red, in the border too: nothing of the blue chart next to it.
                    assert!(c[0] > 150 && c[2] < 90, "({x}, {y}): {c:?}");
                } else if in_rect(x, y, rb) {
                    assert!(c[2] > 150 && c[0] < 90, "({x}, {y}): {c:?}");
                } else {
                    // The gray block and everything outside the rectangles stay.
                    assert_eq!(c, texel(&before, x, y), "({x}, {y})");
                }
            }
        }
    }

    #[test]
    fn texels_outside_the_triangles_do_not_count() {
        // Only one triangle of the square belongs to the chart. Whatever stands next to it must
        // not reach it: noisy on both sides, so that it is the filter that has to tell them
        // apart, not only the estimate of the noise.
        let side = 96u32;
        let build = |outside: [u8; 3]| {
            let mut page = Page::new(256);
            let c = page.chart([GRAY_BLOCK, 0], side, 0);
            let f = page.tex.charts[c].faces[0];
            page.tex.charts[c].faces = vec![f];
            let mut rng = Lcg::new(6);
            page.paint(c, |x, y| {
                let inside = gray_noise(FLAT, 6.0, &mut rng);
                let beyond = gray_noise(outside, 6.0, &mut rng);
                if x >= y {
                    inside
                } else {
                    beyond
                }
            });
            page
        };
        let (mut green, mut dark) = (build([40, 200, 40]), build([20, 20, 20]));
        let in_triangle = |colours: &[[u8; 3]]| -> Vec<f64> {
            let side = side as usize;
            (0..side)
                .flat_map(|y| (y + 2..side).map(move |x| y * side + x))
                .map(|i| luma(colours[i]))
                .collect()
        };
        let before = deviation(&in_triangle(&green.inner(0)));
        green.denoise();
        dark.denoise();
        let (a, b) = (green.inner(0), dark.inner(0));
        let after = deviation(&in_triangle(&a));
        println!("{before:.2} -> {after:.2}");
        assert!(after < 0.6 * before, "not filtered: {before} -> {after}");
        // The filtered texels know nothing of what stood outside their triangle (the diagonal
        // itself and half a texel beyond it belong to the triangle).
        let n = side as usize;
        for y in 0..n {
            for x in y + 2..n {
                assert_eq!(a[y * n + x], b[y * n + x], "({x}, {y})");
            }
        }
    }

    #[test]
    fn border_is_filled_anew() {
        let mut rng = Lcg::new(7);
        let mut page = one_chart(96, |_, _| gray_noise(FLAT, 6.0, &mut rng));
        page.denoise();
        // The rest of the rectangle comes from the filtered texels: as `fill_rest` makes it.
        let chart = page.tex.charts[0].clone();
        let [x0, y0, w, h] = chart.rect;
        let (wu, hu) = (w as usize, h as usize);
        let image = page.image();
        let mut buf = vec![UNSEEN_GRAY; wu * hu * 3];
        let mut filled = vec![false; wu * hu];
        for j in MARGIN..h - MARGIN {
            for i in MARGIN..w - MARGIN {
                let at = j as usize * wu + i as usize;
                buf[at * 3..at * 3 + 3].copy_from_slice(&texel(image, x0 + i, y0 + j));
                filled[at] = true;
            }
        }
        fill_rest(&mut buf, &mut filled, wu, hu);
        for j in 0..h {
            for i in 0..w {
                let at = (j as usize * wu + i as usize) * 3;
                assert_eq!(
                    texel(image, x0 + i, y0 + j),
                    [buf[at], buf[at + 1], buf[at + 2]]
                );
            }
        }
    }

    /// A chart too small for an estimate takes the noise of the large charts of its image.
    #[test]
    fn small_chart_uses_image_noise() {
        // Small: 8² texels, fewer than the samples of an estimate of its own.
        let small = 8;
        let mut page = Page::new(512);
        let big_noisy = page.chart([GRAY_BLOCK, 0], 96, 0);
        let small_of_noisy = page.chart([GRAY_BLOCK + 100, 0], small, 0);
        let big_clean = page.chart([GRAY_BLOCK, 120], 96, 1);
        let small_of_clean = page.chart([GRAY_BLOCK + 100, 120], small, 1);
        let mut rng = Lcg::new(8);
        page.paint(big_noisy, |_, _| gray_noise(FLAT, 6.0, &mut rng));
        page.paint(big_clean, |_, _| FLAT);
        let mut stripes = Lcg::new(9);
        for c in [small_of_noisy, small_of_clean] {
            page.paint(c, |_, _| gray_noise(FLAT, 6.0, &mut stripes));
        }
        let noisy_before = page.inner(small_of_noisy);
        let clean_before = page.inner(small_of_clean);
        page.denoise();
        // Filtered as its image is noisy …
        let noisy_after = page.inner(small_of_noisy);
        assert_ne!(noisy_after, noisy_before);
        assert!(deviation_of(&noisy_after, luma) < 0.75 * deviation_of(&noisy_before, luma));
        // … left alone as its image is clean, however noisy it looks for itself.
        assert_eq!(page.inner(small_of_clean), clean_before);
    }

    #[test]
    fn small_chart_without_a_large_one_of_its_image_uses_the_page() {
        let mut page = Page::new(512);
        let big = page.chart([GRAY_BLOCK, 0], 96, 0);
        // Image 7 has no other chart here: its small chart takes the noise of the page.
        let small = page.chart([GRAY_BLOCK + 100, 0], 8, 7);
        let mut rng = Lcg::new(10);
        page.paint(big, |_, _| gray_noise(FLAT, 6.0, &mut rng));
        page.paint(small, |_, _| gray_noise(FLAT, 6.0, &mut rng));
        let before = page.inner(small);
        page.denoise();
        assert_ne!(page.inner(small), before);
    }

    /// Cutting a chart into tiles leaves no trace: the halo reaches every window (up to the
    /// rounding of the running sums).
    #[test]
    fn tiles_leave_no_trace() {
        let side = 200;
        let mut rng = Lcg::new(12);
        let page = one_chart(side, |x, y| {
            let base = if (x / 7 + y / 11) % 2 == 0 { 70 } else { 150 };
            colour_noise([base, base + 10, base - 10], 6.0, &mut rng)
        });
        let chart = &page.tex.charts[0];
        let whole = filter_chart(&page.tex, chart, 5.0, (side + 2 * MARGIN) as usize);
        for tile in [16, 64, 101] {
            let cut = filter_chart(&page.tex, chart, 5.0, tile);
            let differ = whole.0.iter().zip(&cut.0).filter(|(a, b)| a != b).count();
            let largest = whole
                .0
                .iter()
                .zip(&cut.0)
                .map(|(&a, &b)| a.abs_diff(b))
                .max()
                .unwrap();
            println!("tile {tile}: {differ} bytes differ, by at most {largest}");
            assert!(largest <= 1, "tile {tile}: {largest}");
            assert!(
                differ * 1000 <= whole.0.len(),
                "tile {tile}: {differ} bytes differ"
            );
        }
    }

    #[test]
    fn denoise_is_deterministic() {
        let build = || {
            let mut rng = Lcg::new(11);
            let mut page = Page::new(512);
            let a = page.chart([GRAY_BLOCK, 0], 96, 0);
            let b = page.chart([GRAY_BLOCK + 110, 0], 96, 1);
            page.paint(a, |_, _| colour_noise(FLAT, 6.0, &mut rng));
            page.paint(b, |x, _| {
                gray_noise([60 + (x / 3) as u8 * 2; 3], 3.0, &mut rng)
            });
            page
        };
        let run = |threads: usize| {
            let mut page = build();
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap();
            let stats = pool.install(|| page.denoise());
            (page.tex.pages[0].image.data.clone(), stats)
        };
        let one = run(1);
        assert_eq!(run(1), one);
        assert_eq!(run(4), one);
    }
}
