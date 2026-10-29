//! Seam levelling: evens out the brightness jumps where charts of different images meet.
//!
//! Every chart gets a gain per mesh vertex and channel (linear RGB, solved as its logarithm):
//!
//! 1. **Border vertices** are vertices where charts meet along an edge whose two faces bend by
//!    at most 45° ([`SEAM_MIN_COS`]; across a crease the light differs). Both charts are
//!    sampled at the same spots of every such edge next to the vertex: on the edge itself, at
//!    [`SEAM_SAMPLES`] of its length from the vertex (bilinear; a chart's texels reach half a
//!    texel beyond its triangles, so on the edge both charts show the same spot of the
//!    surface). Per pair of charts at the vertex: the mean colour of each. If their
//!    luminances differ by more than [`MAX_SEAM_RATIO`] (a side darker than
//!    [`MIN_SEAM_LUMINANCE`] counts as that dark), the difference is content (a highlight, an
//!    occlusion) rather than exposure and the pair is left out. Each chart at the vertex is
//!    then pulled towards the others: its log gain is minus the sum of its log ratios
//!    `ln((own + GAIN_FLOOR) / (other + GAIN_FLOOR))` to the charts it shares an edge with
//!    there, divided by the number of charts at the vertex (two charts meet halfway), clamped
//!    to [`MAX_GAIN`] either way. A vertex without such a pair is treated as interior.
//! 2. **Interior vertices** take the harmonic interpolation of the border log gains over the
//!    chart's edge graph (every free vertex the mean of its edge neighbours, border vertices
//!    fixed), solved per channel with conjugate gradients until the largest per-vertex
//!    residual (the change one Jacobi step would still make) is below [`SOLVE_TOLERANCE`], at
//!    most [`SOLVE_ROUNDS_PER_VERTEX`] rounds per free vertex. A chart bordering another on
//!    one side only is thus scaled as a whole, without a halo; it stays within the bound of
//!    its border gains (maximum principle). Parts of a chart without a border vertex keep 1.
//! 3. **Texels.** Every texel of a chart triangle ([`for_each_texel`], first face in face order
//!    wins on shared edges) is multiplied by the barycentric mix of its corners' log gains,
//!    exponentiated; a texel whose log gain is exactly 0 keeps its bytes. Then the rest of the
//!    chart's rectangle is filled anew with [`fill_rest`] (mask: the triangle texels), so no
//!    old colour stays in the border. A chart without any correction is left alone.
//!
//! Why a gain (Task 7 used an offset): exposures differ by a factor, and a chart's interior
//! has other colours than its seam. An offset measured on a light spot of the seam darkens a
//! dark stripe next to it far more than its share: on stripes across a seam with a 30 % step,
//! the offset raised the largest step along the seam from 19 to 51 sRGB levels (the old
//! window median at the vertex: it compared different spots of the surface), the gain lowers
//! it to 1. [`MAX_GAIN`] bounds how much a dark texel's noise can grow; black stays black and
//! white clips at white.
//!
//! Limits, from the black room (paired samples on 640,114 seam spots): the views of one face
//! agree within 3.4 sRGB levels (median, 1/4-scale face means after the exposure fit), but on
//! the seams 34 % of the spots differ by more than 2× (content at the seam, view-dependent
//! glow). With `MAX_SEAM_RATIO` 2 the median step on flat seams fell from 5.7 to 2.4 levels
//! (p90 37 → 29), with 3 to 2.2 (p90 → 17), 4 brought little more; across creases (> 45°,
//! 13 % of the spots) the step is mostly shading and is left alone.
//! So content up to 3× (a speck, a stripe end at the seam) is treated as exposure: the old
//! limit of 2 left more seams alone (p90 29 instead of 17 levels).
//!
//! Unseen (gray) faces belong to no chart: they neither give nor take a correction.
//!
//! A Stelle baked anew onto pages of its own ([`level_region`]) is levelled the same way, but
//! the texture around it stays: where a chart of the Stelle meets it, only the chart moves, by
//! the whole difference.
//!
//! Two seam measures ([`SeamStats`]), sRGB luminance levels: **jump** follows `seam_jumps` of
//! the point colouring: every mesh edge between two textured faces is sampled in both faces
//! just inside the edge ([`EDGE_SAMPLE`]); the mean absolute difference of the pairs from
//! different charts, minus that of the pairs within one chart (0 if either kind is missing).
//! Content next to the edge enters it (a stripe ending at the seam). **Step**: the mean absolute
//! difference of both sides on the edge itself (at [`SEAM_SAMPLES`] from either end), over the
//! edges between faces of different images, creases included (they are not levelled; in the
//! black room 13 % of the seam spots): what levelling aims at; what remains is
//! misregistration, noise and the creases. The step is also given per metre of seam
//! ([`SeamStats::seam_step_per_m`], every sample weighted by the length of its edge): the plain
//! mean takes the same samples from every edge, so a fine mesh puts more of them where its
//! faces are small, and a merged model with larger faces looked better or worse than a
//! single scan by its face size alone.
//!
//! Memory: per chart its vertices with their corrections (≈ 32 bytes per vertex and chart, and
//! the solver's vectors for one chart at a time per thread), per mesh edge between textured
//! faces 16 bytes, per seam edge two paired samples (≈ 120 bytes, freed after step 1), and per
//! corrected chart one copy of its rectangle while it is being written.

use super::atlas::{fill_rest, for_each_texel, TexturePage};
use super::{Mesh, TexturedMesh};
use crate::color::{linear_table, luminance, sample_linear, srgb_value};
use rayon::prelude::*;

/// The solver stops once no free vertex's log gain would change by more than this.
pub const SOLVE_TOLERANCE: f64 = 1e-5;
/// At most this many solver rounds per free vertex of a chart (conjugate gradients reach the
/// exact solution within one round per vertex, in exact arithmetic).
pub const SOLVE_ROUNDS_PER_VERTEX: usize = 2;
/// Two charts at a border vertex whose luminances differ by more than this factor are not
/// levelled against each other: exposures differ by less, so the difference is content (a
/// highlight, an occlusion).
pub const MAX_SEAM_RATIO: f64 = 3.0;
/// Largest gain of a border vertex either way (per channel): √[`MAX_SEAM_RATIO`], so that two
/// charts just within it meet halfway.
pub const MAX_GAIN: f64 = 1.732_050_807_568_877_2;
/// Charts meeting at an edge whose faces' normals differ by more than this (45°, as cosine)
/// are not levelled against each other: across a crease the light differs.
pub const SEAM_MIN_COS: f32 = std::f32::consts::FRAC_1_SQRT_2;
/// Added to both colours of a ratio (linear), so that a near-black sample gives no wild gain.
pub const GAIN_FLOOR: f64 = 0.0005;
/// Luminance (linear) below which a side counts as this dark for [`MAX_SEAM_RATIO`].
pub const MIN_SEAM_LUMINANCE: f64 = 0.01;
/// Where the seam measure samples a face next to one of its edges: barycentric weights of the
/// edge's two corners and of the opposite corner.
pub const EDGE_SAMPLE: [f64; 3] = [0.45, 0.45, 0.1];
/// Where a seam is sampled for the correction of its end vertices: these shares of the way
/// from the vertex along the edge, on the edge itself in both charts.
pub const SEAM_SAMPLES: [f64; 3] = [0.125, 0.25, 0.375];
/// "No chart".
const NONE: u32 = u32::MAX;

/// The seams before and after levelling (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SeamStats {
    /// Jump (sRGB levels).
    pub seam_jump_before: f64,
    pub seam_jump_after: f64,
    /// Step on the edges between images (sRGB levels): mean and median over the samples. The
    /// mean is dominated by content at the seams (an object's edge, a crease).
    pub seam_step_before: f64,
    pub seam_step_after: f64,
    pub seam_step_median_before: f64,
    pub seam_step_median_after: f64,
    /// The step after levelling as a mean per metre of seam: every sample weighted by the length
    /// of its edge. The plain mean counts the same samples per edge whatever its length and so
    /// favours fine meshes.
    pub seam_step_per_m: f64,
    /// Border vertex corrections (vertex and chart) …
    pub corrections: usize,
    /// … of them clamped to [`MAX_GAIN`] in a channel …
    pub clamped: usize,
    /// … and pairs of charts at a vertex left out as content ([`MAX_SEAM_RATIO`]).
    pub content_pairs: usize,
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
    level(mesh, labels, tex, |tex, chart_of, edges, lin| {
        border_corrections(mesh, tex, chart_of, edges, lin)
    })
}

/// One edge between a face of a Stelle and a face outside it that keeps its texture
/// ([`level_region`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrozenEdge {
    /// The face of the Stelle (of the mesh levelled).
    pub inside: u32,
    /// The edge's vertices (of the mesh levelled).
    pub a: u32,
    pub b: u32,
    /// The texture of the face outside on the edge: its mean linear colour at [`SEAM_SAMPLES`]
    /// of the way from `a` towards `b`, and from `b` towards `a`.
    pub outside: [[f64; 3]; 2],
}

/// Levels a Stelle baked onto pages of its own (`tex`, its charts over the faces of `mesh`)
/// against the texture around it, which stays as it is: as [`level_seams`], but at a vertex where
/// a chart meets the outside along a [`FrozenEdge`] (not across a crease: the caller leaves
/// those out) only the Stelle moves, by the whole difference. Such a chart's log gain there is
/// minus its log ratio to the outside (the outside's colours at [`SEAM_SAMPLES`] of its edges at
/// the vertex, its own at the same spots), clamped to `ln` [`MAX_SEAM_RATIO`]; a chart that
/// meets such a chart at the vertex (and not the outside) takes the gain that matches it to that
/// chart's corrected colour, and so on. Sides differing by more than [`MAX_SEAM_RATIO`] are
/// content and left out, as between charts. Vertices without an edge to the outside are levelled
/// between the charts as [`level_seams`] does (each side halfway); then the interior and the
/// texels as there. With no frozen edges it is [`level_seams`]. Same input, byte-identical
/// pages.
///
/// # Panics
/// If `labels` or `tex` do not fit the mesh, or a frozen edge is not an edge of its face.
pub(crate) fn level_region(
    mesh: &Mesh,
    labels: &[Option<u32>],
    tex: &mut TexturedMesh,
    frozen: &[FrozenEdge],
) -> SeamStats {
    for e in frozen {
        let corners = mesh.indices[e.inside as usize];
        assert!(
            e.a != e.b && corners.contains(&e.a) && corners.contains(&e.b),
            "{e:?} is no edge of its face {corners:?}"
        );
    }
    level(mesh, labels, tex, |tex, chart_of, edges, lin| {
        region_corrections(mesh, tex, chart_of, edges, frozen, lin)
    })
}

/// A border vertex's correction: (chart, mesh vertex, log gain per channel).
type Correction = (u32, u32, [f64; 3]);

/// [`level_seams`] and [`level_region`]: the seam measures before and after, the corrections of
/// the border vertices from `corrections` (given the chart of every face, the edges between
/// textured faces and the sRGB table; returns the corrections, by vertex, and the chart pairs
/// left out as content), the interior and the texels.
fn level(
    mesh: &Mesh,
    labels: &[Option<u32>],
    tex: &mut TexturedMesh,
    corrections: impl FnOnce(
        &TexturedMesh,
        &[u32],
        &[EdgePair],
        &[f32; 256],
    ) -> (Vec<Correction>, usize),
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
    let (seam_step_before, seam_step_median_before, _) = seam_step(mesh, tex, labels, &edges, &lin);

    // 1. Every chart's vertices; the corrections of the border vertices from paired samples
    //    on the seams.
    let verts: Vec<ChartVertices> = tex
        .charts
        .par_iter()
        .map(|chart| chart_vertices(mesh, tex, &chart.faces))
        .collect();
    let mut fixed: Vec<Vec<(u32, [f64; 3])>> = vec![Vec::new(); tex.charts.len()];
    let (border, content_pairs) = corrections(tex, &chart_of, &edges, &lin);
    let border_count = border.len();
    let clamped = border
        .iter()
        .filter(|b| b.2.iter().any(|d| d.abs() >= MAX_GAIN.ln()))
        .count();
    for (c, v, d) in border {
        fixed[c as usize].push((local(&verts[c as usize], v) as u32, d));
    }

    // 2. Corrections per chart vertex; `None` for a chart without any.
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

    // 3. Texels, page by page (pages are independent; charts in order within a page).
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
    let (seam_step_after, seam_step_median_after, seam_step_per_m) =
        seam_step(mesh, tex, labels, &edges, &lin);
    SeamStats {
        seam_jump_before,
        seam_jump_after,
        seam_step_before,
        seam_step_after,
        seam_step_median_before,
        seam_step_median_after,
        seam_step_per_m,
        corrections: border_count,
        clamped,
        content_pairs,
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

/// Per seam edge (between charts, not across a crease) and end vertex: the vertex, the lower and
/// the higher chart, and the mean colour of each at [`SEAM_SAMPLES`] from the vertex; in edge
/// order.
type Paired = (u32, u32, u32, [f64; 3], [f64; 3]);

/// The paired samples of the seams (see [`Paired`]).
fn seam_pairs(
    mesh: &Mesh,
    tex: &TexturedMesh,
    chart_of: &[u32],
    edges: &[EdgePair],
    lin: &[f32; 256],
) -> Vec<Paired> {
    edges
        .par_iter()
        .filter(|e| chart_of[e[0] as usize] != chart_of[e[1] as usize])
        .filter(|e| {
            let (a, b) = (
                mesh.face_normal(e[0] as usize),
                mesh.face_normal(e[1] as usize),
            );
            a[0] * b[0] + a[1] * b[1] + a[2] * b[2] >= SEAM_MIN_COS
        })
        .flat_map_iter(|&[f, g, a, b]| {
            let (f, g) = if chart_of[f as usize] < chart_of[g as usize] {
                (f, g)
            } else {
                (g, f)
            };
            [(a, b), (b, a)].map(|(v, w)| {
                (
                    v,
                    chart_of[f as usize],
                    chart_of[g as usize],
                    seam_mean(mesh, tex, f, v, w, lin),
                    seam_mean(mesh, tex, g, v, w, lin),
                )
            })
        })
        .collect()
}

/// The mean linear colour of face `face`'s texture at [`SEAM_SAMPLES`] of the way from its
/// corner `v` towards its corner `w`.
fn seam_mean(
    mesh: &Mesh,
    tex: &TexturedMesh,
    face: u32,
    v: u32,
    w: u32,
    lin: &[f32; 256],
) -> [f64; 3] {
    let mut sum = [0.0f64; 3];
    for &t in &SEAM_SAMPLES {
        let c = edge_colour(mesh, tex, face, v, w, t, lin);
        for k in 0..3 {
            sum[k] += f64::from(c[k]);
        }
    }
    sum.map(|x| x / SEAM_SAMPLES.len() as f64)
}

/// Whether two mean colours of a seam differ by more than an exposure can ([`MAX_SEAM_RATIO`];
/// a side darker than [`MIN_SEAM_LUMINANCE`] counts as that dark): content, not a seam.
fn content(a: [f64; 3], b: [f64; 3]) -> bool {
    let (la, lb) = (
        luminance(a.map(|x| x as f32)),
        luminance(b.map(|x| x as f32)),
    );
    la.max(lb) > MAX_SEAM_RATIO * la.min(lb).max(MIN_SEAM_LUMINANCE)
}

/// `ln((x + GAIN_FLOOR) / (y + GAIN_FLOOR))` per channel.
fn log_ratio(x: [f64; 3], y: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|k| ((x[k] + GAIN_FLOOR) / (y[k] + GAIN_FLOOR)).ln())
}

/// The chart pairs of the vertex `paired[order[*i]].0` (`order`: `paired` by vertex, then
/// charts): per pair of charts the mean of their colours over its seam edges there, into
/// `at_vertex` (cleared first); pairs that are [`content`] are counted, not kept. Moves `*i` past
/// the vertex.
fn pairs_at(
    paired: &[Paired],
    order: &[usize],
    i: &mut usize,
    at_vertex: &mut Vec<(u32, u32, [f64; 3], [f64; 3])>,
    content_pairs: &mut usize,
) {
    let v = paired[order[*i]].0;
    at_vertex.clear();
    while *i < order.len() && paired[order[*i]].0 == v {
        let (_, ca, cb, _, _) = paired[order[*i]];
        let (mut sa, mut sb, mut n) = ([0.0f64; 3], [0.0f64; 3], 0.0);
        while *i < order.len() && {
            let p = &paired[order[*i]];
            p.0 == v && p.1 == ca && p.2 == cb
        } {
            let p = &paired[order[*i]];
            for k in 0..3 {
                sa[k] += p.3[k];
                sb[k] += p.4[k];
            }
            n += 1.0;
            *i += 1;
        }
        let (sa, sb) = (sa.map(|x| x / n), sb.map(|x| x / n));
        // Sides that differ by more than an exposure can are content (a highlight, an
        // occlusion) rather than a seam.
        if content(sa, sb) {
            *content_pairs += 1;
            continue;
        }
        at_vertex.push((ca, cb, sa, sb));
    }
}

/// The corrections at vertex `v` from its chart pairs `at_vertex`, both sides moving: each chart
/// minus the sum of its log ratios to the others, over the number of charts there, clamped to
/// [`MAX_GAIN`] either way; by chart, appended to `out` (`sums`: room to work in).
fn halfway(
    at_vertex: &[(u32, u32, [f64; 3], [f64; 3])],
    v: u32,
    sums: &mut Vec<(u32, [f64; 3])>,
    out: &mut Vec<Correction>,
) {
    sums.clear();
    for &(ca, cb, sa, sb) in at_vertex {
        for (c, d) in [(ca, log_ratio(sa, sb)), (cb, log_ratio(sb, sa))] {
            match sums.iter_mut().find(|s| s.0 == c) {
                Some(s) => {
                    for (sum, x) in s.1.iter_mut().zip(d) {
                        *sum += x;
                    }
                }
                None => sums.push((c, d)),
            }
        }
    }
    let charts = sums.len() as f64;
    sums.sort_by_key(|s| s.0);
    for &(c, d) in sums.iter() {
        let lim = MAX_GAIN.ln();
        let delta = d.map(|x| (-x / charts).clamp(-lim, lim));
        out.push((c, v, delta));
    }
}

/// The log gain of every border vertex of every chart that borders another one there:
/// (chart, mesh vertex, log gain per channel), from paired samples on the seams (see the module
/// docs); and how many chart pairs at a vertex were left out as content.
fn border_corrections(
    mesh: &Mesh,
    tex: &TexturedMesh,
    chart_of: &[u32],
    edges: &[EdgePair],
    lin: &[f32; 256],
) -> (Vec<Correction>, usize) {
    let paired = seam_pairs(mesh, tex, chart_of, edges, lin);
    let mut order: Vec<usize> = (0..paired.len()).collect();
    order.par_sort_by_key(|&i| (paired[i].0, paired[i].1, paired[i].2));
    let mut out = Vec::new();
    let mut content_pairs = 0usize;
    // Per vertex: the chart pairs there (charts, mean colours), then per chart the sum of its
    // differences to the others.
    let mut at_vertex: Vec<(u32, u32, [f64; 3], [f64; 3])> = Vec::new();
    let mut sums: Vec<(u32, [f64; 3])> = Vec::new();
    let mut i = 0;
    while i < order.len() {
        let v = paired[order[i]].0;
        pairs_at(&paired, &order, &mut i, &mut at_vertex, &mut content_pairs);
        halfway(&at_vertex, v, &mut sums, &mut out);
    }
    (out, content_pairs)
}

/// [`border_corrections`] for [`level_region`]: at vertices on a [`FrozenEdge`] the charts take
/// the whole difference to the outside (see there), elsewhere as between charts.
fn region_corrections(
    mesh: &Mesh,
    tex: &TexturedMesh,
    chart_of: &[u32],
    edges: &[EdgePair],
    frozen: &[FrozenEdge],
    lin: &[f32; 256],
) -> (Vec<Correction>, usize) {
    let paired = seam_pairs(mesh, tex, chart_of, edges, lin);
    let mut order: Vec<usize> = (0..paired.len()).collect();
    order.par_sort_by_key(|&i| (paired[i].0, paired[i].1, paired[i].2));
    // Per frozen edge of a face in a chart and end vertex: the vertex, the chart, its own mean
    // colour and the outside's at SEAM_SAMPLES from the vertex; by vertex and chart, then edge.
    let held: Vec<(u32, u32, [f64; 3], [f64; 3])> = frozen
        .par_iter()
        .filter(|e| chart_of[e.inside as usize] != NONE)
        .flat_map_iter(|e| {
            let c = chart_of[e.inside as usize];
            [(e.a, e.b, e.outside[0]), (e.b, e.a, e.outside[1])]
                .map(|(v, w, out)| (v, c, seam_mean(mesh, tex, e.inside, v, w, lin), out))
        })
        .collect();
    let mut held_order: Vec<usize> = (0..held.len()).collect();
    held_order.par_sort_by_key(|&i| (held[i].0, held[i].1));

    let lim = MAX_SEAM_RATIO.ln();
    let mut out = Vec::new();
    let mut content_pairs = 0usize;
    let mut at_vertex: Vec<(u32, u32, [f64; 3], [f64; 3])> = Vec::new();
    let mut sums: Vec<(u32, [f64; 3])> = Vec::new();
    // Per chart at the vertex with a frozen side: its mean colour and the outside's there.
    let mut outside: Vec<(u32, [f64; 3], [f64; 3])> = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < order.len() || j < held_order.len() {
        let v = match (order.get(i), held_order.get(j)) {
            (Some(&p), Some(&h)) => paired[p].0.min(held[h].0),
            (Some(&p), None) => paired[p].0,
            (None, Some(&h)) => held[h].0,
            (None, None) => unreachable!("one list is not done"),
        };
        if i < order.len() && paired[order[i]].0 == v {
            pairs_at(&paired, &order, &mut i, &mut at_vertex, &mut content_pairs);
        } else {
            at_vertex.clear();
        }
        outside.clear();
        while j < held_order.len() && held[held_order[j]].0 == v {
            let c = held[held_order[j]].1;
            let (mut own, mut out_side, mut n) = ([0.0f64; 3], [0.0f64; 3], 0.0);
            while j < held_order.len() && {
                let h = &held[held_order[j]];
                h.0 == v && h.1 == c
            } {
                let h = &held[held_order[j]];
                for k in 0..3 {
                    own[k] += h.2[k];
                    out_side[k] += h.3[k];
                }
                n += 1.0;
                j += 1;
            }
            let (own, out_side) = (own.map(|x| x / n), out_side.map(|x| x / n));
            if content(own, out_side) {
                content_pairs += 1;
                continue;
            }
            outside.push((c, own, out_side));
        }
        if outside.is_empty() {
            halfway(&at_vertex, v, &mut sums, &mut out);
            continue;
        }
        // The charts with a frozen side match it; then the charts that share a seam with a
        // matched one match its corrected colour, until no chart at the vertex is left that
        // does.
        let mut matched: Vec<(u32, [f64; 3])> = outside
            .iter()
            .map(|&(c, own, out_side)| (c, log_ratio(own, out_side).map(|x| (-x).clamp(-lim, lim))))
            .collect();
        loop {
            let mut next: Vec<(u32, [f64; 3], f64)> = Vec::new();
            for &(ca, cb, sa, sb) in &at_vertex {
                for (c, own, p, other) in [(ca, sa, cb, sb), (cb, sb, ca, sa)] {
                    if matched.iter().any(|m| m.0 == c) {
                        continue;
                    }
                    let Some(&(_, dp)) = matched.iter().find(|m| m.0 == p) else {
                        continue;
                    };
                    // own · e^d = other · e^dp.
                    let r = log_ratio(other, own);
                    let d = [0, 1, 2].map(|k| dp[k] + r[k]);
                    match next.iter_mut().find(|x| x.0 == c) {
                        Some(x) => {
                            for (sum, v) in x.1.iter_mut().zip(d) {
                                *sum += v;
                            }
                            x.2 += 1.0;
                        }
                        None => next.push((c, d, 1.0)),
                    }
                }
            }
            if next.is_empty() {
                break;
            }
            next.sort_by_key(|x| x.0);
            for (c, sum, n) in next {
                matched.push((c, sum.map(|x| (x / n).clamp(-lim, lim))));
            }
        }
        matched.sort_by_key(|m| m.0);
        for (c, d) in matched {
            out.push((c, v, d));
        }
    }
    (out, content_pairs)
}

/// The step measure over `edges` (see the module docs): sRGB luminance levels. Returns the
/// mean and the median over the samples, and the mean with every sample weighted by the length
/// of its edge (m), i.e. per metre of seam.
fn seam_step(
    mesh: &Mesh,
    tex: &TexturedMesh,
    labels: &[Option<u32>],
    edges: &[EdgePair],
    lin: &[f32; 256],
) -> (f64, f64, f64) {
    let seam = |e: &&EdgePair| labels[e[0] as usize] != labels[e[1] as usize];
    let per_edge = 2 * SEAM_SAMPLES.len();
    // In edge order: `per_edge` samples of every seam edge.
    let mut steps: Vec<f64> = edges
        .par_iter()
        .filter(seam)
        .flat_map_iter(|&[f, g, a, b]| {
            [(a, b), (b, a)].into_iter().flat_map(move |(v, w)| {
                SEAM_SAMPLES.iter().map(move |&t| {
                    let y = |face: u32| {
                        srgb_value(luminance(edge_colour(mesh, tex, face, v, w, t, lin)) as f32)
                    };
                    (y(f) - y(g)).abs()
                })
            })
        })
        .collect();
    if steps.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    let mean = steps.iter().sum::<f64>() / steps.len() as f64;
    let (weighted, length) = edges
        .iter()
        .filter(seam)
        .zip(steps.chunks_exact(per_edge))
        .fold((0.0, 0.0), |(sum, total), (&[_, _, a, b], s)| {
            let (p, q) = (mesh.positions[a as usize], mesh.positions[b as usize]);
            let len = (0..3)
                .map(|k| (f64::from(p[k]) - f64::from(q[k])).powi(2))
                .sum::<f64>()
                .sqrt();
            (
                sum + len * s.iter().sum::<f64>(),
                total + len * per_edge as f64,
            )
        });
    let per_m = if length > 0.0 {
        weighted / length
    } else {
        mean
    };
    steps.par_sort_unstable_by(f64::total_cmp);
    (mean, steps[steps.len() / 2], per_m)
}

/// Linear colour of face `f`'s texture at the point `t` of the way from its corner `v` to its
/// corner `w` (bilinear).
fn edge_colour(
    mesh: &Mesh,
    tex: &TexturedMesh,
    f: u32,
    v: u32,
    w: u32,
    t: f64,
    lin: &[f32; 256],
) -> [f32; 3] {
    let (p, i) = tex.face_ref[f as usize];
    let page = &tex.pages[p as usize];
    let tri = page.indices[i as usize];
    let corners = mesh.indices[f as usize];
    let uv_of = |m: u32| {
        let k = corners
            .iter()
            .position(|&c| c == m)
            .expect("corner of the face");
        page.uvs[tri[k] as usize]
    };
    let (a, b) = (uv_of(v), uv_of(w));
    let px = [0, 1].map(|k| {
        let size = f64::from(if k == 0 {
            page.image.width
        } else {
            page.image.height
        });
        ((f64::from(a[k]) * (1.0 - t) + f64::from(b[k]) * t) * size - 0.5) as f32
    });
    sample_linear(&page.image, px, lin)
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

/// The corrections of a chart's vertices: `fixed` at border vertices, the harmonic
/// interpolation over the chart's edges elsewhere (see the module docs).
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
        .filter(|&(a, b)| a != b)
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
    let around = |v: usize| pairs[start[v]..start[v + 1]].iter().map(|p| p.1 as usize);
    let mut is_fixed = vec![false; m];
    let mut delta = vec![[0.0f64; 3]; m];
    for &(l, d) in fixed {
        is_fixed[l as usize] = true;
        delta[l as usize] = d;
    }
    let free = |v: usize| !is_fixed[v] && start[v] < start[v + 1];
    let free_count = (0..m).filter(|&v| free(v)).count();
    let cap = SOLVE_ROUNDS_PER_VERTEX * free_count;
    // Per channel: L x = b over the free vertices, L the graph Laplacian restricted to them,
    // b the sum of their fixed neighbours' corrections. Conjugate gradients from x = 0.
    let solved: Vec<Vec<f64>> = (0..3)
        .map(|c| {
            let laplace = |p: &[f64], out: &mut [f64]| {
                for v in 0..m {
                    out[v] = if free(v) {
                        let mut s = (start[v + 1] - start[v]) as f64 * p[v];
                        for u in around(v) {
                            if free(u) {
                                s -= p[u];
                            }
                        }
                        s
                    } else {
                        0.0
                    };
                }
            };
            let mut r: Vec<f64> = (0..m)
                .map(|v| {
                    if free(v) {
                        around(v)
                            .filter(|&u| is_fixed[u])
                            .map(|u| delta[u][c])
                            .sum()
                    } else {
                        0.0
                    }
                })
                .collect();
            let mut x = vec![0.0f64; m];
            let mut p = r.clone();
            let mut ap = vec![0.0f64; m];
            let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
            let mut rs = dot(&r, &r);
            // The change a Jacobi step would make at the worst vertex.
            let worst = |r: &[f64]| {
                (0..m)
                    .filter(|&v| free(v))
                    .map(|v| r[v].abs() / (start[v + 1] - start[v]) as f64)
                    .fold(0.0, f64::max)
            };
            for _ in 0..cap {
                if worst(&r) < SOLVE_TOLERANCE {
                    break;
                }
                laplace(&p, &mut ap);
                let pap = dot(&p, &ap);
                if pap <= 0.0 {
                    break;
                }
                let alpha = rs / pap;
                for v in 0..m {
                    x[v] += alpha * p[v];
                    r[v] -= alpha * ap[v];
                }
                let next = dot(&r, &r);
                let beta = next / rs;
                rs = next;
                for v in 0..m {
                    p[v] = r[v] + beta * p[v];
                }
            }
            x
        })
        .collect();
    for v in (0..m).filter(|&v| free(v)) {
        delta[v] = [0, 1, 2].map(|c| solved[c][v]);
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
                if add == 0.0 {
                    continue;
                }
                let l = f64::from(lin[buf[at * 3 + c] as usize]) * add.exp();
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
    use super::super::atlas::{ChartInfo, TexturePage, GRAY_BLOCK, MARGIN, UNSEEN_GRAY};
    use super::*;
    use crate::color::{linear_table, srgb_value};
    use driftless_io::images::RgbImage;

    /// A wall of `cells[0] × cells[1]` cells (step 0.1 m) split at `iy = cells[0] / 2` into two
    /// charts on one page of `page²`, laid out as `bake` would: shared page vertices per chart,
    /// `texels` per cell edge, a [`MARGIN`] border, the [`GRAY_BLOCK`] at the origin, the charts
    /// to its right. Chart 0 (lower `iy`) is filled with `left`, chart 1 with `right`, the rest
    /// of the page with [`UNSEEN_GRAY`]. With `gray_row`, the faces of the last row of cells
    /// (highest `iz`) are unseen: no chart, triangle at the gray block's centre.
    struct Wall {
        cells: [usize; 2],
        texels: u32,
        page: u32,
        gray_row: bool,
    }

    const SMALL: Wall = Wall {
        cells: [12, 12],
        texels: 4,
        page: 128,
        gray_row: false,
    };

    impl Wall {
        fn vertex(&self, iy: usize, iz: usize) -> u32 {
            (iy * (self.cells[1] + 1) + iz) as u32
        }

        fn build(&self, left: [u8; 3], right: [u8; 3]) -> (Mesh, Vec<Option<u32>>, TexturedMesh) {
            let [ny, nz] = self.cells;
            let mut mesh = Mesh::default();
            for iy in 0..=ny {
                for iz in 0..=nz {
                    mesh.positions.push([0.0, iy as f32 * 0.1, iz as f32 * 0.1]);
                }
            }
            let mut cell_of = Vec::new();
            for iy in 0..ny {
                for iz in 0..nz {
                    let id = |a, b| self.vertex(a, b);
                    mesh.indices
                        .push([id(iy, iz), id(iy + 1, iz), id(iy, iz + 1)]);
                    mesh.indices
                        .push([id(iy + 1, iz), id(iy + 1, iz + 1), id(iy, iz + 1)]);
                    cell_of.extend([(iy, iz); 2]);
                }
            }
            let n = mesh.indices.len();
            let half = ny / 2;
            let labels: Vec<Option<u32>> = cell_of
                .iter()
                .map(|&(iy, iz)| {
                    (!(self.gray_row && iz == nz - 1)).then_some(u32::from(iy >= half))
                })
                .collect();
            let p = self.page;
            let mut page = TexturePage {
                positions: Vec::new(),
                uvs: Vec::new(),
                indices: Vec::new(),
                image: RgbImage {
                    width: p,
                    height: p,
                    data: vec![UNSEEN_GRAY; (p * p * 3) as usize],
                },
            };
            let mut tex = TexturedMesh {
                pages: Vec::new(),
                face_ref: vec![(0, 0); n],
                textured: labels.iter().map(Option::is_some).collect(),
                charts: Vec::new(),
            };
            let centre = (GRAY_BLOCK / 2) as f32 / p as f32;
            for f in (0..n).filter(|&f| labels[f].is_none()) {
                let base = page.positions.len() as u32;
                for &v in &mesh.indices[f] {
                    page.positions.push(mesh.positions[v as usize]);
                    page.uvs.push([centre, centre]);
                }
                tex.face_ref[f] = (0, page.indices.len() as u32);
                page.indices.push([base, base + 1, base + 2]);
            }
            let w = half as u32 * self.texels + 1 + 2 * MARGIN;
            let h = nz as u32 * self.texels + 1 + 2 * MARGIN;
            for (c, colour) in [left, right].into_iter().enumerate() {
                let origin = [GRAY_BLOCK + c as u32 * (w + 1), 0];
                let mut local = vec![u32::MAX; mesh.positions.len()];
                let faces: Vec<u32> = (0..n as u32)
                    .filter(|&f| labels[f as usize] == Some(c as u32))
                    .collect();
                for &f in &faces {
                    let mut tri = [0u32; 3];
                    for (k, &v) in mesh.indices[f as usize].iter().enumerate() {
                        if local[v as usize] == u32::MAX {
                            local[v as usize] = page.positions.len() as u32;
                            let (iy, iz) = (v as usize / (nz + 1), v as usize % (nz + 1));
                            let t = |o: u32, cell: usize| {
                                (o as f32
                                    + MARGIN as f32
                                    + 0.5
                                    + (cell as u32 * self.texels) as f32)
                                    / p as f32
                            };
                            page.positions.push(mesh.positions[v as usize]);
                            page.uvs
                                .push([t(origin[0], iy - c * half), t(origin[1], iz)]);
                        }
                        tri[k] = local[v as usize];
                    }
                    tex.face_ref[f as usize] = (0, page.indices.len() as u32);
                    page.indices.push(tri);
                }
                for y in origin[1]..origin[1] + h {
                    for x in origin[0]..origin[0] + w {
                        let o = ((y * p + x) * 3) as usize;
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

        /// Texel coordinates of vertex `(iy, iz)` in chart `c`.
        fn texel_of(&self, tex: &TexturedMesh, c: usize, iy: usize, iz: usize) -> [u32; 2] {
            let [x, y, _, _] = tex.charts[c].rect;
            let iy = iy - c * self.cells[0] / 2;
            [
                x + MARGIN + iy as u32 * self.texels,
                y + MARGIN + iz as u32 * self.texels,
            ]
        }
    }

    fn lin(v: u8) -> f64 {
        f64::from(linear_table()[v as usize])
    }

    /// `rgb` (sRGB) brightened by `factor` in linear light.
    fn brighter(rgb: [u8; 3], factor: f64) -> [u8; 3] {
        rgb.map(|v| srgb_value((lin(v) * factor) as f32).round() as u8)
    }

    /// sRGB luminance of `rgb`.
    fn lum(rgb: [u8; 3]) -> f64 {
        luminance(rgb.map(|v| lin(v) as f32))
    }

    fn texel(tex: &TexturedMesh, x: u32, y: u32) -> [u8; 3] {
        let w = tex.pages[0].image.width;
        let o = ((y * w + x) * 3) as usize;
        let d = &tex.pages[0].image.data;
        [d[o], d[o + 1], d[o + 2]]
    }

    fn set_texel(tex: &mut TexturedMesh, x: u32, y: u32, rgb: [u8; 3]) {
        let w = tex.pages[0].image.width;
        let o = ((y * w + x) * 3) as usize;
        tex.pages[0].image.data[o..o + 3].copy_from_slice(&rgb);
    }

    /// The texels of chart `c`'s rectangle.
    fn rect_texels(tex: &TexturedMesh, c: usize) -> impl Iterator<Item = [u32; 2]> {
        let [x, y, w, h] = tex.charts[c].rect;
        (y..y + h).flat_map(move |j| (x..x + w).map(move |i| [i, j]))
    }

    #[test]
    fn seam_jump_drops() {
        let left = [120, 110, 100];
        let right = brighter(left, 1.2);
        let (mesh, labels, mut tex) = SMALL.build(left, right);
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
        // Both sides meet in the middle, and the border texels of chart 1 next to the seam
        // (refilled, owned by no triangle) carry the corrected colour, not the old one.
        let [x1, _, _, h] = tex.charts[1].rect;
        let mid = h / 2;
        let (yl, yr) = (lum(left), lum(right));
        for x in [x1, x1 + MARGIN, x1 + MARGIN + 1] {
            let got = lum(texel(&tex, x, mid));
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
        let (_, _, mut again) = SMALL.build(left, right);
        assert_eq!(level_seams(&mesh, &labels, &mut again), stats);
        assert_eq!(again, tex);
    }

    #[test]
    fn uniform_untouched() {
        let colour = [90, 140, 200];
        // Two charts of the same colour meet without a seam.
        let (mesh, labels, mut tex) = SMALL.build(colour, colour);
        let before = tex.clone();
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert_eq!(tex, before);
        assert!(stats.seam_jump_before.abs() < 1e-9, "{stats:?}");
        assert!(stats.seam_jump_after.abs() < 1e-9, "{stats:?}");
    }

    #[test]
    fn large_chart_converges() {
        // 100 cells (edges) from the seam to each chart's far side.
        let wall = Wall {
            cells: [200, 4],
            texels: 2,
            page: 512,
            gray_row: false,
        };
        let left = [120, 110, 100];
        let right = brighter(left, 1.2);
        let (mesh, labels, mut tex) = wall.build(left, right);
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(
            stats.seam_jump_after < 0.3 * stats.seam_jump_before,
            "{stats:?}"
        );
        // A chart bordering the other on one side only is shifted as a whole: half the
        // difference, at the far side too.
        let expected: [u8; 3] = std::array::from_fn(|k| {
            srgb_value(((lin(left[k]) + lin(right[k])) / 2.0) as f32).round() as u8
        });
        for (c, iy) in [(0, 0), (0, 50), (1, 150), (1, 200)] {
            let [x, y] = wall.texel_of(&tex, c, iy, 2);
            let got = texel(&tex, x, y);
            for k in 0..3 {
                assert!(
                    got[k].abs_diff(expected[k]) <= 1,
                    "Karte {c}, Zelle {iy}: {got:?} statt {expected:?}"
                );
            }
        }
    }

    #[test]
    fn outlier_corner_is_bounded() {
        let colour = [100, 100, 100];
        let seam = (SMALL.cells[0] / 2, SMALL.cells[1] / 2);
        // The small speck covers the seam samples next to the vertex, the large one is white:
        // both differ from the other side by more than an exposure can.
        for (radius, speck, label) in [(1u32, 200u8, "kleiner Fleck"), (4, 255, "großer Fleck")] {
            let (mesh, labels, mut tex) = SMALL.build(colour, colour);
            // A bright speck in chart 1 around a seam vertex.
            let [cx, cy] = SMALL.texel_of(&tex, 1, seam.0, seam.1);
            for y in cy - radius..=cy + radius {
                for x in cx.saturating_sub(radius).max(tex.charts[1].rect[0])..=cx + radius {
                    set_texel(&mut tex, x, y, [speck; 3]);
                }
            }
            let before = tex.clone();
            level_seams(&mesh, &labels, &mut tex);
            for [x, y] in rect_texels(&tex, 0) {
                let (old, new) = (texel(&before, x, y), texel(&tex, x, y));
                // No gradient spreads.
                assert!(
                    (0..3).all(|k| new[k].abs_diff(old[k]) <= 1),
                    "{label}: Texel ({x}, {y}) {old:?} -> {new:?}"
                );
            }
            assert_eq!(tex, before, "{label}");
        }
    }

    #[test]
    fn dark_next_to_bright_clamps() {
        // A white island in the darker chart and a black one in the brighter chart, away from
        // the seam: the correction must clamp them at 255 and 0, not wrap.
        let (dark, bright) = ([100; 3], [130; 3]);
        let (mesh, labels, mut tex) = SMALL.build(dark, bright);
        let [wx, wy] = SMALL.texel_of(&tex, 0, 2, 6);
        let [bx, by] = SMALL.texel_of(&tex, 1, 10, 6);
        for d in 0..3 {
            set_texel(&mut tex, wx + d, wy, [255; 3]);
            set_texel(&mut tex, bx + d, by, [0; 3]);
        }
        let before = tex.clone();
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(
            stats.seam_jump_after < 0.3 * stats.seam_jump_before,
            "{stats:?}"
        );
        for d in 0..3 {
            assert_eq!(texel(&tex, wx + d, wy), [255; 3]);
            assert_eq!(texel(&tex, bx + d, by), [0; 3]);
        }
        for c in 0..2 {
            for [x, y] in rect_texels(&tex, c) {
                let (old, new) = (texel(&before, x, y), texel(&tex, x, y));
                for k in 0..3 {
                    let change = lin(new[k]) - lin(old[k]);
                    // The dark side only brightens, the bright side only darkens.
                    let ok = if c == 0 { change >= 0.0 } else { change <= 0.0 };
                    assert!(ok, "Karte {c}, Texel ({x}, {y}): {old:?} -> {new:?}");
                }
            }
        }

        // Nearly twice as bright (1.94 in linear light): both sides meet halfway, each scaled
        // by at most MAX_GAIN.
        let (dark, bright) = ([190; 3], [255; 3]);
        let (mesh, labels, mut tex) = SMALL.build(dark, bright);
        let before = tex.clone();
        level_seams(&mesh, &labels, &mut tex);
        for c in 0..2 {
            for [x, y] in rect_texels(&tex, c) {
                let gain = lin(texel(&tex, x, y)[0]) / lin(texel(&before, x, y)[0]);
                assert!(
                    (1.0 / MAX_GAIN - 0.01..=MAX_GAIN + 0.01).contains(&gain),
                    "Karte {c}, Texel ({x}, {y}): {gain}"
                );
            }
        }
        let [lx, ly] = SMALL.texel_of(&tex, 0, 6, 6);
        let [rx, ry] = SMALL.texel_of(&tex, 1, 6, 6);
        let (l, r) = (texel(&tex, lx, ly)[0], texel(&tex, rx, ry)[0]);
        assert!(l.abs_diff(r) <= 2, "{l} / {r}");

        // Sides differing by more than MAX_SEAM_RATIO are content: nothing changes.
        let (mesh, labels, mut tex) = SMALL.build([40; 3], [200; 3]);
        let before = tex.clone();
        level_seams(&mesh, &labels, &mut tex);
        assert_eq!(tex, before);
    }

    #[test]
    fn exposure_step_on_texture_levels() {
        // Stripes across the seam (dark and light, 3 texels each), the right chart 30 % brighter
        // in linear light: an exposure step on textured content.
        let (dark, light) = ([40, 45, 50], [150, 140, 120]);
        let (mesh, labels, mut tex) = SMALL.build(dark, dark);
        for c in 0..2 {
            for [x, y] in rect_texels(&tex, c) {
                let base = if (y / 3) % 2 == 0 { dark } else { light };
                let rgb = if c == 1 { brighter(base, 1.3) } else { base };
                set_texel(&mut tex, x, y, rgb);
            }
        }
        // Both sides of the seam texel by texel along it (the same spot of the wall).
        let half = SMALL.cells[0] / 2;
        let step = |tex: &TexturedMesh| {
            let [_, y0] = SMALL.texel_of(tex, 0, half, 0);
            let [x0, _] = SMALL.texel_of(tex, 0, half, 0);
            let [x1, _] = SMALL.texel_of(tex, 1, half, 0);
            let rows = SMALL.cells[1] as u32 * SMALL.texels;
            (y0..y0 + rows)
                .map(|y| {
                    let (a, b) = (texel(tex, x0, y), texel(tex, x1, y));
                    (0..3).map(|k| a[k].abs_diff(b[k])).max().unwrap()
                })
                .max()
                .unwrap()
        };
        let before = step(&tex);
        let stats = level_seams(&mesh, &labels, &mut tex);
        let after = step(&tex);
        println!("largest step along the seam {before} → {after} sRGB levels; {stats:?}");
        assert!(before > 15, "{before}");
        assert!(after <= 3, "{before} → {after}");
    }

    #[test]
    fn crease_is_not_levelled() {
        // The wall folded by 90° along the seam: the two sides are different surfaces, their
        // brightness differs by the light, not by the exposure.
        let (mut mesh, labels, mut tex) = SMALL.build([100; 3], [130; 3]);
        let half = SMALL.cells[0] / 2;
        for iy in half + 1..=SMALL.cells[0] {
            for iz in 0..=SMALL.cells[1] {
                let p = &mut mesh.positions[SMALL.vertex(iy, iz) as usize];
                p[0] = (iy - half) as f32 * 0.1;
                p[1] = half as f32 * 0.1;
            }
        }
        let before = tex.clone();
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert_eq!(tex, before, "{stats:?}");
        assert_eq!(stats.corrections, 0, "{stats:?}");
    }

    #[test]
    fn large_exposure_step_is_levelled() {
        // 2.5 times brighter in linear light: beyond the old limit of 2, within MAX_SEAM_RATIO.
        let left = [60, 55, 50];
        let (mesh, labels, mut tex) = SMALL.build(left, brighter(left, 2.5));
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(
            stats.seam_step_after < 0.2 * stats.seam_step_before,
            "{stats:?}"
        );
    }

    #[test]
    fn step_per_metre_is_the_mean_on_equal_edges() {
        let left = [60, 55, 50];
        let (mesh, labels, mut tex) = SMALL.build(left, brighter(left, 1.5));
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(stats.seam_step_before > 0.0, "{stats:?}");
        // Every seam edge of the wall is equally long.
        assert!(
            (stats.seam_step_per_m - stats.seam_step_after).abs() < 1e-9,
            "{stats:?}"
        );
    }

    #[test]
    fn max_gain_follows_the_ratio() {
        // Two charts just within MAX_SEAM_RATIO meet halfway.
        assert!((MAX_GAIN * MAX_GAIN - MAX_SEAM_RATIO).abs() < 1e-12);
    }

    #[test]
    fn gray_faces_untouched() {
        let wall = Wall {
            gray_row: true,
            ..SMALL
        };
        let left = [120, 110, 100];
        let (mesh, labels, mut tex) = wall.build(left, brighter(left, 1.2));
        let before = tex.clone();
        let stats = level_seams(&mesh, &labels, &mut tex);
        assert!(
            stats.seam_jump_after < 0.3 * stats.seam_jump_before,
            "{stats:?}"
        );
        for y in 0..GRAY_BLOCK {
            for x in 0..GRAY_BLOCK {
                assert_eq!(texel(&tex, x, y), [UNSEEN_GRAY; 3]);
            }
        }
        let page = &tex.pages[0];
        assert_eq!(page.positions, before.pages[0].positions);
        assert_eq!(page.uvs, before.pages[0].uvs);
        assert_eq!(page.indices, before.pages[0].indices);
        assert_eq!(tex.face_ref, before.face_ref);
        assert!(tex.textured.iter().any(|t| !t));
    }
    #[test]
    fn region_without_frozen_edges_is_level_seams() {
        let left = [120, 110, 100];
        for right in [brighter(left, 1.2), brighter(left, 2.5), left] {
            let (mesh, labels, mut a) = SMALL.build(left, right);
            let mut b = a.clone();
            let sa = level_seams(&mesh, &labels, &mut a);
            let sb = level_region(&mesh, &labels, &mut b, &[]);
            assert_eq!(a, b);
            assert_eq!(sa, sb);
        }
    }

    /// The edges between chart `outside` and the other chart of the wall as frozen edges of the
    /// other chart's faces, with the colours of `outside` there.
    fn frozen_from(mesh: &Mesh, tex: &TexturedMesh, outside: u32) -> Vec<FrozenEdge> {
        let lin = linear_table();
        let mut chart_of = vec![NONE; mesh.indices.len()];
        for (c, chart) in tex.charts.iter().enumerate() {
            for &f in &chart.faces {
                chart_of[f as usize] = c as u32;
            }
        }
        edge_pairs(mesh, &chart_of)
            .into_iter()
            .filter(|e| chart_of[e[0] as usize] != chart_of[e[1] as usize])
            .map(|[f, g, a, b]| {
                let (out, inside) = if chart_of[f as usize] == outside {
                    (f, g)
                } else {
                    (g, f)
                };
                FrozenEdge {
                    inside,
                    a,
                    b,
                    outside: [
                        seam_mean(mesh, tex, out, a, b, &lin),
                        seam_mean(mesh, tex, out, b, a, &lin),
                    ],
                }
            })
            .collect()
    }

    #[test]
    fn region_takes_the_whole_difference() {
        // Chart 1 of the wall as a Stelle, chart 0 as the texture around it that stays: the
        // Stelle borders it on one side only and takes the whole difference, as a whole.
        let left = [120, 110, 100];
        for factor in [1.2, 0.8, 2.5] {
            let (mesh, mut labels, mut tex) = SMALL.build(left, brighter(left, factor));
            let frozen = frozen_from(&mesh, &tex, 0);
            // One edge of the seam per row of cells.
            assert_eq!(frozen.len(), SMALL.cells[1]);
            let outside = tex.charts.remove(0);
            for &f in &outside.faces {
                labels[f as usize] = None;
                tex.textured[f as usize] = false;
            }
            let before = tex.clone();
            let stats = level_region(&mesh, &labels, &mut tex, &frozen);
            assert_eq!(stats.corrections, SMALL.cells[1] + 1, "{stats:?}");
            let [x, y, w, h] = outside.rect;
            for j in y..y + h {
                for i in x..x + w {
                    assert_eq!(texel(&tex, i, j), texel(&before, i, j), "({i}, {j})");
                }
            }
            for [i, j] in rect_texels(&tex, 0) {
                let got = texel(&tex, i, j);
                for k in 0..3 {
                    assert!(
                        got[k].abs_diff(left[k]) <= 1,
                        "× {factor}: texel ({i}, {j}) {got:?} vs {left:?}"
                    );
                }
            }
        }
        // Sides further apart than an exposure can be are content: nothing changes.
        let (mesh, mut labels, mut tex) = SMALL.build([40; 3], [200; 3]);
        let frozen = frozen_from(&mesh, &tex, 0);
        let outside = tex.charts.remove(0);
        for &f in &outside.faces {
            labels[f as usize] = None;
            tex.textured[f as usize] = false;
        }
        let before = tex.clone();
        let stats = level_region(&mesh, &labels, &mut tex, &frozen);
        assert_eq!(tex, before);
        assert_eq!(stats.content_pairs, SMALL.cells[1] + 1, "{stats:?}");
    }

    #[test]
    fn region_match_reaches_the_chart_beside() {
        // Both charts of the wall (one colour) are the Stelle; the texture that stays lies below
        // chart 0 only, 20 % brighter. At the corner where the seam meets it, chart 1 has no
        // edge to it: it matches chart 0's corrected colour there, so the seam stays closed.
        let colour = [120, 110, 100];
        let (mesh, labels, mut tex) = SMALL.build(colour, colour);
        let half = SMALL.cells[0] / 2;
        let below: [f64; 3] = brighter(colour, 1.2).map(lin);
        let frozen: Vec<FrozenEdge> = (0..half)
            .map(|iy| FrozenEdge {
                // The lower triangle of cell (iy, 0): its first edge is the wall's bottom.
                inside: (2 * iy * SMALL.cells[1]) as u32,
                a: SMALL.vertex(iy, 0),
                b: SMALL.vertex(iy + 1, 0),
                outside: [below; 2],
            })
            .collect();
        level_region(&mesh, &labels, &mut tex, &frozen);
        let at = |c: usize, iy: usize, iz: usize| {
            let [x, y] = SMALL.texel_of(&tex, c, iy, iz);
            texel(&tex, x, y)
        };
        let want = brighter(colour, 1.2);
        for (c, iy) in [(0, 0), (0, half / 2), (0, half), (1, half)] {
            let got = at(c, iy, 0);
            for k in 0..3 {
                assert!(
                    got[k].abs_diff(want[k]) <= 1,
                    "chart {c}, ({iy}, 0): {got:?} vs {want:?}"
                );
            }
        }
        // Away from the bottom the seam has no step either.
        for iz in 1..=SMALL.cells[1] {
            let (a, b) = (at(0, half, iz), at(1, half, iz));
            assert!(
                (0..3).all(|k| a[k].abs_diff(b[k]) <= 1),
                "({half}, {iz}): {a:?} vs {b:?}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "is no edge of its face")]
    fn frozen_edge_must_be_an_edge_of_its_face() {
        let (mesh, labels, mut tex) = SMALL.build([100; 3], [100; 3]);
        let edge = FrozenEdge {
            inside: 0,
            a: SMALL.vertex(5, 5),
            b: SMALL.vertex(5, 6),
            outside: [[0.5; 3]; 2],
        };
        level_region(&mesh, &labels, &mut tex, &[edge]);
    }
}
