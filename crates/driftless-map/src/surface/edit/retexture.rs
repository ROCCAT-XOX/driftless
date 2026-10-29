//! „Bilder neu einspielen“: a Stelle baked anew from the camera images, onto pages of its own.
//!
//! The surface step looks through every image of every scan for every face; for one Stelle
//! that would read and decode all images again (Büro: 3,831 images, about three times). Here
//! no image is decoded to find the candidates, only the best few get a depth buffer, and only
//! the best few are read and decoded at all.
//!
//! **Which images show the Stelle** ([`region_views`]):
//!
//! 1. **Candidates**: the image headers of every source, from the index of its images
//!    ([`index_images`]: made once, no image read); an image counts if its capture time lies
//!    on the trajectory, the device turned no faster than [`ColorConfig::max_angular_rate`]
//!    then, and from its camera (`trajectory(stamp + time_offset) · cam_in_body`) the Stelle
//!    lies within [`ColorConfig::max_dist`], within [`ColorConfig::rim_theta`] of the optical
//!    axis and at least one of its faces faces the camera. The quality pass of the surface
//!    step (sharpness against the scan's median) needs the decoded images and is left out: the
//!    sharpness on the Stelle itself is measured on the decoded few (step 6).
//! 2. **Visibility**: a depth buffer as the surface step builds it, but only from the faces
//!    that can reach the cells the Stelle is tested in: those in the cone from the camera
//!    around the Stelle, widened by [`MAX_SPLAT`] + 2 cells ([`CONE_MARGIN`] times over), and
//!    not farther than the Stelle's farthest point. A face beyond both changes no cell a face
//!    of the Stelle is tested in, so a face of the Stelle is seen exactly when the surface step
//!    would see it (with its own normals and depth tolerance). Which faces are in the cone is
//!    decided with `acos`/`asin`, whose last bit can differ between machines; it decides only
//!    about faces far from those cells, so the views do not depend on it. The faces are found
//!    in a [`FaceBvh`] of the surface. The buffer itself holds only those cells: the cells of
//!    the centroids and corners of the faces tested and their neighbours
//!    ([`windowed_views`]); splats outside them change nothing that is read. The score of a
//!    face is the surface step's ([`super::super::views`]: pixels at full size, capped at
//!    [`SurfaceConfig::min_texel`], times the cosine to the normal smoothed over two rings)
//!    with `sharpness_norm` 1; a view that may show the carrier counts at
//!    [`CARRIER_ALTERNATIVE`](super::super::views::CARRIER_ALTERNATIVE) of it, as in the slot
//!    order there. So the scores of one face compare across images and scans. The faces of a
//!    ring around the Stelle are looked at the same way (the cone around both), for step 5.
//! 3. **Ranking**: an image covering at least [`FULL_COVERAGE`] of the Stelle's area can
//!    texture it alone; those come first, then the others, each by their quality (the area-
//!    weighted mean over the Stelle of `√(score / best score of the face)`, 0 where unseen),
//!    then in stream order. [`decode_views`] reads (at their places in the bags,
//!    [`read_indexed`]) and decodes the first [`DECODED_VIEWS`] at full size.
//!
//! **Only the best get a depth buffer.** Without a depth buffer every candidate already has, for
//! every face of the Stelle, the score it would give it if nothing were in the way: the same
//! numbers as with the buffer, which can only take a face away. These bounds rank the
//! candidates; the first [`PRERANK_BATCH`] get a depth buffer, then, in rounds of twice as
//! many, every candidate that could still change the best [`DECODED_VIEWS`] views: one that
//! could raise the best score of a face one of them sees (and so their qualities), or whose
//! bounds would put it before the last of them (its coverage could reach [`FULL_COVERAGE`]
//! while that one's does not, or its quality could reach that one's, ties included). When
//! none is left, the best views are the ones a depth buffer for every candidate gives, in the
//! same order, with the same numbers ([`Sight`]); how many candidates that takes depends on
//! how much is in the way.
//!
//! **Baking it anew** ([`retexture_region`]):
//!
//! 4. **The old texture around it**: the faces within [`RING_ROUNDS`] rounds around the Stelle
//!    (faces sharing a vertex) are found in the current surface by the bits of their corners
//!    (the computed pages as the earlier steps left them, then the pages those added); only the
//!    pages that hold them are decoded, [`PAGES_AT_ONCE`] at a time. A face whose corners share
//!    one texture coordinate is unseen (gray): no reference.
//! 5. **Gains**: per decoded image and channel the median ratio of its colour to the old one
//!    over the faces of the ring it sees (at least [`MIN_GAIN_FACES`]; a face's colour: its
//!    centroid and the points halfway to its corners, as the views sample it): the image's
//!    exposure against the surface's. An image without enough such faces takes the median gain
//!    of the other images of its scan, else of all, else 1. The surface's edge falloff is not on
//!    disk, so none is divided out; what remains at the border is levelled (step 8).
//! 6. **Choice**: the sharpness of every decoded image on the Stelle itself
//!    ([`footprint_sharpness`]: at the finest texel size any of them reaches there, but not
//!    finer than [`SurfaceConfig::min_texel`] and `sqrt(area / SHARPNESS_TEXELS)`; colours
//!    divided by the image's gain), relative to the median of them and at most 1, times its
//!    quality orders the images: those covering [`FULL_COVERAGE`] first, then by quality ×
//!    sharpness, then by source and stream order. Variant `k` takes the `k`-th (`k mod n`) for
//!    every face it sees, but where the photo consistency (below) leaves its views out in a
//!    group of at least [`ISLAND_FACES`] faces, and along the edge of such a group
//!    ([`Prepared::taken_from`]). The other faces take the next best by [`ICM_ROUNDS`] rounds
//!    of ICM over the views kept in the other images, the faces of the chosen one and the
//!    outside held (the outside never has the same image), as the surface step labels; then
//!    islands and small gray patches take the image around them where its camera sees them
//!    and may take them ([`absorb_islands_where`]). A face no image takes keeps its old
//!    texture. An image that takes faces from the chosen one is matched to it around them
//!    ([`Judged::match_gain`]): the gains match the images to the old texture only around the
//!    Stelle.
//! 7. **Bake** ([`bake_region`]): the charts as the surface step makes them, at its texel size
//!    ([`SurfaceConfig::min_texel`]), on pages of their own: the smallest power of two from
//!    [`REGION_PAGE_MIN`] to [`REGION_PAGE_MAX`] that holds them, several if one does not; the
//!    colours divided by the gains. The old pages stay as they are: nothing is packed anew.
//! 8. **Level** ([`level_region`]): where a chart meets the old texture (an edge to a textured
//!    face that keeps it, not across a crease of [`SEAM_MIN_COS`]) only the new page adapts, by
//!    the whole difference; the seams between its own charts are levelled both ways, as the
//!    surface step does.
//! 9. **Patch**: the faces with a new texture leave their pages; the new pages (JPEG of
//!    [`SurfaceConfig::jpeg_quality`]) hold them, corner `k` of a triangle the copy of its
//!    face's corner `k`, where the (possibly smoothed) surface has it.
//!
//! **Photo consistency** ([`judge_views`]): a thing the surface does not have (a rod or a pipe
//! well in front of a wall, a person walking by) shows in an image on the faces behind it, and
//! from every place on other faces. As in the surface step, the views of every face in the
//! decoded images are judged against the median of their votes (one per camera and stretch of
//! capture time; a strict majority decides; the same decision, [`consistent`]), and a view that
//! differs is left out for that face. What is compared is not the colour itself but the colour
//! against the image's own colours of the faces around it: per view the face's colour in the
//! image at 1/[`SHARPNESS_DOWNSCALE`], as the surface step looks (the mean at [`FACE_SAMPLES`],
//! divided by the gain), over the median of its colours of the faces of the Stelle within
//! [`CONSISTENCY_RADIUS`]; more than [`CONSISTENCY_RATIO`] off the median of the votes in a
//! channel is something else. Measured on the wall Stellen of 1 m² of the black room and the
//! Büro (24 images each), the tolerances keep apart what is not a thing in front:
//! - the gains match the images to the old texture only around the Stelle, without the edge
//!   falloff: in the black room the views of one face between two lamps differed by up to twice
//!   in brightness (the glow seen from elsewhere), so the colours themselves are not compared;
//! - against its surroundings a face differs between the images by up to three times where the
//!   surface is glossy (the bright edges of the cabinet doors in the Büro): the factor is the
//!   seam levelling's limit between exposure and content ([`MAX_SEAM_RATIO`]);
//! - image noise: in the dark (sRGB 10 to 15) a face alone differs from its surroundings by
//!   twice, so both sides count at least as bright as [`MIN_SEAM_LUMINANCE`];
//! - the images lie a few centimetres apart on the surface (3–4 cm at a dark band next to a pipe
//!   in the black room, up to 8 cm at the handles in the Büro): a view that agrees with the
//!   median at a face within [`CONSISTENCY_SHIFT`] is kept.
//!
//! With a factor of 2 and without the floor and the shift, the views of every image of the Büro
//! differed at 4 to 18 % of its faces, and taking them from the chosen image cut the handles and
//! notched the edges of the doors; in the black room a brighter image replaced the chosen one's
//! band in part. With these tolerances the images of the first six variants differ at no face of
//! the black room's Stelle and at 1 to 11 of the Büro's, and the chosen image keeps every face. A
//! rod 0.5 m in front of a wall, seen from 1.5 m, lies 25 cm and more apart in the images and
//! differs by far more than three times. A thing wider than the surroundings is found only at its
//! edges; one that differs from the wall by less than three times, or that the images show less
//! than [`CONSISTENCY_SHIFT`] apart, is not found.
//!
//! Without an image that sees the Stelle it keeps its texture ([`EditError::NoImages`]); so it
//! does with nothing around it to match the brightness to: no ring face whose texture an image
//! sees, and no edge to a textured face ([`EditError::NoReference`]; a Stelle among gray faces).
//!
//! Everything runs on the CPU and is deterministic: per image an ordered map, sums and sorts
//! in a fixed order with a tie rule, the depth buffer a minimum on the bits of an `f32`; hash
//! maps only to look up.

use super::{dot, sub, EditError, FaceBvh, PatchPage, SurfacePatch, WeldedSurface};
use crate::color::{
    angular_rate, linear_table, luminance, sample_linear, ColorCamera, ColorConfig,
    SHARPNESS_DOWNSCALE,
};
use crate::exposure::Vignetting;
use crate::surface::atlas::{
    absorb_islands_where, bake_region, footprint_sharpness, projects, RegionImage, ICM_ROUNDS,
    ISLAND_FACES,
};
#[cfg(doc)]
use crate::surface::atlas::{REGION_PAGE_MAX, REGION_PAGE_MIN};
use crate::surface::level::{
    level_region, FrozenEdge, GAIN_FLOOR, MAX_SEAM_RATIO, MIN_SEAM_LUMINANCE, SEAM_MIN_COS,
    SEAM_SAMPLES,
};
use crate::surface::views::{
    all_cameras, consistent, stretch_of, vote_median, voters, windowed_views, CornerFaces, Faces,
    Shot, TextureSource, UsedImage, ViewTable, MAX_VIEWS_PER_FACE, SMOOTH_COS_MIN,
};
use crate::surface::{encode_pages, Mesh, PageJpegs, SurfaceConfig, TexturedMesh};
use crate::visibility::MAX_SPLAT;
use crate::MapError;
use driftless_export::mesh::{ImageKind, Mesh as PageMesh};
use driftless_io::camera::Camera;
use driftless_io::images::{
    decode_jpeg, read_indexed, ImageError, ImageHeader, ImageIndex, RgbImage,
};
#[cfg(doc)]
use driftless_io::images::{for_each_image_where, index_images};
use driftless_io::progress::Progress;
use driftless_traj::interpolate_pose;
use nalgebra::{Isometry3, Point3, Vector3};
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::Arc;

/// At most this many images of a Stelle are decoded at full size: the best ones
/// ([`decode_views`]).
pub(crate) const DECODED_VIEWS: usize = 24;

/// An image that sees at least this share of a Stelle's area can texture it alone: such images
/// rank first ([`region_views`]).
pub(crate) const FULL_COVERAGE: f32 = 0.9;

/// Candidates of a Stelle given a depth buffer in the first round, the best by what they
/// could see with nothing in the way ([`region_views`]); each further round twice as many,
/// of those that could still change the best [`DECODED_VIEWS`]. How many it is changes only
/// the time, never the views.
pub(crate) const PRERANK_BATCH: usize = 2 * DECODED_VIEWS;

/// The cone of faces that can hide a Stelle from a camera is widened by this many times
/// the angle of [`MAX_SPLAT`] + 2 depth-buffer cells at the image centre: a splat reaches
/// [`MAX_SPLAT`] cells, the silhouette test one more, and a pixel off the centre of a fisheye
/// spans more angle than one at it.
pub(crate) const CONE_MARGIN: f64 = 3.0;

/// Rounds of faces (sharing a vertex) around a Stelle whose old texture the new images are
/// matched to ([`retexture_region`]).
pub(crate) const RING_ROUNDS: usize = 3;

/// An image's gain against the old texture needs at least this many faces of the ring it sees.
pub(crate) const MIN_GAIN_FACES: usize = 6;

/// The sharpness of the images of a Stelle is compared at a texel size of at least
/// `sqrt(area of the Stelle / SHARPNESS_TEXELS)`, so that one measure stays small.
pub(crate) const SHARPNESS_TEXELS: f64 = 262_144.0;

/// At most this many pages of the surface are decoded at once to read the old texture.
pub(crate) const PAGES_AT_ONCE: usize = 4;

/// The photo consistency of a Stelle ([`judge_views`]) compares how a view shows a face against
/// how it shows the faces within this distance around it (m): the median of those colours.
pub(crate) const CONSISTENCY_RADIUS: f64 = 0.2;

/// A view of a face whose colour, against its own surroundings ([`CONSISTENCY_RADIUS`]), lies
/// more than this factor (linear light, in any channel) from the median of the votes of the
/// face's views shows something else there ([`judge_views`]): the limit between exposure and
/// content of the seam levelling ([`MAX_SEAM_RATIO`]). Both sides count at least as bright as
/// [`MIN_SEAM_LUMINANCE`].
pub(crate) const CONSISTENCY_RATIO: f64 = MAX_SEAM_RATIO;

/// A view that differs from the median of the votes at a face is not left out if it agrees with
/// the median at a face within this distance (m): the images of a Stelle lie a few centimetres
/// apart on it (black room 3–4 cm, Büro up to 8 cm), and their edges would differ everywhere.
pub(crate) const CONSISTENCY_SHIFT: f64 = 0.05;

// Every view of a face is judged in one array of the photo consistency.
const _: () = assert!(DECODED_VIEWS <= MAX_VIEWS_PER_FACE);

/// Where a face is sampled for its colour (barycentric weights of its corners): its centroid
/// and the points halfway from it to the corners.
const FACE_SAMPLES: [[f64; 3]; 4] = [
    [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
    [2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0],
    [1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
    [1.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0],
];

/// Slack of the tests that leave out images and faces (m, rad): far above the rounding of the
/// `f32` positions the surface step looks with, far below anything that matters; they only
/// ever keep more.
const SLACK: f64 = 1e-6;

/// Phase of [`region_views`]: the headers of the images, one step each.
const PHASE_FIND: &str = "Bilder der Stelle suchen";
/// Phase of [`decode_views`]: one step per image decoded.
const PHASE_DECODE: &str = "Bilder der Stelle lesen";
/// Phase of the rest of [`retexture_region`]: old texture, choice, baking, levelling.
const PHASE_BAKE: &str = "Stelle neu backen";

/// An image that shows (part of) a Stelle, as [`region_views`] finds it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RegionView {
    /// Its source: index into the sources given to [`region_views`].
    pub source: u32,
    /// Its place in the source's image stream ([`ImageHeader::index`]).
    pub frame: u32,
    /// Its stamp (ns, [`ImageHeader::stamp_ns`]).
    pub stamp: u64,
    /// Its camera: index into the source's [`TextureSource::cams`].
    pub camera: u32,
    /// Where the camera stood: `trajectory(stamp + time_offset) · cam_in_body`.
    pub world_from_cam: Isometry3<f64>,
    /// The share of the Stelle's area it sees (0 to 1).
    pub coverage: f32,
    /// The area-weighted mean over the Stelle of `√(score / best score of the face)`, 0 where
    /// it does not see the face (0 to 1; 1: the best image of every face).
    pub quality: f32,
    /// The faces of the Stelle it sees, sorted, each with its score (see the module docs;
    /// comparable across images).
    pub faces: Vec<(u32, f32)>,
    /// The faces of the ring around the Stelle it sees (of `around`, see [`region_views`]),
    /// sorted.
    pub ring: Vec<u32>,
}

/// How [`region_views`] finds the images that see a Stelle (see the module docs, step 2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Sight {
    /// The best this many views are found, exactly as with a depth buffer for every candidate;
    /// no more are given. `usize::MAX`: every view.
    pub keep: usize,
    /// Candidates given a depth buffer in the first round; each further round twice as many.
    pub batch: usize,
    /// Each depth buffer only over the cells the Stelle and its ring are tested in, else over
    /// the whole image (the same views either way).
    pub cutout: bool,
}

impl Sight {
    /// What „Bilder neu“ needs: the [`DECODED_VIEWS`] best, in rounds from [`PRERANK_BATCH`]
    /// candidates, each depth buffer over the cut-out.
    pub(crate) const BEST: Sight = Sight {
        keep: DECODED_VIEWS,
        batch: PRERANK_BATCH,
        cutout: true,
    };
    /// Every view: every candidate gets a depth buffer, in one round.
    #[cfg(test)]
    pub(crate) const ALL: Sight = Sight {
        keep: usize::MAX,
        batch: usize::MAX,
        cutout: true,
    };
}

/// What [`region_views`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct SightStats {
    /// The candidates (by headers and poses), …
    pub candidates: usize,
    /// … how many got a depth buffer, in how many rounds, …
    pub buffered: usize,
    pub rounds: usize,
    /// … and the cells of their depth buffers, together.
    pub cells: usize,
}

/// The images of `sources` that see faces of the Stelle `faces` (welded ids, any order) on the
/// surface `s` (its faces in the tree `bvh`), best first (see the module docs): those covering
/// [`FULL_COVERAGE`] of it, then the others, each by [`RegionView::quality`], then by source and
/// frame. Of them the best `sight.keep` ([`Sight::BEST`]: [`DECODED_VIEWS`]; `usize::MAX`:
/// all), exactly as a depth buffer for every candidate gives them; and what it took. Each view
/// also lists the faces of `around` it sees ([`RegionView::ring`]; `around`: sorted, none of
/// the Stelle, typically a ring around it), as the surface step would see them; the images and
/// their views of the Stelle are the same with any `around`. The images are listed from
/// `indices` (per source the index of its images); none is read or decoded. `cfg` is the
/// surface's (as its report has it). Progress: phase „Bilder der Stelle suchen“; a cancel is
/// checked at every image and every round and returns [`EditError::Cancelled`]. Same input,
/// same views.
///
/// Fails if `cfg` is invalid or two cameras of one source share a name.
///
/// # Panics
/// If a face is not one of `s`, `around` is not sorted or holds a face of the Stelle, or
/// `indices` are not as many as `sources`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn region_views(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    faces: &[u32],
    around: &[u32],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    cfg: &SurfaceConfig,
    sight: Sight,
    progress: &Progress,
) -> Result<(Vec<RegionView>, SightStats), EditError> {
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    cfg.check()?;
    assert_eq!(indices.len(), sources.len(), "an index per source");
    for src in sources {
        for (i, c) in src.cams.iter().enumerate() {
            if src.cams[..i].iter().any(|o| o.camera.name == c.camera.name) {
                return Err(MapError::Config(format!(
                    "Kamera {} ist doppelt angegeben",
                    c.camera.name
                ))
                .into());
            }
        }
    }
    let mut region = faces.to_vec();
    region.sort_unstable();
    region.dedup();
    if let Some(&last) = region.last() {
        assert!(
            (last as usize) < s.face_count(),
            "face {last} of {}",
            s.face_count()
        );
    }
    assert!(
        around.windows(2).all(|w| w[0] < w[1]),
        "the ring is not sorted"
    );
    assert!(
        around.iter().all(|f| region.binary_search(f).is_err()),
        "the ring holds a face of the Stelle"
    );
    let mut stats = SightStats::default();
    if region.is_empty() || sources.is_empty() {
        return Ok((Vec::new(), stats));
    }
    let ccfg = &cfg.color;
    let shape = Shape::new(s, &region);
    let candidates = candidates(&shape, sources, indices, ccfg, progress)?;
    stats.candidates = candidates.len();
    if candidates.is_empty() {
        return Ok((Vec::new(), stats));
    }
    // The faces looked at: the Stelle and the ring; the cone of the depth buffer around both.
    let mut looked = region.clone();
    looked.extend_from_slice(around);
    looked.sort_unstable();
    let reach = (!around.is_empty()).then(|| Shape::new(s, &looked));
    let reach = reach.as_ref().unwrap_or(&shape);

    // The geometry of the faces looked at: with the two rings around the Stelle, its smoothed
    // normals are those of the whole surface.
    let mut near = ring(s, &ring(s, &region));
    near.extend_from_slice(around);
    near.sort_unstable();
    near.dedup();
    let mesh = local_mesh(s, &near);
    let geometry = Faces::new(&mesh);
    let to_local = |f: &u32| near.binary_search(f).expect("a face of the part") as u32;
    let view: Vec<u32> = looked.iter().map(to_local).collect();
    let downscaled: Vec<Vec<Camera>> = sources
        .iter()
        .map(|src| {
            src.cams
                .iter()
                .map(|c| c.camera.downscaled(ccfg.downscale))
                .collect()
        })
        .collect();
    let shot = |c: usize| {
        let cand = &candidates[c];
        let (src, cam) = (cand.source as usize, cand.camera as usize);
        Shot {
            geometry: &downscaled[src][cam],
            full: &sources[src].cams[cam].camera,
            world_from_cam: cand.world_from_cam,
            rank: u32::try_from(c).expect("fewer than 2^32 images"),
            sharpness_norm: 1.0,
            rgb: None,
            min_texel: cfg.min_texel,
        }
    };

    // Per candidate the scores its image could give the faces of the Stelle with nothing in the
    // way, and the order by them.
    let open = OpenFaces::new(&mesh, &region.iter().map(to_local).collect::<Vec<u32>>());
    let bounds: Vec<Vec<f32>> = (0..candidates.len())
        .into_par_iter()
        .map(|c| open.scores(&mesh, &geometry, &shot(c), ccfg))
        .collect();
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    let weights = shape.weights();
    let total: f64 = weights.iter().sum();
    let order = by_bounds(&candidates, &bounds, &weights, total);

    // Depth buffers in rounds, until the best `keep` cannot change any more (see the module
    // docs): what each candidate sees of the Stelle (with its scores) and of the ring.
    let mut seen: Vec<Option<SeenFaces>> = vec![None; candidates.len()];
    let mut best = vec![0.0f32; region.len()];
    let mut round: Vec<usize> = order.iter().copied().take(sight.batch).collect();
    let mut size = sight.batch;
    while !round.is_empty() {
        stats.rounds += 1;
        let found: Vec<(SeenFaces, usize)> = round
            .par_iter()
            .map(|&c| {
                let cand = &candidates[c];
                let cam = &sources[cand.source as usize].cams[cand.camera as usize].camera;
                let splat = in_front(s, bvh, reach, &cand.world_from_cam, cam, ccfg);
                let corners = CornerFaces {
                    positions: s.positions(),
                    faces: s.faces(),
                };
                let (views, cells) = windowed_views(
                    &mesh,
                    &geometry,
                    &view,
                    corners,
                    &splat,
                    &shot(c),
                    ccfg,
                    sight.cutout,
                );
                // The bound of a face it sees is its score (`OpenFaces::scores`).
                #[cfg(debug_assertions)]
                for (f, v) in &views {
                    if let Ok(i) = region.binary_search(&near[*f as usize]) {
                        assert_eq!(v.score.to_bits(), bounds[c][i].to_bits(), "face {f}");
                    }
                }
                let (stelle, ring): Scored2 = views
                    .into_iter()
                    .map(|(f, v)| (near[f as usize], v.rank_score()))
                    .partition(|(f, _)| region.binary_search(f).is_ok());
                ((stelle, ring.into_iter().map(|(f, _)| f).collect()), cells)
            })
            .collect();
        for (&c, (faces, cells)) in round.iter().zip(found) {
            for (f, score) in &faces.0 {
                let i = region.binary_search(f).expect("a face of the Stelle");
                best[i] = best[i].max(*score);
            }
            stats.buffered += 1;
            stats.cells += cells;
            seen[c] = Some(faces);
        }
        if progress.is_cancelled() {
            return Err(EditError::Cancelled);
        }
        let ranked = ranked_views(&candidates, &seen, &region, &best, &weights, total);
        let rest: Vec<usize> = order
            .iter()
            .copied()
            .filter(|&c| seen[c].is_none())
            .collect();
        size = size.saturating_mul(2);
        round = if ranked.len() < sight.keep {
            rest.into_iter().take(size).collect()
        } else {
            let last = &ranked[sight.keep - 1];
            let mut held = vec![false; region.len()];
            for v in &ranked[..sight.keep] {
                for (f, _) in &v.faces {
                    held[region.binary_search(f).expect("a face of the Stelle")] = true;
                }
            }
            let limit = Limit {
                best: &best,
                held: &held,
                full: last.coverage >= FULL_COVERAGE,
                quality: last.quality,
            };
            rest.into_iter()
                .filter(|&c| limit.could_matter(&bounds[c], &weights, total))
                .take(size)
                .collect()
        };
    }
    let mut views = ranked_views(&candidates, &seen, &region, &best, &weights, total);
    views.truncate(sight.keep);
    Ok((views, stats))
}

/// The views of the candidates that got a depth buffer (those of `seen`) and see a face of the
/// Stelle `region`, ranked as [`region_views`] gives them, their qualities by `best`.
fn ranked_views(
    candidates: &[Candidate],
    seen: &[Option<SeenFaces>],
    region: &[u32],
    best: &[f32],
    weights: &[f64],
    total: f64,
) -> Vec<RegionView> {
    let mut views: Vec<RegionView> = candidates
        .iter()
        .zip(seen)
        .filter_map(|(c, seen)| {
            let (faces, ring) = seen.as_ref()?;
            if faces.is_empty() {
                return None;
            }
            let (mut covered, mut quality) = (0.0f64, 0.0f64);
            for (f, score) in faces {
                let i = region.binary_search(f).expect("a face of the Stelle");
                covered += weights[i];
                quality += weights[i] * f64::from(score / best[i]).sqrt();
            }
            Some(RegionView {
                source: c.source,
                frame: c.frame,
                stamp: c.stamp,
                camera: c.camera,
                world_from_cam: c.world_from_cam,
                coverage: (covered / total) as f32,
                quality: (quality / total) as f32,
                faces: faces.clone(),
                ring: ring.clone(),
            })
        })
        .collect();
    views.sort_by(|a, b| {
        let full = |v: &RegionView| v.coverage >= FULL_COVERAGE;
        full(b)
            .cmp(&full(a))
            .then(b.quality.total_cmp(&a.quality))
            .then((a.source, a.frame).cmp(&(b.source, b.frame)))
    });
    views
}

/// The candidates that may see a face of the Stelle (some score of their `bounds` above 0),
/// best first by their bounds: those that could cover [`FULL_COVERAGE`] of the Stelle, then by
/// the quality the bounds would give (relative to the best bound of each face), then by source
/// and frame.
fn by_bounds(
    candidates: &[Candidate],
    bounds: &[Vec<f32>],
    weights: &[f64],
    total: f64,
) -> Vec<usize> {
    let mut top = vec![0.0f32; weights.len()];
    for b in bounds {
        for (t, &x) in top.iter_mut().zip(b) {
            *t = t.max(x);
        }
    }
    let keys: Vec<(bool, f64)> = bounds
        .iter()
        .map(|b| {
            let (mut covered, mut quality) = (0.0f64, 0.0f64);
            for ((&x, &w), &t) in b.iter().zip(weights).zip(&top) {
                if x > 0.0 {
                    covered += w;
                    quality += w * f64::from(x / t).sqrt();
                }
            }
            (covered / total >= f64::from(FULL_COVERAGE), quality / total)
        })
        .collect();
    let mut order: Vec<usize> = (0..candidates.len())
        .filter(|&c| bounds[c].iter().any(|&x| x > 0.0))
        .collect();
    order.sort_by(|&a, &b| {
        let id = |c: usize| (candidates[c].source, candidates[c].frame);
        keys[b]
            .0
            .cmp(&keys[a].0)
            .then(keys[b].1.total_cmp(&keys[a].1))
            .then(id(a).cmp(&id(b)))
    });
    order
}

/// Relative slack of the bounds: they are the scores themselves where a face is seen; this
/// only keeps a rounding of their last bit from ever deciding.
const BOUND_SLACK: f32 = 1e-6;

/// What the best `keep` views are, after a round (see [`Limit::could_matter`]).
struct Limit<'a> {
    /// The best score of every face of the Stelle so far.
    best: &'a [f32],
    /// The faces of the Stelle the best `keep` views see.
    held: &'a [bool],
    /// The last of them: whether it covers [`FULL_COVERAGE`], and its quality.
    full: bool,
    quality: f32,
}

impl Limit<'_> {
    /// Whether a candidate without a depth buffer yet, which can give the faces of the Stelle
    /// at most the scores `bounds`, could still change the best views: it could raise the best
    /// score of a face one of them sees (and with it their qualities), or come before the last
    /// of them by the most it could cover and the best quality it could have.
    fn could_matter(&self, bounds: &[f32], weights: &[f64], total: f64) -> bool {
        let raises = bounds
            .iter()
            .zip(self.best)
            .zip(self.held)
            .any(|((&x, &b), &h)| h && x * (1.0 + BOUND_SLACK) > b);
        if raises {
            return true;
        }
        let (mut covered, mut quality) = (0.0f64, 0.0f64);
        for ((&x, &w), &b) in bounds.iter().zip(weights).zip(self.best) {
            if x > 0.0 {
                covered += w;
                quality += w * if b > 0.0 {
                    f64::from(x * (1.0 + BOUND_SLACK) / b).sqrt().min(1.0)
                } else {
                    1.0
                };
            }
        }
        // With a margin far above the rounding of the sums and of their `f32`: a tie with the
        // last of the best (decided by source and frame) counts as coming before it.
        let full = covered / total + 1e-6 >= f64::from(FULL_COVERAGE);
        match (full, self.full) {
            (true, false) => true,
            (false, true) => false,
            _ => quality / total + 1e-6 >= f64::from(self.quality),
        }
    }
}

/// The faces of a Stelle (indices into a part of the surface made a mesh) and their corners,
/// for the scores an image could give them with nothing in the way ([`OpenFaces::scores`]).
struct OpenFaces {
    /// The faces, in the order of the Stelle.
    faces: Vec<u32>,
    /// The corners of all of them (vertices of the mesh), sorted …
    verts: Vec<u32>,
    /// … and per face its corners as indices into `verts`.
    corners: Vec<[u32; 3]>,
    /// Per face the cap of its score at the finest texel ([`SurfaceConfig::min_texel`]),
    /// without the texel: half the length of the cross product of its edges.
    half_cross: Vec<f64>,
}

impl OpenFaces {
    fn new(mesh: &Mesh, faces: &[u32]) -> OpenFaces {
        let mut verts: Vec<u32> = faces
            .iter()
            .flat_map(|&f| mesh.indices[f as usize])
            .collect();
        verts.sort_unstable();
        verts.dedup();
        let corners = faces
            .iter()
            .map(|&f| {
                mesh.indices[f as usize].map(|v| verts.binary_search(&v).expect("a corner") as u32)
            })
            .collect();
        let half_cross = faces
            .iter()
            .map(|&f| {
                let [a, b, c] = mesh.indices[f as usize]
                    .map(|i| Vector3::from(mesh.positions[i as usize]).cast::<f64>());
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .collect();
        OpenFaces {
            faces: faces.to_vec(),
            verts,
            corners,
            half_cross,
        }
    }

    /// The score `shot` could give every face with nothing in the way: what
    /// [`windowed_views`] gives it if the depth buffer lets it through (the same numbers), 0
    /// where a test other than the depth buffer's rules it out (the tests only the depth
    /// buffer's image needs are left out: never below the score it can get).
    fn scores(&self, mesh: &Mesh, geometry: &Faces, shot: &Shot, cfg: &ColorConfig) -> Vec<f32> {
        let world_to_cam = shot.world_from_cam.inverse();
        let cam_pos = shot.world_from_cam.translation.vector;
        let uv: Vec<Option<[f64; 2]>> = self
            .verts
            .iter()
            .map(|&v| {
                let p = Vector3::from(mesh.positions[v as usize]).cast::<f64>();
                let pc = world_to_cam * Point3::from(p);
                if pc.x.hypot(pc.y).atan2(pc.z) >= cfg.rim_theta {
                    return None;
                }
                shot.full.project(&pc.coords)
            })
            .collect();
        let texel2 = shot.min_texel * shot.min_texel;
        self.faces
            .iter()
            .zip(&self.corners)
            .zip(&self.half_cross)
            .map(|((&f, c), &half_cross)| {
                let f = f as usize;
                let to_cam = cam_pos - Vector3::from(geometry.centroid[f]).cast::<f64>();
                let cos =
                    Vector3::from(geometry.normal[f]).cast::<f64>().dot(&to_cam) / to_cam.norm();
                if cos.is_nan() || cos <= 0.0 {
                    return 0.0;
                }
                let (Some(a), Some(b), Some(d)) =
                    (uv[c[0] as usize], uv[c[1] as usize], uv[c[2] as usize])
                else {
                    return 0.0;
                };
                let area =
                    0.5 * ((b[0] - a[0]) * (d[1] - a[1]) - (d[0] - a[0]) * (b[1] - a[1])).abs();
                let cap = half_cross / texel2;
                let cos_s =
                    Vector3::from(geometry.smooth[f]).cast::<f64>().dot(&to_cam) / to_cam.norm();
                let cos_s = cos_s.max(SMOOTH_COS_MIN);
                let area = area * (cos_s / cos).clamp(0.5, 2.0);
                let score = (area.min(cap) * cos_s) as f32 * shot.sharpness_norm;
                if score.is_nan() || score <= 0.0 {
                    0.0
                } else {
                    score
                }
            })
            .collect()
    }
}

/// Decodes the images of the first `k` of `views` (as [`region_views`] gives them: best first)
/// at full size, read from the bags at their places (`indices`, per source the index of its
/// images, [`read_indexed`]). Returns them as (index into `views`, image), by index. An image
/// that cannot be decoded or has another size than its camera is left out, so there may be
/// fewer. Progress: phase „Bilder der Stelle lesen“, one step per image; a cancel is checked at
/// every image and returns [`EditError::Cancelled`].
///
/// Fails if the bags cannot be read or no longer hold the images the views were found in.
///
/// # Panics
/// If a view is not of `sources` (its source or camera is not there, or its frame not in the
/// index of its source).
pub(crate) fn decode_views(
    views: &[RegionView],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    k: usize,
    progress: &Progress,
) -> Result<Vec<(usize, RgbImage)>, EditError> {
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    let chosen = &views[..k.min(views.len())];
    for v in chosen {
        let cams = sources.get(v.source as usize).map_or(0, |s| s.cams.len());
        assert!(
            (v.camera as usize) < cams,
            "view of source {} camera {}",
            v.source,
            v.camera
        );
    }
    progress.phase(PHASE_DECODE, Some(chosen.len() as u64));
    let mut decoded: Vec<(usize, RgbImage)> = Vec::with_capacity(chosen.len());
    let changed = || {
        EditError::from(MapError::Images(
            "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
        ))
    };
    for (s, src) in sources.iter().enumerate() {
        // (frame, index into `views`) of the images of this source, in stream order.
        let mut wanted: Vec<(usize, usize)> = chosen
            .iter()
            .enumerate()
            .filter(|(_, v)| v.source as usize == s)
            .map(|(i, v)| (v.frame as usize, i))
            .collect();
        if wanted.is_empty() {
            continue;
        }
        wanted.sort_unstable();
        // The views were found in this index: the same image there.
        let index = &indices[s];
        let headers: Vec<&ImageHeader> = index.headers().collect();
        for &(frame, i) in &wanted {
            let view = &chosen[i];
            let at = headers.partition_point(|h| h.index < frame);
            let same = headers.get(at).is_some_and(|h| {
                h.index == frame
                    && h.stamp_ns == view.stamp
                    && src.cams[view.camera as usize].camera.name == h.camera
            });
            if !same {
                return Err(changed());
            }
        }
        let frames: Vec<usize> = wanted.iter().map(|w| w.0).collect();
        let mut delivered = 0usize;
        let read = read_indexed(index, &frames, 1, |frame, image| {
            let i = wanted[delivered].1;
            debug_assert_eq!(wanted[delivered].0, frame);
            delivered += 1;
            progress.advance(1);
            if let Ok(image) = image {
                let cam = &src.cams[chosen[i].camera as usize].camera;
                if (image.rgb.width, image.rgb.height) == (cam.width, cam.height) {
                    decoded.push((i, image.rgb));
                }
            }
            !progress.is_cancelled()
        });
        if progress.is_cancelled() {
            return Err(EditError::Cancelled);
        }
        match read {
            Ok(_) => {}
            Err(ImageError::Changed(_)) => return Err(changed()),
            Err(e) => return Err(MapError::Images(e.to_string()).into()),
        }
        if delivered < wanted.len() {
            return Err(changed());
        }
    }
    decoded.sort_unstable_by_key(|d| d.0);
    Ok(decoded)
}

/// Faces with their scores.
type Scored = Vec<(u32, f32)>;
/// Faces with their scores, two lists.
type Scored2 = (Scored, Scored);
/// What one image sees: the faces of the Stelle with their scores, the faces of the ring.
type SeenFaces = (Scored, Vec<u32>);

/// The faces of a Stelle as the search for its images needs them.
struct Shape {
    /// Per face (in the order of the sorted Stelle) its centroid, unit normal and area (m²).
    centroid: Vec<[f64; 3]>,
    normal: Vec<[f64; 3]>,
    area: Vec<f64>,
    /// A sphere holding every corner.
    centre: [f64; 3],
    radius: f64,
}

impl Shape {
    fn new(s: &WeldedSurface, region: &[u32]) -> Shape {
        let centroid = region.iter().map(|&f| s.face_centroid(f)).collect();
        let normal = region.iter().map(|&f| s.face_normal(f)).collect();
        let area = region
            .iter()
            .map(|&f| 0.5 * norm(s.face_cross(f)))
            .collect();
        let corners = || region.iter().flat_map(|&f| s.face_corners(f));
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in corners() {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let centre = [0, 1, 2].map(|k| (lo[k] + hi[k]) / 2.0);
        let radius = corners().map(|p| norm(sub(p, centre))).fold(0.0, f64::max);
        Shape {
            centroid,
            normal,
            area,
            centre,
            radius,
        }
    }

    /// The weight of every face in the coverage and the quality: its area; every face alike
    /// if they have none.
    fn weights(&self) -> Vec<f64> {
        if self.area.iter().sum::<f64>() > 0.0 {
            self.area.clone()
        } else {
            vec![1.0; self.area.len()]
        }
    }
}

/// An image that may see a Stelle (see [`candidates`]).
struct Candidate {
    source: u32,
    frame: u32,
    stamp: u64,
    camera: u32,
    world_from_cam: Isometry3<f64>,
}

/// The images of `sources` that may see the Stelle `shape`, by their headers (`indices`, per
/// source the index of its images) and the poses alone (see the module docs), source by source
/// in stream order.
fn candidates(
    shape: &Shape,
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    ccfg: &ColorConfig,
    progress: &Progress,
) -> Result<Vec<Candidate>, EditError> {
    progress.phase(PHASE_FIND, None);
    let mut found = Vec::new();
    for (s, (src, index)) in sources.iter().zip(indices).enumerate() {
        for h in index.headers() {
            if progress.is_cancelled() {
                return Err(EditError::Cancelled);
            }
            let Some(camera) = src.cams.iter().position(|c| c.camera.name == h.camera) else {
                continue;
            };
            let t = h.stamp + ccfg.time_offset;
            let Some(body) = interpolate_pose(&src.poses, t) else {
                continue;
            };
            if angular_rate(&src.poses, t) > ccfg.max_angular_rate {
                continue;
            }
            let cam = &src.cams[camera];
            let world_from_cam = body * cam.cam_in_body;
            if may_see(shape, &world_from_cam, &cam.camera, ccfg) {
                found.push(Candidate {
                    source: s as u32,
                    frame: u32::try_from(h.index).expect("fewer than 2^32 images"),
                    stamp: h.stamp_ns,
                    camera: camera as u32,
                    world_from_cam,
                });
            }
        }
    }
    Ok(found)
}

/// Whether the camera at `world_from_cam` may see a face of `shape`: some centroid within
/// [`ColorConfig::max_dist`], some point within [`ColorConfig::rim_theta`] of the optical axis
/// (and the lens's range), some face facing it. Never false for a camera the surface step's
/// tests would let see a face.
fn may_see(
    shape: &Shape,
    world_from_cam: &Isometry3<f64>,
    cam: &Camera,
    ccfg: &ColorConfig,
) -> bool {
    let pos = world_from_cam.translation.vector;
    let pos = [pos.x, pos.y, pos.z];
    let to = sub(shape.centre, pos);
    let dist = norm(to);
    if dist - shape.radius > ccfg.max_dist + SLACK {
        return false;
    }
    if dist > shape.radius {
        let axis = world_from_cam.rotation * Vector3::z();
        let theta = (dot([axis.x, axis.y, axis.z], to) / dist)
            .clamp(-1.0, 1.0)
            .acos();
        // No point of the Stelle nearer to the axis than this.
        let nearest = theta - (shape.radius / dist).asin();
        if nearest >= ccfg.rim_theta + SLACK || nearest > cam.max_theta + SLACK {
            return false;
        }
    }
    shape.centroid.iter().zip(&shape.normal).any(|(c, n)| {
        let to_cam = sub(pos, *c);
        dot(*n, to_cam) > -SLACK * norm(to_cam)
    })
}

/// The faces of `s` that can hide a face of `shape` from the camera at `world_from_cam`, or
/// change a cell it is tested in (see the module docs), the Stelle's own among them: those in
/// the cone from the camera around the Stelle, widened by [`CONE_MARGIN`] times the angle of
/// [`MAX_SPLAT`] + 2 cells, that come nearer than the Stelle's farthest point and
/// [`ColorConfig::max_dist`]. Sorted.
fn in_front(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    shape: &Shape,
    world_from_cam: &Isometry3<f64>,
    cam: &Camera,
    ccfg: &ColorConfig,
) -> Vec<u32> {
    let apex = world_from_cam.translation.vector;
    let apex = [apex.x, apex.y, apex.z];
    let to = sub(shape.centre, apex);
    let dist = norm(to);
    let reach = (dist + shape.radius).min(ccfg.max_dist);
    // Pixels (at full size) of a depth-buffer cell, and their angle at the image centre.
    let cell = f64::from(ccfg.zbuffer_scale) * f64::from(ccfg.downscale);
    let margin = CONE_MARGIN * f64::from(MAX_SPLAT + 2) * cell / cam.fx.min(cam.fy);
    // Inside the Stelle's sphere every direction counts.
    let cone = (dist > shape.radius).then(|| {
        let axis = to.map(|c| c / dist);
        (axis, (shape.radius / dist).asin() + margin)
    });
    // Whether a sphere (centre, radius) reaches into the cone and nearer than `reach`.
    let reaches = |c: [f64; 3], r: f64| {
        let d = sub(c, apex);
        let l = norm(d);
        if l - r > reach + SLACK {
            return false;
        }
        let Some((axis, angle)) = cone else {
            return true;
        };
        if l <= r || angle >= std::f64::consts::PI {
            return true;
        }
        let theta = (dot(d, axis) / l).clamp(-1.0, 1.0).acos();
        theta - (r / l).asin() <= angle
    };
    bvh.faces_where(
        |lo, hi| {
            let c = [0, 1, 2].map(|k| (lo[k] + hi[k]) / 2.0);
            reaches(c, norm(sub(hi, c)))
        },
        |f| {
            let c = s.face_centroid(f);
            let r = s
                .face_corners(f)
                .iter()
                .map(|p| norm(sub(*p, c)))
                .fold(0.0, f64::max);
            reaches(c, r)
        },
    )
}

/// `faces` (sorted) and the faces sharing a vertex with one of them, sorted.
fn ring(s: &WeldedSurface, faces: &[u32]) -> Vec<u32> {
    let corners = s.faces();
    let mut touched = vec![false; s.positions().len()];
    for &f in faces {
        for v in corners[f as usize] {
            touched[v as usize] = true;
        }
    }
    (0..s.face_count() as u32)
        .into_par_iter()
        .filter(|&f| corners[f as usize].iter().any(|&v| touched[v as usize]))
        .collect()
}

/// The faces `local` (sorted) of `s` as a mesh of their own: face `i` is `local[i]`, its
/// vertices those of the surface (as `f32`, as the glb holds them) in the order of their ids.
fn local_mesh(s: &WeldedSurface, local: &[u32]) -> Mesh {
    let corners = s.faces();
    let mut verts: Vec<u32> = local.iter().flat_map(|&f| corners[f as usize]).collect();
    verts.sort_unstable();
    verts.dedup();
    let positions = verts
        .iter()
        .map(|&v| s.positions()[v as usize].map(|c| c as f32))
        .collect();
    let indices = local
        .iter()
        .map(|&f| corners[f as usize].map(|v| verts.binary_search(&v).expect("a corner") as u32))
        .collect();
    Mesh { positions, indices }
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

// ---- Second part: choose, bake, level -----------------------------------------------------

/// What [`retexture_region`] did.
#[derive(Clone, Debug, PartialEq)]
pub struct RetextureDone {
    /// The faces with a new texture leave their pages; the new pages hold them.
    pub patch: SurfacePatch,
    /// The variant taken, counted from 1, and how many there are: `(k + 1, n)` („Variante 2
    /// von 5“).
    pub variant_of: (u32, u32),
}

/// Bakes the Stelle `faces` (welded ids, any order) anew from the camera images of `sources`
/// onto pages of its own (see the module docs), as variant `variant`: of the `n` images that
/// can take it, best first, the `variant mod n`-th takes every face it sees; each further
/// variant tries the next one. `s` is the welded surface as the earlier steps left it (their
/// moves applied, [`WeldedSurface::apply_moves`]) and `bvh` the tree of its faces,
/// `current` the surface with their patches applied ([`super::apply_patches`]): its texture
/// around the Stelle is what the new images are matched to. `indices` hold, per source, where
/// its images lie ([`index_images`]). `cfg` is the surface's (as its report has it). Progress:
/// the phases „Bilder der Stelle suchen“, „Bilder der Stelle lesen“ and „Stelle neu backen“; a
/// cancel returns [`EditError::Cancelled`]. Same input, the same patch, byte for byte.
///
/// # Errors
/// - [`EditError::NoFaces`] without faces, [`EditError::NoImages`] if no image of `sources`
///   shows the Stelle, [`EditError::NoReference`] if nothing around it can match the brightness
///   (see the module docs): the Stelle keeps its texture.
/// - Otherwise if `cfg` is invalid, two cameras of one source share a name, the bags cannot be
///   read or have changed, or a page of `current` cannot be decoded.
///
/// # Panics
/// If a face is not one of `s`, `bvh` is not the tree of `s`'s faces, or `indices` are not
/// as many as `sources`.
#[allow(clippy::too_many_arguments)]
pub fn retexture_region(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    current: &PageMesh,
    faces: &[u32],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    variant: u32,
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<RetextureDone, EditError> {
    retexture(
        s,
        bvh,
        current,
        faces,
        sources,
        indices,
        variant,
        cfg,
        progress,
        Matching::Full,
        Sight::BEST,
        Judging::Consistent,
    )
}

/// How [`retexture`] matches the brightness of the new texture to the old one around it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Matching {
    /// Gains against the ring, the border levelled: [`retexture_region`].
    Full,
    /// Neither: the images' own colours, to compare with.
    #[cfg(test)]
    Raw,
}

/// Which views of a face [`retexture`] lets take it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Judging {
    /// Those the photo consistency keeps ([`judge_views`]): [`retexture_region`].
    Consistent,
    /// Every view, to compare with.
    #[cfg(test)]
    All,
}

/// [`retexture_region`], the brightness matched as `matching` says, the images found as
/// `sight` says, their views judged as `judging` says.
#[allow(clippy::too_many_arguments)]
fn retexture(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    current: &PageMesh,
    faces: &[u32],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    variant: u32,
    cfg: &SurfaceConfig,
    progress: &Progress,
    matching: Matching,
    sight: Sight,
    judging: Judging,
) -> Result<RetextureDone, EditError> {
    let prep = prepare(
        s, bvh, current, faces, sources, indices, cfg, sight, judging, progress,
    )?;
    let n = prep.ranked.len();
    let k = variant as usize % n;
    let main = prep.ranked[k];
    let (mut labels, taken) = prep.labels(main, sources, cfg);
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }

    // The faces the photo consistency took from the chosen image and another image took
    // instead: that image's colours there matched to the chosen image's around them, as an
    // image of its own (`baked_from`: the decoded image of every label).
    let mut baked_from: Vec<usize> = (0..prep.images.len()).collect();
    let mut matched_gains: Vec<[f32; 3]> = Vec::new();
    let mut replaced: Vec<(u32, usize)> = labels
        .iter()
        .enumerate()
        .filter_map(|(i, &l)| {
            let l = l?;
            (l != main as u32 && taken[i]).then_some((l, i))
        })
        .collect();
    replaced.sort_unstable();
    if !replaced.is_empty() {
        progress.info(format!(
            "Bilder der Stelle: {} von {} Flächen aus einem anderen Bild, weil das gewählte dort \
             etwas anderes zeigt als die übrigen",
            replaced.len(),
            prep.region.len()
        ));
    }
    let max_gain = cfg.color.exposure.max_gain;
    for same in replaced.chunk_by(|a, b| a.0 == b.0) {
        let c = same[0].0;
        let faces: Vec<usize> = same.iter().map(|r| r.1).collect();
        let m = prep
            .judged
            .match_gain(c, main as u32, &faces, &taken, max_gain);
        let own = prep.gains[c as usize];
        matched_gains.push(std::array::from_fn(|ch| own[ch] * m[ch]));
        let label = baked_from.len() as u32;
        baked_from.push(c as usize);
        for i in faces {
            labels[i] = Some(label);
        }
    }

    // The faces with a new texture: those an image takes that it can bake them from.
    let to_cam: Vec<Isometry3<f64>> = prep
        .images
        .iter()
        .map(|c| c.view.world_from_cam.inverse())
        .collect();
    let (new, new_labels): (Vec<u32>, Vec<u32>) = prep
        .region
        .iter()
        .zip(&labels)
        .enumerate()
        .filter_map(|(i, (&f, &l))| {
            let c = baked_from[l? as usize];
            let camera = &camera_of(sources, &prep.images[c].view).camera;
            projects(&prep.mesh, i, &to_cam[c], camera).then_some((f, l?))
        })
        .unzip();
    if new.is_empty() {
        return Err(EditError::NoImages);
    }

    // The old texture of the faces of the Stelle that keep it next to a new one, and the edges
    // where the new texture meets the old one.
    let kept: Vec<u32> = prep
        .region
        .iter()
        .zip(&prep.neighbours)
        .filter(|(f, _)| new.binary_search(f).is_err())
        .filter(|(_, edges)| edges.iter().flatten().any(|g| new.binary_search(g).is_ok()))
        .map(|(&f, _)| f)
        .collect();
    let kept_old = prep.old.faces_of(&kept, progress)?;
    let mut verts: Vec<u32> = new.iter().flat_map(|&f| s.faces()[f as usize]).collect();
    verts.sort_unstable();
    verts.dedup();
    let frozen = frozen_edges(s, &prep, &new, &verts, &kept, &kept_old);
    if !prep.own_gain.contains(&true) && frozen.is_empty() {
        return Err(EditError::NoReference);
    }

    // Bake, level, encode.
    let gains = prep.gains.iter().chain(&matched_gains);
    let images: Vec<RegionImage> = baked_from
        .iter()
        .zip(gains)
        .map(|(&c, &gain)| {
            let c = &prep.images[c];
            RegionImage {
                world_from_cam: c.view.world_from_cam,
                camera: &camera_of(sources, &c.view).camera,
                rgb: &c.rgb,
                gain: if matching == Matching::Full {
                    gain
                } else {
                    [1.0; 3]
                },
            }
        })
        .collect();
    let mesh = local_mesh(s, &new);
    let mut tex = bake_region(&mesh, &new_labels, &images, cfg);
    if matching == Matching::Full {
        let labels: Vec<Option<u32>> = new_labels.iter().map(|&l| Some(l)).collect();
        level_region(&mesh, &labels, &mut tex, &frozen);
    }
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    let jpegs = encode_pages(&tex, cfg.jpeg_quality)?;
    let pages = patch_pages(s, &new, &tex, &jpegs);
    Ok(RetextureDone {
        patch: SurfacePatch {
            moved: Vec::new(),
            removed_faces: new,
            pages,
        },
        variant_of: (k as u32 + 1, n as u32),
    })
}

/// The images the variants of [`retexture_region`] take for the Stelle `faces`, in order:
/// (source, frame).
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
fn variants(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    current: &PageMesh,
    faces: &[u32],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<Vec<(u32, u32)>, EditError> {
    let prep = prepare(
        s,
        bvh,
        current,
        faces,
        sources,
        indices,
        cfg,
        Sight::BEST,
        Judging::Consistent,
        progress,
    )?;
    Ok(prep
        .ranked
        .iter()
        .map(|&i| (prep.images[i].view.source, prep.images[i].view.frame))
        .collect())
}

/// A decoded image of a Stelle.
struct Decoded {
    view: RegionView,
    /// At full size.
    rgb: RgbImage,
}

/// The camera of a view.
fn camera_of<'a>(sources: &'a [TextureSource], view: &RegionView) -> &'a ColorCamera {
    &sources[view.source as usize].cams[view.camera as usize]
}

/// What every variant of a Stelle shares (see the module docs, steps 1 to 6).
struct Prepared<'a> {
    /// The Stelle (sorted) and the ring around it (sorted).
    region: Vec<u32>,
    ring: Vec<u32>,
    /// The Stelle as a mesh of its own: face `i` is `region[i]`.
    mesh: Mesh,
    /// The other faces on every edge of the Stelle's faces: per face and edge `k` (its corners
    /// `k`, `k + 1`), sorted.
    neighbours: Vec<[Vec<u32>; 3]>,
    /// The decoded images, as [`region_views`] ranked them.
    images: Vec<Decoded>,
    /// Per image its gain against the old texture, and whether it is its own (not a fallback).
    gains: Vec<[f32; 3]>,
    own_gain: Vec<bool>,
    /// The images best first: the variants.
    ranked: Vec<usize>,
    /// The photo consistency of the views ([`judge_views`]).
    judged: Judged,
    /// Where the faces of the Stelle and of the ring sit in the current surface.
    old: Located<'a>,
    /// The old texture of the ring's faces.
    ring_old: Vec<Option<OldFace>>,
}

/// Steps 1 to 6 of the module docs: the images of the Stelle `faces`, decoded, their gains,
/// their order and their views judged (as `judging` says).
#[allow(clippy::too_many_arguments)]
fn prepare<'a>(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    current: &'a PageMesh,
    faces: &[u32],
    sources: &[TextureSource],
    indices: &[Arc<ImageIndex>],
    cfg: &SurfaceConfig,
    sight: Sight,
    judging: Judging,
    progress: &Progress,
) -> Result<Prepared<'a>, EditError> {
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    cfg.check()?;
    let mut region = faces.to_vec();
    region.sort_unstable();
    region.dedup();
    if let Some(&last) = region.last() {
        assert!(
            (last as usize) < s.face_count(),
            "face {last} of {}",
            s.face_count()
        );
    }
    if region.is_empty() {
        return Err(EditError::NoFaces);
    }
    let ring = ring_around(s, &region);
    let sight = Sight {
        keep: sight.keep.max(DECODED_VIEWS),
        ..sight
    };
    let (views, looked) = region_views(
        s, bvh, &region, &ring, sources, indices, cfg, sight, progress,
    )?;
    if looked.candidates > 0 {
        progress.info(format!(
            "Bilder der Stelle: {} von {} möglichen genau geprüft ({} Runde(n))",
            looked.buffered, looked.candidates, looked.rounds
        ));
    }
    if views.is_empty() {
        return Err(EditError::NoImages);
    }
    let decoded = decode_views(&views, sources, indices, DECODED_VIEWS, progress)?;
    if decoded.is_empty() {
        return Err(EditError::NoImages);
    }
    let images: Vec<Decoded> = decoded
        .into_iter()
        .map(|(i, rgb)| Decoded {
            view: views[i].clone(),
            rgb,
        })
        .collect();
    drop(views);
    progress.phase(PHASE_BAKE, None);

    // The old texture of the ring, and the gains of the images against it.
    let mut looked = region.clone();
    looked.extend_from_slice(&ring);
    looked.sort_unstable();
    let old = Located::new(s, current, looked);
    let ring_old = old.faces_of(&ring, progress)?;
    let lin = linear_table();
    let max_gain = cfg.color.exposure.max_gain;
    let own: Vec<Option<[f32; 3]>> = images
        .par_iter()
        .map(|c| {
            let camera = &camera_of(sources, &c.view).camera;
            ring_gain(s, c, camera, &ring, &ring_old, max_gain, &lin)
        })
        .collect();
    let gains = gains_with_fallback(&images, &own);
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }

    let mesh = local_mesh(s, &region);
    let ranked = rank(s, &region, &mesh, &images, &gains, sources, cfg);
    let judged = match judging {
        Judging::Consistent => judge_views(s, bvh, &region, &images, &gains, sources, &lin),
        #[cfg(test)]
        Judging::All => Judged::none(region.len()),
    };
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    Ok(Prepared {
        neighbours: edge_neighbours(s, &region),
        region,
        ring,
        mesh,
        images,
        gains,
        own_gain: own.iter().map(Option::is_some).collect(),
        ranked,
        judged,
        old,
        ring_old,
    })
}

impl Prepared<'_> {
    /// The image of every face of the Stelle (an index into `images`) for the variant whose
    /// image is `main` (see the module docs, step 6); `None`: no image takes the face. And the
    /// faces `main` does not take after the photo consistency ([`Prepared::taken_from`]); the
    /// other images take no face whose view of it the photo consistency leaves out
    /// ([`Judged::left_out`]).
    fn labels(
        &self,
        main: usize,
        sources: &[TextureSource],
        cfg: &SurfaceConfig,
    ) -> (Vec<Option<u32>>, Vec<bool>) {
        let n = self.region.len();
        let main = main as u32;
        let taken = self.taken_from(main);
        // Per face its views in the decoded images, best first (equal: the earlier image).
        let mut views: Vec<Vec<(u32, f32)>> = vec![Vec::new(); n];
        for (c, image) in self.images.iter().enumerate() {
            let c = c as u32;
            for &(f, score) in &image.view.faces {
                let i = self.region.binary_search(&f).expect("a face of the Stelle");
                let out = if c == main {
                    taken[i]
                } else {
                    self.judged.left_out(i, c)
                };
                if !out {
                    views[i].push((c, score));
                }
            }
        }
        for v in &mut views {
            v.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
        }
        let held: Vec<bool> = views
            .iter()
            .map(|v| v.iter().any(|x| x.0 == main))
            .collect();
        let mut labels: Vec<Option<u32>> = views
            .iter()
            .zip(&held)
            .map(|(v, &h)| {
                if h {
                    Some(main)
                } else {
                    v.first().map(|x| x.0)
                }
            })
            .collect();
        if cfg.smooth_weight > 0.0 {
            // ICM over the faces the chosen image does not see; its own faces and the outside
            // (never the same image) are held.
            let inside: Vec<Vec<u32>> = self
                .neighbours
                .iter()
                .map(|edges| {
                    let mut v: Vec<u32> = edges
                        .iter()
                        .flatten()
                        .filter_map(|g| self.region.binary_search(g).ok().map(|i| i as u32))
                        .collect();
                    v.sort_unstable();
                    v.dedup();
                    v
                })
                .collect();
            for _ in 0..ICM_ROUNDS {
                let mut changed = false;
                for f in 0..n {
                    if held[f] {
                        continue;
                    }
                    let Some(&(_, best)) = views[f].first() else {
                        continue;
                    };
                    if best <= 0.0 {
                        continue;
                    }
                    let mut pick = (f32::INFINITY, labels[f]);
                    for &(c, score) in views[f].iter().take_while(|v| v.1 > 0.0) {
                        let differ = inside[f]
                            .iter()
                            .filter(|&&g| labels[g as usize] != Some(c))
                            .count();
                        let cost = -(score / best).sqrt() + cfg.smooth_weight * differ as f32;
                        if cost < pick.0 {
                            pick = (cost, Some(c));
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
        }
        // Islands and small gray patches take the image around them where its camera sees them,
        // but not where the photo consistency kept it from them.
        let cams = all_cameras(sources);
        let cam_base = camera_bases(sources);
        let table = ViewTable {
            images: self
                .images
                .iter()
                .map(|c| UsedImage {
                    source: c.view.source as usize,
                    camera: c.view.camera as usize,
                    exposure_camera: cam_base[c.view.source as usize] + c.view.camera as usize,
                    stamp: c.view.stamp as f64 * 1e-9,
                    world_from_cam: c.view.world_from_cam,
                })
                .collect(),
            per_face: Vec::new(),
            gains: self.gains.clone(),
            vignetting: Vignetting::flat(cams.len(), cfg.color.exposure.max_gain),
        };
        let kept = |f: usize, c: u32| {
            if c == main {
                !taken[f]
            } else {
                !self.judged.left_out(f, c)
            }
        };
        absorb_islands_where(&self.mesh, &table, &cams, cfg, &mut labels, &kept);
        (labels, taken)
    }

    /// The faces of the Stelle the chosen image `main` does not take although it sees them (see
    /// the module docs, step 6): those where the photo consistency leaves its view out, in groups
    /// over shared edges of at least [`ISLAND_FACES`] faces (a smaller group stays with it, as an
    /// island would), and the faces sharing an edge with such a group that it sees and another
    /// image sees and keeps: a thing in front covers the faces along its edge only in part, and
    /// their mean colour differs less.
    fn taken_from(&self, main: u32) -> Vec<bool> {
        let n = self.region.len();
        let mut sees = vec![false; n];
        for &(f, _) in &self.images[main as usize].view.faces {
            sees[self.region.binary_search(&f).expect("a face of the Stelle")] = true;
        }
        let mut others = vec![false; n];
        for (c, image) in self.images.iter().enumerate() {
            if c as u32 == main {
                continue;
            }
            for &(f, _) in &image.view.faces {
                let i = self.region.binary_search(&f).expect("a face of the Stelle");
                others[i] |= !self.judged.left_out(i, c as u32);
            }
        }
        // The faces of the Stelle sharing an edge with each.
        let beside: Vec<Vec<usize>> = self
            .neighbours
            .iter()
            .map(|edges| {
                let mut v: Vec<usize> = edges
                    .iter()
                    .flatten()
                    .filter_map(|g| self.region.binary_search(g).ok())
                    .collect();
                v.sort_unstable();
                v.dedup();
                v
            })
            .collect();
        let out: Vec<bool> = (0..n).map(|i| self.judged.left_out(i, main)).collect();
        let mut taken = vec![false; n];
        let mut group_of = vec![usize::MAX; n];
        for start in 0..n {
            if !out[start] || group_of[start] != usize::MAX {
                continue;
            }
            let mut group = vec![start];
            group_of[start] = start;
            let mut k = 0;
            while k < group.len() {
                for &j in &beside[group[k]] {
                    if out[j] && group_of[j] == usize::MAX {
                        group_of[j] = start;
                        group.push(j);
                    }
                }
                k += 1;
            }
            if group.len() < ISLAND_FACES {
                continue;
            }
            for &i in &group {
                taken[i] = true;
            }
        }
        let grown: Vec<usize> = (0..n)
            .filter(|&j| !taken[j] && sees[j] && others[j])
            .filter(|&j| beside[j].iter().any(|&i| taken[i]))
            .collect();
        for j in grown {
            taken[j] = true;
        }
        taken
    }
}

/// Per source the number of its first camera among the cameras of all sources in turn
/// ([`all_cameras`], [`UsedImage::exposure_camera`]).
fn camera_bases(sources: &[TextureSource]) -> Vec<usize> {
    sources
        .iter()
        .scan(0, |base, src| {
            let b = *base;
            *base += src.cams.len();
            Some(b)
        })
        .collect()
}

/// The faces within [`RING_ROUNDS`] rounds around the Stelle `region` (sorted): the faces
/// sharing a vertex with it, those sharing one with them, and so on; without the Stelle's own.
fn ring_around(s: &WeldedSurface, region: &[u32]) -> Vec<u32> {
    let mut all = region.to_vec();
    for _ in 0..RING_ROUNDS {
        all = ring(s, &all);
    }
    all.retain(|f| region.binary_search(f).is_err());
    all
}

/// The other faces on every edge of the faces `faces` (sorted): per face and edge `k` (its
/// corners `k` and `k + 1`), sorted.
fn edge_neighbours(s: &WeldedSurface, faces: &[u32]) -> Vec<[Vec<u32>; 3]> {
    let corners = s.faces();
    let key = |t: [u32; 3], k: usize| {
        let (a, b) = (t[k], t[(k + 1) % 3]);
        (a.min(b), a.max(b))
    };
    let mut edges: Vec<((u32, u32), u32)> = ring(s, faces)
        .into_iter()
        .flat_map(|f| (0..3).map(move |k| (key(corners[f as usize], k), f)))
        .collect();
    edges.par_sort_unstable();
    edges.dedup();
    faces
        .iter()
        .map(|&f| {
            std::array::from_fn(|k| {
                let e = key(corners[f as usize], k);
                let lo = edges.partition_point(|x| x.0 < e);
                let hi = edges.partition_point(|x| x.0 <= e);
                edges[lo..hi]
                    .iter()
                    .map(|x| x.1)
                    .filter(|&g| g != f)
                    .collect()
            })
        })
        .collect()
}

/// The bits of a place, +0.0 and −0.0 alike (as the welding joins corners).
fn place(p: [f64; 3]) -> [u64; 3] {
    p.map(|c| (c + 0.0).to_bits())
}

/// Where faces of the surface sit in the current surface: the computed pages as the earlier
/// steps left them, then the pages those added. A face is found by the bits of its corners,
/// which every copy of a vertex shares (moved ones too: the steps move every copy alike).
struct Located<'a> {
    current: &'a PageMesh,
    /// The faces asked for (sorted) and the triangle (page, triangle) holding each.
    faces: Vec<u32>,
    at: Vec<Option<(u32, u32)>>,
}

impl<'a> Located<'a> {
    /// Finds `faces` (sorted) of `s` in `current`. Only the pages that can hold them are looked
    /// through: the computed pages with a copy of one of their vertices, and the added pages.
    fn new(s: &WeldedSurface, current: &'a PageMesh, faces: Vec<u32>) -> Located<'a> {
        let corners = s.faces();
        let mut verts: Vec<u32> = faces.iter().flat_map(|&f| corners[f as usize]).collect();
        verts.sort_unstable();
        verts.dedup();
        // Only looked up, never iterated.
        let by_place: HashMap<[u64; 3], u32> = verts
            .iter()
            .map(|&v| (place(s.positions()[v as usize]), v))
            .collect();
        let by_corners: HashMap<[u32; 3], u32> =
            faces.iter().map(|&f| (corners[f as usize], f)).collect();
        let mut pages: Vec<u32> = verts
            .iter()
            .flat_map(|&v| s.copies(v).iter().map(|c| c.0))
            .collect();
        pages.extend(s.page_count() as u32..current.pages.len() as u32);
        pages.sort_unstable();
        pages.dedup();
        pages.retain(|&p| (p as usize) < current.pages.len());
        let found: Vec<Vec<(u32, u32, u32)>> = pages
            .par_iter()
            .map(|&p| {
                let page = &current.pages[p as usize];
                page.indices
                    .iter()
                    .enumerate()
                    .filter_map(|(t, tri)| {
                        let mut v = [0u32; 3];
                        for (k, &c) in tri.iter().enumerate() {
                            v[k] = *by_place.get(&place(*page.positions.get(c as usize)?))?;
                        }
                        by_corners.get(&v).map(|&f| (f, p, t as u32))
                    })
                    .collect()
            })
            .collect();
        let mut at = vec![None; faces.len()];
        for (f, p, t) in found.into_iter().flatten() {
            at[faces.binary_search(&f).expect("a face asked for")] = Some((p, t));
        }
        Located { current, faces, at }
    }

    /// The old texture of `faces` (among those asked for, sorted; [`OldFace`]); `None` where a
    /// face was not found, is unseen (gray: its corners share one texture coordinate) or its
    /// page has no JPEG. Every page needed is decoded once, [`PAGES_AT_ONCE`] at a time; a
    /// cancel is checked before each.
    fn faces_of(
        &self,
        faces: &[u32],
        progress: &Progress,
    ) -> Result<Vec<Option<OldFace>>, EditError> {
        // (page, face, its texture coordinates) of every face with a texture.
        let mut wanted: Vec<(u32, usize, [[f32; 2]; 3])> = Vec::new();
        for (i, f) in faces.iter().enumerate() {
            let Ok(j) = self.faces.binary_search(f) else {
                continue;
            };
            let Some((p, t)) = self.at[j] else {
                continue;
            };
            let page = &self.current.pages[p as usize];
            let jpeg = page
                .image
                .as_ref()
                .is_some_and(|im| im.kind == ImageKind::Jpeg);
            if !jpeg || page.uvs.len() != page.positions.len() {
                continue;
            }
            let uvs = page.indices[t as usize].map(|c| page.uvs[c as usize]);
            if !gray(uvs) {
                wanted.push((p, i, uvs));
            }
        }
        wanted.sort_unstable_by_key(|w| (w.0, w.1));
        let mut pages: Vec<u32> = wanted.iter().map(|w| w.0).collect();
        pages.dedup();
        let lin = linear_table();
        let mut out: Vec<Option<OldFace>> = vec![None; faces.len()];
        for chunk in pages.chunks(PAGES_AT_ONCE) {
            if progress.is_cancelled() {
                return Err(EditError::Cancelled);
            }
            let decoded: Vec<_> = chunk
                .par_iter()
                .map(|&p| {
                    let image = self.current.pages[p as usize].image.as_ref();
                    decode_jpeg(&image.expect("a page with a JPEG").bytes, 1)
                })
                .collect();
            for (&p, image) in chunk.iter().zip(decoded) {
                let image = image.map_err(|e| {
                    MapError::Images(format!("Seite {} der Oberfläche: {e}", p + 1))
                })?;
                let lo = wanted.partition_point(|w| w.0 < p);
                let hi = wanted.partition_point(|w| w.0 <= p);
                let read: Vec<(usize, OldFace)> = wanted[lo..hi]
                    .par_iter()
                    .map(|&(_, i, uvs)| (i, OldFace::read(&image, uvs, &lin)))
                    .collect();
                for (i, old) in read {
                    out[i] = Some(old);
                }
            }
        }
        Ok(out)
    }
}

/// Whether a triangle's texture coordinates are all one point: an unseen (gray) face.
fn gray(uvs: [[f32; 2]; 3]) -> bool {
    uvs[0] == uvs[1] && uvs[1] == uvs[2]
}

/// The old texture of a face: its colour (linear, the mean at [`FACE_SAMPLES`]), and per edge
/// `k` (its corners `k` and `k + 1`) the mean at [`SEAM_SAMPLES`] of the way from corner `k`
/// towards `k + 1`, then from `k + 1` towards `k`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct OldFace {
    colour: [f64; 3],
    edges: [[[f64; 3]; 2]; 3],
}

impl OldFace {
    /// Reads a face from its page's image (decoded) and its corners' texture coordinates.
    fn read(image: &RgbImage, uvs: [[f32; 2]; 3], lin: &[f32; 256]) -> OldFace {
        let size = [image.width, image.height].map(f64::from);
        // As the seam levelling samples a page: bilinear, pixel centres at integers.
        let at = |w: [f64; 3]| {
            let px = [0, 1].map(|a| {
                let uv = (0..3).map(|k| w[k] * f64::from(uvs[k][a])).sum::<f64>();
                (uv * size[a] - 0.5) as f32
            });
            sample_linear(image, px, lin)
        };
        let mean = |ws: &[[f64; 3]]| {
            let mut sum = [0.0f64; 3];
            for &w in ws {
                let c = at(w);
                for k in 0..3 {
                    sum[k] += f64::from(c[k]);
                }
            }
            sum.map(|x| x / ws.len() as f64)
        };
        let along = |from: usize, to: usize| {
            let ws: Vec<[f64; 3]> = SEAM_SAMPLES
                .iter()
                .map(|&t| {
                    let mut w = [0.0; 3];
                    w[from] = 1.0 - t;
                    w[to] = t;
                    w
                })
                .collect();
            mean(&ws)
        };
        OldFace {
            colour: mean(&FACE_SAMPLES),
            edges: std::array::from_fn(|k| [along(k, (k + 1) % 3), along((k + 1) % 3, k)]),
        }
    }
}

/// The median of `v` (the mean of the middle two for an even count); 0 if empty.
fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

/// The linear colour of face `f` of `s` in `rgb` (full size) of `camera` at `world_to_cam`: the
/// mean at [`FACE_SAMPLES`]; `None` if one of them is not in the image.
fn face_colour(
    s: &WeldedSurface,
    f: u32,
    world_to_cam: &Isometry3<f64>,
    camera: &Camera,
    rgb: &RgbImage,
    lin: &[f32; 256],
) -> Option<[f64; 3]> {
    let corners = s.face_corners(f);
    let mut sum = [0.0f64; 3];
    for w in FACE_SAMPLES {
        let p = [0, 1, 2].map(|a| (0..3).map(|k| w[k] * corners[k][a]).sum::<f64>());
        let uv = camera.project(&(world_to_cam * Point3::from(p)).coords)?;
        let c = sample_linear(rgb, [uv[0] as f32, uv[1] as f32], lin);
        for k in 0..3 {
            sum[k] += f64::from(c[k]);
        }
    }
    Some(sum.map(|x| x / FACE_SAMPLES.len() as f64))
}

/// Linear luminance of a colour.
fn lum(c: [f64; 3]) -> f64 {
    luminance(c.map(|x| x as f32))
}

/// The gain of `image` against the old texture (see the module docs, step 5): per channel the
/// median of `ln((new + GAIN_FLOOR) / (old + GAIN_FLOOR))` over the faces of the ring it sees
/// whose old texture is known and where neither side is darker than [`MIN_SEAM_LUMINANCE`],
/// exponentiated and clamped to `[1 / max_gain, max_gain]`; `None` with fewer than
/// [`MIN_GAIN_FACES`] such faces. `old` holds the old texture of every face of `ring`.
fn ring_gain(
    s: &WeldedSurface,
    image: &Decoded,
    camera: &Camera,
    ring: &[u32],
    old: &[Option<OldFace>],
    max_gain: f64,
    lin: &[f32; 256],
) -> Option<[f32; 3]> {
    let to_cam = image.view.world_from_cam.inverse();
    let mut logs: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for &f in &image.view.ring {
        let Ok(i) = ring.binary_search(&f) else {
            continue;
        };
        let Some(old) = &old[i] else {
            continue;
        };
        let Some(new) = face_colour(s, f, &to_cam, camera, &image.rgb, lin) else {
            continue;
        };
        if lum(new) < MIN_SEAM_LUMINANCE || lum(old.colour) < MIN_SEAM_LUMINANCE {
            continue;
        }
        for (k, l) in logs.iter_mut().enumerate() {
            l.push(((new[k] + GAIN_FLOOR) / (old.colour[k] + GAIN_FLOOR)).ln());
        }
    }
    if logs[0].len() < MIN_GAIN_FACES {
        return None;
    }
    let lim = max_gain.max(1.0);
    Some(logs.map(|mut l| median(&mut l).exp().clamp(1.0 / lim, lim) as f32))
}

/// Every image's gain: its own, else the median (per channel) of the own gains of the other
/// images of its source, else of all images, else 1.
fn gains_with_fallback(images: &[Decoded], own: &[Option<[f32; 3]>]) -> Vec<[f32; 3]> {
    let median_of = |pick: &dyn Fn(usize) -> bool| -> Option<[f32; 3]> {
        let mut per: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for (i, g) in own.iter().enumerate() {
            if let Some(g) = g.filter(|_| pick(i)) {
                for k in 0..3 {
                    per[k].push(f64::from(g[k]));
                }
            }
        }
        (!per[0].is_empty()).then(|| per.map(|mut v| median(&mut v) as f32))
    };
    let all = median_of(&|_| true);
    images
        .iter()
        .zip(own)
        .map(|(c, g)| {
            g.or_else(|| median_of(&|j| images[j].view.source == c.view.source))
                .or(all)
                .unwrap_or([1.0; 3])
        })
        .collect()
}

/// The size on the surface of a pixel of `image` over the faces of the Stelle it sees (m):
/// `sqrt(their area / their area in the image)`; `None` if none of them is in the image.
fn texel_on(s: &WeldedSurface, image: &Decoded, camera: &Camera) -> Option<f64> {
    let to_cam = image.view.world_from_cam.inverse();
    let (mut surface, mut pixels) = (0.0f64, 0.0f64);
    for &(f, _) in &image.view.faces {
        let mut uv = [[0.0f64; 2]; 3];
        let mut inside = true;
        for (k, p) in s.face_corners(f).into_iter().enumerate() {
            match camera.project(&(to_cam * Point3::from(p)).coords) {
                Some(q) => uv[k] = q,
                None => inside = false,
            }
        }
        if inside {
            pixels += 0.5
                * ((uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1])
                    - (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]))
                    .abs();
            surface += 0.5 * norm(s.face_cross(f));
        }
    }
    (pixels > 0.0).then(|| (surface / pixels).sqrt())
}

/// The images best first (see the module docs, step 6): indices into `images`. `mesh` is the
/// Stelle `region` as a mesh of its own.
fn rank(
    s: &WeldedSurface,
    region: &[u32],
    mesh: &Mesh,
    images: &[Decoded],
    gains: &[[f32; 3]],
    sources: &[TextureSource],
    cfg: &SurfaceConfig,
) -> Vec<usize> {
    // The texel size the images are compared at: the finest any of them reaches, but not finer
    // than the surface's nor than keeps one measure small.
    let area: f64 = region.iter().map(|&f| 0.5 * norm(s.face_cross(f))).sum();
    let finest = images
        .iter()
        .filter_map(|c| texel_on(s, c, &camera_of(sources, &c.view).camera))
        .fold(f64::INFINITY, f64::min);
    let mut texel = cfg.min_texel.max((area / SHARPNESS_TEXELS).sqrt());
    if finest.is_finite() {
        texel = texel.max(finest);
    }
    let sharp: Vec<f64> = images
        .par_iter()
        .zip(gains)
        .map(|(c, &gain)| {
            let mut seen = vec![false; region.len()];
            for (f, _) in &c.view.faces {
                seen[region.binary_search(f).expect("a face of the Stelle")] = true;
            }
            let image = RegionImage {
                world_from_cam: c.view.world_from_cam,
                camera: &camera_of(sources, &c.view).camera,
                rgb: &c.rgb,
                gain,
            };
            footprint_sharpness(mesh, &seen, &image, texel).sharpness()
        })
        .collect();
    let mut positive: Vec<f64> = sharp.iter().copied().filter(|&x| x > 0.0).collect();
    let middle = median(&mut positive);
    let score: Vec<f64> = images
        .iter()
        .zip(&sharp)
        .map(|(c, &x)| {
            let norm = if middle > 0.0 {
                (x / middle).min(1.0)
            } else {
                1.0
            };
            f64::from(c.view.quality) * norm
        })
        .collect();
    let mut order: Vec<usize> = (0..images.len()).collect();
    order.sort_by(|&a, &b| {
        let full = |i: usize| images[i].view.coverage >= FULL_COVERAGE;
        let id = |i: usize| (images[i].view.source, images[i].view.frame);
        full(b)
            .cmp(&full(a))
            .then(score[b].total_cmp(&score[a]))
            .then(id(a).cmp(&id(b)))
    });
    order
}

/// The photo consistency of the views of a Stelle (see the module docs): per face of `region`
/// (sorted; its faces in the tree `bvh` of `s`) the decoded `images` (indices, sorted) whose
/// view of it shows something else than the others' views. A view gives the face its colour in
/// its image at 1/[`SHARPNESS_DOWNSCALE`] ([`quarter_colours`]) against the median of its
/// colours of the faces of the Stelle within [`CONSISTENCY_RADIUS`]: per channel
/// `ln((colour + MIN_SEAM_LUMINANCE) / (median + MIN_SEAM_LUMINANCE))`. The views of a face vote
/// as in the surface step, one per camera and stretch of capture time, and a view further than
/// `ln` [`CONSISTENCY_RATIO`] from the median of the votes in a channel, where the votes within
/// it are a strict majority ([`consistent`]), is left out, unless it lies within that of the
/// median of the votes ([`vote_median`]) at a face within [`CONSISTENCY_SHIFT`]. Deterministic:
/// per image and face an ordered map, medians of values.
fn judge_views(
    s: &WeldedSurface,
    bvh: &FaceBvh,
    region: &[u32],
    images: &[Decoded],
    gains: &[[f32; 3]],
    sources: &[TextureSource],
    lin: &[f32; 256],
) -> Judged {
    let n = region.len();
    // The faces of the Stelle around each (indices into `region`, sorted; itself among them).
    let around: Vec<Vec<u32>> = region
        .par_iter()
        .map(|&f| {
            bvh.faces_within(s, s.face_centroid(f), CONSISTENCY_RADIUS)
                .into_iter()
                .filter_map(|g| region.binary_search(&g).ok().map(|i| i as u32))
                .collect()
        })
        .collect();
    // Per image the colour of every face it sees, …
    let colours: Vec<Vec<Option<[f64; 3]>>> = images
        .par_iter()
        .zip(gains)
        .map(|(c, gain)| {
            let camera = &camera_of(sources, &c.view).camera;
            quarter_colours(s, region, c, camera, gain, lin)
        })
        .collect();
    // … against the median of its colours around the face.
    let relative: Vec<Vec<Option<[f64; 3]>>> = colours
        .par_iter()
        .map(|colours| {
            let mut near: Vec<f64> = Vec::new();
            (0..n)
                .map(|i| {
                    let own = colours[i]?;
                    Some(std::array::from_fn(|k| {
                        near.clear();
                        near.extend(
                            around[i]
                                .iter()
                                .filter_map(|&j| colours[j as usize].map(|c| c[k])),
                        );
                        let middle = select_median(&mut near);
                        ((own[k] + MIN_SEAM_LUMINANCE) / (middle + MIN_SEAM_LUMINANCE)).ln()
                    }))
                })
                .collect()
        })
        .collect();
    let cam_base = camera_bases(sources);
    let tolerance = CONSISTENCY_RATIO.ln();
    let judged: Vec<FaceVotes> = (0..n)
        .into_par_iter()
        .map(|i| {
            let f = region[i];
            let mut views: Vec<(u32, f32)> = images
                .iter()
                .enumerate()
                .filter(|&(c, _)| relative[c][i].is_some())
                .filter_map(|(c, image)| {
                    let at = image.view.faces.binary_search_by_key(&f, |v| v.0).ok()?;
                    Some((c as u32, image.view.faces[at].1))
                })
                .collect();
            views.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
            let views: Vec<(u32, [f64; 3])> = views
                .iter()
                .map(|v| (v.0, relative[v.0 as usize][i].expect("a judged view")))
                .collect();
            let voter = voters(views.len(), |k| {
                let view = &images[views[k].0 as usize].view;
                let camera = cam_base[view.source as usize] + view.camera as usize;
                stretch_of(camera, view.stamp as f64 * 1e-9)
            });
            let values: Vec<[f64; 3]> = views.iter().map(|v| v.1).collect();
            let median = vote_median(&values, &voter[..views.len()]);
            FaceVotes {
                views,
                voter,
                median,
            }
        })
        .collect();
    // A view that differs where the votes decide is left out, unless it agrees with the median
    // of the votes at a face within CONSISTENCY_SHIFT.
    let near = |a: &[f64; 3], b: &[f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() <= tolerance);
    let out = (0..n)
        .into_par_iter()
        .map(|i| {
            let FaceVotes { views, voter, .. } = &judged[i];
            let values: Vec<[f64; 3]> = views.iter().map(|v| v.1).collect();
            let Some(kept) = consistent(&values, &voter[..views.len()], tolerance) else {
                return Vec::new();
            };
            let shifted: Vec<usize> = bvh
                .faces_within(s, s.face_centroid(region[i]), CONSISTENCY_SHIFT)
                .into_iter()
                .filter_map(|g| region.binary_search(&g).ok())
                .collect();
            let mut out: Vec<u32> = views
                .iter()
                .zip(&kept)
                .filter(|&(v, &ok)| {
                    !ok && !shifted
                        .iter()
                        .any(|&j| judged[j].median.is_some_and(|m| near(&v.1, &m)))
                })
                .map(|(v, _)| v.0)
                .collect();
            out.sort_unstable();
            out
        })
        .collect();
    Judged {
        out,
        colours,
        around,
    }
}

/// The views of one face of a Stelle as [`judge_views`] judges them.
struct FaceVotes {
    /// The images that see it with their values, best first (equal: the earlier image), as
    /// [`Prepared::labels`] has them.
    views: Vec<(u32, [f64; 3])>,
    /// Which of them vote ([`voters`]) …
    voter: [bool; MAX_VIEWS_PER_FACE],
    /// … and the median of the votes ([`vote_median`]).
    median: Option<[f64; 3]>,
}

/// What [`judge_views`] found for the views of a Stelle.
struct Judged {
    /// Per face of the Stelle the images (indices into the decoded images, sorted) whose view
    /// of it is left out.
    out: Vec<Vec<u32>>,
    /// Per image the colour of every face of the Stelle it sees ([`quarter_colours`]); empty
    /// where nothing was judged.
    colours: Vec<Vec<Option<[f64; 3]>>>,
    /// Per face of the Stelle the faces within [`CONSISTENCY_RADIUS`] (indices, sorted).
    around: Vec<Vec<u32>>,
}

impl Judged {
    /// Nothing left out, for a Stelle of `n` faces.
    #[cfg(test)]
    fn none(n: usize) -> Judged {
        Judged {
            out: vec![Vec::new(); n],
            colours: Vec::new(),
            around: Vec::new(),
        }
    }

    /// Whether the view of face `i` (of the Stelle) in image `c` is left out.
    fn left_out(&self, i: usize, c: u32) -> bool {
        self.out[i].binary_search(&c).is_ok()
    }

    /// The brightness of image `c` against image `main` around the faces `replaced` (indices,
    /// sorted) of the Stelle, which `c` takes because `main` may not (`taken`, per face; see
    /// the module docs, step 6): per channel the median of
    /// `ln((c + GAIN_FLOOR) / (main + GAIN_FLOOR))` of their colours ([`quarter_colours`]) over
    /// the faces within [`CONSISTENCY_RADIUS`] of them that `main` takes and where `c`'s view is
    /// not left out (at least [`MIN_GAIN_FACES`]), exponentiated and clamped to
    /// `[1 / max_gain, max_gain]`; 1 without enough such faces.
    fn match_gain(
        &self,
        c: u32,
        main: u32,
        replaced: &[usize],
        taken: &[bool],
        max_gain: f64,
    ) -> [f32; 3] {
        let mut near: Vec<u32> = replaced
            .iter()
            .flat_map(|&i| self.around[i].iter().copied())
            .collect();
        near.sort_unstable();
        near.dedup();
        let (ci, mi) = (c as usize, main as usize);
        let mut logs: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for &j in &near {
            let j = j as usize;
            if self.left_out(j, c) || taken[j] {
                continue;
            }
            let (Some(a), Some(b)) = (self.colours[ci][j], self.colours[mi][j]) else {
                continue;
            };
            for (k, l) in logs.iter_mut().enumerate() {
                l.push(((a[k] + GAIN_FLOOR) / (b[k] + GAIN_FLOOR)).ln());
            }
        }
        if logs[0].len() < MIN_GAIN_FACES {
            return [1.0; 3];
        }
        let lim = max_gain.max(1.0);
        logs.map(|mut l| median(&mut l).exp().clamp(1.0 / lim, lim) as f32)
    }
}

/// The colours of the faces of `region` (sorted) that `image` sees ([`RegionView::faces`]), in
/// the order of `region` (`None` for the others and where a point is not in the image): the
/// linear mean at [`FACE_SAMPLES`], bilinear in the image at 1/[`SHARPNESS_DOWNSCALE`] (each
/// pixel the rounded mean of its block of the full image, as [`decode_jpeg`] makes it) of
/// `camera`, divided by `gain`. Only the part of the small image that holds the points is made.
fn quarter_colours(
    s: &WeldedSurface,
    region: &[u32],
    image: &Decoded,
    camera: &Camera,
    gain: &[f32; 3],
    lin: &[f32; 256],
) -> Vec<Option<[f64; 3]>> {
    let d = SHARPNESS_DOWNSCALE;
    let small = camera.downscaled(d);
    let to_cam = image.view.world_from_cam.inverse();
    // Every face seen: its place in the Stelle and its points in the small image.
    let points: Vec<(usize, [[f64; 2]; 4])> = image
        .view
        .faces
        .iter()
        .filter_map(|&(f, _)| {
            let corners = s.face_corners(f);
            let mut uv = [[0.0f64; 2]; 4];
            for (uv, w) in uv.iter_mut().zip(&FACE_SAMPLES) {
                let p = [0, 1, 2].map(|a| (0..3).map(|k| w[k] * corners[k][a]).sum::<f64>());
                *uv = small.project(&(to_cam * Point3::from(p)).coords)?;
            }
            Some((region.binary_search(&f).expect("a face of the Stelle"), uv))
        })
        .collect();
    let mut out = vec![None; region.len()];
    let (w, h) = (image.rgb.width / d, image.rgb.height / d);
    if points.is_empty() || w == 0 || h == 0 {
        return out;
    }
    // The pixels the bilinear samples read, and one more on every side.
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (_, uv) in &points {
        for p in uv {
            for a in 0..2 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
    }
    let x0 = (lo[0].floor() - 1.0).clamp(0.0, f64::from(w - 1)) as u32;
    let y0 = (lo[1].floor() - 1.0).clamp(0.0, f64::from(h - 1)) as u32;
    let x1 = (hi[0].ceil() + 1.0).clamp(0.0, f64::from(w - 1)) as u32;
    let y1 = (hi[1].ceil() + 1.0).clamp(0.0, f64::from(h - 1)) as u32;
    let part = shrunk(&image.rgb, d, [x0, y0], [x1 - x0 + 1, y1 - y0 + 1]);
    for (i, uv) in points {
        let mut sum = [0.0f64; 3];
        for p in uv {
            let at = [(p[0] - f64::from(x0)) as f32, (p[1] - f64::from(y0)) as f32];
            let c = sample_linear(&part, at, lin);
            for k in 0..3 {
                sum[k] += f64::from(c[k]);
            }
        }
        out[i] = Some(std::array::from_fn(|k| {
            sum[k] / FACE_SAMPLES.len() as f64 / f64::from(gain[k])
        }));
    }
    out
}

/// The part `size` at `origin` of `image` made `d` times smaller, as [`decode_jpeg`] makes the
/// whole: every pixel the rounded mean of its `d × d` block.
fn shrunk(image: &RgbImage, d: u32, origin: [u32; 2], size: [u32; 2]) -> RgbImage {
    let (w, d) = (image.width as usize, d as usize);
    let area = (d * d) as u32;
    let mut data = Vec::with_capacity(size[0] as usize * size[1] as usize * 3);
    for oy in origin[1] as usize..(origin[1] + size[1]) as usize {
        for ox in origin[0] as usize..(origin[0] + size[0]) as usize {
            let mut sum = [0u32; 3];
            for dy in 0..d {
                let row = ((oy * d + dy) * w + ox * d) * 3;
                for px in image.data[row..row + d * 3].as_chunks::<3>().0 {
                    for c in 0..3 {
                        sum[c] += u32::from(px[c]);
                    }
                }
            }
            data.extend(sum.iter().map(|s| ((s + area / 2) / area) as u8));
        }
    }
    RgbImage {
        width: size[0],
        height: size[1],
        data,
    }
}

/// The median of `v` (the mean of the middle two for an even count), by selection; 0 if empty.
/// The same value as [`median`].
fn select_median(v: &mut [f64]) -> f64 {
    let n = v.len();
    if n == 0 {
        return 0.0;
    }
    let (below, &mut upper, _) = v.select_nth_unstable_by(n / 2, f64::total_cmp);
    if n % 2 == 1 {
        return upper;
    }
    let (_, &mut lower, _) = below.select_nth_unstable_by(n / 2 - 1, f64::total_cmp);
    (lower + upper) / 2.0
}

/// The edges where the new texture meets the old one (see the module docs, step 8): every edge
/// of a face of `new` (sorted; their vertices `verts`, sorted, numbered as [`local_mesh`] does)
/// to a face that keeps its texture (of the ring, or of the Stelle: `kept`, with its old texture
/// `kept_old`), if that face has a texture and the two do not meet at a crease
/// ([`SEAM_MIN_COS`]); in face and edge order.
fn frozen_edges(
    s: &WeldedSurface,
    prep: &Prepared,
    new: &[u32],
    verts: &[u32],
    kept: &[u32],
    kept_old: &[Option<OldFace>],
) -> Vec<FrozenEdge> {
    let local = |v: u32| verts.binary_search(&v).expect("a corner of the new faces") as u32;
    let mut out = Vec::new();
    for (j, &f) in new.iter().enumerate() {
        let i = prep.region.binary_search(&f).expect("a face of the Stelle");
        let corners = s.faces()[f as usize];
        let normal = s.face_normal(f);
        for (k, others) in prep.neighbours[i].iter().enumerate() {
            let (a, b) = (corners[k], corners[(k + 1) % 3]);
            for &g in others {
                if new.binary_search(&g).is_ok() {
                    continue;
                }
                let old = match prep.ring.binary_search(&g) {
                    Ok(r) => prep.ring_old[r],
                    Err(_) => kept.binary_search(&g).ok().and_then(|r| kept_old[r]),
                };
                let Some(old) = old else {
                    continue;
                };
                if dot(normal, s.face_normal(g)) < f64::from(SEAM_MIN_COS) {
                    continue;
                }
                let gc = s.faces()[g as usize];
                let Some(e) = (0..3).find(|&e| {
                    let (x, y) = (gc[e], gc[(e + 1) % 3]);
                    (x, y) == (a, b) || (x, y) == (b, a)
                }) else {
                    continue;
                };
                let [first, second] = old.edges[e];
                out.push(FrozenEdge {
                    inside: j as u32,
                    a: local(a),
                    b: local(b),
                    outside: if gc[e] == a {
                        [first, second]
                    } else {
                        [second, first]
                    },
                });
            }
        }
    }
    out
}

/// The pages of `tex` (the faces `new` baked: face `j` is `new[j]`) as pages of a patch: corner
/// `k` of every triangle at the place of its face's corner `k` in `s`, the face of every
/// triangle, the page's image as JPEG.
fn patch_pages(
    s: &WeldedSurface,
    new: &[u32],
    tex: &TexturedMesh,
    jpegs: &PageJpegs,
) -> Vec<PatchPage> {
    let mut faces: Vec<Vec<u32>> = tex
        .pages
        .iter()
        .map(|p| vec![u32::MAX; p.indices.len()])
        .collect();
    for (j, &(p, t)) in tex.face_ref.iter().enumerate() {
        faces[p as usize][t as usize] = new[j];
    }
    tex.pages
        .iter()
        .zip(faces)
        .enumerate()
        .map(|(p, (page, faces))| {
            let mut positions = vec![[0.0f64; 3]; page.positions.len()];
            for (tri, &f) in page.indices.iter().zip(&faces) {
                for (k, &c) in tri.iter().enumerate() {
                    positions[c as usize] = s.positions()[s.faces()[f as usize][k] as usize];
                }
            }
            PatchPage {
                positions,
                uvs: page.uvs.clone(),
                indices: page.indices.clone(),
                faces,
                jpeg: Arc::from(jpegs.page(p)),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::{sample_linear, srgb_value};
    use crate::surface::atlas::REGION_PAGE_MIN;
    use crate::surface::edit::{apply_patches, write_patches, PATCHES_FILE};
    use crate::surface::testutil::{
        paged, painted_mesh, scene, testscan_config, testscan_source, testscan_source_scaled,
        testscan_surface, walk_source,
    };
    use crate::surface::views::image_views;
    use driftless_export::mesh::Mesh as PageMesh;
    use driftless_io::images::{
        decode_jpeg, for_each_image, for_each_image_where, index_images, RgbImage, Wanted,
    };
    use driftless_io::testdir::{SharedDir, TestDir};
    use driftless_io::testscan::{scan_with_images, shared, IMAGES, WALL_X};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::collections::BTreeMap;
    use std::fs;
    use std::sync::OnceLock;
    use std::time::Instant;

    /// The scan folder of `scan_with_images`, its surface (pages and welded) and the scan's own
    /// source (the device at the origin, its camera looking along +x).
    struct Room {
        dir: SharedDir,
        mesh: PageMesh,
        surface: WeldedSurface,
        scan: TextureSource,
    }

    fn room() -> &'static Room {
        static ROOM: OnceLock<Room> = OnceLock::new();
        ROOM.get_or_init(|| {
            let dir = shared("map-retexture");
            scan_with_images(&dir);
            let (mesh, scan) = testscan_surface(&dir);
            let surface = WeldedSurface::from_mesh(&mesh);
            Room {
                dir,
                mesh,
                surface,
                scan,
            }
        })
    }

    /// The scan's images with the device standing at `at`, turned to look along −x (its camera
    /// looks along the device's +x, 10° down).
    fn looking_back(at: [f64; 3]) -> TextureSource {
        let device = Isometry3::from_parts(
            Translation3::from(Vector3::from(at)),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI),
        );
        testscan_source(&room().dir, device)
    }

    /// A Stelle on a wall: the faces of the −x wall (x = −4, facing +x) whose centroid lies
    /// within `r` of (−4, 1, 0.25).
    fn wall_spot(s: &WeldedSurface, r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                let d = [c[0] + 4.0, c[1] - 1.0, c[2] - 0.25];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() < r && s.face_normal(f)[0] > 0.9
            })
            .collect()
    }

    fn of_source(views: &[RegionView], source: u32) -> Vec<&RegionView> {
        views.iter().filter(|v| v.source == source).collect()
    }

    /// `mesh` without the triangles whose centroid `cut` takes.
    fn without(mesh: &PageMesh, cut: impl Fn([f64; 3]) -> bool) -> PageMesh {
        let mut out = mesh.clone();
        for page in &mut out.pages {
            let positions = page.positions.clone();
            page.indices.retain(|t| {
                let c = t.map(|i| positions[i as usize]);
                !cut([0, 1, 2].map(|k| (c[0][k] + c[1][k] + c[2][k]) / 3.0))
            });
        }
        out
    }

    /// The index of the images of every source.
    fn index_of(sources: &[TextureSource]) -> Vec<Arc<ImageIndex>> {
        sources
            .iter()
            .map(|src| {
                let index =
                    index_images(&src.images.bags, &src.images.topics, &Progress::new()).unwrap();
                Arc::new(index)
            })
            .collect()
    }

    /// Every view of the Stelle `spot` in `sources` (no ring), each candidate with a depth
    /// buffer.
    fn views(s: &WeldedSurface, spot: &[u32], sources: &[TextureSource]) -> Vec<RegionView> {
        views_with(s, spot, &[], sources, &testscan_config())
    }

    fn views_with(
        s: &WeldedSurface,
        spot: &[u32],
        around: &[u32],
        sources: &[TextureSource],
        cfg: &SurfaceConfig,
    ) -> Vec<RegionView> {
        let bvh = FaceBvh::build(s);
        let indices = index_of(sources);
        region_views(
            s,
            &bvh,
            spot,
            around,
            sources,
            &indices,
            cfg,
            Sight::ALL,
            &Progress::new(),
        )
        .unwrap()
        .0
    }

    /// [`retexture_region`] with the tree of `s` and the indices of `sources` made here.
    fn retexture_here(
        s: &WeldedSurface,
        current: &PageMesh,
        faces: &[u32],
        sources: &[TextureSource],
        variant: u32,
        cfg: &SurfaceConfig,
        progress: &Progress,
    ) -> Result<RetextureDone, EditError> {
        let bvh = FaceBvh::build(s);
        let indices = index_of(sources);
        retexture_region(
            s, &bvh, current, faces, sources, &indices, variant, cfg, progress,
        )
    }

    #[test]
    fn region_views_see_the_region() {
        let room = room();
        let s = &room.surface;
        let spot = wall_spot(s, 0.6);
        assert!(spot.len() >= 4, "{} faces", spot.len());
        // 0: the scan as it was, at the origin, looking at the +x wall: away from the Stelle.
        // 1: 5 m in front of the Stelle, looking at it.
        // 2: behind the +x wall, looking at the Stelle through it.
        let sources = [
            room.scan.clone(),
            looking_back([1.0, 1.0, 0.25]),
            looking_back([6.6, 1.0, 0.25]),
        ];
        // By the headers and poses alone, the images in front of the Stelle and those behind
        // the wall may see it; the depth buffer tells them apart.
        let cfg = testscan_config();
        let shape = Shape::new(s, &spot);
        let maybe = candidates(
            &shape,
            &sources,
            &index_of(&sources),
            &cfg.color,
            &Progress::new(),
        )
        .unwrap();
        let by_source = |k: u32| maybe.iter().filter(|c| c.source == k).count();
        assert_eq!(
            [by_source(0), by_source(1), by_source(2)],
            [0, IMAGES, IMAGES]
        );
        let found = views(s, &spot, &sources);
        println!("{} candidates, {} views", maybe.len(), found.len());
        for v in &found {
            println!(
                "source {} frame {} stamp {} coverage {:.3} quality {:.3} faces {}",
                v.source,
                v.frame,
                v.stamp,
                v.coverage,
                v.quality,
                v.faces.len()
            );
        }
        assert!(of_source(&found, 0).is_empty(), "looks away");
        assert!(of_source(&found, 2).is_empty(), "behind a wall");
        let front = of_source(&found, 1);
        assert_eq!(front.len(), IMAGES);
        let camera = sources[1].poses[0].p
            + sources[1].poses[0].q * sources[1].cams[0].cam_in_body.translation.vector;
        for (k, v) in front.iter().enumerate() {
            assert_eq!(v.frame, k as u32);
            let at = v.world_from_cam.translation.vector;
            assert!((at - camera).norm() < 1e-12, "{at} vs. {camera}");
            assert_eq!(v.stamp, 1_000_300_000_000 + 200_000_000 * k as u64);
            assert_eq!(v.camera, 0);
            assert!(v.coverage >= 0.9, "{}", v.coverage);
            assert!(v.quality > 0.0 && v.quality <= 1.0, "{}", v.quality);
            assert!(v.faces.windows(2).all(|w| w[0].0 < w[1].0));
            assert!(v
                .faces
                .iter()
                .all(|&(f, score)| spot.binary_search(&f).is_ok() && score > 0.0));
        }
        // The same images, the same Stelle, the same views.
        assert_eq!(views(s, &spot, &sources), found);

        // Without the +x wall the camera behind it sees the Stelle: the wall hid it, not the
        // field of view.
        let open = WeldedSurface::from_mesh(&without(&room.mesh, |c| c[0] > 5.9));
        let spot = wall_spot(&open, 0.6);
        let found = views(&open, &spot, &sources);
        assert_eq!(of_source(&found, 2).len(), IMAGES);
        assert_eq!(of_source(&found, 1).len(), IMAGES);
        assert!(of_source(&found, 0).is_empty());
    }

    #[test]
    fn ring_changes_no_view_of_the_stelle() {
        // Looking at a ring around the Stelle too: the same images, the same views of the
        // Stelle; the ring's faces they see listed, sorted, on the same wall.
        let room = room();
        let s = &room.surface;
        let spot = front_spot(s, 0.6);
        let around = ring_around(s, &spot);
        assert!(
            around.len() > spot.len(),
            "{} around {}",
            around.len(),
            spot.len()
        );
        let sources = [room.scan.clone(), nearer(2.0)];
        let cfg = testscan_config();
        let plain = views_with(s, &spot, &[], &sources, &cfg);
        let ringed = views_with(s, &spot, &around, &sources, &cfg);
        assert_eq!(plain.len(), ringed.len());
        for (a, b) in plain.iter().zip(&ringed) {
            assert!(a.ring.is_empty());
            assert_eq!(
                RegionView {
                    ring: Vec::new(),
                    ..b.clone()
                },
                *a
            );
            assert!(!b.ring.is_empty());
            assert!(b.ring.windows(2).all(|w| w[0] < w[1]));
            assert!(b.ring.iter().all(|f| around.binary_search(f).is_ok()));
        }
    }

    #[test]
    fn nearer_views_rank_first() {
        // The Stelle straight on from 5 m and from 2 m: the nearer images see every face in
        // more pixels, score higher on each and come first.
        let s = &room().surface;
        let spot = wall_spot(s, 0.6);
        let sources = [
            looking_back([1.0, 1.0, 0.25]),
            looking_back([-2.0, 1.0, 0.25]),
        ];
        let found = views(s, &spot, &sources);
        assert_eq!(found.len(), 2 * IMAGES);
        assert!(found[..IMAGES].iter().all(|v| v.source == 1), "{found:?}");
        let (near, far) = (&found[0], &found[IMAGES]);
        assert_eq!(near.quality, 1.0);
        assert!(far.quality < 0.6, "{}", far.quality);
        let mut compared = 0;
        for &(f, score) in &near.faces {
            if let Some(&(_, other)) = far.faces.iter().find(|x| x.0 == f) {
                assert!(score > 2.0 * other, "face {f}: {score} vs {other}");
                compared += 1;
            }
        }
        assert!(
            compared >= spot.len() * 9 / 10,
            "{compared} of {}",
            spot.len()
        );
    }

    /// The scan's image at full size, as the bag holds it (all its images are the same).
    fn scan_image() -> RgbImage {
        let src = &room().scan;
        let mut first = None;
        for_each_image(&src.images.bags, &src.images.topics, 1, |r| {
            first = Some(r.unwrap().rgb);
            false
        })
        .unwrap();
        first.unwrap()
    }

    #[test]
    fn decode_only_k() {
        // Six places 3 to 8 m in front of the Stelle, five images each: 30 candidates.
        let s = &room().surface;
        let spot = wall_spot(s, 0.6);
        let sources: Vec<TextureSource> = (0..6)
            .map(|i| looking_back([-1.0 + f64::from(i), 1.0, 0.25]))
            .collect();
        let found = views(s, &spot, &sources);
        assert_eq!(found.len(), 6 * IMAGES);
        // Best first: the nearest place first.
        assert!(found.windows(2).all(|w| w[0].source <= w[1].source));
        assert_eq!(DECODED_VIEWS, 24);
        let progress = Progress::new();
        let indices = index_of(&sources);
        let decoded = decode_views(&found, &sources, &indices, DECODED_VIEWS, &progress).unwrap();
        println!(
            "{} candidates, {} decoded ({:?})",
            found.len(),
            decoded.len(),
            progress.snapshot()
        );
        assert_eq!(decoded.len(), DECODED_VIEWS);
        assert_eq!(progress.snapshot().done, DECODED_VIEWS as u64);
        let full = scan_image();
        assert_eq!((full.width, full.height), (160, 120));
        for (k, (i, image)) in decoded.iter().enumerate() {
            assert_eq!(*i, k, "the best 24, in their order");
            assert_eq!(image, &full);
        }
        // The same again.
        let again = views(s, &spot, &sources);
        assert_eq!(again, found);
        let decoded_again =
            decode_views(&again, &sources, &indices, DECODED_VIEWS, &Progress::new()).unwrap();
        assert_eq!(decoded_again, decoded);
        // Fewer views than asked for: all of them; none asked: none.
        assert_eq!(
            decode_views(
                &found[..3],
                &sources,
                &indices,
                DECODED_VIEWS,
                &Progress::new()
            )
            .unwrap()
            .len(),
            3
        );
        assert!(
            decode_views(&found, &sources, &indices, 0, &Progress::new())
                .unwrap()
                .is_empty()
        );
        // A cancel stops it.
        let cancelled = Progress::new();
        cancelled.cancel();
        assert!(matches!(
            decode_views(&found, &sources, &indices, DECODED_VIEWS, &cancelled),
            Err(EditError::Cancelled)
        ));
    }

    #[test]
    fn whole_views_rank_first() {
        // Within 28° of the axis only: from 1.5 m the camera sees most of the Stelle (about
        // four fifths) but not its top, from 5 m all of it. The near images see their part sharper (the better
        // quality), but the far ones, which can texture the whole Stelle, come first.
        let s = &room().surface;
        let spot = wall_spot(s, 0.6);
        let sources = [
            looking_back([1.0, 1.0, 0.25]),
            looking_back([-2.5, 1.0, 0.25]),
        ];
        let mut cfg = testscan_config();
        cfg.color.rim_theta = 28f64.to_radians();
        let found = views_with(s, &spot, &[], &sources, &cfg);
        assert_eq!(found.len(), 2 * IMAGES);
        let (far, near) = (&found[0], &found[IMAGES]);
        println!(
            "far: coverage {} quality {}; near: coverage {} quality {}",
            far.coverage, far.quality, near.coverage, near.quality
        );
        assert!(found[..IMAGES].iter().all(|v| v.source == 0), "{found:?}");
        assert!(far.coverage >= FULL_COVERAGE, "{}", far.coverage);
        assert!(
            near.coverage > 0.7 && near.coverage < 0.85,
            "{}",
            near.coverage
        );
        assert!(
            near.quality > far.quality,
            "{} vs. {}",
            near.quality,
            far.quality
        );
    }

    /// The walk through the painted room (`scene::walk_scene`, a floor-to-ceiling pillar at
    /// x −3.5 … −2.5, y −3 … −1) as pages of 40 faces on 3 pages, welded; and the walk's images
    /// of both cameras.
    fn walk() -> &'static (WeldedSurface, TextureSource) {
        static WALK: OnceLock<(WeldedSurface, TextureSource)> = OnceLock::new();
        WALK.get_or_init(|| {
            let mesh = painted_mesh(&scene::walk_scene(), 0.1);
            let positions: Vec<[f64; 3]> =
                mesh.positions.iter().map(|p| p.map(f64::from)).collect();
            let pages = paged(&positions, &mesh.indices, 40, 3);
            let src = walk_source("retexture-walk", &[0, 1], 1..=11, 1.0);
            (WeldedSurface::from_mesh(&pages), src)
        })
    }

    #[test]
    fn same_views_as_the_whole_surface() {
        // Stellen on the −y wall behind the pillar (as the right camera sees it from the walk),
        // on the floor, and on the +y wall. Per image, the faces seen and their scores are
        // those of the same look at the whole surface: every face in the depth buffer, the
        // normals smoothed over all faces.
        let (s, src) = walk();
        let cfg = SurfaceConfig::default();
        let ccfg = &cfg.color;
        let all: Vec<u32> = (0..s.face_count() as u32).collect();
        let whole = local_mesh(s, &all);
        let geometry = Faces::new(&whole);
        let lin = linear_table();
        let mut headers = Vec::new();
        for_each_image_where(
            &src.images.bags,
            &src.images.topics,
            1,
            |h| {
                headers.push(h.clone());
                Wanted::No
            },
            |_| true,
        )
        .unwrap();
        let near = |centre: [f64; 3], r: f64, normal: [f64; 3]| -> Vec<u32> {
            (0..s.face_count() as u32)
                .filter(|&f| {
                    let c = s.face_centroid(f);
                    norm(sub(c, centre)) < r && dot(s.face_normal(f), normal) > 0.9
                })
                .collect()
        };
        let stellen = [
            near([-3.5, -4.0, 0.0], 1.6, [0.0, 1.0, 0.0]),
            near([0.5, 1.5, -1.2], 0.8, [0.0, 0.0, 1.0]),
            near([1.0, 5.0, 0.3], 0.7, [0.0, -1.0, 0.0]),
        ];
        let bvh = FaceBvh::build(s);
        let mut partly = 0;
        for stelle in &stellen {
            assert!(stelle.len() > 20, "{}", stelle.len());
            let found = views(s, stelle, std::slice::from_ref(src));
            let shape = Shape::new(s, stelle);
            let mut want: Vec<(u32, Vec<(u32, f32)>)> = Vec::new();
            let mut smaller = 0;
            for h in &headers {
                let camera = src
                    .cams
                    .iter()
                    .position(|c| c.camera.name == h.camera)
                    .unwrap();
                let t = h.stamp + ccfg.time_offset;
                let body = interpolate_pose(&src.poses, t).unwrap();
                assert!(angular_rate(&src.poses, t) <= ccfg.max_angular_rate);
                let world_from_cam = body * src.cams[camera].cam_in_body;
                let cam = &src.cams[camera].camera;
                let shot = Shot {
                    geometry: &cam.downscaled(ccfg.downscale),
                    full: cam,
                    world_from_cam,
                    rank: 0,
                    sharpness_norm: 1.0,
                    rgb: None,
                    min_texel: cfg.min_texel,
                };
                let seen: Vec<(u32, f32)> = image_views(
                    &whole,
                    &geometry,
                    &all,
                    stelle,
                    &shot,
                    ccfg,
                    |_| false,
                    &lin,
                )
                .into_iter()
                .map(|(f, v, _)| (f, v.rank_score()))
                .collect();
                if !seen.is_empty() {
                    want.push((h.index as u32, seen));
                    smaller += usize::from(
                        in_front(s, &bvh, &shape, &world_from_cam, cam, ccfg).len()
                            < s.face_count() / 2,
                    );
                }
            }
            let mut got: Vec<(u32, Vec<(u32, f32)>)> =
                found.iter().map(|v| (v.frame, v.faces.clone())).collect();
            got.sort_by_key(|g| g.0);
            let bits = |v: &[(u32, Vec<(u32, f32)>)]| -> Vec<(u32, Vec<(u32, u32)>)> {
                v.iter()
                    .map(|(i, f)| (*i, f.iter().map(|(f, x)| (*f, x.to_bits())).collect()))
                    .collect()
            };
            assert_eq!(bits(&got), bits(&want));
            assert!(!want.is_empty());
            partly += found
                .iter()
                .filter(|v| v.coverage < 1.0 && v.coverage > 0.0)
                .count();
            println!(
                "Stelle of {} faces: {} of {} images see it, {} of them from a depth buffer of \
                 under half the faces",
                stelle.len(),
                want.len(),
                headers.len(),
                smaller
            );
            assert!(smaller > 0);
        }
        assert!(partly > 0, "some image sees a Stelle in part");
    }

    #[test]
    fn region_views_of_nothing() {
        let s = &room().surface;
        assert!(views(s, &[], &[looking_back([1.0, 1.0, 0.25])]).is_empty());
        assert!(views(s, &wall_spot(s, 0.6), &[]).is_empty());
    }

    // ---- Second part: choose, bake, level ------------------------------------------------

    /// A Stelle in the middle of the scan's view: the faces of the +x wall (x = 6, facing −x)
    /// whose centroid lies within `r` of (6, 1, 0); the scan's camera looks along +x, 10° down,
    /// and saw the wall plain [`WALL_X`].
    fn front_spot(s: &WeldedSurface, r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                norm(sub(c, [6.0, 1.0, 0.0])) < r && s.face_normal(f)[0] < -0.9
            })
            .collect()
    }

    /// The scan's images with every linear value times `gain` (another exposure of the same
    /// moments), taken where the scan's were: 1.2 and 0.8.
    fn exposed(gain: f64) -> &'static TextureSource {
        static BRIGHT: OnceLock<(SharedDir, TextureSource)> = OnceLock::new();
        static DARK: OnceLock<(SharedDir, TextureSource)> = OnceLock::new();
        let (cell, name) = match gain {
            1.2 => (&BRIGHT, "map-retexture-hell"),
            0.8 => (&DARK, "map-retexture-dunkel"),
            _ => panic!("no source for gain {gain}"),
        };
        &cell
            .get_or_init(|| {
                let out = shared(name);
                let src = testscan_source_scaled(&room().dir, &out, Isometry3::identity(), gain);
                (out, src)
            })
            .1
    }

    /// The scan's images as if the device had stood `dx` metres nearer the +x wall (only the
    /// geometry counts where they are used).
    fn nearer(dx: f64) -> TextureSource {
        testscan_source(&room().dir, Isometry3::translation(dx, 0.0, 0.0))
    }

    fn retextured(spot: &[u32], sources: &[TextureSource], variant: u32) -> RetextureDone {
        let room = room();
        retexture_here(
            &room.surface,
            &room.mesh,
            spot,
            sources,
            variant,
            &testscan_config(),
            &Progress::new(),
        )
        .unwrap()
    }

    /// The colour (sRGB) of `image` at `uv` (bilinear, `v` down).
    fn colour_at(image: &RgbImage, uv: [f32; 2]) -> [f64; 3] {
        let px = [
            uv[0] * image.width as f32 - 0.5,
            uv[1] * image.height as f32 - 0.5,
        ];
        sample_linear(image, px, &linear_table()).map(srgb_value)
    }

    /// Whether a triangle's texture coordinates are all one point: an unseen (gray) face.
    fn gray(uvs: [[f32; 2]; 3]) -> bool {
        uvs[0] == uvs[1] && uvs[1] == uvs[2]
    }

    /// The mean difference (sRGB levels, over the channels and the samples) between both sides
    /// of the border of the Stelle `patch` re-textured on the computed surface of [`room`]: every
    /// edge between a face on the patch's pages and a textured face that stayed on its page,
    /// sampled on the edge at a quarter, half and three quarters of its length in both textures;
    /// and the number of such edges.
    fn border_step(patch: &SurfacePatch) -> (f64, usize) {
        let room = room();
        let (s, mesh) = (&room.surface, &room.mesh);
        let new_pages: Vec<RgbImage> = patch
            .pages
            .iter()
            .map(|p| decode_jpeg(&p.jpeg, 1).unwrap())
            .collect();
        let old_pages: Vec<RgbImage> = mesh
            .pages
            .iter()
            .map(|p| decode_jpeg(&p.image.as_ref().unwrap().bytes, 1).unwrap())
            .collect();
        let mut by_edge: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
        for (f, t) in s.faces().iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                by_edge
                    .entry((a.min(b), a.max(b)))
                    .or_default()
                    .push(f as u32);
            }
        }
        let (mut sum, mut samples, mut edges) = (0.0f64, 0usize, 0usize);
        for (p, page) in patch.pages.iter().enumerate() {
            for (tri, &f) in page.indices.iter().zip(&page.faces) {
                let corners = s.faces()[f as usize];
                for k in 0..3 {
                    let (a, b) = (corners[k], corners[(k + 1) % 3]);
                    for &g in &by_edge[&(a.min(b), a.max(b))] {
                        if g == f || patch.removed_faces.binary_search(&g).is_ok() {
                            continue;
                        }
                        let (gp, gt) = s.face_origin(g);
                        let gpage = &mesh.pages[gp as usize];
                        let gtri = gpage.indices[gt as usize];
                        if gray(gtri.map(|c| gpage.uvs[c as usize])) {
                            continue;
                        }
                        let gc = s.faces()[g as usize];
                        let at = |v: u32| gtri[gc.iter().position(|&x| x == v).unwrap()] as usize;
                        edges += 1;
                        for t in [0.25f32, 0.5, 0.75] {
                            let mix = |p: [f32; 2], q: [f32; 2]| {
                                [p[0] * (1.0 - t) + q[0] * t, p[1] * (1.0 - t) + q[1] * t]
                            };
                            let inside = colour_at(
                                &new_pages[p],
                                mix(
                                    page.uvs[tri[k] as usize],
                                    page.uvs[tri[(k + 1) % 3] as usize],
                                ),
                            );
                            let outside = colour_at(
                                &old_pages[gp as usize],
                                mix(gpage.uvs[at(a)], gpage.uvs[at(b)]),
                            );
                            for c in 0..3 {
                                sum += (inside[c] - outside[c]).abs();
                                samples += 1;
                            }
                        }
                    }
                }
            }
        }
        (sum / samples.max(1) as f64, edges)
    }

    #[test]
    fn new_page_for_region() {
        let room = room();
        let s = &room.surface;
        let spot = front_spot(s, 0.8);
        assert!(spot.len() >= 10, "{} faces", spot.len());
        let done = retextured(&spot, std::slice::from_ref(&room.scan), 0);
        let patch = &done.patch;
        // Exactly the faces of the Stelle leave their pages, and the new pages hold them.
        assert!(patch.moved.is_empty());
        assert_eq!(patch.removed_faces, spot);
        assert!(!patch.pages.is_empty());
        let mut held: Vec<u32> = patch.pages.iter().flat_map(|p| p.faces.clone()).collect();
        held.sort_unstable();
        assert_eq!(held, spot);
        for page in &patch.pages {
            assert_eq!(page.uvs.len(), page.positions.len());
            // Corner k of every triangle is a copy of its face's corner k, where the surface has it.
            for (tri, &f) in page.indices.iter().zip(&page.faces) {
                for (k, &c) in tri.iter().enumerate() {
                    let v = s.faces()[f as usize][k];
                    assert_eq!(
                        page.positions[c as usize].map(f64::to_bits),
                        s.positions()[v as usize].map(f64::to_bits)
                    );
                }
            }
            assert!(page.uvs.iter().flatten().all(|c| (0.0..=1.0).contains(c)));
            // A page of its own, the smallest for this little Stelle.
            let image = decode_jpeg(&page.jpeg, 1).unwrap();
            assert_eq!(
                (image.width, image.height),
                (REGION_PAGE_MIN, REGION_PAGE_MIN)
            );
            // It shows the wall the scan saw there.
            for tri in &page.indices {
                let uv =
                    [0, 1].map(|a| tri.iter().map(|&c| page.uvs[c as usize][a]).sum::<f32>() / 3.0);
                let got = colour_at(&image, uv);
                for c in 0..3 {
                    assert!(
                        (got[c] - f64::from(WALL_X[c])).abs() <= 8.0,
                        "{got:?} vs {WALL_X:?}"
                    );
                }
            }
        }
        assert_eq!(done.variant_of, (1, IMAGES as u32));
        // Applied: every face once, the old pages keep their images.
        let out = apply_patches(&room.mesh, s, std::slice::from_ref(patch));
        assert_eq!(out.pages.len(), room.mesh.pages.len() + patch.pages.len());
        assert_eq!(out.triangles(), room.mesh.triangles());
        for (a, b) in room.mesh.pages.iter().zip(&out.pages) {
            assert_eq!(a.image, b.image);
        }
    }

    /// A Stelle across the checkerboard of the +y wall (y = 5, facing −y), near the edge of the
    /// scan's view: the faces whose centroid lies within `r` of (3, 5, 0).
    fn side_spot(s: &WeldedSurface, r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                norm(sub(c, [3.0, 5.0, 0.0])) < r && s.face_normal(f)[1] < -0.9
            })
            .collect()
    }

    #[test]
    fn border_matches_outside() {
        // The surface was textured from the scan's images; the Stelle gets the same images with
        // another exposure (20 % brighter, or darker, in linear light). On the plain wall in
        // front, and across the checkerboard on the side, where the texture's own edges, sampled
        // anew, leave about 5 levels even with the same exposure.
        let room = room();
        let s = &room.surface;
        let cfg = testscan_config();
        for (name, spot, least) in [
            ("front", front_spot(s, 0.8), 8),
            ("side", side_spot(s, 0.8), 4),
        ] {
            for gain in [1.2, 0.8] {
                let src = std::slice::from_ref(exposed(gain));
                let raw = retexture(
                    s,
                    &FaceBvh::build(s),
                    &room.mesh,
                    &spot,
                    src,
                    &index_of(src),
                    0,
                    &cfg,
                    &Progress::new(),
                    Matching::Raw,
                    Sight::BEST,
                    Judging::Consistent,
                )
                .unwrap();
                let clock = Instant::now();
                let done =
                    retexture_here(s, &room.mesh, &spot, src, 0, &cfg, &Progress::new()).unwrap();
                let took = clock.elapsed();
                assert_eq!(raw.patch.removed_faces, done.patch.removed_faces);
                let (before, edges) = border_step(&raw.patch);
                let (after, again) = border_step(&done.patch);
                let bag = fs::metadata(&src[0].images.bags[0]).unwrap().len();
                println!(
                    "{name}, exposure × {gain}: {} of {} faces new, border step {before:.2} → \
                     {after:.2} sRGB levels over {edges} edges; {took:?}; the bag ({bag} bytes) \
                     read whole once (headers), once up to the last image decoded",
                    done.patch.removed_faces.len(),
                    spot.len()
                );
                assert_eq!(edges, again);
                assert!(edges >= least, "{name}: {edges} edges");
                // The other exposure shows at the border …
                assert!(before > 8.0, "{name}: {before}");
                // … and levelling takes it away.
                assert!(after <= before, "{name}: {before} → {after}");
                assert!(after <= 6.0, "{name}: {after}");
            }
        }
    }

    #[test]
    fn variants_cycle() {
        let room = room();
        let s = &room.surface;
        let spot = front_spot(s, 0.8);
        // Three places, five images each: the scan's, and 2 m and 3.5 m nearer the wall.
        let sources = [room.scan.clone(), nearer(2.0), nearer(3.5)];
        let cfg = testscan_config();
        let ranked = variants(
            s,
            &FaceBvh::build(s),
            &room.mesh,
            &spot,
            &sources,
            &index_of(&sources),
            &cfg,
            &Progress::new(),
        )
        .unwrap();
        let n = ranked.len();
        println!("{n} variants: {ranked:?}");
        assert_eq!(n, 3 * IMAGES);
        let mut distinct = ranked.clone();
        distinct.sort_unstable();
        distinct.dedup();
        assert_eq!(distinct.len(), n, "every image once");
        // The images that see the whole Stelle first: from 3.5 m nearer the camera sees only
        // a part of it.
        assert!(ranked[..2 * IMAGES].iter().all(|r| r.0 != 2), "{ranked:?}");
        assert!(ranked[2 * IMAGES..].iter().all(|r| r.0 == 2), "{ranked:?}");
        let n = n as u32;
        let patch = |k: u32| {
            let done = retextured(&spot, &sources, k);
            assert_eq!(done.variant_of, (k % n + 1, n), "variant {k}");
            done.patch
        };
        let first = patch(0);
        // Another place, another texture; after the last variant the first again.
        let second = patch(IMAGES as u32);
        let third = patch(2 * IMAGES as u32);
        assert_ne!(second, first);
        assert_ne!(third, first);
        assert_ne!(third, second);
        assert_eq!(patch(n), first);
        assert_eq!(patch(n + IMAGES as u32), second);
    }

    #[test]
    fn deterministic() {
        let room = room();
        let spot = front_spot(&room.surface, 0.8);
        let sources = [exposed(1.2).clone(), nearer(2.0)];
        let dirs = [0, 1].map(|i| {
            TestDir::new(format!(
                "driftless-edit-retexture-{}-{i}",
                std::process::id()
            ))
        });
        for dir in &dirs {
            write_patches(dir, &[retextured(&spot, &sources, 3).patch]).unwrap();
        }
        let [a, b] = dirs.map(|d| fs::read(d.join(PATCHES_FILE)).unwrap());
        assert!(a.len() > 1000, "{}", a.len());
        assert!(a == b, "the same Stelle, the same bytes");
    }

    #[test]
    fn no_image_keeps_the_texture() {
        let room = room();
        let s = &room.surface;
        let cfg = testscan_config();
        let run = |spot: &[u32], sources: &[TextureSource]| {
            retexture_here(s, &room.mesh, spot, sources, 0, &cfg, &Progress::new())
        };
        // The −x wall lies behind the scan's camera.
        let behind = wall_spot(s, 0.6);
        let r = run(&behind, std::slice::from_ref(&room.scan));
        assert!(matches!(r, Err(EditError::NoImages)), "{r:?}");
        let r = run(&front_spot(s, 0.8), &[]);
        assert!(matches!(r, Err(EditError::NoImages)), "{r:?}");
        let r = run(&[], std::slice::from_ref(&room.scan));
        assert!(matches!(r, Err(EditError::NoFaces)), "{r:?}");
        // A cancel stops it.
        let cancelled = Progress::new();
        cancelled.cancel();
        let r = retexture_here(
            s,
            &room.mesh,
            &front_spot(s, 0.8),
            std::slice::from_ref(&room.scan),
            0,
            &cfg,
            &cancelled,
        );
        assert!(matches!(r, Err(EditError::Cancelled)), "{r:?}");
    }

    #[test]
    fn gray_surroundings_keep_the_texture() {
        // The −x wall was behind the scan's camera: gray, all around the Stelle. Images that see
        // it have nothing to match their brightness to.
        let room = room();
        let s = &room.surface;
        let behind = wall_spot(s, 0.6);
        for &f in &behind {
            let (p, t) = s.face_origin(f);
            let page = &room.mesh.pages[p as usize];
            assert!(gray(page.indices[t as usize].map(|c| page.uvs[c as usize])));
        }
        let r = retexture_here(
            s,
            &room.mesh,
            &behind,
            &[looking_back([1.0, 1.0, 0.25])],
            0,
            &testscan_config(),
            &Progress::new(),
        );
        assert!(matches!(r, Err(EditError::NoReference)), "{r:?}");
    }

    /// The rod of [`rod_wall`]: its middle (x, m), half its width, its colour. It stands 0.5 m
    /// in front of the +y wall (y 4.46 … 4.54), from z −1.1 to 1.7.
    const ROD_X: f64 = 0.025;
    const ROD_HALF: f64 = 0.04;
    const ROD: [u8; 3] = [40, 35, 30];
    /// Where the camera of [`rod_wall`] stands (x, m), 1.5 m before the wall, one image a second.
    const ROD_PLACES: [f64; 5] = [0.0, -0.5, 0.5, -1.0, 1.0];
    /// The colour of the +y wall of the painted room (`scene::WALLS[3]`).
    const WALL_Y: [u8; 3] = [60, 200, 200];

    /// The +y wall of the painted room (y = 5, facing −y; x −2.5 … 2.5, z −1.2 … 1.8) as squares
    /// of 5 cm, textured as the surface step does it (1 cm texels, pages of 1024²) from images
    /// of the bare room, welded; and the images of the same moments with a thin rod in front of
    /// the wall, which the surface does not have ([`ROD_X`]): the camera `left` looking at the
    /// wall from [`ROD_PLACES`], at 1.5 m. From each place the rod hides another stripe of the
    /// wall (12 cm wide, 25 cm and more apart).
    fn rod_wall() -> &'static (PageMesh, WeldedSurface, TextureSource) {
        use crate::surface::testutil::{grid, Steps};
        use crate::surface::{texture_mesh, SurfaceStats};
        static WALL: OnceLock<(PageMesh, WeldedSurface, TextureSource)> = OnceLock::new();
        WALL.get_or_init(|| {
            let bare = scene::Painted::new(vec![]);
            let rod = scene::Painted::new(vec![(
                scene::aabb(
                    [ROD_X - ROD_HALF, 4.46, -1.1],
                    [ROD_X + ROD_HALF, 4.54, 1.7],
                ),
                ROD,
            )]);
            let cam = scene::cameras()[0].clone();
            let cam_y = cam.cam_in_body.translation.vector.y;
            let body = move |x: f64| Isometry3::translation(x, 3.5 - cam_y, 0.0);
            // Each place from half a second before its image on.
            let traj = Steps(
                ROD_PLACES
                    .iter()
                    .enumerate()
                    .map(|(i, &x)| {
                        let from = if i == 0 {
                            0.0
                        } else {
                            scene::T0 + 0.5 + i as f64
                        };
                        (from, body(x))
                    })
                    .collect(),
            );
            let shots = || -> Vec<scene::Shot> {
                (0..ROD_PLACES.len())
                    .map(|i| scene::shot(0, scene::T0 + 1.0 + i as f64))
                    .collect()
            };
            let cams = std::slice::from_ref(&cam);
            let source = |name: &str, painted: &scene::Painted| TextureSource {
                images: scene::write_bag(name, painted, &traj, cams, &shots()),
                poses: scene::poses(&traj, 6.0),
                cams: vec![cam.clone()],
            };
            let plain = source("retexture-wall-bare", &bare);
            let with_rod = source("retexture-wall-rod", &rod);
            let (positions, indices) = grid([100, 60], |i, j| {
                [-2.5 + 0.05 * i as f64, 5.0, -1.2 + 0.05 * j as f64]
            });
            let mesh = Mesh {
                positions: positions.iter().map(|p| p.map(|c| c as f32)).collect(),
                indices,
            };
            let cfg = walk_config();
            let mut stats = SurfaceStats::default();
            let tex = texture_mesh(
                &mesh,
                std::slice::from_ref(&plain),
                &cfg,
                &Progress::new(),
                &mut stats,
                None,
            )
            .unwrap();
            let jpegs = encode_pages(&tex, cfg.jpeg_quality).unwrap();
            let pages = tex.to_mesh(&jpegs).unwrap();
            let s = WeldedSurface::from_mesh(&pages);
            (pages, s, with_rod)
        })
    }

    /// The largest difference from the wall's colour (sRGB levels, any channel) of the patch's
    /// texture at points `(dx, z)` of the +y wall of [`rod_wall`] (y = 5), `dx` from the line
    /// where the rod lies in the image of its first place (seen from there, its middle on the
    /// wall): bilinear in the page of the patch triangle that holds each.
    fn off_the_wall(patch: &SurfacePatch, src: &TextureSource, points: &[[f64; 2]]) -> Vec<f64> {
        let first = src.poses[0].p + src.poses[0].q * src.cams[0].cam_in_body.translation.vector;
        // From the camera through the rod's middle (y 4.5) onto the wall (y 5).
        let x = first.x + (ROD_X - first.x) * (5.0 - first.y) / (4.5 - first.y);
        let images: Vec<RgbImage> = patch
            .pages
            .iter()
            .map(|p| decode_jpeg(&p.jpeg, 1).unwrap())
            .collect();
        let colour = |p: [f64; 2]| -> [f64; 3] {
            for (page, image) in patch.pages.iter().zip(&images) {
                for tri in &page.indices {
                    let c = tri.map(|k| page.positions[k as usize]);
                    let (a, b, d) = ([c[0][0], c[0][2]], [c[1][0], c[1][2]], [c[2][0], c[2][2]]);
                    let det = (b[0] - a[0]) * (d[1] - a[1]) - (d[0] - a[0]) * (b[1] - a[1]);
                    let u = ((p[0] - a[0]) * (d[1] - a[1]) - (d[0] - a[0]) * (p[1] - a[1])) / det;
                    let v = ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / det;
                    let w = [1.0 - u - v, u, v];
                    if w.iter().all(|&x| x >= -1e-9) {
                        let uv = [0, 1].map(|a| {
                            (0..3)
                                .map(|k| w[k] * f64::from(page.uvs[tri[k] as usize][a]))
                                .sum::<f64>() as f32
                        });
                        return colour_at(image, uv);
                    }
                }
            }
            panic!("{p:?} is on no triangle of the patch")
        };
        points
            .iter()
            .map(|&[dx, z]| {
                let got = colour([x + dx, z]);
                (0..3)
                    .map(|c| (got[c] - f64::from(WALL_Y[c])).abs())
                    .fold(0.0, f64::max)
            })
            .collect()
    }

    #[test]
    fn thin_occluder_not_baked() {
        // A thin rod 0.5 m in front of the wall, in the images but not in the surface: seen from
        // the first place (head-on, the best image: variant 0) it hides a stripe of the Stelle.
        // The other four images show the wall there; their own stripes lie elsewhere. The Stelle
        // takes the first image, but where the rod hides the wall in it (and along the edge of
        // that, where it hides the faces in part) the faces take another.
        let (pages, s, src) = rod_wall();
        let cfg = walk_config();
        let spot = spot_at(s, [0.0, 5.0, 0.0], 0.45, [0.0, -1.0, 0.0]);
        assert!(spot.len() > 400, "{}", spot.len());
        let sources = std::slice::from_ref(src);
        let ranked = variants(
            s,
            &FaceBvh::build(s),
            pages,
            &spot,
            sources,
            &index_of(sources),
            &cfg,
            &Progress::new(),
        )
        .unwrap();
        assert_eq!(ranked.len(), ROD_PLACES.len(), "{ranked:?}");
        assert_eq!(ranked[0], (0, 0), "the head-on image first: {ranked:?}");
        let done = retexture_here(s, pages, &spot, sources, 0, &cfg, &Progress::new()).unwrap();
        assert_eq!(done.variant_of, (1, ROD_PLACES.len() as u32));
        // Along the rod as the first image shows it, from z −0.3 to 0.3 m, and across it (10 cm
        // either side of its middle; it hides 6 cm either side).
        let along: Vec<[f64; 2]> = (-6..=6).map(|i| [0.0, 0.05 * f64::from(i)]).collect();
        let across: Vec<[f64; 2]> = [-0.2, 0.0, 0.2]
            .iter()
            .flat_map(|&z| (-10..=10).map(move |i| [0.01 * f64::from(i), z]))
            .collect();
        let off = off_the_wall(&done.patch, src, &along);
        println!("along the rod: {off:.1?} sRGB levels from the wall");
        assert!(off.iter().all(|&d| d <= 8.0), "{off:.1?}");
        let off = off_the_wall(&done.patch, src, &across);
        println!("across it: {off:.0?}");
        assert!(off.iter().all(|&d| d <= 8.0), "{off:.1?}");
        // Without the judgement every view takes its faces: the first image bakes the rod onto
        // the wall.
        let all = retexture(
            s,
            &FaceBvh::build(s),
            pages,
            &spot,
            sources,
            &index_of(sources),
            0,
            &cfg,
            &Progress::new(),
            Matching::Full,
            Sight::BEST,
            Judging::All,
        )
        .unwrap();
        let baked = off_the_wall(&all.patch, src, &along);
        println!("without the judgement: {baked:.1?}");
        assert!(baked.iter().all(|&d| d > 100.0), "{baked:.1?}");
        // The same again, byte for byte.
        let again = retexture_here(s, pages, &spot, sources, 0, &cfg, &Progress::new()).unwrap();
        assert_eq!(patch_hash(&again.patch), patch_hash(&done.patch));
    }

    #[test]
    fn select_median_is_the_median() {
        use crate::surface::testutil::Lcg;
        let mut rng = Lcg::new(7);
        for n in [0, 1, 2, 3, 4, 5, 8, 9, 100, 101] {
            let v: Vec<f64> = (0..n).map(|_| rng.range(0.0, 0.05)).collect();
            let (mut a, mut b) = (v.clone(), v);
            assert_eq!(
                select_median(&mut a).to_bits(),
                median(&mut b).to_bits(),
                "{n}"
            );
        }
    }

    #[test]
    fn match_gain_over_the_faces_around() {
        // Nine faces in a row, each with the faces up to four away around it; image 1 shows them
        // 1.5 times as bright as image 0 (face 8: 4 times). Face 4 takes image 1 for image 0,
        // whose view of it is left out.
        let colour = |x: f64| Some([x, x, x]);
        let judged = |out: Vec<Vec<u32>>| Judged {
            out,
            colours: vec![
                (0..9).map(|_| colour(0.1)).collect(),
                (0..9)
                    .map(|i| colour(if i == 8 { 0.4 } else { 0.15 }))
                    .collect(),
            ],
            around: (0..9u32)
                .map(|i| (i.saturating_sub(4)..(i + 5).min(9)).collect())
                .collect(),
        };
        // Image 0 does not take the faces where its view is left out.
        let taken =
            |out: &[Vec<u32>]| -> Vec<bool> { out.iter().map(|o| o.contains(&0)).collect() };
        let rounded = |g: [f32; 3]| g.map(|x| (f64::from(x) * 1e6).round() / 1e6);
        let lifted = ((0.15 + GAIN_FLOOR) / (0.1 + GAIN_FLOOR)) as f32;
        let mut out = vec![Vec::new(); 9];
        out[4] = vec![0];
        // Around face 4: faces 0 to 8 but itself, seven at 1.5 and one at 4 times.
        let g = judged(out.clone()).match_gain(1, 0, &[4], &taken(&out), 4.0);
        assert_eq!(rounded(g), rounded([lifted; 3]));
        // Clamped to the largest gain.
        let mut bright = judged(out.clone());
        for c in &mut bright.colours[1] {
            *c = colour(0.5);
        }
        assert_eq!(bright.match_gain(1, 0, &[4], &taken(&out), 4.0), [4.0; 3]);
        // Fewer than MIN_GAIN_FACES faces around that image 0 takes and where image 1's view is
        // kept: no matching.
        out[0] = vec![1];
        out[1] = vec![0];
        out[2] = vec![0, 1];
        let few = taken(&out);
        assert_eq!(judged(out).match_gain(1, 0, &[4], &few, 4.0), [1.0; 3]);
    }
    /// A patch by the hash of its bytes and the variant it is, or why there is none.
    type PatchOf = Result<(u64, (u32, u32)), String>;

    /// The bytes [`write_patches`] writes for `patch`, hashed (FNV-1a).
    fn patch_hash(patch: &SurfacePatch) -> u64 {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = TestDir::new(format!(
            "driftless-edit-retexture-hash-{}-{n}",
            std::process::id()
        ));
        write_patches(&dir, std::slice::from_ref(patch)).unwrap();
        crate::content_hash(&fs::read(dir.join(PATCHES_FILE)).unwrap())
    }

    /// The walk through the painted room (`scene::walk_scene`) textured as the surface step
    /// does it (1 cm texels, pages of 1024²) from the walk's images of both cameras, welded.
    fn textured_walk() -> &'static (PageMesh, WeldedSurface, TextureSource) {
        use crate::surface::{texture_mesh, SurfaceStats};
        static WALK: OnceLock<(PageMesh, WeldedSurface, TextureSource)> = OnceLock::new();
        WALK.get_or_init(|| {
            let mesh = painted_mesh(&scene::walk_scene(), 0.1);
            let src = walk_source("retexture-textured-walk", &[0, 1], 1..=11, 1.0);
            let cfg = walk_config();
            let mut stats = SurfaceStats::default();
            let tex = texture_mesh(
                &mesh,
                std::slice::from_ref(&src),
                &cfg,
                &Progress::new(),
                &mut stats,
                None,
            )
            .unwrap();
            let jpegs = encode_pages(&tex, cfg.jpeg_quality).unwrap();
            let pages = tex.to_mesh(&jpegs).unwrap();
            let s = WeldedSurface::from_mesh(&pages);
            (pages, s, src)
        })
    }

    /// The settings [`textured_walk`] was textured with.
    fn walk_config() -> SurfaceConfig {
        SurfaceConfig {
            min_texel: 0.01,
            page_size: 1024,
            ..SurfaceConfig::default()
        }
    }

    /// The faces of `s` whose centroid lies within `r` of `centre` and whose normal is within
    /// about 25° of `normal`.
    fn spot_at(s: &WeldedSurface, centre: [f64; 3], r: f64, normal: [f64; 3]) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                norm(sub(s.face_centroid(f), centre)) < r && dot(s.face_normal(f), normal) > 0.9
            })
            .collect()
    }

    /// The patches of „Bilder neu“ on the synthetic scenes, recorded (Apple silicon, as
    /// [`crate::surface::tests::full_surface_unchanged`]) before the images were found through
    /// an index of the bags and the visibility was computed only for the best candidates and
    /// over the cut-out of the image: the same images, the same patches, byte for byte.
    #[test]
    fn retexture_unchanged() {
        let room = room();
        let cfg = testscan_config();
        let front = front_spot(&room.surface, 0.8);
        let side = side_spot(&room.surface, 0.8);
        let three = [room.scan.clone(), nearer(2.0), nearer(3.5)];
        let two = [exposed(1.2).clone(), nearer(2.0)];
        let mut got: Vec<(String, PatchOf)> = Vec::new();
        let mut run = |name: &str,
                       s: &WeldedSurface,
                       mesh: &PageMesh,
                       spot: &[u32],
                       sources: &[TextureSource],
                       variant: u32,
                       cfg: &SurfaceConfig| {
            let r = retexture_here(s, mesh, spot, sources, variant, cfg, &Progress::new())
                .map(|d| (patch_hash(&d.patch), d.variant_of))
                .map_err(|e| e.to_string());
            got.push((name.to_string(), r));
        };
        run(
            "front, scan",
            &room.surface,
            &room.mesh,
            &front,
            std::slice::from_ref(&room.scan),
            0,
            &cfg,
        );
        for v in [0, 4, 5, 10, 14] {
            run(
                &format!("front, three places, variant {v}"),
                &room.surface,
                &room.mesh,
                &front,
                &three,
                v,
                &cfg,
            );
        }
        run(
            "front, two sources, variant 3",
            &room.surface,
            &room.mesh,
            &front,
            &two,
            3,
            &cfg,
        );
        run(
            "side, darker",
            &room.surface,
            &room.mesh,
            &side,
            std::slice::from_ref(exposed(0.8)),
            0,
            &cfg,
        );
        let (pages, s, src) = textured_walk();
        let wcfg = walk_config();
        let wall = spot_at(s, [1.0, 5.0, 0.3], 0.565, [0.0, -1.0, 0.0]);
        let behind = spot_at(s, [-3.5, -4.0, 0.0], 1.0, [0.0, 1.0, 0.0]);
        let floor = spot_at(s, [0.5, 1.5, -1.2], 0.6, [0.0, 0.0, 1.0]);
        for (name, spot) in [
            ("walk, +y wall", &wall),
            ("walk, behind the pillar", &behind),
        ] {
            for v in [0, 1, 7] {
                run(
                    &format!("{name}, variant {v}"),
                    s,
                    pages,
                    spot,
                    std::slice::from_ref(src),
                    v,
                    &wcfg,
                );
            }
        }
        run(
            "walk, floor",
            s,
            pages,
            &floor,
            std::slice::from_ref(src),
            0,
            &wcfg,
        );
        for (name, r) in &got {
            match r {
                Ok((h, v)) => println!("({name:?}, 0x{h:016x}, {v:?}),"),
                Err(e) => println!("({name:?}, {e:?}),"),
            }
        }
        #[cfg(all(target_os = "macos", target_arch = "aarch64"))]
        {
            let want: [(&str, u64, (u32, u32)); 15] = [
                ("front, scan", 0xad94db987087cb22, (1, 5)),
                (
                    "front, three places, variant 0",
                    0xd31cfad808618677,
                    (1, 15),
                ),
                (
                    "front, three places, variant 4",
                    0xd31cfad808618677,
                    (5, 15),
                ),
                (
                    "front, three places, variant 5",
                    0xad94db987087cb22,
                    (6, 15),
                ),
                (
                    "front, three places, variant 10",
                    0xc9827c5ba3b54492,
                    (11, 15),
                ),
                (
                    "front, three places, variant 14",
                    0xc9827c5ba3b54492,
                    (15, 15),
                ),
                ("front, two sources, variant 3", 0xd31cfad808618677, (4, 10)),
                ("side, darker", 0xe9d513978068f927, (1, 5)),
                ("walk, +y wall, variant 0", 0xe9188df5a88c99e5, (1, 11)),
                ("walk, +y wall, variant 1", 0xd761518c9f1aaeae, (2, 11)),
                ("walk, +y wall, variant 7", 0x8d19242da74739e6, (8, 11)),
                (
                    "walk, behind the pillar, variant 0",
                    0xdbe31d78ed7db39c,
                    (1, 11),
                ),
                (
                    "walk, behind the pillar, variant 1",
                    0x27c92c6de0d7443d,
                    (2, 11),
                ),
                (
                    "walk, behind the pillar, variant 7",
                    0xc4db214a0b83abfc,
                    (8, 11),
                ),
                ("walk, floor", 0x67b8828b5c08eca0, (1, 11)),
            ];
            let want: Vec<(String, PatchOf)> = want
                .iter()
                .map(|&(name, hash, variant)| (name.to_string(), Ok((hash, variant))))
                .collect();
            assert_eq!(got, want, "patches changed");
        }
    }

    /// The room of [`room`] with a box (x 1.5 … 1.8, y −0.25 … 0.25, z −0.4 … 0.3; not in the
    /// images): it hides [`front_spot`] from the places of [`crowd`] behind it.
    fn boxed_room() -> &'static (PageMesh, WeldedSurface) {
        static BOXED: OnceLock<(PageMesh, WeldedSurface)> = OnceLock::new();
        BOXED.get_or_init(|| {
            let mut mesh = room().mesh.clone();
            let (lo, hi) = ([1.5, -0.25, -0.4], [1.8, 0.25, 0.3]);
            let corner = |i: usize| [0, 1, 2].map(|k| if i >> k & 1 == 1 { hi[k] } else { lo[k] });
            // Each side as two triangles, wound so that the normals point out of the box.
            let sides: [[usize; 4]; 6] = [
                [0, 2, 6, 4],
                [1, 5, 7, 3],
                [0, 4, 5, 1],
                [2, 3, 7, 6],
                [0, 1, 3, 2],
                [4, 6, 7, 5],
            ];
            let mut page = driftless_export::mesh::MeshPage {
                positions: (0..8).map(corner).collect(),
                uvs: vec![[0.5, 0.5]; 8],
                indices: Vec::new(),
                image: None,
                color: [128, 128, 128],
            };
            for [a, b, c, d] in sides {
                page.indices.push([a as u32, b as u32, c as u32]);
                page.indices.push([a as u32, c as u32, d as u32]);
            }
            mesh.pages.push(page);
            let s = WeldedSurface::from_mesh(&mesh);
            (mesh, s)
        })
    }

    /// Places on a grid in front of the +x wall (x −2 … 4 m, y −1 … 3 m), the scan's images at
    /// each (only the geometry counts): 100 candidates for [`front_spot`], some behind the
    /// box of [`boxed_room`].
    fn crowd() -> &'static Vec<TextureSource> {
        static CROWD: OnceLock<Vec<TextureSource>> = OnceLock::new();
        CROWD.get_or_init(|| {
            let mut out = Vec::new();
            for x in [-3.0, -2.0, -1.0, 0.0, 1.0] {
                for y in [0.0, 0.5, 1.0, 1.5] {
                    out.push(testscan_source(
                        &room().dir,
                        Isometry3::translation(x, y, 0.0),
                    ));
                }
            }
            out
        })
    }

    /// A synthetic case of [`region_views`]: the surface, its tree, the Stelle, the ring looked
    /// at with it, the sources, their indices, the settings.
    struct Case {
        name: String,
        s: &'static WeldedSurface,
        bvh: FaceBvh,
        spot: Vec<u32>,
        around: Vec<u32>,
        sources: Vec<TextureSource>,
        indices: Vec<Arc<ImageIndex>>,
        cfg: SurfaceConfig,
    }

    impl Case {
        fn new(
            name: &str,
            s: &'static WeldedSurface,
            spot: Vec<u32>,
            ring: bool,
            sources: Vec<TextureSource>,
            cfg: SurfaceConfig,
        ) -> Case {
            assert!(!spot.is_empty(), "{name}");
            let around = if ring {
                ring_around(s, &spot)
            } else {
                Vec::new()
            };
            Case {
                name: name.to_string(),
                s,
                bvh: FaceBvh::build(s),
                spot,
                around,
                indices: index_of(&sources),
                sources,
                cfg,
            }
        }

        fn views(&self, sight: Sight) -> (Vec<RegionView>, SightStats) {
            region_views(
                self.s,
                &self.bvh,
                &self.spot,
                &self.around,
                &self.sources,
                &self.indices,
                &self.cfg,
                sight,
                &Progress::new(),
            )
            .unwrap()
        }
    }

    /// The surface of [`room`] without its +x wall (behind it, the camera sees the Stelle).
    fn open_room() -> &'static WeldedSurface {
        static OPEN: OnceLock<WeldedSurface> = OnceLock::new();
        OPEN.get_or_init(|| WeldedSurface::from_mesh(&without(&room().mesh, |c| c[0] > 5.9)))
    }

    /// Every synthetic scene of the tests above, as cases of [`region_views`].
    fn cases() -> Vec<Case> {
        let room = room();
        let s = &room.surface;
        let cfg = testscan_config();
        let mut narrow = testscan_config();
        narrow.color.rim_theta = 28f64.to_radians();
        let (_, boxed) = boxed_room();
        let (walk_s, walk_src) = walk();
        let (_, tw, tw_src) = textured_walk();
        let near = |s: &WeldedSurface, c: [f64; 3], r: f64, n: [f64; 3]| spot_at(s, c, r, n);
        let mut out = vec![
            Case::new(
                "room, −x wall, three sources",
                s,
                wall_spot(s, 0.6),
                false,
                vec![
                    room.scan.clone(),
                    looking_back([1.0, 1.0, 0.25]),
                    looking_back([6.6, 1.0, 0.25]),
                ],
                cfg.clone(),
            ),
            Case::new(
                "room without the +x wall",
                open_room(),
                wall_spot(open_room(), 0.6),
                false,
                vec![
                    room.scan.clone(),
                    looking_back([1.0, 1.0, 0.25]),
                    looking_back([6.6, 1.0, 0.25]),
                ],
                cfg.clone(),
            ),
            Case::new(
                "room, 2 m and 5 m",
                s,
                wall_spot(s, 0.6),
                true,
                vec![
                    looking_back([1.0, 1.0, 0.25]),
                    looking_back([-2.0, 1.0, 0.25]),
                ],
                cfg.clone(),
            ),
            Case::new(
                "room, six places",
                s,
                wall_spot(s, 0.6),
                false,
                (0..6)
                    .map(|i| looking_back([-1.0 + f64::from(i), 1.0, 0.25]))
                    .collect(),
                cfg.clone(),
            ),
            Case::new(
                "room, within 28°",
                s,
                wall_spot(s, 0.6),
                false,
                vec![
                    looking_back([1.0, 1.0, 0.25]),
                    looking_back([-2.5, 1.0, 0.25]),
                ],
                narrow,
            ),
            Case::new(
                "room, front wall with ring",
                s,
                front_spot(s, 0.6),
                true,
                vec![room.scan.clone(), nearer(2.0)],
                cfg.clone(),
            ),
            Case::new(
                "room, front wall, three places",
                s,
                front_spot(s, 0.8),
                true,
                vec![room.scan.clone(), nearer(2.0), nearer(3.5)],
                cfg.clone(),
            ),
            Case::new(
                "room, the same place three times",
                s,
                front_spot(s, 0.8),
                true,
                vec![nearer(2.0), nearer(2.0), nearer(2.0)],
                cfg.clone(),
            ),
            Case::new(
                "room, a crowd of places",
                s,
                front_spot(s, 0.8),
                true,
                crowd().clone(),
                cfg.clone(),
            ),
            Case::new(
                "room with a box, a crowd of places",
                boxed,
                front_spot(boxed, 0.8),
                true,
                crowd().clone(),
                cfg.clone(),
            ),
            Case::new(
                "textured walk, +y wall",
                tw,
                near(tw, [1.0, 5.0, 0.3], 0.565, [0.0, -1.0, 0.0]),
                true,
                vec![tw_src.clone()],
                walk_config(),
            ),
            Case::new(
                "textured walk, behind the pillar",
                tw,
                near(tw, [-3.5, -4.0, 0.0], 1.0, [0.0, 1.0, 0.0]),
                true,
                vec![tw_src.clone()],
                walk_config(),
            ),
            Case::new(
                "textured walk, floor",
                tw,
                near(tw, [0.5, 1.5, -1.2], 0.6, [0.0, 0.0, 1.0]),
                true,
                vec![tw_src.clone()],
                walk_config(),
            ),
        ];
        for (c, r, n) in [
            ([-3.5, -4.0, 0.0], 1.6, [0.0, 1.0, 0.0]),
            ([0.5, 1.5, -1.2], 0.8, [0.0, 0.0, 1.0]),
            ([1.0, 5.0, 0.3], 0.7, [0.0, -1.0, 0.0]),
        ] {
            out.push(Case::new(
                &format!("walk on pages, Stelle at {c:?}"),
                walk_s,
                near(walk_s, c, r, n),
                false,
                vec![walk_src.clone()],
                SurfaceConfig::default(),
            ));
        }
        out
    }

    /// Only over the cut-out of each image the Stelle and its ring are tested in, the depth
    /// buffer gives the same views as over the whole image: the same images, the same faces,
    /// the same scores, bit for bit, in every scene; and it is smaller.
    #[test]
    fn cutout_visibility_matches_full() {
        let all = Sight {
            cutout: false,
            ..Sight::ALL
        };
        let mut smaller = 0;
        for case in cases() {
            let (whole, whole_stats) = case.views(all);
            let (cut, cut_stats) = case.views(Sight::ALL);
            assert_eq!(cut, whole, "{}", case.name);
            assert!(!whole.is_empty(), "{}", case.name);
            assert_eq!(cut_stats.buffered, whole_stats.buffered, "{}", case.name);
            assert!(cut_stats.cells <= whole_stats.cells, "{}", case.name);
            smaller += usize::from(cut_stats.cells * 2 < whole_stats.cells);
            println!(
                "{}: {} views; depth buffers of {} candidates, {} cells instead of {}",
                case.name,
                cut.len(),
                cut_stats.buffered,
                cut_stats.cells,
                whole_stats.cells
            );
        }
        assert!(smaller >= 8, "the cut-out is smaller in {smaller} scenes");
    }

    /// Ranked first by what each image could see with nothing in the way, and given a depth
    /// buffer only as long as one can still change the best [`DECODED_VIEWS`]: the same best
    /// views as when every candidate gets one, in the same order, in every scene and in rounds
    /// of any size; with them the same images decoded and the same patches, byte for byte. In
    /// the crowd before the plain wall fewer candidates get a depth buffer (behind the box
    /// every place that could see the Stelle without it has to be looked at).
    #[test]
    fn prerank_keeps_choice() {
        for case in cases() {
            let (all, all_stats) = case.views(Sight::ALL);
            let best: Vec<RegionView> = all.iter().take(DECODED_VIEWS).cloned().collect();
            for batch in [1, 2, 5, PRERANK_BATCH] {
                let (got, stats) = case.views(Sight {
                    batch,
                    ..Sight::BEST
                });
                assert_eq!(got, best, "{}, rounds of {batch}", case.name);
                assert!(stats.buffered <= all_stats.buffered, "{}", case.name);
                if batch == PRERANK_BATCH {
                    println!(
                        "{}: {} candidates, {} views; depth buffers for {} of them in {} \
                         round(s)",
                        case.name,
                        all_stats.candidates,
                        all.len(),
                        stats.buffered,
                        stats.rounds
                    );
                }
            }
            if case.name == "room, a crowd of places" {
                let (_, stats) = case.views(Sight::BEST);
                assert!(
                    stats.buffered < all_stats.buffered,
                    "{}: {} of {}",
                    case.name,
                    stats.buffered,
                    all_stats.buffered
                );
            }
        }
        // The patches: the room and its crowd, the textured walk.
        let room = room();
        let (boxed_mesh, boxed) = boxed_room();
        let (pages, tw, tw_src) = textured_walk();
        let three = [room.scan.clone(), nearer(2.0), nearer(3.5)];
        type Check<'a> = (
            &'a str,
            &'a WeldedSurface,
            &'a PageMesh,
            Vec<u32>,
            Vec<TextureSource>,
            SurfaceConfig,
        );
        let checks: Vec<Check> = vec![
            (
                "front",
                &room.surface,
                &room.mesh,
                front_spot(&room.surface, 0.8),
                three.to_vec(),
                testscan_config(),
            ),
            (
                "crowd",
                &room.surface,
                &room.mesh,
                front_spot(&room.surface, 0.8),
                crowd().clone(),
                testscan_config(),
            ),
            (
                "crowd, box",
                boxed,
                boxed_mesh,
                front_spot(boxed, 0.8),
                crowd().clone(),
                testscan_config(),
            ),
            (
                "walk wall",
                tw,
                pages,
                spot_at(tw, [1.0, 5.0, 0.3], 0.565, [0.0, -1.0, 0.0]),
                vec![tw_src.clone()],
                walk_config(),
            ),
            (
                "walk pillar",
                tw,
                pages,
                spot_at(tw, [-3.5, -4.0, 0.0], 1.0, [0.0, 1.0, 0.0]),
                vec![tw_src.clone()],
                walk_config(),
            ),
        ];
        for (name, s, mesh, spot, sources, cfg) in checks {
            let bvh = FaceBvh::build(s);
            let indices = index_of(&sources);
            for variant in [0, 1, 7, 23] {
                let patch = |sight: Sight| {
                    retexture(
                        s,
                        &bvh,
                        mesh,
                        &spot,
                        &sources,
                        &indices,
                        variant,
                        &cfg,
                        &Progress::new(),
                        Matching::Full,
                        sight,
                        Judging::Consistent,
                    )
                    .map(|d| (patch_hash(&d.patch), d.variant_of))
                    .map_err(|e| e.to_string())
                };
                let (with, without) = (patch(Sight::BEST), patch(Sight::ALL));
                println!("{name}, variant {variant}: {with:?}");
                assert_eq!(with, without, "{name}, variant {variant}");
            }
        }
    }

    /// Time of „Bilder neu einspielen“ per phase, and the bytes of the bag it reads: the walk
    /// through the painted room (22 images of 640 × 480 from two cameras), textured as the
    /// surface step does it, and a Stelle of about 1 m² on the +y wall.
    /// `cargo test -p driftless-map --lib retexture::tests::timing_walk -- --ignored --nocapture`
    #[test]
    #[ignore = "Messung"]
    fn timing_walk() {
        use crate::surface::{texture_mesh, SurfaceStats};
        use std::sync::{Arc, Mutex};
        let mesh = painted_mesh(&scene::walk_scene(), 0.1);
        let src = walk_source("retexture-timing", &[0, 1], 1..=11, 1.0);
        let cfg = SurfaceConfig {
            min_texel: 0.005,
            page_size: 2048,
            ..SurfaceConfig::default()
        };
        let mut stats = SurfaceStats::default();
        let sources = std::slice::from_ref(&src);
        let tex = texture_mesh(&mesh, sources, &cfg, &Progress::new(), &mut stats, None).unwrap();
        let jpegs = encode_pages(&tex, cfg.jpeg_quality).unwrap();
        let pages = tex.to_mesh(&jpegs).unwrap();
        let s = WeldedSurface::from_mesh(&pages);
        let spot: Vec<u32> = (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                norm(sub(c, [1.0, 5.0, 0.3])) < 0.565 && s.face_normal(f)[1] < -0.9
            })
            .collect();
        let area: f64 = spot.iter().map(|&f| 0.5 * norm(s.face_cross(f))).sum();
        let marks: Arc<Mutex<Vec<(String, Instant)>>> = Arc::default();
        let seen = Arc::clone(&marks);
        let progress = Progress::new().with_observer(move |snap| {
            let mut m = seen.lock().unwrap();
            if m.last().is_none_or(|l| l.0 != snap.phase) {
                m.push((snap.phase.clone(), Instant::now()));
            }
        });
        let start = Instant::now();
        let done = retexture_here(&s, &pages, &spot, sources, 0, &cfg, &progress).unwrap();
        let end = Instant::now();
        let marks = marks.lock().unwrap().clone();
        let phases: Vec<String> = marks
            .iter()
            .enumerate()
            .map(|(i, (name, t))| {
                let next = marks.get(i + 1).map_or(end, |m| m.1);
                format!("{name}: {:?}", next - *t)
            })
            .collect();
        let bag = fs::metadata(&src.images.bags[0]).unwrap().len();
        println!(
            "{} faces ({area:.2} m²) of {}, {} new on {} page(s) of {}², variant {:?}: {:?} in \
             all ({}); bag {bag} bytes, read whole for the headers and up to the last image \
             decoded",
            spot.len(),
            s.face_count(),
            done.patch.removed_faces.len(),
            done.patch.pages.len(),
            decode_jpeg(&done.patch.pages[0].jpeg, 1).unwrap().width,
            done.variant_of,
            end - start,
            phases.join(", ")
        );
    }
    /// Time to find the ring of a Stelle of 1 m² and its old texture's triangles on a surface of
    /// Büro size (1818² squares of 3.5 cm: 6.6 M triangles in charts of 18 faces dealt to 28
    /// pages, so the Stelle's charts lie on every page).
    /// `cargo test -p driftless-map --release retexture::tests::timing_buero_size -- --ignored --nocapture`
    #[test]
    #[ignore = "Messung"]
    fn timing_buero_size() {
        use crate::surface::testutil::{grid, Lcg};
        let (n, step) = (1818, 0.035);
        let mut rng = Lcg::new(1);
        let (positions, indices) = grid([n, n], |i, j| {
            [i as f64 * step, j as f64 * step, 0.005 * rng.normal()].map(|c| f64::from(c as f32))
        });
        let mesh = paged(&positions, &indices, 18, 28);
        drop((positions, indices));
        let s = WeldedSurface::from_mesh(&mesh);
        let middle = n as f64 * step / 2.0;
        let r = (1.0 / std::f64::consts::PI).sqrt();
        let spot: Vec<u32> = (0..s.face_count() as u32)
            .filter(|&f| norm(sub(s.face_centroid(f), [middle, middle, 0.0])) < r)
            .collect();
        let clock = Instant::now();
        let around = ring_around(&s, &spot);
        let ring_time = clock.elapsed();
        let clock = Instant::now();
        let neighbours = edge_neighbours(&s, &spot);
        let edge_time = clock.elapsed();
        let mut looked = spot.clone();
        looked.extend_from_slice(&around);
        looked.sort_unstable();
        let clock = Instant::now();
        let located = Located::new(&s, &mesh, looked.clone());
        let locate_time = clock.elapsed();
        assert!(located.at.iter().all(Option::is_some));
        for (&f, at) in looked.iter().zip(&located.at) {
            let (p, t) = s.face_origin(f);
            assert_eq!(*at, Some((p, t)));
        }
        assert_eq!(neighbours.len(), spot.len());
        println!(
            "{} faces, ring of {}: ring {ring_time:?}, edge neighbours {edge_time:?}, found on \
             the pages {locate_time:?}",
            spot.len(),
            around.len()
        );
    }
}
