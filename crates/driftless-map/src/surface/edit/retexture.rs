//! „Bilder neu einspielen“, first part: which camera images show a Stelle, and how well.
//!
//! The surface step looks through every image of every scan for every face; for one Stelle
//! that would read and decode all images again (Büro: 3,831 images, about three times). Here
//! no image is decoded to find the candidates, and only the best few are decoded at all:
//!
//! 1. **Candidates** ([`region_views`]): the image headers of every source are listed without
//!    decoding a JPEG ([`for_each_image_where`]); an image counts if its capture time lies on
//!    the trajectory, the device turned no faster than [`ColorConfig::max_angular_rate`] then,
//!    and from its camera (`trajectory(stamp + time_offset) · cam_in_body`) the Stelle lies
//!    within [`ColorConfig::max_dist`], within [`ColorConfig::rim_theta`] of the optical axis
//!    and at least one of its faces faces the camera. The quality pass of the surface step
//!    (sharpness against the scan's median) needs the decoded images and is left out: the
//!    sharpness on the Stelle itself is measured on the decoded few (the second part).
//! 2. **Visibility**: per candidate a depth buffer as the surface step builds it, but only from
//!    the faces that can reach the cells the Stelle is tested in: those in the cone from the
//!    camera around the Stelle, widened by [`MAX_SPLAT`] + 2 cells ([`CONE_MARGIN`] times
//!    over), and not farther than the Stelle's farthest point. A face beyond both changes no
//!    cell a face of the Stelle is tested in, so a face of the Stelle is seen exactly when the
//!    surface step would see it (with its own normals and depth tolerance). Which faces are in
//!    the cone is decided with `acos`/`asin`, whose last bit can differ between machines; it
//!    decides only about faces far from those cells, so the views do not depend on it. The
//!    faces are found in a [`FaceBvh`] of the surface. The score of a face is the surface
//!    step's ([`super::super::views`]: pixels at full size, capped at
//!    [`SurfaceConfig::min_texel`], times the cosine to the normal smoothed over two rings)
//!    with `sharpness_norm` 1; a view that may show the carrier counts at
//!    [`CARRIER_ALTERNATIVE`](super::super::views::CARRIER_ALTERNATIVE) of it, as in the slot
//!    order there. So the scores of one face compare across images and scans.
//! 3. **Ranking**: an image covering at least [`FULL_COVERAGE`] of the Stelle's area can
//!    texture it alone; those come first, then the others, each by their quality (the area-
//!    weighted mean over the Stelle of `√(score / best score of the face)`, 0 where unseen),
//!    then in stream order. [`decode_views`] decodes the first [`DECODED_VIEWS`] at full size.
//!
//! Everything runs on the CPU and is deterministic: per image an ordered map, sums and sorts
//! in a fixed order with a tie rule, the depth buffer a minimum on the bits of an `f32`.

// Only the tests use this part until the second part of „Bilder neu einspielen“ (in this file)
// does; then the expectation fails and goes.
#![cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "used by the second part of „Bilder neu einspielen“"
    )
)]

use super::{dot, sub, EditError, FaceBvh, WeldedSurface};
use crate::color::{angular_rate, image_count, linear_table, ColorConfig};
use crate::surface::views::{image_views, Faces, Shot, TextureSource};
use crate::surface::{Mesh, SurfaceConfig};
use crate::visibility::MAX_SPLAT;
use crate::MapError;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image_where, ImageHeader, RgbImage, Wanted};
use driftless_io::progress::Progress;
use driftless_traj::interpolate_pose;
use nalgebra::{Isometry3, Vector3};
use rayon::prelude::*;

/// At most this many images of a Stelle are decoded at full size: the best ones
/// ([`decode_views`]).
pub(crate) const DECODED_VIEWS: usize = 24;

/// An image that sees at least this share of a Stelle's area can texture it alone: such images
/// rank first ([`region_views`]).
pub(crate) const FULL_COVERAGE: f32 = 0.9;

/// The cone of faces that can hide a Stelle from a camera is widened by this many times
/// the angle of [`MAX_SPLAT`] + 2 depth-buffer cells at the image centre: a splat reaches
/// [`MAX_SPLAT`] cells, the silhouette test one more, and a pixel off the centre of a fisheye
/// spans more angle than one at it.
pub(crate) const CONE_MARGIN: f64 = 3.0;

/// Slack of the tests that leave out images and faces (m, rad): far above the rounding of the
/// `f32` positions the surface step looks with, far below anything that matters; they only
/// ever keep more.
const SLACK: f64 = 1e-6;

/// Phase of [`region_views`]: the headers of the images, one step each.
const PHASE_FIND: &str = "Bilder der Stelle suchen";
/// Phase of [`decode_views`]: one step per image decoded.
const PHASE_DECODE: &str = "Bilder der Stelle lesen";

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
}

/// The images of `sources` that see faces of the Stelle `faces` (welded ids, any order) on the
/// surface `s`, best first (see the module docs): those covering [`FULL_COVERAGE`] of it, then
/// the others, each by [`RegionView::quality`], then by source and frame. `cfg` is the surface's
/// (as its report has it). No image is decoded; the bags of every source are read once.
/// Progress: phase „Bilder der Stelle suchen“, one step per image; a cancel is checked at every
/// image and returns [`EditError::Cancelled`]. Same input, same views.
///
/// Fails if `cfg` is invalid, two cameras of one source share a name or the bags cannot be
/// read.
///
/// # Panics
/// If a face is not one of `s`.
pub(crate) fn region_views(
    s: &WeldedSurface,
    faces: &[u32],
    sources: &[TextureSource],
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<Vec<RegionView>, EditError> {
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    cfg.check()?;
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
    if region.is_empty() || sources.is_empty() {
        return Ok(Vec::new());
    }
    let ccfg = &cfg.color;
    let shape = Shape::new(s, &region);
    let candidates = candidates(&shape, sources, ccfg, progress)?;
    if candidates.is_empty() {
        return Ok(Vec::new());
    }

    // Per candidate the faces that can hide the Stelle from it; with the two rings around the
    // Stelle (for its smoothed normals) the part of the surface to look at.
    let bvh = FaceBvh::build(s);
    let splats: Vec<Vec<u32>> = candidates
        .par_iter()
        .map(|c| {
            let cam = &sources[c.source as usize].cams[c.camera as usize].camera;
            in_front(s, &bvh, &shape, &c.world_from_cam, cam, ccfg)
        })
        .collect();
    drop(bvh);
    if progress.is_cancelled() {
        return Err(EditError::Cancelled);
    }
    let mut local = ring(s, &ring(s, &region));
    local.extend(splats.iter().flatten());
    local.sort_unstable();
    local.dedup();
    let mesh = local_mesh(s, &local);
    let geometry = Faces::new(&mesh);
    let to_local = |f: &u32| local.binary_search(f).expect("a face of the part") as u32;
    let view: Vec<u32> = region.iter().map(to_local).collect();
    let lin = linear_table();
    let downscaled: Vec<Vec<Camera>> = sources
        .iter()
        .map(|src| {
            src.cams
                .iter()
                .map(|c| c.camera.downscaled(ccfg.downscale))
                .collect()
        })
        .collect();

    // Per candidate the faces of the Stelle it sees, with their scores.
    let seen: Vec<Vec<(u32, f32)>> = candidates
        .par_iter()
        .zip(&splats)
        .enumerate()
        .map(|(rank, (c, splat))| {
            let (src, cam) = (c.source as usize, c.camera as usize);
            let splat: Vec<u32> = splat.iter().map(to_local).collect();
            let shot = Shot {
                geometry: &downscaled[src][cam],
                full: &sources[src].cams[cam].camera,
                world_from_cam: c.world_from_cam,
                rank: u32::try_from(rank).expect("fewer than 2^32 images"),
                sharpness_norm: 1.0,
                rgb: None,
                min_texel: cfg.min_texel,
            };
            image_views(
                &mesh,
                &geometry,
                &splat,
                &view,
                &shot,
                ccfg,
                |_| false,
                &lin,
            )
            .into_iter()
            .map(|(f, v, _)| (local[f as usize], v.rank_score()))
            .collect()
        })
        .collect();
    drop(splats);

    // The best score of every face of the Stelle, for the quality.
    let mut best = vec![0.0f32; region.len()];
    for (f, score) in seen.iter().flatten() {
        let i = region.binary_search(f).expect("a face of the Stelle");
        best[i] = best[i].max(*score);
    }
    let weights = shape.weights();
    let total: f64 = weights.iter().sum();
    let mut views: Vec<RegionView> = candidates
        .into_iter()
        .zip(seen)
        .filter(|(_, faces)| !faces.is_empty())
        .map(|(c, faces)| {
            let (mut covered, mut quality) = (0.0f64, 0.0f64);
            for (f, score) in &faces {
                let i = region.binary_search(f).expect("a face of the Stelle");
                covered += weights[i];
                quality += weights[i] * f64::from(score / best[i]).sqrt();
            }
            RegionView {
                source: c.source,
                frame: c.frame,
                stamp: c.stamp,
                camera: c.camera,
                world_from_cam: c.world_from_cam,
                coverage: (covered / total) as f32,
                quality: (quality / total) as f32,
                faces,
            }
        })
        .collect();
    views.sort_by(|a, b| {
        let full = |v: &RegionView| v.coverage >= FULL_COVERAGE;
        full(b)
            .cmp(&full(a))
            .then(b.quality.total_cmp(&a.quality))
            .then((a.source, a.frame).cmp(&(b.source, b.frame)))
    });
    Ok(views)
}

/// Decodes the images of the first `k` of `views` (as [`region_views`] gives them: best first)
/// at full size, the bags of each source read once, up to its last such image. Returns them
/// as (index into `views`, image), by index. An image that cannot be decoded or has another
/// size than its camera is left out, so there may be fewer. Progress: phase „Bilder der Stelle
/// lesen“, one step per image; a cancel is checked at every image and returns
/// [`EditError::Cancelled`].
///
/// Fails if the bags cannot be read or no longer hold the images the views were found in.
///
/// # Panics
/// If a view is not of `sources` (its source or camera is not there).
pub(crate) fn decode_views(
    views: &[RegionView],
    sources: &[TextureSource],
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
    for (s, src) in sources.iter().enumerate() {
        // (frame, index into `views`) of the images of this source, in stream order.
        let mut wanted: Vec<(u32, usize)> = chosen
            .iter()
            .enumerate()
            .filter(|(_, v)| v.source as usize == s)
            .map(|(i, v)| (v.frame, i))
            .collect();
        if wanted.is_empty() {
            continue;
        }
        wanted.sort_unstable();
        let (mut next, mut cancelled, mut changed) = (0usize, false, false);
        let mut delivered = 0usize;
        let read = for_each_image_where(
            &src.images.bags,
            &src.images.topics,
            1,
            |h: &ImageHeader| {
                if progress.is_cancelled() {
                    cancelled = true;
                    return Wanted::Stop;
                }
                let Some(&(frame, i)) = wanted.get(next) else {
                    return Wanted::Stop;
                };
                let frame = frame as usize;
                if h.index < frame {
                    return Wanted::No;
                }
                let view = &chosen[i];
                let same = h.index == frame
                    && h.stamp_ns == view.stamp
                    && src.cams[view.camera as usize].camera.name == h.camera;
                if !same {
                    changed = true;
                    return Wanted::Stop;
                }
                next += 1;
                Wanted::Yes
            },
            |frame| {
                let i = wanted[delivered].1;
                delivered += 1;
                progress.advance(1);
                if let Ok(frame) = frame {
                    let cam = &src.cams[chosen[i].camera as usize].camera;
                    if (frame.rgb.width, frame.rgb.height) == (cam.width, cam.height) {
                        decoded.push((i, frame.rgb));
                    }
                }
                !progress.is_cancelled()
            },
        );
        if cancelled || progress.is_cancelled() {
            return Err(EditError::Cancelled);
        }
        read.map_err(|e| MapError::Images(e.to_string()))?;
        if changed || delivered < wanted.len() {
            return Err(MapError::Images(
                "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
            )
            .into());
        }
    }
    decoded.sort_unstable_by_key(|d| d.0);
    Ok(decoded)
}

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

/// The images of `sources` that may see the Stelle `shape`, by their headers and the poses
/// alone (see the module docs), source by source in stream order.
fn candidates(
    shape: &Shape,
    sources: &[TextureSource],
    ccfg: &ColorConfig,
    progress: &Progress,
) -> Result<Vec<Candidate>, EditError> {
    let total = sources
        .iter()
        .map(|src| image_count(&src.images))
        .sum::<Option<u64>>();
    progress.phase(PHASE_FIND, total);
    let mut found = Vec::new();
    for (s, src) in sources.iter().enumerate() {
        let mut cancelled = false;
        let read = for_each_image_where(
            &src.images.bags,
            &src.images.topics,
            1,
            |h| {
                if progress.is_cancelled() {
                    cancelled = true;
                    return Wanted::Stop;
                }
                progress.advance(1);
                let Some(camera) = src.cams.iter().position(|c| c.camera.name == h.camera) else {
                    return Wanted::No;
                };
                let t = h.stamp + ccfg.time_offset;
                let Some(body) = interpolate_pose(&src.poses, t) else {
                    return Wanted::No;
                };
                if angular_rate(&src.poses, t) > ccfg.max_angular_rate {
                    return Wanted::No;
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
                Wanted::No
            },
            |_| true,
        );
        if cancelled {
            return Err(EditError::Cancelled);
        }
        read.map_err(|e| MapError::Images(e.to_string()))?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::testutil::{
        paged, painted_mesh, scene, testscan_config, testscan_source, testscan_surface, walk_source,
    };
    use driftless_export::mesh::Mesh as PageMesh;
    use driftless_io::images::{for_each_image, RgbImage};
    use driftless_io::testdir::SharedDir;
    use driftless_io::testscan::{scan_with_images, shared, IMAGES};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
    use std::sync::OnceLock;

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

    fn views(s: &WeldedSurface, spot: &[u32], sources: &[TextureSource]) -> Vec<RegionView> {
        region_views(s, spot, sources, &testscan_config(), &Progress::new()).unwrap()
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
        let maybe = candidates(&shape, &sources, &cfg.color, &Progress::new()).unwrap();
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
        let decoded = decode_views(&found, &sources, DECODED_VIEWS, &progress).unwrap();
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
            decode_views(&again, &sources, DECODED_VIEWS, &Progress::new()).unwrap();
        assert_eq!(decoded_again, decoded);
        // Fewer views than asked for: all of them; none asked: none.
        assert_eq!(
            decode_views(&found[..3], &sources, DECODED_VIEWS, &Progress::new())
                .unwrap()
                .len(),
            3
        );
        assert!(decode_views(&found, &sources, 0, &Progress::new())
            .unwrap()
            .is_empty());
        // A cancel stops it.
        let cancelled = Progress::new();
        cancelled.cancel();
        assert!(matches!(
            decode_views(&found, &sources, DECODED_VIEWS, &cancelled),
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
        let found = region_views(s, &spot, &sources, &cfg, &Progress::new()).unwrap();
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
}
