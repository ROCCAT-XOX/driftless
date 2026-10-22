//! Which camera images show each face of the mesh, and how well: the input of the texture.
//!
//! Two passes over the images, both decoded at 1/[`SHARPNESS_DOWNSCALE`] (the views need only
//! the geometry and a colour sample for the exposure):
//!
//! 1. **Quality**, as for colouring (see [`crate::color`]): sharpness and angular rate decide
//!    which images are used; the used ones are ranked in (capture time, camera name) order.
//! 2. **Views**: per used image a depth buffer of the faces near the camera (cells of
//!    [`ColorConfig::zbuffer_scale`] pixels of the image at 1/[`ColorConfig::downscale`]),
//!    splatted from the centroid and the corners of every face. A face that would cover more
//!    than [`SAMPLE_GAP_CELLS`] cells between its samples gets a barycentric grid of up to
//!    [`MAX_SUBDIVISIONS`] rows instead, so that nothing behind it shows through. The samples
//!    then lie at most [`SAMPLE_GAP_CELLS`] cells apart as long as the face's nearest point is
//!    at least `longest edge · focal / (SAMPLE_GAP_CELLS · MAX_SUBDIVISIONS)` away (`focal` in
//!    cells per radian; with [`SurfaceConfig::max_edge`] 0.3 m, a lens of about 600 px/rad and
//!    `zbuffer_scale` 4: about 0.7 m); nearer, the grid is capped and gaps can open. A face is
//!    seen if its centroid lies within [`ColorConfig::max_dist`], its centroid and all three
//!    corners pass the depth buffer (a partly hidden face is not seen), its normal points
//!    towards the camera and its corners lie in the image within [`ColorConfig::rim_theta`] of
//!    the optical axis. Seen at a grazing angle (beyond about 75°), a face is often hidden by
//!    the nearer part of its own wall in the same depth cell, as in colouring. Its score is
//!    `projected area (pixels at full size) × sharpness_norm × cos(angle to the normal)`.
//!    Each face keeps its [`SurfaceConfig::views_per_face`] best views (higher score first;
//!    equal scores: the earlier image).
//!
//! **The carrier** of the device stands next to it and below the camera: a view from a camera
//! less than [`CARRIER_RADIUS`] away horizontally (world Z up) of a face's centroid that lies
//! more than [`CARRIER_BELOW`] below the camera may show the carrier's legs or shoes instead of
//! the floor. Such a view is tagged ([`FaceView::carrier`]) and ranks behind every untagged
//! view of the face; after the pass ([`filter_views`]) it is dropped if the face has an
//! untagged one, so the floor along the walking path is textured from farther away.
//!
//! Exposure and white balance: every `k`-th face (at most [`EXPOSURE_FACES`]) also keeps the
//! colour (bilinear, linear RGB, at its centroid) and the angle from the optical axis of its
//! kept views; from these, [`crate::exposure::estimate_exposure`] fits the gains and the edge
//! falloff as in colouring. Unlike colouring (which fits on [`ColorConfig::views_kept`] views
//! per point, at most one per camera and [`ColorConfig::view_gap`]), the fit sees the
//! [`SurfaceConfig::views_per_face`] best views by score.

use super::{Mesh, SurfaceConfig};
use crate::color::{
    frame_camera, keep_best, linear_table, quality_pass, sample_linear, ColorCamera, ColorConfig,
    ImageSource, Status, View, SHARPNESS_DOWNSCALE,
};
use crate::exposure::{estimate_exposure, sample_observations, Vignetting};
use crate::visibility::{project_point, DepthBuffer, PointGrid};
use crate::MapError;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, ImageError, RgbImage};
use driftless_io::progress::Progress;
use driftless_traj::{interpolate_pose, Pose};
use nalgebra::{Isometry3, Point3, Vector3};
use rayon::prelude::*;

/// At most this many faces (every `k`-th) keep the colours of their views for the exposure fit.
pub const EXPOSURE_FACES: usize = 200_000;
/// The depth-buffer samples of a face are meant to lie at most this many cells apart in the
/// image (see the module docs for when the cap [`MAX_SUBDIVISIONS`] prevents it).
pub const SAMPLE_GAP_CELLS: f64 = 2.0;
/// A face is split into at most this many sample rows for the depth buffer.
pub const MAX_SUBDIVISIONS: usize = 32;
/// A view of a face whose centroid lies within this horizontal distance (m) of the camera …
pub const CARRIER_RADIUS: f64 = 1.2;
/// … and more than this far below it (m) may show the carrier of the device (see the module
/// docs).
pub const CARRIER_BELOW: f64 = 0.5;
/// [`FaceView::tag`] bit of a view that may show the carrier.
const TAG_CARRIER: u8 = 0x80;

/// The camera images of one scan with its trajectory and cameras.
#[derive(Debug, Clone, PartialEq)]
pub struct TextureSource {
    pub images: ImageSource,
    /// Body trajectory (e.g. the SLAM `traj.tum`).
    pub poses: Vec<Pose>,
    /// Every camera with its mount.
    pub cams: Vec<ColorCamera>,
}

/// One view of a face; `score == 0` marks an empty slot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FaceView {
    /// Rank of the image: index into [`ViewTable::images`].
    pub image: u32,
    pub score: f32,
    /// Bit 7 ([`FaceView::carrier`]): the view may show the carrier of the device.
    pub tag: u8,
}

impl FaceView {
    pub(crate) const EMPTY: FaceView = FaceView {
        image: u32::MAX,
        score: 0.0,
        tag: 0,
    };

    /// Whether the view may show the carrier of the device (see the module docs).
    pub(crate) fn carrier(&self) -> bool {
        self.tag & TAG_CARRIER != 0
    }

    /// A view before an empty slot; an untagged view before a [`FaceView::carrier`] one; then
    /// the higher score; equal scores: the earlier image.
    fn better_than(&self, other: &FaceView) -> bool {
        if (self.score > 0.0) != (other.score > 0.0) {
            return self.score > 0.0;
        }
        if self.carrier() != other.carrier() {
            return !self.carrier();
        }
        self.score > other.score || (self.score == other.score && self.image < other.image)
    }
}

/// Inserts `v` into a face's views (best first) if it is among the best.
fn keep_best_view(slots: &mut [FaceView], v: FaceView) {
    if let Some(j) = slots.iter().position(|s| v.better_than(s)) {
        slots[j..].rotate_right(1);
        slots[j] = v;
    }
}

/// A used image.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct UsedImage {
    /// Index into [`TextureSource::cams`].
    pub camera: usize,
    /// Image stamp (s), as in the bag.
    pub stamp: f64,
    /// Camera pose in the world: `trajectory(stamp + time_offset) · cam_in_body`.
    pub world_from_cam: Isometry3<f64>,
}

/// The views of every face.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ViewTable {
    /// The used images by rank: (capture time, camera name) order.
    pub images: Vec<UsedImage>,
    /// [`SurfaceConfig::views_per_face`] views per face, best first; empty slots (score 0)
    /// at the end.
    pub per_face: Vec<FaceView>,
    /// Exposure and white-balance gain of every used image (by rank, linear RGB): divide an
    /// image's colours by it.
    pub gains: Vec<[f32; 3]>,
    /// Edge falloff of every camera (numbered as [`TextureSource::cams`]).
    pub vignetting: Vignetting,
}

impl ViewTable {
    /// Rank of the image of `camera` stamped `stamp`, if it is used.
    pub(crate) fn rank_of(&self, camera: usize, stamp: f64) -> Option<u32> {
        let start = self.images.partition_point(|i| i.stamp < stamp);
        self.images[start..]
            .iter()
            .take_while(|i| i.stamp == stamp)
            .position(|i| i.camera == camera)
            .map(|j| u32::try_from(start + j).expect("fewer than 2^32 images"))
    }
}

/// Per face: centroid, unit normal and longest edge; and the largest distance from a centroid
/// to a corner.
struct Faces {
    centroid: Vec<[f32; 3]>,
    normal: Vec<[f32; 3]>,
    longest: Vec<f32>,
    reach: f64,
}

impl Faces {
    fn new(mesh: &Mesh) -> Faces {
        let corners = |f: usize| mesh.indices[f].map(|i| Vector3::from(mesh.positions[i as usize]));
        let (centroid, longest): (Vec<[f32; 3]>, Vec<f32>) = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| {
                let [a, b, c] = corners(f);
                let m = (a + b + c) / 3.0;
                let edge = (b - a).norm().max((c - b).norm()).max((a - c).norm());
                ([m.x, m.y, m.z], edge)
            })
            .unzip();
        let reach = (0..mesh.indices.len())
            .into_par_iter()
            .map(|f| {
                let m = Vector3::from(centroid[f]);
                corners(f)
                    .iter()
                    .map(|c| f64::from((c - m).norm()))
                    .fold(0.0, f64::max)
            })
            .reduce(|| 0.0, f64::max);
        Faces {
            centroid,
            normal: (0..mesh.indices.len())
                .into_par_iter()
                .map(|f| mesh.face_normal(f))
                .collect(),
            longest,
            reach,
        }
    }
}

/// What one image contributes: per seen face its view, and for an exposure sample face also the
/// view with its colour.
type Found = Vec<(u32, FaceView, Option<View>)>;

/// One used image, ready to be looked through.
struct Shot<'a> {
    /// Camera at 1/[`ColorConfig::downscale`] (depth buffer) and at full size (score).
    geometry: &'a Camera,
    full: &'a Camera,
    world_from_cam: Isometry3<f64>,
    rank: u32,
    sharpness_norm: f32,
    /// The image at 1/[`SHARPNESS_DOWNSCALE`].
    rgb: &'a RgbImage,
}

/// The faces `shot` sees, in face order.
fn image_views(
    mesh: &Mesh,
    faces: &Faces,
    grid: &PointGrid,
    shot: &Shot,
    cfg: &ColorConfig,
    sampled: impl Fn(usize) -> bool + Sync,
    lin: &[f32; 256],
) -> Found {
    let world_to_cam = shot.world_from_cam.inverse();
    let cam_pos = shot.world_from_cam.translation.vector;
    let mut candidates = grid.near(&cam_pos, cfg.max_dist + faces.reach);
    candidates.par_sort_unstable();
    let cam = shot.geometry;
    let buffer = DepthBuffer::new(cam, cfg.zbuffer_scale);
    // Cells per radian, as the depth buffer sizes its splats.
    let focal = cam.fx.max(cam.fy) / f64::from(cfg.zbuffer_scale);
    candidates.par_iter().for_each(|&f| {
        let f = f as usize;
        let [a, b, c] = mesh.indices[f].map(|i| Vector3::from(mesh.positions[i as usize]));
        let longest = faces.longest[f];
        // No point of the face is nearer than this.
        let near = [a, b, c]
            .iter()
            .map(|p| (p.cast::<f64>() - cam_pos).norm())
            .fold(f64::INFINITY, f64::min)
            - f64::from(longest);
        let n = (f64::from(longest) * focal / (near.max(1e-3) * SAMPLE_GAP_CELLS)).ceil();
        let n = (n as usize).clamp(1, MAX_SUBDIVISIONS);
        let spacing = longest / n as f32;
        let splat = |p: Vector3<f32>| {
            if let Some(proj) = project_point(&[p.x, p.y, p.z], cam, &world_to_cam, cfg.max_dist) {
                buffer.splat(&proj, spacing);
            }
        };
        splat(Vector3::from(faces.centroid[f]));
        for i in 0..=n {
            for j in 0..=n - i {
                let (u, v) = (i as f32 / n as f32, j as f32 / n as f32);
                splat(a + (b - a) * u + (c - a) * v);
            }
        }
    });

    // Image coordinates at 1/`downscale` → at 1/SHARPNESS_DOWNSCALE (pixel centres at integers).
    let to_colour = f64::from(cfg.downscale) / f64::from(SHARPNESS_DOWNSCALE);
    candidates
        .par_iter()
        .filter_map(|&f| {
            let f = f as usize;
            let centre = faces.centroid[f];
            let proj = project_point(&centre, cam, &world_to_cam, cfg.max_dist)?;
            if f64::from(proj.theta) >= cfg.rim_theta {
                return None;
            }
            let to_cam = cam_pos - Vector3::from(centre).cast::<f64>();
            let cos = Vector3::from(faces.normal[f]).cast::<f64>().dot(&to_cam) / to_cam.norm();
            if cos.is_nan() || cos <= 0.0 || !buffer.is_visible(&proj, cfg.depth_tolerance) {
                return None;
            }
            let mut uv = [[0.0f64; 2]; 3];
            for (uv, &i) in uv.iter_mut().zip(&mesh.indices[f]) {
                let p = Vector3::from(mesh.positions[i as usize]).cast::<f64>();
                let pc = world_to_cam * Point3::from(p);
                if pc.x.hypot(pc.y).atan2(pc.z) >= cfg.rim_theta {
                    return None;
                }
                *uv = shot.full.project(&pc.coords)?;
                // The corner must not be hidden either (same tolerance as the centroid).
                let corner = project_point(
                    &mesh.positions[i as usize],
                    cam,
                    &world_to_cam,
                    f64::INFINITY,
                )?;
                if !buffer.is_visible(&corner, cfg.depth_tolerance) {
                    return None;
                }
            }
            let area = 0.5
                * ((uv[1][0] - uv[0][0]) * (uv[2][1] - uv[0][1])
                    - (uv[2][0] - uv[0][0]) * (uv[1][1] - uv[0][1]))
                    .abs();
            let score = (area * cos) as f32 * shot.sharpness_norm;
            if score.is_nan() || score <= 0.0 {
                return None;
            }
            let below = to_cam.z;
            let carrier = to_cam.x.hypot(to_cam.y) < CARRIER_RADIUS && below > CARRIER_BELOW;
            // The carrier does not belong in the exposure fit either.
            let colour = (sampled(f) && !carrier).then(|| {
                let at = proj
                    .uv
                    .map(|x| ((f64::from(x) + 0.5) * to_colour - 0.5) as f32);
                View::new(
                    score,
                    shot.rank,
                    sample_linear(shot.rgb, at, lin),
                    proj.theta,
                )
            });
            let view = FaceView {
                image: shot.rank,
                score,
                tag: if carrier { TAG_CARRIER } else { 0 },
            };
            Some((f as u32, view, colour))
        })
        .collect()
}

/// Drops the views of a face that may show the carrier (see the module docs) if it has
/// another; `per_face` holds `k` slots per face, as [`keep_best_view`] fills them.
pub(crate) fn filter_views(per_face: &mut [FaceView], k: usize) {
    per_face.par_chunks_mut(k).for_each(|slots| {
        // Tagged views rank behind every untagged one.
        if slots[0].score > 0.0 && !slots[0].carrier() {
            for s in slots.iter_mut().filter(|s| s.carrier()) {
                *s = FaceView::EMPTY;
            }
        }
    });
}

/// The views of every face of `mesh` in the images of `src`, and the exposure of the images
/// (see the module docs). Progress: phase „Bilder prüfen“ (the quality pass) and „Ansichten
/// suchen“, one step per image of the stream; a cancel is checked at every image and returns
/// [`MapError::Cancelled`]. Same input, same result.
///
/// Fails if `cfg` is invalid, two cameras share a name or the images cannot be read.
pub(crate) fn collect_views(
    mesh: &Mesh,
    src: &TextureSource,
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<ViewTable, MapError> {
    if progress.is_cancelled() {
        return Err(MapError::Cancelled);
    }
    cfg.check()?;
    let (cams, poses, ccfg) = (&src.cams, &src.poses, &cfg.color);
    for (i, c) in cams.iter().enumerate() {
        if cams[..i].iter().any(|o| o.camera.name == c.camera.name) {
            return Err(MapError::Config(format!(
                "Kamera {} ist doppelt angegeben",
                c.camera.name
            )));
        }
    }
    let (records, _, _) = quality_pass(&src.images, poses, cams, ccfg, progress)?;

    // The used images by rank, with their world poses.
    let mut images: Vec<Option<UsedImage>> = Vec::new();
    for r in &records {
        if let Status::Used { rank, .. } = r.status {
            let rank = rank as usize;
            if images.len() <= rank {
                images.resize(rank + 1, None);
            }
            let body = interpolate_pose(poses, r.stamp + ccfg.time_offset)
                .expect("checked in the quality pass");
            images[rank] = Some(UsedImage {
                camera: r.camera,
                stamp: r.stamp,
                world_from_cam: body * cams[r.camera].cam_in_body,
            });
        }
    }
    let images: Vec<UsedImage> = images
        .into_iter()
        .map(|i| i.expect("ranks are dense"))
        .collect();

    let k = cfg.views_per_face;
    let face_count = mesh.indices.len();
    let mut per_face = vec![FaceView::EMPTY; face_count * k];
    // Exposure samples: every `stride`-th face, `slot = face / stride`.
    let stride = face_count.div_ceil(EXPOSURE_FACES).max(1);
    let sample_faces = if ccfg.exposure.samples > 0 {
        face_count.div_ceil(stride)
    } else {
        0
    };
    let empty = View::new(0.0, u32::MAX, [0.0; 3], 0.0);
    let mut samples = vec![empty; sample_faces * k];

    let last_used = records
        .iter()
        .rposition(|r| matches!(r.status, Status::Used { .. }));
    let walked = match (last_used, face_count) {
        (Some(last_used), 1..) => last_used as u64 + 1,
        _ => 0,
    };
    progress.phase("Ansichten suchen", Some(walked));
    if let (Some(last_used), 1..) = (last_used, face_count) {
        let faces = Faces::new(mesh);
        let grid = PointGrid::new(&faces.centroid);
        let lin = linear_table();
        let geometry: Vec<Camera> = cams
            .iter()
            .map(|c| c.camera.downscaled(ccfg.downscale))
            .collect();
        let sampled = |f: usize| sample_faces > 0 && f.is_multiple_of(stride);
        let mut index = 0usize;
        let mut failure: Option<MapError> = None;
        let mut one = |frame: Result<Frame, ImageError>| {
            let i = index;
            index += 1;
            let Some(&crate::color::ImageRecord {
                camera,
                stamp,
                status:
                    Status::Used {
                        rank,
                        sharpness_norm,
                    },
            }) = records.get(i)
            else {
                return i < last_used;
            };
            let frame = match frame {
                Ok(f)
                    if f.stamp == stamp
                        && frame_camera(&f, cams, SHARPNESS_DOWNSCALE) == Some(camera) =>
                {
                    f
                }
                _ => {
                    failure = Some(MapError::Images(
                        "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
                    ));
                    return false;
                }
            };
            let shot = Shot {
                geometry: &geometry[camera],
                full: &cams[camera].camera,
                world_from_cam: images[rank as usize].world_from_cam,
                rank,
                sharpness_norm,
                rgb: &frame.rgb,
            };
            for (f, view, colour) in image_views(mesh, &faces, &grid, &shot, ccfg, sampled, &lin) {
                let f = f as usize;
                keep_best_view(&mut per_face[f * k..(f + 1) * k], view);
                if let Some(v) = colour {
                    let s = f / stride;
                    keep_best(&mut samples[s * k..(s + 1) * k], v);
                }
            }
            i < last_used
        };
        let mut cancelled = false;
        let read = for_each_image(
            &src.images.bags,
            &src.images.topics,
            SHARPNESS_DOWNSCALE,
            |frame| {
                if progress.is_cancelled() {
                    cancelled = true;
                    return false;
                }
                let go = one(frame);
                progress.advance(1);
                go
            },
        );
        if cancelled {
            return Err(MapError::Cancelled);
        }
        read.map_err(|e| MapError::Images(e.to_string()))?;
        if let Some(e) = failure {
            return Err(e);
        }
    }

    filter_views(&mut per_face, k);

    // Exposure and white balance, fitted on the sample faces' kept views (as `colorize`).
    let cameras: Vec<usize> = images.iter().map(|i| i.camera).collect();
    let (gains, vignetting) = if ccfg.exposure.samples == 0 {
        (
            vec![[1.0; 3]; images.len()],
            Vignetting::flat(cams.len(), ccfg.exposure.max_gain),
        )
    } else {
        let obs = sample_observations(&samples, k, cameras, ccfg.exposure.samples);
        drop(samples);
        let (gains, mut vignetting) = estimate_exposure(&obs, &ccfg.exposure);
        vignetting.coeffs.resize(cams.len(), [[0.0; 2]; 3]);
        (gains, vignetting)
    };
    Ok(ViewTable {
        images,
        per_face,
        gains,
        vignetting,
    })
}

#[cfg(test)]
mod tests {
    use super::super::testutil::scene::{
        aabb, cameras, poses, render, shot, surface_cloud, walk, walk_data, walk_scene, write_bag,
        Painted, Shot, Trajectory, T0,
    };
    use super::super::testutil::{blur, centroid, face_near, painted_mesh, Steps};
    use super::*;
    use crate::{colorize, ColorConfig, ExposureConfig};
    use nalgebra::{UnitQuaternion, Vector3};

    /// Square size of the test meshes (m).
    const STEP: f64 = 0.1;

    fn source(images: &ImageSource, poses: &[Pose], cams: &[ColorCamera]) -> TextureSource {
        TextureSource {
            images: images.clone(),
            poses: poses.to_vec(),
            cams: cams.to_vec(),
        }
    }

    fn views(table: &ViewTable, k: usize, f: usize) -> &[FaceView] {
        &table.per_face[f * k..(f + 1) * k]
    }

    /// Body pose that puts camera `cam` at `world_cam`.
    fn body_for(cam: &ColorCamera, world_cam: Isometry3<f64>) -> Isometry3<f64> {
        world_cam * cam.cam_in_body.inverse()
    }

    #[test]
    fn front_view_beats_grazing() {
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        // A face on the +y wall, seen by the left camera head-on from 1.5 m and at 80° from
        // its normal from 1.5 m.
        let target = Vector3::new(1.0, 5.0, 0.3);
        let f = face_near(&mesh, [1.0, 5.0, 0.3]);
        let rot = cams[0].cam_in_body.rotation;
        let front = Isometry3::from_parts((target - Vector3::new(0.0, 1.5, 0.0)).into(), rot);
        let a = 80f64.to_radians();
        let graze = Isometry3::from_parts(
            (target + 1.5 * Vector3::new(a.sin(), -a.cos(), 0.0)).into(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, a) * rot,
        );
        let traj = Steps(vec![
            (0.0, body_for(&cams[0], front)),
            (T0 + 1.5, body_for(&cams[0], graze)),
        ]);
        let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
        let images = write_bag("views-graze", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let v = views(&table, cfg.views_per_face, f);
        println!("face {f} at {:?}: {v:?}", centroid(&mesh, f));
        // Rank 0 is the frontal image (the earlier one).
        assert_eq!(v[0].image, 0, "{v:?}");
        assert!(v[0].score > 0.0, "{v:?}");
        if let Some(g) = v.iter().find(|x| x.image == 1) {
            assert!(g.score < 0.2 * v[0].score, "{v:?}");
        }
        // At 80° the wall's own nearer stretch in the same depth cell hides the face; with a
        // loose depth tolerance the grazing view comes through and scores far lower
        // (area × cos² 80° ≈ 3 % of the frontal one).
        let loose = SurfaceConfig {
            color: ColorConfig {
                depth_tolerance: 0.5,
                ..cfg.color.clone()
            },
            ..cfg.clone()
        };
        let table = collect_views(&mesh, &src, &loose, &Progress::new()).unwrap();
        let v = views(&table, cfg.views_per_face, f);
        println!("loose: {v:?}");
        assert_eq!(v[0].image, 0, "{v:?}");
        let g = v.iter().find(|x| x.image == 1).expect("grazing view");
        assert!(g.score > 0.0 && g.score < 0.1 * v[0].score, "{v:?}");
    }

    #[test]
    fn occluded_view_dropped() {
        // A pillar between the left camera at the origin and the +y wall; from 2 m to the
        // side the same stretch of wall is in plain view.
        let painted = Painted::new(vec![(
            aabb([-0.2, 2.0, -1.2], [0.2, 2.4, 1.8]),
            [240, 140, 20],
        )]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let traj = Steps(vec![
            (0.0, Isometry3::identity()),
            (T0 + 1.5, Isometry3::translation(2.0, 0.0, 0.0)),
        ]);
        let shots = [shot(0, T0 + 1.0), shot(0, T0 + 2.0)];
        let images = write_bag("views-occluded", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let k = cfg.views_per_face;
        // The wall behind the pillar, seen from the origin: x within ±0.49 m at every height.
        let hidden: Vec<usize> = (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(&mesh, f);
                c[1] > 4.99 && c[0].abs() < 0.25 && (c[2] - 0.05).abs() < 0.6
            })
            .collect();
        assert!(hidden.len() > 20, "{}", hidden.len());
        for &f in &hidden {
            let v = views(&table, k, f);
            assert!(v.iter().all(|x| x.image != 0), "face {f}: {v:?}");
            assert!(v.iter().any(|x| x.image == 1), "face {f}: {v:?}");
        }
        // The pillar's front is seen from the origin.
        let pillar = face_near(&mesh, [0.0, 2.0, 0.05]);
        assert!(views(&table, k, pillar).iter().any(|x| x.image == 0));
    }

    /// Adds triangle `t` to `mesh`, wound so that its normal points along `normal`; returns
    /// its face index.
    fn add_triangle(mesh: &mut Mesh, t: [[f32; 3]; 3], normal: [f32; 3]) -> usize {
        let base = mesh.positions.len() as u32;
        mesh.positions.extend_from_slice(&t);
        let f = mesh.indices.len();
        mesh.indices.push([base, base + 1, base + 2]);
        let n = mesh.face_normal(f);
        if (0..3).map(|k| n[k] * normal[k]).sum::<f32>() < 0.0 {
            mesh.indices[f] = [base, base + 2, base + 1];
        }
        f
    }

    #[test]
    fn partly_hidden_face_rejected() {
        // A pillar between the left camera at the origin and the +y wall hides the wall for
        // x within about −0.29 … 0.22 m; the depth buffer widens that by its splats and the
        // silhouette rule to about ±0.5 m. One extra wall face has a corner deep in the shadow
        // and its centroid in view (x = 0.7); a second one lies wholly in view.
        let painted = Painted::new(vec![(
            aabb([-0.1, 2.0, -1.2], [0.1, 2.2, 1.8]),
            [240, 140, 20],
        )]);
        let cams = cameras();
        let mut mesh = painted_mesh(&painted, STEP);
        let wall = [0.0, -1.0, 0.0];
        let partly = add_triangle(
            &mut mesh,
            [[0.1, 5.0, 0.0], [1.0, 5.0, 0.0], [1.0, 5.0, 0.4]],
            wall,
        );
        let clear = add_triangle(
            &mut mesh,
            [[0.7, 5.0, 0.0], [1.3, 5.0, 0.0], [1.3, 5.0, 0.4]],
            wall,
        );
        let traj = Steps(vec![(0.0, Isometry3::identity())]);
        let images = write_bag("views-partly", &painted, &traj, &cams, &[shot(0, T0 + 1.0)]);
        let src = source(&images, &poses(&traj, 2.0), &cams);
        let cfg = SurfaceConfig::default();
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 1);
        let k = cfg.views_per_face;
        // The centroid alone is in view: the grid face around it is seen …
        let around = face_near(&painted_mesh(&painted, STEP), centroid(&mesh, partly));
        assert!(views(&table, k, around).iter().any(|v| v.image == 0));
        assert!(views(&table, k, clear).iter().any(|v| v.image == 0));
        // … but not the face with the hidden corner.
        let v = views(&table, k, partly);
        assert!(v.iter().all(|x| x.image != 0), "{v:?}");
    }

    #[test]
    fn simplified_wall_gets_views() {
        // A 10 × 2 m stretch of the +y wall as 2 triangles, and the left camera walking along
        // it 1.5 m away. Only views within 50° of the optical axis count (a stand-in for a wall
        // too long for one image): the big triangles fit into no image, their split pieces do.
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mut wall = Mesh::default();
        let n = [0.0, -1.0, 0.0];
        add_triangle(
            &mut wall,
            [[-4.0, 5.0, -0.8], [6.0, 5.0, -0.8], [6.0, 5.0, 1.2]],
            n,
        );
        add_triangle(
            &mut wall,
            [[-4.0, 5.0, -0.8], [6.0, 5.0, 1.2], [-4.0, 5.0, 1.2]],
            n,
        );
        let cam_y = cams[0].cam_in_body.translation.vector.y;
        let xs: Vec<f64> = (0..8).map(|i| -3.5 + 1.4 * f64::from(i)).collect();
        let traj = Steps(
            xs.iter()
                .enumerate()
                .map(|(i, &x)| {
                    let from = if i == 0 { 0.0 } else { T0 + 0.5 + i as f64 };
                    (from, Isometry3::translation(x, 3.5 - cam_y, 0.0))
                })
                .collect(),
        );
        let shots: Vec<Shot> = (0..xs.len())
            .map(|i| shot(0, T0 + 1.0 + i as f64))
            .collect();
        let images = write_bag("views-split", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, xs.len() as f64 + 1.0), &cams);
        let mut cfg = SurfaceConfig::default();
        cfg.color.rim_theta = 50f64.to_radians();
        let seen = |mesh: &Mesh| {
            let table = collect_views(mesh, &src, &cfg, &Progress::new()).unwrap();
            assert_eq!(table.images.len(), xs.len());
            (0..mesh.indices.len())
                .filter(|&f| views(&table, cfg.views_per_face, f)[0].score > 0.0)
                .count()
        };
        assert_eq!(seen(&wall), 0);
        let split = super::super::simplify(&wall, &cfg);
        assert!(split.indices.len() > 100, "{}", split.indices.len());
        assert_eq!(seen(&split), split.indices.len());
    }

    /// The floor faces under "shoes" (a box the mesh does not have) 0.6–1.0 m in front of the
    /// left camera at the origin, 0.75 m above the floor.
    fn under_shoes(mesh: &Mesh) -> Vec<usize> {
        (0..mesh.indices.len())
            .filter(|&f| {
                let c = centroid(mesh, f);
                c[2] < -1.19 && c[0].abs() < 0.15 && c[1] > 0.75 && c[1] < 1.05
            })
            .collect()
    }

    #[test]
    fn carrier_views_left_out() {
        // The carrier's shoes stand on the floor 0.6–1.0 m in front of the camera, which
        // looks down at them from 0.75 m (near and steep: the best view by far). A second image
        // 2.3–2.7 m away sees the same floor without them (the carrier has walked on).
        let painted = Painted::new(vec![]);
        let cams = cameras();
        let mesh = painted_mesh(&painted, STEP);
        let shoes = aabb([-0.25, 0.65, -1.2], [0.25, 1.15, -1.17]);
        let near = Isometry3::translation(0.0, 0.0, -0.5);
        let far = Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, -0.6, 0.5),
            UnitQuaternion::from_euler_angles(-0.6, 0.0, 0.0),
        );
        let traj = Steps(vec![(0.0, near), (T0 + 1.5, far)]);
        let cam = cams[0].clone();
        let with_shoes = move |_| {
            let scene = Painted::new(vec![(shoes, [150, 90, 40])]);
            render(&scene, &cam.camera, &(near * cam.cam_in_body))
        };
        let shots = [
            Shot {
                camera: 0,
                stamp: T0 + 1.0,
                edit: Box::new(with_shoes.clone()),
            },
            shot(0, T0 + 2.0),
        ];
        let images = write_bag("views-carrier", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 3.0), &cams);
        let cfg = SurfaceConfig::default();
        let k = cfg.views_per_face;
        let floor = under_shoes(&mesh);
        assert!(floor.len() > 10, "{}", floor.len());
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 2, "{:?}", table.images);
        let labels = crate::surface::atlas::choose_labels(&mesh, &table, &cfg);
        for &f in &floor {
            let v = views(&table, k, f);
            assert!(v.iter().all(|x| x.image != 0), "face {f}: {v:?}");
            assert!(v.iter().any(|x| x.image == 1), "face {f}: {v:?}");
            assert_eq!(labels[f], Some(1), "face {f}");
        }
        // The wall ahead is far from the carrier: both images keep it.
        let wall = face_near(&mesh, [0.0, 5.0, -0.5]);
        let v = views(&table, k, wall);
        assert!(
            v.iter().any(|x| x.image == 0) && v.iter().any(|x| x.image == 1),
            "{v:?}"
        );
        // Seen from near only, the floor keeps that view: better than none.
        let only = write_bag(
            "views-carrier-only",
            &painted,
            &traj,
            &cams,
            &[Shot {
                camera: 0,
                stamp: T0 + 1.0,
                edit: Box::new(with_shoes),
            }],
        );
        let src = source(&only, &poses(&traj, 3.0), &cams);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        for &f in &floor {
            assert_eq!(views(&table, k, f)[0].image, 0, "face {f}");
        }
    }

    #[test]
    fn blurry_image_never_chosen() {
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let mut shots: Vec<Shot> = (1..=11).map(|i| shot(0, T0 + 0.5 * f64::from(i))).collect();
        let blurry = T0 + 2.25;
        shots.push(Shot {
            camera: 0,
            stamp: blurry,
            edit: Box::new(|rgb| blur(&rgb, 6.0)),
        });
        let images = write_bag("views-blurry", &painted, &traj, &cams, &shots);
        let src = source(&images, &poses(&traj, 6.0), &cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(&painted, STEP);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.images.len(), 11, "{:?}", table.images);
        assert!(table.images.iter().all(|i| i.stamp != blurry));
        let seen = table.per_face.iter().filter(|v| v.score > 0.0);
        assert!(seen.clone().count() > mesh.indices.len() / 4);
        assert!(seen.clone().all(|v| (v.image as usize) < 11));
        // Every used image is someone's view, and ranks follow the capture time.
        for r in 0..11u32 {
            assert!(seen.clone().any(|v| v.image == r), "image {r} unused");
        }
        assert!(table.images.windows(2).all(|w| w[0].stamp < w[1].stamp));
    }

    #[test]
    fn gains_match_colorize() {
        // Every second stamp 1.2 times brighter (linear).
        let painted = walk_scene();
        let cams = cameras();
        let traj = walk();
        let shots: Vec<Shot> = (1..=11)
            .flat_map(|i| {
                let g = if i % 2 == 0 { 1.2 } else { 1.0 };
                [0, 1].map(move |c| Shot {
                    camera: c,
                    stamp: T0 + 0.5 * f64::from(i),
                    edit: Box::new(move |rgb: Vec<u8>| {
                        rgb.into_iter()
                            .map(|v| {
                                let l = (f64::from(v) / 255.0).powf(2.2) * g;
                                (l.min(1.0).powf(1.0 / 2.2) * 255.0).round() as u8
                            })
                            .collect()
                    }),
                })
            })
            .collect();
        let images = write_bag("views-gains", &painted, &traj, &cams, &shots);
        let poses = poses(&traj, 6.0);
        let mut cloud = surface_cloud(&painted);
        let stats = colorize(
            &mut cloud,
            images.clone(),
            &poses,
            &cams,
            &ColorConfig::default(),
        )
        .unwrap();
        let src = source(&images, &poses, &cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(&painted, STEP);
        let table = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(table.gains.len(), stats.gains.len());
        assert_eq!(table.vignetting.coeffs.len(), cams.len());
        let mut spread = (f32::INFINITY, 0.0f32);
        for (ours, theirs) in table.gains.iter().zip(&stats.gains) {
            for c in 0..3 {
                let r = ours[c] / theirs.gain[c];
                assert!((r - 1.0).abs() < 0.02, "{ours:?} vs {theirs:?}");
                spread = (spread.0.min(ours[c]), spread.1.max(ours[c]));
            }
        }
        // The gains differ between the images, so the comparison means something.
        assert!(spread.1 / spread.0 > 1.1, "{spread:?}");
        // Without equalisation every gain is 1.
        let off = SurfaceConfig {
            color: ColorConfig {
                exposure: ExposureConfig {
                    samples: 0,
                    ..ExposureConfig::default()
                },
                ..cfg.color.clone()
            },
            ..cfg.clone()
        };
        let flat = collect_views(&mesh, &src, &off, &Progress::new()).unwrap();
        assert!(flat.gains.iter().all(|g| *g == [1.0; 3]));
        assert_eq!(flat.per_face, table.per_face);
    }

    #[test]
    fn views_deterministic_and_cancellable() {
        let (painted, images, poses, cams) = walk_data();
        let src = source(images, poses, cams);
        let cfg = SurfaceConfig::default();
        let mesh = painted_mesh(painted, STEP);
        let a = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        let b = collect_views(&mesh, &src, &cfg, &Progress::new()).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.per_face.len(), mesh.indices.len() * cfg.views_per_face);
        // Each image's pose is the trajectory's at its capture time.
        let offset = cfg.color.time_offset;
        for (r, i) in a.images.iter().enumerate() {
            assert_eq!(a.rank_of(i.camera, i.stamp), Some(r as u32));
            let want = walk().pose(i.stamp + offset) * cams[i.camera].cam_in_body;
            let d = (i.world_from_cam.translation.vector - want.translation.vector).norm();
            assert!(d < 0.01, "{d}");
        }
        assert_eq!(a.rank_of(0, T0 + 0.25), None);
        for phase in ["Bilder prüfen", "Ansichten suchen"] {
            let p = Progress::new();
            let q = p.clone();
            let p = p.with_observer(move |s| {
                if s.phase == phase && s.done == 3 {
                    q.cancel();
                }
            });
            let r = collect_views(&mesh, &src, &cfg, &p);
            assert!(matches!(r, Err(MapError::Cancelled)), "{phase}: {r:?}");
            assert_eq!(p.snapshot().phase, phase);
        }
    }
}
