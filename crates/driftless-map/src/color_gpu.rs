//! The colour pass of [`crate::colorize_with`] on the GPU.
//!
//! Per used image the GPU projects every point, builds the depth buffer, tests visibility,
//! weighs the view like [`view_weight`], reads the colour from the uploaded image like
//! [`crate::color::sample_linear`] and keeps the best [`ColorConfig::views_kept`] views per point
//! like [`keep_best_apart`] (one invocation per point, images in stream order: no atomics,
//! the same views as the CPU rule on equal weights, ties broken by rank). The images are
//! decoded on the CPU in a thread of their own, up to [`LOOKAHEAD`] images ahead of the GPU.
//! After the last image the slots come back to the CPU; exposure, blend and statistics stay
//! there, unchanged.

use crate::color::{
    frame_camera, linear_table, ColorCamera, ColorConfig, ImageRecord, ImageSource, Status, View,
    FULL_WEIGHT_THETA,
};
#[cfg(doc)]
use crate::color::{keep_best_apart, view_weight};
use crate::visibility::{DEPTH_TOLERANCE_REL, MAX_SPLAT, SILHOUETTE_REL};
use crate::MapError;
use driftless_gpu::{
    guarded, views_batch_points, views_bytes, GpuCamera, GpuCloud, GpuContext, GpuRunError,
    ImageViews, ViewImage, ViewParams, ViewSlot, VisParams,
};
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, ImageError, RgbImage};
use driftless_io::progress::Progress;
use driftless_traj::{interpolate_pose, Pose};
use nalgebra::{Isometry3, Vector3};
use std::collections::BTreeMap;
use std::sync::mpsc::sync_channel;

/// Images decoded ahead of the GPU.
pub const LOOKAHEAD: usize = 2;

/// What the GPU colour pass needs (the state of [`crate::colorize_with`] after the quality
/// pass).
pub(crate) struct GpuInput<'a> {
    pub(crate) ctx: &'a GpuContext,
    /// Points per batch on the GPU; `None`: as large as the device allows.
    pub(crate) batch_points: Option<usize>,
    pub(crate) xyz: &'a [[f32; 3]],
    pub(crate) normals: &'a [Option<Vector3<f32>>],
    pub(crate) spacing: &'a [f32],
    pub(crate) images: &'a ImageSource,
    pub(crate) records: &'a [ImageRecord],
    /// Stream index of the last used image.
    pub(crate) last_used: usize,
    /// Capture time and camera of every used image, by rank.
    pub(crate) by_rank: &'a [(f64, usize)],
    pub(crate) poses: &'a [Pose],
    pub(crate) cams: &'a [ColorCamera],
    pub(crate) cfg: &'a ColorConfig,
}

/// Visibility settings of `cfg` for the GPU.
pub(crate) fn vis_params(cfg: &ColorConfig) -> VisParams {
    VisParams {
        max_dist: cfg.max_dist,
        depth_tolerance: cfg.depth_tolerance,
        zbuffer_scale: cfg.zbuffer_scale,
        depth_tolerance_rel: DEPTH_TOLERANCE_REL,
        silhouette_rel: SILHOUETTE_REL,
        max_splat: MAX_SPLAT,
    }
}

/// Per rank the id of the image's (camera, ⌊t / gap⌋) stretch, as `keep_best_apart` compares
/// them; `None` for `gap = 0` (no limit).
pub(crate) fn stretch_ids(by_rank: &[(f64, usize)], gap: f64) -> Option<Vec<u32>> {
    if gap <= 0.0 {
        return None;
    }
    let mut ids: BTreeMap<(usize, u64), u32> = BTreeMap::new();
    Some(
        by_rank
            .iter()
            .map(|&(t, cam)| {
                // `floor` gives whole numbers; + 0.0 turns −0 into 0, which `==` equates.
                let stretch = (t / gap).floor() + 0.0;
                let next = ids.len() as u32;
                *ids.entry((cam, stretch.to_bits())).or_insert(next)
            })
            .collect(),
    )
}

/// The view selection of one cloud on the GPU.
pub(crate) struct Accumulator<'a> {
    views: ImageViews<'a>,
}

impl<'a> Accumulator<'a> {
    pub(crate) fn new(
        ctx: &'a GpuContext,
        cloud: &'a GpuCloud,
        cfg: &ColorConfig,
        by_rank: &[(f64, usize)],
    ) -> Accumulator<'a> {
        let params = ViewParams {
            vis: vis_params(cfg),
            views_kept: cfg.views_kept,
            full_weight_theta: FULL_WEIGHT_THETA,
            stretch: stretch_ids(by_rank, cfg.view_gap),
        };
        Accumulator {
            views: ImageViews::new(ctx, cloud, &linear_table(), params),
        }
    }

    /// Adds the views of image `rank` (decoded at the size of `cam`) at pose `world_cam`.
    pub(crate) fn add(
        &mut self,
        cam: &Camera,
        world_cam: &Isometry3<f64>,
        rgb: &RgbImage,
        rank: u32,
        sharpness_norm: f32,
    ) -> Result<(), GpuRunError> {
        self.views.add_image(&ViewImage {
            cam: GpuCamera::from_camera(cam),
            world_from_cam: *world_cam,
            rgb: &rgb.data,
            rank,
            sharpness: sharpness_norm,
        })
    }

    /// The kept views, `views_kept` per point, best first.
    /// The slots are converted chunk by chunk while they are read back: the CPU holds the
    /// result and one chunk, not a second copy of all slots.
    pub(crate) fn finish(&self) -> Result<Vec<View>, GpuRunError> {
        self.views.read(|s: ViewSlot| View {
            weight: s.weight,
            rank: s.rank,
            rgb: s.rgb,
            theta: s.theta,
        })
    }

    /// Checks the device before the first image.
    pub(crate) fn probe(&mut self) -> Result<(), GpuRunError> {
        self.views.probe()
    }
}

/// Why [`gpu_slots`] gave no views.
#[derive(Debug)]
pub(crate) enum GpuStop {
    /// The GPU could not start (`--gpu auto`): colour on the CPU, with this reason (German).
    Fallback(String),
    /// Cancelled, images changed, or the GPU failed (`--gpu an`, or during the run).
    Fail(MapError),
}

impl From<MapError> for GpuStop {
    fn from(e: MapError) -> GpuStop {
        GpuStop::Fail(e)
    }
}

/// The colour pass on the GPU: the kept views of every point, as the CPU loop of
/// [`crate::colorize_with`] leaves them.
///
/// Before the first image the memory is checked against the context's budget and the
/// allocations, uploads and a first dispatch run inside error scopes; a failure there is
/// [`GpuStop::Fallback`] if the context falls back (`--gpu auto`), else an error. A failure
/// during the run (device lost, readback) is always a [`MapError::Gpu`], never a panic.
pub(crate) fn gpu_slots(input: &GpuInput<'_>, progress: &Progress) -> Result<Vec<View>, GpuStop> {
    let GpuInput {
        ctx,
        batch_points,
        xyz,
        normals,
        spacing,
        images,
        records,
        last_used,
        by_rank,
        poses,
        cams,
        cfg,
    } = *input;
    let scaled: Vec<Camera> = cams
        .iter()
        .map(|c| c.camera.downscaled(cfg.downscale))
        .collect();
    let start = |e: GpuRunError| {
        if ctx.falls_back() {
            GpuStop::Fallback(e.to_string())
        } else {
            GpuStop::Fail(MapError::Gpu(e.to_string()))
        }
    };
    let image_bytes = scaled
        .iter()
        .map(|c| c.width as usize * c.height as usize * 3);
    let zs = cfg.zbuffer_scale;
    let depth_cells = scaled
        .iter()
        .map(|c| c.width.div_ceil(zs) as usize * c.height.div_ceil(zs) as usize);
    let need = views_bytes(
        xyz.len(),
        cfg.views_kept,
        image_bytes.max().unwrap_or(0),
        depth_cells.max().unwrap_or(0),
    );
    ctx.check_budget(need).map_err(start)?;
    let batch = batch_points.unwrap_or_else(|| views_batch_points(ctx, cfg.views_kept));
    let cloud = guarded(ctx, || {
        GpuCloud::with_batch_points(ctx, xyz, normals, spacing, batch)
    })
    .and_then(|cloud| cloud)
    .map_err(start)?;
    let mut acc = guarded(ctx, || Accumulator::new(ctx, &cloud, cfg, by_rank)).map_err(start)?;
    acc.probe().map_err(start)?;

    let mut failure: Option<MapError> = None;
    let mut cancelled = false;
    let read = std::thread::scope(|scope| {
        // Decoding runs ahead of the GPU by up to `LOOKAHEAD` images; it stops after the last
        // used image, at a cancel or when the GPU side stops listening.
        let (tx, rx) = sync_channel::<Result<Frame, ImageError>>(LOOKAHEAD);
        let reader = scope.spawn(move || {
            let mut index = 0usize;
            for_each_image(&images.bags, &images.topics, cfg.downscale, |frame| {
                if progress.is_cancelled() {
                    return false;
                }
                let more = index < last_used;
                index += 1;
                tx.send(frame).is_ok() && more
            })
        });
        for (i, frame) in rx.iter().enumerate() {
            if progress.is_cancelled() {
                cancelled = true;
                break;
            }
            if let Some(ImageRecord {
                camera,
                stamp,
                status:
                    Status::Used {
                        rank,
                        sharpness_norm,
                    },
            }) = records.get(i).copied()
            {
                match frame {
                    Ok(f)
                        if f.stamp == stamp
                            && frame_camera(&f, cams, cfg.downscale) == Some(camera) =>
                    {
                        let body = interpolate_pose(poses, stamp + cfg.time_offset)
                            .expect("checked in the quality pass");
                        let world_cam = body * cams[camera].cam_in_body;
                        let added =
                            acc.add(&scaled[camera], &world_cam, &f.rgb, rank, sharpness_norm);
                        if let Err(e) = added {
                            failure = Some(MapError::Gpu(e.to_string()));
                            break;
                        }
                    }
                    _ => {
                        failure = Some(MapError::Images(
                            "Bilder haben sich zwischen zwei Durchläufen geändert".into(),
                        ));
                        break;
                    }
                }
            }
            progress.advance(1);
        }
        drop(rx);
        reader.join().expect("Bilder-Thread abgestürzt")
    });
    if cancelled || progress.is_cancelled() {
        return Err(MapError::Cancelled.into());
    }
    read.map_err(|e| MapError::Images(e.to_string()))?;
    if let Some(e) = failure {
        return Err(e.into());
    }
    acc.finish()
        .map_err(|e| GpuStop::Fail(MapError::Gpu(e.to_string())))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::{keep_best_apart, sample_linear, view_weight, EMPTY};
    use crate::visibility::visible_points;
    use driftless_io::calib::CameraModel;

    fn pinhole() -> Camera {
        Camera {
            name: "test".into(),
            width: 64,
            height: 48,
            fx: 40.0,
            fy: 40.0,
            cx: 31.5,
            cy: 23.5,
            model: CameraModel::Pinhole,
            max_theta: std::f64::consts::FRAC_PI_2,
        }
    }

    /// Points, normals and spacings of a test cloud.
    type TestCloud = (Vec<[f32; 3]>, Vec<Option<Vector3<f32>>>, Vec<f32>);

    /// A wall 2 m ahead of a camera at the origin looking along +z, points 5 cm apart.
    fn wall() -> TestCloud {
        let mut xyz = Vec::new();
        for y in -9..=9 {
            for x in -12..=12 {
                xyz.push([0.05 * x as f32, 0.05 * y as f32, 2.0]);
            }
        }
        let normals = vec![Some(Vector3::new(0.0, 0.0, -1.0)); xyz.len()];
        let spacing = vec![0.05; xyz.len()];
        (xyz, normals, spacing)
    }

    fn plain(rgb: [u8; 3]) -> RgbImage {
        let cam = pinhole();
        RgbImage {
            width: cam.width,
            height: cam.height,
            data: rgb
                .iter()
                .copied()
                .cycle()
                .take((cam.width * cam.height * 3) as usize)
                .collect(),
        }
    }

    /// The CPU loop of `colorize_with` over `images` (rank, image) at the identity pose.
    fn cpu_views(
        xyz: &[[f32; 3]],
        normals: &[Option<Vector3<f32>>],
        spacing: &[f32],
        cfg: &ColorConfig,
        by_rank: &[(f64, usize)],
        images: &[(u32, &RgbImage)],
    ) -> Vec<View> {
        let k = cfg.views_kept;
        let cam = pinhole();
        let lin = linear_table();
        let mut slots = vec![EMPTY; xyz.len() * k];
        let all: Vec<u32> = (0..xyz.len() as u32).collect();
        for &(rank, img) in images {
            let pose = Isometry3::identity();
            let seen = visible_points(
                xyz,
                spacing,
                &all,
                &cam,
                &pose,
                cfg.max_dist,
                cfg.zbuffer_scale,
                cfg.depth_tolerance,
            );
            for s in &seen {
                let i = s.index as usize;
                let w = view_weight(
                    &xyz[i],
                    normals[i].as_ref(),
                    &Vector3::zeros(),
                    s,
                    cam.max_theta,
                    1.0,
                );
                if w > 0.0 {
                    let rgb = sample_linear(img, s.proj.uv, &lin);
                    let v = View::new(w, rank, rgb, s.proj.theta);
                    keep_best_apart(&mut slots[i * k..(i + 1) * k], v, by_rank, cfg.view_gap);
                }
            }
        }
        slots
    }

    fn gpu_views(
        ctx: &GpuContext,
        xyz: &[[f32; 3]],
        normals: &[Option<Vector3<f32>>],
        spacing: &[f32],
        cfg: &ColorConfig,
        by_rank: &[(f64, usize)],
        images: &[(u32, &RgbImage)],
    ) -> Vec<View> {
        let cloud = GpuCloud::new(ctx, xyz, normals, spacing).unwrap();
        let mut acc = Accumulator::new(ctx, &cloud, cfg, by_rank);
        for &(rank, img) in images {
            acc.add(&pinhole(), &Isometry3::identity(), img, rank, 1.0)
                .unwrap();
        }
        acc.finish().unwrap()
    }

    #[test]
    fn ties_break_by_rank_like_cpu() {
        let Some(ctx) = GpuContext::new_for_tests() else {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            return;
        };
        let (xyz, normals, spacing) = wall();
        let (red, blue) = (plain([255, 0, 0]), plain([0, 0, 255]));
        // Two cameras at the same pose and time: equal weights everywhere.
        let two_cams = [(0.5, 0), (0.5, 1)];
        // One camera twice within a stretch of `view_gap`.
        let one_cam = [(0.1, 0), (0.2, 0)];
        for (by_rank, kept) in [(&two_cams, [1, 2]), (&one_cam, [1, 1])] {
            for (k, expect) in [(1usize, kept[0]), (2, kept[1])] {
                let cfg = ColorConfig {
                    views_kept: k,
                    views_blended: k,
                    ..ColorConfig::default()
                };
                for order in [[(1, &red), (0, &blue)], [(0, &blue), (1, &red)]] {
                    let cpu = cpu_views(&xyz, &normals, &spacing, &cfg, by_rank, &order);
                    let gpu = gpu_views(&ctx, &xyz, &normals, &spacing, &cfg, by_rank, &order);
                    let seen = cpu.chunks(k).filter(|s| s[0].weight > 0.0).count();
                    assert!(seen > 100, "{seen}");
                    for (p, (c, g)) in cpu.chunks(k).zip(gpu.chunks(k)).enumerate() {
                        let ranks = |s: &[View]| s.iter().map(|v| v.rank).collect::<Vec<_>>();
                        assert_eq!(ranks(c), ranks(g), "point {p}, k {k}");
                        if c[0].weight > 0.0 {
                            // The earlier image (rank 0, blue) first, the stretch keeps it alone.
                            assert_eq!(g[0].rank, 0, "point {p}");
                            assert_eq!(g[0].rgb, [0, 0, 16384], "point {p}");
                            assert_eq!(g.iter().filter(|v| v.weight > 0.0).count(), expect);
                        }
                        for (a, b) in c.iter().zip(g) {
                            assert!((a.weight - b.weight).abs() <= 1e-5 * a.weight.max(1e-3));
                            assert!(a.theta.abs_diff(b.theta) <= 2, "{a:?} vs {b:?}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn stretch_ids_follow_keep_best_apart() {
        let by_rank = [(0.2, 0), (0.9, 0), (0.95, 1), (1.0, 0), (-0.5, 0), (2.5, 1)];
        assert_eq!(stretch_ids(&by_rank, 0.0), None);
        let ids = stretch_ids(&by_rank, 1.0).unwrap();
        for (a, ra) in by_rank.iter().enumerate() {
            for (b, rb) in by_rank.iter().enumerate() {
                let same = ra.1 == rb.1 && (ra.0 / 1.0).floor() == (rb.0 / 1.0).floor();
                assert_eq!(ids[a] == ids[b], same, "{a} {b}");
            }
        }
    }
}
