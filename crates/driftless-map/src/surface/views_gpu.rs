//! The pass of [`super::views`] over the images on the GPU.
//!
//! The GPU does what costs the time on the CPU: per used image the depth buffer from every
//! face's samples, the visibility of centroid and corners, the score, the carrier tag, the
//! mean colour and the angle, and the insertion into the face's slots (as `keep_best_view`)
//! and the exposure samples (as `keep_sample`). One invocation per face, images in stream
//! order: no atomics on the slots, the same order as the CPU on equal scores. The smoothed
//! normals come from the CPU (`Faces`); after the last image the slots come back and the
//! exposure fit, the occlusion mask and the photo consistency run on the CPU, unchanged.
//! The images are decoded in a thread of their own, up to [`LOOKAHEAD`] images ahead.

use super::views::{
    FaceView, Pass, ANGLE_STEP, CARRIER_ALTERNATIVE, CARRIER_BELOW, CARRIER_NORMAL_Z,
    CARRIER_RADIUS, MAX_SUBDIVISIONS, SAMPLE_GAP_CELLS, SMOOTH_COS_MIN,
};
use crate::color::{frame_camera, linear_table, ImageRecord, Status, View, SHARPNESS_DOWNSCALE};
use crate::color_gpu::{vis_params, GpuStop, LOOKAHEAD};
use crate::MapError;
use driftless_gpu::{
    faces_batch, faces_bytes, guarded, CarrierRule, FaceData, FaceImage, FaceParams, FaceViews,
    GpuCamera, GpuContext, GpuRunError,
};
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, ImageError};
use driftless_io::progress::Progress;
use std::sync::mpsc::sync_channel;
use std::time::{Duration, Instant};

/// The views per face and the exposure samples as the CPU pass of [`super::views`] leaves
/// them, and how long the GPU waited for decoded images (s).
///
/// Before the first image the memory is checked against the context's budget and the
/// uploads and a first dispatch run inside error scopes; a failure there is
/// [`GpuStop::Fallback`] if the context falls back (`--gpu auto`), else an error. A failure
/// during the run is always a [`MapError::Gpu`], never a panic.
#[allow(clippy::type_complexity)]
pub(crate) fn gpu_views(
    pass: &Pass,
    ctx: &GpuContext,
    batch_faces: Option<usize>,
    progress: &Progress,
) -> Result<((Vec<FaceView>, Vec<View>), f64), GpuStop> {
    let Pass {
        mesh,
        faces,
        src,
        cfg,
        records,
        images,
        last_used,
        stride,
        sample_faces,
    } = *pass;
    let (cams, ccfg) = (&src.cams, &cfg.color);
    let k = cfg.views_per_face;
    let start = |e: GpuRunError| {
        if ctx.falls_back() {
            GpuStop::Fallback(e.to_string())
        } else {
            GpuStop::Fail(MapError::Gpu(e.to_string()))
        }
    };
    let geometry: Vec<Camera> = cams
        .iter()
        .map(|c| c.camera.downscaled(ccfg.downscale))
        .collect();
    let colour: Vec<Camera> = cams
        .iter()
        .map(|c| c.camera.downscaled(SHARPNESS_DOWNSCALE))
        .collect();
    let zs = ccfg.zbuffer_scale;
    let image_bytes = colour
        .iter()
        .map(|c| c.width as usize * c.height as usize * 3)
        .max()
        .unwrap_or(0);
    let depth_cells = geometry
        .iter()
        .map(|c| c.width.div_ceil(zs) as usize * c.height.div_ceil(zs) as usize)
        .max()
        .unwrap_or(0);
    let count = mesh.indices.len();
    let need = faces_bytes(count, k, sample_faces, image_bytes, depth_cells);
    ctx.check_budget(need).map_err(start)?;
    let params = FaceParams {
        vis: vis_params(ccfg),
        rim_theta: ccfg.rim_theta,
        min_texel: cfg.min_texel,
        views_per_face: k,
        sample_stride: (sample_faces > 0).then_some(stride),
        sample_gap_cells: SAMPLE_GAP_CELLS,
        max_subdivisions: MAX_SUBDIVISIONS as u32,
        smooth_cos_min: SMOOTH_COS_MIN,
        carrier: CarrierRule {
            radius: CARRIER_RADIUS,
            below: CARRIER_BELOW,
            normal_z: CARRIER_NORMAL_Z,
            alternative: CARRIER_ALTERNATIVE,
        },
        angle_step: ANGLE_STEP,
        reach: faces.reach,
    };
    let face = |f: usize| FaceData {
        corners: mesh.indices[f].map(|i| mesh.positions[i as usize]),
        centroid: faces.centroid[f],
        longest: faces.longest[f],
        normal: faces.normal[f],
        smooth: faces.smooth[f],
    };
    let batch = batch_faces.unwrap_or_else(|| faces_batch(ctx, k));
    let mut views = guarded(ctx, || {
        FaceViews::new(ctx, count, face, &linear_table(), params, batch)
    })
    .and_then(|v| v)
    .map_err(start)?;
    views.probe().map_err(start)?;

    // Pixel of the depth buffer's image → pixel of the decoded one.
    let to_colour = (f64::from(ccfg.downscale) / f64::from(SHARPNESS_DOWNSCALE)) as f32;
    let mut failure: Option<MapError> = None;
    let mut cancelled = false;
    let mut waited = Duration::ZERO;
    let read = std::thread::scope(|scope| {
        // Decoding runs ahead of the GPU by up to `LOOKAHEAD` images; it stops after the last
        // used image, at a cancel or when the GPU side stops listening.
        let (tx, rx) = sync_channel::<Result<Frame, ImageError>>(LOOKAHEAD);
        let reader = scope.spawn(move || {
            let mut index = 0usize;
            for_each_image(
                &src.images.bags,
                &src.images.topics,
                SHARPNESS_DOWNSCALE,
                |frame| {
                    if progress.is_cancelled() {
                        return false;
                    }
                    let more = index < last_used;
                    index += 1;
                    tx.send(frame).is_ok() && more
                },
            )
        });
        let mut i = 0usize;
        loop {
            let asked = Instant::now();
            let Ok(frame) = rx.recv() else { break };
            waited += asked.elapsed();
            if progress.is_cancelled() {
                cancelled = true;
                break;
            }
            if let Some(&ImageRecord {
                camera,
                stamp,
                status:
                    Status::Used {
                        rank,
                        sharpness_norm,
                    },
            }) = records.get(i)
            {
                match frame {
                    Ok(f)
                        if f.stamp == stamp
                            && frame_camera(&f, cams, SHARPNESS_DOWNSCALE) == Some(camera) =>
                    {
                        let added = views.add_image(&FaceImage {
                            geometry: GpuCamera::from_camera(&geometry[camera]),
                            full: GpuCamera::from_camera(&cams[camera].camera),
                            world_from_cam: images[rank as usize].world_from_cam,
                            rgb: &f.rgb.data,
                            rgb_size: (f.rgb.width, f.rgb.height),
                            to_colour,
                            rank,
                            sharpness: sharpness_norm,
                        });
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
            i += 1;
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
    let gpu = |e: GpuRunError| GpuStop::Fail(MapError::Gpu(e.to_string()));
    let per_face = views
        .read_views(|s| FaceView {
            image: s.image,
            score: s.score,
            colour: s.colour,
            tag: s.tag,
        })
        .map_err(gpu)?;
    let samples = views
        .read_samples(|s| View {
            weight: s.weight,
            rank: s.rank,
            rgb: s.rgb,
            theta: s.theta,
        })
        .map_err(gpu)?;
    Ok(((per_face, samples), waited.as_secs_f64()))
}
