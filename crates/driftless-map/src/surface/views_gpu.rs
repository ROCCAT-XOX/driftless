//! The pass of [`super::views`] over the images on the GPU.
//!
//! The GPU does what costs the time on the CPU: per used image the depth buffer from every
//! face's samples, the visibility of centroid and corners, the score, the carrier tag, the
//! mean colour and the angle, and the insertion into the face's slots (as `keep_best_view`)
//! and the exposure samples (as `keep_sample`). One invocation per face, images in stream
//! order (the sources' streams one after the other): no atomics on the slots, the same order
//! as the CPU on equal scores. The smoothed normals come from the CPU (`Faces`); after the
//! last image the slots come back and the exposure fit, the occlusion mask and the photo
//! consistency run on the CPU, unchanged.
//! The images are decoded in a thread of their own, up to [`LOOKAHEAD`] images ahead.
//!
//! When the slots of all faces do not fit the context's budget, the pass runs once per
//! contiguous range of faces ([`face_passes`]), the images decoded again each time; every
//! face is in every depth buffer, so the views are the same as in one pass.

use super::views::{
    FaceView, Pass, ANGLE_STEP, CARRIER_ALTERNATIVE, CARRIER_BELOW, CARRIER_NORMAL_Z,
    CARRIER_RADIUS, MAX_SUBDIVISIONS, SAMPLE_GAP_CELLS, SMOOTH_COS_MIN,
};
use crate::color::{
    frame_camera, linear_table, ColorCamera, ColorConfig, View, SHARPNESS_DOWNSCALE,
};
use crate::color_gpu::{vis_params, GpuStop, LOOKAHEAD};
use crate::MapError;
use driftless_gpu::{
    face_passes, faces_batch, guarded, CarrierRule, FaceData, FaceImage, FaceParams, FaceViews,
    GpuCamera, GpuContext, GpuRunError,
};
use driftless_io::bag::BagError;
use driftless_io::camera::Camera;
use driftless_io::images::{for_each_image, Frame, ImageError};
use driftless_io::progress::Progress;
use std::sync::mpsc::sync_channel;
use std::time::{Duration, Instant};

/// What [`gpu_views`] found.
pub(crate) struct GpuViews {
    /// The views per face and the exposure samples as the CPU pass of [`super::views`] leaves
    /// them.
    pub per_face: Vec<FaceView>,
    pub samples: Vec<View>,
    /// How long the GPU waited for decoded images (s).
    pub waited: f64,
    /// Passes over the images (ranges of faces, see the module docs).
    pub passes: usize,
}

/// The views per face and the exposure samples as the CPU pass of [`super::views`] leaves
/// them, in as few passes over the images as the context's budget allows. With more than
/// one pass the phase „Ansichten suchen“ starts again with one step per image and pass.
///
/// Before the first image the memory of a pass is checked against the context's budget and
/// the uploads and a first dispatch run inside error scopes; a failure there (also: not even
/// a pass of one face fits) is [`GpuStop::Fallback`] if the context falls back
/// (`--gpu auto`), else an error. A failure during the run is always a [`MapError::Gpu`],
/// never a panic.
pub(crate) fn gpu_views(
    pass: &Pass,
    ctx: &GpuContext,
    batch_faces: Option<usize>,
    progress: &Progress,
) -> Result<GpuViews, GpuStop> {
    let Pass {
        mesh,
        faces,
        cams,
        cfg,
        streams,
        stride,
        sample_faces,
        ..
    } = *pass;
    let ccfg = &cfg.color;
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
    let (image_bytes, depth_cells) = stream_sizes(cams, ccfg);
    let count = mesh.indices.len();
    let sample_stride = (sample_faces > 0).then_some(stride);
    let ranges =
        face_passes(ctx, count, k, sample_stride, image_bytes, depth_cells).map_err(start)?;
    let params = FaceParams {
        vis: vis_params(ccfg),
        rim_theta: ccfg.rim_theta,
        min_texel: cfg.min_texel,
        views_per_face: k,
        sample_stride,
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
        FaceViews::new(
            ctx,
            count,
            face,
            &linear_table(),
            params,
            batch,
            ranges[0].clone(),
        )
    })
    .and_then(|v| v)
    .map_err(start)?;
    views.probe().map_err(start)?;
    let passes = ranges.len();
    if passes > 1 {
        progress.info(format!(
            "Ansichten auf der Grafikkarte in {passes} Durchläufen über die Bilder \
             (je bis zu {} von {count} Flächen, Grafikspeicher {} MB)",
            ranges[0].len(),
            ctx.budget() / 1_000_000
        ));
        let walked: u64 = streams.iter().map(|s| s.walked()).sum();
        progress.phase("Ansichten suchen", Some(walked * passes as u64));
    }

    let gpu = |e: GpuRunError| GpuStop::Fail(MapError::Gpu(e.to_string()));
    let mut per_face = Vec::with_capacity(count * k);
    let mut samples = Vec::with_capacity(sample_faces * k);
    let mut waited = Duration::ZERO;
    for (p, range) in ranges.into_iter().enumerate() {
        if p > 0 {
            guarded(ctx, || views.keep(range))
                .and_then(|v| v)
                .map_err(gpu)?;
        }
        waited += image_pass(pass, &mut views, &geometry, progress)?;
        views
            .read_views_into(&mut per_face, |s| FaceView {
                image: s.image,
                score: s.score,
                colour: s.colour,
                tag: s.tag,
            })
            .map_err(gpu)?;
        samples.extend(
            views
                .read_samples(|s| View {
                    weight: s.weight,
                    rank: s.rank,
                    rgb: s.rgb,
                    theta: s.theta,
                })
                .map_err(gpu)?,
        );
    }
    Ok(GpuViews {
        per_face,
        samples,
        waited: waited.as_secs_f64(),
        passes,
    })
}

/// The largest decoded image (bytes) and depth buffer (cells) of `cams`.
pub(crate) fn stream_sizes(cams: &[ColorCamera], ccfg: &ColorConfig) -> (usize, usize) {
    let zs = ccfg.zbuffer_scale.max(1);
    let image_bytes = cams
        .iter()
        .map(|c| c.camera.downscaled(SHARPNESS_DOWNSCALE))
        .map(|c| c.width as usize * c.height as usize * 3)
        .max()
        .unwrap_or(0);
    let depth_cells = cams
        .iter()
        .map(|c| c.camera.downscaled(ccfg.downscale))
        .map(|c| c.width.div_ceil(zs) as usize * c.height.div_ceil(zs) as usize)
        .max()
        .unwrap_or(0);
    (image_bytes, depth_cells)
}

/// One pass over the images of every stream into `views`; how long it waited for decoded
/// images.
fn image_pass(
    pass: &Pass,
    views: &mut FaceViews<'_>,
    geometry: &[Camera],
    progress: &Progress,
) -> Result<Duration, GpuStop> {
    let Pass {
        sources,
        cams,
        cfg,
        streams,
        images,
        ..
    } = *pass;
    let ccfg = &cfg.color;
    // Pixel of the depth buffer's image → pixel of the decoded one.
    let to_colour = (f64::from(ccfg.downscale) / f64::from(SHARPNESS_DOWNSCALE)) as f32;
    let mut failure: Option<MapError> = None;
    let mut cancelled = false;
    let mut waited = Duration::ZERO;
    let read = std::thread::scope(|scope| {
        // Decoding runs ahead of the GPU by up to `LOOKAHEAD` images, source by source; a
        // source's stream stops after its last used image, all of them at a cancel or when the
        // GPU side stops listening. Every frame comes with its source.
        let (tx, rx) = sync_channel::<(usize, Result<Frame, ImageError>)>(LOOKAHEAD);
        let reader = scope.spawn(move || {
            let mut stop = false;
            for (s, (src, stream)) in sources.iter().zip(streams).enumerate() {
                let Some(last_used) = stream.last_used else {
                    continue;
                };
                let mut index = 0usize;
                for_each_image(
                    &src.images.bags,
                    &src.images.topics,
                    SHARPNESS_DOWNSCALE,
                    |frame| {
                        if progress.is_cancelled() || tx.send((s, frame)).is_err() {
                            stop = true;
                            return false;
                        }
                        let more = index < last_used;
                        index += 1;
                        more
                    },
                )?;
                if stop {
                    break;
                }
            }
            Ok::<(), BagError>(())
        });
        // The source of the frames coming in and the stream index within it.
        let (mut source, mut i) = (usize::MAX, 0usize);
        loop {
            let asked = Instant::now();
            let Ok((s, frame)) = rx.recv() else { break };
            waited += asked.elapsed();
            if progress.is_cancelled() {
                cancelled = true;
                break;
            }
            if s != source {
                (source, i) = (s, 0);
            }
            let (src, stream) = (&sources[s], &streams[s]);
            if let Some((camera, stamp, rank, sharpness_norm)) = stream.used(i) {
                match frame {
                    Ok(f)
                        if f.stamp == stamp
                            && frame_camera(&f, &src.cams, SHARPNESS_DOWNSCALE) == Some(camera) =>
                    {
                        let c = stream.cam_base + camera;
                        let added = views.add_image(&FaceImage {
                            geometry: GpuCamera::from_camera(&geometry[c]),
                            full: GpuCamera::from_camera(&cams[c].camera),
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
    Ok(waited)
}
