//! The best views of every face of a mesh over a stream of images, on the GPU.
//!
//! Per image `faces.wgsl` builds the depth buffer from every face's centroid and a
//! barycentric grid of samples, then finds each face's view (centroid and corners visible,
//! score, carrier tag, mean colour, angle from the optical axis) and keeps the best
//! `views_per_face` views per face, and the exposure samples of every `stride`-th face (by
//! half octave of score, then a hash of sample and image), like the CPU pass of
//! `driftless_map::surface::views`. The slots stay on the GPU until
//! [`FaceViews::read_views`] / [`FaceViews::read_samples`].

use crate::views::{failed, wait, READ_CHUNK};
use crate::visibility::{entry, read_buffer_at};
use crate::{GpuCamera, GpuContext, GpuRunError, ViewSlot, VisParams, CAMERA_WGSL};
use bytemuck::{Pod, Zeroable};
use nalgebra::{Isometry3, Translation3, Vector3};
use std::collections::VecDeque;

const WORKGROUP: u32 = 64;
/// Faces per batch so that one dispatch stays within 65535 workgroups.
const MAX_DISPATCH_FACES: usize = 65535 * WORKGROUP as usize;
/// Bytes of one face on the GPU (six `vec4<f32>`).
const FACE_BYTES: usize = 96;
/// Bytes of one kept view of a face (three words).
const SLOT_BYTES: usize = 12;
/// Bytes of one exposure sample (`vec4<u32>`).
const SAMPLE_BYTES: usize = 16;
/// Images submitted ahead of the GPU at most (bounds the staged image uploads).
const IN_FLIGHT: usize = 2;
/// Faces staged on the CPU at a time while uploading.
const UPLOAD_CHUNK: usize = 1 << 18;
/// Margin of the distance cut (m): a face is skipped only if its centroid lies clearly
/// farther than `max_dist + reach` (f32 rounding).
const CULL_MARGIN: f64 = 0.01;

/// One face of the mesh in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceData {
    pub corners: [[f32; 3]; 3],
    pub centroid: [f32; 3],
    /// Longest edge (m).
    pub longest: f32,
    /// Unit normal (visibility, facing, carrier).
    pub normal: [f32; 3],
    /// Smoothed normal (score); zero where it cancels out.
    pub smooth: [f32; 3],
}

/// The carrier rule of `driftless_map::surface::views` (see there).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarrierRule {
    pub radius: f64,
    pub below: f64,
    pub normal_z: f32,
    /// A tagged view's score counts at this share in the slot order.
    pub alternative: f32,
}

/// Settings of the face views.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceParams {
    pub vis: VisParams,
    /// Views within this angle of the optical axis (rad) only.
    pub rim_theta: f64,
    /// Finest texel (m): caps the projected area.
    pub min_texel: f64,
    pub views_per_face: usize,
    /// Every `stride`-th face keeps exposure samples (`sample_faces` of them); `None`: none.
    pub sample_stride: Option<usize>,
    /// Meant gap between depth-buffer samples (cells) and most sample rows of a face.
    pub sample_gap_cells: f64,
    pub max_subdivisions: u32,
    /// The smoothed normal's cosine counts as at least this.
    pub smooth_cos_min: f64,
    pub carrier: CarrierRule,
    /// Step of the angle in the tag (rad).
    pub angle_step: f32,
    /// Largest distance from a centroid to its corners (m).
    pub reach: f64,
}

/// One image of the stream.
pub struct FaceImage<'b> {
    /// The camera of the depth buffer and at full size (score).
    pub geometry: GpuCamera,
    pub full: GpuCamera,
    pub world_from_cam: Isometry3<f64>,
    /// The decoded image, RGB8 row by row.
    pub rgb: &'b [u8],
    pub rgb_size: (u32, u32),
    /// Pixel of the depth buffer's image → pixel of the decoded one: `(p + ½) · to_colour − ½`.
    pub to_colour: f32,
    pub rank: u32,
    /// Sharpness relative to the scan's median, at most 1.
    pub sharpness: f32,
}

/// One kept view of a face, unpacked. `score == 0` marks an empty slot (image `u32::MAX`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceSlot {
    pub image: u32,
    pub score: f32,
    /// sRGB.
    pub colour: [u8; 3],
    /// Bit 7: may show the carrier; bits 0–6: angle from the optical axis in steps.
    pub tag: u8,
}

impl FaceSlot {
    fn unpack(w: [u32; 3]) -> FaceSlot {
        FaceSlot {
            score: f32::from_bits(w[0]),
            image: !w[1],
            colour: [w[2] as u8, (w[2] >> 8) as u8, (w[2] >> 16) as u8],
            tag: (w[2] >> 24) as u8,
        }
    }
}

/// Faces per batch for `views_per_face` views: a batch's faces and slots fit the device's
/// limits.
pub fn faces_batch(ctx: &GpuContext, views_per_face: usize) -> usize {
    let limits = ctx.device().limits();
    let binding = usize::try_from(limits.max_storage_buffer_binding_size).unwrap_or(usize::MAX);
    let buffer = usize::try_from(limits.max_buffer_size).unwrap_or(usize::MAX);
    let slots = SLOT_BYTES * views_per_face.max(1);
    let most = binding.min(buffer);
    (most / FACE_BYTES)
        .min(most / slots)
        .clamp(1, MAX_DISPATCH_FACES)
}

/// GPU memory of [`FaceViews`] (bytes): faces (96 B), slots (12 B per view), exposure
/// samples (16 B per view), the image and depth buffers ([`stream_bytes`]) and the readback
/// chunk.
pub fn faces_bytes(
    faces: usize,
    views_per_face: usize,
    sample_faces: usize,
    image_bytes: usize,
    depth_cells: usize,
) -> u64 {
    let k = views_per_face as u64;
    faces as u64 * (FACE_BYTES as u64 + SLOT_BYTES as u64 * k)
        + sample_faces as u64 * SAMPLE_BYTES as u64 * k
        + stream_bytes(image_bytes, depth_cells, IN_FLIGHT)
        + READ_CHUNK
        + 4096
}

/// Memory of an image stream with images of at most `image_bytes` and depth buffers of at
/// most `depth_cells`: the image buffer, the one it replaces while growing, and a staged
/// copy of the image for every submission that may be in flight (`in_flight` waiting plus
/// the one just submitted); the depth buffer twice (growing).
pub(crate) fn stream_bytes(image_bytes: usize, depth_cells: usize, in_flight: usize) -> u64 {
    image_bytes as u64 * (2 + in_flight as u64 + 1) + depth_cells as u64 * 4 * 2
}

/// World position of a local origin for `points`: the centre of their bounding box, rounded
/// to whole metres (zero for none), so that `f32` keeps millimetres far from the world's.
pub fn local_origin(points: impl Iterator<Item = [f32; 3]>) -> Vector3<f64> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            let v = f64::from(p[k]);
            if v.is_finite() {
                lo[k] = lo[k].min(v);
                hi[k] = hi[k].max(v);
            }
        }
    }
    Vector3::from([0, 1, 2].map(|k| {
        if lo[k] <= hi[k] {
            (0.5 * (lo[k] + hi[k])).round()
        } else {
            0.0
        }
    }))
}

/// Uniform block of `faces.wgsl`.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FacesUniform {
    geo: GpuCamera,
    full: GpuCamera,
    rows: [[f32; 4]; 3],
    cam_pos: [f32; 4],
    depth_w: u32,
    depth_h: u32,
    scale: f32,
    focal: f32,
    max_dist: f32,
    tol_abs: f32,
    tol_rel: f32,
    sil_rel: f32,
    max_splat: i32,
    max_sub: u32,
    gap_cells: f32,
    rim_theta: f32,
    min_texel2: f32,
    smooth_cos_min: f32,
    sharpness: f32,
    rank: u32,
    k: u32,
    img_w: u32,
    img_h: u32,
    to_colour: f32,
    stride: u32,
    carrier_nz: f32,
    carrier_below: f32,
    carrier_radius: f32,
    carrier_alt: f32,
    angle_step: f32,
    _pad: [u32; 2],
}

/// One batch of faces on the GPU.
struct FaceBatch {
    start: usize,
    len: usize,
    faces: wgpu::Buffer,
    slots: wgpu::Buffer,
    info: wgpu::Buffer,
    group: Option<wgpu::BindGroup>,
}

/// The view selection of a mesh over a stream of images (see the module docs).
pub struct FaceViews<'a> {
    ctx: &'a GpuContext,
    params: FaceParams,
    origin: Vector3<f64>,
    batches: Vec<FaceBatch>,
    sample_faces: usize,
    samples: wgpu::Buffer,
    uniform: wgpu::Buffer,
    depth: wgpu::Buffer,
    depth_capacity: usize,
    image: wgpu::Buffer,
    image_capacity: u64,
    lin: wgpu::Buffer,
    in_flight: VecDeque<wgpu::SubmissionIndex>,
}

impl<'a> FaceViews<'a> {
    /// Uploads `count` faces (`face(i)`, world coordinates) in batches of at most
    /// `batch_faces`, with empty slots; `linear` maps sRGB bytes to linear values. Fails
    /// without views per face, with 2³² faces or more, or when a buffer cannot be written.
    pub fn new(
        ctx: &'a GpuContext,
        count: usize,
        face: impl Fn(usize) -> FaceData,
        linear: &[f32; 256],
        params: FaceParams,
        batch_faces: usize,
    ) -> Result<FaceViews<'a>, GpuRunError> {
        let k = params.views_per_face;
        if k == 0 {
            return Err(GpuRunError::Input("keine Ansichten je Fläche".into()));
        }
        if u32::try_from(count).is_err() {
            return Err(GpuRunError::Input(format!(
                "{count} Flächen, erlaubt sind weniger als 2³²"
            )));
        }
        let device = ctx.device();
        let origin = local_origin((0..count).map(|i| face(i).centroid));
        let local = |p: [f32; 3]| [0, 1, 2].map(|j| (f64::from(p[j]) - origin[j]) as f32);
        let batch = batch_faces.clamp(1, MAX_DISPATCH_FACES);
        let batches = (0..count)
            .step_by(batch)
            .map(|start| {
                let len = batch.min(count - start);
                let faces = upload(device, len, |i| {
                    let f = face(start + i);
                    let w = |p: [f32; 3], w: f32| {
                        let l = local(p);
                        [l[0], l[1], l[2], w]
                    };
                    let v = |p: [f32; 3]| [p[0], p[1], p[2], 0.0];
                    [
                        w(f.corners[0], 0.0),
                        w(f.corners[1], 0.0),
                        w(f.corners[2], 0.0),
                        w(f.centroid, f.longest),
                        v(f.normal),
                        v(f.smooth),
                    ]
                })?;
                // All zero = empty slot (score 0, image !0 = u32::MAX).
                let slots = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Flächen Ansichten"),
                    size: (len * k * SLOT_BYTES).max(4) as u64,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                });
                let info = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Flächen Stapel"),
                    size: 16,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                ctx.queue().write_buffer(
                    &info,
                    0,
                    bytemuck::cast_slice(&[start as u32, len as u32, 0, 0]),
                );
                Ok(FaceBatch {
                    start,
                    len,
                    faces,
                    slots,
                    info,
                    group: None,
                })
            })
            .collect::<Result<_, GpuRunError>>()?;
        let sample_faces = params.sample_stride.map_or(0, |s| count.div_ceil(s.max(1)));
        let samples = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Flächen Belichtung"),
            size: (sample_faces * k * SAMPLE_BYTES).max(SAMPLE_BYTES) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let lin = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Linear-Tabelle"),
            size: 256 * 4,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue()
            .write_buffer(&lin, 0, bytemuck::cast_slice(linear));
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Flächen Parameter"),
            size: size_of::<FacesUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut views = FaceViews {
            ctx,
            params,
            origin,
            batches,
            sample_faces,
            samples,
            uniform,
            depth: storage(device, "Tiefenpuffer", 4),
            depth_capacity: 1,
            image: storage(device, "Bild", 4),
            image_capacity: 4,
            lin,
            in_flight: VecDeque::new(),
        };
        views.bind();
        Ok(views)
    }

    fn bind(&mut self) {
        let device = self.ctx.device();
        let layout = &pipelines(self.ctx).layout;
        for b in &mut self.batches {
            b.group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Flächen"),
                layout,
                entries: &[
                    entry(0, &self.uniform),
                    entry(1, &b.info),
                    entry(2, &b.faces),
                    entry(3, &self.depth),
                    entry(4, &b.slots),
                    entry(5, &self.samples),
                    entry(6, &self.image),
                    entry(7, &self.lin),
                ],
            }));
        }
    }

    /// Checks the device before the first image: one dispatch with an all-zero uniform (a
    /// reach of 0 m: nothing is splatted or seen, the slots stay empty), waited for.
    pub fn probe(&mut self) -> Result<(), GpuRunError> {
        failed(self.ctx)?;
        self.ctx.queue().write_buffer(
            &self.uniform,
            0,
            bytemuck::bytes_of(&FacesUniform::zeroed()),
        );
        let index = self.dispatch(0);
        wait(self.ctx, index)?;
        failed(self.ctx)
    }

    /// Adds the views of one image. Submits the work and returns once at most
    /// [`IN_FLIGHT`] images are still being computed. Fails (German) once the device
    /// reported an error or is lost.
    pub fn add_image(&mut self, img: &FaceImage<'_>) -> Result<(), GpuRunError> {
        failed(self.ctx)?;
        let (w, h) = img.rgb_size;
        check_image(img.rgb.len(), w, h)?;
        let ctx = self.ctx;
        let (device, queue) = (ctx.device(), ctx.queue());
        let p = &self.params;
        let scale = p.vis.zbuffer_scale;
        if scale == 0 {
            return Err(GpuRunError::Input("Tiefenpuffer-Maßstab 0".into()));
        }
        let (dw, dh) = (
            img.geometry.width.div_ceil(scale),
            img.geometry.height.div_ceil(scale),
        );
        let cells = (dw as usize * dh as usize).max(1);
        let padded = (img.rgb.len() as u64).div_ceil(4) * 4;
        let mut rebind = false;
        if cells > self.depth_capacity {
            self.depth = storage(device, "Tiefenpuffer", cells as u64 * 4);
            self.depth_capacity = cells;
            rebind = true;
        }
        if padded > self.image_capacity {
            self.image = storage(device, "Bild", padded);
            self.image_capacity = padded;
            rebind = true;
        }
        if rebind {
            self.bind();
        }
        let whole = img.rgb.len() / 4 * 4;
        if whole > 0 {
            queue.write_buffer(&self.image, 0, &img.rgb[..whole]);
        }
        if whole < img.rgb.len() {
            let mut tail = [0u8; 4];
            tail[..img.rgb.len() - whole].copy_from_slice(&img.rgb[whole..]);
            queue.write_buffer(&self.image, whole as u64, &tail);
        }

        let p = &self.params;
        let local_from_cam = Translation3::from(-self.origin) * img.world_from_cam;
        let m = local_from_cam.inverse().to_homogeneous();
        let rows = [0, 1, 2].map(|i| [0, 1, 2, 3].map(|j| m[(i, j)] as f32));
        let pos = local_from_cam.translation.vector;
        let geo = img.geometry;
        let focal = f64::from(geo.fx.max(geo.fy)) / f64::from(scale);
        let uniform = FacesUniform {
            geo,
            full: img.full,
            rows,
            cam_pos: [
                pos.x as f32,
                pos.y as f32,
                pos.z as f32,
                (p.vis.max_dist + p.reach + CULL_MARGIN) as f32,
            ],
            depth_w: dw,
            depth_h: dh,
            scale: scale as f32,
            focal: focal as f32,
            max_dist: p.vis.max_dist as f32,
            tol_abs: p.vis.depth_tolerance as f32,
            tol_rel: p.vis.depth_tolerance_rel as f32,
            sil_rel: p.vis.silhouette_rel as f32,
            max_splat: p.vis.max_splat,
            max_sub: p.max_subdivisions.max(1),
            gap_cells: p.sample_gap_cells as f32,
            rim_theta: p.rim_theta as f32,
            min_texel2: (p.min_texel * p.min_texel) as f32,
            smooth_cos_min: p.smooth_cos_min as f32,
            sharpness: img.sharpness,
            rank: img.rank,
            k: p.views_per_face as u32,
            img_w: w,
            img_h: h,
            to_colour: img.to_colour,
            stride: p.sample_stride.map_or(0, |s| s.max(1) as u32),
            carrier_nz: p.carrier.normal_z,
            carrier_below: p.carrier.below as f32,
            carrier_radius: p.carrier.radius as f32,
            carrier_alt: p.carrier.alternative,
            angle_step: p.angle_step,
            _pad: [0; 2],
        };
        queue.write_buffer(&self.uniform, 0, bytemuck::bytes_of(&uniform));
        let index = self.dispatch(cells);
        self.in_flight.push_back(index);
        while self.in_flight.len() > IN_FLIGHT {
            let Some(index) = self.in_flight.pop_front() else {
                break;
            };
            wait(ctx, index)?;
        }
        failed(ctx)
    }

    /// Clears `cells` of the depth buffer, splats every batch, then finds the views of every
    /// batch.
    fn dispatch(&self, cells: usize) -> wgpu::SubmissionIndex {
        let ctx = self.ctx;
        let pipes = pipelines(ctx);
        let mut encoder = ctx
            .device()
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Flächen"),
            });
        if cells > 0 {
            encoder.clear_buffer(&self.depth, 0, Some(cells as u64 * 4));
        }
        for (label, pipeline) in [("Tiefenpuffer", &pipes.splat), ("Ansichten", &pipes.views)] {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(label),
                timestamp_writes: None,
            });
            pass.set_pipeline(pipeline);
            for b in &self.batches {
                pass.set_bind_group(0, b.group.as_ref(), &[]);
                pass.dispatch_workgroups((b.len as u32).div_ceil(WORKGROUP), 1, 1);
            }
        }
        ctx.queue().submit([encoder.finish()])
    }

    /// The kept views of every face, `views_per_face` per face, best first, each converted
    /// by `f` (waits for the GPU; read back chunk by chunk).
    pub fn read_views<T>(&self, f: impl Fn(FaceSlot) -> T) -> Result<Vec<T>, GpuRunError> {
        failed(self.ctx)?;
        let k = self.params.views_per_face;
        let total: usize = self.batches.iter().map(|b| b.len).sum();
        let mut out = Vec::with_capacity(total * k);
        let chunk = READ_CHUNK / SLOT_BYTES as u64 * SLOT_BYTES as u64;
        for b in &self.batches {
            let bytes = (b.len * k * SLOT_BYTES) as u64;
            let mut offset = 0;
            while offset < bytes {
                let n = chunk.min(bytes - offset);
                let words: Vec<[u32; 3]> = read_buffer_at(self.ctx, &b.slots, offset, n)?;
                out.extend(words.into_iter().map(|w| f(FaceSlot::unpack(w))));
                offset += n;
            }
        }
        debug_assert!(self
            .batches
            .windows(2)
            .all(|w| w[0].start + w[0].len == w[1].start));
        Ok(out)
    }

    /// The exposure samples, `views_per_face` per sampled face (face `s · stride`), best
    /// first, each converted by `f`.
    pub fn read_samples<T>(&self, f: impl Fn(ViewSlot) -> T) -> Result<Vec<T>, GpuRunError> {
        failed(self.ctx)?;
        let bytes = (self.sample_faces * self.params.views_per_face * SAMPLE_BYTES) as u64;
        let mut out = Vec::with_capacity(self.sample_faces * self.params.views_per_face);
        let mut offset = 0;
        while offset < bytes {
            let n = READ_CHUNK.min(bytes - offset);
            let words: Vec<[u32; 4]> = read_buffer_at(self.ctx, &self.samples, offset, n)?;
            out.extend(words.into_iter().map(|w| f(ViewSlot::unpack(w))));
            offset += n;
        }
        Ok(out)
    }
}

fn storage(device: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

/// An RGB8 image of `w × h` has `len` bytes and is not empty.
pub(crate) fn check_image(len: usize, w: u32, h: u32) -> Result<(), GpuRunError> {
    let want = w as usize * h as usize * 3;
    if w == 0 || h == 0 || len != want {
        return Err(GpuRunError::Input(format!(
            "Bild {w} × {h} mit {len} statt {want} Bytes"
        )));
    }
    Ok(())
}

/// A storage buffer of `n` faces filled from `item`, staged in chunks.
fn upload(
    device: &wgpu::Device,
    n: usize,
    item: impl Fn(usize) -> [[f32; 4]; 6],
) -> Result<wgpu::Buffer, GpuRunError> {
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Flächen"),
        size: (n * FACE_BYTES).max(FACE_BYTES) as u64,
        usage: wgpu::BufferUsages::STORAGE,
        mapped_at_creation: true,
    });
    {
        let mut view = buffer
            .get_mapped_range_mut(..)
            .map_err(|e| GpuRunError::Device(format!("Puffer nicht beschreibbar ({e:?})")))?;
        let mut staged: Vec<[[f32; 4]; 6]> = Vec::with_capacity(n.min(UPLOAD_CHUNK));
        for start in (0..n).step_by(UPLOAD_CHUNK) {
            let end = (start + UPLOAD_CHUNK).min(n);
            staged.clear();
            staged.extend((start..end).map(&item));
            view.slice(start * FACE_BYTES..end * FACE_BYTES)
                .copy_from_slice(bytemuck::cast_slice(&staged));
        }
    }
    buffer.unmap();
    Ok(buffer)
}

/// Compute pipelines of `faces.wgsl`, sharing one layout.
pub(crate) struct Pipelines {
    layout: wgpu::BindGroupLayout,
    splat: wgpu::ComputePipeline,
    views: wgpu::ComputePipeline,
}

fn pipelines(ctx: &GpuContext) -> &Pipelines {
    ctx.faces_pipelines.get_or_init(|| {
        let device = ctx.device();
        let buffer = |binding: u32, ty: wgpu::BufferBindingType| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let read = wgpu::BufferBindingType::Storage { read_only: true };
        let write = wgpu::BufferBindingType::Storage { read_only: false };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Flächen"),
            entries: &[
                buffer(0, wgpu::BufferBindingType::Uniform),
                buffer(1, wgpu::BufferBindingType::Uniform),
                buffer(2, read),
                buffer(3, write),
                buffer(4, write),
                buffer(5, write),
                buffer(6, read),
                buffer(7, read),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Flächen"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let source = format!("{CAMERA_WGSL}\n{}", include_str!("faces.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Flächen"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let pipeline = |entry: &str| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        Pipelines {
            splat: pipeline("splat_main"),
            views: pipeline("views_main"),
            layout,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_unpacks_like_the_shader_packs() {
        // score 2.5, image 7, colour (1, 2, 3), tag 0x85.
        let w = [2.5f32.to_bits(), !7u32, 1 | 2 << 8 | 3 << 16 | 0x85 << 24];
        assert_eq!(
            FaceSlot::unpack(w),
            FaceSlot {
                image: 7,
                score: 2.5,
                colour: [1, 2, 3],
                tag: 0x85
            }
        );
        // A zero slot is empty.
        let e = FaceSlot::unpack([0; 3]);
        assert_eq!((e.score, e.image), (0.0, u32::MAX));
    }

    #[test]
    fn uniform_matches_the_shader_layout() {
        // Two cameras (48 B each), three rows and the camera position (16 B each), then
        // seven blocks of four words.
        assert_eq!(size_of::<FacesUniform>(), 96 + 64 + 7 * 16);
    }

    #[test]
    fn origin_is_the_rounded_centre() {
        let o = local_origin([[1000.2, -3.0, 0.4], [1002.9, 5.0, 0.6]].into_iter());
        assert_eq!(o, Vector3::new(1002.0, 1.0, 1.0));
        assert_eq!(local_origin(std::iter::empty()), Vector3::zeros());
    }
}
