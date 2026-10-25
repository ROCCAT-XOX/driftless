//! The best views of every point of a [`GpuCloud`] over a stream of images, on the GPU.
//!
//! Per image: depth buffer and visibility ([`ImagePass`]), then `views.wgsl` weighs every
//! seen point's view, reads its colour from the uploaded image and keeps the best
//! `views_kept` views per point (at most one per camera and stretch of capture time), like
//! `driftless_map::color`. The slots stay on the GPU until [`ImageViews::read`].

use crate::visibility::{entry, read_buffer_at};
use crate::{GpuCamera, GpuCloud, GpuContext, GpuRunError, ImagePass, VisParams, CAMERA_WGSL};
use bytemuck::{Pod, Zeroable};
use nalgebra::Isometry3;
use std::collections::VecDeque;

const WORKGROUP: u32 = 256;
/// Bytes of one kept view on the GPU (`vec4<u32>`).
const SLOT_BYTES: usize = 16;
/// Images submitted ahead of the GPU at most (bounds the staged image uploads).
const IN_FLIGHT: usize = 2;
/// Slots read back at a time (bytes).
pub(crate) const READ_CHUNK: u64 = 64 << 20;

/// One kept view as the GPU stores it, unpacked. `weight == 0` marks an empty slot (rank
/// `u32::MAX`, colour and angle 0).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewSlot {
    pub weight: f32,
    /// The image's position in (capture time, camera name) order.
    pub rank: u32,
    /// Linear RGB / 4 × 65535 (`View::rgb` of `driftless_map`).
    pub rgb: [u16; 3],
    /// θ / π × 65535.
    pub theta: u16,
}

impl ViewSlot {
    pub(crate) fn unpack(s: [u32; 4]) -> ViewSlot {
        ViewSlot {
            weight: f32::from_bits(s[0]),
            rank: !s[1],
            rgb: [s[2] as u16, (s[2] >> 16) as u16, s[3] as u16],
            theta: (s[3] >> 16) as u16,
        }
    }
}

/// Settings of the view selection.
#[derive(Debug, Clone, PartialEq)]
pub struct ViewParams {
    pub vis: VisParams,
    /// Views kept per point.
    pub views_kept: usize,
    /// Up to this angle from the optical axis a view has full weight (rad), beyond it the
    /// weight falls linearly to 0 at the camera's `max_theta`.
    pub full_weight_theta: f64,
    /// Per rank the id of the image's (camera, ⌊t / view_gap⌋) stretch; `None`: no limit.
    pub stretch: Option<Vec<u32>>,
}

/// One image of the stream.
pub struct ViewImage<'b> {
    /// The camera at the decoded image size.
    pub cam: GpuCamera,
    pub world_from_cam: Isometry3<f64>,
    /// RGB8, `cam.width` × `cam.height`, row by row.
    pub rgb: &'b [u8],
    pub rank: u32,
    /// Sharpness relative to the scan's median, at most 1.
    pub sharpness: f32,
}

/// Points per batch of a [`GpuCloud`] for [`ImageViews`] with `views_kept` views: the cloud's
/// own batches and the slot buffer of a batch both fit the device's limits.
pub fn views_batch_points(ctx: &GpuContext, views_kept: usize) -> usize {
    let limits = ctx.device().limits();
    let binding = usize::try_from(limits.max_storage_buffer_binding_size).unwrap_or(usize::MAX);
    let buffer = usize::try_from(limits.max_buffer_size).unwrap_or(usize::MAX);
    let per_point = SLOT_BYTES * views_kept.max(1);
    (crate::MAX_BATCH_BYTES / 32)
        .min(binding / 16)
        .min(buffer / 16)
        .min(binding / per_point)
        .min(buffer / per_point)
        .max(1)
}

/// Uniform block of `views.wgsl`.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ViewsUniform {
    cam: GpuCamera,
    rows: [[f32; 4]; 3],
    cam_pos: [f32; 4],
    img_w: u32,
    img_h: u32,
    rank: u32,
    views_kept: u32,
    sharpness: f32,
    full_theta: f32,
    use_stretch: u32,
    _pad: u32,
}

/// The view selection over a stream of images (see the module docs).
pub struct ImageViews<'a> {
    ctx: &'a GpuContext,
    cloud: &'a GpuCloud,
    pass: ImagePass<'a>,
    params: ViewParams,
    uniform: wgpu::Buffer,
    /// Slots per batch.
    slots: Vec<wgpu::Buffer>,
    image: wgpu::Buffer,
    image_capacity: u64,
    lin: wgpu::Buffer,
    stretch: wgpu::Buffer,
    groups: Vec<wgpu::BindGroup>,
    in_flight: VecDeque<wgpu::SubmissionIndex>,
}

impl<'a> ImageViews<'a> {
    /// Empty slots for every point of `cloud`; `linear` maps sRGB bytes to linear values.
    pub fn new(
        ctx: &'a GpuContext,
        cloud: &'a GpuCloud,
        linear: &[f32; 256],
        params: ViewParams,
    ) -> ImageViews<'a> {
        assert!(params.views_kept > 0, "at least one view per point");
        let device = ctx.device();
        let storage = |label: &str, bytes: &[u8]| {
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes.len().max(4) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            ctx.queue().write_buffer(&buffer, 0, bytes);
            buffer
        };
        let lin = storage("Linear-Tabelle", bytemuck::cast_slice(linear));
        let stretch = storage(
            "Zeitabschnitte",
            bytemuck::cast_slice(params.stretch.as_deref().unwrap_or(&[0])),
        );
        // All zero = empty slot (weight 0, rank !0 = u32::MAX).
        let slots = cloud
            .batches()
            .iter()
            .map(|b| {
                device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Ansichten"),
                    size: (b.len * params.views_kept * SLOT_BYTES).max(SLOT_BYTES) as u64,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                })
            })
            .collect();
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Ansichten Parameter"),
            size: size_of::<ViewsUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut views = ImageViews {
            ctx,
            cloud,
            pass: ImagePass::new(ctx, cloud),
            params,
            uniform,
            slots,
            image: image_buffer(device, 4),
            image_capacity: 4,
            lin,
            stretch,
            groups: Vec::new(),
            in_flight: VecDeque::new(),
        };
        views.bind();
        views
    }

    fn bind(&mut self) {
        let device = self.ctx.device();
        let layout = &pipeline(self.ctx).layout;
        self.groups = self
            .cloud
            .batches()
            .iter()
            .enumerate()
            .map(|(b, batch)| {
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("Ansichten"),
                    layout,
                    entries: &[
                        entry(0, &self.uniform),
                        entry(1, &batch.points),
                        entry(2, &batch.normals),
                        entry(3, &self.pass.bits[b]),
                        entry(4, &self.slots[b]),
                        entry(5, &self.image),
                        entry(6, &self.lin),
                        entry(7, &self.stretch),
                    ],
                })
            })
            .collect();
    }

    /// Checks the device before the first image: one dispatch of the views kernel (nothing
    /// is seen yet, so the slots stay empty), waited for.
    pub fn probe(&mut self) -> Result<(), GpuRunError> {
        failed(self.ctx)?;
        let index = self.dispatch_views();
        wait(self.ctx, index)?;
        failed(self.ctx)
    }

    /// Adds the views of one image. Submits the work and returns once at most
    /// [`IN_FLIGHT`] images are still being computed. Fails (German) once the device
    /// reported an error or is lost.
    pub fn add_image(&mut self, img: &ViewImage<'_>) -> Result<(), GpuRunError> {
        failed(self.ctx)?;
        let (w, h) = (img.cam.width, img.cam.height);
        crate::faces::check_image(img.rgb.len(), w, h)?;
        let ctx = self.ctx;
        let (device, queue) = (ctx.device(), ctx.queue());
        if let Some(s) = &self.params.stretch {
            if img.rank as usize >= s.len() {
                return Err(GpuRunError::Input(format!(
                    "keine Belichtung für Bild {}",
                    img.rank
                )));
            }
        }

        // The image, padded to whole words.
        let padded = (img.rgb.len() as u64).div_ceil(4) * 4;
        if padded > self.image_capacity {
            self.image = image_buffer(device, padded);
            self.image_capacity = padded;
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

        let local = self.cloud.origin();
        let pos = img.world_from_cam.translation.vector - local;
        let uniform = ViewsUniform {
            cam: img.cam,
            rows: self.cloud.cam_from_local(&img.world_from_cam),
            cam_pos: [pos.x as f32, pos.y as f32, pos.z as f32, 0.0],
            img_w: w,
            img_h: h,
            rank: img.rank,
            views_kept: self.params.views_kept as u32,
            sharpness: img.sharpness,
            full_theta: self.params.full_weight_theta as f32,
            use_stretch: u32::from(self.params.stretch.is_some()),
            _pad: 0,
        };
        let vis = self.params.vis;
        // Submitted now; the views kernel below runs after it on the queue.
        let _ = self
            .pass
            .depth_and_visibility(&img.cam, &img.world_from_cam, &vis);
        queue.write_buffer(&self.uniform, 0, bytemuck::bytes_of(&uniform));
        let index = self.dispatch_views();
        self.in_flight.push_back(index);
        while self.in_flight.len() > IN_FLIGHT {
            let Some(index) = self.in_flight.pop_front() else {
                break;
            };
            wait(ctx, index)?;
        }
        failed(ctx)
    }

    /// Submits the views kernel over every batch.
    fn dispatch_views(&self) -> wgpu::SubmissionIndex {
        let ctx = self.ctx;
        let mut encoder = ctx
            .device()
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Ansichten"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Ansichten"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&pipeline(ctx).pipeline);
            for (batch, group) in self.cloud.batches().iter().zip(&self.groups) {
                pass.set_bind_group(0, group, &[]);
                pass.dispatch_workgroups((batch.len as u32).div_ceil(WORKGROUP), 1, 1);
            }
        }
        ctx.queue().submit([encoder.finish()])
    }

    /// The kept views of every point, `views_kept` per point, best first, each converted by
    /// `f` (waits for the GPU). Read back in chunks of [`READ_CHUNK`] and converted chunk by
    /// chunk, so only the result and one chunk are in memory at once.
    pub fn read<T>(&self, f: impl Fn(ViewSlot) -> T) -> Result<Vec<T>, GpuRunError> {
        failed(self.ctx)?;
        let k = self.params.views_kept;
        let mut out = Vec::with_capacity(self.cloud.len() * k);
        for (batch, buffer) in self.cloud.batches().iter().zip(&self.slots) {
            let bytes = (batch.len * k * SLOT_BYTES) as u64;
            let mut offset = 0;
            while offset < bytes {
                let n = READ_CHUNK.min(bytes - offset);
                let words: Vec<[u32; 4]> = read_buffer_at(self.ctx, buffer, offset, n)?;
                out.extend(words.into_iter().map(|w| f(ViewSlot::unpack(w))));
                offset += n;
            }
        }
        Ok(out)
    }
}

/// The device's recorded error, if any.
pub(crate) fn failed(ctx: &GpuContext) -> Result<(), GpuRunError> {
    ctx.error().map_or(Ok(()), |e| Err(GpuRunError::Device(e)))
}

/// Waits for submission `index`.
pub(crate) fn wait(ctx: &GpuContext, index: wgpu::SubmissionIndex) -> Result<(), GpuRunError> {
    ctx.device()
        .poll(wgpu::PollType::Wait {
            submission_index: Some(index),
            timeout: None,
        })
        .map(|_| ())
        .map_err(|e| GpuRunError::Device(format!("antwortet nicht ({e})")))
}

/// GPU memory of [`ImageViews`] over `points` points with `views_kept` views (bytes): points
/// and normals (32 B), slots (16 B per view), visibility bits, the image and depth buffers
/// (with the staged uploads, see `faces::stream_bytes`) and the readback chunk.
pub fn views_bytes(
    points: usize,
    views_kept: usize,
    image_bytes: usize,
    depth_cells: usize,
) -> u64 {
    let points = points as u64;
    points * (32 + (SLOT_BYTES * views_kept) as u64)
        + points.div_ceil(32) * 4
        + crate::faces::stream_bytes(image_bytes, depth_cells, IN_FLIGHT)
        + READ_CHUNK
        + 4096
}

/// Runs `f` (allocations, pipelines, uploads) inside out-of-memory and validation error
/// scopes and waits for the GPU: an error (German) instead of a panic or a broken device.
pub fn guarded<T>(ctx: &GpuContext, f: impl FnOnce() -> T) -> Result<T, GpuRunError> {
    failed(ctx)?;
    let device = ctx.device();
    let oom = device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
    let validation = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let out = f();
    let poll = device.poll(wgpu::PollType::wait_indefinitely());
    let validation = pollster::block_on(validation.pop());
    let oom = pollster::block_on(oom.pop());
    if let Some(e) = oom {
        return Err(GpuRunError::Device(format!(
            "zu wenig Grafikspeicher ({e})"
        )));
    }
    if let Some(e) = validation {
        return Err(GpuRunError::Device(format!("ungültiger Auftrag ({e})")));
    }
    poll.map_err(|e| GpuRunError::Device(format!("antwortet nicht ({e})")))?;
    failed(ctx)?;
    Ok(out)
}

fn image_buffer(device: &wgpu::Device, bytes: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Bild"),
        size: bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

/// Compute pipeline of `views.wgsl`.
pub(crate) struct Pipeline {
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

fn pipeline(ctx: &GpuContext) -> &Pipeline {
    ctx.views_pipeline.get_or_init(|| {
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
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Ansichten"),
            entries: &[
                buffer(0, wgpu::BufferBindingType::Uniform),
                buffer(1, read),
                buffer(2, read),
                buffer(3, read),
                buffer(4, wgpu::BufferBindingType::Storage { read_only: false }),
                buffer(5, read),
                buffer(6, read),
                buffer(7, read),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Ansichten"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let source = format!("{CAMERA_WGSL}\n{}", include_str!("views.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Ansichten"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("views_main"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("views_main"),
            compilation_options: Default::default(),
            cache: None,
        });
        Pipeline { layout, pipeline }
    })
}
