//! Depth buffer and per-point visibility for one camera image on the GPU.
//!
//! Mirrors `driftless_map::visibility::visible_points`: project every point, write its range
//! into the cells of its footprint (minimum per cell, an integer atomic on the bits of the
//! positive `f32`, so the result does not depend on the order of the writes), then mark the
//! points that lie within the depth tolerance of their cell and on no occluder's silhouette.

use crate::{GpuCamera, GpuContext, GpuRunError, CAMERA_WGSL};
use bytemuck::{Pod, Zeroable};
use nalgebra::{Isometry3, Translation3, Vector3};

/// Largest size of one batch of the cloud on the GPU (points and normals together).
pub const MAX_BATCH_BYTES: usize = 256 << 20;

/// Settings of the visibility test: the values of `driftless_map::ColorConfig` and the
/// constants of `driftless_map::visibility`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisParams {
    /// Points farther from the camera are not seen (m).
    pub max_dist: f64,
    /// Absolute part of the depth tolerance (m).
    pub depth_tolerance: f64,
    /// A depth-buffer cell covers `zbuffer_scale` × `zbuffer_scale` pixels.
    pub zbuffer_scale: u32,
    /// Relative part of the depth tolerance (`DEPTH_TOLERANCE_REL`).
    pub depth_tolerance_rel: f64,
    /// Relative depth step to a neighbouring cell that marks a silhouette (`SILHOUETTE_REL`).
    pub silhouette_rel: f64,
    /// Largest splat half-size in cells (`MAX_SPLAT`).
    pub max_splat: i32,
}

impl Default for VisParams {
    /// The defaults of `ColorConfig` and the constants of `driftless_map::visibility`.
    fn default() -> VisParams {
        VisParams {
            max_dist: 12.0,
            depth_tolerance: 0.05,
            zbuffer_scale: 4,
            depth_tolerance_rel: 0.02,
            silhouette_rel: 0.2,
            max_splat: 4,
        }
    }
}

/// Bytes per point on the GPU: position and spacing, normal (each a `vec4<f32>`).
const POINT_BYTES: usize = 32;
const WORKGROUP: u32 = 256;
/// Points per batch so that one dispatch stays within 65535 workgroups.
const MAX_DISPATCH_POINTS: usize = 65535 * WORKGROUP as usize;
/// Points staged on the CPU at a time while uploading.
const UPLOAD_CHUNK: usize = 1 << 20;

/// A point cloud on the GPU, uploaded once: per point the position (relative to
/// [`GpuCloud::origin`], so large coordinates keep their precision in `f32`) and spacing,
/// and the normal, in batches of at most [`MAX_BATCH_BYTES`].
pub struct GpuCloud {
    origin: Vector3<f64>,
    len: usize,
    batches: Vec<Batch>,
}

/// One batch of a [`GpuCloud`]: points `start..start + len`.
pub struct Batch {
    pub start: usize,
    pub len: usize,
    /// `array<vec4<f32>>`: x, y, z relative to the origin, spacing (m).
    pub points: wgpu::Buffer,
    /// `array<vec4<f32>>`: unit normal and 1, or all 0 for a point without normal.
    pub normals: wgpu::Buffer,
}

impl GpuCloud {
    /// Uploads the cloud: `xyz` in world coordinates, per point its normal (if any) and its
    /// spacing (m, sets the splat size). An error if a buffer cannot be written (the GPU
    /// could not allocate it): the caller falls back to the CPU.
    pub fn new(
        ctx: &GpuContext,
        xyz: &[[f32; 3]],
        normals: &[Option<Vector3<f32>>],
        spacing: &[f32],
    ) -> Result<GpuCloud, GpuRunError> {
        let limits = ctx.device().limits();
        let max_bytes = MAX_BATCH_BYTES
            .min(usize::try_from(limits.max_storage_buffer_binding_size).unwrap_or(usize::MAX) * 2)
            .min(usize::try_from(limits.max_buffer_size).unwrap_or(usize::MAX) * 2);
        GpuCloud::with_batch_points(ctx, xyz, normals, spacing, max_bytes / POINT_BYTES)
    }

    /// [`GpuCloud::new`] with at most `batch_points` points per batch.
    pub fn with_batch_points(
        ctx: &GpuContext,
        xyz: &[[f32; 3]],
        normals: &[Option<Vector3<f32>>],
        spacing: &[f32],
        batch_points: usize,
    ) -> Result<GpuCloud, GpuRunError> {
        assert_eq!(normals.len(), xyz.len(), "a normal slot per point");
        assert_eq!(spacing.len(), xyz.len(), "a spacing per point");
        assert!(
            u32::try_from(xyz.len()).is_ok(),
            "fewer than 2^32 points on the GPU"
        );
        let batch_points = batch_points.clamp(1, MAX_DISPATCH_POINTS);
        let origin = centre(xyz);
        let device = ctx.device();
        let batches = (0..xyz.len())
            .step_by(batch_points)
            .map(|start| {
                let end = (start + batch_points).min(xyz.len());
                let points = upload(device, "Wolke Punkte", end - start, |i| {
                    let p = xyz[start + i];
                    let local = [0, 1, 2].map(|k| (f64::from(p[k]) - origin[k]) as f32);
                    [
                        local[0],
                        local[1],
                        local[2],
                        finite_spacing(spacing[start + i]),
                    ]
                })?;
                let normals = upload(device, "Wolke Normalen", end - start, |i| {
                    match normals[start + i] {
                        Some(n) => [n.x, n.y, n.z, 1.0],
                        None => [0.0; 4],
                    }
                })?;
                Ok(Batch {
                    start,
                    len: end - start,
                    points,
                    normals,
                })
            })
            .collect::<Result<_, _>>()?;
        Ok(GpuCloud {
            origin,
            len: xyz.len(),
            batches,
        })
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// World position of the cloud's local origin (the rounded centre of its bounding box).
    pub fn origin(&self) -> Vector3<f64> {
        self.origin
    }

    pub fn batch_count(&self) -> usize {
        self.batches.len()
    }

    pub fn batches(&self) -> &[Batch] {
        &self.batches
    }

    /// `cam_from_world` for points relative to the origin, in f32 rows (rotation |
    /// translation), computed in f64.
    pub fn cam_from_local(&self, world_from_cam: &Isometry3<f64>) -> [[f32; 4]; 3] {
        let local_from_cam = Translation3::from(-self.origin) * world_from_cam;
        let m = local_from_cam.inverse().to_homogeneous();
        [0, 1, 2].map(|i| [0, 1, 2, 3].map(|j| m[(i, j)] as f32))
    }
}

/// Largest spacing uploaded (m): far beyond any splat of [`VisParams::max_splat`] cells.
const MAX_SPACING: f32 = 1e6;

/// `spacing` made finite for the GPU with the CPU's splat size: NaN and negative spacings
/// give no splat (half-size 0 on the CPU as well), infinite ones the largest.
fn finite_spacing(spacing: f32) -> f32 {
    if spacing.is_nan() || spacing <= 0.0 {
        0.0
    } else {
        spacing.min(MAX_SPACING)
    }
}

/// Centre of the bounding box of `xyz`, rounded to whole metres (zero for no points).
fn centre(xyz: &[[f32; 3]]) -> Vector3<f64> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in xyz {
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

/// A storage buffer of `n` `vec4<f32>` filled from `item`, staged in chunks; an error if the
/// GPU could not allocate it.
fn upload(
    device: &wgpu::Device,
    label: &str,
    n: usize,
    item: impl Fn(usize) -> [f32; 4],
) -> Result<wgpu::Buffer, GpuRunError> {
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: (n * 16) as u64,
        usage: wgpu::BufferUsages::STORAGE,
        mapped_at_creation: true,
    });
    {
        let mut view = buffer
            .get_mapped_range_mut(..)
            .map_err(|e| GpuRunError::Device(format!("Puffer nicht beschreibbar ({e:?})")))?;
        let mut staged: Vec<[f32; 4]> = Vec::with_capacity(n.min(UPLOAD_CHUNK));
        for start in (0..n).step_by(UPLOAD_CHUNK) {
            let end = (start + UPLOAD_CHUNK).min(n);
            staged.clear();
            staged.extend((start..end).map(&item));
            view.slice(start * 16..end * 16)
                .copy_from_slice(bytemuck::cast_slice(&staged));
        }
    }
    buffer.unmap();
    Ok(buffer)
}

/// Uniform block of `visibility.wgsl`.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct VisUniform {
    cam: GpuCamera,
    rows: [[f32; 4]; 3],
    depth_w: u32,
    depth_h: u32,
    scale: f32,
    focal: f32,
    max_dist: f32,
    tol_abs: f32,
    tol_rel: f32,
    sil_rel: f32,
    max_splat: i32,
    _pad: [u32; 3],
}

/// The depth buffer and visibility of one image after another over a [`GpuCloud`]. Buffers
/// are kept between images (the depth buffer grows with the largest image).
pub struct ImagePass<'a> {
    ctx: &'a GpuContext,
    cloud: &'a GpuCloud,
    uniform: wgpu::Buffer,
    /// Depth buffer, see `visibility.wgsl`; holds `depth_capacity` cells.
    depth: wgpu::Buffer,
    depth_capacity: usize,
    /// Size of the current image's depth buffer in cells.
    depth_size: (u32, u32),
    /// Visibility bits per batch.
    pub(crate) bits: Vec<wgpu::Buffer>,
    /// Bind group per batch (rebuilt when the depth buffer grows).
    bind_groups: Vec<wgpu::BindGroup>,
}

impl<'a> ImagePass<'a> {
    pub fn new(ctx: &'a GpuContext, cloud: &'a GpuCloud) -> ImagePass<'a> {
        let device = ctx.device();
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Sichtbarkeit Parameter"),
            size: size_of::<VisUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bits = cloud
            .batches
            .iter()
            .map(|b| {
                device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Sichtbarkeit Bits"),
                    size: (b.len.div_ceil(32) * 4) as u64,
                    usage: wgpu::BufferUsages::STORAGE
                        | wgpu::BufferUsages::COPY_SRC
                        | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                })
            })
            .collect();
        let mut pass = ImagePass {
            ctx,
            cloud,
            uniform,
            depth: depth_buffer(device, 1),
            depth_capacity: 1,
            depth_size: (1, 1),
            bits,
            bind_groups: Vec::new(),
        };
        pass.bind();
        pass
    }

    fn bind(&mut self) {
        let device = self.ctx.device();
        let layout = &pipelines(self.ctx).layout;
        self.bind_groups = self
            .cloud
            .batches
            .iter()
            .zip(&self.bits)
            .map(|(batch, bits)| {
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("Sichtbarkeit"),
                    layout,
                    entries: &[
                        entry(0, &self.uniform),
                        entry(1, &batch.points),
                        entry(2, &self.depth),
                        entry(3, bits),
                    ],
                })
            })
            .collect();
    }

    /// Depth buffer and visibility of the image of `cam` at pose `world_from_cam`. The work
    /// is submitted, not waited for; the result stays on the GPU until read.
    pub fn depth_and_visibility(
        &mut self,
        cam: &GpuCamera,
        world_from_cam: &Isometry3<f64>,
        params: &VisParams,
    ) -> Visible<'_> {
        assert!(params.zbuffer_scale > 0, "depth-buffer scale at least 1");
        let ctx = self.ctx;
        let (device, queue) = (ctx.device(), ctx.queue());
        let scale = params.zbuffer_scale;
        let (w, h) = (cam.width.div_ceil(scale), cam.height.div_ceil(scale));
        let cells = (w as usize * h as usize).max(1);
        if cells > self.depth_capacity {
            self.depth = depth_buffer(device, cells);
            self.depth_capacity = cells;
            self.bind();
        }
        self.depth_size = (w, h);
        let uniform = VisUniform {
            cam: *cam,
            rows: self.cloud.cam_from_local(world_from_cam),
            depth_w: w,
            depth_h: h,
            scale: scale as f32,
            focal: cam.fx.max(cam.fy) / scale as f32,
            max_dist: params.max_dist as f32,
            tol_abs: params.depth_tolerance as f32,
            tol_rel: params.depth_tolerance_rel as f32,
            sil_rel: params.silhouette_rel as f32,
            max_splat: params.max_splat,
            _pad: [0; 3],
        };
        queue.write_buffer(&self.uniform, 0, bytemuck::bytes_of(&uniform));

        let pipes = pipelines(ctx);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Sichtbarkeit"),
        });
        encoder.clear_buffer(&self.depth, 0, Some((cells * 4) as u64));
        for bits in &self.bits {
            encoder.clear_buffer(bits, 0, None);
        }
        for (label, pipeline) in [
            ("Tiefenpuffer", &pipes.splat),
            ("Sichtbarkeit", &pipes.visible),
        ] {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(label),
                timestamp_writes: None,
            });
            pass.set_pipeline(pipeline);
            for (batch, group) in self.cloud.batches.iter().zip(&self.bind_groups) {
                pass.set_bind_group(0, group, &[]);
                pass.dispatch_workgroups((batch.len as u32).div_ceil(WORKGROUP), 1, 1);
            }
        }
        queue.submit([encoder.finish()]);
        Visible { pass: self }
    }
}

pub(crate) fn entry(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}

fn depth_buffer(device: &wgpu::Device, cells: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Tiefenpuffer"),
        size: (cells * 4) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

/// The result of [`ImagePass::depth_and_visibility`], on the GPU until read.
pub struct Visible<'p> {
    pass: &'p ImagePass<'p>,
}

impl Visible<'_> {
    /// Depth-buffer size in cells (width, height).
    pub fn depth_size(&self) -> (u32, u32) {
        self.pass.depth_size
    }

    /// Depth buffer (`array<u32>`, row by row, `depth_size` cells used): per cell the bitwise
    /// complement of the minimum range's `f32` bits, 0 where empty.
    pub fn depth_buffer(&self) -> &wgpu::Buffer {
        &self.pass.depth
    }

    /// Visibility bits of batch `batch` (`array<u32>`, bit `i % 32` of word `i / 32` for the
    /// batch's point `i`).
    pub fn bits(&self, batch: usize) -> &wgpu::Buffer {
        &self.pass.bits[batch]
    }

    /// Per point of the cloud whether the image sees it (reads back from the GPU).
    pub fn read(&self) -> Vec<bool> {
        let mut out = Vec::with_capacity(self.pass.cloud.len);
        for (batch, buffer) in self.pass.cloud.batches.iter().zip(&self.pass.bits) {
            let words: Vec<u32> = read_buffer(self.pass.ctx, buffer, buffer.size());
            out.extend((0..batch.len).map(|i| words[i / 32] >> (i % 32) & 1 == 1));
        }
        out
    }

    /// The visibility bits, batch after batch (`ceil(len / 32)` words each).
    pub fn read_bits(&self) -> Vec<u32> {
        self.pass
            .bits
            .iter()
            .flat_map(|b| read_buffer::<u32>(self.pass.ctx, b, b.size()))
            .collect()
    }

    /// Minimum range per depth-buffer cell, row by row (`f32::INFINITY` where empty).
    pub fn read_depth(&self) -> Vec<f32> {
        let (w, h) = self.pass.depth_size;
        let cells = (w as usize * h as usize) as u64;
        read_buffer::<u32>(self.pass.ctx, &self.pass.depth, cells * 4)
            .into_iter()
            .map(|s| {
                if s == 0 {
                    f32::INFINITY
                } else {
                    f32::from_bits(!s)
                }
            })
            .collect()
    }
}

/// Copies the first `bytes` of `buffer` back to the CPU (waits for the GPU).
fn read_buffer<T: Pod>(ctx: &GpuContext, buffer: &wgpu::Buffer, bytes: u64) -> Vec<T> {
    read_buffer_at(ctx, buffer, 0, bytes).unwrap_or_else(|e| panic!("{e}"))
}

/// Copies `bytes` of `buffer` from `offset` on back to the CPU (waits for the GPU); an error
/// (German) if the device fails or the buffer cannot be mapped.
pub(crate) fn read_buffer_at<T: Pod>(
    ctx: &GpuContext,
    buffer: &wgpu::Buffer,
    offset: u64,
    bytes: u64,
) -> Result<Vec<T>, GpuRunError> {
    if bytes == 0 {
        return Ok(Vec::new());
    }
    let device = ctx.device();
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Rücklesen"),
        size: bytes,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("Rücklesen"),
    });
    encoder.copy_buffer_to_buffer(buffer, offset, &readback, 0, bytes);
    ctx.queue().submit([encoder.finish()]);
    let (tx, rx) = std::sync::mpsc::channel();
    readback.map_async(wgpu::MapMode::Read, .., move |r| {
        let _ = tx.send(r);
    });
    let failed = |what: &str| GpuRunError::Device(what.to_string());
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|e| failed(&format!("antwortet nicht ({e})")))?;
    match rx.try_recv() {
        Ok(Ok(())) => {}
        Ok(Err(e)) => return Err(failed(&format!("Ergebnis nicht lesbar ({e})"))),
        Err(_) => return Err(failed("Rücklesen abgebrochen")),
    }
    if let Some(e) = ctx.error() {
        return Err(GpuRunError::Device(e));
    }
    let view = readback
        .get_mapped_range(..)
        .map_err(|e| failed(&format!("Ergebnis nicht lesbar ({e})")))?;
    let out = bytemuck::cast_slice(&view).to_vec();
    drop(view);
    readback.unmap();
    Ok(out)
}

/// Compute pipelines of the depth buffer and visibility kernels, sharing one layout.
pub(crate) struct Pipelines {
    layout: wgpu::BindGroupLayout,
    splat: wgpu::ComputePipeline,
    visible: wgpu::ComputePipeline,
}

fn pipelines(ctx: &GpuContext) -> &Pipelines {
    ctx.visibility_pipelines.get_or_init(|| {
        let device = ctx.device();
        let storage = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Sichtbarkeit"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                storage(1, true),
                storage(2, false),
                storage(3, false),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Sichtbarkeit"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let source = format!("{CAMERA_WGSL}\n{}", include_str!("visibility.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Sichtbarkeit"),
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
            visible: pipeline("visible_main"),
            layout,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spacing_is_made_finite() {
        assert_eq!(finite_spacing(f32::NAN), 0.0);
        assert_eq!(finite_spacing(-0.1), 0.0);
        assert_eq!(finite_spacing(f32::INFINITY), MAX_SPACING);
        assert_eq!(finite_spacing(0.03), 0.03);
        // The CPU gives the largest splat for an infinite spacing; so does MAX_SPACING at any
        // range within reach.
        let cells = MAX_SPACING * 2000.0 / 12.0;
        assert!(cells.is_finite() && (0.5 * cells - 0.5).ceil() > 4.0);
    }
}
