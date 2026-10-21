//! The GPU side of the 3D view: the point cloud as instanced quads, one vertex buffer per
//! block of the octree, plus lines (screen-space quads of a width in points) and markers on
//! top, drawn into an offscreen texture that the interface shows as an image.
//!
//! Depth is reverse-Z (see [`crate::camera`]): cleared to 0, nearer is larger. wgpu is only
//! used through `egui_wgpu::wgpu`, so that it is always the version egui renders with.

use crate::lod::{Octree, PointVertex};
use bytemuck::{Pod, Zeroable};
use egui_wgpu::wgpu;
use std::ops::Range;

/// Format of [`RenderTarget::depth`].
pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// What the colour of a point shows.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ColorMode {
    /// The measured colour; without colours the intensity (see [`PointVertex::rgbi`]).
    #[default]
    Rgb = 0,
    Intensity = 1,
    /// The submap quality, with the ramp of `driftless_steps::overlay::quality_color`.
    Quality = 2,
}

/// An end of a line segment, relative to [`Octree::origin`]; segments are pairs of them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineVertex {
    pub pos: [f32; 3],
    /// `r | g << 8 | b << 16 | a << 24`.
    pub rgba: u32,
    /// Width in points (times [`FrameParams::line_scale`] on screen); a segment takes that of
    /// its first end.
    pub width: f32,
}

/// One segment as the line shader takes it (an instance of four corners).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct Segment {
    a: [f32; 3],
    b: [f32; 3],
    rgba: u32,
    width: f32,
}

/// What a frame is drawn with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameParams {
    /// Column-major (as `nalgebra::Matrix4::into` gives it).
    pub view_proj: [[f32; 4]; 4],
    /// Size of the target in physical pixels.
    pub viewport_px: [f32; 2],
    /// Point diameter in physical pixels (the setting × pixels per point).
    pub point_px: f32,
    /// Marker diameter in physical pixels.
    pub marker_px: f32,
    /// Physical pixels per point of a line's width (the window's pixels per point).
    pub line_scale: f32,
    pub color_mode: ColorMode,
    /// Round points instead of squares.
    pub round: bool,
    /// Clear colour, RGBA 0..1, written as is.
    pub background: [f32; 4],
    /// Whether the cloud carries submap qualities; without, [`ColorMode::Quality`] is gray.
    pub quality_known: bool,
    /// Height cut (relative z, inclusive): points outside are not drawn; markers always are.
    /// [`NO_CUT`] draws everything.
    pub z_range: [f32; 2],
}

/// A height cut that keeps every point.
pub const NO_CUT: [f32; 2] = [f32::MIN, f32::MAX];

/// The uniform block of both shaders (`Frame` in the WGSL).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    viewport_px: [f32; 2],
    point_px: f32,
    color_mode: u32,
    round: u32,
    quality_known: u32,
    z_min: f32,
    z_max: f32,
}

impl Uniforms {
    fn new(frame: &FrameParams, point_px: f32, color_mode: ColorMode) -> Self {
        Uniforms {
            view_proj: frame.view_proj,
            // A 0 here would divide by zero in the shader; a zero-size view is not drawn.
            viewport_px: frame.viewport_px.map(|v| v.max(1.0)),
            point_px,
            color_mode: color_mode as u32,
            round: u32::from(frame.round),
            quality_known: u32::from(frame.quality_known),
            z_min: frame.z_range[0],
            z_max: frame.z_range[1],
        }
    }
}

/// The offscreen image: colour (shown by egui) and depth.
pub struct RenderTarget {
    /// `RENDER_ATTACHMENT | TEXTURE_BINDING | COPY_SRC`.
    pub color: wgpu::Texture,
    /// The view egui registers.
    pub color_view: wgpu::TextureView,
    /// [`DEPTH_FORMAT`].
    pub depth: wgpu::Texture,
    pub depth_view: wgpu::TextureView,
    /// Width and height in physical pixels (at least 1).
    pub size: [u32; 2],
}

impl RenderTarget {
    /// A size of 0 is taken as 1 (wgpu has no empty textures).
    pub fn new(device: &wgpu::Device, size: [u32; 2], format: wgpu::TextureFormat) -> Self {
        let size = size.map(|s| s.max(1));
        let extent = wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        };
        let texture = |label, format, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: extent,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let color = texture(
            "3D-Ansicht Farbe",
            format,
            wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = texture(
            "3D-Ansicht Tiefe",
            DEPTH_FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        RenderTarget {
            color_view: color.create_view(&wgpu::TextureViewDescriptor::default()),
            depth_view: depth.create_view(&wgpu::TextureViewDescriptor::default()),
            color,
            depth,
            size,
        }
    }
}

/// A block of the cloud on the GPU: the vertices `range` of the octree.
struct Block {
    range: Range<u32>,
    buffer: wgpu::Buffer,
    /// All its vertices are written (only then is it drawn).
    filled: bool,
}

/// Size of a vertex on the GPU (bytes).
pub const VERTEX_BYTES: u64 = size_of::<PointVertex>() as u64;

/// How far an upload of the blocks has got: block `block`, vertices before `offset` of it
/// are written.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UploadCursor {
    pub block: usize,
    pub offset: u32,
}

/// The next pieces to write, at most `max_vertices` (≥ 1) in all: (block, vertex range
/// relative to the block). Moves `cursor` past them; empty when all is written.
pub fn upload_slices(
    chunks: &[Range<u32>],
    cursor: &mut UploadCursor,
    max_vertices: u32,
) -> Vec<(usize, Range<u32>)> {
    let mut left = max_vertices.max(1);
    let mut out = Vec::new();
    while left > 0 {
        let Some(chunk) = chunks.get(cursor.block) else {
            break;
        };
        let len = chunk.end - chunk.start;
        let take = (len - cursor.offset).min(left);
        if take > 0 {
            out.push((cursor.block, cursor.offset..cursor.offset + take));
        }
        left -= take;
        cursor.offset += take;
        if cursor.offset >= len {
            cursor.block += 1;
            cursor.offset = 0;
        }
    }
    out
}

/// Readbacks of the GPU time in flight at most.
const TIMER_SLOTS: usize = 4;

/// The GPU time of a frame from two timestamps (begin of the render pass, begin of an empty
/// pass after it), read
/// back without waiting: each frame resolves into a free readback buffer and maps it; a
/// later frame takes the result once the map is done.
struct GpuTimer {
    queries: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    slots: Vec<TimerSlot>,
    /// Nanoseconds per tick.
    period: f32,
    last_ms: Option<f32>,
}

struct TimerSlot {
    buffer: wgpu::Buffer,
    /// In use by a frame; `ready` is set by the map callback.
    busy: bool,
    ready: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

impl GpuTimer {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Option<Self> {
        if !device.features().contains(wgpu::Features::TIMESTAMP_QUERY) {
            return None;
        }
        let queries = device.create_query_set(&wgpu::QuerySetDescriptor {
            label: Some("3D-Ansicht Zeit"),
            ty: wgpu::QueryType::Timestamp,
            count: 2,
        });
        let buffer = |label, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: 16,
                usage,
                mapped_at_creation: false,
            })
        };
        let resolve = buffer(
            "Zeit auflösen",
            wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
        );
        let slots = (0..TIMER_SLOTS)
            .map(|_| TimerSlot {
                buffer: buffer(
                    "Zeit lesen",
                    wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                ),
                busy: false,
                ready: Default::default(),
            })
            .collect();
        Some(GpuTimer {
            queries,
            resolve,
            slots,
            period: queue.get_timestamp_period(),
            last_ms: None,
        })
    }

    /// Takes the results of finished readbacks (newest wins).
    fn collect(&mut self) {
        use std::sync::atomic::Ordering;
        for slot in &mut self.slots {
            if slot.busy && slot.ready.load(Ordering::Acquire) {
                if let Ok(bytes) = slot.buffer.slice(..).get_mapped_range() {
                    let t: &[u64] = bytemuck::cast_slice(&bytes);
                    if t[1] > t[0] {
                        self.last_ms = Some((t[1] - t[0]) as f32 * self.period / 1e6);
                    }
                    drop(bytes);
                    slot.buffer.unmap();
                }
                slot.ready.store(false, Ordering::Release);
                slot.busy = false;
            }
        }
    }
}

/// Draws a cloud uploaded with [`PointRenderer::upload_cloud`] and an overlay.
pub struct PointRenderer {
    points: wgpu::RenderPipeline,
    markers: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    /// Uniforms for the cloud, for the markers (other size, always [`ColorMode::Rgb`]) and
    /// for the lines (`point_px` = [`FrameParams::line_scale`]).
    cloud_uniforms: wgpu::Buffer,
    marker_uniforms: wgpu::Buffer,
    line_uniforms: wgpu::Buffer,
    cloud_bind: wgpu::BindGroup,
    marker_bind: wgpu::BindGroup,
    line_bind: wgpu::BindGroup,
    blocks: Vec<Block>,
    lines_buffer: Option<(wgpu::Buffer, u32)>,
    markers_buffer: Option<(wgpu::Buffer, u32)>,
    /// How far the cloud is written into `blocks`.
    upload: UploadCursor,
    /// `None` without timestamp queries on the device.
    timer: Option<GpuTimer>,
}

impl PointRenderer {
    /// Pipelines for a colour target of `format` and a depth target of [`DEPTH_FORMAT`].
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("3D-Ansicht Parameter"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(size_of::<Uniforms>() as u64),
                },
                count: None,
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("3D-Ansicht"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let uniforms = |label| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let bind = |buffer: &wgpu::Buffer| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("3D-Ansicht Parameter"),
                layout: &layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            })
        };
        let cloud_uniforms = uniforms("Wolke Parameter");
        let marker_uniforms = uniforms("Marker Parameter");
        let line_uniforms = uniforms("Linien Parameter");

        let points_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("points.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("points.wgsl").into()),
        });
        let lines_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("lines.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("lines.wgsl").into()),
        });
        let point_attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Uint32, 2 => Uint32];
        let line_attributes = wgpu::vertex_attr_array![
            0 => Float32x3, 1 => Float32x3, 2 => Uint32, 3 => Float32
        ];
        let pipeline = |label,
                        shader: &wgpu::ShaderModule,
                        vertex_entry,
                        buffer: wgpu::VertexBufferLayout<'_>,
                        topology,
                        depth: (bool, wgpu::CompareFunction),
                        blend| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: shader,
                    entry_point: Some(vertex_entry),
                    compilation_options: Default::default(),
                    buffers: &[Some(buffer)],
                },
                primitive: wgpu::PrimitiveState {
                    topology,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth.0),
                    depth_compare: Some(depth.1),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: shader,
                    entry_point: Some("fs_main"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(blend),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let point_buffer = wgpu::VertexBufferLayout {
            array_stride: size_of::<PointVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &point_attributes,
        };
        let points = pipeline(
            "Punkte",
            &points_shader,
            "vs_main",
            point_buffer.clone(),
            wgpu::PrimitiveTopology::TriangleStrip,
            (true, wgpu::CompareFunction::GreaterEqual),
            wgpu::BlendState::REPLACE,
        );
        // Markers lie on top of everything (no depth test), in the order given.
        let markers = pipeline(
            "Marker",
            &points_shader,
            "vs_marker",
            point_buffer,
            wgpu::PrimitiveTopology::TriangleStrip,
            (false, wgpu::CompareFunction::Always),
            wgpu::BlendState::REPLACE,
        );
        let lines = pipeline(
            "Linien",
            &lines_shader,
            "vs_main",
            wgpu::VertexBufferLayout {
                array_stride: size_of::<Segment>() as u64,
                step_mode: wgpu::VertexStepMode::Instance,
                attributes: &line_attributes,
            },
            wgpu::PrimitiveTopology::TriangleStrip,
            (false, wgpu::CompareFunction::GreaterEqual),
            wgpu::BlendState::ALPHA_BLENDING,
        );
        PointRenderer {
            points,
            markers,
            lines,
            cloud_bind: bind(&cloud_uniforms),
            marker_bind: bind(&marker_uniforms),
            line_bind: bind(&line_uniforms),
            line_uniforms,
            cloud_uniforms,
            marker_uniforms,
            blocks: Vec::new(),
            lines_buffer: None,
            markers_buffer: None,
            upload: UploadCursor::default(),
            timer: GpuTimer::new(device, queue),
        }
    }

    /// Whether GPU times of drawn frames are still on their way back.
    pub fn gpu_timer_pending(&self) -> bool {
        self.timer
            .as_ref()
            .is_some_and(|t| t.slots.iter().any(|s| s.busy))
    }

    /// Whether the device measures GPU time ([`wgpu::Features::TIMESTAMP_QUERY`]).
    pub fn has_gpu_timer(&self) -> bool {
        self.timer.is_some()
    }

    /// The GPU time of the latest frame whose time has come back (ms); never waits.
    pub fn gpu_ms(&mut self, device: &wgpu::Device) -> Option<f32> {
        let timer = self.timer.as_mut()?;
        // Runs the map callbacks of finished work, without blocking.
        let _ = device.poll(wgpu::PollType::Poll);
        timer.collect();
        timer.last_ms
    }

    /// Creates one empty vertex buffer per block of `tree.chunks` (replacing the previous
    /// cloud); [`PointRenderer::upload_step`] then fills them. A block is drawn once it is
    /// full.
    pub fn begin_upload(&mut self, device: &wgpu::Device, tree: &Octree) {
        self.blocks.clear();
        self.upload = UploadCursor::default();
        for range in &tree.chunks {
            let bytes = u64::from(range.end - range.start) * VERTEX_BYTES;
            if bytes == 0 {
                continue;
            }
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Wolke Block"),
                size: bytes,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.blocks.push(Block {
                range: range.clone(),
                buffer,
                filled: false,
            });
        }
    }

    /// Writes the next vertices of `tree` (the tree of [`PointRenderer::begin_upload`]), at
    /// most `max_bytes` (at least one vertex); returns the bytes written.
    pub fn upload_step(&mut self, queue: &wgpu::Queue, tree: &Octree, max_bytes: u64) -> u64 {
        let max_vertices = (max_bytes / VERTEX_BYTES).clamp(1, u64::from(u32::MAX)) as u32;
        let chunks: Vec<Range<u32>> = self.blocks.iter().map(|b| b.range.clone()).collect();
        let mut written = 0;
        for (b, rel) in upload_slices(&chunks, &mut self.upload, max_vertices) {
            let block = &mut self.blocks[b];
            let start = (block.range.start + rel.start) as usize;
            let end = (block.range.start + rel.end) as usize;
            let bytes: &[u8] = bytemuck::cast_slice(&tree.vertices[start..end]);
            queue.write_buffer(&block.buffer, u64::from(rel.start) * VERTEX_BYTES, bytes);
            written += bytes.len() as u64;
            if rel.end == block.range.end - block.range.start {
                block.filled = true;
            }
        }
        written
    }

    /// Share of the cloud written, 0..1 (1 for an empty cloud).
    pub fn upload_fraction(&self) -> f32 {
        let total: u64 = self
            .blocks
            .iter()
            .map(|b| u64::from(b.range.len() as u32))
            .sum();
        if total == 0 {
            return 1.0;
        }
        let done: u64 = self.blocks[..self.upload.block.min(self.blocks.len())]
            .iter()
            .map(|b| u64::from(b.range.len() as u32))
            .sum::<u64>()
            + u64::from(self.upload.offset);
        (done as f64 / total as f64) as f32
    }

    pub fn upload_done(&self) -> bool {
        self.upload.block >= self.blocks.len()
    }

    /// One vertex buffer per block of `tree.chunks`, filled at once; replaces the previous
    /// cloud. An empty cloud has no blocks. (The view uploads in slices instead.)
    pub fn upload_cloud(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, tree: &Octree) {
        self.begin_upload(device, tree);
        self.upload_step(queue, tree, u64::MAX);
    }

    /// Lines (pairs of vertices) and markers, both relative to the cloud's origin; replaces
    /// the previous overlay. Lines are hidden behind nearer points; markers lie on top of
    /// everything, later ones over earlier ones, in their RGB colour, `extra` giving the size
    /// in percent of [`FrameParams::marker_px`] (0 = 100 %).
    pub fn set_overlay(
        &mut self,
        device: &wgpu::Device,
        lines: &[LineVertex],
        markers: &[PointVertex],
    ) {
        let segments: Vec<Segment> = lines
            .as_chunks::<2>()
            .0
            .iter()
            .map(|[a, b]| Segment {
                a: a.pos,
                b: b.pos,
                rgba: a.rgba,
                width: a.width,
            })
            .collect();
        self.lines_buffer = filled_buffer(device, "Linien", bytemuck::cast_slice(&segments))
            .map(|b| (b, segments.len() as u32));
        self.markers_buffer = filled_buffer(device, "Marker", bytemuck::cast_slice(markers))
            .map(|b| (b, markers.len() as u32));
    }

    /// Clears the target and draws the nodes `drawn` of `tree` (the tree last uploaded),
    /// then the lines and the markers. Each node is drawn with one call per block it lies
    /// in; unknown node indices are ignored. A viewport side below 1 is taken as 1.
    ///
    /// Contract: both uniform buffers are shared by every frame and are written with
    /// `queue.write_buffer`, which takes effect at the next submit. So each call records
    /// and submits its own work (returned here) before the next call writes them again;
    /// two frames must never be recorded into one submission.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &RenderTarget,
        frame: &FrameParams,
        tree: &Octree,
        drawn: &[u32],
    ) -> wgpu::SubmissionIndex {
        queue.write_buffer(
            &self.cloud_uniforms,
            0,
            bytemuck::bytes_of(&Uniforms::new(frame, frame.point_px, frame.color_mode)),
        );
        queue.write_buffer(
            &self.marker_uniforms,
            0,
            bytemuck::bytes_of(&Uniforms::new(
                &FrameParams {
                    z_range: NO_CUT,
                    ..*frame
                },
                frame.marker_px,
                ColorMode::Rgb,
            )),
        );
        queue.write_buffer(
            &self.line_uniforms,
            0,
            bytemuck::bytes_of(&Uniforms::new(frame, frame.line_scale, ColorMode::Rgb)),
        );
        let draws = self.draws(tree, drawn);
        // A free readback for the GPU time; without one this frame is not timed.
        let timer_slot = self
            .timer
            .as_ref()
            .and_then(|t| t.slots.iter().position(|s| !s.busy));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("3D-Ansicht"),
        });
        {
            let [r, g, b, a] = frame.background.map(f64::from);
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3D-Ansicht"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r, g, b, a }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(0.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: timer_slot.map(|_| wgpu::RenderPassTimestampWrites {
                    query_set: &self.timer.as_ref().expect("Zeitmesser").queries,
                    beginning_of_pass_write_index: Some(0),
                    end_of_pass_write_index: None,
                }),
                occlusion_query_set: None,
                multiview_mask: None,
            });

            pass.set_pipeline(&self.points);
            pass.set_bind_group(0, &self.cloud_bind, &[]);
            let mut bound = None;
            for (block, instances) in draws {
                if bound != Some(block) {
                    pass.set_vertex_buffer(0, self.blocks[block].buffer.slice(..));
                    bound = Some(block);
                }
                pass.draw(0..4, instances);
            }

            if let Some((buffer, count)) = &self.lines_buffer {
                pass.set_pipeline(&self.lines);
                pass.set_bind_group(0, &self.line_bind, &[]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..4, 0..*count);
            }
            if let Some((buffer, count)) = &self.markers_buffer {
                pass.set_pipeline(&self.markers);
                pass.set_bind_group(0, &self.marker_bind, &[]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..4, 0..*count);
            }
        }
        if let (Some(timer), Some(i)) = (&self.timer, timer_slot) {
            // The end as the beginning of an empty pass after ours (same target, so it
            // starts when ours is done): Metal writes end-of-pass timestamps too late for
            // the resolve that follows, and drops empty compute passes.
            drop(encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3D-Ansicht Ende"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: Some(wgpu::RenderPassTimestampWrites {
                    query_set: &timer.queries,
                    beginning_of_pass_write_index: Some(1),
                    end_of_pass_write_index: None,
                }),
                occlusion_query_set: None,
                multiview_mask: None,
            }));
            encoder.resolve_query_set(&timer.queries, 0..2, &timer.resolve, 0);
            encoder.copy_buffer_to_buffer(&timer.resolve, 0, &timer.slots[i].buffer, 0, 16);
        }
        let submitted = queue.submit([encoder.finish()]);
        if let (Some(timer), Some(i)) = (&mut self.timer, timer_slot) {
            let slot = &mut timer.slots[i];
            slot.busy = true;
            let ready = slot.ready.clone();
            slot.buffer
                .slice(..)
                // set also on failure, so that the slot is freed
                .map_async(wgpu::MapMode::Read, move |_| {
                    ready.store(true, std::sync::atomic::Ordering::Release);
                });
        }
        submitted
    }

    /// (block, instances relative to the block) for every slice of a drawn node, sorted by
    /// block so that each buffer is bound once.
    fn draws(&self, tree: &Octree, drawn: &[u32]) -> Vec<(usize, Range<u32>)> {
        let mut out = Vec::new();
        for node in drawn.iter().filter_map(|&i| tree.nodes.get(i as usize)) {
            let (start, end) = (node.first, node.first + node.count);
            if start == end {
                continue;
            }
            // The first block that ends after the node starts.
            let mut b = self.blocks.partition_point(|blk| blk.range.end <= start);
            while let Some(block) = self.blocks.get(b) {
                if block.range.start >= end {
                    break;
                }
                if !block.filled {
                    b += 1;
                    continue;
                }
                let s = start.max(block.range.start);
                let e = end.min(block.range.end);
                out.push((b, s - block.range.start..e - block.range.start));
                b += 1;
            }
        }
        out.sort_by_key(|(block, _)| *block);
        out
    }
}

/// A vertex buffer holding `bytes`, or `None` when empty.
fn filled_buffer(device: &wgpu::Device, label: &str, bytes: &[u8]) -> Option<wgpu::Buffer> {
    (!bytes.is_empty()).then(|| {
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.len() as u64,
            usage: wgpu::BufferUsages::VERTEX,
            mapped_at_creation: true,
        });
        buffer
            .slice(..)
            .get_mapped_range_mut()
            .expect("frisch abgebildeter Puffer")
            .copy_from_slice(bytes);
        buffer.unmap();
        buffer
    })
}

/// A device without a window (tests, tools): the first adapter of the primary backends
/// (Metal, Vulkan, DX12), or `None` when there is none.
pub fn headless_device() -> Option<(wgpu::Device, wgpu::Queue, wgpu::AdapterInfo)> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    pollster::block_on(async {
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .ok()?;
        let info = adapter.get_info();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("Driftless ohne Fenster"),
                required_limits: wgpu::Limits::downlevel_defaults()
                    .using_resolution(adapter.limits()),
                ..Default::default()
            })
            .await
            .ok()?;
        Some((device, queue, info))
    })
}

/// `setup` that also asks for [`wgpu::Features::TIMESTAMP_QUERY`] where the adapter has it,
/// so that the view can show the GPU time of its frames.
pub fn with_timestamps(mut setup: egui_wgpu::WgpuSetupCreateNew) -> egui_wgpu::WgpuSetupCreateNew {
    let base = setup.device_descriptor.clone();
    setup.device_descriptor = std::sync::Arc::new(move |adapter| {
        let mut desc = base(adapter);
        if adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY) {
            desc.required_features |= wgpu::Features::TIMESTAMP_QUERY;
        }
        desc
    });
    setup
}
