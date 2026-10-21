//! The GPU side of the 3D view: the point cloud as instanced quads, one vertex buffer per
//! block of the octree, plus lines and markers on top, drawn into an offscreen texture that
//! the interface shows as an image.
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

/// A corner of a line, relative to [`Octree::origin`].
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct LineVertex {
    pub pos: [f32; 3],
    /// `r | g << 8 | b << 16 | a << 24`.
    pub rgba: u32,
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
    pub color_mode: ColorMode,
    /// Round points instead of squares.
    pub round: bool,
    /// Clear colour, RGBA 0..1, written as is.
    pub background: [f32; 4],
}

/// The uniform block of both shaders (`Frame` in the WGSL).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    viewport_px: [f32; 2],
    point_px: f32,
    color_mode: u32,
    round: u32,
    _pad: [u32; 3],
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
            _pad: [0; 3],
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
}

/// Draws a cloud uploaded with [`PointRenderer::upload_cloud`] and an overlay.
pub struct PointRenderer {
    points: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    /// Uniforms for the cloud and for the markers (other size, always [`ColorMode::Rgb`]).
    cloud_uniforms: wgpu::Buffer,
    marker_uniforms: wgpu::Buffer,
    cloud_bind: wgpu::BindGroup,
    marker_bind: wgpu::BindGroup,
    blocks: Vec<Block>,
    lines_buffer: Option<(wgpu::Buffer, u32)>,
    markers_buffer: Option<(wgpu::Buffer, u32)>,
}

impl PointRenderer {
    /// Pipelines for a colour target of `format` and a depth target of [`DEPTH_FORMAT`].
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
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

        let points_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("points.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("points.wgsl").into()),
        });
        let lines_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("lines.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("lines.wgsl").into()),
        });
        let point_attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Uint32, 2 => Uint32];
        let line_attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Uint32];
        let pipeline = |label,
                        shader: &wgpu::ShaderModule,
                        buffer: wgpu::VertexBufferLayout<'_>,
                        topology,
                        depth_write,
                        blend| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[Some(buffer)],
                },
                primitive: wgpu::PrimitiveState {
                    topology,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
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
        let points = pipeline(
            "Punkte",
            &points_shader,
            wgpu::VertexBufferLayout {
                array_stride: size_of::<PointVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Instance,
                attributes: &point_attributes,
            },
            wgpu::PrimitiveTopology::TriangleStrip,
            true,
            wgpu::BlendState::REPLACE,
        );
        let lines = pipeline(
            "Linien",
            &lines_shader,
            wgpu::VertexBufferLayout {
                array_stride: size_of::<LineVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &line_attributes,
            },
            wgpu::PrimitiveTopology::LineList,
            false,
            wgpu::BlendState::ALPHA_BLENDING,
        );
        PointRenderer {
            points,
            lines,
            cloud_bind: bind(&cloud_uniforms),
            marker_bind: bind(&marker_uniforms),
            cloud_uniforms,
            marker_uniforms,
            blocks: Vec::new(),
            lines_buffer: None,
            markers_buffer: None,
        }
    }

    /// One vertex buffer per block of `tree.chunks`; replaces the previous cloud. An empty
    /// cloud has no blocks.
    pub fn upload_cloud(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, tree: &Octree) {
        self.blocks.clear();
        for range in &tree.chunks {
            let vertices = &tree.vertices[range.start as usize..range.end as usize];
            let Some(buffer) = vertex_buffer(device, "Wolke Block", bytemuck::cast_slice(vertices))
            else {
                continue;
            };
            queue.write_buffer(&buffer, 0, bytemuck::cast_slice(vertices));
            self.blocks.push(Block {
                range: range.clone(),
                buffer,
            });
        }
    }

    /// Lines (pairs of vertices) and markers, both relative to the cloud's origin; replaces
    /// the previous overlay.
    pub fn set_overlay(
        &mut self,
        device: &wgpu::Device,
        lines: &[LineVertex],
        markers: &[PointVertex],
    ) {
        let lines = &lines[..lines.len() / 2 * 2];
        self.lines_buffer = filled_buffer(device, "Linien", bytemuck::cast_slice(lines))
            .map(|b| (b, lines.len() as u32));
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
        &self,
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
            bytemuck::bytes_of(&Uniforms::new(frame, frame.marker_px, ColorMode::Rgb)),
        );
        let draws = self.draws(tree, drawn);

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
                timestamp_writes: None,
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
                pass.set_bind_group(0, &self.cloud_bind, &[]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..*count, 0..1);
            }
            if let Some((buffer, count)) = &self.markers_buffer {
                pass.set_pipeline(&self.points);
                pass.set_bind_group(0, &self.marker_bind, &[]);
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..4, 0..*count);
            }
        }
        queue.submit([encoder.finish()])
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

/// A vertex buffer for `bytes` (not filled), or `None` when empty.
fn vertex_buffer(device: &wgpu::Device, label: &str, bytes: &[u8]) -> Option<wgpu::Buffer> {
    (!bytes.is_empty()).then(|| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.len() as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    })
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
