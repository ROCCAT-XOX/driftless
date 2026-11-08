//! The textured surface on the GPU: one vertex and index buffer and one texture (with its
//! mipmaps) per page, drawn unlit (the pages are photos) into the same pass and the same
//! depth buffer as the points, so both hide each other where they should.
//!
//! Positions are relative to the surface's origin; a uniform offset (surface origin minus
//! the view's origin, see [`MeshRenderer::set_view_origin`]) moves them into the frame of
//! the cloud and the camera. The height cut applies per fragment.
//!
//! Over the surface lie the marked Stellen ([`MeshRenderer::set_marks`]): copies of their
//! triangles, drawn after the pages with `mark.wgsl`, tinted in the Stelle's colour and
//! outlined along its rim. A copy has the depth of the triangle it lies on bit for bit – both
//! shaders compute the position alike and mark it `@invariant` – so it passes the depth test
//! (`GreaterEqual`, writing no depth) exactly where its triangle is seen and stays hidden
//! behind whatever is nearer.
//!
//! The touched-up state („Nachher“, [`MeshRenderer::set_edit`]) is kept beside the computed
//! one: new buffers for the pages it changed, sharing their textures, plus the pages it
//! added. [`MeshRenderer::show_edit`] picks the buffers per page, so switching between
//! „Vorher“ and „Nachher“ uploads nothing.

use super::{FrameParams, ShowMode, DEPTH_FORMAT, SCAN_COLORS};
use crate::surface::LoadedSurface;
use bytemuck::{Pod, Zeroable};
use egui_wgpu::wgpu;
use egui_wgpu::wgpu::util::DeviceExt;
use image::RgbaImage;

/// A corner of a surface triangle: position relative to [`LoadedSurface::origin`] (Z up)
/// and texture coordinate (0…1, `v` down, row 0 of the page at `v = 0`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct MeshVertex {
    pub pos: [f32; 3],
    pub uv: [f32; 2],
}

/// The alpha (of 255) a Stelle is tinted with: the photo still shows through.
pub const MARK_TINT: u8 = 102;
/// Width of the outline along the rim of a Stelle (points; opaque, in the Stelle's colour).
pub const MARK_OUTLINE_PT: f32 = 2.0;

/// The colour of Stelle `nr` (counted from 1) for [`MarkTriangle::color`]: the colours of
/// [`SCAN_COLORS`] in turn, with the alpha [`MARK_TINT`].
pub fn mark_color(nr: u32) -> [u8; 4] {
    let [r, g, b] = SCAN_COLORS[nr.saturating_sub(1) as usize % SCAN_COLORS.len()];
    [r, g, b, MARK_TINT]
}

/// A triangle of a marked Stelle: a copy of one of the surface's, drawn over it
/// ([`MeshRenderer::set_marks`]). [`mark_triangles`] makes them for a Stelle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MarkTriangle {
    /// The corners exactly as on the page they lie on: the same bits (relative to
    /// [`LoadedSurface::origin`], made from the f64 place as [`crate::surface::load_mesh`]
    /// makes them) and the same order. Only then the copy has the depth of the triangle.
    pub pos: [[f32; 3]; 3],
    /// Whether edge `k` – from corner `k` to corner `(k + 1) % 3` – lies on the rim of the
    /// Stelle (no other of its triangles has it).
    pub boundary: [bool; 3],
    /// Whether corner `k` lies on the rim (it ends a rim edge, of this triangle or another of
    /// the Stelle). A triangle touching the rim with a corner only outlines around that corner,
    /// so the outline has no gaps there.
    pub boundary_corners: [bool; 3],
    /// The Stelle's colour and the alpha of its tint ([`mark_color`]).
    pub color: [u8; 4],
}

/// The triangles of one Stelle as marks in `color`. `corners` are their corners as vertex
/// numbers that are equal for the same place (the welded vertices of the surface), in the
/// order of the triangles on their pages; `pos` gives a vertex's place as on the pages (see
/// [`MarkTriangle::pos`]). The rim is what only one of the triangles has: an edge no other
/// one uses, and the corners at its ends. Same input, same output.
pub fn mark_triangles(
    corners: &[[u32; 3]],
    pos: impl Fn(u32) -> [f32; 3],
    color: [u8; 4],
) -> Vec<MarkTriangle> {
    // Every edge by its two ends (lower first), with its triangle and number: sorted, an edge
    // alone in its run is on the rim.
    let mut edges: Vec<(u32, u32, usize)> = Vec::with_capacity(3 * corners.len());
    for (t, c) in corners.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (c[k], c[(k + 1) % 3]);
            edges.push((a.min(b), a.max(b), 3 * t + k));
        }
    }
    edges.sort_unstable();
    let mut boundary = vec![[false; 3]; corners.len()];
    let mut rim_vertices = Vec::new();
    for run in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        if let [(a, b, e)] = run {
            boundary[e / 3][e % 3] = true;
            rim_vertices.extend([*a, *b]);
        }
    }
    rim_vertices.sort_unstable();
    rim_vertices.dedup();
    corners
        .iter()
        .zip(boundary)
        .map(|(c, boundary)| MarkTriangle {
            pos: c.map(&pos),
            boundary,
            boundary_corners: c.map(|v| rim_vertices.binary_search(&v).is_ok()),
            color,
        })
        .collect()
}

/// The touched-up state of a surface („Nachher“) for [`MeshRenderer::set_edit`]: what changed
/// on the pages of its [`LoadedSurface`], and the pages added after them.
/// [`crate::surface::mesh_edit`] makes it from what the touch-up gives.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeshEdit {
    /// Per changed page – its index in [`LoadedSurface::pages`] –: all its corners anew if
    /// some moved, all its triangles (three indices each) anew if some were taken off; `None`
    /// keeps the page's own.
    #[allow(clippy::type_complexity)]
    pub pages: Vec<(u32, Option<Vec<MeshVertex>>, Option<Vec<u32>>)>,
    /// The pages added: corners, triangles and texture, as [`LoadedSurface::pages`].
    pub extra: Vec<(Vec<MeshVertex>, Vec<u32>, RgbaImage)>,
    /// Per added page the halvings of its texture ([`mip_levels`]); a page without them gets
    /// them made when it is uploaded.
    pub extra_mips: Vec<Vec<RgbaImage>>,
}

/// The uniform block of `mesh.wgsl` and `mark.wgsl` (`Mesh`).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct MeshUniforms {
    view_proj: [[f32; 4]; 4],
    offset: [f32; 4],
    z_range: [f32; 4],
    /// x: width of a Stelle's outline (physical pixels).
    mark: [f32; 4],
}

// The WGSL struct is 16-byte aligned (its mat4x4); the Rust one must match it byte for byte.
const _: () = assert!(size_of::<MeshUniforms>().is_multiple_of(16));

/// A corner of a marked triangle as `mark.wgsl` takes it: three per triangle, not indexed.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct MarkVertex {
    pos: [f32; 3],
    /// Bit `k`: edge `k` lies on the rim; bit `3 + k`: corner `k` does.
    rim: u32,
    /// RGBA.
    color: [u8; 4],
}

const MARK_VERTEX_BYTES: u64 = size_of::<MarkVertex>() as u64;
/// The smallest buffer for the marks, so that a Stelle can grow by many dabs before it needs
/// a new one.
const MIN_MARK_BYTES: u64 = 1 << 16;

impl MarkTriangle {
    /// Its corners as `mark.wgsl` takes them.
    fn vertices(&self) -> [MarkVertex; 3] {
        let bits = |flags: [bool; 3], first: u32| {
            (0..3u32)
                .filter(|&k| flags[k as usize])
                .fold(0, |bits, k| bits | 1 << (first + k))
        };
        let rim = bits(self.boundary, 0) | bits(self.boundary_corners, 3);
        self.pos.map(|pos| MarkVertex {
            pos,
            rim,
            color: self.color,
        })
    }
}

/// A page on the GPU.
struct Page {
    /// Its index in [`LoadedSurface::pages`] (empty pages are not uploaded), or among the
    /// added pages of a [`MeshEdit`].
    index: u32,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
    /// Its texture and sampler (group 1).
    bind: wgpu::BindGroup,
}

/// What a page of the computed surface draws in the touched-up state: its own buffers or new
/// ones (a [`wgpu::Buffer`] is shared, not copied, when cloned).
struct Swap {
    vertices: wgpu::Buffer,
    /// `None`: no triangle of the page is left.
    indices: Option<wgpu::Buffer>,
    index_count: u32,
}

/// The touched-up state on the GPU.
struct Edit {
    /// Per page of [`MeshRenderer::pages`] (by position) its buffers there, if they differ.
    swaps: Vec<Option<Swap>>,
    /// The pages added, drawn after the others.
    extra: Vec<Page>,
}

/// Draws a [`LoadedSurface`] uploaded with [`MeshRenderer::upload`], the marked Stellen over
/// it and its touched-up state.
pub struct MeshRenderer {
    pipeline: wgpu::RenderPipeline,
    mark_pipeline: wgpu::RenderPipeline,
    uniforms: wgpu::Buffer,
    uniform_bind: wgpu::BindGroup,
    page_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    pages: Vec<Page>,
    origin: [f64; 3],
    view_origin: [f64; 3],
    /// A surface being uploaded page by page: its origin and the pages so far.
    incoming: Option<([f64; 3], Vec<Page>)>,
    /// The corners of the marked triangles (the buffer has room to grow) and how many of them
    /// are drawn.
    marks: Option<wgpu::Buffer>,
    mark_vertices: u32,
    /// The touched-up state of the surface shown, and whether it is drawn.
    edit: Option<Edit>,
    show_edit: bool,
    /// Bytes written to the GPU so far: pages, touched-up states, marks.
    uploaded: u64,
}

impl MeshRenderer {
    /// The pipelines for a colour target of `format` and a depth target of [`DEPTH_FORMAT`].
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        let uniform_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Oberfläche Parameter"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(size_of::<MeshUniforms>() as u64),
                },
                count: None,
            }],
        });
        let page_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Oberfläche Seite"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Oberfläche"),
            bind_group_layouts: &[Some(&uniform_layout), Some(&page_layout)],
            immediate_size: 0,
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("mesh.wgsl").into()),
        });
        let attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x2];
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Oberfläche"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: size_of::<MeshVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &attributes,
                })],
            },
            // No culling: rooms are seen from inside and from outside.
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            // Reverse-Z like the points.
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let mark_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Stellen"),
            bind_group_layouts: &[Some(&uniform_layout)],
            immediate_size: 0,
        });
        let mark_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mark.wgsl"),
            source: wgpu::ShaderSource::Wgsl(include_str!("mark.wgsl").into()),
        });
        let mark_attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Uint32, 2 => Unorm8x4];
        let mark_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Stellen"),
            layout: Some(&mark_layout),
            vertex: wgpu::VertexState {
                module: &mark_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: MARK_VERTEX_BYTES,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &mark_attributes,
                })],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            // Over the surface's own triangles, with their depth; writes none, so marks never
            // hide anything.
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &mark_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Oberfläche Parameter"),
            size: size_of::<MeshUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let uniform_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Oberfläche Parameter"),
            layout: &uniform_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms.as_entire_binding(),
            }],
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Oberfläche"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        MeshRenderer {
            pipeline,
            mark_pipeline,
            uniforms,
            uniform_bind,
            page_layout,
            sampler,
            pages: Vec::new(),
            origin: [0.0; 3],
            view_origin: [0.0; 3],
            incoming: None,
            marks: None,
            mark_vertices: 0,
            edit: None,
            show_edit: false,
            uploaded: 0,
        }
    }

    /// Puts `surface` on the GPU at once, replacing the previous one (tests and tools; the
    /// view uploads a page per frame with [`MeshRenderer::begin_upload`]).
    pub fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, surface: &LoadedSurface) {
        self.begin_upload(surface.origin);
        for i in 0..surface.pages.len() {
            self.upload_page(device, queue, surface, i);
        }
        self.finish_upload();
    }

    /// Starts uploading a surface with this origin; the one shown stays drawn until
    /// [`MeshRenderer::finish_upload`]. Pages of an upload not finished are dropped.
    pub fn begin_upload(&mut self, origin: [f64; 3]) {
        self.incoming = Some((origin, Vec::new()));
    }

    /// Uploads page `i` of `surface` (after [`MeshRenderer::begin_upload`]); returns the
    /// bytes written. A page larger than the device allows is halved until it fits; an empty
    /// page is skipped. Its mipmaps are those of [`LoadedSurface::mips`], made only if
    /// missing.
    pub fn upload_page(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        surface: &LoadedSurface,
        i: usize,
    ) -> u64 {
        let Some((vertices, indices, image)) = surface.pages.get(i) else {
            return 0;
        };
        let mips = surface.mips.get(i).map(Vec::as_slice);
        let Some((page, bytes)) =
            self.make_page(device, queue, i as u32, vertices, indices, image, mips)
        else {
            return 0;
        };
        let incoming = self
            .incoming
            .get_or_insert_with(|| (surface.origin, Vec::new()));
        incoming.1.push(page);
        bytes
    }

    /// A page on the GPU and the bytes written for it (counted in
    /// [`MeshRenderer::uploaded_bytes`]); `None` for a page without corners or triangles. A
    /// texture larger than the device allows starts at the first of its `mips` that fits;
    /// without `mips` they are made here.
    #[allow(clippy::too_many_arguments)]
    fn make_page(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        index: u32,
        vertices: &[MeshVertex],
        indices: &[u32],
        image: &RgbaImage,
        mips: Option<&[RgbaImage]>,
    ) -> Option<(Page, u64)> {
        if indices.is_empty() || vertices.is_empty() {
            return None;
        }
        let made;
        let mips = match mips {
            Some(m) => m,
            None => {
                made = mip_levels(image);
                &made
            }
        };
        let max_side = device.limits().max_texture_dimension_2d.max(1);
        let mut levels: Vec<&RgbaImage> = std::iter::once(image).chain(mips).collect();
        let skip = levels
            .iter()
            .position(|l| l.width() <= max_side && l.height() <= max_side)
            .unwrap_or(levels.len() - 1);
        levels.drain(..skip);
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Oberfläche Seite"),
            size: wgpu::Extent3d {
                width: levels[0].width(),
                height: levels[0].height(),
                depth_or_array_layers: 1,
            },
            mip_level_count: levels.len() as u32,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            // Not sRGB: the bytes are shown as they are, like the colours of the points.
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let mut bytes = 0;
        for (level, img) in levels.iter().enumerate() {
            queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &texture,
                    mip_level: level as u32,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                img.as_raw(),
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(img.width() * 4),
                    rows_per_image: Some(img.height()),
                },
                wgpu::Extent3d {
                    width: img.width(),
                    height: img.height(),
                    depth_or_array_layers: 1,
                },
            );
            bytes += img.as_raw().len() as u64;
        }
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Oberfläche Seite"),
            layout: &self.page_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        let vertex_bytes: &[u8] = bytemuck::cast_slice(vertices);
        let index_bytes: &[u8] = bytemuck::cast_slice(indices);
        bytes += (vertex_bytes.len() + index_bytes.len()) as u64;
        let page = Page {
            index,
            vertices: filled(
                device,
                "Oberfläche Ecken",
                vertex_bytes,
                wgpu::BufferUsages::VERTEX,
            ),
            indices: filled(
                device,
                "Oberfläche Dreiecke",
                index_bytes,
                wgpu::BufferUsages::INDEX,
            ),
            index_count: indices.len() as u32,
            bind,
        };
        self.uploaded += bytes;
        Some((page, bytes))
    }

    /// Shows the pages uploaded since [`MeshRenderer::begin_upload`] instead of the previous
    /// surface; the view's origin becomes the surface's until
    /// [`MeshRenderer::set_view_origin`] says otherwise. The marks and the touched-up state of
    /// the previous surface go with it.
    pub fn finish_upload(&mut self) {
        if let Some((origin, pages)) = self.incoming.take() {
            self.pages = pages;
            self.origin = origin;
            self.view_origin = origin;
            self.mark_vertices = 0;
            self.edit = None;
        }
    }

    /// Removes the surface (and one being uploaded), its marks and its touched-up state.
    pub fn clear(&mut self) {
        self.pages.clear();
        self.incoming = None;
        self.mark_vertices = 0;
        self.edit = None;
    }

    /// Whether nothing is uploaded.
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }

    /// Bytes written to the GPU so far: the pages of every surface, the touched-up states and
    /// the marks (not the few bytes of uniforms per frame).
    pub fn uploaded_bytes(&self) -> u64 {
        self.uploaded
    }

    /// Marks `tris` over the surface shown, replacing the marks before (none: an empty slice).
    /// They belong to its triangles: a new surface ([`MeshRenderer::finish_upload`],
    /// [`MeshRenderer::clear`]) drops them. They lie on the state drawn: where the touch-up
    /// moved a Stelle's corners, its marks for „Nachher“ do not lie on „Vorher“, so give them
    /// anew when [`MeshRenderer::show_edit`] switches. Written into a buffer with room to
    /// grow, so painting does not make a new one with every dab; triangles beyond what the
    /// device holds in one buffer are left out.
    pub fn set_marks(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, tris: &[MarkTriangle]) {
        let max_bytes = device.limits().max_buffer_size;
        let most = (max_bytes / (3 * MARK_VERTEX_BYTES)).min(u64::from(u32::MAX / 3)) as usize;
        let vertices: Vec<MarkVertex> = tris[..tris.len().min(most)]
            .iter()
            .flat_map(MarkTriangle::vertices)
            .collect();
        self.mark_vertices = vertices.len() as u32;
        if vertices.is_empty() {
            return;
        }
        let bytes: &[u8] = bytemuck::cast_slice(&vertices);
        let size = bytes.len() as u64;
        if self.marks.as_ref().is_none_or(|b| b.size() < size) {
            self.marks = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Stellen"),
                size: size.next_power_of_two().max(MIN_MARK_BYTES).min(max_bytes),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
        }
        let buffer = self.marks.as_ref().expect("Puffer der Stellen");
        queue.write_buffer(buffer, 0, bytes);
        self.uploaded += size;
    }

    /// The touched-up state of the surface shown („Nachher“), replacing the one before, or
    /// none. Uploads new buffers for the pages `edit` changed (the textures stay shared) and
    /// the pages it added; drawn while [`MeshRenderer::show_edit`] is on. It belongs to the
    /// surface shown: a new surface drops it. A page `edit` names that is not on the GPU (no
    /// page of the surface, or an empty one) is passed over.
    pub fn set_edit(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, edit: Option<MeshEdit>) {
        self.edit = None;
        let Some(edit) = edit else {
            return;
        };
        let mut swaps: Vec<Option<Swap>> = (0..self.pages.len()).map(|_| None).collect();
        let mut bytes = 0;
        for (index, vertices, indices) in edit.pages {
            let Some(slot) = self.pages.iter().position(|p| p.index == index) else {
                continue;
            };
            if vertices.is_none() && indices.is_none() {
                continue;
            }
            let page = &self.pages[slot];
            let mut upload = |label, data: &[u8], usage| {
                bytes += data.len() as u64;
                (!data.is_empty()).then(|| filled(device, label, data, usage))
            };
            let new_vertices = vertices.map(|v| {
                upload(
                    "Oberfläche Ecken nachgearbeitet",
                    bytemuck::cast_slice(&v),
                    wgpu::BufferUsages::VERTEX,
                )
            });
            let (indices, index_count) = match indices {
                Some(i) => (
                    upload(
                        "Oberfläche Dreiecke nachgearbeitet",
                        bytemuck::cast_slice(&i),
                        wgpu::BufferUsages::INDEX,
                    ),
                    i.len() as u32,
                ),
                None => (Some(page.indices.clone()), page.index_count),
            };
            swaps[slot] = Some(match new_vertices {
                None => Swap {
                    vertices: page.vertices.clone(),
                    indices,
                    index_count,
                },
                Some(Some(vertices)) => Swap {
                    vertices,
                    indices,
                    index_count,
                },
                // no corners left: nothing to draw
                Some(None) => Swap {
                    vertices: page.vertices.clone(),
                    indices: None,
                    index_count: 0,
                },
            });
        }
        self.uploaded += bytes;
        let mut extra = Vec::new();
        for (i, (vertices, indices, image)) in edit.extra.iter().enumerate() {
            let mips = edit.extra_mips.get(i).map(Vec::as_slice);
            if let Some((page, _)) =
                self.make_page(device, queue, i as u32, vertices, indices, image, mips)
            {
                extra.push(page);
            }
        }
        self.edit = Some(Edit { swaps, extra });
    }

    /// Draws the touched-up state given with [`MeshRenderer::set_edit`] („Nachher“, `true`)
    /// or the computed surface („Vorher“). Uploads nothing.
    pub fn show_edit(&mut self, on: bool) {
        self.show_edit = on;
    }

    /// The origin the camera and the cloud are relative to (the cloud's).
    pub fn set_view_origin(&mut self, origin: [f64; 3]) {
        self.view_origin = origin;
    }

    /// Writes the uniforms of `frame`; takes effect at the next submit, so call it before
    /// recording the pass that [`MeshRenderer::render`] draws into.
    pub fn prepare(&self, queue: &wgpu::Queue, frame: &FrameParams) {
        let offset = [0, 1, 2].map(|k| (self.origin[k] - self.view_origin[k]) as f32);
        let u = MeshUniforms {
            view_proj: frame.view_proj,
            offset: [offset[0], offset[1], offset[2], 0.0],
            z_range: [frame.z_range[0], frame.z_range[1], 0.0, 0.0],
            mark: [MARK_OUTLINE_PT * frame.line_scale, 0.0, 0.0, 0.0],
        };
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(&u));
    }

    /// Draws every page into `pass` (after [`MeshRenderer::prepare`]) – those of the
    /// touched-up state while it is shown – then the marks over them; nothing for
    /// [`ShowMode::Cloud`].
    pub fn render(&self, pass: &mut wgpu::RenderPass<'_>, frame: &FrameParams) {
        if frame.show == ShowMode::Cloud || self.pages.is_empty() {
            return;
        }
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.uniform_bind, &[]);
        let edit = self.edit.as_ref().filter(|_| self.show_edit);
        for (slot, page) in self.pages.iter().enumerate() {
            match edit.and_then(|e| e.swaps.get(slot)?.as_ref()) {
                Some(swap) => {
                    if let Some(indices) = &swap.indices {
                        draw_page(pass, &page.bind, &swap.vertices, indices, swap.index_count);
                    }
                }
                None => draw_page(
                    pass,
                    &page.bind,
                    &page.vertices,
                    &page.indices,
                    page.index_count,
                ),
            }
        }
        for page in edit.map_or(&[][..], |e| &e.extra) {
            draw_page(
                pass,
                &page.bind,
                &page.vertices,
                &page.indices,
                page.index_count,
            );
        }
        if let Some(marks) = self.marks.as_ref().filter(|_| self.mark_vertices > 0) {
            pass.set_pipeline(&self.mark_pipeline);
            pass.set_bind_group(0, &self.uniform_bind, &[]);
            pass.set_vertex_buffer(0, marks.slice(..));
            pass.draw(0..self.mark_vertices, 0..1);
        }
    }
}

/// A buffer of `usage` holding `contents` (not empty).
fn filled(
    device: &wgpu::Device,
    label: &str,
    contents: &[u8],
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents,
        usage,
    })
}

/// Draws `count` indices of a page with the texture of `bind`.
fn draw_page(
    pass: &mut wgpu::RenderPass<'_>,
    bind: &wgpu::BindGroup,
    vertices: &wgpu::Buffer,
    indices: &wgpu::Buffer,
    count: u32,
) {
    if count == 0 {
        return;
    }
    pass.set_bind_group(1, bind, &[]);
    pass.set_vertex_buffer(0, vertices.slice(..));
    pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
    pass.draw_indexed(0..count, 0, 0..1);
}

/// The halvings of `image` down to 1 × 1, without `image` itself (2 × 2 box filter; an odd
/// side repeats its last row or column). Empty for a 1 × 1 image.
pub fn mip_levels(image: &RgbaImage) -> Vec<RgbaImage> {
    let mut levels: Vec<RgbaImage> = Vec::new();
    loop {
        let prev = levels.last().unwrap_or(image);
        let (w, h) = prev.dimensions();
        if w <= 1 && h <= 1 {
            break;
        }
        let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
        let src = prev.as_raw();
        let mut out = vec![0u8; (nw * nh * 4) as usize];
        for (y, row) in out.chunks_exact_mut((nw * 4) as usize).enumerate() {
            let y = y as u32;
            let (y0, y1) = ((2 * y).min(h - 1), (2 * y + 1).min(h - 1));
            for (x, px) in row.as_chunks_mut::<4>().0.iter_mut().enumerate() {
                let x = x as u32;
                let (x0, x1) = ((2 * x).min(w - 1), (2 * x + 1).min(w - 1));
                let at = |xx: u32, yy: u32| ((yy * w + xx) * 4) as usize;
                let (a, b, c, d) = (at(x0, y0), at(x1, y0), at(x0, y1), at(x1, y1));
                for (k, v) in px.iter_mut().enumerate() {
                    let sum = u32::from(src[a + k])
                        + u32::from(src[b + k])
                        + u32::from(src[c + k])
                        + u32::from(src[d + k]);
                    *v = ((sum + 2) / 4) as u8;
                }
            }
        }
        levels.push(RgbaImage::from_raw(nw, nh, out).expect("size matches"));
    }
    levels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mip_levels_halve_to_one() {
        let mut img = RgbaImage::new(5, 2);
        for (x, _, p) in img.enumerate_pixels_mut() {
            *p = image::Rgba([if x % 2 == 0 { 0 } else { 200 }, 10, 20, 255]);
        }
        let sizes: Vec<_> = mip_levels(&img).iter().map(|l| l.dimensions()).collect();
        assert_eq!(sizes, [(2, 1), (1, 1)]);
        assert_eq!(mip_levels(&img)[0].get_pixel(0, 0).0, [100, 10, 20, 255]);
        assert!(mip_levels(&RgbaImage::new(1, 1)).is_empty());
    }

    /// The place of vertex `v` for the tests: on a line, so they can tell the corners apart.
    fn place(v: u32) -> [f32; 3] {
        [v as f32, 0.0, 0.0]
    }

    #[test]
    fn rim_is_what_one_triangle_has() {
        let color = mark_color(1);
        // One triangle: all of it is rim.
        let one = mark_triangles(&[[4, 5, 6]], place, color);
        assert_eq!(one[0].pos, [place(4), place(5), place(6)]);
        assert_eq!(one[0].boundary, [true; 3]);
        assert_eq!(one[0].boundary_corners, [true; 3]);
        assert_eq!(one[0].color, color);

        // A square of 2 × 2 cells, vertices numbered row by row (0 1 2 / 3 4 5 / 6 7 8): the
        // middle 4 is the only vertex inside, the edges along the border are the rim.
        let v = |i: u32, j: u32| 3 * j + i;
        let mut tris = Vec::new();
        for j in 0..2 {
            for i in 0..2 {
                tris.push([v(i, j), v(i + 1, j), v(i + 1, j + 1)]);
                tris.push([v(i, j), v(i + 1, j + 1), v(i, j + 1)]);
            }
        }
        let marks = mark_triangles(&tris, place, color);
        let edges: usize = marks
            .iter()
            .map(|m| m.boundary.iter().filter(|&&b| b).count())
            .sum();
        assert_eq!(edges, 8);
        for (m, t) in marks.iter().zip(&tris) {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let along_border = |x: u32, y: u32| {
                    let (xi, xj, yi, yj) = (x % 3, x / 3, y % 3, y / 3);
                    (xi == yi && (xi == 0 || xi == 2)) || (xj == yj && (xj == 0 || xj == 2))
                };
                assert_eq!(m.boundary[k], along_border(a, b), "{t:?} edge {k}");
                assert_eq!(m.boundary_corners[k], t[k] != 4, "{t:?} corner {k}");
            }
        }
        // the cell at the lower left: its lower triangle has the lower edge, its upper one the
        // left edge
        assert_eq!(marks[0].boundary, [true, false, false]);
        assert_eq!(marks[1].boundary, [false, false, true]);
        // the same Stelle in another order gives the same rim
        let mut reversed = tris.clone();
        reversed.reverse();
        let back = mark_triangles(&reversed, place, color);
        assert!(back.iter().rev().eq(marks.iter()));
    }

    #[test]
    fn mark_colors_cycle_through_scan_colors() {
        assert_eq!(mark_color(1), [230, 159, 0, MARK_TINT]);
        assert_eq!(mark_color(2)[..3], SCAN_COLORS[1]);
        assert_eq!(mark_color(9), mark_color(1));
        // no Stelle 0, but no panic either
        assert_eq!(mark_color(0), mark_color(1));
    }

    #[test]
    fn mark_vertex_bits() {
        let t = MarkTriangle {
            pos: [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            boundary: [true, false, true],
            boundary_corners: [false, true, true],
            color: [1, 2, 3, 4],
        };
        let v = t.vertices();
        assert!(v
            .iter()
            .all(|c| c.rim == 0b110_101 && c.color == [1, 2, 3, 4]));
        assert_eq!(v.map(|c| c.pos), t.pos);
        assert_eq!(MARK_VERTEX_BYTES, 20);
    }
}
