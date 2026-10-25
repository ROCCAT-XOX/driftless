//! The textured surface on the GPU: one vertex and index buffer and one texture (with its
//! mipmaps) per page, drawn unlit (the pages are photos) into the same pass and the same
//! depth buffer as the points, so both hide each other where they should.
//!
//! Positions are relative to the surface's origin; a uniform offset (surface origin minus
//! the view's origin, see [`MeshRenderer::set_view_origin`]) moves them into the frame of
//! the cloud and the camera. The height cut applies per fragment.

use super::{FrameParams, ShowMode, DEPTH_FORMAT};
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

/// The uniform block of `mesh.wgsl` (`Mesh`).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct MeshUniforms {
    view_proj: [[f32; 4]; 4],
    offset: [f32; 4],
    z_range: [f32; 4],
}

/// A page on the GPU.
struct Page {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
    /// Its texture and sampler (group 1).
    bind: wgpu::BindGroup,
}

/// Draws a [`LoadedSurface`] uploaded with [`MeshRenderer::upload`].
pub struct MeshRenderer {
    pipeline: wgpu::RenderPipeline,
    uniforms: wgpu::Buffer,
    uniform_bind: wgpu::BindGroup,
    page_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    pages: Vec<Page>,
    origin: [f64; 3],
    view_origin: [f64; 3],
    /// A surface being uploaded page by page: its origin and the pages so far.
    incoming: Option<([f64; 3], Vec<Page>)>,
}

impl MeshRenderer {
    /// The pipeline for a colour target of `format` and a depth target of [`DEPTH_FORMAT`].
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
            uniforms,
            uniform_bind,
            page_layout,
            sampler,
            pages: Vec::new(),
            origin: [0.0; 3],
            view_origin: [0.0; 3],
            incoming: None,
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
        if indices.is_empty() || vertices.is_empty() {
            return 0;
        }
        let made;
        let mips = match surface.mips.get(i) {
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
        let buffer = |label, contents: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents,
                usage,
            })
        };
        let vertex_bytes: &[u8] = bytemuck::cast_slice(vertices);
        let index_bytes: &[u8] = bytemuck::cast_slice(indices);
        bytes += (vertex_bytes.len() + index_bytes.len()) as u64;
        let page = Page {
            vertices: buffer("Oberfläche Ecken", vertex_bytes, wgpu::BufferUsages::VERTEX),
            indices: buffer(
                "Oberfläche Dreiecke",
                index_bytes,
                wgpu::BufferUsages::INDEX,
            ),
            index_count: indices.len() as u32,
            bind,
        };
        let incoming = self
            .incoming
            .get_or_insert_with(|| (surface.origin, Vec::new()));
        incoming.1.push(page);
        bytes
    }

    /// Shows the pages uploaded since [`MeshRenderer::begin_upload`] instead of the previous
    /// surface; the view's origin becomes the surface's until
    /// [`MeshRenderer::set_view_origin`] says otherwise.
    pub fn finish_upload(&mut self) {
        if let Some((origin, pages)) = self.incoming.take() {
            self.pages = pages;
            self.origin = origin;
            self.view_origin = origin;
        }
    }

    /// Removes the surface (and one being uploaded).
    pub fn clear(&mut self) {
        self.pages.clear();
        self.incoming = None;
    }

    /// Whether nothing is uploaded.
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
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
        };
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(&u));
    }

    /// Draws every page into `pass` (after [`MeshRenderer::prepare`]); nothing for
    /// [`ShowMode::Cloud`].
    pub fn render(&self, pass: &mut wgpu::RenderPass<'_>, frame: &FrameParams) {
        if frame.show == ShowMode::Cloud || self.pages.is_empty() {
            return;
        }
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.uniform_bind, &[]);
        for page in &self.pages {
            pass.set_bind_group(1, &page.bind, &[]);
            pass.set_vertex_buffer(0, page.vertices.slice(..));
            pass.set_index_buffer(page.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..page.index_count, 0, 0..1);
        }
    }
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
}
