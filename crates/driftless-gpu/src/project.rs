//! Fisheye projection of many points on the GPU.

use crate::GpuContext;
use bytemuck::{Pod, Zeroable};
use driftless_io::calib::CameraModel;
use driftless_io::camera::Camera;
use nalgebra::Isometry3;

/// WGSL camera model: `struct Camera` (layout of [`GpuCamera`]) and
/// `fn camera_project(cam: Camera, p: vec3<f32>) -> vec3<f32>` (`.xy` pixel, `.z` 1 if valid).
pub const CAMERA_WGSL: &str = include_str!("camera.wgsl");

/// [`Camera`] in the layout of the WGSL `struct Camera` (48 bytes, uniform-compatible).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct GpuCamera {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    /// Kannala-Brandt k1..k4 (zero for pinhole).
    pub k: [f32; 4],
    pub width: u32,
    pub height: u32,
    pub max_theta: f32,
    /// [`GpuCamera::MODEL_FISHEYE_KB`] or [`GpuCamera::MODEL_PINHOLE`].
    pub model: u32,
}

impl GpuCamera {
    pub const MODEL_FISHEYE_KB: u32 = 0;
    pub const MODEL_PINHOLE: u32 = 1;

    pub fn from_camera(cam: &Camera) -> GpuCamera {
        let (k, model) = match &cam.model {
            CameraModel::FisheyeKB { k } => (k.map(|v| v as f32), GpuCamera::MODEL_FISHEYE_KB),
            CameraModel::Pinhole => ([0.0; 4], GpuCamera::MODEL_PINHOLE),
        };
        GpuCamera {
            fx: cam.fx as f32,
            fy: cam.fy as f32,
            cx: cam.cx as f32,
            cy: cam.cy as f32,
            k,
            width: cam.width,
            height: cam.height,
            max_theta: cam.max_theta as f32,
            model,
        }
    }
}

impl From<&Camera> for GpuCamera {
    fn from(cam: &Camera) -> GpuCamera {
        GpuCamera::from_camera(cam)
    }
}

/// Uniform block of `project.wgsl`.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Params {
    cam: GpuCamera,
    /// Rows of `cam_from_world` (rotation | translation).
    rows: [[f32; 4]; 3],
    count: u32,
    _pad: [u32; 3],
}

/// Points per dispatch (keeps buffers small and the workgroup count below 65535).
const CHUNK: usize = 1 << 20;
const WORKGROUP: u32 = 64;

/// Projects world points `xyz` into the camera at `world_from_cam`: pixel coordinates as
/// [`Camera::project`] gives them (in f32), `None` where that gives `None`.
pub fn project_points(
    ctx: &GpuContext,
    cam: &GpuCamera,
    world_from_cam: &Isometry3<f64>,
    xyz: &[[f32; 3]],
) -> Vec<Option<[f32; 2]>> {
    if xyz.is_empty() {
        return Vec::new();
    }
    let device = ctx.device();
    let queue = ctx.queue();
    let pipeline = ctx
        .project_pipeline
        .get_or_init(|| project_pipeline(device));
    let m = world_from_cam.inverse().to_homogeneous();
    let rows = [0, 1, 2].map(|i| [0, 1, 2, 3].map(|j| m[(i, j)] as f32));

    let chunk = xyz.len().min(CHUNK);
    let uniform = buffer(
        device,
        "Projektion Parameter",
        size_of::<Params>(),
        wgpu::BufferUsages::UNIFORM,
    );
    let points = buffer(
        device,
        "Projektion Punkte",
        chunk * 12,
        wgpu::BufferUsages::STORAGE,
    );
    let out = buffer(
        device,
        "Projektion Ergebnis",
        chunk * 8,
        wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
    );
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Projektion Rücklesen"),
        size: (chunk * 8) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("Projektion"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: points.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: out.as_entire_binding(),
            },
        ],
    });

    let mut result = Vec::with_capacity(xyz.len());
    for part in xyz.chunks(CHUNK) {
        let params = Params {
            cam: *cam,
            rows,
            count: part.len() as u32,
            _pad: [0; 3],
        };
        queue.write_buffer(&uniform, 0, bytemuck::bytes_of(&params));
        queue.write_buffer(&points, 0, bytemuck::cast_slice(part));
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Projektion"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Projektion"),
                timestamp_writes: None,
            });
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups((part.len() as u32).div_ceil(WORKGROUP), 1, 1);
        }
        let bytes = (part.len() * 8) as u64;
        encoder.copy_buffer_to_buffer(&out, 0, &readback, 0, bytes);
        queue.submit([encoder.finish()]);

        let (tx, rx) = std::sync::mpsc::channel();
        readback.map_async(wgpu::MapMode::Read, ..bytes, move |r| {
            let _ = tx.send(r);
        });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("Grafikkarte antwortet nicht");
        rx.recv()
            .expect("Rücklesen abgebrochen")
            .expect("Ergebnis der Grafikkarte nicht lesbar");
        {
            let view = readback.get_mapped_range(..bytes).expect("Rücklesen");
            let uv: &[[f32; 2]] = bytemuck::cast_slice(&view);
            result.extend(uv.iter().map(|&[u, v]| (u >= 0.0).then_some([u, v])));
        }
        readback.unmap();
    }
    result
}

fn buffer(
    device: &wgpu::Device,
    label: &str,
    size: usize,
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: size as u64,
        usage: usage | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

fn project_pipeline(device: &wgpu::Device) -> wgpu::ComputePipeline {
    let source = format!("{CAMERA_WGSL}\n{}", include_str!("project.wgsl"));
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("Projektion"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("Projektion"),
        layout: None,
        module: &module,
        entry_point: Some("project_main"),
        compilation_options: Default::default(),
        cache: None,
    })
}
