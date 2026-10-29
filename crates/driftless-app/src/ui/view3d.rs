//! The 3D view in the middle of the window: the cloud drawn offscreen with
//! [`PointRenderer`] into a texture of the panel's size in physical pixels, which egui then
//! shows as an image. Every frame chooses the nodes to draw for the camera
//! ([`select_nodes`]) within the point budget; a frame whose camera, size and settings are
//! unchanged is not drawn again.
//!
//! Mouse and keys: the left button orbits (in fly mode: looks), with the tool „Verschieben“
//! ([`Tool::Pan`]) it pans; panning with the right or middle button or Shift, zooming with the
//! wheel; flying with W/A/S/D and Q/E, looking with the right button, Shift faster; F or a
//! double click into the empty = show everything. A click without dragging picks a point with
//! the tools „Punkt wählen“ and „Messen“ ([`Tool::picks_points`]), and in the mode „Stellen
//! wählen“ a pose of the trajectory whatever the tool. The info, the tools and the
//! measurements over the view are drawn by [`super::tools`].
//!
//! With the tool „Pinsel“ ([`Tool::Brush`]) the left button paints from the moment it is
//! pressed (with ⌥/Alt it erases): the view moves nothing then and hands the way of the pointer
//! to the window ([`View3d::brush_input`], [`super::brush`]). Shift + left and the middle button
//! pan, the right button orbits (in fly mode: looks), and a double click frames nothing.
//!
//! Over the cloud the view draws the run of [`ViewOverlay`] (trajectory, odometry, loops,
//! anchors, picked places); without a cloud the run alone. The height cut applies to the
//! points and the surface only: the trajectory is neither cut nor kept from being clicked by
//! it, so the places of a loop can be chosen with the ceiling cut away or not.
//!
//! A surface ([`View3d::set_surface`]) is drawn with [`MeshRenderer`] in the same depth
//! buffer, moved into the frame of the cloud (else its own); „Oberfläche /
//! Punktwolke / beides“ ([`ShowMode`]) picks what is drawn. It goes to the GPU a few pages per
//! frame (about [`UPLOAD_SLICE_BYTES`]), the surface shown before stays until the last page is
//! up. Picking still looks at points only.
//!
//! Over the surface the view marks the Stellen ([`View3d::set_marks`]) and draws its touched-up
//! state („Nachher“, [`View3d::set_edit`], [`View3d::show_edit`]). Both belong to the surface
//! last given: they wait until it is on the GPU, and a new surface drops them.
//!
//! A camera kept in absolute coordinates (a report of „Melden“) is set once what was given is
//! on the GPU and has framed itself ([`View3d::restore_camera`]), in the frame the view then
//! draws in ([`View3d::view_origin`]).

use super::tools::Tool;
use super::{text, ViewSettings};
use crate::camera::{fly_motion, Camera, CameraMode, CameraState, FlyKeys, ZOOM_FACTOR};
use crate::cloud::{LoadedCloud, DEFAULT_POINT_M};
use crate::lod::{select_nodes, Octree, PointVertex};
use crate::overlay::RunOverlay;
use crate::overlay::{COMPARE_B_COLOR, COMPARE_COLOR, PICK_COLOR};
use crate::pick::{pick_point_spots, project, to_physical, Spots};
use crate::render::mesh::{MarkTriangle, MeshEdit, MeshRenderer};
use crate::render::{
    ColorMode, FrameParams, PointRenderer, PointSize, RenderTarget, ShowMode, NO_CUT,
};
use crate::surface::LoadedSurface;
use eframe::egui::{self, Color32, Key, PointerButton, Rect, Sense, Ui};
use egui_wgpu::wgpu;
use std::time::{Duration, Instant};

pub use crate::cloud::view_bounds;

/// Format of the offscreen image. Not an sRGB format: the shaders write the LAS sRGB bytes
/// as they are, and egui expects textures that are not sRGB-aware (it would convert twice).
pub const VIEW_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
/// A press and release closer than this (points) and …
pub const CLICK_MAX_MOVE: f32 = 4.0;
/// … shorter than this (s) is a click, not a drag.
pub const CLICK_MAX_SECONDS: f64 = 0.3;
/// Points within this distance (points) of the mouse can be picked.
const PICK_RADIUS: f32 = 6.0;
/// Diameter of the largest markers – the picked point and the picked places (points); the
/// submap anchors are half as large.
pub const MARKER_PX: f32 = 14.0;
/// Largest diameter of a world-size spot (points), so that nothing near the eye covers the
/// view.
pub const SPOT_MAX_PX: f32 = 24.0;
/// One step of the mouse wheel (points of scrolling).
const WHEEL_STEP: f32 = 50.0;
/// Flying speed: this fraction of the cloud's diagonal per second, within the bounds (m/s).
const FLY_SPEED_PER_DIAGONAL: f32 = 0.05;
const FLY_SPEED_MIN: f32 = 1.0;
const FLY_SPEED_MAX: f32 = 50.0;
/// Most bytes of a new cloud written to the GPU per frame, so that uploading 1 GB does not
/// stop the window: about 20 frames for 50 million points.
pub const UPLOAD_SLICE_BYTES: u64 = 50_000_000;
/// Two frames drawn closer than this count as continuous drawing (frame-to-frame time).
const CONTINUOUS: Duration = Duration::from_millis(500);
/// While GPU times are on their way, the view looks again this often.
const TIMER_POLL: Duration = Duration::from_millis(16);
/// Clear colour of the view's target: transparent, so the gradient of
/// [`super::theme::view_background`] shows behind the points.
const CLEAR: [f32; 4] = [0.0; 4];
const MARKER_COLOR: u32 = 0xffff_ffff;
/// The legend of a comparison starts this far below the top of the view (points), under the
/// floating switch at the top right.
const LEGEND_TOP: f32 = 56.0;

/// What the view reports from a click.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewEvent {
    /// The index of the clicked point in the cloud's [`crate::cloud::PointSet`].
    PickedPoint(u32),
    /// The index of the clicked pose of the trajectory ([`RunOverlay::poses`]).
    PickedPose(usize),
    /// A click in the mode „Stellen wählen“ while the trajectory is hidden: nothing picked.
    TrajectoryHidden,
}

/// The run drawn over the cloud.
#[derive(Debug, Clone, Copy)]
pub struct ViewOverlay<'a> {
    pub run: &'a RunOverlay,
    /// Raised by the program whenever `run` changed (also its origin).
    pub version: u64,
    /// The poses of the places A and B, marked and labelled.
    pub places: [Option<usize>; 2],
    /// While choosing B: the submap of A, which a click at a crossing avoids.
    pub avoid_submap: Option<usize>,
    /// A click picks a pose instead of a point („Stellen wählen“).
    pub pick_places: bool,
}

/// What the overlay was built from; an equal key needs no new overlay.
#[derive(Debug, Clone, PartialEq)]
struct OverlayKey {
    version: Option<u64>,
    layers: [bool; 6],
    places: [Option<usize>; 2],
    picked: Option<u32>,
}

/// The legend of a comparison drawn over the run, in the upper right corner: „A: …“ in the
/// colour of A, „B: …“ in that of B.
fn paint_legend(painter: &egui::Painter, rect: Rect, run: &RunOverlay, font: egui::FontId) {
    let Some(names) = run
        .compare_legend
        .as_ref()
        .filter(|_| !run.compare.is_empty())
    else {
        return;
    };
    let mut y = rect.top() + LEGEND_TOP;
    for (name, [r, g, b]) in names.iter().zip([COMPARE_COLOR, COMPARE_B_COLOR]) {
        let color = Color32::from_rgb(r, g, b);
        let galley = painter.layout_no_wrap(name.clone(), font.clone(), color);
        let at = egui::pos2(rect.right() - 8.0 - galley.size().x, y);
        let back = Rect::from_min_size(at, galley.size()).expand(3.0);
        painter.rect_filled(back, 3.0, super::theme::DARK.float_fill);
        y = back.bottom() + 4.0;
        painter.galley(at, galley, color);
    }
}

/// How the points are drawn in world size, for picking on their spots; `None` in the pixel
/// mode. The same size as the frame: `point_m` of the settings or of the map, between the
/// point size of the settings and [`SPOT_MAX_PX`].
fn spots(settings: &ViewSettings, cloud: Option<&LoadedCloud>, ppp: f32) -> Option<Spots> {
    if settings.point_size != PointSize::World {
        return None;
    }
    let point_px = (settings.point_px * ppp).max(1.0);
    Some(Spots {
        point_m: settings
            .point_m
            .unwrap_or_else(|| cloud.map_or(DEFAULT_POINT_M, |c| c.point_m)),
        px_range: [point_px, (SPOT_MAX_PX * ppp).max(point_px)],
    })
}

/// The size of the offscreen image for a panel of `panel_points` at `ppp` pixels per
/// point: rounded physical pixels, at least 1 × 1.
pub fn target_size(panel_points: egui::Vec2, ppp: f32) -> [u32; 2] {
    [panel_points.x, panel_points.y].map(|v| {
        let px = (v * ppp).round();
        if px.is_finite() && px >= 1.0 {
            px.min(u32::MAX as f32) as u32
        } else {
            1
        }
    })
}

/// Whether a press at time `press` (s) and a release at `release`, with the pointer at most
/// `moved_points` away from where it was pressed, is a click (else a drag).
pub fn is_click(press: f64, release: f64, moved_points: f32) -> bool {
    moved_points < CLICK_MAX_MOVE && release - press < CLICK_MAX_SECONDS
}

/// The fly keys held, given which keys are down: W/S forward/back, A/D left/right, E/Q
/// up/down.
pub fn fly_keys(down: impl Fn(Key) -> bool) -> FlyKeys {
    FlyKeys {
        forward: down(Key::W),
        back: down(Key::S),
        left: down(Key::A),
        right: down(Key::D),
        up: down(Key::E),
        down: down(Key::Q),
    }
}

/// The colour modes a cloud offers: without colours no [`ColorMode::Rgb`], „je Scan“
/// ([`ColorMode::Scan`]) only for a cloud of several scans.
pub fn color_modes(has_rgb: bool, has_scans: bool) -> Vec<ColorMode> {
    let mut modes = vec![ColorMode::Intensity, ColorMode::Quality];
    if has_rgb {
        modes.insert(0, ColorMode::Rgb);
    }
    if has_scans {
        modes.push(ColorMode::Scan);
    }
    modes
}

/// The mode drawn: [`ColorMode::Rgb`] on a cloud without colours shows the intensity,
/// [`ColorMode::Scan`] on a cloud of one scan what [`ColorMode::Rgb`] would.
pub fn effective_color_mode(mode: ColorMode, has_rgb: bool, has_scans: bool) -> ColorMode {
    match mode {
        ColorMode::Scan if !has_scans => effective_color_mode(ColorMode::Rgb, has_rgb, false),
        ColorMode::Rgb if !has_rgb => ColorMode::Intensity,
        _ => mode,
    }
}

/// What is drawn for the setting `mode`: without a surface the points, without a cloud the
/// surface, else as set.
pub fn effective_show(mode: ShowMode, has_surface: bool, has_cloud: bool) -> ShowMode {
    match (has_surface, has_cloud) {
        (false, _) => ShowMode::Cloud,
        (true, false) => ShowMode::Surface,
        (true, true) => mode,
    }
}

/// What the properties show of a picked point.
#[derive(Debug, Clone, PartialEq)]
pub struct PointInfo {
    /// Index in the cloud's point set.
    pub index: u32,
    /// Absolute: the cloud's origin (f64) plus the relative position.
    pub position: [f64; 3],
    /// Seconds since the start of the scan.
    pub t: f32,
    pub intensity: u8,
    /// `None` for a cloud without colours or a point without measured colour.
    pub rgb: Option<[u8; 3]>,
    /// Quality of the point's submap × 255; `None` without SLAM report.
    pub quality: Option<u8>,
    /// The removal reason of a point of `removed.las` (its flags), else `None`.
    pub removed: Option<u8>,
    /// The scan of a point of a merged model ([`crate::cloud::PointSet::source`], from 0);
    /// `None` for a cloud of one scan.
    pub scan: Option<u16>,
}

/// The info of point `index` (of the point set, not a vertex) of `cloud`.
pub fn point_info(cloud: &LoadedCloud, index: u32) -> PointInfo {
    let p = &cloud.points;
    let i = index as usize;
    let removed = cloud.is_removed().then_some(p.flags[i]);
    // Outside removed.las a flag of 1 marks a point the cameras did not colour.
    let coloured = removed.is_some() || p.flags[i] != 1;
    PointInfo {
        index,
        position: [0, 1, 2].map(|k| p.origin[k] + f64::from(p.xyz[i][k])),
        t: p.t[i],
        intensity: p.intensity[i],
        rgb: p.rgb.as_ref().map(|c| c[i]).filter(|_| coloured),
        quality: p.quality_known.then_some(p.quality[i]),
        removed,
        scan: p.source.get(i).copied(),
    }
}

/// What a frame was drawn with; an equal key needs no new frame.
#[derive(Debug, Clone, PartialEq)]
struct FrameKey {
    view_proj: [[f32; 4]; 4],
    size: [u32; 2],
    point_px: f32,
    color_mode: ColorMode,
    round: bool,
    budget: usize,
    picked: Option<u32>,
    z_range: [f32; 2],
    overlay: u64,
    show: ShowMode,
    point_size: PointSize,
    point_m: f32,
    /// How often a surface was uploaded, and the origin it is drawn in.
    surface: (u64, Option<[f64; 3]>),
    /// How often the marks and the touched-up state on the GPU were replaced.
    marks: u64,
    edit: u64,
    /// „Nachher“: the touched-up state is drawn.
    show_edit: bool,
}

/// The surface on the GPU: what the view still needs of it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SurfaceShown {
    origin: [f64; 3],
    /// Relative to `origin`.
    bounds: Option<([f32; 3], [f32; 3])>,
    triangles: usize,
}

/// What the pointer did for the brush in the last frame ([`View3d::brush_input`]): filled only
/// while the tool is [`Tool::Brush`]. Pixels are physical, from the top left of the view.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BrushInput {
    /// The size of the view (physical pixels), for the rays through its pixels.
    pub viewport: [f32; 2],
    /// Where the pointer is over the view while nothing else lies over it there, or while it
    /// paints.
    pub hover: Option<[f32; 2]>,
    /// The left button went down on the view this frame without Shift: a stroke begins.
    pub pressed: bool,
    /// ⌥/Alt: held now, while painting since the press. The stroke erases.
    pub erase: bool,
    /// The way of the pointer this frame while it paints, in order: where it was pressed
    /// first, then every place it moved to (and where it was let go).
    pub path: Vec<[f32; 2]>,
    /// The stroke ended this frame: the left button was let go (or the view lost it).
    pub released: bool,
}

/// A press of the primary button inside the view, until it is released.
#[derive(Debug, Clone, Copy)]
struct Press {
    time: f64,
    pos: egui::Pos2,
    /// Farthest the pointer got from `pos` (points).
    moved: f32,
}

/// The 3D view. Holds the camera and the GPU side; the cloud itself belongs to the program.
#[derive(Default)]
pub struct View3d {
    camera: Camera,
    renderer: Option<PointRenderer>,
    target: Option<RenderTarget>,
    texture: Option<egui::TextureId>,
    /// The nodes of the last frame (picking looks only at these).
    drawn: Vec<u32>,
    drawn_points: usize,
    frame_ms: f32,
    /// The current cloud's upload has begun (it is filled slice by slice).
    uploaded: bool,
    /// Bytes written to the GPU in the last frame.
    last_upload_bytes: u64,
    /// When the last frame was drawn (for the frame-to-frame time without GPU timer).
    last_render_at: Option<Instant>,
    /// The frame time is GPU time (else CPU time between frames).
    gpu_timed: bool,
    /// Bounds of the cloud, from the upload.
    bounds: Option<([f32; 3], [f32; 3])>,
    /// Show everything at the next frame with a usable size.
    frame_all: bool,
    last: Option<FrameKey>,
    press: Option<Press>,
    /// The picked point (index in the point set), marked in the view.
    picked: Option<u32>,
    /// What the left button and a click do.
    tool: Tool,
    /// The height cut of this frame (picking uses it too).
    z_range: [f32; 2],
    /// What the overlay on the GPU shows, and how often it was replaced.
    overlay_key: Option<OverlayKey>,
    overlay_count: u64,
    /// The run whose trajectory was framed while there was no cloud.
    framed_run: Option<std::path::PathBuf>,
    /// The surface's GPU side; `None` before the first surface.
    mesh: Option<MeshRenderer>,
    /// A surface waiting to be uploaded (its pixels are dropped once they are on the GPU),
    /// and its next page to upload.
    pending_surface: Option<LoadedSurface>,
    next_page: usize,
    surface: Option<SurfaceShown>,
    surface_count: u64,
    /// Marks and a touched-up state (`Some(None)`: none) waiting for the surface to be on the
    /// GPU, and how often those on the GPU were replaced.
    pending_marks: Option<Vec<MarkTriangle>>,
    pending_edit: Option<Option<MeshEdit>>,
    marks_count: u64,
    edit_count: u64,
    /// A touched-up state was given for the surface last given (waiting or on the GPU).
    edit_given: bool,
    /// „Nachher“: draw the touched-up state.
    show_edit: bool,
    /// A stroke of the brush is being painted: whether it erases.
    painting: Option<bool>,
    /// What the pointer did for the brush in the last frame.
    brush_input: BrushInput,
    /// The origin the camera's frame was drawn in, in the last frame drawn: the cloud's, else
    /// the surface's, else the run's.
    origin: Option<[f64; 3]>,
    /// A camera to set once what was given is up on the GPU ([`View3d::restore_camera`]).
    pending_camera: Option<CameraState>,
}

impl View3d {
    /// The program replaced the cloud: upload it and show all of it at the next frame.
    pub fn cloud_replaced(&mut self) {
        self.uploaded = false;
        self.last_render_at = None;
        self.picked = None;
        self.overlay_key = None;
        self.drawn.clear();
        self.drawn_points = 0;
        self.last = None;
    }

    /// Shows `surface` from the next frame on (replacing the one shown), or none. Marks and a
    /// touched-up state given before belong to the surface before: they are dropped.
    pub fn set_surface(&mut self, surface: Option<LoadedSurface>) {
        self.pending_marks = None;
        self.pending_edit = None;
        self.edit_given = false;
        if surface.is_none() {
            if let Some(mesh) = &mut self.mesh {
                mesh.clear();
            }
            self.surface = None;
            self.surface_count += 1;
        }
        self.pending_surface = surface;
        self.next_page = 0;
        self.last = None;
    }

    /// Marks `tris` on the surface last given ([`View3d::set_surface`]), replacing the marks
    /// before (none: empty): drawn from the next frame on, once the surface is on the GPU. See
    /// [`MeshRenderer::set_marks`] for what they must be.
    pub fn set_marks(&mut self, tris: Vec<MarkTriangle>) {
        self.pending_marks = Some(tris);
    }

    /// The touched-up state of the surface last given, replacing the one before, or none;
    /// uploaded once the surface is on the GPU ([`MeshRenderer::set_edit`]).
    pub fn set_edit(&mut self, edit: Option<MeshEdit>) {
        self.edit_given = edit.is_some();
        self.pending_edit = Some(edit);
    }

    /// Draws the touched-up state („Nachher“, `true`) or the computed surface („Vorher“) from
    /// the next frame on; uploads nothing.
    pub fn show_edit(&mut self, on: bool) {
        self.show_edit = on;
    }

    /// Whether the surface last given has a touched-up state ([`View3d::set_edit`]), on the GPU
    /// or waiting for it.
    pub fn has_edit(&self) -> bool {
        self.edit_given
    }

    /// Whether „Nachher“ is drawn ([`View3d::show_edit`]) – where there is a touched-up state.
    pub fn showing_edit(&self) -> bool {
        self.show_edit
    }

    /// Whether a surface is shown or about to be.
    pub fn has_surface(&self) -> bool {
        self.surface.is_some() || self.pending_surface.is_some()
    }

    /// Triangles of the surface on the GPU.
    pub fn surface_triangles(&self) -> Option<usize> {
        self.surface.map(|s| s.triangles)
    }

    /// Whether a surface is still being written to the GPU.
    pub fn is_uploading_surface(&self) -> bool {
        self.pending_surface.is_some()
    }

    /// Points drawn in the last frame.
    pub fn drawn_points(&self) -> usize {
        self.drawn_points
    }

    /// Time of the last frame (ms): the GPU time of its drawing where the device has
    /// timestamp queries ([`View3d::gpu_timed`]), else the time from the frame before
    /// while drawing continuously (CPU, includes waiting for the display), else the CPU
    /// time of choosing and recording. Never waits for the GPU.
    pub fn frame_ms(&self) -> f32 {
        self.frame_ms
    }

    pub fn gpu_timed(&self) -> bool {
        self.gpu_timed
    }

    /// Bytes of the cloud written to the GPU in the last frame (at most
    /// [`UPLOAD_SLICE_BYTES`]).
    pub fn last_upload_bytes(&self) -> u64 {
        self.last_upload_bytes
    }

    /// Whether the cloud is still being written to the GPU.
    pub fn is_uploading(&self) -> bool {
        !self.uploaded || self.renderer.as_ref().is_some_and(|r| !r.upload_done())
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    /// The origin the camera is relative to: that of the last frame drawn (the cloud's, else
    /// the surface's, else the run's); `None` before the first one.
    pub fn view_origin(&self) -> Option<[f64; 3]> {
        self.origin
    }

    /// Sets the camera as `state` says (absolute, [`Camera::restore`]) once the cloud and the
    /// surface given are up on the GPU and have framed themselves, in the frame the view then
    /// draws in. A camera asked for before replaces this one.
    pub fn restore_camera(&mut self, state: CameraState) {
        self.pending_camera = Some(state);
    }

    /// Whether a camera of [`View3d::restore_camera`] still waits.
    pub fn camera_pending(&self) -> bool {
        self.pending_camera.is_some()
    }

    /// For tools and measurements that move the camera themselves.
    pub fn camera_mut(&mut self) -> &mut Camera {
        &mut self.camera
    }

    /// What the left button and a click do from the next frame on.
    pub fn set_tool(&mut self, tool: Tool) {
        self.tool = tool;
    }

    /// What the pointer did for the brush in the last frame (empty unless the tool is
    /// [`Tool::Brush`]).
    pub fn brush_input(&self) -> &BrushInput {
        &self.brush_input
    }

    /// Marks point `index` of the cloud (index in the point set), or none.
    pub fn set_picked(&mut self, index: Option<u32>) {
        self.picked = index;
    }

    /// Shows everything at the next frame (the button „Alles“, as the key `F`).
    pub fn frame_all(&mut self) {
        self.frame_all = true;
    }

    /// Fills `ui` with the view of `cloud` and `overlay` (nothing to draw without either or
    /// for a panel of size 0). Returns what a click picked.
    pub fn show(
        &mut self,
        ui: &mut Ui,
        render_state: &egui_wgpu::RenderState,
        cloud: Option<&LoadedCloud>,
        overlay: Option<ViewOverlay<'_>>,
        settings: &ViewSettings,
    ) -> Option<ViewEvent> {
        // what the pointer does for the brush is told anew every frame
        self.brush_input = BrushInput::default();
        let size = ui.available_size();
        let (rect, response) = ui.allocate_exact_size(size, Sense::click_and_drag());
        // A collapsed or squeezed panel: no target, no camera change (aspect 0).
        if !(rect.width() >= 1.0 && rect.height() >= 1.0) {
            return None;
        }
        if cloud.is_none() && overlay.is_none() && !self.has_surface() {
            return None;
        }
        let ppp = ui.ctx().pixels_per_point();
        let px = target_size(rect.size(), ppp);
        let viewport = [px[0] as f32, px[1] as f32];
        let aspect = viewport[0] / viewport[1];
        let (device, queue) = (&render_state.device, &render_state.queue);

        let renderer = self
            .renderer
            .get_or_insert_with(|| PointRenderer::new(device, queue, VIEW_FORMAT));
        // stands in for the cloud while only a run is shown (allocates nothing)
        let empty = Octree::default();
        let tree = cloud.map_or(&empty, |c| &c.tree);
        if !self.uploaded {
            renderer.begin_upload(device, tree);
            self.bounds = cloud.and_then(|c| c.view_bounds);
            self.uploaded = true;
            self.frame_all = cloud.is_some();
            self.overlay_key = None;
            self.last = None;
        }
        // The frame of the camera: the cloud's, else the surface's (the program moves the
        // run there too), else the run's.
        let view_origin = cloud
            .map(|c| c.tree.origin)
            .or(self.pending_surface.as_ref().map(|s| s.origin))
            .or(self.surface.map(|s| s.origin))
            .or(overlay.map(|o| o.run.origin));
        let mut surface_new = false;
        if let Some(surface) = &self.pending_surface {
            let mesh = self
                .mesh
                .get_or_insert_with(|| MeshRenderer::new(device, VIEW_FORMAT));
            if self.next_page == 0 {
                mesh.begin_upload(surface.origin);
            }
            // pages until a slice is full: a 4096² page with its mipmaps is about 85 MB
            let mut bytes = 0;
            while self.next_page < surface.pages.len() && bytes < UPLOAD_SLICE_BYTES {
                bytes += mesh.upload_page(device, queue, surface, self.next_page);
                self.next_page += 1;
            }
            if self.next_page >= surface.pages.len() {
                mesh.finish_upload();
                self.surface = Some(SurfaceShown {
                    origin: surface.origin,
                    bounds: surface.bounds(),
                    triangles: surface.triangles(),
                });
                self.surface_count += 1;
                surface_new = true;
                self.pending_surface = None;
                self.next_page = 0;
            } else {
                ui.ctx().request_repaint();
            }
        }
        if let (Some(mesh), Some(origin)) = (&mut self.mesh, view_origin) {
            mesh.set_view_origin(origin);
        }
        if let Some(mesh) = &mut self.mesh {
            // „Vorher/Nachher“ also for the surface still shown while the next one uploads.
            mesh.show_edit(self.show_edit);
            // Marks and touched-up state once their surface is up.
            if self.pending_surface.is_none() {
                if let Some(tris) = self.pending_marks.take() {
                    mesh.set_marks(device, queue, &tris);
                    self.marks_count += 1;
                }
                if let Some(edit) = self.pending_edit.take() {
                    mesh.set_edit(device, queue, edit);
                    self.edit_count += 1;
                }
            }
        }
        // The surface's bounds in the camera's frame.
        let surface_bounds = self.surface.and_then(|s| {
            let (lo, hi) = s.bounds?;
            let o = view_origin.unwrap_or(s.origin);
            let shift = [0, 1, 2].map(|k| (s.origin[k] - o[k]) as f32);
            Some((
                [0, 1, 2].map(|k| lo[k] + shift[k]),
                [0, 1, 2].map(|k| hi[k] + shift[k]),
            ))
        });
        if cloud.is_none() {
            // without a cloud: frame the surface when it comes, else the run once
            let run = overlay.map(|o| o.run);
            if surface_new && surface_bounds.is_some() {
                self.bounds = surface_bounds;
                self.frame_all = true;
            } else if run.map(|r| &r.run) != self.framed_run.as_ref() {
                self.framed_run = run.map(|r| r.run.clone());
                self.bounds = surface_bounds.or(run.and_then(RunOverlay::bounds));
                self.frame_all = true;
            }
        }
        self.last_upload_bytes = 0;
        if !renderer.upload_done() {
            // A slice per frame; the window stays usable meanwhile.
            self.last_upload_bytes = renderer.upload_step(queue, tree, UPLOAD_SLICE_BYTES);
            let percent = (renderer.upload_fraction() * 100.0).round();
            // the same gradient as behind the points, so the view does not jump
            super::theme::view_background(ui.painter(), rect);
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                text::uploading(percent),
                egui::TextStyle::Body.resolve(ui.style()),
                super::theme::DARK.text_dim,
            );
            ui.ctx().request_repaint();
            return None;
        }
        if self.frame_all {
            if let Some((min, max)) = self.bounds {
                self.camera.frame_bounds(min, max, aspect);
            }
            self.frame_all = false;
        }
        self.origin = view_origin;
        // a camera asked for once the surface is up too (it framed itself above)
        if self.pending_surface.is_none() {
            if let (Some(state), Some(origin)) = (&self.pending_camera, view_origin) {
                self.camera.restore(state, origin);
                self.pending_camera = None;
            }
        }
        self.camera.set_mode(settings.camera_mode);

        self.z_range = settings.cut.unwrap_or(NO_CUT);
        let event = self.input(ui, &response, rect, ppp, viewport, cloud, overlay, settings);
        self.update_overlay(device, cloud, overlay, settings);

        // The target follows the panel size; egui shows it under one texture id.
        if self.target.as_ref().map(|t| t.size) != Some(px) {
            let target = RenderTarget::new(device, px, VIEW_FORMAT);
            let mut egui_renderer = render_state.renderer.write();
            let filter = wgpu::FilterMode::Nearest;
            match self.texture {
                Some(id) => egui_renderer.update_egui_texture_from_wgpu_texture(
                    device,
                    &target.color_view,
                    filter,
                    id,
                ),
                None => {
                    self.texture = Some(egui_renderer.register_native_texture(
                        device,
                        &target.color_view,
                        filter,
                    ))
                }
            }
            self.target = Some(target);
            self.last = None;
        }

        let has_rgb = cloud.is_some_and(|c| c.points.rgb.is_some());
        let has_scans = cloud.is_some_and(|c| !c.points.source.is_empty());
        let point_px = (settings.point_px * ppp).max(1.0);
        let key = FrameKey {
            view_proj: self.camera.view_proj(aspect).into(),
            size: px,
            point_px,
            color_mode: effective_color_mode(settings.color_mode, has_rgb, has_scans),
            round: settings.round,
            budget: settings.budget,
            picked: self.picked,
            z_range: settings.cut.unwrap_or(NO_CUT),
            overlay: self.overlay_count,
            show: effective_show(settings.show, self.surface.is_some(), cloud.is_some()),
            point_size: settings.point_size,
            point_m: settings
                .point_m
                .unwrap_or_else(|| cloud.map_or(DEFAULT_POINT_M, |c| c.point_m)),
            surface: (self.surface_count, view_origin),
            marks: self.marks_count,
            edit: self.edit_count,
            show_edit: self.show_edit,
        };
        if self.last.as_ref() != Some(&key) {
            let started = Instant::now();
            // Refine while the points of a node are farther apart on screen than drawn.
            let info = self.camera.view_info(viewport, point_px);
            // the surface alone: no points, none to pick
            self.drawn = match cloud {
                Some(c) if key.show != ShowMode::Surface => {
                    select_nodes(&c.tree, &info, settings.budget)
                }
                _ => Vec::new(),
            };
            self.drawn_points = self
                .drawn
                .iter()
                .map(|&i| tree.nodes[i as usize].count as usize)
                .sum();
            let frame = FrameParams {
                view_proj: key.view_proj,
                viewport_px: viewport,
                point_px,
                marker_px: MARKER_PX * ppp,
                line_scale: ppp,
                color_mode: key.color_mode,
                round: key.round,
                background: CLEAR,
                quality_known: cloud.is_some_and(|c| c.points.quality_known),
                z_range: key.z_range,
                show: key.show,
                point_size: key.point_size,
                point_m: key.point_m,
                // the setting is the smallest spot, so that far ones never vanish
                point_px_range: [point_px, (SPOT_MAX_PX * ppp).max(point_px)],
            };
            if let (Some(target), Some(renderer)) = (&self.target, &mut self.renderer) {
                // `render_with` writes the shared uniform buffers and submits its own work,
                // so this frame is complete before anything else is drawn into the view.
                let mesh = self.mesh.as_ref().filter(|m| !m.is_empty());
                renderer.render_with(device, queue, target, &frame, tree, &self.drawn, mesh);
            }
            let now = Instant::now();
            self.frame_ms = match self.last_render_at {
                Some(prev) if now - prev < CONTINUOUS => (now - prev).as_secs_f32() * 1000.0,
                _ => (now - started).as_secs_f32() * 1000.0,
            };
            self.last_render_at = Some(now);
            self.last = Some(key);
        }

        if let Some(renderer) = &mut self.renderer {
            if let Some(ms) = renderer.gpu_ms(device) {
                self.frame_ms = ms;
                self.gpu_timed = true;
            }
            if renderer.gpu_timer_pending() {
                ui.ctx().request_repaint_after(TIMER_POLL);
            }
        }
        if let Some(id) = self.texture {
            let uv = Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
            super::theme::view_background(ui.painter(), rect);
            ui.painter().image(id, rect, uv, Color32::WHITE);
        }
        let painter = ui.painter_at(rect);
        if let Some(o) = overlay {
            let body = egui::TextStyle::Body.resolve(ui.style());
            if settings.show_compare {
                paint_legend(&painter, rect, o.run, body.clone());
            }
            let hover = response.hover_pos();
            self.paint_places(&painter, rect, ppp, viewport, o, settings, hover, body);
        }
        event
    }

    /// The labels „A“ and „B“ next to the places, and in the mode „Stellen wählen“ the time
    /// and submap of the pose under the mouse.
    #[allow(clippy::too_many_arguments)]
    fn paint_places(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        o: ViewOverlay<'_>,
        settings: &ViewSettings,
        hover: Option<egui::Pos2>,
        font: egui::FontId,
    ) {
        let vp = self.camera.view_proj(viewport[0] / viewport[1]);
        let on_screen = |pose: usize| {
            let p = project(&vp, *o.run.positions.get(pose)?, viewport)?;
            (p[2] <= 1.0).then(|| rect.min + egui::vec2(p[0], p[1]) / ppp)
        };
        let label = |at: egui::Pos2, s: String, color: Color32| {
            let galley = painter.layout_no_wrap(s, font.clone(), color);
            let r = Rect::from_min_size(at, galley.size()).expand(2.0);
            painter.rect_filled(r, 3.0, super::theme::DARK.float_fill);
            painter.galley(at, galley, color);
        };
        let [r, g, b] = PICK_COLOR;
        let pick = Color32::from_rgb(r, g, b);
        // A to the upper left, B to the upper right: a loop's places often lie on each other
        let offsets = [
            egui::vec2(-MARKER_PX * 1.4, -MARKER_PX * 1.6),
            egui::vec2(MARKER_PX * 0.6, -MARKER_PX * 1.6),
        ];
        let names = [text::PLACE_A_SHORT, text::PLACE_B_SHORT];
        for ((pose, name), offset) in o.places.iter().zip(names).zip(offsets) {
            if let Some(at) = pose.and_then(on_screen) {
                label(at + offset, name.into(), pick);
            }
        }
        let Some(pos) = hover.filter(|_| o.pick_places && settings.show_trajectory) else {
            return;
        };
        let pixel = to_physical([pos.x, pos.y], [rect.min.x, rect.min.y], ppp);
        if let Some(i) = o
            .run
            .pick(&vp, viewport, pixel, PICK_RADIUS * ppp, o.avoid_submap)
        {
            let s = text::pose_hover(o.run.time_of(i), o.run.submap_of(i));
            label(pos + egui::vec2(14.0, 10.0), s, super::theme::DARK.text);
        }
    }

    /// Puts the run of `overlay` and the marker of the picked point on the GPU when they
    /// changed.
    fn update_overlay(
        &mut self,
        device: &wgpu::Device,
        cloud: Option<&LoadedCloud>,
        overlay: Option<ViewOverlay<'_>>,
        settings: &ViewSettings,
    ) {
        let key = OverlayKey {
            version: overlay.map(|o| o.version),
            layers: [
                settings.show_trajectory,
                settings.show_odometry,
                settings.show_loops,
                settings.show_rejected,
                settings.show_submaps,
                settings.show_compare,
            ],
            places: overlay.map_or([None; 2], |o| o.places),
            picked: self.picked,
        };
        if self.overlay_key.as_ref() == Some(&key) {
            return;
        }
        let (lines, mut markers) = overlay.map_or_else(Default::default, |o| {
            let picks: Vec<usize> = o.places.iter().flatten().copied().collect();
            o.run.lines(settings, &picks)
        });
        // the picked point first, so that picked places lie over it
        let point = self
            .picked
            .and_then(|i| cloud?.points.xyz.get(i as usize))
            .map(|&pos| PointVertex {
                pos,
                rgbi: MARKER_COLOR,
                extra: 0,
            });
        markers.splice(0..0, point);
        if let Some(renderer) = &mut self.renderer {
            renderer.set_overlay(device, &lines, &markers);
        }
        self.overlay_key = Some(key);
        self.overlay_count += 1;
    }

    /// Mouse and keys of one frame; a click picks.
    #[allow(clippy::too_many_arguments)]
    fn input(
        &mut self,
        ui: &Ui,
        response: &egui::Response,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        cloud: Option<&LoadedCloud>,
        overlay: Option<ViewOverlay<'_>>,
        settings: &ViewSettings,
    ) -> Option<ViewEvent> {
        let z_range = self.z_range;
        let ctx = ui.ctx();
        let mode = self.camera.mode;
        let shift = ctx.input(|i| i.modifiers.shift);
        let brush = self.tool == Tool::Brush;
        self.brush_stroke(ctx, response, rect, ppp, viewport, shift);
        let painting = self.painting.is_some();
        let delta = response.drag_delta() * ppp;
        let by = |b| response.dragged_by(b);
        // the tool „Verschieben“ pans with the left button, the brush paints with it (with
        // Shift it pans); every other tool turns
        let left_pans = self.tool == Tool::Pan || brush;
        match mode {
            CameraMode::Orbit => {
                if by(PointerButton::Primary) && painting {
                    // the brush paints: the view stays
                } else if (by(PointerButton::Primary) && !shift && !left_pans)
                    || (by(PointerButton::Secondary) && brush)
                {
                    // the brush turns with the right button
                    self.camera.orbit(delta.x, delta.y);
                } else if by(PointerButton::Primary)
                    || by(PointerButton::Secondary)
                    || by(PointerButton::Middle)
                {
                    self.camera.pan(delta.x, delta.y, viewport);
                }
            }
            CameraMode::Fly => {
                if by(PointerButton::Primary) && painting {
                    // the brush paints: the view stays
                } else if by(PointerButton::Primary) && left_pans {
                    self.camera.pan(delta.x, delta.y, viewport);
                } else if by(PointerButton::Primary) || by(PointerButton::Secondary) {
                    self.camera.look(delta.x, delta.y);
                } else if by(PointerButton::Middle) {
                    self.camera.pan(delta.x, delta.y, viewport);
                }
            }
        }
        if response.hovered() {
            let (scroll, zoom) = ctx.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            let mut steps = scroll / WHEEL_STEP;
            if zoom != 1.0 && zoom > 0.0 {
                steps += zoom.ln() / (1.0 / ZOOM_FACTOR).ln();
            }
            if steps != 0.0 {
                self.camera.zoom(steps);
            }
        }

        // Keys only while no text field has the keyboard.
        if super::tools::keys_free(ctx) {
            if ctx.input(|i| i.key_pressed(Key::F)) {
                self.frame_all = true;
            }
            if mode == CameraMode::Fly {
                let (keys, dt) = ctx.input(|i| (fly_keys(|k| i.key_down(k)), i.stable_dt));
                let (motion, factor) = fly_motion(keys, shift);
                if motion != [0.0; 3] {
                    let speed = self.fly_speed() * factor;
                    self.camera
                        .fly(motion[0], motion[1], motion[2], dt.min(0.1), speed);
                    ctx.request_repaint();
                }
            }
        }

        // Click or drag: decided at the release.
        let (pressed, released, pos, time) = ctx.input(|i| {
            (
                i.pointer.primary_pressed(),
                i.pointer.primary_released(),
                i.pointer.interact_pos(),
                i.time,
            )
        });
        if pressed && response.hovered() {
            self.press = pos.map(|pos| Press {
                time,
                pos,
                moved: 0.0,
            });
        }
        if let (Some(p), Some(pos)) = (&mut self.press, pos) {
            p.moved = p.moved.max(p.pos.distance(pos));
        }
        let mut event = None;
        if released {
            if let (Some(p), Some(pos)) = (self.press.take(), pos) {
                if is_click(p.time, time, p.moved) {
                    let pixel = to_physical([pos.x, pos.y], [rect.min.x, rect.min.y], ppp);
                    // the trajectory is not cut by the height cut, so its poses are not either
                    let places = overlay.filter(|o| o.pick_places);
                    event = match (places, cloud) {
                        (Some(_), _) if !settings.show_trajectory => {
                            Some(ViewEvent::TrajectoryHidden)
                        }
                        (Some(o), _) => {
                            let vp = self.camera.view_proj(viewport[0] / viewport[1]);
                            let radius = PICK_RADIUS * ppp;
                            o.run
                                .pick(&vp, viewport, pixel, radius, o.avoid_submap)
                                .map(ViewEvent::PickedPose)
                        }
                        (None, Some(cloud)) if self.tool.picks_points() => pick_point_spots(
                            &cloud.tree,
                            &self.drawn,
                            &self.camera,
                            viewport,
                            pixel,
                            PICK_RADIUS * ppp,
                            z_range,
                            spots(settings, Some(cloud), ppp),
                        )
                        .map(|v| ViewEvent::PickedPoint(cloud.tree.source[v as usize])),
                        (None, _) => None,
                    };
                }
            }
        }
        // a double click with the brush paints twice, it frames nothing
        if response.double_clicked()
            && event.is_none()
            && !brush
            && self.hit_nothing(rect, ppp, viewport, cloud, pos, settings)
        {
            self.frame_all = true;
        }
        if self.frame_all {
            ctx.request_repaint();
        }
        event
    }

    /// The brush's part of the input ([`BrushInput`]): a stroke begins where the left button
    /// goes down on the view without Shift (at once, a click paints too), follows every place
    /// the pointer moves to and ends where the button is let go. Another tool ends a stroke
    /// that runs.
    fn brush_stroke(
        &mut self,
        ctx: &egui::Context,
        response: &egui::Response,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        shift: bool,
    ) {
        let input = &mut self.brush_input;
        input.viewport = viewport;
        if self.tool != Tool::Brush {
            if self.painting.take().is_some() {
                input.released = true;
            }
            return;
        }
        let at = |p: egui::Pos2| to_physical([p.x, p.y], [rect.min.x, rect.min.y], ppp);
        let (pressed, down, alt, hover, events) = ctx.input(|i| {
            (
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.modifiers.alt,
                i.pointer.hover_pos(),
                i.events.clone(),
            )
        });
        let begins = pressed && response.hovered() && !shift && self.painting.is_none();
        if begins {
            self.painting = Some(alt);
            input.pressed = true;
        }
        if let Some(erase) = self.painting {
            input.erase = erase;
            // the way since the press (this frame) or since the last frame
            let mut on = !begins;
            for event in &events {
                match event {
                    egui::Event::PointerButton {
                        pos,
                        button: PointerButton::Primary,
                        pressed,
                        ..
                    } => {
                        if *pressed {
                            on = true;
                        }
                        if on {
                            input.path.push(at(*pos));
                        }
                    }
                    egui::Event::PointerMoved(pos) if on => input.path.push(at(*pos)),
                    _ => {}
                }
            }
            if begins && input.path.is_empty() {
                let press = ctx.input(|i| i.pointer.press_origin());
                input.path.extend(press.map(at));
            }
            if !down {
                input.released = true;
                self.painting = None;
            }
        } else {
            input.erase = alt;
        }
        let shown = response.hovered() || input.pressed || self.painting.is_some();
        input.hover = hover.filter(|p| shown && rect.contains(*p)).map(at);
    }

    /// Whether no drawn point is under `pos`.
    fn hit_nothing(
        &self,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        cloud: Option<&LoadedCloud>,
        pos: Option<egui::Pos2>,
        settings: &ViewSettings,
    ) -> bool {
        let (Some(pos), Some(cloud)) = (pos, cloud) else {
            return true;
        };
        let z_range = self.z_range;
        let pixel = to_physical([pos.x, pos.y], [rect.min.x, rect.min.y], ppp);
        pick_point_spots(
            &cloud.tree,
            &self.drawn,
            &self.camera,
            viewport,
            pixel,
            PICK_RADIUS * ppp,
            z_range,
            spots(settings, Some(cloud), ppp),
        )
        .is_none()
    }

    fn fly_speed(&self) -> f32 {
        let diagonal = self.bounds.map_or(0.0, |(min, max)| {
            (0..3)
                .map(|k| (max[k] - min[k]).powi(2))
                .sum::<f32>()
                .sqrt()
        });
        (diagonal * FLY_SPEED_PER_DIAGONAL).clamp(FLY_SPEED_MIN, FLY_SPEED_MAX)
    }
}
