//! The 3D view in the middle of the window: the cloud drawn offscreen with
//! [`PointRenderer`] into a texture of the panel's size in physical pixels, which egui then
//! shows as an image. Every frame chooses the nodes to draw for the camera
//! ([`select_nodes`]) within the point budget; a frame whose camera, size and settings are
//! unchanged is not drawn again.
//!
//! Mouse and keys: orbiting with the left button, panning with the right or middle button
//! or Shift, zooming with the wheel; flying with W/A/S/D and Q/E, looking with the right
//! button, Shift faster; F or a double click into the empty = show everything; a click
//! without dragging picks a point, or, in the mode „Stellen wählen“, a pose of the trajectory.
//!
//! Over the cloud the view draws the run of [`ViewOverlay`] (trajectory, odometry, loops,
//! anchors, picked places); without a cloud the run alone. The height cut applies to the
//! points only: the trajectory is neither cut nor kept from being clicked by it, so the
//! places of a loop can be chosen with the ceiling cut away or not.

use super::{text, ViewSettings};
use crate::camera::{fly_motion, Camera, CameraMode, FlyKeys, ZOOM_FACTOR};
use crate::cloud::LoadedCloud;
use crate::lod::{select_nodes, Octree, PointVertex};
use crate::overlay::RunOverlay;
use crate::overlay::PICK_COLOR;
use crate::pick::{pick_point_in, project, to_physical};
use crate::render::{ColorMode, FrameParams, PointRenderer, RenderTarget, NO_CUT};
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
/// Background of the view, sRGB 0..1 as written.
const BACKGROUND: [f32; 4] = [0.08, 0.085, 0.09, 1.0];
const MARKER_COLOR: u32 = 0xffff_ffff;

/// [`BACKGROUND`] for egui (the same sRGB bytes).
fn background_color() -> Color32 {
    let [r, g, b, _] = BACKGROUND.map(|v| (v * 255.0).round() as u8);
    Color32::from_rgb(r, g, b)
}

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

/// The colour modes a cloud offers: without colours no [`ColorMode::Rgb`].
pub fn color_modes(has_rgb: bool) -> Vec<ColorMode> {
    let mut modes = vec![ColorMode::Intensity, ColorMode::Quality];
    if has_rgb {
        modes.insert(0, ColorMode::Rgb);
    }
    modes
}

/// The mode drawn: [`ColorMode::Rgb`] on a cloud without colours shows the intensity.
pub fn effective_color_mode(mode: ColorMode, has_rgb: bool) -> ColorMode {
    if mode == ColorMode::Rgb && !has_rgb {
        ColorMode::Intensity
    } else {
        mode
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
    /// The height cut of this frame (picking uses it too).
    z_range: [f32; 2],
    /// What the overlay on the GPU shows, and how often it was replaced.
    overlay_key: Option<OverlayKey>,
    overlay_count: u64,
    /// The run whose trajectory was framed while there was no cloud.
    framed_run: Option<std::path::PathBuf>,
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

    /// For tools and measurements that move the camera themselves.
    pub fn camera_mut(&mut self) -> &mut Camera {
        &mut self.camera
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
        let size = ui.available_size();
        let (rect, response) = ui.allocate_exact_size(size, Sense::click_and_drag());
        // A collapsed or squeezed panel: no target, no camera change (aspect 0).
        if !(rect.width() >= 1.0 && rect.height() >= 1.0) {
            return None;
        }
        if cloud.is_none() && overlay.is_none() {
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
        if cloud.is_none() {
            // only the run: frame its trajectory once
            let run = overlay.map(|o| o.run);
            if run.map(|r| &r.run) != self.framed_run.as_ref() {
                self.framed_run = run.map(|r| r.run.clone());
                self.bounds = run.and_then(RunOverlay::bounds);
                self.frame_all = true;
            }
        }
        self.last_upload_bytes = 0;
        if !renderer.upload_done() {
            // A slice per frame; the window stays usable meanwhile.
            self.last_upload_bytes = renderer.upload_step(queue, tree, UPLOAD_SLICE_BYTES);
            let percent = (renderer.upload_fraction() * 100.0).round();
            ui.painter().rect_filled(rect, 0.0, background_color());
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                text::uploading(percent),
                egui::TextStyle::Body.resolve(ui.style()),
                Color32::LIGHT_GRAY,
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
        self.camera.set_mode(settings.camera_mode);

        self.z_range = settings.cut.unwrap_or(NO_CUT);
        let event = self.input(ui, &response, rect, ppp, viewport, cloud, overlay, settings);
        if let Some(ViewEvent::PickedPoint(i)) = event {
            self.picked = Some(i);
        }
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
        let point_px = (settings.point_px * ppp).max(1.0);
        let key = FrameKey {
            view_proj: self.camera.view_proj(aspect).into(),
            size: px,
            point_px,
            color_mode: effective_color_mode(settings.color_mode, has_rgb),
            round: settings.round,
            budget: settings.budget,
            picked: self.picked,
            z_range: settings.cut.unwrap_or(NO_CUT),
            overlay: self.overlay_count,
        };
        if self.last.as_ref() != Some(&key) {
            let started = Instant::now();
            // Refine while the points of a node are farther apart on screen than drawn.
            let info = self.camera.view_info(viewport, point_px);
            self.drawn = match cloud {
                Some(c) => select_nodes(&c.tree, &info, settings.budget),
                None => Vec::new(),
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
                background: BACKGROUND,
                quality_known: cloud.is_some_and(|c| c.points.quality_known),
                z_range: key.z_range,
            };
            if let (Some(target), Some(renderer)) = (&self.target, &mut self.renderer) {
                // `render` writes the shared uniform buffers and submits its own work, so
                // this frame is complete before anything else is drawn into the view.
                renderer.render(device, queue, target, &frame, tree, &self.drawn);
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
            ui.painter().image(id, rect, uv, Color32::WHITE);
        }
        let stats = text::drawn_stats(
            self.drawn_points,
            cloud.map_or(0, |c| c.points.len()),
            self.frame_ms,
            self.gpu_timed,
        );
        let help = match settings.camera_mode {
            _ if overlay.is_some_and(|o| o.pick_places) => text::VIEW_HELP_PICK,
            CameraMode::Orbit => text::VIEW_HELP_ORBIT,
            CameraMode::Fly => text::VIEW_HELP_FLY,
        };
        let font = egui::TextStyle::Small.resolve(ui.style());
        let painter = ui.painter_at(rect);
        let corner = rect.left_top() + egui::vec2(8.0, 6.0);
        let shown = painter.text(
            corner,
            egui::Align2::LEFT_TOP,
            stats,
            font.clone(),
            Color32::LIGHT_GRAY,
        );
        painter.text(
            egui::pos2(corner.x, shown.bottom() + 2.0),
            egui::Align2::LEFT_TOP,
            help,
            font,
            Color32::GRAY,
        );
        if let Some(o) = overlay {
            let body = egui::TextStyle::Body.resolve(ui.style());
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
            painter.rect_filled(r, 3.0, Color32::from_black_alpha(200));
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
            label(pos + egui::vec2(14.0, 10.0), s, Color32::WHITE);
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
        let delta = response.drag_delta() * ppp;
        let by = |b| response.dragged_by(b);
        match mode {
            CameraMode::Orbit => {
                if by(PointerButton::Primary) && !shift {
                    self.camera.orbit(delta.x, delta.y);
                } else if by(PointerButton::Primary)
                    || by(PointerButton::Secondary)
                    || by(PointerButton::Middle)
                {
                    self.camera.pan(delta.x, delta.y, viewport);
                }
            }
            CameraMode::Fly => {
                if by(PointerButton::Primary) || by(PointerButton::Secondary) {
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
        if ctx.memory(|m| m.focused()).is_none() {
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
                        (None, Some(cloud)) => pick_point_in(
                            &cloud.tree,
                            &self.drawn,
                            &self.camera,
                            viewport,
                            pixel,
                            PICK_RADIUS * ppp,
                            z_range,
                        )
                        .map(|v| ViewEvent::PickedPoint(cloud.tree.source[v as usize])),
                        (None, None) => None,
                    };
                }
            }
        }
        if response.double_clicked()
            && event.is_none()
            && self.hit_nothing(rect, ppp, viewport, cloud, pos)
        {
            self.frame_all = true;
        }
        if self.frame_all {
            ctx.request_repaint();
        }
        event
    }

    /// Whether no drawn point is under `pos`.
    fn hit_nothing(
        &self,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        cloud: Option<&LoadedCloud>,
        pos: Option<egui::Pos2>,
    ) -> bool {
        let (Some(pos), Some(cloud)) = (pos, cloud) else {
            return true;
        };
        let z_range = self.z_range;
        let pixel = to_physical([pos.x, pos.y], [rect.min.x, rect.min.y], ppp);
        pick_point_in(
            &cloud.tree,
            &self.drawn,
            &self.camera,
            viewport,
            pixel,
            PICK_RADIUS * ppp,
            z_range,
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
