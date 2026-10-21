//! The 3D view in the middle of the window: the cloud drawn offscreen with
//! [`PointRenderer`] into a texture of the panel's size in physical pixels, which egui then
//! shows as an image. Every frame chooses the nodes to draw for the camera
//! ([`select_nodes`]) within the point budget; a frame whose camera, size and settings are
//! unchanged is not drawn again.
//!
//! Mouse and keys: orbiting with the left button, panning with the right or middle button
//! or Shift, zooming with the wheel; flying with W/A/S/D and Q/E, looking with the right
//! button, Shift faster; F or a double click into the empty = show everything; a click
//! without dragging picks a point.

use super::{text, ViewSettings};
use crate::camera::{fly_motion, Camera, CameraMode, FlyKeys, ZOOM_FACTOR};
use crate::cloud::{LoadedCloud, PointSet};
use crate::lod::{select_nodes, PointVertex};
use crate::pick::{pick_point, to_physical};
use crate::render::{ColorMode, FrameParams, PointRenderer, RenderTarget};
use eframe::egui::{self, Color32, Key, PointerButton, Rect, Sense, Ui};
use egui_wgpu::wgpu;
use std::time::{Duration, Instant};

/// Format of the offscreen image. Not an sRGB format: the shaders write the LAS sRGB bytes
/// as they are, and egui expects textures that are not sRGB-aware (it would convert twice).
pub const VIEW_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
/// A press and release closer than this (points) and …
pub const CLICK_MAX_MOVE: f32 = 4.0;
/// … shorter than this (s) is a click, not a drag.
pub const CLICK_MAX_SECONDS: f64 = 0.3;
/// Points within this distance (points) of the mouse can be picked.
const PICK_RADIUS: f32 = 6.0;
/// Diameter of the marker on the picked point (points).
const MARKER_PX: f32 = 10.0;
/// One step of the mouse wheel (points of scrolling).
const WHEEL_STEP: f32 = 50.0;
/// Flying speed: this fraction of the cloud's diagonal per second, within the bounds (m/s).
const FLY_SPEED_PER_DIAGONAL: f32 = 0.05;
const FLY_SPEED_MIN: f32 = 1.0;
const FLY_SPEED_MAX: f32 = 50.0;
/// Longest wait for the GPU when timing a frame.
const GPU_WAIT: Duration = Duration::from_millis(250);
/// Background of the view, sRGB 0..1 as written.
const BACKGROUND: [f32; 4] = [0.08, 0.085, 0.09, 1.0];
const MARKER_COLOR: u32 = 0xffff_ffff;

/// What the view reports from a click.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewEvent {
    /// The index of the clicked point in the cloud's [`crate::cloud::PointSet`].
    PickedPoint(u32),
    /// The index of the clicked pose of the trajectory (used from the trajectory overlay on).
    PickedPose(usize),
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

/// Points looked at by [`view_bounds`] at most.
const BOUNDS_SAMPLE: usize = 200_000;
/// Share of the points cut off on each side of each axis by [`view_bounds`].
const BOUNDS_CUT: f64 = 0.005;

/// The box „show everything“ frames: per axis from the 0,5 % to the 99,5 % point of an
/// evenly spread sample, so that a few far outliers do not shrink the cloud to a dot.
/// `None` without points.
pub fn view_bounds(points: &PointSet) -> Option<([f32; 3], [f32; 3])> {
    let n = points.len();
    if n == 0 {
        return None;
    }
    let step = n.div_ceil(BOUNDS_SAMPLE);
    let mut min = [0.0; 3];
    let mut max = [0.0; 3];
    for k in 0..3 {
        let mut v: Vec<f32> = points.xyz.iter().step_by(step).map(|p| p[k]).collect();
        v.sort_unstable_by(f32::total_cmp);
        let cut = ((v.len() as f64 * BOUNDS_CUT) as usize).min((v.len() - 1) / 2);
        min[k] = v[cut];
        max[k] = v[v.len() - 1 - cut];
    }
    Some((min, max))
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
    /// Quality of the point's submap × 255.
    pub quality: u8,
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
        quality: p.quality[i],
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
    /// The cloud on the GPU is the current one.
    uploaded: bool,
    /// Bounds of the cloud, from the upload.
    bounds: Option<([f32; 3], [f32; 3])>,
    /// Show everything at the next frame with a usable size.
    frame_all: bool,
    last: Option<FrameKey>,
    press: Option<Press>,
    /// The picked point (index in the point set), marked in the view.
    picked: Option<u32>,
}

impl View3d {
    /// The program replaced the cloud: upload it and show all of it at the next frame.
    pub fn cloud_replaced(&mut self) {
        self.uploaded = false;
        self.picked = None;
        self.drawn.clear();
        self.drawn_points = 0;
        self.last = None;
    }

    /// Points drawn in the last frame.
    pub fn drawn_points(&self) -> usize {
        self.drawn_points
    }

    /// Time of the last frame drawn (ms): choosing the nodes plus the GPU until done.
    pub fn frame_ms(&self) -> f32 {
        self.frame_ms
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    /// For tools and measurements that move the camera themselves.
    pub fn camera_mut(&mut self) -> &mut Camera {
        &mut self.camera
    }

    /// Fills `ui` with the view of `cloud` (nothing to draw without one or for a panel of
    /// size 0). Returns what a click picked.
    pub fn show(
        &mut self,
        ui: &mut Ui,
        render_state: &egui_wgpu::RenderState,
        cloud: Option<&LoadedCloud>,
        settings: &ViewSettings,
    ) -> Option<ViewEvent> {
        let size = ui.available_size();
        let (rect, response) = ui.allocate_exact_size(size, Sense::click_and_drag());
        // A collapsed or squeezed panel: no target, no camera change (aspect 0).
        if !(rect.width() >= 1.0 && rect.height() >= 1.0) {
            return None;
        }
        let cloud = cloud?;
        let ppp = ui.ctx().pixels_per_point();
        let px = target_size(rect.size(), ppp);
        let viewport = [px[0] as f32, px[1] as f32];
        let aspect = viewport[0] / viewport[1];
        let (device, queue) = (&render_state.device, &render_state.queue);

        let renderer = self
            .renderer
            .get_or_insert_with(|| PointRenderer::new(device, VIEW_FORMAT));
        if !self.uploaded {
            let renderer = &mut *renderer;
            renderer.upload_cloud(device, queue, &cloud.tree);
            renderer.set_overlay(device, &[], &[]);
            self.bounds = view_bounds(&cloud.points);
            self.uploaded = true;
            self.frame_all = true;
            self.last = None;
        }
        if self.frame_all {
            if let Some((min, max)) = self.bounds {
                self.camera.frame_bounds(min, max, aspect);
            }
            self.frame_all = false;
        }
        self.camera.set_mode(settings.camera_mode);

        let event = self.input(ui, &response, rect, ppp, viewport, cloud);
        if let Some(ViewEvent::PickedPoint(i)) = event {
            self.set_picked(device, cloud, Some(i));
        }

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

        let has_rgb = cloud.points.rgb.is_some();
        let point_px = (settings.point_px * ppp).max(1.0);
        let key = FrameKey {
            view_proj: self.camera.view_proj(aspect).into(),
            size: px,
            point_px,
            color_mode: effective_color_mode(settings.color_mode, has_rgb),
            round: settings.round,
            budget: settings.budget,
            picked: self.picked,
        };
        if self.last.as_ref() != Some(&key) {
            let started = Instant::now();
            // Refine while the points of a node are farther apart on screen than drawn.
            let info = self.camera.view_info(viewport, point_px);
            self.drawn = select_nodes(&cloud.tree, &info, settings.budget);
            self.drawn_points = self
                .drawn
                .iter()
                .map(|&i| cloud.tree.nodes[i as usize].count as usize)
                .sum();
            let frame = FrameParams {
                view_proj: key.view_proj,
                viewport_px: viewport,
                point_px,
                marker_px: MARKER_PX * ppp,
                color_mode: key.color_mode,
                round: key.round,
                background: BACKGROUND,
            };
            if let (Some(target), Some(renderer)) = (&self.target, &self.renderer) {
                // `render` writes the shared uniform buffers and submits its own work, so
                // this frame is complete before anything else is drawn into the view.
                let submitted =
                    renderer.render(device, queue, target, &frame, &cloud.tree, &self.drawn);
                // Waiting here times the GPU work as well; egui could not show the image
                // before it is done anyway.
                let _ = device.poll(wgpu::PollType::Wait {
                    submission_index: Some(submitted),
                    timeout: Some(GPU_WAIT),
                });
            }
            self.frame_ms = started.elapsed().as_secs_f32() * 1000.0;
            self.last = Some(key);
        }

        if let Some(id) = self.texture {
            let uv = Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
            ui.painter().image(id, rect, uv, Color32::WHITE);
        }
        let stats = text::drawn_stats(self.drawn_points, cloud.points.len(), self.frame_ms);
        let help = match settings.camera_mode {
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
        event
    }

    fn set_picked(&mut self, device: &wgpu::Device, cloud: &LoadedCloud, picked: Option<u32>) {
        self.picked = picked;
        let markers: Vec<PointVertex> = picked
            .and_then(|i| cloud.points.xyz.get(i as usize))
            .map(|&pos| PointVertex {
                pos,
                rgbi: MARKER_COLOR,
                extra: 0,
            })
            .into_iter()
            .collect();
        if let Some(renderer) = &mut self.renderer {
            renderer.set_overlay(device, &[], &markers);
        }
    }

    /// Mouse and keys of one frame; a click picks.
    fn input(
        &mut self,
        ui: &Ui,
        response: &egui::Response,
        rect: Rect,
        ppp: f32,
        viewport: [f32; 2],
        cloud: &LoadedCloud,
    ) -> Option<ViewEvent> {
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
                    let hit = pick_point(
                        &cloud.tree,
                        &self.drawn,
                        &self.camera,
                        viewport,
                        pixel,
                        PICK_RADIUS * ppp,
                    );
                    event = hit.map(|v| ViewEvent::PickedPoint(cloud.tree.source[v as usize]));
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
        cloud: &LoadedCloud,
        pos: Option<egui::Pos2>,
    ) -> bool {
        let Some(pos) = pos else {
            return true;
        };
        let pixel = to_physical([pos.x, pos.y], [rect.min.x, rect.min.y], ppp);
        pick_point(
            &cloud.tree,
            &self.drawn,
            &self.camera,
            viewport,
            pixel,
            PICK_RADIUS * ppp,
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
