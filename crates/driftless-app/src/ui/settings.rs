//! The settings window ⚙: how the 3D view draws, for the whole program – colour mode, point
//! size, point budget, camera („Bedienung“), layers – and what maps and surfaces are computed on
//! („Grafikkarte nutzen“).

use super::steps::parse_decimal;
use super::view3d::{color_modes, effective_color_mode};
use super::{text, AppState, ViewSettings};
use crate::camera::CameraMode;
use crate::render::{ColorMode, PointSize};
use driftless_gpu::GpuPref;
use eframe::egui::{self, Context, Ui};

/// Width of the window (points).
const WIDTH: f32 = 360.0;

/// Shows the window while [`AppState::settings_window`] is open; its × closes it.
pub fn show(ctx: &Context, state: &mut AppState) {
    let mut open = state.settings_window.open;
    egui::Window::new(text::SETTINGS)
        .id(egui::Id::new("einstellungen"))
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .default_width(WIDTH)
        .show(ctx, |ui| {
            view_section(ui, state);
            ui.separator();
            gpu_edit(ui, &mut state.gpu);
        });
    // the × closes it; the button in the rail may have closed it meanwhile
    state.settings_window.open &= open;
}

/// How the 3D view draws. The switch Punkte · Fläche · beides, the height cut and the numbers
/// of the last frame are in the floating bars of the view ([`super::tools`]).
fn view_section(ui: &mut Ui, state: &mut AppState) {
    let has_rgb = state.cloud.as_ref().is_none_or(|c| c.has_rgb);
    let map_point_m = state
        .cloud
        .as_ref()
        .map_or(crate::cloud::DEFAULT_POINT_M, |c| c.point_m);
    view_settings_edit(ui, &mut state.view, has_rgb, map_point_m);
    if state.cloud.as_ref().is_some_and(|c| !c.quality_known) {
        ui.weak(text::QUALITY_NONE_HINT);
    }
}

/// How the 3D view draws. `has_rgb`: whether the loaded cloud has colours (offers the
/// colour mode „Farbe“); `map_point_m`: the spot size of the loaded map.
fn view_settings_edit(ui: &mut Ui, view: &mut ViewSettings, has_rgb: bool, map_point_m: f32) {
    let shown = effective_color_mode(view.color_mode, has_rgb);
    ui.horizontal_wrapped(|ui| {
        ui.label(text::COLOR_MODE);
        for mode in color_modes(has_rgb) {
            let label = match mode {
                ColorMode::Rgb => text::MODE_RGB,
                ColorMode::Intensity => text::MODE_INTENSITY,
                ColorMode::Quality => text::MODE_QUALITY,
            };
            if ui.selectable_label(shown == mode, label).clicked() {
                view.color_mode = mode;
            }
        }
    });
    point_size_edit(ui, view, map_point_m);
    ui.horizontal(|ui| {
        let label = ui.label(text::BUDGET);
        let (lo, hi) = (*text::BUDGET_RANGE.start(), *text::BUDGET_RANGE.end());
        let mut millions = view.budget as f64 / 1e6;
        let slider = egui::Slider::new(&mut millions, lo as f64 / 1e6..=hi as f64 / 1e6)
            .step_by(0.5)
            .custom_formatter(|v, _| format!("{} Mio.", text::decimal(v, 1)))
            .custom_parser(|s| parse_decimal(s.trim_end_matches("Mio.").trim()));
        if ui.add(slider).labelled_by(label.id).changed() {
            view.budget = ((millions * 1e6).round() as usize).clamp(lo, hi);
        }
    });
    ui.horizontal(|ui| {
        ui.label(text::CAMERA_MODE);
        ui.selectable_value(&mut view.camera_mode, CameraMode::Orbit, text::ORBIT);
        ui.selectable_value(&mut view.camera_mode, CameraMode::Fly, text::FLY);
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(text::LAYERS);
        ui.checkbox(&mut view.show_trajectory, text::LAYER_TRAJECTORY);
        ui.checkbox(&mut view.show_odometry, text::LAYER_ODOMETRY);
        ui.checkbox(&mut view.show_loops, text::LAYER_LOOPS);
        ui.checkbox(&mut view.show_rejected, text::LAYER_REJECTED);
        ui.checkbox(&mut view.show_submaps, text::LAYER_SUBMAPS);
        ui.checkbox(&mut view.show_compare, text::LAYER_COMPARE);
    });
}

/// „Punkte: Pixel / Meter“, the size in pixels (in world size the smallest a spot gets) and,
/// in world size, the spot size: from the map (`map_point_m`) or set by hand.
fn point_size_edit(ui: &mut Ui, view: &mut ViewSettings, map_point_m: f32) {
    let world = view.point_size == PointSize::World;
    ui.horizontal(|ui| {
        ui.label(text::POINT_SIZE_MODE)
            .on_hover_text(text::POINT_SIZE_MODE_HINT);
        ui.selectable_value(
            &mut view.point_size,
            PointSize::Pixel,
            text::POINT_SIZE_PIXEL,
        );
        ui.selectable_value(
            &mut view.point_size,
            PointSize::World,
            text::POINT_SIZE_WORLD,
        );
    });
    ui.horizontal(|ui| {
        let label = ui.label(if world {
            text::POINT_SIZE_MIN
        } else {
            text::POINT_SIZE
        });
        ui.add(
            egui::Slider::new(&mut view.point_px, 1.0..=10.0)
                .step_by(0.5)
                .custom_formatter(|v, _| text::decimal(v, 1))
                .custom_parser(parse_decimal),
        )
        .labelled_by(label.id);
        // spots are always round
        if !world {
            ui.checkbox(&mut view.round, text::ROUND_POINTS);
        }
    });
    if world {
        ui.horizontal(|ui| {
            let label = ui.label(text::SPOT_SIZE);
            let mut from_map = view.point_m.is_none();
            let mut cm = view.point_m.unwrap_or(map_point_m) * 100.0;
            let [lo, hi] = text::POINT_M_RANGE.map(|m| m * 100.0);
            let changed = ui
                .add_enabled(
                    !from_map,
                    egui::DragValue::new(&mut cm)
                        .range(lo..=hi)
                        .speed(0.05)
                        .custom_formatter(|v, _| text::decimal(v, 1))
                        .custom_parser(parse_decimal),
                )
                .labelled_by(label.id)
                .changed();
            if ui
                .checkbox(&mut from_map, text::SPOT_FROM_MAP)
                .on_hover_text(text::SPOT_FROM_MAP_HINT)
                .changed()
                || changed
            {
                view.point_m = (!from_map).then_some(cm / 100.0);
            }
        });
    }
}

/// „Grafikkarte nutzen“: Auto · An · Aus.
fn gpu_edit(ui: &mut Ui, gpu: &mut GpuPref) {
    ui.horizontal(|ui| {
        ui.label(text::GPU).on_hover_text(text::GPU_HINT);
        ui.selectable_value(gpu, GpuPref::Auto, text::GPU_AUTO);
        ui.selectable_value(gpu, GpuPref::On, text::GPU_ON);
        ui.selectable_value(gpu, GpuPref::Off, text::GPU_OFF);
    });
}
