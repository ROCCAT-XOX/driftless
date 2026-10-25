//! The settings window ⚙: how the 3D view draws, for the whole program – colour mode, point
//! size, point budget, camera („Bedienung“), layers –, what it shows while a surface is
//! loaded, the height cut and the numbers of the last frame.

use super::steps::parse_decimal;
use super::view3d::{color_modes, effective_color_mode};
use super::{text, AppState, ViewSettings};
use crate::camera::CameraMode;
use crate::render::{ColorMode, PointSize, ShowMode};
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
        .show(ctx, |ui| view_section(ui, state));
    // the × closes it; the button in the rail may have closed it meanwhile
    state.settings_window.open &= open;
}

/// How the 3D view draws, the height cut and the numbers of the last frame.
fn view_section(ui: &mut Ui, state: &mut AppState) {
    if state.surface.is_some() {
        show_mode_edit(ui, &mut state.view.show, state.cloud.is_some());
    }
    let has_rgb = state.cloud.as_ref().is_none_or(|c| c.has_rgb);
    let map_point_m = state
        .cloud
        .as_ref()
        .map_or(crate::cloud::DEFAULT_POINT_M, |c| c.point_m);
    view_settings_edit(ui, &mut state.view, has_rgb, map_point_m);
    if state.cloud.as_ref().is_some_and(|c| !c.quality_known) {
        ui.weak(text::QUALITY_NONE_HINT);
    }
    // the cut over the cloud, else over the surface alone
    let cut_over = match (&state.cloud, &state.surface) {
        (Some(c), _) => Some((c.z_range, c.ceiling)),
        (None, Some(s)) => Some((s.z_range, None)),
        (None, None) => None,
    };
    if let Some((z_range, ceiling)) = cut_over {
        ui.separator();
        cut_edit(ui, &mut state.view.cut, z_range, ceiling);
    }
    ui.separator();
    if let Some(stats) = &state.view_stats {
        ui.label(match stats.triangles {
            Some(triangles) => text::surface_stats(triangles, stats.frame_ms, stats.gpu),
            None => text::drawn_stats(stats.drawn, stats.total, stats.frame_ms, stats.gpu),
        });
    }
    ui.weak(text::FRAME_ALL_HINT);
}

/// „Anzeige: Oberfläche / Punktwolke / beides“; without a cloud only the surface can be
/// shown, so the switch is off.
fn show_mode_edit(ui: &mut Ui, show: &mut ShowMode, has_cloud: bool) {
    ui.add_enabled_ui(has_cloud, |ui| {
        ui.horizontal(|ui| {
            ui.label(text::SHOW_MODE)
                .on_hover_text(text::SHOW_MODE_KEY_HINT);
            ui.selectable_value(show, ShowMode::Surface, text::SHOW_SURFACE);
            ui.selectable_value(show, ShowMode::Cloud, text::SHOW_CLOUD);
            ui.selectable_value(show, ShowMode::Both, text::SHOW_BOTH);
        });
    });
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

/// The height cut: z min / z max over `z_range` (of the cloud, or of the surface alone),
/// „Decke ausblenden“ (z max just below the estimated `ceiling`) and „Zurücksetzen“.
fn cut_edit(ui: &mut Ui, cut: &mut Option<[f32; 2]>, z_range: [f32; 2], ceiling: Option<f32>) {
    let [lo, hi] = z_range;
    ui.strong(text::CUT).on_hover_text(text::CUT_HINT);
    let mut range = cut.unwrap_or([lo, hi]);
    let mut changed = false;
    for (label, k) in [(text::CUT_Z_MIN, 0), (text::CUT_Z_MAX, 1)] {
        ui.horizontal(|ui| {
            ui.label(label);
            let slider = egui::Slider::new(&mut range[k], lo..=hi.max(lo))
                .step_by(0.05)
                .custom_formatter(|v, _| text::height(v as f32))
                .custom_parser(|s| parse_decimal(s.trim_end_matches('m').trim()));
            changed |= ui.add(slider).changed();
        });
    }
    if changed {
        // the other end follows, so that the range never turns over
        if range[0] > range[1] {
            range = [range[0].min(range[1]), range[0].max(range[1])];
        }
        *cut = Some(range);
    }
    ui.horizontal(|ui| {
        let button = ui.add_enabled(ceiling.is_some(), egui::Button::new(text::HIDE_CEILING));
        if button.clicked() {
            if let Some(ceiling) = ceiling {
                *cut = Some(below_ceiling(range, ceiling, lo));
            }
        }
        button.on_disabled_hover_text(text::NO_CEILING);
        if ui
            .add_enabled(cut.is_some(), egui::Button::new(text::CUT_RESET))
            .clicked()
        {
            *cut = None;
        }
    });
}

/// The cut of „Decke ausblenden“: z max at the ceiling (not below the cloud's lowest point
/// `lo`), z min kept but never above z max, so that the range never turns over.
fn below_ceiling(range: [f32; 2], ceiling: f32, lo: f32) -> [f32; 2] {
    let top = ceiling.max(lo);
    [range[0].min(top), top]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hiding_the_ceiling_never_turns_the_range_over() {
        assert_eq!(below_ceiling([0.5, 4.0], 2.5, 0.0), [0.5, 2.5]);
        // z min above the ceiling follows it down
        assert_eq!(below_ceiling([3.0, 4.0], 2.5, 0.0), [2.5, 2.5]);
        // a ceiling below the cloud is taken as its lowest point
        assert_eq!(below_ceiling([0.5, 4.0], -1.0, 0.0), [0.0, 0.0]);
    }
}
