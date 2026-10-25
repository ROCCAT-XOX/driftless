//! The tools of the 3D view and its floating bars, as in the design:
//!
//! - top left the info: what is shown („Punktwolke · 6,0 Mio. Punkte“ or „Oberfläche ·
//!   712.218 Dreiecke“), the numbers of the last frame, what a click does now and the clicked
//!   point;
//! - top right, while a surface is loaded, the switch Punkte · Fläche · beides (key `O`);
//! - bottom centre the tools Drehen · Verschieben · Punkt wählen · Messen · Schnitt and the
//!   button „Alles“ (key `F`); exactly one tool is active ([`Tool`]);
//! - above them, with the tool „Schnitt“, the height cut: z min, z max, „Decke ausblenden“.
//!
//! Measuring: the first click on a point sets its first end, the second the other and adds the
//! [`Measurement`]; Esc or another tool drops a first end. The view draws the lines with their
//! distance ([`paint_measurements`]), the field „Planung“ lists them.

use super::steps::{parse_decimal, point_lines};
use super::theme::{self, icon, DARK};
use super::view3d::{effective_show, target_size, PointInfo};
use super::{text, Action, AppState};
use crate::camera::{Camera, CameraMode};
use crate::pick::{project, relative_to};
use crate::render::ShowMode;
use eframe::egui::{
    self, Align2, Area, Context, CornerRadius, FontId, Id, Order, Rect, RichText, Sense, Stroke,
    Ui, WidgetInfo, WidgetType,
};

/// The tool of the view: what the left mouse button does.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub enum Tool {
    /// The left button turns the view (in fly mode: looks around).
    #[default]
    Orbit,
    /// The left button moves the view.
    Pan,
    /// A click shows the values of a point.
    Pick,
    /// Two clicks measure the distance of two points.
    Measure,
    /// The height cut; its bar shows above the tools.
    Cut,
}

impl Tool {
    /// Whether a click picks a point of the cloud (else it does nothing; dragging turns the
    /// view as with [`Tool::Orbit`], except with [`Tool::Pan`]).
    pub fn picks_points(self) -> bool {
        matches!(self, Tool::Pick | Tool::Measure)
    }
}

/// The tools of the bar: tool, symbol, name, tooltip.
const TOOLS: [(Tool, &str, &str, &str); 5] = [
    (
        Tool::Orbit,
        icon::ORBIT,
        text::TOOL_ORBIT,
        text::TOOL_ORBIT_HINT,
    ),
    (Tool::Pan, icon::PAN, text::TOOL_PAN, text::TOOL_PAN_HINT),
    (
        Tool::Pick,
        icon::PICK,
        text::TOOL_PICK,
        text::TOOL_PICK_HINT,
    ),
    (
        Tool::Measure,
        icon::MEASURE,
        text::TOOL_MEASURE,
        text::TOOL_MEASURE_HINT,
    ),
    (Tool::Cut, icon::CUT, text::TOOL_CUT, text::CUT_HINT),
];

/// The distance between two points of the cloud (absolute coordinates).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measurement {
    pub a: [f64; 3],
    pub b: [f64; 3],
}

impl Measurement {
    /// Metres from `a` to `b`.
    pub fn distance(&self) -> f64 {
        (0..3)
            .map(|k| (self.b[k] - self.a[k]).powi(2))
            .sum::<f64>()
            .sqrt()
    }
}

/// Where the panels and the floating bars were in the last frame (for tests and for
/// placing things next to them).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScreenLayout {
    pub rail: Rect,
    pub step: Rect,
    pub status: Rect,
    pub view: Rect,
    pub info: Option<Rect>,
    pub show: Option<Rect>,
    pub tools: Option<Rect>,
    pub cut: Option<Rect>,
}

impl Default for ScreenLayout {
    fn default() -> Self {
        ScreenLayout {
            rail: Rect::NOTHING,
            step: Rect::NOTHING,
            status: Rect::NOTHING,
            view: Rect::NOTHING,
            info: None,
            show: None,
            tools: None,
            cut: None,
        }
    }
}

/// Distance of the floating bars from the edges of the view (points).
const MARGIN: f32 = 12.0;
/// Distance of the tools from the bottom edge (points).
const BOTTOM_MARGIN: f32 = 14.0;
/// Gap between the cut and the tools below it (points).
const GAP: f32 = 8.0;
/// The info takes at most this share of the view's width, so it never reaches the switch.
const INFO_SHARE: f32 = 0.45;
/// Height of a button of the bars (points).
const BUTTON_HEIGHT: f32 = 28.0;
const BUTTON_PAD: f32 = 9.0;
/// A button with a symbol only (×) is smaller.
const SMALL_BUTTON: f32 = 20.0;
/// Least height of a row of text in the bars (points).
const ROW_HEIGHT: f32 = 16.0;
const ICON_SIZE: f32 = 15.0;
/// Corner radius of a button of the bars.
const BUTTON_RADIUS: u8 = 7;
/// Text size of the numbers of the last frame.
const FRAME_LINE_SIZE: f32 = 11.0;
/// Width of a cut slider (points).
const SLIDER_WIDTH: f32 = 140.0;
/// Radius of the ends of a measurement (points), and the width of its line.
const END_RADIUS: f32 = 4.0;
const LINE_WIDTH: f32 = 2.0;

/// Makes `tool` the active one; a measurement being made is dropped, and „Punkt wählen“ or
/// „Messen“ ends picking the places of a loop.
pub fn select(state: &mut AppState, tool: Tool) {
    if state.tool != tool {
        state.tool = tool;
        state.measure_a = None;
    }
    // a click picks points now, not the places of a loop
    if tool.picks_points() {
        state.correct.picking = false;
    }
}

/// A click on the point `info` with the active tool: „Punkt wählen“ shows it, „Messen“ sets
/// the first end or, with one set, adds the measurement (not to the first end itself).
pub fn point_clicked(state: &mut AppState, info: PointInfo) {
    match state.tool {
        Tool::Pick => state.picked = Some(info),
        Tool::Measure => match state.measure_a {
            // the same point again (a double click) measures nothing: the first end stays
            Some(a) if a == info.position => {}
            Some(a) => {
                state.measure_a = None;
                state.measurements.push(Measurement {
                    a,
                    b: info.position,
                });
            }
            None => state.measure_a = Some(info.position),
        },
        Tool::Orbit | Tool::Pan | Tool::Cut => {}
    }
}

/// Esc (while no text field has the keyboard) drops the first end of a measurement.
pub fn keys(ctx: &Context, state: &mut AppState) {
    let pressed = ctx.input(|i| i.key_pressed(egui::Key::Escape));
    if pressed && keys_free(ctx) && state.measure_a.is_some() {
        state.measure_a = None;
    }
}

/// Where [`remember_text_focus`] keeps its flag.
fn text_focus_id() -> Id {
    Id::new("driftless_text_focus")
}

/// Remembers, at the end of a pass, whether a text field has the keyboard. egui takes the
/// focus from a text field on Esc at the start of the next pass, before the keys are read:
/// without this flag, Esc in a text field would also cancel a measurement.
pub fn remember_text_focus(ctx: &Context) {
    let focused = ctx.text_edit_focused();
    ctx.data_mut(|d| d.insert_temp(text_focus_id(), focused));
}

/// Whether the keys of the view (Esc, `O`, `F`, flying) are free: no text field has the
/// keyboard, now or at the end of the last pass. A button or slider with the keyboard (after
/// Tab, say) does not hold them.
pub fn keys_free(ctx: &Context) -> bool {
    let before = ctx.data(|d| d.get_temp::<bool>(text_focus_id()));
    !before.unwrap_or(false) && !ctx.text_edit_focused()
}

/// The floating bars over the view `view`, while a cloud or a surface is loaded. Their
/// rectangles go into `layout`. Returns „Alles“ as [`Action::FrameAll`].
pub fn floats(
    ctx: &Context,
    view: Rect,
    state: &mut AppState,
    layout: &mut ScreenLayout,
) -> Option<Action> {
    layout.view = view;
    layout.info = None;
    layout.show = None;
    layout.tools = None;
    layout.cut = None;
    if state.cloud.is_none() && state.surface.is_none() {
        return None;
    }
    layout.info = Some(info(ctx, view, state));
    if state.surface.is_some() {
        layout.show = Some(show_switch(ctx, view, state));
    }
    let (tools, action) = toolbar(ctx, view, state);
    layout.tools = Some(tools);
    if state.tool == Tool::Cut {
        layout.cut = cut_bar(ctx, view, tools.top() - GAP, state);
    }
    action
}

/// A floating bar: an area at `pos` of the view (`pivot` its corner there), in
/// [`theme::float_frame`]. Returns its rectangle.
fn float(
    ctx: &Context,
    id: &str,
    view: Rect,
    pos: egui::Pos2,
    pivot: Align2,
    body: impl FnOnce(&mut Ui),
) -> Rect {
    Area::new(Id::new(id))
        .order(Order::Middle)
        .fixed_pos(pos)
        .pivot(pivot)
        .constrain_to(view)
        .show(ctx, |ui| {
            theme::float_frame().show(ui, |ui| {
                ui.spacing_mut().item_spacing = egui::vec2(4.0, 4.0);
                body(ui)
            });
        })
        .response
        .rect
}

/// The info at the top left: what is shown, the numbers of the last frame, what a click does
/// now and the clicked point (with × to close it).
fn info(ctx: &Context, view: Rect, state: &mut AppState) -> Rect {
    let mut close = false;
    let s = &*state;
    let pos = view.left_top() + egui::vec2(MARGIN, MARGIN);
    let rect = float(ctx, "info", view, pos, Align2::LEFT_TOP, |ui| {
        ui.set_max_width(view.width() * INFO_SHARE);
        // rows of text as high as the text, not as a button
        ui.spacing_mut().interact_size.y = ROW_HEIGHT;
        let shown = effective_show(s.view.show, s.surface.is_some(), s.cloud.is_some());
        let head = match (&s.cloud, &s.surface) {
            (Some(c), _) if shown != ShowMode::Surface => text::info_cloud(c.points),
            (_, Some(surface)) => text::info_surface(surface.triangles),
            (Some(c), None) => text::info_cloud(c.points),
            (None, None) => return,
        };
        let help = match s.view.camera_mode {
            CameraMode::Orbit => text::VIEW_HELP_ORBIT,
            CameraMode::Fly => text::VIEW_HELP_FLY,
        };
        ui.label(RichText::new(head).color(DARK.text).strong())
            .on_hover_text(help);
        if let Some(stats) = &s.view_stats {
            let drawn = stats.triangles.is_none().then_some(stats.drawn);
            let line = text::frame_line(drawn, stats.frame_ms, stats.gpu);
            ui.label(
                RichText::new(line)
                    .size(FRAME_LINE_SIZE)
                    .color(DARK.text_dim),
            );
        }
        // the cut hides points; while its bar is not shown, say so here
        if let (Some(cut), false) = (s.view.cut, s.tool == Tool::Cut) {
            ui.label(RichText::new(text::cut_active(cut)).color(DARK.text_dim))
                .on_hover_text(text::CUT_ACTIVE_HINT);
        }
        if let Some(hint) = click_hint(s) {
            ui.add(egui::Label::new(RichText::new(hint).color(DARK.measure)).wrap());
        }
        if let Some(point) = &s.picked {
            // no separator: it would stretch the info to its widest
            ui.add_space(6.0);
            close = point_block(ui, point);
        }
    });
    if close {
        state.picked = None;
    }
    rect
}

/// What a click into the view does now, if it is more than turning it.
fn click_hint(s: &AppState) -> Option<&'static str> {
    if s.correct.picking {
        return Some(text::VIEW_HELP_PICK);
    }
    if s.tool.picks_points() && s.cloud.is_none() {
        return Some(text::TOOL_NEEDS_CLOUD);
    }
    match s.tool {
        Tool::Measure if s.measure_a.is_some() => Some(text::MEASURE_SECOND),
        Tool::Measure => Some(text::MEASURE_FIRST),
        Tool::Pick if s.picked.is_none() => Some(text::PICK_POINT),
        _ => None,
    }
}

/// „Punkt“ with ×, and its values. True when × was clicked.
fn point_block(ui: &mut Ui, point: &PointInfo) -> bool {
    let close = ui
        .horizontal(|ui| {
            ui.strong(text::POINT);
            bar_button(ui, Some(icon::CLOSE), None, text::CLOSE_POINT, None)
                .on_hover_text(text::CLOSE_POINT)
                .clicked()
        })
        .inner;
    egui::Grid::new("punkt_info")
        .num_columns(2)
        .spacing([12.0, 2.0])
        .show(ui, |ui| {
            for (label, value) in point_lines(point) {
                ui.label(RichText::new(label).color(DARK.text_dim));
                ui.label(value);
                ui.end_row();
            }
        });
    close
}

/// The switch at the top right: „Ansicht“ Punkte · Fläche · beides. Only a surface without a
/// cloud locks Punkte and beides, with the reason.
fn show_switch(ctx: &Context, view: Rect, state: &mut AppState) -> Rect {
    let has_cloud = state.cloud.is_some();
    let shown = effective_show(state.view.show, true, has_cloud);
    let mut chosen = None;
    let pos = view.right_top() + egui::vec2(-MARGIN, MARGIN);
    let rect = float(ctx, "ansicht", view, pos, Align2::RIGHT_TOP, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new(text::SHOW_MODE).color(DARK.text_dim))
                .on_hover_text(text::SHOW_MODE_KEY_HINT);
            for (mode, label) in [
                (ShowMode::Cloud, text::SHOW_CLOUD),
                (ShowMode::Surface, text::SHOW_SURFACE),
                (ShowMode::Both, text::SHOW_BOTH),
            ] {
                let enabled = has_cloud || mode == ShowMode::Surface;
                let r = ui
                    .add_enabled_ui(enabled, |ui| {
                        bar_button(ui, None, Some(label), label, Some(shown == mode))
                    })
                    .inner
                    .on_disabled_hover_text(text::SHOW_NEEDS_CLOUD);
                if r.clicked() {
                    chosen = Some(mode);
                }
            }
        });
    });
    if let Some(mode) = chosen {
        state.view.show = mode;
    }
    rect
}

/// The tools at the bottom centre and „Alles“. Returns the bar's rectangle and „Alles“.
fn toolbar(ctx: &Context, view: Rect, state: &mut AppState) -> (Rect, Option<Action>) {
    let mut chosen = None;
    let mut action = None;
    let active = state.tool;
    let pos = view.center_bottom() - egui::vec2(0.0, BOTTOM_MARGIN);
    let rect = float(ctx, "werkzeuge", view, pos, Align2::CENTER_BOTTOM, |ui| {
        ui.horizontal(|ui| {
            for (tool, symbol, name, hint) in TOOLS {
                let r = bar_button(ui, Some(symbol), Some(name), name, Some(tool == active));
                if r.on_hover_text(hint).clicked() {
                    chosen = Some(tool);
                }
            }
            ui.separator();
            let r = bar_button(
                ui,
                Some(icon::FIT),
                Some(text::FIT_ALL),
                text::FIT_ALL,
                None,
            );
            if r.on_hover_text(text::FRAME_ALL_HINT).clicked() {
                action = Some(Action::FrameAll);
            }
        });
    });
    if let Some(tool) = chosen {
        select(state, tool);
    }
    (rect, action)
}

/// A button of the floating bars: symbol and label, filled with the accent colour while `on`.
/// `name` is what screen readers (and tests) see; `on` = `None` is a plain button, else a
/// toggle.
fn bar_button(
    ui: &mut Ui,
    symbol: Option<&str>,
    label: Option<&str>,
    name: &str,
    on: Option<bool>,
) -> egui::Response {
    let selected = on.unwrap_or(false);
    let fg = if !ui.is_enabled() {
        DARK.text_dim.gamma_multiply(0.6)
    } else if selected {
        DARK.on_accent
    } else {
        DARK.text
    };
    let symbol = symbol
        .map(|s| ui.fonts_mut(|f| f.layout_no_wrap(s.into(), FontId::proportional(ICON_SIZE), fg)));
    let font = egui::TextStyle::Button.resolve(ui.style());
    let label = label.map(|l| ui.fonts_mut(|f| f.layout_no_wrap(l.into(), font.clone(), fg)));
    let gap = if symbol.is_some() && label.is_some() {
        5.0
    } else {
        0.0
    };
    let content = symbol.as_ref().map_or(0.0, |g| g.size().x)
        + gap
        + label.as_ref().map_or(0.0, |g| g.size().x);
    let size = if label.is_none() {
        egui::vec2(SMALL_BUTTON.max(content + 6.0), SMALL_BUTTON)
    } else {
        egui::vec2(content + 2.0 * BUTTON_PAD, BUTTON_HEIGHT)
    };
    let pad = (size.x - content) / 2.0;
    let (rect, response) = ui.allocate_exact_size(size, Sense::click());
    let enabled = ui.is_enabled();
    response.widget_info(|| match on {
        Some(on) => WidgetInfo::selected(WidgetType::Button, enabled, on, name),
        None => WidgetInfo::labeled(WidgetType::Button, enabled, name),
    });
    if ui.is_rect_visible(rect) {
        let fill = if selected {
            DARK.accent
        } else if response.hovered() && enabled {
            DARK.surface_hi
        } else {
            egui::Color32::TRANSPARENT
        };
        let painter = ui.painter();
        painter.rect_filled(rect, CornerRadius::same(BUTTON_RADIUS), fill);
        let mut x = rect.left() + pad;
        for galley in [symbol, label].into_iter().flatten() {
            let at = egui::pos2(x, rect.center().y - galley.size().y / 2.0);
            x += galley.size().x + gap;
            painter.galley(at, galley, fg);
        }
    }
    response
}

/// The height cut over the tools (its bottom at `bottom`): z min, z max, „Decke ausblenden“
/// and „Zurücksetzen“, over the cloud, else over the surface alone. `None` without either.
fn cut_bar(ctx: &Context, view: Rect, bottom: f32, state: &mut AppState) -> Option<Rect> {
    let (z_range, ceiling) = match (&state.cloud, &state.surface) {
        (Some(c), _) => (c.z_range, c.ceiling),
        (None, Some(s)) => (s.z_range, None),
        (None, None) => return None,
    };
    let cut = &mut state.view.cut;
    let pos = egui::pos2(view.center().x, bottom);
    Some(float(
        ctx,
        "schnitt",
        view,
        pos,
        Align2::CENTER_BOTTOM,
        |ui| {
            cut_edit(ui, cut, z_range, ceiling);
        },
    ))
}

/// The height cut: z min / z max over `z_range` (of the cloud, or of the surface alone),
/// „Decke ausblenden“ (z max just below the estimated `ceiling`) and „Zurücksetzen“.
fn cut_edit(ui: &mut Ui, cut: &mut Option<[f32; 2]>, z_range: [f32; 2], ceiling: Option<f32>) {
    let [lo, hi] = z_range;
    let mut range = cut.unwrap_or([lo, hi]);
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.spacing_mut().slider_width = SLIDER_WIDTH;
        for (label, k) in [(text::CUT_MIN, 0), (text::CUT_MAX, 1)] {
            let label = ui.label(label).on_hover_text(text::CUT_HINT);
            let slider = egui::Slider::new(&mut range[k], lo..=hi.max(lo))
                .step_by(0.05)
                .custom_formatter(|v, _| text::height(v as f32))
                .custom_parser(|s| parse_decimal(s.trim_end_matches('m').trim()));
            changed |= ui.add(slider).labelled_by(label.id).changed();
            ui.add_space(6.0);
        }
    });
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
        let reset = ui.add_enabled(cut.is_some(), egui::Button::new(text::CUT_RESET));
        if reset.clicked() {
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

/// Draws the measurements over the view in `rect`, seen by `camera` in the frame at `origin`:
/// a line between the ends, a dot at each, and the distance at the middle as a label (a
/// widget, so that it can be read out). A first end being measured gets its dot and a dashed
/// line to the pointer.
pub fn paint_measurements(
    ui: &mut Ui,
    rect: Rect,
    camera: &Camera,
    origin: [f64; 3],
    state: &AppState,
) {
    if state.measurements.is_empty() && state.measure_a.is_none() {
        return;
    }
    let ppp = ui.ctx().pixels_per_point();
    let px = target_size(rect.size(), ppp);
    let viewport = [px[0] as f32, px[1] as f32];
    let vp = camera.view_proj(viewport[0] / viewport[1]);
    // on the screen and in front of the near plane
    let screen = |p: [f64; 3]| {
        let q = project(&vp, relative_to(origin, p), viewport)?;
        (q[2] <= 1.0).then(|| rect.min + egui::vec2(q[0], q[1]) / ppp)
    };
    let painter = ui.painter_at(rect);
    let stroke = Stroke::new(LINE_WIDTH, DARK.measure);
    let font = egui::TextStyle::Body.resolve(ui.style());
    for m in &state.measurements {
        let (Some(a), Some(b)) = (screen(m.a), screen(m.b)) else {
            continue;
        };
        painter.line_segment([a, b], stroke);
        painter.circle_filled(a, END_RADIUS, DARK.measure);
        painter.circle_filled(b, END_RADIUS, DARK.measure);
        let mid = a + (b - a) / 2.0;
        if !rect.contains(mid) {
            continue;
        }
        let distance = text::meters(m.distance());
        let size = ui.fonts_mut(|f| {
            f.layout_no_wrap(distance.clone(), font.clone(), DARK.text)
                .size()
        });
        let tag = Rect::from_center_size(mid - egui::vec2(0.0, 14.0), size + egui::vec2(12.0, 6.0));
        painter.rect_filled(tag, CornerRadius::same(6), DARK.float_fill);
        let label = egui::Label::new(RichText::new(distance).color(DARK.text)).selectable(false);
        ui.put(tag, label);
    }
    if let Some(a) = state.measure_a.and_then(screen) {
        painter.circle_filled(a, END_RADIUS, DARK.measure);
        if let Some(hover) = ui.ctx().pointer_hover_pos().filter(|p| rect.contains(*p)) {
            painter.extend(egui::Shape::dashed_line(&[a, hover], stroke, 6.0, 4.0));
        }
    }
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
