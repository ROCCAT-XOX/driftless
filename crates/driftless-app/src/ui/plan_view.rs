//! The plan in the central area of „Planung“: a 2D surface of its own, drawn with the painter of
//! egui from the same [`Drawing`] the DXF is written from. The 3D view stays untouched (see the
//! spec: a camera from above would change the size of the points and with it the renderer).
//!
//! The mouse wheel zooms to the pointer, dragging moves the plan, `F` fits it. Texts and the
//! picture from above are drawn with a name for screen readers and tests, only as long as they
//! are on the screen.

use super::theme::DARK;
use super::tools::keys_free;
use super::{text, Action, AppState};
use driftless_export::dxf::{Drawing, EntityKind, Shape as DrawnShape};
use driftless_steps::plan::TopViewInfo;
use driftless_steps::plan_drawing::LAYERS;
use eframe::egui::epaint::TextShape;
use eframe::egui::{
    self, Color32, FontId, Id, Key, Pos2, Rect, Sense, Shape, Stroke, TextureHandle,
    TextureOptions, Ui, Vec2, WidgetInfo, WidgetType,
};
use std::collections::BTreeMap;
use std::path::Path;
use std::sync::Arc;

/// Room around the plan when it is fitted (points).
const FIT_MARGIN: f32 = 40.0;
/// One step of the wheel changes the scale by this much.
const ZOOM_STEP: f64 = 1.2;
/// Points the wheel scrolls per step.
const POINTS_PER_STEP: f32 = 50.0;
/// The closest and the furthest view (mm per point).
const MIN_SCALE: f64 = 0.05;
const MAX_SCALE: f64 = 1.0e5;
/// Line widths (points): the walls stronger than the rest.
const WALL_WIDTH: f32 = 1.8;
const LINE_WIDTH: f32 = 1.0;
/// Dashes of a dashed line (points).
const DASH: f32 = 6.0;
const GAP: f32 = 4.0;
/// A text smaller than this cannot be read and is left out (points).
const MIN_TEXT: f32 = 4.0;
/// Number next to the symbol of a block, as the DXF has it (mm): its height, and where it stands
/// from the insertion point.
const NUMBER_HEIGHT: f64 = 100.0;
const NUMBER_X: f64 = 120.0;
/// Radius of the ring around the object marked in the list (points).
const MARK_RING: f32 = 12.0;

/// Where the plan lies on the surface. `mm_per_point` 0 means not placed yet: it is fitted at
/// the next frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PlanCamera {
    pub centre_mm: [f64; 2],
    pub mm_per_point: f64,
}

impl PlanCamera {
    /// Shows `min`…`max` (mm) whole on a surface of `size` (points).
    pub fn fit(&mut self, min: [f64; 2], max: [f64; 2], size: [f32; 2]) {
        self.centre_mm = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0];
        let usable = [
            f64::from((size[0] - 2.0 * FIT_MARGIN).max(1.0)),
            f64::from((size[1] - 2.0 * FIT_MARGIN).max(1.0)),
        ];
        let scale =
            ((max[0] - min[0]).max(1.0) / usable[0]).max((max[1] - min[1]).max(1.0) / usable[1]);
        self.mm_per_point = scale.clamp(MIN_SCALE, MAX_SCALE);
    }

    /// Zooms by `steps` of the wheel (positive: closer) about `cursor`, whose point stays under
    /// it. `cursor` as [`Self::to_screen`] gives it, from the top left of the surface.
    pub fn zoom_at(&mut self, cursor: [f32; 2], size: [f32; 2], steps: f32) {
        let before = self.plan_point(cursor, size);
        let factor = ZOOM_STEP.powf(-f64::from(steps));
        self.mm_per_point = (self.mm_per_point * factor).clamp(MIN_SCALE, MAX_SCALE);
        let after = self.plan_point(cursor, size);
        self.centre_mm[0] += before[0] - after[0];
        self.centre_mm[1] += before[1] - after[1];
    }

    /// Where the point `p` (mm, y up) lies, from the top left of the surface (points, y down).
    pub fn to_screen(&self, p: [f64; 2], size: [f32; 2]) -> [f32; 2] {
        [
            size[0] / 2.0 + ((p[0] - self.centre_mm[0]) / self.mm_per_point) as f32,
            size[1] / 2.0 - ((p[1] - self.centre_mm[1]) / self.mm_per_point) as f32,
        ]
    }

    /// The point (mm) under `screen`, the reverse of [`Self::to_screen`].
    fn plan_point(&self, screen: [f32; 2], size: [f32; 2]) -> [f64; 2] {
        [
            self.centre_mm[0] + f64::from(screen[0] - size[0] / 2.0) * self.mm_per_point,
            self.centre_mm[1] - f64::from(screen[1] - size[1] / 2.0) * self.mm_per_point,
        ]
    }

    /// Moves the plan by `delta` points on the surface.
    fn drag(&mut self, delta: Vec2) {
        self.centre_mm[0] -= f64::from(delta.x) * self.mm_per_point;
        self.centre_mm[1] += f64::from(delta.y) * self.mm_per_point;
    }
}

/// The smallest and largest point (mm) of what is drawn on the layers switched on; a square
/// metre around the origin for an empty drawing.
pub fn bounds(drawing: &Drawing) -> ([f64; 2], [f64; 2]) {
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    let mut add = |p: [f64; 2]| {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    };
    let on: Vec<&str> = drawing
        .layers
        .iter()
        .filter(|l| l.on)
        .map(|l| l.name.as_str())
        .collect();
    for entity in drawing
        .entities
        .iter()
        .filter(|e| on.contains(&e.layer.as_str()))
    {
        match &entity.kind {
            EntityKind::Insert { at, .. } => add(*at),
            EntityKind::Shape(DrawnShape::Line { a, b, .. }) => {
                add(*a);
                add(*b);
            }
            EntityKind::Shape(DrawnShape::Polyline { points, .. }) => {
                points.iter().for_each(|p| add(*p));
            }
            EntityKind::Shape(DrawnShape::Circle { centre, radius }) => {
                add([centre[0] - radius, centre[1] - radius]);
                add([centre[0] + radius, centre[1] + radius]);
            }
            EntityKind::Shape(DrawnShape::Text { at, .. }) => add(*at),
        }
    }
    if min[0] <= max[0] && min[1] <= max[1] {
        (min, max)
    } else {
        ([0.0, 0.0], [1000.0, 1000.0])
    }
}

/// The colour of an AutoCAD colour index on the dark background, for the indices of [`LAYERS`].
pub(super) fn layer_colour(index: u8) -> Color32 {
    match index {
        1 => Color32::from_rgb(235, 87, 87),
        2 => Color32::from_rgb(235, 205, 90),
        3 => Color32::from_rgb(100, 200, 120),
        4 => Color32::from_rgb(90, 200, 220),
        5 => Color32::from_rgb(110, 140, 235),
        6 => Color32::from_rgb(225, 110, 210),
        7 => Color32::from_rgb(235, 235, 235),
        8 => Color32::from_rgb(150, 150, 150),
        30 => Color32::from_rgb(240, 150, 60),
        34 => Color32::from_rgb(190, 130, 80),
        200 => Color32::from_rgb(170, 110, 225),
        _ => Color32::GRAY,
    }
}

/// The picture from above, uploaded once per plan (kept in the memory of egui: the plan state
/// holds the image itself, the texture belongs to the window).
fn top_view_texture(ui: &Ui, key: usize, name: &Path, image: &egui::ColorImage) -> TextureHandle {
    let id = Id::new("grundriss_draufsicht");
    if let Some((held, texture)) = ui.data(|d| d.get_temp::<(usize, TextureHandle)>(id)) {
        if held == key {
            return texture;
        }
    }
    let texture = ui.ctx().load_texture(
        name.display().to_string(),
        image.clone(),
        TextureOptions::LINEAR,
    );
    ui.data_mut(|d| d.insert_temp(id, (key, texture.clone())));
    texture
}

/// What [`draw`] needs of the surface.
struct Surface {
    painter: egui::Painter,
    rect: Rect,
    size: [f32; 2],
    camera: PlanCamera,
    /// Texts drawn, as (rectangle, text) for the names.
    texts: Vec<(Rect, String)>,
}

impl Surface {
    fn at(&self, p: [f64; 2]) -> Pos2 {
        let s = self.camera.to_screen(p, self.size);
        self.rect.min + egui::vec2(s[0], s[1])
    }

    /// A line through `points`, dashed if asked.
    fn line(&self, points: Vec<Pos2>, stroke: Stroke, dashed: bool) {
        if dashed {
            self.painter
                .extend(Shape::dashed_line(&points, stroke, DASH, GAP));
        } else {
            self.painter.add(Shape::line(points, stroke));
        }
    }

    /// A text with its left lower corner at `at` (mm), turned by `rotation_deg` counter-clockwise.
    fn text(&mut self, at: [f64; 2], height: f64, rotation_deg: f64, value: &str, colour: Color32) {
        let size = (height / self.camera.mm_per_point) as f32;
        if size < MIN_TEXT {
            return;
        }
        // whole sizes only: every size is one more set of glyphs in the font atlas
        let font = FontId::proportional(size.round());
        let galley = self.painter.layout_no_wrap(value.to_string(), font, colour);
        // the top left corner of the text, about which it is turned
        let turn = rotation_deg.to_radians();
        let up = [-turn.sin(), turn.cos()];
        let corner = self.at([at[0] + up[0] * height, at[1] + up[1] * height]);
        let rect = Rect::from_min_size(corner, galley.size());
        self.painter
            .add(TextShape::new(corner, galley, colour).with_angle(-turn as f32));
        self.texts.push((rect, value.to_string()));
    }

    /// `shape`, moved by `offset` (mm), on a layer with this colour and line width.
    fn shape(&mut self, shape: &DrawnShape, offset: [f64; 2], colour: Color32, width: f32) {
        let stroke = Stroke::new(width, colour);
        let moved = |p: [f64; 2]| [p[0] + offset[0], p[1] + offset[1]];
        match shape {
            DrawnShape::Line { a, b, dashed } => {
                let points = vec![self.at(moved(*a)), self.at(moved(*b))];
                self.line(points, stroke, *dashed);
            }
            DrawnShape::Polyline {
                points,
                closed,
                dashed,
            } => {
                let mut pts: Vec<Pos2> = points.iter().map(|p| self.at(moved(*p))).collect();
                if *closed {
                    pts.extend(pts.first().copied());
                }
                self.line(pts, stroke, *dashed);
            }
            DrawnShape::Circle { centre, radius } => {
                let c = self.at(moved(*centre));
                self.painter.add(Shape::circle_stroke(
                    c,
                    (*radius / self.camera.mm_per_point) as f32,
                    stroke,
                ));
            }
            DrawnShape::Text {
                at,
                height,
                rotation_deg,
                value,
            } => self.text(moved(*at), *height, *rotation_deg, value, colour),
        }
    }
}

/// The picture from above under the plan: its corners by [`TopViewInfo`]; row 0 of the file is
/// the largest y.
fn draw_top_view(ui: &Ui, surface: &Surface, texture: &TextureHandle, info: &TopViewInfo) {
    let min = [info.origin[0] * 1000.0, info.origin[1] * 1000.0];
    let max = [
        min[0] + f64::from(info.width) * info.cell * 1000.0,
        min[1] + f64::from(info.height) * info.cell * 1000.0,
    ];
    let rect = Rect::from_two_pos(surface.at([min[0], max[1]]), surface.at([max[0], min[1]]));
    let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
    surface
        .painter
        .image(texture.id(), rect, uv, Color32::from_white_alpha(140));
    let view = rect.intersect(surface.rect);
    if view.width() > 0.0 && view.height() > 0.0 {
        ui.interact(view, Id::new("grundriss_bild"), Sense::hover())
            .widget_info(|| {
                WidgetInfo::labeled(WidgetType::Label, true, text::PLAN_TOP_VIEW_PICTURE)
            });
    }
}

/// The ring around object `nr` of the list („Klick markiert im Plan“), on a line a thick stroke
/// over it. Nothing if its layer is off.
fn mark_object(surface: &Surface, drawing: &Drawing, nr: u32) {
    let number = nr.to_string();
    let tag = format!("NR={nr}");
    let stroke = Stroke::new(WALL_WIDTH + 1.5, DARK.accent);
    let on = |layer: &str| drawing.layers.iter().any(|l| l.on && l.name == layer);
    for entity in drawing.entities.iter().filter(|e| on(&e.layer)) {
        match &entity.kind {
            EntityKind::Insert { at, attributes, .. }
                if attributes.first().is_some_and(|(_, v)| *v == number) =>
            {
                surface
                    .painter
                    .circle_stroke(surface.at(*at), MARK_RING, stroke);
            }
            EntityKind::Shape(DrawnShape::Polyline { points, .. })
                if entity.xdata.first() == Some(&tag) =>
            {
                let pts = points.iter().map(|p| surface.at(*p)).collect();
                surface.painter.add(Shape::line(pts, stroke));
            }
            _ => {}
        }
    }
}

/// Draws the plan shown on all the space of `ui`, and handles moving and zooming it.
pub fn show(ui: &mut Ui, state: &mut AppState) -> Option<Action> {
    let rect = ui.max_rect();
    let size = [rect.width(), rect.height()];
    let id = ui.id().with("grundriss_flaeche");
    let response = ui.interact(rect, id, Sense::click_and_drag());
    let shown = state.plan.shown.clone()?;
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, DARK.bg);

    let (min, max) = bounds(&shown.drawing);
    let camera = &mut state.plan.camera;
    if camera.mm_per_point <= 0.0 {
        camera.fit(min, max, size);
    }
    if keys_free(ui.ctx()) && ui.input(|i| i.key_pressed(Key::F)) {
        camera.fit(min, max, size);
    }
    if response.dragged() {
        camera.drag(response.drag_delta());
    }
    if ui.rect_contains_pointer(rect) {
        let (scroll, zoom) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
        let steps = scroll / POINTS_PER_STEP + (zoom.ln() / (ZOOM_STEP as f32).ln());
        let pointer = ui.input(|i| i.pointer.hover_pos());
        if let (Some(pointer), true) = (pointer, steps != 0.0) {
            let at = pointer - rect.min;
            camera.zoom_at([at.x, at.y], size, steps);
            ui.ctx().request_repaint();
        }
    }
    let camera = *camera;
    let show_picture = state.plan.top_view;

    let mut surface = Surface {
        painter,
        rect,
        size,
        camera,
        texts: Vec::new(),
    };
    if let (true, Some(image)) = (show_picture, &shown.top_view) {
        let key = Arc::as_ptr(&shown) as usize;
        let texture = top_view_texture(ui, key, &shown.dir, image);
        draw_top_view(ui, &surface, &texture, &shown.report.top_view);
    }
    let layers: BTreeMap<&str, (bool, u8)> = shown
        .drawing
        .layers
        .iter()
        .map(|l| (l.name.as_str(), (l.on, l.color)))
        .collect();
    for entity in &shown.drawing.entities {
        let Some(&(true, index)) = layers.get(entity.layer.as_str()) else {
            continue;
        };
        let colour = layer_colour(index);
        let width = if entity.layer == LAYERS[0].0 {
            WALL_WIDTH
        } else {
            LINE_WIDTH
        };
        match &entity.kind {
            EntityKind::Shape(shape) => surface.shape(shape, [0.0, 0.0], colour, width),
            EntityKind::Insert {
                block,
                at,
                attributes,
            } => {
                if let Some(block) = shown.drawing.blocks.iter().find(|b| b.name == *block) {
                    for shape in &block.shapes {
                        surface.shape(shape, *at, colour, width);
                    }
                }
                // the first attribute, the number, is the visible one, as in the DXF
                if let Some((_, value)) = attributes.first() {
                    let place = [at[0] + NUMBER_X, at[1]];
                    surface.text(place, NUMBER_HEIGHT, 0.0, value, colour);
                }
            }
        }
    }
    if let Some(nr) = state.plan.selected_object {
        mark_object(&surface, &shown.drawing, nr);
    }
    // the names of the texts on the surface, for screen readers and the tests
    let texts = std::mem::take(&mut surface.texts);
    for (i, (text_rect, value)) in texts.iter().enumerate() {
        let on_surface = text_rect.intersect(rect);
        if on_surface.width() <= 0.0 || on_surface.height() <= 0.0 {
            continue;
        }
        ui.interact(on_surface, id.with(("text", i)), Sense::hover())
            .widget_info(|| WidgetInfo::labeled(WidgetType::Label, true, value));
    }
    None
}
