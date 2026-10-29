//! The dark theme: colours, rounding, spacing, the icon font and the soft view background.
//! Colours of the surface come from here only.

use eframe::egui::{
    self, epaint::Vertex, Color32, Context, CornerRadius, Frame, Margin, Mesh, Painter, Rect,
    Shadow, Stroke,
};

/// The colours of the surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Palette {
    /// Ground behind everything.
    pub bg: Color32,
    /// Side panels and bars.
    pub panel: Color32,
    /// The field of the active step and text fields (`#151920` in the design).
    pub field: Color32,
    /// Cards and buttons.
    pub surface: Color32,
    /// Hovered or raised surfaces.
    pub surface_hi: Color32,
    pub accent: Color32,
    /// Text and symbols on [`Palette::accent`].
    pub on_accent: Color32,
    pub text: Color32,
    pub text_dim: Color32,
    pub ok: Color32,
    pub warn: Color32,
    pub error: Color32,
    /// Fill of the floating bars over the view (translucent).
    pub float_fill: Color32,
    /// Edge of the floating bars.
    pub float_stroke: Color32,
    /// The light middle of the view's gradient; its edges are [`Palette::bg`].
    pub view_glow: Color32,
    /// Lines and ends of the measurements in the view.
    pub measure: Color32,
}

/// The only theme.
pub const DARK: Palette = Palette {
    bg: Color32::from_rgb(0x0f, 0x11, 0x15),
    panel: Color32::from_rgb(0x12, 0x15, 0x1a),
    field: Color32::from_rgb(0x15, 0x19, 0x20),
    surface: Color32::from_rgb(0x1b, 0x1f, 0x27),
    surface_hi: Color32::from_rgb(0x2a, 0x30, 0x3b),
    accent: Color32::from_rgb(0x2f, 0x6f, 0xed),
    on_accent: Color32::WHITE,
    text: Color32::from_rgb(0xc9, 0xce, 0xd6),
    text_dim: Color32::from_rgb(0x8a, 0x91, 0x9c),
    ok: Color32::from_rgb(0x3e, 0xcf, 0x8e),
    warn: Color32::from_rgb(0xe0, 0xa4, 0x3a),
    error: Color32::from_rgb(0xe5, 0x5b, 0x5b),
    float_fill: Color32::from_rgba_unmultiplied_const(22, 26, 33, 217),
    // white at 14/255, premultiplied
    float_stroke: Color32::from_rgba_premultiplied(14, 14, 14, 14),
    view_glow: Color32::from_rgb(0x2a, 0x2f, 0x38),
    measure: Color32::from_rgb(0xf2, 0xc9, 0x4c),
};

/// Corner radius of buttons and fields.
const WIDGET_RADIUS: u8 = 8;
/// Corner radius of the floating bars.
const FLOAT_RADIUS: u8 = 12;

/// Symbols of the Phosphor font (MIT), as text. Needs [`apply`].
pub mod icon {
    use egui_phosphor::regular as p;

    pub const MENU: &str = p::LIST;
    pub const DATA: &str = p::CUBE;
    pub const IMAGES: &str = p::IMAGES;
    pub const PLAN: &str = p::MAP_TRIFOLD;
    pub const SETTINGS: &str = p::GEAR;
    pub const ORBIT: &str = p::ARROWS_CLOCKWISE;
    pub const PAN: &str = p::HAND;
    pub const PICK: &str = p::CROSSHAIR;
    pub const MEASURE: &str = p::RULER;
    pub const CUT: &str = p::SCISSORS;
    pub const BRUSH: &str = p::PAINT_BRUSH;
    /// ↶: takes back the last stroke or step of the brush.
    pub const UNDO: &str = p::ARROW_U_UP_LEFT;
    pub const PAIRS: &str = p::LINK;
    pub const FIT: &str = p::CORNERS_OUT;
    pub const CHECK: &str = p::CHECK_CIRCLE;
    pub const RUNNING: &str = p::CIRCLE_NOTCH;
    pub const STALE: &str = p::WARNING_CIRCLE;
    /// A scan placed in a model, but not surely.
    pub const UNSURE: &str = p::WARNING;
    /// A scan that could not be placed in a model.
    pub const FAILED: &str = p::X_CIRCLE;
    pub const ADD: &str = p::PLUS;
    pub const FOLDED: &str = p::CARET_RIGHT;
    pub const UNFOLDED: &str = p::CARET_DOWN;
    pub const CLOSE: &str = p::X;
    /// A hint for the user (of a surface): information, not a warning.
    pub const HINT: &str = p::INFO;
}

/// Size of the heading of a step (the body text is 12.5).
pub const HEADING_SIZE: f32 = 20.0;
/// Corner radius of the cards and the segments of a switch.
pub const CARD_RADIUS: u8 = WIDGET_RADIUS;

/// Frame of a card in the field of a step (a scan); `selected` draws an accent edge.
pub fn card_frame(selected: bool) -> Frame {
    let stroke = if selected {
        Stroke::new(1.0, DARK.accent)
    } else {
        Stroke::NONE
    };
    Frame::new()
        .fill(DARK.surface)
        .stroke(stroke)
        .corner_radius(CornerRadius::same(CARD_RADIUS))
        .inner_margin(Margin::same(9))
}

/// Frame of a segmented switch (e.g. the precision): a dark trough around its segments.
pub fn segment_frame() -> Frame {
    Frame::new()
        .fill(DARK.surface)
        .corner_radius(CornerRadius::same(CARD_RADIUS))
        .inner_margin(Margin::same(3))
}

/// Sets the dark visuals, spacing and the icon font. Call once at start.
pub fn apply(ctx: &Context) {
    let mut fonts = egui::FontDefinitions::default();
    egui_phosphor::add_to_fonts(&mut fonts, egui_phosphor::Variant::Regular);
    ctx.set_fonts(fonts);

    let p = DARK;
    let mut v = egui::Visuals::dark();
    v.override_text_color = Some(p.text);
    v.panel_fill = p.panel;
    v.window_fill = p.panel;
    v.faint_bg_color = p.surface;
    v.extreme_bg_color = p.field;
    v.window_stroke = Stroke::new(1.0, p.surface_hi);
    v.window_corner_radius = CornerRadius::same(FLOAT_RADIUS);
    v.menu_corner_radius = CornerRadius::same(WIDGET_RADIUS);
    v.selection.bg_fill = p.accent;
    v.selection.stroke = Stroke::new(1.0, p.on_accent);
    v.hyperlink_color = p.accent;
    v.warn_fg_color = p.warn;
    v.error_fg_color = p.error;
    v.popup_shadow = Shadow::NONE;
    v.window_shadow = Shadow::NONE;

    let w = &mut v.widgets;
    let radius = CornerRadius::same(WIDGET_RADIUS);
    w.noninteractive.bg_fill = p.panel;
    w.noninteractive.weak_bg_fill = p.panel;
    w.noninteractive.bg_stroke = Stroke::new(1.0, p.surface);
    w.noninteractive.fg_stroke = Stroke::new(1.0, p.text_dim);
    w.inactive.bg_fill = p.surface;
    w.inactive.weak_bg_fill = p.surface;
    w.inactive.bg_stroke = Stroke::NONE;
    w.inactive.fg_stroke = Stroke::new(1.0, p.text);
    w.hovered.bg_fill = p.surface_hi;
    w.hovered.weak_bg_fill = p.surface_hi;
    w.hovered.bg_stroke = Stroke::new(1.0, p.surface_hi);
    w.hovered.fg_stroke = Stroke::new(1.0, Color32::WHITE);
    w.active.bg_fill = p.accent;
    w.active.weak_bg_fill = p.accent;
    w.active.bg_stroke = Stroke::NONE;
    w.active.fg_stroke = Stroke::new(1.0, p.on_accent);
    w.open.bg_fill = p.surface_hi;
    w.open.weak_bg_fill = p.surface_hi;
    for s in [
        &mut w.noninteractive,
        &mut w.inactive,
        &mut w.hovered,
        &mut w.active,
        &mut w.open,
    ] {
        s.corner_radius = radius;
    }
    // The surface is dark whatever the system says.
    ctx.set_theme(egui::Theme::Dark);
    ctx.set_visuals_of(egui::Theme::Dark, v);

    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = egui::vec2(8.0, 8.0);
        style.spacing.button_padding = egui::vec2(10.0, 6.0);
        style.spacing.interact_size.y = 28.0;
        style.spacing.window_margin = Margin::same(12);
    });
}

/// Corner radius of the buttons of the rail.
pub const RAIL_RADIUS: u8 = 10;

/// Frame of the rail at the left edge.
pub fn rail_frame() -> Frame {
    Frame::new()
        .fill(DARK.panel)
        .inner_margin(Margin::symmetric(8, 12))
}

/// Frame of the field of the active step, next to the rail.
pub fn step_frame() -> Frame {
    Frame::new().fill(DARK.field).inner_margin(Margin::same(14))
}

/// Frame of the floating bars over the view: translucent, rounded.
pub fn float_frame() -> Frame {
    Frame::new()
        .fill(DARK.float_fill)
        .stroke(Stroke::new(1.0, DARK.float_stroke))
        .corner_radius(CornerRadius::same(FLOAT_RADIUS))
        .inner_margin(Margin::same(8))
}

/// Paints the soft gradient behind the 3D view: [`Palette::view_glow`] in the lower middle,
/// [`Palette::bg`] at the edges. One mesh of a few triangles; the cloud is drawn over it with a transparent clear.
pub fn view_background(painter: &Painter, rect: Rect) {
    const SEGMENTS: usize = 32;
    let inner = DARK.view_glow;
    let outer = DARK.bg;
    // Ellipse centre at 50% / 60% of the rect; the colour reaches the edge colour at 72%.
    let centre = egui::pos2(rect.center().x, rect.top() + rect.height() * 0.6);
    // CSS "farthest-corner" ellipse: radii are the farthest side distances times sqrt 2.
    let radius =
        egui::vec2(rect.width() * 0.5, rect.height() * 0.6) * std::f32::consts::SQRT_2 * 0.72;
    painter.rect_filled(rect, 0.0, outer);
    let mut mesh = Mesh::default();
    let uv = egui::epaint::WHITE_UV;
    mesh.vertices.push(Vertex {
        pos: centre,
        uv,
        color: inner,
    });
    for i in 0..=SEGMENTS {
        let a = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        mesh.vertices.push(Vertex {
            pos: centre + egui::vec2(a.cos() * radius.x, a.sin() * radius.y),
            uv,
            color: outer,
        });
    }
    for i in 0..SEGMENTS as u32 {
        mesh.indices.extend([0, i + 1, i + 2]);
    }
    // Keep the spill of the ellipse inside the view.
    painter.with_clip_rect(rect).add(mesh);
}
