//! The dark theme and the icon font.

use driftless_app::ui::theme::{self, icon, DARK};
use eframe::egui;

#[test]
fn theme_sets_dark_visuals() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let v = ctx.global_style().visuals.clone();
    assert!(v.dark_mode);
    assert_eq!(v.panel_fill, DARK.panel);
    let r = v.widgets.inactive.corner_radius;
    assert!((8..=12).contains(&r.nw), "radius {}", r.nw);
    assert_eq!(v.selection.bg_fill, DARK.accent);
}

#[test]
fn icons_render_in_font() {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let all = [
        icon::MENU,
        icon::DATA,
        icon::IMAGES,
        icon::PLAN,
        icon::SETTINGS,
        icon::ORBIT,
        icon::PAN,
        icon::PICK,
        icon::MEASURE,
        icon::CUT,
        icon::FIT,
        icon::CHECK,
        icon::RUNNING,
    ];
    // Fonts are laid out on the first frame.
    let mut out = ctx.run_ui(Default::default(), |_| {});
    out.textures_delta.clear();
    ctx.fonts_mut(|f| {
        for s in all {
            assert!(!s.is_empty());
            let font = egui::FontId::proportional(14.0);
            assert!(f.has_glyphs(&font, s), "missing glyph in {s:?}");
        }
    });
}

#[test]
fn float_frame_is_translucent_and_round() {
    let f = theme::float_frame();
    assert_eq!(f.corner_radius.nw, 12);
    assert!(f.fill.a() < 255 && f.fill.a() > 0);
}
