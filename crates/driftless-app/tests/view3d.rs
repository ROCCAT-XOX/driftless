//! The 3D view without a window: target size, click or drag, fly keys, colour modes, the
//! point info, and (with a GPU adapter) the view drawn by egui with its colours unchanged.

use driftless_app::camera::{fly_motion, FlyKeys};
use driftless_app::cloud::{LoadedCloud, PointSet};
use driftless_app::render::{headless_device, ColorMode};
use driftless_app::ui::view3d::{
    color_modes, effective_color_mode, fly_keys, is_click, point_info, target_size, view_bounds,
    View3d,
};
use driftless_app::ui::{right, text, ViewSettings};
use driftless_io::progress::Progress;
use eframe::egui;
use std::path::PathBuf;

fn cloud(path: &str, points: PointSet) -> LoadedCloud {
    LoadedCloud::build(PathBuf::from(path), points, 0.0, &Progress::new()).unwrap()
}

#[test]
fn view_target_size_uses_pixels_per_point() {
    assert_eq!(target_size(egui::vec2(400.0, 300.0), 2.0), [800, 600]);
    assert_eq!(target_size(egui::vec2(400.0, 300.0), 1.0), [400, 300]);
    assert_eq!(target_size(egui::vec2(100.4, 50.6), 1.5), [151, 76]);
    // Never 0, also not for a collapsed panel or a broken scale.
    assert_eq!(target_size(egui::vec2(0.0, 0.0), 2.0), [1, 1]);
    assert_eq!(target_size(egui::vec2(-5.0, f32::NAN), 2.0), [1, 1]);
}

#[test]
fn click_vs_drag_threshold() {
    assert!(is_click(1.0, 1.1, 0.0));
    assert!(is_click(1.0, 1.29, 3.9));
    // moved too far
    assert!(!is_click(1.0, 1.1, 4.0));
    assert!(!is_click(1.0, 1.1, 25.0));
    // held too long
    assert!(!is_click(1.0, 1.3, 0.0));
    assert!(!is_click(1.0, 3.0, 1.0));
}

#[test]
fn fly_keys_map_to_motion() {
    use egui::Key;
    let held = |keys: &[Key]| fly_keys(|k| keys.contains(&k));
    assert_eq!(held(&[]), FlyKeys::default());
    let wd = held(&[Key::W, Key::D]);
    assert!(wd.forward && wd.right && !wd.back && !wd.left && !wd.up && !wd.down);
    let (v, speed) = fly_motion(wd, false);
    let h = std::f32::consts::FRAC_1_SQRT_2;
    assert!((v[0] - h).abs() < 1e-6 && (v[1] - h).abs() < 1e-6 && v[2] == 0.0);
    assert_eq!(speed, 1.0);
    // S back, A left, E up, Q down; Shift faster.
    let (v, speed) = fly_motion(held(&[Key::S]), true);
    assert_eq!((v, speed), ([-1.0, 0.0, 0.0], 4.0));
    assert_eq!(fly_motion(held(&[Key::A]), false).0, [0.0, -1.0, 0.0]);
    assert_eq!(fly_motion(held(&[Key::E]), false).0, [0.0, 0.0, 1.0]);
    assert_eq!(fly_motion(held(&[Key::Q]), false).0, [0.0, 0.0, -1.0]);
    // Opposite keys cancel out.
    assert_eq!(fly_motion(held(&[Key::W, Key::S]), false).0, [0.0; 3]);
}

#[test]
fn color_mode_falls_back_without_rgb() {
    assert_eq!(
        color_modes(true),
        vec![ColorMode::Rgb, ColorMode::Intensity, ColorMode::Quality]
    );
    assert_eq!(
        color_modes(false),
        vec![ColorMode::Intensity, ColorMode::Quality]
    );
    assert_eq!(
        effective_color_mode(ColorMode::Rgb, false),
        ColorMode::Intensity
    );
    assert_eq!(effective_color_mode(ColorMode::Rgb, true), ColorMode::Rgb);
    assert_eq!(
        effective_color_mode(ColorMode::Quality, false),
        ColorMode::Quality
    );
}

#[test]
fn pick_info_formats_absolute_coordinates() {
    assert_eq!(text::metres(512_345.678), "512.345,678 m");
    assert_eq!(text::metres(-1_234.5), "-1.234,500 m");
    assert_eq!(text::metres(0.0004), "0,000 m");
    assert_eq!(text::metres(5_400_000.25), "5.400.000,250 m");

    let mut points = PointSet::from_xyz(
        [512_000.0, 5_400_000.0, 100.0],
        vec![[0.0; 3], [345.678, 0.25, -2.5]],
    );
    points.rgb = Some(vec![[1, 2, 3], [200, 120, 40]]);
    points.intensity = vec![10, 77];
    points.t = vec![0.0, 12.5];
    points.quality = vec![255, 204];
    let c = cloud("/k/color.las", points);
    let info = point_info(&c, 1);
    // f64 origin + f32 relative: millimetres survive
    for (got, want) in info.position.iter().zip([512_345.678, 5_400_000.25, 97.5]) {
        assert!((got - want).abs() < 1e-4, "{got} vs {want}");
    }
    let lines = right::point_lines(&info);
    let value = |label: &str| {
        lines
            .iter()
            .find(|(l, _)| *l == label)
            .map(|(_, v)| v.clone())
            .unwrap_or_else(|| panic!("no line {label}: {lines:?}"))
    };
    assert_eq!(value(text::POINT_X), "512.345,678 m");
    assert_eq!(value(text::POINT_Y), "5.400.000,250 m");
    assert_eq!(value(text::POINT_Z), "97,500 m");
    assert_eq!(value(text::POINT_TIME), "12,50 s");
    assert_eq!(value(text::POINT_INTENSITY), "77");
    assert_eq!(value(text::POINT_COLOR), "200 / 120 / 40");
    assert_eq!(value(text::POINT_QUALITY), "0,80");
    assert!(lines.iter().all(|(l, _)| *l != text::POINT_REMOVED));
}

#[test]
fn pick_info_without_colour_and_removed() {
    let mut points = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]; 3]);
    points.flags = vec![0, 1, 3];
    // A cloud without colours.
    let c = cloud("/k/dense.las", points.clone());
    let lines = right::point_lines(&point_info(&c, 0));
    assert!(lines.contains(&(text::POINT_COLOR, text::NO_COLOR.to_string())));
    // Flag 1 in a coloured cloud: no measured colour.
    points.rgb = Some(vec![[9, 9, 9]; 3]);
    let c = cloud("/k/color.las", points.clone());
    let colour = |c: &LoadedCloud, i| {
        right::point_lines(&point_info(c, i))
            .into_iter()
            .find(|(l, _)| *l == text::POINT_COLOR)
            .unwrap()
            .1
    };
    assert_eq!(colour(&c, 0), "9 / 9 / 9");
    assert_eq!(colour(&c, 1), text::NO_COLOR);
    // In removed.las the flags are the removal reason.
    let c = cloud("/k/removed.las", points);
    let lines = right::point_lines(&point_info(&c, 2));
    assert!(lines.contains(&(text::POINT_REMOVED, "Durchblick".to_string())));
}

#[test]
fn view_bounds_ignore_far_outliers() {
    assert_eq!(view_bounds(&PointSet::default()), None);
    let mut xyz: Vec<[f32; 3]> = (0..10_000)
        .map(|i| {
            [
                (i % 100) as f32 * 0.1,
                (i / 100) as f32 * 0.1,
                (i % 7) as f32,
            ]
        })
        .collect();
    xyz.push([10_000.0, -5_000.0, 300.0]);
    xyz.push([-8_000.0, 9_000.0, -40.0]);
    let (min, max) = view_bounds(&PointSet::from_xyz([0.0; 3], xyz)).unwrap();
    for k in 0..2 {
        assert!(min[k] >= 0.0 && min[k] < 0.5, "min {min:?}");
        assert!(max[k] < 9.91 && max[k] > 9.4, "max {max:?}");
    }
    assert!(min[2] >= 0.0 && max[2] <= 6.0);
    // a single point is its own box
    let one = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]]);
    assert_eq!(view_bounds(&one), Some(([1.0, 2.0, 3.0], [1.0, 2.0, 3.0])));
}

#[test]
fn removed_reason_text() {
    assert_eq!(text::removal_reason(1), Some("Geist"));
    assert_eq!(text::removal_reason(2), Some("Ausreißer"));
    assert_eq!(text::removal_reason(3), Some("Durchblick"));
    assert_eq!(text::removal_reason(0), None);
    assert_eq!(text::removal_reason(9), None);
}

#[test]
fn drawn_stats_text() {
    assert_eq!(
        text::drawn_stats(5_900_000, 15_600_000, 8.44),
        "gezeichnet 5,9 Mio. von 15,6 Mio. Punkten · 8,4 ms"
    );
    assert_eq!(
        text::drawn_stats(300_000, 300_000, 1.0),
        "gezeichnet 300.000 von 300.000 Punkten · 1,0 ms"
    );
}

#[test]
fn view_settings_defaults() {
    let s = ViewSettings::default();
    assert_eq!(s.point_px, 2.0);
    assert_eq!(s.budget, 6_000_000);
    assert!(text::BUDGET_RANGE.contains(&s.budget));
    assert_eq!(*text::BUDGET_RANGE.start(), 1_000_000);
    assert_eq!(*text::BUDGET_RANGE.end(), 30_000_000);
}

/// A 5 × 5 m plate of one colour, drawn with large points into a view of 200 × 160 points
/// by egui: the pixel in the middle of the window has the colour of the points, i.e. the
/// sRGB bytes are neither converted twice nor washed out.
#[test]
fn view_shows_point_colours_unchanged() {
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    use egui_kittest::Harness;

    let colour = [200u8, 120, 40];
    let xyz: Vec<[f32; 3]> = (0..51 * 51)
        .map(|i| [(i % 51) as f32 * 0.1, (i / 51) as f32 * 0.1, 0.0])
        .collect();
    let n = xyz.len();
    let mut points = PointSet::from_xyz([0.0; 3], xyz);
    points.rgb = Some(vec![colour; n]);
    let c = cloud("/k/color.las", points);
    let settings = ViewSettings {
        point_px: 12.0,
        round: false,
        ..ViewSettings::default()
    };
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut view = View3d::default();
    let r = rs.clone();
    let mut h = Harness::builder()
        .with_size([200.0, 160.0])
        .with_pixels_per_point(2.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui(move |ui| {
            egui::CentralPanel::no_frame().show(ui, |ui| {
                view.show(ui, &r, Some(&c), &settings);
            });
        });
    h.run();
    let image = h.render().unwrap();
    assert_eq!(image.dimensions(), (400, 320));
    let p = image.get_pixel(200, 160).0;
    // the corner is background, not the plate
    let corner = image.get_pixel(2, 318).0;
    assert!(corner[0] < 40, "corner {corner:?}");
    for k in 0..3 {
        assert!(
            p[k].abs_diff(colour[k]) <= 2,
            "pixel {p:?}, points {colour:?}"
        );
    }
}

/// The time of a frame of the 3D view (choosing the nodes plus the GPU) while orbiting a
/// synthetic cloud of 50 million points, at budgets of 6 and 12 million, in a view of
/// 1000 × 800 points at 2 pixels per point (Retina). Only prints. Run with
/// `cargo test -p driftless-app --release --test view3d -- --ignored --nocapture`.
#[test]
#[ignore = "50 million points, several GB of memory, needs a GPU"]
fn synthetic_50m_frame_time() {
    use driftless_app::synthetic::synthetic_cloud;
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    use egui_kittest::Harness;
    use std::time::Instant;

    let Some((_, _, info)) = headless_device() else {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    };
    let t = Instant::now();
    let points = synthetic_cloud(50_000_000, 1);
    let generate = t.elapsed().as_secs_f64();
    let c = cloud("synthetisch", points);
    eprintln!(
        "{}: generated {generate:.1} s, octree {:.1} s",
        info.name, c.build_seconds
    );
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let r = rs.clone();
    let state = (View3d::default(), c, ViewSettings::default());
    // building the harness draws the first frame: upload (UI thread) and pipelines
    let t = Instant::now();
    let mut h = Harness::builder()
        .with_size([1000.0, 800.0])
        .with_pixels_per_point(2.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(
            move |ui, (view, c, settings): &mut (View3d, LoadedCloud, ViewSettings)| {
                egui::CentralPanel::no_frame().show(ui, |ui| {
                    view.show(ui, &r, Some(c), settings);
                });
            },
            state,
        );
    eprintln!(
        "first frame with upload {:.0} ms",
        t.elapsed().as_secs_f64() * 1000.0
    );
    for budget in [6_000_000, 12_000_000] {
        h.state_mut().2.budget = budget;
        // the first frames upload the cloud and build the pipelines
        for _ in 0..3 {
            h.step();
        }
        for distance in [1.0f32, 0.3, 0.1] {
            let (view, ..) = h.state_mut();
            let full = view.camera().distance;
            view.camera_mut().distance = full * distance;
            let mut times = Vec::new();
            let mut drawn = 0;
            for _ in 0..60 {
                h.state_mut().0.camera_mut().orbit(6.0, 0.0);
                h.step();
                times.push(h.state().0.frame_ms());
                drawn = h.state().0.drawn_points();
            }
            h.state_mut().0.camera_mut().distance = full;
            times.sort_by(f32::total_cmp);
            eprintln!(
                "budget {budget}, distance ×{distance}: {drawn} points drawn, frame median {:.1} ms, 90 % {:.1} ms, max {:.1} ms",
                times[times.len() / 2],
                times[times.len() * 9 / 10],
                times[times.len() - 1]
            );
        }
    }
}
