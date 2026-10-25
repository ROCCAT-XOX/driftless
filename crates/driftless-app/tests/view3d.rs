//! The 3D view without a window: target size, click or drag, fly keys, colour modes, the
//! point info, and (with a GPU adapter) the view drawn by egui with its colours unchanged.

use driftless_app::camera::{fly_motion, FlyKeys};
use driftless_app::cloud::{LoadedCloud, PointSet};
use driftless_app::render::{headless_device, ColorMode, PointSize, ShowMode};
use driftless_app::ui::view3d::{
    color_modes, effective_color_mode, effective_show, fly_keys, is_click, point_info, target_size,
    view_bounds, View3d,
};
use driftless_app::ui::{steps, text, ViewSettings};
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
    points.quality_known = true;
    let c = cloud("/k/color.las", points);
    let info = point_info(&c, 1);
    // f64 origin + f32 relative: millimetres survive
    for (got, want) in info.position.iter().zip([512_345.678, 5_400_000.25, 97.5]) {
        assert!((got - want).abs() < 1e-4, "{got} vs {want}");
    }
    let lines = steps::point_lines(&info);
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

/// Without a SLAM report the quality is unknown, not perfect.
#[test]
fn pick_info_quality_unknown_without_report() {
    let points = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]]);
    assert!(!points.quality_known);
    let c = cloud("/k/dense.las", points);
    let info = point_info(&c, 0);
    assert_eq!(info.quality, None);
    let lines = steps::point_lines(&info);
    assert!(lines.contains(&(text::POINT_QUALITY, text::QUALITY_UNKNOWN.to_string())));
}

/// A box room (floor, four walls, ceiling at 3 m, as the synthetic rooms): the ceiling is
/// found just below 3 m, above most of the walls; a flat cloud has none.
#[test]
fn ceiling_estimate_on_box_room() {
    use driftless_app::cloud::estimate_ceiling;
    use driftless_app::synthetic::synthetic_cloud;
    let room = synthetic_cloud(250_000, 3);
    let z = estimate_ceiling(&room).expect("ceiling");
    assert!(z > 2.7 && z < 3.0, "ceiling estimate {z}");
    // the cut keeps the walls up to there and hides the whole ceiling plate
    let ceiling_points = room.xyz.iter().filter(|p| p[2] > 2.99).count();
    let hidden = room.xyz.iter().filter(|p| p[2] > z).count();
    assert!(hidden >= ceiling_points && hidden < ceiling_points * 2);
    let floor = PointSet::from_xyz(
        [0.0; 3],
        (0..10_000)
            .map(|i| [(i % 100) as f32 * 0.1, (i / 100) as f32 * 0.1, 0.0])
            .collect(),
    );
    assert_eq!(estimate_ceiling(&floor), None);
    assert_eq!(estimate_ceiling(&PointSet::default()), None);
    // computed in the load job, with the z range for the sliders
    let c = cloud("/k/raum.las", room);
    assert_eq!(c.ceiling, Some(z));
    assert!(c.z_range[0] < 0.0 && c.z_range[1] > 3.0);
}

/// The planner of the sliced upload: never more than the limit per call, every vertex once,
/// in order.
#[test]
fn upload_slices_respect_limit() {
    use driftless_app::render::{upload_slices, UploadCursor};
    let chunks = vec![0..10, 10..13, 13..30];
    let mut cursor = UploadCursor::default();
    let mut seen = Vec::new();
    loop {
        let slices = upload_slices(&chunks, &mut cursor, 4);
        if slices.is_empty() {
            break;
        }
        let n: u32 = slices.iter().map(|(_, r)| r.end - r.start).sum();
        assert!(n <= 4 && n > 0, "{slices:?}");
        for (b, r) in slices {
            for v in r {
                seen.push(chunks[b].start + v);
            }
        }
    }
    assert_eq!(seen, (0..30).collect::<Vec<u32>>());
    assert_eq!(cursor.block, 3);
}

#[test]
fn pick_info_without_colour_and_removed() {
    let mut points = PointSet::from_xyz([0.0; 3], vec![[1.0, 2.0, 3.0]; 3]);
    points.flags = vec![0, 1, 3];
    // A cloud without colours.
    let c = cloud("/k/dense.las", points.clone());
    let lines = steps::point_lines(&point_info(&c, 0));
    assert!(lines.contains(&(text::POINT_COLOR, text::NO_COLOR.to_string())));
    // Flag 1 in a coloured cloud: no measured colour.
    points.rgb = Some(vec![[9, 9, 9]; 3]);
    let c = cloud("/k/color.las", points.clone());
    let colour = |c: &LoadedCloud, i| {
        steps::point_lines(&point_info(c, i))
            .into_iter()
            .find(|(l, _)| *l == text::POINT_COLOR)
            .unwrap()
            .1
    };
    assert_eq!(colour(&c, 0), "9 / 9 / 9");
    assert_eq!(colour(&c, 1), text::NO_COLOR);
    // In removed.las the flags are the removal reason.
    let c = cloud("/k/removed.las", points);
    let lines = steps::point_lines(&point_info(&c, 2));
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
        text::drawn_stats(5_900_000, 15_600_000, 8.44, true),
        "gezeichnet 5,9 Mio. von 15,6 Mio. Punkten · 8,4 ms GPU"
    );
    assert_eq!(
        text::drawn_stats(300_000, 300_000, 16.7, false),
        "gezeichnet 300.000 von 300.000 Punkten · 16,7 ms je Bild"
    );
    assert_eq!(text::uploading(40.0), "Wolke wird hochgeladen … 40 %");
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
        // fixed squares: world-size spots are darker at their rim
        point_size: PointSize::Pixel,
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
                view.show(ui, &r, Some(&c), None, &settings);
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

/// A new cloud reaches the GPU in slices: no frame writes more than
/// [`UPLOAD_SLICE_BYTES`], the view says so meanwhile, and then draws the whole cloud.
#[test]
fn upload_is_spread_over_frames() {
    use driftless_app::ui::view3d::UPLOAD_SLICE_BYTES;
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    use egui_kittest::Harness;
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    // 3 million points: 60 MB, more than one slice
    let points = driftless_app::synthetic::synthetic_cloud(3_000_000, 2);
    let c = cloud("/k/gross.las", points);
    let total = c.tree.vertices.len() as u64 * driftless_app::render::VERTEX_BYTES;
    assert!(total > UPLOAD_SLICE_BYTES);
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let r = rs.clone();
    let mut h = Harness::builder()
        .with_size([300.0, 200.0])
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(
            move |ui, (view, c): &mut (View3d, Option<LoadedCloud>)| {
                egui::CentralPanel::no_frame().show(ui, |ui| {
                    view.show(ui, &r, c.as_ref(), None, &ViewSettings::default());
                });
            },
            (View3d::default(), None),
        );
    // the cloud arrives after the harness has settled, as from the loader
    h.state_mut().1 = Some(c);
    let mut written = 0;
    let mut frames = 0;
    while frames == 0 || h.state().0.is_uploading() {
        h.step();
        let bytes = h.state().0.last_upload_bytes();
        assert!(bytes <= UPLOAD_SLICE_BYTES, "{bytes} bytes in one frame");
        written += bytes;
        frames += 1;
        assert!(frames < 100);
    }
    assert_eq!(written, total);
    assert!(frames >= 2, "{frames} frames");
    h.step();
    assert!(h.state().0.drawn_points() > 0);
}

/// The GPU time of a frame of the 3D view (timestamp queries; without them the time
/// between frames) and the time of a UI step, while orbiting a
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
    // with GPU timestamps where the adapter has them, as the program asks for
    let setup = match default_wgpu_setup() {
        egui_wgpu::WgpuSetup::CreateNew(s) => {
            egui_wgpu::WgpuSetup::CreateNew(driftless_app::render::with_timestamps(s))
        }
        other => other,
    };
    let rs = create_render_state(setup, egui_wgpu::RendererOptions::PREDICTABLE);
    let r = rs.clone();
    let state = (View3d::default(), c, ViewSettings::default());
    let t = Instant::now();
    let mut h = Harness::builder()
        .with_size([1000.0, 800.0])
        .with_pixels_per_point(2.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(
            move |ui, (view, c, settings): &mut (View3d, LoadedCloud, ViewSettings)| {
                egui::CentralPanel::no_frame().show(ui, |ui| {
                    view.show(ui, &r, Some(c), None, settings);
                });
            },
            state,
        );
    // the upload in slices: the longest frame meanwhile
    let mut longest = t.elapsed().as_secs_f64();
    let mut frames = 1;
    while h.state().0.is_uploading() {
        let t = Instant::now();
        h.step();
        longest = longest.max(t.elapsed().as_secs_f64());
        frames += 1;
    }
    eprintln!(
        "upload over {frames} frames, longest frame {:.0} ms",
        longest * 1000.0
    );
    for budget in [6_000_000, 12_000_000] {
        h.state_mut().2.budget = budget;
        for _ in 0..3 {
            h.step();
        }
        for distance in [1.0f32, 0.3, 0.1] {
            let (view, ..) = h.state_mut();
            let full = view.camera().distance;
            view.camera_mut().distance = full * distance;
            let mut times = Vec::new();
            let mut steps = Vec::new();
            let mut drawn = 0;
            for _ in 0..60 {
                h.state_mut().0.camera_mut().orbit(6.0, 0.0);
                let t = Instant::now();
                h.step();
                steps.push(t.elapsed().as_secs_f32() * 1000.0);
                // paced like a 60 Hz display, so that the GPU times come back in between
                std::thread::sleep(std::time::Duration::from_millis(16));
                times.push(h.state().0.frame_ms());
                drawn = h.state().0.drawn_points();
            }
            steps.sort_by(f32::total_cmp);
            let gpu = h.state().0.gpu_timed();
            h.state_mut().0.camera_mut().distance = full;
            times.sort_by(f32::total_cmp);
            eprintln!(
                "budget {budget}, distance ×{distance}: {drawn} points drawn, frame (GPU: {gpu}) median {:.1} ms, 90 % {:.1} ms, max {:.1} ms; UI step median {:.1} ms",
                times[times.len() / 2],
                times[times.len() * 9 / 10],
                times[times.len() - 1],
                steps[steps.len() / 2]
            );
        }
    }
}

/// Without a cloud the view draws the run alone, framed on its trajectory; in the mode
/// „Stellen wählen“ a click on the trajectory picks the pose there – also with a height cut
/// that would hide points at its height (the cut is for points only).
#[test]
fn trajectory_alone_is_drawn_and_its_poses_clicked() {
    use driftless_app::overlay::RunOverlay;
    use driftless_app::pick::project;
    use driftless_app::ui::view3d::{ViewEvent, ViewOverlay};
    use driftless_slam::report::{BeforeAfter, SubmapReport};
    use driftless_slam::SlamReport;
    use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
    use driftless_traj::Pose;
    use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
    use egui_kittest::Harness;
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    }
    let none = BeforeAfter {
        before: 0.0,
        after: 0.0,
    };
    let poses: Vec<Pose> = (0..=20)
        .map(|i| Pose {
            t: 100.0 + f64::from(i),
            p: Vector3::new(500_000.0 + f64::from(i), 5_400_000.0, 100.0),
            q: UnitQuaternion::identity(),
        })
        .collect();
    let mut run = RunOverlay {
        run: PathBuf::from("/k/slam-001"),
        scan_start: 100.0,
        poses: poses.clone(),
        odometry: poses,
        origin: [0.0; 3],
        positions: Vec::new(),
        odometry_positions: Vec::new(),
        compare: Vec::new(),
        compare_positions: Vec::new(),
        compare_b: Vec::new(),
        compare_b_positions: Vec::new(),
        compare_legend: None,
        report: SlamReport {
            scan_start: 100.0,
            submaps: vec![SubmapReport {
                id: 0,
                t_start: 0.0,
                t_end: 20.0,
                quality: 1.0,
                anchor: [500_000.0, 5_400_000.0, 100.0],
                odom_anchor: [500_000.0, 5_400_000.0, 100.0],
            }],
            loops: Vec::new(),
            graph: None,
            start_end_gap: none.clone(),
            path_length: none,
        },
    };
    run.set_origin(None);
    let settings = ViewSettings {
        // a cut far below the trajectory: points there would be hidden
        cut: Some([-10.0, -5.0]),
        ..ViewSettings::default()
    };
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let r = rs.clone();
    type State = (View3d, RunOverlay, Option<ViewEvent>, ViewSettings);
    let mut h = Harness::builder()
        .with_size([300.0, 200.0])
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(
            move |ui, (view, run, event, settings): &mut State| {
                egui::CentralPanel::no_frame().show(ui, |ui| {
                    let overlay = ViewOverlay {
                        run,
                        version: 1,
                        places: [None; 2],
                        avoid_submap: None,
                        pick_places: true,
                    };
                    if let Some(e) = view.show(ui, &r, None, Some(overlay), settings) {
                        *event = Some(e);
                    }
                });
            },
            (View3d::default(), run, None, settings),
        );
    h.run();
    // pose 5 on the screen
    let (view, run, ..) = h.state();
    let vp = view.camera().view_proj(300.0 / 200.0);
    let p = project(&vp, run.positions[5], [300.0, 200.0]).unwrap();
    let pos = egui::pos2(p[0], p[1]);
    let image = h.render().unwrap();
    let c = image.get_pixel(p[0].round() as u32, p[1].round() as u32).0;
    assert!(c[1] > 200 && c[0] < 60, "green trajectory at pose 5: {c:?}");
    let button = |pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: egui::Modifiers::NONE,
    };
    let click = |h: &mut Harness<'_, State>| {
        h.event(egui::Event::PointerMoved(pos));
        h.step();
        h.event(button(true));
        h.step();
        h.event(button(false));
        h.step();
        h.state_mut().2.take()
    };
    assert_eq!(click(&mut h), Some(ViewEvent::PickedPose(5)));
    // with the trajectory hidden a click says so instead of picking anything
    h.state_mut().3.show_trajectory = false;
    h.step();
    assert_eq!(click(&mut h), Some(ViewEvent::TrajectoryHidden));
}

#[test]
fn show_mode_falls_back_to_what_is_there() {
    use ShowMode::*;
    for mode in [Surface, Cloud, Both] {
        assert_eq!(effective_show(mode, false, true), Cloud, "no surface");
        assert_eq!(effective_show(mode, true, false), Surface, "no cloud");
        assert_eq!(effective_show(mode, true, true), mode);
    }
    assert_eq!(ViewSettings::default().show, Surface, "the surface first");
    // one default for both
    assert_eq!(ShowMode::default(), ViewSettings::default().show);
}
