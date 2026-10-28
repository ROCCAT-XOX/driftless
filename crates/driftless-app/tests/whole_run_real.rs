//! The whole program on the four real scans, as a user runs it in the window (Analyse A of
//! `docs/ergebnisse/2026-10-28-version-0-1.md` did it with the CLI): a new project with the
//! dark room S1–S3 and the Büro B from `$DRIFTLESS_TESTDATA`, „Alle rechnen“ (trajectory and
//! map with the defaults), the surfaces of all maps, the model of S1–S3 and its surface, and
//! the ten export formats. Prints times, memory and what came out; with `$DRIFTLESS_SHOTS`
//! also pictures of the window. The project and the exports go into `$DRIFTLESS_WORK` (about
//! 10 GB; nothing is written into the test data). Run with
//!
//! ```text
//! DRIFTLESS_TESTDATA=… DRIFTLESS_WORK=… DRIFTLESS_SHOTS=… cargo test -p driftless-app \
//!     --release --test whole_run_real -- --ignored --nocapture
//! ```

use driftless_app::project::MergePick;
use driftless_app::render::headless_device;
use driftless_app::ui::{Action, DriftlessApp, ScanNode, Selection, Step, UiMessage};
use driftless_steps::export::{ExportFormat, ExportKind, ExportRequest, ExportSource};
use driftless_steps::map::peak_rss_bytes;
use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
use egui_kittest::Harness;
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// S1 (the reference of the model), S2 and S3 of the dark room, then the Büro B.
const SCANS: [&str; 4] = [
    "jmk6-2026-10-22/20261022133554",
    "jmk6-2026-10-22/20261022132520",
    "jmk6-2026-10-22/20261022133028",
    "jmk6-2026-10-20/20261020173822",
];

const HOUR: Duration = Duration::from_secs(3600);

fn gb(bytes: u64) -> f64 {
    bytes as f64 / 1e9
}

fn peak_gb() -> f64 {
    peak_rss_bytes().map_or(f64::NAN, gb)
}

fn json(path: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(path).unwrap())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            _ => None,
        })
        .collect()
}

/// What a step does, with the scan it works on.
fn label(action: &Action) -> String {
    let scan = |p: &Path| {
        p.components()
            .map(|c| c.as_os_str().to_string_lossy().into_owned())
            .find(|c| c.starts_with("2026"))
            .unwrap_or_default()
    };
    match action {
        Action::RunSlam(id) => format!("Laufbahn {id}"),
        Action::RunMap(run) => format!("Karte {}", scan(run)),
        Action::RunSurface(map) => format!("Oberfläche {}", scan(map)),
        Action::RunMerge { .. } => "Modell zusammenführen".into(),
        Action::RunModelSurface { .. } => "Oberfläche Modell".into(),
        Action::RunExport(r) => format!("Export {}", r.format.name()),
        other => format!("{other:?}"),
    }
}

/// Ticks until no step, none queued and no background work is left; prints each step that
/// ran with its seconds.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + HOUR;
    let mut current: Option<(Action, Instant)> = None;
    loop {
        app.tick();
        let running = app.state().running.clone();
        if current.as_ref().map(|c| &c.0) != running.as_ref() {
            if let Some((a, t)) = current.take() {
                eprintln!(
                    "  {}: {:.1} s (Spitze bisher {:.2} GB)",
                    label(&a),
                    t.elapsed().as_secs_f64(),
                    peak_gb()
                );
            }
            current = running.map(|a| (a, Instant::now()));
        }
        if current.is_none() && app.state().queue.is_empty() && !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn scan<'a>(app: &'a DriftlessApp, id: &str) -> &'a ScanNode {
    app.state()
        .scans
        .iter()
        .find(|s| s.id == id)
        .unwrap_or_else(|| panic!("scan {id} not in the tree"))
}

fn id_of(rel: &str) -> &str {
    rel.rsplit('/').next().unwrap()
}

/// What came out for one scan: its trajectory, map and surface.
fn print_scan(node: &ScanNode) {
    let run = &node.runs[0];
    let slam = json(&run.dir.join("slam.json"));
    let loops = slam["loops"].as_array().unwrap();
    let accepted = loops.iter().filter(|l| l["accepted"] == true).count();
    eprintln!(
        "{}: Weg {:.2} m, Start↔Ende {:.1} cm, Schleifen {accepted} von {}",
        node.id,
        slam["path_length"]["after"].as_f64().unwrap(),
        slam["start_end_gap"]["after"].as_f64().unwrap() * 100.0,
        loops.len()
    );
    let map = &run.maps[0];
    let report = json(&map.dir.join("map.json"));
    eprintln!(
        "  {}: {:.1} s, Spitze {:.2} GB, Wolken {:?}",
        map.label,
        report["seconds_total"].as_f64().unwrap(),
        gb(report["peak_rss_bytes"].as_u64().unwrap()),
        map.clouds
            .iter()
            .map(|c| c.file_name().unwrap().to_string_lossy().into_owned())
            .collect::<Vec<_>>()
    );
    for surface in &map.surfaces {
        print_surface(&surface.dir, &surface.label, &surface.hints);
    }
}

fn print_surface(dir: &Path, label: &str, hints: &[String]) {
    let report = json(&dir.join("oberflaeche.json"));
    let stats = &report["stats"];
    eprintln!(
        "  {label}: {:.1} s, Spitze {:.2} GB, {} Dreiecke ({} ohne Foto), Genauigkeit Mittel {:.1} mm, \
         {} MB; Hinweise {hints:?}",
        report["seconds_total"].as_f64().unwrap(),
        gb(report["peak_rss_bytes"].as_u64().unwrap()),
        stats["faces_final"],
        stats["texture"]["faces_untextured"],
        stats["accuracy_mean_mm"].as_f64().unwrap_or(f64::NAN),
        fs::metadata(dir.join("oberflaeche.glb")).unwrap().len() / 1_000_000
    );
}

fn wait_cloud(h: &mut Harness<'_, DriftlessApp>) {
    let deadline = Instant::now() + Duration::from_secs(600);
    while h.state().is_loading() || h.state().cloud().is_none() || h.state().view().is_uploading() {
        h.step();
        assert!(Instant::now() < deadline, "cloud not loaded");
        std::thread::sleep(Duration::from_millis(20));
    }
    for _ in 0..3 {
        h.step();
    }
}

fn wait_surface(h: &mut Harness<'_, DriftlessApp>) {
    let deadline = Instant::now() + Duration::from_secs(600);
    while h.state().is_loading_surface() || h.state().view().is_uploading_surface() {
        h.step();
        assert!(Instant::now() < deadline, "surface not loaded");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn shot(h: &mut Harness<'_, DriftlessApp>, dir: &Path, name: &str) {
    h.step();
    let path = dir.join(format!("{name}.png"));
    h.render().expect("render").save(&path).expect("save png");
    eprintln!("{}", path.display());
}

/// The surface folder `surface` in „Bilder“, from above with the upper 40 % cut away.
fn show_surface(h: &mut Harness<'_, DriftlessApp>, surface: &Path) {
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.step();
    h.state_mut()
        .apply(Action::Select(Selection::Surface(surface.to_path_buf())));
    wait_surface(h);
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    let [lo, hi] = h.state().state().surface.as_ref().expect("surface").z_range;
    h.state_mut().state_mut().view.cut = Some([lo, lo + (hi - lo) * 0.6]);
    h.state_mut().apply(Action::FrameAll);
    h.state_mut().view_mut().camera_mut().pitch = -70f32.to_radians();
    for _ in 0..3 {
        h.step();
    }
}

/// Pictures of the computed project: the Büro cloud in „Daten“, the surfaces of the Büro
/// and of the model in „Bilder“.
fn pictures(project: &Path, buero: &str, dir: &Path) {
    if headless_device().is_none() {
        eprintln!("kein GPU-Adapter, keine Bilder");
        return;
    }
    fs::create_dir_all(dir).unwrap();
    let rs = create_render_state(
        default_wgpu_setup(),
        egui_wgpu::RendererOptions::PREDICTABLE,
    );
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::OpenProject(project.to_path_buf()));
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .with_pixels_per_point(1.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    let deadline = Instant::now() + Duration::from_secs(300);
    while h.state().wants_repaint_while_busy() {
        h.step();
        assert!(Instant::now() < deadline, "statuses not read");
        std::thread::sleep(Duration::from_millis(50));
    }
    let map = scan(h.state(), buero).runs[0].maps[0].clone();
    let cloud = map.clouds[0].clone();
    h.state_mut().apply(Action::Select(Selection::Cloud(cloud)));
    wait_cloud(&mut h);
    shot(&mut h, dir, "daten-buero");
    show_surface(&mut h, &map.surfaces[0].dir);
    shot(&mut h, dir, "bilder-buero");
    let model = h.state().state().models[0].surfaces[0].dir.clone();
    show_surface(&mut h, &model);
    shot(&mut h, dir, "bilder-modell");
}

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA and $DRIFTLESS_WORK, about 20 min and 10 GB"]
fn whole_run_on_real_scans() {
    let (Some(data), Some(work)) = (
        std::env::var_os("DRIFTLESS_TESTDATA").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_WORK").map(PathBuf::from),
    ) else {
        eprintln!("DRIFTLESS_TESTDATA oder DRIFTLESS_WORK nicht gesetzt, Test übersprungen");
        return;
    };
    if let Some(missing) = SCANS.iter().map(|s| data.join(s)).find(|s| !s.is_dir()) {
        eprintln!(
            "{} fehlt (siehe docs/testdaten.md), Test übersprungen",
            missing.display()
        );
        return;
    }
    let project = work.join("projekt");
    let out = work.join("export");
    assert!(
        !project.exists() && !out.exists(),
        "{} has a project or exports already",
        work.display()
    );
    let ids: Vec<&str> = SCANS.iter().map(|s| id_of(s)).collect();
    let start = Instant::now();
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    app.apply(Action::NewProject {
        dir: project.clone(),
        name: "Abnahme".into(),
    });
    settle(&mut app);
    for s in SCANS {
        app.apply(Action::AddScan(data.join(s)));
        settle(&mut app);
    }
    assert_eq!(errors(&app), Vec::<String>::new());

    eprintln!("Alle rechnen (Laufbahn und Karte):");
    let t = Instant::now();
    app.apply(Action::ComputeAllMaps);
    settle(&mut app);
    eprintln!("  zusammen {:.1} min", t.elapsed().as_secs_f64() / 60.0);
    assert_eq!(errors(&app), Vec::<String>::new());

    eprintln!("Alle Oberflächen:");
    let t = Instant::now();
    app.apply(Action::ComputeAllSurfaces);
    settle(&mut app);
    eprintln!("  zusammen {:.1} min", t.elapsed().as_secs_f64() / 60.0);
    assert_eq!(errors(&app), Vec::<String>::new());
    for id in &ids {
        let node = scan(&app, id);
        assert_eq!(node.runs.len(), 1, "{id}");
        let map = &node.runs[0].maps[0];
        assert!(map.dir.join("color.las").is_file(), "{id}");
        assert_eq!(map.surfaces.len(), 1, "{id}");
        assert!(
            map.surfaces[0].dir.join("oberflaeche.glb").is_file(),
            "{id}"
        );
    }

    eprintln!("Modell aus S1–S3:");
    let picks: Vec<MergePick> = ids[..3]
        .iter()
        .map(|id| {
            let run = &scan(&app, id).runs[0];
            MergePick {
                scan_id: id.to_string(),
                run: run.dir.clone(),
                map: run.maps[0].dir.clone(),
            }
        })
        .collect();
    app.apply(Action::RunMerge {
        picks,
        include_unsure: false,
    });
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    let model = app.state().models[0].dir.clone();
    app.apply(Action::RunModelSurface {
        model: model.clone(),
    });
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    let model_info = app.state().models[0].clone();
    assert_eq!(model_info.surfaces.len(), 1);
    let model_surface = model_info.surfaces[0].dir.clone();

    eprintln!("Export (Punkte aus dem Modell, Netz aus seiner Oberfläche, Laufbahn von S1):");
    fs::create_dir_all(&out).unwrap();
    let s1_run = scan(&app, ids[0]).runs[0].dir.clone();
    for format in ExportFormat::ALL {
        let source = match format.kind() {
            ExportKind::Points => ExportSource::Model(model.clone()),
            ExportKind::Mesh => ExportSource::Surface(model_surface.clone()),
            ExportKind::Trajectory => ExportSource::Run(s1_run.clone()),
        };
        let file = out.join(format!("abnahme-{}.{}", format.name(), format.extension()));
        app.apply(Action::RunExport(ExportRequest {
            source,
            out: file.clone(),
            format,
            overwrite: false,
        }));
        settle(&mut app);
        assert_eq!(errors(&app), Vec::<String>::new());
        let size = fs::metadata(&file).map_or(0, |m| m.len());
        assert!(size > 0, "{}", file.display());
    }
    let total = start.elapsed().as_secs_f64() / 60.0;

    eprintln!("\nErgebnisse:");
    for id in &ids {
        print_scan(scan(&app, id));
    }
    let r = &model_info.report;
    eprintln!(
        "{}: {} Punkte, {:.1} s, Spitze {:.2} GB",
        model_info.label,
        r.points,
        r.seconds_total,
        gb(r.peak_rss_bytes)
    );
    for m in &r.members {
        eprintln!(
            "  {}: {} ({}), Wände {:?} mm, Abstand Median {:.2} mm, Überlappung {:.0} %, {:?}",
            m.scan.file_name().unwrap().to_string_lossy(),
            m.verdict,
            m.method,
            m.wall_residual_mm.map(|w| (w * 100.0).round() / 100.0),
            m.residual_median_mm,
            m.overlap * 100.0,
            m.reason
        );
    }
    print_surface(
        &model_surface,
        "Oberfläche Modell",
        &model_info.surfaces[0].hints,
    );
    for m in &app.state().messages {
        match m {
            UiMessage::Warning(w) => eprintln!("Hinweis: {w}"),
            UiMessage::Info(i) if i.starts_with("Exportiert") => eprintln!("{i}"),
            _ => {}
        }
    }
    eprintln!(
        "Zusammen {total:.1} min, höchster Arbeitsspeicher {:.2} GB",
        peak_gb()
    );
    for m in &r.members[1..] {
        assert_eq!(m.verdict, "sicher", "{}", m.scan.display());
    }
    if let Some(dir) = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from) {
        drop(app);
        pictures(&project, ids[3], &dir);
    }
}
