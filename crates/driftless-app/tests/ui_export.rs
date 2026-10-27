//! „Exportieren …“: the context menu of the tree (run, map, model, surface), ☰ for what the
//! view shows, the window with Was · Format · „Ziel wählen …“, the question before replacing
//! side files, and the export as a job (also without a project).

use driftless_app::project::{
    MapInfo, ModelInfo, Project, RunInfo, ScanEntry, ScanState, SurfaceInfo,
};
use driftless_app::ui::export::{self, ExportDialog};
use driftless_app::ui::{
    rail, steps, text, Action, AppState, CloudSummary, DriftlessApp, ScanNode, Selection,
    SurfaceSummary, UiMessage,
};
use driftless_export::mesh::{write_glb, Mesh, MeshPage};
use driftless_io::testscan::tmp;
use driftless_steps::export::{
    ExportFormat, ExportKind, ExportReport, ExportRequest, ExportSource,
};
use driftless_steps::map::MapSwitches;
use driftless_steps::merge::{ModelReport, MODEL_LAS};
use driftless_steps::slam::SlamSettings;
use eframe::egui::{self, accesskit::Role};
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, (AppState, Option<Action>)>;

const RUN: &str = "/p/scans/halle/slam-001";
const MAP: &str = "/p/scans/halle/slam-001/karte-001";
const SURFACE: &str = "/p/scans/halle/slam-001/karte-001/oberflaeche-001";
const MODEL: &str = "/p/modelle/modell-001";

fn app() -> DriftlessApp {
    DriftlessApp::new(Arc::new(|| {}))
}

/// A harness that shows `show` with `state` and keeps the last action.
fn panel(state: AppState, show: fn(&mut egui::Ui, &mut AppState) -> Option<Action>) -> H {
    let mut h = Harness::builder().with_size([360.0, 900.0]).build_ui_state(
        move |ui, (state, action): &mut (AppState, Option<Action>)| {
            if let Some(a) = show(ui, state) {
                *action = Some(a);
            }
        },
        (state, None),
    );
    h.run();
    h
}

/// The window „Exportieren“ of `state`.
fn dialog(state: AppState) -> H {
    panel(state, |ui, state| export::show(ui.ctx(), state))
}

/// A project „Halle“ with one scan (selected, so its run unfolds): „SLAM 001“ → „Karte 001“
/// with `color.las` and „Oberfläche 001“; and „Modell 001“. Not read from disk.
fn tree_state() -> AppState {
    AppState {
        project: Some(Project {
            dir: PathBuf::from("/p"),
            name: "Halle".into(),
            scans: vec![ScanEntry {
                id: "halle".into(),
                path: PathBuf::from("/daten/halle"),
                name: "Halle".into(),
                extra_runs: Vec::new(),
            }],
        }),
        scans: vec![ScanNode {
            id: "halle".into(),
            name: "Halle".into(),
            state: ScanState::Ready,
            no_color: None,
            runs: vec![RunInfo {
                dir: RUN.into(),
                label: "SLAM 001".into(),
                settings: SlamSettings::default(),
                has_result: true,
                has_cache: true,
                has_previous: false,
                has_odometry: true,
                cloud: None,
                maps: vec![MapInfo {
                    dir: MAP.into(),
                    label: "Karte 001".into(),
                    settings: Some(MapSwitches::default()),
                    clouds: vec![PathBuf::from(MAP).join("color.las")],
                    surfaces: vec![SurfaceInfo {
                        dir: SURFACE.into(),
                        label: "Oberfläche 001".into(),
                        stale: false,
                        settings: None,
                    }],
                }],
                history: Vec::new(),
            }],
        }],
        models: vec![ModelInfo {
            dir: MODEL.into(),
            label: "Modell 001".into(),
            report: ModelReport {
                version: 1,
                input_id: "0".into(),
                members: Vec::new(),
                points: 0,
                seconds_total: 1.0,
                peak_rss_bytes: 0,
                include_unsure: false,
            },
            surfaces: Vec::new(),
        }],
        selection: Selection::Scan("halle".into()),
        ..AppState::default()
    }
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            UiMessage::Info(_) => None,
        })
        .collect()
}

/// Ticks like frames do until no step and no background work is left.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        app.tick();
        if !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A surface of two triangles, without a texture.
fn mesh() -> Mesh {
    Mesh {
        pages: vec![MeshPage {
            positions: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            uvs: Vec::new(),
            indices: vec![[0, 1, 2], [0, 2, 3]],
            image: None,
            color: [200, 10, 30],
        }],
    }
}

/// `dir/quelle/netz.glb` with [`mesh`].
fn glb_source(dir: &Path) -> PathBuf {
    let glb = dir.join("quelle").join("netz.glb");
    fs::create_dir_all(glb.parent().unwrap()).unwrap();
    write_glb(&glb, &mesh()).unwrap();
    glb
}

/// The dialog for one source, with `format`.
fn dialog_for(source: ExportSource, format: ExportFormat) -> ExportDialog {
    let mut d = ExportDialog::new(vec![(source, "Quelle".into())]).unwrap();
    d.format = format;
    d
}

#[test]
fn tree_context_menu_offers_export() {
    let cases = [
        ("SLAM 001", ExportSource::Run(RUN.into())),
        ("Karte 001", ExportSource::Map(MAP.into())),
        ("Oberfläche 001", ExportSource::Surface(SURFACE.into())),
        ("Modell 001", ExportSource::Model(MODEL.into())),
    ];
    let mut h = panel(tree_state(), steps::show);
    assert!(h.query_by_label(text::EXPORT).is_none());
    for (label, source) in cases {
        h.state_mut().1 = None;
        h.get_by_label(label).click_secondary();
        h.run();
        h.get_by_label(text::EXPORT).click();
        h.run();
        assert_eq!(h.state().1, Some(Action::OpenExport(source)), "{label}");
        // the menu closes after the choice
        assert!(h.query_by_label(text::EXPORT).is_none(), "{label}");
    }
    // a left click still selects
    h.state_mut().1 = None;
    h.get_by_label("Karte 001").click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::Select(Selection::Map(MAP.into())))
    );
    // the same in „Bilder“: the surface below its map
    let mut state = tree_state();
    state.step = driftless_app::ui::Step::Bilder;
    let mut h = panel(state, steps::show);
    h.get_by_label("Oberfläche 001").click_secondary();
    h.run();
    h.get_by_label(text::EXPORT).click();
    h.run();
    assert_eq!(
        h.state().1,
        Some(Action::OpenExport(ExportSource::Surface(SURFACE.into())))
    );
}

#[test]
fn open_export_from_tree_names_the_source() {
    let mut app = app();
    *app.state_mut() = tree_state();
    app.apply(Action::OpenExport(ExportSource::Map(MAP.into())));
    let d = app.state().export.clone().expect("dialog open");
    assert_eq!(
        d.choices,
        [(
            ExportSource::Map(MAP.into()),
            "Karte 001 – color.las".to_string()
        )]
    );
    assert_eq!(d.format, ExportFormat::Laz);
    let state = tree_state();
    let name = |s: ExportSource| export::source_name(&state, &s);
    assert_eq!(
        name(ExportSource::Surface(SURFACE.into())),
        "Oberfläche 001"
    );
    assert_eq!(
        name(ExportSource::Model(MODEL.into())),
        format!("Modell 001 – {MODEL_LAS}")
    );
    assert_eq!(name(ExportSource::Run(RUN.into())), "SLAM 001 – traj.tum");
    assert_eq!(
        name(ExportSource::PointFile(
            PathBuf::from(MAP).join("color.las")
        )),
        "Karte 001 – color.las"
    );
    assert_eq!(
        name(ExportSource::MeshFile("/x/Büro Netz.obj".into())),
        "Büro Netz.obj"
    );
}

#[test]
fn dialog_lists_formats_of_kind() {
    let cases = [
        (ExportSource::Map(MAP.into()), 5, ExportFormat::Laz),
        (ExportSource::Surface(SURFACE.into()), 3, ExportFormat::Glb),
        (ExportSource::Run(RUN.into()), 2, ExportFormat::Csv),
    ];
    for (source, n, default) in cases {
        let kind = source.kind();
        let d = ExportDialog::new(vec![(source.clone(), "Quelle".into())]).unwrap();
        assert_eq!(d.format, default, "{source:?}");
        let formats = d.formats();
        assert_eq!(formats.len(), n, "{source:?}");
        assert!(formats.iter().all(|f| f.kind() == kind));

        let state = AppState {
            export: Some(d),
            ..AppState::default()
        };
        let mut h = dialog(state);
        h.get_by_label(text::EXPORT_TITLE);
        h.get_by_label(text::EXPORT_WHAT);
        h.get_by_label(text::EXPORT_PICK);
        h.get_by_role_and_label(Role::ComboBox, text::EXPORT_FORMAT)
            .click();
        h.run();
        for f in ExportFormat::ALL {
            let shown = h.query_all_by_label(f.label()).count() > 0;
            assert_eq!(shown, f.kind() == kind, "{:?} for {source:?}", f);
        }
        // a choice changes the format
        let other = *formats.last().unwrap();
        h.get_all_by_label(other.label()).last().unwrap().click();
        h.run();
        assert_eq!(h.state().0.export.as_ref().unwrap().format, other);
    }
}

#[test]
fn dialog_cancel_closes_and_pick_asks_for_the_target() {
    let state = AppState {
        export: Some(dialog_for(ExportSource::Map(MAP.into()), ExportFormat::E57)),
        ..AppState::default()
    };
    let mut h = dialog(state);
    h.get_by_label(text::EXPORT_PICK).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::PickExportTarget));
    h.get_by_label(text::CANCEL).click();
    h.run();
    assert!(h.state().0.export.is_none());
    assert!(h.query_by_label(text::EXPORT_PICK).is_none());
}

#[test]
fn dialog_choice_of_another_kind_resets_the_format() {
    let dir = tmp("ui-export-wahl");
    let glb = glb_source(&dir);
    let mut d = ExportDialog::new(vec![
        (
            ExportSource::PointFile(dir.join("wolke.las")),
            "Wolke".into(),
        ),
        (ExportSource::MeshFile(glb), "Netz".into()),
    ])
    .unwrap();
    d.format = ExportFormat::E57;
    let mut h = dialog(AppState {
        export: Some(d),
        ..AppState::default()
    });
    h.get_by_role_and_label(Role::ComboBox, text::EXPORT_WHAT)
        .click();
    h.run();
    h.get_all_by_label("Netz").last().unwrap().click();
    h.run();
    let d = h.state().0.export.clone().unwrap();
    assert_eq!(d.chosen, 1);
    assert_eq!(d.format, ExportFormat::Glb);
}

fn cloud(path: &Path) -> CloudSummary {
    CloudSummary {
        path: path.to_path_buf(),
        points: 10,
        has_rgb: true,
        has_scans: false,
        quality_known: false,
        z_range: [0.0, 1.0],
        ceiling: None,
        point_m: 0.01,
    }
}

fn surface(path: &Path) -> SurfaceSummary {
    SurfaceSummary {
        path: path.to_path_buf(),
        triangles: 2,
        pages: 1,
        origin: [0.0; 3],
        z_range: [0.0, 1.0],
    }
}

#[test]
fn menu_export_needs_view() {
    let open_menu = |h: &mut H| {
        h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
            .click();
        h.run();
    };
    // nothing shown: locked
    let mut h = panel(AppState::default(), rail::show);
    open_menu(&mut h);
    assert!(h.get_by_label(text::EXPORT).accesskit_node().is_disabled());
    let mut app = app();
    app.apply(Action::OpenExportShown);
    assert!(app.state().export.is_none());

    // a cloud and a surface: both to choose from
    let dir = tmp("ui-export-ansicht");
    let las = dir.join("wolke.las");
    fs::write(&las, b"").unwrap();
    let surface_dir = dir.join("oberflaeche-001");
    fs::create_dir_all(&surface_dir).unwrap();
    let glb = surface_dir.join("oberflaeche.glb");
    write_glb(&glb, &mesh()).unwrap();
    let state = AppState {
        cloud: Some(cloud(&las)),
        surface: Some(surface(&glb)),
        ..AppState::default()
    };
    let mut h = panel(state.clone(), rail::show);
    open_menu(&mut h);
    assert!(!h.get_by_label(text::EXPORT).accesskit_node().is_disabled());
    h.get_by_label(text::EXPORT).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::OpenExportShown));
    let sources: Vec<ExportSource> = export::shown_sources(&state)
        .into_iter()
        .map(|(s, _)| s)
        .collect();
    assert_eq!(
        sources,
        [
            ExportSource::PointFile(las.clone()),
            ExportSource::Surface(surface_dir.clone())
        ]
    );
    let mut app = self::app();
    *app.state_mut() = state;
    app.apply(Action::OpenExportShown);
    let d = app.state().export.clone().expect("dialog open");
    assert_eq!(d.choices.len(), 2);
    assert_eq!(d.choices[0].1, "wolke.las");
    assert_eq!(d.format, ExportFormat::Laz);

    // `modell.las` is the model, so that the scans keep their names
    let model = dir.join("modell-001");
    fs::create_dir_all(&model).unwrap();
    fs::write(model.join("modell.json"), b"{}").unwrap();
    let state = AppState {
        cloud: Some(cloud(&model.join(MODEL_LAS))),
        ..AppState::default()
    };
    let sources: Vec<ExportSource> = export::shown_sources(&state)
        .into_iter()
        .map(|(s, _)| s)
        .collect();
    assert_eq!(sources, [ExportSource::Model(model)]);
}

#[test]
fn export_asks_before_replacing_side_files() {
    let dir = tmp("ui-export-ersetzen");
    let glb = glb_source(&dir);
    let target_dir = dir.join("ziel");
    fs::create_dir_all(&target_dir).unwrap();
    let obj = target_dir.join("x.obj");
    let mtl = target_dir.join("x.mtl");
    fs::write(&mtl, b"alt").unwrap();
    let source = ExportSource::MeshFile(glb.clone());
    let mut state = AppState {
        export: Some(dialog_for(source.clone(), ExportFormat::Obj)),
        ..AppState::default()
    };
    // the dialog confirmed `x.obj`; `x.mtl` is there as well: asked
    assert_eq!(export::target_picked(&mut state, obj.clone()), Ok(None));
    let question = state.export.as_ref().unwrap().replace.clone().unwrap();
    assert_eq!(question.files, std::slice::from_ref(&mtl));
    assert!(question.request.overwrite);
    let mut h = dialog(state);
    h.get_by_label(&text::export_replace_question(std::slice::from_ref(&mtl)));
    assert_eq!(
        text::export_replace_question(std::slice::from_ref(&mtl)),
        "Diese Dateien gibt es schon:\nx.mtl\nErsetzen?"
    );
    assert!(h.query_by_label(text::EXPORT_PICK).is_none());
    // „Abbrechen“: nothing starts, the window stays for another target
    h.get_by_label(text::CANCEL).click();
    h.run();
    assert_eq!(h.state().1, None);
    let d = h.state().0.export.clone().expect("still open");
    assert!(d.replace.is_none());
    assert_eq!(fs::read(&mtl).unwrap(), b"alt");
    h.get_by_label(text::EXPORT_PICK);

    // „Ersetzen“
    let mut state = h.state().0.clone();
    assert_eq!(export::target_picked(&mut state, obj.clone()), Ok(None));
    let mut h = dialog(state);
    h.get_by_label(text::EXPORT_REPLACE).click();
    h.run();
    let request = ExportRequest {
        source: source.clone(),
        out: obj.clone(),
        format: ExportFormat::Obj,
        overwrite: true,
    };
    assert_eq!(h.state().1, Some(Action::ConfirmExport(request.clone())));
    assert_eq!(fs::read(&mtl).unwrap(), b"alt", "not before the export");

    // carried out: the export runs with `overwrite`, the window closes
    let mut app = app();
    app.state_mut().export = h.state().0.export.clone();
    app.apply(Action::ConfirmExport(request.clone()));
    assert_eq!(errors(&app), Vec::<String>::new());
    assert_eq!(
        app.state().running,
        Some(Action::RunExport(request.clone()))
    );
    assert!(app.state().export.is_none());
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    assert!(obj.is_file());
    assert_ne!(fs::read(&mtl).unwrap(), b"alt");
    let last = app.state().messages.last().cloned();
    let Some(UiMessage::Info(info)) = last else {
        panic!("no message: {last:?}");
    };
    assert!(
        info.starts_with("Exportiert: x.obj (2 Dreiecke, "),
        "{info}"
    );
}

#[test]
fn picked_target_without_side_files_starts_at_once() {
    let dir = tmp("ui-export-ziel");
    let glb = glb_source(&dir);
    let source = ExportSource::MeshFile(glb.clone());
    let pick = |format: ExportFormat, out: PathBuf| {
        let mut state = AppState {
            export: Some(dialog_for(source.clone(), format)),
            ..AppState::default()
        };
        let r = export::target_picked(&mut state, out);
        (r, state)
    };
    let request = |out: PathBuf, format: ExportFormat, overwrite: bool| {
        Action::RunExport(ExportRequest {
            source: source.clone(),
            out,
            format,
            overwrite,
        })
    };
    // a new file
    let out = dir.join("neu.glb");
    let (r, _) = pick(ExportFormat::Glb, out.clone());
    assert_eq!(r, Ok(Some(request(out, ExportFormat::Glb, false))));
    // an existing one the dialog asked about itself
    let out = dir.join("da.glb");
    fs::write(&out, b"alt").unwrap();
    let (r, _) = pick(ExportFormat::Glb, out.clone());
    assert_eq!(r, Ok(Some(request(out, ExportFormat::Glb, true))));
    // without the format's extension it gets it, and that file was not asked about
    let (r, _) = pick(ExportFormat::PlyMesh, dir.join("ohne"));
    assert_eq!(
        r,
        Ok(Some(request(
            dir.join("ohne.ply"),
            ExportFormat::PlyMesh,
            false
        )))
    );
    fs::write(dir.join("da2.glb"), b"alt").unwrap();
    let (r, state) = pick(ExportFormat::Glb, dir.join("da2"));
    assert_eq!(r, Ok(None));
    let question = state.export.unwrap().replace.unwrap();
    assert_eq!(question.files, [dir.join("da2.glb")]);
    // the target is the source: refused, nothing asked
    let (r, state) = pick(ExportFormat::Glb, glb.clone());
    assert!(
        r.as_ref()
            .unwrap_err()
            .contains("Ziel ist die Quelle selbst"),
        "{r:?}"
    );
    assert!(state.export.unwrap().replace.is_none());
}

#[test]
fn exported_message() {
    let report = ExportReport {
        files: vec![PathBuf::from("/x/Büro Scan.laz")],
        count: 1_234_567,
        bytes: 12_345_678,
        seconds: 1.5,
    };
    assert_eq!(
        text::exported(&report, ExportKind::Points),
        "Exportiert: Büro Scan.laz (1.234.567 Punkte, 12,3 MB)"
    );
    assert_eq!(
        text::exported(&report, ExportKind::Mesh),
        "Exportiert: Büro Scan.laz (1.234.567 Dreiecke, 12,3 MB)"
    );
    assert_eq!(
        text::exported(&report, ExportKind::Trajectory),
        "Exportiert: Büro Scan.laz (1.234.567 Lagen, 12,3 MB)"
    );
}
