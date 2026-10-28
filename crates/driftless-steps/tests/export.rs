//! `run_export`: maps, models, surfaces, runs and single files into every format, the checks
//! before anything is written (kind, target = source, existing targets) and what a cancel or
//! an overwrite leaves behind. Scenes are synthetic: `synthetic_points`, a small `modell.json`
//! and meshes with tiny JPEG pages.

use driftless_export::mesh::{
    obj_files, read_mesh, write_glb, ImageKind, Mesh, MeshPage, PageImage,
};
use driftless_export::testutil::synthetic_points;
use driftless_export::{read_points, write_points, Layout, PointFormat, Points};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_steps::export::{
    existing_targets, export_targets, file_kind, run_export, ExportFormat, ExportKind,
    ExportReport, ExportRequest, ExportSource, MESH_EXTENSIONS, POINT_EXTENSIONS,
};
use driftless_steps::merge::{member_transform, ModelMember, ModelReport, MODEL_LAS, MODEL_REPORT};
use driftless_steps::slam::SlamSettings;
use driftless_steps::StepError;
use driftless_traj::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use driftless_traj::{write_tum, Pose};
use std::fs;
use std::path::{Path, PathBuf};

const POINT_FORMATS: [ExportFormat; 5] = [
    ExportFormat::Las,
    ExportFormat::Laz,
    ExportFormat::E57,
    ExportFormat::Ply,
    ExportFormat::Pcd,
];
const MESH_FORMATS: [ExportFormat; 3] =
    [ExportFormat::Glb, ExportFormat::Obj, ExportFormat::PlyMesh];

fn all_columns() -> Layout {
    Layout {
        rgb: true,
        intensity: true,
        source: true,
        time: true,
        flags: true,
    }
}

fn write_cloud(path: &Path, points: &Points) {
    let format = PointFormat::from_path(path).unwrap();
    write_points(path, format, points, &Progress::new()).unwrap();
}

/// A map folder: `map.json`, `clean.las` (400 points without colour) and, if `color`,
/// `color.las` (300 coloured points).
fn map_dir(name: &str, color: bool) -> PathBuf {
    let dir = tmp(name).join("karte");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join("map.json"), "{}").unwrap();
    write_cloud(
        &dir.join("clean.las"),
        &synthetic_points(400, 1, Layout::default()),
    );
    if color {
        let layout = Layout {
            rgb: true,
            intensity: true,
            ..Layout::default()
        };
        write_cloud(&dir.join("color.las"), &synthetic_points(300, 2, layout));
    }
    dir
}

fn pose(x: f64, yaw_deg: f64) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(x, -0.5, 0.25),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw_deg.to_radians()),
    )
}

fn member(scan: &str, pose: &Isometry3<f64>, included: bool) -> ModelMember {
    let h = pose.to_homogeneous();
    ModelMember {
        scan: PathBuf::from("/scans").join(scan),
        scan_rel: None,
        slam_dir: PathBuf::from("/scans").join(scan).join("driftless"),
        slam_dir_rel: None,
        map_dir: PathBuf::from("/scans").join(scan).join("driftless"),
        map_dir_rel: None,
        slam: SlamSettings::default(),
        transform: std::array::from_fn(|k| h[(k / 4, k % 4)]),
        placed_against: None,
        method: "bezug".into(),
        verdict: "sicher".into(),
        reason: None,
        overlap: 1.0,
        residual_median_mm: 0.0,
        residual_rms_mm: 0.0,
        agreement: 1.0,
        wall_residual_mm: 0.0,
        pair_rms_mm: None,
        included,
        color_gain: [1.0; 3],
        points: 0,
        clean_id: None,
    }
}

/// The members of [`model_dir`]: three in the model, a fourth left out.
fn model_members() -> Vec<ModelMember> {
    vec![
        member("Keller", &Isometry3::identity(), true),
        member("Flur Süd", &pose(4.0, 30.0), true),
        member("Dach", &pose(-2.0, -75.0), true),
        member("Garage", &pose(9.0, 10.0), false),
    ]
}

/// A model folder: `modell.las` (600 points of scans 0–2 with every column) and
/// `modell.json` with [`model_members`].
fn model_dir(name: &str) -> PathBuf {
    let dir = tmp(name).join("modell");
    fs::create_dir_all(&dir).unwrap();
    let mut points = synthetic_points(600, 3, all_columns());
    points.scans.clear();
    write_cloud(&dir.join(MODEL_LAS), &points);
    let report = ModelReport {
        version: 2,
        input_id: "test".into(),
        members: model_members(),
        points: 600,
        seconds_total: 0.0,
        peak_rss_bytes: 0,
        include_unsure: false,
        warnings: Vec::new(),
    };
    fs::write(
        dir.join(MODEL_REPORT),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    dir
}

/// A tiny JPEG of one colour.
fn jpeg(rgb: [u8; 3]) -> PageImage {
    let mut bytes = Vec::new();
    let pixels: Vec<u8> = (0..16).flat_map(|_| rgb).collect();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, 90)
        .encode(&pixels, 4, 4, image::ExtendedColorType::Rgb8)
        .unwrap();
    PageImage {
        kind: ImageKind::Jpeg,
        bytes: bytes.into(),
    }
}

/// A square of two triangles at height `z` with an image.
fn page(z: f64, rgb: [u8; 3]) -> MeshPage {
    MeshPage {
        positions: vec![[0.0, 0.0, z], [1.0, 0.0, z], [1.0, 1.0, z], [0.0, 1.0, z]],
        uvs: vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        indices: vec![[0, 1, 2], [0, 2, 3]],
        image: Some(jpeg(rgb)),
        color: [255; 3],
    }
}

fn mesh(pages: usize) -> Mesh {
    Mesh {
        pages: (0..pages)
            .map(|p| page(p as f64, [40 * p as u8, 100, 200]))
            .collect(),
    }
}

/// A surface folder: `oberflaeche.glb` with `pages` pages and `oberflaeche.json`.
fn surface_dir(name: &str, pages: usize) -> PathBuf {
    let dir = tmp(name).join("oberflaeche");
    fs::create_dir_all(&dir).unwrap();
    write_glb(&dir.join("oberflaeche.glb"), &mesh(pages), &Progress::new()).unwrap();
    fs::write(dir.join("oberflaeche.json"), "{}").unwrap();
    dir
}

fn poses() -> Vec<Pose> {
    (0..5)
        .map(|i| Pose {
            t: 1_700_000_000.0 + f64::from(i) * 0.1,
            p: Vector3::new(f64::from(i) * 0.25, -1.0 / 3.0, 1.5),
            q: UnitQuaternion::from_euler_angles(0.0, 0.0, f64::from(i) * 0.2),
        })
        .collect()
}

/// A run folder: `traj.tum` with [`poses`].
fn run_dir(name: &str) -> PathBuf {
    let dir = tmp(name).join("lauf");
    fs::create_dir_all(&dir).unwrap();
    write_tum(&dir.join("traj.tum"), &poses()).unwrap();
    dir
}

fn request(
    source: ExportSource,
    out: &Path,
    format: ExportFormat,
    overwrite: bool,
) -> ExportRequest {
    ExportRequest {
        source,
        out: out.to_path_buf(),
        format,
        overwrite,
    }
}

fn export(source: ExportSource, out: &Path, format: ExportFormat) -> ExportReport {
    run_export(&request(source, out, format, false), &Progress::new()).unwrap()
}

fn failed(source: ExportSource, out: &Path, format: ExportFormat, overwrite: bool) -> String {
    match run_export(&request(source, out, format, overwrite), &Progress::new()) {
        Err(StepError::Failed(m)) => m,
        other => panic!("expected a failure, got {other:?}"),
    }
}

/// Files in `dir` whose name ends in `.tmp`.
fn leftovers(dir: &Path) -> Vec<PathBuf> {
    fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.to_string_lossy().ends_with(".tmp"))
        .collect()
}

fn names(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

fn bytes_of(files: &[PathBuf]) -> u64 {
    files.iter().map(|f| fs::metadata(f).unwrap().len()).sum()
}

#[test]
fn map_exports_color_else_clean() {
    let dir = map_dir("export-map", true);
    let out = dir.parent().unwrap().join("karte.ply");
    let report = export(ExportSource::Map(dir.clone()), &out, ExportFormat::Ply);
    assert_eq!(report.count, 300);
    assert_eq!(report.files, vec![out.clone()]);
    assert_eq!(report.bytes, fs::metadata(&out).unwrap().len());
    let back = read_points(&out, &Progress::new()).unwrap();
    assert_eq!(back.len(), 300);
    assert!(back.rgb.is_some());

    fs::remove_file(dir.join("color.las")).unwrap();
    let out = dir.parent().unwrap().join("bereinigt.las");
    let report = export(ExportSource::Map(dir.clone()), &out, ExportFormat::Las);
    assert_eq!(report.count, 400);
    assert!(read_points(&out, &Progress::new()).unwrap().rgb.is_none());

    fs::remove_file(dir.join("clean.las")).unwrap();
    let out = dir.parent().unwrap().join("nichts.las");
    assert_eq!(
        failed(ExportSource::Map(dir), &out, ExportFormat::Las, false),
        "Karte ohne color.las oder clean.las"
    );
    assert!(!out.exists());
}

#[test]
fn model_e57_has_named_scans_with_poses() {
    let dir = model_dir("export-model");
    let out = dir.parent().unwrap().join("modell.e57");
    let report = export(ExportSource::Model(dir.clone()), &out, ExportFormat::E57);
    assert_eq!(report.count, 600);
    let back = read_points(&out, &Progress::new()).unwrap();
    assert_eq!(back.len(), 600);
    let members = model_members();
    let names: Vec<&str> = back.scans.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Keller", "Flur Süd", "Dach"]);
    for (scan, m) in back.scans.iter().zip(&members) {
        let (a, b) = (
            scan.pose.to_homogeneous(),
            member_transform(m).to_homogeneous(),
        );
        for (x, y) in a.iter().zip(b.iter()) {
            assert!((x - y).abs() < 1e-9, "{}: {a} vs {b}", scan.name);
        }
    }
    // The points are where the model has them, whatever pose their entry carries.
    let model = read_points(&dir.join(MODEL_LAS), &Progress::new()).unwrap();
    let mut want: Vec<[f64; 3]> = model.xyz.clone();
    let mut got: Vec<[f64; 3]> = back.xyz.clone();
    let key = |p: &[f64; 3]| p.map(|c| (c * 1e4).round() as i64);
    want.sort_by_key(key);
    got.sort_by_key(key);
    for (g, w) in got.iter().zip(&want) {
        for k in 0..3 {
            assert!((g[k] - w[k]).abs() < 1e-6, "{g:?} vs {w:?}");
        }
    }
    // modell.las directly (as the app passes it) is the same source.
    assert_eq!(
        ExportSource::detect(&dir.join(MODEL_LAS)).unwrap(),
        ExportSource::Model(dir)
    );
}

#[test]
fn detect_sources() {
    let map = map_dir("export-detect", false);
    let base = map.parent().unwrap().to_path_buf();
    assert_eq!(
        ExportSource::detect(&map).unwrap(),
        ExportSource::Map(map.clone())
    );
    let model = base.join("m");
    fs::create_dir_all(&model).unwrap();
    fs::write(model.join(MODEL_REPORT), "{}").unwrap();
    assert_eq!(
        ExportSource::detect(&model).unwrap(),
        ExportSource::Model(model.clone())
    );
    let surface = base.join("s");
    fs::create_dir_all(&surface).unwrap();
    fs::write(surface.join("oberflaeche.glb"), "").unwrap();
    assert_eq!(
        ExportSource::detect(&surface).unwrap(),
        ExportSource::Surface(surface.clone())
    );
    let run = base.join("r");
    fs::create_dir_all(&run).unwrap();
    fs::write(run.join("traj.tum"), "").unwrap();
    assert_eq!(
        ExportSource::detect(&run).unwrap(),
        ExportSource::Run(run.clone())
    );
    // A map folder is also the run folder of `driftless map`: the map wins, unless a
    // trajectory (or a surface) is asked for.
    fs::write(map.join("traj.tum"), "").unwrap();
    assert_eq!(
        ExportSource::detect(&map).unwrap(),
        ExportSource::Map(map.clone())
    );
    assert_eq!(
        ExportSource::detect_as(&map, ExportKind::Trajectory).unwrap(),
        ExportSource::Run(map.clone())
    );
    assert_eq!(
        ExportSource::detect_as(&map, ExportKind::Mesh).unwrap(),
        ExportSource::Map(map.clone()),
        "no surface there"
    );
    fs::write(map.join("oberflaeche.glb"), "").unwrap();
    assert_eq!(
        ExportSource::detect_as(&map, ExportKind::Mesh).unwrap(),
        ExportSource::Surface(map.clone())
    );
    assert_eq!(
        ExportSource::detect_as(&map, ExportKind::Points).unwrap(),
        ExportSource::Map(map.clone())
    );
    assert_eq!(
        ExportSource::detect(&map.join("traj.tum")).unwrap(),
        ExportSource::Run(map.clone())
    );
    for ext in POINT_EXTENSIONS {
        let f = base.join(format!("wolke.{}", ext.to_uppercase()));
        assert_eq!(
            ExportSource::detect(&f).unwrap(),
            ExportSource::PointFile(f.clone())
        );
    }
    for ext in MESH_EXTENSIONS {
        let f = base.join(format!("netz.{ext}"));
        assert_eq!(
            ExportSource::detect(&f).unwrap(),
            ExportSource::MeshFile(f.clone())
        );
    }
    // A modell.las without modell.json next to it is a point file.
    let las = base.join(MODEL_LAS);
    assert_eq!(
        ExportSource::detect(&las).unwrap(),
        ExportSource::PointFile(las.clone())
    );
    let other = base.join("notiz.txt");
    let empty = base.join("leer");
    fs::create_dir_all(&empty).unwrap();
    for p in [&other, &empty] {
        assert_eq!(
            ExportSource::detect(p).unwrap_err(),
            StepError::Failed(format!(
                "{}: weder Karte, Modell, Oberfläche, Lauf noch Punktwolken- oder Netzdatei",
                p.display()
            ))
        );
    }

    assert_eq!(ExportSource::Map(map.clone()).kind(), ExportKind::Points);
    assert_eq!(ExportSource::Model(model).kind(), ExportKind::Points);
    assert_eq!(ExportSource::PointFile(las).kind(), ExportKind::Points);
    assert_eq!(
        ExportSource::Surface(surface.clone()).kind(),
        ExportKind::Mesh
    );
    assert_eq!(ExportSource::MeshFile(other).kind(), ExportKind::Mesh);
    assert_eq!(
        ExportSource::Run(run.clone()).kind(),
        ExportKind::Trajectory
    );

    // Suggested names: the folder (the scan's for `<scan>/driftless`), the file stem.
    let scan_run = base.join("Büro 2").join("driftless");
    assert_eq!(ExportSource::Map(scan_run.clone()).default_name(), "Büro 2");
    assert_eq!(
        ExportSource::Run(scan_run).default_name(),
        "Büro 2_laufbahn"
    );
    assert_eq!(ExportSource::Map(map).default_name(), "karte");
    assert_eq!(
        ExportSource::Surface(surface).default_name(),
        "s_oberflaeche"
    );
    assert_eq!(
        ExportSource::PointFile(base.join("Halle A.e57")).default_name(),
        "Halle A"
    );
}

#[test]
fn format_from_extension_and_override() {
    let names: Vec<&str> = ExportFormat::ALL.iter().map(|f| f.name()).collect();
    assert_eq!(
        names,
        ["las", "laz", "e57", "ply", "pcd", "glb", "obj", "ply-netz", "tum", "csv"]
    );
    let labels: Vec<&str> = ExportFormat::ALL.iter().map(|f| f.label()).collect();
    assert_eq!(
        labels,
        [
            "LAS 1.4",
            "LAZ",
            "E57",
            "PLY (binär)",
            "PCD (binär)",
            "glb (Y oben)",
            "OBJ mit Seitenbildern",
            "PLY-Netz (Farbe je Ecke)",
            "TUM",
            "CSV"
        ]
    );
    for f in ExportFormat::ALL {
        assert_eq!(ExportFormat::parse(f.name()), Some(f));
        assert_eq!(ExportFormat::parse(&f.name().to_uppercase()), Some(f));
        let path = PathBuf::from(format!("x.{}", f.extension()));
        assert_eq!(ExportFormat::for_path(&path, f.kind()), Some(f), "{f:?}");
    }
    assert_eq!(ExportFormat::PlyMesh.extension(), "ply");
    assert_eq!(ExportFormat::parse("xyz"), None);
    assert_eq!(
        ExportFormat::for_path(Path::new("x.LAZ"), ExportKind::Points),
        Some(ExportFormat::Laz)
    );
    assert_eq!(
        ExportFormat::for_path(Path::new("x.xyz"), ExportKind::Points),
        None
    );
    assert_eq!(
        ExportFormat::for_path(Path::new("ohne"), ExportKind::Points),
        None
    );
    let kinds: Vec<ExportKind> = ExportFormat::ALL.iter().map(|f| f.kind()).collect();
    use ExportKind::{Mesh as M, Points as P, Trajectory as T};
    assert_eq!(kinds, [P, P, P, P, P, M, M, M, T, T]);

    // `format` decides, not the extension of the target.
    let dir = map_dir("export-override", true);
    let out = dir.parent().unwrap().join("wolke.dat");
    export(ExportSource::Map(dir), &out, ExportFormat::Laz);
    let bytes = fs::read(&out).unwrap();
    assert_eq!(&bytes[..4], b"LASF");
    // LAZ: the compressed point format has bit 7 set.
    assert_ne!(bytes[104] & 0x80, 0);
}

#[test]
fn ply_extension_by_kind() {
    let ply = Path::new("Halle.PLY");
    assert_eq!(
        ExportFormat::for_path(ply, ExportKind::Points),
        Some(ExportFormat::Ply)
    );
    assert_eq!(
        ExportFormat::for_path(ply, ExportKind::Mesh),
        Some(ExportFormat::PlyMesh)
    );
    assert_eq!(file_kind(ply), Some(ExportKind::Points));
    assert_eq!(file_kind(Path::new("a.Obj")), Some(ExportKind::Mesh));
    assert_eq!(file_kind(Path::new("a.glb")), Some(ExportKind::Mesh));
    assert_eq!(file_kind(Path::new("a.e57")), Some(ExportKind::Points));
    assert_eq!(file_kind(Path::new("a.tum")), None);
    assert_eq!(file_kind(Path::new("a")), None);

    // A surface to .ply is a mesh with colours per corner.
    let dir = surface_dir("export-ply-kind", 2);
    let out = dir.parent().unwrap().join("netz.ply");
    let report = export(ExportSource::Surface(dir), &out, ExportFormat::PlyMesh);
    assert_eq!(report.count, 4);
    let text = String::from_utf8_lossy(&fs::read(&out).unwrap()[..300]).into_owned();
    assert!(text.contains("element face 4"), "{text}");
}

#[test]
fn kind_mismatch_is_refused() {
    let map = map_dir("export-mismatch", true);
    let out = map.parent().unwrap().join("x.glb");
    assert_eq!(
        failed(
            ExportSource::Map(map.clone()),
            &out,
            ExportFormat::Glb,
            false
        ),
        "glb (Y oben) passt nicht zu Karte: möglich sind LAS 1.4, LAZ, E57, PLY (binär), \
         PCD (binär)"
    );
    let surface = surface_dir("export-mismatch-s", 1);
    assert_eq!(
        failed(
            ExportSource::Surface(surface),
            &out,
            ExportFormat::Las,
            false
        ),
        "LAS 1.4 passt nicht zu Oberfläche: möglich sind glb (Y oben), OBJ mit Seitenbildern, \
         PLY-Netz (Farbe je Ecke)"
    );
    let run = run_dir("export-mismatch-r");
    assert_eq!(
        failed(ExportSource::Run(run), &out, ExportFormat::Ply, false),
        "PLY (binär) passt nicht zu Lauf: möglich sind TUM, CSV"
    );
    assert!(!out.exists());
}

#[test]
fn existing_target_refused_without_overwrite() {
    let map = map_dir("export-existing", true);
    let out = map.parent().unwrap().join("da.las");
    fs::write(&out, b"alt").unwrap();
    let req = request(
        ExportSource::Map(map.clone()),
        &out,
        ExportFormat::Las,
        false,
    );
    assert_eq!(existing_targets(&req), vec![out.clone()]);
    assert_eq!(
        failed(ExportSource::Map(map), &out, ExportFormat::Las, false),
        format!("gibt es schon: {}", out.display())
    );
    assert_eq!(fs::read(&out).unwrap(), b"alt");
}

#[test]
fn overwrite_replaces() {
    let map = map_dir("export-overwrite", true);
    let out = map.parent().unwrap().join("da.pcd");
    fs::write(&out, b"alt").unwrap();
    let req = request(ExportSource::Map(map), &out, ExportFormat::Pcd, true);
    let report = run_export(&req, &Progress::new()).unwrap();
    assert_eq!(report.count, 300);
    assert_eq!(read_points(&out, &Progress::new()).unwrap().len(), 300);
    assert!(leftovers(out.parent().unwrap()).is_empty());
}

#[test]
fn target_is_source_refused() {
    let map = map_dir("export-self", true);
    let base = map.parent().unwrap().to_path_buf();
    let las = base.join("wolke.las");
    fs::copy(map.join("clean.las"), &las).unwrap();
    let before = fs::read(&las).unwrap();
    // Also spelled another way: through `.` and the folder's own name.
    let spelled = base
        .join(".")
        .join("..")
        .join(base.file_name().unwrap())
        .join("wolke.las");
    for out in [&las, &spelled] {
        for overwrite in [false, true] {
            assert_eq!(
                failed(
                    ExportSource::PointFile(las.clone()),
                    out,
                    ExportFormat::Las,
                    overwrite
                ),
                format!("Ziel ist die Quelle selbst: {}", out.display())
            );
        }
    }
    assert_eq!(fs::read(&las).unwrap(), before);
    let color = map.join("color.las");
    let before = fs::read(&color).unwrap();
    assert!(failed(
        ExportSource::Map(map.clone()),
        &color,
        ExportFormat::Las,
        true
    )
    .starts_with("Ziel ist die Quelle selbst"));
    assert_eq!(fs::read(&color).unwrap(), before);
    let model = model_dir("export-self-model");
    let out = model.join(MODEL_LAS);
    assert!(
        failed(ExportSource::Model(model), &out, ExportFormat::Laz, true)
            .starts_with("Ziel ist die Quelle selbst")
    );
    let surface = surface_dir("export-self-surface", 1);
    let glb = surface.join("oberflaeche.glb");
    assert!(failed(
        ExportSource::Surface(surface),
        &glb,
        ExportFormat::Glb,
        true
    )
    .starts_with("Ziel ist die Quelle selbst"));
    let run = run_dir("export-self-run");
    let tum = run.join("traj.tum");
    assert!(
        failed(ExportSource::Run(run), &tum, ExportFormat::Tum, true)
            .starts_with("Ziel ist die Quelle selbst")
    );
}

#[test]
fn obj_side_file_exists_refused() {
    let surface = surface_dir("export-obj-side", 2);
    let base = surface.parent().unwrap().to_path_buf();
    let out = base.join("x.obj");
    let page = base.join("x_seite_00.jpg");
    fs::write(&page, b"alt").unwrap();
    let req = request(
        ExportSource::Surface(surface.clone()),
        &out,
        ExportFormat::Obj,
        false,
    );
    assert_eq!(
        export_targets(&req),
        vec![out.clone(), base.join("x.mtl"), page.clone()]
    );
    assert_eq!(existing_targets(&req), vec![page.clone()]);
    assert_eq!(
        failed(
            ExportSource::Surface(surface),
            &out,
            ExportFormat::Obj,
            false
        ),
        format!("gibt es schon: {}", page.display())
    );
    assert_eq!(fs::read(&page).unwrap(), b"alt");
    assert!(!out.exists() && !base.join("x.mtl").exists());
}

#[test]
fn obj_overwrite_removes_stale_pages() {
    let surface = surface_dir("export-obj-stale", 2);
    let base = surface.parent().unwrap().to_path_buf();
    let out = base.join("x.obj");
    for p in 0..4 {
        fs::write(base.join(format!("x_seite_{p:02}.jpg")), b"alt").unwrap();
    }
    fs::write(base.join("x_seite_01.png"), b"alt").unwrap();
    // Not pages of x: kept.
    fs::write(base.join("xy_seite_00.jpg"), b"fremd").unwrap();
    fs::write(base.join("x_seite_notizen.txt"), b"fremd").unwrap();
    fs::write(base.join("x_seite_02.jpg.bak"), b"fremd").unwrap();
    let req = request(
        ExportSource::Surface(surface),
        &out,
        ExportFormat::Obj,
        true,
    );
    let report = run_export(&req, &Progress::new()).unwrap();
    let mesh = read_mesh(&out, &Progress::new()).unwrap();
    assert_eq!(mesh.pages.len(), 2);
    assert_eq!(report.files, obj_files(&out, &mesh));
    assert_eq!(report.bytes, bytes_of(&report.files));
    assert_eq!(report.count, 4);
    assert_eq!(
        names(&base),
        [
            "oberflaeche",
            "x.mtl",
            "x.obj",
            "x_seite_00.jpg",
            "x_seite_01.jpg",
            "x_seite_02.jpg.bak",
            "x_seite_notizen.txt",
            "xy_seite_00.jpg"
        ]
    );
}

/// A killed export leaves `<name>.tmp` next to its target; the next export to that target
/// removes them (for an OBJ also those of its `.mtl` and pages), others stay.
#[test]
fn stale_tmp_files_of_the_target_are_removed() {
    let surface = surface_dir("export-tmp", 2);
    let base = surface.parent().unwrap().to_path_buf();
    let own = [
        "netz.obj.tmp",
        "netz.mtl.tmp",
        "netz_seite_00.jpg.tmp",
        "netz_seite_07.png.tmp",
    ];
    let foreign = ["netzwerk.obj.tmp", "netz_seite_xy.jpg.tmp", "wolke.laz.tmp"];
    for f in own.iter().chain(&foreign) {
        fs::write(base.join(f), b"halb").unwrap();
    }
    export(
        ExportSource::Surface(surface),
        &base.join("netz.obj"),
        ExportFormat::Obj,
    );
    for f in own {
        assert!(!base.join(f).exists(), "{f} left");
    }
    for f in foreign {
        assert!(base.join(f).exists(), "{f} removed");
    }
    let map = map_dir("export-tmp-points", true);
    let out = map.parent().unwrap().join("wolke.laz");
    fs::write(map.parent().unwrap().join("wolke.laz.tmp"), b"halb").unwrap();
    export(ExportSource::Map(map), &out, ExportFormat::Laz);
    assert!(leftovers(out.parent().unwrap()).is_empty());
}

/// A target in a folder that is not there: refused in German before anything is read, not
/// with the system's English message from the writer.
#[test]
fn missing_target_folder_is_refused_in_german() {
    let map = map_dir("export-kein-ordner", true);
    let base = map.parent().unwrap().to_path_buf();
    let missing = base.join("gibt es nicht");
    let m = failed(
        ExportSource::Map(map.clone()),
        &missing.join("karte.laz"),
        ExportFormat::Laz,
        false,
    );
    assert_eq!(
        m,
        format!(
            "Zielordner {} gibt es nicht – einen vorhandenen Ordner wählen oder ihn erst anlegen",
            missing.display()
        )
    );
    assert!(!missing.exists());
    fs::write(base.join("datei"), b"").unwrap();
    let m = failed(
        ExportSource::Map(map),
        &base.join("datei").join("karte.laz"),
        ExportFormat::Laz,
        true,
    );
    assert!(m.contains("ist kein Ordner"), "{m}");
}

#[test]
fn cancel_keeps_old_files() {
    let surface = surface_dir("export-cancel", 2);
    let base = surface.parent().unwrap().to_path_buf();
    let out = base.join("alt.obj");
    // The old OBJ: of a surface with three pages.
    let old_surface = surface_dir("export-cancel-old", 3);
    export(ExportSource::Surface(old_surface), &out, ExportFormat::Obj);
    let before: Vec<(String, Vec<u8>)> = names(&base)
        .into_iter()
        .filter(|n| n.starts_with("alt"))
        .map(|n| {
            let bytes = fs::read(base.join(&n)).unwrap();
            (n, bytes)
        })
        .collect();
    assert_eq!(before.len(), 5);

    let progress = Progress::new();
    let watcher = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == driftless_export::PHASE_WRITE {
            watcher.cancel();
        }
    });
    let req = request(
        ExportSource::Surface(surface),
        &out,
        ExportFormat::Obj,
        true,
    );
    assert_eq!(run_export(&req, &progress), Err(StepError::Cancelled));
    let after: Vec<(String, Vec<u8>)> = names(&base)
        .into_iter()
        .filter(|n| n.starts_with("alt"))
        .map(|n| {
            let bytes = fs::read(base.join(&n)).unwrap();
            (n, bytes)
        })
        .collect();
    assert_eq!(after, before);
    assert!(leftovers(&base).is_empty());

    // The same for a point format.
    let map = map_dir("export-cancel-points", true);
    let out = map.parent().unwrap().join("alt.laz");
    fs::write(&out, b"alt").unwrap();
    let progress = Progress::new();
    let watcher = progress.clone();
    let progress = progress.with_observer(move |s| {
        if s.phase == driftless_export::PHASE_WRITE {
            watcher.cancel();
        }
    });
    let req = request(ExportSource::Map(map), &out, ExportFormat::Laz, true);
    assert_eq!(run_export(&req, &progress), Err(StepError::Cancelled));
    assert_eq!(fs::read(&out).unwrap(), b"alt");
    assert!(leftovers(out.parent().unwrap()).is_empty());
}

#[test]
fn run_exports_tum_and_csv() {
    let run = run_dir("export-run");
    let base = run.parent().unwrap().to_path_buf();
    let tum = base.join("bahn.tum");
    let csv = base.join("bahn.csv");
    let report = export(ExportSource::Run(run.clone()), &tum, ExportFormat::Tum);
    assert_eq!(report.count, 5);
    assert_eq!(
        fs::read(&tum).unwrap(),
        fs::read(run.join("traj.tum")).unwrap()
    );
    let report = export(ExportSource::Run(run), &csv, ExportFormat::Csv);
    assert_eq!(report.count, 5);
    assert_eq!(report.bytes, fs::metadata(&csv).unwrap().len());
    let csv_text = fs::read_to_string(&csv).unwrap();
    let tum_text = fs::read_to_string(&tum).unwrap();
    let csv_lines: Vec<&str> = csv_text.split('\n').collect();
    let tum_lines: Vec<&str> = tum_text.lines().collect();
    // For German spreadsheets: `;` and a decimal comma; the stamp as in TUM, then the seconds
    // since the first pose, then the TUM values.
    assert_eq!(csv_lines[0], "stempel;zeit_s;x;y;z;qx;qy;qz;qw");
    for (k, (csv, tum)) in csv_lines[1..6].iter().zip(&tum_lines[1..]).enumerate() {
        let tum: Vec<&str> = tum.split(' ').collect();
        let fields: Vec<&str> = csv.split(';').collect();
        assert_eq!(fields.len(), 9, "{csv}");
        assert!(!csv.contains('.'), "{csv}");
        assert_eq!(fields[0], tum[0].replace('.', ","));
        let t = |s: &str| s.parse::<f64>().unwrap();
        let since = t(&fields[1].replace(',', "."))
            - (t(tum[0]) - t(tum_lines[1].split(' ').next().unwrap()));
        assert!(since.abs() < 5e-4, "{k}: {csv}");
        assert_eq!(fields[1].split(',').nth(1).map(str::len), Some(3), "{csv}");
        for (c, v) in fields[2..].iter().zip(&tum[1..]) {
            assert_eq!(*c, v.replace('.', ","));
        }
    }
    assert!(csv_lines[1].starts_with("1700000000,000000000;0,000;0,000000000;-0,333333333;1,5"));
    assert_eq!(csv_lines.len(), 7, "five poses, a newline at the end");
    assert_eq!(csv_lines[6], "");
    assert!(!csv_text.contains('\r'));
}

/// A point file of `n` points with colour and intensity.
fn point_file(dir: &Path, n: usize) -> PathBuf {
    let path = dir.join("quelle.las");
    let layout = Layout {
        rgb: true,
        intensity: true,
        source: true,
        ..Layout::default()
    };
    write_cloud(&path, &synthetic_points(n, 7, layout));
    path
}

#[test]
fn empty_source_every_format() {
    let dir = tmp("export-empty");
    let src = point_file(&dir, 0);
    for f in POINT_FORMATS {
        let out = dir.join(format!("leer.{}", f.extension()));
        let report = export(ExportSource::PointFile(src.clone()), &out, f);
        assert_eq!(report.count, 0, "{f:?}");
        assert_eq!(
            read_points(&out, &Progress::new()).unwrap().len(),
            0,
            "{f:?}"
        );
    }
    let glb = dir.join("leer_netz.glb");
    write_glb(&glb, &Mesh::default(), &Progress::new()).unwrap();
    for f in MESH_FORMATS {
        let out = dir.join(format!("leer_aus_netz.{}", f.extension()));
        let report = export(ExportSource::MeshFile(glb.clone()), &out, f);
        assert_eq!(report.count, 0, "{f:?}");
        if f == ExportFormat::PlyMesh {
            let text = String::from_utf8_lossy(&fs::read(&out).unwrap()).into_owned();
            assert!(text.contains("element vertex 0") && text.contains("element face 0"));
        } else {
            assert_eq!(
                read_mesh(&out, &Progress::new()).unwrap().triangles(),
                0,
                "{f:?}"
            );
        }
    }
    assert!(leftovers(&dir).is_empty());
}

#[test]
fn umlaut_paths_every_format() {
    let dir = tmp("export-umlaut").join("Büro Scan");
    fs::create_dir_all(&dir).unwrap();
    let src = point_file(&dir, 2_000);
    for f in POINT_FORMATS {
        let out = dir.join(format!("Prüfung ä.{}", f.extension()));
        let report = export(ExportSource::PointFile(src.clone()), &out, f);
        assert_eq!(report.count, 2_000);
        let back = read_points(&out, &Progress::new()).unwrap();
        assert_eq!(back.len(), 2_000, "{f:?}");
        // And back again from the new file.
        let again = dir.join(format!("Prüfung ä zurück.{}", f.extension()));
        export(ExportSource::PointFile(out.clone()), &again, f);
        assert_eq!(read_points(&again, &Progress::new()).unwrap().len(), 2_000);
    }
    let glb = dir.join("Fläche ö.glb");
    write_glb(&glb, &mesh(2), &Progress::new()).unwrap();
    for f in MESH_FORMATS {
        let out = dir.join(format!("Prüfung ä.{}", f.extension()));
        let out = if f == ExportFormat::PlyMesh {
            dir.join("Prüfung ä netz.ply")
        } else {
            out
        };
        let report = export(ExportSource::MeshFile(glb.clone()), &out, f);
        assert_eq!(report.count, 4, "{f:?}");
        assert_eq!(report.bytes, bytes_of(&report.files));
        if f != ExportFormat::PlyMesh {
            let back = read_mesh(&out, &Progress::new()).unwrap();
            assert_eq!(back.triangles(), 4, "{f:?}");
            assert!(back.pages.iter().all(|p| p.image.is_some()), "{f:?}");
        }
    }
    let obj = dir.join("Prüfung ä.obj");
    let again = dir.join("Prüfung ä zurück.glb");
    export(ExportSource::MeshFile(obj), &again, ExportFormat::Glb);
    assert_eq!(read_mesh(&again, &Progress::new()).unwrap().triangles(), 4);
    let run = dir.join("Lauf ü");
    fs::create_dir_all(&run).unwrap();
    write_tum(&run.join("traj.tum"), &poses()).unwrap();
    for f in [ExportFormat::Tum, ExportFormat::Csv] {
        let out = dir.join(format!("Prüfung ä.{}", f.extension()));
        assert_eq!(export(ExportSource::Run(run.clone()), &out, f).count, 5);
    }
    let tum = dir.join("Prüfung ä.tum");
    assert_eq!(driftless_traj::read_tum(&tum).unwrap().len(), 5);
    assert!(leftovers(&dir).is_empty());
}
