//! Export: a map, a model, a surface, a run or a single point cloud or mesh file into one
//! of the open formats of [`driftless_export`]. The command line (`driftless export`) and
//! the desktop program both call [`run_export`].
//!
//! Before anything is read, in this order: the format must fit the source
//! ([`ExportFormat::kind`]), the target must not be a file of the source (compared as the
//! file system resolves them), and without [`ExportRequest::overwrite`] none of the targets
//! ([`export_targets`]: for OBJ also the `.mtl` and every `<stem>_seite_*` there) may exist.
//! Every file is written through `<name>.tmp` and renamed at the end; a cancel or an error
//! leaves the old files as they were. An OBJ written over an older one removes the older
//! one's pages that it does not write itself.

use crate::error::{check_cancel, failed, StepError};
use crate::merge::{member_transform, read_model, MODEL_LAS, MODEL_REPORT};
use driftless_export::mesh::{self, obj_files, Mesh};
use driftless_export::traj::{write_traj, TrajFormat};
use driftless_export::{read_points, write_points, PointFormat, Points, ScanInfo, PHASE_WRITE};
use driftless_io::progress::Progress;
use serde::Serialize;
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Map folder: its report and the clouds an export takes (coloured first).
const MAP_REPORT: &str = "map.json";
const MAP_COLOR: &str = "color.las";
const MAP_CLEAN: &str = "clean.las";
/// Surface folder: its report and the mesh an export takes.
const SURFACE_GLB: &str = "oberflaeche.glb";
const SURFACE_REPORT: &str = "oberflaeche.json";
/// Run folder: the trajectory.
const RUN_TRAJ: &str = "traj.tum";

/// What is exported: points, a mesh or a trajectory.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportKind {
    Points,
    Mesh,
    Trajectory,
}

/// A format to export to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExportFormat {
    Las,
    Laz,
    E57,
    Ply,
    Pcd,
    Glb,
    Obj,
    PlyMesh,
    Tum,
    Csv,
}

impl ExportFormat {
    pub const ALL: [ExportFormat; 10] = [
        ExportFormat::Las,
        ExportFormat::Laz,
        ExportFormat::E57,
        ExportFormat::Ply,
        ExportFormat::Pcd,
        ExportFormat::Glb,
        ExportFormat::Obj,
        ExportFormat::PlyMesh,
        ExportFormat::Tum,
        ExportFormat::Csv,
    ];

    pub fn kind(self) -> ExportKind {
        match self {
            ExportFormat::Las
            | ExportFormat::Laz
            | ExportFormat::E57
            | ExportFormat::Ply
            | ExportFormat::Pcd => ExportKind::Points,
            ExportFormat::Glb | ExportFormat::Obj | ExportFormat::PlyMesh => ExportKind::Mesh,
            ExportFormat::Tum | ExportFormat::Csv => ExportKind::Trajectory,
        }
    }

    /// The name for `--format`.
    pub fn name(self) -> &'static str {
        match self {
            ExportFormat::Las => "las",
            ExportFormat::Laz => "laz",
            ExportFormat::E57 => "e57",
            ExportFormat::Ply => "ply",
            ExportFormat::Pcd => "pcd",
            ExportFormat::Glb => "glb",
            ExportFormat::Obj => "obj",
            ExportFormat::PlyMesh => "ply-netz",
            ExportFormat::Tum => "tum",
            ExportFormat::Csv => "csv",
        }
    }

    /// The name in the desktop program.
    pub fn label(self) -> &'static str {
        match self {
            ExportFormat::Las => "LAS 1.4",
            ExportFormat::Laz => "LAZ",
            ExportFormat::E57 => "E57",
            ExportFormat::Ply => "PLY (binär)",
            ExportFormat::Pcd => "PCD (binär)",
            ExportFormat::Glb => "glb (Y oben)",
            ExportFormat::Obj => "OBJ mit Seitenbildern",
            ExportFormat::PlyMesh => "PLY-Netz (Farbe je Ecke)",
            ExportFormat::Tum => "TUM",
            ExportFormat::Csv => "CSV",
        }
    }

    /// The usual extension, without the dot.
    pub fn extension(self) -> &'static str {
        match self {
            ExportFormat::PlyMesh => "ply",
            f => f.name(),
        }
    }

    /// The format of a `--format` name (any case).
    pub fn parse(name: &str) -> Option<Self> {
        let name = name.to_ascii_lowercase();
        Self::ALL.into_iter().find(|f| f.name() == name)
    }

    /// The format of a target's extension (any case); `.ply` is [`ExportFormat::PlyMesh`]
    /// for a mesh and [`ExportFormat::Ply`] otherwise.
    pub fn for_path(path: &Path, kind: ExportKind) -> Option<Self> {
        let ext = path.extension()?.to_str()?.to_ascii_lowercase();
        if ext == "ply" {
            return Some(if kind == ExportKind::Mesh {
                ExportFormat::PlyMesh
            } else {
                ExportFormat::Ply
            });
        }
        Self::ALL
            .into_iter()
            .find(|f| *f != ExportFormat::PlyMesh && f.extension() == ext)
    }

    /// The formats of `kind`, by label, for messages.
    fn labels_of(kind: ExportKind) -> String {
        Self::ALL
            .iter()
            .filter(|f| f.kind() == kind)
            .map(|f| f.label())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// What an export reads.
#[derive(Clone, Debug, PartialEq)]
pub enum ExportSource {
    /// A map folder (`map.json`): `color.las`, else `clean.las`.
    Map(PathBuf),
    /// A model folder (`modell.json`): `modell.las` with one scan per member in the model.
    Model(PathBuf),
    /// A surface folder: `oberflaeche.glb`.
    Surface(PathBuf),
    /// A run folder: `traj.tum`.
    Run(PathBuf),
    /// A point cloud file ([`POINT_EXTENSIONS`]).
    PointFile(PathBuf),
    /// A mesh file ([`MESH_EXTENSIONS`]).
    MeshFile(PathBuf),
}

/// Extensions of the point cloud files an export (and the desktop program) reads.
pub const POINT_EXTENSIONS: [&str; 5] = ["las", "laz", "e57", "ply", "pcd"];
/// Extensions of the mesh files an export reads.
pub const MESH_EXTENSIONS: [&str; 2] = ["glb", "obj"];

/// What a file is by its extension (any case); `.ply` counts as points.
pub fn file_kind(path: &Path) -> Option<ExportKind> {
    let ext = path.extension()?.to_str()?.to_ascii_lowercase();
    if POINT_EXTENSIONS.contains(&ext.as_str()) {
        Some(ExportKind::Points)
    } else if MESH_EXTENSIONS.contains(&ext.as_str()) {
        Some(ExportKind::Mesh)
    } else {
        None
    }
}

/// The name of a folder for suggestions: its own, or for a run folder `<scan>/driftless`
/// the scan's.
fn folder_name(dir: &Path) -> String {
    let own = |p: &Path| p.file_name().map(|n| n.to_string_lossy().into_owned());
    match own(dir) {
        Some(n) if n.eq_ignore_ascii_case("driftless") => dir.parent().and_then(own).unwrap_or(n),
        Some(n) => n,
        None => "export".into(),
    }
}

impl ExportSource {
    /// The source at `path`:
    /// - a folder with `modell.json` is a model, with `map.json` a map, with
    ///   `oberflaeche.glb` a surface, with `traj.tum` a run (the first that fits);
    /// - `modell.las` next to a `modell.json` is that model;
    /// - a file with an extension of [`POINT_EXTENSIONS`] or [`MESH_EXTENSIONS`] is a point
    ///   or mesh file; `traj.tum` is its run folder.
    pub fn detect(path: &Path) -> Result<Self, StepError> {
        if path.is_dir() {
            if path.join(MODEL_REPORT).is_file() {
                return Ok(ExportSource::Model(path.to_path_buf()));
            }
            if path.join(MAP_REPORT).is_file() {
                return Ok(ExportSource::Map(path.to_path_buf()));
            }
            if path.join(SURFACE_GLB).is_file() {
                return Ok(ExportSource::Surface(path.to_path_buf()));
            }
            if path.join(RUN_TRAJ).is_file() {
                return Ok(ExportSource::Run(path.to_path_buf()));
            }
        } else {
            let dir = path.parent().unwrap_or(Path::new(""));
            if path.file_name().is_some_and(|n| n == MODEL_LAS) && dir.join(MODEL_REPORT).is_file()
            {
                return Ok(ExportSource::Model(dir.to_path_buf()));
            }
            if path.file_name().is_some_and(|n| n == RUN_TRAJ) && path.is_file() {
                return Ok(ExportSource::Run(dir.to_path_buf()));
            }
            match file_kind(path) {
                Some(ExportKind::Points) => return Ok(ExportSource::PointFile(path.to_path_buf())),
                Some(ExportKind::Mesh) => return Ok(ExportSource::MeshFile(path.to_path_buf())),
                _ => {}
            }
        }
        Err(format!(
            "{}: weder Karte, Modell, Oberfläche, Lauf noch Punktwolken- oder Netzdatei",
            path.display()
        )
        .into())
    }

    /// The source at `path` ([`Self::detect`]), but for `kind` if the folder holds that as
    /// well: a run folder of `driftless slam` is often also the map folder, a map folder may
    /// hold the surface. Falls back to [`Self::detect`].
    pub fn detect_as(path: &Path, kind: ExportKind) -> Result<Self, StepError> {
        let found = Self::detect(path)?;
        if found.kind() == kind || !path.is_dir() {
            return Ok(found);
        }
        let dir = path.to_path_buf();
        Ok(match kind {
            ExportKind::Trajectory if path.join(RUN_TRAJ).is_file() => ExportSource::Run(dir),
            ExportKind::Mesh if path.join(SURFACE_GLB).is_file() => ExportSource::Surface(dir),
            ExportKind::Points if path.join(MAP_REPORT).is_file() => ExportSource::Map(dir),
            _ => found,
        })
    }

    pub fn kind(&self) -> ExportKind {
        match self {
            ExportSource::Map(_) | ExportSource::Model(_) | ExportSource::PointFile(_) => {
                ExportKind::Points
            }
            ExportSource::Surface(_) | ExportSource::MeshFile(_) => ExportKind::Mesh,
            ExportSource::Run(_) => ExportKind::Trajectory,
        }
    }

    /// A name for the file dialog, without extension: the folder (the scan's for
    /// `<scan>/driftless`) with `_oberflaeche` or `_laufbahn` for a surface or a run, the
    /// file stem for a file.
    pub fn default_name(&self) -> String {
        match self {
            ExportSource::Map(d) | ExportSource::Model(d) => folder_name(d),
            ExportSource::Surface(d) => format!("{}_oberflaeche", folder_name(d)),
            ExportSource::Run(d) => format!("{}_laufbahn", folder_name(d)),
            ExportSource::PointFile(f) | ExportSource::MeshFile(f) => f
                .file_stem()
                .map_or_else(|| "export".into(), |s| s.to_string_lossy().into_owned()),
        }
    }

    /// What it is called in messages.
    fn noun(&self) -> &'static str {
        match self {
            ExportSource::Map(_) => "Karte",
            ExportSource::Model(_) => "Modell",
            ExportSource::Surface(_) => "Oberfläche",
            ExportSource::Run(_) => "Lauf",
            ExportSource::PointFile(_) => "Punktwolke",
            ExportSource::MeshFile(_) => "Netz",
        }
    }

    /// The files of the source a target must never be: what is read and the report of the
    /// folder; for an OBJ also its `.mtl`.
    fn files(&self) -> Vec<PathBuf> {
        match self {
            ExportSource::Map(d) => vec![d.join(MAP_COLOR), d.join(MAP_CLEAN), d.join(MAP_REPORT)],
            ExportSource::Model(d) => vec![d.join(MODEL_LAS), d.join(MODEL_REPORT)],
            ExportSource::Surface(d) => vec![d.join(SURFACE_GLB), d.join(SURFACE_REPORT)],
            ExportSource::Run(d) => vec![d.join(RUN_TRAJ)],
            ExportSource::PointFile(f) => vec![f.clone()],
            ExportSource::MeshFile(f) => vec![f.clone(), f.with_extension("mtl")],
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExportRequest {
    pub source: ExportSource,
    /// The target file (for OBJ the `.obj`; the `.mtl` and the pages go next to it).
    pub out: PathBuf,
    pub format: ExportFormat,
    /// Replace targets that exist already.
    pub overwrite: bool,
}

/// What an export wrote.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ExportReport {
    /// Every file written, the main one first.
    pub files: Vec<PathBuf>,
    /// Points, triangles or poses.
    pub count: u64,
    /// Sum of the sizes of [`Self::files`].
    pub bytes: u64,
    pub seconds: f64,
}

/// Whether `name` is a page of the OBJ with file stem `stem`: `<stem>_seite_…`.
fn is_page_of(name: &std::ffi::OsStr, stem: &std::ffi::OsStr) -> bool {
    let mut prefix: OsString = stem.to_os_string();
    prefix.push("_seite_");
    name.as_encoded_bytes()
        .starts_with(prefix.as_encoded_bytes())
}

/// The pages `<stem>_seite_*` next to the OBJ `out` that exist now, sorted.
fn existing_pages(out: &Path) -> Vec<PathBuf> {
    let (Some(stem), dir) = (out.file_stem(), out.parent().unwrap_or(Path::new(""))) else {
        return Vec::new();
    };
    let dir = if dir.as_os_str().is_empty() {
        Path::new(".")
    } else {
        dir
    };
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut pages: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .filter(|e| is_page_of(&e.file_name(), stem) && e.path().is_file())
        .map(|e| out.with_file_name(e.file_name()))
        .collect();
    pages.sort();
    pages
}

/// The files an export may write or replace: the target; for OBJ also its `.mtl` and every
/// `<stem>_seite_*` there now (a new page, or an old one the export removes).
pub fn export_targets(req: &ExportRequest) -> Vec<PathBuf> {
    let mut files = vec![req.out.clone()];
    if req.format == ExportFormat::Obj {
        files.push(req.out.with_extension("mtl"));
        files.extend(existing_pages(&req.out));
    }
    files
}

/// The [`export_targets`] that exist (asked about before replacing them).
pub fn existing_targets(req: &ExportRequest) -> Vec<PathBuf> {
    export_targets(req)
        .into_iter()
        .filter(|p| fs::symlink_metadata(p).is_ok())
        .collect()
}

/// The checks before anything is read: the format fits the source and no target is a file
/// of the source. The existing targets are not checked here ([`existing_targets`]).
pub fn check_export(req: &ExportRequest) -> Result<(), StepError> {
    let kind = req.source.kind();
    if req.format.kind() != kind {
        return Err(format!(
            "{} passt nicht zu {}: möglich sind {}",
            req.format.label(),
            req.source.noun(),
            ExportFormat::labels_of(kind)
        )
        .into());
    }
    let sources: Vec<PathBuf> = req
        .source
        .files()
        .iter()
        .filter_map(|f| fs::canonicalize(f).ok())
        .collect();
    for target in export_targets(req) {
        // A target that does not exist cannot be a file of the source.
        if let Ok(t) = fs::canonicalize(&target) {
            if sources.contains(&t) {
                return Err(format!("Ziel ist die Quelle selbst: {}", target.display()).into());
            }
        }
    }
    Ok(())
}

/// What was read.
enum Data {
    Points(Points),
    Mesh(Mesh),
    Poses(Vec<driftless_traj::Pose>),
}

fn read_source(source: &ExportSource, progress: &Progress) -> Result<Data, StepError> {
    Ok(match source {
        ExportSource::Map(d) => {
            let file = [MAP_COLOR, MAP_CLEAN]
                .into_iter()
                .map(|f| d.join(f))
                .find(|p| p.is_file())
                .ok_or("Karte ohne color.las oder clean.las")?;
            Data::Points(read_points(&file, progress)?)
        }
        ExportSource::Model(d) => {
            let report = read_model(d)?;
            // The scan column stays even when every point is of one scan: it says which.
            let mut points = driftless_export::las::read_las(&d.join(MODEL_LAS), progress)?;
            let source = points.source.take();
            points.drop_empty_columns();
            points.source = source;
            points.scans = report
                .members
                .iter()
                .enumerate()
                .filter(|(_, m)| m.included)
                .map(|(i, m)| {
                    Ok(ScanInfo {
                        source: u16::try_from(i)
                            .map_err(|_| failed("Modell mit mehr als 65536 Scans"))?,
                        name: m.scan.file_name().map_or_else(
                            || m.scan.display().to_string(),
                            |n| n.to_string_lossy().into_owned(),
                        ),
                        pose: member_transform(m),
                    })
                })
                .collect::<Result<_, StepError>>()?;
            Data::Points(points)
        }
        ExportSource::PointFile(f) => Data::Points(read_points(f, progress)?),
        ExportSource::Surface(d) => Data::Mesh(mesh::read_mesh(&d.join(SURFACE_GLB), progress)?),
        ExportSource::MeshFile(f) => Data::Mesh(mesh::read_mesh(f, progress)?),
        ExportSource::Run(d) => {
            let path = d.join(RUN_TRAJ);
            Data::Poses(driftless_traj::read_tum(&path).map_err(failed)?)
        }
    })
}

fn point_format(f: ExportFormat) -> PointFormat {
    match f {
        ExportFormat::Laz => PointFormat::Laz,
        ExportFormat::E57 => PointFormat::E57,
        ExportFormat::Ply => PointFormat::Ply,
        ExportFormat::Pcd => PointFormat::Pcd,
        _ => PointFormat::Las,
    }
}

/// Exports as `req` says (see the module docs); every message goes to `progress`. Fails with
/// a German message for a format that does not fit the source, a target that is a file of
/// the source, an existing target without [`ExportRequest::overwrite`] and a source that
/// cannot be read.
pub fn run_export(req: &ExportRequest, progress: &Progress) -> Result<ExportReport, StepError> {
    let start = Instant::now();
    check_cancel(progress)?;
    check_export(req)?;
    if !req.overwrite {
        let existing = existing_targets(req);
        if !existing.is_empty() {
            let list: Vec<String> = existing.iter().map(|p| p.display().to_string()).collect();
            return Err(format!("gibt es schon: {}", list.join(", ")).into());
        }
    }

    let data = read_source(&req.source, progress)?;
    // Last chance to stop before a file is touched.
    progress.phase(PHASE_WRITE, None);
    check_cancel(progress)?;
    let out = &req.out;
    let (files, count) = match (&data, req.format) {
        (Data::Points(points), f) => {
            write_points(out, point_format(f), points, progress)?;
            (vec![out.clone()], points.len() as u64)
        }
        (Data::Mesh(m), ExportFormat::Glb) => {
            mesh::write_glb(out, m)?;
            (vec![out.clone()], m.triangles() as u64)
        }
        (Data::Mesh(m), ExportFormat::Obj) => {
            let old_pages = existing_pages(out);
            mesh::write_obj(out, m)?;
            let files = obj_files(out, m);
            for stale in old_pages.iter().filter(|p| !files.contains(p)) {
                fs::remove_file(stale)
                    .map_err(|e| failed(format!("{} nicht entfernbar: {e}", stale.display())))?;
            }
            (files, m.triangles() as u64)
        }
        (Data::Mesh(m), _) => {
            mesh::write_ply_mesh(out, m, progress)?;
            (vec![out.clone()], m.triangles() as u64)
        }
        (Data::Poses(poses), f) => {
            let format = if f == ExportFormat::Csv {
                TrajFormat::Csv
            } else {
                TrajFormat::Tum
            };
            write_traj(out, poses, format)?;
            (vec![out.clone()], poses.len() as u64)
        }
    };
    let bytes = files
        .iter()
        .map(|f| fs::metadata(f).map_or(0, |m| m.len()))
        .sum();
    Ok(ExportReport {
        files,
        count,
        bytes,
        seconds: start.elapsed().as_secs_f64(),
    })
}
