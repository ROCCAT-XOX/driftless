//! The touch-up of a surface („Nacharbeit“): the steps of the Stellen painted on its map or
//! model (`stellen.toml`, [`crate::marks`]) replayed on the computed surface ([`run_touchup`]),
//! kept as patches in [`TOUCHUP_DIR`] in the surface folder, and the touched-up surface made
//! from them: [`edited_mesh`] for the export, [`edited_pages`] for the view.
//!
//! **Replay.** The steps of all Stellen run in the order they were done (`seq`; at a tie in the
//! order of the file), each on its Stelle as it was then and on the surface as the steps before
//! it left it:
//! - the Stelle is the faces the dabs painted before the step cover ([`select_faces`]); dabs
//!   painted later count for later steps only;
//! - „Glätten“ moves the vertices of the Stelle ([`smooth_region`]); then its images are baked
//!   anew with variant 0 ([`retexture_region`]), so that they sit on the smoothed surface.
//!   Without the camera images ([`TextureSources::Unavailable`]: the raw data gone, or no
//!   longer those the surface was made from) the Stelle keeps its texture coordinates (the
//!   texture moves with the vertices), and a hint says why;
//! - „Bilder neu einspielen“ bakes the images of the Stelle anew with the step's variant;
//!   without the camera images it is skipped, with the same hint.
//!
//! Each step gives one [`SurfacePatch`]: a smoothing and the images baked after it are one. A
//! step that changes nothing is refused ([`TouchupDone::refused`], the reason as the error says
//! it): „ganz eben“ on a Stelle that is not flat enough, no image that shows the Stelle, nothing
//! around it to match the brightness to, a Stelle that does not lie on this surface. The other
//! steps run on; `stellen.toml` is only read here, taking a refused step out is the program's
//! part.
//!
//! **Cache.** [`TOUCHUP_DIR`] holds the patches ([`PATCHES_FILE`]) and [`TOUCHUP_REPORT`] with
//! the key ([`touchup_fits`]): the input id of the surface's report and the size of its glb (the
//! patches belong to exactly this glb), [`EDIT_VERSION`], and per step a key of the steps up to
//! it – each Stelle as its step sees it: its number, the dabs painted before the step and the
//! step –, so that painting a Stelle without a step, or into one after its last step, changes
//! nothing. Without a step on any Stelle there is nothing to keep. The cache names no path: it
//! moves with the project. Per step the report also keeps whether the step ran without the
//! camera images: a run computes only from the first step that changed (adding a step computes
//! that one alone), or that ran without them while they are here now.
//!
//! **Kept aside.** A run that replaces steps – „Löschen“ of a Stelle with steps or of one before
//! later steps, its ↶ – keeps the replaced ones aside in [`FORMER_DIR`] (patches and report as
//! in the cache), each under its key, as long as ↶ can still bring it back: those baked with the
//! camera images (they and every step before them) whose step is still in `stellen.toml` (of a
//! Stelle, or of one taken out with „Löschen“). When its key returns, the step is taken as it
//! was instead of computed – with the same steps before it, as long as those came out as with
//! the camera images –, also while they are gone: so ↶ brings back the images of the later
//! Stellen byte for byte without the raw data, and fast with them. A step taken back with ↶
//! never returns and is not kept. At most [`FORMER_MAX_STEPS`] steps with about
//! [`FORMER_MAX_BYTES`] of patches are kept, the newest; older ones go. With no step on any
//! Stelle, the cache holds only what is kept aside, or goes.
//!
//! The folder is written as `nacharbeit.tmp-<process>-neu` under the lock of the cache and then
//! renamed into place, the old one put aside first (`…-alt`) and removed after: there is never
//! half a cache. A rename the system refuses for the moment (on Windows a virus scanner or an
//! indexer holding a file) is tried again for about 1.6 s ([`rename_retrying`]). A cancel writes
//! nothing. What a killed run left, the next run sorts out under the lock: a run killed between
//! its two renames left no cache in place but the old one intact aside – that goes back in
//! place ([`touchup_cut_off`]) –, everything else goes ([`touchup_leftovers`]). The files of the
//! surface stay as they are, byte for byte. No name starts with `oberflaeche`, and none ends in
//! `.tmp` once a run is through.
//!
//! Everything runs on the CPU and is deterministic: the same surface, Stellen and steps give
//! the same patches, byte for byte, on every machine – also when they are taken from what was
//! kept aside, which a full run would compute the same.

use crate::error::{check_cancel, failed, StepError};
use crate::export::SURFACE_GLB;
use crate::map::hex;
use crate::marks::{Dab, EditStep, Mark, Marks, SmoothLevel};
use crate::report_text;
use crate::surface::{surface_config, TextureSources, STALE_SURFACE, SURFACE_REPORT};
use driftless_export::mesh::{read_mesh, Mesh, MeshPage};
use driftless_io::atomic_write;
use driftless_io::images::{index_images, ImageError, ImageIndex, ImageIndices};
use driftless_io::lock::FileLock;
use driftless_io::progress::Progress;
use driftless_map::surface::edit::{
    apply_patches, page_edits, read_patches, retexture_region, select_faces, smooth_region,
    write_patches, EditError, FaceBvh, PageEdit, RetextureDone, SelectDab, Smoothing, SurfacePatch,
    WeldedSurface, EDIT_VERSION, PATCHES_FILE,
};
use driftless_map::surface::{SurfaceConfig, TextureSource};
use driftless_map::MapError;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

/// The folder of the touch-up in the surface folder.
pub const TOUCHUP_DIR: &str = "nacharbeit";
/// The report of the touch-up in [`TOUCHUP_DIR`]: its key and what came of every step.
pub const TOUCHUP_REPORT: &str = "nacharbeit.json";
/// The folder in [`TOUCHUP_DIR`] of the steps a run replaced that were baked with the camera
/// images, kept aside for ↶ (see the module docs).
pub const FORMER_DIR: &str = "frueher";
/// How often a rename of a folder the system refuses for the moment is tried
/// ([`rename_retrying`]).
const RENAME_TRIES: u32 = 10;
/// The first wait before such a rename is tried again; each further one twice as long …
const RENAME_WAIT: Duration = Duration::from_millis(5);
/// … at most this long.
const RENAME_WAIT_MAX: Duration = Duration::from_millis(500);
/// At most this many steps are kept aside ([`FORMER_DIR`]) …
const FORMER_MAX_STEPS: usize = 64;
/// … with at most about this many bytes of patches ([`patch_bytes`]).
const FORMER_MAX_BYTES: u64 = 64 * 1024 * 1024;
/// The hint of a step that needed the camera images while the raw data were gone (with a stale
/// surface: [`STALE_SURFACE`]).
const NO_RAW_DATA: &str = "Rohdaten fehlen – Bilder nicht neu eingespielt";
/// The start of the names of the folders a run writes before they are renamed into place
/// (`-neu`), and of the old cache put aside meanwhile (`-alt`): what a killed run leaves.
const TEMP_PREFIX: &str = "nacharbeit.tmp-";
/// The phase of reading the surface.
const PHASE_READ: &str = "Nacharbeit: Oberfläche lesen";
/// The phase of indexing the camera images of the sources ([`index_images`]).
const PHASE_INDEX: &str = "Bilder verzeichnen";

/// The touch-up of one surface.
#[derive(Clone, Debug)]
pub struct TouchupRequest {
    /// The surface folder (`oberflaeche-NNN`).
    pub surface_dir: PathBuf,
    /// The folder of the map or model whose Stellen are replayed: the folder the surface folder
    /// lies in (as [`edited_mesh`] and [`edited_pages`] take it).
    pub owner_dir: PathBuf,
    /// The camera images of the surface ([`crate::surface::texture_sources`]), or why there are
    /// none: the raw data gone, or no longer those of the surface.
    pub sources: TextureSources,
    /// What the program has at hand already; by default nothing, and everything is read here.
    pub prepared: TouchupPrepared,
}

/// What the program has at hand already for a touch-up ([`TouchupRequest::prepared`]): it
/// changes how long it takes, never what comes of it.
#[derive(Clone, Debug, Default)]
pub struct TouchupPrepared {
    /// The computed surface, read and welded already; used only for the glb it was read
    /// from, as long as the file is unchanged ([`PreparedSurface::fits`]).
    pub surface: Option<PreparedSurface>,
    /// Where the indices of the camera images are kept between runs; without, a run that bakes
    /// images anew indexes them for itself.
    pub indices: Option<Arc<ImageIndices>>,
}

/// A file as it was when it was read: its path, size and modification time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FileStamp {
    path: PathBuf,
    len: u64,
    modified: Option<SystemTime>,
}

impl FileStamp {
    /// The stamp of the file at `path` now; `None` if it cannot be looked at.
    pub fn of(path: &Path) -> Option<FileStamp> {
        let meta = fs::metadata(path).ok()?;
        Some(FileStamp {
            path: path.to_path_buf(),
            len: meta.len(),
            modified: meta.modified().ok(),
        })
    }
}

/// The glb of a surface folder as [`run_touchup`] reads it: the mesh as read, welded, and the
/// tree of its faces (the window's loader builds them for the brush). It belongs to the file it
/// was read from, as it was then.
#[derive(Clone, Debug)]
pub struct PreparedSurface {
    pub mesh: Arc<Mesh>,
    pub welded: Arc<WeldedSurface>,
    pub bvh: Arc<FaceBvh>,
    glb: FileStamp,
}

impl PreparedSurface {
    /// The surface read as `mesh` from the glb whose stamp was `glb` before it was read
    /// ([`FileStamp::of`]), welded as `welded` ([`WeldedSurface::from_mesh`]) with the tree
    /// `bvh` of its faces ([`FaceBvh::build`]).
    pub fn new(
        glb: FileStamp,
        mesh: Arc<Mesh>,
        welded: Arc<WeldedSurface>,
        bvh: Arc<FaceBvh>,
    ) -> PreparedSurface {
        PreparedSurface {
            mesh,
            welded,
            bvh,
            glb,
        }
    }

    /// Reads the glb at `glb`, welds it and builds the tree of its faces. Progress: the
    /// reading. Fails as reading the glb fails.
    pub fn read(glb: &Path, progress: &Progress) -> Result<PreparedSurface, StepError> {
        let stamp = FileStamp::of(glb)
            .ok_or_else(|| StepError::Failed(format!("{} nicht lesbar", glb.display())))?;
        let mesh = read_mesh(glb, progress)?;
        let welded = WeldedSurface::from_mesh(&mesh);
        let bvh = FaceBvh::build(&welded);
        Ok(PreparedSurface::new(
            stamp,
            Arc::new(mesh),
            Arc::new(welded),
            Arc::new(bvh),
        ))
    }

    /// Whether it is the glb at `glb` as that file is now: the same path, size and
    /// modification time.
    pub fn fits(&self, glb: &Path) -> bool {
        glb == self.glb.path && FileStamp::of(glb).is_some_and(|now| now == self.glb)
    }
}

/// What [`run_touchup`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TouchupDone {
    /// The steps of the Stellen the touch-up holds, all of them; those in [`Self::refused`]
    /// changed nothing.
    pub steps: usize,
    /// Hints for the program, German, each once: the raw data were missing or the surface is
    /// stale ([`STALE_SURFACE`]), a smoothed Stelle whose images could not be baked anew.
    pub hints: Vec<String>,
    /// The steps that changed nothing, in their order: the Stelle and why (German).
    pub refused: Vec<(u32, String)>,
    /// What came of every step, in the order of the replay.
    pub outcomes: Vec<StepOutcome>,
}

/// What came of one step of the touch-up ([`TouchupDone::outcomes`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StepOutcome {
    /// The Stelle …
    pub mark: u32,
    /// … and the step's `seq`.
    pub seq: u64,
    /// The images it baked: the variant, counted from 1, of how many („Variante 2 von 5“).
    pub variant_of: Option<(u32, u32)>,
    /// Why it changed nothing (German), if it was refused.
    pub refused: Option<String>,
    /// It needed the camera images, but they were not at hand
    /// ([`TextureSources::Unavailable`]: the raw data gone, or no longer those the surface was
    /// made from).
    pub without_raw_data: bool,
}

/// [`TOUCHUP_REPORT`].
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
struct Report {
    /// The key (see [`touchup_fits`]): the input id of the surface ...
    input_id: String,
    /// ... the size of its glb ...
    glb_bytes: u64,
    /// ... the version of the computation (and the keys of the steps).
    edit_version: u32,
    /// The size of [`PATCHES_FILE`]: one of another run does not fit.
    patches_bytes: u64,
    /// Every step in the order of the replay; the patches are in the same order.
    steps: Vec<StepRecord>,
}

/// What came of one step ([`Report::steps`]).
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
struct StepRecord {
    /// The Stelle.
    mark: u32,
    /// The step's `seq`.
    seq: u64,
    /// The key of this step and every one before it: the same steps up to here on the same
    /// surface give the same patches.
    key: String,
    /// It needed the camera images, but they were not at hand ([`StepOutcome::without_raw_data`]).
    without_raw_data: bool,
    /// The images it baked: the variant, counted from 1, of how many („Variante 2 von 5“).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    variant_of: Option<(u32, u32)>,
    /// Hints for the program (German).
    #[serde(rename = "hinweise", default)]
    hints: Vec<String>,
    /// Why it changed nothing (German), if it was refused.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    refused: Option<String>,
}

impl Report {
    /// Whether it was made for the surface `key` by this version of the computation.
    fn belongs_to(&self, key: &SurfaceKey) -> bool {
        self.input_id == key.input_id
            && self.glb_bytes == key.glb_bytes
            && self.edit_version == EDIT_VERSION
    }

    /// What its steps came to.
    fn done(&self) -> TouchupDone {
        let mut hints: Vec<String> = Vec::new();
        for hint in self.steps.iter().flat_map(|s| &s.hints) {
            if !hints.contains(hint) {
                hints.push(hint.clone());
            }
        }
        TouchupDone {
            steps: self.steps.len(),
            hints,
            refused: self
                .steps
                .iter()
                .filter_map(|s| Some((s.mark, s.refused.clone()?)))
                .collect(),
            outcomes: self
                .steps
                .iter()
                .map(|s| StepOutcome {
                    mark: s.mark,
                    seq: s.seq,
                    variant_of: s.variant_of,
                    refused: s.refused.clone(),
                    without_raw_data: s.without_raw_data,
                })
                .collect(),
        }
    }
}

/// What the patches of a surface belong to: the input id of its report and the size of its
/// glb.
struct SurfaceKey {
    input_id: String,
    glb_bytes: u64,
}

impl SurfaceKey {
    /// The key of the surface in `dir` with the report `report`.
    fn of(dir: &Path, report: &Value) -> Result<SurfaceKey, StepError> {
        let input_id = report["input_id"].as_str().ok_or_else(|| {
            format!(
                "{} ohne Eingabe-Kennung",
                dir.join(SURFACE_REPORT).display()
            )
        })?;
        let glb = dir.join(SURFACE_GLB);
        let glb_bytes = fs::metadata(&glb)
            .map_err(|e| format!("{} nicht lesbar: {e}", glb.display()))?
            .len();
        Ok(SurfaceKey {
            input_id: input_id.to_string(),
            glb_bytes,
        })
    }
}

/// The report of the surface in `dir`.
fn surface_report(dir: &Path) -> Result<Value, StepError> {
    let path = dir.join(SURFACE_REPORT);
    fs::read_to_string(&path)
        .map_err(|e| e.to_string())
        .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        .map_err(|e| format!("{} nicht lesbar: {e}", path.display()).into())
}

/// The report in the cache folder `cache`, if one is there and reads.
fn read_report(cache: &Path) -> Option<Report> {
    let text = fs::read_to_string(cache.join(TOUCHUP_REPORT)).ok()?;
    serde_json::from_str(&text).ok()
}

/// Whether the patches in the cache folder `cache` have the size `report` recorded.
fn patches_intact(cache: &Path, report: &Report) -> bool {
    fs::metadata(cache.join(PATCHES_FILE))
        .is_ok_and(|m| m.is_file() && m.len() == report.patches_bytes)
}

/// Whether `report` holds the steps `events`: as many, each with the same key.
fn holds(report: &Report, events: &[Event]) -> bool {
    report.steps.len() == events.len()
        && report.steps.iter().zip(events).all(|(r, e)| r.key == e.key)
}

/// The report of the cache of the surface in `surface_dir` if it holds the steps `events`;
/// `None` also without a step.
fn fitting(surface_dir: &Path, events: &[Event]) -> Option<Report> {
    if events.is_empty() {
        return None;
    }
    let key = SurfaceKey::of(surface_dir, &surface_report(surface_dir).ok()?).ok()?;
    let cache = surface_dir.join(TOUCHUP_DIR);
    read_report(&cache)
        .filter(|r| r.belongs_to(&key) && holds(r, events) && patches_intact(&cache, r))
}

/// Whether the cache in the surface folder `surface_dir` holds the touch-up of `marks`, the
/// Stellen of its map or model: its key is the input id of the surface's report, the size of
/// its glb, [`EDIT_VERSION`] and the keys of the steps (each with the Stelle as the step sees
/// it: a stroke painted after the last step of a Stelle counts for later steps only), and the
/// patches have the size it recorded. True without a step on any Stelle: there is nothing to
/// keep. A cache computed without the camera images fits as well; [`run_touchup`] with them
/// computes those steps anew.
pub fn touchup_fits(surface_dir: &Path, marks: &Marks) -> bool {
    let events = events(marks);
    events.is_empty() || fitting(surface_dir, &events).is_some()
}

/// Whether the cache in the surface folder `surface_dir` holds a step computed without the camera
/// images (one that needed them while they were not at hand: the raw data gone, or no longer
/// those the surface was made from): once they are found ([`TextureSources::Found`]),
/// [`run_touchup`] computes it anew, although the cache fits ([`touchup_fits`]). False without
/// a cache.
pub fn touchup_without_raw_data(surface_dir: &Path) -> bool {
    read_report(&surface_dir.join(TOUCHUP_DIR))
        .is_some_and(|r| r.steps.iter().any(|s| s.without_raw_data))
}

/// A step of a Stelle, in the order of the replay.
struct Event<'a> {
    mark: &'a Mark,
    step: EditStep,
    /// [`StepRecord::key`].
    key: String,
}

/// The steps of all Stellen of `marks` in the order of the replay: by `seq`, at a tie in the
/// order of the file. The key of each is made from the one before it and the Stelle as the
/// step sees it: its number, the dabs painted before the step and the step, written as
/// [`Marks::fingerprint`] writes them.
fn events(marks: &Marks) -> Vec<Event<'_>> {
    let mut order: Vec<(u64, usize, usize)> = marks
        .marks
        .iter()
        .enumerate()
        .flat_map(|(i, m)| {
            let steps = m.steps.iter().enumerate();
            steps.map(move |(j, s)| (s.seq(), i, j))
        })
        .collect();
    // Unique: any sort gives the same order.
    order.sort_unstable();
    let mut key = String::new();
    order
        .into_iter()
        .map(|(_, i, j)| {
            let mark = &marks.marks[i];
            let step = mark.steps[j];
            let mut text = format!("{key}\nstelle {}\n", mark.nr);
            for d in mark.dabs.iter().filter(|d| d.seq < step.seq()) {
                text += &format!("{d:?}\n");
            }
            text += &format!("{step:?}\n");
            key = hex(&text);
            Event {
                mark,
                step,
                key: key.clone(),
            }
        })
        .collect()
}

/// A dab of a Stelle as the selection takes it.
fn select_dab(d: &Dab) -> SelectDab {
    SelectDab {
        centre: d.centre,
        normal: d.normal,
        radius: f64::from(d.radius),
        erase: d.erase,
    }
}

/// How the surface is smoothed for `level`.
fn smoothing(level: SmoothLevel) -> Smoothing {
    match level {
        SmoothLevel::Light => Smoothing::Light,
        SmoothLevel::Strong => Smoothing::Strong,
        SmoothLevel::Flat => Smoothing::Flat,
    }
}

/// `e` if it refuses the step (the step changes nothing), else the error of the touch-up.
fn refusal(e: EditError) -> Result<EditError, StepError> {
    match e {
        EditError::NotFlat { .. }
        | EditError::NoFaces
        | EditError::NoImages
        | EditError::NoReference => Ok(e),
        e => Err(e.into()),
    }
}

/// The hint for Stelle `nr`, smoothed, whose images could not be baked anew because of `why`.
fn smoothed_only(nr: u32, why: &EditError) -> String {
    let reason = match why {
        EditError::NoReference => {
            "um sie gibt es keine Textur, an die sich neue Bilder angleichen lassen"
        }
        _ => "kein Bild zeigt sie",
    };
    format!("Stelle {nr} ist geglättet, ihre Bilder aber nicht neu eingespielt: {reason}.")
}

/// The surface as the steps so far left it.
struct Replay<'a> {
    /// The computed surface, as its glb holds it.
    mesh: &'a Mesh,
    /// The welded surface with the moves of the steps so far, and the tree of its faces
    /// (shared with the program's until a step moves a vertex).
    s: Arc<WeldedSurface>,
    bvh: Arc<FaceBvh>,
    /// The patches of the steps so far, one per step.
    patches: Vec<SurfacePatch>,
    sources: &'a TextureSources,
    /// Per source the index of its images, once a step needed them, and where they are kept.
    indices: Option<Vec<Arc<ImageIndex>>>,
    kept: Option<&'a ImageIndices>,
    cfg: &'a SurfaceConfig,
}

/// What came of a step besides its patch ([`StepRecord`]).
#[derive(Default)]
struct Outcome {
    without_raw_data: bool,
    variant_of: Option<(u32, u32)>,
    hints: Vec<String>,
    refused: Option<String>,
}

/// How baking the images of a Stelle anew went.
enum Baked {
    Done(RetextureDone),
    /// There are no camera images to bake from: the hint that says why.
    Unavailable(&'static str),
    /// It changed nothing, for this reason.
    Refused(EditError),
}

impl Replay<'_> {
    /// Runs `step` of `mark` (see the module docs) and adds its patch.
    fn run(
        &mut self,
        mark: &Mark,
        step: EditStep,
        progress: &Progress,
    ) -> Result<Outcome, StepError> {
        let dabs: Vec<SelectDab> = mark
            .dabs
            .iter()
            .filter(|d| d.seq < step.seq())
            .map(select_dab)
            .collect();
        let faces = select_faces(&self.s, &self.bvh, &dabs);
        let mut out = Outcome::default();
        if faces.is_empty() {
            out.refused = Some(EditError::NoFaces.to_string());
            self.patches.push(SurfacePatch::default());
            return Ok(out);
        }
        let patch = match step {
            EditStep::Smooth { level, .. } => {
                match smooth_region(&self.s, &faces, smoothing(level)) {
                    Ok(moved) => {
                        Arc::make_mut(&mut self.s).apply_moves(&moved);
                        self.bvh = Arc::new(FaceBvh::build(&self.s));
                        // The images of the Stelle anew, onto the surface as it is smoothed now.
                        self.patches.push(SurfacePatch {
                            moved,
                            ..SurfacePatch::default()
                        });
                        let baked = self.bake(&faces, 0, progress);
                        let mut patch = self.patches.pop().expect("the smoothing");
                        match baked? {
                            Baked::Done(done) => {
                                patch.removed_faces = done.patch.removed_faces;
                                patch.pages = done.patch.pages;
                                out.variant_of = Some(done.variant_of);
                            }
                            Baked::Unavailable(hint) => {
                                out.without_raw_data = true;
                                out.hints.push(hint.into());
                            }
                            Baked::Refused(why) => out.hints.push(smoothed_only(mark.nr, &why)),
                        }
                        patch
                    }
                    Err(e) => {
                        out.refused = Some(refusal(e)?.to_string());
                        SurfacePatch::default()
                    }
                }
            }
            EditStep::Retexture { variant, .. } => match self.bake(&faces, variant, progress)? {
                Baked::Done(done) => {
                    out.variant_of = Some(done.variant_of);
                    done.patch
                }
                Baked::Unavailable(hint) => {
                    out.without_raw_data = true;
                    out.hints.push(hint.into());
                    SurfacePatch::default()
                }
                Baked::Refused(why) => {
                    out.refused = Some(why.to_string());
                    SurfacePatch::default()
                }
            },
        };
        self.patches.push(patch);
        Ok(out)
    }

    /// Takes `patch` as the patch of the next step, as computing that step gave it before.
    fn take(&mut self, patch: SurfacePatch) {
        if !patch.moved.is_empty() {
            Arc::make_mut(&mut self.s).apply_moves(&patch.moved);
            self.bvh = Arc::new(FaceBvh::build(&self.s));
        }
        self.patches.push(patch);
    }

    /// The images of the Stelle `faces` baked anew as `variant`, onto the surface as the steps
    /// so far left it.
    fn bake(
        &mut self,
        faces: &[u32],
        variant: u32,
        progress: &Progress,
    ) -> Result<Baked, StepError> {
        let sources = match self.sources {
            TextureSources::Found(sources) => sources,
            TextureSources::Unavailable(why) if why.raw_data_missing() => {
                return Ok(Baked::Unavailable(NO_RAW_DATA))
            }
            TextureSources::Unavailable(_) => return Ok(Baked::Unavailable(STALE_SURFACE)),
        };
        let indices = self.indices(sources, progress)?;
        let current = apply_patches(self.mesh, &self.s, &self.patches);
        match retexture_region(
            &self.s, &self.bvh, &current, faces, sources, &indices, variant, self.cfg, progress,
        ) {
            Ok(done) => Ok(Baked::Done(done)),
            Err(e) => refusal(e).map(Baked::Refused),
        }
    }

    /// Per source the index of its images: those kept ([`ImageIndices`]) where they are current,
    /// else made now (phase „Bilder verzeichnen“), once per run.
    fn indices(
        &mut self,
        sources: &[TextureSource],
        progress: &Progress,
    ) -> Result<Vec<Arc<ImageIndex>>, StepError> {
        if let Some(indices) = &self.indices {
            return Ok(indices.clone());
        }
        let kept = self.kept;
        let mut announced = false;
        let indices = sources
            .iter()
            .map(|src| {
                let (bags, topics) = (&src.images.bags, &src.images.topics);
                let made = kept.is_some_and(|k| k.holds(bags, topics));
                if !made && !announced {
                    progress.phase(PHASE_INDEX, None);
                    announced = true;
                }
                match kept {
                    Some(kept) => kept.get(bags, topics, progress),
                    None => index_images(bags, topics, progress).map(Arc::new),
                }
            })
            .collect::<Result<Vec<_>, ImageError>>()
            .map_err(|e| match e {
                ImageError::Cancelled => StepError::Cancelled,
                e => EditError::from(MapError::Images(e.to_string())).into(),
            })?;
        self.indices = Some(indices.clone());
        Ok(indices)
    }
}

/// Whether every vertex and face the patches name is one of `s`: they were made for its glb.
fn patches_fit(s: &WeldedSurface, patches: &[SurfacePatch]) -> bool {
    let (vertices, faces) = (s.positions().len(), s.face_count());
    // Both sorted (a patch read is checked); the faces of the pages are among the removed.
    patches.iter().all(|p| {
        p.moved.last().is_none_or(|&(v, _)| (v as usize) < vertices)
            && p.removed_faces.last().is_none_or(|&f| (f as usize) < faces)
    })
}

/// A step of a cache with its patch.
type Kept = (StepRecord, SurfacePatch);

/// The report in the cache folder `cache` if it is of the surface `key` and this computation
/// ([`Report::belongs_to`]) and the patches have the size it recorded.
fn fitting_report(cache: &Path, key: &SurfaceKey) -> Option<Report> {
    read_report(cache).filter(|r| r.belongs_to(key) && patches_intact(cache, r))
}

/// The steps of `report`, the report in the cache folder `cache`, with their patches, read in
/// full and checked; none if they do not read (a damaged cache is computed anew).
fn with_patches(cache: &Path, report: &Report) -> Vec<Kept> {
    match read_patches(cache) {
        Ok(p) if p.len() == report.steps.len() => report.steps.iter().cloned().zip(p).collect(),
        _ => Vec::new(),
    }
}

/// The steps kept aside in the cache folder `cache` ([`FORMER_DIR`]), oldest first, with their
/// patches; none if there are none, or they are of another surface or computation than `key`,
/// or damaged.
fn read_former(cache: &Path, key: &SurfaceKey) -> Vec<Kept> {
    let dir = cache.join(FORMER_DIR);
    fitting_report(&dir, key).map_or_else(Vec::new, |r| with_patches(&dir, &r))
}

/// About the bytes `patch` takes in [`PATCHES_FILE`]: its moved vertices, its faces, its pages
/// with their corners, triangles and images.
fn patch_bytes(patch: &SurfacePatch) -> u64 {
    let n = |len: usize| len as u64;
    let pages: u64 = patch
        .pages
        .iter()
        .map(|p| 24 + 32 * n(p.positions.len()) + 16 * n(p.indices.len()) + n(p.jpeg.len()))
        .sum();
    24 + 28 * n(patch.moved.len()) + 4 * n(patch.removed_faces.len()) + pages
}

/// The steps to keep aside of `former` (oldest first, each key once): the newest, at most
/// [`FORMER_MAX_STEPS`] of them and about [`FORMER_MAX_BYTES`] of patches ([`patch_bytes`]);
/// older ones go.
fn bounded(mut former: Vec<Kept>) -> Vec<Kept> {
    let (mut bytes, mut keep) = (0, 0);
    for (_, patch) in former.iter().rev() {
        let more = patch_bytes(patch);
        if keep == FORMER_MAX_STEPS || bytes + more > FORMER_MAX_BYTES {
            break;
        }
        bytes += more;
        keep += 1;
    }
    former.drain(..former.len() - keep);
    former
}

/// What to keep aside after a run (see the module docs): `before` (kept aside so far, oldest
/// first), then `dropped` (the steps of the replaced cache it does not hold, in their order),
/// without those `now` holds, those whose step is not in `in_file` any more and – of two with
/// one key – the older, [`bounded`].
fn kept_aside(
    before: Vec<Kept>,
    dropped: Vec<Kept>,
    now: &[StepRecord],
    in_file: &BTreeSet<(u32, u64)>,
) -> Vec<Kept> {
    let held: BTreeSet<&str> = now.iter().map(|s| s.key.as_str()).collect();
    let mut seen = BTreeSet::new();
    let mut former: Vec<Kept> = before
        .into_iter()
        .chain(dropped)
        .rev()
        .filter(|(r, _)| {
            !held.contains(r.key.as_str())
                && in_file.contains(&(r.mark, r.seq))
                && seen.insert(r.key.clone())
        })
        .collect();
    former.reverse();
    bounded(former)
}

/// Replays the steps of the Stellen of the map or model in [`TouchupRequest::owner_dir`] on the
/// surface in [`TouchupRequest::surface_dir`] and keeps the patches in its [`TOUCHUP_DIR`] (see
/// the module docs). What the cache holds of the same steps is kept: if it fits the Stellen
/// ([`touchup_fits`]), nothing is computed or written. A step kept aside ([`FORMER_DIR`]) whose
/// key comes back is taken as it was. Without a step on any Stelle an older cache goes, but for
/// what is kept aside. Progress: the phase „Nacharbeit: Oberfläche lesen“ (with the reading of
/// the glb), then „Nacharbeit: Schritt i von n“ per step computed, with the phases of baking
/// images anew within; a cancel is checked between the steps and within them and leaves the
/// cache as it was.
///
/// Fails if the surface's report or glb or `stellen.toml` cannot be read, the cache cannot be
/// written (or its lock is held by another program meanwhile), or the camera images cannot be
/// read. A refused step is no failure ([`TouchupDone::refused`]).
pub fn run_touchup(req: &TouchupRequest, progress: &Progress) -> Result<TouchupDone, StepError> {
    check_cancel(progress)?;
    let dir = &req.surface_dir;
    let report = surface_report(dir)?;
    let key = SurfaceKey::of(dir, &report)?;
    let marks = Marks::load(&req.owner_dir)?;
    // A run killed while it put its cache in place: the old one back first.
    sort_out(dir)?;
    let events = events(&marks);
    let n = events.len();
    let found = req.sources.found().is_some();
    let in_file: BTreeSet<(u32, u64)> = marks.all_steps().collect();

    // What the cache holds of these steps: the same steps up to there, none computed without
    // the raw data that are here now. Of the others, those baked with the camera images (they
    // and every step before them) whose step is still in `stellen.toml` go aside.
    let cache = dir.join(TOUCHUP_DIR);
    let old = fitting_report(&cache, &key);
    let listed = old.as_ref().map_or(&[][..], |r| &r.steps[..]);
    let keeps = |r: &StepRecord, e: &Event| r.key == e.key && (!found || !r.without_raw_data);
    let kept = listed
        .iter()
        .zip(&events)
        .take_while(|(r, e)| keeps(r, e))
        .count();
    // the steps from the first on that came out as with the camera images
    let baked = listed.iter().take_while(|r| !r.without_raw_data).count();
    let goes_aside = (kept..baked).any(|i| in_file.contains(&(listed[i].mark, listed[i].seq)));
    // Read in full (checked): a damaged file is computed anew.
    let mut cached = match &old {
        Some(old) if kept > 0 || goes_aside => with_patches(&cache, old),
        _ => Vec::new(),
    };
    let kept = kept.min(cached.len());
    if let Some(old) = old.filter(|_| n > 0 && kept == n && cached.len() == n) {
        return Ok(old.done());
    }
    let dropped: Vec<Kept> = cached
        .drain(kept..)
        .take(baked.saturating_sub(kept))
        .collect();
    let before = read_former(&cache, &key);
    let (mut steps, mut patches): (Vec<StepRecord>, Vec<SurfacePatch>) = cached.into_iter().unzip();
    let mut reused = steps.len();

    // The steps from the first not kept on: taken from those kept aside as long as every step
    // before came out as with the camera images, else computed.
    let aside: BTreeMap<&str, &Kept> = before.iter().map(|k| (k.0.key.as_str(), k)).collect();
    let rest = &events[steps.len()..];
    let all_aside = !rest.is_empty()
        && steps.iter().all(|s| !s.without_raw_data)
        && rest.iter().all(|e| aside.contains_key(e.key.as_str()));
    if all_aside {
        // nothing to compute: the surface need not be read
        for event in rest {
            let (record, patch) = aside[event.key.as_str()].clone();
            steps.push(record);
            patches.push(patch);
            reused += 1;
        }
    } else if !rest.is_empty() {
        let cfg = surface_config(&report).ok_or_else(|| {
            format!(
                "{}: Einstellungen nicht lesbar",
                dir.join(SURFACE_REPORT).display()
            )
        })?;
        // The surface as read from its glb: the one the program has at hand, if it is of
        // this glb as it is now.
        let glb = dir.join(SURFACE_GLB);
        let (mesh, mut s, tree) = match &req.prepared.surface {
            Some(p) if p.fits(&glb) => (
                Arc::clone(&p.mesh),
                Arc::clone(&p.welded),
                Some(Arc::clone(&p.bvh)),
            ),
            _ => {
                progress.phase(PHASE_READ, None);
                let mesh = read_mesh(&glb, progress)?;
                check_cancel(progress)?;
                let s = WeldedSurface::from_mesh(&mesh);
                (Arc::new(mesh), Arc::new(s), None)
            }
        };
        if !patches_fit(&s, &patches) {
            steps.clear();
            patches.clear();
            reused = 0;
        }
        let moved = patches.iter().any(|p| !p.moved.is_empty());
        for patch in patches.iter().filter(|p| !p.moved.is_empty()) {
            Arc::make_mut(&mut s).apply_moves(&patch.moved);
        }
        let bvh = match tree {
            Some(tree) if !moved => tree,
            _ => Arc::new(FaceBvh::build(&s)),
        };
        let mut replay = Replay {
            mesh: &mesh,
            s,
            bvh,
            patches,
            sources: &req.sources,
            indices: None,
            kept: req.prepared.indices.as_deref(),
            cfg: &cfg,
        };
        let mut exact = steps.iter().all(|s| !s.without_raw_data);
        for (i, event) in events.iter().enumerate().skip(steps.len()) {
            check_cancel(progress)?;
            let former = aside
                .get(event.key.as_str())
                .filter(|(_, p)| exact && patches_fit(&replay.s, std::slice::from_ref(p)));
            if let Some((record, patch)) = former {
                replay.take(patch.clone());
                steps.push(record.clone());
                reused += 1;
                continue;
            }
            progress.phase(&format!("Nacharbeit: Schritt {} von {n}", i + 1), None);
            let out = replay.run(event.mark, event.step, progress)?;
            exact &= !out.without_raw_data;
            steps.push(StepRecord {
                mark: event.mark.nr,
                seq: event.step.seq(),
                key: event.key.clone(),
                without_raw_data: out.without_raw_data,
                variant_of: out.variant_of,
                hints: out.hints,
                refused: out.refused,
            });
        }
        patches = replay.patches;
    }
    let former = kept_aside(before, dropped, &steps, &in_file);
    // Last chance to stop: from here on the cache is written.
    check_cancel(progress)?;
    let report = |steps: Vec<StepRecord>| Report {
        input_id: key.input_id.clone(),
        glb_bytes: key.glb_bytes,
        edit_version: EDIT_VERSION,
        patches_bytes: 0,
        steps,
    };
    let (former_steps, former_patches): (Vec<StepRecord>, Vec<SurfacePatch>) =
        former.into_iter().unzip();
    let mut aside_report = report(former_steps);
    let aside_content =
        (!aside_report.steps.is_empty()).then_some((&mut aside_report, former_patches.as_slice()));
    if n == 0 {
        // Nothing to compute: no cache but what is kept aside, if that changed.
        let unchanged = read_report(&cache).is_none()
            && read_report(&cache.join(FORMER_DIR)).map(|r| r.steps)
                == aside_content.as_ref().map(|(r, _)| r.steps.clone());
        if !unchanged {
            store(dir, None, aside_content)?;
        }
        return Ok(TouchupDone::default());
    }
    let mut main = report(steps);
    store(dir, Some((&mut main, &patches)), aside_content)?;
    progress.info(format!(
        "Nacharbeit: {n} Schritt(e), davon {} neu gerechnet → {}",
        n - reused,
        cache.display()
    ));
    Ok(main.done())
}

/// What runs of the touch-up that were killed left in the surface folder `surface_dir`:
/// everything named `nacharbeit.tmp-…` (the folders a run writes before it renames them into
/// place, and the old cache put aside meanwhile), sorted. The next run removes them under the
/// lock of the cache – after it put back an old cache put aside while there is none in place
/// ([`touchup_cut_off`]).
pub fn touchup_leftovers(surface_dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(surface_dir) else {
        return Vec::new();
    };
    let mut found: Vec<PathBuf> = entries
        .flatten()
        .filter(|e| {
            e.file_name()
                .to_str()
                .is_some_and(|n| n.starts_with(TEMP_PREFIX))
        })
        .map(|e| e.path())
        .collect();
    found.sort();
    found
}

/// The folder of a touch-up that `path` is or lies in: the [`TOUCHUP_DIR`] of a surface folder
/// (one with its report), or a folder a run of it writes or leaves (`nacharbeit.tmp-…`); `None`
/// if none. What lies there is the touch-up's: its next run replaces the folder.
pub(crate) fn cache_folder_of(path: &Path) -> Option<PathBuf> {
    let cache = |a: &&Path| {
        let name = a.file_name().and_then(|n| n.to_str());
        let named = name.is_some_and(|n| n == TOUCHUP_DIR || n.starts_with(TEMP_PREFIX));
        named && a.parent().is_some_and(|s| s.join(SURFACE_REPORT).is_file())
    };
    path.ancestors().find(cache).map(Path::to_path_buf)
}

/// Whether a run of the touch-up of the surface in `surface_dir` was killed between its two
/// renames: no cache in place ([`TOUCHUP_DIR`]), but one put aside (`nacharbeit.tmp-…-alt`) that
/// is intact – its reports read and its patches have the sizes they recorded. The next run puts
/// it back; until then its `nacharbeit.tmp-…` are no leftovers to clean up.
pub fn touchup_cut_off(surface_dir: &Path) -> bool {
    put_aside(surface_dir).is_some()
}

/// The intact cache put aside by a run killed between its two renames, while none is in place
/// ([`touchup_cut_off`]); of several the first by name.
fn put_aside(surface_dir: &Path) -> Option<PathBuf> {
    if fs::symlink_metadata(surface_dir.join(TOUCHUP_DIR)).is_ok() {
        return None;
    }
    touchup_leftovers(surface_dir).into_iter().find(|p| {
        let aside = p
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.ends_with("-alt"));
        aside && p.is_dir() && intact(p)
    })
}

/// Whether the cache folder `cache` is intact: it holds a report, the touch-up's or that of
/// the steps kept aside ([`FORMER_DIR`]), and every report there reads and has its patches of
/// the size it recorded.
fn intact(cache: &Path) -> bool {
    let former = cache.join(FORMER_DIR);
    let check = |dir: &Path| {
        let there = fs::symlink_metadata(dir.join(TOUCHUP_REPORT)).is_ok();
        let fine = read_report(dir).is_some_and(|r| patches_intact(dir, &r));
        (there, fine)
    };
    let (main, main_fine) = check(cache);
    let (aside, aside_fine) = check(&former);
    (main || aside) && (!main || main_fine) && (!aside || aside_fine)
}

/// What killed runs left in the surface folder `dir` ([`touchup_leftovers`]).
fn leftovers(dir: &Path) -> Vec<PathBuf> {
    touchup_leftovers(dir)
}

/// Sorts out what killed runs left in the surface folder `dir`, under the lock of the cache
/// ([`sort_out_locked`]); without any (the usual case) the lock is not taken.
fn sort_out(dir: &Path) -> Result<(), StepError> {
    if leftovers(dir).is_empty() {
        return Ok(());
    }
    let _lock = FileLock::acquire(&dir.join(TOUCHUP_DIR)).map_err(failed)?;
    sort_out_locked(dir)
}

/// [`sort_out`] with the lock of the cache held: an intact cache put aside while none is in
/// place goes back ([`touchup_cut_off`]), everything else a killed run left goes. Fails if it
/// cannot be put back: then nothing is removed, it waits for the next run.
fn sort_out_locked(dir: &Path) -> Result<(), StepError> {
    if let Some(old) = put_aside(dir) {
        let dest = dir.join(TOUCHUP_DIR);
        rename_retrying(&old, &dest).map_err(|e| {
            format!(
                "{} nicht in {} zurückbenennbar ({e}): die Nacharbeit vor dem unterbrochenen \
                 Lauf liegt noch dort; nichts gerechnet",
                old.display(),
                dest.display()
            )
        })?;
    }
    for path in leftovers(dir) {
        remove(&path);
    }
    Ok(())
}

/// Renames the folder `from` to `to`; while the system refuses it for the moment, it tries
/// again ([`retrying`]).
fn rename_retrying(from: &Path, to: &Path) -> io::Result<()> {
    retrying(|| fs::rename(from, to), std::thread::sleep)
}

/// Runs `op`, and again while it fails with a refusal that passes ([`passing`]):
/// [`RENAME_TRIES`] times in all, with `sleep` between – [`RENAME_WAIT`], twice as long each
/// time up to [`RENAME_WAIT_MAX`], about 1.6 s in all –, as cargo and rustup do for Windows.
/// Gives the last error.
fn retrying(
    mut op: impl FnMut() -> io::Result<()>,
    mut sleep: impl FnMut(Duration),
) -> io::Result<()> {
    let mut wait = RENAME_WAIT;
    for _ in 1..RENAME_TRIES {
        match op() {
            Err(e) if passing(&e) => {
                sleep(wait);
                wait = (wait * 2).min(RENAME_WAIT_MAX);
            }
            done => return done,
        }
    }
    op()
}

/// Whether the rename error `e` is a refusal that passes: access denied – what Windows says
/// while a virus scanner, an indexer or another program holds a file in the folder – and, on
/// Windows, a sharing or lock violation.
fn passing(e: &io::Error) -> bool {
    e.kind() == io::ErrorKind::PermissionDenied
        || (cfg!(windows) && matches!(e.raw_os_error(), Some(32 | 33)))
}

/// Removes the file or folder `path` as far as it goes; what stays, the next run removes.
fn remove(path: &Path) {
    let _ = if fs::symlink_metadata(path).is_ok_and(|m| m.is_dir()) {
        fs::remove_dir_all(path)
    } else {
        fs::remove_file(path)
    };
}

/// A cache as [`store`] writes it: its report, whose `patches_bytes` is set there, and its
/// patches.
type Content<'a> = (&'a mut Report, &'a [SurfacePatch]);

/// Puts the cache of the surface in `surface_dir` in place: `main` (the touch-up) and `former`
/// (the steps kept aside, [`FORMER_DIR`]), or none if neither. Under the lock of the cache,
/// after what killed runs left is removed (see the module docs).
fn store(
    surface_dir: &Path,
    main: Option<Content>,
    former: Option<Content>,
) -> Result<(), StepError> {
    let dest = surface_dir.join(TOUCHUP_DIR);
    let there = |p: &Path| fs::symlink_metadata(p).is_ok();
    let empty = main.is_none() && former.is_none();
    if empty && !there(&dest) && leftovers(surface_dir).is_empty() {
        return Ok(());
    }
    let _lock = FileLock::acquire(&dest).map_err(failed)?;
    sort_out_locked(surface_dir)?;
    let pid = std::process::id();
    let aside = surface_dir.join(format!("{TEMP_PREFIX}{pid}-alt"));
    let unmovable = |e: std::io::Error| -> StepError {
        format!("{} nicht ersetzbar: {e}", dest.display()).into()
    };
    if empty {
        if there(&dest) {
            rename_retrying(&dest, &aside).map_err(unmovable)?;
            remove(&aside);
        }
        return Ok(());
    }
    let new = surface_dir.join(format!("{TEMP_PREFIX}{pid}-neu"));
    let write = || -> Result<(), StepError> {
        if let Some((report, patches)) = main {
            write_cache(&new, report, patches)?;
        }
        if let Some((report, patches)) = former {
            write_cache(&new.join(FORMER_DIR), report, patches)?;
        }
        Ok(())
    };
    let written = write().and_then(|()| {
        // The old one aside, the new one in its place, then the old one away.
        let old = there(&dest);
        if old {
            rename_retrying(&dest, &aside).map_err(unmovable)?;
        }
        if let Err(e) = rename_retrying(&new, &dest) {
            if old {
                let _ = rename_retrying(&aside, &dest);
            }
            return Err(unmovable(e));
        }
        if old {
            remove(&aside);
        }
        Ok(())
    });
    if written.is_err() {
        remove(&new);
    }
    written
}

/// Writes the cache `report` and `patches` into the new folder `dir`.
fn write_cache(dir: &Path, report: &mut Report, patches: &[SurfacePatch]) -> Result<(), StepError> {
    write_patches(dir, patches)?;
    let path = dir.join(PATCHES_FILE);
    report.patches_bytes = fs::metadata(&path)
        .map_err(|e| format!("{} nicht lesbar: {e}", path.display()))?
        .len();
    let path = dir.join(TOUCHUP_REPORT);
    atomic_write(&path, report_text(report)?.as_bytes())
        .map_err(|e| format!("{} nicht schreibbar: {e}", path.display()).into())
}

/// The cache of a surface for the touched-up surface.
enum Fit {
    /// No Stelle of its map or model has a step.
    Nothing,
    /// The cache does not fit the Stellen (yet).
    Stale,
    Patches(Vec<SurfacePatch>),
}

/// The patches of the cache of the surface in `surface_dir` if it fits the Stellen of its map
/// or model, the folder it lies in ([`touchup_fits`]). Fails if `stellen.toml` cannot be read
/// or the patches of a fitting cache are damaged.
fn fitting_patches(surface_dir: &Path) -> Result<Fit, StepError> {
    let owner = surface_dir.parent().unwrap_or(Path::new(""));
    let marks = Marks::load(owner)?;
    let events = events(&marks);
    if events.is_empty() {
        return Ok(Fit::Nothing);
    }
    let Some(report) = fitting(surface_dir, &events) else {
        return Ok(Fit::Stale);
    };
    let patches = read_patches(&surface_dir.join(TOUCHUP_DIR))?;
    Ok(if patches.len() == report.steps.len() {
        Fit::Patches(patches)
    } else {
        Fit::Stale
    })
}

/// The touched-up surface of a surface folder, for the export.
pub(crate) enum Edited {
    /// No Stelle of its map or model has a step: it is the surface as computed.
    Unchanged,
    /// The cache does not fit the Stellen (yet): [`run_touchup`] first.
    Stale,
    Touched(Mesh),
}

/// The touched-up surface of the surface folder `surface_dir` (see [`edited_mesh`]).
pub(crate) fn edited(surface_dir: &Path, progress: &Progress) -> Result<Edited, StepError> {
    let patches = match fitting_patches(surface_dir)? {
        Fit::Nothing => return Ok(Edited::Unchanged),
        Fit::Stale => return Ok(Edited::Stale),
        Fit::Patches(p) => p,
    };
    let mesh = read_mesh(&surface_dir.join(SURFACE_GLB), progress)?;
    let s = WeldedSurface::from_mesh(&mesh);
    if !patches_fit(&s, &patches) {
        return Ok(Edited::Stale);
    }
    Ok(Edited::Touched(apply_patches(&mesh, &s, &patches)))
}

/// The error of an export of the touched-up surface in `dir` whose touch-up does not fit the
/// Stellen (yet).
pub(crate) fn stale(dir: &Path) -> StepError {
    format!(
        "Die Nacharbeit der Oberfläche {} ist nicht aktuell: erst die Nacharbeit rechnen \
         lassen, dann exportieren",
        dir.display()
    )
    .into()
}

/// The touched-up surface of the surface folder `surface_dir`: its glb with the patches of its
/// cache applied, the old pages with their images as they are and the pages the steps added
/// after them. `None` without a step on any Stelle of its map or model (the folder it lies in),
/// and while the cache does not fit the Stellen ([`touchup_fits`]). Progress: the reading of
/// the glb. Fails if the glb or `stellen.toml` cannot be read or the patches of a fitting cache
/// are damaged.
pub fn edited_mesh(surface_dir: &Path, progress: &Progress) -> Result<Option<Mesh>, StepError> {
    Ok(match edited(surface_dir, progress)? {
        Edited::Touched(mesh) => Some(mesh),
        Edited::Unchanged | Edited::Stale => None,
    })
}

/// What the touch-up changes on a surface, for the view ([`edited_pages`]): the changed pages of
/// the computed surface and the pages the steps added, as [`page_edits`] gives them.
pub type EditedPages = (Vec<PageEdit>, Vec<MeshPage>);

/// What the touch-up changes on the surface of the surface folder `surface_dir`, for the view
/// ([`page_edits`]): the changed pages and the added ones. `mesh` is its glb as read and `s`
/// its welded surface. `None` as for [`edited_mesh`]. Fails as [`edited_mesh`].
///
/// # Panics
/// If `s` is not the welded surface of `mesh` (see [`page_edits`]).
pub fn edited_pages(
    surface_dir: &Path,
    s: &WeldedSurface,
    mesh: &Mesh,
) -> Result<Option<EditedPages>, StepError> {
    let Fit::Patches(patches) = fitting_patches(surface_dir)? else {
        return Ok(None);
    };
    if !patches_fit(s, &patches) {
        return Ok(None);
    }
    Ok(Some(page_edits(s, mesh, &patches)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_map::surface::edit::PatchPage;

    fn record(mark: u32, seq: u64, key: &str) -> StepRecord {
        StepRecord {
            mark,
            seq,
            key: key.into(),
            without_raw_data: false,
            variant_of: None,
            hints: Vec::new(),
            refused: None,
        }
    }

    /// A patch with one page holding an image of `bytes` bytes (shared: no memory per patch).
    fn patch(image: &Arc<[u8]>) -> SurfacePatch {
        SurfacePatch {
            pages: vec![PatchPage {
                jpeg: Arc::clone(image),
                ..PatchPage::default()
            }],
            ..SurfacePatch::default()
        }
    }

    /// A rename the system refuses for the moment (on Windows a virus scanner or an indexer
    /// holding a file in the folder) is tried again, waiting longer each time, about 1.6 s in
    /// all; any other error stops at once (R2-M5 of the final review).
    #[test]
    fn a_refused_rename_is_tried_again() {
        let denied = || io::Error::from(io::ErrorKind::PermissionDenied);
        let (mut tries, mut waits) = (0, Vec::new());
        let done = retrying(
            || {
                tries += 1;
                if tries < 4 {
                    Err(denied())
                } else {
                    Ok(())
                }
            },
            |d| waits.push(d),
        );
        assert!(done.is_ok());
        assert_eq!(tries, 4);
        let ms = Duration::from_millis;
        assert_eq!(waits, [ms(5), ms(10), ms(20)]);

        // refused for good: given up after the last try, the error as it was
        let (mut tries, mut waits) = (0, Vec::new());
        let err = retrying(
            || {
                tries += 1;
                Err(denied())
            },
            |d| waits.push(d),
        )
        .unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::PermissionDenied);
        assert_eq!(tries, RENAME_TRIES);
        let waited: Duration = waits.iter().sum();
        assert!(waited > ms(1000) && waited < ms(2000), "{waited:?}");

        // another error: at once
        let (mut tries, mut waits) = (0, Vec::new());
        let err = retrying(
            || {
                tries += 1;
                Err(io::Error::from(io::ErrorKind::NotFound))
            },
            |d| waits.push(d),
        )
        .unwrap_err();
        assert_eq!(
            (err.kind(), tries, waits.len()),
            (io::ErrorKind::NotFound, 1, 0)
        );

        // a folder renamed for real
        let dir = driftless_io::testscan::tmp("steps-umbenennen");
        fs::create_dir_all(dir.join("a").join("innen")).unwrap();
        rename_retrying(&dir.join("a"), &dir.join("b")).unwrap();
        assert!(dir.join("b").join("innen").is_dir() && !dir.join("a").exists());
    }

    fn keys(kept: &[Kept]) -> Vec<&str> {
        kept.iter().map(|(r, _)| r.key.as_str()).collect()
    }

    /// At most 64 steps and about 64 MiB of patches are kept aside, the newest; the oldest go.
    #[test]
    fn what_is_kept_aside_is_bounded() {
        let small: Arc<[u8]> = Arc::from(vec![0u8; 10]);
        let many: Vec<Kept> = (0..70)
            .map(|i| (record(1, i, &format!("k{i:02}")), patch(&small)))
            .collect();
        let kept = bounded(many);
        assert_eq!(kept.len(), FORMER_MAX_STEPS);
        assert_eq!(kept[0].0.key, "k06");
        assert_eq!(kept.last().unwrap().0.key, "k69");

        let large: Arc<[u8]> = Arc::from(vec![0u8; 20 << 20]);
        assert_eq!(patch_bytes(&patch(&large)), 24 + 24 + (20 << 20));
        let big: Vec<Kept> = (0..4)
            .map(|i| (record(1, i, &format!("g{i}")), patch(&large)))
            .collect();
        assert_eq!(keys(&bounded(big)), ["g1", "g2", "g3"]);
        // one alone larger than all that is kept: none
        let huge: Arc<[u8]> = Arc::from(vec![0u8; (FORMER_MAX_BYTES + 1) as usize]);
        assert!(bounded(vec![(record(1, 1, "h"), patch(&huge))]).is_empty());
    }

    /// Kept aside: what was before, then what the run dropped – not what the cache holds now,
    /// nor a step gone from `stellen.toml`, and of one key only the newer.
    #[test]
    fn what_goes_aside() {
        let image: Arc<[u8]> = Arc::from(vec![1u8; 3]);
        let other: Arc<[u8]> = Arc::from(vec![2u8; 3]);
        let before = vec![
            (record(1, 2, "a"), patch(&image)),
            (record(2, 4, "b"), patch(&image)),
            (record(3, 6, "gone"), patch(&image)),
        ];
        let dropped = vec![
            (record(2, 4, "b"), patch(&other)),
            (record(4, 8, "c"), patch(&image)),
            (record(5, 9, "now"), patch(&image)),
        ];
        let now = [record(5, 9, "now")];
        let in_file: BTreeSet<(u32, u64)> = [(1, 2), (2, 4), (4, 8), (5, 9)].into();
        let aside = kept_aside(before, dropped, &now, &in_file);
        assert_eq!(keys(&aside), ["a", "b", "c"]);
        assert_eq!(aside[1].1, patch(&other), "the newer of one key");
    }
}
