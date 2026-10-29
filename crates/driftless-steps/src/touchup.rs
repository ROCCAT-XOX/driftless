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
//!   Without the raw data the Stelle keeps its texture coordinates (the texture moves with the
//!   vertices), and a hint says so;
//! - „Bilder neu einspielen“ bakes the images of the Stelle anew with the step's variant;
//!   without the raw data it is skipped, with the same hint.
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
//! patches belong to exactly this glb), [`EDIT_VERSION`], and the fingerprint of the Stellen
//! that have a step ([`Marks::fingerprint`]), so that painting a Stelle without a step changes
//! nothing. Without a step on any Stelle there is nothing to keep. The cache names no path: it
//! moves with the project. Per step the report also keeps a key of the steps up to it and
//! whether the step ran without the raw data: a run computes only from the first step that
//! changed (adding a step computes that one alone), or that ran without the raw data while they
//! are here now.
//!
//! The folder is written as `nacharbeit.tmp-<process>-neu` under the lock of the cache and then
//! renamed into place, the old one put aside first (`…-alt`) and removed after: there is never
//! half a cache. A cancel writes nothing; what a killed run left, the next run removes under the
//! lock. The files of the surface stay as they are, byte for byte. No name starts with
//! `oberflaeche`, and none ends in `.tmp` once a run is through.
//!
//! Everything runs on the CPU and is deterministic: the same surface, Stellen and steps give
//! the same patches, byte for byte, on every machine.

use crate::error::{check_cancel, failed, StepError};
use crate::export::SURFACE_GLB;
use crate::map::hex;
use crate::marks::{Dab, EditStep, Mark, Marks, SmoothLevel};
use crate::report_text;
use crate::surface::{surface_config, SURFACE_REPORT};
use driftless_export::mesh::{read_mesh, Mesh, MeshPage};
use driftless_io::atomic_write;
use driftless_io::lock::FileLock;
use driftless_io::progress::Progress;
use driftless_map::surface::edit::{
    apply_patches, page_edits, read_patches, retexture_region, select_faces, smooth_region,
    write_patches, EditError, FaceBvh, PageEdit, RetextureDone, SelectDab, Smoothing, SurfacePatch,
    WeldedSurface, EDIT_VERSION, PATCHES_FILE,
};
use driftless_map::surface::{SurfaceConfig, TextureSource};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

/// The folder of the touch-up in the surface folder.
pub const TOUCHUP_DIR: &str = "nacharbeit";
/// The report of the touch-up in [`TOUCHUP_DIR`]: its key and what came of every step.
pub const TOUCHUP_REPORT: &str = "nacharbeit.json";
/// The hint of a step that needed the camera images while the raw data were gone.
const NO_RAW_DATA: &str = "Rohdaten fehlen – Bilder nicht neu eingespielt";
/// The start of the names of the folders a run writes before they are renamed into place
/// (`-neu`), and of the old cache put aside meanwhile (`-alt`): what a killed run leaves.
const TEMP_PREFIX: &str = "nacharbeit.tmp-";
/// The phase of reading the surface.
const PHASE_READ: &str = "Nacharbeit: Oberfläche lesen";

/// The touch-up of one surface.
#[derive(Clone, Debug, PartialEq)]
pub struct TouchupRequest {
    /// The surface folder (`oberflaeche-NNN`).
    pub surface_dir: PathBuf,
    /// The folder of the map or model whose Stellen are replayed: the folder the surface folder
    /// lies in (as [`edited_mesh`] and [`edited_pages`] take it).
    pub owner_dir: PathBuf,
    /// The camera images of the surface ([`crate::surface::texture_sources`]); `None` if the
    /// raw data are gone.
    pub sources: Option<Vec<TextureSource>>,
}

/// What [`run_touchup`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TouchupDone {
    /// The steps of the Stellen the touch-up holds, all of them; those in [`Self::refused`]
    /// changed nothing.
    pub steps: usize,
    /// Hints for the program, German, each once: the raw data were missing, a smoothed Stelle
    /// whose images could not be baked anew.
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
    /// It needed the camera images, but the raw data were gone.
    pub without_raw_data: bool,
}

/// [`TOUCHUP_REPORT`].
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
struct Report {
    /// The key (see [`touchup_fits`]): the input id of the surface ...
    input_id: String,
    /// ... the size of its glb ...
    glb_bytes: u64,
    /// ... the version of the computation ...
    edit_version: u32,
    /// ... and the fingerprint of the Stellen with a step.
    marks: String,
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
    /// It needed the camera images, but the raw data were gone.
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

/// The fingerprint of the Stellen of `marks` that have a step ([`Marks::fingerprint`]); `None`
/// if none has one.
fn stepped_fingerprint(marks: &Marks) -> Option<String> {
    let mut stepped = Marks::default();
    stepped.marks = marks
        .marks
        .iter()
        .filter(|m| !m.steps.is_empty())
        .cloned()
        .collect();
    (!stepped.marks.is_empty()).then(|| stepped.fingerprint())
}

/// The report of the cache of the surface in `surface_dir` if it fits `marks`; `None` also
/// without a step on any Stelle.
fn fitting(surface_dir: &Path, marks: &Marks) -> Option<Report> {
    let fingerprint = stepped_fingerprint(marks)?;
    let key = SurfaceKey::of(surface_dir, &surface_report(surface_dir).ok()?).ok()?;
    let cache = surface_dir.join(TOUCHUP_DIR);
    read_report(&cache)
        .filter(|r| r.belongs_to(&key) && r.marks == fingerprint && patches_intact(&cache, r))
}

/// Whether the cache in the surface folder `surface_dir` holds the touch-up of `marks`, the
/// Stellen of its map or model: its key is the input id of the surface's report, the size of
/// its glb, [`EDIT_VERSION`] and the fingerprint of the Stellen that have a step, and the
/// patches have the size it recorded. True without a step on any Stelle: there is nothing to
/// keep. A cache computed without the raw data fits as well; [`run_touchup`] with them
/// computes those steps anew.
pub fn touchup_fits(surface_dir: &Path, marks: &Marks) -> bool {
    stepped_fingerprint(marks).is_none() || fitting(surface_dir, marks).is_some()
}

/// Whether the cache in the surface folder `surface_dir` holds a step computed without the raw
/// data (one that needed the camera images while they were gone): with the raw data back,
/// [`run_touchup`] computes it anew, although the cache fits ([`touchup_fits`]). False
/// without a cache.
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
    /// The welded surface with the moves of the steps so far, and the tree of its faces.
    s: WeldedSurface,
    bvh: FaceBvh,
    /// The patches of the steps so far, one per step.
    patches: Vec<SurfacePatch>,
    sources: Option<&'a [TextureSource]>,
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
    /// The raw data are gone.
    NoRawData,
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
                        self.s.apply_moves(&moved);
                        self.bvh = FaceBvh::build(&self.s);
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
                            Baked::NoRawData => {
                                out.without_raw_data = true;
                                out.hints.push(NO_RAW_DATA.into());
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
                Baked::NoRawData => {
                    out.without_raw_data = true;
                    out.hints.push(NO_RAW_DATA.into());
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

    /// The images of the Stelle `faces` baked anew as `variant`, onto the surface as the steps
    /// so far left it.
    fn bake(&self, faces: &[u32], variant: u32, progress: &Progress) -> Result<Baked, StepError> {
        let Some(sources) = self.sources else {
            return Ok(Baked::NoRawData);
        };
        let current = apply_patches(self.mesh, &self.s, &self.patches);
        match retexture_region(
            &self.s, &current, faces, sources, variant, self.cfg, progress,
        ) {
            Ok(done) => Ok(Baked::Done(done)),
            Err(e) => refusal(e).map(Baked::Refused),
        }
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

/// Replays the steps of the Stellen of the map or model in [`TouchupRequest::owner_dir`] on the
/// surface in [`TouchupRequest::surface_dir`] and keeps the patches in its [`TOUCHUP_DIR`] (see
/// the module docs). What the cache holds of the same steps is kept: if it fits the Stellen
/// ([`touchup_fits`]), nothing is computed or written. Without a step on any Stelle an older
/// cache goes. Progress: the phase „Nacharbeit: Oberfläche lesen“ (with the reading of the glb),
/// then „Nacharbeit: Schritt i von n“ per step computed, with the phases of baking images anew
/// within; a cancel is checked between the steps and within them and leaves the cache as it
/// was.
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
    let Some(fingerprint) = stepped_fingerprint(&marks) else {
        // Nothing to keep: an older cache goes.
        store(dir, None)?;
        return Ok(TouchupDone::default());
    };
    let events = events(&marks);
    let n = events.len();

    // What the cache holds of these steps: the same steps up to there, none computed without
    // the raw data that are here now.
    let cache = dir.join(TOUCHUP_DIR);
    let old = read_report(&cache).filter(|r| r.belongs_to(&key) && patches_intact(&cache, r));
    let mut steps = Vec::new();
    let mut patches = Vec::new();
    if let Some(old) = old {
        let kept = old
            .steps
            .iter()
            .zip(&events)
            .take_while(|(r, e)| r.key == e.key && (req.sources.is_none() || !r.without_raw_data))
            .count();
        // Read in full (checked): a damaged file is computed anew.
        let read = (kept > 0).then(|| read_patches(&cache).ok()).flatten();
        if let Some(read) = read.filter(|p| p.len() == old.steps.len()) {
            if kept == n && old.steps.len() == n && old.marks == fingerprint {
                return Ok(old.done());
            }
            steps = old.steps;
            patches = read;
            steps.truncate(kept);
            patches.truncate(kept);
        }
    }

    let mut reused = steps.len();
    if reused < n {
        let cfg = surface_config(&report).ok_or_else(|| {
            format!(
                "{}: Einstellungen nicht lesbar",
                dir.join(SURFACE_REPORT).display()
            )
        })?;
        progress.phase(PHASE_READ, None);
        let mesh = read_mesh(&dir.join(SURFACE_GLB), progress)?;
        check_cancel(progress)?;
        let mut s = WeldedSurface::from_mesh(&mesh);
        if !patches_fit(&s, &patches) {
            steps.clear();
            patches.clear();
            reused = 0;
        }
        for patch in &patches {
            s.apply_moves(&patch.moved);
        }
        let bvh = FaceBvh::build(&s);
        let mut replay = Replay {
            mesh: &mesh,
            s,
            bvh,
            patches,
            sources: req.sources.as_deref(),
            cfg: &cfg,
        };
        for (i, event) in events.iter().enumerate().skip(steps.len()) {
            check_cancel(progress)?;
            progress.phase(&format!("Nacharbeit: Schritt {} von {n}", i + 1), None);
            let out = replay.run(event.mark, event.step, progress)?;
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
    // Last chance to stop: from here on the cache is written.
    check_cancel(progress)?;
    let mut report = Report {
        input_id: key.input_id,
        glb_bytes: key.glb_bytes,
        edit_version: EDIT_VERSION,
        marks: fingerprint,
        patches_bytes: 0,
        steps,
    };
    store(dir, Some((&mut report, &patches)))?;
    progress.info(format!(
        "Nacharbeit: {n} Schritt(e), davon {} neu gerechnet → {}",
        n - reused,
        cache.display()
    ));
    Ok(report.done())
}

/// What killed runs left in the surface folder `dir`: everything named [`TEMP_PREFIX`]….
fn leftovers(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    entries
        .flatten()
        .filter(|e| {
            e.file_name()
                .to_str()
                .is_some_and(|n| n.starts_with(TEMP_PREFIX))
        })
        .map(|e| e.path())
        .collect()
}

/// Removes the file or folder `path` as far as it goes; what stays, the next run removes.
fn remove(path: &Path) {
    let _ = if fs::symlink_metadata(path).is_ok_and(|m| m.is_dir()) {
        fs::remove_dir_all(path)
    } else {
        fs::remove_file(path)
    };
}

/// Puts the cache of the surface in `surface_dir` in place: `content` (its report, whose
/// `patches_bytes` is set here, and its patches), or none. Under the lock of the cache, after
/// what killed runs left is removed (see the module docs).
fn store(
    surface_dir: &Path,
    content: Option<(&mut Report, &[SurfacePatch])>,
) -> Result<(), StepError> {
    let dest = surface_dir.join(TOUCHUP_DIR);
    let there = |p: &Path| fs::symlink_metadata(p).is_ok();
    if content.is_none() && !there(&dest) && leftovers(surface_dir).is_empty() {
        return Ok(());
    }
    let _lock = FileLock::acquire(&dest).map_err(failed)?;
    for path in leftovers(surface_dir) {
        remove(&path);
    }
    let pid = std::process::id();
    let aside = surface_dir.join(format!("{TEMP_PREFIX}{pid}-alt"));
    let unmovable = |e: std::io::Error| -> StepError {
        format!("{} nicht ersetzbar: {e}", dest.display()).into()
    };
    let Some((report, patches)) = content else {
        if there(&dest) {
            fs::rename(&dest, &aside).map_err(unmovable)?;
            remove(&aside);
        }
        return Ok(());
    };
    let new = surface_dir.join(format!("{TEMP_PREFIX}{pid}-neu"));
    let written = write_cache(&new, report, patches).and_then(|()| {
        // The old one aside, the new one in its place, then the old one away.
        let old = there(&dest);
        if old {
            fs::rename(&dest, &aside).map_err(unmovable)?;
        }
        if let Err(e) = fs::rename(&new, &dest) {
            if old {
                let _ = fs::rename(&aside, &dest);
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
    if stepped_fingerprint(&marks).is_none() {
        return Ok(Fit::Nothing);
    }
    let Some(report) = fitting(surface_dir, &marks) else {
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
