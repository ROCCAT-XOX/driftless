//! Merging several scans into one model: `modell.las` (the scan of each point in
//! `point_source_id`) and `modell.json` (where every scan sits in the model and how sure that
//! is); with [`MergeRequest::ply`] (`driftless merge --ply`) also `modell.ply` (the points
//! of `modell.las` with a property `scan`).
//!
//! The model frame is the world frame of the first member (the reference). Each member's map
//! is its `clean.bin` (refused if missing or of another input), coloured from `color.las` of
//! the same folder if that holds the same points.
//!
//! 1. **Chain**: the reference is placed. As long as members are left, every one of them is
//!    aligned ([`align_pair`], on the points thinned to [`AlignConfig::fine_voxel`]) to every
//!    member placed and in the model, each pair once; the best pair places its member:
//!    a better verdict first, then agreeing edges ([`MIN_AGREEMENT`]), then the larger
//!    overlap, then the smaller numbers. `T_i = T_j · pose`. An unsure or failed member is
//!    entered with its best result; it is in the model (and an anchor for the others) only
//!    with [`MergeRequest::include_unsure`] and if not failed. A member placed surely against
//!    such an anchor, or against one placed so, is reported "unsicher" as well: its place in
//!    the model is only as sure as the anchor's.
//!
//!    Two members with switched-on point pairs ([`MergeRequest::pairs`], in either
//!    direction) are aligned through them ([`align_with_pairs`]) instead of [`align_pair`];
//!    the pairs of all such entries of the two count together, an entry that cannot be used
//!    ([`PointPairEntry::check`]) is left out with a warning. A sure result of point pairs
//!    goes before every sure automatic one. It is sure on the fit of the pairs and the
//!    residuals alone, not on the overlap: the user chose the place.
//! 2. **Colour**: per member in the model after the reference, the median ratio per channel
//!    between the colour of the earlier members and its own, over the 1 cm voxels both
//!    coloured ([`COLOR_VOXEL`]), clamped to [[`MIN_GAIN`], [`MAX_GAIN`]]; 1 with fewer than
//!    [`MIN_COLOR_VOXELS`] such voxels.
//! 3. **Thinning**: one point per [`MergeRequest::voxel`] cell of the model, a coloured one
//!    before an uncoloured one, then the one of the smaller member number, then the earlier
//!    one; then, with [`MergeRequest::smooth`], the surfaces of all scans are smoothed
//!    together ([`smooth_surfaces`]).
//!
//! A cancel is honoured up to the writing. `modell.las` and `modell.ply` are written through
//! a temporary file each, `modell.json` last and atomically: a folder with a report holds a
//! complete model. A folder that already holds a report is refused, nothing is overwritten;
//! the `.tmp` files a killed merge left in it are removed at the start. Every step is
//! deterministic: the same request gives the same files.
//!
//! `modell.json` names the folders of every scan twice: absolute and resolved (`scan`,
//! `slam_dir`, `map_dir`) and relative to the model folder (`scan_rel`, `slam_dir_rel`,
//! `map_dir_rel`, with `/` and `..`; `null` where none can be computed: another volume, drive
//! or share). [`read_model`] takes the relative one where that folder exists, else the
//! absolute one: a project renamed, moved or copied with its model reads the maps in its own
//! folder.

use crate::error::{check_cancel, failed, StepError};
use crate::leftovers::remove_stale_tmp;
use crate::map::{clean_stage_id, hex, map_input_for, peak_rss_bytes, CLEAN_BIN, REPORT};
use crate::pairs::PointPairEntry;
use crate::slam::SlamSettings;
use driftless_io::progress::Progress;
use driftless_io::{atomic_write, paths};
use driftless_map::{
    intensity_grays, load_stage, smooth_surfaces, stage_fits, write_las_sources, write_ply_sources,
    Cloud, SmoothConfig,
};
use driftless_odom::voxel_map::{voxel_key, FixedState, VoxelKey};
use driftless_slam::align::{
    align_pair, align_with_pairs, AlignConfig, AlignMethod, Alignment, Verdict, MIN_AGREEMENT,
    WALLS_UNMEASURABLE,
};
use driftless_traj::nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Rotation3, Translation3, UnitQuaternion, Vector3,
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::btree_map::Entry;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Part of the input id: raised whenever the merge gives another result for the same
/// request, so an older model counts as stale. Only then: the program version is no part of
/// the id (see the rule at the step versions in [`crate::map`]).
pub const MERGE_VERSION: u32 = 2;
pub const MODEL_REPORT: &str = "modell.json";
pub const MODEL_LAS: &str = "modell.las";
pub const MODEL_PLY: &str = "modell.ply";
/// Voxel of the colour comparison (m).
pub const COLOR_VOXEL: f64 = 0.01;
/// Range of a colour gain.
pub const MIN_GAIN: f32 = 0.5;
pub const MAX_GAIN: f32 = 2.0;
/// Fewest shared coloured voxels for a colour gain (else 1).
pub const MIN_COLOR_VOXELS: usize = 100;
/// A channel of the member darker than this (0–255) gives no ratio: too dark to measure.
const MIN_CHANNEL: f64 = 8.0;
/// `color.las` belongs to `clean.bin` if every point lies this close (m); LAS keeps 0.1 mm.
const COLOR_MATCH: f64 = 0.001;
/// Phase of the alignment.
const PHASE_ALIGN: &str = "Ausrichten";

/// One scan of a model: its map and what the map was made from.
#[derive(Clone, Debug, PartialEq)]
pub struct MergeMember {
    /// Scan folder (with LIDAR_*.bag).
    pub scan: PathBuf,
    /// SLAM folder: `traj.tum`.
    pub slam_dir: PathBuf,
    /// Map folder: `clean.bin`, `map.json` and, if coloured, `color.las`.
    pub map_dir: PathBuf,
    pub slam: SlamSettings,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MergeRequest {
    /// The scans; the first is the reference (its frame is the model frame).
    pub members: Vec<MergeMember>,
    /// Where `modell.las`, `modell.json` and, with [`Self::ply`], `modell.ply` go.
    pub out_dir: PathBuf,
    pub align: AlignConfig,
    /// One point per cell of this size (m) in the model.
    pub voxel: f64,
    /// Smooth the surfaces of the merged points ([`smooth_surfaces`]).
    pub smooth: bool,
    /// Take unsure members into the model as well.
    pub include_unsure: bool,
    /// Point pairs marked by hand, `a` and `b` as paths of the file system
    /// ([`crate::pairs::PointPairs::resolved`]). An entry counts if it is switched on and
    /// names the map folders ([`MergeMember::map_dir`]) of two members; others are ignored.
    pub pairs: Vec<PointPairEntry>,
    /// Also `modell.ply` (`--ply`): the same points. Not part of the input id.
    pub ply: bool,
}

/// Where a member sits in the model and how sure that is.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct ModelMember {
    /// Scan folder, absolute; as [`read_model`] gives it, the folder found (see
    /// [`Self::scan_rel`]).
    pub scan: PathBuf,
    /// [`Self::scan`] relative to the model folder, with `/` and `..`, as merged; `None`
    /// where none could be computed (another volume, drive or share) and in a model written
    /// before the field. Taken before the absolute path where its folder exists.
    #[serde(default)]
    pub scan_rel: Option<String>,
    /// SLAM folder, as [`Self::scan`].
    pub slam_dir: PathBuf,
    /// [`Self::slam_dir`] relative to the model folder, as [`Self::scan_rel`].
    #[serde(default)]
    pub slam_dir_rel: Option<String>,
    /// Map folder, as [`Self::scan`].
    pub map_dir: PathBuf,
    /// [`Self::map_dir`] relative to the model folder, as [`Self::scan_rel`].
    #[serde(default)]
    pub map_dir_rel: Option<String>,
    pub slam: SlamSettings,
    /// `T_i`: world of this scan → model, 4 × 4 row by row.
    pub transform: [f64; 16],
    /// The member it was aligned to (`None` for the reference).
    pub placed_against: Option<usize>,
    /// "bezug" | "merkmale" | "raster" | "punktpaare".
    pub method: String,
    /// "sicher" | "unsicher" | "gescheitert".
    pub verdict: String,
    /// Why it is unsure or failed (German). For a scan placed by point pairs whose walls could
    /// not be measured it also says so ([`WALLS_UNMEASURABLE`], as information: the pairs
    /// place it), even when it is sure.
    pub reason: Option<String>,
    pub overlap: f64,
    pub residual_median_mm: f64,
    pub residual_rms_mm: f64,
    /// Edge agreement ([`driftless_slam::align::PairQuality::agreement`]).
    pub agreement: f64,
    /// Residual on the walls of the worst orientation
    /// ([`driftless_slam::align::PairQuality::wall_residual_mm`]); `None` (not written) where
    /// the walls could not be measured ([`WALLS_UNMEASURABLE`]), for the reference and in a
    /// model written before the field. A model that wrote 0 for walls it could not measure
    /// reads as `Some(0.0)`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wall_residual_mm: Option<f64>,
    /// For a member placed by point pairs: their RMS distance at its pose
    /// ([`driftless_slam::align::Alignment::pair_rms_mm`]).
    #[serde(default)]
    pub pair_rms_mm: Option<f64>,
    /// Whether its points are in the model.
    pub included: bool,
    /// Colour gain per channel applied to its coloured points.
    pub color_gain: [f32; 3],
    /// Its points in the model (after thinning; 0 if not included).
    pub points: u64,
    /// `hex` of the clean stage id of its map when merged: a map computed anew since then
    /// does not fit [`Self::transform`] any more. `None` in a model written before the field.
    #[serde(default)]
    pub clean_id: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct ModelReport {
    /// [`MERGE_VERSION`].
    pub version: u32,
    /// [`merge_input_id`].
    pub input_id: String,
    pub members: Vec<ModelMember>,
    /// Points of the model.
    pub points: u64,
    pub seconds_total: f64,
    /// 0 if the platform does not report it.
    pub peak_rss_bytes: u64,
    /// [`MergeRequest::include_unsure`] of the merge. `false` in a model written before the
    /// field.
    #[serde(default)]
    pub include_unsure: bool,
    /// What the merge warned about (German, as in the log): a switched-on point pair entry
    /// that names no two of the scans or cannot be used, a `color.las` that does not fit its
    /// map. Empty in a model written before the field.
    #[serde(default)]
    pub warnings: Vec<String>,
}

/// Whether `report` was merged by this program's merge ([`MERGE_VERSION`]). A model of
/// another version is stale whatever its maps, like one whose maps were computed anew: the
/// merge would place its scans or thin its points differently now. The app marks it
/// „veraltet“, `driftless surface --model` refuses it ([`crate::surface::model_surface_fits`]
/// does not take it either).
pub fn version_fits(report: &ModelReport) -> bool {
    report.version == MERGE_VERSION
}

/// `T_i` of a member: world of its scan → model.
pub fn member_transform(m: &ModelMember) -> Isometry3<f64> {
    let h = Matrix4::from_row_slice(&m.transform);
    let r: Matrix3<f64> = h.fixed_view::<3, 3>(0, 0).into_owned();
    let rot = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&r));
    Isometry3::from_parts(Translation3::new(h[(0, 3)], h[(1, 3)], h[(2, 3)]), rot)
}

fn transform_rows(pose: &Isometry3<f64>) -> [f64; 16] {
    let h = pose.to_homogeneous();
    std::array::from_fn(|k| h[(k / 4, k % 4)])
}

/// Reads `modell.json` of the model folder `dir`, with the folders of every scan as found:
/// the relative path below `dir` where that folder exists, else the absolute one (see the
/// module documentation).
pub fn read_model(dir: &Path) -> Result<ModelReport, StepError> {
    let path = dir.join(MODEL_REPORT);
    let text = fs::read_to_string(&path)
        .map_err(|e| format!("Modell {} nicht lesbar: {e}", path.display()))?;
    let mut report: ModelReport = serde_json::from_str(&text)
        .map_err(|e| StepError::from(format!("Modell {}: {e}", path.display())))?;
    let base = canonical(dir);
    for m in &mut report.members {
        let stored = [
            (&mut m.scan, &m.scan_rel),
            (&mut m.slam_dir, &m.slam_dir_rel),
            (&mut m.map_dir, &m.map_dir_rel),
        ];
        for (abs, rel) in stored {
            if let Some(found) = rel
                .as_deref()
                .map(|r| paths::stored_path(&base, r))
                .filter(|p| p.is_dir())
            {
                *abs = found;
            }
        }
    }
    Ok(report)
}

/// `path` as `modell.json` keeps it: absolute and resolved (as given where that fails), and
/// relative to the model folder `out` (resolved) where one can be computed.
fn stored_member_path(out: &Path, path: &Path) -> (PathBuf, Option<String>) {
    let abs = canonical(path);
    let rel = paths::relative_to(out, &abs);
    (abs, rel)
}

/// Name of a scan for messages: its folder name.
fn name(m: &MergeMember) -> String {
    m.scan.file_name().map_or_else(
        || m.scan.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// The input id of the colouring of the map in `map_dir` (from its `map.json`), or "none".
fn color_part(map_dir: &Path) -> String {
    let id = fs::read_to_string(map_dir.join(REPORT))
        .ok()
        .and_then(|t| serde_json::from_str::<Value>(&t).ok())
        .and_then(|j| j["steps"]["color"]["input_id"].as_str().map(String::from));
    format!("color {}\n", id.as_deref().unwrap_or("none"))
}

/// The numbers of the two members whose map folders `e` names (`a`, then `b`), compared as
/// the file system resolves them; `None` if it names another folder or one member twice.
/// Whether it is switched on does not matter.
pub fn pair_members(req: &MergeRequest, e: &PointPairEntry) -> Option<(usize, usize)> {
    let index = |p: &Path| {
        let p = canonical(p);
        req.members.iter().position(|m| canonical(&m.map_dir) == p)
    };
    let (i, j) = (index(&e.a)?, index(&e.b)?);
    (i != j).then_some((i, j))
}

/// Point pairs of two members: (point in the first, point in the second).
type HandPairs = Vec<(Vector3<f64>, Vector3<f64>)>;

/// The point pairs that count ([`MergeRequest::pairs`]) by members (smaller, larger number),
/// those of several entries one after another in their order, and a warning (German) for
/// every switched-on entry that is left out, as if switched off: one whose map folders are
/// not those of two members ([`pair_members`]: another scan, another map of it, or a path
/// that does not resolve to the member's folder), and one of two members that cannot be used
/// ([`PointPairEntry::check`]).
fn hand_pairs(req: &MergeRequest) -> (BTreeMap<(usize, usize), HandPairs>, Vec<String>) {
    let mut out: BTreeMap<(usize, usize), HandPairs> = BTreeMap::new();
    let mut warnings = Vec::new();
    for (k, e) in req.pairs.iter().enumerate().filter(|(_, e)| e.enabled) {
        let Some((i, j)) = pair_members(req, e) else {
            warnings.push(format!(
                "Punktpaare Eintrag {}: {} und {} sind nicht zwei der gewählten Karten, nicht \
                 genutzt",
                k + 1,
                e.a.display(),
                e.b.display()
            ));
            continue;
        };
        if let Err(m) = e.check() {
            warnings.push(format!(
                "{m} ({} – {}), nicht genutzt",
                req.members[i].map_dir.display(),
                req.members[j].map_dir.display()
            ));
            continue;
        }
        let pairs = e.pairs();
        let slot = out.entry((i.min(j), i.max(j))).or_default();
        if i < j {
            slot.extend(pairs);
        } else {
            slot.extend(pairs.into_iter().map(|(p, q)| (q, p)));
        }
    }
    (out, warnings)
}

/// The point pairs of members `i` and `j` ([`hand_pairs`]) as (point in `i`, point in `j`).
fn pairs_between(
    by_hand: &BTreeMap<(usize, usize), HandPairs>,
    i: usize,
    j: usize,
) -> Option<HandPairs> {
    let pairs = by_hand.get(&(i.min(j), i.max(j)))?;
    Some(if i < j {
        pairs.clone()
    } else {
        pairs.iter().map(|&(p, q)| (q, p)).collect()
    })
}

/// Full text of the input id and the clean id of every member.
fn input_text(req: &MergeRequest) -> Result<(String, Vec<String>), StepError> {
    let mut id = format!("merge v{MERGE_VERSION}\n");
    let mut clean_ids = Vec::with_capacity(req.members.len());
    for (k, m) in req.members.iter().enumerate() {
        let input = map_input_for(&m.scan, &m.slam_dir, &m.map_dir, m.slam.clone());
        let clean = clean_stage_id(&input)?;
        id += &format!("member {k}\n{clean}{}", color_part(&m.map_dir));
        clean_ids.push(clean);
    }
    id += &format!(
        "{:?}\nvoxel {:?}\nsmooth {}\ninclude_unsure {}\n",
        req.align, req.voxel, req.smooth, req.include_unsure
    );
    for ((i, j), pairs) in hand_pairs(req).0 {
        let pairs: Vec<_> = pairs
            .iter()
            .map(|(p, q)| (p.as_slice(), q.as_slice()))
            .collect();
        id += &format!("pairs {i} {j} {pairs:?}\n");
    }
    Ok((id, clean_ids))
}

/// Input id of a merge: [`MERGE_VERSION`], the clean stage id and the colouring of every
/// member in order, the settings and the point pairs that count (by member numbers, not
/// paths).
pub fn merge_input_id(req: &MergeRequest) -> Result<String, StepError> {
    Ok(hex(&input_text(req)?.0))
}

/// The path as the file system resolves it, in its plain form ([`paths::canonical`]), else
/// as given.
fn canonical(p: &Path) -> PathBuf {
    paths::canonical(p).unwrap_or_else(|_| p.to_path_buf())
}

/// Checks the request; returns the clean id of every member and the input id.
fn check(req: &MergeRequest) -> Result<(Vec<String>, String), StepError> {
    if req.members.len() < 2 {
        return Err("Zum Zusammenführen mindestens zwei Scans wählen".into());
    }
    if req.members.len() > usize::from(u16::MAX) + 1 {
        return Err("Zu viele Scans: höchstens 65 536 in einem Modell".into());
    }
    if !(req.voxel.is_finite() && req.voxel > 0.0) {
        return Err("Rastergröße des Modells muss größer als 0 sein".into());
    }
    let mut seen = BTreeSet::new();
    for m in &req.members {
        if !seen.insert((canonical(&m.scan), canonical(&m.slam_dir))) {
            return Err(format!(
                "Derselbe Lauf ist doppelt gewählt: {} ({})",
                m.scan.display(),
                m.slam_dir.display()
            )
            .into());
        }
    }
    let (id, clean_ids) = input_text(req)?;
    for (m, clean) in req.members.iter().zip(&clean_ids) {
        if !stage_fits(&m.map_dir.join(CLEAN_BIN), clean) {
            return Err(stale_map(m));
        }
    }
    Ok((clean_ids, hex(&id)))
}

fn stale_map(m: &MergeMember) -> StepError {
    format!(
        "Karte fehlt oder ist veraltet: {} – erst Karte rechnen",
        m.map_dir.display()
    )
    .into()
}

/// Takes the colours of `color.las` in `map_dir` into `cloud` (its `clean.bin`) if it holds
/// the same points in the same order. Returns whether it did; a file that does not fit
/// gives a warning, also kept in `warnings`.
fn attach_color(
    cloud: &mut Cloud,
    member: &MergeMember,
    progress: &Progress,
    warnings: &mut Vec<String>,
) -> Result<bool, StepError> {
    let path = member.map_dir.join("color.las");
    if !path.is_file() {
        return Ok(false);
    }
    let mut unfit = |why: String| {
        let w = format!(
            "{}: {} passt nicht zur Karte ({why}), Scan bleibt ungefärbt",
            name(member),
            path.display()
        );
        progress.warn(w.clone());
        warnings.push(w);
        Ok(false)
    };
    let err = |e: las::Error| StepError::Failed(format!("{}: {e}", path.display()));
    let (mut reader, limits) =
        driftless_export::las::open_las(&path).map_err(|e| StepError::Failed(e.to_string()))?;
    let header = reader.header();
    if !header.point_format().has_color {
        return unfit("keine Farben".into());
    }
    let total = header.number_of_points();
    if total != cloud.len() as u64 {
        return unfit(format!("{total} statt {} Punkte", cloud.len()));
    }
    let mut rgb = Vec::with_capacity(cloud.len());
    let mut colored = Vec::with_capacity(cloud.len());
    let mut batch = las::PointDataBuilder::new()
        .for_header(reader.header())
        .build();
    loop {
        check_cancel(progress)?;
        if reader.fill_points(limits.batch, &mut batch).map_err(err)? == 0 {
            break;
        }
        for p in batch.points() {
            let p = p.map_err(err)?;
            let i = rgb.len();
            let Some(q) = cloud.xyz.get(i) else {
                return unfit("mehr Punkte als angegeben".into());
            };
            let far = [p.x, p.y, p.z]
                .iter()
                .zip(q)
                .any(|(a, b)| (a - f64::from(*b)).abs() > COLOR_MATCH);
            if far {
                return unfit(format!("Punkt {i} liegt anders"));
            }
            let c = p.color.unwrap_or_default();
            rgb.push([c.red, c.green, c.blue].map(|v| (v >> 8) as u8));
            colored.push(p.user_data == 0);
        }
    }
    if rgb.len() != cloud.len() {
        return unfit(format!("{} statt {} Punkte", rgb.len(), cloud.len()));
    }
    cloud.rgb = Some(rgb);
    cloud.colored = Some(colored);
    Ok(true)
}

/// The points of `xyz`, the first of each `voxel` cell, in f64.
fn thinned(xyz: &[[f32; 3]], voxel: f64) -> Vec<Vector3<f64>> {
    let mut seen: HashSet<VoxelKey, FixedState> = HashSet::default();
    xyz.iter()
        .map(|p| Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2])))
        .filter(|p| seen.insert(voxel_key(p, voxel)))
        .collect()
}

/// Where a member was placed.
struct Placement {
    /// `T_i`.
    pose: Isometry3<f64>,
    against: Option<usize>,
    /// `None` for the reference.
    alignment: Option<Alignment>,
    included: bool,
    /// The nearest member on the way to the reference that is placed unsurely (only with
    /// [`MergeRequest::include_unsure`] an anchor), if any.
    unsure_anchor: Option<usize>,
}

/// Rank of an alignment in the chain; larger is better.
fn rank(a: &Alignment) -> (u8, bool, bool, f64) {
    let verdict = match a.verdict {
        Verdict::Sure => 2,
        Verdict::Unsure(_) => 1,
        Verdict::Failed(_) => 0,
    };
    (
        verdict,
        a.verdict == Verdict::Sure && a.method == AlignMethod::PointPairs,
        a.quality.agreement >= MIN_AGREEMENT,
        a.quality.overlap,
    )
}

fn better(a: &Alignment, b: &Alignment) -> bool {
    let (ra, rb) = (rank(a), rank(b));
    (ra.0, ra.1, ra.2)
        .cmp(&(rb.0, rb.1, rb.2))
        .then(ra.3.total_cmp(&rb.3))
        .is_gt()
}

/// Step 1 of the module documentation on the thinned points `pts` of the members, with the
/// point pairs that count ([`hand_pairs`]).
fn chain(
    req: &MergeRequest,
    pts: &[Vec<Vector3<f64>>],
    by_hand: &BTreeMap<(usize, usize), HandPairs>,
    progress: &Progress,
) -> Result<Vec<Placement>, StepError> {
    let n = pts.len();
    let mut placed: Vec<Option<Placement>> = (0..n).map(|_| None).collect();
    placed[0] = Some(Placement {
        pose: Isometry3::identity(),
        against: None,
        alignment: None,
        included: true,
        unsure_anchor: None,
    });
    let mut cache: BTreeMap<(usize, usize), Alignment> = BTreeMap::new();
    // Every pair is aligned at most once, each open member against each anchor: n (n − 1) / 2
    // when every member is in the model, fewer otherwise (the rest is counted at the end).
    let total = (n * (n - 1) / 2) as u64;
    progress.phase(PHASE_ALIGN, Some(total));
    let mut done = 0u64;
    loop {
        let open: Vec<usize> = (0..n).filter(|&i| placed[i].is_none()).collect();
        if open.is_empty() {
            break;
        }
        let anchors: Vec<usize> = (0..n)
            .filter(|&j| placed[j].as_ref().is_some_and(|p| p.included))
            .collect();
        let mut best: Option<(usize, usize)> = None;
        for &i in &open {
            for &j in &anchors {
                if let Entry::Vacant(slot) = cache.entry((i, j)) {
                    check_cancel(progress)?;
                    let start = Instant::now();
                    let a = match pairs_between(by_hand, i, j) {
                        Some(pairs) => {
                            align_with_pairs(&pts[i], &pts[j], &pairs, &req.align, progress)
                        }
                        None => align_pair(&pts[i], &pts[j], &req.align, progress),
                    };
                    // A cancelled alignment comes back as failed.
                    check_cancel(progress)?;
                    let (mi, mj) = (&req.members[i], &req.members[j]);
                    progress.info(format!(
                        "{} an {}: {} ({}), Überlappung {:.0} %, Restfehler {:.1} mm, {:.1} s",
                        name(mi),
                        name(mj),
                        a.verdict.word(),
                        method_word(a.method),
                        100.0 * a.quality.overlap,
                        a.quality.residual_median_mm,
                        start.elapsed().as_secs_f64()
                    ));
                    slot.insert(a);
                    progress.advance(1);
                    done += 1;
                }
                let a = &cache[&(i, j)];
                if best.is_none_or(|b| better(a, &cache[&b])) {
                    best = Some((i, j));
                }
            }
        }
        let (i, j) = best.expect("the reference is always an anchor");
        let a = cache[&(i, j)].clone();
        let included = match a.verdict {
            Verdict::Sure => true,
            Verdict::Unsure(_) => req.include_unsure,
            Verdict::Failed(_) => false,
        };
        let anchor = placed[j].as_ref().expect("anchor is placed");
        let pose = anchor.pose * a.pose;
        let anchor_unsure = anchor
            .alignment
            .as_ref()
            .is_some_and(|b| b.verdict != Verdict::Sure);
        let unsure_anchor = if anchor_unsure {
            Some(j)
        } else {
            anchor.unsure_anchor
        };
        placed[i] = Some(Placement {
            pose,
            against: Some(j),
            alignment: Some(a),
            included,
            unsure_anchor,
        });
    }
    progress.advance(total.saturating_sub(done));
    Ok(placed.into_iter().map(|p| p.expect("all placed")).collect())
}

fn method_word(m: AlignMethod) -> &'static str {
    match m {
        AlignMethod::Features => "merkmale",
        AlignMethod::Grid => "raster",
        AlignMethod::PointPairs => "punktpaare",
    }
}

/// Moves the points of `cloud` by `pose`.
fn move_cloud(cloud: &mut Cloud, pose: &Isometry3<f64>) {
    if *pose == Isometry3::identity() {
        return;
    }
    for p in &mut cloud.xyz {
        let q = pose * Point3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
        *p = [q.x as f32, q.y as f32, q.z as f32];
    }
}

/// Whether point `i` of `cloud` has a measured colour (all of a cloud with colours but no
/// flags do).
fn is_colored(cloud: &Cloud, i: usize) -> bool {
    cloud.rgb.is_some() && cloud.colored.as_ref().is_none_or(|c| c[i])
}

/// The coloured points of `cloud`: (index, colour).
fn colored_points(cloud: &Cloud) -> impl Iterator<Item = (usize, [u8; 3])> + '_ {
    let rgb = cloud.rgb.as_deref().unwrap_or(&[]);
    rgb.iter()
        .enumerate()
        .filter(|(i, _)| is_colored(cloud, *i))
        .map(|(i, c)| (i, *c))
}

fn color_voxel(p: &[f32; 3]) -> VoxelKey {
    let v = Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
    voxel_key(&v, COLOR_VOXEL)
}

/// Median of `v` (sorted in place); `None` if empty.
fn median(v: &mut [f64]) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    v.sort_unstable_by(f64::total_cmp);
    let m = v.len() / 2;
    Some(if v.len() % 2 == 1 {
        v[m]
    } else {
        0.5 * (v[m - 1] + v[m])
    })
}

/// Colour gain of `clouds[i]` against the coloured points of `earlier` (step 2 of the module
/// documentation); all clouds in the model frame.
fn color_gain(clouds: &[Cloud], i: usize, earlier: &[usize]) -> [f32; 3] {
    // Sum and count of the member's colours per voxel.
    let mut own: HashMap<VoxelKey, ([f64; 3], u32), FixedState> = HashMap::default();
    for (k, c) in colored_points(&clouds[i]) {
        let e = own.entry(color_voxel(&clouds[i].xyz[k])).or_default();
        for (s, v) in e.0.iter_mut().zip(c) {
            *s += f64::from(v);
        }
        e.1 += 1;
    }
    let mut refs: HashMap<VoxelKey, ([f64; 3], u32), FixedState> = HashMap::default();
    for &j in earlier {
        for (k, c) in colored_points(&clouds[j]) {
            let key = color_voxel(&clouds[j].xyz[k]);
            if own.contains_key(&key) {
                let e = refs.entry(key).or_default();
                for (s, v) in e.0.iter_mut().zip(c) {
                    *s += f64::from(v);
                }
                e.1 += 1;
            }
        }
    }
    if refs.len() < MIN_COLOR_VOXELS {
        return [1.0; 3];
    }
    let mut ratios: [Vec<f64>; 3] = Default::default();
    for (key, (sum, count)) in &refs {
        let (own_sum, own_count) = own[key];
        for ch in 0..3 {
            let mine = own_sum[ch] / f64::from(own_count);
            if mine >= MIN_CHANNEL {
                ratios[ch].push(sum[ch] / f64::from(*count) / mine);
            }
        }
    }
    ratios.map(|mut r| {
        if r.len() < MIN_COLOR_VOXELS {
            return 1.0;
        }
        median(&mut r).map_or(1.0, |g| (g as f32).clamp(MIN_GAIN, MAX_GAIN))
    })
}

fn apply_gain(cloud: &mut Cloud, gain: [f32; 3]) {
    if gain == [1.0; 3] {
        return;
    }
    let flags = cloud.colored.as_deref();
    if let Some(rgb) = cloud.rgb.as_mut() {
        for (i, c) in rgb.iter_mut().enumerate() {
            if flags.is_none_or(|f| f[i]) {
                for (v, g) in c.iter_mut().zip(gain) {
                    *v = (f32::from(*v) * g).round().clamp(0.0, 255.0) as u8;
                }
            }
        }
    }
}

/// Which points stay: one per `voxel` cell over all `clouds` (step 3 of the module
/// documentation); one flag per point of each cloud.
fn thinning(clouds: &[Cloud], voxel: f64) -> Vec<Vec<bool>> {
    // (cell, uncoloured · 2¹⁶ + member, index): sorted, the first of each cell stays.
    let total: usize = clouds.iter().map(Cloud::len).sum();
    let mut order: Vec<(VoxelKey, u32, u32)> = Vec::with_capacity(total);
    for (m, cloud) in clouds.iter().enumerate() {
        for (i, p) in cloud.xyz.iter().enumerate() {
            let v = Vector3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            let rank = (u32::from(!is_colored(cloud, i)) << 16) | m as u32;
            order.push((voxel_key(&v, voxel), rank, i as u32));
        }
    }
    order.sort_unstable();
    let mut keep: Vec<Vec<bool>> = clouds.iter().map(|c| vec![false; c.len()]).collect();
    let mut last: Option<VoxelKey> = None;
    for (key, rank, i) in order {
        if last != Some(key) {
            last = Some(key);
            keep[(rank & 0xffff) as usize][i as usize] = true;
        }
    }
    keep
}

/// The kept points of all clouds in member order, with the member of each point. Each
/// cloud is freed once taken. Uncoloured points of a coloured model get the gray of
/// their intensity.
fn assemble(clouds: Vec<Cloud>, keep: &[Vec<bool>]) -> (Cloud, Vec<u16>) {
    let colored_model = clouds.iter().any(|c| c.rgb.is_some());
    let total: usize = keep.iter().map(|k| k.iter().filter(|&&x| x).count()).sum();
    let mut out = Cloud {
        xyz: Vec::with_capacity(total),
        intensity: Vec::with_capacity(total),
        t: Vec::with_capacity(total),
        frame: Vec::with_capacity(total),
        rgb: colored_model.then(|| Vec::with_capacity(total)),
        colored: colored_model.then(|| Vec::with_capacity(total)),
    };
    let mut sources = Vec::with_capacity(total);
    for (m, (mut cloud, keep)) in clouds.into_iter().zip(keep).enumerate() {
        if colored_model && cloud.rgb.is_none() {
            cloud.rgb = Some(
                intensity_grays(&cloud.intensity)
                    .into_iter()
                    .map(|g| [g; 3])
                    .collect(),
            );
            cloud.colored = Some(vec![false; cloud.len()]);
        }
        if colored_model && cloud.colored.is_none() {
            cloud.colored = Some(vec![true; cloud.len()]);
        }
        let (kept, _) = cloud.select(keep);
        drop(cloud);
        out.xyz.extend(kept.xyz);
        out.intensity.extend(kept.intensity);
        out.t.extend(kept.t);
        out.frame.extend(kept.frame);
        if let (Some(o), Some(k)) = (out.rgb.as_mut(), kept.rgb) {
            o.extend(k);
        }
        if let (Some(o), Some(k)) = (out.colored.as_mut(), kept.colored) {
            o.extend(k);
        }
        sources.resize(out.xyz.len(), m as u16);
    }
    (out, sources)
}

/// Merges the scans of `req` into one model as `driftless merge` does; every console line
/// goes to `progress`. Returns the `modell.json` written to [`MergeRequest::out_dir`].
///
/// Fails with a German message for fewer than two members, a run chosen twice, a map that
/// is missing or stale, and a folder that already holds a model. A cancel before the files
/// are written leaves the folder as it was.
pub fn run_merge(req: &MergeRequest, progress: &Progress) -> Result<ModelReport, StepError> {
    let start = Instant::now();
    check_cancel(progress)?;
    let (clean_ids, input_id) = check(req)?;
    let out = &req.out_dir;
    let report_path = out.join(MODEL_REPORT);
    if report_path.exists() {
        return Err(format!(
            "{} enthält schon ein Modell – einen neuen Ordner wählen",
            out.display()
        )
        .into());
    }
    let owns = |name: &str| [MODEL_LAS, MODEL_PLY, MODEL_REPORT].contains(&name);
    remove_stale_tmp(out, owns, progress);
    let n = req.members.len();
    let (by_hand, mut warnings) = hand_pairs(req);
    for w in &warnings {
        progress.warn(w.clone());
    }

    progress.phase("Karten laden", Some(n as u64));
    let mut clouds = Vec::with_capacity(n);
    let mut pts = Vec::with_capacity(n);
    for (m, clean) in req.members.iter().zip(&clean_ids) {
        check_cancel(progress)?;
        let (mut cloud, _) = load_stage(&m.map_dir.join(CLEAN_BIN), clean).map_err(|e| {
            progress.info(format!("{}: {e}", m.map_dir.join(CLEAN_BIN).display()));
            stale_map(m)
        })?;
        let colored = attach_color(&mut cloud, m, progress, &mut warnings)?;
        progress.info(format!(
            "{}: {} Punkte{}",
            name(m),
            cloud.len(),
            if colored {
                ", gefärbt"
            } else {
                ", ungefärbt"
            }
        ));
        pts.push(thinned(&cloud.xyz, req.align.fine_voxel));
        clouds.push(cloud);
        progress.advance(1);
    }

    let placed = chain(req, &pts, &by_hand, progress)?;
    drop(pts);
    check_cancel(progress)?;

    // Members not in the model are freed, the others moved to the model frame.
    for (cloud, p) in clouds.iter_mut().zip(&placed) {
        if p.included {
            move_cloud(cloud, &p.pose);
        } else {
            *cloud = Cloud::default();
        }
    }
    progress.phase("Farben angleichen", None);
    let mut gains = vec![[1.0f32; 3]; n];
    for i in 1..n {
        check_cancel(progress)?;
        if !placed[i].included || clouds[i].rgb.is_none() {
            continue;
        }
        let earlier: Vec<usize> = (0..i).filter(|&j| placed[j].included).collect();
        gains[i] = color_gain(&clouds, i, &earlier);
        apply_gain(&mut clouds[i], gains[i]);
    }

    check_cancel(progress)?;
    progress.phase("Ausdünnen", None);
    let keep = thinning(&clouds, req.voxel);
    let counts: Vec<u64> = keep
        .iter()
        .map(|k| k.iter().filter(|&&x| x).count() as u64)
        .collect();
    let (mut model, sources) = assemble(clouds, &keep);
    drop(keep);
    check_cancel(progress)?;
    if req.smooth {
        progress.phase("Glätten", None);
        let (xyz, stats) = smooth_surfaces(&model.xyz, &SmoothConfig::default());
        model.xyz = xyz;
        progress.info(format!(
            "Geglättet: {} Punkte um im Mittel {:.1} mm verschoben, Wanddicke {:.1} → {:.1} mm",
            stats.moved,
            1000.0 * stats.mean_shift,
            1000.0 * stats.thickness_before,
            1000.0 * stats.thickness_after
        ));
    }

    // Last chance to stop: from here on files are written.
    check_cancel(progress)?;
    progress.phase("Schreiben", None);
    fs::create_dir_all(out).map_err(|e| format!("{} nicht anlegbar: {e}", out.display()))?;
    // Each through `<name>.tmp` (the writers' own), so that no half file is left.
    write_las_sources(&out.join(MODEL_LAS), &model, &sources).map_err(failed)?;
    if req.ply {
        write_ply_sources(&out.join(MODEL_PLY), &model, &sources).map_err(failed)?;
    }
    let points = model.len() as u64;
    drop((model, sources));

    let out_abs = canonical(out);
    let members = req
        .members
        .iter()
        .zip(&placed)
        .enumerate()
        .map(|(k, (m, p))| {
            let anchor = p.unsure_anchor.map(|j| name(&req.members[j]));
            model_member(m, &out_abs, p, anchor, gains[k], counts[k], &clean_ids[k])
        })
        .collect();
    let peak = peak_rss_bytes();
    let report = ModelReport {
        version: MERGE_VERSION,
        input_id,
        members,
        points,
        seconds_total: (start.elapsed().as_secs_f64() * 1000.0).round() / 1000.0,
        peak_rss_bytes: peak.unwrap_or(0),
        include_unsure: req.include_unsure,
        warnings,
    };
    let text = crate::report_text(&report)?;
    atomic_write(&report_path, text.as_bytes())
        .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;

    for (m, p) in req.members.iter().zip(&placed) {
        if let (false, Some(a)) = (p.included, &p.alignment) {
            let why = match &a.verdict {
                Verdict::Unsure(r) | Verdict::Failed(r) => r.as_str(),
                Verdict::Sure => "",
            };
            progress.warn(format!(
                "{} ist {} ({why}) und fehlt im Modell",
                name(m),
                a.verdict.word()
            ));
        }
    }
    let ram = peak.map_or_else(
        || "Spitzen-RAM unbekannt".to_string(),
        |b| format!("Spitzen-RAM {:.2} GB", b as f64 / 1e9),
    );
    let used = placed.iter().filter(|p| p.included).count();
    progress.info(format!(
        "Modell: {points} Punkte aus {used} von {n} Scans, Laufzeit {:.1} s, {ram} → {}",
        start.elapsed().as_secs_f64(),
        out.display()
    ));
    // The report as it reads back from the file (floats lose their last bit in the text).
    serde_json::from_str(&text).map_err(failed)
}

/// The report of member `m` of the model in the folder `out` (resolved), placed at `p`;
/// `unsure_anchor` names the unsurely placed member its place depends on
/// ([`Placement::unsure_anchor`]).
fn model_member(
    m: &MergeMember,
    out: &Path,
    p: &Placement,
    unsure_anchor: Option<String>,
    gain: [f32; 3],
    points: u64,
    clean_id: &str,
) -> ModelMember {
    let (method, verdict, reason, q) = match &p.alignment {
        None => ("bezug", "sicher", None, None),
        Some(a) => {
            let (word, reason) = match (&a.verdict, unsure_anchor) {
                (Verdict::Sure, None) => (a.verdict.word(), None),
                (Verdict::Sure, Some(anchor)) => (
                    Verdict::Unsure(String::new()).word(),
                    Some(format!(
                        "sicher an {anchor} ausgerichtet, dessen Lage aber unsicher ist"
                    )),
                ),
                (Verdict::Unsure(r) | Verdict::Failed(r), _) => (a.verdict.word(), Some(r.clone())),
            };
            // Point pairs place a scan without its walls (a door, a short corridor shows none):
            // that they could not be measured is named, the verdict stays the pairs'.
            let unmeasured = a.method == AlignMethod::PointPairs
                && a.quality.wall_residual_mm.is_none()
                && !matches!(a.verdict, Verdict::Failed(_));
            let reason = match reason {
                Some(r) if unmeasured => Some(format!("{r}; {WALLS_UNMEASURABLE}")),
                None if unmeasured => Some(WALLS_UNMEASURABLE.to_string()),
                r => r,
            };
            (method_word(a.method), word, reason, Some(a))
        }
    };
    let (scan, scan_rel) = stored_member_path(out, &m.scan);
    let (slam_dir, slam_dir_rel) = stored_member_path(out, &m.slam_dir);
    let (map_dir, map_dir_rel) = stored_member_path(out, &m.map_dir);
    ModelMember {
        scan,
        scan_rel,
        slam_dir,
        slam_dir_rel,
        map_dir,
        map_dir_rel,
        slam: m.slam.clone(),
        transform: transform_rows(&p.pose),
        placed_against: p.against,
        method: method.into(),
        verdict: verdict.into(),
        reason,
        overlap: q.map_or(1.0, |a| a.quality.overlap),
        residual_median_mm: q.map_or(0.0, |a| a.quality.residual_median_mm),
        residual_rms_mm: q.map_or(0.0, |a| a.quality.residual_rms_mm),
        agreement: q.map_or(1.0, |a| a.quality.agreement),
        wall_residual_mm: q.and_then(|a| a.quality.wall_residual_mm),
        pair_rms_mm: q.and_then(|a| a.pair_rms_mm),
        included: p.included,
        color_gain: gain,
        points: if p.included { points } else { 0 },
        clean_id: Some(hex(clean_id)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(xyz: Vec<[f32; 3]>, rgb: Option<Vec<[u8; 3]>>) -> Cloud {
        let n = xyz.len();
        Cloud {
            xyz,
            intensity: vec![10.0; n],
            t: vec![0.0; n],
            frame: vec![0; n],
            colored: rgb.as_ref().map(|_| vec![true; n]),
            rgb,
        }
    }

    #[test]
    fn hand_pairs_by_member_and_direction() {
        let member = |dir: &str| MergeMember {
            scan: dir.into(),
            slam_dir: dir.into(),
            map_dir: format!("{dir}/karte").into(),
            slam: SlamSettings::default(),
        };
        let entry = |a: &str, b: &str, from: f64, n: usize| PointPairEntry {
            a: format!("{a}/karte").into(),
            b: format!("{b}/karte").into(),
            points_a: (0..n).map(|k| [from + k as f64, 0.0, 0.0]).collect(),
            points_b: (0..n).map(|k| [0.0, from + k as f64, 0.0]).collect(),
            enabled: true,
            note: String::new(),
            added: String::new(),
            unknown: Default::default(),
        };
        let mut off = entry("s1", "s0", 50.0, 3);
        off.enabled = false;
        let req = MergeRequest {
            members: vec![member("s0"), member("s1"), member("s2")],
            out_dir: "m".into(),
            align: AlignConfig::default(),
            voxel: 0.01,
            smooth: false,
            include_unsure: false,
            ply: false,
            pairs: vec![
                entry("s2", "s0", 0.0, 3),
                entry("s1", "s1", 10.0, 3),
                entry("s0", "s9", 20.0, 3),
                off,
                entry("s0", "s2", 30.0, 4),
                // Not about two members: not checked, but warned about.
                entry("s9", "s0", 40.0, 2),
            ],
        };
        let (by_hand, warnings) = hand_pairs(&req);
        // Switched on, but no two members: one member twice, a folder that is no member's.
        // The switched-off entry is left out without a word.
        assert_eq!(warnings.len(), 3, "{warnings:?}");
        for (w, (nr, a, b)) in
            warnings
                .iter()
                .zip([(2, "s1", "s1"), (3, "s0", "s9"), (6, "s9", "s0")])
        {
            assert!(
                w.starts_with(&format!(
                    "Punktpaare Eintrag {nr}: {a}/karte und {b}/karte "
                )) && w.ends_with("sind nicht zwei der gewählten Karten, nicht genutzt"),
                "{w}"
            );
        }
        assert_eq!(by_hand.keys().copied().collect::<Vec<_>>(), vec![(0, 2)]);
        let x = |v: f64| Vector3::new(v, 0.0, 0.0);
        let y = |v: f64| Vector3::new(0.0, v, 0.0);
        // Member 2 as source, member 0 as target: as the first entry was marked.
        let from_2 = pairs_between(&by_hand, 2, 0).unwrap();
        assert_eq!(from_2.len(), 7);
        assert_eq!(from_2[0], (x(0.0), y(0.0)));
        assert_eq!(from_2[3], (y(30.0), x(30.0)));
        let from_0 = pairs_between(&by_hand, 0, 2).unwrap();
        assert_eq!(from_0[0], (y(0.0), x(0.0)));
        assert_eq!(from_0[6], (x(33.0), y(33.0)));
        assert!(pairs_between(&by_hand, 0, 1).is_none());

        // A switched-on entry of two members that cannot be used is left out with a warning.
        let mut bad = req.clone();
        bad.pairs.push(entry("s1", "s2", 60.0, 2));
        let (by_hand, warnings) = hand_pairs(&bad);
        assert_eq!(by_hand.keys().copied().collect::<Vec<_>>(), vec![(0, 2)]);
        assert_eq!(warnings.len(), 4);
        assert!(
            warnings[3].contains("mindestens drei") && warnings[3].contains("s1/karte"),
            "{warnings:?}"
        );
    }

    /// The input id of a merge holds [`MERGE_VERSION`] and the stage ids of the maps, never
    /// the program version: a new release keeps every model whose maps still fit.
    #[test]
    fn input_id_holds_no_program_version() {
        use crate::map::{run_map, MapRequest, MapStep};
        use crate::slam::{run_slam, SlamRequest};
        let dir = driftless_io::testscan::tmp("steps-merge-id");
        driftless_io::testscan::room_bag(&dir, true);
        let out = dir.join("out");
        let slam = SlamRequest {
            scan: dir.clone(),
            out: out.clone(),
            settings: SlamSettings::default(),
            reuse: false,
            history: false,
        };
        run_slam(&slam, &Progress::new()).unwrap();
        let mut input = map_input_for(&dir, &out, &out, SlamSettings::default());
        input.map.step = MapStep::Clean;
        let map = MapRequest {
            input,
            out_dir: out.clone(),
            all_files: false,
        };
        run_map(&map, &Progress::new()).unwrap();
        let member = MergeMember {
            scan: dir.clone(),
            slam_dir: out.clone(),
            map_dir: out,
            slam: SlamSettings::default(),
        };
        let req = MergeRequest {
            members: vec![member.clone(), member],
            out_dir: dir.join("modell"),
            align: AlignConfig::default(),
            voxel: 0.01,
            smooth: true,
            include_unsure: false,
            pairs: Vec::new(),
            ply: false,
        };
        let (text, clean_ids) = input_text(&req).unwrap();
        assert!(!text.contains(env!("CARGO_PKG_VERSION")), "{text}");
        assert!(!text.contains("driftless"), "{text}");
        assert!(
            text.starts_with(&format!("merge v{MERGE_VERSION}\n")),
            "{text}"
        );
        assert!(text.contains(&clean_ids[0]), "{text}");
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn transform_round_trips() {
        let pose = Isometry3::from_parts(
            Translation3::new(0.8, -0.5, 0.1),
            UnitQuaternion::from_euler_angles(0.01, -0.02, 0.6),
        );
        let m = ModelMember {
            scan: "a".into(),
            scan_rel: None,
            slam_dir: "a/d".into(),
            slam_dir_rel: None,
            map_dir: "a/d".into(),
            map_dir_rel: None,
            slam: SlamSettings::default(),
            transform: transform_rows(&pose),
            placed_against: Some(0),
            method: "merkmale".into(),
            verdict: "sicher".into(),
            reason: None,
            overlap: 0.5,
            residual_median_mm: 1.0,
            residual_rms_mm: 2.0,
            agreement: 0.9,
            wall_residual_mm: Some(1.0),
            pair_rms_mm: None,
            included: true,
            color_gain: [1.0; 3],
            points: 3,
            clean_id: None,
        };
        // Row by row: the translation is the last column.
        assert_eq!(m.transform[3], 0.8);
        assert_eq!(m.transform[7], -0.5);
        assert_eq!(&m.transform[12..], &[0.0, 0.0, 0.0, 1.0]);
        let back = member_transform(&m);
        assert!((back.translation.vector - pose.translation.vector).norm() < 1e-12);
        assert!(back.rotation.angle_to(&pose.rotation) < 1e-12);
    }

    #[test]
    fn sure_against_an_unsure_anchor_is_unsure() {
        let m = MergeMember {
            scan: "s2".into(),
            slam_dir: "s2/d".into(),
            map_dir: "s2/d".into(),
            slam: SlamSettings::default(),
        };
        let quality = driftless_slam::align::PairQuality {
            overlap: 0.9,
            residual_median_mm: 2.0,
            residual_rms_mm: 3.0,
            tilt_deg: 0.1,
            agreement: 0.98,
            wall_residual_mm: Some(2.1),
        };
        let mut p = Placement {
            pose: Isometry3::identity(),
            against: Some(1),
            alignment: Some(Alignment {
                pose: Isometry3::identity(),
                method: AlignMethod::Features,
                quality,
                verdict: Verdict::Sure,
                runner_up: None,
                pair_rms_mm: None,
            }),
            included: true,
            unsure_anchor: None,
        };
        let out = Path::new("m");
        let alone = model_member(&m, out, &p, None, [1.0; 3], 5, "id");
        assert_eq!(alone.verdict, "sicher");
        assert!(alone.reason.is_none());
        assert_eq!(alone.wall_residual_mm, Some(2.1));
        p.unsure_anchor = Some(1);
        let inherited = model_member(&m, out, &p, Some("s1".into()), [1.0; 3], 5, "id");
        assert_eq!(inherited.verdict, "unsicher");
        assert!(
            inherited
                .reason
                .as_deref()
                .is_some_and(|r| r.contains("an s1")),
            "{:?}",
            inherited.reason
        );
        assert!(inherited.included);
    }

    /// Walls that cannot be measured: an automatic placement is unsure for it (the align
    /// verdict), one by point pairs keeps the verdict of the pairs and names it as information.
    #[test]
    fn unmeasurable_walls_in_the_report() {
        let m = MergeMember {
            scan: "s2".into(),
            slam_dir: "s2/d".into(),
            map_dir: "s2/d".into(),
            slam: SlamSettings::default(),
        };
        let quality = driftless_slam::align::PairQuality {
            overlap: 0.05,
            residual_median_mm: 2.0,
            residual_rms_mm: 3.0,
            tilt_deg: 0.1,
            agreement: 0.7,
            wall_residual_mm: None,
        };
        let placed = |method, verdict| Placement {
            pose: Isometry3::identity(),
            against: Some(0),
            alignment: Some(Alignment {
                pose: Isometry3::identity(),
                method,
                quality,
                verdict,
                runner_up: None,
                pair_rms_mm: (method == AlignMethod::PointPairs).then_some(12.0),
            }),
            included: true,
            unsure_anchor: None,
        };
        let out = Path::new("m");
        let member = |p: &Placement| model_member(&m, out, p, None, [1.0; 3], 5, "id");
        let by_hand = member(&placed(AlignMethod::PointPairs, Verdict::Sure));
        assert_eq!(by_hand.verdict, "sicher");
        assert_eq!(by_hand.reason.as_deref(), Some(WALLS_UNMEASURABLE));
        assert_eq!(by_hand.wall_residual_mm, None);
        let off = member(&placed(
            AlignMethod::PointPairs,
            Verdict::Unsure("Punktpaare liegen im Mittel 8.0 cm neben der Lage".into()),
        ));
        assert_eq!(off.verdict, "unsicher");
        assert_eq!(
            off.reason.as_deref(),
            Some("Punktpaare liegen im Mittel 8.0 cm neben der Lage; Wände nicht messbar")
        );
        let auto = member(&placed(
            AlignMethod::Features,
            Verdict::Unsure(WALLS_UNMEASURABLE.into()),
        ));
        assert_eq!(auto.verdict, "unsicher");
        assert_eq!(auto.reason.as_deref(), Some(WALLS_UNMEASURABLE));
        let failed = member(&placed(
            AlignMethod::PointPairs,
            Verdict::Failed("mindestens drei Punktpaare nötig".into()),
        ));
        assert_eq!(
            failed.reason.as_deref(),
            Some("mindestens drei Punktpaare nötig")
        );
        // not written; the reference has none either
        let text = serde_json::to_string(&by_hand).unwrap();
        assert!(!text.contains("wall_residual_mm"), "{text}");
        let reference = Placement {
            against: None,
            alignment: None,
            ..placed(AlignMethod::Features, Verdict::Sure)
        };
        assert_eq!(member(&reference).wall_residual_mm, None);
    }

    /// `modell.json` of before: 0 reads as 0 (that program wrote 0 also for walls it could
    /// not measure), a member without the field as not measured.
    #[test]
    fn old_wall_residuals_read() {
        let mut json = serde_json::to_value(model_member(
            &MergeMember {
                scan: "s".into(),
                slam_dir: "s/d".into(),
                map_dir: "s/d".into(),
                slam: SlamSettings::default(),
            },
            Path::new("m"),
            &Placement {
                pose: Isometry3::identity(),
                against: None,
                alignment: None,
                included: true,
                unsure_anchor: None,
            },
            None,
            [1.0; 3],
            1,
            "id",
        ))
        .unwrap();
        json["wall_residual_mm"] = 0.0.into();
        let zero: ModelMember = serde_json::from_value(json.clone()).unwrap();
        assert_eq!(zero.wall_residual_mm, Some(0.0));
        json["wall_residual_mm"] = 1.54.into();
        let measured: ModelMember = serde_json::from_value(json.clone()).unwrap();
        assert_eq!(measured.wall_residual_mm, Some(1.54));
        json.as_object_mut().unwrap().remove("wall_residual_mm");
        let missing: ModelMember = serde_json::from_value(json).unwrap();
        assert_eq!(missing.wall_residual_mm, None);
    }

    #[test]
    fn thinning_prefers_colored_then_lower_member() {
        // Member 0 uncoloured, members 1 and 2 coloured, all in the same cell; a second
        // cell only member 0 has.
        let a = cloud(vec![[0.001, 0.001, 0.001], [0.5, 0.5, 0.5]], None);
        let b = cloud(vec![[0.002, 0.002, 0.002]], Some(vec![[1, 2, 3]]));
        let c = cloud(vec![[0.003, 0.003, 0.003]], Some(vec![[4, 5, 6]]));
        let keep = thinning(&[a, b, c], 0.01);
        assert_eq!(keep, vec![vec![false, true], vec![true], vec![false]]);
    }

    #[test]
    fn assemble_marks_members_and_grays_uncolored() {
        let a = cloud(vec![[0.0; 3], [1.0; 3]], Some(vec![[9, 9, 9]; 2]));
        let b = cloud(vec![[2.0; 3]], None);
        let keep = vec![vec![false, true], vec![true]];
        let (m, sources) = assemble(vec![a, b], &keep);
        assert_eq!(m.xyz, vec![[1.0; 3], [2.0; 3]]);
        assert_eq!(sources, vec![0, 1]);
        assert_eq!(m.colored, Some(vec![true, false]));
        let rgb = m.rgb.unwrap();
        assert_eq!(rgb[0], [9, 9, 9]);
        assert!(rgb[1][0] == rgb[1][1] && rgb[1][1] == rgb[1][2], "gray");
        // Without any colour the model has none.
        let (m, _) = assemble(vec![cloud(vec![[0.0; 3]], None)], &[vec![true]]);
        assert!(m.rgb.is_none() && m.colored.is_none());
    }

    #[test]
    fn color_gain_is_median_ratio_clamped() {
        let xyz: Vec<[f32; 3]> = (0..200).map(|i| [i as f32 * 0.02, 0.0, 0.0]).collect();
        let reference = cloud(xyz.clone(), Some(vec![[100, 150, 200]; 200]));
        let darker = cloud(xyz.clone(), Some(vec![[80, 120, 40]; 200]));
        let gain = color_gain(&[reference.clone(), darker], 1, &[0]);
        assert!((gain[0] - 1.25).abs() < 1e-6 && (gain[1] - 1.25).abs() < 1e-6);
        assert_eq!(gain[2], MAX_GAIN, "5× is clamped");
        // Too few shared voxels: 1.
        let few = cloud(xyz[..50].to_vec(), Some(vec![[50, 50, 50]; 50]));
        assert_eq!(color_gain(&[reference, few], 1, &[0]), [1.0; 3]);
    }
}
