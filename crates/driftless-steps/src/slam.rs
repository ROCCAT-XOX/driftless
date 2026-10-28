//! SLAM run: odometry, submaps, loop closure. Writes `traj.tum` (corrected), `traj_odom.tum`
//! (odometry before loop closure), `map.las` (submaps at the corrected anchors, 10 cm grid),
//! `slam.json` (submaps, loop checks, graph) and `submaps.bin` (the submaps for a reuse). The
//! manual loops of `corrections.toml` in the output folder are always applied.
//!
//! A reuse skips the odometry: it loads `submaps.bin`, solves the pose graph again and keeps
//! the previous `traj.tum` as `traj.vorher.tum`. It replaces a finished run, so it writes its
//! new files beside the old ones first (`<name>.tmp`) and puts them in place together only
//! once all are written: the old ones aside (`<name>.alt.tmp`), the new ones in, `slam.json`
//! last. A failure on the way – a full disk, a file in the way, the history state that cannot
//! be recorded – puts every file back as it was, so the run keeps its result; a file that
//! cannot go back stops that, and the next run finishes it as after a kill. A reuse killed
//! while it put them in place (a crash, a power cut) leaves the run without `slam.json` or
//! `traj.vorher.tum`, its old files aside: the next run puts them back first
//! ([`reuse_cut_off`]).
//!
//! A full run into a folder that holds the run of another scan is refused: its frames (from
//! `slam.json`) lie outside the times of this scan's LiDAR bags ([`run_fits_scan`]). Its
//! corrections would be applied to this scan and its results replaced.
//!
//! A run removes the `.tmp` files a killed run of its files left ([`RUN_FILES`], also those a
//! killed reuse staged, and the old files it put aside once they no longer belong to the run)
//! at its start; a full run removes `slam.json` before it writes its first file. `slam.json`
//! comes last again, so a folder with a `slam.json` holds the files of one finished run.

use crate::error::{check_cancel, failed, StepError};
use crate::history::{self, HistoryKind};
use crate::leftovers::{remove_report, remove_stale_tmp};
use crate::scan::{
    parse_extrinsic, read_imu, require_frames, warn_if_not_static, with_lidar_frames, write_las,
};
use driftless_io::atomic_write;
use driftless_io::bag::BagReader;
use driftless_io::progress::Progress;
use driftless_io::project::discover_project;
use driftless_odom::OdomConfig;
use driftless_slam::{
    build_submaps, cache_fits as cache_header_fits, load_cache, match_manual_loops,
    match_rejected_loops, save_cache, solve_with, Corrections, LoopVerdict, ManualLoop,
    RejectedLoop, SlamConfig, SlamReport, SlamResult, Submap, POSITION_ONLY, VETOED,
};
use driftless_traj::write_tum;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const CACHE_FILE: &str = "submaps.bin";
pub const CORRECTIONS_FILE: &str = "corrections.toml";
/// The report of a run, written last.
pub const SLAM_REPORT: &str = "slam.json";
/// The files a run writes into its folder (besides `verlauf/`), the report last.
pub const RUN_FILES: [&str; 5] = [
    CACHE_FILE,
    "traj.tum",
    "traj_odom.tum",
    "map.las",
    SLAM_REPORT,
];
/// The trajectory before the last reuse.
pub const PREVIOUS_TRAJ: &str = "traj.vorher.tum";
/// The files a reuse writes anew, in the order they are put in place: the report last.
const REUSE_FILES: [&str; 4] = ["traj.tum", "traj_odom.tum", "map.las", SLAM_REPORT];

/// Whether a stale `<name>.tmp` in a run folder is the run's own: a file of [`RUN_FILES`]
/// being written (`traj.tum.tmp`), staged by a reuse (`traj.tum.tmp`, `map.las.tmp.tmp` while
/// it is written) or put aside by it (`slam.json.alt.tmp`, `traj.vorher.tum.alt.tmp`).
fn owns_tmp(name: &str) -> bool {
    let file = name
        .strip_suffix(".tmp")
        .or_else(|| name.strip_suffix(".alt"))
        .unwrap_or(name);
    RUN_FILES.contains(&file) || file == PREVIOUS_TRAJ
}

/// How far (s) a run's frames may lie outside the bag times of its scan: frame stamps come
/// from the sensor, bag times from the recorder.
pub const RUN_TIME_TOLERANCE: f64 = 5.0;

/// The times of the run in `dir` from its `slam.json` (s, the clock of the bags): the start
/// of the scan and the end of the last submap; `None` without a readable `slam.json` with
/// `scan_start`.
pub fn run_times(dir: &Path) -> Option<(f64, f64)> {
    let text = fs::read_to_string(dir.join("slam.json")).ok()?;
    let report: serde_json::Value = serde_json::from_str(&text).ok()?;
    let start = report["scan_start"].as_f64()?;
    let end = report["submaps"]
        .as_array()
        .and_then(|m| m.last())
        .and_then(|m| m["t_end"].as_f64())
        .unwrap_or(0.0);
    Some((start, start + end))
}

/// The time range of the LiDAR bags of a scan (s, the clock of the bags), from their
/// indexes; `None` if a bag has none (cut off while recording) or cannot be opened.
pub fn scan_times(bags: &[PathBuf]) -> Option<(f64, f64)> {
    let mut range: Option<(f64, f64)> = None;
    for bag in bags {
        let (a, b) = BagReader::open(bag).ok()?.time_range()?;
        range = Some(range.map_or((a, b), |(lo, hi)| (lo.min(a), hi.max(b))));
    }
    range
}

/// Whether a run of the times `run` ([`run_times`]) can be of a scan of the times `scan`
/// ([`scan_times`]): within them, give or take [`RUN_TIME_TOLERANCE`].
pub fn run_fits_scan(run: (f64, f64), scan: (f64, f64)) -> bool {
    run.0 >= scan.0 - RUN_TIME_TOLERANCE && run.1 <= scan.1 + RUN_TIME_TOLERANCE
}

/// Why a full run of the scan `req.scan` (its LiDAR bags `bags`) cannot go into `req.out`:
/// the folder holds the run of another scan. `None` if it holds none, or if that cannot be
/// told (no readable `slam.json`, bags without an index).
fn foreign_run(req: &SlamRequest, bags: &[PathBuf]) -> Option<String> {
    let run = run_times(&req.out)?;
    let scan = scan_times(bags)?;
    (!run_fits_scan(run, scan)).then(|| {
        format!(
            "{} enthält den Lauf eines anderen Scans ({:.1}–{:.1} s, dieser Scan {:.1}–{:.1} \
             s) und wird nicht überschrieben; einen eigenen Zielordner wählen, z. B. {}",
            req.out.display(),
            run.0,
            run.1,
            scan.0,
            scan.1,
            req.scan.join("driftless").display()
        )
    })
}

/// Why a full run of `req` cannot go into `req.out` because the folder holds the run of
/// another scan, as [`run_slam`] refuses it (for a check before it).
pub fn other_scan_refusal(req: &SlamRequest) -> Option<String> {
    let project = discover_project(&req.scan).ok()?;
    foreign_run(req, &project.lidar_bags)
}

/// The [`RUN_FILES`] in `dir`: a full run would replace them (the CLI asks for
/// `--ueberschreiben` then).
pub fn run_files_in(dir: &Path) -> Vec<&'static str> {
    RUN_FILES
        .iter()
        .copied()
        .filter(|f| dir.join(f).is_file())
        .collect()
}

/// Settings of the odometry that a run depends on (the same for `slam` and `map`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SlamSettings {
    /// LiDAR pose in the IMU frame: x y z qx qy qz qw; identity without.
    pub lidar_to_imu: Option<[f64; 7]>,
    /// PointCloud2 topic; without, the only one in the bags.
    pub lidar_topic: Option<String>,
    /// Imu topic; without, the only one in the bags.
    pub imu_topic: Option<String>,
}

impl SlamSettings {
    /// The SLAM configuration of these settings, exactly as `driftless slam` runs it.
    pub fn config(&self) -> Result<SlamConfig, StepError> {
        Ok(SlamConfig {
            odom: OdomConfig {
                lidar_to_imu: parse_extrinsic(self.lidar_to_imu.as_ref().map(|v| &v[..]))?,
                ..OdomConfig::default()
            },
            ..SlamConfig::default()
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SlamRequest {
    /// Scan folder (with LIDAR_*.bag).
    pub scan: PathBuf,
    /// Output folder.
    pub out: PathBuf,
    pub settings: SlamSettings,
    /// Only recompute from `submaps.bin`.
    pub reuse: bool,
    /// Keep the result as a state in `verlauf/` (see [`crate::history`]).
    pub history: bool,
}

#[derive(Clone, Debug)]
pub struct SlamOutcome {
    pub report: SlamReport,
    pub map_points: usize,
    pub seconds: f64,
    /// The state written to `verlauf/`, with [`SlamRequest::history`].
    pub history: Option<PathBuf>,
}

/// What the cache depends on: names and sizes of the LiDAR bags, the chosen topics and the
/// settings.
fn input_id(bags: &[PathBuf], settings: &SlamSettings, cfg: &SlamConfig) -> Result<String, String> {
    let mut id = String::new();
    for bag in bags {
        let name = bag
            .file_name()
            .map(|n| n.to_string_lossy())
            .unwrap_or_default();
        let len = fs::metadata(bag)
            .map_err(|e| format!("{} nicht lesbar: {e}", bag.display()))?
            .len();
        id += &format!("{name} {len}\n");
    }
    id += &format!(
        "lidar_topic {:?}\nimu_topic {:?}\n{cfg:?}\n",
        settings.lidar_topic, settings.imu_topic
    );
    Ok(id)
}

/// Whether the `submaps.bin` in `req.out` was written for this scan and these settings (so a
/// reuse can load it); reads only its header.
pub fn cache_fits(req: &SlamRequest) -> bool {
    let fits = || -> Option<bool> {
        let cfg = req.settings.config().ok()?;
        let project = discover_project(&req.scan).ok()?;
        let input = input_id(&project.lidar_bags, &req.settings, &cfg).ok()?;
        Some(cache_header_fits(&req.out.join(CACHE_FILE), &input))
    };
    fits().unwrap_or(false)
}

/// Runs SLAM as `driftless slam` does; every console line goes to `progress` in the same
/// order. Phases: „IMU lesen“, „LiDAR lesen“ (or „Zwischenspeicher laden“ for a reuse),
/// „Schleifen schließen“, „Schreiben“.
///
/// A cancel is honoured while the frames are read and right after them, before
/// `submaps.bin` is written (full run), and for a reuse until its new files are written,
/// before they are put in place. A cancelled run leaves the output folder as it was (a full
/// run into a new folder leaves it empty). From the replaced `submaps.bin` on a full run is
/// not interruptible and finishes, also when cancelled meanwhile. A reuse that fails leaves
/// the folder as it was as well (see the module documentation).
///
/// A full run into a folder with the run of another scan is refused before anything is read
/// (see the module documentation).
pub fn run_slam(req: &SlamRequest, progress: &Progress) -> Result<SlamOutcome, StepError> {
    let start = Instant::now();
    let out = &req.out;
    let cfg = req.settings.config()?;
    let project = discover_project(&req.scan).map_err(failed)?;
    // the run a reuse was cut off in first: what is checked and kept below is that run
    put_back(out, progress)?;
    if !req.reuse {
        if let Some(why) = foreign_run(req, &project.lidar_bags) {
            return Err(why.into());
        }
    }
    remove_stale_tmp(out, owns_tmp, progress);
    let input = input_id(&project.lidar_bags, &req.settings, &cfg)?;
    let cache = out.join(CACHE_FILE);
    let corrections = Corrections::load(&out.join(CORRECTIONS_FILE))?;
    let manual = corrections.manual_loops();
    // automatic loops switched off by hand stay out of the pose graph
    let rejected = corrections.rejected_loops();
    // A bad correction fails only in `solve`; the cache is written by then. The CLI and the
    // app each add their own hint how to switch it off.
    let solve_hint = |e: driftless_slam::ManualLoopError| StepError::BadCorrection(e.to_string());
    check_cancel(progress)?;

    if req.reuse {
        progress.phase("Zwischenspeicher laden", None);
        let (submaps, odom_anchors) = load_cache(&cache, &input).map_err(failed)?;
        // A result the history does not hold (a run made without history, e.g. by the CLI, or
        // one recomputed by the CLI after the last state) is kept before it is replaced, with
        // the corrections it was computed with.
        let import = (req.history && out.join("traj.tum").exists() && !history::holds_current(out))
            .then(|| corrections_of_run(out, &submaps, &corrections));
        progress.phase("Schleifen schließen", None);
        let result =
            solve_with(submaps, odom_anchors, &manual, &rejected, &cfg).map_err(solve_hint)?;
        check_cancel(progress)?;
        progress.info(format!(
            "Zwischenspeicher {CACHE_FILE}: {} Teilkarten, {} manuelle Schleifen",
            result.submaps.len(),
            manual.len()
        ));
        progress.phase("Schreiben", None);
        // the new files beside the old ones; removed again unless they are put in place
        let staged = Staged::write(out, &result)?;
        // The last check: recording, putting in place and recording again belong together.
        check_cancel(progress)?;
        if let Some(then) = &import {
            history::record_with(out, HistoryKind::Imported, 0.0, Some(then))
                .map_err(|e| unchanged(&e))?;
        }
        let mut swap = staged.put_in_place()?;
        let seconds = start.elapsed().as_secs_f64();
        // the corrections read at the start: a change written meanwhile was not applied; if
        // the state cannot be recorded, `swap` puts every file back (and `staged` removes the
        // new ones)
        let history = if req.history {
            Some(
                history::record_with(out, HistoryKind::Reuse, seconds, Some(&corrections))
                    .map_err(|e| swap.undone(&e))?,
            )
        } else {
            None
        };
        swap.keep();
        print_summary(&result, &staged.report, staged.points, progress);
        progress.info(format!("Laufzeit {seconds:.1} s → {}", out.display()));
        return Ok(SlamOutcome {
            report: staged.report.clone(),
            map_points: staged.points,
            seconds,
            history,
        });
    }

    fs::create_dir_all(out).map_err(failed)?;
    let (imu, bad) = read_imu(
        &project.lidar_bags,
        req.settings.imu_topic.as_deref(),
        progress,
    )?;
    let ((submaps, stats), read) = with_lidar_frames(
        &project.lidar_bags,
        req.settings.lidar_topic.as_deref(),
        progress,
        |frames| build_submaps(frames, &imu, &cfg),
    )?;
    // After a cancel the frames stopped early: the result is incomplete.
    check_cancel(progress)?;
    require_frames(&stats)?;
    let odom_anchors: Vec<_> = submaps.iter().map(|m| m.anchor).collect();
    remove_report(&out.join(SLAM_REPORT))?;
    save_cache(&cache, &submaps, &odom_anchors, &input)
        .map_err(|e| format!("Zwischenspeicher {} nicht schreibbar: {e}", cache.display()))?;
    progress.phase("Schleifen schließen", None);
    let result = solve_with(submaps, odom_anchors, &manual, &rejected, &cfg).map_err(solve_hint)?;
    warn_if_not_static(&stats, progress);
    progress.info(format!(
        "{} Posen aus {} LiDAR-Frames ({} übersprungen), {} IMU-Messungen, \
         {} nicht lesbare Nachrichten",
        stats.frames_used,
        stats.frames_in,
        stats.frames_skipped,
        imu.len(),
        bad + read.bad
    ));

    progress.phase("Schreiben", None);
    let report = SlamReport::new(&result);
    let map_points = write_files(&result, &report, |name| out.join(name))?;
    print_summary(&result, &report, map_points, progress);
    let seconds = start.elapsed().as_secs_f64();
    progress.info(format!("Laufzeit {seconds:.1} s → {}", out.display()));
    let history = if req.history {
        // the corrections read at the start: a change written meanwhile was not applied
        Some(history::record_with(
            out,
            HistoryKind::Full,
            seconds,
            Some(&corrections),
        )?)
    } else {
        None
    };
    Ok(SlamOutcome {
        report,
        map_points,
        seconds,
        history,
    })
}

/// A reuse that failed before its new files were all in place: every file is back as it was.
fn unchanged(e: &StepError) -> StepError {
    match e {
        StepError::Cancelled => StepError::Cancelled,
        e => StepError::Failed(format!("{e}; nichts geändert, der Lauf ist wie vorher")),
    }
}

/// `<name>.tmp` in `out`: a new file of a reuse, written but not yet in place.
fn staged_path(out: &Path, name: &str) -> PathBuf {
    out.join(format!("{name}.tmp"))
}

/// `<name>.alt.tmp` in `out`: an old file a reuse put aside while it puts the new ones in.
fn aside_path(out: &Path, name: &str) -> PathBuf {
    out.join(format!("{name}.alt.tmp"))
}

/// Whether a reuse was cut off in `dir` while it put its new files in place (a crash, a power
/// cut) and left old files of the run aside that the next run puts back ([`put_back`]): the
/// report is missing and lies aside (`slam.json.alt.tmp`), or the trajectory before the last
/// reuse does (`traj.vorher.tum.alt.tmp`). Until then its `.tmp` files are no leftovers.
pub fn reuse_cut_off(dir: &Path) -> bool {
    put_aside(dir, SLAM_REPORT) || put_aside(dir, PREVIOUS_TRAJ)
}

/// Whether the file `name` of the run in `dir` is missing and lies aside (`<name>.alt.tmp`).
fn put_aside(dir: &Path, name: &str) -> bool {
    !dir.join(name).exists() && aside_path(dir, name).is_file()
}

/// Puts back what a reuse cut off while it put its new files in place left aside
/// ([`reuse_cut_off`]), so that the run is as before that reuse: with the report aside, the
/// old trajectory comes back from [`PREVIOUS_TRAJ`] (over a new one in place), the other old
/// files from `<name>.alt.tmp`, the report last; then the trajectory before the last reuse,
/// if it lies aside and none is in place. What the reuse staged stays as `<name>.tmp`, a stale
/// file of the run. A rename that fails stops the run before anything is removed: the old
/// files stay aside for the next start.
fn put_back(out: &Path, progress: &Progress) -> Result<(), StepError> {
    let file = |name: &str| out.join(name);
    let back = |from: PathBuf, to: PathBuf| -> Result<(), StepError> {
        fs::rename(&from, &to).map_err(|e| {
            StepError::Failed(format!(
                "{} nicht in {} zurückbenennbar ({e}): der Lauf vor dem unterbrochenen \
                 Neu-Rechnen liegt noch als *.alt.tmp in {}; nichts gerechnet",
                from.display(),
                to.display(),
                out.display()
            ))
        })
    };
    let mut names = Vec::new();
    if put_aside(out, SLAM_REPORT) {
        // the trajectory before the last reuse went aside before the report: what is in its
        // place now is the old trajectory
        if file(PREVIOUS_TRAJ).is_file() {
            back(file(PREVIOUS_TRAJ), file("traj.tum"))?;
            names.push("traj.tum");
        }
        for name in ["traj_odom.tum", "map.las", SLAM_REPORT] {
            if aside_path(out, name).is_file() {
                back(aside_path(out, name), file(name))?;
                names.push(name);
            }
        }
    }
    if put_aside(out, PREVIOUS_TRAJ) {
        back(aside_path(out, PREVIOUS_TRAJ), file(PREVIOUS_TRAJ))?;
        names.push(PREVIOUS_TRAJ);
    }
    if !names.is_empty() {
        progress.info(format!(
            "Das letzte Neu-Rechnen von {} wurde unterbrochen: den Lauf davor zurückgeholt ({})",
            out.display(),
            names.join(", ")
        ));
    }
    Ok(())
}

/// The new files of a reuse ([`REUSE_FILES`]) written beside the run's own as `<name>.tmp`,
/// each complete on the disk. Dropped, they are removed: those not put in place, or put back
/// by [`Swap`].
struct Staged<'a> {
    out: &'a Path,
    report: SlamReport,
    points: usize,
}

impl<'a> Staged<'a> {
    /// Writes them; on an error nothing of them stays, and the run is as it was.
    fn write(out: &'a Path, result: &SlamResult) -> Result<Staged<'a>, StepError> {
        let mut staged = Staged {
            out,
            report: SlamReport::new(result),
            points: 0,
        };
        staged.points = write_files(result, &staged.report, |name| staged_path(out, name))
            .map_err(|e| unchanged(&e))?;
        Ok(staged)
    }

    /// Puts them in place of the old ones: the trajectory before the last reuse aside first
    /// ([`PREVIOUS_TRAJ`] as `<name>.alt.tmp`), the report out of the way, the other old files
    /// aside, the old trajectory to [`PREVIOUS_TRAJ`], then the new ones in, the report last.
    /// So after a kill [`put_back`] can tell where each old file is: once the report is aside,
    /// a [`PREVIOUS_TRAJ`] in place can only be the old trajectory. All or nothing: on an
    /// error what was renamed goes back.
    fn put_in_place(&self) -> Result<Swap, StepError> {
        let mut swap = Swap {
            out: self.out.to_path_buf(),
            done: Vec::new(),
            kept: false,
        };
        for (from, to) in swap_plan(self.out) {
            swap.rename(from, to)?;
        }
        Ok(swap)
    }
}

/// The renames of [`Staged::put_in_place`] in `out`, in their order (from, to).
fn swap_plan(out: &Path) -> Vec<(PathBuf, PathBuf)> {
    let file = |name: &str| out.join(name);
    let mut plan = Vec::new();
    for name in [PREVIOUS_TRAJ, SLAM_REPORT, "traj_odom.tum", "map.las"] {
        if file(name).is_file() {
            plan.push((file(name), aside_path(out, name)));
        }
    }
    if file("traj.tum").is_file() {
        plan.push((file("traj.tum"), file(PREVIOUS_TRAJ)));
    }
    for name in REUSE_FILES {
        plan.push((staged_path(out, name), file(name)));
    }
    plan
}

impl Drop for Staged<'_> {
    fn drop(&mut self) {
        for name in REUSE_FILES {
            let _ = fs::remove_file(staged_path(self.out, name));
        }
    }
}

/// The renames that put the new files of a reuse in place ([`Staged::put_in_place`]). Dropped
/// without [`Swap::keep`] they are undone ([`Swap::undo`]): every file is back as it was.
struct Swap {
    /// The run folder.
    out: PathBuf,
    /// (from, to) of every rename done.
    done: Vec<(PathBuf, PathBuf)>,
    kept: bool,
}

impl Swap {
    fn rename(&mut self, from: PathBuf, to: PathBuf) -> Result<(), StepError> {
        if let Err(e) = fs::rename(&from, &to) {
            return Err(self.undone(&StepError::Failed(format!(
                "{} nicht in {} umbenennbar: {e}",
                from.display(),
                to.display()
            ))));
        }
        self.done.push((from, to));
        Ok(())
    }

    /// Undoes the renames done, newest first. It stops at the first that fails, so that what
    /// is left is a reuse cut off after its first renames, which [`put_back`] finishes at the
    /// next start; going on would put files back over others that are not yet back (the old
    /// trajectory in [`PREVIOUS_TRAJ`]). Whether every rename was undone.
    fn undo(&mut self) -> bool {
        while let Some((from, to)) = self.done.pop() {
            if fs::rename(&to, &from).is_err() {
                self.done.clear();
                return false;
            }
        }
        true
    }

    /// `e` after undoing the renames ([`Swap::undo`]), saying where the old run is.
    fn undone(&mut self, e: &StepError) -> StepError {
        if self.undo() {
            unchanged(e)
        } else {
            StepError::Failed(format!(
                "{e}; der Lauf davor liegt noch zum Teil als *.alt.tmp in {}: der nächste Start \
                 holt ihn zurück",
                self.out.display()
            ))
        }
    }

    /// Keeps the new files; the old ones put aside go.
    fn keep(mut self) {
        self.kept = true;
        for (_, to) in &self.done {
            let aside = to
                .file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with(".alt.tmp"));
            if aside {
                let _ = fs::remove_file(to);
            }
        }
    }
}

impl Drop for Swap {
    fn drop(&mut self) {
        if !self.kept {
            self.undo();
        }
    }
}

/// The corrections of `current` with the switches of the run whose `slam.json` lies in
/// `out`: an entry is on iff its two submaps are joined by a manual loop there. A pair takes
/// one loop, so only one entry of a pair is on: the first entry that is on now, else the
/// first one (an entry switched off since the run). An entry added after `slam.json` was
/// written cannot have been used and stays off. Without a readable `slam.json` all are off.
fn corrections_of_run(out: &Path, submaps: &[Submap], current: &Corrections) -> Corrections {
    let json = out.join("slam.json");
    // as seconds since 1970, like a parsed `added`
    let written = fs::metadata(&json)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| d.as_secs_f64());
    let report = fs::read_to_string(&json)
        .ok()
        .and_then(|t| serde_json::from_str::<SlamReport>(&t).ok());
    let pairs_where = |keep: &dyn Fn(&driftless_slam::report::LoopReport) -> bool| {
        report.as_ref().map_or_else(BTreeSet::new, |r| {
            r.loops
                .iter()
                .filter(|l| keep(l))
                .map(|l| (l.a.min(l.b), l.a.max(l.b)))
                .collect::<BTreeSet<(usize, usize)>>()
        })
    };
    let pairs = pairs_where(&|l| l.manual);
    // the automatic loops that run left out by hand
    let vetoed = pairs_where(&|l| !l.manual && l.reason.as_deref() == Some(VETOED));
    let mut used = BTreeSet::new();
    let mut then = current.clone();
    let pair_of = |l: &driftless_slam::LoopEntry| {
        match_manual_loops(submaps, &[ManualLoop { a_t: l.a, b_t: l.b }])
            .ok()
            .and_then(|m| m.first().map(|m| (m.a.min(m.b), m.a.max(m.b))))
    };
    let entries: Vec<_> = then.loops.iter().map(|l| (pair_of(l), l.enabled)).collect();
    // entries that are on now claim their pair first
    for claim_enabled in [true, false] {
        for (l, (pair, on)) in then.loops.iter_mut().zip(&entries) {
            if *on == claim_enabled {
                // an unreadable `added` (a hand edit) does not rule the entry out
                let added = crate::correct::parse_stamp(&l.added);
                let later = matches!((added, written), (Some(a), Some(w)) if a > w.floor());
                l.enabled = !later && pair.is_some_and(|p| pairs.contains(&p) && used.insert(p));
            }
        }
    }
    // a reject counts if that run left its loop out
    then.rejects.retain(|r| {
        match_rejected_loops(submaps, &[RejectedLoop { a_t: r.a, b_t: r.b }])
            .ok()
            .is_some_and(|p| vetoed.contains(&p[0]))
    });
    then
}

/// Writes every file of `result` but the cache, each through a `.tmp`, [`SLAM_REPORT`] last,
/// to `path(name)`; returns the number of map points.
fn write_files(
    result: &SlamResult,
    report: &SlamReport,
    path: impl Fn(&str) -> PathBuf,
) -> Result<usize, StepError> {
    let write = || -> Result<usize, Box<dyn std::error::Error>> {
        write_tum(&path("traj.tum"), &result.poses())?;
        write_tum(&path("traj_odom.tum"), &result.odom_poses())?;
        let map = result.map_points();
        write_las(&path("map.las"), &map)?;
        let report_path = path(SLAM_REPORT);
        atomic_write(&report_path, crate::report_text(report)?.as_bytes())
            .map_err(|e| format!("{} nicht schreibbar: {e}", report_path.display()))?;
        Ok(map.len())
    };
    Ok(write()?)
}

fn print_summary(result: &SlamResult, report: &SlamReport, points: usize, progress: &Progress) {
    for (l, r) in result.loops.iter().zip(&report.loops) {
        if let LoopVerdict::PositionOnly(why) = l.verdict {
            let (a, b) = (&report.submaps[l.a], &report.submaps[l.b]);
            progress.warn(format!(
                "manuelle Schleife Teilkarte {} ({:.1}–{:.1} s) ↔ {} ({:.1}–{:.1} s): \
                 {POSITION_ONLY} ({why}; Anteil passender Punkte {:.2})",
                l.a, a.t_start, a.t_end, l.b, b.t_start, b.t_end, r.inlier_ratio
            ));
        }
    }
    let accepted = result
        .loops
        .iter()
        .filter(|l| !matches!(l.verdict, LoopVerdict::Rejected(_)))
        .count();
    let rejected = result.loops.len() - accepted;
    let manual = result.loops.iter().filter(|l| l.manual).count();
    let vetoed = result
        .loops
        .iter()
        .filter(|l| l.verdict == LoopVerdict::Rejected(VETOED))
        .count();
    let mut loops = if accepted == 0 {
        format!("keine Schleife gefunden ({rejected} Kandidaten verworfen)")
    } else if manual == 0 {
        format!("Schleifen: {accepted} akzeptiert, {rejected} verworfen")
    } else {
        format!("Schleifen: {accepted} akzeptiert ({manual} manuell), {rejected} verworfen")
    };
    if vetoed > 0 {
        loops += &format!(", davon {vetoed} von Hand abgeschaltet");
    }
    progress.info(format!(
        "{} Teilkarten, {} Kartenpunkte, {loops}",
        result.submaps.len(),
        points
    ));
    progress.info(format!(
        "Start↔Ende {:.3} m → {:.3} m, Länge {:.1} m → {:.1} m",
        report.start_end_gap.before,
        report.start_end_gap.after,
        report.path_length.before,
        report.path_length.after
    ));
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_io::testdir::TestDir;
    use std::collections::BTreeMap;

    /// Every file directly in `dir` (name → contents).
    fn files(dir: &Path) -> BTreeMap<String, String> {
        fs::read_dir(dir)
            .unwrap()
            .flatten()
            .map(|e| {
                let name = e.file_name().to_string_lossy().into_owned();
                (name, fs::read_to_string(e.path()).unwrap())
            })
            .collect()
    }

    /// A finished run in `dir` (with the trajectory before an earlier reuse if `previous`) and
    /// the new files a reuse staged beside it; returns the files of the run.
    fn run_with_staged(dir: &Path, previous: bool) -> BTreeMap<String, String> {
        fs::create_dir_all(dir).unwrap();
        let mut old = vec![
            (SLAM_REPORT, "alter Bericht"),
            ("traj.tum", "alte Laufbahn"),
            ("traj_odom.tum", "alte Odometrie"),
            ("map.las", "alte Karte"),
        ];
        if previous {
            old.push((PREVIOUS_TRAJ, "Laufbahn davor"));
        }
        for (name, text) in &old {
            fs::write(dir.join(name), text).unwrap();
        }
        for name in REUSE_FILES {
            fs::write(staged_path(dir, name), format!("neu: {name}")).unwrap();
        }
        old.iter()
            .map(|(n, t)| (n.to_string(), t.to_string()))
            .collect()
    }

    /// A reuse cut off after any number of the renames that put its files in place (files
    /// only, no real kill): the start of the next run puts the run back as it was before that
    /// reuse – also the trajectory before the last reuse – and then removes what the reuse
    /// staged. Cut off after its last rename, the new run stays, as a finished reuse leaves
    /// it. A rename that fails stops it with the old files still aside; the next start
    /// finishes it.
    #[test]
    fn a_reuse_cut_off_while_swapping_is_put_back() {
        let root = TestDir::new(format!("driftless-slam-put-back-{}", std::process::id()));
        for previous in [true, false] {
            let plan = root.join(format!("plan-{previous}"));
            run_with_staged(&plan, previous);
            // the old files aside (one more with a trajectory before), the trajectory, the
            // new files in
            let n = swap_plan(&plan).len();
            assert_eq!(n, if previous { 9 } else { 8 });
            for k in 0..=n {
                let dir = root.join(format!("{previous}-{k}"));
                let old = run_with_staged(&dir, previous);
                for (from, to) in swap_plan(&dir).into_iter().take(k) {
                    fs::rename(from, to).unwrap();
                }
                assert_eq!(reuse_cut_off(&dir), 0 < k && k < n, "{previous} {k}");
                let p = Progress::new();
                put_back(&dir, &p).unwrap();
                remove_stale_tmp(&dir, owns_tmp, &p);
                if k < n {
                    assert_eq!(files(&dir), old, "{previous}: cut off after {k} of {n}");
                } else {
                    let mut new: BTreeMap<String, String> = REUSE_FILES
                        .iter()
                        .map(|f| (f.to_string(), format!("neu: {f}")))
                        .collect();
                    new.insert(PREVIOUS_TRAJ.into(), "alte Laufbahn".into());
                    assert_eq!(files(&dir), new, "{previous}: after the last rename");
                }
            }
        }

        // the old map.las cannot go back (a folder in its place): stopped, the report still
        // aside; with the folder gone the next start puts the rest back
        let dir = root.join("im-weg");
        let old = run_with_staged(&dir, true);
        for (from, to) in swap_plan(&dir).into_iter().take(4) {
            fs::rename(from, to).unwrap();
        }
        fs::create_dir_all(dir.join("map.las").join("x")).unwrap();
        let err = put_back(&dir, &Progress::new()).unwrap_err().to_string();
        assert!(
            err.contains("map.las") && err.contains("*.alt.tmp"),
            "{err}"
        );
        assert!(reuse_cut_off(&dir));
        fs::remove_dir_all(dir.join("map.las")).unwrap();
        let p = Progress::new();
        put_back(&dir, &p).unwrap();
        remove_stale_tmp(&dir, owns_tmp, &p);
        assert_eq!(files(&dir), old);
    }

    /// A rename that fails while the new files go in, and then one that fails while they go
    /// back (a folder in the way of both): the undo stops there, so the files are those of a
    /// reuse cut off after its first renames, and the error says that the next start puts the
    /// run back. Going on would put the trajectory before the last reuse back over the old
    /// trajectory. With the folder gone, the next start puts back the old run.
    #[test]
    fn an_undo_that_fails_leaves_what_the_next_start_puts_back() {
        let root = TestDir::new(format!("driftless-slam-undo-{}", std::process::id()));
        let dir = root.join("lauf");
        let old = run_with_staged(&dir, true);
        let mut swap = Swap {
            out: dir.clone(),
            done: Vec::new(),
            kept: false,
        };
        let mut err = None;
        for (from, to) in swap_plan(&dir) {
            let old_traj = from == dir.join("traj.tum");
            if let Err(e) = swap.rename(from, to) {
                err = Some(e.to_string());
                break;
            }
            if old_traj {
                // the new trajectory cannot go in, and the old one cannot come back
                fs::create_dir_all(dir.join("traj.tum").join("x")).unwrap();
            }
        }
        drop(swap);
        let err = err.expect("the new trajectory cannot go in");
        assert!(err.contains("der nächste Start holt ihn zurück"), "{err}");
        assert!(!err.contains("nichts geändert"), "{err}");
        assert!(reuse_cut_off(&dir));
        assert_eq!(
            fs::read_to_string(dir.join(PREVIOUS_TRAJ)).unwrap(),
            "alte Laufbahn"
        );
        fs::remove_dir_all(dir.join("traj.tum")).unwrap();
        let p = Progress::new();
        put_back(&dir, &p).unwrap();
        remove_stale_tmp(&dir, owns_tmp, &p);
        assert_eq!(files(&dir), old);
    }
}
