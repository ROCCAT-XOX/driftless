//! SLAM run: odometry, submaps, loop closure. Writes `traj.tum` (corrected), `traj_odom.tum`
//! (odometry before loop closure), `map.las` (submaps at the corrected anchors, 10 cm grid),
//! `slam.json` (submaps, loop checks, graph) and `submaps.bin` (the submaps for a reuse). The
//! manual loops of `corrections.toml` in the output folder are always applied.
//!
//! A reuse skips the odometry: it loads `submaps.bin`, solves the pose graph again and keeps
//! the previous `traj.tum` as `traj.vorher.tum`.
//!
//! A full run into a folder that holds the run of another scan is refused: its frames (from
//! `slam.json`) lie outside the times of this scan's LiDAR bags ([`run_fits_scan`]). Its
//! corrections would be applied to this scan and its results replaced.

use crate::error::{check_cancel, failed, StepError};
use crate::history::{self, HistoryKind};
use crate::scan::{
    parse_extrinsic, read_imu, require_frames, warn_if_not_static, with_lidar_frames, write_las,
};
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
/// `submaps.bin` is written (full run), and after the pose graph, before `traj.tum` is
/// renamed to `traj.vorher.tum` (reuse). A cancelled run leaves the output folder as it was
/// (a full run into a new folder leaves it empty). From the replaced `submaps.bin` or the
/// rename on the run is not interruptible and finishes, also when cancelled meanwhile.
///
/// A full run into a folder with the run of another scan is refused before anything is read
/// (see the module documentation).
pub fn run_slam(req: &SlamRequest, progress: &Progress) -> Result<SlamOutcome, StepError> {
    let start = Instant::now();
    let out = &req.out;
    let cfg = req.settings.config()?;
    let project = discover_project(&req.scan).map_err(failed)?;
    if !req.reuse {
        if let Some(why) = foreign_run(req, &project.lidar_bags) {
            return Err(why.into());
        }
    }
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

    let result = if req.reuse {
        progress.phase("Zwischenspeicher laden", None);
        let (submaps, odom_anchors) = load_cache(&cache, &input).map_err(failed)?;
        let traj = out.join("traj.tum");
        // A result the history does not hold (a run made without history, e.g. by the CLI, or
        // one recomputed by the CLI after the last state) is kept before it is replaced, with
        // the corrections it was computed with.
        let import = (req.history && traj.exists() && !history::holds_current(out))
            .then(|| corrections_of_run(out, &submaps, &corrections));
        progress.phase("Schleifen schließen", None);
        let result =
            solve_with(submaps, odom_anchors, &manual, &rejected, &cfg).map_err(solve_hint)?;
        // The last check: renaming, writing and recording belong together.
        check_cancel(progress)?;
        if let Some(then) = &import {
            history::record_with(out, HistoryKind::Imported, 0.0, Some(then))?;
        }
        let previous = out.join("traj.vorher.tum");
        if traj.exists() {
            fs::rename(&traj, &previous).map_err(|e| {
                format!(
                    "{} nicht in {} umbenennbar: {e}",
                    traj.display(),
                    previous.display()
                )
            })?;
        }
        progress.info(format!(
            "Zwischenspeicher {CACHE_FILE}: {} Teilkarten, {} manuelle Schleifen",
            result.submaps.len(),
            manual.len()
        ));
        result
    } else {
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
        save_cache(&cache, &submaps, &odom_anchors, &input)
            .map_err(|e| format!("Zwischenspeicher {} nicht schreibbar: {e}", cache.display()))?;
        progress.phase("Schleifen schließen", None);
        let result =
            solve_with(submaps, odom_anchors, &manual, &rejected, &cfg).map_err(solve_hint)?;
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
        result
    };

    progress.phase("Schreiben", None);
    let (report, map_points) = write_outputs(out, &result)?;
    print_summary(&result, &report, map_points, progress);
    let seconds = start.elapsed().as_secs_f64();
    progress.info(format!("Laufzeit {seconds:.1} s → {}", out.display()));
    let history = if req.history {
        let kind = if req.reuse {
            HistoryKind::Reuse
        } else {
            HistoryKind::Full
        };
        // the corrections read at the start: a change written meanwhile was not applied
        Some(history::record_with(
            out,
            kind,
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

/// Writes everything but the cache; returns the report and the number of map points.
fn write_outputs(out: &Path, result: &SlamResult) -> Result<(SlamReport, usize), StepError> {
    let write = || -> Result<(SlamReport, usize), Box<dyn std::error::Error>> {
        let report = SlamReport::new(result);
        write_tum(&out.join("traj.tum"), &result.poses())?;
        write_tum(&out.join("traj_odom.tum"), &result.odom_poses())?;
        let map = result.map_points();
        write_las(&out.join("map.las"), &map)?;
        fs::write(
            out.join("slam.json"),
            serde_json::to_string_pretty(&report)? + "\n",
        )?;
        Ok((report, map.len()))
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
