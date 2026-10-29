//! „Melden“: a bug or a wish reported from anywhere in the window, kept on this computer only.
//! Nothing is uploaded or shared; Claude reads the reports in a later session and writes what
//! came of them into the files ([`Feedback::status`], [`Feedback::answer`]), and ☰ →
//! „Meldungen …“ shows that the next time it is opened.
//!
//! **⇧⇧** ([`DoubleShift`]) opens the dialog: two taps on Shift (left or right, mixed too),
//! each pressed and let go within [`TAP_MAX`] (0.30 s), the second pressed within [`GAP_MAX`]
//! (0.40 s) after the first was let go. Any other key, any text (a capital letter typed with
//! Shift), any mouse button, the mouse wheel or a pinch and the window losing the focus in
//! between start it anew; a Shift held longer (panning with Shift, flying faster) never counts.
//!
//! **The folder** of the reports ([`default_dir`]):
//!
//! | System | Folder |
//! |---|---|
//! | macOS | `~/Library/Application Support/Driftless/Meldungen/` |
//! | Windows | `%LOCALAPPDATA%\Driftless\Meldungen\` (not eframe's `%APPDATA%`) |
//! | others | `$XDG_DATA_HOME/driftless/meldungen`, else `~/.local/share/driftless/meldungen` |
//!
//! Each report is a folder of its own, `JJJJ-MM-TT_hhmmss_fehler` or `…_wunsch` (local time of
//! [`Feedback::created`]; a second one in the same second gets `-2`, then `-3`, …), with
//!
//! - [`REPORT_FILE`] `meldung.toml`: what was reported and what came along with it (below);
//!   written last, so a folder without it is not (yet) a report;
//! - [`SCREENSHOT_FILE`] `fenster.png`: the window as it was before the dialog opened, if
//!   attached;
//! - `stellen.toml`: a copy of the Stellen of the surface shown ([`Marks`]), if attached – the
//!   report stays readable when the project is moved or deleted;
//! - [`LOG_FILE`] `protokoll.txt`: the last [`LOG_LINES`] lines of the log file
//!   (`crate::logfile`), else of the Protokoll of the window.
//!
//! **`meldung.toml`** (German keys like the files of a project; they stay as they are):
//!
//! ```toml
//! art = "fehler"                       # „fehler“ or „wunsch“
//! erstellt = "2026-10-29T10:14:03+01:00"  # RFC 3339, local time with its offset
//! stand = "offen"                      # plain text: „offen“, „erledigt in 0.2.0“, …
//! antwort = ""                         # plain text: what came of it
//! text = "Decke flimmert beim Drehen"  # what the user wrote
//! version = "Driftless 0.1.0 (abc123def)"
//! system = "macOS 27.0 (aarch64)"
//! grafik = "Apple M4 Pro (Metal, IntegratedGpu, …)"  # empty if unknown
//! schritt = "Bilder"                   # the step shown: „Daten“, „Bilder“, „Planung“
//! stelle = 1                           # the Stelle reported („Melden …“ of the brush)
//!
//! [projekt]                            # the project shown, if one was open
//! name = "Abnahme"
//! pfad = "/Users/…/Abnahme"
//!
//! [auswahl]                            # what was selected in the field, if anything
//! art = "oberflaeche"                  # scan, lauf, karte, punktwolke, oberflaeche, stand, modell
//! text = "Oberfläche 001 · Karte 001 · Halle"
//! pfad = "/Users/…/karte-001/oberflaeche-001"   # its folder or file
//! scan = "20261020173822"              # a scan: its id in the project
//! nr = 3                               # a state of the history: its number
//!
//! [kamera]                             # the view, absolute ([`CameraState`]), if one was drawn
//! modus = "Orbit"                      # „Orbit“ or „Fly“
//! ziel = [512002.0, 5400001.0, 101.25]
//! auge = [512002.0, 5399998.0, 101.5]
//! gier_grad = 90.0
//! neigung_grad = -4.76
//! abstand_m = 3.01
//! bildwinkel_grad = 60.0
//! ```
//!
//! `stand` and `antwort` are plain text another program edits by hand; a key the program does
//! not know – at the top or in one of the tables (a key written below the last table belongs to
//! it) – is read and written back as it is. A report that cannot be read is left out of the list
//! ([`read_all`]).

use crate::camera::CameraState;
use crate::ui::text;
use chrono::{DateTime, Local, NaiveDateTime, TimeZone};
use driftless_steps::marks::Marks;
use eframe::egui::{ColorImage, Event, Key};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// The report in its folder.
pub const REPORT_FILE: &str = "meldung.toml";
/// The picture of the window.
pub const SCREENSHOT_FILE: &str = "fenster.png";
/// The last lines of the log.
pub const LOG_FILE: &str = "protokoll.txt";
/// How many lines of the log go along.
pub const LOG_LINES: usize = 200;
/// [`Feedback::status`] of a new report.
pub const OPEN: &str = "offen";
/// A tap of ⇧⇧: Shift pressed and let go within this (s) …
pub const TAP_MAX: f64 = 0.30;
/// … and the second pressed within this after the first was let go (s).
pub const GAP_MAX: f64 = 0.40;
/// Times of frames are sums of steps: within this (s) of a limit counts as on it.
const SLACK: f64 = 1e-6;
/// The end of a log file read for its last lines (bytes): far more than [`LOG_LINES`] lines.
const TAIL_BYTES: u64 = 1 << 20;

/// The comment at the top of `meldung.toml`.
const HEADER: &str = "# Driftless-Meldung: ein Fehler oder Wunsch, nur auf diesem Rechner \
gespeichert (nichts wird hochgeladen).\n# „stand“ und „antwort“ sind Klartext und dürfen von Hand \
geändert werden, etwa auf „erledigt in 0.2.0“.\n\n";

/// The folder of the reports (see the module documentation); `None` if the system names no
/// home for it (`HOME` or `LOCALAPPDATA` missing).
pub fn default_dir() -> Option<PathBuf> {
    let var = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty());
    if cfg!(windows) {
        Some(
            PathBuf::from(var("LOCALAPPDATA")?)
                .join("Driftless")
                .join("Meldungen"),
        )
    } else if cfg!(target_os = "macos") {
        let home = PathBuf::from(var("HOME")?);
        Some(
            home.join("Library")
                .join("Application Support")
                .join("Driftless")
                .join("Meldungen"),
        )
    } else {
        let data = var("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| var("HOME").map(|h| PathBuf::from(h).join(".local").join("share")))?;
        Some(data.join("driftless").join("meldungen"))
    }
}

/// What a report is.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeedbackKind {
    /// „Fehler“: something went wrong.
    #[default]
    #[serde(rename = "fehler")]
    Bug,
    /// „Wunsch“: something wanted.
    #[serde(rename = "wunsch")]
    Wish,
}

impl FeedbackKind {
    /// The word in the file and in the name of the folder: „fehler“, „wunsch“.
    pub fn word(self) -> &'static str {
        match self {
            FeedbackKind::Bug => "fehler",
            FeedbackKind::Wish => "wunsch",
        }
    }
}

/// The project shown when the report was made (`[projekt]`).
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct FeedbackProject {
    pub name: String,
    /// Its folder.
    #[serde(rename = "pfad")]
    pub dir: PathBuf,
    /// Keys the program does not know, kept.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

impl FeedbackProject {
    pub fn new(name: &str, dir: &Path) -> FeedbackProject {
        FeedbackProject {
            name: name.to_string(),
            dir: dir.to_path_buf(),
            unknown: toml::Table::new(),
        }
    }
}

/// What was selected in the field when the report was made (`[auswahl]`).
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct FeedbackSelection {
    /// „scan“, „lauf“, „karte“, „punktwolke“, „oberflaeche“, „stand“ or „modell“.
    #[serde(rename = "art")]
    pub kind: String,
    /// As the window names it: „Oberfläche 001 · Karte 001 · Halle“.
    pub text: String,
    /// Its folder (a cloud: its file).
    #[serde(rename = "pfad", default, skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,
    /// A scan: its id in the project.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scan: Option<String>,
    /// A state of the history of a run: its number.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nr: Option<u32>,
    /// Keys the program does not know, kept.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

impl FeedbackSelection {
    pub fn new(kind: &str, text: &str, path: Option<PathBuf>) -> FeedbackSelection {
        FeedbackSelection {
            kind: kind.to_string(),
            text: text.to_string(),
            path,
            scan: None,
            nr: None,
            unknown: toml::Table::new(),
        }
    }
}

/// The camera of the view when the report was made (`[kamera]`), absolute.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct FeedbackCamera {
    #[serde(flatten)]
    pub state: CameraState,
    /// Keys the program does not know, kept.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

impl FeedbackCamera {
    pub fn new(state: CameraState) -> FeedbackCamera {
        FeedbackCamera {
            state,
            unknown: toml::Table::new(),
        }
    }
}

fn open_status() -> String {
    OPEN.to_string()
}

/// A report, as `meldung.toml` holds it (see the module documentation).
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Feedback {
    #[serde(rename = "art")]
    pub kind: FeedbackKind,
    /// When it was made: RFC 3339, local time with its offset.
    #[serde(rename = "erstellt", default)]
    pub created: String,
    /// What came of it, as plain text: [`OPEN`], „erledigt in 0.2.0“, …
    #[serde(rename = "stand", default = "open_status")]
    pub status: String,
    /// The answer to it, as plain text.
    #[serde(rename = "antwort", default)]
    pub answer: String,
    /// What the user wrote.
    #[serde(default)]
    pub text: String,
    /// „Driftless 0.1.0 (abc123def)“ (`about::version_line`).
    #[serde(default)]
    pub version: String,
    /// „macOS 27.0 (aarch64)“ (`logfile::os_text`).
    #[serde(default)]
    pub system: String,
    /// The graphics adapter the window drew with; empty if unknown.
    #[serde(rename = "grafik", default)]
    pub graphics: String,
    /// The step of the work shown: „Daten“, „Bilder“, „Planung“.
    #[serde(rename = "schritt", default, skip_serializing_if = "Option::is_none")]
    pub step: Option<String>,
    /// The Stelle the report is about („Melden …“ in the bar of the brush).
    #[serde(rename = "stelle", default, skip_serializing_if = "Option::is_none")]
    pub mark: Option<u32>,
    #[serde(rename = "projekt", default, skip_serializing_if = "Option::is_none")]
    pub project: Option<FeedbackProject>,
    #[serde(rename = "auswahl", default, skip_serializing_if = "Option::is_none")]
    pub selection: Option<FeedbackSelection>,
    #[serde(rename = "kamera", default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<FeedbackCamera>,
    /// Keys the program does not know (written by another program, Claude), kept so that
    /// writing the report again does not drop them.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

impl Feedback {
    /// A report of `kind` with nothing else in it yet: [`OPEN`], no answer.
    pub fn new(kind: FeedbackKind) -> Feedback {
        Feedback {
            kind,
            created: String::new(),
            status: open_status(),
            answer: String::new(),
            text: String::new(),
            version: String::new(),
            system: String::new(),
            graphics: String::new(),
            step: None,
            mark: None,
            project: None,
            selection: None,
            camera: None,
            unknown: toml::Table::new(),
        }
    }

    /// Whether it is still open ([`OPEN`]).
    pub fn is_open(&self) -> bool {
        self.status.trim() == OPEN
    }
}

/// Why a report could not be written (German, for display).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeedbackError(pub String);

impl fmt::Display for FeedbackError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for FeedbackError {}

fn io_failed(what: &str, path: &Path, e: &io::Error) -> FeedbackError {
    FeedbackError(text::io_error(what, path, e))
}

/// Writes the report `f` as a new folder in `dir` (made if missing): the picture of the window,
/// a copy of the Stellen, the last [`LOG_LINES`] of `log` and, last, `meldung.toml`. A folder
/// that could not be written whole is removed again. Returns the folder.
pub fn write(
    dir: &Path,
    f: &Feedback,
    screenshot: Option<&ColorImage>,
    marks: Option<&Marks>,
    log: &[String],
) -> Result<PathBuf, FeedbackError> {
    fs::create_dir_all(dir).map_err(|e| io_failed(text::FEEDBACK_FOLDER_FAILED, dir, &e))?;
    let folder = claim(dir, &folder_name(f))?;
    match write_into(&folder, f, screenshot, marks, log) {
        Ok(()) => Ok(folder),
        Err(e) => {
            let _ = fs::remove_dir_all(&folder);
            Err(e)
        }
    }
}

/// `JJJJ-MM-TT_hhmmss_fehler`: the local time of [`Feedback::created`] (now if it cannot be
/// read) and the kind.
fn folder_name(f: &Feedback) -> String {
    let time = DateTime::parse_from_rfc3339(&f.created)
        .map(|t| t.naive_local())
        .unwrap_or_else(|_| Local::now().naive_local());
    format!("{}_{}", time.format("%Y-%m-%d_%H%M%S"), f.kind.word())
}

/// Makes the folder `base` in `dir`, else `base-2`, `base-3`, … – the first one not there yet.
fn claim(dir: &Path, base: &str) -> Result<PathBuf, FeedbackError> {
    for n in 1..=u32::MAX {
        let name = if n == 1 {
            base.to_string()
        } else {
            format!("{base}-{n}")
        };
        let path = dir.join(name);
        match fs::create_dir(&path) {
            Ok(()) => return Ok(path),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(io_failed(text::FEEDBACK_FOLDER_FAILED, &path, &e)),
        }
    }
    Err(FeedbackError(text::FEEDBACK_FOLDER_FAILED.to_string()))
}

fn write_into(
    folder: &Path,
    f: &Feedback,
    screenshot: Option<&ColorImage>,
    marks: Option<&Marks>,
    log: &[String],
) -> Result<(), FeedbackError> {
    if let Some(image) = screenshot {
        let path = folder.join(SCREENSHOT_FILE);
        let bytes = png_of(image).map_err(|e| FeedbackError(text::feedback_picture_failed(&e)))?;
        fs::write(&path, bytes).map_err(|e| io_failed(text::FEEDBACK_FILE_FAILED, &path, &e))?;
    }
    if let Some(marks) = marks {
        marks
            .save(folder)
            .map_err(|e| FeedbackError(e.to_string()))?;
    }
    let tail = &log[log.len().saturating_sub(LOG_LINES)..];
    let mut lines = tail.join("\n");
    if !lines.is_empty() {
        lines.push('\n');
    }
    let path = folder.join(LOG_FILE);
    fs::write(&path, lines).map_err(|e| io_failed(text::FEEDBACK_FILE_FAILED, &path, &e))?;
    let path = folder.join(REPORT_FILE);
    let body = toml::to_string(f).map_err(|e| FeedbackError(text::feedback_unwritable(&e)))?;
    driftless_io::atomic_write(&path, format!("{HEADER}{body}").as_bytes())
        .map_err(|e| io_failed(text::FEEDBACK_FILE_FAILED, &path, &e))
}

/// The picture as PNG (RGBA, 8 bit), quickly packed.
fn png_of(image: &ColorImage) -> Result<Vec<u8>, png::EncodingError> {
    let [w, h] = image.size;
    let rgba: Vec<u8> = image
        .pixels
        .iter()
        .flat_map(|c| c.to_srgba_unmultiplied())
        .collect();
    let mut bytes = Vec::new();
    let mut encoder = png::Encoder::new(&mut bytes, w as u32, h as u32);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.set_deflate_compression(png::DeflateCompression::FdeflateUltraFast);
    encoder.set_filter(png::Filter::Paeth);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(&rgba)?;
    writer.finish()?;
    Ok(bytes)
}

/// The report in `folder` (its `meldung.toml`).
pub fn read(folder: &Path) -> Result<Feedback, FeedbackError> {
    let path = folder.join(REPORT_FILE);
    let text =
        fs::read_to_string(&path).map_err(|e| io_failed(text::FEEDBACK_FILE_FAILED, &path, &e))?;
    toml::from_str(&text).map_err(|e| FeedbackError(text::damaged_toml(&path, &text, &e)))
}

/// Every report in `dir`, the newest first (by [`Feedback::created`], within one second the
/// later folder); a folder without a readable `meldung.toml` is left out, so is anything that
/// is no folder. Empty if `dir` cannot be read (it may not exist yet).
pub fn read_all(dir: &Path) -> Vec<(PathBuf, Feedback)> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut all: Vec<(PathBuf, Feedback)> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .filter_map(|p| read(&p).ok().map(|f| (p, f)))
        .collect();
    all.sort_by_cached_key(|(p, f)| std::cmp::Reverse(order(p, f)));
    all
}

/// What orders the reports: the time made (ms), the number of the folder within its second
/// (1 without one), and its name.
fn order(folder: &Path, f: &Feedback) -> (i64, u32, String) {
    let name = folder
        .file_name()
        .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
    let time = DateTime::parse_from_rfc3339(&f.created)
        .map(|t| t.timestamp_millis())
        .ok()
        .or_else(|| name_time(&name))
        .unwrap_or(i64::MIN);
    let nr = name
        .rsplit_once('-')
        .and_then(|(_, n)| n.parse::<u32>().ok())
        .unwrap_or(1);
    (time, nr, name)
}

/// The local time at the start of the name of a folder of a report (ms).
fn name_time(name: &str) -> Option<i64> {
    let naive = NaiveDateTime::parse_from_str(name.get(..17)?, "%Y-%m-%d_%H%M%S").ok()?;
    Some(
        Local
            .from_local_datetime(&naive)
            .earliest()?
            .timestamp_millis(),
    )
}

/// The last `n` lines of the file at `path` (of its last MiB, which is plenty).
pub fn last_lines(path: &Path, n: usize) -> io::Result<Vec<String>> {
    let mut file = fs::File::open(path)?;
    let len = file.metadata()?.len();
    let start = len.saturating_sub(TAIL_BYTES);
    file.seek(SeekFrom::Start(start))?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    let text = String::from_utf8_lossy(&bytes);
    let mut lines: Vec<&str> = text.lines().collect();
    if start > 0 && !lines.is_empty() {
        // the first one is cut
        lines.remove(0);
    }
    let from = lines.len().saturating_sub(n);
    Ok(lines[from..].iter().map(|l| l.to_string()).collect())
}

/// The detector of ⇧⇧, as a state machine over the events of the frames (see the module
/// documentation). The window feeds it every frame ([`DoubleShift::feed`]).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DoubleShift {
    tap: Tap,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
enum Tap {
    #[default]
    Idle,
    /// A Shift is down since `at`; `second`: it is the second tap.
    Down { key: Key, at: f64, second: bool },
    /// The first tap ended at `at`.
    Up { at: f64 },
}

impl DoubleShift {
    /// Takes the events of a frame at its time `time` (s): whether ⇧⇧ was completed in them.
    pub fn feed(&mut self, events: &[Event], time: f64) -> bool {
        let mut fired = false;
        for event in events {
            fired |= self.event(event, time);
        }
        fired
    }

    fn event(&mut self, event: &Event, time: f64) -> bool {
        match event {
            Event::Key {
                key: key @ (Key::ShiftLeft | Key::ShiftRight),
                pressed,
                repeat,
                ..
            } => self.shift(*key, *pressed, *repeat, time),
            Event::Key { .. }
            | Event::Text(_)
            | Event::Ime(_)
            | Event::Copy
            | Event::Cut
            | Event::Paste(_)
            | Event::PointerButton { .. }
            | Event::MouseWheel { .. }
            | Event::Zoom(_)
            | Event::Rotate(_)
            | Event::Touch { .. }
            | Event::WindowFocused(false) => {
                self.tap = Tap::Idle;
                false
            }
            _ => false,
        }
    }

    fn shift(&mut self, key: Key, pressed: bool, repeat: bool, time: f64) -> bool {
        match (self.tap, pressed) {
            // the system repeats a key held down: still the same press
            (_, true) if repeat => false,
            (Tap::Idle, true) => {
                self.tap = Tap::Down {
                    key,
                    at: time,
                    second: false,
                };
                false
            }
            (Tap::Up { at }, true) => {
                self.tap = Tap::Down {
                    key,
                    at: time,
                    second: time - at <= GAP_MAX + SLACK,
                };
                false
            }
            // the other Shift while one is down: no tap
            (Tap::Down { .. }, true) => {
                self.tap = Tap::Idle;
                false
            }
            (
                Tap::Down {
                    key: held,
                    at,
                    second,
                },
                false,
            ) if held == key && time - at <= TAP_MAX + SLACK => {
                self.tap = if second {
                    Tap::Idle
                } else {
                    Tap::Up { at: time }
                };
                second
            }
            // held too long, or the other one let go
            (Tap::Down { .. }, false) => {
                self.tap = Tap::Idle;
                false
            }
            (Tap::Idle | Tap::Up { .. }, false) => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{CameraMode, CameraState};
    use driftless_io::testdir::TestDir;
    use driftless_steps::marks::{DabInput, Marks, MARKS_FILE};
    use eframe::egui::{self, Color32, ColorImage, Event, Key, Modifiers, PointerButton};
    use std::fs;
    use std::path::{Path, PathBuf};

    /// A fresh, empty folder for the test `name`, removed when it is done.
    fn tmp(name: &str) -> TestDir {
        TestDir::new(format!("driftless-feedback-{}-{name}", std::process::id()))
    }

    /// Shift `key` pressed or let go.
    fn shift(key: Key, pressed: bool) -> Event {
        let modifiers = if pressed {
            Modifiers::SHIFT
        } else {
            Modifiers::NONE
        };
        Event::Key {
            key,
            physical_key: Some(key),
            pressed,
            repeat: false,
            modifiers,
        }
    }

    /// Another key, pressed or let go with Shift held.
    fn other(key: Key, pressed: bool) -> Event {
        Event::Key {
            key,
            physical_key: Some(key),
            pressed,
            repeat: false,
            modifiers: Modifiers::SHIFT,
        }
    }

    /// A tap of `key`: pressed at `at`, let go `hold` later.
    fn tap(key: Key, at: f64, hold: f64) -> Vec<(f64, Event)> {
        vec![(at, shift(key, true)), (at + hold, shift(key, false))]
    }

    /// The times ⇧⇧ fired when each event comes in a frame of its own at its time.
    fn fired(events: &[(f64, Event)]) -> Vec<f64> {
        let mut detector = DoubleShift::default();
        events
            .iter()
            .filter(|(t, e)| detector.feed(std::slice::from_ref(e), *t))
            .map(|(t, _)| *t)
            .collect()
    }

    fn joined(parts: &[Vec<(f64, Event)>]) -> Vec<(f64, Event)> {
        parts.concat()
    }

    /// `got` are the times `want` (within float noise).
    fn assert_fired(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?} ≠ {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-9, "{got:?} ≠ {want:?}");
        }
    }

    #[test]
    fn double_shift() {
        use Key::{ShiftLeft as L, ShiftRight as R};
        // tap, tap: 0.1 s down, 0.2 s between
        assert_fired(
            &fired(&joined(&[tap(L, 0.0, 0.1), tap(L, 0.3, 0.1)])),
            &[0.4],
        );
        // at the limits: 0.3 s down each, the second press 0.4 s after the first release
        assert_fired(
            &fired(&joined(&[tap(L, 1.0, 0.3), tap(L, 1.7, 0.3)])),
            &[2.0],
        );
        // left and right mixed
        assert_fired(
            &fired(&joined(&[tap(L, 0.0, 0.1), tap(R, 0.3, 0.1)])),
            &[0.4],
        );
        assert_fired(
            &fired(&joined(&[tap(R, 0.0, 0.1), tap(L, 0.3, 0.1)])),
            &[0.4],
        );
        // Shift held 0.5 s (panning with Shift, flying faster): never, not twice either
        assert!(fired(&joined(&[tap(L, 0.0, 0.5), tap(L, 0.6, 0.5)])).is_empty());
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), tap(L, 0.3, 0.5)])).is_empty());
        assert!(fired(&joined(&[tap(L, 0.0, 0.5), tap(L, 0.6, 0.1)])).is_empty());
        // held with the repeats of the system: the same press
        let mut held = vec![(0.0, shift(L, true))];
        for k in 1..10 {
            let mut repeat = shift(L, true);
            if let Event::Key { repeat: r, .. } = &mut repeat {
                *r = true;
            }
            held.push((f64::from(k) * 0.05, repeat));
        }
        held.push((0.5, shift(L, false)));
        held.extend(tap(L, 0.6, 0.1));
        assert!(fired(&held).is_empty());
        // a gap of 0.6 s: never
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), tap(L, 0.7, 0.1)])).is_empty());
        // Shift–A–Shift: a capital letter between, never
        let capital = vec![
            (0.0, shift(L, true)),
            (0.05, other(Key::A, true)),
            (0.05, Event::Text("A".into())),
            (0.08, other(Key::A, false)),
            (0.1, shift(L, false)),
        ];
        assert!(fired(&joined(&[capital.clone(), tap(L, 0.3, 0.1)])).is_empty());
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), capital.clone()])).is_empty());
        let typed = vec![(0.15, Event::Text("a".into()))];
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), typed, tap(L, 0.3, 0.1)])).is_empty());
        // Shift + click: never
        let click = |at: f64| {
            let button = |pressed| Event::PointerButton {
                pos: egui::pos2(10.0, 10.0),
                button: PointerButton::Primary,
                pressed,
                modifiers: Modifiers::SHIFT,
            };
            vec![(at, button(true)), (at + 0.02, button(false))]
        };
        let shift_click = joined(&[
            vec![(0.0, shift(L, true))],
            click(0.05),
            vec![(0.1, shift(L, false))],
        ]);
        assert!(fired(&joined(&[shift_click.clone(), tap(L, 0.3, 0.1)])).is_empty());
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), click(0.15), tap(L, 0.3, 0.1)])).is_empty());
        // the wheel and losing the focus between reset it too, moving the pointer does not
        let wheel = vec![(
            0.15,
            Event::MouseWheel {
                unit: egui::MouseWheelUnit::Line,
                delta: egui::vec2(0.0, 1.0),
                phase: egui::TouchPhase::Move,
                modifiers: Modifiers::NONE,
            },
        )];
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), wheel, tap(L, 0.3, 0.1)])).is_empty());
        let unfocused = vec![(0.15, Event::WindowFocused(false))];
        assert!(fired(&joined(&[tap(L, 0.0, 0.1), unfocused, tap(L, 0.3, 0.1)])).is_empty());
        let moved = vec![
            (0.15, Event::PointerMoved(egui::pos2(40.0, 30.0))),
            (0.16, Event::ModifiersChanged(Modifiers::NONE)),
        ];
        assert_fired(
            &fired(&joined(&[tap(L, 0.0, 0.1), moved, tap(L, 0.3, 0.1)])),
            &[0.4],
        );
        // both taps in one frame (a slow frame) count as well
        let mut detector = DoubleShift::default();
        let frame: Vec<Event> = joined(&[tap(L, 0.0, 0.0), tap(L, 0.0, 0.0)])
            .into_iter()
            .map(|(_, e)| e)
            .collect();
        assert!(detector.feed(&frame, 0.0));
        // three taps fire once; four taps twice
        let three = joined(&[tap(L, 0.0, 0.1), tap(L, 0.3, 0.1), tap(L, 0.6, 0.1)]);
        assert_fired(&fired(&three), &[0.4]);
        let four = joined(&[three, tap(L, 0.9, 0.1)]);
        assert_fired(&fired(&four), &[0.4, 1.0]);
        // left and right down together: no tap
        let both = vec![
            (0.0, shift(L, true)),
            (0.02, shift(R, true)),
            (0.05, shift(R, false)),
            (0.08, shift(L, false)),
        ];
        assert!(fired(&joined(&[both, tap(L, 0.3, 0.1)])).is_empty());
    }

    /// A picture of `w` × `h` pixels with a colour per pixel.
    fn picture(w: usize, h: usize) -> ColorImage {
        let pixels = (0..w * h)
            .map(|i| Color32::from_rgb((i * 7) as u8, (i * 13) as u8, 200))
            .collect();
        ColorImage::new([w, h], pixels)
    }

    /// One Stelle on a wall.
    fn marks() -> Marks {
        let mut marks = Marks::default();
        let dab = DabInput {
            centre: [2.0, 1.0, 1.25],
            normal: [0.0, -1.0, 0.0],
            radius: 0.2,
            erase: false,
        };
        marks.paint(None, &[dab]);
        marks
    }

    fn report(kind: FeedbackKind, created: &str, text: &str) -> Feedback {
        let mut f = Feedback::new(kind);
        f.created = created.into();
        f.text = text.into();
        f.version = "Driftless 0.1.0 (test)".into();
        f.system = "macOS 27.0 (aarch64)".into();
        f.graphics = "Apple M4 Pro (Metal)".into();
        f.step = Some("Bilder".into());
        f.mark = Some(1);
        f.project = Some(FeedbackProject::new("Halle", Path::new("/Projekte/Halle")));
        f.selection = Some(FeedbackSelection::new(
            "oberflaeche",
            "Oberfläche 001 · Karte 001 · Halle",
            Some(PathBuf::from(
                "/Projekte/Halle/scans/halle/slam-001/karte-001/oberflaeche-001",
            )),
        ));
        f.camera = Some(FeedbackCamera::new(CameraState {
            mode: CameraMode::Orbit,
            target: [512_002.0, 5_400_001.0, 101.25],
            eye: [512_002.0, 5_399_998.0, 101.5],
            yaw_deg: 90.0,
            pitch_deg: -4.76,
            distance_m: 3.01,
            fov_y_deg: 60.0,
        }));
        f
    }

    fn name(p: &Path) -> String {
        p.file_name().unwrap().to_string_lossy().into_owned()
    }

    #[test]
    fn write_and_read() {
        let dir = tmp("schreiben");
        let reports = dir.join("Meldungen");
        let bug = report(
            FeedbackKind::Bug,
            "2026-10-29T10:14:03+01:00",
            "Decke flimmert beim Drehen\nnur oben links",
        );
        let lines: Vec<String> = (1..=250).map(|k| format!("Zeile {k}")).collect();
        // the folder is made; two reports in the same second get two folders
        let first = write(&reports, &bug, Some(&picture(8, 4)), Some(&marks()), &lines).unwrap();
        let again = report(FeedbackKind::Bug, "2026-10-29T10:14:03+01:00", "noch einer");
        let second = write(&reports, &again, None, None, &[]).unwrap();
        let third = write(&reports, &again, None, None, &[]).unwrap();
        assert_eq!(name(&first), "2026-10-29_101403_fehler");
        assert_eq!(name(&second), "2026-10-29_101403_fehler-2");
        assert_eq!(name(&third), "2026-10-29_101403_fehler-3");
        let wish = report(FeedbackKind::Wish, "2026-10-29T10:15:00+01:00", "Größer");
        let fourth = write(&reports, &wish, None, None, &["eine Zeile".into()]).unwrap();
        assert_eq!(name(&fourth), "2026-10-29_101500_wunsch");
        // what lies in a folder: the picture, a copy of the Stellen, the last lines of the log
        let png = fs::read(first.join(SCREENSHOT_FILE)).unwrap();
        let decoded = image::load_from_memory(&png).unwrap().to_rgba8();
        assert_eq!(decoded.dimensions(), (8, 4));
        assert_eq!(
            decoded.get_pixel(3, 1).0,
            picture(8, 4).pixels[11].to_array()
        );
        assert_eq!(Marks::load(&first).unwrap(), marks());
        assert!(first.join(MARKS_FILE).is_file());
        let log = fs::read_to_string(first.join(LOG_FILE)).unwrap();
        let kept: Vec<&str> = log.lines().collect();
        assert_eq!(kept.len(), LOG_LINES);
        assert_eq!(kept[0], "Zeile 51");
        assert_eq!(kept[LOG_LINES - 1], "Zeile 250");
        for folder in [&second, &third] {
            assert!(!folder.join(SCREENSHOT_FILE).exists());
            assert!(!folder.join(MARKS_FILE).exists());
            assert!(folder.join(LOG_FILE).is_file());
        }
        assert_eq!(
            fs::read_to_string(fourth.join(LOG_FILE)).unwrap(),
            "eine Zeile\n"
        );
        // nothing half written lies around
        for folder in [&first, &second, &third, &fourth] {
            for entry in fs::read_dir(folder).unwrap() {
                let n = entry.unwrap().file_name().to_string_lossy().into_owned();
                assert!(!n.ends_with(".tmp") && !n.ends_with(".lock"), "{n}");
            }
        }
        // the file says it in plain words
        let toml = fs::read_to_string(first.join(REPORT_FILE)).unwrap();
        for line in [
            "art = \"fehler\"",
            "erstellt = \"2026-10-29T10:14:03+01:00\"",
            "stand = \"offen\"",
            "antwort = \"\"",
            "schritt = \"Bilder\"",
            "stelle = 1",
            "[projekt]",
            "[auswahl]",
            "[kamera]",
        ] {
            assert!(toml.contains(line), "{line} in\n{toml}");
        }
        // newest first; within one second the later folder
        let all = read_all(&reports);
        let names: Vec<String> = all.iter().map(|(p, _)| name(p)).collect();
        assert_eq!(
            names,
            [
                "2026-10-29_101500_wunsch",
                "2026-10-29_101403_fehler-3",
                "2026-10-29_101403_fehler-2",
                "2026-10-29_101403_fehler"
            ]
        );
        assert_eq!(all[3].1, bug);
        assert_eq!(all[0].1, wish);
        // Claude answers in the file and adds keys of its own: they are read …
        let path = first.join(REPORT_FILE);
        let edited = toml
            .replace("stand = \"offen\"", "stand = \"erledigt\"")
            .replace(
                "antwort = \"\"",
                "antwort = \"erledigt in 0.2.0: Wände werden als ganze Fläche geglättet\"\n\
                 notiz = \"von Hand\"",
            );
        // a key written below the last table belongs to it
        let edited = format!("{edited}geprueft = true\n\n[claude]\nsitzung = \"abc\"\n");
        fs::write(&path, edited).unwrap();
        let all = read_all(&reports);
        let (_, answered) = all.iter().find(|(p, _)| *p == first).unwrap();
        assert_eq!(answered.status, "erledigt");
        assert_eq!(
            answered.answer,
            "erledigt in 0.2.0: Wände werden als ganze Fläche geglättet"
        );
        assert_eq!(answered.unknown["notiz"].as_str(), Some("von Hand"));
        assert_eq!(answered.unknown["claude"]["sitzung"].as_str(), Some("abc"));
        let camera = answered.camera.as_ref().unwrap();
        assert_eq!(camera.unknown["geprueft"].as_bool(), Some(true));
        assert_eq!(camera.state, bug.camera.as_ref().unwrap().state);
        assert_eq!(answered.text, bug.text);
        // … and kept when it is written again
        let rewritten = write(&reports, answered, None, None, &[]).unwrap();
        let back = read_all(&reports)
            .into_iter()
            .find(|(p, _)| *p == rewritten)
            .unwrap()
            .1;
        assert_eq!(&back, answered);
        // what cannot be read is left out: a folder without its file, a broken file, a file
        let empty = reports.join("2026-10-30_090000_fehler");
        fs::create_dir_all(&empty).unwrap();
        let broken = reports.join("2026-10-30_090100_wunsch");
        fs::create_dir_all(&broken).unwrap();
        fs::write(broken.join(REPORT_FILE), "art = \"frage\"\n").unwrap();
        fs::write(reports.join(".DS_Store"), b"x").unwrap();
        assert_eq!(read_all(&reports).len(), 5);
        assert!(read_all(&dir.join("fehlt")).is_empty());
    }

    #[test]
    fn the_last_lines_of_a_file() {
        let dir = tmp("zeilen");
        let path = dir.join("driftless.log");
        let text: String = (1..=5000).map(|k| format!("Zeile {k}\n")).collect();
        fs::write(&path, text).unwrap();
        let lines = last_lines(&path, 200).unwrap();
        assert_eq!(lines.len(), 200);
        assert_eq!(lines[0], "Zeile 4801");
        assert_eq!(lines[199], "Zeile 5000");
        fs::write(&path, "eins\nzwei").unwrap();
        assert_eq!(last_lines(&path, 200).unwrap(), ["eins", "zwei"]);
        assert!(last_lines(&dir.join("fehlt.log"), 200).is_err());
    }

    #[test]
    fn the_folder_is_the_one_of_the_system() {
        let dir = default_dir().expect("HOME or LOCALAPPDATA is set");
        if cfg!(target_os = "macos") {
            assert!(
                dir.ends_with("Library/Application Support/Driftless/Meldungen"),
                "{}",
                dir.display()
            );
            assert!(dir.starts_with(std::env::var_os("HOME").unwrap()));
        }
        if cfg!(windows) {
            assert!(dir.ends_with("Driftless\\Meldungen"), "{}", dir.display());
            assert!(dir.starts_with(std::env::var_os("LOCALAPPDATA").unwrap()));
        }
        // not the folder of eframe's own storage on Windows (Roaming)
        assert!(!dir.to_string_lossy().contains("Roaming"));
    }

    #[test]
    fn kinds_are_german_words() {
        assert_eq!(FeedbackKind::Bug.word(), "fehler");
        assert_eq!(FeedbackKind::Wish.word(), "wunsch");
        assert_eq!(Feedback::new(FeedbackKind::Wish).status, OPEN);
    }
}
