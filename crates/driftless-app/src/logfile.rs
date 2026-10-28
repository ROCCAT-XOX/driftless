//! The log file of the program, for when something fails on another PC: what goes into the
//! „Protokoll“ and the status bar (messages, warnings and errors of the steps; the window writes
//! each as it comes), at each start a line with version, commit, system, graphics adapter and
//! project, every panic with its place and backtrace ([`install_panic_hook`]), and the warnings
//! and errors the libraries (wgpu, eframe, winit) report through the `log` facade
//! ([`install_library_logger`]; they stay out of the Protokoll on screen).
//!
//! - macOS: `~/Library/Logs/Driftless/driftless.log`
//! - Windows: `%LOCALAPPDATA%\Driftless\Logs\driftless.log`
//!
//! Each start moves the files of the earlier ones on by one (`driftless.log` becomes
//! `driftless.1.log`, …, `driftless.4.log` is deleted), and so does a file that reaches
//! [`MAX_BYTES`]: at most [`KEEP`] files of about 10 MB each. Writing never stops the program; a
//! line that cannot be written is lost. The command line `driftless` keeps writing to stderr.

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::panic::PanicHookInfo;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;

/// The name of the file of this start.
pub const FILE_NAME: &str = "driftless.log";
/// How many files are kept: the one of this start and four of earlier ones.
pub const KEEP: usize = 5;
/// The size from which a file is moved on before the next line.
pub const MAX_BYTES: u64 = 10 * 1024 * 1024;
/// The last line of a start that ended normally.
pub const END_TEXT: &str = "Programm beendet";
/// At most this many lines of the libraries per start ([`install_library_logger`]): a
/// warning that comes every frame must not fill the disk.
pub const LIBRARY_LINES: usize = 500;
/// After [`LIBRARY_LINES`]: the last line of the libraries.
pub const LIBRARY_LIMIT_TEXT: &str =
    "weitere Warnungen der Bibliotheken in dieser Sitzung nicht geschrieben";

/// The folder of the log files: on macOS in `~/Library/Logs`, on Windows in `%LOCALAPPDATA%`
/// (elsewhere the XDG state folder); `None` if that folder is unknown.
pub fn default_dir() -> Option<PathBuf> {
    let var = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty());
    if cfg!(windows) {
        Some(
            PathBuf::from(var("LOCALAPPDATA")?)
                .join("Driftless")
                .join("Logs"),
        )
    } else if cfg!(target_os = "macos") {
        let home = PathBuf::from(var("HOME")?);
        Some(home.join("Library").join("Logs").join("Driftless"))
    } else {
        let state = var("XDG_STATE_HOME")
            .map(PathBuf::from)
            .or_else(|| var("HOME").map(|h| PathBuf::from(h).join(".local").join("state")))?;
        Some(state.join("driftless"))
    }
}

/// What a line of the file is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// The line of a start ([`start_text`]).
    Start,
    /// A message of a step (Protokoll).
    Info,
    /// A warning of a step (Protokoll).
    Warning,
    /// A message of the status bar.
    Message,
    /// An error of the status bar, e.g. a failed step.
    Error,
    /// A panic ([`install_panic_hook`]).
    Panic,
}

impl Level {
    /// The word for it in the file.
    pub fn tag(self) -> &'static str {
        match self {
            Level::Start => "Start",
            Level::Info => "Info",
            Level::Warning => "Warnung",
            Level::Message => "Meldung",
            Level::Error => "Fehler",
            Level::Panic => "Absturz",
        }
    }
}

/// One entry of the file: time, kind, text; further lines of the text indented by four spaces.
pub fn format_line(time: &str, level: Level, text: &str) -> String {
    let mut out = format!("{time} {:<7} ", level.tag());
    for (i, line) in text.lines().enumerate() {
        if i > 0 {
            out.push_str("\n    ");
        }
        out.push_str(line);
    }
    out.push('\n');
    out
}

/// The text of the start line: „Driftless 0.1.0 (abc123def) · macOS 27.0 (aarch64) · Grafik:
/// Apple M4 Pro (Metal) · Projekt: /Users/…/Halle“; `program` is the version line of „Über
/// Driftless“.
pub fn start_text(program: &str, os: &str, gpu: Option<&str>, project: Option<&Path>) -> String {
    let project = project.map_or_else(|| "keins".to_string(), |p| p.display().to_string());
    format!(
        "{program} · {os} · Grafik: {} · Projekt: {project}",
        gpu.unwrap_or("unbekannt")
    )
}

/// The graphics adapter for the start line: its name and, in brackets, what is known of it
/// (backend, kind, driver): „NVIDIA GeForce RTX 5080 (Vulkan, DiscreteGpu, NVIDIA 580.95.05)“.
/// Empty details are left out.
pub fn gpu_text(name: &str, details: &[String]) -> String {
    let details: Vec<&str> = details
        .iter()
        .map(|d| d.trim())
        .filter(|d| !d.is_empty())
        .collect();
    if details.is_empty() {
        name.to_string()
    } else {
        format!("{name} ({})", details.join(", "))
    }
}

/// The system for the start line: „macOS 27.0 (aarch64)“, „Windows 10.0.26100 (x86_64)“.
pub fn os_text() -> String {
    let name = if cfg!(target_os = "macos") {
        "macOS"
    } else if cfg!(windows) {
        "Windows"
    } else {
        std::env::consts::OS
    };
    let arch = std::env::consts::ARCH;
    match os_version() {
        Some(version) => format!("{name} {version} ({arch})"),
        None => format!("{name} ({arch})"),
    }
}

/// The version of macOS, from the system's `SystemVersion.plist`.
#[cfg(target_os = "macos")]
fn os_version() -> Option<String> {
    let plist = fs::read_to_string("/System/Library/CoreServices/SystemVersion.plist").ok()?;
    plist_string(&plist, "ProductVersion")
}

/// The string after `<key>{key}</key>` in an XML property list.
#[cfg(target_os = "macos")]
fn plist_string(plist: &str, key: &str) -> Option<String> {
    let rest = &plist[plist.find(&format!("<key>{key}</key>"))?..];
    let start = rest.find("<string>")? + "<string>".len();
    let end = start + rest[start..].find("</string>")?;
    Some(rest[start..end].trim().to_string())
}

/// The version of Windows, „10.0.26100“ (the build tells Windows 10 and 11 apart), from
/// `RtlGetVersion`, which, unlike `GetVersionEx`, does not depend on the program's manifest.
#[cfg(windows)]
fn os_version() -> Option<String> {
    /// `OSVERSIONINFOW`.
    #[repr(C)]
    struct OsVersionInfo {
        size: u32,
        major: u32,
        minor: u32,
        build: u32,
        _platform: u32,
        _service_pack: [u16; 128],
    }
    #[link(name = "ntdll", kind = "raw-dylib")]
    extern "system" {
        fn RtlGetVersion(info: *mut OsVersionInfo) -> i32;
    }
    let mut info = OsVersionInfo {
        size: std::mem::size_of::<OsVersionInfo>() as u32,
        major: 0,
        minor: 0,
        build: 0,
        _platform: 0,
        _service_pack: [0; 128],
    };
    // SAFETY: `info` is an OSVERSIONINFOW with its size set, which RtlGetVersion fills.
    let status = unsafe { RtlGetVersion(&mut info) };
    (status == 0).then(|| format!("{}.{}.{}", info.major, info.minor, info.build))
}

#[cfg(not(any(windows, target_os = "macos")))]
fn os_version() -> Option<String> {
    None
}

/// The local time of a line, to the millisecond.
fn now() -> String {
    chrono::Local::now()
        .format("%Y-%m-%d %H:%M:%S%.3f")
        .to_string()
}

/// `driftless.log` for 0, else `driftless.<n>.log`.
fn numbered(dir: &Path, n: usize) -> PathBuf {
    if n == 0 {
        dir.join(FILE_NAME)
    } else {
        dir.join(format!("driftless.{n}.log"))
    }
}

/// Moves the files on by one: `driftless.4.log` is deleted, `driftless.3.log` becomes
/// `driftless.4.log`, …, `driftless.log` becomes `driftless.1.log`. A file that cannot be moved
/// stays where it is (and is then written on).
fn rotate(dir: &Path) {
    let _ = fs::remove_file(numbered(dir, KEEP - 1));
    for n in (0..KEEP - 1).rev() {
        let _ = fs::rename(numbered(dir, n), numbered(dir, n + 1));
    }
}

/// Appends `text` with one write (the file is opened for it: the window and a panic on another
/// thread can write at the same time).
fn append(path: &Path, text: &str) -> std::io::Result<()> {
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    file.write_all(text.as_bytes())
}

/// The log file of this start. The window writes into it from its thread ([`LogFile::write`]),
/// the panic hook from any thread.
#[derive(Debug)]
pub struct LogFile {
    path: PathBuf,
    max_bytes: u64,
    /// The version line of the program, for the start line.
    program: String,
    /// The start line was written.
    started: AtomicBool,
    /// Lines of the libraries written ([`LIBRARY_LINES`]).
    library_lines: AtomicUsize,
}

impl LogFile {
    /// Moves the files of earlier starts on by one ([`KEEP`]) and begins the file of this start
    /// in `dir` (made if missing). `program` is the version line of „Über Driftless“.
    pub fn start(dir: &Path, program: &str) -> std::io::Result<LogFile> {
        LogFile::start_with_limit(dir, program, MAX_BYTES)
    }

    /// [`LogFile::start`] with another size from which a file is moved on (tests).
    pub fn start_with_limit(dir: &Path, program: &str, max_bytes: u64) -> std::io::Result<LogFile> {
        fs::create_dir_all(dir)?;
        rotate(dir);
        let path = dir.join(FILE_NAME);
        // Not truncated: if the last file could not be moved on, it is written on.
        OpenOptions::new().create(true).append(true).open(&path)?;
        Ok(LogFile {
            path,
            max_bytes,
            program: program.to_string(),
            started: AtomicBool::new(false),
            library_lines: AtomicUsize::new(0),
        })
    }

    /// The file of this start.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The start line ([`start_text`]), once per start: what comes first writes it, so a line
    /// written before (an error before the window, a panic) gets it without adapter and
    /// project.
    pub fn write_start(&self, gpu: Option<&str>, project: Option<&Path>) {
        self.start_once(gpu, project, true);
    }

    /// A line; the start line first if there was none yet. A full file is moved on before
    /// ([`MAX_BYTES`]). Only for one thread (the window's).
    pub fn write(&self, level: Level, text: &str) {
        self.start_once(None, None, true);
        self.put(level, text, true);
    }

    fn start_once(&self, gpu: Option<&str>, project: Option<&Path>, may_rotate: bool) {
        if !self.started.swap(true, Ordering::AcqRel) {
            let text = start_text(&self.program, &os_text(), gpu, project);
            self.put(Level::Start, &text, may_rotate);
        }
    }

    fn put(&self, level: Level, text: &str, may_rotate: bool) {
        let line = format_line(&now(), level, text);
        if may_rotate {
            let size = fs::metadata(&self.path).map_or(0, |m| m.len());
            if size > 0 && size + line.len() as u64 > self.max_bytes {
                if let Some(dir) = self.path.parent() {
                    rotate(dir);
                }
            }
        }
        let _ = append(&self.path, &line);
    }

    /// The entry of a panic, from any thread: never moves the files on (the window may write at
    /// the same time) and takes no lock (the panic may have come from inside this file).
    fn write_panic(&self, text: &str) {
        self.start_once(None, None, false);
        self.put(Level::Panic, text, false);
    }

    /// A line of a library, from any thread ([`install_library_logger`]): like a panic it
    /// never moves the files on; after [`LIBRARY_LINES`] one last line says that more came.
    pub fn write_library(&self, level: Level, target: &str, message: &str) {
        let n = self.library_lines.fetch_add(1, Ordering::AcqRel);
        if n > LIBRARY_LINES {
            return;
        }
        self.start_once(None, None, false);
        if n == LIBRARY_LINES {
            self.put(Level::Warning, LIBRARY_LIMIT_TEXT, false);
        } else {
            self.put(level, &library_text(target, message), false);
        }
    }
}

/// A line of a library: „[wgpu_core::device] …“ (the module that reported it, then its text).
pub fn library_text(target: &str, message: &str) -> String {
    format!("[{target}] {message}")
}

/// The logger of the `log` facade: the warnings and errors of the libraries (wgpu, eframe,
/// egui, winit, …) into the log file, nothing into the Protokoll on screen.
struct LibraryLogger {
    log: Arc<LogFile>,
}

impl log::Log for LibraryLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Warn
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let level = match record.level() {
            log::Level::Error => Level::Error,
            _ => Level::Warning,
        };
        let message = record.args().to_string();
        self.log.write_library(level, record.target(), &message);
    }

    fn flush(&self) {}
}

/// Makes the warnings and errors that wgpu, eframe, winit and the other libraries report
/// through the `log` facade go into `log` (at most [`LIBRARY_LINES`] per start). One logger per
/// process: false if one was installed before (nothing changes then).
pub fn install_library_logger(log: Arc<LogFile>) -> bool {
    let installed = log::set_boxed_logger(Box::new(LibraryLogger { log })).is_ok();
    if installed {
        log::set_max_level(log::LevelFilter::Warn);
    }
    installed
}

/// Every panic goes into `log` first (thread, message, place, backtrace), then to the hook that
/// was installed before (the default one prints it to stderr). The panics of steps are caught
/// afterwards (`crate::jobs`) and shown as errors; the file then holds where they came from.
pub fn install_panic_hook(log: Arc<LogFile>) {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        log.write_panic(&panic_text(info));
        previous(info);
    }));
}

/// Thread, message, place and backtrace of a panic.
fn panic_text(info: &PanicHookInfo<'_>) -> String {
    let thread = std::thread::current();
    let thread = thread.name().unwrap_or("ohne Namen");
    let message = info.payload_as_str().unwrap_or("(keine Meldung)");
    let place = info
        .location()
        .map_or_else(|| "unbekannt".to_string(), |l| l.to_string());
    let backtrace = std::backtrace::Backtrace::force_capture();
    format!("Thread „{thread}“: {message}\nOrt: {place}\nBacktrace:\n{backtrace}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use driftless_io::testdir::TestDir;

    /// A fresh, empty folder for the test `name`, removed when it is done.
    fn tmp(name: &str) -> TestDir {
        TestDir::new(format!("driftless-logfile-{}-{name}", std::process::id()))
    }

    fn read(path: &Path) -> String {
        fs::read_to_string(path).unwrap()
    }

    #[test]
    fn a_start_moves_the_files_on_and_keeps_five() {
        let dir = tmp("rotation");
        for (n, text) in ["neu", "eins", "zwei", "drei", "vier"].iter().enumerate() {
            fs::write(numbered(&dir, n), text).unwrap();
        }
        fs::write(dir.join("anderes.txt"), "bleibt").unwrap();
        let log = LogFile::start(&dir, "Driftless 0.1.0 (test)").unwrap();
        assert_eq!(log.path(), dir.join(FILE_NAME));
        assert_eq!(read(log.path()), "", "the file of this start begins empty");
        assert_eq!(read(&numbered(&dir, 1)), "neu");
        assert_eq!(read(&numbered(&dir, 2)), "eins");
        assert_eq!(read(&numbered(&dir, 3)), "zwei");
        assert_eq!(read(&numbered(&dir, 4)), "drei");
        assert!(!numbered(&dir, 5).exists(), "the oldest is deleted");
        assert_eq!(read(&dir.join("anderes.txt")), "bleibt");
        let mut names: Vec<String> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        names.sort();
        assert_eq!(
            names,
            [
                "anderes.txt",
                "driftless.1.log",
                "driftless.2.log",
                "driftless.3.log",
                "driftless.4.log",
                "driftless.log"
            ]
        );
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_start_in_a_new_folder_makes_it() {
        let root = tmp("neu");
        let dir = root.join("Logs").join("Driftless");
        let log = LogFile::start(&dir, "Driftless").unwrap();
        assert!(log.path().is_file());
        assert!(!numbered(&dir, 1).exists());
    }

    #[test]
    fn a_full_file_moves_on_and_at_most_five_stay() {
        let dir = tmp("voll");
        let log = LogFile::start_with_limit(&dir, "Driftless", 400).unwrap();
        let text = "x".repeat(60);
        for _ in 0..100 {
            log.write(Level::Info, &text);
        }
        for n in 0..KEEP {
            let size = fs::metadata(numbered(&dir, n)).unwrap().len();
            assert!(size > 0 && size <= 400, "{n}: {size} B");
        }
        assert!(!numbered(&dir, KEEP).exists());
        // the newest lines are in the file of this start
        assert!(read(log.path()).ends_with(&format!("{text}\n")));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_start_line_names_version_system_adapter_and_project() {
        let text = start_text(
            "Driftless 0.1.0 (abc123def)",
            "macOS 27.0 (aarch64)",
            Some("Apple M4 Pro (Metal)"),
            Some(Path::new("/Projekte/Halle")),
        );
        assert_eq!(
            text,
            "Driftless 0.1.0 (abc123def) · macOS 27.0 (aarch64) · Grafik: Apple M4 Pro (Metal) · \
             Projekt: /Projekte/Halle"
        );
        assert_eq!(
            start_text("Driftless 0.1.0 (x)", "Windows 10.0.26100 (x86_64)", None, None),
            "Driftless 0.1.0 (x) · Windows 10.0.26100 (x86_64) · Grafik: unbekannt · Projekt: keins"
        );
        let details = |d: &[&str]| d.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(
            gpu_text("Apple M4 Pro", &details(&["Metal", "IntegratedGpu", " "])),
            "Apple M4 Pro (Metal, IntegratedGpu)"
        );
        assert_eq!(
            gpu_text(
                "NVIDIA GeForce RTX 5080",
                &details(&["Vulkan", "DiscreteGpu", "NVIDIA 580.95.05"])
            ),
            "NVIDIA GeForce RTX 5080 (Vulkan, DiscreteGpu, NVIDIA 580.95.05)"
        );
        assert_eq!(gpu_text("Adapter", &[]), "Adapter");
        let os = os_text();
        assert!(
            os.ends_with(&format!("({})", std::env::consts::ARCH)),
            "{os}"
        );
        if cfg!(target_os = "macos") {
            assert!(
                os.starts_with("macOS 1") || os.starts_with("macOS 2"),
                "{os}"
            );
        }
        if cfg!(windows) {
            assert!(os.starts_with("Windows 10.0."), "{os}");
        }
    }

    #[test]
    fn each_start_writes_one_start_line_first() {
        let dir = tmp("start");
        let log = LogFile::start(&dir, "Driftless 0.1.0 (abc123def)").unwrap();
        log.write_start(
            Some("Apple M4 Pro (Metal)"),
            Some(Path::new("/Projekte/Halle")),
        );
        log.write_start(Some("anderes"), None);
        log.write(Level::Warning, "Bild 3 übersprungen\nzu unscharf");
        let text = read(log.path());
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        assert!(lines[0].ends_with(&format!(
            " Start   {}",
            start_text(
                "Driftless 0.1.0 (abc123def)",
                &os_text(),
                Some("Apple M4 Pro (Metal)"),
                Some(Path::new("/Projekte/Halle"))
            )
        )));
        assert!(
            lines[1].ends_with(" Warnung Bild 3 übersprungen"),
            "{}",
            lines[1]
        );
        assert_eq!(lines[2], "    zu unscharf");
        // the time in front: „2026-10-28 14:03:12.345“
        let time = &lines[0][..23];
        assert!(chrono::NaiveDateTime::parse_from_str(time, "%Y-%m-%d %H:%M:%S%.3f").is_ok());
        let _ = fs::remove_dir_all(&dir);

        // a line before the window: the start line without adapter and project first
        let dir = tmp("start-frueh");
        let log = LogFile::start(&dir, "Driftless 0.1.0 (abc123def)").unwrap();
        log.write(Level::Error, "Fehler: Fenster nicht startbar: kein Adapter");
        let text = read(log.path());
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2, "{text}");
        assert!(lines[0].contains(" Start   Driftless 0.1.0 (abc123def) · "));
        assert!(lines[0].ends_with(" · Grafik: unbekannt · Projekt: keins"));
        assert!(lines[1].ends_with(" Fehler  Fehler: Fenster nicht startbar: kein Adapter"));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn lines_have_time_kind_and_indented_continuations() {
        assert_eq!(
            format_line("2026-10-28 14:03:12.345", Level::Info, "SLAM fertig"),
            "2026-10-28 14:03:12.345 Info    SLAM fertig\n"
        );
        assert_eq!(
            format_line("t", Level::Panic, "eins\r\nzwei\n"),
            "t Absturz eins\n    zwei\n"
        );
        let tags: Vec<&str> = [
            Level::Start,
            Level::Info,
            Level::Warning,
            Level::Message,
            Level::Error,
            Level::Panic,
        ]
        .iter()
        .map(|l| l.tag())
        .collect();
        assert_eq!(
            tags,
            ["Start", "Info", "Warnung", "Meldung", "Fehler", "Absturz"]
        );
        assert!(tags.iter().all(|t| t.chars().count() <= 7));
    }

    #[test]
    fn the_folder_is_the_one_of_the_system() {
        let dir = default_dir().expect("HOME or LOCALAPPDATA is set");
        if cfg!(target_os = "macos") {
            assert!(dir.ends_with("Library/Logs/Driftless"), "{}", dir.display());
            assert!(dir.starts_with(std::env::var_os("HOME").unwrap()));
        }
        if cfg!(windows) {
            assert!(dir.ends_with("Driftless\\Logs"), "{}", dir.display());
            assert!(dir.starts_with(std::env::var_os("LOCALAPPDATA").unwrap()));
        }
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn the_macos_version_is_read_from_the_property_list() {
        let plist = "<dict>\n\t<key>ProductName</key>\n\t<string>macOS</string>\n\
                     \t<key>ProductVersion</key>\n\t<string>27.0</string>\n</dict>";
        assert_eq!(
            plist_string(plist, "ProductVersion").as_deref(),
            Some("27.0")
        );
        assert_eq!(plist_string(plist, "ProductName").as_deref(), Some("macOS"));
        assert_eq!(plist_string(plist, "BuildVersion"), None);
        assert!(os_version().is_some());
    }
}
