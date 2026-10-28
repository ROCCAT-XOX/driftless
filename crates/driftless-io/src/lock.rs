//! A lock file for a file that several programs may change: two windows of the program on one
//! project, the window and the command line on one run, two colleagues on a project on a
//! network share. `<name>.lock` next to the file says who holds it; it is held only while the
//! file is read, changed and written (milliseconds), and a lock older than [`LOCK_STALE`] is
//! taken to be left by a program that ended without removing it.

use std::fmt;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime};

/// A lock this old was left behind (its holder ended without removing it).
pub const LOCK_STALE: Duration = Duration::from_secs(30);
/// How long [`FileLock::acquire`] waits for another holder before it gives up.
pub const LOCK_WAIT: Duration = Duration::from_millis(1500);
/// How often a held lock is tried again.
const RETRY: Duration = Duration::from_millis(20);

/// The lock of a file, held until dropped.
#[derive(Debug)]
pub struct FileLock {
    path: PathBuf,
}

/// Why a lock could not be taken (German, for display).
#[derive(Debug)]
pub struct LockError {
    /// The file the lock is for.
    pub file: PathBuf,
    /// What the lock file says of its holder, if another holds it.
    pub holder: Option<String>,
    /// Why the lock file could not be made, if that was the reason.
    pub error: Option<io::Error>,
}

impl fmt::Display for LockError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = self.file.file_name().map_or_else(
            || self.file.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        match (&self.holder, &self.error) {
            (Some(holder), _) => write!(
                f,
                "{name} wird gerade von einem anderen Fenster oder Programm geändert ({holder}); \
                 gleich noch einmal versuchen\n{}",
                self.file.display()
            ),
            (None, Some(e)) => write!(
                f,
                "{name} nicht sperrbar: {e}\n{}",
                lock_path(&self.file).display()
            ),
            (None, None) => write!(f, "{name} nicht sperrbar"),
        }
    }
}

impl std::error::Error for LockError {}

/// `<file>.lock`.
fn lock_path(file: &Path) -> PathBuf {
    let mut name = file.file_name().unwrap_or_default().to_os_string();
    name.push(".lock");
    file.with_file_name(name)
}

/// Who holds a lock taken now: „Prozess 1234 · Benutzer anna · Rechner Werkstatt-PC“ (user and
/// computer as far as the environment names them).
pub fn holder_text() -> String {
    let var = |name: &str| std::env::var(name).ok().filter(|v| !v.trim().is_empty());
    let mut text = format!("Prozess {}", std::process::id());
    if let Some(user) = var("USER").or_else(|| var("USERNAME")) {
        text += &format!(" · Benutzer {user}");
    }
    if let Some(host) = var("COMPUTERNAME").or_else(|| var("HOSTNAME")) {
        text += &format!(" · Rechner {host}");
    }
    text
}

/// Whether the lock file at `path` is older than `stale`.
fn is_stale(path: &Path, stale: Duration) -> bool {
    fs::metadata(path)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| SystemTime::now().duration_since(t).ok())
        .is_some_and(|age| age > stale)
}

impl FileLock {
    /// Takes the lock of `file`, waiting up to [`LOCK_WAIT`] while another holds it; a lock
    /// left behind ([`LOCK_STALE`]) is taken over.
    pub fn acquire(file: &Path) -> Result<FileLock, LockError> {
        FileLock::acquire_within(file, LOCK_WAIT, LOCK_STALE)
    }

    /// [`FileLock::acquire`] with another wait and stale time.
    pub fn acquire_within(
        file: &Path,
        wait: Duration,
        stale: Duration,
    ) -> Result<FileLock, LockError> {
        let path = lock_path(file);
        let deadline = Instant::now() + wait;
        loop {
            match fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
            {
                Ok(mut f) => {
                    // who holds it, for the message of the next one; the lock holds without
                    let _ = f.write_all(holder_text().as_bytes());
                    return Ok(FileLock { path });
                }
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                    if is_stale(&path, stale) {
                        let _ = fs::remove_file(&path);
                        continue;
                    }
                    if Instant::now() >= deadline {
                        let holder = fs::read_to_string(&path)
                            .ok()
                            .map(|t| t.trim().to_string())
                            .filter(|t| !t.is_empty())
                            .unwrap_or_else(|| "unbekannt".into());
                        return Err(LockError {
                            file: file.to_path_buf(),
                            holder: Some(holder),
                            error: None,
                        });
                    }
                    std::thread::sleep(RETRY);
                }
                Err(e) => {
                    return Err(LockError {
                        file: file.to_path_buf(),
                        holder: None,
                        error: Some(e),
                    })
                }
            }
        }
    }
}

impl Drop for FileLock {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}
