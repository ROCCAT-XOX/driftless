//! Temporary folders of tests that clean up after themselves (feature `test-util`).
//!
//! A test that writes files gets a [`TestDir`]: a fresh, empty folder below the system's
//! temporary folder, removed with everything in it when the test is done – unless the test
//! fails: while its thread panics the folder is kept, so that what the test wrote can be looked
//! at. A folder shared by the tests of one test program (a scene built once and kept in a
//! `static`, which is never dropped) is a [`SharedDir`]: it is removed when the test program
//! exits; after a crash the next test program that makes a shared folder of the same name
//! removes it, once the process that made it has ended.
//!
//! Both deref to their [`PathBuf`], so they stand in for the path they replace: `dir.join(…)`,
//! `&dir` for a `&Path`, `dir.clone()` for a `PathBuf` of it.

use std::ffi::OsStr;
use std::fs::{self, File};
use std::ops::Deref;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, Once, PoisonError};

/// A fresh, empty folder for one test: removed when dropped, kept while the thread panics.
#[must_use = "dropping it removes the folder"]
#[derive(Debug)]
pub struct TestDir {
    /// The folder the test works in: the test folder or one inside it ([`TestDir::inside`]).
    path: PathBuf,
    /// The test folder, removed on drop.
    root: PathBuf,
}

impl TestDir {
    /// The folder `name` below the system's temporary folder (a name of its own per test and
    /// process, such as `driftless-export-<pid>-<test>`), emptied and created.
    ///
    /// # Panics
    /// If the folder cannot be created.
    pub fn new(name: impl AsRef<Path>) -> TestDir {
        let path = std::env::temp_dir().join(name);
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        TestDir {
            root: path.clone(),
            path,
        }
    }

    /// The folder `rel` inside (created): what derefs is that folder, what is removed on drop
    /// is still the whole test folder.
    ///
    /// # Panics
    /// If the folder cannot be created.
    pub fn inside(mut self, rel: impl AsRef<Path>) -> TestDir {
        self.path = self.path.join(rel);
        fs::create_dir_all(&self.path).unwrap_or_else(|e| panic!("{}: {e}", self.path.display()));
        self
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TestDir {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            let _ = fs::remove_dir_all(&self.root);
        }
    }
}

/// A folder shared by the tests of one test program, made once and kept in a `static` (never
/// dropped). It is removed when the program exits ([`remove_shared`]), also when a test of it
/// failed: a test's own folder ([`TestDir`]) is what stays for a look.
/// Next to it lies `<folder>.lock`, which the process holds locked while it runs (the system
/// releases the lock when the process ends); [`SharedDir::new`] first removes every folder of
/// the same prefix and name whose lock it can take, i.e. whose process has ended.
#[derive(Debug)]
pub struct SharedDir {
    path: PathBuf,
    /// The lock file, locked until the process ends.
    _held: File,
}

impl SharedDir {
    /// `<temp>/<prefix><process id>-<name>`, emptied and created, after removing such folders
    /// of earlier processes that have ended.
    ///
    /// # Panics
    /// If the folder or its lock cannot be made.
    pub fn new(prefix: &str, name: &str) -> SharedDir {
        let temp = std::env::temp_dir();
        remove_ended(&temp, prefix, name);
        let path = temp.join(format!("{prefix}{}-{name}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let lock = lock_file(&path);
        let held = File::create(&lock)
            .and_then(|f| f.lock().map(|()| f))
            .unwrap_or_else(|e| panic!("{}: {e}", lock.display()));
        remove_at_exit(&path);
        SharedDir { path, _held: held }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// The shared folders of this process, removed at its exit.
static SHARED: Mutex<Vec<PathBuf>> = Mutex::new(Vec::new());

/// Registers the shared folder `dir` for [`remove_shared`]; the first time also installs that
/// as an exit handler of the process.
fn remove_at_exit(dir: &Path) {
    static INSTALL: Once = Once::new();
    INSTALL.call_once(|| {
        // SAFETY: registers a plain function without arguments with the C runtime; it runs once
        // when the process exits normally (`exit`, also from a failed test program).
        unsafe {
            libc::atexit(remove_shared);
        }
    });
    SHARED
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .push(dir.to_path_buf());
}

/// At the exit of the test program: removes its shared folders with their lock files.
extern "C" fn remove_shared() {
    let dirs = SHARED.lock().unwrap_or_else(PoisonError::into_inner);
    for dir in dirs.iter() {
        let _ = fs::remove_dir_all(dir);
        let _ = fs::remove_file(lock_file(dir));
    }
}

/// `<folder>.lock` next to the shared folder `dir`.
pub fn lock_file(dir: &Path) -> PathBuf {
    let mut name = dir.file_name().unwrap_or_default().to_os_string();
    name.push(".lock");
    dir.with_file_name(name)
}

/// Removes the folders `<prefix><process id>-<name>` in `temp` of other processes, with their
/// lock file, whose lock file is free: their process has ended. A folder without a lock file
/// (not made by a [`SharedDir`]) or held by a running process is left alone.
fn remove_ended(temp: &Path, prefix: &str, name: &str) {
    let suffix = format!("-{name}.lock");
    let own = std::process::id().to_string();
    for entry in fs::read_dir(temp).into_iter().flatten().flatten() {
        let file_name = entry.file_name();
        let Some(pid) = file_name
            .to_str()
            .and_then(|n| n.strip_prefix(prefix))
            .and_then(|n| n.strip_suffix(&suffix))
        else {
            continue;
        };
        if pid.is_empty() || !pid.bytes().all(|b| b.is_ascii_digit()) || pid == own {
            continue;
        }
        let lock = entry.path();
        let Ok(file) = File::open(&lock) else {
            continue;
        };
        if file.try_lock().is_ok() {
            drop(file);
            let _ = fs::remove_dir_all(temp.join(format!("{prefix}{pid}-{name}")));
            let _ = fs::remove_file(&lock);
        }
    }
}

macro_rules! path_like {
    ($t:ty) => {
        impl Deref for $t {
            type Target = PathBuf;
            fn deref(&self) -> &PathBuf {
                &self.path
            }
        }

        impl AsRef<Path> for $t {
            fn as_ref(&self) -> &Path {
                &self.path
            }
        }

        impl AsRef<OsStr> for $t {
            fn as_ref(&self) -> &OsStr {
                self.path.as_os_str()
            }
        }
    };
}

path_like!(TestDir);
path_like!(SharedDir);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn removed_when_dropped() {
        let dir = TestDir::new(format!("driftless-io-testdir-{}", std::process::id()));
        let path = dir.to_path_buf();
        fs::write(dir.join("a.txt"), "a").unwrap();
        fs::create_dir_all(dir.join("b/c")).unwrap();
        assert!(path.join("b/c").is_dir());
        // it stands in for its path
        let clone: PathBuf = dir.clone();
        assert_eq!(clone, path);
        assert_eq!(dir.path(), path.as_path());
        drop(dir);
        assert!(!path.exists());
    }

    #[test]
    fn a_folder_inside_goes_with_the_test_folder() {
        let name = format!("driftless-io-testdir-innen-{}", std::process::id());
        let root = std::env::temp_dir().join(&name);
        let dir = TestDir::new(&name).inside("Büro Scan");
        assert_eq!(*dir, root.join("Büro Scan"));
        assert!(dir.is_dir());
        drop(dir);
        assert!(!root.exists());
    }

    /// A shared folder is gone when its test program has exited.
    #[test]
    fn shared_folder_goes_at_exit() {
        let name = format!("ende-{}", std::process::id());
        let made = std::env::temp_dir().join(format!("driftless-io-geteilt-{name}.gemacht"));
        let _ = fs::remove_file(&made);
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "testdir::tests::shared_folder_of_a_child",
                "--ignored",
            ])
            .env(CHILD, &name)
            .output()
            .unwrap();
        assert!(child.status.success(), "{child:?}");
        let path = PathBuf::from(fs::read_to_string(&made).unwrap());
        fs::remove_file(&made).unwrap();
        assert!(
            path.to_string_lossy().ends_with(&format!("-{name}")),
            "{}",
            path.display()
        );
        assert!(!path.exists(), "removed at exit: {}", path.display());
        assert!(!lock_file(&path).exists());
    }

    /// The name of the shared folder of [`shared_folder_of_a_child`].
    const CHILD: &str = "DRIFTLESS_TESTDIR_CHILD";

    /// For [`shared_folder_goes_at_exit`], run in a child process: makes a shared folder with a
    /// file in it, notes its path, and exits.
    #[test]
    #[ignore = "run by shared_folder_goes_at_exit"]
    fn shared_folder_of_a_child() {
        let Ok(name) = std::env::var(CHILD) else {
            return;
        };
        let shared = SharedDir::new("driftless-io-geteilt-", &name);
        fs::write(shared.join("szene.bin"), "x").unwrap();
        let made = std::env::temp_dir().join(format!("driftless-io-geteilt-{name}.gemacht"));
        fs::write(made, shared.to_string_lossy().as_bytes()).unwrap();
        // kept like a scene in a `static`: never dropped
        std::mem::forget(shared);
    }

    #[test]
    fn kept_when_the_test_fails() {
        let name = format!("driftless-io-testdir-panik-{}", std::process::id());
        let path = std::env::temp_dir().join(&name);
        let failed = std::thread::spawn(move || {
            let dir = TestDir::new(name);
            fs::write(dir.join("a.txt"), "a").unwrap();
            panic!("a failing test (expected: this test checks what such a test leaves)");
        })
        .join();
        assert!(failed.is_err());
        assert!(path.join("a.txt").is_file(), "kept for a look");
        fs::remove_dir_all(&path).unwrap();
    }

    #[test]
    fn emptied_when_made() {
        let name = format!("driftless-io-testdir-leer-{}", std::process::id());
        let old = std::env::temp_dir().join(&name);
        fs::create_dir_all(&old).unwrap();
        fs::write(old.join("alt.txt"), "alt").unwrap();
        let dir = TestDir::new(&name);
        assert_eq!(fs::read_dir(&*dir).unwrap().count(), 0);
    }

    /// A shared folder of a process that has ended goes with its lock file when the next one
    /// is made; one of a running process, one not made by a [`SharedDir`] and one of another
    /// name stay, and so does the new one after it is dropped (only a later process removes it).
    #[test]
    fn shared_folders_of_ended_processes_go() {
        let temp = std::env::temp_dir();
        let prefix = "driftless-io-geteilt-";
        let name = format!("szene-{}", std::process::id());
        let folder = |pid: &str| temp.join(format!("{prefix}{pid}-{name}"));
        // a process that has ended: its lock file is there, nobody holds it
        let ended = folder("1");
        fs::create_dir_all(&ended).unwrap();
        fs::write(ended.join("daten.bin"), "x").unwrap();
        fs::write(lock_file(&ended), "").unwrap();
        // a running process: its lock held by another open file (as by another process)
        let running = folder("2");
        fs::create_dir_all(&running).unwrap();
        let held = File::create(lock_file(&running)).unwrap();
        held.lock().unwrap();
        // not made by a shared folder: no lock file
        let foreign = folder("3");
        fs::create_dir_all(&foreign).unwrap();
        let other = temp.join(format!("{prefix}4-anders-{name}"));
        fs::create_dir_all(&other).unwrap();
        fs::write(lock_file(&other), "").unwrap();

        let shared = SharedDir::new(prefix, &name);
        assert_eq!(*shared, folder(&std::process::id().to_string()));
        assert!(lock_file(&shared).is_file());
        assert_eq!(
            fs::read_dir(&*shared).unwrap().count(),
            0,
            "nothing in the folder"
        );
        assert!(
            !ended.exists() && !lock_file(&ended).exists(),
            "ended process"
        );
        assert!(running.is_dir(), "running process");
        assert!(foreign.is_dir(), "not a shared folder");
        assert!(other.is_dir(), "another name");
        let path = shared.to_path_buf();
        drop(shared);
        assert!(path.is_dir(), "kept by its own process");
        drop(held);
        for d in [&running, &foreign, &other, &path] {
            fs::remove_dir_all(d).unwrap();
            let _ = fs::remove_file(lock_file(d));
        }
    }
}
