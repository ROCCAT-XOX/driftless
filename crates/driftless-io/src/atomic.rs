//! Writing files so that an interruption never leaves half a file behind.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

/// Writes `bytes` to `<path>.tmp` next to `path`, flushes it to the disk and then renames it
/// to `path`. On an error the `.tmp` is removed and `path` is left as it was.
pub fn atomic_write(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let temp = TempPath::new(path);
    let mut file = fs::File::create(temp.tmp())?;
    file.write_all(bytes)?;
    drop(file);
    temp.commit()
}

/// A file written as `<name>.tmp` next to its target and renamed to the target only by
/// [`TempPath::commit`]. Dropped without a commit (an error, a cancel), it removes the `.tmp`;
/// the target is left as it was either way until the commit.
///
/// Whoever writes the `.tmp` closes it before the commit or the drop (Windows renames and
/// removes only closed files reliably).
#[derive(Debug)]
pub struct TempPath {
    path: PathBuf,
    tmp: PathBuf,
    committed: bool,
}

impl TempPath {
    pub fn new(path: &Path) -> TempPath {
        TempPath {
            path: path.to_path_buf(),
            tmp: tmp_path(path),
            committed: false,
        }
    }

    /// `<name>.tmp` next to the target.
    pub fn tmp(&self) -> &Path {
        &self.tmp
    }

    /// The target.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Flushes the `.tmp` to the disk and renames it to the target (replacing it). On an
    /// error the `.tmp` is removed.
    pub fn commit(mut self) -> io::Result<()> {
        // Opened for writing: Windows refuses `FlushFileBuffers` on a read-only handle.
        fs::OpenOptions::new()
            .write(true)
            .open(&self.tmp)?
            .sync_all()?;
        fs::rename(&self.tmp, &self.path)?;
        self.committed = true;
        Ok(())
    }
}

impl Drop for TempPath {
    fn drop(&mut self) {
        if !self.committed {
            let _ = fs::remove_file(&self.tmp);
        }
    }
}

fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".tmp");
    path.with_file_name(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_through_a_temporary_file() {
        assert_eq!(
            tmp_path(Path::new("a/corrections.toml")),
            Path::new("a/corrections.toml.tmp")
        );
        let dir = std::env::temp_dir().join(format!("driftless-io-atomic-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("stand.toml");
        atomic_write(&path, b"nr = 1\n").unwrap();
        atomic_write(&path, b"nr = 2\n").unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"nr = 2\n");
        assert!(!dir.join("stand.toml.tmp").exists());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn temp_path_commit_and_drop() {
        let dir =
            std::env::temp_dir().join(format!("driftless-io-temp-path-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("wolke.laz");
        let temp = TempPath::new(&path);
        assert_eq!(temp.path(), path);
        assert_eq!(temp.tmp(), dir.join("wolke.laz.tmp"));
        fs::write(temp.tmp(), b"neu").unwrap();
        temp.commit().unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"neu");
        assert!(!dir.join("wolke.laz.tmp").exists());

        // Without a commit the `.tmp` goes on drop and the target stays as it was.
        let temp = TempPath::new(&path);
        fs::write(temp.tmp(), b"halb").unwrap();
        drop(temp);
        assert!(!dir.join("wolke.laz.tmp").exists());
        assert_eq!(fs::read(&path).unwrap(), b"neu");

        // A failed commit (the target is a non-empty folder) removes the `.tmp` as well.
        let blocked = dir.join("ordner.las");
        fs::create_dir_all(blocked.join("x")).unwrap();
        let temp = TempPath::new(&blocked);
        fs::write(temp.tmp(), b"x").unwrap();
        assert!(temp.commit().is_err());
        assert!(!dir.join("ordner.las.tmp").exists());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn failed_write_leaves_no_temporary_file() {
        let dir =
            std::env::temp_dir().join(format!("driftless-io-atomic-fail-{}", std::process::id()));
        // The target is a non-empty folder: the rename fails.
        let path = dir.join("bericht.json");
        fs::create_dir_all(path.join("x")).unwrap();
        assert!(atomic_write(&path, b"{}").is_err());
        assert!(!dir.join("bericht.json.tmp").exists());
        fs::remove_dir_all(&dir).unwrap();
    }
}
