//! Writing small files so that an interruption never leaves half a file behind.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

/// Writes `bytes` to `<path>.tmp` next to `path`, flushes it to the disk and then renames it
/// to `path`. On an error the `.tmp` is removed and `path` is left as it was.
pub fn atomic_write(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let tmp = tmp_path(path);
    let result = (|| {
        let mut file = fs::File::create(&tmp)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        fs::rename(&tmp, path)
    })();
    if result.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    result
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
