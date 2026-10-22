//! Writing small files so that an interruption never leaves half a file behind.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// Writes `bytes` to `<path>.tmp` next to `path` and then renames it to `path`.
pub fn atomic_write(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let tmp = tmp_path(path);
    fs::write(&tmp, bytes)?;
    fs::rename(&tmp, path)
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
}
