//! Detection of the recordings in a scan project folder.

use std::path::{Path, PathBuf};

/// Errors that make a folder unusable as a project.
#[derive(Debug, thiserror::Error)]
pub enum ProjectError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Ordner nicht gefunden: {0}")]
    NotFound(PathBuf),
    #[error("Kein Ordner: {0}")]
    NotADirectory(PathBuf),
    #[error("Keine LiDAR-Aufnahme (LIDAR_*.bag) in {0}")]
    NoLidarBag(PathBuf),
}

/// A scan folder (RayStudio project or MicroSD copy).
#[derive(Debug, Clone, PartialEq)]
pub struct Project {
    pub root: PathBuf,
    pub lidar_bags: Vec<PathBuf>,
    pub image_bags: Vec<PathBuf>,
    pub calibration: Option<PathBuf>,
    /// Missing parts, in German for display to the user.
    pub problems: Vec<String>,
}

/// Splits a name into digit and non-digit runs for natural ordering.
fn natural_key(name: &str) -> Vec<(u8, u64, String)> {
    let mut key = Vec::new();
    let mut chars = name.chars().peekable();
    while let Some(&c) = chars.peek() {
        let digit = c.is_ascii_digit();
        let mut run = String::new();
        while let Some(&d) = chars.peek() {
            if d.is_ascii_digit() != digit {
                break;
            }
            run.push(d);
            chars.next();
        }
        if digit {
            // Very long digit runs saturate; ties fall back to the text.
            key.push((0, run.parse().unwrap_or(u64::MAX), run));
        } else {
            key.push((1, 0, run));
        }
    }
    key
}

fn is_bag(name: &str, prefix: &str) -> bool {
    name.starts_with(prefix)
        && Path::new(name)
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("bag"))
}

/// Finds `LIDAR_*.bag`, `IMAGE_*.bag` and `calibration/calib.json` in `root`.
/// Missing camera bags or calibration are reported in `problems`.
pub fn discover_project(root: &Path) -> Result<Project, ProjectError> {
    if !root.exists() {
        return Err(ProjectError::NotFound(root.to_path_buf()));
    }
    if !root.is_dir() {
        return Err(ProjectError::NotADirectory(root.to_path_buf()));
    }
    let mut lidar_bags = Vec::new();
    let mut image_bags = Vec::new();
    for entry in std::fs::read_dir(root)? {
        let entry = entry?;
        if !entry.file_type()?.is_file() {
            continue;
        }
        let name = entry.file_name().to_string_lossy().into_owned();
        if is_bag(&name, "LIDAR_") {
            lidar_bags.push((name, entry.path()));
        } else if is_bag(&name, "IMAGE_") {
            image_bags.push((name, entry.path()));
        }
    }
    let sorted = |mut v: Vec<(String, PathBuf)>| -> Vec<PathBuf> {
        v.sort_by_cached_key(|(n, _)| natural_key(n));
        v.into_iter().map(|(_, p)| p).collect()
    };
    let lidar_bags = sorted(lidar_bags);
    let image_bags = sorted(image_bags);
    if lidar_bags.is_empty() {
        return Err(ProjectError::NoLidarBag(root.to_path_buf()));
    }
    let calib = root.join("calibration").join("calib.json");
    let calibration = calib.is_file().then_some(calib);

    let mut problems = Vec::new();
    if image_bags.is_empty() {
        problems.push("Kamera-Aufnahme (IMAGE_*.bag) fehlt".to_string());
    }
    if calibration.is_none() {
        problems.push("Kalibrierung (calibration/calib.json) fehlt".to_string());
    }
    Ok(Project {
        root: root.to_path_buf(),
        lidar_bags,
        image_bags,
        calibration,
        problems,
    })
}
