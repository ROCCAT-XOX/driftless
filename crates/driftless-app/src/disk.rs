//! How much room a project takes and how much its drive has left: the size of each run, map,
//! surface and model folder, the whole project folder, the raw data of each scan, and the free
//! space of the drive ([`measure`], off the UI thread; it walks the folders). Below
//! [`LOW_SPACE`], or below what the next step will need ([`need`], an estimate from the raw data
//! of its scans), the window warns.

use crate::ui::Tree;
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

/// Free space below which the window warns whatever comes next (10 GB).
pub const LOW_SPACE: u64 = 10_000_000_000;

/// The disk use of a project, as measured for one version of the tree.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DiskUse {
    /// Bytes per folder of the tree: runs, maps, surfaces and models (everything below each).
    pub folders: BTreeMap<PathBuf, u64>,
    /// The whole project folder; a file linked into two folders counts once where the system
    /// tells it (macOS, Linux).
    pub total: u64,
    /// Free space on the drive of the project folder; `None` if the system does not say.
    pub free: Option<u64>,
    /// The raw data of each scan folder (its bags).
    pub raw: BTreeMap<PathBuf, u64>,
}

/// The identity of a file on its drive, to count a file linked twice once: device and inode
/// on Unix; `None` elsewhere (every link then counts).
#[cfg(unix)]
fn file_id(meta: &fs::Metadata) -> Option<(u64, u64)> {
    use std::os::unix::fs::MetadataExt;
    Some((meta.dev(), meta.ino()))
}

#[cfg(not(unix))]
fn file_id(_meta: &fs::Metadata) -> Option<(u64, u64)> {
    None
}

/// The bytes of the files below `dir` (links not followed); with `seen`, a file already seen
/// there (by its identity) is not counted again.
fn walk(dir: &Path, mut seen: Option<&mut HashSet<(u64, u64)>>) -> u64 {
    let Ok(entries) = fs::read_dir(dir) else {
        return 0;
    };
    let mut bytes = 0;
    for entry in entries.flatten() {
        let Ok(meta) = entry.metadata() else {
            continue;
        };
        if meta.is_dir() {
            bytes += walk(&entry.path(), seen.as_deref_mut());
        } else if meta.is_file() {
            let new = match (&mut seen, file_id(&meta)) {
                (Some(seen), Some(id)) => seen.insert(id),
                _ => true,
            };
            if new {
                bytes += meta.len();
            }
        }
    }
    bytes
}

/// The bytes of the files below `dir` (every link counts, as the folder holds them).
pub fn folder_size(dir: &Path) -> u64 {
    walk(dir, None)
}

/// The bytes of the files below `dir`, a file linked twice counted once where the system tells
/// it (what deleting all of it would free).
pub fn total_size(dir: &Path) -> u64 {
    walk(dir, Some(&mut HashSet::new()))
}

/// The free space on the drive of `dir` for this user; `None` if the system does not say.
#[cfg(unix)]
pub fn free_space(dir: &Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let path = std::ffi::CString::new(dir.as_os_str().as_bytes()).ok()?;
    let mut stat = std::mem::MaybeUninit::<libc::statvfs>::zeroed();
    // SAFETY: `path` is a NUL-terminated string, `stat` a statvfs the call fills.
    let ok = unsafe { libc::statvfs(path.as_ptr(), stat.as_mut_ptr()) } == 0;
    // SAFETY: filled by the successful call.
    let stat = ok.then(|| unsafe { stat.assume_init() })?;
    // the field types differ between the systems (u32 blocks on macOS, u64 on Linux)
    #[allow(clippy::unnecessary_cast)]
    let free = (stat.f_bavail as u64).checked_mul(stat.f_frsize as u64);
    free
}

/// The free space on the drive of `dir` for this user; `None` if the system does not say.
#[cfg(windows)]
pub fn free_space(dir: &Path) -> Option<u64> {
    use std::os::windows::ffi::OsStrExt;
    #[link(name = "kernel32", kind = "raw-dylib")]
    extern "system" {
        fn GetDiskFreeSpaceExW(
            directory: *const u16,
            free_to_caller: *mut u64,
            total: *mut u64,
            free_total: *mut u64,
        ) -> i32;
    }
    let wide: Vec<u16> = dir.as_os_str().encode_wide().chain([0]).collect();
    let mut free = 0u64;
    // SAFETY: `wide` is a NUL-terminated wide string, `free` a u64 the call fills; the other
    // two results are not asked for (null).
    let ok = unsafe {
        GetDiskFreeSpaceExW(
            wide.as_ptr(),
            &mut free,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    (ok != 0).then_some(free)
}

/// The free space on the drive of `dir`; this system does not say.
#[cfg(not(any(unix, windows)))]
pub fn free_space(_dir: &Path) -> Option<u64> {
    None
}

/// The folders of `tree` whose size the window shows: every run, map, surface and model.
pub fn tree_folders(tree: &Tree) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for run in tree.scans.iter().flat_map(|s| &s.runs) {
        out.push(run.dir.clone());
        for map in &run.maps {
            out.push(map.dir.clone());
            out.extend(map.surfaces.iter().map(|s| s.dir.clone()));
        }
    }
    for model in &tree.models {
        out.push(model.dir.clone());
        out.extend(model.surfaces.iter().map(|s| s.dir.clone()));
    }
    out
}

/// The bytes of the bags of a scan folder.
fn raw_size(scan: &Path) -> u64 {
    match driftless_io::project::discover_project(scan) {
        Ok(found) => found
            .lidar_bags
            .iter()
            .chain(&found.image_bags)
            .filter_map(|b| fs::metadata(b).ok())
            .map(|m| m.len())
            .sum(),
        Err(_) => 0,
    }
}

/// How the free space of a drive is told (the system's way, or a test's).
pub type FreeSpace = fn(&Path) -> Option<u64>;

/// Measures the disk use of the project in `dir` with the folders `folders` (its tree) and the
/// scan folders `scans`; `free` tells the free space. Walks every folder: call it off the UI
/// thread.
pub fn measure(dir: &Path, folders: &[PathBuf], scans: &[PathBuf], free: FreeSpace) -> DiskUse {
    DiskUse {
        folders: folders
            .iter()
            .map(|d| (d.clone(), folder_size(d)))
            .collect(),
        total: total_size(dir),
        free: free(dir),
        raw: scans.iter().map(|s| (s.clone(), raw_size(s))).collect(),
    }
}

/// What a step will write, at most, estimated from the raw data `raw` of its scans (bytes) and
/// its voxel size (m): a SLAM run a tenth of it, a map a fifth at 1 cm (twice that at 5 mm), a
/// surface a tenth at 1 cm (a quarter of that at 2 cm), a model a fifth. Measured on the
/// office and the black rooms (analysis of 28.10.: 0.3–0.5 of the raw data for a whole scan
/// at the defaults); rounded up, as the warning should come early.
pub fn need(kind: NeedKind, raw: u64, voxel: f64) -> u64 {
    let raw = raw as f64;
    let fine = |exp: i32| (0.01 / voxel.clamp(0.002, 0.1)).powi(exp);
    let bytes = match kind {
        NeedKind::Slam => raw * 0.1,
        NeedKind::Map => raw * 0.2 * fine(1).clamp(0.5, 4.0),
        NeedKind::Surface => raw * 0.1 * fine(2).clamp(0.1, 4.0),
        NeedKind::Model => raw * 0.2,
    };
    bytes.ceil() as u64
}

/// The kinds of steps [`need`] estimates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeedKind {
    Slam,
    Map,
    Surface,
    Model,
}
