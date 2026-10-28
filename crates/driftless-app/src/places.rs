//! Where projects go, and where they had better not: the start folder of the dialogs
//! ([`start_folder`]: `~/Driftless Projekte`, on Windows `%USERPROFILE%\Driftless Projekte`,
//! made when first needed, then the folder used last), and whether a folder lies in cloud
//! storage that uploads large files and moves them off the disk when space runs low
//! ([`cloud_storage`]: iCloud Drive, also `~/Documents` and `~/Desktop` with „Schreibtisch &
//! Dokumente“, OneDrive, the providers under `~/Library/CloudStorage`). Such a file loads many
//! times slower once it was moved away.

use std::path::{Path, PathBuf};

/// The folder for projects in the home folder.
pub const PROJECTS_FOLDER: &str = "Driftless Projekte";

/// The home folder: `USERPROFILE` on Windows, `HOME` elsewhere.
pub fn home() -> Option<PathBuf> {
    let var = if cfg!(windows) { "USERPROFILE" } else { "HOME" };
    std::env::var_os(var)
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
}

/// `<home>/Driftless Projekte` (not made here).
pub fn projects_home() -> Option<PathBuf> {
    home().map(|h| h.join(PROJECTS_FOLDER))
}

/// The folder a dialog opens in: `last` if it is still a folder, else `default` (the projects
/// folder), made if missing; `None` if neither is there and `default` cannot be made (the
/// dialog then opens where the system wants).
pub fn start_folder(last: Option<&Path>, default: Option<PathBuf>) -> Option<PathBuf> {
    if let Some(last) = last.filter(|l| l.is_dir()) {
        return Some(last.to_path_buf());
    }
    let default = default?;
    std::fs::create_dir_all(&default).ok()?;
    default.is_dir().then_some(default)
}

/// The text of a path for comparing it with another: `\` as `/`, without a `/` at the end,
/// and on a Windows path (a drive or a share) in lower case, as Windows compares names.
fn comparable(path: &str) -> String {
    let text = path.replace('\\', "/");
    let text = text.trim_end_matches('/').to_string();
    let b = text.as_bytes();
    let windows = (b.len() >= 2 && b[1] == b':') || text.starts_with("//");
    if windows {
        text.to_lowercase()
    } else {
        text
    }
}

/// Whether `path` is `root` or lies below it (as text, [`comparable`]).
fn below(path: &str, root: &str) -> bool {
    let (path, root) = (comparable(path), comparable(root));
    !root.is_empty() && (path == root || path.starts_with(&format!("{root}/")))
}

/// The name of a provider folder of `~/Library/CloudStorage` (`OneDrive-Firma`, `Dropbox`,
/// `GoogleDrive-name@…`, `Box-Box`), as the warning says it.
fn provider(folder: &str) -> String {
    let lower = folder.to_lowercase();
    for (start, name) in [
        ("onedrive", "OneDrive"),
        ("dropbox", "Dropbox"),
        ("googledrive", "Google Drive"),
        ("box", "Box"),
        ("icloud", "iCloud Drive"),
    ] {
        if lower.starts_with(start) {
            return name.to_string();
        }
    }
    folder.to_string()
}

/// The cloud storage `dir` lies in, told by its path alone: `~/Library/Mobile Documents` (iCloud
/// Drive), a provider under `~/Library/CloudStorage`, or the OneDrive folders Windows names in
/// `OneDrive`, `OneDriveCommercial` and `OneDriveConsumer` (`var` reads the environment). Both
/// kinds on every system, so the rules are tested everywhere.
pub fn cloud_by_path(
    dir: &Path,
    home: Option<&Path>,
    var: &dyn Fn(&str) -> Option<String>,
) -> Option<String> {
    let path = dir.to_string_lossy();
    if let Some(home) = home {
        let library = home.join("Library");
        if below(&path, &library.join("Mobile Documents").to_string_lossy()) {
            return Some("iCloud Drive".into());
        }
        let storage = library.join("CloudStorage");
        if let Ok(rest) = dir.strip_prefix(&storage) {
            let folder = rest
                .components()
                .next()
                .map(|c| c.as_os_str().to_string_lossy().into_owned())
                .unwrap_or_default();
            return Some(provider(&folder));
        }
    }
    let onedrive = ["OneDrive", "OneDriveCommercial", "OneDriveConsumer"]
        .iter()
        .filter_map(|v| var(v))
        .any(|root| below(&path, &root));
    onedrive.then(|| "OneDrive".into())
}

/// Whether the system keeps `dir` in iCloud Drive: on macOS the resource value
/// `NSURLIsUbiquitousItemKey` of the folder, which is also true for `~/Documents` and
/// `~/Desktop` when „Schreibtisch & Dokumente“ is synced; false elsewhere.
pub fn in_icloud(dir: &Path) -> bool {
    ubiquitous(dir)
}

#[cfg(target_os = "macos")]
fn ubiquitous(dir: &Path) -> bool {
    use objc2::rc::Retained;
    use objc2::runtime::AnyObject;
    use objc2_foundation::{NSNumber, NSString, NSURLIsUbiquitousItemKey, NSURL};
    let Some(text) = dir.to_str() else {
        return false;
    };
    let url = NSURL::fileURLWithPath(&NSString::from_str(text));
    let mut value: Option<Retained<AnyObject>> = None;
    // SAFETY: the key is a constant of Foundation; its value is an NSNumber (a BOOL), and
    // `value` takes any object, which is only read as an NSNumber after checking its class.
    let read = unsafe { url.getResourceValue_forKey_error(&mut value, NSURLIsUbiquitousItemKey) };
    read.is_ok()
        && value
            .as_deref()
            .and_then(|v| v.downcast_ref::<NSNumber>())
            .is_some_and(|n| n.boolValue())
}

#[cfg(not(target_os = "macos"))]
fn ubiquitous(_dir: &Path) -> bool {
    false
}

/// The cloud storage the project folder `dir` lies in: by its path ([`cloud_by_path`]), else
/// iCloud Drive if the system keeps it there ([`in_icloud`]); `None` for a local folder.
/// Asks the system on macOS: call it off the UI thread.
pub fn cloud_storage(dir: &Path) -> Option<String> {
    let var = |name: &str| std::env::var(name).ok().filter(|v| !v.trim().is_empty());
    cloud_by_path(dir, home().as_deref(), &var)
        .or_else(|| in_icloud(dir).then(|| "iCloud Drive".into()))
}
