//! The system's trash for „Löschen …“ and „Aufräumen“: a folder or file goes there and can be
//! taken back; nothing is deleted for good without the user's yes. Where the system has no
//! trash (on Windows a network share or a removable drive, where deleting would be for good),
//! nothing is deleted and the window says why. On Windows a folder larger than the recycle bin
//! may get (or on a drive whose bin is switched off) cannot be recycled: the `trash` crate
//! (5.2.9) deletes through `IFileOperation` with `FOF_NO_UI | FOF_ALLOWUNDO |
//! FOF_WANTNUKEWARNING`, and with `FOF_WANTNUKEWARNING` Windows asks before it destroys an item
//! instead of recycling it, despite `FOF_NO_UI` ([`crate::ui::text::DELETE_TOO_BIG_WINDOWS`]).
//! „Nein“ aborts it: nothing is deleted, the crate reports the abort and the window says so.

use crate::ui::text;
use std::path::Path;

/// How the window moves something to the trash (the system's way, or a test's).
pub type Trash = fn(&Path) -> Result<(), String>;

/// Moves `path` to the system's trash: on macOS through the file manager of the system (no
/// question to the Finder), on Windows into the recycle bin; the error in German.
pub fn system_trash(path: &Path) -> Result<(), String> {
    if let Some(why) = no_trash(path) {
        return Err(text::trash_unavailable(path, why));
    }
    context()
        .delete(path)
        .map_err(|e| text::trash_failed(path, &e.to_string()))
}

#[cfg(target_os = "macos")]
fn context() -> trash::TrashContext {
    use trash::macos::{DeleteMethod, TrashContextExtMacos};
    let mut ctx = trash::TrashContext::default();
    // the Finder would ask for the right to be controlled, and plays a sound
    ctx.set_delete_method(DeleteMethod::NsFileManager);
    ctx
}

#[cfg(not(target_os = "macos"))]
fn context() -> trash::TrashContext {
    trash::TrashContext::default()
}

/// Why `path` has no trash on Windows: a network share or a removable drive (Windows deletes
/// there for good).
#[cfg(windows)]
fn no_trash(path: &Path) -> Option<&'static str> {
    use std::os::windows::ffi::OsStrExt;
    use std::path::{Component, Prefix};
    #[link(name = "kernel32", kind = "raw-dylib")]
    extern "system" {
        fn GetDriveTypeW(root: *const u16) -> u32;
    }
    const REMOVABLE: u32 = 2;
    const REMOTE: u32 = 4;
    let root = match path.components().next() {
        Some(Component::Prefix(p)) => match p.kind() {
            Prefix::UNC(..) | Prefix::VerbatimUNC(..) => return Some("Netzlaufwerk"),
            Prefix::Disk(d) | Prefix::VerbatimDisk(d) => format!("{}:\\", char::from(d)),
            _ => return None,
        },
        _ => return None,
    };
    let wide: Vec<u16> = std::ffi::OsStr::new(&root)
        .encode_wide()
        .chain([0])
        .collect();
    // SAFETY: `wide` is a NUL-terminated wide string naming a drive root.
    match unsafe { GetDriveTypeW(wide.as_ptr()) } {
        REMOTE => Some("Netzlaufwerk"),
        REMOVABLE => Some("Wechseldatenträger"),
        _ => None,
    }
}

/// On macOS and Linux the system's call says itself when it has no trash for a place.
#[cfg(not(windows))]
fn no_trash(_path: &Path) -> Option<&'static str> {
    None
}
