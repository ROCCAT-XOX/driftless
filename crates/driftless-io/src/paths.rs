//! Paths as the project files store them, the same on every system: the plain form of a
//! Windows path (without the `\\?\` that resolving adds), a path relative to a folder with
//! `/`, and reading a stored path back. The functions on text treat Unix and Windows paths
//! alike on every system, so the Windows cases are tested everywhere.

use std::borrow::Cow;
use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};

/// Windows device names, which name no file whatever the extension.
const WINDOWS_DEVICES: [&str; 22] = [
    "con", "prn", "aux", "nul", "com1", "com2", "com3", "com4", "com5", "com6", "com7", "com8",
    "com9", "lpt1", "lpt2", "lpt3", "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9",
];

/// Whether a part of a verbatim path means the same without the prefix: not `.` or `..`, no
/// dot or space at the end, no device name, no character Windows refuses in a name (`/` is
/// a separator only outside a verbatim path).
fn plain_part(part: &str) -> bool {
    let stem = part
        .split('.')
        .next()
        .unwrap_or("")
        .trim_end()
        .to_lowercase();
    !part.is_empty()
        && part != "."
        && part != ".."
        && !part.ends_with(['.', ' '])
        && !part
            .chars()
            .any(|c| c < ' ' || matches!(c, '<' | '>' | ':' | '"' | '/' | '|' | '?' | '*'))
        && !WINDOWS_DEVICES.contains(&stem.as_str())
}

/// The plain form of a Windows verbatim path, as the `dunce` crate makes it for drives, and
/// for shares as well: `\\?\C:\…` → `C:\…`, `\\?\UNC\server\share\…` → `\\server\share\…`.
/// A verbatim path that would mean something else without the prefix stays as it is (a part
/// `.` or `..`, one with a dot or space at the end, a device name such as `NUL`, a `/`), and
/// so do the other kinds (`\\?\Volume{…}\`, `\\.\…`) and every path that is not verbatim.
/// Rust's file functions add the prefix again themselves where a long path needs it.
pub fn plain_form(text: &str) -> Cow<'_, str> {
    let Some(rest) = text.strip_prefix(r"\\?\") else {
        return Cow::Borrowed(text);
    };
    let unc = rest
        .get(..4)
        .filter(|p| p.eq_ignore_ascii_case(r"UNC\"))
        .map(|_| &rest[4..]);
    // The start in its plain form, and the rest below it (if any).
    let (head, tail) = if let Some(share) = unc {
        let mut parts = share.splitn(3, '\\');
        let (Some(server), Some(name)) = (parts.next(), parts.next()) else {
            return Cow::Borrowed(text);
        };
        if !plain_part(server) || !plain_part(name) {
            return Cow::Borrowed(text);
        }
        (format!(r"\\{server}\{name}"), parts.next())
    } else {
        let b = rest.as_bytes();
        if !(b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && b[2] == b'\\') {
            return Cow::Borrowed(text);
        }
        (rest[..2].to_string(), Some(&rest[3..]))
    };
    let mut out = head;
    if let Some(tail) = tail {
        if !tail.split('\\').filter(|p| !p.is_empty()).all(plain_part) {
            return Cow::Borrowed(text);
        }
        out.push('\\');
        out.push_str(tail);
    }
    Cow::Owned(out)
}

/// The absolute, resolved form of an existing path, as [`fs::canonicalize`] gives it but in
/// its plain form ([`plain_form`]) on Windows: `C:\…` and `\\server\share\…`, not `\\?\…`.
pub fn canonical(path: &Path) -> io::Result<PathBuf> {
    let p = fs::canonicalize(path)?;
    if cfg!(windows) {
        if let Some(Cow::Owned(plain)) = p.to_str().map(plain_form) {
            return Ok(PathBuf::from(plain));
        }
    }
    Ok(p)
}

/// Absolute on Windows: `C:\…`, `C:/…`, `\\server\…` or `//server/…` (also read on another
/// system, where such a path is just not found instead of being taken as relative).
fn windows_absolute(text: &str) -> bool {
    let b = text.as_bytes();
    let drive =
        b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'/' | b'\\');
    drive || text.starts_with("\\\\") || text.starts_with("//")
}

/// Where an absolute path starts: the Unix root, a drive or a share.
#[derive(Debug)]
enum Root<'a> {
    Unix,
    /// The drive letter, upper case.
    Drive(u8),
    /// Server and share.
    Share(&'a str, &'a str),
}

impl Root<'_> {
    /// The same drive or share: letters, server and share names ignore case, as on Windows.
    fn same(&self, other: &Root) -> bool {
        let same_name = |a: &str, b: &str| a.to_lowercase() == b.to_lowercase();
        match (self, other) {
            (Root::Unix, Root::Unix) => true,
            (Root::Drive(a), Root::Drive(b)) => a == b,
            (Root::Share(s, n), Root::Share(t, m)) => same_name(s, t) && same_name(n, m),
            _ => false,
        }
    }
}

/// An absolute path as its root and its parts (`.` left out); `None` for a relative one,
/// one with a `..` (only resolved paths are compared) and any other kind of path.
fn absolute_parts(text: &str) -> Option<(Root<'_>, Vec<&str>)> {
    fn parts<'a>(rest: &'a str, seps: &[char]) -> Option<Vec<&'a str>> {
        let parts: Vec<&str> = rest
            .split(seps)
            .filter(|p| !p.is_empty() && *p != ".")
            .collect();
        (!parts.contains(&"..")).then_some(parts)
    }
    let windows = ['\\', '/'];
    let b = text.as_bytes();
    if b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && matches!(b[2], b'/' | b'\\') {
        return Some((
            Root::Drive(b[0].to_ascii_uppercase()),
            parts(&text[3..], &windows)?,
        ));
    }
    let share = text.strip_prefix(r"\\").or_else(|| text.strip_prefix("//"));
    if let Some(share) = share {
        let mut it = share.splitn(3, windows);
        let (server, name) = (it.next()?, it.next()?);
        if server.is_empty() || name.is_empty() || matches!(server, "?" | ".") {
            return None;
        }
        let rest = parts(it.next().unwrap_or(""), &windows)?;
        return Some((Root::Share(server, name), rest));
    }
    let rest = text.strip_prefix('/')?;
    Some((Root::Unix, parts(rest, &['/'])?))
}

/// `path` relative to the folder `base`, with `/` and `..` for each folder up: `scans/a` or
/// `../../Rohdaten/a`; `.` for `base` itself. Both must be absolute paths of the same kind
/// and root: Unix, the same drive or the same share (a verbatim one in its plain form,
/// [`plain_form`]); `None` across drives, servers or shares, and for anything else. Works on
/// the text alone, the same on every system.
pub fn relative_text(base: &str, path: &str) -> Option<String> {
    let (base, path) = (plain_form(base), plain_form(path));
    let (root_a, a) = absolute_parts(&base)?;
    let (root_b, b) = absolute_parts(&path)?;
    if !root_a.same(&root_b) {
        return None;
    }
    let common = a.iter().zip(&b).take_while(|(x, y)| x == y).count();
    let mut out: Vec<&str> = vec![".."; a.len() - common];
    out.extend(&b[common..]);
    Some(if out.is_empty() {
        ".".to_string()
    } else {
        out.join("/")
    })
}

/// Whether `a` and `b` lie on the same volume as far as the system can tell: on Unix the
/// device of both (where both can be read); on Windows the drive or share, which
/// [`relative_text`] compares.
fn same_volume(a: &Path, b: &Path) -> bool {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if let (Ok(x), Ok(y)) = (fs::metadata(a), fs::metadata(b)) {
            return x.dev() == y.dev();
        }
    }
    let _ = (a, b);
    true
}

/// `path` relative to the folder `base` ([`relative_text`]) to store next to the absolute
/// path, if one can be computed: both on the same volume, drive or share, both valid text.
pub fn relative_to(base: &Path, path: &Path) -> Option<String> {
    if !same_volume(base, path) {
        return None;
    }
    relative_text(base.to_str()?, path.to_str()?)
}

/// A path read from a project file: absolute as written (a verbatim Windows path in its plain
/// form), else relative to `base`, with `/` or `\` between the parts. Absolute is also a path
/// of the other system: a Windows one read on Unix, and one that starts at the root (`/…`,
/// a Unix path read on Windows), which a relative stored path never does. `.` is left out,
/// and `..` takes the last part off `base` (in the text: a verbatim Windows path would take
/// `..` literally); above the root it stays at the root, above a relative `base` it is kept.
pub fn stored_path(base: &Path, text: &str) -> PathBuf {
    let rooted = text.starts_with(['/', '\\']);
    if rooted || Path::new(text).is_absolute() || windows_absolute(text) {
        return PathBuf::from(plain_form(text).into_owned());
    }
    let mut out = base.to_path_buf();
    for part in text
        .split(['/', '\\'])
        .filter(|s| !s.is_empty() && *s != ".")
    {
        if part != ".." {
            out.push(part);
            continue;
        }
        match out.components().next_back() {
            Some(Component::Normal(_)) => {
                out.pop();
            }
            Some(Component::RootDir | Component::Prefix(_)) => {}
            Some(Component::CurDir | Component::ParentDir) | None => out.push(".."),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testdir::TestDir;

    #[test]
    fn plain_form_of_drives_and_shares() {
        assert_eq!(plain_form(r"\\?\C:\Projekte\Halle"), r"C:\Projekte\Halle");
        assert_eq!(plain_form(r"\\?\d:\"), r"d:\");
        assert_eq!(
            plain_form(r"\\?\UNC\nas\projekte\Halle 2026\scans"),
            r"\\nas\projekte\Halle 2026\scans"
        );
        assert_eq!(plain_form(r"\\?\unc\nas\p"), r"\\nas\p");
        assert_eq!(plain_form(r"\\?\UNC\nas\p\"), r"\\nas\p\");
        // Already plain, or no Windows path at all: unchanged.
        for text in [r"C:\x", r"\\nas\p\x", "/Users/y/x", "relativ/x", ""] {
            assert_eq!(plain_form(text), text);
            assert!(matches!(plain_form(text), Cow::Borrowed(_)));
        }
    }

    #[test]
    fn plain_form_keeps_what_would_change_its_meaning() {
        for text in [
            r"\\?\C:\a\..\b",
            r"\\?\C:\a\.\b",
            r"\\?\C:\Ordner.",
            r"\\?\C:\Ordner \x",
            r"\\?\C:\x\NUL",
            r"\\?\C:\x\con.txt",
            r"\\?\C:\a/b",
            r"\\?\C:",
            r"\\?\Volume{0b3a4b6e-0000-0000-0000-100000000000}\x",
            r"\\.\C:\x",
            r"\\?\UNC\nas",
            r"\\?\UNC\nas\p.\x",
            r"\\?\UNC\nas\p\a\..\b",
        ] {
            assert_eq!(plain_form(text), text, "{text}");
        }
    }

    #[test]
    fn relative_text_on_unix() {
        let rel = |a: &str, b: &str| relative_text(a, b);
        assert_eq!(rel("/p", "/p/a/b").as_deref(), Some("a/b"));
        assert_eq!(
            rel("/p/modelle/m1", "/p/scans/a").as_deref(),
            Some("../../scans/a")
        );
        assert_eq!(rel("/p", "/q/a").as_deref(), Some("../q/a"));
        assert_eq!(rel("/p/x", "/p/x").as_deref(), Some("."));
        assert_eq!(rel("/p/", "/p//a/./b/").as_deref(), Some("a/b"));
        assert_eq!(rel("/", "/a").as_deref(), Some("a"));
        // Only absolute, resolved paths.
        assert_eq!(rel("p", "/p/a"), None);
        assert_eq!(rel("/p", "a"), None);
        assert_eq!(rel("/p", "/p/../a"), None);
        // A backslash is part of a Unix name.
        assert_eq!(rel("/p", r"/p/a\b").as_deref(), Some(r"a\b"));
    }

    #[test]
    fn relative_text_on_drives() {
        let rel = |a: &str, b: &str| relative_text(a, b);
        assert_eq!(
            rel(r"C:\Projekte\Halle", r"C:\Projekte\Halle\scans\a").as_deref(),
            Some("scans/a")
        );
        assert_eq!(
            rel(r"C:\Projekte\Halle\modelle\m1", r"c:\Rohdaten\S1").as_deref(),
            Some("../../../../Rohdaten/S1")
        );
        assert_eq!(rel("C:/p", r"C:\p\a").as_deref(), Some("a"));
        assert_eq!(rel(r"\\?\C:\p", r"C:\p\a").as_deref(), Some("a"));
        // Another drive, or a drive and a share: none.
        assert_eq!(rel(r"C:\p", r"D:\p\a"), None);
        assert_eq!(rel(r"C:\p", r"\\nas\p\a"), None);
        assert_eq!(rel(r"C:\p", "/p/a"), None);
    }

    #[test]
    fn relative_text_on_shares() {
        let rel = |a: &str, b: &str| relative_text(a, b);
        assert_eq!(
            rel(
                r"\\nas\projekte\Halle\modelle",
                r"\\nas\projekte\Halle\scans\a"
            )
            .as_deref(),
            Some("../scans/a")
        );
        // Verbatim paths, as resolving gives them, compare in their plain form; server and
        // share ignore case, the folders below them do not.
        assert_eq!(
            rel(
                r"\\?\UNC\nas\projekte\Halle\modelle\m1",
                r"\\NAS\Projekte\Halle\scans\a\slam-001\karte-001"
            )
            .as_deref(),
            Some("../../scans/a/slam-001/karte-001")
        );
        assert_eq!(
            rel(r"\\nas\p\Halle", r"\\nas\p\halle\a").as_deref(),
            Some("../halle/a")
        );
        assert_eq!(rel("//nas/p/x", r"\\nas\p\y").as_deref(), Some("../y"));
        assert_eq!(rel(r"\\nas\p", r"\\nas\p").as_deref(), Some("."));
        // Never across a server or a share.
        assert_eq!(rel(r"\\nas\p\x", r"\\other\p\x"), None);
        assert_eq!(rel(r"\\nas\p\x", r"\\nas\q\x"), None);
        assert_eq!(rel(r"\\nas", r"\\nas\p"), None);
    }

    #[test]
    fn stored_paths_resolve_dot_dot_in_the_text() {
        let base = Path::new("/p/modelle");
        assert_eq!(stored_path(base, "../scans/a"), Path::new("/p/scans/a"));
        assert_eq!(stored_path(base, r"..\scans\a"), Path::new("/p/scans/a"));
        assert_eq!(stored_path(base, "./a//b/"), Path::new("/p/modelle/a/b"));
        assert_eq!(stored_path(base, "../../../../x"), Path::new("/x"));
        // From the root: absolute on every system (a Unix path read on Windows as well).
        assert_eq!(stored_path(base, "/q/a"), PathBuf::from("/q/a"));
        assert_eq!(stored_path(base, r"\q\a"), PathBuf::from(r"\q\a"));
        // Above a relative folder `..` stays.
        assert_eq!(stored_path(Path::new("m"), "../../a"), Path::new("../a"));
        assert_eq!(stored_path(Path::new(""), "../a"), Path::new("../a"));
        // Windows paths are absolute everywhere; a verbatim one is read in its plain form.
        for (text, read) in [
            (r"C:\Daten\Scan", r"C:\Daten\Scan"),
            (r"\\server\freigabe\scan", r"\\server\freigabe\scan"),
            (r"\\?\UNC\server\freigabe\scan", r"\\server\freigabe\scan"),
            (r"\\?\D:\x", r"D:\x"),
        ] {
            assert_eq!(stored_path(base, text), PathBuf::from(read), "{text}");
        }
    }

    /// A share opened on Windows: `..` in a stored path is taken off in the text, so the
    /// folder is found even below a verbatim base, which would take `..` literally.
    #[cfg(windows)]
    #[test]
    fn stored_paths_below_a_verbatim_share() {
        let base = Path::new(r"\\?\UNC\nas\p\modelle");
        assert_eq!(
            stored_path(base, "../scans/a"),
            Path::new(r"\\?\UNC\nas\p\scans\a")
        );
        let base = Path::new(r"\\nas\p\modelle");
        assert_eq!(
            stored_path(base, "../scans/a"),
            Path::new(r"\\nas\p\scans\a")
        );
    }

    #[test]
    fn canonical_is_absolute_and_plain() {
        let dir = TestDir::new(format!("driftless-io-paths-{}", std::process::id()));
        fs::create_dir_all(dir.join("a")).unwrap();
        let p = canonical(&dir.join("a").join("..").join("a")).unwrap();
        assert!(p.is_absolute());
        assert!(!p.to_string_lossy().starts_with(r"\\?\"), "{}", p.display());
        assert_eq!(p, canonical(&dir.join("a")).unwrap());
        assert!(canonical(&dir.join("fehlt")).is_err());
        let base = canonical(&dir).unwrap();
        assert_eq!(relative_to(&base, &p).as_deref(), Some("a"));
        assert_eq!(relative_to(&p, &base).as_deref(), Some(".."));
        // Not resolved or no text: none.
        assert_eq!(relative_to(Path::new("rel"), &p), None);
        fs::remove_dir_all(&dir).unwrap();
    }
}
