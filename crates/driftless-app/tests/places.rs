//! Where projects go: the start folder of the dialogs and the cloud storage a project should
//! not lie in ([`driftless_app::places`]).

use driftless_app::places::{cloud_by_path, in_icloud, start_folder, PROJECTS_FOLDER};
use driftless_io::testscan::tmp;
use std::path::Path;

/// No OneDrive in the environment.
fn none(_: &str) -> Option<String> {
    None
}

#[test]
fn cloud_storage_by_its_path() {
    let home = Path::new("/Users/anna");
    let cloud = |p: &str| cloud_by_path(Path::new(p), Some(home), &none);
    assert_eq!(
        cloud("/Users/anna/Library/Mobile Documents/com~apple~CloudDocs/Projekte/Halle"),
        Some("iCloud Drive".into())
    );
    assert_eq!(
        cloud("/Users/anna/Library/CloudStorage/OneDrive-FirmaGmbH/Halle"),
        Some("OneDrive".into())
    );
    assert_eq!(
        cloud("/Users/anna/Library/CloudStorage/Dropbox/Halle"),
        Some("Dropbox".into())
    );
    assert_eq!(
        cloud("/Users/anna/Library/CloudStorage/GoogleDrive-anna@example.org/Meine Ablage"),
        Some("Google Drive".into())
    );
    // a local folder, and one that only starts with the same letters
    assert_eq!(cloud("/Users/anna/Driftless Projekte/Halle"), None);
    assert_eq!(cloud("/Users/anna/Library/Mobile Documents2/x"), None);
    assert_eq!(cloud("/Volumes/Daten/Projekte"), None);

    // Windows: the folders OneDrive names in the environment, compared as Windows does
    let env = |name: &str| match name {
        "OneDrive" => Some(r"C:\Users\anna\OneDrive - Firma".to_string()),
        "OneDriveConsumer" => Some(r"C:\Users\anna\OneDrive".to_string()),
        _ => None,
    };
    let windows = |p: &str| cloud_by_path(Path::new(p), None, &env);
    assert_eq!(
        windows(r"c:\users\anna\onedrive - firma\Projekte\Halle"),
        Some("OneDrive".into())
    );
    assert_eq!(
        windows(r"C:\Users\anna\OneDrive\Dokumente\Halle"),
        Some("OneDrive".into())
    );
    assert_eq!(windows(r"C:\Users\anna\Driftless Projekte\Halle"), None);
    assert_eq!(windows(r"C:\Users\anna\OneDriveAlt\Halle"), None);
}

/// iCloud Drive as the system tells it: a temporary folder is not in it; where this computer
/// has iCloud Drive, its folder is (read only, nothing is written there).
#[test]
fn icloud_as_the_system_tells_it() {
    assert!(!in_icloud(&tmp("orte-lokal")));
    assert!(!in_icloud(Path::new("/gibt/es/nicht")));
    let Some(home) = std::env::var_os("HOME") else {
        return;
    };
    let drive = Path::new(&home).join("Library/Mobile Documents/com~apple~CloudDocs");
    if cfg!(target_os = "macos") && drive.is_dir() {
        assert!(in_icloud(&drive), "{}", drive.display());
    } else {
        eprintln!("ÜBERSPRUNGEN: kein iCloud Drive auf diesem Rechner");
    }
}

/// The dialogs start in the folder used last, while it is there; else in the projects folder,
/// made when first needed.
#[test]
fn dialogs_start_in_the_last_folder_or_the_projects_folder() {
    let dir = tmp("orte-start");
    let home = dir.join("home").join(PROJECTS_FOLDER);
    assert!(!home.exists());
    assert_eq!(start_folder(None, Some(home.clone())), Some(home.clone()));
    assert!(home.is_dir(), "made when first needed");
    let last = dir.join("Projekte von Anna");
    std::fs::create_dir_all(&last).unwrap();
    assert_eq!(
        start_folder(Some(&last), Some(home.clone())),
        Some(last.clone())
    );
    std::fs::remove_dir_all(&last).unwrap();
    assert_eq!(start_folder(Some(&last), Some(home.clone())), Some(home));
    assert_eq!(start_folder(None, None), None);
    let _ = std::fs::remove_dir_all(&dir);
}
