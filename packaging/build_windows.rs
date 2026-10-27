// The Windows part of the `build.rs` of `driftless-app` and `driftless-cli`, taken in with
// `include!("../../packaging/build_windows.rs")`: puts the program symbol
// (`assets/icon/driftless.ico`) and the version information into the `.exe` as a resource, so
// that Explorer, the taskbar and the Start menu show it.
//
// Only a build on Windows does anything: `winresource` is a build dependency for Windows hosts
// only and calls the resource compiler `rc.exe` of the Windows SDK. On macOS this is empty.

/// Embeds the program symbol and the version information into the Windows program.
/// `description` is the „Beschreibung“ of the file (the name in the task manager).
#[cfg(windows)]
fn embed_windows_resources(description: &str) {
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return;
    }
    let manifest_dir = std::path::PathBuf::from(
        std::env::var_os("CARGO_MANIFEST_DIR").expect("Cargo sets CARGO_MANIFEST_DIR"),
    );
    // crates/<name> → the workspace root; joined part by part, so the path keeps `\` only.
    let ico = manifest_dir
        .parent()
        .and_then(|crates| crates.parent())
        .expect("the crate lies in crates/<name>")
        .join("assets")
        .join("icon")
        .join("driftless.ico");
    println!("cargo:rerun-if-changed={}", ico.display());
    let ico = ico.to_str().expect("the path of the symbol is valid UTF-8");
    let mut res = winresource::WindowsResource::new();
    res.set_icon(ico)
        .set("ProductName", "Driftless")
        .set("FileDescription", description);
    if let Err(e) = res.compile() {
        panic!("Programmsymbol nicht eingebettet (rc.exe des Windows SDK nötig): {e}");
    }
}

/// Nothing to embed outside Windows.
#[cfg(not(windows))]
fn embed_windows_resources(_description: &str) {}
