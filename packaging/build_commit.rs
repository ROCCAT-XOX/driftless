// The common part of the `build.rs` of `driftless-app` and `driftless-cli`, taken in with
// `include!("../../packaging/build_commit.rs")`: sets `DRIFTLESS_COMMIT` for `env!`.
//
// The commit is the environment variable `DRIFTLESS_COMMIT` if set (CI, builds from a source
// archive; a full hash is cut to 9 characters), else `git rev-parse --short=9 HEAD`, else
// „unbekannt“. Cargo runs the script again when the variable changes or when `HEAD`, the
// branch it points to or `packed-refs` change.

/// `commit` trimmed; a hash longer than 9 hex digits cut to 9, as `git rev-parse --short=9`.
fn short_commit(commit: &str) -> String {
    let commit = commit.trim();
    if commit.len() > 9 && commit.bytes().all(|b| b.is_ascii_hexdigit()) {
        commit[..9].to_string()
    } else {
        commit.to_string()
    }
}

/// Sets `DRIFTLESS_COMMIT` and tells Cargo when to run the build script again.
fn emit_commit() {
    println!("cargo:rerun-if-env-changed=DRIFTLESS_COMMIT");
    let manifest_dir = std::path::PathBuf::from(
        std::env::var_os("CARGO_MANIFEST_DIR").expect("Cargo sets CARGO_MANIFEST_DIR"),
    );
    let git = |args: &[&str]| -> Option<String> {
        let out = std::process::Command::new("git")
            .args(args)
            .current_dir(&manifest_dir)
            .output()
            .ok()?;
        let s = String::from_utf8(out.stdout).ok()?;
        let s = s.trim();
        (out.status.success() && !s.is_empty()).then(|| s.to_string())
    };
    // A relative answer of git is relative to where it ran.
    let git_path = |args: &[&str]| git(args).map(|p| manifest_dir.join(p));
    if let Some(git_dir) = git_path(&["rev-parse", "--git-dir"]) {
        // Branches live in the common folder (the main one for a linked worktree).
        let common = git_path(&["rev-parse", "--git-common-dir"]).unwrap_or(git_dir.clone());
        let head = git_dir.join("HEAD");
        let mut watched = vec![head.clone()];
        if let Ok(text) = std::fs::read_to_string(&head) {
            if let Some(name) = text.trim().strip_prefix("ref: ") {
                watched.push(git_dir.join(name));
                watched.push(common.join(name));
            }
        }
        watched.push(common.join("packed-refs"));
        watched.dedup();
        for path in watched.iter().filter(|p| p.exists()) {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }
    let commit = std::env::var("DRIFTLESS_COMMIT")
        .ok()
        .map(|c| short_commit(&c))
        .filter(|c| !c.is_empty())
        .or_else(|| git(&["rev-parse", "--short=9", "HEAD"]))
        .unwrap_or_else(|| "unbekannt".to_string());
    println!("cargo:rustc-env=DRIFTLESS_COMMIT={commit}");
}
