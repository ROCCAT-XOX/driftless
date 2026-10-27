//! Sets `DRIFTLESS_COMMIT` (the commit shown by `--version`), see `packaging/build_commit.rs`,
//! and on Windows embeds the program symbol, see `packaging/build_windows.rs`.

include!("../../packaging/build_commit.rs");
include!("../../packaging/build_windows.rs");

fn main() {
    emit_commit();
    embed_windows_resources("Driftless");
}
