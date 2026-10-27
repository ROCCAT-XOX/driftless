//! Sets `DRIFTLESS_COMMIT` (the commit shown by `--version`), see `packaging/build_commit.rs`.

include!("../../packaging/build_commit.rs");

fn main() {
    emit_commit();
}
