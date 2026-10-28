//! Sets `DRIFTLESS_COMMIT` (the commit every step report names under `programm`), see
//! `packaging/build_commit.rs`.

include!("../../packaging/build_commit.rs");

fn main() {
    emit_commit();
}
