//! The commit `packaging/build_commit.rs` shows: a full hash (as CI passes it) is cut to the 9
//! characters of `git rev-parse --short=9`, anything else stays as given.

// The build script's functions, taken in as the build scripts do (`emit_commit` is not used).
#![allow(dead_code)]
include!("../../../packaging/build_commit.rs");

#[test]
fn full_hash_is_shortened() {
    let full = "2d9a150c0ffee0123456789abcdef0123456789a";
    assert_eq!(short_commit(full), "2d9a150c0");
    assert_eq!(short_commit(" 2D9A150C0FFEE "), "2D9A150C0");
    assert_eq!(short_commit("2d9a150c0"), "2d9a150c0");
    // Not a hash (a source archive may say anything): unchanged.
    assert_eq!(short_commit("v0.1.0-quelle"), "v0.1.0-quelle");
}
