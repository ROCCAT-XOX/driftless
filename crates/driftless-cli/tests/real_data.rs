//! `driftless slam` and `driftless map` on the long Promedia scan in `$DRIFTLESS_TESTDATA`
//! (see `docs/testdaten.md`): every map step in its own process, so that `peak_rss_bytes` in
//! `map.json` is the peak of that step alone. Skips when the data is missing.
//!
//! The bounds are the goals of Meilenstein 4; the measured values are in
//! `docs/ergebnisse/2026-10-21-meilenstein-4.md`.
//!
//! `cargo test --release -p driftless-cli --test real_data -- --ignored --nocapture`

use driftless_io::testdata;
use serde_json::Value;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Peak memory of every map step on Promedia at 1 cm (bytes; measured up to 3.8 GB).
const PROMEDIA_PEAK_RSS: u64 = 4_000_000_000;
/// Run time of the three map steps on Promedia together (s; measured ≈ 6 min).
const PROMEDIA_MAP_SECONDS: f64 = 15.0 * 60.0;

/// A temporary output folder that is removed on drop, also when the test panics.
struct TempDir(PathBuf);

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Runs the `driftless` binary with `args` and fails the test with its output if it fails.
fn driftless(args: &[&str]) {
    let out = Command::new(env!("CARGO_BIN_EXE_driftless"))
        .args(args)
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "driftless {args:?} failed:\n{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

fn report(dir: &Path) -> Value {
    serde_json::from_str(&std::fs::read_to_string(dir.join("map.json")).unwrap()).unwrap()
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn promedia_map_under_4_gb_and_15_min() {
    let Some(scan) = testdata::path("jmk6-2026-10-20/20261020173822").filter(|p| p.exists()) else {
        eprintln!("SKIPPED: {} not set or Promedia missing", testdata::ENV_VAR);
        return;
    };
    let guard = TempDir(
        std::env::temp_dir().join(format!("driftless-cli-promedia-{}", std::process::id())),
    );
    std::fs::create_dir_all(&guard.0).unwrap();
    let (scan, out) = (scan.to_str().unwrap(), guard.0.to_str().unwrap());
    driftless(&["slam", scan, "-o", out]);

    let mut total = 0.0;
    for step in ["dense", "clean", "color"] {
        driftless(&["map", scan, "--slam", out, "--step", step]);
        let r = report(&guard.0);
        let entry = &r["steps"][step];
        assert_eq!(entry["reused"], Value::Bool(false), "{step}: {entry}");
        let seconds = entry["seconds"].as_f64().unwrap();
        total += seconds;
        let peak = r["peak_rss_bytes"].as_u64().expect("peak memory reported");
        println!(
            "Promedia {step}: {seconds:.1} s, peak {:.2} GB",
            peak as f64 / 1e9
        );
        assert!(peak < PROMEDIA_PEAK_RSS, "{step}: peak {peak} bytes");
    }
    println!("Promedia map: {total:.1} s");
    assert!(total < PROMEDIA_MAP_SECONDS, "map took {total:.0} s");
}
