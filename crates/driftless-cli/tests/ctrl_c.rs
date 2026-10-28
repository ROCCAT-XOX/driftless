//! Ctrl+C in `driftless`: the first one cancels the running step through its progress (the
//! step stops at its next check and cleans up as on any cancel, the program says so in
//! German and exits with 130), a second one ends the program at once, also with 130.
//!
//! Unix only: the tests send SIGINT to a child process. On Windows the handler is only
//! compiled (`ctrlc` maps Ctrl+C to the same callback there).
#![cfg(unix)]

mod common;

use common::{room_bag, tmp};
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{channel, Receiver};
use std::time::Duration;

fn driftless() -> Command {
    Command::new(assert_cmd::cargo::cargo_bin("driftless"))
}

fn run(args: &[&std::ffi::OsStr]) {
    let status = driftless().args(args).status().unwrap();
    assert!(status.success(), "{args:?}");
}

/// A scan with SLAM and a map in `<dir>/out`.
fn mapped(name: &str) -> (driftless_io::testdir::TestDir, std::path::PathBuf) {
    let dir = tmp(name);
    room_bag(&dir, true);
    let out = dir.join("out");
    run(&[
        "slam".as_ref(),
        dir.as_os_str(),
        "-o".as_ref(),
        out.as_os_str(),
    ]);
    run(&[
        "map".as_ref(),
        dir.as_os_str(),
        "--slam".as_ref(),
        out.as_os_str(),
    ]);
    (dir, out)
}

/// `driftless map <dir> --slam <out> --no-outliers` (another cleaning: the map is computed
/// again), held at the start of the clean step (the debug build's test hook
/// `DRIFTLESS_TEST_HALT`) until it is cancelled, or for good with `stubborn`. Returns once it
/// halts; the lines of its stderr keep coming through the receiver.
fn halted_map(dir: &Path, out: &Path, stubborn: bool) -> (Child, Receiver<String>) {
    let mut cmd = driftless();
    cmd.arg("map")
        .arg(dir)
        .arg("--slam")
        .arg(out)
        .arg("--no-outliers")
        .env("DRIFTLESS_TEST_HALT", "Geister")
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    if stubborn {
        cmd.env("DRIFTLESS_TEST_HALT_STUR", "1");
    }
    let mut child = cmd.spawn().unwrap();
    let stderr = child.stderr.take().unwrap();
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        for line in BufReader::new(stderr).lines() {
            if tx.send(line.unwrap()).is_err() {
                break;
            }
        }
    });
    let halted = rx
        .recv_timeout(Duration::from_secs(120))
        .expect("the map halts");
    assert_eq!(halted, "Test: hält in Phase Geister");
    (child, rx)
}

fn sigint(child: &Child) {
    let status = Command::new("kill")
        .args(["-INT", &child.id().to_string()])
        .status()
        .unwrap();
    assert!(status.success());
}

/// The exit code of `child`, which has to end within `secs`.
fn exit_code(child: &mut Child, secs: u64) -> i32 {
    for _ in 0..secs * 100 {
        if let Some(status) = child.try_wait().unwrap() {
            return status.code().expect("exits, not killed by the signal");
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let _ = child.kill();
    panic!("still running after {secs} s");
}

/// Every line still to come from the child's stderr.
fn rest(rx: &Receiver<String>) -> String {
    rx.iter().collect::<Vec<_>>().join("\n")
}

/// Files below `dir` whose name ends in `.tmp`.
fn leftovers(dir: &Path) -> Vec<String> {
    std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".tmp"))
        .collect()
}

#[test]
fn ctrl_c_cancels_the_step() {
    let (dir, out) = mapped("strg-c");
    // Held in the clean step, after `dense.bin` was written again: the old map.json is gone.
    let (mut child, rx) = halted_map(&dir, &out, false);
    sigint(&child);
    assert_eq!(exit_code(&mut child, 60), 130);
    let stderr = rest(&rx);
    assert!(stderr.contains("Strg+C: wird abgebrochen"), "{stderr}");
    assert!(stderr.contains("Abgebrochen mit Strg+C"), "{stderr}");
    assert!(!stderr.contains("Fehler"), "{stderr}");
    assert!(!out.join("map.json").exists(), "the half map has no report");
    assert!(leftovers(&out).is_empty(), "{:?}", leftovers(&out));
    // The same command again finishes the map.
    run(&[
        "map".as_ref(),
        dir.as_os_str(),
        "--slam".as_ref(),
        out.as_os_str(),
        "--no-outliers".as_ref(),
    ]);
    assert!(out.join("map.json").is_file());
}

#[test]
fn second_ctrl_c_exits_at_once() {
    let (dir, out) = mapped("strg-c-zweimal");
    // A step that does not stop: only the second Ctrl+C ends it.
    let (mut child, rx) = halted_map(&dir, &out, true);
    sigint(&child);
    let first = rx.recv_timeout(Duration::from_secs(30)).unwrap();
    assert!(
        first.contains("noch einmal Strg+C beendet sofort"),
        "{first}"
    );
    std::thread::sleep(Duration::from_millis(300));
    assert!(child.try_wait().unwrap().is_none(), "still running");
    sigint(&child);
    assert_eq!(exit_code(&mut child, 10), 130);
    let stderr = rest(&rx);
    assert!(stderr.contains("sofort beendet"), "{stderr}");
}
