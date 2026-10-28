//! The panic hook of the log file ([`logfile::install_panic_hook`]), in a test program of its
//! own: a panic hook is one for the whole process.

use driftless_app::logfile::{self, LogFile};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

/// How often the hook installed before ran.
static PREVIOUS_HOOK: AtomicUsize = AtomicUsize::new(0);

#[test]
fn a_panic_goes_into_the_file_with_place_and_backtrace_then_to_the_hook_before() {
    let dir = driftless_io::testscan::tmp("logfile-absturz");
    let log = Arc::new(LogFile::start(&dir, "Driftless 0.1.0 (test)").unwrap());
    // the hook before: counts instead of printing
    std::panic::set_hook(Box::new(|_| {
        PREVIOUS_HOOK.fetch_add(1, Ordering::SeqCst);
    }));
    logfile::install_panic_hook(log.clone());
    // a job thread, as `driftless_app::jobs` names them
    let result = std::thread::Builder::new()
        .name("job: Testschritt".into())
        .spawn(|| panic!("Absturz im Test: {}", 42))
        .unwrap()
        .join();
    // back to the default hook
    drop(std::panic::take_hook());
    assert!(result.is_err());
    assert_eq!(
        PREVIOUS_HOOK.load(Ordering::SeqCst),
        1,
        "the hook before ran too"
    );

    let text = std::fs::read_to_string(log.path()).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert!(lines.len() > 5, "{text}");
    // the start line first: nothing was written before the panic
    assert!(
        lines[0].contains(" Start   Driftless 0.1.0 (test) · "),
        "{}",
        lines[0]
    );
    assert!(
        lines[1].ends_with(" Absturz Thread „job: Testschritt“: Absturz im Test: 42"),
        "{}",
        lines[1]
    );
    assert!(
        lines[2].starts_with(&format!("    Ort: {}:", file!())),
        "{}",
        lines[2]
    );
    assert_eq!(lines[3], "    Backtrace:");
    // the frames, indented, with the test that panicked among them
    assert!(
        lines[4..].iter().all(|l| l.starts_with("    ")),
        "indented: {text}"
    );
    let frames = lines[4..]
        .iter()
        .filter(|l| {
            let l = l.trim_start();
            l.split_once(": ")
                .is_some_and(|(n, _)| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
        })
        .count();
    assert!(frames >= 3, "{frames} frames: {text}");
    assert!(
        text.contains("a_panic_goes_into_the_file_with_place_and_backtrace"),
        "{text}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}
