//! The lock file of a file several programs may change ([`driftless_io::lock`]).

use driftless_io::lock::{FileLock, LOCK_STALE};
use driftless_io::testscan::tmp;
use std::fs;
use std::time::{Duration, SystemTime};

#[test]
fn a_lock_is_held_until_dropped() {
    let dir = tmp("sperre-halten");
    let file = dir.join("project.toml");
    let lock = FileLock::acquire(&file).unwrap();
    let lock_file = dir.join("project.toml.lock");
    assert!(lock_file.is_file());
    let holder = fs::read_to_string(&lock_file).unwrap();
    assert!(
        holder.contains(&format!("Prozess {}", std::process::id())),
        "{holder}"
    );

    // a second writer waits, then gives up with a German message naming the file and holder
    let started = std::time::Instant::now();
    let e = FileLock::acquire_within(&file, Duration::from_millis(150), LOCK_STALE).unwrap_err();
    assert!(started.elapsed() >= Duration::from_millis(150));
    let message = e.to_string();
    assert!(
        message.starts_with(
            "project.toml wird gerade von einem anderen Fenster oder Programm geändert"
        ),
        "{message}"
    );
    assert!(
        message.contains(&format!("Prozess {}", std::process::id())),
        "{message}"
    );

    drop(lock);
    assert!(!lock_file.exists(), "removed when dropped");
    let again = FileLock::acquire(&file).unwrap();
    drop(again);
    let _ = fs::remove_dir_all(&dir);
}

/// A lock whose holder died (older than the stale time) is taken over.
#[test]
fn a_stale_lock_is_taken_over() {
    let dir = tmp("sperre-alt");
    let file = dir.join("corrections.toml");
    let lock_file = dir.join("corrections.toml.lock");
    fs::write(&lock_file, "Prozess 1 · Benutzer niemand").unwrap();
    let old = SystemTime::now() - LOCK_STALE - Duration::from_secs(5);
    fs::File::options()
        .write(true)
        .open(&lock_file)
        .unwrap()
        .set_modified(old)
        .unwrap();
    let lock = FileLock::acquire_within(&file, Duration::from_millis(50), LOCK_STALE).unwrap();
    let holder = fs::read_to_string(&lock_file).unwrap();
    assert!(
        holder.contains(&format!("Prozess {}", std::process::id())),
        "{holder}"
    );
    drop(lock);
    let _ = fs::remove_dir_all(&dir);
}

/// Two threads that each read, change and write the same file under the lock lose nothing.
#[test]
fn writers_under_the_lock_lose_nothing() {
    let dir = tmp("sperre-zaehlen");
    let file = dir.join("zaehler.txt");
    fs::write(&file, "0").unwrap();
    let threads: Vec<_> = (0..4)
        .map(|_| {
            let file = file.clone();
            std::thread::spawn(move || {
                for _ in 0..25 {
                    let _lock = FileLock::acquire(&file).unwrap();
                    let n: u32 = fs::read_to_string(&file).unwrap().parse().unwrap();
                    fs::write(&file, (n + 1).to_string()).unwrap();
                }
            })
        })
        .collect();
    for t in threads {
        t.join().unwrap();
    }
    assert_eq!(fs::read_to_string(&file).unwrap(), "100");
    let _ = fs::remove_dir_all(&dir);
}
