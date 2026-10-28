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

/// A left lock that cannot be removed – here in a folder this program may not write, as on a
/// share where it is another user's – is waited for like a held one and then refused, never
/// tried again at once (before, the loop spun forever at full load); the German message names
/// the lock file and says to delete it by hand once no one else has the project open.
#[cfg(unix)]
#[test]
fn a_stale_lock_that_cannot_be_removed_is_refused() {
    use std::os::unix::fs::PermissionsExt;
    let dir = tmp("sperre-nicht-entfernbar");
    let file = dir.join("project.toml");
    let lock_file = dir.join("project.toml.lock");
    fs::write(&lock_file, "Prozess 1 · Benutzer anna").unwrap();
    let old = SystemTime::now() - LOCK_STALE - Duration::from_secs(5);
    fs::File::options()
        .write(true)
        .open(&lock_file)
        .unwrap()
        .set_modified(old)
        .unwrap();
    let writable = |mode: u32| fs::set_permissions(&dir, fs::Permissions::from_mode(mode));
    writable(0o555).unwrap();
    // as root the folder stays writable: nothing to check then
    if fs::write(dir.join("probe"), b"").is_ok() {
        writable(0o755).unwrap();
        eprintln!("Ordner bleibt beschreibbar (root?): übersprungen");
        return;
    }
    let started = std::time::Instant::now();
    let wait = Duration::from_millis(200);
    let result = FileLock::acquire_within(&file, wait, LOCK_STALE);
    let took = started.elapsed();
    writable(0o755).unwrap();
    let e = result.unwrap_err();
    assert!(took >= wait, "{took:?}");
    assert!(took < Duration::from_secs(10), "{took:?}");
    let message = e.to_string();
    assert!(
        message.starts_with(
            "project.toml nicht sperrbar: die liegen gebliebene Sperrdatei project.toml.lock \
             (Prozess 1 · Benutzer anna) lässt sich nicht entfernen ("
        ),
        "{message}"
    );
    assert!(
        message.contains("hat niemand sonst das Projekt offen, sie von Hand löschen"),
        "{message}"
    );
    assert!(
        message.ends_with(&format!("\n{}", lock_file.display())),
        "{message}"
    );
    assert!(e.stuck.is_some());
    assert!(lock_file.is_file(), "nothing else removed it");
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
