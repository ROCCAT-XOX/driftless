use driftless_io::progress::{LogLine, Progress, ProgressSnapshot};
use std::sync::{Arc, Mutex};

#[test]
fn progress_phase_resets_counter() {
    let p = Progress::new();
    assert_eq!(p.snapshot(), ProgressSnapshot::default());
    p.phase("LiDAR lesen", Some(10));
    p.advance(3);
    p.advance(4);
    assert_eq!(
        p.snapshot(),
        ProgressSnapshot {
            phase: "LiDAR lesen".into(),
            done: 7,
            total: Some(10),
        }
    );
    p.phase("Schleifen", None);
    assert_eq!(
        p.snapshot(),
        ProgressSnapshot {
            phase: "Schleifen".into(),
            done: 0,
            total: None,
        }
    );
}

#[test]
fn progress_is_shared_between_clones() {
    let p = Progress::new();
    let worker = p.clone();
    let handle = std::thread::spawn(move || {
        worker.phase("Arbeit", Some(1_000_000));
        let mut n = 0u64;
        while !worker.is_cancelled() {
            worker.advance(1);
            n += 1;
            std::thread::yield_now();
        }
        n
    });
    while p.snapshot().done < 10 {
        std::thread::yield_now();
    }
    p.cancel();
    let n = handle.join().unwrap();
    assert!(p.is_cancelled());
    assert_eq!(p.snapshot().done, n);
    assert_eq!(p.snapshot().phase, "Arbeit");
}

#[test]
fn log_keeps_order_and_reaches_sink() {
    let p = Progress::new();
    p.info("eins");
    p.warn("zwei");
    p.info(String::from("drei"));
    assert_eq!(
        p.take_log(),
        vec![
            LogLine::Info("eins".into()),
            LogLine::Warning("zwei".into()),
            LogLine::Info("drei".into()),
        ]
    );
    assert!(p.take_log().is_empty());
    p.warn("vier");
    assert_eq!(p.take_log(), vec![LogLine::Warning("vier".into())]);

    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = seen.clone();
    let p = Progress::with_log(move |l| sink.lock().unwrap().push(l.clone()));
    let worker = p.clone();
    std::thread::spawn(move || {
        worker.info("a");
        worker.warn("b");
    })
    .join()
    .unwrap();
    assert_eq!(
        *seen.lock().unwrap(),
        vec![LogLine::Info("a".into()), LogLine::Warning("b".into())]
    );
    // A sink takes the messages; nothing is kept.
    assert!(p.take_log().is_empty());
}

#[test]
fn observer_sees_every_update_and_can_cancel() {
    let p = Progress::new();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (log, q) = (seen.clone(), p.clone());
    let p = p.with_observer(move |s| {
        log.lock().unwrap().push((s.phase.clone(), s.done));
        if s.done >= 2 {
            q.cancel();
        }
    });
    p.phase("A", Some(3));
    p.advance(1);
    assert!(!p.is_cancelled());
    p.advance(1);
    assert!(p.is_cancelled());
    assert_eq!(
        *seen.lock().unwrap(),
        vec![
            ("A".to_string(), 0),
            ("A".to_string(), 1),
            ("A".to_string(), 2)
        ]
    );
}
