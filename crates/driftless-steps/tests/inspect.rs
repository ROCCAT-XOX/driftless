//! `inspect` through the library: report, progress and cancelling.

use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testscan::{room_bag, tmp};
use driftless_steps::inspect::{inspect, render_text};
use driftless_steps::StepError;
use std::sync::{Arc, Mutex};

#[test]
fn inspect_counts_messages_and_cancels() {
    let dir = tmp("steps-inspect");
    room_bag(&dir, true);
    let last: Arc<Mutex<Option<ProgressSnapshot>>> = Arc::default();
    let seen = last.clone();
    let p = Progress::new().with_observer(move |s| *seen.lock().unwrap() = Some(s.clone()));
    let report = inspect(&dir, &p).unwrap();
    let counts: Vec<(&str, u64)> = report
        .topics
        .iter()
        .map(|t| (t.topic.as_str(), t.count))
        .collect();
    assert_eq!(counts, [("/imu", 301), ("/lidar", 15)]);
    let last = last.lock().unwrap().clone().unwrap();
    assert_eq!(last.phase, "Bags prüfen");
    assert_eq!((last.done, last.total), (316, Some(316)));
    assert!(render_text(&report).contains("  /lidar  [sensor_msgs/PointCloud2]\n"));

    let p = Progress::new();
    let handle = p.clone();
    let p = p.with_observer(move |s: &ProgressSnapshot| {
        if s.done >= 10 {
            handle.cancel();
        }
    });
    assert_eq!(inspect(&dir, &p).unwrap_err(), StepError::Cancelled);
}
