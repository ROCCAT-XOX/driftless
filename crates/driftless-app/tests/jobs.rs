use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::channel;
use std::sync::Arc;
use std::time::{Duration, Instant};

use driftless_app::jobs::{JobOutput, JobRunner, StartError};
use driftless_io::progress::LogLine;
use driftless_steps::StepError;

fn runner() -> JobRunner {
    JobRunner::new(Arc::new(|| {}))
}

/// Polls until the job has ended (generous deadline, only for a hung test).
fn wait_finished(r: &mut JobRunner) -> driftless_app::jobs::JobFinished {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(f) = r.poll() {
            return f;
        }
        assert!(Instant::now() < deadline, "job did not finish");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn wait_until(mut cond: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !cond() {
        assert!(Instant::now() < deadline, "condition not reached");
        std::thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn start_returns_immediately() {
    let mut r = runner();
    let (tx, rx) = channel::<()>();
    let t = Instant::now();
    r.start("warten", move |_| {
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    assert!(t.elapsed() < Duration::from_millis(500));
    assert!(r.is_busy());
    tx.send(()).unwrap();
    assert!(matches!(wait_finished(&mut r).result, Ok(JobOutput::Done)));
    assert!(!r.is_busy());
}

#[test]
fn status_shows_phase_and_counts() {
    let mut r = runner();
    let (tx, rx) = channel::<()>();
    let (reached_tx, reached) = channel::<()>();
    r.start("zählen", move |p| {
        p.phase("Zählen", Some(10));
        p.advance(4);
        reached_tx.send(()).unwrap();
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    reached.recv().unwrap();
    let s = r.status().unwrap();
    assert_eq!(s.label, "zählen");
    assert_eq!(s.progress.phase, "Zählen");
    assert_eq!((s.progress.done, s.progress.total), (4, Some(10)));
    assert!(!s.cancelling);
    tx.send(()).unwrap();
    wait_finished(&mut r);
    assert!(r.status().is_none());
}

#[test]
fn second_job_is_refused_while_busy() {
    let mut r = runner();
    let (tx, rx) = channel::<()>();
    r.start("erster", move |_| {
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    let busy = r.start("zweiter", |_| Ok(JobOutput::Done)).unwrap_err();
    assert_eq!(busy, StartError::Busy("erster".into()));
    assert_eq!(busy.to_string(), "Es läuft schon: erster");
    tx.send(()).unwrap();
    wait_finished(&mut r);
    // free again once the result was taken
    r.start("dritter", |_| Ok(JobOutput::Done)).unwrap();
    wait_finished(&mut r);
}

#[test]
fn cancel_reaches_job() {
    let mut r = runner();
    r.start("abbrechen", |p| {
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(1));
        }
        Err(StepError::Cancelled)
    })
    .unwrap();
    assert!(!r.status().unwrap().cancelling);
    r.cancel();
    assert!(r.status().unwrap().cancelling);
    let f = wait_finished(&mut r);
    assert_eq!(f.label, "abbrechen");
    assert!(matches!(f.result, Err(StepError::Cancelled)));
}

#[test]
fn panicking_job_becomes_error() {
    let mut r = runner();
    r.start("panik", |_| panic!("kaputt")).unwrap();
    match wait_finished(&mut r).result {
        Err(StepError::Failed(m)) => assert_eq!(m, "interner Fehler: kaputt"),
        other => panic!("unexpected: {other:?}"),
    }
    // the runner still works
    r.start("danach", |_| Ok(JobOutput::Done)).unwrap();
    assert!(wait_finished(&mut r).result.is_ok());
}

#[test]
fn dropping_runner_cancels_job() {
    let stopped = Arc::new(AtomicBool::new(false));
    let seen = stopped.clone();
    let (started_tx, started) = channel::<()>();
    let mut r = runner();
    r.start("läuft", move |p| {
        started_tx.send(()).unwrap();
        while !p.is_cancelled() {
            std::thread::sleep(Duration::from_millis(1));
        }
        // a slow last step: drop must wait for it
        std::thread::sleep(Duration::from_millis(50));
        seen.store(true, Ordering::SeqCst);
        Err(StepError::Cancelled)
    })
    .unwrap();
    started.recv().unwrap();
    drop(r);
    assert!(
        stopped.load(Ordering::SeqCst),
        "drop returned before the job ended"
    );
}

#[test]
fn log_lines_arrive_in_order() {
    let mut r = runner();
    r.start("meldungen", |p| {
        p.info("eins");
        p.warn("zwei");
        p.info("drei");
        Ok(JobOutput::Done)
    })
    .unwrap();
    wait_finished(&mut r);
    assert_eq!(
        r.take_log(),
        vec![
            LogLine::Info("eins".into()),
            LogLine::Warning("zwei".into()),
            LogLine::Info("drei".into())
        ]
    );
    assert!(r.take_log().is_empty());
}

#[test]
fn repaint_is_requested_on_finish() {
    let count = Arc::new(AtomicUsize::new(0));
    let c = count.clone();
    let mut r = JobRunner::new(Arc::new(move || {
        c.fetch_add(1, Ordering::SeqCst);
    }));
    r.start("fertig", |_| Ok(JobOutput::Done)).unwrap();
    wait_until(|| count.load(Ordering::SeqCst) >= 1);
    // the repaint comes after the result was sent: the woken UI finds it at once
    assert!(r.poll().is_some());
}

#[test]
fn dropping_runner_with_unpolled_result() {
    let mut r = runner();
    let (done_tx, done) = channel::<()>();
    r.start("fertig, nicht abgeholt", move |p| {
        p.info("ende");
        done_tx.send(()).unwrap();
        Ok(JobOutput::Done)
    })
    .unwrap();
    done.recv().unwrap();
    // the thread may still be between send and return; drop joins it without hanging
    let t = Instant::now();
    drop(r);
    assert!(t.elapsed() < Duration::from_secs(5));
}

#[test]
fn separate_runners_work_side_by_side() {
    let mut a = runner();
    let mut b = runner();
    let (tx, rx) = channel::<()>();
    a.start("a", move |_| {
        let _ = rx.recv();
        Ok(JobOutput::Done)
    })
    .unwrap();
    b.start("b", |_| Ok(JobOutput::Done)).unwrap();
    wait_finished(&mut b);
    assert!(a.is_busy());
    tx.send(()).unwrap();
    wait_finished(&mut a);
}
