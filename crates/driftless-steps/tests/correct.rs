//! Manual corrections through the library, in the file format of `driftless correct`.

use driftless_io::progress::Progress;
use driftless_io::testscan::{room_bag, tmp};
use driftless_slam::Corrections;
use driftless_steps::correct;
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings, CORRECTIONS_FILE};
use driftless_steps::StepError;
use std::fs;

#[test]
fn add_loop_round_trip_with_cli_format() {
    let run = tmp("steps-correct-format");
    let added = "2026-10-21T12:34:56Z";
    assert_eq!(
        correct::add_loop_at(&run, 1.5, 20.25, "Treppe", added).unwrap(),
        1
    );
    assert_eq!(correct::add_loop_at(&run, 3.0, 40.0, "", added).unwrap(), 2);

    let reference = run.join("reference.toml");
    let mut c = Corrections::default();
    c.add_loop(1.5, 20.25, "Treppe", added);
    c.add_loop(3.0, 40.0, "", added);
    c.save(&reference).unwrap();

    let written = run.join(CORRECTIONS_FILE);
    assert_eq!(fs::read(&written).unwrap(), fs::read(&reference).unwrap());
    assert_eq!(
        correct::load(&run).unwrap(),
        Corrections::load(&reference).unwrap()
    );

    // `add_loop` stamps the current time
    correct::add_loop(&run, 5.0, 50.0, "").unwrap();
    let now = &correct::load(&run).unwrap().loops[2].added;
    assert!(now.len() == 20 && now.ends_with('Z') && now.as_bytes()[10] == b'T');
}

#[test]
fn add_loop_rejects_same_submap() {
    // 1.5 s: a single submap, so both times fall into it
    let dir = tmp("steps-correct-same");
    room_bag(&dir, true);
    let out = dir.join("out");
    let err = correct::load(&out).unwrap_err();
    assert!(err.to_string().starts_with("Zielordner"), "{err}");

    let req = SlamRequest {
        scan: dir.clone(),
        out: out.clone(),
        settings: SlamSettings::default(),
        reuse: false,
        history: false,
    };
    run_slam(&req, &Progress::new()).unwrap();
    let err = correct::add_loop(&out, 0.2, 1.0, "").unwrap_err();
    assert!(
        matches!(&err, StepError::Failed(m) if m.contains("derselben Teilkarte")),
        "{err}"
    );
    assert!(!out.join(CORRECTIONS_FILE).exists(), "nothing written");

    let err = correct::add_loop(&out, -1.0, 1.0, "").unwrap_err();
    assert!(err.to_string().contains("außerhalb des Scans"), "{err}");
    let err = correct::set_enabled(&out, 1, true).unwrap_err();
    assert!(err.to_string().contains("kein Eintrag 1"), "{err}");
}
