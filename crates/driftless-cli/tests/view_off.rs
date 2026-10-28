//! `driftless view` in a build without the debug viewer (the default, the packages): a short
//! German message instead, whatever the arguments.
#![cfg(not(feature = "debug-viewer"))]

use assert_cmd::Command;
use predicates::prelude::*;

#[test]
fn view_says_where_the_clouds_are_shown() {
    for args in [
        &["view"][..],
        &["view", "ordner", "--slam", "lauf", "--cloud", "color.las"],
    ] {
        Command::cargo_bin("driftless")
            .unwrap()
            .args(args)
            .assert()
            .failure()
            .stderr(predicate::str::starts_with(
                "Fehler: Der Fehlersuche-Viewer ist in diesem Programm nicht enthalten",
            ))
            .stderr(predicate::str::contains("Desktop-App"))
            .stderr(predicate::str::contains("--features debug-viewer"));
    }
}
