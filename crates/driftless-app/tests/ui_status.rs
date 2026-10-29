//! The status bar at the bottom, without a window: progress, runtime, „Abbrechen“, log.

use driftless_app::jobs::JobStatus;
use driftless_app::ui::{status, text, Action, AppState, LogEntry, LogKind, UiMessage};
use driftless_io::progress::ProgressSnapshot;
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::{NodeT, Queryable};
use egui_kittest::Harness;

fn job(done: u64, total: Option<u64>, cancelling: bool) -> JobStatus {
    JobStatus {
        label: "SLAM rechnen".into(),
        progress: ProgressSnapshot {
            phase: "Odometrie".into(),
            done,
            total,
        },
        seconds: 12.4,
        cancelling,
    }
}

fn harness(state: AppState) -> Harness<'static, (AppState, Option<Action>)> {
    Harness::builder().with_size([900.0, 300.0]).build_ui_state(
        |ui, (state, action): &mut (AppState, Option<Action>)| {
            if let Some(a) = status::show(ui, state) {
                *action = Some(a);
            }
        },
        (state, None),
    )
}

fn with_job(job: JobStatus) -> AppState {
    AppState {
        job: Some(job),
        ..AppState::default()
    }
}

#[test]
fn status_shows_progress_and_cancel() {
    let mut h = harness(with_job(job(40, Some(100), false)));
    // the progress bar animates: step instead of running until idle
    h.run_steps(3);
    h.get_by_label("40 / 100");
    h.get_by_label_contains("Odometrie");
    h.get_by_label("12 s");
    let bar = h.get_by_role(Role::ProgressIndicator);
    assert_eq!(bar.accesskit_node().numeric_value(), Some(40.0));
    h.get_by_label(text::CANCEL).click();
    h.run_steps(2);
    assert_eq!(h.state().1, Some(Action::Cancel));
}

#[test]
fn a_phase_that_names_the_step_stands_alone() {
    // „Nacharbeit: Schritt 1 von 2“, not „Nacharbeit: Nacharbeit: Schritt 1 von 2“
    let mut job = job(0, None, false);
    job.label = text::STEP_TOUCHUP.into();
    job.progress.phase = "Nacharbeit: Schritt 1 von 2".into();
    let mut h = harness(with_job(job));
    h.run_steps(3);
    h.get_by_label("Nacharbeit: Schritt 1 von 2");
    assert!(h
        .query_by_label_contains("Nacharbeit: Nacharbeit")
        .is_none());
}

#[test]
fn status_without_total_shows_spinner() {
    let mut h = harness(with_job(job(7, None, false)));
    h.run_steps(3);
    let spinner = h.get_by_role(Role::ProgressIndicator);
    assert_eq!(spinner.accesskit_node().numeric_value(), None);
    assert!(h.query_by_label_contains(" / ").is_none());
    h.get_by_label(text::CANCEL);
}

#[test]
fn cancelling_disables_button() {
    let mut h = harness(with_job(job(40, Some(100), true)));
    h.run_steps(3);
    let button = h.get_by_label(text::CANCELLING);
    assert!(button.accesskit_node().is_disabled());
    assert!(h.query_by_label(text::CANCEL).is_none());
    h.get_by_label(text::CANCELLING).click();
    h.run_steps(2);
    assert_eq!(h.state().1, None);
}

#[test]
fn idle_shows_ready_and_last_message() {
    let mut h = harness(AppState::default());
    h.run();
    h.get_by_label(text::READY);
    assert!(h.query_by_label(text::CANCEL).is_none());

    let state = AppState {
        messages: vec![
            UiMessage::Info("erste".into()),
            UiMessage::Error("Kein Projekt: project.toml fehlt".into()),
        ],
        ..AppState::default()
    };
    let mut h = harness(state);
    h.run();
    h.get_by_label("Kein Projekt: project.toml fehlt");
    assert!(h.query_by_label("erste").is_none());
    h.get_by_label(text::DISMISS).click();
    h.run();
    assert_eq!(h.state().1, Some(Action::DismissMessages));
}

#[test]
fn log_lists_lines_with_warnings_and_errors() {
    let entry = |kind, text: &str| LogEntry {
        time: "14:03:12".into(),
        kind,
        text: text.into(),
    };
    let state = AppState {
        log: vec![
            entry(LogKind::Info, "Lese Bags"),
            entry(LogKind::Warning, "IMU-Lücke bei 12 s"),
            entry(
                LogKind::Error,
                "Karte rechnen: keine Berechtigung: /Users/x/Halle 2026/scans/a/karte-001/color.las",
            ),
        ],
        ..AppState::default()
    };
    let mut h = harness(state);
    h.run();
    let header = format!("{} (3)", text::LOG);
    h.get_by_label(&header).click();
    h.run();
    h.get_by_label("Lese Bags");
    h.get_by_label("IMU-Lücke bei 12 s");
    // the reason stays in view: the path is cut to its file name (all of it on hover)
    h.get_by_label("Karte rechnen: keine Berechtigung: color.las");
    // each line with its time
    assert_eq!(h.query_all_by_label("14:03:12").count(), 3);
}

/// A message whose path is long: the line shows the reason and the file name, a warning in
/// its own colour (not checked here: only that it is shown).
#[test]
fn messages_show_the_reason_with_the_file_name() {
    let state = AppState {
        messages: vec![UiMessage::Error(
            "Ordner ist nicht leer – ein neues Projekt braucht einen leeren Ordner: Projekte\n\
             /Users/x/Documents/Firma/Kunden/Projekte"
                .into(),
        )],
        ..AppState::default()
    };
    let mut h = harness(state);
    h.run();
    h.get_by_label(
        "Ordner ist nicht leer – ein neues Projekt braucht einen leeren Ordner: Projekte",
    );
    let state = AppState {
        messages: vec![UiMessage::Warning(
            "Projekt liegt in iCloud Drive: /Users/x/Documents/Halle".into(),
        )],
        ..AppState::default()
    };
    let mut h = harness(state);
    h.run();
    h.get_by_label("Projekt liegt in iCloud Drive: Halle");
}
