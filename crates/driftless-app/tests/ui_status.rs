//! The status bar at the bottom, without a window: progress, runtime, „Abbrechen“, log.

use driftless_app::jobs::JobStatus;
use driftless_app::ui::{status, text, Action, AppState, UiMessage};
use driftless_io::progress::{LogLine, ProgressSnapshot};
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
fn log_lists_lines_with_warnings() {
    let state = AppState {
        log: vec![
            LogLine::Info("Lese Bags".into()),
            LogLine::Warning("IMU-Lücke bei 12 s".into()),
        ],
        ..AppState::default()
    };
    let mut h = harness(state);
    h.run();
    let header = format!("{} (2)", text::LOG);
    h.get_by_label(&header).click();
    h.run();
    h.get_by_label("Lese Bags");
    h.get_by_label("IMU-Lücke bei 12 s");
}
