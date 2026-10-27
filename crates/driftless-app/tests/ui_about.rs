//! The window „Über Driftless“: version, commit, licence, repository and where the command
//! line is; ☰ opens it.

use driftless_app::ui::{about, rail, text, AppState};
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::Queryable;
use egui_kittest::Harness;
use std::path::Path;

fn window(open: bool) -> Harness<'static, bool> {
    let mut h = Harness::builder()
        .with_size([800.0, 600.0])
        .build_ui_state(|ui, open: &mut bool| about::show(ui.ctx(), open), open);
    h.run();
    h
}

#[test]
fn about_shows_version_commit_license() {
    let h = window(true);
    h.get_by_label(text::ABOUT_TITLE);
    h.get_by_label(&text::about_version(env!("CARGO_PKG_VERSION")));
    h.get_by_label(&text::about_commit(about::COMMIT));
    h.get_by_label(text::ABOUT_LICENSE);
    h.get_by_label(env!("CARGO_PKG_REPOSITORY"));
    h.get_by_label(&about::cli_line(about::cli_path().as_deref()));
    assert!(!about::COMMIT.is_empty());
    assert_eq!(about::VERSION, env!("CARGO_PKG_VERSION"));
}

#[test]
fn about_closed_shows_nothing() {
    let h = window(false);
    assert!(h.query_by_label(text::ABOUT_TITLE).is_none());
}

#[test]
fn about_cross_closes_it() {
    let mut h = window(true);
    h.get_by_role_and_label(Role::Button, "Close window")
        .click();
    h.run();
    assert!(!*h.state());
    assert!(h.query_by_label(text::ABOUT_TITLE).is_none());
}

#[test]
fn cli_line_names_the_program_next_to_the_app_or_says_it_is_missing() {
    let dir = driftless_io::testscan::tmp("about-cli");
    assert_eq!(about::cli_path_in(&dir), None);
    assert_eq!(about::cli_line(None), text::ABOUT_CLI_MISSING);
    let cli = dir.join(about::CLI_NAME);
    std::fs::write(&cli, b"").unwrap();
    assert_eq!(about::cli_path_in(&dir), Some(cli.clone()));
    assert_eq!(
        about::cli_line(Some(&cli)),
        text::about_cli(&cli.display().to_string())
    );
    assert!(about::cli_line(Some(Path::new("/x/driftless"))).starts_with("Kommandozeile: "));
    assert!(text::ABOUT_CLI_MISSING.starts_with("Kommandozeile: "));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn menu_opens_about() {
    let mut h = Harness::builder().with_size([320.0, 700.0]).build_ui_state(
        |ui, state: &mut AppState| {
            rail::show(ui, state);
        },
        AppState::default(),
    );
    h.run();
    assert!(!h.state().about_open);
    h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
        .click();
    h.run();
    h.get_by_label(text::ABOUT).click();
    h.run();
    assert!(h.state().about_open);
}
