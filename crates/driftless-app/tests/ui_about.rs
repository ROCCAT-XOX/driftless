//! The window „Über Driftless“: version, commit, licence, repository and where the command
//! line is; ☰ opens it.

use driftless_app::ui::{about, rail, text, AppState};
use eframe::egui::accesskit::Role;
use egui_kittest::kittest::{NodeT, Queryable};
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

#[test]
fn notices_are_looked_for_next_to_the_program_in_the_bundle_and_in_target_package() {
    let root = driftless_io::testscan::tmp("about-lizenzen");
    // the program in a macOS bundle: next to it, then Resources, then a `package` beside it
    let macos = root.join("Driftless.app").join("Contents").join("MacOS");
    assert_eq!(
        about::notices_candidates(&macos),
        [
            macos.join(about::NOTICES_NAME),
            root.join("Driftless.app/Contents/Resources")
                .join(about::NOTICES_NAME),
            root.join("Driftless.app/Contents/package")
                .join(about::NOTICES_NAME),
        ]
    );
    assert_eq!(about::notices_path_in(&macos), None);
    let resources = root.join("Driftless.app/Contents/Resources");
    std::fs::create_dir_all(&macos).unwrap();
    std::fs::create_dir_all(&resources).unwrap();
    std::fs::write(resources.join(about::NOTICES_NAME), "<html>").unwrap();
    assert_eq!(
        about::notices_path_in(&macos),
        Some(resources.join(about::NOTICES_NAME))
    );
    // next to the program comes first (the Windows install folder)
    std::fs::write(macos.join(about::NOTICES_NAME), "<html>").unwrap();
    assert_eq!(
        about::notices_path_in(&macos),
        Some(macos.join(about::NOTICES_NAME))
    );
    // a build in target/release finds what the package scripts wrote to target/package
    let release = root.join("target").join("release");
    let package = root.join("target").join("package");
    std::fs::create_dir_all(&release).unwrap();
    std::fs::create_dir_all(&package).unwrap();
    assert_eq!(about::notices_path_in(&release), None);
    std::fs::write(package.join(about::NOTICES_NAME), "<html>").unwrap();
    assert_eq!(
        about::notices_path_in(&release),
        Some(package.join(about::NOTICES_NAME))
    );
    // a folder of that name is no file
    let dir_only = root.join("leer").join("bin");
    std::fs::create_dir_all(dir_only.join(about::NOTICES_NAME)).unwrap();
    assert_eq!(about::notices_path_in(&dir_only), None);
    let _ = std::fs::remove_dir_all(&root);
}

/// The window with the notices and the command line as given.
fn window_with(notices: Option<&'static Path>) -> Harness<'static, bool> {
    let mut h = Harness::builder().with_size([800.0, 600.0]).build_ui_state(
        move |ui, open: &mut bool| about::show_with(ui.ctx(), open, notices, None),
        true,
    );
    h.run();
    h
}

#[test]
fn about_offers_the_notices_or_says_they_are_missing() {
    let h = window_with(Some(Path::new(
        "/Programme/Driftless/THIRD-PARTY-NOTICES.html",
    )));
    h.get_by_role_and_label(Role::Button, text::ABOUT_NOTICES);
    assert!(h.query_by_label(text::ABOUT_NOTICES_MISSING).is_none());
    h.get_by_label(text::ABOUT_CLI_MISSING);

    let h = window_with(None);
    h.get_by_label(text::ABOUT_NOTICES_MISSING);
    assert!(h.query_by_label(text::ABOUT_NOTICES).is_none());
    assert!(text::ABOUT_NOTICES_MISSING.starts_with("Lizenzen Dritter: "));
}

#[test]
fn menu_has_the_log_folder() {
    let mut h = Harness::builder().with_size([320.0, 700.0]).build_ui_state(
        |ui, state: &mut AppState| {
            rail::show(ui, state);
        },
        AppState::default(),
    );
    h.run();
    assert!(h.query_by_label(text::OPEN_LOG_FOLDER).is_none());
    h.get_by_role_and_label(Role::Button, text::MENU_PROJECT)
        .click();
    h.run();
    // not clicked: it would open the file manager
    let entry = h.get_by_role_and_label(Role::Button, text::OPEN_LOG_FOLDER);
    assert!(!entry.accesskit_node().is_disabled());
}

#[test]
fn the_system_opens_with_its_own_program() {
    let target = std::ffi::OsStr::new("/Programme/Driftless/THIRD-PARTY-NOTICES.html");
    let command = about::open_command(target);
    let program = if cfg!(target_os = "macos") {
        "open"
    } else if cfg!(windows) {
        "explorer"
    } else {
        "xdg-open"
    };
    assert_eq!(command.get_program(), program);
    assert_eq!(command.get_args().collect::<Vec<_>>(), [target]);
    // an address goes the same way (the repository link of the window)
    let url = std::ffi::OsStr::new(env!("CARGO_PKG_REPOSITORY"));
    assert_eq!(
        about::open_command(url).get_args().collect::<Vec<_>>(),
        [url]
    );
}
