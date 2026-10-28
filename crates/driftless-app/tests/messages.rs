//! What the window says: system errors in German with the reason first, the command-line
//! hints of the steps as actions in the window, symbols the fonts lack drawn by the icon font,
//! paths cut to their names, and every message and Protokoll line in the log file as it comes
//! (also one added and cleared within a frame).

use driftless_app::logfile::LogFile;
use driftless_app::ui::{text, theme, Action, DriftlessApp, LogKind, UiMessage};
use driftless_io::testscan::tmp;
use eframe::egui::{self, FontFamily, FontId};
use std::io::{self, ErrorKind};
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// A system error code of this system with the kind `kind` (13 on Unix, 5 on Windows for a
/// missing permission, …).
fn code_of(kind: ErrorKind) -> i32 {
    (1..20_000)
        .find(|&c| io::Error::from_raw_os_error(c).kind() == kind)
        .expect("a code of this kind")
}

#[test]
fn system_errors_are_german() {
    for (kind, reason) in [
        (ErrorKind::PermissionDenied, "keine Berechtigung"),
        (ErrorKind::NotFound, "nicht gefunden"),
        (ErrorKind::StorageFull, "Datenträger voll"),
    ] {
        let e = io::Error::from_raw_os_error(code_of(kind));
        let message = format!("/Projekte/Halle/karte.laz: {e}");
        assert_eq!(
            text::german(&message),
            format!("/Projekte/Halle/karte.laz: {reason}"),
            "{message}"
        );
        // the reason with the file it was about, the whole path below
        let path = Path::new("/Projekte/Halle/karte.laz");
        assert_eq!(
            text::io_error("Export nicht schreibbar", path, &e),
            format!("Export nicht schreibbar: {reason}: karte.laz\n/Projekte/Halle/karte.laz")
        );
    }
    // a code that is no system error of this text stays as it is
    assert_eq!(
        text::german("Fehler (os error 13x)"),
        "Fehler (os error 13x)"
    );
    assert_eq!(text::german("ohne Fehler"), "ohne Fehler");
}

#[test]
fn command_line_hints_become_window_actions() {
    let said = "Laufbahn /p/traj.tum nicht lesbar: kaputt; zuerst `driftless slam <scan> -o \
                <ziel>` laufen lassen";
    let german = text::german(said);
    assert!(!german.contains("driftless slam"), "{german}");
    assert!(german.contains("zuerst den Scan rechnen"), "{german}");
    let said = "/p/slam.json ist kein gültiger SLAM-Bericht: kaputt; mit `driftless slam` neu \
                erzeugen";
    let german = text::german(said);
    assert!(!german.contains('`'), "{german}");
    assert!(german.contains("den Scan neu rechnen"), "{german}");
    // one that names its own folders
    let said = "Zielordner /p/x fehlt; zuerst `driftless slam <ordner> -o /p/x` laufen lassen";
    let german = text::german(said);
    assert_eq!(
        german,
        "Zielordner /p/x fehlt; zuerst den Scan rechnen (Scan wählen, „Rechnen“)"
    );
}

#[test]
fn readers_say_where_in_german() {
    assert_eq!(
        text::german("/p/punktpaare.toml nicht lesbar: TOML parse error at line 1, column 5\n  |"),
        "/p/punktpaare.toml nicht lesbar: beschädigt (Zeile 1, Spalte 5)\n  |"
    );
    assert_eq!(
        text::german("invalid type: string \"gestern\", expected f64 at line 1 column 24"),
        "invalid type: string \"gestern\", expected f64 (Zeile 1, Spalte 24)"
    );
    // a TOML file of the project that cannot be read: name, line and column first
    let content = "format = 1\nname = \n";
    let e = toml::from_str::<toml::Table>(content).unwrap_err();
    let message = text::damaged_toml(Path::new("/p/project.toml"), content, &e);
    assert!(
        message.starts_with("project.toml ist beschädigt (Zeile 2, Spalte "),
        "{message}"
    );
    assert!(message.ends_with("\n/p/project.toml"), "{message}");
}

#[test]
fn paths_are_cut_to_their_names() {
    for (message, short) in [
        (
            "Ordner /Users/x/Projekte ist nicht leer",
            "Ordner Projekte ist nicht leer",
        ),
        (
            "/Users/x/Halle 2026/scans/a/slam-001/slam.json ist kein gültiger Bericht",
            "slam.json ist kein gültiger Bericht",
        ),
        (
            "Export: /Users/x/karte.laz: keine Berechtigung",
            "Export: karte.laz: keine Berechtigung",
        ),
        (
            r"Scan-Ordner nicht gefunden (C:\Daten\Scans\Halle Ost)",
            "Scan-Ordner nicht gefunden (Halle Ost)",
        ),
        (
            r"\\nas\daten\scan: Netzlaufwerk nicht erreichbar",
            "scan: Netzlaufwerk nicht erreichbar",
        ),
        // what only looks like a path stays
        (
            "Bilder verkleinern: 1/2, 3 m/s",
            "Bilder verkleinern: 1/2, 3 m/s",
        ),
        ("A / B", "A / B"),
    ] {
        assert_eq!(text::short_paths(message), short, "{message}");
    }
    assert_eq!(
        text::short_paths("erste /a/b.txt\nzweite /c/d.txt"),
        "erste b.txt\nzweite d.txt"
    );
}

/// The window's fonts, as the program sets them.
fn fonts() -> egui::Context {
    let ctx = egui::Context::default();
    theme::apply(&ctx);
    let mut out = ctx.run_ui(Default::default(), |_| {});
    out.textures_delta.clear();
    ctx
}

/// The characters the fonts of the window cannot draw, of `s`.
fn missing(ctx: &egui::Context, s: &str) -> String {
    let font = FontId::new(14.0, FontFamily::Proportional);
    s.chars()
        .filter(|c| !c.is_ascii())
        .filter(|&c| !ctx.fonts_mut(|f| f.has_glyph(&font, c)))
        .collect()
}

/// Every non-ASCII character in the texts of the window (the string literals of `text.rs`)
/// has a glyph, and the arrows and marks the steps write are drawn by the icon font.
#[test]
fn every_symbol_of_the_texts_has_a_glyph() {
    let ctx = fonts();
    let source = include_str!("../src/ui/text.rs");
    // the string literals only (not the comments, which hold arrows)
    let mut literals = String::new();
    for line in source.lines() {
        let code = line.split("//").next().unwrap_or_default();
        let mut inside = false;
        for c in code.chars() {
            if c == '"' {
                inside = !inside;
            } else if inside {
                literals.push(c);
            }
        }
    }
    // the symbols of `shown` are written as escapes in the source: they come from the steps
    assert_eq!(missing(&ctx, &literals), "", "texts of text.rs");
    let steps = "Wanddicke 0,3 → 0,0 mm, Start↔Ende 1,2 m, fertig ✓, gescheitert ✗, \
                 ⚠ Achtung, ▸ zu, ▾ auf, ↑ ↓ ←";
    assert!(
        !missing(&ctx, steps).is_empty(),
        "the plain fonts lack them"
    );
    assert_eq!(
        missing(&ctx, &text::shown(steps)),
        "",
        "{}",
        text::shown(steps)
    );
    // nothing changes where nothing is missing
    assert!(matches!(
        text::shown("Wanddicke 0,3 mm"),
        std::borrow::Cow::Borrowed(_)
    ));
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap()
}

/// Ticks until no job is left.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while app.wants_repaint_while_busy() {
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(2));
        app.tick();
    }
}

/// Every message goes into the log file as it comes, in German: also one that is added and
/// cleared before the next frame (the per-frame comparison of earlier missed those), and
/// errors and warnings go into the Protokoll as well.
#[test]
fn messages_and_lines_go_into_the_log_file_as_they_come() {
    let dir = tmp("meldungen-protokolldatei");
    let log = Arc::new(LogFile::start(&dir.join("logs"), "Driftless 0.1.0 (test)").unwrap());
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_log_file(log.clone());
    // an error, cleared at once by „Schließen“ in the same frame
    app.apply(Action::OpenProject(dir.join("gibt-es-nicht")));
    app.apply(Action::DismissMessages);
    assert!(app.state().messages.is_empty());
    let text = read(log.path());
    assert!(
        text.lines()
            .any(|l| l.contains(" Fehler  Kein Projekt: project.toml fehlt")),
        "{text}"
    );
    // … and it is in the Protokoll
    let entry = app.state().log.last().cloned().unwrap();
    assert_eq!(entry.kind, LogKind::Error);
    assert!(entry.text.starts_with("Kein Projekt"), "{entry:?}");
    assert_eq!(entry.time.len(), 8, "{entry:?}");

    // the lines of a step, and its end
    app.start_job("zählen", |p| {
        p.info("Lese Bags");
        p.warn("Wanddicke 0,3 → 0,0 mm");
        Ok(driftless_app::jobs::JobOutput::Done)
    })
    .unwrap();
    settle(&mut app);
    let text = read(log.path());
    let lines: Vec<&str> = text.lines().collect();
    let at = |what: &str| {
        lines
            .iter()
            .position(|l| l.contains(what))
            .unwrap_or_else(|| panic!("{what}: {text}"))
    };
    // in the file as written (the arrow stays), each once, the end of the step after them
    assert!(at(" Info    Lese Bags") < at(" Warnung Wanddicke 0,3 → 0,0 mm"));
    assert!(at(" Warnung Wanddicke") < at(" Meldung zählen fertig"));
    assert_eq!(text.matches("Lese Bags").count(), 1, "{text}");
    // in the Protokoll as shown: the arrow of the icon font
    let shown: Vec<&str> = app.state().log.iter().map(|e| e.text.as_str()).collect();
    assert!(
        shown.contains(&text::shown("Wanddicke 0,3 → 0,0 mm").as_ref()),
        "{shown:?}"
    );
    // infos stay on the status bar only
    assert!(
        !shown.iter().any(|t| t.contains("zählen fertig")),
        "{shown:?}"
    );
    assert!(matches!(
        app.state().messages.last(),
        Some(UiMessage::Info(m)) if m.starts_with("zählen fertig")
    ));

    // the same error twice: once on the status bar and in the Protokoll
    let before = app.state().log.len();
    app.apply(Action::OpenProject(dir.join("gibt-es-nicht")));
    app.apply(Action::OpenProject(dir.join("gibt-es-nicht")));
    let errors = app
        .state()
        .messages
        .iter()
        .filter(|m| matches!(m, UiMessage::Error(_)))
        .count();
    assert_eq!(errors, 1);
    assert_eq!(app.state().log.len(), before + 1);
    let _ = std::fs::remove_dir_all(&dir);
}
