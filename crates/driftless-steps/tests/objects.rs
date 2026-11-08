//! `objekte.toml`: the objects set by hand in a map or model and the measurements kept there,
//! through the library.

use driftless_io::lock::FileLock;
use driftless_io::testscan::tmp;
use driftless_steps::objects::{
    ConnectionKind, ObjectInput, ObjectKind, Objects, ObjectsError, Undone, OBJECTS_FILE,
};
use driftless_steps::StepError;
use std::collections::BTreeMap;
use std::fs;

const P: [f64; 3] = [1.0, 2.0, 0.5];
const Q: [f64; 3] = [3.0, 2.0, 0.5];

/// An object of `kind` at `points`, without name, note or anything else.
fn input(kind: ObjectKind, points: &[[f64; 3]]) -> ObjectInput {
    ObjectInput {
        kind,
        points: points.to_vec(),
        name: String::new(),
        note: String::new(),
        connection: None,
        diameter_mm: None,
        properties: BTreeMap::new(),
    }
}

fn socket(x: f64) -> ObjectInput {
    input(ObjectKind::Socket, &[[x, 1.0, 0.3]])
}

fn connection(kind: ConnectionKind) -> ObjectInput {
    ObjectInput {
        connection: Some(kind),
        ..input(ObjectKind::Connection, &[P])
    }
}

fn pipe(points: &[[f64; 3]], diameter_mm: Option<f32>) -> ObjectInput {
    ObjectInput {
        diameter_mm,
        ..input(ObjectKind::Pipe, points)
    }
}

fn measure(x: f64) -> ObjectInput {
    input(ObjectKind::Measure, &[[x, 0.0, 1.0], [x, 2.0, 1.0]])
}

/// One object of every kind: a named socket with free properties and characters that need care
/// in a file, far from the origin as points are absolute.
fn one_of_each() -> Objects {
    let mut objects = Objects::default();
    let properties = BTreeMap::from([
        ("hersteller".to_string(), "Gira".to_string()),
        ("stromkreis".to_string(), "F3 \"Küche\"".to_string()),
    ]);
    objects
        .add(ObjectInput {
            name: "ä\nb;😀".into(),
            note: "hinter dem \\ Schrank\r\n# keine Zeile".into(),
            properties,
            ..input(ObjectKind::Socket, &[[1_234.567_890_123, -0.1, 1e-7]])
        })
        .unwrap();
    objects.add(connection(ConnectionKind::Drain)).unwrap();
    objects
        .add(input(ObjectKind::Cable, &[P, Q, [3.0, 4.0, 0.5]]))
        .unwrap();
    objects.add(pipe(&[P, Q], Some(12.5))).unwrap();
    objects.add(measure(5.0)).unwrap();
    objects
}

fn numbers(objects: &Objects) -> Vec<u32> {
    objects.all().iter().map(|o| o.nr).collect()
}

#[test]
fn roundtrip_keeps_everything() {
    let dir = tmp("steps-objects-roundtrip");
    let objects = one_of_each();
    assert_eq!(numbers(&objects), [1, 2, 3, 4, 5]);
    objects.save(&dir).unwrap();
    let back = Objects::load(&dir).unwrap();
    assert_eq!(back, objects);
    assert_eq!(back.get(1).unwrap().name, "ä\nb;😀");
    assert_eq!(
        back.get(1).unwrap().points,
        [[1_234.567_890_123, -0.1, 1e-7]]
    );
    assert_eq!(back.get(4).unwrap().diameter_mm, Some(12.5));
    assert_eq!(back.get(2).unwrap().connection, Some(ConnectionKind::Drain));

    let path = dir.join(OBJECTS_FILE);
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# Driftless-Objekte"), "{text}");
    for line in [
        "[[objekt]]",
        "art = \"steckdose\"",
        "art = \"anschluss\"",
        "art = \"kabel\"",
        "art = \"rohr\"",
        "art = \"messung\"",
        "anschluss = \"abwasser\"",
        "durchmesser_mm = 12.5",
        "quelle = \"hand\"",
        "[objekt.eigenschaften]",
        "hersteller = \"Gira\"",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
    assert!(!text.contains("geloescht"), "{text}");

    // a line added by hand to the second object survives loading and saving
    assert_eq!(text.matches("\nnr = 2\n").count(), 1, "{text}");
    fs::write(
        &path,
        text.replace("\nnr = 2\n", "\nnr = 2\nfarbe = \"rot\"\n"),
    )
    .unwrap();
    let mut noted = Objects::load(&dir).unwrap();
    let colour = |o: &Objects| {
        let value = o.get(2).unwrap().unknown.get("farbe");
        value.and_then(|v| v.as_str()).map(str::to_string)
    };
    assert_eq!(colour(&noted).as_deref(), Some("rot"));
    noted.save(&dir).unwrap();
    assert_eq!(Objects::load(&dir).unwrap(), noted);
    assert!(fs::read_to_string(&path)
        .unwrap()
        .contains("farbe = \"rot\""));

    // also through a removal and its undo
    assert!(noted.remove(2));
    noted.save(&dir).unwrap();
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.contains("[[geloescht]]"), "{text}");
    assert!(text.contains("farbe = \"rot\""), "{text}");
    let mut back = Objects::load(&dir).unwrap();
    assert_eq!(back, noted);
    assert_eq!(back.undo(), Some(Undone::Removed(2)));
    assert_eq!(colour(&back).as_deref(), Some("rot"));
}

#[test]
fn connection_kinds_have_their_names_in_the_file() {
    let dir = tmp("steps-objects-connections");
    let kinds = [
        (ConnectionKind::Power, "strom"),
        (ConnectionKind::Water, "wasser"),
        (ConnectionKind::Drain, "abwasser"),
        (ConnectionKind::Heating, "heizung"),
        (ConnectionKind::Gas, "gas"),
        (ConnectionKind::Network, "netzwerk"),
        (ConnectionKind::Other, "sonstiges"),
    ];
    let mut objects = Objects::default();
    for (kind, _) in kinds {
        objects.add(connection(kind)).unwrap();
    }
    objects.save(&dir).unwrap();
    let text = fs::read_to_string(dir.join(OBJECTS_FILE)).unwrap();
    for (_, name) in kinds {
        let line = format!("anschluss = \"{name}\"");
        assert!(text.contains(&line), "{line} missing in\n{text}");
    }
    assert_eq!(Objects::load(&dir).unwrap(), objects);
}

#[test]
fn missing_file_is_empty() {
    let dir = tmp("steps-objects-missing");
    let objects = Objects::load(&dir).unwrap();
    assert_eq!(objects, Objects::default());
    assert!(objects.all().is_empty() && !objects.can_undo());
    assert_eq!(
        Objects::load(&dir.join("karte-009")).unwrap(),
        Objects::default()
    );
    assert!(!dir.join(OBJECTS_FILE).exists());
}

#[test]
fn invalid_inputs_are_refused() {
    use ObjectKind::{Cable, Measure, Socket};
    let nan = [1.0, f64::NAN, 0.5];
    let inf = [f64::INFINITY, 2.0, 0.5];
    let power = |points: &[[f64; 3]]| ObjectInput {
        points: points.to_vec(),
        ..connection(ConnectionKind::Power)
    };
    let not_a_number = "keine gültige Zahl";
    let cases = [
        // the points of each kind
        (input(Socket, &[]), "Steckdose braucht genau einen Punkt"),
        (
            input(Socket, &[P, Q]),
            "Steckdose braucht genau einen Punkt",
        ),
        (power(&[]), "Anschluss braucht genau einen Punkt"),
        (power(&[P, Q]), "Anschluss braucht genau einen Punkt"),
        (input(Cable, &[]), "Kabel braucht mindestens zwei Punkte"),
        (input(Cable, &[P]), "Kabel braucht mindestens zwei Punkte"),
        (
            pipe(&[P], Some(50.0)),
            "Rohr braucht mindestens zwei Punkte",
        ),
        (input(Measure, &[P]), "Messung braucht genau zwei Punkte"),
        (
            input(Measure, &[P, Q, P]),
            "Messung braucht genau zwei Punkte",
        ),
        // a connection needs its kind
        (input(ObjectKind::Connection, &[P]), "Sorte"),
        // a pipe needs a diameter above 0
        (pipe(&[P, Q], None), "Durchmesser"),
        (pipe(&[P, Q], Some(0.0)), "Durchmesser"),
        (pipe(&[P, Q], Some(-50.0)), "Durchmesser"),
        (pipe(&[P, Q], Some(f32::NAN)), "Durchmesser"),
        (pipe(&[P, Q], Some(f32::INFINITY)), "Durchmesser"),
        // every coordinate finite, for each kind
        (input(Socket, &[nan]), not_a_number),
        (power(&[inf]), not_a_number),
        (input(Cable, &[P, nan]), not_a_number),
        (pipe(&[inf, Q], Some(50.0)), not_a_number),
        (
            input(Measure, &[P, [1.0, 2.0, f64::NEG_INFINITY]]),
            not_a_number,
        ),
    ];
    for (bad, part) in cases {
        let mut objects = Objects::default();
        let err = objects.add(bad.clone()).unwrap_err();
        assert!(
            matches!(&err, ObjectsError::Invalid(m) if m.contains(part)),
            "{bad:?}: {err}"
        );
        assert!(err.to_string().contains(part), "{err}");
        assert_eq!(objects, Objects::default(), "{bad:?} changed something");
    }

    // what the rules allow
    let mut objects = Objects::default();
    objects.add(input(Socket, &[P])).unwrap();
    objects.add(power(&[P])).unwrap();
    objects.add(input(Cable, &[P, Q])).unwrap();
    objects.add(pipe(&[P, Q, P, Q], Some(0.5))).unwrap();
    objects.add(input(Measure, &[P, Q])).unwrap();
    assert_eq!(objects.all().len(), 5);
}

#[test]
fn what_does_not_belong_to_the_kind_is_left_out() {
    // the tool keeps its kind of connection and its diameter while another kind is set
    let mut objects = Objects::default();
    let nr = objects
        .add(ObjectInput {
            connection: Some(ConnectionKind::Gas),
            diameter_mm: Some(50.0),
            ..socket(1.0)
        })
        .unwrap();
    let socket = objects.get(nr).unwrap();
    assert_eq!((socket.connection, socket.diameter_mm), (None, None));
    let nr = objects
        .add(ObjectInput {
            diameter_mm: Some(50.0),
            ..connection(ConnectionKind::Gas)
        })
        .unwrap();
    let kept = objects.get(nr).unwrap();
    assert_eq!(kept.connection, Some(ConnectionKind::Gas));
    assert_eq!(kept.diameter_mm, None);
    let nr = objects
        .add(ObjectInput {
            connection: Some(ConnectionKind::Gas),
            ..pipe(&[P, Q], Some(50.0))
        })
        .unwrap();
    let kept = objects.get(nr).unwrap();
    assert_eq!(kept.connection, None);
    assert_eq!(kept.diameter_mm, Some(50.0));
}

#[test]
fn numbers_are_not_used_twice() {
    let dir = tmp("steps-objects-numbers");
    let mut objects = Objects::default();
    assert_eq!(objects.add(socket(1.0)).unwrap(), 1);
    // after an undo and a removal the next object counts on
    assert_eq!(objects.undo(), Some(Undone::Added(1)));
    assert_eq!(objects.add(socket(1.0)).unwrap(), 2);
    assert!(objects.remove(2));
    assert_eq!(objects.add(socket(1.0)).unwrap(), 3);

    // also after a restart, with the removed one still able to come back
    objects.save(&dir).unwrap();
    let mut objects = Objects::load(&dir).unwrap();
    assert_eq!(objects.add(socket(1.0)).unwrap(), 4);
    assert_eq!(objects.undo(), Some(Undone::Added(4)));
    assert_eq!(objects.undo(), Some(Undone::Added(3)));
    assert_eq!(objects.undo(), Some(Undone::Removed(2)));
    assert_eq!(numbers(&objects), [2]);
    objects.save(&dir).unwrap();
    let mut objects = Objects::load(&dir).unwrap();
    assert_eq!(objects.add(socket(1.0)).unwrap(), 5);

    // in a file without counters a removed number is not given again
    let path = dir.join(OBJECTS_FILE);
    assert!(objects.remove(5));
    objects.save(&dir).unwrap();
    let text = fs::read_to_string(&path).unwrap();
    let without: Vec<&str> = text
        .lines()
        .filter(|l| !l.starts_with("next_nr") && !l.starts_with("next_seq"))
        .collect();
    fs::write(&path, without.join("\n")).unwrap();
    let mut objects = Objects::load(&dir).unwrap();
    assert_eq!(objects.add(socket(1.0)).unwrap(), 6);
}

#[test]
fn undo_order_and_restart() {
    for restart in [false, true] {
        let dir = tmp(&format!("steps-objects-undo-{restart}"));
        let mut objects = Objects::default();
        let first = objects.add(socket(1.0)).unwrap();
        let second = objects.add(socket(2.0)).unwrap();
        let added = objects.clone();
        assert!(objects.rename(first, "Küche"));
        let renamed = objects.clone();
        assert!(objects.set_note(first, "neben der Tür"));
        let noted = objects.clone();
        assert_eq!(objects.get(first).unwrap().name, "Küche");
        assert_eq!(objects.get(first).unwrap().note, "neben der Tür");
        assert!(objects.remove(first));
        assert!(objects.get(first).is_none());
        assert!(objects.can_undo());

        let undo = |objects: &mut Objects| {
            if restart {
                objects.save(&dir).unwrap();
                *objects = Objects::load(&dir).unwrap();
            }
            objects.undo()
        };
        // back in its place, then the note, the name, then the additions, youngest first
        assert_eq!(undo(&mut objects), Some(Undone::Removed(first)));
        assert_eq!(objects.all(), noted.all());
        assert_eq!(undo(&mut objects), Some(Undone::Changed(first)));
        assert_eq!(objects.all(), renamed.all());
        assert_eq!(undo(&mut objects), Some(Undone::Changed(first)));
        assert_eq!(objects.all(), added.all());
        assert_eq!(undo(&mut objects), Some(Undone::Added(second)));
        assert_eq!(numbers(&objects), [first]);
        assert_eq!(undo(&mut objects), Some(Undone::Added(first)));
        assert!(objects.all().is_empty());
        assert!(!objects.can_undo());
        assert_eq!(undo(&mut objects), None);
    }
}

#[test]
fn a_change_that_changes_nothing_is_no_step() {
    let mut objects = Objects::default();
    assert!(!objects.rename(1, "x"));
    assert!(!objects.set_note(1, "x"));
    assert!(!objects.remove(1));
    assert_eq!(objects, Objects::default());

    let nr = objects
        .add(ObjectInput {
            name: "Flur".into(),
            ..socket(1.0)
        })
        .unwrap();
    let before = objects.clone();
    assert!(objects.rename(nr, "Flur"));
    assert!(objects.set_note(nr, ""));
    assert_eq!(objects, before);
    assert_eq!(objects.undo(), Some(Undone::Added(nr)));
}

#[test]
fn remove_kind_is_one_undo() {
    let dir = tmp("steps-objects-remove-kind");
    let mut objects = Objects::default();
    let a = objects.add(measure(1.0)).unwrap();
    let plug = objects.add(socket(1.0)).unwrap();
    let b = objects.add(measure(2.0)).unwrap();
    let c = objects.add(measure(3.0)).unwrap();
    let before = objects.clone();

    assert_eq!(objects.remove_kind(ObjectKind::Measure), 3);
    assert_eq!(numbers(&objects), [plug]);
    objects.save(&dir).unwrap();
    let mut objects = Objects::load(&dir).unwrap();

    // one step back brings all of them, in their places
    assert_eq!(objects.undo(), Some(Undone::Removed(a)));
    assert_eq!(objects.all(), before.all());
    assert_eq!(objects.undo(), Some(Undone::Added(c)));
    assert_eq!(objects.undo(), Some(Undone::Added(b)));
    assert_eq!(numbers(&objects), [a, plug]);

    // none of the kind: no step, nothing to bring back
    let mut only = Objects::default();
    only.add(socket(1.0)).unwrap();
    let before = only.clone();
    assert_eq!(only.remove_kind(ObjectKind::Cable), 0);
    assert_eq!(only, before);
}

#[test]
fn change_keeps_what_another_window_saved() {
    let dir = tmp("steps-objects-change");
    let path = dir.join(OBJECTS_FILE);
    // nothing changed: no file
    let (objects, undone) = Objects::change(&dir, |o| Ok(o.undo())).unwrap();
    assert_eq!((objects, undone), (Objects::default(), None));
    assert!(!path.exists());

    // a window that still shows the objects from before another one saved
    let stale = Objects::load(&dir).unwrap();
    let (_, a) = Objects::change(&dir, |o| o.add(socket(1.0))).unwrap();
    assert_eq!(stale, Objects::default());
    let (objects, b) = Objects::change(&dir, |o| o.add(socket(2.0))).unwrap();
    assert_ne!(a, b);
    assert_eq!(numbers(&objects), [a, b]);
    assert_eq!(Objects::load(&dir).unwrap(), objects);

    // a refused change writes nothing, also what it did before it was refused
    let before = fs::read(&path).unwrap();
    let err = Objects::change(&dir, |o| {
        o.rename(a, "nicht gespeichert");
        o.add(socket(f64::NAN))
    })
    .unwrap_err();
    assert!(matches!(err, ObjectsError::Invalid(_)), "{err}");
    assert_eq!(fs::read(&path).unwrap(), before);

    // a change that changes nothing does not write either
    fs::write(
        &path,
        format!("# von Hand\n{}", fs::read_to_string(&path).unwrap()),
    )
    .unwrap();
    let edited = fs::read(&path).unwrap();
    let (_, there) = Objects::change(&dir, |o| Ok(o.rename(a, ""))).unwrap();
    assert!(there);
    assert_eq!(fs::read(&path).unwrap(), edited);
}

#[test]
fn changes_at_the_same_time_lose_nothing() {
    let dir = tmp("steps-objects-together");
    std::thread::scope(|s| {
        for t in 0..3 {
            let dir = &dir;
            s.spawn(move || {
                for i in 0..3 {
                    let x = f64::from(t) + f64::from(i) / 10.0;
                    Objects::change(dir, |o| o.add(socket(x))).unwrap();
                }
            });
        }
    });
    let objects = Objects::load(&dir).unwrap();
    let mut nrs = numbers(&objects);
    nrs.sort_unstable();
    assert_eq!(nrs, (1..=9).collect::<Vec<u32>>());
}

#[test]
fn file_name_is_safe() {
    assert_eq!(OBJECTS_FILE, "objekte.toml");
    assert!(!OBJECTS_FILE.starts_with("oberflaeche"));
    assert!(!OBJECTS_FILE.ends_with(".tmp"));

    // saving leaves nothing else in the folder: no `.tmp`, no lock
    let dir = tmp("steps-objects-file");
    one_of_each().save(&dir).unwrap();
    Objects::change(&dir, |o| Ok(o.undo())).unwrap();
    let names: Vec<String> = fs::read_dir(&dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(names, [OBJECTS_FILE]);
}

#[test]
fn source_is_hand() {
    let dir = tmp("steps-objects-source");
    let mut objects = Objects::default();
    let nr = objects.add(socket(1.0)).unwrap();
    assert_eq!(objects.get(nr).unwrap().source, "hand");
    objects.save(&dir).unwrap();
    let path = dir.join(OBJECTS_FILE);
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.contains("quelle = \"hand\""), "{text}");

    // what another source wrote stays its own
    fs::write(
        &path,
        text.replace("quelle = \"hand\"", "quelle = \"erkennung\""),
    )
    .unwrap();
    let mut objects = Objects::load(&dir).unwrap();
    assert_eq!(objects.get(nr).unwrap().source, "erkennung");
    assert!(objects.rename(nr, "Flur"));
    assert_eq!(objects.get(nr).unwrap().source, "erkennung");
}

#[test]
fn an_object_written_by_hand_needs_only_the_essentials() {
    let dir = tmp("steps-objects-by-hand");
    let path = dir.join(OBJECTS_FILE);
    fs::write(
        &path,
        "[[objekt]]\nnr = 7\nart = \"steckdose\"\npunkte = [[1, 2, 0.3]]\n",
    )
    .unwrap();
    let mut objects = Objects::load(&dir).unwrap();
    let written = objects.get(7).unwrap();
    assert_eq!(written.points, [[1.0, 2.0, 0.3]]);
    assert_eq!((written.name.as_str(), written.note.as_str()), ("", ""));
    assert_eq!(written.source, "hand");

    // ↶ never takes it away, it was not set in the program
    assert!(!objects.can_undo());
    assert_eq!(objects.undo(), None);
    assert_eq!(objects.add(socket(1.0)).unwrap(), 8);
    assert_eq!(objects.undo(), Some(Undone::Added(8)));
    assert_eq!(objects.undo(), None);
    assert_eq!(numbers(&objects), [7]);

    // a change of it can be taken back, it stays afterwards
    assert!(objects.rename(7, "Flur"));
    assert_eq!(objects.undo(), Some(Undone::Changed(7)));
    assert_eq!(objects.get(7).unwrap().name, "");
    assert_eq!(objects.undo(), None);
    assert_eq!(numbers(&objects), [7]);
}

#[test]
fn a_damaged_file_is_an_error_and_stays() {
    let dir = tmp("steps-objects-damaged");
    let path = dir.join(OBJECTS_FILE);
    let plug = "[[objekt]]\nnr = 1\nart = \"steckdose\"\npunkte = [[0.0, 0.0, 0.3]]\n";
    let is_format = |e: &ObjectsError| matches!(e, ObjectsError::Format(_));
    let is_invalid = |e: &ObjectsError| matches!(e, ObjectsError::Invalid(_));
    type Check<'a> = &'a dyn Fn(&ObjectsError) -> bool;
    let cases: [(String, &str, Check); 8] = [
        // no TOML
        (
            "[[objekt]\nnr = 1\n".to_string(),
            "nicht lesbar",
            &is_format,
        ),
        // an unknown key beside `objekt`, in a removal
        ("next_seq = 1\nfarbe = 2\n".to_string(), "farbe", &is_format),
        (
            format!(
                "[[geloescht]]\nseq = 1\ngrund = \"x\"\n[geloescht.objekt]\n{}",
                &plug[11..]
            ),
            "grund",
            &is_format,
        ),
        // an unknown kind, a value of the wrong type
        (plug.replace("steckdose", "lampe"), "lampe", &is_format),
        ("[[objekt]]\nnr = \"eins\"\n".to_string(), "nr", &is_format),
        // against the rules, in the list and among what ↶ can bring back
        (
            plug.replace("[[0.0, 0.0, 0.3]]", "[]"),
            "Objekt 1: Eine Steckdose braucht genau einen Punkt",
            &is_invalid,
        ),
        (
            format!(
                "[[geloescht]]\nseq = 1\n[geloescht.objekt]\n{}",
                plug[11..].replace("[[0.0, 0.0, 0.3]]", "[[nan, 0.0, 0.3]]")
            ),
            "Objekt 1: ",
            &is_invalid,
        ),
        // one number twice
        (format!("{plug}{plug}"), "Nummer 1", &is_invalid),
    ];
    for (text, part, right_kind) in cases {
        fs::write(&path, &text).unwrap();
        let err = Objects::load(&dir).unwrap_err();
        assert!(right_kind(&err), "{text}: {err:?}");
        let message = err.to_string();
        assert!(
            message.contains(OBJECTS_FILE) && message.contains(part),
            "{text}: {message}"
        );
        assert_eq!(Objects::change(&dir, |o| Ok(o.undo())).unwrap_err(), err);
        assert_eq!(fs::read_to_string(&path).unwrap(), text);
    }
}

#[test]
fn labels_name_kind_number_and_name() {
    let mut objects = Objects::default();
    let kinds = [
        (socket(1.0), "Steckdose"),
        (connection(ConnectionKind::Water), "Anschluss"),
        (input(ObjectKind::Cable, &[P, Q]), "Kabel"),
        (pipe(&[P, Q], Some(50.0)), "Rohr"),
        (measure(1.0), "Messung"),
    ];
    for (i, (object, kind)) in kinds.into_iter().enumerate() {
        let nr = objects.add(object).unwrap();
        assert_eq!(nr as usize, i + 1);
        assert_eq!(objects.get(nr).unwrap().label(), format!("{kind} {nr}"));
    }
    assert!(objects.rename(1, "Küche"));
    assert_eq!(objects.get(1).unwrap().label(), "Steckdose 1 · Küche");
    // a name of blanks is no name
    assert!(objects.rename(1, "  "));
    assert_eq!(objects.get(1).unwrap().label(), "Steckdose 1");
}

#[test]
fn errors_are_german_and_step_errors() {
    let dir = tmp("steps-objects-locked");
    // a lock another window holds: saving waits, then gives up and writes nothing
    let held = FileLock::acquire(&dir.join(OBJECTS_FILE)).unwrap();
    let objects = one_of_each();
    let err = objects.save(&dir).unwrap_err();
    assert!(
        matches!(&err, ObjectsError::Lock(m) if m.contains("anderen Fenster")),
        "{err}"
    );
    assert!(!dir.join(OBJECTS_FILE).exists());
    assert_eq!(
        StepError::from(err.clone()),
        StepError::Failed(err.to_string())
    );
    drop(held);
    objects.save(&dir).unwrap();
    assert_eq!(Objects::load(&dir).unwrap(), objects);
}
