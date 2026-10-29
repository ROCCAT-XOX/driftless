//! `stellen.toml`: the marks painted on the surfaces of a map or model, with their steps,
//! through the library.

use driftless_io::testscan::tmp;
use driftless_steps::marks::{
    DabInput, EditStep, Marks, MarksError, Placement, SmoothLevel, StepInput, Undone, MARKS_FILE,
};
use std::collections::BTreeSet;
use std::fs;

/// A dab of 20 cm on a wall that faces +x.
fn dab(centre: [f64; 3]) -> DabInput {
    DabInput {
        centre,
        normal: [1.0, 0.0, 0.0],
        radius: 0.2,
        erase: false,
    }
}

/// A stroke of `n` dabs 5 cm apart, upwards from `start`.
fn stroke(start: [f64; 3], n: usize) -> Vec<DabInput> {
    (0..n)
        .map(|i| dab([start[0], start[1], start[2] + 0.05 * i as f64]))
        .collect()
}

/// Where the wall of [`two_marks`] is: far from the origin, as centres are absolute.
const WALL_X: f64 = 1_234.567_890_123;

/// Mark 1 on a wall: a stroke, smoothed strongly, an erasing dab, new images. Mark 2 on the
/// floor: one dab, smoothed flat.
fn two_marks() -> Marks {
    let mut marks = Marks::default();
    let wall = marks.paint(None, &stroke([WALL_X, -0.1, 1e-7], 3));
    let smooth = |level| StepInput::Smooth { level };
    marks.add_step(wall, smooth(SmoothLevel::Strong)).unwrap();
    let floor = marks.paint(
        None,
        &[DabInput {
            centre: [0.25, -1.125, 0.0],
            normal: [0.267_261_24, 0.534_522_5, 0.801_783_7],
            radius: 0.05,
            erase: false,
        }],
    );
    let erase = DabInput {
        erase: true,
        ..dab([WALL_X, -0.1, 0.05])
    };
    marks.paint(Some(wall), &[erase]);
    marks
        .add_step(wall, StepInput::Retexture { variant: 0 })
        .unwrap();
    marks.add_step(floor, smooth(SmoothLevel::Flat)).unwrap();
    marks
}

/// The variants of the „Bilder neu“ steps of the first mark.
fn variants(marks: &Marks) -> Vec<u32> {
    marks.marks[0]
        .steps
        .iter()
        .filter_map(|s| match s {
            EditStep::Retexture { variant, .. } => Some(*variant),
            EditStep::Smooth { .. } => None,
        })
        .collect()
}

#[test]
fn roundtrip_keeps_everything() {
    let dir = tmp("steps-marks-roundtrip");
    let marks = two_marks();
    assert_eq!(marks.marks.len(), 2);
    assert_eq!(marks.marks[0].dabs.len(), 4);
    assert_eq!(marks.marks[0].steps.len(), 2);
    marks.save(&dir).unwrap();
    assert_eq!(Marks::load(&dir).unwrap(), marks);

    let path = dir.join(MARKS_FILE);
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# Driftless-Stellen"), "{text}");
    for line in [
        "[[stelle.tupfer]]",
        "kind = \"glaetten\"",
        "level = \"stark\"",
        "level = \"eben\"",
        "kind = \"bilder_neu\"",
        "variant = 0",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }

    // a line added by hand to the second mark survives loading and saving
    assert_eq!(text.matches("\nnr = 2\n").count(), 1, "{text}");
    fs::write(
        &path,
        text.replace("\nnr = 2\n", "\nnr = 2\nnotiz = \"x\"\n"),
    )
    .unwrap();
    let noted = Marks::load(&dir).unwrap();
    let note = noted.marks[1].unknown.get("notiz");
    assert_eq!(note.and_then(|v| v.as_str()), Some("x"));
    noted.save(&dir).unwrap();
    let again = Marks::load(&dir).unwrap();
    assert_eq!(again, noted);
    assert!(fs::read_to_string(&path).unwrap().contains("notiz = \"x\""));
    let mut without = again.clone();
    without.marks[1].unknown.clear();
    assert_eq!(without, marks);
}

#[test]
fn missing_file_is_empty() {
    let dir = tmp("steps-marks-missing");
    assert_eq!(Marks::load(&dir).unwrap(), Marks::default());
    assert_eq!(
        Marks::load(&dir.join("karte-009")).unwrap(),
        Marks::default()
    );
    assert!(!dir.join(MARKS_FILE).exists());
}

#[test]
fn undo_takes_the_youngest() {
    // stroke B into the mark of stroke A, and into a new mark
    for same in [true, false] {
        let mut marks = Marks::default();
        let a = marks.paint(None, &stroke([1.0, 0.0, 1.0], 3));
        let light = StepInput::Smooth {
            level: SmoothLevel::Light,
        };
        marks.add_step(a, light).unwrap();
        let b = marks.paint(same.then_some(a), &stroke([1.0, 0.5, 1.0], 2));
        assert_eq!(b == a, same);

        assert_eq!(marks.undo(), Some(Undone::Stroke { mark: b, dabs: 2 }));
        assert_eq!(marks.marks.len(), 1, "a new mark goes with its only stroke");
        assert_eq!(marks.marks[0].dabs.len(), 3);
        assert!(matches!(
            marks.undo(),
            Some(Undone::Step {
                mark,
                step: EditStep::Smooth {
                    level: SmoothLevel::Light,
                    ..
                },
            }) if mark == a
        ));
        assert!(marks.marks[0].steps.is_empty());
        assert_eq!(marks.undo(), Some(Undone::Stroke { mark: a, dabs: 3 }));
        assert!(marks.marks.is_empty());
        assert_eq!(marks.undo(), None);
    }
}

#[test]
fn undo_survives_restart() {
    let dir = tmp("steps-marks-undo-restart");
    // a restart before every undo takes back the same, in the same order
    let mut kept = two_marks();
    let mut marks = two_marks();
    while let Some(undone) = kept.undo() {
        marks.save(&dir).unwrap();
        marks = Marks::load(&dir).unwrap();
        assert_eq!(marks.undo(), Some(undone));
        assert_eq!(marks, kept);
    }
    assert!(marks.marks.is_empty());

    // a stroke after a restart is younger than everything before it
    two_marks().save(&dir).unwrap();
    let mut marks = Marks::load(&dir).unwrap();
    assert_eq!(marks.paint(Some(2), &stroke([0.5, 0.5, 0.0], 4)), 2);
    marks.save(&dir).unwrap();
    let mut marks = Marks::load(&dir).unwrap();
    assert_eq!(marks.undo(), Some(Undone::Stroke { mark: 2, dabs: 4 }));
    assert_eq!(marks.marks, two_marks().marks);
}

#[test]
fn remove_takes_steps_along() {
    let dir = tmp("steps-marks-remove");
    let mut marks = two_marks();
    assert!(marks.remove(1));
    assert!(!marks.remove(1));
    assert_eq!(marks.marks.len(), 1);
    assert_eq!(marks.marks[0].nr, 2);
    marks.save(&dir).unwrap();
    let text = fs::read_to_string(dir.join(MARKS_FILE)).unwrap();
    assert!(
        !text.contains("stark") && !text.contains("bilder_neu"),
        "{text}"
    );

    // undo takes only what mark 2 had: its step, then its stroke
    let mut marks = Marks::load(&dir).unwrap();
    assert!(matches!(marks.undo(), Some(Undone::Step { mark: 2, .. })));
    assert_eq!(marks.undo(), Some(Undone::Stroke { mark: 2, dabs: 1 }));
    assert_eq!(marks.undo(), None);
}

#[test]
fn next_variant_counts_retextures() {
    let mut marks = Marks::default();
    let nr = marks.paint(None, &stroke([1.0, 0.0, 1.0], 3));
    assert_eq!(marks.marks[0].next_variant(), 0);
    for after in [1, 2] {
        let variant = marks.marks[0].next_variant();
        marks
            .add_step(nr, StepInput::Retexture { variant })
            .unwrap();
        assert_eq!(marks.marks[0].next_variant(), after);
    }
    assert_eq!(variants(&marks), [0, 1]);
}

#[test]
fn next_variant_starts_again_after_smoothing() {
    // smoothing bakes the images of the mark anew with variant 0: the next press is 1
    let mut marks = Marks::default();
    let nr = marks.paint(None, &stroke([1.0, 0.0, 1.0], 3));
    let retexture = |marks: &mut Marks| {
        let variant = marks.marks[0].next_variant();
        marks
            .add_step(nr, StepInput::Retexture { variant })
            .unwrap();
    };
    retexture(&mut marks);
    retexture(&mut marks);
    let strong = StepInput::Smooth {
        level: SmoothLevel::Strong,
    };
    marks.add_step(nr, strong).unwrap();
    assert_eq!(marks.marks[0].next_variant(), 1);
    retexture(&mut marks);
    assert_eq!(marks.marks[0].next_variant(), 2);
    assert_eq!(variants(&marks), [0, 1, 1]);
    // undone, as before the smoothing
    marks.undo();
    marks.undo();
    assert_eq!(marks.marks[0].next_variant(), 2);
}

#[test]
fn placement_from_normals() {
    let tilted = |z: f32| [(1.0 - z * z).sqrt(), 0.0, z];
    let cases = [
        ([0.0, 0.0, 1.0], Placement::Floor, "Boden"),
        ([0.0, 0.0, -1.0], Placement::Ceiling, "Decke"),
        ([1.0, 0.0, 0.0], Placement::Wall, "Wand"),
        ([0.8, 0.0, 0.6], Placement::Surface, "Fläche"),
        // the limits: from |z| 0.7 floor or ceiling, up to |z| 0.3 wall
        (tilted(0.71), Placement::Floor, "Boden"),
        (tilted(0.69), Placement::Surface, "Fläche"),
        (tilted(-0.71), Placement::Ceiling, "Decke"),
        (tilted(-0.69), Placement::Surface, "Fläche"),
        (tilted(0.29), Placement::Wall, "Wand"),
        (tilted(-0.29), Placement::Wall, "Wand"),
        (tilted(0.31), Placement::Surface, "Fläche"),
    ];
    for (normal, placement, word) in cases {
        let mut marks = Marks::default();
        let nr = marks.paint(
            None,
            &[DabInput {
                normal,
                ..dab([0.0, 0.0, 0.0])
            }],
        );
        let mark = &marks.marks[0];
        assert_eq!(mark.placement(), placement, "{normal:?}");
        assert_eq!(placement.label(), word);
        assert_eq!(mark.label(), format!("Stelle {nr} · {word}"));
    }

    // the mean normal of the dabs: a noisy floor is a floor
    let mut marks = Marks::default();
    let floor: Vec<DabInput> = [[0.3, 0.0, 0.95], [-0.25, 0.1, 0.96], [0.0, -0.3, 0.95]]
        .into_iter()
        .map(|normal| DabInput {
            normal,
            ..dab([0.0, 0.0, 0.0])
        })
        .collect();
    marks.paint(None, &floor);
    assert_eq!(marks.marks[0].label(), "Stelle 1 · Boden");
}

#[test]
fn fingerprint_changes_with_content_only() {
    let dir = tmp("steps-marks-fingerprint");
    let marks = two_marks();
    let key = marks.fingerprint();
    assert_eq!(key.len(), 64, "{key}");
    assert!(key
        .bytes()
        .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)));
    assert_eq!(two_marks().fingerprint(), key);

    // the same after saving and loading
    marks.save(&dir).unwrap();
    let loaded = Marks::load(&dir).unwrap();
    assert_eq!(loaded.fingerprint(), key);

    // a note added by hand is no content
    let mut noted = loaded.clone();
    noted.marks[1].unknown.insert("notiz".into(), "x".into());
    assert_eq!(noted.fingerprint(), key);

    // another stroke is
    let mut painted = loaded.clone();
    painted.paint(Some(2), &[dab([0.25, -1.0, 0.0])]);
    assert_ne!(painted.fingerprint(), key);
    // a stroke into a new mark as well
    let mut painted = loaded.clone();
    painted.paint(None, &[dab([0.25, -1.0, 0.0])]);
    assert_ne!(painted.fingerprint(), key);

    // a step is; undone, the key is the one from before (counters are no content)
    let mut stepped = loaded.clone();
    stepped
        .add_step(2, StepInput::Retexture { variant: 0 })
        .unwrap();
    assert_ne!(stepped.fingerprint(), key);
    stepped.undo();
    assert_eq!(stepped.fingerprint(), key);
}

#[test]
fn file_name_is_safe() {
    assert_eq!(MARKS_FILE, "stellen.toml");
    assert!(!MARKS_FILE.starts_with("oberflaeche"));
    assert!(!MARKS_FILE.ends_with(".tmp"));

    // saving leaves nothing else in the folder: no `.tmp`, no lock
    let dir = tmp("steps-marks-file");
    two_marks().save(&dir).unwrap();
    Marks::change(&dir, |m| Ok(m.undo())).unwrap();
    let names: Vec<String> = fs::read_dir(&dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(names, [MARKS_FILE]);
}

#[test]
fn change_works_on_the_file_as_it_is_now() {
    let dir = tmp("steps-marks-change");
    // nothing changed: no file
    let (marks, undone) = Marks::change(&dir, |m| Ok(m.undo())).unwrap();
    assert_eq!((marks, undone), (Marks::default(), None));
    assert!(!dir.join(MARKS_FILE).exists());

    // a window that still shows the marks from before another one painted
    let stale = Marks::load(&dir).unwrap();
    let (_, a) = Marks::change(&dir, |m| Ok(m.paint(None, &stroke([1.0, 0.0, 1.0], 3)))).unwrap();
    assert_eq!(stale, Marks::default());
    let (marks, b) =
        Marks::change(&dir, |m| Ok(m.paint(None, &stroke([2.0, 0.0, 1.0], 2)))).unwrap();
    assert_ne!(a, b);
    assert_eq!(marks.marks.len(), 2);
    assert_eq!(Marks::load(&dir).unwrap(), marks);

    // a refused change writes nothing
    let before = fs::read(dir.join(MARKS_FILE)).unwrap();
    let err = Marks::change(&dir, |m| {
        m.paint(Some(a), &stroke([1.0, 0.2, 1.0], 2));
        let light = StepInput::Smooth {
            level: SmoothLevel::Light,
        };
        m.add_step(99, light)
    })
    .unwrap_err();
    assert_eq!(err, MarksError::NoMark(99));
    assert_eq!(fs::read(dir.join(MARKS_FILE)).unwrap(), before);
}

#[test]
fn changes_at_the_same_time_lose_nothing() {
    let dir = tmp("steps-marks-together");
    std::thread::scope(|s| {
        for t in 0..3 {
            let dir = &dir;
            s.spawn(move || {
                for i in 0..3 {
                    let start = [f64::from(t), f64::from(i), 1.0];
                    Marks::change(dir, |m| Ok(m.paint(None, &stroke(start, 2)))).unwrap();
                }
            });
        }
    });
    let marks = Marks::load(&dir).unwrap();
    let nrs: BTreeSet<u32> = marks.marks.iter().map(|m| m.nr).collect();
    assert_eq!(nrs, (1..=9).collect());
    let seqs: BTreeSet<u64> = marks
        .marks
        .iter()
        .flat_map(|m| m.dabs.iter().map(|d| d.seq))
        .collect();
    assert_eq!(seqs, (1..=9).collect());
}

#[test]
fn unreadable_file_is_an_error_and_stays() {
    let dir = tmp("steps-marks-unreadable");
    let path = dir.join(MARKS_FILE);
    let dab = "[[stelle.tupfer]]\ncentre = [0.0, 0.0, 0.0]\nnormal = [1.0, 0.0, 0.0]\n\
               radius = 0.2\nerase = false\nseq = 1\n";
    for (text, part) in [
        // no TOML
        ("[[stelle]\nnr = 1\n".to_string(), "nicht lesbar"),
        // an unknown key beside `stelle`, in a dab, in a step
        ("next_seq = 1\nfarbe = 2\n".to_string(), "farbe"),
        (format!("[[stelle]]\nnr = 1\n{dab}druck = 0.5\n"), "druck"),
        (
            "[[stelle]]\nnr = 1\n[[stelle.schritt]]\nkind = \"glaetten\"\nlevel = \"eben\"\n\
             seq = 1\nstaerke = 2\n"
                .to_string(),
            "staerke",
        ),
        // a value of the wrong kind, a missing one
        ("[[stelle]]\nnr = \"eins\"\n".to_string(), "nr"),
        (format!("[[stelle]]\n{dab}"), "nr"),
    ] {
        fs::write(&path, &text).unwrap();
        let err = Marks::load(&dir).unwrap_err();
        assert!(
            matches!(&err, MarksError::Unreadable(m) if m.contains(MARKS_FILE) && m.contains(part)),
            "{err}"
        );
        assert_eq!(Marks::change(&dir, |m| Ok(m.undo())).unwrap_err(), err);
        assert_eq!(fs::read_to_string(&path).unwrap(), text);
    }
}
