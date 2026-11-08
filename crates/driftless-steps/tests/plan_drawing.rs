//! The floor plan as a drawing: walls cut at the openings, symbols and texts, where things lie
//! in the two placements, blocks and attributes of the objects. All plans are made by hand.

use driftless_export::dxf::{dxf_bytes, write_dxf, Drawing, Entity, EntityKind, Shape};
use driftless_io::progress::Progress;
use driftless_io::testscan::tmp;
use driftless_plan::{
    Dim, FloorPlan, Leveling, Opening, OpeningKind, PlanFrame, Room, Storey, WallFace, WallPair,
};
use driftless_steps::objects::{ConnectionKind, ObjectInput, ObjectKind, Objects};
use driftless_steps::plan::{PlanReport, PlanSettings, TopViewInfo, PLAN_VERSION};
use driftless_steps::plan_drawing::{
    elevation_mm, plan_drawing, PlanExport, PlanPlacement, LAYERS,
};
use dxf::entities::EntityType;
use dxf::XDataItem;
use std::collections::BTreeMap;

const LEVEL_FRAME: PlanFrame = PlanFrame {
    axis: [1.0, 0.0],
    origin: [0.0, 0.0, 0.0],
    leveling: Leveling::NONE,
};

fn face(a: [f64; 2], b: [f64; 2], inward: [f64; 2]) -> WallFace {
    WallFace {
        a,
        b,
        inward,
        seen_m: (b[0] - a[0]).hypot(b[1] - a[1]),
    }
}

/// A wall of 10 m along x, the room above it.
fn long_wall() -> WallFace {
    face([0.0, 0.0], [10.0, 0.0], [0.0, 1.0])
}

fn opening(kind: OpeningKind, face: usize, from: f64, to: f64, sill: f64, head: f64) -> Opening {
    Opening {
        kind,
        face,
        from,
        to,
        sill,
        head,
        crossed: false,
        note: String::new(),
    }
}

fn plan_of(faces: Vec<WallFace>) -> FloorPlan {
    FloorPlan {
        storey: Storey {
            floor_z: 0.0,
            ceiling_z: Some(2.6),
            pose_share: 1.0,
            floor_area: 20.0,
        },
        storeys: Vec::new(),
        frame: LEVEL_FRAME,
        faces,
        pairs: Vec::new(),
        openings: Vec::new(),
        rooms: Vec::new(),
        dims: Vec::new(),
        hints: Vec::new(),
        cell: 0.02,
    }
}

fn report(plan: FloorPlan) -> PlanReport {
    PlanReport {
        input_id: "abc".to_string(),
        version: PLAN_VERSION,
        settings: PlanSettings::default(),
        plan,
        top_view: TopViewInfo {
            width: 1,
            height: 1,
            origin: [0.0, 0.0],
            cell: 0.02,
        },
        files: Vec::new(),
        bytes: BTreeMap::new(),
        points: 0,
        seconds_total: 0.0,
        peak_rss_bytes: None,
    }
}

fn draw(plan: FloorPlan) -> Drawing {
    plan_drawing(&report(plan), &Objects::default(), &PlanExport::default())
}

fn input(kind: ObjectKind, points: Vec<[f64; 3]>) -> ObjectInput {
    ObjectInput {
        kind,
        points,
        name: String::new(),
        note: String::new(),
        connection: None,
        diameter_mm: None,
        height_m: None,
        sill_m: None,
        properties: BTreeMap::new(),
    }
}

fn add(objects: &mut Objects, input: ObjectInput) {
    objects.add(input).unwrap();
}

fn entities<'a>(d: &'a Drawing, layer: &str) -> Vec<&'a Entity> {
    d.entities.iter().filter(|e| e.layer == layer).collect()
}

fn shapes<'a>(d: &'a Drawing, layer: &str) -> Vec<&'a Shape> {
    entities(d, layer)
        .into_iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Shape(s) => Some(s),
            EntityKind::Insert { .. } => None,
        })
        .collect()
}

fn texts<'a>(d: &'a Drawing, layer: &str) -> Vec<&'a str> {
    shapes(d, layer)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Text { value, .. } => Some(value.as_str()),
            _ => None,
        })
        .collect()
}

/// `a`, `b` and whether dashed, of every `Line`.
fn lines(d: &Drawing, layer: &str) -> Vec<([f64; 2], [f64; 2], bool)> {
    shapes(d, layer)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Line { a, b, dashed } => Some((*a, *b, *dashed)),
            _ => None,
        })
        .collect()
}

fn polylines<'a>(d: &'a Drawing, layer: &str) -> Vec<&'a Vec<[f64; 2]>> {
    shapes(d, layer)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Polyline { points, .. } => Some(points),
            _ => None,
        })
        .collect()
}

fn length(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

/// Within a millimetre.
fn near(a: [f64; 2], b: [f64; 2]) -> bool {
    length(a, b) < 1.0
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The attributes of the only `INSERT` on `layer`.
fn attributes<'a>(d: &'a Drawing, layer: &str) -> &'a [(String, String)] {
    let found: Vec<_> = entities(d, layer)
        .into_iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Insert { attributes, .. } => Some(attributes.as_slice()),
            EntityKind::Shape(_) => None,
        })
        .collect();
    assert_eq!(found.len(), 1, "{layer}");
    found[0]
}

fn value<'a>(attributes: &'a [(String, String)], tag: &str) -> &'a str {
    attributes
        .iter()
        .find(|(t, _)| t == tag)
        .unwrap_or_else(|| panic!("{tag} fehlt"))
        .1
        .as_str()
}

fn insert_at(d: &Drawing, layer: &str) -> [f64; 2] {
    entities(d, layer)
        .into_iter()
        .find_map(|e| match &e.kind {
            EntityKind::Insert { at, .. } => Some(*at),
            EntityKind::Shape(_) => None,
        })
        .expect("INSERT")
}

#[test]
fn walls_cut_at_openings() {
    let mut plan = plan_of(vec![
        long_wall(),
        face([0.0, 5.0], [6.0, 5.0], [0.0, -1.0]),
        face([0.0, 5.2], [6.0, 5.2], [0.0, 1.0]),
    ]);
    plan.pairs = vec![WallPair {
        front: 1,
        back: 2,
        thickness: 0.2,
    }];
    plan.openings = vec![
        opening(OpeningKind::Door, 0, 2.0, 3.0, 0.0, 2.05),
        // up to the end of the face: no piece behind it
        opening(OpeningKind::Window, 0, 8.0, 10.0, 0.9, 2.2),
        // the same door, seen from the two faces of a wall
        opening(OpeningKind::Door, 1, 1.0, 2.0, 0.0, 2.05),
        opening(OpeningKind::Door, 2, 1.0, 2.0, 0.0, 2.05),
    ];
    let d = draw(plan);

    let pieces = polylines(&d, "Wand");
    let on_y = |y: f64| -> Vec<(f64, f64)> {
        pieces
            .iter()
            .filter(|p| close(p[0][1], y))
            .map(|p| (p[0][0], p[1][0]))
            .collect()
    };
    assert_eq!(
        on_y(0.0),
        [(0.0, 2000.0), (3000.0, 8000.0)],
        "a piece before and after the door, none behind the window"
    );
    assert_eq!(on_y(5000.0), [(0.0, 1000.0), (2000.0, 6000.0)]);
    assert_eq!(
        on_y(5200.0),
        [(0.0, 1000.0), (2000.0, 6000.0)],
        "beide Flächen"
    );
    assert_eq!(pieces.len(), 6);

    let caps = lines(&d, "Wand");
    let of = |y: f64, x: f64| {
        caps.iter()
            .find(|(a, _, _)| close(a[1], y) && close(a[0], x))
    };
    let (a, b, dashed) = of(0.0, 2000.0).expect("Endkappe");
    assert!(!dashed);
    assert!(close(length(*a, *b), 120.0), "ohne Paar 120 mm");
    assert!(b[1] < a[1], "in die Wand, von der Raumseite weg");
    assert!(of(0.0, 3000.0).is_some());
    let (a, b, _) = of(5000.0, 1000.0).expect("Endkappe der gepaarten Wand");
    assert!(close(length(*a, *b), 200.0), "Dicke aus dem Paar");
    assert!(close(b[1], 5200.0), "reicht bis zur anderen Fläche");
    assert_eq!(caps.len(), 6, "je Öffnung zwei, die doppelte nur einmal");
    assert_eq!(texts(&d, "Tür").len(), 2, "die doppelte Tür nur einmal");
}

#[test]
fn door_and_window_have_symbols_and_text() {
    let mut plan = plan_of(vec![long_wall()]);
    plan.openings = vec![
        opening(OpeningKind::Door, 0, 1.0, 1.9, 0.0, 2.05),
        opening(OpeningKind::Window, 0, 4.0, 5.2, 0.9, 2.2),
    ];
    let d = draw(plan);

    let door = lines(&d, "Tür");
    assert_eq!(door.len(), 3, "zwei Laibungen, eine Schwelle");
    let dashed: Vec<_> = door.iter().filter(|l| l.2).collect();
    assert_eq!(dashed.len(), 1, "die Schwelle ist gestrichelt");
    assert!(near(dashed[0].0, [1000.0, 0.0]) && near(dashed[0].1, [1900.0, 0.0]));
    let reveals: Vec<_> = door.iter().filter(|l| !l.2).collect();
    assert!(close(reveals[0].0[0], 1000.0) && close(reveals[1].0[0], 1900.0));
    assert_eq!(texts(&d, "Tür"), ["Tür 0,90 × 2,05 m"]);

    let window = lines(&d, "Fenster");
    assert_eq!(window.len(), 2);
    assert!(window.iter().all(|l| !l.2), "durchgezogen");
    assert!(close(window[0].0[0], 4000.0) && close(window[0].1[0], 5200.0));
    assert!(
        close((window[0].0[1] - window[1].0[1]).abs(), 60.0),
        "Doppellinie 60 mm"
    );
    let text = texts(&d, "Fenster");
    assert_eq!(text, ["Fenster 1,20 × 1,30 m, BRH 0,90 m"]);
    assert!(text[0].contains("BRH"));
}

#[test]
fn unclear_says_unclear_and_keeps_its_reason_in_the_data() {
    let mut plan = plan_of(vec![long_wall()]);
    let mut unclear = opening(OpeningKind::Unclear, 0, 3.0, 4.0, 0.0, 2.0);
    unclear.note = "Tür oder bodentiefes Fenster?".to_string();
    plan.openings = vec![unclear];
    let d = draw(plan);
    let gap = lines(&d, "Öffnung-unklar");
    assert_eq!(gap.len(), 1);
    assert!(gap[0].2, "gestrichelt");
    assert_eq!(texts(&d, "Öffnung-unklar"), ["unklar"]);
    let dashed = entities(&d, "Öffnung-unklar")
        .into_iter()
        .find(|e| !e.xdata.is_empty())
        .expect("Grund in den Daten");
    assert_eq!(dashed.xdata, ["GRUND=Tür oder bodentiefes Fenster?"]);
}

#[test]
fn texts_of_neighbouring_openings_do_not_lie_on_each_other() {
    let mut plan = plan_of(vec![long_wall()]);
    plan.openings = vec![
        opening(OpeningKind::Window, 0, 2.0, 2.4, 1.0, 2.0),
        opening(OpeningKind::Window, 0, 2.5, 2.9, 1.0, 2.0),
        opening(OpeningKind::Door, 0, 3.0, 3.9, 0.0, 2.05),
    ];
    let d = draw(plan);
    let boxes: Vec<(String, [f64; 4])> = d
        .entities
        .iter()
        .filter_map(|e| match &e.kind {
            EntityKind::Shape(Shape::Text {
                at, height, value, ..
            }) => {
                let width = value.chars().count() as f64 * 0.6 * height;
                Some((value.clone(), [at[0], at[1], at[0] + width, at[1] + height]))
            }
            _ => None,
        })
        .collect();
    assert_eq!(boxes.len(), 3, "{boxes:?}");
    for (i, (a, ba)) in boxes.iter().enumerate() {
        for (b, bb) in &boxes[i + 1..] {
            let apart = ba[0] >= bb[2] || bb[0] >= ba[2] || ba[1] >= bb[3] || bb[1] >= ba[3];
            assert!(apart, "{a} {ba:?} / {b} {bb:?}");
        }
    }
}

#[test]
fn room_label_and_dimensions() {
    let mut plan = plan_of(vec![long_wall()]);
    plan.rooms = vec![
        Room {
            nr: 1,
            outline: vec![[0.0, 0.0], [8.3, 0.0], [8.3, 7.9], [0.0, 7.9]],
            area: 65.57,
            height: Some(2.6),
            open: false,
            label_at: [4.0, 4.0],
        },
        Room {
            nr: 2,
            outline: vec![[9.0, 0.0], [12.0, 0.0], [12.0, 3.0]],
            area: 4.5,
            height: None,
            open: true,
            label_at: [11.0, 1.0],
        },
    ];
    plan.dims = vec![Dim {
        a: [0.0, 0.0],
        b: [8.3, 0.0],
        line_a: [0.0, 0.3],
        line_b: [8.3, 0.3],
        text: "8,30 m".to_string(),
    }];
    let d = draw(plan);

    let outlines: Vec<_> = shapes(&d, "Raum")
        .into_iter()
        .filter(|s| matches!(s, Shape::Polyline { .. }))
        .collect();
    assert_eq!(outlines.len(), 2);
    let Shape::Polyline { points, closed, .. } = outlines[0] else {
        panic!()
    };
    assert!(*closed, "geschlossen");
    assert_eq!(points.len(), 4);
    assert!(near(points[2], [8300.0, 7900.0]));
    let text = texts(&d, "Raum");
    for line in ["Raum 1", "65,57 m²", "Höhe 2,60 m"] {
        assert!(text.contains(&line), "{line} in {text:?}");
    }
    assert!(text.contains(&"Raum 2 (offen)"), "{text:?}");
    assert_eq!(text.len(), 5, "ohne Decke keine Höhe");

    let dim = lines(&d, "Bemaßung");
    assert_eq!(dim.len(), 3, "Linie und zwei Häkchen");
    assert!(near(dim[0].0, [0.0, 300.0]) && near(dim[0].1, [8300.0, 300.0]));
    for (end, tick) in [(dim[0].0, dim[1]), (dim[0].1, dim[2])] {
        let centre = [(tick.0[0] + tick.1[0]) / 2.0, (tick.0[1] + tick.1[1]) / 2.0];
        assert!(near(centre, end), "Häkchen auf dem Ende: {tick:?}");
        assert!(length(tick.0, tick.1) > 40.0, "{tick:?}");
    }
    assert_eq!(texts(&d, "Bemaßung"), ["8,30 m"]);
}

#[test]
fn all_layers_exist_even_when_empty() {
    let d = draw(plan_of(Vec::new()));
    let got: Vec<(&str, u8)> = d
        .layers
        .iter()
        .map(|l| (l.name.as_str(), l.color))
        .collect();
    assert_eq!(got, LAYERS);
    assert_eq!(
        LAYERS.map(|l| l.0),
        [
            "Wand",
            "Tür",
            "Fenster",
            "Öffnung-unklar",
            "Raum",
            "Bemaßung",
            "Messung",
            "Steckdose",
            "Anschluss",
            "Kabel",
            "Rohr"
        ]
    );
    assert!(d.layers.iter().all(|l| l.on));
    assert!(d.entities.is_empty());

    let names: Vec<&str> = d.blocks.iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names, ["Steckdose", "Anschluss"]);
    for block in &d.blocks {
        assert_eq!(block.attributes, ["NR", "NAME", "ART", "HOEHE", "NOTIZ"]);
        assert!(!block.shapes.is_empty());
    }

    let comments = d.comments.join("\n");
    assert!(comments.contains("Driftless"), "{comments}");
    assert!(
        comments.contains(driftless_steps::PROGRAM.version),
        "{comments}"
    );
    assert!(comments.contains("Lage"), "{comments}");
    assert!(comments.contains("Drehung") && comments.contains("Verschiebung"));
    assert!(comments.contains("Rohre: von Hand gesetzt, automatische Erkennung folgt"));
    dxf_bytes(&d, 0.0).expect("eine leere Zeichnung lässt sich schreiben");
}

#[test]
fn hidden_layer_is_off() {
    let mut plan = plan_of(vec![long_wall()]);
    plan.dims = vec![Dim {
        a: [0.0, 0.0],
        b: [1.0, 0.0],
        line_a: [0.0, 0.3],
        line_b: [1.0, 0.3],
        text: "1,00 m".to_string(),
    }];
    let export = PlanExport {
        hidden: vec!["Bemaßung".to_string()],
        ..PlanExport::default()
    };
    let d = plan_drawing(&report(plan), &Objects::default(), &export);
    let off: Vec<&str> = d
        .layers
        .iter()
        .filter(|l| !l.on)
        .map(|l| l.name.as_str())
        .collect();
    assert_eq!(off, ["Bemaßung"]);
    assert_eq!(texts(&d, "Bemaßung"), ["1,00 m"], "mitgeschrieben, nur aus");
}

#[test]
fn as_cloud_moves_it_and_walls_is_default() {
    let (sin, cos) = 23f64.to_radians().sin_cos();
    let origin = [12.5, -3.0, -1.3];
    let to_map = |u: f64, v: f64| {
        [
            (origin[0] + cos * u - sin * v) * 1000.0,
            (origin[1] + sin * u + cos * v) * 1000.0,
        ]
    };
    let mut plan = plan_of(vec![face([1.0, 2.0], [5.0, 2.0], [0.0, 1.0])]);
    plan.frame.axis = [cos, sin];
    plan.frame.origin = origin;
    plan.openings = vec![opening(OpeningKind::Door, 0, 1.0, 2.0, 0.0, 2.05)];
    let frame = plan.frame;
    let report = report(plan);

    let mut objects = Objects::default();
    let socket = frame.to_map([3.0, 1.0, 0.4]);
    add(&mut objects, input(ObjectKind::Socket, vec![socket]));

    assert_eq!(PlanExport::default().placement, PlanPlacement::Walls);
    assert_eq!(PlanExport::default().hidden, Vec::<String>::new());

    let walls = plan_drawing(&report, &objects, &PlanExport::default());
    let first = polylines(&walls, "Wand")[0];
    assert!(near(first[0], [1000.0, 2000.0]), "{first:?}");
    assert!(near(insert_at(&walls, "Steckdose"), [3000.0, 1000.0]));
    assert!(close(elevation_mm(&report, &PlanExport::default()), 0.0));

    let cloud = PlanExport {
        placement: PlanPlacement::AsCloud,
        hidden: Vec::new(),
    };
    let d = plan_drawing(&report, &objects, &cloud);
    let first = polylines(&d, "Wand")[0];
    assert!(near(first[0], to_map(1.0, 2.0)), "{first:?}");
    assert!(near(first[1], to_map(2.0, 2.0)), "{first:?}");
    let at = insert_at(&d, "Steckdose");
    assert!(near(at, [socket[0] * 1000.0, socket[1] * 1000.0]), "{at:?}");
    let Some(Shape::Text { rotation_deg, .. }) = shapes(&d, "Tür")
        .into_iter()
        .find(|s| matches!(s, Shape::Text { .. }))
    else {
        panic!("Text fehlt");
    };
    assert!((rotation_deg - 23.0).abs() < 0.01, "{rotation_deg}");
    assert!((elevation_mm(&report, &cloud) + 1300.0).abs() < 1e-6);
    assert_eq!(
        value(attributes(&d, "Steckdose"), "HOEHE"),
        "400",
        "die Höhe bleibt die über dem Boden"
    );
}

#[test]
fn socket_has_attributes() {
    let mut objects = Objects::default();
    let mut socket = input(ObjectKind::Socket, vec![[2.0, 3.0, 0.31]]);
    socket.name = "Küche".to_string();
    socket.note = "hinter dem Schrank".to_string();
    add(&mut objects, socket);
    let mut water = input(ObjectKind::Connection, vec![[1.0, 1.0, 1.2345]]);
    water.connection = Some(ConnectionKind::Water);
    add(&mut objects, water);
    let d = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );

    let list = attributes(&d, "Steckdose");
    assert_eq!(value(list, "NR"), "1");
    assert_eq!(value(list, "NAME"), "Küche");
    assert_eq!(value(list, "ART"), "Steckdose");
    assert_eq!(value(list, "NOTIZ"), "hinter dem Schrank");
    let height: i64 = value(list, "HOEHE").parse().expect("ganzzahlig");
    assert!((height - 310).abs() <= 1, "{height}");
    assert!(near(insert_at(&d, "Steckdose"), [2000.0, 3000.0]));

    let list = attributes(&d, "Anschluss");
    assert_eq!(value(list, "ART"), "Wasser");
    assert_eq!(value(list, "HOEHE"), "1235");
    assert!(near(insert_at(&d, "Anschluss"), [1000.0, 1000.0]));
    let block = d.blocks.iter().find(|b| b.name == "Anschluss").unwrap();
    assert_eq!(block.attributes[0], "NR", "die Nummer steht sichtbar");
    assert_eq!(
        d.blocks.iter().map(|b| b.name.as_str()).collect::<Vec<_>>(),
        ["Steckdose", "Anschluss"]
    );
}

#[test]
fn free_properties_become_attributes() {
    let long_key = "Lange Bezeichnung ".repeat(30);
    let mut objects = Objects::default();
    let mut socket = input(ObjectKind::Socket, vec![[0.0, 0.0, 0.3]]);
    socket.properties = BTreeMap::from([
        ("Größe".to_string(), "16 A".to_string()),
        ("hersteller".to_string(), "Gira".to_string()),
        // takes a name the block already has: it must not overwrite it
        ("Name".to_string(), "frei".to_string()),
        // the same tag as `a_b`
        ("a b".to_string(), "1".to_string()),
        ("a_b".to_string(), "2".to_string()),
        // nothing of it is left
        ("😀".to_string(), "lacht".to_string()),
        (long_key.clone(), "lang".to_string()),
    ]);
    socket.name = "Küche".to_string();
    add(&mut objects, socket);
    let mut second = input(ObjectKind::Socket, vec![[1.0, 0.0, 0.3]]);
    second.properties = BTreeMap::from([("nur hier".to_string(), "x".to_string())]);
    add(&mut objects, second);
    let d = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );

    let block = d.blocks.iter().find(|b| b.name == "Steckdose").unwrap();
    for tag in [
        "GROESSE",
        "HERSTELLER",
        "NAME_2",
        "A_B",
        "A_B_2",
        "EIGENSCHAFT",
        "NUR_HIER",
    ] {
        assert!(
            block.attributes.contains(&tag.to_string()),
            "{tag}: {:?}",
            block.attributes
        );
    }
    for tag in &block.attributes {
        assert!(tag
            .chars()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_'));
        assert!(tag.len() <= 70, "{tag}");
    }
    assert!(
        !d.blocks[1].attributes.contains(&"GROESSE".to_string()),
        "nur beim eigenen Block"
    );

    let first = entities(&d, "Steckdose")[0];
    let EntityKind::Insert { attributes, .. } = &first.kind else {
        panic!()
    };
    assert_eq!(value(attributes, "GROESSE"), "16 A");
    assert_eq!(value(attributes, "HERSTELLER"), "Gira");
    assert_eq!(value(attributes, "NAME"), "Küche");
    assert_eq!(value(attributes, "NAME_2"), "frei");
    assert_eq!(value(attributes, "A_B"), "1");
    assert_eq!(value(attributes, "A_B_2"), "2");
    assert_eq!(value(attributes, "EIGENSCHAFT"), "lacht");

    // Every tag an INSERT uses is one of its block: the writer checks that.
    let bytes = dxf_bytes(&d, 0.0).expect("die Zeichnung lässt sich schreiben");
    assert!(!bytes.is_empty());
}

#[test]
fn rising_cable_says_its_span_of_heights() {
    let mut objects = Objects::default();
    add(
        &mut objects,
        input(ObjectKind::Cable, vec![[0.0, 0.0, 2.0], [2.0, 0.0, 0.5]]),
    );
    let d = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );
    let cable = entities(&d, "Kabel")[0];
    assert!(cable.xdata.contains(&"HOEHE=500..2000".to_string()));
}

#[test]
fn a_door_seen_from_one_side_cuts_both_faces() {
    let mut plan = plan_of(vec![
        face([0.0, 5.0], [6.0, 5.0], [0.0, -1.0]),
        face([6.0, 5.2], [0.0, 5.2], [0.0, 1.0]),
    ]);
    plan.pairs = vec![WallPair {
        front: 0,
        back: 1,
        thickness: 0.2,
    }];
    // only the first face sees the door; on the other a cupboard hides it.
    plan.openings = vec![opening(OpeningKind::Door, 0, 1.0, 2.0, 0.0, 2.05)];
    let d = draw(plan);
    let pieces = polylines(&d, "Wand");
    let on_y = |y: f64| -> Vec<(f64, f64)> {
        pieces
            .iter()
            .filter(|p| close(p[0][1], y))
            .map(|p| (p[0][0].min(p[1][0]), p[0][0].max(p[1][0])))
            .collect()
    };
    assert_eq!(on_y(5000.0), [(0.0, 1000.0), (2000.0, 6000.0)]);
    let mut other = on_y(5200.0);
    other.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(other, [(0.0, 1000.0), (2000.0, 6000.0)]);
}

#[test]
fn cable_and_pipe_have_text_and_xdata() {
    let mut objects = Objects::default();
    let mut cable = input(ObjectKind::Cable, vec![[0.0, 0.0, 2.0], [2.0, 0.0, 2.0]]);
    cable.note = "NYM".to_string();
    add(&mut objects, cable);
    add(
        &mut objects,
        input(ObjectKind::Measure, vec![[0.0; 3], [1.0, 0.0, 0.0]]),
    );
    let mut pipe = input(
        ObjectKind::Pipe,
        vec![[0.0, 1.0, 2.4], [3.0, 1.0, 2.4], [3.0, 5.0, 2.4]],
    );
    pipe.name = "Heizung".to_string();
    pipe.diameter_mm = Some(50.0);
    pipe.properties = BTreeMap::from([("Größe".to_string(), "DN 50".to_string())]);
    add(&mut objects, pipe);
    let mut thin = input(ObjectKind::Pipe, vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]);
    thin.diameter_mm = Some(12.5);
    add(&mut objects, thin);
    let d = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );

    assert_eq!(texts(&d, "Kabel"), ["Kabel 1"]);
    let cable = entities(&d, "Kabel")[0];
    assert!(matches!(
        &cable.kind,
        EntityKind::Shape(Shape::Polyline { points, closed: false, .. }) if points.len() == 2
    ));
    assert_eq!(
        cable.xdata,
        [
            "NR=1",
            "NAME=",
            "ART=Kabel",
            "HOEHE=2000",
            "NOTIZ=NYM",
            "QUELLE=hand"
        ]
    );

    assert_eq!(
        texts(&d, "Rohr"),
        ["Rohr 3 · Heizung · Ø 50 mm", "Rohr 4 · Ø 12,5 mm"]
    );
    let pipes = polylines(&d, "Rohr");
    assert_eq!(pipes[0].len(), 3);
    assert!(near(pipes[0][2], [3000.0, 5000.0]));
    let lines_with_data: Vec<_> = entities(&d, "Rohr")
        .into_iter()
        .filter(|e| !e.xdata.is_empty())
        .collect();
    assert_eq!(lines_with_data.len(), 2, "die Texte tragen keine Daten");
    assert_eq!(
        lines_with_data[0].xdata,
        [
            "NR=3",
            "NAME=Heizung",
            "ART=Rohr",
            "HOEHE=2400",
            "NOTIZ=",
            "QUELLE=hand",
            "GROESSE=DN 50",
            "DURCHMESSER_MM=50"
        ]
    );
    assert!(lines_with_data[1]
        .xdata
        .iter()
        .all(|x| !x.starts_with("GROESSE")));
    assert!(lines_with_data[1]
        .xdata
        .contains(&"DURCHMESSER_MM=12.5".to_string()));

    // The text lies at the longest stretch and runs along it.
    let Some(Shape::Text {
        at, rotation_deg, ..
    }) = shapes(&d, "Rohr")
        .into_iter()
        .find(|s| matches!(s, Shape::Text { .. }))
    else {
        panic!("Text fehlt");
    };
    assert!((rotation_deg - 90.0).abs() < 1e-6, "{rotation_deg}");
    assert!((at[1] - 3000.0).abs() < 2000.0, "{at:?}");
    assert!((at[0] - 3000.0).abs() < 400.0, "{at:?}");
}

#[test]
fn measurement_on_its_layer() {
    let mut objects = Objects::default();
    add(
        &mut objects,
        input(ObjectKind::Measure, vec![[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]),
    );
    let mut named = input(ObjectKind::Measure, vec![[0.0, 0.0, 0.0], [0.0, 0.0, 2.5]]);
    named.name = "Decke".to_string();
    add(&mut objects, named);
    let d = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );
    let list = lines(&d, "Messung");
    assert_eq!(list.len(), 2);
    assert!(near(list[0].0, [0.0, 0.0]) && near(list[0].1, [3000.0, 4000.0]));
    assert!(!list[0].2);
    assert_eq!(
        texts(&d, "Messung"),
        ["5,00 m", "Decke · 2,50 m"],
        "die Länge im Raum"
    );
    assert!(
        texts(&d, "Bemaßung").is_empty(),
        "nicht auf dem Layer der Maße"
    );
}

#[test]
fn object_far_outside_stays_where_it_is() {
    let mut objects = Objects::default();
    add(
        &mut objects,
        input(ObjectKind::Socket, vec![[500.0, -300.0, 1.0]]),
    );
    let mut plan = plan_of(vec![long_wall()]);
    plan.openings = vec![opening(OpeningKind::Door, 0, 1.0, 2.0, 0.0, 2.05)];
    let d = plan_drawing(&report(plan), &objects, &PlanExport::default());
    assert!(near(insert_at(&d, "Steckdose"), [500_000.0, -300_000.0]));
    assert_eq!(value(attributes(&d, "Steckdose"), "HOEHE"), "1000");
    assert_eq!(
        polylines(&d, "Wand").len(),
        2,
        "der Plan bleibt, wie er ist"
    );
    dxf_bytes(&d, 0.0).expect("weit draußen lässt sich schreiben");
}

#[test]
fn odd_characters_in_names_read_back() {
    let odd = format!("ä\nb;😀{}", "x".repeat(5000));
    let mut objects = Objects::default();
    let mut socket = input(ObjectKind::Socket, vec![[1.0, 2.0, 0.3]]);
    socket.name = odd.clone();
    socket.note = odd.clone();
    socket.properties = BTreeMap::from([("ä\nb".to_string(), odd.clone())]);
    add(&mut objects, socket);
    let mut pipe = input(ObjectKind::Pipe, vec![[0.0, 0.0, 2.0], [3.0, 0.0, 2.0]]);
    pipe.name = odd.clone();
    pipe.note = odd.clone();
    pipe.diameter_mm = Some(50.0);
    add(&mut objects, pipe);

    let drawing = plan_drawing(
        &report(plan_of(Vec::new())),
        &objects,
        &PlanExport::default(),
    );
    let dir = tmp("plan-drawing-odd");
    let path = dir.join("plan.dxf");
    write_dxf(&path, &drawing, 0.0, &Progress::new()).unwrap();
    let back = dxf::Drawing::load_file(&path).expect("der Leser verschluckt sich nicht");

    let found: Vec<_> = back.entities().collect();
    let insert = found
        .iter()
        .find_map(|e| match &e.specific {
            EntityType::Insert(i) => Some(i),
            _ => None,
        })
        .expect("INSERT");
    let name = insert
        .attributes()
        .find(|a| a.attribute_tag == "NAME")
        .unwrap();
    assert!(name.value.starts_with("ä b;?xxx"), "{}", &name.value[..20]);
    assert!(name.value.len() <= 2100);
    assert!(
        insert
            .attributes()
            .any(|a| a.attribute_tag == "AE_B" && a.value.starts_with("ä b;")),
        "die freie Eigenschaft ist da"
    );

    let (text, line) = (
        found.iter().find_map(|e| match &e.specific {
            EntityType::Text(t) if e.common.layer == "Rohr" => Some(t),
            _ => None,
        }),
        found
            .iter()
            .find(|e| matches!(e.specific, EntityType::LwPolyline(_)))
            .unwrap(),
    );
    let text = text.expect("Text");
    assert!(
        text.value.starts_with("Rohr 2 · ä b;?"),
        "{}",
        &text.value[..20]
    );
    assert!(
        text.value.ends_with("Ø 50 mm"),
        "der Durchmesser bleibt: {}",
        text.value
    );
    assert!(text.value.chars().count() < 80, "ein Plan-Text bleibt kurz");

    let items = &line.common.x_data[0].items;
    let XDataItem::Str(name) = &items[1] else {
        panic!("{:?}", items[1]);
    };
    assert!(name.starts_with("NAME=ä b;?"), "{name}");
    assert!(name.len() <= 300, "{}", name.len());
}

// ─── doors and windows set by hand ───────────────────────────────────────────────────────────

/// A door or window set by hand with its two edges in the plan frame of [`plan_of`].
fn hand(kind: ObjectKind, a: [f64; 3], b: [f64; 3]) -> ObjectInput {
    input(kind, vec![a, b])
}

fn hand_door(a: [f64; 3], b: [f64; 3]) -> ObjectInput {
    ObjectInput {
        height_m: Some(2.0),
        ..hand(ObjectKind::Door, a, b)
    }
}

fn hand_window(a: [f64; 3], b: [f64; 3]) -> ObjectInput {
    hand(ObjectKind::Window, a, b)
}

fn draw_with(plan: FloorPlan, objects: &Objects) -> Drawing {
    plan_drawing(&report(plan), objects, &PlanExport::default())
}

fn counts(plan: &FloorPlan, objects: &Objects) -> [usize; 3] {
    driftless_steps::plan_drawing::opening_counts(plan, objects)
}

/// The wall pieces lying at `y` (mm), from small to large x, as (from, to) x.
fn pieces_at(d: &Drawing, y: f64) -> Vec<(f64, f64)> {
    let mut pieces: Vec<(f64, f64)> = polylines(d, "Wand")
        .iter()
        .filter(|p| close(p[0][1], y) && close(p[1][1], y))
        .map(|p| (round(p[0][0].min(p[1][0])), round(p[0][0].max(p[1][0]))))
        .collect();
    pieces.sort_by(|a, b| a.0.total_cmp(&b.0));
    pieces
}

/// To a thousandth of a millimetre: floating point noise of the projection on another system.
fn round(mm: f64) -> f64 {
    (mm * 1000.0).round() / 1000.0
}

fn data<'a>(d: &'a Drawing, layer: &str) -> Vec<&'a Entity> {
    entities(d, layer)
        .into_iter()
        .filter(|e| !e.xdata.is_empty())
        .collect()
}

#[test]
fn a_door_set_by_hand_cuts_the_wall_like_a_found_one() {
    let plan = plan_of(vec![long_wall()]);
    let mut objects = Objects::default();
    // not exactly on the wall and with clicks at two heights: laid on the wall
    let mut door = hand_door([2.0, 0.10, 0.5], [2.9, 0.12, 1.3]);
    door.name = "Notausgang".to_string();
    door.note = "schließt selbst".to_string();
    add(&mut objects, door);
    let d = draw_with(plan.clone(), &objects);

    assert_eq!(pieces_at(&d, 0.0), [(0.0, 2000.0), (2900.0, 10_000.0)]);
    let caps = lines(&d, "Wand");
    assert_eq!(caps.len(), 2, "zwei Laibungen");
    assert!(caps
        .iter()
        .any(|(a, b, _)| near(*a, [2000.0, 0.0]) && close(length(*a, *b), 120.0)));

    let door_lines = lines(&d, "Tür");
    assert_eq!(door_lines.len(), 3, "zwei Laibungen, eine Schwelle");
    let dashed: Vec<_> = door_lines.iter().filter(|l| l.2).collect();
    assert_eq!(dashed.len(), 1, "die Schwelle ist gestrichelt");
    assert!(near(dashed[0].0, [2000.0, 0.0]) && near(dashed[0].1, [2900.0, 0.0]));
    assert_eq!(
        texts(&d, "Tür"),
        ["Tür 0,90 × 2,00 m · Notausgang"],
        "wie bei der Automatik, mit Name"
    );

    let with_data = data(&d, "Tür");
    assert_eq!(with_data.len(), 1, "die Schwelle trägt die Angaben");
    assert_eq!(
        with_data[0].xdata,
        [
            "NR=1",
            "NAME=Notausgang",
            "ART=Tür",
            "HOEHE=2000",
            "NOTIZ=schließt selbst",
            "QUELLE=hand"
        ]
    );
    assert_eq!(counts(&plan, &objects), [1, 0, 0]);
    dxf_bytes(&d, 0.0).expect("lässt sich schreiben");
}

#[test]
fn a_window_set_by_hand_says_what_is_known() {
    let plan = plan_of(vec![long_wall()]);
    let mut objects = Objects::default();
    add(
        &mut objects,
        hand_window([4.0, 0.05, 1.0], [5.2, 0.05, 1.0]),
    );
    let mut known = hand_window([6.0, 0.0, 1.0], [7.2, 0.0, 1.0]);
    known.height_m = Some(1.3);
    known.sill_m = Some(0.9);
    add(&mut objects, known);
    let d = draw_with(plan, &objects);

    assert_eq!(
        pieces_at(&d, 0.0),
        [(0.0, 4000.0), (5200.0, 6000.0), (7200.0, 10_000.0)]
    );
    let window = lines(&d, "Fenster");
    assert_eq!(window.len(), 4, "je Fenster die Doppellinie");
    assert!(window.iter().all(|l| !l.2), "durchgezogen");
    assert_eq!(
        texts(&d, "Fenster"),
        ["Fenster 1,20 m", "Fenster 1,20 × 1,30 m, BRH 0,90 m"]
    );
    assert_eq!(
        data(&d, "Fenster")[0].xdata[3],
        "HOEHE=",
        "ohne bekannte Höhe"
    );
    assert!(data(&d, "Fenster")
        .iter()
        .any(|e| e.xdata.iter().any(|x| x == "BRUESTUNG=900")));
}

/// The same door by hand in both placements of a frame turned by `turn` degrees.
fn door_in_both_placements(mut plan: FloorPlan, objects: Objects, check: impl Fn(&Drawing, bool)) {
    let (sin, cos) = 23f64.to_radians().sin_cos();
    plan.frame.axis = [cos, sin];
    plan.frame.origin = [12.5, -3.0, -1.3];
    let frame = plan.frame;
    // the objects are stored in the map: set them there
    let in_map = {
        let mut map = Objects::default();
        for o in objects.all() {
            let points = o.points.iter().map(|p| frame.to_map(*p)).collect();
            let mut input = hand(o.kind, [0.0; 3], [1.0, 0.0, 0.0]);
            input.points = points;
            input.height_m = o.height_m;
            input.sill_m = o.sill_m;
            input.name = o.name.clone();
            map.add(input).unwrap();
        }
        map
    };
    for (placement, as_cloud) in [
        (PlanPlacement::Walls, false),
        (PlanPlacement::AsCloud, true),
    ] {
        let export = PlanExport {
            placement,
            hidden: Vec::new(),
        };
        let d = plan_drawing(&report(plan.clone()), &in_map, &export);
        check(&d, as_cloud);
        dxf_bytes(&d, 0.0).expect("lässt sich schreiben");
    }
}

/// From the plan into the drawing of [`door_in_both_placements`].
fn in_drawing(p: [f64; 2], as_cloud: bool) -> [f64; 2] {
    if !as_cloud {
        return [p[0] * 1000.0, p[1] * 1000.0];
    }
    let (sin, cos) = 23f64.to_radians().sin_cos();
    [
        (12.5 + cos * p[0] - sin * p[1]) * 1000.0,
        (-3.0 + sin * p[0] + cos * p[1]) * 1000.0,
    ]
}

#[test]
fn a_door_by_hand_cuts_both_faces_and_is_labelled_once_in_both_placements() {
    let mut plan = plan_of(vec![
        face([0.0, 5.0], [6.0, 5.0], [0.0, -1.0]),
        face([6.0, 5.2], [0.0, 5.2], [0.0, 1.0]),
    ]);
    plan.pairs = vec![WallPair {
        front: 0,
        back: 1,
        thickness: 0.2,
    }];
    let mut objects = Objects::default();
    // the two edges in the frame of the plan, nearer the face at y = 5.0
    add(&mut objects, hand_door([1.0, 5.05, 1.0], [1.9, 5.05, 1.0]));

    door_in_both_placements(plan, objects, |d, as_cloud| {
        let piece = |a: [f64; 2], b: [f64; 2]| {
            let (a, b) = (in_drawing(a, as_cloud), in_drawing(b, as_cloud));
            polylines(d, "Wand")
                .iter()
                .any(|p| (near(p[0], a) && near(p[1], b)) || (near(p[0], b) && near(p[1], a)))
        };
        assert!(piece([0.0, 5.0], [1.0, 5.0]), "{:?}", polylines(d, "Wand"));
        assert!(piece([1.9, 5.0], [6.0, 5.0]), "{:?}", polylines(d, "Wand"));
        assert!(piece([0.0, 5.2], [1.0, 5.2]), "die andere Fläche auch");
        assert!(piece([1.9, 5.2], [6.0, 5.2]), "die andere Fläche auch");
        assert_eq!(polylines(d, "Wand").len(), 4);
        assert_eq!(texts(d, "Tür"), ["Tür 0,90 × 2,00 m"], "nur einmal");
        assert_eq!(lines(d, "Tür").iter().filter(|l| l.2).count(), 1);
    });
}

#[test]
fn the_hand_wins_over_an_opening_found() {
    let mut plan = plan_of(vec![
        face([0.0, 5.0], [8.0, 5.0], [0.0, -1.0]),
        face([8.0, 5.2], [0.0, 5.2], [0.0, 1.0]),
    ]);
    plan.pairs = vec![WallPair {
        front: 0,
        back: 1,
        thickness: 0.2,
    }];
    let mut found_door = opening(OpeningKind::Door, 0, 2.0, 3.0, 0.0, 2.05);
    found_door.note = "gefunden".to_string();
    // the second entry of the same door lies on the other face, measured from its own end
    let found_other_side = opening(OpeningKind::Door, 1, 5.0, 6.0, 0.0, 2.05);
    let unclear = opening(OpeningKind::Unclear, 0, 6.0, 7.0, 0.0, 2.0);
    let far_door = opening(OpeningKind::Door, 0, 3.5, 4.5, 0.0, 2.05);
    plan.openings = vec![found_door, found_other_side, unclear, far_door];
    let mut objects = Objects::default();
    // a window by hand over the found door, a door by hand over the unclear one
    add(
        &mut objects,
        hand_window([2.2, 5.05, 1.0], [3.1, 5.05, 1.0]),
    );
    add(&mut objects, hand_door([5.9, 5.1, 1.0], [7.1, 5.1, 1.0]));
    let d = draw_with(plan.clone(), &objects);

    assert_eq!(texts(&d, "Tür"), ["Tür 1,00 × 2,05 m", "Tür 1,20 × 2,00 m"]);
    assert_eq!(
        texts(&d, "Fenster"),
        ["Fenster 0,90 m"],
        "die von Hand gewinnt"
    );
    assert!(texts(&d, "Öffnung-unklar").is_empty(), "unklar ist ersetzt");
    assert_eq!(
        pieces_at(&d, 5000.0),
        [
            (0.0, 2200.0),
            (3100.0, 3500.0),
            (4500.0, 5900.0),
            (7100.0, 8000.0)
        ],
        "geschnitten wie die von Hand, nicht wie die gefundene"
    );
    assert_eq!(
        pieces_at(&d, 5200.0),
        [
            (0.0, 2200.0),
            (3100.0, 3500.0),
            (4500.0, 5900.0),
            (7100.0, 8000.0)
        ],
        "beide Flächen"
    );
    assert_eq!(counts(&plan, &objects), [2, 1, 0]);
}

#[test]
fn a_centimetre_of_overlap_leaves_the_found_opening() {
    let mut plan = plan_of(vec![long_wall()]);
    plan.openings = vec![opening(OpeningKind::Door, 0, 2.0, 3.0, 0.0, 2.05)];
    let mut objects = Objects::default();
    add(&mut objects, hand_door([2.99, 0.0, 1.0], [3.9, 0.0, 1.0]));
    let d = draw_with(plan, &objects);
    assert_eq!(texts(&d, "Tür"), ["Tür 1,00 × 2,05 m", "Tür 0,91 × 2,00 m"]);
}

#[test]
fn an_opening_far_from_a_wall_stays_as_a_line_with_its_text() {
    let plan = plan_of(vec![long_wall()]);
    let mut objects = Objects::default();
    let (a, b) = ([2.0, 3.0, 1.0], [2.9, 3.0, 1.0]);
    add(&mut objects, hand_door(a, b));
    let d = draw_with(plan.clone(), &objects);
    assert_eq!(pieces_at(&d, 0.0), [(0.0, 10_000.0)], "die Wand bleibt");
    let drawn = polylines(&d, "Tür");
    assert_eq!(drawn.len(), 1);
    assert!(near(drawn[0][0], [2000.0, 3000.0]) && near(drawn[0][1], [2900.0, 3000.0]));
    assert!(lines(&d, "Tür").is_empty(), "kein Türsymbol ohne Wand");
    assert_eq!(texts(&d, "Tür"), ["Tür 0,90 × 2,00 m"]);
    assert_eq!(
        data(&d, "Tür")[0].xdata[0],
        "NR=1",
        "die Angaben stehen dran"
    );
    assert_eq!(counts(&plan, &objects), [1, 0, 0]);
    dxf_bytes(&d, 0.0).expect("lässt sich schreiben");
}

#[test]
fn without_walls_an_opening_is_a_line_at_its_place_in_both_placements() {
    let mut objects = Objects::default();
    add(&mut objects, hand_window([1.0, 1.0, 1.0], [2.2, 1.0, 1.0]));

    door_in_both_placements(plan_of(Vec::new()), objects, |d, as_cloud| {
        let drawn = polylines(d, "Fenster");
        assert_eq!(drawn.len(), 1);
        let (a, b) = (
            in_drawing([1.0, 1.0], as_cloud),
            in_drawing([2.2, 1.0], as_cloud),
        );
        assert!(near(drawn[0][0], a) && near(drawn[0][1], b), "{drawn:?}");
        assert_eq!(texts(d, "Fenster"), ["Fenster 1,20 m"]);
    });
}

#[test]
fn an_opening_across_a_wall_or_past_its_end_or_on_another_storey_is_no_cut() {
    let plan = plan_of(vec![long_wall()]);
    for (name, a, b) in [
        ("quer zur Wand", [3.0, 0.0, 1.0], [3.05, 0.30, 1.0]),
        // the wall is 10 m long
        ("hinter dem Ende", [10.2, 0.1, 1.0], [11.0, 0.1, 1.0]),
        // 2.6 m to the ceiling in `plan_of`
        ("ein Stockwerk höher", [3.0, 0.1, 4.0], [3.9, 0.1, 4.0]),
    ] {
        let mut objects = Objects::default();
        add(&mut objects, hand_door(a, b));
        let d = draw_with(plan.clone(), &objects);
        assert_eq!(pieces_at(&d, 0.0), [(0.0, 10_000.0)], "{name}");
        assert_eq!(polylines(&d, "Tür").len(), 1, "{name}");
    }
}

#[test]
fn a_door_at_the_end_of_a_wall_keeps_its_width() {
    let plan = plan_of(vec![long_wall()]);
    let mut objects = Objects::default();
    // the last 40 cm of the door are past the end of the 10 m wall
    add(&mut objects, hand_door([9.5, 0.0, 1.0], [10.4, 0.0, 1.0]));
    let d = draw_with(plan, &objects);
    assert_eq!(pieces_at(&d, 0.0), [(0.0, 9500.0)]);
    assert_eq!(texts(&d, "Tür"), ["Tür 0,90 × 2,00 m"]);
}

#[test]
fn doors_by_hand_read_back_from_the_dxf() {
    let plan = plan_of(vec![
        long_wall(),
        face([0.0, 5.0], [6.0, 5.0], [0.0, -1.0]),
        face([6.0, 5.2], [0.0, 5.2], [0.0, 1.0]),
    ]);
    let mut plan = plan;
    plan.pairs = vec![WallPair {
        front: 1,
        back: 2,
        thickness: 0.2,
    }];
    let mut objects = Objects::default();
    add(&mut objects, hand_door([2.0, 0.0, 1.0], [2.9, 0.0, 1.0]));
    add(&mut objects, hand_window([1.0, 5.1, 1.0], [2.2, 5.1, 1.0]));
    add(
        &mut objects,
        hand_door([20.0, 20.0, 1.0], [20.9, 20.0, 1.0]),
    );
    let drawing = plan_drawing(&report(plan), &objects, &PlanExport::default());

    let dir = tmp("plan-drawing-tueren");
    let path = dir.join("plan.dxf");
    write_dxf(&path, &drawing, 0.0, &Progress::new()).unwrap();
    let back = dxf::Drawing::load_file(&path).expect("der Leser verschluckt sich nicht");
    let found: Vec<_> = back.entities().collect();
    let of_layer = |layer: &str| -> Vec<&&dxf::entities::Entity> {
        found.iter().filter(|e| e.common.layer == layer).collect()
    };
    let doors = of_layer("Tür");
    assert!(doors
        .iter()
        .any(|e| matches!(e.specific, EntityType::Line(_))));
    let text_of = |e: &&&dxf::entities::Entity| match &e.specific {
        EntityType::Text(t) => Some(t.value.clone()),
        _ => None,
    };
    let door_texts: Vec<String> = doors.iter().filter_map(text_of).collect();
    assert_eq!(door_texts.len(), 2, "{door_texts:?}");
    assert!(
        door_texts.iter().all(|t| t == "Tür 0,90 × 2,00 m"),
        "{door_texts:?}"
    );
    let windows = of_layer("Fenster");
    assert_eq!(windows.iter().filter_map(text_of).count(), 1);
    assert_eq!(
        of_layer("Wand")
            .iter()
            .filter(|e| matches!(e.specific, EntityType::LwPolyline(_)))
            .count(),
        6
    );
    assert!(doors.iter().any(|e| e.common.x_data.iter().any(|x| x
        .items
        .iter()
        .any(|i| matches!(i, XDataItem::Str(s) if s == "ART=Tür")))));
}
