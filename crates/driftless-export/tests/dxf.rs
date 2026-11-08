//! The DXF writer, checked from the outside: read back with the `dxf` crate, and pair by pair
//! on the raw bytes where the reader would hide a mistake.

use driftless_export::dxf::{
    dxf_bytes, write_dxf, Block, Drawing, Entity, EntityKind, Layer, Shape,
};
use driftless_export::ExportError;
use driftless_io::progress::Progress;
use driftless_io::testdir::TestDir;
use dxf::entities::EntityType;
use dxf::enums::{AcadVersion, Units};
use dxf::XDataItem;
use std::collections::BTreeSet;
use std::path::Path;

fn temp_dir(name: &str) -> TestDir {
    TestDir::new(format!(
        "driftless-export-dxf-{}-{name}",
        std::process::id()
    ))
}

fn layer(name: &str, color: u8) -> Layer {
    Layer {
        name: name.to_string(),
        color,
        on: true,
    }
}

fn entity(layer: &str, shape: Shape) -> Entity {
    Entity {
        layer: layer.to_string(),
        kind: EntityKind::Shape(shape),
        xdata: Vec::new(),
    }
}

fn text(value: &str) -> Shape {
    Shape::Text {
        at: [1000.0, 2000.0],
        height: 100.0,
        rotation_deg: 0.0,
        value: value.to_string(),
    }
}

fn drawing_of(entities: Vec<Entity>) -> Drawing {
    Drawing {
        layers: vec![layer("Wand", 7), layer("Tür", 30)],
        blocks: Vec::new(),
        entities,
        comments: Vec::new(),
    }
}

/// A drawing with every kind of entity.
fn full_drawing() -> Drawing {
    let mut off = layer("Bemaßung", 2);
    off.on = false;
    Drawing {
        layers: vec![
            layer("Wand", 7),
            layer("Tür", 30),
            layer("Öffnung-unklar", 8),
            off,
        ],
        blocks: vec![Block {
            name: "Steckdose".to_string(),
            shapes: vec![
                Shape::Circle {
                    centre: [0.0, 0.0],
                    radius: 75.0,
                },
                Shape::Line {
                    a: [-75.0, 0.0],
                    b: [75.0, 0.0],
                    dashed: false,
                },
            ],
            attributes: vec!["NR".to_string(), "NAME".to_string(), "NOTIZ".to_string()],
        }],
        entities: vec![
            entity(
                "Wand",
                Shape::Line {
                    a: [0.0, 0.0],
                    b: [4000.0, 0.0],
                    dashed: false,
                },
            ),
            entity(
                "Tür",
                Shape::Line {
                    a: [1000.0, 0.0],
                    b: [1900.0, 0.0],
                    dashed: true,
                },
            ),
            entity(
                "Wand",
                Shape::Polyline {
                    points: vec![[0.0, 0.0], [4000.0, 0.0], [4000.0, 3000.0]],
                    closed: true,
                    dashed: false,
                },
            ),
            entity(
                "Wand",
                Shape::Circle {
                    centre: [500.0, 600.0],
                    radius: 50.0,
                },
            ),
            entity(
                "Bemaßung",
                Shape::Text {
                    at: [100.0, 200.0],
                    height: 120.0,
                    rotation_deg: 90.0,
                    value: "8,30 m".to_string(),
                },
            ),
            Entity {
                layer: "Wand".to_string(),
                kind: EntityKind::Insert {
                    block: "Steckdose".to_string(),
                    at: [2000.0, 1000.0],
                    attributes: vec![
                        ("NR".to_string(), "7".to_string()),
                        ("NAME".to_string(), "Küche".to_string()),
                    ],
                },
                xdata: vec!["NR=7".to_string()],
            },
            Entity {
                layer: "Wand".to_string(),
                kind: EntityKind::Shape(Shape::Polyline {
                    points: vec![[0.0, 0.0], [100.0, 0.0]],
                    closed: false,
                    dashed: false,
                }),
                xdata: vec!["NR=3".to_string(), "NOTIZ=Kabel".to_string()],
            },
        ],
        comments: vec!["Driftless, Lage: an den Wänden".to_string()],
    }
}

/// Reads through a file, as the brief says, with the independent reader.
fn read_back(bytes: &[u8], dir: &Path) -> dxf::Drawing {
    let path = dir.join("read.dxf");
    std::fs::write(&path, bytes).unwrap();
    dxf::Drawing::load_file(&path).unwrap()
}

/// Group code and value of every pair; values as Latin-1 so that the raw bytes stay visible.
fn pairs(bytes: &[u8]) -> Vec<(i32, String)> {
    assert_eq!(bytes.last(), Some(&b'\n'), "file ends with a newline");
    let lines: Vec<&[u8]> = bytes[..bytes.len() - 1].split(|&b| b == b'\n').collect();
    assert_eq!(lines.len() % 2, 0, "an even number of lines");
    lines
        .chunks(2)
        .map(|pair| {
            let code = std::str::from_utf8(pair[0]).unwrap().trim();
            let code: i32 = code.parse().unwrap_or_else(|_| panic!("code: {code:?}"));
            assert!(
                !pair[1].contains(&b'\r'),
                "a carriage return inside a value"
            );
            (code, pair[1].iter().map(|&b| b as char).collect())
        })
        .collect()
}

/// The value after header variable `name`.
fn variable(pairs: &[(i32, String)], name: &str) -> Vec<(i32, String)> {
    let at = pairs
        .iter()
        .position(|p| p.0 == 9 && p.1 == name)
        .unwrap_or_else(|| panic!("{name} is missing"));
    pairs[at + 1..]
        .iter()
        .take_while(|p| p.0 != 9 && p.0 != 0)
        .cloned()
        .collect()
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.windows(needle.len()).any(|w| w == needle)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

#[test]
fn header_says_r2000_and_millimetres() {
    let dir = temp_dir("header");
    let bytes = dxf_bytes(&full_drawing(), 0.0).unwrap();
    let drawing = read_back(&bytes, dir.path());
    assert_eq!(drawing.header.version, AcadVersion::R2000);
    assert_eq!(drawing.header.default_drawing_units, Units::Millimeters);

    let all = pairs(&bytes);
    assert_eq!(
        variable(&all, "$DWGCODEPAGE"),
        vec![(3, "ANSI_1252".to_string())]
    );
    assert_eq!(variable(&all, "$ACADVER"), vec![(1, "AC1015".to_string())]);
    assert_eq!(variable(&all, "$INSUNITS"), vec![(70, "4".to_string())]);
    assert_eq!(variable(&all, "$MEASUREMENT"), vec![(70, "1".to_string())]);
    for name in [
        "$INSBASE",
        "$EXTMIN",
        "$EXTMAX",
        "$LIMMIN",
        "$LIMMAX",
        "$LTSCALE",
        "$CLAYER",
        "$TEXTSTYLE",
        "$DIMSTYLE",
        "$LUNITS",
        "$AUNITS",
        "$EXTNAMES",
        "$LWDISPLAY",
        "$HANDSEED",
    ] {
        assert!(!variable(&all, name).is_empty(), "{name} without a value");
    }
}

#[test]
fn layers_read_back() {
    let dir = temp_dir("layers");
    let bytes = dxf_bytes(&full_drawing(), 0.0).unwrap();
    let drawing = read_back(&bytes, dir.path());
    let find = |name: &str| {
        drawing
            .layers()
            .find(|l| l.name == name)
            .unwrap_or_else(|| panic!("layer {name} is missing"))
    };
    assert_eq!(find("0").color.index(), Some(7));
    assert!(find("Tür").is_layer_on);
    assert_eq!(find("Tür").color.index(), Some(30));
    assert!(find("Öffnung-unklar").is_layer_on);
    assert!(!find("Bemaßung").is_layer_on, "the hidden layer is off");
    assert_eq!(find("Bemaßung").color.index(), Some(2));
    assert_eq!(drawing.layers().count(), 5);
}

#[test]
fn entities_read_back() {
    let dir = temp_dir("entities");
    let z = 1234.5;
    let bytes = dxf_bytes(&full_drawing(), z).unwrap();
    let drawing = read_back(&bytes, dir.path());
    let entities: Vec<_> = drawing.entities().collect();
    assert_eq!(entities.len(), 7);

    let EntityType::Line(line) = &entities[0].specific else {
        panic!("line expected");
    };
    assert_eq!(entities[0].common.layer, "Wand");
    assert!(near(line.p2.x, 4000.0) && near(line.p1.z, z) && near(line.p2.z, z));
    assert_ne!(entities[0].common.line_type_name, "GESTRICHELT");

    assert_eq!(entities[1].common.layer, "Tür");
    assert_eq!(entities[1].common.line_type_name, "GESTRICHELT");

    let EntityType::LwPolyline(poly) = &entities[2].specific else {
        panic!("polyline expected");
    };
    assert!(poly.is_closed());
    assert_eq!(poly.vertices.len(), 3);
    assert!(near(poly.vertices[2].y, 3000.0));
    assert!(near(entities[2].common.elevation, z));

    let EntityType::Circle(circle) = &entities[3].specific else {
        panic!("circle expected");
    };
    assert!(near(circle.center.x, 500.0) && near(circle.center.z, z));
    assert!(near(circle.radius, 50.0));

    let EntityType::Text(label) = &entities[4].specific else {
        panic!("text expected");
    };
    assert_eq!(label.value, "8,30 m");
    assert!(near(label.location.y, 200.0) && near(label.location.z, z));
    assert!(near(label.text_height, 120.0) && near(label.rotation, 90.0));
    assert_eq!(entities[4].common.layer, "Bemaßung");

    let EntityType::Insert(insert) = &entities[5].specific else {
        panic!("insert expected");
    };
    assert_eq!(insert.name, "Steckdose");
    assert!(near(insert.location.x, 2000.0) && near(insert.location.z, z));
    let attributes: Vec<_> = insert
        .attributes()
        .map(|a| (a.attribute_tag.as_str(), a.value.as_str()))
        .collect();
    assert_eq!(
        attributes,
        vec![("NR", "7"), ("NAME", "Küche"), ("NOTIZ", "")],
        "every ATTDEF has its ATTRIB, in the order of the block"
    );
    assert!(insert.attributes().all(|a| near(a.location.z, z)));
    let x = &entities[5].common.x_data;
    assert_eq!(x.len(), 1);
    assert_eq!(x[0].application_name, "DRIFTLESS");
    assert_eq!(x[0].items, vec![XDataItem::Str("NR=7".to_string())]);

    let x = &entities[6].common.x_data;
    assert_eq!(x[0].application_name, "DRIFTLESS");
    assert_eq!(
        x[0].items,
        vec![
            XDataItem::Str("NR=3".to_string()),
            XDataItem::Str("NOTIZ=Kabel".to_string())
        ]
    );

    let block = drawing
        .blocks()
        .find(|b| b.name == "Steckdose")
        .expect("block");
    let tags: Vec<_> = block
        .entities
        .iter()
        .filter_map(|e| match &e.specific {
            EntityType::AttributeDefinition(a) => Some(a.text_tag.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(tags, vec!["NR", "NAME", "NOTIZ"]);
    assert_eq!(block.entities.len(), 5, "circle, line, three ATTDEF");
    assert!(block.entities.iter().all(|e| e.common.layer == "0"));
}

#[test]
fn insert_without_attributes_has_no_sequence() {
    let mut drawing = drawing_of(vec![Entity {
        layer: "Wand".to_string(),
        kind: EntityKind::Insert {
            block: "Marke".to_string(),
            at: [0.0, 0.0],
            attributes: Vec::new(),
        },
        xdata: Vec::new(),
    }]);
    drawing.blocks.push(Block {
        name: "Marke".to_string(),
        shapes: vec![Shape::Circle {
            centre: [0.0, 0.0],
            radius: 10.0,
        }],
        attributes: Vec::new(),
    });
    let all = pairs(&dxf_bytes(&drawing, 0.0).unwrap());
    assert!(!all.iter().any(|p| p.0 == 0 && p.1 == "SEQEND"));
    assert!(!all.iter().any(|p| p.0 == 66));
}

#[test]
fn handles_are_unique_and_seed_is_higher() {
    let bytes = dxf_bytes(&full_drawing(), 0.0).unwrap();
    let all = pairs(&bytes);
    let seed_at = all
        .iter()
        .position(|p| p.0 == 9 && p.1 == "$HANDSEED")
        .unwrap();
    let seed = u64::from_str_radix(&all[seed_at + 1].1, 16).unwrap();

    let mut handles = BTreeSet::new();
    for (i, (code, value)) in all.iter().enumerate() {
        if (*code == 5 || *code == 105) && i != seed_at + 1 {
            let handle = u64::from_str_radix(value, 16).unwrap();
            assert!(handles.insert(handle), "handle {value} twice");
            assert!(handle > 0 && handle < seed, "handle {value}, seed {seed:X}");
        }
    }
    assert!(handles.len() > 40);

    // Every reference leads somewhere: the owners, the entries of the dictionaries, the
    // layouts of the block records, the plot style of the layers.
    for (code, value) in &all {
        if matches!(*code, 330 | 340 | 350 | 390) {
            let target = u64::from_str_radix(value, 16).unwrap();
            assert!(
                target == 0 || handles.contains(&target),
                "group {code} points to {value}, which does not exist"
            );
        }
    }

    // Each object in OBJECTS starts with handle and owner.
    let start = all
        .iter()
        .position(|p| p.0 == 2 && p.1 == "OBJECTS")
        .unwrap();
    for (i, pair) in all.iter().enumerate().skip(start + 1) {
        if pair.0 == 0 && pair.1 != "ENDSEC" && pair.1 != "EOF" {
            assert_eq!(all[i + 1].0, 5, "{} without a handle", pair.1);
            assert!(
                all[i + 2..i + 8].iter().any(|p| p.0 == 330),
                "{} without an owner",
                pair.1
            );
        }
    }
}

#[test]
fn sections_in_order() {
    let all = pairs(&dxf_bytes(&full_drawing(), 0.0).unwrap());
    let mut sections = Vec::new();
    let mut open = false;
    for (i, (code, value)) in all.iter().enumerate() {
        if *code != 0 {
            continue;
        }
        match value.as_str() {
            "SECTION" => {
                assert!(!open, "section inside a section");
                open = true;
                assert_eq!(all[i + 1].0, 2);
                sections.push(all[i + 1].1.clone());
            }
            "ENDSEC" => {
                assert!(open, "ENDSEC without a section");
                open = false;
            }
            _ => {}
        }
    }
    assert!(!open);
    assert_eq!(
        sections,
        ["HEADER", "CLASSES", "TABLES", "BLOCKS", "ENTITIES", "OBJECTS"]
    );
    assert_eq!(all.last().unwrap(), &(0, "EOF".to_string()));

    let tables: Vec<&str> = all
        .windows(2)
        .filter(|w| w[0] == (0, "TABLE".to_string()))
        .map(|w| w[1].1.as_str())
        .collect();
    assert_eq!(
        tables,
        [
            "VPORT",
            "LTYPE",
            "LAYER",
            "STYLE",
            "VIEW",
            "UCS",
            "APPID",
            "DIMSTYLE",
            "BLOCK_RECORD"
        ]
    );

    let has = |code: i32, value: &str| all.iter().any(|p| p.0 == code && p.1 == value);
    for class in ["LAYOUT", "ACDBDICTIONARYWDFLT", "ACDBPLACEHOLDER"] {
        assert!(has(1, class), "class {class}");
    }
    for name in [
        "ByBlock",
        "ByLayer",
        "Continuous",
        "GESTRICHELT",
        "*Active",
        "ACAD",
        "DRIFTLESS",
        "*Model_Space",
        "*Paper_Space",
    ] {
        assert!(has(2, name), "entry {name}");
    }
    assert!(has(3, "arial.ttf"));
    for name in [
        "ACAD_GROUP",
        "ACAD_LAYOUT",
        "Model",
        "Layout1",
        "ACAD_MLINESTYLE",
        "Standard",
        "ACAD_PLOTSETTINGS",
        "ACAD_PLOTSTYLENAME",
        "Normal",
    ] {
        assert!(has(3, name), "dictionary entry {name}");
    }
}

#[test]
fn umlauts_are_windows_1252() {
    let drawing = drawing_of(vec![entity("Tür", text("ü€Ω😀"))]);
    let bytes = dxf_bytes(&drawing, 0.0).unwrap();
    assert!(contains(&bytes, b"T\xFCr\n"), "layer name");
    assert!(contains(&bytes, b"\n\xFC\x80\\U+03A9?\n"), "text value");

    // Every sign of the table once, and what the table leaves out.
    let drawing = drawing_of(vec![entity(
        "Wand",
        text("ÄÖÜäöüß²×·Ø‚ƒ„…†‡ˆ‰Š‹ŒŽ‘’“”•–—˜™š›œžŸ"),
    )]);
    let bytes = dxf_bytes(&drawing, 0.0).unwrap();
    assert!(contains(
        &bytes,
        b"\n\xC4\xD6\xDC\xE4\xF6\xFC\xDF\xB2\xD7\xB7\xD8\x82\x83\x84\x85\x86\x87\x88\x89\x8A\x8B\x8C\x8E\x91\x92\x93\x94\x95\x96\x97\x98\x99\x9A\x9B\x9C\x9E\x9F\n"
    ));
}

#[test]
fn newlines_in_values_do_not_break_the_file() {
    let odd = "a\nb\r\nc;d\u{2028}e\tf\u{85}g";
    let mut drawing = full_drawing();
    drawing.entities.push(Entity {
        layer: "Wand".to_string(),
        kind: EntityKind::Shape(text(odd)),
        xdata: vec![odd.to_string()],
    });
    drawing.entities.push(Entity {
        layer: "Wand".to_string(),
        kind: EntityKind::Insert {
            block: "Steckdose".to_string(),
            at: [0.0, 0.0],
            attributes: vec![("NOTIZ".to_string(), odd.to_string())],
        },
        xdata: Vec::new(),
    });
    drawing.comments.push(odd.to_string());

    let dir = temp_dir("newlines");
    let bytes = dxf_bytes(&drawing, 0.0).unwrap();
    let all = pairs(&bytes);
    assert_eq!(all.last().unwrap(), &(0, "EOF".to_string()));

    let expected = "a b  c;d e f g";
    let back = read_back(&bytes, dir.path());
    let entities: Vec<_> = back.entities().collect();
    let EntityType::Text(label) = &entities[7].specific else {
        panic!("text expected");
    };
    assert_eq!(label.value, expected);
    assert_eq!(
        entities[7].common.x_data[0].items,
        vec![XDataItem::Str(expected.to_string())]
    );
    let EntityType::Insert(insert) = &entities[8].specific else {
        panic!("insert expected");
    };
    let notiz = insert.attributes().find(|a| a.attribute_tag == "NOTIZ");
    assert_eq!(notiz.unwrap().value, expected);
    assert!(all.iter().any(|p| p.0 == 999 && p.1 == expected));
}

#[test]
fn very_long_values_are_cut_in_xdata() {
    let mut drawing = drawing_of(Vec::new());
    drawing.entities.push(Entity {
        layer: "Wand".to_string(),
        kind: EntityKind::Shape(text("x")),
        xdata: vec!["a".repeat(1000), "Ω".repeat(1000), "b".to_string()],
    });
    let all = pairs(&dxf_bytes(&drawing, 0.0).unwrap());
    let values: Vec<&String> = all.iter().filter(|p| p.0 == 1000).map(|p| &p.1).collect();
    assert_eq!(values.len(), 3);
    assert_eq!(values[0].len(), 255);
    assert_eq!(values[1], &"\\U+03A9".repeat(36), "no cut inside an escape");
    assert_eq!(values[2], "b");

    // Text outside XDATA stays within the line length of the format.
    let drawing = drawing_of(vec![entity("Wand", text(&"c".repeat(100_000)))]);
    let bytes = dxf_bytes(&drawing, 0.0).unwrap();
    let longest = pairs(&bytes).iter().map(|p| p.1.len()).max().unwrap();
    assert_eq!(longest, 2049);
}

fn refused(mut drawing: Drawing, change: impl FnOnce(&mut Drawing)) -> String {
    change(&mut drawing);
    dxf_bytes(&drawing, 0.0).unwrap_err()
}

#[test]
fn bad_layer_name_is_refused() {
    for bad in [
        "a<b", "a>b", "a/b", "a\\b", "a\"b", "a:b", "a;b", "a?b", "a*b", "a|b", "a=b", "a`b",
        "a\nb", "a\tb", "", "a😀",
    ] {
        let message = refused(full_drawing(), |d| d.layers[0].name = bad.to_string());
        assert!(message.contains("Layer"), "{bad:?}: {message}");
        let message = refused(full_drawing(), |d| d.blocks[0].name = bad.to_string());
        assert!(message.contains("Block"), "{bad:?}: {message}");
    }

    let dir = temp_dir("bad-name");
    let path = dir.path().join("plan.dxf");
    let mut drawing = full_drawing();
    drawing.layers[0].name = "a/b".to_string();
    let error = write_dxf(&path, &drawing, 0.0, &Progress::new()).unwrap_err();
    assert!(matches!(error, ExportError::Format { .. }), "{error}");
    assert!(!path.exists());
}

#[test]
fn what_cannot_be_drawn_is_refused() {
    refused(full_drawing(), |d| d.layers[1].name = "wand".to_string());
    refused(full_drawing(), |d| d.layers[1].name = "0".to_string());
    refused(full_drawing(), |d| d.layers[1].color = 0);
    refused(full_drawing(), |d| {
        let copy = d.blocks[0].clone();
        d.blocks.push(copy);
    });
    refused(full_drawing(), |d| {
        d.entities[0].layer = "Fehlt".to_string()
    });
    refused(full_drawing(), |d| {
        d.blocks[0].attributes[0] = "A B".to_string()
    });
    refused(full_drawing(), |d| {
        d.blocks[0].attributes[1] = "NR".to_string();
    });
    for kind in [
        EntityKind::Insert {
            block: "Fehlt".to_string(),
            at: [0.0, 0.0],
            attributes: Vec::new(),
        },
        EntityKind::Insert {
            block: "Steckdose".to_string(),
            at: [0.0, 0.0],
            attributes: vec![("UNBEKANNT".to_string(), "1".to_string())],
        },
        EntityKind::Insert {
            block: "Steckdose".to_string(),
            at: [f64::NAN, 0.0],
            attributes: Vec::new(),
        },
        EntityKind::Shape(Shape::Circle {
            centre: [0.0, 0.0],
            radius: 0.0,
        }),
        EntityKind::Shape(Shape::Polyline {
            points: vec![[0.0, 0.0]],
            closed: false,
            dashed: false,
        }),
        EntityKind::Shape(Shape::Text {
            at: [0.0, 0.0],
            height: 0.0,
            rotation_deg: 0.0,
            value: "x".to_string(),
        }),
        EntityKind::Shape(Shape::Line {
            a: [f64::INFINITY, 0.0],
            b: [0.0, 0.0],
            dashed: false,
        }),
    ] {
        let message = refused(full_drawing(), |d| d.entities[0].kind = kind.clone());
        assert!(!message.is_empty(), "{kind:?}");
    }
    assert!(dxf_bytes(&full_drawing(), f64::NAN).is_err());
}

#[test]
fn extents_in_header() {
    let dir = temp_dir("extents");
    let mut drawing = drawing_of(vec![
        entity(
            "Wand",
            Shape::Line {
                a: [-100.0, 200.0],
                b: [300.0, 400.0],
                dashed: false,
            },
        ),
        entity(
            "Wand",
            Shape::Circle {
                centre: [1000.0, 0.0],
                radius: 50.0,
            },
        ),
        Entity {
            layer: "Wand".to_string(),
            kind: EntityKind::Insert {
                block: "Marke".to_string(),
                at: [0.0, 5000.0],
                attributes: Vec::new(),
            },
            xdata: Vec::new(),
        },
    ]);
    drawing.blocks.push(Block {
        name: "Marke".to_string(),
        shapes: vec![Shape::Circle {
            centre: [0.0, 0.0],
            radius: 75.0,
        }],
        attributes: Vec::new(),
    });
    let bytes = dxf_bytes(&drawing, 12.5).unwrap();
    let header = read_back(&bytes, dir.path()).header;
    let (min, max) = (
        header.minimum_drawing_extents,
        header.maximum_drawing_extents,
    );
    assert!(near(min.x, -100.0) && near(min.y, -50.0) && near(min.z, 12.5));
    assert!(near(max.x, 1050.0) && near(max.y, 5075.0) && near(max.z, 12.5));
    let (min, max) = (header.minimum_drawing_limits, header.maximum_drawing_limits);
    assert!(near(min.x, -100.0) && near(min.y, -50.0));
    assert!(near(max.x, 1050.0) && near(max.y, 5075.0));

    // An empty drawing still is a valid file.
    let bytes = dxf_bytes(&drawing_of(Vec::new()), 0.0).unwrap();
    let back = read_back(&bytes, dir.path());
    assert_eq!(back.entities().count(), 0);
}

#[test]
fn line_is_written_as_expected() {
    let drawing = drawing_of(vec![entity(
        "Tür",
        Shape::Line {
            a: [1000.0, 2000.25],
            b: [3000.5, -0.0001],
            dashed: true,
        },
    )]);
    let bytes = dxf_bytes(&drawing, 0.0).unwrap();
    let start = bytes
        .windows(9)
        .position(|w| w == b"  0\nLINE\n")
        .expect("a LINE");
    let rest = &bytes[start..];
    let end = rest[4..]
        .windows(4)
        .enumerate()
        .position(|(i, w)| w == b"  0\n" && rest[3 + i] == b'\n')
        .map(|i| i + 4)
        .expect("something after the LINE");

    let all = pairs(&bytes);
    let at = all
        .iter()
        .position(|p| p == &(0, "LINE".to_string()))
        .unwrap();
    assert_eq!(all[at + 1].0, 5);
    assert_eq!(all[at + 2].0, 330);
    let (handle, owner) = (&all[at + 1].1, &all[at + 2].1);

    let expected = format!(
        "  0\nLINE\n  5\n{handle}\n330\n{owner}\n100\nAcDbEntity\n  8\nT\u{FC}r\n  6\nGESTRICHELT\n\
         100\nAcDbLine\n 10\n1000.000\n 20\n2000.250\n 30\n0.000\n 11\n3000.500\n 21\n0.000\n 31\n0.000\n"
    );
    let expected: Vec<u8> = expected.chars().map(|c| c as u8).collect();
    assert_eq!(
        String::from_utf8_lossy(&rest[..end]),
        String::from_utf8_lossy(&expected)
    );
    assert_eq!(&rest[..end], expected.as_slice());
}

#[test]
fn same_drawing_same_bytes() {
    assert_eq!(
        dxf_bytes(&full_drawing(), 7.5).unwrap(),
        dxf_bytes(&full_drawing(), 7.5).unwrap()
    );
}

#[test]
fn no_tmp_left() {
    let dir = temp_dir("no-tmp");
    let path = dir.path().join("plan.dxf");
    write_dxf(&path, &full_drawing(), 0.0, &Progress::new()).unwrap();
    assert_eq!(
        std::fs::read(&path).unwrap(),
        dxf_bytes(&full_drawing(), 0.0).unwrap()
    );
    let names: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name())
        .collect();
    assert_eq!(names, vec![std::ffi::OsString::from("plan.dxf")]);
}

#[test]
fn cancel_writes_nothing() {
    let dir = temp_dir("cancel");
    let path = dir.path().join("plan.dxf");
    let progress = Progress::new();
    progress.cancel();
    let error = write_dxf(&path, &full_drawing(), 0.0, &progress).unwrap_err();
    assert!(matches!(error, ExportError::Cancelled), "{error}");
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 0);
}
