//! Foreign files: small samples written by laspy, pye57, open3d and trimesh (see
//! `tools/export/make_foreign.py`) read against `fixtures/foreign/expected.json`: the number
//! of points or triangles, the bounds and the colour of the first points. A deviation is a
//! bug in the reader, not in the expectation.

use driftless_export::mesh::{read_mesh, ImageKind};
use driftless_export::{read_points, ExportError};
use driftless_io::progress::Progress;
use serde_json::Value;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/foreign")
}

fn expected() -> serde_json::Map<String, Value> {
    let text = std::fs::read_to_string(fixtures().join("expected.json")).unwrap();
    serde_json::from_str::<Value>(&text)
        .unwrap()
        .as_object()
        .unwrap()
        .clone()
}

fn triple(v: &Value) -> [f64; 3] {
    let a = v.as_array().unwrap();
    [0, 1, 2].map(|k| a[k].as_f64().unwrap())
}

/// Per-axis minimum and maximum of `points`.
fn bounds<'a>(points: impl Iterator<Item = &'a [f64; 3]>) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

fn assert_bounds(name: &str, got: ([f64; 3], [f64; 3]), want: &Value, tol: f64) {
    let b = want.as_array().unwrap();
    let (lo, hi) = (triple(&b[0]), triple(&b[1]));
    for k in 0..3 {
        assert!(
            (got.0[k] - lo[k]).abs() <= tol && (got.1[k] - hi[k]).abs() <= tol,
            "{name}: bounds {got:?}, expected {lo:?} – {hi:?} (±{tol})"
        );
    }
}

#[test]
fn foreign_point_files_read_as_written() {
    let mut seen = 0;
    for (name, want) in expected() {
        let path = fixtures().join(&name);
        if !matches!(
            path.extension().and_then(|e| e.to_str()),
            Some("las" | "laz" | "e57" | "ply" | "pcd")
        ) {
            continue;
        }
        seen += 1;
        let result = read_points(&path, &Progress::new());
        if let Some(msg) = want.get("error") {
            match result {
                Err(ExportError::Format { msg: got, .. }) => {
                    assert_eq!(got, msg.as_str().unwrap(), "{name}")
                }
                other => panic!("{name}: expected the message, got {other:?}"),
            }
            continue;
        }
        let points = result.unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(
            points.len() as u64,
            want["count"].as_u64().unwrap(),
            "{name}"
        );
        let tol = want["tolerance"].as_f64().unwrap();
        assert_bounds(&name, bounds(points.xyz.iter()), &want["bounds"], tol);
        let rgb = points
            .rgb
            .as_ref()
            .unwrap_or_else(|| panic!("{name}: no colour"));
        for (i, c) in want["rgb"].as_array().unwrap().iter().enumerate() {
            let c = triple(c).map(|v| v as u16);
            assert_eq!(rgb[i], c, "{name}: colour of point {i}");
        }
        if let Some(n) = want.get("sources") {
            let sources: BTreeSet<u16> = points
                .source
                .as_ref()
                .unwrap_or_else(|| panic!("{name}: no scan numbers"))
                .iter()
                .copied()
                .collect();
            assert_eq!(sources.len() as u64, n.as_u64().unwrap(), "{name}");
        }
        if let Some(scans) = want.get("scans") {
            let names: Vec<&str> = points.scans.iter().map(|s| s.name.as_str()).collect();
            let want: Vec<&str> = scans
                .as_array()
                .unwrap()
                .iter()
                .map(|s| s.as_str().unwrap())
                .collect();
            assert_eq!(names, want, "{name}");
        }
    }
    assert!(seen >= 7, "only {seen} point files");
}

#[test]
fn foreign_meshes_read_as_written() {
    let mut seen = 0;
    for (name, want) in expected() {
        let path = fixtures().join(&name);
        if !matches!(
            path.extension().and_then(|e| e.to_str()),
            Some("glb" | "obj")
        ) {
            continue;
        }
        seen += 1;
        let progress = Progress::new();
        let mesh = read_mesh(&path, &progress).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(progress.take_log().is_empty(), "{name}: warnings");
        assert_eq!(
            mesh.triangles() as u64,
            want["triangles"].as_u64().unwrap(),
            "{name}"
        );
        let tol = want["tolerance"].as_f64().unwrap();
        let corners = mesh.pages.iter().flat_map(|p| p.positions.iter());
        assert_bounds(&name, bounds(corners), &want["bounds"], tol);
        let kind = match want["image"].as_str().unwrap() {
            "png" => ImageKind::Png,
            _ => ImageKind::Jpeg,
        };
        assert!(
            mesh.pages
                .iter()
                .all(|p| p.image.as_ref().is_some_and(|i| i.kind == kind)),
            "{name}: every page with its image"
        );
    }
    assert_eq!(seen, 2);
}
