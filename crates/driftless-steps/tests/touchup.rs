//! The touch-up step („Nacharbeit“) on a synthetic project: the steps of the Stellen of a map
//! replayed on its computed surface, kept in `nacharbeit/` in the surface folder and exported.
//! Every test works on its own copy of one project (`scan_with_images`, SLAM, map, surface).

use driftless_export::mesh::{read_mesh, Mesh};
use driftless_gpu::GpuPref;
use driftless_io::images::ImageIndices;
use driftless_io::progress::{Progress, ProgressSnapshot};
use driftless_io::testdir::{SharedDir, TestDir};
use driftless_io::testscan::{scan_with_images, shared, tmp};
use driftless_map::surface::edit::{dab_normal, FaceBvh, WeldedSurface, PATCHES_FILE};
use driftless_map::surface::{SurfaceConfig, TextureSource};
use driftless_steps::export::{run_export, ExportFormat, ExportKind, ExportRequest, ExportSource};
use driftless_steps::map::{run_map, MapInput, MapRequest, MapSettings, MapStep};
use driftless_steps::marks::{DabInput, Marks, SmoothLevel, StepInput};
use driftless_steps::slam::{run_slam, SlamRequest, SlamSettings};
use driftless_steps::surface::{run_surface, surface_fits, texture_sources, SurfaceRequest};
use driftless_steps::touchup::{
    edited_mesh, edited_pages, run_touchup, touchup_fits, touchup_without_raw_data,
    PreparedSurface, StepOutcome, TouchupDone, TouchupPrepared, TouchupRequest, TOUCHUP_DIR,
    TOUCHUP_REPORT,
};
use driftless_steps::StepError;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::SystemTime;

/// The hint of a step that needed the camera images while the raw data were gone.
const NO_RAW_DATA: &str = "Rohdaten fehlen – Bilder nicht neu eingespielt";

/// Where the device stood: the rays of the brush start here.
const EYE: [f64; 3] = [0.0, 0.0, 0.0];

/// Coarse settings: the synthetic room has a point every 10 cm or so.
fn small_config() -> SurfaceConfig {
    SurfaceConfig {
        voxel: 0.15,
        min_weight: 1,
        min_component_faces: 10,
        page_size: 256,
        simplify_error: 0.01,
        ..SurfaceConfig::default()
    }
}

fn map_input(scan: &Path, run: &Path) -> MapInput {
    MapInput {
        scan: scan.to_path_buf(),
        slam_dir: run.to_path_buf(),
        slam: SlamSettings::default(),
        map: MapSettings {
            step: MapStep::Clean,
            ..MapSettings::default()
        },
    }
}

/// The surface of the map of the project in `project` into `out`, with `config`.
fn surface_request(project: &Path, out: &Path, config: SurfaceConfig) -> SurfaceRequest {
    let scan = project.join("rohdaten").join("raum");
    let run = project.join("scans").join("raum").join("slam-001");
    SurfaceRequest {
        map: map_input(&scan, &run),
        map_dir: run.join("karte-001"),
        out_dir: out.to_path_buf(),
        config,
        texture: true,
        gpu: GpuPref::Off,
        obj: false,
    }
}

/// A project as the program lays it out, made once: the scan with camera images in
/// `projekt/rohdaten/raum`, its run `scans/raum/slam-001` with `lauf.toml`, the map
/// `karte-001` and its surface `oberflaeche-001`, and `project.toml`.
fn template() -> &'static Path {
    static DIR: OnceLock<SharedDir> = OnceLock::new();
    DIR.get_or_init(|| {
        let dir = shared("nacharbeit-vorlage");
        let project = dir.join("projekt");
        let scan = project.join("rohdaten").join("raum");
        fs::create_dir_all(&scan).unwrap();
        scan_with_images(&scan);
        fs::write(
            project.join("project.toml"),
            "format = 1\nname = \"P\"\n\n[[scan]]\nid = \"raum\"\npath = \"rohdaten/raum\"\n\
             name = \"Raum\"\n",
        )
        .unwrap();
        let run = project.join("scans").join("raum").join("slam-001");
        let slam = SlamRequest {
            scan: scan.clone(),
            out: run.clone(),
            settings: SlamSettings::default(),
            reuse: false,
            history: false,
        };
        run_slam(&slam, &Progress::new()).unwrap();
        fs::write(
            run.join("lauf.toml"),
            "scan = \"rohdaten/raum\"\nerstellt = \"2026-10-29T08:00:00Z\"\n\n[slam]\n",
        )
        .unwrap();
        let map = MapRequest {
            input: map_input(&scan, &run),
            out_dir: run.join("karte-001"),
            all_files: false,
        };
        run_map(&map, &Progress::new()).unwrap();
        let out = run.join("karte-001").join("oberflaeche-001");
        let report = run_surface(
            &surface_request(&project, &out, small_config()),
            &Progress::new(),
        )
        .unwrap();
        assert!(
            report["stats"]["texture"]["faces_textured"]
                .as_u64()
                .unwrap()
                > 0,
            "the surface has images: {report}"
        );
        dir
    })
    .path()
}

/// Copies the folder `from` with everything in it to `to`.
fn copy_dir(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for e in fs::read_dir(from).unwrap() {
        let e = e.unwrap();
        let target = to.join(e.file_name());
        if e.file_type().unwrap().is_dir() {
            copy_dir(&e.path(), &target);
        } else {
            fs::copy(e.path(), &target).unwrap();
        }
    }
}

/// A copy of [`template`] of a test: the project, its map folder (which holds the Stellen)
/// and the surface in it.
struct Scene {
    dir: TestDir,
    project: PathBuf,
    map: PathBuf,
    surface: PathBuf,
}

impl Scene {
    fn new(name: &str) -> Scene {
        let dir = tmp(name);
        copy_dir(template(), &dir);
        Scene::at(dir)
    }

    /// The project copied into `dir`.
    fn at(dir: TestDir) -> Scene {
        let project = dir.join("projekt");
        let map = project
            .join("scans")
            .join("raum")
            .join("slam-001")
            .join("karte-001");
        let surface = map.join("oberflaeche-001");
        Scene {
            dir,
            project,
            map,
            surface,
        }
    }

    /// The camera images of the surface, as the program finds them.
    fn sources(&self) -> Option<Vec<TextureSource>> {
        let sources = texture_sources(&self.surface).unwrap();
        assert!(sources.as_ref().is_some_and(|s| s.len() == 1));
        sources
    }

    fn request(&self, sources: Option<Vec<TextureSource>>) -> TouchupRequest {
        TouchupRequest {
            surface_dir: self.surface.clone(),
            owner_dir: self.map.clone(),
            sources,
            prepared: TouchupPrepared::default(),
        }
    }

    /// Runs the touch-up with the camera images.
    fn touch_up(&self) -> TouchupDone {
        run_touchup(&self.request(self.sources()), &Progress::new()).unwrap()
    }

    fn marks(&self) -> Marks {
        Marks::load(&self.map).unwrap()
    }

    /// Paints `dabs` as a new Stelle, then adds `step` to it if given; its number.
    fn mark(&self, dabs: &[DabInput], step: Option<StepInput>) -> u32 {
        let (_, nr) = Marks::change(&self.map, |m| {
            let nr = m.paint(None, dabs);
            if let Some(step) = step {
                m.add_step(nr, step)?;
            }
            Ok(nr)
        })
        .unwrap();
        nr
    }

    fn glb(&self) -> Mesh {
        read_mesh(&self.surface.join("oberflaeche.glb"), &Progress::new()).unwrap()
    }

    fn cache(&self) -> PathBuf {
        self.surface.join(TOUCHUP_DIR)
    }
}

/// Dabs of `radius` where the rays from [`EYE`] towards `targets` meet the surface in
/// `surface_dir`, each with the normal the brush gives it.
fn dabs(surface_dir: &Path, targets: &[[f64; 3]], radius: f64) -> Vec<DabInput> {
    let mesh = read_mesh(&surface_dir.join("oberflaeche.glb"), &Progress::new()).unwrap();
    let s = WeldedSurface::from_mesh(&mesh);
    let bvh = FaceBvh::build(&s);
    targets
        .iter()
        .map(|t| {
            let dir = [t[0] - EYE[0], t[1] - EYE[1], t[2] - EYE[2]];
            let hit = bvh
                .raycast(&s, EYE, dir, None)
                .unwrap_or_else(|| panic!("the ray towards {t:?} hits the surface"));
            DabInput {
                centre: hit.pos,
                normal: dab_normal(&s, &bvh, &hit, radius),
                radius: radius as f32,
                erase: false,
            }
        })
        .collect()
}

/// A Stelle on the +x wall, the plain one the camera looks at: about 1.5 × 0.7 m around
/// (6, 1, 0).
fn front_wall(surface_dir: &Path) -> Vec<DabInput> {
    let targets: Vec<[f64; 3]> = (0..5)
        .map(|i| [6.0, 0.6 + 0.2 * f64::from(i), 0.0])
        .collect();
    let dabs = dabs(surface_dir, &targets, 0.35);
    for d in &dabs {
        assert!(d.normal[0] < -0.9, "faces the device: {d:?}");
    }
    dabs
}

/// A Stelle on the −x wall, behind the camera: no image shows it, its faces are gray.
fn back_wall(surface_dir: &Path) -> Vec<DabInput> {
    let targets = [[-4.0, 1.0, 0.25], [-4.0, 1.4, 0.25]];
    let dabs = dabs(surface_dir, &targets, 0.35);
    for d in &dabs {
        assert!(d.normal[0] > 0.9, "faces the device: {d:?}");
    }
    dabs
}

/// A Stelle across the corner of the −x and the −y wall: not flat.
fn corner(surface_dir: &Path) -> Vec<DabInput> {
    let dabs = dabs(surface_dir, &[[-4.0, -2.6, 0.0], [-3.6, -3.0, 0.0]], 0.45);
    assert!(
        dabs[0].normal[0] > 0.9 && dabs[1].normal[1] > 0.9,
        "{dabs:?}"
    );
    dabs
}

fn smooth(level: SmoothLevel) -> Option<StepInput> {
    Some(StepInput::Smooth { level })
}

/// The places the triangles of `mesh` use, by their bits.
fn used(mesh: &Mesh) -> BTreeSet<[u64; 3]> {
    mesh.pages
        .iter()
        .flat_map(|p| {
            p.indices
                .iter()
                .flatten()
                .map(|&i| p.positions[i as usize].map(|c| (c + 0.0).to_bits()))
        })
        .collect()
}

/// [`used`] as `f32`: an OBJ holds the places as the shortest text of their `f32`, which reads
/// back as another `f64`.
fn used_f32(mesh: &Mesh) -> BTreeSet<[u32; 3]> {
    used(mesh)
        .into_iter()
        .map(|p| p.map(|c| (f64::from_bits(c) as f32).to_bits()))
        .collect()
}

/// The vertices of a binary PLY mesh with `float` coordinates, by their bits (as `f64`), and
/// its number of faces.
fn ply_vertices(path: &Path) -> (BTreeSet<[u64; 3]>, usize) {
    let bytes = fs::read(path).unwrap();
    let end = b"end_header\n";
    let at = bytes.windows(end.len()).position(|w| w == end).unwrap() + end.len();
    let header = std::str::from_utf8(&bytes[..at]).unwrap();
    assert!(header.contains("property float x"), "{header}");
    let count = |element: &str| -> usize {
        header
            .lines()
            .find_map(|l| l.strip_prefix(&format!("element {element} ")))
            .unwrap()
            .parse()
            .unwrap()
    };
    let (vertices, faces) = (count("vertex"), count("face"));
    let row = 3 * 4 + 3;
    let set = bytes[at..at + vertices * row]
        .chunks(row)
        .map(|r| {
            std::array::from_fn(|k| {
                let c = f32::from_le_bytes(r[4 * k..4 * k + 4].try_into().unwrap());
                (f64::from(c) + 0.0).to_bits()
            })
        })
        .collect();
    (set, faces)
}

/// The files directly in `dir`, with their bytes.
fn files_in(dir: &Path) -> BTreeMap<String, Vec<u8>> {
    fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap())
        .filter(|e| e.file_type().unwrap().is_file())
        .map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            (name, fs::read(e.path()).unwrap())
        })
        .collect()
}

/// The names in `dir`, sorted.
fn names(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

/// Every path below `dir` whose name ends in `.tmp`.
fn tmp_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for e in fs::read_dir(dir).unwrap() {
        let path = e.unwrap().path();
        if path.to_string_lossy().ends_with(".tmp") {
            out.push(path.clone());
        }
        if path.is_dir() {
            out.extend(tmp_files(&path));
        }
    }
    out
}

/// When the files of the cache were last written.
fn cache_times(scene: &Scene) -> Vec<SystemTime> {
    [PATCHES_FILE, TOUCHUP_REPORT]
        .iter()
        .map(|f| {
            fs::metadata(scene.cache().join(f))
                .unwrap()
                .modified()
                .unwrap()
        })
        .collect()
}

/// A progress that records every phase it is told.
fn recording() -> (Progress, Arc<Mutex<Vec<String>>>) {
    let phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::clone(&phases);
    let progress = Progress::new().with_observer(move |s: &ProgressSnapshot| {
        let mut seen = seen.lock().unwrap();
        if seen.last() != Some(&s.phase) {
            seen.push(s.phase.clone());
        }
    });
    (progress, phases)
}

fn export(source: ExportSource, out: &Path, format: ExportFormat) {
    let req = ExportRequest {
        source,
        out: out.to_path_buf(),
        format,
        overwrite: false,
    };
    run_export(&req, &Progress::new()).unwrap();
}

#[test]
fn smooth_then_export_contains_it() {
    let scene = Scene::new("nacharbeit-export");
    let before = files_in(&scene.surface);
    let computed = scene.glb();
    let nr = scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    assert!(!touchup_fits(&scene.surface, &scene.marks()));
    assert_eq!(edited_mesh(&scene.surface, &Progress::new()).unwrap(), None);

    let done = scene.touch_up();
    assert_eq!((done.steps, &done.hints), (1, &Vec::new()));
    assert!(done.refused.is_empty(), "{done:?}");
    assert_eq!(nr, 1);
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    // What came of the step: smoothed, then its images baked anew as the best variant.
    let seq = scene.marks().marks[0].steps[0].seq();
    let [outcome] = &done.outcomes[..] else {
        panic!("{done:?}")
    };
    let variant_of = outcome.variant_of.expect("images baked");
    assert!(variant_of.0 == 1 && variant_of.1 >= 1, "{done:?}");
    assert_eq!(
        *outcome,
        StepOutcome {
            mark: nr,
            seq,
            variant_of: Some(variant_of),
            refused: None,
            without_raw_data: false,
        }
    );
    assert!(!touchup_without_raw_data(&scene.surface));

    // Moved points, and the images of the Stelle baked anew onto a page of its own.
    let edited = edited_mesh(&scene.surface, &Progress::new())
        .unwrap()
        .expect("the touched-up surface");
    let moved: Vec<_> = used(&edited)
        .difference(&used(&computed))
        .copied()
        .collect();
    assert!(!moved.is_empty(), "no point moved");
    assert!(edited.pages.len() > computed.pages.len());
    assert_eq!(edited.triangles(), computed.triangles());
    for (a, b) in computed.pages.iter().zip(&edited.pages) {
        assert_eq!(a.image, b.image, "an old page keeps its image");
    }

    // What the view takes: the changed pages and the added ones.
    let s = WeldedSurface::from_mesh(&computed);
    let (changed, added) = edited_pages(&scene.surface, &s, &computed)
        .unwrap()
        .expect("the changes for the view");
    assert!(!changed.is_empty());
    assert_eq!(added, edited.pages[computed.pages.len()..]);

    // The exports hold it.
    let out = scene.dir.join("ausgabe");
    fs::create_dir_all(&out).unwrap();
    let source = ExportSource::EditedSurface(scene.surface.clone());
    assert_eq!(source.kind(), ExportKind::Mesh);
    assert_eq!(
        source.default_name(),
        ExportSource::Surface(scene.surface.clone()).default_name()
    );
    export(source.clone(), &out.join("n.glb"), ExportFormat::Glb);
    export(source.clone(), &out.join("n.obj"), ExportFormat::Obj);
    export(source.clone(), &out.join("n.ply"), ExportFormat::PlyMesh);
    let glb = read_mesh(&out.join("n.glb"), &Progress::new()).unwrap();
    assert_eq!(glb.pages.len(), edited.pages.len());
    assert_eq!(used(&glb), used(&edited));
    let obj = read_mesh(&out.join("n.obj"), &Progress::new()).unwrap();
    assert_eq!(obj.pages.len(), edited.pages.len());
    assert_eq!(used_f32(&obj), used_f32(&edited));
    let (ply, faces) = ply_vertices(&out.join("n.ply"));
    assert_eq!(ply, used(&edited));
    assert_eq!(faces, edited.triangles());

    // A file of the touch-up is no target.
    let target = scene.cache().join(PATCHES_FILE);
    let req = ExportRequest {
        source,
        out: target.clone(),
        format: ExportFormat::Glb,
        overwrite: true,
    };
    match run_export(&req, &Progress::new()) {
        Err(StepError::Failed(m)) => assert!(m.starts_with("Ziel ist die Quelle selbst"), "{m}"),
        other => panic!("expected a refusal, got {other:?}"),
    }

    // The computed surface is as it was, byte for byte, and still current.
    assert_eq!(files_in(&scene.surface), before);
    let mut entries: Vec<String> = before.keys().cloned().collect();
    entries.push(TOUCHUP_DIR.into());
    entries.sort();
    assert_eq!(names(&scene.surface), entries);
    assert_eq!(names(&scene.cache()), [PATCHES_FILE, TOUCHUP_REPORT]);
    assert!(tmp_files(&scene.dir).is_empty());
    assert!(surface_fits(&surface_request(
        &scene.project,
        &scene.surface,
        small_config()
    )));
}

#[test]
fn cache_is_reused() {
    let scene = Scene::new("nacharbeit-wieder");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();
    let bytes = fs::read(scene.cache().join(PATCHES_FILE)).unwrap();
    let times = cache_times(&scene);

    // Nothing changed: nothing is computed or written.
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    let (progress, phases) = recording();
    let done = run_touchup(&scene.request(scene.sources()), &progress).unwrap();
    assert_eq!(done.steps, 1);
    assert!(phases.lock().unwrap().is_empty(), "{phases:?}");
    assert_eq!(cache_times(&scene), times);

    // A new Stelle without a step changes nothing either.
    scene.mark(&back_wall(&scene.surface), None);
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    scene.touch_up();
    assert_eq!(cache_times(&scene), times);

    // Dabs painted into the smoothed Stelle after its step count for later steps only: the
    // cache no longer fits the Stellen, but the step is kept, not computed anew.
    Marks::change(&scene.map, |m| {
        let more = dabs(&scene.surface, &[[6.0, 2.0, 0.5]], 0.3);
        m.paint(Some(1), &more);
        Ok(())
    })
    .unwrap();
    assert!(!touchup_fits(&scene.surface, &scene.marks()));
    let (progress, phases) = recording();
    run_touchup(&scene.request(scene.sources()), &progress).unwrap();
    assert!(
        !phases
            .lock()
            .unwrap()
            .iter()
            .any(|p| p.starts_with("Nacharbeit: Schritt")),
        "{phases:?}"
    );
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), bytes);

    // Computed anew from nothing, the step gives the same bytes: the later dabs do not count.
    fs::remove_dir_all(scene.cache()).unwrap();
    assert!(!touchup_fits(&scene.surface, &scene.marks()));
    scene.touch_up();
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), bytes);
}

#[test]
fn added_step_computes_only_itself() {
    let scene = Scene::new("nacharbeit-dazu");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();
    let back = scene.mark(&back_wall(&scene.surface), smooth(SmoothLevel::Light));
    let (progress, phases) = recording();
    let done = run_touchup(&scene.request(scene.sources()), &progress).unwrap();
    let phases = phases.lock().unwrap().clone();
    assert!(
        phases.iter().any(|p| p == "Nacharbeit: Schritt 2 von 2"),
        "{phases:?}"
    );
    assert!(
        !phases.iter().any(|p| p == "Nacharbeit: Schritt 1 von 2"),
        "{phases:?}"
    );
    // The back wall is smoothed, but no image shows it: it keeps its texture, with a hint.
    assert_eq!(done.steps, 2);
    assert!(done.refused.is_empty(), "{done:?}");
    assert_eq!(done.hints.len(), 1, "{done:?}");
    assert!(
        done.hints[0].starts_with(&format!("Stelle {back} ")),
        "{done:?}"
    );
    assert!(done.hints[0].contains("geglättet"), "{done:?}");

    // The same as computed from nothing.
    let bytes = fs::read(scene.cache().join(PATCHES_FILE)).unwrap();
    fs::remove_dir_all(scene.cache()).unwrap();
    assert_eq!(scene.touch_up(), done);
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), bytes);
}

#[test]
fn refused_steps_leave_the_others() {
    let scene = Scene::new("nacharbeit-abgelehnt");
    let flat = scene.mark(&corner(&scene.surface), smooth(SmoothLevel::Flat));
    let unseen = scene.mark(
        &back_wall(&scene.surface),
        Some(StepInput::Retexture { variant: 0 }),
    );
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    let before = scene.marks();
    let done = scene.touch_up();
    assert_eq!(done.steps, 3);
    assert_eq!(done.refused.len(), 2, "{done:?}");
    assert_eq!(done.refused[0].0, flat);
    assert!(
        done.refused[0]
            .1
            .starts_with("Diese Stelle ist nicht eben genug für „ganz eben“"),
        "{done:?}"
    );
    assert_eq!(
        done.refused[1],
        (
            unseen,
            "Kein Bild zeigt diese Stelle; sie bleibt, wie sie ist.".to_string()
        )
    );
    assert!(done.hints.is_empty(), "{done:?}");
    // Each step with its seq: the refused ones say why, in the order of the replay.
    let marks = scene.marks();
    let seq_of = |nr: u32| marks.marks.iter().find(|m| m.nr == nr).unwrap().steps[0].seq();
    let refused: Vec<(u32, u64, Option<&str>)> = done
        .outcomes
        .iter()
        .map(|o| (o.mark, o.seq, o.refused.as_deref()))
        .collect();
    assert_eq!(refused.len(), 3, "{done:?}");
    assert_eq!(
        (refused[0].0, refused[0].1),
        (flat, seq_of(flat)),
        "{done:?}"
    );
    assert_eq!(refused[0].2, Some(done.refused[0].1.as_str()));
    assert_eq!(
        refused[1],
        (
            unseen,
            seq_of(unseen),
            Some("Kein Bild zeigt diese Stelle; sie bleibt, wie sie ist.")
        )
    );
    assert_eq!(refused[2].2, None);
    assert!(done.outcomes[..2].iter().all(|o| o.variant_of.is_none()));
    // The third step ran.
    let edited = edited_mesh(&scene.surface, &Progress::new())
        .unwrap()
        .unwrap();
    assert!(!used(&edited).is_subset(&used(&scene.glb())));
    assert!(edited.pages.len() > scene.glb().pages.len());
    // The Stellen are left as they were: taking a refused step out is the program's part.
    assert_eq!(scene.marks(), before);
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    // Reused, the cache says the same.
    assert_eq!(scene.touch_up(), done);
}

#[test]
fn moved_project_keeps_cache() {
    let scene = Scene::new("nacharbeit-verschoben");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();
    let edited = edited_mesh(&scene.surface, &Progress::new()).unwrap();
    assert!(edited.is_some());

    let moved = Scene::at(tmp("nacharbeit-verschoben-neu"));
    copy_dir(&scene.project, &moved.project);
    drop(scene);
    assert!(touchup_fits(&moved.surface, &moved.marks()));
    assert_eq!(
        edited_mesh(&moved.surface, &Progress::new()).unwrap(),
        edited
    );
    let times = cache_times(&moved);
    let (progress, phases) = recording();
    run_touchup(&moved.request(moved.sources()), &progress).unwrap();
    assert!(phases.lock().unwrap().is_empty(), "{phases:?}");
    assert_eq!(cache_times(&moved), times);
}

#[test]
fn new_surface_reapplies() {
    let scene = Scene::new("nacharbeit-neue-oberflaeche");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();

    // A second surface of the same map, coarser: it takes the same Stellen.
    let second = scene.map.join("oberflaeche-002");
    let config = SurfaceConfig {
        voxel: 0.2,
        ..small_config()
    };
    run_surface(
        &surface_request(&scene.project, &second, config),
        &Progress::new(),
    )
    .unwrap();
    assert!(!touchup_fits(&second, &scene.marks()));
    let req = TouchupRequest {
        surface_dir: second.clone(),
        owner_dir: scene.map.clone(),
        sources: texture_sources(&second).unwrap(),
        prepared: TouchupPrepared::default(),
    };
    let done = run_touchup(&req, &Progress::new()).unwrap();
    assert_eq!(done.steps, 1);
    assert!(done.refused.is_empty(), "{done:?}");
    assert!(touchup_fits(&second, &scene.marks()));
    assert!(touchup_fits(&scene.surface, &scene.marks()));

    // Its own patches, of its own faces.
    let own = fs::read(second.join(TOUCHUP_DIR).join(PATCHES_FILE)).unwrap();
    let first = fs::read(scene.cache().join(PATCHES_FILE)).unwrap();
    assert_ne!(own, first);
    let computed = read_mesh(&second.join("oberflaeche.glb"), &Progress::new()).unwrap();
    let edited = edited_mesh(&second, &Progress::new()).unwrap().unwrap();
    assert!(!used(&edited).is_subset(&used(&computed)));
    assert!(edited.pages.len() > computed.pages.len());
}

#[test]
fn no_raw_data() {
    let scene = Scene::new("nacharbeit-ohne-rohdaten");
    let computed = scene.glb();
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    let retexture = Some(StepInput::Retexture { variant: 0 });
    scene.mark(&dabs(&scene.surface, &[[6.0, -1.5, 0.5]], 0.3), retexture);

    // Smoothing works; the images are not baked anew, which a hint says once.
    let done = run_touchup(&scene.request(None), &Progress::new()).unwrap();
    assert_eq!(
        (done.steps, &done.hints, &done.refused),
        (2, &vec![NO_RAW_DATA.to_string()], &Vec::new())
    );
    let outcomes: Vec<_> = done
        .outcomes
        .iter()
        .map(|o| (o.mark, o.without_raw_data, o.variant_of))
        .collect();
    assert_eq!(outcomes, [(1, true, None), (2, true, None)]);
    assert!(touchup_without_raw_data(&scene.surface));
    let edited = edited_mesh(&scene.surface, &Progress::new())
        .unwrap()
        .unwrap();
    assert!(!used(&edited).is_subset(&used(&computed)));
    assert_eq!(edited.pages.len(), computed.pages.len());
    for (a, b) in computed.pages.iter().zip(&edited.pages) {
        assert_eq!(a.uvs, b.uvs, "the texture moves with the points");
        assert_eq!(a.image, b.image);
    }
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    // Still without them: kept as it is.
    let times = cache_times(&scene);
    assert_eq!(
        run_touchup(&scene.request(None), &Progress::new()).unwrap(),
        done
    );
    assert_eq!(cache_times(&scene), times);

    // Once the raw data are back, both steps get their images.
    let done = scene.touch_up();
    assert_eq!(done.steps, 2);
    assert!(done.hints.is_empty() && done.refused.is_empty(), "{done:?}");
    assert!(
        done.outcomes
            .iter()
            .all(|o| !o.without_raw_data && o.variant_of.is_some()),
        "{done:?}"
    );
    assert!(!touchup_without_raw_data(&scene.surface));
    let edited = edited_mesh(&scene.surface, &Progress::new())
        .unwrap()
        .unwrap();
    assert!(edited.pages.len() >= computed.pages.len() + 2);
}

#[test]
fn cancel_leaves_nothing() {
    let scene = Scene::new("nacharbeit-abbruch");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    let before = names(&scene.surface);
    let cancelling = |phase: &'static str| {
        let progress = Progress::new();
        let handle = progress.clone();
        progress.with_observer(move |s: &ProgressSnapshot| {
            if s.phase == phase {
                handle.cancel();
            }
        })
    };
    // In the middle of baking the images of the smoothed Stelle anew.
    let err = run_touchup(
        &scene.request(scene.sources()),
        &cancelling("Bilder der Stelle suchen"),
    )
    .unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(names(&scene.surface), before);
    assert!(!touchup_fits(&scene.surface, &scene.marks()));

    // What a run killed while writing left: the next run removes it.
    let killed = scene.surface.join(format!("{TOUCHUP_DIR}.tmp-4242-neu"));
    fs::create_dir_all(&killed).unwrap();
    fs::write(killed.join(format!("{PATCHES_FILE}.tmp")), b"halb").unwrap();
    fs::create_dir_all(scene.surface.join(format!("{TOUCHUP_DIR}.tmp-4242-alt"))).unwrap();
    scene.touch_up();
    let mut after = before.clone();
    after.push(TOUCHUP_DIR.into());
    after.sort();
    assert_eq!(names(&scene.surface), after);
    assert!(tmp_files(&scene.surface).is_empty());
    assert!(touchup_fits(&scene.surface, &scene.marks()));

    // A cancelled run leaves the cache of the steps before as it was.
    let bytes = fs::read(scene.cache().join(PATCHES_FILE)).unwrap();
    scene.mark(&back_wall(&scene.surface), smooth(SmoothLevel::Light));
    let err = run_touchup(
        &scene.request(scene.sources()),
        &cancelling("Nacharbeit: Schritt 2 von 2"),
    )
    .unwrap_err();
    assert_eq!(err, StepError::Cancelled);
    assert_eq!(names(&scene.surface), after);
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), bytes);
    assert!(!touchup_fits(&scene.surface, &scene.marks()));
}

#[test]
fn cli_export_unchanged() {
    let scene = Scene::new("nacharbeit-cli");
    scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();
    // The command line finds the computed surface in the folder and exports it as computed.
    assert_eq!(
        ExportSource::detect(&scene.surface).unwrap(),
        ExportSource::Surface(scene.surface.clone())
    );
    let out = scene.dir.join("ausgabe");
    fs::create_dir_all(&out).unwrap();
    let source = ExportSource::Surface(scene.surface.clone());
    export(source.clone(), &out.join("s.glb"), ExportFormat::Glb);
    export(source, &out.join("s.obj"), ExportFormat::Obj);
    let computed = scene.glb();
    assert_eq!(
        read_mesh(&out.join("s.glb"), &Progress::new()).unwrap(),
        computed
    );
    let obj = read_mesh(&out.join("s.obj"), &Progress::new()).unwrap();
    assert_eq!(obj.pages.len(), computed.pages.len());
    assert_eq!(used_f32(&obj), used_f32(&computed));
}

#[test]
fn edited_export_without_steps_or_current_cache() {
    let scene = Scene::new("nacharbeit-export-leer");
    let out = scene.dir.join("ausgabe");
    fs::create_dir_all(&out).unwrap();
    let source = ExportSource::EditedSurface(scene.surface.clone());
    let computed = scene.glb();

    // No step on any Stelle: nothing to keep, and the touched-up surface is the computed one.
    scene.mark(&front_wall(&scene.surface), None);
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    assert_eq!(scene.touch_up(), TouchupDone::default());
    assert!(!scene.cache().exists());
    assert_eq!(edited_mesh(&scene.surface, &Progress::new()).unwrap(), None);
    export(source.clone(), &out.join("leer.glb"), ExportFormat::Glb);
    assert_eq!(
        read_mesh(&out.join("leer.glb"), &Progress::new()).unwrap(),
        computed
    );

    // A step the touch-up has not computed yet: the export says so instead of leaving it out.
    Marks::change(&scene.map, |m| {
        m.add_step(
            1,
            StepInput::Smooth {
                level: SmoothLevel::Light,
            },
        )
    })
    .unwrap();
    let req = ExportRequest {
        source: source.clone(),
        out: out.join("alt.glb"),
        format: ExportFormat::Glb,
        overwrite: false,
    };
    match run_export(&req, &Progress::new()) {
        Err(StepError::Failed(m)) => assert!(m.contains("Nacharbeit"), "{m}"),
        other => panic!("expected a refusal, got {other:?}"),
    }
    assert!(!out.join("alt.glb").exists());

    // Taken back again: the cache of it goes with the next run.
    scene.touch_up();
    assert!(scene.cache().is_dir());
    Marks::change(&scene.map, |m| Ok(m.undo())).unwrap();
    assert!(touchup_fits(&scene.surface, &scene.marks()));
    assert_eq!(edited_mesh(&scene.surface, &Progress::new()).unwrap(), None);
    assert_eq!(scene.touch_up(), TouchupDone::default());
    assert!(!scene.cache().exists());
    assert!(names(&scene.surface)
        .iter()
        .all(|n| !n.starts_with(TOUCHUP_DIR)));
}

/// The program has the computed surface at hand already (read, welded, its tree): the
/// touch-up takes it instead of reading the glb – the glb here made unreadable with its size
/// and time kept – and gives the same patches. The indices of the images are built once and
/// kept for the next run. A glb with another size or time is read anew.
#[test]
fn prepared_surface_is_used() {
    let scene = Scene::new("nacharbeit-vorbereitet");
    let front = scene.mark(&front_wall(&scene.surface), smooth(SmoothLevel::Strong));
    scene.touch_up();
    Marks::change(&scene.map, |m| {
        m.add_step(front, StepInput::Retexture { variant: 1 })?;
        Ok(())
    })
    .unwrap();
    scene.touch_up();
    let want = fs::read(scene.cache().join(PATCHES_FILE)).unwrap();
    let glb = scene.surface.join("oberflaeche.glb");
    let prepared = PreparedSurface::read(&glb, &Progress::new()).unwrap();
    assert!(prepared.fits(&glb));

    // The glb unreadable, its size and time as they were.
    let meta = fs::metadata(&glb).unwrap();
    let bytes = fs::read(&glb).unwrap();
    fs::write(&glb, vec![0u8; bytes.len()]).unwrap();
    fs::File::options()
        .write(true)
        .open(&glb)
        .unwrap()
        .set_modified(meta.modified().unwrap())
        .unwrap();
    assert!(prepared.fits(&glb));
    fs::remove_dir_all(scene.cache()).unwrap();
    // Without it the touch-up cannot read the surface …
    assert!(run_touchup(&scene.request(scene.sources()), &Progress::new()).is_err());
    // … with it, it computes the same patches without reading it, and keeps the indices.
    let indices = Arc::new(ImageIndices::default());
    let with = |prepared: &PreparedSurface| TouchupRequest {
        prepared: TouchupPrepared {
            surface: Some(prepared.clone()),
            indices: Some(Arc::clone(&indices)),
        },
        ..scene.request(scene.sources())
    };
    let (progress, phases) = recording();
    let done = run_touchup(&with(&prepared), &progress).unwrap();
    assert_eq!(done.steps, 2);
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), want);
    let phases = phases.lock().unwrap().clone();
    assert!(
        !phases.iter().any(|p| p == "Nacharbeit: Oberfläche lesen"),
        "{phases:?}"
    );
    assert_eq!(
        phases.iter().filter(|p| *p == "Bilder verzeichnen").count(),
        1,
        "{phases:?}"
    );
    // Once more from nothing: the indices are kept.
    fs::remove_dir_all(scene.cache()).unwrap();
    let (progress, phases) = recording();
    run_touchup(&with(&prepared), &progress).unwrap();
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), want);
    assert!(
        !phases
            .lock()
            .unwrap()
            .iter()
            .any(|p| p == "Bilder verzeichnen"),
        "{phases:?}"
    );

    // The glb back as it was, but written anew: another time, so it is read again.
    fs::write(&glb, &bytes).unwrap();
    fs::File::options()
        .write(true)
        .open(&glb)
        .unwrap()
        .set_modified(meta.modified().unwrap() + std::time::Duration::from_secs(5))
        .unwrap();
    assert!(!prepared.fits(&glb));
    fs::remove_dir_all(scene.cache()).unwrap();
    let (progress, phases) = recording();
    run_touchup(&with(&prepared), &progress).unwrap();
    assert!(
        phases
            .lock()
            .unwrap()
            .iter()
            .any(|p| p == "Nacharbeit: Oberfläche lesen"),
        "{phases:?}"
    );
    assert_eq!(fs::read(scene.cache().join(PATCHES_FILE)).unwrap(), want);
}
