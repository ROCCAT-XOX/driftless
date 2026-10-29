//! The brush and the touch-up („Nacharbeit“) on the four real scans, as a user works in the
//! window (the numbers of `docs/ergebnisse/2026-10-29-pinsel.md`): a project with the dark room
//! S1–S3 and the Büro B from `$DRIFTLESS_TESTDATA` as `whole_run_real` makes it (trajectories and
//! maps with the defaults, the surfaces of S1 and B, the model of S1–S3 and its surface), then on
//! the surface of the model and on that of B one wall Stelle of about 1 m², painted with fixed
//! dabs ([`Action::Paint`]), and on it „ganz eben“, „stark“ and „Bilder neu“ (variants 0 and 1),
//! each in the window with its progress, ↶ in between. The wall is found from the places of the
//! device: a plain one (no object on it or in front of it, room before it), the roughest of them.
//!
//! Per action it prints the time and the peak memory, where the time went (the phases of the
//! touch-up, the bytes read meanwhile, the lines it adds to the protocol), the checksum of the
//! patches (their cache is copied to `$DRIFTLESS_WORK/vergleich/`, to compare two versions of the
//! program byte for byte), the size of the touched-up state the view uploads and the frame that
//! uploads it; per source the time to index its images from the disk and from the page cache;
//! per Stelle how many images „Bilder neu“ looks at, the distance of
//! its points to their plane and their roughness before and after, the moves next to its border
//! and the height step across it (the border itself stays), the colour difference across the
//! border before and after the images were baked anew, and the holes the selection encloses
//! (Ruling 12: only those up to 25 cm² are closed). Also the frame rate while painting on B, the
//! angle a pixel of the real cameras spans against the one at the centre (the cone of the depth
//! buffer of „Bilder neu“ assumes at most 3 times), and the folder of the S1 surface for
//! comparing it with another version. With `$DRIFTLESS_SHOTS` pictures before and after go
//! there. The project goes into `$DRIFTLESS_WORK` (about 8 GB; nothing is written into the test
//! data); a project there from an earlier run is used again, only its Stellen start anew. The
//! window draws with the graphics card (a software rasterizer only without one); the touch-up
//! itself runs on the CPU. Run with
//!
//! ```text
//! DRIFTLESS_TESTDATA=… DRIFTLESS_WORK=… DRIFTLESS_SHOTS=… cargo test -p driftless-app \
//!     --release --test touchup_real -- --ignored --nocapture
//! ```
//!
//! On Linux the peak memory of each action is its own (`/proc/self/clear_refs`), the bytes read
//! come from `/proc/self/io`, and „Bilder neu“ variant 0 runs with the image bags out of the page
//! cache (`posix_fadvise`), the other actions with them in it; elsewhere these are left out.
//!
//! `painting_here` measures only the painting, on the surface in `$DRIFTLESS_SURFACE` with the
//! wall `$DRIFTLESS_WALL` that the run above wrote (`wand-buero.toml`), in a small project it
//! makes in `$DRIFTLESS_WORK`: for the GPU of another machine, without any scan.

use driftless_app::pick::{project, relative_to};
use driftless_app::project::{canonical, MergePick, Project, SurfaceSettings};
use driftless_app::render::headless_device;
use driftless_app::render::mesh::MeshVertex;
use driftless_app::ui::tools::{self, Tool};
use driftless_app::ui::{brush, text, Action, DriftlessApp, ScanNode, Selection, Step, UiMessage};
use driftless_export::mesh::{read_mesh, Mesh};
use driftless_io::camera::Camera;
use driftless_io::images::{decode_jpeg, for_each_image_where, index_images, RgbImage, Wanted};
use driftless_io::progress::Progress;
use driftless_map::surface::edit::{
    dab_faces, dab_normal, read_patches, select_faces, write_patches, FaceBvh, SelectDab,
    SurfacePatch, WeldedSurface, MAX_HOLE_AREA, PATCHES_FILE,
};
use driftless_map::surface::TextureSource;
use driftless_steps::map::peak_rss_bytes;
use driftless_steps::marks::{DabInput, EditStep, Marks, SmoothLevel, MARKS_FILE};
use driftless_steps::surface::texture_sources;
use driftless_steps::touchup::{edited_pages, TOUCHUP_DIR};
use driftless_traj::{interpolate_pose, Pose};
use eframe::egui::{self, Modifiers, PointerButton};
use egui_kittest::wgpu::{create_render_state, default_wgpu_setup, WgpuTestRenderer};
use egui_kittest::Harness;
use nalgebra::{Matrix3, SymmetricEigen, Vector3};
use rayon::prelude::*;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

type H = Harness<'static, DriftlessApp>;

/// S1 (the reference of the model), S2 and S3 of the dark room, then the Büro B.
const SCANS: [&str; 4] = [
    "jmk6-2026-10-22/20261022133554",
    "jmk6-2026-10-22/20261022132520",
    "jmk6-2026-10-22/20261022133028",
    "jmk6-2026-10-20/20261020173822",
];

const HOUR: Duration = Duration::from_secs(3600);
/// The radius of the brush (its default, 20 cm).
const RADIUS: f64 = 0.2;
/// The dabs of a Stelle: rows this far from its middle along the wall's up …
const ROWS: [f64; 4] = [0.3, 0.1, -0.1, -0.3];
/// … each from this far left to as far right of it, a dab every quarter radius as the brush
/// lays them: four strokes across 1 × 1 m with the brush of 20 cm.
const ROW_HALF: f64 = 0.3;
const DAB_STEP: f64 = RADIUS / 4.0;
/// How far a dab may lie in front of or behind the plane of its wall (m): no object, no hole.
const DAB_OFF_PLANE: f64 = 0.05;
/// A point of a Stelle at least this far from its border is moved in full by the smoothing
/// (its ramp, `driftless-map` `smooth.rs`).
const RAMP: f64 = 0.10;
/// The cosine below which two faces meet at a crease: no seam to compare across
/// (`SEAM_MIN_COS` of the surface step).
const CREASE_COS: f64 = std::f64::consts::FRAC_1_SQRT_2;
/// A plain wall Stelle: no point farther than this from the plane of its points (m), and in the
/// mean (RMS) not farther than this.
const PLAIN_OFF: f64 = 0.015;
const PLAIN_RMS: f64 = 0.008;
/// Free in front of a wall Stelle (m): the camera of „Zur Stelle“ sees it.
const FREE_AHEAD: f64 = 2.0;
/// The body turns at most this between the poses around an image to get its rate
/// (`RATE_HALF_WINDOW` of `driftless-map` `color.rs`, s).
const RATE_HALF_WINDOW: f64 = 0.05;
/// The cone of „Bilder neu“ assumes a pixel spans at most this many times the angle of the one
/// at the centre (`CONE_MARGIN` of `retexture.rs`).
const CONE_MARGIN: f64 = 3.0;
/// Pages of the surface decoded at once for the old texture (`PAGES_AT_ONCE`).
const PAGES_AT_ONCE: usize = 4;

// ---------------------------------------------------------------------------------------------
// Small things
// ---------------------------------------------------------------------------------------------

fn gb(bytes: u64) -> f64 {
    bytes as f64 / 1e9
}

fn mb(bytes: u64) -> f64 {
    bytes as f64 / 1e6
}

fn json(path: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(path).unwrap())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

fn errors(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .filter_map(|m| match m {
            UiMessage::Error(e) => Some(e.clone()),
            _ => None,
        })
        .collect()
}

fn said(app: &DriftlessApp) -> Vec<String> {
    app.state()
        .messages
        .iter()
        .map(|m| m.text().to_string())
        .collect()
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    scale(a, 1.0 / norm(a))
}

/// The value at `q` (0 to 1) of `v`, sorted in place; NaN if empty.
fn quantile(v: &mut [f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * q).round() as usize]
}

fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

fn rms(v: &[f64]) -> f64 {
    if v.is_empty() {
        f64::NAN
    } else {
        (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt()
    }
}

// ---------------------------------------------------------------------------------------------
// Memory, reading and the page cache (Linux)
// ---------------------------------------------------------------------------------------------

/// The value of `key` in a `/proc/self` file of `key: value` lines; `None` elsewhere.
fn proc_value(file: &str, key: &str) -> Option<u64> {
    let text = fs::read_to_string(file).ok()?;
    let line = text.lines().find(|l| l.starts_with(key))?;
    line[key.len()..].split_whitespace().next()?.parse().ok()
}

/// Bytes read so far by this process (from the page cache too).
fn read_bytes() -> Option<u64> {
    proc_value("/proc/self/io", "rchar:")
}

/// The memory the process holds now and its peak since [`reset_peak`].
fn rss_now() -> Option<u64> {
    proc_value("/proc/self/status", "VmRSS:").map(|kb| kb * 1024)
}

fn rss_peak() -> Option<u64> {
    proc_value("/proc/self/status", "VmHWM:")
        .map(|kb| kb * 1024)
        .or_else(peak_rss_bytes)
}

/// Starts the peak memory anew at what the process holds now.
fn reset_peak() {
    let _ = fs::write("/proc/self/clear_refs", "5");
}

/// Takes the file out of the page cache, so that the next read comes from the disk.
#[cfg(target_os = "linux")]
fn evict(path: &Path) -> bool {
    use std::os::fd::AsRawFd;
    let Ok(file) = fs::File::open(path) else {
        return false;
    };
    // SAFETY: the descriptor is open for the call; advice changes no memory of ours.
    unsafe { libc::posix_fadvise(file.as_raw_fd(), 0, 0, libc::POSIX_FADV_DONTNEED) == 0 }
}

#[cfg(not(target_os = "linux"))]
fn evict(_path: &Path) -> bool {
    false
}

/// Reads the file once from start to end; its bytes and the seconds it took.
fn read_through(path: &Path) -> (u64, f64) {
    let t = Instant::now();
    let mut file = fs::File::open(path).unwrap();
    let mut buffer = vec![0u8; 8 << 20];
    let mut total = 0u64;
    loop {
        let n = file.read(&mut buffer).unwrap();
        if n == 0 {
            break;
        }
        total += n as u64;
    }
    (total, t.elapsed().as_secs_f64())
}

/// The image bags of the camera images of `sources`.
fn bags(sources: &[TextureSource]) -> Vec<PathBuf> {
    sources
        .iter()
        .flat_map(|s| s.images.bags.iter().cloned())
        .collect()
}

// ---------------------------------------------------------------------------------------------
// The project
// ---------------------------------------------------------------------------------------------

/// What a step does, with the scan it works on.
fn label(action: &Action) -> String {
    let scan = |p: &Path| {
        p.components()
            .map(|c| c.as_os_str().to_string_lossy().into_owned())
            .find(|c| c.starts_with("2026"))
            .unwrap_or_default()
    };
    match action {
        Action::RunSlam(id) => format!("Laufbahn {id}"),
        Action::RunMap(run) => format!("Karte {}", scan(run)),
        Action::RunSurface(map) => format!("Oberfläche {}", scan(map)),
        Action::RunMerge { .. } => "Modell zusammenführen".into(),
        Action::RunModelSurface { .. } => "Oberfläche Modell".into(),
        other => format!("{other:?}"),
    }
}

/// Ticks the window without a GPU until no step, none queued and no background work is left;
/// prints each step that ran with its seconds.
fn settle(app: &mut DriftlessApp) {
    let deadline = Instant::now() + HOUR;
    let mut current: Option<(Action, Instant)> = None;
    loop {
        app.tick();
        let running = app.state().running.clone();
        if current.as_ref().map(|c| &c.0) != running.as_ref() {
            if let Some((a, t)) = current.take() {
                eprintln!(
                    "  {}: {:.1} s (Spitze bisher {:.2} GB)",
                    label(&a),
                    t.elapsed().as_secs_f64(),
                    peak_rss_bytes().map_or(f64::NAN, gb)
                );
            }
            current = running.map(|a| (a, Instant::now()));
        }
        if current.is_none() && app.state().queue.is_empty() && !app.wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn scan<'a>(app: &'a DriftlessApp, id: &str) -> &'a ScanNode {
    app.state()
        .scans
        .iter()
        .find(|s| s.id == id)
        .unwrap_or_else(|| panic!("scan {id} not in the tree"))
}

fn id_of(rel: &str) -> &str {
    rel.rsplit('/').next().unwrap()
}

/// The surface folders of the project in `project` the measurement needs: of S1 and of B, and
/// of the model of S1–S3, with the model's folder.
struct Computed {
    s1_surface: PathBuf,
    buero_surface: PathBuf,
    model: PathBuf,
    model_surface: PathBuf,
}

/// What the project has of [`Computed`], if all of it.
fn computed(app: &DriftlessApp, ids: &[&str]) -> Option<Computed> {
    let surface_of = |id: &str| {
        let node = app.state().scans.iter().find(|s| s.id == id)?;
        let map = node.runs.first()?.maps.first()?;
        Some(map.surfaces.first()?.dir.clone())
    };
    let model = app.state().models.first()?;
    Some(Computed {
        s1_surface: surface_of(ids[0])?,
        buero_surface: surface_of(ids[3])?,
        model: model.dir.clone(),
        model_surface: model.surfaces.first()?.dir.clone(),
    })
}

/// Opens the project in `project`, computing what it lacks first: a new one with the four scans,
/// „Alle rechnen“, the surfaces of S1 and B, the model of S1–S3 and its surface, all with the
/// defaults of the program. Takes the Stellen and their touch-up of an earlier run away.
fn prepare_project(data: &Path, project: &Path) -> Computed {
    let ids: Vec<&str> = SCANS.iter().map(|s| id_of(s)).collect();
    let mut app = DriftlessApp::new(Arc::new(|| {}));
    if project.join("project.toml").is_file() {
        eprintln!("Projekt {} von einem früheren Lauf", project.display());
        app.apply(Action::OpenProject(project.to_path_buf()));
    } else {
        app.apply(Action::NewProject {
            dir: project.to_path_buf(),
            name: "Abnahme".into(),
        });
        settle(&mut app);
        for s in SCANS {
            app.apply(Action::AddScan(data.join(s)));
            settle(&mut app);
        }
        assert_eq!(errors(&app), Vec::<String>::new());
        eprintln!("Alle rechnen (Laufbahn und Karte):");
        let t = Instant::now();
        app.apply(Action::ComputeAllMaps);
        settle(&mut app);
        eprintln!("  zusammen {:.1} min", t.elapsed().as_secs_f64() / 60.0);
    }
    settle(&mut app);
    assert_eq!(errors(&app), Vec::<String>::new());
    for id in [ids[0], ids[3]] {
        let map = &scan(&app, id).runs[0].maps[0];
        if map.surfaces.is_empty() {
            eprintln!("Oberfläche {id}:");
            app.apply(Action::RunSurface(map.dir.clone()));
            settle(&mut app);
            assert_eq!(errors(&app), Vec::<String>::new());
        }
    }
    if app.state().models.is_empty() {
        eprintln!("Modell aus S1–S3:");
        let picks: Vec<MergePick> = ids[..3]
            .iter()
            .map(|id| {
                let run = &scan(&app, id).runs[0];
                MergePick {
                    scan_id: id.to_string(),
                    run: run.dir.clone(),
                    map: run.maps[0].dir.clone(),
                }
            })
            .collect();
        app.apply(Action::RunMerge {
            picks,
            include_unsure: false,
        });
        settle(&mut app);
        assert_eq!(errors(&app), Vec::<String>::new());
    }
    if app.state().models[0].surfaces.is_empty() {
        let model = app.state().models[0].dir.clone();
        app.apply(Action::RunModelSurface { model });
        settle(&mut app);
        assert_eq!(errors(&app), Vec::<String>::new());
    }
    let c = computed(&app, &ids).expect("the surfaces of S1, B and the model");
    drop(app);
    // the Stellen start anew
    for (owner, surface) in [
        (c.model.clone(), c.model_surface.clone()),
        (
            c.buero_surface.parent().unwrap().to_path_buf(),
            c.buero_surface.clone(),
        ),
    ] {
        let _ = fs::remove_file(owner.join(MARKS_FILE));
        let _ = fs::remove_dir_all(surface.join(TOUCHUP_DIR));
    }
    c
}

/// What came out of the surface step in `dir`.
fn print_surface(name: &str, dir: &Path) -> Value {
    let report = json(&dir.join("oberflaeche.json"));
    let stats = &report["stats"];
    let glb = fs::metadata(dir.join("oberflaeche.glb")).unwrap().len();
    eprintln!(
        "{name}: {} Dreiecke ({} ohne Foto), {} MB glb, gerechnet in {:.1} s, Spitze {:.2} GB",
        stats["faces_final"],
        stats["texture"]["faces_untextured"],
        glb / 1_000_000,
        report["seconds_total"].as_f64().unwrap_or(f64::NAN),
        gb(report["peak_rss_bytes"].as_u64().unwrap_or(0)),
    );
    json!({
        "dreiecke": stats["faces_final"],
        "ohne_foto": stats["texture"]["faces_untextured"],
        "glb_bytes": glb,
        "sekunden": report["seconds_total"],
    })
}

// ---------------------------------------------------------------------------------------------
// The surface as the touch-up sees it
// ---------------------------------------------------------------------------------------------

/// The faces around every welded vertex.
struct VertexFaces {
    start: Vec<u32>,
    faces: Vec<u32>,
}

impl VertexFaces {
    fn new(s: &WeldedSurface) -> VertexFaces {
        let n = s.positions().len();
        let mut start = vec![0u32; n + 1];
        for tri in s.faces() {
            for &v in tri {
                start[v as usize + 1] += 1;
            }
        }
        for v in 0..n {
            start[v + 1] += start[v];
        }
        let mut fill = start[..n].to_vec();
        let mut faces = vec![0u32; start[n] as usize];
        for (f, tri) in s.faces().iter().enumerate() {
            for &v in tri {
                faces[fill[v as usize] as usize] = f as u32;
                fill[v as usize] += 1;
            }
        }
        VertexFaces { start, faces }
    }

    fn of(&self, v: u32) -> &[u32] {
        &self.faces[self.start[v as usize] as usize..self.start[v as usize + 1] as usize]
    }
}

/// The computed surface of a surface folder, welded, with its tree and its vertices' faces.
struct Geometry {
    mesh: Mesh,
    s: WeldedSurface,
    bvh: FaceBvh,
    vf: VertexFaces,
}

impl Geometry {
    fn read(dir: &Path) -> Geometry {
        let mesh = read_mesh(&dir.join("oberflaeche.glb"), &Progress::new()).unwrap();
        let s = WeldedSurface::from_mesh(&mesh);
        let bvh = FaceBvh::build(&s);
        let vf = VertexFaces::new(&s);
        Geometry { mesh, s, bvh, vf }
    }

    fn area(&self, faces: &[u32]) -> f64 {
        faces
            .iter()
            .map(|&f| norm(self.s.face_cross(f)) / 2.0)
            .sum()
    }

    /// The faces sharing a vertex with `faces` (sorted), and those.
    fn grow(&self, faces: &[u32]) -> Vec<u32> {
        let mut out: Vec<u32> = faces
            .iter()
            .flat_map(|&f| self.s.faces()[f as usize])
            .flat_map(|v| self.vf.of(v).iter().copied())
            .collect();
        out.extend_from_slice(faces);
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The faces within three rounds around the Stelle `faces` (sorted), without it: whose old
    /// texture „Bilder neu“ reads (`RING_ROUNDS`).
    fn ring(&self, faces: &[u32]) -> Vec<u32> {
        let mut all = faces.to_vec();
        for _ in 0..3 {
            all = self.grow(&all);
        }
        all.retain(|f| faces.binary_search(f).is_err());
        all
    }

    /// The other faces on the edge `a`–`b`.
    fn across(&self, f: u32, a: u32, b: u32) -> Vec<u32> {
        let near_b = self.vf.of(b);
        self.vf
            .of(a)
            .iter()
            .copied()
            .filter(|&g| g != f && near_b.contains(&g))
            .collect()
    }

    /// The texture coordinates of the face `f` on the page it was computed on, and whether it
    /// has a texture there (a gray face has one point for all three corners).
    fn computed_uvs(&self, f: u32) -> Option<(u32, [[f32; 2]; 3])> {
        let (p, t) = self.s.face_origin(f);
        let page = &self.mesh.pages[p as usize];
        page.image.as_ref()?;
        if page.uvs.len() != page.positions.len() {
            return None;
        }
        let uvs = page.indices[t as usize].map(|c| page.uvs[c as usize]);
        (!gray(uvs)).then_some((p, uvs))
    }

    /// How rough the surface is at the vertices `verts` that `keep` says, at the places
    /// `place` gives: the RMS of how far each lies from the mean of its neighbours along `n`
    /// (mm). The noise, not the shape: a plane or a gentle bow is 0.
    fn roughness(
        &self,
        verts: &[u32],
        keep: &[bool],
        place: &dyn Fn(u32) -> [f64; 3],
        n: [f64; 3],
    ) -> f64 {
        let offsets: Vec<f64> = verts
            .iter()
            .zip(keep)
            .filter(|(_, &k)| k)
            .map(|(&v, _)| {
                let mut around: Vec<u32> = self
                    .vf
                    .of(v)
                    .iter()
                    .flat_map(|&f| self.s.faces()[f as usize])
                    .filter(|&w| w != v)
                    .collect();
                around.sort_unstable();
                around.dedup();
                let sum = around.iter().fold([0.0; 3], |s, &w| add(s, place(w)));
                let mean = scale(sum, 1.0 / around.len().max(1) as f64);
                dot(sub(place(v), mean), n) * 1000.0
            })
            .collect();
        rms(&offsets)
    }
}

fn gray(uvs: [[f32; 2]; 3]) -> bool {
    uvs[0] == uvs[1] && uvs[1] == uvs[2]
}

/// The plane through `points` that fits them best: its centre, normal and the three spreads
/// (the eigenvalues of their covariance, smallest first).
fn plane(points: &[[f64; 3]]) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let n = points.len() as f64;
    let c = scale(points.iter().fold([0.0; 3], |s, p| add(s, *p)), 1.0 / n);
    let mut m = Matrix3::zeros();
    for p in points {
        let d = Vector3::from(sub(*p, c));
        m += d * d.transpose();
    }
    m /= n;
    let eig = SymmetricEigen::new(m);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j]));
    let v = eig.eigenvectors.column(order[0]);
    (c, [v[0], v[1], v[2]], order.map(|i| eig.eigenvalues[i]))
}

// ---------------------------------------------------------------------------------------------
// The wall Stelle
// ---------------------------------------------------------------------------------------------

/// A wall Stelle of about 1 m², painted with fixed dabs.
#[derive(Clone)]
struct Wall {
    /// Its middle on the surface, the wall's normal (towards the room) and two directions in it.
    centre: [f64; 3],
    normal: [f64; 3],
    across: [f64; 3],
    up: [f64; 3],
    dabs: Vec<DabInput>,
    /// The faces it covers, as the touch-up selects them, and their area.
    faces: Vec<u32>,
    area: f64,
    /// Distance of its points to their plane (m): RMS and largest; and their roughness
    /// ([`Geometry::roughness`], mm).
    rms: f64,
    max_off: f64,
    rough: f64,
    /// How far it was seen from.
    distance: f64,
}

impl Wall {
    fn select_dabs(&self) -> Vec<SelectDab> {
        self.dabs.iter().map(select_dab).collect()
    }
}

fn select_dab(d: &DabInput) -> SelectDab {
    SelectDab {
        centre: d.centre,
        normal: d.normal,
        radius: f64::from(d.radius),
        erase: d.erase,
    }
}

/// Where the device stood: twelve places along every trajectory of `sources`.
fn eyes(sources: &[TextureSource]) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    for s in sources {
        let n = s.poses.len();
        for k in 0..12 {
            let p = &s.poses[(k * n / 12).min(n.saturating_sub(1))];
            out.push([p.p.x, p.p.y, p.p.z]);
        }
    }
    out
}

/// The dabs of the Stelle around `centre` on the wall with `normal`, row by row as four
/// strokes go (see [`ROWS`]), each where a ray from in front of the wall meets it; `None` if
/// one misses or meets something off the wall (an object, a hole, a corner).
fn wall_dabs(
    g: &Geometry,
    centre: [f64; 3],
    normal: [f64; 3],
    across: [f64; 3],
    up: [f64; 3],
) -> Option<Vec<DabInput>> {
    let steps = (2.0 * ROW_HALF / DAB_STEP).round() as i32;
    let mut dabs = Vec::new();
    for (r, &row) in ROWS.iter().enumerate() {
        for k in 0..=steps {
            let k = if r % 2 == 0 { k } else { steps - k };
            let a = -ROW_HALF + f64::from(k) * DAB_STEP;
            let target = add(add(centre, scale(across, a)), scale(up, row));
            let from = add(target, scale(normal, 0.4));
            let hit = g.bvh.raycast(&g.s, from, scale(normal, -1.0), None)?;
            if (hit.t - 0.4).abs() > DAB_OFF_PLANE || dot(g.s.face_normal(hit.face), normal) < 0.8 {
                return None;
            }
            dabs.push(DabInput {
                centre: hit.pos,
                normal: dab_normal(&g.s, &g.bvh, &hit, RADIUS),
                radius: RADIUS as f32,
                erase: false,
            });
        }
    }
    Some(dabs)
}

/// The plain wall Stellen of about 1 m² seen from the places of the device (rays out level in
/// 16 directions, hitting a wall 1 to 3.5 m away nearly head on): every point within
/// [`PLAIN_OFF`] of their plane and [`PLAIN_RMS`] in the mean (no object on the wall or in front
/// of it), [`FREE_AHEAD`] free in front, the roughest first ([`Geometry::roughness`]: the noise,
/// not the shape).
fn find_walls(g: &Geometry, eyes: &[[f64; 3]]) -> Vec<Wall> {
    let mut found: Vec<(usize, Wall)> = Vec::new();
    let mut order = 0;
    for &eye in eyes {
        for k in 0..16 {
            order += 1;
            let a = f64::from(k) * std::f64::consts::TAU / 16.0;
            let dir = [a.cos(), a.sin(), 0.0];
            let Some(hit) = g.bvh.raycast(&g.s, eye, dir, None) else {
                continue;
            };
            let n_face = g.s.face_normal(hit.face);
            if !(1.0..=3.5).contains(&hit.t) || n_face[2].abs() > 0.2 || dot(n_face, dir) > -0.7 {
                continue;
            }
            let n = dab_normal(&g.s, &g.bvh, &hit, RADIUS).map(f64::from);
            if n[2].abs() > 0.15 || dot(n, dir) > -0.7 {
                continue;
            }
            let across = unit(cross([0.0, 0.0, 1.0], n));
            let up = cross(n, across);
            if found
                .iter()
                .any(|(_, w)| norm(sub(w.centre, hit.pos)) < 0.8)
            {
                continue;
            }
            let Some(dabs) = wall_dabs(g, hit.pos, n, across, up) else {
                continue;
            };
            let select: Vec<SelectDab> = dabs.iter().map(select_dab).collect();
            let faces = select_faces(&g.s, &g.bvh, &select);
            let area = g.area(&faces);
            if !(0.8..=1.3).contains(&area) {
                continue;
            }
            let verts = vertices_of(&g.s, &faces);
            let points = verts
                .iter()
                .map(|&v| g.s.positions()[v as usize])
                .collect::<Vec<_>>();
            let (pc, pn, spread) = plane(&points);
            let rms = spread[0].max(0.0).sqrt();
            let max_off = points
                .iter()
                .map(|p| dot(sub(*p, pc), pn).abs())
                .fold(0.0, f64::max);
            // a plain wall: noise, no object on it or in front of it (a ladder, a shelf)
            let plain = rms <= PLAIN_RMS
                && max_off <= PLAIN_OFF
                && spread[0] <= 0.05 * (spread[0] + spread[1] + spread[2]) / 3.0;
            if !plain || pn[2].abs() > 0.1 {
                continue;
            }
            // room in front of it: the view at it is free
            let ahead = g.bvh.raycast(&g.s, add(hit.pos, scale(n, 0.05)), n, None);
            if ahead.is_some_and(|a| a.t < FREE_AHEAD) {
                continue;
            }
            let place = |v: u32| g.s.positions()[v as usize];
            let rough = g.roughness(&verts, &vec![true; verts.len()], &place, pn);
            found.push((
                order,
                Wall {
                    centre: hit.pos,
                    normal: n,
                    across,
                    up,
                    dabs,
                    faces,
                    area,
                    rms,
                    max_off,
                    rough,
                    distance: hit.t,
                },
            ));
        }
    }
    found.sort_by(|a, b| b.1.rough.total_cmp(&a.1.rough).then(a.0.cmp(&b.0)));
    found.into_iter().map(|(_, w)| w).collect()
}

fn vertices_of(s: &WeldedSurface, faces: &[u32]) -> Vec<u32> {
    let mut v: Vec<u32> = faces.iter().flat_map(|&f| s.faces()[f as usize]).collect();
    v.sort_unstable();
    v.dedup();
    v
}

// ---------------------------------------------------------------------------------------------
// The holes of a selection (Ruling 12), as `select_faces` finds them
// ---------------------------------------------------------------------------------------------

/// The holes a Stelle encloses (faces no dab took, connected over edges, touching the Stelle
/// and nothing else): those `select_faces` closes (up to 25 cm²) and those it leaves open.
#[derive(Default)]
struct Holes {
    closed: Vec<f64>,
    open: Vec<f64>,
    /// The selection made here with the closed holes is the one of `select_faces`.
    same: bool,
}

/// [`Holes`] of the dabs `dabs`, found as `driftless-map` `select.rs` finds them.
fn holes(g: &Geometry, dabs: &[SelectDab]) -> Holes {
    let s = &g.s;
    let near: Vec<Vec<u32>> = dabs
        .par_iter()
        .map(|d| g.bvh.faces_within(s, d.centre, d.radius))
        .collect();
    let took: Vec<Vec<u32>> = dabs.par_iter().map(|d| dab_faces(s, &g.bvh, d)).collect();
    let mut by_face: Vec<(u32, usize)> = took
        .iter()
        .enumerate()
        .flat_map(|(d, t)| t.iter().map(move |&f| (f, d)))
        .collect();
    by_face.sort_unstable();
    let (mut selected, mut taken) = (Vec::new(), Vec::new());
    for same in by_face.chunk_by(|x, y| x.0 == y.0) {
        let (face, last) = same[same.len() - 1];
        taken.push(face);
        if !dabs[last].erase {
            selected.push(face);
        }
    }
    let mut holding: Vec<(u32, usize)> = near
        .iter()
        .enumerate()
        .flat_map(|(d, n)| n.iter().map(move |&f| (f, d)))
        .collect();
    holding.sort_unstable();
    let candidates: Vec<u32> = holding
        .chunk_by(|x, y| x.0 == y.0)
        .filter_map(|same| {
            let (face, last) = same[same.len() - 1];
            (!dabs[last].erase && taken.binary_search(&face).is_err()).then_some(face)
        })
        .collect();
    let candidate = |f: u32| candidates.binary_search(&f).ok();
    let longest = |f: u32| {
        let [a, b, c] = s.face_corners(f);
        [(a, b), (b, c), (c, a)]
            .map(|(p, q)| norm(sub(p, q)))
            .into_iter()
            .fold(0.0, f64::max)
    };
    let mut around: Vec<u32> = dabs
        .iter()
        .zip(&near)
        .filter(|(d, _)| !d.erase)
        .flat_map(|(d, n)| {
            let reach = n
                .iter()
                .filter(|&&f| candidate(f).is_some())
                .map(|&f| longest(f))
                .fold(f64::NEG_INFINITY, f64::max);
            if reach == f64::NEG_INFINITY {
                Vec::new()
            } else {
                g.bvh.faces_within(s, d.centre, d.radius + reach + 1e-6)
            }
        })
        .collect();
    around.sort_unstable();
    around.dedup();
    let mut edges: Vec<(u32, u32, u32)> = Vec::new();
    for &f in &around {
        let t = s.faces()[f as usize];
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if a != b {
                edges.push((a.min(b), a.max(b), f));
            }
        }
    }
    edges.sort_unstable();
    let mut parent: Vec<usize> = (0..candidates.len()).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let mut members = same.iter().filter_map(|e| candidate(e.2));
        if let Some(first) = members.next() {
            for other in members {
                let (a, b) = (find(&mut parent, first), find(&mut parent, other));
                parent[a.max(b)] = a.min(b);
            }
        }
    }
    let mut touches = vec![false; candidates.len()];
    let mut leaks = vec![false; candidates.len()];
    for same in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let mut groups: Vec<usize> = same
            .iter()
            .filter_map(|e| candidate(e.2))
            .map(|i| find(&mut parent, i))
            .collect();
        if groups.is_empty() {
            continue;
        }
        groups.sort_unstable();
        groups.dedup();
        let (mut stelle, mut other) = (false, same.len() == 1);
        for e in same.iter().filter(|e| candidate(e.2).is_none()) {
            if selected.binary_search(&e.2).is_ok() {
                stelle = true;
            } else {
                other = true;
            }
        }
        for gr in groups {
            touches[gr] |= stelle;
            leaks[gr] |= other;
        }
    }
    let mut area = vec![0.0f64; candidates.len()];
    for (i, &f) in candidates.iter().enumerate() {
        let r = find(&mut parent, i);
        area[r] += norm(s.face_cross(f)) / 2.0;
    }
    let mut out = Holes::default();
    let mut filled = Vec::new();
    for (i, &f) in candidates.iter().enumerate() {
        let r = find(&mut parent, i);
        let enclosed = touches[r] && !leaks[r];
        if enclosed && area[r] <= MAX_HOLE_AREA {
            filled.push(f);
        }
        if enclosed && r == i {
            if area[r] <= MAX_HOLE_AREA {
                out.closed.push(area[r]);
            } else {
                out.open.push(area[r]);
            }
        }
    }
    selected.extend(filled);
    selected.sort_unstable();
    out.same = selected == select_faces(s, &g.bvh, dabs);
    out
}

// ---------------------------------------------------------------------------------------------
// What the touch-up did to the Stelle
// ---------------------------------------------------------------------------------------------

/// The vertices of a Stelle: its border (shared with a face outside: they never move), the
/// ring next to it inside, and those at least [`RAMP`] from the border.
struct Shape {
    vertices: Vec<u32>,
    border: Vec<bool>,
    first_ring: Vec<bool>,
    inner: Vec<bool>,
}

fn shape(g: &Geometry, faces: &[u32]) -> Shape {
    let vertices = vertices_of(&g.s, faces);
    let border: Vec<bool> = vertices
        .iter()
        .map(|&v| g.vf.of(v).iter().any(|f| faces.binary_search(f).is_err()))
        .collect();
    let is_border = |v: u32| vertices.binary_search(&v).is_ok_and(|i| border[i]);
    let first_ring: Vec<bool> = vertices
        .iter()
        .zip(&border)
        .map(|(&v, &b)| {
            !b && g.vf.of(v).iter().any(|&f| {
                faces.binary_search(&f).is_ok()
                    && g.s.faces()[f as usize].iter().any(|&w| is_border(w))
            })
        })
        .collect();
    let rim: Vec<[f64; 3]> = vertices
        .iter()
        .zip(&border)
        .filter(|(_, &b)| b)
        .map(|(&v, _)| g.s.positions()[v as usize])
        .collect();
    let inner = vertices
        .iter()
        .map(|&v| {
            let p = g.s.positions()[v as usize];
            rim.iter().all(|&q| norm(sub(p, q)) >= RAMP)
        })
        .collect();
    Shape {
        vertices,
        border,
        first_ring,
        inner,
    }
}

/// How smooth the Stelle is before and after the moves `moved` (welded vertex, new place).
#[derive(Default)]
struct Smoothness {
    /// Distance of the points to the plane of the points before (mm): all, and those the
    /// smoothing moves in full ([`RAMP`] from the border).
    rms_before: f64,
    rms_after: f64,
    inner_before: f64,
    inner_after: f64,
    /// The largest move of a point next to the border, and of 95 % of them (mm); the border
    /// itself (mm, 0 if it stayed).
    jump_max: f64,
    jump_p95: f64,
    border_max: f64,
    /// The height step across the edges from the border to the points next to it, along the
    /// plane's normal (mm): its RMS before and after, its largest after, and 95 % of its changes.
    step_before: f64,
    step_after: f64,
    step_after_max: f64,
    step_change_p95: f64,
    /// The roughness of the points moved in full before and after ([`Geometry::roughness`], mm).
    rough_before: f64,
    rough_after: f64,
    /// The largest move of any point (mm), and how many moved.
    moved_max: f64,
    moved: usize,
}

fn smoothness(g: &Geometry, faces: &[u32], sh: &Shape, moved: &[(u32, [f64; 3])]) -> Smoothness {
    let moves: BTreeMap<u32, [f64; 3]> = moved.iter().copied().collect();
    let before: Vec<[f64; 3]> = sh
        .vertices
        .iter()
        .map(|&v| g.s.positions()[v as usize])
        .collect();
    let after: Vec<[f64; 3]> = sh
        .vertices
        .iter()
        .zip(&before)
        .map(|(v, &p)| moves.get(v).copied().unwrap_or(p))
        .collect();
    let (c, n, _) = plane(&before);
    let off = |p: &[f64; 3]| dot(sub(*p, c), n) * 1000.0;
    let pick = |points: &[[f64; 3]], keep: &[bool]| -> Vec<f64> {
        points
            .iter()
            .zip(keep)
            .filter(|(_, &k)| k)
            .map(|(p, _)| off(p))
            .collect()
    };
    let all = vec![true; before.len()];
    let shift: Vec<f64> = before
        .iter()
        .zip(&after)
        .map(|(a, b)| norm(sub(*a, *b)) * 1000.0)
        .collect();
    let mut ring: Vec<f64> = shift
        .iter()
        .zip(&sh.first_ring)
        .filter(|(_, &r)| r)
        .map(|(d, _)| *d)
        .collect();
    let border = shift
        .iter()
        .zip(&sh.border)
        .filter(|(_, &b)| b)
        .map(|(d, _)| *d)
        .fold(0.0, f64::max);
    let place_before = |v: u32| g.s.positions()[v as usize];
    let place_after = |v: u32| {
        moves
            .get(&v)
            .copied()
            .unwrap_or(g.s.positions()[v as usize])
    };
    // the edges from a border point to a point next to it inside
    let index = |v: u32| sh.vertices.binary_search(&v).ok();
    let mut pairs: Vec<(u32, u32)> = Vec::new();
    for &f in faces {
        let t = g.s.faces()[f as usize];
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            for (rim, inside) in [(a, b), (b, a)] {
                if let (Some(i), Some(j)) = (index(rim), index(inside)) {
                    if sh.border[i] && sh.first_ring[j] {
                        pairs.push((rim, inside));
                    }
                }
            }
        }
    }
    pairs.sort_unstable();
    pairs.dedup();
    let step = |place: &dyn Fn(u32) -> [f64; 3]| -> Vec<f64> {
        pairs
            .iter()
            .map(|&(b, r)| dot(sub(place(r), place(b)), n) * 1000.0)
            .collect()
    };
    let (step_before, step_after) = (step(&place_before), step(&place_after));
    let mut change: Vec<f64> = step_before
        .iter()
        .zip(&step_after)
        .map(|(a, b)| (b - a).abs())
        .collect();
    Smoothness {
        step_before: rms(&step_before),
        step_after: rms(&step_after),
        step_after_max: step_after.iter().map(|h| h.abs()).fold(0.0, f64::max),
        step_change_p95: quantile(&mut change, 0.95),
        rough_before: g.roughness(&sh.vertices, &sh.inner, &place_before, n),
        rough_after: g.roughness(&sh.vertices, &sh.inner, &place_after, n),
        rms_before: rms(&pick(&before, &all)),
        rms_after: rms(&pick(&after, &all)),
        inner_before: rms(&pick(&before, &sh.inner)),
        inner_after: rms(&pick(&after, &sh.inner)),
        jump_max: ring.iter().copied().fold(0.0, f64::max),
        jump_p95: quantile(&mut ring, 0.95),
        border_max: border,
        moved_max: shift.iter().copied().fold(0.0, f64::max),
        moved: moves.len(),
    }
}

/// Where a face's texture is: on a page of the computed surface, or on page `page` of patch
/// `patch`, as triangle `t`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Page {
    Computed(u32),
    Patch(usize, usize),
}

/// The colour difference across the border of the images a step baked anew (mean over the
/// channels at ¼, ½ and ¾ of every edge between a face with a new texture and one that keeps its
/// own, in sRGB levels), before (both faces as computed) and after.
#[derive(Default)]
struct Colour {
    edges: usize,
    before: f64,
    after: f64,
    after_p90: f64,
    /// Faces with a new texture, and the pages they lie on.
    faces: usize,
    pages: usize,
}

/// The texture of face `f` and its texture coordinates after `patches` (the last patch holding
/// it, else the computed page).
fn texture_of(
    g: &Geometry,
    patches: &[SurfacePatch],
    held: &BTreeMap<u32, (usize, usize, usize)>,
    f: u32,
) -> Option<(Page, [[f32; 2]; 3])> {
    if let Some(&(q, p, t)) = held.get(&f) {
        let page = &patches[q].pages[p];
        return Some((
            Page::Patch(q, p),
            page.indices[t].map(|c| page.uvs[c as usize]),
        ));
    }
    let (p, uvs) = g.computed_uvs(f)?;
    Some((Page::Computed(p), uvs))
}

/// Where an edge is looked at in a texture: the page, and the texture coordinates of its two
/// ends there.
type Look = (Page, [f32; 2], [f32; 2]);

fn colour(g: &Geometry, patches: &[SurfacePatch]) -> Colour {
    let Some(last) = patches.last() else {
        return Colour::default();
    };
    let new = &last.removed_faces;
    // face → (patch, page, triangle), the later patch winning
    let mut held: BTreeMap<u32, (usize, usize, usize)> = BTreeMap::new();
    for (q, patch) in patches.iter().enumerate() {
        for (p, page) in patch.pages.iter().enumerate() {
            for (t, &f) in page.faces.iter().enumerate() {
                held.insert(f, (q, p, t));
            }
        }
    }
    let corners = g.s.faces();
    // (inside, outside, a, b) of every edge where a new texture meets an old one
    let mut edges: Vec<(u32, u32, u32, u32)> = Vec::new();
    for &f in new {
        let t = corners[f as usize];
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            for o in g.across(f, a, b) {
                if new.binary_search(&o).is_err()
                    && dot(g.s.face_normal(f), g.s.face_normal(o)) >= CREASE_COS
                {
                    edges.push((f, o, a, b));
                }
            }
        }
    }
    let mut needed: Vec<Page> = Vec::new();
    let mut looks: Vec<[Look; 4]> = Vec::new();
    let uv_of = |f: u32, uvs: [[f32; 2]; 3], v: u32| -> [f32; 2] {
        let k = corners[f as usize].iter().position(|&w| w == v).unwrap();
        uvs[k]
    };
    for &(f, o, a, b) in &edges {
        let (Some(inside), Some(outside), Some(old_in), Some((po, uo))) = (
            texture_of(g, patches, &held, f),
            texture_of(g, patches, &held, o),
            g.computed_uvs(f),
            g.computed_uvs(o),
        ) else {
            continue;
        };
        let look = [
            (inside.0, uv_of(f, inside.1, a), uv_of(f, inside.1, b)),
            (outside.0, uv_of(o, outside.1, a), uv_of(o, outside.1, b)),
            (
                Page::Computed(old_in.0),
                uv_of(f, old_in.1, a),
                uv_of(f, old_in.1, b),
            ),
            (Page::Computed(po), uv_of(o, uo, a), uv_of(o, uo, b)),
        ];
        needed.extend(look.iter().map(|l| l.0));
        looks.push(look);
    }
    needed.sort_unstable();
    needed.dedup();
    let images: BTreeMap<Page, RgbImage> = needed
        .par_iter()
        .map(|&p| {
            let bytes: &[u8] = match p {
                Page::Computed(i) => &g.mesh.pages[i as usize].image.as_ref().unwrap().bytes,
                Page::Patch(q, i) => &patches[q].pages[i].jpeg,
            };
            (p, decode_jpeg(bytes, 1).unwrap())
        })
        .collect();
    let sample = |p: Page, uv: [f32; 2]| -> [f64; 3] {
        let im = &images[&p];
        let (w, h) = (im.width as f64, im.height as f64);
        let x = (f64::from(uv[0]) * w - 0.5).clamp(0.0, w - 1.0);
        let y = (f64::from(uv[1]) * h - 0.5).clamp(0.0, h - 1.0);
        let (x0, y0) = (x.floor() as usize, y.floor() as usize);
        let (x1, y1) = (
            (x0 + 1).min(im.width as usize - 1),
            (y0 + 1).min(im.height as usize - 1),
        );
        let (fx, fy) = (x - x0 as f64, y - y0 as f64);
        let px =
            |x: usize, y: usize, c: usize| f64::from(im.data[(y * im.width as usize + x) * 3 + c]);
        [0, 1, 2].map(|c| {
            let top = px(x0, y0, c) * (1.0 - fx) + px(x1, y0, c) * fx;
            let bottom = px(x0, y1, c) * (1.0 - fx) + px(x1, y1, c) * fx;
            top * (1.0 - fy) + bottom * fy
        })
    };
    let lerp =
        |a: [f32; 2], b: [f32; 2], t: f32| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
    let differ = |x: [f64; 3], y: [f64; 3]| (0..3).map(|c| (x[c] - y[c]).abs()).sum::<f64>() / 3.0;
    let (mut before, mut after) = (Vec::new(), Vec::new());
    for look in &looks {
        for t in [0.25f32, 0.5, 0.75] {
            let at = |l: &Look| sample(l.0, lerp(l.1, l.2, t));
            after.push(differ(at(&look[0]), at(&look[1])));
            before.push(differ(at(&look[2]), at(&look[3])));
        }
    }
    let pages = last.pages.len();
    Colour {
        edges: looks.len(),
        before: mean(&before),
        after: mean(&after),
        after_p90: quantile(&mut after, 0.9),
        faces: new.len(),
        pages,
    }
}

/// Bytes the view uploads for the touched-up state of the surface in `dir` (the corners and
/// triangles of the pages it changes, the pages it adds with their textures and mipmaps).
fn upload_bytes(dir: &Path, g: &Geometry) -> Option<u64> {
    let (edits, added) = edited_pages(dir, &g.s, &g.mesh).ok()??;
    let vertex = std::mem::size_of::<MeshVertex>() as u64;
    let mut bytes = 0;
    for e in &edits {
        let page = &g.mesh.pages[e.page as usize];
        if e.positions.is_some() {
            bytes += page.positions.len() as u64 * vertex;
        }
        if !e.removed.is_empty() {
            bytes += (page.indices.len() - e.removed.len()) as u64 * 12;
        }
    }
    for page in &added {
        bytes += page.positions.len() as u64 * vertex + page.indices.len() as u64 * 12;
        if let Some(im) = &page.image {
            let rgb = decode_jpeg(&im.bytes, 1).ok()?;
            let (mut w, mut h) = (u64::from(rgb.width), u64::from(rgb.height));
            loop {
                bytes += w * h * 4;
                if w == 1 && h == 1 {
                    break;
                }
                w = (w / 2).max(1);
                h = (h / 2).max(1);
            }
        }
    }
    Some(bytes)
}

/// Seconds to decode the computed pages that hold `faces` with a texture, [`PAGES_AT_ONCE`] at
/// a time as „Bilder neu“ reads the old texture, and how many pages.
fn decode_pages(g: &Geometry, faces: &[u32]) -> (usize, f64) {
    let mut pages: Vec<u32> = faces
        .iter()
        .filter_map(|&f| g.computed_uvs(f).map(|(p, _)| p))
        .collect();
    pages.sort_unstable();
    pages.dedup();
    let t = Instant::now();
    for chunk in pages.chunks(PAGES_AT_ONCE) {
        let decoded: Vec<_> = chunk
            .par_iter()
            .map(|&p| decode_jpeg(&g.mesh.pages[p as usize].image.as_ref().unwrap().bytes, 1))
            .collect();
        assert!(decoded.iter().all(Result::is_ok));
    }
    (pages.len(), t.elapsed().as_secs_f64())
}

/// The rotation rate of the body around `t` (rad/s), as `driftless-map` `color.rs` takes it.
fn angular_rate(poses: &[Pose], t: f64) -> f64 {
    let (Some(first), Some(last)) = (poses.first(), poses.last()) else {
        return 0.0;
    };
    let (lo, hi) = (
        (t - RATE_HALF_WINDOW).max(first.t),
        (t + RATE_HALF_WINDOW).min(last.t),
    );
    match (interpolate_pose(poses, lo), interpolate_pose(poses, hi)) {
        (Some(a), Some(b)) if hi > lo => (a.rotation.inverse() * b.rotation).angle() / (hi - lo),
        _ => 0.0,
    }
}

/// How many images „Bilder neu“ reads the header of for the Stelle `faces`, and for how many of
/// them it builds a depth buffer, as `retexture.rs` `candidates` decides: the time of the image
/// on the trajectory, the device turning no faster than `max_angular_rate`, the Stelle within
/// `max_dist` and `rim_theta` of the camera's axis, a face of it facing the camera.
fn candidates(
    g: &Geometry,
    faces: &[u32],
    sources: &[TextureSource],
    ccfg: &driftless_map::ColorConfig,
) -> (u64, u64) {
    let centroid: Vec<[f64; 3]> = faces.iter().map(|&f| g.s.face_centroid(f)).collect();
    let normal: Vec<[f64; 3]> = faces.iter().map(|&f| g.s.face_normal(f)).collect();
    let corners: Vec<[f64; 3]> = faces.iter().flat_map(|&f| g.s.face_corners(f)).collect();
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in &corners {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let centre = [0, 1, 2].map(|k| (lo[k] + hi[k]) / 2.0);
    let radius = corners
        .iter()
        .map(|p| norm(sub(*p, centre)))
        .fold(0.0, f64::max);
    let (mut images, mut found) = (0u64, 0u64);
    for src in sources {
        let read = for_each_image_where(
            &src.images.bags,
            &src.images.topics,
            1,
            |h| {
                images += 1;
                let Some(cam) = src.cams.iter().find(|c| c.camera.name == h.camera) else {
                    return Wanted::No;
                };
                let t = h.stamp + ccfg.time_offset;
                let Some(body) = interpolate_pose(&src.poses, t) else {
                    return Wanted::No;
                };
                if angular_rate(&src.poses, t) > ccfg.max_angular_rate {
                    return Wanted::No;
                }
                let world_from_cam = body * cam.cam_in_body;
                let v = world_from_cam.translation.vector;
                let pos = [v.x, v.y, v.z];
                let to = sub(centre, pos);
                let dist = norm(to);
                if dist - radius > ccfg.max_dist + 1e-6 {
                    return Wanted::No;
                }
                if dist > radius {
                    let axis = world_from_cam.rotation * Vector3::z();
                    let theta = (dot([axis.x, axis.y, axis.z], to) / dist)
                        .clamp(-1.0, 1.0)
                        .acos();
                    let nearest = theta - (radius / dist).asin();
                    if nearest >= ccfg.rim_theta + 1e-6 || nearest > cam.camera.max_theta + 1e-6 {
                        return Wanted::No;
                    }
                }
                let facing = centroid.iter().zip(&normal).any(|(c, n)| {
                    let to_cam = sub(pos, *c);
                    dot(*n, to_cam) > -1e-6 * norm(to_cam)
                });
                found += u64::from(facing);
                Wanted::No
            },
            |_| true,
        );
        read.unwrap();
    }
    (images, found)
}

// ---------------------------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------------------------

/// The window with a GPU on the project in `project`, with GPU timestamps where the adapter has
/// them, and the name of the adapter; `None` without one. It draws with the graphics card, a
/// software rasterizer only where there is no other: the harness of the snapshot tests prefers
/// the software one (the same pixels everywhere), a measurement must not.
fn open_window(project: &Path) -> Option<(H, String)> {
    headless_device()?;
    let setup = match default_wgpu_setup() {
        egui_wgpu::WgpuSetup::CreateNew(mut s) => {
            s.native_adapter_selector = Some(Arc::new(|adapters, _surface| {
                use egui_wgpu::wgpu::DeviceType;
                let rank = |a: &&egui_wgpu::wgpu::Adapter| match a.get_info().device_type {
                    DeviceType::DiscreteGpu => 0,
                    DeviceType::IntegratedGpu => 1,
                    DeviceType::VirtualGpu | DeviceType::Other => 2,
                    DeviceType::Cpu => 3,
                };
                adapters
                    .iter()
                    .min_by_key(rank)
                    .cloned()
                    .ok_or_else(|| "kein Grafikadapter".to_string())
            }));
            egui_wgpu::WgpuSetup::CreateNew(driftless_app::render::with_timestamps(s))
        }
        other => other,
    };
    let rs = create_render_state(setup, egui_wgpu::RendererOptions::PREDICTABLE);
    let info = rs.adapter.get_info();
    let mut app = DriftlessApp::new(Arc::new(|| {})).with_render_state(rs.clone());
    app.apply(Action::OpenProject(project.to_path_buf()));
    let mut h = Harness::builder()
        .with_size([1400.0, 900.0])
        .with_pixels_per_point(1.0)
        .with_step_dt(1.0 / 60.0)
        .renderer(WgpuTestRenderer::from_render_state(rs))
        .build_ui_state(|ui, app: &mut DriftlessApp| app.show(ui), app);
    frames_until_idle(&mut h);
    assert_eq!(errors(h.state()), Vec::<String>::new());
    Some((h, format!("{} ({:?})", info.name, info.backend)))
}

fn frames_until_idle(h: &mut H) {
    let deadline = Instant::now() + Duration::from_secs(600);
    loop {
        h.step();
        if h.state().state().job.is_none() && !h.state().wants_repaint_while_busy() {
            return;
        }
        assert!(Instant::now() < deadline, "work did not end");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Shows the surface in the folder `dir` in „Bilder“, uploaded; the seconds that took. `select`:
/// chosen in the field as a click does (with the trajectory of its run), else only loaded.
fn show_surface(h: &mut H, dir: &Path, select: bool) -> f64 {
    let t = Instant::now();
    h.state_mut().apply(Action::SetStep(Step::Bilder));
    h.step();
    h.state_mut().apply(if select {
        Action::Select(Selection::Surface(dir.to_path_buf()))
    } else {
        Action::LoadSurface(dir.to_path_buf())
    });
    let glb = dir.join("oberflaeche.glb");
    let deadline = Instant::now() + Duration::from_secs(600);
    loop {
        h.step();
        let shown = h
            .state()
            .state()
            .surface
            .as_ref()
            .is_some_and(|s| s.path == glb);
        if shown && !h.state().is_loading_surface() && !h.state().view().is_uploading_surface() {
            break;
        }
        assert!(Instant::now() < deadline, "surface not shown");
        std::thread::sleep(Duration::from_millis(5));
    }
    let seconds = t.elapsed().as_secs_f64();
    frames_until_idle(h);
    for _ in 0..3 {
        h.step();
    }
    assert_eq!(brush::blocked(h.state().state()), None);
    seconds
}

/// Where the absolute point `p` lies in the window (points).
fn on_screen(h: &H, p: [f64; 3]) -> Option<egui::Pos2> {
    let app = h.state();
    let view = app.layout().view;
    let origin = app.state().surface.as_ref()?.origin;
    let vp = app.view().camera().view_proj(view.width() / view.height());
    let q = project(&vp, relative_to(origin, p), [view.width(), view.height()])?;
    let pos = view.min + egui::vec2(q[0], q[1]);
    view.contains(pos).then_some(pos)
}

/// The camera straight at the wall from about 2 m.
fn look_at(h: &mut H, wall: &Wall) {
    let origin = h.state().state().surface.as_ref().unwrap().origin;
    let view = h.state().layout().view;
    let aspect = view.width() / view.height();
    let centre = relative_to(origin, wall.centre);
    let normal = wall.normal.map(|c| c as f32);
    h.state_mut()
        .view_mut()
        .camera_mut()
        .frame_facing(centre, normal, 1.0, aspect);
    for _ in 0..3 {
        h.step();
    }
}

/// Saves the window as `<dir>/<name>.png`, without the marks of the Stellen if `bare`.
fn shot(h: &mut H, dir: &Path, name: &str, bare: bool) {
    for _ in 0..3 {
        h.step();
    }
    if bare {
        h.state_mut().view_mut().set_marks(Vec::new());
        h.step();
    }
    let path = dir.join(format!("{name}.png"));
    h.render().expect("render").save(&path).expect("save png");
    eprintln!("  Bild {}", path.display());
}

/// Before and after of the Stelle `nr`: straight at it and, with `slant`, along the wall.
fn before_after(h: &mut H, dir: &Path, name: &str, nr: u32, slant: bool) {
    h.state_mut().apply(Action::FrameMark(nr));
    for _ in 0..3 {
        h.step();
    }
    let camera = h.state().view().camera().clone();
    let mut views = vec![("", camera.clone())];
    if slant {
        let mut along = camera;
        along.yaw += 65f32.to_radians();
        along.pitch -= 10f32.to_radians();
        views.push(("-schraeg", along));
    }
    for (view, camera) in views {
        for (state, after) in [("vorher", false), ("nachher", true)] {
            *h.state_mut().view_mut().camera_mut() = camera.clone();
            h.state_mut().apply(Action::ShowEdit(after));
            shot(h, dir, &format!("{name}{view}-{state}"), true);
        }
    }
    h.state_mut().apply(Action::ShowEdit(true));
    for _ in 0..3 {
        h.step();
    }
}

// ---------------------------------------------------------------------------------------------
// An action, timed
// ---------------------------------------------------------------------------------------------

/// A stretch of an action: what ran (the phase of the touch-up), how long, and how many bytes
/// the process read meanwhile.
struct Phase {
    name: String,
    seconds: f64,
    read: Option<u64>,
}

/// An action of the bar as it ran in the window.
struct Run {
    what: String,
    seconds: f64,
    phases: Vec<Phase>,
    /// The frame that took the touched-up state to the view (the slowest frame from the end of
    /// the touch-up on), ms.
    upload_frame_ms: f64,
    /// Memory before and the peak while it ran.
    base: Option<u64>,
    peak: Option<u64>,
    /// The messages it left in the status bar …
    messages: Vec<String>,
    /// … and in the protocol.
    protocol: Vec<String>,
}

/// The phase the window shows now: the touch-up's, „Bilder der Stelle suchen“ split into
/// reading the headers of the images and the visibility; `None` without a step.
fn phase_now(app: &DriftlessApp) -> Option<String> {
    let job = app.state().job.as_ref()?;
    if job.label != text::STEP_TOUCHUP {
        return Some(job.label.clone());
    }
    let p = &job.progress;
    Some(if p.phase.is_empty() {
        "Rohdaten und Laufbahn finden".to_string()
    } else if p.phase == "Bilder der Stelle suchen" {
        if p.total.is_some_and(|t| p.done < t) {
            "Bilder der Stelle suchen (Köpfe lesen)".to_string()
        } else {
            "Bilder der Stelle suchen (Sichtbarkeit)".to_string()
        }
    } else {
        p.phase.clone()
    })
}

/// Asks for `action` and steps the window until it is done, timing its phases.
fn act(h: &mut H, action: Action, what: &str) -> Run {
    let messages_before = h.state().state().messages.len();
    let log_before = h.state().state().log.len();
    reset_peak();
    let base = rss_now();
    let start = Instant::now();
    h.state_mut().apply(action);
    let mut phases: Vec<Phase> = Vec::new();
    let mut current = (
        "Stelle speichern, auf den Auftrag warten".to_string(),
        start,
        read_bytes(),
    );
    let mut seen_job = false;
    let mut ended_at: Option<usize> = None;
    let mut steps: Vec<(f64, Instant)> = Vec::new();
    let mut idle = 0;
    let deadline = start + HOUR / 2;
    loop {
        let t = Instant::now();
        h.step();
        let now = Instant::now();
        steps.push(((now - t).as_secs_f64() * 1000.0, now));
        let phase = phase_now(h.state());
        seen_job |= phase.is_some();
        let name = match phase {
            Some(p) => p,
            None if seen_job => "Abschluss und Ansicht".to_string(),
            None => current.0.clone(),
        };
        if seen_job && ended_at.is_none() && h.state().state().job.is_none() {
            ended_at = Some(steps.len() - 1);
        }
        if name != current.0 {
            let read = read_bytes();
            phases.push(Phase {
                name: std::mem::replace(&mut current.0, name),
                seconds: (now - current.1).as_secs_f64(),
                read: read.zip(current.2).map(|(a, b)| a - b),
            });
            current.1 = now;
            current.2 = read;
        }
        let busy = h.state().wants_repaint_while_busy() || h.state().state().job.is_some();
        idle = if busy { 0 } else { idle + 1 };
        if (seen_job && !busy && steps.len() > ended_at.unwrap_or(0) + 2) || idle > 30 {
            break;
        }
        assert!(Instant::now() < deadline, "{what} did not end");
        std::thread::sleep(Duration::from_millis(2));
    }
    let peak = rss_peak();
    let read = read_bytes();
    let (upload_frame_ms, done_at) = match ended_at {
        Some(i) => steps[i..]
            .iter()
            .take(3)
            .fold(
                (0.0f64, steps[i].1),
                |(ms, at), &(m, t)| if m > ms { (m, t) } else { (ms, at) },
            ),
        None => (0.0, steps.last().map_or(start, |s| s.1)),
    };
    phases.push(Phase {
        name: current.0,
        seconds: (done_at.max(current.1) - current.1).as_secs_f64(),
        read: read.zip(current.2).map(|(a, b)| a - b),
    });
    let messages = h.state().state().messages
        [messages_before.min(h.state().state().messages.len())..]
        .iter()
        .map(|m| m.text().to_string())
        .collect();
    let log = &h.state().state().log;
    let protocol = log[log_before.min(log.len())..]
        .iter()
        .map(|l| l.text.clone())
        .collect();
    Run {
        what: what.to_string(),
        seconds: (done_at - start).as_secs_f64(),
        phases,
        upload_frame_ms,
        base,
        peak,
        messages,
        protocol,
    }
}

fn print_run(r: &Run) {
    let memory = match (r.base, r.peak) {
        (Some(b), Some(p)) => format!(
            "Speicher vorher {:.2} GB, Spitze {:.2} GB (+{:.2} GB)",
            gb(b),
            gb(p),
            gb(p.saturating_sub(b))
        ),
        (_, Some(p)) => format!("Spitze im Prozess {:.2} GB", gb(p)),
        _ => "Speicher –".to_string(),
    };
    eprintln!(
        "  {}: {:.2} s, Bild mit dem Hochladen {:.0} ms, {memory}",
        r.what, r.seconds, r.upload_frame_ms
    );
    for p in &r.phases {
        eprintln!(
            "    {:<48} {:>7.3} s{}",
            p.name,
            p.seconds,
            p.read
                .map_or(String::new(), |b| format!(", gelesen {:.0} MB", mb(b)))
        );
    }
    for m in &r.messages {
        eprintln!("    Meldung: {m}");
    }
    for m in &r.protocol {
        eprintln!("    Protokoll: {m}");
    }
}

fn run_json(r: &Run) -> Value {
    json!({
        "was": r.what,
        "sekunden": r.seconds,
        "hochladen_bild_ms": r.upload_frame_ms,
        "speicher_vorher": r.base,
        "speicher_spitze": r.peak,
        "phasen": r.phases.iter().map(|p| json!({"name": p.name, "sekunden": p.seconds, "gelesen": p.read})).collect::<Vec<_>>(),
        "meldungen": r.messages,
        "protokoll": r.protocol,
    })
}

// ---------------------------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------------------------

/// Times of frames (ms): the UI step on the CPU and the drawing on the GPU.
#[derive(Default)]
struct Frames {
    ui: Vec<f64>,
    gpu: Vec<f64>,
}

impl Frames {
    fn push(&mut self, h: &H, ui_ms: f64) {
        self.ui.push(ui_ms);
        self.gpu.push(f64::from(h.state().view().frame_ms()));
    }

    fn summary(&self) -> Value {
        let (mut ui, mut gpu) = (self.ui.clone(), self.gpu.clone());
        // a frame takes as long as the slower of the two
        let mut frame: Vec<f64> = self
            .ui
            .iter()
            .zip(&self.gpu)
            .map(|(u, g)| u.max(*g))
            .collect();
        let slow = frame.iter().filter(|&&f| f > 1000.0 / 30.0).count();
        let median = quantile(&mut frame, 0.5);
        let p90 = quantile(&mut frame, 0.9);
        json!({
            "bilder": self.ui.len(),
            "ui_median_ms": quantile(&mut ui, 0.5),
            "ui_90_ms": quantile(&mut ui, 0.9),
            "ui_max_ms": quantile(&mut ui, 1.0),
            "gpu_median_ms": quantile(&mut gpu, 0.5),
            "gpu_90_ms": quantile(&mut gpu, 0.9),
            "gpu_max_ms": quantile(&mut gpu, 1.0),
            "bilder_pro_s_median": 1000.0 / median,
            "bilder_pro_s_90": 1000.0 / p90,
            "unter_30_pro_s": slow,
        })
    }
}

fn print_frames(what: &str, v: &Value) {
    eprintln!(
        "  {what}: {} Bilder; UI Median {:.1} ms, 90 % {:.1} ms, max {:.1} ms; GPU Median {:.1} ms, 90 % {:.1} ms, max {:.1} ms; {:.0} Bilder/s (Median), {:.0} (90 %), {} Bilder langsamer als 30/s",
        v["bilder"],
        v["ui_median_ms"].as_f64().unwrap_or(f64::NAN),
        v["ui_90_ms"].as_f64().unwrap_or(f64::NAN),
        v["ui_max_ms"].as_f64().unwrap_or(f64::NAN),
        v["gpu_median_ms"].as_f64().unwrap_or(f64::NAN),
        v["gpu_90_ms"].as_f64().unwrap_or(f64::NAN),
        v["gpu_max_ms"].as_f64().unwrap_or(f64::NAN),
        v["bilder_pro_s_median"].as_f64().unwrap_or(f64::NAN),
        v["bilder_pro_s_90"].as_f64().unwrap_or(f64::NAN),
        v["unter_30_pro_s"],
    );
}

/// One frame with `event`, paced like a 60 Hz display so that the GPU times come back.
fn frame(h: &mut H, event: Option<egui::Event>, frames: &mut Frames) {
    if let Some(e) = event {
        h.event(e);
    }
    let t = Instant::now();
    h.step();
    let ms = t.elapsed().as_secs_f64() * 1000.0;
    std::thread::sleep(Duration::from_millis(16));
    frames.push(h, ms);
}

/// The window at 2 pixels per point (Retina), the brush chosen, the camera at the wall: frames
/// while turning the view, while the pointer hovers over the wall (the ring), and while a new
/// Stelle is painted over it in four strokes (pressed to released); the stroke is taken back
/// after.
fn painting(h: &mut H, wall: &Wall) -> Value {
    h.set_pixels_per_point(2.0);
    h.state_mut().apply(Action::SelectMark(None));
    tools::select(h.state_mut().state_mut(), Tool::Brush);
    for _ in 0..5 {
        h.step();
    }
    look_at(h, wall);
    let gpu_timed = h.state().view().gpu_timed();
    let mut orbit = Frames::default();
    for _ in 0..60 {
        h.state_mut().view_mut().camera_mut().orbit(2.0, 0.0);
        frame(h, None, &mut orbit);
    }
    look_at(h, wall);
    // the way over the wall: four rows of 0.9 m, 60 frames each, down between them
    let mut way: Vec<[f64; 3]> = Vec::new();
    for (r, &row) in ROWS.iter().enumerate() {
        for k in 0..=60 {
            let k = if r % 2 == 0 { k } else { 60 - k };
            let a = -0.45 + 0.9 * f64::from(k) / 60.0;
            way.push(add(
                add(wall.centre, scale(wall.across, a)),
                scale(wall.up, row),
            ));
        }
    }
    let points: Vec<egui::Pos2> = way.iter().filter_map(|&p| on_screen(h, p)).collect();
    assert!(points.len() > way.len() / 2, "the wall is not in the view");
    let mut hover = Frames::default();
    for p in points.iter().take(30).rev() {
        frame(h, Some(egui::Event::PointerMoved(*p)), &mut hover);
    }
    let press = |pos, pressed| egui::Event::PointerButton {
        pos,
        button: PointerButton::Primary,
        pressed,
        modifiers: Modifiers::NONE,
    };
    let mut stroke = Frames::default();
    frame(h, Some(press(points[0], true)), &mut stroke);
    for p in &points[1..] {
        frame(h, Some(egui::Event::PointerMoved(*p)), &mut stroke);
    }
    let t = Instant::now();
    h.event(press(*points.last().unwrap(), false));
    h.step();
    let release_ms = t.elapsed().as_secs_f64() * 1000.0;
    frames_until_idle(h);
    let painted = h.state().state().brush.active;
    let dabs = painted
        .and_then(|nr| {
            let owner = brush::shown_owner(h.state().state())?;
            let marks = Marks::load(owner).ok()?;
            Some(marks.marks.iter().find(|m| m.nr == nr)?.dabs.len())
        })
        .unwrap_or(0);
    // the stroke goes again
    h.state_mut().apply(Action::UndoMark);
    frames_until_idle(h);
    tools::select(h.state_mut().state_mut(), Tool::Orbit);
    h.set_pixels_per_point(1.0);
    for _ in 0..3 {
        h.step();
    }
    let out = json!({
        "gpu_zeiten": gpu_timed,
        "drehen": orbit.summary(),
        "zeiger": hover.summary(),
        "malen": stroke.summary(),
        "loslassen_ms": release_ms,
        "tupfer": dabs,
    });
    print_frames("Drehen", &out["drehen"]);
    print_frames("Zeiger über der Wand", &out["zeiger"]);
    print_frames("Malen", &out["malen"]);
    eprintln!("  Loslassen (Zug speichern): {release_ms:.1} ms, {dabs} Tupfer im Zug");
    out
}

// ---------------------------------------------------------------------------------------------
// The cameras
// ---------------------------------------------------------------------------------------------

fn angle(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a.cross(b).norm().atan2(a.dot(b))
}

/// The largest angle a pixel of `cam` spans (to its right or lower neighbour) against the one
/// at the centre, over the pixels up to `max_theta` from the axis: (pixel at the centre in mrad,
/// largest ratio, the angle from the axis where it is in degrees, the largest angle from the
/// axis in the image).
fn pixel_angles(cam: &Camera, max_theta: f64) -> (f64, f64, f64, f64) {
    let span = |u: f64, v: f64| -> Option<(f64, f64)> {
        let r = cam.unproject([u, v])?;
        let right = cam.unproject([u + 1.0, v])?;
        let down = cam.unproject([u, v + 1.0])?;
        Some((
            angle(&r, &right).max(angle(&r, &down)),
            r.z.clamp(-1.0, 1.0).acos(),
        ))
    };
    let (centre, _) = span(cam.cx, cam.cy).expect("the centre of the image");
    let (mut worst, mut at, mut widest) = (0.0f64, 0.0f64, 0.0f64);
    for v in (0..cam.height.saturating_sub(1)).step_by(2) {
        for u in (0..cam.width.saturating_sub(1)).step_by(2) {
            let Some((a, theta)) = span(f64::from(u), f64::from(v)) else {
                continue;
            };
            widest = widest.max(theta);
            if theta <= max_theta && a / centre > worst {
                worst = a / centre;
                at = theta;
            }
        }
    }
    (centre * 1000.0, worst, at.to_degrees(), widest.to_degrees())
}

fn cameras(sources: &[TextureSource]) -> Value {
    let rim = driftless_map::ColorConfig::default().rim_theta;
    let mut out = Vec::new();
    for (i, s) in sources.iter().enumerate() {
        for c in &s.cams {
            let cam = &c.camera;
            let (centre, worst, at, widest) = pixel_angles(cam, rim);
            eprintln!(
                "  Quelle {} Kamera {}: {} × {}, f {:.1} px, Pixel in der Mitte {:.2} mrad, bis {:.0}° größtes Verhältnis {:.2} (bei {:.1}°), Bild reicht bis {:.1}° → Annahme ≤ {CONE_MARGIN} {}",
                i + 1,
                cam.name,
                cam.width,
                cam.height,
                cam.fx,
                centre,
                rim.to_degrees(),
                worst,
                at,
                widest,
                if worst <= CONE_MARGIN { "hält" } else { "hält NICHT" }
            );
            out.push(json!({
                "quelle": i + 1, "kamera": cam.name, "breite": cam.width, "hoehe": cam.height,
                "fx": cam.fx, "fy": cam.fy, "mitte_mrad": centre, "verhaeltnis_max": worst,
                "bei_grad": at, "bild_bis_grad": widest, "modell": format!("{:?}", cam.model),
            }));
        }
    }
    Value::Array(out)
}

// ---------------------------------------------------------------------------------------------
// One surface
// ---------------------------------------------------------------------------------------------

/// The surface of the measurement: a surface folder of a map or model of the project.
struct Target {
    name: &'static str,
    /// For the names of the pictures.
    file: &'static str,
    surface: PathBuf,
}

/// The newest patches of the touch-up of the surface in `dir`.
fn patches(dir: &Path) -> Vec<SurfacePatch> {
    read_patches(&dir.join(TOUCHUP_DIR)).unwrap_or_default()
}

/// Seconds to write `patches` as the touch-up keeps them, and the size of the file.
fn write_time(patches: &[SurfacePatch], work: &Path) -> (f64, u64) {
    let dir = work.join("schreiben-probe");
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    let t = Instant::now();
    write_patches(&dir, patches).unwrap();
    let seconds = t.elapsed().as_secs_f64();
    let bytes = fs::metadata(dir.join(PATCHES_FILE)).map_or(0, |m| m.len());
    fs::remove_dir_all(&dir).unwrap();
    (seconds, bytes)
}

/// The hash of the patches the touch-up of the surface in `dir` keeps now; a copy of its cache
/// goes to `work/vergleich/<name>`, to compare the bytes of two versions of the program.
fn keep_patches(dir: &Path, work: &Path, name: &str) -> Option<u64> {
    let cache = dir.join(TOUCHUP_DIR);
    let bytes = fs::read(cache.join(PATCHES_FILE)).ok()?;
    let dest = work.join("vergleich").join(name);
    let _ = fs::remove_dir_all(&dest);
    fs::create_dir_all(&dest).unwrap();
    for e in fs::read_dir(&cache).unwrap().flatten() {
        fs::copy(e.path(), dest.join(e.file_name())).unwrap();
    }
    Some(driftless_map::content_hash(&bytes))
}

/// The steps of Stelle `nr` of the map or model in `owner`, as saved.
fn steps_of(owner: &Path, nr: u32) -> Vec<EditStep> {
    Marks::load(owner)
        .ok()
        .and_then(|m| m.marks.into_iter().find(|m| m.nr == nr))
        .map(|m| m.steps)
        .unwrap_or_default()
}

/// The measurement on one surface (see the module docs).
fn measure(h: &mut H, target: &Target, work: &Path, shots: Option<&Path>, paint: bool) -> Value {
    eprintln!("\n{}:", target.name);
    let shown = show_surface(h, &target.surface, true);
    eprintln!("  Oberfläche gezeigt nach {shown:.1} s");
    let owner = brush::shown_owner(h.state().state())
        .expect("a surface of the project")
        .to_path_buf();
    let sources = texture_sources(&target.surface)
        .unwrap()
        .expect("the raw data of the surface");
    let t = Instant::now();
    let g = Geometry::read(&target.surface);
    eprintln!(
        "  {} Flächen, {} Punkte, {} Seiten; verschweißt in {:.1} s",
        g.s.face_count(),
        g.s.positions().len(),
        g.mesh.pages.len(),
        t.elapsed().as_secs_f64()
    );
    let walls = find_walls(&g, &eyes(&sources));
    assert!(!walls.is_empty(), "no plain wall of 1 m² found");
    let wall = walls[0].clone();
    eprintln!(
        "  {} glatte Wandstellen gefunden; gewählt (die rauheste): Mitte {:.2?}, Normale {:.2?}, {:.1} m vom Gerät, {} Tupfer, {} Flächen, {:.2} m², Abstand zur Ebene {:.1} mm, Rauheit {:.2} mm",
        walls.len(),
        wall.centre,
        wall.normal,
        wall.distance,
        wall.dabs.len(),
        wall.faces.len(),
        wall.area,
        wall.rms * 1000.0,
        wall.rough
    );
    for (i, w) in walls.iter().enumerate().take(12) {
        eprintln!(
            "    Wand {i}: Mitte {:.2?}, {:.2} m², Abstand zur Ebene {:.1} mm (höchstens {:.1} mm), Rauheit {:.2} mm",
            w.centre,
            w.area,
            w.rms * 1000.0,
            w.max_off * 1000.0,
            w.rough
        );
    }
    // the images „Bilder neu“ looks at for it
    let ccfg = SurfaceSettings::from_report(&json(&target.surface.join("oberflaeche.json")))
        .expect("the settings of the surface")
        .config
        .color;
    let t = Instant::now();
    let (images, looked_at) = candidates(&g, &wall.faces, &sources, &ccfg);
    eprintln!(
        "  Bilder der Quellen {images}, davon Kandidaten nach Lage {looked_at} (nachgezählt in {:.1} s)",
        t.elapsed().as_secs_f64()
    );
    fs::write(
        work.join(format!("wand-{}.toml", target.file)),
        format!(
            "mitte = {:?}\nnormale = {:?}\nquer = {:?}\noben = {:?}\n",
            wall.centre, wall.normal, wall.across, wall.up
        ),
    )
    .unwrap();

    // the holes of the selections (Ruling 12): this Stelle and the other walls found
    let mut hole_rows = Vec::new();
    for (i, w) in walls.iter().take(12).enumerate() {
        let hs = holes(&g, &w.select_dabs());
        if !hs.same {
            eprintln!("  WARNUNG: Nachbau der Lückenregel weicht bei Wand {i} ab");
        }
        hole_rows.push(json!({
            "wand": i, "abstand_ebene_mm": w.rms * 1000.0, "rauheit_mm": w.rough, "flaechen": w.faces.len(),
            "geschlossen": hs.closed.len(), "geschlossen_cm2": hs.closed.iter().sum::<f64>() * 1e4,
            "offen": hs.open.len(), "offen_cm2": hs.open.iter().map(|a| a * 1e4).collect::<Vec<_>>(),
            "gleich_wie_select_faces": hs.same,
        }));
    }
    let open: usize = hole_rows
        .iter()
        .map(|r| r["offen"].as_u64().unwrap() as usize)
        .sum();
    let closed: usize = hole_rows
        .iter()
        .map(|r| r["geschlossen"].as_u64().unwrap() as usize)
        .sum();
    eprintln!(
        "  Lücken in {} Wandstellen: {closed} geschlossen (≤ 25 cm²), {open} offen (> 25 cm²)",
        hole_rows.len()
    );

    // the old texture „Bilder neu“ decodes: the pages of the ring around the Stelle
    let ring = g.ring(&wall.faces);
    let (ring_pages, ring_decode) = decode_pages(&g, &ring);
    eprintln!(
        "  Ring um die Stelle: {} Flächen auf {ring_pages} Seiten; dekodieren {ring_decode:.2} s",
        ring.len()
    );
    let sh = shape(&g, &wall.faces);
    let image_bags = bags(&sources);
    let bag_bytes: u64 = image_bags
        .iter()
        .map(|b| fs::metadata(b).map_or(0, |m| m.len()))
        .sum();
    eprintln!(
        "  Bildtaschen: {} mit zusammen {:.2} GB",
        image_bags.len(),
        gb(bag_bytes)
    );
    // the index of the images of every source (as the first action builds it), from the disk
    // and from the page cache
    let mut index_rows = Vec::new();
    for src in &sources {
        let evicted = src.images.bags.iter().all(|b| evict(b));
        let t = Instant::now();
        let index = index_images(&src.images.bags, &src.images.topics, &Progress::new()).unwrap();
        let cold = t.elapsed().as_secs_f64();
        let t = Instant::now();
        index_images(&src.images.bags, &src.images.topics, &Progress::new()).unwrap();
        let warm = t.elapsed().as_secs_f64();
        eprintln!(
            "  Bildverzeichnis {} ({} Bilder): {}{cold:.3} s, aus dem Cache {warm:.3} s",
            src.images.bags[0].display(),
            index.len(),
            if evicted { "von der Platte " } else { "" },
        );
        index_rows.push(
            json!({"bilder": index.len(), "kalt_s": evicted.then_some(cold), "warm_s": warm}),
        );
    }
    // with the bags in the page cache, as after the first action
    let t = Instant::now();
    for b in &image_bags {
        read_through(b);
    }
    eprintln!(
        "  Bildtaschen einmal gelesen (in den Cache): {:.1} s",
        t.elapsed().as_secs_f64()
    );

    // paint
    let paint_run = act(
        h,
        Action::Paint {
            owner: owner.clone(),
            mark: None,
            dabs: wall.dabs.clone(),
        },
        "Stelle malen (ein Zug)",
    );
    print_run(&paint_run);
    let nr = h.state().state().brush.active.expect("the Stelle painted");
    if let Some(dir) = shots {
        h.state_mut().apply(Action::FrameMark(nr));
        shot(h, dir, &format!("{}-stelle", target.file), false);
    }

    let mut actions = Vec::new();
    let steps: [(&str, &str, Action); 4] = [
        (
            "ganz eben",
            "ganz-eben",
            Action::Smooth {
                mark: nr,
                level: SmoothLevel::Flat,
            },
        ),
        (
            "stark",
            "stark",
            Action::Smooth {
                mark: nr,
                level: SmoothLevel::Strong,
            },
        ),
        (
            "Bilder neu, Variante 0 (Bildtaschen nicht im Cache)",
            "bilder-neu-v0",
            Action::Retexture { mark: nr },
        ),
        (
            "Bilder neu, Variante 1",
            "bilder-neu-v1",
            Action::Retexture { mark: nr },
        ),
    ];
    for (what, file, action) in steps {
        let retexture = matches!(action, Action::Retexture { .. });
        let cold = file == "bilder-neu-v0";
        if cold {
            let evicted = image_bags.iter().filter(|b| evict(b)).count();
            eprintln!(
                "  {evicted} von {} Bildtaschen aus dem Cache genommen",
                image_bags.len()
            );
        }
        let count = steps_of(&owner, nr).len();
        let run = act(h, action, what);
        print_run(&run);
        let hash = keep_patches(&target.surface, work, &format!("{}-{file}", target.file));
        eprintln!(
            "    Änderungen: Prüfsumme {}",
            hash.map_or("–".to_string(), |h| format!("{h:016x}"))
        );
        let done = steps_of(&owner, nr);
        let refused = done.len() == count;
        let ps = patches(&target.surface);
        let (write_s, patch_bytes) = write_time(&ps, work);
        let upload = upload_bytes(&target.surface, &g);
        let smooth = (!retexture && !refused).then(|| {
            smoothness(
                &g,
                &wall.faces,
                &sh,
                ps.last().map_or(&[][..], |p| &p.moved),
            )
        });
        let col = if refused {
            Colour::default()
        } else {
            colour(&g, &ps)
        };
        // the second read of the old texture: the faces of the Stelle that keep it next to a new
        let kept: Vec<u32> = ps.last().map_or(Vec::new(), |last| {
            wall.faces
                .iter()
                .copied()
                .filter(|f| last.removed_faces.binary_search(f).is_err())
                .filter(|&f| {
                    let t = g.s.faces()[f as usize];
                    [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
                        .iter()
                        .any(|&(a, b)| {
                            g.across(f, a, b)
                                .iter()
                                .any(|o| last.removed_faces.binary_search(o).is_ok())
                        })
                })
                .collect()
        });
        let (kept_pages, kept_decode) = decode_pages(&g, &kept);
        let variant = done.last().and_then(|s| match s {
            EditStep::Retexture { variant, .. } => Some(*variant),
            EditStep::Smooth { .. } => None,
        });
        eprintln!(
            "    {}; Änderungen {:.1} MB, schreiben (Nachbau) {write_s:.3} s; Ansicht lädt {:.1} MB hoch; alte Textur: Ring {ring_pages} Seiten {ring_decode:.2} s, dazu {kept_pages} Seiten der Flächen am inneren Rand {kept_decode:.2} s",
            if refused { "ABGELEHNT".to_string() } else { format!("Schritt eingetragen{}", variant.map_or(String::new(), |v| format!(", Variante {v}"))) },
            mb(patch_bytes),
            upload.map_or(f64::NAN, mb),
        );
        if let Some(s) = &smooth {
            eprintln!(
                "    Abstand zur Ebene: alle Punkte {:.2} → {:.2} mm, ganz bewegte (≥ 10 cm vom Rand) {:.2} → {:.2} mm; Rauheit dort {:.2} → {:.2} mm; Sprung am Rand (Punkte neben dem Rand) max {:.2} mm, 95 % {:.2} mm; Rand selbst {:.3} mm; {} Punkte bewegt, max {:.1} mm",
                s.rms_before, s.rms_after, s.inner_before, s.inner_after, s.rough_before, s.rough_after, s.jump_max, s.jump_p95, s.border_max, s.moved, s.moved_max
            );
            eprintln!(
                "    Höhenstufe über die Kanten vom Rand nach innen: RMS {:.2} → {:.2} mm, nachher höchstens {:.2} mm, Änderung 95 % {:.2} mm",
                s.step_before, s.step_after, s.step_after_max, s.step_change_p95
            );
        }
        if col.edges > 0 {
            eprintln!(
                "    Farbunterschied am Rand der neuen Bilder: vorher {:.2}, nachher {:.2} (90 % {:.2}) Stufen über {} Kanten; {} Flächen auf {} neuen Seiten",
                col.before, col.after, col.after_p90, col.edges, col.faces, col.pages
            );
        }
        if let Some(dir) = shots {
            if !refused {
                before_after(h, dir, &format!("{}-{file}", target.file), nr, !retexture);
            }
        }
        let mut row = run_json(&run);
        row["abgelehnt"] = json!(refused);
        row["variante"] = json!(variant);
        row["aenderungen_pruefsumme"] = json!(hash.map(|h| format!("{h:016x}")));
        row["aenderungen_bytes"] = json!(patch_bytes);
        row["schreiben_s"] = json!(write_s);
        row["hochladen_bytes"] = json!(upload);
        row["innerer_rand_seiten"] = json!(kept_pages);
        row["innerer_rand_dekodieren_s"] = json!(kept_decode);
        if let Some(s) = &smooth {
            row["glatt"] = json!({
                "alle_vorher_mm": s.rms_before, "alle_nachher_mm": s.rms_after,
                "innen_vorher_mm": s.inner_before, "innen_nachher_mm": s.inner_after,
                "rauheit_vorher_mm": s.rough_before, "rauheit_nachher_mm": s.rough_after,
                "sprung_max_mm": s.jump_max, "sprung_95_mm": s.jump_p95, "rand_mm": s.border_max,
                "stufe_vorher_mm": s.step_before, "stufe_nachher_mm": s.step_after,
                "stufe_nachher_max_mm": s.step_after_max, "stufe_aenderung_95_mm": s.step_change_p95,
                "bewegt": s.moved, "bewegt_max_mm": s.moved_max,
            });
        }
        if col.edges > 0 {
            row["farbe"] = json!({
                "kanten": col.edges, "vorher": col.before, "nachher": col.after,
                "nachher_90": col.after_p90, "flaechen": col.faces, "seiten": col.pages,
            });
        }
        actions.push(row);
        if !retexture && !refused {
            let undo = act(h, Action::UndoMark, &format!("↶ ({what})"));
            print_run(&undo);
            actions.push(run_json(&undo));
        }
    }
    let frames = paint.then(|| painting(h, &wall));
    json!({
        "oberflaeche_gezeigt_s": shown,
        "flaechen": g.s.face_count(),
        "seiten": g.mesh.pages.len(),
        "wandstellen": walls.len(),
        "stelle": {
            "mitte": wall.centre, "normale": wall.normal, "abstand_m": wall.distance,
            "tupfer": wall.dabs.len(), "flaechen": wall.faces.len(), "flaeche_m2": wall.area,
            "bilder": images, "kandidaten": looked_at, "hoechstens_ebene_mm": wall.max_off * 1000.0,
            "punkte": sh.vertices.len(), "rand": sh.border.iter().filter(|&&b| b).count(),
            "abstand_ebene_mm": wall.rms * 1000.0, "rauheit_mm": wall.rough,
        },
        "luecken": hole_rows,
        "ring": {"flaechen": ring.len(), "seiten": ring_pages, "dekodieren_s": ring_decode},
        "bildtaschen_bytes": bag_bytes,
        "bildverzeichnis": index_rows,
        "malen_zug": run_json(&paint_run),
        "aktionen": actions,
        "malen": frames,
    })
}

// ---------------------------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------------------------

#[test]
#[ignore = "needs $DRIFTLESS_TESTDATA, $DRIFTLESS_WORK and a GPU, about 40 min and 8 GB"]
fn touchup_on_real_scans() {
    let (Some(data), Some(work)) = (
        std::env::var_os("DRIFTLESS_TESTDATA").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_WORK").map(PathBuf::from),
    ) else {
        eprintln!("DRIFTLESS_TESTDATA oder DRIFTLESS_WORK nicht gesetzt, Test übersprungen");
        return;
    };
    if let Some(missing) = SCANS.iter().map(|s| data.join(s)).find(|s| !s.is_dir()) {
        eprintln!(
            "{} fehlt (siehe docs/testdaten.md), Test übersprungen",
            missing.display()
        );
        return;
    }
    let shots = std::env::var_os("DRIFTLESS_SHOTS").map(PathBuf::from);
    if let Some(dir) = &shots {
        fs::create_dir_all(dir).unwrap();
    }
    fs::create_dir_all(&work).unwrap();
    let project = work.join("projekt");
    let start = Instant::now();
    let c = prepare_project(&data, &project);
    eprintln!(
        "\nProjekt fertig nach {:.1} min",
        start.elapsed().as_secs_f64() / 60.0
    );
    let mut out = json!({});
    out["s1"] = print_surface("Oberfläche S1", &c.s1_surface);
    out["s1"]["ordner"] = json!(c.s1_surface);
    out["buero"] = print_surface("Oberfläche B", &c.buero_surface);
    out["modell"] = print_surface("Oberfläche Modell S1–S3", &c.model_surface);
    eprintln!(
        "Dateien der Oberfläche S1 zum Vergleichen: {}",
        c.s1_surface.display()
    );

    eprintln!("\nKameras (Kegel der Verdeckung bei „Bilder neu“):");
    let model_sources = texture_sources(&c.model_surface).unwrap().unwrap();
    let buero_sources = texture_sources(&c.buero_surface).unwrap().unwrap();
    let mut all = model_sources.clone();
    all.extend(buero_sources);
    out["kameras"] = cameras(&all);

    let Some((mut h, gpu)) = open_window(&project) else {
        eprintln!("kein GPU-Adapter: keine Messung im Fenster");
        return;
    };
    eprintln!("\nFenster mit {gpu}");
    out["gpu"] = json!(gpu);
    let targets = [
        Target {
            name: "Modell S1–S3 (dunkler Raum)",
            file: "modell",
            surface: c.model_surface.clone(),
        },
        Target {
            name: "Büro",
            file: "buero",
            surface: c.buero_surface.clone(),
        },
    ];
    let mut results = Vec::new();
    for (i, t) in targets.iter().enumerate() {
        results.push(measure(&mut h, t, &work, shots.as_deref(), i == 1));
    }
    out["oberflaechen"] = Value::Array(results);

    // reading the image bag of B from the disk and from the page cache
    let bag = bags(&texture_sources(&c.buero_surface).unwrap().unwrap())[0].clone();
    if evict(&bag) {
        let (bytes, cold) = read_through(&bag);
        let (_, warm) = read_through(&bag);
        eprintln!(
            "\nBildtasche B ({:.2} GB) ganz gelesen: von der Platte {cold:.1} s ({:.2} GB/s), aus dem Cache {warm:.1} s ({:.2} GB/s)",
            gb(bytes),
            gb(bytes) / cold,
            gb(bytes) / warm
        );
        out["tasche_lesen"] = json!({"bytes": bytes, "kalt_s": cold, "warm_s": warm});
    }
    let errors_now = errors(h.state());
    eprintln!("\nFehler im Fenster: {errors_now:?}");
    eprintln!("Zusammen {:.1} min", start.elapsed().as_secs_f64() / 60.0);
    fs::write(
        work.join("messung.json"),
        serde_json::to_string_pretty(&out).unwrap(),
    )
    .unwrap();
    eprintln!("Zahlen: {}", work.join("messung.json").display());
    assert_eq!(errors_now, Vec::<String>::new(), "{:?}", said(h.state()));
}

/// A project with one scan (synthetic, a few seconds) whose map has the surface in
/// `surface` as its surface, as complete folders would be (the reports files of a few bytes):
/// painting needs nothing else. Returns the project.
fn painting_project(work: &Path, surface: &Path) -> PathBuf {
    let raw = work.join("halle");
    fs::create_dir_all(&raw).unwrap();
    driftless_io::testscan::room_bag_secs(&raw, true, 1.0);
    let project = work.join("malen");
    let mut p = Project::create(&project, "Malen").unwrap();
    p.add_scan(&raw).unwrap();
    p.save().unwrap();
    let project = canonical(&project).unwrap();
    let run = project.join("scans/halle/slam-001");
    let map = run.join("karte-001");
    let dir = map.join("oberflaeche-001");
    fs::create_dir_all(&dir).unwrap();
    for (folder, file) in [
        (&run, "slam.json"),
        (&run, "traj.tum"),
        (&map, "map.json"),
        (&dir, "oberflaeche.json"),
    ] {
        fs::write(folder.join(file), b"{}").unwrap();
    }
    let glb = dir.join("oberflaeche.glb");
    if fs::hard_link(surface.join("oberflaeche.glb"), &glb).is_err() {
        fs::copy(surface.join("oberflaeche.glb"), &glb).unwrap();
    }
    project
}

/// A vector `[x, y, z]` of `key` in the TOML `text`.
fn vector(text: &toml::Table, key: &str) -> [f64; 3] {
    let v = text[key].as_array().unwrap();
    [0, 1, 2].map(|k| v[k].as_float().unwrap())
}

#[test]
#[ignore = "needs $DRIFTLESS_SURFACE, $DRIFTLESS_WALL, $DRIFTLESS_WORK and a GPU"]
fn painting_here() {
    let (Some(surface), Some(wall), Some(work)) = (
        std::env::var_os("DRIFTLESS_SURFACE").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_WALL").map(PathBuf::from),
        std::env::var_os("DRIFTLESS_WORK").map(PathBuf::from),
    ) else {
        eprintln!("DRIFTLESS_SURFACE, DRIFTLESS_WALL oder DRIFTLESS_WORK nicht gesetzt, Test übersprungen");
        return;
    };
    let table: toml::Table = toml::from_str(&fs::read_to_string(&wall).unwrap()).unwrap();
    let project = painting_project(&work, &surface);
    let Some((mut h, gpu)) = open_window(&project) else {
        eprintln!("kein GPU-Adapter, Test übersprungen");
        return;
    };
    eprintln!("Fenster mit {gpu}");
    let dir = project.join("scans/halle/slam-001/karte-001/oberflaeche-001");
    // only loaded: the run of this project has no trajectory to draw
    let shown = show_surface(&mut h, &dir, false);
    eprintln!("Oberfläche gezeigt nach {shown:.1} s");
    let wall = Wall {
        centre: vector(&table, "mitte"),
        normal: vector(&table, "normale"),
        across: vector(&table, "quer"),
        up: vector(&table, "oben"),
        dabs: Vec::new(),
        faces: Vec::new(),
        area: 0.0,
        rms: 0.0,
        max_off: 0.0,
        rough: 0.0,
        distance: 0.0,
    };
    let out = painting(&mut h, &wall);
    fs::write(
        work.join("malen.json"),
        serde_json::to_string_pretty(&out).unwrap(),
    )
    .unwrap();
    assert_eq!(errors(h.state()), Vec::<String>::new());
}
