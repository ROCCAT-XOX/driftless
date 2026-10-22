//! Photo-like surface (baustein 2): a TSDF from the cleaned points, then mesh, texture, glb/obj.
//!
//! [`build_surface`] runs the chain: [`Tsdf`] (the cleaned points and the positions they were
//! measured from fused into a truncated signed distance field) → [`extract`] → [`cleanup`] →
//! [`simplify`] → camera views per face → one image per face → texture pages → seam
//! levelling. [`write_glb`] and [`write_obj`] write the result, [`render_preview`] draws it.

mod atlas;
mod cleanup;
mod extract;
mod level;
mod mc_tables;
mod preview;
mod simplify;
mod tsdf;
mod views;
mod write;

#[cfg(test)]
pub(crate) mod testutil;

pub use atlas::{TexturePage, TextureStats, TexturedMesh};
pub use cleanup::{cleanup, CleanupStats};
pub use extract::{extract, Mesh};
pub use level::SeamStats;
pub use preview::render_preview;
pub use simplify::simplify;
pub use tsdf::Tsdf;
pub use views::TextureSource;
pub use write::{write_glb, write_obj};

use crate::{ColorConfig, MapError};
use driftless_io::progress::Progress;
use driftless_traj::Pose;
use nalgebra::Isometry3;
use std::collections::BTreeMap;

/// Points with the positions they were measured from.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceInput<'a> {
    /// Points in the world frame (m).
    pub xyz: &'a [[f32; 3]],
    /// Point `i` was measured from `sensors[sensor_of_point[i]]`.
    pub sensor_of_point: &'a [u32],
    /// LiDAR positions in the world frame (m). For a map: one per frame, from
    /// [`sensors_for_frames`]; several scans are these lists appended.
    pub sensors: &'a [[f32; 3]],
}

/// The LiDAR position of every frame: `pose(frame_end) · lidar_to_imu`.
pub fn sensors_for_frames(poses: &[Pose], lidar_to_imu: &Isometry3<f64>) -> Vec<[f32; 3]> {
    let mount = lidar_to_imu.translation.vector;
    poses
        .iter()
        .map(|p| {
            let w = p.p + p.q * mount;
            [w.x as f32, w.y as f32, w.z as f32]
        })
        .collect()
}

/// Smallest texture page: the gray block of unseen faces and a chart with its border fit.
pub const MIN_PAGE_SIZE: u32 = 16;

#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceConfig {
    /// Edge length of a TSDF voxel (m).
    pub voxel: f64,
    /// The TSDF band reaches this many voxels in front of and behind a surface.
    pub truncation_voxels: f32,
    /// A voxel needs this many measurements to count as observed.
    pub min_weight: u32,
    /// Mesh components with fewer faces are dropped.
    pub min_component_faces: usize,
    /// Holes with at most this many boundary edges are filled.
    pub max_hole_edges: usize,
    /// Largest distance simplification may move the surface (m).
    pub simplify_error: f64,
    /// After simplification, edges longer than this are split (m): a face must fit into one
    /// camera image to get a texture.
    pub max_edge: f64,
    /// Smallest texel size on the surface (m).
    pub min_texel: f64,
    /// Edge of a texture page (pixels, at least [`MIN_PAGE_SIZE`]).
    pub page_size: u32,
    /// Camera views kept per face.
    pub views_per_face: usize,
    /// Weight of the smoothing step of the texture blend (0 to 1).
    pub smooth_weight: f32,
    /// JPEG quality of the texture pages (1 to 100).
    pub jpeg_quality: u8,
    /// Camera selection and exposure for the texture; images are used at full size.
    pub color: ColorConfig,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        SurfaceConfig {
            voxel: 0.01,
            truncation_voxels: 4.0,
            min_weight: 2,
            min_component_faces: 200,
            max_hole_edges: 64,
            simplify_error: 0.002,
            max_edge: 0.3,
            min_texel: 0.002,
            page_size: 4096,
            views_per_face: 4,
            smooth_weight: 0.5,
            jpeg_quality: 90,
            color: ColorConfig {
                downscale: 1,
                ..ColorConfig::default()
            },
        }
    }
}

impl SurfaceConfig {
    pub fn check(&self) -> Result<(), MapError> {
        let bad = |msg: &str| Err(MapError::Config(msg.into()));
        if !(self.voxel.is_finite() && self.voxel > 0.0) {
            return bad("Voxelgröße der Oberfläche muss größer als 0 sein");
        }
        if !(self.truncation_voxels.is_finite() && self.truncation_voxels >= 1.0) {
            return bad("Bandbreite der Oberfläche muss mindestens 1 Voxel sein");
        }
        if self.min_weight == 0 {
            return bad("Mindestgewicht eines Voxels muss mindestens 1 sein");
        }
        if !(self.simplify_error.is_finite() && self.simplify_error >= 0.0) {
            return bad("Vereinfachungsfehler darf nicht negativ sein");
        }
        if !(self.max_edge.is_finite() && self.max_edge > 0.0) {
            return bad("längste Kante der Oberfläche muss größer als 0 sein");
        }
        if !(self.min_texel.is_finite() && self.min_texel > 0.0) {
            return bad("kleinste Texelgröße muss größer als 0 sein");
        }
        if self.page_size < MIN_PAGE_SIZE {
            return Err(MapError::Config(format!(
                "Texturseite muss mindestens {MIN_PAGE_SIZE} Pixel breit sein"
            )));
        }
        if self.views_per_face == 0 {
            return bad("Ansichten je Fläche müssen mindestens 1 sein");
        }
        if !(0.0..=1.0).contains(&self.smooth_weight) {
            return bad("Glättungsgewicht muss zwischen 0 und 1 liegen");
        }
        if !(1..=100).contains(&self.jpeg_quality) {
            return bad("JPEG-Qualität muss zwischen 1 und 100 liegen");
        }
        self.color.check()
    }
}

/// What [`build_surface`] did.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStats {
    /// Allocated TSDF blocks.
    pub tsdf_blocks: usize,
    /// Faces of the extracted mesh, before cleanup.
    pub faces_raw: usize,
    /// Faces of the final mesh.
    pub faces_final: usize,
    /// Vertices of the final mesh (shared between faces; the texture pages repeat the ones
    /// on chart borders).
    pub vertices: usize,
    pub cleanup: CleanupStats,
    pub texture: TextureStats,
    pub seams: SeamStats,
    /// Mean distance of the final vertices to their nearest input point (mm).
    pub accuracy_mean_mm: f64,
    /// 95th percentile of that distance (mm).
    pub accuracy_p95_mm: f64,
    /// Wall-clock seconds per phase (not deterministic; never written into the mesh files).
    pub seconds: BTreeMap<&'static str, f64>,
}

/// The textured surface of `input`: [`Tsdf::integrate`] → [`extract`] → [`cleanup`] →
/// [`simplify`] → views, labels, baking and seam levelling from `sources`. Without `sources`
/// (a scan without images) every face is gray, on one page of [`MIN_PAGE_SIZE`]². Only one
/// source is supported for now (one scan); several scans come later.
///
/// Progress: the phases of the parts; a cancel is checked between them and returns
/// [`MapError::Cancelled`]. Same input, same mesh and pages (only `seconds` varies).
///
/// Fails if `cfg` is invalid, more than one source is given or the images cannot be read.
pub fn build_surface(
    input: &SurfaceInput,
    sources: &[TextureSource],
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<(TexturedMesh, SurfaceStats), MapError> {
    cfg.check()?;
    if sources.len() > 1 {
        return Err(MapError::Config(
            "mehrere Bildquellen für eine Oberfläche werden noch nicht unterstützt".into(),
        ));
    }
    let mut stats = SurfaceStats::default();
    let mut clock = Clock::new();
    let check = || {
        if progress.is_cancelled() {
            Err(MapError::Cancelled)
        } else {
            Ok(())
        }
    };
    check()?;

    let tsdf = Tsdf::integrate(input, cfg, progress)?;
    stats.tsdf_blocks = tsdf.blocks().count();
    clock.lap(&mut stats, "tsdf");
    check()?;

    progress.phase("Oberfläche: Netz", None);
    let mut raw = extract(&tsdf, cfg);
    drop(tsdf);
    stats.faces_raw = raw.indices.len();
    clock.lap(&mut stats, "extract");
    check()?;

    stats.cleanup = cleanup(&mut raw, cfg);
    clock.lap(&mut stats, "cleanup");
    check()?;

    let mesh = simplify(&raw, cfg);
    drop(raw);
    stats.faces_final = mesh.indices.len();
    stats.vertices = mesh.positions.len();
    clock.lap(&mut stats, "simplify");
    check()?;

    let tex = match sources.first() {
        Some(src) => {
            let views = views::collect_views(&mesh, src, cfg, progress)?;
            clock.lap(&mut stats, "views");
            check()?;
            let labels = atlas::choose_labels(&mesh, &views, cfg);
            clock.lap(&mut stats, "labels");
            check()?;
            let (mut tex, texture) = atlas::bake(&mesh, &labels, &views, src, cfg, progress)?;
            drop(views);
            stats.texture = texture;
            clock.lap(&mut stats, "bake");
            check()?;
            progress.phase("Oberfläche: Nähte", None);
            stats.seams = level::level_seams(&mesh, &labels, &mut tex);
            clock.lap(&mut stats, "seams");
            tex
        }
        None => {
            let (tex, texture) = gray_only(&mesh, cfg, progress)?;
            stats.texture = texture;
            clock.lap(&mut stats, "bake");
            tex
        }
    };
    check()?;

    progress.phase("Oberfläche: Genauigkeit", None);
    (stats.accuracy_mean_mm, stats.accuracy_p95_mm) = accuracy(&mesh.positions, input.xyz);
    clock.lap(&mut stats, "accuracy");
    Ok((tex, stats))
}

/// Every face of `mesh` gray: [`atlas::bake`] without labels and images, on a page of
/// [`MIN_PAGE_SIZE`]² (it holds only the gray block).
fn gray_only(
    mesh: &Mesh,
    cfg: &SurfaceConfig,
    progress: &Progress,
) -> Result<(TexturedMesh, TextureStats), MapError> {
    let cfg = SurfaceConfig {
        page_size: MIN_PAGE_SIZE,
        ..cfg.clone()
    };
    let views = views::ViewTable {
        images: Vec::new(),
        per_face: Vec::new(),
        gains: Vec::new(),
        vignetting: crate::Vignetting::default(),
    };
    let none = TextureSource {
        images: crate::ImageSource {
            bags: Vec::new(),
            topics: Vec::new(),
        },
        poses: Vec::new(),
        cams: Vec::new(),
    };
    let labels = vec![None; mesh.indices.len()];
    atlas::bake(mesh, &labels, &views, &none, &cfg, progress)
}

/// Mean and 95th percentile (mm) of the distance of every vertex to its nearest point of
/// `xyz` (k-d tree); zero without vertices or points.
fn accuracy(vertices: &[[f32; 3]], xyz: &[[f32; 3]]) -> (f64, f64) {
    use kiddo::{ImmutableKdTree, SquaredEuclidean};
    use rayon::prelude::*;
    if vertices.is_empty() || xyz.is_empty() {
        return (0.0, 0.0);
    }
    let dist = |a: &[f32; 3], b: &[f32; 3]| -> f64 {
        (0..3)
            .map(|k| (f64::from(a[k]) - f64::from(b[k])).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    let mut mm: Vec<f64> = if xyz.len() < 2 {
        vertices.iter().map(|v| dist(v, &xyz[0]) * 1000.0).collect()
    } else {
        let tree: ImmutableKdTree<f32, 3> =
            ImmutableKdTree::new_from_slice(xyz).expect("fewer than 2^32 points");
        vertices
            .par_iter()
            .map(|v| {
                let nn = tree
                    .query(v)
                    .nearest_one::<SquaredEuclidean<f32>>()
                    .execute();
                // Distance from the f32 coordinates again in f64: no squared f32 rounding.
                dist(v, &xyz[nn.item as usize]) * 1000.0
            })
            .collect()
    };
    let mean = mm.iter().sum::<f64>() / mm.len() as f64;
    mm.par_sort_unstable_by(f64::total_cmp);
    let p95 = mm[((mm.len() - 1) as f64 * 0.95).round() as usize];
    (mean, p95)
}

/// Wall-clock time per phase.
struct Clock(std::time::Instant);

impl Clock {
    fn new() -> Clock {
        Clock(std::time::Instant::now())
    }

    /// Adds the time since the last lap to `phase`.
    fn lap(&mut self, stats: &mut SurfaceStats, phase: &'static str) {
        let now = std::time::Instant::now();
        *stats.seconds.entry(phase).or_default() += (now - self.0).as_secs_f64();
        self.0 = now;
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::*;
    use super::*;

    fn small() -> SurfaceConfig {
        SurfaceConfig {
            min_component_faces: 50,
            ..SurfaceConfig::default()
        }
    }

    #[test]
    fn build_surface_without_images_is_gray() {
        let scan = box_room([0.0; 3], [1.0, 0.8, 0.6], 0.004);
        let (tex, stats) = build_surface(&scan.input(), &[], &small(), &Progress::new()).unwrap();
        assert!(stats.tsdf_blocks > 0);
        assert!(stats.faces_raw >= stats.faces_final);
        assert!(stats.faces_final > 0);
        assert_eq!(stats.texture.faces_textured, 0);
        assert_eq!(stats.texture.faces_untextured, stats.faces_final);
        assert_eq!(stats.vertices, tex.pages[0].positions.len());
        // One small page: the gray block, every face on it, every texel gray.
        assert_eq!(tex.pages.len(), 1);
        let page = &tex.pages[0];
        assert_eq!(page.indices.len(), stats.faces_final);
        assert_eq!(page.image.width, MIN_PAGE_SIZE);
        assert!(page.image.data.iter().all(|&v| v == atlas::UNSEEN_GRAY));
        // Vertices lie on the walls: within a few mm of the 4 mm grid of points.
        assert!(
            stats.accuracy_mean_mm > 0.0 && stats.accuracy_mean_mm < 3.0,
            "{stats:?}"
        );
        assert!(stats.accuracy_p95_mm >= stats.accuracy_mean_mm);
        assert!(stats.accuracy_p95_mm < 5.0, "{stats:?}");
        for phase in ["tsdf", "extract", "cleanup", "simplify", "accuracy"] {
            assert!(stats.seconds.contains_key(phase), "{phase}");
        }
        // Same input, same mesh.
        let (again, _) = build_surface(&scan.input(), &[], &small(), &Progress::new()).unwrap();
        assert_eq!(again, tex);
    }

    /// Points 2 cm apart on every surface of `painted`, each measured from 0.5 m in front of
    /// its surface (into the room, out of the boxes); box sides on a room side are left out.
    fn painted_scan(painted: &scene::Painted) -> Scan {
        let (mut xyz, mut sensors) = (Vec::new(), Vec::new());
        let room = &painted.scene.room;
        let mut add = |b: &scene::Aabb, inward: bool| {
            for axis in 0..3 {
                let (a, c) = ((axis + 1) % 3, (axis + 2) % 3);
                for high in [false, true] {
                    let at = if high { b.max[axis] } else { b.min[axis] };
                    if !inward && (at == room.min[axis] || at == room.max[axis]) {
                        continue;
                    }
                    let out = if high != inward { 1.0 } else { -1.0 };
                    let n = |lo: f64, hi: f64| ((hi - lo) / 0.02).round() as usize;
                    let (na, nc) = (n(b.min[a], b.max[a]), n(b.min[c], b.max[c]));
                    for ia in 0..=na {
                        for ic in 0..=nc {
                            let mut p = [0.0f64; 3];
                            p[axis] = at;
                            p[a] = b.min[a] + (b.max[a] - b.min[a]) * ia as f64 / na as f64;
                            p[c] = b.min[c] + (b.max[c] - b.min[c]) * ic as f64 / nc as f64;
                            let mut s = p;
                            s[axis] += out * 0.5;
                            xyz.push(p.map(|v| v as f32));
                            sensors.push(s.map(|v| v as f32));
                        }
                    }
                }
            }
        };
        add(room, true);
        for b in &painted.scene.boxes {
            add(b, false);
        }
        Scan {
            sensor_of_point: (0..xyz.len() as u32).collect(),
            xyz,
            sensors,
        }
    }

    #[test]
    fn build_surface_textures_painted_room() {
        let (painted, images, poses, cams) = scene::walk_data();
        let scan = painted_scan(painted);
        let src = TextureSource {
            images: images.clone(),
            poses: poses.clone(),
            cams: cams.clone(),
        };
        let cfg = SurfaceConfig {
            voxel: 0.045,
            min_texel: 0.01,
            page_size: 1024,
            ..SurfaceConfig::default()
        };
        let (tex, stats) = build_surface(
            &scan.input(),
            std::slice::from_ref(&src),
            &cfg,
            &Progress::new(),
        )
        .unwrap();
        assert!(stats.texture.faces_textured > 0, "{stats:?}");
        assert_eq!(
            stats.texture.faces_textured + stats.texture.faces_untextured,
            stats.faces_final
        );
        assert!(
            stats.seams.seam_jump_after <= stats.seams.seam_jump_before,
            "{stats:?}"
        );
        // Mostly the 2 cm spacing of the points (a vertex between points is up to 14 mm away).
        assert!(stats.accuracy_mean_mm < 25.0, "{stats:?}");
        for phase in ["views", "labels", "bake", "seams"] {
            assert!(stats.seconds.contains_key(phase), "{phase}");
        }
        // The +y wall in front of the walk, as the left camera saw it.
        let img = render_preview(&tex, [0.5, 0.0, 0.0], [0.5, 5.0, 0.0], 40.0, [32, 24]);
        let i = (12 * 32 + 16) * 3;
        let got = [img.data[i], img.data[i + 1], img.data[i + 2]];
        assert!(
            scene::close(got, scene::WALLS[3], 2 * scene::TOL),
            "{got:?}"
        );

        // Two sources are refused.
        let two = [src.clone(), src];
        let r = build_surface(&scan.input(), &two, &cfg, &Progress::new());
        assert!(matches!(r, Err(MapError::Config(_))), "{r:?}");
    }

    #[test]
    fn build_surface_cancelled() {
        let scan = box_room([0.0; 3], [1.0, 0.8, 0.6], 0.01);
        let progress = Progress::new();
        progress.cancel();
        let r = build_surface(&scan.input(), &[], &small(), &progress);
        assert!(matches!(r, Err(MapError::Cancelled)), "{r:?}");
    }
}
