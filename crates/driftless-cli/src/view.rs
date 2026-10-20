//! `driftless view <ordner>`: Fehlersuche-Viewer (rerun) für eine Rohaufnahme.
//!
//! Die Bags werden Nachricht für Nachricht gelesen, geloggt und verworfen;
//! der Speicher bleibt unabhängig von der Aufnahmelänge klein.

use crate::topics::{file_name, TopicPicker};
use driftless_io::bag::{BagError, BagReader};
use driftless_io::msg::{decode_compressed_image, decode_imu, decode_pointcloud2, LidarFrame};
use driftless_io::project::discover_project;
use driftless_slam::SlamReport;
use driftless_traj::read_tum;
use std::path::PathBuf;

const TIMELINE: &str = "stamp";
/// With `--slam`: seconds since the start of the scan, the times `driftless correct` expects.
const SCAN_TIMELINE: &str = "ab_scan_beginn";
/// Slack when comparing times read from `traj.tum` (9 decimals) with `slam.json`.
const TIME_TOLERANCE: f64 = 1e-6;
const CLOUD_TYPE: &str = "sensor_msgs/PointCloud2";
const IMU_TYPE: &str = "sensor_msgs/Imu";
const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";
const RAW_IMAGE_TYPE: &str = "sensor_msgs/Image";

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Trajektorie (TUM-Datei) als Linie einblenden
    #[arg(long)]
    traj: Option<PathBuf>,
    /// Ausgabeordner von `driftless slam` (slam.json, traj.tum, traj_odom.tum): Laufbahn nach
    /// Güte eingefärbt, Odometrie, Schleifen und Teilkarten-Anker einblenden
    #[arg(long, value_name = "ORDNER")]
    slam: Option<PathBuf>,
    /// PointCloud2-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    lidar_topic: Option<String>,
    /// Imu-Topic (Standard: das einzige in der Bag)
    #[arg(long)]
    imu_topic: Option<String>,
    /// Nur jedes N-te Kamerabild loggen
    #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u32).range(1..))]
    every: u32,
    /// Punktwolke (LAS, z. B. color.las von `driftless map`) einblenden: mit ihren Farben,
    /// ohne Farben grau nach Intensität; über 5 Mio. Punkte gleichmäßig ausgedünnt
    #[arg(long, value_name = "DATEI.las")]
    cloud: Option<PathBuf>,
    /// Statt den Viewer zu starten in diese .rrd-Datei schreiben (kein Fenster)
    #[arg(long, value_name = "DATEI.rrd")]
    dry_run: Option<PathBuf>,
}

type BoxError = Box<dyn std::error::Error>;

/// At most this many points of a `--cloud` are shown.
pub const MAX_CLOUD_POINTS: u64 = 5_000_000;

/// Keep every `cloud_stride(n)`-th point of `n` so that at most [`MAX_CLOUD_POINTS`] remain.
fn cloud_stride(n: u64) -> u64 {
    n.div_ceil(MAX_CLOUD_POINTS).max(1)
}

/// A LAS cloud, thinned to every [`cloud_stride`]-th point (always the same ones).
struct CloudView {
    name: String,
    total: u64,
    xyz: Vec<[f32; 3]>,
    colors: Vec<rerun::Color>,
    /// Colours from the file (else gray from intensity).
    rgb: bool,
}

fn load_cloud(path: &std::path::Path) -> Result<CloudView, BoxError> {
    let err = |e: las::Error| format!("{}: {e}", path.display());
    let mut reader = las::Reader::from_path(path).map_err(err)?;
    let total = reader.header().number_of_points();
    let rgb = reader.header().point_format().has_color;
    let stride = cloud_stride(total);
    let mut xyz = Vec::new();
    let mut gray = Vec::new();
    let mut colors = Vec::new();
    let mut batch = las::PointDataBuilder::new()
        .for_header(reader.header())
        .build();
    let mut i = 0u64;
    while reader.fill_points(1 << 20, &mut batch).map_err(err)? > 0 {
        for p in batch.points() {
            let p = p.map_err(err)?;
            let take = i.is_multiple_of(stride);
            i += 1;
            if !take {
                continue;
            }
            xyz.push([p.x as f32, p.y as f32, p.z as f32]);
            match (rgb, p.color) {
                (true, Some(c)) => colors.push(rerun::Color::from_rgb(
                    (c.red >> 8) as u8,
                    (c.green >> 8) as u8,
                    (c.blue >> 8) as u8,
                )),
                _ => gray.push(p.intensity),
            }
        }
    }
    if !rgb {
        let max = gray.iter().copied().max().unwrap_or(0).max(1);
        colors = gray
            .iter()
            .map(|&i| {
                let v = (u32::from(i) * 255 / u32::from(max)) as u8;
                rerun::Color::from_rgb(v, v, v)
            })
            .collect();
    }
    Ok(CloudView {
        name: file_name(path),
        total,
        xyz,
        colors,
        rgb,
    })
}

/// Grayscale ramp, normalised per frame to the maximum intensity.
fn intensity_colors(frame: &LidarFrame) -> Vec<rerun::Color> {
    let max = frame
        .points
        .iter()
        .map(|p| p.intensity)
        .filter(|v| v.is_finite())
        .fold(0.0_f32, f32::max);
    let scale = if max > 0.0 { 255.0 / max } else { 0.0 };
    frame
        .points
        .iter()
        .map(|p| {
            let v = if p.intensity.is_finite() {
                (p.intensity * scale).clamp(0.0, 255.0) as u8
            } else {
                0
            };
            rerun::Color::from_rgb(v, v, v)
        })
        .collect()
}

/// Entity path of a camera topic: `/camera_left/image_raw` becomes `camera/left`,
/// `/alphasense/cam0/image_raw/compressed` becomes `camera/alphasense/cam0`.
fn camera_entity(topic: &str) -> String {
    let mut parts: Vec<&str> = topic.split('/').filter(|p| !p.is_empty()).collect();
    while parts.len() > 1 && matches!(parts.last(), Some(&("compressed" | "image_raw" | "image"))) {
        parts.pop();
    }
    if let Some(first) = parts.first_mut() {
        *first = first.strip_prefix("camera_").unwrap_or(first);
    }
    format!("camera/{}", parts.join("/"))
}

/// Colour of a submap quality: red (0) via yellow (0.5) to green (1).
fn quality_color(quality: f32) -> [u8; 3] {
    let q = if quality.is_finite() {
        quality.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let ramp = |x: f32| (x * 255.0).round() as u8;
    if q < 0.5 {
        [255, ramp(q * 2.0), 0]
    } else {
        [ramp(2.0 - q * 2.0), 255, 0]
    }
}

const LOOP_ACCEPTED: [u8; 3] = [0, 200, 0];
const LOOP_REJECTED: [u8; 3] = [140, 140, 140];
const LOOP_MANUAL: [u8; 3] = [40, 120, 255];

/// Everything of a `driftless slam` output the viewer shows, read and checked up front.
struct SlamView {
    report: SlamReport,
    corrected: Vec<driftless_traj::Pose>,
    odometry: Vec<driftless_traj::Pose>,
}

fn load_slam(dir: &std::path::Path) -> Result<SlamView, BoxError> {
    let json_path = dir.join("slam.json");
    let text = std::fs::read_to_string(&json_path)
        .map_err(|e| format!("{} nicht lesbar: {e}", json_path.display()))?;
    let report: SlamReport = serde_json::from_str(&text).map_err(|e| {
        format!(
            "{} ist kein gültiger SLAM-Bericht: {e}; mit `driftless slam` neu erzeugen",
            json_path.display()
        )
    })?;
    Ok(SlamView {
        report,
        corrected: read_tum(&dir.join("traj.tum"))?,
        odometry: read_tum(&dir.join("traj_odom.tum"))?,
    })
}

fn xyz(p: &driftless_traj::Pose) -> [f32; 3] {
    [p.p.x as f32, p.p.y as f32, p.p.z as f32]
}

fn xyz64(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

/// One line strip per submap: a pose belongs to the last submap that started at or before
/// it (within [`TIME_TOLERANCE`]). Each strip also ends on the first point of the next one,
/// so the coloured line has no gaps. Submaps without poses are skipped.
fn trajectory_strips(view: &SlamView) -> (Vec<Vec<[f32; 3]>>, Vec<[u8; 3]>) {
    let submaps = &view.report.submaps;
    let t0 = view.report.scan_start;
    let mut groups: Vec<Vec<[f32; 3]>> = vec![Vec::new(); submaps.len()];
    for pose in &view.corrected {
        let t = pose.t - t0;
        let idx = submaps
            .partition_point(|m| m.t_start <= t + TIME_TOLERANCE)
            .saturating_sub(1);
        if let Some(g) = groups.get_mut(idx) {
            g.push(xyz(pose));
        }
    }
    let mut strips: Vec<Vec<[f32; 3]>> = Vec::new();
    let mut colors = Vec::new();
    let mut last: Option<usize> = None;
    for (i, g) in groups.iter().enumerate() {
        if g.is_empty() {
            continue;
        }
        if let Some(prev) = last {
            strips[prev].push(g[0]);
        }
        strips.push(g.clone());
        colors.push(quality_color(submaps[i].quality));
        last = Some(strips.len() - 1);
    }
    (strips, colors)
}

fn log_slam(rec: &rerun::RecordingStream, view: &SlamView) -> Result<(), BoxError> {
    let (strips, colors) = trajectory_strips(view);
    rec.log_static(
        "slam/trajectory",
        &rerun::LineStrips3D::new(strips)
            .with_colors(
                colors
                    .iter()
                    .map(|c| rerun::Color::from_rgb(c[0], c[1], c[2])),
            )
            .with_radii([rerun::Radius::new_ui_points(2.5)]),
    )?;
    rec.log_static(
        "slam/odometry",
        &rerun::LineStrips3D::new([view.odometry.iter().map(xyz).collect::<Vec<_>>()])
            .with_colors([rerun::Color::from_rgb(130, 130, 130)])
            .with_radii([rerun::Radius::new_ui_points(0.8)]),
    )?;

    let submaps = &view.report.submaps;
    let mut strips = Vec::new();
    let mut colors = Vec::new();
    let mut labels = Vec::new();
    for l in &view.report.loops {
        let (Some(a), Some(b)) = (submaps.get(l.a), submaps.get(l.b)) else {
            continue;
        };
        strips.push(vec![xyz64(a.anchor), xyz64(b.anchor)]);
        let c = if l.manual {
            LOOP_MANUAL
        } else if l.accepted {
            LOOP_ACCEPTED
        } else {
            LOOP_REJECTED
        };
        colors.push(rerun::Color::from_rgb(c[0], c[1], c[2]));
        labels.push(match &l.reason {
            Some(why) => format!("{}↔{} ({why})", l.a, l.b),
            None => format!("{}↔{}", l.a, l.b),
        });
    }
    rec.log_static(
        "slam/loops",
        &rerun::LineStrips3D::new(strips)
            .with_colors(colors)
            .with_labels(labels)
            .with_radii([rerun::Radius::new_ui_points(1.5)]),
    )?;

    rec.log_static(
        "slam/anchors",
        &rerun::Points3D::new(submaps.iter().map(|m| xyz64(m.anchor)))
            .with_colors(submaps.iter().map(|m| {
                let c = quality_color(m.quality);
                rerun::Color::from_rgb(c[0], c[1], c[2])
            }))
            .with_labels(
                submaps
                    .iter()
                    .map(|m| format!("{}: {:.0}–{:.0} s", m.id, m.t_start, m.t_end)),
            )
            .with_radii([rerun::Radius::new_ui_points(4.0)]),
    )?;
    Ok(())
}

/// Sets the time of the next log calls: the bag stamp and, with `--slam`, the seconds since
/// the start of the scan.
fn set_stamp(rec: &rerun::RecordingStream, stamp: f64, scan_start: Option<f64>) {
    rec.set_duration_secs(TIMELINE, stamp);
    if let Some(t0) = scan_start {
        rec.set_duration_secs(SCAN_TIMELINE, stamp - t0);
    }
}

/// Topics to show from one bag.
struct BagPlan {
    path: PathBuf,
    lidar: Option<String>,
    imu: Option<String>,
    cameras: Vec<String>,
}

#[derive(Default)]
struct Counts {
    lidar: u64,
    imu: u64,
    images: u64,
}

/// Opens every bag once (index only) and decides which topics to log. Fails before the
/// viewer starts if the topic choice is ambiguous, an explicit name does not exist, or
/// there is no LiDAR topic at all.
fn plan_bags(
    bags: &[PathBuf],
    args: &Args,
    warnings: &mut Vec<String>,
) -> Result<Vec<BagPlan>, BoxError> {
    let mut lidar = TopicPicker::new(CLOUD_TYPE, args.lidar_topic.as_deref(), "--lidar-topic");
    let mut imu = TopicPicker::new(IMU_TYPE, args.imu_topic.as_deref(), "--imu-topic");
    let mut plans = Vec::new();
    let mut raw_images = std::collections::BTreeSet::new();
    for path in bags {
        let reader = BagReader::open(path).map_err(|e| format!("{}: {e}", file_name(path)))?;
        let mut cameras: Vec<String> = Vec::new();
        for c in reader.connections() {
            if c.msg_type == COMPRESSED_IMAGE_TYPE && !cameras.contains(&c.topic) {
                cameras.push(c.topic.clone());
            } else if c.msg_type == RAW_IMAGE_TYPE {
                raw_images.insert(c.topic.clone());
            }
        }
        cameras.sort();
        plans.push(BagPlan {
            path: path.clone(),
            lidar: lidar.pick(&reader)?,
            imu: imu.pick(&reader)?,
            cameras,
        });
    }
    lidar.finish()?;
    imu.finish()?;
    if plans.iter().all(|p| p.lidar.is_none()) {
        return Err(
            format!("kein {CLOUD_TYPE}-Topic in den Bags; es gibt nichts zu zeigen").into(),
        );
    }
    if !raw_images.is_empty() {
        let names: Vec<&str> = raw_images.iter().map(String::as_str).collect();
        warnings.push(format!(
            "{RAW_IMAGE_TYPE}-Topics ({}) werden noch nicht angezeigt, nur {COMPRESSED_IMAGE_TYPE}",
            names.join(", ")
        ));
    }
    Ok(plans)
}

fn log_bag(
    rec: &rerun::RecordingStream,
    plan: &BagPlan,
    every: u32,
    scan_start: Option<f64>,
    counts: &mut Counts,
    warnings: &mut Vec<String>,
) -> Result<(), BoxError> {
    let name = file_name(&plan.path);
    let topics: Vec<&str> = plan
        .lidar
        .iter()
        .chain(&plan.imu)
        .chain(&plan.cameras)
        .map(String::as_str)
        .collect();
    if topics.is_empty() {
        return Ok(());
    }
    let mut reader = BagReader::open(&plan.path)?;
    let mut image_count: std::collections::HashMap<String, u32> = Default::default();
    let mut decode_errors = 0u64;
    for item in reader.messages(&topics) {
        let msg = match item {
            Ok(m) => m,
            Err(BagError::Truncated { offset }) => {
                warnings.push(format!(
                    "{name}: Datei abgeschnitten bei Byte {offset}, Rest übersprungen"
                ));
                break;
            }
            Err(e) => return Err(format!("{name}: {e}").into()),
        };
        let topic = Some(&msg.topic);
        if topic == plan.lidar.as_ref() {
            match decode_pointcloud2(&msg.data) {
                Ok(f) => {
                    set_stamp(rec, f.stamp, scan_start);
                    rec.log(
                        "lidar/points",
                        &rerun::Points3D::new(f.points.iter().map(|p| p.xyz))
                            .with_colors(intensity_colors(&f)),
                    )?;
                    counts.lidar += 1;
                }
                Err(_) => decode_errors += 1,
            }
        } else if topic == plan.imu.as_ref() {
            match decode_imu(&msg.data) {
                Ok(s) => {
                    set_stamp(rec, s.stamp, scan_start);
                    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
                        rec.log(
                            format!("imu/gyro/{axis}"),
                            &rerun::Scalars::single(s.gyro[i]),
                        )?;
                        rec.log(
                            format!("imu/accel/{axis}"),
                            &rerun::Scalars::single(s.accel[i]),
                        )?;
                    }
                    counts.imu += 1;
                }
                Err(_) => decode_errors += 1,
            }
        } else {
            let n = image_count.entry(msg.topic.clone()).or_insert(0);
            let take = n.is_multiple_of(every);
            *n += 1;
            if !take {
                continue;
            }
            match decode_compressed_image(&msg.data) {
                Ok(img) => {
                    set_stamp(rec, img.stamp, scan_start);
                    rec.log(
                        camera_entity(&msg.topic),
                        &rerun::EncodedImage::from_file_contents(img.data),
                    )?;
                    counts.images += 1;
                }
                Err(_) => decode_errors += 1,
            }
        }
    }
    if decode_errors > 0 {
        warnings.push(format!("{name}: {decode_errors} Nachrichten nicht lesbar"));
    }
    Ok(())
}

pub fn run(args: &Args) -> Result<(), BoxError> {
    let project = discover_project(&args.folder)?;
    for p in &project.problems {
        eprintln!("Hinweis: {p}");
    }
    // Everything that can fail on the input is checked before a viewer window opens.
    let line: Option<Vec<[f32; 3]>> = match &args.traj {
        Some(traj) => Some(
            read_tum(traj)?
                .iter()
                .map(|p| [p.p.x as f32, p.p.y as f32, p.p.z as f32])
                .collect(),
        ),
        None => None,
    };
    let slam = match &args.slam {
        Some(dir) => Some(load_slam(dir)?),
        None => None,
    };
    let cloud = match &args.cloud {
        Some(path) => Some(load_cloud(path)?),
        None => None,
    };
    let mut warnings = Vec::new();
    let all_bags: Vec<PathBuf> = project
        .lidar_bags
        .iter()
        .chain(&project.image_bags)
        .cloned()
        .collect();
    let plans = plan_bags(&all_bags, args, &mut warnings)?;

    let builder = rerun::RecordingStreamBuilder::new("driftless");
    let rec = match &args.dry_run {
        Some(file) => builder.save(file)?,
        None => builder.spawn().map_err(|e| {
            format!(
                "Viewer konnte nicht gestartet werden: {e}\n\
                 Der rerun-Viewer muss installiert sein: cargo install rerun-cli --version 0.38.1 --locked"
            )
        })?,
    };
    if let Some(line) = line {
        rec.log_static("trajectory", &rerun::LineStrips3D::new([line]))?;
    }
    if let Some(view) = &slam {
        log_slam(&rec, view)?;
    }
    if let Some(c) = &cloud {
        rec.log_static(
            "map/cloud",
            &rerun::Points3D::new(c.xyz.iter().copied())
                .with_colors(c.colors.iter().copied())
                .with_radii([rerun::Radius::new_ui_points(1.0)]),
        )?;
    }
    let scan_start = slam.as_ref().map(|v| v.report.scan_start);
    let mut counts = Counts::default();
    for plan in &plans {
        log_bag(
            &rec,
            plan,
            args.every,
            scan_start,
            &mut counts,
            &mut warnings,
        )?;
    }
    rec.flush_blocking()?;
    for w in &warnings {
        eprintln!("Warnung: {w}");
    }
    println!(
        "{} LiDAR-Frames, {} IMU-Messungen, {} Kamerabilder geloggt",
        counts.lidar, counts.imu, counts.images
    );
    if let Some(c) = &cloud {
        println!(
            "Wolke {}: {} von {} Punkten ({})",
            c.name,
            c.xyz.len(),
            c.total,
            if c.rgb { "Farbe" } else { "Intensität" }
        );
    }
    if slam.is_some() {
        println!(
            "Zeitleiste {SCAN_TIMELINE}: Sekunden ab Scan-Beginn, wie `driftless correct loop` sie erwartet"
        );
    }
    if let Some(file) = &args.dry_run {
        println!("Aufzeichnung geschrieben: {}", file.display());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        camera_entity, cloud_stride, quality_color, trajectory_strips, SlamView, MAX_CLOUD_POINTS,
    };
    use driftless_slam::report::{BeforeAfter, SlamReport, SubmapReport};
    use driftless_traj::nalgebra::{UnitQuaternion, Vector3};
    use driftless_traj::Pose;

    /// A pose on a submap start, rounded below it by the 9 decimals of `traj.tum`, belongs
    /// to that submap.
    #[test]
    fn trajectory_strips_tolerate_rounded_times() {
        let sm = |id: usize, t_start: f64, quality: f32| SubmapReport {
            id,
            t_start,
            t_end: t_start + 0.5,
            quality,
            anchor: [0.0; 3],
            odom_anchor: [0.0; 3],
        };
        let none = BeforeAfter {
            before: 0.0,
            after: 0.0,
        };
        let scan_start = 1000.25;
        let pose = |t: f64, x: f64| Pose {
            t,
            p: Vector3::new(x, 0.0, 0.0),
            q: UnitQuaternion::identity(),
        };
        let view = SlamView {
            report: SlamReport {
                scan_start,
                submaps: vec![sm(0, 0.0, 1.0), sm(1, 0.6, 0.0)],
                loops: Vec::new(),
                graph: None,
                start_end_gap: none.clone(),
                path_length: none,
            },
            // the second pose is the first frame of submap 1, written 0.4 ns early
            corrected: vec![
                pose(scan_start, 0.0),
                pose(scan_start + 0.6 - 4e-10, 1.0),
                pose(scan_start + 1.0, 2.0),
            ],
            odometry: Vec::new(),
        };
        let (strips, colors) = trajectory_strips(&view);
        assert_eq!(colors, [quality_color(1.0), quality_color(0.0)]);
        assert_eq!(strips[0].len(), 2, "{strips:?}");
        assert_eq!(strips[1].len(), 2, "{strips:?}");
        assert_eq!(strips[1][0], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn cloud_stride_keeps_at_most_the_limit() {
        for n in [
            0,
            1,
            MAX_CLOUD_POINTS,
            MAX_CLOUD_POINTS + 1,
            12_500_000,
            3 * MAX_CLOUD_POINTS,
        ] {
            let s = cloud_stride(n);
            assert!(n.div_ceil(s) <= MAX_CLOUD_POINTS, "{n}");
        }
        assert_eq!(cloud_stride(MAX_CLOUD_POINTS), 1);
        assert_eq!(cloud_stride(MAX_CLOUD_POINTS + 1), 2);
    }

    #[test]
    fn quality_color_endpoints() {
        assert_eq!(quality_color(0.0), [255, 0, 0]);
        assert_eq!(quality_color(0.5), [255, 255, 0]);
        assert_eq!(quality_color(1.0), [0, 255, 0]);
        assert_eq!(quality_color(-3.0), [255, 0, 0]);
        assert_eq!(quality_color(f32::NAN), [255, 0, 0]);
    }

    #[test]
    fn camera_entities() {
        assert_eq!(camera_entity("/camera_left/image_raw"), "camera/left");
        assert_eq!(
            camera_entity("/alphasense/cam0/image_raw/compressed"),
            "camera/alphasense/cam0"
        );
        assert_eq!(camera_entity("/cam"), "camera/cam");
    }
}
