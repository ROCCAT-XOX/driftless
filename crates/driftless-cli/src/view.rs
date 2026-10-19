//! `driftless view <ordner>`: Fehlersuche-Viewer (rerun) für eine Rohaufnahme.
//!
//! Die Bags werden Nachricht für Nachricht gelesen, geloggt und verworfen;
//! der Speicher bleibt unabhängig von der Aufnahmelänge klein.

use driftless_io::bag::{BagError, BagReader};
use driftless_io::msg::{decode_compressed_image, decode_imu, decode_pointcloud2, LidarFrame};
use driftless_io::project::discover_project;
use driftless_traj::read_tum;
use std::path::{Path, PathBuf};

const TIMELINE: &str = "stamp";

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Trajektorie (TUM-Datei) als Linie einblenden
    #[arg(long)]
    traj: Option<PathBuf>,
    /// Nur jedes N-te Kamerabild loggen
    #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u32).range(1..))]
    every: u32,
    /// Statt den Viewer zu starten in diese .rrd-Datei schreiben (kein Fenster)
    #[arg(long, value_name = "DATEI.rrd")]
    dry_run: Option<PathBuf>,
}

type BoxError = Box<dyn std::error::Error>;

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

/// `/camera_left/image_raw` becomes `camera/left`.
fn camera_entity(topic: &str) -> String {
    let name = topic
        .trim_start_matches('/')
        .split('/')
        .next()
        .unwrap_or("");
    let name = name.strip_prefix("camera_").unwrap_or(name);
    format!("camera/{name}")
}

fn set_stamp(rec: &rerun::RecordingStream, stamp: f64) {
    rec.set_duration_secs(TIMELINE, stamp);
}

fn log_bag(
    rec: &rerun::RecordingStream,
    path: &Path,
    every: u32,
    warnings: &mut Vec<String>,
) -> Result<(), BoxError> {
    let name = path.display();
    let mut reader = BagReader::open(path)?;
    let mut image_count: std::collections::HashMap<String, u32> = Default::default();
    let mut decode_errors = 0u64;
    for item in reader.messages(&[]) {
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
        match msg.topic.as_str() {
            "/vanjee_722z" => match decode_pointcloud2(&msg.data) {
                Ok(f) => {
                    set_stamp(rec, f.stamp);
                    rec.log(
                        "lidar/points",
                        &rerun::Points3D::new(f.points.iter().map(|p| p.xyz))
                            .with_colors(intensity_colors(&f)),
                    )?;
                }
                Err(_) => decode_errors += 1,
            },
            "/vanjee_imu_packets" => match decode_imu(&msg.data) {
                Ok(s) => {
                    set_stamp(rec, s.stamp);
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
                }
                Err(_) => decode_errors += 1,
            },
            t if t.starts_with("/camera_") && t.ends_with("/image_raw") => {
                let n = image_count.entry(msg.topic.clone()).or_insert(0);
                let take = n.is_multiple_of(every);
                *n += 1;
                if !take {
                    continue;
                }
                match decode_compressed_image(&msg.data) {
                    Ok(img) => {
                        set_stamp(rec, img.stamp);
                        rec.log(
                            camera_entity(&msg.topic),
                            &rerun::EncodedImage::from_file_contents(img.data),
                        )?;
                    }
                    Err(_) => decode_errors += 1,
                }
            }
            _ => {}
        }
    }
    if decode_errors > 0 {
        warnings.push(format!("{name}: {decode_errors} Nachrichten nicht lesbar"));
    }
    Ok(())
}

pub fn run(args: &Args) -> Result<(), BoxError> {
    let project = discover_project(&args.folder)?;
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
    for p in &project.problems {
        eprintln!("Hinweis: {p}");
    }
    if let Some(traj) = &args.traj {
        let poses = read_tum(traj)?;
        let line: Vec<[f32; 3]> = poses
            .iter()
            .map(|p| [p.p.x as f32, p.p.y as f32, p.p.z as f32])
            .collect();
        rec.log_static("trajectory", &rerun::LineStrips3D::new([line]))?;
    }
    let mut warnings = Vec::new();
    for bag in project.lidar_bags.iter().chain(&project.image_bags) {
        log_bag(&rec, bag, args.every, &mut warnings)?;
    }
    rec.flush_blocking()?;
    for w in &warnings {
        eprintln!("Warnung: {w}");
    }
    if let Some(file) = &args.dry_run {
        println!("Aufzeichnung geschrieben: {}", file.display());
    }
    Ok(())
}
