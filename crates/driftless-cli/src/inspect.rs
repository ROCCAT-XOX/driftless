//! `driftless inspect <ordner> [--json]`: report about a scan folder.

use driftless_io::bag::{BagError, BagReader};
use driftless_io::calib::load_calibration;
use driftless_io::msg::{decode_compressed_image, decode_imu, decode_pointcloud2, MsgError};
use driftless_io::project::{discover_project, Project};
use serde::Serialize;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(clap::Args)]
pub struct Args {
    /// Scan-Ordner (mit LIDAR_*.bag)
    folder: PathBuf,
    /// Ausgabe als JSON
    #[arg(long)]
    json: bool,
}

#[derive(Serialize, Debug, Default, PartialEq)]
pub struct Device {
    pub sn: Option<String>,
    pub model: Option<String>,
}

#[derive(Serialize, Debug)]
pub struct TopicReport {
    pub topic: String,
    #[serde(rename = "type")]
    pub msg_type: String,
    pub count: u64,
    /// `None` with fewer than two messages or a zero time span.
    pub rate_hz: Option<f64>,
    /// Seconds; `None` for a topic without a decodable message.
    pub first: Option<f64>,
    pub last: Option<f64>,
    pub non_monotonic: u64,
    pub decode_errors: u64,
}

#[derive(Serialize, Debug)]
pub struct Report {
    pub device: Option<Device>,
    pub duration_s: f64,
    pub topics: Vec<TopicReport>,
    pub dropped_points: u64,
    pub problems: Vec<String>,
}

/// Running statistics of one topic; only counters, so memory stays constant.
struct Stat {
    topic: String,
    msg_type: String,
    count: u64,
    first: Option<f64>,
    last: Option<f64>,
    prev: Option<f64>,
    non_monotonic: u64,
    decode_errors: u64,
}

impl Stat {
    fn push(&mut self, stamp: f64) {
        self.count += 1;
        self.first = Some(self.first.map_or(stamp, |f| f.min(stamp)));
        self.last = Some(self.last.map_or(stamp, |l| l.max(stamp)));
        if self.prev.is_some_and(|p| stamp < p) {
            self.non_monotonic += 1;
        }
        self.prev = Some(stamp);
    }

    fn into_report(self) -> TopicReport {
        let rate_hz = match (self.first, self.last) {
            (Some(f), Some(l)) if self.count >= 2 && l > f => {
                Some((self.count - 1) as f64 / (l - f))
            }
            _ => None,
        };
        TopicReport {
            topic: self.topic,
            msg_type: self.msg_type,
            count: self.count,
            rate_hz,
            first: self.first,
            last: self.last,
            non_monotonic: self.non_monotonic,
            decode_errors: self.decode_errors,
        }
    }
}

/// Header stamp (seconds) of a message, plus dropped LiDAR points.
fn stamp_of(msg_type: &str, data: &[u8], record_time: f64) -> Result<(f64, u32), MsgError> {
    match msg_type {
        "sensor_msgs/PointCloud2" => decode_pointcloud2(data).map(|f| (f.stamp, f.dropped)),
        "sensor_msgs/Imu" => decode_imu(data).map(|s| (s.stamp, 0)),
        "sensor_msgs/CompressedImage" => decode_compressed_image(data).map(|i| (i.stamp, 0)),
        _ => Ok((record_time, 0)),
    }
}

fn file_name(p: &Path) -> String {
    p.file_name().map_or_else(
        || p.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// Streams one bag into `stats`; errors become entries in `problems`.
fn scan_bag(
    path: &Path,
    stats: &mut Vec<Stat>,
    index: &mut HashMap<String, usize>,
    dropped: &mut u64,
    problems: &mut Vec<String>,
) {
    let name = file_name(path);
    let report_err = |e: &BagError, problems: &mut Vec<String>| match e {
        BagError::Truncated { offset } => problems.push(format!(
            "{name}: Datei abgeschnitten bei Byte {offset} (Kopie unterbrochen?), \
             vollständige Teile davor wurden gelesen"
        )),
        other => problems.push(format!("{name}: {other}")),
    };
    let mut reader = match BagReader::open(path) {
        Ok(r) => r,
        Err(e) => return report_err(&e, problems),
    };
    let types: HashMap<String, String> = reader
        .connections()
        .iter()
        .map(|c| (c.topic.clone(), c.msg_type.clone()))
        .collect();
    for item in reader.messages(&[]) {
        let msg = match item {
            Ok(m) => m,
            Err(e) => {
                report_err(&e, problems);
                return;
            }
        };
        let msg_type = types.get(&msg.topic).map_or("", String::as_str);
        let i = *index.entry(msg.topic.clone()).or_insert_with(|| {
            stats.push(Stat {
                topic: msg.topic.clone(),
                msg_type: msg_type.to_string(),
                count: 0,
                first: None,
                last: None,
                prev: None,
                non_monotonic: 0,
                decode_errors: 0,
            });
            stats.len() - 1
        });
        match stamp_of(msg_type, &msg.data, msg.time.as_secs_f64()) {
            Ok((stamp, d)) => {
                stats[i].push(stamp);
                *dropped += u64::from(d);
            }
            Err(_) => stats[i].decode_errors += 1,
        }
    }
}

pub fn inspect_project(project: &Project) -> Report {
    let mut problems = project.problems.clone();
    let device = project
        .calibration
        .as_ref()
        .and_then(|p| match load_calibration(p) {
            Ok(c) => Some(Device {
                sn: c.device_sn,
                model: c.device_model,
            }),
            Err(e) => {
                problems.push(format!("Kalibrierung (calib.json) nicht lesbar: {e}"));
                None
            }
        });
    let mut stats = Vec::new();
    let mut index = HashMap::new();
    let mut dropped = 0;
    for bag in project.lidar_bags.iter().chain(&project.image_bags) {
        scan_bag(bag, &mut stats, &mut index, &mut dropped, &mut problems);
    }
    let topics: Vec<TopicReport> = stats.into_iter().map(Stat::into_report).collect();
    let first = topics
        .iter()
        .filter_map(|t| t.first)
        .fold(f64::INFINITY, f64::min);
    let last = topics
        .iter()
        .filter_map(|t| t.last)
        .fold(f64::NEG_INFINITY, f64::max);
    let duration_s = if last >= first { last - first } else { 0.0 };
    Report {
        device,
        duration_s,
        topics,
        dropped_points: dropped,
        problems,
    }
}

fn render(r: &Report) -> String {
    let mut s = String::new();
    match &r.device {
        Some(d) => s.push_str(&format!(
            "Gerät: {} (Seriennummer {})\n",
            d.model.as_deref().unwrap_or("unbekannt"),
            d.sn.as_deref().unwrap_or("unbekannt")
        )),
        None => s.push_str("Gerät: unbekannt (keine Kalibrierung)\n"),
    }
    s.push_str(&format!("Dauer: {:.1} s\n", r.duration_s));
    s.push_str(&format!(
        "Verworfene Punkte: {}\n\nTopics:\n",
        r.dropped_points
    ));
    for t in &r.topics {
        let rate = t.rate_hz.map_or("-".to_string(), |h| format!("{h:.2} Hz"));
        let dur = match (t.first, t.last) {
            (Some(f), Some(l)) => format!("{:.1} s", l - f),
            _ => "-".to_string(),
        };
        s.push_str(&format!(
            "  {}  [{}]\n    Anzahl {}, Rate {}, Dauer {}\n",
            t.topic, t.msg_type, t.count, rate, dur
        ));
        if t.non_monotonic > 0 {
            s.push_str(&format!(
                "    Warnung: {} Zeitstempel nicht monoton\n",
                t.non_monotonic
            ));
        }
        if t.decode_errors > 0 {
            s.push_str(&format!(
                "    Warnung: {} Nachrichten nicht lesbar\n",
                t.decode_errors
            ));
        }
    }
    s.push_str("\nProbleme:\n");
    if r.problems.is_empty() {
        s.push_str("  keine\n");
    }
    for p in &r.problems {
        s.push_str(&format!("  - {p}\n"));
    }
    s
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let project = discover_project(&args.folder)?;
    let report = inspect_project(&project);
    if args.json {
        println!("{}", serde_json::to_string_pretty(&report)?);
    } else {
        print!("{}", render(&report));
    }
    Ok(())
}
