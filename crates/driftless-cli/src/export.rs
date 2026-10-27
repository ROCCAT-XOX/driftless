//! `driftless export <quelle> -o <datei> [--format <name>] [--ueberschreiben]`: a map, a
//! model, a surface, a run or a point cloud or mesh file in an open format. The run itself is
//! [`driftless_steps::export::run_export`].

use driftless_steps::export::{
    check_export, existing_targets, run_export, ExportFormat, ExportKind, ExportReport,
    ExportRequest, ExportSource,
};
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Karten-, Modell-, Oberflächen- oder Lauf-Ordner, oder eine Datei (.las, .laz, .e57,
    /// .ply, .pcd, .glb, .obj)
    #[arg(value_name = "QUELLE")]
    source: PathBuf,
    /// Zieldatei; das Format folgt der Endung, wenn --format fehlt
    #[arg(short = 'o', long = "out", value_name = "DATEI")]
    out: PathBuf,
    /// Format: las, laz, e57, ply, pcd, glb, obj, ply-netz, tum, csv
    #[arg(long, value_name = "NAME")]
    format: Option<String>,
    /// Vorhandene Zieldateien ersetzen (bei OBJ auch .mtl und Seitenbilder)
    #[arg(long)]
    ueberschreiben: bool,
}

/// `las, laz, …` for messages.
fn names() -> String {
    ExportFormat::ALL
        .iter()
        .map(|f| f.name())
        .collect::<Vec<_>>()
        .join(", ")
}

fn request(args: &Args) -> Result<ExportRequest, Box<dyn std::error::Error>> {
    let source = ExportSource::detect(&args.source)?;
    let format = match &args.format {
        Some(name) => ExportFormat::parse(name)
            .ok_or_else(|| format!("unbekanntes Format „{name}“ – möglich sind: {}", names()))?,
        None => ExportFormat::for_path(&args.out, source.kind()).ok_or_else(|| {
            let ext = args
                .out
                .extension()
                .map(|e| format!(".{}", e.to_string_lossy()))
                .unwrap_or_default();
            format!(
                "unbekannte Endung „{ext}“ – mit --format angeben: {}",
                names()
            )
        })?,
    };
    Ok(ExportRequest {
        source,
        out: args.out.clone(),
        format,
        overwrite: args.ueberschreiben,
    })
}

/// `<count> Punkte|Dreiecke|Lagen → <datei> (<MB> MB, <s> s)`.
fn summary(req: &ExportRequest, report: &ExportReport) -> String {
    let what = match req.format.kind() {
        ExportKind::Points => "Punkte",
        ExportKind::Mesh => "Dreiecke",
        ExportKind::Trajectory => "Lagen",
    };
    format!(
        "{} {what} → {} ({:.1} MB, {:.1} s)",
        report.count,
        req.out.display(),
        report.bytes as f64 / 1e6,
        report.seconds
    )
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let req = request(args)?;
    check_export(&req)?;
    if !req.overwrite {
        if let Some(first) = existing_targets(&req).first() {
            return Err(format!(
                "{} gibt es schon – mit --ueberschreiben ersetzen",
                first.display()
            )
            .into());
        }
    }
    let report = run_export(&req, &crate::console())?;
    println!("{}", summary(&req, &report));
    Ok(())
}
