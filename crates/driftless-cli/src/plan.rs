//! `driftless plan <quelle> [-o <ziel>] [--stockwerk <n>]`: the floor plan of a map or a merged
//! model (`grundriss.json`, `draufsicht.png`). The run itself is
//! [`driftless_steps::plan::run_plan`]; a folder that already holds this plan is not computed
//! again.

use driftless_plan::{metres, Storey};
use driftless_steps::plan::{
    plan_fits, run_plan, PlanOutcome, PlanRequest, PlanSettings, PlanSource,
};
use std::path::PathBuf;

#[derive(clap::Args)]
pub struct Args {
    /// Ordner einer Karte (mit map.json) oder eines Modells (mit modell.json)
    #[arg(value_name = "QUELLE")]
    source: PathBuf,
    /// Zielordner des Grundrisses (Standard: <QUELLE>/grundriss)
    #[arg(short, long, value_name = "ZIEL")]
    output: Option<PathBuf>,
    /// Welches Stockwerk gezeichnet wird, von unten ab 1 gezählt; nötig, wenn die Karte mehrere
    /// hat
    #[arg(long, value_name = "N")]
    stockwerk: Option<usize>,
}

/// One line of the list of storeys: „1 · Boden 0,00 m · Decke 2,60 m · 80 % der Laufbahn“.
fn storey_line(nr: usize, storey: &Storey) -> String {
    let ceiling = storey.ceiling_z.map_or_else(
        || "keine Decke".to_string(),
        |z| format!("Decke {}", metres(z)),
    );
    format!(
        "{nr} · Boden {} · {ceiling} · {:.0} % der Laufbahn",
        metres(storey.floor_z),
        100.0 * storey.pose_share
    )
}

pub fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let source = PlanSource::detect(&args.source)?;
    let out_dir = args
        .output
        .clone()
        .unwrap_or_else(|| args.source.join("grundriss"));
    let req = PlanRequest {
        source,
        model: None,
        out_dir,
        settings: PlanSettings {
            storey: args.stockwerk,
        },
    };
    let progress = crate::console();
    if plan_fits(&req) {
        progress.info(format!(
            "Der Grundriss in {} passt schon, nichts zu rechnen",
            req.out_dir.display()
        ));
        return Ok(());
    }
    match run_plan(&req, &progress)? {
        PlanOutcome::Done(_) => Ok(()),
        PlanOutcome::SeveralStoreys(storeys) => {
            let list: Vec<String> = storeys
                .iter()
                .enumerate()
                .map(|(i, storey)| storey_line(i + 1, storey))
                .collect();
            Err(format!(
                "Mehrere Stockwerke gefunden, geschrieben wurde nichts:\n{}\nEines wählen mit \
                 --stockwerk <N>",
                list.join("\n")
            )
            .into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Parser)]
    struct Cli {
        #[command(flatten)]
        args: Args,
    }

    #[test]
    fn options_parse() {
        let args = Cli::try_parse_from(["plan", "karte"]).unwrap().args;
        assert_eq!(args.source, PathBuf::from("karte"));
        assert_eq!(args.output, None);
        assert_eq!(args.stockwerk, None);
        let argv = ["plan", "karte", "-o", "ziel", "--stockwerk", "2"];
        let args = Cli::try_parse_from(argv).unwrap().args;
        assert_eq!(args.output, Some(PathBuf::from("ziel")));
        assert_eq!(args.stockwerk, Some(2));
        assert!(Cli::try_parse_from(["plan"]).is_err());
    }

    #[test]
    fn a_storey_reads_like_the_app() {
        let storey = Storey {
            floor_z: 0.0,
            ceiling_z: Some(2.6),
            pose_share: 0.8,
            floor_area: 20.0,
        };
        assert_eq!(
            storey_line(1, &storey),
            "1 · Boden 0,00 m · Decke 2,60 m · 80 % der Laufbahn"
        );
        let open = Storey {
            ceiling_z: None,
            ..storey
        };
        assert!(storey_line(2, &open).contains("keine Decke"));
    }
}
