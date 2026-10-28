//! `driftless view` in a build without the debug viewer: the default, and so the packages.
//! The viewer (rerun) brings about 290 crates of its own; the desktop program shows the
//! clouds. `cargo build -p driftless-cli --features debug-viewer` builds it in (`main.rs`
//! then takes `view.rs` instead of this module).

use std::ffi::OsString;

/// Whatever `driftless view` is given: the command only says that the viewer is not built in.
#[derive(clap::Args)]
pub struct Args {
    /// Wird nicht ausgewertet
    #[arg(
        trailing_var_arg = true,
        allow_hyphen_values = true,
        value_name = "ARGUMENTE"
    )]
    _ignored: Vec<OsString>,
}

/// What `driftless view` says in a build without the viewer.
pub const NOT_BUILT_IN: &str = "Der Fehlersuche-Viewer ist in diesem Programm nicht enthalten: \
     Punktwolken, Laufbahnen und Oberflächen zeigt die Desktop-App (driftless-app); den alten \
     Viewer baut `cargo build --release -p driftless-cli --features debug-viewer`";

pub fn run(_args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    Err(NOT_BUILT_IN.into())
}
