//! Adapter discovery and device creation.

use std::fmt;
use std::str::FromStr;
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

/// Most GPU memory a run may use (bytes): the 4 GB of the design for a room scan.
pub const MAX_GPU_BYTES: u64 = 4 << 30;
/// Environment variable that lowers the budget (bytes), e.g. to test the CPU fallback.
pub const BUDGET_ENV: &str = "DRIFTLESS_GPU_BUDGET";

/// Whether to compute on the GPU (`--gpu auto|an|aus`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GpuPref {
    /// Use a usable hardware GPU if there is one, else the CPU.
    #[default]
    Auto,
    /// Require a GPU; error without one.
    On,
    /// Always the CPU.
    Off,
}

impl FromStr for GpuPref {
    type Err = String;

    fn from_str(s: &str) -> Result<GpuPref, String> {
        match s.trim().to_lowercase().as_str() {
            "auto" => Ok(GpuPref::Auto),
            "an" => Ok(GpuPref::On),
            "aus" => Ok(GpuPref::Off),
            other => Err(format!(
                "Unbekannte Einstellung „{other}“ für die Grafikkarte (erlaubt: auto|an|aus)"
            )),
        }
    }
}

impl fmt::Display for GpuPref {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            GpuPref::Auto => "auto",
            GpuPref::On => "an",
            GpuPref::Off => "aus",
        })
    }
}

/// The adapter a [`GpuContext`] runs on (for reports).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpuInfo {
    pub name: String,
    /// `vulkan`, `metal`, `dx12`, ...
    pub backend: String,
    pub driver: String,
    /// Software rasteriser (lavapipe, WARP, llvmpipe): only used by tests.
    pub software: bool,
}

#[derive(Debug, thiserror::Error)]
pub enum GpuError {
    #[error("Keine taugliche Grafikkarte gefunden (--gpu an verlangt eine)")]
    NoAdapter,
    #[error("Grafikkarte „{name}“ ließ sich nicht öffnen: {reason}")]
    Device { name: String, reason: String },
}

/// Why a run computes on the CPU although `--gpu auto` asked for the GPU (German, for the
/// notice on the console).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CpuFallback {
    /// `--gpu aus`.
    Off,
    /// No adapter at all.
    NoAdapter,
    /// Only software rasterisers (names listed).
    OnlySoftware(Vec<String>),
    /// The chosen adapter did not open (e.g. held by a VM).
    Device { name: String, reason: String },
}

impl fmt::Display for CpuFallback {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CpuFallback::Off => f.write_str("Grafikkarte abgeschaltet (--gpu aus)"),
            CpuFallback::NoAdapter => f.write_str("keine Grafikkarte gefunden"),
            CpuFallback::OnlySoftware(names) => write!(
                f,
                "nur Software-Grafik vorhanden ({}), die wäre langsamer als die CPU",
                names.join(", ")
            ),
            CpuFallback::Device { name, reason } => write!(
                f,
                "Grafikkarte „{name}“ ließ sich nicht öffnen (belegt, z. B. von einer anderen \
                 VM?): {reason}"
            ),
        }
    }
}

/// What [`GpuContext::open`] found: a device, or why the CPU computes.
pub enum GpuChoice {
    Gpu(GpuContext),
    Cpu(CpuFallback),
}

/// An open GPU device with its queue.
pub struct GpuContext {
    info: GpuInfo,
    device: wgpu::Device,
    queue: wgpu::Queue,
    pub(crate) project_pipeline: OnceLock<wgpu::ComputePipeline>,
    pub(crate) visibility_pipelines: OnceLock<crate::visibility::Pipelines>,
    pub(crate) views_pipeline: OnceLock<crate::views::Pipeline>,
    pub(crate) faces_pipelines: OnceLock<crate::faces::Pipelines>,
    /// First uncaptured error or loss of the device (German), see [`GpuContext::error`].
    error: Arc<Mutex<Option<String>>>,
    budget: u64,
    falls_back: bool,
}

/// Why a run on the GPU stopped or could not start (German).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GpuRunError {
    #[error("Grafikkarte: braucht etwa {need_mb} MB Grafikspeicher, erlaubt sind {budget_mb} MB")]
    Budget { need_mb: u64, budget_mb: u64 },
    #[error("Grafikkarte: {0}")]
    Device(String),
    /// Input the kernels cannot take (a caller's mistake, reported instead of a panic).
    #[error("Grafikkarte: ungültige Eingabe ({0})")]
    Input(String),
}

impl GpuContext {
    /// Opens the best usable adapter according to `pref`: `Off` and `Auto` without a usable
    /// hardware adapter give `Ok(None)`, `On` without one an error.
    pub fn new(pref: GpuPref) -> Result<Option<GpuContext>, GpuError> {
        Ok(match GpuContext::open(pref)? {
            GpuChoice::Gpu(ctx) => Some(ctx),
            GpuChoice::Cpu(_) => None,
        })
    }

    /// Like [`GpuContext::new`], but says why the CPU computes when there is no device.
    pub fn open(pref: GpuPref) -> Result<GpuChoice, GpuError> {
        if pref == GpuPref::Off {
            return Ok(GpuChoice::Cpu(CpuFallback::Off));
        }
        open(pref, false)
    }

    /// Like [`GpuContext::new`] with `Auto`, but software adapters count too (GPU tests in
    /// CI). `None` if there is no adapter at all.
    pub fn new_for_tests() -> Option<GpuContext> {
        match open(GpuPref::Auto, true) {
            Ok(GpuChoice::Gpu(ctx)) => Some(ctx),
            _ => None,
        }
    }

    pub fn info(&self) -> &GpuInfo {
        &self.info
    }

    /// Most GPU memory a run may use (bytes): [`MAX_GPU_BYTES`], lowered by [`BUDGET_ENV`].
    pub fn budget(&self) -> u64 {
        self.budget
    }

    /// The context with a budget of `bytes` (at most [`MAX_GPU_BYTES`]).
    pub fn with_budget(mut self, bytes: u64) -> GpuContext {
        self.budget = bytes.min(MAX_GPU_BYTES);
        self
    }

    /// Whether a run that cannot start on the GPU takes the CPU instead (`--gpu auto`; also
    /// for [`GpuContext::new_for_tests`]) or fails (`--gpu an`).
    pub fn falls_back(&self) -> bool {
        self.falls_back
    }

    pub fn with_fallback(mut self, falls_back: bool) -> GpuContext {
        self.falls_back = falls_back;
        self
    }

    /// Fails if `bytes` of GPU memory exceed the budget.
    pub fn check_budget(&self, bytes: u64) -> Result<(), GpuRunError> {
        if bytes > self.budget {
            return Err(GpuRunError::Budget {
                need_mb: bytes.div_ceil(1_000_000),
                budget_mb: self.budget / 1_000_000,
            });
        }
        Ok(())
    }

    /// The first error the device reported, or its loss (stays set: a device that failed
    /// once is not trusted again).
    pub fn error(&self) -> Option<String> {
        self.error
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }
}

/// Enumerates the adapters of the primary backends (Vulkan, Metal, DX12; `WGPU_BACKEND`
/// overrides) and opens the one [`choose`] picks.
fn open(pref: GpuPref, allow_software: bool) -> Result<GpuChoice, GpuError> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::from_env().unwrap_or(wgpu::Backends::PRIMARY),
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::all()));
    let infos: Vec<wgpu::AdapterInfo> = adapters.iter().map(wgpu::Adapter::get_info).collect();
    let Some(index) = choose(pref, &infos, allow_software)? else {
        return Ok(GpuChoice::Cpu(fallback(&infos)));
    };
    let adapter = &adapters[index];
    let raw = &infos[index];
    let info = GpuInfo {
        name: raw.name.clone(),
        backend: raw.backend.to_str().to_string(),
        driver: [raw.driver.as_str(), raw.driver_info.as_str()]
            .into_iter()
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join(" "),
        software: is_software(raw),
    };
    let opened = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("Driftless Rechnen"),
        // Everything the adapter offers: large buffers for whole scans.
        required_limits: adapter.limits(),
        ..Default::default()
    }));
    match opened {
        Ok((device, queue)) => {
            // Errors and a lost device are recorded instead of panicking (wgpu's default); the
            // runs check them with `take_error`.
            let error = Arc::new(Mutex::new(None::<String>));
            let first = |slot: &Mutex<Option<String>>, text: String| {
                slot.lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .get_or_insert(text);
            };
            let on_error = error.clone();
            device.on_uncaptured_error(Arc::new(move |e: wgpu::Error| {
                first(&on_error, format!("Fehler auf der Grafikkarte: {e}"));
            }));
            let on_lost = error.clone();
            device.set_device_lost_callback(move |reason, message| {
                first(
                    &on_lost,
                    format!("Grafikkarte nicht mehr verfügbar ({reason:?}): {message}"),
                );
            });
            let budget = std::env::var(BUDGET_ENV)
                .ok()
                .and_then(|v| v.trim().parse::<u64>().ok())
                .map_or(MAX_GPU_BYTES, |b| b.min(MAX_GPU_BYTES));
            Ok(GpuChoice::Gpu(GpuContext {
                info,
                device,
                queue,
                project_pipeline: OnceLock::new(),
                visibility_pipelines: OnceLock::new(),
                views_pipeline: OnceLock::new(),
                faces_pipelines: OnceLock::new(),
                error,
                budget,
                falls_back: pref == GpuPref::Auto,
            }))
        }
        // `Auto` falls back to the CPU when the device cannot be opened (e.g. held by a VM).
        Err(e) if pref == GpuPref::Auto => Ok(GpuChoice::Cpu(CpuFallback::Device {
            name: info.name,
            reason: e.to_string(),
        })),
        Err(e) => Err(GpuError::Device {
            name: info.name,
            reason: e.to_string(),
        }),
    }
}

/// Software rasterisers (lavapipe, llvmpipe, WARP, SwiftShader).
fn is_software(info: &wgpu::AdapterInfo) -> bool {
    let name = info.name.to_lowercase();
    info.device_type == wgpu::DeviceType::Cpu
        || [
            "llvmpipe",
            "lavapipe",
            "swiftshader",
            "microsoft basic render",
        ]
        .iter()
        .any(|s| name.contains(s))
}

/// Why no adapter of `infos` is used (without software adapters).
fn fallback(infos: &[wgpu::AdapterInfo]) -> CpuFallback {
    if infos.is_empty() {
        CpuFallback::NoAdapter
    } else {
        CpuFallback::OnlySoftware(infos.iter().map(|i| i.name.clone()).collect())
    }
}

/// Index of the adapter to use among `infos`, or `None` (see [`GpuContext::new`]): the
/// first discrete GPU, else integrated, else virtual/other; software only if allowed.
fn choose(
    pref: GpuPref,
    infos: &[wgpu::AdapterInfo],
    allow_software: bool,
) -> Result<Option<usize>, GpuError> {
    if pref == GpuPref::Off {
        return Ok(None);
    }
    let rank = |info: &wgpu::AdapterInfo| {
        if is_software(info) {
            return allow_software.then_some(0);
        }
        Some(match info.device_type {
            wgpu::DeviceType::DiscreteGpu => 4,
            wgpu::DeviceType::IntegratedGpu => 3,
            wgpu::DeviceType::VirtualGpu => 2,
            wgpu::DeviceType::Other | wgpu::DeviceType::Cpu => 1,
        })
    };
    // `max_by_key` keeps the last of equal maxima; reversing keeps the first instead.
    let best = infos
        .iter()
        .enumerate()
        .rev()
        .filter_map(|(i, info)| rank(info).map(|r| (i, r)))
        .max_by_key(|&(_, r)| r)
        .map(|(i, _)| i);
    match (best, pref) {
        (None, GpuPref::On) => Err(GpuError::NoAdapter),
        (best, _) => Ok(best),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adapter(name: &str, device_type: wgpu::DeviceType) -> wgpu::AdapterInfo {
        wgpu::AdapterInfo {
            name: name.into(),
            ..wgpu::AdapterInfo::new(device_type, wgpu::Backend::Vulkan)
        }
    }

    #[test]
    fn pref_parses() {
        assert_eq!("auto".parse(), Ok(GpuPref::Auto));
        assert_eq!("an".parse(), Ok(GpuPref::On));
        assert_eq!("aus".parse(), Ok(GpuPref::Off));
        assert_eq!(" AN ".parse(), Ok(GpuPref::On));
        let err = "ja".parse::<GpuPref>().unwrap_err();
        assert!(err.contains("auto|an|aus"), "{err}");
        for p in [GpuPref::Auto, GpuPref::On, GpuPref::Off] {
            assert_eq!(p.to_string().parse(), Ok(p));
        }
    }

    #[test]
    fn auto_without_adapter_falls_back() {
        assert!(GpuContext::new(GpuPref::Off).unwrap().is_none());
        assert_eq!(choose(GpuPref::Auto, &[], false).unwrap(), None);
        assert!(matches!(
            choose(GpuPref::On, &[], false),
            Err(GpuError::NoAdapter)
        ));
        // Software adapters do not count for `Auto`/`On`, only for tests.
        let soft = [adapter(
            "llvmpipe (LLVM 17.0.6, 256 bits)",
            wgpu::DeviceType::Cpu,
        )];
        assert_eq!(choose(GpuPref::Auto, &soft, false).unwrap(), None);
        assert!(choose(GpuPref::On, &soft, false).is_err());
        assert_eq!(choose(GpuPref::Auto, &soft, true).unwrap(), Some(0));
        assert_eq!(choose(GpuPref::Off, &soft, true).unwrap(), None);
        // A discrete GPU beats an integrated one, regardless of order.
        let both = [
            adapter("Intel UHD", wgpu::DeviceType::IntegratedGpu),
            adapter("NVIDIA GeForce RTX 5080", wgpu::DeviceType::DiscreteGpu),
            adapter("lavapipe", wgpu::DeviceType::Cpu),
        ];
        assert_eq!(choose(GpuPref::Auto, &both, true).unwrap(), Some(1));
    }

    #[test]
    fn fallback_says_why() {
        assert!(matches!(
            GpuContext::open(GpuPref::Off),
            Ok(GpuChoice::Cpu(CpuFallback::Off))
        ));
        assert_eq!(fallback(&[]), CpuFallback::NoAdapter);
        let soft = [adapter("llvmpipe (LLVM 17)", wgpu::DeviceType::Cpu)];
        let why = fallback(&soft);
        assert_eq!(
            why,
            CpuFallback::OnlySoftware(vec!["llvmpipe (LLVM 17)".into()])
        );
        assert!(why.to_string().contains("Software"), "{why}");
        let held = CpuFallback::Device {
            name: "NVIDIA GeForce RTX 5080".into(),
            reason: "device lost".into(),
        };
        assert!(held.to_string().contains("RTX 5080"), "{held}");
        assert!(held.to_string().contains("nicht öffnen"), "{held}");
        assert_eq!(
            CpuFallback::NoAdapter.to_string(),
            "keine Grafikkarte gefunden"
        );
    }
}
