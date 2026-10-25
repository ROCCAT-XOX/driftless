//! Colouring on the GPU against the CPU reference, on the synthetic walk of the colour tests.

mod common;

use common::*;
use driftless_gpu::GpuContext;
use driftless_io::progress::Progress;
use driftless_map::{colorize, colorize_gpu_batched, colorize_with, Cloud, ColorConfig, MapError};

/// A GPU context (software adapters allowed), or `None` with a note when there is none.
fn gpu() -> Option<GpuContext> {
    match GpuContext::new_for_tests() {
        Some(ctx) => {
            eprintln!("GPU-Test auf {:?}", ctx.info());
            Some(ctx)
        }
        None => {
            eprintln!("übersprungen: kein Grafikadapter vorhanden");
            None
        }
    }
}

/// Points whose colour differs by more than `tol` sRGB levels in some channel, the largest
/// difference, and the points coloured in `a` and in `b`.
fn compare(a: &Cloud, b: &Cloud, tol: i32) -> (usize, i32, usize, usize) {
    let (ra, rb) = (a.rgb.as_ref().unwrap(), b.rgb.as_ref().unwrap());
    let (ca, cb) = (a.colored.as_ref().unwrap(), b.colored.as_ref().unwrap());
    let mut off = 0;
    let mut max = 0;
    for i in 0..a.len() {
        let d = (0..3)
            .map(|c| (i32::from(ra[i][c]) - i32::from(rb[i][c])).abs())
            .max()
            .unwrap();
        max = max.max(d);
        if d > tol || ca[i] != cb[i] {
            off += 1;
        }
    }
    let count = |c: &[bool]| c.iter().filter(|&&x| x).count();
    (off, max, count(ca), count(cb))
}

#[test]
fn colorize_gpu_matches_cpu() {
    let Some(ctx) = gpu() else { return };
    let (painted, images, poses, cams) = walk_data();
    let cfg = ColorConfig::default();
    let mut cpu = surface_cloud(painted);
    let cpu_stats = colorize(&mut cpu, images.clone(), poses, cams, &cfg).unwrap();
    let mut on_gpu = surface_cloud(painted);
    let gpu_stats = colorize_with(
        &mut on_gpu,
        images.clone(),
        poses,
        cams,
        &cfg,
        &Progress::new(),
        Some(&ctx),
    )
    .unwrap();
    let n = cpu.len();
    let (off, max, colored_cpu, colored_gpu) = compare(&cpu, &on_gpu, 2);
    println!(
        "{n} Punkte: {off} weichen um mehr als 2 Stufen ab (größte {max}); gefärbt CPU \
         {colored_cpu}, GPU {colored_gpu}"
    );
    assert!(
        off as f64 <= 0.01 * n as f64,
        "{off} of {n} off by more than 2"
    );
    assert!(
        (colored_cpu as f64 - colored_gpu as f64).abs() <= 0.001 * n as f64,
        "coloured {colored_cpu} vs {colored_gpu} of {n}"
    );
    assert_eq!(gpu_stats.images_used, cpu_stats.images_used);
    assert_eq!(gpu_stats.points_colored, colored_gpu);
    color_match(painted, &on_gpu).assert_good();

    // Same device, same result.
    let mut again = surface_cloud(painted);
    let again_stats = colorize_with(
        &mut again,
        images.clone(),
        poses,
        cams,
        &cfg,
        &Progress::new(),
        Some(&ctx),
    )
    .unwrap();
    assert_eq!(again.rgb, on_gpu.rgb);
    assert_eq!(again.colored, on_gpu.colored);
    assert_eq!(again_stats, gpu_stats);
}

#[test]
fn colorize_gpu_batches_large_cloud() {
    let Some(ctx) = gpu() else { return };
    let (painted, images, poses, cams) = walk_data();
    let cfg = ColorConfig::default();
    let mut one = surface_cloud(painted);
    let one_stats = colorize_with(
        &mut one,
        images.clone(),
        poses,
        cams,
        &cfg,
        &Progress::new(),
        Some(&ctx),
    )
    .unwrap();
    let batch = 7_777;
    assert!(one.len() > 3 * batch, "{} points", one.len());
    let mut many = surface_cloud(painted);
    let many_stats = colorize_gpu_batched(
        &mut many,
        images.clone(),
        poses,
        cams,
        &cfg,
        &Progress::new(),
        &ctx,
        batch,
    )
    .unwrap();
    assert_eq!(many.rgb, one.rgb);
    assert_eq!(many.colored, one.colored);
    assert_eq!(many_stats, one_stats);
}

#[test]
fn cancel_during_gpu_pass() {
    let Some(ctx) = gpu() else { return };
    let (painted, images, poses, cams) = walk_data();
    let before = surface_cloud(painted);
    let p = Progress::new();
    let q = p.clone();
    let p = p.with_observer(move |s| {
        if s.phase == "Einfärben" && s.done == 2 {
            q.cancel();
        }
    });
    let mut cloud = before.clone();
    let r = colorize_with(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
        &p,
        Some(&ctx),
    );
    assert!(matches!(r, Err(MapError::Cancelled)), "{r:?}");
    assert_eq!(cloud, before);
    assert_eq!(p.snapshot().phase, "Einfärben");
    assert_eq!(p.snapshot().done, 2, "stops at the next image");
}

#[test]
fn auto_falls_back_when_budget_exceeded() {
    let Some(ctx) = gpu() else { return };
    let ctx = ctx.with_budget(1000);
    assert!(ctx.falls_back());
    let (painted, images, poses, cams) = walk_data();
    let cfg = ColorConfig::default();
    let mut cpu = surface_cloud(painted);
    let cpu_stats = colorize(&mut cpu, images.clone(), poses, cams, &cfg).unwrap();
    let log = Progress::new();
    let mut fallback = surface_cloud(painted);
    let stats = colorize_with(
        &mut fallback,
        images.clone(),
        poses,
        cams,
        &cfg,
        &log,
        Some(&ctx),
    )
    .unwrap();
    assert_eq!(fallback, cpu, "the CPU path, byte for byte");
    let why = stats.gpu_fallback.clone().expect("fallback recorded");
    assert!(why.contains("Grafikspeicher"), "{why}");
    assert_eq!(
        driftless_map::ColorStats {
            gpu_fallback: None,
            ..stats
        },
        cpu_stats
    );
    let lines: Vec<String> = log
        .take_log()
        .into_iter()
        .map(|l| match l {
            driftless_io::progress::LogLine::Info(t)
            | driftless_io::progress::LogLine::Warning(t) => t,
        })
        .collect();
    assert!(
        lines
            .iter()
            .any(|l| l.starts_with("Einfärben auf der CPU:")),
        "{lines:?}"
    );
}

#[test]
fn on_fails_cleanly_when_budget_exceeded() {
    let Some(ctx) = gpu() else { return };
    let ctx = ctx.with_budget(1000).with_fallback(false);
    let (painted, images, poses, cams) = walk_data();
    let before = surface_cloud(painted);
    let mut cloud = before.clone();
    let r = colorize_with(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
        &Progress::new(),
        Some(&ctx),
    );
    match r {
        Err(MapError::Gpu(why)) => assert!(why.contains("Grafikspeicher"), "{why}"),
        other => panic!("{other:?}"),
    }
    assert_eq!(cloud, before);
}
