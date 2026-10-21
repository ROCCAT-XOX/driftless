//! Memory of the colouring: its own test binary, because the counting allocator is global
//! and a second test running in parallel would disturb the peak. Exactly one test.

mod common;

use common::*;
use driftless_map::{colorize, ColorConfig};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

/// The system allocator, counting the bytes currently alive and their peak.
struct Counting;

static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn add(n: usize) {
    let live = LIVE.fetch_add(n, Relaxed) + n;
    PEAK.fetch_max(live, Relaxed);
}

// SAFETY: every call is forwarded unchanged to `System`; the counters only observe sizes.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc(layout);
        if !p.is_null() {
            add(layout.size());
        }
        p
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc_zeroed(layout);
        if !p.is_null() {
            add(layout.size());
        }
        p
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
        LIVE.fetch_sub(layout.size(), Relaxed);
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let p = System.realloc(ptr, layout, new_size);
        if !p.is_null() {
            LIVE.fetch_sub(layout.size(), Relaxed);
            add(new_size);
        }
        p
    }
}

#[global_allocator]
static ALLOC: Counting = Counting;

#[test]
fn colorize_live_heap_per_point() {
    let (painted, images, poses, cams) = walk_data();
    let mut cloud = surface_cloud(painted);
    let n = cloud.len();
    let base = LIVE.load(Relaxed);
    PEAK.store(base, Relaxed);
    colorize(
        &mut cloud,
        images.clone(),
        poses,
        cams,
        &ColorConfig::default(),
    )
    .unwrap();
    let peak = PEAK.load(Relaxed) - base;
    let limit = 150 * n + (4 << 20);
    eprintln!(
        "{n} points, peak {peak} B ({:.0} B/point)",
        peak as f64 / n as f64
    );
    assert!(
        peak <= limit,
        "peak {peak} B for {n} points, limit {limit} B"
    );
}
