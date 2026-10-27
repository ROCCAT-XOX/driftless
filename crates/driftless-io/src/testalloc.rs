//! A counting allocator for the memory tests. A test binary installs it as its global
//! allocator and then measures with [`peak_during`]:
//!
//! ```ignore
//! #[global_allocator]
//! static ALLOC: driftless_io::testalloc::Counting = driftless_io::testalloc::Counting;
//! ```
//!
//! Such a binary must hold exactly one test, so that no second test running in parallel
//! disturbs the peak.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

/// The system allocator, counting the bytes currently alive and their peak.
pub struct Counting;

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

/// Peak of the bytes allocated in addition to those alive when `f` starts, and its result.
/// Only meaningful in a binary whose global allocator is [`Counting`].
pub fn peak_during<T>(f: impl FnOnce() -> T) -> (usize, T) {
    let base = LIVE.load(Relaxed);
    PEAK.store(base, Relaxed);
    let out = f();
    (PEAK.load(Relaxed).saturating_sub(base), out)
}
