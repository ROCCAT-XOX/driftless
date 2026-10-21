//! Points measured through windows: the LiDAR sees through glass, so a scan of a room with
//! windows holds scraps of the street, the courtyard or the next building.
//!
//! A window band has wall below it (the parapet) and above it (the lintel). Wall voxels are
//! the occupied voxels of the shared grid ([`Occupied`]) whose surface is planar and upright
//! (normal with `|n_z| <` [`WALL_MAX_NZ`]); they are kept per column (x, y) with their z keys.
//! A ray or a line of sight from the sensor to a point goes *through a window* if it crosses
//! a free voxel whose column has wall within [`CleanConfig::window_reach`] above **and**
//! below it, facing the ray (`|n · d| ≥` [`FACING_MIN_COS`]), and then runs on for at least
//! [`CleanConfig::through_beyond`]. A door has floor below it, not wall, and stays.
//!
//! Every ray and every point's line of sight (from the sensor of its own frame) counts, in
//! the occupied voxel it ends in, as "through" or "direct". Points of voxels that were only
//! ever reached through a window are removed; a room behind glass that the walk later enters
//! gets direct counts and stays. Glass down to the floor has no parapet and is not found.
//!
//! The per-ray test is a pure function and the counters are integers, so the result does
//! not depend on the order the rays run in.

use crate::clean::{check_rays, key, vec3, walk_ray, Occupied};
use crate::{CleanConfig, Cloud, RaySource};
use driftless_odom::voxel_map::{FixedState, VoxelKey};
use nalgebra::Vector3;
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};

/// Largest vertical component of the normal of a wall voxel.
const WALL_MAX_NZ: f64 = 0.3;

/// Smallest `|n · d|` between a wall voxel's normal and the ray direction for the wall to
/// frame the ray's window (at most 60° off the wall's normal).
const FACING_MIN_COS: f64 = 0.5;

/// Points per parallel work item when walking the lines of sight.
const POINT_CHUNK: usize = 1 << 12;

/// Wall voxels per column (x, y): z key and normal, sorted by z.
type Walls = HashMap<(i32, i32), Vec<(i32, [f32; 3])>, FixedState>;

fn walls(grid: &Occupied) -> Walls {
    let mut walls: Walls = HashMap::default();
    for (k, n) in grid.keys.iter().zip(&grid.normal) {
        if let Some(n) = n.filter(|n| n.z.abs() < WALL_MAX_NZ) {
            walls
                .entry((k[0], k[1]))
                .or_default()
                .push((k[2], [n.x as f32, n.y as f32, n.z as f32]));
        }
    }
    for col in walls.values_mut() {
        col.sort_unstable_by_key(|&(z, _)| z);
    }
    walls
}

/// What the window test needs besides the ray.
struct Ctx<'a> {
    grid: &'a Occupied,
    walls: Walls,
    /// [`CleanConfig::window_reach`] in voxels.
    reach: i32,
    beyond: f64,
}

/// Whether the ray `from` → `to` goes through a window (see the module documentation).
fn through_ray(from: &[f32; 3], to: &[f32; 3], ctx: &Ctx) -> bool {
    let (o, e) = (vec3(from), vec3(to));
    let len = (e - o).norm();
    if len <= ctx.beyond {
        return false;
    }
    let dir = (e - o) / len;
    let facing = |&(_, n): &(i32, [f32; 3])| {
        Vector3::from(n.map(f64::from)).dot(&dir).abs() >= FACING_MIN_COS
    };
    let voxel = ctx.grid.voxel;
    let mut found = false;
    walk_ray(from, to, voxel, |k: VoxelKey, _| {
        if found {
            return;
        }
        let Some(col) = ctx.walls.get(&(k[0], k[1])) else {
            return;
        };
        if ctx.grid.slot.contains_key(&k) {
            return;
        }
        let centre = Vector3::from(k.map(|c| (f64::from(c) + 0.5) * voxel));
        if (e - centre).norm() < ctx.beyond {
            return;
        }
        let at = col.partition_point(|&(z, _)| z < k[2]);
        let below = col[..at]
            .iter()
            .rev()
            .take_while(|&&(z, _)| k[2] - z <= ctx.reach)
            .any(facing);
        found = below
            && col[at..]
                .iter()
                .take_while(|&&(z, _)| z - k[2] <= ctx.reach)
                .any(facing);
    });
    found
}

/// Counts the ray `from` → `to` as through or direct in its end voxel, if that is occupied.
fn count(from: &[f32; 3], to: &[f32; 3], slot: Option<u32>, ctx: &Ctx, c: &[[AtomicU32; 2]]) {
    if let Some(s) = slot {
        let through = through_ray(from, to, ctx);
        c[s as usize][usize::from(through)].fetch_add(1, Ordering::Relaxed);
    }
}

/// [`find_through`] on the occupied voxels of the cloud.
pub(crate) fn through_flags(
    grid: &Occupied,
    cloud: &Cloud,
    rays: &RaySource,
    cfg: &CleanConfig,
) -> Vec<bool> {
    let ctx = Ctx {
        grid,
        walls: walls(grid),
        reach: (cfg.window_reach / grid.voxel + 1e-9).floor() as i32,
        beyond: cfg.through_beyond,
    };
    // [direct, through] per occupied voxel.
    let counts: Vec<[AtomicU32; 2]> = (0..grid.keys.len())
        .map(|_| [AtomicU32::new(0), AtomicU32::new(0)])
        .collect();
    let fs = &rays.frame_start;
    (0..rays.sensor.len()).into_par_iter().for_each(|f| {
        let sensor = &rays.sensor[f];
        for end in &rays.ends[fs[f] as usize..fs[f + 1] as usize] {
            let slot = grid.slot.get(&key(end, grid.voxel)).copied();
            count(sensor, end, slot, &ctx, &counts);
        }
    });
    (0..cloud.len())
        .into_par_iter()
        .with_min_len(POINT_CHUNK)
        .for_each(|i| {
            let f = cloud.frame[i] as usize;
            let sensor = rays.sensor.get(f).expect("a sensor position per frame");
            count(
                sensor,
                &cloud.xyz[i],
                Some(grid.point_slot[i]),
                &ctx,
                &counts,
            );
        });
    let window_only: Vec<bool> = counts
        .iter()
        .map(|[d, t]| d.load(Ordering::Relaxed) == 0 && t.load(Ordering::Relaxed) >= 1)
        .collect();
    grid.point_slot
        .iter()
        .map(|&s| window_only[s as usize])
        .collect()
}

/// Flags, one per point, of the points measured through a window (see the module
/// documentation): every ray of `rays` and every point's line of sight from the sensor of
/// its frame that ends in the point's voxel (edge [`CleanConfig::ghost_voxel`]) went
/// through a window, and at least one did.
///
/// # Panics
/// If `cfg` is invalid (see [`crate::clean`]), `rays` is inconsistent (see
/// [`crate::find_ghosts`]) or a point's frame has no sensor position in `rays`.
pub fn find_through(cloud: &Cloud, rays: &RaySource, cfg: &CleanConfig) -> Vec<bool> {
    cfg.check().expect("valid CleanConfig");
    check_rays(rays);
    through_flags(
        &Occupied::new(&cloud.xyz, cfg.ghost_voxel),
        cloud,
        rays,
        cfg,
    )
}
