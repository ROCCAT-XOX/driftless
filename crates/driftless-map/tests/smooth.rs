//! Thinner walls: projection onto the local plane, on synthetic walls, corners and rods.

use driftless_map::{clean, smooth_surfaces, CleanConfig, Cloud, RaySource, SmoothConfig};

/// Deterministic pseudo-random numbers.
struct Lcg(u64);

impl Lcg {
    /// Uniform in [0, 1).
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Normal with standard deviation `sigma` (Box–Muller).
    fn gauss(&mut self, sigma: f64) -> f64 {
        let (u, v) = (self.next().max(1e-12), self.next());
        sigma * (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

fn p32(x: f64, y: f64, z: f64) -> [f32; 3] {
    [x as f32, y as f32, z as f32]
}

/// Wall y = 2 (x 0–2, z 0–2) on a jittered 1.25 cm grid, noise across the wall within
/// ±2 cm: normal with a standard deviation of 7 mm (Büro: 6.5 mm), cut at ±2 cm.
fn noisy_wall(seed: u64) -> Vec<[f32; 3]> {
    let mut rng = Lcg(seed);
    let mut xyz = Vec::new();
    for i in 0..160 {
        for k in 0..160 {
            let x = 0.0125 * (f64::from(i) + rng.next());
            let z = 0.0125 * (f64::from(k) + rng.next());
            xyz.push(p32(x, 2.0 + rng.gauss(0.007).clamp(-0.02, 0.02), z));
        }
    }
    xyz
}

#[test]
fn thins_noisy_wall() {
    let xyz = noisy_wall(7);
    let (out, stats) = smooth_surfaces(&xyz, &SmoothConfig::default());
    assert_eq!(out.len(), xyz.len());
    assert!(
        stats.moved > xyz.len() * 3 / 4,
        "{} of {}",
        stats.moved,
        xyz.len()
    );
    assert!(
        stats.thickness_before > 0.006 && stats.thickness_before < 0.008,
        "{stats:?}"
    );
    assert!(
        stats.thickness_after < 0.4 * stats.thickness_before,
        "{stats:?}"
    );
    // The wall stays where it was: only y changes, and the points come closer to y = 2.
    let rms = |pts: &[[f32; 3]]| {
        (pts.iter()
            .map(|p| (f64::from(p[1]) - 2.0).powi(2))
            .sum::<f64>()
            / pts.len() as f64)
            .sqrt()
    };
    assert!(rms(&out) < 0.4 * rms(&xyz), "{} {}", rms(&out), rms(&xyz));
    for (a, b) in xyz.iter().zip(&out) {
        assert!(
            (a[0] - b[0]).abs() < 2e-3 && (a[2] - b[2]).abs() < 2e-3,
            "{a:?} {b:?}"
        );
    }
}

/// Distance from `p` to the corner of the walls x = 0 (y 0–1) and y = 0 (x 0–1).
fn corner_distance(p: &[f32; 3]) -> f64 {
    let (x, y) = (f64::from(p[0]), f64::from(p[1]));
    let to_x0 = if y >= 0.0 { x.abs() } else { x.hypot(y) };
    let to_y0 = if x >= 0.0 { y.abs() } else { x.hypot(y) };
    to_x0.min(to_y0)
}

/// Distance from `p` to the surface of the rod of radius 1 cm along z through (2, 2).
fn rod_distance(p: &[f32; 3]) -> f64 {
    let (dx, dy) = (f64::from(p[0]) - 2.0, f64::from(p[1]) - 2.0);
    ((dx * dx + dy * dy).sqrt() - 0.01).abs()
}

#[test]
fn keeps_corners_and_thin_rods() {
    let mut rng = Lcg(11);
    let mut corner = Vec::new();
    // 90° corner, 1 cm grid, 1 mm noise across the walls.
    for i in 0..100 {
        for k in 0..100 {
            let (a, z) = (0.01 * f64::from(i) + 0.005, 0.01 * f64::from(k));
            corner.push(p32(rng.gauss(0.001), a, z));
            corner.push(p32(a, rng.gauss(0.001), z));
        }
    }
    // Rod of 2 cm diameter, 20 points around, 5 mm apart along it.
    let mut rod = Vec::new();
    for k in 0..200 {
        for j in 0..20 {
            let a = std::f64::consts::TAU * (f64::from(j) + 0.5 * f64::from(k % 2)) / 20.0;
            let r = 0.01 + rng.gauss(0.001);
            rod.push(p32(
                2.0 + r * a.cos(),
                2.0 + r * a.sin(),
                0.005 * f64::from(k),
            ));
        }
    }
    let xyz: Vec<[f32; 3]> = corner.iter().chain(&rod).copied().collect();
    let (out, _) = smooth_surfaces(&xyz, &SmoothConfig::default());

    let worst = |pts: &[[f32; 3]], d: fn(&[f32; 3]) -> f64| pts.iter().map(d).fold(0.0, f64::max);
    let (c_in, c_out) = (&xyz[..corner.len()], &out[..corner.len()]);
    let (r_in, r_out) = (&xyz[corner.len()..], &out[corner.len()..]);
    assert!(worst(c_in, corner_distance) <= 0.005);
    assert!(worst(r_in, rod_distance) <= 0.005);
    assert!(
        worst(c_out, corner_distance) <= 0.005,
        "{}",
        worst(c_out, corner_distance)
    );
    assert!(
        worst(r_out, rod_distance) <= 0.005,
        "{}",
        worst(r_out, rod_distance)
    );
    // The edge is not rounded: points within 9 cm of it (where the neighbourhood holds
    // points of both walls) come no further from the true corner than they were.
    let near: Vec<(f64, f64)> = c_in
        .iter()
        .zip(c_out)
        .filter(|(p, _)| p[0].abs().max(p[1].abs()) < 0.09)
        .map(|(a, b)| (corner_distance(a), corner_distance(b)))
        .collect();
    assert!(near.len() > 1500);
    let mean = |f: fn(&(f64, f64)) -> f64| near.iter().map(f).sum::<f64>() / near.len() as f64;
    let (before, after) = (mean(|d| d.0), mean(|d| d.1));
    assert!(after <= before + 1e-4, "edge {before} → {after}");
    for &(b, a) in &near {
        assert!(
            a <= b.max(0.001) + 5e-4,
            "edge point {b} → {a} from the corner"
        );
    }
    // The rod is not planar: it is left alone.
    let rod_moved = r_in.iter().zip(r_out).filter(|(a, b)| a != b).count();
    assert!(rod_moved < r_in.len() / 100, "{rod_moved} rod points moved");
}

#[test]
fn shift_is_capped() {
    // Flat floor z = 0 on a 2.5 cm grid, one point 4 cm above it in the middle: its
    // neighbourhood is still planar, the plane is the floor.
    let mut xyz = Vec::new();
    for i in 0..40 {
        for j in 0..40 {
            xyz.push(p32(0.025 * f64::from(i), 0.025 * f64::from(j), 0.0));
        }
    }
    let above = xyz.len();
    xyz.push(p32(0.5125, 0.5125, 0.04));
    let cfg = SmoothConfig::default();
    let (out, stats) = smooth_surfaces(&xyz, &cfg);
    let dz = f64::from(xyz[above][2] - out[above][2]);
    assert!(
        (dz - cfg.max_shift).abs() < 1e-4,
        "moved {dz} instead of {}",
        cfg.max_shift
    );
    assert!(stats.moved >= 1);
    for (a, b) in xyz.iter().zip(&out) {
        let d: f64 = (0..3)
            .map(|i| f64::from(a[i] - b[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(d <= cfg.max_shift + 1e-6, "{d}");
    }

    // A smaller cap holds too.
    let small = SmoothConfig {
        max_shift: 0.01,
        ..SmoothConfig::default()
    };
    let (out, _) = smooth_surfaces(&xyz, &small);
    let dz = f64::from(xyz[above][2] - out[above][2]);
    assert!((dz - 0.01).abs() < 1e-4, "{dz}");

    // 8 cm above the floor the neighbourhood is not planar: the point stays.
    xyz[above][2] = 0.08;
    let (out, _) = smooth_surfaces(&xyz, &cfg);
    assert_eq!(out[above], xyz[above]);
}

#[test]
fn smooth_is_deterministic() {
    let xyz = noisy_wall(3);
    let cfg = SmoothConfig::default();
    let a = smooth_surfaces(&xyz, &cfg);
    let b = smooth_surfaces(&xyz, &cfg);
    assert_eq!(a, b);

    // Each point is moved from the input coordinates and neighbours at the same distance are
    // ordered by their coordinates, so the order of the points does not matter, also on a
    // grid full of equal distances.
    let mut grid = Vec::new();
    for i in 0..60u32 {
        for j in 0..60u32 {
            let z = 0.003 * f64::from((i * 7 + j * 3) % 5);
            grid.push(p32(0.01 * f64::from(i), 0.01 * f64::from(j), z));
        }
    }
    for pts in [xyz, grid] {
        let (out, _) = smooth_surfaces(&pts, &cfg);
        // A fixed permutation: reversed, then every 7th point first.
        let n = pts.len();
        let order: Vec<usize> = (0..n)
            .rev()
            .filter(|i| i % 7 == 0)
            .chain((0..n).rev().filter(|i| i % 7 != 0))
            .collect();
        let shuffled: Vec<[f32; 3]> = order.iter().map(|&i| pts[i]).collect();
        let (out_s, _) = smooth_surfaces(&shuffled, &cfg);
        for (k, &i) in order.iter().enumerate() {
            assert_eq!(out_s[k], out[i], "point {i}");
        }
    }
}

#[test]
fn disabled_or_sparse_leaves_points() {
    let xyz = noisy_wall(5);
    let off = SmoothConfig {
        enabled: false,
        ..SmoothConfig::default()
    };
    let (out, stats) = smooth_surfaces(&xyz, &off);
    assert_eq!(out, xyz);
    assert_eq!(stats.moved, 0);

    // Fewer neighbours than needed: nothing moves.
    let few: Vec<[f32; 3]> = xyz.iter().step_by(400).copied().collect();
    let (out, stats) = smooth_surfaces(&few, &SmoothConfig::default());
    assert_eq!(out, few);
    assert_eq!(stats.moved, 0);
    assert!(smooth_surfaces(&[], &SmoothConfig::default()).0.is_empty());
}

fn cloud(xyz: Vec<[f32; 3]>) -> Cloud {
    let n = xyz.len();
    Cloud {
        xyz,
        intensity: vec![1.0; n],
        t: vec![0.0; n],
        frame: vec![0; n],
        rgb: None,
        colored: None,
    }
}

#[test]
fn clean_smooths_kept_points_only() {
    let mut xyz = noisy_wall(9);
    // A stray point far off the wall: removed as an outlier, and written as it was.
    xyz.push(p32(1.5, 3.5, 1.0));
    let rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    let cfg = CleanConfig {
        ghosts: false,
        through: false,
        ..CleanConfig::default()
    };
    let (kept, removed, _, stats) = clean(cloud(xyz.clone()), &rays, &cfg).unwrap();
    assert!(removed.xyz.contains(&p32(1.5, 3.5, 1.0)));
    let raw: Vec<[f32; 3]> = xyz
        .iter()
        .filter(|p| !removed.xyz.contains(p))
        .copied()
        .collect();
    let (want, want_stats) = smooth_surfaces(&raw, &cfg.smooth);
    assert_eq!(kept.xyz, want);
    assert_eq!(stats.smooth, want_stats);
    assert!(stats.smooth.moved > 0);

    // Smoothing off: the kept points are the raw ones.
    let off = CleanConfig {
        smooth: SmoothConfig {
            enabled: false,
            ..SmoothConfig::default()
        },
        ..cfg
    };
    let (kept, _, _, stats) = clean(cloud(xyz), &rays, &off).unwrap();
    assert_eq!(kept.xyz, raw);
    assert_eq!(stats.smooth.moved, 0);
}

#[test]
fn clean_rejects_bad_smooth_config() {
    let rays = RaySource {
        frame_start: vec![0],
        ..RaySource::default()
    };
    for smooth in [
        SmoothConfig {
            radius: 0.0,
            ..SmoothConfig::default()
        },
        SmoothConfig {
            min_neighbours: 2,
            ..SmoothConfig::default()
        },
        SmoothConfig {
            max_neighbours: 5,
            ..SmoothConfig::default()
        },
        SmoothConfig {
            planarity: f64::NAN,
            ..SmoothConfig::default()
        },
        SmoothConfig {
            max_shift: -0.01,
            ..SmoothConfig::default()
        },
    ] {
        let cfg = CleanConfig {
            smooth,
            ..CleanConfig::default()
        };
        assert!(clean(Cloud::default(), &rays, &cfg).is_err(), "{cfg:?}");
    }
}
