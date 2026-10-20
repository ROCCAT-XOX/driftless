use driftless_traj::{
    align_umeyama, ate_rmse, match_count, path_length, read_tum, start_end_gap, write_tum, Pose,
    TrajError,
};
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

const DT: f64 = 0.01;

/// Deterministic standard normal numbers (LCG + Box-Muller), no extra dependency.
struct Rng(u64);
impl Rng {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

fn pose(t: f64, p: Vector3<f64>) -> Pose {
    Pose {
        t,
        p,
        q: UnitQuaternion::identity(),
    }
}

/// A 3D curve that is not planar or symmetric, sampled at `rate` Hz.
fn curve(n: usize, rate: f64) -> Vec<Pose> {
    (0..n)
        .map(|i| {
            let t = i as f64 / rate;
            pose(
                t,
                Vector3::new(
                    3.0 * t.cos(),
                    2.0 * (0.7 * t).sin(),
                    0.3 * t + 0.2 * (2.0 * t).sin(),
                ),
            )
        })
        .collect()
}

fn tmp(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("driftless-traj-{}-{name}", std::process::id()))
}

#[test]
fn identity_ate_zero() {
    let a = curve(200, 10.0);
    assert!(ate_rmse(&a, &a, DT).unwrap() < 1e-12);
}

#[test]
fn rigid_offset_removed_by_alignment() {
    let gt = curve(300, 10.0);
    let iso = Isometry3::new(Vector3::new(5.0, -2.0, 1.0), Vector3::new(0.3, -0.5, 1.2));
    let est: Vec<Pose> = gt
        .iter()
        .map(|g| Pose {
            t: g.t,
            p: iso.transform_point(&nalgebra::Point3::from(g.p)).coords,
            q: iso.rotation * g.q,
        })
        .collect();
    assert!(ate_rmse(&est, &gt, DT).unwrap() < 1e-9);
    let t = align_umeyama(&est, &gt, DT).unwrap();
    let back = t.transform_point(&nalgebra::Point3::from(est[7].p));
    assert!((back.coords - gt[7].p).norm() < 1e-9);
}

#[test]
fn noise_ate_matches() {
    let gt = curve(3000, 10.0);
    let mut rng = Rng(42);
    let s = 0.01 / 3f64.sqrt(); // per axis, so the 3D RMS error is 0.01 m
    let est: Vec<Pose> = gt
        .iter()
        .map(|g| {
            pose(
                g.t,
                g.p + Vector3::new(rng.normal(), rng.normal(), rng.normal()) * s,
            )
        })
        .collect();
    let ate = ate_rmse(&est, &gt, DT).unwrap();
    assert!((ate - 0.01).abs() < 0.002, "ate = {ate}");
}

#[test]
fn different_rates_are_matched_by_time() {
    let hi = curve(2000, 200.0); // 10 s at 200 Hz
    let lo: Vec<Pose> = hi.iter().step_by(20).cloned().collect(); // 10 Hz
    let iso = Isometry3::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.2, 0.0));
    let lo_moved: Vec<Pose> = lo
        .iter()
        .map(|g| {
            pose(
                g.t,
                iso.transform_point(&nalgebra::Point3::from(g.p)).coords,
            )
        })
        .collect();
    let matched_hi_vs_lo = ate_rmse(&lo_moved, &hi, DT).unwrap();
    assert!(matched_hi_vs_lo < 1e-9, "{matched_hi_vs_lo}");
    // Other way round (dense est, sparse gt): only poses within max_dt of a gt
    // sample match, and those differ by at most speed * max_dt (about 3 m/s * 0.01 s).
    let dense = ate_rmse(&hi, &lo, DT).unwrap();
    assert!(dense < 0.03, "{dense}");
}

#[test]
fn unsorted_input_is_sorted() {
    let gt = curve(100, 10.0);
    let mut est = gt.clone();
    est.reverse();
    assert!(ate_rmse(&est, &gt, DT).unwrap() < 1e-12);
}

#[test]
fn too_few_matches_is_error() {
    let a = curve(2, 10.0);
    assert!(matches!(
        ate_rmse(&a, &a, DT),
        Err(TrajError::TooFewMatches { found: 2, .. })
    ));
    let far: Vec<Pose> = curve(50, 10.0)
        .into_iter()
        .map(|mut p| {
            p.t += 100.0;
            p
        })
        .collect();
    assert!(matches!(
        align_umeyama(&far, &curve(50, 10.0), DT),
        Err(TrajError::TooFewMatches { found: 0, .. })
    ));
}

#[test]
fn mirrored_trajectory_gives_proper_rotation() {
    // Est is a mirror image of gt: no rigid motion fits, but the result
    // must still be a proper rotation (det +1), never a reflection.
    let gt = curve(200, 10.0);
    let est: Vec<Pose> = gt
        .iter()
        .map(|g| pose(g.t, Vector3::new(-g.p.x, g.p.y, g.p.z)))
        .collect();
    let iso = align_umeyama(&est, &gt, DT).unwrap();
    let r = iso.rotation.to_rotation_matrix();
    assert!((r.matrix().determinant() - 1.0).abs() < 1e-12);
    assert!(ate_rmse(&est, &gt, DT).unwrap() > 0.1);
}

fn square(side: f64, closing: f64) -> Vec<Pose> {
    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(side, 0.0, 0.0),
        Vector3::new(side, side, 0.0),
        Vector3::new(0.0, side, 0.0),
        Vector3::new(0.0, closing, 0.0),
    ];
    corners
        .iter()
        .enumerate()
        .map(|(i, c)| pose(i as f64, *c))
        .collect()
}

#[test]
fn start_end_gap_square_loop() {
    let closed = square(10.0, 0.0);
    assert!(start_end_gap(&closed) < 1e-12);
    assert!((path_length(&closed) - 40.0).abs() < 1e-12);
    let open = square(10.0, 0.5);
    assert!((start_end_gap(&open) - 0.5).abs() < 1e-12);
    assert_eq!(start_end_gap(&[]), 0.0);
    assert_eq!(path_length(&[]), 0.0);
}

#[test]
fn tum_roundtrip() {
    let poses: Vec<Pose> = (0..50)
        .map(|i| Pose {
            t: 1_792_430_344.725_116 + i as f64 * 0.005,
            p: Vector3::new(i as f64 * 0.1, -0.5, 1.234_567_891),
            q: UnitQuaternion::from_euler_angles(0.1 * i as f64, 0.2, -0.3),
        })
        .collect();
    let path = tmp("roundtrip.tum");
    write_tum(&path, &poses).unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# timestamp tx ty tz qx qy qz qw\n"));
    let back = read_tum(&path).unwrap();
    assert_eq!(back.len(), poses.len());
    for (a, b) in poses.iter().zip(&back) {
        assert!((a.t - b.t).abs() < 1e-6);
        assert!((a.p - b.p).norm() < 1e-8);
        assert!(a.q.angle_to(&b.q) < 1e-8);
    }
    let _ = std::fs::remove_file(path);
}

fn parse(name: &str, text: &str) -> Result<Vec<Pose>, TrajError> {
    let path = tmp(name);
    std::fs::write(&path, text).unwrap();
    let r = read_tum(&path);
    let _ = std::fs::remove_file(path);
    r
}

#[test]
fn tum_comments_blank_and_quaternion_order() {
    let poses = parse("order.tum", "# c\n\n  \n1.0 1 2 3 0 0 0.6 0.8\n").unwrap();
    assert_eq!(poses.len(), 1);
    // qw = 0.8 (last), qz = 0.6.
    assert!((poses[0].q.w - 0.8).abs() < 1e-12);
    assert!((poses[0].q.k - 0.6).abs() < 1e-12);
}

#[test]
fn tum_normalizes_quaternion() {
    let poses = parse("norm.tum", "0 0 0 0 0 0 0 2\n").unwrap();
    assert!((poses[0].q.w - 1.0).abs() < 1e-12);
}

#[test]
fn tum_malformed_lines_name_the_line() {
    for (text, line) in [
        ("0 0 0 0 0 0 0 1\n1 2 3\n", 2),
        ("# h\n0 0 0 0 0 0 0 1\n\nabc 0 0 0 0 0 0 1\n", 4),
        ("0 0 0 0 0 0 0 1 9\n", 1),
        ("0 0 0 0 0 0 0 NaN\n", 1),
        ("0 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 0\n", 2), // zero quaternion
    ] {
        match parse("bad.tum", text) {
            Err(e) => {
                let msg = e.to_string();
                assert!(msg.contains(&format!("Zeile {line}")), "{msg}");
            }
            Ok(_) => panic!("accepted: {text:?}"),
        }
    }
}

#[test]
#[ignore = "needs DRIFTLESS_TESTDATA"]
fn real_trj_compares_with_itself_zero() {
    let Ok(dir) = std::env::var("DRIFTLESS_TESTDATA") else {
        eprintln!("SKIPPED: DRIFTLESS_TESTDATA not set");
        return;
    };
    let path = std::path::Path::new(&dir).join("Test/shading/trj.txt");
    let poses = read_tum(&path).unwrap();
    // 53,175 lines in the file, one of them the header comment.
    assert_eq!(poses.len(), 53_174);
    assert!(ate_rmse(&poses, &poses, DT).unwrap() < 1e-9);
    assert!(path_length(&poses) > 0.0);
}

#[test]
fn match_count_agrees_with_alignment_rule() {
    let est: Vec<Pose> = (0..10)
        .map(|i| pose(i as f64, Vector3::new(i as f64, 0.0, 0.0)))
        .collect();
    // gt covers t = 0..5 only, plus one pose 0.004 s off t = 7.
    let mut gt: Vec<Pose> = (0..6)
        .map(|i| pose(i as f64, Vector3::new(i as f64, 0.0, 0.0)))
        .collect();
    gt.push(pose(7.004, Vector3::zeros()));
    assert_eq!(match_count(&est, &gt, 0.003), 6);
    assert_eq!(match_count(&est, &gt, DT), 7);
    assert_eq!(match_count(&est, &[], DT), 0);
    // Unsorted input gives the same count.
    let mut shuffled = est.clone();
    shuffled.reverse();
    assert_eq!(match_count(&shuffled, &gt, DT), 7);
}
