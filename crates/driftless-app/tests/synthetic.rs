//! The synthetic cloud behind `--synthetic N`.

use driftless_app::synthetic::synthetic_cloud;

#[test]
fn synthetic_cloud_is_deterministic() {
    let a = synthetic_cloud(100_000, 7);
    let b = synthetic_cloud(100_000, 7);
    assert_eq!(a, b);
    assert_eq!(a.len(), 100_000);
    assert!(a.rgb.as_ref().is_some_and(|c| c.len() == a.len()));
    for column in [a.intensity.len(), a.t.len(), a.quality.len(), a.flags.len()] {
        assert_eq!(column, a.len());
    }
    // Another seed, other points.
    assert_ne!(synthetic_cloud(100_000, 8).xyz, a.xyz);
}
