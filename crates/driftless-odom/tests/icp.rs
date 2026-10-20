mod common;

use common::room_scene;
use driftless_odom::icp::{align, IcpParams, IcpTarget};
use driftless_odom::voxel_map::{voxel_downsample, VoxelMap};
use driftless_traj::nalgebra::{Isometry3, Point3, Unit, UnitQuaternion, Vector3};

#[test]
fn icp_recovers_known_transform() {
    let scene = room_scene();
    let target = IcpTarget::new(scene.surface_samples(0.1, 0.0)).expect("non-empty target");

    // Truth: 0.2 m and 5° about a mostly vertical axis.
    let truth = Isometry3::from_parts(
        Vector3::new(0.12, -0.1, 0.12).into(),
        UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(Vector3::new(0.2, 0.1, 1.0)),
            5f64.to_radians(),
        ),
    );
    assert!((truth.translation.vector.norm() - 0.2) < 0.01);
    // Source: a different sampling of the same surfaces, expressed in the moved frame.
    let world = voxel_downsample(&scene.surface_samples(0.1, 0.05), 0.3);
    let inv = truth.inverse();
    let source: Vec<Vector3<f64>> = world
        .iter()
        .map(|p| (inv * Point3::from(*p)).coords)
        .collect();

    let r = align(
        &source,
        &target,
        &Isometry3::identity(),
        &IcpParams::default(),
    );
    let err = r.pose.inverse() * truth;
    let dt = err.translation.vector.norm();
    let da = err.rotation.angle().to_degrees();
    assert!(
        r.converged,
        "not converged after {} iterations",
        r.iterations
    );
    assert!(dt < 0.01, "translation error {dt} m");
    assert!(da < 0.1, "rotation error {da}°");
}

/// Registers the room problem with everything moved by `shift` (world points, truth, init).
fn shifted_room_result(shift: Vector3<f64>) -> (Isometry3<f64>, Isometry3<f64>) {
    let scene = room_scene();
    let to_shift = Isometry3::translation(shift.x, shift.y, shift.z);
    let target_pts = scene
        .surface_samples(0.1, 0.0)
        .into_iter()
        .map(|p| p + shift)
        .collect();
    let target = IcpTarget::new(target_pts).unwrap();
    let truth = to_shift
        * Isometry3::from_parts(
            Vector3::new(0.12, -0.1, 0.12).into(),
            UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(Vector3::new(0.2, 0.1, 1.0)),
                5f64.to_radians(),
            ),
        );
    let inv = truth.inverse();
    let source: Vec<Vector3<f64>> = voxel_downsample(&scene.surface_samples(0.1, 0.05), 0.3)
        .iter()
        .map(|p| (inv * Point3::from(*p + shift)).coords)
        .collect();
    let r = align(&source, &target, &to_shift, &IcpParams::default());
    (r.pose, truth)
}

#[test]
fn icp_result_does_not_depend_on_distance_from_origin() {
    let (near, truth_near) = shifted_room_result(Vector3::zeros());
    let shift = Vector3::new(400.0, -300.0, 20.0); // ≈ 500 m
    let (far, truth_far) = shifted_room_result(shift);
    for (pose, truth) in [(near, truth_near), (far, truth_far)] {
        let err = pose.inverse() * truth;
        assert!(
            err.translation.vector.norm() < 0.01,
            "{}",
            err.translation.vector.norm()
        );
        assert!(
            err.rotation.angle().to_degrees() < 0.1,
            "{}",
            err.rotation.angle().to_degrees()
        );
    }
    // Same answer relative to the shift.
    let far_local = Isometry3::translation(-shift.x, -shift.y, -shift.z) * far;
    let d = near.inverse() * far_local;
    assert!(
        d.translation.vector.norm() < 1e-4,
        "{}",
        d.translation.vector.norm()
    );
    assert!(d.rotation.angle() < 1e-5, "{}", d.rotation.angle());
}

#[test]
fn voxel_downsample_keeps_one_point_per_voxel_in_input_order() {
    let pts = vec![
        Vector3::new(0.05, 0.05, 0.05),
        Vector3::new(0.9, 0.0, 0.0),
        Vector3::new(0.06, 0.01, 0.02), // same voxel as the first
        Vector3::new(-0.05, 0.0, 0.0),  // negative side: own voxel
    ];
    let out = voxel_downsample(&pts, 0.5);
    assert_eq!(out, vec![pts[0], pts[1], pts[3]]);
}

#[test]
fn voxel_map_caps_points_skips_close_ones_and_forgets_far_voxels() {
    let mut map = VoxelMap::new(1.0, 2, 0.1);
    map.insert(&[
        Vector3::new(0.1, 0.1, 0.1),
        Vector3::new(0.1, 0.1, 0.15), // too close to the first
        Vector3::new(0.2, 0.2, 0.2),
        Vector3::new(0.3, 0.3, 0.3), // voxel full
        Vector3::new(50.5, 0.5, 0.5),
    ]);
    assert_eq!(map.len(), 3);
    map.remove_far(&Vector3::zeros(), 10.0);
    assert_eq!(
        map.points(),
        vec![Vector3::new(0.1, 0.1, 0.1), Vector3::new(0.2, 0.2, 0.2)]
    );
}
