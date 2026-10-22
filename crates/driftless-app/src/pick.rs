//! Picking in the 3D view: the point of the cloud or the pose of the trajectory under the
//! mouse. All pixel values are physical pixels from the top left of the view; positions
//! are in the frame the view-projection matrix expects (for the cloud: relative to
//! [`Octree::origin`]).

use crate::camera::{valid_viewport, Camera};
use crate::lod::Octree;
use nalgebra::{Matrix4, Vector4};

/// The mouse position `pos_points` in egui points (window coordinates) as physical pixels
/// within the view whose top left corner is at `rect_min_points`.
pub fn to_physical(pos_points: [f32; 2], rect_min_points: [f32; 2], ppp: f32) -> [f32; 2] {
    [
        (pos_points[0] - rect_min_points[0]) * ppp,
        (pos_points[1] - rect_min_points[1]) * ppp,
    ]
}

/// An absolute position (e.g. a pose of the trajectory) in the origin-relative frame of
/// the cloud and the camera: the origin is subtracted in f64 before the cast to f32, so
/// that millimetres survive at UTM coordinates.
pub fn relative_to(origin: [f64; 3], p: [f64; 3]) -> [f32; 3] {
    [0, 1, 2].map(|k| (p[k] - origin[k]) as f32)
}

/// Pixel x, y and reverse-Z depth of `p`; `None` behind the camera. A depth above 1 lies
/// between the eye and the near plane (not drawn).
pub fn project(view_proj: &Matrix4<f32>, p: [f32; 3], viewport_px: [f32; 2]) -> Option<[f32; 3]> {
    let c = view_proj * Vector4::new(p[0], p[1], p[2], 1.0);
    if c.w <= 0.0 {
        return None;
    }
    let (x, y, z) = (c.x / c.w, c.y / c.w, c.z / c.w);
    Some([
        (x + 1.0) / 2.0 * viewport_px[0],
        (1.0 - y) / 2.0 * viewport_px[1],
        z,
    ])
}

/// On screen within `radius_px` and drawn (in front of the near plane): squared pixel
/// distance and depth.
fn hit(
    view_proj: &Matrix4<f32>,
    p: [f32; 3],
    viewport_px: [f32; 2],
    pixel_px: [f32; 2],
    radius_px: f32,
) -> Option<(f32, f32)> {
    let q = project(view_proj, p, viewport_px)?;
    let d2 = (q[0] - pixel_px[0]).powi(2) + (q[1] - pixel_px[1]).powi(2);
    (q[2] <= 1.0 && d2 <= radius_px * radius_px).then_some((d2, q[2]))
}

/// The vertex index of the front-most drawn point (largest depth) within `radius_px` of
/// the pixel; on a tie the smallest index. Drawn nodes whose bounding sphere misses the
/// cone around the ray through the pixel are skipped without looking at their points.
/// `None` for a degenerate viewport.
pub fn pick_point(
    tree: &Octree,
    drawn: &[u32],
    cam: &Camera,
    viewport_px: [f32; 2],
    pixel_px: [f32; 2],
    radius_px: f32,
) -> Option<u32> {
    pick_point_in(
        tree,
        drawn,
        cam,
        viewport_px,
        pixel_px,
        radius_px,
        [f32::MIN, f32::MAX],
    )
}

/// Like [`pick_point`], but only points whose z lies in `z_range` (inclusive), as the
/// height cut of the view draws them.
pub fn pick_point_in(
    tree: &Octree,
    drawn: &[u32],
    cam: &Camera,
    viewport_px: [f32; 2],
    pixel_px: [f32; 2],
    radius_px: f32,
    z_range: [f32; 2],
) -> Option<u32> {
    if !valid_viewport(viewport_px) {
        return None;
    }
    let view_proj = cam.view_proj(viewport_px[0] / viewport_px[1]);
    let ray = cam.ray(pixel_px, viewport_px);
    // Two rays through pixels `radius_px` apart differ by at most radius / focal length
    // (radians), the most at the centre of the image; a little slack for rounding.
    let focal_px = viewport_px[1] / (2.0 * (cam.fov_y / 2.0).tan());
    let half_angle = (radius_px / focal_px + 1e-4).min(1.5);
    let (sin_a, cos_a) = half_angle.sin_cos();
    let mut best: Option<(f32, u32)> = None;
    for &id in drawn {
        let node = &tree.nodes[id as usize];
        if node.min[2] > z_range[1] || node.max()[2] < z_range[0] {
            continue;
        }
        let v = nalgebra::Vector3::from(node.centre()) - ray.origin;
        let along = v.dot(&ray.dir);
        let perp = (v - ray.dir * along).norm();
        // Distance of the centre from the cone's surface (an underestimate behind the apex).
        let r = node.size * 0.5 * 3f32.sqrt();
        if perp * cos_a - along * sin_a > r * 1.001 {
            continue;
        }
        let range = node.first..node.first + node.count;
        for i in range {
            let p = tree.vertices[i as usize].pos;
            if p[2] < z_range[0] || p[2] > z_range[1] {
                continue;
            }
            let Some((_, z)) = hit(&view_proj, p, viewport_px, pixel_px, radius_px) else {
                continue;
            };
            let better = match best {
                None => true,
                Some((bz, bi)) => z > bz || (z == bz && i < bi),
            };
            if better {
                best = Some((z, i));
            }
        }
    }
    best.map(|(_, i)| i)
}

/// The index of the pose nearest to the pixel on screen within `radius_px`; on a tie the
/// front-most, then the smallest index. `positions` must be in the frame of `view_proj`,
/// i.e. relative to the cloud's origin ([`Octree::origin`]) like the camera: convert
/// absolute poses with [`relative_to`] (subtract in f64, then cast), never by casting the
/// absolute values. `None` for a degenerate viewport.
pub fn pick_pose(
    positions: &[[f32; 3]],
    view_proj: &Matrix4<f32>,
    viewport_px: [f32; 2],
    pixel_px: [f32; 2],
    radius_px: f32,
) -> Option<usize> {
    pick_poses(positions, view_proj, viewport_px, pixel_px, radius_px)
        .first()
        .copied()
}

/// Every pose within `radius_px` of the pixel, best first in the order of [`pick_pose`].
pub fn pick_poses(
    positions: &[[f32; 3]],
    view_proj: &Matrix4<f32>,
    viewport_px: [f32; 2],
    pixel_px: [f32; 2],
    radius_px: f32,
) -> Vec<usize> {
    if !valid_viewport(viewport_px) {
        return Vec::new();
    }
    let mut hits: Vec<(f32, f32, usize)> = positions
        .iter()
        .enumerate()
        .filter_map(|(i, &p)| {
            let (d2, z) = hit(view_proj, p, viewport_px, pixel_px, radius_px)?;
            Some((d2, z, i))
        })
        .collect();
    // nearer on screen, then in front (larger reverse-Z depth), then the smaller index
    hits.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then(b.1.total_cmp(&a.1))
            .then(a.2.cmp(&b.2))
    });
    hits.into_iter().map(|(_, _, i)| i).collect()
}
