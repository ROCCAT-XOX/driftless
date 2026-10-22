//! Triangle reduction: flat walls become a few big triangles, the shape stays within a tight error.

use super::{Mesh, SurfaceConfig};
use meshopt::{SimplifyOptions, VertexDataAdapter};

/// Reduces the triangles of `mesh` with meshoptimizer.
///
/// No vertex moves further than `cfg.simplify_error` (m) from the surface, and the vertices on
/// the border of the mesh stay where they are, so open edges keep their length. Vertices no face
/// uses after the reduction are dropped; the order of the rest is kept.
pub fn simplify(mesh: &Mesh, cfg: &SurfaceConfig) -> Mesh {
    if mesh.indices.is_empty() {
        return Mesh::default();
    }
    let flat: Vec<u32> = mesh.indices.iter().flatten().copied().collect();
    let bytes: &[u8] = positions_as_bytes(&mesh.positions);
    let adapter = VertexDataAdapter::new(bytes, 12, 0).expect("Eckpunkte sind drei f32 je Punkt");
    // Target count 0: reduce as far as the error allows. The error is absolute (m), which is
    // the same as the relative error of the brief scaled by the extent of the mesh.
    let reduced = meshopt::simplify(
        &flat,
        &adapter,
        0,
        cfg.simplify_error as f32,
        SimplifyOptions::LockBorder | SimplifyOptions::ErrorAbsolute,
        None,
    );
    let mut out = Mesh {
        positions: mesh.positions.clone(),
        indices: reduced.as_chunks::<3>().0.to_vec(),
    };
    drop_unused(&mut out);
    out
}

fn positions_as_bytes(p: &[[f32; 3]]) -> &[u8] {
    // SAFETY: `[f32; 3]` has no padding and every bit pattern is a valid `u8`.
    unsafe { std::slice::from_raw_parts(p.as_ptr().cast::<u8>(), std::mem::size_of_val(p)) }
}

/// Drops vertices no face uses, keeping the order of the rest.
fn drop_unused(mesh: &mut Mesh) {
    let mut new_index = vec![u32::MAX; mesh.positions.len()];
    for f in &mesh.indices {
        for &i in f {
            new_index[i as usize] = 0;
        }
    }
    let mut next = 0u32;
    let mut positions = Vec::new();
    for (i, slot) in new_index.iter_mut().enumerate() {
        if *slot == 0 {
            *slot = next;
            next += 1;
            positions.push(mesh.positions[i]);
        }
    }
    for f in &mut mesh.indices {
        *f = f.map(|i| new_index[i as usize]);
    }
    mesh.positions = positions;
}

#[cfg(test)]
mod tests {
    use super::super::testutil::*;
    use super::*;

    fn cfg() -> SurfaceConfig {
        SurfaceConfig::default()
    }

    /// A flat wall `x = 0.5` of `nx × ny` squares of `step` (m), normals `+x`.
    fn grid_wall(nx: usize, ny: usize, step: f32) -> Mesh {
        let mut mesh = Mesh::default();
        for iy in 0..=nx {
            for iz in 0..=ny {
                mesh.positions
                    .push([0.5, iy as f32 * step, iz as f32 * step]);
            }
        }
        let id = |iy: usize, iz: usize| (iy * (ny + 1) + iz) as u32;
        for iy in 0..nx {
            for iz in 0..ny {
                mesh.indices
                    .push([id(iy, iz), id(iy + 1, iz), id(iy, iz + 1)]);
                mesh.indices
                    .push([id(iy + 1, iz), id(iy + 1, iz + 1), id(iy, iz + 1)]);
            }
        }
        mesh
    }

    /// Distance from `p` to the triangle `abc` (Ericson, Real-Time Collision Detection).
    fn point_triangle(p: [f64; 3], tri: [[f64; 3]; 3]) -> f64 {
        let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let [a, b, c] = tri;
        let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
        let (d1, d2) = (dot(ab, ap), dot(ac, ap));
        let closest = if d1 <= 0.0 && d2 <= 0.0 {
            a
        } else {
            let bp = sub(p, b);
            let (d3, d4) = (dot(ab, bp), dot(ac, bp));
            let cp = sub(p, c);
            let (d5, d6) = (dot(ab, cp), dot(ac, cp));
            let (vc, vb, va) = (d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4);
            let at = |u: f64, v: f64| {
                [
                    a[0] + ab[0] * u + ac[0] * v,
                    a[1] + ab[1] * u + ac[1] * v,
                    a[2] + ab[2] * u + ac[2] * v,
                ]
            };
            if d3 >= 0.0 && d4 <= d3 {
                b
            } else if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                at(d1 / (d1 - d3), 0.0)
            } else if d6 >= 0.0 && d5 <= d6 {
                c
            } else if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                at(0.0, d2 / (d2 - d6))
            } else if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                [
                    b[0] + (c[0] - b[0]) * w,
                    b[1] + (c[1] - b[1]) * w,
                    b[2] + (c[2] - b[2]) * w,
                ]
            } else {
                let denom = 1.0 / (va + vb + vc);
                at(vb * denom, vc * denom)
            }
        };
        let d = sub(p, closest);
        dot(d, d).sqrt()
    }

    /// Largest distance from a vertex of `from` to the surface of `to` (m).
    fn one_sided(from: &Mesh, to: &Mesh) -> f64 {
        let up = |p: [f32; 3]| p.map(f64::from);
        let tris: Vec<[[f64; 3]; 3]> = to
            .indices
            .iter()
            .map(|f| f.map(|i| up(to.positions[i as usize])))
            .collect();
        // Only vertices a face uses: fixtures may leave orphans behind.
        let used: std::collections::BTreeSet<u32> =
            from.indices.iter().flatten().copied().collect();
        used.iter()
            .map(|&i| {
                let p = from.positions[i as usize];
                tris.iter()
                    .map(|&t| point_triangle(up(p), t))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn flat_wall_gets_few_triangles() {
        let mesh = grid_wall(300, 250, 0.01); // 3 × 2.5 m
        let out = simplify(&mesh, &cfg());
        assert!(
            out.indices.len() * 100 < mesh.indices.len(),
            "{} von {} Dreiecken übrig",
            out.indices.len(),
            mesh.indices.len()
        );
        assert!(out.positions.iter().all(|p| (p[0] - 0.5).abs() <= 0.001));
        assert!(out.positions.len() < mesh.positions.len());
        // Every vertex is used.
        let mut used = vec![false; out.positions.len()];
        out.indices
            .iter()
            .flatten()
            .for_each(|&i| used[i as usize] = true);
        assert!(used.iter().all(|&u| u));
    }

    #[test]
    fn simplify_keeps_rod() {
        let (mesh, _) = rod_on_wall(40, 0.01, 30);
        let out = simplify(&mesh, &cfg());
        assert!(out.indices.len() < mesh.indices.len());
        let hausdorff = one_sided(&out, &mesh).max(one_sided(&mesh, &out));
        assert!(hausdorff <= 0.002, "Hausdorff-Abstand {hausdorff} m");
        // The tip of the rod is still there.
        let tip = out.positions.iter().map(|p| p[0]).fold(0.0, f32::max);
        assert!(tip >= 0.3 - 0.002);
    }

    #[test]
    fn border_unchanged() {
        let mesh = wall_with_hole(150, 0.01, Some((25, 100)));
        let before = edge_report(&mesh);
        assert_eq!(before, (0, 400));
        let out = simplify(&mesh, &cfg());
        assert_eq!(edge_report(&out), before);
        assert!(out.indices.len() < mesh.indices.len());
    }

    #[test]
    fn deterministic_and_empty() {
        let mesh = grid_wall(40, 30, 0.01);
        let (a, b) = (simplify(&mesh, &cfg()), simplify(&mesh, &cfg()));
        assert_eq!((a.positions, a.indices), (b.positions, b.indices));
        assert!(simplify(&Mesh::default(), &cfg()).indices.is_empty());
    }
}
