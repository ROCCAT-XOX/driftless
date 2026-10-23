//! A software preview of a [`TexturedMesh`]: for result images and the visual acceptance,
//! without a GPU.
//!
//! A pinhole camera at `eye` looks at `target`, Z up (looking straight up or down, Y is up
//! in the image), square pixels with the vertical field of view `fov_deg`. Triangles are
//! clipped at the near plane ([`NEAR`]), so a floor running behind the camera still shows,
//! and rasterised with pixel centres, a depth buffer (nearest surface wins; on a tie the
//! triangle drawn first, in page and index order) and perspective-correct texture
//! coordinates; the colour is the bilinear sRGB texel colour (clamped at the page edges).
//! Both sides of every face are drawn: the walls of a room show from outside too. What no
//! triangle covers is [`BACKGROUND`].
//!
//! Deterministic: the image is split into bands of rows, each band draws every triangle in
//! the same order.

use super::TexturedMesh;
use driftless_io::images::RgbImage;
use rayon::prelude::*;

/// Near clipping distance (m).
pub const NEAR: f32 = 0.01;
/// Colour where no surface is.
pub const BACKGROUND: [u8; 3] = [40, 40, 40];
/// Rows per band of parallel work.
const BAND_ROWS: usize = 16;

/// A clipped triangle on the screen: per corner `(x, y)` in pixels, `1/z`, `u/z`, `v/z`.
#[derive(Clone, Copy)]
struct ScreenTri {
    page: u32,
    corners: [[f32; 5]; 3],
    /// Rows `[y0, y1)` it may cover.
    rows: [usize; 2],
}

/// Renders `tex` as seen from `eye` towards `target` (see the module docs).
pub fn render_preview(
    tex: &TexturedMesh,
    eye: [f32; 3],
    target: [f32; 3],
    fov_deg: f32,
    size: [u32; 2],
) -> RgbImage {
    let (w, h) = (size[0] as usize, size[1] as usize);
    let mut data = BACKGROUND.repeat(w * h);
    let Some(cam) = Cam::new(eye, target, fov_deg, size) else {
        return RgbImage {
            width: size[0],
            height: size[1],
            data,
        };
    };
    let tris: Vec<ScreenTri> = tex
        .pages
        .iter()
        .enumerate()
        .flat_map(|(p, page)| {
            let cam = &cam;
            page.indices.iter().flat_map(move |tri| {
                let corners = tri.map(|i| {
                    let i = i as usize;
                    let c = cam.to_cam(page.positions[i]);
                    [c[0], c[1], c[2], page.uvs[i][0], page.uvs[i][1]]
                });
                clip_near(corners)
                    .into_iter()
                    .flatten()
                    .filter_map(move |t| cam.to_screen(p as u32, t, h))
            })
        })
        .collect();

    let mut depth = vec![0f32; w * h];
    data.par_chunks_mut(w * 3 * BAND_ROWS)
        .zip(depth.par_chunks_mut(w * BAND_ROWS))
        .enumerate()
        .for_each(|(band, (rgb, depth))| {
            let y0 = band * BAND_ROWS;
            let y1 = y0 + depth.len() / w.max(1);
            for t in &tris {
                if t.rows[1] <= y0 || t.rows[0] >= y1 {
                    continue;
                }
                let page = &tex.pages[t.page as usize].image;
                raster(
                    t,
                    page,
                    w,
                    [y0.max(t.rows[0]), y1.min(t.rows[1])],
                    y0,
                    rgb,
                    depth,
                );
            }
        });
    RgbImage {
        width: size[0],
        height: size[1],
        data,
    }
}

/// Draws `t` into the rows `rows` of a band starting at row `y0`.
fn raster(
    t: &ScreenTri,
    page: &RgbImage,
    w: usize,
    rows: [usize; 2],
    y0: usize,
    rgb: &mut [u8],
    depth: &mut [f32],
) {
    let [a, b, c] = t.corners;
    let edge = |p: [f32; 5], q: [f32; 5], x: f32, y: f32| {
        (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0])
    };
    let area = edge(a, b, c[0], c[1]);
    if area == 0.0 || !area.is_finite() {
        return;
    }
    let xmin = a[0].min(b[0]).min(c[0]).floor().max(0.0) as usize;
    let xmax = (a[0].max(b[0]).max(c[0]).ceil().max(0.0) as usize).min(w);
    for y in rows[0]..rows[1] {
        let py = y as f32 + 0.5;
        for x in xmin..xmax {
            let px = x as f32 + 0.5;
            let l = [
                edge(b, c, px, py) / area,
                edge(c, a, px, py) / area,
                edge(a, b, px, py) / area,
            ];
            if l.iter().any(|&v| v < 0.0) {
                continue;
            }
            let at = |k: usize| l[0] * a[k] + l[1] * b[k] + l[2] * c[k];
            let inv_z = at(2);
            let i = (y - y0) * w + x;
            if inv_z <= depth[i] {
                continue;
            }
            depth[i] = inv_z;
            let (u, v) = (at(3) / inv_z, at(4) / inv_z);
            rgb[i * 3..i * 3 + 3].copy_from_slice(&bilinear(page, u, v));
        }
    }
}

/// The bilinear colour of `page` at `(u, v)` (0…1, `v` down), clamped at the edges.
fn bilinear(page: &RgbImage, u: f32, v: f32) -> [u8; 3] {
    let (w, h) = (page.width as usize, page.height as usize);
    if w == 0 || h == 0 {
        return BACKGROUND;
    }
    let fx = (u * w as f32 - 0.5).clamp(0.0, (w - 1) as f32);
    let fy = (v * h as f32 - 0.5).clamp(0.0, (h - 1) as f32);
    let (x0, y0) = (fx.floor() as usize, fy.floor() as usize);
    let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
    let (tx, ty) = (fx - x0 as f32, fy - y0 as f32);
    let px = |x: usize, y: usize, c: usize| f32::from(page.data[(y * w + x) * 3 + c]);
    [0, 1, 2].map(|c| {
        let top = px(x0, y0, c) * (1.0 - tx) + px(x1, y0, c) * tx;
        let bottom = px(x0, y1, c) * (1.0 - tx) + px(x1, y1, c) * tx;
        (top * (1.0 - ty) + bottom * ty).round() as u8
    })
}

/// Clips a triangle (camera coordinates and uv per corner) at `z = NEAR`: none, one or two
/// triangles.
fn clip_near(t: [[f32; 5]; 3]) -> [Option<[[f32; 5]; 3]>; 2] {
    let inside = t.map(|c| c[2] >= NEAR);
    let mut poly: Vec<[f32; 5]> = Vec::with_capacity(4);
    for k in 0..3 {
        let (p, q) = (t[k], t[(k + 1) % 3]);
        if inside[k] {
            poly.push(p);
        }
        if inside[k] != inside[(k + 1) % 3] {
            let s = (NEAR - p[2]) / (q[2] - p[2]);
            poly.push([0, 1, 2, 3, 4].map(|i| p[i] + (q[i] - p[i]) * s));
        }
    }
    match poly.len() {
        3 => [Some([poly[0], poly[1], poly[2]]), None],
        4 => [
            Some([poly[0], poly[1], poly[2]]),
            Some([poly[0], poly[2], poly[3]]),
        ],
        _ => [None, None],
    }
}

/// The preview camera.
struct Cam {
    eye: [f32; 3],
    right: [f32; 3],
    up: [f32; 3],
    forward: [f32; 3],
    focal: f32,
    centre: [f32; 2],
}

impl Cam {
    /// `None` if `eye` and `target` coincide or the field of view is not in (0°, 180°).
    fn new(eye: [f32; 3], target: [f32; 3], fov_deg: f32, size: [u32; 2]) -> Option<Cam> {
        if !(fov_deg > 0.0 && fov_deg < 180.0) {
            return None;
        }
        let forward = normalize(sub(target, eye))?;
        let right = normalize(cross(forward, [0.0, 0.0, 1.0]))
            .or_else(|| normalize(cross(forward, [0.0, 1.0, 0.0])))?;
        let up = cross(right, forward);
        Some(Cam {
            eye,
            right,
            up,
            forward,
            focal: size[1] as f32 / 2.0 / (fov_deg.to_radians() / 2.0).tan(),
            centre: [size[0] as f32 / 2.0, size[1] as f32 / 2.0],
        })
    }

    /// `(x right, y up, z forward)` of a world point.
    fn to_cam(&self, p: [f32; 3]) -> [f32; 3] {
        let d = sub(p, self.eye);
        [dot(d, self.right), dot(d, self.up), dot(d, self.forward)]
    }

    /// The triangle on the screen, `None` if it lies wholly beside the image.
    fn to_screen(&self, page: u32, t: [[f32; 5]; 3], h: usize) -> Option<ScreenTri> {
        let corners = t.map(|c| {
            let iz = 1.0 / c[2];
            [
                self.centre[0] + self.focal * c[0] * iz,
                self.centre[1] - self.focal * c[1] * iz,
                iz,
                c[3] * iz,
                c[4] * iz,
            ]
        });
        let ys = corners.map(|c| c[1]);
        let (ymin, ymax) = (ys[0].min(ys[1]).min(ys[2]), ys[0].max(ys[1]).max(ys[2]));
        let xs = corners.map(|c| c[0]);
        let (xmin, xmax) = (xs[0].min(xs[1]).min(xs[2]), xs[0].max(xs[1]).max(xs[2]));
        let wide = self.centre[0] * 2.0;
        if ymax < 0.0 || ymin > h as f32 || xmax < 0.0 || xmin > wide {
            return None;
        }
        Some(ScreenTri {
            page,
            corners,
            rows: [
                ymin.floor().max(0.0) as usize,
                (ymax.ceil().max(0.0) as usize).min(h),
            ],
        })
    }
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(a: [f32; 3]) -> Option<[f32; 3]> {
    let len = dot(a, a).sqrt();
    (len > 1e-9 && len.is_finite()).then(|| a.map(|x| x / len))
}

#[cfg(test)]
mod tests {
    use super::super::testutil::*;
    use super::*;

    fn pixel(img: &RgbImage, x: u32, y: u32) -> [u8; 3] {
        let i = ((y * img.width + x) * 3) as usize;
        [img.data[i], img.data[i + 1], img.data[i + 2]]
    }

    /// A wall `x = x0` of `2·half` square, one colour, facing −x.
    fn wall(x0: f32, half: f32, rgb: [u8; 3]) -> super::super::TexturePage {
        quad_page(
            [
                [x0, half, -half],
                [x0, -half, -half],
                [x0, -half, half],
                [x0, half, half],
            ],
            rgb,
            rgb,
            16,
        )
    }

    #[test]
    fn preview_shows_wall_color() {
        let tex = TexturedMesh {
            pages: vec![wall(3.0, 2.0, [230, 10, 10])],
            ..Default::default()
        };
        let img = render_preview(&tex, [0.0; 3], [1.0, 0.0, 0.0], 60.0, [64, 48]);
        assert_eq!((img.width, img.height), (64, 48));
        assert_eq!(pixel(&img, 32, 24), [230, 10, 10]);
        // The wall (±2 m at 3 m, ±33.7°) is not in the corner of a 120° view: background.
        let wide = render_preview(&tex, [0.0; 3], [1.0, 0.0, 0.0], 120.0, [64, 48]);
        let bg = pixel(&wide, 0, 0);
        assert_ne!(bg, [230, 10, 10]);
        assert!(
            bg[0] == bg[1] && bg[1] == bg[2] && bg[0] < 80,
            "dark gray {bg:?}"
        );
        assert_eq!(
            render_preview(&tex, [0.0; 3], [1.0, 0.0, 0.0], 60.0, [64, 48]),
            img
        );
    }

    #[test]
    fn preview_nearest_surface_wins() {
        let near = wall(2.0, 0.5, [10, 200, 10]);
        let far = wall(3.0, 2.0, [230, 10, 10]);
        for pages in [vec![near.clone(), far.clone()], vec![far, near]] {
            let tex = TexturedMesh {
                pages,
                ..Default::default()
            };
            let img = render_preview(&tex, [0.0; 3], [1.0, 0.0, 0.0], 60.0, [64, 48]);
            assert_eq!(pixel(&img, 32, 24), [10, 200, 10]);
            // Column 8: 29.5° off the axis, beside the near wall (14°), on the far one (33.7°).
            assert_eq!(pixel(&img, 8, 24), [230, 10, 10]);
        }
    }

    #[test]
    fn preview_floor_through_near_plane() {
        // Standing on a floor that runs behind the camera (it crosses the near plane): the
        // floor (red or green, blue 20) shows below the horizon, the ceiling above.
        let tex = floor_and_ceiling();
        let img = render_preview(&tex, [2.0, 0.2, 1.6], [2.0, 3.0, 1.6], 90.0, [64, 64]);
        let below = pixel(&img, 32, 60);
        assert_eq!(below[2], 20, "floor at the bottom {below:?}");
        let above = pixel(&img, 32, 3);
        assert!(above[2] > 200, "ceiling at the top {above:?}");
    }
}
