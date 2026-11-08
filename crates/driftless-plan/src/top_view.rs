//! The picture of a plan from above: per cell the colour of the highest point of the wall
//! band, so that the ceiling does not hide the room. Without colours the height gives a grey.

use crate::grid::{robust_extent, Band};
use crate::walls::band_top;
use crate::{StoreyPoints, WallFace};

/// Longest side of the picture.
const MAX_PIXELS: usize = 4096;
/// The band starts this far below the floor (metres).
const BELOW_FLOOR: f64 = 0.10;
/// Empty border around the measured area (metres).
const MARGIN: f64 = 0.10;
/// Grey of the floor and of the top of the band.
const GREY_LOW: f64 = 70.0;
const GREY_HIGH: f64 = 230.0;

/// The picture, in the frame of the plan. Row 0 is the largest y.
#[derive(Debug, Clone, PartialEq)]
pub struct TopView {
    pub width: u32,
    pub height: u32,
    /// The corner of the picture with the smallest x and y.
    pub origin: [f64; 2],
    /// Edge of a pixel (metres).
    pub cell: f64,
    /// Four bytes per pixel; a pixel without a point is transparent.
    pub rgba: Vec<u8>,
}

/// Draws the picture. The points and the faces are in the frame of the plan.
pub(crate) fn top_view(points: &StoreyPoints, faces: &[WallFace], cell: f64) -> TopView {
    let top = band_top(points.height);
    let band = Band {
        low: -BELOW_FLOOR,
        high: top,
    };
    let (mut min, mut max) =
        robust_extent(&points.xyz, band).unwrap_or(([f64::MAX; 2], [f64::MIN; 2]));
    for p in faces.iter().flat_map(|f| [f.a, f.b]) {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    }
    if !(min[0] <= max[0] && min[1] <= max[1]) {
        return TopView {
            width: 1,
            height: 1,
            origin: [0.0, 0.0],
            cell,
            rgba: vec![0; 4],
        };
    }
    let origin = [min[0] - MARGIN, min[1] - MARGIN];
    let size = [max[0] + MARGIN - origin[0], max[1] + MARGIN - origin[1]];
    let cell = cell.max(size[0].max(size[1]) / MAX_PIXELS as f64);
    let nx = ((size[0] / cell).ceil() as usize).clamp(1, MAX_PIXELS);
    let ny = ((size[1] / cell).ceil() as usize).clamp(1, MAX_PIXELS);

    // LAS colours are 16 bits, but some writers keep 0..255 in them.
    let shift = match &points.rgb {
        Some(rgb) if rgb.iter().flatten().any(|&c| c > 255) => 8,
        _ => 0,
    };
    let mut highest = vec![f32::NEG_INFINITY; nx * ny];
    let mut rgba = vec![0u8; nx * ny * 4];
    for (index, p) in points.xyz.iter().enumerate() {
        if !band.contains(p[2] as f64) {
            continue;
        }
        let (x, y) = (
            (p[0] as f64 - origin[0]) / cell,
            (p[1] as f64 - origin[1]) / cell,
        );
        if !(x >= 0.0 && y >= 0.0) {
            continue;
        }
        let (i, j) = (x as usize, y as usize);
        if i >= nx || j >= ny {
            continue;
        }
        let pixel = (ny - 1 - j) * nx + i;
        if p[2] <= highest[pixel] {
            continue;
        }
        highest[pixel] = p[2];
        let colour = match &points.rgb {
            Some(rgb) => rgb[index].map(|c| (c >> shift) as u8),
            None => {
                let share = (p[2] as f64 / top).clamp(0.0, 1.0);
                [(GREY_LOW + (GREY_HIGH - GREY_LOW) * share) as u8; 3]
            }
        };
        rgba[pixel * 4..pixel * 4 + 4].copy_from_slice(&[colour[0], colour[1], colour[2], 255]);
    }
    TopView {
        width: nx as u32,
        height: ny as u32,
        origin,
        cell,
        rgba,
    }
}
