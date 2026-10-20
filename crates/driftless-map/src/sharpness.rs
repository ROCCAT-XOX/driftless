//! Image sharpness: variance of the Laplacian of the gray image.
//!
//! Blur (focus or motion) removes the high frequencies the Laplacian responds to, so the
//! variance drops. The value depends on resolution and scene content, so it is only compared
//! between images of the same scan at the same resolution (see
//! [`crate::ColorConfig::min_sharpness`]).

use driftless_io::images::RgbImage;
use rayon::prelude::*;

/// Sum and sum of squares of the 4-neighbour Laplacian over the interior pixels of row `y`.
fn row_moments(gray: &[u8], w: usize, y: usize) -> (f64, f64) {
    let at = |x: usize, y: usize| i32::from(gray[y * w + x]);
    (1..w - 1).fold((0.0, 0.0), |(s, s2), x| {
        let lap = at(x - 1, y) + at(x + 1, y) + at(x, y - 1) + at(x, y + 1) - 4 * at(x, y);
        let l = f64::from(lap);
        (s + l, s2 + l * l)
    })
}

/// Variance of the 4-neighbour Laplacian of the gray image over its interior pixels; 0 for
/// images smaller than 3 × 3. Rows are summed in parallel and reduced in row order, so the
/// result is the same in every run.
pub fn sharpness(img: &RgbImage) -> f64 {
    let (w, h) = (img.width as usize, img.height as usize);
    if w < 3 || h < 3 {
        return 0.0;
    }
    let gray = img.to_gray();
    let rows: Vec<(f64, f64)> = (1..h - 1)
        .into_par_iter()
        .map(|y| row_moments(&gray, w, y))
        .collect();
    let (s, s2) = rows
        .iter()
        .fold((0.0, 0.0), |(a, b), (s, s2)| (a + s, b + s2));
    let n = ((w - 2) * (h - 2)) as f64;
    let mean = s / n;
    (s2 / n - mean * mean).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(w: u32, h: u32, f: impl Fn(u32, u32) -> u8) -> RgbImage {
        let data = (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .flat_map(|(x, y)| [f(x, y); 3])
            .collect();
        RgbImage {
            width: w,
            height: h,
            data,
        }
    }

    #[test]
    fn flat_and_linear_images_have_no_sharpness() {
        assert_eq!(sharpness(&image(16, 12, |_, _| 90)), 0.0);
        // The Laplacian of a linear ramp is 0.
        assert_eq!(sharpness(&image(16, 12, |x, y| (3 * x + 2 * y) as u8)), 0.0);
        assert_eq!(sharpness(&image(2, 12, |x, _| (x * 200) as u8)), 0.0);
    }

    #[test]
    fn checkerboard_is_sharper_than_smooth_stripes() {
        let checker = sharpness(&image(
            32,
            32,
            |x, y| if (x + y) % 2 == 0 { 0 } else { 200 },
        ));
        let smooth = sharpness(&image(32, 32, |x, _| {
            (100.0 + 80.0 * (f64::from(x) * 0.3).sin()) as u8
        }));
        assert!(checker > 100.0 * smooth, "{checker} vs {smooth}");
        // Single isolated pixel: Laplacian −4·200 there, +200 at its 4 neighbours.
        let dot = sharpness(&image(5, 5, |x, y| if (x, y) == (2, 2) { 200 } else { 0 }));
        let (s, s2) = (-800.0 + 4.0 * 200.0, 640_000.0 + 4.0 * 40_000.0);
        let want: f64 = s2 / 9.0 - (s / 9.0f64).powi(2);
        assert!((dot - want).abs() < 1e-9, "{dot} vs {want}");
    }
}
