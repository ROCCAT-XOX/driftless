//! Image sharpness: variance of the Laplacian of the gray image, divided by the mean gray
//! value.
//!
//! Blur (focus or motion) removes the high frequencies the Laplacian responds to, so the
//! variance drops. Dividing by the mean makes the value grow with the contrast in proportion to
//! the brightness instead of with its square. Image noise does not grow with the square of the
//! brightness, so dividing by the squared mean (as first tried in Meilenstein 4) made bright
//! images look "blurry". It still depends on resolution and scene content, so it is only
//! compared between images of the same scan at the same resolution (see
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

/// Mean gray values below this are treated as this value, so near-black images stay finite
/// and are not inflated by the division.
pub const SHARPNESS_MIN_MEAN: f64 = 16.0;

/// Variance of the 4-neighbour Laplacian of the gray image over its interior pixels divided by
/// `max(mean gray, SHARPNESS_MIN_MEAN)`; 0 for images smaller than 3 × 3. Rows are summed in
/// parallel and reduced in row order, so the result is the same in every run.
pub fn sharpness(img: &RgbImage) -> f64 {
    measure(img).0
}

/// (sharpness, mean gray value over the whole image); (0, 0) for images smaller than 3 × 3.
pub(crate) fn measure(img: &RgbImage) -> (f64, f64) {
    let (w, h) = (img.width as usize, img.height as usize);
    if w < 3 || h < 3 {
        return (0.0, 0.0);
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
    let var = (s2 / n - mean * mean).max(0.0);
    let total: u64 = gray.iter().map(|&g| u64::from(g)).sum();
    let gray_mean = total as f64 / gray.len() as f64;
    (var / gray_mean.max(SHARPNESS_MIN_MEAN), gray_mean)
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

    /// Deterministic textured test pattern with values in 40..=240.
    fn texture(x: u32, y: u32) -> u8 {
        (40 + (x * 37 + y * 91 + x * y * 13 + (x ^ y) * 7) % 201) as u8
    }

    fn scaled(img: &RgbImage, f: f64) -> RgbImage {
        RgbImage {
            width: img.width,
            height: img.height,
            data: img
                .data
                .iter()
                .map(|&v| (f64::from(v) * f).round() as u8)
                .collect(),
        }
    }

    /// Separable Gaussian blur with clamped borders.
    fn blurred(img: &RgbImage, sigma: f64) -> RgbImage {
        let r = (3.0 * sigma).ceil() as i64;
        let k: Vec<f64> = (-r..=r)
            .map(|i| (-((i * i) as f64) / (2.0 * sigma * sigma)).exp())
            .collect();
        let norm: f64 = k.iter().sum();
        let (w, h) = (img.width as i64, img.height as i64);
        let pass = |src: &[f64], horizontal: bool| -> Vec<f64> {
            (0..h * w)
                .map(|i| {
                    let (x, y) = (i % w, i / w);
                    k.iter()
                        .enumerate()
                        .map(|(j, kv)| {
                            let d = j as i64 - r;
                            let (xx, yy) = if horizontal {
                                ((x + d).clamp(0, w - 1), y)
                            } else {
                                (x, (y + d).clamp(0, h - 1))
                            };
                            kv * src[(yy * w + xx) as usize]
                        })
                        .sum::<f64>()
                        / norm
                })
                .collect()
        };
        let gray: Vec<f64> = img.to_gray().iter().map(|&g| f64::from(g)).collect();
        let out = pass(&pass(&gray, true), false);
        RgbImage {
            width: img.width,
            height: img.height,
            data: out.iter().flat_map(|&v| [v.round() as u8; 3]).collect(),
        }
    }

    #[test]
    fn brightness_scaling_is_proportional() {
        let base = image(64, 48, texture);
        let (a, b) = (sharpness(&base), sharpness(&scaled(&base, 0.3)));
        assert!(a > 0.0);
        // Unnormalised variance would give 0.09, the squared-mean normalisation 1.0.
        assert!((b / a - 0.3).abs() < 0.03, "{a} vs {b}");
    }

    #[test]
    fn blur_still_lowers_sharpness() {
        let base = image(64, 48, texture);
        let (a, b) = (sharpness(&base), sharpness(&blurred(&base, 2.0)));
        assert!(b < 0.3 * a, "{a} vs {b}");
    }

    #[test]
    fn dark_and_flat_images_are_finite() {
        for img in [
            image(32, 32, |_, _| 0),
            image(32, 32, |_, _| 3),
            image(32, 32, |x, y| ((x + y) % 2) as u8),
            image(32, 32, |_, _| 255),
        ] {
            let s = sharpness(&img);
            assert!(s.is_finite() && s >= 0.0, "{s}");
        }
        assert_eq!(sharpness(&image(32, 32, |_, _| 0)), 0.0);
        assert_eq!(measure(&image(4, 4, |_, _| 100)).1, 100.0);
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
        // Mean gray is 200 / 25 = 8, below the floor, so the floor is the divisor.
        let want: f64 = (s2 / 9.0 - (s / 9.0f64).powi(2)) / SHARPNESS_MIN_MEAN;
        assert!((dot - want).abs() < 1e-9, "{dot} vs {want}");
    }
}
