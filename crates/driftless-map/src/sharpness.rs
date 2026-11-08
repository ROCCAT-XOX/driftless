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

/// Sums of the sharpness measure over some pixels of an image ([`Moments::of`]); the sums of
/// several images (the charts of a Stelle) add up to one measure ([`Moments::sharpness`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Moments {
    /// Sum, sum of squares and number of the 4-neighbour Laplacians.
    lap: f64,
    lap2: f64,
    lap_n: f64,
    /// Sum and number of the gray values.
    gray: f64,
    gray_n: f64,
}

impl Moments {
    /// The sums over the pixels of `img` that `inside` (row-major, one per pixel) marks: the
    /// 4-neighbour Laplacian of the gray image at every marked pixel whose four neighbours are
    /// marked too (so no edge of the marked area counts), the gray value at every marked pixel.
    /// Rows are summed in parallel and added in row order, so the sums are the same in every run.
    ///
    /// # Panics
    /// If `inside` does not have one entry per pixel.
    pub(crate) fn of(img: &RgbImage, inside: &[bool]) -> Moments {
        let (w, h) = (img.width as usize, img.height as usize);
        assert_eq!(inside.len(), w * h, "mask does not fit the image");
        let gray = img.to_gray();
        let rows: Vec<Moments> = (0..h)
            .into_par_iter()
            .map(|y| {
                let mut m = Moments::default();
                for x in 0..w {
                    let i = y * w + x;
                    if !inside[i] {
                        continue;
                    }
                    m.gray += f64::from(gray[i]);
                    m.gray_n += 1.0;
                    let interior = x > 0 && y > 0 && x + 1 < w && y + 1 < h;
                    if !(interior
                        && inside[i - 1]
                        && inside[i + 1]
                        && inside[i - w]
                        && inside[i + w])
                    {
                        continue;
                    }
                    let at = |j: usize| i32::from(gray[j]);
                    let lap = at(i - 1) + at(i + 1) + at(i - w) + at(i + w) - 4 * at(i);
                    let l = f64::from(lap);
                    m.lap += l;
                    m.lap2 += l * l;
                    m.lap_n += 1.0;
                }
                m
            })
            .collect();
        let mut sum = Moments::default();
        for r in &rows {
            sum.add(r);
        }
        sum
    }

    /// Adds the sums of `other`.
    pub(crate) fn add(&mut self, other: &Moments) {
        self.lap += other.lap;
        self.lap2 += other.lap2;
        self.lap_n += other.lap_n;
        self.gray += other.gray;
        self.gray_n += other.gray_n;
    }

    /// The sharpness of the summed pixels as [`sharpness`] measures it: the variance of the
    /// Laplacians over `max(mean gray, SHARPNESS_MIN_MEAN)`; 0 without Laplacians.
    pub(crate) fn sharpness(&self) -> f64 {
        if self.lap_n == 0.0 {
            return 0.0;
        }
        let mean = self.lap / self.lap_n;
        let var = (self.lap2 / self.lap_n - mean * mean).max(0.0);
        let gray = if self.gray_n > 0.0 {
            self.gray / self.gray_n
        } else {
            0.0
        };
        var / gray.max(SHARPNESS_MIN_MEAN)
    }
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
    #[test]
    fn moments_measure_the_marked_pixels() {
        let img = image(64, 48, texture);
        // Everything marked: the measure of the whole image.
        let (s, _) = measure(&img);
        let all = Moments::of(&img, &vec![true; 64 * 48]);
        assert!(
            (all.sharpness() - s).abs() <= 1e-9 * s,
            "{} vs {s}",
            all.sharpness()
        );
        // A rectangle marked: the measure of the image cut to it, whatever lies around it.
        let (x0, y0, w, h) = (10usize, 7usize, 30usize, 20usize);
        let mut noisy = img.clone();
        let mut inside = vec![false; 64 * 48];
        for y in 0..48 {
            for x in 0..64 {
                let i = y * 64 + x;
                if (x0..x0 + w).contains(&x) && (y0..y0 + h).contains(&y) {
                    inside[i] = true;
                } else {
                    noisy.data[i * 3..i * 3 + 3]
                        .copy_from_slice(&[((x * 97 + y * 31) % 256) as u8; 3]);
                }
            }
        }
        let cut = image(w as u32, h as u32, |x, y| {
            texture(x + x0 as u32, y + y0 as u32)
        });
        let (want, _) = measure(&cut);
        let got = Moments::of(&noisy, &inside).sharpness();
        assert!((got - want).abs() <= 1e-9 * want, "{got} vs {want}");
        // Sums add up; nothing marked measures nothing.
        let mut twice = Moments::of(&noisy, &inside);
        twice.add(&Moments::of(&noisy, &inside));
        assert!((twice.sharpness() - want).abs() <= 1e-9 * want);
        assert_eq!(Moments::of(&img, &vec![false; 64 * 48]).sharpness(), 0.0);
        assert_eq!(Moments::default().sharpness(), 0.0);
    }
}
