//! The point cloud as columns of equal length.

/// Point cloud in the world frame, one entry per point in every column.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Cloud {
    pub xyz: Vec<[f32; 3]>,
    pub intensity: Vec<f32>,
    /// Capture time (s): the end time of the point's frame.
    pub t: Vec<f64>,
    /// Index of the point's frame in the trajectory.
    pub frame: Vec<u32>,
    pub rgb: Option<Vec<[u8; 3]>>,
    /// Whether `rgb` holds a measured colour (a camera saw the point).
    pub colored: Option<Vec<bool>>,
}

/// Entries of `col` where `keep` is `want`, in order.
fn pick<T: Copy>(col: &[T], keep: &[bool], want: bool) -> Vec<T> {
    col.iter()
        .zip(keep)
        .filter(|(_, &k)| k == want)
        .map(|(v, _)| *v)
        .collect()
}

impl Cloud {
    pub fn len(&self) -> usize {
        self.xyz.len()
    }

    pub fn is_empty(&self) -> bool {
        self.xyz.is_empty()
    }

    /// Splits by `keep` (one flag per point) into (kept, removed), both in point order.
    ///
    /// # Panics
    /// If `keep` does not have one flag per point.
    pub fn select(&self, keep: &[bool]) -> (Cloud, Cloud) {
        assert_eq!(keep.len(), self.len(), "one flag per point");
        let part = |want: bool| Cloud {
            xyz: pick(&self.xyz, keep, want),
            intensity: pick(&self.intensity, keep, want),
            t: pick(&self.t, keep, want),
            frame: pick(&self.frame, keep, want),
            rgb: self.rgb.as_deref().map(|c| pick(c, keep, want)),
            colored: self.colored.as_deref().map(|c| pick(c, keep, want)),
        };
        (part(true), part(false))
    }
}
