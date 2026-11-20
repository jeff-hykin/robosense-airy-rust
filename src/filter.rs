//! Optional downsampling applied to each scan after decoding.

use crate::decoder::Point;
use std::collections::HashSet;

/// Post-decode filters. The defaults change nothing.
#[derive(Debug, Clone, PartialEq)]
pub struct FilterConfig {
    /// Voxel edge in metres; each occupied voxel keeps its first point (with that point's ring, time and
    /// intensity). 0 = off.
    pub voxel: f32,
    /// Keep every Nth point of a scan, in decode order (1 = all).
    pub decimate_points: usize,
    /// Keep every Nth scan, dropping the others whole (1 = all).
    pub decimate_scans: usize,
}

impl Default for FilterConfig {
    fn default() -> Self {
        FilterConfig { voxel: 0.0, decimate_points: 1, decimate_scans: 1 }
    }
}

/// Keeps every `n`th point (n <= 1 keeps all).
pub fn decimate(points: &mut Vec<Point>, n: usize) {
    if n > 1 {
        let mut i: usize = 0;
        points.retain(|_| {
            let keep = i.is_multiple_of(n);
            i += 1;
            keep
        });
    }
}

/// Keeps the first point in each `edge`-metre voxel; NaN points are dropped. `edge <= 0` keeps all.
pub fn voxel_downsample(points: &mut Vec<Point>, edge: f32) {
    if edge <= 0.0 {
        return;
    }
    let inv = 1.0 / edge;
    let mut seen = HashSet::with_capacity(points.len() / 4);
    points.retain(|p| {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return false;
        }
        let key = ((p.x * inv).floor() as i32, (p.y * inv).floor() as i32, (p.z * inv).floor() as i32);
        seen.insert(key)
    });
}

/// Applies `voxel` then `decimate_points` (scan decimation is the caller's, since it drops whole scans).
pub fn apply(points: &mut Vec<Point>, config: &FilterConfig) {
    voxel_downsample(points, config.voxel);
    decimate(points, config.decimate_points);
}
