// Ported from RoboSense's rs_driver (BSD-3-Clause), see LICENSE-THIRD-PARTY.
//! rs_driver's `Trigon`: sin/cos lookup tables over -90..450 degrees in 0.01 degree steps, as f32.

const ANGLE_MIN: i32 = -9000;
const ANGLE_MAX: i32 = 45000;

pub struct Trigon {
    sins: Vec<f32>,
    coss: Vec<f32>,
}

impl Default for Trigon {
    fn default() -> Self {
        Self::new()
    }
}

impl Trigon {
    pub fn new() -> Self {
        let (sins, coss) = (ANGLE_MIN..ANGLE_MAX)
            .map(|i| {
                let rad = (i as f64 * 0.01).to_radians();
                (rad.sin() as f32, rad.cos() as f32)
            })
            .unzip();
        Trigon { sins, coss }
    }

    // out-of-table angles read angle 0, as rs_driver does
    fn index(angle: i32) -> usize {
        let a = if (ANGLE_MIN..ANGLE_MAX).contains(&angle) { angle } else { 0 };
        (a - ANGLE_MIN) as usize
    }

    /// sin of an angle in 0.01 degree.
    pub fn sin(&self, angle: i32) -> f32 {
        self.sins[Self::index(angle)]
    }

    /// cos of an angle in 0.01 degree.
    pub fn cos(&self, angle: i32) -> f32 {
        self.coss[Self::index(angle)]
    }
}
