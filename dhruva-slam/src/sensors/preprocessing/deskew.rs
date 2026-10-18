//! Motion compensation ("deskew") of one lidar sweep.
//!
//! A spinning lidar takes ~0.17 s for a sweep. When the robot turns at 60°/s during
//! that time, the first and last beams are 10° apart in robot heading, so the raw
//! scan is a twisted copy of the room. Each beam is moved into the robot frame at the
//! END of the sweep (when the scan is stamped) using the robot's motion since that
//! beam was taken, as Cartographer and LIO-SAM do.
//!
//! The scan carries one timestamp, so a beam's time is inferred from its angle: the
//! sweep runs clockwise (the beam angle falls with time) at a constant rate, and the
//! lidar spins continuously, so a sweep lasts the time between two scans.

use super::LidarOffset;
use crate::core::types::{PointCloud2D, Pose2D};

/// When each beam of a sweep was taken.
#[derive(Debug, Clone, Copy)]
pub struct Sweep {
    /// Angle (rad) of a sweep's first beam, in the frame of the scan's angles as
    /// received (before [`LidarOffset::angle_offset`]).
    pub start_angle: f32,
}

impl Sweep {
    /// Fraction (0..1) of the sweep done when the beam towards `angle` was taken.
    fn fraction(&self, angle: f32) -> f32 {
        (self.start_angle - angle).rem_euclid(std::f32::consts::TAU) / std::f32::consts::TAU
    }

    /// Move every point (robot frame, as taken) to the robot frame at the sweep's end.
    ///
    /// `mounting` places the lidar in the robot frame (the beam angle is measured
    /// about it); `motion(age_s)` is the robot pose `age_s` seconds before the end of
    /// the sweep, relative to the pose at the end.
    pub fn deskew(
        &self,
        cloud: &mut PointCloud2D,
        mounting: &LidarOffset,
        period_s: f32,
        motion: impl Fn(f32) -> Pose2D,
    ) {
        let (ox, oy, oth) = (
            mounting.total_x(),
            mounting.total_y(),
            mounting.total_theta(),
        );
        for i in 0..cloud.len() {
            let (x, y) = (cloud.xs[i], cloud.ys[i]);
            let angle = (y - oy).atan2(x - ox) - oth;
            let age = (1.0 - self.fraction(angle)) * period_s;
            let m = motion(age);
            let (s, c) = m.theta.sin_cos();
            cloud.xs[i] = m.x + c * x - s * y;
            cloud.ys[i] = m.y + s * x + c * y;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn clockwise_fraction_grows_as_the_angle_falls() {
        let s = Sweep { start_angle: 0.0 };
        assert_relative_eq!(s.fraction(0.0), 0.0);
        assert_relative_eq!(s.fraction(-FRAC_PI_2), 0.25, epsilon = 1e-6);
        assert_relative_eq!(s.fraction(PI), 0.5, epsilon = 1e-6);
        assert_relative_eq!(s.fraction(FRAC_PI_2), 0.75, epsilon = 1e-6);
    }

    #[test]
    fn the_beam_angle_is_measured_in_the_received_frame() {
        // Lidar rotated by +90°: the beam it reports at 0 points along robot +y.
        let s = Sweep { start_angle: 0.0 };
        let mounting = LidarOffset::new(0.0, 0.0, 0.0, FRAC_PI_2);
        let mut cloud = PointCloud2D::new();
        cloud.push_xy(0.0, 2.0); // received angle 0: fraction 0, age = the whole period
        s.deskew(&mut cloud, &mounting, 1.0, |age| Pose2D::new(age, 0.0, 0.0));
        assert_relative_eq!(cloud.xs[0], 1.0, epsilon = 1e-5);
    }

    #[test]
    fn a_turn_during_the_sweep_is_undone() {
        // The robot turns counter-clockwise at 1 rad/s during a 1 s sweep, so a beam
        // taken `age` seconds before the end was taken while the robot faced `age` rad
        // to the right of its final heading: its point turns by -age.
        let s = Sweep { start_angle: 0.0 };
        let mut cloud = PointCloud2D::new();
        cloud.push_xy(2.0, 0.0); // fraction 0: taken 1 s before the end
        cloud.push_xy(-2.0, 0.0); // fraction 0.5: taken 0.5 s before the end
        s.deskew(&mut cloud, &LidarOffset::default(), 1.0, |age| {
            Pose2D::new(0.0, 0.0, -age)
        });
        assert_relative_eq!(cloud.xs[0], 2.0 * 1f32.cos(), epsilon = 1e-5);
        assert_relative_eq!(cloud.ys[0], -2.0 * 1f32.sin(), epsilon = 1e-5);
        assert_relative_eq!(cloud.xs[1], -2.0 * 0.5f32.cos(), epsilon = 1e-5);
        assert_relative_eq!(cloud.ys[1], 2.0 * 0.5f32.sin(), epsilon = 1e-5);
    }
}
