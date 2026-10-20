//! What stands still in the scans, for strategies to steer by: a point with one of the scan
//! before near it, in the world frame. A person walking by moves centimetres between two
//! turns of the lidar (166 ms), so a strategy that follows the nearest edge does not follow
//! their legs. Someone standing still stands still, and is steered round like a chair.
//!
//! Known limits:
//! - The beams fall a degree apart, and farther apart along a wall seen at a grazing angle,
//!   so such a far wall may flicker.
//! - A jump of the SLAM pose (a loop closure) leaves one scan empty.
//! - A bump into someone walking is explained by nothing the strategy sees, so it is taken
//!   for glass. Keeping what moves next to the body would explain it, but the robot then
//!   steers round passing legs: on the bench, with people in every room, the cost tripled.

use crate::core::types::{Point2D, PointCloud2D, Pose2D, Timestamped};
use crate::state::PlacedScan;

/// A point stands still with a point of the scan before this near (m)…
const NEAR: f32 = 0.03;
/// …plus this much per metre of range: the beams, a degree apart, fall elsewhere on a wall
/// as the robot moves.
const NEAR_PER_M: f32 = 0.0175;

/// The last scan, and what of it stood still.
#[derive(Debug, Clone, Default)]
pub(super) struct Steady {
    /// The last scan's stamp; `None` before the first.
    stamp: Option<u64>,
    /// Its points in the world frame.
    before: PointCloud2D,
    /// What of it stood still, where SLAM placed it.
    scan: PlacedScan,
}

impl Steady {
    /// Takes in `scan` if it is new. The first one has nothing to stand still against.
    pub(super) fn update(&mut self, scan: &Timestamped<PlacedScan>) {
        if self.stamp == Some(scan.timestamp_us) {
            return;
        }
        let placed = &scan.data;
        let world = placed.cloud.transform(&placed.pose);
        let local = &placed.cloud;
        let mut cloud = PointCloud2D::with_capacity(local.len());
        for i in 0..local.len() {
            let (x, y) = (local.xs[i], local.ys[i]);
            let range = x.hypot(y);
            let near = NEAR + NEAR_PER_M * range;
            let (wx, wy) = (world.xs[i], world.ys[i]);
            let stood = (self.before.xs.iter().zip(&self.before.ys))
                .any(|(bx, by)| (bx - wx).powi(2) + (by - wy).powi(2) < near * near);
            if stood {
                match &local.intensities {
                    Some(quality) => cloud.push_with_intensity(Point2D::new(x, y), quality[i]),
                    None => cloud.push_xy(x, y),
                }
            }
        }
        self.stamp = Some(scan.timestamp_us);
        self.before = world;
        self.scan = PlacedScan {
            cloud,
            pose: placed.pose,
        };
    }

    /// What stood still of the last scan, from the robot at `pose`.
    pub(super) fn seen_from(&self, pose: &Pose2D) -> PointCloud2D {
        self.scan.seen_from(pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan(stamp: u64, pose: Pose2D, points: &[(f32, f32)]) -> Timestamped<PlacedScan> {
        let mut cloud = PointCloud2D::new();
        for &(x, y) in points {
            cloud.push_xy(x, y);
        }
        Timestamped::new(PlacedScan { cloud, pose }, stamp)
    }

    #[test]
    fn a_wall_stands_still_as_the_robot_moves_and_turns_and_a_passer_by_does_not() {
        let mut steady = Steady::default();
        // A wall 0.4 m to the right, in the world frame, and a leg.
        let wall: Vec<(f32, f32)> = (0..50).map(|i| (i as f32 * 0.02, -0.4)).collect();
        let mut first = wall.clone();
        first.push((0.5, 0.5));
        steady.update(&scan(1, Pose2D::identity(), &first));
        assert!(
            steady.scan.cloud.is_empty(),
            "nothing to compare the first with"
        );

        // 3 cm on and turned 0.2 rad left; the leg moved 15 cm.
        let moved = Pose2D::new(0.03, 0.0, 0.2);
        let to_robot = |&(x, y): &(f32, f32)| {
            let p = moved.inverse().compose(&Pose2D::new(x, y, 0.0));
            (p.x, p.y)
        };
        let mut second: Vec<(f32, f32)> = wall.iter().map(to_robot).collect();
        second.push(to_robot(&(0.65, 0.5)));
        let next = scan(2, moved, &second);
        steady.update(&next);
        assert_eq!(steady.scan.cloud.len(), wall.len(), "the wall, not the leg");
        steady.update(&next);
        assert_eq!(
            steady.scan.cloud.len(),
            wall.len(),
            "the same scan again changes nothing"
        );
    }
}
