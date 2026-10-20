//! Edge passes: strategies that run once round a room's edge, the wall kept on one side.
//!
//! - [`wall_follow`]: the corner sweep, ours: follows the nearest edge point, touches the wall
//!   to learn the lidar's offset, plans outer corners.
//!
//! Each is an [`ExplorationStrategy`](super::ExplorationStrategy) of its own; they share
//! the side, the odometer and the lap.

mod lap;
pub mod wall_follow;

pub use lap::Lap;

use serde::Deserialize;

use crate::core::math::angle_diff;
use crate::core::types::Pose2D;

/// Pose steps longer than this are relocalisation jumps, not travel (m).
const MAX_POSE_STEP: f32 = 0.1;

/// Which side the wall is kept on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WallSide {
    Left,
    #[default]
    Right,
}

impl WallSide {
    /// +1 for a wall on the right (the frame the strategies are written in), -1 for the
    /// left: the robot-frame y and the turn direction are multiplied by it.
    pub fn mirror(self) -> f32 {
        match self {
            WallSide::Right => 1.0,
            WallSide::Left => -1.0,
        }
    }
}

/// The distance driven and the turn of the last step, from the SLAM pose.
#[derive(Debug, Clone, Copy, Default)]
pub struct Travel {
    pub last: Option<Pose2D>,
    /// Driven in total (m).
    pub distance: f32,
}

impl Travel {
    /// Moves on to `pose`; the turn since the last pose (rad, + left). A step longer than
    /// `MAX_POSE_STEP` is SLAM correcting the pose, not the robot moving or turning: it
    /// counts for neither.
    pub fn update(&mut self, pose: &Pose2D) -> f32 {
        let mut turn = 0.0;
        if let Some(last) = self.last {
            let step = (pose.x - last.x).hypot(pose.y - last.y);
            if step < MAX_POSE_STEP {
                self.distance += step;
                turn = angle_diff(last.theta, pose.theta);
            }
        }
        self.last = Some(*pose);
        turn
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_relocalisation_jump_is_no_travel_or_turn() {
        let mut travel = Travel::default();
        assert_eq!(travel.update(&Pose2D::new(0.0, 0.0, 0.0)), 0.0);
        assert_eq!(travel.update(&Pose2D::new(0.05, 0.0, 0.1)), 0.1);
        assert_eq!(travel.update(&Pose2D::new(0.5, 0.0, 1.0)), 0.0, "a jump");
        assert_eq!(travel.update(&Pose2D::new(0.55, 0.0, 1.0)), 0.0);
        assert!((travel.distance - 0.1).abs() < 1e-6, "{}", travel.distance);
    }
}
