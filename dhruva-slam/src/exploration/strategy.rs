//! Exploration strategy trait for extensible exploration algorithms.
//!
//! This module defines the `ExplorationStrategy` trait that all exploration
//! algorithms must implement. This allows easy swapping between different
//! exploration strategies.
//!
//! Note: Some types are defined for planned exploration features.

use crate::core::types::{PointCloud2D, Pose2D};
use crate::state::CurrentMapData;

/// Type of hazard detected by contact sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HazardType {
    /// Bumper collision - solid obstacle in front.
    BumperLeft,
    BumperRight,
    BumperBoth,
    /// Cliff sensor - drop-off hazard.
    CliffLeftSide,
    CliffLeftFront,
    CliffRightFront,
    CliffRightSide,
}

/// The bumper zones (rad from the heading): each reaches round its side this far and past
/// the front by `BUMPER_OVERLAP`, where both press (as the mock's `mock_bench.toml` has
/// them; the v8's are not measured).
const BUMPER_REACH: f32 = 1.7;
const BUMPER_OVERLAP: f32 = 0.3;

impl HazardType {
    /// Where the hazard was sensed: (bearing from the heading, rad, + left; distance from
    /// the centre, m). A bumper pressed alone was touched round its side, not at the front
    /// where both press: the middle of the shell only it reaches.
    pub fn sensor_offset(&self) -> (f32, f32) {
        let alone = (BUMPER_OVERLAP + BUMPER_REACH) / 2.0;
        match self {
            HazardType::BumperLeft => (alone, 0.17),
            HazardType::BumperRight => (-alone, 0.17),
            HazardType::BumperBoth => (0.0, 0.17),
            HazardType::CliffLeftSide => (1.57, 0.15), // 90° left (side)
            HazardType::CliffLeftFront => (0.52, 0.16), // ~30° left front
            HazardType::CliffRightFront => (-0.52, 0.16), // ~30° right front
            HazardType::CliffRightSide => (-1.57, 0.15), // 90° right (side)
        }
    }

    /// The bearings (rad, from the heading, + left) of the shell a bumper's press may come
    /// from (`BUMPER_REACH`, `BUMPER_OVERLAP`). `None` for cliffs.
    pub fn bumper_arc(&self) -> Option<(f32, f32)> {
        match self {
            HazardType::BumperLeft => Some((-BUMPER_OVERLAP, BUMPER_REACH)),
            HazardType::BumperRight => Some((-BUMPER_REACH, BUMPER_OVERLAP)),
            HazardType::BumperBoth => Some((-BUMPER_REACH, BUMPER_REACH)),
            _ => None,
        }
    }

    /// Whether this is a cliff hazard (vs bumper).
    pub fn is_cliff(&self) -> bool {
        matches!(
            self,
            HazardType::CliffLeftSide
                | HazardType::CliffLeftFront
                | HazardType::CliffRightFront
                | HazardType::CliffRightSide
        )
    }
}

/// A hazard event detected by contact sensors.
#[derive(Debug, Clone)]
pub struct HazardEvent {
    /// Type of hazard detected.
    pub hazard_type: HazardType,
    /// World position where hazard was detected.
    pub world_position: Pose2D,
}

impl HazardEvent {
    /// Create a new hazard event, computing world position from robot pose and sensor offset.
    pub fn new(hazard_type: HazardType, robot_pose: &Pose2D) -> Self {
        let (angle_offset, distance) = hazard_type.sensor_offset();
        let hazard_angle = robot_pose.theta + angle_offset;
        let world_position = Pose2D::new(
            robot_pose.x + distance * hazard_angle.cos(),
            robot_pose.y + distance * hazard_angle.sin(),
            hazard_angle,
        );
        Self {
            hazard_type,
            world_position,
        }
    }
}

/// Result of exploration planning.
#[derive(Debug, Clone)]
pub enum ExplorationAction {
    /// Move toward target pose (direct line motion, for hazard recovery).
    MoveTo {
        /// Target pose to navigate to.
        target: Pose2D,
        /// Human-readable reason for this target.
        reason: &'static str,
    },
    /// Navigate to target using A* path planning (for frontier exploration).
    NavigateTo {
        /// Target pose to navigate to via planned path.
        target: Pose2D,
        /// Human-readable reason for this target.
        reason: &'static str,
    },
    /// Drive with this velocity until the next call (reactive behaviours).
    Drive {
        /// Linear velocity (m/s, negative = backward).
        linear: f32,
        /// Angular velocity (rad/s, positive = CCW).
        angular: f32,
        /// Human-readable reason for this motion.
        reason: &'static str,
    },
    /// Rotate in place to scan surroundings.
    RotateInPlace {
        /// Angle to rotate (radians, positive = CCW).
        angle_rad: f32,
    },
    /// Exploration complete - all reachable areas mapped.
    Complete,
    /// Cannot proceed - stuck or unreachable.
    Stuck {
        /// Reason for being stuck.
        reason: String,
    },
}

/// Trait for exploration strategies.
///
/// Implement this trait to create custom exploration algorithms.
/// The exploration thread will call `next_action` periodically to
/// determine what the robot should do next.
pub trait ExplorationStrategy: Send + Sync {
    /// Name of this strategy for logging.
    fn name(&self) -> &'static str;

    /// Compute next action given current state.
    ///
    /// # Arguments
    /// * `map` - Current occupancy grid map data
    /// * `current_pose` - Current robot pose in map frame
    /// * `scan` - Latest preprocessed scan (robot frame)
    /// * `hazard` - Optional hazard event if bumper/cliff triggered since last call
    /// * `held` - The bumper or cliff contact held now, if any (the most urgent)
    ///
    /// # Returns
    /// The next action the robot should take.
    fn next_action(
        &mut self,
        map: &CurrentMapData,
        current_pose: &Pose2D,
        scan: &PointCloud2D,
        hazard: Option<&HazardEvent>,
        held: Option<HazardType>,
    ) -> ExplorationAction;

    /// Reset strategy state (e.g., when starting new map).
    fn reset(&mut self);

    /// Get the number of frontiers currently being tracked.
    fn frontier_count(&self) -> usize {
        0
    }

    /// Get estimated completion percentage (0-100).
    fn completion_percent(&self) -> f32 {
        0.0
    }

    /// Mark a target location as blocked (navigation failed to reach it).
    /// This allows the strategy to avoid selecting this location again.
    fn mark_target_blocked(&mut self, _x: f32, _y: f32) {
        // Default implementation does nothing
    }

    /// Mark current target as successfully reached.
    /// Called when navigation completes successfully to clear internal target state.
    fn mark_target_reached(&mut self) {
        // Default implementation does nothing
    }
}
