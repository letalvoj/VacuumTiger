//! Online SLAM module (Phase 7).
//!
//! Provides simultaneous localization and mapping capability by combining
//! odometry, scan matching, and mapping into a unified system.
//!
//! # Architecture
//!
//! Keyframe SLAM:
//!
//! ```text
//! Scan → Preprocess → Match (local map of recent keyframes, or active submap) → Pose
//!                                     │
//!                             Keyframe decision
//!                                     │
//!                   Keyframe → pose graph (when loop closure is on)
//!                                     │
//!          Global map = keyframe scans ray-traced at their (optimized) poses,
//!          rebuilt lazily after the graph moves them
//! ```
//!
//! Submaps are only used for matching when `use_submap_matching` is set.
//!
//! # Components
//!
//! - [`Keyframe`]: A pose-stamped scan used for loop closure
//! - [`KeyframeManager`]: Decides when to create keyframes
//! - [`Submap`]: A local occupancy grid with associated poses
//! - [`SubmapManager`]: Manages submap lifecycle
//! - [`OnlineSlam`]: Main SLAM engine combining all components
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::slam::{OnlineSlam, OnlineSlamConfig};
//! use dhruva_slam::types::{Pose2D, PointCloud2D};
//!
//! let config = OnlineSlamConfig::default();
//! let mut slam = OnlineSlam::new(config);
//!
//! // In sensor loop
//! let result = slam.process_scan(&scan, &odom_delta, timestamp_us);
//! println!("Current pose: {:?}", result.pose);
//!
//! // Get the map
//! let map = slam.global_map();
//! ```
//!
//! Note: Some types and utility methods are defined for future use.

pub mod keyframe;
mod online_slam;
pub mod submap;

// Core SLAM types - used by OnlineSlam and exported for SlamEngine trait
pub use keyframe::KeyframeManagerConfig;
pub use online_slam::{
    LoopClosureConfig, OnlineSlam, OnlineSlamConfig, SlamMode, SlamResult, SlamStatus,
};
pub use submap::SubmapManagerConfig;

use crate::core::types::{PointCloud2D, Pose2D};

/// Trait for SLAM engines.
///
/// Defines the interface for any SLAM implementation.
pub trait SlamEngine {
    /// Process a new laser scan with odometry.
    ///
    /// # Arguments
    ///
    /// * `scan` - Preprocessed point cloud in robot frame
    /// * `odom_delta` - Odometry-derived motion since last scan
    /// * `timestamp_us` - Scan timestamp in microseconds
    ///
    /// # Returns
    ///
    /// SLAM result containing corrected pose and status.
    fn process_scan(
        &mut self,
        scan: &PointCloud2D,
        odom_delta: &Pose2D,
        timestamp_us: u64,
    ) -> SlamResult;

    /// Get the current pose estimate.
    fn current_pose(&self) -> Pose2D;

    /// Get SLAM status.
    fn status(&self) -> SlamStatus;

    /// Reset SLAM to initial state.
    fn reset(&mut self);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slam_result_default() {
        let result = SlamResult::default();
        assert!(!result.keyframe_created);
        assert_eq!(result.match_score, 0.0);
    }
}
