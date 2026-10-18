//! Keyframe management for SLAM.
//!
//! Keyframes are pose-stamped scans that are stored for loop closure detection
//! and map optimization. Not every scan becomes a keyframe - only scans that
//! represent sufficient motion or rotation from the previous keyframe.

use crate::core::types::{PointCloud2D, Pose2D};

/// A keyframe containing a pose and associated scan data.
#[derive(Debug, Clone)]
pub struct Keyframe {
    /// Unique identifier for this keyframe.
    pub id: u64,

    /// Global pose of the robot when this keyframe was captured.
    pub pose: Pose2D,

    /// Point cloud in robot-local frame.
    pub scan: PointCloud2D,

    /// Timestamp when keyframe was captured (microseconds).
    pub timestamp_us: u64,

    /// Index of the submap this keyframe belongs to.
    pub submap_id: u64,
}

impl Keyframe {
    /// Create a new keyframe.
    pub fn new(
        id: u64,
        pose: Pose2D,
        scan: PointCloud2D,
        timestamp_us: u64,
        submap_id: u64,
    ) -> Self {
        Self {
            id,
            pose,
            scan,
            timestamp_us,
            submap_id,
        }
    }

    /// Append the scan, placed at the keyframe pose, to a world-frame cloud.
    pub fn push_world_points(&self, into: &mut PointCloud2D) {
        let (s, c) = self.pose.theta.sin_cos();
        for (x, y) in self.scan.xs.iter().zip(&self.scan.ys) {
            into.push_xy(self.pose.x + c * x - s * y, self.pose.y + s * x + c * y);
        }
    }
}

/// Configuration for keyframe selection.
#[derive(Debug, Clone)]
pub struct KeyframeManagerConfig {
    /// Minimum translation (meters) before creating a new keyframe.
    pub min_translation: f32,

    /// Minimum rotation (radians) before creating a new keyframe.
    pub min_rotation: f32,

    /// Maximum number of keyframes to keep in memory.
    ///
    /// Older keyframes may be removed to save memory, but their
    /// poses are kept in the pose graph.
    pub max_keyframes: usize,

    /// Minimum time between keyframes (microseconds).
    pub min_interval_us: u64,
}

impl Default for KeyframeManagerConfig {
    fn default() -> Self {
        Self {
            min_translation: 0.5, // 50cm
            min_rotation: 0.5,    // ~30 degrees
            max_keyframes: 1000,
            min_interval_us: 500_000, // 500ms minimum
        }
    }
}

/// Manages keyframe creation and storage.
pub struct KeyframeManager {
    config: KeyframeManagerConfig,

    /// All keyframes in chronological order.
    keyframes: Vec<Keyframe>,

    /// Pose of last keyframe (for motion threshold check).
    last_keyframe_pose: Option<Pose2D>,

    /// Timestamp of last keyframe.
    last_keyframe_time: u64,

    /// Next keyframe ID.
    next_id: u64,
}

impl KeyframeManager {
    /// Create a new keyframe manager.
    pub fn new(config: KeyframeManagerConfig) -> Self {
        Self {
            config,
            keyframes: Vec::new(),
            last_keyframe_pose: None,
            last_keyframe_time: 0,
            next_id: 0,
        }
    }

    /// Check if a new keyframe should be created.
    ///
    /// Returns true if the motion since the last keyframe exceeds
    /// the configured thresholds.
    pub fn should_create_keyframe(&self, current_pose: &Pose2D, timestamp_us: u64) -> bool {
        // Always create first keyframe
        let last_pose = match &self.last_keyframe_pose {
            Some(pose) => pose,
            None => return true,
        };

        // Check time threshold
        if timestamp_us < self.last_keyframe_time + self.config.min_interval_us {
            return false;
        }

        // Check translation threshold
        let dx = current_pose.x - last_pose.x;
        let dy = current_pose.y - last_pose.y;
        let translation = (dx * dx + dy * dy).sqrt();

        if translation >= self.config.min_translation {
            return true;
        }

        // Check rotation threshold
        let rotation = crate::core::math::angle_diff(last_pose.theta, current_pose.theta).abs();
        if rotation >= self.config.min_rotation {
            return true;
        }

        false
    }

    /// Create a new keyframe.
    ///
    /// # Arguments
    ///
    /// * `pose` - Global pose at keyframe time
    /// * `scan` - Point cloud in robot frame
    /// * `timestamp_us` - Timestamp
    /// * `submap_id` - ID of the submap this keyframe belongs to
    ///
    /// # Returns
    ///
    /// Reference to the newly created keyframe.
    pub fn create_keyframe(
        &mut self,
        pose: Pose2D,
        scan: PointCloud2D,
        timestamp_us: u64,
        submap_id: u64,
    ) -> &Keyframe {
        let id = self.next_id;
        self.next_id += 1;

        let keyframe = Keyframe::new(id, pose, scan, timestamp_us, submap_id);
        self.keyframes.push(keyframe);

        self.last_keyframe_pose = Some(pose);
        self.last_keyframe_time = timestamp_us;

        // Trim if over limit
        if self.keyframes.len() > self.config.max_keyframes {
            // Remove oldest keyframe (but keep its pose in graph)
            self.keyframes.remove(0);
        }

        self.keyframes.last().unwrap()
    }

    /// Get all keyframes.
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Get mutable keyframe by ID.
    pub fn get_mut(&mut self, id: u64) -> Option<&mut Keyframe> {
        self.keyframes.iter_mut().find(|kf| kf.id == id)
    }

    /// Get number of keyframes.
    pub fn len(&self) -> usize {
        self.keyframes.len()
    }

    /// Clear all keyframes.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.last_keyframe_pose = None;
        self.last_keyframe_time = 0;
        self.next_id = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::types::Point2D;

    fn create_test_scan() -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        // Create a simple square pattern
        for i in 0..10 {
            let angle = i as f32 * std::f32::consts::TAU / 10.0;
            let x = 2.0 * angle.cos();
            let y = 2.0 * angle.sin();
            scan.push(Point2D::new(x, y));
        }
        scan
    }

    #[test]
    fn test_keyframe_creation() {
        let pose = Pose2D::new(1.0, 2.0, 0.5);
        let scan = create_test_scan();
        let keyframe = Keyframe::new(0, pose, scan, 1000, 0);

        assert_eq!(keyframe.id, 0);
        assert_eq!(keyframe.pose.x, 1.0);
        assert_eq!(keyframe.submap_id, 0);
    }

    #[test]
    fn test_keyframe_manager_first_keyframe() {
        let config = KeyframeManagerConfig::default();
        let manager = KeyframeManager::new(config);

        // Should always create first keyframe
        assert!(manager.should_create_keyframe(&Pose2D::identity(), 0));
    }

    #[test]
    fn test_keyframe_manager_motion_threshold() {
        let config = KeyframeManagerConfig {
            min_translation: 0.5,
            min_rotation: 0.5,
            min_interval_us: 0, // Disable time threshold for test
            ..Default::default()
        };
        let mut manager = KeyframeManager::new(config);

        // Create first keyframe
        let scan = create_test_scan();
        manager.create_keyframe(Pose2D::identity(), scan.clone(), 0, 0);

        // Small motion - should not create
        let small_motion = Pose2D::new(0.1, 0.0, 0.0);
        assert!(!manager.should_create_keyframe(&small_motion, 1000));

        // Large translation - should create
        let large_translation = Pose2D::new(1.0, 0.0, 0.0);
        assert!(manager.should_create_keyframe(&large_translation, 1000));

        // Large rotation - should create
        let large_rotation = Pose2D::new(0.0, 0.0, 1.0);
        assert!(manager.should_create_keyframe(&large_rotation, 1000));
    }
}
