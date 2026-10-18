//! The per-sample SLAM pipeline: odometry → scan preprocessing → [`OnlineSlam`].
//!
//! Pure processing, no I/O and no shared state: the daemon's SLAM thread feeds it
//! live data and publishes what it returns; offline tools (replay, debugging,
//! visualisation) feed it a recording and inspect every [`ScanStep`].

use crate::core::types::{LaserScan, PointCloud2D, Pose2D, PoseTracker};
use crate::engine::slam::{OnlineSlam, OnlineSlamConfig, SlamEngine, SlamResult};
use crate::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};
use crate::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor};

/// Scans with fewer points after preprocessing are skipped (their odometry carries
/// over to the next scan).
const MIN_SCAN_POINTS: usize = 50;

/// Configuration of the whole pipeline.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// SLAM engine configuration.
    pub slam_config: OnlineSlamConfig,
    /// Odometry algorithm type.
    pub odometry_type: OdometryType,
    /// Odometry configuration.
    pub odometry_config: DynOdometryConfig,
    /// Scan preprocessor configuration.
    pub preprocessor_config: PreprocessorConfig,
}

/// Everything one lidar scan went through.
#[derive(Debug, Clone)]
pub struct ScanStep {
    /// Data time of the scan.
    pub timestamp_us: u64,
    /// The scan as received (lidar frame, polar).
    pub laser: LaserScan,
    /// The preprocessed scan the engine saw (robot frame).
    pub cloud: PointCloud2D,
    /// Cumulative odometry pose at this scan.
    pub odom_pose: Pose2D,
    /// Odometry motion since the previous processed scan.
    pub odom_delta: Pose2D,
    /// The engine's result; `None` when the engine is not asked to process scans.
    pub result: Option<SlamResult>,
}

/// Odometry + preprocessing + SLAM engine, driven one sensor sample at a time.
pub struct SlamPipeline {
    slam: OnlineSlam,
    odometry: DynOdometry,
    preprocessor: ScanPreprocessor,
    /// Cumulative odometry pose.
    odom_tracker: PoseTracker,
    /// Odometry pose at the last processed scan.
    last_odom_pose: Pose2D,
}

impl SlamPipeline {
    /// Build the pipeline.
    pub fn new(config: &PipelineConfig) -> Self {
        log::info!(
            "Initializing odometry with algorithm: {:?}",
            config.odometry_type
        );
        Self {
            slam: OnlineSlam::new(config.slam_config.clone()),
            odometry: DynOdometry::new(config.odometry_type, config.odometry_config.clone()),
            preprocessor: ScanPreprocessor::new(config.preprocessor_config.clone()),
            odom_tracker: PoseTracker::new(),
            last_odom_pose: Pose2D::identity(),
        }
    }

    /// The SLAM engine (pose, map, keyframes, pose graph).
    pub fn slam(&self) -> &OnlineSlam {
        &self.slam
    }

    /// The SLAM engine, for commands (mode, reset, maps).
    pub fn slam_mut(&mut self) -> &mut OnlineSlam {
        &mut self.slam
    }

    /// The scan preprocessor (lidar mounting offsets).
    pub fn preprocessor(&self) -> &ScanPreprocessor {
        &self.preprocessor
    }

    /// Feed one wheel encoder + gyro sample. Returns the new cumulative odometry
    /// pose, or `None` while the odometry is still initialising.
    pub fn on_odometry(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
    ) -> Option<Pose2D> {
        let odom_pose = self.odometry.update(left, right, gyro_yaw, timestamp_us)?;
        // Every DynOdometry variant returns the CUMULATIVE odometry pose (the
        // wrappers compose encoder deltas internally), so set(), not update().
        // Composing it again (oangelo b81eee3) turns any motion into a spiral:
        // replay of a mock 3-lap rectangle (18 m) drifted to 200 m. The zero
        // odom_delta that commit chased was the dropped sensor channel (9753ae7).
        self.odom_tracker.set(odom_pose);
        Some(odom_pose)
    }

    /// Odometry motion since the last processed scan.
    pub fn odom_since_scan(&self) -> Pose2D {
        self.last_odom_pose
            .inverse()
            .compose(&self.odom_tracker.pose())
    }

    /// Best current pose: the engine's pose at the last scan plus the odometry since.
    ///
    /// The odometry since the scan is the cumulative tracker relative to its value at
    /// that scan. Composing a single odometry sample (local) onto the SLAM pose
    /// (global) instead held the published pose back near the scan (close_loop saw
    /// -24° while the scan matcher already had -133°).
    pub fn interpolated_pose(&self) -> Pose2D {
        self.slam.current_pose().compose(&self.odom_since_scan())
    }

    /// Feed one lidar scan. `process` = hand it to the engine (mapping or
    /// localising); otherwise only the odometry reference moves on. Returns `None`
    /// for a scan too sparse to use.
    pub fn on_scan(
        &mut self,
        laser: LaserScan,
        timestamp_us: u64,
        process: bool,
    ) -> Option<ScanStep> {
        let cloud = self.preprocessor.process(&laser);
        if cloud.len() < MIN_SCAN_POINTS {
            log::debug!("Skipping sparse scan: {} points", cloud.len());
            return None;
        }
        let odom_pose = self.odom_tracker.pose();
        let odom_delta = self.last_odom_pose.inverse().compose(&odom_pose);
        let result = process.then(|| self.slam.process_scan(&cloud, &odom_delta, timestamp_us));
        self.last_odom_pose = odom_pose;
        self.odom_tracker.take_snapshot();
        Some(ScanStep {
            timestamp_us,
            laser,
            cloud,
            odom_pose,
            odom_delta,
            result,
        })
    }
}
