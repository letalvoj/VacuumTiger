//! The per-sample SLAM pipeline: odometry → scan preprocessing → [`OnlineSlam`].
//!
//! Pure processing, no I/O and no shared state: the daemon's SLAM thread feeds it
//! live data and publishes what it returns; offline tools (replay, debugging,
//! visualisation) feed it a recording and inspect every [`ScanStep`].

use std::collections::VecDeque;

use crate::core::math::angle_lerp;
use crate::core::types::{LaserScan, PointCloud2D, Pose2D, PoseTracker};
use crate::engine::slam::{OnlineSlam, OnlineSlamConfig, SlamEngine, SlamResult};
use crate::sensors::odometry::{DynOdometry, DynOdometryConfig, OdometryType};
use crate::sensors::preprocessing::{PreprocessorConfig, ScanPreprocessor, Sweep};

/// Scans with fewer points after preprocessing are skipped (their odometry carries
/// over to the next scan).
const MIN_SCAN_POINTS: usize = 50;

/// Odometry kept for deskewing: comfortably more than one sweep.
const ODOM_HISTORY_US: u64 = 1_000_000;

/// A sweep period (time between scans) outside this range is a gap in the data,
/// not a sweep: such a scan is not deskewed.
const PLAUSIBLE_SWEEP_PERIOD_US: std::ops::RangeInclusive<u64> = 50_000..=400_000;

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
    /// How the lidar sweeps, to deskew scans with the odometry; `None` takes scans
    /// as instantaneous.
    pub sweep: Option<Sweep>,
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
    sweep: Option<Sweep>,
    /// Recent cumulative odometry poses (time, pose), oldest first, for deskewing.
    odom_history: VecDeque<(u64, Pose2D)>,
    /// Time of the previous scan: the sweep period is the time between scans.
    last_scan_us: Option<u64>,
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
            sweep: config.sweep,
            odom_history: VecDeque::new(),
            last_scan_us: None,
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
        if self.sweep.is_some() {
            // Interpolation needs increasing times: a clock jumping back starts over.
            if self.odom_history.back().is_some_and(|&(t, _)| t > timestamp_us) {
                self.odom_history.clear();
            }
            self.odom_history.push_back((timestamp_us, odom_pose));
            while self
                .odom_history
                .front()
                .is_some_and(|&(t, _)| t + ODOM_HISTORY_US < timestamp_us)
            {
                self.odom_history.pop_front();
            }
        }
        Some(odom_pose)
    }

    /// Cumulative odometry pose at `t_us`, interpolated (clamped at the ends).
    fn odom_at(&self, t_us: u64) -> Option<Pose2D> {
        let after = self.odom_history.partition_point(|&(t, _)| t < t_us);
        let (t1, p1) = *self.odom_history.get(after).or(self.odom_history.back())?;
        let Some(&(t0, p0)) = after.checked_sub(1).and_then(|i| self.odom_history.get(i)) else {
            return Some(p1);
        };
        if t1 <= t0 {
            return Some(p1);
        }
        let f = (t_us - t0) as f32 / (t1 - t0) as f32;
        Some(Pose2D::new(
            p0.x + f * (p1.x - p0.x),
            p0.y + f * (p1.y - p0.y),
            angle_lerp(p0.theta, p1.theta, f),
        ))
    }

    /// Undo the robot's motion during the sweep that ended at `timestamp_us`.
    ///
    /// Points land in the robot frame at the LATEST odometry sample, the pose the
    /// engine is handed through `odom_delta`, not at the scan stamp, so the cloud
    /// and the motion prior agree. The first scan (no period yet) and scans after a
    /// gap are left as they are.
    fn deskew(&self, cloud: &mut PointCloud2D, timestamp_us: u64) {
        let (Some(sweep), Some(prev)) = (&self.sweep, self.last_scan_us) else {
            return;
        };
        let period_us = timestamp_us.saturating_sub(prev);
        if !PLAUSIBLE_SWEEP_PERIOD_US.contains(&period_us) || self.odom_history.is_empty() {
            return;
        }
        let to_latest = self.odom_tracker.pose().inverse();
        let mounting = self.preprocessor.lidar_offset();
        sweep.deskew(cloud, mounting, period_us as f32 * 1e-6, |age_s| {
            let t = timestamp_us.saturating_sub((age_s * 1e6) as u64);
            self.odom_at(t).map_or(Pose2D::identity(), |p| to_latest.compose(&p))
        });
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
        let mut cloud = self.preprocessor.process(&laser);
        self.deskew(&mut cloud, timestamp_us);
        self.last_scan_us = Some(timestamp_us);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deskew_puts_a_turning_sweep_in_the_final_robot_frame() {
        let mut pipeline = SlamPipeline::new(&PipelineConfig {
            slam_config: OnlineSlamConfig::default(),
            odometry_type: OdometryType::Wheel,
            odometry_config: DynOdometryConfig::default(),
            preprocessor_config: PreprocessorConfig::default(),
            sweep: Some(Sweep { start_angle: 0.0 }),
        });
        // Turning in place at 1 rad/s; the sweep runs from 0.8 s to the stamp at 1.0 s.
        for ms in (500..=1000).step_by(10) {
            let t = ms as f32 * 1e-3;
            pipeline.odom_history.push_back((ms * 1000, Pose2D::new(0.0, 0.0, t)));
        }
        pipeline.odom_tracker.set(Pose2D::new(0.0, 0.0, 1.0));
        pipeline.last_scan_us = Some(800_000);

        // The beam at sweep fraction f was taken at 0.8 + 0.2 f s, pointing at angle
        // -2 pi f in the robot frame of that time.
        let fractions = [0.1f32, 0.5, 0.9];
        let mut cloud = PointCloud2D::new();
        for f in fractions {
            let a = -std::f32::consts::TAU * f;
            cloud.push_xy(2.0 * a.cos(), 2.0 * a.sin());
        }
        pipeline.deskew(&mut cloud, 1_000_000);

        for (i, f) in fractions.into_iter().enumerate() {
            let heading_then = 0.8 + 0.2 * f;
            let a = -std::f32::consts::TAU * f + heading_then - 1.0;
            assert!((cloud.xs[i] - 2.0 * a.cos()).abs() < 1e-3, "beam {f}");
            assert!((cloud.ys[i] - 2.0 * a.sin()).abs() < 1e-3, "beam {f}");
        }
    }
}
