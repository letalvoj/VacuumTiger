//! TOML configuration (`dhruva-*.toml`) and the engine configs built from it.
//!
//! Shared by the daemon (`main.rs`), `--replay` and offline tools built on the library.

use std::path::Path;

use serde::Deserialize;

use crate::algorithms::mapping::{MapIntegratorConfig, OccupancyGridConfig};
use crate::algorithms::matching::{
    CorrelativeConfig, DynMatcherConfig, IcpConfig, MatcherType, MultiResolutionConfig,
    PointToLineIcpConfig, RobustKernel,
};
use crate::engine::graph::{GraphOptimizerConfig, LoopDetectorConfig};
use crate::engine::pipeline::PipelineConfig;
use crate::engine::slam::{
    KeyframeManagerConfig, LoopClosureConfig as SlamLoopClosureConfig, OnlineSlamConfig,
    SubmapManagerConfig,
};
use crate::exploration::WallFollowConfig;
use crate::sensors::odometry::{
    ComplementaryConfig, DynOdometryConfig, MahonyConfig, OdometryType, WheelOdometryConfig,
};
use crate::sensors::preprocessing::{
    AngularDownsamplerConfig, LidarOffset, OutlierFilterConfig, PreprocessorConfig,
    RangeFilterConfig, Sweep,
};
use crate::threads::SlamThreadConfig;

#[derive(Debug, Deserialize, Default)]
pub struct Config {
    #[serde(default)]
    pub source: SourceConfig,
    #[serde(default)]
    pub output: OutputConfig,
    #[serde(default)]
    pub stream: StreamConfig,
    #[serde(default)]
    pub map_storage: MapStorageConfig,
    #[serde(default)]
    pub preprocessing: PreprocessingConfig,
    #[serde(default)]
    pub slam: SlamConfig,
    #[serde(default)]
    pub lidar: LidarConfig,
    #[serde(default)]
    pub odometry: OdometryConfig,
    #[serde(default)]
    pub matcher: MatcherConfig,
    #[serde(default)]
    pub keyframe: KeyframeConfig,
    #[serde(default)]
    pub submap: SubmapConfig,
    #[serde(default)]
    pub map: MapConfig,
    #[serde(default)]
    pub loop_closure: LoopClosureConfig,
    #[serde(default)]
    pub optimization: OptimizationConfig,
    #[serde(default)]
    pub exploration: ExplorationCfg,
    #[serde(default)]
    pub navigation: NavigationConfig,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct SourceConfig {
    /// SangamIO address for TCP commands and UDP sensor data
    /// Format: "host:port" (e.g., "192.168.68.101:5555")
    /// Both TCP and UDP use the same port for simpler configuration.
    pub sangam_address: String,
    /// UDP port override for sensor streaming (optional).
    /// If specified, UDP receiver will bind to this port instead of the TCP port.
    /// Useful for localhost testing where TCP and UDP need different ports.
    pub udp_port: Option<u16>,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            sangam_address: "192.168.68.101:5555".to_string(),
            udp_port: None,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct OutputConfig {
    pub bind_port: u16,
    pub odometry_rate_hz: f32,
    pub map_rate_hz: f32,
    pub feature_rate_hz: f32,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            bind_port: 5557,
            odometry_rate_hz: 50.0,
            map_rate_hz: 1.0,
            feature_rate_hz: 0.2, // Every 5 seconds
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct StreamConfig {
    pub robot_status_hz: f32,
    pub sensor_status_hz: f32,
    pub map_hz: f32,
    pub navigation_hz: f32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            robot_status_hz: 10.0,
            sensor_status_hz: 10.0,
            map_hz: 1.0,
            navigation_hz: 5.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct MapStorageConfig {
    pub path: String,
}

impl Default for MapStorageConfig {
    fn default() -> Self {
        Self {
            path: "/var/lib/dhruva/maps".to_string(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct PreprocessingConfig {
    pub min_range: f32,
    pub max_range: f32,
    pub target_points: usize,
    pub min_angle_step: f32,
}

impl Default for PreprocessingConfig {
    fn default() -> Self {
        Self {
            min_range: 0.15,
            max_range: 8.0,
            target_points: 180,
            min_angle_step: 0.0175, // ~1 degree
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct SlamConfig {
    pub min_match_score: f32,
    pub lost_threshold: f32,
    pub min_scan_points: usize,
    pub encoder_weight: f32,
    pub use_submap_matching: bool,
    /// Maximum rotation deviation from odometry before rejecting scan match (radians)
    pub max_theta_deviation: f32,
    /// Maximum translation deviation from odometry before rejecting scan match (meters)
    pub max_translation_deviation: f32,
    /// Reference-scan matching: move the reference after this distance (m); 0 = scan-to-scan
    pub reference_update_distance: f32,
    /// ... or after this rotation (rad)
    pub reference_update_angle: f32,
    /// Local-map matching: the last this many keyframes; 0 = off
    pub local_map_keyframes: usize,
    /// ... within this distance (m) of the predicted pose
    pub local_map_max_distance: f32,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            min_match_score: 0.3,
            lost_threshold: 0.1,
            min_scan_points: 50,
            encoder_weight: 0.8,
            use_submap_matching: false,
            max_theta_deviation: 0.35,      // ~20 degrees
            max_translation_deviation: 0.5, // 50cm
            reference_update_distance: 0.0,
            reference_update_angle: 0.3,
            local_map_keyframes: 0,
            local_map_max_distance: 8.0,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct LidarConfig {
    pub mounting_x: f32,
    pub mounting_y: f32,
    pub optical_offset: f32,
    pub angle_offset: f32,
    /// Angle (rad) of a sweep's first beam, in the frame of the received scan (before
    /// `angle_offset`). Set it to deskew scans with the odometry (the sweep runs
    /// clockwise and is stamped at its end, as SangamIO's Delta-2D driver publishes
    /// it); unset, scans are taken as instantaneous.
    pub sweep_start_angle: Option<f32>,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            mounting_x: -0.0936,
            mounting_y: 0.0,
            optical_offset: 0.0258,
            angle_offset: 0.0,
            sweep_start_angle: None,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct OdometryConfig {
    /// Odometry algorithm: "wheel", "complementary", "eskf", "mahony"
    pub algorithm: String,
    /// Encoder ticks per meter of wheel travel
    pub ticks_per_meter: f32,
    /// Distance between wheel centers in meters
    pub wheel_base: f32,
    /// "complementary" only: gyro share (0..=1) of the heading increment while the
    /// wheels move; while they are still the wheel heading is used.
    pub gyro_weight: f32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            algorithm: "mahony".to_string(),
            // CRL-200S defaults measured on this unit (2026-09-30, tape measure):
            // 1 tick = 0.2214 mm -> 4516.7 ticks/m; 209 mm wheel base.
            ticks_per_meter: 4516.7,
            wheel_base: 0.209,
            gyro_weight: ComplementaryConfig::default().alpha,
        }
    }
}

impl OdometryConfig {
    pub fn algorithm_type(&self) -> OdometryType {
        match self.algorithm.to_lowercase().as_str() {
            "wheel" => OdometryType::Wheel,
            "complementary" => OdometryType::Complementary,
            "eskf" => OdometryType::Eskf,
            _ => OdometryType::Mahony,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct MatcherConfig {
    /// Matcher algorithm: "icp", "p2l", "correlative", "multi_res", "hybrid_icp", "hybrid_p2l"
    pub algorithm: String,
    /// Always run correlative first
    pub always_correlative: bool,
    /// Encoder weight for initial guess
    pub encoder_weight: f32,
    // Correlative search window
    pub search_window_x: f32,
    pub search_window_y: f32,
    pub search_window_theta: f32,
    pub linear_resolution: f32,
    pub angular_resolution: f32,
    pub grid_resolution: f32,
    pub correlative_min_score: f32,
    // ICP configuration
    pub icp_max_iterations: u32,
    pub icp_translation_epsilon: f32,
    pub icp_rotation_epsilon: f32,
    pub icp_max_correspondence_distance: f32,
    pub icp_min_correspondences: usize,
    pub icp_outlier_ratio: f32,
    pub icp_robust_kernel: String,
    pub icp_kernel_scale: f32,
    // Multi-resolution matcher search window (coarse level); defaults match
    // MultiResolutionConfig::default() (0.5 m, 0.5 rad).
    pub multires_window_xy: f32,
    pub multires_window_theta: f32,
    pub multires_levels: usize,
}

impl Default for MatcherConfig {
    fn default() -> Self {
        Self {
            algorithm: "hybrid_p2l".to_string(),
            always_correlative: true,
            encoder_weight: 1.0,
            search_window_x: 0.3,
            search_window_y: 0.3,
            search_window_theta: 0.5,
            linear_resolution: 0.03,
            angular_resolution: 0.03,
            grid_resolution: 0.05,
            correlative_min_score: 0.4,
            icp_max_iterations: 50,
            icp_translation_epsilon: 0.001,
            icp_rotation_epsilon: 0.001,
            icp_max_correspondence_distance: 0.5,
            icp_min_correspondences: 10,
            icp_outlier_ratio: 0.1,
            icp_robust_kernel: "welsch".to_string(),
            icp_kernel_scale: 0.1,
            multires_window_xy: 0.5,
            multires_window_theta: 0.5,
            multires_levels: 3,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct KeyframeConfig {
    pub min_translation: f32,
    pub min_rotation: f32,
    pub max_keyframes: usize,
    pub min_interval_ms: u64,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        Self {
            min_translation: 0.5,
            min_rotation: 0.5,
            max_keyframes: 1000,
            min_interval_ms: 500,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct SubmapConfig {
    pub scans_per_submap: u32,
    pub overlap_scans: u32,
    pub max_active_submaps: usize,
}

impl Default for SubmapConfig {
    fn default() -> Self {
        Self {
            scans_per_submap: 100,
            overlap_scans: 10,
            max_active_submaps: 2,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct MapConfig {
    pub resolution: f32,
    pub initial_width: f32,
    pub initial_height: f32,
    pub log_odds_occupied: f32,
    pub log_odds_free: f32,
    pub log_odds_max: f32,
    pub log_odds_min: f32,
    pub occupied_threshold: f32,
    pub free_threshold: f32,
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            resolution: 0.02,
            initial_width: 20.0,
            initial_height: 20.0,
            log_odds_occupied: 0.9,
            log_odds_free: -0.7,
            log_odds_max: 50.0,
            log_odds_min: -50.0,
            occupied_threshold: 0.5,
            free_threshold: -0.5,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct LoopClosureConfig {
    pub enabled: bool,
    /// Look for a closure on every N-th keyframe.
    pub detection_interval: usize,
    #[serde(flatten)]
    pub detector: LoopDetectorConfig,
}

impl Default for LoopClosureConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            detection_interval: 5,
            detector: LoopDetectorConfig::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct OptimizationConfig {
    pub solver: String,
    pub max_iterations: usize,
    pub tolerance: f64,
}

impl Default for OptimizationConfig {
    fn default() -> Self {
        Self {
            solver: "sparse_cg".to_string(),
            max_iterations: 50,
            tolerance: 1e-6,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct ExplorationCfg {
    /// Enable exploration thread.
    pub enabled: bool,
    /// Exploration strategy: "frontier" or "wall_follow" (the corner sweep).
    pub strategy: String,
    /// Loop rate for exploration thread (Hz).
    pub loop_rate_hz: f32,
    #[serde(default)]
    pub motion: ExplorationMotionConfig,
    #[serde(default)]
    pub frontier: ExplorationFrontierConfig,
    #[serde(default)]
    pub wall_follow: WallFollowConfig,
}

impl Default for ExplorationCfg {
    fn default() -> Self {
        Self {
            enabled: true,
            strategy: "frontier".to_string(),
            loop_rate_hz: 10.0,
            motion: ExplorationMotionConfig::default(),
            frontier: ExplorationFrontierConfig::default(),
            wall_follow: WallFollowConfig::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct ExplorationMotionConfig {
    /// Maximum linear velocity (m/s).
    pub max_linear_vel: f32,
    /// Maximum angular velocity (rad/s).
    pub max_angular_vel: f32,
    /// Minimum distance from obstacles (m).
    pub obstacle_clearance: f32,
    /// Normal linear velocity (m/s).
    pub linear_vel: f32,
    /// Normal angular velocity for rotation (rad/s).
    pub angular_vel: f32,
}

impl Default for ExplorationMotionConfig {
    fn default() -> Self {
        Self {
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            obstacle_clearance: 0.15,
            linear_vel: 0.2,
            angular_vel: 0.3,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct ExplorationFrontierConfig {
    /// Minimum frontier size to consider (cells).
    pub min_frontier_size: usize,
    /// Maximum detection range (m).
    pub detection_range: f32,
    /// Radius to block after bumper trigger (m).
    pub blocked_radius: f32,
    /// Clustering threshold for frontier grouping (m).
    pub clustering_threshold: f32,
}

impl Default for ExplorationFrontierConfig {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            detection_range: 10.0,
            blocked_radius: 0.3,
            clustering_threshold: 0.5,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct NavigationConfig {
    /// Enable navigation thread.
    pub enabled: bool,
    /// Update rate for navigation thread (Hz).
    pub update_rate_hz: f32,
    /// Robot radius for A* inflation (m).
    pub robot_radius: f32,
    /// Safety margin beyond robot radius (m).
    /// Adds extra clearance to prevent getting too close to walls.
    pub safety_margin: f32,
    /// Maximum linear velocity (m/s).
    pub max_linear_vel: f32,
    /// Maximum angular velocity (rad/s).
    pub max_angular_vel: f32,
    /// Waypoint reached threshold (m).
    pub waypoint_threshold: f32,
    /// Heading threshold (rad).
    pub heading_threshold: f32,
    /// Treat unknown cells as obstacles for path planning.
    /// Set to false to allow navigation through unexplored areas.
    pub unknown_is_obstacle: bool,
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            update_rate_hz: 10.0,
            robot_radius: 0.18,
            safety_margin: 0.10, // 10cm extra clearance from walls
            max_linear_vel: 0.3,
            max_angular_vel: 0.5,
            waypoint_threshold: 0.15,
            heading_threshold: 0.15,
            unknown_is_obstacle: true,
        }
    }
}

impl Config {
    /// Parse a TOML config file.
    pub fn from_file(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        basic_toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
    }
}

/// Build OnlineSlamConfig from TOML configuration.
pub fn build_slam_config(config: &Config) -> OnlineSlamConfig {
    // Parse matcher type from config string
    let matcher_type = match config.matcher.algorithm.to_lowercase().as_str() {
        "icp" => MatcherType::Icp,
        "p2l" => MatcherType::P2l,
        "correlative" => MatcherType::Correlative,
        "multi_res" => MatcherType::MultiRes,
        "hybrid_icp" => MatcherType::HybridIcp,
        _ => MatcherType::HybridP2l, // default
    };

    // Build correlative config from matcher settings
    let correlative_config = CorrelativeConfig {
        search_window_x: config.matcher.search_window_x,
        search_window_y: config.matcher.search_window_y,
        search_window_theta: config.matcher.search_window_theta,
        linear_resolution: config.matcher.linear_resolution,
        angular_resolution: config.matcher.angular_resolution,
        grid_resolution: config.matcher.grid_resolution,
        min_score: config.matcher.correlative_min_score,
    };

    // Build P2P ICP config from matcher settings (with robust kernel)
    let robust_kernel = match config.matcher.icp_robust_kernel.to_lowercase().as_str() {
        "huber" => RobustKernel::Huber,
        "cauchy" => RobustKernel::Cauchy,
        "welsch" => RobustKernel::Welsch,
        _ => RobustKernel::None,
    };

    let icp_config = IcpConfig {
        max_iterations: config.matcher.icp_max_iterations,
        translation_epsilon: config.matcher.icp_translation_epsilon,
        rotation_epsilon: config.matcher.icp_rotation_epsilon,
        max_correspondence_distance: config.matcher.icp_max_correspondence_distance,
        min_correspondences: config.matcher.icp_min_correspondences,
        outlier_ratio: config.matcher.icp_outlier_ratio,
        robust_kernel,
        kernel_scale: config.matcher.icp_kernel_scale,
        ..Default::default()
    };

    // Build P2L ICP config from matcher settings
    // Note: P2L ICP doesn't support robust kernels - those are only in P2P ICP
    let p2l_config = PointToLineIcpConfig {
        max_iterations: config.matcher.icp_max_iterations,
        translation_epsilon: config.matcher.icp_translation_epsilon,
        rotation_epsilon: config.matcher.icp_rotation_epsilon,
        max_correspondence_distance: config.matcher.icp_max_correspondence_distance,
        min_correspondences: config.matcher.icp_min_correspondences,
        outlier_ratio: config.matcher.icp_outlier_ratio,
        ..Default::default()
    };

    // Build multi-resolution config (uses default values for most settings)
    let multi_res_config = MultiResolutionConfig {
        min_score: config.matcher.correlative_min_score,
        coarse_search_window_xy: config.matcher.multires_window_xy,
        coarse_search_window_theta: config.matcher.multires_window_theta,
        num_levels: config.matcher.multires_levels.max(1),
        ..Default::default()
    };

    // Build dynamic matcher config
    let matcher_config = DynMatcherConfig {
        correlative: correlative_config,
        icp: icp_config,
        p2l: p2l_config,
        multi_res: multi_res_config,
        always_correlative: config.matcher.always_correlative,
        encoder_weight: config.matcher.encoder_weight,
    };

    // Build global map config
    let global_map_config = OccupancyGridConfig {
        resolution: config.map.resolution,
        initial_width: config.map.initial_width,
        initial_height: config.map.initial_height,
        log_odds_occupied: config.map.log_odds_occupied,
        log_odds_free: config.map.log_odds_free,
        log_odds_max: config.map.log_odds_max,
        log_odds_min: config.map.log_odds_min,
        occupied_threshold: config.map.occupied_threshold,
        free_threshold: config.map.free_threshold,
    };

    // Build keyframe manager config
    let keyframe_config = KeyframeManagerConfig {
        min_translation: config.keyframe.min_translation,
        min_rotation: config.keyframe.min_rotation,
        max_keyframes: config.keyframe.max_keyframes,
        min_interval_us: config.keyframe.min_interval_ms * 1000,
    };

    // Build submap manager config
    let submap_config = SubmapManagerConfig {
        grid_config: OccupancyGridConfig {
            resolution: config.map.resolution,
            initial_width: 10.0, // Smaller for submaps
            initial_height: 10.0,
            ..global_map_config.clone()
        },
        integrator_config: MapIntegratorConfig::default(),
        scans_per_submap: config.submap.scans_per_submap,
        overlap_scans: config.submap.overlap_scans,
    };

    OnlineSlamConfig {
        keyframe: keyframe_config,
        submap: submap_config,
        matcher_type,
        matcher: matcher_config,
        global_map: global_map_config,
        min_match_score: config.slam.min_match_score,
        lost_threshold: config.slam.lost_threshold,
        use_submap_matching: config.slam.use_submap_matching,
        min_scan_points: config.slam.min_scan_points,
        max_theta_deviation: config.slam.max_theta_deviation,
        max_translation_deviation: config.slam.max_translation_deviation,
        reference_update_distance: config.slam.reference_update_distance,
        reference_update_angle: config.slam.reference_update_angle,
        local_map_keyframes: config.slam.local_map_keyframes,
        local_map_max_distance: config.slam.local_map_max_distance,
        loop_closure: SlamLoopClosureConfig {
            enabled: config.loop_closure.enabled,
            detection_interval: config.loop_closure.detection_interval,
            detector: config.loop_closure.detector.clone(),
            optimizer: GraphOptimizerConfig {
                max_iterations: config.optimization.max_iterations as u32,
                convergence_threshold: config.optimization.tolerance,
                ..Default::default()
            },
        },
    }
}

/// SLAM thread config from the TOML config (shared by the daemon and `--replay`).
pub fn build_slam_thread_config(config: &Config) -> SlamThreadConfig {
    let slam_config = build_slam_config(config);
    let odometry_config = DynOdometryConfig {
        wheel: WheelOdometryConfig {
            ticks_per_meter: config.odometry.ticks_per_meter,
            wheel_base: config.odometry.wheel_base,
        },
        complementary: ComplementaryConfig {
            alpha: config.odometry.gyro_weight,
            ..Default::default()
        },
        mahony: MahonyConfig::default(),
        ..Default::default()
    };
    let preprocessor_config = PreprocessorConfig {
        range_filter: RangeFilterConfig {
            min_range: config.preprocessing.min_range,
            max_range: config.preprocessing.max_range,
        },
        outlier_filter: OutlierFilterConfig::default(),
        downsampler: AngularDownsamplerConfig {
            target_points: config.preprocessing.target_points,
            min_angle_step: config.preprocessing.min_angle_step,
        },
        lidar_offset: LidarOffset::new(
            config.lidar.mounting_x,
            config.lidar.mounting_y,
            config.lidar.optical_offset,
            config.lidar.angle_offset,
        ),
    };

    SlamThreadConfig {
        sangam_address: config.source.sangam_address.clone(),
        udp_port: config.source.udp_port,
        pipeline: PipelineConfig {
            slam_config,
            odometry_type: config.odometry.algorithm_type(),
            odometry_config,
            preprocessor_config,
            sweep: config.lidar.sweep_start_angle.map(|start_angle| Sweep { start_angle }),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loop_closure_keys_are_flat_and_optional() {
        let config: LoopClosureConfig =
            basic_toml::from_str("enabled = false\nsearch_radius = 2.5\nmin_score = 0.7\n")
                .unwrap();
        let defaults = LoopDetectorConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.detection_interval, 5);
        assert_eq!(config.detector.search_radius, 2.5);
        assert_eq!(config.detector.min_score, 0.7);
        assert_eq!(config.detector.area_keyframes, defaults.area_keyframes);
    }
}
