//! SLAM Thread - Real-time sensor processing.
//!
//! This thread:
//! - Receives sensor data via UDP from SangamIO
//! - Processes odometry and lidar data through SLAM engine
//! - Updates SharedState with pose, map, and sensor status
//! - Handles commands from Command Thread via channel (non-blocking)
//!
//! # Communication Architecture
//!
//! Event-driven via UDP:
//! - Sensors arrive via crossbeam channels from SangamUdpReceiver
//! - Uses `crossbeam::select!` to wait on sensor_status + lidar channels
//! - Processes immediately on arrival (no fixed rate loop)
//!
//! CRITICAL: This thread NEVER blocks on commands or publishing.
//! Sensor data flows uninterrupted.

use crossbeam_channel::select;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use crate::core::types::{LaserScan, Pose2D, Timestamped};
use crate::engine::pipeline::{PipelineConfig, SlamPipeline};
use crate::engine::slam::{SlamEngine, SlamMode};
use crate::io::sangam_udp_receiver::{
    LidarData, ReceiverConfig, SangamUdpReceiver, SensorStatusData,
};
use crate::state::{
    CommandReceiver, CommandResponse, CommandResult, PlacedScan, RobotState, SharedStateHandle,
    SlamCommand,
};

/// Configuration for the SLAM thread.
#[derive(Debug, Clone)]
pub struct SlamThreadConfig {
    /// SangamIO address for TCP commands and UDP sensor data.
    /// Format: "host:port" (e.g., "192.168.68.101:5555").
    /// Both TCP and UDP use the same port.
    pub sangam_address: String,
    /// UDP port override for sensor streaming (optional).
    /// If specified, UDP receiver will bind to this port instead of the TCP port.
    /// Useful for localhost testing where TCP and UDP need different ports.
    pub udp_port: Option<u16>,
    /// Odometry, preprocessing and SLAM engine.
    pub pipeline: PipelineConfig,
}

/// SLAM Thread handle.
pub struct SlamThread {
    handle: JoinHandle<()>,
}

impl SlamThread {
    /// Spawn the SLAM thread.
    pub fn spawn(
        config: SlamThreadConfig,
        shared_state: SharedStateHandle,
        command_rx: CommandReceiver,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("slam".into())
            .spawn(move || {
                run_live_loop(config, shared_state, command_rx, running);
            })
            .expect("Failed to spawn SLAM thread");

        Self { handle }
    }

    /// Wait for thread to finish.
    pub fn join(self) -> thread::Result<()> {
        self.handle.join()
    }
}

/// Run SLAM loop with live SangamIO UDP connection.
///
/// Uses event-driven architecture:
/// - UDP receiver runs in separate thread, sends data via crossbeam channels
/// - SLAM thread uses `select!` to wait on sensor_status + lidar channels
/// - Processes immediately on arrival (no fixed rate loop)
fn run_live_loop(
    config: SlamThreadConfig,
    shared_state: SharedStateHandle,
    command_rx: CommandReceiver,
    running: Arc<AtomicBool>,
) {
    log::info!("SLAM thread starting (live mode - UDP)");

    // Get UDP port: use explicit udp_port if set, otherwise extract from sangam_address
    let tcp_port = config
        .sangam_address
        .rsplit(':')
        .next()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(5555);
    let udp_port = config.udp_port.unwrap_or(tcp_port);

    let receiver_config = ReceiverConfig {
        bind_addr: format!("0.0.0.0:{}", udp_port),
    };

    if udp_port == tcp_port {
        log::info!(
            "  Starting UDP receiver on port {} (same as TCP)...",
            udp_port
        );
    } else {
        log::info!(
            "  Starting UDP receiver on port {} (TCP on {})...",
            udp_port,
            tcp_port
        );
    }

    // Create UDP receiver
    let (receiver, sensor_rx, lidar_rx) =
        match SangamUdpReceiver::new(receiver_config, running.clone()) {
            Ok(r) => r,
            Err(e) => {
                log::error!("Failed to create UDP receiver: {}", e);
                return;
            }
        };

    // Spawn UDP receiver thread
    let _receiver_running = running.clone();
    let receiver_handle = thread::Builder::new()
        .name("udp-receiver".into())
        .spawn(move || receiver.run())
        .expect("Failed to spawn UDP receiver thread");

    log::info!("  UDP receiver started, waiting for sensor data...");

    // Initialize components
    let mut slam_context = SlamContext::new(&config);

    // Main event loop - process sensor data as it arrives
    while running.load(Ordering::Relaxed) {
        // Check for commands (non-blocking)
        slam_context.process_commands(&command_rx, &shared_state);

        // Wait for sensor data from UDP channels using select!
        // This is event-driven: we process immediately when data arrives
        select! {
            recv(sensor_rx) -> result => {
                if let Ok(sensor_data) = result {
                    slam_context.process_udp_sensor_status(&sensor_data, &shared_state);
                    // Drain o resto do canal: o sensor chega a ~323Hz mas o SLAM
                    // thread so consome UM por iteracao -> se sobra canal cheio o
                    // receiver (try_send bounded=8) DROPA quase tudo e a odometria
                    // some (odom_delta ~0 no giro). Processar todos os que chegaram.
                    while let Ok(more) = sensor_rx.try_recv() {
                        slam_context.process_udp_sensor_status(&more, &shared_state);
                    }
                }
            }
            recv(lidar_rx) -> result => {
                if let Ok(lidar_data) = result {
                    slam_context.process_udp_lidar(&lidar_data, &shared_state);
                }
            }
            // Timeout to allow checking running flag and processing commands
            default(std::time::Duration::from_millis(10)) => {}
        }
    }

    log::info!("SLAM thread shutting down (waiting for UDP receiver)...");

    // Wait for UDP receiver to stop
    receiver_handle.join().ok();

    log::info!("SLAM thread shutdown complete");
}

/// Robot radius used for bumper obstacle marking.
const ROBOT_RADIUS: f32 = 0.18;

/// The SLAM thread's state: the processing pipeline plus what the daemon
/// publishes about it. The thread feeds it from the UDP receiver; `run_replay` and the
/// lab's sweep bench feed it the same messages from a recording or the simulator.
pub struct SlamContext {
    /// Odometry, preprocessing and SLAM engine.
    pipeline: SlamPipeline,
    /// Whether SLAM is in mapping mode.
    is_mapping: bool,
    /// Whether SLAM is in localization mode.
    is_localizing: bool,
    /// Total distance traveled.
    distance_traveled: f32,
    /// Last pose for distance computation.
    last_pose_for_distance: Pose2D,
    /// Data timestamp (us) of the last shared-state map update.
    last_map_update_us: u64,
    /// Previous bumper state for edge detection (left, right).
    prev_bumper_state: (bool, bool),
}

impl SlamContext {
    /// Idle until a `StartMapping` or `EnableMap` command.
    pub(crate) fn new(config: &SlamThreadConfig) -> Self {
        Self {
            pipeline: SlamPipeline::new(&config.pipeline),
            is_mapping: false,
            is_localizing: false,
            distance_traveled: 0.0,
            last_pose_for_distance: Pose2D::identity(),
            last_map_update_us: 0,
            prev_bumper_state: (false, false),
        }
    }

    /// Mapping from the first message on, as `StartMapping` leaves it.
    pub fn mapping(config: &SlamThreadConfig) -> Self {
        let mut ctx = Self::new(config);
        ctx.pipeline.slam_mut().set_mode(SlamMode::Mapping);
        ctx.is_mapping = true;
        ctx
    }

    /// Process commands from command channel (non-blocking).
    fn process_commands(&mut self, command_rx: &CommandReceiver, shared_state: &SharedStateHandle) {
        while let Ok(cmd_with_response) = command_rx.try_recv() {
            let result = self.handle_command(&cmd_with_response.command, shared_state);
            // Send response back (ignore send errors - command thread may have timed out)
            cmd_with_response.response_tx.send(result).ok();
        }
    }

    /// Handle a single SLAM command.
    fn handle_command(
        &mut self,
        cmd: &SlamCommand,
        shared_state: &SharedStateHandle,
    ) -> CommandResult {
        match cmd {
            SlamCommand::StartMapping { map_id, name } => {
                log::info!("Starting mapping: {} ({})", name, map_id);
                self.pipeline.slam_mut().reset();
                self.pipeline.slam_mut().set_mode(SlamMode::Mapping);
                self.is_mapping = true;
                self.is_localizing = false;
                self.distance_traveled = 0.0;
                self.last_pose_for_distance = Pose2D::identity();

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.start_mapping(map_id.clone(), name.clone());
                    // Enable exploration so the robot starts moving autonomously
                    state.exploration_state.mode = crate::state::ExplorationMode::Exploring;
                    state.exploration_state.status = "Starting mapping exploration".to_string();
                }

                Ok(CommandResponse::MappingStarted {
                    map_id: map_id.clone(),
                })
            }

            SlamCommand::StopMapping => {
                log::info!("Stopping mapping");
                self.pipeline.slam_mut().set_mode(SlamMode::Idle);
                self.is_mapping = false;

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.stop_mapping();
                    // Disable exploration when mapping stops
                    state.exploration_state.mode = crate::state::ExplorationMode::Disabled;
                    state.exploration_state.status = "Mapping stopped".to_string();
                }

                Ok(CommandResponse::MappingStopped)
            }

            SlamCommand::ClearMap => {
                log::info!("Clearing map");
                self.pipeline.slam_mut().reset();
                self.distance_traveled = 0.0;
                self.last_pose_for_distance = Pose2D::identity();

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.current_map = None;
                    state.robot_status.keyframe_count = 0;
                    state.robot_status.loop_closures = 0;
                    state.robot_status.distance_traveled_m = 0.0;
                    state.robot_status.map_area_m2 = 0.0;
                }

                Ok(CommandResponse::MapCleared)
            }

            SlamCommand::EnableMap { map_id, map, name } => {
                log::info!("Enabling map for localization: {} ({})", name, map_id);

                // Set the reference map for localization matching
                self.pipeline.slam_mut().set_reference_map(map.clone());

                // Set localization mode
                self.pipeline.slam_mut().set_mode(SlamMode::Localization);
                self.is_mapping = false;
                self.is_localizing = true;

                // Update shared state
                if let Ok(mut state) = shared_state.write() {
                    state.enable_localization(map_id.clone());
                    state.current_map_name = name.clone();
                    state.update_map(map);
                }

                Ok(CommandResponse::MapEnabled)
            }

            SlamCommand::EmergencyStop => {
                log::warn!("Emergency stop activated!");

                // Stop mapping/localization
                self.pipeline.slam_mut().set_mode(SlamMode::Idle);
                self.is_mapping = false;
                self.is_localizing = false;

                // Update shared state to stop exploration (which will stop motors and lidar)
                if let Ok(mut state) = shared_state.write() {
                    state.stop_mapping();
                    state.exploration_state.mode = crate::state::ExplorationMode::Disabled;
                    state.exploration_state.status = "Emergency stopped".to_string();
                }

                Ok(CommandResponse::EmergencyStopped)
            }
        }
    }

    /// Common odometry processing.
    fn process_odometry(
        &mut self,
        left: u16,
        right: u16,
        gyro_yaw: i16,
        timestamp_us: u64,
        shared_state: &SharedStateHandle,
    ) {
        if let Some(odom_pose) = self
            .pipeline
            .on_odometry(left, right, gyro_yaw, timestamp_us)
        {
            static ODOM_LOG_COUNT: std::sync::atomic::AtomicU32 =
                std::sync::atomic::AtomicU32::new(0);
            let c = ODOM_LOG_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if c.is_multiple_of(200) {
                log::debug!(
                    "[odom] left={} right={} gyro={} -> pose=({:.3},{:.3},{:.1}°)",
                    left,
                    right,
                    gyro_yaw,
                    odom_pose.x,
                    odom_pose.y,
                    odom_pose.theta.to_degrees()
                );
            }
            let published_pose = self.pipeline.interpolated_pose();

            // Update distance traveled
            let dx = published_pose.x - self.last_pose_for_distance.x;
            let dy = published_pose.y - self.last_pose_for_distance.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist > 0.01 {
                // Only count if moved more than 1cm
                self.distance_traveled += dist;
                self.last_pose_for_distance = published_pose;
            }

            // Update shared state (quick write)
            if let Ok(mut state) = shared_state.write() {
                state.robot_status.pose = published_pose;
                state.robot_status.timestamp_us = timestamp_us;
                state.robot_status.distance_traveled_m = self.distance_traveled;

                // Update sensor status
                state.sensor_status.left_encoder_ticks = left as i32;
                state.sensor_status.right_encoder_ticks = right as i32;
                state.sensor_status.gyro_z_radps = gyro_yaw as f32 * 0.001; // Rough conversion
                state.sensor_status.raw_odometry = odom_pose;
                state.sensor_status.timestamp_us = timestamp_us;
            }
        }
    }

    /// Process lidar scan.
    fn process_lidar(
        &mut self,
        lidar_data: &crate::io::LidarScan,
        timestamp_us: u64,
        shared_state: &SharedStateHandle,
    ) {
        let process = self.is_mapping || self.is_localizing;
        let laser = LaserScan::from_lidar_scan(lidar_data);
        let Some(step) = self.pipeline.on_scan(laser, timestamp_us, process) else {
            return; // too sparse
        };

        // Log scan processing decision (debug level - routine idle state)
        if !process {
            static SKIP_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
            let count = SKIP_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if count < 5 || count.is_multiple_of(50) {
                log::debug!(
                    "Skipping scan (not mapping/localizing): is_mapping={}, is_localizing={}, skip#{}",
                    self.is_mapping,
                    self.is_localizing,
                    count
                );
            }
        }

        if let Some(result) = &step.result {
            let odom_delta = step.odom_delta;
            // Log pose updates at debug level (high frequency diagnostic data)
            static POSE_LOG_COUNT: std::sync::atomic::AtomicU32 =
                std::sync::atomic::AtomicU32::new(0);
            let log_count = POSE_LOG_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if log_count < 5 || log_count.is_multiple_of(50) {
                log::debug!(
                    "SLAM result: pose=({:.3}, {:.3}, {:.1}°), score={:.3}, odom_delta=({:.3}, {:.3}, {:.1}°)",
                    result.pose.x,
                    result.pose.y,
                    result.pose.theta.to_degrees(),
                    result.match_score,
                    odom_delta.x,
                    odom_delta.y,
                    odom_delta.theta.to_degrees()
                );
            }

            // Update shared state
            if let Ok(mut state) = shared_state.write() {
                let status = self.pipeline.slam().status();
                state.update_from_slam(
                    result.pose,
                    if self.is_mapping {
                        RobotState::Mapping
                    } else if status.is_lost {
                        RobotState::Lost
                    } else {
                        RobotState::Localizing
                    },
                    status.num_keyframes as u32,
                    self.pipeline.slam().total_loop_closures() as u32,
                    result.match_score,
                    timestamp_us,
                );

                // Update lidar data in sensor status
                // Apply lidar angle_offset to transform from lidar frame to robot frame
                let angle_offset = self.pipeline.preprocessor().lidar_offset().total_theta();
                let laser_scan = &step.laser;
                let placed = PlacedScan {
                    cloud: step.cloud.clone(),
                    pose: result.pose,
                };
                state.sensor_status.scan_cloud = Timestamped::new(placed, timestamp_us);
                state.sensor_status.lidar_ranges = laser_scan.ranges.clone();
                state.sensor_status.lidar_angles = (0..laser_scan.ranges.len())
                    .map(|i| {
                        let raw_angle =
                            laser_scan.angle_min + i as f32 * laser_scan.angle_increment;
                        raw_angle + angle_offset // Apply lidar angular offset
                    })
                    .collect();
            }

            // Update map periodically (not every scan - expensive)
            // Data time, not wall time: same cadence live, deterministic on replay.
            if timestamp_us.saturating_sub(self.last_map_update_us) > 500_000 {
                if let Ok(mut state) = shared_state.write() {
                    let map = self.pipeline.slam_mut().global_map();
                    let (w, h) = map.dimensions();
                    log::debug!("Updating map in shared state: {}x{}", w, h);
                    state.update_map(map);
                }
                self.last_map_update_us = timestamp_us;
            }
        }
    }

    /// Process sensor status from UDP receiver.
    pub fn process_udp_sensor_status(
        &mut self,
        data: &SensorStatusData,
        shared_state: &SharedStateHandle,
    ) {
        let timestamp_us = data.timestamp_us;

        // Process odometry
        self.process_odometry(
            data.left_encoder,
            data.right_encoder,
            data.gyro_yaw_raw,
            timestamp_us,
            shared_state,
        );

        // Detect bumper rising edge and mark obstacle in map
        // Only trigger on rising edge to avoid marking multiple times for same collision
        let bumper_left_rising = data.bumper_left && !self.prev_bumper_state.0;
        let bumper_right_rising = data.bumper_right && !self.prev_bumper_state.1;

        if (bumper_left_rising || bumper_right_rising) && self.is_mapping {
            // Get current pose for obstacle marking
            let pose = self.pipeline.slam().current_pose();
            log::info!(
                "Bumper triggered (left={}, right={}) at pose ({:.2}, {:.2}, {:.2}°) - marking obstacle",
                data.bumper_left,
                data.bumper_right,
                pose.x,
                pose.y,
                pose.theta.to_degrees()
            );

            // Mark obstacle in map
            self.pipeline.slam_mut().mark_bumper_obstacle(
                pose.x,
                pose.y,
                pose.theta,
                data.bumper_left,
                data.bumper_right,
                ROBOT_RADIUS,
            );
        }

        // Update previous bumper state for next iteration
        self.prev_bumper_state = (data.bumper_left, data.bumper_right);

        // Update bumper and cliff states in shared state
        if let Ok(mut state) = shared_state.write() {
            // Update bumper state
            state.sensor_status.bumper.left_pressed = data.bumper_left;
            state.sensor_status.bumper.right_pressed = data.bumper_right;
            if data.bumper_left || data.bumper_right {
                state.sensor_status.bumper.last_trigger_us = Some(timestamp_us);
            }

            // Update cliff state
            state.sensor_status.cliff.left_side = data.cliff_left_side;
            state.sensor_status.cliff.left_front = data.cliff_left_front;
            state.sensor_status.cliff.right_front = data.cliff_right_front;
            state.sensor_status.cliff.right_side = data.cliff_right_side;
            if data.cliff_left_side
                || data.cliff_left_front
                || data.cliff_right_front
                || data.cliff_right_side
            {
                state.sensor_status.cliff.last_trigger_us = Some(timestamp_us);
            }

            state.robot_status.is_charging = data.is_charging;
            state.robot_status.is_docked = data.is_docked;
        }
    }

    /// Process lidar data from UDP receiver.
    pub fn process_udp_lidar(&mut self, data: &LidarData, shared_state: &SharedStateHandle) {
        let timestamp_us = data.timestamp_us;

        // Log scans at debug level (high frequency diagnostic data)
        static SCAN_COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let count = SCAN_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if count < 5 || count.is_multiple_of(50) {
            log::debug!(
                "Processing UDP lidar: {} points, is_mapping={}, is_localizing={}, scan#{}",
                data.points.len(),
                self.is_mapping,
                self.is_localizing,
                count
            );
        }

        // Convert UDP LidarData to LidarScan format expected by process_lidar
        let lidar_scan: crate::io::LidarScan = data
            .points
            .iter()
            .map(|(angle, dist, quality)| (*angle, *dist, *quality))
            .collect();

        self.process_lidar(&lidar_scan, timestamp_us, shared_state);
    }
}

// ============================================================================
// Offline replay (deterministic benchmark)
// ============================================================================

/// Summary of an offline replay run.
pub struct ReplayReport {
    pub scans: usize,
    pub sensor_msgs: usize,
    pub data_span_s: f64,
    pub wall_s: f64,
    pub scan_ms: Vec<f64>,
}

/// Replay a recorded SangamIO stream (see `io::recording`: an MCAP file, a run
/// directory of MCAP segments, or an older SGIO1 file) through the same SLAM
/// pipeline the daemon runs, synchronously, as fast as possible.
///
/// Writes into `out_dir`: `timing.csv` (per scan: data time, processing ms, pose,
/// score, keyframes), `map.pgm` + `map.yaml` (ROS map_server format).
pub fn run_replay(
    config: SlamThreadConfig,
    input: &std::path::Path,
    out_dir: &std::path::Path,
) -> std::io::Result<ReplayReport> {
    use crate::io::recording::for_each_sample;
    use crate::io::sangam_udp_receiver::Sample;
    use std::io::Write;

    std::fs::create_dir_all(out_dir)?;
    let shared_state = crate::state::create_shared_state();
    let mut ctx = SlamContext::mapping(&config);

    let mut csv = std::io::BufWriter::new(std::fs::File::create(out_dir.join("timing.csv"))?);
    writeln!(csv, "t_us,ms,x,y,theta,score,keyframes,points")?;

    let mut report = ReplayReport {
        scans: 0,
        sensor_msgs: 0,
        data_span_s: 0.0,
        wall_s: 0.0,
        scan_ms: Vec::new(),
    };
    let (mut t_first, mut t_last) = (0u64, 0u64);
    let wall = Instant::now();
    for_each_sample(input, |sample| {
        let t = sample.timestamp_us();
        if t_first == 0 {
            t_first = t;
        }
        t_last = t_last.max(t);
        match sample {
            Sample::Status(d) => {
                ctx.process_udp_sensor_status(&d, &shared_state);
                report.sensor_msgs += 1;
            }
            Sample::Lidar(d) => {
                let t0 = Instant::now();
                ctx.process_udp_lidar(&d, &shared_state);
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                let p = ctx.pipeline.slam().current_pose();
                let st = ctx.pipeline.slam().status();
                let score = shared_state
                    .read()
                    .map(|s| s.robot_status.localization_confidence)
                    .unwrap_or(0.0);
                writeln!(
                    csv,
                    "{},{:.3},{:.4},{:.4},{:.5},{:.3},{},{}",
                    d.timestamp_us,
                    ms,
                    p.x,
                    p.y,
                    p.theta,
                    score,
                    st.num_keyframes,
                    d.points.len()
                )?;
                report.scan_ms.push(ms);
                report.scans += 1;
            }
        }
        Ok(())
    })?;
    report.wall_s = wall.elapsed().as_secs_f64();
    report.data_span_s = t_last.saturating_sub(t_first) as f64 / 1e6;
    csv.flush()?;

    // Map in ROS map_server format (0 occupied, 254 free, 205 unknown).
    let map = crate::state::CurrentMapData::from_occupancy_grid(
        ctx.pipeline.slam_mut().global_map(),
        "replay",
        "replay",
    );
    let (w, h) = (map.width as usize, map.height as usize);
    let mut pgm = std::io::BufWriter::new(std::fs::File::create(out_dir.join("map.pgm"))?);
    write!(pgm, "P5\n{} {}\n255\n", w, h)?;
    for cy in (0..h).rev() {
        let row: Vec<u8> = (0..w)
            .map(|cx| match map.cells[cy * w + cx] {
                0..=49 => 254,
                50..=150 => 0,
                _ => 205,
            })
            .collect();
        pgm.write_all(&row)?;
    }
    std::fs::write(
        out_dir.join("map.yaml"),
        format!(
            "image: map.pgm\nresolution: {}\norigin: [{}, {}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            map.resolution, map.origin_x, map.origin_y
        ),
    )?;
    Ok(report)
}
