//! Mock device driver for hardware-free robot simulation
//!
//! This module provides a complete simulation of CRL-200S robot hardware,
//! enabling SLAM and navigation algorithm development without physical robots.
//!
//! # Overview
//!
//! The mock driver simulates all sensors and actuators of the CRL-200S:
//!
//! | Component | Simulation Method |
//! |-----------|-------------------|
//! | Drive | Delayed, lagged response to the command; the bumper interlock |
//! | Lidar (Delta-2D) | Beams ray-cast from the mount as it turns, against the lidar mask or grid |
//! | Wheel encoders | Differential drive kinematics + slip noise |
//! | IMU (gyro/accel) | Physics-based with configurable noise |
//! | Bumpers | Collision detection against map |
//! | Cliff sensors | Cliff mask lookup or boundary detection |
//! | Battery/buttons | Fixed configurable states |
//!
//! [`MockRobot`] is the simulated robot, stepped by its caller: this driver steps it in real
//! time, benches as fast as they like. `mock_bench.toml` is the v8 as measured.
//!
//! # Map Format
//!
//! Uses ROS-standard PGM + YAML format for maximum compatibility:
//!
//! ```yaml
//! # maps/example.yaml
//! image: example.pgm        # Grayscale occupancy grid
//! resolution: 0.02          # meters per pixel
//! origin: [-5.0, -5.0, 0.0] # [x, y, yaw] of bottom-left pixel
//! occupied_thresh: 0.65     # Darker = occupied
//! cliff_mask: cliffs.pgm    # Optional cliff layer
//! lidar_mask: lidar.pgm     # Optional: what the lidar sees (glass, curtains)
//! ```
//!
//! PGM pixel values:
//! - **White (255)**: Free space
//! - **Black (0)**: Occupied/wall
//! - **Gray (205)**: Unknown
//!
//! # Configuration
//!
//! Enable the `mock` feature to use this driver:
//!
//! ```bash
//! cargo build --features mock
//! cargo run --features mock -- mock.toml
//! ```
//!
//! Example configuration (`mock.toml`):
//!
//! ```toml
//! [device]
//! type = "mock"
//! name = "Mock CRL-200S"
//!
//! [device.simulation]
//! map_file = "maps/example.yaml"
//! start_x = 1.5
//! start_y = 3.5
//! start_theta = 0.0
//! speed_factor = 1.0    # 2.0 = 2x speed
//! random_seed = 42      # 0 = random each run
//!
//! [network]
//! bind_address = "0.0.0.0:5555"
//! ```
//!
//! # Simulation Loop
//!
//! The simulation runs at 110Hz (matching CRL-200S hardware):
//!
//! ```text
//! Every ~9 ms of wall time (divided by `speed_factor`):
//! 1. Pass a new velocity command from shared state to the robot
//! 2. Step it, 1/110 s of simulated time a step, until it has caught up with the clock:
//!    wheels, physics, encoders, IMU, bumpers, cliffs, the beams due
//! 3. Publish each step's sensor_status via the streaming channel
//! 4. When a revolution completes (`scan_rate_hz`): update the lidar sensor group
//! ```
//!
//! # Speed Factor
//!
//! The `speed_factor` setting accelerates simulation time: `sensor_status` and the scans
//! come `speed_factor` times as often in wall time, stamped with the simulation's clock.
//! Benches that need more speed step a [`MockRobot`] directly.
//!
//! # Noise Models
//!
//! All sensors include configurable noise for realistic testing:
//!
//! - **Lidar**: Range stddev, angle stddev, miss rate, quality decay
//! - **Encoders**: Wheel slip (multiplicative), quantization noise
//! - **IMU**: Per-axis stddev, bias, gyro drift rate
//!
//! # Thread Model
//!
//! ```text
//! ┌─────────────────┐
//! │   Main Thread   │
//! │  (initialize)   │
//! └────────┬────────┘
//!          │ spawns
//!          ▼
//! ┌─────────────────┐     ┌─────────────────┐
//! │ Simulation Loop │────▶│ Streaming Chan  │
//! │  (mock-sim)     │     │ (sensor_status) │
//! └─────────────────┘     └─────────────────┘
//! ```
//!
//! # Module Structure
//!
//! - [`config`]: Configuration structures for all simulation parameters
//! - [`physics`]: Differential drive kinematics and collision handling
//! - [`lidar_sim`]: Ray-casting lidar simulation
//! - [`imu_sim`]: IMU data generation with noise
//! - [`encoder_sim`]: Wheel encoder simulation with slip
//! - [`sensor_sim`]: Bumper, cliff, and binary sensor simulation
//! - [`map_loader`]: PGM + YAML map loading
//! - [`noise`]: Configurable noise generator

pub mod config;
pub mod encoder_sim;
pub mod imu_sim;
pub mod lidar_sim;
pub mod map_loader;
pub mod noise;
pub mod physics;
pub mod robot;
pub mod sensor_sim;

pub use robot::MockRobot;

use crate::config::DeviceConfig;
use crate::core::driver::{DeviceDriver, DriverInitResult};
use crate::core::types::{Command, SensorGroupData, SensorValue};
use crate::error::{Error, Result};

use config::SimulationConfig;
use map_loader::SimulationMap;
use sensor_sim::ActuatorState;

use crossbeam_channel::Sender;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// Component state for tracking enable/disable commands
/// Mirrors CRL-200S behavior where lidar and motors must be explicitly enabled
pub struct ComponentState {
    /// Whether lidar motor is enabled (must be enabled to get scan data)
    pub lidar_enabled: AtomicBool,
    /// Whether wheel motors are enabled (must be enabled for drive commands to work)
    pub wheel_motor_enabled: AtomicBool,
}

impl ComponentState {
    fn new() -> Self {
        Self {
            lidar_enabled: AtomicBool::new(false),
            wheel_motor_enabled: AtomicBool::new(false),
        }
    }
}

/// Shared state for thread communication
struct SharedState {
    /// A drive command (m/s, rad/s) not yet passed to the robot. Only new commands reach
    /// it, as with the real driver, whose bumper interlock zeroes the command until the
    /// client sends another.
    drive: Mutex<Option<(f32, f32)>>,
    shutdown: AtomicBool,
    components: ComponentState,
}

impl SharedState {
    fn new() -> Self {
        Self {
            drive: Mutex::new(None),
            shutdown: AtomicBool::new(false),
            components: ComponentState::new(),
        }
    }

    /// Queues a drive command over any not yet taken.
    fn configure_drive(&self, linear: f32, angular: f32) {
        if let Ok(mut drive) = self.drive.lock() {
            *drive = Some((linear, angular));
        }
    }
}

/// Mock device driver for simulating CRL-200S hardware
pub struct MockDriver {
    config: SimulationConfig,
    shared_state: Arc<SharedState>,
    simulation_handle: Option<JoinHandle<()>>,
    actuator_state: Arc<Mutex<ActuatorState>>,
}

impl MockDriver {
    /// Create a new mock driver from device configuration
    pub fn new(device_config: DeviceConfig) -> Result<Self> {
        let sim_config = device_config.simulation.ok_or_else(|| {
            Error::Config("Mock device requires simulation configuration".to_string())
        })?;

        Ok(Self {
            config: sim_config,
            shared_state: Arc::new(SharedState::new()),
            simulation_handle: None,
            actuator_state: Arc::new(Mutex::new(ActuatorState::new())),
        })
    }

    fn shutdown_all(&mut self) {
        self.shared_state.shutdown.store(true, Ordering::Relaxed);

        if let Some(handle) = self.simulation_handle.take() {
            let _ = handle.join();
        }
    }
}

impl DeviceDriver for MockDriver {
    fn initialize(&mut self) -> Result<DriverInitResult> {
        log::info!("Initializing mock device driver");

        // Load map
        let map = SimulationMap::load(&self.config.map_file)?;
        log::info!(
            "Loaded map: {}x{} pixels, resolution: {} m/px",
            map.width(),
            map.height(),
            map.resolution()
        );

        // Groups are stamped with the simulation's clock from now on: it runs
        // `speed_factor` times as fast as the wall clock, and consumers integrate the
        // odometry over the stamps.
        let epoch_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_micros() as u64);
        let speed_factor = self.config.speed_factor;
        if !(speed_factor.is_finite() && speed_factor > 0.0) {
            return Err(Error::Config(format!(
                "speed_factor {speed_factor} must be > 0"
            )));
        }
        let robot = MockRobot::new(self.config.clone(), map, epoch_us)?;
        let kinematics = robot.kinematics();

        // Create sensor groups
        let sensor_status = Arc::new(Mutex::new(SensorGroupData::new("sensor_status")));
        let lidar_data = Arc::new(Mutex::new(SensorGroupData::new("lidar")));
        let version_data = Arc::new(Mutex::new(SensorGroupData::new("device_version")));

        // Set version info
        {
            let mut ver = version_data
                .lock()
                .map_err(|e| Error::MutexPoisoned(format!("version_data lock failed: {}", e)))?;
            ver.set(
                "version_string",
                SensorValue::String("MockDevice v1.0".to_string()),
            );
            ver.set("version_code", SensorValue::I32(100));
            ver.touch();
        }

        // Create streaming channel for sensor_status
        let (stream_tx, stream_rx) = crate::core::types::create_stream_channel();

        // Clone references for simulation thread
        let shared = Arc::clone(&self.shared_state);
        let sensor_status_clone = Arc::clone(&sensor_status);
        let lidar_data_clone = Arc::clone(&lidar_data);

        // Spawn simulation thread
        let handle = thread::Builder::new()
            .name("mock-simulation".to_string())
            .spawn(move || {
                simulation_loop(
                    speed_factor,
                    robot,
                    shared,
                    sensor_status_clone,
                    lidar_data_clone,
                    stream_tx,
                );
            })
            .map_err(|e| Error::Other(format!("Failed to spawn simulation thread: {}", e)))?;

        self.simulation_handle = Some(handle);

        log::info!("Mock device driver initialized");

        // Build result
        let mut sensor_data = HashMap::new();
        sensor_data.insert("sensor_status".to_string(), sensor_status);
        sensor_data.insert("lidar".to_string(), lidar_data);
        sensor_data.insert("device_version".to_string(), version_data);
        sensor_data.insert(
            "kinematics".to_string(),
            Arc::new(Mutex::new(kinematics.to_sensor_group())),
        );

        let mut stream_receivers = HashMap::new();
        stream_receivers.insert("sensor_status".to_string(), stream_rx);

        Ok(DriverInitResult {
            sensor_data,
            stream_receivers,
        })
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        match cmd {
            Command::ComponentControl { id, action } => {
                match id.as_str() {
                    "drive" => {
                        // Handle drive commands (matches CRL-200S behavior)
                        // - Enable: enables wheel motors (required before Configure works)
                        // - Configure: sets velocity (only if wheel motors enabled)
                        // - Disable: stops and disables wheel motors
                        match &action {
                            crate::core::types::ComponentAction::Enable { .. } => {
                                self.shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .store(true, Ordering::Relaxed);
                                log::debug!("Drive enabled (wheel motors on)");
                            }
                            crate::core::types::ComponentAction::Configure { config } => {
                                // As the CRL-200S driver: both, or nothing is stored.
                                let (
                                    Some(SensorValue::F32(linear)),
                                    Some(SensorValue::F32(angular)),
                                ) = (config.get("linear"), config.get("angular"))
                                else {
                                    return Err(Error::InvalidParameter(
                                        "drive Configure needs linear and angular (f32)".into(),
                                    ));
                                };
                                if !self
                                    .shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .load(Ordering::Relaxed)
                                {
                                    log::warn!(
                                        "Drive command ignored: wheel motors not enabled (send Enable first)"
                                    );
                                    return Ok(());
                                }
                                log::debug!("Drive command: linear={linear}, angular={angular}");
                                self.shared_state.configure_drive(*linear, *angular);
                            }
                            crate::core::types::ComponentAction::Disable { .. } => {
                                self.shared_state
                                    .components
                                    .wheel_motor_enabled
                                    .store(false, Ordering::Relaxed);
                                self.shared_state.configure_drive(0.0, 0.0);
                                log::debug!("Drive disabled (wheel motors off)");
                            }
                            _ => {}
                        }
                    }
                    "lidar" => {
                        // Handle lidar enable/disable
                        match &action {
                            crate::core::types::ComponentAction::Enable { .. } => {
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(true, Ordering::Relaxed);
                                log::debug!("Lidar enabled");
                            }
                            crate::core::types::ComponentAction::Disable { .. } => {
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(false, Ordering::Relaxed);
                                log::debug!("Lidar disabled");
                            }
                            crate::core::types::ComponentAction::Configure { config } => {
                                // Configure also enables lidar (like CRL-200S)
                                self.shared_state
                                    .components
                                    .lidar_enabled
                                    .store(true, Ordering::Relaxed);
                                if let Some(SensorValue::U8(pwm)) = config.get("pwm") {
                                    log::debug!("Lidar enabled (PWM={}%)", pwm);
                                } else {
                                    log::debug!("Lidar enabled via configure");
                                }
                            }
                            _ => {}
                        }
                    }
                    _ => {
                        // Handle other actuator commands (vacuum, brushes, etc.)
                        if let Ok(mut state) = self.actuator_state.lock() {
                            state.handle_command(
                                &id,
                                &action,
                                self.config.actuators.log_state_changes,
                            );
                        }
                    }
                }
            }
            Command::Shutdown => {
                log::info!("Shutdown command received");
                self.shutdown_all();
            }
            Command::ProtocolSync => {
                // Nothing to do for mock
            }
        }
        Ok(())
    }
}

impl Drop for MockDriver {
    fn drop(&mut self) {
        self.shutdown_all();
    }
}

/// How far (simulated s) the loop may fall behind the clock before it gives the time up.
const MAX_LAG: f64 = 0.5;

/// Steps the robot until its clock reaches `due` (s), handing `publish` every step's
/// `sensor_status`. Returns whether a scan completed.
fn catch_up(
    robot: &mut MockRobot,
    due: f64,
    lidar_on: bool,
    mut publish: impl FnMut(&SensorGroupData),
) -> bool {
    let mut scanned = false;
    while robot.time() < due {
        scanned |= robot.step(lidar_on);
        publish(robot.status());
    }
    scanned
}

/// Steps the robot in (scaled) real time at the CRL-200S's 110 Hz and publishes its groups.
fn simulation_loop(
    speed_factor: f32,
    mut robot: MockRobot,
    shared: Arc<SharedState>,
    sensor_status: Arc<Mutex<SensorGroupData>>,
    lidar_data: Arc<Mutex<SensorGroupData>>,
    stream_tx: Sender<SensorGroupData>,
) {
    let interval = Duration::from_secs_f32(robot::STEP_DT / speed_factor);
    log::info!("Simulation loop started: speed_factor={speed_factor}, interval={interval:?}");
    let start = Instant::now();
    // Simulated time given up when the loop fell too far behind (stalled, or too slow).
    let mut skipped = 0.0;

    while !shared.shutdown.load(Ordering::Relaxed) {
        if let Some((linear, angular)) = shared.drive.lock().ok().and_then(|mut d| d.take()) {
            robot.drive(linear, angular);
        }
        let lidar_on = shared.components.lidar_enabled.load(Ordering::Relaxed);
        let mut due = start.elapsed().as_secs_f64() * f64::from(speed_factor) - skipped;
        if due - robot.time() > MAX_LAG {
            log::warn!(
                "Simulation {:.1} s behind: skipping ahead",
                due - robot.time()
            );
            skipped += due - robot.time();
            due = robot.time();
        }
        let scanned = catch_up(&mut robot, due, lidar_on, |status| {
            let _ = stream_tx.try_send(status.clone());
        });
        if let Ok(mut status) = sensor_status.lock() {
            status.clone_from(robot.status());
        }
        if scanned && let Ok(mut lidar) = lidar_data.lock() {
            lidar.clone_from(robot.scan());
        }
        thread::sleep(interval);
    }

    log::info!("Simulation loop terminated");
}

#[cfg(test)]
mod tests {
    use crate::config::Config;

    fn simulation(file: &str) -> super::SimulationConfig {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/").to_owned() + file;
        let config = Config::load(&path).unwrap_or_else(|e| panic!("{file}: {e}"));
        config.device.simulation.expect(file)
    }

    #[test]
    fn the_shipped_mock_configs_load() {
        simulation("mock.toml");
        // The measured values are where they belong.
        let v8 = simulation("mock_bench.toml");
        assert_eq!(v8.robot.response_delay, 0.08);
        assert_eq!(v8.lidar.sweep_start_angle, 0.2182);
        assert_eq!(v8.lidar.noise.range_stddev_per_m, 0.003);
        assert_eq!(v8.imu.gyro_scale, 5729.6);
        assert_eq!(v8.imu.gyro_noise.deadband, 100.0);
    }

    #[test]
    fn catching_up_publishes_every_sample() {
        let config = simulation("mock.toml");
        let map = super::SimulationMap::load(&config.map_file).expect("mock.toml's map");
        let mut robot = super::MockRobot::new(config, map, 0).expect("a valid config");
        let mut sequence = Vec::new();
        let scanned = super::catch_up(&mut robot, 1.0, true, |status| {
            sequence.push(status.sequence_number)
        });
        assert!(scanned);
        assert_eq!(sequence, (1..=110).collect::<Vec<u64>>());
    }

    #[test]
    fn a_drive_configure_needs_linear_and_angular() {
        use crate::core::driver::DeviceDriver;
        use crate::core::types::{Command, ComponentAction, SensorValue};
        let device = Config::load(concat!(env!("CARGO_MANIFEST_DIR"), "/mock.toml"))
            .expect("mock.toml")
            .device;
        let mut driver = super::MockDriver::new(device).expect("a mock driver");
        let configure = |keys: &[&str]| Command::ComponentControl {
            id: "drive".into(),
            action: ComponentAction::Configure {
                config: keys
                    .iter()
                    .map(|k| (k.to_string(), SensorValue::F32(0.1)))
                    .collect(),
            },
        };
        assert!(driver.send_command(configure(&["linear"])).is_err());
        assert!(
            driver
                .send_command(configure(&["linear", "angular"]))
                .is_ok()
        );
    }

    #[test]
    fn a_mistyped_key_is_refused() {
        let parse = basic_toml::from_str::<super::config::LidarConfig>;
        assert!(parse("sweep_start_angle = 0.2").is_ok());
        assert!(parse("sweep_start_angel = 0.2").is_err());
    }
}
