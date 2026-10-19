//! The simulated CRL-200S, advanced one sensor sample at a time by its caller.
//!
//! [`MockDriver`](super::MockDriver) steps it in (scaled) real time and serves its groups like
//! any driver; offline benches step it as fast as they like and read the same groups.

use super::config::SimulationConfig;
use super::encoder_sim::EncoderSimulator;
use super::imu_sim::{self, ImuSimulator};
use super::lidar_sim::{LidarSimulator, Pose};
use super::map_loader::SimulationMap;
use super::noise::NoiseGenerator;
use super::physics::PhysicsState;
use super::sensor_sim::{BumperSimulator, CliffSimulator};
use crate::core::drive::bumper_stops;
use crate::core::types::{Kinematics, SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use std::collections::VecDeque;

/// One step (s): the CRL-200S publishes `sensor_status` at 110 Hz. Commands reach the
/// wheels on a step boundary, so the response delay is rounded up to a step.
pub const STEP_DT: f32 = 1.0 / STEP_HZ as f32;
const STEP_HZ: f64 = 110.0;

const GYRO_KEYS: [&str; 3] = ["gyro_x", "gyro_y", "gyro_z"];
const ACCEL_KEYS: [&str; 3] = ["accel_x", "accel_y", "accel_z"];
const TILT_KEYS: [&str; 3] = ["tilt_x", "tilt_y", "tilt_z"];
const CLIFFS: [(&str, &str); 4] = [
    ("left_side", "cliff_left_side"),
    ("left_front", "cliff_left_front"),
    ("right_front", "cliff_right_front"),
    ("right_side", "cliff_right_side"),
];

/// The robot in its map: physics and every simulated sensor.
pub struct MockRobot {
    config: SimulationConfig,
    map: SimulationMap,
    physics: PhysicsState,
    lidar: LidarSimulator,
    imu: ImuSimulator,
    encoders: EncoderSimulator,
    bumpers: BumperSimulator,
    cliffs: CliffSimulator,
    /// The drive command (m/s, rad/s); commands on their way to the wheels (due time,
    /// command); the speed the wheels head for and their speed now (m/s, rad/s).
    command: (f32, f32),
    pending: VecDeque<(f64, (f32, f32))>,
    target: (f32, f32),
    speed: (f32, f32),
    /// Steps since the start, and the wall time it started at (µs since the Unix epoch):
    /// groups are stamped `epoch_us` + simulated time.
    steps: u64,
    epoch_us: u64,
    /// Either front bumper pressed, as of the last step.
    bumped: bool,
    status: SensorGroupData,
    scan: SensorGroupData,
}

impl MockRobot {
    /// The robot at the configured start pose in `map`, its clock starting at `epoch_us`.
    /// Errors on a configuration it cannot simulate.
    pub fn new(config: SimulationConfig, map: SimulationMap, epoch_us: u64) -> Result<Self> {
        let (lidar, robot) = (&config.lidar, &config.robot);
        let turn = lidar.scan_rate_hz * STEP_DT;
        if !(turn > 0.0 && turn <= 1.0 && lidar.num_rays > 0) {
            return Err(Error::Config(format!(
                "lidar: scan_rate_hz in (0, {STEP_HZ}] and num_rays > 0"
            )));
        }
        if !(robot.response_delay >= 0.0 && robot.response_time_constant >= 0.0) {
            return Err(Error::Config(
                "robot: response_delay and response_time_constant >= 0".into(),
            ));
        }
        let mut noise = NoiseGenerator::new(config.random_seed);
        let robot = &config.robot;
        Ok(Self {
            physics: PhysicsState::new(config.start_x, config.start_y, config.start_theta, robot),
            lidar: LidarSimulator::new(&config.lidar, noise.split()),
            imu: ImuSimulator::new(&config.imu, noise.split()),
            encoders: EncoderSimulator::new(&config.encoder, robot.ticks_per_meter, noise.split()),
            bumpers: BumperSimulator::new(&config.bumpers),
            cliffs: CliffSimulator::new(&config.cliffs),
            command: (0.0, 0.0),
            pending: VecDeque::new(),
            target: (0.0, 0.0),
            speed: (0.0, 0.0),
            steps: 0,
            epoch_us,
            bumped: false,
            status: SensorGroupData::new("sensor_status"),
            scan: SensorGroupData::new("lidar"),
            map,
            config,
        })
    }

    /// The drive geometry and sensor scales, as the `kinematics` group publishes them.
    pub fn kinematics(&self) -> Kinematics {
        Kinematics {
            wheel_m_per_tick: 1.0 / self.config.robot.ticks_per_meter,
            wheel_track_m: self.config.robot.wheel_base,
            wheel_tick_bits: 16,
            gyro_rad_per_lsb: 1.0 / self.config.imu.gyro_scale,
            accel_mps2_per_lsb: Some(9.81 / imu_sim::ACCEL_SCALE),
        }
    }

    /// Sets the drive command (m/s, rad/s), as SangamIO's `drive` Configure does. The
    /// wheels follow it after the configured response delay.
    pub fn drive(&mut self, linear: f32, angular: f32) {
        if (linear, angular) != self.command {
            self.command = (linear, angular);
            let due = self.time() + f64::from(self.config.robot.response_delay);
            self.pending.push_back((due, self.command));
        }
    }

    /// Ground truth: (x, y, theta) in the map frame.
    pub fn pose(&self) -> Pose {
        (self.physics.x(), self.physics.y(), self.physics.theta())
    }

    /// Simulated time since the start (s).
    pub fn time(&self) -> f64 {
        self.steps as f64 / STEP_HZ
    }

    /// The `sensor_status` group of the last step.
    pub fn status(&self) -> &SensorGroupData {
        &self.status
    }

    /// The `lidar` group of the last scan: the points, and the ground truth pose when the
    /// revolution completed (`gt_x`, `gt_y`, `gt_theta`, for scoring SLAM offline).
    pub fn scan(&self) -> &SensorGroupData {
        &self.scan
    }

    /// Advances the robot one [`STEP_DT`], updating [`status`](Self::status). Returns
    /// whether a scan completed (with `lidar_on`), updating [`scan`](Self::scan).
    pub fn step(&mut self, lidar_on: bool) -> bool {
        let dt = STEP_DT;
        // SangamIO's heartbeat zeroes the command itself: it stays stopped until the
        // client sends a new one.
        if bumper_stops(self.bumped, self.command.0) {
            self.drive(0.0, 0.0);
        }
        let (linear, angular) = self.wheels(dt);
        let from = self.pose();
        let robot = &self.config.robot;
        if self.physics.update(dt, linear, angular, &self.map, robot)
            && self.config.log_level != "minimal"
        {
            log::debug!(
                "Collision at ({:.3}, {:.3})",
                self.physics.x(),
                self.physics.y()
            );
        }
        let (left_vel, right_vel) =
            self.physics
                .wheel_velocities(linear, angular, robot.wheel_base);
        let (wheel_left, wheel_right) = self.encoders.update(left_vel, right_vel, dt);
        let imu = self.imu.generate(angular, linear, dt);
        let to = self.pose();
        let (x, y, theta) = to;
        let (bumper_left, bumper_right) =
            self.bumpers
                .check(&self.map, x, y, theta, robot.robot_radius);
        let cliffs = self.cliffs.check(&self.map, x, y, theta);
        self.bumped = bumper_left || bumper_right;
        self.steps += 1;
        let stamp = self.epoch_us + (self.time() * 1e6).round() as u64;
        self.fill_status(
            wheel_left,
            wheel_right,
            &imu,
            bumper_left,
            bumper_right,
            &cliffs,
        );
        self.status.stamp(stamp);
        if !lidar_on {
            self.lidar.stop();
            return false;
        }
        let Some(points) = self.lidar.sweep(&self.map, from, to, dt) else {
            return false;
        };
        self.scan.set("scan", SensorValue::PointCloud2D(points));
        self.scan.set("gt_x", SensorValue::F32(x));
        self.scan.set("gt_y", SensorValue::F32(y));
        self.scan.set("gt_theta", SensorValue::F32(theta));
        self.scan.stamp(stamp);
        true
    }

    /// The wheels' mean speed over the next `dt` (m/s, rad/s): the due command, lagged
    /// exponentially (exact for any `dt`).
    fn wheels(&mut self, dt: f32) -> (f32, f32) {
        let robot = &self.config.robot;
        while let Some(&(due, (linear, angular))) = self.pending.front()
            && due <= self.time()
        {
            self.pending.pop_front();
            self.target = (robot.response_gain * linear, robot.response_gain * angular);
        }
        let tau = robot.response_time_constant;
        // Share of the gap closed by the end of the step, and on average over it.
        let (end, mean) = if tau > 0.0 {
            let k = 1.0 - (-dt / tau).exp();
            (k, 1.0 - tau * k / dt)
        } else {
            (1.0, 1.0)
        };
        let (target, speed) = (self.target, self.speed);
        self.speed = (
            speed.0 + end * (target.0 - speed.0),
            speed.1 + end * (target.1 - speed.1),
        );
        (
            speed.0 + mean * (target.0 - speed.0),
            speed.1 + mean * (target.1 - speed.1),
        )
    }

    fn fill_status(
        &mut self,
        wheel_left: u16,
        wheel_right: u16,
        imu: &imu_sim::ImuReading,
        bumper_left: bool,
        bumper_right: bool,
        cliffs: &std::collections::HashMap<String, bool>,
    ) {
        let sensors = &self.config.sensors;
        let s = &mut self.status;
        s.set("wheel_left", SensorValue::U16(wheel_left));
        s.set("wheel_right", SensorValue::U16(wheel_right));
        for i in 0..3 {
            s.set(GYRO_KEYS[i], SensorValue::I16(imu.gyro[i]));
            s.set(ACCEL_KEYS[i], SensorValue::I16(imu.accel[i]));
            s.set(TILT_KEYS[i], SensorValue::I16(imu.tilt[i]));
        }
        s.set("bumper_left", SensorValue::Bool(bumper_left));
        s.set("bumper_right", SensorValue::Bool(bumper_right));
        // As the CRL-200S publishes them: front zones only, no dock in the simulation.
        let zones = u8::from(bumper_left) << 2 | u8::from(bumper_right) << 1;
        s.set("bumper_zones", SensorValue::U8(zones));
        s.set("dock_ir", SensorValue::U8(0));
        for (name, key) in CLIFFS {
            let cliff = cliffs.get(name).copied().unwrap_or(false);
            s.set(key, SensorValue::Bool(cliff));
        }
        s.set("battery_voltage", SensorValue::F32(sensors.battery.voltage));
        s.set("battery_level", SensorValue::U8(sensors.battery.level));
        s.set(
            "is_charging",
            SensorValue::Bool(sensors.battery.is_charging),
        );
        s.set(
            "dustbox_attached",
            SensorValue::Bool(sensors.dustbox_attached),
        );
        s.set(
            "is_dock_connected",
            SensorValue::Bool(sensors.dock_connected),
        );
        s.set("start_button", SensorValue::U16(sensors.start_button));
        s.set("dock_button", SensorValue::U16(sensors.dock_button));
    }
}

#[cfg(test)]
mod tests {
    use super::super::config::RobotConfig;
    use super::*;

    /// A 10 x 10 m floor at 1 cm per cell, walled beyond x = 6, the robot at (5, 5) facing +x.
    fn robot_with(robot: RobotConfig) -> MockRobot {
        let img = image::GrayImage::from_fn(1000, 1000, |i, _| {
            image::Luma([if i >= 600 { 0 } else { 255 }])
        });
        let map = SimulationMap::from_image(img, 0.01, (0.0, 0.0));
        let config = SimulationConfig {
            start_x: 5.0,
            start_y: 5.0,
            random_seed: 1,
            robot,
            ..SimulationConfig::default()
        };
        MockRobot::new(config, map, 1_000_000).expect("a valid config")
    }

    fn robot() -> MockRobot {
        robot_with(RobotConfig::default())
    }

    /// As measured on the v8 (`mock_bench.toml`).
    fn v8_drive() -> RobotConfig {
        RobotConfig {
            response_delay: 0.08,
            response_time_constant: 0.12,
            response_gain: 0.9,
            ..RobotConfig::default()
        }
    }

    /// The points of the last scan.
    fn points(robot: &MockRobot) -> super::super::lidar_sim::Scan {
        match robot.scan().values.get("scan") {
            Some(SensorValue::PointCloud2D(points)) => points.clone(),
            _ => panic!("no scan"),
        }
    }

    /// Steps the robot `steps` times, the lidar off.
    fn run(robot: &mut MockRobot, steps: usize) {
        (0..steps).for_each(|_| _ = robot.step(false));
    }

    #[test]
    fn the_wheels_answer_late_and_lagged() {
        let mut robot = robot_with(v8_drive());
        robot.drive(0.2, 0.0);
        // 0.08 s of dead time ends on the 9th step boundary.
        run(&mut robot, 9);
        assert_eq!(robot.pose().0, 5.0, "not moving in the dead time");
        let t0 = 9.0 * STEP_DT;
        // 0.18 m/s approached with a 0.12 s time constant: x = v (t - tau (1 - e^(-t/tau))).
        let x = |t: f32| 5.0 + 0.18 * (t - 0.12 * (1.0 - (-t / 0.12).exp()));
        for steps in [5, 100] {
            run(&mut robot, steps);
            let t = robot.time() as f32 - t0;
            let (at, expected) = (robot.pose().0, x(t));
            assert!((at - expected).abs() < 1e-4, "{at} vs {expected} at {t}");
        }
    }

    #[test]
    fn forward_into_a_pressed_bumper_stops_backing_off_passes() {
        let mut robot = robot_with(v8_drive());
        robot.drive(0.2, 0.0);
        run(&mut robot, 1100);
        let pressed = robot.pose();
        assert!(
            (pressed.0 - (6.0 - 0.17)).abs() < 0.02,
            "at the wall: {pressed:?}"
        );
        // Forward with a turn: the interlock stops it all, before the wheels respond.
        robot.drive(0.1, 0.5);
        run(&mut robot, 55);
        let (x, _, theta) = robot.pose();
        assert!((x - pressed.0).abs() < 1e-3 && (theta - pressed.2).abs() < 0.01);
        robot.drive(-0.1, 0.0);
        run(&mut robot, 110);
        assert!(robot.pose().0 < pressed.0 - 0.04);
    }

    #[test]
    fn a_seed_replays_the_same_run() {
        let replay = || {
            let mut robot = robot_with(v8_drive());
            robot.drive(0.2, 0.3);
            let mut scans = Vec::new();
            for _ in 0..330 {
                if robot.step(true) {
                    scans.push(points(&robot));
                }
            }
            (scans, robot.pose())
        };
        assert_eq!(replay(), replay());
    }

    #[test]
    fn steps_drive_the_robot_and_scan_at_the_lidar_rate() {
        let mut robot = robot();
        robot.drive(0.2, 0.0);
        let mut scans = Vec::new();
        for _ in 0..110 {
            if robot.step(true) {
                scans.push(robot.scan().timestamp_us);
            }
        }
        let (x, y, _) = robot.pose();
        assert!(
            (x - 5.2).abs() < 2e-3 && (y - 5.0).abs() < 1e-3,
            "at ({x}, {y})"
        );
        assert_eq!(scans.len(), 5, "a second at 5 Hz");
        // Stamped with the simulated clock from the epoch.
        assert_eq!(robot.status().timestamp_us, 2_000_000);
        assert_eq!(scans[0], 1_200_000);
        let scan = robot.scan();
        assert!(
            ["scan", "gt_x", "gt_y", "gt_theta"]
                .iter()
                .all(|k| scan.values.contains_key(*k))
        );
    }
}
