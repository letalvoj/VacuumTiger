//! One sweep in one scene, closed loop through the software that runs on the robot:
//!
//! SangamIO's mock robot (stepped at its 110 Hz) → SangamIO's wire format → dhruva-slam's
//! decode → its SLAM → the exploration session, ticked at the exploration thread's rate on
//! the simulated clock → the drive command back to the mock.
//!
//! The bench plays the exploration thread's part as it does it: the lidar and the motors
//! get `CONTROLLER_SETTLE` to start, a contact stops the robot for `HALT_PAUSE` before the
//! next command, commands are limited as the motion controller limits them. Navigation is
//! not simulated: a strategy asking for it ends the run.
//!
//! What the robot has and the bench does not: SLAM takes no time here (on the R16 a scan
//! takes tens of ms, and the pose a tick reads is older); ticks fall on the mock's steps;
//! the command reaches the wheels after the mock's response delay but not over TCP; the
//! lidar spins from the start instead of booting for ~30 s; nothing else (STOP, teleop)
//! writes the shared state.

use std::time::{Duration, Instant};

use dhruva_slam::config::{Config, build_exploration, build_slam_thread_config};
use dhruva_slam::exploration::session::{Command, Inputs, Session};
use dhruva_slam::io::sangam_udp_receiver::Sample;
use dhruva_slam::state::{SharedStateHandle, create_shared_state};
use dhruva_slam::threads::{CONTROLLER_SETTLE, HALT_PAUSE, SlamContext};
use sangamio::core::types::SensorGroupData;
use sangamio::devices::mock::MockRobot;
use sangamio::devices::mock::config::SimulationConfig;
use sangamio::devices::mock::robot::STEP_DT;
use sangamio::streaming::wire::Serializer;

use crate::scene::{Point, Scene};

/// The groups' clock starts here (µs): any fixed time, so runs replay exactly.
const EPOCH_US: u64 = 1_700_000_000_000_000;

/// What happened in a run.
#[derive(Debug, Clone, Default)]
pub struct Run {
    pub completed: bool,
    /// Why it ended early (stuck, out of time), if it did.
    pub ended: Option<String>,
    /// The true pose (x, y, theta) every step, `dt` apart.
    pub path: Vec<(f32, f32, f32)>,
    pub dt: f32,
    /// Where SLAM believed the robot was every tick, in the scene's frame (SLAM's starts
    /// where the robot does).
    pub belief: Vec<(f32, f32, f32)>,
    /// Where the bumper was pressed (a new press).
    pub bumps: Vec<Point>,
    /// The session's status every tick, with the speed it commanded.
    pub ticks: Vec<(String, f32)>,
}

impl Run {
    pub fn seconds(&self) -> f32 {
        self.path.len() as f32 * self.dt
    }
}

/// The mock and the SLAM it feeds.
struct Loop {
    robot: MockRobot,
    serializer: Serializer,
    slam: SlamContext,
    shared: SharedStateHandle,
    pressed: bool,
    run: Run,
}

impl Loop {
    /// Steps the robot until `until` (s, simulated), feeding SLAM every group as SangamIO
    /// would send it.
    fn advance(&mut self, until: f64) -> Result<(), String> {
        while self.robot.time() + 1e-9 < until {
            let scanned = self.robot.step(true);
            let status = self.wire(self.robot.status())?;
            self.feed(&status)?;
            if scanned {
                let scan = self.wire(self.robot.scan())?;
                self.feed(&scan)?;
            }
            self.run.path.push(self.robot.pose());
        }
        Ok(())
    }

    /// A group as SangamIO sends it.
    fn wire(&self, group: &SensorGroupData) -> Result<Vec<u8>, String> {
        (self.serializer.serialize_sensor_group(group)).map_err(|e| e.to_string())
    }

    /// One message through dhruva-slam's decode into its SLAM.
    fn feed(&mut self, bytes: &[u8]) -> Result<(), String> {
        match Sample::decode(bytes).map_err(|e| e.to_string())? {
            Some(Sample::Status(status)) => {
                let pressed = status.bumper_left || status.bumper_right;
                if pressed && !self.pressed {
                    let (x, y, _) = self.robot.pose();
                    self.run.bumps.push((x, y));
                }
                self.pressed = pressed;
                self.slam.process_udp_sensor_status(&status, &self.shared);
            }
            Some(Sample::Lidar(scan)) => self.slam.process_udp_lidar(&scan, &self.shared),
            None => {}
        }
        Ok(())
    }
}

/// Sweeps `scene` with dhruva-slam configured by `dhruva` and the robot simulated by
/// `sim` (`seed` for its noise). Only `wall_follow` runs on the simulated clock (the
/// frontier strategy reads the wall clock), so only it is accepted.
pub fn run(
    scene: &Scene,
    dhruva: &Config,
    mut sim: SimulationConfig,
    seed: u64,
) -> Result<Run, String> {
    if dhruva.exploration.strategy != "wall_follow" {
        return Err(format!(
            "exploration strategy {:?}: the bench runs wall_follow only",
            dhruva.exploration.strategy
        ));
    }
    let start = scene.start;
    (sim.start_x, sim.start_y, sim.start_theta) = start;
    sim.random_seed = seed;
    let (exploration, strategy) = build_exploration(dhruva)?;
    let mut session = Session::new(
        strategy,
        exploration.motion.clone(),
        exploration.stuck_after,
    );
    let mut bench = Loop {
        robot: MockRobot::new(sim, scene.map(), EPOCH_US).map_err(|e| e.to_string())?,
        serializer: Serializer::new(),
        slam: SlamContext::mapping(&build_slam_thread_config(dhruva)),
        shared: create_shared_state(),
        pressed: false,
        run: Run {
            dt: STEP_DT,
            ..Run::default()
        },
    };
    let period = 1.0 / f64::from(exploration.loop_rate_hz);
    let clock = Instant::now();
    let now = |t: f64| clock + Duration::from_secs_f64(t);
    let snapshot = |shared: &SharedStateHandle| {
        Inputs::snapshot(&shared.read().expect("shared state poisoned"))
    };
    let (s, c) = start.2.sin_cos();
    let in_scene = |x: f32, y: f32, theta: f32| {
        (
            start.0 + c * x - s * y,
            start.1 + s * x + c * y,
            start.2 + theta,
        )
    };

    // The lidar settles before the first tick; the session starts then.
    bench.advance(CONTROLLER_SETTLE.as_secs_f64())?;
    session.start(snapshot(&bench.shared).scan.timestamp_us);
    let mut motors_on = false;
    while bench.robot.time() < f64::from(scene.seconds) {
        let started = bench.robot.time();
        let inputs = snapshot(&bench.shared);
        let tick = session.tick(&inputs, now(started));
        let pose = inputs.pose;
        bench.run.belief.push(in_scene(pose.x, pose.y, pose.theta));
        if tick.halt {
            bench.robot.drive(0.0, 0.0);
            bench.advance(started + HALT_PAUSE.as_secs_f64())?;
        }
        if !motors_on && session.scans_flowing(now(started)) {
            motors_on = true;
            bench.advance(bench.robot.time() + CONTROLLER_SETTLE.as_secs_f64())?;
            continue;
        }
        let mut speed = 0.0;
        match tick.command {
            Command::Drive { linear, angular } if motors_on => {
                let (linear, angular) = exploration.motion.limit(linear, angular);
                bench.robot.drive(linear, angular);
                speed = linear;
            }
            Command::Drive { .. } => {}
            Command::Finish { completed } => {
                bench.run.completed = completed;
                if !completed {
                    bench.run.ended = Some(tick.status);
                }
                return Ok(bench.run);
            }
            Command::NavigateTo(target) => {
                bench.run.ended = Some(format!("asked to navigate to {target:?}"));
                return Ok(bench.run);
            }
        }
        bench.run.ticks.push((tick.status, speed));
        bench.advance(started + period)?;
    }
    bench.run.ended = Some("out of time".into());
    Ok(bench.run)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CONFIG, SIM, scene, settings};

    /// A run in the box, and the exploration's tick period (s).
    fn box_run(seconds: f32) -> (Run, f32) {
        let (config, sim) = settings(CONFIG.as_ref(), SIM.as_ref()).unwrap();
        let mut scene = (scene::scenes().into_iter())
            .find(|s| s.name == "box")
            .unwrap();
        scene.seconds = seconds;
        let period = 1.0 / config.exploration.loop_rate_hz;
        (run(&scene, &config, sim, 1).unwrap(), period)
    }

    #[test]
    fn a_seed_replays_exactly_and_slam_tracks_the_true_pose() {
        let ((a, period), (b, _)) = (box_run(40.0), box_run(40.0));
        assert_eq!(
            a.path, b.path,
            "the climb compares runs: a seed must replay"
        );
        let travelled: f32 = (a.path.windows(2))
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum();
        assert!(travelled > 2.0, "it swept: {travelled:.2} m");
        // SLAM's belief at its last tick, a period before the end, against the truth then.
        let (bx, by, bt) = *a.belief.last().unwrap();
        let (tx, ty, tt) = a.path[a.path.len() - (period / a.dt).round() as usize];
        let off = (bx - tx).hypot(by - ty);
        let turn = dhruva_slam::core::math::angle_diff(bt, tt).abs();
        assert!(
            off < 0.05 && turn < 0.05,
            "belief off by {off:.3} m, {turn:.3} rad"
        );
    }
}
