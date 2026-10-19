//! Exploration thread for autonomous mapping.
//!
//! Each loop it reads a snapshot of the SharedState, ticks the exploration
//! [`Session`] and carries out its command: through SangamIO (the MotionController),
//! or by handing a target to the navigation thread. The decisions are the session's;
//! this thread does the talking: turning the lidar and motors on, yielding to
//! navigation, publishing the status.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::exploration::ExplorationStrategy;
use crate::exploration::session::{Command, Inputs, Session};
use crate::io::motion_controller::{MotionConfig, SharedMotionController};
use crate::navigation::{NavState, NavTarget};
use crate::state::{ExplorationMode, SharedStateHandle};

/// Configuration for exploration thread.
#[derive(Debug, Clone)]
pub struct ExplorationConfig {
    /// Motion controller configuration (for velocity limits).
    pub motion: MotionConfig,
    /// Loop rate in Hz.
    pub loop_rate_hz: f32,
    /// Not leaving a `STUCK_RADIUS` circle for this long ends exploration with an error.
    pub stuck_after: Duration,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            motion: MotionConfig::default(),
            loop_rate_hz: 10.0,
            stuck_after: Duration::from_secs(180),
        }
    }
}

/// After a stop for a contact, before the next command: the motors' time to stop.
pub const HALT_PAUSE: Duration = Duration::from_millis(50);
/// After turning the lidar or the motors on: their controllers' time to settle.
pub const CONTROLLER_SETTLE: Duration = Duration::from_secs(3);

/// Handle to the exploration thread.
pub struct ExplorationThread {
    handle: JoinHandle<()>,
}

impl ExplorationThread {
    /// Spawn the exploration thread.
    ///
    /// # Arguments
    ///
    /// - `config`: Thread configuration.
    /// - `strategy`: What to explore with (`[exploration] strategy`).
    /// - `shared_state`: Handle to shared state.
    /// - `motion_controller`: Shared motion controller (shared with navigation thread).
    /// - `running`: Atomic flag for shutdown.
    pub fn spawn(
        config: ExplorationConfig,
        strategy: Box<dyn ExplorationStrategy>,
        shared_state: SharedStateHandle,
        motion_controller: SharedMotionController,
        running: Arc<AtomicBool>,
    ) -> Self {
        let handle = thread::Builder::new()
            .name("exploration".into())
            .spawn(move || {
                run_exploration_loop(config, strategy, shared_state, motion_controller, running);
            })
            .expect("Failed to spawn exploration thread");

        Self { handle }
    }

    /// Wait for the thread to finish.
    pub fn join(self) {
        if let Err(e) = self.handle.join() {
            log::error!("Exploration thread panicked: {:?}", e);
        }
    }
}

/// Main exploration loop.
fn run_exploration_loop(
    config: ExplorationConfig,
    strategy: Box<dyn ExplorationStrategy>,
    shared_state: SharedStateHandle,
    motion_controller: SharedMotionController,
    running: Arc<AtomicBool>,
) {
    log::info!("Exploration thread started ({})", strategy.name());

    // Check if motion controller has an address configured
    let motion_available = {
        let mc = motion_controller.lock().unwrap();
        mc.has_address()
    };

    let loop_duration = Duration::from_secs_f32(1.0 / config.loop_rate_hz);
    let mut session = Session::new(strategy, config.motion.clone(), config.stuck_after);
    let mut motion_enabled = false;
    let mut lidar_enabled = false;
    let mut was_exploring = false;

    while running.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        let (exploration_mode, inputs, nav_has_targets, nav_state, current_target_pos) = {
            let state = match shared_state.read() {
                Ok(s) => s,
                Err(e) => {
                    log::warn!("Failed to read shared state: {}", e);
                    thread::sleep(loop_duration);
                    continue;
                }
            };
            let navigation = &state.navigation_state;
            (
                state.exploration_state.mode,
                Inputs::snapshot(&state),
                navigation.has_targets(),
                navigation.nav_state,
                navigation
                    .current_target()
                    .map(|t| (t.position.x, t.position.y)),
            )
        };

        // Every exploration session starts afresh (also after one completed); one that
        // ends leaves the robot standing.
        let exploring = exploration_mode == ExplorationMode::Exploring;
        if exploring && !was_exploring {
            session.start(inputs.scan.timestamp_us);
        }
        if !exploring
            && was_exploring
            && motion_enabled
            && let Err(e) = motion_controller.lock().unwrap().set_velocity(0.0, 0.0)
        {
            log::warn!("Failed to stop after exploring: {}", e);
        }
        was_exploring = exploring;

        // Handle navigation failure - clear target and notify strategy
        if nav_state == NavState::Failed && nav_has_targets {
            log::warn!("Navigation failed for frontier target, clearing and marking blocked");

            // Get and notify strategy about blocked target before clearing
            if let Some((tx, ty)) = current_target_pos {
                session.target_blocked(tx, ty);
            }

            // Clear the failed target from navigation state
            if let Ok(mut state) = shared_state.write() {
                state.navigation_state.clear_targets();
                state.exploration_state.status = "Frontier blocked, finding new target".to_string();
            }

            // Continue to next iteration - exploration will pick a new frontier
            thread::sleep(loop_duration);
            continue;
        }

        // Handle navigation success - notify strategy that target was reached
        // This happens when navigation completed but we still yielded last iteration
        if nav_state == NavState::TargetReached && !nav_has_targets {
            log::info!("Navigation completed successfully, notifying exploration strategy");

            // Notify strategy that target was reached (clears internal state)
            session.target_reached();

            // Clear the TargetReached state
            if let Ok(mut state) = shared_state.write() {
                state.navigation_state.nav_state = NavState::Idle;
                state.exploration_state.status = "Frontier reached, scanning...".to_string();
            }

            // Continue to next iteration - exploration will pick a new frontier
            thread::sleep(loop_duration);
            continue;
        }

        // Only explore if in exploration mode AND navigation is not active
        if !exploring || nav_has_targets {
            // Only disable motors/lidar if exploration mode changed to Disabled
            // When yielding to navigation (nav_has_targets), keep motors enabled!
            if exploration_mode == ExplorationMode::Disabled && (motion_enabled || lidar_enabled) {
                let mut mc = motion_controller.lock().unwrap();
                if motion_enabled {
                    if let Err(e) = mc.disable() {
                        log::warn!("Failed to disable motors: {}", e);
                    }
                    motion_enabled = false;
                }
                if lidar_enabled {
                    if let Err(e) = mc.disable_lidar() {
                        log::warn!("Failed to disable lidar: {}", e);
                    }
                    lidar_enabled = false;
                }
            }

            // If yielding to navigation, update status (but keep motors enabled!)
            if nav_has_targets
                && exploring
                && let Ok(mut state) = shared_state.write()
            {
                state.exploration_state.status = "Paused: Navigation active".to_string();
            }

            thread::sleep(loop_duration);
            continue;
        }

        // Enable lidar first (needed for SLAM to build map)
        if !lidar_enabled && motion_available {
            let result = motion_controller.lock().unwrap().enable_lidar();
            match result {
                Ok(()) => {
                    lidar_enabled = true;
                    log::debug!("Lidar enabled, waiting {CONTROLLER_SETTLE:?} to settle");
                    thread::sleep(CONTROLLER_SETTLE);
                    log::debug!("Lidar controller settled");
                }
                Err(e) => {
                    // Only log at debug level - MotionController will retry automatically
                    log::debug!("Lidar enable pending: {}", e);
                }
            }
        }

        let now = Instant::now();
        let tick = session.tick(&inputs, now);

        if tick.halt {
            if let Err(e) = motion_controller.lock().unwrap().emergency_stop() {
                log::warn!("Emergency stop failed: {}", e);
            }
            thread::sleep(HALT_PAUSE);
        }

        // Motors go on once SLAM's scans flow; the command waits for the next tick.
        if !motion_enabled && session.scans_flowing(now) && motion_available {
            match motion_controller.lock().unwrap().enable() {
                Ok(()) => {
                    motion_enabled = true;
                    log::debug!("Motors enabled, waiting {CONTROLLER_SETTLE:?} to settle");
                    thread::sleep(CONTROLLER_SETTLE);
                    log::debug!("Motors settled, ready for exploration");
                    continue;
                }
                // Only log at debug level - MotionController will retry automatically
                Err(e) => log::debug!("Motor enable pending: {}", e),
            }
        }

        if let Ok(mut state) = shared_state.write() {
            let exploration = &mut state.exploration_state;
            exploration.frontiers_remaining = session.strategy().frontier_count();
            exploration.completion_percent = session.strategy().completion_percent();
            if let Some(map) = &inputs.map {
                exploration.explored_area_m2 = map.explored_area_m2;
            }
            exploration.status = tick.status.clone();
        }

        match tick.command {
            Command::Drive { linear, angular } => {
                if motion_enabled
                    && let Err(e) = motion_controller
                        .lock()
                        .unwrap()
                        .set_velocity(linear, angular)
                {
                    log::warn!("Failed to set velocity: {}", e);
                }
            }

            Command::NavigateTo(target) => {
                log::info!("Navigating to ({:.2}, {:.2})", target.x, target.y);
                // The navigation thread plans the path and drives it.
                if let Ok(mut state) = shared_state.write() {
                    // Clear any existing targets first to prevent stale targets
                    state.navigation_state.clear_targets();
                    state
                        .navigation_state
                        .push_target(NavTarget::frontier(target.x, target.y));
                    state.exploration_state.current_target = Some(target);
                }

                // Set zero velocity - navigation thread will take over control
                // DON'T use emergency_stop() as it disables motors
                if let Err(e) = motion_controller.lock().unwrap().set_velocity(0.0, 0.0) {
                    log::warn!("Failed to stop before navigation handoff: {}", e);
                }
            }

            Command::Finish { completed } => {
                if completed {
                    log::info!("Exploration complete - all areas mapped!");
                } else {
                    log::error!("{}", tick.status);
                }
                // Ends like a completed sweep (the lidar keeps spinning, mapping goes on);
                // only the status tells a stuck one apart (there is no failed mode).
                if let Err(e) = motion_controller.lock().unwrap().disable() {
                    log::warn!("Failed to disable motors: {}", e);
                }
                motion_enabled = false;
                if let Ok(mut state) = shared_state.write() {
                    state.exploration_state.mode = ExplorationMode::Complete;
                    state.exploration_state.current_target = None;
                    if completed {
                        state.exploration_state.completion_percent = 100.0;
                    }
                }
            }
        }

        // Maintain loop rate
        let elapsed = loop_start.elapsed();
        if elapsed < loop_duration {
            thread::sleep(loop_duration - elapsed);
        }
    }

    // Cleanup: disable motors and lidar
    if motion_available {
        let mut mc = motion_controller.lock().unwrap();
        if let Err(e) = mc.disable() {
            log::warn!("Failed to disable motors on shutdown: {}", e);
        }
        if let Err(e) = mc.disable_lidar() {
            log::warn!("Failed to disable lidar on shutdown: {}", e);
        }
    }

    log::info!("Exploration thread stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exploration_config_default() {
        let config = ExplorationConfig::default();
        assert_eq!(config.loop_rate_hz, 10.0);
    }
}
