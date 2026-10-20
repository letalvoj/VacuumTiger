//! Bumper, cliff, and actuator simulation
//!
//! Provides collision-based bumper detection and map-based cliff detection.

use super::config::{BumperZoneConfig, BumpersConfig, CliffsConfig};
use super::map_loader::SimulationMap;
use crate::core::types::ComponentAction;
use std::collections::HashMap;

/// Bumper simulator using angular zones
pub struct BumperSimulator {
    config: BumpersConfig,
}

impl BumperSimulator {
    /// Create new bumper simulator
    pub fn new(config: &BumpersConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Which bumpers (left, right) touch something: a zone is pressed when any point of
    /// its arc is occupied. `pushed` into something (the body was stopped), the shell gives
    /// all round, the arc `trigger_distance` out from the body; otherwise it is held only as
    /// far as it slides back, by `cos` of the bearing. The arc is sampled every half cell, so
    /// a thin chair leg anywhere along it presses it, as on the real shell.
    pub fn check(
        &self,
        map: &SimulationMap,
        x: f32,
        y: f32,
        theta: f32,
        robot_radius: f32,
        pushed: bool,
    ) -> (bool, bool) {
        let reach = robot_radius + self.config.trigger_distance;
        let pressed = |zone: &BumperZoneConfig| {
            let span = zone.end_angle - zone.start_angle;
            let samples = (span.abs() * reach / (map.resolution() / 2.0)).ceil() as usize;
            (0..=samples).any(|i| {
                let bearing = zone.start_angle + span * i as f32 / samples.max(1) as f32;
                let travel = if pushed { 1.0 } else { bearing.cos().max(0.0) };
                let radius = robot_radius + self.config.trigger_distance * travel;
                let angle = theta + bearing;
                map.is_occupied(x + radius * angle.cos(), y + radius * angle.sin())
            })
        };
        (pressed(&self.config.left), pressed(&self.config.right))
    }
}

/// Cliff sensor simulator
pub struct CliffSimulator {
    config: CliffsConfig,
}

impl CliffSimulator {
    /// Create new cliff simulator
    pub fn new(config: &CliffsConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Check cliff states based on map
    ///
    /// # Arguments
    /// * `map` - Simulation map with cliff mask
    /// * `x` - Robot X position (meters)
    /// * `y` - Robot Y position (meters)
    /// * `theta` - Robot orientation (radians)
    ///
    /// # Returns
    /// HashMap with sensor names and their triggered states
    pub fn check(&self, map: &SimulationMap, x: f32, y: f32, theta: f32) -> HashMap<String, bool> {
        let mut states = HashMap::new();

        if !self.config.enabled {
            // Return all sensors as not triggered
            for sensor in &self.config.sensors {
                states.insert(sensor.name.clone(), false);
            }
            return states;
        }

        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        for sensor in &self.config.sensors {
            // Transform sensor position to world coordinates
            let world_x = x + sensor.x * cos_theta - sensor.y * sin_theta;
            let world_y = y + sensor.x * sin_theta + sensor.y * cos_theta;

            // Check if cliff at sensor position
            let triggered = map.is_cliff(world_x, world_y);
            states.insert(sensor.name.clone(), triggered);
        }

        states
    }
}

/// Actuator state tracker (for logging)
pub struct ActuatorState {
    /// Current states by actuator ID
    states: HashMap<String, String>,
}

impl ActuatorState {
    /// Create new actuator state tracker
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
        }
    }

    /// Handle an actuator command
    pub fn handle_command(&mut self, id: &str, action: &ComponentAction, log_changes: bool) {
        let state_str = match action {
            ComponentAction::Enable { config } => {
                if let Some(cfg) = config {
                    format!("enabled({:?})", cfg)
                } else {
                    "enabled".to_string()
                }
            }
            ComponentAction::Disable { .. } => "disabled".to_string(),
            ComponentAction::Reset { .. } => "reset".to_string(),
            ComponentAction::Configure { config } => {
                format!("configured({:?})", config)
            }
        };

        let old_state = self.states.get(id).cloned();
        self.states.insert(id.to_string(), state_str.clone());

        if log_changes {
            if let Some(old) = old_state {
                if old != state_str {
                    log::debug!("Actuator '{}': {} -> {}", id, old, state_str);
                }
            } else {
                log::debug!("Actuator '{}': -> {}", id, state_str);
            }
        }
    }
}

impl Default for ActuatorState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bumper_no_collision() {
        let config = BumpersConfig::default();
        let bumper = BumperSimulator::new(&config);

        // Create a simple free space map (all white)
        let img = image::GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap::from_image(img, 0.1, (0.0, 0.0));

        let (left, right) = bumper.check(&map, 5.0, 5.0, 0.0, 0.17, true);

        assert!(!left);
        assert!(!right);
    }

    #[test]
    fn a_thin_leg_presses_the_shell_where_it_touches() {
        let bumper = BumperSimulator::new(&BumpersConfig::default());
        // A 2 cm chair leg on a 1 cm map, touching the shell of a robot at (1, 1)
        // from `bearing` (radians, CCW from the robot's heading).
        let leg = |bearing: f32| {
            let (lx, ly) = (1.0 + 0.175 * bearing.cos(), 1.0 + 0.175 * bearing.sin());
            let img = image::GrayImage::from_fn(200, 200, |px, py| {
                let (x, y) = ((px as f32 + 0.5) * 0.01, (199 - py) as f32 * 0.01 + 0.005);
                let inside = (x - lx).hypot(y - ly) < 0.01;
                image::Luma([if inside { 0 } else { 255 }])
            });
            SimulationMap::from_image(img, 0.01, (0.0, 0.0))
        };
        let pressed = |bearing: f32| bumper.check(&leg(bearing), 1.0, 1.0, 0.0, 0.17, false);

        // Every 0.05 rad along the shell (finer than the old 16 samples per zone), away
        // from the overlap's edges, where the 2 cm leg straddles them.
        for i in -32..=32 {
            let bearing = i as f32 * 0.05;
            let want = match bearing {
                b if b.abs() < 0.2 => (true, true),
                b if b > 0.4 => (true, false),
                b if b < -0.4 => (false, true),
                _ => continue,
            };
            assert_eq!(pressed(bearing), want, "a leg at {bearing:.2} rad");
        }
        assert_eq!(
            pressed(std::f32::consts::PI),
            (false, false),
            "the back has no shell"
        );
    }

    #[test]
    fn the_shell_slides_back_so_a_wall_alongside_does_not_press_it() {
        let bumper = BumperSimulator::new(&BumpersConfig::default());
        let trigger = BumpersConfig::default().trigger_distance;
        // A wall up to y = 0.01 on a 1 mm map.
        let img = image::GrayImage::from_fn(1000, 1000, |_, py| {
            image::Luma([if py >= 990 { 0 } else { 255 }])
        });
        let map = SimulationMap::from_image(img, 0.001, (0.0, 0.0));
        // The robot `gap` (m) off it, heading `theta`.
        let pressed = |gap: f32, theta: f32, pushed: bool| {
            let y = 0.01 + 0.17 + gap;
            let (left, right) = bumper.check(&map, 0.5, y, theta, 0.17, pushed);
            left || right
        };
        let into = -std::f32::consts::FRAC_PI_2;
        assert!(
            pressed(trigger * 0.8, into, false),
            "head on, within the travel"
        );
        assert!(!pressed(trigger * 1.2, into, false), "head on, beyond it");
        assert!(pressed(0.003, -1.2, false), "at a slant, slid back");
        assert!(!pressed(0.003, 0.0, false), "alongside, no longer pushing");
        assert!(pressed(0.003, 0.0, true), "alongside, pushing into it");
    }

    #[test]
    fn test_cliff_simulator() {
        let config = CliffsConfig::default();
        let cliff = CliffSimulator::new(&config);

        // Create map with no cliff mask
        let img = image::GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap::from_image(img, 0.1, (0.0, 0.0));

        let states = cliff.check(&map, 5.0, 5.0, 0.0);

        // All sensors should report no cliff
        assert_eq!(states.len(), 4);
        assert!(!states["left_side"]);
        assert!(!states["left_front"]);
        assert!(!states["right_front"]);
        assert!(!states["right_side"]);
    }

    #[test]
    fn test_actuator_state() {
        let mut state = ActuatorState::new();

        state.handle_command(
            "main_brush",
            &ComponentAction::Enable { config: None },
            false,
        );

        assert_eq!(state.states.get("main_brush"), Some(&"enabled".to_string()));

        state.handle_command(
            "main_brush",
            &ComponentAction::Disable { config: None },
            false,
        );

        assert_eq!(
            state.states.get("main_brush"),
            Some(&"disabled".to_string())
        );
    }
}
