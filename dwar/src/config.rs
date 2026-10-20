//! dwar configuration (TOML). Every section and key is optional.

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub http: Http,
    pub sangamio: Sangamio,
    pub dhruva: Dhruva,
    pub control: Control,
    pub recordings: Recordings,
    pub robot: Robot,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Http {
    /// Listen address of the web server.
    pub bind: String,
    /// Optional shared secret: if set, `/api` needs `Authorization: Bearer <token>`
    /// or `?token=<token>` (the WebSocket can't send headers from a browser).
    pub token: Option<String>,
    /// Without a token, requests must name the robot by IP, `localhost`, a
    /// `.local` name or one of these hosts (blocks DNS-rebinding pages).
    pub allowed_hosts: Vec<String>,
}

impl Default for Http {
    fn default() -> Self {
        Self {
            bind: "0.0.0.0:8080".into(),
            token: None,
            allowed_hosts: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Sangamio {
    pub address: String,
}

impl Default for Sangamio {
    fn default() -> Self {
        Self {
            address: "127.0.0.1:5555".into(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Dhruva {
    pub enabled: bool,
    pub address: String,
}

impl Default for Dhruva {
    fn default() -> Self {
        Self {
            enabled: true,
            address: "127.0.0.1:5557".into(),
        }
    }
}

/// What the web UI may do to the hardware. Motion is off unless enabled here.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Control {
    /// Teleop (wheels). Keep off unless the robot is on blocks or supervised.
    pub allow_drive: bool,
    /// Vacuum, brushes, water pump, LED. The lidar is always allowed.
    pub allow_actuators: bool,
    /// Teleop limits (m/s, rad/s).
    pub max_linear: f32,
    pub max_angular: f32,
    /// No teleop message for this long -> zero velocity (SangamIO's heartbeat
    /// dead-man, `deadman_timeout_ms`, stays as the layer below).
    pub deadman_ms: u64,
}

impl Default for Control {
    fn default() -> Self {
        Self {
            allow_drive: false,
            allow_actuators: false,
            max_linear: 0.2,
            max_angular: 1.0,
            deadman_ms: 300,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Recordings {
    /// Where `.mcap` recordings go. Keep it in RAM (/tmp) on the robot.
    pub dir: String,
    /// A recording stops by itself at this size (MB on disk).
    pub max_mb: u64,
    /// No new recording once the directory holds this much (MB).
    pub max_total_mb: u64,
}

impl Default for Recordings {
    fn default() -> Self {
        Self {
            dir: "/tmp/v8/cap".into(),
            max_mb: 64,
            max_total_mb: 128,
        }
    }
}

/// Static robot description for the UI.
#[derive(Debug, Clone, Deserialize, serde::Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct Robot {
    pub name: String,
    pub model: String,
    /// Lidar position in the robot frame (m). SangamIO rotates scans already;
    /// the translation is left to consumers.
    pub lidar_offset_x: f32,
    pub lidar_offset_y: f32,
    /// Body radius for drawing (m).
    pub radius: f32,
}

impl Default for Robot {
    fn default() -> Self {
        Self {
            name: "VacuumTiger".into(),
            model: "CRL-200S".into(),
            lidar_offset_x: -0.105,
            lidar_offset_y: 0.0,
            radius: 0.175,
        }
    }
}

impl Config {
    pub fn load(path: Option<&str>) -> Result<Self, String> {
        let cfg: Self = match path {
            None => Self::default(),
            Some(p) => {
                let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {}", p, e))?;
                basic_toml::from_str(&text).map_err(|e| format!("{}: {}", p, e))?
            }
        };
        cfg.validate()?;
        Ok(cfg)
    }

    fn validate(&self) -> Result<(), String> {
        let c = &self.control;
        let limit = |name: &str, v: f32, max: f32| {
            if v.is_finite() && v > 0.0 && v <= max {
                Ok(())
            } else {
                Err(format!("[control] {} must be in (0, {}], got {}", name, max, v))
            }
        };
        limit("max_linear", c.max_linear, 0.5)?;
        limit("max_angular", c.max_angular, 3.0)?;
        if !(50..=2000).contains(&c.deadman_ms) {
            return Err(format!("[control] deadman_ms must be 50..2000, got {}", c.deadman_ms));
        }
        if self.recordings.max_mb == 0 || self.recordings.max_total_mb < self.recordings.max_mb {
            return Err("[recordings] need 0 < max_mb <= max_total_mb".into());
        }
        if self.http.token.as_deref() == Some("") {
            return Err("[http] token must not be empty".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_valid_and_bad_values_are_refused() {
        assert!(Config::default().validate().is_ok());
        assert!(basic_toml::from_str::<Config>("[control]\nalow_drive = true").is_err(), "typo");
        let mut c = Config::default();
        c.control.max_linear = f32::NAN;
        assert!(c.validate().is_err());
        c = Config::default();
        c.control.deadman_ms = 0;
        assert!(c.validate().is_err());
    }
}
