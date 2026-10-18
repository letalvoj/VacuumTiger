//! State hub: the latest value of every topic, shared by the HTTP handlers and
//! the WebSocket sessions.
//!
//! Upstream tasks (SangamIO, DhruvaSLAM) publish into `watch` channels, which
//! keep only the newest value. A WebSocket session that falls behind (a phone
//! on bad Wi-Fi) therefore skips to fresh data instead of building a backlog.

use crate::config::Config;
use crate::control::Teleop;
use crate::map::MapState;
use crate::proto::{dhruva, sangamio};
use axum::extract::ws::Message;
use serde_json::{Map, Value, json};
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, mpsc, watch};

pub use crate::sangam::Action;

/// Snapshot values for REST handlers and the status topic.
#[derive(Default)]
pub struct Shared {
    pub sangam_connected: bool,
    pub dhruva_connected: bool,
    pub sensors: Map<String, Value>,
    pub sensors_t_us: u64,
    pub version: Map<String, Value>,
    pub groups: BTreeMap<String, u64>,
    pub robot_status: Option<Value>,
    pub pose: Option<(f32, f32, f32)>,
    pub mapping: Option<Value>,
    pub maps: Value,
    /// Last state the UI commanded per actuator. Optimistic: SangamIO has no
    /// replies and doesn't report actuator state.
    pub actuators: Map<String, Value>,
    pub sensor_hz: f32,
    pub scan_hz: f32,
    pub scan_points: usize,
}

pub struct Hub {
    pub cfg: Config,
    pub started: Instant,
    pub shared: Mutex<Shared>,
    pub(crate) teleop: Mutex<Teleop>,
    /// Text topics: latest JSON message.
    pub sensors: watch::Sender<Option<Message>>,
    pub pose: watch::Sender<Option<Message>>,
    pub status: watch::Sender<Option<Message>>,
    /// Binary scan frame (`wire::scan_frame`).
    pub scan: watch::Sender<Option<Message>>,
    pub map: watch::Sender<Option<Arc<MapState>>>,
    /// Events: refused commands, upstream up/down, dead-man trips.
    pub events: broadcast::Sender<Message>,
    pub to_sangam: mpsc::Sender<sangamio::Command>,
    /// Stops and dead-man zeroes: sent before anything in `to_sangam`.
    pub to_sangam_urgent: mpsc::Sender<sangamio::Command>,
    pub to_dhruva: mpsc::Sender<dhruva::DhruvaCommand>,
    pub recorder: crate::recorder::Recorder,
    /// Open WebSocket sessions (capped in ws.rs).
    pub ws_sessions: std::sync::atomic::AtomicUsize,
    next_session: AtomicU64,
    next_request: AtomicU64,
    /// Map versions, monotonic across DhruvaSLAM reconnects.
    map_version: AtomicU64,
    /// When each throttled event text was last sent (drops come in bursts).
    last_throttled: Mutex<HashMap<String, Instant>>,
}

pub struct HubInit {
    pub hub: Arc<Hub>,
    pub sangam_rx: crate::sangam::Queues,
    pub dhruva_rx: mpsc::Receiver<dhruva::DhruvaCommand>,
}

impl Hub {
    /// The hub plus the receiving ends of its command queues (for the upstream tasks).
    pub fn build(cfg: Config) -> Result<HubInit, String> {
        let (to_sangam, normal) = mpsc::channel(64);
        let (to_sangam_urgent, urgent) = mpsc::channel(32);
        let (to_dhruva, dhruva_rx) = mpsc::channel(16);
        let recorder = crate::recorder::Recorder::new(&cfg.recordings)?;
        let hub = Arc::new(Hub {
            cfg,
            started: Instant::now(),
            shared: Mutex::new(Shared {
                maps: json!([]),
                ..Default::default()
            }),
            teleop: Mutex::default(),
            sensors: watch::channel(None).0,
            pose: watch::channel(None).0,
            status: watch::channel(None).0,
            scan: watch::channel(None).0,
            map: watch::channel(None).0,
            events: broadcast::channel(64).0,
            to_sangam,
            to_sangam_urgent,
            to_dhruva,
            recorder,
            ws_sessions: Default::default(),
            next_session: AtomicU64::new(1),
            next_request: AtomicU64::new(1),
            map_version: AtomicU64::new(0),
            last_throttled: Mutex::default(),
        });
        Ok(HubInit {
            hub,
            sangam_rx: crate::sangam::Queues { normal, urgent },
            dhruva_rx,
        })
    }

    pub fn shared(&self) -> std::sync::MutexGuard<'_, Shared> {
        self.shared.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn next_map_version(&self) -> u64 {
        self.map_version.fetch_add(1, Ordering::Relaxed) + 1
    }

    pub fn new_session_id(&self) -> u64 {
        self.next_session.fetch_add(1, Ordering::Relaxed)
    }

    /// Log and broadcast an event to every WebSocket session.
    pub fn event(&self, level: &str, text: impl Into<String>) {
        let text = text.into();
        match level {
            "error" => log::error!("{}", text),
            "warn" => log::warn!("{}", text),
            _ => log::info!("{}", text),
        }
        let msg = json!({
            "type": "event",
            "level": level,
            "text": text,
            "t": self.started.elapsed().as_secs_f64(),
        });
        let _ = self.events.send(Message::Text(msg.to_string().into()));
    }

    /// Like `event`, but each distinct text at most once a second (for
    /// per-command failures).
    pub fn event_throttled(&self, level: &str, text: impl Into<String>) {
        let text = text.into();
        let mut last = self.last_throttled.lock().unwrap_or_else(|e| e.into_inner());
        last.retain(|_, t| t.elapsed() < Duration::from_secs(1));
        if !last.contains_key(&text) {
            last.insert(text.clone(), Instant::now());
            drop(last);
            self.event(level, text);
        }
    }

    /// Queue a SangamIO component command. Returns false if it was dropped.
    pub fn sangam(&self, id: &str, action: Action, config: &[(&str, sangamio::sensor_value::Value)]) -> bool {
        self.queue(&self.to_sangam, id, action, config)
    }

    /// Like `sangam`, on the lane that goes first (stops, dead-man zero).
    pub fn sangam_urgent(&self, id: &str, action: Action, config: &[(&str, sangamio::sensor_value::Value)]) -> bool {
        self.queue(&self.to_sangam_urgent, id, action, config)
    }

    fn queue(
        &self,
        lane: &mpsc::Sender<sangamio::Command>,
        id: &str,
        action: Action,
        config: &[(&str, sangamio::sensor_value::Value)],
    ) -> bool {
        match lane.try_send(crate::sangam::component(id, action, config)) {
            Ok(()) => true,
            Err(e) => {
                self.event_throttled("warn", format!("SangamIO command '{}' dropped: {}", id, e));
                false
            }
        }
    }

    /// Queue a DhruvaSLAM command; returns its request id.
    pub fn dhruva(&self, cmd: dhruva::dhruva_command::Command) -> Result<String, String> {
        if !self.shared().dhruva_connected {
            return Err("DhruvaSLAM offline".into());
        }
        let id = format!("dwar-{}", self.next_request.fetch_add(1, Ordering::Relaxed));
        let msg = dhruva::DhruvaCommand {
            request_id: id.clone(),
            command: Some(cmd),
        };
        self.to_dhruva
            .try_send(msg)
            .map(|_| id)
            .map_err(|e| format!("DhruvaSLAM command dropped: {}", e))
    }

    /// What a recording says about where it came from (its `run` metadata).
    pub fn run_metadata(&self) -> BTreeMap<String, String> {
        let r = &self.cfg.robot;
        let mut m = BTreeMap::from([
            ("robot".to_string(), r.name.clone()),
            ("model".to_string(), r.model.clone()),
            ("dwar".to_string(), env!("CARGO_PKG_VERSION").to_string()),
        ]);
        for (k, v) in &self.shared().version {
            let v = v.as_str().map_or_else(|| v.to_string(), str::to_string);
            m.insert(format!("sangamio.{}", k), v);
        }
        m
    }

    /// The JSON status topic (also `GET /api/v1/status`).
    pub fn status_json(&self) -> Value {
        let teleop_holder = self.teleop_holder();
        let recording = self.recorder.info();
        let s = self.shared();
        json!({
            "type": "status",
            "uptime_s": self.started.elapsed().as_secs(),
            "sangamio": {
                "connected": s.sangam_connected,
                "address": self.cfg.sangamio.address,
                "sensor_hz": s.sensor_hz,
                "scan_hz": s.scan_hz,
                "scan_points": s.scan_points,
                "groups": s.groups,
            },
            "dhruva": {
                "enabled": self.cfg.dhruva.enabled,
                "connected": s.dhruva_connected,
                "address": self.cfg.dhruva.address,
                "robot_status": s.robot_status,
                "mapping": s.mapping,
            },
            "teleop_holder": teleop_holder,
            "actuators": s.actuators,
            "recording": recording,
            "control": {
                "allow_drive": self.cfg.control.allow_drive,
                "allow_actuators": self.cfg.control.allow_actuators,
                "max_linear": self.cfg.control.max_linear,
                "max_angular": self.cfg.control.max_angular,
            },
        })
    }
}
