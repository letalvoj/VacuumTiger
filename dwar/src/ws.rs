//! One WebSocket per browser tab, `/api/v1/ws`. A thin transport: motion
//! rules (teleop holder, dead-man, STOP) live in `control.rs`.
//!
//! Server → client: JSON text frames for small topics (`hello`, `status`,
//! `sensors`, `pose`, `map_meta`, `event`, `teleop`) and binary frames for
//! bulk data (`wire.rs`: lidar scans, map tiles).
//!
//! Client → server (JSON):
//! - `{"type":"subscribe","topics":{"sensors":10,"scan":6,"pose":10,"map":1,"status":1,"events":1}}`
//!   (Hz caps; 0 or absent = off). Every topic is latest-wins: a slow client
//!   gets fresh data at a lower rate, never a backlog.
//! - `{"type":"teleop","take":true|false}` → reply `{"type":"teleop","holder":bool,"error"?}`
//! - `{"type":"drive","linear":0.1,"angular":0.0}` (holder only, 10–20 Hz)
//! - `{"type":"stop"}` (anyone, always; revokes teleop)

use crate::control::SessionId;
use crate::hub::Hub;
use crate::map::MapState;
use crate::wire;
use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use serde::Deserialize;
use serde_json::json;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, watch};

/// Client messages are tiny; anything bigger is a mistake or an attack.
const MAX_MESSAGE: usize = 4096;
/// A client that can't take a frame within this long is gone (or hopeless).
const SEND_TIMEOUT: Duration = Duration::from_secs(2);
const MAX_HZ: f32 = 50.0;
const MIN_HZ: f32 = 0.1;

pub async fn upgrade(State(hub): State<Arc<Hub>>, ws: WebSocketUpgrade) -> Response {
    ws.max_message_size(MAX_MESSAGE).on_upgrade(move |socket| async move {
        let id = hub.new_session_id();
        log::info!("WebSocket session #{} opened", id);
        Session::new(Arc::clone(&hub), id).run(socket).await;
        hub.release_teleop(id, "tab closed");
        log::info!("WebSocket session #{} closed", id);
    })
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientMsg {
    Subscribe { topics: HashMap<String, f32> },
    Teleop { take: bool },
    Drive { linear: f32, angular: f32 },
    Stop,
}

/// Minimum interval for a requested rate; None = off.
fn period(hz: f32) -> Option<Duration> {
    (hz.is_finite() && hz > 0.0).then(|| Duration::from_secs_f32(1.0 / hz.clamp(MIN_HZ, MAX_HZ)))
}

/// A latest-wins text/binary topic with a rate cap.
struct Topic {
    rx: watch::Receiver<Option<Message>>,
    period: Option<Duration>,
    last: Option<Instant>,
}

impl Topic {
    fn new(tx: &watch::Sender<Option<Message>>) -> Self {
        Self {
            rx: tx.subscribe(),
            period: None,
            last: None,
        }
    }

    fn set_rate(&mut self, hz: f32) {
        self.period = period(hz);
        self.rx.mark_changed(); // send the current value right away
    }

    /// The newest value, if it changed and the rate cap allows sending now.
    fn poll(&mut self) -> Option<Message> {
        let period = self.period?;
        if self.last.is_some_and(|l| l.elapsed() < period) || !self.rx.has_changed().unwrap_or(false) {
            return None;
        }
        self.last = Some(Instant::now());
        self.rx.borrow_and_update().clone()
    }
}

struct Session {
    hub: Arc<Hub>,
    id: SessionId,
    sensors: Topic,
    scan: Topic,
    pose: Topic,
    status: Topic,
    map: watch::Receiver<Option<Arc<MapState>>>,
    map_period: Option<Duration>,
    map_last: Option<Instant>,
    /// Map version / geometry this client has.
    map_sent: (u64, u64),
    events: Option<broadcast::Receiver<Message>>,
    /// A "take teleop first" reply was sent; don't repeat it per drive frame.
    refused_drive: bool,
}

async fn send(socket: &mut WebSocket, m: Message) -> Result<(), ()> {
    match tokio::time::timeout(SEND_TIMEOUT, socket.send(m)).await {
        Ok(Ok(())) => Ok(()),
        _ => Err(()),
    }
}

impl Session {
    fn new(hub: Arc<Hub>, id: SessionId) -> Self {
        Self {
            sensors: Topic::new(&hub.sensors),
            scan: Topic::new(&hub.scan),
            pose: Topic::new(&hub.pose),
            status: Topic::new(&hub.status),
            map: hub.map.subscribe(),
            map_period: None,
            map_last: None,
            map_sent: (0, 0),
            events: None,
            refused_drive: false,
            hub,
            id,
        }
    }

    async fn run(mut self, mut socket: WebSocket) {
        let hello = json!({
            "type": "hello",
            "session": self.id,
            "robot": self.hub.cfg.robot,
            "topics": ["sensors", "scan", "pose", "map", "status", "events"],
        });
        if send(&mut socket, Message::Text(hello.to_string().into())).await.is_err() {
            return;
        }
        let mut tick = tokio::time::interval(Duration::from_millis(20));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            let event = async {
                match self.events.as_mut() {
                    Some(rx) => rx.recv().await,
                    None => std::future::pending().await,
                }
            };
            let ok = tokio::select! {
                msg = socket.recv() => match msg {
                    Some(Ok(Message::Text(t))) => match self.on_message(&t) {
                        Some(reply) => send(&mut socket, Message::Text(reply.into())).await,
                        None => Ok(()),
                    },
                    Some(Ok(Message::Close(_))) | None | Some(Err(_)) => Err(()),
                    Some(Ok(_)) => Ok(()),
                },
                ev = event => match ev {
                    Ok(m) => send(&mut socket, m).await,
                    Err(broadcast::error::RecvError::Lagged(_)) => Ok(()),
                    Err(broadcast::error::RecvError::Closed) => {
                        self.events = None;
                        Ok(())
                    }
                },
                _ = tick.tick() => self.flush(&mut socket).await,
            };
            if ok.is_err() {
                break;
            }
        }
    }

    async fn flush(&mut self, socket: &mut WebSocket) -> Result<(), ()> {
        for topic in [&mut self.status, &mut self.sensors, &mut self.pose, &mut self.scan] {
            if let Some(m) = topic.poll() {
                send(socket, m).await?;
            }
        }
        if let Some(period) = self.map_period
            && self.map_last.is_none_or(|l| l.elapsed() >= period)
            && self.map.has_changed().unwrap_or(false)
        {
            self.map_last = Some(Instant::now());
            let map = self.map.borrow_and_update().clone();
            if let Some(m) = map {
                self.send_map(socket, &m).await?;
            }
        }
        Ok(())
    }

    /// Send what this client lacks: everything after a geometry change (or on
    /// subscribe), otherwise only the tiles newer than its version.
    async fn send_map(&mut self, socket: &mut WebSocket, m: &MapState) -> Result<(), ()> {
        let (sent_version, sent_geometry) = self.map_sent;
        let full = sent_geometry != m.geometry;
        if full {
            send(socket, Message::Text(m.meta_json().to_string().into())).await?;
        }
        for ty in 0..m.tiles_y {
            for tx in 0..m.tiles_x {
                if full || m.tile_version[ty * m.tiles_x + tx] > sent_version {
                    send(socket, Message::Binary(wire::tile_frame(m, tx, ty))).await?;
                }
            }
        }
        self.map_sent = (m.version, m.geometry);
        Ok(())
    }

    fn on_message(&mut self, text: &str) -> Option<String> {
        let msg = match serde_json::from_str::<ClientMsg>(text) {
            Ok(m) => m,
            Err(e) => return Some(json!({"type": "error", "error": e.to_string()}).to_string()),
        };
        match msg {
            ClientMsg::Subscribe { topics } => {
                let hz = |k: &str| topics.get(k).copied().unwrap_or(0.0);
                self.sensors.set_rate(hz("sensors"));
                self.scan.set_rate(hz("scan"));
                self.pose.set_rate(hz("pose"));
                self.status.set_rate(hz("status"));
                let map_period = period(hz("map").min(10.0));
                if map_period.is_some() && self.map_period.is_none() {
                    self.map_sent = (0, 0); // newly subscribed: send everything
                    self.map.mark_changed();
                }
                self.map_period = map_period;
                self.events = (hz("events") > 0.0).then(|| self.hub.events.subscribe());
                None
            }
            ClientMsg::Stop => {
                let r = self.hub.stop_all(&format!("web session #{}", self.id));
                r.err().map(|e| json!({"type": "error", "error": e}).to_string())
            }
            ClientMsg::Teleop { take } => {
                let result = if take {
                    self.hub.take_teleop(self.id)
                } else {
                    self.hub.release_teleop(self.id, "released");
                    Ok(())
                };
                self.refused_drive = false;
                let holder = self.hub.teleop_holder() == Some(self.id);
                Some(json!({"type": "teleop", "holder": holder, "error": result.err()}).to_string())
            }
            ClientMsg::Drive { linear, angular } => match self.hub.drive(self.id, linear, angular) {
                Ok(()) => None,
                Err(e) if !self.refused_drive => {
                    self.refused_drive = true;
                    Some(json!({"type": "teleop", "holder": false, "error": e}).to_string())
                }
                Err(_) => None,
            },
        }
    }
}
