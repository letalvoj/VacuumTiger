//! SangamIO upstream: one TCP connection carrying commands out and telemetry in
//! (dwar asks for TCP telemetry, so it never competes with DhruvaSLAM for the
//! UDP port on the same host). Reconnects with backoff.

use crate::hub::Hub;
use crate::proto::sangamio;
use crate::recorder::now_us;
use crate::wire;
use axum::extract::ws::Message;
use prost::Message as _;
use serde_json::{Map, Value, json};
use std::sync::Arc;
use std::time::{Duration, Instant};
use futures_util::StreamExt;
use tokio::io::AsyncWriteExt;
use tokio_util::codec::{FramedRead, LengthDelimitedCodec};
use tokio::net::TcpStream;
use tokio::sync::mpsc;

/// The sensors topic is rebuilt at most this often (SangamIO sends ~110 Hz).
const SENSORS_MIN_INTERVAL: Duration = Duration::from_millis(50);
/// SangamIO streams telemetry continuously; this much silence is a dead link.
const SILENCE: Duration = Duration::from_secs(5);
/// Largest telemetry frame accepted.
const MAX_FRAME: usize = 1 << 20;
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// A command write that takes longer means SangamIO stopped reading: reconnect.
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Clone, Copy, Debug)]
pub enum Action {
    Enable,
    Disable,
    Configure,
}

/// Command queues to SangamIO: `urgent` (stops, dead-man zero) is always sent
/// before `normal`, so a STOP never waits behind queued drive commands.
pub struct Queues {
    pub normal: mpsc::Receiver<sangamio::Command>,
    pub urgent: mpsc::Receiver<sangamio::Command>,
}

impl Queues {
    /// Commands queued while offline are stale; don't replay them.
    fn drain(&mut self) -> usize {
        let mut n = 0;
        while self.normal.try_recv().is_ok() || self.urgent.try_recv().is_ok() {
            n += 1;
        }
        n
    }
}

pub async fn run(hub: Arc<Hub>, mut rx: Queues) {
    let mut backoff = Duration::from_millis(500);
    loop {
        let connect = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(&hub.cfg.sangamio.address));
        if let Ok(Ok(sock)) = connect.await {
            backoff = Duration::from_millis(500);
            rx.drain();
            let err = session(&hub, sock, &mut rx).await;
            hub.event("warn", format!("SangamIO disconnected: {}", err));
            {
                let mut s = hub.shared();
                s.sangam_connected = false;
                s.sensor_hz = 0.0;
                s.scan_hz = 0.0;
            }
            hub.sensors.send_replace(None);
            hub.scan.send_replace(None);
            hub.sangam_lost();
        } else if rx.drain() > 0 {
            hub.event_throttled("warn", "SangamIO offline: command dropped");
        }
        tokio::time::sleep(backoff).await;
        backoff = (backoff * 2).min(Duration::from_secs(5));
    }
}

async fn session(hub: &Hub, sock: TcpStream, rx: &mut Queues) -> String {
    let _ = sock.set_nodelay(true);
    let (r, mut w) = sock.into_split();
    // Switch this client's telemetry to TCP before anything else.
    if let Err(e) = w.write_all(&frame(component("telemetry", Action::Enable, &[]))).await {
        return e.to_string();
    }
    hub.shared().sangam_connected = true;
    hub.event("info", format!("SangamIO connected ({})", hub.cfg.sangamio.address));

    // `u32 BE length | protobuf`; FramedRead is cancel-safe in select!.
    let mut frames = FramedRead::new(
        r,
        LengthDelimitedCodec::builder().length_field_length(4).max_frame_length(MAX_FRAME).new_codec(),
    );
    let mut telemetry = Telemetry::new();
    let silence = tokio::time::sleep(SILENCE);
    tokio::pin!(silence);
    loop {
        // Stops (urgent) go before queued drive commands; telemetry last.
        let cmd = tokio::select! {
            biased;
            cmd = rx.urgent.recv() => {
                // Anything still queued predates the stop: a stale drive
                // command sent after it would undo it.
                let stale = std::iter::from_fn(|| rx.normal.try_recv().ok()).count();
                if stale > 0 {
                    log::info!("dropped {} queued commands behind a stop", stale);
                }
                cmd
            }
            cmd = rx.normal.recv() => cmd,
            body = frames.next() => match body {
                Some(Ok(body)) => {
                    silence.as_mut().reset(tokio::time::Instant::now() + SILENCE);
                    telemetry.on_frame(hub, &body);
                    continue;
                }
                Some(Err(e)) => return e.to_string(),
                None => return "connection closed".into(),
            },
            _ = &mut silence => return format!("no telemetry for {} s", SILENCE.as_secs()),
        };
        let Some(cmd) = cmd else { return "shutting down".into() };
        match tokio::time::timeout(WRITE_TIMEOUT, w.write_all(&frame(cmd))).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => return e.to_string(),
            Err(_) => return "command write timed out".into(),
        }
    }
}

fn frame(cmd: sangamio::Command) -> Vec<u8> {
    let msg = sangamio::Message {
        topic: "command".into(),
        payload: Some(sangamio::message::Payload::Command(cmd)),
    };
    let body = msg.encode_to_vec();
    let mut out = Vec::with_capacity(4 + body.len());
    out.extend_from_slice(&(body.len() as u32).to_be_bytes());
    out.extend_from_slice(&body);
    out
}

/// Rate counter over 1 s windows.
struct Rate {
    n: u32,
    since: Instant,
}

impl Rate {
    fn new() -> Self {
        Self { n: 0, since: Instant::now() }
    }

    /// Count one event; returns the rate when a window closes.
    fn tick(&mut self) -> Option<f32> {
        self.n += 1;
        let dt = self.since.elapsed().as_secs_f32();
        (dt >= 1.0).then(|| {
            let hz = self.n as f32 / dt;
            self.n = 0;
            self.since = Instant::now();
            hz
        })
    }
}

/// Decodes telemetry frames into the hub's topics.
struct Telemetry {
    last_sensors: Instant,
    sensor_rate: Rate,
    scan_rate: Rate,
    scan_seq: u32,
}

impl Telemetry {
    fn new() -> Self {
        Self {
            last_sensors: Instant::now() - SENSORS_MIN_INTERVAL,
            sensor_rate: Rate::new(),
            scan_rate: Rate::new(),
            scan_seq: 0,
        }
    }

    fn on_frame(&mut self, hub: &Hub, body: &[u8]) {
        hub.recorder.write(now_us(), body);
        let Ok(msg) = sangamio::Message::decode(body) else {
            log::debug!("undecodable SangamIO frame ({} B)", body.len());
            return;
        };
        let Some(sangamio::message::Payload::SensorGroup(sg)) = msg.payload else {
            return;
        };
        {
            let mut s = hub.shared();
            match s.groups.get_mut(&sg.group_id) {
                Some(n) => *n += 1,
                None => {
                    s.groups.insert(sg.group_id.clone(), 1);
                }
            }
        }
        match sg.group_id.as_str() {
            "sensor_status" => self.on_sensors(hub, &sg),
            "lidar" => self.on_lidar(hub, &sg),
            "device_version" | "kinematics" => {
                let mut s = hub.shared();
                for (k, v) in &sg.values {
                    s.version.insert(k.clone(), value_json(v));
                }
            }
            _ => {}
        }
    }

    fn on_sensors(&mut self, hub: &Hub, sg: &sangamio::SensorGroup) {
        if let Some(hz) = self.sensor_rate.tick() {
            hub.shared().sensor_hz = hz;
        }
        if self.last_sensors.elapsed() < SENSORS_MIN_INTERVAL {
            return;
        }
        self.last_sensors = Instant::now();
        let values: Map<_, _> = sg.values.iter().map(|(k, v)| (k.clone(), value_json(v))).collect();
        if hub.sensors.receiver_count() > 0 {
            let text = json!({ "type": "sensors", "t_us": sg.timestamp_us, "values": values }).to_string();
            hub.sensors.send_replace(Some(Message::Text(text.into())));
        }
        let mut s = hub.shared();
        s.sensors = values;
        s.sensors_t_us = sg.timestamp_us;
    }

    fn on_lidar(&mut self, hub: &Hub, sg: &sangamio::SensorGroup) {
        let Some(packed) = wire::packed_scan(sg) else { return };
        if let Some(hz) = self.scan_rate.tick() {
            hub.shared().scan_hz = hz;
        }
        self.scan_seq = self.scan_seq.wrapping_add(1);
        let pose = {
            let mut s = hub.shared();
            s.scan_points = wire::packed_points(&packed);
            s.pose
        };
        if hub.scan.receiver_count() > 0 {
            let f = wire::scan_frame(self.scan_seq, sg.timestamp_us, pose, &packed);
            hub.scan.send_replace(Some(Message::Binary(f)));
        }
    }
}

/// A SangamIO `ComponentControl` command.
pub fn component(id: &str, action: Action, config: &[(&str, sangamio::sensor_value::Value)]) -> sangamio::Command {
    use sangamio::component_action::ActionType;
    let r#type = match action {
        Action::Enable => ActionType::Enable,
        Action::Disable => ActionType::Disable,
        Action::Configure => ActionType::Configure,
    } as i32;
    let config = config
        .iter()
        .map(|(k, v)| (k.to_string(), sangamio::SensorValue { value: Some(v.clone()) }))
        .collect();
    sangamio::Command {
        command: Some(sangamio::command::Command::ComponentControl(sangamio::ComponentControl {
            id: id.to_string(),
            action: Some(sangamio::ComponentAction { r#type, config }),
        })),
    }
}

/// SangamIO sensor value -> JSON (vectors as `[x, y, z]`, blobs as their size).
pub fn value_json(v: &sangamio::SensorValue) -> Value {
    use sangamio::sensor_value::Value as V;
    let finite = |x: f64| if x.is_finite() { json!(x) } else { Value::Null };
    match &v.value {
        Some(V::BoolVal(b)) => json!(b),
        Some(V::U32Val(x)) => json!(x),
        Some(V::U64Val(x)) => json!(x),
        Some(V::I32Val(x)) => json!(x),
        Some(V::I64Val(x)) => json!(x),
        Some(V::F32Val(x)) => finite(*x as f64),
        Some(V::F64Val(x)) => finite(*x),
        Some(V::StringVal(s)) => json!(s),
        Some(V::BytesVal(b)) => json!({ "bytes": b.len() }),
        Some(V::Vector3Val(v)) => json!([finite(v.x as f64), finite(v.y as f64), finite(v.z as f64)]),
        Some(V::PointcloudVal(p)) => json!({ "points": p.points.len() }),
        None => Value::Null,
    }
}
