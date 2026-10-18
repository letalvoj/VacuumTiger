//! DhruvaSLAM upstream: TCP carries commands, responses, maps and the map
//! list; pose and SLAM status arrive over UDP at the TCP peer address, so we
//! listen on our own TCP port number. Reconnects with backoff.

use crate::hub::Hub;
use crate::map::MapState;
use crate::proto::dhruva::{self, dhruva_stream::Data};
use axum::extract::ws::Message;
use prost::Message as _;
use serde_json::{Value, json};
use std::sync::Arc;
use std::time::Duration;
use futures_util::StreamExt;
use tokio::io::AsyncWriteExt;
use tokio_util::codec::{FramedRead, LengthDelimitedCodec};
use tokio::net::{TcpStream, UdpSocket};
use tokio::sync::mpsc;

/// Largest TCP frame accepted (a full map: 4000 × 4000 cells = 16 MB).
const MAX_FRAME: usize = 16 << 20;

const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// A command write that takes longer means DhruvaSLAM stopped reading: reconnect.
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

pub async fn run(hub: Arc<Hub>, mut rx: mpsc::Receiver<dhruva::DhruvaCommand>) {
    let mut backoff = Duration::from_millis(500);
    loop {
        let connect = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(&hub.cfg.dhruva.address));
        if let Ok(Ok(sock)) = connect.await {
            backoff = Duration::from_millis(500);
            // Commands queued while offline are stale; don't replay them.
            while rx.try_recv().is_ok() {}
            match session(&hub, sock, &mut rx).await {
                Ok(()) => {}
                Err(e) => hub.event("warn", format!("DhruvaSLAM disconnected: {}", e)),
            }
            {
                let mut s = hub.shared();
                s.dhruva_connected = false;
                s.pose = None;
                s.robot_status = None;
                s.mapping = None;
                s.maps = serde_json::json!([]);
            }
            hub.pose.send_replace(None);
            hub.map.send_replace(None);
        } else {
            while rx.try_recv().is_ok() {}
        }
        tokio::time::sleep(backoff).await;
        backoff = (backoff * 2).min(Duration::from_secs(5));
    }
}

async fn session(
    hub: &Hub,
    sock: TcpStream,
    rx: &mut mpsc::Receiver<dhruva::DhruvaCommand>,
) -> Result<(), String> {
    let local = sock.local_addr().map_err(|e| e.to_string())?;
    let udp = UdpSocket::bind(("0.0.0.0", local.port()))
        .await
        .map_err(|e| format!("UDP bind {}: {}", local.port(), e))?;
    let (r, mut w) = sock.into_split();
    hub.shared().dhruva_connected = true;
    hub.event("info", format!("DhruvaSLAM connected ({})", hub.cfg.dhruva.address));

    // `u32 BE length | protobuf`; FramedRead is cancel-safe in select!.
    let mut frames = FramedRead::new(
        r,
        LengthDelimitedCodec::builder().length_field_length(4).max_frame_length(MAX_FRAME).new_codec(),
    );
    let mut buf = vec![0u8; 65536];
    loop {
        tokio::select! {
            body = frames.next() => match body {
                Some(Ok(body)) => on_tcp(hub, &body),
                Some(Err(e)) => return Err(e.to_string()),
                None => return Err("connection closed".into()),
            },
            got = udp.recv(&mut buf) => {
                if let Ok(n) = got
                    && n >= 4
                {
                    let len = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
                    if 4 + len <= n
                        && let Ok(msg) = dhruva::DhruvaStream::decode(&buf[4..4 + len])
                    {
                        on_stream(hub, msg);
                    }
                }
            }
            cmd = rx.recv() => {
                let Some(cmd) = cmd else { return Ok(()) };
                let body = cmd.encode_to_vec();
                let mut out = (body.len() as u32).to_be_bytes().to_vec();
                out.extend_from_slice(&body);
                match tokio::time::timeout(WRITE_TIMEOUT, w.write_all(&out)).await {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => return Err(e.to_string()),
                    Err(_) => return Err("command write timed out".into()),
                }
            }
        }
    }
}

/// TCP frames are either a `DhruvaStream` (map, map list) or a `DhruvaResponse`.
/// The two don't parse as each other (field 1 is a varint in one, a string in
/// the other), so try the stream first.
fn on_tcp(hub: &Hub, body: &[u8]) {
    if let Ok(msg) = dhruva::DhruvaStream::decode(body)
        && msg.data.is_some()
    {
        on_stream(hub, msg);
    } else if let Ok(resp) = dhruva::DhruvaResponse::decode(body) {
        let level = if resp.success { "info" } else { "warn" };
        let text = if resp.success {
            format!("DhruvaSLAM {}: ok {}", resp.request_id, response_detail(&resp))
        } else {
            format!("DhruvaSLAM {}: {}", resp.request_id, resp.error_message)
        };
        hub.event(level, text);
    }
}

fn response_detail(r: &dhruva::DhruvaResponse) -> String {
    use dhruva::dhruva_response::Data as D;
    match &r.data {
        Some(D::MappingStarted(m)) => format!("(mapping started, map {})", m.map_id),
        Some(D::MappingStopped(m)) => format!("(mapping stopped, saved={}, {:.1} m²)", m.saved, m.area_m2),
        Some(D::GoalAccepted(g)) => format!("(goal {} accepted)", g.target_id),
        _ => String::new(),
    }
}

fn on_stream(hub: &Hub, msg: dhruva::DhruvaStream) {
    match msg.data {
        Some(Data::RobotStatus(rs)) => {
            let pose = rs.pose.as_ref().map(|p| (p.x, p.y, p.theta));
            let state = dhruva::RobotState::try_from(rs.state)
                .map(|s| s.as_str_name().trim_start_matches("ROBOT_STATE_").to_lowercase())
                .unwrap_or_default();
            let status = json!({
                "state": state,
                "active_map_id": rs.active_map_id,
                "distance_traveled_m": rs.distance_traveled_m,
                "keyframe_count": rs.keyframe_count,
                "loop_closures": rs.loop_closures,
                "map_area_m2": rs.map_area_m2,
                "localization_confidence": rs.localization_confidence,
            });
            let (x, y, theta) = pose.unwrap_or((0.0, 0.0, 0.0));
            let text = json!({
                "type": "pose",
                "t_us": msg.timestamp_us,
                "x": x, "y": y, "theta": theta,
                "slam": status,
            })
            .to_string();
            {
                let mut s = hub.shared();
                s.pose = pose;
                s.robot_status = Some(status);
            }
            hub.pose.send_replace(Some(Message::Text(text.into())));
        }
        Some(Data::CurrentMap(m)) => {
            let prev = hub.map.borrow().clone();
            if let Some(next) = MapState::update(prev.as_deref(), m, hub.next_map_version()) {
                let changed = prev.as_ref().is_none_or(|p| next.tile_version.iter().any(|&v| v > p.version));
                if changed {
                    hub.map.send_replace(Some(Arc::new(next)));
                }
            }
        }
        Some(Data::MapList(l)) => {
            let maps: Vec<Value> = l
                .maps
                .iter()
                .map(|m| {
                    json!({
                        "map_id": m.map_id,
                        "name": m.name,
                        "created_at_us": m.created_at_us,
                        "updated_at_us": m.updated_at_us,
                        "area_m2": m.area_m2,
                        "room_count": m.room_count,
                        "is_complete": m.is_complete,
                        "active": m.map_id == l.active_map_id,
                    })
                })
                .collect();
            hub.shared().maps = Value::Array(maps);
        }
        Some(Data::MappingProgress(p)) => {
            let state = dhruva::MappingState::try_from(p.state)
                .map(|s| s.as_str_name().trim_start_matches("MAPPING_STATE_").to_lowercase())
                .unwrap_or_default();
            hub.shared().mapping = Some(json!({
                "state": state,
                "scan_count": p.scan_count,
                "area_explored_m2": p.area_explored_m2,
                "elapsed_time_ms": p.elapsed_time_ms,
                "status_message": p.status_message,
            }));
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_response_is_not_mistaken_for_a_stream() {
        let resp = dhruva::DhruvaResponse {
            request_id: "dwar-1".into(),
            success: true,
            ..Default::default()
        };
        let bytes = resp.encode_to_vec();
        assert!(dhruva::DhruvaStream::decode(&bytes[..]).map_or(true, |s| s.data.is_none()));
        assert_eq!(dhruva::DhruvaResponse::decode(&bytes[..]).unwrap().request_id, "dwar-1");
    }
}
