//! Telemetry publisher: fans sensor data out to every connected client
//!
//! Each frame is serialized once and sent to every client in the
//! [`ClientHub`]: one UDP datagram per distinct `<IP>:udp_port` target, or a
//! queued frame for clients that switched to TCP telemetry. It's optimized for
//! low-latency, high-frequency sensor delivery on embedded systems.
//!
//! # Design Principles
//!
//! ## Two Data Delivery Modes
//!
//! Different sensors have different update rates and delivery requirements:
//!
//! | Mode | Sensors | Rate | Mechanism |
//! |------|---------|------|-----------|
//! | Streaming | sensor_status (GD32) | 110Hz | Crossbeam channel |
//! | Polling | lidar, device_version | 5Hz | Shared mutex |
//!
//! **Streaming mode** uses lock-free channels for high-rate sensors where
//! every packet matters and we need to minimize latency.
//!
//! **Polling mode** uses shared mutexes for low-rate sensors where it's
//! acceptable to occasionally miss an update if the publisher is busy.
//!
//! ## Unicast vs Broadcast
//!
//! UDP streaming uses **unicast** (not broadcast) because:
//!
//! - **Privacy**: only connected clients receive data (no auth: trusted LAN)
//! - **Efficiency**: No processing overhead on non-listening hosts
//! - **Firewall-friendly**: Works across subnets with proper routing
//!
//! # Clients
//!
//! Clients join the hub when they connect via TCP (see [`crate::streaming::clients`]).
//! When a client joins or switches transport, the polled one-shot groups
//! (device version) are sent again so it learns them.
//!
//! # Wire Format
//!
//! Each UDP datagram uses the same format as TCP for client compatibility:
//!
//! ```text
//! ┌──────────────────┬──────────────────────┐
//! │ Length (4 bytes) │ Protobuf SensorGroup │
//! │ Big-endian u32   │ (~150 bytes typical) │
//! └──────────────────┴──────────────────────┘
//! ```
//!
//! # Performance Characteristics
//!
//! - **Buffer reuse**: Pre-allocated send buffer avoids allocation per packet
//! - **Non-blocking**: Channel draining and mutex polling are non-blocking
//! - **Rate limiting**: 500μs minimum sleep between packets to yield CPU
//! - **Graceful degradation**: Packet loss is logged but doesn't block sender
//!
//! # Example Packet Rates
//!
//! At default settings:
//! - sensor_status: ~110 packets/sec (~16.5 KB/s)
//! - lidar: ~5 packets/sec (~10 KB/s with 360 points)
//! - device_version: 1 packet (one-time)

use crate::core::types::{SensorGroupData, StreamReceiver};
use crate::error::Result;
use crate::streaming::clients::ClientHub;
use crate::streaming::wire::Serializer;
use crossbeam_channel::TryRecvError;
use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Maximum expected UDP payload size (4-byte length prefix + protobuf message)
/// Typical sensor message ~150 bytes, lidar ~2KB, allow headroom
const MAX_UDP_BUFFER_SIZE: usize = 4096;

/// Small, static groups resent periodically even when unchanged, so a client
/// that reconnects (or lost the one datagram) still learns them.
const REPUBLISHED_GROUPS: &[&str] = &["kinematics"];
const REPUBLISH_INTERVAL: Duration = Duration::from_secs(2);

/// Publisher that fans sensor data out to every connected client
pub struct TelemetryPublisher {
    socket: UdpSocket,
    serializer: Serializer,
    /// Mutex-based sensor data for low-rate polling (lidar, version)
    sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
    /// Channel-based receivers for high-rate streaming (sensor_status @ 110Hz)
    stream_receivers: HashMap<String, StreamReceiver>,
    /// Global running flag (daemon shutdown)
    running: Arc<AtomicBool>,
    /// Connected clients (each with its own UDP target or TCP queue)
    hub: Arc<ClientHub>,
}

impl TelemetryPublisher {
    /// Create a new publisher
    pub fn new(
        socket: UdpSocket,
        serializer: Serializer,
        sensor_data: HashMap<String, Arc<Mutex<SensorGroupData>>>,
        stream_receivers: HashMap<String, StreamReceiver>,
        running: Arc<AtomicBool>,
        hub: Arc<ClientHub>,
    ) -> Self {
        Self {
            socket,
            serializer,
            sensor_data,
            stream_receivers,
            running,
            hub,
        }
    }

    /// Main publishing loop
    pub fn run(&mut self) -> Result<()> {
        log::info!("Telemetry publisher started (fan-out to all connected clients)");

        let mut send_buffer = Vec::with_capacity(MAX_UDP_BUFFER_SIZE);

        // Track last sent sequence for polled groups (non-streaming)
        let mut last_seq: HashMap<String, u64> = HashMap::new();
        for group_id in self.sensor_data.keys() {
            if !self.stream_receivers.contains_key(group_id) {
                last_seq.insert(group_id.clone(), 0);
            }
        }

        // Hub generation: when a client joins or switches transport, the polled
        // groups (version, lidar) are resent; the others just see them once more.
        let mut generation = u64::MAX;
        let mut clients = Vec::new();
        let mut outgoing: Vec<SensorGroupData> = Vec::new();
        let mut udp_sent: Vec<SocketAddr> = Vec::new();
        let mut last_republish = Instant::now();

        while self.running.load(Ordering::Relaxed) {
            // Refresh the client snapshot only when the hub changed (this loop
            // runs ~1 kHz). Generation first: the hub changes the set, then
            // bumps it, so a bump seen here comes with the change.
            let current_generation = self.hub.generation();
            if current_generation != generation {
                clients = self.hub.clients();
            }
            if clients.is_empty() {
                // Drain streaming channels so stale telemetry isn't sent on connect
                for rx in self.stream_receivers.values() {
                    while rx.try_recv().is_ok() {}
                }
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
            if current_generation != generation {
                generation = current_generation;
                for seq in last_seq.values_mut() {
                    *seq = 0;
                }
            }

            if last_republish.elapsed() >= REPUBLISH_INTERVAL {
                for group_id in REPUBLISHED_GROUPS {
                    if let Some(seq) = last_seq.get_mut(*group_id) {
                        *seq = 0;
                    }
                }
                last_republish = Instant::now();
            }

            outgoing.clear();

            // Streaming groups: drain every queued packet
            for (group_id, rx) in &self.stream_receivers {
                loop {
                    match rx.try_recv() {
                        Ok(data) => outgoing.push(data),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => {
                            log::warn!("Stream channel for {} disconnected", group_id);
                            break;
                        }
                    }
                }
            }

            // Polled groups: send when the sequence number changed
            for (group_id, data_mutex) in &self.sensor_data {
                if self.stream_receivers.contains_key(group_id) {
                    continue;
                }
                let Ok(data) = data_mutex.lock() else {
                    log::error!("Failed to lock sensor data for {}", group_id);
                    continue;
                };
                let last = last_seq.get(group_id).copied().unwrap_or(0);
                if data.sequence_number != last {
                    let cloned = data.clone();
                    drop(data); // Release lock before serializing
                    if let Some(seq) = last_seq.get_mut(group_id) {
                        *seq = cloned.sequence_number;
                    }
                    outgoing.push(cloned);
                }
            }


            for data in &outgoing {
                match self.serializer.serialize_sensor_group(data) {
                    Ok(payload) => {
                        send_buffer.clear();
                        send_buffer.extend_from_slice(&(payload.len() as u32).to_be_bytes());
                        send_buffer.extend_from_slice(&payload);
                        // One datagram per distinct UDP target, one shared frame
                        // for all TCP clients.
                        let mut shared = None;
                        udp_sent.clear();
                        for c in &clients {
                            c.deliver(&send_buffer, &mut shared, &self.socket, &mut udp_sent);
                        }
                    }
                    Err(e) => log::warn!("Failed to serialize {}: {}", data.group_id, e),
                }
            }

            if outgoing.is_empty() {
                std::thread::sleep(Duration::from_millis(1));
            } else {
                std::thread::sleep(Duration::from_micros(500));
            }
        }

        log::info!("Telemetry publisher stopped");
        Ok(())
    }
}
