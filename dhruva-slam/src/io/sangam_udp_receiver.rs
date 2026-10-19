//! UDP receiver for SangamIO sensor data.
//!
//! Receives sensor data from SangamIO via UDP unicast and distributes it
//! to processing threads via crossbeam channels.
//!
//! # Wire Protocol
//!
//! Same as TCP for client compatibility:
//! ```text
//! ┌──────────────────┬──────────────────────────┐
//! │ Length (4 bytes) │ Payload (variable)       │
//! │ Big-endian u32   │ Protobuf binary          │
//! └──────────────────┴──────────────────────────┘
//! ```
//!
//! Note: Some fields are defined for planned IMU integration.
//!
//! # Message Types
//!
//! - `sensor_status` @ 110Hz: Encoder ticks, gyro, bumpers, cliffs
//! - `lidar` @ 5Hz: Point cloud scans
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::io::sangam_udp_receiver::{SangamUdpReceiver, ReceiverConfig};
//! use std::sync::Arc;
//! use std::sync::atomic::AtomicBool;
//!
//! let config = ReceiverConfig {
//!     bind_addr: "0.0.0.0:5555".to_string(),
//! };
//! let running = Arc::new(AtomicBool::new(true));
//! let (receiver, sensor_rx, lidar_rx) = SangamUdpReceiver::new(config, running)?;
//!
//! // Spawn receiver thread
//! std::thread::spawn(move || receiver.run());
//!
//! // Process sensor data in another thread
//! while let Ok(msg) = sensor_rx.recv() {
//!     println!("Got sensor data: {:?}", msg.timestamp_us);
//! }
//! ```

use crossbeam_channel::{Receiver, Sender, bounded};
use prost::Message as ProstMessage;
use socket2::{Domain, Protocol, Socket, Type};
use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use thiserror::Error;

// Re-use proto types from sangam_client
use crate::io::sangam_client::proto::sangamio;

/// Receiver errors
#[derive(Error, Debug)]
pub enum ReceiverError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Decode error: {0}")]
    Decode(String),
}

impl From<prost::DecodeError> for ReceiverError {
    fn from(e: prost::DecodeError) -> Self {
        ReceiverError::Decode(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, ReceiverError>;

/// Configuration for UDP receiver.
#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    /// Address to bind UDP socket (e.g., "0.0.0.0:5555").
    pub bind_addr: String,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            bind_addr: "0.0.0.0:5555".to_string(),
        }
    }
}

/// Sensor status message extracted from protobuf.
#[derive(Debug, Clone)]
pub struct SensorStatusData {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Left wheel encoder ticks.
    pub left_encoder: u16,
    /// Right wheel encoder ticks.
    pub right_encoder: u16,
    /// Gyro Z (yaw rate) raw value in 0.1 deg/s.
    pub gyro_yaw_raw: i16,
    /// Left bumper pressed.
    pub bumper_left: bool,
    /// Right bumper pressed.
    pub bumper_right: bool,
    /// Cliff sensor: left side.
    pub cliff_left_side: bool,
    /// Cliff sensor: left front.
    pub cliff_left_front: bool,
    /// Cliff sensor: right front.
    pub cliff_right_front: bool,
    /// Cliff sensor: right side.
    pub cliff_right_side: bool,
    /// Charging (on the dock, or the DC jack).
    pub is_charging: bool,
    /// On the dock's contacts.
    pub is_docked: bool,
}

/// A single LiDAR point: (angle_rad, distance_m, quality).
pub type LidarPoint = (f32, f32, u8);

/// Decode SangamIO's compact `scan_packed` blob (kaiaai format):
/// `u16 num_bins` then per bin `u16 distance (0.25 mm units, 0 = no return), u8 quality`,
/// all little-endian; bin i is at angle `i * 2pi / num_bins`. Empty bins are dropped.
pub fn unpack_scan(blob: &[u8]) -> Option<Vec<LidarPoint>> {
    if blob.len() < 2 {
        return None;
    }
    let n = u16::from_le_bytes([blob[0], blob[1]]) as usize;
    if n == 0 || blob.len() < 2 + 3 * n {
        return None;
    }
    let step = std::f32::consts::TAU / n as f32;
    Some(
        (0..n)
            .filter_map(|i| {
                let o = 2 + 3 * i;
                let raw = u16::from_le_bytes([blob[o], blob[o + 1]]);
                (raw > 0).then(|| (i as f32 * step, raw as f32 * 0.00025, blob[o + 2]))
            })
            .collect(),
    )
}

/// LiDAR scan message.
#[derive(Debug, Clone)]
pub struct LidarData {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// Point cloud: Vec<(angle_rad, distance_m, quality)>.
    pub points: Vec<LidarPoint>,
}

/// A sensor sample for the SLAM pipeline, decoded from one SangamIO message: the live
/// stream, a recording and the simulator all go through [`Sample::decode`].
#[derive(Debug, Clone)]
pub enum Sample {
    /// Wheel encoders, gyro, bumpers, cliffs (the `sensor_status` group).
    Status(SensorStatusData),
    /// One lidar revolution (the `lidar` group).
    Lidar(LidarData),
}

impl Sample {
    /// Decode a protobuf `Message` (without the length prefix). `Ok(None)` for messages
    /// SLAM does not use (commands, other sensor groups); an error for malformed ones.
    pub fn decode(body: &[u8]) -> Result<Option<Sample>> {
        let msg = sangamio::Message::decode(body)?;
        let Some(sangamio::message::Payload::SensorGroup(sg)) = msg.payload else {
            return Ok(None);
        };
        let sample = match sg.group_id.as_str() {
            "sensor_status" => SangamUdpReceiver::parse_sensor_status(&sg).map(Sample::Status),
            "lidar" => SangamUdpReceiver::parse_lidar(&sg).map(Sample::Lidar),
            _ => return Ok(None),
        };
        let Some(sample) = sample else {
            return Err(ReceiverError::Decode(format!(
                "malformed {} group at {} us",
                sg.group_id, sg.timestamp_us
            )));
        };
        Ok(Some(sample))
    }

    /// Data time of the sample.
    pub fn timestamp_us(&self) -> u64 {
        match self {
            Sample::Status(d) => d.timestamp_us,
            Sample::Lidar(d) => d.timestamp_us,
        }
    }
}

/// Channel capacity for sensor data. Era 8: com o sensor a ~323Hz e o SLAM thread
/// ocupado no scan matching, o canal enchia e o try_send do receiver DROPAVA a
/// odometria (odom_delta ~0). Subido bastante p/ absorver o burst sem perder.
const SENSOR_CHANNEL_CAPACITY: usize = 256;

/// Channel capacity for lidar data.
const LIDAR_CHANNEL_CAPACITY: usize = 4;

/// Maximum UDP datagram size (64KB for fragmented packets).
const MAX_DATAGRAM_SIZE: usize = 65536;

/// UDP receiver for SangamIO sensor data.
pub struct SangamUdpReceiver {
    socket: UdpSocket,
    running: Arc<AtomicBool>,
    sensor_tx: Sender<SensorStatusData>,
    lidar_tx: Sender<LidarData>,
}

impl SangamUdpReceiver {
    /// Create a new UDP receiver.
    ///
    /// Returns the receiver and channels for sensor/lidar data.
    pub fn new(
        config: ReceiverConfig,
        running: Arc<AtomicBool>,
    ) -> Result<(Self, Receiver<SensorStatusData>, Receiver<LidarData>)> {
        // Use socket2 to set SO_REUSEADDR before binding
        // This is needed for localhost testing where TCP and UDP share the same port
        let addr: SocketAddr = config.bind_addr.parse().map_err(|e| {
            ReceiverError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Invalid bind address: {}", e),
            ))
        })?;

        let socket2 = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        socket2.set_reuse_address(true)?;
        #[cfg(target_os = "macos")]
        socket2.set_reuse_port(true)?;
        socket2.bind(&addr.into())?;

        // Convert to std UdpSocket
        let socket: UdpSocket = socket2.into();

        // Set non-blocking with short timeout for shutdown checks
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;

        // Create channels for sensor data
        let (sensor_tx, sensor_rx) = bounded(SENSOR_CHANNEL_CAPACITY);
        let (lidar_tx, lidar_rx) = bounded(LIDAR_CHANNEL_CAPACITY);

        log::info!(
            "UDP receiver bound to {} (SO_REUSEADDR enabled)",
            config.bind_addr
        );

        Ok((
            Self {
                socket,
                running,
                sensor_tx,
                lidar_tx,
            },
            sensor_rx,
            lidar_rx,
        ))
    }

    /// Run the receiver loop (blocking).
    ///
    /// Receives UDP datagrams, parses them, and sends to appropriate channels.
    pub fn run(self) {
        log::info!("UDP receiver started");

        let mut buffer = vec![0u8; MAX_DATAGRAM_SIZE];
        let mut packet_count: u64 = 0;
        let mut last_log_time = std::time::Instant::now();
        // Malformed messages in this 5 s window, and the last one's error: reported once
        // per window, not at 110 Hz.
        let mut malformed: (u64, Option<ReceiverError>) = (0, None);

        while self.running.load(Ordering::Relaxed) {
            // Receive UDP datagram
            let (len, _src) = match self.socket.recv_from(&mut buffer) {
                Ok(result) => result,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => {
                    log::warn!("UDP recv error: {}", e);
                    continue;
                }
            };

            // Track reception rate (debug level - periodic telemetry)
            packet_count += 1;
            if last_log_time.elapsed().as_secs() >= 5 {
                log::debug!(
                    "UDP receiver: {} packets in last 5s ({})",
                    packet_count,
                    _src
                );
                if let (n, Some(e)) = std::mem::take(&mut malformed) {
                    log::warn!(
                        "UDP receiver: dropped {n} malformed messages in 5 s, the last: {e}"
                    );
                }
                packet_count = 0;
                last_log_time = std::time::Instant::now();
            }

            // Parse length-prefixed message
            if len < 4 {
                log::warn!("UDP datagram too short: {} bytes", len);
                continue;
            }

            let msg_len = u32::from_be_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]) as usize;
            if msg_len + 4 > len {
                log::warn!(
                    "UDP message incomplete: expected {} + 4 bytes, got {}",
                    msg_len,
                    len
                );
                continue;
            }

            // Non-blocking sends: when a channel is full, the sample is dropped (old data).
            match Sample::decode(&buffer[4..4 + msg_len]) {
                Ok(Some(Sample::Status(data))) => {
                    self.sensor_tx.try_send(data).ok();
                }
                Ok(Some(Sample::Lidar(data))) => {
                    log::debug!("Lidar scan received: {} points", data.points.len());
                    self.lidar_tx.try_send(data).ok();
                }
                Ok(None) => {}
                Err(e) => malformed = (malformed.0 + 1, Some(e)),
            }
        }

        log::info!("UDP receiver stopped");
    }

    /// Parse sensor_status group into SensorStatusData.
    fn parse_sensor_status(sg: &sangamio::SensorGroup) -> Option<SensorStatusData> {
        let values = &sg.values;

        let left_encoder = Self::get_u32(values, "wheel_left")? as u16;
        let right_encoder = Self::get_u32(values, "wheel_right")? as u16;
        let gyro_yaw_raw = Self::get_i32(values, "gyro_z").unwrap_or(0) as i16;

        Some(SensorStatusData {
            timestamp_us: sg.timestamp_us,
            left_encoder,
            right_encoder,
            gyro_yaw_raw,
            bumper_left: Self::get_bool(values, "bumper_left").unwrap_or(false),
            bumper_right: Self::get_bool(values, "bumper_right").unwrap_or(false),
            cliff_left_side: Self::get_bool(values, "cliff_left_side").unwrap_or(false),
            cliff_left_front: Self::get_bool(values, "cliff_left_front").unwrap_or(false),
            cliff_right_front: Self::get_bool(values, "cliff_right_front").unwrap_or(false),
            cliff_right_side: Self::get_bool(values, "cliff_right_side").unwrap_or(false),
            is_charging: Self::get_bool(values, "is_charging").unwrap_or(false),
            is_docked: Self::get_bool(values, "is_dock_connected").unwrap_or(false),
        })
    }

    /// Parse lidar group into LidarData.
    fn parse_lidar(sg: &sangamio::SensorGroup) -> Option<LidarData> {
        let points: Vec<LidarPoint> = if let Some(scan) = sg.values.get("scan") {
            match &scan.value {
                Some(sangamio::sensor_value::Value::PointcloudVal(pc)) => pc
                    .points
                    .iter()
                    .map(|p| (p.angle_rad, p.distance_m, p.quality as u8))
                    .collect(),
                _ => return None,
            }
        } else {
            match &sg.values.get("scan_packed")?.value {
                Some(sangamio::sensor_value::Value::BytesVal(b)) => unpack_scan(b)?,
                _ => return None,
            }
        };

        Some(LidarData {
            timestamp_us: sg.timestamp_us,
            points,
        })
    }

    /// Helper to extract u32 from sensor value map.
    fn get_u32(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<u32> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::U32Val(v) => Some(*v),
            _ => None,
        }
    }

    /// Helper to extract i32 from sensor value map.
    fn get_i32(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<i32> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::I32Val(v) => Some(*v),
            _ => None,
        }
    }

    /// Helper to extract bool from sensor value map.
    fn get_bool(values: &HashMap<String, sangamio::SensorValue>, key: &str) -> Option<bool> {
        match values.get(key)?.value.as_ref()? {
            sangamio::sensor_value::Value::BoolVal(v) => Some(*v),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn group(id: &str) -> Vec<u8> {
        sangamio::Message {
            topic: format!("sensors/{id}"),
            payload: Some(sangamio::message::Payload::SensorGroup(
                sangamio::SensorGroup {
                    group_id: id.into(),
                    timestamp_us: 7,
                    values: HashMap::new(),
                },
            )),
        }
        .encode_to_vec()
    }

    #[test]
    fn decode_skips_unused_groups_and_rejects_broken_ones() {
        assert!(matches!(Sample::decode(&group("kinematics")), Ok(None)));
        assert!(
            Sample::decode(&group("lidar")).is_err(),
            "a lidar group without a scan"
        );
        assert!(
            Sample::decode(&group("sensor_status")).is_err(),
            "no encoders"
        );
        assert!(Sample::decode(b"not protobuf").is_err());
    }
}
