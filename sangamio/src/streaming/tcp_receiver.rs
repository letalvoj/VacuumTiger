//! TCP command receiver for client control
//!
//! This module handles incoming commands from connected clients over TCP.
//! Commands include robot motion control, actuator control, and component
//! enable/disable operations.
//!
//! # Purpose
//!
//! TCP is used for commands (not UDP) because:
//!
//! - **Reliability**: Commands must not be lost (e.g., "stop motors")
//! - **Ordering**: Commands must execute in sequence
//! - **No replies**: the protocol has no acknowledgements; refusals are logged
//! - **State sync**: TCP connection state tracks client presence
//!
//! # Wire Format
//!
//! Commands use length-prefixed Protobuf encoding:
//!
//! ```text
//! ┌──────────────────┬─────────────────────┐
//! │ Length (4 bytes) │ Protobuf Command    │
//! │ Big-endian u32   │ (variable size)     │
//! └──────────────────┴─────────────────────┘
//! ```
//!
//! # Command Types
//!
//! | Command | Description |
//! |---------|-------------|
//! | `ComponentControl::drive` | Set linear/angular velocity |
//! | `ComponentControl::lidar` | Enable/disable lidar motor |
//! | `ComponentControl::vacuum` | Control vacuum motor speed |
//! | `ComponentControl::main_brush` | Control main brush speed |
//! | `ComponentControl::side_brush` | Control side brush speed |
//! | `ComponentControl::water_pump` | Control water pump speed |
//! | `ComponentControl::led` | Set LED state |
//! | `Shutdown` | Graceful daemon shutdown |
//! | `ComponentControl::lease` | Take (Enable, `force`) / release (Disable) the control lease |
//! | `ComponentControl::telemetry` | Switch this client's telemetry to TCP (Enable) / UDP |
//!
//! Only the lease holder may drive or switch actuators; stops are accepted
//! from anyone (see [`crate::streaming::clients`]). Refused commands are logged.
//!
//! # Connection Lifecycle
//!
//! ```text
//! 1. Client connects to TCP port 5555
//! 2. Server spawns TcpReceiver thread for this client
//! 3. The client is registered in the ClientHub (telemetry fan-out)
//! 4. Receiver loop processes commands until disconnect
//! 5. On disconnect it leaves the hub; if it held the lease, the drive is zeroed
//! ```
//!
//! # Safety Features
//!
//! - **Read timeout**: 500ms timeout allows periodic shutdown flag checks
//! - **Buffer limit**: Commands > 1MB are rejected (DoS protection)
//! - **Graceful shutdown**: Handles both global and per-connection flags

use crate::core::driver::DeviceDriver;
use crate::core::types::{Command, ComponentAction};
use crate::error::{Error, Result};
use crate::streaming::clients::{Client, ClientHub, Verdict, is_stop};
use crate::streaming::wire::Serializer;
use std::io::Read;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// TCP receiver that handles commands from connected client
pub struct TcpReceiver {
    serializer: Serializer,
    driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
    /// Global running flag (daemon shutdown)
    running: Arc<AtomicBool>,
    /// All clients and the control lease
    hub: Arc<ClientHub>,
    /// This connection's entry in the hub
    client: Arc<Client>,
    /// Reusable buffer for reading command payloads (avoids allocation per command)
    read_buffer: Vec<u8>,
}

/// Initial capacity for command read buffer (typical command size)
const INITIAL_BUFFER_CAPACITY: usize = 256;

/// A partir de quanto tempo uma chamada de handle_command e considerada um BLOQUEIO do
/// canal de comando (o `lidar enable` leva 17-25 s: rail-off 10 s + boot 12 s + spin-up).
const BUSY_WARN_MS: u128 = 1000;

// NOTA (05/10/2026, medido no metal): existiu aqui um guard "STALE-DRIVE" que DESCARTAVA
// comando de drive que chegava logo apos um bloqueio do canal. Ele foi REMOVIDO porque
// descartou o `drive enable` de um cliente LEGITIMO: o driver de LiDAR publica **scan
// PARCIAL** no timeout do sensor, o cliente ve "chegou scan", manda a tracao — e o comando
// cai dentro da janela. O conserto e do lado do cliente: criterio de prontidao = TAXA de
// scan COMPLETO (ver `map-odom.py --lidar-rate/--lidar-min-points`). O daemon so LOGA.

/// Ha byte esperando no socket? (peek nao consome)
pub fn has_pending(stream: &TcpStream) -> bool {
    let mut probe = [0u8; 1];
    matches!(stream.peek(&mut probe), Ok(n) if n > 0)
}

impl TcpReceiver {
    /// Create a new TCP receiver
    pub fn new(
        serializer: Serializer,
        driver: Arc<Mutex<Box<dyn DeviceDriver>>>,
        running: Arc<AtomicBool>,
        hub: Arc<ClientHub>,
        client: Arc<Client>,
    ) -> Self {
        Self {
            serializer,
            driver,
            running,
            hub,
            client,
            // Pre-allocate buffer to avoid allocation on first command
            read_buffer: Vec::with_capacity(INITIAL_BUFFER_CAPACITY),
        }
    }

    /// Run the receiver loop for a connected client
    pub fn run(&mut self, mut stream: TcpStream) -> Result<()> {
        log::info!("TCP receiver started for client: {:?}", stream.peer_addr());

        // Set read timeout so we can check shutdown flag
        if let Err(e) = stream.set_read_timeout(Some(std::time::Duration::from_millis(500))) {
            log::warn!("Failed to set read timeout: {}", e);
        }

        log::debug!("Entering receiver loop");

        loop {
            // Check both global running flag and per-connection alive flag
            if !self.running.load(Ordering::Relaxed) {
                log::debug!("Running flag cleared, exiting");
                break;
            }
            if !self.client.alive.load(Ordering::Relaxed) {
                log::debug!("Connection alive flag cleared, exiting");
                break;
            }

            match self.read_command(&mut stream) {
                Ok(Some(cmd)) => {
                    log::debug!("Received command: {:?}", cmd);
                    let label = describe(&cmd);
                    let t0 = Instant::now();
                    let result = self.handle_command(cmd);
                    let busy_ms = t0.elapsed().as_millis();
                    if busy_ms >= BUSY_WARN_MS {
                        let pending = has_pending(&stream);
                        log::warn!(
                            "canal de comando BLOQUEADO {} ms por '{}' — {}",
                            busy_ms,
                            label,
                            if pending {
                                "havia comando do cliente JA no buffer ao fim do bloqueio: ele \
                                 sera executado AGORA, em ordem. Se era comando de drive, pode nao \
                                 engatar (foi mandado pensando que o LiDAR ja estava pronto). \
                                 ATENCAO: o driver publica scan PARCIAL no timeout do sensor, entao \
                                 o cliente pode ter sido ENGANADO por um scan parcial — o criterio \
                                 de prontidao tem que ser TAXA de scan COMPLETO, nao 'chegou scan' \
                                 (medido no metal 05/10). NAO descartamos o comando de proposito: \
                                 um guard que descartava matou um cliente legitimo nessa mesma data."
                            } else {
                                "nada pendente no socket (cliente esperou — ok)"
                            }
                        );
                    }
                    if let Err(e) = result {
                        log::warn!("Failed to handle command: {}", e);
                    }
                }
                Ok(None) => {
                    // Timeout or non-command message, continue loop
                }
                Err(e) => {
                    // Signal connection is dead and shutdown socket
                    self.client.alive.store(false, Ordering::Relaxed);
                    let _ = stream.shutdown(std::net::Shutdown::Both);

                    // Check if it's a connection closed error
                    if let Error::Io(ref io_err) = e
                        && (io_err.kind() == std::io::ErrorKind::UnexpectedEof
                            || io_err.kind() == std::io::ErrorKind::ConnectionReset
                            || io_err.kind() == std::io::ErrorKind::TimedOut)
                    {
                        log::info!("Client disconnected");
                        return Ok(());
                    }
                    log::warn!("Failed to read message: {}", e);
                    return Err(e);
                }
            }
        }

        // Clean shutdown: signal connection dead and close socket
        self.client.alive.store(false, Ordering::Relaxed);
        let _ = stream.shutdown(std::net::Shutdown::Both);

        log::info!("TCP receiver stopped");
        Ok(())
    }

    /// Read a command from the client
    ///
    /// Uses a reusable internal buffer to avoid allocation per command.
    fn read_command(&mut self, stream: &mut TcpStream) -> Result<Option<Command>> {
        // Read length prefix
        let mut len_buf = [0u8; 4];
        match stream.read_exact(&mut len_buf) {
            Ok(_) => {
                log::trace!("Read length prefix: {:?}", len_buf);
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
            // SO_RCVTIMEO expiry is WouldBlock on Unix; TimedOut here means the
            // connection died (keepalive / TCP_USER_TIMEOUT): fall through to error.
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                log::trace!("EOF on length read");
                return Err(Error::Io(e));
            }
            Err(e) => {
                log::trace!("Error reading length: {:?}", e.kind());
                return Err(Error::Io(e));
            }
        }

        let len = u32::from_be_bytes(len_buf) as usize;

        // Sanity check on length
        if len > 1024 * 1024 {
            return Err(Error::Other(format!("Message too large: {} bytes", len)));
        }

        // Reuse buffer - resize only if needed (no allocation if capacity sufficient)
        self.read_buffer.clear();
        self.read_buffer.resize(len, 0);
        stream.read_exact(&mut self.read_buffer)?;

        // Deserialize command
        self.serializer.deserialize_command(&self.read_buffer)
    }

    /// Handle a command
    fn handle_command(&self, cmd: Command) -> Result<()> {
        // Lease policy, plus the reserved pseudo-components "telemetry" (switch
        // this client's stream to TCP/UDP) and "lease" (take/release control).
        // The drive is zeroed whenever the lease changes hands.
        match self.hub.authorize(&self.client, &cmd, || stop_drive(&self.driver)) {
            Verdict::Execute => {}
            Verdict::Handled => return Ok(()),
            Verdict::Refused(holder) => {
                log::warn!(
                    "Client #{} refused '{}': control lease held by #{}",
                    self.client.id,
                    describe(&cmd),
                    holder
                );
                return Ok(());
            }
        }

        log::trace!("Executing command: {:?}", cmd);
        let mut driver = self.driver.lock().map_err(|_| Error::ThreadPanic)?;
        // The lease may have changed hands while we waited for the driver
        // (a forced takeover zeroes the drive first): don't undo that.
        if !is_stop_command(&cmd) && self.hub.lease_holder() != Some(self.client.id) {
            log::warn!(
                "Client #{} lost the lease before '{}' ran: dropped",
                self.client.id,
                describe(&cmd)
            );
            return Ok(());
        }
        let result = driver.send_command(cmd);
        if result.is_err() {
            log::warn!("Command execution failed: {:?}", result);
        }
        result
    }
}

fn is_stop_command(cmd: &Command) -> bool {
    matches!(cmd, Command::ComponentControl { id, action } if is_stop(id, action))
}

/// Zero and disable the drive. The GD32 holds the last velocity forever, so
/// this runs whenever the controlling client changes or goes away.
pub fn stop_drive(driver: &Mutex<Box<dyn DeviceDriver>>) {
    let mut drv = driver.lock().unwrap_or_else(|e| e.into_inner());
    if let Err(e) = drv.send_command(Command::ComponentControl {
        id: "drive".to_string(),
        action: ComponentAction::Disable { config: None },
    }) {
        log::warn!("Failed to stop the drive: {}", e);
    }
}

/// Rotulo curto do comando para o log ("drive/enable", "lidar/Disable", ...).
pub fn describe(cmd: &Command) -> String {
    match cmd {
        Command::ComponentControl { id, action } => format!("{}/{:?}", id, action),
        other => format!("{:?}", other),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn describe_traz_componente_e_acao() {
        let cmd = Command::ComponentControl {
            id: "drive".to_string(),
            action: crate::core::types::ComponentAction::Disable { config: None },
        };
        let s = describe(&cmd);
        assert!(s.starts_with("drive/"), "rotulo inesperado: {}", s);
        assert!(describe(&Command::ProtocolSync).contains("ProtocolSync"));
    }
}
