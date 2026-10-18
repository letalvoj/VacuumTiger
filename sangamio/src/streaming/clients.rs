//! Connected clients and the control lease.
//!
//! SangamIO serves several clients at once (DhruvaSLAM, a web UI, a recorder,
//! a debugger on a laptop). Every client gets the full telemetry stream; only
//! one of them, the **lease holder**, may drive the robot or switch actuators.
//!
//! # Telemetry
//!
//! A client starts on UDP to `<its IP>:<udp_port>`. Two clients on one host
//! share that port (one datagram per distinct target), so a second local
//! client should switch to TCP telemetry (frames written into its own command
//! socket) with the reserved command `ComponentControl { id: "telemetry",
//! Enable }` (`Disable` switches back). TCP frames go through a bounded
//! per-client queue drained by a writer thread, so a slow client only loses
//! its own frames.
//!
//! # Control lease
//!
//! | Command | Who may send it |
//! |---------|-----------------|
//! | `lease` Enable (config `force: true` to take it over) / Disable | anyone |
//! | `telemetry` Enable/Disable | anyone (affects only the sender) |
//! | stop: `drive` Disable/Reset, actuator Disable; `lidar` | anyone, always |
//! | cleaning actuators, LED | anyone while the lease is free (not taken), else the holder |
//! | everything else (`drive`, calibration, power, `Shutdown`, ...) | the holder; taken implicitly when free |
//!
//! A client that drives while nobody holds the lease gets it implicitly, so
//! single-client tools keep working unchanged. Whenever the lease changes hands
//! (release, forced takeover, holder disconnect) the drive is zeroed: the GD32
//! holds the last velocity forever, and a new holder must not inherit motion.
//! The holder changes before the stop, and the receiver re-checks the lease
//! once it has the driver, so a command authorized before a takeover is
//! dropped instead of running after the stop.
//!
//! Limits: stops still queue behind a driver command in progress (a lidar
//! enable blocks the driver for ~20 s); the GD32 heartbeat dead-man is the
//! backstop. There is no authentication: anyone on the LAN can `force` the
//! lease. Refused commands are logged; the protocol has no replies.

use crate::core::types::{Command, ComponentAction, SensorValue};
use crate::streaming::Transport;
use crossbeam_channel::{Sender, TrySendError, bounded};
use std::io::Write;
use std::net::{SocketAddr, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

/// Frames queued per TCP-telemetry client before new frames are dropped
/// (~0.3 s of telemetry at 110 Hz).
const TCP_QUEUE_FRAMES: usize = 32;

/// Components whose `Disable` counts as a stop and is accepted from anyone.
const STOPPABLE: &[&str] = &["drive", "vacuum", "main_brush", "side_brush", "water_pump"];

/// Actuators any client may switch while the lease is free.
const UNLEASED_ACTUATORS: &[&str] = &["vacuum", "main_brush", "side_brush", "water_pump", "led"];

/// Client ids start at 1; 0 means "no lease holder".
pub type ClientId = u64;
const NOBODY: ClientId = 0;

/// A shared telemetry frame (length prefix + protobuf).
pub type Frame = Arc<[u8]>;

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// A client's TCP telemetry writer thread.
struct TcpWriter {
    tx: Sender<Frame>,
    /// Set when the client goes back to UDP (dropping `tx` alone would let
    /// the thread drain its whole queue first).
    cancel: Arc<AtomicBool>,
    thread: std::thread::JoinHandle<()>,
}

#[derive(Default)]
struct TcpTelemetry {
    active: Option<TcpWriter>,
    /// The previous writer, joined before a new one starts.
    retired: Option<std::thread::JoinHandle<()>>,
}

/// One connected client.
pub struct Client {
    pub id: ClientId,
    pub peer: SocketAddr,
    /// UDP telemetry target (peer IP, configured UDP port).
    pub udp_addr: SocketAddr,
    /// Cleared to make this client's receiver exit (eviction, write failure).
    pub alive: Arc<AtomicBool>,
    /// Write-clone of the command socket, cloned again for each TCP switch.
    socket: Option<TcpStream>,
    /// The TCP telemetry writer, while telemetry goes over TCP.
    tcp: Mutex<TcpTelemetry>,
}

impl Client {
    pub fn transport(&self) -> Transport {
        if lock(&self.tcp).active.is_some() {
            Transport::Tcp
        } else {
            Transport::Udp
        }
    }

    /// Deliver one serialized frame: queue it if this client is on TCP
    /// (non-blocking, dropped when the queue is full), else send a datagram
    /// unless `udp_sent` shows its target already got this frame. `shared`
    /// holds the frame as an `Arc` once, for all TCP clients.
    pub fn deliver(
        &self,
        buf: &[u8],
        shared: &mut Option<Frame>,
        udp: &UdpSocket,
        udp_sent: &mut Vec<SocketAddr>,
    ) {
        if let Some(TcpWriter { tx, .. }) = lock(&self.tcp).active.as_ref() {
            let frame = shared.get_or_insert_with(|| Arc::from(buf));
            if let Err(TrySendError::Full(_)) = tx.try_send(Arc::clone(frame)) {
                log::trace!(
                    "Client #{}: TCP telemetry queue full, frame dropped",
                    self.id
                );
            }
        } else if !udp_sent.contains(&self.udp_addr) {
            // Packet loss is acceptable for real-time data
            if let Err(e) = udp.send_to(buf, self.udp_addr) {
                log::trace!("UDP send to {} failed: {}", self.udp_addr, e);
            }
            udp_sent.push(self.udp_addr);
        }
    }
}

/// Outcome of the lease check for one command.
#[derive(Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Forward to the driver.
    Execute,
    /// Handled by the hub itself (lease/telemetry), nothing to forward.
    Handled,
    /// Refused: another client holds the lease.
    Refused(ClientId),
}

/// All connected clients plus the lease.
pub struct ClientHub {
    clients: Mutex<Vec<Arc<Client>>>,
    /// Serializes lease transitions (and the drive stop that goes with them).
    lease: Mutex<()>,
    /// The holder, readable without the lock (accept loop, driver re-check).
    holder: AtomicU64,
    next_id: AtomicU64,
    /// Bumped whenever the client set or a transport changes: the publisher
    /// refreshes its snapshot and resends the one-shot groups (device version).
    generation: AtomicU64,
    max_clients: usize,
}

impl ClientHub {
    pub fn new(max_clients: usize) -> Arc<Self> {
        Arc::new(Self {
            clients: Mutex::new(Vec::new()),
            lease: Mutex::new(()),
            holder: AtomicU64::new(NOBODY),
            next_id: AtomicU64::new(1),
            generation: AtomicU64::new(0),
            max_clients: max_clients.max(1),
        })
    }

    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Register a new client. `socket` is a write-clone of its command
    /// socket, used if it switches to TCP telemetry. When the hub is full the
    /// oldest client that doesn't hold the lease is evicted.
    pub fn connect(
        &self,
        peer: SocketAddr,
        udp_port: u16,
        socket: Option<TcpStream>,
    ) -> Arc<Client> {
        let client = Arc::new(Client {
            id: self.next_id.fetch_add(1, Ordering::Relaxed),
            peer,
            udp_addr: SocketAddr::new(peer.ip(), udp_port),
            alive: Arc::new(AtomicBool::new(true)),
            socket,
            tcp: Mutex::default(),
        });
        let mut clients = lock(&self.clients);
        while clients.len() >= self.max_clients {
            let holder = self.holder.load(Ordering::Acquire);
            let pos = clients.iter().position(|c| c.id != holder).unwrap_or(0);
            let old = clients.remove(pos);
            old.alive.store(false, Ordering::Relaxed);
            self.generation.fetch_add(1, Ordering::Release);
            log::warn!(
                "Client limit ({}) reached: evicting client #{} ({})",
                self.max_clients,
                old.id,
                old.peer
            );
        }
        clients.push(Arc::clone(&client));
        log::info!(
            "Client #{} connected from {} ({} connected)",
            client.id,
            peer,
            clients.len()
        );
        drop(clients);
        self.generation.fetch_add(1, Ordering::Release);
        client
    }

    /// Remove a client. If it held the lease, `stop` runs (zero the drive)
    /// *before* the lease is freed, so no new holder's command can land
    /// between the two.
    pub fn disconnect(&self, id: ClientId, stop: impl FnOnce()) {
        {
            let mut clients = lock(&self.clients);
            clients.retain(|c| c.id != id);
            self.generation.fetch_add(1, Ordering::Release);
            log::info!("Client #{} disconnected ({} connected)", id, clients.len());
        }
        let _transition = lock(&self.lease);
        if self.holder.load(Ordering::Acquire) == id {
            // Hand over first: a command of the old holder still waiting for
            // the driver then fails its re-check instead of running after the stop.
            self.holder.store(NOBODY, Ordering::Release);
            stop();
            log::info!("Lease released: holder #{} disconnected", id);
        }
    }

    /// Snapshot of the connected clients.
    pub fn clients(&self) -> Vec<Arc<Client>> {
        lock(&self.clients).clone()
    }

    pub fn lease_holder(&self) -> Option<ClientId> {
        Some(self.holder.load(Ordering::Acquire)).filter(|&h| h != NOBODY)
    }

    /// Decide what to do with a command from `client`. Takes the lease
    /// implicitly when it is free, and handles the `lease`/`telemetry`
    /// pseudo-components. `stop` zeroes the drive when the lease changes hands.
    pub fn authorize(&self, client: &Client, cmd: &Command, stop: impl FnOnce()) -> Verdict {
        let Command::ComponentControl { id, action } = cmd else {
            // ProtocolSync / Shutdown are controlling commands.
            return self.require_lease(client.id);
        };
        match (id.as_str(), action) {
            ("telemetry", ComponentAction::Enable { .. }) => {
                self.set_tcp_telemetry(client, true);
                Verdict::Handled
            }
            ("telemetry", ComponentAction::Disable { .. }) => {
                self.set_tcp_telemetry(client, false);
                Verdict::Handled
            }
            ("lease", ComponentAction::Enable { config }) => {
                let force = config
                    .as_ref()
                    .and_then(|c| c.get("force"))
                    .is_some_and(|v| matches!(v, SensorValue::Bool(true)));
                self.acquire(client.id, force, stop);
                Verdict::Handled
            }
            ("lease", ComponentAction::Disable { .. }) => {
                self.release(client.id, stop);
                Verdict::Handled
            }
            ("lease" | "telemetry", other) => {
                log::warn!(
                    "Client #{}: unsupported {} action {:?}",
                    client.id,
                    id,
                    other
                );
                Verdict::Handled
            }
            _ => match lease_rule(cmd) {
                LeaseRule::Free => Verdict::Execute,
                LeaseRule::IfUnheld => match self.lease_holder() {
                    Some(h) if h != client.id => Verdict::Refused(h),
                    _ => Verdict::Execute,
                },
                LeaseRule::Acquire => self.require_lease(client.id),
            },
        }
    }

    /// Whether `client` may still run `cmd` now (re-checked once the driver is
    /// free: the lease may have changed hands while it waited).
    pub fn still_allowed(&self, client: &Client, cmd: &Command) -> bool {
        match lease_rule(cmd) {
            LeaseRule::Free => true,
            LeaseRule::IfUnheld => self.lease_holder().is_none_or(|h| h == client.id),
            LeaseRule::Acquire => self.lease_holder() == Some(client.id),
        }
    }

    fn require_lease(&self, id: ClientId) -> Verdict {
        let _transition = lock(&self.lease);
        match self.holder.load(Ordering::Acquire) {
            h if h == id => Verdict::Execute,
            NOBODY => {
                self.holder.store(id, Ordering::Release);
                log::info!("Lease taken implicitly by client #{}", id);
                Verdict::Execute
            }
            h => Verdict::Refused(h),
        }
    }

    /// Take the lease. Without `force` this only succeeds when it is free;
    /// a forced takeover zeroes the previous holder's drive first.
    fn acquire(&self, id: ClientId, force: bool, stop: impl FnOnce()) {
        let _transition = lock(&self.lease);
        match self.holder.load(Ordering::Acquire) {
            h if h == id => {}
            h if h != NOBODY && !force => {
                log::info!("Lease request from #{} refused: held by #{}", id, h);
            }
            prev => {
                self.holder.store(id, Ordering::Release);
                if prev != NOBODY {
                    stop();
                }
                log::info!("Lease taken by client #{} (was #{})", id, prev);
            }
        }
    }

    fn release(&self, id: ClientId, stop: impl FnOnce()) {
        let _transition = lock(&self.lease);
        if self.holder.load(Ordering::Acquire) == id {
            self.holder.store(NOBODY, Ordering::Release);
            stop();
            log::info!("Lease released by client #{}", id);
        }
    }

    fn set_tcp_telemetry(&self, client: &Client, on: bool) {
        let mut tcp = lock(&client.tcp);
        if on == tcp.active.is_some() {
            return;
        }
        if on {
            // A writer retired by a previous switch may still be mid-frame:
            // two writers on one socket would interleave frames.
            if let Some(old) = tcp.retired.take() {
                let _ = old.join();
            }
            let Some(sock) = client.socket.as_ref() else {
                log::warn!("Client #{}: no socket for TCP telemetry", client.id);
                return;
            };
            match sock
                .try_clone()
                .and_then(|w| spawn_writer(client.id, w, Arc::clone(&client.alive)))
            {
                Ok(writer) => tcp.active = Some(writer),
                Err(e) => {
                    log::error!("Client #{}: TCP telemetry writer: {}", client.id, e);
                    return;
                }
            }
        } else {
            // The writer finishes its current frame at most, then exits.
            if let Some(w) = tcp.active.take() {
                w.cancel.store(true, Ordering::Relaxed);
                tcp.retired = Some(w.thread);
            }
        }
        drop(tcp);
        self.generation.fetch_add(1, Ordering::Release);
        log::info!(
            "Client #{} telemetry -> {}",
            client.id,
            if on { "TCP" } else { "UDP" }
        );
    }
}

/// What a command needs from the lease.
#[derive(Debug, PartialEq, Eq)]
pub enum LeaseRule {
    /// Anyone, always: stops, and the lidar (a sensor every client relies on).
    Free,
    /// Anyone while nobody holds the lease, without taking it (cleaning
    /// actuators, LED): a UI toggling the vacuum must not lock out the SLAM
    /// controller.
    IfUnheld,
    /// Motion, calibration, power and daemon control: the holder only; taken
    /// implicitly when free so single-client tools keep working.
    Acquire,
}

pub fn lease_rule(cmd: &Command) -> LeaseRule {
    match cmd {
        Command::ComponentControl { id, action } if is_stop(id, action) || id == "lidar" => LeaseRule::Free,
        Command::ComponentControl { id, .. } if UNLEASED_ACTUATORS.contains(&id.as_str()) => LeaseRule::IfUnheld,
        _ => LeaseRule::Acquire,
    }
}

/// `Disable` of a motion component, or `Reset` of the drive (emergency stop).
pub fn is_stop(id: &str, action: &ComponentAction) -> bool {
    match action {
        ComponentAction::Disable { .. } => STOPPABLE.contains(&id),
        ComponentAction::Reset { .. } => id == "drive",
        _ => false,
    }
}

fn spawn_writer(id: ClientId, mut w: TcpStream, alive: Arc<AtomicBool>) -> std::io::Result<TcpWriter> {
    // A wedged peer must not block the writer forever.
    w.set_write_timeout(Some(Duration::from_millis(500)))?;
    let (tx, rx) = bounded::<Frame>(TCP_QUEUE_FRAMES);
    let cancel = Arc::new(AtomicBool::new(false));
    let stop = Arc::clone(&cancel);
    let thread = std::thread::Builder::new()
        .name(format!("tcp-telemetry-{}", id))
        .spawn(move || {
            // Ends when the sender is dropped (disconnect, back to UDP) or a
            // write fails (a partial frame would corrupt the stream: drop the
            // client).
            for frame in rx.iter() {
                if stop.load(Ordering::Relaxed) {
                    break;
                }
                if let Err(e) = w.write_all(&frame) {
                    log::info!("Client #{} TCP telemetry write failed: {}", id, e);
                    alive.store(false, Ordering::Relaxed);
                    break;
                }
            }
        })?;
    Ok(TcpWriter { tx, cancel, thread })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::collections::HashMap;

    fn addr(port: u16) -> SocketAddr {
        SocketAddr::from(([127, 0, 0, 1], port))
    }

    fn cc(id: &str, action: ComponentAction) -> Command {
        Command::ComponentControl {
            id: id.to_string(),
            action,
        }
    }

    fn drive() -> Command {
        let mut config = HashMap::new();
        config.insert("linear".to_string(), SensorValue::F32(0.1));
        cc("drive", ComponentAction::Configure { config })
    }

    fn take(force: bool) -> Command {
        let mut config = HashMap::new();
        config.insert("force".to_string(), SensorValue::Bool(force));
        cc(
            "lease",
            ComponentAction::Enable {
                config: Some(config),
            },
        )
    }

    #[test]
    fn first_controller_takes_the_lease_implicitly() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        assert_eq!(hub.authorize(&a, &drive(), || {}), Verdict::Execute);
        assert_eq!(hub.lease_holder(), Some(a.id));
        assert_eq!(hub.authorize(&b, &drive(), || {}), Verdict::Refused(a.id));
    }

    #[test]
    fn stop_is_accepted_from_anyone() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        hub.authorize(&a, &drive(), || {});
        for stop in [
            cc("drive", ComponentAction::Disable { config: None }),
            cc("drive", ComponentAction::Reset { config: None }),
            cc("vacuum", ComponentAction::Disable { config: None }),
        ] {
            assert_eq!(hub.authorize(&b, &stop, || {}), Verdict::Execute);
        }
        // The lidar is a shared sensor: anyone may switch it.
        let lidar_on = cc("lidar", ComponentAction::Enable { config: None });
        assert_eq!(hub.authorize(&b, &lidar_on, || {}), Verdict::Execute);
        // Actuators need the lease while someone holds it...
        let vacuum_on = cc("vacuum", ComponentAction::Enable { config: None });
        assert_eq!(hub.authorize(&b, &vacuum_on, || {}), Verdict::Refused(a.id));
        // ...and calibration always does.
        let imu_reset = cc("imu", ComponentAction::Reset { config: None });
        assert_eq!(hub.authorize(&b, &imu_reset, || {}), Verdict::Refused(a.id));
    }

    #[test]
    fn actuators_run_without_taking_a_free_lease() {
        let hub = ClientHub::new(8);
        let ui = hub.connect(addr(1), 5556, None);
        let slam = hub.connect(addr(2), 5556, None);
        let vacuum_on = cc("vacuum", ComponentAction::Enable { config: None });
        assert_eq!(hub.authorize(&ui, &vacuum_on, || {}), Verdict::Execute);
        assert_eq!(hub.lease_holder(), None, "the UI didn't take the lease");
        assert_eq!(hub.authorize(&slam, &drive(), || {}), Verdict::Execute);
        assert!(!hub.still_allowed(&ui, &vacuum_on), "now held by the SLAM client");
    }

    #[test]
    fn lease_changes_hands_only_with_force_and_always_stops_the_drive() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        let stops = Cell::new(0);
        let stop = || stops.set(stops.get() + 1);

        assert_eq!(hub.authorize(&a, &take(false), stop), Verdict::Handled);
        assert_eq!(
            (hub.lease_holder(), stops.get()),
            (Some(a.id), 0),
            "free lease: no stop"
        );
        hub.authorize(&b, &take(false), stop);
        assert_eq!(
            (hub.lease_holder(), stops.get()),
            (Some(a.id), 0),
            "no takeover without force"
        );
        hub.authorize(&b, &take(true), || {
            // Handed over before the stop: a's queued commands fail the re-check.
            assert_eq!(hub.lease_holder(), Some(b.id));
            stop()
        });
        assert_eq!(
            (hub.lease_holder(), stops.get()),
            (Some(b.id), 1),
            "takeover stops a's motion"
        );
        hub.authorize(
            &a,
            &cc("lease", ComponentAction::Disable { config: None }),
            stop,
        );
        assert_eq!(
            (hub.lease_holder(), stops.get()),
            (Some(b.id), 1),
            "non-holder can't release"
        );
        hub.authorize(
            &b,
            &cc("lease", ComponentAction::Disable { config: None }),
            stop,
        );
        assert_eq!(
            (hub.lease_holder(), stops.get()),
            (None, 2),
            "release stops the drive"
        );
    }

    #[test]
    fn holder_disconnect_stops_then_frees_the_lease() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        hub.authorize(&a, &drive(), || {});
        let stopped = Cell::new(false);
        hub.disconnect(b.id, || stopped.set(true));
        assert!(!stopped.get(), "non-holder disconnect is not a stop");
        // stop() runs under the lease lock, so no new holder can slip in.
        hub.disconnect(a.id, || {
            assert!(hub.lease.try_lock().is_err(), "lease locked while stopping");
            assert_ne!(hub.lease_holder(), Some(a.id), "handed over before the stop");
            stopped.set(true)
        });
        assert!(stopped.get());
        assert_eq!(hub.lease_holder(), None);
        assert!(hub.clients().is_empty());
    }

    #[test]
    fn full_hub_evicts_the_oldest_non_holder() {
        let hub = ClientHub::new(2);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        hub.authorize(&a, &drive(), || {});
        let _c = hub.connect(addr(3), 5556, None);
        assert!(a.alive.load(Ordering::Relaxed), "the lease holder stays");
        assert!(!b.alive.load(Ordering::Relaxed));
        assert_eq!(hub.clients().len(), 2);
    }

    #[test]
    fn tcp_telemetry_reaches_every_tcp_client_and_can_toggle() {
        use std::io::Read;
        use std::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
        let hub = ClientHub::new(8);
        let on = cc("telemetry", ComponentAction::Enable { config: None });
        let off = cc("telemetry", ComponentAction::Disable { config: None });
        let mut readers = Vec::new();
        for _ in 0..2 {
            let reader = TcpStream::connect(("127.0.0.1", port)).unwrap();
            let (server_side, peer) = listener.accept().unwrap();
            let c = hub.connect(peer, 5556, Some(server_side));
            let g = hub.generation();
            hub.authorize(&c, &on, || {});
            assert_eq!(c.transport(), Transport::Tcp);
            assert!(
                hub.generation() > g,
                "switch triggers a resend of one-shot groups"
            );
            // TCP -> UDP -> TCP works (the socket is cloned per switch).
            hub.authorize(&c, &off, || {});
            assert_eq!(c.transport(), Transport::Udp);
            hub.authorize(&c, &on, || {});
            assert_eq!(c.transport(), Transport::Tcp);
            readers.push(reader);
        }
        let (mut shared, mut udp_sent) = (None, Vec::new());
        for c in hub.clients() {
            c.deliver(b"hello", &mut shared, &udp, &mut udp_sent);
        }
        assert!(udp_sent.is_empty(), "TCP clients get no datagram");
        for mut r in readers {
            r.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            let mut buf = [0u8; 5];
            r.read_exact(&mut buf).unwrap();
            assert_eq!(&buf, b"hello");
        }
    }

    #[test]
    fn deposed_holder_is_visible_lock_free() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        assert_eq!(hub.authorize(&a, &drive(), || {}), Verdict::Execute);
        hub.authorize(&b, &take(true), || {});
        // What the receiver re-checks under the driver lock before executing.
        assert_ne!(hub.lease_holder(), Some(a.id));
    }

    #[test]
    fn non_holder_cannot_shutdown() {
        let hub = ClientHub::new(8);
        let a = hub.connect(addr(1), 5556, None);
        let b = hub.connect(addr(2), 5556, None);
        hub.authorize(&a, &drive(), || {});
        assert_eq!(
            hub.authorize(&b, &Command::Shutdown, || {}),
            Verdict::Refused(a.id)
        );
    }
}
