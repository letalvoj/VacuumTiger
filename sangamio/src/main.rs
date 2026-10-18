//! SangamIO - Hardware abstraction daemon for robot vacuum
//!
//! ## Protocol Architecture
//!
//! - **TCP (port 5555)**: commands, and telemetry for clients that ask for it
//! - **UDP unicast (`udp_client_port`)**: telemetry to every other client
//!
//! Several clients may be connected at once; each gets the full telemetry
//! stream, and a control lease decides which one may drive (see
//! `streaming/clients.rs`).

use sangamio::config::Config;
use sangamio::core::types::{Command, ComponentAction};
use sangamio::devices::create_device;
use sangamio::error::{Error, Result};
use sangamio::streaming::net::enable_dead_peer_detection;
use sangamio::streaming::tcp_receiver::stop_drive;
use sangamio::streaming::{ClientHub, TcpReceiver, TelemetryPublisher, create_serializer};
use std::env;
use std::net::{TcpListener, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

/// Parse config path from command line arguments.
///
/// Supports:
/// - `sangam-io <path>` (positional)
/// - `sangam-io --config <path>` (flag-based)
/// - `sangam-io -c <path>` (short flag)
///
/// Defaults to `/etc/sangamio.toml` if not specified.
fn parse_config_path() -> String {
    let args: Vec<String> = env::args().collect();

    // Look for --config or -c flag
    for i in 1..args.len() {
        if (args[i] == "--config" || args[i] == "-c") && i + 1 < args.len() {
            return args[i + 1].clone();
        }
    }

    // Fall back to first positional argument (if it doesn't start with -)
    if args.len() > 1 && !args[1].starts_with('-') {
        return args[1].clone();
    }

    // Default path
    "/etc/sangamio.toml".to_string()
}

fn main() -> Result<()> {
    // Initialize logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("SangamIO v0.3.0 starting...");

    // Get config path from args or default
    // Supports: sangam-io <path> OR sangam-io --config <path>
    let config_path = parse_config_path();

    // Load configuration
    log::info!("Using config: {}", config_path);
    let config = Config::load(&config_path)?;

    log::info!(
        "Device: {} ({})",
        config.device.name,
        config.device.device_type
    );

    // Create device driver
    let mut driver = create_device(&config)?;

    // Initialize driver and get sensor data + streaming channels
    let init_result = driver.initialize()?;
    let sensor_data = init_result.sensor_data;
    let stream_receivers = init_result.stream_receivers;
    log::info!(
        "Initialized {} sensor groups ({} with streaming channels)",
        sensor_data.len(),
        stream_receivers.len()
    );

    let driver = Arc::new(Mutex::new(driver));

    // Safe start (opt-in via [device.hardware] safe_start): force motors,
    // actuators and the LiDAR off on launch. The CRL-200S can retain a running
    // state across restarts, so a previous session may leave things moving.
    let safe_start = config
        .device
        .hardware
        .as_ref()
        .map(|h| h.safe_start)
        .unwrap_or(false);
    if safe_start {
        let sd = Arc::clone(&driver);
        // Runs to completion BEFORE the TCP server accepts clients (~6 s). As a
        // background thread its later rounds could queue behind a client's first
        // command (lidar enable blocks ~30 s) and then switch the lidar back off.
        (move || {
                // Retry across the GD32's ~5s wake window so the commands land
                // even if issued early. Re-stopping is idempotent.
                for attempt in 1..=3 {
                    thread::sleep(std::time::Duration::from_secs(2));
                    let stops = [
                        ("drive", ComponentAction::Reset { config: None }),
                        ("lidar", ComponentAction::Disable { config: None }),
                        ("vacuum", ComponentAction::Disable { config: None }),
                        ("main_brush", ComponentAction::Disable { config: None }),
                        ("side_brush", ComponentAction::Disable { config: None }),
                        ("water_pump", ComponentAction::Disable { config: None }),
                    ];
                    if let Ok(mut d) = sd.lock() {
                        for (id, action) in stops {
                            let _ = d.send_command(Command::ComponentControl {
                                id: id.to_string(),
                                action,
                            });
                        }
                    }
                    if attempt == 1 {
                        log::info!(
                            "safe_start: commanded motors, actuators and LiDAR off"
                        );
                    }
                }
        })();
    }

    // Set up shutdown signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = Arc::clone(&running);

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .map_err(|e| Error::Other(format!("Error setting Ctrl-C handler: {}", e)))?;

    log::debug!("Wire format: Protobuf");

    // =========================================================================
    // Telemetry (UDP unicast or TCP, to every connected client)
    // =========================================================================
    // UDP streaming port (may be different from TCP for localhost testing)
    let udp_streaming_port = config.network.udp_port();

    // Bind UDP socket to any available port (we only send, not receive)
    let udp_socket = UdpSocket::bind("0.0.0.0:0")
        .map_err(|e| Error::Other(format!("Failed to create UDP socket: {}", e)))?;

    if udp_streaming_port == config.network.port() {
        log::debug!(
            "UDP unicast streaming enabled (port {} - same as TCP)",
            udp_streaming_port
        );
    } else {
        log::debug!(
            "UDP unicast streaming enabled (port {} - different from TCP:{})",
            udp_streaming_port,
            config.network.port()
        );
    }

    // Every connected client gets the telemetry stream; the control lease
    // decides who may drive (see streaming/clients.rs).
    let hub = ClientHub::new(config.network.max_clients);

    // Spawn the telemetry publisher (fan-out to all clients)
    let udp_serializer = create_serializer();
    let udp_sensor_data = sensor_data.clone();
    let udp_running = Arc::clone(&running);
    let pub_hub = Arc::clone(&hub);
    let _udp_handle = thread::Builder::new()
        .name("telemetry".to_string())
        .spawn(move || {
            let mut publisher = TelemetryPublisher::new(
                udp_socket,
                udp_serializer,
                udp_sensor_data,
                stream_receivers, // the publisher owns the streaming channels
                udp_running,
                pub_hub,
            );
            if let Err(e) = publisher.run() {
                log::error!("Telemetry publisher error: {}", e);
            }
        })
        .map_err(|e| Error::Other(format!("Failed to spawn telemetry publisher: {}", e)))?;

    // =========================================================================
    // TCP Server Setup (commands only)
    // =========================================================================
    let bind_addr = &config.network.bind_address;
    let listener = TcpListener::bind(bind_addr)
        .map_err(|e| Error::Other(format!("Failed to bind to {}: {}", bind_addr, e)))?;
    if let Err(e) = listener.set_nonblocking(true) {
        log::warn!("Failed to set nonblocking mode: {}", e);
    }

    log::info!(
        "TCP server listening on {} (up to {} clients)",
        bind_addr,
        config.network.max_clients
    );
    log::info!("SangamIO running. Press Ctrl-C to stop.");

    // Main loop - accept TCP connections. Each connection is registered in the
    // hub (UDP telemetry to <peer IP>:udp_port until it asks for TCP).
    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, addr)) => {
                // Set socket to blocking mode for reliable command handling
                if let Err(e) = stream.set_nonblocking(false) {
                    log::error!("Failed to set socket to blocking mode: {}", e);
                    continue;
                }
                enable_dead_peer_detection(&stream);
                // Write-clone for TCP telemetry; the write timeout keeps a wedged
                // remote from blocking its writer thread forever.
                let writer = match stream.try_clone() {
                    Ok(w) => Some(w),
                    Err(e) => {
                        log::warn!("Failed to clone stream for TCP telemetry: {}", e);
                        None
                    }
                };
                let client = hub.connect(addr, udp_streaming_port, writer);
                let client_id = client.id;

                let driver_clone = Arc::clone(&driver);
                let disconnect_driver = Arc::clone(&driver);
                let recv_serializer = create_serializer();
                let recv_running = Arc::clone(&running);
                let recv_hub = Arc::clone(&hub);

                let spawned = thread::Builder::new()
                    .name("tcp-receiver".to_string())
                    .spawn(move || {
                        let id = client.id;
                        let mut receiver = TcpReceiver::new(
                            recv_serializer,
                            driver_clone,
                            recv_running,
                            Arc::clone(&recv_hub),
                            client,
                        );
                        if let Err(e) = receiver.run(stream) {
                            log::error!("TCP receiver error: {}", e);
                        }
                        log::info!("TCP client #{} disconnected: {}", id, addr);

                        // Dead-man (deterministic): if this client held the lease, the
                        // drive is zeroed before the lease is freed. The heartbeat
                        // dead-man stays as a backstop.
                        recv_hub.disconnect(id, || stop_drive(&disconnect_driver));
                    });
                if let Err(e) = spawned {
                    log::error!("Failed to spawn receiver for client #{}: {}", client_id, e);
                    hub.disconnect(client_id, || stop_drive(&driver));
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // No connection pending, sleep briefly (10ms for responsive connection acceptance)
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Err(e) => {
                // e.g. EMFILE: back off instead of spinning on the error.
                log::error!("Accept error: {}", e);
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        }
    }

    // Shutdown
    log::info!("Shutting down...");
    {
        let mut driver = driver
            .lock()
            .map_err(|e| Error::MutexPoisoned(format!("driver mutex: {}", e)))?;
        driver.send_command(Command::Shutdown)?;
    }

    log::info!("SangamIO stopped");
    Ok(())
}
