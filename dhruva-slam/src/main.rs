//! dhruva-slam daemon: the SLAM threads of the `dhruva_slam` library behind a TCP
//! stream, or an offline `--replay` of a recording.
//!
//! ```bash
//! # With default config
//! cargo run --release
//!
//! # With custom config file
//! cargo run --release -- --config dhruva-slam.toml
//!
//! # Offline replay
//! cargo run --release -- -c dhruva-v8.toml --replay run.mcap --out /tmp/out
//! ```

// ============================================================================
// Imports
// ============================================================================

use std::io::Write;
use std::path::Path;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use dhruva_slam::algorithms::planning::AStarConfig;
use dhruva_slam::config::{Config, build_exploration, build_slam_thread_config};
use dhruva_slam::io::map_manager::MapManager;
use dhruva_slam::io::motion_controller::{MotionConfig, create_shared_motion_controller};
use dhruva_slam::navigation::NavigatorConfig;
use dhruva_slam::state::{
    CommandReceiver, CommandSender, SharedStateHandle, create_command_channel, create_shared_state,
};
use dhruva_slam::threads::{
    ExplorationThread, NavigationThread, NavigationThreadConfig, PublisherThread, SlamThread,
    SlamThreadConfig, StreamConfig as ThreadStreamConfig, create_nav_channel, run_replay,
};

// ============================================================================
// CLI Arguments
// ============================================================================

struct Args {
    config_path: Option<String>,
    /// Offline replay of a recording (MCAP file, run directory, or SGIO1) instead of running the daemon.
    replay: Option<String>,
    /// Output directory for `--replay` (timing.csv, map.pgm, map.yaml).
    out: Option<String>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let mut result = Args {
        config_path: None,
        replay: None,
        out: None,
    };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--config" | "-c" => {
                if i + 1 < args.len() {
                    result.config_path = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--replay" => {
                if i + 1 < args.len() {
                    result.replay = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--out" => {
                if i + 1 < args.len() {
                    result.out = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            _ => {
                eprintln!("Unknown argument: {}", args[i]);
                print_help();
                std::process::exit(1);
            }
        }
        i += 1;
    }

    result
}

fn print_help() {
    println!("dhruva-slam - SLAM daemon for robotic vacuum cleaners");
    println!();
    println!("USAGE:");
    println!("    dhruva-slam [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -c, --config <FILE>     Configuration file (default: dhruva-slam.toml)");
    println!(
        "        --replay <PATH>     Offline: run SLAM over a recording (.mcap, run dir, .sgio) and exit"
    );
    println!("        --out <DIR>         Output dir for --replay (default: replay_out)");
    println!("    -h, --help              Print help information");
    println!();
    println!("CONFIGURATION:");
    println!("    All settings are configured via the TOML config file:");
    println!("    - [source] sangam_address: SangamIO daemon address");
    println!("    - [output] bind_port: TCP stream port");
    println!();
    println!("THREADS:");
    println!("    The daemon runs with 3 fixed threads:");
    println!("    - SLAM Thread: Sensor processing and scan matching");
    println!("    - Publisher Thread: Streams data to clients");
    println!("    - Command Thread: Handles control commands");
}

fn load_config(args: &Args) -> Config {
    match &args.config_path {
        Some(path) => match Config::from_file(Path::new(path)) {
            Ok(cfg) => {
                log::info!("Loaded config from {}", path);
                cfg
            }
            Err(e) => {
                log::warn!("Failed to load config: {}", e);
                Config::default()
            }
        },
        None => {
            // Try default paths
            for path in &["dhruva-slam.toml", "/etc/dhruva-slam.toml"] {
                if let Ok(cfg) = Config::from_file(Path::new(path)) {
                    log::info!("Loaded config from {}", path);
                    return cfg;
                }
            }
            Config::default()
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

fn main() {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(|buf, record| {
            writeln!(
                buf,
                "[{}] {} - {}",
                record.level(),
                record.target(),
                record.args()
            )
        })
        .init();

    let args = parse_args();
    let config = load_config(&args);

    if let Some(input) = &args.replay {
        let out = args.out.clone().unwrap_or_else(|| "replay_out".to_string());
        let cfg = build_slam_thread_config(&config);
        match run_replay(cfg, Path::new(input), Path::new(&out)) {
            Ok(r) => {
                let mut ms = r.scan_ms.clone();
                ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let pct = |p: f64| {
                    ms.get(((ms.len() as f64 - 1.0) * p) as usize)
                        .copied()
                        .unwrap_or(0.0)
                };
                let mean = ms.iter().sum::<f64>() / ms.len().max(1) as f64;
                println!(
                    "replay: {} scans, {} sensor msgs, data {:.1} s, wall {:.2} s ({:.1}x realtime)",
                    r.scans,
                    r.sensor_msgs,
                    r.data_span_s,
                    r.wall_s,
                    r.data_span_s / r.wall_s.max(1e-9)
                );
                println!(
                    "per-scan ms: mean {:.2} p50 {:.2} p95 {:.2} max {:.2}  -> {}/",
                    mean,
                    pct(0.5),
                    pct(0.95),
                    pct(1.0),
                    out
                );
            }
            Err(e) => {
                eprintln!("replay failed: {}", e);
                std::process::exit(1);
            }
        }
        return;
    }

    log::info!("dhruva-slam starting");
    log::info!(
        "  SangamIO: {} (TCP commands + UDP sensors)",
        config.source.sangam_address
    );
    log::info!(
        "  Output port: {} (TCP commands+maps, UDP status)",
        config.output.bind_port
    );
    log::info!("  Odometry: {}", config.odometry.algorithm);
    log::info!("  Matcher: {}", config.matcher.algorithm);
    log::info!(
        "  Loop Closure: {} ({})",
        if config.loop_closure.enabled {
            "enabled"
        } else {
            "disabled"
        },
        config.optimization.solver
    );

    // Setup signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    // Run multi-threaded daemon (always)
    if let Err(e) = run_threaded_mode(&config, running.clone()) {
        log::error!("Daemon error: {}", e);
    }

    log::info!("dhruva-slam shutdown complete");
}

// ============================================================================
// Multi-Threaded Daemon
// ============================================================================

fn run_threaded_mode(
    config: &Config,
    running: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    log::info!("Initializing multi-threaded SLAM daemon...");

    // 1. Create shared state
    let shared_state = create_shared_state();
    log::info!("  Shared state initialized");

    // 2. Create command channel for SLAM thread communication
    let (command_tx, command_rx) = create_command_channel();
    log::info!("  Command channel created");

    // 3. Create map manager
    let map_storage_path = Path::new(&config.map_storage.path);
    let map_manager = match MapManager::new(map_storage_path) {
        Ok(mgr) => {
            log::info!("  Map manager initialized ({})", config.map_storage.path);
            Arc::new(Mutex::new(mgr))
        }
        Err(e) => {
            log::warn!(
                "  Map manager init failed ({}): {}",
                config.map_storage.path,
                e
            );
            // Create with temp path instead
            let temp_path = std::env::temp_dir().join("dhruva_maps");
            let mgr = MapManager::new(&temp_path)?;
            log::info!("  Using temp map storage: {:?}", temp_path);
            Arc::new(Mutex::new(mgr))
        }
    };

    // 4. Load map list into shared state
    {
        if let Ok(mgr) = map_manager.lock()
            && let Ok(mut state) = shared_state.write()
        {
            state.map_list = mgr.as_map_list();
            log::info!("  Loaded {} saved maps", state.map_list.maps.len());
        }
    }

    // 5. Build SLAM thread config
    let slam_thread_config = build_slam_thread_config(config);

    // 6. Build stream config
    let stream_config = ThreadStreamConfig {
        robot_status_hz: config.stream.robot_status_hz,
        sensor_status_hz: config.stream.sensor_status_hz,
        map_hz: config.stream.map_hz,
        navigation_hz: config.stream.navigation_hz,
    };
    run_threaded_mode_rest(
        config,
        running,
        shared_state,
        command_tx,
        command_rx,
        map_manager,
        slam_thread_config,
        stream_config,
    )
}

#[allow(clippy::too_many_arguments)]
fn run_threaded_mode_rest(
    config: &Config,
    running: Arc<AtomicBool>,
    shared_state: SharedStateHandle,
    command_tx: CommandSender,
    command_rx: CommandReceiver,
    map_manager: Arc<Mutex<MapManager>>,
    slam_thread_config: SlamThreadConfig,
    stream_config: ThreadStreamConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    // 7. Create navigation channel
    let (nav_tx, nav_rx) = create_nav_channel();
    log::info!("  Navigation channel created");

    // 8. Create shared motion controller (shared between navigation and exploration)
    // This ensures a single TCP connection to SangamIO
    let motion_config = MotionConfig {
        max_linear_vel: config
            .navigation
            .max_linear_vel
            .max(config.exploration.motion.max_linear_vel),
        max_angular_vel: config
            .navigation
            .max_angular_vel
            .max(config.exploration.motion.max_angular_vel),
        linear_vel: config.exploration.motion.linear_vel,
        angular_vel: config.exploration.motion.angular_vel,
    };
    let shared_motion = create_shared_motion_controller(motion_config.clone());

    // Connect motion controller to SangamIO (will auto-reconnect on operations)
    {
        let mut mc = shared_motion.lock().unwrap();
        match mc.connect(&config.source.sangam_address) {
            Ok(()) => log::info!("  Motion controller connected to SangamIO"),
            Err(e) => log::warn!(
                "  Motion controller initial connect failed (will retry): {}",
                e
            ),
        }
    }

    // 9. Spawn threads
    log::info!("Spawning threads...");

    // SLAM Thread - processes sensors, updates shared state
    let slam_thread = SlamThread::spawn(
        slam_thread_config,
        shared_state.clone(),
        command_rx,
        running.clone(),
    );
    log::info!("  SLAM thread started");

    // Navigation Thread - goal-based path following (optional)
    // Must be spawned before Publisher so we can pass nav_tx to Publisher
    let (navigation_thread, nav_command_tx) = if config.navigation.enabled {
        let nav_config = NavigationThreadConfig {
            navigator: NavigatorConfig {
                planner: AStarConfig {
                    robot_radius: config.navigation.robot_radius,
                    safety_margin: config.navigation.safety_margin,
                    unknown_is_obstacle: config.navigation.unknown_is_obstacle,
                    ..Default::default()
                },
                waypoint_reached_threshold: config.navigation.waypoint_threshold,
                heading_threshold: config.navigation.heading_threshold,
                max_linear_vel: config.navigation.max_linear_vel,
                max_angular_vel: config.navigation.max_angular_vel,
                ..Default::default()
            },
            mapping: Default::default(),
            update_rate_hz: config.navigation.update_rate_hz,
        };

        let thread = NavigationThread::spawn(
            nav_config,
            shared_state.clone(),
            shared_motion.clone(),
            nav_rx,
            running.clone(),
        );
        log::info!(
            "  Navigation thread started ({}Hz)",
            config.navigation.update_rate_hz
        );
        (Some(thread), Some(nav_tx))
    } else {
        log::info!("  Navigation thread disabled");
        // Drop nav_rx and nav_tx if navigation disabled
        drop(nav_rx);
        drop(nav_tx);
        (None, None)
    };

    // Publisher Thread - unified client interface (streams + commands)
    // Single port for TCP (commands + maps) and UDP (status)
    let publisher_thread = PublisherThread::spawn(
        config.output.bind_port,
        shared_state.clone(),
        stream_config,
        command_tx,
        nav_command_tx,
        map_manager.clone(),
        running.clone(),
    );
    log::info!(
        "  Publisher thread started (port {} - TCP commands+maps, UDP status)",
        config.output.bind_port
    );

    // Exploration Thread - autonomous mapping (optional)
    let exploration_thread = if config.exploration.enabled {
        let (exploration_config, strategy) = build_exploration(&config);
        let name = strategy.name();

        let thread = ExplorationThread::spawn(
            exploration_config,
            strategy,
            shared_state.clone(),
            shared_motion.clone(),
            running.clone(),
        );
        log::info!("  Exploration thread started (strategy: {name})");
        Some(thread)
    } else {
        log::info!("  Exploration thread disabled");
        None
    };

    log::info!("Multi-threaded daemon running");
    log::info!(
        "  Output port: {} (TCP commands+maps, UDP status)",
        config.output.bind_port
    );
    if config.navigation.enabled {
        log::info!(
            "  Navigation: enabled ({}Hz)",
            config.navigation.update_rate_hz
        );
    }
    if config.exploration.enabled {
        log::info!("  Exploration: enabled ({:?})", config.exploration.strategy);
    }

    // 9. Wait for shutdown signal (main thread just monitors)
    // SangamIO only streams telemetry to connected TCP clients, and it may
    // start listening seconds after us (MCU init): keep trying to connect.
    // The controller's reconnect cooldown is the real rate limit; this timer
    // only avoids taking the lock every 100 ms, and a busy lock skips a round.
    let mut last_connect_check = Instant::now();
    while running.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(100));
        if last_connect_check.elapsed() >= Duration::from_secs(2) {
            last_connect_check = Instant::now();
            if let Ok(mut mc) = shared_motion.try_lock() {
                mc.keep_connected();
            }
        }
    }

    log::info!("Shutdown signal received, waiting for threads...");

    // 10. Join threads
    if let Err(e) = slam_thread.join() {
        log::error!("SLAM thread panicked: {:?}", e);
    }
    if let Err(e) = publisher_thread.join() {
        log::error!("Publisher thread panicked: {:?}", e);
    }
    if let Some(thread) = navigation_thread
        && let Err(e) = thread.join()
    {
        log::error!("Navigation thread panicked: {:?}", e);
    }
    if let Some(thread) = exploration_thread {
        thread.join();
    }

    log::info!("All threads stopped");
    Ok(())
}
