//! DhruvaSLAM - Modular SLAM implementation for robotic vacuum cleaners
//!
//! The library behind the `dhruva-slam` daemon. Offline tools (replay, debugging,
//! visualisation) build on it instead of on the daemon.
//!
//! - **Matcher**: Configurable scan matching (MultiRes, ICP, Correlative, Hybrid)
//! - **Odometry**: Configurable fusion (Mahony, ESKF, Complementary, Wheel-only)
//! - **Maps**: Dual representation (Occupancy Grid + Feature-based)
//! - **Loop Closure**: LiDAR-IRIS descriptor + Sparse CG optimization
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │   threads/, state/, exploration/, navigation/       │  ← Daemon
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                config, io/                          │  ← Infrastructure
//! │      (TOML config, sangam_client, recordings)       │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                    engine/                          │  ← Orchestration
//! │              (slam, graph optimization)             │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                  algorithms/                        │  ← Core algorithms
//! │         (matching, mapping, localization)           │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                   sensors/                          │  ← Sensor processing
//! │            (odometry, preprocessing)                │
//! └─────────────────────────────────────────────────────┘
//!                          │
//! ┌─────────────────────────────────────────────────────┐
//! │                     core/                           │  ← Foundation
//! │                (types, math)                        │
//! └─────────────────────────────────────────────────────┘
//! ```

// Foundation (no internal deps)
pub mod core;

// Sensor processing (depends on core)
pub mod sensors;

// Algorithms (depends on core, sensors)
pub mod algorithms;

// SLAM engine (depends on core, sensors, algorithms)
pub mod engine;

// Infrastructure: I/O and the TOML configuration
pub mod config;
pub mod io;

// Daemon: shared state, threads, autonomous exploration, navigation
pub mod exploration;
pub mod navigation;
pub mod state;
pub mod threads;
