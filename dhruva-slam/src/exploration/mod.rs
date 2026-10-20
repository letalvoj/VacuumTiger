//! Autonomous exploration for mapping.
//!
//! This module provides algorithms for autonomous exploration during map building:
//!
//! - [`strategy`]: the strategy trait every algorithm plugs in through
//! - [`frontier`]: frontier exploration, driving to targets by A* (the navigation thread)
//! - [`edge`]: edge passes, once round the room with the wall on one side
//! - [`session`]: one session tick by tick: contacts, the SLAM gate, undocking, the
//!   stuck watch, the strategy's actions as commands
//!
//! # Architecture
//!
//! The exploration thread reads a [`session::Inputs`] snapshot of the SharedState each
//! loop, ticks the [`session::Session`] and sends its command to SangamIO (or the
//! navigation thread). Benches tick the same session against a simulated robot.
//!
//! # Example
//!
//! ```ignore
//! use dhruva_slam::exploration::{ExplorationStrategy, FrontierExploration, FrontierConfig};
//!
//! let config = FrontierConfig::default();
//! let mut strategy = FrontierExploration::new(config);
//!
//! // In exploration loop:
//! let action = strategy.next_action(&map, &pose, &scan, hazard.as_ref(), None);
//! match action {
//!     ExplorationAction::MoveTo { target, .. } => { /* navigate to target */ }
//!     ExplorationAction::Complete => { /* mapping finished */ }
//!     // ...
//! }
//! ```

mod edge;
mod frontier;
pub mod session;
mod steady;
mod strategy;

pub use edge::WallSide;
pub use edge::wall_follow::{WallFollowConfig, WallFollower};
pub use frontier::{FrontierConfig, FrontierExploration};
pub use strategy::{ExplorationAction, ExplorationStrategy, HazardEvent, HazardType};
