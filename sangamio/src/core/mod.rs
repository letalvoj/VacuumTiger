//! Core abstractions for device drivers.
//!
//! - [`driver::DeviceDriver`]: Trait to implement for new hardware
//! - [`types`]: Sensor values, commands, and data structures
//! - [`drive`]: Drive rules every driver applies (the bumper interlock)

pub mod drive;
pub mod driver;
pub mod types;
