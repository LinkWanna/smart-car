//! Services: control loop, closed-loop odometry and command links.
//!
//! The command link is selected by the mutually exclusive `uart` / `ble`
//! cargo features, so exactly one link module is compiled in.

#[cfg(feature = "ble")]
pub mod ble;
pub mod control;
pub mod link;
pub mod odometry;
#[cfg(feature = "uart")]
pub mod uart;
