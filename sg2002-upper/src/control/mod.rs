//! 下位机（ESP32-C3）通信与控制。
//!
//! - [`protocol`]：线协议 `AA 55 CMD LEN PAYLOAD CHK`——上下位机共用的
//!   `smart-car-protocol` crate，这里整体重导出，调用方照旧
//!   `sg2002_upper::control::protocol::...`；
//! - [`serial`]：`/dev/ttyS1` 打开、raw termios 与 `O_NONBLOCK`；
//! - [`car`]：链路线程（重发 / 心跳 / 安全看门狗）与 `Status` 缓存；
//! - [`servo`]：视觉伺服控制律（检测结果 -> 左右轮速度）。
//!
//! 常用类型在此统一重导出，调用方 `use sg2002_upper::control::{Car, ControlLoop};`
//! 即可；需要组帧函数/枚举时再进子模块。

pub mod car;
pub mod serial;
pub mod servo;

pub use smart_car_protocol as protocol;

pub use car::{Car, CarConfig, Counters, Desired};
pub use servo::{Action, ControlConfig, ControlLoop, Distance, Observation};
