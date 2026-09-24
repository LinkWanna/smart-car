//! 传输层：与 ESP32-C3 下位机的串口通信。
//!
//! - `link`：链路层——`/dev/ttyS1` 打开、raw termios、`O_NONBLOCK` 读写与
//!   读线程（只搬字节/帧）；
//! - [`car`]：传输层——协议对话（命令/应答）、意图状态机、心跳与死手开关。
//!
//! 本层不解释业务语义：应用策略（控制律、模式仲裁、什么时候补 `Init`）在
//! [`crate::control`]；线协议 `AA 55 CMD LEN PAYLOAD CHK` 由上下位机共用的
//! `protocol` crate 定义，这里整体重导出，调用方照旧
//! `sg2002_upper::transport::protocol::...`。
//!
//! 常用类型在此统一重导出，调用方 `use sg2002_upper::transport::{Car, CarConfig};`
//! 即可；需要组帧函数/枚举时再进子模块。

pub mod car;

mod link;

pub use protocol;

pub use car::{Car, CarConfig, Counters, LinkState};
