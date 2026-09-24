//! 下位机（ESP32-C3）通信与控制。
//!
//! - [`protocol`]：线协议 `AA 55 CMD LEN PAYLOAD CHK`——上下位机共用的
//!   `protocol` crate，这里整体重导出，调用方照旧
//!   `sg2002_upper::control::protocol::...`；
//! - `link`：链路层——`/dev/ttyS1` 打开、raw termios、`O_NONBLOCK` 读写与读线程（只搬字节/帧）；
//! - [`car`]：传输层——协议对话（命令/应答）、意图状态机、心跳与死手开关；
//! - [`teleop`]：手动遥控控制律（按键 -> 轮速，纯函数式）；
//! - [`servo`]：视觉伺服控制律（观测 -> 左右轮速度）；
//! - [`position`]：位置分析（检测框 -> 九宫格分区/距离分级 -> 控制律观测）；
//! - [`session`]：控制会话——控制节拍 + 手动/自动模式仲裁（`smartcar` 用它）。
//!
//! 常用类型在此统一重导出，调用方 `use sg2002_upper::control::{Car, ControlLoop};`
//! 即可；需要组帧函数/枚举时再进子模块。

pub mod car;
pub mod position;
pub mod servo;
pub mod session;
pub mod teleop;

mod link;

pub use protocol;

pub use car::{Car, CarConfig, Counters, LinkState};
pub use position::{Distance, Observation, PositionAnalyzer};
pub use servo::{Action, ControlConfig, ControlLoop};
pub use session::{ControlSession, ControlStatus, Mode};
pub use teleop::{Teleop, TeleopConfig};
