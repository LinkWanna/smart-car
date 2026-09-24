//! 控制：控制律、位置分析与控制会话（手动/自动仲裁）。
//!
//! - [`teleop`]：手动遥控控制律（按键 -> 轮速，纯函数式）；
//! - [`servo`]：视觉伺服控制律（观测 -> 左右轮速度）；
//! - [`position`]：位置分析（检测框 -> 九宫格分区/距离分级 -> 控制律观测）；
//! - [`session`]：控制会话——控制节拍 + 手动/自动模式仲裁（`smartcar` 用它）。
//!
//! 串口链路与协议对话在 [`crate::transport`]（`link` 字节流 + [`Car`](crate::transport::Car)）：
//! 本模块只做控制律与仲裁，输出经 `Car` 的意图接口落地，不碰串口细节。
//!
//! 常用类型在此统一重导出，调用方 `use sg2002_upper::control::{ControlLoop, Teleop};`
//! 即可；需要组帧函数/枚举时再进子模块。

pub mod position;
pub mod servo;
pub mod session;
pub mod teleop;

pub use position::{Distance, Observation, PositionAnalyzer};
pub use servo::{Action, ControlConfig, ControlLoop};
pub use session::{ControlSession, ControlStatus, Mode};
pub use teleop::{Teleop, TeleopConfig};
