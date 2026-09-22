//! 网页遥控：HTTP + WebSocket + MJPEG 预览（不含 TLS）。
//!
//! 用一台手机/电脑连上小车的 AP（`scripts/init_ap.sh`，默认 `192.168.4.1`），
//! 浏览器打开页面即可遥控，驾驶手感与 `tools/play.py` 一致。分层：
//!
//! - [`teleop`]：纯控制律（W/S 油门斜坡、A/D 转向、低速原地旋转、输入看门狗），
//!   只输出 [`teleop::Output`]，不碰链路；
//! - [`video`]：MJPG 采集线程 + 最新帧广播；
//! - [`server`]：基于 `rouille`（HTTP + WebSocket）与 `serde_json` 的路由、
//!   会话、控制节拍与状态推送；
//! - `driver.rs`：把 [`DriveTarget`] 落到 [`crate::control::Car`] 上。
//!
//! # 接口
//!
//! | 路径 | 说明 |
//! | --- | --- |
//! | `GET /` | 内嵌的遥控页面（单文件，零外部资源） |
//! | `GET /api/status` | 一帧状态 JSON（与 WebSocket 推送同构） |
//! | `GET /api/frame.jpg` | 摄像头快照（无预览时 503） |
//! | `GET /api/input?keys=wasd` | 按键快照（curl 调试用） |
//! | `GET /api/action?action=stop\|brake\|init\|reset` | 动作按钮 |
//! | `GET /api/events` | WebSocket：按键/动作（文本 JSON）+ 状态/JPEG（推送） |
//!
//! # 安全
//!
//! 浏览器每 100ms 发一次完整按键快照（变化时立即发），服务端
//! [`TeleopConfig::input_timeout`](teleop::TeleopConfig::input_timeout) 收不到就
//! 松开所有键；WebSocket 断开时只清理该客户端的按键；控制线程退出前滑行停车，
//! [`Car`](crate::control::Car) 自己的看门狗与 `Drop` 再兜底一层。

use std::time::Duration;

use crate::control::Counters;

pub mod teleop;
pub mod video;

mod driver;
mod server;

pub use server::{Server, WebConfig};
pub use teleop::{Action, Keys, Output, Teleop, TeleopConfig};
pub use video::{CameraStream, VideoStatus};

/// 下位机链路快照（HUD 用；真实实现见 `driver.rs`）。
#[derive(Debug, Clone, Default)]
pub struct LinkSnapshot {
    /// 最近是否收到过 `Status`（链路可用）。
    pub link_ok: bool,
    /// 最近一次 `Status` 距今的时间。
    pub link_age: Option<Duration>,
    /// 固件状态机（`Uninit`/`Ready`/`Running`），无应答为 `None`。
    pub sys: Option<String>,
    /// 双轮实测转速（RPM），`[左, 右]`。
    pub rpm: [i16; 2],
    /// 固件闭环运动是否运行中。
    pub dist_active: bool,
    /// 0 = 无/运行中，1 = 已到达目标。
    pub dist_result: u8,
    /// 链路计数。
    pub counters: Counters,
    /// 最近一帧应答的可读文本。
    pub last_frame: String,
}

/// 遥控输出端：真实串口链路（[`crate::control::Car`]）或测试替身。
pub trait DriveTarget: Send + Sync {
    /// 双轮目标速度（-100..100）。
    fn set_speeds(&self, left: i16, right: i16);

    /// 滑行。
    fn coast(&self);

    /// 短接制动。
    fn brake(&self);

    /// 编码器清零并进入 Ready。
    fn init(&self);

    /// 回 Uninit（调试用）。
    fn reset(&self);

    /// 下位机处于 Uninit / 状态不对时补发 `Init`；返回是否触发。
    fn maybe_reinit(&self) -> bool {
        false
    }

    /// 当前链路快照。
    fn snapshot(&self) -> LinkSnapshot;
}
