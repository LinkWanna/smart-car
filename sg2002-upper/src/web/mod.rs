//! 网页遥控：HTTP + WebSocket + MJPEG 预览（不含 TLS）。
//!
//! 用一台手机/电脑连上小车的 AP（`scripts/init_ap.sh`，默认 `192.168.4.1`），
//! 浏览器打开页面即可遥控；也支持**自动模式**（视觉伺服），由页面按钮切换。
//!
//! 本模块只负责「页面/接口」：路由、WebSocket 会话、JSON 组装。控制节拍与
//! 手动/自动仲裁在 [`crate::control::ControlSession`]（按键 -> 控制律 -> 链路），
//! 预览来自 `VisionStream`（CPU 管线）或 [`crate::vision::VpssStream`]（VPSS
//! 管线）——网页层只依赖 [`crate::preview::PreviewSource`] 抽象，拿到的 JPEG
//! 与检测框**同帧**。
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
//! | `GET /api/mode?set=auto\|manual` | 输入模式（自动需要视觉就绪） |
//! | `GET /api/events` | WebSocket：按键/动作/模式（文本 JSON）+ 状态/JPEG/视觉（推送） |
//!
//! # 安全
//!
//! 浏览器每 100ms 发一次完整按键快照（变化时立即发），服务端
//! [`TeleopConfig::input_timeout`](crate::control::TeleopConfig::input_timeout)
//! 收不到就松开所有键；WebSocket 断开时只清理该客户端的按键；模式切换、控制
//! 线程退出前都会滑行停车，[`Car`](crate::control::Car) 自己的看门狗与 `Drop`
//! 再兜底一层；自动模式下视觉观测过期（>500ms）按“看不到”处理并滑行。

mod server;

pub use server::{Server, WebConfig};
