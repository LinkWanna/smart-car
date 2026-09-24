//! SG2002 上位机：视觉检测（相机 + TPU）与 ESP32-C3 下位机的串口控制。
//!
//! 模块分层：
//! - 感知：`preprocess`（YUYV→RGB CPU 参考转换，探针用）、`yolo`（TPU 零拷贝
//!   推理 + NMS）、`vision`（视觉域：`camera` V4L2 零拷贝采集、`preview` 与
//!   网页/控制的中性契约、VPSS 硬件管线；只输出检测框，不做位置/追踪语义）；
//! - 下位机通信：`transport`（`protocol` 线协议——与固件共用的 `protocol`
//!   crate、`link` 串口链路（字节流）、`car` 传输层（协议对话 + 心跳/死手
//!   开关））；
//! - 控制：`control`（`position` 位置/距离分级（追踪侧语义）、`servo` 视觉
//!   伺服控制律、`teleop` 手动遥控控制律、`session` 控制节拍 + 手动/自动仲裁）。
//! - 网页遥控：`web`（零依赖 HTTP/WebSocket、手动遥控控制律、MJPEG 预览、
//!   手动/自动仲裁）。
//! - 观测：`logging`（`log` + `simple_logger` 初始化）。
//!
//! 入口 bin：`smartcar`（整合：视觉 + 网页 + 手动/自动）、`vpss_probe`（上板探针）。
//!
//! `yolo` / `position` / `vision` 依赖板端 C 库与 V4L2，随 crate
//! 无条件编译：板端直接运行，主机上可用 `cargo check` 做编译检查
//! （链接/运行需要厂商库，只在板端进行）。
//!
//! # 线程与生命周期
//!
//! 每个带线程的组件**自带停止标志**，[`stop`](vision::preview::PreviewSource::stop)
//! 只停自己的线程，不碰别的组件：
//!
//! | 组件 | 线程 | 停止 |
//! | --- | --- | --- |
//! | [`transport::Car`] | 串口收 + 发 | `Car::shutdown`（`Drop` 兜底） |
//! | [`control::ControlSession`] | 控制节拍 | `ControlSession::stop` |
//! | [`vision::VpssStream`] | 单线程管线 | `stop`（`Drop` 兜底） |
//! | [`web::Server`] | HTTP 池 + 每连接 WebSocket | `Server::run` 轮询内部标志 |
//!
//! 进程退出（`smartcar`）：信号处理器只把 [`web::Server::stop_flag`] 置 false →
//! `run` 返回 → 依次 `ControlSession::stop` → `preview.stop` → `Car::shutdown`。
//! 线程等待统一走 `join_with_timeout`（`STOP_TIMEOUT`），超时则 detach：
//! 硬件句柄可能来不及清理，内核里会留下 VB 池 / bind 节点（下次启动的
//! `Sys::init` 与 `clear_venc_bind` 会兜底，彻底恢复需要重启设备）。
//!
//! ## 会话（`Sys`）与生命周期
//!
//! `CVI_SYS_Init` 是进程级状态，一个进程只能有一个 [`cvimpi_rs::sys::Sys`]：
//! VPSS 管线（`vision::vpss`）自己持有会话：`VpssPipeline` 持有「借用会话的
//! 句柄 + 会话本身」，用 `unsafe` 把借用延长成 `'static`。两条硬约定：
//!
//! 1. 会话放在 `Box<Sys>` 里且**移动结构体不会移动它**（堆地址稳定）；
//! 2. 结构体字段声明顺序 = 析构顺序，`_sys` 必须声明在最后、最后析构。
//!
//! `VpssPipeline` 在主线程构造、move 进 worker 线程析构，所以只依赖上面两条。

pub mod control;
pub mod logging;
pub mod preprocess;
pub mod transport;
pub mod vision;
pub mod web;
pub mod yolo;

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// 组件 `stop` 等待线程退出的统一超时。
///
/// 正常收尾是毫秒级（实测 VPSS 管线析构 ~40ms）；这里留足余量，只有线程真的
/// 卡在驱动里才会走到超时 + detach。
pub(crate) const STOP_TIMEOUT: Duration = Duration::from_secs(1);

/// 等线程退出；超时则 detach（返回 `false`）。
///
/// detach 的后果：线程可能还持有硬件句柄（VENC 通道 / VB 池 / VPSS 组），
/// 内核里会留下残留（兜底见 crate 文档的「线程与生命周期」）。
pub(crate) fn join_with_timeout(handle: JoinHandle<()>, what: &str) -> bool {
    let deadline = Instant::now() + STOP_TIMEOUT;
    while !handle.is_finished() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    if handle.is_finished() {
        let _ = handle.join();
        true
    } else {
        log::warn!("{what} 未在 {STOP_TIMEOUT:?} 内退出，detach（内核句柄可能残留）");
        false
    }
}
