//! SG2002 上位机：视觉检测（相机 + TPU）与 ESP32-C3 下位机的串口控制。
//!
//! 模块分层：
//! - 感知：`camera`（V4L2 零拷贝 + YUYV 节点发现）、`preprocess`（YUYV→RGB / YUYV→JPEG
//!   的像素工作流）、`tpu`（推理 + NMS）、`position`（位置/距离分级 + 控制观测）、
//!   `vision`（视觉域：单帧核心 + 两条管线 —— CPU `cpu` 与 VPSS 硬件 `vpss`，
//!   共用线程状态与同帧预览契约）；
//! - 下位机通信与控制：`control`（`protocol` 线协议——与固件共用的
//!   `protocol` crate、`serial` 串口、`car` 链路 + 安全看门狗、
//!   `servo` 视觉伺服控制律）。
//! - 网页遥控：`web`（零依赖 HTTP/WebSocket、手动遥控控制律、MJPEG 预览、
//!   手动/自动仲裁）、`preview`（预览与视觉之间的中性契约）。
//! - 观测：`logging`（`log` + `simple_logger` 初始化）。
//! - 硬件编解码：`hwjpeg`（SG2002 VENC 硬件 JPEG，封装在 `cvimpi-rs` 里，
//!   编解码会话/通道/VB 池都由它管理；打不开硬件时上层降级到软件编码）。
//!
//! 入口 bin：`smartcar`（整合：视觉 + 网页 + 手动/自动）、`vpss_probe`（上板探针）。
//!
//! `camera` / `tpu` / `position` / `hwjpeg` 依赖板端 C 库与 V4L2，随 crate
//! 无条件编译：板端直接运行，主机上可用 `cargo check` 做编译检查
//! （链接/运行需要厂商库，只在板端进行）。
//!
//! # 线程与生命周期
//!
//! 每个带线程的组件**自带停止标志**，[`stop`](preview::PreviewSource::stop)
//! 只停自己的线程，不碰别的组件：
//!
//! | 组件 | 线程 | 停止 |
//! | --- | --- | --- |
//! | [`control::Car`] | 串口收 + 发 | `Car::shutdown`（`Drop` 兜底） |
//! | [`control::ControlSession`] | 控制节拍 | `ControlSession::stop` |
//! | [`vision::VisionStream`] | 采集/推理 + 预览编码 | `stop`（`Drop` 兜底） |
//! | [`vision::VpssStream`] | 单线程管线 | `stop`（`Drop` 兜底） |
//! | [`web::Server`] | HTTP 池 + 每连接 WebSocket | `Server::run` 轮询内部标志 |
//!
//! 进程退出（`smartcar`）：信号处理器只把 [`web::Server::stop_flag`] 置 false →
//! `run` 返回 → 依次 `ControlSession::stop` → `preview.stop` → `Car::shutdown`。
//! 线程等待统一走 `join_with_timeout`（`STOP_TIMEOUT`），超时则 detach：
//! 硬件句柄可能来不及清理，内核里会留下 VB 池 / bind 节点（下次启动的
//! `Sys::init` 与 `clear_venc_bind` 会兜底，彻底恢复需要重启设备）。
//!
//! ## `unsafe 'static` 约定
//!
//! [`hwjpeg::HwJpeg`] 与 `vision::vpss::VpssPipeline` 都持有「借用会话的句柄 +
//! 会话本身」，用 `unsafe` 把借用延长成 `'static`（`cvimpi_rs::sys::Sys` 不能
//! 被句柄安全借用，见 `cvimpi-rs` 的说明）。两条硬约定：
//!
//! 1. 会话放在 `Box<Sys>` 里且**移动结构体不会移动它**（堆地址稳定）；
//! 2. 结构体字段声明顺序 = 析构顺序，`_sys` 必须声明在最后、最后析构。
//!
//! `HwJpeg` 还要求构造/使用/析构在同一线程（预览编码线程内部完成）；
//! `VpssPipeline` 在主线程构造、move 进 worker 线程析构，所以只依赖上面两条。

pub mod camera;
pub mod control;
pub mod hwjpeg;
pub mod logging;
pub mod position;
pub mod preprocess;
pub mod preview;
pub mod tpu;
pub mod vision;
pub mod web;

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
