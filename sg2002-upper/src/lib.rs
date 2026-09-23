//! SG2002 上位机：视觉检测（相机 + TPU）与 ESP32-C3 下位机的串口控制。
//!
//! 模块分层：
//! - 感知：`camera`（V4L2 零拷贝）、`preprocess`（YUYV→RGB）、`tpu`（推理 + NMS）、
//!   `position`（位置/距离分级）、`vision`（视觉核心 + 后台线程 + 同帧预览）；
//! - 下位机通信与控制：`control`（`protocol` 线协议——与固件共用的
//!   `protocol` crate、`serial` 串口、`car` 链路 + 安全看门狗、
//!   `servo` 视觉伺服控制律）。
//! - 网页遥控：`web`（零依赖 HTTP/WebSocket、手动遥控控制律、MJPEG 预览、
//!   手动/自动仲裁）、`preview`（预览与视觉之间的中性契约）。
//! - 观测：`stats`（管线耗时统计）、`logging`（`log` + `simple_logger` 初始化）。
//! - 硬件编解码：`hwjpeg`（SG2002 VENC 硬件 JPEG，封装在 `cvimpi-rs` 里，
//!   编解码会话/通道/VB 池都由它管理；打不开硬件时上层降级到软件编码）。
//!
//! 入口 bin：`smartcar`（整合：视觉 + 网页 + 手动/自动）、`webctl`（只遥控）、
//! `pipeline`（只视觉）。
//!
//! `camera` / `tpu` / `position` / `hwjpeg` 依赖板端 C 库与 V4L2，随 crate
//! 无条件编译：板端直接运行，主机上可用 `cargo check` 做编译检查
//! （链接/运行需要厂商库，只在板端进行）。

pub mod camera;
pub mod control;
pub mod hwjpeg;
pub mod logging;
pub mod position;
pub mod preprocess;
pub mod preview;
pub mod stats;
pub mod tpu;
pub mod vision;
pub mod web;
