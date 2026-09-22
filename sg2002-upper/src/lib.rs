//! SG2002 上位机：视觉检测（相机 + TPU）与 ESP32-C3 下位机的串口控制。
//!
//! 模块分层：
//! - 感知：`camera`（V4L2 零拷贝）、`preprocess`（YUYV→RGB）、`tpu`（推理 + NMS）、
//!   `position`（位置/距离分级）。
//! - 下位机通信与控制：`control`（`protocol` 线协议——与固件共用的
//!   `smart-car-protocol` crate、`serial` 串口、`car` 链路 + 安全看门狗、
//!   `servo` 视觉伺服控制律）。
//! - 网页遥控：`web`（零依赖 HTTP/WebSocket、play.py 手感的遥控控制律、
//!   MJPEG 预览）。
//! - 观测：`stats`（管线耗时统计）。
//!
//! `camera` / `tpu` / `position` 依赖板端 C 库与 V4L2，随 crate 无条件编译：
//! 板端直接运行，主机上可用 `cargo check` 做编译检查。

pub mod camera;
pub mod control;
pub mod position;
pub mod preprocess;
pub mod stats;
pub mod tpu;
pub mod web;
