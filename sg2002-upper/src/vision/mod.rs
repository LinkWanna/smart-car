//! 视觉域：相机 YUYV → 预处理 → TPU 推理 →（同帧）JPEG 预览。
//!
//! 视觉只负责「检测框」：本模块的输出是 [`crate::yolo::Detection`]（像素坐标）
//! 与统计信息，不做分区/距离/控制律输入这类高层语义（那是追踪侧的事，
//! 见 [`crate::control::position`]）。
//!
//! 两条管线，共用本模块的状态与契约（选择在 `smartcar`，VPSS 不可用时回退 CPU）：
//!
//! - [`VisionStream`]（`cpu` 子模块）：CPU 版，采集/推理一个线程、JPEG 编码另一个线程。
//!   采集线程把**模型真正消费的那一帧**（YUYV 或 RGB 缓冲拷贝 + 检测帧）投给
//!   编码线程，编码线程编码完把 `(JPEG, 同帧检测)` 发布给网页；
//! - [`VpssStream`]（`vpss` 子模块）：VPSS 硬件版，单线程：相机 YUYV 只 memcpy 一次
//!   进 VB 块，硬件 CSC 出 RGB 平面（TPU 零拷贝）与 NV12（VENC 硬编预览）。
//!
//! 两者都实现 [`crate::preview::PreviewSource`]，网页/控制侧看到的接口与状态完全一致。
//!
//! 子模块分工：
//!
//! - `core`：[`Vision`] 单帧步进（采集 → 预处理 → 推理）；
//! - `state`：两条管线共用的线程状态（`StreamInner`/`StreamState`）与
//!   [`crate::preview::PreviewSource`] 实现；
//! - `cpu`：CPU 管线的线程编排；
//! - `vpss`：VPSS 管线的会话/组/VENC 生命周期与帧循环。
//!
//! 相机必须以模型需要的 **YUYV422** 打开；模型输入（YUYV→RGB 平面）与预览 JPEG
//! 都由同一帧得到，因此不需要第二路相机。模型加载失败不致命：退化为「仅预览」
//! （`model_ok == false`），手动遥控照常，自动模式不可用（见 [`crate::preview::PreviewSource::auto_ready`]）。

use std::time::Instant;

use crate::preview::{DetectionFrame, VisionTimings};
use crate::yolo::Detection;

mod core;
mod cpu;
mod state;
mod vpss;

pub use crate::preprocess::{FRAME_H, FRAME_W};
pub use core::Vision;
pub use cpu::VisionStream;
pub use vpss::VpssStream;

/// 视觉配置。
#[derive(Debug, Clone)]
pub struct VisionConfig {
    /// 相机设备（模型要 YUYV422）。
    pub device: String,
    /// 模型路径。
    pub model: String,
    pub conf_threshold: f32,
    pub iou_threshold: f32,
    /// 单类模型的标签名。
    pub label: String,
    /// 预览 JPEG 质量（1..=100）。
    pub quality: u8,
    /// 预览降采样倍数（1 = 原尺寸，2 = 320x240，省 CPU）。
    pub scale: usize,
    /// 预览帧率上限（编码线程的投递节拍；0 = 不限制）。
    pub preview_fps: f32,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            device: "/dev/video0".to_string(),
            model: String::new(),
            conf_threshold: 0.5,
            iou_threshold: 0.45,
            label: "tennis_ball".to_string(),
            quality: 75,
            scale: 1,
            preview_fps: 10.0,
        }
    }
}

/// 一帧的处理结果。
pub struct VisionStep {
    pub seq: u64,
    pub at: Instant,
    /// 检测框（像素坐标）。
    pub dets: Vec<Detection>,
    /// 给预览线程的数据（按 `PreviewInput` 拷贝，`None` = 不带）。
    pub preview: Option<Vec<u8>>,
    pub timings: VisionTimings,
}

/// 由一帧结果构造检测帧（控制线程与网页共用；与预览 JPEG 同帧）。
pub fn detection_frame(step: &VisionStep) -> DetectionFrame {
    DetectionFrame {
        seq: step.seq,
        at: step.at,
        dets: step.dets.clone(),
    }
}

pub(crate) fn elapsed_ms(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detection_frame_keeps_frame_pairing() {
        let det = Detection {
            label: "tennis_ball".into(),
            confidence: 0.9,
            x1: 160.0,
            y1: 120.0,
            x2: 320.0,
            y2: 240.0,
        };
        let step = VisionStep {
            seq: 7,
            at: Instant::now(),
            dets: vec![det],
            preview: None,
            timings: VisionTimings::default(),
        };
        let frame = detection_frame(&step);
        assert_eq!(frame.seq, 7);
        assert_eq!(frame.at, step.at);
        assert_eq!(frame.dets.len(), 1);
        assert_eq!(frame.dets[0].label, "tennis_ball");
        assert!((frame.dets[0].confidence - 0.9).abs() < 1e-6);
        assert!((frame.dets[0].x1 - 160.0).abs() < 1e-6);
    }
}
