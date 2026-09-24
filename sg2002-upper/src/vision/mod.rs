//! 视觉域：相机 YUYV → VPSS 硬件 CSC → TPU 推理 →（同帧）JPEG 预览。
//!
//! 视觉只负责「检测框」：本模块的输出是 [`crate::yolo::Detection`]（像素坐标）
//! 与统计信息，不做分区/距离/控制律输入这类高层语义（那是追踪侧的事，
//! 见 [`crate::control::position`]）。
//!
//! 唯一管线 [`VpssStream`]（`vpss` 子模块）：单线程，相机 YUYV 只 memcpy 一次
//! 进 VB 块，硬件 CSC 出 RGB 平面（TPU 零拷贝）与 NV12（VENC 硬编预览）。
//! 相机/会话/建组/bind/模型任何一步失败都返回错误，由入口报错退出——没有
//! 降级或回退路径。
//!
//! 子模块分工：
//!
//! - `camera`：V4L2 零拷贝采集（YUYV 节点发现 + mmap 帧）；
//! - `preview`：与网页/控制之间的中性契约（JPEG + 同帧检测框）；
//! - `state`：线程状态（`StreamInner`/`StreamState`）与
//!   [`crate::vision::preview::PreviewSource`] 实现；
//! - `vpss`：会话/组/VENC 生命周期与帧循环。
//!
//! 相机必须以模型需要的 **YUYV422** 打开；模型输入（硬件 CSC 出的 RGB 平面）与
//! 预览 JPEG 都由同一帧得到，因此不需要第二路相机。

use std::time::Instant;

use self::preview::{DetectionFrame, VisionTimings};
use crate::yolo::Detection;

pub mod camera;
pub mod preview;

mod state;
mod vpss;

pub use crate::preprocess::{FRAME_H, FRAME_W};
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
        }
    }
}

/// 一帧的处理结果。
pub struct VisionStep {
    pub seq: u64,
    pub at: Instant,
    /// 检测框（像素坐标）。
    pub dets: Vec<Detection>,
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
