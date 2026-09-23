//! 视觉域：相机 YUYV → 预处理 → TPU 推理 → 位置分析 →（同帧）JPEG 预览。
//!
//! 两条管线，共用本模块的状态与契约（选择在 `smartcar`，VPSS 不可用时回退 CPU）：
//!
//! - [`VisionStream`]（`cpu` 子模块）：CPU 版，采集/推理一个线程、JPEG 编码另一个线程。
//!   采集线程把**模型真正消费的那一帧**（YUYV 或 RGB 缓冲拷贝 + 检测快照）投给
//!   编码线程，编码线程编码完把 `(JPEG, 同帧快照)` 发布给网页；
//! - [`VpssStream`]（`vpss` 子模块）：VPSS 硬件版，单线程：相机 YUYV 只 memcpy 一次
//!   进 VB 块，硬件 CSC 出 RGB 平面（TPU 零拷贝）与 NV12（VENC 硬编预览）。
//!
//! 两者都实现 [`crate::preview::PreviewSource`]，网页/控制侧看到的接口与状态完全一致。
//!
//! 子模块分工：
//!
//! - `core`：[`Vision`] 单帧步进（采集 → 预处理 → 推理 → 位置分析）；
//! - `state`：两条管线共用的线程状态（`StreamInner`/`StreamState`）与
//!   [`crate::preview::PreviewSource`] 实现；
//! - `cpu`：CPU 管线的线程编排；
//! - `vpss`：VPSS 管线的会话/组/VENC 生命周期与帧循环。
//!
//! 相机必须以模型需要的 **YUYV422** 打开；模型输入（`Preprocessor`）与预览 JPEG
//! 都由同一帧得到，因此不需要第二路相机。模型加载失败不致命：退化为「仅预览」
//! （`model_ok == false`），手动遥控照常，自动模式不可用（见 [`crate::preview::PreviewSource::auto_ready`]）。

use std::time::Instant;

use crate::position::{Observation, PositionResult};
use crate::preview::{DetectionBox, TargetInfo, VisionSnapshot, VisionTimings};

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
    pub result: PositionResult,
    /// 给预览线程的数据（按 `PreviewInput` 拷贝，`None` = 不带）。
    pub preview: Option<Vec<u8>>,
    pub timings: VisionTimings,
}

/// 由一帧结果构造网页/控制用的快照（归一化检测框 + 控制律输入 + 分级信息）。
pub fn snapshot(step: &VisionStep) -> VisionSnapshot {
    let observation = Observation::from_result(&step.result);
    let dets = step
        .result
        .all_detections
        .iter()
        .map(|d| DetectionBox {
            x1: d.x1 / FRAME_W as f32,
            y1: d.y1 / FRAME_H as f32,
            x2: d.x2 / FRAME_W as f32,
            y2: d.y2 / FRAME_H as f32,
            confidence: d.confidence,
        })
        .collect();
    let target = step.result.has_target().then(|| TargetInfo {
        err_x: observation.err_x,
        zone: step.result.zone.clone(),
        distance: step.result.distance.clone(),
        confidence: step.result.target_confidence,
    });
    VisionSnapshot {
        seq: step.seq,
        at: step.at,
        dets,
        observation,
        target,
        timings: step.timings,
    }
}

pub(crate) fn elapsed_ms(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tpu::Detection;

    #[test]
    fn snapshot_normalizes_detections() {
        let det = Detection {
            label: "tennis_ball".into(),
            confidence: 0.9,
            x1: 160.0,
            y1: 120.0,
            x2: 320.0,
            y2: 240.0,
        };
        let result = PositionResult {
            target_class: "tennis_ball".into(),
            target_confidence: 0.9,
            center_x: 0.375,
            center_y: 0.375,
            size_ratio: 0.03,
            zone: "left_top".into(),
            distance: "far".into(),
            detection_count: 1,
            all_detections: vec![det],
        };
        let step = VisionStep {
            seq: 7,
            at: Instant::now(),
            result,
            preview: None,
            timings: VisionTimings::default(),
        };
        let snap = snapshot(&step);
        assert_eq!(snap.seq, 7);
        assert_eq!(snap.dets.len(), 1);
        let d = snap.dets[0];
        assert!((d.x1 - 0.25).abs() < 1e-6 && (d.y2 - 0.5).abs() < 1e-6);
        assert!(snap.target.is_some());
        assert!((snap.observation.err_x + 0.25).abs() < 1e-6);
    }
}
