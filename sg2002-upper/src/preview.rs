//! 网页/控制与视觉之间的中性契约：一帧预览 = JPEG 画面 + **同帧**的检测框。
//!
//! 两个实现（`smartcar` 按硬件可用性二选一）：
//! - [`crate::vision::VisionStream`]：CPU 管线（YUYV→RGB 转换 + 编码线程）；
//! - [`crate::vision::VpssStream`]：VPSS 硬件管线（硬件 CSC + VENC 硬编）。
//!
//! 两者都是「相机 YUYV（模型输入格式）采集 + 编码」，画面与检测严格同帧。
//! 这里只传**检测框本身**（像素坐标）与统计信息，不做任何高层语义 ——
//! 分区/距离/控制律输入由消费方自己算（追踪侧见 [`crate::control::position`]），
//! 网页侧只画框、只显示统计。网页层只依赖本模块的抽象，不关心相机怎么打开。

use std::sync::Arc;
use std::time::Instant;

use crate::yolo::Detection;

/// 一帧检测结果（与 [`PreviewFrame`] 里的 JPEG 同帧；像素坐标 = 模型输入帧）。
#[derive(Debug, Clone)]
pub struct DetectionFrame {
    /// 帧序号（与预览帧一致）。
    pub seq: u64,
    /// 采集时刻（自动模式据此判断观测是否过期）。
    pub at: Instant,
    /// 检测框（像素坐标）。
    pub dets: Vec<Detection>,
}

/// 一帧的处理耗时（ms）。
#[derive(Debug, Clone, Copy, Default)]
pub struct VisionTimings {
    pub capture_ms: f64,
    pub preprocess_ms: f64,
    pub infer_ms: f64,
    pub nms_ms: f64,
    pub encode_ms: f64,
    pub total_ms: f64,
}

/// 一帧预览：JPEG + 同帧检测（MJPG 直出源没有检测结果）。
#[derive(Debug, Clone)]
pub struct PreviewFrame {
    pub seq: u64,
    pub jpeg: Option<Arc<[u8]>>,
    /// 同帧检测（网页据此画覆盖框）。
    pub dets: Option<Arc<DetectionFrame>>,
}

/// 相机/预览状态（对应 JSON 里的 `camera` 字段）。
#[derive(Debug, Clone)]
pub struct CameraStatus {
    pub available: bool,
    pub device: String,
    pub format: String,
    pub frames: u64,
    pub fps: f64,
    pub age_ms: Option<u64>,
    pub error: Option<String>,
}

/// 视觉状态（对应 JSON 里的 `vision` 字段）。
#[derive(Debug, Clone)]
pub struct VisionStatus {
    /// 模型是否可用（false = 仅预览，自动模式不可用）。
    pub model_ok: bool,
    pub model: String,
    /// 模型输入张量形状（如 `[1, 3, 480, 640]`）。
    pub input: String,
    /// 预览编码后端：`hw`（VENC 硬件）/ `sw`（纯 Rust）/ `vpss+bind`（VPSS 直连 VENC）
    /// / `none`。
    pub encode: String,
    /// 预览投递成功/丢弃/已发布计数（诊断用）。
    pub sent: u64,
    pub dropped: u64,
    pub published: u64,
    /// 平均编码耗时（EMA，ms）。
    pub encode_avg_ms: f64,
    pub fps: f64,
    pub age_ms: Option<u64>,
    pub infer_ms: f64,
    pub nms_ms: f64,
    pub encode_ms: f64,
    /// 最近一帧的检测框数量。
    pub dets: usize,
    /// 模型/推理错误。
    pub error: Option<String>,
}

/// 网页预览源。
pub trait PreviewSource: Send + Sync {
    /// 比 `after` 新的一帧；没有新帧返回 `None`。
    fn frame_if_new(&self, after: u64) -> Option<Arc<PreviewFrame>>;

    /// 最新一帧（HTTP 快照用）。
    fn latest(&self) -> Option<Arc<PreviewFrame>>;

    /// 相机状态。
    fn camera_status(&self) -> CameraStatus;

    /// 视觉状态；纯画面源（MJPG）返回 `None`。
    fn vision_status(&self) -> Option<VisionStatus>;

    /// 控制线程用：最新一帧检测（像素坐标）；纯画面源返回 `None`。
    fn latest_detections(&self) -> Option<Arc<DetectionFrame>> {
        None
    }

    /// 停止采集并回收资源；幂等，只停自己的线程（默认什么都不做，纯画面源自己实现）。
    fn stop(&self) {}

    /// 自动模式是否可用：有视觉 + 模型可用 + 画面新鲜。
    fn auto_ready(&self) -> bool {
        self.vision_status()
            .is_some_and(|v| v.model_ok && v.age_ms.is_some_and(|ms| ms < 1000))
    }
}
