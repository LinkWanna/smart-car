//! 两条管线共用的线程状态与 [`PreviewSource`] 实现。
//!
//! [`StreamInner`] 是「一次锁拿到全部」的共享状态（模型/预览统计 + 最新帧 +
//! 相机/模型元信息），CPU 与 VPSS 管线都写它；网页/控制侧通过
//! [`PreviewSource`] 只读访问，因此两条管线的对外表现完全一致。

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::preview::{
    CameraStatus, DetectionFrame, PreviewFrame, PreviewSource, VisionStatus, VisionTimings,
};

use super::VisionConfig;

/// 投给预览线程的一帧：数据（YUYV 或 RGB 平面）+ 同帧检测帧。
pub(crate) struct EncodePacket {
    pub(crate) data: Vec<u8>,
    pub(crate) frame: Arc<DetectionFrame>,
}

/// 线程共享状态（一次锁拿到全部，避免撕裂读）。
///
/// 两条管线（CPU 版 [`VisionStream`] 与 VPSS 版 `VpssStream`）共用这份状态、
/// 状态更新方法与 [`PreviewSource`] 实现，所以网页/控制侧看到的字段完全一致。
#[derive(Default)]
pub(crate) struct StreamState {
    /// 模型帧（采集/推理）统计。
    pub(crate) frames: u64,
    pub(crate) fps: f64,
    pub(crate) last_at: Option<Instant>,
    pub(crate) latest_dets: Option<Arc<DetectionFrame>>,
    /// 预览投递计数（诊断：模型帧 → 预览线程）。
    pub(crate) preview_sent: u64,
    pub(crate) preview_dropped: u64,
    pub(crate) preview_published: u64,
    pub(crate) encode_ms_avg: f64,
    /// 预览（JPEG）统计；`frame_if_new` 只认它。
    pub(crate) preview: Option<Arc<PreviewFrame>>,
    pub(crate) preview_at: Option<Instant>,
    pub(crate) preview_fps: f64,
    pub(crate) encode_ms: f64,
    /// 预览编码后端（`hw` / `sw` / `vpss`）。
    pub(crate) encode: String,
    /// 实际使用的相机节点（USB 重新枚举后会变）。
    pub(crate) device: String,
    pub(crate) camera_format: String,
    pub(crate) available: bool,
    pub(crate) camera_error: Option<String>,
    pub(crate) model_ok: bool,
    pub(crate) model: String,
    pub(crate) model_input: String,
    pub(crate) infer_ms: f64,
    pub(crate) nms_ms: f64,
    pub(crate) dets: usize,
    pub(crate) error: Option<String>,
}

impl StreamState {
    pub(crate) fn model_age_ms(&self) -> Option<u64> {
        self.last_at.map(|at| at.elapsed().as_millis() as u64)
    }

    pub(crate) fn preview_age_ms(&self) -> Option<u64> {
        self.preview_at.map(|at| at.elapsed().as_millis() as u64)
    }

    /// 记一帧模型结果（帧数 / EMA 帧率 / 耗时分项 / 最新检测帧）。
    pub(crate) fn record_model_frame(
        &mut self,
        now: Instant,
        timings: &VisionTimings,
        frame: Arc<DetectionFrame>,
    ) {
        if let Some(prev) = self.last_at {
            let dt = now.saturating_duration_since(prev).as_secs_f64();
            if dt > 0.0 {
                self.fps = if self.fps > 0.0 {
                    self.fps * 0.8 + (1.0 / dt) * 0.2
                } else {
                    1.0 / dt
                };
            }
        }
        self.frames += 1;
        self.last_at = Some(now);
        self.infer_ms = timings.infer_ms;
        self.nms_ms = timings.nms_ms;
        self.dets = frame.dets.len();
        self.latest_dets = Some(frame);
    }

    /// 记一帧已发布的预览（投递/发布计数、编码耗时 EMA、预览帧率）。
    pub(crate) fn record_preview(
        &mut self,
        frame: Arc<PreviewFrame>,
        encode_ms: f64,
        now: Instant,
    ) {
        self.preview_published += 1;
        self.encode_ms_avg = if self.encode_ms_avg > 0.0 {
            self.encode_ms_avg * 0.9 + encode_ms * 0.1
        } else {
            encode_ms
        };
        if let Some(prev) = self.preview_at {
            let dt = now.saturating_duration_since(prev).as_secs_f64();
            if dt > 0.0 {
                self.preview_fps = if self.preview_fps > 0.0 {
                    self.preview_fps * 0.8 + (1.0 / dt) * 0.2
                } else {
                    1.0 / dt
                };
            }
        }
        self.preview_at = Some(now);
        self.encode_ms = encode_ms;
        self.preview = Some(frame);
    }
}

pub(crate) struct StreamInner {
    pub(crate) cfg: VisionConfig,
    pub(crate) state: Mutex<StreamState>,
    /// 预览线程要的数据格式：true = YUYV（硬件编码），false = RGB 平面（软件编码）。
    pub(crate) preview_yuyv: AtomicBool,
}

impl StreamInner {
    pub(crate) fn new(cfg: VisionConfig) -> Self {
        Self {
            cfg,
            state: Mutex::new(StreamState::default()),
            // 先按软件编码（RGB 平面）兜底，编码线程定下后端后会更新
            preview_yuyv: AtomicBool::new(false),
        }
    }
}

impl PreviewSource for StreamInner {
    fn frame_if_new(&self, after: u64) -> Option<Arc<PreviewFrame>> {
        let state = self.state.lock().unwrap();
        match &state.preview {
            Some(frame) if frame.seq > after => Some(Arc::clone(frame)),
            _ => None,
        }
    }

    fn latest(&self) -> Option<Arc<PreviewFrame>> {
        self.state.lock().unwrap().preview.clone()
    }

    fn camera_status(&self) -> CameraStatus {
        let state = self.state.lock().unwrap();
        CameraStatus {
            available: state.available,
            device: if state.device.is_empty() {
                self.cfg.device.clone()
            } else {
                state.device.clone()
            },
            format: state.camera_format.clone(),
            frames: state.frames,
            fps: if state.preview_fps > 0.0 {
                state.preview_fps
            } else {
                state.fps
            },
            age_ms: state.preview_age_ms().or_else(|| state.model_age_ms()),
            error: state.camera_error.clone(),
        }
    }

    fn vision_status(&self) -> Option<VisionStatus> {
        let state = self.state.lock().unwrap();
        Some(VisionStatus {
            model_ok: state.model_ok,
            model: state.model.clone(),
            input: state.model_input.clone(),
            encode: state.encode.clone(),
            sent: state.preview_sent,
            dropped: state.preview_dropped,
            published: state.preview_published,
            encode_avg_ms: state.encode_ms_avg,
            fps: state.fps,
            age_ms: state.model_age_ms(),
            infer_ms: state.infer_ms,
            nms_ms: state.nms_ms,
            encode_ms: state.encode_ms,
            dets: state.dets,
            error: state.error.clone(),
        })
    }

    fn latest_detections(&self) -> Option<Arc<DetectionFrame>> {
        self.state.lock().unwrap().latest_dets.clone()
    }
}
