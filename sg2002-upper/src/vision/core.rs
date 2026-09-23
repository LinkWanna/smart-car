//! 单帧步进的视觉核心：[`Vision`]（相机 → 预处理 → 推理 → 位置分析）。

use std::io;
use std::time::Instant;

use crate::camera::{Camera, open_yuyv};
use crate::position::PositionAnalyzer;
use crate::preprocess::{Preprocessor, PreviewInput};
use crate::preview::VisionTimings;
use crate::tpu::TpuInference;

use super::{FRAME_H, FRAME_W, VisionConfig, VisionStep, elapsed_ms};

/// 单帧步进的视觉核心。
pub struct Vision {
    cfg: VisionConfig,
    /// 实际使用的相机节点（USB 重新枚举后编号会变，见 [`open_yuyv`]）。
    device: String,
    camera: Camera,
    pp: Preprocessor,
    /// `None` = 模型不可用（仅预览）。
    infer: Option<TpuInference>,
    pos: PositionAnalyzer,
    seq: u64,
    model_error: Option<String>,
    model_input: String,
}

impl Vision {
    /// 打开相机并加载模型；相机失败返回 `Err`，模型失败降级为仅预览。
    pub fn new(cfg: VisionConfig) -> io::Result<Self> {
        let camera = open_yuyv(&cfg.device)?;
        let device = camera.device().to_string();
        let (infer, model_error, model_input) = match TpuInference::try_new(
            &cfg.model,
            cfg.conf_threshold,
            cfg.iou_threshold,
            vec![cfg.label.clone()],
        ) {
            Ok(infer) => {
                let input = format!("{:?}", infer.input_shape());
                (Some(infer), None, input)
            }
            Err(e) => (None, Some(e.to_string()), "-".to_string()),
        };
        let pos = PositionAnalyzer::for_640x480();
        Ok(Self {
            cfg,
            device,
            camera,
            pp: Preprocessor::new(FRAME_W, FRAME_H),
            infer,
            pos,
            seq: 0,
            model_error,
            model_input,
        })
    }

    /// 驱动实际协商到的像素格式（应为 `YUYV`）。
    pub fn camera_format(&self) -> String {
        self.camera.pixel_format()
    }

    pub fn model_ok(&self) -> bool {
        self.infer.is_some()
    }

    pub fn model_error(&self) -> Option<&str> {
        self.model_error.as_deref()
    }

    pub fn model_input(&self) -> &str {
        &self.model_input
    }

    pub fn device(&self) -> &str {
        &self.device
    }

    pub fn model_path(&self) -> &str {
        &self.cfg.model
    }

    /// 采集并处理一帧；`preview` 决定给预览线程带哪份数据
    /// （硬件编码要 [`PreviewInput::Yuyv`]，软件编码要 [`PreviewInput::RgbPlanar`]，
    /// `None` = 不带预览）。
    pub fn step(&mut self, preview: Option<PreviewInput>) -> io::Result<VisionStep> {
        let t_frame = Instant::now();

        // 1) 采集（零拷贝）
        let t0 = Instant::now();
        let frame = self.camera.get_frame()?;
        let capture_ms = elapsed_ms(t0);

        // 2) 预处理：YUYV422 → RGB 平面 CHW（模型输入）
        let t1 = Instant::now();
        let planar = self.pp.process_yuyv(frame.as_slice(), FRAME_W, FRAME_H);
        // 预览线程要的数据在这里拷（YUYV 需要在归还相机缓冲前取）
        let preview = match preview {
            None => None,
            Some(PreviewInput::RgbPlanar) => Some(planar.to_vec()),
            Some(PreviewInput::Yuyv) => Some(frame.as_slice().to_vec()),
        };
        // 立即归还相机缓冲，让驱动尽早复用（planar 借的是预处理缓冲，不受影响）
        frame.release()?;
        let preprocess_ms = elapsed_ms(t1);

        // 3) 推理（模型不可用/中途失败 → 空检测 + 记错误）
        let mut infer_error: Option<String> = None;
        let (dets, infer_ms, nms_ms) = match self.infer.as_mut() {
            Some(infer) => match infer.try_infer(planar) {
                Ok(dets) => {
                    let (tpu, nms) = infer.last_timing();
                    (dets, tpu, nms)
                }
                Err(e) => {
                    infer_error = Some(e.to_string());
                    (Vec::new(), 0.0, 0.0)
                }
            },
            None => (Vec::new(), 0.0, 0.0),
        };
        if let Some(e) = infer_error {
            self.model_error = Some(e);
            self.infer = None;
        }

        // 4) 位置分析
        let t2 = Instant::now();
        let result = self.pos.analyze(&dets);
        let position_ms = elapsed_ms(t2);

        // 5) 位置分析后的预览数据已在第 2 步拷好
        let seq = self.seq;
        self.seq += 1;
        Ok(VisionStep {
            seq,
            at: t_frame,
            result,
            preview,
            timings: VisionTimings {
                capture_ms,
                preprocess_ms,
                infer_ms,
                nms_ms,
                position_ms,
                encode_ms: 0.0,
                total_ms: elapsed_ms(t_frame),
            },
        })
    }
}
