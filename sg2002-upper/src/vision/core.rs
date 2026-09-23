//! 单帧步进的视觉核心：[`Vision`]（相机 → 预处理 → 推理）。

use std::io;
use std::time::Instant;

use cvimpi_rs::encoder::Frame;
use cvimpi_rs::ffi;
use cvimpi_rs::sys::Sys;

use crate::camera::{Camera, open_yuyv};
use crate::preprocess::{FRAME_H, FRAME_W, PreviewInput, yuyv422_to_rgb, yuyv422_to_rgb_planes};
use crate::preview::VisionTimings;
use crate::yolo::Yolo;

use super::{VisionConfig, VisionStep, elapsed_ms};

/// 模型输入的像素格式（与 VPSS chn0 一致）。
const INPUT_FORMAT: ffi::PIXEL_FORMAT_E = ffi::PIXEL_FORMAT_RGB_888_PLANAR;

/// RGB 平面 CHW（模型输入）的去向。
enum RgbInput<'a> {
    /// VB 帧：写完把物理地址交给 TPU（零拷贝）。
    ///
    /// `Box`：`Frame` 比另一分支大一个量级，装进 `Box` 让枚举保持小体积。
    Vb(Box<Frame<'a>>),
    /// 没有 MMF 会话时的 CPU 缓冲：只能给软件预览编码，不能推理。
    Cpu(Vec<u8>),
}

impl RgbInput<'_> {
    /// YUYV422 → RGB 平面 CHW（直接写进去，无中间缓冲）。
    fn write(&mut self, yuyv: &[u8]) {
        match self {
            Self::Vb(frame) => {
                // 模型要 NCHW 紧凑布局：stride 必须等于宽度（RGB_888_PLANAR
                // 640x480 的 `venc_input_layout` 保证如此）。
                debug_assert_eq!(frame.stride(0) as usize, FRAME_W);
                let [r, g, b] = frame.planes_mut();
                yuyv422_to_rgb_planes(yuyv, r, g, b);
            }
            Self::Cpu(buf) => yuyv422_to_rgb(yuyv, buf),
        }
    }

    /// 送 TPU 前刷 cache（cached 映射的 VB 帧；CPU 缓冲 no-op）。
    fn flush(&self) -> io::Result<()> {
        match self {
            Self::Vb(frame) => frame
                .flush()
                .map_err(|e| io::Error::other(format!("RGB 帧 flush 失败: {e}"))),
            Self::Cpu(_) => Ok(()),
        }
    }

    /// 物理地址（CPU 缓冲没有）。
    fn phy_addr(&self) -> Option<u64> {
        match self {
            Self::Vb(frame) => Some(frame.phy_addr(0)),
            Self::Cpu(_) => None,
        }
    }

    /// 紧凑 RGB 平面（软件预览编码用；VB 帧下一帧会被覆盖，必须拷出去）。
    fn tight(&self) -> Vec<u8> {
        match self {
            Self::Vb(frame) => {
                let stride = frame.stride(0) as usize;
                let mut out = Vec::with_capacity(FRAME_W * FRAME_H * 3);
                for plane in frame.planes() {
                    if stride == FRAME_W {
                        out.extend_from_slice(&plane[..FRAME_W * FRAME_H]);
                    } else {
                        for y in 0..FRAME_H {
                            out.extend_from_slice(&plane[y * stride..y * stride + FRAME_W]);
                        }
                    }
                }
                out
            }
            Self::Cpu(buf) => buf.clone(),
        }
    }
}

/// 单帧步进的视觉核心。
pub struct Vision<'a> {
    cfg: VisionConfig,
    /// 实际使用的相机节点（USB 重新枚举后编号会变，见 [`open_yuyv`]）。
    device: String,
    camera: Camera,
    /// 模型输入（零拷贝 VB 帧；无会话时为 CPU 兜底缓冲）。
    input: RgbInput<'a>,
    /// `None` = 模型不可用（仅预览）。
    yolo: Option<Yolo>,
    seq: u64,
    model_error: Option<String>,
    model_input: String,
}

impl<'a> Vision<'a> {
    /// 打开相机并加载模型；相机失败返回 `Err`，会话/模型失败降级为仅预览。
    pub fn new(cfg: VisionConfig, session: Option<&'a Sys>) -> io::Result<Self> {
        let camera = open_yuyv(&cfg.device)?;
        let device = camera.device().to_string();
        let (input, yolo, model_error, model_input) = match session {
            Some(sys) => init_inference(&cfg, sys),
            None => (
                RgbInput::Cpu(vec![0u8; FRAME_W * FRAME_H * 3]),
                None,
                Some("MMF 会话不可用（零拷贝输入帧无法分配）".to_string()),
                "-".to_string(),
            ),
        };
        Ok(Self {
            cfg,
            device,
            camera,
            input,
            yolo,
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
        self.yolo.is_some()
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

        // 2) 预处理：YUYV422 → RGB 平面 CHW，直接写进模型输入（唯一一次像素写）
        let t1 = Instant::now();
        self.input.write(frame.as_slice());
        // 预览线程要的数据在这里拷：YUYV 需要在归还相机缓冲前取，
        // RGB 平面下一帧会被覆盖
        let preview = match preview {
            None => None,
            Some(PreviewInput::RgbPlanar) => Some(self.input.tight()),
            Some(PreviewInput::Yuyv) => Some(frame.as_slice().to_vec()),
        };
        // 立即归还相机缓冲，让驱动尽早复用
        frame.release()?;
        // cached 映射：送 TPU 前刷 cache（CPU 兜底缓冲 no-op）
        self.input.flush()?;
        let preprocess_ms = elapsed_ms(t1);

        // 3) 推理（零拷贝：输入张量直接指向 VB 帧的物理地址）
        let mut infer_error: Option<String> = None;
        let (dets, infer_ms, nms_ms) = match (self.yolo.as_mut(), self.input.phy_addr()) {
            (Some(yolo), Some(paddr)) => match yolo.infer(paddr) {
                Ok(dets) => {
                    let (tpu, nms) = yolo.last_timing();
                    (dets, tpu, nms)
                }
                Err(e) => {
                    infer_error = Some(e.to_string());
                    (Vec::new(), 0.0, 0.0)
                }
            },
            _ => (Vec::new(), 0.0, 0.0),
        };
        if let Some(e) = infer_error {
            self.model_error = Some(e);
            self.yolo = None;
        }

        // 4) 检测框（位置/距离等语义由追踪侧自己算）+ 预览数据（第 2 步已拷好）
        let seq = self.seq;
        self.seq += 1;
        Ok(VisionStep {
            seq,
            at: t_frame,
            dets,
            preview,
            timings: VisionTimings {
                capture_ms,
                preprocess_ms,
                infer_ms,
                nms_ms,
                encode_ms: 0.0,
                total_ms: elapsed_ms(t_frame),
            },
        })
    }
}

/// 分配零拷贝输入帧 + 加载模型；任何一步失败都只降级为「仅预览」。
fn init_inference<'a>(
    cfg: &VisionConfig,
    sys: &'a Sys,
) -> (RgbInput<'a>, Option<Yolo>, Option<String>, String) {
    // 1) RGB 平面帧：cached 映射（CPU 逐字节写入快），送 TPU 前 flush
    let frame = match sys.alloc_frame_cached(FRAME_W as u32, FRAME_H as u32, INPUT_FORMAT) {
        Ok(frame) => frame,
        Err(e) => {
            return (
                RgbInput::Cpu(vec![0u8; FRAME_W * FRAME_H * 3]),
                None,
                Some(format!("RGB 输入帧分配失败: {e}")),
                "-".to_string(),
            );
        }
    };
    // RGB 三平面等长，总字节数 = 模型输入字节数（NCHW 布局）
    let frame_bytes = frame.plane_len(0) as usize * 3;

    // 2) 模型：加载失败/输入尺寸不匹配都只降级为「仅预览」
    let (yolo, model_error, model_input) = match Yolo::from_file(
        &cfg.model,
        cfg.conf_threshold,
        cfg.iou_threshold,
        vec![cfg.label.clone()],
    ) {
        Ok(yolo) => {
            let input = format!("{:?}", yolo.input_shape());
            if yolo.input_bytes() == frame_bytes {
                (Some(yolo), None, input)
            } else {
                let msg = format!(
                    "模型输入 {} 字节与 RGB 帧 {frame_bytes} 字节不一致，零拷贝不可用",
                    yolo.input_bytes()
                );
                (None, Some(msg), input)
            }
        }
        Err(e) => (None, Some(e.to_string()), "-".to_string()),
    };
    (
        RgbInput::Vb(Box::new(frame)),
        yolo,
        model_error,
        model_input,
    )
}
