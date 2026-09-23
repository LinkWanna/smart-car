//! VPSS 管线：相机 YUYV →（唯一一次 CPU 搬运）→ 硬件 CSC 一路进两路出：
//!
//! ```text
//! V4L2 YUYV ──memcpy 614KB──► VB 块 ──SendFrame──► VPSS grp0
//!                                                   ├─ chn0 RGB_888_PLANAR ─► TPU 零拷贝（物理地址直喂）
//!                                                   └─ chn1 NV12（可硬件缩放）─► VENC PT_JPEG ─► 网页
//! ```
//!
//! 与 CPU 版 [`crate::vision::VisionStream`] 的关系：
//! - 两者都实现 [`PreviewSource`]，网页/控制侧看到的接口与状态完全一致；
//! - 本模块**单线程**：采集、送帧、推理、JPEG 都在一个线程里顺序执行
//!   （硬件路径每帧 ~45ms，相机 16.5fps 的预算 60ms，够用），
//!   因此所有中间件句柄都由管线自己持有，没有跨线程共享；
//! - VPSS 建组失败 / 组号用尽 / 模型输入尺寸不匹配时，
//!   [`VpssStream::try_start`] 直接返回错误，由 `smartcar` 回退到 CPU 管线。
//!
//! 关键实现细节（真机验证，见 `bin/vpss_probe.rs`）：
//! - 组 CSC 要设成 **BT.601 limited → full**（`Vpss::set_yuv601_limited_to_full`），
//!   否则模型看到的是"没做量程扩张"的偏灰画面，置信度明显下降；
//! - chn0 的输出 stride = 640 且三平面物理地址连续，正好是 `[1,3,480,640]`
//!   的 NCHW 布局，可以 `Model::forward_physical` 零拷贝；
//! - 组号用一个少一个（驱动限制），所以 `VpssConfig::grp` 用 `VPSS_GRP_AUTO`。

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::{info, warn};

use cvimpi_rs::encoder::{Encoder, EncoderConfig, Frame};
use cvimpi_rs::ffi;
use cvimpi_rs::sys::{Sys, VbPoolConfig};
use cvimpi_rs::venc_input_layout;
use cvimpi_rs::vpss::{VPSS_GRP_AUTO, Vpss, VpssChnConfig, VpssConfig};

use crate::preview::{
    CameraStatus, PreviewFrame, PreviewSource, VisionSnapshot, VisionStatus, VisionTimings,
};
use crate::position::PositionAnalyzer;
use crate::tpu::TpuInference;
use crate::vision::{
    BOTTOM_BOUNDARY, FRAME_H, FRAME_W, LEFT_BOUNDARY, MID_THRESHOLD, NEAR_THRESHOLD,
    RIGHT_BOUNDARY, StreamInner, TOP_BOUNDARY, VisionConfig, open_camera, snapshot,
};

/// 预览队列深度（chn1 用户态取帧队列）。
const PREVIEW_DEPTH: u32 = 1;
/// `GetChnFrame` 超时（ms）。
const GET_FRAME_TIMEOUT_MS: i32 = 1000;
/// 采集失败后的退避。
const RETRY_BACKOFF: Duration = Duration::from_millis(200);

/// VPSS 视觉管线（实现 [`PreviewSource`]）。
pub struct VpssStream {
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    handle: Mutex<Option<JoinHandle<()>>>,
}

impl VpssStream {
    /// 启动 VPSS 管线：**同步**建好会话/组/VENC/相机/模型（任何一步失败都能回退），
    /// 之后线程只跑帧循环。
    pub fn try_start(cfg: VisionConfig, running: Arc<AtomicBool>) -> io::Result<Self> {
        let inner = Arc::new(StreamInner::new(cfg.clone()));
        let pipeline = VpssPipeline::new(cfg.clone())?;
        info!(
            "视觉：VPSS 管线（{}，{}，chn0 RGB 平面零拷贝 + chn1 NV12 硬编 → {}x{} 预览）",
            pipeline.camera.device(),
            pipeline.camera.pixel_format(),
            config_scale_w(&cfg),
            config_scale_h(&cfg),
        );
        let thread_inner = Arc::clone(&inner);
        let thread_running = Arc::clone(&running);
        let handle = thread::spawn(move || {
            let mut pipeline = pipeline;
            pipeline.run(&thread_inner, &thread_running);
        });
        Ok(Self {
            inner,
            running,
            handle: Mutex::new(Some(handle)),
        })
    }

    /// 停止采集并等线程退出；幂等。
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        let handle = self.handle.lock().unwrap().take();
        if let Some(handle) = handle {
            let deadline = Instant::now() + Duration::from_millis(800);
            while !handle.is_finished() && Instant::now() < deadline {
                thread::sleep(Duration::from_millis(5));
            }
            drop(handle); // 超时则 detach：句柄都归线程所有，不会有悬垂引用
        }
    }
}

impl Drop for VpssStream {
    fn drop(&mut self) {
        self.stop();
    }
}

impl PreviewSource for VpssStream {
    fn frame_if_new(&self, after: u64) -> Option<Arc<PreviewFrame>> {
        self.inner.frame_if_new(after)
    }

    fn latest(&self) -> Option<Arc<PreviewFrame>> {
        self.inner.latest()
    }

    fn camera_status(&self) -> CameraStatus {
        self.inner.camera_status()
    }

    fn vision_status(&self) -> Option<VisionStatus> {
        self.inner.vision_status()
    }

    fn latest_snapshot(&self) -> Option<Arc<VisionSnapshot>> {
        self.inner.latest_snapshot()
    }

    fn stop(&self) {
        VpssStream::stop(self);
    }
}

/// 预览通道的宽度（`--scale` 的整数倍降采样由 VPSS 硬件完成）。
fn config_scale_w(cfg: &VisionConfig) -> u32 {
    (FRAME_W as u32 / cfg.scale.max(1) as u32).max(16)
}

fn config_scale_h(cfg: &VisionConfig) -> u32 {
    (FRAME_H as u32 / cfg.scale.max(1) as u32).max(16)
}

/* ------------------------------------------------------------------ */
/* 管线（线程内）                                                       */
/* ------------------------------------------------------------------ */

/// 线程内的完整管线：会话、VPSS 组、VENC、输入 VB 帧、相机、模型。
///
/// **字段顺序即析构顺序**：`vpss` / `enc` / `input` / `camera` 借用或依赖
/// `_sys` 里的会话（它们的 `'static` 是构造时 `unsafe` 延长的假生命周期，
/// 实际指向堆上的 `_sys`），所以 `_sys` 必须声明在最后、最后析构。
/// 管线整体 `move` 进线程，析构发生在同一个线程里。
struct VpssPipeline {
    vpss: Vpss<'static>,
    enc: Encoder<'static>,
    /// YUYV 输入帧（cached 映射：CPU 写入快，送 VPSS 前 flush）。
    input: Frame<'static>,
    camera: crate::camera::Camera,
    /// `None` = 模型不可用（仅预览，自动模式不可用）。
    infer: Option<TpuInference>,
    model_error: Option<String>,
    model_input: String,
    pos: PositionAnalyzer,
    cfg: VisionConfig,
    seq: u64,
    /// 会话（堆上，地址稳定）。
    _sys: Box<Sys>,
}

impl VpssPipeline {
    fn new(cfg: VisionConfig) -> io::Result<Self> {
        // 1) 相机
        let camera = open_camera(&cfg.device)?;

        // 2) 会话 + VB 池：块要放得下最大的一帧（RGB 平面 921600）
        let yuyv = layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_YUYV)?;
        let rgb = layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_RGB_888_PLANAR)?;
        let nv12 = layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12)?;
        let blk = yuyv.vb_size.max(rgb.vb_size).max(nv12.vb_size);
        let sys = Box::new(
            Sys::init(&[VbPoolConfig::new(blk, 6).with_name("vpss")])
                .map_err(|e| io::Error::other(format!("MMF 会话初始化失败: {e}")))?,
        );
        if sys.pools_match_request() == Some(false) {
            warn!("内核里是残留 VB 池（上次进程没有干净退出），块可能偏少");
        }

        // SAFETY: `sys` 在堆上（移动本结构体不会移动它），且字段顺序保证
        // `vpss`/`enc`/`input` 先于 `_sys` 析构，所以延长的 `'static` 不会真的越界。
        let session: &'static Sys = unsafe { &*(&*sys as *const Sys) };

        // 3) VPSS 组：YUYV 进 → chn0 RGB 平面（给 TPU）+ chn1 NV12（给 VENC，可缩放）
        let scale = cfg.scale.max(1) as u32;
        let chn1_w = (FRAME_W as u32 / scale).max(16);
        let chn1_h = (FRAME_H as u32 / scale).max(16);
        let vpss = session
            .create_vpss(&VpssConfig {
                grp: VPSS_GRP_AUTO, // 组号一个 boot 只能建一次，自动往后取
                max_w: FRAME_W as u32,
                max_h: FRAME_H as u32,
                in_format: ffi::PIXEL_FORMAT_YUYV,
                chns: vec![
                    VpssChnConfig::new(
                        0,
                        FRAME_W as u32,
                        FRAME_H as u32,
                        ffi::PIXEL_FORMAT_RGB_888_PLANAR,
                    ),
                    VpssChnConfig::new(1, chn1_w, chn1_h, ffi::PIXEL_FORMAT_NV12)
                        .with_depth(PREVIEW_DEPTH),
                ],
            })
            .map_err(|e| io::Error::other(format!("VPSS 建组失败: {e}")))?;
        // 相机 YUYV 是 BT.601 limited；默认矩阵不扩量程会让模型看到偏灰画面。
        vpss.set_yuv601_limited_to_full()
            .map_err(|e| io::Error::other(format!("VPSS CSC 配置失败: {e}")))?;

        // 4) VENC（PT_JPEG，NV12；qfactor 50 是"用自定义量化表"的特殊值，避开）
        let quality = if cfg.quality == 50 { 51 } else { cfg.quality };
        let enc = session
            .create_encoder(
                0,
                &EncoderConfig::new(chn1_w, chn1_h, ffi::PIXEL_FORMAT_NV12)
                    .with_quality(u32::from(quality)),
            )
            .map_err(|e| io::Error::other(format!("VENC 通道创建失败: {e}")))?;

        // 5) 输入帧
        let input = session
            .alloc_frame_cached(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_YUYV)
            .map_err(|e| io::Error::other(format!("输入帧分配失败: {e}")))?;

        // 6) 模型：加载失败/输入尺寸不匹配都只降级为"仅预览"
        let (infer, model_error, model_input) = match TpuInference::try_new(
            &cfg.model,
            cfg.conf_threshold,
            cfg.iou_threshold,
            vec![cfg.label.clone()],
        ) {
            Ok(infer) => {
                let input_fmt = format!("{:?}", infer.input_shape());
                if infer.input_bytes() != rgb.vb_size as usize {
                    let msg = format!(
                        "模型输入 {} 字节与 VPSS RGB 帧 {} 字节不一致，零拷贝不可用",
                        infer.input_bytes(),
                        rgb.vb_size
                    );
                    (None, Some(msg), input_fmt)
                } else {
                    (Some(infer), None, input_fmt)
                }
            }
            Err(e) => (None, Some(e.to_string()), "-".to_string()),
        };
        if let Some(err) = &model_error {
            warn!("VPSS 管线模型不可用（仅预览，自动模式不可用）：{err}");
        }

        let pos = PositionAnalyzer::new(
            FRAME_W,
            FRAME_H,
            LEFT_BOUNDARY,
            RIGHT_BOUNDARY,
            TOP_BOUNDARY,
            BOTTOM_BOUNDARY,
            NEAR_THRESHOLD,
            MID_THRESHOLD,
            None,
        );

        Ok(Self {
            vpss,
            enc,
            input,
            camera,
            infer,
            model_error,
            model_input,
            pos,
            cfg,
            seq: 0,
            _sys: sys,
        })
    }

    /// 帧循环：采集 → VPSS → 推理 →（按节拍）JPEG。
    fn run(&mut self, inner: &StreamInner, running: &AtomicBool) {
        // 启动时把静态状态灌进真正的 inner（`new` 里那份是临时的）
        {
            let mut state = inner.state.lock().unwrap();
            state.device = self.camera.device().to_string();
            state.camera_format = self.camera.pixel_format();
            state.available = true;
            state.model_ok = self.infer.is_some();
            state.model = self.cfg.model.clone();
            state.model_input = self.model_input.clone();
            state.error = self.model_error.clone();
            state.encode = "vpss".to_string();
        }

        let preview_period = if self.cfg.preview_fps > 0.0 {
            Duration::from_secs_f32(1.0 / self.cfg.preview_fps)
        } else {
            Duration::ZERO
        };
        let preview_due = preview_period.mul_f32(0.5); // 允许一点余量，避免帧率被量化砍半
        let mut last_preview = Instant::now()
            .checked_sub(preview_period)
            .unwrap_or_else(Instant::now);
        let mut last_error: Option<String> = None;
        let mut last_error_at = Instant::now()
            .checked_sub(Duration::from_secs(5))
            .unwrap_or_else(Instant::now);

        while running.load(Ordering::Relaxed) {
            let preview_due_now = preview_period.is_zero() || last_preview.elapsed() >= preview_due;
            if preview_due_now {
                last_preview = Instant::now();
            }

            match self.step(preview_due_now) {
                Ok(step) => {
                    {
                        let mut state = inner.state.lock().unwrap();
                        state.record_model_frame(Instant::now(), &step.timings, step.snapshot);
                        state.error = self.model_error.clone();
                        state.encode = "vpss".to_string();
                        state.encode_ms = step.timings.encode_ms;
                        if step.preview_sent {
                            state.preview_sent += 1;
                        }
                    }
                    if let Some((jpeg, snapshot, encode_ms)) = step.preview {
                        let frame = Arc::new(PreviewFrame {
                            seq: step.seq,
                            jpeg: Some(jpeg),
                            vision: Some(snapshot),
                        });
                        let mut state = inner.state.lock().unwrap();
                        state.record_preview(frame, encode_ms, Instant::now());
                    }
                    if let Some(err) = last_error.take() {
                        info!("VPSS 采集恢复：{err}");
                    }
                }
                Err(e) => {
                    let msg = e.to_string();
                    {
                        let mut state = inner.state.lock().unwrap();
                        state.camera_error = Some(msg.clone());
                    }
                    if last_error.as_deref() != Some(msg.as_str())
                        || last_error_at.elapsed() > Duration::from_secs(1)
                    {
                        warn!("VPSS 管线失败：{msg}");
                        last_error = Some(msg);
                        last_error_at = Instant::now();
                    }
                    thread::sleep(RETRY_BACKOFF);
                }
            }
        }
        info!("VPSS 视觉线程退出");
    }

    /// 一帧：返回快照 + 可选的（JPEG、同帧快照、编码耗时）。
    fn step(&mut self, with_preview: bool) -> io::Result<VpssStep> {
        let t_frame = Instant::now();

        // 1) 采集（零拷贝）→ 拷进 VB 块（全链路唯一一次 CPU 像素搬运）
        let t0 = Instant::now();
        let frame = self.camera.get_frame()?;
        self.input
            .write_tight(frame.as_slice())
            .map_err(|e| io::Error::other(format!("YUYV 写 VB 失败: {e}")))?;
        frame.release()?; // 尽早归还相机缓冲
        self.input
            .flush()
            .map_err(|e| io::Error::other(format!("VB flush 失败: {e}")))?;
        let capture_ms = elapsed_ms(t0);

        // 2) VPSS：硬件 CSC（+ 缩放），chn0 给 TPU、chn1 给预览
        let t1 = Instant::now();
        self.vpss
            .send_frame(self.input.info(), ffi::CVI_IO_BLOCK)
            .map_err(|e| io::Error::other(format!("VPSS SendFrame 失败: {e}")))?;
        let rgb = self
            .vpss
            .get_chn_frame(0, GET_FRAME_TIMEOUT_MS)
            .map_err(|e| io::Error::other(format!("VPSS GetChnFrame(chn0) 失败: {e}")))?;
        let vpss_ms = elapsed_ms(t1);

        // 3) 推理（零拷贝：输入张量直接指向 chn0 的物理地址）
        let mut infer_error = None;
        let (dets, infer_ms, nms_ms) = match self.infer.as_mut() {
            Some(infer) => match infer.try_infer_physical(rgb.phy_addr(0)) {
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
        drop(rgb); // ReleaseChnFrame：TPU 读完即可复用
        if let Some(e) = infer_error {
            self.model_error = Some(e);
            self.infer = None;
        }

        // 4) 位置分析
        let t2 = Instant::now();
        let result = self.pos.analyze(&dets);
        let position_ms = elapsed_ms(t2);

        // 5) 预览：到节拍才取 chn1 → VENC（硬件 JPEG）。没取走的 chn1 帧由
        //    驱动按 u32Depth 丢掉，不会拖慢模型这一路。
        let mut encode_ms = 0.0;
        let mut preview = None;
        let mut preview_sent = false;
        if with_preview {
            preview_sent = true;
            let t3 = Instant::now();
            let nv12 = self
                .vpss
                .get_chn_frame(1, GET_FRAME_TIMEOUT_MS)
                .map_err(|e| io::Error::other(format!("VPSS GetChnFrame(chn1) 失败: {e}")))?;
            let jpeg = self
                .enc
                .encode_info(nv12.info(), ffi::CVI_IO_BLOCK)
                .map_err(|e| io::Error::other(format!("VENC 编码失败: {e}")))?;
            drop(nv12);
            encode_ms = elapsed_ms(t3);
            if jpeg.is_empty() {
                return Err(io::Error::other("VENC 输出为空"));
            }
            preview = Some(jpeg);
        }

        // 6) 快照（与预览同帧）
        let step = crate::vision::VisionStep {
            seq: self.seq,
            at: t_frame,
            result,
            preview: None,
            timings: VisionTimings {
                capture_ms,
                preprocess_ms: vpss_ms,
                infer_ms,
                nms_ms,
                position_ms,
                encode_ms,
                total_ms: elapsed_ms(t_frame),
            },
        };
        self.seq += 1;
        let snapshot = Arc::new(snapshot(&step));

        Ok(VpssStep {
            seq: step.seq,
            timings: step.timings,
            snapshot: Arc::clone(&snapshot),
            preview: preview.map(|jpeg| (Arc::from(jpeg), snapshot, encode_ms)),
            preview_sent,
        })
    }
}

/// 一帧的结果（快照 + 可选预览）。
struct VpssStep {
    seq: u64,
    timings: VisionTimings,
    snapshot: Arc<VisionSnapshot>,
    /// `(JPEG, 同帧快照, 编码耗时)`。
    preview: Option<(Arc<[u8]>, Arc<VisionSnapshot>, f64)>,
    preview_sent: bool,
}

/* ------------------------------------------------------------------ */
/* 工具                                                                */
/* ------------------------------------------------------------------ */

fn layout(w: u32, h: u32, fmt: ffi::PIXEL_FORMAT_E) -> io::Result<cvimpi_rs::FrameLayout> {
    venc_input_layout(w, h, fmt)
        .ok_or_else(|| io::Error::other(format!("不支持的像素格式 {fmt}")))
}

fn elapsed_ms(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1000.0
}
