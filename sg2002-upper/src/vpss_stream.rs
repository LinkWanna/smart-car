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

use std::collections::VecDeque;
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
///
/// bind 路由下这个队列没人取（帧直接进 VENC），只会挂住一个 VB 块；
/// 保留 depth=1 是为了 bind 失败/降级时还能走用户态取帧。
const PREVIEW_DEPTH: u32 = 1;
/// `GetChnFrame` / `GetStream` 超时（ms）。
const GET_FRAME_TIMEOUT_MS: i32 = 1000;
/// 采集失败后的退避。
const RETRY_BACKOFF: Duration = Duration::from_millis(200);
/// bind 路由连续失败多少次后降级到用户态取帧。
const BIND_ERROR_LIMIT: u32 = 5;
/// bind 路由保留多少个待匹配快照（按 PTS 对帧）。
const SNAPSHOT_QUEUE: usize = 4;

/// 预览通道的交接方式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PreviewRoute {
    /// VPSS chn1 直连 VENC（`CVI_SYS_Bind`）：内核内交接，用户态只 `GetStream`，
    /// 编码与 TPU 天然并行；用帧 PTS 精确对帧。
    Bind,
    /// 用户态 `GetChnFrame` + `SendFrame`，`GetStream` 延迟到下一轮
    /// （编码同样与 TPU 重叠，靠"一送一取"的顺序精确对帧）。
    User,
}

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
            "视觉：VPSS 管线（{}，{}，chn0 RGB 平面零拷贝 + chn1 NV12 全尺寸硬编预览）",
            pipeline.camera.device(),
            pipeline.camera.pixel_format(),
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
    /// 预览交接方式（见 [`PreviewRoute`]）。
    route: PreviewRoute,
    /// user 路由：已 `SendFrame` 进 VENC、等着下一轮 `GetStream` 的快照。
    pending: Option<Arc<VisionSnapshot>>,
    /// bind 路由：`(pts, 快照)`，等码流按 PTS 来取。
    snapshots: VecDeque<(u64, Arc<VisionSnapshot>)>,
    /// bind 路由连续失败计数（超限降级到 user）。
    bind_errors: u32,
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

        // 3) VPSS 组：YUYV 进 → chn0 RGB 平面（给 TPU）+ chn1 NV12（给 VENC）
        //    预览通道不缩放（640x480 原尺寸）：硬件 VENC 完全扛得住，
        //    全分辨率画质优先。
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
                    VpssChnConfig::new(1, FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12)
                        .with_depth(PREVIEW_DEPTH),
                ],
            })
            .map_err(|e| io::Error::other(format!("VPSS 建组失败: {e}")))?;
        // 相机 YUYV 是 BT.601 limited；默认矩阵不扩量程会让模型看到偏灰画面。
        vpss.set_yuv601_limited_to_full()
            .map_err(|e| io::Error::other(format!("VPSS CSC 配置失败: {e}")))?;

        // 4) VENC（PT_JPEG，NV12；qfactor 50 是"用自定义量化表"的特殊值，避开）
        //    先只建通道不收帧：bind 的顺序必须是「建通道 → bind → StartRecvFrame」，
        //    否则绑定后 GetStream 会报 EN_ERR_BUSY（SDK sample 同款顺序）。
        let quality = if cfg.quality == 50 { 51 } else { cfg.quality };
        let enc = session
            .create_encoder_pending(
                0,
                &EncoderConfig::new(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12)
                    .with_quality(u32::from(quality)),
            )
            .map_err(|e| io::Error::other(format!("VENC 通道创建失败: {e}")))?;

        // 5) 预览交接：优先 bind（内核直连），失败退回用户态取帧。
        //    `SMARTCAR_VPSS_ROUTE=user|bind` 可以强制指定（对联调/A-B 有用）。
        let forced = std::env::var("SMARTCAR_VPSS_ROUTE").unwrap_or_default();
        // 上次进程崩溃时可能留下 bind 节点：先清掉，否则新 bind "成功"但数据不通。
        match cvimpi_rs::vpss::clear_venc_bind(0) {
            Ok(true) => warn!("发现 VENC 0 上的残留绑定（上次进程异常退出），已清理"),
            Ok(false) => {}
            Err(e) => warn!("清理残留绑定失败（忽略，继续）：{e}"),
        }
        let route = if forced == "user" {
            info!("预览交接：按 SMARTCAR_VPSS_ROUTE=user 强制用户态取帧");
            PreviewRoute::User
        } else {
            match vpss.bind_chn_to_venc(1, enc.channel()) {
                Ok(()) => {
                    if forced == "bind" {
                        info!("预览交接：按 SMARTCAR_VPSS_ROUTE=bind 强制内核直连");
                    }
                    PreviewRoute::Bind
                }
                Err(e) => {
                    if forced == "bind" {
                        return Err(io::Error::other(format!(
                            "SMARTCAR_VPSS_ROUTE=bind 但 bind 失败: {e}"
                        )));
                    }
                    warn!("VPSS→VENC bind 不可用（{e}）；改用用户态取帧 + 延迟取流");
                    PreviewRoute::User
                }
            }
        };
        enc.start_recv_frame(-1)
            .map_err(|e| io::Error::other(format!("CVI_VENC_StartRecvFrame 失败: {e}")))?;

        // 6) 输入帧
        let input = session
            .alloc_frame_cached(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_YUYV)
            .map_err(|e| io::Error::other(format!("输入帧分配失败: {e}")))?;

        // 7) 模型：加载失败/输入尺寸不匹配都只降级为"仅预览"
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
            route,
            pending: None,
            snapshots: VecDeque::with_capacity(SNAPSHOT_QUEUE),
            bind_errors: 0,
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
            state.encode = self.route_label().to_string();
        }

        let mut last_error: Option<String> = None;
        let mut last_error_at = Instant::now()
            .checked_sub(Duration::from_secs(5))
            .unwrap_or_else(Instant::now);

        info!(
            "预览交接：{}（延迟取流：编码与 TPU 并行）",
            match self.route {
                PreviewRoute::Bind => "VPSS→VENC bind + PTS 对帧",
                PreviewRoute::User => "用户态取帧 + SendFrame",
            }
        );

        while running.load(Ordering::Relaxed) {
            match self.step() {
                Ok(step) => {
                    {
                        let mut state = inner.state.lock().unwrap();
                        state.record_model_frame(Instant::now(), &step.timings, step.snapshot);
                        state.error = self.model_error.clone();
                        state.encode = self.route_label().to_string();
                        state.encode_ms = step.timings.encode_ms;
                        state.preview_sent += 1; // 每帧都投递预览（不锁帧）
                    }
                    if let Some((jpeg, snapshot, encode_ms)) = step.preview {
                        let frame = Arc::new(PreviewFrame {
                            // 用配对快照的 seq：user 路由的码流是上一帧的，
                            // 这样 `jpeg` 和 `vision` 始终同帧。
                            seq: snapshot.seq,
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
        self.shutdown();
        info!("VPSS 视觉线程退出");
    }

    /// 一帧：返回快照 + 可选预览（每帧都出预览，不锁帧）。
    fn step(&mut self) -> io::Result<VpssStep> {
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
        // 帧 PTS = 采集序号：bind 路由靠它把码流和快照精确对上（驱动透传）
        self.input.set_pts(self.seq);
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

        // 5) 快照（预览与检测严格同帧）
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
                encode_ms: 0.0, // 下面按路由补
                total_ms: elapsed_ms(t_frame),
            },
        };
        let snapshot = Arc::new(snapshot(&step));
        self.seq += 1;

        // 6) 预览交接（A+B 的核心）。
        //
        // - Bind：chn1 直接进 VENC，**每帧都必须取走码流**（否则 VENC 队列会顶住
        //   整条链路）；编码与上面的 TPU 天然并行，所以这里几乎不等待。
        //   用帧 PTS 精确对帧（驱动透传：输入帧 PTS == 码流 PTS，已实测）。
        // - User：把当前帧交给 VENC（`SendFrame` 不等编码），码流留到**下一轮**
        //   取（同样与 TPU 重叠）；一送一取，顺序即配对。
        let mut encode_ms = 0.0;
        let mut preview: Option<(Arc<[u8]>, Arc<VisionSnapshot>, f64)> = None;
        match self.route {
            PreviewRoute::Bind => {
                self.register_snapshot(self.seq.wrapping_sub(1), &snapshot);
                let t3 = Instant::now();
                match self.enc.get_stream_meta(GET_FRAME_TIMEOUT_MS) {
                    Ok((jpeg, meta)) => {
                        self.bind_errors = 0;
                        let jpeg: Arc<[u8]> = Arc::from(jpeg);
                        let paired = self
                            .take_snapshot_by_pts(meta.pts)
                            .unwrap_or_else(|| Arc::clone(&snapshot));
                        encode_ms = elapsed_ms(t3);
                        if !jpeg.is_empty() {
                            preview = Some((jpeg, paired, encode_ms));
                        }
                    }
                    Err(e) => {
                        self.bind_errors += 1;
                        if self.bind_errors >= BIND_ERROR_LIMIT {
                            self.degrade_from_bind();
                        }
                        return Err(io::Error::other(format!("bind 取流失败: {e}")));
                    }
                }
            }
            PreviewRoute::User => {
                // 上一轮交给 VENC 的那一帧，码流已经好了
                if let Some(prev) = self.pending.take() {
                    let t3 = Instant::now();
                    let jpeg = self
                        .enc
                        .get_stream(GET_FRAME_TIMEOUT_MS)
                        .map_err(|e| io::Error::other(format!("VENC 取流失败: {e}")))?;
                    encode_ms = elapsed_ms(t3);
                    if !jpeg.is_empty() {
                        preview = Some((Arc::from(jpeg), prev, encode_ms));
                    }
                }
                // 把当前帧交给 VENC（只送不取：编码和下一轮的 TPU 重叠）
                let t3 = Instant::now();
                let nv12 = self
                    .vpss
                    .get_chn_frame(1, GET_FRAME_TIMEOUT_MS)
                    .map_err(|e| io::Error::other(format!("VPSS GetChnFrame(chn1) 失败: {e}")))?;
                self.enc
                    .send_frame_info(nv12.info(), ffi::CVI_IO_BLOCK)
                    .map_err(|e| io::Error::other(format!("VENC SendFrame 失败: {e}")))?;
                drop(nv12);
                self.pending = Some(Arc::clone(&snapshot));
                encode_ms += elapsed_ms(t3);
            }
        }

        let timings = VisionTimings {
            encode_ms,
            ..step.timings
        };
        Ok(VpssStep {
            timings,
            snapshot,
            preview,
        })
    }

    /// bind 路由：登记 `(pts, 快照)` 供码流 PTS 匹配。
    fn register_snapshot(&mut self, pts: u64, snapshot: &Arc<VisionSnapshot>) {
        self.snapshots.push_back((pts, Arc::clone(snapshot)));
        while self.snapshots.len() > SNAPSHOT_QUEUE {
            self.snapshots.pop_front();
        }
    }

    /// bind 路由：按 PTS 取走快照（取不到返回 `None`）。
    fn take_snapshot_by_pts(&mut self, pts: u64) -> Option<Arc<VisionSnapshot>> {
        let idx = self.snapshots.iter().position(|(p, _)| *p == pts)?;
        self.snapshots.remove(idx).map(|(_, s)| s)
    }

    /// bind 连续失败：解绑并退回用户态取帧（解绑失败则保持 bind）。
    fn degrade_from_bind(&mut self) {
        match self.vpss.unbind_chn_from_venc(1, self.enc.channel()) {
            Ok(()) => {
                warn!("bind 取流连续失败：已解绑，退回用户态取帧 + 延迟取流");
                self.route = PreviewRoute::User;
                self.bind_errors = 0;
            }
            Err(e) => {
                warn!("bind 解绑失败（{e}）：保持 bind，继续重试");
            }
        }
    }

    /// 退出前解绑（残留的绑定会污染下一次启动）。
    fn shutdown(&mut self) {
        if self.route == PreviewRoute::Bind {
            if let Err(e) = self.vpss.unbind_chn_from_venc(1, self.enc.channel()) {
                warn!("退出时解绑 VPSS→VENC 失败：{e}");
            }
        }
    }

    /// 状态展示用的后端标签（网页 `vision.encode`）。
    fn route_label(&self) -> &'static str {
        match self.route {
            PreviewRoute::Bind => "vpss+bind",
            PreviewRoute::User => "vpss",
        }
    }
}

/// 一帧的结果（快照 + 可选预览）。
struct VpssStep {
    timings: VisionTimings,
    snapshot: Arc<VisionSnapshot>,
    /// `(JPEG, 同帧快照, 编码耗时)`。
    preview: Option<(Arc<[u8]>, Arc<VisionSnapshot>, f64)>,
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
