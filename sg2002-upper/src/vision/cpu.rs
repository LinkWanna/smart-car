//! CPU 管线：采集/推理线程 + 预览编码线程，实现 [`PreviewSource`]。
//!
//! 采集线程把**模型真正消费的那一帧**（YUYV 或 RGB 缓冲拷贝 + 检测快照）投给
//! 编码线程；编码线程编码完把 `(JPEG, 同帧快照)` 发布给网页。这样模型推理不被
//! 预览编码拖慢（C906 上 640x480 软件编码约 120ms，硬件 VENC 约 12ms），而网页
//! 显示的仍是「模型看到的那一帧」，框与画面严格对齐。

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::{info, warn};

use cvimpi_rs::ffi;
use cvimpi_rs::sys::{Sys, VbPoolConfig};
use cvimpi_rs::venc_input_layout;

use crate::preprocess::{PreviewEncoder, PreviewInput};
use crate::preview::{CameraStatus, DetectionFrame, PreviewFrame, PreviewSource, VisionStatus};

use super::core::Vision;
use super::state::{EncodePacket, StreamInner};
use super::{FRAME_H, FRAME_W, VisionConfig, detection_frame, elapsed_ms};

/// 相机打开失败时的重试次数（USB 枚举/上一个进程释放设备都需要时间）。
const CAMERA_OPEN_RETRIES: usize = 3;
/// VB 公共池块数：RGB 输入帧 + NV12 编码输入帧 + VENC 码流缓冲。
const VB_BLK_CNT: u32 = 6;

/// 后台视觉源：采集/推理线程 + 预览编码线程，实现 [`PreviewSource`]。
///
/// 自带停止标志：`start` 之后 [`stop`](VisionStream::stop) 只停自己的两个线程
/// （幂等，`Drop` 兜底），不影响进程里的其它组件。
pub struct VisionStream {
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    handle: Mutex<Option<JoinHandle<()>>>,
    encoder: Mutex<Option<JoinHandle<()>>>,
}

impl VisionStream {
    /// 打开相机/模型并启动线程；函数返回时状态已确定（相机就绪/失败都已写进共享状态）。
    pub fn start(cfg: VisionConfig) -> Self {
        let inner = Arc::new(StreamInner::new(cfg));
        let running = Arc::new(AtomicBool::new(true));
        // 会话：TPU 零拷贝输入帧与硬件预览编码（VENC）共用一个 `Sys`
        // （`CVI_SYS_Init` 是进程级状态，进程里只能有一个）。拿不到会话时
        // 仍可软件预览，但没有推理（见 `Vision`）。
        let session = match create_session() {
            Ok(sys) => Some(Arc::new(sys)),
            Err(e) => {
                warn!("MMF 会话不可用（{e}）：预览降级软件编码，模型不可用");
                None
            }
        };
        let (ready_tx, ready_rx) = mpsc::channel::<Result<String, String>>();
        // 容量 2：编码线程忙时允许排一帧，避免节拍量化导致预览帧率减半
        let (packet_tx, packet_rx) = mpsc::sync_channel::<EncodePacket>(2);

        let encoder_inner = Arc::clone(&inner);
        let encoder_running = Arc::clone(&running);
        let encoder_session = session.clone();
        // 编码线程先定下后端（hw 要 YUYV、sw 要 RGB 平面），再开采集线程，
        // 避免首帧按错的格式拷贝数据（编码线程自己的初始化可能要几十毫秒）。
        let (enc_ready_tx, enc_ready_rx) = mpsc::channel::<()>();
        let encoder = thread::spawn(move || {
            encoder_loop(
                packet_rx,
                encoder_inner,
                encoder_running,
                enc_ready_tx,
                encoder_session,
            )
        });
        if enc_ready_rx.recv_timeout(Duration::from_secs(5)).is_err() {
            warn!("编码线程初始化超时");
        }

        let thread_inner = Arc::clone(&inner);
        let thread_running = Arc::clone(&running);
        let handle = thread::spawn(move || {
            capture_loop(thread_inner, thread_running, packet_tx, ready_tx, session)
        });

        let device = inner.cfg.device.clone();
        match ready_rx.recv_timeout(Duration::from_secs(10)) {
            Ok(Ok(format)) => {
                let actual = inner.state.lock().unwrap().device.clone();
                let actual = if actual.is_empty() { device } else { actual };
                info!("相机 {actual} 已就绪（{format}）");
            }
            Ok(Err(e)) => warn!("预览不可用：{e}"),
            Err(_) => warn!("视觉线程启动超时"),
        }
        Self {
            inner,
            running,
            handle: Mutex::new(Some(handle)),
            encoder: Mutex::new(Some(encoder)),
        }
    }

    /// 停止采集并等线程退出；幂等（置位自己的 `running` 后 join 两个线程）。
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        for (slot, what) in [(&self.handle, "视觉采集"), (&self.encoder, "预览编码")] {
            if let Some(handle) = slot.lock().unwrap().take() {
                crate::join_with_timeout(handle, what);
            }
        }
    }
}

impl Drop for VisionStream {
    fn drop(&mut self) {
        self.stop();
    }
}

impl PreviewSource for VisionStream {
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

    fn latest_detections(&self) -> Option<Arc<DetectionFrame>> {
        self.inner.latest_detections()
    }

    fn stop(&self) {
        VisionStream::stop(self);
    }
}

/// 创建 CPU 管线的 MMF 会话：一个公共池要同时放得下模型输入的 RGB 平面帧
/// 与 VENC 的 NV12 输入帧。
fn create_session() -> io::Result<Sys> {
    let rgb = venc_input_layout(
        FRAME_W as u32,
        FRAME_H as u32,
        ffi::PIXEL_FORMAT_RGB_888_PLANAR,
    )
    .ok_or_else(|| io::Error::other("venc_input_layout 不支持 RGB_888_PLANAR"))?;
    let nv12 = venc_input_layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12)
        .ok_or_else(|| io::Error::other("venc_input_layout 不支持 NV12"))?;
    Sys::init(&[VbPoolConfig::new(rgb.vb_size.max(nv12.vb_size), VB_BLK_CNT).with_name("cpu")])
        .map_err(|e| io::Error::other(format!("MMF 会话初始化失败: {e}")))
}

/// 采集/推理循环：模型帧 → 快照（控制线程用）+ 预览包（按 `preview_fps` 限频）。
fn capture_loop(
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    packets: mpsc::SyncSender<EncodePacket>,
    ready: mpsc::Sender<Result<String, String>>,
    session: Option<Arc<Sys>>,
) {
    let cfg = inner.cfg.clone();
    // 相机可能还在枚举/被上一个进程占着：重试几次再放弃
    let mut vision = {
        let mut attempt = 0;
        loop {
            match Vision::new(cfg.clone(), session.as_deref()) {
                Ok(vision) => break vision,
                Err(e) if attempt < CAMERA_OPEN_RETRIES => {
                    attempt += 1;
                    warn!("相机未就绪（{e}），重试 {attempt}/{CAMERA_OPEN_RETRIES}…");
                    thread::sleep(Duration::from_millis(600));
                }
                Err(e) => {
                    let msg = e.to_string();
                    let mut state = inner.state.lock().unwrap();
                    state.camera_error = Some(msg.clone());
                    drop(state);
                    let _ = ready.send(Err(msg));
                    return;
                }
            }
        }
    };

    {
        let mut state = inner.state.lock().unwrap();
        state.available = true;
        state.device = vision.device().to_string();
        state.camera_format = vision.camera_format();
        state.model_ok = vision.model_ok();
        state.model = cfg.model.clone();
        state.model_input = vision.model_input().to_string();
        state.error = vision.model_error().map(str::to_string);
        let warn = state.error.clone();
        drop(state);
        if let Some(warn) = warn {
            warn!("模型不可用（仅预览，自动模式不可用）：{warn}");
        }
    }
    let _ = ready.send(Ok(vision.camera_format()));

    let preview_period = if cfg.preview_fps > 0.0 {
        Duration::from_secs_f32(1.0 / cfg.preview_fps)
    } else {
        Duration::ZERO
    };
    // 允许一点余量：模型帧间隔（~80ms）与预览周期不同步时，
    // 严格比较会把预览帧率砍半（80/160/240ms 量化）。
    let preview_due = preview_period.mul_f32(0.5);
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
            // 立刻记账：本帧要处理（拷贝+推理）几十毫秒，
            // 若等发送完再记，下一帧的 elapsed 会被吃掉一半 → 预览帧率减半
            last_preview = Instant::now();
        }
        let preview = if !preview_due_now {
            None
        } else if inner.preview_yuyv.load(Ordering::Relaxed) {
            Some(PreviewInput::Yuyv)
        } else {
            Some(PreviewInput::RgbPlanar)
        };
        match vision.step(preview) {
            Ok(step) => {
                let now = Instant::now();
                let frame = Arc::new(detection_frame(&step));
                let has_preview = step.preview.is_some();
                {
                    let mut state = inner.state.lock().unwrap();
                    state.record_model_frame(now, &step.timings, Arc::clone(&frame));
                    state.error = vision.model_error().map(str::to_string);
                }
                if has_preview {
                    let packet = EncodePacket {
                        data: step.preview.expect("has_preview 时应有数据"),
                        frame,
                    };
                    // 预览线程忙（容量 2）就跳过本帧预览，不阻塞模型
                    let ok = packets.try_send(packet).is_ok();
                    {
                        let mut state = inner.state.lock().unwrap();
                        if ok {
                            state.preview_sent += 1;
                        } else {
                            state.preview_dropped += 1;
                        }
                    }
                    let _ = ok; // 预览成功与否由 try_send 的返回值决定，节拍已在上面记账
                }
                if let Some(err) = last_error.take() {
                    info!("采集恢复：{err}");
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
                    warn!("采集失败：{msg}");
                    last_error = Some(msg);
                    last_error_at = Instant::now();
                }
                thread::sleep(Duration::from_millis(200));
            }
        }
    }
    info!("视觉线程退出");
}

/// 预览编码循环：硬件 VENC 优先、软件兜底（后端选择与降级都在
/// [`PreviewEncoder`] 里），编码结果与**同帧**快照一起发布。
fn encoder_loop(
    packets: mpsc::Receiver<EncodePacket>,
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    ready: mpsc::Sender<()>,
    session: Option<Arc<Sys>>,
) {
    let mut preview = PreviewEncoder::new(session.as_deref(), inner.cfg.scale, inner.cfg.quality);
    sync_preview_backend(&inner, &preview);
    // 采集线程在 `VisionStream::start` 里等这个信号，拿到后才开始采集
    let _ = ready.send(());

    while running.load(Ordering::Relaxed) {
        let Ok(packet) = packets.recv_timeout(Duration::from_millis(200)) else {
            continue;
        };
        let t0 = Instant::now();
        // 硬件中途失败会在 encode 里降级并记录日志，本帧没有 JPEG
        let Ok(jpeg_bytes) = preview.encode(&packet.data) else {
            sync_preview_backend(&inner, &preview);
            continue;
        };
        sync_preview_backend(&inner, &preview);
        let encode_ms = elapsed_ms(t0);
        let frame = Arc::new(PreviewFrame {
            seq: packet.frame.seq,
            jpeg: Some(jpeg_bytes),
            dets: Some(packet.frame),
        });
        let now = Instant::now();
        let mut state = inner.state.lock().unwrap();
        state.record_preview(frame, encode_ms, now);
        if state.preview_published % 100 == 0 {
            let (avg, pubs) = (state.encode_ms_avg, state.preview_published);
            drop(state);
            info!("预览已发布 {pubs} 帧，平均编码 {avg:.1}ms");
        }
    }
    info!("预览编码线程退出");
}

/// 把预览编码器的当前后端同步给采集线程（数据格式）与状态（`hw`/`sw`）。
///
/// [`PreviewEncoder::encode`] 可能在硬件失败时降级，所以每次编码后都调用；
/// 没变化时只读一次原子/锁，开销可忽略。
fn sync_preview_backend(inner: &StreamInner, preview: &PreviewEncoder) {
    let yuyv = preview.input_format() == PreviewInput::Yuyv;
    if inner.preview_yuyv.load(Ordering::Relaxed) != yuyv {
        inner.preview_yuyv.store(yuyv, Ordering::Relaxed);
    }
    let backend = preview.backend();
    let mut state = inner.state.lock().unwrap();
    if state.encode != backend {
        state.encode = backend.to_string();
    }
}
