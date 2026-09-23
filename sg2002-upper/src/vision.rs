//! 视觉核心：相机（YUYV422）→ 预处理 → TPU 推理 → 位置分析 →（同帧）JPEG 预览。
//!
//! 两种用法：
//! - [`Vision`]：单帧步进，`pipeline`（无网页）与 `smartcar`（整合）共用；
//! - [`VisionStream`]：后台线程版，采集/推理一个线程、JPEG 编码另一个线程：
//!   - 采集线程把**模型真正消费的那一帧**（RGB 缓冲拷贝 + 检测快照）投给编码线程；
//!   - 编码线程编码完把 `(JPEG, 同帧快照)` 发布给网页（[`PreviewSource`]）。
//!
//!   这样模型推理不被软件编码拖慢（C906 上 640x480 编码约 120ms），
//!   而网页显示的仍是「模型看到的那一帧」，框与画面严格对齐。
//!
//! 相机必须以模型需要的 **YUYV422** 打开（`Preprocessor` 的输入格式），
//! 预览 JPEG 由同一帧的 RGB 缓冲软件编码得到，因此不需要第二路相机。
//!
//! 模型加载失败不致命：退化为「仅预览」（`model_ok == false`），手动遥控照常，
//! 自动模式不可用（见 [`PreviewSource::auto_ready`]）。

use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use jpeg_encoder::{ColorType, Encoder};

use crate::camera::Camera;
use crate::hwjpeg::HwJpeg;
use crate::position::{PositionAnalyzer, PositionResult};
use crate::preview::{
    CameraStatus, DetectionBox, PreviewFrame, PreviewSource, TargetInfo, VisionSnapshot,
    VisionStatus, VisionTimings,
};
use crate::preprocess::Preprocessor;
use crate::tpu::TpuInference;

/// 模型输入帧宽（与 [`Preprocessor`] 绑定，也是检测框归一化的基准）。
pub const FRAME_W: usize = 640;
/// 模型输入帧高。
pub const FRAME_H: usize = 480;

/// 位置分级边界（与 `pipeline` 一致）：横向/纵向 33%~66%，面积占比 5%/1%。
const LEFT_BOUNDARY: f32 = 0.33;
const RIGHT_BOUNDARY: f32 = 0.66;
const TOP_BOUNDARY: f32 = 0.33;
const BOTTOM_BOUNDARY: f32 = 0.66;
const NEAR_THRESHOLD: f32 = 0.05;
const MID_THRESHOLD: f32 = 0.01;

/// 相机打开失败时的重试次数（USB 枚举/上一个进程释放设备都需要时间）。
const CAMERA_OPEN_RETRIES: usize = 3;

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
    /// 给预览线程的数据（按 [`PreviewCopy`] 拷贝，`None` = 不带）。
    pub preview: Option<Vec<u8>>,
    pub timings: VisionTimings,
}

/// 预览线程需要的数据格式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewCopy {
    /// 不带数据（`pipeline` 等无预览场景）。
    None,
    /// RGB 平面 CHW（软件编码用；来自预处理缓冲）。
    RgbPlanar,
    /// 相机原始 YUYV422（硬件编码用；直接来自相机缓冲）。
    Yuyv,
}

/// 单帧步进的视觉核心。
pub struct Vision {
    cfg: VisionConfig,
    /// 实际使用的相机节点（USB 重新枚举后编号会变，见 [`open_camera`]）。
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
        let camera = open_camera(&cfg.device)?;
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

    /// 采集并处理一帧；`copy` 决定给预览线程带哪份数据（硬件编码要 YUYV，软件编码要 RGB）。
    pub fn step(&mut self, copy: PreviewCopy) -> io::Result<VisionStep> {
        let t_frame = Instant::now();

        // 1) 采集（零拷贝）
        let t0 = Instant::now();
        let frame = self.camera.get_frame()?;
        let capture_ms = elapsed_ms(t0);

        // 2) 预处理：YUYV422 → RGB 平面 CHW（模型输入）
        let t1 = Instant::now();
        let planar = self.pp.process_yuyv(frame.as_slice(), FRAME_W, FRAME_H);
        // 预览线程要的数据在这里拷（YUYV 需要在归还相机缓冲前取）
        let preview = match copy {
            PreviewCopy::None => None,
            PreviewCopy::RgbPlanar => Some(planar.to_vec()),
            PreviewCopy::Yuyv => Some(frame.as_slice().to_vec()),
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

/// 由一帧结果构造网页/控制用的快照（归一化检测框 + 控制律输入 + 分级信息）。
pub fn snapshot(step: &VisionStep) -> VisionSnapshot {
    let observation = crate::control::servo::Observation::from_result(&step.result);
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

/// 打开相机：先用配置的节点；不存在/打不开时在 `/dev/video*` 里找第一个能出 YUYV 的。
///
/// USB 相机重新枚举后编号会变（`video0` → `video1`），所以这里不能死认一个节点。
fn open_camera(device: &str) -> io::Result<Camera> {
    let first_error = match Camera::try_new(device, "yuyv") {
        Ok(camera) => return Ok(camera),
        Err(e) => e,
    };
    if Path::new(device).exists() {
        // 节点在但配置不上（被占用/格式不支持）：直接报错，避免误开别的节点
        return Err(first_error);
    }

    let mut nodes: Vec<PathBuf> = std::fs::read_dir("/dev")
        .map(|entries| {
            entries
                .flatten()
                .map(|entry| entry.path())
                .filter(|path| {
                    path.file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.starts_with("video"))
                })
                .collect()
        })
        .unwrap_or_default();
    nodes.sort();
    for node in nodes {
        let Some(path) = node.to_str() else { continue };
        if path == device {
            continue;
        }
        if let Ok(camera) = Camera::try_new(path, "yuyv") {
            eprintln!("[vision] {device} 不存在，改用 {path}（USB 重新枚举后编号会变）");
            return Ok(camera);
        }
    }
    Err(io::Error::other(format!(
        "找不到可用的 YUYV 相机（{device} 及 /dev/video*）"
    )))
}

/// RGB 平面 CHW → 打包 RGB（可整数倍降采样）→ baseline JPEG（写入 `out`）。
fn encode_jpeg(
    planar: &[u8],
    scale: usize,
    quality: u8,
    packed: &mut Vec<u8>,
    out: &mut Vec<u8>,
) -> io::Result<()> {
    let scale = scale.max(1);
    let plane = FRAME_W * FRAME_H;
    if planar.len() < plane * 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("RGB 缓冲长度 {} 小于 {}", planar.len(), plane * 3),
        ));
    }
    let (r, rest) = planar.split_at(plane);
    let (g, b) = rest.split_at(plane);
    let (ow, oh) = (FRAME_W / scale, FRAME_H / scale);

    packed.clear();
    packed.reserve(ow * oh * 3);
    if scale == 1 {
        for i in 0..plane {
            packed.push(r[i]);
            packed.push(g[i]);
            packed.push(b[i]);
        }
    } else {
        let n = (scale * scale) as u32;
        for oy in 0..oh {
            for ox in 0..ow {
                let mut sum = [0u32; 3];
                for dy in 0..scale {
                    let row = (oy * scale + dy) * FRAME_W + ox * scale;
                    for dx in 0..scale {
                        let i = row + dx;
                        sum[0] += u32::from(r[i]);
                        sum[1] += u32::from(g[i]);
                        sum[2] += u32::from(b[i]);
                    }
                }
                packed.push((sum[0] / n) as u8);
                packed.push((sum[1] / n) as u8);
                packed.push((sum[2] / n) as u8);
            }
        }
    }

    out.clear();
    Encoder::new(&mut *out, quality)
        .encode(packed, ow as u16, oh as u16, ColorType::Rgb)
        .map_err(|e| io::Error::other(format!("JPEG 编码失败: {e}")))?;
    Ok(())
}

fn elapsed_ms(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1000.0
}

// ---------------------------------------------------------------------------
// 后台线程版：采集/推理 + 编码线程 + PreviewSource
// ---------------------------------------------------------------------------

/// 投给预览线程的一帧：数据（YUYV 或 RGB 平面）+ 同帧快照。
struct EncodePacket {
    data: Vec<u8>,
    snapshot: Arc<VisionSnapshot>,
}

/// `preview_format`：预览线程要的数据格式。
const PREVIEW_RGB_PLANAR: u8 = 0;
const PREVIEW_YUYV: u8 = 1;

/// 线程共享状态（一次锁拿到全部，避免撕裂读）。
#[derive(Default)]
struct StreamState {
    /// 模型帧（采集/推理）统计。
    frames: u64,
    fps: f64,
    last_at: Option<Instant>,
    latest_snapshot: Option<Arc<VisionSnapshot>>,
    /// 预览投递计数（诊断：模型帧 → 预览线程）。
    preview_sent: u64,
    preview_dropped: u64,
    preview_published: u64,
    encode_ms_avg: f64,
    /// 预览（JPEG）统计；`frame_if_new` 只认它。
    preview: Option<Arc<PreviewFrame>>,
    preview_at: Option<Instant>,
    preview_fps: f64,
    encode_ms: f64,
    /// 预览编码后端（`hw` / `sw`）。
    encode: String,
    /// 实际使用的相机节点（USB 重新枚举后会变）。
    device: String,
    camera_format: String,
    available: bool,
    camera_error: Option<String>,
    model_ok: bool,
    model: String,
    model_input: String,
    infer_ms: f64,
    nms_ms: f64,
    position_ms: f64,
    dets: usize,
    error: Option<String>,
}

impl StreamState {
    fn model_age_ms(&self) -> Option<u64> {
        self.last_at.map(|at| at.elapsed().as_millis() as u64)
    }

    fn preview_age_ms(&self) -> Option<u64> {
        self.preview_at.map(|at| at.elapsed().as_millis() as u64)
    }
}

struct StreamInner {
    cfg: VisionConfig,
    state: Mutex<StreamState>,
    /// 预览线程要的数据格式（见 `PREVIEW_*`；硬件编码要 YUYV，软件编码要 RGB 平面）。
    preview_format: AtomicU8,
}

impl StreamInner {
    fn new(cfg: VisionConfig) -> Self {
        Self {
            cfg,
            state: Mutex::new(StreamState::default()),
            preview_format: AtomicU8::new(PREVIEW_RGB_PLANAR),
        }
    }
}

/// 后台视觉源：采集/推理线程 + 预览编码线程，实现 [`PreviewSource`]。
pub struct VisionStream {
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    handle: Mutex<Option<JoinHandle<()>>>,
    encoder: Mutex<Option<JoinHandle<()>>>,
}

impl VisionStream {
    /// 打开相机/模型并启动线程；函数返回时状态已确定（同 `CameraStream`）。
    pub fn start(cfg: VisionConfig, running: Arc<AtomicBool>) -> Self {
        let inner = Arc::new(StreamInner::new(cfg));
        let (ready_tx, ready_rx) = mpsc::channel::<Result<String, String>>();
        // 容量 2：编码线程忙时允许排一帧，避免节拍量化导致预览帧率减半
        let (packet_tx, packet_rx) = mpsc::sync_channel::<EncodePacket>(2);

        let encoder_inner = Arc::clone(&inner);
        let encoder_running = Arc::clone(&running);
        let encoder = thread::spawn(move || encoder_loop(packet_rx, encoder_inner, encoder_running));

        let thread_inner = Arc::clone(&inner);
        let thread_running = Arc::clone(&running);
        let handle = thread::spawn(move || {
            capture_loop(thread_inner, thread_running, packet_tx, ready_tx)
        });

        let device = inner.cfg.device.clone();
        match ready_rx.recv_timeout(Duration::from_secs(10)) {
            Ok(Ok(format)) => {
                let actual = inner.state.lock().unwrap().device.clone();
                let actual = if actual.is_empty() { device } else { actual };
                eprintln!("[vision] 相机 {actual} 已就绪（{format}）");
            }
            Ok(Err(e)) => eprintln!("[vision] 预览不可用：{e}"),
            Err(_) => eprintln!("[vision] 视觉线程启动超时"),
        }
        Self {
            inner,
            running,
            handle: Mutex::new(Some(handle)),
            encoder: Mutex::new(Some(encoder)),
        }
    }

    /// 停止采集并等线程退出；幂等（与 `CameraStream` 一样会置位共享的 `running`）。
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        for slot in [&self.handle, &self.encoder] {
            let handle = slot.lock().unwrap().take();
            if let Some(handle) = handle {
                let deadline = Instant::now() + Duration::from_millis(500);
                while !handle.is_finished() && Instant::now() < deadline {
                    thread::sleep(Duration::from_millis(5));
                }
                drop(handle); // 超时则 detach，不阻塞退出
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
        let state = self.inner.state.lock().unwrap();
        match &state.preview {
            Some(frame) if frame.seq > after => Some(Arc::clone(frame)),
            _ => None,
        }
    }

    fn latest(&self) -> Option<Arc<PreviewFrame>> {
        self.inner.state.lock().unwrap().preview.clone()
    }

    fn camera_status(&self) -> CameraStatus {
        let state = self.inner.state.lock().unwrap();
        CameraStatus {
            available: state.available,
            device: if state.device.is_empty() {
                self.inner.cfg.device.clone()
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
        let state = self.inner.state.lock().unwrap();
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
            position_ms: state.position_ms,
            encode_ms: state.encode_ms,
            dets: state.dets,
            error: state.error.clone(),
        })
    }

    fn latest_snapshot(&self) -> Option<Arc<VisionSnapshot>> {
        self.inner.state.lock().unwrap().latest_snapshot.clone()
    }
}

/// 采集/推理循环：模型帧 → 快照（控制线程用）+ 预览包（按 `preview_fps` 限频）。
fn capture_loop(
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
    packets: mpsc::SyncSender<EncodePacket>,
    ready: mpsc::Sender<Result<String, String>>,
) {
    let cfg = inner.cfg.clone();
    // 相机可能还在枚举/被上一个进程占着：重试几次再放弃
    let mut vision = {
        let mut attempt = 0;
        loop {
            match Vision::new(cfg.clone()) {
                Ok(vision) => break vision,
                Err(e) if attempt < CAMERA_OPEN_RETRIES => {
                    attempt += 1;
                    eprintln!(
                        "[vision] 相机未就绪（{e}），重试 {attempt}/{CAMERA_OPEN_RETRIES}…"
                    );
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
            eprintln!("[vision] 模型不可用（仅预览，自动模式不可用）：{warn}");
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
    let mut last_at = Instant::now();
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
        let copy = if !preview_due_now {
            PreviewCopy::None
        } else if inner.preview_format.load(Ordering::Relaxed) == PREVIEW_YUYV {
            PreviewCopy::Yuyv
        } else {
            PreviewCopy::RgbPlanar
        };
        match vision.step(copy) {
            Ok(step) => {
                let now = Instant::now();
                let dt = now.saturating_duration_since(last_at).as_secs_f64();
                last_at = now;
                let snap = Arc::new(snapshot(&step));
                let has_preview = step.preview.is_some();
                {
                    let mut state = inner.state.lock().unwrap();
                    state.frames += 1;
                    if dt > 0.0 {
                        state.fps = if state.fps > 0.0 {
                            state.fps * 0.8 + (1.0 / dt) * 0.2
                        } else {
                            1.0 / dt
                        };
                    }
                    state.last_at = Some(now);
                    state.infer_ms = step.timings.infer_ms;
                    state.nms_ms = step.timings.nms_ms;
                    state.position_ms = step.timings.position_ms;
                    state.dets = snap.dets.len();
                    state.error = vision.model_error().map(str::to_string);
                    state.latest_snapshot = Some(Arc::clone(&snap));
                }
                if has_preview {
                    let packet = EncodePacket {
                        data: step.preview.expect("has_preview 时应有数据"),
                        snapshot: snap,
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
                    eprintln!("[vision] 采集恢复：{err}");
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
                    eprintln!("[vision] 采集失败：{msg}");
                    last_error = Some(msg);
                    last_error_at = Instant::now();
                }
                thread::sleep(Duration::from_millis(200));
            }
        }
    }
    eprintln!("[vision] 视觉线程退出");
}

/// 预览编码循环：优先硬件 VENC（输入 YUYV），不可用时退纯 Rust（输入 RGB 平面），
/// 编码结果与**同帧**快照一起发布。
fn encoder_loop(
    packets: mpsc::Receiver<EncodePacket>,
    inner: Arc<StreamInner>,
    running: Arc<AtomicBool>,
) {
    let (scale, quality) = (inner.cfg.scale, inner.cfg.quality);

    // 优先尝试硬件编码；失败则软件编码（数据格式要求不同，要告诉采集线程）
    let mut hw = match HwJpeg::new(FRAME_W as u32, FRAME_H as u32, quality) {
        Ok(enc) => {
            eprintln!(
                "[vision] 预览编码：硬件 VENC（输入像素格式 {}，qfactor {}）",
                enc.input_format(),
                quality
            );
            inner.preview_format.store(PREVIEW_YUYV, Ordering::Relaxed);
            Some(enc)
        }
        Err(e) => {
            eprintln!("[vision] 硬件编码不可用（{e}）；改用纯 Rust 编码");
            inner.preview_format.store(PREVIEW_RGB_PLANAR, Ordering::Relaxed);
            None
        }
    };
    inner.state.lock().unwrap().encode =
        if hw.is_some() { "hw".into() } else { "sw".into() };

    let mut packed = Vec::new();
    let mut jpeg = Vec::new();
    while running.load(Ordering::Relaxed) {
        let Ok(packet) = packets.recv_timeout(Duration::from_millis(200)) else {
            continue;
        };
        let t0 = Instant::now();
        let encoded: io::Result<Arc<[u8]>> = match &mut hw {
            Some(enc) => enc.encode(&packet.data),
            None => encode_jpeg(&packet.data, scale, quality, &mut packed, &mut jpeg)
                .map(|()| Arc::from(jpeg.as_slice())),
        };
        let jpeg_bytes = match encoded {
            Ok(bytes) => bytes,
            Err(e) => {
                // 硬件中途失败：降级软件编码（下一帧起）
                if hw.is_some() {
                    eprintln!("[vision] 硬件编码失败（{e}）；降级纯 Rust 编码");
                    hw = None;
                    inner.preview_format.store(PREVIEW_RGB_PLANAR, Ordering::Relaxed);
                    inner.state.lock().unwrap().encode = "sw".into();
                } else {
                    eprintln!("[vision] 预览编码失败：{e}");
                }
                continue;
            }
        };
        let encode_ms = elapsed_ms(t0);
        let seq = packet.snapshot.seq;
        {
            let mut state = inner.state.lock().unwrap();
            state.preview_published += 1;
            state.encode_ms_avg = if state.encode_ms_avg > 0.0 {
                state.encode_ms_avg * 0.9 + encode_ms * 0.1
            } else {
                encode_ms
            };
            if state.preview_published % 100 == 0 {
                let (avg, pubs) = (state.encode_ms_avg, state.preview_published);
                drop(state);
                eprintln!("[vision] 预览已发布 {pubs} 帧，平均编码 {avg:.1}ms");
            }
        }
        let frame = Arc::new(PreviewFrame {
            seq,
            jpeg: Some(jpeg_bytes),
            vision: Some(packet.snapshot),
        });
        let now = Instant::now();
        let mut state = inner.state.lock().unwrap();
        if let Some(prev) = state.preview_at {
            let dt = now.saturating_duration_since(prev).as_secs_f64();
            if dt > 0.0 {
                state.preview_fps = if state.preview_fps > 0.0 {
                    state.preview_fps * 0.8 + (1.0 / dt) * 0.2
                } else {
                    1.0 / dt
                };
            }
        }
        state.preview_at = Some(now);
        state.encode_ms = encode_ms;
        state.preview = Some(frame);
    }
    eprintln!("[vision] 预览编码线程退出");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造「左半红、右半蓝」的 RGB 平面缓冲（模型输入顺序 R/G/B 三平面）。
    fn red_blue_planar() -> Vec<u8> {
        let plane = FRAME_W * FRAME_H;
        let mut buf = vec![0u8; plane * 3];
        let (r, gb) = buf.split_at_mut(plane);
        let (g, b) = gb.split_at_mut(plane);
        for y in 0..FRAME_H {
            for x in 0..FRAME_W {
                let i = y * FRAME_W + x;
                if x < FRAME_W / 2 {
                    r[i] = 255;
                } else {
                    b[i] = 255;
                }
                g[i] = 0;
            }
        }
        buf
    }

    #[test]
    fn encode_jpeg_keeps_rgb_order_and_size() {
        let planar = red_blue_planar();
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        encode_jpeg(&planar, 1, 90, &mut packed, &mut jpeg).unwrap();

        assert_eq!(&jpeg[..2], &[0xFF, 0xD8], "JPEG SOI 缺失");
        assert_eq!(&jpeg[jpeg.len() - 2..], &[0xFF, 0xD9], "JPEG EOI 缺失");

        let mut decoder = jpeg_decoder::Decoder::new(&jpeg[..]);
        let pixels = decoder.decode().unwrap();
        let info = decoder.info().unwrap();
        assert_eq!((info.width, info.height), (FRAME_W as u16, FRAME_H as u16));

        // 采样左右两侧（JPEG 有损 + 色度下采样，只看通道主导关系）
        let px = |x: usize, y: usize| {
            let i = (y * FRAME_W + x) * 3;
            (pixels[i], pixels[i + 1], pixels[i + 2])
        };
        let (r, g, b) = px(40, 40);
        assert!(r > 200 && g < 80 && b < 80, "左侧应是红色，实际 {r},{g},{b}");
        let (r, g, b) = px(FRAME_W - 40, FRAME_H - 40);
        assert!(b > 200 && r < 80 && g < 80, "右侧应是蓝色，实际 {r},{g},{b}");
    }

    #[test]
    fn encode_jpeg_downscales_by_integer_factor() {
        let planar = red_blue_planar();
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        encode_jpeg(&planar, 2, 80, &mut packed, &mut jpeg).unwrap();
        let mut decoder = jpeg_decoder::Decoder::new(&jpeg[..]);
        decoder.decode().unwrap();
        let info = decoder.info().unwrap();
        assert_eq!(
            (info.width, info.height),
            ((FRAME_W / 2) as u16, (FRAME_H / 2) as u16)
        );
        assert_eq!(packed.len(), (FRAME_W / 2) * (FRAME_H / 2) * 3);
    }

    #[test]
    fn encode_jpeg_rejects_short_buffer() {
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        assert!(encode_jpeg(&[0u8; 16], 1, 80, &mut packed, &mut jpeg).is_err());
    }

    #[test]
    fn snapshot_normalizes_detections() {
        use crate::tpu::Detection;
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
