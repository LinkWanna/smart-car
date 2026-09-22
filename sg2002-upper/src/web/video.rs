//! MJPG 视频流：后台采集线程 + 最新帧广播，供网页预览。
//!
//! 相机以 MJPG（`FourCC MJPG`）打开 640x480，驱动吐出的每个缓冲本身就是一张
//! JPEG，采集线程按 `fps` 限频拷贝最新一帧；WebSocket 连接线程用
//! [`CameraStream::frame_if_new`] 取新帧广播，HTTP 用 [`CameraStream::latest`]
//! 取快照。相机同时只能被一个进程占用，所以网页预览与 `pipeline` 要错开运行。
//!
//! 打不开相机（没插、被占用、驱动不支持 MJPG）不会让网页服务失败：状态里带
//! `error`，页面显示“预览不可用”，遥控照常。

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::camera::Camera;

/// 预览状态（HUD/调试用）。
#[derive(Debug, Clone)]
pub struct VideoStatus {
    pub device: String,
    /// 相机是否可用（打不开或驱动不支持 MJPG 时为 false）。
    pub available: bool,
    /// 实际协商到的像素格式（如 `MJPG`）。
    pub format: String,
    /// 已发布的帧数。
    pub frames: u64,
    /// 最近一段时间的发布帧率。
    pub fps: f64,
    /// 最近一帧距今的时间。
    pub age_ms: Option<u64>,
    /// 相机错误（打不开/采集失败）。
    pub error: Option<String>,
}

struct Latest {
    seq: u64,
    jpeg: Option<Arc<[u8]>>,
}

struct Inner {
    device: String,
    available: AtomicBool,
    format: Mutex<String>,
    error: Mutex<Option<String>>,
    latest: Mutex<Latest>,
    frames: AtomicU64,
    last_at: Mutex<Option<Instant>>,
    fps: Mutex<f64>,
}

impl Inner {
    fn new(device: &str) -> Self {
        Self {
            device: device.to_string(),
            available: AtomicBool::new(false),
            format: Mutex::new(String::new()),
            error: Mutex::new(None),
            latest: Mutex::new(Latest { seq: 0, jpeg: None }),
            frames: AtomicU64::new(0),
            last_at: Mutex::new(None),
            fps: Mutex::new(0.0),
        }
    }
}

/// 相机预览流句柄。
pub struct CameraStream {
    inner: Arc<Inner>,
    running: Arc<AtomicBool>,
    handle: Mutex<Option<JoinHandle<()>>>,
}

impl CameraStream {
    /// 打开相机并启动采集线程；函数返回时相机状态已确定（看 [`CameraStream::status`]）。
    ///
    /// `running` 置 false 后采集线程退出。
    pub fn start(device: &str, fps: f32, running: Arc<AtomicBool>) -> Self {
        let inner = Arc::new(Inner::new(device));
        let (ready_tx, ready_rx) = std::sync::mpsc::channel::<Result<String, String>>();
        let thread_inner = Arc::clone(&inner);
        let thread_running = Arc::clone(&running);
        let device = device.to_string();
        let handle = thread::spawn(move || {
            let camera = match Camera::try_new(&device, "mjpeg") {
                Ok(camera) => camera,
                Err(e) => {
                    *thread_inner.error.lock().unwrap() = Some(e.to_string());
                    let _ = ready_tx.send(Err(e.to_string()));
                    return;
                }
            };
            let format = camera.pixel_format();
            *thread_inner.format.lock().unwrap() = format.clone();
            if format != "MJPG" {
                let msg = format!("相机不支持 MJPG（协商到 {format}），网页预览不可用");
                eprintln!("[web] {device}: {msg}");
                *thread_inner.error.lock().unwrap() = Some(msg.clone());
                let _ = ready_tx.send(Err(msg));
                return;
            }
            thread_inner.available.store(true, Ordering::Relaxed);
            let _ = ready_tx.send(Ok(format));
            capture_loop(camera, thread_inner, fps, thread_running);
        });

        // 等相机打开结果，让 start 返回时状态就是确定的。
        match ready_rx.recv_timeout(Duration::from_secs(5)) {
            Ok(Ok(fmt)) => {
                eprintln!("[web] 预览相机 {} 已就绪（{fmt}，{fps}fps）", inner.device)
            }
            Ok(Err(e)) => eprintln!("[web] 预览不可用：{e}"),
            Err(_) => {
                let msg = "相机打开超时".to_string();
                *inner.error.lock().unwrap() = Some(msg.clone());
                eprintln!("[web] 预览不可用：{msg}");
            }
        }

        Self {
            inner,
            running,
            handle: Mutex::new(Some(handle)),
        }
    }

    /// 取比 `after` 更新的帧；没有新帧返回 `None`。
    pub fn frame_if_new(&self, after: u64) -> Option<(u64, Arc<[u8]>)> {
        let latest = self.inner.latest.lock().unwrap();
        if latest.seq > after {
            latest.jpeg.clone().map(|jpeg| (latest.seq, jpeg))
        } else {
            None
        }
    }

    /// 最新一帧（HTTP 快照用）。
    pub fn latest(&self) -> Option<Arc<[u8]>> {
        self.inner.latest.lock().unwrap().jpeg.clone()
    }

    /// 当前状态。
    pub fn status(&self) -> VideoStatus {
        let frames = self.inner.frames.load(Ordering::Relaxed);
        let fps = *self.inner.fps.lock().unwrap();
        let age = self
            .inner
            .last_at
            .lock()
            .unwrap()
            .map(|at| at.elapsed().as_millis() as u64);
        VideoStatus {
            device: self.inner.device.clone(),
            available: self.inner.available.load(Ordering::Relaxed),
            format: self.inner.format.lock().unwrap().clone(),
            frames,
            fps,
            age_ms: age,
            error: self.inner.error.lock().unwrap().clone(),
        }
    }

    /// 停止采集并等线程退出；幂等。
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        let handle = self.handle.lock().unwrap().take();
        if let Some(handle) = handle {
            let deadline = Instant::now() + Duration::from_millis(500);
            while !handle.is_finished() && Instant::now() < deadline {
                thread::sleep(Duration::from_millis(5));
            }
            drop(handle); // 超时则 detach，不阻塞退出
        }
    }
}

impl Drop for CameraStream {
    fn drop(&mut self) {
        self.stop();
    }
}

/// 采集循环：按 `fps` 限频把最新一帧放进共享槽位。
fn capture_loop(camera: Camera, inner: Arc<Inner>, fps: f32, running: Arc<AtomicBool>) {
    let period = Duration::from_secs_f32(1.0 / fps.max(0.1));
    let mut last_pub = Instant::now();
    let mut last_error: Option<String> = None;
    let mut last_error_at = Instant::now()
        .checked_sub(Duration::from_secs(5))
        .unwrap_or_else(Instant::now);
    let mut last_frame_at = Instant::now();

    while running.load(Ordering::Relaxed) {
        match camera.get_frame() {
            Ok(frame) => {
                let now = Instant::now();
                if now.saturating_duration_since(last_pub) >= period {
                    let dt = now.saturating_duration_since(last_frame_at).as_secs_f64();
                    last_frame_at = now;
                    if dt > 0.0 {
                        let mut ema = inner.fps.lock().unwrap();
                        *ema = if *ema > 0.0 {
                            *ema * 0.8 + (1.0 / dt) * 0.2
                        } else {
                            1.0 / dt
                        };
                    }
                    let jpeg: Arc<[u8]> = Arc::from(frame.as_slice());
                    {
                        let mut latest = inner.latest.lock().unwrap();
                        latest.seq += 1;
                        latest.jpeg = Some(jpeg);
                    }
                    inner.frames.fetch_add(1, Ordering::Relaxed);
                    *inner.last_at.lock().unwrap() = Some(now);
                    last_pub = now;
                    if let Some(err) = last_error.take() {
                        eprintln!("[web] 相机采集恢复：{err}");
                    }
                }
                // CaptureFrame 在这里 drop，缓冲立即归还驱动。
            }
            Err(e) => {
                let msg = e.to_string();
                *inner.error.lock().unwrap() = Some(msg.clone());
                if last_error.as_deref() != Some(msg.as_str())
                    || last_error_at.elapsed() > Duration::from_secs(1)
                {
                    eprintln!("[web] 相机采集失败：{msg}");
                    last_error = Some(msg);
                    last_error_at = Instant::now();
                }
                thread::sleep(Duration::from_millis(200));
            }
        }
    }
    eprintln!("[web] 预览相机 {} 采集线程退出", inner.device);
}
