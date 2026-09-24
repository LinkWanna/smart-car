//! 上位机整合入口：视觉（相机 YUYV + TPU + 同帧预览）+ 网页遥控（手动/自动）。
//!
//! ```sh
//! smartcar                        # /dev/ttyS1 + /dev/video0 + 自动找模型 + :80
//! smartcar --port 8080            # 非特权端口（本地调试）
//! smartcar --model /root/xxx.cvimodel
//! smartcar --no-vision            # 只遥控（无视觉，自动模式不可用）
//! smartcar --quality 80          # 预览 JPEG 质量
//! smartcar --video-fps 10        # 限制预览帧率（默认 0 = 不锁帧，跟相机）
//! smartcar --vpss off            # 只用 CPU 路径（YUYV→RGB 直写 VB 帧，零拷贝喂 TPU）
//! ```
//!
//! 视觉后端（`--vpss auto` 默认）：
//! - **VPSS 硬件**（`vision::vpss`）：相机 YUYV 只 memcpy 一次进 VB 块，硬件 CSC
//!   同时出 chn0 RGB 平面（物理地址零拷贝喂 TPU）与 chn1 NV12（VENC 硬编 JPEG，
//!   640x480 原尺寸、每帧都出；`--video-fps` 只作为可选的推送上限）；
//! - **CPU 路径**（`vision::cpu`，回退）：YUYV→RGB 直接写进 VB 帧（CPU 转换），
//!   物理地址零拷贝喂 TPU；预览走 VENC/软件编码。VPSS 建组失败、组号用尽或
//!   模型尺寸不匹配时自动回退。
//!
//! 输入模式由页面按钮切换（默认手动，不允许键盘悄悄切换）：
//! - **手动**：WASD（`control::teleop`：油门斜坡 + 转向 + 输入看门狗）；
//! - **自动**：视觉伺服（`control::servo`），丢失目标原地旋转搜索、近距刹车。
//!   视觉观测超过 500ms 未更新按“看不到”处理（滑行）。
//!
//! 相机按模型需要的 **YUYV422** 打开（`camera.rs`），网页预览由同一帧编码
//! 得到（优先硬件 VENC，失败降级软件编码，见 `hwjpeg.rs`），因此检测框与
//! 画面**严格同帧**。模型/相机不可用时降级为「仅预览」或「仅遥控」，手动模式照常。

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;

use clap::Parser;
use log::{error, info, warn};
use sg2002_upper::control::{ControlSession, TeleopConfig};
use sg2002_upper::logging;
use sg2002_upper::preview::PreviewSource;
use sg2002_upper::transport::{Car, CarConfig};
use sg2002_upper::vision::{VisionConfig, VisionStream, VpssStream};
use sg2002_upper::web::{Server, WebConfig};

/// 等待下位机 `Init` 应答的超时。
const READY_TIMEOUT: Duration = Duration::from_secs(3);
/// 模型候选路径（按顺序取第一个存在的）。
const MODEL_CANDIDATES: [&str; 2] = [
    "/root/yolov8n_tennis_v3.cvimodel",
    "/akars_tennis/model/yolov8n_tennis_v3.cvimodel",
];

/// 上位机整合入口：视觉追踪 + 网页遥控（手动/自动）。
#[derive(Debug, Parser)]
#[command(name = "smartcar", version, long_about = None)]
struct Cli {
    /// 监听端口（默认 80；本地调试可用 8080）
    #[arg(long, default_value_t = 80)]
    port: u16,

    /// 监听地址
    #[arg(long, default_value = "0.0.0.0")]
    bind: String,

    /// 下位机串口（默认 /dev/ttyS1；ttyS0 是调试控制台）
    #[arg(long, default_value = "/dev/ttyS1")]
    serial: String,

    /// 波特率
    #[arg(long, default_value_t = 115_200)]
    baud: u32,

    /// 相机设备（以模型需要的 YUYV422 打开）
    #[arg(long, default_value = "/dev/video0")]
    camera: String,

    /// 模型路径（默认自动查找 /root/*.cvimodel）
    #[arg(long)]
    model: Option<String>,

    /// 关闭视觉（只遥控；自动模式不可用）
    #[arg(long)]
    no_vision: bool,

    /// 视觉后端：`auto` = 优先 VPSS 硬件（失败回退 CPU），`off` = 只用 CPU 路径
    #[arg(long, value_enum, default_value_t = VpssMode::Auto)]
    vpss: VpssMode,

    /// 预览 JPEG 质量（1..=100）
    #[arg(long, value_parser = clap::value_parser!(u8).range(1..=100), default_value_t = 70)]
    quality: u8,

    /// 预览帧率上限（0 = 不锁帧：每帧都推，跟相机帧率）
    #[arg(long, value_parser = fps_or_unlimited, default_value_t = 0.0)]
    video_fps: f32,

    /// 检测置信度阈值
    #[arg(long, default_value_t = 0.5)]
    conf: f32,

    /// NMS IoU 阈值
    #[arg(long, default_value_t = 0.45)]
    iou: f32,

    /// 手动前进油门上限
    #[arg(long, value_parser = percent, default_value_t = 55.0)]
    max_speed: f32,

    /// 手动倒车油门上限
    #[arg(long, value_parser = percent, default_value_t = 40.0)]
    max_reverse: f32,

    /// 控制节拍（Hz）
    #[arg(long, value_parser = fast_hz, default_value_t = 50.0)]
    control_hz: f32,

    /// 反转转向
    #[arg(long)]
    invert_steer: bool,
}

/// 0..100（百分比/油门上限）。
fn percent(text: &str) -> Result<f32, String> {
    let value: f32 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if !(0.0..=100.0).contains(&value) {
        return Err("需在 0..=100 之间".to_string());
    }
    Ok(value)
}

/// 1..100 Hz（控制节拍）。
fn fast_hz(text: &str) -> Result<f32, String> {
    let value: f32 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if !(1.0..=100.0).contains(&value) {
        return Err("需在 1..=100 之间".to_string());
    }
    Ok(value)
}

/// 0（不限制）或 0.5..60 fps（预览帧率上限）。
fn fps_or_unlimited(text: &str) -> Result<f32, String> {
    let value: f32 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if value == 0.0 {
        return Ok(0.0);
    }
    if !(0.5..=60.0).contains(&value) {
        return Err("需为 0（不限制）或 0.5..=60".to_string());
    }
    Ok(value)
}

/// 视觉处理后端。
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum VpssMode {
    /// 优先 VPSS 硬件 CSC（chn0 RGB 平面零拷贝喂 TPU + chn1 NV12 硬编预览），
    /// 建组失败/组号用尽/模型尺寸不匹配时自动回退 CPU 路径。
    Auto,
    /// 只用 CPU 路径（YUYV→RGB 直写 VB 帧，零拷贝喂 TPU）。
    Off,
}

/// 找模型：`--model` > 已知部署路径 > `/root` 下第一个 `*.cvimodel`。
fn find_model(explicit: Option<&str>) -> Result<String, String> {
    if let Some(path) = explicit {
        return Ok(path.to_string());
    }
    for candidate in MODEL_CANDIDATES {
        if Path::new(candidate).exists() {
            return Ok(candidate.to_string());
        }
    }
    if let Ok(entries) = std::fs::read_dir("/root") {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|ext| ext == "cvimodel") {
                return Ok(path.to_string_lossy().into_owned());
            }
        }
    }
    Err("找不到 .cvimodel 模型，用 --model 指定".to_string())
}

/// 信号处理器要置位的停止标志（`Arc<AtomicBool>` 内部地址）。
static STOP_FLAG: AtomicUsize = AtomicUsize::new(0);

extern "C" fn handle_sig(_: libc::c_int) {
    let ptr = STOP_FLAG.load(Ordering::SeqCst) as *const AtomicBool;
    if !ptr.is_null() {
        // 只做原子写：信号处理器里这是 async-signal-safe 的。
        unsafe { (*ptr).store(false, Ordering::SeqCst) };
    }
}

fn install_signals(running: &Arc<AtomicBool>) {
    STOP_FLAG.store(Arc::as_ptr(running) as usize, Ordering::SeqCst);
    unsafe {
        libc::signal(libc::SIGINT, handle_sig as *const () as usize);
        libc::signal(libc::SIGTERM, handle_sig as *const () as usize);
    }
}

fn main() {
    logging::init();

    let cli = Cli::parse();

    info!("{}", "=".repeat(60));
    info!("  SG2002 整合上位机 — 视觉追踪 + 网页遥控（手动/自动）");
    info!("{}", "=".repeat(60));
    info!(
        "  链路：{} @ {}  ↔  ESP32-C3（AA 55 帧协议）",
        cli.serial, cli.baud
    );

    let car = Car::open(CarConfig {
        port: cli.serial.clone(),
        baud: cli.baud,
        ..Default::default()
    })
    .unwrap_or_else(|e| {
        error!("打开串口 {} 失败: {e}", cli.serial);
        std::process::exit(2);
    });
    if car.ensure_ready(READY_TIMEOUT) {
        let last = car
            .state()
            .last_response
            .map_or_else(|| "(尚未收到应答)".to_string(), |(r, _)| r.to_string());
        info!("  下位机：就绪（{last}）");
    } else {
        warn!("下位机无应答，检查接线/供电/固件；页面仍可打开，按“初始化”重试");
    }
    let car = Arc::new(car);

    // 信号处理器先指向这个「启动窗口」标志；服务器建好后转交给它的 stop_flag
    // （见下），这样启动期间（相机/模型初始化）的 Ctrl+C 也不会被漏掉。
    let startup_alive = Arc::new(AtomicBool::new(true));
    install_signals(&startup_alive);

    // 视觉：相机以模型需要的 YUYV422 打开，预览与检测同帧（vision）。
    // `--vpss auto` 时优先走硬件 CSC 管线（vision::vpss）：相机 YUYV 只 memcpy 一次
    // 进 VB 块，VPSS 同时出 RGB 平面（零拷贝喂 TPU）与 NV12（VENC 硬编预览，
    // 640x480 原尺寸、每帧都出）；VPSS 不可用时自动回退 CPU 路径。
    let preview: Option<Arc<dyn PreviewSource>> = if cli.no_vision {
        info!("  视觉：已关闭（--no-vision，自动模式不可用）");
        None
    } else {
        match find_model(cli.model.as_deref()) {
            Ok(model) => {
                info!("  模型：{model}");
                let cfg = VisionConfig {
                    device: cli.camera.clone(),
                    model,
                    conf_threshold: cli.conf,
                    iou_threshold: cli.iou,
                    label: "tennis_ball".to_string(),
                    quality: cli.quality,
                    scale: 1,
                    preview_fps: cli.video_fps,
                };
                let mut stream: Option<Arc<dyn PreviewSource>> = None;
                if cli.vpss == VpssMode::Auto {
                    match VpssStream::try_start(cfg.clone()) {
                        Ok(vpss) => stream = Some(Arc::new(vpss) as Arc<dyn PreviewSource>),
                        Err(e) => warn!("VPSS 管线不可用（{e}）；回退 CPU 路径"),
                    }
                }
                stream
                    .or_else(|| Some(Arc::new(VisionStream::start(cfg)) as Arc<dyn PreviewSource>))
            }
            Err(e) => {
                warn!("{e}；改为仅遥控（自动模式不可用）");
                None
            }
        }
    };

    let teleop = TeleopConfig {
        max_speed: cli.max_speed,
        max_reverse: cli.max_reverse,
        control_hz: cli.control_hz,
        invert_steer: cli.invert_steer,
        ..TeleopConfig::default()
    };
    // 控制会话：链路 + 手动控制律 + 视觉伺服（控制线程由它自己 spawn）。
    let session = ControlSession::new(car.clone(), teleop, preview.clone());
    // 服务器接管 preview（用于推送），这里留一份用于退出时 stop。
    let preview_handle = preview.as_ref().map(Arc::clone);
    let server = Server::bind(
        WebConfig {
            bind: cli.bind.clone(),
            port: cli.port,
            status_hz: 10.0,
            video_fps: cli.video_fps,
        },
        Arc::clone(&session),
        preview,
    )
    .unwrap_or_else(|e| {
        error!("{e}");
        std::process::exit(2);
    });
    // 把信号处理器转到服务器的停止标志；启动窗口内收到过信号则立即补上。
    let stop = server.stop_flag();
    install_signals(&stop);
    if !startup_alive.load(Ordering::SeqCst) {
        stop.store(false, Ordering::SeqCst);
    }

    let addr = server.local_addr().expect("读取监听地址失败");
    session.spawn();
    if cli.bind == "0.0.0.0" {
        info!(
            "  页面：http://192.168.4.1{}/   （AP 热点默认地址；本机监听 {addr}）",
            port_suffix(addr.port())
        );
    } else {
        info!("  页面：http://{addr}/");
    }
    info!("  模式：默认手动（页面按钮切换自动）；Ctrl+C 停止");
    info!("{}", "=".repeat(60));

    server.run();

    // 先停控制线程（它会滑行停车），再关视觉，最后关链路。
    session.stop();
    if let Some(preview) = &preview_handle {
        preview.stop();
    }
    car.shutdown();
    let c = car.state().counters;
    info!(
        "链路统计：ACK={} NACK={} 校验错={} 写失败={} 看门狗={}",
        c.acks, c.nacks, c.checksum_fails, c.write_errors, c.watchdog_trips
    );
    info!("已退出。");
}

fn port_suffix(port: u16) -> String {
    if port == 80 {
        String::new()
    } else {
        format!(":{port}")
    }
}
