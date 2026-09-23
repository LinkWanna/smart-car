//! 上位机整合入口：视觉（相机 YUYV + TPU + 同帧预览）+ 网页遥控（手动/自动）。
//!
//! ```sh
//! smartcar                        # /dev/ttyS1 + /dev/video0 + 自动找模型 + :80
//! smartcar --port 8080            # 非特权端口（本地调试）
//! smartcar --model /root/xxx.cvimodel
//! smartcar --no-vision            # 只遥控（页面同 webctl）
//! smartcar --scale 2 --quality 70 # 预览降采样到 320x240，省 CPU
//! ```
//!
//! 输入模式由页面按钮切换（默认手动，不允许键盘悄悄切换）：
//! - **手动**：WASD（`web::teleop`，与 `webctl` 相同的手感与看门狗）；
//! - **自动**：视觉伺服（`control::servo`），丢失目标原地旋转搜索、近距刹车。
//!   视觉观测超过 500ms 未更新按“看不到”处理（滑行）。
//!
//! 相机按模型需要的 **YUYV422** 打开（`camera.rs`），网页预览由同一帧的 RGB
//! 缓冲软件编码（`vision.rs`），因此检测框与画面**严格同帧**。模型/相机不可用
//! 时降级为「仅预览」或「仅遥控」，手动模式照常。

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use clap::Parser;
use sg2002_upper::control::{Car, CarConfig};
use sg2002_upper::preview::PreviewSource;
use sg2002_upper::vision::{VisionConfig, VisionStream};
use sg2002_upper::web::{DriveTarget, Server, TeleopConfig, WebConfig};

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

    /// 预览 JPEG 质量（1..=100）
    #[arg(long, value_parser = clap::value_parser!(u8).range(1..=100), default_value_t = 70)]
    quality: u8,

    /// 预览降采样倍数（1 = 640x480 更清晰更慢，2 = 320x240 更流畅）
    #[arg(long, value_parser = clap::value_parser!(u8).range(1..=8), default_value_t = 2)]
    scale: u8,

    /// 预览帧率上限（软上限：编码线程节拍，也是网页推送上限）
    #[arg(long, value_parser = fps, default_value_t = 15.0)]
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
    #[arg(long, value_parser = fast_hz, default_value_t = 20.0)]
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

/// 0.5..60 fps（预览帧率）。
fn fps(text: &str) -> Result<f32, String> {
    let value: f32 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if !(0.5..=60.0).contains(&value) {
        return Err("需在 0.5..=60 之间".to_string());
    }
    Ok(value)
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
    let cli = Cli::parse();

    println!("{}", "=".repeat(60));
    println!("  SG2002 整合上位机 — 视觉追踪 + 网页遥控（手动/自动）");
    println!("{}", "=".repeat(60));
    println!(
        "  链路：{} @ {}  ↔  ESP32-C3（AA 55 帧协议）",
        cli.serial, cli.baud
    );

    let car = Car::open(CarConfig {
        port: cli.serial.clone(),
        baud: cli.baud,
        ..Default::default()
    })
    .unwrap_or_else(|e| {
        eprintln!("smartcar: 打开串口 {} 失败: {e}", cli.serial);
        std::process::exit(2);
    });
    if car.ensure_ready(READY_TIMEOUT) {
        println!("  下位机：就绪（{}）", car.last_frame());
    } else {
        eprintln!("  [警告] 下位机无应答，检查接线/供电/固件；页面仍可打开，按“初始化”重试");
    }
    let car = Arc::new(car);

    let running = Arc::new(AtomicBool::new(true));
    install_signals(&running);

    // 视觉：相机以模型需要的 YUYV422 打开，预览与检测同帧（vision.rs）。
    let vision: Option<Arc<VisionStream>> = if cli.no_vision {
        println!("  视觉：已关闭（--no-vision，自动模式不可用）");
        None
    } else {
        match find_model(cli.model.as_deref()) {
            Ok(model) => {
                println!("  模型：{model}");
                Some(Arc::new(VisionStream::start(
                    VisionConfig {
                        device: cli.camera.clone(),
                        model,
                        conf_threshold: cli.conf,
                        iou_threshold: cli.iou,
                        label: "tennis_ball".to_string(),
                        quality: cli.quality,
                        scale: usize::from(cli.scale).max(1),
                        preview_fps: cli.video_fps,
                    },
                    Arc::clone(&running),
                )))
            }
            Err(e) => {
                eprintln!("  [警告] {e}；改为仅遥控（自动模式不可用）");
                None
            }
        }
    };
    let preview: Option<Arc<dyn PreviewSource>> = vision
        .as_ref()
        .map(|v| Arc::clone(v) as Arc<dyn PreviewSource>);

    let teleop = TeleopConfig {
        max_speed: cli.max_speed,
        max_reverse: cli.max_reverse,
        control_hz: cli.control_hz,
        invert_steer: cli.invert_steer,
        ..TeleopConfig::default()
    };
    let target: Arc<dyn DriveTarget> = car.clone();
    let server = Server::bind(
        WebConfig {
            bind: cli.bind.clone(),
            port: cli.port,
            status_hz: 10.0,
            video_fps: cli.video_fps,
        },
        teleop,
        target,
        preview,
        Arc::clone(&running),
    )
    .unwrap_or_else(|e| {
        eprintln!("smartcar: {e}");
        std::process::exit(2);
    });

    let addr = server.local_addr().expect("读取监听地址失败");
    let control = server.spawn_control();
    if cli.bind == "0.0.0.0" {
        println!(
            "  页面：http://192.168.4.1{}/   （AP 热点默认地址；本机监听 {addr}）",
            port_suffix(addr.port())
        );
    } else {
        println!("  页面：http://{addr}/");
    }
    println!("  模式：默认手动（页面按钮切换自动）；Ctrl+C 停止");
    println!("{}\n", "=".repeat(60));

    server.run();
    running.store(false, Ordering::SeqCst);

    // 先停控制线程（它会滑行停车），再关视觉，最后关链路。
    control.join().ok();
    if let Some(vision) = &vision {
        vision.stop();
    }
    car.shutdown();
    let c = car.counters();
    println!(
        "链路统计：ACK={} NACK={} 校验错={} 写失败={} 看门狗={}",
        c.acks, c.nacks, c.checksum_fails, c.write_errors, c.watchdog_trips
    );
    println!("已退出。");
}

fn port_suffix(port: u16) -> String {
    if port == 80 {
        String::new()
    } else {
        format!(":{port}")
    }
}
