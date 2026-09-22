//! 网页遥控入口：串口链路 + 网页服务 + MJPEG 预览，不依赖相机/TPU/视觉管线。
//!
//! ```sh
//! webctl                                  # /dev/ttyS1 @115200 + /dev/video0 + :80
//! webctl --port 8080                      # 非特权端口（本地调试）
//! webctl --no-camera                      # 只遥控，不开预览
//! webctl --camera /dev/video0 --video-fps 15
//! webctl --serial /dev/ttyUSB0            # PC 上接 USB 转串口
//! ```
//!
//! 参数解析用 `clap`（`webctl --help` 查看完整说明）。
//!
//! 下位机串口是 **`/dev/ttyS1`**：本板 `ttyS0` 是调试控制台，往里发帧会直接
//! 变成终端上的 `AA 55` 乱码，下位机也收不到。
//!
//! 手机连上小车 AP（`scripts/init_ap.sh`，默认 `192.168.4.1`）后浏览器打开
//! `http://192.168.4.1/` 即可：键盘 W/S/A/D（或方向键）驾驶，A/D 在低速时原地
//! 旋转；页面上的 停止/制动/初始化 按钮对应链路动作。驾驶手感与
//! `esp32c3-smart-car/tools/play.py` 一致（见 [`sg2002_upper::web::teleop`]）。
//!
//! 相机同时只能被一个进程占用：跑本工具时不要再跑 `pipeline`（反之亦然）。

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;

use clap::Parser;
use sg2002_upper::control::{Car, CarConfig};
use sg2002_upper::preview::PreviewSource;
use sg2002_upper::web::{CameraStream, DriveTarget, Server, TeleopConfig, WebConfig};

/// 等待下位机 `Init` 应答的超时。
const READY_TIMEOUT: Duration = Duration::from_secs(3);

/// 网页遥控：串口链路 + WebSocket + MJPEG 预览。
#[derive(Debug, Parser)]
#[command(name = "webctl", version, long_about = None)]
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

    /// 预览相机
    #[arg(long, default_value = "/dev/video0")]
    camera: String,

    /// 关闭 MJPEG 预览
    #[arg(long)]
    no_camera: bool,

    /// 预览帧率上限
    #[arg(long, value_parser = fps, default_value_t = 10.0)]
    video_fps: f32,

    /// 状态推送频率（Hz）
    #[arg(long, value_parser = slow_hz, default_value_t = 10.0)]
    status_hz: f32,

    /// 控制节拍（Hz）
    #[arg(long, value_parser = fast_hz, default_value_t = 20.0)]
    control_hz: f32,

    /// 前进油门上限
    #[arg(long, value_parser = percent, default_value_t = 55.0)]
    max_speed: f32,

    /// 倒车油门上限
    #[arg(long, value_parser = percent, default_value_t = 40.0)]
    max_reverse: f32,

    /// 反转转向
    #[arg(long)]
    invert_steer: bool,
}

/// 浮点参数：解析并校验范围。
fn parse_f32(text: &str, min: f32, max: f32) -> Result<f32, String> {
    let value: f32 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if !(min..=max).contains(&value) {
        return Err(format!("需在 {min}..={max} 之间"));
    }
    Ok(value)
}

/// 0.1..60 fps。
fn fps(text: &str) -> Result<f32, String> {
    parse_f32(text, 0.1, 60.0)
}

/// 0.5..60 Hz（状态推送）。
fn slow_hz(text: &str) -> Result<f32, String> {
    parse_f32(text, 0.5, 60.0)
}

/// 1..100 Hz（控制节拍）。
fn fast_hz(text: &str) -> Result<f32, String> {
    parse_f32(text, 1.0, 100.0)
}

/// 0..100（百分比/油门上限）。
fn percent(text: &str) -> Result<f32, String> {
    parse_f32(text, 0.0, 100.0)
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
    let teleop = TeleopConfig {
        max_speed: cli.max_speed,
        max_reverse: cli.max_reverse,
        control_hz: cli.control_hz,
        invert_steer: cli.invert_steer,
        ..TeleopConfig::default()
    };

    println!("{}", "=".repeat(60));
    println!("  SG2002 网页遥控 — 串口链路 + WebSocket + MJPEG 预览");
    println!(
        "  链路：{} @ {}  ↔  ESP32-C3（AA 55 帧协议）",
        cli.serial, cli.baud
    );
    println!("{}", "=".repeat(60));

    let car = Car::open(CarConfig {
        port: cli.serial.clone(),
        baud: cli.baud,
        ..Default::default()
    })
    .unwrap_or_else(|e| {
        eprintln!("webctl: 打开串口 {} 失败: {e}", cli.serial);
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

    let camera = if cli.no_camera {
        None
    } else {
        Some(Arc::new(CameraStream::start(
            &cli.camera,
            cli.video_fps,
            Arc::clone(&running),
        )))
    };
    if camera.is_none() {
        println!("  预览：已关闭（--no-camera）");
    }
    // 网页只依赖 PreviewSource 抽象（MJPG 源没有视觉结果）
    let preview: Option<Arc<dyn PreviewSource>> = camera
        .as_ref()
        .map(|camera| Arc::clone(camera) as Arc<dyn PreviewSource>);

    let control_hz = cli.control_hz;
    let target: Arc<dyn DriveTarget> = car.clone();
    let server = Server::bind(
        WebConfig {
            bind: cli.bind.clone(),
            port: cli.port,
            status_hz: cli.status_hz,
            video_fps: cli.video_fps,
        },
        teleop,
        target,
        preview,
        Arc::clone(&running),
    )
    .unwrap_or_else(|e| {
        eprintln!("webctl: {e}");
        std::process::exit(2);
    });

    let addr = server.local_addr().expect("读取监听地址失败");
    let control = server.spawn_control();
    if cli.bind == "0.0.0.0" {
        println!(
            "  页面：http://192.168.4.1{}/   （AP 热点默认地址；本机监听 {}）",
            port_suffix(addr.port()),
            addr
        );
    } else {
        println!("  页面：http://{addr}/");
    }
    println!(
        "  控制：{:.0}Hz 节拍，状态 {:.0}Hz，预览 {:.0}fps；Ctrl+C 停止",
        control_hz, cli.status_hz, cli.video_fps
    );
    println!("{}\n", "=".repeat(60));

    // 阻塞在这里，直到 Ctrl+C/SIGTERM 或内部错误把 running 置 false。
    server.run();
    running.store(false, Ordering::SeqCst);

    // 先停控制线程（它会滑行停车），再关预览，最后关链路。
    control.join().ok();
    if let Some(camera) = &camera {
        camera.stop();
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
