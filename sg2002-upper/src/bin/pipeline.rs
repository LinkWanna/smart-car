//! SG2002 上位机主管线：相机采集 → TPU 检测 → 位置分析 → 视觉伺服 → 串口下发。
//!
//! 与 `sg2002_inference/rust/src/bin/pipeline.rs` 的区别：原来的 `Controller`
//! 往串口写 JSON 行协议，这里改为 ESP32-C3 的 `AA 55` 帧协议，
//! 用 `SetSpeeds` 做 10~20Hz 差速追踪、`Heartbeat` 轮询状态，近距 `Brake` 停车，
//! 丢失目标原地旋转搜索（决策见 [`sg2002_upper::control`]）。
//!
//! 用法（板端）：
//!
//! ```sh
//! pipeline
//! ```
//!
//! 参数都在本文件与 [`sg2002_upper::control::ControlConfig`] 里，和原管线一致。

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use sg2002_upper::camera::Camera;
use sg2002_upper::control::{
    Action, Car, CarConfig, ControlConfig, ControlLoop, Distance, Observation,
};
use sg2002_upper::position::{PositionAnalyzer, PositionResult};
use sg2002_upper::preprocess::Preprocessor;
use sg2002_upper::stats::Stats;
use sg2002_upper::tpu::TpuInference;

/// 板端模型路径（与 sg2002_inference 部署一致）。
const MODEL_PATH: &str = "/akars_tennis/model/yolov8n_tennis_v3.cvimodel";
/// SG2002 接 ESP32-C3 的串口（`ttyS0` 是调试控制台，不能占用）。
const SERIAL_PORT: &str = "/dev/ttyS1";
/// 与固件一致的波特率。
const SERIAL_BAUD: u32 = 115_200;
/// 等待下位机 `Init` 应答的超时。
const READY_TIMEOUT: Duration = Duration::from_secs(3);

/// 检测框置信度阈值 / NMS IoU 阈值（与 sg2002_inference 一致）。
const CONF_THRESHOLD: f32 = 0.5;
const IOU_THRESHOLD: f32 = 0.45;
/// 位置分级边界：横向 33%~66% 为中间，面积占比 5%/1%。
const LEFT_BOUNDARY: f32 = 0.33;
const RIGHT_BOUNDARY: f32 = 0.66;
const TOP_BOUNDARY: f32 = 0.33;
const BOTTOM_BOUNDARY: f32 = 0.66;
const NEAR_THRESHOLD: f32 = 0.05;
const MID_THRESHOLD: f32 = 0.01;

static RUNNING: AtomicBool = AtomicBool::new(true);
extern "C" fn handle_sig(_: libc::c_int) {
    RUNNING.store(false, Ordering::SeqCst);
}

fn main() {
    let mut infer = TpuInference::new(
        MODEL_PATH,
        CONF_THRESHOLD,
        IOU_THRESHOLD,
        vec!["tennis_ball".to_string()],
    );
    let (target_w, target_h) = (640, 480);

    println!("{}", "=".repeat(55));
    println!("  SG2002 上位机 — 网球追踪 → ESP32-C3 下位机");
    println!(
        "  相机：640x480 YUYV → Rust yuyv_resize_planar → {}x{} CHW",
        target_w, target_h
    );
    println!("  模型：{} (输入 {}x{})", MODEL_PATH, target_w, target_h);
    println!(
        "  链路：{} @ {} ↔ ESP32-C3（AA 55 帧协议，SetSpeeds 差速）",
        SERIAL_PORT, SERIAL_BAUD
    );
    println!("{}", "=".repeat(55));

    let mut pp = Preprocessor::new(target_w, target_h);
    println!("  预处理器：{}x{} Rust (复用 921600B)", target_w, target_h);
    println!("  推理：{}x{} Rust NMS", target_w, target_h);

    let pos = PositionAnalyzer::new(
        target_w,
        target_h,
        LEFT_BOUNDARY,
        RIGHT_BOUNDARY,
        TOP_BOUNDARY,
        BOTTOM_BOUNDARY,
        NEAR_THRESHOLD,
        MID_THRESHOLD,
        None,
    );

    let ctrl_cfg = ControlConfig::default();
    let mut ctrl = ControlLoop::new(ctrl_cfg);
    println!(
        "  控制：远 {} / 中 {} / 搜索 {}，差速增益 {:.2}，丢失 {}ms 后搜索",
        ctrl_cfg.far_speed,
        ctrl_cfg.mid_speed,
        ctrl_cfg.search_speed,
        ctrl_cfg.turn_gain,
        ctrl_cfg.lost_hold.as_millis()
    );

    let car = Car::open(CarConfig {
        port: SERIAL_PORT.to_string(),
        baud: SERIAL_BAUD,
        ..Default::default()
    })
    .unwrap_or_else(|e| panic!("打开串口 {} 失败: {}", SERIAL_PORT, e));
    if car.ensure_ready(READY_TIMEOUT) {
        println!("  下位机：就绪（{}）", car.last_frame());
    } else {
        eprintln!("  [警告] 下位机无应答，检查接线/供电/固件；继续运行视觉，速度指令会被忽略");
    }
    println!("{}\n", "=".repeat(55));

    let camera = Camera::new("/dev/video0", "yuyv");
    println!("  相机：{}", camera);
    println!("\n启动管线... 按 Ctrl+C 停止。\n");

    let mut stats = Stats::new();
    let mut last_action: Option<Action> = None;

    unsafe {
        libc::signal(libc::SIGINT, handle_sig as *const () as usize);
        libc::signal(libc::SIGTERM, handle_sig as *const () as usize);
    }

    while RUNNING.load(Ordering::SeqCst) {
        let t_frame = Instant::now();

        // 1. 采集（零拷贝）
        let t0 = Instant::now();
        let frame = match camera.get_frame() {
            Ok(v) => v,
            Err(e) => panic!("采集失败: {}", e),
        };
        let capture_ms = t0.elapsed().as_secs_f64() * 1000.0;
        stats.record_capture(capture_ms);
        stats.inc("capture", 1);

        // 2. 预处理 — 零拷贝：mmap → Rust yuyv422_to_rgb → CHW
        let t1 = Instant::now();
        let planar = pp.process_yuyv(frame.as_slice(), 640, 480);
        // 立即归还，让驱动尽早复用缓冲（Drop 兜底）
        if let Err(e) = frame.release() {
            panic!("帧归还失败: {}", e);
        }
        let pre_ms = t1.elapsed().as_secs_f64() * 1000.0;
        stats.record_preprocess(pre_ms);

        // 3. 推理
        let dets = infer.infer(planar);
        let (tpu_ms, nms_ms) = infer.last_timing();
        stats.record_inference(tpu_ms, nms_ms);

        // 4. 位置分析（&[Detection] 零拷贝，避免 clone）
        let t2 = Instant::now();
        let result = pos.analyze(&dets);
        let position_ms = t2.elapsed().as_secs_f64() * 1000.0;
        stats.record_position(position_ms);

        // 5. 控制：视觉伺服 -> ESP32-C3 速度指令
        let t3 = Instant::now();
        let action = ctrl.step(&observation(&result), Instant::now());
        match action {
            Action::Brake => car.brake(),
            _ => {
                if let Some((left, right)) = action.wheels() {
                    car.set_desired(left, right);
                }
            }
        }
        car.maybe_reinit();
        let control_ms = t3.elapsed().as_secs_f64() * 1000.0;
        stats.record_control(control_ms);

        let total_ms = t_frame.elapsed().as_secs_f64() * 1000.0;
        stats.record_frame(total_ms, !dets.is_empty());

        // HUD：每 10 帧或动作变化时打印一行
        let changed = last_action != Some(action);
        if stats.fid % 10 == 0 || changed {
            last_action = Some(action);
            let det_str = if let Some(d) = dets.first() {
                format!(
                    "{:.2} @ ({:.0},{:.0})",
                    d.confidence,
                    d.center_x(),
                    d.center_y()
                )
            } else {
                "无".to_string()
            };
            let (sys_name, rpm) = match car.status() {
                Some(st) => (
                    st.sys.to_string(),
                    format!("({},{})", st.rpm[0], st.rpm[1]),
                ),
                None => ("无应答".to_string(), "(--,--)".to_string()),
            };
            println!(
                "  [{:04}] 检测={} 采集:{:.0}ms 预处理:{:.0}ms TPU:{:.0}ms NMS:{:.0}ms 位置:{:.1}ms 控制:{:.1}ms 总计:{:.0}ms fps:{:.1}\n        目标=[{} {}] 动作={} 下位机:{} rpm{} 链路={}",
                stats.fid,
                det_str,
                capture_ms,
                pre_ms,
                tpu_ms,
                nms_ms,
                position_ms,
                control_ms,
                total_ms,
                stats.avg_fps(),
                result.zone,
                result.distance,
                action,
                sys_name,
                rpm,
                if car.link_ok() { "OK" } else { "丢失" },
            );
        }
    }

    // 退出：滑行停车（Drop 兜底，这里显式执行以便打印异常）。
    car.shutdown();
    println!("\n已停车，链路断开。");
    stats.print_summary();
}

/// 检测结果 -> 控制律输入。
fn observation(result: &PositionResult) -> Observation {
    let err_x = ((result.center_x - 0.5) * 2.0).clamp(-1.0, 1.0);
    let distance = match result.distance.as_str() {
        "near" => Distance::Near,
        "mid" => Distance::Mid,
        _ => Distance::Far,
    };
    Observation {
        present: result.has_target(),
        err_x,
        distance,
        confidence: result.target_confidence,
    }
}
