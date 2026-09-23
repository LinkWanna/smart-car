//! SG2002 上位机主管线：相机采集 → TPU 检测 → 位置分析 → 视觉伺服 → 串口下发。
//!
//! 视觉部分（相机 YUYV422 / 预处理 / 推理 / 位置分析）在
//! [`sg2002_upper::vision::Vision`] 里，与网页整合入口 `smartcar` 共用；
//! 本工具是无网页的调试/对比入口（不编码预览，省 CPU）。
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

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use sg2002_upper::control::{Action, Car, CarConfig, ControlConfig, ControlLoop, Observation};
use sg2002_upper::stats::Stats;
use sg2002_upper::vision::{PreviewCopy, Vision, VisionConfig};

/// 板端模型路径（与 sg2002_inference 部署一致）。
const MODEL_PATH: &str = "/akars_tennis/model/yolov8n_tennis_v3.cvimodel";
/// 相机设备（模型要 YUYV422）。
const CAMERA_DEVICE: &str = "/dev/video0";
/// SG2002 接 ESP32-C3 的串口（`ttyS0` 是调试控制台，不能占用）。
const SERIAL_PORT: &str = "/dev/ttyS1";
/// 与固件一致的波特率。
const SERIAL_BAUD: u32 = 115_200;
/// 等待下位机 `Init` 应答的超时。
const READY_TIMEOUT: Duration = Duration::from_secs(3);

/// 检测框置信度阈值 / NMS IoU 阈值（与 sg2002_inference 一致）。
const CONF_THRESHOLD: f32 = 0.5;
const IOU_THRESHOLD: f32 = 0.45;

static RUNNING: AtomicBool = AtomicBool::new(true);
extern "C" fn handle_sig(_: libc::c_int) {
    RUNNING.store(false, Ordering::SeqCst);
}

fn main() {
    let mut vision = Vision::new(VisionConfig {
        device: CAMERA_DEVICE.to_string(),
        model: MODEL_PATH.to_string(),
        conf_threshold: CONF_THRESHOLD,
        iou_threshold: IOU_THRESHOLD,
        label: "tennis_ball".to_string(),
        ..VisionConfig::default()
    })
    .unwrap_or_else(|e| panic!("打开相机 {CAMERA_DEVICE} 失败: {e}"));

    println!("{}", "=".repeat(55));
    println!("  SG2002 上位机 — 网球追踪 → ESP32-C3 下位机");
    println!(
        "  相机：{} {} → {}x{} CHW",
        vision.device(),
        vision.camera_format(),
        sg2002_upper::vision::FRAME_W,
        sg2002_upper::vision::FRAME_H
    );
    println!("  模型：{}（输入 {}）", vision.model_path(), vision.model_input());
    if let Some(err) = vision.model_error() {
        eprintln!("  [警告] 模型不可用：{err}（将只做空检测）");
    }
    println!(
        "  链路：{} @ {} ↔ ESP32-C3（AA 55 帧协议，SetSpeeds 差速）",
        SERIAL_PORT, SERIAL_BAUD
    );
    println!("{}", "=".repeat(55));

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
    println!("\n启动管线... 按 Ctrl+C 停止。\n");

    let mut stats = Stats::new();
    let mut last_action: Option<Action> = None;

    unsafe {
        libc::signal(libc::SIGINT, handle_sig as *const () as usize);
        libc::signal(libc::SIGTERM, handle_sig as *const () as usize);
    }

    while RUNNING.load(Ordering::SeqCst) {
        let t_frame = Instant::now();

        // 采集 + 预处理 + 推理 + 位置分析（无预览编码）
        let step = match vision.step(PreviewCopy::None) {
            Ok(step) => step,
            Err(e) => panic!("采集失败: {e}"),
        };
        stats.record_capture(step.timings.capture_ms);
        stats.record_preprocess(step.timings.preprocess_ms);
        stats.record_inference(step.timings.infer_ms, step.timings.nms_ms);
        stats.record_position(step.timings.position_ms);

        // 视觉伺服 -> ESP32-C3 速度指令
        let t3 = Instant::now();
        let observation = Observation::from_result(&step.result);
        let action = ctrl.step(&observation, Instant::now());
        match action {
            Action::Brake => car.brake(),
            _ => {
                if let Some((left, right)) = action.wheels() {
                    car.set_desired(left, right);
                }
            }
        }
        car.maybe_reinit();
        stats.record_control(t3.elapsed().as_secs_f64() * 1000.0);
        stats.record_frame(
            t_frame.elapsed().as_secs_f64() * 1000.0,
            !step.result.all_detections.is_empty(),
        );

        // HUD：每 10 帧或动作变化时打印一行
        let changed = last_action != Some(action);
        if stats.fid % 10 == 0 || changed {
            last_action = Some(action);
            let det_str = if let Some(d) = step.result.all_detections.first() {
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
                step.timings.capture_ms,
                step.timings.preprocess_ms,
                step.timings.infer_ms,
                step.timings.nms_ms,
                step.timings.position_ms,
                t3.elapsed().as_secs_f64() * 1000.0,
                t_frame.elapsed().as_secs_f64() * 1000.0,
                stats.avg_fps(),
                step.result.zone,
                step.result.distance,
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
