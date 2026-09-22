//! 下位机调试工具：不依赖相机/TPU，用于验证 SG2002 ↔ ESP32-C3 的串口链路
//! 和固件闭环动作，可在板端单独运行。
//!
//! ```sh
//! carctl status                      # 初始化 + 打印 Status（RPM/状态机）
//! carctl drive 40 40 2               # 双轮差速 2 秒（左 40 / 右 40）
//! carctl brake                       # 双轮短接制动
//! carctl stop                        # 双轮滑行
//! carctl move 500                    # 固件闭环前进 500mm（默认速度 60）
//! carctl move -300 50                # 后退 300mm，速度 50
//! carctl rotate 90                   # 原地左转 90°（默认速度 40）
//! carctl rotate -90 40               # 原地右转 90°
//! carctl --port /dev/ttyUSB0 status  # 在 PC 上接 USB 转串口调试
//! ```
//!
//! 参数解析用 `clap`（`carctl <命令> --help` 查看完整说明）。
//!
//! 关键点：`move`/`rotate` 走固件的编码器闭环，期间上位机不能下发速度指令
//! （会被固件当作新的运动意图），所以本工具先切到 manual 模式，只发目标帧
//! 并用 `Heartbeat` 轮询 `Status` 直到 `dist_result` 置位。

use std::thread;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use sg2002_upper::control::protocol::{self, MotorTarget, MoveDir, Request, RotateDir, SysState};
use sg2002_upper::control::{Car, CarConfig};

/// 闭环动作的最长等待。
const GOAL_TIMEOUT: Duration = Duration::from_secs(30);
/// 闭环动作的 Status 轮询周期。
const POLL_PERIOD: Duration = Duration::from_millis(100);

/// 下位机调试工具：串口链路检查与固件闭环动作。
#[derive(Debug, Parser)]
#[command(name = "carctl", version, long_about = None)]
struct Cli {
    /// 串口设备（默认 /dev/ttyS1；ttyS0 是调试控制台）
    #[arg(long, default_value = "/dev/ttyS1")]
    port: String,
    /// 波特率
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// 初始化并打印下位机 Status
    Status,
    /// 发送 Init（编码器清零，进入 Ready）
    Init,
    /// 双轮滑行
    Stop,
    /// 双轮短接制动
    Brake,
    /// 直接差速（-100..100，不等待到位）
    Drive {
        /// 左轮速度（-100..100）
        #[arg(value_parser = speed)]
        left: i16,
        /// 右轮速度（-100..100）
        #[arg(value_parser = speed)]
        right: i16,
        /// 持续时间（秒）
        #[arg(default_value_t = 2.0)]
        secs: f64,
    },
    /// 固件闭环直行（mm 正=前进，负=后退）
    Move {
        /// 距离（mm，非 0）
        #[arg(value_parser = nonzero_i32)]
        mm: i32,
        /// 速度（1..100）
        #[arg(default_value_t = 60, value_parser = clap::value_parser!(u8).range(1..=100))]
        speed: u8,
    },
    /// 固件闭环原地转向（度 正=左转，负=右转）
    Rotate {
        /// 角度（度，至少 0.1）
        #[arg(value_parser = angle)]
        deg: f64,
        /// 速度（1..100）
        #[arg(default_value_t = 40, value_parser = clap::value_parser!(u8).range(1..=100))]
        speed: u8,
    },
}

/// 车轮速度参数：-100..100。
fn speed(text: &str) -> Result<i16, String> {
    let value: i16 = text.parse().map_err(|_| "不是合法整数".to_string())?;
    if !(-100..=100).contains(&value) {
        return Err("速度需在 -100..100".to_string());
    }
    Ok(value)
}

/// 非零整数参数。
fn nonzero_i32(text: &str) -> Result<i32, String> {
    let value: i32 = text.parse().map_err(|_| "不是合法整数".to_string())?;
    if value == 0 {
        return Err("不能为 0".to_string());
    }
    Ok(value)
}

/// 角度参数：换算成 0.1° 后不能为 0（与 `Rotate` 指令一致）。
fn angle(text: &str) -> Result<f64, String> {
    let value: f64 = text.parse().map_err(|_| "不是合法数字".to_string())?;
    if (value.abs() * 10.0).round() < 1.0 {
        return Err("角度太小（至少 0.1°）".to_string());
    }
    Ok(value)
}

fn open_car(port: &str, baud: u32) -> Car {
    let car = Car::open(CarConfig {
        port: port.to_string(),
        baud,
        ..Default::default()
    })
    .unwrap_or_else(|e| {
        eprintln!("carctl: 打开串口 {port} 失败: {e}");
        std::process::exit(2);
    });
    if !car.ensure_ready(Duration::from_secs(3)) {
        eprintln!("警告：下位机无应答，继续尝试发送指令");
    }
    car.set_manual(true);
    car
}

fn print_status(car: &Car) {
    match car.status() {
        Some(st) => println!(
            "  sys={} rpm=({}, {}) dist_active={} dist_result={}{}",
            st.sys,
            st.rpm[0],
            st.rpm[1],
            st.dist_active as u8,
            st.dist_result,
            if st.sys == SysState::Uninit {
                "（需要 Init）"
            } else {
                ""
            }
        ),
        None => println!("  无应答（链路未建立？）"),
    }
}

/// 发心跳、等一拍、返回最新 Status。
fn poll_status(car: &Car) -> Option<protocol::Status> {
    car.send_request(Request::Heartbeat);
    thread::sleep(POLL_PERIOD);
    car.status()
}

/// 等待固件闭环动作完成；返回是否到达目标。
fn wait_goal(car: &Car, label: &str) -> bool {
    let deadline = Instant::now() + GOAL_TIMEOUT;
    loop {
        if Instant::now() >= deadline {
            eprintln!("  {label}: 超时");
            return false;
        }
        let Some(st) = poll_status(car) else {
            eprintln!("  {label}: 没有 Status（链路丢失？）");
            return false;
        };
        if !st.dist_active {
            if st.done() {
                println!("  {label}: 已到达（rpm=({}, {})）", st.rpm[0], st.rpm[1]);
                return true;
            }
            eprintln!(
                "  {label}: 闭环在到达前停止（rpm=({}, {})）",
                st.rpm[0], st.rpm[1]
            );
            return false;
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let car = open_car(&cli.port, cli.baud);

    match cli.command {
        Command::Status => {
            println!("下位机 Status（{}）:", cli.port);
            for _ in 0..5 {
                poll_status(&car);
                print_status(&car);
            }
        }
        Command::Init => {
            car.send_request(Request::Init);
            thread::sleep(Duration::from_millis(300));
            println!("Init：{}", car.last_frame());
            print_status(&car);
        }
        Command::Stop => {
            car.send_request(Request::Stop {
                target: MotorTarget::Both,
            });
            thread::sleep(Duration::from_millis(300));
            println!("Stop：{}", car.last_frame());
        }
        Command::Brake => {
            car.send_request(Request::Brake {
                target: MotorTarget::Both,
            });
            thread::sleep(Duration::from_millis(300));
            println!("Brake：{}", car.last_frame());
        }
        Command::Drive { left, right, secs } => {
            println!("drive 左={left} 右={right} {secs}s ...");
            let deadline = Instant::now() + Duration::from_secs_f64(secs.max(0.0));
            while Instant::now() < deadline {
                car.send_request(Request::SetSpeeds { left, right });
                thread::sleep(Duration::from_millis(50));
            }
            car.send_request(Request::Stop {
                target: MotorTarget::Both,
            });
            thread::sleep(Duration::from_millis(200));
            println!("已停车：{}", car.last_frame());
        }
        Command::Move { mm, speed } => {
            car.send_request(Request::Stop {
                target: MotorTarget::Both,
            });
            thread::sleep(Duration::from_millis(100));
            let dir = if mm > 0 {
                MoveDir::Forward
            } else {
                MoveDir::Backward
            };
            car.move_goal(dir, speed, mm.unsigned_abs());
            println!("move {mm}mm 速度 {speed} ...");
            wait_goal(&car, "move");
        }
        Command::Rotate { deg, speed } => {
            let tenths = (deg.abs() * 10.0).round() as u32;
            car.send_request(Request::Stop {
                target: MotorTarget::Both,
            });
            thread::sleep(Duration::from_millis(100));
            let dir = if deg > 0.0 {
                RotateDir::Left
            } else {
                RotateDir::Right
            };
            car.rotate_goal(dir, speed, tenths);
            println!("rotate {deg}° 速度 {speed} ...");
            wait_goal(&car, "rotate");
        }
    }

    // 收尾：回到自动模式（会补一帧 Stop），再关闭链路。
    car.set_manual(false);
    car.shutdown();
    let c = car.counters();
    println!(
        "链路统计：ACK={} NACK={} 校验错={} 写失败={}",
        c.acks, c.nacks, c.checksum_fails, c.write_errors
    );
}
