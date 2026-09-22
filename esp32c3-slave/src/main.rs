//! ESP32-C3 dual-motor encoder controller (Rust port of Arduino `base_control.ino`).
//!
//! The command channel is selected by the mutually exclusive `uart` / `ble`
//! cargo features, so exactly one link is compiled in:
//! - `uart` (default): UART0 (GPIO20 RX / GPIO21 TX), 115200 8N1.
//! - `ble`: NUS-style byte pipe (see `ble.rs`).
//!
//! Frame: `[0xAA, 0x55, CMD, LEN, PAYLOAD.., CHK]`, `CHK = CMD ^ LEN ^ PAYLOAD..`.
//! defmt logs go to USB_SERIAL_JTAG only (`esp-println/jtag-serial`),
//! so the UART0 command channel stays clean.

#![no_std]
#![no_main]

// Exactly one link: Cargo features cannot express "one of", so check here.
// `--no-default-features --features uart` / `--features ble` select a build.
#[cfg(all(feature = "ble", feature = "uart"))]
compile_error!("features `ble` and `uart` are mutually exclusive; enable exactly one command link");
#[cfg(not(any(feature = "ble", feature = "uart")))]
compile_error!("no command link selected; enable either the `ble` or the `uart` feature");

mod drivers;
mod service;

// 协议层已抽到上下位机共用的 crate（仓库根目录 `protocol/`）；这里保留
// `crate::protocol` 路径，固件内部照旧 `use crate::protocol::...`。
pub(crate) use smart_car_protocol as protocol;

use drivers::encoder;
use drivers::motor::{Motors, PWM_FREQ_HZ};
use embassy_time::Instant;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Event, Input, InputConfig, Io, Output, OutputConfig, Pull},
    ledc::{LSGlobalClkSource, Ledc, LowSpeed, channel, timer},
    ledc::{channel::ChannelIFace, timer::TimerIFace},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;
use protocol::{ErrorCode, MoveDir, Request, RequestType, RotateDir, RxEvent, RxParser, SysState};
#[cfg(feature = "ble")]
use service::ble;
use service::control::{MotorCmd, State, StateMutex};
use service::link::Link;
use service::odometry::{DistDir, degrees_to_counts, mm_to_counts};
#[cfg(feature = "uart")]
use service::uart;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

/// Feed one received byte (from the compiled-in link) through the frame parser,
/// decode it into a typed [`Request`] and dispatch it.
async fn feed_byte(b: u8, parser: &mut RxParser, st: &mut State, link: &Link) {
    match parser.feed(b) {
        RxEvent::Frame { cmd, payload } => match Request::decode(cmd, payload) {
            Ok(request) => handle_command(request, st, link).await,
            Err(err) => link.send_nack(cmd, err.error_code()).await,
        },
        RxEvent::ChecksumFail { cmd } => link.send_nack(cmd, ErrorCode::BadChecksum).await,
        RxEvent::TooLong { cmd, .. } => link.send_nack(cmd, ErrorCode::InvalidParam).await,
        RxEvent::None => {}
    }
}

// --- Command dispatch (Arduino `handleCommand` parity) ---
async fn handle_command(request: Request, st: &mut State, link: &Link) {
    let cmd = request.request_type();
    // State gate for this command (GetPid and motion commands need Ready).
    if let Some(min) = cmd.min_state() {
        if st.sys < min {
            link.send_nack(cmd.as_u8(), ErrorCode::WrongState).await;
            return;
        }
    }

    match request {
        Request::Init => {
            encoder::Encoders::zero();
            st.last_c0 = 0;
            st.last_c1 = 0;
            st.pid0.reset();
            st.pid1.reset();
            st.odometry.cancel();
            st.rpm0 = 0;
            st.rpm1 = 0;
            st.brake = [false, false];
            st.sys = SysState::Ready;
            link.send_ack(cmd).await;
        }
        Request::SetSpeed { motor, speed } => {
            st.request(motor.as_u8(), MotorCmd::Drive(speed));
            st.sys = SysState::Running;
            link.send_ack(cmd).await;
        }
        Request::SetSpeeds { left, right } => {
            st.request(0, MotorCmd::Drive(left));
            st.request(1, MotorCmd::Drive(right));
            st.sys = SysState::Running;
            link.send_ack(cmd).await;
        }
        Request::Stop { target } => {
            for motor in target.motors() {
                st.request(motor.as_u8(), MotorCmd::Coast);
            }
            link.send_ack(cmd).await;
        }
        Request::Brake { target } => {
            for motor in target.motors() {
                st.request(motor.as_u8(), MotorCmd::Brake);
            }
            link.send_ack(cmd).await;
        }
        Request::SetPid { motor, kp, ki, kd } => {
            let pid = st.pid(motor.as_u8());
            pid.kp = kp as f32 / 100.0;
            pid.ki = ki as f32 / 100.0;
            pid.kd = kd as f32 / 100.0;
            link.send_ack(cmd).await;
        }
        Request::GetPid { motor } => {
            let pid = st.pid(motor.as_u8());
            link.send_pid(pid).await;
        }
        Request::Move {
            dir,
            speed,
            distance_mm,
        } => {
            let dist = match dir {
                MoveDir::Forward => DistDir::Forward,
                MoveDir::Backward => DistDir::Backward,
            };
            run_closed_loop(st, link, cmd, dist, speed, mm_to_counts(distance_mm as f32)).await;
        }
        Request::Rotate {
            dir,
            speed,
            tenths_deg,
        } => {
            let dist = match dir {
                RotateDir::Left => DistDir::Left,
                RotateDir::Right => DistDir::Right,
            };
            run_closed_loop(
                st,
                link,
                cmd,
                dist,
                speed,
                degrees_to_counts(tenths_deg as f32 / 10.0),
            )
            .await;
        }
        Request::Heartbeat => link.send_status(st).await,
        Request::Reset => {
            st.pid0.reset();
            st.pid1.reset();
            st.odometry.cancel();
            st.brake = [false, false];
            st.sys = SysState::Uninit;
            link.send_ack(cmd).await;
        }
    }
}

/// Start a closed-loop Move/Rotate once the target converted to >0 encoder
/// counts (wheel geometry is firmware-side, so this check stays here).
async fn run_closed_loop(
    st: &mut State,
    link: &Link,
    cmd: RequestType,
    dir: DistDir,
    speed: u8,
    counts: i32,
) {
    if counts <= 0 {
        link.send_nack(cmd.as_u8(), ErrorCode::InvalidParam).await;
        return;
    }
    st.odometry.start(dir, counts);
    let [left, right] = dir.wheel_speeds(speed as i16);
    st.request(0, MotorCmd::Drive(left));
    st.request(1, MotorCmd::Drive(right));
    st.sys = SysState::Running;
    link.send_ack(cmd).await;
}

// --- Tasks ---
static PWM_TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

#[embassy_executor::task]
async fn command_task(state: &'static StateMutex) {
    let mut parser = RxParser::new();
    let link = Link::new();

    loop {
        let b = link.receive().await;
        feed_byte(b, &mut parser, &mut *state.lock().await, &link).await;
    }
}

#[esp_hal::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = esp_hal::init(esp_hal::Config::default());
    defmt::info!("base_control init");

    // Scheduler for embassy time.
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, p.FROM_CPU_INTR0);

    // LED (active low).
    let led = Output::new(p.GPIO8, esp_hal::gpio::Level::High, OutputConfig::default());

    // Motor PWM: 4x LEDC, 8-bit, fixed frequency (see `motor::PWM_FREQ_HZ`).
    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let pwm_timer = PWM_TIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_FREQ_HZ),
        })
        .unwrap();
    let ch_l_in1 = ledc.channel(channel::Number::Channel0, p.GPIO2);
    let ch_l_in2 = ledc.channel(channel::Number::Channel1, p.GPIO1);
    let ch_r_in1 = ledc.channel(channel::Number::Channel2, p.GPIO3);
    let ch_r_in2 = ledc.channel(channel::Number::Channel3, p.GPIO4);
    let mut l_in1 = ch_l_in1;
    let mut l_in2 = ch_l_in2;
    let mut r_in1 = ch_r_in1;
    let mut r_in2 = ch_r_in2;
    for ch in [&mut l_in1, &mut l_in2, &mut r_in1, &mut r_in2] {
        ch.configure(channel::config::Config {
            timer: pwm_timer,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();
    }
    let motors = Motors::new(l_in1, l_in2, r_in1, r_in2);
    defmt::info!("boot: motors ok");

    // Encoders with pull-ups + CHANGE interrupts (pins owned by the driver).
    // State first, handler last: no interrupt can observe an empty state.
    let cfg = InputConfig::default().with_pull(Pull::Up);
    let mut e0a = Input::new(p.GPIO7, cfg);
    let mut e0b = Input::new(p.GPIO10, cfg);
    let mut e1a = Input::new(p.GPIO5, cfg);
    let mut e1b = Input::new(p.GPIO6, cfg);
    e0a.listen(Event::AnyEdge);
    e0b.listen(Event::AnyEdge);
    e1a.listen(Event::AnyEdge);
    e1b.listen(Event::AnyEdge);
    encoder::Encoders::install(e0a, e0b, e1a, e1b);
    let mut io = Io::new(p.IO_MUX);
    io.set_interrupt_handler(encoder::gpio_handler);

    let mut st = State::new();
    defmt::info!("boot: state ok");
    st.last_rpm_ms = Instant::now().as_millis();
    st.last_led_ms = st.last_rpm_ms;
    let state = service::control::STATE.init(StateMutex::new(st));

    spawner.spawn(command_task(state).unwrap());
    spawner.spawn(service::control::control_task(state, motors, led).unwrap());
    #[cfg(feature = "uart")]
    spawner.spawn(uart::uart_task(p.UART0, p.GPIO21, p.GPIO20).unwrap());
    #[cfg(feature = "ble")]
    {
        // Heap for the BLE controller/host.
        esp_alloc::heap_allocator!(size: 72 * 1024);
        spawner.spawn(ble::ble_task(p.BT).unwrap());
    }

    core::future::pending::<()>().await;
}
