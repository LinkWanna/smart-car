//! ESP32-C3 下位机链路：命令下发、应答解析、状态缓存与安全看门狗。
//!
//! 线程模型：
//! - 主线程（视觉管线）只调用 [`Car::set_desired`] / [`Car::brake`] /
//!   [`Car::coast`] 表达“意图”，[`Car::init`] / [`Car::reset`] 排一条控制面请求；
//! - **串口只由写线程写**（每 25ms 一拍）：控制面请求优先发，意图
//!   **变化才下发**（UART 是可信链路，协议有校验和、固件逐条 ACK，不做周期
//!   重发）；同时按 [`CarConfig::heartbeat_period`] 自动发心跳轮询 `Status`
//!   （不受视觉管线卡顿影响）；驱动意图超过 [`CarConfig::watchdog_timeout`]
//!   没有被主线程刷新时自动滑行停车；
//! - 读线程以 `O_NONBLOCK` 轮询串口，把 ACK/NACK/Status 解析进共享状态。
//!
//! 说明：ESP32-C3 固件的 700ms 指令超时只在 BLE 已连接时生效，UART 链路
//! 由上位机负责看门狗；进程退出（含 panic 展开）时 [`Drop`] 会补一帧
//! `Stop`，被 SIGKILL 则没有机会发送。

use std::fs::File;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::warn;

use super::protocol::{
    ErrorCode, Frame, MotorTarget, Request, RequestType, Response, RxEvent, RxParser, Status,
    SysState,
};
use super::serial;

/// 这些请求的负载固定，组帧不可能失败。
fn frame_of(request: Request) -> Frame {
    request.to_frame().expect("固定负载组帧不会失败")
}

/// 链路配置。
#[derive(Debug, Clone)]
pub struct CarConfig {
    pub port: String,
    pub baud: u32,
    /// 心跳周期：周期发 `Heartbeat` 轮询 `Status`（探活 + 刷新 HUD；
    /// 也是链路上唯一的周期性报文）。
    pub heartbeat_period: Duration,
    /// 驱动意图超过这么久没有被调用方刷新（[`Car::set_desired`]）→
    /// 本地改成滑行下发（控制线程卡死的兜底；固件侧没有指令超时）。
    pub watchdog_timeout: Duration,
    /// 多久没收到 `Status` 算链路不可用（[`Car::link_ok`] / HUD 判定，不发报文）。
    pub status_timeout: Duration,
}

impl Default for CarConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyS1".to_string(),
            baud: 115_200,
            heartbeat_period: Duration::from_millis(500),
            watchdog_timeout: Duration::from_millis(800),
            status_timeout: Duration::from_millis(1500),
        }
    }
}

/// 链路计数（HUD/排障用）。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    pub acks: u32,
    pub nacks: u32,
    pub checksum_fails: u32,
    pub write_errors: u32,
    /// 看门狗强制停车的次数。
    pub watchdog_trips: u32,
}

/// 下位机链路快照（HUD 用）。
#[derive(Debug, Clone, Default)]
pub struct LinkSnapshot {
    /// 最近是否收到过 `Status`（链路可用）。
    pub link_ok: bool,
    /// 最近一次 `Status` 距今的时间。
    pub link_age: Option<Duration>,
    /// 固件状态机（`Uninit`/`Ready`/`Running`），无应答为 `None`。
    pub sys: Option<String>,
    /// 双轮实测转速（RPM），`[左, 右]`。
    pub rpm: [i16; 2],
    /// 固件闭环运动是否运行中。
    pub dist_active: bool,
    /// 0 = 无/运行中，1 = 已到达目标。
    pub dist_result: u8,
    /// 链路计数。
    pub counters: Counters,
    /// 最近一帧应答的可读文本。
    pub last_frame: String,
}

struct InnerState {
    desired: Request,
    desired_at: Instant,
    last_sent: Option<Request>,
    pending: Option<Request>,
    last_heartbeat_at: Instant,
    last_init_at: Instant,
    status: Option<Status>,
    status_at: Option<Instant>,
    last_ack: Option<(u8, Instant)>,
    last_nack: Option<(u8, u8, Instant)>,
    last_frame: String,
    counters: Counters,
}

struct Inner {
    file: File,
    cfg: CarConfig,
    running: AtomicBool,
    write_lock: Mutex<()>,
    state: Mutex<InnerState>,
}

impl Inner {
    /// 写一帧到串口（`O_NONBLOCK` 下对 `EAGAIN`/`WouldBlock` 做 50ms 重试）。
    fn send(&self, frame: Frame) -> bool {
        let _guard = self.write_lock.lock().unwrap();
        let data = frame.as_bytes();
        let deadline = Instant::now() + Duration::from_millis(50);
        let mut file = &self.file;
        let mut off = 0;
        let mut failure: Option<String> = None;
        while off < data.len() {
            match file.write(&data[off..]) {
                Ok(0) => {
                    failure = Some("写入返回 0".to_string());
                    break;
                }
                Ok(n) => off += n,
                Err(e) => {
                    failure = Some(e.to_string());
                    match e.kind() {
                        io::ErrorKind::Interrupted => continue,
                        io::ErrorKind::WouldBlock => {
                            if Instant::now() >= deadline {
                                break;
                            }
                            thread::sleep(Duration::from_millis(1));
                        }
                        _ => break,
                    }
                }
            }
        }
        if off == data.len() {
            return true;
        }
        let mut st = self.state.lock().unwrap();
        st.counters.write_errors += 1;
        let n = st.counters.write_errors;
        drop(st);
        if n == 1 || n.is_multiple_of(200) {
            warn!(
                "串口写入失败（累计 {} 次）：{}",
                n,
                failure.unwrap_or_else(|| "写入不完整".to_string())
            );
        }
        false
    }

    fn reader_task(self: Arc<Self>) {
        let mut parser = RxParser::new();
        let mut buf = [0u8; 128];
        let mut file = &self.file;
        while self.running.load(Ordering::Relaxed) {
            match file.read(&mut buf) {
                // 读到 0 字节（非阻塞下少见）：歇一下再看退出标志。
                Ok(0) => thread::sleep(Duration::from_millis(5)),
                Ok(n) => {
                    for &byte in &buf[..n] {
                        match parser.feed(byte) {
                            RxEvent::Frame { cmd, payload } => {
                                let text = describe(cmd, payload);
                                let mut st = self.state.lock().unwrap();
                                st.last_frame = text;
                                match Response::decode(cmd, payload) {
                                    Ok(Response::Ack { cmd }) => {
                                        st.counters.acks += 1;
                                        st.last_ack = Some((cmd, Instant::now()));
                                    }
                                    Ok(Response::Nack { cmd, error }) => {
                                        st.counters.nacks += 1;
                                        st.last_nack = Some((cmd, error.as_u8(), Instant::now()));
                                    }
                                    Ok(Response::Status(status)) => {
                                        st.status = Some(status);
                                        st.status_at = Some(Instant::now());
                                    }
                                    // PidData 只用于调试；无法解码的应答（未知应答号/长度不符）忽略。
                                    Ok(Response::PidData(_)) | Err(_) => {}
                                }
                            }
                            RxEvent::ChecksumFail { .. } => {
                                self.state.lock().unwrap().counters.checksum_fails += 1;
                            }
                            RxEvent::TooLong { .. } | RxEvent::None => {}
                        }
                    }
                }
                Err(e) => match e.kind() {
                    io::ErrorKind::Interrupted => {}
                    // Linux 上 EWOULDBLOCK == EAGAIN
                    io::ErrorKind::WouldBlock => thread::sleep(Duration::from_millis(5)),
                    // 设备拔出/异常：慢速重试，避免刷屏。
                    _ => thread::sleep(Duration::from_millis(50)),
                },
            }
        }
    }

    fn writer_task(self: Arc<Self>) {
        while self.running.load(Ordering::Relaxed) {
            // 1) 看门狗：驱动意图长时间没有被刷新 -> 改成滑行（下面统一下发）。
            let watchdog = {
                let mut st = self.state.lock().unwrap();
                let stale = matches!(st.desired, Request::SetSpeeds { .. })
                    && st.desired_at.elapsed() > self.cfg.watchdog_timeout;
                if stale {
                    st.desired = Request::Stop {
                        target: MotorTarget::Both,
                    };
                    st.desired_at = Instant::now();
                    st.counters.watchdog_trips += 1;
                }
                stale
            };
            if watchdog {
                warn!("看门狗：驱动意图超时未刷新，滑行停车");
            }

            // 2) 控制面请求（Init/Reset）优先发。
            if let Some(request) = self.state.lock().unwrap().pending.take() {
                self.send(frame_of(request));
            }

            // 3) 意图只在变化时下发（UART 可信 + 固件 ACK，不做周期重发）。
            let pending = {
                let st = self.state.lock().unwrap();
                (st.last_sent != Some(st.desired)).then_some(st.desired)
            };
            if let Some(desired) = pending
                && self.send(frame_of(desired))
            {
                self.state.lock().unwrap().last_sent = Some(desired);
            }

            // 4) 自动心跳：Status 轮询不依赖视觉管线是否卡顿。
            let heartbeat_due = {
                let st = self.state.lock().unwrap();
                st.last_heartbeat_at.elapsed() >= self.cfg.heartbeat_period
            };
            if heartbeat_due {
                self.send(frame_of(Request::Heartbeat));
                let mut st = self.state.lock().unwrap();
                st.last_heartbeat_at = Instant::now();
            }

            thread::sleep(Duration::from_millis(25));
        }
    }
}

/// 一帧应答的可读文本（HUD/日志用）。
fn describe(cmd: u8, payload: &[u8]) -> String {
    match Response::decode(cmd, payload) {
        Ok(Response::Ack { cmd }) => format!("ACK {}", request_name(cmd)),
        Ok(Response::Nack { cmd, error }) => format!("NACK {} {error}", request_name(cmd)),
        Ok(Response::Status(status)) => format!(
            "Status {} rpm=({}, {}) dist_active={} dist_result={}",
            status.sys, status.rpm[0], status.rpm[1], status.dist_active as u8, status.dist_result
        ),
        Ok(Response::PidData(pid)) => format!(
            "PidData kp={:.2} ki={:.2} kd={:.2}",
            pid.kp as f32 / 100.0,
            pid.ki as f32 / 100.0,
            pid.kd as f32 / 100.0
        ),
        Err(_) => payload
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" "),
    }
}

/// 回显命令号的可读名字。
fn request_name(cmd: u8) -> String {
    RequestType::from_u8(cmd).map_or_else(|| format!("0x{cmd:02X}"), |c| c.to_string())
}

/// 与下位机的链路句柄；`Drop` 时自动滑行停车并退出线程。
///
/// 收/发线程在内部，句柄本身是 `Sync` 的：可以放进 `Arc` 供网页服务等多线程
/// 共享（所有方法都取 `&self`，[`Car::shutdown`] 也不例外）。
pub struct Car {
    inner: Arc<Inner>,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl Car {
    /// 打开串口并启动收发线程。打开失败直接返回错误（上位机启动时应当报错退出）。
    pub fn open(cfg: CarConfig) -> io::Result<Self> {
        let file = serial::open(&cfg.port, cfg.baud)?;
        let now = Instant::now();
        let inner = Arc::new(Inner {
            file,
            cfg,
            running: AtomicBool::new(true),
            write_lock: Mutex::new(()),
            state: Mutex::new(InnerState {
                pending: None,
                desired: Request::Stop {
                    target: MotorTarget::Both,
                },
                desired_at: now,
                last_sent: None,
                last_heartbeat_at: now.checked_sub(Duration::from_secs(1)).unwrap_or(now),
                last_init_at: now,
                status: None,
                status_at: None,
                last_ack: None,
                last_nack: None,
                last_frame: String::from("(尚未收到应答)"),
                counters: Counters::default(),
            }),
        });
        let mut threads = Vec::with_capacity(2);
        threads.push({
            let inner = Arc::clone(&inner);
            thread::spawn(move || inner.reader_task())
        });
        threads.push({
            let inner = Arc::clone(&inner);
            thread::spawn(move || inner.writer_task())
        });
        Ok(Self {
            inner,
            threads: Mutex::new(threads),
        })
    }

    /// 表达双轮速度意图（`-100..100`）；由下发线程在变化时下发。
    pub fn set_desired(&self, left: i16, right: i16) {
        let mut st = self.inner.state.lock().unwrap();
        st.desired = Request::SetSpeeds { left, right };
        st.desired_at = Instant::now();
    }

    /// 意图改为滑行。
    pub fn coast(&self) {
        let mut st = self.inner.state.lock().unwrap();
        st.desired = Request::Stop {
            target: MotorTarget::Both,
        };
        st.desired_at = Instant::now();
    }

    /// 意图改为短接制动（保持到下一条速度指令）。
    pub fn brake(&self) {
        let mut st = self.inner.state.lock().unwrap();
        st.desired = Request::Brake {
            target: MotorTarget::Both,
        };
        st.desired_at = Instant::now();
    }

    /// 请求 `Init`（编码器清零 + 进入 `Ready`）；由下发线程发出。
    pub fn init(&self) {
        let mut st = self.inner.state.lock().unwrap();
        st.last_init_at = Instant::now();
        st.pending = Some(Request::Init);
    }

    /// 请求 `Reset`（回 Uninit，未初始化状态下电机滑行）；由下发线程发出。
    pub fn reset(&self) {
        self.inner.state.lock().unwrap().pending = Some(Request::Reset);
    }

    /// 最近一次 `Status`。
    pub fn status(&self) -> Option<Status> {
        self.inner.state.lock().unwrap().status
    }

    /// 链路是否可用：最近 [`CarConfig::status_timeout`] 内收到过 `Status`。
    pub fn link_ok(&self) -> bool {
        let st = self.inner.state.lock().unwrap();
        st.status_at
            .is_some_and(|at| at.elapsed() < self.inner.cfg.status_timeout)
    }

    /// 最近一次收到 `Status` 距今的时间。
    pub fn link_age(&self) -> Option<Duration> {
        self.inner
            .state
            .lock()
            .unwrap()
            .status_at
            .map(|at| at.elapsed())
    }

    pub fn counters(&self) -> Counters {
        self.inner.state.lock().unwrap().counters
    }

    /// 最近一帧应答的可读文本（HUD 用）。
    pub fn last_frame(&self) -> String {
        self.inner.state.lock().unwrap().last_frame.clone()
    }

    /// 当前链路快照（HUD 用；一次锁拿到全部）。
    pub fn snapshot(&self) -> LinkSnapshot {
        let status = self.status();
        LinkSnapshot {
            link_ok: self.link_ok(),
            link_age: self.link_age(),
            sys: status.map(|s| s.sys.to_string()),
            rpm: status.map_or([0, 0], |s| s.rpm),
            dist_active: status.is_some_and(|s| s.dist_active),
            dist_result: status.map_or(0, |s| s.dist_result),
            counters: self.counters(),
            last_frame: self.last_frame(),
        }
    }

    /// 是否已经到达 Ready/Running（收到过 `Init` 的 ACK，或状态允许驱动）。
    fn is_ready(&self) -> bool {
        let st = self.inner.state.lock().unwrap();
        if let Some((cmd, at)) = st.last_ack
            && cmd == RequestType::Init.as_u8()
            && at.elapsed() < Duration::from_secs(1)
        {
            return true;
        }
        st.status
            .map(|s| s.sys)
            .is_some_and(|s| s >= SysState::Ready)
    }

    /// 重试 `Init` 直到下位机就绪，超时返回 `false`（此时仍可继续跑视觉，
    /// 但速度指令会被固件 NACK）。
    pub fn ensure_ready(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            self.init();
            let wait_until = Instant::now() + Duration::from_millis(400);
            loop {
                if self.is_ready() {
                    return true;
                }
                if Instant::now() >= wait_until {
                    break;
                }
                thread::sleep(Duration::from_millis(10));
            }
            if Instant::now() >= deadline {
                return false;
            }
        }
    }

    /// 向下位机“补初始化”：处于 Uninit 或最近因状态不对被 NACK 时请求
    /// `Init`（限频 1Hz，由下发线程发出）。
    pub fn maybe_reinit(&self) {
        let mut st = self.inner.state.lock().unwrap();
        if st.last_init_at.elapsed() < Duration::from_secs(1) {
            return;
        }
        let uninit = st.status.map(|s| s.sys) == Some(SysState::Uninit);
        let wrong_state = matches!(
            st.last_nack,
            Some((cmd, err, at))
                if cmd == RequestType::SetSpeeds.as_u8()
                    && err == ErrorCode::WrongState.as_u8()
                    && at.elapsed() < Duration::from_secs(2)
        );
        if uninit || wrong_state {
            st.last_init_at = Instant::now();
            st.pending = Some(Request::Init);
        }
    }

    /// 停车并退出收发线程；幂等（多线程同时调用也只执行一次），`Drop` 会自动调用。
    pub fn shutdown(&self) {
        if self.inner.running.swap(false, Ordering::SeqCst) {
            // 直接补一帧 Stop（滑行），不依赖下发线程的时序。
            self.inner.send(frame_of(Request::Stop {
                target: MotorTarget::Both,
            }));
        }
        let mut threads = self.threads.lock().unwrap();
        for handle in threads.drain(..) {
            crate::join_with_timeout(handle, "链路");
        }
    }
}

impl Drop for Car {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::protocol::ResponseType;
    use std::ffi::CStr;
    use std::os::unix::io::{AsRawFd, FromRawFd};

    /// 用 PTY 模拟 ESP32-C3：测试持 master 侧，Car 用 slave 侧。
    fn open_pty() -> Option<(File, String)> {
        unsafe {
            let master = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
            if master < 0 {
                return None; // 环境不支持 PTY，跳过测试
            }
            if libc::grantpt(master) != 0 || libc::unlockpt(master) != 0 {
                libc::close(master);
                return None;
            }
            let mut name = [0 as libc::c_char; 256];
            if libc::ptsname_r(master, name.as_mut_ptr(), name.len()) != 0 {
                libc::close(master);
                return None;
            }
            let path = CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned();
            Some((File::from_raw_fd(master), path))
        }
    }

    fn write_bytes(master: &File, data: &[u8]) {
        let n = unsafe {
            libc::write(
                master.as_raw_fd(),
                data.as_ptr() as *const libc::c_void,
                data.len(),
            )
        };
        assert_eq!(n as usize, data.len());
    }

    fn fake_response(master: &File, cmd: u8, payload: &[u8]) {
        write_bytes(master, Frame::new(cmd, payload).unwrap().as_bytes());
    }

    /// 读一帧应答，跳过无关帧；超时返回 None。
    fn read_frame(master: &File, timeout: Duration) -> Option<(u8, Vec<u8>)> {
        let fd = master.as_raw_fd();
        let mut parser = RxParser::new();
        let mut buf = [0u8; 64];
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if n > 0 {
                for &b in &buf[..n as usize] {
                    if let RxEvent::Frame { cmd, payload } = parser.feed(b) {
                        return Some((cmd, payload.to_vec()));
                    }
                }
            } else {
                thread::sleep(Duration::from_millis(2));
            }
        }
        None
    }

    fn read_until(master: &File, cmd: u8, timeout: Duration) -> Option<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let (c, p) = read_frame(master, Duration::from_millis(100))?;
            if c == cmd {
                return Some(p);
            }
        }
        None
    }

    #[test]
    fn pty_link_end_to_end() {
        let Some((master, slave_path)) = open_pty() else {
            eprintln!("跳过：当前环境不支持 PTY");
            return;
        };
        unsafe {
            libc::fcntl(master.as_raw_fd(), libc::F_SETFL, libc::O_NONBLOCK);
        }
        let cfg = CarConfig {
            port: slave_path,
            watchdog_timeout: Duration::from_millis(150),
            heartbeat_period: Duration::from_millis(50),
            status_timeout: Duration::from_millis(300),
            ..Default::default()
        };
        let car = Car::open(cfg).expect("打开 PTY slave 失败");

        // 启动时下发线程先送一帧 Stop(Both)，保证上一次遗留的运动被取消。
        assert_eq!(
            read_until(&master, RequestType::Stop.as_u8(), Duration::from_secs(1)),
            Some(vec![MotorTarget::Both.as_u8()])
        );

        // Drive 意图 -> SetSpeeds(30, -20)，多字节小端（0x001E / 0xFFEC）。
        car.set_desired(30, -20);
        assert_eq!(
            read_until(
                &master,
                RequestType::SetSpeeds.as_u8(),
                Duration::from_secs(1)
            ),
            Some(vec![0x1E, 0x00, 0xEC, 0xFF])
        );

        // 模拟固件应答：ACK SetSpeeds + Status(Running, 11/-11)，多字节小端。
        fake_response(
            &master,
            ResponseType::Ack.as_u8(),
            &[RequestType::SetSpeeds.as_u8()],
        );
        fake_response(
            &master,
            ResponseType::Status.as_u8(),
            &[
                SysState::Running.as_u8(),
                0x0B,
                0x00,
                0xF5,
                0xFF,
                0x00,
                0x00,
            ],
        );
        thread::sleep(Duration::from_millis(100));
        assert!(car.link_ok());
        let st = car.status().unwrap();
        assert_eq!(st.rpm, [11, -11]);
        assert_eq!(st.sys, SysState::Running);
        assert!(car.last_frame().contains("Status Running"));

        // 心跳：应当过一段时间出现 Heartbeat 帧
        assert_eq!(
            read_until(
                &master,
                RequestType::Heartbeat.as_u8(),
                Duration::from_secs(1)
            ),
            Some(vec![])
        );

        // 不再刷新 Drive -> 看门狗应在 ~150ms 后改成滑行停车
        assert_eq!(
            read_until(&master, RequestType::Stop.as_u8(), Duration::from_secs(1)),
            Some(vec![MotorTarget::Both.as_u8()])
        );
        assert!(car.counters().watchdog_trips >= 1);
        // 停车后不会再触发看门狗（意图已经是 Stop）
        thread::sleep(Duration::from_millis(80));
        assert_eq!(car.counters().watchdog_trips, 1);

        car.shutdown();
    }

    #[test]
    fn ensure_ready_times_out_without_firmware() {
        let Some((_master, slave_path)) = open_pty() else {
            eprintln!("跳过：当前环境不支持 PTY");
            return;
        };
        let cfg = CarConfig {
            port: slave_path,
            ..Default::default()
        };
        let car = Car::open(cfg).expect("打开 PTY slave 失败");
        assert!(!car.ensure_ready(Duration::from_millis(250)));
        assert!(!car.link_ok());
        assert!(car.counters().write_errors == 0);
        car.shutdown();
    }
}
