//! ESP32-C3 下位机链路：命令下发、应答解析、状态缓存与安全看门狗。
//!
//! 传输层：与 ESP32-C3 下位机的协议对话（命令下发、应答解析、状态缓存）
//! 与节奏/安全机制（变化才发、心跳、死手开关）。
//!
//! 分层：串口字节流在 `super::link`（链路层，只搬字节/帧）；协议编解码与
//! 消息类型在 `protocol` crate；**应用策略**（什么时候补 `Init`、模式仲裁、
//! 控制律）在 [`crate::control::session`]，本模块不解释业务语义。
//!
//! 线程模型：
//! - 主线程（视觉管线）只调用 [`Car::set_desired`] / [`Car::brake`] /
//!   [`Car::coast`] 表达“意图”，[`Car::init`] / [`Car::reset`] 排一条控制面请求；
//! - **串口只由写线程写**（每 25ms 一拍）：控制面请求优先发，意图
//!   **变化才下发**（UART 是可信链路，协议有校验和、固件逐条 ACK，不做周期
//!   重发）；同时按 [`CarConfig::heartbeat_period`] 自动发心跳轮询 `Status`
//!   （不受视觉管线卡顿影响）；意图超过 [`CarConfig::watchdog_timeout`] 没被
//!   调用方报活（[`Car::refresh_intent`] 等）时下发 [`CarConfig::fallback`]
//!   （死手开关，不解释意图语义）；
//! - 读线程在链路层：帧解析后回调到状态机，把 ACK/NACK/Status 归入类型化状态。

use std::io;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::warn;

use super::link::Link;
use super::protocol::{
    Frame, MotorTarget, Request, RequestType, Response, RxEvent, Status, SysState,
};

/// 这些请求的负载固定，组帧不可能失败。
fn frame_of(request: Request) -> Frame {
    request.to_frame().expect("固定负载组帧不会失败")
}

/// 链路配置。
#[derive(Debug, Clone)]
pub struct CarConfig {
    pub port: String,
    pub baud: u32,
    pub heartbeat_period: Duration,
    pub watchdog_timeout: Duration,
    /// 意图刷新超时后下发的兜底意图（默认滑行）。
    pub fallback: Request,
    /// 多久没收到 `Status` 算链路不可用（[`LinkState::up`] / HUD 判定，不发报文）。
    pub status_timeout: Duration,
}

impl Default for CarConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyS1".to_string(),
            baud: 115_200,
            heartbeat_period: Duration::from_millis(500),
            watchdog_timeout: Duration::from_millis(800),
            fallback: Request::Stop {
                target: MotorTarget::Both,
            },
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

/// 链路状态（类型化快照；表现层自己决定怎么展示）。
#[derive(Debug, Clone, Default)]
pub struct LinkState {
    /// 最近是否收到过 `Status`（[`CarConfig::status_timeout`] 内）。
    pub up: bool,
    /// 最近一次 `Status` 距今的时间。
    pub age: Option<Duration>,
    /// 固件状态快照（无应答为 `None`）。
    pub status: Option<Status>,
    /// 链路计数。
    pub counters: Counters,
    /// 最近一帧可解析的应答（含到达时刻；文本/展示由调用方决定）。
    pub last_response: Option<(Response, Instant)>,
}

struct InnerState {
    desired: Request,
    desired_at: Instant,
    last_sent: Option<Request>,
    pending: Option<Request>,
    last_heartbeat_at: Instant,
    status: Option<Status>,
    status_at: Option<Instant>,
    last_ack: Option<(u8, Instant)>,
    last_nack: Option<(u8, u8, Instant)>,
    /// 最近一帧可解析的应答（含到达时刻）。
    last_response: Option<(Response, Instant)>,
    counters: Counters,
}

struct Inner {
    /// 链路层：串口 + 读线程（帧事件回调到 [`Inner::feed`]）。
    link: Link,
    cfg: CarConfig,
    state: Mutex<InnerState>,
}

impl Inner {
    /// 写一帧到链路（失败计数 + 节流告警）。
    fn send(&self, frame: Frame) -> bool {
        match self.link.write(frame.as_bytes()) {
            Ok(()) => true,
            Err(e) => {
                let mut st = self.state.lock().unwrap();
                st.counters.write_errors += 1;
                let n = st.counters.write_errors;
                drop(st);
                if n == 1 || n.is_multiple_of(200) {
                    warn!("串口写入失败（累计 {n} 次）：{e}");
                }
                false
            }
        }
    }

    /// 处理链路事件：帧 → 类型化状态；校验失败只计数。
    fn feed(&self, event: RxEvent<'_>) {
        match event {
            RxEvent::Frame { cmd, payload } => {
                // 协议是闭合的：未知应答号/长度不符直接忽略（校验错另有计数）。
                let Ok(response) = Response::decode(cmd, payload) else {
                    return;
                };
                let now = Instant::now();
                let mut st = self.state.lock().unwrap();
                st.last_response = Some((response, now));
                match response {
                    Response::Ack { cmd } => {
                        st.counters.acks += 1;
                        st.last_ack = Some((cmd, now));
                    }
                    Response::Nack { cmd, error } => {
                        st.counters.nacks += 1;
                        st.last_nack = Some((cmd, error.as_u8(), now));
                    }
                    Response::Status(status) => {
                        st.status = Some(status);
                        st.status_at = Some(now);
                    }
                    // PidData 只用于调试。
                    Response::PidData(_) => {}
                }
            }
            RxEvent::ChecksumFail { .. } => {
                self.state.lock().unwrap().counters.checksum_fails += 1;
            }
            RxEvent::TooLong { .. } | RxEvent::None => {}
        }
    }

    fn writer_task(self: Arc<Self>) {
        while self.link.running().load(Ordering::Relaxed) {
            // 1) 死手开关：应用这么久没刷新意图（`set_desired`/`coast`/`brake`）
            //    -> 下发兜底意图（下面统一下发）。传输层不解释意图语义。
            let watchdog = {
                let mut st = self.state.lock().unwrap();
                let stale = st.desired != self.cfg.fallback
                    && st.desired_at.elapsed() > self.cfg.watchdog_timeout;
                if stale {
                    st.desired = self.cfg.fallback;
                    st.desired_at = Instant::now();
                    st.counters.watchdog_trips += 1;
                }
                stale
            };
            if watchdog {
                warn!("看门狗：意图超时未刷新，下发兜底意图");
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
        let link = Link::open(&cfg.port, cfg.baud)?;
        let now = Instant::now();
        let inner = Arc::new(Inner {
            link,
            cfg,
            state: Mutex::new(InnerState {
                pending: None,
                desired: Request::Stop {
                    target: MotorTarget::Both,
                },
                desired_at: now,
                last_sent: None,
                last_heartbeat_at: now.checked_sub(Duration::from_secs(1)).unwrap_or(now),
                status: None,
                status_at: None,
                last_ack: None,
                last_nack: None,
                last_response: None,
                counters: Counters::default(),
            }),
        });
        // 读线程在链路层：解析出的事件回调到状态机（链路只管字节/帧）
        inner.link.spawn_reader({
            let inner = Arc::clone(&inner);
            move |event| inner.feed(event)
        });
        // 写线程在传输层：意图/心跳/死手开关的节奏由它掌握
        let writer = {
            let inner = Arc::clone(&inner);
            thread::spawn(move || inner.writer_task())
        };
        Ok(Self {
            inner,
            threads: Mutex::new(vec![writer]),
        })
    }

    /// 刷新意图时间戳（死手开关）：应用**每个控制节拍**都应调用——即使这一拍
    /// 不改变意图（比如锁存制动期间返回 `Output::Hold`）。停止调用
    /// [`CarConfig::watchdog_timeout`] 之后，下发线程会切到 [`CarConfig::fallback`]。
    pub fn refresh_intent(&self) {
        self.inner.state.lock().unwrap().desired_at = Instant::now();
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
        self.inner.state.lock().unwrap().pending = Some(Request::Init);
    }

    /// 请求 `Reset`（回 Uninit，未初始化状态下电机滑行）；由下发线程发出。
    pub fn reset(&self) {
        self.inner.state.lock().unwrap().pending = Some(Request::Reset);
    }

    /// 当前链路状态（一次锁拿到全部；HUD 与应用策略共用）。
    pub fn state(&self) -> LinkState {
        let st = self.inner.state.lock().unwrap();
        LinkState {
            up: st
                .status_at
                .is_some_and(|at| at.elapsed() < self.inner.cfg.status_timeout),
            age: st.status_at.map(|at| at.elapsed()),
            status: st.status,
            counters: st.counters,
            last_response: st.last_response,
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

    /// 停车并退出收发线程；幂等（多线程同时调用也只执行一次），`Drop` 会自动调用。
    pub fn shutdown(&self) {
        if self.inner.link.running().swap(false, Ordering::SeqCst) {
            // 直接补一帧 Stop（滑行），不依赖下发线程的时序。
            self.inner.send(frame_of(Request::Stop {
                target: MotorTarget::Both,
            }));
        }
        self.inner.link.stop(); // 读线程（链路层）先收
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
    use crate::transport::protocol::{ResponseType, RxParser};
    use std::ffi::CStr;
    use std::fs::File;
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
        let state = car.state();
        assert!(state.up);
        let st = state.status.unwrap();
        assert_eq!(st.rpm, [11, -11]);
        assert_eq!(st.sys, SysState::Running);
        assert!(
            state
                .last_response
                .unwrap()
                .0
                .to_string()
                .contains("Status Running")
        );

        // 只报活、不改变意图（`Output::Hold` 的情形）→ 死手开关不触发
        for _ in 0..6 {
            thread::sleep(Duration::from_millis(40));
            car.refresh_intent();
        }
        assert_eq!(car.state().counters.watchdog_trips, 0);

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
        assert!(car.state().counters.watchdog_trips >= 1);
        // 停车后不会再触发看门狗（意图已经是 Stop）
        thread::sleep(Duration::from_millis(80));
        assert_eq!(car.state().counters.watchdog_trips, 1);

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
        assert!(!car.state().up);
        assert!(car.state().counters.write_errors == 0);
        car.shutdown();
    }
}
