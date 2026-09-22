//! 网页服务器：rouille（HTTP + WebSocket）+ serde_json 状态推送 + 控制节拍。
//!
//! 线程模型：
//!
//! - **HTTP 请求**：`rouille::Server` 用线程池处理，[`Server::run`] 每 200ms
//!   轮询 `running`（`poll_timeout`），退出前 `join` 等在途请求；
//! - **WebSocket 会话**：握手交给 `rouille::websocket`，每个连接一个线程——
//!   `Websocket::next()` 阻塞读客户端消息，收到消息后按 `status_hz` /
//!   `video_fps` 推送状态与 JPEG（页面每 100ms 发按键心跳，天然给出推送节奏）；
//!   客户端静默时输入看门狗（[`Teleop`]）会把车停下；
//! - **控制线程**（[`Server::spawn_control`]）：按 `control_hz` 推进 [`Teleop`]
//!   并把 [`Output`] 落到 [`DriveTarget`]；
//! - **采集线程**：[`super::video::CameraStream`]，与本模块解耦。
//!
//! 页面本身（`assets/index.html`）通过 `include_str!` 内嵌，零外部资源。

use std::io;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use rouille::websocket::{self, Websocket};
use rouille::{Request, Response};
use serde::Deserialize;
use serde_json::{Value, json};

use super::DriveTarget;
use super::teleop::{Action, Keys, Output, Teleop, TeleopConfig};
use super::video::CameraStream;

/// 内嵌的网页（HTML + CSS + JS 单文件，零外部资源）。
const INDEX_HTML: &str = include_str!("assets/index.html");
/// HTTP 请求线程池大小（WebSocket 会话有独立线程，不占池子）。
const POOL_SIZE: usize = 8;
/// `run` 的轮询间隔（决定 Ctrl+C 后的退出延迟上限）。
const POLL_INTERVAL: Duration = Duration::from_millis(200);

/// 网页服务配置。
#[derive(Debug, Clone)]
pub struct WebConfig {
    /// 监听地址（`0.0.0.0` 表示所有网卡，AP 模式下即 `192.168.4.1`）。
    pub bind: String,
    /// 监听端口（默认 80，直接浏览器访问 `http://192.168.4.1`）。
    pub port: u16,
    /// 状态 JSON 推送频率（Hz）。
    pub status_hz: f32,
    /// JPEG 预览帧率上限（Hz）。
    pub video_fps: f32,
}

impl Default for WebConfig {
    fn default() -> Self {
        Self {
            bind: "0.0.0.0".to_string(),
            port: 80,
            status_hz: 10.0,
            video_fps: 10.0,
        }
    }
}

/// 客户端 -> 服务端消息。
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
enum ClientMessage {
    /// 完整按键快照（`keys` 是 `wasd` 的子集）。
    Keys { keys: String },
    /// 动作按钮（`stop`/`brake`/`init`/`reset`）。
    Action { action: String },
    /// 应用层 ping，回一条 pong。
    Ping,
}

/// 所有连接共享的状态。
struct Shared {
    cfg: WebConfig,
    target: Arc<dyn DriveTarget>,
    teleop: Mutex<Teleop>,
    camera: Option<Arc<CameraStream>>,
    running: Arc<AtomicBool>,
    clients: AtomicU64,
    next_client_id: AtomicU64,
    started: Instant,
}

impl Shared {
    /// 控制节拍：推进遥控控制律并落地输出。
    fn control_loop(self: &Arc<Self>) {
        let hz = {
            let teleop = self.teleop.lock().unwrap();
            teleop.config().control_hz.max(1.0)
        };
        let period = Duration::from_secs_f32(1.0 / hz);
        let mut next = Instant::now();
        while self.running.load(Ordering::Relaxed) {
            let out = self.teleop.lock().unwrap().tick(Instant::now());
            apply(&*self.target, out);
            self.target.maybe_reinit();
            next += period;
            let now = Instant::now();
            if next > now {
                thread::sleep(next - now);
            } else {
                next = now;
            }
        }
        eprintln!("[web] 控制线程退出，滑行停车");
        self.target.coast();
    }

    /// 一帧状态（WebSocket 推送与 `GET /api/status` 共用）。
    fn status_value(&self) -> Value {
        let now = Instant::now();
        let (hud, input_age) = {
            let teleop = self.teleop.lock().unwrap();
            (teleop.hud(), teleop.input_age(now))
        };
        let link = self.target.snapshot();
        let mut flags: Vec<&str> = Vec::new();
        if hud.flags.pivot {
            flags.push("PIVOT");
        }
        if hud.flags.reverse {
            flags.push("REV");
        }
        if hud.flags.braking {
            flags.push("BRAKE");
        }
        json!({
            "type": "status",
            "uptime": self.started.elapsed().as_secs_f32(),
            "link": link.link_ok,
            "link_age_ms": link.link_age.map(|d| d.as_millis() as u64),
            "sys": link.sys,
            "rpm": link.rpm,
            "dist": { "active": link.dist_active, "result": link.dist_result },
            "drive": {
                "throttle": hud.throttle,
                "steer": hud.steer,
                "flags": flags,
                "cmd": hud.cmd,
                "keys": hud.keys.to_string(),
                "input_age_ms": input_age.map(|d| d.as_millis() as u64),
            },
            "counters": {
                "acks": link.counters.acks,
                "nacks": link.counters.nacks,
                "checksum_fails": link.counters.checksum_fails,
                "write_errors": link.counters.write_errors,
                "watchdog_trips": link.counters.watchdog_trips,
            },
            "last_frame": link.last_frame,
            "camera": self.camera_value(),
            "clients": self.clients.load(Ordering::Relaxed),
        })
    }

    /// 连接建立时的问候消息（带页面需要的固定参数）。
    fn hello_value(&self) -> Value {
        let (control_hz, input_timeout_ms, max_speed, max_reverse) = {
            let teleop = self.teleop.lock().unwrap();
            let cfg = teleop.config();
            (
                cfg.control_hz,
                cfg.input_timeout.as_millis() as u64,
                cfg.max_speed,
                cfg.max_reverse,
            )
        };
        json!({
            "type": "hello",
            "version": 1,
            "control_hz": control_hz,
            "status_hz": self.cfg.status_hz,
            "video_fps": self.cfg.video_fps,
            "input_timeout_ms": input_timeout_ms,
            "max_speed": max_speed,
            "max_reverse": max_reverse,
            "camera": self.camera_value(),
        })
    }

    fn camera_value(&self) -> Value {
        let Some(camera) = &self.camera else {
            return Value::Null;
        };
        let st = camera.status();
        json!({
            "available": st.available,
            "device": st.device,
            "format": st.format,
            "frames": st.frames,
            "fps": st.fps,
            "age_ms": st.age_ms,
            "error": st.error,
        })
    }
}

/// 网页服务器句柄。
pub struct Server {
    inner: rouille::Server<Handler>,
    shared: Arc<Shared>,
    running: Arc<AtomicBool>,
}

type Handler = Box<dyn Fn(&Request) -> Response + Send + Sync>;

impl Server {
    /// 绑定监听地址；`running` 置 false 后 [`Server::run`] 退出。
    pub fn bind(
        cfg: WebConfig,
        teleop_cfg: TeleopConfig,
        target: Arc<dyn DriveTarget>,
        camera: Option<Arc<CameraStream>>,
        running: Arc<AtomicBool>,
    ) -> io::Result<Self> {
        let shared = Arc::new(Shared {
            cfg,
            target,
            teleop: Mutex::new(Teleop::new(teleop_cfg)),
            camera,
            running: Arc::clone(&running),
            clients: AtomicU64::new(0),
            next_client_id: AtomicU64::new(0),
            started: Instant::now(),
        });
        let handler_shared = Arc::clone(&shared);
        let handler: Handler = Box::new(move |request| route(request, &handler_shared));
        let addr = format!("{}:{}", shared.cfg.bind, shared.cfg.port);
        let inner = rouille::Server::new(addr.as_str(), handler)
            .map_err(|e| io::Error::other(format!("监听 {addr} 失败: {e}")))?
            .pool_size(POOL_SIZE);
        Ok(Self {
            inner,
            shared,
            running,
        })
    }

    /// 实际监听地址（测试里用 `port = 0` 拿随机端口）。
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        Ok(self.inner.server_addr())
    }

    /// 启动控制线程；返回的句柄在 [`Server::run`] 退出后 join。
    pub fn spawn_control(&self) -> JoinHandle<()> {
        let shared = Arc::clone(&self.shared);
        thread::spawn(move || shared.control_loop())
    }

    /// 请求循环；`running` 为 false 时排空在途请求并返回。
    pub fn run(&self) {
        // rouille 没有公开的停止接口，这里用轮询 `running` 的方式收尾。
        // WebSocket 会话在独立线程里阻塞读，进程退出时一并结束。
        while self.running.load(Ordering::Relaxed) {
            self.inner.poll_timeout(POLL_INTERVAL);
        }
        self.inner.join();
        eprintln!("[web] 已停止监听");
    }
}

/// HTTP 路由。
fn route(request: &Request, shared: &Arc<Shared>) -> Response {
    let url = request.url();
    let path = url.split('?').next().unwrap_or("/");
    match path {
        "/" | "/index.html" => Response::html(INDEX_HTML).with_no_cache(),
        "/api/status" => Response::json(&shared.status_value()).with_no_cache(),
        "/api/frame.jpg" => match shared.camera.as_ref().and_then(|c| c.latest()) {
            Some(jpeg) => Response::from_data("image/jpeg", jpeg.to_vec()).with_no_cache(),
            None => Response::text("预览不可用\n").with_status_code(503),
        },
        "/api/input" => {
            let keys = Keys::parse(&request.get_param("keys").unwrap_or_default());
            shared
                .teleop
                .lock()
                .unwrap()
                .set_keys(keys, 0, Instant::now());
            Response::json(&json!({ "ok": true, "keys": keys.to_string() }))
        }
        "/api/action" => {
            let name = request
                .get_param("action")
                .or_else(|| request.get_param("cmd"))
                .unwrap_or_default();
            let Some(action) = Action::parse(&name) else {
                return Response::text(format!("未知动作 {name:?}\n")).with_status_code(400);
            };
            let out = shared.teleop.lock().unwrap().action(action);
            apply(&*shared.target, out);
            Response::json(&json!({ "ok": true, "action": action.to_string() }))
        }
        "/api/events" => websocket_route(request, shared),
        "/favicon.ico" => Response::empty_204(),
        _ => Response::text("not found\n").with_status_code(404),
    }
}

/// WebSocket 升级：握手由 rouille 完成，会话交给独立线程。
fn websocket_route(request: &Request, shared: &Arc<Shared>) -> Response {
    let (response, receiver) = match websocket::start(request, None::<String>) {
        Ok(pair) => pair,
        Err(e) => {
            return Response::text(format!("WebSocket 握手失败: {e}\n")).with_status_code(400);
        }
    };
    let shared = Arc::clone(shared);
    thread::spawn(move || match receiver.recv() {
        Ok(socket) => websocket_session(socket, shared),
        Err(_) => eprintln!("[web] WebSocket 升级失败"),
    });
    response
}

/// 把控制律输出落到链路。
fn apply(target: &dyn DriveTarget, out: Output) {
    match out {
        Output::Drive { left, right } => target.set_speeds(left, right),
        Output::Coast => target.coast(),
        Output::Brake => target.brake(),
        Output::Hold => {}
        Output::Init => {
            target.init();
        }
        Output::Reset => {
            target.reset();
        }
    }
}

/// WebSocket 会话：阻塞读客户端消息，收到消息后按节拍推送状态与预览帧。
fn websocket_session(mut socket: Websocket, shared: Arc<Shared>) {
    let id = shared.next_client_id.fetch_add(1, Ordering::Relaxed) + 1;
    shared.clients.fetch_add(1, Ordering::Relaxed);
    println!("[web] WebSocket #{id} 已连接");

    let status_period = Duration::from_secs_f32(1.0 / shared.cfg.status_hz.max(0.5));
    let video_period = Duration::from_secs_f32(1.0 / shared.cfg.video_fps.max(0.5));
    let mut last_status = Instant::now()
        .checked_sub(status_period)
        .unwrap_or_else(Instant::now);
    let mut last_video = Instant::now()
        .checked_sub(video_period)
        .unwrap_or_else(Instant::now);
    let mut video_seq = 0u64;
    let mut alive = send_json(&mut socket, &shared.hello_value());

    while alive && shared.running.load(Ordering::Relaxed) {
        let now = Instant::now();
        if now.saturating_duration_since(last_status) >= status_period {
            alive = send_json(&mut socket, &shared.status_value());
            last_status = now;
        }
        if alive && now.saturating_duration_since(last_video) >= video_period {
            last_video = now;
            if let Some(camera) = &shared.camera
                && let Some((seq, jpeg)) = camera.frame_if_new(video_seq)
            {
                alive = socket.send_binary(&jpeg).is_ok();
                video_seq = seq;
            }
        }
        if !alive {
            break;
        }
        // 阻塞等待客户端消息（页面每 100ms 发一次按键心跳）。
        match socket.next() {
            Some(websocket::Message::Text(text)) => {
                alive = handle_message(&text, &mut socket, &shared, id);
            }
            Some(websocket::Message::Binary(_)) => {}
            None => break,
        }
    }

    shared.teleop.lock().unwrap().release_keys(id);
    shared.clients.fetch_sub(1, Ordering::Relaxed);
    println!("[web] WebSocket #{id} 已断开");
}

/// 处理客户端文本消息；返回连接是否仍可写。
fn handle_message(text: &str, socket: &mut Websocket, shared: &Arc<Shared>, id: u64) -> bool {
    let Ok(message) = serde_json::from_str::<ClientMessage>(text) else {
        return true; // 忽略坏消息
    };
    match message {
        ClientMessage::Keys { keys } => {
            let keys = Keys::parse(&keys);
            shared
                .teleop
                .lock()
                .unwrap()
                .set_keys(keys, id, Instant::now());
            true
        }
        ClientMessage::Action { action } => {
            let Some(action) = Action::parse(&action) else {
                return send_json(socket, &json!({ "type": "error", "error": "未知动作" }));
            };
            let out = shared.teleop.lock().unwrap().action(action);
            apply(&*shared.target, out);
            true
        }
        ClientMessage::Ping => send_json(socket, &json!({ "type": "pong" })),
    }
}

/// 发一条 JSON 文本消息；返回发送是否成功。
fn send_json(socket: &mut Websocket, value: &Value) -> bool {
    match serde_json::to_string(value) {
        Ok(text) => socket.send_text(&text).is_ok(),
        Err(_) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::web::LinkSnapshot;
    use std::io::{Read, Write};
    use std::net::TcpStream;
    use std::sync::atomic::AtomicBool;

    /// 记录收到的链路调用，供断言。
    #[derive(Default)]
    struct FakeTarget {
        log: Mutex<Vec<String>>,
        rpm: Mutex<[i16; 2]>,
    }

    impl FakeTarget {
        fn saw(&self, prefix: &str) -> bool {
            self.log
                .lock()
                .unwrap()
                .iter()
                .any(|e| e.starts_with(prefix))
        }
    }

    impl DriveTarget for FakeTarget {
        fn set_speeds(&self, left: i16, right: i16) {
            self.log
                .lock()
                .unwrap()
                .push(format!("speeds {left} {right}"));
            *self.rpm.lock().unwrap() = [left, right];
        }
        fn coast(&self) {
            self.log.lock().unwrap().push("coast".to_string());
        }
        fn brake(&self) {
            self.log.lock().unwrap().push("brake".to_string());
        }
        fn init(&self) {
            self.log.lock().unwrap().push("init".to_string());
        }
        fn reset(&self) {
            self.log.lock().unwrap().push("reset".to_string());
        }
        fn snapshot(&self) -> LinkSnapshot {
            LinkSnapshot {
                link_ok: true,
                sys: Some("Running".to_string()),
                rpm: *self.rpm.lock().unwrap(),
                last_frame: "Status Running".to_string(),
                ..Default::default()
            }
        }
    }

    struct TestServer {
        addr: SocketAddr,
        running: Arc<AtomicBool>,
        target: Arc<FakeTarget>,
        accept: JoinHandle<()>,
        control: JoinHandle<()>,
    }

    impl TestServer {
        fn start(status_hz: f32, video_fps: f32) -> Self {
            let running = Arc::new(AtomicBool::new(true));
            let target = Arc::new(FakeTarget::default());
            let target_dyn: Arc<dyn DriveTarget> = target.clone();
            let server = Server::bind(
                WebConfig {
                    bind: "127.0.0.1".to_string(),
                    port: 0,
                    status_hz,
                    video_fps,
                },
                TeleopConfig::default(),
                target_dyn,
                None,
                Arc::clone(&running),
            )
            .expect("绑定端口失败");
            let addr = server.local_addr().unwrap();
            let control = server.spawn_control();
            let accept = thread::spawn(move || server.run());
            Self {
                addr,
                running,
                target,
                accept,
                control,
            }
        }

        fn stop(self) {
            self.running.store(false, Ordering::SeqCst);
            self.accept.join().unwrap();
            self.control.join().unwrap();
        }
    }

    /// 发一个 HTTP GET 请求并读回完整响应（Connection: close）。
    fn http_get(addr: SocketAddr, path: &str) -> String {
        let mut stream = TcpStream::connect(addr).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        write!(
            stream,
            "GET {path} HTTP/1.1\r\nHost: test\r\nConnection: close\r\n\r\n"
        )
        .unwrap();
        let mut response = String::new();
        stream.read_to_string(&mut response).unwrap();
        response
    }

    #[test]
    fn serves_index_and_status_over_http() {
        let server = TestServer::start(10.0, 10.0);
        let index = http_get(server.addr, "/");
        assert!(index.starts_with("HTTP/1.1 200 OK"), "{index}");
        assert!(index.contains("Content-Type: text/html"));
        assert!(index.contains("<!DOCTYPE html>"));

        let status = http_get(server.addr, "/api/status");
        assert!(status.starts_with("HTTP/1.1 200 OK"), "{status}");
        assert!(status.contains("\"type\":\"status\""), "{status}");
        assert!(status.contains("\"sys\":\"Running\""), "{status}");
        assert!(status.contains("\"link\":true"), "{status}");

        assert!(http_get(server.addr, "/nope").starts_with("HTTP/1.1 404"));
        server.stop();
    }

    #[test]
    fn http_input_and_action_endpoints() {
        let server = TestServer::start(20.0, 10.0);
        let resp = http_get(server.addr, "/api/input?keys=w");
        assert!(resp.contains("\"ok\":true"), "{resp}");
        let resp = http_get(server.addr, "/api/action?action=init");
        assert!(resp.contains("\"action\":\"init\""), "{resp}");
        assert!(server.target.saw("init"));
        let resp = http_get(server.addr, "/api/action?action=bogus");
        assert!(resp.starts_with("HTTP/1.1 400"), "{resp}");
        server.stop();
    }

    /// WebSocket 客户端（tungstenite 只在测试里用）。
    fn ws_connect(
        addr: SocketAddr,
    ) -> tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>> {
        let (socket, _) = tungstenite::connect(format!("ws://{addr}/api/events")).unwrap();
        if let tungstenite::stream::MaybeTlsStream::Plain(stream) = socket.get_ref() {
            // 短超时：读不到就是“暂无推送”，由测试循环决定何时放弃。
            stream
                .set_read_timeout(Some(Duration::from_millis(300)))
                .unwrap();
        }
        socket
    }

    /// 读一条文本消息；超时/非文本返回 None。
    fn ws_read(
        socket: &mut tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>>,
    ) -> Option<Value> {
        match socket.read() {
            Ok(tungstenite::Message::Text(text)) => {
                Some(serde_json::from_str(text.as_str()).unwrap())
            }
            Ok(_) => None,
            Err(tungstenite::Error::Io(e))
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                None
            }
            Err(e) => panic!("读 WebSocket 失败: {e}"),
        }
    }

    /// 像页面那样反复发按键心跳并读状态，直到 `pred` 满足或超时。
    fn drive_until(
        socket: &mut tungstenite::WebSocket<tungstenite::stream::MaybeTlsStream<TcpStream>>,
        keys: &str,
        timeout: Duration,
        mut pred: impl FnMut(&Value) -> bool,
    ) -> Option<Value> {
        let deadline = Instant::now() + timeout;
        let mut last = None;
        while Instant::now() < deadline {
            socket
                .send(tungstenite::Message::Text(
                    format!("{{\"type\":\"keys\",\"keys\":\"{keys}\"}}").into(),
                ))
                .unwrap();
            if let Some(status) = ws_read(socket) {
                if pred(&status) {
                    return Some(status);
                }
                last = Some(status);
            }
        }
        last
    }

    #[test]
    fn websocket_drives_and_reports() {
        let server = TestServer::start(20.0, 10.0);
        let mut socket = ws_connect(server.addr);

        // 第一条应是 hello（服务端连上就发）。
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut hello = None;
        while hello.is_none() && Instant::now() < deadline {
            hello = ws_read(&mut socket);
        }
        let hello = hello.expect("没有 hello");
        assert_eq!(hello["type"], "hello", "{hello}");
        assert_eq!(hello["control_hz"], 20.0, "{hello}");

        // 页面每 100ms 发按键心跳；服务端处理后推送状态。
        let status = drive_until(&mut socket, "w", Duration::from_secs(3), |v| {
            v["drive"]["cmd"][0].as_i64().unwrap_or(0) > 0
        })
        .expect("没有状态推送");
        assert!(
            status["drive"]["cmd"][0].as_i64().unwrap_or(0) > 0,
            "没有收到正向轮命令: {status}"
        );
        assert!(server.target.saw("speeds"), "{:?}", server.target.log);

        // 制动动作由会话线程直接落到链路。
        socket
            .send(tungstenite::Message::Text(
                "{\"type\":\"action\",\"action\":\"brake\"}".into(),
            ))
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !server.target.saw("brake") && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(20));
        }
        assert!(server.target.saw("brake"), "{:?}", server.target.log);
        server.stop();
    }

    #[test]
    fn teleop_stops_when_client_disconnects() {
        let server = TestServer::start(20.0, 10.0);
        let mut socket = ws_connect(server.addr);
        assert!(drive_until(&mut socket, "w", Duration::from_secs(2), |_| false).is_some());
        assert!(server.target.saw("speeds"));

        drop(socket); // 断开 -> 服务端 release_keys -> 控制线程滑行
        let deadline = Instant::now() + Duration::from_secs(3);
        while !server.target.saw("coast") && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(20));
        }
        assert!(server.target.saw("coast"), "{:?}", server.target.log);
        server.stop();
    }
}
