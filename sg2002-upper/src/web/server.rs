//! 网页服务器：rouille（HTTP + WebSocket）+ serde_json 状态推送。
//!
//! 线程模型：
//!
//! - **HTTP 请求**：`rouille::Server` 用线程池处理，[`Server::run`] 每 200ms
//!   轮询 `running`（`poll_timeout`），退出前 `join` 等在途请求；
//! - **WebSocket 会话**：握手交给 `rouille::websocket`，每个连接一个线程——
//!   `Websocket::next()` 阻塞读客户端消息，收到消息后按 `status_hz` /
//!   `video_fps` 推送状态与 JPEG（页面每 100ms 发按键心跳，天然给出推送节奏）；
//!   客户端静默时输入看门狗（[`Teleop`](crate::control::Teleop)）会把车停下。
//!
//! 控制节拍与手动/自动仲裁在 [`crate::control::ControlSession`]（本模块只把
//! 按键/动作/模式转成会话方法调用，并从会话状态组装 JSON）。
//!
//! 页面本身（`assets/index.html`）通过 `include_str!` 内嵌，零外部资源。

use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use log::{info, warn};
use rouille::websocket::{self, Websocket};
use rouille::{Request, Response};
use serde::Deserialize;
use serde_json::{Value, json};

use crate::control::ControlSession;
use crate::control::session::{AutoState, Mode};
use crate::control::teleop::{Action, Keys};
use crate::preview::{DetectionFrame, PreviewSource};
use crate::vision::{FRAME_H, FRAME_W};

/// WebSocket 每轮主动推送的时长（略小于页面 100ms 的按键心跳间隔）。
const PUSH_WINDOW: Duration = Duration::from_millis(90);
/// 推送窗口内的检查粒度。
const PUSH_TICK: Duration = Duration::from_millis(5);

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
    /// JPEG 预览帧率上限（Hz）；`0` = 不锁帧（有新帧就推）。
    pub video_fps: f32,
}

impl Default for WebConfig {
    fn default() -> Self {
        Self {
            bind: "0.0.0.0".to_string(),
            port: 80,
            status_hz: 10.0,
            video_fps: 0.0,
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
    /// 输入模式切换（`manual`/`auto`，只能由按钮触发）。
    Mode { mode: String },
    /// 应用层 ping，回一条 pong。
    Ping,
}

/// 所有连接共享的状态。
struct Shared {
    cfg: WebConfig,
    /// 控制会话（链路 + 手动/自动仲裁 + 控制线程）。
    session: Arc<ControlSession>,
    /// 预览/视觉源；`None` = 未启用预览。
    preview: Option<Arc<dyn PreviewSource>>,
    running: Arc<AtomicBool>,
    clients: AtomicU64,
    next_client_id: AtomicU64,
    started: Instant,
}

impl Shared {
    /// 一帧状态（WebSocket 推送与 `GET /api/status` 共用）。
    fn status_value(&self) -> Value {
        let st = self.session.status();
        let mut flags: Vec<&str> = Vec::new();
        if st.hud.flags.pivot {
            flags.push("PIVOT");
        }
        if st.hud.flags.reverse {
            flags.push("REV");
        }
        if st.hud.flags.braking {
            flags.push("BRAKE");
        }
        json!({
            "type": "status",
            "uptime": self.started.elapsed().as_secs_f32(),
            "link": st.link.link_ok,
            "link_age_ms": st.link.link_age.map(|d| d.as_millis() as u64),
            "sys": st.link.sys,
            "rpm": st.link.rpm,
            "dist": { "active": st.link.dist_active, "result": st.link.dist_result },
            "drive": {
                "throttle": st.hud.throttle,
                "steer": st.hud.steer,
                "flags": flags,
                "cmd": st.hud.cmd,
                "keys": st.hud.keys.to_string(),
                "input_age_ms": st.input_age.map(|d| d.as_millis() as u64),
            },
            "mode": st.mode.as_str(),
            "auto_ready": st.auto_ready,
            "auto": auto_value(st.auto.as_ref()),
            "counters": {
                "acks": st.link.counters.acks,
                "nacks": st.link.counters.nacks,
                "checksum_fails": st.link.counters.checksum_fails,
                "write_errors": st.link.counters.write_errors,
                "watchdog_trips": st.link.counters.watchdog_trips,
            },
            "last_frame": st.link.last_frame,
            "camera": self.camera_value(),
            "vision": self.vision_value(),
            "clients": self.clients.load(Ordering::Relaxed),
        })
    }

    /// 连接建立时的问候消息（带页面需要的固定参数）。
    fn hello_value(&self) -> Value {
        let cfg = self.session.teleop_config();
        let st = self.session.status();
        json!({
            "type": "hello",
            "version": 1,
            "control_hz": cfg.control_hz,
            "status_hz": self.cfg.status_hz,
            "video_fps": self.cfg.video_fps,
            "input_timeout_ms": cfg.input_timeout.as_millis() as u64,
            "max_speed": cfg.max_speed,
            "max_reverse": cfg.max_reverse,
            "mode": st.mode.as_str(),
            "auto_ready": st.auto_ready,
            "camera": self.camera_value(),
        })
    }

    fn camera_value(&self) -> Value {
        let Some(preview) = &self.preview else {
            return Value::Null;
        };
        let st = preview.camera_status();
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

    /// 视觉状态（模型/推理/编码耗时）；纯画面源为 null。
    fn vision_value(&self) -> Value {
        let Some(st) = self.preview.as_ref().and_then(|p| p.vision_status()) else {
            return Value::Null;
        };
        json!({
            "model_ok": st.model_ok,
            "model": st.model,
            "input": st.input,
            "encode": st.encode,
            "sent": st.sent,
            "dropped": st.dropped,
            "published": st.published,
            "encode_avg_ms": st.encode_avg_ms,
            "fps": st.fps,
            "age_ms": st.age_ms,
            "infer_ms": st.infer_ms,
            "nms_ms": st.nms_ms,
            "encode_ms": st.encode_ms,
            "dets": st.dets,
            "error": st.error,
        })
    }
}

/// 网页服务器句柄。
///
/// 自带停止标志：信号处理器通过 [`Server::stop_flag`] 置 false 后
/// [`Server::run`] 返回（只停监听循环，其它组件由调用方按序停）。
pub struct Server {
    inner: rouille::Server<Handler>,
    running: Arc<AtomicBool>,
}

type Handler = Box<dyn Fn(&Request) -> Response + Send + Sync>;

impl Server {
    /// 绑定监听地址。
    ///
    /// 控制线程由调用方用 [`ControlSession::spawn`] 启动/join。
    pub fn bind(
        cfg: WebConfig,
        session: Arc<ControlSession>,
        preview: Option<Arc<dyn PreviewSource>>,
    ) -> io::Result<Self> {
        let running = Arc::new(AtomicBool::new(true));
        let shared = Arc::new(Shared {
            cfg,
            session,
            preview,
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
        Ok(Self { inner, running })
    }

    /// 停止标志（信号处理器只做原子写，是 async-signal-safe 的）。
    pub fn stop_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.running)
    }

    /// 实际监听地址（测试里用 `port = 0` 拿随机端口）。
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        Ok(self.inner.server_addr())
    }

    /// 请求循环；`running` 为 false 时排空在途请求并返回。
    pub fn run(&self) {
        // rouille 没有公开的停止接口，这里用轮询 `running` 的方式收尾。
        // WebSocket 会话在独立线程里阻塞读，进程退出时一并结束。
        while self.running.load(Ordering::Relaxed) {
            self.inner.poll_timeout(POLL_INTERVAL);
        }
        self.inner.join();
        info!("已停止监听");
    }
}

/// HTTP 路由。
fn route(request: &Request, shared: &Arc<Shared>) -> Response {
    let url = request.url();
    let path = url.split('?').next().unwrap_or("/");
    match path {
        "/" | "/index.html" => Response::html(INDEX_HTML).with_no_cache(),
        "/api/status" => Response::json(&shared.status_value()).with_no_cache(),
        "/api/frame.jpg" => match shared.preview.as_ref().and_then(|p| p.latest()) {
            Some(frame) => match frame.jpeg.clone() {
                Some(jpeg) => Response::from_data("image/jpeg", jpeg.to_vec()).with_no_cache(),
                None => Response::text("预览不可用\n").with_status_code(503),
            },
            None => Response::text("预览不可用\n").with_status_code(503),
        },
        "/api/mode" => {
            let name = request.get_param("set").unwrap_or_default();
            let Some(mode) = Mode::parse(&name) else {
                return Response::text("用法: /api/mode?set=auto|manual\n").with_status_code(400);
            };
            match shared.session.set_mode(mode) {
                Ok(()) => Response::json(&json!({ "ok": true, "mode": mode.as_str() })),
                Err(e) => Response::text(format!("{e}\n")).with_status_code(409),
            }
        }
        "/api/input" => {
            let keys = Keys::parse(&request.get_param("keys").unwrap_or_default());
            shared.session.set_keys(keys, 0, Instant::now());
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
            shared.session.action(action);
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
        Err(_) => warn!("WebSocket 升级失败"),
    });
    response
}

/// 自动模式最近一次输出的 JSON（`None` → null）。
fn auto_value(auto: Option<&AutoState>) -> Value {
    match auto {
        Some(auto) => json!({ "action": auto.action, "cmd": auto.cmd }),
        None => Value::Null,
    }
}

/// 一帧预览对应的视觉消息（`seq` 与紧随其后的 JPEG 严格同帧）。
///
/// 只发检测框（归一化）与统计；分区/距离这类语义不在视觉侧，页面不依赖。
fn vision_frame_value(seq: u64, frame: &DetectionFrame) -> Value {
    let dets: Vec<Value> = frame
        .dets
        .iter()
        .map(|d| {
            json!({
                "x1": d.x1 / FRAME_W as f32,
                "y1": d.y1 / FRAME_H as f32,
                "x2": d.x2 / FRAME_W as f32,
                "y2": d.y2 / FRAME_H as f32,
                "conf": d.confidence,
            })
        })
        .collect();
    json!({
        "type": "vision",
        "seq": seq,
        "dets": dets,
    })
}

/// WebSocket 会话：阻塞读客户端消息，期间按节拍推送状态与预览帧。
///
/// 注意推送**不能只依赖客户端心跳**：页面的按键心跳是 100ms 一次，如果每收到
/// 一条消息才推一帧，预览就被锁在 10fps。所以每轮先跑一个 [`PUSH_WINDOW`]
/// 长的推送窗口（每 [`PUSH_TICK`] 检查一次有没有新帧），再阻塞等消息。
fn websocket_session(mut socket: Websocket, shared: Arc<Shared>) {
    let id = shared.next_client_id.fetch_add(1, Ordering::Relaxed) + 1;
    shared.clients.fetch_add(1, Ordering::Relaxed);
    info!("WebSocket #{id} 已连接");

    let status_period = Duration::from_secs_f32(1.0 / shared.cfg.status_hz.max(0.5));
    // `video_fps = 0` → 不锁帧：有新帧就推
    let video_period = if shared.cfg.video_fps > 0.0 {
        Duration::from_secs_f32(1.0 / shared.cfg.video_fps)
    } else {
        Duration::ZERO
    };
    let mut last_status = Instant::now()
        .checked_sub(status_period)
        .unwrap_or_else(Instant::now);
    let mut last_video = Instant::now()
        .checked_sub(video_period)
        .unwrap_or_else(Instant::now);
    let mut video_seq = 0u64;
    let mut alive = send_json(&mut socket, &shared.hello_value());

    while alive && shared.running.load(Ordering::Relaxed) {
        // 推送窗口：不等客户端心跳，把状态/新帧按节拍推出去。
        let until = Instant::now() + PUSH_WINDOW;
        while alive && Instant::now() < until {
            let now = Instant::now();
            if now.saturating_duration_since(last_status) >= status_period {
                alive = send_json(&mut socket, &shared.status_value());
                last_status = now;
            }
            if alive && now.saturating_duration_since(last_video) >= video_period {
                last_video = now;
                if let Some(preview) = &shared.preview
                    && let Some(frame) = preview.frame_if_new(video_seq)
                {
                    video_seq = frame.seq;
                    if let Some(jpeg) = &frame.jpeg {
                        alive = socket.send_binary(jpeg).is_ok();
                    }
                    // 紧跟同帧的检测结果：页面据此画覆盖框（与画面严格对齐）
                    if alive && let Some(dets) = &frame.dets {
                        alive = send_json(&mut socket, &vision_frame_value(frame.seq, dets));
                    }
                }
            }
            if !alive {
                break;
            }
            thread::sleep(PUSH_TICK);
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

    shared.session.release_keys(id);
    shared.clients.fetch_sub(1, Ordering::Relaxed);
    info!("WebSocket #{id} 已断开");
}

/// 处理客户端文本消息；返回连接是否仍可写。
fn handle_message(text: &str, socket: &mut Websocket, shared: &Arc<Shared>, id: u64) -> bool {
    let Ok(message) = serde_json::from_str::<ClientMessage>(text) else {
        return true; // 忽略坏消息
    };
    match message {
        ClientMessage::Keys { keys } => {
            let keys = Keys::parse(&keys);
            shared.session.set_keys(keys, id, Instant::now());
            true
        }
        ClientMessage::Action { action } => {
            let Some(action) = Action::parse(&action) else {
                return send_json(socket, &json!({ "type": "error", "error": "未知动作" }));
            };
            // 安全：自动模式下按“停止”先切回手动（会话内部处理），否则下一拍伺服会接管。
            shared.session.action(action);
            true
        }
        ClientMessage::Mode { mode } => {
            let Some(mode) = Mode::parse(&mode) else {
                return send_json(socket, &json!({ "type": "error", "error": "未知模式" }));
            };
            match shared.session.set_mode(mode) {
                Ok(()) => send_json(socket, &json!({ "type": "mode", "mode": mode.as_str() })),
                Err(e) => send_json(socket, &json!({ "type": "error", "error": e })),
            }
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
    use crate::yolo::Detection;

    /// 视觉消息只发检测框（归一化到 0..1）与帧号，不带任何高层语义。
    #[test]
    fn vision_message_normalizes_boxes() {
        let frame = DetectionFrame {
            seq: 3,
            at: Instant::now(),
            dets: vec![Detection {
                label: "tennis_ball".to_string(),
                confidence: 0.8,
                x1: 160.0,
                y1: 120.0,
                x2: 320.0,
                y2: 240.0,
            }],
        };
        let value = vision_frame_value(3, &frame);
        assert_eq!(value["type"], "vision");
        assert_eq!(value["seq"], 3);
        let d = &value["dets"][0];
        let f = |key: &str| d[key].as_f64().unwrap() as f32;
        assert!((f("x1") - 0.25).abs() < 1e-6);
        assert!((f("y1") - 0.25).abs() < 1e-6);
        assert!((f("x2") - 0.5).abs() < 1e-6);
        assert!((f("y2") - 0.5).abs() < 1e-6);
        assert!((f("conf") - 0.8).abs() < 1e-6);
    }
}
