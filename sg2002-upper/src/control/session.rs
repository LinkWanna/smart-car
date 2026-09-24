//! 控制会话：控制节拍（手动遥控 / 自动视觉伺服）+ 输入模式仲裁。
//!
//! 从 web 层独立出来：网页只负责把按键/动作/模式转成方法调用，控制线程与
//! 仲裁逻辑都在这里（[`ControlSession::spawn`] 启动）。
//!
//! 线程模型：控制线程按 `control_hz` 推进 —— 手动模式跑 [`Teleop::tick`]，
//! 自动模式取 [`PreviewSource::latest_detections`]（超过 `AUTO_STALE` 视为
//! 看不到，滑行），在本地做位置分析（[`PositionAnalyzer`]）后推进视觉伺服；
//! 输出统一经 `apply` 落到 [`Car`]。
//! 模式切换时重置伺服状态并清空手动输入（避免残留油门/锁存），退出前滑行。

use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::info;

use crate::vision::preview::{DetectionFrame, PreviewSource};
use crate::transport::car::{Car, LinkState};
use crate::transport::protocol::{ErrorCode, RequestType, Response, SysState};

use super::position::{Observation, PositionAnalyzer};
use super::servo::{Action as ServoAction, ControlConfig, ControlLoop};
use super::teleop::{Action, Hud, Keys, Output, Teleop, TeleopConfig};

/// 观测过期阈值：超过这个时间没有新帧，自动模式按“看不到”处理（滑行）。
const AUTO_STALE: Duration = Duration::from_millis(500);

/// 输入来源：手动遥控（网页按键）或自动视觉（追踪目标）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Manual,
    Auto,
}

impl Mode {
    /// 解析模式名（`manual`/`auto`，也接受中文）。
    pub fn parse(name: &str) -> Option<Self> {
        match name.trim().to_ascii_lowercase().as_str() {
            "manual" | "手动" => Some(Self::Manual),
            "auto" | "自动" => Some(Self::Auto),
            _ => None,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Manual => "manual",
            Self::Auto => "auto",
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// 自动模式的最近一次输出（HUD 展示用）。
#[derive(Debug, Clone)]
pub struct AutoState {
    pub action: String,
    pub cmd: [i16; 2],
}

/// 自动补 `Init` 的限频（应用策略：下位机掉回 Uninit 时不要猛刷）。
const REINIT_PERIOD: Duration = Duration::from_secs(1);
/// 观察窗口：`WrongState` NACK 在这段时间内才算“当前状态不对”。
const WRONG_STATE_WINDOW: Duration = Duration::from_secs(2);

/// 控制层状态快照（网页 JSON 用；一次取全，避免撕裂读）。
pub struct ControlStatus {
    pub mode: Mode,
    /// 自动模式是否可用：有视觉 + 模型可用 + 画面新鲜。
    pub auto_ready: bool,
    pub hud: Hud,
    /// 最近一次按键快照距今的时间。
    pub input_age: Option<Duration>,
    /// 自动模式最近一次输出（手动模式为 `None`）。
    pub auto: Option<AutoState>,
    pub link: LinkState,
}

/// 控制会话：持有链路、手动控制律、模式与视觉伺服。
///
/// 自带停止标志与线程句柄（Car 风格）：[`spawn`](ControlSession::spawn) 启动
/// 控制线程，[`stop`](ControlSession::stop) 置 false 并等它退出；只停自己。
pub struct ControlSession {
    target: Arc<Car>,
    teleop: Mutex<Teleop>,
    mode: Mutex<Mode>,
    /// 自动模式控制律参数。
    servo_cfg: ControlConfig,
    /// 自动模式的位置分析（检测框 → 分区/距离；追踪侧语义，与 vision 解耦）。
    analyzer: PositionAnalyzer,
    auto: Mutex<Option<AutoState>>,
    /// 自动模式的数据源；`None` = 未启用视觉。
    vision: Option<Arc<dyn PreviewSource>>,
    /// 上次请求 `Init` 的时刻（自动补 `Init` 的限频；只有控制线程写）。
    reinit_at: Mutex<Instant>,
    running: AtomicBool,
    handle: Mutex<Option<JoinHandle<()>>>,
}

impl ControlSession {
    /// 创建会话（不启动线程；见 [`ControlSession::spawn`]）。
    pub fn new(
        target: Arc<Car>,
        teleop_cfg: TeleopConfig,
        vision: Option<Arc<dyn PreviewSource>>,
    ) -> Arc<Self> {
        Arc::new(Self {
            target,
            teleop: Mutex::new(Teleop::new(teleop_cfg)),
            mode: Mutex::new(Mode::Manual),
            servo_cfg: ControlConfig::default(),
            analyzer: PositionAnalyzer::for_640x480(),
            auto: Mutex::new(None),
            vision,
            reinit_at: Mutex::new(Instant::now()),
            running: AtomicBool::new(true),
            handle: Mutex::new(None),
        })
    }

    /// 启动控制线程（重复调用只保留第一个）。
    pub fn spawn(self: &Arc<Self>) {
        let session = Arc::clone(self);
        let handle = thread::spawn(move || session.control_loop());
        *self.handle.lock().unwrap() = Some(handle);
    }

    /// 停止控制线程并等它退出（幂等）；只停自己，不影响其它组件。
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.handle.lock().unwrap().take() {
            crate::join_with_timeout(handle, "控制");
        }
    }

    pub fn mode(&self) -> Mode {
        *self.mode.lock().unwrap()
    }

    /// 自动模式是否可用（视觉/模型就绪 + 画面新鲜）。
    pub fn auto_ready(&self) -> bool {
        self.vision.as_ref().is_some_and(|p| p.auto_ready())
    }

    /// 切换输入模式；进入自动模式要求视觉/模型就绪。
    ///
    /// 切换时立即滑行并把手动输入清干净（避免残留油门/锁存），控制线程会在
    /// 下一拍发现模式变化并重置视觉伺服状态。
    pub fn set_mode(&self, mode: Mode) -> Result<(), String> {
        if mode == Mode::Auto && !self.auto_ready() {
            return Err(match &self.vision {
                Some(_) => "视觉未就绪（模型/画面不可用），无法进入自动模式".into(),
                None => "未启用视觉，无法进入自动模式".into(),
            });
        }
        *self.mode.lock().unwrap() = mode;
        self.teleop.lock().unwrap().reset_inputs();
        self.target.coast();
        info!("输入模式 → {}", mode);
        Ok(())
    }

    /// 更新按键快照（浏览器每次变化与心跳都会发一份完整快照）。
    pub fn set_keys(&self, keys: Keys, client: u64, now: Instant) {
        self.teleop.lock().unwrap().set_keys(keys, client, now);
    }

    /// 客户端断线：只清理它自己的按键。
    pub fn release_keys(&self, client: u64) {
        self.teleop.lock().unwrap().release_keys(client);
    }

    /// 动作按钮：立即落地输出（Stop 在自动模式下先切回手动，避免伺服接管）。
    pub fn action(&self, action: Action) -> Output {
        if action == Action::Stop && self.mode() == Mode::Auto {
            self.set_mode(Mode::Manual).ok();
        }
        let out = self.teleop.lock().unwrap().action(action);
        apply(&self.target, out);
        out
    }

    /// 手动遥控参数（网页 hello 消息展示用）。
    pub fn teleop_config(&self) -> TeleopConfig {
        *self.teleop.lock().unwrap().config()
    }

    /// 一帧状态快照（网页 `status`/`hello` 共用）。
    pub fn status(&self) -> ControlStatus {
        let now = Instant::now();
        let mode = self.mode();
        let (hud, input_age) = {
            let teleop = self.teleop.lock().unwrap();
            (teleop.hud(), teleop.input_age(now))
        };
        ControlStatus {
            mode,
            auto_ready: self.auto_ready(),
            hud,
            input_age,
            auto: (mode == Mode::Auto)
                .then(|| self.auto.lock().unwrap().clone())
                .flatten(),
            link: self.target.state(),
        }
    }

    /// 控制节拍：按当前模式推进手动遥控或视觉伺服，并落地输出。
    fn control_loop(self: Arc<Self>) {
        let hz = self.teleop.lock().unwrap().config().control_hz.max(1.0);
        let period = Duration::from_secs_f32(1.0 / hz);
        let mut next = Instant::now();
        let mut servo = ControlLoop::new(self.servo_cfg);
        let mut last_mode = self.mode();
        while self.running.load(Ordering::Relaxed) {
            let now = Instant::now();
            let mode = self.mode();
            if mode != last_mode {
                // 模式刚变化：重置伺服状态（不沿用上次的搜索方向/保持），
                // 手动输入也已在 set_mode 里清空。
                servo = ControlLoop::new(self.servo_cfg);
                last_mode = mode;
            }
            let out = match mode {
                Mode::Manual => self.teleop.lock().unwrap().tick(now),
                Mode::Auto => self.auto_output(&mut servo, now),
            };
            apply(&self.target, out);
            self.maybe_reinit();
            next += period;
            let now = Instant::now();
            if next > now {
                thread::sleep(next - now);
            } else {
                next = now;
            }
        }
        info!("控制线程退出，滑行停车");
        self.target.coast();
    }

    /// 自动模式：取最新检测帧（过期/不可用 → 滑行），做位置分析后推进视觉伺服。
    fn auto_output(&self, servo: &mut ControlLoop, now: Instant) -> Output {
        let frame: Option<Arc<DetectionFrame>> = self
            .vision
            .as_ref()
            .and_then(|preview| preview.latest_detections())
            .filter(|frame| now.saturating_duration_since(frame.at) < AUTO_STALE);
        let Some(frame) = frame else {
            *self.auto.lock().unwrap() = Some(AutoState {
                action: "等待视觉".to_string(),
                cmd: [0, 0],
            });
            return Output::Coast;
        };
        // 追踪语义：检测框 → 位置/距离分级 → 控制律输入
        let obs = Observation::from_result(&self.analyzer.analyze(&frame.dets));
        let action = servo.step(&obs, now);
        let cmd = action.wheels().map_or([0, 0], |(l, r)| [l, r]);
        *self.auto.lock().unwrap() = Some(AutoState {
            action: action.to_string(),
            cmd,
        });
        action_output(action)
    }

    /// 应用策略：下位机掉回 `Uninit`、或刚因状态不对拒过速度指令时，补发
    /// `Init`（限频 [`REINIT_PERIOD`]）。传输层只提供状态与 [`Car::init`]，
    /// 不做这个判断。
    fn maybe_reinit(&self) {
        let LinkState {
            status,
            last_response,
            ..
        } = self.target.state();
        let uninit = status.map(|s| s.sys) == Some(SysState::Uninit);
        let wrong_state = matches!(
            last_response,
            Some((Response::Nack { cmd, error }, at))
                if cmd == RequestType::SetSpeeds.as_u8()
                    && error == ErrorCode::WrongState
                    && at.elapsed() < WRONG_STATE_WINDOW
        );
        if !uninit && !wrong_state {
            return;
        }
        let mut last = self.reinit_at.lock().unwrap();
        if last.elapsed() < REINIT_PERIOD {
            return;
        }
        *last = Instant::now();
        drop(last);
        self.target.init();
    }
}

/// 把控制律输出落到链路。
fn apply(target: &Car, out: Output) {
    // 应用每拍都在说话：刷新死手开关（`Output::Hold` 不改意图，但证明控制线程活着）
    target.refresh_intent();
    match out {
        Output::Drive { left, right } => target.set_desired(left, right),
        Output::Coast => target.coast(),
        Output::Brake => target.brake(),
        Output::Hold => {}
        Output::Init => target.init(),
        Output::Reset => target.reset(),
    }
}

/// 视觉伺服的决策 → 链路输出。
fn action_output(action: ServoAction) -> Output {
    match action {
        ServoAction::Brake => Output::Brake,
        ServoAction::Track { .. } | ServoAction::Search { .. } => action
            .wheels()
            .map_or(Output::Coast, |(left, right)| Output::Drive { left, right }),
    }
}
