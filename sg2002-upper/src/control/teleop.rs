//! 手动遥控控制律：按键快照 -> 左右轮速度（`esp32c3-smart-car/tools/play.py`
//! 的等价实现）。
//!
//! 驾驶手感与 play.py 一致：
//!
//! - W/S 把油门斜坡推向 `max_speed` / `-max_reverse`；反向时先用 `brake_rate`
//!   刹停再反向，松开所有油门键按 `decay_rate` 衰减到 0；
//! - A/D 把转向以 `steer_rate`（每秒）推向 ±1，正 = 左转；
//! - 油门绝对值小于 [`PIVOT_THRESHOLD`] 时两轮反向旋转（原地转向），否则按
//!   `turn_gain × |油门| / max_speed` 做差速；
//! - 超过 [`TeleopConfig::input_timeout`] 没有收到按键快照（浏览器卡死/断网）
//!   就当作全部松开，让车自己停下来；
//! - 制动/复位动作会把输出锁存（`Latch`），直到有新按键或 Init。
//!
//! 本模块只做决策，不碰链路：输出 [`Output`] 由 [`crate::control::session`]
//! 落地成 [`Car`](crate::control::Car) 调用，因此可以在主机上
//! 单独测试。

use std::fmt;
use std::time::{Duration, Instant};

/// 油门低于该值时转向只做原地旋转（与 play.py 的 `PIVOT_THRESHOLD` 一致）。
pub const PIVOT_THRESHOLD: f32 = 2.0;
/// 判断“还在前进/后退”的阈值（与 play.py 的 `EPS` 一致）。
const EPS: f32 = 0.01;

/// 网页按键快照：W/S 油门、A/D 转向。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Keys {
    pub w: bool,
    pub s: bool,
    pub a: bool,
    pub d: bool,
}

impl Keys {
    /// 全松开。
    pub const NONE: Self = Self {
        w: false,
        s: false,
        a: false,
        d: false,
    };

    /// 解析 `"wasd"` 形式的字符串；大小写均可，未知字符忽略。
    pub fn parse(text: &str) -> Self {
        let mut keys = Self::NONE;
        for c in text.chars() {
            match c.to_ascii_lowercase() {
                'w' => keys.w = true,
                's' => keys.s = true,
                'a' => keys.a = true,
                'd' => keys.d = true,
                _ => {}
            }
        }
        keys
    }

    /// 是否有任意键按下。
    pub fn any(self) -> bool {
        self.w || self.s || self.a || self.d
    }

    /// 转向目标：+1 左（A），-1 右（D）。
    pub fn steering(self) -> f32 {
        f32::from(self.a as u8) - f32::from(self.d as u8)
    }
}

impl fmt::Display for Keys {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (on, c) in [(self.w, 'w'), (self.a, 'a'), (self.s, 's'), (self.d, 'd')] {
            if on {
                write!(f, "{c}")?;
            }
        }
        Ok(())
    }
}

/// 控制律参数（默认值与 play.py 的 `tools/config.py` 一致）。
#[derive(Debug, Clone, Copy)]
pub struct TeleopConfig {
    /// 前进油门上限。
    pub max_speed: f32,
    /// 倒车油门上限。
    pub max_reverse: f32,
    /// W/S 斜坡加速度（油门单位/秒）。
    pub accel_rate: f32,
    /// 油门穿越零点（刹车+反向）时的斜率。
    pub brake_rate: f32,
    /// 松开油门键后的衰减斜率。
    pub decay_rate: f32,
    /// 满油门满转向时的差速大小。
    pub turn_gain: f32,
    /// 转向每秒变化量（0..1 尺度）。
    pub steer_rate: f32,
    /// 原地旋转时的轮速。
    pub pivot_speed: f32,
    /// 控制节拍（Hz）。
    pub control_hz: f32,
    /// 按键快照超时：超过该时间没有输入就松开所有键。
    pub input_timeout: Duration,
    /// 反转转向（对应 play.py 的 `invert_steer`）。
    pub invert_steer: bool,
}

impl Default for TeleopConfig {
    fn default() -> Self {
        Self {
            max_speed: 55.0,
            max_reverse: 40.0,
            accel_rate: 90.0,
            brake_rate: 350.0,
            decay_rate: 50.0,
            turn_gain: 40.0,
            steer_rate: 6.0,
            pivot_speed: 40.0,
            control_hz: 20.0,
            input_timeout: Duration::from_millis(600),
            invert_steer: false,
        }
    }
}

/// 控制律输出：由调用方落到链路上。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Output {
    /// 双轮目标速度（-100..100）。
    Drive { left: i16, right: i16 },
    /// 滑行（H 桥断开）。
    Coast,
    /// 短接制动；调用方应锁存到下一次按键。
    Brake,
    /// 不改变链路（锁存期间）。
    Hold,
    /// 发送 Init（编码器清零，进入 Ready）。
    Init,
    /// 发送 Reset（调试用，回 Uninit）。
    Reset,
}

impl Output {
    /// 双轮速度（`Hold`/`Coast`/`Brake`/`Init`/`Reset` 没有速度）。
    pub fn wheels(self) -> Option<(i16, i16)> {
        match self {
            Output::Drive { left, right } => Some((left, right)),
            _ => None,
        }
    }
}

/// 网页动作按钮。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// 立即滑行停车。
    Stop,
    /// 短接制动并锁存。
    Brake,
    /// 发送 Init。
    Init,
    /// 发送 Reset（调试用）。
    Reset,
}

impl Action {
    /// 解析动作名（与 JSON API 一致）。
    pub fn parse(name: &str) -> Option<Self> {
        match name.trim().to_ascii_lowercase().as_str() {
            "stop" | "coast" => Some(Self::Stop),
            "brake" => Some(Self::Brake),
            "init" => Some(Self::Init),
            "reset" => Some(Self::Reset),
            _ => None,
        }
    }
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Stop => "stop",
            Self::Brake => "brake",
            Self::Init => "init",
            Self::Reset => "reset",
        })
    }
}

/// HUD 标志位。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Flags {
    /// 低速原地转向。
    pub pivot: bool,
    /// 正在倒车。
    pub reverse: bool,
    /// 正在刹车（S 而车还在前进）。
    pub braking: bool,
}

/// 一帧 HUD 数据（供网页展示，不参与控制）。
#[derive(Debug, Clone, Copy)]
pub struct Hud {
    pub throttle: f32,
    pub steer: f32,
    pub flags: Flags,
    /// 最近一次下发的左右轮命令。
    pub cmd: [i16; 2],
    pub keys: Keys,
}

/// 锁存状态：动作按钮按下后，节拍不再改链路，直到有新按键或 Init。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Latch {
    /// 短接制动保持。
    Brake,
    /// 完全静默（Reset 后固件在 Uninit，发速度会被 NACK）。
    Silent,
}

/// 有状态的遥控器。
pub struct Teleop {
    cfg: TeleopConfig,
    keys: Keys,
    keys_at: Option<Instant>,
    /// 最近一次按键快照来自哪个客户端（用于断线时只清理自己的输入）。
    key_owner: u64,
    throttle: f32,
    steer: f32,
    last_tick: Option<Instant>,
    latch: Option<Latch>,
    last_cmd: Option<Output>,
}

impl Teleop {
    pub fn new(cfg: TeleopConfig) -> Self {
        Self {
            cfg,
            keys: Keys::NONE,
            keys_at: None,
            key_owner: 0,
            throttle: 0.0,
            steer: 0.0,
            last_tick: None,
            latch: None,
            last_cmd: None,
        }
    }

    pub fn config(&self) -> &TeleopConfig {
        &self.cfg
    }

    /// 更新按键快照（浏览器每次变化与心跳都会发一份完整快照）。
    ///
    /// 有新键按下时清除锁存，让车重新响应；空快照只当作心跳。
    pub fn set_keys(&mut self, keys: Keys, owner: u64, now: Instant) {
        self.keys = keys;
        self.keys_at = Some(now);
        self.key_owner = owner;
        if keys.any() {
            self.latch = None;
        }
    }

    /// 客户端断线：只清理它自己的按键，不打断另一个客户端的操作。
    pub fn release_keys(&mut self, owner: u64) {
        if self.key_owner == owner {
            self.keys = Keys::NONE;
            self.keys_at = None;
        }
    }

    /// 清空输入状态（手动/自动模式切换用）：松开所有键、解除锁存、油门与转向归零。
    pub fn reset_inputs(&mut self) {
        self.keys = Keys::NONE;
        self.keys_at = None;
        self.key_owner = 0;
        self.throttle = 0.0;
        self.steer = 0.0;
        self.latch = None;
        self.last_cmd = None;
    }

    /// 动作按钮：立即改变输出并锁存（Stop/Init 除外）。
    pub fn action(&mut self, action: Action) -> Output {
        self.keys = Keys::NONE;
        self.keys_at = None;
        self.throttle = 0.0;
        self.steer = 0.0;
        let out = match action {
            Action::Stop => {
                self.latch = None;
                Output::Coast
            }
            Action::Brake => {
                self.latch = Some(Latch::Brake);
                Output::Brake
            }
            Action::Init => {
                self.latch = None;
                Output::Init
            }
            // Reset 后固件在 Uninit，任何速度/Stop 帧都会被 NACK，保持静默。
            Action::Reset => {
                self.latch = Some(Latch::Silent);
                Output::Reset
            }
        };
        self.last_cmd = Some(out);
        out
    }

    /// 推进一步控制律；调用方按 `control_hz` 周期调用并落地输出。
    pub fn tick(&mut self, now: Instant) -> Output {
        let dt = match self.last_tick {
            Some(last) => now.saturating_duration_since(last).as_secs_f32().min(0.25),
            None => 0.0,
        };
        self.last_tick = Some(now);

        // 输入看门狗：断网/标签页后台化后没有快照 -> 松开所有键。
        if let Some(at) = self.keys_at
            && now.saturating_duration_since(at) > self.cfg.input_timeout
        {
            self.keys = Keys::NONE;
        }

        // 油门斜坡：目标由按键决定，斜率区分加速 / 刹车 / 衰减。
        let (goal, rate) = if self.keys.w {
            let rate = if self.throttle < 0.0 {
                self.cfg.brake_rate
            } else {
                self.cfg.accel_rate
            };
            (self.cfg.max_speed, rate)
        } else if self.keys.s {
            let rate = if self.throttle > 0.0 {
                self.cfg.brake_rate
            } else {
                self.cfg.accel_rate
            };
            (-self.cfg.max_reverse, rate)
        } else {
            (0.0, self.cfg.decay_rate)
        };
        self.throttle = move_toward(self.throttle, goal, rate * dt);

        // 转向斜坡；+1 左（A）/ -1 右（D）。
        let goal = if self.cfg.invert_steer {
            -self.keys.steering()
        } else {
            self.keys.steering()
        };
        self.steer = move_toward(self.steer, goal, self.cfg.steer_rate * dt);

        let out = if self.latch.is_some() {
            Output::Hold
        } else if self.throttle.abs() < EPS && self.steer.abs() < EPS {
            // 固件对 0 目标也是滑行；直接发 Coast 可少发一类帧。
            Output::Coast
        } else {
            let [left, right] = mix(&self.cfg, self.throttle, self.steer);
            Output::Drive { left, right }
        };
        self.last_cmd = Some(out);
        out
    }

    /// HUD 快照。
    pub fn hud(&self) -> Hud {
        let cmd = match self.last_cmd {
            Some(Output::Drive { left, right }) => [left, right],
            _ => [0, 0],
        };
        let braking = self.keys.s && self.throttle > EPS;
        Hud {
            throttle: self.throttle,
            steer: self.steer,
            flags: Flags {
                pivot: self.throttle.abs() <= PIVOT_THRESHOLD && self.steer.abs() > 0.05,
                reverse: self.throttle < -EPS,
                braking,
            },
            cmd,
            keys: self.keys,
        }
    }

    /// 当前按键快照（网页重连后比对用）。
    pub fn keys(&self) -> Keys {
        self.keys
    }

    /// 最近一次按键快照距今的时间。
    pub fn input_age(&self, now: Instant) -> Option<Duration> {
        self.keys_at.map(|at| now.saturating_duration_since(at))
    }

    /// 锁存期间是否不发速度（Reset 后的静默）。
    pub fn latched(&self) -> bool {
        self.latch.is_some()
    }
}

/// 把 `current` 朝 `target` 移动至多 `step`（play.py 的 `move_toward`）。
fn move_toward(current: f32, target: f32, step: f32) -> f32 {
    if current < target {
        (current + step).min(target)
    } else {
        (current - step).max(target)
    }
}

/// 当前油门/转向对应的双轮命令（play.py 的 `mix`）。
///
/// 油门低于 [`PIVOT_THRESHOLD`] 时两轮反向旋转；`steer` 正 = 左转。
pub fn mix(cfg: &TeleopConfig, throttle: f32, steer: f32) -> [i16; 2] {
    let (base, turn) = if throttle.abs() <= PIVOT_THRESHOLD {
        (0.0, cfg.pivot_speed)
    } else if cfg.max_speed != 0.0 {
        (throttle, cfg.turn_gain * throttle.abs() / cfg.max_speed)
    } else {
        (throttle, 0.0)
    };
    let left = base - steer * turn;
    let right = base + steer * turn;
    [clamp_command(left), clamp_command(right)]
}

fn clamp_command(v: f32) -> i16 {
    v.round().clamp(-100.0, 100.0) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> TeleopConfig {
        TeleopConfig::default()
    }

    /// 推进若干秒；`refresh` = 像浏览器那样每步重发当前按键快照（100ms 心跳）。
    fn advance(teleop: &mut Teleop, now: &mut Instant, secs: f32, refresh: bool) -> Output {
        let hz = teleop.config().control_hz;
        let steps = (secs * hz).round() as usize;
        let mut out = Output::Coast;
        for _ in 0..steps {
            *now += Duration::from_secs_f32(1.0 / hz);
            let keys = teleop.keys();
            if refresh && keys.any() {
                teleop.set_keys(keys, 1, *now);
            }
            out = teleop.tick(*now);
        }
        out
    }

    /// 保持按键并按心跳推进。
    fn run_held(teleop: &mut Teleop, now: &mut Instant, secs: f32) -> Output {
        advance(teleop, now, secs, true)
    }

    /// 不刷新输入（模拟浏览器断流）。
    fn run_silent(teleop: &mut Teleop, now: &mut Instant, secs: f32) -> Output {
        advance(teleop, now, secs, false)
    }

    #[test]
    fn keys_parse_and_display() {
        assert_eq!(
            Keys::parse("WDx"),
            Keys {
                w: true,
                s: false,
                a: false,
                d: true
            }
        );
        assert_eq!(Keys::parse("wasd").to_string(), "wasd");
        assert_eq!(Keys::parse("AS").to_string(), "as");
        assert!(!Keys::NONE.any());
        assert_eq!(Keys::parse("a").steering(), 1.0);
        assert_eq!(Keys::parse("d").steering(), -1.0);
    }

    #[test]
    fn w_accelerates_to_max_speed() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        // 1 秒 * 90/s = 90 -> 封顶 55
        let out = run_held(&mut teleop, &mut now, 1.0);
        assert_eq!(
            out,
            Output::Drive {
                left: 55,
                right: 55
            }
        );
        assert_eq!(teleop.hud().throttle, 55.0);
    }

    #[test]
    fn s_brakes_then_reverses() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.5);
        let forward = teleop.hud().throttle;
        assert!(forward > 30.0 && forward <= 55.0, "油门 {forward}");

        // 换 S：油门按 brake_rate 下降，穿过 0 后变成倒车。
        teleop.set_keys(Keys::parse("s"), 1, now);
        let out = run_held(&mut teleop, &mut now, 1.0);
        assert_eq!(
            out,
            Output::Drive {
                left: -40,
                right: -40
            }
        );
        assert!(teleop.hud().flags.reverse);
    }

    #[test]
    fn release_decays_to_coast() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.4);
        assert!(teleop.hud().throttle > 20.0);

        teleop.set_keys(Keys::NONE, 1, now);
        let out = run_silent(&mut teleop, &mut now, 2.0);
        assert_eq!(out, Output::Coast);
        assert_eq!(teleop.hud().throttle, 0.0);
    }

    #[test]
    fn steering_at_standstill_pivots_in_place() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("a"), 1, now);
        // steer_rate 6/s：0.2s 后到 1.0
        let out = run_held(&mut teleop, &mut now, 0.3);
        assert_eq!(
            out,
            Output::Drive {
                left: -40,
                right: 40
            }
        );
        assert!(teleop.hud().flags.pivot);
    }

    #[test]
    fn steering_right_makes_left_wheel_faster() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("wd"), 1, now);
        let out = run_held(&mut teleop, &mut now, 1.0);
        let Output::Drive { left, right } = out else {
            panic!("应当差速行驶");
        };
        assert!(left > right, "D 键右转应让左轮更快: {left} vs {right}");
    }

    #[test]
    fn stale_input_releases_keys() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.5);
        assert!(teleop.hud().throttle > 20.0);

        // 静默 2.5s（超过 600ms 超时）后，按键应被释放并衰减停车。
        let out = run_silent(&mut teleop, &mut now, 2.5);
        assert_eq!(out, Output::Coast);
        assert_eq!(teleop.keys(), Keys::NONE);
    }

    #[test]
    fn brake_action_latches_until_key() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.5);

        assert_eq!(teleop.action(Action::Brake), Output::Brake);
        // 锁存期间节拍不下发速度
        assert_eq!(run_silent(&mut teleop, &mut now, 1.0), Output::Hold);
        // 新按键解除锁存
        teleop.set_keys(Keys::parse("w"), 1, now);
        assert!(matches!(
            run_held(&mut teleop, &mut now, 0.2),
            Output::Drive { .. }
        ));
    }

    #[test]
    fn stop_action_coasts_and_clears_ramp() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.5);

        assert_eq!(teleop.action(Action::Stop), Output::Coast);
        assert_eq!(teleop.hud().throttle, 0.0);
        assert_eq!(run_silent(&mut teleop, &mut now, 0.5), Output::Coast);
    }

    #[test]
    fn reset_action_keeps_link_silent() {
        let mut teleop = Teleop::new(cfg());
        let mut now = Instant::now();
        assert_eq!(teleop.action(Action::Reset), Output::Reset);
        assert_eq!(run_silent(&mut teleop, &mut now, 0.5), Output::Hold);
        // Init 后恢复
        assert_eq!(teleop.action(Action::Init), Output::Init);
        assert_eq!(run_silent(&mut teleop, &mut now, 0.1), Output::Coast);
    }

    #[test]
    fn reset_inputs_clears_keys_and_latch() {
        let mut teleop = Teleop::new(TeleopConfig::default());
        let mut now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 1, now);
        run_held(&mut teleop, &mut now, 0.5);
        assert_eq!(teleop.action(Action::Brake), Output::Brake);
        assert!(teleop.latched());

        teleop.reset_inputs();
        assert!(!teleop.latched());
        assert!(!teleop.keys().any());
        assert_eq!(teleop.tick(now), Output::Coast);
    }

    #[test]
    fn release_keys_only_clears_matching_owner() {
        let mut teleop = Teleop::new(cfg());
        let now = Instant::now();
        teleop.set_keys(Keys::parse("w"), 7, now);
        teleop.release_keys(3);
        assert_eq!(teleop.keys(), Keys::parse("w"));
        teleop.release_keys(7);
        assert_eq!(teleop.keys(), Keys::NONE);
    }

    #[test]
    fn mix_uses_pivot_below_threshold() {
        let cfg = cfg();
        assert_eq!(mix(&cfg, 0.0, 1.0), [-40, 40]);
        assert_eq!(mix(&cfg, 1.9, -1.0), [40, -40]);
        // 超过阈值后按差速：满油门满转向 = 55 ∓ 40
        assert_eq!(mix(&cfg, 55.0, 1.0), [15, 95]);
        assert_eq!(mix(&cfg, 55.0, 0.0), [55, 55]);
    }
}
