//! 视觉伺服控制律：检测结果 -> 左右轮速度（ESP32-C3 `SetSpeeds`）。
//!
//! 与 `sg2002_inference` 的“离散命令字符串”（FORWARD/TURN_LEFT/...）不同，
//! 这里输出连续差速：
//!
//! ```text
//! 基础速度 base = far_speed / mid_speed        （near 直接刹车）
//! 差速     turn = turn_gain × err_x × base      （带中心死区与最小差速）
//! 左轮     left  = clamp(base + turn)
//! 右轮     right = clamp(base - turn)
//! ```
//!
//! 其中 `err_x = (中心x / 画面宽 - 0.5) × 2`，目标在右侧为正；右偏时左轮
//! 加速、右轮减速（`left > right`）实现右转，`err_x` 大到使内轮反转时自然
//! 变成原地转向。
//!
//! 目标丢失：先按 [`ControlConfig::lost_hold`] 保持上一动作（避免单帧漏检
//! 造成抖停），随后原地旋转搜索；搜索方向优先延续目标最后出现的方向，每
//! [`ControlConfig::search_flip`] 换一次向。
//!
//! 本模块只做决策，不碰串口：输出 [`Action`] 由调用方落地成小车指令，
//! 因此可以在主机上 `cargo test --lib` 单独验证。

use std::fmt;
use std::time::{Duration, Instant};

use crate::position::PositionResult;

/// 目标距离分级（由 [`crate::position`] 按检测框面积占比给出）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Distance {
    /// 很近：直接停车（双轮制动）。
    Near,
    /// 中等距离：减速前进。
    Mid,
    /// 远距离：全速前进。
    Far,
}

impl Distance {
    /// 位置分析输出的分级名（`near`/`mid`/`far`）。
    pub fn parse(name: &str) -> Self {
        match name {
            "near" => Self::Near,
            "mid" => Self::Mid,
            _ => Self::Far,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Near => "near",
            Self::Mid => "mid",
            Self::Far => "far",
        }
    }
}

/// 一帧视觉观测（由 [`crate::position::PositionResult`] 转换）。
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    /// 本帧是否检测到有效目标。
    pub present: bool,
    /// 目标横向偏差，右为正，范围 -1..=1。
    pub err_x: f32,
    /// 目标距离分级。
    pub distance: Distance,
    /// 置信度（仅用于日志）。
    pub confidence: f32,
}

impl Observation {
    /// 由位置分析结果构造：`err_x = (中心x - 0.5) × 2`（右为正，夹到 ±1）。
    pub fn from_result(result: &PositionResult) -> Self {
        Self {
            present: result.has_target(),
            err_x: ((result.center_x - 0.5) * 2.0).clamp(-1.0, 1.0),
            distance: Distance::parse(&result.distance),
            confidence: result.target_confidence,
        }
    }
}

/// 控制决策输出。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// 差速追踪。
    Track { left: i16, right: i16 },
    /// 原地旋转搜索，`dir` +1 = 右 / -1 = 左。
    Search { left: i16, right: i16, dir: i8 },
    /// 刹车（近距停住）。
    Brake,
}

impl Action {
    /// 双轮速度（`Brake` 没有速度，返回 `None`）。
    pub fn wheels(self) -> Option<(i16, i16)> {
        match self {
            Action::Track { left, right } => Some((left, right)),
            Action::Search { left, right, .. } => Some((left, right)),
            Action::Brake => None,
        }
    }
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Action::Track { left, right } => write!(f, "追踪({left},{right})"),
            Action::Search { left, right, dir } => {
                let side = if dir >= 0 { "右" } else { "左" };
                write!(f, "搜索{side}({left},{right})")
            }
            Action::Brake => f.write_str("刹车"),
        }
    }
}

/// 控制律参数（单位与 ESP32-C3 `SetSpeed` 一致，-100..100）。
#[derive(Debug, Clone, Copy)]
pub struct ControlConfig {
    /// 远距离前进基础速度。
    pub far_speed: i16,
    /// 中距离前进基础速度。
    pub mid_speed: i16,
    /// 差速增益：`turn = turn_gain × err_x × base`。
    pub turn_gain: f32,
    /// 中心死区：`|err_x|` 小于该值不做差速。
    pub turn_deadband: f32,
    /// 最小差速，用于克服小车静摩擦。
    pub min_turn: i16,
    /// 搜索（原地旋转）速度。
    pub search_speed: i16,
    /// 搜索换向周期。
    pub search_flip: Duration,
    /// 目标短暂丢失时保持上一动作的时间。
    pub lost_hold: Duration,
    /// 轮速绝对值上限。
    pub max_speed: i16,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            far_speed: 45,
            mid_speed: 32,
            turn_gain: 0.85,
            turn_deadband: 0.06,
            min_turn: 10,
            search_speed: 26,
            search_flip: Duration::from_secs(3),
            lost_hold: Duration::from_millis(250),
            max_speed: 100,
        }
    }
}

/// 视觉伺服控制器（有状态：记住丢失时间与搜索方向）。
pub struct ControlLoop {
    cfg: ControlConfig,
    last_action: Option<Action>,
    last_seen: Option<Instant>,
    search_dir: i8,
    search_since: Instant,
}

impl ControlLoop {
    pub fn new(cfg: ControlConfig) -> Self {
        Self {
            cfg,
            last_action: None,
            last_seen: None,
            search_dir: -1,
            search_since: Instant::now(),
        }
    }

    pub fn config(&self) -> &ControlConfig {
        &self.cfg
    }

    /// 用一帧观测推进控制律。
    pub fn step(&mut self, obs: &Observation, now: Instant) -> Action {
        if obs.present {
            self.last_seen = Some(now);
            if obs.err_x.abs() > self.cfg.turn_deadband {
                self.search_dir = if obs.err_x > 0.0 { 1 } else { -1 };
            }
            let action = self.track(obs);
            self.last_action = Some(action);
            self.search_since = now;
            return action;
        }

        // 目标丢失：短暂保持，避免单帧漏检导致抖停。
        if let Some(last) = self.last_action {
            let held = self
                .last_seen
                .is_some_and(|at| now.saturating_duration_since(at) <= self.cfg.lost_hold);
            if held {
                return last;
            }
        }
        self.search(now)
    }

    /// 差速追踪；近距直接刹车。
    fn track(&self, obs: &Observation) -> Action {
        let base = match obs.distance {
            Distance::Near => return Action::Brake,
            Distance::Mid => self.cfg.mid_speed,
            Distance::Far => self.cfg.far_speed,
        };
        let err = obs.err_x.clamp(-1.0, 1.0);
        let turn = if err.abs() < self.cfg.turn_deadband {
            0.0
        } else {
            let raw = self.cfg.turn_gain * err * base as f32;
            if raw.abs() < f32::from(self.cfg.min_turn) {
                f32::from(self.cfg.min_turn) * err.signum()
            } else {
                raw
            }
        };
        let max = f32::from(self.cfg.max_speed);
        let left = (base as f32 + turn).round().clamp(-max, max) as i16;
        let right = (base as f32 - turn).round().clamp(-max, max) as i16;
        Action::Track { left, right }
    }

    /// 原地旋转搜索：方向优先延续目标最后出现的一侧，周期换向。
    fn search(&mut self, now: Instant) -> Action {
        if now.saturating_duration_since(self.search_since) >= self.cfg.search_flip {
            self.search_dir = -self.search_dir;
            self.search_since = now;
        }
        let s = self.cfg.search_speed;
        let (left, right) = if self.search_dir >= 0 {
            (s, -s) // 右转：左轮前进、右轮后退
        } else {
            (-s, s) // 左转
        };
        let action = Action::Search {
            left,
            right,
            dir: self.search_dir,
        };
        self.last_action = Some(action);
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(present: bool, err_x: f32, distance: Distance) -> Observation {
        Observation {
            present,
            err_x,
            distance,
            confidence: 0.9,
        }
    }

    #[test]
    fn centered_far_target_goes_straight() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        assert_eq!(
            ctrl.step(&obs(true, 0.0, Distance::Far), t),
            Action::Track {
                left: 45,
                right: 45
            }
        );
    }

    #[test]
    fn target_on_the_right_speeds_up_the_left_wheel() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        let Action::Track { left, right } = ctrl.step(&obs(true, 0.5, Distance::Far), t) else {
            panic!("应当差速追踪");
        };
        assert!(left > right, "左轮应快于右轮: {left} vs {right}");
        assert!(right > 0, "远距离不应减速到 0: {right}");
    }

    #[test]
    fn target_on_the_left_speeds_up_the_right_wheel() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        let Action::Track { left, right } = ctrl.step(&obs(true, -0.5, Distance::Far), t) else {
            panic!("应当差速追踪");
        };
        assert!(right > left, "右轮应快于左轮: {right} vs {left}");
    }

    #[test]
    fn mid_distance_is_slower_than_far() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        let Action::Track { left: far, .. } = ctrl.step(&obs(true, 0.0, Distance::Far), t) else {
            panic!();
        };
        let Action::Track { left: mid, .. } = ctrl.step(&obs(true, 0.0, Distance::Mid), t) else {
            panic!();
        };
        assert!(mid < far);
    }

    #[test]
    fn near_target_brakes() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        assert_eq!(ctrl.step(&obs(true, 0.2, Distance::Near), t), Action::Brake);
    }

    #[test]
    fn speeds_are_clamped_to_max() {
        let cfg = ControlConfig {
            turn_gain: 5.0,
            ..Default::default()
        };
        let mut ctrl = ControlLoop::new(cfg);
        let t = Instant::now();
        let Action::Track { left, right } = ctrl.step(&obs(true, 1.0, Distance::Far), t) else {
            panic!();
        };
        assert_eq!(left, cfg.max_speed);
        assert!(right >= -cfg.max_speed);
        assert!(right < 0, "误差拉满时内轮应反转: {right}");
    }

    #[test]
    fn brief_loss_holds_previous_action() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t0 = Instant::now();
        let tracked = ctrl.step(&obs(true, 0.4, Distance::Far), t0);
        let held = ctrl.step(
            &obs(false, 0.0, Distance::Far),
            t0 + Duration::from_millis(120),
        );
        assert_eq!(tracked, held);
    }

    #[test]
    fn long_loss_searches_toward_last_side_then_flips() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t0 = Instant::now();
        // 目标在右侧丢失 -> 先向右搜索
        ctrl.step(&obs(true, 0.5, Distance::Far), t0);
        let Action::Search { dir, left, right } = ctrl.step(
            &obs(false, 0.0, Distance::Far),
            t0 + Duration::from_millis(400),
        ) else {
            panic!("应当进入搜索");
        };
        assert_eq!(dir, 1, "应先延续目标最后出现的右侧");
        assert_eq!((left, right), (26, -26));

        // 超过换向周期后反向
        let Action::Search { dir, .. } = ctrl.step(
            &obs(false, 0.0, Distance::Far),
            t0 + Duration::from_millis(400 + 3100),
        ) else {
            panic!();
        };
        assert_eq!(dir, -1);
    }

    #[test]
    fn never_seen_target_starts_searching_left() {
        let mut ctrl = ControlLoop::new(ControlConfig::default());
        let t = Instant::now();
        let Action::Search { dir, .. } = ctrl.step(&obs(false, 0.0, Distance::Far), t) else {
            panic!("无目标应搜索");
        };
        assert_eq!(dir, -1);
    }
}
