//! 遥控输出端抽象与链路快照。
//!
//! [`DriveTarget`] 是控制层唯一向下的接口：手动遥控（[`crate::control::teleop`]）
//! 与视觉伺服（[`crate::control::servo`]）都只产出速度/动作，由
//! [`crate::control::session`] 落到这里。真实实现是 [`Car`]；测试用替身实现同一
//! trait（见 `web::server` 的测试）。
//!
//! `Car` 的所有方法都取 `&self` 且内部有锁，`Arc<Car>` 可以直接当
//! `Arc<dyn DriveTarget>` 用（`smartcar` 即这么用）。

use std::time::Duration;

use super::Counters;
use super::car::Car;

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

/// 遥控输出端：真实串口链路（[`Car`]）或测试替身。
pub trait DriveTarget: Send + Sync {
    /// 双轮目标速度（-100..100）。
    fn set_speeds(&self, left: i16, right: i16);

    /// 滑行。
    fn coast(&self);

    /// 短接制动。
    fn brake(&self);

    /// 编码器清零并进入 Ready。
    fn init(&self);

    /// 回 Uninit（调试用）。
    fn reset(&self);

    /// 下位机处于 Uninit / 状态不对时补发 `Init`；返回是否触发。
    fn maybe_reinit(&self) -> bool {
        false
    }

    /// 当前链路快照。
    fn snapshot(&self) -> LinkSnapshot;
}

impl DriveTarget for Car {
    fn set_speeds(&self, left: i16, right: i16) {
        self.set_desired(left, right);
    }

    fn coast(&self) {
        Car::coast(self);
    }

    fn brake(&self) {
        Car::brake(self);
    }

    fn init(&self) {
        Car::init(self);
    }

    fn reset(&self) {
        Car::reset(self);
    }

    fn maybe_reinit(&self) -> bool {
        Car::maybe_reinit(self)
    }

    fn snapshot(&self) -> LinkSnapshot {
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
}
