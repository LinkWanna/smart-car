//! [`DriveTarget`] 的串口链路实现：把网页遥控落到 [`Car`] 上。
//!
//! `Car` 的所有方法都取 `&self` 且内部有锁，`Arc<Car>` 可以直接当
//! `Arc<dyn DriveTarget>` 用（见 `src/bin/webctl.rs`）。

use crate::control::Car;

use super::{DriveTarget, LinkSnapshot};

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
