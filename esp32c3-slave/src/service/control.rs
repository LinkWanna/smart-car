//! 50ms control loop: shared state, motor intent, tick task.
//!
//! Layering: `command_task` (in `main`) only stages *intent* into [`State`]
//! (targets, gains, modes, requests). This module owns everything the tick
//! touches: [`State`] itself, the [`STATE`] global, the per-tick updates
//! (`update_state`, `run_motors`, `update_led`) and the
//! [`control_task`], which is the sole owner of `Motors`, encoder
//! bookkeeping and LED. Closed-loop Move/Rotate goals live in
//! [`crate::service::odometry`]: the tick feeds it encoder counts and brakes
//! both wheels when it reports the goal reached.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::gpio::Output;
use static_cell::StaticCell;

use crate::drivers::motor::{Motors, Pid};
use crate::drivers::{encoder, motor};
#[cfg(feature = "ble")]
use crate::service::ble;
use crate::service::odometry::OdometryState;
#[cfg(feature = "ble")]
use portable_atomic::Ordering;
use protocol::SysState;

// --- App state ---
pub(crate) struct State {
    pub(crate) sys: SysState,
    pub(crate) last_c0: i32,
    pub(crate) last_c1: i32,
    pub(crate) rpm0: i16,
    pub(crate) rpm1: i16,
    pub(crate) pid0: Pid,
    pub(crate) pid1: Pid,
    pub(crate) brake: [bool; 2],
    pub(crate) last_rpm_ms: u64,
    pub(crate) last_led_ms: u64,
    pub(crate) odometry: OdometryState,
}

/// Motor intent staged by commands and applied by the tick.
/// Single entry point for everything driving-related in `State`:
///
/// - `Drive` = Arduino `setMotorSpeed` (clamp ±100, clears brake).
/// - `Coast` = drop the target; the tick coasts the outputs.
/// - `Brake` = the tick holds the outputs high until overridden.
pub(crate) enum MotorCmd {
    Drive(i16),
    Coast,
    Brake,
}

impl State {
    pub(crate) fn new() -> Self {
        Self {
            sys: SysState::Uninit,
            last_c0: 0,
            last_c1: 0,
            rpm0: 0,
            rpm1: 0,
            pid0: Pid::new(),
            pid1: Pid::new(),
            brake: [false, false],
            last_rpm_ms: 0,
            last_led_ms: 0,
            odometry: OdometryState::new(),
        }
    }

    pub(crate) fn pid(&mut self, mid: u8) -> &mut Pid {
        if mid == 0 {
            &mut self.pid0
        } else {
            &mut self.pid1
        }
    }

    /// Stage one motor's intent; the tick applies it within 50ms.
    pub(crate) fn request(&mut self, mid: u8, cmd: MotorCmd) {
        match cmd {
            MotorCmd::Drive(speed) => {
                let s = motor::clamp_speed(speed);
                self.pid(mid).target_rpm = s as f32 * motor::PWM_RPM_MAX / 100.0;
                self.brake[mid as usize] = false;
            }
            MotorCmd::Coast => {
                self.brake[mid as usize] = false;
                self.pid(mid).reset();
            }
            MotorCmd::Brake => {
                self.brake[mid as usize] = true;
                self.pid(mid).reset();
            }
        }
    }
}

pub(crate) type StateMutex = AsyncMutex<CriticalSectionRawMutex, State>;

pub(crate) static STATE: StaticCell<StateMutex> = StaticCell::new();

/// Update the encoder deltas, compute RPM, and enforce link-loss safety.
/// Called by the tick.
fn update_state(st: &mut State, now_ms: u64) {
    let dt_ms = now_ms.saturating_sub(st.last_rpm_ms);
    if dt_ms == 0 {
        return;
    }
    let dt = dt_ms as f32 / 1000.0;

    // RPM from encoder deltas; L negated (Arduino parity), ±5 noise gate.
    let (c0, c1) = encoder::Encoders::read();
    let r0 =
        (((c0.wrapping_sub(st.last_c0)) as f32 / motor::ENCODER_PPR as f32) / dt * 60.0) as i16;
    let r1 =
        (((c1.wrapping_sub(st.last_c1)) as f32 / motor::ENCODER_PPR as f32) / dt * 60.0) as i16;
    let mut rpm0 = -r0;
    let mut rpm1 = r1;
    if rpm0 > -motor::RPM_NOISE_GATE && rpm0 < motor::RPM_NOISE_GATE {
        rpm0 = 0;
    }
    if rpm1 > -motor::RPM_NOISE_GATE && rpm1 < motor::RPM_NOISE_GATE {
        rpm1 = 0;
    }
    st.rpm0 = rpm0;
    st.rpm1 = rpm1;
    st.last_c0 = c0;
    st.last_c1 = c1;
    st.last_rpm_ms = now_ms;

    #[cfg(feature = "ble")]
    if ble::LINK_LOST.swap(false, Ordering::Relaxed) && st.sys >= SysState::Running {
        defmt::warn!("safety: BLE link lost, coasting");
        st.request(0, MotorCmd::Coast);
        st.request(1, MotorCmd::Coast);
        st.odometry.cancel();
        st.sys = SysState::Ready;
    }

    // Closed-loop odometry runs last: a reached goal stages the brake in the
    // same tick that `run_motors` applies it.
    if st.sys >= SysState::Ready && st.odometry.reached([st.last_c0, st.last_c1]) {
        st.request(0, MotorCmd::Brake);
        st.request(1, MotorCmd::Brake);
    }
}

/// Apply the staged motor intent to the outputs. Called by the tick.
fn run_motors(st: &mut State, motors: &Motors) {
    for mid in 0..2u8 {
        // Persistent brake request holds the outputs until overridden.
        if st.brake[mid as usize] {
            let pid = st.pid(mid);
            pid.integral = 0.0;
            pid.output = 0;
            motors.brake(mid);
            continue;
        }
        let (target, rpm) = if mid == 0 {
            (st.pid0.target_rpm, st.rpm0 as f32)
        } else {
            (st.pid1.target_rpm, st.rpm1 as f32)
        };
        if st.sys < SysState::Ready || target.abs() < motor::RPM_DEADZONE {
            let pid = st.pid(mid);
            pid.integral = 0.0;
            pid.output = 0;
            motors.coast(mid);
        } else {
            let out = {
                let pid = st.pid(mid);
                motor::pid_compute(pid, target, rpm);
                pid.output
            };
            motors.drive(mid, out);
        }
    }
}

// LED is active-low (Arduino parity).
fn update_led(st: &mut State, led: &mut Output<'_>, now_ms: u64) {
    match st.sys {
        SysState::Uninit => led.set_high(),
        SysState::Ready => {
            if now_ms.saturating_sub(st.last_led_ms) >= 500 {
                st.last_led_ms = now_ms;
                led.toggle();
            }
        }
        SysState::Running => {
            if now_ms.saturating_sub(st.last_led_ms) >= 100 {
                st.last_led_ms = now_ms;
                led.toggle();
            }
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn control_task(
    state: &'static StateMutex,
    motors: Motors<'static>,
    mut led: Output<'static>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(50));

    loop {
        ticker.next().await;
        let mut st = state.lock().await;
        let now_ms = Instant::now().as_millis();
        update_state(&mut st, now_ms);
        run_motors(&mut st, &motors);
        update_led(&mut st, &mut led, now_ms);
    }
}
