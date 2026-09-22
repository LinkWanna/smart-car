//! Motor math and LEDC driver, ported verbatim from Arduino `base_control.ino`.
//!
//! Conventions (do not "fix"):
//! - `SET_SPEED` range is ±100 (clamped on receipt, Arduino parity).
//! - L RPM is negated, R is not.
//! - R (motor index 1) drive outputs are inverted vs L.
//! - `Kd` is stored but unused (same as Arduino).

use esp_hal::ledc::{LowSpeed, channel, channel::ChannelHW};

/// Full-scale RPM at PWM 255 (linear model, kept for the speed scale).
pub const PWM_RPM_MAX: f32 = 150.0;
/// PWM needed to break static friction (measured on this car).
pub const BREAKAWAY_PWM: f32 = 155.0;
/// PWM per rpm once moving (measured on this car).
pub const PWM_PER_RPM: f32 = 0.9;
/// PID gains.
pub const PID_KP: f32 = 0.8;
pub const PID_KI: f32 = 0.10;
pub const PID_KD: f32 = 0.01;
/// Integral anti-windup clamp.
pub const PID_INTEGRAL_MAX: f32 = 300.0;
/// Output clamp (PWM units).
pub const PID_OUTPUT_MAX: i32 = 255;
/// Below this target RPM the motor is considered stopped.
pub const RPM_DEADZONE: f32 = 3.0;
/// Lowest RPM the motor can sustain.
pub const MIN_USEFUL_RPM: f32 = 10.0;
/// Extra PWM to break static friction (only when stalled).
pub const FRICTION_BOOST: f32 = 10.0;
/// Feed-forward gain (the plant model in `rpm_to_pwm` is already measured).
pub const FF_GAIN: f32 = 1.0;
/// RPM noise gate.
pub const RPM_NOISE_GATE: i16 = 5;

/// PWM carrier frequency, fixed at boot. Runtime only varies the duty.
pub const PWM_FREQ_HZ: u32 = 20_000;

/// `SET_SPEED` clamp, Arduino parity.
pub const SPEED_MIN: i16 = -100;
pub const SPEED_MAX: i16 = 100;

/// Encoder pulses per wheel revolution (fixed by the motor/gearbox hardware).
pub const ENCODER_PPR: u16 = 4680;

/// Clamp a `SET_SPEED` value to ±100.
pub fn clamp_speed(v: i16) -> i16 {
    v.clamp(SPEED_MIN, SPEED_MAX)
}

/// Target RPM -> PWM (0 for non-positive input).
///
/// Measured on this car (free running, 8-bit LEDC): the wheels stay put below
/// about 150 PWM, then roughly `155 + 0.9 * rpm`. The old pure-linear
/// `rpm * 255 / 150` model under-drove every low target, so a differential
/// turn stalled its inner wheel instead of turning.
pub fn rpm_to_pwm(rpm: f32) -> f32 {
    if rpm <= 0.0 {
        0.0
    } else {
        BREAKAWAY_PWM + rpm * PWM_PER_RPM
    }
}

/// PID controller state (one per motor).
#[derive(Debug, Clone, Copy)]
pub struct Pid {
    pub target_rpm: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub prev_error: f32,
    pub output: i32,
}

impl Pid {
    pub const fn new() -> Self {
        Self {
            target_rpm: 0.0,
            kp: PID_KP,
            ki: PID_KI,
            kd: PID_KD,
            integral: 0.0,
            prev_error: 0.0,
            output: 0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.output = 0;
        self.target_rpm = 0.0;
    }
}

/// Feed-forward + P + small I (I only fills feed-forward error, never dominates).
/// Verbatim port of Arduino `computePID` (`Kd` intentionally unused).
pub fn pid_compute(pid: &mut Pid, target_rpm: f32, current_rpm: f32) {
    let abs_target = libm_abs(target_rpm);
    let abs_current = libm_abs(current_rpm);
    let sign = if target_rpm > 0.0 {
        1.0
    } else if target_rpm < 0.0 {
        -1.0
    } else {
        0.0
    };
    let bias = target_rpm - current_rpm;

    // Feed-forward.
    let ff = sign * rpm_to_pwm(abs_target) * FF_GAIN;

    // Static-friction breakaway, only when stalled.
    let mut friction_boost = 0.0;
    if abs_current < 2.0 && abs_target > RPM_DEADZONE {
        let pwm_need = rpm_to_pwm(MIN_USEFUL_RPM) + FRICTION_BOOST;
        if rpm_to_pwm(abs_target) < pwm_need {
            friction_boost = sign * (pwm_need - rpm_to_pwm(abs_target));
        }
    }

    // Integral with saturation guard.
    let sat_high = pid.output >= PID_OUTPUT_MAX && bias > 0.0;
    let sat_low = pid.output <= -PID_OUTPUT_MAX && bias < 0.0;
    if !sat_high && !sat_low {
        pid.integral += bias;
    }
    pid.integral = pid.integral.clamp(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    let p_term = PID_KP * bias;
    let i_term = pid.ki * pid.integral;

    let total = ff + friction_boost + p_term + i_term;
    pid.output = (total as i32).clamp(-PID_OUTPUT_MAX, PID_OUTPUT_MAX);
}

fn libm_abs(v: f32) -> f32 {
    if v < 0.0 { -v } else { v }
}

// --- Motor driver (4x LEDC PWM, Arduino analogWrite parity) ---
//
// All channels share LEDC Timer0 at 8-bit resolution and are bound to the
// real timer: the PWM frequency is fixed at boot (`PWM_FREQ_HZ`) and runtime
// only varies the duty, so the timer is never reconfigured and no
// ownership workaround is needed.
/// Motor index of the left wheel.
pub const MOTOR_LEFT: u8 = 0;
/// Motor index of the right wheel (kept for symmetry; wiring docs).
#[allow(dead_code)]
pub const MOTOR_RIGHT: u8 = 1;

pub struct Motors<'a> {
    l_in1: channel::Channel<'a, LowSpeed>,
    l_in2: channel::Channel<'a, LowSpeed>,
    r_in1: channel::Channel<'a, LowSpeed>,
    r_in2: channel::Channel<'a, LowSpeed>,
}

impl<'a> Motors<'a> {
    pub fn new(
        l_in1: channel::Channel<'a, LowSpeed>,
        l_in2: channel::Channel<'a, LowSpeed>,
        r_in1: channel::Channel<'a, LowSpeed>,
        r_in2: channel::Channel<'a, LowSpeed>,
    ) -> Self {
        Self {
            l_in1,
            l_in2,
            r_in1,
            r_in2,
        }
    }

    fn duty(ch: &channel::Channel<'a, LowSpeed>, v: u8) {
        ch.set_duty_hw(v as u32);
    }

    fn l_fwd(&self, pwm: u8) {
        Self::duty(&self.l_in1, pwm);
        Self::duty(&self.l_in2, 0);
    }

    fn l_rev(&self, pwm: u8) {
        Self::duty(&self.l_in1, 0);
        Self::duty(&self.l_in2, pwm);
    }

    fn r_fwd(&self, pwm: u8) {
        Self::duty(&self.r_in1, 0);
        Self::duty(&self.r_in2, pwm);
    }

    fn r_rev(&self, pwm: u8) {
        Self::duty(&self.r_in1, pwm);
        Self::duty(&self.r_in2, 0);
    }

    pub fn drive(&self, motor: u8, out: i32) {
        let pwm = out.abs().min(255) as u8;
        if motor == MOTOR_LEFT {
            if out > 0 {
                self.l_fwd(pwm);
            } else {
                self.l_rev(pwm);
            }
        } else if out > 0 {
            self.r_fwd(pwm);
        } else {
            self.r_rev(pwm);
        }
    }

    pub fn coast(&self, motor: u8) {
        if motor == MOTOR_LEFT {
            Self::duty(&self.l_in1, 0);
            Self::duty(&self.l_in2, 0);
        } else {
            Self::duty(&self.r_in1, 0);
            Self::duty(&self.r_in2, 0);
        }
    }

    pub fn brake(&self, motor: u8) {
        if motor == MOTOR_LEFT {
            Self::duty(&self.l_in1, 255);
            Self::duty(&self.l_in2, 255);
        } else {
            Self::duty(&self.r_in1, 255);
            Self::duty(&self.r_in2, 255);
        }
    }
}
