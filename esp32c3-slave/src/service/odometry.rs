//! Closed-loop Move/Rotate control: goal state, unit conversion, reach check.
//!
//! Layering: [`OdometryState`] is embedded in
//! [`State`](crate::service::control::State) as its `odometry` field.
//! `command_task` stages one goal at a time (direction, encoder snapshot,
//! target in counts) and drives the wheels; the 50ms control tick feeds the
//! latest encoder counts into [`OdometryState::reached`] and brakes both wheels
//! when it reports the goal. Motor staging itself stays in `control`, next to
//! `MotorCmd` and the tick.

use core::f32::consts::PI;

use crate::drivers::{encoder, motor};

/// Direction of a closed-loop odometry goal, one-to-one with the wire `dir`
/// byte: Move uses 0/1 = forward/backward, Rotate uses 0/1 = left/right.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum DistDir {
    Forward,
    Backward,
    Left,
    Right,
}

impl DistDir {
    /// In-place turns measure both counter-rotating wheels; straight moves
    /// average them instead (see [`OdometryState::reached`]).
    fn is_turn(self) -> bool {
        matches!(self, Self::Left | Self::Right)
    }

    /// Wheel speeds `[left, right]` (`SET_SPEED` units, clamped) that drive
    /// this direction.
    pub(crate) fn wheel_speeds(self, speed: i16) -> [i16; 2] {
        let s = motor::clamp_speed(speed);
        match self {
            Self::Forward => [s, s],
            Self::Backward => [-s, -s],
            Self::Left => [-s, s],
            Self::Right => [s, -s],
        }
    }
}

/// One closed-loop goal (Move/Rotate) and its outcome; no goal is staged until
/// [`start`](Self::start).
pub(crate) struct OdometryState {
    /// A goal is staged and still running.
    pub(crate) active: bool,
    /// Latched outcome for `Status`: 0 = none/running, 1 = goal reached.
    pub(crate) result: u8,
    dir: DistDir,
    /// Encoder snapshot at the moment the goal was staged.
    start: [i32; 2],
    /// Goal distance in encoder counts.
    target: i64,
}

impl OdometryState {
    pub(crate) const fn new() -> Self {
        Self {
            active: false,
            result: 0,
            dir: DistDir::Forward,
            start: [0, 0],
            target: 0,
        }
    }

    /// Stage a goal: snapshot the encoders and clear the previous result.
    pub(crate) fn start(&mut self, dir: DistDir, target_counts: i32) {
        let (c0, c1) = encoder::Encoders::read();
        self.start = [c0, c1];
        self.target = target_counts as i64;
        self.dir = dir;
        self.active = true;
        self.result = 0;
    }

    /// Drop the staged goal without latching a result.
    pub(crate) fn cancel(&mut self) {
        self.active = false;
        self.result = 0;
    }

    /// Compare the latest encoder `counts` against the staged goal. Latches
    /// `result = 1` and returns `true` once the encoder delta reaches the
    /// target, so the caller can brake both wheels.
    pub(crate) fn reached(&mut self, counts: [i32; 2]) -> bool {
        if !self.active {
            return false;
        }
        let d0 = counts[0].wrapping_sub(self.start[0]).unsigned_abs() as i64;
        let d1 = counts[1].wrapping_sub(self.start[1]).unsigned_abs() as i64;
        let delta = if self.dir.is_turn() {
            d0.max(d1)
        } else {
            (d0 + d1) / 2
        };
        if delta >= self.target {
            self.active = false;
            self.result = 1;
            defmt::info!("odometry: goal reached ({} counts)", delta);
            return true;
        }
        false
    }
}

// --- Unit conversions (Arduino parity) ---
//
// Wire targets are mm (straight move) and 0.1° (in-place rotation); the
// closed loop counts encoder pulses:
//
// - wheel circumference `C = π * D` (`D` = wheel diameter)
// - straight: `counts = mm * PPR / C`
// - in-place turn: one wheel travels the arc `θ(rad) * L / 2`
//   (`L` = wheelbase, the distance between the two wheel centres)

/// Wheel diameter `D` in mm (measured on this car).
const WHEEL_DIAMETER_MM: f32 = 62.0;
/// Wheelbase `L` in mm: distance between the left and right wheel centres.
const WHEELBASE_MM: f32 = 160.0;

/// Travel distance in mm -> encoder counts (rounded, sign preserved).
pub(crate) fn mm_to_counts(mm: f32) -> i32 {
    let v = mm * motor::ENCODER_PPR as f32 / (WHEEL_DIAMETER_MM * PI);
    if v >= 0.0 {
        (v + 0.5) as i32
    } else {
        (v - 0.5) as i32
    }
}

/// In-place rotation angle in degrees -> one wheel's encoder counts.
pub(crate) fn degrees_to_counts(degrees: f32) -> i32 {
    let arc_mm = degrees * (PI / 180.0) * (WHEELBASE_MM / 2.0);
    mm_to_counts(arc_mm)
}
