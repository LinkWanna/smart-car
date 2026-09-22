"""Shared defaults for the ESP32-C3 car tools.

This is a library, not a script: import it from the tools.

    from dataclasses import replace
    from config import DEFAULTS, Config

    cfg = DEFAULTS                            # built-in defaults
    cfg = replace(DEFAULTS, max_speed=40)     # one-off override

Editing a value here changes the project default for every tool.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Config:
    """BLE link, motor setup and driving feel."""

    # --- BLE link ---
    device_name: str = "ESP32C3-CAR"
    device_address: str | None = None  # set to skip scanning
    scan_timeout: float = 10.0

    # --- motor setup (must match the firmware wiring) ---
    motor_left: int = 0  # motor index of the left wheel
    motor_right: int = 1
    invert_left: bool = False  # flip a wheel that spins backwards
    invert_right: bool = False
    invert_steer: bool = False

    # --- driving feel (play.py) ---
    max_speed: int = 55  # forward throttle ceiling [%]
    max_reverse: int = 40  # reverse throttle ceiling [%]
    accel_rate: float = 90.0  # throttle units per second (W/S targets)
    brake_rate: float = 350.0  # ... when a key crosses zero, i.e. braking
    decay_rate: float = 50.0  # ... toward zero when no throttle key is held
    turn_gain: float = 40.0  # differential at full throttle and full steer
    steer_rate: float = 6.0  # steering travel per second (0..1 scale)
    pivot_speed: float = 40.0  # wheel command for the in-place spin
    control_hz: float = 20.0  # command loop rate
    hud_hz: float = 10.0  # HUD refresh rate
    hold_timeout: float = 0.6  # fallback keyboard: held-key timeout

    # --- closed-loop motion (move.py) ---
    move_speed: int = 60  # straight move throttle [%]
    rotate_speed: int = 40  # in-place rotation throttle [%]


#: Default configuration used by every tool.
DEFAULTS = Config()
