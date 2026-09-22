#!/usr/bin/env python3
"""Keyboard teleop demo for the ESP32-C3 car.

    W (hold)        accelerate forward
    S (hold)        brake, then reverse once stopped
    A / D (hold)    steer left / right (pivots in place at a standstill)
    Q               quit (the car is stopped on exit)

Takes no options: the link, motor setup and driving feel all come from
``tools/src/config.py``. The script connects over BLE, runs ``init`` + ``config``
and then streams ``speeds`` targets at the control rate while showing a one
line HUD built from the ``Heartbeat`` -> ``STATUS`` responses.

Requires ``bleak`` (``pip install bleak``).
"""

from __future__ import annotations

import asyncio
import sys
import time
import warnings
from pathlib import Path

# The shared library modules live in tools/src/, next to this script.
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - dependency hint
    sys.exit("bleak is required: pip install bleak")

from config import DEFAULTS, Config
from keyboard import Keyboard

from protocol import (
    RX_CHAR,
    TX_CHAR,
    Ack,
    Frame,
    FrameParser,
    Heartbeat,
    Init,
    MotorTarget,
    Nack,
    ProtocolError,
    RequestType,
    SetSpeeds,
    Status,
    Stop,
    chunks,
    decode_response,
    encode_request,
    sys_state_name,
)

warnings.filterwarnings("ignore", message="Using default MTU value")

EPS = 0.01
#: Throttle below which steering pivots the car in place instead.
PIVOT_THRESHOLD = 2.0
#: Heartbeat rate: refreshes STATUS for the HUD.
KEEPALIVE_HZ = 5.0


async def send(client: BleakClient, frame: bytes) -> None:
    for chunk in chunks(frame):
        await client.write_gatt_char(RX_CHAR, chunk, response=False)


async def find_device(cfg: Config):
    if cfg.device_address:
        return cfg.device_address
    print(f"scanning for '{cfg.device_name}' ...")
    device = await BleakScanner.find_device_by_name(
        cfg.device_name, timeout=cfg.scan_timeout
    )
    if device is None:
        raise SystemExit(
            f"device '{cfg.device_name}' not found\n"
            "hint: after a hard kill BlueZ can keep the old link alive, run\n"
            "      bluetoothctl disconnect <ADDRESS>   and try again"
        )
    return device


async def prepare(client: BleakClient, cfg: Config) -> None:
    """Bring the firmware from Uninit to Ready and check the ACK."""
    parser = FrameParser()
    pending: asyncio.Queue[int] = asyncio.Queue()

    def on_frame(_char, data: bytearray) -> None:
        for event in parser.feed(bytes(data)):
            if not isinstance(event, Frame):
                continue
            try:
                response = decode_response(event.cmd, event.payload)
            except ProtocolError:
                continue
            if isinstance(response, Ack):
                pending.put_nowait(response.cmd)
            elif isinstance(response, Nack):
                pending.put_nowait(-response.cmd)

    await client.start_notify(TX_CHAR, on_frame)

    await send(client, encode_request(Init()))
    try:
        ack = await asyncio.wait_for(pending.get(), timeout=1.0)
    except asyncio.TimeoutError:
        raise SystemExit("no ACK for init (is the firmware running?)") from None
    if ack != RequestType.INIT:
        raise SystemExit("init refused by the firmware")

    print("car ready")
    await client.stop_notify(TX_CHAR)


def move_toward(current: float, target: float, step: float) -> float:
    """Ramp ``current`` toward ``target`` by at most ``step``."""
    if current < target:
        return min(target, current + step)
    return max(target, current - step)


def mix(cfg: Config, throttle: float, steer: float) -> list[int]:
    """Wheel commands for the current throttle/steer.

    Driving uses a differential; below :data:`PIVOT_THRESHOLD` the wheels
    counter-rotate instead, so steering alone spins the car in place.
    ``steer`` is positive for a left turn.
    """
    if abs(throttle) <= PIVOT_THRESHOLD:
        base, turn = 0.0, cfg.pivot_speed
    else:
        base = throttle
        turn = cfg.turn_gain * abs(throttle) / cfg.max_speed if cfg.max_speed else 0.0

    left, right = base - steer * turn, base + steer * turn
    if cfg.invert_left:
        left = -left
    if cfg.invert_right:
        right = -right
    commands = [0, 0]
    commands[cfg.motor_left] = max(-100, min(100, round(left)))
    commands[cfg.motor_right] = max(-100, min(100, round(right)))
    return commands


def hud(
    cfg: Config,
    throttle: float,
    steer: float,
    flags: str,
    status: Status | None,
    commands: list[int],
) -> str:
    filled = (
        max(0, min(10, round(abs(throttle) / cfg.max_speed * 10)))
        if cfg.max_speed
        else 0
    )
    bar = "#" * filled + "-" * (10 - filled)
    left, right = cfg.motor_left, cfg.motor_right
    if status is None:
        wheels, state = "rpm  ----/----", "----"
    else:
        wheels = f"rpm {status.rpm[left]:+4d}/{status.rpm[right]:+4d}"
        state = sys_state_name(status.sys)
    return (
        f"thr [{bar}] {throttle:+5.1f}  steer {steer:+.2f}  {flags:<16} "
        f"cmd {commands[left]:+4d}/{commands[right]:+4d}  {wheels}  {state:<7}"
    )


async def drive(client: BleakClient, cfg: Config, keyboard: Keyboard) -> None:
    status: Status | None = None
    parser = FrameParser()

    def on_frame(_char, data: bytearray) -> None:
        nonlocal status
        for event in parser.feed(bytes(data)):
            if not isinstance(event, Frame):
                continue
            try:
                response = decode_response(event.cmd, event.payload)
            except ProtocolError:
                continue
            if isinstance(response, Status):
                status = response

    await client.start_notify(TX_CHAR, on_frame)
    print(f"input: {keyboard.description}")
    print("hold W to accelerate, S to brake/reverse, A/D to steer, Q to quit\n")

    throttle = 0.0
    steer = 0.0
    last_sent: list[int] | None = None
    last_keepalive = 0.0
    last_hud = 0.0
    last = time.monotonic()

    while not keyboard.quit:
        await asyncio.sleep(1.0 / cfg.control_hz)
        now = time.monotonic()
        dt = min(now - last, 0.25)
        last = now
        keyboard.poll()

        if not client.is_connected:
            print("\nBLE link lost, stopping")
            return

        w = keyboard.held("w", now)
        s = keyboard.held("s", now)
        a = keyboard.held("a", now)
        d = keyboard.held("d", now)

        # W/S ramp the throttle toward their target; nothing coasts to 0.
        if w:
            goal = float(cfg.max_speed)
            rate = cfg.brake_rate if throttle < 0 else cfg.accel_rate
        elif s:
            goal = -float(cfg.max_reverse)
            rate = cfg.brake_rate if throttle > 0 else cfg.accel_rate
        else:
            goal, rate = 0.0, cfg.decay_rate
        throttle = move_toward(throttle, goal, rate * dt)

        # Steering: +1 is left (A), -1 is right (D).
        goal = (1.0 if a else 0.0) - (1.0 if d else 0.0)
        if cfg.invert_steer:
            goal = -goal
        steer = move_toward(steer, goal, cfg.steer_rate * dt)
        braking = s and throttle > EPS

        commands = mix(cfg, throttle, steer)
        if commands != last_sent:
            await send(client, encode_request(SetSpeeds(commands[0], commands[1])))
            last_sent = commands
        if now - last_keepalive >= 1.0 / KEEPALIVE_HZ:
            # Refreshes STATUS for the HUD.
            await send(client, encode_request(Heartbeat()))
            last_keepalive = now

        if now - last_hud >= 1.0 / cfg.hud_hz:
            last_hud = now
            flags = " ".join(
                name
                for name, on in (
                    ("PIVOT", abs(throttle) <= PIVOT_THRESHOLD and abs(steer) > 0.05),
                    ("REV", throttle < -EPS),
                    ("BRAKE", braking),
                )
                if on
            )
            print(
                "\r" + hud(cfg, throttle, steer, flags, status, commands) + "\x1b[K",
                end="",
                flush=True,
            )

    print("\nstopping")


async def amain() -> None:
    cfg = DEFAULTS
    keyboard = Keyboard(cfg)
    device = await find_device(cfg)
    print(f"connecting to {device} ...")

    async with BleakClient(device, timeout=cfg.scan_timeout) as client:
        print(f"connected (mtu={client.mtu_size})")
        try:
            await prepare(client, cfg)
            keyboard.flush()
            await drive(client, cfg, keyboard)
        finally:
            try:
                if client.is_connected:
                    await asyncio.wait_for(
                        send(client, encode_request(Stop(MotorTarget.BOTH))),
                        timeout=1.0,
                    )
            except Exception:  # noqa: BLE001, S110 - best effort on shutdown
                pass
            print("car stopped")


def main() -> None:
    if len(sys.argv) > 1:
        raise SystemExit(
            "play.py takes no arguments (edit tools/config.py to change the setup)"
        )
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
