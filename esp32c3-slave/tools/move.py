#!/usr/bin/env python3
"""Closed-loop motion tool for the ESP32-C3 car.

    python tools/move.py move 500                # forward 500 mm
    python tools/move.py move -300 --speed 50    # backward 300 mm
    python tools/move.py rotate 90               # turn left 90° in place
    python tools/move.py rotate -90 --speed 40   # turn right 90°
    python tools/move.py demo                    # 500 mm, left 90°, 500 mm

The firmware closes the loop on the wheel encoders and brakes when the
target is reached; this tool sends one ``MOVE``/``ROTATE`` and then polls
``STATUS`` (through Heartbeat frames) until ``dist_result`` latches.

The link and motion defaults come from ``tools/src/config.py``.
Requires ``bleak`` (``pip install bleak``).
"""

from __future__ import annotations

import argparse
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

from protocol import (
    RX_CHAR,
    TX_CHAR,
    Ack,
    Frame,
    FrameParser,
    Heartbeat,
    Init,
    MotorTarget,
    Move,
    MoveDir,
    Nack,
    ProtocolError,
    RequestType,
    Rotate,
    RotateDir,
    Status,
    Stop,
    chunks,
    decode_response,
    encode_request,
)

warnings.filterwarnings("ignore", message="Using default MTU value")

#: Wire unit of rotate targets: 0.1°.
TENTHS_DEG = 10
#: Poll period while a goal is running.
POLL_S = 0.1
#: Give up on a single motion after this long.
MOVE_TIMEOUT_S = 30.0


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


class Link:
    """One notification queue for the ACK/NACK and STATUS frames."""

    def __init__(self, client: BleakClient) -> None:
        self.client = client
        self.parser = FrameParser()
        self.frames: asyncio.Queue[tuple] = asyncio.Queue()

    def _on_frame(self, _char, data: bytearray) -> None:
        for event in self.parser.feed(bytes(data)):
            if not isinstance(event, Frame):
                continue
            try:
                response = decode_response(event.cmd, event.payload)
            except ProtocolError:
                continue
            if isinstance(response, Status):
                self.frames.put_nowait(("status", response))
            elif isinstance(response, Ack):
                self.frames.put_nowait(("ack", response.cmd))
            elif isinstance(response, Nack):
                self.frames.put_nowait(("nack", response.cmd, response.error))

    async def start(self) -> None:
        await self.client.start_notify(TX_CHAR, self._on_frame)

    async def expect_ack(self, cmd: int, timeout: float = 1.0) -> None:
        while True:
            try:
                kind, *args = await asyncio.wait_for(self.frames.get(), timeout)
            except asyncio.TimeoutError:
                raise SystemExit(f"no ACK/NACK for {cmd:#04x}") from None
            if kind == "ack" and args[0] == cmd:
                return
            if kind == "nack" and args[0] == cmd:
                raise SystemExit(f"command {cmd:#04x} refused (NACK {args[1]:#04x})")
            # Anything else (status frames and so on) is drained silently.

    async def poll_status(self, timeout: float) -> Status:
        """Send a Heartbeat and wait for its STATUS."""
        await send(self.client, encode_request(Heartbeat()))
        try:
            kind, *args = await asyncio.wait_for(self.frames.get(), timeout)
        except asyncio.TimeoutError:
            raise SystemExit("no STATUS from the car (link lost?)") from None
        if kind != "status":
            raise SystemExit("unexpected frame while waiting for STATUS")
        return args[0]


async def wait_goal(link: Link) -> None:
    """Poll STATUS until the closed-loop goal completes."""
    deadline = time.monotonic() + MOVE_TIMEOUT_S
    while True:
        if time.monotonic() >= deadline:
            raise SystemExit("motion timed out")
        status = await link.poll_status(POLL_S + 0.5)
        if not status.dist_active:
            if status.done:
                return
            raise SystemExit("closed loop stopped before reaching the target")


def move_frame(mm: int, speed: int) -> bytes:
    """MOVE frame from a signed distance (+forward, -backward)."""
    direction = MoveDir.FORWARD if mm >= 0 else MoveDir.BACKWARD
    return encode_request(Move(direction, speed, abs(mm)))


def rotate_frame(deg: float, speed: int) -> bytes:
    """ROTATE frame from a signed angle (+left, -right)."""
    direction = RotateDir.LEFT if deg >= 0 else RotateDir.RIGHT
    return encode_request(Rotate(direction, speed, round(abs(deg) * TENTHS_DEG)))


def parse_args() -> argparse.Namespace:
    cfg = DEFAULTS
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="action", required=True)

    sub_move = sub.add_parser("move", help="drive straight, mm (+fwd, -back)")
    sub_move.add_argument("mm", type=int)
    sub_move.add_argument("--speed", type=int, default=cfg.move_speed)

    sub_rot = sub.add_parser("rotate", help="spin in place, deg (+left, -right)")
    sub_rot.add_argument("deg", type=float)
    sub_rot.add_argument("--speed", type=int, default=cfg.rotate_speed)

    sub.add_parser("demo", help="500 mm forward, 90° left, 500 mm forward")
    return parser.parse_args()


async def run_job(link: Link, cmd: int, frame: bytes, label: str) -> None:
    started = time.monotonic()
    await send(link.client, frame)
    await link.expect_ack(cmd)
    print(f"{label} ...")
    await wait_goal(link)
    print(f"  reached in {time.monotonic() - started:.1f}s")


async def amain() -> None:
    args = parse_args()
    cfg = DEFAULTS
    if args.action == "move":
        jobs = [
            (RequestType.MOVE, move_frame(args.mm, args.speed), f"move {args.mm} mm")
        ]
    elif args.action == "rotate":
        jobs = [
            (
                RequestType.ROTATE,
                rotate_frame(args.deg, args.speed),
                f"rotate {args.deg}°",
            )
        ]
    else:
        jobs = [
            (RequestType.MOVE, move_frame(500, cfg.move_speed), "move 500 mm"),
            (RequestType.ROTATE, rotate_frame(90, cfg.rotate_speed), "rotate 90°"),
            (RequestType.MOVE, move_frame(500, cfg.move_speed), "move 500 mm"),
        ]

    device = await find_device(cfg)
    print(f"connecting to {device} ...")

    async with BleakClient(device, timeout=cfg.scan_timeout) as client:
        print(f"connected (mtu={client.mtu_size})")
        link = Link(client)
        try:
            await link.start()
            await send(client, encode_request(Init()))
            await link.expect_ack(RequestType.INIT)
            print("car ready")
            for cmd, frame, label in jobs:
                await run_job(link, cmd, frame, label)
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
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
