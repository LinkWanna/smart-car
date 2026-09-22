"""Terminal keyboard input for the car tools.

Prefers the **kitty keyboard protocol** (Konsole, kitty, foot, wezterm,
Ghostty, ...): the terminal then reports press/repeat/release events, so a
held key is an exact state.

Other terminals only report presses and auto-repeat, so there a key counts as
held while its events keep arriving — or while any other key is still
repeating, because Wayland only repeats the most recently pressed key.

``Keyboard.real_events`` says which backend is active.
"""

from __future__ import annotations

import atexit
import os
import select
import signal
import sys
import termios
import time
import tty

from config import Config

#: Keys the car tools use.
KEYS = "wasd"
#: Kitty flags: disambiguate (1) + report event types (2) + all keys as CSI u (8).
KITTY_FLAGS = 1 | 2 | 8
#: How long to wait for the terminal to answer the protocol probe.
PROBE_TIMEOUT = 0.25


class Keyboard:
    """Raw mode key state, exact when the terminal supports it."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.fd = sys.stdin.fileno()
        if not os.isatty(self.fd):
            raise SystemExit("the car tools must run in an interactive terminal")
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        atexit.register(self.restore)

        self.real_events = False
        self.down: dict[str, bool] = {}  # kitty: exact press/release
        self.last: dict[str, float] = {}  # fallback: last press time
        self.activity = 0.0
        self.quit = False

        # 0 = normal, 1 = after ESC, 2 = collecting a CSI sequence.
        self._state = 0
        self._csi = bytearray()

        signal.signal(signal.SIGINT, lambda *_: self.request_quit())
        signal.signal(signal.SIGTERM, lambda *_: self.request_quit())
        self._probe()

    # --- lifecycle -----------------------------------------------------
    def restore(self) -> None:
        if self.real_events:
            os.write(self.fd, b"\x1b[<u")  # pop the pushed kitty flags
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def request_quit(self) -> None:
        self.quit = True

    def flush(self) -> None:
        """Drop everything buffered before the car was ready."""
        self.down.clear()
        self.last.clear()
        self.activity = 0.0
        while select.select([sys.stdin], [], [], 0)[0]:
            os.read(self.fd, 4096)

    @property
    def description(self) -> str:
        return (
            "kitty keyboard protocol (exact press/release)"
            if self.real_events
            else "auto-repeat fallback (held keys are approximated)"
        )

    # --- protocol probe ------------------------------------------------
    def _probe(self) -> None:
        """Ask the terminal whether it speaks the kitty keyboard protocol."""
        os.write(self.fd, b"\x1b[?u")
        deadline = time.monotonic() + PROBE_TIMEOUT
        reply = bytearray()
        while time.monotonic() < deadline:
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                reply += os.read(self.fd, 64)
                if reply.endswith(b"u"):
                    break
        if b"[?" in reply and reply.rstrip(b"u").rsplit(b"?", 1)[-1].isdigit():
            self.real_events = True
            os.write(self.fd, f"\x1b[>{KITTY_FLAGS}u".encode())
        elif reply:  # anything typed during the probe belongs to the fallback
            for byte in reply:
                self._feed(byte)

    # --- input ---------------------------------------------------------
    def poll(self) -> None:
        while select.select([sys.stdin], [], [], 0)[0]:
            try:
                chunk = os.read(self.fd, 64)
            except OSError:
                return
            for byte in chunk:
                self._feed(byte)

    def _feed(self, byte: int) -> None:
        if self._state == 2:  # collecting CSI parameters
            if 0x40 <= byte <= 0x7E:  # final byte
                self._state = 0
                self._handle_csi(bytes(self._csi) + bytes([byte]))
                self._csi.clear()
            else:
                self._csi.append(byte)
            return
        if self._state == 1:  # after ESC: expect '['
            self._state = 2 if byte == ord("[") else 0
            return
        if byte == 0x1B:
            self._state = 1
            return
        now = time.monotonic()
        if self.real_events:
            return  # every key arrives as CSI u while the protocol is on
        self.activity = now
        char = chr(byte)
        if char in ("q", "Q"):
            self.quit = True
        elif char.lower() in KEYS:
            self.last[char.lower()] = now

    def _handle_csi(self, sequence: bytes) -> None:
        """Decode a kitty ``CSI key ; mods : event u`` sequence (mods ignored)."""
        if not self.real_events or not sequence.endswith(b"u"):
            return
        body = sequence[:-1].decode("latin-1")
        if body.startswith("?"):  # probe reply
            return
        params = body.split(";")
        try:
            code = int(params[0].split(":")[0] or "0")
            event = 1
            if len(params) > 1:
                fields = params[1].split(":")
                if len(fields) > 1:
                    event = int(fields[1] or "1")
        except ValueError:
            return

        char = chr(code) if 32 <= code < 127 else ""
        if char in ("q", "Q"):
            if event != 3:
                self.quit = True
        elif char.lower() in KEYS:
            self.down[char.lower()] = (
                event != 3
            )  # press and repeat hold, release clears

    # --- key state -----------------------------------------------------
    def held(self, key: str, now: float) -> bool:
        """Return whether ``key`` is held."""
        if self.real_events:
            return self.down.get(key, False)
        entry = self.last.get(key)
        if entry is None:
            return False
        return now - max(entry, self.activity) < self.cfg.hold_timeout
