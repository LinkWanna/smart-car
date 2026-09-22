"""Host <-> car wire protocol (mirror of the shared ``protocol/`` Rust crate).

Frame format::

    AA 55 CMD LEN PAYLOAD.. CHK      CHK = CMD ^ LEN ^ PAYLOAD..

Multi-byte integers are **little endian** (``struct`` format ``<``, matching
the firmware's ``to_le_bytes``/``from_le_bytes``). ``LEN <= 16`` and lengths
are **exact**: ``encode_request``/``decode_request`` (and the response
counterparts) reject anything else with :class:`ProtocolError`, mirroring the
firmware's ``Request::decode``/``Response::decode``.

The shared Rust crate (repo root ``protocol/``, used by both the firmware and
the SG2002 upper computer) is the single source of truth; the golden bytes in
``protocol/src/tests.rs`` and ``tools/tests/test_protocol.py`` pin the wire
format from both sides.
"""

from __future__ import annotations

import struct
from collections.abc import Iterator
from dataclasses import dataclass
from enum import IntEnum

# --- BLE GATT contract ---
NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> car (write)
TX_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # car -> host (notify)

#: Write/notify payload that always fits the minimum ATT MTU.
MAX_WRITE = 20

H1, H2 = 0xAA, 0x55
#: Header + checksum bytes around the payload.
FRAME_OVERHEAD = 5
#: Largest payload the firmware parser buffers.
MAX_PAYLOAD = 16


class ErrorCode(IntEnum):
    """NACK reasons."""

    WRONG_STATE = 0x01
    BAD_CHECKSUM = 0x02
    INVALID_PARAM = 0x03
    UNKNOWN_REQUEST = 0x04


class ProtocolError(ValueError):
    """协议层错误；``error_code`` 是线上对应的 NACK 原因码。"""

    def __init__(
        self, message: str, error_code: ErrorCode = ErrorCode.INVALID_PARAM
    ) -> None:
        super().__init__(message)
        self.error_code = error_code


class SysState(IntEnum):
    """Firmware state machine; order is the acceptance gate."""

    UNINIT = 0
    READY = 1
    RUNNING = 2


class ResponseType(IntEnum):
    """Car -> host responses."""

    ACK = 0x80
    NACK = 0x81
    STATUS = 0x91
    PID_DATA = 0x92


class RequestType(IntEnum):
    """Host -> car commands."""

    INIT = 0x01
    SET_SPEED = 0x10
    STOP = 0x11
    BRAKE = 0x12
    SET_SPEEDS = 0x13
    SET_PID = 0x14
    GET_PID = 0x15
    MOVE = 0x20
    ROTATE = 0x21
    HEARTBEAT = 0xFE
    RESET = 0xFF

    @property
    def payload_len(self) -> int:
        """Exact payload length, see the firmware protocol table."""
        if self in (RequestType.INIT, RequestType.HEARTBEAT, RequestType.RESET):
            return 0
        if self in (RequestType.STOP, RequestType.BRAKE, RequestType.GET_PID):
            return 1
        if self is RequestType.SET_SPEED:
            return 3
        if self is RequestType.SET_SPEEDS:
            return 4
        if self in (RequestType.MOVE, RequestType.ROTATE):
            return 6
        return 7

    @property
    def min_state(self) -> SysState | None:
        """Lowest firmware state that accepts this request (None = any)."""
        if self in (RequestType.INIT, RequestType.HEARTBEAT, RequestType.RESET):
            return None
        return SysState.READY


class MotorId(IntEnum):
    """``motor`` byte: 0 = left, 1 = right."""

    LEFT = 0
    RIGHT = 1


class MotorTarget(IntEnum):
    """``target`` byte of STOP/BRAKE: 0/1/2 = left/right/both."""

    LEFT = 0
    RIGHT = 1
    BOTH = 2


class MoveDir(IntEnum):
    """``dir`` byte of MOVE."""

    FORWARD = 0
    BACKWARD = 1


class RotateDir(IntEnum):
    """``dir`` byte of ROTATE."""

    LEFT = 0
    RIGHT = 1


SYS_STATE_NAMES = {
    SysState.UNINIT: "Uninit",
    SysState.READY: "Ready",
    SysState.RUNNING: "Running",
}


# --- frame layer (mirrors protocol/src/frame.rs) ---


def checksum(cmd: int, payload: bytes) -> int:
    """XOR checksum over command, length and payload."""
    chk = int(cmd) ^ len(payload)
    for byte in payload:
        chk ^= byte
    return chk & 0xFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """Encode one frame; raises if the payload exceeds ``MAX_PAYLOAD``."""
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload longer than {MAX_PAYLOAD} bytes")
    return bytes([H1, H2, int(cmd), len(payload), *payload, checksum(cmd, payload)])


def chunks(data: bytes, size: int = MAX_WRITE) -> Iterator[bytes]:
    """Split a frame into BLE write/notify sized pieces."""
    for offset in range(0, len(data), size):
        yield data[offset : offset + size]


@dataclass(frozen=True)
class Frame:
    """A complete, checksum-valid frame produced by :class:`FrameParser`."""

    cmd: int
    payload: bytes

    def to_bytes(self) -> bytes:
        return build_frame(self.cmd, self.payload)


@dataclass(frozen=True)
class ChecksumFail:
    """The frame's checksum did not match; ``cmd`` is the frame's command byte."""

    cmd: int


@dataclass(frozen=True)
class TooLong:
    """``LEN > MAX_PAYLOAD``; the frame was dropped and the parser resynced."""

    cmd: int
    length: int


RxEvent = Frame | ChecksumFail | TooLong


class _State(IntEnum):
    H1 = 0
    H2 = 1
    CMD = 2
    LEN = 3
    PAYLOAD = 4
    CHK = 5


class FrameParser:
    """Byte-at-a-time parser mirroring the firmware ``RxParser``."""

    def __init__(self) -> None:
        self._state = _State.H1
        self._cmd = 0
        self._len = 0
        self._buf = bytearray()

    def feed_byte(self, byte: int) -> RxEvent | None:
        """Advance the state machine by one byte."""
        if self._state is _State.H1:
            if byte == H1:
                self._state = _State.H2
        elif self._state is _State.H2:
            if byte == H2:
                self._state = _State.CMD
            elif byte == H1:
                # A second H1 restarts the header hunt instead of eating it.
                self._state = _State.H2
            else:
                self._state = _State.H1
        elif self._state is _State.CMD:
            self._cmd = byte
            self._state = _State.LEN
        elif self._state is _State.LEN:
            self._len = byte
            self._buf.clear()
            if byte > MAX_PAYLOAD:
                self._state = _State.H1
                return TooLong(self._cmd, byte)
            self._state = _State.CHK if byte == 0 else _State.PAYLOAD
        elif self._state is _State.PAYLOAD:
            self._buf.append(byte)
            if len(self._buf) >= self._len:
                self._state = _State.CHK
        else:  # _State.CHK
            self._state = _State.H1
            payload = bytes(self._buf)
            if checksum(self._cmd, payload) == byte:
                return Frame(self._cmd, payload)
            return ChecksumFail(self._cmd)
        return None

    def feed(self, data: bytes) -> list[RxEvent]:
        """Consume bytes, returning every event seen (in order)."""
        events: list[RxEvent] = []
        for byte in data:
            event = self.feed_byte(byte)
            if event is not None:
                events.append(event)
        return events


# --- requests (host -> car, mirrors protocol/src/request.rs) ---


@dataclass(frozen=True)
class Init:
    """Zero the encoders and PIDs, enter Ready."""


@dataclass(frozen=True)
class SetSpeed:
    """One wheel target in percent; the firmware clamps to +/-100."""

    motor: MotorId
    speed: int


@dataclass(frozen=True)
class Stop:
    """Coast the selected motor(s)."""

    target: MotorTarget


@dataclass(frozen=True)
class Brake:
    """Short-brake the selected motor(s) until the next command."""

    target: MotorTarget


@dataclass(frozen=True)
class SetSpeeds:
    """Both wheel targets in percent; the firmware clamps to +/-100."""

    left: int
    right: int


@dataclass(frozen=True)
class SetPid:
    """Gains are x100 fixed point (1.50 -> 150)."""

    motor: MotorId
    kp: int
    ki: int
    kd: int


@dataclass(frozen=True)
class GetPid:
    """Read back the gains as PID_DATA."""

    motor: MotorId


@dataclass(frozen=True)
class Move:
    """Closed-loop straight move; ``distance_mm`` > 0, ``speed`` 1..100."""

    dir: MoveDir
    speed: int
    distance_mm: int


@dataclass(frozen=True)
class Rotate:
    """Closed-loop in-place turn; ``tenths_deg`` > 0 (90 deg -> 900)."""

    dir: RotateDir
    speed: int
    tenths_deg: int


@dataclass(frozen=True)
class Heartbeat:
    """Status poll: replies STATUS (used by the HUD and move.py)."""


@dataclass(frozen=True)
class Reset:
    """Back to Uninit."""


Request = (
    Init
    | SetSpeed
    | Stop
    | Brake
    | SetSpeeds
    | SetPid
    | GetPid
    | Move
    | Rotate
    | Heartbeat
    | Reset
)

_REQUEST_TYPE_OF: dict[type, RequestType] = {
    Init: RequestType.INIT,
    SetSpeed: RequestType.SET_SPEED,
    Stop: RequestType.STOP,
    Brake: RequestType.BRAKE,
    SetSpeeds: RequestType.SET_SPEEDS,
    SetPid: RequestType.SET_PID,
    GetPid: RequestType.GET_PID,
    Move: RequestType.MOVE,
    Rotate: RequestType.ROTATE,
    Heartbeat: RequestType.HEARTBEAT,
    Reset: RequestType.RESET,
}


def _request_type(value: int | RequestType) -> RequestType:
    try:
        return RequestType(value)
    except ValueError:
        raise ProtocolError(
            f"unknown request {int(value):#04x}", ErrorCode.UNKNOWN_REQUEST
        ) from None


def _response_type(value: int | ResponseType) -> ResponseType:
    try:
        return ResponseType(value)
    except ValueError:
        raise ProtocolError(f"unknown response {int(value):#04x}") from None


def _enum(cls, value: int, what: str):
    try:
        return cls(value)
    except ValueError:
        raise ProtocolError(f"invalid {what}: {int(value)}") from None


def _check_len(request_type: RequestType, payload: bytes) -> None:
    expected = request_type.payload_len
    if len(payload) != expected:
        raise ProtocolError(
            f"{request_type.name.lower()} payload length {len(payload)} != {expected}"
        )


def _check_i16(value: int, what: str) -> int:
    if not -0x8000 <= value <= 0x7FFF:
        raise ProtocolError(f"{what} out of i16 range: {value}")
    return value


def _check_move(speed: int, target: int, what: str) -> None:
    if not 1 <= speed <= 100:
        raise ProtocolError(f"speed out of 1..100: {speed}")
    if not 0 < target <= 0x7FFFFFFF:
        raise ProtocolError(f"{what} must be > 0: {target}")


def encode_request(request: Request) -> bytes:
    """Encode one command, validating lengths and values."""
    match request:
        case Init() | Heartbeat() | Reset():
            return build_frame(_REQUEST_TYPE_OF[type(request)])
        case SetSpeed(motor=motor, speed=speed):
            return build_frame(
                RequestType.SET_SPEED,
                bytes([_enum(MotorId, motor, "motor")])
                + struct.pack("<h", _check_i16(speed, "speed")),
            )
        case Stop(target=target):
            return build_frame(
                RequestType.STOP, bytes([_enum(MotorTarget, target, "target")])
            )
        case Brake(target=target):
            return build_frame(
                RequestType.BRAKE, bytes([_enum(MotorTarget, target, "target")])
            )
        case SetSpeeds(left=left, right=right):
            return build_frame(
                RequestType.SET_SPEEDS,
                struct.pack(
                    "<hh", _check_i16(left, "left"), _check_i16(right, "right")
                ),
            )
        case SetPid(motor=motor, kp=kp, ki=ki, kd=kd):
            payload = bytes([_enum(MotorId, motor, "motor")]) + struct.pack(
                "<hhh",
                _check_i16(kp, "kp"),
                _check_i16(ki, "ki"),
                _check_i16(kd, "kd"),
            )
            return build_frame(RequestType.SET_PID, payload)
        case GetPid(motor=motor):
            return build_frame(
                RequestType.GET_PID, bytes([_enum(MotorId, motor, "motor")])
            )
        case Move(dir=direction, speed=speed, distance_mm=distance):
            _check_move(speed, distance, "distance_mm")
            payload = bytes([_enum(MoveDir, direction, "dir"), speed]) + struct.pack(
                "<i", distance
            )
            return build_frame(RequestType.MOVE, payload)
        case Rotate(dir=direction, speed=speed, tenths_deg=tenths):
            _check_move(speed, tenths, "tenths_deg")
            payload = bytes([_enum(RotateDir, direction, "dir"), speed]) + struct.pack(
                "<i", tenths
            )
            return build_frame(RequestType.ROTATE, payload)
    raise ProtocolError(f"unsupported request {request!r}")


def decode_request(request_type: int, payload: bytes) -> Request:
    """Decode and validate one request."""
    request_type = _request_type(request_type)
    _check_len(request_type, payload)
    match request_type:
        case RequestType.INIT:
            return Init()
        case RequestType.SET_SPEED:
            return SetSpeed(
                _enum(MotorId, payload[0], "motor"),
                struct.unpack("<h", payload[1:3])[0],
            )
        case RequestType.STOP:
            return Stop(_enum(MotorTarget, payload[0], "target"))
        case RequestType.BRAKE:
            return Brake(_enum(MotorTarget, payload[0], "target"))
        case RequestType.SET_SPEEDS:
            left, right = struct.unpack("<hh", payload)
            return SetSpeeds(left, right)
        case RequestType.SET_PID:
            kp, ki, kd = struct.unpack("<hhh", payload[1:7])
            return SetPid(_enum(MotorId, payload[0], "motor"), kp, ki, kd)
        case RequestType.GET_PID:
            return GetPid(_enum(MotorId, payload[0], "motor"))
        case RequestType.MOVE:
            direction = _enum(MoveDir, payload[0], "dir")
            speed, distance = payload[1], struct.unpack("<i", payload[2:6])[0]
            _check_move(speed, distance, "distance_mm")
            return Move(direction, speed, distance)
        case RequestType.ROTATE:
            direction = _enum(RotateDir, payload[0], "dir")
            speed, tenths = payload[1], struct.unpack("<i", payload[2:6])[0]
            _check_move(speed, tenths, "tenths_deg")
            return Rotate(direction, speed, tenths)
        case RequestType.HEARTBEAT:
            return Heartbeat()
        case RequestType.RESET:
            return Reset()
    raise ProtocolError(f"unsupported request {int(request_type):#04x}")


# --- responses (car -> host, mirrors protocol/src/response.rs) ---


@dataclass(frozen=True)
class Ack:
    """``cmd`` is the echoed request command byte."""

    cmd: int


@dataclass(frozen=True)
class Nack:
    cmd: int
    error: ErrorCode


@dataclass(frozen=True)
class Status:
    """STATUS payload; ``dist_result`` latches 1 when a goal is reached."""

    sys: SysState
    rpm: tuple[int, int]
    dist_active: bool
    dist_result: int

    @classmethod
    def parse(cls, data: bytes) -> Status:
        if len(data) != 7:
            raise ProtocolError(f"status payload length {len(data)} != 7")
        rpm0, rpm1 = struct.unpack("<hh", data[1:5])
        return cls(
            _enum(SysState, data[0], "sys"),
            (rpm0, rpm1),
            bool(data[5]),
            data[6],
        )

    @property
    def done(self) -> bool:
        """A closed-loop goal was reached (latched until the next Move/Rotate)."""
        return self.dist_result == 1


@dataclass(frozen=True)
class PidData:
    """Gains are x100 fixed point."""

    kp: int
    ki: int
    kd: int

    @classmethod
    def parse(cls, data: bytes) -> PidData:
        if len(data) != 6:
            raise ProtocolError(f"pid payload length {len(data)} != 6")
        kp, ki, kd = struct.unpack("<hhh", data)
        return cls(kp, ki, kd)


Response = Ack | Nack | Status | PidData


def encode_response(response: Response) -> bytes:
    """Encode one response (used by tests and by a host-side echo/relay)."""
    match response:
        case Ack(cmd=cmd):
            return build_frame(ResponseType.ACK, bytes([cmd]))
        case Nack(cmd=cmd, error=error):
            return build_frame(ResponseType.NACK, bytes([cmd, int(error)]))
        case Status():
            payload = bytes([int(response.sys)]) + struct.pack("<hh", *response.rpm)
            payload += bytes([int(response.dist_active), response.dist_result])
            return build_frame(ResponseType.STATUS, payload)
        case PidData():
            return build_frame(
                ResponseType.PID_DATA,
                struct.pack("<hhh", response.kp, response.ki, response.kd),
            )
    raise ProtocolError(f"unsupported response {response!r}")


def decode_response(response_type: int, payload: bytes) -> Response:
    """Decode one response."""
    response_type = _response_type(response_type)
    match response_type:
        case ResponseType.ACK:
            if len(payload) != 1:
                raise ProtocolError(f"ack payload length {len(payload)} != 1")
            return Ack(payload[0])
        case ResponseType.NACK:
            if len(payload) != 2:
                raise ProtocolError(f"nack payload length {len(payload)} != 2")
            return Nack(payload[0], _enum(ErrorCode, payload[1], "error"))
        case ResponseType.STATUS:
            return Status.parse(payload)
        case ResponseType.PID_DATA:
            return PidData.parse(payload)
    raise ProtocolError(f"unsupported response {int(response_type):#04x}")


# --- display helpers ---


def command_name(cmd: int) -> str:
    """CLI style name of a command (``init``, ``set_speed``)."""
    try:
        return RequestType(cmd).name.lower()
    except ValueError:
        return f"0x{cmd:02X}"


def sys_state_name(value: int | SysState) -> str:
    return SYS_STATE_NAMES.get(value, f"state{int(value)}")  # type: ignore[arg-type]


def describe_frame(cmd: int, payload: bytes) -> str:
    """Human readable rendering of one frame (either direction)."""
    for decode in (decode_request, decode_response):
        try:
            return repr(decode(cmd, payload))
        except ProtocolError:
            continue
    return f"0x{cmd:02X} {payload.hex(' ')}"
