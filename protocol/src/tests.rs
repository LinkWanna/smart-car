//! 协议回归测试：黄金字节（小端/长度/校验和钉死）、往返编解码与错误路径。
//!
//! 黄金字节与 `esp32c3-slave/tools/tests/test_protocol.py` 中的
//! `test_little_endian_is_pinned` 同源，任何一端字节序/长度改动都会在这里暴露。

use super::*;
use crate::frame::{checksum, MAX_PAYLOAD};
use core::fmt::Write;

const ALL_REQUESTS: [Request; 12] = [
    Request::Init,
    Request::SetSpeed {
        motor: MotorId::Left,
        speed: 50,
    },
    Request::SetSpeed {
        motor: MotorId::Right,
        speed: -100,
    },
    Request::Stop {
        target: MotorTarget::Both,
    },
    Request::Brake {
        target: MotorTarget::Left,
    },
    Request::SetSpeeds {
        left: 40,
        right: -40,
    },
    Request::SetPid {
        motor: MotorId::Right,
        kp: 150,
        ki: 20,
        kd: -5,
    },
    Request::GetPid {
        motor: MotorId::Left,
    },
    Request::Move {
        dir: MoveDir::Forward,
        speed: 50,
        distance_mm: 500,
    },
    Request::Rotate {
        dir: RotateDir::Left,
        speed: 40,
        tenths_deg: 900,
    },
    Request::Heartbeat,
    Request::Reset,
];

fn bytes(frame: &Frame) -> &[u8] {
    frame.as_bytes()
}

/// `no_std` 下没有 `ToString`，用定长缓冲捕获 `Display` 输出。
struct Rendered {
    buf: [u8; 64],
    len: usize,
}

impl Rendered {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap()
    }
}

impl core::fmt::Write for Rendered {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.len + s.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// 黄金字节：改协议必须同步改 Python 镜像与这些常量。
#[test]
fn golden_request_bytes() {
    let cases: [(Request, &[u8]); 11] = [
        (Request::Init, &[0xAA, 0x55, 0x01, 0x00, 0x01]),
        (
            Request::SetSpeed {
                motor: MotorId::Left,
                speed: 50,
            },
            &[0xAA, 0x55, 0x10, 0x03, 0x00, 0x32, 0x00, 0x21],
        ),
        (
            Request::SetSpeed {
                motor: MotorId::Right,
                speed: -100,
            },
            &[0xAA, 0x55, 0x10, 0x03, 0x01, 0x9C, 0xFF, 0x71],
        ),
        (
            Request::Stop {
                target: MotorTarget::Both,
            },
            &[0xAA, 0x55, 0x11, 0x01, 0x02, 0x12],
        ),
        (
            Request::Brake {
                target: MotorTarget::Left,
            },
            &[0xAA, 0x55, 0x12, 0x01, 0x00, 0x13],
        ),
        (
            Request::SetSpeeds {
                left: 40,
                right: -40,
            },
            &[0xAA, 0x55, 0x13, 0x04, 0x28, 0x00, 0xD8, 0xFF, 0x18],
        ),
        (
            Request::SetPid {
                motor: MotorId::Right,
                kp: 150,
                ki: 20,
                kd: -5,
            },
            &[
                0xAA, 0x55, 0x14, 0x07, 0x01, 0x96, 0x00, 0x14, 0x00, 0xFB, 0xFF, 0x94,
            ],
        ),
        (
            Request::GetPid {
                motor: MotorId::Left,
            },
            &[0xAA, 0x55, 0x15, 0x01, 0x00, 0x14],
        ),
        (
            Request::Move {
                dir: MoveDir::Forward,
                speed: 50,
                distance_mm: 500,
            },
            &[
                0xAA, 0x55, 0x20, 0x06, 0x00, 0x32, 0xF4, 0x01, 0x00, 0x00, 0xE1,
            ],
        ),
        (
            Request::Rotate {
                dir: RotateDir::Left,
                speed: 40,
                tenths_deg: 900,
            },
            &[
                0xAA, 0x55, 0x21, 0x06, 0x00, 0x28, 0x84, 0x03, 0x00, 0x00, 0x88,
            ],
        ),
        (Request::Heartbeat, &[0xAA, 0x55, 0xFE, 0x00, 0xFE]),
    ];
    for (request, expected) in cases {
        assert_eq!(
            bytes(&request.to_frame().unwrap()),
            expected,
            "request {request:?}"
        );
    }
    assert_eq!(
        bytes(&Request::Reset.to_frame().unwrap()),
        &[0xAA, 0x55, 0xFF, 0x00, 0xFF]
    );
}

#[test]
fn golden_response_bytes() {
    let status = Status {
        sys: SysState::Running,
        rpm: [120, -120],
        dist_active: true,
        dist_result: 0,
    };
    let cases: [(Response, &[u8]); 4] = [
        (
            Response::Ack { cmd: 0x01 },
            &[0xAA, 0x55, 0x80, 0x01, 0x01, 0x80],
        ),
        (
            Response::Nack {
                cmd: 0x10,
                error: ErrorCode::InvalidParam,
            },
            &[0xAA, 0x55, 0x81, 0x02, 0x10, 0x03, 0x90],
        ),
        (
            Response::Status(status),
            &[
                0xAA, 0x55, 0x91, 0x07, 0x02, 0x78, 0x00, 0x88, 0xFF, 0x01, 0x00, 0x9A,
            ],
        ),
        (
            Response::PidData(PidData {
                kp: 150,
                ki: 20,
                kd: 0,
            }),
            &[
                0xAA, 0x55, 0x92, 0x06, 0x96, 0x00, 0x14, 0x00, 0x00, 0x00, 0x16,
            ],
        ),
    ];
    for (response, expected) in cases {
        assert_eq!(
            bytes(&response.to_frame().unwrap()),
            expected,
            "response {response:?}"
        );
    }
}

#[test]
fn requests_roundtrip() {
    for request in ALL_REQUESTS {
        let frame = request.to_frame().unwrap();
        assert_eq!(
            frame.len(),
            request.request_type().payload_len() as usize + 5
        );
        let decoded = Request::decode(frame.cmd(), frame.payload()).unwrap();
        assert_eq!(decoded, request);
    }
}

#[test]
fn responses_roundtrip() {
    let responses = [
        Response::Ack { cmd: 0x13 },
        Response::Nack {
            cmd: 0x10,
            error: ErrorCode::WrongState,
        },
        Response::Status(Status {
            sys: SysState::Ready,
            rpm: [10, -10],
            dist_active: false,
            dist_result: 0,
        }),
        Response::Status(Status {
            sys: SysState::Uninit,
            rpm: [0, 0],
            dist_active: false,
            dist_result: 1,
        }),
        Response::PidData(PidData {
            kp: -150,
            ki: 0,
            kd: 32767,
        }),
    ];
    for response in responses {
        let frame = response.to_frame().unwrap();
        let decoded = Response::decode(frame.cmd(), frame.payload()).unwrap();
        assert_eq!(decoded, response);
    }
}

#[test]
fn exact_lengths_are_enforced() {
    // 多一字节少一字节都是 BadLength（Init/Heartbeat 必须是 0）。
    assert_eq!(
        Request::decode(0x01, &[0x00]),
        Err(ProtocolError::BadLength {
            expected: 0,
            got: 1
        })
    );
    assert_eq!(
        Request::decode(0x13, &[0x00, 0x32, 0xFF]),
        Err(ProtocolError::BadLength {
            expected: 4,
            got: 3
        })
    );
    assert_eq!(
        Request::decode(0x91, &[]).unwrap_err(),
        ProtocolError::UnknownRequest(0x91)
    );
}

#[test]
fn unknown_and_invalid_values() {
    assert_eq!(
        Request::decode(0x42, &[]),
        Err(ProtocolError::UnknownRequest(0x42))
    );
    assert_eq!(
        Request::decode(0x11, &[0x03]),
        Err(ProtocolError::InvalidValue)
    );
    assert_eq!(
        Request::decode(0x15, &[0x02]),
        Err(ProtocolError::InvalidValue)
    );
    // Move/Rotate：speed 1..=100 且目标 > 0，编解码两侧都校验。
    let move_frame = |speed: u8, target: i32| {
        let mut payload = [0u8; 6];
        payload[0] = MoveDir::Forward as u8;
        payload[1] = speed;
        payload[2..6].copy_from_slice(&target.to_le_bytes());
        Request::decode(0x20, &payload)
    };
    assert_eq!(move_frame(0, 500), Err(ProtocolError::InvalidValue));
    assert_eq!(move_frame(101, 500), Err(ProtocolError::InvalidValue));
    assert_eq!(move_frame(50, 0), Err(ProtocolError::InvalidValue));
    assert!(move_frame(100, 1).is_ok());

    assert_eq!(
        Request::Move {
            dir: MoveDir::Forward,
            speed: 0,
            distance_mm: 500
        }
        .to_frame(),
        Err(ProtocolError::InvalidValue)
    );
    assert_eq!(
        Request::Rotate {
            dir: RotateDir::Left,
            speed: 40,
            tenths_deg: 0
        }
        .to_frame(),
        Err(ProtocolError::InvalidValue)
    );
    assert_eq!(
        Request::decode(0x21, &[0x02, 0x28, 0x84, 0x03, 0x00, 0x00]),
        Err(ProtocolError::InvalidValue)
    );
}

#[test]
fn error_code_mapping() {
    assert_eq!(
        ProtocolError::UnknownRequest(0x42).error_code(),
        ErrorCode::UnknownRequest
    );
    assert_eq!(
        ProtocolError::BadLength {
            expected: 1,
            got: 2
        }
        .error_code(),
        ErrorCode::InvalidParam
    );
    assert_eq!(
        ProtocolError::InvalidValue.error_code(),
        ErrorCode::InvalidParam
    );
    assert_eq!(
        ProtocolError::PayloadTooLong { len: 17 }.error_code(),
        ErrorCode::InvalidParam
    );
    assert_eq!(
        ProtocolError::UnknownResponse(0x90).error_code(),
        ErrorCode::InvalidParam
    );
}

#[test]
fn frame_rejects_oversized_payload() {
    assert_eq!(
        Frame::new(0x13, &[0u8; MAX_PAYLOAD + 1]).unwrap_err(),
        ProtocolError::PayloadTooLong {
            len: MAX_PAYLOAD + 1
        }
    );
    assert!(Frame::new(0x13, &[0u8; MAX_PAYLOAD]).is_ok());
}

#[test]
fn status_is_strict() {
    // 旧版 5 字节 Status 不再接受，长度/状态值都必须精确。
    assert!(Status::decode(&[0x01, 0x00, 0x0A, 0xFF, 0xF6]).is_err());
    assert!(Status::decode(&[0x03, 0, 0, 0, 0, 0, 0]).is_err());
    let done = Status {
        sys: SysState::Running,
        rpm: [0, 0],
        dist_active: false,
        dist_result: 1,
    };
    let encoded = Response::Status(done).to_frame().unwrap();
    assert!(Status::decode(encoded.payload()).unwrap().done());
}

#[test]
fn parser_decodes_a_stream_and_resyncs() {
    let mut stream = [0u8; 128];
    let mut n = 0;
    // 垃圾字节 + 半个帧头都不能产生事件（但不能停留在半帧里）。
    for b in [0x00, 0xAA, 0x00, 0xAA, 0x00] {
        stream[n] = b;
        n += 1;
    }
    for frame in [
        Response::Ack { cmd: 0x01 }.to_frame().unwrap(),
        Response::Nack {
            cmd: 0x10,
            error: ErrorCode::WrongState,
        }
        .to_frame()
        .unwrap(),
        Response::Status(Status {
            sys: SysState::Running,
            rpm: [11, -11],
            dist_active: false,
            dist_result: 1,
        })
        .to_frame()
        .unwrap(),
    ] {
        stream[n..n + frame.len()].copy_from_slice(frame.as_bytes());
        n += frame.len();
    }

    let mut parser = RxParser::new();
    let mut events = 0;
    for &b in &stream[..n] {
        match parser.feed(b) {
            RxEvent::Frame { cmd, payload } => {
                events += 1;
                match Response::decode(cmd, payload).unwrap() {
                    Response::Ack { cmd } => assert_eq!(cmd, 0x01),
                    Response::Nack { error, .. } => assert_eq!(error, ErrorCode::WrongState),
                    Response::Status(status) => {
                        assert_eq!(status.rpm, [11, -11]);
                        assert_eq!(status.sys, SysState::Running);
                        assert!(status.done());
                    }
                    Response::PidData(_) => panic!("unexpected PidData"),
                }
            }
            RxEvent::None => {}
            other => panic!("unexpected event {other:?}"),
        }
    }
    assert_eq!(events, 3);
}

#[test]
fn parser_reports_bad_checksum_and_recovers() {
    let mut parser = RxParser::new();
    for b in [0xAA, 0x55, 0x01, 0x00] {
        assert_eq!(parser.feed(b), RxEvent::None);
    }
    assert_eq!(parser.feed(0x03), RxEvent::ChecksumFail { cmd: 0x01 });

    // 校验错后立即复位，下一帧正常。
    let heartbeat = Request::Heartbeat.to_frame().unwrap();
    let mut last = RxEvent::None;
    for &b in heartbeat.as_bytes() {
        last = parser.feed(b);
    }
    assert_eq!(last, RxEvent::Frame { cmd: 0xFE, payload: &[] });
}

#[test]
fn parser_drops_too_long_frames_and_resyncs() {
    let mut parser = RxParser::new();
    for b in [0xAA, 0x55, 0x10] {
        assert_eq!(parser.feed(b), RxEvent::None);
    }
    assert_eq!(
        parser.feed(0x11),
        RxEvent::TooLong {
            cmd: 0x10,
            len: 0x11
        }
    );

    // 超长帧被丢弃后紧接着的一帧应当被完整解析（含 LEN = MAX_PAYLOAD 的边界）。
    let mut payload = [0u8; MAX_PAYLOAD];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = i as u8;
    }
    let frame = Frame::new(0x10, &payload).unwrap();
    let mut last = RxEvent::None;
    for &b in frame.as_bytes() {
        last = parser.feed(b);
    }
    assert_eq!(
        last,
        RxEvent::Frame {
            cmd: 0x10,
            payload: payload.as_slice()
        }
    );
}

#[test]
fn checksum_matches_the_wire_rule() {
    assert_eq!(checksum(0x01, &[]), 0x01);
    assert_eq!(checksum(0x20, &[0x00, 0x32, 0xF4, 0x01, 0x00, 0x00]), 0xE1);
    let frame = Request::Heartbeat.to_frame().unwrap();
    assert_eq!(frame.as_bytes(), &[0xAA, 0x55, 0xFE, 0x00, 0xFE]);
}

#[test]
fn display_names_match_the_docs() {
    let mut out = Rendered::new();
    write!(
        out,
        "{}/{}",
        RequestType::SetSpeeds,
        ResponseType::PidData
    )
    .unwrap();
    assert_eq!(out.as_str(), "SetSpeeds/PidData");

    let mut out = Rendered::new();
    write!(
        out,
        "{}/{}/{}/{}",
        ErrorCode::UnknownRequest,
        SysState::Ready,
        MoveDir::Backward,
        RotateDir::Left
    )
    .unwrap();
    assert_eq!(out.as_str(), "UnknownRequest/Ready/Backward/Left");

    let mut out = Rendered::new();
    write!(out, "{}/{}", MotorId::Right, MotorTarget::Both).unwrap();
    assert_eq!(out.as_str(), "Right/Both");

    let mut out = Rendered::new();
    write!(
        out,
        "{}",
        ProtocolError::BadLength {
            expected: 3,
            got: 1
        }
    )
    .unwrap();
    assert_eq!(out.as_str(), "负载长度 1 != 3");
}
