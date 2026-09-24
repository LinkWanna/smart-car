//! 应答（车 → 上位机）的类型化负载：编码供下位机使用，解码供上位机与测试使用。
//!
//! 与请求一样，多字节字段一律小端。

use super::ProtocolError;
use super::frame::{Frame, MAX_PAYLOAD};
use super::types::{ErrorCode, ResponseType, SysState};

/// `Status` 负载的精确长度。
pub const STATUS_LEN: usize = 7;
/// `PidData` 负载的精确长度。
pub const PID_DATA_LEN: usize = 6;

/// 状态快照，`Heartbeat` 的应答。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// 当前系统状态。
    pub sys: SysState,
    /// 双轮实测转速（rpm，左轮已取正方向）。
    pub rpm: [i16; 2],
    /// 闭环 Move/Rotate 是否运行中。
    pub dist_active: bool,
    /// 0 = 无/运行中，1 = 到达目标（下一条 Move/Rotate 或 Init/Reset 清零）。
    pub dist_result: u8,
}

impl Status {
    pub fn decode(payload: &[u8]) -> Result<Self, ProtocolError> {
        if payload.len() != STATUS_LEN {
            return Err(ProtocolError::BadLength {
                expected: STATUS_LEN as u8,
                got: payload.len(),
            });
        }
        let sys = SysState::from_u8(payload[0]).ok_or(ProtocolError::InvalidValue)?;
        Ok(Self {
            sys,
            rpm: [
                i16::from_le_bytes([payload[1], payload[2]]),
                i16::from_le_bytes([payload[3], payload[4]]),
            ],
            dist_active: payload[5] != 0,
            dist_result: payload[6],
        })
    }

    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> Result<usize, ProtocolError> {
        out[0] = self.sys.as_u8();
        out[1..3].copy_from_slice(&self.rpm[0].to_le_bytes());
        out[3..5].copy_from_slice(&self.rpm[1].to_le_bytes());
        out[5] = self.dist_active as u8;
        out[6] = self.dist_result;
        Ok(STATUS_LEN)
    }

    /// 闭环目标是否已到达（`dist_result == 1`）。
    pub const fn done(&self) -> bool {
        self.dist_result == 1
    }
}

/// PID 增益快照，`GetPid` 的应答；三个值都是 ×100 的定点数。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidData {
    pub kp: i16,
    pub ki: i16,
    pub kd: i16,
}

impl PidData {
    pub fn decode(payload: &[u8]) -> Result<Self, ProtocolError> {
        if payload.len() != PID_DATA_LEN {
            return Err(ProtocolError::BadLength {
                expected: PID_DATA_LEN as u8,
                got: payload.len(),
            });
        }
        Ok(Self {
            kp: i16::from_le_bytes([payload[0], payload[1]]),
            ki: i16::from_le_bytes([payload[2], payload[3]]),
            kd: i16::from_le_bytes([payload[4], payload[5]]),
        })
    }

    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> Result<usize, ProtocolError> {
        out[0..2].copy_from_slice(&self.kp.to_le_bytes());
        out[2..4].copy_from_slice(&self.ki.to_le_bytes());
        out[4..6].copy_from_slice(&self.kd.to_le_bytes());
        Ok(PID_DATA_LEN)
    }
}

/// 一条应答及其参数。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    /// 命令被接受，`cmd` 是回显的请求命令号。
    Ack {
        cmd: u8,
    },
    /// 命令被拒绝。
    Nack {
        cmd: u8,
        error: ErrorCode,
    },
    Status(Status),
    PidData(PidData),
}

impl Response {
    /// 对应的应答类型（线上的应答号）。
    pub const fn response_type(&self) -> ResponseType {
        match self {
            Self::Ack { .. } => ResponseType::Ack,
            Self::Nack { .. } => ResponseType::Nack,
            Self::Status(_) => ResponseType::Status,
            Self::PidData(_) => ResponseType::PidData,
        }
    }

    /// 解码一条应答。
    pub fn decode(response_type: u8, payload: &[u8]) -> Result<Self, ProtocolError> {
        let response_type = ResponseType::from_u8(response_type)
            .ok_or(ProtocolError::UnknownResponse(response_type))?;
        let response = match response_type {
            ResponseType::Ack => {
                if payload.len() != 1 {
                    return Err(ProtocolError::BadLength {
                        expected: 1,
                        got: payload.len(),
                    });
                }
                Self::Ack { cmd: payload[0] }
            }
            ResponseType::Nack => {
                if payload.len() != 2 {
                    return Err(ProtocolError::BadLength {
                        expected: 2,
                        got: payload.len(),
                    });
                }
                Self::Nack {
                    cmd: payload[0],
                    error: ErrorCode::from_u8(payload[1]).ok_or(ProtocolError::InvalidValue)?,
                }
            }
            ResponseType::Status => Self::Status(Status::decode(payload)?),
            ResponseType::PidData => Self::PidData(PidData::decode(payload)?),
        };
        Ok(response)
    }

    /// 编码负载，返回负载长度。
    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> Result<usize, ProtocolError> {
        match self {
            Self::Ack { cmd } => {
                out[0] = *cmd;
                Ok(1)
            }
            Self::Nack { cmd, error } => {
                out[0] = *cmd;
                out[1] = error.as_u8();
                Ok(2)
            }
            Self::Status(status) => status.encode(out),
            Self::PidData(pid) => pid.encode(out),
        }
    }

    /// 编码成完整帧（含帧头与校验和）。
    pub fn to_frame(&self) -> Result<Frame, ProtocolError> {
        let mut payload = [0u8; MAX_PAYLOAD];
        let len = self.encode(&mut payload)?;
        Frame::new(self.response_type().as_u8(), &payload[..len])
    }
}

/// 应答的规范文本
impl core::fmt::Display for Response {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Ack { cmd } => {
                f.write_str("ACK ")?;
                fmt_request_name(f, *cmd)
            }
            Self::Nack { cmd, error } => {
                f.write_str("NACK ")?;
                fmt_request_name(f, *cmd)?;
                write!(f, " {error}")
            }
            Self::Status(status) => write!(
                f,
                "Status {} rpm=({}, {}) dist_active={} dist_result={}",
                status.sys,
                status.rpm[0],
                status.rpm[1],
                status.dist_active as u8,
                status.dist_result
            ),
            Self::PidData(pid) => write!(
                f,
                "PidData kp={:.2} ki={:.2} kd={:.2}",
                pid.kp as f32 / 100.0,
                pid.ki as f32 / 100.0,
                pid.kd as f32 / 100.0
            ),
        }
    }
}

/// 回显命令号的可读名字（未知命令显示十六进制）。
fn fmt_request_name(f: &mut core::fmt::Formatter<'_>, cmd: u8) -> core::fmt::Result {
    match super::types::RequestType::from_u8(cmd) {
        Some(ty) => write!(f, "{ty}"),
        None => write!(f, "0x{cmd:02X}"),
    }
}
