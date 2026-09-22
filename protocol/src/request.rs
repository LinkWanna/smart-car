//! 命令（上位机 → 车）的类型化负载：解码带完整校验，编码供上位机与测试使用。
//!
//! 每个变体对应 crate 级文档表格里的一行；`decode` 负责长度与取值的全部校验，
//! 下位机 dispatch 只需要匹配变体并落实动作，不再碰字节。
//!
//! 多字节字段一律小端，直接把 `to_le_bytes`/`from_le_bytes` 的结果搬进搬出负载。

use super::ProtocolError;
use super::frame::{Frame, MAX_PAYLOAD};
use super::types::{MotorId, MotorTarget, MoveDir, RequestType, RotateDir};

/// 一条命令及其参数。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    /// 编码器清零、PID 复位，进 `Ready`。
    Init,
    /// 单轮目标速度；`speed` 单位是百分比，固件按 ±100 截断。
    SetSpeed { motor: MotorId, speed: i16 },
    /// 滑行（coast）。
    Stop { target: MotorTarget },
    /// 短接制动，保持到下条指令。
    Brake { target: MotorTarget },
    /// 双轮目标速度；同样按 ±100 截断。
    SetSpeeds { left: i16, right: i16 },
    /// 改增益；kp/ki/kd 是 ×100 的定点值（1.50 → 150）。
    SetPid {
        motor: MotorId,
        kp: i16,
        ki: i16,
        kd: i16,
    },
    /// 读增益，应答 [`super::PidData`]。
    GetPid { motor: MotorId },
    /// 闭环直行，`distance_mm` > 0。
    Move {
        dir: MoveDir,
        speed: u8,
        distance_mm: u32,
    },
    /// 原地闭环转向，`tenths_deg` 单位 0.1°（90° → 900），> 0。
    Rotate {
        dir: RotateDir,
        speed: u8,
        tenths_deg: u32,
    },
    /// 读状态。
    Heartbeat,
    /// 回 `Uninit`。
    Reset,
}

impl Request {
    /// 对应的请求类型（回 ACK/NACK 用）。
    pub const fn request_type(&self) -> RequestType {
        match self {
            Self::Init => RequestType::Init,
            Self::SetSpeed { .. } => RequestType::SetSpeed,
            Self::Stop { .. } => RequestType::Stop,
            Self::Brake { .. } => RequestType::Brake,
            Self::SetSpeeds { .. } => RequestType::SetSpeeds,
            Self::SetPid { .. } => RequestType::SetPid,
            Self::GetPid { .. } => RequestType::GetPid,
            Self::Move { .. } => RequestType::Move,
            Self::Rotate { .. } => RequestType::Rotate,
            Self::Heartbeat => RequestType::Heartbeat,
            Self::Reset => RequestType::Reset,
        }
    }

    /// 解码并校验一条命令。
    ///
    /// 布局与取值范围见 crate 级文档；`cmd` 是帧里的原始字节，未知时返回
    /// [`ProtocolError::UnknownRequest`]，长度不符返回 [`ProtocolError::BadLength`]，
    /// 枚举越界或数值越界返回 [`ProtocolError::InvalidValue`]。
    pub fn decode(cmd: u8, payload: &[u8]) -> Result<Self, ProtocolError> {
        let request_type = RequestType::from_u8(cmd).ok_or(ProtocolError::UnknownRequest(cmd))?;
        let expected = request_type.payload_len();
        if payload.len() != expected as usize {
            return Err(ProtocolError::BadLength {
                expected,
                got: payload.len(),
            });
        }

        let request = match request_type {
            RequestType::Init => Self::Init,
            RequestType::SetSpeed => Self::SetSpeed {
                motor: motor(payload[0])?,
                speed: i16::from_le_bytes([payload[1], payload[2]]),
            },
            RequestType::Stop => Self::Stop {
                target: target(payload[0])?,
            },
            RequestType::Brake => Self::Brake {
                target: target(payload[0])?,
            },
            RequestType::SetSpeeds => Self::SetSpeeds {
                left: i16::from_le_bytes([payload[0], payload[1]]),
                right: i16::from_le_bytes([payload[2], payload[3]]),
            },
            RequestType::SetPid => Self::SetPid {
                motor: motor(payload[0])?,
                kp: i16::from_le_bytes([payload[1], payload[2]]),
                ki: i16::from_le_bytes([payload[3], payload[4]]),
                kd: i16::from_le_bytes([payload[5], payload[6]]),
            },
            RequestType::GetPid => Self::GetPid {
                motor: motor(payload[0])?,
            },
            RequestType::Move => {
                let dir = MoveDir::from_u8(payload[0]).ok_or(ProtocolError::InvalidValue)?;
                let speed = payload[1];
                let target = i32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                if speed == 0 || speed > 100 || target <= 0 {
                    return Err(ProtocolError::InvalidValue);
                }
                Self::Move {
                    dir,
                    speed,
                    distance_mm: target as u32,
                }
            }
            RequestType::Rotate => {
                let dir = RotateDir::from_u8(payload[0]).ok_or(ProtocolError::InvalidValue)?;
                let speed = payload[1];
                let target = i32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                if speed == 0 || speed > 100 || target <= 0 {
                    return Err(ProtocolError::InvalidValue);
                }
                Self::Rotate {
                    dir,
                    speed,
                    tenths_deg: target as u32,
                }
            }
            RequestType::Heartbeat => Self::Heartbeat,
            RequestType::Reset => Self::Reset,
        };
        Ok(request)
    }

    /// 编码负载，返回负载长度；`out` 按 [`MAX_PAYLOAD`] 大小传入。
    ///
    /// 编码前会用与 [`Self::decode`] 相同的规则校验（非法值不回落到静默截断）。
    pub fn encode(&self, out: &mut [u8; MAX_PAYLOAD]) -> Result<usize, ProtocolError> {
        let len = match self {
            Self::Init | Self::Heartbeat | Self::Reset => 0,
            Self::SetSpeed { motor, speed } => {
                out[0] = motor.as_u8();
                out[1..3].copy_from_slice(&speed.to_le_bytes());
                3
            }
            Self::Stop { target } | Self::Brake { target } => {
                out[0] = target.as_u8();
                1
            }
            Self::SetSpeeds { left, right } => {
                out[0..2].copy_from_slice(&left.to_le_bytes());
                out[2..4].copy_from_slice(&right.to_le_bytes());
                4
            }
            Self::SetPid { motor, kp, ki, kd } => {
                out[0] = motor.as_u8();
                out[1..3].copy_from_slice(&kp.to_le_bytes());
                out[3..5].copy_from_slice(&ki.to_le_bytes());
                out[5..7].copy_from_slice(&kd.to_le_bytes());
                7
            }
            Self::GetPid { motor } => {
                out[0] = motor.as_u8();
                1
            }
            Self::Move {
                dir,
                speed,
                distance_mm,
            } => {
                if *speed == 0 || *speed > 100 || *distance_mm == 0 {
                    return Err(ProtocolError::InvalidValue);
                }
                out[0] = dir.as_u8();
                out[1] = *speed;
                out[2..6].copy_from_slice(&distance_mm.to_le_bytes());
                6
            }
            Self::Rotate {
                dir,
                speed,
                tenths_deg,
            } => {
                if *speed == 0 || *speed > 100 || *tenths_deg == 0 {
                    return Err(ProtocolError::InvalidValue);
                }
                out[0] = dir.as_u8();
                out[1] = *speed;
                out[2..6].copy_from_slice(&tenths_deg.to_le_bytes());
                6
            }
        };
        Ok(len)
    }

    /// 编码成完整帧（含帧头与校验和）。
    pub fn to_frame(&self) -> Result<Frame, ProtocolError> {
        let mut payload = [0u8; MAX_PAYLOAD];
        let len = self.encode(&mut payload)?;
        Frame::new(self.request_type().as_u8(), &payload[..len])
    }
}

fn motor(v: u8) -> Result<MotorId, ProtocolError> {
    MotorId::from_u8(v).ok_or(ProtocolError::InvalidValue)
}

fn target(v: u8) -> Result<MotorTarget, ProtocolError> {
    MotorTarget::from_u8(v).ok_or(ProtocolError::InvalidValue)
}
