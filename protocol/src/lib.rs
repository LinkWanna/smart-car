//! 车 ↔ 上位机线协议：帧格式、命令/应答的完整类型定义与校验。
//!
//! 这是上下位机**共用的唯一 Rust 协议实现**（`no_std`，无堆分配）：
//!
//! - 下位机 `esp32c3-slave` 用它解码命令、编码应答；
//! - 上位机 `sg2002-upper` 用它编码命令、解码应答；
//! - Python 镜像见 `esp32c3-slave/tools/src/protocol.py`。
//!
//! 任何线上字节格式的改动都只改这里，两端一起升级。
//!
//! # 帧格式
//!
//! ```text
//! AA 55 CMD LEN PAYLOAD.. CHK      CHK = CMD ^ LEN ^ PAYLOAD..
//! ```
//!
//! - 多字节整数一律**小端**（LE），直接使用 `to_le_bytes` / `from_le_bytes`；
//! - `LEN ≤ 16`（[`MAX_PAYLOAD`](frame::MAX_PAYLOAD)），超长帧被解析器丢弃并立即重同步；
//! - 校验和错回 `NACK BadChecksum`（回显帧里的 CMD 字节），未知请求回
//!   `NACK UnknownRequest`，长度/取值非法回 `NACK InvalidParam`。
//!
//! # 命令（上位机 → 车）
//!
//! | 命令 | 值 | 负载 | 长度 | 校验 | 受理状态 | 应答 |
//! | --- | --- | --- | --- | --- | --- | --- |
//! | `Init` | 0x01 | — | 0 | — | 任意 | ACK，→Ready |
//! | `SetSpeed` | 0x10 | `motor(1) + speed(i16)` | 3 | motor∈0/1；speed 截断 ±100 | ≥Ready | ACK，→Running |
//! | `Stop` | 0x11 | `target(1)` | 1 | target∈0/1/2 | ≥Ready | ACK |
//! | `Brake` | 0x12 | `target(1)` | 1 | target∈0/1/2 | ≥Ready | ACK |
//! | `SetSpeeds` | 0x13 | `left(i16) + right(i16)` | 4 | 两轮截断 ±100 | ≥Ready | ACK，→Running |
//! | `SetPid` | 0x14 | `motor(1) + kp/ki/kd(i16, ×100)` | 7 | motor∈0/1 | ≥Ready | ACK |
//! | `GetPid` | 0x15 | `motor(1)` | 1 | motor∈0/1 | ≥Ready | PidData |
//! | `Move` | 0x20 | `dir(1) + speed(1) + mm(i32>0)` | 6 | dir∈0/1；speed 1..100；mm>0 | ≥Ready | ACK，→Running |
//! | `Rotate` | 0x21 | `dir(1) + speed(1) + 0.1°(i32>0)` | 6 | dir∈0/1；speed 1..100；0.1°>0 | ≥Ready | ACK，→Running |
//! | `Heartbeat` | 0xFE | — | 0 | — | 任意 | Status |
//! | `Reset` | 0xFF | — | 0 | — | 任意 | ACK，→Uninit |
//!
//! 长度是**精确值**：多一字节少一字节都回 `InvalidParam`。
//! `SetSpeed`/`SetSpeeds` 的速度按 ±100 截断（不是拒绝），其余越界一律拒绝；
//! `Move`/`Rotate` 的 `dir` 与 `speed` 含义见对应类型的文档。
//!
//! # 应答（车 → 上位机）
//!
//! | 应答 | 值 | 负载 | 长度 |
//! | --- | --- | --- | --- |
//! | `Ack` | 0x80 | `cmd(1)`（回显请求命令号） | 1 |
//! | `Nack` | 0x81 | `cmd(1) + error(1)` | 2 |
//! | `Status` | 0x91 | `sys(1) + rpm0/1(i16) + distActive(1) + distResult(1)` | 7 |
//! | `PidData` | 0x92 | `kp/ki/kd(i16, ×100)` | 6 |
//!
//! # 状态机
//!
//! `Uninit →(Init)→ Ready →(速度/Move/Rotate)→ Running`，`Reset` 随时回 `Uninit`。
//! 命令的受理门槛见 [`RequestType::min_state`]，不满足回 `WrongState`。

#![no_std]

pub mod frame;
pub mod request;
pub mod response;
pub mod types;

pub use self::frame::{Frame, RxEvent, RxParser};
pub use self::request::Request;
pub use self::response::{PidData, Response, Status};
pub use self::types::{
    ErrorCode, MotorId, MotorTarget, MoveDir, RequestType, ResponseType, RotateDir, SysState,
};

/// 协议层错误：组帧、编码、解码共用一种类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtocolError {
    PayloadTooLong { len: usize },
    UnknownRequest(u8),
    UnknownResponse(u8),
    BadLength { expected: u8, got: usize },
    InvalidValue,
}

impl ProtocolError {
    /// 该错误在线上对应的 `NACK` 原因码。
    pub const fn error_code(self) -> ErrorCode {
        match self {
            Self::UnknownRequest(_) => ErrorCode::UnknownRequest,
            Self::PayloadTooLong { .. }
            | Self::UnknownResponse(_)
            | Self::BadLength { .. }
            | Self::InvalidValue => ErrorCode::InvalidParam,
        }
    }
}

impl core::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PayloadTooLong { len } => write!(f, "负载 {len} 字节超过协议上限"),
            Self::UnknownRequest(cmd) => write!(f, "未知请求 0x{cmd:02X}"),
            Self::UnknownResponse(res) => write!(f, "未知应答 0x{res:02X}"),
            Self::BadLength { expected, got } => {
                write!(f, "负载长度 {got} != {expected}")
            }
            Self::InvalidValue => f.write_str("负载取值非法"),
        }
    }
}

impl core::error::Error for ProtocolError {}

#[cfg(test)]
mod tests;
