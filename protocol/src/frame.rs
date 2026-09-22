//! 帧层：帧编码与逐字节接收状态机（与链路无关，UART/BLE 共用）。
//!
//! 线格式见 crate 级文档。接收状态机的行为与
//! `esp32c3-slave/tools/src/protocol.py::FrameParser` 逐一对应。

use super::ProtocolError;

/// 帧头第一个字节。
pub const FRAME_H1: u8 = 0xAA;
/// 帧头第二个字节。
pub const FRAME_H2: u8 = 0x55;
/// 单帧最大负载；所有命令负载最长 7 字节，16 是历史值。
pub const MAX_PAYLOAD: usize = 16;
/// 固定开销：帧头 2 + 命令 1 + 长度 1 + 校验 1。
pub const FRAME_OVERHEAD: usize = 5;
/// 最大帧长：帧头 2 + 命令 1 + 长度 1 + 负载 + 校验 1。
pub const FRAME_MAX: usize = MAX_PAYLOAD + FRAME_OVERHEAD;

/// XOR 校验和：`CMD ^ LEN ^ PAYLOAD..`。
pub const fn checksum(cmd: u8, payload: &[u8]) -> u8 {
    let mut chk = cmd ^ (payload.len() as u8);
    let mut i = 0;
    while i < payload.len() {
        chk ^= payload[i];
        i += 1;
    }
    chk
}

/// 编码完成的帧（含帧头与校验和）。`Copy`，可直接放进收发通道。
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    buf: [u8; FRAME_MAX],
    len: u8,
}

impl Frame {
    /// 组帧；负载超过 [`MAX_PAYLOAD`] 返回 [`ProtocolError::PayloadTooLong`]。
    pub fn new(cmd: u8, payload: &[u8]) -> Result<Self, ProtocolError> {
        if payload.len() > MAX_PAYLOAD {
            return Err(ProtocolError::PayloadTooLong { len: payload.len() });
        }
        let mut buf = [0u8; FRAME_MAX];
        buf[0] = FRAME_H1;
        buf[1] = FRAME_H2;
        buf[2] = cmd;
        buf[3] = payload.len() as u8;
        buf[4..4 + payload.len()].copy_from_slice(payload);
        buf[4 + payload.len()] = checksum(cmd, payload);
        Ok(Self {
            buf,
            len: payload.len() as u8 + FRAME_OVERHEAD as u8,
        })
    }

    /// 完整帧字节（含帧头与校验和）。
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    pub const fn cmd(&self) -> u8 {
        self.buf[2]
    }

    pub fn payload(&self) -> &[u8] {
        &self.buf[4..self.len as usize - 1]
    }

    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// 帧永远非空（至少 5 字节），为 `len` 配套存在。
    pub const fn is_empty(&self) -> bool {
        false
    }
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Frame({:?})", self.as_bytes())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Frame {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "Frame({=[u8]:02x})", self.as_bytes())
    }
}

/// [`RxParser::feed`] 产生的事件。
///
/// `Frame` 借用解析器的内部缓冲：把它的内容解码/复制走之后才能继续喂字节。
#[derive(Debug, PartialEq, Eq)]
pub enum RxEvent<'a> {
    /// 完整且校验通过的帧。
    Frame { cmd: u8, payload: &'a [u8] },
    /// 校验和错误（该帧已被消费），`cmd` 用于回 `NACK BadChecksum`。
    ChecksumFail { cmd: u8 },
    /// `LEN > MAX_PAYLOAD`，该帧被丢弃并立即重同步。
    TooLong { cmd: u8, len: u8 },
    /// 尚未凑齐一帧。
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxState {
    H1,
    H2,
    Cmd,
    Len,
    Payload,
    Chk,
}

/// 逐字节接收状态机；每条链路一个实例。
pub struct RxParser {
    state: RxState,
    cmd: u8,
    len: u8,
    idx: u8,
    buf: [u8; MAX_PAYLOAD],
}

impl RxParser {
    pub const fn new() -> Self {
        Self {
            state: RxState::H1,
            cmd: 0,
            len: 0,
            idx: 0,
            buf: [0; MAX_PAYLOAD],
        }
    }

    /// 喂入一个字节并推进状态机。
    pub fn feed(&mut self, b: u8) -> RxEvent<'_> {
        match self.state {
            RxState::H1 => {
                if b == FRAME_H1 {
                    self.state = RxState::H2;
                }
            }
            RxState::H2 => {
                self.state = if b == FRAME_H2 {
                    RxState::Cmd
                } else if b == FRAME_H1 {
                    // A second H1 restarts the header hunt instead of eating it.
                    RxState::H2
                } else {
                    RxState::H1
                };
            }
            RxState::Cmd => {
                self.cmd = b;
                self.state = RxState::Len;
            }
            RxState::Len => {
                self.len = b;
                self.idx = 0;
                if b as usize > MAX_PAYLOAD {
                    self.state = RxState::H1;
                    return RxEvent::TooLong {
                        cmd: self.cmd,
                        len: b,
                    };
                }
                self.state = if b == 0 {
                    RxState::Chk
                } else {
                    RxState::Payload
                };
            }
            RxState::Payload => {
                self.buf[self.idx as usize] = b;
                self.idx += 1;
                if self.idx >= self.len {
                    self.state = RxState::Chk;
                }
            }
            RxState::Chk => {
                self.state = RxState::H1;
                let payload = &self.buf[..self.len as usize];
                return if checksum(self.cmd, payload) == b {
                    RxEvent::Frame {
                        cmd: self.cmd,
                        payload,
                    }
                } else {
                    RxEvent::ChecksumFail { cmd: self.cmd }
                };
            }
        }
        RxEvent::None
    }
}

impl Default for RxParser {
    fn default() -> Self {
        Self::new()
    }
}
