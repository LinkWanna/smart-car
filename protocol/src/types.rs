//! Wire-level enums: the numeric value of every command, response, error and
//! state byte, plus the per-command length/state tables they imply.

/// 为 `no_std` 侧的枚举提供 `Display`（名字即线上文档里的名字）。
macro_rules! display_names {
    ($ty:ty { $($variant:ident => $name:literal),* $(,)? }) => {
        impl core::fmt::Display for $ty {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str(match self {
                    $(Self::$variant => $name,)*
                })
            }
        }
    };
}

/// 上位机 → 车 的请求类型（命令号）。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RequestType {
    /// 编码器清零、PID 复位，进 `Ready`。
    Init = 0x01,
    /// 单轮目标速度，负载 `motor(1) + speed(i16)`。
    SetSpeed = 0x10,
    /// 滑行（coast），负载 `target(1)`：0/1/2 = 左/右/双。
    Stop = 0x11,
    /// 短接制动，负载 `target(1)`：0/1/2 = 左/右/双。
    Brake = 0x12,
    /// 双轮目标速度，负载 `left(i16) + right(i16)`。
    SetSpeeds = 0x13,
    /// 改增益，负载 `motor(1) + kp/ki/kd(i16, ×100)`。
    SetPid = 0x14,
    /// 读增益，负载 `motor(1)`，应答 `PidData`。
    GetPid = 0x15,
    /// 闭环直行，负载 `dir(1) + speed(1) + mm(i32 > 0)`。
    Move = 0x20,
    /// 原地闭环转向，负载 `dir(1) + speed(1) + 0.1°(i32 > 0)`。
    Rotate = 0x21,
    /// 读状态，无负载，应答 `Status`。
    Heartbeat = 0xFE,
    /// 回 `Uninit`，无负载。
    Reset = 0xFF,
}

impl RequestType {
    /// 解码请求类型；未知返回 `None`（对应 `NACK UnknownRequest`）。
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::Init),
            0x10 => Some(Self::SetSpeed),
            0x11 => Some(Self::Stop),
            0x12 => Some(Self::Brake),
            0x13 => Some(Self::SetSpeeds),
            0x14 => Some(Self::SetPid),
            0x15 => Some(Self::GetPid),
            0x20 => Some(Self::Move),
            0x21 => Some(Self::Rotate),
            0xFE => Some(Self::Heartbeat),
            0xFF => Some(Self::Reset),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// 负载的**精确**长度，见 crate 级表格。
    pub const fn payload_len(self) -> u8 {
        match self {
            Self::Init | Self::Heartbeat | Self::Reset => 0,
            Self::Stop | Self::Brake | Self::GetPid => 1,
            Self::SetSpeed => 3,
            Self::SetSpeeds => 4,
            Self::Move | Self::Rotate => 6,
            Self::SetPid => 7,
        }
    }

    /// 受理该命令所需的最低系统状态；`None` 表示无门槛。
    ///
    /// 状态比较用 [`SysState`] 的 `Ord`，不满足时车回 `WrongState`。
    pub const fn min_state(self) -> Option<SysState> {
        match self {
            Self::Init | Self::Heartbeat | Self::Reset => None,
            Self::GetPid
            | Self::SetSpeed
            | Self::SetSpeeds
            | Self::Stop
            | Self::Brake
            | Self::SetPid
            | Self::Move
            | Self::Rotate => Some(SysState::Ready),
        }
    }
}

display_names!(RequestType {
    Init => "Init",
    SetSpeed => "SetSpeed",
    Stop => "Stop",
    Brake => "Brake",
    SetSpeeds => "SetSpeeds",
    SetPid => "SetPid",
    GetPid => "GetPid",
    Move => "Move",
    Rotate => "Rotate",
    Heartbeat => "Heartbeat",
    Reset => "Reset",
});

/// 车 → 上位机 的应答类型（应答号）。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResponseType {
    /// 命令被接受，负载为请求命令号。
    Ack = 0x80,
    /// 命令被拒绝，负载 `cmd(1) + error(1)`。
    Nack = 0x81,
    /// 状态快照，见 [`crate::Status`]。
    Status = 0x91,
    /// PID 增益，见 [`crate::PidData`]。
    PidData = 0x92,
}

impl ResponseType {
    /// 解码应答号；未知返回 `None`。
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x80 => Some(Self::Ack),
            0x81 => Some(Self::Nack),
            0x91 => Some(Self::Status),
            0x92 => Some(Self::PidData),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(ResponseType {
    Ack => "Ack",
    Nack => "Nack",
    Status => "Status",
    PidData => "PidData",
});

/// `NACK` 的原因码。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorCode {
    /// 当前状态不接受该命令（见 [`RequestType::min_state`]）。
    WrongState = 0x01,
    /// 帧校验和错误。
    BadChecksum = 0x02,
    /// 负载长度或取值非法。
    InvalidParam = 0x03,
    /// 请求类型未知。
    UnknownRequest = 0x04,
}

impl ErrorCode {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::WrongState),
            0x02 => Some(Self::BadChecksum),
            0x03 => Some(Self::InvalidParam),
            0x04 => Some(Self::UnknownRequest),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(ErrorCode {
    WrongState => "WrongState",
    BadChecksum => "BadChecksum",
    InvalidParam => "InvalidParam",
    UnknownRequest => "UnknownRequest",
});

/// 系统状态；顺序即 `Ord`，可直接比较"是否达到门槛"。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SysState {
    /// 未初始化：只受理 `Init`/`Reset`/`Heartbeat`。
    Uninit = 0,
    /// 已 `Init`：可受理 `GetPid` 与动作类命令。
    Ready = 1,
    /// 闭环运行中。
    Running = 2,
}

impl SysState {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Uninit),
            1 => Some(Self::Ready),
            2 => Some(Self::Running),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(SysState {
    Uninit => "Uninit",
    Ready => "Ready",
    Running => "Running",
});

/// 电机编号，线值 0 = 左、1 = 右。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorId {
    Left = 0,
    Right = 1,
}

impl MotorId {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Left),
            1 => Some(Self::Right),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(MotorId {
    Left => "Left",
    Right => "Right",
});

/// `Stop`/`Brake` 的作用范围，线值 0/1/2 = 左/右/双。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorTarget {
    Left = 0,
    Right = 1,
    Both = 2,
}

impl MotorTarget {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Left),
            1 => Some(Self::Right),
            2 => Some(Self::Both),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// 该范围覆盖的电机列表，便于 `for` 循环。
    pub const fn motors(self) -> &'static [MotorId] {
        match self {
            Self::Left => &[MotorId::Left],
            Self::Right => &[MotorId::Right],
            Self::Both => &[MotorId::Left, MotorId::Right],
        }
    }
}

display_names!(MotorTarget {
    Left => "Left",
    Right => "Right",
    Both => "Both",
});

/// `Move` 的方向字节，线值 0 = 前进、1 = 后退。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MoveDir {
    Forward = 0,
    Backward = 1,
}

impl MoveDir {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Forward),
            1 => Some(Self::Backward),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(MoveDir {
    Forward => "Forward",
    Backward => "Backward",
});

/// `Rotate` 的方向字节，线值 0 = 左转、1 = 右转。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RotateDir {
    Left = 0,
    Right = 1,
}

impl RotateDir {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Left),
            1 => Some(Self::Right),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

display_names!(RotateDir {
    Left => "Left",
    Right => "Right",
});
