//! CVI MPI 的 Rust 薄 FFI 封装，只覆盖 JPEG 编解码（CV180X / CV181X）。
//!
//! 范围刻意收窄：
//!
//! * [`ffi`] — 原始 `extern "C"` 绑定：JPEG 编解码路径所需的
//!   `sys` / `vb` / `venc` / `vdec` 接口，字段布局对照 `../include` 下的 C 头文件
//!   （用 `tools/abi_probe.c` 实测）。
//! * [`sys`] — 会话所有者：`CVI_SYS_Init` + 全局 VB 池
//!   （[`sys::Sys`]、[`sys::VbPoolConfig`]）；其余句柄都从它派生。
//! * [`encoder`] — `PT_JPEG` 编码通道及其 VB 输入帧 [`encoder::Frame`]。
//! * [`decoder`] — `PT_JPEG` 解码通道及 [`decoder::DecodedFrame`]。
//! * [`vpss`] — 视频后处理（硬件 CSC + 缩放），直接走 `/dev/cvi-vpss` 的 ioctl。
//!
//! 共享逻辑留在 crate 根部：错误类型 [`Error`] 和对照 `cvi_buffer.h` 的缓冲布局计算。
//!
//! # 生命周期
//!
//! [`sys::Sys`] 拥有进程级的中间件状态。`Encoder`、`Decoder`、`Frame` 都借用它
//! （[`encoder::Encoder<'a>`] 等），`DecodedFrame` 则借用产生它的 `Decoder`。
//! 因此"先拆资源、后使用句柄"这类错误在编译期就会被拒绝：
//!
//! * `Encoder`/`Decoder`/`Frame` 还活着时析构 `Sys` —— 编译不过；
//! * `DecodedFrame` 还活着时析构 `Decoder` —— 编译不过。
//!
//! 句柄都是 `Send`：可以移动到别的线程。封装不阻止按引用共享（`&Encoder`），
//! 所以如果要多线程调用中间件，请自行串行化（例如用 `Mutex`）。
//!
//! ```no_run
//! fn assert_send<T: Send>() {}
//! assert_send::<cvimpi_rs::sys::Sys>();
//! assert_send::<cvimpi_rs::encoder::Encoder<'static>>();
//! assert_send::<cvimpi_rs::encoder::Frame<'static>>();
//! assert_send::<cvimpi_rs::decoder::Decoder<'static>>();
//! assert_send::<cvimpi_rs::decoder::DecodedFrame<'static>>();
//! ```
//!
//! ```no_run
//! use cvimpi_rs::{
//!     decoder::DecoderConfig,
//!     encoder::EncoderConfig,
//!     ffi,
//!     sys::{Sys, VbPoolConfig},
//!     vdec_frame_buffer_size, venc_input_layout,
//! };
//!
//! # fn main() -> Result<(), cvimpi_rs::Error> {
//! let enc_layout = venc_input_layout(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420).unwrap();
//! let dec_size = vdec_frame_buffer_size(1920, 1080, ffi::PIXEL_FORMAT_NV12).unwrap();
//!
//! // 一个 common 池即可：block 取编码输入帧与解码输出帧所需的最大值
//! let sys = Sys::init(&[VbPoolConfig::new(enc_layout.vb_size.max(dec_size), 4)])?;
//!
//! // 编码
//! let enc = sys.create_encoder(
//!     0,
//!     &EncoderConfig::new(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420).with_quality(85),
//! )?;
//! let mut frame = sys.alloc_frame(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420)?;
//! frame.write_tight(&vec![0u8; (1920 * 1080 * 3 / 2) as usize])?;
//! let jpeg_data = enc.encode(&frame, ffi::CVI_IO_BLOCK)?;
//!
//! // 解码
//! let dec = sys.create_decoder(0, &DecoderConfig::new(1920, 1080))?;
//! let out = dec.decode(&jpeg_data, ffi::CVI_IO_BLOCK)?;
//! let yuv = out.copy_tight()?;
//! # Ok(())
//! # }
//! ```

pub mod decoder;
pub mod encoder;
pub mod ffi;
pub mod sys;
pub mod vpss;

use core::fmt;
use core::marker::PhantomData;

/// 零大小标记：句柄在 `'a` 期间借用 [`sys::Sys`] 会话。
pub(crate) type SessionRef<'a> = PhantomData<&'a ()>;

/* ------------------------------------------------------------------ */
/* 错误类型                                                            */
/* ------------------------------------------------------------------ */

/// 本 crate 统一使用的 Result 别名。
pub type Result<T> = core::result::Result<T, Error>;

/// 中间件返回的错误（`code`），或封装自身的错误（`code == -1`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Error {
    pub code: i32,
    pub op: &'static str,
}

impl Error {
    pub(crate) fn from_op(code: i32, op: &'static str) -> Self {
        Self { code, op }
    }

    pub(crate) fn invalid(op: &'static str) -> Self {
        Self {
            code: ffi::CVI_FAILURE,
            op,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.code == ffi::CVI_FAILURE {
            write!(f, "{}: invalid argument or unsupported input", self.op)
        } else {
            write!(
                f,
                "{} failed with {:#x} ({})",
                self.op, self.code as u32, self.code
            )
        }
    }
}

impl std::error::Error for Error {}

#[inline]
pub(crate) fn check(ret: ffi::CVI_S32, op: &'static str) -> Result<()> {
    if ret == ffi::CVI_SUCCESS {
        Ok(())
    } else {
        Err(Error::from_op(ret, op))
    }
}

/* ------------------------------------------------------------------ */
/* 缓冲布局计算（对照 cvi_buffer.h 里的 static inline）                */
/* ------------------------------------------------------------------ */

/// 编码输入缓冲的宽度对齐（`VENC_ALIGN_W`）。
pub const VENC_ALIGN_W: u32 = 32;
/// 编码输入缓冲的高度对齐（`VENC_ALIGN_H`）。
pub const VENC_ALIGN_H: u32 = 16;
/// JPEG 解码输出帧的宽度对齐（`JPEGD_ALIGN_W`）。
pub const JPEGD_ALIGN_W: u32 = 64;
/// JPEG 解码输出帧的高度对齐（`JPEGD_ALIGN_H`）。
pub const JPEGD_ALIGN_H: u32 = 16;

const DEFAULT_ALIGN: u32 = 64;
const MAX_ALIGN: u32 = 1024;

#[inline]
pub const fn align_up(x: u32, a: u32) -> u32 {
    (((x as u64) + (a as u64) - 1) / (a as u64) * (a as u64)) as u32
}

/// 单个未压缩帧缓冲的几何信息
/// （`COMPRESS_MODE_NONE` + 8bit，等价 `COMMON_GetPicBufferConfig` 的输出）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameLayout {
    /// 需要向 VB 申请的 block 大小。
    pub vb_size: u32,
    /// 1（packed）、2（NV12/NV16 风格）或 3（planar）。
    pub plane_num: u32,
    /// 平面 0 的 stride。
    pub main_stride: u32,
    /// 色度平面的 stride。
    pub c_stride: u32,
    /// 平面 0 的长度。
    pub y_size: u32,
    /// 单个色度平面的长度。
    pub c_size: u32,
    /// 各平面实际占用的总字节数。
    pub main_size: u32,
    /// block 内各平面物理地址的对齐要求。
    pub addr_align: u32,
}

/// 某种像素格式的紧凑（tightly-packed）平面几何。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlaneDims {
    /// 平面数（1、2 或 3）。
    pub planes: usize,
    /// 紧凑缓冲中每个平面的行字节数。
    pub row_bytes: [u32; 3],
    /// 每个平面的行数。
    pub rows: [u32; 3],
}

/// 本封装支持的像素格式的平面几何。
pub fn plane_dims(fmt: ffi::PIXEL_FORMAT_E, width: u32, height: u32) -> Option<PlaneDims> {
    use ffi::*;
    let (planes, row_bytes, rows) = match fmt {
        PIXEL_FORMAT_YUV_PLANAR_420 => (
            3,
            [width, width / 2, width / 2],
            [height, height / 2, height / 2],
        ),
        PIXEL_FORMAT_YUV_PLANAR_422 => (3, [width, width / 2, width / 2], [height, height, height]),
        PIXEL_FORMAT_YUV_PLANAR_444 => (3, [width, width, width], [height, height, height]),
        PIXEL_FORMAT_YUV_400 => (1, [width, 0, 0], [height, 0, 0]),
        PIXEL_FORMAT_NV12 | PIXEL_FORMAT_NV21 => (2, [width, width, 0], [height, height / 2, 0]),
        PIXEL_FORMAT_NV16 | PIXEL_FORMAT_NV61 => (2, [width, width, 0], [height, height, 0]),
        PIXEL_FORMAT_RGB_888_PLANAR | PIXEL_FORMAT_BGR_888_PLANAR => {
            (3, [width, width, width], [height, height, height])
        }
        PIXEL_FORMAT_YUYV | PIXEL_FORMAT_UYVY | PIXEL_FORMAT_YVYU | PIXEL_FORMAT_VYUY => {
            (1, [width * 2, 0, 0], [height, 0, 0])
        }
        _ => return None,
    };
    Some(PlaneDims {
        planes,
        row_bytes,
        rows,
    })
}

/// `fmt` 格式的紧凑帧总大小。
pub fn tight_frame_size(fmt: ffi::PIXEL_FORMAT_E, width: u32, height: u32) -> Option<u32> {
    let d = plane_dims(fmt, width, height)?;
    let mut total = 0u32;
    for i in 0..d.planes {
        total = total.checked_add(d.row_bytes[i].checked_mul(d.rows[i])?)?;
    }
    Some(total)
}

/// `COMPRESS_MODE_NONE` + 8bit 格式下的 `COMMON_GetPicBufferConfig`。
fn common_pic_layout(
    width: u32,
    height: u32,
    fmt: ffi::PIXEL_FORMAT_E,
    align: u32,
) -> Option<FrameLayout> {
    use ffi::*;

    let align = if align == 0 {
        DEFAULT_ALIGN
    } else if align > MAX_ALIGN {
        MAX_ALIGN
    } else {
        align_up(align, DEFAULT_ALIGN)
    };

    let align_h = match fmt {
        PIXEL_FORMAT_YUV_PLANAR_420 | PIXEL_FORMAT_NV12 | PIXEL_FORMAT_NV21 => align_up(height, 2),
        _ => height,
    };

    let mut main_stride = align_up(width, align);
    let mut y_size = main_stride * align_h;
    let mut c_stride = 0u32;
    let mut c_size = 0u32;
    let main_size;
    let plane_num;

    match fmt {
        PIXEL_FORMAT_YUV_PLANAR_420 => {
            c_stride = align_up(width >> 1, align);
            c_size = (c_stride * align_h) >> 1;
            main_stride = c_stride * 2;
            y_size = main_stride * align_h;
            main_size = y_size + (c_size << 1);
            plane_num = 3;
        }
        PIXEL_FORMAT_YUV_PLANAR_422 => {
            c_stride = align_up(width >> 1, align);
            c_size = c_stride * align_h;
            main_size = y_size + (c_size << 1);
            plane_num = 3;
        }
        PIXEL_FORMAT_YUV_PLANAR_444 => {
            c_stride = main_stride;
            c_size = y_size;
            main_size = y_size + (c_size << 1);
            plane_num = 3;
        }
        PIXEL_FORMAT_RGB_888_PLANAR | PIXEL_FORMAT_BGR_888_PLANAR => {
            c_stride = main_stride;
            c_size = y_size;
            main_size = y_size + (c_size << 1);
            plane_num = 3;
        }
        PIXEL_FORMAT_NV12 | PIXEL_FORMAT_NV21 => {
            c_stride = align_up(width, align);
            c_size = (c_stride * align_h) >> 1;
            main_size = y_size + c_size;
            plane_num = 2;
        }
        PIXEL_FORMAT_NV16 | PIXEL_FORMAT_NV61 => {
            c_stride = align_up(width, align);
            c_size = c_stride * align_h;
            main_size = y_size + c_size;
            plane_num = 2;
        }
        PIXEL_FORMAT_YUYV | PIXEL_FORMAT_YVYU | PIXEL_FORMAT_UYVY | PIXEL_FORMAT_VYUY => {
            main_stride = align_up(width * 2, align);
            y_size = main_stride * align_h;
            main_size = y_size;
            plane_num = 1;
        }
        PIXEL_FORMAT_YUV_400 => {
            main_size = y_size;
            plane_num = 1;
        }
        _ => return None,
    }

    Some(FrameLayout {
        vb_size: main_size,
        plane_num,
        main_stride,
        c_stride,
        y_size,
        c_size,
        main_size,
        addr_align: align,
    })
}

/// 编码输入帧传给 `CVI_VB_GetBlock` / `CVI_VB_CreatePool` 的布局
/// （等价 `VENC_GetPicBufferConfig`）。
pub fn venc_input_layout(width: u32, height: u32, fmt: ffi::PIXEL_FORMAT_E) -> Option<FrameLayout> {
    let w = align_up(width, VENC_ALIGN_W);
    let h = align_up(height, VENC_ALIGN_H);
    common_pic_layout(w, h, fmt, VENC_ALIGN_W)
}

/// 单个解码输出帧的大小（等价 `VDEC_GetPicBufferSize`）。
pub fn vdec_frame_buffer_size(width: u32, height: u32, fmt: ffi::PIXEL_FORMAT_E) -> Option<u32> {
    let w = align_up(width, JPEGD_ALIGN_W);
    let h = align_up(height, JPEGD_ALIGN_H);
    common_pic_layout(w, h, fmt, JPEGD_ALIGN_W).map(|l| l.vb_size)
}
