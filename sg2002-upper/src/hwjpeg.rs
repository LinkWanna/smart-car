//! 硬件 JPEG 编码（SG2002 VENC / `PT_JPEG`）的 Rust 封装。
//!
//! 实现在 `csrc/hwjpeg.c`：运行期 dlopen `/mnt/system/usr/lib/{libsys,libvenc}.so`，
//! 用 VB（ION 物理连续内存）+ VENC 通道把 **YUYV422** 帧编码成 JPEG。
//! 相比纯软件编码（C906 上 640x480 约 70~135ms），硬件路径约 15ms（含 YUYV→NV12 转换）。
//!
//! 三个约定：
//! - [`HwJpeg`] **不是 `Send`**：VENC 通道与线程绑定，`new`/`encode`/`Drop` 必须同一线程；
//! - 输入固定 640x480 YUYV422（与相机一致）；
//! - 任何失败都返回 `io::Error`，上层据此降级到软件编码。

use std::ffi::{CStr, c_char, c_int, c_uint};
use std::io;
use std::marker::PhantomData;
use std::sync::Arc;

use crate::vision::{FRAME_H, FRAME_W};

/// 与 C 封装约定的输出缓冲上限（640x480 高质量 JPEG 远小于此）。
const OUT_CAPACITY: usize = 1 << 20;

#[cfg(not(no_hwjpeg))]
mod ffi {
    use super::*;

    unsafe extern "C" {
        fn hwjpeg_init(width: c_uint, height: c_uint, quality: c_uint) -> c_int;
        fn hwjpeg_encode(
            yuyv: *const u8,
            src_len: c_uint,
            dst: *mut u8,
            dst_cap: c_uint,
            out_len: *mut c_uint,
        ) -> c_int;
        fn hwjpeg_exit();
        fn hwjpeg_error() -> *const c_char;
        fn hwjpeg_input_format() -> c_uint;
    }

    pub fn last_error() -> String {
        unsafe {
            CStr::from_ptr(hwjpeg_error())
                .to_string_lossy()
                .into_owned()
        }
    }

    pub fn input_format() -> u32 {
        unsafe { hwjpeg_input_format() }
    }

    pub fn init(width: u32, height: u32, quality: u32) -> io::Result<()> {
        if unsafe { hwjpeg_init(width, height, quality) } == 0 {
            Ok(())
        } else {
            Err(io::Error::other(last_error()))
        }
    }

    pub fn encode(input: &[u8], out: &mut [u8]) -> io::Result<usize> {
        let mut len: c_uint = 0;
        let ret = unsafe {
            hwjpeg_encode(
                input.as_ptr(),
                input.len() as c_uint,
                out.as_mut_ptr(),
                out.len() as c_uint,
                &mut len,
            )
        };
        if ret == 0 {
            Ok(len as usize)
        } else {
            Err(io::Error::other(last_error()))
        }
    }

    pub fn exit() {
        unsafe { hwjpeg_exit() }
    }
}

#[cfg(no_hwjpeg)]
mod ffi {
    use super::*;

    pub fn init(_width: u32, _height: u32, _quality: u32) -> io::Result<()> {
        Err(io::Error::other(
            "本次构建不含硬件编码（未找到 cvi_mpi SDK 头文件）",
        ))
    }

    pub fn encode(_input: &[u8], _out: &mut [u8]) -> io::Result<usize> {
        Err(io::Error::other("本次构建不含硬件编码"))
    }

    pub fn exit() {}

    pub fn input_format() -> u32 {
        0
    }
}

/// 硬件 JPEG 编码器句柄（与创建它的线程绑定）。
pub struct HwJpeg {
    out: Vec<u8>,
    /// 保持 `!Send`：VENC 通道必须固定线程使用。
    _not_send: PhantomData<*const ()>,
}

impl HwJpeg {
    /// 打开 VENC 通道；失败（没有厂商库/内核不支持/参数非法）返回错误，上层降级。
    pub fn new(width: u32, height: u32, quality: u8) -> io::Result<Self> {
        if width as usize != FRAME_W || height as usize != FRAME_H {
            return Err(io::Error::other(format!(
                "硬件编码只支持 {FRAME_W}x{FRAME_H}，收到 {width}x{height}"
            )));
        }
        ffi::init(width, height, u32::from(quality))?;
        Ok(Self {
            out: vec![0u8; OUT_CAPACITY],
            _not_send: PhantomData,
        })
    }

    /// 编码一帧 640x480 YUYV422，返回 JPEG 字节。
    pub fn encode(&mut self, yuyv: &[u8]) -> io::Result<Arc<[u8]>> {
        let len = ffi::encode(yuyv, &mut self.out)?;
        if len == 0 {
            return Err(io::Error::other("硬件编码输出为空"));
        }
        Ok(Arc::from(&self.out[..len]))
    }

    /// 实际使用的输入像素格式（SDK 的 `PIXEL_FORMAT_E` 数值，日志用）。
    pub fn input_format(&self) -> u32 {
        ffi::input_format()
    }
}

impl Drop for HwJpeg {
    fn drop(&mut self) {
        ffi::exit();
    }
}
