//! `PT_JPEG` 解码通道（`CVI_VDEC_*`）。
//!
//! [`Decoder`] 借用 [`Sys`] 会话，而每个 [`DecodedFrame`] 又借用产生它的
//! `Decoder`，因此在帧还活着时，会话和通道都不会被拆掉：
//!
//! ```no_run
//! use cvimpi_rs::{
//!     decoder::DecoderConfig, ffi,
//!     sys::{Sys, VbPoolConfig},
//!     vdec_frame_buffer_size,
//! };
//!
//! # fn main() -> Result<(), cvimpi_rs::Error> {
//! let blk_size = vdec_frame_buffer_size(1920, 1080, ffi::PIXEL_FORMAT_NV12).unwrap();
//! let sys = Sys::init(&[VbPoolConfig::new(blk_size, 2)])?;
//!
//! let dec = sys.create_decoder(0, &DecoderConfig::new(1920, 1080))?;
//! let jpeg_data: &[u8] = &[];
//! let frame = dec.decode(jpeg_data, ffi::CVI_IO_BLOCK)?;
//! let yuv = frame.copy_tight()?;
//! # Ok(())
//! # }
//! ```

use core::marker::PhantomData;
use core::{mem, slice};

use crate::ffi;
use crate::sys::Sys;
use crate::{
    Error, Result, SessionRef, align_up, check, plane_dims, tight_frame_size,
    vdec_frame_buffer_size,
};

/// JPEG 解码通道的配置。
#[derive(Debug, Clone, Copy)]
pub struct DecoderConfig {
    /// 通道能接受的最大图像宽度（用于分配缓冲）。
    pub max_width: u32,
    /// 通道能接受的最大图像高度（用于分配缓冲）。
    pub max_height: u32,
    /// 输出像素格式（`NV12` / `NV21` / planar 格式）。
    pub pixel_format: ffi::PIXEL_FORMAT_E,
    /// 输出帧缓冲个数（`u32FrameBufCnt`）。
    ///
    /// common 池模式下驱动要求它**不小于池的块数**，所以实际生效值会取
    /// `max(本值, 池最大块数)`（见 [`Sys::max_pool_blk_cnt`]）。默认 1 表示
    /// 期望的最小值；池里块多时中间件会自动放大。
    pub frame_buf_cnt: u32,
    /// 码流缓冲大小；`None` 取 `align_up(w * h, 0x4000)`。
    pub stream_buf_size: Option<u32>,
    /// `u32DisplayFrameNum`（JPEG 用 0）。
    pub display_frame_num: u32,
    /// 输出格式带 alpha 时使用的 alpha 值。
    pub alpha: u32,
}

impl DecoderConfig {
    pub fn new(max_width: u32, max_height: u32) -> Self {
        Self {
            max_width,
            max_height,
            pixel_format: ffi::PIXEL_FORMAT_NV12,
            frame_buf_cnt: 1,
            stream_buf_size: None,
            display_frame_num: 0,
            alpha: 255,
        }
    }

    pub fn with_pixel_format(mut self, fmt: ffi::PIXEL_FORMAT_E) -> Self {
        self.pixel_format = fmt;
        self
    }

    pub fn with_frame_buf_cnt(mut self, cnt: u32) -> Self {
        self.frame_buf_cnt = cnt;
        self
    }
}

/// 一条 `PT_JPEG` 解码通道（`CVI_VDEC_*`）。
///
/// 由 [`Sys::create_decoder`] 创建；析构时停止并销毁通道，且在有
/// [`DecodedFrame`] 借用它时无法析构。
pub struct Decoder<'a> {
    chn: ffi::VDEC_CHN,
    _session: SessionRef<'a>,
}

impl<'a> Decoder<'a> {
    /// 创建通道并开始接收码流。
    ///
    /// VB 池的 block 必须不小于
    /// `vdec_frame_buffer_size(max_width, max_height, pixel_format)`。
    pub(crate) fn create(
        sys: &'a Sys,
        chn: ffi::VDEC_CHN,
        cfg: &DecoderConfig,
    ) -> Result<Decoder<'a>> {
        let frame_buf_size =
            vdec_frame_buffer_size(cfg.max_width, cfg.max_height, cfg.pixel_format).ok_or_else(
                || Error::invalid("vdec_frame_buffer_size (unsupported pixel format)"),
            )?;

        let mut attr: ffi::VDEC_CHN_ATTR_S = unsafe { mem::zeroed() };
        attr.enType = ffi::PT_JPEG;
        attr.enMode = ffi::VIDEO_MODE_FRAME;
        attr.u32PicWidth = cfg.max_width;
        attr.u32PicHeight = cfg.max_height;
        attr.u32StreamBufSize = cfg
            .stream_buf_size
            .unwrap_or_else(|| align_up(cfg.max_width * cfg.max_height, 0x4000));
        attr.u32FrameBufSize = frame_buf_size;
        // common 池模式下驱动要求 `FrameBufCnt >= 池块数`，否则 SendStream 报 BUF_FULL。
        // 配置值只作为下限，实际取 `max(配置值, 池最大块数)`。
        attr.u32FrameBufCnt = cfg.frame_buf_cnt.max(sys.max_pool_blk_cnt());

        check(
            unsafe { ffi::CVI_VDEC_CreateChn(chn, &attr) },
            "CVI_VDEC_CreateChn",
        )?;

        let dec = Decoder {
            chn,
            _session: PhantomData,
        };

        let mut param: ffi::VDEC_CHN_PARAM_S = unsafe { mem::zeroed() };
        param.enType = ffi::PT_JPEG;
        param.enPixelFormat = cfg.pixel_format;
        param.u32DisplayFrameNum = cfg.display_frame_num;
        param.picture_mut().u32Alpha = cfg.alpha;

        check(
            unsafe { ffi::CVI_VDEC_SetChnParam(chn, &param) },
            "CVI_VDEC_SetChnParam",
        )?;
        check(
            unsafe { ffi::CVI_VDEC_StartRecvStream(chn) },
            "CVI_VDEC_StartRecvStream",
        )?;
        Ok(dec)
    }

    pub fn channel(&self) -> ffi::VDEC_CHN {
        self.chn
    }

    /// 把一整张 JPEG 作为一帧送入（`bEndOfFrame = 1`）。
    pub fn send_stream(&self, jpeg: &[u8], timeout_ms: ffi::CVI_S32) -> Result<()> {
        let stream = ffi::VDEC_STREAM_S {
            u32Len: jpeg.len() as u32,
            u64PTS: 0,
            bEndOfFrame: ffi::CVI_TRUE,
            bEndOfStream: ffi::CVI_FALSE,
            bDisplay: ffi::CVI_TRUE,
            pu8Addr: jpeg.as_ptr() as *mut u8,
        };
        check(
            unsafe { ffi::CVI_VDEC_SendStream(self.chn, &stream, timeout_ms) },
            "CVI_VDEC_SendStream",
        )
    }

    /// 取出一帧解码结果。返回的 [`DecodedFrame`] 借用本 decoder，并在析构时
    /// 释放对应的 VB block。
    pub fn get_frame<'s>(&'s self, timeout_ms: ffi::CVI_S32) -> Result<DecodedFrame<'s>> {
        let mut info: ffi::VIDEO_FRAME_INFO_S = unsafe { mem::zeroed() };
        check(
            unsafe { ffi::CVI_VDEC_GetFrame(self.chn, &mut info, timeout_ms) },
            "CVI_VDEC_GetFrame",
        )?;

        // 与 C sample 一致：让 CPU 能看到解码后的数据。
        if let Some(dims) = plane_dims(
            info.stVFrame.enPixelFormat,
            info.stVFrame.u32Width,
            info.stVFrame.u32Height,
        ) {
            for p in 0..dims.planes {
                let vir = info.stVFrame.pu8VirAddr[p];
                if vir.is_null() {
                    continue;
                }
                let size = info.stVFrame.u32Stride[p].saturating_mul(dims.rows[p]);
                unsafe {
                    ffi::CVI_SYS_IonInvalidateCache(info.stVFrame.u64PhyAddr[p], vir.cast(), size);
                }
            }
        }

        Ok(DecodedFrame {
            chn: self.chn,
            info,
            _decoder: PhantomData,
        })
    }

    /// `send_stream` + `get_frame`。
    pub fn decode<'s>(&'s self, jpeg: &[u8], timeout_ms: ffi::CVI_S32) -> Result<DecodedFrame<'s>> {
        self.send_stream(jpeg, timeout_ms)?;
        self.get_frame(timeout_ms)
    }
}

impl Drop for Decoder<'_> {
    fn drop(&mut self) {
        unsafe {
            ffi::CVI_VDEC_StopRecvStream(self.chn);
            ffi::CVI_VDEC_DestroyChn(self.chn);
        }
    }
}

/// 由 decoder 的 VB 池持有的解码帧；析构时释放。
///
/// 借用产生它的 [`Decoder`]（见 [`Decoder::get_frame`]）。
pub struct DecodedFrame<'s> {
    chn: ffi::VDEC_CHN,
    info: ffi::VIDEO_FRAME_INFO_S,
    _decoder: PhantomData<&'s ()>,
}

impl DecodedFrame<'_> {
    pub fn info(&self) -> &ffi::VIDEO_FRAME_INFO_S {
        &self.info
    }

    pub fn width(&self) -> u32 {
        self.info.stVFrame.u32Width
    }

    pub fn height(&self) -> u32 {
        self.info.stVFrame.u32Height
    }

    pub fn pixel_format(&self) -> ffi::PIXEL_FORMAT_E {
        self.info.stVFrame.enPixelFormat
    }

    pub fn stride(&self, plane: usize) -> u32 {
        self.info.stVFrame.u32Stride[plane]
    }

    /// 持有期间查看某个平面（`stride * rows` 字节）。
    pub fn plane(&self, plane: usize) -> Option<&[u8]> {
        let dims = plane_dims(self.pixel_format(), self.width(), self.height())?;
        if plane >= dims.planes {
            return None;
        }
        let ptr = self.info.stVFrame.pu8VirAddr[plane];
        if ptr.is_null() {
            return None;
        }
        let len = (self.info.stVFrame.u32Stride[plane] as usize)
            .checked_mul(dims.rows[plane] as usize)?;
        // SAFETY: 驱动为该平面映射了至少 stride * rows 字节，且帧只在析构时才释放。
        Some(unsafe { slice::from_raw_parts(ptr, len) })
    }

    /// 把各平面拷出为紧凑缓冲（Y，然后 U/V 或交织的 UV）。
    pub fn copy_tight(&self) -> Result<Vec<u8>> {
        let dims = plane_dims(self.pixel_format(), self.width(), self.height())
            .ok_or_else(|| Error::invalid("copy_tight (unsupported pixel format)"))?;
        let total = tight_frame_size(self.pixel_format(), self.width(), self.height())
            .ok_or_else(|| Error::invalid("copy_tight (size overflow)"))?
            as usize;

        let mut out = vec![0u8; total];
        let mut off = 0usize;
        for p in 0..dims.planes {
            let plane = self
                .plane(p)
                .ok_or_else(|| Error::invalid("copy_tight (plane unavailable)"))?;
            let row = dims.row_bytes[p] as usize;
            let rows = dims.rows[p] as usize;
            let stride = self.info.stVFrame.u32Stride[p] as usize;
            for r in 0..rows {
                out[off..off + row].copy_from_slice(&plane[r * stride..r * stride + row]);
                off += row;
            }
        }
        Ok(out)
    }
}

impl Drop for DecodedFrame<'_> {
    fn drop(&mut self) {
        unsafe {
            ffi::CVI_VDEC_ReleaseFrame(self.chn, &self.info);
        }
    }
}

// SAFETY: 理由同 `encoder::Frame` —— 平面裸指针指向中间件持有的 VB 内存，
// 且不携带任何线程局部状态，因此把帧移动到别的线程是安全的。
unsafe impl Send for DecodedFrame<'_> {}
