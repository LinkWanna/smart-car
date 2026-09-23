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

use core::ffi::c_void;
use core::marker::PhantomData;
use core::{mem, slice};
use std::os::unix::io::RawFd;

use crate::ffi;
use crate::sys::Sys;
use crate::{
    Error, Result, SessionRef, align_up, ioctl_none, ioctl_read, ioctl_write, open_device,
    plane_dims, tight_frame_size, vdec_frame_buffer_size,
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

/// 一条 `PT_JPEG` 解码通道（`CVI_VDEC_*`，直连 `/dev/cvi_vc_dec{chn}`）。
///
/// 由 [`Sys::create_decoder`] 创建；析构时停止并销毁通道，且在有
/// [`DecodedFrame`] 借用它时无法析构。
pub struct Decoder<'a> {
    dev: VdecDev,
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

        let dev = VdecDev::create(chn, &attr)?;
        let dec = Decoder {
            dev,
            _session: PhantomData,
        };

        let mut param: ffi::VDEC_CHN_PARAM_S = unsafe { mem::zeroed() };
        param.enType = ffi::PT_JPEG;
        param.enPixelFormat = cfg.pixel_format;
        param.u32DisplayFrameNum = cfg.display_frame_num;
        param.picture_mut().u32Alpha = cfg.alpha;

        dec.dev.set_chn_param(&param)?;
        dec.dev.start_recv_stream()?;
        Ok(dec)
    }

    pub fn channel(&self) -> ffi::VDEC_CHN {
        self.dev.channel()
    }

    /// 读回通道参数（`CVI_VDEC_GetChnParam`，诊断用）。
    pub fn get_chn_param(&self) -> Result<ffi::VDEC_CHN_PARAM_S> {
        let mut param: ffi::VDEC_CHN_PARAM_S = unsafe { mem::zeroed() };
        self.dev.get_chn_param(&mut param)?;
        Ok(param)
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
        self.dev.send_stream(&stream, timeout_ms)
    }

    /// 取出一帧解码结果。返回的 [`DecodedFrame`] 借用本 decoder，并在析构时
    /// 释放对应的 VB block。
    pub fn get_frame<'s>(&'s self, timeout_ms: ffi::CVI_S32) -> Result<DecodedFrame<'s>> {
        let mut info: ffi::VIDEO_FRAME_INFO_S = unsafe { mem::zeroed() };
        self.dev.get_frame(&mut info, timeout_ms)?;

        // 与 C 封装一致：内核只回填物理地址/stride/长度，虚拟地址由用户态映射；
        // `CVI_SYS_MmapCache` 是 cached 映射 + `IonInvalidateCache`（DMA 写完的
        // 数据对 CPU 立即可见），映射长度用驱动给的 `u32Length`。
        let mut maps: [Option<(*mut c_void, u32)>; 3] = [None; 3];
        for p in 0..3 {
            let phy = info.stVFrame.u64PhyAddr[p];
            let len = info.stVFrame.u32Length[p];
            if phy == 0 || len == 0 {
                continue;
            }
            let ptr = unsafe { ffi::CVI_SYS_MmapCache(phy, len) };
            if ptr.is_null() {
                unmap_planes(&mut maps);
                let _ = self.dev.release_frame(&info);
                return Err(Error::invalid("CVI_VDEC_GetFrame (CVI_SYS_MmapCache)"));
            }
            info.stVFrame.pu8VirAddr[p] = ptr.cast();
            maps[p] = Some((ptr, len));
        }

        Ok(DecodedFrame {
            dev: &self.dev,
            info,
            maps,
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
        // 与 C 封装一致：先停收流再销毁通道，失败忽略；fd 由 `VdecDev` 析构关闭。
        let _ = self.dev.stop_recv_stream();
        let _ = self.dev.destroy_chn();
    }
}

/// 解除 `CVI_VDEC_GetFrame` 建立的所有平面映射（幂等）。
fn unmap_planes(maps: &mut [Option<(*mut c_void, u32)>; 3]) {
    for slot in maps.iter_mut() {
        if let Some((ptr, len)) = slot.take() {
            unsafe {
                ffi::CVI_SYS_Munmap(ptr, len);
            }
        }
    }
}

/// 由 decoder 的 VB 池持有的解码帧；析构时解除平面映射并释放。
///
/// 借用产生它的 [`Decoder`]（见 [`Decoder::get_frame`]）。
pub struct DecodedFrame<'s> {
    dev: &'s VdecDev,
    info: ffi::VIDEO_FRAME_INFO_S,
    /// `CVI_VDEC_GetFrame` 建立的平面映射：`(地址, 长度)`。
    maps: [Option<(*mut c_void, u32)>; 3],
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
        unmap_planes(&mut self.maps);
        let _ = self.dev.release_frame(&self.info);
    }
}

// SAFETY: 理由同 `encoder::Frame` —— 平面裸指针指向中间件持有的 VB 内存，
// 且不携带任何线程局部状态，因此把帧移动到别的线程是安全的。
unsafe impl Send for DecodedFrame<'_> {}

/* ------------------------------------------------------------------ */
/* 设备层（/dev/cvi_vc_dec{chn} 的 ioctl 直连）                        */
/* ------------------------------------------------------------------ */

/// VDEC 设备节点前缀（`CVI_VC_DRV_DECODER_DEV_NAME`）。
const VDEC_DEV_PREFIX: &str = "/dev/cvi_vc_dec";

/// 一条 VDEC 通道的内核句柄（一个 `/dev/cvi_vc_dec{chn}` fd）。
///
/// 厂商的 `libvdec.so` 本身就是「open 设备 + ioctl」的浅封装（见 cvi_mpi 的
/// `modules/vdec/src/cvi_vdec.c`），所以这里按
/// `include/linux/cvi_vc_drv_ioctl.h` 的协议直接实现：全部命令都是 `_IO`
/// （type 字符 `'V'`，命令号 = `0x5600 | nr`），`SendStream` / `GetFrame` 的
/// 载荷是「结构体指针 + `s32MilliSec`」的 EX 包装。
///
/// [`VdecDev::create`] 完成 `open` + `CVI_VC_VDEC_CREATE_CHN`；析构只关 fd，
/// `StopRecvStream` / `DestroyChn` 由 [`Decoder`] 的 `Drop` 显式调用。
struct VdecDev {
    fd: RawFd,
    chn: ffi::VDEC_CHN,
}

impl VdecDev {
    /// `open("/dev/cvi_vc_dec{chn}")` + `CVI_VC_VDEC_CREATE_CHN`。
    fn create(chn: ffi::VDEC_CHN, attr: &ffi::VDEC_CHN_ATTR_S) -> Result<Self> {
        let fd = open_device(VDEC_DEV_PREFIX, chn, "open(/dev/cvi_vc_dec*)")?;
        let dev = Self { fd, chn };
        ioctl_write(fd, ffi::CVI_VC_VDEC_CREATE_CHN, attr, "CVI_VDEC_CreateChn")?;
        Ok(dev)
    }

    fn channel(&self) -> ffi::VDEC_CHN {
        self.chn
    }

    /// `CVI_VDEC_StartRecvStream`。
    fn start_recv_stream(&self) -> Result<()> {
        ioctl_none(
            self.fd,
            ffi::CVI_VC_VDEC_START_RECV_STREAM,
            "CVI_VDEC_StartRecvStream",
        )
    }

    /// `CVI_VDEC_StopRecvStream`。
    fn stop_recv_stream(&self) -> Result<()> {
        ioctl_none(
            self.fd,
            ffi::CVI_VC_VDEC_STOP_RECV_STREAM,
            "CVI_VDEC_StopRecvStream",
        )
    }

    /// `CVI_VDEC_DestroyChn`。
    fn destroy_chn(&self) -> Result<()> {
        ioctl_none(self.fd, ffi::CVI_VC_VDEC_DESTROY_CHN, "CVI_VDEC_DestroyChn")
    }

    /// `CVI_VDEC_SetChnParam`。
    fn set_chn_param(&self, param: &ffi::VDEC_CHN_PARAM_S) -> Result<()> {
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VDEC_SET_CHN_PARAM,
            param,
            "CVI_VDEC_SetChnParam",
        )
    }

    /// `CVI_VDEC_GetChnParam`。
    fn get_chn_param(&self, param: &mut ffi::VDEC_CHN_PARAM_S) -> Result<()> {
        ioctl_read(
            self.fd,
            ffi::CVI_VC_VDEC_GET_CHN_PARAM,
            param,
            "CVI_VDEC_GetChnParam",
        )
    }

    /// `CVI_VDEC_SendStream`。
    fn send_stream(&self, stream: &ffi::VDEC_STREAM_S, timeout_ms: ffi::CVI_S32) -> Result<()> {
        let ex = ffi::VDEC_STREAM_EX_S {
            pstStream: stream,
            s32MilliSec: timeout_ms,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VDEC_SEND_STREAM,
            &ex,
            "CVI_VDEC_SendStream",
        )
    }

    /// `CVI_VDEC_GetFrame`：内核把帧描述（物理地址/stride/长度）填进 `info`。
    fn get_frame(
        &self,
        info: &mut ffi::VIDEO_FRAME_INFO_S,
        timeout_ms: ffi::CVI_S32,
    ) -> Result<()> {
        let ex = ffi::VIDEO_FRAME_INFO_EX_S {
            pstFrame: info,
            s32MilliSec: timeout_ms,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VDEC_GET_FRAME,
            &ex,
            "CVI_VDEC_GetFrame",
        )
    }

    /// `CVI_VDEC_ReleaseFrame`。
    fn release_frame(&self, info: &ffi::VIDEO_FRAME_INFO_S) -> Result<()> {
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VDEC_RELEASE_FRAME,
            info,
            "CVI_VDEC_ReleaseFrame",
        )
    }
}

impl Drop for VdecDev {
    fn drop(&mut self) {
        if self.fd >= 0 {
            unsafe { libc::close(self.fd) };
            self.fd = -1;
        }
    }
}
