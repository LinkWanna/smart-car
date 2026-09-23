//! `PT_JPEG` 编码通道（`CVI_VENC_*`）及其基于 VB 的输入帧。
//!
//! [`Encoder`] 和 [`Frame`] 都借用创建它们的 [`Sys`] 会话，因此不可能比会话
//! （以及其中的 VB 池）活得更久：
//!
//! ```no_run
//! use cvimpi_rs::{
//!     encoder::EncoderConfig, ffi,
//!     sys::{Sys, VbPoolConfig},
//!     venc_input_layout,
//! };
//!
//! # fn main() -> Result<(), cvimpi_rs::Error> {
//! let layout = venc_input_layout(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420).unwrap();
//! let sys = Sys::init(&[VbPoolConfig::new(layout.vb_size, 3)])?;
//!
//! let enc = sys.create_encoder(
//!     0,
//!     &EncoderConfig::new(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420).with_quality(85),
//! )?;
//! let mut frame = sys.alloc_frame(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420)?;
//! frame.write_tight(&vec![0u8; (1920 * 1080 * 3 / 2) as usize])?;
//! let jpeg_data = enc.encode(&frame, ffi::CVI_IO_BLOCK)?;
//! # Ok(())
//! # }
//! ```

use core::ffi::c_void;
use core::marker::PhantomData;
use core::{mem, slice};

use crate::ffi;
use crate::sys::Sys;
use crate::{
    Error, FrameLayout, Result, SessionRef, check, plane_dims, tight_frame_size, venc_input_layout,
};

#[inline]
const fn align_up64(x: u64, a: u64) -> u64 {
    (x + a - 1) / a * a
}

/// `CVI_VENC_GetStream` 的 pack 数组容量。`VENC_STREAM_S.pstPack` 是
/// **调用方提供**的数组（中间件只填内容、不改指针），所以不能传空结构体；
/// JPEG 单帧通常只有 1 个 pack（带 MPF 缩略图时多几个），8 个足够。
const MAX_STREAM_PACKS: usize = 8;

/* ------------------------------------------------------------------ */
/* 编码输入帧                                                          */
/* ------------------------------------------------------------------ */

/// 基于 VB 的帧，可直接交给 JPEG 编码器（也可以用于其他 `SendFrame` 接口）。
///
/// 由 [`Sys::alloc_frame`]（uncached，免 flush）或 [`Sys::alloc_frame_cached`]
/// （`MmapCache`，CPU 写入快，发送前由 [`Encoder::send_frame`] 自动 flush）创建；
/// 析构时归还给 VB 池。
pub struct Frame<'a> {
    info: ffi::VIDEO_FRAME_INFO_S,
    layout: FrameLayout,
    blk: ffi::VB_BLK,
    maps: [Option<(*mut c_void, u32)>; 3],
    /// 平面是否用 `CVI_SYS_MmapCache` 映射（见 [`Frame::flush`]）。
    cached: bool,
    _session: SessionRef<'a>,
}

impl<'a> Frame<'a> {
    /// 从 common VB 池申请一个 block 并映射各平面（uncached）。
    pub(crate) fn alloc(
        _sys: &'a Sys,
        width: u32,
        height: u32,
        fmt: ffi::PIXEL_FORMAT_E,
    ) -> Result<Frame<'a>> {
        Self::alloc_with(_sys, width, height, fmt, false)
    }

    /// 同 [`Frame::alloc`]，但平面用 `CVI_SYS_MmapCache` 映射。
    ///
    /// CPU 逐字节写入 cached 内存比 uncached 快一个数量级（实测 YUYV→NV12
    /// 写 460KB：25ms → 3.3ms）；代价是交给硬件前必须 [`Frame::flush`]，
    /// [`Encoder::send_frame`] 会自动调用。
    pub(crate) fn alloc_cached(
        _sys: &'a Sys,
        width: u32,
        height: u32,
        fmt: ffi::PIXEL_FORMAT_E,
    ) -> Result<Frame<'a>> {
        Self::alloc_with(_sys, width, height, fmt, true)
    }

    fn alloc_with(
        _sys: &'a Sys,
        width: u32,
        height: u32,
        fmt: ffi::PIXEL_FORMAT_E,
        cached: bool,
    ) -> Result<Frame<'a>> {
        let layout = venc_input_layout(width, height, fmt)
            .ok_or_else(|| Error::invalid("venc_input_layout (unsupported pixel format)"))?;

        let blk = unsafe { ffi::CVI_VB_GetBlock(ffi::VB_INVALID_POOLID, layout.vb_size) };
        if blk == ffi::VB_INVALID_HANDLE {
            return Err(Error::invalid("CVI_VB_GetBlock (no pool or out of blocks)"));
        }

        let mut info: ffi::VIDEO_FRAME_INFO_S = unsafe { mem::zeroed() };
        let v = &mut info.stVFrame;
        v.u32Width = width;
        v.u32Height = height;
        v.enPixelFormat = fmt;
        v.enVideoFormat = ffi::VIDEO_FORMAT_LINEAR;
        v.enCompressMode = ffi::COMPRESS_MODE_NONE;
        v.enDynamicRange = ffi::DYNAMIC_RANGE_SDR8;
        v.enColorGamut = ffi::COLOR_GAMUT_BT709;
        v.u64PTS = 0;
        v.u32TimeRef = 0;
        info.u32PoolId = unsafe { ffi::CVI_VB_Handle2PoolId(blk) };

        let mut frame = Frame {
            info,
            layout,
            blk,
            maps: [None; 3],
            cached,
            _session: PhantomData,
        };

        let base = unsafe { ffi::CVI_VB_Handle2PhysAddr(blk) };
        if base == 0 {
            return Err(Error::invalid("CVI_VB_Handle2PhysAddr"));
        }

        let plane0 = &mut frame.info.stVFrame;
        plane0.u64PhyAddr[0] = base;
        plane0.u32Stride[0] = layout.main_stride;
        plane0.u32Length[0] = layout.y_size;
        frame.map_plane(0, base, layout.y_size)?;

        let mut phy = base;
        let mut len = layout.y_size;
        for p in 1..layout.plane_num as usize {
            phy = align_up64(phy + len as u64, layout.addr_align as u64);
            frame.info.stVFrame.u32Stride[p] = layout.c_stride;
            frame.info.stVFrame.u32Length[p] = layout.c_size;
            frame.info.stVFrame.u64PhyAddr[p] = phy;
            frame.map_plane(p, phy, layout.c_size)?;
            len = layout.c_size;
        }

        Ok(frame)
    }

    fn map_plane(&mut self, plane: usize, phy: u64, len: u32) -> Result<()> {
        let ptr = unsafe {
            if self.cached {
                ffi::CVI_SYS_MmapCache(phy, len)
            } else {
                ffi::CVI_SYS_Mmap(phy, len)
            }
        };
        if ptr.is_null() {
            return Err(Error::invalid("CVI_SYS_Mmap"));
        }
        self.info.stVFrame.pu8VirAddr[plane] = ptr.cast();
        self.maps[plane] = Some((ptr, len));
        Ok(())
    }

    /// 帧的平面是否为 cached 映射。
    pub fn is_cached(&self) -> bool {
        self.cached
    }

    /// 把 CPU 对平面的写入刷到内存（cached 帧交给硬件前必须调用）。
    ///
    /// uncached 帧是 no-op；[`Encoder::send_frame`] 会自动调用它。
    pub fn flush(&self) -> Result<()> {
        if !self.cached {
            return Ok(());
        }
        let v = &self.info.stVFrame;
        for p in 0..self.layout.plane_num as usize {
            let vir = v.pu8VirAddr[p];
            if vir.is_null() || v.u32Length[p] == 0 {
                continue;
            }
            check(
                unsafe {
                    ffi::CVI_SYS_IonFlushCache(v.u64PhyAddr[p], vir.cast(), v.u32Length[p])
                },
                "CVI_SYS_IonFlushCache",
            )?;
        }
        Ok(())
    }

    /// 底层帧描述，例如可直接传给 `CVI_VENC_SendFrame`。
    pub fn info(&self) -> &ffi::VIDEO_FRAME_INFO_S {
        &self.info
    }

    pub fn layout(&self) -> &FrameLayout {
        &self.layout
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

    pub fn plane_len(&self, plane: usize) -> u32 {
        self.info.stVFrame.u32Length[plane]
    }

    /// 整个平面（含 stride）的可变视图。
    pub fn plane_mut(&mut self, plane: usize) -> Result<&mut [u8]> {
        if plane >= 3 {
            return Err(Error::invalid("plane_mut (plane index)"));
        }
        let ptr = self.info.stVFrame.pu8VirAddr[plane];
        let len = self.info.stVFrame.u32Length[plane] as usize;
        if ptr.is_null() || len == 0 {
            return Err(Error::invalid("plane_mut (plane not mapped)"));
        }
        // SAFETY: 该平面由 `map_plane` 映射了恰好 `len` 字节，且归本 `Frame` 所有。
        Ok(unsafe { slice::from_raw_parts_mut(ptr, len) })
    }

    /// 把紧凑排布的帧数据（Y，然后 U/V 或交织的 UV）拷入已映射的平面，
    /// 自动按硬件 stride 逐行搬运。
    pub fn write_tight(&mut self, data: &[u8]) -> Result<()> {
        let dims = plane_dims(self.pixel_format(), self.width(), self.height())
            .ok_or_else(|| Error::invalid("write_tight (unsupported pixel format)"))?;
        let need = tight_frame_size(self.pixel_format(), self.width(), self.height())
            .ok_or_else(|| Error::invalid("write_tight (size overflow)"))?
            as usize;
        if data.len() < need {
            return Err(Error::invalid("write_tight (input buffer too small)"));
        }

        let mut off = 0usize;
        for p in 0..dims.planes {
            let row = dims.row_bytes[p] as usize;
            let rows = dims.rows[p] as usize;
            let stride = self.stride(p) as usize;
            let plane = self.plane_mut(p)?;
            for r in 0..rows {
                let dst = &mut plane[r * stride..r * stride + row];
                dst.copy_from_slice(&data[off..off + row]);
                off += row;
            }
        }
        Ok(())
    }
}

impl Drop for Frame<'_> {
    fn drop(&mut self) {
        for (ptr, len) in self.maps.iter().flatten() {
            unsafe {
                ffi::CVI_SYS_Munmap(*ptr, *len);
            }
        }
        if self.blk != ffi::VB_INVALID_HANDLE {
            unsafe {
                ffi::CVI_VB_ReleaseBlock(self.blk);
            }
        }
    }
}

// SAFETY: `Frame` 只保存指向 VB/ION 内存的地址和一些与线程无关的簿记信息，
// 移动到别的线程不会引入数据竞争。析构路径（`CVI_SYS_Munmap` /
// `CVI_VB_ReleaseBlock`）本身也与线程无关。
unsafe impl Send for Frame<'_> {}

/* ------------------------------------------------------------------ */
/* JPEG 编码器                                                         */
/* ------------------------------------------------------------------ */

/// JPEG 编码通道的配置。
#[derive(Debug, Clone, Copy)]
pub struct EncoderConfig {
    pub width: u32,
    pub height: u32,
    pub pixel_format: ffi::PIXEL_FORMAT_E,
    /// 硬件码流缓冲大小；`None` 取 `width * height`。
    pub bitstream_buf_size: Option<u32>,
    /// JPEG 质量（`1..=99`）；`None` 表示沿用编码器内置量化表。
    pub quality: Option<u32>,
    /// 传给 `CVI_VENC_StartRecvFrame` 的 `s32RecvPicNum`；`-1` 表示不限制。
    pub recv_pic_num: i32,
}

impl EncoderConfig {
    pub fn new(width: u32, height: u32, pixel_format: ffi::PIXEL_FORMAT_E) -> Self {
        Self {
            width,
            height,
            pixel_format,
            bitstream_buf_size: None,
            quality: None,
            recv_pic_num: -1,
        }
    }

    pub fn with_quality(mut self, q: u32) -> Self {
        self.quality = Some(q);
        self
    }

    pub fn with_bitstream_buf_size(mut self, size: u32) -> Self {
        self.bitstream_buf_size = Some(size);
        self
    }
}

/// 一条 `PT_JPEG` 编码通道（`CVI_VENC_*`）。
///
/// 由 [`Sys::create_encoder`] 创建；析构时停止并销毁通道。
pub struct Encoder<'a> {
    chn: ffi::VENC_CHN,
    _session: SessionRef<'a>,
}

impl<'a> Encoder<'a> {
    /// 创建通道、设置 JPEG 质量并开始接收帧。
    pub(crate) fn create(
        _sys: &'a Sys,
        chn: ffi::VENC_CHN,
        cfg: &EncoderConfig,
    ) -> Result<Encoder<'a>> {
        let mut attr: ffi::VENC_CHN_ATTR_S = unsafe { mem::zeroed() };
        attr.stVencAttr.enType = ffi::PT_JPEG;
        attr.stVencAttr.u32MaxPicWidth = cfg.width;
        attr.stVencAttr.u32MaxPicHeight = cfg.height;
        attr.stVencAttr.u32PicWidth = cfg.width;
        attr.stVencAttr.u32PicHeight = cfg.height;
        attr.stVencAttr.u32BufSize = cfg.bitstream_buf_size.unwrap_or(cfg.width * cfg.height);
        attr.stVencAttr.bByFrame = ffi::CVI_TRUE;
        attr.stVencAttr.u32Profile = 0;
        attr.stVencAttr.stAttr.stAttrJpege = ffi::VENC_ATTR_JPEG_S::default();
        attr.stRcAttr = ffi::VENC_RC_ATTR_S::default();
        attr.stGopAttr = ffi::VENC_GOP_ATTR_S::default();

        check(
            unsafe { ffi::CVI_VENC_CreateChn(chn, &attr) },
            "CVI_VENC_CreateChn",
        )?;

        let enc = Encoder {
            chn,
            _session: PhantomData,
        };
        if let Some(q) = cfg.quality {
            enc.set_quality(q)?;
        }

        let recv = ffi::VENC_RECV_PIC_PARAM_S {
            s32RecvPicNum: cfg.recv_pic_num,
        };
        check(
            unsafe { ffi::CVI_VENC_StartRecvFrame(chn, &recv) },
            "CVI_VENC_StartRecvFrame",
        )?;
        Ok(enc)
    }

    pub fn channel(&self) -> ffi::VENC_CHN {
        self.chn
    }

    /// 用 `CVI_VENC_SetJpegParam` 设置新质量（`1..=99`，50 为标准量化表）。
    pub fn set_quality(&self, quality: u32) -> Result<()> {
        let p = ffi::VENC_JPEG_PARAM_S {
            u32Qfactor: quality,
            ..Default::default()
        };
        check(
            unsafe { ffi::CVI_VENC_SetJpegParam(self.chn, &p) },
            "CVI_VENC_SetJpegParam",
        )
    }

    /// 用 `CVI_VENC_SetJpegParam` 设置自定义量化表。
    pub fn set_jpeg_param(&self, param: &ffi::VENC_JPEG_PARAM_S) -> Result<()> {
        check(
            unsafe { ffi::CVI_VENC_SetJpegParam(self.chn, param) },
            "CVI_VENC_SetJpegParam",
        )
    }

    /// 提交一帧输入（`CVI_VENC_SendFrame`）。
    ///
    /// cached 帧会先 [`Frame::flush`]（uncached 帧 no-op）。
    pub fn send_frame(&self, frame: &Frame<'_>, timeout_ms: ffi::CVI_S32) -> Result<()> {
        frame.flush()?;
        check(
            unsafe { ffi::CVI_VENC_SendFrame(self.chn, frame.info(), timeout_ms) },
            "CVI_VENC_SendFrame",
        )
    }

    /// 取出一帧编码后的 JPEG 码流（`CVI_VENC_GetStream` + `CVI_VENC_ReleaseStream`）。
    /// 每次调用消费一帧已提交的输入。
    pub fn get_stream(&self, timeout_ms: ffi::CVI_S32) -> Result<Vec<u8>> {
        let mut packs: [ffi::VENC_PACK_S; MAX_STREAM_PACKS] = unsafe { mem::zeroed() };
        let mut stream: ffi::VENC_STREAM_S = unsafe { mem::zeroed() };
        stream.pstPack = packs.as_mut_ptr();
        stream.u32PackCount = MAX_STREAM_PACKS as u32;
        check(
            unsafe { ffi::CVI_VENC_GetStream(self.chn, &mut stream, timeout_ms) },
            "CVI_VENC_GetStream",
        )?;

        let mut out = Vec::new();
        if !stream.pstPack.is_null() {
            // SAFETY: 中间件已把 `pstPack`（指向上面的 `packs`）填成
            // `u32PackCount` 个有效的 `VENC_PACK_S`，这些数据归码流缓冲所有。
            unsafe {
                for i in 0..stream.u32PackCount.min(MAX_STREAM_PACKS as u32) as usize {
                    let pack = &*stream.pstPack.add(i);
                    if pack.pu8Addr.is_null() || pack.u32Len == 0 {
                        continue;
                    }
                    out.extend_from_slice(slice::from_raw_parts(
                        pack.pu8Addr,
                        pack.u32Len as usize,
                    ));
                }
            }
        }

        check(
            unsafe { ffi::CVI_VENC_ReleaseStream(self.chn, &mut stream) },
            "CVI_VENC_ReleaseStream",
        )?;
        Ok(out)
    }

    /// `send_frame` + `get_stream`。
    pub fn encode(&self, frame: &Frame<'_>, timeout_ms: ffi::CVI_S32) -> Result<Vec<u8>> {
        self.send_frame(frame, timeout_ms)?;
        self.get_stream(timeout_ms)
    }
}

impl Drop for Encoder<'_> {
    fn drop(&mut self) {
        unsafe {
            ffi::CVI_VENC_StopRecvFrame(self.chn);
            ffi::CVI_VENC_DestroyChn(self.chn);
        }
    }
}
