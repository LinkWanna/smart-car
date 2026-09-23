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
use std::os::unix::io::RawFd;

use crate::ffi;
use crate::sys::Sys;
use crate::{
    Error, FrameLayout, Result, SessionRef, check, ioctl_none, ioctl_read, ioctl_write,
    open_device, plane_dims, tight_frame_size, venc_input_layout,
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
                unsafe { ffi::CVI_SYS_IonFlushCache(v.u64PhyAddr[p], vir.cast(), v.u32Length[p]) },
                "CVI_SYS_IonFlushCache",
            )?;
        }
        Ok(())
    }

    /// 底层帧描述，例如可直接传给 `CVI_VENC_SendFrame`。
    pub fn info(&self) -> &ffi::VIDEO_FRAME_INFO_S {
        &self.info
    }

    /// 设置帧的 PTS。某些链路（VPSS→VENC bind）会把它透传进码流，
    /// 便于把编码结果和采集帧对齐（见 `Encoder::get_stream_meta`）。
    pub fn set_pts(&mut self, pts: u64) {
        self.info.stVFrame.u64PTS = pts;
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

    /// 平面物理地址（交给硬件用；`0` 表示该平面不存在）。
    ///
    /// 典型用途是喂 TPU（`CVI_NN_SetTensorPhysicalAddr`）：RGB 平面帧的
    /// 三个平面物理地址连续、stride 等于宽度，正好是模型要的 NCHW 布局。
    pub fn phy_addr(&self, plane: usize) -> u64 {
        self.info.stVFrame.u64PhyAddr[plane]
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

    /// 三个平面的只读视图（未映射的平面为空切片）。
    ///
    /// 一次读取多个平面（如把 RGB 平面帧打包成紧凑缓冲）时用它，
    /// 免去逐个 [`Frame::plane_mut`] 的借用冲突。
    pub fn planes(&self) -> [&[u8]; 3] {
        let v = &self.info.stVFrame;
        let ptrs = [
            v.pu8VirAddr[0].cast_const(),
            v.pu8VirAddr[1].cast_const(),
            v.pu8VirAddr[2].cast_const(),
        ];
        let lens = [
            v.u32Length[0] as usize,
            v.u32Length[1] as usize,
            v.u32Length[2] as usize,
        ];
        // SAFETY: 三个平面是三次独立映射的区间，互不重叠；`&self` 保证没有
        // 可变借用同时存在（见 `planes_mut` 的说明）。
        [
            unsafe { plane_slice(ptrs[0], lens[0]) },
            unsafe { plane_slice(ptrs[1], lens[1]) },
            unsafe { plane_slice(ptrs[2], lens[2]) },
        ]
    }

    /// 三个平面同时可变借用（未映射的平面为空切片）。
    ///
    /// 适合一次遍历写多个平面的转换（如 YUYV422 → RGB 平面）；
    /// 单平面写入用 [`Frame::plane_mut`] 即可。
    pub fn planes_mut(&mut self) -> [&mut [u8]; 3] {
        let v = &mut self.info.stVFrame;
        let ptrs = [v.pu8VirAddr[0], v.pu8VirAddr[1], v.pu8VirAddr[2]];
        let lens = [
            v.u32Length[0] as usize,
            v.u32Length[1] as usize,
            v.u32Length[2] as usize,
        ];
        // SAFETY: `&mut self` 保证独占；三个平面是三次独立映射的区间，互不重叠。
        [
            unsafe { plane_slice_mut(ptrs[0], lens[0]) },
            unsafe { plane_slice_mut(ptrs[1], lens[1]) },
            unsafe { plane_slice_mut(ptrs[2], lens[2]) },
        ]
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

/// SAFETY: `ptr`/`len` 必须描述一段有效映射，或 `ptr` 为空 / `len` 为 0。
unsafe fn plane_slice<'x>(ptr: *const u8, len: usize) -> &'x [u8] {
    if ptr.is_null() || len == 0 {
        &[]
    } else {
        unsafe { slice::from_raw_parts(ptr, len) }
    }
}

/// SAFETY: 同 [`plane_slice`]；调用方还要保证各平面区间互不重叠。
unsafe fn plane_slice_mut<'x>(ptr: *mut u8, len: usize) -> &'x mut [u8] {
    if ptr.is_null() || len == 0 {
        // 空切片也要非空指针（`from_raw_parts_mut` 的前置条件）。
        let dangling = core::ptr::NonNull::<u8>::dangling().as_ptr();
        unsafe { slice::from_raw_parts_mut(dangling, 0) }
    } else {
        unsafe { slice::from_raw_parts_mut(ptr, len) }
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

/// 一条 `PT_JPEG` 编码通道（`CVI_VENC_*`，直连 `/dev/cvi_vc_enc{chn}`）。
///
/// 由 [`Sys::create_encoder`] 创建；析构时停止并销毁通道。
pub struct Encoder<'a> {
    dev: VencDev,
    _session: SessionRef<'a>,
}

/// `CVI_VENC_GetStream` 的元信息（诊断 / 对帧用）。
#[derive(Debug, Clone, Copy, Default)]
pub struct StreamMeta {
    /// VENC 内部的编码序号（每编一帧 +1）。
    pub seq: u32,
    /// 第一个 pack 的 PTS（bind 链路上等于源帧的 PTS，如果驱动透传）。
    pub pts: u64,
}

impl<'a> Encoder<'a> {
    /// 创建通道、设置 JPEG 质量并开始接收帧。
    ///
    /// bind 场景（VPSS→VENC）请用 [`Sys::create_encoder_pending`] +
    /// [`Encoder::start_recv_frame`]：SDK sample 要求**先 bind 再开始收帧**，
    /// 顺序反了绑定后 `GetStream` 会报 `EN_ERR_BUSY`。
    pub(crate) fn create(
        _sys: &'a Sys,
        chn: ffi::VENC_CHN,
        cfg: &EncoderConfig,
    ) -> Result<Encoder<'a>> {
        let enc = Self::create_pending(_sys, chn, cfg)?;
        enc.start_recv_frame(cfg.recv_pic_num)?;
        Ok(enc)
    }

    /// 只创建通道并设置 JPEG 质量，**不**开始收帧（见 [`Encoder::start_recv_frame`]）。
    pub(crate) fn create_pending(
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

        let dev = VencDev::create(chn, &attr)?;
        let enc = Encoder {
            dev,
            _session: PhantomData,
        };
        if let Some(q) = cfg.quality {
            enc.set_quality(q)?;
        }
        Ok(enc)
    }

    /// 开始接收帧（`CVI_VENC_StartRecvFrame`）。`-1` 表示不限帧数。
    pub fn start_recv_frame(&self, recv_pic_num: i32) -> Result<()> {
        self.dev.start_recv_frame(recv_pic_num)
    }

    pub fn channel(&self) -> ffi::VENC_CHN {
        self.dev.channel()
    }

    /// 用 `CVI_VENC_SetJpegParam` 设置新质量（`1..=99`，50 为标准量化表）。
    pub fn set_quality(&self, quality: u32) -> Result<()> {
        let p = ffi::VENC_JPEG_PARAM_S {
            u32Qfactor: quality,
            ..Default::default()
        };
        self.dev.set_jpeg_param(&p)
    }

    /// 用 `CVI_VENC_SetJpegParam` 设置自定义量化表。
    pub fn set_jpeg_param(&self, param: &ffi::VENC_JPEG_PARAM_S) -> Result<()> {
        self.dev.set_jpeg_param(param)
    }

    /// 读回当前 JPEG 参数（`CVI_VENC_GetJpegParam`，诊断用）。
    pub fn get_jpeg_param(&self) -> Result<ffi::VENC_JPEG_PARAM_S> {
        let mut param = ffi::VENC_JPEG_PARAM_S::default();
        self.dev.get_jpeg_param(&mut param)?;
        Ok(param)
    }

    /// 提交一帧输入（`CVI_VENC_SendFrame`）。
    ///
    /// cached 帧会先 [`Frame::flush`]（uncached 帧 no-op）。
    pub fn send_frame(&self, frame: &Frame<'_>, timeout_ms: ffi::CVI_S32) -> Result<()> {
        frame.flush()?;
        self.dev.send_frame(frame.info(), timeout_ms)
    }

    /// 提交任意一帧 `VIDEO_FRAME_INFO_S`（例如 VPSS 通道输出帧）。
    ///
    /// 与 [`Encoder::send_frame`] 的区别：不经过 [`Frame`]，不做 flush ——
    /// 帧内存由硬件写入时（VPSS/VDEC 输出）无需 CPU 侧缓存操作。
    /// VENC 只在编码完成前引用这块 VB 内存，所以帧的持有者要等
    /// `GetStream` 之后再释放（`send_frame_info` + `get_stream` 是同一个约定）。
    pub fn send_frame_info(
        &self,
        frame: &ffi::VIDEO_FRAME_INFO_S,
        timeout_ms: ffi::CVI_S32,
    ) -> Result<()> {
        self.dev.send_frame(frame, timeout_ms)
    }

    /// `send_frame_info` + `get_stream`。
    pub fn encode_info(
        &self,
        frame: &ffi::VIDEO_FRAME_INFO_S,
        timeout_ms: ffi::CVI_S32,
    ) -> Result<Vec<u8>> {
        self.send_frame_info(frame, timeout_ms)?;
        self.get_stream(timeout_ms)
    }

    /// 取出一帧编码后的 JPEG 码流（`CVI_VENC_GetStream` + `CVI_VENC_ReleaseStream`）。
    /// 每次调用消费一帧已提交的输入。
    pub fn get_stream(&self, timeout_ms: ffi::CVI_S32) -> Result<Vec<u8>> {
        self.get_stream_meta(timeout_ms).map(|(jpeg, _)| jpeg)
    }

    /// 同 [`Encoder::get_stream`]，但额外返回码流元信息（VENC 序号 + PTS），
    /// 用于把编码结果和输入帧对齐（bind 链路上用户态看不到输入帧）。
    pub fn get_stream_meta(&self, timeout_ms: ffi::CVI_S32) -> Result<(Vec<u8>, StreamMeta)> {
        let mut packs: [ffi::VENC_PACK_S; MAX_STREAM_PACKS] = unsafe { mem::zeroed() };
        let mut stream: ffi::VENC_STREAM_S = unsafe { mem::zeroed() };
        stream.pstPack = packs.as_mut_ptr();
        stream.u32PackCount = MAX_STREAM_PACKS as u32;

        self.dev.get_stream(&mut stream, timeout_ms)?;

        // 内核只回填物理地址；逐 pack 映射拷出（见 `copy_stream_packs`）。
        // 无论拷贝成功与否，都要把码流缓冲还给内核。
        let copied = copy_stream_packs(&stream);
        let released = self.dev.release_stream(&mut stream);
        match (copied, released) {
            (Ok(v), Ok(())) => Ok(v),
            (Err(e), _) => Err(e),
            (Ok(_), Err(e)) => Err(e),
        }
    }

    /// `send_frame` + `get_stream`。
    pub fn encode(&self, frame: &Frame<'_>, timeout_ms: ffi::CVI_S32) -> Result<Vec<u8>> {
        self.send_frame(frame, timeout_ms)?;
        self.get_stream(timeout_ms)
    }
}

/// 把 `CVI_VENC_GetStream` 回填的各 pack 拷成一个连续缓冲。
///
/// 与 C 封装一致：每个 pack 的 `u64PhyAddr` 经 `/dev/mem`（libsys 的
/// `CVI_SYS_MmapCache`，cached + invalidate）映射后读取 `u32Len` 字节，
/// 读完立即解除映射；`pu8Addr` 保持内核回填的原值（C 封装也是把它原样还给
/// `ReleaseStream`）。映射/拷贝失败时不会留下未解除的映射。
fn copy_stream_packs(stream: &ffi::VENC_STREAM_S) -> Result<(Vec<u8>, StreamMeta)> {
    let mut out = Vec::new();
    let mut meta = StreamMeta {
        seq: stream.u32Seq,
        pts: 0,
    };
    if stream.pstPack.is_null() {
        return Ok((out, meta));
    }

    // SAFETY: 内核把 `pstPack`（调用方提供的数组）填成 `u32PackCount` 个有效
    // `VENC_PACK_S`；数量不会超过调用时给的容量。
    let count = stream.u32PackCount.min(MAX_STREAM_PACKS as u32) as usize;
    for i in 0..count {
        let pack = unsafe { &*stream.pstPack.add(i) };
        if i == 0 {
            meta.pts = pack.u64PTS;
        }
        if pack.u64PhyAddr == 0 || pack.u32Len == 0 {
            continue;
        }

        let ptr = unsafe { ffi::CVI_SYS_MmapCache(pack.u64PhyAddr, pack.u32Len) };
        if ptr.is_null() {
            return Err(Error::invalid("CVI_VENC_GetStream (CVI_SYS_MmapCache)"));
        }
        // SAFETY: 刚映射了 `u32Len` 字节，且归本次拷贝所有。
        unsafe {
            out.extend_from_slice(slice::from_raw_parts(
                ptr.cast::<u8>(),
                pack.u32Len as usize,
            ));
            ffi::CVI_SYS_Munmap(ptr, pack.u32Len);
        }
    }
    Ok((out, meta))
}

impl Drop for Encoder<'_> {
    fn drop(&mut self) {
        // 与 C 封装（`CVI_VENC_DestroyChn`）一致：先停收帧再销毁通道，失败忽略
        // （通道可能已被内核清掉）；fd 由 `VencDev` 析构关闭。
        let _ = self.dev.stop_recv_frame();
        let _ = self.dev.destroy_chn();
    }
}

/* ------------------------------------------------------------------ */
/* 设备层（/dev/cvi_vc_enc{chn} 的 ioctl 直连）                        */
/* ------------------------------------------------------------------ */

/// VENC 设备节点前缀（`CVI_VC_DRV_ENCODER_DEV_NAME`）。
const VENC_DEV_PREFIX: &str = "/dev/cvi_vc_enc";

/// 一条 VENC 通道的内核句柄（一个 `/dev/cvi_vc_enc{chn}` fd）。
///
/// 厂商的 `libvenc.so` 本身就是「open 设备 + ioctl」的浅封装（见 cvi_mpi 的
/// `modules/venc/src/cvi_venc.c`），所以这里按
/// `include/linux/cvi_vc_drv_ioctl.h` 的协议直接实现：全部命令都是 `_IO`
/// （type 字符 `'V'`，命令号 = `0x5600 | nr`），`SendFrame` / `GetStream` 的
/// 载荷是「结构体指针 + `s32MilliSec`」的 EX 包装。
///
/// [`VencDev::create`] 完成 `open` + `CVI_VC_VENC_CREATE_CHN`；析构时只关 fd，
/// 通道的停止/销毁由 [`Encoder`] 的 `Drop` 调用 [`VencDev::stop_recv_frame`] /
/// [`VencDev::destroy_chn`]（与 C 封装 `CVI_VENC_DestroyChn` 的顺序一致）。
struct VencDev {
    fd: RawFd,
    chn: ffi::VENC_CHN,
}

impl VencDev {
    /// `open("/dev/cvi_vc_enc{chn}")` + `CVI_VC_VENC_CREATE_CHN`。
    fn create(chn: ffi::VENC_CHN, attr: &ffi::VENC_CHN_ATTR_S) -> Result<Self> {
        let fd = open_device(VENC_DEV_PREFIX, chn, "open(/dev/cvi_vc_enc*)")?;
        let dev = Self { fd, chn };
        ioctl_write(fd, ffi::CVI_VC_VENC_CREATE_CHN, attr, "CVI_VENC_CreateChn")?;
        Ok(dev)
    }

    fn channel(&self) -> ffi::VENC_CHN {
        self.chn
    }

    /// `CVI_VENC_StartRecvFrame`（`-1` 表示不限帧数）。
    fn start_recv_frame(&self, recv_pic_num: ffi::CVI_S32) -> Result<()> {
        let recv = ffi::VENC_RECV_PIC_PARAM_S {
            s32RecvPicNum: recv_pic_num,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VENC_START_RECV_FRAME,
            &recv,
            "CVI_VENC_StartRecvFrame",
        )
    }

    /// `CVI_VENC_StopRecvFrame`。
    fn stop_recv_frame(&self) -> Result<()> {
        ioctl_none(
            self.fd,
            ffi::CVI_VC_VENC_STOP_RECV_FRAME,
            "CVI_VENC_StopRecvFrame",
        )
    }

    /// `CVI_VENC_DestroyChn`。
    fn destroy_chn(&self) -> Result<()> {
        ioctl_none(self.fd, ffi::CVI_VC_VENC_DESTROY_CHN, "CVI_VENC_DestroyChn")
    }

    /// `CVI_VENC_SendFrame`。
    fn send_frame(&self, frame: &ffi::VIDEO_FRAME_INFO_S, timeout_ms: ffi::CVI_S32) -> Result<()> {
        let ex = ffi::VIDEO_FRAME_INFO_EX_S {
            pstFrame: frame,
            s32MilliSec: timeout_ms,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VENC_SEND_FRAME,
            &ex,
            "CVI_VENC_SendFrame",
        )
    }

    /// `CVI_VENC_GetStream`：内核把 pack 列表填进 `stream.pstPack`。
    fn get_stream(&self, stream: &mut ffi::VENC_STREAM_S, timeout_ms: ffi::CVI_S32) -> Result<()> {
        let ex = ffi::VENC_STREAM_EX_S {
            pstStream: stream,
            s32MilliSec: timeout_ms,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VENC_GET_STREAM,
            &ex,
            "CVI_VENC_GetStream",
        )
    }

    /// `CVI_VENC_ReleaseStream`。
    fn release_stream(&self, stream: &mut ffi::VENC_STREAM_S) -> Result<()> {
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VENC_RELEASE_STREAM,
            stream,
            "CVI_VENC_ReleaseStream",
        )
    }

    /// `CVI_VENC_SetJpegParam`。
    fn set_jpeg_param(&self, param: &ffi::VENC_JPEG_PARAM_S) -> Result<()> {
        ioctl_write(
            self.fd,
            ffi::CVI_VC_VENC_SET_JPEG_PARAM,
            param,
            "CVI_VENC_SetJpegParam",
        )
    }

    /// `CVI_VENC_GetJpegParam`。
    fn get_jpeg_param(&self, param: &mut ffi::VENC_JPEG_PARAM_S) -> Result<()> {
        ioctl_read(
            self.fd,
            ffi::CVI_VC_VENC_GET_JPEG_PARAM,
            param,
            "CVI_VENC_GetJpegParam",
        )
    }
}

impl Drop for VencDev {
    fn drop(&mut self) {
        if self.fd >= 0 {
            unsafe { libc::close(self.fd) };
            self.fd = -1;
        }
    }
}
