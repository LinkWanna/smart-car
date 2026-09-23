//! CVI MPI 的原始 FFI 声明（只覆盖 JPEG 编解码所需的部分）。
//!
//! 这里只声明 JPEG 编码（`VENC` + `PT_JPEG`）与解码（`VDEC` + `PT_JPEG`）一条静态图
//! 工作流需要的接口，外加初始化中间件、分配帧所需的 sys / VB 调用。
//!
//! 字段名刻意与 `../include` 下的 C 头文件保持一致，方便逐行对照。结构体布局由
//! `tools/abi_probe.c` 在同一套头文件上实测得到（64 位 LP64：riscv64 / aarch64）。
//! 32 位 ARM 的 C 结构里有额外的 `#ifdef __arm__` padding 字段，本封装未覆盖。
#![allow(non_snake_case, non_camel_case_types, dead_code)]

use core::ffi::{c_char, c_int, c_void};

/* ------------------------------------------------------------------ */
/* 基础类型（linux/cvi_type.h）                                        */
/* ------------------------------------------------------------------ */

pub type CVI_S8 = i8;
pub type CVI_U8 = u8;
pub type CVI_S16 = i16;
pub type CVI_U16 = u16;
pub type CVI_S32 = i32;
pub type CVI_U32 = u32;
pub type CVI_S64 = i64;
pub type CVI_U64 = u64;
pub type CVI_BOOL = u8;
pub type CVI_CHAR = c_char;
pub type CVI_VOID = c_void;

pub const CVI_SUCCESS: CVI_S32 = 0;
pub const CVI_FAILURE: CVI_S32 = -1;
pub const CVI_TRUE: CVI_BOOL = 1;
pub const CVI_FALSE: CVI_BOOL = 0;
/// 所有 `*MilliSec` 超时参数的阻塞模式：`-1` 表示一直等待。
///
/// 注意：C 头文件里的 `CVI_IO_BLOCK` 宏是 `CVI_TRUE`（=1），那是给布尔型参数用的；
/// 所有 `*MilliSec` 参数（GetStream / SendStream / GetFrame / SendFrame）的约定是
/// `-1` = 阻塞、`0` = 只尝试一次、`>0` = 毫秒超时
/// （见 `modules/vdec/src/cvi_vdec.c` 的注释与 C sample 的默认值）。
pub const CVI_IO_BLOCK: CVI_S32 = -1;
/// 所有 `*MilliSec` 超时参数的非阻塞模式：只尝试一次。
pub const CVI_IO_NOBLOCK: CVI_S32 = 0;

/* ------------------------------------------------------------------ */
/* 枚举（只保留与 JPEG 相关的成员）                                    */
/* ------------------------------------------------------------------ */

/// `PAYLOAD_TYPE_E`（linux/cvi_common.h）
pub type PAYLOAD_TYPE_E = c_int;
pub const PT_JPEG: PAYLOAD_TYPE_E = 26;
pub const PT_MJPEG: PAYLOAD_TYPE_E = 1002;

/// `PIXEL_FORMAT_E`（linux/cvi_comm_video.h）
pub type PIXEL_FORMAT_E = c_int;
pub const PIXEL_FORMAT_YUV_PLANAR_422: PIXEL_FORMAT_E = 12;
pub const PIXEL_FORMAT_YUV_PLANAR_420: PIXEL_FORMAT_E = 13;
pub const PIXEL_FORMAT_YUV_PLANAR_444: PIXEL_FORMAT_E = 14;
pub const PIXEL_FORMAT_YUV_400: PIXEL_FORMAT_E = 15;
pub const PIXEL_FORMAT_NV12: PIXEL_FORMAT_E = 18;
pub const PIXEL_FORMAT_NV21: PIXEL_FORMAT_E = 19;
pub const PIXEL_FORMAT_NV16: PIXEL_FORMAT_E = 20;
pub const PIXEL_FORMAT_NV61: PIXEL_FORMAT_E = 21;
pub const PIXEL_FORMAT_YUYV: PIXEL_FORMAT_E = 22;
pub const PIXEL_FORMAT_UYVY: PIXEL_FORMAT_E = 23;
pub const PIXEL_FORMAT_YVYU: PIXEL_FORMAT_E = 24;
pub const PIXEL_FORMAT_VYUY: PIXEL_FORMAT_E = 25;

/// `VIDEO_FORMAT_E`
pub type VIDEO_FORMAT_E = c_int;
pub const VIDEO_FORMAT_LINEAR: VIDEO_FORMAT_E = 0;

/// `COMPRESS_MODE_E`
pub type COMPRESS_MODE_E = c_int;
pub const COMPRESS_MODE_NONE: COMPRESS_MODE_E = 0;

/// `DYNAMIC_RANGE_E`
pub type DYNAMIC_RANGE_E = c_int;
pub const DYNAMIC_RANGE_SDR8: DYNAMIC_RANGE_E = 0;

/// `COLOR_GAMUT_E`
pub type COLOR_GAMUT_E = c_int;
pub const COLOR_GAMUT_BT601: COLOR_GAMUT_E = 0;
pub const COLOR_GAMUT_BT709: COLOR_GAMUT_E = 1;

/// `BAYER_FORMAT_E`
pub type BAYER_FORMAT_E = c_int;

/// `VIDEO_MODE_E`（linux/cvi_comm_vdec.h）
pub type VIDEO_MODE_E = c_int;
pub const VIDEO_MODE_STREAM: VIDEO_MODE_E = 0;
pub const VIDEO_MODE_FRAME: VIDEO_MODE_E = 1;

/// `VENC_PIC_RECEIVE_MODE_E`
pub type VENC_PIC_RECEIVE_MODE_E = c_int;
pub const VENC_PIC_RECEIVE_SINGLE: VENC_PIC_RECEIVE_MODE_E = 0;
pub const VENC_PIC_RECEIVE_MULTI: VENC_PIC_RECEIVE_MODE_E = 1;

/// `VB_REMAP_MODE_E`
pub type VB_REMAP_MODE_E = c_int;
pub const VB_REMAP_MODE_NONE: VB_REMAP_MODE_E = 0;
pub const VB_REMAP_MODE_NOCACHE: VB_REMAP_MODE_E = 1;
pub const VB_REMAP_MODE_CACHED: VB_REMAP_MODE_E = 2;

/// `VENC_GOP_MODE_E`
pub type VENC_GOP_MODE_E = c_int;
pub const VENC_GOPMODE_NORMALP: VENC_GOP_MODE_E = 0;

// 中间件里通道号就是普通的 CVI_S32
pub type VENC_CHN = CVI_S32;
pub type VDEC_CHN = CVI_S32;

/* ------------------------------------------------------------------ */
/* 视频帧描述                                                          */
/* ------------------------------------------------------------------ */

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SIZE_S {
    pub u32Width: CVI_U32,
    pub u32Height: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VIDEO_FRAME_S {
    pub u32Width: CVI_U32,
    pub u32Height: CVI_U32,
    pub enPixelFormat: PIXEL_FORMAT_E,
    pub enBayerFormat: BAYER_FORMAT_E,
    pub enVideoFormat: VIDEO_FORMAT_E,
    pub enCompressMode: COMPRESS_MODE_E,
    pub enDynamicRange: DYNAMIC_RANGE_E,
    pub enColorGamut: COLOR_GAMUT_E,
    pub u32Stride: [CVI_U32; 3],
    pub u64PhyAddr: [CVI_U64; 3],
    pub pu8VirAddr: [*mut CVI_U8; 3],
    pub u32Length: [CVI_U32; 3],
    pub s16OffsetTop: CVI_S16,
    pub s16OffsetBottom: CVI_S16,
    pub s16OffsetLeft: CVI_S16,
    pub s16OffsetRight: CVI_S16,
    pub u32TimeRef: CVI_U32,
    pub u64PTS: CVI_U64,
    pub pPrivateData: *mut CVI_VOID,
    pub u32FrameFlag: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VIDEO_FRAME_INFO_S {
    pub stVFrame: VIDEO_FRAME_S,
    pub u32PoolId: CVI_U32,
}

/* ------------------------------------------------------------------ */
/* VB（视频缓冲）池配置                                                */
/* ------------------------------------------------------------------ */

pub const VB_MAX_COMM_POOLS: usize = 16;
pub const MAX_VB_POOL_NAME_LEN: usize = 32;
pub const VB_INVALID_POOLID: CVI_U32 = 0xFFFF_FFFF;
/// `VB_INVALID_HANDLE` 在 C 里是 `(-1U)` 再扩宽到 `CVI_U64`，即 0xFFFF_FFFF。
pub const VB_INVALID_HANDLE: CVI_U64 = 0xFFFF_FFFF;

pub type VB_POOL = CVI_U32;
pub type VB_BLK = CVI_U64;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VB_POOL_CONFIG_S {
    pub u32BlkSize: CVI_U32,
    pub u32BlkCnt: CVI_U32,
    pub enRemapMode: VB_REMAP_MODE_E,
    pub acName: [CVI_CHAR; MAX_VB_POOL_NAME_LEN],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VB_CONFIG_S {
    pub u32MaxPoolCnt: CVI_U32,
    pub astCommPool: [VB_POOL_CONFIG_S; VB_MAX_COMM_POOLS],
}

/* ------------------------------------------------------------------ */
/* VENC                                                                */
/* ------------------------------------------------------------------ */

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_ATTR_H264_S {
    pub bRcnRefShareBuf: CVI_BOOL,
    pub bSingleLumaBuf: CVI_BOOL,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_ATTR_H265_S {
    pub bRcnRefShareBuf: CVI_BOOL,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_MPF_CFG_S {
    pub u8LargeThumbNailNum: CVI_U8,
    pub astLargeThumbNailSize: [SIZE_S; 2],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_ATTR_JPEG_S {
    pub bSupportDCF: CVI_BOOL,
    pub stMPFCfg: VENC_MPF_CFG_S,
    pub enReceiveMode: VENC_PIC_RECEIVE_MODE_E,
}

impl Default for VENC_ATTR_JPEG_S {
    fn default() -> Self {
        Self {
            bSupportDCF: CVI_FALSE,
            stMPFCfg: VENC_MPF_CFG_S {
                u8LargeThumbNailNum: 0,
                astLargeThumbNailSize: [SIZE_S::default(); 2],
            },
            enReceiveMode: VENC_PIC_RECEIVE_SINGLE,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_ATTR_PRORES_S {
    pub cIdentifier: [CVI_CHAR; 4],
    pub enFrameRateCode: c_int,
    pub enAspectRatio: c_int,
}

/// `VENC_ATTR_S` 内部的匿名 union。
#[repr(C)]
#[derive(Clone, Copy)]
pub union VENC_ATTR_U {
    pub stAttrH264e: VENC_ATTR_H264_S,
    pub stAttrH265e: VENC_ATTR_H265_S,
    pub stAttrJpege: VENC_ATTR_JPEG_S,
    pub stAttrProres: VENC_ATTR_PRORES_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_ATTR_S {
    pub enType: PAYLOAD_TYPE_E,
    pub u32MaxPicWidth: CVI_U32,
    pub u32MaxPicHeight: CVI_U32,
    pub u32BufSize: CVI_U32,
    pub u32Profile: CVI_U32,
    pub bByFrame: CVI_BOOL,
    pub u32PicWidth: CVI_U32,
    pub u32PicHeight: CVI_U32,
    pub bSingleCore: CVI_BOOL,
    pub bEsBufQueueEn: CVI_BOOL,
    pub bIsoSendFrmEn: CVI_BOOL,
    /// 匿名 union（`stAttrH264e` / `stAttrJpege` / ...）。
    pub stAttr: VENC_ATTR_U,
}

/// 码率控制属性。这里只命名了 `enRcMode`，具体 RC 模式的 union 作为不透明存储：
/// JPEG 不使用码率控制（C sample 直接把它清零，质量走 `CVI_VENC_SetJpegParam`）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_RC_ATTR_S {
    pub enRcMode: c_int,
    pub payload: [CVI_U32; 7], // 28 字节的 union
}

impl Default for VENC_RC_ATTR_S {
    fn default() -> Self {
        Self {
            enRcMode: 0,
            payload: [0; 7],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_GOP_NORMALP_S {
    pub s32IPQpDelta: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_GOP_DUALP_S {
    pub u32SPInterval: CVI_U32,
    pub s32SPQpDelta: CVI_S32,
    pub s32IPQpDelta: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_GOP_SMARTP_S {
    pub u32BgInterval: CVI_U32,
    pub s32BgQpDelta: CVI_S32,
    pub s32ViQpDelta: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_GOP_BIPREDB_S {
    pub u32BFrmNum: CVI_U32,
    pub s32BQpDelta: CVI_S32,
    pub s32IPQpDelta: CVI_S32,
}

/// `VENC_GOP_ATTR_S` 内部的匿名 union（12 字节）。
#[repr(C)]
#[derive(Clone, Copy)]
pub union VENC_GOP_ATTR_U {
    pub stNormalP: VENC_GOP_NORMALP_S,
    pub stDualP: VENC_GOP_DUALP_S,
    pub stSmartP: VENC_GOP_SMARTP_S,
    pub stAdvSmartP: VENC_GOP_SMARTP_S,
    pub stBipredB: VENC_GOP_BIPREDB_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_GOP_ATTR_S {
    pub enGopMode: VENC_GOP_MODE_E,
    pub stGopAttr: VENC_GOP_ATTR_U,
}

impl Default for VENC_GOP_ATTR_S {
    fn default() -> Self {
        Self {
            enGopMode: VENC_GOPMODE_NORMALP,
            stGopAttr: VENC_GOP_ATTR_U {
                stNormalP: VENC_GOP_NORMALP_S { s32IPQpDelta: 0 },
            },
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_CHN_ATTR_S {
    pub stVencAttr: VENC_ATTR_S,
    pub stRcAttr: VENC_RC_ATTR_S,
    pub stGopAttr: VENC_GOP_ATTR_S,
}

impl Default for VENC_CHN_ATTR_S {
    fn default() -> Self {
        // SAFETY: 纯 POD 结构（无引用、无 Drop），全零位模式是合法的。
        unsafe { core::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct VENC_RECV_PIC_PARAM_S {
    /// `-1` 表示不限制帧数。
    pub s32RecvPicNum: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_JPEG_PARAM_S {
    pub u32Qfactor: CVI_U32,
    pub u8YQt: [CVI_U8; 64],
    pub u8CbQt: [CVI_U8; 64],
    pub u8CrQt: [CVI_U8; 64],
    pub u32MCUPerECS: CVI_U32,
}

impl Default for VENC_JPEG_PARAM_S {
    /// `u32Qfactor = 0` 表示沿用编码器内置的量化表。
    fn default() -> Self {
        Self {
            u32Qfactor: 0,
            u8YQt: [0; 64],
            u8CbQt: [0; 64],
            u8CrQt: [0; 64],
            u32MCUPerECS: 0,
        }
    }
}

/// `VENC_DATA_TYPE_U` union（所有成员都是 4 字节枚举）。
pub type VENC_DATA_TYPE_U = c_int;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_PACK_INFO_S {
    pub u32PackType: VENC_DATA_TYPE_U,
    pub u32PackOffset: CVI_U32,
    pub u32PackLength: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_PACK_S {
    pub u64PhyAddr: CVI_U64,
    pub pu8Addr: *mut CVI_U8,
    pub u32Len: CVI_U32,
    pub u64PTS: CVI_U64,
    pub bFrameEnd: CVI_BOOL,
    pub DataType: VENC_DATA_TYPE_U,
    pub u32Offset: CVI_U32,
    pub u32DataNum: CVI_U32,
    pub stPackInfo: [VENC_PACK_INFO_S; 8],
}

/// `VENC_STREAM_S`。只使用其中的包列表；尾部两个 stream-info union
/// （在 C 头文件里共 368 字节）作为不透明字节保留。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VENC_STREAM_S {
    pub pstPack: *mut VENC_PACK_S,
    pub u32PackCount: CVI_U32,
    pub u32Seq: CVI_U32,
    _opaque: [u8; 368],
}

/* ------------------------------------------------------------------ */
/* VDEC                                                                */
/* ------------------------------------------------------------------ */

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_ATTR_VIDEO_S {
    pub u32RefFrameNum: CVI_U32,
    pub bTemporalMvpEnable: CVI_BOOL,
    pub u32TmvBufSize: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_CHN_ATTR_S {
    pub enType: PAYLOAD_TYPE_E,
    pub enMode: VIDEO_MODE_E,
    pub u32PicWidth: CVI_U32,
    pub u32PicHeight: CVI_U32,
    pub u32StreamBufSize: CVI_U32,
    pub u32FrameBufSize: CVI_U32,
    pub u32FrameBufCnt: CVI_U32,
    pub stVdecVideoAttr: VDEC_ATTR_VIDEO_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_PARAM_PICTURE_S {
    pub u32Alpha: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_PARAM_VIDEO_S {
    pub s32ErrThreshold: CVI_S32,
    pub enDecMode: c_int,
    pub enOutputOrder: c_int,
    pub enCompressMode: COMPRESS_MODE_E,
    pub enVideoFormat: VIDEO_FORMAT_E,
}

/// `VDEC_CHN_PARAM_S`。union 以不透明存储表示；操作图片（JPEG）参数请用
/// [`VDEC_CHN_PARAM_S::picture_mut`]。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_CHN_PARAM_S {
    pub enType: PAYLOAD_TYPE_E,
    pub enPixelFormat: PIXEL_FORMAT_E,
    pub u32DisplayFrameNum: CVI_U32,
    pub payload: [CVI_U32; 5], // 20 字节的 union
}

impl VDEC_CHN_PARAM_S {
    /// 取得 union 中 `stVdecPictureParam` 成员的视图（偏移 12，4 字节）。
    pub fn picture_mut(&mut self) -> &mut VDEC_PARAM_PICTURE_S {
        // SAFETY: `payload` 为 20 字节、对齐 4；`VDEC_PARAM_PICTURE_S` 是 4 字节、
        // 对齐 4 的结构，因此该转换在边界内且对齐正确。
        unsafe { &mut *(self.payload.as_mut_ptr().cast::<VDEC_PARAM_PICTURE_S>()) }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_STREAM_S {
    pub u32Len: CVI_U32,
    pub u64PTS: CVI_U64,
    pub bEndOfFrame: CVI_BOOL,
    pub bEndOfStream: CVI_BOOL,
    pub bDisplay: CVI_BOOL,
    pub pu8Addr: *mut CVI_U8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VDEC_CHN_POOL_S {
    pub hPicVbPool: VB_POOL,
    pub hTmvVbPool: VB_POOL,
}

/* ------------------------------------------------------------------ */
/* 函数                                                                */
/* ------------------------------------------------------------------ */

unsafe extern "C" {
    // sys
    pub fn CVI_SYS_Init() -> CVI_S32;
    pub fn CVI_SYS_Exit() -> CVI_S32;
    pub fn CVI_SYS_Mmap(u64PhyAddr: CVI_U64, u32Size: CVI_U32) -> *mut CVI_VOID;
    pub fn CVI_SYS_MmapCache(u64PhyAddr: CVI_U64, u32Size: CVI_U32) -> *mut CVI_VOID;
    pub fn CVI_SYS_Munmap(pVirAddr: *mut CVI_VOID, u32Size: CVI_U32) -> CVI_S32;
    pub fn CVI_SYS_IonFlushCache(
        u64PhyAddr: CVI_U64,
        pVirAddr: *mut CVI_VOID,
        u32Len: CVI_U32,
    ) -> CVI_S32;
    pub fn CVI_SYS_IonInvalidateCache(
        u64PhyAddr: CVI_U64,
        pVirAddr: *mut CVI_VOID,
        u32Len: CVI_U32,
    ) -> CVI_S32;

    // VB
    pub fn CVI_VB_SetConfig(pstVbConfig: *const VB_CONFIG_S) -> CVI_S32;
    pub fn CVI_VB_GetConfig(pstVbConfig: *mut VB_CONFIG_S) -> CVI_S32;
    pub fn CVI_VB_Init() -> CVI_S32;
    pub fn CVI_VB_Exit() -> CVI_S32;
    pub fn CVI_VB_CreatePool(pstVbPoolCfg: *mut VB_POOL_CONFIG_S) -> VB_POOL;
    pub fn CVI_VB_DestroyPool(Pool: VB_POOL) -> CVI_S32;
    pub fn CVI_VB_GetBlock(Pool: VB_POOL, u32BlkSize: CVI_U32) -> VB_BLK;
    pub fn CVI_VB_ReleaseBlock(Block: VB_BLK) -> CVI_S32;
    pub fn CVI_VB_Handle2PoolId(Block: VB_BLK) -> VB_POOL;
    pub fn CVI_VB_Handle2PhysAddr(Block: VB_BLK) -> CVI_U64;
    pub fn CVI_VB_PhysAddr2Handle(u64PhyAddr: CVI_U64) -> VB_BLK;

    // VENC
    pub fn CVI_VENC_CreateChn(VeChn: VENC_CHN, pstAttr: *const VENC_CHN_ATTR_S) -> CVI_S32;
    pub fn CVI_VENC_DestroyChn(VeChn: VENC_CHN) -> CVI_S32;
    pub fn CVI_VENC_StartRecvFrame(
        VeChn: VENC_CHN,
        pstRecvParam: *const VENC_RECV_PIC_PARAM_S,
    ) -> CVI_S32;
    pub fn CVI_VENC_StopRecvFrame(VeChn: VENC_CHN) -> CVI_S32;
    pub fn CVI_VENC_SendFrame(
        VeChn: VENC_CHN,
        pstFrame: *const VIDEO_FRAME_INFO_S,
        s32MilliSec: CVI_S32,
    ) -> CVI_S32;
    pub fn CVI_VENC_GetStream(
        VeChn: VENC_CHN,
        pstStream: *mut VENC_STREAM_S,
        S32MilliSec: CVI_S32,
    ) -> CVI_S32;
    pub fn CVI_VENC_ReleaseStream(VeChn: VENC_CHN, pstStream: *mut VENC_STREAM_S) -> CVI_S32;
    pub fn CVI_VENC_SetJpegParam(
        VeChn: VENC_CHN,
        pstJpegParam: *const VENC_JPEG_PARAM_S,
    ) -> CVI_S32;
    pub fn CVI_VENC_GetJpegParam(VeChn: VENC_CHN, pstJpegParam: *mut VENC_JPEG_PARAM_S) -> CVI_S32;

    // VDEC
    pub fn CVI_VDEC_CreateChn(VdChn: VDEC_CHN, pstAttr: *const VDEC_CHN_ATTR_S) -> CVI_S32;
    pub fn CVI_VDEC_DestroyChn(VdChn: VDEC_CHN) -> CVI_S32;
    pub fn CVI_VDEC_StartRecvStream(VdChn: VDEC_CHN) -> CVI_S32;
    pub fn CVI_VDEC_StopRecvStream(VdChn: VDEC_CHN) -> CVI_S32;
    pub fn CVI_VDEC_GetChnParam(VdChn: VDEC_CHN, pstParam: *mut VDEC_CHN_PARAM_S) -> CVI_S32;
    pub fn CVI_VDEC_SetChnParam(VdChn: VDEC_CHN, pstParam: *const VDEC_CHN_PARAM_S) -> CVI_S32;
    pub fn CVI_VDEC_SendStream(
        VdChn: VDEC_CHN,
        pstStream: *const VDEC_STREAM_S,
        s32MilliSec: CVI_S32,
    ) -> CVI_S32;
    pub fn CVI_VDEC_GetFrame(
        VdChn: VDEC_CHN,
        pstFrameInfo: *mut VIDEO_FRAME_INFO_S,
        s32MilliSec: CVI_S32,
    ) -> CVI_S32;
    pub fn CVI_VDEC_ReleaseFrame(
        VdChn: VDEC_CHN,
        pstFrameInfo: *const VIDEO_FRAME_INFO_S,
    ) -> CVI_S32;
    pub fn CVI_VDEC_AttachVbPool(VdChn: VDEC_CHN, pstPool: *const VDEC_CHN_POOL_S) -> CVI_S32;
    pub fn CVI_VDEC_DetachVbPool(VdChn: VDEC_CHN) -> CVI_S32;
}
