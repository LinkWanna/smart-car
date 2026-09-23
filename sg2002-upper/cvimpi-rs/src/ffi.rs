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
pub type CVI_FLOAT = f32;
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
pub const PIXEL_FORMAT_RGB_888: PIXEL_FORMAT_E = 0;
pub const PIXEL_FORMAT_BGR_888: PIXEL_FORMAT_E = 1;
pub const PIXEL_FORMAT_RGB_888_PLANAR: PIXEL_FORMAT_E = 2;
pub const PIXEL_FORMAT_BGR_888_PLANAR: PIXEL_FORMAT_E = 3;
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
/* VPSS                                                                */
/* ------------------------------------------------------------------ */

/// `VPSS_GRP` / `VPSS_CHN`（linux/cvi_common.h）。
pub type VPSS_GRP = CVI_S32;
pub type VPSS_CHN = CVI_S32;

/// `VPSS_MAX_PHY_CHN_NUM`（CV181X = 4；CV180X = 3）。
pub const VPSS_MAX_PHY_CHN_NUM: usize = 4;
/// `VPSS_MAX_GRP_NUM`：最多 16 个组。
pub const VPSS_MAX_GRP_NUM: usize = 16;

/// `VPSS_MODE_E`（linux/cvi_comm_sys.h）。
pub type VPSS_MODE_E = c_int;
pub const VPSS_MODE_SINGLE: VPSS_MODE_E = 0;
pub const VPSS_MODE_DUAL: VPSS_MODE_E = 1;

/// `VI_VPSS_MODE_E`（linux/cvi_comm_sys.h）。
pub type VI_VPSS_MODE_E = c_int;
pub const VI_OFFLINE_VPSS_OFFLINE: VI_VPSS_MODE_E = 0;
pub const VI_OFFLINE_VPSS_ONLINE: VI_VPSS_MODE_E = 1;
pub const VI_ONLINE_VPSS_OFFLINE: VI_VPSS_MODE_E = 2;
pub const VI_ONLINE_VPSS_ONLINE: VI_VPSS_MODE_E = 3;

/// `VI_MAX_PIPE_NUM`（CV181X = 4 物理 + 2 虚拟）。
pub const VI_MAX_PIPE_NUM: usize = 6;
/// `VPSS_IP_NUM`：VPSS 有两个设备（dev0/dev1）。
pub const VPSS_IP_NUM: usize = 2;

/// `VI_VPSS_MODE_S`：每个 VI pipe 的 vi/vpss 在线-离线组合。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VI_VPSS_MODE_S {
    pub aenMode: [VI_VPSS_MODE_E; VI_MAX_PIPE_NUM],
}

/// `VPSS_INPUT_E`：VPSS 每个设备的输入来源。
pub type VPSS_INPUT_E = c_int;
pub const VPSS_INPUT_MEM: VPSS_INPUT_E = 0;
pub const VPSS_INPUT_ISP: VPSS_INPUT_E = 1;

/// `VPSS_MODE_S`：VPSS 工作模式 + 各设备输入来源（`enMode == SINGLE` 时用 dev0）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VPSS_MODE_S {
    pub enMode: VPSS_MODE_E,
    pub aenInput: [VPSS_INPUT_E; VPSS_IP_NUM],
    pub ViPipe: [c_int; VPSS_IP_NUM],
}

/// `FRAME_RATE_CTRL_S`。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FRAME_RATE_CTRL_S {
    pub s32SrcFrameRate: CVI_S32,
    pub s32DstFrameRate: CVI_S32,
}

/// `RECT_S`。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RECT_S {
    pub s32X: CVI_S32,
    pub s32Y: CVI_S32,
    pub u32Width: CVI_U32,
    pub u32Height: CVI_U32,
}

/// `ASPECT_RATIO_S`。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ASPECT_RATIO_S {
    pub enMode: c_int,
    pub bEnableBgColor: CVI_BOOL,
    pub u32BgColor: CVI_U32,
    pub stVideoRect: RECT_S,
}

/// `VPSS_NORMALIZE_S`（通道输出归一化：`out = (in - mean) * factor`）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VPSS_NORMALIZE_S {
    pub bEnable: CVI_BOOL,
    pub factor: [CVI_FLOAT; 3],
    pub mean: [CVI_FLOAT; 3],
    pub rounding: c_int,
}

/// `VPSS_GRP_ATTR_S`：组的**输入**尺寸与像素格式（`SendFrame` 送进来的帧）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VPSS_GRP_ATTR_S {
    pub u32MaxW: CVI_U32,
    pub u32MaxH: CVI_U32,
    pub enPixelFormat: PIXEL_FORMAT_E,
    pub stFrameRate: FRAME_RATE_CTRL_S,
    /// 仅在 `VPSS_MODE_DUAL` 下有意义。
    pub u8VpssDev: CVI_U8,
}

/// `VPSS_CHN_ATTR_S`：通道的**输出**尺寸与像素格式。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VPSS_CHN_ATTR_S {
    pub u32Width: CVI_U32,
    pub u32Height: CVI_U32,
    pub enVideoFormat: VIDEO_FORMAT_E,
    pub enPixelFormat: PIXEL_FORMAT_E,
    pub stFrameRate: FRAME_RATE_CTRL_S,
    pub bMirror: CVI_BOOL,
    pub bFlip: CVI_BOOL,
    /// 用户态 `GetChnFrame` 队列深度（0 = 不交给用户，直接 bind 给下游）。
    pub u32Depth: CVI_U32,
    pub stAspectRatio: ASPECT_RATIO_S,
    pub stNormalize: VPSS_NORMALIZE_S,
}

/* ioctl 传输结构（linux/vpss_uapi.h） */

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_crt_grp_cfg {
    pub VpssGrp: VPSS_GRP,
    pub stGrpAttr: VPSS_GRP_ATTR_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_str_grp_cfg {
    pub VpssGrp: VPSS_GRP,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_grp_attr {
    pub VpssGrp: VPSS_GRP,
    pub stGrpAttr: VPSS_GRP_ATTR_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_chn_attr {
    pub VpssGrp: VPSS_GRP,
    pub VpssChn: VPSS_CHN,
    pub stChnAttr: VPSS_CHN_ATTR_S,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_en_chn_cfg {
    pub VpssGrp: VPSS_GRP,
    pub VpssChn: VPSS_CHN,
}

/// `SEND_FRAME` 的载荷。注意 `VpssGrp` 在头文件里是 `__u8`（不是 `VPSS_GRP`）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_snd_frm_cfg {
    pub VpssGrp: CVI_U8,
    pub stVideoFrame: VIDEO_FRAME_INFO_S,
    pub s32MilliSec: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_chn_frm_cfg {
    pub VpssGrp: VPSS_GRP,
    pub VpssChn: VPSS_CHN,
    pub stVideoFrame: VIDEO_FRAME_INFO_S,
    pub s32MilliSec: CVI_S32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_chn_align_cfg {
    pub VpssGrp: VPSS_GRP,
    pub VpssChn: VPSS_CHN,
    pub u32Align: CVI_U32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_chn_rot_cfg {
    pub VpssGrp: VPSS_GRP,
    pub VpssChn: VPSS_CHN,
    pub enRotation: c_int,
}

/// `PROC_AMP_E` 的下标（`vpss_grp_csc_cfg::proc_amp`）。
pub const PROC_AMP_MAX: usize = 4;
pub const PROC_AMP_BRIGHTNESS: usize = 0;
pub const PROC_AMP_CONTRAST: usize = 1;
pub const PROC_AMP_SATURATION: usize = 2;
pub const PROC_AMP_HUE: usize = 3;

/// `struct vpss_grp_csc_cfg`（内部接口，`/proc` 之外的 CSC 配置入口）：
/// `out_i = Σ_j coef[i][j] * (in_j - sub[j]) + add[i]`，
/// `in = Y/U/V`，`out = R/G/B`，`coef` 是 10 位小数的 13 位有符号定点
/// （负数编码为 `|v| * 1024 | BIT(13)`）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct vpss_grp_csc_cfg {
    pub VpssGrp: VPSS_GRP,
    pub proc_amp: [CVI_S32; PROC_AMP_MAX],
    pub coef: [[CVI_U16; 3]; 3],
    pub sub: [CVI_U8; 3],
    pub add: [CVI_U8; 3],
    pub scene: CVI_U8,
}

/* ioctl 号：`_IOW/_IOWR` 编码（asm-generic/ioctl.h）。
 * 结构体尺寸参与编码，所以这里用 `size_of` 现算：布局一旦写错，ioctl 号立刻变，
 * 上板就会失败（等价于编译期检查，见 `vpss::tests` 里对拍 C 头的常量）。 */
const IOC_NRBITS: u32 = 8;
const IOC_TYPEBITS: u32 = 8;
const IOC_SIZEBITS: u32 = 14;
const IOC_TYPESHIFT: u32 = IOC_NRBITS;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + IOC_TYPEBITS;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + IOC_SIZEBITS;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

#[allow(non_snake_case)]
const fn _IOC(dir: u32, ty: u8, nr: u8, size: usize) -> u64 {
    ((dir as u64) << IOC_DIRSHIFT)
        | ((ty as u64) << IOC_TYPESHIFT)
        | ((nr as u64) << 0)
        | ((size as u64) << IOC_SIZESHIFT)
}

#[allow(non_snake_case)]
const fn _IOW<T>(ty: u8, nr: u8) -> u64 {
    _IOC(IOC_WRITE, ty, nr, core::mem::size_of::<T>())
}

#[allow(non_snake_case)]
const fn _IOWR<T>(ty: u8, nr: u8) -> u64 {
    _IOC(IOC_READ | IOC_WRITE, ty, nr, core::mem::size_of::<T>())
}

/// VPSS ioctl 的 type 字符（`'S'`）。
pub const VPSS_IOC_TYPE: u8 = b'S';

pub const CVI_VPSS_CREATE_GROUP: u64 = _IOW::<vpss_crt_grp_cfg>(VPSS_IOC_TYPE, 0x00);
pub const CVI_VPSS_DESTROY_GROUP: u64 = _IOW::<VPSS_GRP>(VPSS_IOC_TYPE, 0x01);
pub const CVI_VPSS_START_GROUP: u64 = _IOW::<vpss_str_grp_cfg>(VPSS_IOC_TYPE, 0x03);
pub const CVI_VPSS_STOP_GROUP: u64 = _IOW::<VPSS_GRP>(VPSS_IOC_TYPE, 0x04);
pub const CVI_VPSS_RESET_GROUP: u64 = _IOW::<VPSS_GRP>(VPSS_IOC_TYPE, 0x05);
pub const CVI_VPSS_SET_GRP_ATTR: u64 = _IOW::<vpss_grp_attr>(VPSS_IOC_TYPE, 0x06);
pub const CVI_VPSS_SEND_FRAME: u64 = _IOW::<vpss_snd_frm_cfg>(VPSS_IOC_TYPE, 0x0c);
pub const CVI_VPSS_SET_CHN_ATTR: u64 = _IOW::<vpss_chn_attr>(VPSS_IOC_TYPE, 0x21);
pub const CVI_VPSS_GET_CHN_ATTR: u64 = _IOWR::<vpss_chn_attr>(VPSS_IOC_TYPE, 0x22);
pub const CVI_VPSS_ENABLE_CHN: u64 = _IOW::<vpss_en_chn_cfg>(VPSS_IOC_TYPE, 0x23);
pub const CVI_VPSS_DISABLE_CHN: u64 = _IOW::<vpss_en_chn_cfg>(VPSS_IOC_TYPE, 0x24);
pub const CVI_VPSS_GET_CHN_FRAME: u64 = _IOWR::<vpss_chn_frm_cfg>(VPSS_IOC_TYPE, 0x2b);
pub const CVI_VPSS_RELEASE_CHN_FRAME: u64 = _IOWR::<vpss_chn_frm_cfg>(VPSS_IOC_TYPE, 0x2c);
pub const CVI_VPSS_SET_CHN_ALIGN: u64 = _IOW::<vpss_chn_align_cfg>(VPSS_IOC_TYPE, 0x2d);
pub const CVI_VPSS_SET_CHN_ROTATION: u64 = _IOW::<vpss_chn_rot_cfg>(VPSS_IOC_TYPE, 0x27);
/// 内部接口：直接写组的 CSC 矩阵（`CVI_VPSS_SetGrpProcAmp` 底层就是它）。
pub const CVI_VPSS_SET_GRP_CSC_CFG: u64 = _IOW::<vpss_grp_csc_cfg>(VPSS_IOC_TYPE, 0x78);

/// ABI 编译期校验：数值全部来自 `tools/vpss_abi_probe.c`
/// （`gcc -I <cvi_mpi>/include -D__CV181X__ tools/vpss_abi_probe.c`）在 C 头上的实测输出。
/// 布局或 ioctl 编码一旦写错，`cargo check` 直接失败（32 位 ARM 未覆盖，同本 crate 其它结构）。
const _: () = {
    use core::mem::{offset_of, size_of};

    assert!(size_of::<FRAME_RATE_CTRL_S>() == 8);
    assert!(size_of::<RECT_S>() == 16);
    assert!(size_of::<ASPECT_RATIO_S>() == 28 && offset_of!(ASPECT_RATIO_S, stVideoRect) == 12);
    assert!(
        size_of::<VPSS_NORMALIZE_S>() == 32
            && offset_of!(VPSS_NORMALIZE_S, factor) == 4
            && offset_of!(VPSS_NORMALIZE_S, mean) == 16
            && offset_of!(VPSS_NORMALIZE_S, rounding) == 28
    );
    assert!(
        size_of::<VPSS_GRP_ATTR_S>() == 24 && offset_of!(VPSS_GRP_ATTR_S, u8VpssDev) == 20
    );
    assert!(
        size_of::<VPSS_CHN_ATTR_S>() == 92
            && offset_of!(VPSS_CHN_ATTR_S, u32Depth) == 28
            && offset_of!(VPSS_CHN_ATTR_S, stAspectRatio) == 32
            && offset_of!(VPSS_CHN_ATTR_S, stNormalize) == 60
    );

    assert!(size_of::<vpss_crt_grp_cfg>() == 28);
    assert!(size_of::<vpss_str_grp_cfg>() == 4);
    assert!(size_of::<vpss_grp_attr>() == 28);
    assert!(size_of::<vpss_en_chn_cfg>() == 8);
    assert!(size_of::<vpss_chn_attr>() == 100);
    assert!(
        size_of::<vpss_snd_frm_cfg>() == 168
            && offset_of!(vpss_snd_frm_cfg, stVideoFrame) == 8
            && offset_of!(vpss_snd_frm_cfg, s32MilliSec) == 160
    );
    assert!(
        size_of::<vpss_chn_frm_cfg>() == 168
            && offset_of!(vpss_chn_frm_cfg, stVideoFrame) == 8
            && offset_of!(vpss_chn_frm_cfg, s32MilliSec) == 160
    );
    assert!(size_of::<vpss_chn_align_cfg>() == 12);
    assert!(size_of::<vpss_chn_rot_cfg>() == 12);
    assert!(
        size_of::<MMF_CHN_S>() == 12
            && offset_of!(MMF_CHN_S, enModId) == 0
            && offset_of!(MMF_CHN_S, s32DevId) == 4
            && offset_of!(MMF_CHN_S, s32ChnId) == 8
    );
    assert!(
        size_of::<vpss_grp_csc_cfg>() == 48
            && offset_of!(vpss_grp_csc_cfg, proc_amp) == 4
            && offset_of!(vpss_grp_csc_cfg, coef) == 20
            && offset_of!(vpss_grp_csc_cfg, sub) == 38
            && offset_of!(vpss_grp_csc_cfg, add) == 41
            && offset_of!(vpss_grp_csc_cfg, scene) == 44
    );
    assert!(size_of::<VIDEO_FRAME_INFO_S>() == 152);

    assert!(CVI_VPSS_CREATE_GROUP == 0x401c_5300);
    assert!(CVI_VPSS_DESTROY_GROUP == 0x4004_5301);
    assert!(CVI_VPSS_START_GROUP == 0x4004_5303);
    assert!(CVI_VPSS_STOP_GROUP == 0x4004_5304);
    assert!(CVI_VPSS_RESET_GROUP == 0x4004_5305);
    assert!(CVI_VPSS_SET_GRP_ATTR == 0x401c_5306);
    assert!(CVI_VPSS_SEND_FRAME == 0x40a8_530c);
    assert!(CVI_VPSS_SET_CHN_ATTR == 0x4064_5321);
    assert!(CVI_VPSS_GET_CHN_ATTR == 0xc064_5322);
    assert!(CVI_VPSS_ENABLE_CHN == 0x4008_5323);
    assert!(CVI_VPSS_DISABLE_CHN == 0x4008_5324);
    assert!(CVI_VPSS_SET_CHN_ROTATION == 0x400c_5327);
    assert!(CVI_VPSS_GET_CHN_FRAME == 0xc0a8_532b);
    assert!(CVI_VPSS_RELEASE_CHN_FRAME == 0xc0a8_532c);
    assert!(CVI_VPSS_SET_CHN_ALIGN == 0x400c_532d);
    assert!(CVI_VPSS_SET_GRP_CSC_CFG == 0x4030_5378);
};

/* ------------------------------------------------------------------ */
/* 通道绑定（CVI_SYS_Bind）                                            */
/* ------------------------------------------------------------------ */

/// `MOD_ID_E` 里用到的几个（linux/cvi_common.h）。
pub type MOD_ID_E = c_int;
pub const CVI_ID_SYS: MOD_ID_E = 2;
pub const CVI_ID_VDEC: MOD_ID_E = 5;
pub const CVI_ID_VPSS: MOD_ID_E = 6;
pub const CVI_ID_VENC: MOD_ID_E = 7;

/// `MMF_CHN_S`：一个模块的通道标识（bind 的源/目的）。
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MMF_CHN_S {
    pub enModId: MOD_ID_E,
    pub s32DevId: CVI_S32,
    pub s32ChnId: CVI_S32,
}

/// 构造 [`MMF_CHN_S`] 的 const 辅助函数。
pub const fn mmf_chn(mod_id: MOD_ID_E, dev: CVI_S32, chn: CVI_S32) -> MMF_CHN_S {
    MMF_CHN_S {
        enModId: mod_id,
        s32DevId: dev,
        s32ChnId: chn,
    }
}

/* ------------------------------------------------------------------ */
/* 函数                                                                */
/* ------------------------------------------------------------------ */

unsafe extern "C" {
    // sys
    pub fn CVI_SYS_Init() -> CVI_S32;
    pub fn CVI_SYS_Exit() -> CVI_S32;
    pub fn CVI_SYS_SetVPSSMode(enVPSSMode: VPSS_MODE_E) -> CVI_S32;
    pub fn CVI_SYS_GetVPSSMode() -> VPSS_MODE_E;
    pub fn CVI_SYS_SetVPSSModeEx(pstVPSSMode: *const VPSS_MODE_S) -> CVI_S32;
    pub fn CVI_SYS_GetVPSSModeEx(pstVPSSMode: *mut VPSS_MODE_S) -> CVI_S32;
    pub fn CVI_SYS_SetVIVPSSMode(pstVIVPSSMode: *const VI_VPSS_MODE_S) -> CVI_S32;
    pub fn CVI_SYS_GetVIVPSSMode(pstVIVPSSMode: *mut VI_VPSS_MODE_S) -> CVI_S32;
    pub fn CVI_SYS_Bind(pstSrcChn: *const MMF_CHN_S, pstDestChn: *const MMF_CHN_S) -> CVI_S32;
    pub fn CVI_SYS_UnBind(pstSrcChn: *const MMF_CHN_S, pstDestChn: *const MMF_CHN_S) -> CVI_S32;
    pub fn CVI_SYS_GetBindbyDest(
        pstDestChn: *const MMF_CHN_S,
        pstSrcChn: *mut MMF_CHN_S,
    ) -> CVI_S32;
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
