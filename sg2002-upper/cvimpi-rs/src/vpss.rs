//! VPSS（视频后处理）：硬件色彩空间转换 + 缩放，一路输入多路输出。
//!
//! 典型用法（[`crate::sys::Sys`] 持有会话与 VB 池）：
//!
//! ```no_run
//! use cvimpi_rs::{ffi, sys::{Sys, VbPoolConfig}, vpss::{VpssChnConfig, VpssConfig}};
//!
//! # fn main() -> Result<(), cvimpi_rs::Error> {
//! // 池 block 要放得下最大的一帧：YUYV 输入 614400、RGB 平面输出 921600、NV12 输出 460800
//! let sys = Sys::init(&[VbPoolConfig::new(640 * 480 * 3, 6)])?;
//! let vpss = sys.create_vpss(&VpssConfig {
//!     grp: 0,
//!     max_w: 640,
//!     max_h: 480,
//!     in_format: ffi::PIXEL_FORMAT_YUYV,
//!     chns: vec![
//!         VpssChnConfig::new(0, 640, 480, ffi::PIXEL_FORMAT_RGB_888_PLANAR),
//!         VpssChnConfig::new(1, 640, 480, ffi::PIXEL_FORMAT_NV12),
//!     ],
//! })?;
//!
//! let mut frame = sys.alloc_frame_cached(640, 480, ffi::PIXEL_FORMAT_YUYV)?;
//! // frame.write_tight(&yuyv)?;  // 写完由 send_frame 前手动 flush
//! frame.flush()?;
//! vpss.send_frame(frame.info(), ffi::CVI_IO_BLOCK)?;
//! let rgb = vpss.get_chn_frame(0, 1000)?;
//! let _ = rgb.phy_addr(0);   // 可以交给 TPU 零拷贝
//! # Ok(())
//! # }
//! ```
//!
//! # 为什么不用 libvpss.so
//!
//! 板端镜像里没有 `libvpss.so`，但 `/dev/cvi-vpss` + `soph_vpss.ko` 都在，
//! 所以这里直接按 `../include/linux/vpss_uapi.h` 的 ioctl 协议实现，
//! ABI 数值由 `tools/vpss_abi_probe.c` 在 C 头上实测（见 `tests::abi`），
//! 不引入任何新的运行期依赖。
//!
//! # 与 C sample 的对应关系
//!
//! [`Sys::create_vpss`] 的顺序 = `SAMPLE_COMM_VPSS_*`：
//! `CVI_VPSS_CreateGrp` → `ResetGrp` → `SetChnAttr` + `EnableChn`(逐通道) →
//! `StartGrp`；析构时 `DisableChn` → `StopGrp` → `DestroyGrp`。
//! 送帧/取帧 = `CVI_VPSS_SendFrame` / `GetChnFrame` / `ReleaseChnFrame`。

use core::ffi::c_void;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicI32, Ordering};
use core::{mem, slice};
use std::os::unix::io::RawFd;

use crate::ffi;
use crate::sys::Sys;
use crate::{Error, Result, SessionRef, check, plane_dims, tight_frame_size};

/// VPSS 设备节点（`SYS_DEV_NAME` 之外的 `VPSS_DEV_NAME`）。
const VPSS_DEV: &str = "/dev/cvi-vpss";

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */

/// 一个输出通道的配置。
#[derive(Debug, Clone, Copy)]
pub struct VpssChnConfig {
    pub chn: ffi::VPSS_CHN,
    pub width: u32,
    pub height: u32,
    /// 输出像素格式（`RGB_888_PLANAR` / `NV12` / …）。
    pub format: ffi::PIXEL_FORMAT_E,
    /// 用户态 `GetChnFrame` 队列深度：1 = 只保留最新一帧，0 = 全部直接给 bind 的下游。
    pub depth: u32,
    pub mirror: bool,
    pub flip: bool,
}

impl VpssChnConfig {
    /// `depth = 1`（只保留最新一帧）、不镜像不翻转。
    pub fn new(chn: ffi::VPSS_CHN, width: u32, height: u32, format: ffi::PIXEL_FORMAT_E) -> Self {
        Self {
            chn,
            width,
            height,
            format,
            depth: 1,
            mirror: false,
            flip: false,
        }
    }

    pub fn with_depth(mut self, depth: u32) -> Self {
        self.depth = depth;
        self
    }

    fn to_attr(self) -> ffi::VPSS_CHN_ATTR_S {
        ffi::VPSS_CHN_ATTR_S {
            u32Width: self.width,
            u32Height: self.height,
            enVideoFormat: ffi::VIDEO_FORMAT_LINEAR,
            enPixelFormat: self.format,
            // `-1/-1` = 不做帧率控制（与 C sample 一致）。
            stFrameRate: ffi::FRAME_RATE_CTRL_S {
                s32SrcFrameRate: -1,
                s32DstFrameRate: -1,
            },
            bMirror: ffi::CVI_BOOL::from(self.mirror),
            bFlip: ffi::CVI_BOOL::from(self.flip),
            u32Depth: self.depth,
            stAspectRatio: ffi::ASPECT_RATIO_S {
                enMode: 0, // ASPECT_RATIO_NONE：不保持宽高比，直接缩放到输出尺寸
                bEnableBgColor: ffi::CVI_FALSE,
                u32BgColor: 0,
                stVideoRect: ffi::RECT_S::default(),
            },
            stNormalize: ffi::VPSS_NORMALIZE_S::default(),
        }
    }
}

/// 一个 VPSS 组的配置（`CreateGrp` + `SetChnAttr`/`EnableChn` 的集合）。
#[derive(Debug, Clone)]
pub struct VpssConfig {
    /// 组号；[`VPSS_GRP_AUTO`] = 自动挑一个还能建的（推荐，见 [`VpssConfig`] 说明）。
    pub grp: ffi::VPSS_GRP,
    /// 输入帧最大尺寸（组属性里的 `u32MaxW/MaxH`）。
    pub max_w: u32,
    pub max_h: u32,
    /// **输入**像素格式：`SendFrame` 送进来的帧必须是这个格式（如 `YUYV`）。
    pub in_format: ffi::PIXEL_FORMAT_E,
    /// 输出通道，数量不能超过 [`ffi::VPSS_MAX_PHY_CHN_NUM`]。
    pub chns: Vec<VpssChnConfig>,
}

/// [`VpssConfig::grp`] 的自动值。
///
/// 实测（SG2002 / 这个 image）：**同一个组号在一个 boot 里只能成功建组一次**，
/// `DestroyGrp` / `CVI_SYS_Exit` 都不会复位它，第二次用同一个组号 `CreateGrp`
/// 稳定返回 `-ENOSYS`；换成没用过的组号立刻正常。所以这里默认从
/// 一个进程级计数器往后取组号，16 个用完了才失败（重启板子恢复）。
pub const VPSS_GRP_AUTO: ffi::VPSS_GRP = -1;

/// 下一个候选组号（`VPSS_GRP_AUTO` 用）。
static NEXT_GRP: AtomicI32 = AtomicI32::new(0);

fn alloc_grp_id() -> ffi::VPSS_GRP {
    let n = NEXT_GRP.fetch_add(1, Ordering::Relaxed);
    n.rem_euclid(ffi::VPSS_MAX_GRP_NUM as i32)
}

/* ------------------------------------------------------------------ */
/* 组 + 通道                                                           */
/* ------------------------------------------------------------------ */

/// 一个已启动的 VPSS 组（`/dev/cvi-vpss` 上的一个 fd）。
///
/// 由 [`Sys::create_vpss`] 创建，借用会话；析构时关闭通道、停止并销毁组。
/// 与其它中间件句柄一样是 `Send`：但同一组上挂载的通道建议**每个线程各开一个
/// [`Vpss`] 句柄**（`/dev/cvi-vpss` 支持多 fd，组状态在内核里），不要跨线程共享
/// 一个 `&Vpss` 做阻塞调用。
pub struct Vpss<'a> {
    fd: RawFd,
    grp: ffi::VPSS_GRP,
    /// 已使能的通道（析构时按序关闭；创建过程中失败时只回退已完成的部分）。
    chns: Vec<ffi::VPSS_CHN>,
    started: bool,
    _session: SessionRef<'a>,
}

impl<'a> Vpss<'a> {
    /// 建组、配通道、使能、启动。
    pub(crate) fn create(_sys: &'a Sys, cfg: &VpssConfig) -> Result<Vpss<'a>> {
        if cfg.chns.len() > ffi::VPSS_MAX_PHY_CHN_NUM {
            return Err(Error::invalid("VpssConfig (too many channels)"));
        }
        let fd = open_device(VPSS_DEV)?;
        let mut vpss = Vpss {
            fd,
            grp: 0,
            chns: Vec::with_capacity(cfg.chns.len()),
            started: false,
            _session: PhantomData,
        };

        set_vpss_modes()?;

        let grp_attr = ffi::VPSS_GRP_ATTR_S {
            u32MaxW: cfg.max_w,
            u32MaxH: cfg.max_h,
            enPixelFormat: cfg.in_format,
            stFrameRate: ffi::FRAME_RATE_CTRL_S {
                s32SrcFrameRate: -1,
                s32DstFrameRate: -1,
            },
            u8VpssDev: 0,
        };

        // 组号：显式给了就用它；[`VPSS_GRP_AUTO`] 则从进程级计数器往后找
        // （驱动实测：一个组号一个 boot 只能建一次，见 [`VPSS_GRP_AUTO`]）。
        let grp = if cfg.grp == VPSS_GRP_AUTO {
            let mut last = Error::invalid("CVI_VPSS_CreateGrp (no free group id)");
            let mut found = None;
            for _ in 0..ffi::VPSS_MAX_GRP_NUM {
                let id = alloc_grp_id();
                let crt = ffi::vpss_crt_grp_cfg {
                    VpssGrp: id,
                    stGrpAttr: grp_attr,
                };
                match ioctl_write(fd, ffi::CVI_VPSS_CREATE_GROUP, &crt, "CVI_VPSS_CreateGrp") {
                    Ok(()) => {
                        found = Some(id);
                        break;
                    }
                    Err(e) => last = e,
                }
            }
            match found {
                Some(id) => id,
                None => return Err(last),
            }
        } else {
            let crt = ffi::vpss_crt_grp_cfg {
                VpssGrp: cfg.grp,
                stGrpAttr: grp_attr,
            };
            ioctl_write(fd, ffi::CVI_VPSS_CREATE_GROUP, &crt, "CVI_VPSS_CreateGrp")?;
            cfg.grp
        };
        vpss.grp = grp;

        ioctl_write(fd, ffi::CVI_VPSS_RESET_GROUP, &grp, "CVI_VPSS_ResetGrp")?;

        for chn in &cfg.chns {
            let attr = ffi::vpss_chn_attr {
                VpssGrp: grp,
                VpssChn: chn.chn,
                stChnAttr: chn.to_attr(),
            };
            ioctl_write(fd, ffi::CVI_VPSS_SET_CHN_ATTR, &attr, "CVI_VPSS_SetChnAttr")?;
            let en = ffi::vpss_en_chn_cfg {
                VpssGrp: grp,
                VpssChn: chn.chn,
            };
            ioctl_write(fd, ffi::CVI_VPSS_ENABLE_CHN, &en, "CVI_VPSS_EnableChn")?;
            vpss.chns.push(chn.chn);
        }

        let start = ffi::vpss_str_grp_cfg { VpssGrp: grp };
        ioctl_write(fd, ffi::CVI_VPSS_START_GROUP, &start, "CVI_VPSS_StartGrp")?;
        vpss.started = true;
        Ok(vpss)
    }

    pub fn group(&self) -> ffi::VPSS_GRP {
        self.grp
    }

    /// 送一帧进组（`CVI_VPSS_SendFrame`）。
    ///
    /// `frame` 的 `enPixelFormat` 必须等于建组时的 [`VpssConfig::in_format`]；
    /// 帧内存必须是 VB block（见 `sys::alloc_frame*`），CPU 写过的话先 `flush`。
    pub fn send_frame(
        &self,
        frame: &ffi::VIDEO_FRAME_INFO_S,
        timeout_ms: ffi::CVI_S32,
    ) -> Result<()> {
        let grp = u8::try_from(self.grp).map_err(|_| Error::invalid("VPSS grp out of range"))?;
        let cfg = ffi::vpss_snd_frm_cfg {
            VpssGrp: grp,
            stVideoFrame: *frame,
            s32MilliSec: timeout_ms,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VPSS_SEND_FRAME,
            &cfg,
            "CVI_VPSS_SendFrame",
        )
    }

    /// 取一帧通道输出（`CVI_VPSS_GetChnFrame`）；返回的 [`VpssFrame`]
    /// 析构时自动 `ReleaseChnFrame`。
    pub fn get_chn_frame<'s>(
        &'s self,
        chn: ffi::VPSS_CHN,
        timeout_ms: ffi::CVI_S32,
    ) -> Result<VpssFrame<'s>> {
        let mut cfg = ffi::vpss_chn_frm_cfg {
            VpssGrp: self.grp,
            VpssChn: chn,
            // SAFETY: 纯 POD 结构。
            stVideoFrame: unsafe { mem::zeroed() },
            s32MilliSec: timeout_ms,
        };
        ioctl_readwrite(
            self.fd,
            ffi::CVI_VPSS_GET_CHN_FRAME,
            &mut cfg,
            "CVI_VPSS_GetChnFrame",
        )?;
        Ok(VpssFrame {
            fd: self.fd,
            grp: self.grp,
            chn,
            info: cfg.stVideoFrame,
            maps: [None; 3],
            _vpss: PhantomData,
        })
    }

    /// 设置通道输出 buffer 的 stride 对齐（`CVI_VPSS_SetChnAlign`）。
    ///
    /// 默认对齐由驱动决定；TPU 零拷贝要求 RGB 平面输出 stride 恰好等于宽度
    /// （640 在 16/32/64 对齐下都成立），必要时用它压小对齐。
    pub fn set_chn_align(&self, chn: ffi::VPSS_CHN, align: u32) -> Result<()> {
        let cfg = ffi::vpss_chn_align_cfg {
            VpssGrp: self.grp,
            VpssChn: chn,
            u32Align: align,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VPSS_SET_CHN_ALIGN,
            &cfg,
            "CVI_VPSS_SetChnAlign",
        )
    }

    /// 设置通道旋转（`CVI_VPSS_SetChnRotation`，`0/90/180/270`）。
    pub fn set_chn_rotation(&self, chn: ffi::VPSS_CHN, rotation: i32) -> Result<()> {
        let cfg = ffi::vpss_chn_rot_cfg {
            VpssGrp: self.grp,
            VpssChn: chn,
            enRotation: rotation,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VPSS_SET_CHN_ROTATION,
            &cfg,
            "CVI_VPSS_SetChnRotation",
        )
    }

    /// 把本组的一个通道绑到 VENC 通道（内核内交接）：
    /// VPSS 的输出帧直接进编码器，用户态不再需要 `GetChnFrame`/`SendFrame`。
    ///
    /// 对应 SDK sample 的 `SAMPLE_COMM_VPSS_Bind_VENC`（VENC 的 `DevId` 固定为 0）。
    /// 绑定后该通道的 `u32Depth` 建议设 0（帧不落用户态队列）；
    /// 不管 `depth` 多少，**每一帧都要把码流取走**（`GetStream`），否则
    /// VENC 的码流队列会顶住整条链路。
    pub fn bind_chn_to_venc(&self, chn: ffi::VPSS_CHN, venc_chn: ffi::VENC_CHN) -> Result<()> {
        let src = ffi::mmf_chn(ffi::CVI_ID_VPSS, self.grp, chn);
        let dst = ffi::mmf_chn(ffi::CVI_ID_VENC, 0, venc_chn);
        check(
            unsafe { ffi::CVI_SYS_Bind(&src, &dst) },
            "CVI_SYS_Bind(VPSS→VENC)",
        )
    }

    /// 解除 [`Vpss::bind_chn_to_venc`] 建立的绑定（退出前应当调用）。
    pub fn unbind_chn_from_venc(&self, chn: ffi::VPSS_CHN, venc_chn: ffi::VENC_CHN) -> Result<()> {
        let src = ffi::mmf_chn(ffi::CVI_ID_VPSS, self.grp, chn);
        let dst = ffi::mmf_chn(ffi::CVI_ID_VENC, 0, venc_chn);
        check(
            unsafe { ffi::CVI_SYS_UnBind(&src, &dst) },
            "CVI_SYS_UnBind(VPSS→VENC)",
        )
    }

    /// 直接写组的 CSC 矩阵（内部 ioctl `CVI_VPSS_SET_GRP_CSC_CFG`，
    /// 官方的 `CVI_VPSS_SetGrpProcAmp` 底层走的就是它）。
    ///
    /// 硬件公式：`out_i = Σ_j coef[i][j] * (in_j - sub[j]) + add[i]`，
    /// 其中 `in = Y/U/V`、`out = R/G/B`。实测驱动默认给的是
    /// **full range** 矩阵（`R = Y + 1.402(V-128)`），而 UVC 相机出的是
    /// BT.601 limited（Y 16~235），直接用会让画面偏灰、模型置信度下降 ——
    /// 用 [`Vpss::set_yuv601_limited_to_full`] 换成 limited→full 的矩阵。
    pub fn set_grp_csc(&self, coef: [[f32; 3]; 3], sub: [u8; 3], add: [u8; 3]) -> Result<()> {
        let cfg = ffi::vpss_grp_csc_cfg {
            VpssGrp: self.grp,
            proc_amp: [50, 50, 50, 50], // 与 `_vpss_proamp_2_csc` 的基准一致
            coef: coef.map(|row| row.map(fixed10)),
            sub,
            add,
            scene: 0,
        };
        ioctl_write(
            self.fd,
            ffi::CVI_VPSS_SET_GRP_CSC_CFG,
            &cfg,
            "CVI_VPSS_SetGrpCsc",
        )
    }

    /// 把 YUV 输入按 **BT.601 limited → full range** 转到 RGB。
    ///
    /// 系数是 BT.601 limited → full 的整数公式
    /// （`298/409/100/208/516` ÷ 256），模型看到的输入分布与预期一致。
    pub fn set_yuv601_limited_to_full(&self) -> Result<()> {
        self.set_grp_csc(
            [
                [1.164_062_5, 0.0, 1.597_656_3],
                [1.164_062_5, -0.390_625, -0.812_5],
                [1.164_062_5, 2.015_625, 0.0],
            ],
            [16, 128, 128],
            [0, 0, 0],
        )
    }
}

/// 查询某个 VENC 通道当前绑定的源（诊断用；没有绑定时返回 `None`）。
///
/// 注意：内核在「没有绑定」时 `CVI_SYS_GetBindbyDest` 返回**成功 + 全零 src**
/// （`libsys` 清零后没填；`0 = CVI_ID_BASE`，不可能是真实源），所以这里过滤掉
/// 全零值 —— 否则每次启动都会误报「残留绑定」。
pub fn venc_bind_source(venc_chn: ffi::VENC_CHN) -> Option<ffi::MMF_CHN_S> {
    let dst = ffi::mmf_chn(ffi::CVI_ID_VENC, 0, venc_chn);
    let mut src = ffi::mmf_chn(ffi::CVI_ID_VPSS, -1, -1);
    let ret = unsafe { ffi::CVI_SYS_GetBindbyDest(&dst, &mut src) };
    let zeroed = src.enModId == 0 && src.s32DevId == 0 && src.s32ChnId == 0;
    (ret == ffi::CVI_SUCCESS && !zeroed).then_some(src)
}

/// 清掉 VENC 通道上可能残留的绑定（上次进程崩溃 / `kill -9` 留下的 bind 节点）。
///
/// 残留节点很坑：新的 `CVI_SYS_Bind` 会返回成功，但数据不通 —— `GetStream` 一直报
/// `EN_ERR_BUSY`，VPSS 的 chn1 输出被堵死，最后整组（包括 chn0）都停止出帧。
/// 所以绑定前先查一次，有残留就先解绑。返回 `true` 表示清掉了残留。
pub fn clear_venc_bind(venc_chn: ffi::VENC_CHN) -> Result<bool> {
    let Some(src) = venc_bind_source(venc_chn) else {
        return Ok(false);
    };
    let dst = ffi::mmf_chn(ffi::CVI_ID_VENC, 0, venc_chn);
    check(
        unsafe { ffi::CVI_SYS_UnBind(&src, &dst) },
        "CVI_SYS_UnBind(残留绑定)",
    )?;
    Ok(true)
}

/// 浮点系数 → 硬件的 13 位有符号定点（10 位小数，负数用 `BIT(13)` 标记）。
fn fixed10(v: f32) -> u16 {
    let scaled = (v.abs() * 1024.0).round() as u32 & 0x1fff;
    let signed = if v < 0.0 { scaled | 0x2000 } else { scaled };
    signed as u16
}

/// 建组前下发 VI/VPSS 模式：对应 `CVI_SYS_Init` 首次初始化时做的事
/// （VI 全部 offline、VPSS 单设备且输入来自内存）。C sample 也是这么做的
/// （`_sys_config_online_mode` / `SAMPLE_COMM_SYS_Init`）。
fn set_vpss_modes() -> Result<()> {
    let vivpss = ffi::VI_VPSS_MODE_S {
        aenMode: [ffi::VI_OFFLINE_VPSS_OFFLINE; ffi::VI_MAX_PIPE_NUM],
    };
    check(
        unsafe { ffi::CVI_SYS_SetVIVPSSMode(&vivpss) },
        "CVI_SYS_SetVIVPSSMode",
    )?;
    let vpss_mode = ffi::VPSS_MODE_S {
        enMode: ffi::VPSS_MODE_SINGLE,
        aenInput: [ffi::VPSS_INPUT_MEM; ffi::VPSS_IP_NUM],
        ViPipe: [0; ffi::VPSS_IP_NUM],
    };
    check(
        unsafe { ffi::CVI_SYS_SetVPSSModeEx(&vpss_mode) },
        "CVI_SYS_SetVPSSModeEx",
    )
}

fn ioctl_write<T>(fd: RawFd, cmd: u64, arg: &T, op: &'static str) -> Result<()> {
    let ret = unsafe { libc::ioctl(fd, cmd as _, arg as *const T) };
    ioctl_check(ret, op)
}

fn ioctl_readwrite<T>(fd: RawFd, cmd: u64, arg: &mut T, op: &'static str) -> Result<()> {
    let ret = unsafe { libc::ioctl(fd, cmd as _, arg as *mut T) };
    ioctl_check(ret, op)
}

impl Drop for Vpss<'_> {
    fn drop(&mut self) {
        // 尽力而为地按 sample 的顺序回收；失败只可能是组已经被内核清掉。
        let chns = core::mem::take(&mut self.chns);
        for chn in chns {
            let cfg = ffi::vpss_en_chn_cfg {
                VpssGrp: self.grp,
                VpssChn: chn,
            };
            let _ = ioctl_write(
                self.fd,
                ffi::CVI_VPSS_DISABLE_CHN,
                &cfg,
                "CVI_VPSS_DisableChn",
            );
        }
        if self.started {
            self.started = false;
            let _ = ioctl_write(
                self.fd,
                ffi::CVI_VPSS_STOP_GROUP,
                &self.grp,
                "CVI_VPSS_StopGrp",
            );
        }
        if self.fd >= 0 {
            let _ = unsafe { libc::close(self.fd) };
            self.fd = -1;
        }
    }
}

/// 只有组号是内核里的全局状态，`Vpss` 本身可以在线程间移动。
unsafe impl Send for Vpss<'_> {}

/* ------------------------------------------------------------------ */
/* 通道输出帧                                                          */
/* ------------------------------------------------------------------ */

/// [`Vpss::get_chn_frame`] 返回的通道输出帧；析构时 `ReleaseChnFrame`。
///
/// 帧内存由 VB 池持有，`phy_addr()` 可以直接交给 TPU 做零拷贝输入
/// （`cviruntime` 的 `CVI_NN_SetTensorPhysicalAddr`）。CPU 要读像素时用
/// [`VpssFrame::map`] / [`VpssFrame::copy_tight`]（默认 uncached 映射，硬件写完即一致）。
pub struct VpssFrame<'s> {
    fd: RawFd,
    grp: ffi::VPSS_GRP,
    chn: ffi::VPSS_CHN,
    info: ffi::VIDEO_FRAME_INFO_S,
    /// 已建立的平面映射：`(地址, 长度)`。
    maps: [Option<(*mut c_void, u32)>; 3],
    _vpss: PhantomData<&'s ()>,
}

impl VpssFrame<'_> {
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

    pub fn plane_len(&self, plane: usize) -> u32 {
        self.info.stVFrame.u32Length[plane]
    }

    /// 平面物理地址（交给硬件用；`0` 表示该平面不存在）。
    pub fn phy_addr(&self, plane: usize) -> u64 {
        self.info.stVFrame.u64PhyAddr[plane]
    }

    /// 平面数（按像素格式推断；驱动实际给出的非空平面数为准）。
    pub fn plane_count(&self) -> usize {
        plane_dims(self.pixel_format(), self.width(), self.height())
            .map(|d| d.planes)
            .unwrap_or_else(|| {
                (0..3)
                    .filter(|&p| self.info.stVFrame.u64PhyAddr[p] != 0)
                    .count()
            })
    }

    /// 映射某平面为可读切片（`stride * rows` 字节）。
    ///
    /// `cached = false` 用 `CVI_SYS_Mmap`（uncached，硬件写完立即可见，适合读像素）；
    /// `cached = true` 用 `CVI_SYS_MmapCache` 并做一次 `IonInvalidateCache`
    /// （CPU 连着算、读取量大时更快）。
    pub fn map(&mut self, plane: usize, cached: bool) -> Result<&[u8]> {
        let dims = plane_dims(self.pixel_format(), self.width(), self.height())
            .ok_or_else(|| Error::invalid("VpssFrame::map (unsupported pixel format)"))?;
        if plane >= dims.planes {
            return Err(Error::invalid("VpssFrame::map (plane index)"));
        }
        if self.maps[plane].is_none() {
            let phy = self.info.stVFrame.u64PhyAddr[plane];
            let len = (self.info.stVFrame.u32Stride[plane] as usize)
                .checked_mul(dims.rows[plane] as usize)
                .ok_or_else(|| Error::invalid("VpssFrame::map (size overflow)"))?
                as u32;
            if phy == 0 || len == 0 {
                return Err(Error::invalid("VpssFrame::map (plane unavailable)"));
            }
            let ptr = unsafe {
                if cached {
                    ffi::CVI_SYS_MmapCache(phy, len)
                } else {
                    ffi::CVI_SYS_Mmap(phy, len)
                }
            };
            if ptr.is_null() {
                return Err(Error::invalid("VpssFrame::map (CVI_SYS_Mmap)"));
            }
            if cached {
                unsafe {
                    ffi::CVI_SYS_IonInvalidateCache(phy, ptr, len);
                }
            }
            self.maps[plane] = Some((ptr, len));
        }
        let (ptr, len) = self.maps[plane].expect("just mapped");
        // SAFETY: 映射了恰好 `len` 字节，帧释放前一直有效。
        Ok(unsafe { slice::from_raw_parts(ptr.cast::<u8>(), len as usize) })
    }

    /// 把各平面拷成紧凑缓冲（RGB 平面 = R|G|B 三段；NV12 = Y|UV 交织）。
    pub fn copy_tight(&mut self) -> Result<Vec<u8>> {
        let fmt = self.pixel_format();
        let dims = plane_dims(fmt, self.width(), self.height())
            .ok_or_else(|| Error::invalid("VpssFrame::copy_tight (unsupported pixel format)"))?;
        let total = tight_frame_size(fmt, self.width(), self.height())
            .ok_or_else(|| Error::invalid("VpssFrame::copy_tight (size overflow)"))?
            as usize;

        let mut out = vec![0u8; total];
        let mut off = 0usize;
        for p in 0..dims.planes {
            let (row, rows, stride) = (
                dims.row_bytes[p] as usize,
                dims.rows[p] as usize,
                self.info.stVFrame.u32Stride[p] as usize,
            );
            let plane = self.map(p, false)?;
            for r in 0..rows {
                out[off..off + row].copy_from_slice(&plane[r * stride..r * stride + row]);
                off += row;
            }
        }
        Ok(out)
    }

    /// 显式释放（等价于析构）。
    pub fn release(mut self) -> Result<()> {
        self.release_inner()
    }

    fn release_inner(&mut self) -> Result<()> {
        self.unmap_all();
        let cfg = ffi::vpss_chn_frm_cfg {
            VpssGrp: self.grp,
            VpssChn: self.chn,
            stVideoFrame: self.info,
            s32MilliSec: 0,
        };
        let ret = unsafe {
            libc::ioctl(
                self.fd,
                ffi::CVI_VPSS_RELEASE_CHN_FRAME as _,
                &cfg as *const _,
            )
        };
        ioctl_check(ret, "CVI_VPSS_ReleaseChnFrame")
    }

    fn unmap_all(&mut self) {
        for slot in self.maps.iter_mut() {
            if let Some((ptr, len)) = slot.take() {
                unsafe {
                    ffi::CVI_SYS_Munmap(ptr, len);
                }
            }
        }
    }
}

impl Drop for VpssFrame<'_> {
    fn drop(&mut self) {
        let _ = self.release_inner();
    }
}

/// 与 `encoder::Frame` 同理：只持有 VB 内存地址与簿记信息，可安全移到别的线程。
unsafe impl Send for VpssFrame<'_> {}

/* ------------------------------------------------------------------ */
/* 底层                                                                */
/* ------------------------------------------------------------------ */

fn open_device(path: &str) -> Result<RawFd> {
    let c_path = std::ffi::CString::new(path).map_err(|_| Error::invalid("open (path)"))?;
    let fd = unsafe { libc::open(c_path.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    if fd < 0 {
        return Err(errno("open(/dev/cvi-vpss)"));
    }
    Ok(fd)
}

/// 当前线程的 errno 转成 [`Error`]（`code` 为 `-errno`，见 [`errno_str`]）。
fn errno(op: &'static str) -> Error {
    let err = unsafe { *libc::__errno_location() };
    Error::from_op(-err, op)
}

fn ioctl_check(ret: libc::c_int, op: &'static str) -> Result<()> {
    if ret >= 0 { Ok(()) } else { Err(errno(op)) }
}

/// 把 [`Error`] 里的负 errno 翻成 `strerror` 文本；不是 errno 错误时返回 `None`。
pub fn errno_str(err: &Error) -> Option<String> {
    if !(0..4096).contains(&-err.code) {
        return None;
    }
    let ptr = unsafe { libc::strerror(-err.code) };
    if ptr.is_null() {
        return None;
    }
    // SAFETY: `strerror` 返回 NUL 结尾的静态字符串。
    Some(
        unsafe { std::ffi::CStr::from_ptr(ptr) }
            .to_string_lossy()
            .into_owned(),
    )
}

// ABI（结构体尺寸 / 字段偏移 / ioctl 号）的编译期校验在 [`crate::ffi`] 里，
// 对照 C 头的实测值见 `tools/vpss_abi_probe.c`。
