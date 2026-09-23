//! 会话所有者：`CVI_SYS_Init` / `CVI_SYS_Exit`，以及全局 VB 池
//! （`CVI_VB_SetConfig` / `CVI_VB_Init` / `CVI_VB_Exit`）。
//!
//! [`Sys`] 是进程级中间件状态的唯一所有者：创建它就会同时初始化系统上下文和 VB 池；
//! 所有 encoder、decoder、frame 都*经由*它创建并借用它，因此"先拆资源、后使用句柄"
//! 会被借用检查器直接拒绝：
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
//!     &EncoderConfig::new(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420),
//! )?;
//! let mut frame = sys.alloc_frame(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420)?;
//! frame.write_tight(&vec![0u8; (1920 * 1080 * 3 / 2) as usize])?;
//! let _jpeg = enc.encode(&frame, ffi::CVI_IO_BLOCK)?;
//! # Ok(())
//! # }
//! ```
//!
//! 句柄还活着时析构 `Sys` —— 编译不过：
//!
//! ```compile_fail
//! use cvimpi_rs::{encoder::EncoderConfig, ffi, sys::{Sys, VbPoolConfig}};
//!
//! fn main() -> Result<(), cvimpi_rs::Error> {
//!     let sys = Sys::init(&[VbPoolConfig::new(1 << 20, 1)])?;
//!     let enc = sys.create_encoder(0, &EncoderConfig::new(64, 64, ffi::PIXEL_FORMAT_NV12))?;
//!     drop(sys);
//!     let _ = enc.channel();
//!     Ok(())
//! }
//! ```
//!
//! `DecodedFrame` 比它的 `Decoder` 活得久也一样：
//!
//! ```compile_fail
//! use cvimpi_rs::{decoder::DecoderConfig, ffi, sys::{Sys, VbPoolConfig}};
//!
//! fn main() -> Result<(), cvimpi_rs::Error> {
//!     let sys = Sys::init(&[VbPoolConfig::new(1 << 20, 1)])?;
//!     let dec = sys.create_decoder(0, &DecoderConfig::new(64, 64))?;
//!     let frame = dec.decode(&[], ffi::CVI_IO_BLOCK)?;
//!     drop(dec);
//!     let _ = frame.width();
//!     Ok(())
//! }
//! ```

use core::mem;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::decoder::{Decoder, DecoderConfig};
use crate::encoder::{Encoder, EncoderConfig, Frame};
use crate::{Error, Result, check, ffi};

/// 进程级归属标志：中间件只能被初始化一次。
static SYS_OWNED: AtomicBool = AtomicBool::new(false);

/// 一个 common VB 池的配置。
#[derive(Debug, Clone, Copy)]
pub struct VbPoolConfig {
    pub blk_size: u32,
    pub blk_cnt: u32,
    pub remap_mode: ffi::VB_REMAP_MODE_E,
    pub name: [ffi::CVI_CHAR; ffi::MAX_VB_POOL_NAME_LEN],
}

impl VbPoolConfig {
    pub fn new(blk_size: u32, blk_cnt: u32) -> Self {
        let mut name = [0 as ffi::CVI_CHAR; ffi::MAX_VB_POOL_NAME_LEN];
        let tag = b"cvimpi";
        for (dst, src) in name.iter_mut().zip(tag.iter()) {
            *dst = *src as ffi::CVI_CHAR;
        }
        Self {
            blk_size,
            blk_cnt,
            remap_mode: ffi::VB_REMAP_MODE_NONE,
            name,
        }
    }

    pub fn with_name(mut self, name: &str) -> Self {
        let mut buf = [0 as ffi::CVI_CHAR; ffi::MAX_VB_POOL_NAME_LEN];
        for (i, b) in name.bytes().take(ffi::MAX_VB_POOL_NAME_LEN - 1).enumerate() {
            buf[i] = b as ffi::CVI_CHAR;
        }
        self.name = buf;
        self
    }
}

/// 内核里实际生效的一个 common 池（`CVI_VB_GetConfig` 的返回项）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KernelPool {
    pub blk_size: u32,
    pub blk_cnt: u32,
}

/// 持有 `CVI_SYS_Init` 和全局 VB 池，并派生所有 JPEG 句柄。
///
/// 初始化流程与官方 sample（`SAMPLE_COMM_SYS_Init`）/ `csrc` 一致：
/// `CVI_SYS_Exit` + `CVI_VB_Exit`（清残留在内核里的 MMF 状态）→
/// `CVI_VB_SetConfig` → `CVI_VB_Init` → `CVI_SYS_Init`；
/// 析构时按 `CVI_SYS_Exit` → `CVI_VB_Exit` 释放。
///
/// 初始化后会再用 `CVI_VB_GetConfig` **读回内核实际生效的池**：
///
/// * 正常情况下就是本次请求的配置；
/// * 如果上一次 MMF 进程是崩溃 / 被 SIGKILL 结束的，内核里的池和块引用计数
///   不会被回收，`VB_SetConfig`/`VB_Init`/`VB_Exit`/`VB_DestroyPool` 都动不了它
///   （实测 `DestroyPool` 返回成功但池仍在）。此时驱动会继续使用那个残留池，
///   所以各种判断必须以**读回的配置**为准，例如解码通道的 `u32FrameBufCnt`
///   要不小于"实际池"的块数，否则 `SendStream` 报 `BUF_FULL`。
///
/// 残留池可以继续用（只要 block 够大），但块引用计数也一起残留，可用块会变少；
/// 遇到奇怪的分配失败时重启设备是最干净的解法。
pub struct Sys {
    /// 内核里**实际生效**的 common 池（`CVI_VB_GetConfig` 读回，可能含残留池）。
    kernel_pools: Vec<KernelPool>,
    /// 内核池与本次请求是否一致；`CVI_VB_GetConfig` 失败时为 `None`。
    pools_match_request: Option<bool>,
    /// 所有池中最大的块数（按内核实际配置算）。
    ///
    /// VDEC 在 common 池模式下有个硬性约束：通道的 `u32FrameBufCnt` 必须
    /// **不小于**它使用的池的块数，否则 `CVI_VDEC_SendStream` 会返回
    /// `CVI_ERR_VDEC_BUF_FULL`（实测：池 2 块 + `FrameBufCnt=1` 必失败，
    /// 2/2 或 1/1 正常）。驱动取第一个池，这里取所有池的最大值更保险。
    max_pool_blk_cnt: u32,
}

impl Sys {
    /// 初始化中间件和全局 VB 池。
    ///
    /// 如果本进程已经初始化过，或 `pools` 为空 / 超过 `VB_MAX_COMM_POOLS`，
    /// 会返回 [`Error`]。
    pub fn init(pools: &[VbPoolConfig]) -> Result<Sys> {
        if pools.is_empty() || pools.len() > ffi::VB_MAX_COMM_POOLS {
            return Err(Error::invalid("Sys::init (pool count)"));
        }
        if SYS_OWNED
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            return Err(Error::invalid("Sys::init (already initialized)"));
        }

        // 与 sample（SAMPLE_COMM_SYS_Init）/ csrc 一致：先复位可能残留的 MMF 状态，
        // 再按 VB → SYS 的顺序初始化。
        unsafe {
            ffi::CVI_SYS_Exit();
            ffi::CVI_VB_Exit();
        }

        if let Err(e) = init_vb(pools) {
            SYS_OWNED.store(false, Ordering::SeqCst);
            return Err(e);
        }

        if let Err(e) = check(unsafe { ffi::CVI_SYS_Init() }, "CVI_SYS_Init") {
            unsafe {
                ffi::CVI_VB_Exit();
            }
            SYS_OWNED.store(false, Ordering::SeqCst);
            return Err(e);
        }

        // 读回内核实际生效的池：残留池（上次进程崩溃/SIGKILL 留下）会遮住本次请求，
        // 后续所有判断都必须以读回结果为准。
        let kernel_pools = query_kernel_pools();
        let (kernel_pools, pools_match_request) = match kernel_pools {
            Some(k) => {
                let same = same_pools(&k, pools);
                (k, Some(same))
            }
            None => (
                pools
                    .iter()
                    .map(|p| KernelPool {
                        blk_size: p.blk_size,
                        blk_cnt: p.blk_cnt,
                    })
                    .collect(),
                None,
            ),
        };
        let max_pool_blk_cnt = kernel_pools.iter().map(|p| p.blk_cnt).max().unwrap_or(1);

        Ok(Sys {
            kernel_pools,
            pools_match_request,
            max_pool_blk_cnt,
        })
    }

    /// 内核里实际生效的 common 池（`CVI_VB_GetConfig` 的结果；查询失败时为本次请求值）。
    pub fn kernel_pools(&self) -> &[KernelPool] {
        &self.kernel_pools
    }

    /// 内核池是否与本次请求一致。`None` = 查询失败（按请求值继续使用）。
    ///
    /// `Some(false)` 通常意味着上一次 MMF 进程崩溃/被强杀，内核里留着它的池；
    /// 这些池可以继续用，但块引用计数也被残留，可用块可能变少。
    pub fn pools_match_request(&self) -> Option<bool> {
        self.pools_match_request
    }

    /// 所有 common 池中最大的块数（见 [`Sys`] 结构体的说明）。
    pub fn max_pool_blk_cnt(&self) -> u32 {
        self.max_pool_blk_cnt
    }

    /// `CVI_VENC_CreateChn` + `CVI_VENC_SetJpegParam` + `CVI_VENC_StartRecvFrame`。
    pub fn create_encoder<'a>(
        &'a self,
        chn: ffi::VENC_CHN,
        cfg: &EncoderConfig,
    ) -> Result<Encoder<'a>> {
        Encoder::create(self, chn, cfg)
    }

    /// `CVI_VDEC_CreateChn` + `CVI_VDEC_SetChnParam` + `CVI_VDEC_StartRecvStream`。
    pub fn create_decoder<'a>(
        &'a self,
        chn: ffi::VDEC_CHN,
        cfg: &DecoderConfig,
    ) -> Result<Decoder<'a>> {
        Decoder::create(self, chn, cfg)
    }

    /// 申请一个 VB block 并映射好，用作编码输入帧。
    ///
    /// 池的 block 必须不小于 `venc_input_layout(width, height, fmt).vb_size`。
    pub fn alloc_frame<'a>(
        &'a self,
        width: u32,
        height: u32,
        fmt: ffi::PIXEL_FORMAT_E,
    ) -> Result<Frame<'a>> {
        Frame::alloc(self, width, height, fmt)
    }
}

/// 全局 VB 池：`CVI_VB_SetConfig` + `CVI_VB_Init`。
fn init_vb(pools: &[VbPoolConfig]) -> Result<()> {
    let mut cfg: ffi::VB_CONFIG_S = unsafe { mem::zeroed() };
    cfg.u32MaxPoolCnt = pools.len() as u32;
    for (dst, src) in cfg.astCommPool.iter_mut().zip(pools.iter()) {
        dst.u32BlkSize = src.blk_size;
        dst.u32BlkCnt = src.blk_cnt;
        dst.enRemapMode = src.remap_mode;
        dst.acName = src.name;
    }

    check(unsafe { ffi::CVI_VB_SetConfig(&cfg) }, "CVI_VB_SetConfig")?;
    check(unsafe { ffi::CVI_VB_Init() }, "CVI_VB_Init")
}

/// `CVI_VB_GetConfig` 读回内核实际生效的 common 池；失败返回 `None`。
fn query_kernel_pools() -> Option<Vec<KernelPool>> {
    let mut cfg: ffi::VB_CONFIG_S = unsafe { mem::zeroed() };
    if unsafe { ffi::CVI_VB_GetConfig(&mut cfg) } != ffi::CVI_SUCCESS {
        return None;
    }
    let n = (cfg.u32MaxPoolCnt as usize).min(ffi::VB_MAX_COMM_POOLS);
    Some(
        cfg.astCommPool[..n]
            .iter()
            .map(|p| KernelPool {
                blk_size: p.u32BlkSize,
                blk_cnt: p.u32BlkCnt,
            })
            .collect(),
    )
}

/// 内核池与请求配置是否逐项一致（忽略池名）。
fn same_pools(kernel: &[KernelPool], requested: &[VbPoolConfig]) -> bool {
    kernel.len() == requested.len()
        && kernel
            .iter()
            .zip(requested)
            .all(|(k, r)| k.blk_size == r.blk_size && k.blk_cnt == r.blk_cnt)
}

impl Drop for Sys {
    fn drop(&mut self) {
        // 顺序与 sample 的 `SAMPLE_COMM_SYS_Exit` 一致（SYS 先于 VB）。
        // 句柄借用 `Sys`，因此执行到这里时它们都已经析构。
        unsafe {
            ffi::CVI_SYS_Exit();
            ffi::CVI_VB_Exit();
        }
        SYS_OWNED.store(false, Ordering::SeqCst);
    }
}
