//! 硬件 JPEG 编码（SG2002 VENC / `PT_JPEG`），基于 [`cvimpi_rs`] 的安全封装。
//!
//! 相比纯软件编码（C906 上 640x480 约 70~135ms），硬件路径约 12ms（含转换）。
//!
//! 相机出的是 **YUYV422**，而 VENC 的 JPEG 通道只认 semi-planar YUV420
//! （把 YUYV 直接交给驱动会被当成 NV12 解释，画面变成绿/品红条纹），
//! 所以编码前要写一份 NV12：转换函数在 [`crate::preprocess`]
//! （`yuyv422_to_nv12_*`），这里只负责按平面写进 VB 块。输入帧用
//! `cvimpi-rs` 的 cached 映射（`alloc_frame_cached`）：CPU 逐字节写 cached
//! 内存比 uncached 快一个数量级（实测 460KB：25ms → 3.3ms），
//! `Encoder::send_frame` 会在送硬件前自动 flush。
//!
//! 每帧的转换/VENC 分项耗时可把日志级别开到 `debug`
//! （`SMARTCAR_LOG=debug` 或 `SMARTCAR_LOG=sg2002_upper::hwjpeg=debug`）。
//!
//! 三个约定：
//! - [`HwJpeg`] **不是 `Send`**：VENC 通道与线程绑定，`new`/`encode`/`Drop` 必须同一线程；
//! - 输入固定 640x480 YUYV422（与相机一致）；
//! - 任何失败都返回 `io::Error`，上层据此降级到软件编码。
//!
//! 编解码会话（`CVI_SYS_Init` + 公共 VB 池）**由调用方提供**：进程里只能有一个
//! [`Sys`]（`CVI_SYS_Init` 是进程级状态），CPU 管线把它和 TPU 零拷贝输入帧
//! 共用（见 `crate::vision::cpu`），所以这里只借用、不拥有。池的 block 要
//! 放得下 NV12 输入帧（`venc_input_layout`）。

use std::io;
use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Instant;

use cvimpi_rs::encoder::{Encoder, EncoderConfig, Frame};
use cvimpi_rs::ffi;
use cvimpi_rs::sys::Sys;

use crate::preprocess::{FRAME_H, FRAME_W, YUYV_LEN, yuyv422_to_nv12_uv, yuyv422_to_nv12_y};

/// 编码输入帧的像素格式：驱动只接受 semi-planar YUV420（见模块注释）。
const INPUT_FORMAT: ffi::PIXEL_FORMAT_E = ffi::PIXEL_FORMAT_NV12;

/// 硬件 JPEG 编码器句柄（与创建它的线程绑定）。
pub struct HwJpeg<'a> {
    /// VENC 通道。字段声明顺序 = 析构顺序：`enc`/`frame` 先于调用方的会话，
    /// 保证句柄不会在会话之后被拆掉。
    enc: Encoder<'a>,
    /// 复用的编码输入帧（VB 块），避免每帧重新申请/映射。
    frame: Frame<'a>,
    /// 保持 `!Send`：VENC 通道必须固定线程使用。
    _not_send: PhantomData<*const ()>,
}

impl<'a> HwJpeg<'a> {
    /// 在会话 `sys` 上打开 VENC 通道；失败（没有厂商库/内核不支持/参数非法）
    /// 返回错误，上层降级到软件编码。
    pub fn new(sys: &'a Sys, width: u32, height: u32, quality: u8) -> io::Result<Self> {
        if width as usize != FRAME_W || height as usize != FRAME_H {
            return Err(io::Error::other(format!(
                "硬件编码只支持 {FRAME_W}x{FRAME_H}，收到 {width}x{height}"
            )));
        }
        // 50 是「用户量化表」的特殊值（需要调用方给出 u8YQt 等表），避开它。
        let quality = if quality == 50 {
            51
        } else {
            quality.clamp(1, 99)
        };

        let enc = sys
            .create_encoder(
                0,
                &EncoderConfig::new(width, height, INPUT_FORMAT).with_quality(u32::from(quality)),
            )
            .map_err(|e| io::Error::other(format!("CVI_VENC 通道创建失败: {e}")))?;
        let frame = sys
            .alloc_frame_cached(width, height, INPUT_FORMAT)
            .map_err(|e| io::Error::other(format!("编码输入帧分配失败: {e}")))?;

        Ok(Self {
            enc,
            frame,
            _not_send: PhantomData,
        })
    }

    /// 编码一帧 640x480 YUYV422，返回 JPEG 字节。
    pub fn encode(&mut self, yuyv: &[u8]) -> io::Result<Arc<[u8]>> {
        if yuyv.len() < YUYV_LEN {
            return Err(io::Error::other(format!(
                "输入长度不足: {} < {YUYV_LEN}",
                yuyv.len()
            )));
        }
        let t0 = Instant::now();
        write_nv12(&mut self.frame, yuyv)
            .map_err(|e| io::Error::other(format!("YUYV→NV12 转换失败: {e}")))?;
        let t1 = Instant::now();
        let jpeg = self
            .enc
            .encode(&self.frame, ffi::CVI_IO_BLOCK)
            .map_err(|e| io::Error::other(format!("硬件编码失败: {e}")))?;
        log::debug!(
            "转换 {:.1}ms + VENC {:.1}ms = {:.1}ms（{} 字节）",
            t1.duration_since(t0).as_secs_f64() * 1000.0,
            t1.elapsed().as_secs_f64() * 1000.0,
            t0.elapsed().as_secs_f64() * 1000.0,
            jpeg.len()
        );
        if jpeg.is_empty() {
            return Err(io::Error::other("硬件编码输出为空"));
        }
        Ok(Arc::from(jpeg))
    }

    /// 实际使用的输入像素格式（SDK 的 `PIXEL_FORMAT_E` 数值，日志用）。
    pub fn input_format(&self) -> u32 {
        INPUT_FORMAT as u32
    }
}

/// 把 640x480 YUYV422 写进 NV12 输入帧（自动处理硬件 stride）。
///
/// 像素转换在 [`crate::preprocess`]（`yuyv422_to_nv12_*`），这里只负责把
/// 结果按平面写进 VB 块；输入帧是 cached 映射（`Sys::alloc_frame_cached`），
/// 写完由 `Encoder::send_frame` 统一 flush 给硬件。
fn write_nv12(frame: &mut Frame<'_>, yuyv: &[u8]) -> cvimpi_rs::Result<()> {
    let y_stride = frame.stride(0) as usize;
    yuyv422_to_nv12_y(yuyv, frame.plane_mut(0)?, y_stride);
    let uv_stride = frame.stride(1) as usize;
    yuyv422_to_nv12_uv(yuyv, frame.plane_mut(1)?, uv_stride);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cvimpi_rs::sys::VbPoolConfig;
    use cvimpi_rs::venc_input_layout;

    /// 构造「YUYV：U=10+行号%2、V=200-列号%2、Y=列号」的可区分测试图。
    fn yuyv_pattern() -> Vec<u8> {
        let mut buf = vec![0u8; YUYV_LEN];
        for y in 0..FRAME_H {
            let row = &mut buf[y * FRAME_W * 2..(y + 1) * FRAME_W * 2];
            for pair in 0..FRAME_W / 2 {
                let x = pair * 2;
                row[pair * 4] = x as u8;
                row[pair * 4 + 1] = 10;
                row[pair * 4 + 2] = (x + 1) as u8;
                row[pair * 4 + 3] = 200;
            }
        }
        buf
    }

    /// 输出指纹（校验缓存/刷写路径的编码结果）。
    fn sum_bytes(b: &[u8]) -> u64 {
        b.iter().map(|&x| u64::from(x)).sum()
    }

    /// 真机基准（需要厂商库）：`cargo test --release -- --ignored --nocapture`。
    ///
    /// 生产路径（cached 输入帧，`send_frame` 自动 flush）对比 uncached 直写；
    /// 顺带校验两帧不同输入经 cached 路径编码结果确实不同（flush 生效）。
    #[test]
    #[ignore = "需要 SG2002 硬件（VENC + 厂商库）"]
    fn bench_encode_on_board() {
        use std::sync::atomic::{Ordering, fence};

        let rounds = 30u32;
        let yuyv = yuyv_pattern();
        let (w, h) = (FRAME_W as u32, FRAME_H as u32);
        let layout = venc_input_layout(w, h, INPUT_FORMAT).expect("NV12 布局");
        let sys = Sys::init(&[VbPoolConfig::new(layout.vb_size, 4).with_name("hwjpeg")])
            .expect("JPEG 会话初始化失败");
        let mut hw = HwJpeg::new(&sys, w, h, 70).expect("硬件编码不可用");
        let ms = |d: std::time::Duration| d.as_secs_f64() * 1000.0;
        assert!(hw.frame.is_cached(), "生产路径应使用 cached 输入帧");

        // 预热：首帧要等 VENC 起流，不参与统计
        for _ in 0..3 {
            write_nv12(&mut hw.frame, &yuyv).unwrap();
            hw.enc.encode(&hw.frame, ffi::CVI_IO_BLOCK).unwrap();
        }

        // 生产路径：cached 输入帧（转换写 cached；flush 在 send_frame 里）
        let (mut conv, mut venc) = (0.0f64, 0.0f64);
        for _ in 0..rounds {
            let t0 = Instant::now();
            write_nv12(&mut hw.frame, &yuyv).unwrap();
            let t1 = Instant::now();
            hw.enc.encode(&hw.frame, ffi::CVI_IO_BLOCK).unwrap();
            let t2 = Instant::now();
            conv += ms(t1 - t0);
            venc += ms(t2 - t1);
        }
        println!(
            "cached 输入帧（生产）：转换 {:.1} + encode(含 flush) {:.1} = {:.1}ms/帧",
            conv / rounds as f64,
            venc / rounds as f64,
            (conv + venc) / rounds as f64
        );

        // 对照：uncached 输入帧直写（store 是 posted 的，加 fence 才是真实耗时）
        let mut uncached = sys.alloc_frame(w, h, INPUT_FORMAT).unwrap();
        let (mut conv, mut venc) = (0.0f64, 0.0f64);
        for _ in 0..rounds {
            fence(Ordering::SeqCst);
            let t0 = Instant::now();
            write_nv12(&mut uncached, &yuyv).unwrap();
            fence(Ordering::SeqCst);
            let t1 = Instant::now();
            hw.enc.encode(&uncached, ffi::CVI_IO_BLOCK).unwrap();
            let t2 = Instant::now();
            conv += ms(t1 - t0);
            venc += ms(t2 - t1);
        }
        println!(
            "uncached 直写（对照） ：转换 {:.1} + VENC {:.1} = {:.1}ms/帧",
            conv / rounds as f64,
            venc / rounds as f64,
            (conv + venc) / rounds as f64
        );

        // cached 路径的正确性：两帧差异极大的输入必须编出不同的 JPEG（flush 生效）
        let yuyv_b: Vec<u8> = yuyv.iter().map(|b| !b).collect();
        let mut sums = [0u64; 2];
        for (i, src) in [&yuyv, &yuyv_b].into_iter().enumerate() {
            write_nv12(&mut hw.frame, src).unwrap();
            let jpeg = hw.enc.encode(&hw.frame, ffi::CVI_IO_BLOCK).unwrap();
            sums[i] = sum_bytes(&jpeg);
        }
        assert_ne!(
            sums[0], sums[1],
            "cached 帧两帧输入不同却编码结果相同（flush 失效？）"
        );
    }
}
