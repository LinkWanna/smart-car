//! 相机 YUYV422 帧的像素工作流（取帧之后的全部处理）。
//!
//! - 模型侧：[`Preprocessor`] 把 YUYV422 转成 RGB 平面 CHW（TPU 输入）；
//! - 预览侧：[`PreviewEncoder`] 把同一帧编码成 JPEG（硬件 VENC 优先，失败降级软件编码）；
//! - 共用转换：[`yuyv422_to_rgb`]、[`yuyv422_to_nv12_y`] / [`yuyv422_to_nv12_uv`]。
//!
//! 相机采集本身在 [`crate::camera`]；帧尺寸固定 [`FRAME_W`]x[`FRAME_H`]，
//! 与相机协商格式、模型输入、预览编码一致。

use std::io;
use std::sync::Arc;

use jpeg_encoder::{ColorType, Encoder};
use log::{info, warn};

use crate::hwjpeg::HwJpeg;

/// 帧宽（相机 / 模型输入 / 预览编码一致）。
pub const FRAME_W: usize = 640;
/// 帧高。
pub const FRAME_H: usize = 480;
/// 一帧 YUYV422 的字节数（2 字节/像素）。
pub const YUYV_LEN: usize = FRAME_W * 2 * FRAME_H;

/// YUYV422 (640x480) → RGB 平面 CHW (640x480)
pub fn yuyv422_to_rgb(yuyv: &[u8], rgb: &mut [u8]) {
    assert_eq!(yuyv.len(), YUYV_LEN);
    assert_eq!(rgb.len(), FRAME_W * FRAME_H * 3);
    let plane = FRAME_W * FRAME_H;
    let (r_plane, rest) = rgb.split_at_mut(plane);
    let (g_plane, b_plane) = rest.split_at_mut(plane);
    for y in 0..FRAME_H {
        for x in 0..FRAME_W / 2 {
            let off = (y * (FRAME_W / 2) + x) * 4;
            let y0 = yuyv[off] as i32;
            let u = yuyv[off + 1] as i32 - 128;
            let y1 = yuyv[off + 2] as i32;
            let v = yuyv[off + 3] as i32 - 128;
            let c0 = y0 - 16;
            let c1 = y1 - 16;
            let dst0 = y * FRAME_W + x * 2;
            let dst1 = dst0 + 1;
            r_plane[dst0] = ((298 * c0 + 409 * v + 128) >> 8).clamp(0, 255) as u8;
            g_plane[dst0] = ((298 * c0 - 100 * u - 208 * v + 128) >> 8).clamp(0, 255) as u8;
            b_plane[dst0] = ((298 * c0 + 516 * u + 128) >> 8).clamp(0, 255) as u8;
            r_plane[dst1] = ((298 * c1 + 409 * v + 128) >> 8).clamp(0, 255) as u8;
            g_plane[dst1] = ((298 * c1 - 100 * u - 208 * v + 128) >> 8).clamp(0, 255) as u8;
            b_plane[dst1] = ((298 * c1 + 516 * u + 128) >> 8).clamp(0, 255) as u8;
        }
    }
}

/// YUYV422 → NV12 的 Y 平面：每对像素取亮度，按 `stride` 逐行写。
///
/// 硬件编码器的输入帧（[`crate::hwjpeg::HwJpeg`]）两个平面不能同时可变借用，
/// 所以 Y / UV 分开写。
pub fn yuyv422_to_nv12_y(yuyv: &[u8], dst: &mut [u8], stride: usize) {
    let src_stride = FRAME_W * 2;
    for y in 0..FRAME_H {
        let src = &yuyv[y * src_stride..y * src_stride + src_stride];
        let dst = &mut dst[y * stride..y * stride + FRAME_W];
        for x in 0..FRAME_W / 2 {
            dst[x * 2] = src[x * 4];
            dst[x * 2 + 1] = src[x * 4 + 2];
        }
    }
}

/// YUYV422 → NV12 的交织 UV 平面：2x2 块取平均，按 `stride` 逐行写。
pub fn yuyv422_to_nv12_uv(yuyv: &[u8], dst: &mut [u8], stride: usize) {
    let src_stride = FRAME_W * 2;
    for y in (0..FRAME_H).step_by(2) {
        let r0 = &yuyv[y * src_stride..y * src_stride + src_stride];
        let r1 = &yuyv[(y + 1) * src_stride..(y + 1) * src_stride + src_stride];
        let dst = &mut dst[(y / 2) * stride..(y / 2) * stride + FRAME_W];
        for x in 0..FRAME_W / 2 {
            let u = (u16::from(r0[x * 4 + 1]) + u16::from(r1[x * 4 + 1])) / 2;
            let v = (u16::from(r0[x * 4 + 3]) + u16::from(r1[x * 4 + 3])) / 2;
            dst[x * 2] = u as u8;
            dst[x * 2 + 1] = v as u8;
        }
    }
}

/// 模型输入预处理器：YUYV422 → RGB 平面 CHW，内部缓冲跨帧复用。
pub struct Preprocessor {
    pub target_w: usize,
    pub target_h: usize,
    buf: Vec<u8>,
}

impl Preprocessor {
    pub fn new(target_w: usize, target_h: usize) -> Self {
        assert_eq!(
            (target_w, target_h),
            (FRAME_W, FRAME_H),
            "仅支持 {FRAME_W}x{FRAME_H} 直通"
        );
        Self {
            target_w,
            target_h,
            buf: vec![0u8; FRAME_W * FRAME_H * 3],
        }
    }

    /// 处理一帧 YUYV422；返回内部 RGB 平面缓冲（下一帧会被覆盖）。
    pub fn process_yuyv(&mut self, yuyv: &[u8], src_w: usize, src_h: usize) -> &[u8] {
        assert_eq!((src_w, src_h), (FRAME_W, FRAME_H));
        yuyv422_to_rgb(yuyv, &mut self.buf);
        &self.buf
    }
}

/// 预览编码后端当前需要的输入格式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewInput {
    /// 相机原始 YUYV422（硬件 VENC：内部转 NV12 后编码；数据直接来自相机缓冲）。
    Yuyv,
    /// RGB 平面 CHW（软件编码；数据来自 [`Preprocessor`] 缓冲）。
    RgbPlanar,
}

/// 网页预览的 JPEG 编码器：硬件 VENC 优先，失败自动降级软件编码。
///
/// 输入要求会随降级变化：构造后与每次 [`PreviewEncoder::encode`] 之后都用
/// [`PreviewEncoder::input_format`] 查询，采集线程据此决定拷贝哪种数据。
pub struct PreviewEncoder {
    /// 硬件 VENC（`None` = 不可用/已降级）。
    ///
    /// `HwJpeg` 与线程绑定，所以整个编码器必须在同一个线程里创建和使用。
    hw: Option<HwJpeg>,
    input: PreviewInput,
    /// 软件编码的降采样倍数（1 = 原尺寸）与质量。
    scale: usize,
    quality: u8,
    /// 软件编码中间缓冲（紧凑 RGB / JPEG），跨帧复用。
    packed: Vec<u8>,
    jpeg: Vec<u8>,
}

impl PreviewEncoder {
    /// 打开硬件 VENC；没有厂商库/内核不支持时直接选软件编码。
    pub fn new(scale: usize, quality: u8) -> Self {
        let hw = match HwJpeg::new(FRAME_W as u32, FRAME_H as u32, quality) {
            Ok(enc) => {
                info!(
                    "预览编码：硬件 VENC（输入像素格式 {}，qfactor {}）",
                    enc.input_format(),
                    quality
                );
                Some(enc)
            }
            Err(e) => {
                warn!("硬件编码不可用（{e}）；改用纯 Rust 编码");
                None
            }
        };
        let input = if hw.is_some() {
            PreviewInput::Yuyv
        } else {
            PreviewInput::RgbPlanar
        };
        Self {
            hw,
            input,
            scale: scale.max(1),
            quality,
            packed: Vec::new(),
            jpeg: Vec::new(),
        }
    }

    /// 当前需要的输入格式（硬件 = [`PreviewInput::Yuyv`]，软件 = [`PreviewInput::RgbPlanar`]）。
    pub fn input_format(&self) -> PreviewInput {
        self.input
    }

    /// 当前后端：`"hw"` / `"sw"`（状态展示用）。
    pub fn backend(&self) -> &'static str {
        if self.hw.is_some() { "hw" } else { "sw" }
    }

    /// 编码一帧；`data` 必须是 [`PreviewEncoder::input_format`] 对应的数据。
    ///
    /// 硬件中途失败会记录日志并切到软件编码（下一帧起要 RGB 平面），
    /// 本次返回错误（该帧没有 JPEG）。
    pub fn encode(&mut self, data: &[u8]) -> io::Result<Arc<[u8]>> {
        if self.hw.is_some() {
            let result = self.hw.as_mut().unwrap().encode(data);
            return match result {
                Ok(jpeg) => Ok(jpeg),
                Err(e) => {
                    warn!("硬件编码失败（{e}）；降级纯 Rust 编码");
                    self.hw = None;
                    self.input = PreviewInput::RgbPlanar;
                    Err(e)
                }
            };
        }
        match self.encode_software(data) {
            Ok(jpeg) => Ok(jpeg),
            Err(e) => {
                warn!("预览编码失败：{e}");
                Err(e)
            }
        }
    }

    /// 软件编码：RGB 平面 CHW → JPEG（可整数倍降采样）。
    fn encode_software(&mut self, planar: &[u8]) -> io::Result<Arc<[u8]>> {
        encode_jpeg(
            planar,
            self.scale,
            self.quality,
            &mut self.packed,
            &mut self.jpeg,
        )?;
        Ok(Arc::from(self.jpeg.as_slice()))
    }
}

/// RGB 平面 CHW → 打包 RGB（可整数倍降采样）→ baseline JPEG（写入 `out`）。
fn encode_jpeg(
    planar: &[u8],
    scale: usize,
    quality: u8,
    packed: &mut Vec<u8>,
    out: &mut Vec<u8>,
) -> io::Result<()> {
    let scale = scale.max(1);
    let plane = FRAME_W * FRAME_H;
    if planar.len() < plane * 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("RGB 缓冲长度 {} 小于 {}", planar.len(), plane * 3),
        ));
    }
    let (r, rest) = planar.split_at(plane);
    let (g, b) = rest.split_at(plane);
    let (ow, oh) = (FRAME_W / scale, FRAME_H / scale);

    packed.clear();
    packed.reserve(ow * oh * 3);
    if scale == 1 {
        for i in 0..plane {
            packed.push(r[i]);
            packed.push(g[i]);
            packed.push(b[i]);
        }
    } else {
        let n = (scale * scale) as u32;
        for oy in 0..oh {
            for ox in 0..ow {
                let mut sum = [0u32; 3];
                for dy in 0..scale {
                    let row = (oy * scale + dy) * FRAME_W + ox * scale;
                    for dx in 0..scale {
                        let i = row + dx;
                        sum[0] += u32::from(r[i]);
                        sum[1] += u32::from(g[i]);
                        sum[2] += u32::from(b[i]);
                    }
                }
                packed.push((sum[0] / n) as u8);
                packed.push((sum[1] / n) as u8);
                packed.push((sum[2] / n) as u8);
            }
        }
    }

    out.clear();
    Encoder::new(&mut *out, quality)
        .encode(packed, ow as u16, oh as u16, ColorType::Rgb)
        .map_err(|e| io::Error::other(format!("JPEG 编码失败: {e}")))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造「YUYV：U=10、V=200、Y=列号」的可区分测试图。
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

    /// 构造「左半红、右半蓝」的 RGB 平面缓冲（模型输入顺序 R/G/B 三平面）。
    fn red_blue_planar() -> Vec<u8> {
        let plane = FRAME_W * FRAME_H;
        let mut buf = vec![0u8; plane * 3];
        let (r, gb) = buf.split_at_mut(plane);
        let (g, b) = gb.split_at_mut(plane);
        for y in 0..FRAME_H {
            for x in 0..FRAME_W {
                let i = y * FRAME_W + x;
                if x < FRAME_W / 2 {
                    r[i] = 255;
                } else {
                    b[i] = 255;
                }
                g[i] = 0;
            }
        }
        buf
    }

    #[test]
    fn test_yuyv422_to_rgb() {
        let yuyv = vec![128u8; YUYV_LEN];
        let mut out = vec![0u8; FRAME_W * FRAME_H * 3];
        yuyv422_to_rgb(&yuyv, &mut out);
        // CHW 验证：R/G/B 三平面各 640*480
        assert_eq!(out.len(), 3 * FRAME_W * FRAME_H);
    }

    #[test]
    fn yuyv422_to_nv12_y_takes_luma_and_respects_stride() {
        let src = yuyv_pattern();
        let stride = FRAME_W + 64; // 模拟驱动 stride padding
        let mut dst = vec![0xAAu8; stride * FRAME_H];
        yuyv422_to_nv12_y(&src, &mut dst, stride);

        for y in [0usize, 1, 239, 479] {
            let row = &dst[y * stride..y * stride + FRAME_W];
            for (i, &v) in row.iter().enumerate() {
                assert_eq!(v as usize, i % 256, "第 {y} 行第 {i} 列");
            }
        }
        // padding 不被写
        assert_eq!(dst[FRAME_W], 0xAA);
        assert_eq!(dst[stride * FRAME_H - 1], 0xAA);
    }

    #[test]
    fn yuyv422_to_nv12_uv_averages_2x2_blocks() {
        let mut src = yuyv_pattern();
        // 让 (0,0)-(1,1) 这 2x2 块取不同的 UV：行 0 的 U=10/V=200，行 1 的 U=30/V=100
        for pair in 0..FRAME_W / 2 {
            src[pair * 4 + 1] = 10;
            src[pair * 4 + 3] = 200;
            let r1 = FRAME_W * 2 + pair * 4;
            src[r1 + 1] = 30;
            src[r1 + 3] = 100;
        }
        let stride = FRAME_W + 64;
        let mut dst = vec![0xAAu8; stride * (FRAME_H / 2)];
        yuyv422_to_nv12_uv(&src, &mut dst, stride);

        assert_eq!(dst[0], 20, "U 取两行平均");
        assert_eq!(dst[1], 150, "V 取两行平均");
        assert_eq!(dst[2], 20);
        assert_eq!(dst[3], 150);
        // 第二行 UV（源的第 2/3 行仍是 10/200）
        assert_eq!(dst[stride], 10);
        assert_eq!(dst[stride + 1], 200);
        // padding 不被写
        assert_eq!(dst[FRAME_W], 0xAA);
        assert_eq!(dst[stride * (FRAME_H / 2) - 1], 0xAA);
    }

    #[test]
    fn encode_jpeg_keeps_rgb_order_and_size() {
        let planar = red_blue_planar();
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        encode_jpeg(&planar, 1, 90, &mut packed, &mut jpeg).unwrap();

        assert_eq!(&jpeg[..2], &[0xFF, 0xD8], "JPEG SOI 缺失");
        assert_eq!(&jpeg[jpeg.len() - 2..], &[0xFF, 0xD9], "JPEG EOI 缺失");

        let mut decoder = jpeg_decoder::Decoder::new(&jpeg[..]);
        let pixels = decoder.decode().unwrap();
        let info = decoder.info().unwrap();
        assert_eq!((info.width, info.height), (FRAME_W as u16, FRAME_H as u16));

        // 采样左右两侧（JPEG 有损 + 色度下采样，只看通道主导关系）
        let px = |x: usize, y: usize| {
            let i = (y * FRAME_W + x) * 3;
            (pixels[i], pixels[i + 1], pixels[i + 2])
        };
        let (r, g, b) = px(40, 40);
        assert!(
            r > 200 && g < 80 && b < 80,
            "左侧应是红色，实际 {r},{g},{b}"
        );
        let (r, g, b) = px(FRAME_W - 40, FRAME_H - 40);
        assert!(
            b > 200 && r < 80 && g < 80,
            "右侧应是蓝色，实际 {r},{g},{b}"
        );
    }

    #[test]
    fn encode_jpeg_downscales_by_integer_factor() {
        let planar = red_blue_planar();
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        encode_jpeg(&planar, 2, 80, &mut packed, &mut jpeg).unwrap();
        let mut decoder = jpeg_decoder::Decoder::new(&jpeg[..]);
        decoder.decode().unwrap();
        let info = decoder.info().unwrap();
        assert_eq!(
            (info.width, info.height),
            ((FRAME_W / 2) as u16, (FRAME_H / 2) as u16)
        );
        assert_eq!(packed.len(), (FRAME_W / 2) * (FRAME_H / 2) * 3);
    }

    #[test]
    fn encode_jpeg_rejects_short_buffer() {
        let mut packed = Vec::new();
        let mut jpeg = Vec::new();
        assert!(encode_jpeg(&[0u8; 16], 1, 80, &mut packed, &mut jpeg).is_err());
    }

    /// 真机降级检查（需要厂商库）：进程内 VENC 会话只能初始化一次，
    /// 第二个编码器拿不到会话，应自动降到软件编码并出图。
    #[test]
    #[ignore = "需要 SG2002 硬件（VENC + 厂商库）"]
    fn preview_encoder_falls_back_to_software() {
        let hw = PreviewEncoder::new(1, 80);
        assert_eq!(hw.backend(), "hw", "第一个编码器应拿到硬件 VENC");
        assert_eq!(hw.input_format(), PreviewInput::Yuyv);

        let mut sw = PreviewEncoder::new(1, 80);
        assert_eq!(sw.backend(), "sw", "第二个编码器应降级软件编码");
        assert_eq!(sw.input_format(), PreviewInput::RgbPlanar);

        // 梯度图，避免全黑压缩后过小
        let mut planar = vec![0u8; FRAME_W * FRAME_H * 3];
        for (i, b) in planar.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        let jpeg = sw.encode(&planar).unwrap();
        assert_eq!(&jpeg[..2], &[0xFF, 0xD8], "JPEG SOI 缺失");
        assert_eq!(&jpeg[jpeg.len() - 2..], &[0xFF, 0xD9], "JPEG EOI 缺失");
    }
}
