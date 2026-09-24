//! 相机 YUYV422 帧的 CPU 参考转换。
//!
//! 业务链路（VPSS 管线）的像素转换由硬件 CSC 完成，不经过这里；本模块只保留
//! [`yuyv422_to_rgb`]（`vpss_probe` 用它做通道顺序/量程的 CPU 参考对比）与帧
//! 尺寸常量。相机采集在 [`crate::vision::camera`]；帧尺寸固定 [`FRAME_W`]x[`FRAME_H`]，
//! 与相机协商格式、模型输入、预览编码一致。

/// 帧宽（相机 / 模型输入 / 预览编码一致）。
pub const FRAME_W: usize = 640;
/// 帧高。
pub const FRAME_H: usize = 480;
/// 一帧 YUYV422 的字节数（2 字节/像素）。
pub const YUYV_LEN: usize = FRAME_W * 2 * FRAME_H;

/// YUYV422 (640x480) → RGB 平面 CHW (640x480)。
///
/// BT.601 limited range（Y-16、UV-128）→ full range，与 VPSS 组 CSC 的
/// `set_yuv601_limited_to_full` 对应。
pub fn yuyv422_to_rgb(yuyv: &[u8], rgb: &mut [u8]) {
    assert_eq!(yuyv.len(), YUYV_LEN);
    let plane = FRAME_W * FRAME_H;
    assert_eq!(rgb.len(), plane * 3);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yuyv422_to_rgb() {
        let yuyv = vec![128u8; YUYV_LEN];
        let mut out = vec![0u8; FRAME_W * FRAME_H * 3];
        yuyv422_to_rgb(&yuyv, &mut out);
        // CHW 验证：R/G/B 三平面各 640*480
        assert_eq!(out.len(), 3 * FRAME_W * FRAME_H);
    }
}
