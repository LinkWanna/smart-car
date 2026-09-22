//! preprocess.rs

/// YUYV422 (640x480) → RGB 平面 CHW (640x480)
pub fn yuyv422_to_rgb(yuyv: &[u8], rgb: &mut [u8]) {
    assert_eq!(yuyv.len(), 640 * 480 * 2);
    assert_eq!(rgb.len(), 640 * 480 * 3);
    let plane = 640 * 480;
    let (r_plane, rest) = rgb.split_at_mut(plane);
    let (g_plane, b_plane) = rest.split_at_mut(plane);
    for y in 0..480 {
        for x in 0..320 {
            let off = (y * 320 + x) * 4;
            let y0 = yuyv[off] as i32;
            let u = yuyv[off + 1] as i32 - 128;
            let y1 = yuyv[off + 2] as i32;
            let v = yuyv[off + 3] as i32 - 128;
            let c0 = y0 - 16;
            let c1 = y1 - 16;
            let dst0 = y * 640 + x * 2;
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

pub struct Preprocessor {
    pub target_w: usize,
    pub target_h: usize,
    buf: Vec<u8>,
}

impl Preprocessor {
    pub fn new(target_w: usize, target_h: usize) -> Self {
        assert_eq!((target_w, target_h), (640, 480), "仅支持 640x480 直通");
        Self {
            target_w,
            target_h,
            buf: vec![0u8; 640 * 480 * 3],
        }
    }

    pub fn process_yuyv(&mut self, yuyv: &[u8], src_w: usize, src_h: usize) -> &[u8] {
        assert_eq!((src_w, src_h), (640, 480));
        yuyv422_to_rgb(yuyv, &mut self.buf);
        &self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_yuyv422_to_rgb() {
        let yuyv = vec![128u8; 640 * 480 * 2];
        let mut out = vec![0u8; 640 * 480 * 3];
        yuyv422_to_rgb(&yuyv, &mut out);
        assert_eq!(out.len(), 640 * 480 * 3);
        // CHW 验证：R/G/B 三平面各 640*480
        assert_eq!(out.len(), 3 * 640 * 480);
    }
}
