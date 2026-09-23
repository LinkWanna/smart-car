//! yolo.rs — YOLOv8 单类检测：TPU 零拷贝推理 + NMS。
//!
//! 只保留零拷贝路径：输入张量直接指向 VPSS chn0 或 CPU 路径 VB 帧的物理地址
//! （`Model::forward_physical`），CPU 不搬运像素；模型、阈值、标签、计时都在
//! [`Yolo`] 一个结构里（原 `tpu.rs` 的 `TpuEngine` / `TpuInference` 两层包装已去掉）。
//!
//! 输出按 YOLOv8 布局解析：`[1, 5, N]`（cx、cy、w、h、conf 各一段），
//! N 由输出字节数推出（480x640 输入是 6300）。

use std::io;
use std::time::Instant;

use cviruntime_rs::Model;

/// 单帧最多输出的检测框数。
const MAX_DETECTIONS: usize = 20;
/// 置信度过滤后的候选框上限（栈上固定数组，NMS 过程不分配）。
const MAX_CANDIDATES: usize = 256;

#[derive(Debug, Clone)]
pub struct Detection {
    pub label: String,
    pub confidence: f32,
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}

impl Detection {
    pub fn center_x(&self) -> f32 {
        (self.x1 + self.x2) * 0.5
    }
    pub fn center_y(&self) -> f32 {
        (self.y1 + self.y2) * 0.5
    }
    pub fn width(&self) -> f32 {
        self.x2 - self.x1
    }
    pub fn height(&self) -> f32 {
        self.y2 - self.y1
    }
    pub fn area(&self) -> f32 {
        self.width() * self.height()
    }
}

pub struct Yolo {
    model: Model,        // 模型
    conf_thresh: f32,    // 置信度阈值
    iou_thresh: f32,     // IOU 阈值
    labels: Vec<String>, // 标签列表
    last_tpu_ms: f64,    // 上次推理耗时（毫秒）
    last_nms_ms: f64,    // 上次 NMS 耗时（毫秒）
}

impl Yolo {
    /// 加载模型
    pub fn from_file(
        path: &str,
        conf_thresh: f32,
        iou_thresh: f32,
        labels: Vec<String>,
    ) -> io::Result<Self> {
        let model =
            Model::from_file(path).map_err(|e| io::Error::other(format!("模型加载失败: {e}")))?;
        Ok(Self {
            model,
            conf_thresh,
            iou_thresh,
            labels,
            last_tpu_ms: 0.0,
            last_nms_ms: 0.0,
        })
    }

    /// yolo 输入张量形状（如 `[1, 3, 480, 640]`）
    pub fn input_shape(&self) -> &[i32] {
        &self.model.inputs[0].shape
    }

    /// yolo 的输入张量字节数
    pub fn input_bytes(&self) -> usize {
        self.model.inputs[0].bytes
    }

    /// 零拷贝推理：输入张量直接指向 `paddr` 处的物理内存（如 VPSS 通道输出帧）
    pub fn infer(&mut self, paddr: u64) -> io::Result<Vec<Detection>> {
        let t_tpu = Instant::now();
        self.model
            .forward_physical(paddr)
            .map_err(|e| io::Error::other(format!("TPU 前向失败（paddr={paddr:#x}）: {e}")))?;
        self.last_tpu_ms = t_tpu.elapsed().as_secs_f64() * 1000.0;

        let t_nms = Instant::now();
        let dets = nms(
            self.model.outputs[0].as_f32_slice(),
            self.conf_thresh,
            self.iou_thresh,
            &self.labels,
        );
        self.last_nms_ms = t_nms.elapsed().as_secs_f64() * 1000.0;
        Ok(dets)
    }

    pub fn last_timing(&self) -> (f64, f64) {
        (self.last_tpu_ms, self.last_nms_ms)
    }
}

/// YOLOv8 原始输出（`[1, 5, N]`：cx、cy、w、h、conf 各一段）→ 置信度过滤 + NMS。
fn nms(raw: &[f32], conf_thresh: f32, iou_thresh: f32, labels: &[String]) -> Vec<Detection> {
    let n = raw.len() / 5; // 每个锚点 5 个浮点
    if n == 0 {
        return Vec::new();
    }

    let mut cand_x1 = [0f32; MAX_CANDIDATES];
    let mut cand_y1 = [0f32; MAX_CANDIDATES];
    let mut cand_x2 = [0f32; MAX_CANDIDATES];
    let mut cand_y2 = [0f32; MAX_CANDIDATES];
    let mut cand_conf = [0f32; MAX_CANDIDATES];
    let mut order = [0usize; MAX_CANDIDATES];
    let mut suppressed = [false; MAX_CANDIDATES];

    // 通道优先布局：前 n 个是 cx，接着依次 cy / w / h / conf。
    let cx = &raw[..n];
    let cy = &raw[n..2 * n];
    let w = &raw[2 * n..3 * n];
    let h = &raw[3 * n..4 * n];
    let conf = &raw[4 * n..5 * n];

    // 1) 置信度过滤：中心宽高 → xyxy，太小的框直接丢掉。
    let mut k = 0;
    for i in 0..n {
        if k >= MAX_CANDIDATES {
            break;
        }
        if conf[i] < conf_thresh {
            continue;
        }
        let (x1, y1) = (cx[i] - w[i] * 0.5, cy[i] - h[i] * 0.5);
        let (x2, y2) = (cx[i] + w[i] * 0.5, cy[i] + h[i] * 0.5);
        if x2 - x1 < 2.0 || y2 - y1 < 2.0 {
            continue;
        }
        cand_x1[k] = x1.max(0.0);
        cand_y1[k] = y1.max(0.0);
        cand_x2[k] = x2;
        cand_y2[k] = y2;
        cand_conf[k] = conf[i];
        order[k] = k;
        k += 1;
    }
    if k == 0 {
        return Vec::new();
    }

    // 2) 候选按置信度降序（插入排序：k 很小）。
    for i in 1..k {
        let key = order[i];
        let key_conf = cand_conf[key];
        let mut j = i as i32 - 1;
        while j >= 0 && cand_conf[order[j as usize]] < key_conf {
            order[(j + 1) as usize] = order[j as usize];
            j -= 1;
        }
        order[(j + 1) as usize] = key;
    }

    // 3) 贪心 NMS：按置信度从高到低保留，压掉 IoU 超阈的后续框。
    let label = labels.first().map(String::as_str).unwrap_or("object");
    let mut out = Vec::with_capacity(MAX_DETECTIONS.min(k));
    for i in 0..k {
        if out.len() >= MAX_DETECTIONS {
            break;
        }
        let a = order[i];
        if suppressed[a] {
            continue;
        }
        let (ax1, ay1) = (cand_x1[a], cand_y1[a]);
        let (ax2, ay2) = (cand_x2[a], cand_y2[a]);
        let area_a = (ax2 - ax1) * (ay2 - ay1);
        out.push(Detection {
            label: label.to_string(),
            confidence: cand_conf[a],
            x1: ax1,
            y1: ay1,
            x2: ax2,
            y2: ay2,
        });
        for &b in &order[i + 1..k] {
            if suppressed[b] {
                continue;
            }
            let (bx1, by1) = (cand_x1[b], cand_y1[b]);
            let (bx2, by2) = (cand_x2[b], cand_y2[b]);
            let inter_w = ax2.min(bx2) - ax1.max(bx1);
            let inter_h = ay2.min(by2) - ay1.max(by1);
            if inter_w <= 0.0 || inter_h <= 0.0 {
                continue;
            }
            let inter = inter_w * inter_h;
            let area_b = (bx2 - bx1) * (by2 - by1);
            if inter / (area_a + area_b - inter + 1e-6) > iou_thresh {
                suppressed[b] = true;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 按 YOLOv8 输出布局（cx/cy/w/h/conf 各一段）拼原始输出。
    fn raw_output(boxes: &[(f32, f32, f32, f32, f32)]) -> Vec<f32> {
        let n = boxes.len();
        let mut raw = vec![0f32; 5 * n];
        for (i, &(cx, cy, w, h, conf)) in boxes.iter().enumerate() {
            raw[i] = cx;
            raw[n + i] = cy;
            raw[2 * n + i] = w;
            raw[3 * n + i] = h;
            raw[4 * n + i] = conf;
        }
        raw
    }

    #[test]
    fn single_box_decodes() {
        let raw = raw_output(&[(100.0, 200.0, 40.0, 20.0, 0.9)]);
        let dets = nms(&raw, 0.5, 0.45, &["tennis_ball".to_string()]);
        assert_eq!(dets.len(), 1);
        assert_eq!(dets[0].label, "tennis_ball");
        assert!((dets[0].confidence - 0.9).abs() < 1e-6);
        assert!((dets[0].x1 - 80.0).abs() < 1e-6);
        assert!((dets[0].y1 - 190.0).abs() < 1e-6);
        assert!((dets[0].x2 - 120.0).abs() < 1e-6);
        assert!((dets[0].y2 - 210.0).abs() < 1e-6);
    }

    #[test]
    fn low_confidence_and_tiny_boxes_are_dropped() {
        let raw = raw_output(&[
            (100.0, 100.0, 40.0, 40.0, 0.4), // 置信度不足
            (100.0, 100.0, 1.0, 1.0, 0.9),   // 框太小
        ]);
        assert!(nms(&raw, 0.5, 0.45, &[]).is_empty());
        assert!(nms(&[], 0.5, 0.45, &[]).is_empty());
    }

    #[test]
    fn overlapping_boxes_keep_the_confident_one() {
        let raw = raw_output(&[
            (100.0, 100.0, 40.0, 40.0, 0.9),
            (104.0, 102.0, 40.0, 40.0, 0.8), // 与上面 IoU≈0.75
        ]);
        let dets = nms(&raw, 0.5, 0.45, &[]);
        assert_eq!(dets.len(), 1);
        assert!((dets[0].confidence - 0.9).abs() < 1e-6);
    }

    #[test]
    fn separate_boxes_survive_in_confidence_order() {
        let raw = raw_output(&[
            (50.0, 50.0, 20.0, 20.0, 0.7),
            (300.0, 300.0, 20.0, 20.0, 0.9),
        ]);
        let dets = nms(&raw, 0.5, 0.45, &[]);
        assert_eq!(dets.len(), 2);
        assert!((dets[0].confidence - 0.9).abs() < 1e-6);
        assert!((dets[1].confidence - 0.7).abs() < 1e-6);
    }

    #[test]
    fn boxes_are_clamped_to_frame_and_capped() {
        let raw = raw_output(&[(-10.0, -10.0, 40.0, 40.0, 0.9)]);
        let dets = nms(&raw, 0.5, 0.45, &[]);
        assert_eq!(dets.len(), 1);
        assert_eq!((dets[0].x1, dets[0].y1), (0.0, 0.0));

        // 相互不重叠的框超过 MAX_DETECTIONS 时只输出前 MAX_DETECTIONS 个
        let many: Vec<(f32, f32, f32, f32, f32)> = (0..MAX_DETECTIONS + 5)
            .map(|i| (i as f32 * 20.0, 10.0, 10.0, 10.0, 0.9))
            .collect();
        assert_eq!(
            nms(&raw_output(&many), 0.5, 0.45, &[]).len(),
            MAX_DETECTIONS
        );
    }

    /// 真机校验（需要 SG2002 + 厂商库 + 模型）：CPU 路径往 VB 帧里写 RGB 平面，
    /// 零拷贝（物理地址）与 memcpy 两条喂法必须逐字节一致。
    ///
    /// `cargo test --release --target riscv64gc-unknown-linux-musl -- --ignored --nocapture`
    #[test]
    #[ignore = "需要 SG2002 硬件（TPU + 厂商库）"]
    fn zero_copy_input_matches_memcpy() {
        use cvimpi_rs::ffi;
        use cvimpi_rs::sys::{Sys, VbPoolConfig};
        use cvimpi_rs::venc_input_layout;

        let path = std::env::var("SMARTCAR_MODEL")
            .unwrap_or_else(|_| "/root/yolov8n_tennis_v3.cvimodel".to_string());
        let (w, h) = (640u32, 480u32);
        let fmt = ffi::PIXEL_FORMAT_RGB_888_PLANAR;
        let layout = venc_input_layout(w, h, fmt).expect("RGB 平面布局");
        let sys = Sys::init(&[VbPoolConfig::new(layout.vb_size, 4).with_name("yolo-test")])
            .expect("MMF 会话初始化失败");
        let mut frame = sys
            .alloc_frame_cached(w, h, fmt)
            .expect("RGB 输入帧分配失败");

        // 可区分的测试图：左半红、右半蓝
        let plane = (w * h) as usize;
        let width = w as usize;
        let mut rgb = vec![0u8; plane * 3];
        for i in 0..plane {
            if i % width < width / 2 {
                rgb[i] = 255;
            } else {
                rgb[2 * plane + i] = 255;
            }
        }
        {
            let [r, g, b] = frame.planes_mut();
            r.copy_from_slice(&rgb[..plane]);
            g.copy_from_slice(&rgb[plane..2 * plane]);
            b.copy_from_slice(&rgb[2 * plane..]);
        }
        frame.flush().expect("flush");

        let model = Model::from_file(&path).expect("模型加载失败");
        // memcpy 路径先跑：`forward_physical` 会释放运行时分配的输入内存
        let by_memcpy = model.forward(&rgb).expect("forward").to_vec();
        let by_paddr = model
            .forward_physical(frame.phy_addr(0))
            .expect("forward_physical")
            .to_vec();
        let max_diff = by_memcpy
            .iter()
            .zip(&by_paddr)
            .map(|(a, b)| a.abs_diff(*b))
            .max()
            .unwrap_or(0);
        assert_eq!(
            max_diff, 0,
            "零拷贝与 memcpy 输出不一致（帧布局/刷写有问题）"
        );
    }
}
