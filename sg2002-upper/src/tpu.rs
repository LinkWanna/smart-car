//! tpu.rs — 上层推理（NMS）+ 对 cviruntime-rs 的薄封装

use std::io;

use cviruntime_rs::Model;

#[derive(Debug, Clone)]
struct NmsDet {
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    conf: f32,
}

fn nms_decode(
    raw: &[f32],
    num_anchors: usize,
    conf_thresh: f32,
    iou_thresh: f32,
    max_det: usize,
) -> Vec<NmsDet> {
    if raw.is_empty() || num_anchors == 0 || max_det == 0 {
        return Vec::new();
    }
    assert!(raw.len() >= 5 * num_anchors);
    const MAX_CANDIDATES: usize = 256;
    let mut cand_x1 = [0f32; MAX_CANDIDATES];
    let mut cand_y1 = [0f32; MAX_CANDIDATES];
    let mut cand_x2 = [0f32; MAX_CANDIDATES];
    let mut cand_y2 = [0f32; MAX_CANDIDATES];
    let mut cand_conf = [0f32; MAX_CANDIDATES];
    let mut cand_order = [0usize; MAX_CANDIDATES];
    let mut suppressed = [false; MAX_CANDIDATES];
    let mut k = 0usize;
    let cx_arr = &raw[..num_anchors];
    let cy_arr = &raw[num_anchors..2 * num_anchors];
    let w_arr = &raw[2 * num_anchors..3 * num_anchors];
    let h_arr = &raw[3 * num_anchors..4 * num_anchors];
    let conf_arr = &raw[4 * num_anchors..5 * num_anchors];
    for i in 0..num_anchors {
        if k >= MAX_CANDIDATES {
            break;
        }
        let conf = conf_arr[i];
        if conf < conf_thresh {
            continue;
        }
        let cx = cx_arr[i];
        let cy = cy_arr[i];
        let w = w_arr[i];
        let h = h_arr[i];
        let x1 = cx - w * 0.5;
        let y1 = cy - h * 0.5;
        let x2 = cx + w * 0.5;
        let y2 = cy + h * 0.5;
        if x2 - x1 < 2.0 || y2 - y1 < 2.0 {
            continue;
        }
        cand_x1[k] = x1.max(0.0);
        cand_y1[k] = y1.max(0.0);
        cand_x2[k] = x2;
        cand_y2[k] = y2;
        cand_conf[k] = conf;
        cand_order[k] = k;
        k += 1;
    }
    if k == 0 {
        return Vec::new();
    }
    for i in 1..k {
        let key = cand_order[i];
        let key_conf = cand_conf[key];
        let mut j = i as i32 - 1;
        while j >= 0 && cand_conf[cand_order[j as usize]] < key_conf {
            cand_order[(j + 1) as usize] = cand_order[j as usize];
            j -= 1;
        }
        cand_order[(j + 1) as usize] = key;
    }
    let mut out = Vec::with_capacity(max_det.min(k));
    for i in 0..k {
        if out.len() >= max_det {
            break;
        }
        let idx_i = cand_order[i];
        if suppressed[idx_i] {
            continue;
        }
        let xi1 = cand_x1[idx_i];
        let yi1 = cand_y1[idx_i];
        let xi2 = cand_x2[idx_i];
        let yi2 = cand_y2[idx_i];
        let a_i = (xi2 - xi1) * (yi2 - yi1);
        out.push(NmsDet {
            x1: xi1,
            y1: yi1,
            x2: xi2,
            y2: yi2,
            conf: cand_conf[idx_i],
        });
        for j in i + 1..k {
            let idx_j = cand_order[j];
            if suppressed[idx_j] {
                continue;
            }
            let xj1 = cand_x1[idx_j];
            let yj1 = cand_y1[idx_j];
            let xj2 = cand_x2[idx_j];
            let yj2 = cand_y2[idx_j];
            let inter_x = xi2.min(xj2) - xi1.max(xj1);
            let inter_y = yi2.min(yj2) - yi1.max(yj1);
            if inter_x <= 0.0 || inter_y <= 0.0 {
                continue;
            }
            let inter = inter_x * inter_y;
            let a_j = (xj2 - xj1) * (yj2 - yj1);
            let iou = inter / (a_i + a_j - inter + 1e-6);
            if iou > iou_thresh {
                suppressed[idx_j] = true;
            }
        }
    }
    out
}

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

/// 输出张量 → 检测框（NMS）。
fn decode(
    out_bytes: &[u8],
    n_anchors: usize,
    conf_thresh: f32,
    iou_thresh: f32,
    labels: &[String],
) -> Vec<Detection> {
    let f32_len = out_bytes.len() / 4;
    let raw: &[f32] =
        unsafe { std::slice::from_raw_parts(out_bytes.as_ptr() as *const f32, f32_len) };

    nms_decode(raw, n_anchors, conf_thresh, iou_thresh, 20)
        .into_iter()
        .map(|d| Detection {
            label: labels
                .first()
                .cloned()
                .unwrap_or_else(|| "object".to_string()),
            confidence: d.conf,
            x1: d.x1,
            y1: d.y1,
            x2: d.x2,
            y2: d.y2,
        })
        .collect()
}

pub struct TpuEngine {
    inner: Model,
}

impl TpuEngine {
    /// 加载模型；失败返回 `io::Error`（不 panic，供网页/整合入口降级）。
    pub fn try_new(model_path: &str) -> io::Result<Self> {
        let model = Model::from_file(model_path)
            .map_err(|e| io::Error::other(format!("TPU Model 加载失败: {e}")))?;
        Ok(Self { inner: model })
    }

    pub fn new(model_path: &str) -> Self {
        Self::try_new(model_path).unwrap_or_else(|e| panic!("{e}"))
    }

    /// 前向推理；失败返回 `io::Error`（不 panic）。
    pub fn try_infer(&self, data: &[u8]) -> io::Result<&[u8]> {
        self.inner
            .forward(data)
            .map_err(|e| io::Error::other(format!("Forward 失败: {e}")))
    }

    /// 零拷贝前向：输入张量直接指向 `paddr` 处的物理内存（例如 VPSS 通道输出帧），
    /// CPU 不搬运像素。调用方负责让这块内存在 Forward 期间保持有效
    /// （`vpss::VpssFrame` 持有期间不会被复用）。
    ///
    /// 注意：`CVI_NN_SetTensorPhysicalAddr` 会释放运行时自动分配的输入内存，
    /// 因此**同一个模型**之后不能再用 [`TpuEngine::try_infer`]（memcpy 路径）。
    pub fn try_forward_physical(&self, paddr: u64) -> io::Result<&[u8]> {
        self.inner
            .forward_physical(paddr)
            .map_err(|e| io::Error::other(format!("Forward(paddr={paddr:#x}) 失败: {e}")))
    }

    pub fn infer(&self, data: &[u8]) -> &[u8] {
        self.try_infer(data).unwrap_or_else(|e| panic!("{e}"))
    }

    pub fn in_shape(&self) -> &[i32] {
        &self.inner.inputs[0].shape
    }

    /// 输入张量的字节数（`[1,3,480,640]` u8 = 921600）。
    pub fn input_bytes(&self) -> usize {
        self.inner.inputs[0].bytes
    }

    pub fn out_shape(&self) -> &[i32] {
        &self.inner.outputs[0].shape
    }
}

pub struct TpuInference {
    engine: TpuEngine,
    conf_thresh: f32,
    iou_thresh: f32,
    labels: Vec<String>,
    last_tpu_ms: f64,
    last_nms_ms: f64,
}

impl TpuInference {
    /// 加载模型；失败返回 `io::Error`（供上层降级为「仅预览」）。
    pub fn try_new(
        model_path: &str,
        conf_thresh: f32,
        iou_thresh: f32,
        labels: Vec<String>,
    ) -> io::Result<Self> {
        Ok(Self {
            engine: TpuEngine::try_new(model_path)?,
            conf_thresh,
            iou_thresh,
            labels,
            last_tpu_ms: 0.0,
            last_nms_ms: 0.0,
        })
    }

    pub fn new(model_path: &str, conf_thresh: f32, iou_thresh: f32, labels: Vec<String>) -> Self {
        Self::try_new(model_path, conf_thresh, iou_thresh, labels).unwrap_or_else(|e| panic!("{e}"))
    }

    /// 模型输入张量形状（如 `[1, 3, 480, 640]`）。
    pub fn input_shape(&self) -> &[i32] {
        self.engine.in_shape()
    }

    /// 模型输入张量的字节数（零拷贝路径要求等于 VPSS RGB 帧的紧凑大小）。
    pub fn input_bytes(&self) -> usize {
        self.engine.input_bytes()
    }

    /// 推理；失败返回 `io::Error`（不 panic）。
    pub fn try_infer(&mut self, planar: &[u8]) -> io::Result<Vec<Detection>> {
        let t0 = std::time::Instant::now();
        let out_bytes = self.engine.try_infer(planar)?;
        let tpu_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let n_anchors = self.engine.out_shape()[2] as usize;
        let t1 = std::time::Instant::now();
        let dets = decode(
            out_bytes,
            n_anchors,
            self.conf_thresh,
            self.iou_thresh,
            &self.labels,
        );
        self.last_tpu_ms = tpu_ms;
        self.last_nms_ms = t1.elapsed().as_secs_f64() * 1000.0;
        Ok(dets)
    }

    /// 零拷贝推理：输入直接指向 VPSS 输出帧的物理地址。
    ///
    /// **调用后本实例不能再走 [`TpuInference::try_infer`]**（运行时已释放
    /// 自动分配的输入内存），需要双路径时请用两个模型实例。
    pub fn try_infer_physical(&mut self, paddr: u64) -> io::Result<Vec<Detection>> {
        let t0 = std::time::Instant::now();
        let out_bytes = self.engine.try_forward_physical(paddr)?;
        let tpu_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let n_anchors = self.engine.out_shape()[2] as usize;
        let t1 = std::time::Instant::now();
        let dets = decode(
            out_bytes,
            n_anchors,
            self.conf_thresh,
            self.iou_thresh,
            &self.labels,
        );
        self.last_tpu_ms = tpu_ms;
        self.last_nms_ms = t1.elapsed().as_secs_f64() * 1000.0;
        Ok(dets)
    }

    pub fn infer(&mut self, planar: &[u8]) -> Vec<Detection> {
        self.try_infer(planar).unwrap_or_else(|e| panic!("{}", e))
    }

    pub fn last_timing(&self) -> (f64, f64) {
        (self.last_tpu_ms, self.last_nms_ms)
    }
}
