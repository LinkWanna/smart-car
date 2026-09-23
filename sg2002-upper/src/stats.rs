//! stats.rs — 管线统计，移植自 python/src/stats.py
//! 单例语义在 Rust 中由 main 持有单一实例实现

use std::collections::HashMap;
use std::time::Instant;

use log::info;

pub struct Stats {
    t0: Instant,
    pub fid: usize,
    fps_window: Vec<f64>,
    capture_ms: Vec<f64>,
    pre_ms: Vec<f64>,
    tpu_ms: Vec<f64>,
    nms_ms: Vec<f64>,
    position_ms: Vec<f64>,
    control_ms: Vec<f64>,
    total_ms: Vec<f64>,
    det_frames: usize,
    no_target_frames: usize,
    counters: HashMap<String, usize>,
    extra: HashMap<String, String>,
    printed: bool,
}

impl Stats {
    pub fn new() -> Self {
        Self {
            t0: Instant::now(),
            fid: 0,
            fps_window: Vec::new(),
            capture_ms: Vec::new(),
            pre_ms: Vec::new(),
            tpu_ms: Vec::new(),
            nms_ms: Vec::new(),
            position_ms: Vec::new(),
            control_ms: Vec::new(),
            total_ms: Vec::new(),
            det_frames: 0,
            no_target_frames: 0,
            counters: HashMap::new(),
            extra: HashMap::new(),
            printed: false,
        }
    }

    fn push_window(v: &mut Vec<f64>, val: f64) {
        v.push(val);
        if v.len() > 30 {
            v.remove(0);
        }
    }

    pub fn record_capture(&mut self, ms: f64) {
        Self::push_window(&mut self.capture_ms, ms);
    }
    pub fn record_preprocess(&mut self, ms: f64) {
        Self::push_window(&mut self.pre_ms, ms);
    }
    pub fn record_inference(&mut self, tpu_ms: f64, nms_ms: f64) {
        Self::push_window(&mut self.tpu_ms, tpu_ms);
        Self::push_window(&mut self.nms_ms, nms_ms);
    }
    pub fn record_position(&mut self, ms: f64) {
        Self::push_window(&mut self.position_ms, ms);
    }
    pub fn record_control(&mut self, ms: f64) {
        Self::push_window(&mut self.control_ms, ms);
    }
    pub fn record_frame(&mut self, total_ms: f64, has_target: bool) {
        self.fid += 1;
        Self::push_window(&mut self.total_ms, total_ms);
        let fps = if total_ms > 0.0 {
            1000.0 / total_ms
        } else {
            0.0
        };
        Self::push_window(&mut self.fps_window, fps);
        if has_target {
            self.det_frames += 1;
        } else {
            self.no_target_frames += 1;
        }
    }

    pub fn inc(&mut self, key: &str, n: usize) {
        *self.counters.entry(key.to_string()).or_insert(0) += n;
    }

    pub fn avg_fps(&self) -> f64 {
        if self.fps_window.is_empty() {
            0.0
        } else {
            self.fps_window.iter().sum::<f64>() / self.fps_window.len() as f64
        }
    }
    pub fn elapsed(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }
    fn avg(v: &[f64]) -> f64 {
        if v.is_empty() {
            0.0
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        }
    }

    pub fn log_summary(&mut self) {
        if self.printed {
            return;
        }
        self.printed = true;
        if self.fid == 0 {
            return;
        }
        let elapsed = self.elapsed();
        let avg_fps = if elapsed > 0.0 {
            self.fid as f64 / elapsed
        } else {
            0.0
        };
        let recent = self.avg_fps();
        info!("{}", "=".repeat(55));
        info!(
            "  {} 帧，耗时 {:.1} 秒，平均 {:.1} 帧/秒",
            self.fid, elapsed, avg_fps
        );
        if !self.fps_window.is_empty() {
            info!("  最近 {} 帧：{:.1} 帧/秒", self.fps_window.len(), recent);
        }
        let has_stage = !self.capture_ms.is_empty()
            || !self.pre_ms.is_empty()
            || !self.tpu_ms.is_empty()
            || !self.position_ms.is_empty()
            || !self.control_ms.is_empty()
            || !self.total_ms.is_empty();
        if has_stage {
            let infer_avg = Self::avg(&self.tpu_ms) + Self::avg(&self.nms_ms);
            let n = if !self.total_ms.is_empty() {
                self.total_ms.len()
            } else {
                self.pre_ms.len().max(30)
            };
            info!(
                "  均值 (最近{}帧)：采集 {:.1}ms  预处理 {:.1}ms  推理 {:.1}ms (TPU {:.1}ms + NMS {:.1}ms)  位置 {:.1}ms  控制 {:.1}ms  总计 {:.1}ms",
                n,
                Self::avg(&self.capture_ms),
                Self::avg(&self.pre_ms),
                infer_avg,
                Self::avg(&self.tpu_ms),
                Self::avg(&self.nms_ms),
                Self::avg(&self.position_ms),
                Self::avg(&self.control_ms),
                Self::avg(&self.total_ms)
            );
            info!(
                "  阶段：[1]采集 {:.1}ms  [2]预处理 {:.1}ms  [3]推理 {:.1}ms  [4]位置 {:.1}ms  [5]控制 {:.1}ms",
                Self::avg(&self.capture_ms),
                Self::avg(&self.pre_ms),
                infer_avg,
                Self::avg(&self.position_ms),
                Self::avg(&self.control_ms)
            );
        }
        info!(
            "  检测帧：{}  无目标帧：{}",
            self.det_frames, self.no_target_frames
        );
        if !self.counters.is_empty() {
            let extra: Vec<String> = self
                .counters
                .iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect();
            info!("  计数：{}", extra.join("  "));
        }
        if !self.extra.is_empty() {
            let extra: Vec<String> = self
                .extra
                .iter()
                .map(|(k, v)| format!("{}={}", k, v))
                .collect();
            info!("  额外：{}", extra.join("  "));
        }
        info!("{}", "=".repeat(55));
    }
}

impl Default for Stats {
    fn default() -> Self {
        Self::new()
    }
}
