//! position.rs — 位置分析，移植自 python/src/position.py
//!
//! 输入 TPU 检测框，输出 [`PositionResult`]（九宫格分区 / 距离分级）；以及
//! 控制律用的 [`Observation`]（`PositionResult` 的轻量投影）与 [`Distance`]。

use crate::yolo::Detection;

/// 九宫格边界（640x480 管线的默认值）：横向/纵向 33%~66%。
const LEFT_BOUNDARY: f32 = 0.33;
const RIGHT_BOUNDARY: f32 = 0.66;
const TOP_BOUNDARY: f32 = 0.33;
const BOTTOM_BOUNDARY: f32 = 0.66;
/// 距离分级阈值（检测框面积占比）：>5% 近、>1% 中、其余远。
const NEAR_THRESHOLD: f32 = 0.05;
const MID_THRESHOLD: f32 = 0.01;

#[derive(Debug, Clone, PartialEq)]
pub struct Zone;
impl Zone {
    pub const LEFT_TOP: &'static str = "left_top";
    pub const CENTER_TOP: &'static str = "center_top";
    pub const RIGHT_TOP: &'static str = "right_top";
    pub const LEFT_MID: &'static str = "left_mid";
    pub const CENTER_MID: &'static str = "center_mid";
    pub const RIGHT_MID: &'static str = "right_mid";
    pub const LEFT_BOTTOM: &'static str = "left_bottom";
    pub const CENTER_BOTTOM: &'static str = "center_bottom";
    pub const RIGHT_BOTTOM: &'static str = "right_bottom";
    pub const NONE: &'static str = "none";
}

#[derive(Debug, Clone)]
pub struct PositionResult {
    pub target_class: String,
    pub target_confidence: f32,
    pub center_x: f32,
    pub center_y: f32,
    pub size_ratio: f32,
    pub zone: String,
    pub distance: String,
    pub detection_count: usize,
    pub all_detections: Vec<Detection>,
}

impl PositionResult {
    pub fn has_target(&self) -> bool {
        self.detection_count > 0 && self.zone != Zone::NONE
    }

    pub fn summary(&self) -> String {
        if !self.has_target() {
            return format!("[{}] 无目标", self.zone);
        }
        format!(
            "[{}] {} 置信度={:.2} 位置=({:.2},{:.2}) 距离={} 尺寸={:.3}",
            self.zone,
            self.target_class,
            self.target_confidence,
            self.center_x,
            self.center_y,
            self.distance,
            self.size_ratio,
        )
    }
}

pub struct PositionAnalyzer {
    frame_w: f32,
    frame_h: f32,
    frame_area: f32,
    left_b: f32,
    right_b: f32,
    top_b: f32,
    bottom_b: f32,
    near_th: f32,
    mid_th: f32,
    target_class: Option<String>,
}

impl PositionAnalyzer {
    /// 640x480 管线的默认配置（阈值与旧 `vision.rs` 的常量一致）。
    pub fn for_640x480() -> Self {
        Self::new(
            640,
            480,
            LEFT_BOUNDARY,
            RIGHT_BOUNDARY,
            TOP_BOUNDARY,
            BOTTOM_BOUNDARY,
            NEAR_THRESHOLD,
            MID_THRESHOLD,
            None,
        )
    }

    pub fn new(
        frame_w: usize,
        frame_h: usize,
        left_boundary: f32,
        right_boundary: f32,
        top_boundary: f32,
        bottom_boundary: f32,
        near_threshold: f32,
        mid_threshold: f32,
        target_class: Option<String>,
    ) -> Self {
        Self {
            frame_w: frame_w as f32,
            frame_h: frame_h as f32,
            frame_area: (frame_w * frame_h) as f32,
            left_b: left_boundary,
            right_b: right_boundary,
            top_b: top_boundary,
            bottom_b: bottom_boundary,
            near_th: near_threshold,
            mid_th: mid_threshold,
            target_class,
        }
    }

    pub fn analyze(&self, detections: &[Detection]) -> PositionResult {
        let mut result = PositionResult {
            target_class: String::new(),
            target_confidence: 0.0,
            center_x: 0.0,
            center_y: 0.0,
            size_ratio: 0.0,
            zone: Zone::NONE.to_string(),
            distance: "none".to_string(),
            detection_count: detections.len(),
            all_detections: detections.to_vec(),
        };

        // 零拷贝过滤：仅借用，避免 Vec 克隆
        let filtered: Vec<&Detection> = if let Some(ref tc) = self.target_class {
            detections.iter().filter(|d| &d.label == tc).collect()
        } else {
            detections.iter().collect()
        };

        if filtered.is_empty() {
            result.zone = Zone::NONE.to_string();
            result.distance = "none".to_string();
            return result;
        }

        let primary = filtered
            .into_iter()
            .max_by(|a, b| a.confidence.partial_cmp(&b.confidence).unwrap())
            .unwrap();

        result.target_class = primary.label.clone();
        result.target_confidence = primary.confidence;
        result.center_x = primary.center_x() / self.frame_w;
        result.center_y = primary.center_y() / self.frame_h;
        result.size_ratio = primary.area() / self.frame_area;
        result.zone = self.classify_zone(result.center_x, result.center_y);
        result.distance = self.classify_distance(result.size_ratio);

        result
    }

    fn classify_zone(&self, cx: f32, cy: f32) -> String {
        let col = if cx < self.left_b {
            "left"
        } else if cx > self.right_b {
            "right"
        } else {
            "center"
        };
        let row = if cy < self.top_b {
            "top"
        } else if cy > self.bottom_b {
            "bottom"
        } else {
            "mid"
        };
        format!("{}_{}", col, row)
    }

    fn classify_distance(&self, ratio: f32) -> String {
        if ratio > self.near_th {
            "near".to_string()
        } else if ratio > self.mid_th {
            "mid".to_string()
        } else {
            "far".to_string()
        }
    }
}

/// 目标距离分级（由 [`PositionResult`] 按检测框面积占比给出）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Distance {
    /// 很近：直接停车（双轮制动）。
    Near,
    /// 中等距离：减速前进。
    Mid,
    /// 远距离：全速前进。
    Far,
}

impl Distance {
    /// 位置分析输出的分级名（`near`/`mid`/`far`）。
    pub fn parse(name: &str) -> Self {
        match name {
            "near" => Self::Near,
            "mid" => Self::Mid,
            _ => Self::Far,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Near => "near",
            Self::Mid => "mid",
            Self::Far => "far",
        }
    }
}

/// 一帧视觉观测（由 [`PositionResult`] 转换），控制律与预览快照的输入。
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    /// 本帧是否检测到有效目标。
    pub present: bool,
    /// 目标横向偏差，右为正，范围 -1..=1。
    pub err_x: f32,
    /// 目标距离分级。
    pub distance: Distance,
    /// 置信度（仅用于日志）。
    pub confidence: f32,
}

impl Observation {
    /// 由位置分析结果构造：`err_x = (中心x - 0.5) × 2`（右为正，夹到 ±1）。
    pub fn from_result(result: &PositionResult) -> Self {
        Self {
            present: result.has_target(),
            err_x: ((result.center_x - 0.5) * 2.0).clamp(-1.0, 1.0),
            distance: Distance::parse(&result.distance),
            confidence: result.target_confidence,
        }
    }
}
