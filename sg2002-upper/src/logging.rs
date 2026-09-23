//! 日志初始化：`log` 门面 + `simple_logger` 后端。
//!
//! 每个 bin 在 `main` 开头调用 [`init`]；库内直接用 `log::{info, warn, error, debug}`
//! 宏，输出到 **stderr**（带时间戳 / 级别 / 模块，tty 上带颜色）。
//!
//! 级别由环境变量 `SMARTCAR_LOG` 控制（未设置时看 `RUST_LOG`，默认 `info`）：
//!
//! ```sh
//! SMARTCAR_LOG=debug                  # 全部 debug
//! SMARTCAR_LOG=warn,vision=debug      # 默认 warn，vision 模块 debug
//! SMARTCAR_LOG=info,smartcar=debug    # bin 的 target 是 bin 名
//! ```

use log::LevelFilter;

/// 本 crate 的模块路径前缀（短模块名 `vision` 会展开成 `sg2002_upper::vision`）。
const CRATE: &str = "sg2002_upper";

/// 初始化全局 logger；重复调用忽略（已经初始化过就保持原样）。
pub fn init() {
    let spec = std::env::var("SMARTCAR_LOG")
        .or_else(|_| std::env::var("RUST_LOG"))
        .unwrap_or_default();
    let (default, overrides) = parse_spec(&spec);

    let mut logger = simple_logger::SimpleLogger::new().with_level(default);
    for (target, level) in overrides {
        logger = logger.with_module_level(&target, level);
    }
    let _ = logger.init();
}

/// 解析 `[默认级别][,模块=级别]...`；无法解析的项忽略。
///
/// 短模块名（不含 `::`）会同时注册原名和 `sg2002_upper::` 前缀，这样
/// `vision=debug`（库模块）和 `smartcar=debug`（bin）都能命中。
pub(crate) fn parse_spec(spec: &str) -> (LevelFilter, Vec<(String, LevelFilter)>) {
    let mut default = LevelFilter::Info;
    let mut overrides = Vec::new();
    for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        match item.split_once('=') {
            Some((target, level)) => {
                let target = target.trim();
                let Ok(level) = level.trim().parse() else {
                    continue;
                };
                if !target.is_empty() {
                    overrides.push((target.to_string(), level));
                    if !target.contains("::") {
                        overrides.push((format!("{CRATE}::{target}"), level));
                    }
                }
            }
            None => {
                if let Ok(level) = item.parse() {
                    default = level;
                }
            }
        }
    }
    (default, overrides)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_default_and_module_levels() {
        let (default, overrides) = parse_spec("warn,vision=debug,web=off");
        assert_eq!(default, LevelFilter::Warn);
        assert!(overrides.contains(&("vision".to_string(), LevelFilter::Debug)));
        assert!(overrides.contains(&("sg2002_upper::vision".to_string(), LevelFilter::Debug)));
        assert!(overrides.contains(&("web".to_string(), LevelFilter::Off)));
    }

    #[test]
    fn empty_or_bad_spec_falls_back_to_info() {
        let (default, overrides) = parse_spec("");
        assert_eq!(default, LevelFilter::Info);
        assert!(overrides.is_empty());

        let (default, overrides) = parse_spec("bogus,=debug,vision=bogus");
        assert_eq!(default, LevelFilter::Info);
        assert!(overrides.is_empty());
    }
}
