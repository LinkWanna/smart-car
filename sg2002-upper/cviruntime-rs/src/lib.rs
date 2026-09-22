//! cviruntime-rs — Safe Rust wrapper for Sophgo CVI Runtime
//!
//! 设计目标：
//!   - `sys` 模块：对 `cviruntime.h` / `bmruntime.h` 的 unsafe FFI 透传
//!   - `model` / `tensor`：safe 封装，early-panic 于错误
//!   - 复用性：不绑定 pipeline 特定逻辑

pub mod error;
pub mod model;
pub mod sys;
pub mod tensor;

pub use error::Error;
pub use model::Model;
pub use tensor::Tensor;

pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}
