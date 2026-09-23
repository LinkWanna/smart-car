//! Raw FFI，1:1 映射上游 `cviruntime.h` / `bmruntime.h`
//! 仅在 `riscv64` 上真实链接，host 仅保留符号以通过 `cargo check`
//! 函数已去除 `CVI_NN_` 前缀并改为 snake_case，Rust 命名空间已足够

use std::os::raw::{c_char, c_int, c_void};

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CviShape {
    pub dim: [i32; 6],
    pub dim_size: usize,
}

#[link(name = "cviruntime")]
unsafe extern "C" {
    #[link_name = "CVI_NN_RegisterModel"]
    pub fn register_model(model_file: *const c_char, model: *mut *mut c_void) -> c_int;
    #[link_name = "CVI_NN_RegisterModelFromBuffer"]
    pub fn register_model_from_buffer(buf: *const i8, size: u32, model: *mut *mut c_void) -> c_int;
    #[link_name = "CVI_NN_CloneModel"]
    pub fn clone_model(model: *mut c_void, cloned: *mut *mut c_void) -> c_int;
    #[link_name = "CVI_NN_GetModelVersion"]
    pub fn get_model_version(model: *mut c_void, major: *mut i32, minor: *mut i32) -> c_int;
    #[link_name = "CVI_NN_GetModelTarget"]
    pub fn get_model_target(model: *mut c_void) -> *const c_char;
    #[link_name = "CVI_NN_SetConfig"]
    pub fn set_config(model: *mut c_void, option: c_int, ...) -> c_int;
    #[link_name = "CVI_NN_GetInputOutputTensors"]
    pub fn get_input_output_tensors(
        model: *mut c_void,
        inputs: *mut *mut c_void,
        input_num: *mut i32,
        outputs: *mut *mut c_void,
        output_num: *mut i32,
    ) -> c_int;
    #[link_name = "CVI_NN_GetInputTensors"]
    pub fn get_input_tensors(
        model: *mut c_void,
        inputs: *mut *mut c_void,
        input_num: *mut i32,
    ) -> c_int;
    #[link_name = "CVI_NN_GetOutputTensors"]
    pub fn get_output_tensors(
        model: *mut c_void,
        outputs: *mut *mut c_void,
        output_num: *mut i32,
    ) -> c_int;
    #[link_name = "CVI_NN_Forward"]
    pub fn forward(
        model: *mut c_void,
        inputs: *mut c_void,
        input_num: i32,
        outputs: *mut c_void,
        output_num: i32,
    ) -> c_int;
    #[link_name = "CVI_NN_ForwardAsync"]
    pub fn forward_async(
        model: *mut c_void,
        inputs: *mut c_void,
        input_num: i32,
        outputs: *mut c_void,
        output_num: i32,
        task_no: *mut *mut c_void,
    ) -> c_int;
    #[link_name = "CVI_NN_ForwardWait"]
    pub fn forward_wait(model: *mut c_void, task_no: *mut c_void) -> c_int;
    #[link_name = "CVI_NN_CleanupModel"]
    pub fn cleanup_model(model: *mut c_void) -> c_int;
    #[link_name = "CVI_NN_GetTensorByName"]
    pub fn get_tensor_by_name(name: *const c_char, tensors: *mut c_void, num: i32) -> *mut c_void;
    #[link_name = "CVI_NN_TensorName"]
    pub fn tensor_name(tensor: *mut c_void) -> *mut c_char;
    #[link_name = "CVI_NN_TensorPtr"]
    pub fn tensor_ptr(tensor: *mut c_void) -> *mut c_void;
    #[link_name = "CVI_NN_TensorSize"]
    pub fn tensor_size(tensor: *mut c_void) -> usize;
    #[link_name = "CVI_NN_TensorCount"]
    pub fn tensor_count(tensor: *mut c_void) -> usize;
    #[link_name = "CVI_NN_TensorShape"]
    pub fn tensor_shape(tensor: *mut c_void) -> CviShape;
    #[link_name = "CVI_NN_TensorQuantScale"]
    pub fn tensor_quant_scale(tensor: *mut c_void) -> f32;
    #[link_name = "CVI_NN_TensorQuantZeroPoint"]
    pub fn tensor_quant_zero_point(tensor: *mut c_void) -> c_int;
    #[link_name = "CVI_NN_SetTensorPtr"]
    pub fn set_tensor_ptr(tensor: *mut c_void, mem: *mut c_void) -> c_int;
    #[link_name = "CVI_NN_SetTensorPhysicalAddr"]
    pub fn set_tensor_physical_addr(tensor: *mut c_void, paddr: u64) -> c_int;
    #[link_name = "CVI_NN_Global_SetSharedMemorySize"]
    pub fn global_set_shared_memory_size(size: usize);
}
