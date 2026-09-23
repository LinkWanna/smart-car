use crate::sys;
use std::os::raw::c_void;

#[derive(Debug, Clone)]
pub struct Shape(pub Vec<i32>);

#[derive(Debug)]
pub struct Tensor {
    pub(crate) ptr: *mut c_void,
    pub shape: Vec<i32>,
    pub count: usize,
    pub bytes: usize,
    pub qscale: f32,
    pub zero_point: i32,
}

impl Tensor {
    /// Safety: ptr 必须来自 CVI 运行时且生命周期由 Model 保证
    pub unsafe fn from_raw(ptr: *mut c_void) -> Self {
        if ptr.is_null() {
            panic!("Tensor::from_raw null");
        }
        let shape = unsafe { sys::tensor_shape(ptr) };
        let vec = shape.dim[..shape.dim_size.min(6)].to_vec();
        let count = unsafe { sys::tensor_count(ptr) };
        let bytes = unsafe { sys::tensor_size(ptr) };
        let qscale = unsafe { sys::tensor_quant_scale(ptr) };
        let zp = unsafe { sys::tensor_quant_zero_point(ptr) };
        Self {
            ptr,
            shape: vec,
            count,
            bytes,
            qscale,
            zero_point: zp,
        }
    }

    pub fn as_ptr(&self) -> *mut u8 {
        unsafe { sys::tensor_ptr(self.ptr) as *mut u8 }
    }

    pub fn as_slice(&self) -> &[u8] {
        let p = self.as_ptr();
        if p.is_null() {
            panic!("Tensor as_slice null");
        }
        unsafe { std::slice::from_raw_parts(p as *const u8, self.bytes) }
    }

    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        let p = self.as_ptr();
        if p.is_null() {
            panic!("Tensor as_slice_mut null");
        }
        unsafe { std::slice::from_raw_parts_mut(p, self.bytes) }
    }

    pub fn as_f32_slice(&self) -> &[f32] {
        let p = self.as_ptr() as *const f32;
        if p.is_null() {
            panic!("Tensor as_f32_slice null");
        }
        unsafe { std::slice::from_raw_parts(p, self.bytes / 4) }
    }
}
