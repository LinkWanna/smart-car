use crate::error::Error;
use crate::sys;
use crate::tensor::Tensor;
use std::ffi::CString;
use std::os::raw::c_void;

#[allow(dead_code)]
pub struct Model {
    handle: *mut c_void,
    pub inputs: Vec<Tensor>,
    pub outputs: Vec<Tensor>,
    in_num: i32,
    out_num: i32,
    in_ptr: *mut c_void,
    out_ptr: *mut c_void,
}

impl Model {
    pub fn from_file(path: &str) -> Result<Self, Error> {
        let c_path =
            CString::new(path).map_err(|e| Error::InvalidModel(format!("NUL in path: {}", e)))?;
        let mut handle: *mut c_void = std::ptr::null_mut();
        let rc = unsafe { sys::register_model(c_path.as_ptr(), &mut handle) };
        if rc != 0 || handle.is_null() {
            return Err(Error::RegisterFailed(rc, path.to_string()));
        }
        let mut in_ptr: *mut c_void = std::ptr::null_mut();
        let mut in_num = 0;
        let mut out_ptr: *mut c_void = std::ptr::null_mut();
        let mut out_num = 0;
        let rc = unsafe {
            sys::get_input_output_tensors(
                handle,
                &mut in_ptr,
                &mut in_num,
                &mut out_ptr,
                &mut out_num,
            )
        };
        if rc != 0 || in_ptr.is_null() || out_ptr.is_null() {
            unsafe { sys::cleanup_model(handle) };
            return Err(Error::GetTensorsFailed(rc));
        }
        let input = unsafe { Tensor::from_raw(in_ptr) };
        let output = unsafe { Tensor::from_raw(out_ptr) };
        eprintln!(
            "[cviruntime-rs] in_shape={:?} out_shape={:?} in_bytes={} out_bytes={}",
            input.shape, output.shape, input.bytes, output.bytes
        );
        Ok(Self {
            handle,
            inputs: vec![input],
            outputs: vec![output],
            in_num,
            out_num,
            in_ptr,
            out_ptr,
        })
    }

    pub fn input(&self) -> &Tensor {
        &self.inputs[0]
    }

    pub fn input_mut(&mut self) -> &mut Tensor {
        &mut self.inputs[0]
    }

    pub fn output(&self) -> &Tensor {
        &self.outputs[0]
    }

    pub fn forward(&self, data: &[u8]) -> Result<&[u8], Error> {
        if data.len() != self.inputs[0].bytes {
            return Err(Error::SizeMismatch {
                expected: self.inputs[0].bytes,
                actual: data.len(),
            });
        }
        let in_tensor = self.inputs[0].ptr;
        let dst = unsafe { sys::tensor_ptr(in_tensor) as *mut u8 };
        if dst.is_null() {
            return Err(Error::TensorNull);
        }
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), dst, data.len()) };
        let rc = unsafe {
            sys::forward(
                self.handle,
                self.in_ptr,
                self.in_num,
                self.out_ptr,
                self.out_num,
            )
        };
        if rc != 0 {
            return Err(Error::ForwardFailed(rc));
        }
        let out_tensor = self.outputs[0].ptr;
        let src = unsafe { sys::tensor_ptr(out_tensor) as *const u8 };
        let len = self.outputs[0].bytes;
        if src.is_null() {
            return Err(Error::TensorNull);
        }
        let slice = unsafe { std::slice::from_raw_parts(src, len) };
        Ok(slice)
    }
}

impl Drop for Model {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::cleanup_model(self.handle) };
        }
    }
}

unsafe impl Send for Model {}
