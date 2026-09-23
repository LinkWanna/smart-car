use std::fmt;

#[derive(Debug)]
pub enum Error {
    RegisterFailed(i32, String),
    GetTensorsFailed(i32),
    ForwardFailed(i32),
    SetTensorFailed(i32),
    InvalidModel(String),
    TensorNull,
    SizeMismatch { expected: usize, actual: usize },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RegisterFailed(rc, p) => write!(f, "register_model failed rc={} path={}", rc, p),
            Self::GetTensorsFailed(rc) => write!(f, "get_input_output_tensors failed rc={}", rc),
            Self::ForwardFailed(rc) => write!(f, "forward failed rc={}", rc),
            Self::SetTensorFailed(rc) => write!(f, "set tensor buffer failed rc={}", rc),
            Self::InvalidModel(s) => write!(f, "Invalid model: {}", s),
            Self::TensorNull => write!(f, "Tensor pointer is null"),
            Self::SizeMismatch { expected, actual } => {
                write!(f, "size mismatch expected {} actual {}", expected, actual)
            }
        }
    }
}

impl std::error::Error for Error {}
