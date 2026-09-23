//! camera.rs — V4L2 零拷贝相机，使用 `v4l` crate 替代手写 `videodev2.h`
//!
//! [`Camera::try_new`] 一步完成打开/校验/配置/申请缓冲/开流；采集用
//! [`Camera::get_frame`]，返回的 [`CaptureFrame`] 是 mmap 缓冲的只读句柄：
//! 帧下标封装在句柄内，归还由 [`CaptureFrame::release`] 或 `Drop` 完成。
//! 可以同时持有多帧（上限是申请的缓冲数），全部不归还时 `DQBUF` 阻塞等待回填。
//!
//! ```ignore
//! let camera = Camera::try_new("/dev/video0", "yuyv")?;
//! let frame = camera.get_frame()?;
//! let pixels = frame.as_slice();   // 零拷贝
//! frame.release()?;                // 或者直接 drop(frame)
//! ```

use std::io;
use std::os::unix::io::RawFd;
use std::ptr;

use v4l::format::FourCC;
use v4l::v4l_sys::{v4l2_buffer, v4l2_capability, v4l2_format, v4l2_requestbuffers};
use v4l::v4l2::vidioc;

/// 一帧 mmap 缓冲的只读句柄，由 [`Camera::get_frame`] 返回。
///
/// 帧下标是句柄内部状态；`Drop` 自动 QBUF 归还，[`CaptureFrame::release`] 可
/// 显式归还并拿到错误。句柄只借用 `&Camera`，所以可以同时持有多帧，
/// 期间不能 `stop`/析构相机。
pub struct CaptureFrame<'a> {
    camera: &'a Camera,
    idx: usize,
    ptr: *mut u8,
    len: usize,
    released: bool,
}

impl CaptureFrame<'_> {
    /// 零拷贝只读视图
    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }

    /// 显式归还缓冲；重复调用不会二次 QBUF（`Drop` 看同一个标志）。
    pub fn release(mut self) -> io::Result<()> {
        self.released = true;
        self.camera.queue_index(self.idx)
    }
}

impl Drop for CaptureFrame<'_> {
    fn drop(&mut self) {
        if !self.released {
            self.released = true;
            self.camera.queue_index(self.idx).ok();
        }
    }
}

struct MmapBuffer {
    ptr: *mut libc::c_void,
    len: usize,
}

pub struct Camera {
    fd: RawFd,
    bufs: Vec<MmapBuffer>,
    w: u32,
    h: u32,
    /// 驱动实际协商到的像素格式（如 `MJPG`）。
    fourcc: FourCC,
    /// 打开的设备节点（USB 相机重新枚举后会变）。
    device: String,
}

impl Camera {
    /// 打开设备并按 `fmt_str`（`yuyv` / `jpeg` / `mjpeg`）配置 640x480、
    /// 申请 mmap 缓冲、启动视频流。
    ///
    /// 任一步失败返回 `io::Error`（消息带 `camera_init 失败:` 前缀），
    /// 调用方（视觉管线 / 网页预览）据此优雅降级；成功后可用
    /// [`Camera::pixel_format`] 检查驱动是否真的接受了请求的格式。
    pub fn try_new(device: &str, fmt_str: &str) -> io::Result<Self> {
        let fourcc = match fmt_str {
            "yuyv" => FourCC::new(b"YUYV"),
            "jpeg" | "mjpeg" => FourCC::new(b"MJPG"),
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("camera_init 失败: 不支持的格式: {other}"),
                ));
            }
        };
        let fd = v4l::v4l2::open(device, libc::O_RDWR).map_err(|e| {
            io::Error::new(e.kind(), format!("camera_init 失败: 打开 {device}: {e}"))
        })?;
        match Self::setup(fd, fourcc, device) {
            Ok(cam) => Ok(cam),
            Err(e) => {
                v4l::v4l2::close(fd).ok();
                Err(e)
            }
        }
    }

    /// 在已打开的 fd 上完成配置；失败时由调用方关闭 fd。
    fn setup(fd: RawFd, fourcc: FourCC, device: &str) -> io::Result<Self> {
        let mut cam = Self {
            fd,
            bufs: Vec::new(),
            w: 640,
            h: 480,
            fourcc: FourCC::new(b"????"),
            device: device.to_string(),
        };
        if let Err(e) = cam.configure(fourcc) {
            cam.unmap_all();
            return Err(e);
        }
        Ok(cam)
    }

    /// 校验设备、协商格式、申请缓冲并开流。
    fn configure(&mut self, fourcc: FourCC) -> io::Result<()> {
        // 校验 caps — 使用 v4l2_capability
        let mut cap: v4l2_capability = unsafe { std::mem::zeroed() };
        unsafe {
            v4l::v4l2::ioctl(
                self.fd,
                vidioc::VIDIOC_QUERYCAP,
                &mut cap as *mut _ as *mut _,
            )
        }
        .map_err(|e| fail("QUERYCAP", e))?;
        if cap.capabilities & 0x00000001 == 0 {
            return Err(err("非视频采集设备"));
        }
        if cap.capabilities & 0x04000000 == 0 {
            return Err(err("不支持 STREAMING"));
        }

        // 设置格式 — 写联合体字段不需要 unsafe，回读 fmt 才需要
        let mut fmt: v4l2_format = unsafe { std::mem::zeroed() };
        fmt.type_ = 1; // V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmt.fmt.pix.width = self.w;
        fmt.fmt.pix.height = self.h;
        fmt.fmt.pix.pixelformat = fourcc.into();
        fmt.fmt.pix.field = 1; // V4L2_FIELD_NONE
        unsafe { v4l::v4l2::ioctl(self.fd, vidioc::VIDIOC_S_FMT, &mut fmt as *mut _ as *mut _) }
            .map_err(|e| fail("S_FMT", e))?;
        let (got_w, got_h) = unsafe { (fmt.fmt.pix.width, fmt.fmt.pix.height) };
        if got_w != self.w || got_h != self.h {
            return Err(err(&format!(
                "驱动修正尺寸 {}x{} != {}x{}",
                got_w, got_h, self.w, self.h
            )));
        }
        let got_fourcc = FourCC::from(unsafe { fmt.fmt.pix.pixelformat });
        if got_fourcc != fourcc {
            return Err(err(&format!("驱动修正格式 {} != {}", got_fourcc, fourcc)));
        }
        self.fourcc = got_fourcc;

        // 申请缓冲
        let mut req: v4l2_requestbuffers = unsafe { std::mem::zeroed() };
        req.count = 4;
        req.type_ = 1;
        req.memory = 1; // V4L2_MEMORY_MMAP
        unsafe {
            v4l::v4l2::ioctl(
                self.fd,
                vidioc::VIDIOC_REQBUFS,
                &mut req as *mut _ as *mut _,
            )
        }
        .map_err(|e| fail("REQBUFS", e))?;
        if req.count < 2 {
            return Err(err(&format!("缓冲数不足: {}", req.count)));
        }
        let nbufs = req.count as usize;
        self.bufs.reserve(nbufs);

        for i in 0..nbufs {
            let mut buf: v4l2_buffer = unsafe { std::mem::zeroed() };
            buf.type_ = 1;
            buf.memory = 1;
            buf.index = i as u32;
            unsafe {
                v4l::v4l2::ioctl(
                    self.fd,
                    vidioc::VIDIOC_QUERYBUF,
                    &mut buf as *mut _ as *mut _,
                )
            }
            .map_err(|e| fail(&format!("QUERYBUF[{i}]"), e))?;
            let len = buf.length as usize;
            let offset = unsafe { buf.m.offset };
            let ptr = unsafe {
                v4l::v4l2::mmap(
                    ptr::null_mut(),
                    len,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    self.fd,
                    offset as libc::off_t,
                )
            }
            .map_err(|e| fail(&format!("mmap[{i}]"), e))?;
            self.bufs.push(MmapBuffer { ptr, len });
        }

        for i in 0..nbufs {
            let mut buf: v4l2_buffer = unsafe { std::mem::zeroed() };
            buf.type_ = 1;
            buf.memory = 1;
            buf.index = i as u32;
            unsafe { v4l::v4l2::ioctl(self.fd, vidioc::VIDIOC_QBUF, &mut buf as *mut _ as *mut _) }
                .map_err(|e| fail(&format!("QBUF[{i}]"), e))?;
        }

        let mut buftype: u32 = 1;
        unsafe {
            v4l::v4l2::ioctl(
                self.fd,
                vidioc::VIDIOC_STREAMON,
                &mut buftype as *mut _ as *mut _,
            )
        }
        .map_err(|e| fail("STREAMON", e))?;
        Ok(())
    }

    /// 打开的设备节点。
    pub fn device(&self) -> &str {
        &self.device
    }

    /// 驱动实际协商到的像素格式（如 `MJPG` / `YUYV`）。
    pub fn pixel_format(&self) -> String {
        self.fourcc.to_string()
    }

    /// 取一帧：`DQBUF` 成功后返回 [`CaptureFrame`]。
    ///
    /// 可以同时持有多帧（上限是驱动申请的缓冲数），都不归还时这里会阻塞
    /// 等待驱动回填。
    pub fn get_frame(&self) -> io::Result<CaptureFrame<'_>> {
        if self.fd < 0 {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "camera 未初始化",
            ));
        }
        let mut buf: v4l2_buffer = unsafe { std::mem::zeroed() };
        buf.type_ = 1;
        buf.memory = 1;
        unsafe { v4l::v4l2::ioctl(self.fd, vidioc::VIDIOC_DQBUF, &mut buf as *mut _ as *mut _) }?;
        let idx = buf.index as usize;
        if idx >= self.bufs.len() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("DQBUF 非法索引 {}", idx),
            ));
        }
        let ptr = self.bufs[idx].ptr as *mut u8;
        let len = buf.bytesused as usize;
        Ok(CaptureFrame {
            camera: self,
            idx,
            ptr,
            len,
            released: false,
        })
    }

    /// 把缓冲还给驱动（[`CaptureFrame`] 的 `release`/`Drop` 调用）。
    fn queue_index(&self, idx: usize) -> io::Result<()> {
        let mut buf: v4l2_buffer = unsafe { std::mem::zeroed() };
        buf.type_ = 1;
        buf.memory = 1;
        buf.index = idx as u32;
        unsafe { v4l::v4l2::ioctl(self.fd, vidioc::VIDIOC_QBUF, &mut buf as *mut _ as *mut _) }?;
        Ok(())
    }

    /// 解除所有 mmap（配置失败与 [`Camera::stop`] 共用）。
    fn unmap_all(&mut self) {
        for b in &self.bufs {
            unsafe { v4l::v4l2::munmap(b.ptr, b.len).ok() };
        }
        self.bufs.clear();
    }

    pub fn stop(&mut self) {
        if self.fd < 0 {
            return;
        }
        let mut buftype: u32 = 1;
        unsafe {
            v4l::v4l2::ioctl(
                self.fd,
                vidioc::VIDIOC_STREAMOFF,
                &mut buftype as *mut _ as *mut _,
            )
            .ok()
        };
        self.unmap_all();
        v4l::v4l2::close(self.fd).ok();
        self.fd = -1;
    }
}

/// `camera_init 失败: <原因>` 形式的错误。
fn err(msg: &str) -> io::Error {
    io::Error::other(format!("camera_init 失败: {msg}"))
}

/// 带 ioctl 名称的错误。
fn fail(what: &str, e: io::Error) -> io::Error {
    io::Error::new(e.kind(), format!("camera_init 失败: {what}: {e}"))
}

/// `CaptureFrame` 只借用相机，`Camera` 本身（fd + mmap 地址）移动到别的线程
/// 使用时没有线程局部状态，V4L2 ioctl 对 fd 也是线程无关的。
unsafe impl Send for Camera {}

impl Drop for Camera {
    fn drop(&mut self) {
        self.stop();
    }
}

impl std::fmt::Display for Camera {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Camera({}, {}x{} {} via v4l)",
            self.device, self.w, self.h, self.fourcc
        )
    }
}

/// 打开 YUYV 相机：先用配置的节点；不存在/打不开时在 `/dev/video*` 里找第一个
/// 能出 YUYV 的。
///
/// USB 相机重新枚举后编号会变（`video0` → `video1`），所以这里不能死认一个节点。
pub fn open_yuyv(device: &str) -> io::Result<Camera> {
    let first_error = match Camera::try_new(device, "yuyv") {
        Ok(camera) => return Ok(camera),
        Err(e) => e,
    };
    if std::path::Path::new(device).exists() {
        // 节点在但配置不上（被占用/格式不支持）：直接报错，避免误开别的节点
        return Err(first_error);
    }

    let mut nodes: Vec<std::path::PathBuf> = std::fs::read_dir("/dev")
        .map(|entries| {
            entries
                .flatten()
                .map(|entry| entry.path())
                .filter(|path| {
                    path.file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.starts_with("video"))
                })
                .collect()
        })
        .unwrap_or_default();
    nodes.sort();
    for node in nodes {
        let Some(path) = node.to_str() else { continue };
        if path == device {
            continue;
        }
        if let Ok(camera) = Camera::try_new(path, "yuyv") {
            log::info!("{device} 不存在，改用 {path}（USB 重新枚举后编号会变）");
            return Ok(camera);
        }
    }
    Err(io::Error::other(format!(
        "找不到可用的 YUYV 相机（{device} 及 /dev/video*）"
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn struct_sizes_via_v4l() {
        assert_eq!(std::mem::size_of::<v4l2_capability>(), 104);
        assert_eq!(std::mem::size_of::<v4l2_format>(), 208);
        assert_eq!(std::mem::size_of::<v4l2_requestbuffers>(), 20);
        assert_eq!(std::mem::size_of::<v4l2_buffer>(), 88);
    }

    #[test]
    fn try_new_reports_instead_of_panicking() {
        // 不存在的设备：返回错误而不是 panic。
        let err = match Camera::try_new("/dev/video-no-such", "mjpeg") {
            Ok(_) => panic!("不存在的设备不应打开成功"),
            Err(e) => e,
        };
        assert!(err.to_string().contains("camera_init 失败"), "{err}");
        // 不支持的格式名同样报错。
        assert!(Camera::try_new("/dev/video-no-such", "rgb").is_err());
    }
}
