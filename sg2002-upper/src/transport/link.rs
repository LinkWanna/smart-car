//! 链路层：串口字节流（打开、termios 配置、非阻塞读写、读线程）

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use log::warn;

use super::protocol::{RxEvent, RxParser};

/// 写整帧时 `WouldBlock` 的重试窗口。
const WRITE_RETRY: Duration = Duration::from_millis(50);
/// 读线程空闲（无数据）时的退避。
const IDLE_BACKOFF: Duration = Duration::from_millis(5);
/// 读线程出错（设备拔出等）时的退避，避免刷屏。
const ERROR_BACKOFF: Duration = Duration::from_millis(50);

/// 串口链路：打开 + 读写 + 读线程。
pub(crate) struct Link {
    file: Arc<File>,
    /// 写串行化（下发线程 + 退出时的收尾帧）。
    write_lock: Mutex<()>,
    /// 线程存活标志（读线程与传输层的写线程共用）。
    running: Arc<AtomicBool>,
    reader: Mutex<Option<JoinHandle<()>>>,
}

impl Link {
    /// 打开并尽力配置串口（raw + 波特率；`O_NONBLOCK`）。
    pub(crate) fn open(path: &str, baud: u32) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        let fd = file.as_raw_fd();
        configure_raw(fd, baud);
        set_nonblocking(fd);
        Ok(Self {
            file: Arc::new(file),
            write_lock: Mutex::new(()),
            running: Arc::new(AtomicBool::new(true)),
            reader: Mutex::new(None),
        })
    }

    /// 线程存活标志（传输层的写线程也看它）。
    pub(crate) fn running(&self) -> &AtomicBool {
        &self.running
    }

    /// 写一整段字节（内部串行化；`WouldBlock` 最多重试 [`WRITE_RETRY`]）。
    pub(crate) fn write(&self, bytes: &[u8]) -> io::Result<()> {
        let _guard = self.write_lock.lock().unwrap();
        let deadline = Instant::now() + WRITE_RETRY;
        let mut file = &*self.file;
        let mut off = 0;
        while off < bytes.len() {
            match file.write(&bytes[off..]) {
                Ok(0) => {
                    return Err(io::Error::new(io::ErrorKind::WriteZero, "串口写入返回 0"));
                }
                Ok(n) => off += n,
                Err(e) => match e.kind() {
                    io::ErrorKind::Interrupted => continue,
                    // Linux 上 EWOULDBLOCK == EAGAIN
                    io::ErrorKind::WouldBlock => {
                        if Instant::now() >= deadline {
                            return Err(e);
                        }
                        thread::sleep(Duration::from_millis(1));
                    }
                    _ => return Err(e),
                },
            }
        }
        Ok(())
    }

    /// 启动读线程：解析出的每条事件回调 `on_event`（在读线程里执行，借用
    /// 解析器的内部缓冲，回调内处理完即可）。
    pub(crate) fn spawn_reader<F>(&self, on_event: F)
    where
        F: Fn(RxEvent<'_>) + Send + 'static,
    {
        let file = Arc::clone(&self.file);
        let running = Arc::clone(&self.running);
        let handle = thread::spawn(move || reader_loop(&file, &running, on_event));
        *self.reader.lock().unwrap() = Some(handle);
    }

    /// 停止并等读线程退出；幂等（`Drop` 兜底）。
    pub(crate) fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.reader.lock().unwrap().take() {
            crate::join_with_timeout(handle, "串口读");
        }
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.reader.lock().unwrap().take() {
            let _ = handle.join();
        }
    }
}

/// 读循环：`O_NONBLOCK` 轮询串口 → 帧事件回调。
fn reader_loop<F>(file: &File, running: &AtomicBool, on_event: F)
where
    F: Fn(RxEvent<'_>),
{
    let mut parser = RxParser::new();
    let mut buf = [0u8; 128];
    let mut file = file;
    while running.load(Ordering::Relaxed) {
        match file.read(&mut buf) {
            // 读到 0 字节（非阻塞下少见）：歇一下再看退出标志。
            Ok(0) => thread::sleep(IDLE_BACKOFF),
            Ok(n) => {
                for &byte in &buf[..n] {
                    on_event(parser.feed(byte));
                }
            }
            Err(e) => match e.kind() {
                io::ErrorKind::Interrupted => {}
                io::ErrorKind::WouldBlock => thread::sleep(IDLE_BACKOFF),
                _ => thread::sleep(ERROR_BACKOFF),
            },
        }
    }
}

/// 常见波特率 -> termios 速度常量；未知波特率返回 `None`（保持现状）。
fn baud_flag(baud: u32) -> Option<libc::speed_t> {
    let v = match baud {
        1200 => libc::B1200,
        2400 => libc::B2400,
        4800 => libc::B4800,
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115_200 => libc::B115200,
        230_400 => libc::B230400,
        460_800 => libc::B460800,
        921_600 => libc::B921600,
        _ => return None,
    };
    Some(v as libc::speed_t)
}

/// 尽力设置 raw 模式与波特率（失败只提示，不返回错误）。
fn configure_raw(fd: libc::c_int, baud: u32) {
    unsafe {
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tio) != 0 {
            warn!("tcgetattr 不可用（板端可能已预配置），保持默认");
            return;
        }
        libc::cfmakeraw(&mut tio);
        tio.c_cc[libc::VMIN] = 0;
        tio.c_cc[libc::VTIME] = 0;
        if let Some(speed) = baud_flag(baud) {
            libc::cfsetispeed(&mut tio, speed);
        } else {
            warn!("不支持的波特率 {}，沿用当前配置", baud);
        }
        if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
            warn!("tcsetattr 失败（板端可能已预配置），保持默认");
        }
    }
}

/// 打开 `O_NONBLOCK`（读线程按 [`IDLE_BACKOFF`] 轮询退出标志）。
fn set_nonblocking(fd: libc::c_int) {
    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags >= 0 {
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baud_table_covers_115200() {
        assert_eq!(baud_flag(115_200), Some(libc::B115200 as libc::speed_t));
        assert_eq!(baud_flag(0), None);
    }

    #[test]
    fn open_missing_device_errors() {
        assert!(Link::open("/dev/definitely-not-a-serial-port", 115_200).is_err());
    }
}
