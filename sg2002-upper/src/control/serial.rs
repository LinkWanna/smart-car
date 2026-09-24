//! 原始串口打开与配置（SG2002 `/dev/ttyS1` ↔ ESP32-C3 UART0；本板 `ttyS0` 是调试控制台）。
//!
//! 板端（StarryOS）对 termios 的支持不保证完整，所以这里的策略是：
//! - 打开 `/dev/ttyS1` 后尽力设置为 raw + 波特率；
//! - `tcgetattr`/`tcsetattr` 失败不致命（板端串口可能已由内核/启动脚本配好），
//!   只留下日志警告；
//! - `O_NONBLOCK` 用于读线程，写侧由 [`crate::control::car`] 处理 `EAGAIN` 重试。

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::io::AsRawFd;

use log::warn;

/// 以读写方式打开串口并尽力配置。
pub fn open(path: &str, baud: u32) -> io::Result<File> {
    let file = OpenOptions::new().read(true).write(true).open(path)?;
    let fd = file.as_raw_fd();
    configure_raw(fd, baud);
    set_nonblocking(fd);
    Ok(file)
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

/// 打开 `O_NONBLOCK`（读线程按 5ms 轮询退出标志）。
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
        assert!(open("/dev/definitely-not-a-serial-port", 115_200).is_err());
    }
}
