//! 构建脚本：把链接器指向固定的 `cvimpi-rs/libs` 目录，并引入
//! `libsys.so`（`CVI_SYS_Init` / VB 池 / `CVI_SYS_Mmap` / bind）。
//!
//! VENC / VDEC 已改为直连 `/dev/cvi_vc_enc*` / `/dev/cvi_vc_dec*` 的 ioctl
//! （见 `src/encoder.rs` / `src/decoder.rs`），不再链接 `libvenc.so` / `libvdec.so`。

use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let libs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("libs");
    println!("cargo:rustc-link-search=native={}", libs.display());

    println!("cargo:rustc-link-lib=dylib=sys");

    // libsys 用到了 1 字节原子操作，riscv64 工具链不会内联它
    // （C SDK 同样要链 -latomic，见 pkgconfig/cvi_common.pc）。
    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    if arch != "x86_64" {
        println!("cargo:rustc-link-lib=dylib=atomic");
    }
}
