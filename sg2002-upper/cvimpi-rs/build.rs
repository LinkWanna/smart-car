//! 构建脚本：把链接器指向固定的 `cvimpi-rs/libs` 目录，并引入 JPEG 工作流用到的
//! 三个中间件库。

use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let libs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("libs");
    println!("cargo:rustc-link-search=native={}", libs.display());

    // 链接期解析传递依赖（libvenc -> libsys 等）。只对可链接目标（bin/test）生效，
    // `cargo check` 不受影响。
    println!("cargo:rustc-link-arg=-Wl,-rpath-link,{}", libs.display());

    for lib in ["sys", "venc", "vdec"] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }

    // libsys 用到了 1 字节原子操作，riscv64 工具链不会内联它
    // （C SDK 同样要链 -latomic，见 pkgconfig/cvi_common.pc）。
    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    if arch != "x86_64" {
        println!("cargo:rustc-link-lib=dylib=atomic");
    }
}
