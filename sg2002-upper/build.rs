//! 构建脚本：编译硬件 JPEG 的 C 薄封装（`csrc/hwjpeg.c`）。
//!
//! 该封装在运行期 dlopen 厂商库 `/mnt/system/usr/lib/{libsys,libvenc}.so`，
//! 编译期只需要 SDK 头文件，所以：
//! - 找得到头文件 → 编进硬件编码路径；
//! - 找不到（或没有交叉 C 编译器）→ 只发警告，软件编码照常工作。

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cviruntime-rs/src/lib.rs");
    build_hwjpeg();
}

/// 找 cvi_mpi 的 include 目录：显式环境变量优先，其次开发机默认路径。
fn find_sdk_include() -> Option<String> {
    if let Ok(dir) = std::env::var("CVI_MPI_INCLUDE") {
        if std::path::Path::new(&dir).join("cvi_venc.h").exists() {
            return Some(dir);
        }
        println!("cargo:warning=CVI_MPI_INCLUDE={dir} 下没有 cvi_venc.h，忽略");
    }
    for dir in [
        "/home/linkwanna/Github/cvi_mpi/include",
        "third_party/cvi_mpi/include",
    ] {
        if std::path::Path::new(dir).join("cvi_venc.h").exists() {
            return Some(dir.to_string());
        }
    }
    None
}

/// 交叉编译时确保 cc 用的是 musl 工具链（与 Rust 目标一致）。
fn musl_gcc() -> Option<String> {
    let candidates = [
        "/opt/musl/riscv64-linux-musl-cross/bin/riscv64-linux-musl-gcc",
        "/usr/bin/riscv64-linux-musl-gcc",
    ];
    for path in candidates {
        if std::path::Path::new(path).exists() {
            return Some(path.to_string());
        }
    }
    // 交给 PATH
    Some("riscv64-linux-musl-gcc".to_string())
}

fn build_hwjpeg() {
    println!("cargo:rerun-if-changed=csrc/hwjpeg.c");
    println!("cargo:rerun-if-env-changed=CVI_MPI_INCLUDE");

    let target = std::env::var("TARGET").unwrap_or_default();
    let Some(include) = find_sdk_include() else {
        println!(
            "cargo:warning=未找到 cvi_mpi SDK 头文件（设置 CVI_MPI_INCLUDE），\
             本次构建不含硬件 JPEG（运行期走软件编码）"
        );
        println!("cargo:rustc-cfg=no_hwjpeg");
        return;
    };

    let mut build = cc::Build::new();
    build
        .file("csrc/hwjpeg.c")
        .include(&include)
        .include(format!("{include}/linux"))
        .opt_level(2)
        .warnings(false);

    // 手动挑 C 编译器（cc-rs 对 riscv64gc-musl 的目标前缀推断不一定命中本机工具链名）
    let cc_env = format!("CC_{}", target.replace('-', "_"));
    if std::env::var(&cc_env).is_err()
        && let Some(gcc) = musl_gcc()
    {
        build.env(&cc_env, gcc);
    }

    build.compile("hwjpeg");
    println!("cargo:rustc-cfg=hwjpeg_sdk");
}
