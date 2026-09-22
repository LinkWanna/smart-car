fn main() {
    let target = std::env::var("TARGET").unwrap_or_default();
    if target.contains("riscv64") {
        // 板端预编译库随 crate 走：cviruntime-rs/libs（RISC-V musl）
        let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let lib_dir = std::path::Path::new(&manifest_dir)
            .join("libs")
            .canonicalize()
            .unwrap_or_else(|_| std::path::PathBuf::from(format!("{}/libs", manifest_dir)));
        let lib_dir_str = lib_dir.to_string_lossy();
        println!("cargo:rustc-link-search=native={}", lib_dir_str);
        println!("cargo:rustc-link-lib=dylib=cviruntime");
        println!("cargo:rustc-link-lib=dylib=cvikernel");
        println!("cargo:rustc-link-arg=-Wl,-rpath,/lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/lib");
        println!("cargo:rerun-if-changed=libs/libcviruntime.so");
        println!("cargo:rerun-if-changed=libs/libcvikernel.so");
        println!("cargo:rerun-if-env-changed=CARGO_MANIFEST_DIR");
    } else {
        // Host (x86_64) 仅用于 cargo check，不强制链接
    }
    // 暴露上游头文件位置（供 bindgen 或文档）
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=TARGET");
}
