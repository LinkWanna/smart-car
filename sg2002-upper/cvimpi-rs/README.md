# cvimpi-rs

`cvi_mpi`（CV180X / CV181X / SG200X）的 **Rust 薄 FFI 封装**，只覆盖 **JPEG 编解码**一条工作流：

| 能力 | 依赖的 MPI 模块 | 对应头文件 |
|---|---|---|
| JPEG 编码（YUV → JPEG） | `venc`（`PT_JPEG`） | `cvi_venc.h` |
| JPEG 解码（JPEG → YUV） | `vdec`（`PT_JPEG`） | `cvi_vdec.h` |
| 初始化 / VB 池 | `sys` | `cvi_sys.h`、`cvi_vb.h` |

VI / ISP / VPSS / VO / RGN / GDC / audio / IVE / bin 全部不在范围内；需要时可直接用 `ffi` 模块里的原始绑定继续扩展。

## 环境准备

1. Rust target（已装可跳过）：

   ```sh
   rustup target add riscv64gc-unknown-linux-musl
   ```

2. 交叉工具链：`.cargo/config.toml` 默认使用 `riscv64-unknown-linux-musl-gcc`
   （Xuantie-900 / CVITEK musl）。换工具链时改这一行即可。

3. `libs/`（固定路径，无需配置）里只保留链接所需的 3 个 `.so`：
   `libsys.so` / `libvenc.so` / `libvdec.so`。运行期这些库由设备提供
   （SDK 会安装到 `/usr/lib`），`libs/` 只用于链接；ABI 对照用的头文件在仓库
   根目录 `../include`。

## 构建 / 部署

```sh
cargo check          # 只做类型检查 + ABI 断言（不需要链接）
cargo build --release
scp target/riscv64gc-unknown-linux-musl/release/cvimpi-rs root@<board>:/usr/bin/
```

设备上直接跑 CLI 做冒烟测试：

```sh
cvimpi-rs encode in_1920x1080.yuv out.jpg 1920 1080 --pixfmt yuv420p --quality 85
cvimpi-rs decode out.jpg back.yuv --pixfmt nv12
```

## 库 API 示例

```rust
use cvimpi_rs::decoder::DecoderConfig;
use cvimpi_rs::encoder::EncoderConfig;
use cvimpi_rs::sys::{Sys, VbPoolConfig};
use cvimpi_rs::{ffi, vdec_frame_buffer_size, venc_input_layout};

// 一次初始化：池 block 取编码输入与解码输出所需的最大值
let enc_layout = venc_input_layout(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420).unwrap();
let dec_size = vdec_frame_buffer_size(1920, 1080, ffi::PIXEL_FORMAT_NV12).unwrap();
let sys = Sys::init(&[VbPoolConfig::new(enc_layout.vb_size.max(dec_size), 4)])?;
// SYS_Init + VB_SetConfig + VB_Init 都在这里；drop 时按 VB_Exit -> SYS_Exit 顺序清理

// ---- 编码 ----
let enc = sys.create_encoder(0,
    &EncoderConfig::new(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420)
        .with_quality(85))?;                          // CreateChn + SetJpegParam + StartRecvFrame
let mut frame = sys.alloc_frame(1920, 1080, ffi::PIXEL_FORMAT_YUV_PLANAR_420)?;
frame.write_tight(&yuv420_bytes)?;                    // 自动处理 stride
let jpeg_data: Vec<u8> = enc.encode(&frame, ffi::CVI_IO_BLOCK)?;  // SendFrame + GetStream
// CPU 写入量大时改 `sys.alloc_frame_cached(...)`：写 cached 内存快一个数量级，
// `send_frame` 会自动 flush（见下方要点）。

// ---- 解码 ----
let dec = sys.create_decoder(0,
    &DecoderConfig::new(1920, 1080).with_pixel_format(ffi::PIXEL_FORMAT_NV12))?;
let out = dec.decode(&jpeg_data, ffi::CVI_IO_BLOCK)?; // SendStream + GetFrame
let yuv = out.copy_tight()?;                          // DecodedFrame Drop 时自动 ReleaseFrame
```

要点：

* **依赖由生命周期保证，不靠约定**：`Encoder<'a>` / `Decoder<'a>` / `Frame<'a>` 借用
  `&'a Sys`，`DecodedFrame<'s>` 借用 `&'s Decoder`。因此"先 drop `Sys` 再用句柄"、
  "先 drop `Decoder` 再用 `DecodedFrame`" 都**编译不过**（`src/sys.rs` 里有两个
  `compile_fail` doctest 固化这两条）。
* `Sys` 是进程级单例：`Sys::init` 用 `AtomicBool` 防重复初始化，重复调用返回 `Err`。
* 句柄都是 `Send`：可以移动到别的线程；封装不阻止 `&` 共享，需要多线程调用中间件时请自行串行化（例如用 `Mutex`）。
* 模块划分：`sys`（会话 + VB 池）、`encoder`、`decoder`；`Error`/`Result` 与缓冲布局
  计算在 crate 根部 `lib.rs`。
* `Encoder::send_frame` + `Encoder::get_stream` 可拆开用于流水线；`encode()` 是二者的组合。
* `DecodedFrame` 期间可以 `plane(i)` 直接读（driver 已映射好虚拟地址），
  `copy_tight()` 会按格式拼成紧凑 YUV。
* `Frame::write_tight` / `DecodedFrame::copy_tight` 支持 `yuv420p/422p/444p`、`nv12/nv21/nv16/nv61`、`yuyv/uyvy/yvyu/vyuy`。
* 编码输入帧默认 uncached（`CVI_SYS_Mmap`，写完免 flush）；CPU 要大量写入时用
  `Sys::alloc_frame_cached`（`CVI_SYS_MmapCache`），**写 cached 内存快一个数量级**。
  cached 帧由 `Encoder::send_frame` 在送硬件前自动 `CVI_SYS_IonFlushCache`；
  手动调 `CVI_VENC_SendFrame` 时用 `Frame::flush`（uncached 帧是 no-op）。
  实测 SG2002 写 640x480 NV12（460KB）：uncached 25ms → cached 3.3ms。

## 缓冲池大小怎么算

`lib.rs` 把 `cvi_buffer.h` 里的静态内联函数原样搬到了 Rust，避免引入 C 头：

* 编码输入帧：`venc_input_layout(w, h, fmt).vb_size`
  （等价 `VENC_GetPicBufferConfig`，宽 32 / 高 16 对齐，stride 对齐 64）
* 解码输出帧：`vdec_frame_buffer_size(w, h, fmt)`
  （等价 `VDEC_GetPicBufferSize`，宽 64 / 高 16 对齐）
* 紧凑帧总大小：`tight_frame_size(w, h, fmt)`

VB 池 block 取二者最大值即可同时服务编码与解码。

注意 common 池模式下的一个驱动约束：解码通道的 `u32FrameBufCnt` 必须**不小于**
它使用的池的块数，否则 `CVI_VDEC_SendStream` 会返回 `BUF_FULL`（实测：4 块池 +
`frame_buf_cnt = 1` 必失败，2 块池 + `frame_buf_cnt = 1` 也一样；池块数 1/1、2/2、
4/4 均正常）。封装会自动取 `max(配置值, 池最大块数)`，所以默认的
`frame_buf_cnt = 1`（与 `sample/vdec` 一致）在块多的共享池里也能正常工作。
拿到 `DecodedFrame` 后仍建议尽快读取/释放，否则池会耗尽阻塞。

### 内核残留池（进程崩溃 / 被 SIGKILL 之后）

进程异常退出（崩溃、`kill -9`）时中间件不会回收内核侧的 VB 池和块引用计数：
`/proc/cvitek/vb` 里能看到池还在、部分块的 USER 引用仍为 1，而且
`CVI_SYS_Exit` / `CVI_VB_Exit` / `CVI_VB_DestroyPool` **都清不掉它**（实测
`DestroyPool` 返回成功但池仍在）。此时驱动会继续使用那个残留池：

* 残留池 block 够大时功能正常，但块的引用计数也残留，**可用块变少**；
* `Sys::init` 之后再 `CVI_VB_GetConfig` 读回内核实际配置，`Sys::kernel_pools()` /
  `Sys::pools_match_request()` 可以查询是否命中了残留池（`Some(false)` 即请求被忽略）；
* 解码的 `u32FrameBufCnt` 现在按**读回的池**计算，因此残留池不会造成 `BUF_FULL`；
* CLI 在检测到不一致时会打印 warning；要彻底恢复干净状态只能**重启设备**。

## ABI 说明

* 绑定按 **64 位 LP64**（riscv64 / aarch64）编写；32 位 ARM 的 C 结构里有额外的
  `#ifdef __arm__` padding 字段，本封装未覆盖（`ffi::abi_check` 也只在非 arm 上启用）。
* `src/ffi.rs` 底部的 `abi_check` 用 `size_of!` / `offset_of!` 做**编译期断言**；
  一旦 `cvi_mpi/include` 的版本和断言不一致，`cargo check` 会直接失败。
* 升级 `cvi_mpi` 后重新测量：

  ```sh
  gcc -I ../include -D__CV181X__ tools/abi_probe.c -o /tmp/abi_probe && /tmp/abi_probe
  ```

  把输出与 `src/ffi.rs::abi_check` 里的数字对齐即可。
* 已知省略：`VENC_STREAM_S` 的两个 stream-info union 作为 368 字节 opaque 处理；
  `VENC_RC_ATTR_S` / `VDEC_CHN_PARAM_S` 的 union 也只保留 opaque 存储
  （JPEG 路径不使用，需要时按 probe 结果补齐）。

## 已验证

* 真机（SG2002 / LicheeRV Nano，Linux 5.10.4-riscv64，musl）：
  `encode`（`yuv420p` / `nv12` 输入）与 `decode`（请求 `nv12` / `yuv420p` / `yuv444p`）
  全部跑通；编码结果与输入 PSNR ≈ 41 dB，解码结果与主机 ffmpeg 解码 PSNR ≈ 75 dB（Y）。
  请求 `yuv444p` 时驱动实际输出 planar 420（`copy_tight` 按驱动返回的格式输出，无影响）。
  同一进程内「编码 + 解码共用一个 4 块 common 池」的 roundtrip 通过，输出与 CLI 编码完全一致。
* 真机残留池场景：先跑进程再 `kill -9` 制造内核残留 VB 池（`/proc/cvitek/vb` 可复现），
  新代码能检测到不一致（`pools_match_request() == Some(false)`）并按内核实际池解码成功；
  输出与干净状态逐字节一致（编码 JPEG md5 相同、解码 PSNR 75.5 dB）。
  同一进程内编码+解码交替 10 轮的共享池测试通过（编 14.6ms / 解 13.6ms 每帧）。
* 真机 `alloc_frame_cached`：640x480 YUYV→NV12 转换写 460KB，uncached 25ms → cached 3.3ms
  （`send_frame` 自动 flush；两帧不同输入编码结果不同，证明 DMA 读到了新数据）。
* `cargo check`：通过（含全部 ABI 断言），无 warning。
* `cargo build --release --target riscv64gc-unknown-linux-musl`：链接成功，
  `NEEDED` 为 `libsys.so / libvenc.so / libvdec.so / libatomic.so.1 / libc.so`。
* `qemu-riscv64` 下运行 CLI：动态加载整条依赖链成功；`CVI_SYS_Init` 因宿主无
  `/dev/cvi-base` 按预期报错（`0xc0028010`）。
* doctest：5 个 `no_run`（编译通过，含 `Send` 断言）+ 2 个 `compile_fail`，后者证明
  "句柄比 `Sys` 活得久"和"`DecodedFrame` 比 `Decoder` 活得久"无法通过编译。

## 已知限制

* 只做 JPEG；H.264/H.265、VI/ISP、显示、音频等未封装。
* 只使用 global common VB 池（`VB_SOURCE_COMMON`），未封装 `CVI_VDEC_AttachVbPool`
  的 per-channel 池模式（`--vbMode=user`）。
* 硬件 JPEG 解码器一般只支持 baseline；progressive JPEG 请先在设备上用 sample 验证。
* 编码输入帧默认 uncached 映射（`CVI_SYS_Mmap`，写完免 flush）；CPU 密集写入时用
  `Sys::alloc_frame_cached`（cached，`send_frame` 自动 flush）。解码输出读取前会
  调用一次 `CVI_SYS_IonInvalidateCache`（与 C sample 一致）。
