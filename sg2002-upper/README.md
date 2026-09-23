ip addr add 192.168.1.2/24 dev eth0
./smartcar --bind 192.168.1.2     # 整合：视觉追踪 + 网页（手动/自动切换）
./webctl --bind 192.168.1.2       # 只遥控（MJPG 预览）

# 日志走 stderr（时间戳/级别/模块），默认 info；级别用 SMARTCAR_LOG 控制：
SMARTCAR_LOG=debug ./smartcar --bind 192.168.1.2           # 全部 debug（含逐帧编码耗时）
SMARTCAR_LOG=warn,hwjpeg=debug ./smartcar --bind 192.168.1.2  # 默认 warn，只看编码耗时


我写了一个 sg2002-upper/cvimpi-rs，请使用它代替 sg2002-upper/csrc 作为 JPEG 的编解码支持，我通过网口连接了 sg2002，对方 IP 为 192.168.1.2 ，你可以通过 ssh进行测试

## 视觉后端（`--vpss auto|off`，默认 auto）

`--vpss auto` 优先走 **VPSS 硬件管线**（`src/vpss_stream.rs`）：相机 YUYV 只 memcpy
一次进 VB 块，VPSS 硬件 CSC 一路进两路出 ——

- chn0 `RGB_888_PLANAR`：stride=640、三平面物理地址连续，正好是模型 `[1,3,480,640]`
  的 NCHW 布局，用 `Model::forward_physical` **零拷贝**喂 TPU；
- chn1 `NV12`：`PT_JPEG` 硬编（`--scale` 由硬件缩放，真的省带宽了）。

VPSS 建组失败 / 组号用尽 / 模型输入尺寸不匹配 → 自动回退 `--vpss off`（CPU 路径）。

实测（640x480 YUYV、`yolov8n_tennis_v3`、同一场景、`--scale 2`）：

| | `--vpss auto` | `--vpss off` |
|---|---|---|
| 模型帧率 | **16.4 fps**（相机满速） | 13.1 fps |
| smartcar CPU 占用 | **6%** | 42% |
| 预览 JPEG | 320x240（硬件缩放）~12KB | 640x480（`--scale` 被忽略）~29KB |
| VENC 耗时 | ~14ms | ~22ms |
| 推理 | ~31ms（零拷贝） | ~32ms（含 921KB memcpy） |

两个板端坑（都已在代码里处理，详见 `cvimpi-rs/README.md` 的 VPSS 章节）：

- **一个组号一个 boot 只能建一次组**（`DestroyGrp` 也不复位），管线用 `VPSS_GRP_AUTO`
  自动往后取，16 个用完后只能回退 CPU（重启板子恢复）；
- 组 CSC 默认是 full range，必须 `set_yuv601_limited_to_full()`，
  否则画面偏灰、模型置信度从 0.71 掉到 0.61。

上板验证探针：`./vpss_probe --csc expand601 --venc --model /root/*.cvimodel --frames 30`。
**别读 `/proc/cvitek/vpss`**（实测会把板子卡死到看门狗重启）。
