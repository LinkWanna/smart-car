ip addr add 192.168.1.2/24 dev eth0
./smartcar --bind 192.168.1.2     # 整合：视觉追踪 + 网页（手动/自动切换）

# 日志走 stderr（时间戳只有 时:分:秒.毫秒 / 级别 / 模块），默认 info；级别用 SMARTCAR_LOG 控制：
SMARTCAR_LOG=debug ./smartcar --bind 192.168.1.2           # 全部 debug（含逐帧编码耗时）
SMARTCAR_LOG=warn,sg2002_upper::vision=debug ./smartcar --bind 192.168.1.2  # 默认 warn，只看视觉模块


我写了一个 sg2002-upper/cvimpi-rs，请使用它代替 sg2002-upper/csrc 作为 JPEG 的编解码支持，我通过网口连接了 sg2002，对方 IP 为 192.168.1.2 ，你可以通过 ssh进行测试

## 视觉后端（VPSS 硬件管线）

视觉只有一条后端：**VPSS 硬件管线**（`vision::vpss`）：相机 YUYV 只 memcpy
一次进 VB 块，VPSS 硬件 CSC 一路进两路出 ——

- chn0 `RGB_888_PLANAR`：stride=640、三平面物理地址连续，正好是模型 `[1,3,480,640]`
  的 NCHW 布局，用 `Model::forward_physical` **零拷贝**喂 TPU；
- chn1 `NV12`：`PT_JPEG` 硬编，**640x480 原尺寸、每帧都出**（不缩放、不锁帧；
  需要限速时用 `--video-fps N`）。

相机/会话/建组/bind/模型任何一步失败都直接报错退出——没有 CPU 回退，也没有
「仅预览」降级；模型是启动硬要求，`--no-vision` 时整条视觉关闭、只遥控。

网页侧的 JPEG 也不再被客户端心跳卡住：WebSocket 会话每轮跑 90ms 的推送窗口
（`PUSH_WINDOW`，5ms 粒度查新帧）；**上行（按键/动作/模式）走 HTTP**，由线程池
即时处理，不受推送窗口与"每轮读一条消息"的限制（WS 每 100ms 的 ping 只当推送时钟）。

实测（640x480 YUYV、`yolov8n_tennis_v3`、同一场景）：

| | VPSS 硬件管线 |
|---|---|
| 模型帧率 | **16.5 fps**（相机满速） |
| smartcar CPU 占用 | **6%** |
| 预览 JPEG | 640x480 ~33KB，WS 推送 **16.5 fps** |
| 预览编码耗时 | ~4ms（bind，编码在 TPU 期间完成） |
| 推理 | ~31ms（零拷贝） |

带宽：640x480@16.5fps ≈ 545KB/s（约 4.4Mbps），走 AP 没问题；要省流量就
`--video-fps 8`（或加大 `--quality` 之外的压缩，但一般不必）。

两个板端坑（都已在代码里处理，详见 `cvimpi-rs/README.md` 的 VPSS 章节）：

- **一个组号一个 boot 只能建一次组**（`DestroyGrp` 也不复位），管线用 `VPSS_GRP_AUTO`
  自动往后取，16 个用完后启动失败（重启板子恢复）；
- 组 CSC 默认是 full range，必须 `set_yuv601_limited_to_full()`，
  否则画面偏灰、模型置信度从 0.71 掉到 0.61。

### 预览交接

VPSS chn1 用 `CVI_SYS_Bind` **直连 VENC**（内核内交接），用户态只 `GetStream`；
编码和 TPU 推理在硬件上并行，所以取流几乎不等待。用输入帧 PTS 精确对帧
（驱动透传：`StreamMeta.pts == 输入帧 PTS`）。

- bind 失败 → `VpssStream::start` 返回错误，`smartcar` 报错退出；
- 启动时 `clear_venc_bind()` 清掉上次崩溃留下的残留节点，退出前 `UnBind`；
- 退出前把**未取的码流取干净**（取流失败过就会积压）：否则驱动在 `DestroyChn`
  时会一直等码流缓冲释放，线程卡死在 ioctl 里、内核留下 VENC 通道 / VB 块
  （实测；修好后 ~0.5s 内干净退出）。

实测（640x480 YUYV，同一场景）：

| | bind |
|---|---|
| 预览编码耗时 | **~4ms**（avg，编码在 TPU 期间完成） |
| 视觉帧率 | 贴着相机（16.5~19.8fps，随光照） |
| smartcar CPU | 6% |

> `/proc/cvitek/vpss` 可以放心读（之前那次看门狗重启是电池松动，不是它）。
> 另外别用 sysfs `unbind`/`bind` 重绑 `uvcvideo`：做完 `/dev/video0` 不会回来，
> 相机卡住（S_FMT `Resource busy` 但没进程持有）时直接重启板子最省事。
