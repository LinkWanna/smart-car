# ESP32-C3 智能小车

ESP32-C3 双电机编码器闭环控制器，上位机经 UART0 或 BLE 下发速度指令，固件做 PID 闭环 + 状态回传。
两条链路编译期二选一（`uart` / `ble` feature，见「构建与烧录」），不共存。

## 硬件接线

| 功能 | 引脚 |
| --- | --- |
| 左电机 IN1 / IN2 | GPIO2 / GPIO1 |
| 右电机 IN1 / IN2 | GPIO3 / GPIO4 |
| 左编码器 A / B | GPIO7 / GPIO10 |
| 右编码器 A / B | GPIO5 / GPIO6 |
| LED（低电平点亮） | GPIO8 |
| UART0 RX / TX（115200 8N1） | GPIO20 / GPIO21 |

电机为 8 位 LEDC PWM，载波频率 20kHz，启动后只调占空比。编码器为正交输入，
双边沿中断计数（单圈 4680 脉冲），左轮转速取反（和 Arduino 版一致）。

CPU 时钟沿用 HAL 默认的 80MHz（`esp_hal::init` 的 preset），UART 波特率、
LEDC 载波、TIMG/systimer 都由时钟树据此计算分频。

## 仓库结构

```
../protocol/         # 上下位机共用的协议 crate（smart-car-protocol，no_std）
  src/frame.rs       # 帧编码 + 逐字节接收状态机
  src/request.rs     # 11 条命令负载的 decode/encode
  src/response.rs    # Ack/Nack/Status/PidData 的 decode/encode
  src/types.rs       # 命令号/应答号/错误码/状态/电机/方向枚举
  src/tests.rs       # 黄金字节与编解码回归测试
src/
  main.rs            # 命令解析分发（command_task）+ 外设初始化
  drivers/
    motor.rs         # PID 运算 + H 桥驱动（coast / brake / drive）
    encoder.rs       # 正交编码器中断 + 计数
  service/
    control.rs       # 共享 State、50ms 控制节拍（control_task）
    odometry.rs      # 闭环 Move/Rotate 目标状态、换算与到达判定
    link.rs          # 命令链路句柄（收字节 / 发帧）
    ble.rs           # BLE NUS 字节管道（feature = "ble"）
    uart.rs          # UART0 收发（feature = "uart"）
tools/
  play.py            # 键盘遥控试车
  move.py            # 闭环走距离 / 原地转向
  tests/
    test_protocol.py # 协议一致性测试
  src/
    config.py        # 所有工具共享的默认参数
    protocol.py      # Python 侧协议库（镜像 ../protocol/）
    keyboard.py      # 终端键盘输入
```

协议实现只有一份：固件和上位机（`sg2002-upper`）都依赖仓库根目录的
`smart-car-protocol` crate，`src/main.rs` 里用
`pub(crate) use smart_car_protocol as protocol;` 保留 `crate::protocol` 路径。

分层：`command_task` 只往 `State` 里写意图（`MotorCmd::{Drive, Coast, Brake}`，
以及 Move/Rotate 的闭环目标），`control_task` 独占电机、编码器记账和 LED，
每 50ms 把意图落实到硬件。两边经一把异步 `Mutex<State>` 互斥，和原来单任务
循环的原子性一致。

## 通信协议

两种链路（UART0、BLE）走同一种帧格式，构建时按 feature 二选一：

```
AA 55 CMD LEN PAYLOAD.. CHK      CHK = CMD ^ LEN ^ PAYLOAD..
```

| 命令 | 值 | 负载 | 说明 |
| --- | --- | --- | --- |
| Init | `0x01` | 无 | 编码器清零，进 Ready |
| SetSpeed | `0x10` | mid + speed(i16) | 单轮目标，-100~100，进 Running |
| SetSpeeds | `0x13` | 左(i16) + 右(i16) | 双轮目标 |
| Stop | `0x11` | mode(0/1/2=左/右/双) | 滑行（coast） |
| Brake | `0x12` | mode | 短接制动，保持到下条指令 |
| SetPid | `0x14` | mid + kp/ki/kd(i16, ×100) | 改增益 |
| GetPid | `0x15` | mid | 回 PidData |
| Move | `0x20` | dir(0前/1后) + speed(1~100) + mm(i32) | 闭环直行，到达自动刹车 |
| Rotate | `0x21` | dir(0左/1右) + speed(1~100) + 0.1°(i32) | 原地闭环转向，到达自动刹车 |
| Heartbeat | `0xFE` | 无 | 回 Status |
| Reset | `0xFF` | 无 | 回 Uninit |

应答：`ACK(0x80)` / `NACK(0x81)` / `Status(0x91)` / `PidData(0x92)`；
NACK 原因：`WrongState` / `BadChecksum` / `InvalidParam` / `UnknownRequest`。
状态机：`Uninit →(Init)→ Ready →(速度)→ Running`，`Reset` 随时回 `Uninit`。

负载长度是**精确值**：多一字节少一字节、未知请求、枚举/数值越界都会回 NACK；
`LEN > 16` 的帧会被解析器丢弃并立即重同步。多字节整数一律**小端**（`struct "<"` /
`to_le_bytes`）。字段级定义、校验规则与受理状态以仓库根目录的 `protocol/`
（`smart-car-protocol` crate）为准——模块文档里就是完整协议表，固件与上位机
`sg2002-upper` 共用这一份实现；Python 侧镜像为 `tools/src/protocol.py`。

```bash
# Rust 协议回归：黄金字节 / 编解码 / 错误路径
(cd ../protocol && cargo test)

# Python 镜像回归
python3 tools/tests/test_protocol.py
```

`Status(0x91)` 为 7 字节：`sys(1) + rpm0(2) + rpm1(2) + distActive(1) + distResult(1)`。
`distActive` = 闭环是否运行中，`distResult` = 0 无/运行中、1 到达目标；
新 Move/Rotate 与 `Init`/`Reset` 会清零 `distResult`。

### 闭环运动（Move / Rotate）

- `Move` 的 target 单位是 mm，`Rotate` 是 0.1°（90° 传 `900`）；两者都用
  `dir` 选方向，target 必须 > 0，speed 取 1~100，超范围回 `InvalidParam`。
- 固件按轮径 D=62mm、轴距 L=160mm、PPR=4680 把物理量换算成编码器计数
  （见 `service/odometry.rs`）：直行取左右轮平均，原地转向取反向两轮的较大值，
  到达目标后 brake 双轮并置 `distResult=1`。
- 主机可用 `Heartbeat` 轮询 `Status` 等待完成（约 100ms 内可见），
  断链保护仍然有效。

## 上位机试车

> 下面两个工具目前只走 BLE，烧录时要选 `ble` feature：
> `cargo run --release --no-default-features --features ble`

```bash
pip install bleak
python tools/play.py
```

- 按住 `W` 前进、`S` 刹车/倒车、`A`/`D` 转向（静止时原地打转）、`Q` 退出。
- 不带参数，所有链路/电机/手感参数都在 `tools/src/config.py` 里改。
- 程序先连 BLE（`ESP32C3-CAR`）、发 `Init` 对拍，然后 20Hz 下发速度、
  5Hz 发心跳（`Heartbeat`→`Status`），终端 HUD 显示转速和系统状态。
- 强杀后蓝牙连不上：`bluetoothctl disconnect <ADDRESS>` 后重试。

闭环走距离 / 转角度（同样连 BLE，默认速度在 `tools/src/config.py`）：

```bash
python tools/move.py move 500                # 前进 500mm
python tools/move.py move -300 --speed 50    # 后退 300mm
python tools/move.py rotate 90               # 原地左转 90°
python tools/move.py rotate -90              # 原地右转 90°
python tools/move.py demo                    # 500mm → 左转 90° → 500mm
```

## 构建与烧录

命令链路是编译期二选一的 feature（默认 `uart`），两个同时开或都不开会直接编译报错：

```bash
cargo build --release        # 生产：UART0 链路（默认），目标已在 .cargo/config.toml 配好
cargo run --release          # 烧录 + 串口监视（espflash）

cargo build --release --no-default-features --features ble   # 测试：BLE 链路
cargo run --release --no-default-features --features ble     # 烧录 + 串口监视
```

只有 `ble` 会链接 BLE 栈（esp-radio / trouble-host）并预留 72KB 堆；
`uart` 版完全不含这些依赖（本机 release 构建对比：flash 少约 200KB、RAM 少约 86KB）。

defmt 日志只走 USB_SERIAL_JTAG，不占用 UART0 命令通道。
日志等级经 `DEFMT_LOG` 环境变量控制（默认 `info`）。

## 安全机制

- 断链保护（仅 `ble`）：BLE 断开即 coast 回 `Ready`（闭环目标一并取消）。
- 没有指令超时：链路未断但上位机卡死时，最后的速度指令不会自动失效，
  需要上位机自行停止（BLE 链路只用于测试；UART 同样不做自动停车）。
