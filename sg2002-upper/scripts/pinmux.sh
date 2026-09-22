#!/bin/sh
# =============================================================================
# pinmux.sh —— 把 UART1 复用到 GPIOA18/A19（ESP32-C3 下位机接线处）
#
# 背景：LicheeRV Nano 默认 pinmux 是
#   A18/A19 = UART1 CTS/RTS，A28/A29 = UART1 TX/RX；
# 本车把 ESP32-C3 接在 A18/A19 上，所以按 Sipeed wiki（外设/UART）写寄存器：
#   A18 -> UART1 RX (0x6)
#   A19 -> UART1 TX (0x6)
#   A28 -> UART2 TX (0x2)   # 让出 UART1 默认引脚，避免 RX 并联两个输入
#   A29 -> UART2 RX (0x2)
#
# 寄存器改动掉电不保存：开机自启可把本脚本放到 /etc/init.d/S99uart1mux
# =============================================================================

devmem 0x03001068 32 0x6   # GPIOA 18 UART1 RX
devmem 0x03001064 32 0x6   # GPIOA 19 UART1 TX
devmem 0x03001070 32 0x2   # GPIOA 28 UART2 TX
devmem 0x03001074 32 0x2   # GPIOA 29 UART2 RX

echo "✅ UART1 -> A18(RX)/A19(TX)，A28/A29 -> UART2"
