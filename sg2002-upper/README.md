ip addr add 192.168.1.2/24 dev eth0
./smartcar --bind 192.168.1.2     # 整合：视觉追踪 + 网页（手动/自动切换）
./webctl --bind 192.168.1.2       # 只遥控（MJPG 预览）

# 日志走 stderr（时间戳/级别/模块），默认 info；级别用 SMARTCAR_LOG 控制：
SMARTCAR_LOG=debug ./smartcar --bind 192.168.1.2           # 全部 debug（含逐帧编码耗时）
SMARTCAR_LOG=warn,hwjpeg=debug ./smartcar --bind 192.168.1.2  # 默认 warn，只看编码耗时


我写了一个 sg2002-upper/cvimpi-rs，请使用它代替 sg2002-upper/csrc 作为 JPEG 的编解码支持，我通过网口连接了 sg2002，对方 IP 为 192.168.1.2 ，你可以通过 ssh进行测试
