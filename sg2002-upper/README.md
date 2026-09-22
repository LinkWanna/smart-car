ip addr add 192.168.1.2/24 dev eth0
./smartcar --bind 192.168.1.2     # 整合：视觉追踪 + 网页（手动/自动切换）
./webctl --bind 192.168.1.2       # 只遥控（MJPG 预览）
