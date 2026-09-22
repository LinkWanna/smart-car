#!/bin/sh
# =============================================================================
# init_ap.sh —— 立即启动 AP
#
# 用法:
#   ./init_ap.sh
#
# 只写 /etc/hostapd.conf 和 /etc/dnsmasq.ap.conf 两个配置文件，不改动系统其他文件。
# 开机自启请部署 script/S98apstart 到 /etc/init.d/。
# 注意: 会断开 AP 接口（默认 wlan0）上已有的 STA 连接
# =============================================================================

AP_IFACE="${AP_IFACE:-wlan0}"          # AP 接口（给手机/控制器连）
AP_IP="${AP_IP:-192.168.4.1}"
NETMASK="${NETMASK:-255.255.255.0}"
DHCP_START="${DHCP_START:-192.168.4.100}"
DHCP_END="${DHCP_END:-192.168.4.200}"
CHANNEL="${CHANNEL:-6}"

# ── 1. 基于 wlan0 MAC 生成唯一 SSID ──
MAC_SUFFIX=$(cat /sys/class/net/${AP_IFACE}/address 2>/dev/null | tr -d ':' | tail -c 6)
[ -z "$MAC_SUFFIX" ] && MAC_SUFFIX="123456"
RANDOM_ID=$((16#${MAC_SUFFIX} % 9999999 + 1))
SSID="chenlong-robot-${RANDOM_ID}"

# ── 2. hostapd 配置（开放热点；如需 WPA2 见注释）──
cat > /etc/hostapd.conf <<EOF
interface=${AP_IFACE}
driver=nl80211
ssid=${SSID}
hw_mode=g
channel=${CHANNEL}
auth_algs=1
EOF

# ── 3. DHCP 配置（dnsmasq；本板无 udhcpd）──
cat > /etc/dnsmasq.ap.conf <<EOF
interface=${AP_IFACE}
bind-interfaces
dhcp-range=${DHCP_START},${DHCP_END},${NETMASK},12h
dhcp-option=3,${AP_IP}
dhcp-option=6,${AP_IP}
EOF

# ── 4. 立即启动 AP ──
echo "── 立即启动 AP（${AP_IFACE} 当前 STA 连接将被断开）──"
wpa_cli -p /var/run/wpa_supplicant -i ${AP_IFACE} terminate 2>/dev/null
killall hostapd dnsmasq udhcpd 2>/dev/null
sleep 1
dhcpcd -k ${AP_IFACE} 2>/dev/null                # 释放 dhcpcd 可能持有的租约
ip addr flush dev ${AP_IFACE} 2>/dev/null        # 清掉遗留的链路本地/DHCP 地址
ifconfig ${AP_IFACE} ${AP_IP} netmask ${NETMASK} up
dnsmasq --conf-file=/etc/dnsmasq.ap.conf --pid-file=/var/run/dnsmasq.ap.pid &
hostapd -B /etc/hostapd.conf
echo "✅ 热点已启动: ${SSID}"
echo "连上热点后浏览器访问: http://${AP_IP}"
