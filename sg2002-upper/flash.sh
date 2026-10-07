#!/usr/bin/env bash
# =============================================================================
# flash.sh —— 编译并部署 SG2002 上位机（本机 ⇄ 网口直连 root@192.168.1.2）
#
# 用法:
#   ./flash.sh
#
# 做三件事:
#   1. 交叉编译 src/bin 全部 [[bin]]（release / riscv64gc-unknown-linux-musl）；
#   2. 部署可执行文件与模型到板端 /root（模型取 assets/*.cvimodel）；
#   3. 部署启动脚本与 AP 配置:
#        scripts/init.d/S97uart1mux  → /etc/init.d/S97uart1mux（UART1 复用）
#        scripts/init.d/S98apstart   → /etc/init.d/S98apstart （AP 热点）
#        scripts/init.d/S99smartcar  → /etc/init.d/S99smartcar（smartcar 上位机）
#        scripts/smartcar-ap.conf    → /etc/smartcar-ap.conf （AP 端点配置）
#      顺带补 libsys/libvenc 符号链接（硬件 JPEG），并清理板端旧版脚本
#      （/etc/init.d/S99uart1mux、/root/scripts/*）。
#
# 前置:
#   板子已开机、eth0 为 192.168.1.2，且本机已配置免密登录 root@192.168.1.2。
# =============================================================================
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

BOARD=root@192.168.1.2          # 板端 ssh 目标（网口直连）
DEST=/root                      # 可执行文件/模型安装目录
TARGET=riscv64gc-unknown-linux-musl
SSH_OPTS=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new)

if [ $# -gt 0 ]; then
    echo "用法: ./flash.sh（无参数）" >&2
    exit 2
fi

fail() { echo "✗ $*" >&2; exit 1; }

# --- 部署清单 -----------------------------------------------------------------

# 待部署的 bin：从 Cargo.toml 的 [[bin]] 读，避免和配置漂移
mapfile -t BINS < <(awk -F'=' '
    /^\[\[bin\]\]/ { want = 1; next }
    want && $1 ~ /^name[ \t]*$/ { gsub(/[" \t]/, "", $2); print $2; want = 0 }
' Cargo.toml)
[ "${#BINS[@]}" -gt 0 ] || fail "Cargo.toml 里没有找到 [[bin]] 目标"

BIN_FILES=()
for bin in "${BINS[@]}"; do
    BIN_FILES+=("target/$TARGET/release/$bin")
done

# 模型（smartcar 自动查找 /root/*.cvimodel）
MODELS=()
for f in assets/*.cvimodel; do
    [ -f "$f" ] && MODELS+=("$f")
done
[ "${#MODELS[@]}" -gt 0 ] || fail "assets/ 下没有 .cvimodel 模型文件"

# 启动脚本（放 scripts/init.d，部署到 /etc/init.d；全部自包含）
INIT_FILES=(scripts/init.d/S97uart1mux scripts/init.d/S98apstart scripts/init.d/S99smartcar)
for f in "${INIT_FILES[@]}"; do
    [ -f "$f" ] || fail "缺少启动脚本 $f"
done

# AP 端点配置（部署到 /etc/smartcar-ap.conf）
AP_CONF=scripts/smartcar-ap.conf
[ -f "$AP_CONF" ] || fail "缺少 AP 配置 $AP_CONF"

# --- 编译 ---------------------------------------------------------------------

echo "== 编译 ${BINS[*]}（release / $TARGET）"
cargo build --release --target "$TARGET" --bins

for f in "${BIN_FILES[@]}"; do
    [ -f "$f" ] || fail "缺少编译产物 $f"
done

# --- 板端可达性 ---------------------------------------------------------------

if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$BOARD" true >/dev/null 2>&1; then
    cat >&2 <<'EOF'
✗ 无法 SSH 登录板子（root@192.168.1.2）

自查:
  1) 本机网口 192.168.1.1/24、网线直连:  ip -brief addr
  2) 板子在线:                        ping -c1 192.168.1.2
  3) 本机免密登录:                    ssh-copy-id root@192.168.1.2
EOF
    exit 1
fi
echo "== 板端登录: $BOARD"

# --- 传输 ---------------------------------------------------------------------

# 临时目录放在 /root（和 /etc 同一个 ext4 根分区）：最后 mv 是同分区 rename，
# 覆盖正在运行的 /root/smartcar 时不会踩 ETXTBSY。
REMOTE_TMP=$(ssh "${SSH_OPTS[@]}" "$BOARD" "mktemp -d '$DEST/.flash.XXXXXX'")
cleanup() {
    [ -n "${REMOTE_TMP:-}" ] &&
        ssh "${SSH_OPTS[@]}" "$BOARD" "rm -rf '$REMOTE_TMP'" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== 传输到 $BOARD:$REMOTE_TMP"
scp -q "${SSH_OPTS[@]}" \
    "${BIN_FILES[@]}" "${MODELS[@]}" "${INIT_FILES[@]}" "$AP_CONF" \
    "$BOARD:$REMOTE_TMP/"

# --- 板端安装（先搬到目标目录，再清理临时目录）--------------------------------

MOVE=""
for bin in "${BINS[@]}"; do
    MOVE+="mv -f '$REMOTE_TMP/$bin' '$DEST/'"$'\n'
done
for f in "${MODELS[@]}"; do
    MOVE+="mv -f '$REMOTE_TMP/$(basename "$f")' '$DEST/'"$'\n'
done
for f in "${INIT_FILES[@]}"; do
    MOVE+="mv -f '$REMOTE_TMP/$(basename "$f")' /etc/init.d/"$'\n'
done
MOVE+="mv -f '$REMOTE_TMP/smartcar-ap.conf' /etc/smartcar-ap.conf"$'\n'

CHMOD755=""
for bin in "${BINS[@]}"; do
    CHMOD755+="'$DEST/$bin' "
done
for f in "${INIT_FILES[@]}"; do
    CHMOD755+="'/etc/init.d/$(basename "$f")' "
done

CHMOD644=""
for f in "${MODELS[@]}"; do
    CHMOD644+="'$DEST/$(basename "$f")' "
done
CHMOD644+="'/etc/smartcar-ap.conf' "

ssh "${SSH_OPTS[@]}" "$BOARD" "
    set -e
    mkdir -p '$DEST'
    $MOVE
    chmod 755 $CHMOD755
    chmod 644 $CHMOD644
    rm -rf '$REMOTE_TMP'

    # 清理旧版脚本（统一为 /etc/init.d 自包含脚本）
    rm -f /etc/init.d/S99uart1mux \\
          /root/scripts/init_ap.sh /root/scripts/pinmux.sh /root/scripts/dhcp-lease.sh
    rmdir /root/scripts 2>/dev/null || true

    # 厂商 MMF 库的符号链接（硬件 JPEG：libvenc 按名字依赖 libsys）
    for lib in libsys.so libvenc.so; do
        if [ -e \"/mnt/system/usr/lib/\$lib\" ] && [ ! -e \"/usr/lib/\$lib\" ]; then
            ln -sf \"/mnt/system/usr/lib/\$lib\" \"/usr/lib/\$lib\" && echo \"  已链接 \$lib → /usr/lib（硬件 JPEG 用）\"
        fi
    done
    true
"
REMOTE_TMP=""       # 远程已自行清理，EXIT trap 不用再跑

# --- 校验 ---------------------------------------------------------------------

echo "== 板端部署结果:"
ssh "${SSH_OPTS[@]}" "$BOARD" "ls -l $CHMOD755 $CHMOD644"

cat <<EOF

✅ 部署完成: $BOARD

开机自启（已安装，重启后按顺序执行）:
    S97uart1mux  → UART1 → A18/A19（ESP32-C3 串口）
    S98apstart   → 建 AP 热点（读 /etc/smartcar-ap.conf；Nano-E 自动跳过）
    S99smartcar  → 启动 /root/smartcar（日志 /root/smartcar.log）

立即生效（不重启）:
    ssh $BOARD '/etc/init.d/S97uart1mux start'
    ssh $BOARD '/etc/init.d/S99smartcar start'
    ssh $BOARD '/etc/init.d/S99smartcar status'
EOF
