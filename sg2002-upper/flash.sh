#!/usr/bin/env bash
# =============================================================================
# flash.sh —— 编译 SG2002 上位机（src/bin 三个 bin）并部署到板端 /root
#
#   网口直连  本机 192.168.1.1/24 ⇄ 板端 192.168.1.2
#   串口控制台  /dev/ttyACM0  115200（--serial-ip 兜底时用）
#   交叉编译配置见 .cargo/config.toml（linker = riscv64-linux-musl-gcc）
#
# 用法:
#   ./flash.sh              # release 交叉编译 → scp 到 root@192.168.1.2:/root
#   ./flash.sh -n           # 跳过编译，只部署已有产物
#   ./flash.sh --serial-ip  # ssh 不通时，先经串口下发 ip addr add ... 再重试
#
# 其它选项: -d/--dest <目录>（默认 /root）、-s/--serial <设备>、-t/--target <triple>、
#           --linker <路径>（覆盖 .cargo/config.toml）
# 板端登录目标固定在下面的 BOARD，不提供选项。
# =============================================================================
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

# --- 配置 -------------------------------------------------------------------
BOARD=root@192.168.1.2   # 板端 ssh 目标；板端禁用 root 登录时改成 linkwanna@192.168.1.2
DEST=/root               # 板端安装目录
SERIAL=/dev/ttyACM0      # 板端串口控制台
TARGET=riscv64gc-unknown-linux-musl
LINKER=                  # 非空则覆盖 .cargo/config.toml 的 linker

DO_BUILD=1
SERIAL_IP=0

usage() { sed -n '3,/^# ====/p' "$0" | sed 's/^# \{0,1\}//' | head -n -1; }

while [ $# -gt 0 ]; do
    case "$1" in
        -d|--dest)     DEST=$2; shift 2 ;;
        -s|--serial)   SERIAL=$2; shift 2 ;;
        -t|--target)   TARGET=$2; shift 2 ;;
        --linker)      LINKER=$2; shift 2 ;;
        -n|--no-build) DO_BUILD=0; shift ;;
        --serial-ip)   SERIAL_IP=1; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "flash.sh: 未知参数 $1（-h 查看用法）" >&2; exit 2 ;;
    esac
done

# --- 链接器（默认由 .cargo/config.toml 指定，--linker 可覆盖）----------------
if [ -n "$LINKER" ]; then
    LINKER_VAR="CARGO_TARGET_$(printf '%s' "$TARGET" | tr 'a-z-' 'A-Z_')_LINKER"
    export "$LINKER_VAR=$LINKER"
    echo "== 链接器: $LINKER_VAR=$LINKER"
fi

# --- 待部署的 bin：从 Cargo.toml 的 [[bin]] 读，避免和配置漂移 ----------------
mapfile -t BINS < <(awk -F'=' '
    /^\[\[bin\]\]/ { want = 1; next }
    want && $1 ~ /^name[ \t]*$/ { gsub(/[" \t]/, "", $2); print $2; want = 0 }
' Cargo.toml)
[ "${#BINS[@]}" -gt 0 ] ||
    { echo "flash.sh: Cargo.toml 里没有找到 [[bin]] 目标" >&2; exit 1; }

# --- 编译 -------------------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
    echo "== 编译 ${BINS[*]}（release / $TARGET）"
    cargo build --release --target "$TARGET" --bins
else
    echo "== 跳过编译（-n）"
fi

OUT="target/$TARGET/release"
FILES=()
for bin in "${BINS[@]}"; do
    [ -f "$OUT/$bin" ] ||
        { echo "flash.sh: 缺少产物 $OUT/$bin（先去掉 -n 编译一次）" >&2; exit 1; }
    FILES+=("$OUT/$bin")
done

# --- 板端可达性 -------------------------------------------------------------
SSH_OPTS=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new)
ssh_ok() { ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$BOARD" true >/dev/null 2>&1; }

BOARD_HOST=${BOARD#*@}

# 兜底：板端 eth0 没配 IP 时（README 里的手工步骤），经串口下发一次
serial_set_ip() {
    [ -c "$SERIAL" ] || { echo "flash.sh: 串口 $SERIAL 不存在" >&2; return 1; }
    echo "== 经串口 $SERIAL 下发: ip addr add $BOARD_HOST/24 dev eth0"
    stty -F "$SERIAL" 115200 raw -echo 2>/dev/null || {
        echo "flash.sh: 打不开 $SERIAL（权限不足？试 sudo 或加入 uucp 组）" >&2
        return 1
    }
    {
        printf '\r'
        sleep 1
        printf 'ip addr add %s/24 dev eth0\r' "$BOARD_HOST"
        sleep 2
    } >"$SERIAL"
    sleep 2
}

if ! ssh_ok && [ "$SERIAL_IP" = 1 ]; then
    serial_set_ip || true
fi

if ! ssh_ok; then
    cat >&2 <<EOF
✗ 无法 SSH 登录板子（$BOARD）

自查:
  1) 本机网口（应为 192.168.1.1/24）: ip -brief addr
  2) 板子是否在线:                    ping -c1 $BOARD_HOST
  3) 板端 eth0 是否配了 IP，可进串口控制台确认:
       picocom -b 115200 $SERIAL
       # 板端 shell 里执行（每次冷启动后可能需要）:
       ip addr add $BOARD_HOST/24 dev eth0
     或加 --serial-ip 让本脚本经串口自动下发这一条。
EOF
    exit 1
fi
echo "== 板端登录: $BOARD"

# --- 传输：先 scp 到板端临时目录，再（必要时 sudo）原子替换到 DEST ----------
REMOTE_TMP=$(ssh "${SSH_OPTS[@]}" "$BOARD" 'mktemp -d')
echo "== 传输到 $BOARD:$REMOTE_TMP"
scp -q "${SSH_OPTS[@]}" "${FILES[@]}" "$BOARD:$REMOTE_TMP/"

if [ "$(ssh "${SSH_OPTS[@]}" "$BOARD" 'id -u')" = 0 ]; then
    SUDO=""
    TTY_ARGS=()
else
    SUDO="sudo"          # 非 root 用户：借一个 tty 让 sudo 能提示密码
    TTY_ARGS=(-t)
fi

# 注意：不要用 printf 多参数拼列表（格式串会循环复用），逐项拼
CHMOD_LIST=""
for bin in "${BINS[@]}"; do
    CHMOD_LIST="$CHMOD_LIST'$DEST/$bin' "
done

# 先搬到 DEST 再删临时目录：即使后续 chmod 失败也不留垃圾；
# 顺带补厂商 MMF 库的符号链接（硬件 JPEG：libvenc 按名字依赖 libsys）
ssh "${SSH_OPTS[@]}" "${TTY_ARGS[@]}" "$BOARD" "
    set -e
    mkdir -p '$DEST'
    $SUDO mv -f '$REMOTE_TMP'/* '$DEST'/
    rm -rf '$REMOTE_TMP'
    $SUDO chmod 755 $CHMOD_LIST
    for lib in libsys.so libvenc.so; do
        if [ -e \"/mnt/system/usr/lib/\$lib\" ] && [ ! -e \"/usr/lib/\$lib\" ]; then
            $SUDO ln -sf \"/mnt/system/usr/lib/\$lib\" \"/usr/lib/\$lib\" \
                && echo \"  已链接 \$lib → /usr/lib（硬件编码用）\"
        fi
    done
"

# --- 校验 -------------------------------------------------------------------
echo "== 板端 $DEST:"
ssh "${SSH_OPTS[@]}" "$BOARD" "ls -l $CHMOD_LIST"

cat <<EOF

✅ 部署完成: $BOARD:$DEST → ${BINS[*]}

板端运行（网口直连，本机 192.168.1.1）:
    ssh $BOARD
    ip addr add $BOARD_HOST/24 dev eth0     # 若还没配
    $DEST/webctl --bind $BOARD_HOST
EOF
