#!/usr/bin/env bash
#
# build_ssc325_modules.sh
#
# 作用：为 SigmaStar SSC325（OpenIPC "infinity6" 方案 / Linux 4.9.84）交叉编译
#       drv/src/drv_ms_cus_*.c 中的 sensor 驱动，输出可直接 insmod 的 .ko。
#
# ---------------------------------------------------------------------------
# 背景：为什么需要这个脚本
# ---------------------------------------------------------------------------
# 本仓库是 SigmaStar 原厂 SDK 的一份「摘录」，不是完整工程：
#   makefile:34            include $(SENSOR_MK)          -> sensor.mk 不存在
#   drv/Makefile:3-4       PROJ_ROOT ?= ../../../project -> 目录树不存在
# 缺失组件还包括 configs/current.configs、release/*/toolchain.mk、kbuild/，以及
# cam_os_wrapper / sensorif / drv_sensor 等头文件。原厂 SDK 需签 NDA，无法补齐。
#
# 但目标设备跑的是 OpenIPC 开源方案，其内核与 sensor 驱动框架都是公开的，且与
# 本仓库使用同一套 SigmaStar 接口，可以等价替代：
#   内核     openipc/linux    分支 sigmastar-infinity6  (Linux 4.9.84)
#   头文件   openipc/sensors  sigmastar/infinity6/include/
#            -> sensor_i2c_api.h / drv_sensor.h / drv_sensor_common.h
#               (drv_sensor.h 只是薄壳，内部 #include <drv_ms_cus_sensor.h>)
#   工具链   openipc/firmware  release tag "toolchain"
#            -> toolchain.sigmastar-infinity6.tgz  (arm-openipc-linux-musleabihf)
#   板级配置 br-ext-chip-sigmastar/board/infinity6/infinity6-ssc009a.config
#
# 以上取值全部来自 OpenIPC 官方 ssc325_lite_defconfig 与 general/external.mk，
# 已逐项校验可达（HTTP 200）。注意 release tag 是 "toolchain"，资产名才带前缀。
#
# 该板级配置中 CONFIG_MODVERSIONS 与 CONFIG_MODULE_SIG 均未开启，所以只需
# `make modules_prepare`，不必完整编译内核（省 10~20 分钟）。
#
# ---------------------------------------------------------------------------
# 运行环境
# ---------------------------------------------------------------------------
# 必须 Linux x86_64（WSL2 / Ubuntu / GitHub Actions runner 均可）。
# Windows 原生不行：工具链是 Linux ELF 二进制，内核模块编译强依赖 Kbuild + make。
# 依赖：make gcc perl tar findutils bc flex bison + (curl 或 wget)
#   Ubuntu: sudo apt-get install -y build-essential bc flex bison perl curl
#
# ---------------------------------------------------------------------------
# 用法
# ---------------------------------------------------------------------------
#   ./build/build_ssc325_modules.sh                 # 编译全部 sensor
#   SENSORS="sc2231_MIPI imx291_MIPI" ./build/build_ssc325_modules.sh
#   JOBS=8 ./build/build_ssc325_modules.sh
#   FULL_KERNEL=1 ./build/build_ssc325_modules.sh   # 完整编译内核以生成 Module.symvers
#
set -euo pipefail

# ------------------------------ 远程资源（已校验） ---------------------------
TOOLCHAIN_URL="https://github.com/openipc/firmware/releases/download/toolchain/toolchain.sigmastar-infinity6.tgz"
KERNEL_BRANCH="sigmastar-infinity6"
KERNEL_URL="https://github.com/openipc/linux/archive/refs/heads/${KERNEL_BRANCH}.tar.gz"
SENSORS_URL="https://github.com/openipc/sensors/archive/refs/heads/master.tar.gz"
BOARD_CFG_URL="https://raw.githubusercontent.com/openipc/firmware/master/br-ext-chip-sigmastar/board/infinity6/infinity6-ssc009a.config"

CROSS_COMPILE_NAME="arm-openipc-linux-musleabihf-"
ARCH="arm"
KERNEL_VERSION="4.9.84"

# 需要 SDK 私有头 srcfg_drv.h（内含 tSensorConfig 联合体与 SNR_MCLK_* 位域定义），
# 无法从公开来源重建；位域错位会写错寄存器，所以不猜，直接跳过。
# 设备端也不需要自建：OpenIPC 固件自带 sensor_config.ko（load_sigmastar 会 insmod）。
DEFAULT_SKIP_LIST="srcfg_drv"

# ------------------------------ 路径与开关 -----------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="${WORK:-$REPO_ROOT/build/work}"
OUT="${OUT:-$REPO_ROOT/build/out}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"
SENSORS="${SENSORS:-}"
SKIP_LIST="${SKIP_LIST:-$DEFAULT_SKIP_LIST}"
FULL_KERNEL="${FULL_KERNEL:-0}"

SRC_DIR="$REPO_ROOT/drv/src"
PUB_DIR="$REPO_ROOT/drv/pub"

log()  { printf '\033[1;36m[build]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn ]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

# ------------------------------ 前置检查 -------------------------------------
log "检查主机环境 ..."
if [ "$(uname -s)" != "Linux" ]; then
    die "必须在 Linux 上运行（当前 $(uname -s)）。工具链是 Linux x86_64 ELF，Windows 原生无法执行。"
fi
[ "$(uname -m)" = "x86_64" ] || warn "主机架构为 $(uname -m)，预期 x86_64，工具链可能无法执行。"

need() { command -v "$1" >/dev/null 2>&1 || die "缺少命令 '$1'。"; }
for c in make tar find perl gcc; do need "$c"; done
command -v curl >/dev/null 2>&1 || command -v wget >/dev/null 2>&1 || die "需要 curl 或 wget。"
for c in bc flex bison; do
    command -v "$c" >/dev/null 2>&1 || warn "未找到 '$c'，内核配置阶段可能失败。"
done

mkdir -p "$WORK" "$OUT"

fetch() { # fetch <url> <outfile>
    local url="$1" out="$2"
    if [ -s "$out" ]; then log "已缓存，跳过下载: $(basename "$out")"; return 0; fi
    log "下载 $(basename "$out") ..."
    if command -v curl >/dev/null 2>&1; then
        curl -fL --retry 3 --connect-timeout 30 -o "$out.part" "$url"
    else
        wget -c -O "$out.part" "$url"
    fi
    mv "$out.part" "$out"
}

# ------------------------------ 1. 工具链 ------------------------------------
TOOLCHAIN_DIR="$WORK/toolchain"
if ! find "$TOOLCHAIN_DIR" -type f -name "${CROSS_COMPILE_NAME}gcc" 2>/dev/null | grep -q .; then
    fetch "$TOOLCHAIN_URL" "$WORK/toolchain.tgz"
    log "解压工具链 ..."
    mkdir -p "$TOOLCHAIN_DIR"
    tar -xzf "$WORK/toolchain.tgz" -C "$TOOLCHAIN_DIR"
fi
CROSS_GCC="$(find "$TOOLCHAIN_DIR" -type f -name "${CROSS_COMPILE_NAME}gcc" | head -n1)"
[ -n "$CROSS_GCC" ] || die "未在 $TOOLCHAIN_DIR 中找到 ${CROSS_COMPILE_NAME}gcc"
export PATH="$(dirname "$CROSS_GCC"):$PATH"
log "交叉编译器: ${CROSS_GCC#"$WORK"/}"
"${CROSS_COMPILE_NAME}gcc" --version | head -n1

# ------------------------------ 2. 内核源码 ----------------------------------
KSRC="$WORK/linux-$KERNEL_BRANCH"
if [ ! -d "$KSRC" ]; then
    fetch "$KERNEL_URL" "$WORK/kernel.tar.gz"
    log "解压内核源码 ..."
    tar -xzf "$WORK/kernel.tar.gz" -C "$WORK"
fi
[ -d "$KSRC/drivers/sstar" ] || die "内核源码缺少 drivers/sstar，路径异常: $KSRC"

# ------------------------------ 3. sensor 头文件 -----------------------------
SENSORS_REPO="$WORK/sensors-master"
if [ ! -d "$SENSORS_REPO" ]; then
    fetch "$SENSORS_URL" "$WORK/sensors.tar.gz"
    log "解压 openipc/sensors ..."
    tar -xzf "$WORK/sensors.tar.gz" -C "$WORK"
fi
SENSORS_INC="$SENSORS_REPO/sigmastar/infinity6/include"
[ -f "$SENSORS_INC/drv_sensor.h" ]        || die "缺少 drv_sensor.h: $SENSORS_INC"
[ -f "$SENSORS_INC/sensor_i2c_api.h" ]    || die "缺少 sensor_i2c_api.h: $SENSORS_INC"
[ -f "$PUB_DIR/drv_ms_cus_sensor.h" ]     || die "缺少仓库自带 drv_ms_cus_sensor.h: $PUB_DIR"

# ------------------------------ 4. 准备内核 ----------------------------------
log "写入板级配置 (.config = infinity6-ssc009a.config) ..."
[ -f "$WORK/board.config" ] || fetch "$BOARD_CFG_URL" "$WORK/board.config"
cp "$WORK/board.config" "$KSRC/.config"

log "olddefconfig ..."
if ! make -C "$KSRC" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE_NAME" olddefconfig \
        >"$WORK/olddefconfig.log" 2>&1; then
    tail -n 40 "$WORK/olddefconfig.log" >&2
    die "olddefconfig 失败"
fi
grep -q '^CONFIG_MODULES=y' "$KSRC/.config" || die "内核 .config 未开启 CONFIG_MODULES。"

# GCC 10+ 默认 -fno-common，会让 4.9 内核的 scripts/dtc 等宿主工具链接失败
# （典型症状：multiple definition of `yylloc'）。先正常试，失败再加 -fcommon 重试。
PREPARE_LOG="$WORK/modules_prepare.log"
log "modules_prepare（首次约 2~5 分钟）..."
if ! make -C "$KSRC" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE_NAME" \
        modules_prepare >"$PREPARE_LOG" 2>&1; then
    warn "modules_prepare 失败，改用 HOSTCFLAGS=-fcommon 重试（老内核 + GCC10+ 已知问题）..."
    if ! make -C "$KSRC" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE_NAME" \
              HOSTCFLAGS="-fcommon" modules_prepare >>"$PREPARE_LOG" 2>&1; then
        tail -n 60 "$PREPARE_LOG" >&2
        die "modules_prepare 仍然失败，完整日志: $PREPARE_LOG"
    fi
fi

if [ "$FULL_KERNEL" = "1" ]; then
    log "FULL_KERNEL=1：完整编译内核以生成 Module.symvers（较慢）..."
    make -C "$KSRC" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE_NAME" \
         HOSTCFLAGS="-fcommon" -j"$JOBS" modules >"$WORK/kernel_modules.log" 2>&1 \
        || warn "内核 modules 编译未完全成功（见 $WORK/kernel_modules.log），通常不影响本仓库驱动"
fi

# ------------------------------ 5. 编译参数 ----------------------------------
# 裸 `M=` 编译且没有 Module.symvers 时，modpost 会把所有内核导出符号判为 undefined
# 而报错。本板 CONFIG_MODVERSIONS 未开启，符号在 insmod 时按名字解析，所以用
# KBUILD_MODPOST_WARN 降级成警告即可。
MODPOST_WARN=1

KINC=()
for d in "drivers/sstar/include" "drivers/sstar/cam_os_wrapper/pub"; do
    [ -d "$KSRC/$d" ] && KINC+=("-I$KSRC/$d")
done
KDEF=('-DSENSOR_MODULE_VERSION="infinity6"' '-DSENSOR_VERSION="infinity6"')

# 两种头文件优先级：
#   A 仓库自带 drv_ms_cus_sensor.h 优先 —— 驱动源码与它来自同一份 SDK 摘录，默认用它
#   B OpenIPC 的 drv_ms_cus_sensor.h 优先 —— A 编译失败时重试，保证整套头文件版本一致
#     （OpenIPC 版的 drv_sensor_common.h 可能依赖它自己的 drv_ms_cus_sensor.h）
CCFLAGS_A=("-I$PUB_DIR" "-I$SENSORS_INC" ${KINC[@]+"${KINC[@]}"} "${KDEF[@]}")
CCFLAGS_B=("-I$SENSORS_INC" "-I$PUB_DIR" ${KINC[@]+"${KINC[@]}"} "${KDEF[@]}")

log "头文件搜索顺序 A（默认）:"
printf '        %s\n' "${CCFLAGS_A[@]}"

# ------------------------------ 6. 逐个编译 ----------------------------------
# 对齐设备端 load_sigmastar 的加载约定：
#   insmod $MODULE/sensor_${SENSOR}_${IFACE:-mipi}.ko chmap=1
# 也就是 /lib/modules/4.9.84/sigmastar/sensor_<型号>_<mipi|parl>.ko，
# 模块内部名 = .ko 文件名（脚本用 `ls /sys/module | grep $SENSOR` 做 rmmod）。
# 注意不能只看文件名：drv_ms_cus_sc2235.c / sc2310.c / sc3235.c / sc2238H.c /
# ar0237_RGBIR.c 实为 PARL，而 drv_ms_cus_opn008.c 实为 MIPI —— 以源码里的
# SENSOR_IFBUS_TYPE 为准。`-` 规范成 `_`，避免 SC4238_MIPI-hdr 这类名字出问题。
sensor_bus_type() { # 输出 mipi 或 parl
    if grep -qE '^#[[:space:]]*define[[:space:]]+SENSOR_IFBUS_TYPE[[:space:]]+CUS_SENIF_BUS_PARL' "$1"; then
        echo parl
    else
        echo mipi
    fi
}

module_name_for() {
    local base suffix mod bus
    base="$(basename "$1" .c)"
    suffix="${base#drv_ms_cus_}"
    mod="$(printf 'sensor_%s' "$suffix" | tr '[:upper:]-' '[:lower:]_')"
    case "$mod" in
        *_mipi*|*_parl*) ;;                     # 名字里已含总线类型（含 _mipi_earlyinit 之类）
        *) mod="${mod}_$(sensor_bus_type "$1")" ;;
    esac
    echo "$mod"
}

should_skip() { local s="$1" k; for k in $SKIP_LIST; do [ "$k" = "$s" ] && return 0; done; return 1; }
wanted()      { [ -z "$SENSORS" ] && return 0; local s="$1" k; for k in $SENSORS; do [ "$k" = "$s" ] && return 0; done; return 1; }

try_build() { # try_build <src> <mod> <A|B>  成功返回 0
    local src="$1" mod="$2" mode="$3"
    local -a flags
    local mdir
    if [ "$mode" = "A" ]; then flags=("${CCFLAGS_A[@]}"); else flags=("${CCFLAGS_B[@]}"); fi
    mdir="$WORK/mod/$mode/$mod"
    rm -rf "$mdir"; mkdir -p "$mdir"
    cp "$src" "$mdir/$mod.c"                       # 源文件名必须与 obj-m 目标同名
    {
        printf 'ccflags-y += %s\n' "${flags[@]}"
        printf 'obj-m := %s.o\n' "$mod"
    } > "$mdir/Makefile"

    if make -C "$KSRC" M="$mdir" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE_NAME" \
            KBUILD_MODPOST_WARN="$MODPOST_WARN" modules >"$mdir/build.log" 2>&1 \
       && [ -f "$mdir/$mod.ko" ]; then
        cp "$mdir/$mod.ko" "$OUT/$mod.ko"
        return 0
    fi
    return 1
}

MODROOT="$WORK/mod"
rm -rf "$MODROOT"; mkdir -p "$MODROOT"
MANIFEST="$OUT/manifest.txt"
: > "$MANIFEST"

BUILT=(); FAILED=(); SKIPPED=()
shopt -s nullglob
for src in "$SRC_DIR"/drv_ms_cus_*.c; do
    base="$(basename "$src" .c)"
    suffix="${base#drv_ms_cus_}"

    if should_skip "$suffix"; then
        warn "跳过 $base（依赖 SDK 私有头 srcfg_drv.h）"
        SKIPPED+=("$base")
        continue
    fi
    wanted "$suffix" || continue

    mod="$(module_name_for "$src")"
    printf '\033[1;36m[build]\033[0m %-34s -> %s.ko\n' "$base" "$mod"

    if try_build "$src" "$mod" A; then
        printf '%-34s -> %s.ko   [headers: repo]\n'   "$base" "$mod" >> "$MANIFEST"
        BUILT+=("$mod.ko")
    elif try_build "$src" "$mod" B; then
        warn "$base：需改用 OpenIPC 头文件优先才通过（已记录到 manifest）"
        printf '%-34s -> %s.ko   [headers: openipc]\n' "$base" "$mod" >> "$MANIFEST"
        BUILT+=("$mod.ko")
    else
        warn "$base 两种头文件顺序都编译失败，最后 12 行日志："
        { tail -n 12 "$MODROOT/B/$mod/build.log" 2>/dev/null \
          || tail -n 12 "$MODROOT/A/$mod/build.log" 2>/dev/null; } | sed 's/^/        /' || true
        FAILED+=("$base")
    fi
done

# ------------------------------ 7. 汇总 --------------------------------------
echo
log "================ 编译结果 ================"
log "成功 ${#BUILT[@]} 个 -> $OUT"
for k in ${BUILT[@]+"${BUILT[@]}"}; do printf '        %s\n' "$k"; done
[ "${#SKIPPED[@]}" -gt 0 ] && warn "跳过 ${#SKIPPED[@]} 个（缺 SDK 私有头）: ${SKIPPED[*]}"
if [ "${#FAILED[@]}" -gt 0 ]; then
    warn "失败 ${#FAILED[@]} 个: ${FAILED[*]}"
    warn "逐个日志: $MODROOT/A/<模块名>/build.log 与 $MODROOT/B/<模块名>/build.log"
fi

log "vermagic 校验（应与设备内核一致，形如 '4.9.84 ... ARMv7'）:"
for k in ${BUILT[@]+"${BUILT[@]}"}; do
    if command -v modinfo >/dev/null 2>&1; then
        printf '        %-32s %s\n' "$k" "$(modinfo -F vermagic "$OUT/$k" 2>/dev/null || echo '?')"
    else
        printf '        %-32s %s\n' "$k" \
            "$(strings "$OUT/$k" | grep -m1 '^vermagic=' || echo '?')"
    fi
done

[ "${#FAILED[@]}" -eq 0 ] || exit 1
log "完成。部署：把 build/out/*.ko 拷到设备 /lib/modules/$KERNEL_VERSION/sigmastar/"
