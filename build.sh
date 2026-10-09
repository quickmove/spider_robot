#!/bin/sh
# ESP-IDF 构建封装：自动加载环境并转发 idf.py 参数
#
# 用法:
#   ./build.sh                  构建
#   ./build.sh flash monitor    烧录 + 串口监视
#   ./build.sh menuconfig       Kconfig
#   ./build.sh size             资源占用
#   ./build.sh bench on|off     单腿台架模式（见 docs/calibration.md §0.5）
#
#   台架模式开/关会改 sdkconfig，而 build/ 的 CMake 缓存不会因此失效，
#   所以切换后必须先清 build 与 sdkconfig，否则烧进去的还是旧固件。
#   这正是 bench 子命令存在的理由——手动改 Kconfig 极易漏掉这一步。
#
# 额外做三件 idf.py 不做的事：
#   1) 自动定位 IDF 源码树（不硬编码单一路径，见下方 find_idf）
#   2) 新增组件后强制重新配置（增量构建会静默跳过未被 CMake 识别的源文件）
#   3) 构建后检查 app 分区余量，超阈值直接失败——避免刷机时才报分区溢出
set -e

# IDF 定位优先级：
#   1) 显式 IDF_PATH_DIR / IDF_PATH
#   2) ~/env/esp/esp-idf-v5.5.1   —— 当前使用（5.5.x，工具链 20241119）
#   3) ~/esp/esp-idf             —— 旧 5.4.1，保留作为回退
#   4) PATH 上 idf.py 反推其所在树
#
# 不用单一硬编码路径：换 IDF 版本或换机器时，不必回来改这个脚本。
find_idf() {
    for c in "$IDF_PATH_DIR" "$IDF_PATH" \
             "$HOME/env/esp/esp-idf-v5.5.1" \
             "$HOME/esp/esp-idf"; do
        if [ -n "$c" ] && [ -f "$c/export.sh" ]; then
            echo "$c"
            return 0
        fi
    done
    p=$(command -v idf.py 2>/dev/null || true)
    if [ -n "$p" ]; then
        d=$(dirname "$(cd "$(dirname "$p")" && pwd)")
        [ -f "$d/export.sh" ] && { echo "$d"; return 0; }
    fi
    return 1
}

IDF_FOUND=$(find_idf) || {
    echo "找不到 ESP-IDF。请设 IDF_PATH_DIR=/path/to/esp-idf" >&2
    exit 1
}

IDF_VER=$(git -C "$IDF_FOUND" describe --tags 2>/dev/null || echo "未知版本")
echo "ESP-IDF: $IDF_FOUND  ($IDF_VER)"

# shellcheck disable=SC1090
. "$IDF_FOUND/export.sh" >/dev/null 2>&1 || {
    echo "无法加载 ESP-IDF 环境: $IDF_FOUND" >&2
    exit 1
}

# 无参数时默认 build —— idf.py 本身无参数只会打印帮助菜单
if [ $# -eq 0 ]; then
    set -- build
fi

# 新增组件后旧的 CMake 缓存不知道它，ninja 会「成功」但漏编新文件
case "$1" in
    build|-C)
        if [ -f build/CMakeCache.txt ]; then
            idf.py reconfigure >/dev/null
        fi
        ;;
esac

# 单腿台架模式开关。改完强制清 build/ 与 sdkconfig：
#   - sdkconfig：让新值生效（IDF 只在首次生成时应用 defaults，
#     menuconfig 改的值则已写入 sdkconfig，改完要重编）
#   - build/：CMake 缓存里存着 sdkconfig 的派生值，不清会编译出旧行为
bench_set() {
    want=$1
    rm -rf build sdkconfig sdkconfig.old
    python3 - "$want" <<'PY'
import sys, pathlib, re
want = sys.argv[1] == "on"
p = pathlib.Path("sdkconfig.defaults")
lines = p.read_text(encoding="utf-8").splitlines()
lines = [l for l in lines if not l.startswith("CONFIG_SPIDER_BENCH_MODE=")]
if want:
    lines.append("CONFIG_SPIDER_BENCH_MODE=y")
p.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"sdkconfig.defaults: SPIDER_BENCH_MODE={'y' if want else 'n'}")
PY
    echo "已清除 build/ 与 sdkconfig（切换 bench 必须如此，否则烧的是旧固件）"
}

case "$1" in
    bench)
        case "${2:-}" in
            on)  bench_set on ;;
            off) bench_set off ;;
            *)   echo "用法: ./build.sh bench on|off" >&2; exit 2 ;;
        esac
        exit 0
        ;;
esac

idf.py "$@"

# app 分区余量检查：固件涨到装不下时，在构建期就发现（失败即中止）
case "$1" in
    build|"")
        if [ -f build/flasher_args.json ]; then
            python3 tools/check_partition.py
        fi
        ;;
esac
