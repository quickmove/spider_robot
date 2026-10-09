#!/usr/bin/env python3
"""app 分区余量检查 —— 固件涨到装不下时，在构建期就发现

分区表解析交给 IDF 自带的 gen_esp32part.py（权威，且顺带做合法性校验），
不手撸二进制偏移——那会随 IDF 版本漂移。

用法: python3 tools/check_partition.py
"""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import subprocess
import sys

WARN_PCT = 90.0


def find_gen_esp32part() -> pathlib.Path | None:
    """定位 IDF 自带的 gen_esp32part.py。

    分区表解析交给 IDF 官方脚本（权威且会做合法性校验），
    不手撸二进制偏移。这里按环境变量 -> 常见安装位置依次找。
    """
    home = pathlib.Path.home()
    roots = [
        os.environ.get("IDF_PATH_DIR"),
        os.environ.get("IDF_PATH"),          # export.sh 会导出
        home / "env/esp/esp-idf-v5.5.1",
        home / "esp/esp-idf",
    ]
    p = pathlib.Path(sys.argv[0]).resolve().parent if sys.argv else None
    if p:                                   # 仓库同级也可能有一份
        roots.append(p.parent.parent)
    for r in roots:
        if not r:
            continue
        cand = pathlib.Path(r) / "components/partition_table/gen_esp32part.py"
        if cand.exists():
            return cand
    return None


GEN_ESP32PART = find_gen_esp32part()


def parse_partitions(table: pathlib.Path) -> list[tuple[str, int]]:
    """返回 [(标签, 字节数)]，仅 app 类型。失败返回 []。"""
    if GEN_ESP32PART is None:
        return []
    out = subprocess.run([sys.executable, GEN_ESP32PART, str(table)],
                         capture_output=True, text=True)
    if out.returncode != 0:
        print(f"[分区检查] 分区表解析失败:\n{out.stderr.strip()}", file=sys.stderr)
        return []

    apps = []
    for line in out.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("ESP-IDF"):
            continue
        parts = line.split(",")
        if len(parts) < 5:
            continue
        name, ptype, _sub, _offset, size = parts[:5]
        if ptype != "app":
            continue
        size_s = size.strip().upper()
        mult = 1
        if size_s.endswith("K"):
            mult, size_s = 1024, size_s[:-1]
        elif size_s.endswith("M"):
            mult, size_s = 1024 * 1024, size_s[:-1]
        try:
            apps.append((name, int(size_s, 0) * mult))
        except ValueError:
            continue
    return apps


def main() -> int:
    args_file = pathlib.Path("build/flasher_args.json")
    if not args_file.exists():
        return 0

    args = json.loads(args_file.read_text())
    app_rel = args["app"]["file"] if isinstance(args["app"], dict) else args["app"]
    app_bin = pathlib.Path("build") / app_rel
    if not app_bin.exists():
        return 0
    app_size = app_bin.stat().st_size

    apps = parse_partitions(pathlib.Path("build/partition_table/partition-table.bin"))
    if not apps:
        print("[分区检查] 未找到 app 分区条目，跳过")
        return 0

    label, size = min(apps, key=lambda x: x[1])
    used = 100.0 * app_size / size
    print(f"[分区检查] 固件 {app_size/1024:.0f}KB / 最小 app 分区 {label} "
          f"{size/1024:.0f}KB -> 已用 {used:.0f}%，余量 {100-used:.0f}%")

    if used > WARN_PCT:
        print(f"[分区检查] 失败：余量不足 {100-WARN_PCT:.0f}%，"
              f"请调整 partitions.csv 或精简依赖", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())