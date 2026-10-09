#!/usr/bin/env python3
"""
protocol_check.py — 文档与实现的协议一致性检查

项目里已经因为「文档写了、实现没跟上」被咬过两次：
  1. params.yaml 的腿位方位角配错腿（校验器已补）
  2. 抬脚高度超过膝软限位（校验器已补）

协议层同样会漂移：docs/protocol.md 列了命令，ws_cmd.c 可能漏实现或改名。
本脚本双向比对，并把命令的处理方式（ack / 静默）也查一遍。

用法: python3 tools/protocol_check.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DOC = REPO / "docs" / "protocol.md"
WS = REPO / "components" / "web_ctrl" / "src" / "ws_cmd.c"
CLI = REPO / "components" / "cli" / "src" / "cli.c"

# 文档里出现、但按设计不通过 WS 传输的命令（属串口专用或控制通道自身）
DOC_ONLY = {
    "ping",          # 实现里有，但走 ack 通道；仍在实现集合内
    "advanced",      # 仅串口可开（防误触），Web 侧只回执
    "cal_scan",      # 长时序动作，仅串口
    "keyframe_upload",  # M4 接入
    "set_params", "save_params",  # M4
}

# 表格首列的表头单元格，不是命令
HEADER_ROWS = {"t", "type", "field", "cmd"}

# 文档里描述但尚未实现的部分。
#
# keyframe_upload 已于 M4 实现并移出本集合；检查随即对 set_params /
# save_params 转为强制——即上表一旦落地实现就必须同步接线，
# 不允许再次借 NOT_YET 绕过。
NOT_YET = {
    "set_params", "save_params",
}


def doc_commands() -> set[str]:
    """从 docs/protocol.md 的 §3「命令类型全集」提取 t 值

    只取 §3 的表格首列——§2 的字段表里也有大量反引号标识符
    （vx / neutral / trim ...），把它们当命令会淹没真实差异。
    """
    text = DOC.read_text(encoding="utf-8")
    start = text.find("## 3. 命令类型全集")
    end = text.find("## 4. 关键帧上传")
    if start == -1:
        return set()
    section = text[start:end if end != -1 else len(text)]

    cmds = set()
    for line in section.splitlines():
        m = re.match(r"^\|\s*`([a-z_]+)`\s*\|", line)
        if not m:
            continue
        name = m.group(1)
        if name in HEADER_ROWS:        # 表头单元格，不是命令
            continue
        cmds.add(name)
    return cmds


def ws_commands() -> set[str]:
    """从 ws_cmd.c 提取已实现的命令字面量"""
    text = WS.read_text(encoding="utf-8")
    return set(re.findall(r'!strcmp\(t,\s*"([a-z_]+)"\)', text))


def cli_commands() -> set[str]:
    """从 cli.c 提取已实现的 CLI 命令名"""
    text = CLI.read_text(encoding="utf-8")
    cmds = set()
    for m in re.finditer(r'strcmp\(cmd,\s*"([a-z_]+)"\)', text):
        cmds.add(m.group(1))
    for m in re.finditer(r'strcmp\(sub,\s*"([a-z_]+)"\)', text):
        cmds.add(m.group(1))
    return cmds


# CJK、全角标点与括号：help 与 §5 里命令右侧都是中文说明，
# 必须先剥掉，否则「姿态」这类词会被当成参数名抽出来。
_CJK = re.compile(r"[\u3000-\u303f\u4e00-\u9fff\uff00-\uffef]")


def _bare_words(text: str) -> set[str]:
    """抽出文本里的裸词（去掉中文说明与 <参数> 区），再与真值集合求交。

    求交是关键：`gait crawl|trot` 里的 crawl/trot、`leg sweep` 里的
    fast/slow/step_us 都是参数而非命令，与 dispatch 相交后自动消失，
    不需要维护一份易腐的排除名单。
    """
    out: set[str] = set()
    for line in text.splitlines():
        s = _CJK.sub(" ", line.strip())
        s = s.split("<")[0].split("[")[0]     # 截断到参数区
        out |= set(re.findall(r"\b([a-z][a-z0-9_]*)\b", s))
    return out


def cli_help_commands() -> set[str]:
    """从 cli.c 的 print_help() 提取 help 里列出的命令名"""
    text = CLI.read_text(encoding="utf-8")
    i = text.find("static void print_help")
    if i < 0:
        return set()
    j = text.find("支撑裕度", i)                # 末条 help 的结尾锚
    if j < 0:
        j = text.find("\n    }", i)
    lits = re.findall(r'"((?:[^"\\]|\\.)*)"', text[i:j + 20])
    return _bare_words("".join(lits).replace("\\n", "\n"))


def doc_cli_section() -> tuple[str, str]:
    """返回 §5 代码块的 (非预留部分, 预留部分)。

    只取 ``` 代码块：下方散文里的 docs / md / calibration 之类会被
    误当成命令名。
    """
    text = DOC.read_text(encoding="utf-8")
    sec = text[text.find("## 5. 串口 CLI"):text.find("## 6.")]
    a, b = sec.find("```"), sec.rfind("```")
    block = sec[a + 3:b] if a >= 0 and b > a else ""
    reserved = "\n".join(ln for ln in block.splitlines() if "【预留】" in ln)
    active = "\n".join(ln for ln in block.splitlines() if "【预留】" not in ln)
    return active, reserved


def doc_cli_commands(text: str) -> set[str]:
    """从 §5 逐行取出「这一行声称的命令」。

    只认行首的命令词；`cal` / `leg` 这类子命令组再带上第二个词。
    不能直接抽全文所有裸词——`pose stand|sit|rest|spawn` 的 stand/sit、
    `gait crawl|trot` 的 crawl 都会是参数而非命令，会被误判成幽灵命令。
    """
    cmds: set[str] = set()
    for line in text.splitlines():
        s = _CJK.sub(" ", line.strip())
        if not s:
            continue
        words = re.findall(r"\b([a-z][a-z0-9_]*)\b", s)
        if not words:
            continue
        cmds.add(words[0])
        if words[0] in ("cal", "leg") and len(words) > 1:
            cmds.add(words[1])
    return cmds


def main() -> int:
    doc = doc_commands()
    ws = ws_commands()
    cli = cli_commands()

    print(f"文档列出 {len(doc)} 个命令，WS 实现 {len(ws)} 个，CLI 实现 {len(cli)} 个")
    print()

    fails = 0

    # 1) 文档有、WS 无（M3 应实现的）
    missing = sorted(doc - ws - DOC_ONLY - NOT_YET)
    if missing:
        print("[FAIL] 文档列出但 WS 未实现:")
        for c in missing:
            print(f"         {c}")
        fails += 1
    else:
        print("[PASS] 文档命令均已在 WS 实现（或仍在 NOT_YET 白名单）")

    # 2) WS 有、文档无（实现多出来的，也要写进文档）
    extra = sorted(ws - doc)
    if extra:
        print("[WARN] WS 实现但文档未记录（应补进 docs/protocol.md）:")
        for c in extra:
            print(f"         {c}")
    else:
        print("[PASS] WS 无文档外的命令")

    # 3) 高频 drive 必须静默（否则 20Hz 摇杆会把 httpd 刷爆）
    text = WS.read_text(encoding="utf-8")
    drive_block = re.search(r'if \(!strcmp\(t, "drive"\)\).*?goto done;', text, re.S)
    if drive_block and "ack(" not in drive_block.group(0):
        print("[PASS] drive 高频命令不回 ack（防刷屏/自锁）")
    else:
        print("[FAIL] drive 命令不应回 ack")
        fails += 1

    # 4) 安全命令必须在标定互斥**调用点**之前（不是函数定义处）
    safety_pos = text.find('strcmp(t, "estop")')
    calib_gate = text.find("reject_if_calibrated(t)")
    if safety_pos != -1 and calib_gate != -1 and safety_pos < calib_gate:
        print("[PASS] estop 在标定互斥之前处理（任何时候可用）")
    else:
        print("[FAIL] estop 被标定互斥挡住或缺失")
        fails += 1

    # 5) 标定模式必须能拒绝运动命令
    if "cal_not_active" in text and "reject_if_calibrated" in text:
        print("[PASS] 标定模式拒绝运动命令并回明确原因")
    else:
        print("[FAIL] 标定与运动互斥未实现")
        fails += 1

    # 6) CLI 必须保留急停与标定入口
    missing_cli = [n for n in ("estop", "estop_clear", "cal") if n not in cli]
    if missing_cli:
        print("[FAIL] CLI 缺少:", ", ".join(missing_cli))
        fails += 1
    else:
        print("[PASS] CLI 保留急停与标定入口（Web 不可用时的兜底）")

    # 8) CLI 三方一致：dispatch ⊆ help，dispatch ⊆ §5（【预留】行不计入）
    #    真值是 dispatch（strcmp 分支，即「实际敲什么会生效」），
    #    help 与 §5 都只是需要被校验的文档。
    #
    #    【预留】行已足够防混用：把已实现命令标成预留会让它离开 active，
    #    上面的「§5 覆盖全部」随即失败，无需再单设一条「预留行不含
    #    真实命令」的检查——那条会把 params show 里的 show 误判
    #    （show 是 cal show 的命令名，但在 params 后是参数）。
    disp = cli_commands()
    in_help = cli_help_commands() & disp
    missing_help = sorted(disp - in_help)
    if missing_help:
        print("[FAIL] 已实现但 help 未列出:", ", ".join(missing_help))
        fails += 1
    else:
        print(f"[PASS] help 覆盖全部 {len(disp)} 条已实现 CLI 命令")

    doc_active, _reserved = doc_cli_section()
    active_cmds = _bare_words(doc_active) & disp

    missing_doc = sorted(disp - active_cmds)
    if missing_doc:
        print("[FAIL] 已实现但 docs/protocol.md §5 未记录:", ", ".join(missing_doc))
        fails += 1
    else:
        print(f"[PASS] §5 覆盖全部 {len(disp)} 条已实现 CLI 命令"
              "（【预留】行不计入）")

    # 反向：§5 声称的命令却没实现（幽灵命令）。
    # 这正是最初 protocol_check 没覆盖的那一类——文档列了 play/params，
    # 敲下去只会得到 ERR 未知命令。用【预留】标记即可合法豁免。
    ghosts = sorted(doc_cli_commands(doc_active) - disp - {"cal", "leg"})
    if ghosts:
        print("[FAIL] §5 列了但 CLI 未实现（应标【预留】或删除）:", ", ".join(ghosts))
        fails += 1
    else:
        print("[PASS] §5 无幽灵命令（未实现的条目已标【预留】）")

    # 7) §4 上限数值：文档 <-> 代码常量必须一致
    kf_hdr = (REPO / "components/posture/include/spider_kf_parse.h").read_text(
        encoding="utf-8")
    key_hdr = (REPO / "components/posture/include/spider_keyframe.h").read_text(
        encoding="utf-8")
    prog_text = DOC.read_text(encoding="utf-8")

    def define_int(text: str, macro: str) -> int | None:
        m = re.search(rf"^#define\s+{macro}\s+(\d+)", text, re.M)
        return int(m.group(1)) if m else None

    before = fails
    for macro, text, src in (
        ("KF_MAX_UPLOAD_BYTES", kf_hdr, prog_text),
        ("KF_MAX_SEQS", key_hdr, prog_text),
    ):
        val = define_int(text, macro)
        if val is None:
            print(f"[FAIL] 代码中找不到 {macro}")
            fails += 1
        elif f"{val}" not in src:
            print(f"[FAIL] {macro} = {val}，但 docs/protocol.md §4 未出现该数值")
            fails += 1
    if fails == before:
        print("[PASS] §4 上限数值与代码常量一致")

    # 上限必须真的容得下协议 §4 给的示例报文
    cap = define_int(kf_hdr, "KF_MAX_UPLOAD_BYTES")
    if cap is not None and cap < 512:
        print(f"[FAIL] KF_MAX_UPLOAD_BYTES={cap} 容不下 §4 示例报文")
        fails += 1

    print()
    print("=" * 56)
    print("协议一致性检查: " + ("全部通过" if fails == 0 else f"{fails} 项未通过"))
    print("=" * 56)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())