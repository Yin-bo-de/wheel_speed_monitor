#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 KiCad 里打开的图纸「恢复」到磁盘上的最新版本——不用关软件。

原理：连上正在运行的 KiCad（9.0+ 的 IPC API），对打开中的文档发 RevertDocument
命令——效果等价于菜单里的 File → Revert（"恢复到已保存的版本"），一步刷新。

用法：
    python3 scripts/reload_kicad.py        # PCB 和原理图都刷
    python3 scripts/reload_kicad.py pcb    # 只刷 PCB
    python3 scripts/reload_kicad.py sch    # 只刷原理图

前提（不满足会被脚本点名）：
    1. KiCad 正在运行，且「设置 → 插件」里启用了 API（enable_server，
       启用后要重启一次 KiCad 服务才会监听）；
    2. 要刷的图纸已经在对应编辑器里打开（PCB 在 pcbnew、原理图在 eeschema）；
    3. 打开中的图纸没有未保存的改动（Revert 会丢内存里的改动，和菜单行为一致）。

客户端库 kicad-python 装在 ~/.venvs/kicad-api（pip install kicad-python）。
本脚本没找到库时会自动用那个环境重启自己，所以用系统 python3 直接跑也行。
"""
import os
import sys

VENV_PY = os.path.expanduser("~/.venvs/kicad-api/bin/python3")


def ensure_kipy():
    """不在装 kicad-python 的环境里就换过去重启自己。"""
    try:
        import kipy  # noqa: F401
        return
    except ImportError:
        pass
    if os.path.abspath(sys.executable) == os.path.abspath(VENV_PY):
        sys.exit("kicad-python 没装上：~/.venvs/kicad-api/bin/pip install kicad-python")
    if not os.path.exists(VENV_PY):
        sys.exit(f"没找到 {VENV_PY}；先建环境并装库：\n"
                 "  python3 -m venv ~/.venvs/kicad-api\n"
                 "  ~/.venvs/kicad-api/bin/pip install kicad-python")
    os.execv(VENV_PY, [VENV_PY, os.path.abspath(__file__)] + sys.argv[1:])


def main():
    ensure_kipy()
    which = (sys.argv[1] if len(sys.argv) > 1 else "all").lower()

    from google.protobuf.empty_pb2 import Empty
    from kipy import KiCad
    from kipy.proto.common.commands import editor_commands_pb2 as ec
    from kipy.proto.common.types.base_types_pb2 import DocumentType

    kicad = KiCad()
    try:
        kicad.ping()
    except Exception as exc:  # noqa: BLE001 —— 连接失败要给出人能懂的指引
        sys.exit(f"连不上 KiCad：{exc}\n"
                 "检查两件事：① KiCad 是否在运行；② 「设置 → 插件」里的 API 是否已启用"
                 "（启用后要重启一次 KiCad 才会生效）。")

    wanted = []
    if which in ("all", "pcb", "board"):
        wanted.append((DocumentType.DOCTYPE_PCB, "PCB"))
    if which in ("all", "sch", "schematic"):
        wanted.append((DocumentType.DOCTYPE_SCHEMATIC, "原理图"))

    n = 0
    for doc_type, label in wanted:
        try:
            docs = kicad.get_open_documents(doc_type)
        except Exception as exc:  # noqa: BLE001
            # KiCad 10 的 IPC 接口还没实现 eeschema 侧（服务器回"没有处理器"）
            print(f"- {label}：KiCad 的 API 暂不支持（{exc}）——"
                  f"在对应编辑器里按 File → Revert 手动刷新即可（一步，不用关软件）")
            continue
        for doc in docs:
            cmd = ec.RevertDocument()
            cmd.document.CopyFrom(doc)
            kicad._client.send(cmd, Empty)  # kipy 的 board.revert 就是这么发的
            n += 1
            print(f"✓ {label} 已刷新到磁盘上的最新版")
    if n == 0 and not wanted:
        print("参数不认识：用 all / pcb / sch")


if __name__ == "__main__":
    main()
