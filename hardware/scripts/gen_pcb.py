#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成轮速监测台转接底板的 PCB（.kicad_pcb，KiCad 9 格式，10 可直接打开）。

流程：
  1. 调 kicad-cli 从原理图导网表——焊盘网络的真源是原理图，绝不手写
  2. 按 PLACEMENT 表把各封装嵌进板文件，焊盘自动挂网（对不上就报错，防漏件）
  3. 画板框、四角 M2.5 安装孔、背面 GND 铺铜
  4. 布线（TRACKS/VIAS 表）+ 丝印标注（build_silk；先过 check_silk 几何自检：
     逐条文字对焊盘开窗/板边/彼此做包围盒检查，不过直接构建失败）
  5. kicad-cli DRC + 渲染目检 -> 嘉立创打样文件（导出命令见 README）

机械基准（图纸测量 + 用户尺子实测确认）：
  - 开发板两排针中心距 25.4mm（一英寸）；每排 20 针 2.54 间距
  - 开发板约 28 × 52mm；针 1 距板顶（天线端）约 1.27mm，USB 口在板底
  - 底板草案 96 × 116mm，四角 M2.5 安装孔

坐标约定：与 KiCad 一致，原点在左上角，x 向右、y 向下，单位 mm。
旋转：按封装默认朝向逆时针旋转（1xN 排针默认针 1 在上、针列向下；
转 90° 后针列向右）。渲染目检确认。

用法：python3 gen_pcb.py [输出路径]
"""
import copy
import json
import os
import subprocess
import sys
import uuid as _uuid

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kicad_lib as K

CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
TEMPLATE_PCB = ("/Applications/KiCad/KiCad.app/Contents/SharedSupport/template/"
                "stm32f100-discovery-shield/stm32f100-discovery-shield.kicad_pcb")
SCH = os.path.normpath(os.path.join(HERE, "..", "wheel_carrier", "wheel_carrier.kicad_sch"))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "wheel_carrier", "wheel_carrier.kicad_pcb"))
NETLIST_TMP = "/tmp/wheel_carrier_netlist.net"

ROOT_UUID = "11111111-2222-4333-8444-555555555555"
NS = _uuid.UUID(ROOT_UUID)
def uid(tag):
    return str(_uuid.uuid5(NS, "pcb/" + tag))

# ---------------------------------------------------------------- 板级常量

BOARD_W, BOARD_H = 96.0, 98.5           # v5：只压 y 轴（122→98.5），x 轴不动——
                                        # 目标是把高度压进嘉立创 100×100 的免费档
MOUNTING_HOLES = [(3.5, 3.5), (BOARD_W - 3.5, 3.5),
                  (3.5, BOARD_H - 3.5), (BOARD_W - 3.5, BOARD_H - 3.5)]
MOUNTING_FP = "MountingHole:MountingHole_2.7mm_M2.5"

# 放置表：(位号, 封装, 值, x, y, 旋转)；(x,y) = 焊盘包围盒中心。
# 间距按真实封装尺寸留（电阻跨 10.16、二极管跨 15.24、D10 电解外框 10.5）。
# 分区：顶部=霍尔接口+上拉+指示行；中部=开发板插座；右侧=舵机/遥控；
#       中下=扩展排针+电源排针；底部=电源区
PLACEMENT = [
    # == 顶部：霍尔接口（针 1=信号 在左），同一列的正下方依次是
    #    10k 上拉 → 330R → 红灯，一路霍尔一"列" ==
    ("J4",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·右前", 11, 6, 90),
    ("J3",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·左前", 21, 6, 90),
    ("J5",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·左后", 34, 6, 90),
    ("J6",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·右后", 47, 6, 90),
    ("R2",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 8, 11.5, 0),
    ("R1",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 21, 11.5, 0),
    ("R3",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 34, 11.5, 0),
    ("R4",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 47, 11.5, 0),
    ("R6",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 8, 17, 0),
    ("R7",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 21, 17, 0),
    ("R8",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 34, 17, 0),
    ("R9",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 47, 17, 0),
    ("LED2", "LED_THT:LED_D5.0mm", "红灯", 8, 22.5, 90),
    ("LED3", "LED_THT:LED_D5.0mm", "红灯", 21, 22.5, 90),
    ("LED4", "LED_THT:LED_D5.0mm", "红灯", 34, 22.5, 90),
    ("LED5", "LED_THT:LED_D5.0mm", "红灯", 47, 22.5, 90),
    # == 开发板插座（间距 25.4 是实测基准；针 1 在左端 x=12.7）
    #    中心坐标 = 针 1 的 x + 24.13（20 针跨 48.26） ==
    #    v5：J13/J14 从 J2 下方 18/26mm 收到 5.08mm——这是本轮压缩的最大一笔，
    #    扇形引出改走半间距错位的"车道"，两层都不用打孔（详见 build_routes）
    ("J1", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板左排 20P", 36.83, 33.5, 90),
    ("J2", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板右排 20P", 36.83, 58.9, 90),
    # == 右侧：舵机 + 遥控。自上而下的顺序是 J8→J7→J11→J10→J9：
    #    按各自 J2 目标针的 x 递增排列，B 层那五条走线才不打结（布局服务布线） ==
    ("J8",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·后差速", 88, 34, 0),
    ("J7",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·前差速", 88, 43, 0),
    ("J11", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·后差速", 88, 52, 0),
    ("J10", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·前差速", 88, 61, 0),
    ("J9",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·模式", 88, 70, 0),
    # == v4 新增：电调输入 + 油门直通（两口之间是板上直通走线） ==
    #    放在左侧空通道：+BEC 左脊（x=1.3）就在旁边，两口取电只有几毫米；
    #    J16 收电调的 3 针插头（BEC 进板、油门信号进板），J17 用杜邦线接回
    #    接收机油门通道（J17 的 +V 顺带把电送给接收机）
    ("J16", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "电调·BEC输入", 6.5, 50, 0),
    ("J17", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "接收机·油门", 6.5, 62, 0),
    # == 扩展排针（与 J1/J2 一一对应）+ 电源条 ==
    ("J13", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·左排(直连 J1)", 36.83, 63.98, 90),
    ("J14", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·右排(直连 J2)", 36.83, 69.06, 90),
    # 电源条 2×8（v4）：rot 270 ⇒ 8 列 × 2 行，针 1 在右端、奇针=上排（电源）、
    # 偶针=下排（全 GND）；列（西→东）= 3V3×3、5V×3、BEC×2
    ("J15", "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical", "电源排针", 71.89, 25.27, 270),
    # == 电源区：三行摆位，间距按外框实数（D10=10.5、DO-201AD=18.9、DIN0207=12.3） ==
    ("J12", "TerminalBlock:TerminalBlock_MaiXu_MX126-5.0-02P_1x02_P5.00mm", "BEC 输入 6~7.4V", 6, 81.5, 0),
    ("D1",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "1N5824", 22.5, 80, 0),
    ("C1",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "1000µF/25V", 40, 80.5, 0),
    ("R5",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "1.5k", 53, 80, 0),
    ("LED1", "LED_THT:LED_D5.0mm", "绿灯", 63, 80, 90),
    ("D3",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "SB560", 22.5, 91, 0),
    ("C2",  "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 36, 91, 0),
    ("D2",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "1N5822", 49.5, 91, 0),
    ("R10", "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "1k", 52, 86, 0),
    ("LED6", "LED_THT:LED_D5.0mm", "绿灯", 75, 94, 90),
    ("U1",  "Package_TO_SOT_THT:TO-220-5_Vertical", "LM2596T-5", 78, 81, 0),
    ("C3",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "220µF/25V", 89.5, 80.25, 0),
    ("L1",  "Inductor_THT:L_Radial_D10.0mm_P5.00mm_Fastron_07P", "47µH", 65.5, 91.25, 0),
    ("C4",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "220µF/16V", 83.5, 91.25, 0),
]


# ---------------------------------------------------------------- 网表

def export_netlist():
    """跑 kicad-cli 导网表并解析成 (net 列表, {位号: {焊盘号: net名}})。"""
    subprocess.run([CLI, "sch", "export", "netlist", "--format", "kicadsexpr",
                    "--output", NETLIST_TMP, SCH], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = K.load(NETLIST_TMP)
    nets, padmap = [], {}
    for nets_node in K.get_all(node, "nets"):
        for net in K.get_all(nets_node, "net"):
            name = str(K.get(net, "name")[1])
            if name.startswith("/"):        # 网表导出的根前缀，板文件里惯例不带
                name = name[1:]
            members = []
            for nd in K.get_all(net, "node"):
                ref = str(K.get(nd, "ref")[1])
                pin = str(K.get(nd, "pin")[1])
                members.append((ref, pin))
                padmap.setdefault(ref, {})[pin] = name
            nets.append((name, members))
    return nets, padmap


# ---------------------------------------------------------------- 封装嵌入

_fpcache = {}


def rot_off(dx, dy, rot):
    """屏幕坐标（y 向下）里逆时针旋转 rot 度的位移换算。
    与原理图生成器同一套约定：(0,-1)（上）转 90° 后是 (-1,0)（左）。"""
    r = rot % 360
    if r == 0:
        return (dx, dy)
    if r == 90:
        return (dy, -dx)
    if r == 180:
        return (-dx, -dy)
    if r == 270:
        return (-dy, dx)
    raise ValueError(rot)


def load_footprint(lib_id):
    if lib_id not in _fpcache:
        lib, name = lib_id.split(":")
        path = os.path.join(FPDIR, lib + ".pretty", name + ".kicad_mod")
        _fpcache[lib_id] = K.load(path)
    return _fpcache[lib_id]


# 位号文字位置的局部坐标覆写（库里的默认位置实测会压到邻件丝印或焊盘开窗，
# 被 DRC 点名；表里的坐标就是各封装自己的局部系）。值 = (局部 x, 局部 y)。
REF_MOVES = {
    "LED1": ("-0.83", "4.3"),    # 挪到灯右侧（原位置压在 R5.2 焊盘开窗上，字会被阻焊裁断）
    "LED6": ("4.8", "-1.0"),     # v5：位号放灯左上方的空位
    "C1":   ("6.5", "-5.5"),     # 原位置压在扩展排针外框线上
    "C3":   ("5.5", "-5.25"),    # v5：挪到右上方，让开 J9 的地脚
    "C4":   ("5.5", "-5.25"),    # 原位置压在 C3 的丝印圆上（一个位号压出 12 条 DRC 告警）
    "R10":  ("0.0", "-2.5"),     # v5：R10 换了朝向，位号挪到上方空位
    "D2":   ("0.0", "3.4"),      # v5：R10 挪到附近，D2 位号让到二极管下方
    "J15":  ("0", "-4.22"),      # 2×8 的默认位号位置贴着自带外框线，往右让一点
    # v5：右侧接口列距从 10mm 收到 8mm，位号让到接口左侧的 S/+V/G 列之外
    "J7":   ("-5.5", "0"),
    "J8":   ("-5.5", "0"),
    "J9":   ("-5.5", "0"),
    "J10":  ("-5.5", "0"),
    "J11":  ("-5.5", "0"),
    "J17":  ("2.5", "3"),        # 默认位置压「电调 BEC 输入」的标注
}


def ref_local(ref, lib_at):
    """位号的局部坐标：库里默认位置 + 上面的覆写规则（embed 和自检共用，
    保证「自检看到的位号位置」就是「文件里写的位置」）。
    J1/J2/J13 的位号放各自排针左端上方、J14 放左端下方（v5 的 5.08mm
    行距下，默认位置会压到邻排的标注或接口）。"""
    if ref in REF_MOVES:
        return REF_MOVES[ref]
    if ref.startswith("LED"):
        return ("0", "-6.0")
    if ref == "J14":
        return ("-4.5", "-4.5")
    if ref in ("J1", "J2", "J13"):
        return ("4", "-3.2")
    return (str(lib_at[1]), str(lib_at[2]))


def embed_footprint(lib_id, ref, value, x, y, rot, pad_nets):
    """库里的 .kicad_mod 改写成一个板上的封装实例：
    - 顶层的 (version)/(generator) 去掉，改名成 库:名
    - (x,y) 语义是「焊盘包围盒的中心」：KiCad 封装原点通常在 pad1 上，
      直接拿原点当中心摆位，器件会展出半个身位（实测撞过 22 处外框重叠）
    - 每个焊盘按网表挂 (net 号 "名")；网表里查不到的焊盘直接报错"""
    node = copy.deepcopy(load_footprint(lib_id))
    xs, ys = [], []
    for c in node:
        if isinstance(c, list) and c and c[0] == "pad":
            at = K.get(c, "at")
            xs.append(float(at[1]))
            ys.append(float(at[2]))
    ccx = (min(xs) + max(xs)) / 2.0
    ccy = (min(ys) + max(ys)) / 2.0
    ox, oy = rot_off(ccx, ccy, rot)
    ox, oy = round(x - ox, 4), round(y - oy, 4)

    children = [node[1]]                                    # 名字占位
    for c in node[2:]:
        if isinstance(c, list) and c and c[0] in ("version", "generator", "layer"):
            continue
        children.append(copy.deepcopy(c))
    kids = ["footprint", K.Q(lib_id)]        # 首项是类型标记，没有它 KiCad 认不出
    kids.append(["layer", K.Q("F.Cu")])
    kids.append(["uuid", K.Q(uid("fp-" + ref))])
    at = ["at", f"{ox:g}", f"{oy:g}"]
    if rot % 360:
        at.append(str(rot))
    kids.append(at)
    for c in children[1:]:
        if isinstance(c, list) and c and c[0] == "property":
            pname = str(c[1])
            if pname == "Reference":
                c[2] = K.Q(ref)
                # 位号字号统一压到 0.8；有问题的位号按 REF_MOVES 挪位
                eff = K.get(c, "effects")
                font = K.get(eff, "font") if eff is not None else None
                size = K.get(font, "size") if font is not None else None
                if size is not None:
                    size[1], size[2] = "0.8", "0.8"
                at = K.get(c, "at")
                at[1], at[2] = ref_local(ref, at)
            elif pname == "Value":
                c[2] = K.Q(value)
                # 板上丝印不印值、只留位号（也是丝印压焊盘告警的大头）
                c.append(["hide", "yes"])
        kids.append(c)
    # 焊盘挂网；U1（官方 TO-220-5）焊盘窄边环宽不够板厂下限，等比放大到 1.4；
    # 右侧接口的 GND 脚四周被走线夹着，花焊盘只剩一根辐条，改实心连接
    wanted = pad_nets.get(ref, {})
    for c in kids:
        if isinstance(c, list) and c and c[0] == "pad":
            if ref == "U1":
                # 库盘 1.275×1.8 配 1.1 孔，窄边环宽只剩 0.0875（板厂常规下限 0.13）
                # ——等比放大到 1.4（环 0.15）；间距、孔径照库不动，插装不受影响
                for sub in c:
                    if isinstance(sub, list) and sub and sub[0] == "size":
                        sub[1] = "1.4"
                        sub[2] = "1.4"
            if ref == "J15" and str(c[1]) == "1":
                # 电源条第 1 脚（右上角 BEC 列）的方形 1 脚标改圆盘——2026-10-13
                # 在 KiCad 里手工改的，同步回脚本；脚序靠丝印的电压标注仍读得出
                if c[3] != "rect":
                    raise SystemExit(f"J15 的 1 脚形状变成 {c[3]!r}，改圆盘的前提失效")
                c[3] = "circle"
            if (ref in ("J8", "J7", "J11", "J10", "J9") and str(c[1]) == "3"):
                c.append(["zone_connect", "2"])
            if (ref == "U1" and str(c[1]) in ("3", "5")) or (ref == "LED1" and str(c[1]) == "1"):
                # 被相邻铜挤得花焊盘连不上，这两处 GND 改实心
                c.append(["zone_connect", "2"])
    seen_pads = set()
    for c in kids:
        if isinstance(c, list) and c and c[0] == "pad":
            padnum = str(c[1])
            seen_pads.add(padnum)
            if padnum not in wanted:
                raise SystemExit(f"{ref} 焊盘 {padnum} 在网表里没有对应网络，放置表和原理图对不上")
            netname = wanted[padnum]
            code = NET_CODES[netname]
            c.append(["net", str(code), K.Q(netname)])
    missing = set(wanted) - seen_pads
    if missing:
        raise SystemExit(f"{ref} 网表里的焊盘 {sorted(missing)} 在封装里不存在")
    kids.append(["path", K.Q("/" + ROOT_UUID)])
    kids.append(["sheetname", K.Q("/")])
    return kids


def mounting_hole(x, y, tag):
    node = copy.deepcopy(load_footprint(MOUNTING_FP))
    kids = ["footprint", K.Q(MOUNTING_FP), ["layer", K.Q("F.Cu")],
            ["uuid", K.Q(uid("mnt-" + tag))], ["at", f"{x:g}", f"{y:g}"]]
    for c in node[2:]:
        # pad 必须留着：2.7mm 的钻孔就挂在它身上。曾经跳过它，结果钻带里一把
        # 2.7 的刀都没有（照文件做出来板子没有安装孔），铜还直接从孔位上空过——
        # 别再把 "pad" 加回这个跳过列表（2026-10-13 踩过，用户眼尖发现的）
        if isinstance(c, list) and c and c[0] in ("version", "generator", "layer",
                                                 "property"):
            continue
        kids.append(copy.deepcopy(c))
    return kids


# ---------------------------------------------------------------- 布线
# 每条路径 = (网络, 层, 线宽, [折点...])，全部直角。THT 焊盘两层可达：
# 起点/终点直接落在焊盘上，无需 via；换层处显式加 via。
# F.Cu：信号 + 电源走线（密集区：插座到扩展排针的 40 根竖线等）
# B.Cu：留给 GND 铺铜 + 跨板长走线（霍尔四条、遥控/舵机五条），铺铜自动避让

TRACKS = []          # (net, layer, w, pts)
VIAS = []            # (net, x, y)


def t(net, layer, w, pts):
    TRACKS.append((net, layer, w, pts))


def via(net, x, y):
    VIAS.append((net, x, y))


def build_routes(padmap):
    wS, w3, w5, wB, wM = 0.3, 0.7, 0.8, 1.2, 1.5   # 信号/3V3/5V/支线/主干
    x0 = 12.7

    # ---- 扩展排针 1:1：左排（J1→J13）在 J2 行处拐进左间隙，
    #      再换 B 层跨完 J13 行（正面列与间隙留给右排）；
    #      右排（J2→J14）拐右间隙跨 J13 行。两排在各个 y 段上严格错开，
    #      这套走法是跟 DRC 磨出来的：半间距错位的 20 列对 20 间隙，
    #      单层怎么排都会撞，借一次 B 层（每针一个过孔）才解得开。
    for i in range(20):
        x = round(x0 + 2.54 * i, 4)
        t(padmap["J1"][str(i + 1)], "F.Cu", wS,
          [(x, 33.5), (x, 56.5), (x - 1.27, 56.5), (x - 1.27, 60.5)])
        via(padmap["J1"][str(i + 1)], round(x - 1.27, 4), 60.5)
        t(padmap["J1"][str(i + 1)], "B.Cu", wS,
          [(x - 1.27, 60.5), (x, 60.5), (x, 63.98)])
        t(padmap["J2"][str(i + 1)], "F.Cu", wS,
          [(x, 58.9), (x, 61.2), (x + 1.27, 61.2), (x + 1.27, 67.4),
           (x, 67.4), (x, 69.06)])

    # ---- +BEC：左脊上到顶、顶干线横穿、右走廊下、底部大电流母线；
    #      左上第一跨绕开安装孔（3.5,3.5）——以前孔没进板文件，DRC 看不到它，
    #      主干线是直接压着孔位过去的
    bec = "+BEC"
    t(bec, "F.Cu", 1.0, [(1.3, 75.8), (1.3, 6.35)])            # 左脊（让开板边 0.5，顶到孔下）
    # 绕孔：线心离孔心要 ≥ 1.35（孔半径）+ 0.25（孔间隙规则）+ 0.75（半线宽）= 2.35，
    # y=6.35 一跨实留 2.85（铜边距孔边 0.75），x=6.35 抬回 y=3.5 再并入原干线
    t(bec, "F.Cu", wM, [(1.3, 6.35), (6.35, 6.35), (6.35, 3.5)])
    xs = [6.35, 11, 21, 34, 47, 80.78, 89.8]
    for a, b in zip(xs[:-1], xs[1:]):                          # 顶干线（分段在分接点处分断）
        t(bec, "F.Cu", wM, [(a, 3.5), (b, 3.5)])
    for x in (11, 21, 34, 47):                                 # 顶部四个接口 +V
        t(bec, "F.Cu", w5, [(x, 3.5), (x, 6)])
    t(bec, "F.Cu", w5, [(80.78, 3.5), (80.78, 24)])            # 下到电源条第 1 列（BEC）
    t(bec, "F.Cu", w5, [(78.24, 24), (80.78, 24)])             # 顶上把两列 BEC 链起来
    ys = [34, 43, 52, 61, 70]      # 连接器中心针 = +V（中心锚定后针 2 就在放置点）
    t(bec, "F.Cu", wB, [(89.8, 3.5), (89.8, ys[0])])           # 右走廊
    for a, b in zip(ys[:-1], ys[1:]):
        t(bec, "F.Cu", wB, [(89.8, a), (89.8, b)])
    for y in ys:                                               # 右侧五个接口 +V
        t(bec, "F.Cu", w5, [(89.8, y), (88, y)])
    t(bec, "F.Cu", wM, [(1.3, 75.8), (87, 75.8)])              # 底部母线（v5 提到 y=75.8）
    for x, y in ((14.88, 80), (37.5, 80.5), (47.92, 80),
                 (74.6, 81), (87, 80.25)):                    # 分接到各耗电点
        t(bec, "F.Cu", w5, [(x, 75.8), (x, y)])

    # ---- +3V3：汇流在 y=29.5（让开霍尔行与导线）+ 每路一列（列线直穿
    #      自己的两个上拉焊盘），再下到 J1.1；电源条的 3V3 列在最东端，
    #      沿 y=29.5 一路兜过去、从西侧进上排
    v3 = "+3V3"
    for x in (2.92, 15.92, 28.92, 41.92):
        t(v3, "F.Cu", w3, [(x, 11.5), (x, 29.5)])
    t(v3, "F.Cu", w3, [(2.92, 29.5), (61.5, 29.5)])
    t(v3, "F.Cu", w3, [(2.92, 29.5), (2.92, 31.5), (12.7, 31.5), (12.7, 33.5)])
    # 电源条那三列：y=29.5 走到 x=61.5 再北上进上排（条本身没动）
    t(v3, "F.Cu", w3, [(61.5, 29.5), (61.5, 24), (63, 24)])
    t(v3, "F.Cu", w3, [(63, 24), (68.08, 24)])                 # 顶上链三列 3V3

    # ---- 霍尔四路（v5）：本地段不动（信号脚→10k→指示灯），信号从各列下来后
    #      走半间距车道回自己的排针脚：GPIO14 直接进 J1.19 焊盘（同网自达），
    #      GPIO21/47 下到 J2.17/J2.16、GPIO1 下到 J2.3。横向段的 y 按
    #      "越靠西跑越靠下"排序，保证四条互不相交（跟 DRC 磨出来的次序）。
    hall = {
        14: ([(8.46, 6), (8.46, 9)], [(8.46, 9), (13.08, 9)],
             [(13.08, 9), (13.08, 11.5)], [(8.46, 9), (14.5, 9)],
             [(14.5, 9), (14.5, 25.6)], [(14.5, 25.6), (8, 25.6), (8, 23.77)],
             ("V", (14.5, 25.6)),
             ("B", [(14.5, 25.6), (58.42, 25.6), (58.42, 33.5)])),
        1:  ([(18.46, 6), (18.46, 9)], [(18.46, 9), (26.08, 9)],
             [(26.08, 9), (26.08, 11.5)], [(18.46, 9), (27.5, 9)],
             [(27.5, 9), (27.5, 23.77)], [(27.5, 23.77), (21, 23.77)],
             ("V", (27.5, 23.77)),
             ("B", [(27.5, 23.77), (24.13, 23.77), (24.13, 24.3)]),
             ("V", (24.13, 24.3)),
             ("F", [(24.13, 24.3), (24.13, 26.5)]),
             ("V", (24.13, 26.5)),
             ("B", [(24.13, 26.5), (19.05, 26.5), (19.05, 35.5),
                    (17.78, 35.5), (17.78, 58.9)])),
        21: ([(31.46, 6), (31.46, 9)], [(31.46, 9), (39.08, 9)],
             [(39.08, 9), (39.08, 11.5)], [(31.46, 9), (40.5, 9)],
             [(40.5, 9), (40.5, 23.77)], [(40.5, 23.77), (34, 23.77)],
             [(40.5, 23.77), (40.5, 27.8)],
             ("V", (40.5, 27.8)),
             ("B", [(40.5, 27.8), (54.61, 27.8), (54.61, 56.5),
                    (53.34, 56.5), (53.34, 58.9)])),
        47: ([(44.46, 6), (44.46, 9)], [(44.46, 9), (52.08, 9)],
             [(52.08, 9), (52.08, 11.5)], [(44.46, 9), (53.5, 9)],
             [(53.5, 9), (53.5, 23.77)], [(53.5, 23.77), (47, 23.77)],
             [(53.5, 23.77), (53.5, 28.5)],
             ("V", (53.5, 28.5)),
             ("B", [(53.5, 28.5), (49.53, 28.5), (49.53, 56.5),
                    (50.8, 56.5), (50.8, 58.9)])),
    }
    for gpio, items in hall.items():
        net = f"GPIO{gpio}"
        for item in items:
            if isinstance(item, tuple) and item[0] == "B":
                t(net, "B.Cu", wS, list(item[1]))
            elif isinstance(item, tuple) and item[0] == "F":
                t(net, "F.Cu", wS, list(item[1]))
            elif isinstance(item, tuple) and item[0] == "V":
                via(net, item[1][0], item[1][1])
            else:
                t(net, "F.Cu", wS, list(item))

    # 霍尔链 LED 的 A 脚走线（R xx0 → LED）：从 LED 左侧绕进，避开
    # 竖列与 +3V3 汇流（xL-2.54 = A 脚的左侧接近点）
    for r_ref, led_ref, xr, xl in (("R6", "LED2", 13.08, 8), ("R7", "LED3", 26.08, 21),
                                   ("R8", "LED4", 39.08, 34), ("R9", "LED5", 52.08, 47)):
        net = padmap[r_ref]["2"]
        t(net, "F.Cu", wS, [(xr, 17), (xr, 18.3), (xl - 2.54, 18.3),
                            (xl - 2.54, 21.23), (xl, 21.23)])

    # ---- 遥控/舵机五路：信号脚（p1，中心锚定后在上方一针）→ 短 B 段 →
    #      F 走廊（纵向，各列互不相交）→ 换回 B 走南部横巷 → 在 J2 目标针的
    #      正下方兜上来（J13/J14 同网，径直穿过它们的焊盘到达）
    right5 = {
        "J8":  (31.46, 85.9, 74.3, 22.86),
        "J7":  (40.46, 84.5, 73.6, 25.4),
        "J11": (49.46, 83.1, 72.9, 27.94),
        "J10": (58.46, 81.7, 72.2, 30.48),
        # J9 的横巷最靠北（y70.4），西行不与其他四条走廊打架
        "J9":  (67.46, 86.8, 71.5, 33.02),
    }
    for ref, (sy, cx, ly, tx) in right5.items():
        net = padmap[ref]["1"]
        t(net, "B.Cu", wS, [(88, sy), (cx, sy)])
        via(net, cx, sy)
        t(net, "F.Cu", wS, [(cx, sy), (cx, ly)])
        via(net, cx, ly)
        # 从下面直接扎进 J14 的目标针（同网），再由 J14↔J2 的扇形引出带上 J2
        t(net, "B.Cu", wS, [(cx, ly), (tx, ly), (tx, 69.06)])

    # ---- 电源局部 ----
    # VIN_RAW 与 DEV_5V 整段走 B 层（B 层西侧/东缘当时都是空的），
    # 避免与 F 层的 +BEC 母线、+5V 汇流交叉
    t("Net-(D1-A)", "B.Cu", 1.0, [(3.5, 81.5), (3.5, 84.5), (30.12, 84.5), (30.12, 80)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 91), (41.88, 87.8)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 87.8), (33.5, 87.8), (33.5, 91)])       # C2.1
    t("DEV_5V", "B.Cu", w5, [(41.88, 87.8), (41.88, 97.0), (89.8, 97.0),
                             (89.8, 30.0), (60.96, 30.0), (60.96, 33.5)])      # 沿板底东缘北上到 J1.20
    sw = "Net-(D3-K)"
    t(sw, "F.Cu", w5, [(76.3, 81), (76.3, 83.5), (17, 83.5),
                       (17, 88.5), (14.88, 88.5), (14.88, 91)])
    t(sw, "F.Cu", wS, [(76.3, 81), (76.3, 83.5)])
    via(sw, 76.3, 83.5)
    t(sw, "B.Cu", wS, [(76.3, 83.5), (63.0, 83.5), (63.0, 91.25)])  # 跳 +5V 汇流，从上面进 L1.1

    # ---- v4：电调输入 J16 / 接收机 J17 ----
    # J16 的 +V 接在底部母线东端（ESC 的 BEC 从这里进板）；J17 的 +V 由右走廊
    # 延长下来（顺带继续给接收机供电）。两口之间的 S 针是「油门直通」：纯走线，
    # 板子不通电也照传，主控不在线时油门路径不断。
    if not (padmap["J16"]["1"] == padmap["J17"]["1"] == "THROTTLE"
            and padmap["J16"]["2"] == padmap["J17"]["2"] == "+BEC"
            and padmap["J16"]["3"] == padmap["J17"]["3"] == "GND"):
        raise SystemExit("J16/J17 网表与「电调 / 油门」的设计对不上，先查原理图")
    thr = padmap["J16"]["1"]
    t(bec, "F.Cu", w5, [(1.3, 50), (6.5, 50)])                 # 左脊 → J16.+V
    t(bec, "F.Cu", w5, [(1.3, 62), (6.5, 62)])                 # 左脊 → J17.+V
    t(thr, "F.Cu", wS, [(6.5, 47.46), (8.5, 47.46)])           # 油门直通：S—S 绕东侧
    t(thr, "F.Cu", wS, [(8.5, 47.46), (8.5, 59.46)])
    t(thr, "F.Cu", wS, [(8.5, 59.46), (6.5, 59.46)])
    t("+5V", "F.Cu", w5, [(46.92, 86), (46.92, 84.6), (95.0, 84.6)])
    t("+5V", "F.Cu", w5, [(55.62, 84.6), (55.62, 88.5), (57.12, 88.5), (57.12, 91)])  # D2.2
    t("+5V", "B.Cu", w5, [(95.0, 20), (73.16, 20)])            # B 层跳到电源条上方
    via("+5V", 73.16, 20)
    t("+5V", "F.Cu", w5, [(73.16, 20), (73.16, 24)])           # 下来进 5V 列
    t("+5V", "F.Cu", w5, [(70.62, 24), (75.7, 24)])            # 顶上链三列 5V
    t("+5V", "F.Cu", w5, [(68.0, 84.6), (68.0, 91.25)])        # L1.2
    t("+5V", "F.Cu", w5, [(81.0, 84.6), (81.0, 91.25)])        # C4.1
    t("+5V", "F.Cu", w5, [(79.7, 84.6), (79.7, 81)])           # U1.FB
    t("+5V", "F.Cu", w5, [(95.0, 84.6), (95.0, 20)])
    via("+5V", 95.0, 20)
    t("Net-(LED1-A)", "F.Cu", wS, [(58.08, 80), (58.08, 78.73), (63, 78.73)])
    t("Net-(LED6-A)", "F.Cu", wS, [(57.08, 86), (57.08, 85.5), (59.5, 85.5)])
    via("Net-(LED6-A)", 59.5, 85.5)
    t("Net-(LED6-A)", "B.Cu", wS, [(59.5, 85.5), (59.5, 93.2), (75, 93.2), (75, 92.73)])
    # R5/LED1 的电源指示链（+BEC 经 R5 到 LED1）——见上方 Net-(LED1-A) 一条
    # LED1 的 GND 脚被周围走线围在一个铺铜孤岛里，补一根到 C1 地脚的短走线
    t("GND", "F.Cu", 0.5, [(63, 81.27), (63, 82.3), (42.5, 82.3), (42.5, 80.5)])
    # 注：v4 那根「救孤岛」的 GND 缝合线在 v5 里删了——布局收紧后 J1 上方
    # 的空档已经小到铺铜自己会忽略，留着反而是一根悬空端（DRC 会点名）


def emit_routes(out):
    for net, layer, w, pts in TRACKS:
        code = NET_CODES[net]
        for (x1, y1), (x2, y2) in zip(pts[:-1], pts[1:]):
            out.append("\t(segment")
            out.append(f"\t\t(start {x1:g} {y1:g})")
            out.append(f"\t\t(end {x2:g} {y2:g})")
            out.append(f"\t\t(width {w:g})")
            out.append(f'\t\t(layer "{layer}")')
            out.append(f'\t\t(net {code})')
            out.append(f'\t\t(uuid "{uid(f"seg-{net}-{layer}-{x1:g},{y1:g}-{x2:g},{y2:g}")}")')
            out.append("\t)")
    for net, x, y in VIAS:
        code = NET_CODES[net]
        out.append("\t(via")
        out.append(f"\t\t(at {x:g} {y:g})")
        out.append("\t\t(size 0.6)")
        out.append("\t\t(drill 0.3)")
        out.append('\t\t(layers "F.Cu" "B.Cu")')
        out.append(f"\t\t(net {code})")
        out.append(f'\t\t(uuid "{uid(f"via-{net}-{x:g},{y:g}")}")')
        out.append("\t)")


# ---------------------------------------------------------------- 丝印标注
# 标注内容全部在这里生成（坐标 = 文字中心点，与走线同一套板面坐标）。
# 写完先过 check_silk() 的几何自检（对焊盘开窗 / 板边 / 彼此做包围盒检查），
# 再进 KiCad DRC 复核。两处都过了才算数——丝印被阻焊裁掉是不可接受的。
#
# 约定：功能名用中文（一眼知道接什么）；引脚级标用 ASCII 小字（IO1 / S / +V / G，
# 0.8mm 高，字号小但笔画简单，仍然清晰）。GPIO 号不手写——从网表取，
# 原理图改了分配这里会自检报错，不会印出过期的号。

SILK_TEXTS = []   # (文字, x, y, rot, 字号, 线宽)

def silk(text, x, y, rot=0, size=1.2, th=0.15):
    SILK_TEXTS.append((text, round(x, 4), round(y, 4), rot, size, th))


def io_name(net):
    """网名 → 板上丝印短名：GPIO1→IO1；IO4 原样；+3V3→3V3；DEV_5V/+5V→5V。"""
    if net.startswith("GPIO"):
        return "IO" + net[4:]
    if net.startswith("IO") and net[2:].isdigit():
        return net
    if net in ("+3V3", "+BEC"):
        return net[1:]
    if net in ("DEV_5V", "+5V"):
        return "5V"
    if net in ("GND", "EN"):
        return net
    raise SystemExit(f"丝印短名表不认识网名 {net}")


def build_silk(padmap):
    # 先对网表：信号脚网名是丝印内容的依据，对不上宁可构建失败（改动原理图后
    # 这里会拦住，提醒同步更新丝印假设）
    expect = {"J3": "GPIO1", "J4": "GPIO14", "J5": "GPIO21", "J6": "GPIO47",
              "J7": "GPIO41", "J8": "GPIO42", "J9": "GPIO38", "J10": "GPIO39",
              "J11": "GPIO40"}
    for ref, want in expect.items():
        got = padmap[ref]["1"]
        if got != want:
            raise SystemExit(f"{ref} 信号脚网名是 {got}，丝印表假设 {want}——"
                             "原理图改动后先更新 build_silk 的假设")

    # 霍尔：功能 + GPIO 标在本列指示灯正下方（一整列 = 一路霍尔，
    # 接口/上拉/限流/指示灯顺读下来）；S/+V/G 标在接口焊盘下沿
    halls = [("J4", "右前霍尔", 11, 8), ("J3", "左前霍尔", 21, 21),
             ("J5", "左后霍尔", 34, 34), ("J6", "右后霍尔", 47, 47)]
    for ref, name, xc, col in halls:
        silk(f"{name} {io_name(padmap[ref]['1'])}", col, 27.0)
        for dx, ch in ((-2.54, "S"), (0, "+V"), (2.54, "G")):
            silk(ch, xc + dx, 8.0, size=0.8)

    # 右侧舵机/遥控：功能 + GPIO 标在排针左前方的空白区（y 对齐到接口中心，
    # 一眼对上哪行是哪路）；S/+V/G 标在焊盘左侧列
    right = [("J8", "舵机 后差速", 34), ("J7", "舵机 前差速", 43),
             ("J11", "遥控 后差速", 52), ("J10", "遥控 前差速", 61),
             ("J9", "遥控 模式", 70)]
    for ref, name, yc in right:
        silk(f"{name} {io_name(padmap[ref]['1'])}", 73, yc)
        for dy, ch in ((-2.54, "S"), (0, "+V"), (2.54, "G")):
            silk(ch, 85.2, yc + dy, size=0.8)

    # 电源条（2×8）：上排是电源、下排全地。每个电源脚把电压标在自己那一列的
    # 正上方（横排 2.54 间距排不下三个字，竖排），下排在两端各标一个 GND——
    # 一眼读出「上排电源、下排地、每列一对」
    j15 = [io_name(padmap["J15"][str(i)]) for i in range(1, 17)]
    if j15 != ["BEC", "GND", "BEC", "GND", "5V", "GND", "5V", "GND",
               "5V", "GND", "3V3", "GND", "3V3", "GND", "3V3", "GND"]:
        raise SystemExit(f"J15 电源条脚序变成 {j15}，丝印与检查要跟着改")
    # 列（西→东）= 奇针号从 15 递减到 1，电源名取各自列的奇针
    for i, x in enumerate((63, 65.54, 68.08, 70.62, 73.16, 75.7, 78.24, 80.78)):
        silk(j15[14 - 2 * i], x, 21.0, rot=90, size=1.0)
    silk("GND", 59.8, 26.54, size=1.0)
    silk("GND", 84.5, 26.54, size=1.0)

    # 电调输入 / 接收机油门（v4，都在左侧通道；J16 的标让开 J2 第 1 针的「IO43」）
    silk("电调 BEC 输入", 5.9, 56.6, size=1.0)
    silk("接收机 油门", 5.0, 69.3, size=1.0)

    # 开发板插座 + 扩展排针：每针网名（开发板插上去前靠它对针位）。
    # 20 针 2.54 间距一行排不下 IO43 这种 4 字符，隔针错两行排。
    # v5：J13/J14 贴到 J2 下面只剩 5.08 的缝，名字排到各自排针的下沿
    # v5：行距收紧后放不下两行 0.8 号字，改成单行 0.58 号（4 字符 2.32mm < 2.54）。
    # 0.58 低于板文件默认的 0.8mm 文字高度下限，生成器会把该下限调到 0.55
    # （见 patch_project_ignore）——丝印密集区的现实折中，板厂工艺无压力
    for ref, y_lab in (("J1", 30.6), ("J2", 56.0), ("J13", 61.4), ("J14", 66.5)):
        for i in range(1, 21):
            silk(io_name(padmap[ref][str(i)]), 12.7 + 2.54 * (i - 1),
                 y_lab, size=0.58)
    # 扩展排针右端标注对应关系（左端被自带外框和 1 脚标占着）
    silk("J13=J1", 69, 66.5, size=0.8)
    silk("J14=J2", 69, 74.8, size=0.8)

    # 电源区
    silk("BEC 6~7.4V", 6.3, 88.9, size=1.1)       # J12 端子下方（端子自带丝印框到 y≈85.4）
    silk("+", 3.5, 86.95, size=1.4, th=0.25)
    silk("-", 8.5, 86.95, size=1.4, th=0.25)
    silk("BEC", 58.5, 83.6, size=1.0)             # LED1：+BEC 上电指示
    silk("5V", 66, 87.2, size=1.0)                # LED6：降压段输出指示
    silk("+BEC", 30, 75.6, size=1.0)              # 底部大电流母线（y=75.8）
    silk("+3V3", 5, 29.6, size=1.0)               # 霍尔上拉汇流（母线 y29.5 的西端）
    silk("+5V", 56, 87.9, size=1.0)
    silk("轮速监测台转接底板 v5", 80, 9.0, size=1.1)


PIN1_ROWS = (33.5, 58.9, 63.98, 69.06)   # J1/J2/J13/J14 的 1 脚都在这四个 y 上（x=12.7）


def check_silk(padmap):
    """几何自检：每条文字对全部焊盘开窗、板边、其他文字做包围盒检查。
    KiCad DRC 也能报同样的错，但在这里拦下改起来快得多。"""
    def advance(ch, size):
        if ord(ch) > 0x2E80:            # CJK：全角等宽
            return 1.15 * size
        if ch == " ":
            return 0.6 * size
        return 1.0 * size

    pads, ref_boxes = [], []
    for ref, lib_id, _v, x, y, rot in PLACEMENT:
        node = load_footprint(lib_id)
        xs, ys, lib_at = [], [], None
        for c in node:
            if isinstance(c, list) and c and c[0] == "pad":
                at = K.get(c, "at")
                xs.append(float(at[1])); ys.append(float(at[2]))
            elif (isinstance(c, list) and c and c[0] == "property"
                  and str(c[1]) == "Reference"):
                lib_at = K.get(c, "at")
        ox, oy = rot_off((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, rot)
        ox, oy = x - ox, y - oy
        for c in node:
            if isinstance(c, list) and c and c[0] == "pad":
                at, sz = K.get(c, "at"), K.get(c, "size")
                px, py = rot_off(float(at[1]), float(at[2]), rot)
                pads.append((f"{ref}.{c[1]}", ox + px, oy + py,
                             float(sz[1]) / 2 + 0.15, float(sz[2]) / 2 + 0.15))
        # 位号文字同样纳入检查：位号压焊盘开窗 = 字被阻焊裁掉，是真缺陷
        lx, ly = ref_local(ref, lib_at)
        rx, ry = rot_off(float(lx), float(ly), rot)
        w = sum(advance(ch, 0.8) for ch in ref) + 0.15
        ref_boxes.append((f"{ref} 位号", ox + rx, oy + ry, w / 2, 1.15 * 0.8 / 2))

    # 四个安装孔现在有真实钻孔和阻焊开窗了，一并纳入检查
    for mx, my in MOUNTING_HOLES:
        pads.append((f"安装孔({mx:g},{my:g})", mx, my, 1.35 + 0.15, 1.35 + 0.15))

    boxes = []
    for text, x, y, rot, size, th in SILK_TEXTS:
        w = sum(advance(ch, size) for ch in text) + th
        h = 1.15 * size + th
        if rot in (90, 270):
            w, h = h, w
        boxes.append((text, x, y, w / 2, h / 2))
    boxes += ref_boxes

    problems = []
    for text, x, y, hw, hh in boxes:
        for name, px, py, phw, phh in pads:
            if abs(x - px) < hw + phw and abs(y - py) < hh + phh:
                problems.append(f"丝印「{text}」压住焊盘 {name} 的开窗")
        if not (0.3 <= x - hw and x + hw <= BOARD_W - 0.3
                and 0.3 <= y - hh and y + hh <= BOARD_H - 0.3):
            problems.append(f"丝印「{text}」出板边安全区 "
                            f"(x {x-hw:.2f}..{x+hw:.2f} y {y-hh:.2f}..{y+hh:.2f})")
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            t1, x1, y1, w1, h1 = boxes[i]
            t2, x2, y2, w2, h2 = boxes[j]
            if abs(x1 - x2) < w1 + w2 and abs(y1 - y2) < h1 + h2:
                problems.append(f"丝印「{t1}」与「{t2}」重叠")
    if problems:
        raise SystemExit("丝印自检不过：\n  " + "\n  ".join(problems))
    print(f"丝印自检通过：{len(boxes)} 条文字，与 {len(pads)} 个焊盘开窗、板边、彼此均无冲突")


def emit_silk(out):
    for i, (text, x, y, rot, size, th) in enumerate(SILK_TEXTS):
        # 这里的输出是手拼文本行（不走 K.dump 的序列化器），引号要自己带上
        assert '"' not in text and "\\" not in text, f"丝印内容不能含引号/反斜杠：{text}"
        out.append(f'\t(gr_text "{text}"')
        at = f"{x:g} {y:g}" + (f" {rot}" if rot else "")
        out.append(f"\t\t(at {at})")
        out.append('\t\t(layer "F.SilkS")')
        out.append(f'\t\t(uuid "{uid(f"silk-{i}-{text}")}")')
        out.append("\t\t(effects")
        out.append(f"\t\t\t(font (size {size:g} {size:g}) (thickness {th:g}))")
        out.append("\t\t)")
        out.append("\t)")
    for i, y in enumerate(PIN1_ROWS):
        out.append("\t(gr_poly")
        out.append(f"\t\t(pts (xy 10.3 {y-0.5:g}) (xy 10.3 {y+0.5:g}) (xy 11.1 {y:g}))")
        out.append("\t\t(stroke (width 0.1) (type default))")
        out.append("\t\t(fill yes)")
        out.append('\t\t(layer "F.SilkS")')
        out.append(f'\t\t(uuid "{uid(f"silk-pin1-{i}")}")')
        out.append("\t)")


# ---------------------------------------------------------------- 板文件组装

def layer_and_setup():
    """层定义与 setup 直接搬官方模板（字节级正确比手写强）。"""
    tpl = K.load(TEMPLATE_PCB)
    layers = K.get(tpl, "layers")
    setup = K.get(tpl, "setup")
    # 去掉模板里的自定义原点（我们不需要）
    setup = [c for c in setup
             if not (isinstance(c, list) and c and c[0] in ("aux_axis_origin", "grid_origin"))]
    return layers, setup


def emit():
    layers, setup = layer_and_setup()
    out = []
    out.append("(kicad_pcb")
    out.append("\t(version 20241229)")
    out.append('\t(generator "pcbnew")')
    out.append('\t(generator_version "9.0")')
    out.append("\t(general")
    out.append("\t\t(thickness 1.6)")
    out.append("\t\t(legacy_teardrops no)")
    out.append("\t)")
    out.append('\t(paper "A3")')
    out.append("\t(title_block")
    out.append('\t\t(title "轮速监测台转接底板")')
    out.append('\t\t(date "2026-10-11")')
    out.append('\t\t(rev "v4")')
    out.append("\t)")
    out.append(K.dump(layers, 1).replace("\n", "\n\t"))
    out.append(K.dump(setup, 1).replace("\n", "\n\t"))
    out.append('\t(net 0 "")')
    for name, code in sorted(NET_CODES.items(), key=lambda kv: kv[1]):
        out.append(f'\t(net {code} "{name}")')

    nets, padmap = NETLIST_DATA
    refs_in_netlist = set(padmap)
    refs_in_place = set(p[0] for p in PLACEMENT)
    if refs_in_netlist != refs_in_place:
        raise SystemExit(f"放置表和网表对不上：网表多 {sorted(refs_in_netlist - refs_in_place)}，"
                         f"放置表多 {sorted(refs_in_place - refs_in_netlist)}")

    for ref, lib_id, value, x, y, rot in PLACEMENT:
        fp = embed_footprint(lib_id, ref, value, x, y, rot, padmap)
        out.append(K.dump(fp, 1).replace("\n", "\n\t"))

    for i, (mx, my) in enumerate(MOUNTING_HOLES):
        out.append(K.dump(mounting_hole(mx, my, str(i)), 1).replace("\n", "\n\t"))

    emit_silk(out)
    emit_routes(out)

    # 板框
    out.append("\t(gr_rect")
    out.append("\t\t(start 0 0)")
    out.append(f"\t\t(end {BOARD_W:g} {BOARD_H:g})")
    out.append("\t\t(stroke (width 0.05) (type default))")
    out.append("\t\t(fill none)")
    out.append('\t\t(layer "Edge.Cuts")')
    out.append(f'\t\t(uuid "{uid("outline")}")')
    out.append("\t)")

    # 背面整板 GND 铺铜
    out.append("\t(zone")
    out.append(f'\t\t(net {NET_CODES["GND"]})')
    out.append('\t\t(net_name "GND")')
    out.append('\t\t(layer "B.Cu")')
    out.append(f'\t\t(uuid "{uid("zone-gnd")}")')
    out.append("\t\t(hatch edge 0.508)")
    out.append("\t\t(connect_pads (clearance 0.508))")
    # 0.2 而不是 KiCad 默认的 0.254：密集区的窄缝（如 J15 电源条下方 ~1mm 宽的
    # 通道）默认厚度填不进去，中区铺铜整体与外围断成两片（DRC 报"未连接项目"）。
    # 0.2 仍 ≥ 设计规则下限（min_track_width 0.2），板厂工艺无压力，铺铜整片连通
    out.append("\t\t(min_thickness 0.2)")
    out.append("\t\t(filled_areas_thickness no)")
    out.append("\t\t(fill (thermal_gap 0.508) (thermal_bridge_width 0.508))")
    out.append("\t\t(polygon")
    out.append(f"\t\t\t(pts (xy 0.5 0.5) (xy {BOARD_W-0.5:g} 0.5) "
               f"(xy {BOARD_W-0.5:g} {BOARD_H-0.5:g}) (xy 0.5 {BOARD_H-0.5:g}))")
    out.append("\t\t)")
    out.append("\t)")
    out.append(")")
    return "\n".join(out) + "\n"


NET_CODES = {}
NETLIST_DATA = (None, None)


def patch_project_ignore(out_path):
    """kicad-cli 的某些操作（导出/DRC）会把 .kicad_pro 重写成完整版，
    顺手把我们对 lib_footprint_mismatch 的降级忽略丢掉（实测，会被重新
    报成 warning）。生成器每次跑完都补一刀——README 的流程里 gen_pcb 永远在
    DRC 之前，这样 DRC 看到的始终是忽略状态。"""
    pro = os.path.join(os.path.dirname(os.path.abspath(out_path)),
                       "wheel_carrier.kicad_pro")
    if not os.path.exists(pro):
        return
    with open(pro, encoding="utf-8") as fh:
        data = json.load(fh)
    ds = data.setdefault("board", {}).setdefault("design_settings", {})
    sev = ds.setdefault("rule_severities", {})
    rules = ds.setdefault("rules", {})
    changed = False
    if sev.get("lib_footprint_mismatch") != "ignore":
        sev["lib_footprint_mismatch"] = "ignore"
        changed = True
    # v5：排针脚名的丝印字压到 0.58（行距收紧后的折中），下限跟着放到 0.55
    if rules.get("min_text_height", 1.0) > 0.55:
        rules["min_text_height"] = 0.55
        changed = True
    if changed:
        with open(pro, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(data, fh, indent=2, ensure_ascii=False)
            fh.write("\n")
        print(f"已把 {os.path.basename(pro)} 的忽略/文字下限设置补回")


def main():
    global NET_CODES, NETLIST_DATA
    out_path = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUT
    nets, padmap = export_netlist()
    NETLIST_DATA = (nets, padmap)
    for i, (name, _members) in enumerate(nets, start=1):
        NET_CODES[name] = i
    build_routes(padmap)
    build_silk(padmap)
    check_silk(padmap)
    text = emit()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    patch_project_ignore(out_path)
    print(f"已生成 {out_path}")
    print(f"元件 {len(PLACEMENT)} 个 / 网络 {len(nets)} 张 / 板框 {BOARD_W:g}×{BOARD_H:g}mm / "
          f"安装孔 {len(MOUNTING_HOLES)} 个(M2.5) / 走线段 {sum(len(p)-1 for *_, p in TRACKS)} 段 / "
          f"过孔 {len(VIAS)} 个 / 丝印 {len(SILK_TEXTS)} 条 + 1 脚标 {len(PIN1_ROWS)} 个")


if __name__ == "__main__":
    main()
