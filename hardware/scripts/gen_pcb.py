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

BOARD_W, BOARD_H = 96.0, 122.0          # 草案尺寸，渲染后调
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
    ("R2",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 8, 13, 0),
    ("R1",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 21, 13, 0),
    ("R3",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 34, 13, 0),
    ("R4",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "10k", 47, 13, 0),
    ("R6",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 8, 20, 0),
    ("R7",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 21, 20, 0),
    ("R8",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 34, 20, 0),
    ("R9",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "330", 47, 20, 0),
    ("LED2", "LED_THT:LED_D5.0mm", "红灯", 8, 27, 90),
    ("LED3", "LED_THT:LED_D5.0mm", "红灯", 21, 27, 90),
    ("LED4", "LED_THT:LED_D5.0mm", "红灯", 34, 27, 90),
    ("LED5", "LED_THT:LED_D5.0mm", "红灯", 47, 27, 90),
    # == 开发板插座（间距 25.4 是实测基准；针 1 在左端 x=12.7）
    #    中心坐标 = 针 1 的 x + 24.13（20 针跨 48.26） ==
    ("J1", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板左排 20P", 36.83, 40.64, 90),
    ("J2", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板右排 20P", 36.83, 66.04, 90),
    # == 右侧：舵机 + 遥控。自上而下的顺序是 J8→J7→J11→J10→J9：
    #    按各自 J2 目标针的 x 递增排列，B 层那五条走线才不打结（布局服务布线） ==
    ("J8",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·后差速", 88, 46, 0),
    ("J7",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·前差速", 88, 56, 0),
    ("J11", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·后差速", 88, 66, 0),
    ("J10", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·前差速", 88, 76, 0),
    ("J9",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·模式", 88, 86, 0),
    # == 扩展排针（与 J1/J2 一一对应）+ 电源排针 ==
    ("J13", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·左排(直连 J1)", 36.83, 84, 90),
    ("J14", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·右排(直连 J2)", 36.83, 92, 90),
    ("J15", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", "电源排针", 70, 24, 90),
    # == 电源区：三行摆位，间距按外框实数（D10=10.5、DO-201AD=18.9、DIN0207=12.3） ==
    ("J12", "TerminalBlock:TerminalBlock_MaiXu_MX126-5.0-02P_1x02_P5.00mm", "BEC 输入 6~7.4V", 6, 101, 0),
    ("D1",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "1N5824", 22.5, 99.5, 0),
    ("C1",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "1000µF/16V", 40, 100, 0),
    ("R5",  "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "1k", 53, 99.5, 0),
    ("LED1", "LED_THT:LED_D5.0mm", "绿灯", 63, 99.5, 90),
    ("D3",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "1N5822", 22.5, 110.5, 0),
    ("C2",  "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 36, 110.5, 0),
    ("D2",  "Diode_THT:D_DO-201AD_P15.24mm_Horizontal", "1N5822", 49.5, 110.5, 0),
    ("R10", "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal", "1k", 62, 110.5, 270),
    ("LED6", "LED_THT:LED_D5.0mm", "绿灯", 69, 115, 90),
    ("U1",  "Package_TO_SOT_THT:TO-220-5_Vertical", "LM2596T-5", 78, 100.5, 0),
    ("C3",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "220µF/25V", 89.5, 99.75, 0),
    ("L1",  "Inductor_THT:L_Radial_D10.0mm_P5.00mm_Fastron_07P", "47µH", 78, 110.75, 0),
    ("C4",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "220µF/16V", 89.5, 110.75, 0),
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
    "LED6": ("-3.03", "-3.5"),   # 同上，原位置压 R10.2
    "C1":   ("6.5", "-5.5"),     # 原位置压在 J14 排针外框线上
    "C4":   ("5.5", "-5.25"),    # 原位置压在 C3 的丝印圆上（一个位号压出 12 条 DRC 告警）
    "R10":  ("7.0", "-3.4"),     # 原位置压自己的丝印框（R10 转了 270°，位号没跟着让位）
    "J15":  ("0", "-3.2"),       # 原位置压自己的外框角
}


def ref_local(ref, lib_at):
    """位号的局部坐标：库里默认位置 + 上面的覆写规则（embed 和自检共用，
    保证「自检看到的位号位置」就是「文件里写的位置」）。"""
    if ref in REF_MOVES:
        return REF_MOVES[ref]
    if ref.startswith("LED"):
        return ("0", "-6.0")
    if ref in ("J1", "J2", "J13", "J14"):
        return ("0", "-4.0")
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
    # 焊盘挂网；U1（官方 TO-220-5）焊盘环比板厂最小环宽还差一点，加粗到 0.9；
    # 右侧接口的 GND 脚四周被走线夹着，花焊盘只剩一根辐条，改实心连接
    wanted = pad_nets.get(ref, {})
    for c in kids:
        if isinstance(c, list) and c and c[0] == "pad":
            if ref == "U1":
                # 原封装是椭圆孔 0.8 宽，小盘环过不了板厂 DFM——放大到 1.4（环 0.3）
                for sub in c:
                    if isinstance(sub, list) and sub and sub[0] == "size":
                        sub[1] = "1.4"
                        sub[2] = "1.4"
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
        if isinstance(c, list) and c and c[0] in ("version", "generator", "layer",
                                                 "property", "pad"):
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
          [(x, 40.64), (x, 62.23), (x - 1.27, 62.23), (x - 1.27, 76.5)])
        via(padmap["J1"][str(i + 1)], round(x - 1.27, 4), 76.5)
        t(padmap["J1"][str(i + 1)], "B.Cu", wS,
          [(x - 1.27, 76.5), (x - 1.27, 82.55), (x, 82.55), (x, 84)])
        t(padmap["J2"][str(i + 1)], "F.Cu", wS,
          [(x, 66.04), (x, 80.01), (x + 1.27, 80.01), (x + 1.27, 86.36),
           (x, 86.36), (x, 92)])

    # ---- +BEC：左脊上到顶、顶干线横穿、右走廊下、底部大电流母线，
    #      全程绕开四个安装孔与焊盘（都是对着 DRC 逐段修出来的）
    bec = "+BEC"
    t(bec, "F.Cu", 1.0, [(1.3, 95.9), (1.3, 3.5)])             # 左脊（让开板边 0.5）
    xs = [1.3, 11, 21, 34, 47, 66.19, 89.8]
    for a, b in zip(xs[:-1], xs[1:]):                          # 顶干线（分段在分接点处分断）
        t(bec, "F.Cu", wM, [(a, 3.5), (b, 3.5)])
    for x in (11, 21, 34, 47):                                 # 顶部四个接口 +V
        t(bec, "F.Cu", w5, [(x, 3.5), (x, 6)])
    t(bec, "F.Cu", w5, [(66.19, 3.5), (66.19, 24)])            # 下到 J15.1
    ys = [46, 56, 66, 76, 86]      # 连接器中心针 = +V（中心锚定后针 2 就在放置点）
    t(bec, "F.Cu", wB, [(89.8, 3.5), (89.8, ys[0])])           # 右走廊
    for a, b in zip(ys[:-1], ys[1:]):
        t(bec, "F.Cu", wB, [(89.8, a), (89.8, b)])
    for y in ys:                                               # 右侧五个接口 +V
        t(bec, "F.Cu", w5, [(89.8, y), (88, y)])
    t(bec, "F.Cu", wM, [(1.3, 95.9), (87, 95.9)])              # 底部母线
    for x, y in ((14.88, 99.5), (37.5, 100), (47.92, 99.5),
                 (74.6, 100.5), (87, 99.75)):                  # 分接到各耗电点
        t(bec, "F.Cu", w5, [(x, 95.9), (x, y)])

    # ---- +3V3：底部汇流（y32，让开霍尔过孔排）+ 每路一列（列线直穿
    #      自己的两个上拉焊盘），再下到 J1.1、东行到 J15.3
    v3 = "+3V3"
    for x in (2.92, 15.92, 28.92, 41.92):
        t(v3, "F.Cu", w3, [(x, 13), (x, 33.5)])
    t(v3, "F.Cu", w3, [(2.92, 33.5), (41.92, 33.5)])
    t(v3, "F.Cu", w3, [(2.92, 33.5), (2.92, 36), (12.7, 36), (12.7, 40.64)])
    t(v3, "F.Cu", w3, [(41.92, 33.5), (41.92, 36), (71.27, 36), (71.27, 24)])

    # ---- 霍尔四路：F.Cu 走本地（信号脚→10k→指示灯），过孔下 B.Cu 后
    #      走"东侧三列 + 南部三巷 + 三根上行短腿"回 J2/J14。列/巷/腿的
    #      x、y 是联立解出来的：任意两段都不相交（跟 DRC 磨了两轮）。
    #      GPIO14 的腿接在自己在扩展排针的 B 层绕行段上（同网，天然汇合）。
    hall = {
        14: ([(8.46, 6), (8.46, 9)], [(8.46, 9), (13.08, 9)],
             [(13.08, 9), (13.08, 13)], [(8.46, 9), (14.5, 9)],
             [(14.5, 9), (14.5, 29.6)], [(14.5, 29.6), (8, 29.6), (8, 28.27)],
             ("V", (14.5, 29.6)),
             ("B", [(14.5, 29.6), (92.5, 29.6)]),
             ("V", (92.5, 29.7)),
             ("F", [(92.5, 29.7), (92.5, 32.7)]),
             ("V", (92.5, 32.7)),
             ("B", [(92.5, 32.7), (92.5, 94), (57.15, 94), (57.15, 82.55)])),
        1:  ([(18.46, 6), (18.46, 9)], [(18.46, 9), (26.08, 9)],
             [(26.08, 9), (26.08, 13)], [(18.46, 9), (27.5, 9)],
             [(27.5, 9), (27.5, 28.27)], [(27.5, 28.27), (21, 28.27)],
             ("V", (27.5, 28.27)),
             ("B", [(27.5, 28.27), (24.13, 28.27), (24.13, 28.8)]),
             ("V", (24.13, 28.8)),
             ("F", [(24.13, 28.8), (24.13, 30.3)]),
             ("V", (24.13, 30.3)),
             ("B", [(24.13, 30.3), (24.13, 64), (17.78, 64), (17.78, 66.04)])),
        21: ([(31.46, 6), (31.46, 9)], [(31.46, 9), (39.08, 9)],
             [(39.08, 9), (39.08, 13)], [(31.46, 9), (40.5, 9)],
             [(40.5, 9), (40.5, 28.27)], [(40.5, 28.27), (34, 28.27)],
             [(40.5, 28.27), (40.5, 30.4)],
             ("V", (40.5, 30.4)),
             ("B", [(40.5, 30.4), (93.3, 30.4)]),
             ("V", (93.3, 30.4)),
             ("F", [(93.3, 30.4), (93.3, 32.1)]),
             ("V", (93.3, 32.1)),
             ("B", [(93.3, 32.1), (93.3, 95.5), (53.34, 95.5), (53.34, 92)])),
        47: ([(44.46, 6), (44.46, 9)], [(44.46, 9), (52.08, 9)],
             [(52.08, 9), (52.08, 13)], [(44.46, 9), (53.5, 9)],
             [(53.5, 9), (53.5, 28.27)], [(53.5, 28.27), (47, 28.27)],
             [(53.5, 28.27), (53.5, 31.4)],
             ("V", (53.5, 31.4)),
             ("B", [(53.5, 31.4), (94.1, 31.4), (94.1, 96.6), (50.8, 96.6),
                    (50.8, 92)])),
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
        t(net, "F.Cu", wS, [(xr, 20), (xr, 21.5), (xl - 2.54, 21.5),
                            (xl - 2.54, 25.73), (xl, 25.73)])

    # ---- 遥控/舵机五路：信号脚（p1，中心锚定后在上方一针）→ 短 B 段 →
    #      F 走廊 → 换回 B 走南部横巷 → 上到 J2 目标针。J9 的走廊最东
    #      （它的横巷要跨过其他四条走廊不现实，索性从外面绕）
    right5 = {
        "J8":  [(88, 43.46), (85.9, 43.46), (85.9, 74.8), (22.86, 74.8), (22.86, 66.04)],
        "J7":  [(88, 53.46), (84.5, 53.46), (84.5, 72.8), (25.4, 72.8), (25.4, 66.04)],
        "J11": [(88, 63.46), (83.1, 63.46), (83.1, 72.0), (27.94, 72.0), (27.94, 66.04)],
        "J10": [(88, 73.46), (81.7, 73.46), (81.7, 70.6), (30.48, 70.6), (30.48, 66.04)],
        # J9 走廊到自己的横巷（y69.2），向西直接汇进 J2.9（同网）的扩展竖线
        "J9":  [(88, 83.46), (86.8, 83.46), (86.8, 69.2), (33.02, 69.2), (33.02, 66.04)],
    }
    for ref, pts in right5.items():
        net = padmap[ref]["1"]
        (x0_, y0_), (x1_, y1_) = pts[0], pts[1]
        t(net, "B.Cu", wS, [(x0_, y0_), (x1_, y1_)])
        via(net, x1_, y1_)
        (x2_, y2_) = pts[2]
        t(net, "F.Cu", wS, [(x1_, y1_), (x2_, y2_)])
        via(net, x2_, y2_)
        # J9 的横巷在 y76 与 J2.9 的扩展竖线同网汇合：B 段之后回到 F 汇入
        if ref == "J9":
            (x3_, y3_) = pts[3]
            t(net, "B.Cu", wS, [(x2_, y2_), (x3_, y3_)])
            via(net, x3_, y3_)
            t(net, "F.Cu", wS, [(x3_, y3_), pts[4]])
            continue
        t(net, "B.Cu", wS, [(x2_, y2_)] + list(pts[3:]))

    # ---- 电源局部 ----
    # VIN_RAW 与 DEV_5V 整段走 B 层（B 层西侧/东缘当时都是空的），
    # 避免与 F 层的 +BEC 母线、+5V 汇流交叉
    t("Net-(D1-A)", "B.Cu", 1.0, [(3.5, 101), (3.5, 103.2), (30.12, 103.2), (30.12, 99.5)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 110.5), (41.88, 106.8)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 106.8), (33.5, 106.8), (33.5, 110.5)])  # C2.1
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 106.8), (89.8, 106.8), (89.8, 97.8)])
    via("DEV_5V", 89.8, 97.8)
    t("DEV_5V", "F.Cu", w5, [(89.8, 97.8), (89.8, 93.2)])                 # 跳三条霍尔横巷
    via("DEV_5V", 89.8, 93.2)
    t("DEV_5V", "B.Cu", w5, [(89.8, 93.2), (89.8, 38), (60.96, 38),
                             (60.96, 40.64)])                              # 东缘北上到 J1.20
    sw = "Net-(D3-K)"
    t(sw, "F.Cu", w5, [(76.3, 100.5), (76.3, 103.5), (17, 103.5),
                       (17, 108), (14.88, 108), (14.88, 110.5)])
    t(sw, "F.Cu", wS, [(76.3, 100.5), (76.3, 98.5), (85.2, 98.5), (85.2, 107.6)])
    via(sw, 85.2, 107.6)
    t(sw, "B.Cu", wS, [(85.2, 107.6), (85.2, 109.6)])                     # 跳 +5V 汇流
    via(sw, 85.2, 109.6)
    t(sw, "F.Cu", wS, [(85.2, 109.6), (85.2, 113), (75.5, 113), (75.5, 110.75)])
    t("+5V", "F.Cu", w5, [(57.12, 110.5), (57.12, 108.5), (95.0, 108.5)])
    t("+5V", "F.Cu", w5, [(62, 108.5), (62, 105.42)])          # R10.1（旋转 270：1 号脚朝上）
    t("+5V", "F.Cu", w5, [(87, 108.5), (87, 110.75)])          # C4.1
    t("+5V", "F.Cu", w5, [(80.5, 108.5), (80.5, 110.75)])      # L1.2
    t("+5V", "F.Cu", w5, [(83.5, 108.5), (83.5, 102.3), (79.7, 102.3), (79.7, 100.5)])  # U1.FB
    t("+5V", "F.Cu", w5, [(95.0, 108.5), (95.0, 20)])
    via("+5V", 95.0, 20)
    t("+5V", "B.Cu", w5, [(95.0, 20), (68.73, 20), (68.73, 24)])   # Б 层跳开 +BEC 走廊
    t("Net-(LED1-A)", "F.Cu", wS, [(58.08, 99.5), (58.08, 98.23), (63, 98.23)])
    t("Net-(LED6-A)", "F.Cu", wS, [(62, 115.58), (66, 115.58), (66, 113.73), (69, 113.73)])
    for r_ref, led_ref, xr, xl in (("R6", "LED2", 13.08, 8), ("R7", "LED3", 26.08, 21),
                                   ("R8", "LED4", 39.08, 34), ("R9", "LED5", 52.08, 47)):
        net = padmap[r_ref]["2"]
        t(net, "F.Cu", wS, [(xr, 20), (xr, 23.5), (xl, 23.5), (xl, 25.73)])
    # R5/LED1 的电源指示链（+BEC 经 R5 到 LED1）——见上方 Net-(LED1-A) 一条
    # LED1 的 GND 脚被周围走线围在一个铺铜孤岛里，补一根到 C1 地脚的短走线
    t("GND", "F.Cu", 0.5, [(63, 100.77), (63, 101.8), (42.5, 101.8), (42.5, 100)])


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
        silk(f"{name} {io_name(padmap[ref]['1'])}", col, 31.7)
        for dx, ch in ((-2.54, "S"), (0, "+V"), (2.54, "G")):
            silk(ch, xc + dx, 8.0, size=0.8)

    # 右侧舵机/遥控：功能 + GPIO 标在排针左前方的空白区（y 对齐到接口中心，
    # 一眼对上哪行是哪路）；S/+V/G 标在焊盘左侧列
    right = [("J8", "舵机 后差速", 46), ("J7", "舵机 前差速", 56),
             ("J11", "遥控 后差速", 66), ("J10", "遥控 前差速", 76),
             ("J9", "遥控 模式", 86)]
    for ref, name, yc in right:
        silk(f"{name} {io_name(padmap[ref]['1'])}", 73, yc)
        for dy, ch in ((-2.54, "S"), (0, "+V"), (2.54, "G")):
            silk(ch, 85.2, yc + dy, size=0.8)

    # J15 电源排针：每脚一个竖排词标（正下方，方向与排针垂直）
    j15 = [io_name(padmap["J15"][str(i)]) for i in range(1, 5)]
    if j15 != ["BEC", "5V", "3V3", "GND"]:
        raise SystemExit(f"J15 电源排针的脚序变成 {j15} 了，丝印标注要跟着改")
    for i, word in enumerate(j15):
        # 3V3 让开自己那根 +3V3 竖线 1mm，免得文字跟导线叠着（其余三脚线不到这里）
        dx = 1.0 if word == "3V3" else 0.0
        silk(word, 66.19 + 2.54 * i + dx, 27.2, rot=90, size=1.0)
    silk("电源排针", 78.7, 21.3)

    # 开发板插座 + 扩展排针：每针网名（开发板插上去前靠它对针位）。
    # 20 针 2.54 间距一行排不下 IO43 这种 4 字符，隔针错两行排
    for ref, y_odd, y_even in (("J1", 38.5, 37.3), ("J2", 64.05, 62.95),
                               ("J13", 81.5, 79.6), ("J14", 89.7, 87.5)):
        for i in range(1, 21):
            silk(io_name(padmap[ref][str(i)]), 12.7 + 2.54 * (i - 1),
                 y_odd if i % 2 else y_even, size=0.8)
    # 扩展排针右端标注对应关系（左端被自带外框和 1 脚标占着）
    silk("J13=J1", 69, 84, size=0.8)
    silk("J14=J2", 69, 92, size=0.8)

    # 电源区
    silk("BEC 6~7.4V", 6.3, 108.4, size=1.1)      # J12 端子下方（端子自带丝印框到 y≈104.9）
    silk("+", 3.5, 106.45, size=1.4, th=0.25)
    silk("-", 8.5, 106.45, size=1.4, th=0.25)
    silk("BEC", 68.5, 99.4, size=1.0)             # LED1：+BEC 上电指示
    silk("5V", 69, 118.4, size=1.0)               # LED6：降压段输出指示
    silk("+BEC", 30, 94.4, size=1.0)              # 底部大电流母线
    silk("+3V3", 20, 34.75, size=1.0)             # 霍尔上拉汇流（母线 y33.5 的正下方空档）
    silk("+5V", 67, 107.2, size=1.0)
    silk("轮速监测台转接底板 v3", 80, 9.0, size=1.1)


PIN1_ROWS = (40.64, 66.04, 84.0, 92.0)   # J1/J2/J13/J14 的 1 脚都在这四个 y 上（x=12.7）


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
    out.append('\t\t(date "2026-10-10")')
    out.append('\t\t(rev "v3")')
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
    out.append("\t\t(min_thickness 0.254)")
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
    print(f"已生成 {out_path}")
    print(f"元件 {len(PLACEMENT)} 个 / 网络 {len(nets)} 张 / 板框 {BOARD_W:g}×{BOARD_H:g}mm / "
          f"安装孔 {len(MOUNTING_HOLES)} 个(M2.5) / 走线段 {sum(len(p)-1 for *_, p in TRACKS)} 段 / "
          f"过孔 {len(VIAS)} 个 / 丝印 {len(SILK_TEXTS)} 条 + 1 脚标 {len(PIN1_ROWS)} 个")


if __name__ == "__main__":
    main()
