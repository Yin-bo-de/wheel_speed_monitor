#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成轮速监测台转接底板的 PCB（.kicad_pcb，KiCad 9 格式，10 可直接打开）。

流程：
  1. 调 kicad-cli 从原理图导网表——焊盘网络的真源是原理图，绝不手写
  2. 按 PLACEMENT 表把各封装嵌进板文件，焊盘自动挂网（对不上就报错，防漏件）
  3. 画板框、M2.5 安装孔（3 个）、In1/B 层 GND 铺铜 + In2 三路电源分割
  4. 布线（TRACKS/VIAS 表）+ 丝印标注（build_silk；先过 check_silk 几何自检：
     逐条文字对焊盘开窗/板边/彼此做包围盒检查，不过直接构建失败）
  5. kicad-cli DRC + 渲染目检 -> 嘉立创打样文件（导出命令见 README）

机械基准（图纸测量 + 用户尺子实测确认）：
  - 开发板两排针中心距 25.4mm（一英寸）；每排 20 针 2.54 间距
  - 开发板约 28 × 52mm；针 1 距板顶（天线端）约 1.27mm，USB 口在板底
  - 底板 96 × 98.5mm（v7 四层）；3 个 M2.5 安装孔（左下角取消，给 J12 让位）

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
PCB_PY = ("/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
          "Versions/3.9/bin/python3")
REF_4L = "/tmp/wheel_carrier_4l_ref.kicad_pcb"


ROOT_UUID = "11111111-2222-4333-8444-555555555555"
NS = _uuid.UUID(ROOT_UUID)
def uid(tag):
    return str(_uuid.uuid5(NS, "pcb/" + tag))

# ---------------------------------------------------------------- 板级常量

BOARD_W, BOARD_H = 96.0, 98.5           # v5：只压 y 轴（122→98.5），x 轴不动——
                                        # 目标是把高度压进嘉立创 100×100 的免费档
# v6：左下角安装孔取消，给 J12 端子下移让位；其余三个角保留
MOUNTING_HOLES = [(3.5, 3.5), (BOARD_W - 3.5, 3.5),
                  (BOARD_W - 3.5, BOARD_H - 3.5)]
MOUNTING_FP = "MountingHole:MountingHole_2.7mm_M2.5"

R_FP = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal"
D_FP = "Diode_THT:D_DO-201AD_P15.24mm_Horizontal"
CP_FP = "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm"

# 放置表：(位号, 封装, 值, x, y, 旋转)；(x,y) = 焊盘包围盒中心。
# v7 全新布局：顶部霍尔区（每路：接口/上拉/LED/1k 串阻，100nF 贴 MCU 引脚）、
# 中部开发板插座与遥控 J9、右侧四路舵机+电源条、左侧电调/油门、底部电源区。
PLACEMENT = [
    # == 顶部霍尔区：四列（列距 18），从西到东 = 左前/左后/右后/右前。
    #    每列 x 基准 xc：接口在 xc，10k 上拉在 xc-4，330R 在 xc+5、
    #    LED 在 xc+6.35，1k 串阻在 xc+2.54。右前（GPIO14）在最东侧，
    #    与 IO13/IO15 的引出车道拉开距离。 ==
    ("J3",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·左前", 12, 5, 90),
    ("J5",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·左后", 30, 5, 90),
    ("J6",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·右后", 48, 5, 90),
    ("J4",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "霍尔·右前", 64, 5, 90),
    ("R1",  "Resistor_SMD:R_0805_2012Metric", "10k", 8, 13.5, 90),
    ("R3",  R_FP, "10k", 26, 13.5, 90),
    ("R4",  R_FP, "10k", 44, 13.5, 90),
    ("R2",  R_FP, "10k", 61, 13.5, 90),
    ("R7",  R_FP, "330", 18, 11, 90),
    ("R8",  R_FP, "330", 36, 11, 90),
    ("R9",  R_FP, "330", 54, 11, 90),
    ("R6",  R_FP, "330", 70, 11, 90),
    ("R11", R_FP, "1k", 14.54, 13, 270),
    ("R12", R_FP, "1k", 32.54, 13, 270),
    ("R13", R_FP, "1k", 50.54, 13, 270),
    ("R14", R_FP, "1k", 66.54, 13, 270),
    ("LED3", "LED_THT:LED_D5.0mm", "红灯", 19.62, 21, 90),
    ("LED4", "LED_THT:LED_D5.0mm", "红灯", 37.62, 21, 90),
    ("LED5", "LED_THT:LED_D5.0mm", "红灯", 55.62, 21, 90),
    ("LED2", "LED_THT:LED_D5.0mm", "红灯", 71.62, 21, 90),
    # == MCU 侧 100nF（C5~C8）：贴各自 GPIO 进 J1/J2 的位置 ==
    ("C5", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 20.32, 28.0, 270),
    ("C6", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 56.515, 28.0, 270),
    ("C7", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 47.625, 28.0, 270),
    ("C8", "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm", "100nF", 60.96, 52.5, 270),
    # == 开发板插座（间距 25.4 是实测基准；针 1 在左端 x=12.7）
    #    中心坐标 = 针 1 的 x + 24.13（20 针跨 48.26）。J13/J14 仍贴 J2 下方
    #    5.08mm，扇形引出沿用 v6 的"半间距错位车道"（详见 build_routes） ==
    ("J1", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板排母1·右排(IO43列)", 36.83, 33.5, 90),
    ("J2", "Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical", "开发板排母2·左排(3V3列)", 36.83, 58.9, 90),
    # == 右侧：四路舵机自上而下 J7/J8/J10/J11；+5V/GND 直接吃 In2/In1 平面，
    #    只有信号要拉线。J9 遥控三针在霍尔与 MCU 之间（见下） ==
    ("J7",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·前差速", 78, 38, 0),
    ("J8",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·后差速", 78, 47, 0),
    ("J10", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·自控1", 78, 56, 0),
    ("J11", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "舵机·自控2", 78, 65, 0),
    ("J9",  "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "遥控·三通道信号", 30.48, 26.5, 270),
    # == 左侧：电调输入 + 油门直通 ==
    ("J16", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "电调·BEC输入", 6.5, 50, 0),
    ("J17", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", "接收机·油门", 6.5, 62, 0),
    # == 扩展排针（与 J1/J2 一一对应） ==
    ("J13", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·排母1(直连 J1/右排)", 36.83, 63.98, 90),
    ("J14", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical", "扩展·排母2(直连 J2/左排)", 36.83, 69.06, 90),
    # 电源条 2×8：rot 90 ⇒ 8 列 × 2 行；奇针（电源）= 南排 y=31.27，
    # 偶针（GND）= 北排 y=28.73；列（西→东）= BEC×2、5V×3、3V3×3
    ("J15", "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical", "电源排针", 85, 30, 90),
    # == 电源区：沿用 v6 三行摆位，间距按外框实数（D10=10.5、DO-201AD=18.9、
    #    DIN0207=12.3）。J12 位置固定（装车线束基准） ==
    ("J12", "TerminalBlock:TerminalBlock_MaiXu_MX126-5.0-02P_1x02_P5.00mm", "BEC 输入 6~7.4V", 6, 88.58, 0),
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
    ("C4",  "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", "1000µF/16V", 83.5, 91.25, 0),
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
    "C3":   ("5.5", "-5.25"),    # v5：挪到右上方，让开右侧接口的地脚
    "C4":   ("5.5", "-5.25"),    # 原位置压在 C3 的丝印圆上（一个位号压出 12 条 DRC 告警）
    "R10":  ("0.0", "-2.5"),     # v5：R10 换了朝向，位号挪到上方空位
    "D2":   ("0.0", "3.4"),      # v5：R10 挪到附近，D2 位号让到二极管下方
    "J17":  ("2.5", "3"),        # 默认位置压「电调 BEC 输入」的标注
    "J1":   ("2.0", "53.3"),     # 挪到东端（v7 左侧被 J9 占住）
    "J9":   ("-2", "5.48"),      # 屏幕 (27.5,23)，让开焊盘与 C5/LED4
    "U1":   ("-6.6", "-2.5"),    # 默认压在 J9 的 GND 焊盘上（v7 舵机列加长后）
    # v7 顶部霍尔列：接口位号挪到左上方（屏幕 (-5.5,-3.6)；rot90 的局部坐标
    # = (3.6,-5.5)），避开 10k 上拉的顶部焊盘；+V/G/S 小字在另一侧
    "J3":   ("3.6", "-2.96"),
    "J4":   ("3.6", "-2.96"),
    "J5":   ("3.6", "-2.96"),
    "J6":   ("3.6", "-2.96"),
    # v7 右侧舵机：位号放接口东侧 x+3.5（功能名从 x=87.5 起，留空档）
    "J7":   ("3.5", "2.54"),
    "J8":   ("3.5", "2.54"),
    "J10":  ("3.5", "2.54"),
    "J11":  ("3.5", "2.54"),
    # 电源条位号放西端空位（屏幕 (73,28)）
    "J15":  ("3.27", "-3.11"),
    # 霍尔列的电阻位号都挪到本件西/东侧，别压在本体或邻件丝印上
    "R1":   ("0", "-2.5"),
    "R2":   ("5.08", "-2.5"),
    "R3":   ("5.08", "-2.5"),
    "R4":   ("5.08", "-2.5"),
    "R6":   ("1.58", "2.5"),
    "R7":   ("1.58", "2.5"),
    "R8":   ("1.58", "2.5"),
    "R9":   ("1.58", "2.5"),
    "R11":  ("5.08", "3.04"),
    "R12":  ("5.08", "3.04"),
    "R13":  ("5.08", "3.04"),
    "R14":  ("5.08", "3.04"),
    # MCU 侧 100nF 位号放各自电容上方 2.1mm（屏幕 (x,24.4)，rot270 局部 (-2.1,0)），
    # 正好落在 J1 脚名（y=29.5）与电容上焊盘（y=26.5）之间
    "C5":   ("-2.1", "-3.68"),
    "C6":   ("-2.1", "-3.68"),
    "C7":   ("-2.1", "-3.68"),
    "C8":   ("-4.0", "-3.68"),
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
            # 电源/GND 焊盘一律实心连接平面：v7 的 In1/In2 上有大量围栏过孔
            # 和分区边界，花焊盘在密集区很容易只剩一根辐条（v6 为此逐处
            # 手工改过）；统一 zone_connect 2 后由 DRC 兜底，焊接用热风/大功率
            # 烙铁即可。具体网络在下面挂网时判定。
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
            if netname in ("GND", "+BEC", "+5V", "+3V3"):
                c.append(["zone_connect", "2"])
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
# v7 四层：In1 整面 GND、In2 +BEC/+5V/+3V3 分割。
# F.Cu：信号 + 局部电源（霍尔本地链、MCU 侧电容、舵机/遥控源端）
# B.Cu：GND 铺铜 + 跨板长走线（GPIO14 东缘、舵机/遥控横巷、DEV_5V），铺铜自动避让

TRACKS = []          # (net, layer, w, pts)
VIAS = []            # (net, x, y)


def t(net, layer, w, pts):
    TRACKS.append((net, layer, w, pts))


def via(net, x, y):
    VIAS.append((net, x, y))


def _pad_obstacles():
    """全部焊盘/安装孔的（x,y,半径），围栏过滤用。与 embed 同一套中心换算。"""
    out = []
    for ref, lib_id, _v, x, y, rot in PLACEMENT:
        node = load_footprint(lib_id)
        xs, ys = [], []
        for c in node:
            if isinstance(c, list) and c and c[0] == "pad":
                at = K.get(c, "at")
                xs.append(float(at[1]))
                ys.append(float(at[2]))
        ox, oy = rot_off((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, rot)
        ox, oy = x - ox, y - oy
        for c in node:
            if isinstance(c, list) and c and c[0] == "pad":
                at, sz = K.get(c, "at"), K.get(c, "size")
                px, py = rot_off(float(at[1]), float(at[2]), rot)
                # 方焊盘/矩形焊盘的角比半宽远，按半对角线放大 1.42 留余量
                out.append((ox + px, oy + py,
                            max(float(sz[1]), float(sz[2])) / 2 * 1.42))
    for mx, my in MOUNTING_HOLES:
        out.append((mx, my, 1.35))
    return out


def _dist_point_seg(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    dx, dy = px - (ax + t * vx), py - (ay + t * vy)
    return (dx * dx + dy * dy) ** 0.5


def fence_net(net, pitch=2.54, offset=1.0):
    """沿 net 的每条走线两侧布 GND 过孔（0.6/0.3），给敏感信号做地围栏。

    候选点对全部焊盘开窗、安装孔、异网走线、已有过孔做净距过滤，不满足
    直接跳过——密集扇出区天然跳空，DRC 只负责兜底。GND 过孔直连 In1 平面。"""
    pads = _pad_obstacles()
    base, others = [], []
    for onet, _layer, w, pts in TRACKS:
        for a, b in zip(pts[:-1], pts[1:]):
            if onet == net:
                base.append((a, b))
            if onet != "GND":
                # 本网其它段也当障碍：围栏过孔落在自网另一根线的 0.45 内同样是短路
                others.append((a, b, w))
    added = 0
    for a, b in base:
        (ax, ay), (bx, by) = a, b
        L = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
        if L < 1.0:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L
        nx, ny = -uy, ux
        s = 1.27
        while s <= L - 1.27 + 1e-9:
            for side in (-1.0, 1.0):
                cx = ax + ux * s + nx * offset * side
                cy = ay + uy * s + ny * offset * side
                if not (0.6 <= cx <= BOARD_W - 0.6 and 0.6 <= cy <= BOARD_H - 0.6):
                    continue
                if any((cx - px) ** 2 + (cy - py) ** 2 < (pr + 0.55) ** 2
                       for px, py, pr in pads):
                    continue
                if any(_dist_point_seg(cx, cy, oa[0], oa[1], ob[0], ob[1])
                       < ow / 2 + 0.45 and (oa, ob) != (a, b)
                       for oa, ob, ow in others):
                    continue
                if any((cx - vx) ** 2 + (cy - vy) ** 2 < 0.9 ** 2 for _n, vx, vy in VIAS):
                    continue
                via("GND", round(cx, 4), round(cy, 4))
                added += 1
            s += pitch
    return added


def build_routes(padmap):
    wS, w3, w5, wB, wM = 0.3, 0.7, 0.8, 1.2, 1.5   # 信号/3V3/5V/支线/主干
    x0 = 12.7

    # ---- 扩展排针 1:1（沿用 v6 的半间距错位车道）----
    # v7 四层板：J1.20 是 GND，两端焊盘都直连 In1 平面，这一根跨板长线
    # 省掉（两层板时它必须跟着走）；其余 19 针照旧。
    for i in range(20):
        x = round(x0 + 2.54 * i, 4)
        if i < 19:   # J1.20 是 GND，省掉；J2.20=DEV_5V 必须照走
            t(padmap["J1"][str(i + 1)], "F.Cu", wS,
              [(x, 33.5), (x, 56.5), (x - 1.27, 56.5), (x - 1.27, 60.5)])
            via(padmap["J1"][str(i + 1)], round(x - 1.27, 4), 60.5)
            t(padmap["J1"][str(i + 1)], "B.Cu", wS,
              [(x - 1.27, 60.5), (x, 60.5), (x, 63.98)])
        t(padmap["J2"][str(i + 1)], "F.Cu", wS,
          [(x, 58.9), (x, 61.2), (x + 1.27, 61.2), (x + 1.27, 67.4),
           (x, 67.4), (x, 69.06)])

    # ---- +BEC：左脊 → 绕孔 → 顶干线（y=2.0）→ 四路霍尔 +V + J15 BEC ----
    # v7 霍尔接口上移到 y=5，1.5 宽的顶干线走 3.5 会压到焊盘开窗，抬到 2.0；
    # 首跨仍从 y=6.35 绕安装孔（3.5,3.5），x=6.35 再抬回顶。
    bec = "+BEC"
    t(bec, "F.Cu", 1.0, [(1.3, 75.8), (1.3, 6.35)])
    t(bec, "F.Cu", w5, [(1.3, 6.35), (6.35, 6.35), (6.35, 2.0)])
    xs = [6.35, 9.46, 27.46, 45.46, 61.46, 74.5]
    for a, b in zip(xs[:-1], xs[1:]):
        t(bec, "F.Cu", wM, [(a, 2.0), (b, 2.0)])
    for x in (9.46, 27.46, 45.46, 61.46):              # 霍尔 1 脚 = +V
        t(bec, "F.Cu", w5, [(x, 2.0), (x, 5.0)])
    # J15 BEC 两列（x=75.11/77.65，南排 y=31.27）：从顶干线东端下到电源条，
    # 这条竖线走 RF 列 LED（x=72.35）与 J15 西端焊盘之间
    t(bec, "F.Cu", w5, [(74.5, 2.0), (74.5, 31.27)])
    t(bec, "F.Cu", w5, [(74.5, 31.27), (77.65, 31.27)])
    # 底部大电流母线；四层板上它主要靠 In2 +BEC 分担，表面仍留粗线
    t(bec, "F.Cu", wM, [(1.3, 75.8), (87.0, 75.8)])
    for x, y in ((14.88, 80), (37.5, 80.5), (47.92, 80),
                 (74.6, 81), (87.0, 80.25)):
        t(bec, "F.Cu", w5, [(x, 75.8), (x, y)])
    for x in (5.0, 30.0, 55.0):
        via(bec, x, 75.8)                                # 母线 → In2 +BEC
    via(bec, 1.3, 72.5)                                  # 左脊 → In2 +BEC 左带

    # ---- +3V3：顶部 In2 平面直供上拉/指示灯与 J15 3V3；J2.1 打孔走 B 层 ----
    v3 = "+3V3"
    via(v3, 8.0, 31.0)
    t(v3, "B.Cu", w3, [(8.0, 31.0), (8.0, 55.5), (12.7, 55.5), (12.7, 58.9)])
    # 左前上拉（R1）换成 0805 贴片后，焊盘不再是通孔、落不到 In2 平面：
    # 就近打一颗 0.6/0.3 过孔，短线把贴片的 +3V3 端接回 3V3 平面
    t(v3, "F.Cu", wS, [(8.0, 14.4125), (8.0, 16.9)])
    via(v3, 8.0, 16.9)

    # ---- 霍尔四路 ----
    # 每列：接口 S → R11 → GPIOx；HALL_x 同时挂上拉和 LED 链。
    # 上拉的 +3V3 端、330R 的 +3V3 端都靠 In2 顶带，GND 端靠 In1，
    # 所以本地只剩 HALL_x / LED 阳极 / GPIOx 三段要画。左前上拉换成 0805 贴片后
    # 焊盘离接口更近，起始点用 pu_y 单独给（其余三路仍是直插焊盘的上端）。
    def hall_cell(xc, r330, led, hall_net, gpio_net, lane_x, corridor_y,
                  target_x, target_y, via_pt=None, pu_y=8.42):
        t(hall_net, "F.Cu", wS, [(xc - 4, pu_y), (xc - 4, 6.5), (xc + 2.54, 6.5)])
        t(hall_net, "F.Cu", wS, [(xc + 2.54, 5.0), (xc + 2.54, 7.92)])
        t(hall_net, "F.Cu", wS, [(xc, 6.5), (xc, 22.27), (xc + 7.62, 22.27)])
        anode = padmap[r330]["2"]
        t(anode, "F.Cu", wS, [(xc + 6, 5.92), (xc + 7.62, 5.92),
                              (xc + 7.62, 19.73)])
        # 纵向车道在 y=22.27 会被 LED 阴极横线（HALL_x）跨过：在 y=20~24
        # 借一次 B 层，两层互不打架。右后本来就要下 B 层，直接多走一段。
        if via_pt is not None:
            vx, vy = via_pt
            t(gpio_net, "F.Cu", wS, [(xc + 2.54, 18.08), (vx, 18.08), (vx, 20.0)])
            via(gpio_net, vx, 20.0)
            t(gpio_net, "B.Cu", wS, [(vx, 20.0), (vx, vy)])
            t(gpio_net, "B.Cu", wS, [(vx, vy), (target_x, vy), (target_x, target_y)])
        else:
            t(gpio_net, "F.Cu", wS, [(xc + 2.54, 18.08), (lane_x, 18.08),
                                     (lane_x, 20.0)])
            via(gpio_net, lane_x, 20.0)
            t(gpio_net, "B.Cu", wS, [(lane_x, 20.0), (lane_x, 23.05)])
            via(gpio_net, lane_x, 23.05)
            t(gpio_net, "F.Cu", wS, [(lane_x, 23.05), (lane_x, corridor_y),
                                     (target_x, corridor_y), (target_x, target_y)])

    # 左前 / 左后：F 层北侧横巷，越靠西越靠北；右后借一次 B 层；
    # 右前（GPIO14）走东缘长巷，刻意远离 IO13/IO15 的引出车道。
    hall_cell(12, "R7", "LED3", "HALL_LF", "GPIO1", 16.51, 25.5, 17.78, 33.5,
              pu_y=12.5875)
    hall_cell(30, "R8", "LED4", "HALL_LB", "GPIO21", 34.51, 28.5, 53.34, 33.5)
    hall_cell(48, "R9", "LED5", "HALL_RB", "GPIO47", 52.51, 25.0, 50.8, 33.5,
              via_pt=(52.51, 25.0))
    # 右前霍尔列（J4/R2/R6/LED2/R14）：本地 HALL/LED 链，GPIO14 长线见下
    t("HALL_RF", "F.Cu", wS, [(61, 8.42), (61, 6.5), (66.54, 6.5)])
    t("HALL_RF", "F.Cu", wS, [(66.54, 5.0), (66.54, 7.92)])
    t("HALL_RF", "F.Cu", wS, [(64, 6.5), (64, 22.27), (71.62, 22.27)])
    t(padmap["R6"]["2"], "F.Cu", wS, [(70, 5.92), (71.62, 5.92),
                                      (71.62, 19.73)])
    # 右前：R11 下焊盘 → B 层东行 → 东缘 F 层南下 → 底部西行 → 从北侧进 J2.19
    t("GPIO14", "F.Cu", wS, [(66.54, 18.08), (67.85, 18.08)])
    via("GPIO14", 67.85, 18.08)
    t("GPIO14", "B.Cu", wS, [(67.85, 18.08), (95.2, 18.08)])
    via("GPIO14", 95.2, 18.08)
    t("GPIO14", "F.Cu", wS, [(95.2, 18.08), (95.2, 73.5), (63.0, 73.5),
                             (63.0, 57.3), (58.42, 57.3), (58.42, 58.9)])

    # ---- MCU 侧 100nF：电容装在各自 GPIO 进焊盘之前 ----
    t("GPIO1", "F.Cu", wS, [(17.78, 25.5), (20.32, 25.5)])          # C5.1
    t("GPIO21", "F.Cu", wS, [(53.34, 25.5), (56.515, 25.5)])        # C6.1
    t("GPIO21", "F.Cu", wS, [(53.34, 25.5), (53.34, 28.5)])         # 接回横巷
    t("GPIO47", "B.Cu", wS, [(47.625, 25.5), (50.8, 25.5)])         # C7.1
    t("GPIO14", "F.Cu", wS, [(60.96, 50.0), (63.0, 50.0), (63.0, 57.3)])  # C8.1

    # ---- 右侧七路信号（四路舵机 + 遥控三通道） ----
    # 源在 x=78 的焊盘，目标在 J1 第 4~9/15 针。一律：F 层从焊盘向西到各自的
    # 纵向走廊 → 竖直走到 B 层横巷 → 过孔下 B 层 → 横巷到目标 x → 北进焊盘。
    # F 层纵向走廊按源焊盘 y 从北到南递减（下层的源不许穿过上层源的横线），
    # B 层横巷按目标 x 从东到西由北向南排（东边的目标走北巷，西边的目标
    # 的竖线不会穿过东边横巷的东端）。
    right_sig = [
        # (源位号, 源焊盘号, 源 y, F 层走廊 x, B 层横巷 y, 目标 x, 说明)
        ("J7",  "1", 35.46, 74.0, 36.8, 25.4,  "GPIO41 → J1.6"),
        ("J8",  "1", 44.46, 72.5, 37.8, 22.86, "GPIO42 → J1.5"),
        ("J10", "1", 53.46, 71.0, 38.8, 20.32, "IO2 → J1.4"),
        ("J11", "1", 62.46, 69.5, 35.8, 48.26, "IO48 → J1.15"),
    ]
    for ref, pin, sy, cx, ly, tx, _note in right_sig:
        net = padmap[ref][pin]
        t(net, "F.Cu", wS, [(78, sy), (cx, sy), (cx, ly)])
        via(net, cx, ly)
        t(net, "B.Cu", wS, [(cx, ly), (tx, ly), (tx, 33.5)])

    # ---- 遥控 J9（v7 挪到霍尔与 MCU 之间的空带）----
    # rot270 让三针的 x 恰好落在 J1.9/J1.8/J1.7 的目标列（33.02/30.48/
    # 27.94）：每路从焊盘垂直下到 J1 焊盘，不用横巷，也就没有交叉。
    for pin, tx in (("1", 33.02), ("2", 30.48), ("3", 27.94)):
        net = padmap["J9"][pin]
        t(net, "B.Cu", wS, [(tx, 26.5), (tx, 33.5)])

    # ---- 电源局部 ----
    # BEC 输入端是整条功率路径最窄的一环：1.5 宽 + 两个 45° 倒角，
    # 四路舵机堵转时不至于在这段发热/压降
    t("Net-(D1-A)", "B.Cu", wM, [(3.5, 88.58), (3.5, 86.5), (5.5, 84.5),
                                 (28.12, 84.5), (30.12, 82.5), (30.12, 80)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 91), (41.88, 87.8)])
    t("DEV_5V", "B.Cu", 0.5, [(41.88, 87.8), (33.5, 87.8), (33.5, 91)])       # C2.1
    # 沿板底东缘北上后从 J13/J14 东侧走近 J2.20：老路线在 y=58.9 直接横穿
    # J11 信号焊盘（78,59.46），四层板顺手改走 y=75.5 外圈
    t("DEV_5V", "B.Cu", w5, [(41.88, 87.8), (41.88, 97.0), (89.8, 97.0),
                             (89.8, 42.5), (63.0, 42.5), (63.0, 58.9),
                             (60.96, 58.9)])
    sw = "Net-(D3-K)"
    t(sw, "F.Cu", w5, [(76.3, 81), (76.3, 83.5), (17, 83.5),
                       (17, 88.5), (14.88, 88.5), (14.88, 91)])
    t(sw, "F.Cu", wS, [(76.3, 81), (76.3, 83.5)])
    via(sw, 76.3, 83.5)
    t(sw, "B.Cu", wS, [(76.3, 83.5), (63.0, 83.5), (63.0, 91.25)])  # 跳 +5V 汇流，从上面进 L1.1

    # ---- 电调输入 J16 / 接收机 J17 ----
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

    # ---- +5V 局部与 In2 平面 ----
    t("+5V", "F.Cu", w5, [(46.92, 86), (46.92, 84.6), (68.0, 84.6)])
    t("+5V", "F.Cu", w5, [(68.0, 84.6), (68.0, 85.8)])
    t("+5V", "F.Cu", wM, [(68.0, 85.8), (86.0, 85.8)])
    t("+5V", "F.Cu", w5, [(55.62, 84.6), (55.62, 88.5), (57.12, 88.5), (57.12, 91)])  # D2.2
    t("+5V", "F.Cu", w5, [(68.0, 85.8), (68.0, 91.25)])        # L1.2
    t("+5V", "F.Cu", w5, [(81.0, 85.8), (81.0, 91.25)])        # C4.1
    t("+5V", "F.Cu", w5, [(79.7, 85.8), (79.7, 81)])           # U1.FB
    via("+5V", 75.0, 85.8)                                      # 母线 → In2 +5V
    via("+5V", 86.0, 85.8)
    # J15 的 5V 三列（x=80.19/82.73/85.27，南排 y=31.27）：从上方的 In2 +5V
    # 右带打孔上来；竖线落在两列引脚之间的空档，不碰北排 GND 焊盘
    via("+5V", 82.73, 33.2)
    t("+5V", "F.Cu", wS, [(82.73, 33.2), (82.73, 31.27)])
    t("+5V", "F.Cu", w5, [(80.19, 31.27), (85.27, 31.27)])

    t("Net-(LED1-A)", "F.Cu", wS, [(58.08, 80), (58.08, 78.73), (63, 78.73)])
    t("Net-(LED6-A)", "F.Cu", wS, [(57.08, 86), (57.08, 85.5), (59.5, 85.5)])
    via("Net-(LED6-A)", 59.5, 85.5)
    t("Net-(LED6-A)", "B.Cu", wS, [(59.5, 85.5), (59.5, 93.2), (75, 93.2), (75, 92.73)])

    # ---- 信号地围栏（方案第 1 节）----
    # 重点：GPIO14 整条链、IO13/IO15 引出段、四路霍尔、七路舵机/遥控。
    # 过滤规则见 fence_net()；DRC 复核兜底。
    fenced = 0
    for net in ("GPIO14", "HALL_LF", "HALL_LB", "HALL_RB", "HALL_RF",
                "GPIO1", "GPIO21", "GPIO47",
                "GPIO41", "GPIO42", "IO2", "IO48",
                "GPIO38", "GPIO39", "GPIO40", "IO13", "IO15"):
        fenced += fence_net(net)
    print(f"地围栏过孔 {fenced} 个")


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
    # 先对网表：v7 的关键脚序/网络全部钉死。原理图改了而这里没同步时，
    # 构建直接失败——丝印和走线都不会带病出厂。
    expect = {
        "J1.1": "IO43", "J2.1": "+3V3",
        "J1.5": "GPIO42", "J2.19": "GPIO14", "J2.20": "DEV_5V",
        "J3.1": "+BEC", "J3.2": "GND", "J3.3": "HALL_LF",
        "J4.3": "HALL_RF", "J5.3": "HALL_LB", "J6.3": "HALL_RB",
        "R11.1": "HALL_LF", "R11.2": "GPIO1",
        "R12.1": "HALL_LB", "R12.2": "GPIO21",
        "R13.1": "HALL_RB", "R13.2": "GPIO47",
        "R14.1": "HALL_RF", "R14.2": "GPIO14",
        "C5.1": "GPIO1", "C5.2": "GND",
        "C6.1": "GPIO21", "C6.2": "GND",
        "C7.1": "GPIO47", "C7.2": "GND",
        "C8.1": "GPIO14", "C8.2": "GND",
        "J7.1": "GPIO41", "J7.2": "+5V", "J7.3": "GND",
        "J8.1": "GPIO42", "J8.2": "+5V", "J8.3": "GND",
        "J10.1": "IO2", "J10.2": "+5V", "J10.3": "GND",
        "J11.1": "IO48", "J11.2": "+5V", "J11.3": "GND",
        "J9.1": "GPIO38", "J9.2": "GPIO39", "J9.3": "GPIO40",
        "J15.1": "+BEC", "J15.2": "GND", "J15.3": "+BEC", "J15.4": "GND",
        "J15.5": "+5V", "J15.6": "GND", "J15.7": "+5V", "J15.8": "GND",
        "J15.9": "+5V", "J15.10": "GND", "J15.11": "+3V3", "J15.12": "GND",
        "J15.13": "+3V3", "J15.14": "GND", "J15.15": "+3V3", "J15.16": "GND",
        "J16.2": "+BEC", "J17.2": "+BEC",
    }
    for key, want in expect.items():
        ref, pin = key.split(".")
        got = padmap[ref][pin]
        if got != want:
            raise SystemExit(f"{key} 的网名是 {got}，v7 假设 {want}——先同步原理图")

    # 霍尔：功能+GPIO 标在每列 1k/LED 下方空白带；信号名从串阻 MCU 侧取，
    # 不手写。+V/G/S 标在接口焊盘上方。
    halls = [("J3", "R11", "左前霍尔", 12), ("J5", "R12", "左后霍尔", 30),
             ("J6", "R13", "右后霍尔", 48), ("J4", "R14", "右前霍尔", 66)]
    for ref, rser, name, xc in halls:
        # 功能名放本列 LED 与下一列之间（x=xc+12.5），不和电容位号抢 y=24.4 空带
        silk(f"{name} {io_name(padmap[rser]['2'])}", xc + 16.0, 21, size=0.9)
        for dx, ch in ((-2.54, "+V"), (0, "G"), (2.54, "S")):
            silk(ch, xc + dx, 1.4, size=0.8)

    # 右侧四路舵机 + 遥控：功能名放接口东侧（x=87.5），S/+V/G 放西侧 x=74.5。
    right = [("J7", "前差速舵机", 38), ("J8", "后差速舵机", 47),
             ("J10", "自控舵机1", 56), ("J11", "自控舵机2", 65)]
    for ref, name, yc in right:
        silk(f"{name} {io_name(padmap[ref]['1'])}", 87.5, yc, size=0.9)
        for dy, ch in ((-2.54, "S"), (0, "+V"), (2.54, "G")):
            silk(ch, 74.5, yc + dy, size=0.8)
    # 遥控 J9：rot270 三针横排，脚序 1=模式、2=前差速、3=后差速
    silk("遥控 3通道", 30.48, 30.3, size=0.9)
    for px, ch in ((33.02, "模"), (30.48, "前"), (27.94, "后")):
        silk(ch, px, 28.8, size=0.8)

    # 电源条 J15（rot90）：南排=电源、北排=GND；每个电源列在自己下方标电压。
    # 列（西→东）= BEC BEC 5V 5V 5V 3V3 3V3 3V3。
    j15 = [io_name(padmap["J15"][str(i)]) for i in range(1, 17)]
    if j15 != ["BEC", "GND", "BEC", "GND", "5V", "GND", "5V", "GND",
               "5V", "GND", "3V3", "GND", "3V3", "GND", "3V3", "GND"]:
        raise SystemExit(f"J15 电源条脚序变成 {j15}，丝印与检查要跟着改")
    # 电压名竖排在上方（南排电源、北排 GND 的布局 v6 已验证）；2.54 列距
    # 横排放不下三个字，竖排 + 0.8 号正好躲开北排焊盘
    for i in range(8):
        silk(j15[2 * i], 75.11 + 2.54 * i, 26.0, rot=90, size=0.8)
    silk("GND", 72.6, 33.6, size=0.8)
    silk("GND", 93.9, 33.6, size=0.8)

    # 电调输入 / 接收机油门（左侧通道）
    silk("电调 BEC 输入", 5.9, 56.6, size=1.0)
    silk("接收机 油门", 5.0, 68.6, size=1.0)

    # 开发板插座 + 扩展排针：每针网名。20 针 2.54 间距一行排不下 IO43 这种
    # 4 字符，用 0.58 号字（板文件文字下限同步放到 0.55）。
    # J1 的脚名放 J1 上方（电容焊盘之间的空带），其余放各自排针下方。
    for ref, y_lab in (("J1", 36.0), ("J2", 61.5), ("J13", 66.5), ("J14", 71.5)):
        for i in range(1, 21):
            silk(io_name(padmap[ref][str(i)]), 12.7 + 2.54 * (i - 1),
                 y_lab, size=0.58)
    silk("J13=J1", 65.0, 66.5, size=0.8)
    silk("J14=J2", 65.0, 71.5, size=0.8)

    # 电源区
    silk("BEC 6~7.4V", 6.3, 96.3, size=1.1)
    silk("+", 3.5, 94.0, size=1.4, th=0.25)
    silk("-", 8.5, 94.0, size=1.4, th=0.25)
    silk("BEC", 58.5, 83.6, size=1.0)             # LED1：+BEC 上电指示
    silk("5V", 66, 87.2, size=1.0)                # LED6：降压段输出指示
    silk("+BEC", 30, 75.6, size=1.0)              # 底部大电流母线（y=75.8）
    silk("+3V3", 5, 30.8, size=1.0)               # 顶部 3V3 平面
    silk("+5V", 56, 87.9, size=1.0)
    silk("正面插入 USB→", 68, 48, size=1.0)        # 开发板朝上插，USB 朝右
    silk("轮速监测台转接底板 v7（4层）", 45, 96.3, size=1.1)


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

    # 安装孔现在有真实钻孔和阻焊开窗了，一并纳入检查（v6 为 3 个）
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

def ensure_4l_ref():
    """用 KiCad 自带 pcbnew 生成 4 层参考板，只为了拿它「字节级正确」的
    (layers ...) 节点——四层板层号（F=0/In1=4/In2=6/B=2）不是猜的。
    setup/stackup 仍取官方模板，再把两层介质扩成四层。"""
    if os.path.exists(REF_4L):
        try:
            node = K.load(REF_4L)
            names = {str(c[1]) for c in K.get(node, "layers") if isinstance(c, list)}
            if "In1.Cu" in names and "In2.Cu" in names:
                return
        except Exception:
            pass
    code = ("import pcbnew;b=pcbnew.BOARD();b.SetCopperLayerCount(4);"
            f"pcbnew.SaveBoard({REF_4L!r},b)")
    subprocess.run([PCB_PY, "-c", code], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def layer_and_setup():
    ensure_4l_ref()
    ref = K.load(REF_4L)
    layers = copy.deepcopy(K.get(ref, "layers"))
    for c in layers:
        if isinstance(c, list) and len(c) >= 3 and str(c[1]) in ("In1.Cu", "In2.Cu"):
            c[2] = "power"          # 内层是整面/分割平面，不是信号层
    tpl = K.load(TEMPLATE_PCB)
    setup = copy.deepcopy(K.get(tpl, "setup"))
    setup = [c for c in setup
             if not (isinstance(c, list) and c and c[0] in ("aux_axis_origin", "grid_origin"))]
    stackup = K.get(setup, "stackup")
    old = {str(c[1]): c for c in stackup if isinstance(c, list) and c and c[0] == "layer"}

    def cu(name, th="0.035"):
        return ["layer", K.Q(name), ["type", K.Q("copper")], ["thickness", th]]

    def diel(name, typ, th):
        return ["layer", K.Q(name), ["type", K.Q(typ)], ["thickness", th],
                ["material", K.Q("FR4")], ["epsilon_r", "4.5"], ["loss_tangent", "0.02"]]

    # 按嘉立创 4 层 1.6mm 标准叠层 JLC04161H-7628 写：外层 1oz、内层 0.5oz、
    # 7628 半固化片 0.2104、芯板 1.065；加两面 0.01 阻焊约 1.61。
    # 之前写的 0.1/1.24/0.1、内层 1oz 不是厂家标准叠层，下单时会名实不符。
    new_layers = [
        copy.deepcopy(old["F.SilkS"]),
        copy.deepcopy(old["F.Paste"]),
        copy.deepcopy(old["F.Mask"]),
        cu("F.Cu"),
        diel("dielectric 1", "prepreg", "0.2104"),
        cu("In1.Cu", "0.0152"),
        diel("dielectric 2", "core", "1.065"),
        cu("In2.Cu", "0.0152"),
        diel("dielectric 3", "prepreg", "0.2104"),
        cu("B.Cu"),
        copy.deepcopy(old["B.Mask"]),
        copy.deepcopy(old["B.Paste"]),
        copy.deepcopy(old["B.SilkS"]),
    ]
    keep = [c for c in stackup[1:]
            if not (isinstance(c, list) and c and c[0] == "layer")]
    stackup[1:] = new_layers + keep     # 保留 stackup 头，别把节点名一起换掉
    return layers, setup


def emit_zone(out, net, layer, pts, tag):
    out.append("\t(zone")
    out.append(f'\t\t(net {NET_CODES[net]})')
    out.append(f'\t\t(net_name "{net}")')
    out.append(f'\t\t(layer "{layer}")')
    out.append(f'\t\t(uuid "{uid("zone-" + tag)}")')
    out.append("\t\t(hatch edge 0.508)")
    out.append("\t\t(connect_pads (clearance 0.508))")
    # 0.2 而不是 KiCad 默认的 0.254：密集区窄缝填不进去会把整张平面分裂
    # （v6 的 B 层地铜实测踩过），0.2 仍 ≥ 设计规则下限。四层板内层同样适用。
    out.append("\t\t(min_thickness 0.2)")
    out.append("\t\t(filled_areas_thickness no)")
    out.append("\t\t(fill (thermal_gap 0.508) (thermal_bridge_width 0.508))")
    out.append("\t\t(polygon")
    pts_s = " ".join(f"(xy {x:g} {y:g})" for x, y in pts)
    out.append(f"\t\t\t(pts {pts_s})")
    out.append("\t\t)")
    out.append("\t)")


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
    out.append('\t\t(date "2026-11-23")')
    out.append('\t\t(rev "v7")')
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

    # ---- 铺铜 ----
    # B.Cu：整板 GND（和 In1 一起兜回流，顶/底 GND 焊盘都能就近下去）
    emit_zone(out, "GND", "B.Cu",
              [(0.5, 0.5), (BOARD_W - 0.5, 0.5),
               (BOARD_W - 0.5, BOARD_H - 0.5), (0.5, BOARD_H - 0.5)], "gnd-b")
    # In1：整面 GND。所有 GND 焊盘是 THT，直接落在平面上，不再靠表层长途回流
    emit_zone(out, "GND", "In1.Cu",
              [(0.5, 0.5), (BOARD_W - 0.5, 0.5),
               (BOARD_W - 0.5, BOARD_H - 0.5), (0.5, BOARD_H - 0.5)], "gnd-in1")
    # In2：电源分割。顶带 3V3（霍尔上拉/指示灯 + J15 3V3 列），左下 L 形
    # +BEC（电源区 + 左脊 J16/J17），右下 L 形 +5V（降压段 + 舵机列 + J15 5V）。
    # 信号全在 F/B 两层，分割线不需要给信号让路；跨分割的只有过孔，
    # 平面会按净距自动掏洞。
    emit_zone(out, "+3V3", "In2.Cu",
              [(0.5, 0.5), (BOARD_W - 0.5, 0.5),
               (BOARD_W - 0.5, 32.5), (0.5, 32.5)], "v3-in2")
    emit_zone(out, "+BEC", "In2.Cu",
              [(0.5, 32.5), (10.5, 32.5), (10.5, 72.0), (60.0, 72.0),
               (60.0, BOARD_H - 0.5), (0.5, BOARD_H - 0.5)], "bec-in2")
    emit_zone(out, "+5V", "In2.Cu",
              [(75.5, 32.5), (BOARD_W - 0.5, 32.5),
               (BOARD_W - 0.5, BOARD_H - 0.5), (60.0, BOARD_H - 0.5),
               (60.0, 72.0), (75.5, 72.0)], "v5-in2")
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
