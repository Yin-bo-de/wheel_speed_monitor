#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成「轮速监测台转接底板」原理图（KiCad 9 格式，KiCad 10 可直接打开）。

用法：
    python3 gen_schematic.py [输出路径]
    默认输出 ../wheel_carrier/wheel_carrier.kicad_sch

图纸内容全部由本文件的常量定义（引脚映射 / 元件表 / 布局坐标 / 网络标签）：
- 开发板两排 20 针排母（J1/J2）：针号 = 物理位置，名 = 板上 GPIO 功能；
  针号 1 从开发板远离 USB 口的一端数起。
- 霍尔×4 / 舵机×2 / 遥控×3，全部 3 针接口，统一「信号 / +5V / GND」。
- 5V 输入 → 防反接二极管 → 5V 主路（大电容 + 指示灯 + 电源旗标）；
  经第二只二极管给开发板 5V 脚（防止 USB 与底板互相倒灌）。
- 霍尔信号各配 10k 上拉到 +3V3（上拉电压取 3.3V，传感器 VCC 走 5V——
  A3144 开集电极输出，高电平由 3.3V 上拉决定，对 GPIO 安全）。

坐标全部落在 1.27mm 网格（g() 会断言）；uuid 用 uuid5 从固定种子派生，
重跑生成时同一元素的 uuid 不变，git diff 干净。

改动图纸的正确姿势：优先改本文件常量重跑；也允许直接在 KiCad 里改
（改完记得把改动同步回这里，否则下次重跑会覆盖）。
"""
import copy
import os
import sys
import uuid as _uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_lib as K

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "wheel_carrier", "wheel_carrier.kicad_sch"))
SYMDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"
LIBFILES = {
    "Device": "Device.kicad_sym",
    "Connector_Generic": "Connector_Generic.kicad_sym",
    "Connector": "Connector.kicad_sym",
    "Regulator_Switching": "Regulator_Switching.kicad_sym",
    "power": "power.kicad_sym",
}

PROJECT = "wheel_carrier"
ROOT_UUID = "11111111-2222-4333-8444-555555555555"
NS = _uuid.UUID(ROOT_UUID)

TITLE = "轮速监测台转接底板"
DATE = "2026-10-10"
REV = "v3"
PAPER = "A3"


def uid(tag):
    return str(_uuid.uuid5(NS, tag))


def g(v):
    """把坐标对齐检查钉死在 1.27 网格上——图里任何一根线错位半格，
    KiCad 就不认为是同一个连接点，这类错误只有渲染出来才看得出来，
    所以在生成期直接断言掉。"""
    n = round(float(v) / 1.27)
    snapped = n * 1.27
    assert abs(snapped - float(v)) < 1e-6, f"坐标 {v} 不在 1.27 网格上"
    return round(snapped, 4)


def f(v):
    s = f"{float(v):.4f}".rstrip("0").rstrip(".")
    return s if s else "0"


# ---------------------------------------------------------------- 库符号提取

_libcache = {}


def _lib(name):
    if name not in _libcache:
        if name == "wheel_carrier":
            _libcache[name] = custom_lib_node()
        else:
            _libcache[name] = K.load(os.path.join(SYMDIR, LIBFILES[name]))
    return _libcache[name]


def lib_symbol_text(lib_id):
    """取库中符号（自动展开 extends 派生），改名成 '库:符号' 完整块文本，
    供嵌进 lib_symbols。"""
    lib, name = lib_id.split(":")
    node = copy.deepcopy(K.resolve_symbol(_lib(lib), name))
    node[1] = K.Q(lib_id)
    text = K.dump(node, 1)
    return "\n".join("  " + line for line in text.splitlines())


def _symbol_node(lib_id):
    lib, name = lib_id.split(":")
    return K.resolve_symbol(_lib(lib), name)


def lib_pins(lib_id):
    """0° 摆放时各引脚连接点相对符号原点的偏移（图纸坐标，y 向下）。"""
    node = _symbol_node(lib_id)
    pins = {}
    for unit in K.get_all(node, "symbol"):
        for p in K.get_all(unit, "pin"):
            num = K.get(p, "number")[1]
            at = K.get(p, "at")
            pins[num] = (float(at[1]), -float(at[2]))
    return pins


def lib_prop_off(lib_id, prop):
    """库里属性（如 Value）相对原点的默认位置（图纸坐标，y 向下）。"""
    node = _symbol_node(lib_id)
    for p in K.get_all(node, "property"):
        if p[1] == prop:
            at = K.get(p, "at")
            return (float(at[1]), -float(at[2]))
    return (0.0, 0.0)


def lib_desc(lib_id):
    node = _symbol_node(lib_id)
    for p in K.get_all(node, "property"):
        if p[1] == "Description":
            return str(p[2])
    return ""


def rot_off(dx, dy, rot):
    """把 0° 偏移旋转 rot 度（屏幕上逆时针，图纸坐标 y 向下）。

    公式推导：屏幕坐标 (1,0)（向右）逆时针 90° 后应指向上方 (0,-1)，
    代入 (dx,dy)->(dy,-dx) 得 (0,-1)，故 90° 用 (dy,-dx)，其余同理。
    此约定由渲染结果目检确认（见 README 的"生成-渲染-目检"步骤）。"""
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


# ---------------------------------------------------------------- 图纸元素收集

WIRES = []       # (x1,y1,x2,y2,tag)
LABELS = []      # (name,x,y,justify)
NCS = []         # (x,y,tag)
JUNCTIONS = []   # (x,y,tag)
TEXTS = []       # (text,x,y)
SYMBOLS = []     # 实例 dict


def add_wire(x1, y1, x2, y2, tag):
    x1, y1, x2, y2 = g(x1), g(y1), g(x2), g(y2)
    assert (x1, y1) != (x2, y2), f"零长导线 {tag}"
    WIRES.append((x1, y1, x2, y2, tag))


def add_label(name, x, y, justify):
    LABELS.append((name, g(x), g(y), justify))


def add_nc(x, y, tag):
    NCS.append((g(x), g(y), tag))


def add_junction(x, y, tag):
    JUNCTIONS.append((g(x), g(y), tag))


def add_text(text, x, y):
    TEXTS.append((text, g(x), g(y)))


def place(ref, lib_id, value, at, rot=0, fp="", props=None, hide_ref=False,
          hide_value=False):
    """登记一个符号实例。props 可给 Reference/Value 位置微调 (dx,dy,justify)。"""
    inst = {
        "ref": ref, "lib": lib_id, "value": value, "at": (g(at[0]), g(at[1])),
        "rot": rot, "fp": fp, "props": props or {}, "hide_ref": hide_ref,
        "hide_value": hide_value, "pins": lib_pins(lib_id),
    }
    SYMBOLS.append(inst)
    return inst


def pin_pos(inst, num):
    dx, dy = inst["pins"][num]
    rx, ry = rot_off(dx, dy, inst["rot"])
    return (round(inst["at"][0] + rx, 4), round(inst["at"][1] + ry, 4))


def stub_label(inst, num, direction, net, length=2.54):
    """从引脚端点往 up/down/left/right 伸短线，末端挂网络标签。"""
    x, y = pin_pos(inst, num)
    if direction == "left":
        end, just = (x - length, y), "right"
    elif direction == "right":
        end, just = (x + length, y), "left"
    elif direction == "up":
        end, just = (x, y - length), "left"
    elif direction == "down":
        end, just = (x, y + length), "left"
    else:
        raise ValueError(direction)
    add_wire(x, y, end[0], end[1], f"{inst['ref']}-p{num}-stub")
    add_label(net, end[0], end[1], just)


def pin_label(inst, num, net, justify):
    """标签直接贴在引脚端点上（不伸短线）。密集区用它避免短线端点互撞。"""
    x, y = pin_pos(inst, num)
    add_label(net, x, y, justify)


def chain_hall_led(r_ref, led_ref, x, y, net, side="left"):
    """霍尔信号指示灯链：+3V3 — 330R — LED — 信号网。

    A3144 把信号拉低（磁铁靠近）时电流才通、灯亮，轮子转动时能看到闪烁，
    亮度也顺带反映传感器是不是被正确触发。330R 下电流约 4mA，在传感器
    20mA 灌电流能力之内；信号为高时 LED 两端等电位、自动熄灭，不吃电。
    side 决定位号文字放链的哪一边：右列用 'left'，左侧留空用 'right'。"""
    r_dx, r_just = (-2.54, "right") if side == "left" else (2.54, "left")
    led_dx = -5.08 if side == "left" else 5.08
    r = place(r_ref, "Device:R", "330", (x, y), fp=R_FP,
              props={"Reference": (r_dx, -1.27, r_just), "Value": (r_dx, 1.27, r_just)})
    pin_label(r, "1", "+3V3", "left")
    led = place(led_ref, "Device:LED", "红灯", (x, y + 10.16), rot=90,
                fp="LED_THT:LED_D5.0mm",
                props={"Reference": (led_dx, -1.27, None), "Value": (led_dx, 1.27, None)})
    add_wire(*pin_pos(r, "2"), *pin_pos(led, "2"), f"{r_ref}-led")
    stub_label(led, "1", "down", net)


# ---------------------------------------------------------------- 自定义符号：开发板两排针座

ROW_L_NAMES = ["3V3", "EN", "IO4", "IO5", "IO6", "IO7", "IO15", "IO16", "IO17", "IO18",
               "IO8", "IO3", "IO46", "IO9", "IO10", "IO11", "IO12", "IO13", "IO14", "5V"]
ROW_R_NAMES = ["IO43", "IO44", "IO1", "IO2", "IO42", "IO41", "IO40", "IO39", "IO38", "IO37",
               "IO36", "IO35", "IO0", "IO45", "IO48", "IO47", "IO21", "IO20", "IO19", "GND"]


def row_symbol_text(symname, pin_names, side, prefixed=True):
    """左右两排针座的自定义符号文本。side='L' 引脚朝左（线从左侧出），
    'R' 朝右。针号就是物理位置，是底板排母对号入座的唯一依据。
    prefixed=True 用于嵌进图纸（要求 '库:名' 形式），False 用于独立库文件。"""
    lib_id = f"wheel_carrier:{symname}" if prefixed else symname
    px = -10.16 if side == "L" else 10.16
    ang = 0 if side == "L" else 180
    pins = []
    for i, nm in enumerate(pin_names, start=1):
        y = 24.13 - 2.54 * (i - 1)
        pins.append(f"""      (pin passive line
        (at {f(px)} {f(y)} {ang})
        (length 5.08)
        (name "{nm}" (effects (font (size 1.27 1.27))))
        (number "{i}" (effects (font (size 1.27 1.27))))
      )""")
    return f"""    (symbol "{lib_id}"
      (pin_names (offset 1.016))
      (exclude_from_sim no)
      (in_bom yes)
      (on_board yes)
      (property "Reference" "J"
        (at 0 27.94 0)
        (effects (font (size 1.27 1.27)))
      )
      (property "Value" "{symname}"
        (at 0 -27.94 0)
        (effects (font (size 1.27 1.27)))
      )
      (property "Footprint" ""
        (at 0 0 0)
        (effects (font (size 1.27 1.27)) (hide yes))
      )
      (property "Datasheet" "~"
        (at 0 0 0)
        (effects (font (size 1.27 1.27)) (hide yes))
      )
      (property "Description" "Freenove ESP32-S3 WROOM CAM 开发板针座（针号=物理位置，自远离 USB 端数起）"
        (at 0 0 0)
        (effects (font (size 1.27 1.27)) (hide yes))
      )
      (symbol "{symname}_0_1"
        (rectangle
          (start -5.08 26.67)
          (end 5.08 -26.67)
          (stroke (width 0.254) (type default))
          (fill (type background))
        )
      )
      (symbol "{symname}_1_1"
{chr(10).join(pins)}
      )
      (embedded_fonts no)
    )"""


def custom_lib_node():
    """把写独立库文件用的同一份文本解析回来，供查引脚坐标/属性用——
    「放置」和「写 .kicad_sym」走同一数据源，两边不会漂移。"""
    text = ("(kicad_symbol_lib "
            + row_symbol_text("DevBoard_Row_L", ROW_L_NAMES, "L", prefixed=False)
            + row_symbol_text("DevBoard_Row_R", ROW_R_NAMES, "R", prefixed=False) + ")")
    return K.parse(K.tokenize(text))


# ---------------------------------------------------------------- 布局常量

# 开发板两排针座（J1 左排、J2 右排），原点取针列中点
J1_AT = (139.7, 119.38)
J2_AT = (280.67, 119.38)

# 两排针脚的完整去向：针号 -> 网络名。40 针全部有去向——没在用的脚接到
# 扩展排针 J13/J14（1:1），将来固件用新 GPIO 时杜邦线直接插扩展排针即可。
ROW_L_NETS = {
    "1": "+3V3", "2": "EN", "3": "IO4", "4": "IO5", "5": "IO6", "6": "IO7",
    "7": "IO15", "8": "IO16", "9": "IO17", "10": "IO18", "11": "IO8", "12": "IO3",
    "13": "IO46", "14": "IO9", "15": "IO10", "16": "IO11", "17": "IO12",
    "18": "IO13", "19": "GPIO14", "20": "DEV_5V",
}
ROW_R_NETS = {
    "1": "IO43", "2": "IO44", "3": "GPIO1", "4": "IO2", "5": "GPIO42", "6": "GPIO41",
    "7": "GPIO40", "8": "GPIO39", "9": "GPIO38", "10": "IO37", "11": "IO36",
    "12": "IO35", "13": "IO0", "14": "IO45", "15": "IO48", "16": "GPIO47",
    "17": "GPIO21", "18": "IO20", "19": "IO19", "20": "GND",
}

# 右侧接口列：3 针接口从上到下
CONN_COL_X = 337.82
CONN_ROWS = [
    ("J3", "霍尔·左前", "GPIO1", 96.52),
    ("J5", "霍尔·左后", "GPIO21", 109.22),
    ("J6", "霍尔·右后", "GPIO47", 121.92),
    ("J7", "舵机·前差速", "GPIO41", 134.62),
    ("J8", "舵机·后差速", "GPIO42", 147.32),
    ("J9", "遥控·模式", "GPIO38", 160.02),
    ("J10", "遥控·前差速", "GPIO39", 172.72),
    ("J11", "遥控·后差速", "GPIO40", 185.42),
]
# 上拉电阻列（与各自接口同高）
PULLUP_X = 354.33
PULLUPS = [("R1", "GPIO1", 96.52), ("R3", "GPIO21", 109.22), ("R4", "GPIO47", 121.92)]
# 左侧：霍尔·右前 + 它的上拉
J4_AT = (59.69, 134.62)
R2_AT = (76.2, 128.27)


def build():
    # ---- 两排针座 -----------------------------------------------------------
    j1 = place("J1", "wheel_carrier:DevBoard_Row_L", "开发板左排 20P", J1_AT,
               fp="Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical",
               props={"Reference": (-20.32, -27.94, "left"), "Value": (0, -29.21, None)})
    j2 = place("J2", "wheel_carrier:DevBoard_Row_R", "开发板右排 20P", J2_AT,
               fp="Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical",
               props={"Reference": (20.32, -27.94, "right"), "Value": (0, -29.21, None)})
    for inst, nets, direction in ((j1, ROW_L_NETS, "left"), (j2, ROW_R_NETS, "right")):
        for num in sorted(inst["pins"], key=int):
            stub_label(inst, num, direction, nets[num])

    # ---- 扩展排针：开发板 40 针 1:1 引出 --------------------------------------
    # 没在用的脚全部引到这里：将来固件用新脚（I2C 传感器、IO 复用等）插线即可，
    # 不用重新打板。丝印上每针标 GPI0 号、特殊脚打警示，在布局阶段做。
    j13 = place("J13", "Connector_Generic:Conn_01x20", "扩展·左排(直连 J1)",
                (200.66, 133.35),
                fp="Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
                props={"Reference": (0, -29.21, None), "Value": (0, 29.21, None)})
    for num in sorted(j13["pins"], key=int):
        stub_label(j13, num, "left", ROW_L_NETS[num])
    j14 = place("J14", "Connector_Generic:Conn_01x20", "扩展·右排(直连 J2)",
                (241.3, 133.35),
                fp="Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
                props={"Reference": (0, -29.21, None), "Value": (0, 29.21, None)})
    for num in sorted(j14["pins"], key=int):
        stub_label(j14, num, "left", ROW_R_NETS[num])
    # 电源排针：以后接 IMU/传感器模块的取电口，省得从别处飞线
    j15 = place("J15", "Connector_Generic:Conn_01x04", "电源排针", (200.66, 172.72),
                fp="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
                props={"Reference": (2.54, -2.54, "left"), "Value": (2.54, 0, "left")})
    for num, net in (("1", "+BEC"), ("2", "+5V"), ("3", "+3V3"), ("4", "GND")):
        stub_label(j15, num, "left", net)

    # ---- 右侧 3 针接口列 + 上拉 ---------------------------------------------
    for ref, value, sig, y in CONN_ROWS:
        inst = place(ref, "Connector_Generic:Conn_01x03", value, (CONN_COL_X, y),
                     fp="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
                     # 编号挪到器件右侧：左侧右对齐的"J3"正好跟"GPIO1"标签贴死
                     props={"Reference": (2.54, -3.81, "left"), "Value": (-13.97, 3.81, "right")})
        stub_label(inst, "1", "left", sig)
        stub_label(inst, "2", "left", "+BEC")
        stub_label(inst, "3", "left", "GND")
    for ref, sig, y in PULLUPS:
        r = place(ref, "Device:R", "10k", (PULLUP_X, y), fp=R_FP,
                  props={"Reference": (-2.54, -1.27, "right"), "Value": (-2.54, 1.27, "right")})
        # 顶端直接贴 +3V3 标签、不伸短线：相邻上拉若上下各留 2.54 短线，
        # 端点会落在同一坐标上把两条网络短接（间距恰好 12.7 = 两倍整个器件跨度）
        # +3V3 标签往左伸：右伸会顶到邻列霍尔指示灯链的"LEDx"文字
        pin_label(r, "1", "+3V3", "right")
        stub_label(r, "2", "down", sig)
    # 霍尔指示灯（右侧三路）：错列摆放——链条高 20.32 超过 12.7 的行距，
    # 同一列上会前后压住，隔一列一错开刚好错得开
    chain_hall_led("R7", "LED3", 365.76, 96.52, "GPIO1", side="left")
    chain_hall_led("R8", "LED4", 377.19, 109.22, "GPIO21", side="left")
    chain_hall_led("R9", "LED5", 365.76, 121.92, "GPIO47", side="left")

    # ---- 左侧：霍尔·右前 -----------------------------------------------------
    j4 = place("J4", "Connector_Generic:Conn_01x03", "霍尔·右前", J4_AT,
               fp="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
               props={"Reference": (2.54, -3.81, "left"), "Value": (-13.97, 3.81, "right")})
    stub_label(j4, "1", "left", "GPIO14")
    stub_label(j4, "2", "left", "+BEC")
    stub_label(j4, "3", "left", "GND")
    r2 = place("R2", "Device:R", "10k", R2_AT, fp=R_FP,
               props={"Reference": (2.54, -1.27, "left"), "Value": (2.54, 1.27, "left")})
    pin_label(r2, "1", "+3V3", "left")
    stub_label(r2, "2", "down", "GPIO14")
    # 霍尔指示灯（右前）
    chain_hall_led("R6", "LED2", 88.9, 128.27, "GPIO14", side="right")

    # ---- 电源区（底部） -------------------------------------------------------
    # BEC 输入端子：1=VIN 2=GND（电调 BEC 输出，6V 或 7.4V 档）
    j12 = place("J12", "Connector:Screw_Terminal_01x02", "BEC 输入 6~7.4V", (152.4, 199.39),
                fp="TerminalBlock:TerminalBlock_MaiXu_MX126-5.0-02P_1x02_P5.00mm",
                props={"Reference": (2.54, -3.81, "left"), "Value": (2.54, 0, "left")})
    p1 = pin_pos(j12, "1")
    p2 = pin_pos(j12, "2")
    # GND 端：短线到网络标签；PWR_FLAG 让 ERC 认可地网有驱动源
    add_wire(*p2, 144.78, p2[1], "J12-gnd-stub")
    add_label("GND", 144.78, p2[1], "right")
    place("#FLG01", "power:PWR_FLAG", "PWR_FLAG", (144.78, p2[1]), hide_ref=True,
          hide_value=True)   # 旗标文字亮着会压到 D1 的标注，藏掉只留旗形

    # 防反接二极管 D1（K 朝电路，A 朝端子）：端子 -> A，K -> BEC 主路。
    # 用 5A 的 1N5824：两路舵机堵转的总电流接近 3A，比 1N5822 的额定值留余地
    d1 = place("D1", "Device:D_Schottky", "1N5824", (138.43, 196.85), fp=D_FP,
               # 编号/型号叠放在管子上方：下方一线挤着 C1 编号和 J12 的 GND 标签
               props={"Reference": (0, -4.57, None), "Value": (0, -6.86, None)})
    add_wire(*p1, *pin_pos(d1, "2"), "J12-D1A")
    rail_y = 196.85
    rail_end = 111.76
    k = pin_pos(d1, "1")
    add_wire(*k, rail_end, rail_y, "bec-rail")
    add_label("+BEC", rail_end, rail_y, "right")
    # BEC 主路向下喂降压段；这条母线有 U1 的 power_in 引脚，配 PWR_FLAG
    add_junction(114.3, rail_y, "tap-buck")
    place("#FLG02", "power:PWR_FLAG", "PWR_FLAG", (114.3, rail_y), hide_ref=True,
          hide_value=True)
    add_wire(114.3, rail_y, 114.3, 228.6, "bec-feed-down")
    add_wire(114.3, 228.6, 121.92, 228.6, "bec-feed-vin")

    # 大电容 C1
    c1 = place("C1", "Device:C_Polarized", "1000µF/16V", (130.81, 203.2), fp=CP_FP,
               props={"Reference": (2.54, -1.27, "left"), "Value": (2.54, 1.27, "left")})
    add_junction(130.81, rail_y, "tap-C1")
    add_wire(130.81, rail_y, *pin_pos(c1, "1"), "rail-C1")
    add_wire(*pin_pos(c1, "2"), 130.81, 209.55, "C1-gnd")
    add_label("GND", 130.81, 209.55, "left")

    # 电源指示灯：+BEC -> R5 -> LED1 -> GND
    r5 = place("R5", "Device:R", "1k", (124.46, 204.47), fp=R_FP,
               props={"Reference": (2.54, -1.27, "left"), "Value": (2.54, 1.27, "left")})
    add_junction(124.46, rail_y, "tap-led")
    add_wire(124.46, rail_y, *pin_pos(r5, "1"), "rail-R5")
    led = place("LED1", "Device:LED", "绿灯", (124.46, 214.63), rot=90, fp="LED_THT:LED_D5.0mm",
                props={"Reference": (7.62, -1.27, None), "Value": (7.62, 1.27, None)})
    add_wire(*pin_pos(r5, "2"), *pin_pos(led, "2"), "R5-LED")
    add_wire(*pin_pos(led, "1"), 124.46, 220.98, "LED-gnd")
    add_label("GND", 124.46, 220.98, "left")

    # ---- 降压段（行 B）：BEC 6~7.4V -> LM2596T-5.0 -> 板上 5V -----------------
    # 开发板标称 5V，BEC 的 6/7.4V 直接灌会烧它，必须真降压。固定 5V 版标准接法：
    # VIN 吃 +BEC 并挂输入电容 C3；OUT->电感->输出；FB 直接接回输出；ON/OFF 接地常开
    u1 = place("U1", "Regulator_Switching:LM2596T-5", "LM2596T-5", (134.62, 231.14),
               fp="Package_TO_SOT_THT:TO-220-5_Vertical",
               props={"Reference": (0, -8.89, None), "Value": (0, -6.35, None)})
    add_junction(114.3, 228.6, "c3-tap")
    c3 = place("C3", "Device:C_Polarized", "220µF/25V", (114.3, 232.41), fp=CP_FP,
               props={"Reference": (-2.54, -1.27, "right"), "Value": (-2.54, 1.27, "right")})
    add_wire(114.3, 236.22, 114.3, 238.76, "C3-gnd")
    add_label("GND", 114.3, 238.76, "right")
    add_wire(121.92, 233.68, 121.92, 236.22, "U1-onoff-gnd")
    add_label("GND", 121.92, 236.22, "right")
    add_wire(134.62, 238.76, 134.62, 241.3, "U1-gnd")
    add_label("GND", 134.62, 241.3, "right")
    add_wire(147.32, 228.6, 149.86, 228.6, "U1-fb")
    add_label("+5V", 149.86, 228.6, "left")
    add_wire(147.32, 233.68, 147.32, 236.22, "U1-out-down")
    add_wire(147.32, 236.22, 154.94, 236.22, "sw-a")
    add_wire(154.94, 236.22, 161.29, 236.22, "sw-b")
    add_junction(154.94, 236.22, "d3-tap")
    d3 = place("D3", "Device:D_Schottky", "1N5822", (154.94, 240.03), rot=270, fp=D_FP,
               # 续流二极管：开关管关断时给电感电流兜出路，K 朝开关节点
               props={"Reference": (0, -2.54, None), "Value": (-8.89, 0, None)})
    add_label("GND", 154.94, 243.84, "right")
    l1 = place("L1", "Device:L", "47µH", (161.29, 240.03),
               fp="Inductor_THT:L_Radial_D10.0mm_P5.00mm_Fastron_07P",
               props={"Reference": (0, -6.35, None), "Value": (0, -8.89, None)})
    add_wire(161.29, 243.84, 161.29, 246.38, "C4-top")
    c4 = place("C4", "Device:C_Polarized", "220µF/16V", (161.29, 250.19), fp=CP_FP,
               props={"Reference": (-2.54, -1.27, "right"), "Value": (-2.54, 1.27, "right")})
    add_wire(161.29, 254, 161.29, 256.54, "C4-gnd")
    add_label("GND", 161.29, 256.54, "right")
    add_wire(161.29, 243.84, 170.18, 243.84, "+5-a")
    add_wire(170.18, 243.84, 175.26, 243.84, "+5-b")
    add_wire(175.26, 243.84, 180.34, 243.84, "+5-c")
    add_junction(175.26, 243.84, "led6-tap")
    add_label("+5V", 180.34, 243.84, "left")
    # 开发板供电支路：+5V -> D2 -> DEV_5V（防 USB 与底板互相倒灌）
    d2 = place("D2", "Device:D_Schottky", "1N5822", (170.18, 247.65), rot=90, fp=D_FP,
               # 旋转 90° 的器件：KiCad 会把 180° 的文字翻正并反转左右对齐，
               # 对齐语义不可靠，用「居中」定位放在器件上方空位
               props={"Reference": (0, -6.35, None), "Value": (0, -8.89, None)})
    add_wire(170.18, 251.46, 170.18, 252.73, "D2K-C2")
    add_label("DEV_5V", 170.18, 252.73, "right")
    c2 = place("C2", "Device:C", "100nF", (170.18, 256.54),
               fp="Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm",
               props={"Reference": (-2.54, -1.27, "right"), "Value": (-2.54, 1.27, "right")})
    add_wire(170.18, 260.35, 170.18, 262.89, "C2-gnd")
    add_label("GND", 170.18, 262.89, "right")
    # 5V 指示灯：灯亮 = 降压段有输出（打样板试电时不用万用表）
    r10 = place("R10", "Device:R", "1k", (175.26, 247.65), fp=R_FP,
                props={"Reference": (2.54, -1.27, "left"), "Value": (2.54, 1.27, "left")})
    led6 = place("LED6", "Device:LED", "绿灯", (175.26, 257.81), rot=90,
                 fp="LED_THT:LED_D5.0mm",
                 props={"Reference": (6.35, -1.27, None), "Value": (6.35, 1.27, None)})
    add_wire(175.26, 251.46, 175.26, 254, "R10-LED6")
    add_wire(175.26, 261.62, 175.26, 264.16, "LED6-gnd")
    add_label("GND", 175.26, 264.16, "right")

    # 电源网络（+BEC/+5V/GND/+3V3）全图统一用网络标签表达，不摆电源符号：
    # 同名电源符号与标签并存会被 ERC 报 same_local_global_label（本就是同一张网）。
    # PWR_FLAG 是例外——它不改网络名，只负责让 ERC 认可母线有驱动源。

    # ---- 图面文字说明 ---------------------------------------------------------
    add_text("针号 1 从开发板远离 USB 口的一端数起；装板前对开发板丝印核一遍", 127.0, 88.9)
    add_text("扩展排针 J13/J14 直连开发板全部 40 针；J15 是电源排针", 170.18, 99.06)
    add_text("35/36/37=PSRAM，19/20=USB，0/3/45/46=启动脚：接外部电路前先看 README", 170.18, 101.6)
    add_text("BEC 输入 6~7.4V：板上降到 5V 给开发板；舵机/接收机/霍尔走 BEC 原电压", 105.41, 270.51)
    add_text("接口顺序统一：信号 / BEC / GND，与舵机、接收机插头一致", 320.04, 88.9)
    add_text("霍尔指示灯：亮 = 磁铁触发（信号被拉低）；轮子转动能看到闪烁", 354.33, 83.82)


R_FP = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal"
D_FP = "Diode_THT:D_DO-201AD_P15.24mm_Horizontal"
CP_FP = "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm"


# ---------------------------------------------------------------- 文本输出

def emit_property(name, value, x, y, rot=0, hide=False, justify=None, size=1.27):
    eff = f"(effects (font (size {f(size)} {f(size)}))"
    if justify:
        eff += f" (justify {justify})"
    if hide:
        eff += " (hide yes)"
    eff += ")"
    return (f'    (property "{name}" "{value}"\n'
            f"      (at {f(x)} {f(y)} {rot})\n"
            f"      {eff}\n"
            f"    )")


def emit_symbol(inst):
    x, y = inst["at"]
    lines = [
        "  (symbol",
        f'    (lib_id "{inst["lib"]}")',
        f"    (at {f(x)} {f(y)} {inst['rot']})",
        "    (unit 1)",
        "    (exclude_from_sim no)",
        "    (in_bom yes)",
        "    (on_board yes)",
        "    (dnp no)",
        f'    (uuid "{uid(inst["ref"])}")',
    ]
    overrides = inst["props"]
    # Reference
    if "Reference" in overrides:
        dx, dy, just = overrides["Reference"]
        rpos = (x + dx, y + dy)
    else:
        pdx, pdy = lib_prop_off(inst["lib"], "Reference")
        rx, ry = rot_off(pdx, pdy, inst["rot"])
        rpos = (x + rx, y + ry)
    # 属性文字角度是相对符号的：器件旋转 90°/270° 时属性要写 90 才会横排，
    # 否则 KiCad 会把文字跟着竖过来压在器件上（D2/LED1 实测踩过）
    pa = 90 if inst["rot"] % 180 else 0
    lines.append(emit_property("Reference", inst["ref"], rpos[0], rpos[1], rot=pa,
                               hide=inst["hide_ref"], justify=overrides.get("Reference", (0, 0, None))[2]))
    # Value
    if "Value" in overrides:
        dx, dy, just = overrides["Value"]
        vpos = (x + dx, y + dy)
    else:
        pdx, pdy = lib_prop_off(inst["lib"], "Value")
        vx, vy = rot_off(pdx, pdy, inst["rot"])
        vpos = (x + vx, y + vy)
    lines.append(emit_property("Value", inst["value"], vpos[0], vpos[1], rot=pa,
                               hide=inst["hide_value"],
                               justify=overrides.get("Value", (0, 0, None))[2]))
    lines.append(emit_property("Footprint", inst["fp"], x, y, rot=pa, hide=True))
    ds = "" if inst["lib"].startswith("power:") else "~"
    lines.append(emit_property("Datasheet", ds, x, y, rot=pa, hide=True))
    lines.append(emit_property("Description", lib_desc(inst["lib"]), x, y, rot=pa, hide=True))
    for num in sorted(inst["pins"], key=int):
        lines.append(f'    (pin "{num}"\n      (uuid "{uid(inst["ref"] + "-pin" + num)}")\n    )')
    lines += [
        "    (instances",
        f'      (project "{PROJECT}"',
        f'        (path "/{ROOT_UUID}"',
        f'          (reference "{inst["ref"]}")',
        "          (unit 1)",
        "        )",
        "      )",
        "    )",
        "  )",
    ]
    return "\n".join(lines)


def emit_all():
    out = []
    out.append("(kicad_sch")
    out.append("\t(version 20250114)")
    out.append('\t(generator "eeschema")')
    out.append('\t(generator_version "9.0")')
    out.append(f'\t(uuid "{ROOT_UUID}")')
    out.append(f'\t(paper "{PAPER}")')
    out.append("\t(title_block")
    out.append(f'\t\t(title "{TITLE}")')
    out.append(f'\t\t(date "{DATE}")')
    out.append(f'\t\t(rev "{REV}")')
    out.append('\t\t(comment 1 "配 Freenove ESP32-S3 WROOM CAM 开发板（FNK0085）")')
    out.append('\t\t(comment 2 "接口与引脚分配和固件 wheel_config.h 一致")')
    out.append("\t)")

    out.append("\t(lib_symbols")
    lib_ids = [
        "Connector:Screw_Terminal_01x02",
        "Connector_Generic:Conn_01x03",
        "Connector_Generic:Conn_01x04",
        "Connector_Generic:Conn_01x20",
        "Device:C",
        "Device:C_Polarized",
        "Device:D_Schottky",
        "Device:L",
        "Device:LED",
        "Device:R",
        "Regulator_Switching:LM2596T-5",
        "power:PWR_FLAG",
    ]
    for lib_id in lib_ids:
        out.append(lib_symbol_text(lib_id))
    out.append(row_symbol_text("DevBoard_Row_L", ROW_L_NAMES, "L"))
    out.append(row_symbol_text("DevBoard_Row_R", ROW_R_NAMES, "R"))
    out.append("\t)")

    for x, y, tag in JUNCTIONS:
        out.append(f'  (junction\n    (at {f(x)} {f(y)})\n    (diameter 0)\n'
                   f"    (color 0 0 0 0)\n    (uuid \"{uid('junc-' + tag)}\")\n  )")
    for x, y, tag in NCS:
        out.append(f'  (no_connect\n    (at {f(x)} {f(y)})\n    (uuid "{uid("nc-" + tag)}")\n  )')
    for x1, y1, x2, y2, tag in WIRES:
        out.append(f"  (wire\n    (pts\n      (xy {f(x1)} {f(y1)}) (xy {f(x2)} {f(y2)})\n    )\n"
                   f"    (stroke (width 0) (type default))\n    (uuid \"{uid('wire-' + tag)}\")\n  )")
    for name, x, y, justify in LABELS:
        out.append(f'  (label "{name}"\n    (at {f(x)} {f(y)} 0)\n'
                   f"    (effects (font (size 1.27 1.27)) (justify {justify} bottom))\n"
                   f"    (uuid \"{uid(f'label-{name}-{f(x)}-{f(y)}')}\")\n  )")
    for text, x, y in TEXTS:
        out.append(f'  (text "{text}"\n    (exclude_from_sim no)\n    (at {f(x)} {f(y)} 0)\n'
                   f"    (effects (font (size 1.27 1.27)) (justify left bottom))\n"
                   f"    (uuid \"{uid(f'text-{f(x)}-{f(y)}')}\")\n  )")
    for inst in sorted(SYMBOLS, key=lambda s: (s["ref"][0] != "#", s["ref"])):
        out.append(emit_symbol(inst))

    out.append("\t(sheet_instances\n\t\t(path \"/\"\n\t\t\t(page \"1\")\n\t\t)\n\t)")
    out.append("\t(embedded_fonts no)")
    out.append(")")
    return "\n".join(out) + "\n"


def write_custom_lib_files(out_path):
    """写自定义符号库 .kicad_sym 与工程符号表 sym-lib-table：
    让 KiCad 能解析 'wheel_carrier' 这个符号库（否则 ERC 报"库未配置"）。
    库内容与图纸里嵌的 lib_symbols 同源（同一 row_symbol_text 生成）。"""
    out_dir = os.path.dirname(out_path)
    lib_path = os.path.join(out_dir, "wheel_carrier.kicad_sym")
    with open(lib_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("(kicad_symbol_lib\n"
                 "\t(version 20250114)\n"
                 '\t(generator "gen_schematic.py")\n'
                 + row_symbol_text("DevBoard_Row_L", ROW_L_NAMES, "L", prefixed=False) + "\n"
                 + row_symbol_text("DevBoard_Row_R", ROW_R_NAMES, "R", prefixed=False) + "\n"
                 ")\n")
    with open(os.path.join(out_dir, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write('(sym_lib_table\n'
                 '\t(version 7)\n'
                 '\t(lib (name "wheel_carrier")(type "KiCad")'
                 '(uri "${KIPRJMOD}/wheel_carrier.kicad_sym")(options "")'
                 '(descr "本工程自定义符号：开发板两排针座"))\n'
                 ')\n')
    # 最小工程文件：让 KiCad/ERC 有"工程"语境（${KIPRJMOD} 才能解析出符号库路径），
    # 双击 .kicad_pro 打开即是完整工程；缺失的配置段 KiCad 会按默认值补齐
    pro_path = os.path.join(out_dir, "wheel_carrier.kicad_pro")
    with open(pro_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write('{\n'
                 '  "board": {"design_settings": {"rule_severities": '
                 '{"lib_footprint_mismatch": "ignore"}}},\n'
                 '  "boards": [],\n'
                 '  "cvpcb": {},\n'
                 '  "erc": {},\n'
                 '  "libraries": {},\n'
                 '  "meta": {"filename": "wheel_carrier.kicad_pro", "version": 1},\n'
                 '  "net_settings": {},\n'
                 '  "pcbnew": {},\n'
                 '  "schematic": {},\n'
                 '  "sheets": [],\n'
                 '  "text_variables": {}\n'
                 '}\n')
    return lib_path


def main():
    out_path = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUT
    build()
    text = emit_all()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    lib_path = write_custom_lib_files(out_path)
    parts = [s for s in SYMBOLS if not s["ref"].startswith("#")]
    print(f"已生成 {out_path}")
    print(f"       + {lib_path}（自定义符号库）")
    print(f"元件 {len(parts)} 个 / 导线 {len(WIRES)} 根 / 标签 {len(LABELS)} 个 / "
          f"空脚标记 {len(NCS)} 个 / 结点 {len(JUNCTIONS)} 个")


if __name__ == "__main__":
    main()
