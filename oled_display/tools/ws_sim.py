#!/usr/bin/env python3
"""WS 遥测模拟脚本：按车端遥测帧格式喂数据，验证 OLED 显示链路。

电脑开热点模拟车端网络，在电脑上跑本脚本当 WS 服务端；C3 配好热点凭据
（或在配置页面把服务端地址填成本机），连上来收的就是和真车端一模一样的
帧。只用标准库，不用装依赖。

用法：
    python3 ws_sim.py [--port 8080] [--path /ws] [--interval-ms 100]

运行中可用的命令（stdin 敲）：
    rpm <左前> <右前> <左后> <右后>   四轮 RPM
    en <轮号 0-3> <0|1>               启用/禁用某轮（模拟坏轮剔除）
    mode auto|manual                  生效模式
    phase front|rear idle|pending_sync|probe|locked
    pause / resume                    停发/恢复（掐掉数据模拟断线）
    quit
"""

import argparse
import base64
import hashlib
import json
import socket
import struct
import sys
import threading
import time

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

PHASES = ("idle", "pending_sync", "probe", "locked")


class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.rpm = [60.0, 60.0, 60.0, 60.0]
        self.en = [True, True, True, True]
        self.mode = "auto"
        self.phase = ["idle", "idle"]  # 前/后差速
        self.paused = False

    def frame(self):
        """组一帧，结构与车端 telem_render_frame() 的输出一致"""
        wheels = [
            {
                "i": i,
                "en": self.en[i],
                "rpm": round(self.rpm[i], 1),
                "rps": round(self.rpm[i] / 60.0, 2),
                "speed_cms": 0.0,
                "freq": round(self.rpm[i] / 60.0, 3),
                "pulses": 0,
                "suspect": 0,
                "trigger": False,
            }
            for i in range(4)
        ]
        servo_ch = [
            {
                "i": i,
                "en": True,
                "phase": self.phase[i],
                "hold_ms": 0,
                "ratio": 0.0,
                "slip_left": False,
                "target_us": 1500,
                "cur_us": 1500.0,
                "reached": True,
            }
            for i in range(2)
        ]
        rc_ch = [{"us": 1500, "valid": True, "seen": True, "age_ms": 0} for _ in range(3)]
        frame = {
            "type": "telemetry",
            "seq": self.seq,
            "t": int(time.time() * 1000) & 0xFFFFFFFF,
            "sys": {
                "gpio": [1, 1, 1, 1],
                "free_heap": 0,
                "heap_int": 0,
                "uptime_s": int(time.time()) % 100000,
                "cam": {
                    "policy": "on_demand",
                    "state": "off",
                    "preview": "inactive",
                    "streaming": False,
                    "clients": 0,
                    "stop_in_s": 0,
                    "res": "SVGA",
                    "quality": 12,
                    "quality_eff": 12,
                    "fps_cap": 15,
                    "fps_user_cap": 15,
                    "fps_eff": 15,
                    "overlay": 0,
                    "fps": 0.0,
                    "bytes": 0,
                },
            },
            "collectors": [
                {
                    "type": "wheel_speed",
                    "src": "sim",
                    "wheels": wheels,
                    "cfg": {
                        "magnets": 1,
                        "diam_mm": 66,
                        "alpha": 0.30,
                        "debounce_ms": 0,
                        "sim": True,
                        "sim_rpm": self.rpm,
                        "gear_ratio": [1.0, 1.0, 1.0, 1.0],
                    },
                },
                {
                    "type": "servo",
                    "src": "sim",
                    "mode": self.mode,
                    "mode_src": "cfg",
                    "sim": True,
                    "servos": servo_ch,
                    "cfg": {
                        "sim": True,
                        "unlock_us": [1100, 1100],
                        "lock_us": [1900, 1900],
                        "manual_us": [1500, 1500],
                        "mode": self.mode,
                        "engage": 0.30,
                        "min_rpm": 20.0,
                        "confirm_ms": 300,
                        "hold_ms": 3000,
                        "hold_max_ms": 30000,
                        "probe_ms": 2000,
                        "gate_speed": 5.0,
                        "gate_delta": 2.0,
                        "gate_stable_ms": 500,
                        "manual_gate": False,
                        "sim_speed": 0,
                    },
                },
                {
                    "type": "rc",
                    "src": "sim",
                    "mode": self.mode,
                    "mode_src": "cfg",
                    "ch": rc_ch,
                    "target": [{"us": 1500, "from_rc": False} for _ in range(2)],
                    "cfg": {"sim": True, "sim_us": [1500, 1500, 1500]},
                },
            ],
        }
        self.seq += 1
        return json.dumps(frame, separators=(",", ":"))


def ws_accept(conn):
    """握手：HTTP Upgrade → WebSocket"""
    req = b""
    while b"\r\n\r\n" not in req:
        chunk = conn.recv(1024)
        if not chunk:
            return False
        req += chunk
    key = None
    for line in req.decode("latin-1").split("\r\n"):
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
    if key is None:
        return False
    accept = base64.b64encode(
        hashlib.sha1((key + WS_GUID).encode()).digest()
    ).decode()
    resp = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
    )
    conn.sendall(resp.encode())
    return True


def ws_send_text(conn, text):
    """服务端发帧不掩码"""
    data = text.encode()
    hdr = bytes([0x81])
    n = len(data)
    if n < 126:
        hdr += bytes([n])
    elif n < 65536:
        hdr += bytes([126]) + struct.pack(">H", n)
    else:
        hdr += bytes([127]) + struct.pack(">Q", n)
    conn.sendall(hdr + data)


def ws_recv(conn):
    """收客户端帧（带掩码），处理 ping/close，返回文本或 None"""
    hdr = conn.recv(2)
    if len(hdr) < 2:
        return None
    opcode = hdr[0] & 0x0F
    n = hdr[1] & 0x7F
    if n == 126:
        n = struct.unpack(">H", conn.recv(2))[0]
    elif n == 127:
        n = struct.unpack(">Q", conn.recv(8))[0]
    mask = conn.recv(4) if hdr[1] & 0x80 else None
    payload = b""
    while len(payload) < n:
        chunk = conn.recv(n - len(payload))
        if not chunk:
            return None
        payload += chunk
    if mask:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    if opcode == 0x8:
        return None
    if opcode == 0x9:
        conn.sendall(bytes([0x8A, len(payload)]) + payload)
        return ""
    return payload.decode("utf-8", "replace")


class Server:
    def __init__(self, sim, port, path):
        self.sim = sim
        self.path = path
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.stop = False

    def accept_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.port))
        srv.listen(4)
        srv.settimeout(0.5)
        print(f"WS 服务端已启动 ws://0.0.0.0:{self.port}{self.path}（等 C3 连上来）")
        while not self.stop:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            conn.settimeout(0.5)
            try:
                if not ws_accept(conn):
                    conn.close()
                    continue
            except OSError:
                conn.close()
                continue
            print(f"客户端已连接 {addr[0]}")
            with self.lock:
                self.clients.append(conn)
            threading.Thread(target=self.recv_loop, args=(conn,), daemon=True).start()
        srv.close()

    def recv_loop(self, conn):
        """丢掉客户端发来的帧，只在意连接死活"""
        while not self.stop:
            try:
                if ws_recv(conn) is None:
                    break
            except (OSError, struct.error):
                break
        with self.lock:
            if conn in self.clients:
                self.clients.remove(conn)
        print("客户端已断开")

    def broadcast_loop(self, interval):
        while not self.stop:
            time.sleep(interval)
            with self.sim.lock:
                if self.sim.paused:
                    continue
                frame = self.sim.frame()
            dead = []
            with self.lock:
                targets = list(self.clients)
            for conn in targets:
                try:
                    ws_send_text(conn, frame)
                except OSError:
                    dead.append(conn)
            if dead:
                with self.lock:
                    for conn in dead:
                        if conn in self.clients:
                            self.clients.remove(conn)


def parse_args(argv):
    p = argparse.ArgumentParser(description="OLED 显示链路验证用 WS 模拟车端")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--path", default="/ws")
    p.add_argument("--interval-ms", type=int, default=100,
                   help="推送周期，跟车端 100ms 对齐")
    return p.parse_args(argv)


def cmd_loop(sim, server):
    print("命令：rpm <四轮> / en <轮号> <0|1> / mode auto|manual / "
          "phase front|rear <阶段> / pause / resume / quit")
    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue
        with sim.lock:
            try:
                if parts[0] == "rpm" and len(parts) == 5:
                    sim.rpm = [float(x) for x in parts[1:]]
                    print(f"rpm -> {sim.rpm}")
                elif parts[0] == "en" and len(parts) == 3:
                    sim.en[int(parts[1])] = parts[2] == "1"
                    print(f"en -> {sim.en}")
                elif parts[0] == "mode" and parts[1] in ("auto", "manual"):
                    sim.mode = parts[1]
                    print(f"mode -> {sim.mode}")
                elif (parts[0] == "phase" and len(parts) == 3
                      and parts[1] in ("front", "rear")
                      and parts[2] in PHASES):
                    sim.phase[0 if parts[1] == "front" else 1] = parts[2]
                    print(f"phase -> {sim.phase}")
                elif parts[0] == "pause":
                    sim.paused = True
                    print("已停发（模拟断线）")
                elif parts[0] == "resume":
                    sim.paused = False
                    print("已恢复")
                elif parts[0] == "quit":
                    break
                else:
                    print("看不懂这条命令")
            except (ValueError, IndexError):
                print("命令参数不对")
    server.stop = True


def main():
    args = parse_args(sys.argv[1:])
    sim = Sim()
    server = Server(sim, args.port, args.path)
    threading.Thread(target=server.accept_loop, daemon=True).start()
    threading.Thread(
        target=server.broadcast_loop,
        args=(args.interval_ms / 1000.0,),
        daemon=True,
    ).start()
    cmd_loop(sim, server)


if __name__ == "__main__":
    main()
