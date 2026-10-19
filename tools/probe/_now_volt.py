# -*- coding: utf-8 -*-
"""只读: 连续读 20 秒电池电压/电流, 并打印 STATUSTEXT"""
import sys, time
from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.138:5760"
sec = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
CELLS = 8

m = mavutil.mavlink_connection("tcp:" + addr, source_system=255)
if not m.wait_heartbeat(timeout=10):
    print("无 HEARTBEAT")
    sys.exit(1)
print("已连接 %s，采样 %.0f 秒\n" % (addr, sec))

sv = []        # SYS_STATUS 电压
si = []        # SYS_STATUS 电流
sr = []        # SYS_STATUS 剩余
bv = []        # BATTERY_STATUS 电压
bi = []        # BATTERY_STATUS 电流
cell_last = None
t0 = time.time()

while time.time() - t0 < sec:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    ty = msg.get_type()
    if ty == "SYS_STATUS":
        v = getattr(msg, "voltage_battery", 0)
        if v and v != 65535:
            sv.append(v / 1000.0)
            si.append(getattr(msg, "current_battery", 0) / 100.0)
            sr.append(getattr(msg, "battery_remaining", -1))
    elif ty == "BATTERY_STATUS":
        cells = getattr(msg, "voltages", None)
        if cells:
            valid = [c / 1000.0 for c in cells[:10] if c and c != 65535]
            if valid:
                bv.append(valid[0])   # ArduPilot 把整包电压放在 voltages[0]
                cell_last = valid
        bi.append(getattr(msg, "current_battery", 0) / 100.0)
    elif ty == "STATUSTEXT":
        txt = msg.text
        if isinstance(txt, bytes):
            txt = txt.decode(errors="replace")
        txt = txt.replace("\x00", "").strip()
        if txt:
            print("  [%.1fs] TEXT: %s" % (time.time() - t0, txt))


def show(tag, arr, unit="V", cells=None):
    if not arr:
        print("  %-30s : 无数据" % tag)
        return
    s = sorted(arr)
    print("  %-30s : n=%-4d  最小=%.2f  中位=%.2f  最大=%.2f  末值=%.2f %s"
          % (tag, len(arr), s[0], s[len(s) // 2], s[-1], arr[-1], unit))
    if cells:
        print("  %-30s   (末值 %.2fV / %dS = %.2f V per cell)  安全下限 2.5V/cell = %.1fV"
              % ("", arr[-1], CELLS, arr[-1] / CELLS, 2.5 * CELLS))


print("=" * 68)
show("SYS_STATUS.voltage_battery", sv)
show("BATTERY_STATUS.voltages[0]", bv)
if si:
    print("  %-30s : 最小=%.1f  中位=%.1f  最大=%.1f A"
          % ("SYS_STATUS.current_battery", min(si), sorted(si)[len(si) // 2], max(si)))
if sr:
    print("  %-30s : 末值=%d %%   (最小 %d, 最大 %d)"
          % ("SYS_STATUS.battery_remaining", sr[-1], min(sr), max(sr)))
if cell_last:
    print("  %-30s : %s" % ("BATTERY_STATUS 明细", cell_last))
print("=" * 68)
if bv:
    print("\n结论: %.2f V => %.2f V/cell (8S)" % (bv[-1], bv[-1] / CELLS))
