# -*- coding: utf-8 -*-
"""检查 GCS_PID_MASK 改动结果：现在到底流了哪几条 PID_TUNING(哪些 axis)。"""
import collections
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 14.0

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)
print("已连接", ADDR)

AXIS_NAME = {0: "None", 1: "Roll", 2: "Pitch(或 Speed?)", 3: "Yaw",
             4: "Throttle/AccZ", 5: "Steering 转向", 6: "Landing", 7: "Aux/Speed?"}

# 先读参数
for p in ("GCS_PID_MASK", "ATC_STR_RAT_P", "ATC_STR_RAT_FF",
          "ATC_SPEED_P", "ATC_SPEED_I", "ATC_SPEED_D",
          "CRUISE_SPEED", "CRUISE_THROTTLE", "WP_SPEED", "MOT_SLEWRITE"):
    m.mav.param_request_read_send(1, 1, p.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.replace("\x00", "").strip() == p:
            print("   %-18s = %g" % (p, msg.param_value))
            break

print("--- 采集 %.0f 秒 PID_TUNING ---" % SEC)
cnt = collections.Counter()
last = {}
t0 = time.time()
while time.time() - t0 < SEC:
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is None:
        continue
    if msg.get_type() == "PID_TUNING":
        ax = getattr(msg, "axis", -1)
        cnt[ax] += 1
        last[ax] = msg

print("出现的 axis 及速率:")
if not cnt:
    print("   (一条都没收到)")
for ax in sorted(cnt):
    print("   axis=%-3s %-18s %6.2f Hz" % (ax, AXIS_NAME.get(ax, "?"), cnt[ax] / SEC))

for ax in sorted(last):
    mm = last[ax]
    print("--- axis=%s 样例 ---" % ax)
    print("   desired=%9.4f  achieved=%9.4f" % (mm.desired, mm.achieved))
    print("   FF=%11.6f  P=%11.6f  I=%11.6f  D=%11.6f" % (mm.FF, mm.P, mm.I, mm.D))
