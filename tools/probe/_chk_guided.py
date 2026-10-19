# -*- coding: utf-8 -*-
"""GUIDED 相关核查：
1) 速度类参数现状（GUIDED 默认速度用的是 WP_SPEED）
2) 桥上能否看到其它系统(遥控器 GCS)发出的 GUIDED 目标指令
"""
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
m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)
print("已连接", ADDR)

P = ["WP_SPEED", "WP_ACCEL", "WP_JERK", "WP_RADIUS", "WP_PIVOT_ANGLE", "WP_PIVOT_RATE",
     "CRUISE_SPEED", "CRUISE_THROTTLE", "ATC_SPEED_P", "ATC_SPEED_I",
     "MOT_THR_MAX", "MOT_SLEWRATE", "ATC_STR_RAT_MAX", "TURN_RADIUS", "ACRO_TURN_RATE",
     "LOIT_SPEED", "RTL_SPEED"]
got = {}
for p in P:
    m.mav.param_request_read_send(1, 1, p.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.replace("\x00", "").strip() == p:
            got[p] = msg.param_value
            break
print("--- 速度相关参数 ---")
for p in P:
    print("   %-18s = %s" % (p, ("%g" % got[p]) if p in got else "(不存在)"))

print("--- 监听 10 秒：按来源系统统计，找目标指令 ---")
cnt = collections.Counter()
srcs = collections.Counter()
NAV_IDS = {}
for nm in ("SET_POSITION_TARGET_GLOBAL_INT", "SET_POSITION_TARGET_LOCAL_NED",
           "COMMAND_LONG", "COMMAND_INT", "MISSION_ITEM_INT", "MISSION_COUNT",
           "NAV_CONTROLLER_OUTPUT", "POSITION_TARGET_GLOBAL_INT"):
    v = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + nm, None)
    if v is not None:
        NAV_IDS[v] = nm
t0 = time.time()
while time.time() - t0 < 10:
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is None:
        continue
    srcs["sys%d/comp%d" % (msg.get_srcSystem(), msg.get_srcComponent())] += 1
    mid = msg.get_msgId()
    if mid in NAV_IDS:
        cnt[NAV_IDS[mid]] += 1

print("   收到的消息来源:")
for k, v in srcs.most_common(10):
    print("      %-14s %5d 条  (%.1f/s)" % (k, v, v / 10.0))
print("   导航/指令类消息:")
if cnt:
    for k, v in cnt.most_common():
        print("      %-34s %d 条" % (k, v))
else:
    print("      (无 —— 桥上只有飞控单向输出，看不到遥控器发给飞控的指令)")
