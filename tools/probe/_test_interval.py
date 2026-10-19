# -*- coding: utf-8 -*-
"""测试 SET_MESSAGE_INTERVAL 是否真的能改流速率（只发流速率请求，不改持久参数，结束恢复默认）。
对照项：
  SERVO_OUTPUT_RAW(36)      基线 2Hz  -> 想提到 10Hz（差速输出目前是瓶颈）
  PID_TUNING(194)           基线 10Hz -> 想提到 20Hz（提高 RAT 诊断分辨率）
  NAV_CONTROLLER_OUTPUT(62) 基线 0     -> 想提到 5Hz（外环诊断）
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

C = mavutil.mavlink
IDS = {
    "SERVO_OUTPUT_RAW": C.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    "PID_TUNING": C.MAVLINK_MSG_ID_PID_TUNING,
    "NAV_CONTROLLER_OUTPUT": C.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
    "ATTITUDE": C.MAVLINK_MSG_ID_ATTITUDE,
}


def measure(sec, ids):
    want = {v: k for k, v in ids.items()}
    cnt = collections.Counter()
    names = {k: 0 for k in ids}
    t0 = time.time()
    while time.time() - t0 < sec:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mid = msg.get_msgId()
        if mid in want:
            names[want[mid]] += 1
    return {k: v / sec for k, v in names.items()}


def set_interval(msg_id, us):
    m.mav.command_long_send(1, 1, C.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, us, 0, 0, 0, 0, 0)


print("=== 基线速率 (3s) ===")
base = measure(3, IDS)
for k, v in base.items():
    print("   %-22s %.2f Hz" % (k, v))

TRIALS = [
    ("SERVO_OUTPUT_RAW", 100000, "10Hz"),      # 100 ms
    ("PID_TUNING", 50000, "20Hz"),             # 50 ms
    ("NAV_CONTROLLER_OUTPUT", 200000, "5Hz"),  # 200 ms
]
for name, us, want in TRIALS:
    set_interval(IDS[name], us)
    got = measure(4, IDS)
    print("--- 请求 %s -> %s : 实测 %.2f Hz" % (name, want, got[name]))

# 一起开着再看一次总量
print("=== 三项同时请求后 (5s) ===")
set_interval(IDS["SERVO_OUTPUT_RAW"], 100000)
set_interval(IDS["PID_TUNING"], 50000)
set_interval(IDS["NAV_CONTROLLER_OUTPUT"], 200000)
tot = 0
t0 = time.time()
cnt = collections.Counter()
while time.time() - t0 < 5:
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is not None:
        cnt[msg.get_type()] += 1
        tot += 1
for k in ("PID_TUNING", "SERVO_OUTPUT_RAW", "NAV_CONTROLLER_OUTPUT", "ATTITUDE", "VFR_HUD"):
    print("   %-22s %.2f Hz" % (k, cnt.get(k, 0) / 5.0))
print("   全部消息合计 %.1f 条/秒" % (tot / 5.0))

print("=== 恢复默认 ===")
for name in IDS:
    set_interval(IDS[name], -1)
print("done")
