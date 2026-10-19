# -*- coding: utf-8 -*-
"""查清提速 PID_TUNING / SERVO_OUTPUT_RAW 后，ATTITUDE 为什么掉到 0。
逐项隔离测试，结束恢复默认。"""
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
IDS = {n: getattr(C, "MAVLINK_MSG_ID_" + n) for n in
       ("ATTITUDE", "PID_TUNING", "SERVO_OUTPUT_RAW", "VFR_HUD", "AHRS2", "RC_CHANNELS")}
WATCH = tuple(IDS.values())


def set_interval(msg_id, us):
    m.mav.command_long_send(1, 1, C.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, us, 0, 0, 0, 0, 0)


def measure(sec):
    cnt = collections.Counter()
    t0 = time.time()
    while time.time() - t0 < sec:
        msg = m.recv_match(blocking=True, timeout=0.4)
        if msg is not None and msg.get_msgId() in WATCH:
            cnt[msg.get_type()] += 1
    return {k: cnt.get(k, 0) / sec for k in IDS}


def show(tag, d):
    print("%-34s " % tag + "  ".join("%s=%.1f" % (k, v) for k, v in d.items()))


show("基线", measure(4))
set_interval(IDS["PID_TUNING"], 50000)
show("仅 PID_TUNING->20Hz", measure(4))
set_interval(IDS["SERVO_OUTPUT_RAW"], 100000)
show("再 + SERVO_OUTPUT_RAW->10Hz", measure(4))
set_interval(IDS["ATTITUDE"], 100000)
show("再显式请求 ATTITUDE->10Hz", measure(4))
set_interval(IDS["ATTITUDE"], 200000)
show("ATTITUDE 改成 ->5Hz", measure(4))

print("--- 恢复默认 ---")
for v in IDS.values():
    set_interval(v, -1)
show("恢复后", measure(4))
