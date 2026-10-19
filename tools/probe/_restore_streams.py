# -*- coding: utf-8 -*-
"""恢复被 interval=-1 误关掉的消息流。
本飞控上 SET_MESSAGE_INTERVAL 的 -1 = 停止发送（不是恢复默认），
所以这里全部用【显式正速率】写回原值。"""
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

# 消息名 -> 出厂默认速率(Hz)，均为实测得到的原值
RESTORE = [
    ("ATTITUDE", 10),
    ("AHRS2", 10),
    ("VFR_HUD", 10),
    ("PID_TUNING", 10),
    ("SERVO_OUTPUT_RAW", 2),
    ("RC_CHANNELS", 2),
]

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)
C = mavutil.mavlink
IDS = {n: getattr(C, "MAVLINK_MSG_ID_" + n, None) for n, _ in RESTORE}
WATCH = set(v for v in IDS.values() if v is not None)


def measure(sec):
    cnt = collections.Counter()
    t0 = time.time()
    while time.time() - t0 < sec:
        msg = m.recv_match(blocking=True, timeout=0.4)
        if msg is not None and msg.get_msgId() in WATCH:
            cnt[msg.get_type()] += 1
    return {k: cnt.get(k, 0) / sec for k, _ in RESTORE}


print("--- 恢复前 ---")
for k, v in measure(4).items():
    print("   %-18s %.2f Hz" % (k, v))

print("--- 写回原速率 ---")
for name, hz in RESTORE:
    mid = IDS[name]
    if mid is None:
        continue
    m.mav.command_long_send(1, 1, C.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mid, int(1000000.0 / hz), 0, 0, 0, 0, 0)
    time.sleep(0.05)
    print("   %s -> %s Hz" % (name, hz))

time.sleep(1.0)
print("--- 恢复后 ---")
ok = True
got = measure(5)
for k, hz in RESTORE:
    v = got.get(k, 0.0)
    mark = "OK " if v >= hz * 0.7 else "!! "
    if v < hz * 0.7:
        ok = False
    print("   %s%-18s 期望 %.1f Hz  实测 %.2f Hz" % (mark, k, hz, v))
print("结果:", "全部恢复" if ok else "仍有条目未恢复")
