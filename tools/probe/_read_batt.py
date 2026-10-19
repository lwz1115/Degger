# -*- coding: utf-8 -*-
"""只读: 读取电池/解锁/罗盘相关参数 (param_request_read, 不写任何参数)"""
import sys, time
from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.138:5760"

NAMES = [
    # 电池监测
    "BATT_MONITOR", "BATT_VOLT_PIN", "BATT_CURR_PIN", "BATT_VOLT_MULT",
    "BATT_AMP_PERVLT", "BATT_CAPACITY",
    # 解锁电压门限 (4.4+ 新名 / 旧名)
    "ARMING_VOLT_MIN", "ARMING_VOLT2_MIN", "BATT_ARM_VOLT", "BATT_ARM_MAH",
    "BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_LOW_MAH", "BATT_CRT_MAH",
    "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT",
    # 解锁检查
    "ARMING_CHECK", "ARMING_REQUIRE", "ARMING_RUDDER",
    # 罗盘
    "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3", "COMPASS_CAL_FIT",
    "COMPASS_EXTERNAL", "COMPASS_ORIENT", "COMPASS_DEC",
    # 失效保护
    "FS_THR_ENABLE", "FS_THR_VALUE", "FS_ACTION", "FS_TIMEOUT",
    "FS_GCS_ENABLE", "FS_EKF_ACTION", "FS_OPTIONS", "FS_CRASH_CHECK",
    # 舵机 (核对是否仍是 1100/1900)
    "SERVO1_MIN", "SERVO1_MAX", "SERVO1_TRIM", "SERVO1_FUNCTION",
    "SERVO3_MIN", "SERVO3_MAX", "SERVO3_TRIM", "SERVO3_FUNCTION",
    # 转向/速度 (核对当前值)
    "ATC_STR_RAT_P", "ATC_STR_RAT_MAX", "ATC_STR_ANG_P", "WP_SPEED",
    "TURN_RADIUS", "CRUISE_SPEED", "CRUISE_THROTTLE", "GCS_PID_MASK",
]

want = set(NAMES)
got = {}

m = mavutil.mavlink_connection("tcp:" + addr, source_system=255)
if not m.wait_heartbeat(timeout=10):
    print("没有 HEARTBEAT")
    sys.exit(1)
print("已连接 %s，读取 %d 个参数 (只读)\n" % (addr, len(NAMES)))

t0 = time.time()
last_ask = 0.0
while time.time() - t0 < 40:
    now = time.time() - t0
    if now - last_ask > 4.0 and len(got) < len(NAMES):
        last_ask = now
        for nm in NAMES:
            if nm in got:
                continue
            try:
                m.mav.param_request_read_send(1, 1, nm.encode(), -1)
            except Exception:
                pass
        time.sleep(0.05)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if msg is None:
        continue
    nm = msg.param_id
    if isinstance(nm, bytes):
        nm = nm.decode(errors="replace")
    nm = nm.replace("\x00", "").strip()
    if nm in want and nm not in got:
        got[nm] = float(msg.param_value)

print("=" * 62)
print(" 电池监测")
print("=" * 62)
for nm in ("BATT_MONITOR", "BATT_VOLT_PIN", "BATT_CURR_PIN", "BATT_VOLT_MULT",
           "BATT_AMP_PERVLT", "BATT_CAPACITY"):
    print("   %-18s = %s" % (nm, got.get(nm, "(未取到)")))

print("\n" + "=" * 62)
print(" 解锁门限 / 告警门限   <<< 报错源头")
print("=" * 62)
for nm in ("ARMING_VOLT_MIN", "ARMING_VOLT2_MIN", "BATT_ARM_VOLT",
           "BATT_ARM_MAH", "BATT_LOW_VOLT", "BATT_CRT_VOLT",
           "BATT_LOW_MAH", "BATT_CRT_MAH",
           "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT"):
    v = got.get(nm)
    mark = ""
    if nm == "ARMING_VOLT_MIN" and v is not None and v > 18.40:
        mark = "   <<< 18.40V 低于此值 -> 拒绝解锁"
    print("   %-18s = %s%s" % (nm, v if v is not None else "(未取到)", mark))

print("\n" + "=" * 62)
print(" 解锁检查位 / 罗盘")
print("=" * 62)
for nm in ("ARMING_CHECK", "ARMING_REQUIRE", "ARMING_RUDDER",
           "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",
           "COMPASS_CAL_FIT", "COMPASS_EXTERNAL", "COMPASS_ORIENT"):
    print("   %-18s = %s" % (nm, got.get(nm, "(未取到)")))

print("\n" + "=" * 62)
print(" 失效保护")
print("=" * 62)
for nm in ("FS_THR_ENABLE", "FS_THR_VALUE", "FS_ACTION", "FS_TIMEOUT",
           "FS_GCS_ENABLE", "FS_EKF_ACTION", "FS_OPTIONS", "FS_CRASH_CHECK"):
    print("   %-18s = %s" % (nm, got.get(nm, "(未取到)")))

print("\n" + "=" * 62)
print(" 舵机 / 转向 (当前实时值)")
print("=" * 62)
for nm in ("SERVO1_FUNCTION", "SERVO1_MIN", "SERVO1_MAX", "SERVO1_TRIM",
           "SERVO3_FUNCTION", "SERVO3_MIN", "SERVO3_MAX", "SERVO3_TRIM",
           "ATC_STR_RAT_P", "ATC_STR_RAT_MAX", "ATC_STR_ANG_P",
           "WP_SPEED", "TURN_RADIUS", "CRUISE_SPEED", "CRUISE_THROTTLE",
           "GCS_PID_MASK"):
    print("   %-18s = %s" % (nm, got.get(nm, "(未取到)")))

miss = [n for n in NAMES if n not in got]
if miss:
    print("\n未返回的参数: %s" % ", ".join(miss))
