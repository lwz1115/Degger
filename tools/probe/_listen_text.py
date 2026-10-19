# -*- coding: utf-8 -*-
"""只读监听: 抓飞控的 STATUSTEXT 告警 + SYS_STATUS 健康位 + 模式/解锁状态
用法: python3 _listen_text.py <addr> [sec]
绝不写任何参数/指令。
"""
import sys, time, collections
from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.138:5760"
sec = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0

SEV = {0: "EMERG", 1: "ALERT", 2: "CRIT ", 3: "ERROR", 4: "WARN ", 5: "NOTIC",
       6: "INFO ", 7: "DEBUG"}

# 常见 SYS_STATUS 健康位名
BITS = {
    0: "3D_GYRO", 1: "3D_ACCEL", 2: "3D_MAG", 3: "ABS_PRESSURE",
    4: "DIFF_PRESSURE", 5: "GPS", 6: "OPTICAL_FLOW", 7: "VISION_POSITION",
    8: "LASER_POSITION", 9: "EXTERNAL_GROUND_TRUTH", 10: "ANGULAR_RATE_CONTROL",
    11: "ATTITUDE_STABILIZATION", 12: "YAW_POSITION", 13: "Z_ALTITUDE_CONTROL",
    14: "XY_POSITION_CONTROL", 15: "MOTOR_OUTPUTS", 16: "RC_RECEIVER",
    17: "3D_GYRO2", 18: "3D_ACCEL2", 19: "3D_MAG2", 20: "GEO_FENCE",
    21: "AHRS", 22: "TERRAIN", 23: "REVERSE_MOTOR", 24: "LOGGING",
    25: "BATTERY", 26: "PROXIMITY", 27: "SATCOM", 28: "PREARM_CHECK",
    29: "OBSTACLE_AVOIDANCE", 30: "PROPULSION", 31: "EXTENSION_USED0",
}

MODENAME = {0: "MANUAL", 1: "ACRO", 2: "STEERING", 3: "HOLD", 4: "LOITER",
            5: "FOLLOW", 6: "SIMPLE", 7: "DOCK", 10: "AUTO", 11: "RTL",
            12: "SMART_RTL", 15: "GUIDED", 16: "INITIALISING"}

print("连接 %s ..." % addr)
m = mavutil.mavlink_connection("tcp:" + addr, source_system=255)
if not m.wait_heartbeat(timeout=10):
    print("没有 HEARTBEAT，连不上")
    sys.exit(1)
print("已连接. 监听 %.0f 秒 (只读, 不发任何指令)\n" % sec)

t0 = time.time()
texts = collections.Counter()
order = []
last_hb = 0.0
last_ss = 0.0
n = collections.Counter()

while time.time() - t0 < sec:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    ty = msg.get_type()
    n[ty] += 1
    now = time.time() - t0

    if ty == "STATUSTEXT":
        sev = getattr(msg, "severity", 6)
        txt = msg.text
        if isinstance(txt, bytes):
            txt = txt.decode(errors="replace")
        txt = txt.replace("\x00", "").strip()
        texts[(sev, txt)] += 1
        if len(order) < 400:
            order.append((now, sev, txt))
            print("  [%6.1fs] %s  %s" % (now, SEV.get(sev, "?"), txt))

    elif ty == "SYS_STATUS":
        if now - last_ss < 5.0:
            continue
        last_ss = now
        pres = getattr(msg, "onboard_control_sensors_present", 0)
        en = getattr(msg, "onboard_control_sensors_enabled", 0)
        he = getattr(msg, "onboard_control_sensors_health", 0)
        bad = [BITS.get(i, "bit%d" % i) for i in range(32)
               if (en >> i) & 1 and not (he >> i) & 1]
        print("  [%6.1fs] SYS_STATUS  电压=%.2fV 电流=%.1fA  剩余=%d%%  "
              "不健康=%s" % (now,
                             getattr(msg, "voltage_battery", 0) / 1000.0,
                             getattr(msg, "current_battery", 0) / 100.0,
                             getattr(msg, "battery_remaining", -1),
                             bad or "无"))

    elif ty == "HEARTBEAT":
        if now - last_hb < 5.0:
            continue
        last_hb = now
        cm = getattr(msg, "custom_mode", -1)
        armed = bool(getattr(msg, "base_mode", 0) & 128)
        print("  [%6.1fs] HEARTBEAT  mode=%-10s armed=%s  system_status=%d"
              % (now, MODENAME.get(cm, str(cm)), armed,
                 getattr(msg, "system_status", -1)))

print("\n================ 统计 ================")
print("收到的报文类型:", dict(n.most_common(20)))
if texts:
    print("\n重复最多的告警 (次数 从多到少):")
    for (sev, txt), c in texts.most_common(40):
        print("  %4d x  %s  %s" % (c, SEV.get(sev, "?"), txt))
else:
    print("\n监听窗口内没有任何 STATUSTEXT")
