#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read RC channel mapping + option assignment from the flight controller."""
import sys
import time

from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"

WANT = ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW", "MODE_CH",
        "SERVO1_FUNCTION", "SERVO2_FUNCTION", "SERVO3_FUNCTION",
        "SERVO4_FUNCTION", "SERVO5_FUNCTION",
        "RC_OPTIONS", "FS_THR_VALUE", "FS_THR_ENABLE", "ARMING_CHECK"]
WANT += ["RC%d_OPTION" % i for i in range(5, 17)]
WANT += ["RC%d_MIN" % i for i in range(5, 17)]

m = mavutil.mavlink_connection("tcp:" + addr, timeout=6)
m.wait_heartbeat(timeout=8)
got = {}
for p in WANT:
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  p.encode("ascii"), -1)
    time.sleep(0.03)
deadline = time.time() + 20
while time.time() < deadline and len(got) < len(WANT):
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if msg is None:
        continue
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode("ascii", "replace")
    got[pid.rstrip("\x00")] = msg.param_value

print("--- channel <-> function map ---")
for p in WANT[:14]:
    print("%-18s %s" % (p, ("%g" % got[p]) if p in got else "<no reply>"))
print("--- per-channel options (what each switch does) ---")
for i in range(5, 17):
    k = "RC%d_OPTION" % i
    km = "RC%d_MIN" % i
    v = ("%g" % got[k]) if k in got else "?"
    mv = ("%g" % got[km]) if km in got else "?"
    print("ch%-3d option=%-8s min=%-8s" % (i, v, mv))
