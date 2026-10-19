#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read specific params over MAVLink to compare with reported RC_CHANNELS values."""
import sys
import time

from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
WANT = ["RC1_MIN", "RC1_TRIM", "RC1_MAX", "RC3_MIN", "RC3_TRIM", "RC3_MAX",
        "RC5_MIN", "RC5_MAX", "RC14_MIN", "RC14_TRIM", "RC14_MAX",
        "RC16_MIN", "RC16_MAX", "RC_PROTOCOLS", "RSSI_TYPE", "RCMAP_ROLL",
        "FS_THR_ENABLE", "RC_OPTIONS"]

m = mavutil.mavlink_connection("tcp:" + addr, timeout=6)
m.wait_heartbeat(timeout=8)
got = {}
deadline = time.time() + 12
for p in WANT:
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  p.encode("ascii"), -1)
    time.sleep(0.05)
while time.time() < deadline and len(got) < len(WANT):
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if msg is None:
        continue
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode("ascii", "replace")
    got[pid.rstrip("\x00")] = msg.param_value

print("%-16s %s" % ("param", "value"))
for p in WANT:
    if p in got:
        print("%-16s %g" % (p, got[p]))
    else:
        print("%-16s <no reply>" % p)
