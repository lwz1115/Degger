#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G12 RC monitor -- read-only. Prints RC_CHANNELS live.

Usage:  python _rc_monitor.py 192.168.10.162:5760 [seconds]

ASCII only (Windows console is GBK).
"""
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

RC_CHANNELS = 65
RC_CHANNELS_RAW = 35

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

print("connecting tcp:%s ..." % addr)
m = mavutil.mavlink_connection("tcp:" + addr, timeout=6)

hb = m.wait_heartbeat(timeout=8)
if hb is None:
    print("NO HEARTBEAT -- link dead")
    sys.exit(2)
print("HEARTBEAT sys=%d comp=%d type=%d autopilot=%d"
      % (m.target_system, m.target_component, hb.type, hb.autopilot))

# ask for RC_CHANNELS at 10 Hz for OUR target only
try:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            RC_CHANNELS, 100000, 0, 0, 0, 0, 0)
except Exception as e:
    print("set_interval failed: %r" % (e,))

stats = defaultdict(lambda: [9999.0, 0.0, 0.0, 0])   # min,max,sum,n
t0 = time.time()
last_print = 0.0
n_rc = 0
rssi = None

print("-" * 78)
print("%6s | %s" % ("t", " ".join("ch%-5d" % i for i in range(1, 9))))
print("-" * 78)

while time.time() - t0 < dur:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    ty = msg.get_type()
    if ty not in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
        continue
    n_rc += 1
    if ty == "RC_CHANNELS":
        vals = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw,
                msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw]
        rssi = msg.rssi
    else:
        vals = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]
        vals += [0] * 8
        rssi = msg.rssi
    for i, v in enumerate(vals):
        if v:
            s = stats[i + 1]
            s[0] = min(s[0], v)
            s[1] = max(s[1], v)
            s[2] += v
            s[3] += 1
    now = time.time() - t0
    if now - last_print >= 0.5:
        last_print = now
        print("%6.1f | %s" % (now, " ".join("%-5d" % v for v in vals[:8])))

print("-" * 78)
print("RC msgs: %d in %.1fs  (%.1f Hz)   rssi=%s" % (n_rc, dur, n_rc / dur, rssi))
print("channel statistics over window:")
print("%-6s %-8s %-8s %-8s %-8s %s" % ("ch", "min", "max", "mean", "span", "activity"))
for i in range(1, 17):
    s = stats[i]
    if s[3] == 0:
        print("%-6d %s" % (i, "-- no data --"))
        continue
    mean = s[2] / s[3]
    span = s[1] - s[0]
    tag = "MOVING" if span > 30 else ("static" if span < 5 else "slight")
    print("%-6d %-8.0f %-8.0f %-8.1f %-8.0f %s" % (i, s[0], s[1], mean, span, tag))
