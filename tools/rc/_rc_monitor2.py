#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G12 RC monitor v2 -- read-only, change-triggered.

Prints:
  * chancount (RC_Channels::get_valid_channel_count)  <- 0 means NO valid RC input
  * every channel change >= 5
  * a heartbeat line when nothing changes

Usage: python _rc_monitor2.py 192.168.10.162:5760 [seconds]
ASCII only (Windows console is GBK).
"""
import sys
import time

from pymavlink import mavutil

RC_CHANNELS = 65

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

print("connecting tcp:%s ..." % addr)
m = mavutil.mavlink_connection("tcp:" + addr, timeout=6)
hb = m.wait_heartbeat(timeout=8)
if hb is None:
    print("NO HEARTBEAT -- link dead")
    sys.exit(2)
print("HEARTBEAT sys=%d comp=%d type=%d" % (m.target_system, m.target_component, hb.type))
try:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            RC_CHANNELS, 100000, 0, 0, 0, 0, 0)
except Exception as e:
    print("set_interval failed: %r" % (e,))

prev = None
t0 = time.time()
last_hb = 0.0
n = 0
cc_seen = set()
changes = []
stats = {}

print("-" * 76)
while time.time() - t0 < dur:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    if msg.get_type() != "RC_CHANNELS":
        continue
    n += 1
    now = time.time() - t0
    cc = getattr(msg, "chancount", None)
    cc_seen.add(cc)
    vals = list(msg.__dict__.get("_values", [])) or [
        getattr(msg, "chan%d_raw" % i, 0) for i in range(1, 17)]
    if prev is None:
        print("chancount = %s   (0 = NO valid RC input)" % cc)
        print("%6s %-5s | %s" % ("t", "cc", " ".join("ch%-4d" % i for i in range(1, 9))))
        print("-" * 76)
        print("%6.1f %-5s | %s" % (now, cc, " ".join("%-5d" % v for v in vals[:8])))
        prev = list(vals)
    else:
        moved = [i + 1 for i in range(16) if abs(vals[i] - prev[i]) >= 5]
        if moved:
            for i in range(16):
                s = stats.setdefault(i + 1, [vals[i], vals[i]])
                s[0] = min(s[0], vals[i])
                s[1] = max(s[1], vals[i])
            print("%6.1f %-5s | %s   <== changed ch%s" %
                  (now, cc, " ".join("%-5d" % v for v in vals[:8]), ",".join(map(str, moved))))
            changes.append((now, list(vals), moved))
            prev = list(vals)
    if now - last_hb >= 5.0:
        last_hb = now
        print("%6.1f %-5s | %s   (no change)" % (now, cc, " ".join("%-5d" % v for v in vals[:8])))
    for i in range(16):
        s = stats.setdefault(i + 1, [vals[i], vals[i]])
        s[0] = min(s[0], vals[i])
        s[1] = max(s[1], vals[i])

print("-" * 76)
print("RC_CHANNELS msgs: %d in %.1fs (%.1f Hz)" % (n, dur, n / max(dur, 1e-9)))
print("chancount values seen: %s" % sorted(str(x) for x in cc_seen))
print("channels that changed: %d" % len(changes))
print("%-6s %-8s %-8s %-8s %s" % ("ch", "min", "max", "span", "verdict"))
for i in range(1, 17):
    lo, hi = stats.get(i, (0, 0))
    span = hi - lo
    print("%-6d %-8d %-8d %-8d %s" % (i, lo, hi, span,
                                      "MOVED" if span >= 30 else ("static" if span == 0 else "slight")))
