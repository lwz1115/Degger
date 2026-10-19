# -*- coding: utf-8 -*-
"""按前缀全量导出参数，确认是否拉全 + 把关键类别的每一项都列出来。只读。"""
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import usv_debug as U
from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
PREFIXES = sys.argv[2].split(",") if len(sys.argv) > 2 else None

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)

allp = {}
# 反复请求几次，直到数量稳定（防止丢包导致拉不全）
for _try in range(3):
    m.mav.param_request_list_send(1, 1)
    t0 = time.time()
    while time.time() - t0 < 40:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            continue
        nm = msg.param_id.replace("\x00", "").strip()
        allp[nm] = msg.param_value
    print("第%d轮后累计参数数 = %d" % (_try + 1, len(allp)))

if PREFIXES is None:
    PREFIXES = ["COMPASS", "GPS", "ARMING", "EK3", "ATC_", "MOT_", "SERVO", "RC",
                "NAV", "L1", "WP_", "TURN", "ACRO", "CRUISE", "RTL"]
print()
for pre in PREFIXES:
    hits = sorted(k for k in allp if k.startswith(pre))
    if not hits:
        print("=== %s* : 无 ===" % pre)
        continue
    print("=== %s* (%d 项) ===" % (pre, len(hits)))
    for k in hits:
        print("   %-26s = %g" % (k, allp[k]))
    print()
