# -*- coding: utf-8 -*-
"""快速参数审计：单轮拉取(最多35秒)，只列关心的类别。只读。"""
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import usv_debug as U          # 启用 pymavlink 加固补丁
from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
PREFIXES = (sys.argv[2].split(",") if len(sys.argv) > 2
            else ["COMPASS", "ARMING", "EK3_SRC", "EK3_MAG", "MOT_", "SERVO1_", "SERVO3_"])

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)
allp = {}
m.mav.param_request_list_send(1, 1)
t0 = time.time()
while time.time() - t0 < 35:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
    if msg is None:
        continue
    allp[msg.param_id.replace("\x00", "").strip()] = msg.param_value
print("参数总数 = %d   (耗时 %.1fs)" % (len(allp), time.time() - t0))
print()
for pre in PREFIXES:
    hs = sorted(k for k in allp if k.startswith(pre))
    if not hs:
        print("--- %s* : 不存在 ---" % pre)
        continue
    print("--- %s*  (%d 项) ---" % (pre, len(hs)))
    for k in hs:
        print("   %-26s = %g" % (k, allp[k]))
