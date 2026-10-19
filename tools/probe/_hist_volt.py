# -*- coding: utf-8 -*-
"""从历史 tlog 里提取电池电压历史(只读)"""
import sys, glob, os
from pymavlink import mavutil

files = sys.argv[1:] or sorted(glob.glob("Logs/*.tlog"))
CELLS = 8  # 8S

for fn in files:
    if os.path.getsize(fn) < 200000:
        continue
    print("=" * 70)
    print("文件: %s  (%.1f MB)" % (os.path.basename(fn),
                                  os.path.getsize(fn) / 1e6))
    try:
        m = mavutil.mavlink_connection(fn)
    except Exception as e:
        print("  打不开: %r" % (e,))
        continue

    vs = []          # SYS_STATUS 电压
    bs = []          # BATTERY_STATUS 电压
    n = 0
    while True:
        try:
            msg = m.recv_match(blocking=False)
        except Exception:
            break
        if msg is None:
            break
        n += 1
        ty = msg.get_type()
        if ty == "SYS_STATUS":
            v = getattr(msg, "voltage_battery", 0)
            if v and v != 65535:
                vs.append(v / 1000.0)
        elif ty == "BATTERY_STATUS":
            v = getattr(msg, "voltages", None)
            if v:
                tot = sum(x for x in v[:10] if x and x != 65535)
                if tot:
                    bs.append(tot / 1000.0)
        if n > 4000000:
            break

    print("  总报文 %d" % n)
    for tag, arr in (("SYS_STATUS.voltage_battery", vs),
                     ("BATTERY_STATUS(单元格求和)", bs)):
        if not arr:
            print("  %-26s : 无数据" % tag)
            continue
        arr_s = sorted(arr)
        print("  %-26s : n=%-7d min=%.2fV  中位=%.2fV  max=%.2fV  末值=%.2fV"
              % (tag, len(arr), arr_s[0], arr_s[len(arr_s) // 2], arr_s[-1],
                 arr[-1]))
        print("  %-26s   (按 %dS 折算 中位=%.2fV/cell  末值=%.2fV/cell)"
              % ("", CELLS, arr_s[len(arr_s) // 2] / CELLS, arr[-1] / CELLS))
    print()
