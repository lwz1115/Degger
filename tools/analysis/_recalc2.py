# -*- coding: utf-8 -*-
"""模式切换后重新核算：数据是否还是冻结的、反算 ATC_STR_RAT_P 是多少。"""
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
R2D = 180.0 / 3.141592653589793

# 上一轮的基线（用于判断数据有没有真的刷新）
OLD = {5: (-17.3323, -0.0699, -0.210654, -0.129709, -0.001100, -0.002723),
       4: (0.0326, -0.0159, 0.009656, 0.054978, 0.0, 0.008142)}
MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW",
         9: "CIRCLE", 10: "AUTO", 11: "RTL", 15: "GUIDED", 16: "INITIALISING"}

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)

par = {}
for p in ("ATC_STR_RAT_P", "ATC_STR_RAT_FF", "ATC_SPEED_P", "GCS_PID_MASK"):
    m.mav.param_request_read_send(1, 1, p.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.replace("\x00", "").strip() == p:
            par[p] = msg.param_value
            break
print("参数: " + "  ".join("%s=%g" % (k, v) for k, v in par.items()))

samples = {5: [], 4: []}
mode = None
armed = None
t0 = time.time()
while time.time() - t0 < SEC:
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is None:
        continue
    ty = msg.get_type()
    if ty == "PID_TUNING":
        ax = getattr(msg, "axis", -1)
        if ax in samples:
            samples[ax].append((msg.desired, msg.achieved, msg.P, msg.I, msg.D, msg.FF))
    elif ty == "HEARTBEAT":
        if getattr(msg, "custom_mode", None) is not None:
            mode = msg.custom_mode
        armed = bool(getattr(msg, "base_mode", 0) & 128)
print("模式 = %s   已解锁 = %s" % (MODES.get(mode, mode), armed))


def mean(v):
    return sum(v) / len(v) if v else float("nan")


for ax in (5, 4):
    s = samples[ax]
    print()
    if len(s) < 10:
        print("=== axis=%d  样本不足(%d) ===" % (ax, len(s)))
        continue
    last = s[-1]
    distinct = len(set(round(x[0], 5) for x in s))
    print("=== axis=%d  样本 %d  唯一desired=%d %s"
          % (ax, len(s), distinct, "<< 仍然冻结" if distinct <= 2 else "⇒ 已刷新(实时)"))
    print("   最新一条: des=%+9.4f ach=%+9.4f | P=%+11.6f I=%+11.6f D=%+11.6f FF=%+11.6f"
          % last)
    print("   上轮基线: des=%+9.4f ach=%+9.4f | P=%+11.6f I=%+11.6f D=%+11.6f FF=%+11.6f"
          % OLD[ax])
    same = all(abs(a - b) < 1e-6 for a, b in zip(last, OLD[ax]))
    print("   -> %s" % ("与上轮完全一致(数据没变)" if same else "数据已变化!"))
    div = R2D if ax == 5 else 1.0
    gp = [x[2] / ((x[0] - x[1]) / div) for x in s if abs((x[0] - x[1]) / div) > 0.02]
    pname = "ATC_STR_RAT_P" if ax == 5 else "ATC_SPEED_P"
    if gp:
        o = mean(gp)
        p = par.get(pname)
        print("   反算 P = %.6f   参数 %s = %.6g   比值 = %.4f%s"
              % (o, pname, p if p else float("nan"),
                 (o / p) if p else float("nan"),
                 "   <== 不符" if (p and abs(o / p - 1) > 0.15) else "   ✅ 吻合"))
    if ax == 5 and par.get("ATC_STR_RAT_FF"):
        r = [x[2] / x[5] for x in s if abs(x[5]) > 1e-9]
        if r:
            print("   与单位无关: FF × (P項/FF項) = %.6f" % (par["ATC_STR_RAT_FF"] * mean(r)))
