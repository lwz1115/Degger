# -*- coding: utf-8 -*-
"""重新核算 ATC_STR_RAT_P 的有效值。
三条独立路径互相验证（第三条与单位换算无关）：
  ① FF项 / (desired/57.2958)  -> 与 ATC_STR_RAT_FF 比，验证单位模型
  ② P項 / ((desired-achieved)/57.2958) -> 与 ATC_STR_RAT_P 比
  ③ kp = kff × (P項/FF項)     -> 不需要任何单位假设
"""
import collections
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

m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)

PNAMES = ["ATC_STR_RAT_P", "ATC_STR_RAT_FF", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
          "ATC_SPEED_P", "ATC_SPEED_I", "GCS_PID_MASK", "WP_SPEED"]
par = {}
for p in PNAMES:
    m.mav.param_request_read_send(1, 1, p.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.replace("\x00", "").strip() == p:
            par[p] = msg.param_value
            break
print("--- 飞控当前参数 ---")
for p in PNAMES:
    print("   %-18s = %s" % (p, ("%.10g" % par[p]) if p in par else "(未读到)"))

samples = {5: [], 4: []}
mode = None
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
    elif ty == "HEARTBEAT" and getattr(msg, "custom_mode", None) is not None:
        mode = msg.custom_mode

print("--- 模式 = %s ---" % mode)


def mean(v):
    return sum(v) / len(v) if v else float("nan")


for ax, pname in ((5, "ATC_STR_RAT_P"), (4, "ATC_SPEED_P")):
    s = samples[ax]
    if len(s) < 10:
        print("axis=%d 样本不足(%d)" % (ax, len(s)))
        continue
    distinct = len(set(round(x[0], 6) for x in s))
    print()
    print("=== axis=%d  样本 %d  唯一desired值个数=%d %s"
          % (ax, len(s), distinct, "<< 冻结(旧值)" if distinct <= 2 else "(实时变化)"))
    kff = par.get("ATC_STR_RAT_FF") if ax == 5 else None
    # ① FF
    g = [x[5] / (x[0] / R2D) for x in s if abs(x[0]) > 1e-4]
    if g and kff:
        o = mean(g)
        print("  ① 反算 FF = %.6f   参数=%.6g   比值=%.4f" % (o, kff, o / kff))
    # ② P  (只有转向环需要 /57.2958)
    div = R2D if ax == 5 else 1.0
    gp = [x[2] / ((x[0] - x[1]) / div) for x in s if abs((x[0] - x[1]) / div) > 0.02]
    if gp:
        o = mean(gp)
        p = par.get(pname)
        print("  ② 反算 P  = %.6f   参数=%.6g   比值=%.4f%s"
              % (o, p if p is not None else float("nan"),
                 (o / p) if p else float("nan"),
                 "   <== 不符" if (p and abs(o / p - 1) > 0.15) else ""))
        print("     P项/误差 逐样本: min=%.6f  max=%.6f  标准差=%.6f"
              % (min(gp), max(gp),
                 (sum((x - o) ** 2 for x in gp) / len(gp)) ** 0.5))
    # ③ 与单位无关
    if ax == 5 and kff:
        r = [x[2] / x[5] for x in s if abs(x[5]) > 1e-9]
        if r:
            rr = mean(r)
            print("  ③ 与单位无关: P項/FF項 = %.4f  ->  P = FF × 比值 = %.6f"
                  % (rr, kff * rr))
    # 分项占比
    tot = {k: sum(abs(x[i]) for x in s) for k, i in
           (("FF", 5), ("P", 2), ("I", 3), ("D", 4))}
    g2 = sum(tot.values())
    print("     占比: " + "  ".join("%s=%.1f%%" % (k, v / g2 * 100) for k, v in tot.items()))
    print("     均值: " + "  ".join("|%s|=%.6f" % (k, tot[k] / len(s)) for k in ("FF", "P", "I", "D")))
