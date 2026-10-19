# -*- coding: utf-8 -*-
"""航段"摆动恢复"分析: 用滑动窗口 RMS 曲线看大扰动后误差多快收回来
用法: python3 _recovery.py <csv> <t0> <t1> <标签>
"""
import csv, sys, math

path = sys.argv[1]
T0, T1 = float(sys.argv[2]), float(sys.argv[3])
label = sys.argv[4] if len(sys.argv) > 4 else "段"
WIN = 5.0          # 滑动窗(秒)
STEP = 1.0

rows = []
with open(path, encoding="utf-8", errors="ignore") as f:
    for x in csv.DictReader(f):
        try:
            t = float(x["t"])
        except Exception:
            continue
        if not (T0 <= t <= T1):
            continue
        try:
            rows.append((t, float(x["des"]), float(x["ach"]),
                         float(x["thr"]), float(x["gs"])))
        except Exception:
            pass

print("=" * 78)
print(" %s   样本 %d   t=%.1f..%.1f s" % (label, len(rows), T0, T1))
print("=" * 78)
if len(rows) < 50:
    sys.exit(0)

err = [(t, d - a) for (t, d, a, _th, _gs) in rows]
t_end = err[-1][0]

# 滑动窗口 RMS
curve = []
t = err[0][0]
while t + WIN <= t_end:
    w = [e for (tt, e) in err if t <= tt < t + WIN]
    if len(w) >= 10:
        curve.append((t - err[0][0], math.sqrt(sum(x * x for x in w) / len(w))))
    t += STEP

if not curve:
    sys.exit(0)

vals = sorted(r for _t, r in curve)
med = vals[len(vals) // 2]
pk_t, pk_r = max(curve, key=lambda x: x[1])
print("  滑窗(%ds)RMS 曲线:  中位=%.1f  峰值=%.1f @ t=%.0fs  (峰值/中位=%.2f)"
      % (WIN, med, pk_r, pk_t, pk_r / max(med, 1e-9)))

# 从峰值之后, 找回到"中位*1.5"以内所需时间
thr = max(med * 1.5, 2.0)
rec = None
for (tt, rr) in curve:
    if tt > pk_t and rr <= thr:
        rec = tt - pk_t
        break
print("  从峰值回落到中位x1.5(%.1f)以内: %s" % (thr, ("%.0f s" % rec) if rec else "本段内未恢复!"))

# 分段 RMS: 峰值后每 10s 一段, 看衰减形状
print()
print("  峰值后的衰减 (每 10s 一段的 RMS):")
seg = []
t = pk_t
while t < curve[-1][0]:
    w = [r for (tt, r) in curve if t <= tt < t + 10.0]
    if w:
        seg.append((t, sum(w) / len(w)))
    t += 10.0
line = "   ".join("%.0fs:%.0f" % (a, b) for a, b in seg[:12])
print("    " + line)

# 前/后半, 以及四分之一段
n = len(curve)
q = n // 4
if q:
    parts = []
    for k in range(4):
        w = [r for _t, r in curve[k * q:(k + 1) * q]]
        parts.append(sum(w) / len(w) if w else 0)
    print()
    print("  四等分 RMS 走势:  %.1f -> %.1f -> %.1f -> %.1f"
          % tuple(parts))

# 原始曲线摘要(方便看形状)
print()
print("  滑窗 RMS 时间序列 (每 ~20s 一个点):")
line = []
for k in range(0, len(curve), max(1, int(20 / STEP))):
    tt, rr = curve[k]
    line.append("%.0f" % rr)
print("    " + "  ".join(line))
