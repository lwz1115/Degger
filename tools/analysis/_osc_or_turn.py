# -*- coding: utf-8 -*-
"""判别: 误差 RMS 的峰是"控制器振荡"还是"新的机动(转弯)需求"
方法: 逐 10s 同时看 平均|期望| 与 误差RMS, 若两者同峰 -> 是需求驱动, 不是自激"""
import csv, sys, math

path = sys.argv[1]
T0, T1 = float(sys.argv[2]), float(sys.argv[3])
label = sys.argv[4] if len(sys.argv) > 4 else "段"
BIN = 10.0

rows = []
with open(path, encoding="utf-8", errors="ignore") as f:
    for x in csv.DictReader(f):
        try:
            t = float(x["t"])
        except Exception:
            continue
        if T0 <= t <= T1:
            try:
                rows.append((t, float(x["des"]), float(x["ach"]), float(x["gs"])))
            except Exception:
                pass
if len(rows) < 50:
    sys.exit(0)

t_start = rows[0][0]
print("=" * 86)
print(" %s   样本 %d" % (label, len(rows)))
print("=" * 86)
print("  时刻    平均|期望|  误差RMS  RMS/|期望|  地速    <- 说明")
print("  " + "-" * 80)

bins = []
t = t_start
while t < rows[-1][0]:
    w = [r for r in rows if t <= r[0] < t + BIN]
    if len(w) >= 10:
        ad = sum(abs(d) for (_t, d, _a, _g) in w) / len(w)
        er = [d - a for (_t, d, a, _g) in w]
        rms = math.sqrt(sum(x * x for x in er) / len(er))
        gs = sum(g for (*_x, g) in w) / len(w)
        bins.append((t - t_start, ad, rms, gs))
    t += BIN

# 用"平均|期望|"判断是否在机动; 用 RMS/|期望| 判断跟踪是否差
ads = sorted(b[1] for b in bins)
med_ad = ads[len(ads) // 2]
print("  中位 平均|期望| = %.1f deg/s" % med_ad)

for (tt, ad, rms, gs) in bins:
    tag = ""
    if ad > med_ad * 1.8:
        tag = "机动中(强需求)"
    elif ad < med_ad * 0.4:
        tag = "怠速/直线"
    ratio = rms / max(ad, 1e-6)
    flag = ""
    if ratio > 1.0 and ad < med_ad * 0.6:
        flag = "  <<< 需求小但误差大 = 真异常"
    print("  %5.0fs  %8.1f  %8.1f  %9.2f  %6.2f  %s%s"
          % (tt, ad, rms, ratio, gs, tag, flag))

print()
print("  判读:")
print("    - RMS 峰与 |期望| 峰同时出现  -> 是机动需求驱动的正常跟踪误差")
print("    - |期望| 已回落但 RMS 仍高    -> 才是控制器自身在振荡(拉不回来)")
ad_ok = [b for b in bins if b[1] > med_ad * 1.8]
print("    强需求时段: %d/%d 个区间" % (len(ad_ok), len(bins)))
if ad_ok:
    print("    强需求区间的平均 RMS/|期望| = %.2f"
          % (sum(b[2] / max(b[1], 1e-6) for b in ad_ok) / len(ad_ok)))
low = [b for b in bins if b[1] < med_ad * 0.6]
if low:
    print("    低需求区间的平均 RMS/|期望| = %.2f  (>1 就是拉不回来)"
          % (sum(b[2] / max(b[1], 1e-6) for b in low) / len(low)))
