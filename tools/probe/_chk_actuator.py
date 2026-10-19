# -*- coding: utf-8 -*-
"""验证差速混合器的输出上限模型, 并算出执行器余量
模型(源码 AP_MotorsUGV::output_skid_steering + MOT_STR_THR_MIX>=0.5 优先转向):
    |S1-S3| <= 800 * (1 - 油门/100)
"""
import csv, sys, os

path = sys.argv[1]
HALF = 400.0          # SERVO1/3: TRIM=1500, MIN/MAX=1100/1900 -> 单侧行程 400us

rows = []
with open(path, encoding="utf-8", errors="ignore") as f:
    for x in csv.DictReader(f):
        try:
            rows.append((float(x["t"]), float(x["thr"]),
                         float(x["s1"]), float(x["s3"])))
        except Exception:
            pass

print("文件: %s   有效配对样本 %d" % (os.path.basename(path), len(rows)))
if not rows:
    sys.exit(0)

print("\n===== [1] 模型检验: |S1-S3| 是否 <= 800*(1-油门) =====")
viol = []
for (t, thr, s1, s3) in rows:
    d = abs(s1 - s3)
    bound = 2 * HALF * (1.0 - max(0.0, min(1.0, thr / 100.0)))
    if d > bound + 5.0:            # 5us 容差
        viol.append((t, thr, d, bound))
print("  违反模型的样本: %d / %d (%.2f%%)" % (len(viol), len(rows),
                                             100.0 * len(viol) / len(rows)))
for v in viol[:5]:
    print("     t=%7.1f  油门=%5.1f%%  差=%6.0f  上限=%6.0f" % v)

print("\n===== [2] 执行器占用率 (相对当前油门下的理论上限) =====")
occ = []
for (t, thr, s1, s3) in rows:
    bound = 2 * HALF * (1.0 - max(0.0, min(1.0, thr / 100.0)))
    if bound > 20:
        occ.append(abs(s1 - s3) / bound)
occ_s = sorted(occ)
if occ_s:
    def q(p):
        return occ_s[min(len(occ_s) - 1, int(p * len(occ_s)))]
    print("  占用率 中位=%.2f  P90=%.2f  P99=%.2f  max=%.2f"
          % (q(0.5), q(0.9), q(0.99), occ_s[-1]))
    for lim in (0.90, 0.95, 0.99, 1.00):
        n = sum(1 for x in occ if x >= lim)
        print("    >= %.2f 的样本: %4d (%.2f%%)" % (lim, n, 100.0 * n / len(occ)))

print("\n===== [3] 裸峰值 =====")
d = [abs(s1 - s3) for (_t, _th, s1, s3) in rows]
thr = [t3 for (_t, t3, _s1, _s3) in rows]
print("  |S1-S3| 峰值 = %.0f   (单侧行程 %0.0fus, 绝对值上限 = %.0f)"
      % (max(d), HALF, 2 * HALF))
i = d.index(max(d))
print("  峰值处油门 = %.1f%%  -> 该油门下的上限 = %.0f"
      % (thr[i], 2 * HALF * (1.0 - max(0.0, min(1.0, thr[i] / 100.0)))))
print("  S1 范围: %.0f ~ %.0f    S3 范围: %.0f ~ %.0f"
      % (min(r[2] for r in rows), max(r[2] for r in rows),
         min(r[3] for r in rows), max(r[3] for r in rows)))
n_sat1 = sum(1 for r in rows if r[2] <= 1105 or r[2] >= 1895)
n_sat3 = sum(1 for r in rows if r[3] <= 1105 or r[3] >= 1895)
print("  SERVO1 贴 1100/1900 边: %d (%.1f%%)   SERVO3: %d (%.1f%%)"
      % (n_sat1, 100.0 * n_sat1 / len(rows), n_sat3, 100.0 * n_sat3 / len(rows)))
