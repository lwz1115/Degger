# -*- coding: utf-8 -*-
"""从 usv_log CSV 里反推可见异常: 模式跳变 / 数据断流 / 输出饱和"""
import csv, sys, collections

path = sys.argv[1] if len(sys.argv) > 1 else "usv_log_20260910_141357.csv"
MODENAME = {0: "MANUAL", 1: "ACRO", 2: "STEERING", 3: "HOLD", 4: "LOITER",
            5: "FOLLOW", 6: "SIMPLE", 7: "DOCK", 10: "AUTO", 11: "RTL",
            12: "SMART_RTL", 15: "GUIDED", 16: "INITIALISING"}

rows = []
with open(path, encoding="utf-8", errors="ignore") as f:
    for x in csv.DictReader(f):
        rows.append(x)

print("总行数 %d   t=%.1f..%.1f s" % (len(rows), float(rows[0]["t"]), float(rows[-1]["t"])))

# ---- 1. 模式跳变 ----
print("\n===== [1] 模式跳变 =====")
prev = None
last_t = None
for r in rows:
    try:
        m = int(float(r["mode"]))
    except Exception:
        continue
    if m != prev:
        t = float(r["t"])
        gap = "" if last_t is None else "   (距上次 %+.1fs)" % (t - last_t)
        print("  t=%8.1fs   ->  %-10s (%d)%s" % (t, MODENAME.get(m, "?"), m, gap))
        prev = m
        last_t = t

# ---- 2. 数据断流 ----
print("\n===== [2] 采样断流 (相邻样本 >0.5s) =====")
n = 0
for a, b in zip(rows, rows[1:]):
    try:
        dt = float(b["t"]) - float(a["t"])
    except Exception:
        continue
    if dt > 0.5:
        n += 1
        if n <= 15:
            print("  t=%8.1f -> %8.1f   空档 %.2fs" % (float(a["t"]), float(b["t"]), dt))
print("  断流次数 = %d" % n)


def fnum(x, k):
    try:
        return float(x[k])
    except Exception:
        return None


# ---- 3. 异常值统计 ----
print("\n===== [3] 关键量异常统计 =====")
bad = collections.Counter()
mx = collections.defaultdict(float)
for r in rows:
    for k in ("gs", "thr", "yawrate", "s1", "s3"):
        v = fnum(r, k)
        if v is None:
            bad[k + ":空"] += 1
            continue
        if abs(v) > mx[k]:
            mx[k] = abs(v)
    # CRC 错/NaN 迹象
    for k in ("des", "ach"):
        v = fnum(r, k)
        if v is None:
            bad[k + ":空"] += 1
print("  最大绝对值:", {k: round(v, 3) for k, v in mx.items()})
print("  空值:", dict(bad) or "无")

# ---- 4. 舵机输出是否长期贴边 ----
print("\n===== [4] 舵机输出贴边比例 (1100/1900) =====")
n1 = n2 = tot = 0
for r in rows:
    s1, s3 = fnum(r, "s1"), fnum(r, "s3")
    if s1 is None or s3 is None:
        continue
    tot += 1
    if s1 <= 1105 or s1 >= 1895:
        n1 += 1
    if s3 <= 1105 or s3 >= 1895:
        n2 += 1
print("  SERVO1 贴边 %d/%d (%.1f%%)" % (n1, tot, 100.0 * n1 / max(tot, 1)))
print("  SERVO3 贴边 %d/%d (%.1f%%)" % (n2, tot, 100.0 * n2 / max(tot, 1)))

# ---- 5. 差速极限 ----
print("\n===== [5] 左右差 (SERVO1-SERVO3) 分位 =====")
d = sorted((fnum(r, "s1") - fnum(r, "s3")) for r in rows
           if fnum(r, "s1") is not None and fnum(r, "s3") is not None)
if d:
    def q(p):
        return d[min(len(d) - 1, int(p * len(d)))]
    print("  min=%.0f  P1=%.0f  P50=%.0f  P99=%.0f  max=%.0f  (理论上限 +-800)"
          % (d[0], q(0.01), q(0.5), q(0.99), d[-1]))
