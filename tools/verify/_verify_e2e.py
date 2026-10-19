# -*- coding: utf-8 -*-
"""端到端: 用合成 bundle 跑 compute_diag + save_segment, 确认报告路径没坏"""
import re, sys, os, math, random, types, tempfile, glob

if "tkinter" not in sys.modules:
    try:
        import tkinter  # noqa: F401
    except ImportError:
        def _stub(name):
            class _S(types.ModuleType):
                def __getattr__(self, attr):
                    c = type(attr, (object,), {
                        "__init__": lambda s, *a, **k: None,
                        "__call__": lambda s, *a, **k: None,
                    })
                    setattr(self, attr, c)
                    return c
            return _S(name)
        for _n in ("tkinter", "tkinter.ttk", "tkinter.messagebox",
                   "tkinter.filedialog"):
            sys.modules[_n] = _stub(_n)

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
import usv_debug as U

random.seed(7)
HZ = 10.0
DUR = 60.0
N = int(HZ * DUR)

# 场景: 真实巡航 —— 期望转向率顶到 RAT_MAX, 航向误差大, 船在跑
PID = U.PID_AXIS
SPD = U.SPEED_AXIS
hist = {PID: [], SPD: []}
att, hud, vel, gps, ekf, s1, s3, pos, mcur = [], [], [], [], [], [], [], [], []
for i in range(N):
    t = i / HZ
    he = 30.0 * math.sin(2 * math.pi * 0.06 * t)          # 航向误差 (慢)
    des = max(-75.0, min(75.0, 2.0 * he))                  # 会被 RAT_MAX 限幅
    ach = des * 0.35
    hist[PID].append((t, des, ach, 0.24, 0.12, 0.008, 0.0047))
    sp_des, sp_ach = 0.8, 0.81
    hist[SPD].append((t, sp_des, sp_ach, 0.02, 0.12, 0.0, 0.19))
    att.append((t, math.radians(ach), 0.0, 0.0, math.radians(10 * math.sin(t / 5))))
    hud.append((t, 0.82, 29.8))
    vel.append((t, 0.8, 0.05))
    gps.append((t, 3, 28, 60.0))
    ekf.append((t, 0x33F, 0.004, 0.07))
    s1.append((t, 1500 + 400 * math.sin(t / 3)))
    s3.append((t, 1500 - 400 * math.sin(t / 3)))
    mcur.append((t, 2))
    pos.append((t, -35.0 + i * 1e-5, 149.0 + i * 1e-5, 500.0, 80.0, 5.0, 10.0))
mission = [(0, -35.0, 149.0, 16, 0, 0, 0),
           (1, -34.999, 149.001, 16, 0, 0, 0),
           (2, -34.998, 149.002, 16, 0, 0, 0)]
params = {"ATC_STR_RAT_FF": 0.009, "ATC_STR_RAT_P": 0.6, "ATC_STR_RAT_I": 0.07,
          "ATC_STR_RAT_D": 0.05, "ATC_STR_RAT_MAX": 75, "ATC_STR_ACC_MAX": 120,
          "ATC_STR_ANG_P": 2, "ATC_SPEED_P": 0.2, "ATC_SPEED_I": 0.2,
          "CRUISE_SPEED": 2, "CRUISE_THROTTLE": 50, "WP_SPEED": 1}
d = {"hist": hist, "att": att, "hud": hud, "vel": vel, "gps": gps, "ekf": ekf,
     "s1": s1, "s3": s3, "params": params, "mode": 10, "pos": pos,
     "mcur": mcur, "mission": mission}

print("=" * 72)
print(" [D] 端到端: 巡航段(有效) —— 应出现正常的 ANG_P 判定")
print("=" * 72)
lines, rep = U.compute_diag(d, DUR, PID)
print("  生成 %d 行" % len(lines))
for ln in rep.splitlines():
    if any(k in ln for k in ("怠速", "无效段", "反算 ANG_P", "误差RMS", "误差均值",
                             "摆动频率", "实测", "依据", "合格窗口", "原因", "只有")):
        print("   | " + ln.strip())

# 场景2: 怠速段
d2 = dict(d)
d2["hud"] = [(t, 0.05, 0.0) for (t, _g, _th) in hud]
print()
print("=" * 72)
print(" [E] 端到端: 怠速段 —— 应打上「怠速/无效段」")
print("=" * 72)
lines2, rep2 = U.compute_diag(d2, DUR, PID)
for ln in rep2.splitlines()[:14]:
    print("   | " + ln)

# 场景3: 存盘路径
print()
print("=" * 72)
print(" [F] save_segment 存盘路径")
print("=" * 72)
tmp = tempfile.mkdtemp(prefix="pidchk_")
body, path, nnn = U.save_segment(10, 0.0, DUR, d, seg_dir=tmp)
print("  写出: %s  (#%d, %d 字节)" % (os.path.basename(path),
                                      nnn, os.path.getsize(path)))
print("  汇总.txt: %s" % ("有" if os.path.exists(
    os.path.join(tmp, "汇总.txt")) else "缺失!"))
print("  汇总内容:")
for ln in open(os.path.join(tmp, "汇总.txt"), encoding="utf-8").read().splitlines():
    print("   | " + ln[:100])
