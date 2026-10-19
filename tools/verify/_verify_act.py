# -*- coding: utf-8 -*-
"""用真实 CSV 构造 bundle 跑 compute_diag, 交叉验证 [6] 执行器指标
对照脚本: _chk_actuator.py
"""
import sys, os, csv, math, types

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

CSV = sys.argv[1]
T0 = float(sys.argv[2])       # 段起点(相对 CSV 起点, 秒)
T1 = float(sys.argv[3])       # 段终点

PID, SPD = U.PID_AXIS, U.SPEED_AXIS
hist = {PID: [], SPD: []}
att, hud, s1, s3 = [], [], [], []


def f(x, k):
    try:
        return float(x[k])
    except Exception:
        return None


with open(CSV, encoding="utf-8", errors="ignore") as fh:
    for r in csv.DictReader(fh):
        t = f(r, "t")
        if t is None or not (T0 <= t <= T1):
            continue
        for k in ("des", "ach", "P", "I", "D", "FF"):
            if f(r, k) is None:
                break
        else:
            hist[PID].append((t, f(r, "des"), f(r, "ach"),
                              f(r, "P"), f(r, "I"), f(r, "D"), f(r, "FF")))
            att.append((t, math.radians(f(r, "ach") or 0.0), 0.0, 0.0, 0.0))
        if all(f(r, k) is not None for k in ("sp_des", "sp_ach", "sp_P",
                                             "sp_I", "sp_D", "sp_FF")):
            hist[SPD].append((t, f(r, "sp_des"), f(r, "sp_ach"), f(r, "sp_P"),
                              f(r, "sp_I"), f(r, "sp_D"), f(r, "sp_FF")))
        if f(r, "gs") is not None and f(r, "thr") is not None:
            hud.append((t, f(r, "gs"), f(r, "thr")))
        if f(r, "s1") is not None:
            s1.append((t, f(r, "s1")))
        if f(r, "s3") is not None:
            s3.append((t, f(r, "s3")))

print("段 t=%.1f~%.1f  样本: PID=%d  SPD=%d  HUD=%d  S1=%d  S3=%d"
      % (T0, T1, len(hist[PID]), len(hist[SPD]), len(hud), len(s1), len(s3)))

params = {"ATC_STR_RAT_FF": 0.3, "ATC_STR_RAT_P": 0.6, "ATC_STR_RAT_I": 0.07,
          "ATC_STR_RAT_D": 0.05, "ATC_STR_RAT_MAX": 75, "ATC_STR_ACC_MAX": 120,
          "ATC_STR_ANG_P": 2, "ATC_SPEED_P": 0.2, "ATC_SPEED_I": 0.2,
          "CRUISE_SPEED": 2, "CRUISE_THROTTLE": 50, "WP_SPEED": 1}
d = {"hist": hist, "att": att, "hud": hud, "vel": [], "gps": [], "ekf": [],
     "s1": s1, "s3": s3, "params": params, "mode": 10, "pos": [], "mcur": [],
     "mission": []}

lines, rep = U.compute_diag(d, T1 - T0, PID)
print()
got = False
for ln in rep.splitlines():
    if "[6]" in ln:
        got = True
    if got:
        print(" | " + ln)
        if "贴 MIN" in ln:
            break
