# -*- coding: utf-8 -*-
"""验证「判读建议」在各种工况下给出的方向是否正确"""
import sys, os, math, types

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

HZ, DUR = 10.0, 60.0
N = int(HZ * DUR)
PID = U.PID_AXIS


def bundle(des_amp, sat_frac, ff, p, i, d, rat_max=75.0, spd_ok=True):
    """sat_frac: 期望中有多少比例的样本被顶到限幅"""
    hist = {PID: [], U.SPEED_AXIS: []}
    att, hud, vel, gps, ekf, s1, s3 = [], [], [], [], [], [], []
    pos, mcur = [], []
    for k in range(N):
        t = k / HZ
        w = math.sin(2 * math.pi * 0.1 * t)          # 对称振荡 -> bias≈0
        des = des_amp * w
        if (k % 100) < int(100 * sat_frac):          # 人为制造限幅样本
            des = math.copysign(rat_max, des or 1.0)
        ach = des * 0.5                              # 跟不上 -> 误差大
        hist[PID].append((t, des, ach, p, i, d, ff))
        hist[U.SPEED_AXIS].append((t, 0.8, 0.8, 0.02, 0.12, 0.0, 0.19))
        att.append((t, 0.0, 0.0, 0.0, 0.0))
        hud.append((t, 0.85, 33.0))
        vel.append((t, 0.85, 0.0))
        gps.append((t, 3, 28, 60.0))
        ekf.append((t, 0x33F, 0.004, 0.01))
        s1.append((t, 1500.0))
        s3.append((t, 1500.0))
        mcur.append((t, 2))
        pos.append((t, -35.0, 149.0, 0, 85.0, 0.0, 0.0))
    params = {"ATC_STR_RAT_FF": ff, "ATC_STR_RAT_P": p, "ATC_STR_RAT_I": i,
              "ATC_STR_RAT_D": d, "ATC_STR_RAT_MAX": rat_max,
              "ATC_STR_ACC_MAX": 120, "ATC_STR_ANG_P": 2,
              "ATC_SPEED_P": 0.2, "ATC_SPEED_I": 0.2, "CRUISE_SPEED": 2,
              "CRUISE_THROTTLE": 50, "WP_SPEED": 1}
    return {"hist": hist, "att": att, "hud": hud, "vel": vel, "gps": gps,
            "ekf": ekf, "s1": s1, "s3": s3, "params": params, "mode": 10,
            "pos": pos, "mcur": mcur,
            "mission": [(0, -35.0, 149.0, 16, 0, 0, 0),
                        (1, -34.999, 149.001, 16, 0, 0, 0),
                        (2, -34.998, 149.002, 16, 0, 0, 0)]}


CASES = [
    ("A 期望顶满 RAT_MAX + FF 占比低  (= 今天 #003/#006 的样子)",
     dict(des_amp=70, sat_frac=0.30, ff=0.009, p=0.6, i=0.07, d=0.05),
     ["权限不足", "不要降"], ["增益偏高", "优先降"]),
    ("B 期望不饱和 + FF 占比低  (= 前馈不足的样子)",
     dict(des_amp=30, sat_frac=0.0, ff=0.009, p=0.6, i=0.07, d=0.01),
     ["优先提高 ATC_STR_RAT_FF"], ["优先降 ATC_STR_RAT_P"]),
    ("C 期望不饱和 + FF 占比高  (= 真的增益偏高)",
     dict(des_amp=30, sat_frac=0.0, ff=0.5, p=0.1, i=0.05, d=0.01),
     ["增益/相位偏高"], ["权限不足", "优先提高 ATC_STR_RAT_FF"]),
]

ok_all = True
for name, kw, must, mustnot in CASES:
    d = bundle(**kw)
    lines, rep = U.compute_diag(d, DUR, PID)
    print("=" * 74)
    print(" " + name)
    print("=" * 74)
    for ln in rep.splitlines():
        if any(k in ln for k in ("期望强度", "相对误差", "限幅饱和", "跟踪误差",
                                 "期望转向率")):
            print("   | " + ln.strip())
    tips = [ln.strip() for ln in rep.splitlines()
            if ln.strip()[:2] in tuple("%d." % i for i in range(1, 9))]
    print("   --- 判读建议 ---")
    for tp in tips:
        print("   | " + tp)
    blob = "\n".join(tips)
    hit = [m for m in must if m in blob]
    bad = [m for m in mustnot if m in blob]
    ok = (len(hit) == len(must)) and not bad
    ok_all = ok_all and ok
    print("   期望出现: %s  -> %s" % (must, "找到" if hit else "!!! 缺失"))
    print("   不该出现: %s  -> %s" % (mustnot, "干净" if not bad else "!!! 出现了"))
    print("   结论: %s" % ("通过" if ok else "不通过"))
    print()

print("=" * 74)
print(" 总结果: %s" % ("全部通过" if ok_all else "存在不通过项"))
print("=" * 74)
