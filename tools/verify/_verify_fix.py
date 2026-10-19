# -*- coding: utf-8 -*-
"""验证 usv_debug.py 的三处改动"""
import re, sys, os, math, random, glob, types

# WSL 里没有 tkinter, 打桩以便只测试纯计算函数(不启动 GUI)
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

print("=" * 72)
print(" [A] _angp_regress 合成数据测试")
print("=" * 72)


def synth(kind, k_true=2.0, sec=90.0, hz=10.0, seed=1):
    random.seed(seed)
    n = int(sec * hz)
    rows, pid = [], []
    ph = 0.0
    for i in range(n):
        t = i / hz
        if kind == "pivot":
            # 原地转向: 航向误差慢变, 期望转向率严格 = k * 误差 (带一点噪声)
            ph += random.gauss(0, 0.35)
            ph = max(-40.0, min(40.0, ph))
            he = ph
            des = k_true * he + random.gauss(0, 0.4)
        else:
            # 航线巡航: 期望转向率由位置环给出, 与航向误差无关
            he = 25.0 * math.sin(2 * math.pi * 0.08 * t) + random.gauss(0, 3)
            des = 30.0 * math.sin(2 * math.pi * 0.05 * t + 1.3) + random.gauss(0, 2)
        if abs(des) > 0.92 * 75:
            des = math.copysign(0.92 * 75, des)
        rows.append((t, he, 0.0, 0.0, 10.0, 1))
        pid.append((t, des, 0.0, 0.0, 0.0, 0.0, 0.0))
    return rows, pid


for kind, expect in (("pivot", "应反算出 ~2.0"), ("cruise", "应为 None 或极少合格窗")):
    rows, pid = synth(kind)
    r = U._angp_regress(rows, pid, 75.0)
    print("  数据=%s  ->  %s" % (kind, r))
    print("    期望: %s" % expect)
print()

print("=" * 72)
print(" [B] 怠速判据 用今天 6 段真实数据回放")
print("=" * 72)
d = "/mnt/e/WSL2/PID_Check"
files = sorted(glob.glob(os.path.join(d, "20260910_00*_AUTO_*.txt")))
print("   阈值: 油门 < %.1f%% 或 地速 < %.2f m/s  =>  判为无效段"
      % (U.IDLE_THR_PCT, U.IDLE_GS_MPS))
print()
print("   段  油门mean 地速mean   判定")
for fn in files:
    txt = open(fn, encoding="utf-8", errors="replace").read()
    # 只看转向环那半段, 避免重复
    half = txt.split("速度环 ATC_SPEED")[0]

    def g(p):
        m = re.search(p, half)
        return float(m.group(1)) if m else None
    thr = g(r"油门 mean=([\d.]+)")
    gs = g(r"地速 mean=([\d.]+)")
    seg = re.search(r"#(\d+)", txt).group(1)
    idle = (thr is not None and gs is not None) and \
           (thr < U.IDLE_THR_PCT or gs < U.IDLE_GS_MPS)
    mark = "  <<< 无效段" if idle else "  有效"
    print("   %-3s %8s %8s  %s" % (seg, thr, gs, mark))

print()
print("=" * 72)
print(" [C] 振荡前置条件 用今天 6 段真实数据回放")
print("=" * 72)
print("   阈值: 过零频率 >= %.2f Hz 且 过零次数 >= %d 才算振荡"
      % (U.OSC_MIN_HZ, U.OSC_MIN_ZC))
print()
print("   段  摆动频率  过零次数   旧判定        新判定")
for fn in files:
    txt = open(fn, encoding="utf-8", errors="replace").read()
    half = txt.split("速度环 ATC_SPEED")[0]
    seg = re.search(r"#(\d+)", txt).group(1)
    m = re.search(r"误差摆动频率 = ([\d.]+) Hz\s+\((\d+) 次过零\)", half)
    f = float(m.group(1)) if m else None
    z = int(m.group(2)) if m else 0
    old = re.search(r"误差RMS 前半=[\d.]+ 后半=[\d.]+ -> (\S+)", half)
    osc = (f is not None) and (f >= U.OSC_MIN_HZ) and (z >= U.OSC_MIN_ZC)
    new = ("正常发散判定" if osc else "未检测到振荡, 不做发散判定")
    print("   %-3s %9s %9s   %-12s  %s" % (seg, f, z,
                                           old.group(1) if old else "-", new))
