# -*- coding: utf-8 -*-
"""量化性能优化效果 + 正确性验证
对照: 优化前的 "全量复制 + 全量扫描" vs 优化后的 "二分切片"
"""
import sys, os, time, math, random, types

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

random.seed(1)
HZ = 10.0
WINDOW = U.WINDOW_SEC


def mk(key, sec, hz):
    n = int(sec * hz)
    t = time.time()
    return [(t + i / hz, random.uniform(1100, 1900)) for i in range(n)]


def old_redraw(buf):
    """优化前的 _redraw 核心"""
    snap = {k: list(v) for k, v in buf.items()}
    now = 0.0
    for v in snap.values():
        if v and v[-1][0] > now:
            now = v[-1][0]
    tm = now - WINDOW
    out = []
    for k in ("rc1", "rc3", "s1", "s3"):
        out.append([(t - tm, v) for t, v in snap[k] if t >= tm])
    return out


def new_redraw(buf):
    """优化后的 _redraw 核心"""
    now = 0.0
    for v in buf.values():
        if v and v[-1][0] > now:
            now = v[-1][0]
    tm = now - WINDOW
    return [U._tail_from(buf[k], tm) for k in ("rc1", "rc3", "s1", "s3")]


print("=" * 78)
print(" [1] _tail_from 正确性 (对照全量扫描)")
print("=" * 78)
ok = True
for sec in (1, 20, 120, 600):
    b = mk("x", sec, HZ)
    for frac in (0.0, 0.25, 0.5, 0.9, 1.0):
        tf = b[0][0] + (b[-1][0] - b[0][0]) * frac
        ref = [r for r in b if r[0] >= tf]
        got = U._tail_from(b, tf)
        if ref != got:
            ok = False
            print("  !! 不匹配 sec=%s frac=%s  ref=%d got=%d" % (sec, frac, len(ref), len(got)))
print("  各种长度/阈值下与线性扫描完全一致: %s" % ("是" if ok else "否"))
print("  空序列: %r   尾部越界: %d 条" % (U._tail_from([], 0.0), len(U._tail_from(mk("x", 5, 10), 1e18))))

print()
print("=" * 78)
print(" [2] _decimate 保包络验证")
print("=" * 78)
for n in (100, 500, 2000, 20000):
    seg = [(i * 0.01, math.sin(i * 0.3) + (5.0 if i == n // 2 else 0.0)) for i in range(n)]
    d = U._decimate(seg)
    ymin_a = min(y for _x, y in seg)
    ymax_a = max(y for _x, y in seg)
    ymin_b = min(y for _x, y in d)
    ymax_b = max(y for _x, y in d)
    keep = 100.0 * len(d) / n
    print("   n=%-6d -> %-5d 点 (%.1f%%)   min %.3f/%.3f  max %.3f/%.3f  包络%s"
          % (n, len(d), keep, ymin_a, ymin_b, ymax_a, ymax_b,
             "保持" if (abs(ymin_a - ymin_b) < 1e-9 and abs(ymax_a - ymax_b) < 1e-9)
             else "丢失!"))

print()
print("=" * 78)
print(" [3] 单次重绘开销: 优化前 vs 优化后  (ms)")
print("=" * 78)
print("   缓冲时长   rc/s 数   优化前    优化后   加速")
for sec in (5, 30, 60, 120, 300, 600):
    buf = {"rc1": mk("rc1", sec, 2.0), "rc3": mk("rc3", sec, 2.0),
           "s1": mk("s1", sec, 10.0), "s3": mk("s3", sec, 10.0)}
    n_tot = sum(len(v) for v in buf.values())
    REP = 40
    t0 = time.perf_counter()
    for _ in range(REP):
        old_redraw(buf)
    t_old = (time.perf_counter() - t0) / REP * 1000
    t0 = time.perf_counter()
    for _ in range(REP):
        new_redraw(buf)
    t_new = (time.perf_counter() - t0) / REP * 1000
    print("   %6ds   %6d   %7.3f  %8.4f   %5.1fx"
          % (sec, n_tot, t_old, t_new, t_old / max(t_new, 1e-9)))

print()
print("=" * 78)
print(" [4] 稳态下的 GUI 负载估算")
print("=" * 78)
buf = {"rc1": mk("rc1", 600, 2.0), "rc3": mk("rc3", 600, 2.0),
       "s1": mk("s1", 600, 10.0), "s3": mk("s3", 600, 10.0)}
REP = 40
t0 = time.perf_counter()
for _ in range(REP):
    old_redraw(buf)
t_old = (time.perf_counter() - t0) / REP * 1000
t0 = time.perf_counter()
for _ in range(REP):
    new_redraw(buf)
t_new = (time.perf_counter() - t0) / REP * 1000
print("   缓冲区满(600s)时:")
print("     优化前: 每条报文都重画 -> ~12 次/s x %.2f ms = %6.1f ms/s CPU (%.1f%% 单核)"
      % (t_old, t_old * 12, t_old * 12 / 10))
print("     优化后: 节流到 5 次/s   ->   5 次/s x %.3f ms = %6.2f ms/s CPU (%.2f%% 单核)"
      % (t_new, t_new * 5, t_new * 5 / 10))
print("     UI 负载降到原来的 %.1f%%" % (t_new * 5 / max(t_old * 12, 1e-9) * 100))

print()
print("=" * 78)
print(" [5] PidView 单次重绘开销 (全量历史 vs 时间窗切片)")
print("=" * 78)
FIELDS = 6


def old_pid(hist):
    hist = list(hist)
    now = hist[-1][0]
    tm = now - WINDOW
    out = []
    for idx in range(FIELDS):
        out.append([(r[0] - tm, r[1 + idx]) for r in hist if r[0] >= tm])
    return out


def new_pid(hist):
    h = U._tail_from(hist, hist[-1][0] - WINDOW)
    tm = hist[-1][0] - WINDOW
    return [[(r[0] - tm, r[1 + idx]) for r in h] for idx in range(FIELDS)]


for sec in (60, 300, 600):
    hist = [(time.time() + i / 10.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
            for i in range(int(sec * 10))]
    t0 = time.perf_counter()
    for _ in range(REP):
        old_pid(hist)
    a = (time.perf_counter() - t0) / REP * 1000
    t0 = time.perf_counter()
    for _ in range(REP):
        new_pid(hist)
    b = (time.perf_counter() - t0) / REP * 1000
    print("   历史 %4ds (%5d 条):  优化前 %6.3f ms   优化后 %6.4f ms   加速 %5.1fx"
          % (sec, len(hist), a, b, a / max(b, 1e-9)))
