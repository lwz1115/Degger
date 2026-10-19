# -*- coding: utf-8 -*-
"""测量报告生成(compute_diag / save_segment / _close_segment 快照)的耗时。
这几处跑在持锁的读取线程里, 一旦变慢界面就会卡住。"""
import sys, os, time, math, random, types, tempfile

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

random.seed(3)


def mk_bundle(sec, mcur_hz=2.0, mission_pts=6):
    n_pid = int(sec * 10)
    n_pos = int(sec * 3)
    t0 = time.time()
    PID, SPD = U.PID_AXIS, U.SPEED_AXIS
    hist = {PID: [], SPD: []}
    att, hud, vel, gps, ekf, s1, s3, pos, mcur = [], [], [], [], [], [], [], [], []
    for i in range(n_pid):
        t = t0 + i / 10.0
        he = 30.0 * math.sin(2 * math.pi * 0.06 * (i / 10.0))
        des = max(-75.0, min(75.0, 2.0 * he))
        v = 25.0 * math.sin(i * 0.31) + random.gauss(0, 3)
        hist[PID].append((t, v, v * 0.35, 0.2, 0.1, 0.01, 0.06))
        hist[SPD].append((t, 0.8, 0.81, 0.02, 0.12, 0.0, 0.19))
        att.append((t, math.radians(v), 0.0, 0.0, math.radians(i % 360)))
        hud.append((t, 0.9, 35.0))
        s1.append((t, 1500 + 400 * math.sin(i * 0.2)))
        s3.append((t, 1500 - 400 * math.sin(i * 0.2)))
    for i in range(n_pos):
        t = t0 + i / 3.0
        vel.append((t, 0.9, 0.03))
        gps.append((t, 3, 28, 60.0))
        ekf.append((t, 0x33F, 0.004, 0.01))
        pos.append((t, -35.0 + i * 1e-5, 149.0 + i * 1e-5, 0.0, 90.0, 3.0, 10.0))
    for i in range(int(sec * mcur_hz)):
        mcur.append((t0 + i / mcur_hz, i % max(1, mission_pts - 1) + 1))
    mission = [(k, -35.0 + k * 1e-5, 149.0 + k * 1e-5, 16, 0, 0, 0)
               for k in range(mission_pts)]
    params = {"ATC_STR_RAT_FF": 0.2, "ATC_STR_RAT_P": 0.6, "ATC_STR_RAT_I": 0.07,
              "ATC_STR_RAT_D": 0.05, "ATC_STR_RAT_MAX": 75, "ATC_STR_ACC_MAX": 120,
              "ATC_STR_ANG_P": 2, "ATC_SPEED_P": 0.2, "ATC_SPEED_I": 0.2,
              "CRUISE_SPEED": 2, "CRUISE_THROTTLE": 50, "WP_SPEED": 1}
    return {"hist": hist, "att": att, "hud": hud, "vel": vel, "gps": gps,
            "ekf": ekf, "s1": s1, "s3": s3, "params": params, "mode": 10,
            "pos": pos, "mcur": mcur, "mission": mission}


print("=" * 84)
print(" [1] compute_diag 单轴耗时 (报告生成, 跑在持锁的读取线程里)")
print("=" * 84)
print("   段时长   PID样本  mcur条   转向环     速度环     两轴合计")
for sec in (30, 90, 240):
    d = mk_bundle(sec)
    t0 = time.perf_counter()
    U.compute_diag(d, sec, U.PID_AXIS)
    t1 = time.perf_counter()
    U.compute_diag(d, sec, U.SPEED_AXIS)
    t2 = time.perf_counter()
    print("   %5ds   %6d  %6d   %7.1f ms  %7.1f ms  %8.1f ms"
          % (sec, len(d["hist"][U.PID_AXIS]), len(d["mcur"]),
             (t1 - t0) * 1000, (t2 - t1) * 1000, (t2 - t0) * 1000))

print()
print("=" * 84)
print(" [2] _nav_block 里 seq_at() 的线性扫描代价 (mcur 越长越慢)")
print("=" * 84)
print("   段长240s, 改变 MISSION_CURRENT 的密度:")
for hz in (0.5, 2.0, 5.0, 10.0):
    d = mk_bundle(240, mcur_hz=hz)
    t0 = time.perf_counter()
    U.compute_diag(d, 240, U.PID_AXIS)
    dt = (time.perf_counter() - t0) * 1000
    print("   mcur %4.1f Hz (%5d 条) -> %7.1f ms" % (hz, len(d["mcur"]), dt))

print()
print("=" * 84)
print(" [3] save_segment 总耗时 (含两次 compute_diag + 写文件)")
print("=" * 84)
tmp = tempfile.mkdtemp(prefix="bench_seg_")
for sec in (30, 240):
    d = mk_bundle(sec)
    t0 = time.perf_counter()
    U.save_segment(10, time.time(), time.time() + sec, d, seg_dir=tmp)
    dt = (time.perf_counter() - t0) * 1000
    print("   段 %3ds -> %7.1f ms" % (sec, dt))

print()
print("=" * 84)
print(" [4] _close_segment 的快照深拷贝耗时 (也在锁内)")
print("=" * 84)
for sec in (30, 240, 600):
    d = mk_bundle(sec)
    t0 = time.perf_counter()
    snap = {k: (list(v) if isinstance(v, list) else dict(v))
            for k, v in d.items() if isinstance(v, (list, dict))}
    dt = (time.perf_counter() - t0) * 1000
    tot = sum(len(v) for v in snap.values() if isinstance(v, list))
    print("   缓冲 %3ds (%6d 元组) -> 深拷贝 %6.2f ms" % (sec, tot, dt))
