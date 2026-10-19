# -*- coding: utf-8 -*-
"""验证 [13] 过弯事件分析的判据:
   场景A 每次过弯后都回到同一基线  -> 应判\"不累积\"
   场景B 基线逐次抬高              -> 应判\"累积性劣化\"
"""
import sys, os, types, math, time

exec(open("_bench_diag.py", encoding="utf-8").read().split('print("=" * 84)')[0])
import usv_debug as U

LAT0, LON0 = -35.0, 149.0
LEG_M = 200.0                      # 航段长度(m), 走东向
LEG_DEG = LEG_M / 111320.0
NPTS = 6
LEG_T = 40.0                       # 每航段 40 秒
SPEED = LEG_M / LEG_T              # m/s


def build(offsets, sec_per_leg=LEG_T, total=None):
    """offsets[k] = 第 k 个航段上的横向偏移(m, 正=北)"""
    total = total or (len(offsets) * sec_per_leg)
    d = mk_bundle(total)                 # 先建, 用它自己的时间基准
    t0 = d["hist"][U.PID_AXIS][0][0]
    mission = [(k, LAT0, LON0 + k * LEG_DEG, 16, 0, 0, 0) for k in range(NPTS)]
    pos, mcur = [], []
    for i in range(int(total * 3)):
        t = t0 + i / 3.0
        leg = min(len(offsets) - 1, int((t - t0) / sec_per_leg))
        frac = ((t - t0) / sec_per_leg) - leg
        off = offsets[leg]
        peak = off * 3.0                 # 过弯瞬间冲高, 之后指数收敛回该段基线
        conv = peak + (off - peak) * (1.0 - math.exp(-frac * sec_per_leg / 8.0))
        pos.append((t, LAT0 + conv / 110540.0,
                    LON0 + (leg * LEG_M + frac * LEG_M) / 111320.0,
                    0.0, SPEED * 100, 0.0, 0.0))
        mcur.append((t, leg + 1))
    d.update({"pos": pos, "mcur": mcur, "mission": mission})
    return d


def show(label, d, sec):
    _l, rep = U.compute_diag(d, sec, U.PID_AXIS)
    print("=" * 74)
    print(" " + label)
    print("=" * 74)
    hit = False
    for ln in rep.splitlines():
        if "[13]" in ln:
            hit = True
        if hit:
            print("  | " + ln)
        if hit and ("基线" in ln and ("稳定" in ln or "抬高" in ln or "下降" in ln)):
            break
    print()


# A) 每段基线都是 0.5 m, 弯瞬间冲到 1.5 m 然后收敛回 0.5
show("场景A  每段基线相同 (0.5 m)", build([0.5] * 5), 200)

# B) 每段基线逐次抬高 0.5 -> 2.5 m
show("场景B  基线逐次抬高 (0.5 -> 2.5 m)", build([0.5, 1.0, 1.5, 2.0, 2.5]), 200)
