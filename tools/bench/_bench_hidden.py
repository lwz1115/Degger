# -*- coding: utf-8 -*-
"""验证: 未显示的 ZoomPlot 会不会堆积 after 定时器链"""
import sys, os, time
sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
import tkinter as tk
import usv_debug as U

CNT = {"n": 0}


class Probe(U.ZoomPlot):
    """统计 redraw 被调用的次数"""
    def redraw(self):
        CNT["n"] += 1
        return U.ZoomPlot.redraw(self)


root = tk.Tk()
root.geometry("500x300")
root.withdraw()


def mk(pack):
    p = Probe(root, height=90)
    if pack:
        p.pack(fill="x")
    return p


def series(npts=200):
    t = time.time()
    return [("a", [(i / 10.0, (i % 50)) for i in range(npts)])]


def run(label, p, polls=30, interval=0.02):
    """模拟 30 次轮询(每次 20ms), 然后在最后 1 秒里数 redraw 被触发多少次"""
    root.update()
    CNT["n"] = 0
    for _ in range(polls):
        p.set_series(series())
        root.update()
        time.sleep(interval)
    # 停掉轮询, 只让 after 链自己跑
    t0 = time.time()
    n_before = CNT["n"]
    while time.time() - t0 < 1.0:
        root.update()
        time.sleep(0.005)
    fired = CNT["n"] - n_before
    print("   %-44s 轮询停止后 1s 内 redraw 被触发 %4d 次" % (label, fired))
    return fired


print("=" * 84)
print(" 复现: ZoomPlot 未显示时, 每次 set_series 是否都堆一条 after 链")
print("=" * 84)

a = run("① 已 pack 正常显示 (基线)", mk(True))
b = run("② 未 pack (模拟切到别的页签)", mk(False), polls=30)

print()
print("=" * 84)
print(" 放大: 轮询次数越多, 停止后残留的定时器链越多 (正是\"越用越卡\")")
print("=" * 84)
for npoll in (10, 30, 60, 120):
    p = mk(False)
    root.update()
    CNT["n"] = 0
    for _ in range(npoll):
        p.set_series(series())
        root.update()
        time.sleep(0.01)
    t0 = time.time()
    nb = CNT["n"]
    while time.time() - t0 < 1.0:
        root.update()
        time.sleep(0.005)
    print("   轮询 %3d 次 -> 停后 1s 内仍触发 %4d 次 redraw  (每链 60ms = 16.7/s)"
          % (npoll, CNT["n"] - nb))
    p.destroy()

print()
print("=" * 84)
print(" 结论: ① 与 ② 的差值就是泄漏量")
print("=" * 84)
print("   显示中: %d 次/s      未显示: %d 次/s      泄漏倍数: %.1fx"
      % (a, b, b / max(a, 1)))
root.destroy()
