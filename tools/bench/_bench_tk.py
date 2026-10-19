# -*- coding: utf-8 -*-
"""用真实 Tk 实测控件层开销: Treeview 行管理与 Canvas 折线重绘"""
import time
import tkinter as tk
from tkinter import ttk

root = tk.Tk()
root.withdraw()          # 不弹窗

COLS = ("a", "b", "c", "d", "e", "f", "g", "h")
ROWS = 12


def mk_tree():
    t = ttk.Treeview(root, columns=COLS, show="headings", height=7)
    for c in COLS:
        t.heading(c, text=c)
        t.column(c, width=80, anchor="center")
    return t


vals = tuple("%.3f" % (i * 1.5) for i in range(len(COLS)))


def bench(label, fn, n):
    t0 = time.perf_counter()
    fn(n)
    dt = (time.perf_counter() - t0) * 1000.0
    print("   %-46s %8.1f ms  (%.3f ms/行)" % (label, dt, dt / n))
    return dt


print("=" * 84)
print(" [A] Treeview 插入 + 裁剪: 旧做法 vs 新做法")
print("=" * 84)

CAP = 3000


def old_style(n):
    """旧: 每插一行 get_children() 取全部 id, 满 3000 就删第一行"""
    t = mk_tree()
    for i in range(n):
        t.insert("", "end", values=vals)
        kids = t.get_children()          # <- 每次构造 3000 个 id
        if len(kids) > CAP:
            t.delete(kids[0])
    t.destroy()


def new_style(n):
    """新: 自己用 deque 记 id, 只删最老的"""
    import collections
    t = mk_tree()
    ids = collections.deque()
    for i in range(n):
        ids.append(t.insert("", "end", values=vals))
        while len(ids) > CAP:
            t.delete(ids.popleft())
    t.destroy()


def new_style_1000(n):
    """新 + 行数上限降到 1000"""
    import collections
    t = mk_tree()
    ids = collections.deque()
    for i in range(n):
        ids.append(t.insert("", "end", values=vals))
        while len(ids) > 1000:
            t.delete(ids.popleft())
    t.destroy()


N = 2000            # 约 3.3 分钟 @10Hz 的插入量
print("   插入 %d 行(表满后会持续删最老一行):" % N)
a = bench("A1 旧做法 (每行 get_children, 上限3000)", old_style, N)
b = bench("A2 新做法 (deque, 上限3000)", new_style, N)
c = bench("A3 新做法 + 上限压到 1000", new_style_1000, N)

print()
print("=" * 84)
print(" [B] Canvas create_line: 点数对重绘耗时的影响")
print("=" * 84)
cv = tk.Canvas(root, width=700, height=120)


def draw(npts, reps):
    for _ in range(reps):
        cv.delete("all")
        coords = []
        for i in range(npts):
            coords.append(i % 700)
            coords.append(60 + 50 * ((i * 7) % 100) / 100.0)
        cv.create_line(*coords, fill="#1f77b4")


for npts in (200, 600, 1200, 4000, 12000):
    reps = max(5, 20000 // npts)
    t0 = time.perf_counter()
    draw(npts, reps)
    dt = (time.perf_counter() - t0) / reps * 1000.0
    print("   %6d 点/条  ->  %8.3f ms/次重绘" % (npts, dt))

root.destroy()
print()
print("=" * 84)
print(" 说明: A1/A2 的差值就是每插入一行的额外开销 x 插入次数")
print("=" * 84)
