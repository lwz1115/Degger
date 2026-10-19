# -*- coding: utf-8 -*-
"""功能回归: 修复后, 图在"显示出来"的时候必须能正常画出来"""
import sys, os, time
sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
import tkinter as tk
import usv_debug as U

root = tk.Tk()
root.geometry("520x360+3200+3200")   # 挪到屏幕外, 避免打扰
root.update()

p = U.ZoomPlot(root, height=120)
root.update()
print("1) 尚未 pack (未 layout):  画布项数 = %d" % len(p.find_all()))

data = [("a", [(i / 10.0, 100 * ((i % 37) - 18)) for i in range(200)]),
        ("b", [(i / 10.0, -100 * ((i % 29) - 14)) for i in range(200)])]
p.set_series(data, ylabel="PWM")
for _ in range(10):
    root.update(); time.sleep(0.02)
print("2) 未 pack 时反复 set_series: 画布项数 = %d  (应为 0, 不该空转)" % len(p.find_all()))

p.pack(fill="x")
for _ in range(30):
    root.update(); time.sleep(0.02)
n_after = len(p.find_all())
print("3) pack 显示后:              画布项数 = %d  %s"
      % (n_after, "OK 正常绘制" if n_after > 5 else "!! 没画出来"))

# 再喂一次新数据, 确认显示状态下能持续更新
p.set_series([("a", [(i / 10.0, 200 * ((i % 41) - 20)) for i in range(200)])])
for _ in range(10):
    root.update(); time.sleep(0.02)
n2 = len(p.find_all())
print("4) 显示状态下换新数据:        画布项数 = %d  %s"
      % (n2, "OK" if n2 > 5 else "!! 更新失败"))

# 缩放也要能用
p.set_zoom(2.0, 8.0)
for _ in range(10):
    root.update(); time.sleep(0.02)
print("5) 缩放后:                    画布项数 = %d  view=%s"
      % (len(p.find_all()), None if p._view is None else
         "x %.2f~%.2f" % (p._view[0], p._view[1])))

ok = (len(p.find_all()) > 5 and p._view is not None)
print()
print("结论: %s" % ("通过" if ok else "不通过"))
root.destroy()
