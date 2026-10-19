#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G12 实时监视（TCP MAVLink 直连）
=================================
直接连 G12(云卓遥控器) rcservice 桥出的 TCP 5760，实时拿飞控数据。
页面结构：
  「实时监视」页 = 连接条(共用) + 子页切换:
      · 实时监视  : RC 输入 / S1/S3 输出滚动波形 + 打杆自动测 OUT 爬升
      · PID 调试  : 5 路波形(期望/实际/P/I/D)实时滚动, 可滚轮缩放;
                    底部数据栏每行 5 个数值实时滚动, 右侧时间列, 滚轮回看历史
独立运行:  python g12_live.py
无界面自检: python g12_live.py --probe [ip:port]
"""
import os
import glob
import sys
import time
import queue
import threading
import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
try:
    from g12_debug_gui import (TkPlot, DEFAULT_CFG, load_cfg, measure_action,
                               save_cfg, load_param_snapshot,
                               adb_run, adb_devices, adb_connect, adb_pull,
                               adb_list_tlogs, G12_TLOG_REMOTE, discover_live_addr)
except Exception:
    TkPlot = None
    DEFAULT_CFG = {}
    adb_run = adb_devices = adb_connect = adb_pull = adb_list_tlogs = None
    discover_live_addr = None
    G12_TLOG_REMOTE = ""

APP_TITLE = "G12 实时监视"
WINDOW_SEC = 20.0            # 波形滚动窗口(秒)
MAX_AGE = 300.0              # 数据缓冲保留(秒)
PID_AXIS = 5                 # Steering 转向 PID
AXIS_NAME = {0: "None", 1: "Roll", 2: "Pitch", 3: "Yaw", 4: "Throttle/AccZ",
             5: "Steering", 6: "Landing", 7: "Aux"}
PID_FIELDS = ("desired", "achieved", "P", "I", "D")
PID_TITLES = {"desired": "期望 desired (deg/s)", "achieved": "实际 achieved (deg/s)",
              "P": "P 比例输出", "I": "I 积分输出", "D": "D 微分输出"}
# ArduRover 模式号
MODE_NAMES = {0: "MANUAL 手动", 1: "ACRO", 3: "STEERING", 4: "HOLD",
              5: "LOITER", 6: "FOLLOW", 9: "CIRCLE", 10: "AUTO 自动",
              11: "RTL", 15: "GUIDED 引导", 16: "INITIALISING"}


# ---------------- 实时数据线程 ----------------
class LiveFeed(threading.Thread):
    """后台收 MAVLink：RC/输出缓冲 + PID_TUNING 历史 + 打杆事件测量。
    测量结果(动作 dict)放入 out_q。"""

    def __init__(self, addr, cfg, out_q):
        super().__init__(daemon=True)
        self.addr = addr
        self.cfg = cfg
        self.out_q = out_q
        self.stop_flag = False
        self.lock = threading.RLock()   # 可重入，处理中可再次加锁
        self.buf = {"rc1": [], "rc3": [], "s1": [], "s3": []}
        self.pid_hist = {}              # axis -> [(t, desired, achieved, P, I, D, FF)]
        self.error = None
        self.connected = False
        self.mode = None                # 最新 HEARTBEAT custom_mode
        self._active = {"rc1": False, "rc3": False}
        self._t0 = {"rc1": None, "rc3": None}
        self._sign = {"rc1": 1, "rc3": 1}
        self._pending = []              # (key, t0, t1, sign)

    def stop(self):
        self.stop_flag = True
        try:
            self.m.close()
        except Exception:
            pass

    def _trim(self, key):
        b = self.buf[key]
        if not b:
            return
        cutoff = b[-1][0] - MAX_AGE
        i = 0
        while i < len(b) and b[i][0] < cutoff:
            i += 1
        if i:
            del b[:i]

    def _update_state(self, key, t, v):
        th = float(self.cfg.get("action_thresh", 120))
        if v is None:
            return
        if not self._active[key]:
            if abs(v - 1500) > th:
                self._active[key] = True
                self._t0[key] = t
                self._sign[key] = 1 if v > 1500 else -1
        else:
            if abs(v - 1500) <= th * 0.5:
                self._active[key] = False
                t0 = self._t0[key]
                if t0 is not None and t - t0 > 0.05:
                    self._pending.append((key, t0, t, self._sign[key]))
                self._t0[key] = None

    def _flush_pending(self):
        if not self._pending or not self.buf["s1"]:
            return
        last_out = self.buf["s1"][-1][0]
        keep = []
        for p in self._pending:
            _key, t0, t1, _sign = p
            if last_out >= t1 + 0.8:
                try:
                    with self.lock:
                        rc = list(self.buf[p[0]])
                        a = list(self.buf["s1"])
                        b = list(self.buf["s3"])
                    it = measure_action(rc, a, b, t0, t1, p[3], self.cfg)
                except Exception:
                    it = None
                if it is not None:
                    it["stick"] = p[0]
                    self.out_q.put(it)
            else:
                keep.append(p)
        self._pending = keep

    def run(self):
        try:
            from pymavlink import mavutil
        except Exception as e:
            self.error = f"缺少 pymavlink: {e}"
            return
        try:
            self.m = mavutil.mavlink_connection("tcp:" + self.addr, timeout=6)
            self.connected = True
        except Exception as e:
            self.error = f"连接 {self.addr} 失败: {e}"
            return
        while not self.stop_flag:
            try:
                msg = self.m.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                if not self.stop_flag:
                    self.error = f"连接中断: {e}"
                break
            if msg is None:
                continue
            t = getattr(msg, "_timestamp", 0.0)
            ty = msg.get_type()
            with self.lock:
                if ty == "RC_CHANNELS":
                    for key, v in (("rc1", msg.chan1_raw), ("rc3", msg.chan3_raw)):
                        if v is not None:
                            self.buf[key].append((t, v))
                    self._update_state("rc1", t, msg.chan1_raw)
                    self._update_state("rc3", t, msg.chan3_raw)
                    self._trim("rc1")
                    self._trim("rc3")
                elif ty == "SERVO_OUTPUT_RAW":
                    for key, v in (("s1", msg.servo1_raw), ("s3", msg.servo3_raw)):
                        if v is not None:
                            self.buf[key].append((t, v))
                    self._trim("s1")
                    self._trim("s3")
                elif ty == "PID_TUNING":
                    axis = getattr(msg, "axis", -1)
                    rec = (t, getattr(msg, "desired", 0.0), getattr(msg, "achieved", 0.0),
                           getattr(msg, "P", 0.0), getattr(msg, "I", 0.0),
                           getattr(msg, "D", 0.0), getattr(msg, "FF", 0.0))
                    self.pid_hist.setdefault(axis, []).append(rec)
                    h = self.pid_hist[axis]
                    if h and t - h[0][0] > MAX_AGE:
                        cutoff = t - MAX_AGE
                        i = 0
                        while i < len(h) and h[i][0] < cutoff:
                            i += 1
                        if i:
                            del h[:i]
                elif ty == "HEARTBEAT":
                    self.mode = getattr(msg, "custom_mode", None)
                self._flush_pending()


# ---------------- 可缩放波形(带时间窗缩放) ----------------
class ZoomPlot(tk.Canvas):
    """极简折线图，支持共享时间窗缩放(以鼠标位置为中心)。x 用相对时间。"""

    def __init__(self, master, height=120, **kw):
        kw.setdefault("bg", "white")
        kw.setdefault("highlightthickness", 1)
        kw.setdefault("highlightbackground", "#ccc")
        super().__init__(master, height=height, **kw)
        self._series = []
        self._colors = ("#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd")
        self._ylabel = ""
        self._title = ""
        self._zoom = None            # (xlo, xhi) 相对时间；None=全部
        self._view = None            # 最近一次绘制的 (x0, x1, pad_l, pw)
        self._legend = ""

    def set_series(self, series, colors=None, ylabel="", legend=""):
        self._series = series
        if colors:
            self._colors = colors
        self._ylabel = ylabel
        self._legend = legend
        self.redraw()

    def set_title(self, title):
        self._title = title
        self.redraw()

    def set_zoom(self, lo, hi):
        self._zoom = (lo, hi)
        self.redraw()

    def reset_zoom(self):
        self._zoom = None
        self.redraw()

    def pixel_to_x(self, xpx):
        """像素 -> 相对时间(用最近一次绘图的坐标)"""
        if self._view is None:
            return 0.0
        x0, x1, pad_l, pw = self._view
        if pw <= 5:
            return x0
        f = (xpx - pad_l) / pw
        return x0 + (x1 - x0) * f

    def redraw(self):
        self.delete("all")
        W = max(self.winfo_width(), 60)
        H = max(self.winfo_height(), 40)
        pad_l, pad_r, pad_t, pad_b = 46, 10, 28, 24
        pw, ph = W - pad_l - pad_r, H - pad_t - pad_b
        if pw <= 5 or ph <= 5:
            self.after(50, self.redraw)
            return
        # 收集点
        allx = []
        alldata = []
        for _n, pts in self._series:
            for x, y in pts:
                allx.append(x)
                alldata.append((x, y))
        if not allx:
            self.create_text(W / 2, H / 2, text="(无数据)", fill="#999")
            self._view = None
            return
        amin, amax = min(allx), max(allx)
        if amin == amax:
            amax += 1
        if self._zoom:
            x0, x1 = self._zoom
        else:
            x0, x1 = amin, amax
        # 窗口内点
        ymin, ymax = None, None
        for x, y in alldata:
            if x0 <= x <= x1:
                ymin = y if ymin is None else min(ymin, y)
                ymax = y if ymax is None else max(ymax, y)
        if ymin is None:
            self.create_text(W / 2, H / 2, text="(窗口内无数据)", fill="#999")
            self._view = (x0, x1, pad_l, pw)
            return
        if ymin == ymax:
            ymax = ymin + 1
        pad = (ymax - ymin) * 0.08
        y0, y1 = ymin - pad, ymax + pad
        self._view = (x0, x1, pad_l, pw)

        def sx(x):
            return pad_l + (x - x0) / (x1 - x0) * pw

        def sy(y):
            return pad_t + (y1 - y) / (y1 - y0) * ph

        # 零线(1500 或 0)
        zl = 1500 if y0 < 1500 < y1 else 0.0
        if y0 < zl < y1:
            self.create_line(sx(x0), sy(zl), sx(x1), sy(zl), fill="#bbb", dash=(2, 2))
        # 轴
        self.create_line(pad_l, pad_t, pad_l, pad_t + ph, fill="#333")
        self.create_line(pad_l, pad_t + ph, pad_l + pw, pad_t + ph, fill="#333")
        # y 刻度(3档)
        for i in range(3):
            yy = y0 + (y1 - y0) * i / 2
            self.create_text(pad_l - 5, sy(yy), text=f"{yy:.2g}", anchor="e",
                             font=("Consolas", 9), fill="#555")
        # x 刻度(5档)
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4
            self.create_text(sx(xx), H - 4, text=f"{xx:.1f}", anchor="n",
                             font=("Consolas", 9), fill="#555")
        # 曲线
        ci = 0
        for name, pts in self._series:
            col = self._colors[ci % len(self._colors)]
            ci += 1
            seg = [(x, y) for x, y in pts if x0 <= x <= x1]
            if len(seg) > 1:
                coords = []
                for x, y in seg:
                    coords += [sx(x), sy(y)]
                self.create_line(*coords, fill=col, width=1)
            elif len(seg) == 1:
                x, y = seg[0]
                self.create_oval(sx(x) - 2, sy(y) - 2, sx(x) + 2, sy(y) + 2, fill=col)
        # 图例/标签
        if self._ylabel:
            self.create_text(pad_l + 6, pad_t + ph / 2, text=self._ylabel,
                             anchor="center", angle=90, font=("Microsoft YaHei", 9), fill="#444")
        if self._title:
            self.create_text(pad_l + 2, 8, text=self._title,
                             anchor="w", font=("Microsoft YaHei", 9), fill="#111")
        if self._legend:
            self.create_text(pad_l + pw - 4, 8, text=self._legend,
                             anchor="ne", font=("Microsoft YaHei", 8), fill="#888")
        if self._zoom:
            self.create_text(pad_l + pw - 4, pad_t - 2, text="已放大(双击复位)",
                             anchor="ne", font=("Microsoft YaHei", 8), fill="#f90")


# ---------------- 普通监视子页 ----------------
class MonitorView(tk.Frame):
    """实时监视-普通子页：RC 输入/输出波形 + 打杆动作表(owner 提供 feed/q)"""

    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self.cfg = owner.cfg
        self.q = owner.q
        self._draw_n = -1
        self.alias_s1 = self.cfg.get("alias_out_s1", "S1")
        self.alias_s3 = self.cfg.get("alias_out_s3", "S3")
        self.alias_in = {"rc3": self.cfg.get("alias_in_left", "RC3"),
                         "rc1": self.cfg.get("alias_in_right", "RC1")}

        ttk.Label(self, text="输入 RC1/RC3（PWM，1500≈中）— 最近 20s 滚动",
                  foreground="#555").pack(anchor="w", padx=10)
        self.plot_in = TkPlot(self, height=120)
        self.plot_in.pack(fill="both", expand=True, padx=8)
        ttk.Label(self, text="电机输出 S1/S3（>1500 正转/加速，<1500 反转/减速，1500≈停）",
                  foreground="#555").pack(anchor="w", padx=10, pady=(6, 0))
        self.plot_out = TkPlot(self, height=150)
        self.plot_out.pack(fill="both", expand=True, padx=8)

        ttk.Label(self, text="自动测得动作（每次打杆→回中 记一行）",
                  foreground="#555").pack(anchor="w", padx=10, pady=(6, 0))
        cols = ("no", "tm", "stick", "lat", "rise", "target", "swing", "ends")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=5)
        for c, txt, w in (("no", "#", 32), ("tm", "时刻", 74), ("stick", "杆", 70),
                          ("lat", "反应(s)", 66), ("rise", "OUT爬升(s)", 92),
                          ("target", "目标PWM", 70),
                          ("swing", "摆幅PWM", 70), ("ends", "S1/S3最终PWM", 100)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", padx=8)
        fbt = ttk.Frame(self)
        fbt.pack(fill="x", padx=8, pady=4)
        self.summary = ttk.Label(fbt, text="", foreground="#0066cc")
        self.summary.pack(side="left")
        self.rows = []
        ttk.Button(fbt, text="清空表格", command=self.clear_rows).pack(side="right")
        ttk.Button(fbt, text="清空波形", command=self.clear_buf).pack(side="right", padx=6)
        self.after(150, self._poll)

    def _poll(self):
        while True:
            try:
                it = self.q.get_nowait()
            except queue.Empty:
                break
            self._add_row(it)
        f = self.owner.feed
        if f is not None:
            with f.lock:
                tot = sum(len(v) for v in f.buf.values())
            if tot != self._draw_n:
                self._draw_n = tot
                self._redraw()
        self.after(150, self._poll)

    def _redraw(self):
        f = self.owner.feed
        with f.lock:
            snap = {k: list(v) for k, v in f.buf.items()}
        now = 0.0
        for v in snap.values():
            if v and v[-1][0] > now:
                now = v[-1][0]
        t_min = now - WINDOW_SEC
        s_in = [(self.alias_in[k], [(t - t_min, v) for t, v in snap[k] if t >= t_min])
                for k in ("rc1", "rc3")]
        s_out = [(self.alias_s1 if k == "s1" else self.alias_s3,
                  [(t - t_min, v) for t, v in snap[k] if t >= t_min])
                 for k in ("s1", "s3")]
        self.plot_in.set_series(s_in, ylabel="PWM", xlabel=f"最近 {WINDOW_SEC:.0f}s")
        self.plot_out.set_series(s_out, ylabel="PWM", xlabel=f"最近 {WINDOW_SEC:.0f}s")

    def _add_row(self, it):
        tm = datetime.datetime.fromtimestamp(it["t0"]).strftime("%H:%M:%S")
        s1v = f"{it['s1_end']:.0f}" if it.get("s1_end") is not None else "--"
        s3v = f"{it['s3_end']:.0f}" if it.get("s3_end") is not None else "--"
        self.rows.append(it)
        self.tree.insert("", "end", values=(
            len(self.rows), tm, self.alias_in.get(it.get("stick", ""), it.get("stick", "")),
            f"{it.get('lat', 0):.2f}", f"{it['rise']:.2f}", f"{it['target']:.0f}",
            f"{it['swing']:.0f}", f"{s1v} / {s3v}"))
        if len(self.rows) > 300:
            self.tree.delete(self.tree.get_children()[0])
            self.rows.pop(0)
        n = len(self.rows)
        if n:
            avg = sum(x["rise"] for x in self.rows) / n
            mx = max(x["rise"] for x in self.rows)
            avgl = sum(x.get("lat", 0) for x in self.rows) / n
            self.summary.config(text=f"已测 {n} 次 | 平均反应 {avgl:.2f}s | "
                                      f"平均OUT爬升 {avg:.2f}s | 最长 {mx:.2f}s")
        else:
            self.summary.config(text="")

    def clear_rows(self):
        self.rows = []
        for row in self.tree.get_children():
            self.tree.delete(row)
        self.summary.config(text="")

    def clear_buf(self):
        f = self.owner.feed
        if f:
            with f.lock:
                for k in f.buf:
                    f.buf[k] = []
        self._draw_n = -1
        self.plot_in.set_series([])
        self.plot_out.set_series([])


# ---------------- PID 调试子页 ----------------
class PidDebugView(tk.Frame):
    """5 路 PID 波形(可滚轮缩放,共享时间窗) + 底部滚动数据栏(每行5值+右侧时间)"""

    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self._last_t = {}          # axis -> 已处理到的时间戳
        self._last_len = {}        # axis -> 已绘制时的样本数(节流)
        self._plots = {}
        self._hover = None
        self._axis_var = tk.IntVar(value=PID_AXIS)
        self._follow = True
        self._paused = False
        self._skip_to_latest = False
        self._rown = 0
        self._param_pending = None    # 后台线程放解析结果,主线程 _poll 取走
        self._params_init = False     # 首次切到本页才后台刷一次参数

        bar = ttk.Frame(self, padding=(8, 2))
        bar.pack(fill="x")
        ttk.Label(bar, text="PID 轴: ").pack(side="left")
        self.axis_cb = ttk.Combobox(bar, textvariable=self._axis_var, state="readonly", width=18)
        self.axis_cb["values"] = [PID_AXIS]
        self.axis_cb.pack(side="left")
        self.axis_cb.bind("<<ComboboxSelected>>", lambda e: self._axis_changed())
        self.mode_lbl = ttk.Label(bar, text="", foreground="#b00")
        self.mode_lbl.pack(side="left", padx=8)
        ttk.Label(bar, text="滚轮=缩放 | 双击=复位",
                  foreground="#888").pack(side="left", padx=12)

        # PID 参数快照(来自日志)显示区
        prow = ttk.Frame(self, padding=(8, 0, 8, 2))
        prow.pack(fill="x")
        ttk.Button(prow, text="更新 PID 参数", command=self.refresh_params).pack(side="left")
        ttk.Button(prow, text="选择日志…", command=self.pick_params_file).pack(side="left", padx=4)
        self.param_lbl = tk.Label(prow, text="参数: 点[更新 PID 参数] 经无线调试临时读 G12 最新日志(不占电脑空间，不删 G12 日志)",
                                  fg="#888", anchor="w", justify="left", height=3, wraplength=1500)
        self.param_lbl.pack(side="left", fill="x", expand=True, padx=8)

        # 波形框: 第 1 框=期望/实际同轴对比(蓝/红两线), 后接 P/I/D 各一框
        self.plot_holder = ttk.Frame(self)
        self.plot_holder.pack(fill="both", expand=True, padx=8)
        self.plot_holder.columnconfigure(0, weight=1)
        rows = [("da", ("desired", "achieved"), ("#1f77b4", "#d62728"), 140),
                ("P", ("P",), ("#2ca02c",), 84),
                ("I", ("I",), ("#ff7f0e",), 84),
                ("D", ("D",), ("#9467bd",), 84)]
        for i in range(len(rows)):
            self.plot_holder.rowconfigure(i, weight=1, uniform="pidrow")
        for i, (key, flds, cols, h) in enumerate(rows):
            p = ZoomPlot(self.plot_holder, height=h)
            p.grid(row=i, column=0, sticky="nsew", pady=1)
            if key == "da":
                p.set_title("转向角 期望(蓝) vs 实际(红) — 同一 Y 轴 deg/s")
            else:
                p.set_title(PID_TITLES[flds[0]])
            p.bind("<Enter>", lambda e, pl=p: setattr(self, "_hover", pl))
            p.bind("<MouseWheel>", lambda e, pl=p: self._zoom_at(pl, e))
            p.bind("<Double-Button-1>", lambda e: self._reset_zoom())
            self._plots[key] = (flds, cols, p)

        # 底部数据栏
        ttk.Label(self, text="PID 实时回传（每行 5 个数据；滚轮上下回看历史，右侧=时间）",
                  foreground="#555").pack(anchor="w", padx=10, pady=(4, 0))
        cols = ("desired", "achieved", "P", "I", "D", "tm")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=6)
        for c, txt, w in (("desired", "期望 desired", 110), ("achieved", "实际 achieved", 110),
                          ("P", "P", 70), ("I", "I", 70), ("D", "D", 70), ("tm", "时间", 110)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", padx=8)
        # 滚轮: 上/下回看(滚动会停止自动跟随)
        self.tree.bind("<MouseWheel>", self._on_wheel)
        self.tree.bind("<Button-4>", self._on_wheel)     # linux
        self.tree.bind("<Button-5>", self._on_wheel)
        fbt = ttk.Frame(self)
        fbt.pack(fill="x", padx=8, pady=4)
        self.lbl_info = ttk.Label(fbt, text="未收到 PID_TUNING（需 GCS_PID_MASK≠0 且飞控在自动/引导模式）",
                                  foreground="#b00")
        self.lbl_info.pack(side="left")
        self.pause_btn = tk.Button(fbt, text="⏸ 暂停", width=8,
                                   font=("Microsoft YaHei", 10),
                                   command=self.toggle_pause)
        self.pause_btn.pack(side="left", padx=(14, 0))
        ttk.Button(fbt, text="继续跟随最新", command=lambda: self._set_follow(True)).pack(side="right", padx=4)
        ttk.Button(fbt, text="清空", command=self.clear_all).pack(side="right")
        self.after(150, self._poll)

    def on_shown(self):
        """首次切到 PID 页才触发一次参数后台刷新(避免启动/切页卡顿)"""
        if not self._params_init:
            self._params_init = True
            self.refresh_params()

    # ---- 数据刷新 ----
    def _cur_axis(self):
        try:
            return int(self.axis_cb.get())
        except Exception:
            return PID_AXIS

    def _axis_changed(self):
        ax = self._cur_axis()
        self._last_t[ax] = self._last_t.get(ax, 0.0)
        # 直接重画该轴已有历史
        self._redraw_plots(ax)

    def toggle_pause(self):
        """⏸ 暂停/▶ 继续: 暂停时波形冻结可滚轮细看; 继续后跳到最新实时更新"""
        self._paused = not self._paused
        self.pause_btn.config(text="▶ 继续" if self._paused else "⏸ 暂停",
                              fg="#c90" if self._paused else "#111")
        if not self._paused:
            self._skip_to_latest = True   # 丢弃暂停期间积压, 直接接最新

    def _poll(self):
        pp = self._param_pending
        if pp is not None:
            self._param_pending = None
            self._show_params(*pp)
        f = self.owner.feed
        if f is not None:
            with f.lock:
                md = f.mode
            if md is not None:
                nm = MODE_NAMES.get(md, f"模式{md}")
                if md == 0:
                    self.mode_lbl.config(text=f"⚠ {nm}：PID 不生效", foreground="#d00")
                else:
                    self.mode_lbl.config(text=f"模式: {nm}", foreground="#0a0")
        if f is not None and f.connected:
            ax = self._cur_axis()
            with f.lock:
                avail = sorted(f.pid_hist.keys())
                hist = list(f.pid_hist.get(ax, []))
            # 更新可选项
            cur = list(self.axis_cb["values"])
            if avail != cur:
                self.axis_cb["values"] = avail
                if self.axis_cb.get() == "" and avail:
                    self.axis_cb.set(avail[0])
            # 暂停: 冻结画面, 不消费新样本(此时仍可滚轮缩放细看当前帧)
            if self._paused:
                self.lbl_info.config(text="⏸ 已暂停 — 点 [▶ 继续] 恢复实时", foreground="#f90")
                self.after(150, self._poll)
                return
            # 恢复时跳到最新, 丢弃暂停期间积压的数据
            if self._skip_to_latest and hist:
                self._last_t[ax] = hist[-1][0]
                self._skip_to_latest = False
            # 新样本 -> 表格
            last = self._last_t.get(ax, 0.0)
            new = [r for r in hist if r[0] > last]
            if new:
                self._last_t[ax] = new[-1][0]
                if self._follow:
                    self._append_rows(new)
                else:
                    for r in new:
                        self._append_row(r, autoview=False)
            # 有新增数据才重绘(节流)；zoom 单独在 _zoom_at 已处理
            if len(hist) != self._last_len.get(ax, -1):
                self._last_len[ax] = len(hist)
                self._redraw_plots(ax)
            if not hist:
                self.lbl_info.config(foreground="#b00")
        self.after(150, self._poll)

    # ---- 底部数据栏 ----
    def _append_rows(self, rows):
        for r in rows:
            self._append_row(r, autoview=False)
        # 自动跟随最新
        try:
            self.tree.yview_moveto(1.0)
        except Exception:
            pass

    def _append_row(self, r, autoview=True):
        _t, d, a, p, i, di, _ff = r
        tm = datetime.datetime.fromtimestamp(_t).strftime("%H:%M:%S")
        self._rown += 1
        self.tree.insert("", "end", values=(f"{d:.2f}", f"{a:.2f}",
                                            f"{p:.4f}", f"{i:.5f}", f"{di:.4f}", tm))
        kids = self.tree.get_children()
        if len(kids) > 3000:
            self.tree.delete(kids[0])
        if autoview:
            try:
                self.tree.yview_moveto(1.0)
            except Exception:
                pass

    def _on_wheel(self, ev):
        self._set_follow(False)
        if getattr(ev, "num", 0):
            units = -1 if ev.num == 4 else (1 if ev.num == 5 else 0)
        else:
            units = -1 if ev.delta > 0 else 1
        if units:
            try:
                self.tree.yview_scroll(units, "units")
            except Exception:
                pass
        return "break"

    def _set_follow(self, on):
        self._follow = on
        if on:
            try:
                self.tree.yview_moveto(1.0)
            except Exception:
                pass

    def clear_all(self):
        self._rown = 0
        for row in self.tree.get_children():
            self.tree.delete(row)
        self._last_t = {}

    # ---- PID 参数快照(日志) ----
    def pick_params_file(self):
        p = filedialog.askopenfilename(
            title="选择 tlog(需含 G12 拉取参数段)",
            filetypes=[("MAVLink 日志", "*.tlog"), ("所有文件", "*.*")],
            initialdir=self.owner.cfg.get("storage_dir", ""))
        if p:
            self._load_params_async(p)

    def refresh_params(self):
        """一键更新: 设备检测/拉日志/解析全部在后台线程完成, 不阻塞主线程"""
        self.param_lbl.config(text="正在从 G12 获取 PID 参数…(后台)", fg="#888")
        threading.Thread(target=self._refresh_worker_live, daemon=True).start()

    def _refresh_worker_live(self):
        cfg = self.owner.cfg
        tmp = None
        try:
            # 设备检测放后台(adb 可能 1~2s, 不能卡主线程)
            if adb_devices is None:
                self._param_pending = ("G12", {}, "缺少 adb，请到 设置 配置 adb 路径 或点 [选择日志…]")
                return
            code, out = adb_devices(cfg)
            ok = any(ln.strip().endswith("device") for ln in out.splitlines()[1:])
            if not ok and cfg.get("conn_addr"):
                adb_connect(cfg)
                code, out = adb_devices(cfg)
                ok = any(ln.strip().endswith("device") for ln in out.splitlines()[1:])
            if not ok:
                self._param_pending = (
                    "G12", {},
                    "G12 未连接(无线调试)。请先在『日志读取』页连好 G12 再点[更新 PID 参数]；或点 [选择日志…]")
                return
            files, err = adb_list_tlogs(cfg)
            if not files:
                self._param_pending = ("G12", {}, f"G12 日志列表为空: {err or ''}")
                return
            name, _size = files[-1]      # 列表已按名排序，末尾=最新
            import tempfile
            fd, tmp = tempfile.mkstemp(prefix="g12param_", suffix=".tlog")
            os.close(fd)
            code, out = adb_pull(cfg, G12_TLOG_REMOTE.rstrip("/") + "/" + name, tmp, timeout=240)
            if code != 0 or not os.path.exists(tmp):
                self._param_pending = ("G12", {}, f"拉取失败: {(out or '')[:160]}")
                return
            data = load_param_snapshot(tmp)
            self._param_pending = (f"G12:{name}（临时拉取解析，本地副本已删除）", data, None)
        except Exception as e:
            self._param_pending = ("G12", {}, str(e))
        finally:
            # 只删电脑上的临时副本，绝不动 G12 上的日志
            if tmp and os.path.exists(tmp):
                try:
                    os.remove(tmp)
                except Exception:
                    pass

    def _load_params_async(self, path):
        self.param_lbl.config(text=f"参数: 解析 {os.path.basename(path)} …", fg="#888")
        threading.Thread(target=self._load_worker, args=(path,), daemon=True).start()

    def _load_worker(self, path):
        try:
            data = load_param_snapshot(path)
            err = None
        except Exception as e:
            data, err = {}, str(e)
        # 不跨线程调 Tk API: 结果放共享位, 由主线程 _poll 取走显示
        self._param_pending = (os.path.basename(path), data, err)

    def _show_params(self, src, data, err=None):
        if err:
            self.param_lbl.config(text=f"读取参数失败[{src}]: {err}", fg="#d00")
            return
        if not data:
            self.param_lbl.config(text="该日志没有 PARAM_VALUE 参数快照(可能未含 G12 拉参数段)", fg="#b00")
            return
        groups = [
            ("内环RAT(角速率)", ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D", "ATC_STR_RAT_FF"]),
            ("限制", ["ATC_STR_RAT_MAX", "ATC_STR_ACC_MAX"]),
            ("外环ANG(角度)", ["ATC_STR_ANG_P"]),
            ("SkyDroid MUN", ["ATC_STR_MUN_P", "ATC_STR_MUN_I", "ATC_STR_MUN_D", "ATC_STR_MUN_FF"]),
            ("速度SPD", ["ATC_SPEED_P", "ATC_SPEED_I", "ATC_SPEED_D"]),
        ]
        lines = [f"PID 参数(来源: {src}):"]
        for gname, keys in groups:
            seg = []
            for k in keys:
                if k in data:
                    seg.append(f"{k.split('_')[-1]}={data[k]:.4g}")
            if seg:
                lines.append(f"  {gname}: " + "  ".join(seg))
        self.param_lbl.config(text="\n".join(lines), fg="#1a1a1a")

    # ---- 波形(5 图共享时间窗) ----
    def _redraw_plots(self, ax):
        f = self.owner.feed
        if f is None:
            return
        with f.lock:
            hist = list(f.pid_hist.get(ax, []))
        now = hist[-1][0] if hist else 0.0
        t_min = now - WINDOW_SEC
        # 每个框可含多个字段(第1框: desired+achieved 同轴两线)
        for key, (flds, cols, p) in self._plots.items():
            series = []
            for fl in flds:
                idx = PID_FIELDS.index(fl)       # r=(t,des,ach,P,I,D,FF)
                pts = [(r[0] - t_min, r[1 + idx]) for r in hist if r[0] >= t_min]
                series.append((fl, pts))
            p.set_series(series, colors=cols)
        self.lbl_info.config(text="")
        if hist:
            _t, d, a, p, i, di, _ff = hist[-1]
            nm = AXIS_NAME.get(ax, f"axis{ax}")
            self.lbl_info.config(foreground="#0a0", text=(
                f"[{nm}] 当前: 期望 {d:.2f} | 实际 {a:.2f} | P {p:.4f} I {i:.5f} D {di:.4f} FF {_ff:.3f}"))
        else:
            self.lbl_info.config(foreground="#b00", text="未收到 PID_TUNING")

    def _zoom_at(self, plot, ev):
        """以鼠标 x 位置为中心，缩放共享时间窗"""
        cx = plot.pixel_to_x(ev.x)
        if plot._zoom:
            lo, hi = plot._zoom
        else:
            lo, hi = 0.0, WINDOW_SEC
        span = hi - lo
        if span < 0.3:
            return
        # 向上滚=放大(span缩小), 向下滚=缩小
        factor = 0.72 if ev.delta > 0 else (1.4 if ev.delta < 0 else 1.0)
        if ev.num == 4:
            factor = 0.72
        elif ev.num == 5:
            factor = 1.4
        nspan = max(0.3, min(WINDOW_SEC, span * factor))
        # 保持中心
        frac = 0.5 if cx is None else max(0.0, min(1.0, (cx - lo) / span if span > 0 else 0.5))
        center = lo + span * frac
        nlo = max(0.0, center - nspan / 2)
        nhi = min(WINDOW_SEC, nlo + nspan)
        nlo = max(0.0, nhi - nspan)
        for _flds, _cols, p in self._plots.values():
            p.set_zoom(nlo, nhi)
        return "break"

    def _reset_zoom(self):
        for _flds, _cols, p in self._plots.values():
            p.reset_zoom()
        return "break"


# ---------------- 实时监视整页(连接 + 子页切换) ----------------
class LiveView(tk.Frame):
    """「实时监视」页：共用连接条 + [实时监视|PID调试] 子页切换"""

    def __init__(self, master, cfg=None):
        super().__init__(master)
        if TkPlot is None:
            raise RuntimeError("无法导入主模块 g12_debug_gui")
        self.cfg = cfg if cfg is not None else load_cfg()
        self.q = queue.Queue()
        self.feed = None

        bar = ttk.Frame(self, padding=(6, 4))
        bar.pack(fill="x")
        ttk.Label(bar, text="飞控地址 ").pack(side="left")
        self.addr_var = tk.StringVar(
            value=self.cfg.get("live_addr",
                               f"{self.cfg.get('default_ip', '192.168.10.169')}:5760"))
        ttk.Entry(bar, textvariable=self.addr_var, width=22).pack(side="left", padx=(0, 8))
        self.btn_disc = ttk.Button(bar, text="🔍 自动发现", width=11, command=self.do_discover)
        self.btn_disc.pack(side="left", padx=(0, 4))
        self.btn_usb = ttk.Button(bar, text="USB 一键连", width=10, command=self.do_usb_connect)
        self.btn_usb.pack(side="left", padx=(0, 4))
        self.btn_c = ttk.Button(bar, text="连接", width=8, command=self.do_connect)
        self.btn_c.pack(side="left")
        self.btn_d = ttk.Button(bar, text="断开", width=8, command=self.do_disconnect,
                                state="disabled")
        self.btn_d.pack(side="left", padx=4)
        self.status = tk.StringVar(value="未连接")
        ttk.Label(bar, textvariable=self.status, foreground="#888").pack(side="left", padx=10)
        self.mode_var = tk.StringVar(value="模式: --")
        self.mode_lbl = ttk.Label(bar, textvariable=self.mode_var, foreground="#888")
        self.mode_lbl.pack(side="left", padx=12)

        # 子页切换
        sub = tk.Frame(self)
        sub.pack(fill="x")
        self.btn_sub_mon = tk.Button(sub, text=" 实时监视 ", width=12, relief="sunken",
                                     bg="#cfdfff", command=lambda: self.switch("mon"))
        self.btn_sub_pid = tk.Button(sub, text=" PID 调试 ", width=12, relief="raised",
                                     bg="#e6e6e6", command=lambda: self.switch("pid"))
        self.btn_sub_mon.pack(side="left", padx=(8, 2), pady=2)
        self.btn_sub_pid.pack(side="left", padx=2, pady=2)

        self.holder = ttk.Frame(self)
        self.holder.pack(fill="both", expand=True)
        self.view_mon = MonitorView(self.holder, self)
        self.view_pid = PidDebugView(self.holder, self)
        self._sub = None
        self.switch("mon")
        self.after(200, self._poll_status)
        self.after(400, self.do_discover)     # 启动即自动发现当前设备并连接

    # ---- 自动发现 ----
    def do_discover(self):
        """自动发现当前调试设备(G12/G20...)并连接:
        无线优先(取 adb 设备 IP 拼 :5760); 无无线则 USB adb forward。
        不用手填 IP, 换设备/换 IP 后点一下即可。"""
        if discover_live_addr is None:
            self.status.set("缺少 adb, 请到 设置 配置 adb 路径")
            return
        self.status.set("正在自动发现设备…")

        def work():
            try:
                return discover_live_addr(self.cfg)
            except Exception as e:
                return "", str(e)

        def done(res):
            addr, note = res
            if not addr:
                self.status.set("⚠ 自动发现失败: " + str(note))
                return
            self.addr_var.set(addr)
            self.cfg["live_addr"] = addr
            try:
                save_cfg(self.cfg)
            except Exception:
                pass
            self.status.set(f"✅ {note} → {addr}")
            if not (self.feed and self.feed.is_alive()):
                self.do_connect()

        def runner():
            r = work()
            self.after(0, lambda: done(r))

        threading.Thread(target=runner, daemon=True).start()

    # ---- 连接 ----
    def do_usb_connect(self):
        """USB 有线一键连: 检测 USB adb 设备 -> forward 5760 -> 连上本机转发口"""
        cfg = self.cfg
        if adb_devices is None:
            messagebox.showwarning(APP_TITLE, "缺少 adb，请到 设置 配置 adb 路径")
            return
        code, out = adb_devices(cfg)
        usb = None
        for ln in out.splitlines()[1:]:
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.split()
            if len(parts) >= 2 and parts[1] == "device" and ":" not in parts[0]:
                usb = parts[0]      # 不含 ip:port = USB 设备
                break
        if not usb:
            messagebox.showwarning(
                APP_TITLE,
                "未检测到 USB 调试设备。\n请用数据线连接 G12，并在开发者选项里开启 USB 调试。")
            return
        port = None
        for p in range(15760, 15800):
            code, out = adb_run(cfg, ["-s", usb, "forward", f"tcp:{p}", "tcp:5760"])
            if code == 0 and "error" not in out.lower():
                port = p
                break
        if port is None:
            messagebox.showerror(APP_TITLE, "adb forward 失败")
            return
        self._usb_serial = usb
        self._usb_port = port
        self.addr_var.set(f"127.0.0.1:{port}")
        self.status.set(f"USB 已转发 5760→本机 {port}，连接中…")
        self.do_connect()

    def _remove_forward(self):
        s = getattr(self, "_usb_serial", None)
        p = getattr(self, "_usb_port", None)
        if s and p:
            try:
                adb_run(self.cfg, ["-s", s, "forward", "--remove", f"tcp:{p}"])
            except Exception:
                pass
        self._usb_serial = None
        self._usb_port = None

    def do_connect(self):
        if self.feed and self.feed.is_alive():
            return
        addr = self.addr_var.get().strip()
        if not addr:
            messagebox.showwarning(APP_TITLE, "请填 飞控地址，如 192.168.10.169:5760")
            return
        self.status.set("连接中 …")
        self.btn_c.config(state="disabled")
        self.feed = LiveFeed(addr, self.cfg, self.q)
        self.feed.start()
        self.cfg["live_addr"] = addr
        try:
            save_cfg(self.cfg)
        except Exception:
            pass

    def do_disconnect(self):
        if self.feed:
            self.feed.stop()
            self.feed = None
        self._remove_forward()
        self.status.set("已断开")
        self.btn_c.config(state="normal")
        self.btn_d.config(state="disabled")

    def shutdown(self):
        if self.feed:
            self.feed.stop()
        self._remove_forward()

    def switch(self, name):
        self.view_mon.pack_forget()
        self.view_pid.pack_forget()
        if name == "pid":
            self.view_pid.pack(fill="both", expand=True)
            self.view_pid.on_shown()
            self.btn_sub_pid.config(relief="sunken", bg="#cfdfff")
            self.btn_sub_mon.config(relief="raised", bg="#e6e6e6")
        else:
            self.view_mon.pack(fill="both", expand=True)
            self.btn_sub_mon.config(relief="sunken", bg="#cfdfff")
            self.btn_sub_pid.config(relief="raised", bg="#e6e6e6")
        self._sub = name

    def _poll_status(self):
        f = self.feed
        if f is not None:
            if f.error:
                self.status.set("⚠ " + f.error)
                self.btn_c.config(state="normal")
                self.btn_d.config(state="disabled")
            elif f.connected and not f.stop_flag:
                self.status.set("已连接 · 实时刷新中")
                self.btn_d.config(state="normal")
            with f.lock:
                md = f.mode
            if md is not None:
                nm = MODE_NAMES.get(md, f"模式{md}")
                if md == 0:
                    self.mode_var.set(f"⚠ {nm}：PID 不生效，请切 AUTO/GUIDED")
                    self.mode_lbl.config(foreground="#d00")
                else:
                    self.mode_var.set(f"模式: {nm}")
                    self.mode_lbl.config(foreground="#0a0")
        self.after(200, self._poll_status)


def main():
    root = tk.Tk()
    root.title(APP_TITLE)
    try:
        root.geometry("1180x820")
    except Exception:
        pass
    try:
        ico = os.path.join(HERE, "g12_icon.ico")
        if os.path.exists(ico) and sys.platform.startswith("win"):
            root.iconbitmap(ico)
    except Exception:
        pass
    try:
        root.option_add("*Font", ("Microsoft YaHei", 10))
        st = ttk.Style(root)
        st.configure(".", font=("Microsoft YaHei", 10))
        st.configure("Treeview", font=("Microsoft YaHei", 10), rowheight=22)
        st.configure("Treeview.Heading", font=("Microsoft YaHei", 10))
    except Exception:
        pass
    view = LiveView(root)
    view.pack(fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", lambda: (view.shutdown(), root.destroy()))
    root.mainloop()


def probe(addr):
    """无界面自检：连 5760 收几秒，打印收到数量，验证整条链路"""
    cfg = dict(DEFAULT_CFG)
    cfg["action_thresh"] = 120
    q = queue.Queue()
    feed = LiveFeed(addr, cfg, q)
    feed.start()
    t0 = time.time()
    n_rc = n_out = n_pid = 0
    while time.time() - t0 < 6:
        if feed.error:
            print("错误:", feed.error)
            feed.stop()
            return 1
        with feed.lock:
            n_rc = len(feed.buf["rc1"]) + len(feed.buf["rc3"])
            n_out = len(feed.buf["s1"]) + len(feed.buf["s3"])
            n_pid = sum(len(v) for v in feed.pid_hist.values())
        time.sleep(0.2)
    print(f"OK 收到: RC={n_rc}条 OUT={n_out}条 PID_TUNING={n_pid}条")
    feed.stop()
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--probe":
        _c = load_cfg() if DEFAULT_CFG else {}
        _addr = sys.argv[2] if len(sys.argv) > 2 else \
            f"{_c.get('default_ip', '192.168.10.169')}:5760"
        sys.exit(probe(_addr))
    else:
        main()
