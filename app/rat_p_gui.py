# -*- coding: utf-8 -*-
"""
rat_p_gui.py —— RAT(转向)环 + 导航相关 观测量/参数 只读 GUI 监视

只读, 不写飞控。用法:
    python rat_p_gui.py [COM14|COM15|tcp:192.168.10.162:5760]
默认 COM14。窗口固定显示(不会滚掉):
  实时: desired/achieved/误差/有效增益, P/I/D/FF 项输出
  观测: 地速 / 航向 / 转向舵(SERVO1) / 油门(SERVO3) / 模式
  参数: ATC_STR_RAT_* + ATC_STR_ANG_P + WP_SPEED/ACCEL/JERK/PIVOT_ANGLE
        (每 2 秒读一次, 变化写入日志)
"""
import csv
import os
import sys
import math
import time
import queue
import threading

import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
# 项目根目录(本文件在 app\ 子目录), 日志统一放根目录 Logs\ 下便于查看
PROJECT_ROOT = os.path.dirname(HERE)
LOG_DIR = os.path.join(PROJECT_ROOT, "Logs")


def _err(msg):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(os.path.join(LOG_DIR, "rat_gui_err.log"), "a", encoding="utf-8") as f:
            f.write(time.strftime("[%Y-%m-%d %H:%M:%S] ") + str(msg) + "\n")
    except Exception:
        pass


try:
    from pymavlink import mavutil
except Exception as e:
    _err("import pymavlink failed: %r" % (e,))
    raise

# 参数解析: 第一个非 -- 开头的参数 = 设备; --sec N = N 秒后自动退出
SEC = 0.0
_dev = ""
_av = list(sys.argv[1:])
_i = 0
while _i < len(_av):
    _a = _av[_i]
    if _a == "--sec" and _i + 1 < len(_av):
        SEC = float(_av[_i + 1]); _i += 2; continue
    if _a.startswith("--sec="):
        SEC = float(_a.split("=", 1)[1]); _i += 1; continue
    if not _dev and not _a.startswith("--"):
        _dev = _a
    _i += 1
ARG_DEV = _dev   # 空 = 自动发现(无线桥优先)


def resolve_dev():
    """返回 (dev, note): dev 形如 tcp:IP:5760 或 COMxx; 空参时自动发现"""
    if ARG_DEV:
        return ARG_DEV, "手动指定"
    try:
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        from g12_debug_gui import DEFAULT_CFG, load_cfg, discover_live_addr
        cfg = load_cfg() if DEFAULT_CFG else {}
        addr, note = discover_live_addr(cfg)
        if addr:
            return "tcp:" + addr, note
        return "", note
    except Exception as e:
        return "", "自动发现失败: %s" % e
AXIS = 5                      # 5 = Steering
WATCH = ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
         "ATC_STR_RAT_FF", "ATC_STR_RAT_MAX",
         "ATC_STR_ANG_P", "WP_SPEED", "WP_ACCEL",
         "WP_JERK", "WP_PIVOT_ANGLE"]
MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
         6: "FOLLOW", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 15: "GUIDED"}


class App:
    def __init__(self, root):
        self.root = root
        self.dev, self.dev_note = resolve_dev()
        self.stop = False
        self.sample = None          # (des, ach, P, I, D, FF)
        self.mode = None
        self.params = {}
        # 观测: 地速/航向/yaw/舵1/舵3/油门
        self.obs = {"gs": None, "hdg": None, "yaw": None,
                    "s1": None, "s3": None, "thr": None}
        self.log_q = queue.Queue()
        self.param_flash = {}       # nm -> 最近变化时间(用于高亮)
        self.paused = False
        self.n = 0
        # ---- CSV 记录(边跑边落盘, 供离线分析; 每条 flush 便于实时读取) ----
        self.t0 = time.time()
        self.s1_t = -1.0
        self.s3_t = -1.0
        _stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(LOG_DIR, "rat_log_%s.csv" % _stamp)
        self.pcsv_path = os.path.join(LOG_DIR, "rat_log_%s.params.csv" % _stamp)
        self._csv = self._pcsv = None
        self._csvw = self._pcsvw = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            self._csv = open(self.csv_path, "w", newline="", encoding="utf-8")
            self._csvw = csv.writer(self._csv)
            self._csvw.writerow(["t", "mode", "des", "ach", "P", "I", "D", "FF",
                                 "thr", "gs", "hdg", "yaw", "s1", "s3", "s1_t", "s3_t"])
            self._csv.flush()
            self._pcsv = open(self.pcsv_path, "w", newline="", encoding="utf-8")
            self._pcsvw = csv.writer(self._pcsv)
            self._pcsvw.writerow(["t", "name", "value"])
            self._pcsv.flush()
        except Exception as e:
            _err("open csv: %r" % (e,))
        self._build()
        threading.Thread(target=self.reader, daemon=True).start()
        self.tick()

    # ---------- UI ----------
    def _build(self):
        r = self.root
        r.title("RAT 环 + 导航观测量监视 — 只读")
        r.geometry("900x660")

        top = ttk.Frame(r, padding=(10, 8)); top.pack(fill="x")
        ttk.Label(top, text="连接 " + (self.dev or "(未发现设备)"),
                  font=("Microsoft YaHei", 11, "bold")).pack(side="left")
        self.lbl_mode = ttk.Label(top, text="模式: --", font=("Microsoft YaHei", 11))
        self.lbl_mode.pack(side="left", padx=16)
        self.lbl_state = ttk.Label(top, text="启动中…", foreground="#c60")
        self.lbl_state.pack(side="left", padx=16)
        ttk.Button(top, text="暂停显示", command=self.toggle_pause).pack(side="right")
        ttk.Button(top, text="清空日志", command=self.clear_log).pack(side="right", padx=6)

        self.v = {}

        # --- 实时(转向环) ---
        big = ttk.LabelFrame(r, text="转向环 PID_TUNING (axis=5)"); big.pack(fill="x", padx=10, pady=6)
        g = ttk.Frame(big); g.pack(fill="x", padx=8, pady=6)
        for i, (cap, key, col) in enumerate([
                ("desired 期望\ndeg/s", "des", "#1f77b4"),
                ("achieved 实际\ndeg/s", "ach", "#d62728"),
                ("误差\ndes-ach", "err", "#333333"),
                ("有效增益\nP/误差", "gain", "#8a2be2")]):
            f = ttk.Frame(g); f.grid(row=0, column=i, padx=12, sticky="w")
            ttk.Label(f, text=cap, justify="center").pack()
            lb = tk.Label(f, text="-", fg=col, font=("Consolas", 22, "bold"))
            lb.pack(); self.v[key] = lb

        g2 = ttk.Frame(big); g2.pack(fill="x", padx=8, pady=(0, 8))
        for i, (cap, key, col) in enumerate([
                ("P 项输出", "P", "#2ca02c"), ("I 项输出", "I", "#ff7f0e"),
                ("D 项输出", "D", "#9467bd"), ("FF 项输出", "FF", "#555555")]):
            f = ttk.Frame(g2); f.grid(row=0, column=i, padx=12, sticky="w")
            ttk.Label(f, text=cap).pack()
            lb = tk.Label(f, text="-", fg=col, font=("Consolas", 18, "bold"))
            lb.pack(); self.v[key] = lb

        # --- 观测量 ---
        ob = ttk.LabelFrame(r, text="观测 (差速船: SERVO1=左油门73 / SERVO3=右油门74)")
        ob.pack(fill="x", padx=10, pady=6)
        og = ttk.Frame(ob); og.pack(fill="x", padx=8, pady=8)
        for i, (cap, key, col) in enumerate([
                ("地速 groundspeed\nm/s", "gs", "#006699"),
                ("航向 heading\n°", "hdg", "#006699"),
                ("yaw 偏航\n°", "yaw", "#006699"),
                ("左油门 servo1\n(ThrottleLeft)", "s1", "#aa5500"),
                ("右油门 servo3\n(ThrottleRight)", "s3", "#aa5500"),
                ("左右差\n(转向代理)", "diff", "#cc0066"),
                ("油门% throttle", "thr", "#aa5500")]):
            f = ttk.Frame(og); f.grid(row=0, column=i, padx=10, sticky="w")
            ttk.Label(f, text=cap, justify="center", font=("Microsoft YaHei", 8)).pack()
            lb = tk.Label(f, text="-", fg=col, font=("Consolas", 16, "bold"))
            lb.pack(); self.v[key] = lb

        # --- 参数(两行) ---
        ps = ttk.LabelFrame(r, text="飞控参数 (每 0.5 秒读回一次, 变化高亮并记入日志)")
        ps.pack(fill="x", padx=10, pady=6)
        pg = ttk.Frame(ps); pg.pack(fill="x", padx=8, pady=6)
        for i, name in enumerate(WATCH):
            row = 0 if i < 5 else 2
            col = i if i < 5 else (i - 5)
            ttk.Label(pg, text=name, font=("Consolas", 10)).grid(row=row, column=col, padx=8, pady=(4, 0))
            lb = tk.Label(pg, text="-", font=("Consolas", 15, "bold"), fg="#003366")
            lb.grid(row=row + 1, column=col, padx=8)
            self.v["p_" + name] = lb

        # --- 日志 ---
        lf = ttk.LabelFrame(r, text="变化日志"); lf.pack(fill="both", expand=True, padx=10, pady=6)
        self.log = tk.Text(lf, height=8, font=("Consolas", 10))
        self.log.pack(fill="both", expand=True, padx=6, pady=6)

    def toggle_pause(self):
        self.paused = not self.paused

    def clear_log(self):
        self.log.delete("1.0", "end")

    # ---------- 读取线程 ----------
    def reader(self):
        try:
            dev = self.dev
            if not dev:
                self.log_q.put("未发现设备: %s" % self.dev_note)
                return
            if dev.startswith("tcp:"):
                m = mavutil.mavlink_connection(dev, timeout=2)
            else:
                m = mavutil.mavlink_connection(dev, baud=115200, timeout=1)
            self.log_q.put("正在连接 %s (%s) ..." % (dev, self.dev_note))
            m.wait_heartbeat(timeout=8)
            self.log_q.put("已收到心跳, 开始读取")
            t_param = 0.0
            while not self.stop:
                now = time.time()
                if now - t_param >= 0.5:
                    t_param = now
                    for p in WATCH:
                        try:
                            m.mav.param_request_read_send(1, 1, p.encode(), -1)
                        except Exception:
                            pass
                try:
                    msg = m.recv_match(blocking=True, timeout=0.5)
                except Exception:
                    msg = None
                if msg is None:
                    continue
                ty = msg.get_type()
                if ty in ("BAD_DATA", "UNKNOWN"):
                    continue
                if ty == "HEARTBEAT":
                    cm = getattr(msg, "custom_mode", None)
                    if cm is not None:
                        self.mode = cm
                elif ty == "PARAM_VALUE":
                    nm = msg.param_id.replace("\x00", "").strip()
                    if nm in WATCH:
                        v = msg.param_value
                        old = self.params.get(nm)
                        self.params[nm] = v
                        self._log_param(nm, v)
                        if old is not None and abs(v - old) > max(1e-6, abs(old) * 1e-5):
                            ms = int((time.time() % 1) * 1000)
                            self.log_q.put("%s.%03d   %s  %s -> %s" % (
                                time.strftime("%H:%M:%S"), ms, nm,
                                ("%.6g" % old), ("%.6g" % v)))
                            self.param_flash[nm] = time.time()
                elif ty == "PID_TUNING" and getattr(msg, "axis", -1) == AXIS:
                    self.sample = (msg.desired, msg.achieved, msg.P,
                                   msg.I, msg.D, msg.FF)
                    self.n += 1
                    self._log_sample(msg)
                elif ty == "VFR_HUD":
                    self.obs["gs"] = msg.groundspeed
                    self.obs["hdg"] = msg.heading
                    self.obs["thr"] = msg.throttle
                elif ty == "ATTITUDE":
                    self.obs["yaw"] = math.degrees(msg.yaw)
                elif ty == "SERVO_OUTPUT_RAW":
                    self.obs["s1"] = msg.servo1_raw
                    self.obs["s3"] = msg.servo3_raw
                    self.s1_t = time.time() - self.t0
                    self.s3_t = self.s1_t
        except Exception as e:
            _err("reader: %r" % (e,))
            self.log_q.put("读取异常: %s (详见 rat_gui_err.log)" % e)

    # ---------- CSV 记录 ----------
    def _log_sample(self, msg):
        if self._csvw is None:
            return
        o = self.obs
        try:
            self._csvw.writerow([
                "%.3f" % (time.time() - self.t0),
                self.mode if self.mode is not None else -1,
                "%.5f" % msg.desired, "%.5f" % msg.achieved,
                "%.5f" % msg.P, "%.5f" % msg.I, "%.5f" % msg.D, "%.5f" % msg.FF,
                "" if o["thr"] is None else "%.1f" % o["thr"],
                "" if o["gs"] is None else "%.3f" % o["gs"],
                "" if o["hdg"] is None else "%.1f" % o["hdg"],
                "" if o["yaw"] is None else "%.2f" % o["yaw"],
                "" if o["s1"] is None else "%d" % o["s1"],
                "" if o["s3"] is None else "%d" % o["s3"],
                "%.2f" % self.s1_t, "%.2f" % self.s3_t,
            ])
            self._csv.flush()
        except Exception as e:
            _err("csv sample: %r" % (e,))

    def _log_param(self, name, value):
        if self._pcsvw is None:
            return
        try:
            self._pcsvw.writerow(["%.3f" % (time.time() - self.t0), name, "%.7g" % value])
            self._pcsv.flush()
        except Exception as e:
            _err("csv param: %r" % (e,))

    def close_csv(self):
        for h in (self._csv, self._pcsv):
            try:
                if h:
                    h.close()
            except Exception:
                pass

    # ---------- 主线程刷新 ----------
    def tick(self):
        if not self.paused:
            s = self.sample
            if s:
                des, ach, P, I, D, FF = s
                err = des - ach
                self.v["des"].config(text="%.2f" % des)
                self.v["ach"].config(text="%.2f" % ach)
                self.v["err"].config(text="%+.2f" % err)
                self.v["P"].config(text="%.4f" % P)
                self.v["I"].config(text="%.4f" % I)
                self.v["D"].config(text="%.4f" % D)
                self.v["FF"].config(text="%.4f" % FF)
                self.v["gain"].config(text=("%.4f" % (P / err)) if abs(err) > 0.5 else "-")
                self.lbl_state.config(text="运行中 (样本 %d)" % self.n, foreground="#0a0")
            o = self.obs
            self.v["gs"].config(text="-" if o["gs"] is None else "%.2f" % o["gs"])
            self.v["hdg"].config(text="-" if o["hdg"] is None else "%d" % o["hdg"])
            self.v["yaw"].config(text="-" if o["yaw"] is None else "%.0f" % o["yaw"])
            self.v["s1"].config(text="-" if o["s1"] is None else "%d" % o["s1"])
            self.v["s3"].config(text="-" if o["s3"] is None else "%d" % o["s3"])
            self.v["thr"].config(text="-" if o["thr"] is None else "%d" % o["thr"])
            if o["s1"] is not None and o["s3"] is not None:
                self.v["diff"].config(text="%+d" % (o["s1"] - o["s3"]))
            else:
                self.v["diff"].config(text="-")
            nowt = time.time()
            for nm in WATCH:
                v = self.params.get(nm)
                fl = (nowt - self.param_flash.get(nm, 0)) < 1.5   # 变化后高亮 1.5 秒
                self.v["p_" + nm].config(
                    text=("-" if v is None else "%.6g" % v),
                    bg=("#ffe08a" if fl else "SystemButtonFace"))
            if self.mode is not None:
                self.lbl_mode.config(text="模式: %s(%s)" % (MODES.get(self.mode, "?"), self.mode))
        try:
            while True:
                line = self.log_q.get_nowait()
                self.log.insert("end", line + "\n")
                self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(120, self.tick)


def main():
    root = tk.Tk()
    try:
        root.option_add("*Font", ("Microsoft YaHei", 10))
    except Exception:
        pass
    app = App(root)
    if os.environ.get("RAT_GUI_SMOKE"):
        root.after(5000, root.destroy)
    if SEC > 0:
        root.after(int(SEC * 1000), root.destroy)
    root.mainloop()
    app.stop = True
    app.close_csv()
    print("数据已记录: %s" % app.csv_path)
    print("参数已记录: %s" % app.pcsv_path)


if __name__ == "__main__":
    main()
