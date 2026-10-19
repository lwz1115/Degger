#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G12 飞控调试助手（Rover 无人船）
================================
功能：
  1) 通过 G12 无线调试(ADB)连接遥控器，读取云卓 RoverGCS 的运行日志(.tlog)
  2) 自定义日志存储目录，历史日志列表(不覆盖旧文件)
  3) 图形化显示：
       · 左摇杆(油门) / 右摇杆(转向) 的 RC 输入变化速率曲线
       · OUT(S1/S3) 输出变化曲线
       · OUT 从中轴值到最大值的耗时(逐条动作 + 平均值)
       · 时间轴同时给相对时间 与 真实世界时间
  4) 单文件分析 与 双栏对比(两份日志并排)两种模式
  5) 顶部设置：日志存储路径 / adb 路径 / 更新源目录 / G12 连接参数 /
     通道中文显示名(只改 GUI 显示，不改数据)
  6) 自动更新：启动自动检查 + 菜单手动检查；发现新版→确认→关旧窗口→开新进程

依赖: Windows 原生 Python3 + tkinter(自带) + pymavlink + adb(platform-tools)
运行: python g12_debug_gui.py
"""
import os
import sys
import json
import time
import shutil
import threading
import datetime
import subprocess
try:
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    _TK = True
except Exception:
    _TK = False
    tk = None
    ttk = filedialog = messagebox = None

if not _TK:
    # 无 tkinter 环境(如无头自检)也能导入本模块的纯逻辑部分
    class _Tk:
        class Tk: pass
        class Canvas: pass
        class Menu: pass
        StringVar = lambda *a, **k: None
    tk = _Tk
    ttk = type("ttk", (), {"LabelFrame": type("LF", (), {}), "Frame": type("F", (), {}),
                           "Label": type("L", (), {}), "Button": type("B", (), {}),
                           "Combobox": type("C", (), {}), "Entry": type("E", (), {}),
                           "Treeview": type("TV", (), {})})
    filedialog = None
    messagebox = None

APP_NAME = "G12 飞控调试助手"
VERSION = "1.0.0"
MAIN_FILENAME = os.path.basename(__file__)

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

# ---------------- 配置 ----------------
DEFAULT_CFG = {
    "storage_dir": r"C:\Users\Administrator\Desktop\G12Debugger\Logs",
    "adb_path": "",
    "update_dir": r"C:\Users\Administrator\Desktop\G12Debugger\updates",
    # G12 无线调试 = 两步：
    #  配对(一次性): 来自 G12『使用配对码配对设备』弹窗 的 IP:端口 + 配对码
    "pair_addr": "",
    "pair_code": "",
    #  连接(每次): 来自 G12『无线调试』主界面顶部的 IP地址和端口(与配对端口不同)
    "conn_addr": "",
    # 固定预填的 G12 IP(仅开头部分, 端口每次变)
    "default_ip": "192.168.10.169",
    # 通道显示名（仅影响 GUI 显示）
    "alias_left_stick": "左摇杆(油门)",
    "alias_right_stick": "右摇杆(转向)",
    "alias_in_left": "RC3",
    "alias_out_left": "OUT(S1/S3)",
    "alias_in_right": "RC1",
    "alias_out_right": "OUT(S1/S3)",
    "alias_out_s1": "S1",
    "alias_out_s3": "S3",
    # 动作识别阈值(PWM)
    "action_thresh": 120,
    # 双栏对比功能开关(设置里可选, 默认关闭)
    "dual_enabled": False,
    # 记住的模式选择: single / dual
    "mode": "single",
}

G12_TLOG_REMOTE = "/sdcard/Android/data/com.skydroid.fly.rover/files/tlogs"


def config_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "g12_config.json")


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    try:
        with open(config_path(), "r", encoding="utf-8") as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    # 兼容旧版 host/port 拆分配置 -> 合并成 addr
    if not cfg.get("pair_addr") and cfg.get("pair_host") and cfg.get("pair_port"):
        cfg["pair_addr"] = f"{cfg['pair_host']}:{cfg['pair_port']}"
    if not cfg.get("conn_addr") and cfg.get("conn_host") and cfg.get("conn_port"):
        cfg["conn_addr"] = f"{cfg['conn_host']}:{cfg['conn_port']}"
    return cfg


def save_cfg(cfg):
    try:
        with open(config_path(), "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception as e:
        messagebox.showerror("错误", f"保存配置失败:\n{e}")


def find_adb():
    """查找 adb 可执行文件"""
    p = shutil.which("adb")
    if p:
        return p
    cands = [
        os.path.expandvars(r"%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"),
        r"C:\Users\%s\AppData\Local\Android\Sdk\platform-tools\adb.exe" % os.environ.get("USERNAME", ""),
        r"C:\Android\platform-tools\adb.exe",
        r"C:\platform-tools\adb.exe",
    ]
    for c in cands:
        if os.path.isfile(c):
            return c
    return ""


# ---------------- ADB 辅助 ----------------
def _pick_serial(cfg):
    """多设备时选一个目标 adb 设备:
    优先用配置里的 serial; 否则自动探测(USB 物理设备优先于无线 adb-xxx)"""
    s = (cfg.get("serial") or "").strip()
    if s:
        return s
    adb = cfg.get("adb_path") or "adb"
    kw = {"capture_output": True, "text": True, "timeout": 15}
    if os.name == "nt":
        kw["creationflags"] = 0x08000000
    try:
        r = subprocess.run([adb, "devices"], **kw)
        lines = [ln.split("\t")[0].strip() for ln in (r.stdout or "").splitlines()
                 if "\tdevice" in ln]
        if not lines:
            return ""
        for ln in lines:                       # 优先 USB 物理设备
            if ln and not ln.startswith("adb-") and "_" not in ln:
                return ln
        return lines[0]                        # 只剩一个或全是无线
    except Exception:
        return ""


def adb_run(cfg, args, timeout=60):
    adb = cfg.get("adb_path") or "adb"
    cmd = [adb]
    # devices/pair/connect 不需要指定目标; 其余命令在多设备时必须 -s,
    # 否则 adb 报 "more than one device/emulator"
    if args and args[0] not in ("devices", "pair", "connect"):
        ser = _pick_serial(cfg)
        if ser:
            cmd += ["-s", ser]
    cmd += args
    kwargs = {"capture_output": True, "text": True, "timeout": timeout}
    if os.name == "nt":
        # adb 是控制台程序, 从 pythonw 启动会闪黑框 -> 禁止创建控制台窗口
        kwargs["creationflags"] = 0x08000000  # CREATE_NO_WINDOW
    try:
        r = subprocess.run(cmd, **kwargs)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return -1, str(e)


def _ip_autodot(entry, var):
    """IP 输入辅助：仅当刚输满 3 位的一段后继续输入下一位时，自动在该段后补 '.'
    绝不动用户已输入的 '.' 与 ':'(端口)，也不会把 IP 重新分组，避免弄坏如 192.168.10.169"""
    # 全角冒号 → 英文半角冒号
    text = var.get()
    if "：" in text:
        var.set(text.replace("：", ":"))
        text = var.get()
    try:
        idx = entry.index("insert")
    except Exception:
        return
    # 只在末尾追加输入时才自动补点(避免改中间把结构弄乱)
    if idx != len(text) or idx <= 0:
        return
    ip = text.split(":", 1)[0]   # 只看冒号前的 IP 部分
    if idx > len(ip):
        return                   # 光标已到端口区，不动
    ch = text[idx - 1]
    if not ch.isdigit():
        return                   # 刚敲的不是数字(如手动点 . 或冒号)，不自动
    seg = ip[:idx - 1].split(".")[-1]   # 最后一位前的那个段(用户刚输满的)
    if len(seg) == 3:
        var.set(text[:idx - 1] + "." + text[idx - 1:])
        try:
            entry.icursor(idx + 1)
        except Exception:
            pass


def _entry_protect(entry, var):
    """让 '.' 与 ':' 不可被删除(退格/删除/剪切选中均拦截)"""
    def _would_remove(kind):
        text = var.get()
        if entry.selection_present():
            try:
                s = int(entry.index("sel.first"))
                e = int(entry.index("sel.last"))
                if any(ch in (".", ":") for ch in text[s:e]):
                    return True
            except Exception:
                pass
        try:
            idx = int(entry.index("insert"))
        except Exception:
            idx = len(text)
        if kind == "back" and idx > 0 and text[idx - 1] in (".", ":"):
            return True
        if kind == "del" and idx < len(text) and text[idx] in (".", ":"):
            return True
        return False

    def _back(ev):
        return "break" if _would_remove("back") else None

    def _del(ev):
        return "break" if _would_remove("del") else None

    def _cut(ev):
        return "break" if _would_remove("cut") else None

    entry.bind("<BackSpace>", _back)
    entry.bind("<Delete>", _del)
    entry.bind("<Control-x>", _cut)


def _digits_only(entry):
    """只允许输入数字"""
    def check(S):
        if not S:
            return True
        return all(ch in "0123456789" for ch in S)
    entry.config(validate="key", validatecommand=(entry.register(check), "%S"))


def _addr_mask(entry, var, ip):
    """IP 输入模板：预填 ip + ':'，. 与 : 已存在；只允许输数字，分隔符删不掉"""
    ip = (ip or "").replace(" ", "").strip(".:")
    var.set((ip + ":") if ip else "")
    entry.icursor(len(var.get()))
    _digits_only(entry)
    _entry_protect(entry, var)


def _split_addr(addr):
    """把 'IP:端口' 拆成 (ip, port)"""
    addr = (addr or "").strip()
    if ":" in addr:
        h, p = addr.rsplit(":", 1)
        return h.strip(), p.strip()
    return "", ""


def _center_over(parent, win, w, h):
    """把子窗口居中显示在父窗口中间"""
    try:
        parent.update_idletasks()
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        win.geometry(f"{w}x{h}+{max(x, 0)}+{max(y, 0)}")
    except Exception:
        win.geometry(f"{w}x{h}")


def adb_pair(cfg):
    """第一步: 配对(一次性)。地址来自 G12『使用配对码配对设备』弹窗"""
    host, port = _split_addr(cfg.get("pair_addr", ""))
    code = (cfg.get("pair_code") or "").strip()
    if not (host and port and code):
        return False, ("配对信息不完整。\n请在 菜单→设置 填入：\n"
                       "· 配对地址(IP:端口)：来自 G12『使用配对码配对设备』弹窗\n"
                       "· 配对码：同上弹窗里的 6 位数字")
    return adb_run(cfg, ["pair", f"{host}:{port}", code])


def adb_connect(cfg):
    """第二步: 连接。地址来自 G12『无线调试』主界面顶部 IP地址和端口"""
    host, port = _split_addr(cfg.get("conn_addr", ""))
    if not (host and port):
        return False, ("连接地址不完整。\n请到 G12『无线调试』主界面(非配对弹窗)顶部查看\n"
                       "『IP地址和端口』，填入 设置→G12 连接地址。")
    return adb_run(cfg, ["connect", f"{host}:{port}"])


def adb_devices(cfg):
    return adb_run(cfg, ["devices", "-l"])


def discover_live_addr(cfg, mav_port=5760):
    """自动发现当前调试设备的 MAVLink 桥地址(不用手填 IP):
      · 无线设备: 从 adb devices 取 IP:port -> 用其 IP 拼 :5760
      · 无无线但有 USB: adb forward 5760 到本机空闲端口 -> 127.0.0.1:port
    返回 (addr, note); 失败 addr 为空, note 为原因。
    """
    code, out = adb_devices(cfg)
    wireless, usb = [], None
    for ln in (out or "").splitlines()[1:]:
        parts = ln.split()
        if len(parts) < 2 or parts[1] != "device":
            continue
        ser = parts[0]
        if ":" in ser and "." in ser:        # 形如 192.168.10.162:41497 = 无线
            ip = ser.split(":")[0]
            if ip not in wireless:
                wireless.append(ip)
        else:                                 # 纯串号 = USB
            usb = ser
    if wireless:
        ip = wireless[0]
        return f"{ip}:{mav_port}", f"无线设备 {ip}(自动发现)"
    if usb:
        for p in range(15760, 15800):
            c2, o2 = adb_run(cfg, ["-s", usb, "forward", f"tcp:{p}", f"tcp:{mav_port}"])
            if c2 == 0 and "error" not in (o2 or "").lower():
                return f"127.0.0.1:{p}", f"USB 设备 {usb} 经 adb forward(自动发现)"
        return "", f"USB 设备 {usb} 端口转发失败"
    return "", "未发现设备：请确认遥控器已开『无线调试』(或插好 USB 线)"


def adb_pull(cfg, remote, local, timeout=300):
    return adb_run(cfg, ["pull", remote, local], timeout=timeout)


def adb_list_tlogs(cfg):
    code, out = adb_run(cfg, ["shell", "ls", "-la", G12_TLOG_REMOTE])
    if code != 0:
        return [], out
    files = []
    for line in out.splitlines():
        parts = line.split()
        # 例: -rwxrwx--- 1 u0_a173 ext_data_rw 3785588 2026-08-26 09:41 xxx.tlog
        # 共 8 段: 权限 链接 属主 属组 大小 日期 时间 文件名  -> 文件名[-1], 大小[-4]
        if len(parts) >= 5 and parts[-1].endswith(".tlog"):
            try:
                size = int(parts[-4])
            except Exception:
                try:
                    size = int(parts[-5])
                except Exception:
                    size = 0
            files.append((parts[-1], size))
    files.sort(key=lambda x: x[0])
    return files, ""


# ---------------- tlog 解析 ----------------
def load_series(path):
    """解析 tlog -> 各通道绝对时间序列。返回 (dict, 错误串)
    dict: {rc1:[(t,val)], rc3:[...], s1:[...], s3:[...]}  t 为 epoch 秒
    """
    if mavutil is None:
        return None, "缺少 pymavlink，请先安装"
    try:
        m = mavutil.mavlink_connection(path)
    except Exception as e:
        return None, f"无法打开 tlog: {e}"
    out = {"rc1": [], "rc3": [], "s1": [], "s3": []}
    while True:
        try:
            msg = m.recv_match()
        except Exception:
            break
        if msg is None:
            break
        t = getattr(msg, "_timestamp", 0)
        ty = msg.get_type()
        if ty == "RC_CHANNELS":
            out["rc1"].append((t, msg.chan1_raw))
            out["rc3"].append((t, msg.chan3_raw))
        elif ty == "SERVO_OUTPUT_RAW":
            out["s1"].append((t, msg.servo1_raw))
            out["s3"].append((t, msg.servo3_raw))
    return out, ""


def rate_series(series, win=2):
    """计算变化速率(PWM/s)，做 win 点均值平滑"""
    res = []
    for i in range(len(series)):
        j0 = max(0, i - win)
        j1 = min(len(series) - 1, i + win)
        t0, v0 = series[j0]
        t1, v1 = series[j1]
        dt = t1 - t0
        if dt > 1e-6:
            res.append((series[i][0], (v1 - v0) / dt))
        else:
            res.append((series[i][0], 0.0))
    return res


def load_param_snapshot(path):
    """读 tlog 里记录过的 PARAM_VALUE -> {参数名: 值}(G12 拉参时会写入日志)"""
    if mavutil is None:
        return {}
    out = {}
    try:
        m = mavutil.mavlink_connection(path)
    except Exception:
        return out
    while True:
        try:
            msg = m.recv_match(type="PARAM_VALUE")
        except Exception:
            break
        if msg is None:
            break
        pid = msg.param_id
        if not isinstance(pid, str):
            pid = pid.decode().strip("\x00")
        out[pid] = msg.param_value
    return out


def detect_events(chan, thresh=120):
    """按输入通道识别动作: 偏离中心>thresh 开始, 回中心结束
    返回 [(t_start_abs, t_end_abs, sign, vmax, t_vmax_abs)]"""
    events = []
    i = 0
    n = len(chan)
    while i < n - 1:
        t, v = chan[i]
        if abs(v - 1500) <= thresh:
            i += 1
            continue
        sign = 1 if v > 1500 else -1
        j = i
        vmax = 0
        t_vmax = t
        while j < n - 1 and abs(chan[j][1] - 1500) > thresh * 0.5:
            dev = sign * (chan[j][1] - 1500)
            if dev > vmax:
                vmax = dev
                t_vmax = chan[j][0]
            j += 1
        events.append((chan[i][0], chan[j][0], sign, vmax, t_vmax))
        i = j
    return events


def measure_action(chan, outA, outB, t0, t1, sign, cfg):
    """单个动作输出指标：OUT 从中轴(1500)开始动 -> 第一次连续爬升到位(到拐点/平台)的时间(rise)。
    只量第一段连续的爬升，避免把“推-松-再推”多次操作误并成一次长爬升。
    另给出该次到达的目标 PWM(target) 与摆幅(swing)。返回 dict 或 None"""
    # 输入真正启动点：事件起点前最近一次从中心(1500±30)离开的时刻(≈手开始打)。
    # 用与输出一致的 30 阈值，避免“输入先过120才算开始”造成的系统性偏晚。
    in_start = None
    for r in reversed(chan):
        if r[0] < t0 - 0.8 or r[0] > t0:
            continue
        if abs(r[1] - 1500) > 30:
            in_start = r[0]
        else:
            break
    if in_start is None:
        in_start = t0
    win_lo = max(t0 - 0.3, in_start)   # 输出窗口也从输入启动点起，排除上一动作残留

    # 选“驱动侧”通道：前进/转向看上升(>1500)那一侧(右转S1、左转S3)；若都在下降(倒车)则选下降大的
    best = None
    for name, out in (("A", outA), ("B", outB)):
        win = [r for r in out if win_lo <= r[0] <= t1 + 0.8]
        if not win:
            continue
        rise_m = max(0.0, max((v - 1500) for _, v in win))
        fall_m = max(0.0, -min((v - 1500) for _, v in win))
        if best is None or (rise_m, fall_m) > best[0]:
            best = ((rise_m, fall_m), name, win)
    (rise_m, fall_m), oname, win = best
    if max(rise_m, fall_m) <= 30:
        return None

    # 开始离开中轴
    i0 = None
    for i, r in enumerate(win):
        if abs(r[1] - 1500) > 30:
            i0 = i
            break
    if i0 is None:
        return None
    t_start = win[i0][0]

    # 找“第一段连续爬升”的终点：沿离开方向单调增长，直到第一次不再增长(平台/回落/转向)
    peak_i = i0
    peak_exc = abs(win[i0][1] - 1500)
    for i in range(i0 + 1, len(win)):
        exc = abs(win[i][1] - 1500)
        if exc > peak_exc:
            peak_exc = exc
            peak_i = i
        else:
            break                       # 第一次停止继续增长 -> 第一段爬升结束
    t_out = win[peak_i][0]
    peak_val = win[peak_i][1]

    def _plateau(out):
        """通道在动作窗内稳定后的输出(自己方向的第一段爬升平台值); 基本没动则取末值"""
        win = [r for r in out if win_lo <= r[0] <= t1 + 0.8]
        if not win:
            return None
        i0 = next((i for i, r in enumerate(win) if abs(r[1] - 1500) > 30), None)
        if i0 is None:
            return win[-1][1]          # 该通道基本没动 -> 末值(≈1500 保持中位)
        pk = i0
        ex = abs(win[i0][1] - 1500)
        for i in range(i0 + 1, len(win)):
            e = abs(win[i][1] - 1500)
            if e > ex:
                pk = i
                ex = e
            else:
                break
        return win[pk][1]

    return {
        "t0": t_start,                    # OUT 实际开始动的时刻(绝对值)
        "lat": max(0.0, t_start - in_start),  # 反应延迟: 输入启动(±30) -> OUT 启动
        "t1": t_out,                      # 第一次爬升到位的时刻(绝对值)
        "sign": sign,
        "rise": t_out - t_start,          # 第一段连续爬升时间(s)
        "target": peak_val,               # 该次到达的目标 PWM 绝对值
        "swing": peak_exc,                # 摆幅(从中轴移动的 PWM 量)
        "out_chan": oname,
        "s1_end": _plateau(outA),         # 动作稳定后 S1(左电机) 最终输出 PWM
        "s3_end": _plateau(outB),         # 动作稳定后 S3(右电机) 最终输出 PWM
    }


def analyze_stick(inp, outA, outB, cfg, chan_label):
    """返回 (事件列表, 汇总, 系列字典用于绘图)"""
    thresh = cfg["action_thresh"]
    events_raw = detect_events(inp, thresh)
    items = []
    for (t0, t1, sign, vmax, _tv) in events_raw:
        it = measure_action(inp, outA, outB, t0, t1, sign, cfg)
        if it:
            items.append(it)
    summary = None
    if items:
        avg_rise = sum(x["rise"] for x in items) / len(items)
        max_rise = max(x["rise"] for x in items)
        avg_swing = sum(x["swing"] for x in items) / len(items)
        avg_lat = sum(x.get("lat", 0) for x in items) / len(items)
        summary = {
            "n": len(items),
            "avg_lat": avg_lat,
            "avg_rise": avg_rise, "max_rise": max_rise, "avg_swing": avg_swing,
        }
    return items, summary


# ---------------- 简单画图(Canvas) ----------------
class TkPlot(tk.Canvas):
    """极简折线图：一次放多条系列 (name, [(x, y), ...])"""
    def __init__(self, master, height=140, **kw):
        kw.setdefault("bg", "white")
        kw.setdefault("highlightthickness", 1)
        kw.setdefault("highlightbackground", "#ccc")
        super().__init__(master, height=height, **kw)
        self._series = []

    def set_series(self, series, colors=("#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"),
                   xlabel="相对时间 (s)", ylabel="", t0abs=None):
        self._series = series
        self._colors = colors
        self._xlabel = xlabel
        self._ylabel = ylabel
        self._t0abs = t0abs
        self.redraw()

    def redraw(self):
        self.delete("all")
        W = max(self.winfo_width(), 60)
        H = max(self.winfo_height(), 40)
        pad_l, pad_r, pad_t, pad_b = 46, 10, 12, 26
        pw, ph = W - pad_l - pad_r, H - pad_t - pad_b
        if pw <= 5 or ph <= 5:
            self.after(50, self.redraw)
            return
        # 收集 x/y 范围
        xs, ys = [], []
        for _n, pts in self._series:
            for x, y in pts:
                xs.append(x)
                ys.append(y)
        if not xs:
            self.create_text(W / 2, H / 2, text="(无数据)", fill="#999")
            return
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        if x1 - x0 < 1e-9:
            x1 = x0 + 1
        if y1 - y0 < 1e-9:
            y1 = y0 + 1
        y0, y1 = y0 - (y1 - y0) * 0.05, y1 + (y1 - y0) * 0.05

        def sx(x):
            return pad_l + (x - x0) / (x1 - x0) * pw

        def sy(y):
            return pad_t + (y1 - y) / (y1 - y0) * ph

        # 零线/中轴线
        self.create_line(sx(x0), sy(0), sx(x1), sy(0), fill="#bbb", dash=(2, 2))
        # 轴
        self.create_line(pad_l, pad_t, pad_l, pad_t + ph, fill="#333")
        self.create_line(pad_l, pad_t + ph, pad_l + pw, pad_t + ph, fill="#333")
        # y 刻度(3档)
        for i in range(3):
            yy = y0 + (y1 - y0) * i / 2
            if abs(yy) < 1e-9:
                continue
            self.create_text(pad_l - 5, sy(yy), text=f"{yy:.0f}", anchor="e", font=("Consolas", 9), fill="#555")
        # x 刻度(4档)
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4
            self.create_text(sx(xx), H - 6, text=f"{xx - x0:.1f}", anchor="n", font=("Consolas", 9), fill="#555")
        # 曲线
        ci = 0
        for name, pts in self._series:
            col = self._colors[ci % len(self._colors)]
            ci += 1
            if len(pts) > 1:
                coords = []
                for x, y in pts:
                    coords += [sx(x), sy(y)]
                self.create_line(*coords, fill=col, width=1)
            elif len(pts) == 1:
                x, y = pts[0]
                self.create_oval(sx(x) - 2, sy(y) - 2, sx(x) + 2, sy(y) + 2, fill=col)
        # 图例
        lx = pad_l + 4
        ci = 0
        for name, _pts in self._series:
            col = self._colors[ci % len(self._colors)]
            ci += 1
            self.create_line(lx, 8, lx + 18, 8, fill=col)
            self.create_text(lx + 21, 8, text=name, anchor="w", font=("Microsoft YaHei", 9), fill="#111")
            lx += 26 + 22 + len(name) * 8
        # x 轴标签
        if self._ylabel:
            self.create_text(pad_l + pw / 2, H - 1, text=self._xlabel, anchor="s", font=("Microsoft YaHei", 9), fill="#444")
        self.create_text(pad_l + 6, pad_t + ph / 2, text=self._ylabel, anchor="center", angle=90, font=("Microsoft YaHei", 9), fill="#444")


# ---------------- 数据分析视图 ----------------
class MiniTimeline(tk.Canvas):
    """动作时间轴：每个动作一条色块(起点~终点)，便于对比位置/时长/间隔"""
    COLORS = ("#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b")

    def __init__(self, master, height=42, **kw):
        kw.setdefault("bg", "white")
        kw.setdefault("highlightthickness", 1)
        kw.setdefault("highlightbackground", "#ccc")
        super().__init__(master, height=height, **kw)
        self._items = []
        self._base = 0.0
        self._span = 1.0
        self.bind("<Configure>", lambda e: self.redraw())

    def draw(self, items, base):
        self._items = list(items)
        self._base = base
        if items:
            t0s = [it["t0"] for it in items]
            t1s = [it["t1"] for it in items]
            lo, hi = min(t0s), max(t1s)
            self._span = max(hi - lo, 0.5) + 2.0   # 两侧留一点空隙
            self._base = lo - 1.0
        else:
            self._span = 1.0
        self.redraw()

    def redraw(self):
        self.delete("all")
        W = max(self.winfo_width(), 80)
        H = max(self.winfo_height(), 30)
        if not self._items:
            self.create_text(W / 2, H / 2, text="(无动作)", fill="#999", font=("Microsoft YaHei", 10))
            return
        pad_l, pad_r = 8, 8
        top, axis_y = 8, H - 4
        span = self._span if self._span > 0 else 1.0

        def sx(t):
            return pad_l + (t - self._base) / span * (W - pad_l - pad_r)

        # 时间轴
        self.create_line(pad_l, axis_y, W - pad_r, axis_y, fill="#888")
        # 刻度(相对文件起点的秒, 取整数)
        step = max(1, int(span / 8))
        for i in range(int(self._base) - 1, int(self._base + span) + 2, max(1, step)):
            tt = float(i)
            if self._base - 0.5 <= tt <= self._base + span + 0.5:
                x = sx(tt)
                self.create_line(x, axis_y - 3, x, axis_y + 3, fill="#888")
                self.create_text(x, axis_y - 2, text=f"{max(0, int(tt))}",
                                 anchor="s", font=("Consolas", 8), fill="#555")
        # 色块(每动作一块, 序号=表格行号)
        for i, it in enumerate(self._items):
            col = self.COLORS[i % len(self.COLORS)]
            x0 = sx(it["t0"])
            x1 = max(x0 + 2, sx(it["t1"]))
            self.create_rectangle(x0, top, x1, axis_y - 5, fill=col, outline="")
            if x1 - x0 >= 16:
                self.create_text((x0 + x1) / 2, (top + axis_y - 5) / 2, text=str(i + 1),
                                 font=("Consolas", 9), fill="white")


class StickView(ttk.LabelFrame):
    """一块摇杆的分析视图: 三张图 + 事件表 + 汇总"""
    def __init__(self, master, title, cfg, alias_in, alias_out, show_s1=True, show_s3=True):
        super().__init__(master, text=title, padding=4)
        self.cfg = cfg
        self.data = None  # analyze 结果
        self._build(alias_in, alias_out, show_s1, show_s3)

    def _build(self, alias_in, alias_out, show_s1, show_s3):
        # 输入
        f0 = ttk.Frame(self)
        f0.pack(fill="x")
        ttk.Label(f0, text=f"输入 {alias_in} (PWM)").pack(side="left")
        self.plot_in = TkPlot(f0, height=150)
        self.plot_in.pack(fill="both", expand=True)
        f1 = ttk.Frame(self)
        f1.pack(fill="both", expand=True)
        ttk.Label(f1, text=f"输入 {alias_in} 变化速率 (PWM/s)").pack(side="left")
        self.plot_rate = TkPlot(f1, height=120)
        self.plot_rate.pack(fill="both", expand=True)
        f2 = ttk.Frame(self)
        f2.pack(fill="both", expand=True)
        ttk.Label(f2, text=f"输出 {alias_out} (PWM)").pack(side="left")
        self.plot_out = TkPlot(f2, height=190)
        self.plot_out.pack(fill="both", expand=True)
        # S1/S3 含义说明
        s1a = self.cfg.get("alias_out_s1", "S1")
        s3a = self.cfg.get("alias_out_s3", "S3")
        ttk.Label(self, text=f"{s1a}/{s3a} = 飞控发给两个电机输出的实际 PWM 值 "
                             f"(1500≈中点停, >1500 正转/加速, <1500 反转/减速)",
                  foreground="#888").pack(anchor="w")
        # 动作时间轴(每个动作一条色块, 序号=下表行号, 底边秒=相对文件起点)
        ttk.Label(self, text="动作时间轴（色块=一次动作，序号对应下表行；底部数字=开始时刻秒）",
                  foreground="#666").pack(anchor="w")
        self.timeline = MiniTimeline(self, height=58)
        self.timeline.pack(fill="x")
        # 事件表
        cols = ("no", "t0", "lat", "rise", "target", "swing", "chan")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=6)
        for c, txt, w in (("no", "动作", 36), ("t0", "开始(s)", 54),
                          ("lat", "反应(s)", 62), ("rise", "OUT爬升(s)", 80),
                          ("target", "目标PWM", 66), ("swing", "摆幅PWM", 66),
                          ("chan", f"{s1a}/{s3a}最终PWM", 100)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x")
        ts = ttk.Scrollbar(self, orient="horizontal", command=self.tree.xview)
        self.tree.configure(xscrollcommand=ts.set)
        ts.pack(fill="x")
        self.summary = ttk.Label(self, text="", foreground="#0066cc")
        self.summary.pack(anchor="w", pady=(4, 0))

    def show(self, stick_result, abs_t0, real_start, real_end):
        """stick_result = (items, summary, series)"""
        items, summary, series = stick_result
        self.data = series
        # 画图
        self.plot_in.set_series([(self.cfg["alias_in_left"] if False else "输入", series["in"])], ylabel="PWM", t0abs=abs_t0)
        self.plot_rate.set_series([("速率", series["rate"])], ylabel="PWM/s", t0abs=abs_t0)
        out_s = []
        if "s1" in series:
            out_s.append((self.cfg["alias_out_s1"], series["s1"]))
        if "s3" in series:
            out_s.append((self.cfg["alias_out_s3"], series["s3"]))
        self.plot_out.set_series(out_s, ylabel="PWM", t0abs=abs_t0)
        # 表格
        for row in self.tree.get_children():
            self.tree.delete(row)
        self.timeline.draw(items, abs_t0)
        for i, it in enumerate(items, 1):
            s1v = f"{it['s1_end']:.0f}" if it.get("s1_end") is not None else "--"
            s3v = f"{it['s3_end']:.0f}" if it.get("s3_end") is not None else "--"
            self.tree.insert("", "end", values=(
                i, f"{it['t0'] - abs_t0:.1f}", f"{it['lat']:.2f}", f"{it['rise']:.2f}",
                f"{it['target']:.0f}", f"{it['swing']:.0f}", f"{s1v} / {s3v}"))
        rt = datetime.datetime.fromtimestamp(real_start).strftime("%H:%M:%S")
        rt2 = datetime.datetime.fromtimestamp(real_end).strftime("%H:%M:%S")
        if summary:
            self.summary.config(text=(
                f"{summary['n']} 次动作 | 平均反应 {summary['avg_lat']:.2f}s | "
                f"平均 OUT 爬升 {summary['avg_rise']:.2f}s | "
                f"最长 {summary['max_rise']:.2f}s | 平均摆幅 {summary['avg_swing']:.0f}PWM | "
                f"真实时间 {rt}→{rt2}"))
        else:
            self.summary.config(text=f"无动作(阈值 {self.cfg['action_thresh']} PWM) | 真实时间 {rt}→{rt2}")


def build_stick_series(chan_in, outA, outB, abs_t0):
    """把绝对时间序列转成相对时间 + 速率，供绘图与指标"""
    base = abs_t0 if abs_t0 else (chan_in[0][0] if chan_in else 0)
    rel = [(t - base, v) for t, v in chan_in]
    rate = rate_series(chan_in)
    rate_rel = [(t - base, v) for t, v in rate]
    sA = [(t - base, v) for t, v in outA]
    sB = [(t - base, v) for t, v in outB]
    return {"in": rel, "rate": rate_rel, "s1": sA, "s3": sB}


def analyze_one_file(path, cfg):
    """加载并分析单个 tlog -> 视图数据字典"""
    series, err = load_series(path)
    if series is None:
        return None, err
    rc1, rc3 = series["rc1"], series["rc3"]
    s1, s3 = series["s1"], series["s3"]
    if not rc1 and not rc3:
        return None, "日志里没有 RC_CHANNELS 数据"
    # 绝对时间基准
    all_t = []
    for a in (rc1, rc3, s1, s3):
        if a:
            all_t.append(a[0][0])
    t0 = min(all_t) if all_t else 0
    t1 = max([a[-1][0] for a in (rc1, rc3, s1, s3) if a] or [t0])
    left_in = rc3 or []
    left_series = build_stick_series(left_in, s1, s3, t0)
    left_items, left_sum = analyze_stick(left_in, s1, s3, cfg, "油门")
    right_in = rc1 or []
    right_series = build_stick_series(right_in, s1, s3, t0)
    right_items, right_sum = analyze_stick(right_in, s1, s3, cfg, "转向")
    return {
        "path": path, "t0": t0, "t1": t1,
        "left": (left_items, left_sum, left_series),
        "right": (right_items, right_sum, right_series),
    }, ""


# ---------------- 主应用 ----------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"{APP_NAME} v{VERSION}")
        self.geometry("1500x940")
        self.minsize(1200, 760)
        try:
            self.state("zoomed")   # Windows 下默认最大化，看得更清楚
        except Exception:
            pass
        # 加大默认字体
        try:
            self.option_add("*Font", ("Microsoft YaHei", 10))
            st = ttk.Style(self)
            st.configure(".", font=("Microsoft YaHei", 10))
            st.configure("Treeview", font=("Microsoft YaHei", 10), rowheight=24)
            st.configure("Treeview.Heading", font=("Microsoft YaHei", 10))
        except Exception:
            pass
        self._set_icon()
        self.cfg = load_cfg()
        if not self.cfg.get("adb_path"):
            self.cfg["adb_path"] = find_adb()
        self.left_file = None      # 单文件模式当前文件
        self.compA = None          # 双栏 A
        self.compB = None          # 双栏 B
        self._build_menu()
        self._build_page_bar()
        self._build_connect_bar()
        self._build_mode_bar()
        self._build_body()
        self.status = tk.StringVar(value="就绪")
        sbar = ttk.Frame(self)
        sbar.pack(fill="x", side="bottom")
        self.spin = ttk.Progressbar(sbar, mode="indeterminate", length=150)
        self.spin.pack(side="right", padx=8, pady=2)
        ttk.Label(sbar, textvariable=self.status, anchor="w").pack(side="left", fill="x", expand=True)
        self._apply_dual_state()
        # 恢复上次模式：默认单栏；双栏需已在设置里开启
        initial = "single"
        if self.cfg.get("mode") == "dual" and self.cfg.get("dual_enabled"):
            initial = "dual"
        self.set_mode(initial, persist=False)
        self.show_page("logs")
        self.after(300, self.check_update_auto)

    def _set_icon(self):
        """设置窗口/任务栏图标(使用同目录的 g12_icon.png / g12_icon.ico)"""
        base = os.path.dirname(os.path.abspath(__file__))
        png = os.path.join(base, "g12_icon.png")
        ico = os.path.join(base, "g12_icon.ico")
        try:
            if os.path.isfile(png):
                img = tk.PhotoImage(file=png)
                self.iconphoto(True, img)
                self._icon_img = img  # 防止被回收
        except Exception:
            pass
        try:
            if os.path.isfile(ico) and sys.platform.startswith("win"):
                self.iconbitmap(default=ico)
        except Exception:
            pass

    # ---------- 菜单 ----------
    def _build_menu(self):
        mb = tk.Menu(self)
        fm = tk.Menu(mb, tearoff=0)
        fm.add_command(label="打开本地日志(.tlog)...", command=self.open_local_single)
        fm.add_separator()
        fm.add_command(label="退出", command=self.destroy)
        mb.add_cascade(label="文件", menu=fm)
        sm = tk.Menu(mb, tearoff=0)
        sm.add_command(label="设置...", command=self.open_settings)
        mb.add_cascade(label="设置", menu=sm)
        hm = tk.Menu(mb, tearoff=0)
        hm.add_command(label="检查更新", command=self.check_update_manual)
        hm.add_command(label="关于", command=self.show_about)
        mb.add_cascade(label="帮助", menu=hm)
        self.config(menu=mb)

    # ---------- 页面切换(日志读取 / 实时监视) ----------
    def _build_page_bar(self):
        bar = tk.Frame(self)
        bar.pack(fill="x")
        self.btn_pg_logs = tk.Button(bar, text=" 日志读取 ", width=12, relief="sunken",
                                     bg="#cfdfff", command=lambda: self.show_page("logs"))
        self.btn_pg_live = tk.Button(bar, text=" 实时监视 ", width=12, relief="raised",
                                     bg="#e6e6e6", command=lambda: self.show_page("live"))
        self.btn_pg_logs.pack(side="left", padx=(8, 2), pady=3)
        self.btn_pg_live.pack(side="left", padx=2, pady=3)
        self.page_holder = ttk.Frame(self)
        self.page_holder.pack(fill="both", expand=True)
        self.page_logs = ttk.Frame(self.page_holder)
        self.page_live = ttk.Frame(self.page_holder)

    def show_page(self, name):
        """日志读取页 / 实时监视页 切换(两个虚拟按键)"""
        self.page_logs.pack_forget()
        self.page_live.pack_forget()
        if name == "live":
            self._ensure_live()
            self.page_live.pack(fill="both", expand=True)
            self.btn_pg_live.config(relief="sunken", bg="#cfdfff")
            self.btn_pg_logs.config(relief="raised", bg="#e6e6e6")
        else:
            self.page_logs.pack(fill="both", expand=True)
            self.btn_pg_logs.config(relief="sunken", bg="#cfdfff")
            self.btn_pg_live.config(relief="raised", bg="#e6e6e6")

    def _ensure_live(self):
        """首次进入实时页才 import 并创建(避免启动即连 G12)"""
        if getattr(self, "live_view", None) is None:
            from g12_live import LiveView
            self.live_view = LiveView(self.page_live, self.cfg)
            self.live_view.pack(fill="both", expand=True)

    # ---------- 连接栏 ----------
    def _build_connect_bar(self):
        bar = ttk.Frame(self.page_logs, padding=(6, 4))
        bar.pack(fill="x")
        ttk.Label(bar, text="G12:").pack(side="left")
        self.g12_status = tk.StringVar(value="未配对 / 未连接")
        ttk.Label(bar, textvariable=self.g12_status, foreground="#888",
                  width=24).pack(side="left", padx=6)
        ttk.Button(bar, text="连接", width=7, command=self.open_pair_wizard).pack(side="left", padx=2)
        ttk.Button(bar, text="?", width=3, command=self.show_g12_help).pack(side="left")
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(bar, text="读G12日志列表", command=self.do_list_remote).pack(side="left", padx=2)
        self.remote_list = ttk.Combobox(bar, width=44)
        self.remote_list.pack(side="left", padx=4)
        ttk.Button(bar, text="拉取选中", command=self.do_pull).pack(side="left", padx=2)
        ttk.Button(bar, text="拉取最新", command=self.do_pull_latest).pack(side="left", padx=2)

    def show_g12_help(self):
        messagebox.showinfo(
            "怎么连 G12",
            "点左边的『连接』按钮，按向导分两步：\n\n"
            "① 在 G12 点『使用配对码配对设备』→ 把弹窗里的\n"
            "   配对码 + IP地址和端口 填进向导第一步\n\n"
            "② 配对成功后，回到 G12『无线调试』主界面 →\n"
            "   把顶部『IP地址和端口』填进向导第二步\n\n"
            "提示：两个端口不同；连接端口每次开无线调试可能变，以 G12 当前显示为准。")

    def _apply_dual_state(self):
        on = bool(self.cfg.get("dual_enabled"))
        st = "normal" if on else "disabled"
        for b in (self.btn_dual, self.btn_loadA, self.btn_loadB):
            try:
                b.config(state=st)
            except Exception:
                pass

    def _build_mode_bar(self):
        bar = ttk.Frame(self.page_logs, padding=(6, 2))
        bar.pack(fill="x")
        ttk.Button(bar, text="单文件分析", command=lambda: self.set_mode("single")).pack(side="left")
        self.btn_dual = ttk.Button(bar, text="双栏对比", command=lambda: self.set_mode("dual"))
        self.btn_dual.pack(side="left", padx=6)
        self.btn_loadA = ttk.Button(bar, text="载入到 左栏", command=lambda: self.load_slot("A"))
        self.btn_loadA.pack(side="left", padx=6)
        self.btn_loadB = ttk.Button(bar, text="载入到 右栏", command=lambda: self.load_slot("B"))
        self.btn_loadB.pack(side="left")
        self.mode_lbl = ttk.Label(bar, text="模式: 单文件分析")
        self.mode_lbl.pack(side="right")

    # ---------- 主体 ----------
    def _build_body(self):
        self.body = ttk.Frame(self.page_logs)
        self.body.pack(fill="both", expand=True)
        # 单文件: 左右摇杆两块
        self.single = ttk.Frame(self.body)
        self.left_stick = StickView(self.single, "左摇杆", self.cfg,
                                    self.cfg["alias_in_left"], self.cfg["alias_out_left"])
        self.right_stick = StickView(self.single, "右摇杆", self.cfg,
                                     self.cfg["alias_in_right"], self.cfg["alias_out_right"])
        self.left_stick.pack(side="left", fill="both", expand=True, padx=3)
        self.right_stick.pack(side="left", fill="both", expand=True, padx=3)
        # 双栏: A/B 左右并排
        self.dual = ttk.Frame(self.body)
        self.compA_view = ttk.LabelFrame(self.dual, text="日志 A")
        self.compB_view = ttk.LabelFrame(self.dual, text="日志 B")
        self.compA_view.pack(side="left", fill="both", expand=True, padx=3)
        self.compB_view.pack(side="left", fill="both", expand=True, padx=3)
        self.dualA = {"left": None, "right": None}
        self.dualB = {"left": None, "right": None}

    def set_mode(self, mode, persist=True):
        """切换显示模式；双栏需已在设置里启用"""
        if mode == "dual" and not self.cfg.get("dual_enabled"):
            messagebox.showwarning(
                "未启用双栏",
                "双栏对比功能默认关闭。\n请到 菜单→设置 → 勾选『启用双栏对比』，再回来使用。")
            return
        self.mode = mode
        if persist:
            self.cfg["mode"] = mode
            save_cfg(self.cfg)
        self.single.pack_forget()
        self.dual.pack_forget()
        if mode == "single":
            self.mode_lbl.config(text="模式: 单文件分析")
            self.single.pack(fill="both", expand=True)
        else:
            self.mode_lbl.config(text="模式: 双栏对比")
            self._ensure_dual_views()
            self.dual.pack(fill="both", expand=True)
            self._fill_dual()

    def _ensure_dual_views(self):
        for w in self.compA_view.winfo_children():
            w.destroy()
        for w in self.compB_view.winfo_children():
            w.destroy()
        # 每个栏内: 左摇杆/右摇杆 上下排
        self.dualA["left"] = StickView(self.compA_view, "左摇杆", self.cfg,
                                        self.cfg["alias_in_left"], self.cfg["alias_out_left"])
        self.dualA["right"] = StickView(self.compA_view, "右摇杆", self.cfg,
                                         self.cfg["alias_in_right"], self.cfg["alias_out_right"])
        self.dualB["left"] = StickView(self.compB_view, "左摇杆", self.cfg,
                                        self.cfg["alias_in_left"], self.cfg["alias_out_left"])
        self.dualB["right"] = StickView(self.compB_view, "右摇杆", self.cfg,
                                         self.cfg["alias_in_right"], self.cfg["alias_out_right"])
        self.dualA["left"].pack(side="top", fill="both", expand=True)
        self.dualA["right"].pack(side="top", fill="both", expand=True)
        self.dualB["left"].pack(side="top", fill="both", expand=True)
        self.dualB["right"].pack(side="top", fill="both", expand=True)

    def _rebuild_dual(self):
        if self.compA_view is not None and self.compA_view.winfo_ismapped():
            pass

    # ---------- 数据载入 ----------
    def open_local_single(self):
        p = filedialog.askopenfilename(
            title="选择 tlog 日志", filetypes=[("MAVLink 日志", "*.tlog"), ("所有文件", "*.*")],
            initialdir=self.cfg.get("storage_dir", ""))
        if not p:
            return
        self.load_file_to(p, "single")

    def load_slot(self, slot):
        if not self.cfg.get("dual_enabled"):
            messagebox.showwarning(
                "未启用双栏",
                "双栏对比功能默认关闭。\n请先到 菜单→设置 → 勾选『启用双栏对比』。")
            return
        p = filedialog.askopenfilename(
            title=f"选择载入 {slot} 栏的 tlog",
            filetypes=[("MAVLink 日志", "*.tlog"), ("所有文件", "*.*")],
            initialdir=self.cfg.get("storage_dir", ""))
        if not p:
            return
        self.set_mode("dual")
        self.load_file_to(p, slot)

    def _start_spin(self):
        try:
            self.spin.start(12)
        except Exception:
            pass

    def _stop_spin(self):
        try:
            self.spin.stop()
        except Exception:
            pass

    def load_file_to(self, path, slot):
        """后台线程解析(大文件不卡界面)，转圈提示，完成后回主线程刷新"""
        self._start_spin()
        self.status.set(f"解析中: {os.path.basename(path)} ...")
        name = os.path.basename(path)

        def work():
            try:
                data, err = analyze_one_file(path, self.cfg)
                self.after(0, lambda: self._finish_load(data, err, name, slot))
            except Exception as e:
                import traceback
                tb = traceback.format_exc()
                self.after(0, lambda: self._finish_load(None, str(e) + "\n\n" + tb, name, slot))

        threading.Thread(target=work, daemon=True).start()

    def _finish_load(self, data, err, name, slot):
        self._stop_spin()
        if data is None:
            messagebox.showerror("解析失败", err)
            self.status.set("解析失败")
            return
        try:
            self._present(data, slot)
            self.status.set(f"已载入 {name}")
        except Exception as e:
            import traceback
            messagebox.showerror("载入出错", traceback.format_exc())
            self.status.set("载入出错")

    def _present(self, data, slot):
        if slot == "single":
            self.set_mode("single")
            self._show_on(data, self.left_stick, self.right_stick)
        elif slot == "A":
            self.compA = data
            self.set_mode("dual")
            self._fill_dual()
        elif slot == "B":
            self.compB = data
            self.set_mode("dual")
            self._fill_dual()

    def _show_on(self, data, vL, vR):
        vL.show(data["left"], data["t0"], data["t0"], data["t1"])
        vR.show(data["right"], data["t0"], data["t0"], data["t1"])

    def _fill_dual(self):
        if self.compA:
            self._show_on(self.compA, self.dualA["left"], self.dualA["right"])
        if self.compB:
            self._show_on(self.compB, self.dualB["left"], self.dualB["right"])

    # ---------- ADB 动作 ----------
    def _run(self, fn, ok_msg, err_prefix=""):
        def work():
            r = fn()
            self.after(0, lambda: self._done(r, ok_msg, err_prefix))
        threading.Thread(target=work, daemon=True).start()

    def _done(self, r, ok_msg, err_prefix):
        code, out = r
        if code == 0:
            self.status.set(ok_msg)
            messagebox.showinfo("成功", ok_msg + ("\n" + out if out.strip() else ""))
        else:
            self.status.set("操作失败")
            messagebox.showerror("失败", err_prefix + "\n" + out)

    def open_pair_wizard(self):
        """两步向导：①配对(配对码+配对IP) ②连接(无线调试主界面IP)——贴合 G12 操作顺序"""
        d = tk.Toplevel(self)
        d.title("连接 G12")
        _center_over(self, d, 620, 440)
        d.transient(self)
        d.resizable(False, False)

        def _clear():
            for w in d.winfo_children():
                w.destroy()

        def page_pair():
            _clear()
            ttk.Label(d, text="① 配对（一次性）", font=("Microsoft YaHei", 12, "bold")
                      ).pack(anchor="w", padx=18, pady=(16, 2))
            ttk.Label(d, justify="left", text=(
                "开始前请确认：\n"
                "· G12 与本电脑连接的是同一个 WiFi\n"
                "· G12 已开启 开发者模式，并已点开『使用配对码配对设备』\n\n"
                "然后从 G12 弹出的配对窗口里，把下面两项填进来："
            )).pack(anchor="w", padx=18)
            f = ttk.Frame(d)
            f.pack(anchor="w", padx=18, pady=10)
            ttk.Label(f, text="配对码:").grid(row=0, column=0, sticky="e", pady=4)
            e_code = ttk.Entry(f, width=36)
            e_code.grid(row=0, column=1, padx=8)
            _digits_only(e_code)
            # 配对码每次不同 -> 每次打开都清空
            ttk.Label(f, text="IP 地址和端口:").grid(row=1, column=0, sticky="e", pady=4)
            e_pair_var = tk.StringVar(value="")
            e_pair = ttk.Entry(f, textvariable=e_pair_var, width=36)
            e_pair.grid(row=1, column=1, padx=8)
            _addr_mask(e_pair, e_pair_var, self.cfg.get("default_ip", "192.168.10.169"))
            btn = ttk.Button(d, text="确认并配对", width=16)
            btn.pack(anchor="w", padx=18, pady=8)

            def do_pair():
                code = e_code.get().strip()
                addr = e_pair.get().strip()
                if not (code and addr):
                    messagebox.showwarning("提示", "请填写 配对码 和 IP 地址和端口", parent=d)
                    return
                self.cfg["pair_code"] = code
                self.cfg["pair_addr"] = addr
                save_cfg(self.cfg)
                btn.config(state="disabled", text="正在配对…")
                def work():
                    rc, out = adb_pair(self.cfg)
                    self.after(0, lambda: after_pair(rc, out))
                threading.Thread(target=work, daemon=True).start()

            def after_pair(rc, out):
                btn.config(state="normal", text="确认并配对")
                if rc == 0:
                    self.g12_status.set("配对: 成功 ✓")
                    page_conn()
                else:
                    messagebox.showerror("配对失败", out, parent=d)

            btn.config(command=do_pair)

        def page_conn():
            _clear()
            ttk.Label(d, text="② 连接", font=("Microsoft YaHei", 12, "bold")
                      ).pack(anchor="w", padx=18, pady=(16, 2))
            ttk.Label(d, justify="left", text=(
                "配对成功 ✓\n\n"
                "现在回到 G12『无线调试』主界面（不是配对弹窗），\n"
                "看顶部『IP 地址和端口』（端口与刚才配对不同），填到下面："
            )).pack(anchor="w", padx=18)
            f = ttk.Frame(d)
            f.pack(anchor="w", padx=18, pady=10)
            ttk.Label(f, text="IP 地址和端口:").grid(row=0, column=0, sticky="e")
            # 连接地址多数不变 -> 回填上次保存的; 空/无效时才用默认 IP+':'
            conn_saved = (self.cfg.get("conn_addr", "") or "").strip()
            base_ip = (self.cfg.get("default_ip", "192.168.10.169") or "").strip().rstrip(".:")
            if conn_saved and ":" in conn_saved:
                init_val = conn_saved
            else:
                init_val = (base_ip + ":") if base_ip else ""
            e_conn_var = tk.StringVar(value=init_val)
            e_conn = ttk.Entry(f, textvariable=e_conn_var, width=36)
            e_conn.grid(row=0, column=1, padx=8)
            e_conn.icursor(len(init_val))
            _digits_only(e_conn)
            _entry_protect(e_conn, e_conn_var)
            btn = ttk.Button(d, text="连接", width=16)
            btn.pack(anchor="w", padx=18, pady=8)
            saved = {}

            def do_conn():
                addr = e_conn.get().strip()
                if not addr:
                    messagebox.showwarning("提示", "请填写 IP 地址和端口", parent=d)
                    return
                saved["addr"] = addr
                self.cfg["conn_addr"] = addr
                save_cfg(self.cfg)
                btn.config(state="disabled", text="正在连接…")
                def work():
                    rc, out = adb_connect(self.cfg)
                    self.after(0, lambda: after_conn(rc, out))
                threading.Thread(target=work, daemon=True).start()

            def after_conn(rc, out):
                btn.config(state="normal", text="连接")
                if rc == 0:
                    self.g12_status.set(f"已连接 {saved.get('addr', '')}")
                    messagebox.showinfo("成功",
                                        "G12 已连接 ✓\n现在可以点『读G12日志列表』拉取日志。",
                                        parent=d)
                    d.destroy()
                else:
                    messagebox.showerror(
                        "连接失败",
                        out + "\n\n请确认：\n· G12 与本电脑同一 WiFi\n· G12 无线调试仍保持打开",
                        parent=d)

            btn.config(command=do_conn)

        page_pair()

    def do_list_remote(self):
        self.status.set("正在读取 G12 日志列表…")
        def work():
            return adb_list_tlogs(self.cfg)
        def done(res):
            files, err = res
            if err:
                messagebox.showerror("失败", err)
                return
            if not files:
                messagebox.showinfo("提示", "G12 上没有找到 tlog（先飞一段产生日志）")
                return
            self.remote_files = files
            self.remote_list["values"] = [f"{n}  ({s//1024}KB)" for n, s in files]
            self.remote_list.current(len(files) - 1)
            self.status.set(f"G12 上找到 {len(files)} 个日志")
        def runner():
            res = work()
            self.after(0, lambda: done(res))
        threading.Thread(target=runner, daemon=True).start()

    def _pick_remote_name(self, latest=False):
        files = getattr(self, "remote_files", None)
        if not files:
            return None
        if latest:
            return files[-1][0]
        sel = self.remote_list.current()
        if sel < 0:
            return files[-1][0]
        return files[sel]

    def do_pull(self):
        self._pull_pick(latest=False)

    def do_pull_latest(self):
        self._pull_pick(latest=True)

    def _pull_pick(self, latest):
        name = self._pick_remote_name(latest)
        if not name:
            messagebox.showinfo("提示", "请先点‘读G12日志列表’")
            return
        stor = self.cfg["storage_dir"]
        os.makedirs(stor, exist_ok=True)
        remote = f"{G12_TLOG_REMOTE}/{name}"
        local = os.path.join(stor, name)
        if os.path.exists(local):
            base, ext = os.path.splitext(name)
            local = os.path.join(stor, f"{base}_{int(time.time())}{ext}")
        self.status.set(f"拉取 {name} ...")
        def work():
            code, out = adb_pull(self.cfg, remote, local)
            return code, out, local
        def done(res):
            try:
                code, out, lpath = res
                if code == 0:
                    self.status.set(f"已保存到 {lpath}")
                    self.load_file_to(lpath, "single")   # 拉到即自动载入并进入单文件分析
                    messagebox.showinfo("完成", f"已拉取并载入分析:\n{lpath}")
                else:
                    self.status.set("拉取失败")
                    messagebox.showerror("失败", out)
            except Exception as e:
                import traceback
                messagebox.showerror("出错", traceback.format_exc())
        def runner():
            res = work()
            self.after(0, lambda: done(res))
        threading.Thread(target=runner, daemon=True).start()

    # ---------- 设置 ----------
    def open_settings(self):
        d = tk.Toplevel(self)
        d.title("设置")
        _center_over(self, d, 560, 640)
        d.transient(self)
        c = dict(self.cfg)
        ent = {}
        rows = [
            ("日志存储目录", "storage_dir"),
            ("adb 路径", "adb_path"),
            ("更新源目录", "update_dir"),
            ("G12 配对地址(IP:端口)［一次性，来自配对码弹窗］", "pair_addr"),
            ("G12 配对码［一次性，同上弹窗］", "pair_code"),
            ("G12 连接地址(IP:端口)［无线调试主界面顶部］", "conn_addr"),
            ("默认 G12 IP(开头部分, 固定预填用)", "default_ip"),
            ("左摇杆显示名", "alias_left_stick"),
            ("右摇杆显示名", "alias_right_stick"),
            ("左输入通道名", "alias_in_left"),
            ("右输入通道名", "alias_in_right"),
            ("输出通道名", "alias_out_left"),
            ("S1 显示名", "alias_out_s1"),
            ("S3 显示名", "alias_out_s3"),
            ("动作识别阈值(PWM)", "action_thresh"),
        ]
        for i, (label, key) in enumerate(rows):
            ttk.Label(d, text=label).grid(row=i, column=0, sticky="e", padx=6, pady=2)
            v = tk.StringVar(value=str(c.get(key, "")))
            ent[key] = v
            ttk.Entry(d, textvariable=v, width=46).grid(row=i, column=1, sticky="w", padx=6, pady=2)
        base = len(rows)
        # 功能开关
        self._dual_var = tk.BooleanVar(value=bool(c.get("dual_enabled")))
        ttk.Checkbutton(d, text="启用『双栏对比』功能（默认关闭；开启后上方按钮才可用）",
                        variable=self._dual_var).grid(row=base, column=0, columnspan=2,
                                                      sticky="w", padx=6, pady=(8, 0))
        mode_txt = "双栏对比" if c.get("mode") == "dual" else "单文件分析"
        ttk.Label(d, text=f"下次启动将记住的模式: {mode_txt}（在单/双栏按钮上切换会自动记住）",
                  foreground="#666").grid(row=base + 1, column=0, columnspan=2,
                                          sticky="w", padx=6, pady=2)
        ttk.Button(d, text="配对(ADB)", command=lambda: self._run(
            lambda: adb_pair({**c, **{k: ent[k].get() for k in ent}}),
            "配对命令已发送", "配对失败")).grid(row=base + 2, column=0, pady=8)

        def ok():
            for k in ent:
                v = ent[k].get()
                if k == "action_thresh":
                    try:
                        c[k] = int(v)
                    except Exception:
                        c[k] = 120
                else:
                    c[k] = v
            c["dual_enabled"] = bool(self._dual_var.get())
            self.cfg = c
            save_cfg(self.cfg)
            self._apply_dual_state()
            # 若关闭双栏但当前正处于双栏, 退回单栏
            if self.mode == "dual" and not c.get("dual_enabled"):
                self.set_mode("single", persist=True)
            d.destroy()
            messagebox.showinfo("已保存", "设置已保存并生效。")
        ttk.Button(d, text="保存", command=ok).grid(row=base + 2, column=1, sticky="w", pady=8)

    # ---------- 更新 ----------
    def check_update_auto(self):
        try:
            self.check_update(silent=True)
        except Exception:
            pass

    def check_update_manual(self):
        self.check_update(silent=False)

    def check_update(self, silent=False):
        udir = self.cfg.get("update_dir", "")
        if not udir or not os.path.isdir(udir):
            if not silent:
                messagebox.showinfo("检查更新", "更新源目录不存在，请在设置里配置。")
            return
        new_path, new_ver = None, None
        for fn in os.listdir(udir):
            if not fn.endswith(".py"):
                continue
            fp = os.path.join(udir, fn)
            ver = self._read_version(fp)
            if ver and ver != VERSION and (new_ver is None or ver > new_ver):
                new_path, new_ver = fp, ver
        if new_path is None:
            if not silent:
                messagebox.showinfo("检查更新", f"已是最新版 v{VERSION}")
            return
        if messagebox.askyesno("发现新版本", f"发现新版本 v{new_ver}\n当前 v{VERSION}\n\n是否更新？(将关闭当前窗口并启动新版)"):
            self._relaunch(new_path)

    @staticmethod
    def _read_version(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("VERSION"):
                        v = line.split("=", 1)[1].strip().strip('"').strip("'")
                        return v
        except Exception:
            pass
        return None

    def _relaunch(self, new_path):
        try:
            subprocess.Popen([sys.executable, new_path])
        except Exception as e:
            messagebox.showerror("启动新版失败", str(e))
            return
        self.destroy()

    def show_about(self):
        has_adb = bool(self.cfg.get("adb_path"))
        messagebox.showinfo(
            "关于",
            f"{APP_NAME} v{VERSION}\n\n"
            f"· 解析引擎: pymavlink {'OK' if mavutil else '缺失'}\n"
            f"· adb: {self.cfg.get('adb_path') or '未找到'}\n"
            f"· 日志目录: {self.cfg.get('storage_dir')}\n"
            f"· 更新目录: {self.cfg.get('update_dir')}")


def main():
    App().mainloop()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        # 无界面自检：解析一个文件并打印指标
        path = sys.argv[2] if len(sys.argv) > 2 else None
        if not path or not os.path.exists(path):
            print("用法: python g12_debug_gui.py --selftest <file.tlog>")
            sys.exit(1)
        data, err = analyze_one_file(path, dict(DEFAULT_CFG))
        if data is None:
            print("错误:", err)
            sys.exit(1)
        for side in ("left", "right"):
            items, summary, series = data[side]
            print(f"== {side} ==  动作数={summary['n'] if summary else 0}")
            for i, it in enumerate(items, 1):
                s1v = f"{it['s1_end']:.0f}" if it.get("s1_end") is not None else "--"
                s3v = f"{it['s3_end']:.0f}" if it.get("s3_end") is not None else "--"
                print(f"  动作{i}: 反应 {it.get('lat',0):.2f}s | OUT爬升 {it['rise']:.2f}s | "
                      f"目标PWM {it['target']:.0f} | "
                      f"摆幅 {it['swing']:.0f}PWM | S1/S3最终 {s1v}/{s3v}")
            if summary:
                print(f"  汇总: 平均反应 {summary.get('avg_lat',0):.2f}s | "
                      f"平均OUT爬升 {summary['avg_rise']:.2f}s | 最长 {summary['max_rise']:.2f}s | "
                      f"平均摆幅 {summary['avg_swing']:.0f}PWM")
    else:
        main()
