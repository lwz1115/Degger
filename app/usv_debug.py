#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人船调试助手 (USV Debug Assistant)
====================================
统一了原「G12 飞控调试助手」的连接能力 + 「G12 实时监视」的波形能力 + RAT 环诊断。

页面：
  · 实时监视 : RC 输入 / S1 S3 输出 滚动波形 + 打杆动作自动测量表
  · PID 调试 : 转向环 期望/实际 同轴对比 + P/I/D 分项波形（滚轮缩放）
                底部数据滚动栏（滚轮回看历史，可暂停/继续）
  · 全链路诊断: 导航(航向/航迹误差) -> 外环ANG -> 内环RAT -> 速度环 -> 船体/侧滑 -> 可靠性
                反算各环有效增益并与参数值校验；自动按航段存档到 E:/WSL2/PID_Check
  · 参数     : 飞控参数实时监视，变化高亮 + 变化日志

连接方式：
  · 自动发现   : 无线调试优先（adb 取 IP 拼 :5760），无无线则 USB adb forward
  · USB 一键连
  · 手动填 IP:端口
  · 串口 COMx

运行:  python usv_debug.py [COM14|tcp:IP:5760]
自检:  python usv_debug.py --probe [IP:5760]
依赖:  Windows Python3 + tkinter + pymavlink (+ 可选 adb)
"""
import collections
import csv
import datetime
import glob
import json
import math
import os
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

APP_NAME = "无人船调试助手"
VERSION = "1.0.0"
RAD2DEG = 180.0 / math.pi

# 项目根目录(本文件在 app\ 子目录), 日志统一放根目录 Logs\ 下便于查看
PROJECT_ROOT = os.path.dirname(HERE)
LOG_DIR = os.path.join(PROJECT_ROOT, "Logs")

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None


def errlog(msg):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(os.path.join(LOG_DIR, "usv_err.log"), "a", encoding="utf-8") as f:
            f.write(time.strftime("[%Y-%m-%d %H:%M:%S] ") + str(msg) + "\n")
    except Exception:
        pass


if mavutil is not None:
    # 加固补丁：pymavlink 的 add_message 对“带实例字段的消息”(如 BATTERY_STATUS id≠0、
    # DISTANCE_SENSOR 等) 会报 TypeError: 'NoneType' object does not support item
    # assignment (_instances 为 None)，进而随机打断整条连接。这里包一层容错。
    def _patch_mavutil_add_message():
        try:
            orig = mavutil.add_message
            if getattr(orig, "_usv_patched", False):
                return

            def _safe_add_message(messages, mtype, msg):
                try:
                    orig(messages, mtype, msg)
                except TypeError:
                    try:
                        messages[mtype] = msg      # 退化为只保留最新一条
                    except Exception:
                        pass
            _safe_add_message._usv_patched = True
            mavutil.add_message = _safe_add_message
        except Exception as e:
            errlog("patch add_message: %r" % (e,))

    _patch_mavutil_add_message()


# ============================================================ 配置
DEFAULT_CFG = {
    "storage_dir": r"C:\Users\Administrator\Desktop\G12Debugger\Logs",
    "adb_path": "",
    "serial": "",
    # G12/G20 无线调试：配对(一次性) 用 『使用配对码配对设备』弹窗 的 IP:端口 + 配对码
    "pair_addr": "",
    "pair_code": "",
    # 连接(每次) 用 『无线调试』主界面顶部的 IP地址和端口
    "conn_addr": "",
    "default_ip": "192.168.10.162",
    "live_addr": "",
    # 多遥控器时记住的选择: 发现到多个设备时默认选它, 不弹窗
    "preferred_ip": "",
    # 遥控器通讯录: {uid: {name, pair_addr, conn_addr, ip, last_used}}
    #   uid = adb shell getprop ro.serialno 读到的 Android 序列号(稳定唯一)
    "devices": {},
    "active_uid": "",
    "mav_port": 5760,
    "alias_in_left": "RC3",
    "alias_in_right": "RC1",
    "alias_out_s1": "S1",
    "alias_out_s3": "S3",
    # 实时监视要显示的通道（逗号分隔, 1-16）
    #   rc_channels  : RC 输入通道  (CH1-CH16, 对应 RC_CHANNELS)
    #   servo_channels: 电机/SERVO 输出通道 (SERVO1-16, 对应 SERVO_OUTPUT_RAW)
    "rc_channels": "1,3",
    "servo_channels": "1,3",
    "action_thresh": 120,
    "diag_sec": 30,
    "diag_target": "两者",
    "seg_dir": r"C:\Users\Administrator\Desktop\G12Debugger\航段记录",
    # 调试用: 为 True 时在 MANUAL 等无效模式下也采集 PID 数据(数据是旧值, 诊断不可信)
    "allow_any_mode": False,
    # 连接后自动提速关键消息(运行时请求, 不改持久参数)
    # 实测本机飞控: PID_TUNING 10->20Hz, SERVO_OUTPUT_RAW 2->10Hz 有效
    # 注意: 本飞控上 SET_MESSAGE_INTERVAL 的 -1 是「停止发送」而不是「恢复默认」,
    #       所以断开时必须显式写回原速率, 绝不能用 -1
    "boost_rates": True,
    "att_hz": 10,
    "pid_hz": 20,
    "servo_hz": 10,
}

CFG_NAME = "usv_config.json"
OLD_CFG_NAME = "g12_config.json"


def config_path():
    return os.path.join(HERE, CFG_NAME)


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    # 首次运行自动迁移旧配置（保留原有 IP / 配对信息）
    new = config_path()
    src = new if os.path.exists(new) else os.path.join(HERE, OLD_CFG_NAME)
    try:
        with open(src, "r", encoding="utf-8") as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    if not cfg.get("pair_addr") and cfg.get("pair_host") and cfg.get("pair_port"):
        cfg["pair_addr"] = "%s:%s" % (cfg["pair_host"], cfg["pair_port"])
    if not cfg.get("conn_addr") and cfg.get("conn_host") and cfg.get("conn_port"):
        cfg["conn_addr"] = "%s:%s" % (cfg["conn_host"], cfg["conn_port"])
    return cfg


def save_cfg(cfg):
    try:
        with open(config_path(), "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception as e:
        errlog("save_cfg: %r" % (e,))
        try:
            messagebox.showerror(APP_NAME, "保存配置失败:\n%s" % e)
        except Exception:
            pass


# ============================================================ ADB 工具
def find_adb():
    p = shutil.which("adb")
    if p:
        return p
    cands = [
        os.path.expandvars(r"%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"),
        os.path.join(os.environ.get("USERPROFILE", ""), "AppData", "Local", "Android",
                     "Sdk", "platform-tools", "adb.exe"),
        r"C:\Android\platform-tools\adb.exe",
        r"C:\platform-tools\adb.exe",
    ]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return ""


def _adb_exe(cfg):
    return cfg.get("adb_path") or find_adb() or "adb"


def _sub_kw(timeout=60):
    kw = {"capture_output": True, "text": True, "encoding": "utf-8", "errors": "replace",
          "timeout": timeout}
    if os.name == "nt":
        kw["creationflags"] = 0x08000000   # CREATE_NO_WINDOW：从 pythonw 启动不闪黑框
    return kw


def _pick_serial(cfg):
    """多设备时选目标：优先配置里的 serial；否则 USB 物理设备优先于无线 adb-xxx"""
    s = (cfg.get("serial") or "").strip()
    if s:
        return s
    try:
        r = subprocess.run([_adb_exe(cfg), "devices"], **_sub_kw(15))
        lines = [ln.split("\t")[0].strip() for ln in (r.stdout or "").splitlines()
                 if "\tdevice" in ln]
        if not lines:
            return ""
        for ln in lines:
            if ln and not ln.startswith("adb-") and "_" not in ln:
                return ln
        return lines[0]
    except Exception:
        return ""


def adb_run(cfg, args, timeout=60):
    cmd = [_adb_exe(cfg)]
    # 注意: 调用方自己带 "-s" 时不能再自动追加, 否则会变成 adb -s X -s Y
    if args and args[0] not in ("devices", "pair", "connect", "start-server",
                                "kill-server", "-s"):
        ser = _pick_serial(cfg)
        if ser:
            cmd += ["-s", ser]
    cmd += list(args)
    try:
        r = subprocess.run(cmd, **_sub_kw(timeout))
        return r.returncode, ((r.stdout or "") + (r.stderr or ""))
    except Exception as e:
        return -1, str(e)


def adb_devices(cfg):
    return adb_run(cfg, ["devices", "-l"])


def _split_addr(addr):
    addr = (addr or "").strip().replace("：", ":")
    if ":" in addr:
        h, p = addr.rsplit(":", 1)
        return h.strip(), p.strip()
    return "", ""


def adb_pair(cfg):
    host, port = _split_addr(cfg.get("pair_addr", ""))
    code = (cfg.get("pair_code") or "").strip()
    if not (host and port and code):
        return -1, ("配对信息不完整。\n请到 [设置] 填：\n"
                    "· 配对地址(IP:端口) —— 来自遥控器『使用配对码配对设备』弹窗\n"
                    "· 配对码 —— 同一弹窗里的 6 位数字")
    return adb_run(cfg, ["pair", "%s:%s" % (host, port), code])


def adb_connect(cfg):
    host, port = _split_addr(cfg.get("conn_addr", ""))
    if not (host and port):
        return -1, ("连接地址不完整。\n请到遥控器『无线调试』主界面顶部查看『IP地址和端口』，"
                    "填到 [设置] → 连接地址。")
    return adb_run(cfg, ["connect", "%s:%s" % (host, port)])


# ============================================================ 遥控器通讯录
# 手动配对向导 / 多遥控器识别与命名 的共用底层

def _center_over(parent, win, w, h):
    """把子窗口居中显示在父窗口中间"""
    try:
        parent.update_idletasks()
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        win.geometry("%dx%d+%d+%d" % (w, h, max(x, 0), max(y, 0)))
    except Exception:
        win.geometry("%dx%d" % (w, h))


def _digits_only(entry):
    """只允许输入数字"""
    def check(S):
        if not S:
            return True
        return all(ch in "0123456789" for ch in S)
    entry.config(validate="key", validatecommand=(entry.register(check), "%S"))


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

    entry.bind("<BackSpace>", lambda ev: "break" if _would_remove("back") else None)
    entry.bind("<Delete>", lambda ev: "break" if _would_remove("del") else None)
    entry.bind("<Control-x>", lambda ev: "break" if _would_remove("cut") else None)


def _addr_mask(entry, var, ip):
    """IP 输入模板: 预填 ip + ':'; . 与 : 已存在, 只允许输数字, 分隔符删不掉"""
    ip = (ip or "").replace(" ", "").strip(".:")
    var.set((ip + ":") if ip else "")
    entry.icursor(len(var.get()))
    _digits_only(entry)
    _entry_protect(entry, var)


def adb_run_s(cfg, serial, args, timeout=30):
    """指定某一台设备执行 adb（adb_run 会自动挑 serial, 这里强制指定）"""
    cmd = [_adb_exe(cfg), "-s", serial] + list(args)
    try:
        r = subprocess.run(cmd, **_sub_kw(timeout))
        return r.returncode, ((r.stdout or "") + (r.stderr or ""))
    except Exception as e:
        return -1, str(e)


def device_uid(cfg, serial):
    """读遥控器的唯一标识(Android 序列号)。读不到返回空串。"""
    for prop in ("ro.serialno", "ro.boot.serialno"):
        rc, out = adb_run_s(cfg, serial, ["shell", "getprop", prop], timeout=8)
        if rc == 0:
            lines = [x.strip() for x in (out or "").splitlines() if x.strip()]
            if lines and "error" not in lines[0].lower():
                return lines[0]
    return ""


def book_all(cfg):
    """返回遥控器通讯录 {uid: rec}"""
    d = cfg.get("devices")
    return d if isinstance(d, dict) else {}


def book_name_by_ip(cfg, ip):
    """按 IP 反查通讯录里的名字（picker 显示用）"""
    ip = (ip or "").strip()
    if not ip:
        return ""
    for rec in book_all(cfg).values():
        if rec.get("ip") == ip and rec.get("name"):
            return rec["name"]
    return ""


def book_set(cfg, uid, **kw):
    """写入/更新一条遥控器记录"""
    if not uid:
        return None
    d = cfg.setdefault("devices", {})
    rec = d.get(uid) or {}
    rec.update({k: v for k, v in kw.items() if v not in (None, "")})
    rec["last_used"] = time.time()
    d[uid] = rec
    save_cfg(cfg)
    return rec


def book_rename(cfg, uid, name):
    d = cfg.setdefault("devices", {})
    if uid in d:
        d[uid]["name"] = name
        save_cfg(cfg)
        return True
    return False


def book_del(cfg, uid):
    d = cfg.setdefault("devices", {})
    if uid in d:
        del d[uid]
        save_cfg(cfg)
        return True
    return False


def list_rc_devices(cfg):
    """解析 adb devices -l。返回 [{'serial','state','ip','port','name','usb'}]
    state: device=在线 / offline=掉线 / unauthorized=未授权"""
    code, out = adb_devices(cfg)
    devs = []
    for ln in (out or "").splitlines()[1:]:
        parts = ln.split()
        if len(parts) < 2:
            continue
        ser, state = parts[0], parts[1]
        if state not in ("device", "offline", "unauthorized", "no"):
            continue
        d = {"serial": ser, "state": state, "ip": "", "port": "",
             "name": "", "usb": False}
        if ":" in ser and "." in ser:
            ip, port = ser.rsplit(":", 1)
            d["ip"], d["port"] = ip, port
        else:
            d["usb"] = True
        for p in parts[2:]:
            if ":" in p:
                k, v = p.split(":", 1)
                if k == "model":
                    d["name"] = v
                    break
        devs.append(d)
    return devs


def discover_candidates(cfg, mav_port=None):
    """收集可选遥控器。返回 (cands, usb_serial)
    cands: [{'ip','port','name','state'}]，在线在前，同 IP 去重"""
    devs = list_rc_devices(cfg)
    cands, usb_serial = [], None
    for d in devs:
        if d["usb"]:
            if d["state"] == "device":
                usb_serial = usb_serial or d["serial"]
            continue
        cands.append({"ip": d["ip"], "port": d["port"],
                      "name": d["name"] or "", "state": d["state"]})
    cands.sort(key=lambda c: c["state"] != "device")
    seen, uniq = set(), []
    for c in cands:
        if c["ip"] in seen:
            continue
        seen.add(c["ip"])
        uniq.append(c)
    return uniq, usb_serial


def discover_live_addr(cfg, mav_port=None, prefer_ip=None):
    """自动发现 MAVLink 桥地址。返回 (addr, note, cands)
    - 只有一个在线无线设备         -> 直接用它
    - prefer_ip 命中在线设备        -> 直接用它（记住的选择, 不弹窗）
    - 有多个候选 / 无法自动确定     -> addr 返回空, 由调用方用 cands 弹窗让用户选
    - 无无线但有 USB               -> adb forward 到本机空闲端口"""
    mav_port = int(str(mav_port or cfg.get("mav_port") or 5760))
    cands, usb = discover_candidates(cfg, mav_port)
    online = [c for c in cands if c["state"] == "device"]
    if prefer_ip:
        for c in online:
            if c["ip"] == prefer_ip:
                return ("%s:%d" % (c["ip"], mav_port),
                        "无线设备 %s (记住的选择)" % c["ip"], cands)
    if len(online) == 1:
        c = online[0]
        return ("%s:%d" % (c["ip"], mav_port),
                "无线设备 %s (自动发现)" % c["ip"], cands)
    if online:
        return "", "发现 %d 个在线遥控器，请选择" % len(online), cands
    if cands:
        return "", "无法自动确定（%d 个未在线条目），请选择" % len(cands), cands
    if usb:
        for p in range(15760, 15800):
            c2, o2 = adb_run(cfg, ["-s", usb, "forward", "tcp:%d" % p, "tcp:%d" % mav_port])
            if c2 == 0 and "error" not in (o2 or "").lower():
                return "127.0.0.1:%d" % p, "USB 设备 %s 经 adb forward (自动发现)" % usb, []
        return "", "USB 设备 %s 端口转发失败" % usb, []
    return "", "未发现设备：请确认遥控器已开『无线调试』(或插好 USB 线)", []


def pick_device(parent, cands, current="", book_map=None):
    """模态设备选择框。返回 (ip, remember)；取消返回 (None, False)
    book_map: {'IP': '自定义名'} —— 通讯录里的名字优先于型号"""
    book_map = book_map or {}
    top = tk.Toplevel(parent)
    top.title("选择遥控器")
    top.transient(parent)
    top.resizable(False, False)
    ttk.Label(top, padding=(12, 10, 12, 4),
              text="发现 %d 个遥控器 / 条目，请选择要连接的：" % len(cands)
              ).pack(anchor="w")
    lb = tk.Listbox(top, width=58, height=min(9, max(3, len(cands))),
                    font=("Consolas", 10), activestyle="none")
    lb.pack(fill="both", expand=True, padx=12)
    for c in cands:
        nm = book_map.get(c["ip"]) or ("(%s)" % c["name"] if c["name"] else "(未命名)")
        lb.insert("end", "%-17s %-8s %s" % (c["ip"] or "?", c["state"], nm))
    default = 0
    for i, c in enumerate(cands):
        if c["ip"] and c["ip"] == current:
            default = i
            break
    else:
        for i, c in enumerate(cands):
            if c["state"] == "device":
                default = i
                break
    if lb.size():
        lb.selection_set(default)
        lb.activate(default)
    remember = tk.BooleanVar(value=True)
    ttk.Checkbutton(top, text="记住这个选择（下次自动用它，不再询问）",
                    variable=remember).pack(anchor="w", padx=12, pady=(6, 0))
    out = {"ip": None, "remember": False}

    def ok(_ev=None):
        sel = lb.curselection()
        if sel:
            out["ip"] = cands[sel[0]]["ip"]
            out["remember"] = bool(remember.get())
        top.destroy()

    bf = ttk.Frame(top, padding=(12, 8, 12, 12))
    bf.pack(fill="x")
    ttk.Button(bf, text="取消", width=8, command=top.destroy).pack(side="right", padx=4)
    ttk.Button(bf, text="连接", width=8, command=ok).pack(side="right")
    ttk.Label(bf, foreground="#888",
              text="改名/删除请用连接栏的【通讯录】按钮").pack(side="left")
    lb.bind("<Double-Button-1>", ok)
    lb.bind("<Return>", ok)
    top.grab_set()
    top.wait_window()
    return out["ip"], out["remember"]


# ============================================================ MAVLink 配置 / 常量
PID_AXIS = 5                     # 5 = Steering 转向
PID_FIELDS = ("desired", "achieved", "P", "I", "D", "FF")
AXIS_NAME = {0: "None", 1: "Roll", 2: "Pitch", 3: "Yaw", 4: "Throttle/AccZ",
             5: "Steering 转向", 6: "Landing", 7: "Aux"}
MODE_NAMES = {0: "MANUAL 手动", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
              6: "FOLLOW", 9: "CIRCLE", 10: "AUTO 自动", 11: "RTL",
              15: "GUIDED 引导", 16: "INITIALISING"}
# 只有这些模式才真正运行「转向角速率控制器」，PID_TUNING 才是实时有效数据。
# MANUAL(0) 是直接把遥控器输入透传给电机，不经过姿态控制器，
# 此时报文里是上一次运行留下的旧值 -> 属于垃圾数据，不采集。
PID_ACTIVE_MODES = {1, 3, 4, 5, 6, 9, 10, 11, 15}

# 速度环用的 PID_TUNING axis（实测本机飞控：GCS_PID_MASK=3 时 axis=4 是速度环）
SPEED_AXIS = 4

# 每个 PID 轴的画像：报文单位、内部单位换算、对应参数、相关参数
#   need_div=True 表示报文单位与内部不同（转向率：报文 deg/s，内部 rad/s）
AXIS_PROFILE = {
    5: {"title": "转向环 ATC_STR_RAT", "what": "转向率", "out_unit": "deg/s",
        "ref": "yawrate", "need_div": True, "limit": "ATC_STR_RAT_MAX",
        "map": {"FF": "ATC_STR_RAT_FF", "P": "ATC_STR_RAT_P",
                "I": "ATC_STR_RAT_I", "D": "ATC_STR_RAT_D"},
        "extra": ("ATC_STR_RAT_MAX", "ATC_STR_ACC_MAX", "ATC_STR_ANG_P",
                  "ATC_STR_RAT_IMAX", "ATC_STR_RAT_FLTD", "ATC_STR_RAT_FLTE",
                  "ATC_STR_RAT_SMAX", "MOT_SLEWRATE")},
    4: {"title": "速度环 ATC_SPEED", "what": "速度", "out_unit": "m/s",
        "ref": "gs", "need_div": False, "limit": None,
        "map": {"FF": None, "P": "ATC_SPEED_P",
                "I": "ATC_SPEED_I", "D": "ATC_SPEED_D"},
        "extra": ("CRUISE_SPEED", "CRUISE_THROTTLE", "WP_SPEED", "WP_ACCEL",
                  "MOT_SLEWRATE")},
}
WATCH = ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D", "ATC_STR_RAT_FF",
         "ATC_STR_RAT_MAX", "ATC_STR_ACC_MAX", "ATC_STR_RAT_FLTT", "ATC_STR_RAT_FLTD",
         "ATC_STR_RAT_FLTE", "ATC_STR_RAT_IMAX", "ATC_STR_RAT_SMAX",
         "ATC_STR_ANG_P",
         "ATC_SPEED_P", "ATC_SPEED_I", "ATC_SPEED_D", "ATC_SPEED_FF", "ATC_SPEED_IMAX",
         "CRUISE_SPEED", "CRUISE_THROTTLE",
         "WP_SPEED", "WP_ACCEL", "WP_JERK", "WP_PIVOT_ANGLE",
         "SERVO1_MIN", "SERVO1_MAX", "SERVO1_TRIM", "SERVO1_FUNCTION",
         "SERVO3_MIN", "SERVO3_MAX", "SERVO3_TRIM", "SERVO3_FUNCTION",
         # 外环(位置控制器) —— S 型走位在这里产生, 不能只盯内环
         "PSC_POS_P", "PSC_VEL_P", "PSC_VEL_I", "PSC_VEL_D",
         "PSC_VEL_IMAX", "PSC_VEL_FLTE", "PSC_VEL_FLTD", "PSC_VEL_FF",
         "TURN_RADIUS", "WP_RADIUS", "ATC_TURN_MAX_G"]

# SERVO 行程缺省值(参数未取到时的回退)
SERVO_DEF = {"MIN": 1100.0, "MAX": 1900.0, "TRIM": 1500.0}
WINDOW_SEC = 20.0
MAX_AGE = 600.0
# rc/舵机缓冲只需覆盖显示窗口 + 动作测量窗口; 跟着 MAX_AGE=600s 走会把界面拖卡
BUF_AGE = 120.0
# 单条曲线最大绘制点数(超出按包络抽稀), 防止画布项坐标数爆炸
PLOT_MAX_PTS = 600
# 参数轮询速度(个/秒)。参数多了后太慢会让报告里印出的值偏旧
PARAM_PER_SEC = 15
# 航段记录目录：每跑完一段导航数据自动存一份报告到这里
SEG_DIR_DEFAULT = r"C:\Users\Administrator\Desktop\G12Debugger\航段记录"


def parse_chan_list(s, lo=1, hi=16):
    """解析通道配置字符串 -> 升序去重列表。
    支持 "1,3" / "1-4" / "1,3,5-8"；空串或非法返回默认 [1,3]。
    """
    out = []
    for part in str(s or "").replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            try:
                a, b = int(a), int(b)
            except Exception:
                continue
            if a > b:
                a, b = b, a
            out += list(range(max(lo, a), min(hi, b) + 1))
        else:
            try:
                v = int(part)
            except Exception:
                continue
            if lo <= v <= hi:
                out.append(v)
    out = sorted(set(out))
    return out or [1, 3]


def chan_key(kind, c):
    """通道缓冲名: rcN / sN"""
    return ("rc%d" if kind == "rc" else "s%d") % c


def get_seg_dir(cfg=None):
    return (cfg or DEFAULT_CFG).get("seg_dir") or SEG_DIR_DEFAULT


# 航段有效性判据: 低于这些值说明船根本没在跑, 该段不能用于调参
IDLE_THR_PCT = 5.0      # 油门均值下限 (%)
IDLE_GS_MPS = 0.30      # 地速均值下限 (m/s)
# "发散"判定前必须先确认真的在振荡, 否则缓慢漂移会被误判
OSC_MIN_HZ = 0.30       # 过零频率下限 (Hz)
OSC_MIN_ZC = 4          # 过零次数下限
# 期望值认为"已顶到限幅"的比例 (与飞控内部 padding 一致)
SAT_FRAC = 0.92


def _next_seg_index(seg_dir, day):
    """当日编号：扫描目录里 <day>_NNN_* 的最大编号 + 1（重启后也不会重复）"""
    mx = 0
    try:
        for fn in os.listdir(seg_dir):
            if fn.startswith(day + "_"):
                part = fn[len(day) + 1:].split("_", 1)[0]
                if part.isdigit():
                    mx = max(mx, int(part))
    except Exception:
        pass
    return mx + 1


# ============================================================ 局域网扫描（不依赖 adb）
# 常见手机热点 / 路由器 / 软 AP 网段（第一遍扫不到时深扫用）
COMMON_PREFIXES = ("192.168.43.",   # Android 热点(默认)
                   "192.168.42.",   # Android USB tethering
                   "192.168.137.",  # Windows 移动热点
                   "192.168.0.", "192.168.1.", "192.168.2.", "192.168.4.",
                   "192.168.8.", "192.168.100.",
                   "172.20.10.",    # iPhone 个人热点
                   "10.42.0.", "10.0.0.")


def _ipconfig_ipv4():
    """用 ipconfig 枚举【所有网卡】的 IPv4（ghostbyname 会漏掉热点/虚拟网卡）"""
    if os.name != "nt":
        return []
    try:
        r = subprocess.run(["ipconfig"], **_sub_kw(10))
        out = r.stdout or ""
    except Exception:
        return []
    ips = []
    for ln in out.splitlines():
        if "IPv4" not in ln or ":" not in ln:
            continue
        ip = ln.rsplit(":", 1)[1].strip()
        for junk in ("(Preferred)", "(首选)", "(Deprecated)", "(已弃用)"):
            ip = ip.replace(junk, "")
        ip = ip.strip()
        if ip.count(".") == 3 and not ip.startswith("127."):
            ips.append(ip)
    return ips


def local_subnets(limit=6):
    """返回本机候选 /24 网段前缀，如 ['192.168.43.']（含热点网卡）"""
    cand = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))       # 默认路由的那个网卡（最可能对着热点）
            ip = s.getsockname()[0]
            if not ip.startswith("127."):
                cand.append(ip.rsplit(".", 1)[0] + ".")
        finally:
            s.close()
    except Exception:
        pass
    for ip in _ipconfig_ipv4():
        cand.append(ip.rsplit(".", 1)[0] + ".")
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if not ip.startswith("127."):
                cand.append(ip.rsplit(".", 1)[0] + ".")
    except Exception:
        pass
    out, seen = [], set()
    for p in cand:
        if p and p not in seen:
            seen.add(p)
            out.append(p)
    return out[:limit]


def probe_mavlink(ip, port=5760, timeout=2.5):
    """连一下看能不能收到 HEARTBEAT（确认是 MAVLink 桥而不是别的服务）"""
    if mavutil is None:
        return False
    try:
        m = mavutil.mavlink_connection("tcp:%s:%d" % (ip, port), timeout=timeout)
        hb = m.wait_heartbeat(timeout=timeout)
        try:
            m.close()
        except Exception:
            pass
        return hb is not None
    except Exception:
        return False


def scan_lan(port=5760, extra_prefixes=(), timeout=0.35, workers=64, deep=False):
    """扫描候选 /24 网段的 TCP 端口，返回【确认是 MAVLink】的 IP 列表。
    不需要 adb / 无线调试。deep=True 时额外扫常见热点/路由器网段。"""
    import concurrent.futures as cf
    prefixes = []
    for p in list(extra_prefixes) + local_subnets():
        if p and p not in prefixes:
            prefixes.append(p)
    if deep:
        for p in COMMON_PREFIXES:
            if p not in prefixes:
                prefixes.append(p)
    if not prefixes:
        return []
    targets = ["%s%d" % (p, i) for p in prefixes for i in range(1, 255)]

    def one(ip):
        try:
            s = socket.create_connection((ip, port), timeout)
            s.close()
            return ip
        except Exception:
            return None

    reach = []
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        for r in ex.map(one, targets):
            if r:
                reach.append(r)
    return [ip for ip in reach if probe_mavlink(ip, port)]


# ============================================================ 数据线程
class LiveFeed(threading.Thread):
    """后台收 MAVLink；缓冲 RC/输出/PID/姿态/HUD；自动轮询参数；可选 CSV 落盘"""

    def __init__(self, dev, cfg, out_q, csv_path=None):
        super().__init__(daemon=True)
        self.dev = dev
        self.cfg = cfg
        self.out_q = out_q
        self.csv_path = csv_path
        self.stop_flag = False
        self.lock = threading.RLock()
        # 实时监视通道: 由配置决定（默认 rc1/rc3 + s1/s3）
        self.rc_list = parse_chan_list(self.cfg.get("rc_channels"))
        self.servo_list = parse_chan_list(self.cfg.get("servo_channels"))
        self.buf = {}
        for c in self.rc_list:
            self.buf[chan_key("rc", c)] = []
        for c in self.servo_list:
            self.buf[chan_key("s", c)] = []
        self.s1_t = -1.0
        self.pid_hist = {}
        self.att = []                # [(t, yawspeed rad/s)]
        self.hud = []                # [(t, groundspeed, throttle)]
        self.params = {}
        self.param_changes = []
        self.mode = None
        self.n_hb = 0
        self.pid_valid = False      # 仅当模式会跑转向速率控制器时, PID_TUNING 才有效
        self.n_pid_raw = 0          # 收到的 PID_TUNING 原始条数(含无效模式)
        self.n_recv_err = 0         # pymavlink 内部容错丢包的次数
        self.speed_rec = None       # 最近一条速度环(axis=4)采样
        # 航段记录: 只在有效模式下采集, 模式切走时自动收尾成一段
        self.seg_start = None
        self.seg_total = 0
        self.last_seg = None        # (idx, path, body) 供界面显示
        self.vel = []               # [(t, vx, vy)]  LOCAL_POSITION_NED (m/s)
        self.gps = []               # [(t, fix_type, sats, eph)]
        self.ekf = []               # [(t, flags, pos_horiz_var, compass_var)]
        self.pos = []               # [(t, lat, lon, alt, vx, vy, hdg)] 全局位置
        self.mcur = []              # [(t, seq)] 当前航点序号
        # 航线(只读下载): [(seq, lat, lon, cmd, p1, p2, p4)]
        self.mission = []
        self.mission_total = None
        self.mission_state = "idle"     # idle/ask/items/done
        self.mission_try = 0
        self.mission_t = 0.0
        self.boost_note = ""
        self._boosted = []
        self.error = None
        self.connected = False
        self.t0 = None
        self._active = {k: False for k in self.buf}
        self._t0 = {k: None for k in self.buf}
        self._sign = {k: 1 for k in self.buf}
        self._pending = []
        self._csv = self._pcsv = None
        self._csvw = self._pcsvw = None
        self._pcsv_flush = 0.0   # 参数 CSV 的节流 flush 时刻
        self._param_i = 0        # 参数轮询游标
        self.params_t = {}       # 每个参数最近一次更新的时刻(用于报告里标"参数多旧")

    # ---------- 生命周期 ----------
    def stop(self):
        """只置标志。真正的收尾(写回速率/关连接)由读取线程自己做,
        避免两个线程同时操作同一个 Mavlink 连接(会损坏 pymavlink 内部缓冲)"""
        self.stop_flag = True

    def _open_csv(self):
        if not self.csv_path:
            return
        try:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self._csv = open(self.csv_path, "w", newline="", encoding="utf-8")
            self._csvw = csv.writer(self._csv)
            head = ["t", "mode", "des", "ach", "P", "I", "D", "FF",
                    "thr", "gs", "yawrate"]
            head += ["s%d" % c for c in self.servo_list]   # s1/s3/...
            head += ["s1_t", "sp_des", "sp_ach", "sp_P", "sp_I", "sp_D", "sp_FF"]
            self._csvw.writerow(head)
            self._csv.flush()
            self._pcsv = open(self.csv_path[:-4] + ".params.csv", "w", newline="",
                              encoding="utf-8")
            self._pcsvw = csv.writer(self._pcsv)
            self._pcsvw.writerow(["t", "name", "value"])
            self._pcsv.flush()
        except Exception as e:
            errlog("open csv: %r" % (e,))

    def close_csv(self):
        for h in (self._csv, self._pcsv):
            try:
                if h:
                    h.close()
            except Exception:
                pass

    # ---------- 缓冲 ----------
    def _trim(self, key):
        b = self.buf[key]
        if not b:
            return
        cutoff = b[-1][0] - BUF_AGE
        i = 0
        while i < len(b) and b[i][0] < cutoff:
            i += 1
        if i:
            del b[:i]

    def tail(self, key, t_from):
        """只取 buf[key] 中 t >= t_from 的样本(新列表)。O(logN+k)
        代替 list(整个 600s 缓冲) —— 这是界面越跑越卡的主因。"""
        b = self.buf.get(key)
        return _tail_from(b, t_from) if b else []

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
        if not self._pending:
            return
        # 动作测量需要 s1/s3 缓冲；配置不含时跳过（不崩溃）
        if not self.buf.get("s1") or not self.buf.get("s3"):
            self._pending = []
            return
        last_out = self.buf["s1"][-1][0]
        keep = []
        for p in self._pending:
            _key, t0, t1, _sign = p
            if last_out >= t1 + 0.8:
                try:
                    with self.lock:
                        # 动作测量只需要 t0-0.8 之后的数据, 不再复制整段缓冲
                        rc = self.tail(p[0], t0 - 1.0)
                        a = self.tail("s1", t0 - 1.0)
                        b = self.tail("s3", t0 - 1.0)
                    it = measure_action(rc, a, b, t0, t1, p[3], self.cfg)
                except Exception:
                    it = None
                if it is not None:
                    it["stick"] = p[0]
                    self.out_q.put(it)
            else:
                keep.append(p)
        self._pending = keep

    # ---------- CSV ----------
    def _log_sample(self, rec):
        if self._csvw is None:
            return
        try:
            t, d, a, p, i, di, ff = rec
            sp = self.speed_rec
            with self.lock:
                gs = self.hud[-1][1] if self.hud else None
                thr = self.hud[-1][2] if self.hud else None
                yr = self.att[-1][1] if self.att else None
                svals = []
                for c in self.servo_list:
                    b = self.buf.get(chan_key("s", c))
                    svals.append(b[-1][1] if b else None)
            row = [
                "%.3f" % (t - self.t0), self.mode if self.mode is not None else -1,
                "%.5f" % d, "%.5f" % a, "%.5f" % p, "%.5f" % i, "%.5f" % di, "%.5f" % ff,
                "" if thr is None else "%.1f" % thr,
                "" if gs is None else "%.3f" % gs,
                "" if yr is None else "%.4f" % yr,
            ]
            row += ["" if v is None else "%d" % v for v in svals]
            row += ["%.2f" % self.s1_t,
                    "" if sp is None else "%.4f" % sp[1],
                    "" if sp is None else "%.4f" % sp[2],
                    "" if sp is None else "%.5f" % sp[3],
                    "" if sp is None else "%.5f" % sp[4],
                    "" if sp is None else "%.5f" % sp[5],
                    "" if sp is None else "%.5f" % sp[6]]
            self._csvw.writerow(row)
            self._csv.flush()
        except Exception as e:
            errlog("csv: %r" % (e,))

    def _log_param(self, name, value):
        if self._pcsvw is None:
            return
        try:
            self._pcsvw.writerow(["%.3f" % (time.time() - self.t0), name, "%.7g" % value])
            # 不再每条都 flush: Windows + 杀软下每秒几十次写盘很贵
            now_m = time.monotonic()
            if now_m - self._pcsv_flush >= 1.0:
                self._pcsv_flush = now_m
                self._pcsv.flush()
        except Exception:
            pass

    # ---------- 流速率提速 ----------
    # 原速率表(断开时写回用)
    ORIG_HZ = {"ATTITUDE": 10, "PID_TUNING": 10, "SERVO_OUTPUT_RAW": 2}

    def _close_segment(self, t_end):
        """一段「有效模式」结束(模式切走)时：快照 + 出报告 + 存盘。
        必须在 pid_hist.clear() 之前调用。跑在读取线程里(只做计算和写文件)。"""
        if self.seg_start is None or not self.pid_valid:
            self.seg_start = None
            return
        t0 = self.seg_start
        self.seg_start = None
        with self.lock:
            d = {
                "hist": {ax: [r for r in h if r[0] >= t0] for ax, h in self.pid_hist.items()},
                "params": dict(self.params), "mode": self.mode,
                "params_age": _params_age(self.params_t),
                "att": [r for r in self.att if r[0] >= t0],
                "hud": [r for r in self.hud if r[0] >= t0],
                "vel": [r for r in self.vel if r[0] >= t0],
                "gps": [r for r in self.gps if r[0] >= t0],
                "ekf": [r for r in self.ekf if r[0] >= t0],
                "pos": [r for r in self.pos if r[0] >= t0],
                "mcur": [r for r in self.mcur if r[0] >= t0],
                "mission": list(self.mission),
                "s1": [r for r in self.buf.get("s1", []) if r[0] >= t0],
                "s3": [r for r in self.buf.get("s3", []) if r[0] >= t0],
                "servo": {chan_key("s", c):
                          [r for r in self.buf.get(chan_key("s", c), []) if r[0] >= t0]
                          for c in self.servo_list},
                "rc": {chan_key("rc", c):
                       [r for r in self.buf.get(chan_key("rc", c), []) if r[0] >= t0]
                       for c in self.rc_list},
            }
        if sum(len(v) for v in d["hist"].values()) < 60:
            return                      # 太短，不值得留
        self.seg_total += 1
        body, path, nnn = save_segment(self.mode, t0, t_end, d,
                                       seg_dir=get_seg_dir(self.cfg))
        self.last_seg = (nnn, path, body)

    # ---------- 航线下载(只读，不修改飞控) ----------
    def _mission_ask(self):
        if self.mission_try >= 4:
            self.mission_state = "done"
            return
        self.mission_try += 1
        self.mission_state = "ask"
        self.mission_t = time.time()
        try:
            self.m.mav.mission_request_list_send(1, 1, 0)
        except Exception as e:
            errlog("mission_request_list: %r" % (e,))

    def _mission_next(self, i):
        self.mission_t = time.time()
        try:
            self.m.mav.mission_request_int_send(1, 1, i, 0)
        except Exception as e:
            errlog("mission_request_int: %r" % (e,))

    def _set_interval(self, msg_id, us):
        try:
            self.m.mav.command_long_send(
                1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, us, 0, 0, 0, 0, 0)
        except Exception as e:
            errlog("set_interval(%s): %r" % (msg_id, e))

    def _boost_rates(self):
        """提高关键消息流速率。纯运行时请求，不改任何持久参数。
        ATTITUDE         -> 10Hz  (单位判定用; 也防止之前被 -1 关掉后不恢复)
        PID_TUNING       -> 20Hz  (RAT 诊断分辨率翻倍)
        SERVO_OUTPUT_RAW -> 10Hz  (差速输出不再是瓶颈)"""
        if mavutil is None or not self.cfg.get("boost_rates", True):
            return
        C = mavutil.mavlink
        self._boosted = []
        notes = []
        plan = (("ATTITUDE", "att_hz", 10),
                ("PID_TUNING", "pid_hz", 20),
                ("SERVO_OUTPUT_RAW", "servo_hz", 10))
        for name, key, dflt in plan:
            mid = getattr(C, "MAVLINK_MSG_ID_" + name, None)
            if mid is None:
                continue
            try:
                hz = float(self.cfg.get(key, dflt))
            except Exception:
                hz = dflt
            if hz <= 0:
                continue
            self._set_interval(mid, int(1000000.0 / hz))
            # 记住原速率, 断开时写回(不能用 -1!)
            self._boosted.append((mid, self.ORIG_HZ.get(name, hz)))
            notes.append("%s→%gHz" % (name, hz))
        self.boost_note = ("已提速 " + ", ".join(notes)) if notes else ""

    def _restore_rates(self):
        """写回原速率。实测本飞控 -1 = 停止发送, 所以只能显式写回。"""
        for mid, hz in list(self._boosted or []):
            self._set_interval(mid, int(1000000.0 / hz))
        self._boosted = []

    # ---------- 主循环 ----------
    def run(self):
        if mavutil is None:
            self.error = "缺少 pymavlink"
            return
        try:
            dev = self.dev
            if dev.lower().startswith("tcp:") or dev.lower().startswith("udp"):
                self.m = mavutil.mavlink_connection(dev, timeout=6)
            elif ":" in dev and not dev.upper().startswith("COM"):
                self.m = mavutil.mavlink_connection("tcp:" + dev, timeout=6)
            else:
                self.m = mavutil.mavlink_connection(dev, baud=115200, timeout=2)
            self.connected = True
        except Exception as e:
            self.error = "连接 %s 失败: %s" % (self.dev, e)
            return
        self.t0 = time.time()
        self._open_csv()
        self._boost_rates()
        self._mission_ask()
        t_param = 0.0
        while not self.stop_flag:
            now = time.time()
            if self.mission_state == "ask" and now - self.mission_t > 3.0:
                self._mission_ask()       # 未收到 MISSION_COUNT -> 重试
            if now - t_param >= 1.0:
                t_param = now
                # 轮转读取: 原来一口气发 28 条, 每秒被回包 28 次 PARAM_VALUE
                nw = len(WATCH)
                for _k in range(min(PARAM_PER_SEC, nw)):
                    pnm = WATCH[self._param_i % nw]
                    self._param_i += 1
                    try:
                        self.m.mav.param_request_read_send(1, 1, pnm.encode(), -1)
                    except Exception:
                        pass
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                if not self.stop_flag:
                    es = str(e)
                    if "item assignment" in es or "_instances" in es:
                        # pymavlink 内部小毛病：丢一条消息就行，不要断连接
                        self.n_recv_err += 1
                        continue
                    self.error = "连接中断: %s" % es
                    errlog("run 异常: %r" % (e,))
                    try:
                        import traceback
                        errlog(traceback.format_exc())
                    except Exception:
                        pass
                break
            if msg is None:
                continue
            t = getattr(msg, "_timestamp", 0.0) or time.time()
            ty = msg.get_type()
            with self.lock:
                if ty == "RC_CHANNELS":
                    for c in self.rc_list:
                        v = getattr(msg, "chan%d_raw" % c, None)
                        if v is None:
                            continue
                        k = chan_key("rc", c)
                        self.buf[k].append((t, v))
                        self._update_state(k, t, v)
                        self._trim(k)
                elif ty == "SERVO_OUTPUT_RAW":
                    for c in self.servo_list:
                        v = getattr(msg, "servo%d_raw" % c, None)
                        if v is None:
                            continue
                        k = chan_key("s", c)
                        self.buf[k].append((t, v))
                    self.s1_t = t - self.t0
                    for c in self.servo_list:
                        self._trim(chan_key("s", c))
                elif ty == "PID_TUNING":
                    self.n_pid_raw += 1
                    # MANUAL 等模式飞控不跑转向速率控制器, 报文是上一次留下的旧值 -> 不采集
                    if self.pid_valid:
                        ax = getattr(msg, "axis", -1)
                        rec = (t, float(getattr(msg, "desired", 0.0)),
                               float(getattr(msg, "achieved", 0.0)),
                               float(getattr(msg, "P", 0.0)), float(getattr(msg, "I", 0.0)),
                               float(getattr(msg, "D", 0.0)), float(getattr(msg, "FF", 0.0)))
                        self.pid_hist.setdefault(ax, []).append(rec)
                        h = self.pid_hist[ax]
                        if h and t - h[0][0] > MAX_AGE:
                            cut = t - MAX_AGE
                            k = 0
                            while k < len(h) and h[k][0] < cut:
                                k += 1
                            if k:
                                del h[:k]
                        if ax == PID_AXIS:
                            self._log_sample(rec)
                        elif ax == SPEED_AXIS:
                            self.speed_rec = rec
                elif ty == "ATTITUDE":
                    self.att.append((t, float(getattr(msg, "yawspeed", 0.0)),
                                     float(getattr(msg, "roll", 0.0)),
                                     float(getattr(msg, "pitch", 0.0)),
                                     float(getattr(msg, "yaw", 0.0))))
                    if len(self.att) > 20000:
                        del self.att[:10000]
                elif ty == "VFR_HUD":
                    self.hud.append((t, float(msg.groundspeed), float(msg.throttle)))
                    if len(self.hud) > 20000:
                        del self.hud[:10000]
                elif ty == "LOCAL_POSITION_NED":
                    self.vel.append((t, float(msg.vx), float(msg.vy)))
                    if len(self.vel) > 20000:
                        del self.vel[:10000]
                elif ty == "GPS_RAW_INT":
                    _eph = float(getattr(msg, "eph", 65535))
                    self.gps.append((t, int(getattr(msg, "fix_type", 0)),
                                     int(getattr(msg, "satellites_visible", 0)),
                                     -1.0 if _eph >= 65535 else _eph))
                    if len(self.gps) > 5000:
                        del self.gps[:2000]
                elif ty == "EKF_STATUS_REPORT":
                    self.ekf.append((t, int(getattr(msg, "flags", 0)),
                                     float(getattr(msg, "pos_horiz_variance", -1)),
                                     float(getattr(msg, "compass_variance", -1))))
                    if len(self.ekf) > 5000:
                        del self.ekf[:2000]
                elif ty == "HEARTBEAT":
                    cm = getattr(msg, "custom_mode", None)
                    if cm is not None:
                        if cm != self.mode:
                            # 先给上一段有效数据收尾(必须在 clear 之前快照)
                            try:
                                self._close_segment(t)
                            except Exception as e:
                                errlog("close_segment: %r" % (e,))
                            self.pid_hist.clear()
                        self.mode = cm
                        self.pid_valid = (cm in PID_ACTIVE_MODES or
                                          bool(self.cfg.get("allow_any_mode")))
                        if self.pid_valid and self.seg_start is None:
                            self.seg_start = t
                    self.n_hb += 1
                elif ty == "PARAM_VALUE":
                    nm = msg.param_id
                    if not isinstance(nm, str):
                        nm = nm.decode(errors="replace")
                    nm = nm.replace("\x00", "").strip()
                    if nm in WATCH:
                        v = float(msg.param_value)
                        old = self.params.get(nm)
                        self.params[nm] = v
                        self.params_t[nm] = time.time()
                        changed = (old is None or
                                   abs(v - old) > max(1e-6, abs(old) * 1e-5))
                        if changed:
                            # 只在首次/变化时落盘: 否则 28 个参数x1Hz 会持续写盘,
                            # 参数 CSV 也会无限膨胀
                            self._log_param(nm, v)
                        if old is not None and changed:
                            self.param_changes.append((time.time(), nm, old, v))
                            if len(self.param_changes) > 500:
                                del self.param_changes[:200]
                elif ty == "GLOBAL_POSITION_INT":
                    self.pos.append((t, msg.lat / 1e7, msg.lon / 1e7,
                                     msg.relative_alt / 1000.0,
                                     msg.vx / 100.0, msg.vy / 100.0,
                                     msg.hdg / 100.0))
                    if len(self.pos) > 6000:
                        del self.pos[:3000]
                elif ty == "MISSION_CURRENT":
                    sq = int(getattr(msg, "seq", -1))
                    if sq >= 0:
                        self.mcur.append((t, sq))
                        if len(self.mcur) > 4000:
                            del self.mcur[:2000]
                        # 航线可能被改过 -> 重新下载
                        if self.mission_state == "done" and sq + 1 > len(self.mission):
                            self.mission_try = 0
                            self._mission_ask()
                elif ty == "MISSION_COUNT":
                    n = int(getattr(msg, "count", 0))
                    self.mission = []
                    self.mission_total = n
                    if n > 0:
                        self.mission_state = "items"
                        self._mission_next(0)
                    else:
                        self.mission_state = "done"
                elif ty in ("MISSION_ITEM_INT", "MISSION_ITEM"):
                    try:
                        if ty == "MISSION_ITEM_INT":
                            la, lo = msg.x / 1e7, msg.y / 1e7
                        else:
                            la, lo = float(msg.x), float(msg.y)
                        sq = int(msg.seq)
                        self.mission = [mm for mm in self.mission if mm[0] != sq]
                        self.mission.append((sq, la, lo, int(getattr(msg, "command", 0)),
                                             float(getattr(msg, "param1", 0.0)),
                                             float(getattr(msg, "param2", 0.0)),
                                             float(getattr(msg, "param4", 0.0))))
                        self.mission.sort()
                    except Exception as e:
                        errlog("mission item: %r" % (e,))
                    if (self.mission_total is not None and
                            len(self.mission) < self.mission_total):
                        self._mission_next(len(self.mission))
                    else:
                        self.mission_state = "done"
                        self.mission_t = time.time()
                elif ty == "MISSION_ACK":
                    if self.mission_state != "items":
                        self.mission_state = "done"
                self._flush_pending()
            if self.stop_flag:
                break
        # 收尾：全部在读取线程内做
        try:
            self._restore_rates()
        except Exception as e:
            errlog("restore_rates: %r" % (e,))
        try:
            self.m.close()
        except Exception:
            pass


# ============================================================ 打杆动作测量
def detect_events(chan, thresh=120):
    events = []
    i, n = 0, len(chan)
    while i < n - 1:
        t, v = chan[i]
        if abs(v - 1500) <= thresh:
            i += 1
            continue
        sign = 1 if v > 1500 else -1
        j = i
        while j < n - 1 and abs(chan[j][1] - 1500) > thresh * 0.5:
            j += 1
        events.append((chan[i][0], chan[j][0], sign, 0, 0))
        i = j
    return events


def measure_action(chan, outA, outB, t0, t1, sign, cfg):
    """量一次动作：OUT 启动延迟 + 第一段连续爬升时间 + 目标 PWM + 摆幅"""
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
    win_lo = max(t0 - 0.3, in_start)

    best = None
    for name, out in (("A", outA), ("B", outB)):
        win = [r for r in out if win_lo <= r[0] <= t1 + 0.8]
        if not win:
            continue
        rise_m = max(0.0, max((v - 1500) for _, v in win))
        fall_m = max(0.0, -min((v - 1500) for _, v in win))
        if best is None or (rise_m, fall_m) > best[0]:
            best = ((rise_m, fall_m), name, win)
    if best is None:
        return None
    (rise_m, fall_m), oname, win = best
    if max(rise_m, fall_m) <= 30:
        return None
    i0 = next((i for i, r in enumerate(win) if abs(r[1] - 1500) > 30), None)
    if i0 is None:
        return None
    t_start = win[i0][0]
    peak_i = i0
    peak_exc = abs(win[i0][1] - 1500)
    for i in range(i0 + 1, len(win)):
        exc = abs(win[i][1] - 1500)
        if exc > peak_exc:
            peak_exc, peak_i = exc, i
        else:
            break
    t_out = win[peak_i][0]

    def _plateau(out):
        w = [r for r in out if win_lo <= r[0] <= t1 + 0.8]
        if not w:
            return None
        k = next((i for i, r in enumerate(w) if abs(r[1] - 1500) > 30), None)
        if k is None:
            return w[-1][1]
        pk, ex = k, abs(w[k][1] - 1500)
        for i in range(k + 1, len(w)):
            e = abs(w[i][1] - 1500)
            if e > ex:
                pk, ex = i, e
            else:
                break
        return w[pk][1]

    return {
        "t0": t_start, "lat": max(0.0, t_start - in_start), "t1": t_out, "sign": sign,
        "rise": t_out - t_start, "target": win[peak_i][1], "swing": peak_exc,
        "out_chan": oname, "s1_end": _plateau(outA), "s3_end": _plateau(outB),
    }


# ============================================================ 波形控件
class ZoomPlot(tk.Canvas):
    """极简折线图，支持共享时间窗缩放（以鼠标位置为中心）"""

    def __init__(self, master, height=120, **kw):
        kw.setdefault("bg", "white")
        kw.setdefault("highlightthickness", 1)
        kw.setdefault("highlightbackground", "#ccc")
        super().__init__(master, height=height, **kw)
        self._series = []
        self._colors = ("#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd")
        self._ylabel = ""
        self._title = ""
        self._legend = ""
        self._zoom = None
        self._view = None
        self._zero = 0.0
        self._rd_pending = False
        # 尺寸变化(首次布局 / 切换页签)后补一次重绘。
        # 不能用"尺寸不合法就 after 自我续期": 未显示的图宽高为 1, 那样每轮
        # set_series 都会再堆一条定时器, 越跑越卡。
        self.bind("<Configure>", self._on_configure)

    def _on_configure(self, _ev=None):
        self._defer_redraw(1)

    def _defer_redraw(self, delay=60):
        """排一次延后重绘; 同时最多只有一条待处理定时器。
        delay>1 时要求控件真的可见, 否则隐藏页签上的图会无限堆积链。"""
        if self._rd_pending:
            return
        if delay > 1 and not self.winfo_ismapped():
            return
        self._rd_pending = True
        self.after(delay, self._deferred_redraw)

    def _deferred_redraw(self):
        self._rd_pending = False
        self.redraw()

    def set_series(self, series, colors=None, ylabel=""):
        self._series = series
        if colors:
            self._colors = colors
        self._ylabel = ylabel
        self.redraw()

    def set_title(self, title):
        self._title = title
        self.redraw()

    def set_zero(self, z):
        self._zero = z
        self.redraw()

    def set_zoom(self, lo, hi):
        self._zoom = (lo, hi)
        self.redraw()

    def reset_zoom(self):
        self._zoom = None
        self.redraw()

    def pixel_to_x(self, xpx):
        if self._view is None:
            return 0.0
        x0, x1, pad_l, pw = self._view
        if pw <= 5:
            return x0
        return x0 + (x1 - x0) * ((xpx - pad_l) / pw)

    def redraw(self):
        self.delete("all")
        W = max(self.winfo_width(), 60)
        H = max(self.winfo_height(), 40)
        pad_l, pad_r, pad_t, pad_b = 46, 10, 26, 22
        pw, ph = W - pad_l - pad_r, H - pad_t - pad_b
        if pw <= 5 or ph <= 5:
            self._defer_redraw(60)
            return
        # x 范围只看每条序列的首尾(序列按时间有序), 不再先建两份冗余列表再 min/max
        amin = amax = None
        for _n, pts in self._series:
            if not pts:
                continue
            if amin is None or pts[0][0] < amin:
                amin = pts[0][0]
            if amax is None or pts[-1][0] > amax:
                amax = pts[-1][0]
        if amin is None:
            self.create_text(W / 2, H / 2, text="(无数据)", fill="#999")
            self._view = None
            return
        if amin == amax:
            amax = amin + 1
        x0, x1 = self._zoom if self._zoom else (amin, amax)
        ymin = ymax = None
        for _n, pts in self._series:
            for x, y in pts:
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

        if y0 < self._zero < y1:
            self.create_line(sx(x0), sy(self._zero), sx(x1), sy(self._zero),
                             fill="#bbb", dash=(2, 2))
        self.create_line(pad_l, pad_t, pad_l, pad_t + ph, fill="#333")
        self.create_line(pad_l, pad_t + ph, pad_l + pw, pad_t + ph, fill="#333")
        for i in range(3):
            yy = y0 + (y1 - y0) * i / 2
            self.create_text(pad_l - 5, sy(yy), text="%.3g" % yy, anchor="e",
                             font=("Consolas", 8), fill="#555")
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4
            self.create_text(sx(xx), H - 4, text="%.1f" % xx, anchor="n",
                             font=("Consolas", 8), fill="#555")
        ci = 0
        for _name, pts in self._series:
            col = self._colors[ci % len(self._colors)]
            ci += 1
            seg = [p for p in pts if x0 <= p[0] <= x1]
            if len(seg) > 1:
                seg = _decimate(seg)
                coords = []
                for x, y in seg:
                    coords += [sx(x), sy(y)]
                self.create_line(*coords, fill=col, width=1)
            elif len(seg) == 1:
                x, y = seg[0]
                self.create_oval(sx(x) - 2, sy(y) - 2, sx(x) + 2, sy(y) + 2, fill=col)
        if self._ylabel:
            self.create_text(pad_l + 6, pad_t + ph / 2, text=self._ylabel, anchor="center",
                             angle=90, font=("Microsoft YaHei", 8), fill="#444")
        if self._title:
            self.create_text(pad_l + 2, 7, text=self._title, anchor="w",
                             font=("Microsoft YaHei", 9), fill="#111")
        if self._legend:
            self.create_text(pad_l + pw - 4, 7, text=self._legend, anchor="ne",
                             font=("Microsoft YaHei", 8), fill="#888")
        if self._zoom:
            self.create_text(pad_l + pw - 4, pad_t - 1, text="已放大(双击复位)", anchor="ne",
                             font=("Microsoft YaHei", 8), fill="#f90")


# ============================================================ 诊断计算
def _tail_from(seq, t_from):
    """seq 按时间有序, 返回 t >= t_from 的切片。
    用二分定位 + 切片, 复杂度 O(logN + k), 代替 [r for r in seq if ...]
    的全量扫描 —— 序列越长扫描越慢, 这正是界面越跑越卡的原因。
    返回的是新列表(切片), 调用方可在锁外安全使用。"""
    if not seq:
        return []
    lo, hi = 0, len(seq)
    while lo < hi:
        mid = (lo + hi) // 2
        if seq[mid][0] < t_from:
            lo = mid + 1
        else:
            hi = mid
    return seq[lo:]


def _decimate(seg, max_pts=PLOT_MAX_PTS):
    """等分桶取 min/max 两点, 保留包络地把曲线点数降到 ~2*max_pts。
    点数不设上限时 create_line 会带着几千个坐标, Tk 重绘极慢。"""
    n = len(seg)
    if n <= max_pts * 2:
        return seg
    step = n / float(max_pts)
    out = []
    for b in range(max_pts):
        i0 = int(b * step)
        i1 = min(n, int((b + 1) * step))
        if i1 <= i0:
            continue
        lo = hi = i0
        for i in range(i0 + 1, i1):
            y = seg[i][1]
            if y < seg[lo][1]:
                lo = i
            elif y > seg[hi][1]:
                hi = i
        if lo == hi:
            out.append(seg[lo])
        elif lo < hi:
            out.append(seg[lo])
            out.append(seg[hi])
        else:
            out.append(seg[hi])
            out.append(seg[lo])
    return out


def _stats(xs):
    n = len(xs)
    m = sum(xs) / n
    return {"n": n, "mean": m, "rms": math.sqrt(sum(x * x for x in xs) / n),
            "max": max(xs), "min": min(xs)}


def _resample(pts, t0, t1, dt=0.05):
    pts = sorted([p for p in pts if p[0] is not None])
    if len(pts) < 2:
        return []
    grid, k, tg = [], 0, t0
    while tg <= t1:
        while k + 2 < len(pts) and pts[k + 1][0] < tg:
            k += 1
        a = pts[k]
        b = pts[k + 1] if k + 1 < len(pts) else pts[k]
        if b[0] - a[0] < 1e-9:
            v = b[1]
        else:
            f = max(0.0, min(1.0, (tg - a[0]) / (b[0] - a[0])))
            v = a[1] + (b[1] - a[1]) * f
        grid.append((tg, v))
        tg += dt
    return grid


def _lag(g1, g2, dt, max_lag_s=2.0):
    n = min(len(g1), len(g2))
    if n < 20:
        return None, None
    a = [g1[i][1] for i in range(n)]
    b = [g2[i][1] for i in range(n)]
    ma, mb = sum(a) / n, sum(b) / n
    a = [x - ma for x in a]
    b = [x - mb for x in b]
    va, vb = sum(x * x for x in a), sum(x * x for x in b)
    if va < 1e-12 or vb < 1e-12:
        return None, None
    best, best_c = 0, -2.0
    for lg in range(0, int(max_lag_s / dt) + 1):
        if n - lg < 20:
            break
        c = sum(a[i] * b[i + lg] for i in range(n - lg)) / math.sqrt(va * vb)
        if c > best_c:
            best_c, best = c, lg
    return best * dt, best_c


def _zcf(vals, dt):
    if len(vals) < 8:
        return None, 0
    m = sum(vals) / len(vals)
    z = sum(1 for i in range(1, len(vals)) if (vals[i - 1] - m) * (vals[i] - m) < 0)
    dur = len(vals) * dt
    return (z / 2.0 / dur) if dur > 0 else None, z


# ---------------------------------------------------------------- 导航几何
MISSION_CMD = {16: "WAYPOINT", 17: "LOITER", 18: "LOITER_TURNS", 19: "LOITER_TIME",
               20: "RTL", 21: "LAND", 22: "TAKEOFF", 177: "DO_JUMP"}


def _bearing_deg(lat1, lon1, lat2, lon2):
    """A->B 初始方位角(deg, 0=北, 顺时针)"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _dist_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def _leg_lengths(mission):
    """相邻航点之间的距离(m), 用于把摆动波长换算成"每航段几次起伏"""
    pts = [(m[1], m[2]) for m in (mission or []) if len(m) > 2]
    out = []
    for a, b in zip(pts, pts[1:]):
        d = _dist_m(a[0], a[1], b[0], b[1])
        if d > 0.5:
            out.append(d)
    return out


def _cross_m(lat, lon, latA, lonA, latB, lonB):
    """点到 A->B 线段的偏差(m)。返回 (|垂距|, 沿线距离, 有向垂距[正=航线右侧])"""
    kx = 111320.0 * math.cos(math.radians(latA))
    ky = 110540.0
    bx, by = (lonB - lonA) * kx, (latB - latA) * ky
    px, py = (lon - lonA) * kx, (lat - latA) * ky
    L = math.hypot(bx, by)
    if L < 1e-6:
        return _dist_m(lat, lon, latA, lonA), 0.0, 0.0
    ux, uy = bx / L, by / L
    return abs(px * uy - py * ux), px * ux + py * uy, px * uy - py * ux


def _angp_regress(rows, pid, amax, win_s=2.5, hop_s=1.25, r2_min=0.90,
                  min_n=12, min_span=3.0, sat=0.92):
    """滑动窗 + 过原点线性回归反算 ATC_STR_ANG_P（数据驱动, 不靠固定阈值）。

    只有飞控真的走「期望转向率 = ANG_P x 航向误差」这条路径(AR_PivotTurn 原地转向)
    时, 两者才严格成比例; 航线巡航的期望转向率由位置环给出, 与航向误差无关,
    拟合 R2 自然很差而被淘汰。因此 R2 门槛会自动挑出可用的那段数据。

    返回 (中位斜率, 中位R2, 合格窗数, 总窗数, 斜率标准差, 斜率列表) 或 None
    """
    ser, pi_ = [], 0
    for (tt, he, _x, _c, _ds, _sq) in rows:
        while pi_ + 1 < len(pid) and pid[pi_ + 1][0] <= tt:
            pi_ += 1
        if not pid or abs(pid[pi_][0] - tt) > 0.15:
            continue
        des = pid[pi_][1]
        if amax and abs(des) > sat * amax:
            continue                    # 被 ATC_STR_RAT_MAX 限幅, 不是比例关系
        ser.append((tt, he, des))
    if len(ser) < min_n * 2:
        return None
    ser.sort()
    t, t1 = ser[0][0], ser[-1][0]
    ks, r2s, nw, nt = [], [], 0, 0
    while t + win_s <= t1:
        w = [r for r in ser if t <= r[0] < t + win_s]
        t += hop_s
        nt += 1
        if len(w) < min_n:
            continue
        xs = [r[1] for r in w]
        ys = [r[2] for r in w]
        if max(xs) - min(xs) < min_span:
            continue                    # 航向误差几乎不变, 拟合无意义
        sxx = sum(x * x for x in xs)
        if sxx < 1e-9:
            continue
        k = sum(x * y for x, y in zip(xs, ys)) / sxx
        if k <= 0:
            continue                    # 符号不对, 说明不是这条控制路径
        ym = sum(ys) / len(ys)
        ss_tot = sum((y - ym) ** 2 for y in ys)
        ss_res = sum((y - k * x) ** 2 for x, y in zip(xs, ys))
        if ss_tot < 1e-9:
            continue
        r2 = 1.0 - ss_res / ss_tot
        if r2 < r2_min:
            continue
        ks.append(k)
        r2s.append(r2)
        nw += 1
    if not ks:
        return None
    med = sorted(ks)[len(ks) // 2]
    r2m = sorted(r2s)[len(r2s) // 2]
    sd = math.sqrt(sum((x - med) ** 2 for x in ks) / len(ks)) if len(ks) > 1 else 0.0
    return med, r2m, nw, nt, sd, ks


def _turn_events(rows, add, pre=15.0, post=15.0, band=5.0):
    """过弯事件分析。rows = [(t, he, xt, csign, dist, sq)]

    用航点序号变化定位"过了哪个航点"，逐次看航迹误差怎么变。
    关键区分: 弯前基线逐次抬高 = 累积性劣化(有状态在堆积);
              每次都回到同一基线 = 只是弯后暂态, 不累积。
    """
    # 只有"上一个航点至少保持了 min_hold 秒"才算一次真正的过弯,
    # 否则报文抖动/重发会产生一大堆假事件
    min_hold = 5.0
    ev, prev, prev_t = [], None, None
    for (tt, _he, _xt, _c, _d, sq) in rows:
        if prev is None:
            prev, prev_t = sq, tt
            continue
        if sq != prev:
            if tt - prev_t >= min_hold:
                ev.append(tt)
            prev, prev_t = sq, tt
    add("")
    add("[13] 过弯事件分析")
    if not ev:
        add("    本段没有记录到航线点切换(可能整段都在同一航段上)")
        return
    add("    共 %d 次航线点切换" % len(ev))
    add("    弯#   时刻    弯前%.0fs基线  弯中±%.0fs峰值  弯后%.0fs  恢复用时"
        % (pre, band, post))
    bases = []
    for k, tt in enumerate(ev, 1):
        pw = [r[2] for r in rows if tt - pre <= r[0] < tt - 2.0]
        mw = [r[2] for r in rows if tt - band <= r[0] <= tt + band]
        ow = [r[2] for r in rows if tt + 2.0 < r[0] <= tt + post]
        if len(pw) < 3 or len(mw) < 3:
            continue
        pb = sum(pw) / len(pw)
        pk = max(mw)
        po = (sum(ow) / len(ow)) if ow else None
        thr = max(pb * 1.5, 1.0)
        rec = None
        for i, r in enumerate(rows):
            if r[0] <= tt + band or r[2] > thr:
                continue
            seg = [q for q in rows[i:i + 8] if q[0] <= r[0] + 3.0]
            if len(seg) >= 2 and all(q[2] <= thr for q in seg):
                rec = r[0] - tt
                break
        bases.append(pb)
        if k <= 20:
            add("    %3d  %6.0fs  %10.2f m  %12.2f m  %8s  %s"
                % (k, tt - rows[0][0], pb, pk,
                   ("%.2f m" % po) if po is not None else "   --",
                   (("%.0f s" % rec) if rec is not None else ">%.0f s 未恢复" % post)
                   + ("   <- 段初, 可能含切入收敛" if k == 1 else "")),
                "warn" if rec is None else "")
    if len(ev) > 20:
        add("    ... 另有 %d 次未列出" % (len(ev) - 20))
    # 第 1 个过弯通常混了"刚切入 AUTO 的初始收敛", 趋势要用后面的
    tb = bases[1:] if len(bases) >= 4 else bases
    if len(tb) >= 3:
        show = tb if len(tb) <= 12 else tb[:6] + ["..."] + tb[-5:]
        add("    弯前基线序列%s: %s"
            % ("(已排除第1个)" if tb is not bases else "",
               "  ".join(x if isinstance(x, str) else "%.2f" % x for x in show)))
        d0, d1 = tb[0], tb[-1]
        rel = (d1 - d0) / max(d0, 0.30)
        if rel > 0.4:
            add("      -> 基线逐次抬高 (%.2f -> %.2f m): 【累积性劣化】,"
                % (d0, d1), "bad")
            add("         每次过弯都把误差留给下一段 -> 先查积分堆积/估计器,"
                " 而不是单纯调增益", "bad")
        elif rel < -0.4:
            add("      -> 基线逐次下降 (%.2f -> %.2f m): 越来越收敛"
                % (d0, d1), "ok")
        else:
            add("      -> 基线基本稳定 (%.2f -> %.2f m): 过弯只是暂态, 不累积"
                % (d0, d1), "warn" if max(tb) > 2.0 else "ok")


def _nav_block(d, t_start, t_end, params, add, frozen=False):
    """[10] 导航误差（航线跟踪）+ 反算外环 ATC_STR_ANG_P"""
    mission = d.get("mission") or []
    pos = [p for p in (d.get("pos") or []) if t_start <= p[0] <= t_end]
    mcur = d.get("mcur") or []
    pid = [r for r in (d["hist"].get(PID_AXIS) or []) if t_start <= r[0] <= t_end]
    att = d.get("att") or []
    add("")
    add("[10] 导航误差（航线跟踪 / 外环）")
    if not mission:
        add("    没拿到航线(飞控里无航点, 或下载失败) -> 无法评估航迹误差", "warn")
        return
    if not pos:
        add("    没收到位置(GLOBAL_POSITION_INT) -> 无法评估航迹误差", "warn")
        return
    add("    航线 %d 点: %s"
        % (len(mission), "  ".join("#%d%s" % (m[0], MISSION_CMD.get(m[3], "/%d" % m[3]))
                                    for m in mission[:10])))

    def seq_at(tt):
        s = None
        for mt, sq in mcur:
            if mt <= tt:
                s = sq
            else:
                break
        return s

    rows, ai, sqs = [], 0, set()
    for (tt, la, lo, _alt, _vx, _vy, _hdg) in pos:
        while ai + 1 < len(att) and att[ai + 1][0] <= tt:
            ai += 1
        if not att or abs(att[ai][0] - tt) > 0.5:
            continue
        sq = seq_at(tt)
        if sq is None or sq < 1 or sq >= len(mission):
            continue
        A, B = mission[sq - 1], mission[sq]
        yaw = math.degrees(att[ai][4])
        he = (_bearing_deg(la, lo, B[1], B[2]) - yaw + 540.0) % 360.0 - 180.0
        xt, _along, csign = _cross_m(la, lo, A[1], A[2], B[1], B[2])
        rows.append((tt, he, xt, csign, _dist_m(la, lo, B[1], B[2]), sq))
        sqs.add(sq)
    if not rows:
        add("    有航线但本段没有「位置+姿态+航点」的有效配对样本", "warn")
        return
    hes = [r[1] for r in rows]
    xts = [r[2] for r in rows]
    csg = [r[3] for r in rows]
    dst = [r[4] for r in rows]
    add("    本段涉及航点: %s   (n=%d)"
        % (", ".join("#%d" % s for s in sorted(sqs)), len(rows)))
    add("    航向误差    mean=%+7.2f°  RMS=%6.2f°  峰值=%6.2f°"
        % (sum(hes) / len(hes), _stats(hes)["rms"], max(abs(min(hes)), abs(max(hes)))))
    add("    横向航迹误差 mean=%6.2f m  RMS=%6.2f m  峰值=%6.2f m"
        % (sum(xts) / len(xts), _stats(xts)["rms"], max(xts)),
        "warn" if max(xts) > 5 else "ok")
    ms = sum(csg) / len(csg)
    add("    平均偏向    %s %.2f m   (正=航线右侧)"
        % ("右侧" if ms > 0 else "左侧", abs(ms)))
    add("    到目标航点距离 mean=%.1f m  末值=%.1f m" % (sum(dst) / len(dst), dst[-1]))

    # 每次过弯前后航迹误差怎么变 —— 区分"弯后暂态"和"累积性劣化"
    try:
        _turn_events(rows, add)
    except Exception as e:
        errlog("turn_events: %r" % (e,))

    amax = params.get("ATC_STR_RAT_MAX")
    if frozen:
        add("    反算 ANG_P: 本段 PID 是冻结旧值, 不做反算(需真实运动数据)", "warn")
        return
    est = _angp_regress(rows, pid, amax)
    if est is None:
        add("    反算 ANG_P: 不适用 (未找到比例段)", "warn")
        add("    原因: 航线巡航的期望转向率由位置环算出, 不读 ATC_STR_ANG_P;", "warn")
        add("          只有 pivot 原地转向才走 ANG_P, 那时的数据才会严格成比例。", "warn")
        return
    k, r2m, nw, nt, sd, ks = est
    p = params.get("ATC_STR_ANG_P")
    line = ("    反算 ANG_P = %.5f   (合格窗口 %d/%d, 中位 R2=%.3f, 窗间 sigma=%.4f)"
            % (k, nw, nt, r2m, sd))
    if p:
        line += "   参数(ATC_STR_ANG_P)=%.6g  比值=%.3f" % (p, k / p)
        if abs(k / p - 1.0) > 0.15:
            line += "   <== 不符!"
    add(line, "bad" if "不符" in line else "ok")
    add("    依据: 2.5s 滑窗内 desired 与航向误差过原点线性相关 且 R2>0.9")
    add("    各合格窗口斜率: %s" % " ".join("%.3f" % x for x in ks[:12]))


def _params_age(params_t):
    """最旧的那个监视参数距现在多少秒。报告里要标出来 ——
    否则会把"几秒前读到的值"当成当时的真值来判读, 容易得出错误结论。"""
    if not params_t:
        return None
    try:
        return max(0.0, time.time() - min(params_t.values()))
    except Exception:
        return None


def collect_bundle(feed, sec=None):
    """从 LiveFeed 取一份带锁快照，供 compute_diag 使用。"""
    with feed.lock:
        d = {
            "hist": {k: list(v) for k, v in feed.pid_hist.items()},
            "att": list(feed.att), "hud": list(feed.hud),
            "vel": list(feed.vel), "gps": list(feed.gps), "ekf": list(feed.ekf),
            "pos": list(feed.pos), "mcur": list(feed.mcur),
            "mission": list(feed.mission),
            "s1": list(feed.buf.get("s1", [])), "s3": list(feed.buf.get("s3", [])),
            "params": dict(feed.params), "mode": feed.mode,
            "params_age": _params_age(getattr(feed, "params_t", None)),
        }
    return d


def save_segment(mode, t0, t1, d, seg_dir=None):
    """把一段导航数据出成报告并存盘。
    文件名：<YYYYMMDD>_<当日编号>_<模式>_<HHMMSS>.txt
    返回 (报告正文, 文件路径, 编号)"""
    seg_dir = seg_dir or SEG_DIR_DEFAULT
    dur = max(1.0, t1 - t0)
    reps = []
    for ax in (PID_AXIS, SPEED_AXIS):
        try:
            _, rep = compute_diag(d, dur, ax)
            reps.append(rep)
        except Exception as e:
            errlog("seg diag axis=%d: %r" % (ax, e))
    body = "\n\n".join(reps)
    mnm = MODE_NAMES.get(mode, "mode%s" % mode)
    day = time.strftime("%Y%m%d", time.localtime(t0))
    try:
        os.makedirs(seg_dir, exist_ok=True)
    except Exception:
        pass
    nnn = _next_seg_index(seg_dir, day)
    base = "%s_%03d_%s_%s" % (day, nnn, mnm.split()[0],
                               time.strftime("%H%M%S", time.localtime(t0)))
    p = os.path.join(seg_dir, base + ".txt")
    head = ("航段 %s  #%d   模式=%s   起止 %.1f~%.1f s (持续 %.1fs)\n"
            "记录时间 %s\n"
            "本次航线 %s\n%s\n"
            % (day, nnn, mnm, t0, t1, dur,
               time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)),
               "  ".join("#%d%s" % (m[0], MISSION_CMD.get(m[3], "/%d" % m[3]))
                         for m in (d.get("mission") or [])[:10]) or "(无)",
               "=" * 70))
    try:
        with open(p, "w", encoding="utf-8") as f:
            f.write(head + body)
    except Exception as e:
        errlog("write seg: %r" % (e,))
    try:
        with open(os.path.join(seg_dir, "汇总.txt"), "a", encoding="utf-8") as f:
            f.write("%s  #%03d %-9s %6.1fs  样本5=%-5d 样本4=%-5d  %s  -> %s.txt\n"
                    "        参数: %s\n"
                    % (day, nnn, mnm, dur, len(d["hist"].get(PID_AXIS, [])),
                       len(d["hist"].get(SPEED_AXIS, [])),
                       time.strftime("%H:%M:%S", time.localtime(t0)), base,
                       " ".join("%s=%g" % (k, v) for k, v in sorted(d["params"].items())
                                 if k.startswith(("ATC_STR_RAT_FF", "ATC_STR_RAT_P",
                                                  "ATC_STR_RAT_D", "ATC_STR_RAT_IMAX",
                                                  "ATC_STR_RAT_FLTD",
                                                  "ATC_SPEED_P", "ATC_SPEED_I",
                                                  "CRUISE_SPEED", "CRUISE_THROTTLE",
                                                  "WP_SPEED")))))
    except Exception:
        pass
    return body, p, nnn


def compute_diag(d, sec, axis=PID_AXIS):
    """对指定 PID 轴出诊断报告。d = collect_bundle() 或航段快照。
    返回 (lines, report_text)。"""
    prof = AXIS_PROFILE.get(axis, AXIS_PROFILE[PID_AXIS])
    hist = list(d["hist"].get(axis, []))
    att = d["att"]
    hud = d["hud"]
    vel = d["vel"]
    gps = d["gps"]
    ekf = d["ekf"]
    s1 = d.get("s1", []) or []
    s3 = d.get("s3", []) or []
    params = d["params"]
    mode = d["mode"]
    if len(hist) < 20:
        return [("【%s】样本太少(%d 条)。请让船动起来并保持更长时间。"
                 % (prof["title"], len(hist)), "bad")], ""
    t_end = hist[-1][0]
    t_start = max(hist[0][0], t_end - sec)
    win = [r for r in hist if r[0] >= t_start]
    dur = t_end - t_start
    rate = len(win) / dur if dur > 0 else 0
    L = []
    R = []
    is_frozen = False

    def add(txt, tag=""):
        L.append((txt, tag))
        R.append(txt)

    add("=" * 70)
    add(" %s  实时诊断   axis=%d   窗口 %.1fs   样本 %d  (%.2f Hz)"
        % (prof["title"], axis, dur, len(win), rate))
    add("=" * 70)

    # 有效段判据: 船没真在跑时, 后面的跟踪质量/航迹误差全都没有参考价值
    hw_chk = [r for r in hud if r[0] >= t_start]
    if hw_chk:
        gs_chk = sum(r[1] for r in hw_chk) / len(hw_chk)
        th_chk = sum(r[2] for r in hw_chk) / len(hw_chk)
    else:
        gs_chk = th_chk = 0.0
    if hw_chk and (th_chk < IDLE_THR_PCT or gs_chk < IDLE_GS_MPS):
        add("")
        add("  *****  怠速 / 无效段  *****", "bad")
        add("  油门均值 %.1f%%   地速均值 %.2f m/s" % (th_chk, gs_chk), "bad")
        add("  船基本没有航行 -> [1][4][10] 的跟踪与航迹误差指标均无参考价值,", "bad")
        add("  请勿据此调参。请重录一段船真正在跑的航段。", "bad")
        add("")
    if rate < 15.0:
        add("  提示: PID_TUNING 实测 %.1f Hz (<15Hz), 提速未生效, 滞后/振荡估计偏粗"
            % rate, "warn")

    # 冻结检测
    frozen = 0
    for i in (1, 2, 3):
        vs = set(round(r[i], 6) for r in win)
        if len(vs) <= 2:
            frozen += 1
    if frozen >= 3:
        is_frozen = True
        add("")
        add("[!] desired / achieved / P 全程几乎不变", "bad")
        add("    -> 该控制器很可能没在运行, 报文是上一次运行的【旧值】。", "bad")
        add("    -> 请让船真正跑起来 (已解锁 + 有相应指令) 后再看。", "bad")

    # 模式
    if mode is not None:
        add(" 飞行模式 : %s" % MODE_NAMES.get(mode, "模式%d" % mode),
            "bad" if mode == 0 else "ok")
    else:
        add(" 飞行模式 : 未收到 HEARTBEAT", "warn")

    # 单位 / 一致性判定
    unit, kk = prof["out_unit"], 1.0
    to_int = (1.0 / RAD2DEG) if prof["need_div"] else 1.0
    mv = sum(abs(r[2]) for r in win) / len(win)
    if prof["ref"] == "yawrate":
        a_win = [r for r in att if r[0] >= t_start]
        if a_win:
            mr = sum(abs(r[1]) for r in a_win) / len(a_win)      # rad/s
            if mr > 0.002 and mv > 0.01:
                ratio = mv / mr
                if 0.5 < ratio < 2:
                    unit, kk, to_int = "rad/s", RAD2DEG, 1.0
                add(" 单位判定 : 报文 %s   (|achieved|/|yawspeed rad/s| = %.1f)" % (unit, ratio))
    else:
        h_win = [r for r in hud if r[0] >= t_start]
        if h_win:
            mgs = sum(abs(r[1]) for r in h_win) / len(h_win)     # m/s
            if mgs > 0.02 and mv > 0.005:
                ratio = mv / mgs
                bad_axis = not (0.3 < ratio < 3.0)
                add(" 单位判定 : 报文 m/s   (|achieved|/|groundspeed| = %.2f%s)"
                    % (ratio, "  <-- 相差大, 确认 axis 选对没" if bad_axis else ""),
                    "warn" if bad_axis else "")

    des = [r[1] * kk for r in win]
    ach = [r[2] * kk for r in win]
    err = [(r[1] - r[2]) * kk for r in win]
    sd, sa, se = _stats(des), _stats(ach), _stats(err)
    mabs = sum(abs(x) for x in err) / len(err)
    bias = (se["mean"] / mabs) if mabs > 1e-6 else 0.0

    # 限幅饱和占比 + 需求强度 —— 这两个是区分「权限不足」和「增益过高」的关键,
    # 只看误差 RMS 根本分不清"需求自己在摆"和"船跟不上平稳需求"
    sat_lim_name = prof.get("limit")
    sat_lim = params.get(sat_lim_name) if sat_lim_name else None
    if sat_lim:
        n_sat = sum(1 for r in win if abs(r[1] * kk) >= SAT_FRAC * sat_lim)
        sat_pct = 100.0 * n_sat / len(win)
    else:
        n_sat, sat_pct = 0, None
    dabs = sum(abs(x) for x in des) / len(des)

    add("")
    add("[1] 跟踪质量  (单位 %s)" % unit)
    add("    期望%-4s mean=%8.3f  峰值=%8.3f"
        % (prof["what"], sd["mean"], max(abs(sd["max"]), abs(sd["min"]))))
    add("    实际%-4s mean=%8.3f  峰值=%8.3f"
        % (prof["what"], sa["mean"], max(abs(sa["max"]), abs(sa["min"]))))
    add("    跟踪误差   mean=%+8.3f  RMS=%7.3f" % (se["mean"], se["rms"]))
    add("    期望强度   mean|期望|=%7.3f  RMS=%7.3f   (需求自己有多大/是否在摆)"
        % (dabs, sd["rms"]))
    if dabs > 1e-6:
        rr = mabs / dabs
        add("    相对误差   mean|误差|/mean|期望| = %.2f   (>1 = 船跟不上需求)" % rr,
            "bad" if rr > 1.0 else ("warn" if rr > 0.6 else "ok"))
    if sat_pct is not None:
        add("    限幅饱和   %d/%d = %.1f%%  (|期望| >= %.2f x %s=%.6g)"
            % (n_sat, len(win), sat_pct, SAT_FRAC, sat_lim_name, sat_lim),
            "bad" if sat_pct > 5.0 else ("warn" if sat_pct > 1.0 else "ok"))
    add("    稳态偏置度 %+.2f    (>0.6 长期同号=有稳态误差; 接近0=零附近来回)" % bias,
        "warn" if bias > 0.6 else "ok")

    # 分项占比 (rec = (t, desired, achieved, P, I, D, FF))
    _IDX = {"FF": 6, "P": 3, "I": 4, "D": 5}
    tots = {k: sum(abs(r[_IDX[k]]) for r in win) for k in ("FF", "P", "I", "D")}
    grand = sum(tots.values())
    terms = {k: (v / grand if grand > 1e-9 else 0.0) for k, v in tots.items()}
    add("")
    add("[2] 控制输出分项 (占 |总和| 的比例)")
    for k in ("FF", "P", "I", "D"):
        vals = [r[_IDX[k]] for r in win]
        add("    %-2s 均值=%+10.5f   |均值|=%10.5f   占比=%5.1f%%"
            % (k, sum(vals) / len(vals), sum(abs(v) for v in vals) / len(vals),
               terms[k] * 100),
            "warn" if (k == "I" and terms[k] > 0.55) else "")

    # 反算增益
    add("")
    if prof["need_div"]:
        add("[3] 反算有效增益  (报文 %s -> 内部 rad/s, 反算需 /%0.4f)" % (unit, RAD2DEG))
    else:
        add("[3] 反算有效增益  (报文与内部同单位, 直接相除)")
    mismatch = []

    def _gain(vals, pname, label):
        if not vals:
            return
        o = sum(vals) / len(vals)
        p = params.get(pname) if pname else None
        line = "    反算 %-2s = %13.6f  (n=%d)" % (label, o, len(vals))
        if p:
            line += "   参数(%s)=%.6g  比值=%.3f" % (pname, p, o / p)
            if abs(o / p - 1.0) > 0.15:
                line += "   <== 不符!"
                mismatch.append(pname)
        add(line, "bad" if "不符" in line else "ok")

    if prof["map"]["FF"]:
        _gain([r[6] / (r[1] * to_int) for r in win if abs(r[1]) > 1e-4],
              prof["map"]["FF"], "FF")
    else:
        _g = [r[6] / (r[1] * to_int) for r in win if abs(r[1]) > 1e-4]
        if _g:
            o = sum(_g) / len(_g)
            add("    反算 FF斜率 = FF项/期望 = %12.6f  (n=%d)  [速度环无 FF 参数]" % (o, len(_g)))
            cs = params.get("CRUISE_SPEED")
            ct = params.get("CRUISE_THROTTLE")
            if cs and ct and cs > 0:
                exp = (ct / 100.0) / cs
                rr = o / exp if exp else 0.0
                line = ("    预期 (CRUISE_THROTTLE/100)/CRUISE_SPEED = %12.6f  "
                        "实/预=%.3f" % (exp, rr))
                if abs(rr - 1.0) > 0.15:
                    line += "   <== 不符! CRUISE_* 与实际不一致"
                    mismatch.append("CRUISE_SPEED/CRUISE_THROTTLE")
                add(line, "bad" if "不符" in line else "ok")
                add("    (该斜率就是油门-速度直线的设计斜率; 若与实测油门/地速相差大,"
                    " 说明 CRUISE_* 需重新标定)")
    _gain([r[3] / ((r[1] - r[2]) * to_int) for r in win
           if abs((r[1] - r[2]) * to_int) > 0.02], prof["map"]["P"], "P")

    # 滞后/振荡
    add("")
    add("[4] 滞后 与 振荡")
    gd = _resample([(r[0], r[1] * kk) for r in win], t_start, t_end)
    ga = _resample([(r[0], r[2] * kk) for r in win], t_start, t_end)
    lag, cc = _lag(gd, ga, 0.05)
    f = r1 = r2 = None
    z = 0
    if lag is not None:
        add("    期望->实际 滞后 = %.2f s   (相关系数 %.3f)" % (lag, cc),
            "warn" if lag > 0.6 else "ok")
    if len(gd) > 20 and len(ga) > 20:
        n = min(len(gd), len(ga))
        diff = [ga[i][1] - gd[i][1] for i in range(n)]
        f, z = _zcf(diff, 0.05)
        if f:
            add("    误差摆动频率 = %.2f Hz   (%d 次过零)" % (f, z))
        # 先确认确实在"振荡", 否则缓慢漂移会被误判成发散
        oscillating = (f is not None) and (f >= OSC_MIN_HZ) and (z >= OSC_MIN_ZC)
        h = n // 2
        r1 = math.sqrt(sum(x * x for x in diff[:h]) / max(1, h))
        r2 = math.sqrt(sum(x * x for x in diff[h:]) / max(1, n - h))
        m1 = sum(diff[:h]) / max(1, h)
        m2 = sum(diff[h:]) / max(1, n - h)
        if not oscillating:
            add("    误差RMS 前半=%.3f 后半=%.3f -> 未检测到振荡(缓慢漂移), 不做发散判定"
                % (r1, r2), "warn")
        else:
            v = "收敛" if r2 < r1 * 0.85 else ("发散!" if r2 > r1 * 1.15 else "持平")
            add("    误差RMS 前半=%.3f 后半=%.3f -> %s" % (r1, r2, v),
                "bad" if v == "发散!" else "")
        add("    误差均值 前半=%+.3f 后半=%+.3f  (均值自身漂移=工况变了, 不是振荡发散)"
            % (m1, m2))
        if abs(m2 - m1) > 0.5 * max(r1, r2, 1e-6):
            add("      -> 前后工况差异明显, 上面的趋势判定参考价值低", "warn")

    # 速度/油门
    hw = [r for r in hud if r[0] >= t_start]
    if hw:
        gss = [r[1] for r in hw]
        ths = [r[2] for r in hw]
        add("")
        add("[5] 速度 / 油门   (n=%d)" % len(hw))
        add("    地速 mean=%.3f m/s  峰值=%.3f" % (sum(gss) / len(gss), max(gss)))
        add("    油门 mean=%.1f %%   峰值=%.1f" % (sum(ths) / len(ths), max(ths)))
        if sum(gss) / len(gss) > 0.05:
            add("    油门/地速 = %.2f  %%/(m/s)  -> ATC_SPEED_P/I 量级参考"
                % (sum(ths) / len(ths) / (sum(gss) / len(gss))))

    # 差速输出
    fresh = [(t1, a, b) for (t1, a), (_t2, b) in zip(s1, s3) if t1 >= t_start]
    if len(fresh) >= 3:
        dv = [a - b for (_t, a, b) in fresh]
        add("")
        add("[6] 差速输出 (SERVO1-SERVO3, n=%d)" % len(fresh))
        add("    左右差 mean=%+.1f  RMS=%.1f  峰值=%+.0f"
            % (sum(dv) / len(dv), _stats(dv)["rms"], max(abs(max(dv)), abs(min(dv)))))
        # 执行器余量：这里的饱和与 ATC_STR_RAT_MAX 的饱和是两回事,
        # 它是真正的物理上限 —— 转向权限用尽就发生在这里
        s1min = params.get("SERVO1_MIN", SERVO_DEF["MIN"])
        s1max = params.get("SERVO1_MAX", SERVO_DEF["MAX"])
        t1c = params.get("SERVO1_TRIM", SERVO_DEF["TRIM"])
        s3min = params.get("SERVO3_MIN", SERVO_DEF["MIN"])
        s3max = params.get("SERVO3_MAX", SERVO_DEF["MAX"])
        t3c = params.get("SERVO3_TRIM", SERVO_DEF["TRIM"])
        f1, r1 = s1max - t1c, t1c - s1min
        f3, r3 = s3max - t3c, t3c - s3min
        span = min(f1, r1, f3, r3) if min(f1, r1, f3, r3) > 10 else 400.0
        add("    行程       S1 %g~%g (前%+.0f/后%+.0f)   S3 %g~%g (前%+.0f/后%+.0f)"
            % (s1min, s1max, f1, r1, s3min, s3max, f3, r3))
        hs = [r for r in hud if t_start <= r[0] <= t_end]
        occ, pk_o, pk_th = [], 0.0, None
        hi, nh = 0, len(hs)
        for (ts, a, b) in fresh:
            if not nh:
                break
            # fresh 与 hs 都按时间有序 -> 双指针往前推即可。
            # (不要写成 min(hs, key=lambda r: abs(r[0]-ts)): 那是 O(n*m),
            #  240s 航段会多花 ~2.4s, 而这里是在持锁的读取线程里跑的)
            while hi + 1 < nh and abs(hs[hi + 1][0] - ts) <= abs(hs[hi][0] - ts):
                hi += 1
            h = hs[hi]
            if abs(h[0] - ts) > 0.6:
                continue
            lim = 2.0 * span * (1.0 - max(0.0, min(1.0, h[2] / 100.0)))
            if lim < 20.0:
                continue
            o = abs(a - b) / lim
            occ.append(o)
            if o > pk_o:
                pk_o, pk_th = o, h[2]
        if occ:
            oc = sorted(occ)
            mid = oc[len(oc) // 2]
            p90 = oc[min(len(oc) - 1, int(0.9 * len(oc)))]
            n95 = sum(1 for x in oc if x >= 0.95)
            n100 = sum(1 for x in oc if x >= 1.0)
            add("    执行器占用 中位=%.2f  P90=%.2f  峰值=%.2f  (相对当前油门下的差速上限)"
                % (mid, p90, oc[-1]),
                "bad" if oc[-1] >= 0.95 else ("warn" if p90 >= 0.8 else "ok"))
            add("    >=0.95 的样本 %d/%d = %.1f%%   >=1.00: %.1f%%"
                % (n95, len(oc), 100.0 * n95 / len(oc), 100.0 * n100 / len(oc)),
                "bad" if 100.0 * n95 / len(oc) > 5.0 else
                ("warn" if n95 else "ok"))
            if pk_th is not None:
                add("    峰值出现在油门 %.0f%% 时 (该油门下差速上限 %.0f)"
                    % (pk_th, 2.0 * span * (1.0 - max(0.0, min(1.0, pk_th / 100.0)))))
        n1 = sum(1 for (_t, a, _b) in fresh if a <= s1min + 5 or a >= s1max - 5)
        n3 = sum(1 for (_t, _a, b) in fresh if b <= s3min + 5 or b >= s3max - 5)
        p1, p3 = 100.0 * n1 / len(fresh), 100.0 * n3 / len(fresh)
        asym = "   <<< 左右不对称, 查左右推力/接线是否一致" \
            if abs(n1 - n3) > max(3, 0.5 * max(n1, n3)) else ""
        add("    贴 MIN/MAX 边  S1 %d (%.1f%%)   S3 %d (%.1f%%)%s"
            % (n1, p1, n3, p3, asym), "warn" if (p1 > 2 or p3 > 2) else "ok")

    # 对地运动 / 侧滑 / 姿态 / 估计器 / 传感器
    aw = [r for r in att if r[0] >= t_start]
    vw = [r for r in vel if r[0] >= t_start]
    if vw and aw:
        pairs = []
        for tv, vx, vy in vw:
            cand = None
            for a in aw:
                if a[0] >= tv - 0.35:
                    if abs(a[0] - tv) < 0.35:
                        cand = a
                    break
            if cand is None:
                continue
            sp = math.hypot(vx, vy)
            if sp < 0.1:
                continue
            hdg = math.degrees(math.atan2(vy, vx))
            slip = (hdg - math.degrees(cand[4]) + 540.0) % 360.0 - 180.0
            pairs.append((sp, slip))
        if pairs:
            sps = [p[0] for p in pairs]
            sls = [p[1] for p in pairs]
            msl = sum(sls) / len(sls)
            add("")
            add("[7] 对地运动 / 侧滑   (n=%d)" % len(pairs))
            add("    对地速度 mean=%.3f m/s  峰值=%.3f" % (sum(sps) / len(sps), max(sps)))
            add("    侧滑角   mean=%+.1f°  峰值=%.1f°"
                % (msl, max(abs(min(sls)), abs(max(sls)))),
                "warn" if abs(msl) > 15 else "ok")
            if abs(msl) > 15:
                add("    -> 侧滑偏大: 船在横漂(风/流/转向过猛), 航迹精度会受影响。", "warn")
    if aw:
        ro = [r[2] for r in aw]
        pi = [r[3] for r in aw]
        add("")
        add("[8] 姿态   滚转 mean=%+.1f° 峰=%.1f°   |   俯仰 mean=%+.1f° 峰=%.1f°"
            % (math.degrees(sum(ro) / len(ro)),
               math.degrees(max(abs(min(ro)), abs(max(ro)))),
               math.degrees(sum(pi) / len(pi)),
               math.degrees(max(abs(min(pi)), abs(max(pi))))))
    ee = [r for r in ekf if r[0] >= t_start]
    gg = [r for r in gps if r[0] >= t_start]
    if ee or gg:
        add("")
        add("[9] 估计器 / 传感器")
    if ee:
        fl = ee[-1][1]
        ready = bool(fl & 0x1F)
        add("    EKF flags=0x%02X (%s)  posHorizVar=%.3f  compassVar=%.3f"
            % (fl, "姿态/水平位置基本就绪" if ready else "未就绪! 数据不可信",
               ee[-1][2], ee[-1][3]), "ok" if ready else "bad")
    if gg:
        fxt, sats, eph = gg[-1][1], gg[-1][2], gg[-1][3]
        good = fxt >= 3 and sats >= 8
        add("    GPS fix=%d sats=%d eph=%.1f  %s"
            % (fxt, sats, eph, "OK" if good else "偏弱(fix<3 或 卫星<8), 定位噪声大"),
            "ok" if good else "warn")

    # 导航误差(航线跟踪) + 外环反算
    try:
        _nav_block(d, t_start, t_end, params, add, is_frozen)
    except Exception as e:
        errlog("nav_block: %r" % (e,))
        add("")
        add("[10] 导航误差: 计算失败 %s" % e, "warn")

    # 参数
    add("")
    add("[11] 飞控参数")
    _keys = [v for v in prof["map"].values() if v] + list(prof["extra"])
    _seen = set()
    for nm in _keys:
        if nm in _seen:
            continue
        _seen.add(nm)
        if nm in params:
            _mk = "   <== 与实测不符" if nm in mismatch else ""
            add("    %-18s = %-12.6g%s" % (nm, params[nm], _mk), "bad" if _mk else "")
    _age = d.get("params_age")
    if _age is not None and _age > 20.0:
        add("    !! 以上参数是 %.0f 秒前读到的, 可能不是本段开始时的值" % _age, "warn")

    # 外环(位置控制器) —— 航点巡航的期望转向率由 PSC_* 生成,
    # 内环 ATC_STR_RAT_* 只是跟随它。S 型走位归属要先看这一段。
    _psc = ["PSC_POS_P", "PSC_VEL_P", "PSC_VEL_I", "PSC_VEL_D",
            "PSC_VEL_IMAX", "PSC_VEL_FLTE", "PSC_VEL_FLTD", "PSC_VEL_FF"]
    if any(n in params for n in _psc):
        add("")
        add("[12] 外环 / 位置控制器 (PSC_*)   <- S 型走位在这里产生")
        for n in _psc:
            if n in params:
                bad = (n == "PSC_VEL_D" and not params[n])
                add("    %-16s = %-10.6g%s"
                    % (n, params[n], "   <== 阻尼项为0" if bad else ""),
                    "bad" if bad else "")
        geo = [n for n in ("ATC_TURN_MAX_G", "TURN_RADIUS", "WP_RADIUS",
                           "WP_SPEED", "WP_ACCEL") if n in params]
        if geo:
            add("    导航几何    " +
                "  ".join("%s=%g" % (n, params[n]) for n in geo))
        pp, vp = params.get("PSC_POS_P"), params.get("PSC_VEL_P")
        if pp and vp and pp > 0 and vp > 0:
            wn = math.sqrt(pp * vp)
            _vd = params.get("PSC_VEL_D")
            _note = ""
            if _vd is not None:
                _note = ("   [PSC_VEL_D=%g 提供阻尼]" % _vd if _vd
                         else "   <== PSC_VEL_D=0 无阻尼, 会在此频率上振荡")
            add("    固有频率 sqrt(PSC_POS_P x PSC_VEL_P) = %.3f rad/s"
                " = %.3f Hz (周期 %.1f s)%s"
                % (wn, wn / (2 * math.pi), 2 * math.pi / wn, _note),
                "" if (_vd is None or _vd) else "bad")
        # 摆动波长 : 航段长度 —— 判定 S 型到底属于内环还是外环
        if f and hw:
            gsm = sum(r[1] for r in hw) / len(hw)
            if gsm > 0.05:
                lam = gsm / f
                add("    摆动 %.2f Hz @ 地速 %.2f m/s -> 波长 %.1f m"
                    % (f, gsm, lam))
                legs = _leg_lengths(d.get("mission"))
                if legs:
                    ml = sum(legs) / len(legs)
                    ratio = lam / ml
                    add("    平均航段长 %.1f m  ->  波长/航段 = %.2f" % (ml, ratio))
                    if abs(ratio - 1.0) <= 0.3:
                        add("      约等于1: 每个航段一次起伏 -> 更像过航点的转弯响应",
                            "warn")
                    elif ratio < 0.75:
                        add("      明显小于1: 单个航段内来回多次 -> 外环固有振荡,"
                            " 调 PSC_VEL_D / PSC_VEL_P, 不是 ATC_STR_RAT_*", "bad")
                    else:
                        add("      介于两者之间, 需要更长航段才能分辨")

    # 结论
    add("")
    add("-" * 70)
    add(" 判读建议")
    add("-" * 70)
    mFF = prof["map"]["FF"] or "CRUISE_THROTTLE"
    mP = prof["map"]["P"] or "?"
    mI = prof["map"]["I"] or "?"
    mD = prof["map"]["D"] or "?"
    tips = []
    for nm in mismatch:
        tips.append("[关键] %s 实测有效增益与参数值不符: 参数可能未生效或被内部缩放,"
                    " 建议做一次大幅改动(如 2 倍)验证反算值能否跟上。" % nm)
    if mabs < 1.0 and abs(sd["mean"]) > 1.0:
        tips.append("跟踪很好 (平均绝对误差 < 1 %s), 当前 P/I 基本合适。" % unit)
    if bias > 0.6 and mabs > max(1.0, abs(sd["mean"]) * 0.15):
        if terms["FF"] < 0.35:
            tips.append("稳态误差长期同号(偏置 %.2f)且 FF 仅占 %.0f%%: 前馈不足, "
                        "可小幅提高 %s。" % (bias, terms["FF"] * 100, mFF))
        else:
            tips.append("稳态误差长期同号(偏置 %.2f): 前馈够但回正慢, 可小幅提高 %s。"
                        % (bias, mP))
    if bias < 0.3 and mabs > max(2.0, abs(sd["mean"]) * 0.4):
        # 先分清到底是「权限不够」还是「增益过高」, 结论完全不同, 不能靠猜
        if sat_pct is not None and sat_pct > 5.0:
            tips.append("[关键] 误差在零附近来回且幅度大 (平均绝对误差 %.1f %s), "
                        "但期望有 %.1f%% 的样本顶到 %s=%.6g 上限: 这是【权限不足】"
                        "而不是增益过高 -> 优先提高 %s(或改善物理转向能力/降速)，"
                        "不要降 %s。"
                        % (mabs, unit, sat_pct, sat_lim_name, sat_lim,
                           sat_lim_name, mP))
        elif terms["FF"] < 0.15:
            tips.append("误差在零附近来回且幅度大 (平均绝对误差 %.1f %s), 且 FF 仅占 "
                        "%.0f%%: 优先提高 %s。前馈不足时 P/I 被迫硬顶, 表观上很像"
                        "「增益过高」,此时降 %s 会把方向做反。"
                        % (mabs, unit, terms["FF"] * 100, mFF, mP))
        else:
            tips.append("误差在零附近来回且幅度大 (平均绝对误差 %.1f %s; FF 已占 "
                        "%.0f%%, 饱和 %.1f%%): 增益/相位偏高, 可小幅降 %s 或加 %s。"
                        % (mabs, unit, terms["FF"] * 100,
                           sat_pct if sat_pct is not None else 0.0, mP, mD))
    if terms["I"] > 0.55:
        tips.append("I 项承担 %.0f%% 输出: P/FF 偏小, 易积分饱和 -> 降 %s 或提高 %s。"
                    % (terms["I"] * 100, mI, mP))
    if terms["D"] < 0.03:
        tips.append("D 项几乎不出力 (%.1f%%): 若有摆振/超调, 加 %s 最直接。"
                    % (terms["D"] * 100, mD))
    if lag is not None and lag > 0.6:
        tips.append("滞后 %.2f s 偏大: 可能是 MOT_SLEWRATE 限制或输出饱和, 检查是否推满。" % lag)
    if f and r1 and r2 and f >= OSC_MIN_HZ and z >= OSC_MIN_ZC and r2 > r1 * 1.15:
        tips.append("检测到 %.2f Hz 振荡且幅度增大: 明确发散趋势 -> 减小 %s 或加 %s。"
                    % (f, mP, mD))
    if abs(sd["mean"]) < max(0.5, 0.05 * sa["rms"]):
        tips.append("窗口内%s几乎没有变化: 需有持续的%s指令才有诊断价值。"
                    % (prof["what"], prof["what"]))
    if not tips:
        tips.append("无明显异常特征, 需要更长/更明确的转向动作才能进一步判断。")
    for i, tp in enumerate(tips, 1):
        add("  %d. %s" % (i, tp))
    return L, "\n".join(R)


# ============================================================ 子页：实时监视
class MonitorView(tk.Frame):
    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self.cfg = owner.cfg
        self.q = owner.q
        self._draw_t = -1.0
        self._draw_wall = 0.0
        self.rows = []

        rc_text = ", ".join("RC%d" % c for c in parse_chan_list(self.cfg.get("rc_channels")))
        self.lbl_in = ttk.Label(self, text="输入通道 %s (PWM, 1500≈中) — 最近 20s 滚动" % rc_text,
                                foreground="#555")
        self.lbl_in.pack(anchor="w", padx=10)
        self.plot_in = ZoomPlot(self, height=110)
        self.plot_in.set_zero(1500)
        self.plot_in.pack(fill="both", expand=True, padx=8)
        s_text = ", ".join("S%d" % c for c in parse_chan_list(self.cfg.get("servo_channels")))
        self.lbl_out = ttk.Label(self, text="电机输出 %s (>1500 正转/加速, <1500 反转/减速, 1500≈停)" % s_text,
                                 foreground="#555")
        self.lbl_out.pack(anchor="w", padx=10, pady=(4, 0))
        self.plot_out = ZoomPlot(self, height=130)
        self.plot_out.set_zero(1500)
        self.plot_out.pack(fill="both", expand=True, padx=8)

        ttk.Label(self, text="自动测得动作（每次打杆→回中 记一行）",
                  foreground="#555").pack(anchor="w", padx=10, pady=(4, 0))
        cols = ("no", "tm", "stick", "lat", "rise", "target", "swing", "ends")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=5)
        for c, txt, w in (("no", "#", 32), ("tm", "时刻", 74), ("stick", "杆", 60),
                          ("lat", "反应(s)", 66), ("rise", "OUT爬升(s)", 92),
                          ("target", "目标PWM", 70), ("swing", "摆幅PWM", 70),
                          ("ends", "S1/S3 最终", 100)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", padx=8)
        fb = ttk.Frame(self)
        fb.pack(fill="x", padx=8, pady=4)
        self.summary = ttk.Label(fb, text="", foreground="#06c")
        self.summary.pack(side="left")
        ttk.Button(fb, text="清空表格", command=self.clear_rows).pack(side="right")
        ttk.Button(fb, text="清空波形", command=self.clear_buf).pack(side="right", padx=6)
        self.after(150, self._poll)

    def _poll(self):
        while True:
            try:
                it = self.q.get_nowait()
            except queue.Empty:
                break
            self._add_row(it)
        now_m = time.monotonic()
        if now_m - getattr(self, "_chk_t", 0.0) >= 1.5:
            self._chk_t = now_m
            self._refresh_cfg_labels()
        f = self.owner.feed
        if f is not None:
            with f.lock:
                last = 0.0
                for v in f.buf.values():
                    if v and v[-1][0] > last:
                        last = v[-1][0]
            # 节流: 原来"样本总数一变就重画" -> 每收到一条报文都全量重画(~12Hz)。
            # 现在最多 5Hz, 且只看最新时间戳。
            now_m = time.monotonic()
            if last != self._draw_t and now_m - self._draw_wall >= 0.20:
                self._draw_t = last
                self._draw_wall = now_m
                self._redraw()
        self.after(150, self._poll)

    def _refresh_cfg_labels(self):
        """按最新配置刷新标题；已连接且采集通道与配置不一致时提示重连"""
        rc_list = parse_chan_list(self.cfg.get("rc_channels"))
        sv_list = parse_chan_list(self.cfg.get("servo_channels"))
        self.lbl_in.config(text="输入通道 %s (PWM, 1500≈中) — 最近 20s 滚动"
                                % ", ".join("RC%d" % c for c in rc_list))
        self.lbl_out.config(text="电机输出 %s (>1500 正转/加速, <1500 反转/减速, 1500≈停)"
                                 % ", ".join("S%d" % c for c in sv_list))
        f = self.owner.feed
        if f is not None and f.is_alive():
            if list(f.rc_list) != rc_list or list(f.servo_list) != sv_list:
                self.owner.status.config(
                    text="⚠ 通道配置已改，请【断开】再【连接】后生效", foreground="#c60")

    def on_shown(self):
        """切回本页时立即刷新标题"""
        self._refresh_cfg_labels()
        self._draw_t = -1.0

    def _redraw(self):
        f = self.owner.feed
        if f is None:
            return
        # 只取最近 WINDOW_SEC 的尾巴(二分切片), 不再 list() 复制整段 600s 缓冲
        with f.lock:
            now = 0.0
            for v in f.buf.values():
                if v and v[-1][0] > now:
                    now = v[-1][0]
            tm = now - WINDOW_SEC
            snap = {k: _tail_from(v, tm) for k, v in f.buf.items()}
        if not any(snap.values()):
            return
        rc_s = []
        for c in f.rc_list:
            k = chan_key("rc", c)
            rc_s.append(("RC%d" % c, [(t - tm, v) for t, v in snap.get(k, [])]))
        self.plot_in.set_series(rc_s, ylabel="PWM")
        sv_s = []
        for c in f.servo_list:
            k = chan_key("s", c)
            sv_s.append(("S%d" % c, [(t - tm, v) for t, v in snap.get(k, [])]))
        self.plot_out.set_series(sv_s, ylabel="PWM")

    def _add_row(self, it):
        tm = datetime.datetime.fromtimestamp(it["t0"]).strftime("%H:%M:%S")
        s1v = "%.0f" % it["s1_end"] if it.get("s1_end") is not None else "--"
        s3v = "%.0f" % it["s3_end"] if it.get("s3_end") is not None else "--"
        self.rows.append(it)
        self.tree.insert("", "end", values=(
            len(self.rows), tm, it.get("stick", ""), "%.2f" % it.get("lat", 0),
            "%.2f" % it["rise"], "%.0f" % it["target"], "%.0f" % it["swing"],
            "%s / %s" % (s1v, s3v)))
        if len(self.rows) > 300:
            self.tree.delete(self.tree.get_children()[0])
            self.rows.pop(0)
        n = len(self.rows)
        if n:
            avg = sum(x["rise"] for x in self.rows) / n
            mx = max(x["rise"] for x in self.rows)
            avgl = sum(x.get("lat", 0) for x in self.rows) / n
            self.summary.config(text="已测 %d 次 | 平均反应 %.2fs | 平均OUT爬升 %.2fs | 最长 %.2fs"
                                     % (n, avgl, avg, mx))

    def clear_rows(self):
        self.rows = []
        for r in self.tree.get_children():
            self.tree.delete(r)
        self.summary.config(text="")

    def clear_buf(self):
        f = self.owner.feed
        if f:
            with f.lock:
                for k in f.buf:
                    f.buf[k] = []
        self._draw_t = -1.0

    def on_shown(self):
        """切回本页时重读配置(设置里改了通道后立即生效)"""
        rc_list = parse_chan_list(self.cfg.get("rc_channels"))
        sv_list = parse_chan_list(self.cfg.get("servo_channels"))
        self.lbl_in.config(text="输入通道 %s (PWM, 1500≈中) — 最近 20s 滚动"
                                % ", ".join("RC%d" % c for c in rc_list))
        self.lbl_out.config(text="电机输出 %s (>1500 正转/加速, <1500 反转/减速, 1500≈停)"
                                 % ", ".join("S%d" % c for c in sv_list))
        # 已连接时: 若采集通道与配置不一致, 提示需重连(采集列表在建连时确定)
        f = self.owner.feed
        if f is not None and f.is_alive():
            if list(f.rc_list) != rc_list or list(f.servo_list) != sv_list:
                self.owner.status.config(
                    text="⚠ 通道配置已改，请【断开】再【连接】后生效", foreground="#c60")
        self._draw_t = -1.0


# ============================================================ 子页：PID 调试
class PidView(tk.Frame):
    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self._last_t = {}
        self._drawn_t = {}
        self._draw_wall = 0.0
        self._ids = collections.deque()
        self._plots = {}
        self._hover = None
        self._follow = True
        self._paused = False
        self._skip = False
        self._rown = 0

        bar = ttk.Frame(self, padding=(8, 3))
        bar.pack(fill="x")
        ttk.Label(bar, text="PID 轴: ").pack(side="left")
        self.axis_cb = ttk.Combobox(bar, values=[PID_AXIS], width=6, state="readonly")
        self.axis_cb.set(PID_AXIS)
        self.axis_cb.pack(side="left")
        ttk.Label(bar, text="  滚轮=缩放 | 双击=复位", foreground="#888").pack(side="left", padx=10)
        self.mode_lbl = ttk.Label(bar, text="", foreground="#888")
        self.mode_lbl.pack(side="left", padx=8)

        self.holder = ttk.Frame(self)
        self.holder.pack(fill="both", expand=True, padx=8)
        self.holder.columnconfigure(0, weight=1)
        rows = [("da", ("desired", "achieved"), ("#1f77b4", "#d62728"), "转向率 期望(蓝) vs 实际(红) — 同一 Y 轴 deg/s"),
                ("P", ("P",), ("#2ca02c",), "P 比例项输出"),
                ("I", ("I",), ("#ff7f0e",), "I 积分项输出"),
                ("D", ("D",), ("#9467bd",), "D 微分项输出"),
                ("FF", ("FF",), ("#7f7f7f",), "FF 前馈项输出")]
        for i in range(len(rows)):
            self.holder.rowconfigure(i, weight=1, uniform="r")
        for i, (key, flds, cols, title) in enumerate(rows):
            p = ZoomPlot(self.holder, height=90)
            p.grid(row=i, column=0, sticky="nsew", pady=1)
            p.set_title(title)
            p.bind("<Enter>", lambda e, pl=p: setattr(self, "_hover", pl))
            p.bind("<MouseWheel>", lambda e, pl=p: self._zoom_at(pl, e))
            p.bind("<Double-Button-1>", lambda e: self._reset_zoom())
            self._plots[key] = (flds, cols, p)

        ttk.Label(self, text="数据滚动（滚轮上下回看历史，右侧=时间）",
                  foreground="#555").pack(anchor="w", padx=10, pady=(4, 0))
        cols = ("desired", "achieved", "err", "P", "I", "D", "FF", "tm")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=7)
        for c, txt, w in (("desired", "期望", 76), ("achieved", "实际", 76), ("err", "误差", 76),
                          ("P", "P", 74), ("I", "I", 74), ("D", "D", 74),
                          ("FF", "FF", 74), ("tm", "时间", 96)):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=8)
        self.tree.bind("<MouseWheel>", self._on_wheel)
        self.tree.bind("<Button-4>", self._on_wheel)
        self.tree.bind("<Button-5>", self._on_wheel)
        self.tree.bind("<B1-Motion>", lambda e: self._set_follow(False))
        fb = ttk.Frame(self)
        fb.pack(fill="x", padx=8, pady=4)
        self.lbl_info = ttk.Label(fb, text="等待 PID_TUNING…", foreground="#888")
        self.lbl_info.pack(side="left")
        self.pause_btn = tk.Button(fb, text="⏸ 暂停", width=9, command=self.toggle_pause)
        self.pause_btn.pack(side="left", padx=(14, 0))
        ttk.Button(fb, text="继续跟随最新", command=lambda: self._set_follow(True)).pack(side="right", padx=4)
        ttk.Button(fb, text="清空", command=self.clear_all).pack(side="right")
        self.after(150, self._poll)

    def on_shown(self):
        pass

    def _cur_axis(self):
        try:
            return int(self.axis_cb.get())
        except Exception:
            return PID_AXIS

    def toggle_pause(self):
        self._paused = not self._paused
        self.pause_btn.config(text="▶ 继续" if self._paused else "⏸ 暂停",
                              fg="#c90" if self._paused else "#111")
        if not self._paused:
            self._skip = True

    def _poll(self):
        f = self.owner.feed
        if f is not None:
            with f.lock:
                md = f.mode
            if md is not None:
                nm = MODE_NAMES.get(md, "模式%d" % md)
                self.mode_lbl.config(text="模式: " + nm,
                                     foreground="#d00" if md == 0 else "#0a0")
            if f.connected and not f.pid_valid:
                _nm = MODE_NAMES.get(md, "模式%s" % md) if md is not None else "未收到心跳"
                self.lbl_info.config(
                    text="⚠ 当前 %s：飞控不运行转向速率控制器，PID_TUNING 是旧值(垃圾数据)，"
                         "已停止采集 — 请切到 AUTO / GUIDED / CIRCLE / LOITER 等模式。" % _nm,
                    foreground="#c00")
                self.after(150, self._poll)
                return
            if f.connected:
                ax = self._cur_axis()
                with f.lock:
                    avail = sorted(f.pid_hist.keys())
                    h = f.pid_hist.get(ax) or []
                    last_ts = h[-1][0] if h else 0.0
                    prev = self._last_t.get(ax, 0.0)
                    if prev > 0:
                        # 只取上次之后的新样本(二分切片), 不再复制整段历史
                        new = _tail_from(h, prev)
                        if new and new[0][0] <= prev:
                            new = new[1:]
                    else:
                        new = list(h)
                if avail and list(self.axis_cb["values"]) != avail:
                    self.axis_cb["values"] = avail
                if self._paused:
                    self.lbl_info.config(text="⏸ 已暂停 — 点 [▶ 继续] 恢复实时", foreground="#f90")
                    self.after(150, self._poll)
                    return
                if self._skip and h:
                    self._last_t[ax] = last_ts
                    new = []
                    self._skip = False
                if new:
                    self._last_t[ax] = new[-1][0]
                    self._append_rows(new)
                # 重绘节流到 5Hz: 原来每来一个 PID 样本都全量重画
                now_m = time.monotonic()
                if (last_ts != self._drawn_t.get(ax, -1.0)
                        and now_m - self._draw_wall >= 0.20):
                    self._drawn_t[ax] = last_ts
                    self._draw_wall = now_m
                    self._redraw(ax)
        self.after(150, self._poll)

    def _append_rows(self, rows):
        for r in rows:
            self._append_row(r, autoview=False)
        if self._follow:
            try:
                self.tree.yview_moveto(1.0)
            except Exception:
                pass

    def _append_row(self, r, autoview=True):
        t, d, a, p, i, di, ff = r
        tm = datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S")
        self._rown += 1
        # 自己记住行 id, 不再每插一行就 get_children() 构造 3000 个 id 的列表
        self._ids.append(self.tree.insert(
            "", "end", values=("%.2f" % d, "%.2f" % a, "%+.2f" % (d - a),
                                "%.4f" % p, "%.5f" % i, "%.4f" % di,
                                "%.4f" % ff, tm)))
        while len(self._ids) > 3000:
            try:
                self.tree.delete(self._ids.popleft())
            except Exception:
                self._ids.popleft()
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
        self._ids.clear()
        for r in self.tree.get_children():
            self.tree.delete(r)
        self._last_t = {}

    def _redraw(self, ax):
        f = self.owner.feed
        if f is None:
            return
        # 只取最近 WINDOW_SEC(二分切片), 不再复制并扫描整段历史
        with f.lock:
            h = f.pid_hist.get(ax) or []
            if h:
                hist = _tail_from(h, h[-1][0] - WINDOW_SEC)
                last = h[-1]
            else:
                hist, last = [], None
        if not hist:
            self.lbl_info.config(text="未收到 PID_TUNING (需 GCS_PID_MASK≠0 且切到 AUTO/GUIDED)",
                                 foreground="#b00")
            return
        now = hist[-1][0]
        tm = now - WINDOW_SEC
        for key, (flds, cols, p) in self._plots.items():
            series = []
            for fl in flds:
                idx = PID_FIELDS.index(fl)
                series.append((fl, [(r[0] - tm, r[1 + idx]) for r in hist]))
            p.set_series(series, colors=cols)
        t, d, a, p, i, di, ff = last
        self.lbl_info.config(
            foreground="#0a0",
            text="[%s] 期望 %.2f | 实际 %.2f | 误差 %+.2f | P %.4f I %.5f D %.4f FF %.4f"
                 % (AXIS_NAME.get(ax, "axis%d" % ax), d, a, d - a, p, i, di, ff))

    def _zoom_at(self, plot, ev):
        cx = plot.pixel_to_x(ev.x)
        lo, hi = plot._zoom if plot._zoom else (0.0, WINDOW_SEC)
        span = hi - lo
        if span < 0.3:
            return "break"
        if getattr(ev, "num", 0):
            factor = 0.72 if ev.num == 4 else 1.4
        else:
            factor = 0.72 if ev.delta > 0 else 1.4
        nspan = max(0.3, min(WINDOW_SEC, span * factor))
        frac = 0.5 if cx is None else max(0.0, min(1.0, (cx - lo) / span))
        center = lo + span * frac
        nlo = max(0.0, center - nspan / 2)
        nhi = min(WINDOW_SEC, nlo + nspan)
        nlo = max(0.0, nhi - nspan)
        for _f, _c, p in self._plots.values():
            p.set_zoom(nlo, nhi)
        return "break"

    def _reset_zoom(self):
        for _f, _c, p in self._plots.values():
            p.reset_zoom()
        return "break"


# ============================================================ 子页：RAT 诊断
class DiagView(tk.Frame):
    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self.last_report = ""
        self.n_seg = 0
        bar = ttk.Frame(self, padding=(8, 4))
        bar.pack(fill="x")
        ttk.Label(bar, text="目标:").pack(side="left")
        self.tgt = ttk.Combobox(bar, state="readonly", width=9,
                                values=["两者", "转向环", "速度环"])
        self.tgt.set(owner.cfg.get("diag_target", "两者"))
        self.tgt.pack(side="left", padx=(2, 12))
        ttk.Label(bar, text="分析窗口(秒):").pack(side="left")
        self.sec = tk.IntVar(value=int(owner.cfg.get("diag_sec", 30)))
        ttk.Spinbox(bar, from_=5, to=300, increment=5, textvariable=self.sec,
                    width=6).pack(side="left", padx=4)
        ttk.Button(bar, text="▶ 生成诊断", command=self.run_diag).pack(side="left", padx=8)
        ttk.Button(bar, text="💾 保存报告", command=self.save_report).pack(side="left")
        ttk.Button(bar, text="📂 航段文件夹", command=self.open_seg_dir).pack(side="left", padx=4)
        self.seg_lbl = ttk.Label(bar, text="航段: 0", foreground="#666")
        self.seg_lbl.pack(side="left", padx=6)
        ttk.Button(bar, text="清空", command=lambda: self.txt.delete("1.0", "end")).pack(side="left", padx=4)
        self.state = ttk.Label(bar, text="", foreground="#888")
        self.state.pack(side="left", padx=10)

        self.txt = tk.Text(self, font=("Consolas", 10), wrap="none",
                           bg="#fbfbfb", relief="flat")
        sb = ttk.Scrollbar(self, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(fill="both", expand=True, padx=8, pady=6)
        self.txt.tag_config("bad", foreground="#c00", font=("Consolas", 10, "bold"))
        self.txt.tag_config("warn", foreground="#c60")
        self.txt.tag_config("ok", foreground="#080")

    def run_diag(self):
        f = self.owner.feed
        if f is None or not f.connected:
            self.state.config(text="未连接", foreground="#c00")
            return
        if not f.pid_valid:
            _nm = MODE_NAMES.get(f.mode, "模式%s" % f.mode) if f.mode is not None else "未收到心跳"
            self.txt.delete("1.0", "end")
            self.txt.insert("end", "当前模式不允许诊断：\n\n"
                            "  飞控处于 %s 时，转向角速率控制器不运行，\n"
                            "  PID_TUNING 报文里是上一次运行留下的旧值（垃圾数据）。\n\n"
                            "  请切到 AUTO(自动) / GUIDED(引导) / CIRCLE(绕圈) / LOITER，\n"
                            "  并让船真正转向后再生成诊断。\n" % _nm, "bad")
            self.state.config(text="模式无效", foreground="#c60")
            self.last_report = ""
            return
        self.state.config(text="计算中…", foreground="#888")
        tgt = self.tgt.get()
        axes = {"转向环": [PID_AXIS], "速度环": [SPEED_AXIS],
                "两者": [PID_AXIS, SPEED_AXIS]}.get(tgt, [PID_AXIS])
        self.txt.delete("1.0", "end")
        reps = []
        for ax in axes:
            try:
                bundle = collect_bundle(f)
                lines, rep = compute_diag(bundle, float(self.sec.get()), ax)
            except Exception as e:
                errlog("diag axis=%d: %r" % (ax, e))
                self.txt.insert("end", "诊断出错(axis=%d): %s\n" % (ax, e))
                continue
            for txt, tag in lines:
                self.txt.insert("end", txt + "\n", tag or "")
            self.txt.insert("end", "\n")
            reps.append(rep)
        self.owner.cfg["diag_sec"] = int(self.sec.get())
        self.owner.cfg["diag_target"] = tgt
        self.last_report = "\n\n".join(reps)
        self.state.config(text="完成 %s" % datetime.datetime.now().strftime("%H:%M:%S"),
                          foreground="#080")

    def save_report(self):
        if not self.last_report:
            self.run_diag()
        if not self.last_report:
            return
        p = os.path.join(LOG_DIR, "诊断报告_%s.txt" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                f.write(self.last_report)
            messagebox.showinfo(APP_NAME, "报告已保存:\n%s" % p)
        except Exception as e:
            messagebox.showerror(APP_NAME, "保存失败: %s" % e)

    # ---- 航段自动记录 ----
    def show_segment(self, idx, path, body):
        """一段导航数据结束、已存档 -> 在主窗口里直接展示出来"""
        self.n_seg = idx
        try:
            self.seg_lbl.config(text="航段: 已记 %d 段" % idx, foreground="#080")
        except Exception:
            pass
        self.txt.delete("1.0", "end")
        self.txt.insert("end", "★ 航段 #%d 结束，已自动存档：\n    %s\n" % (idx, path),
                        "ok")
        self.txt.insert("end", "=" * 74 + "\n")
        self.txt.insert("end", body)
        self.last_report = body
        self.state.config(text="航段 #%d 已存档" % idx, foreground="#080")

    def open_seg_dir(self):
        d = get_seg_dir(self.owner.cfg)
        try:
            os.makedirs(d, exist_ok=True)
            if os.name == "nt":
                os.startfile(d)
            else:
                subprocess.Popen(["xdg-open", d])
        except Exception as e:
            messagebox.showerror(APP_NAME, "打开失败: %s" % e)


# ============================================================ 子页：参数
class ParamView(tk.Frame):
    def __init__(self, master, owner):
        super().__init__(master)
        self.owner = owner
        self.labels = {}
        self.flash = {}
        ttk.Label(self, text="飞控参数 (每秒读回一次；变化的行黄色高亮并记入下方日志)",
                  foreground="#555").pack(anchor="w", padx=10, pady=6)
        grid = ttk.Frame(self, padding=(10, 0))
        grid.pack(fill="x")
        for i, nm in enumerate(WATCH):
            r, c = divmod(i, 4)
            cell = ttk.Frame(grid)
            cell.grid(row=r, column=c, padx=10, pady=5, sticky="w")
            ttk.Label(cell, text=nm, font=("Consolas", 9)).pack(anchor="w")
            lb = tk.Label(cell, text="-", font=("Consolas", 14, "bold"), fg="#036")
            lb.pack(anchor="w")
            self.labels[nm] = lb
        ttk.Label(self, text="变化日志", foreground="#555").pack(anchor="w", padx=10, pady=(10, 0))
        self.log = tk.Text(self, font=("Consolas", 10), height=12)
        self.log.pack(fill="both", expand=True, padx=8, pady=6)
        ttk.Button(self, text="清空日志", command=lambda: self.log.delete("1.0", "end")) \
            .pack(anchor="e", padx=10, pady=(0, 6))
        self.after(200, self._poll)

    def _poll(self):
        f = self.owner.feed
        if f is not None:
            with f.lock:
                params = dict(f.params)
                chg = list(f.param_changes)
                f.param_changes = []
            now = time.time()
            for nm, v in params.items():
                if nm in self.labels:
                    fl = (now - self.flash.get(nm, 0)) < 1.5
                    self.labels[nm].config(text="%.6g" % v,
                                           bg="#ffe08a" if fl else "SystemButtonFace")
            for t, nm, old, new in chg:
                self.flash[nm] = t
                ms = int((t % 1) * 1000)
                self.log.insert("end", "%s.%03d   %-18s %s -> %s\n"
                                % (datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S"),
                                   ms, nm, "%.6g" % old, "%.6g" % new))
                self.log.see("end")
        self.after(200, self._poll)


# ============================================================ 主视图
class MainView(tk.Frame):
    def __init__(self, master, cfg):
        super().__init__(master)
        self.cfg = cfg
        self.q = queue.Queue()
        self.feed = None
        self._usb = None
        self._seg_shown = 0

        bar = ttk.Frame(self, padding=(8, 6))
        bar.pack(fill="x")
        ttk.Label(bar, text="飞控地址").pack(side="left")
        self.addr = tk.StringVar(value=cfg.get("live_addr") or
                                 "%s:%s" % (cfg.get("default_ip", ""), cfg.get("mav_port", 5760)))
        ttk.Entry(bar, textvariable=self.addr, width=22).pack(side="left", padx=6)
        ttk.Button(bar, text="🔍 自动发现", width=11, command=self.do_discover).pack(side="left", padx=2)
        ttk.Button(bar, text="� 手动配对", width=10, command=self.open_pair_wizard).pack(side="left", padx=2)
        ttk.Button(bar, text="�📡 扫描网段", width=10, command=self.do_scan).pack(side="left", padx=2)
        ttk.Button(bar, text="USB 一键连", width=10, command=self.do_usb).pack(side="left", padx=2)
        ttk.Button(bar, text="连接", width=7, command=self.do_connect).pack(side="left", padx=2)
        self.btn_dis = ttk.Button(bar, text="断开", width=7, command=self.do_disconnect,
                                  state="disabled")
        self.btn_dis.pack(side="left", padx=2)
        ttk.Button(bar, text="⚙ 设置", width=8, command=self.open_settings).pack(side="right")
        ttk.Button(bar, text="📖 通讯录", width=10,
                   command=self.open_device_book).pack(side="right", padx=2)
        self.status = ttk.Label(bar, text="未连接", foreground="#888")
        self.status.pack(side="left", padx=12)

        tabs = tk.Frame(self)
        tabs.pack(fill="x")
        self.tab_btns = {}
        for key, txt in (("mon", " 实时监视 "), ("pid", " PID 调试 "),
                         ("diag", " 全链路诊断 "), ("param", " 参数 ")):
            b = tk.Button(tabs, text=txt, width=12, relief="raised", bg="#e6e6e6",
                          command=lambda k=key: self.switch(k))
            b.pack(side="left", padx=(6, 1), pady=3)
            self.tab_btns[key] = b

        self.holder = ttk.Frame(self)
        self.holder.pack(fill="both", expand=True)
        self.views = {
            "mon": MonitorView(self.holder, self),
            "pid": PidView(self.holder, self),
            "diag": DiagView(self.holder, self),
            "param": ParamView(self.holder, self),
        }
        self.cur = None
        self.switch("pid")
        self.after(300, self._poll)
        # 启动只"发现并填地址", 不自动连接 —— 避免多遥控器时被抢连到旧的那台
        self.after(600, lambda: self.do_discover(auto_connect=False))

    # ---------- 连接 ----------
    def do_discover(self, auto_connect=True):
        """auto_connect=False 时只发现+填地址, 把"连接"留给用户点
        (启动时用 False; 点【自动发现】按钮时用 True)"""
        self.status.config(text="正在自动发现设备…", foreground="#888")

        def work():
            try:
                return discover_live_addr(
                    self.cfg, prefer_ip=(self.cfg.get("preferred_ip") or ""))
            except Exception as e:
                return "", str(e), []

        def done(res):
            addr, note, cands = res
            if not addr and cands:
                # 多个遥控器 / 无法自动确定 -> 弹窗让用户选
                ip, remember = pick_device(self, cands,
                                           self.cfg.get("preferred_ip") or "",
                                           book_map=self._book_map())
                if not ip:
                    self.status.config(text="已取消选择", foreground="#c60")
                    return
                if remember:
                    self.cfg["preferred_ip"] = ip
                    save_cfg(self.cfg)
                rec = next((c for c in cands if c["ip"] == ip), None)
                if rec and rec["state"] != "device" and rec["port"]:
                    # 离线条目: 先重连一次
                    self.status.config(text="正在连接 %s…" % ip, foreground="#888")
                    adb_run(self.cfg, ["connect", "%s:%s" % (ip, rec["port"])],
                            timeout=8)
                addr = "%s:%s" % (ip, self.cfg.get("mav_port", 5760))
                note = "已选择 %s" % ip
                if rec and rec["port"]:
                    self._register_serial("%s:%s" % (ip, rec["port"]), ip)
            if not addr:
                # 自动发现失败(常见: 无线调试关了) -> 回退到上次成功的桥地址
                fallback = (self.cfg.get("live_addr") or "").strip()
                if fallback:
                    self.status.config(text="⚠ %s；改用上次地址 %s" % (note, fallback),
                                       foreground="#c60")
                    self._connect_switching(fallback, auto_connect)
                    return
                self.status.config(text="⚠ " + str(note), foreground="#c60")
                return
            self.status.config(text="✔ %s → %s" % (note, addr), foreground="#080")
            self._connect_switching(addr, auto_connect)

        def runner():
            r = work()
            self.after(0, lambda: done(r))

        threading.Thread(target=runner, daemon=True).start()

    # ---------- 遥控器通讯录 ----------
    def _book_map(self):
        """{IP: 自定义名} —— 给 pick_device 显示用"""
        m = {}
        for rec in book_all(self.cfg).values():
            if rec.get("ip"):
                m[rec["ip"]] = rec.get("name") or ""
        return m

    def _register_serial(self, serial, ip):
        """把一个 adb 设备登记进通讯录; 新设备询问名字"""
        if not serial:
            return

        def work():
            uid = device_uid(self.cfg, serial)
            self.after(0, lambda: self._after_register(uid, ip))

        threading.Thread(target=work, daemon=True).start()

    def _after_register(self, uid, ip):
        if not uid:
            return
        old = book_all(self.cfg).get(uid) or {}
        self.cfg["active_uid"] = uid
        book_set(self.cfg, uid, ip=ip,
                 conn_addr=self.cfg.get("conn_addr", ""),
                 name=old.get("name") or "")
        if not old.get("name"):
            nm = simpledialog.askstring(
                "识别到新遥控器",
                "序列号 : %s\nIP     : %s\n\n给它起个名字（方便以后区分，可留空）：" % (uid, ip),
                parent=self)
            if nm and nm.strip():
                book_rename(self.cfg, uid, nm.strip())
                self.status.config(text="已登记遥控器「%s」" % nm.strip(),
                                   foreground="#080")
        else:
            self.status.config(text="当前遥控器：%s" % old.get("name"), foreground="#080")

    def open_device_book(self):
        """遥控器通讯录：查看 / 重命名 / 删除 / 设为默认"""
        d = tk.Toplevel(self)
        d.title("遥控器通讯录")
        _center_over(self, d, 640, 380)
        d.transient(self)
        ttk.Label(d, padding=(14, 12, 14, 4),
                  text="已识别的遥控器（名字只存本地，不影响遥控器本身）"
                  ).pack(anchor="w")
        cols = ("名字", "IP", "adb 连接地址", "序列号")
        tv = ttk.Treeview(d, columns=cols, show="headings", height=10)
        for c in cols:
            tv.heading(c, text=c)
        tv.column(cols[0], width=130)
        tv.column(cols[1], width=120)
        tv.column(cols[2], width=170)
        tv.column(cols[3], width=190)
        tv.pack(fill="both", expand=True, padx=14)

        def refresh():
            tv.delete(*tv.get_children())
            recs = book_all(self.cfg)
            for uid, rec in sorted(recs.items(),
                                   key=lambda kv: -(kv[1].get("last_used") or 0)):
                mark = "* " if uid == self.cfg.get("active_uid") else ""
                tv.insert("", "end", iid=uid,
                          values=(mark + (rec.get("name") or "(未命名)"),
                                  rec.get("ip") or "",
                                  rec.get("conn_addr") or "", uid))

        bf = ttk.Frame(d, padding=(14, 8, 14, 14))
        bf.pack(fill="x")

        def sel_uid():
            s = tv.selection()
            return s[0] if s else None

        def do_rename():
            uid = sel_uid()
            if not uid:
                return
            cur = (book_all(self.cfg).get(uid) or {}).get("name") or ""
            nm = simpledialog.askstring("重命名", "新名字：", initialvalue=cur, parent=d)
            if nm is not None:
                book_rename(self.cfg, uid, nm.strip())
                refresh()

        def do_del():
            uid = sel_uid()
            if not uid:
                return
            if messagebox.askyesno("删除", "从通讯录删除这条记录？\n（不影响遥控器本身）",
                                   parent=d):
                book_del(self.cfg, uid)
                if self.cfg.get("active_uid") == uid:
                    self.cfg["active_uid"] = ""
                    save_cfg(self.cfg)
                refresh()

        def do_default():
            uid = sel_uid()
            if not uid:
                return
            rec = book_all(self.cfg).get(uid) or {}
            self.cfg["active_uid"] = uid
            if rec.get("ip"):
                self.cfg["preferred_ip"] = rec["ip"]
            save_cfg(self.cfg)
            refresh()
            self.status.config(text="默认遥控器：%s" % (rec.get("name") or rec.get("ip") or uid),
                               foreground="#080")

        ttk.Button(bf, text="重命名", width=10, command=do_rename).pack(side="left", padx=3)
        ttk.Button(bf, text="设为默认", width=10, command=do_default).pack(side="left", padx=3)
        ttk.Button(bf, text="删除", width=8, command=do_del).pack(side="left", padx=3)
        ttk.Button(bf, text="关闭", width=8, command=d.destroy).pack(side="right")
        ttk.Label(bf, foreground="#888",
                  text="* = 当前默认").pack(side="left", padx=12)
        tv.bind("<Double-Button-1>", lambda ev: do_rename())
        refresh()

    def open_pair_wizard(self):
        """两步向导：①配对(配对码+配对地址) ②连接(无线调试主界面 IP:端口)
        —— 与 G12 遥控器的实际 UI 顺序一致"""
        d = tk.Toplevel(self)
        d.title("手动配置连接（配对 → 连接）")
        _center_over(self, d, 620, 430)
        d.transient(self)
        d.resizable(False, False)
        d.grab_set()        # 模态: 配对期间锁住主窗口, 避免误点【连接】/【断开】

        def _clear():
            for w in d.winfo_children():
                w.destroy()

        def page_pair():
            _clear()
            ttk.Label(d, text="① 配对（一次性）",
                      font=("Microsoft YaHei", 12, "bold")
                      ).pack(anchor="w", padx=18, pady=(16, 2))
            ttk.Label(d, justify="left", text=(
                "开始前请确认：\n"
                "· 遥控器与本电脑连的是同一个 WiFi\n"
                "· 遥控器已开 开发者模式，并已点开『使用配对码配对设备』\n\n"
                "然后从遥控器弹出的配对窗口里，把下面两项填进来："
            )).pack(anchor="w", padx=18)
            f = ttk.Frame(d)
            f.pack(anchor="w", padx=18, pady=10)
            ttk.Label(f, text="配对码:").grid(row=0, column=0, sticky="e", pady=4)
            e_code = ttk.Entry(f, width=36)
            e_code.grid(row=0, column=1, padx=8)
            _digits_only(e_code)
            ttk.Label(f, text="IP 地址和端口:").grid(row=1, column=0, sticky="e", pady=4)
            v_pair = tk.StringVar()
            e_pair = ttk.Entry(f, textvariable=v_pair, width=36)
            e_pair.grid(row=1, column=1, padx=8)
            _addr_mask(e_pair, v_pair, self.cfg.get("default_ip", ""))
            btn = ttk.Button(d, text="确认并配对", width=16)
            btn.pack(anchor="w", padx=18, pady=8)
            ttk.Label(d, foreground="#888",
                      text="配对码每次不同；配对地址里的端口与第二步的不同，别抄错。"
                      ).pack(anchor="w", padx=18)

            def do_pair():
                code = e_code.get().strip()
                a = e_pair.get().strip()
                if not (code and a and _split_addr(a)[1]):
                    messagebox.showwarning("提示", "请填写 配对码 和 IP地址和端口", parent=d)
                    return
                self.cfg["pair_code"] = code
                self.cfg["pair_addr"] = a
                save_cfg(self.cfg)
                btn.config(state="disabled", text="正在配对…")

                def work():
                    rc, out = adb_pair(self.cfg)
                    self.after(0, lambda: after_pair(rc, out))

                threading.Thread(target=work, daemon=True).start()

            def after_pair(rc, out):
                btn.config(state="normal", text="确认并配对")
                if rc == 0 and "failed" not in (out or "").lower():
                    page_conn()
                else:
                    messagebox.showerror("配对失败", out or "(无输出)", parent=d)

            btn.config(command=do_pair)
            e_code.focus_set()

        def page_conn():
            _clear()
            ttk.Label(d, text="② 连接", font=("Microsoft YaHei", 12, "bold")
                      ).pack(anchor="w", padx=18, pady=(16, 2))
            ttk.Label(d, justify="left", text=(
                "配对成功。\n\n"
                "现在回到遥控器『无线调试』主界面（不是配对弹窗），\n"
                "看顶部『IP 地址和端口』（端口与刚才配对不同），填到下面："
            )).pack(anchor="w", padx=18)
            f = ttk.Frame(d)
            f.pack(anchor="w", padx=18, pady=10)
            ttk.Label(f, text="IP 地址和端口:").grid(row=0, column=0, sticky="e")
            saved = (self.cfg.get("conn_addr") or "").strip()
            base = (self.cfg.get("default_ip") or "").strip().rstrip(".:")
            init = saved if ":" in saved else ((base + ":") if base else "")
            v_conn = tk.StringVar(value=init)
            e_conn = ttk.Entry(f, textvariable=v_conn, width=36)
            e_conn.grid(row=0, column=1, padx=8)
            e_conn.icursor(len(init))
            _digits_only(e_conn)
            _entry_protect(e_conn, v_conn)
            btn = ttk.Button(d, text="连接", width=16)
            btn.pack(anchor="w", padx=18, pady=8)
            mem = {}

            def do_conn():
                a = e_conn.get().strip()
                if not a or not _split_addr(a)[1]:
                    messagebox.showwarning("提示", "请填写 IP地址和端口", parent=d)
                    return
                mem["addr"] = a
                self.cfg["conn_addr"] = a
                self.cfg["default_ip"] = _split_addr(a)[0]
                save_cfg(self.cfg)
                btn.config(state="disabled", text="正在连接…")

                def work():
                    rc, out = adb_connect(self.cfg)
                    self.after(0, lambda: after_conn(rc, out))

                threading.Thread(target=work, daemon=True).start()

            def after_conn(rc, out):
                btn.config(state="normal", text="连接")
                if rc != 0 or "failed" in (out or "").lower():
                    messagebox.showerror(
                        "连接失败",
                        (out or "(无输出)") + "\n\n请确认：\n· 遥控器与本电脑同一 WiFi\n"
                        "· 遥控器『无线调试』仍保持打开", parent=d)
                    return
                ip = _split_addr(mem.get("addr", ""))[0]
                live = "%s:%s" % (ip, self.cfg.get("mav_port", 5760))
                d.destroy()                     # 先关向导, 释放模态
                # 登记进通讯录（新设备会询问名字）
                self._register_serial(mem.get("addr", ""), ip)
                self.status.config(text="adb 已连上 %s" % mem.get("addr", ""),
                                   foreground="#080")
                self._connect_switching(live)

            btn.config(command=do_conn)
            e_conn.focus_set()

        page_pair()

    def do_scan(self):
        """扫描局域网 5760 端口找飞控桥 —— 不需要 adb / 无线调试。
        第一遍扫本机网段；没找到则深扫常见手机热点/路由器网段。"""
        port = int(self.cfg.get("mav_port", 5760))
        extra = []
        for s in (self.cfg.get("live_addr") or "", self.cfg.get("default_ip") or ""):
            s = (s or "").strip()
            ip = s.split(":")[0]
            if ip.count(".") == 3:
                extra.append(ip.rsplit(".", 1)[0] + ".")
        self.status.config(text="正在扫描本机网段 %s…" % (" ".join(local_subnets()) or "?"),
                           foreground="#888")

        def work():
            hosts = scan_lan(port, extra)
            if hosts:
                return hosts, False
            self.after(0, lambda: self.status.config(
                text="本机网段没找到，正在深扫常见热点/路由器网段…(约 20 秒)", foreground="#888"))
            return scan_lan(port, extra, deep=True), True

        def done(res):
            hosts, was_deep = res
            if not hosts:
                self.status.config(
                    text="⚠ 没找到 MAVLink 桥。野外请检查：① 电脑和遥控器是否连【同一个】手机热点 "
                         "② 热点是否开了『客户端隔离/AP 隔离』 ③ 电脑是否用的 USB 共享(网段不同)。",
                    foreground="#c60")
                return
            tag = "深扫" if was_deep else "扫描"
            if len(hosts) == 1:
                ip = hosts[0]
            else:
                # 同网多台都在监听 5760 -> 让用户选（显示通讯录里的名字）
                cands = [{"ip": h, "port": str(port), "name": "", "state": "device"}
                         for h in hosts]
                ip, remember = pick_device(self, cands,
                                           self.cfg.get("preferred_ip") or "",
                                           book_map=self._book_map())
                if not ip:
                    self.status.config(text="%s到 %d 个，已取消选择" % (tag, len(hosts)),
                                       foreground="#c60")
                    return
                if remember:
                    self.cfg["preferred_ip"] = ip
                    save_cfg(self.cfg)
            addr = "%s:%d" % (ip, port)
            self.status.config(text="✔ %s到 %d 个: %s → %s"
                                    % (tag, len(hosts), ", ".join(hosts), addr),
                               foreground="#080")
            self._connect_switching(addr, True)

        def runner():
            r = work()
            self.after(0, lambda: done(r))

        threading.Thread(target=runner, daemon=True).start()

    def do_usb(self):
        # USB 偶发掉线: 失败时自动重启 adb server 并重试, 最多 3 轮
        for attempt in range(3):
            usb = self._usb_dev()
            if usb:
                break
            if attempt < 2:
                adb_run(self.cfg, ["kill-server"])
                adb_run(self.cfg, ["start-server"])
                time.sleep(1.0)
        if not usb:
            messagebox.showwarning(APP_NAME,
                                   "未检测到 USB 设备。\n请用数据线连接遥控器、开启 USB 调试，"
                                   "并在遥控器上允许『USB 调试授权』。")
            return
        for port in range(15760, 15800):
            c2, o2 = adb_run(self.cfg, ["-s", usb, "forward", "tcp:%d" % port, "tcp:5760"])
            if c2 == 0 and "error" not in (o2 or "").lower():
                self._usb = (usb, port)
                self.status.config(text="USB 已转发 5760 → 本机 %d" % port, foreground="#080")
                self._connect_switching("127.0.0.1:%d" % port)
                return
        messagebox.showerror(APP_NAME, "adb forward 失败")

    def _usb_dev(self):
        """返回当前 USB 物理设备的 adb serial; 无则空串"""
        code, out = adb_devices(self.cfg)
        for ln in (out or "").splitlines()[1:]:
            p = ln.split()
            if len(p) >= 2 and p[1] == "device" and ":" not in p[0]:
                return p[0]
        return ""

    def _connect_switching(self, new_addr, auto_connect=True):
        """把目标切到 new_addr。已连接别的目标 -> 先断开再连（避免静默失败）"""
        new_addr = (new_addr or "").strip()
        if not new_addr:
            return
        old = (self.cfg.get("live_addr") or "").strip()
        if self.feed and self.feed.is_alive():
            if new_addr == old:
                self.addr.set(new_addr)
                self.status.config(text="已连接 %s（目标未变）" % new_addr,
                                   foreground="#080")
                return
            self.status.config(text="目标已改为 %s，正在断开旧连接…" % new_addr,
                               foreground="#888")
            self.do_disconnect()
        self.addr.set(new_addr)
        if auto_connect:
            self.do_connect()
        else:
            self.cfg["live_addr"] = new_addr
            save_cfg(self.cfg)

    def do_connect(self):
        if self.feed and self.feed.is_alive():
            return
        dev = self.addr.get().strip()
        if not dev:
            messagebox.showwarning(APP_NAME, "请填写飞控地址，例如 192.168.10.162:5760")
            return
        csv_path = os.path.join(LOG_DIR, "usv_log_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        self.feed = LiveFeed(dev, self.cfg, self.q, csv_path=csv_path)
        self.feed.start()
        self.cfg["live_addr"] = dev
        save_cfg(self.cfg)
        self.status.config(text="连接中… %s" % dev, foreground="#888")

    def do_disconnect(self):
        if self.feed:
            self.feed.stop()
            self.feed.close_csv()
            self.feed = None
        if self._usb:
            try:
                adb_run(self.cfg, ["-s", self._usb[0], "forward", "--remove",
                                   "tcp:%d" % self._usb[1]])
            except Exception:
                pass
            self._usb = None
        self.status.config(text="已断开", foreground="#888")
        self.btn_dis.config(state="disabled")

    def shutdown(self):
        if self.feed:
            self.feed.stop()
            self.feed.close_csv()
        if self._usb:
            try:
                adb_run(self.cfg, ["-s", self._usb[0], "forward", "--remove",
                                   "tcp:%d" % self._usb[1]])
            except Exception:
                pass

    def _poll(self):
        f = self.feed
        if f is not None:
            if f.error:
                self.status.config(text="⚠ " + f.error, foreground="#c00")
                self.btn_dis.config(state="disabled")
            elif f.connected and not f.stop_flag:
                with f.lock:
                    np = sum(len(v) for v in f.pid_hist.values())
                    md = f.mode
                extra = ""
                if md is not None:
                    extra = "  |  %s" % MODE_NAMES.get(md, "模式%d" % md)
                if f.boost_note:
                    extra += "  |  " + f.boost_note
                self.status.config(text="✔ 已连接 · 实时刷新中 · PID样本 %d%s" % (np, extra),
                                   foreground="#080" if md != 0 else "#c60")
                self.btn_dis.config(state="normal")
            ls = f.last_seg
            if ls is not None and ls[0] != self._seg_shown:
                self._seg_shown = ls[0]
                try:
                    self.views["diag"].show_segment(*ls)
                except Exception as e:
                    errlog("show_segment: %r" % (e,))
        self.after(300, self._poll)

    # ---------- 页切换 ----------
    def switch(self, key):
        for k, v in self.views.items():
            v.pack_forget()
        self.views[key].pack(fill="both", expand=True)
        for k, b in self.tab_btns.items():
            on = (k == key)
            b.config(relief="sunken" if on else "raised",
                     bg="#cfdfff" if on else "#e6e6e6")
        self.cur = key
        if hasattr(self.views[key], "on_shown"):
            self.views[key].on_shown()

    # ---------- 设置 ----------
    def open_settings(self):
        d = tk.Toplevel(self)
        d.title("设置")
        d.transient(self)
        try:
            self.update_idletasks()
            d.geometry("+%d+%d" % (self.winfo_rootx() + 60, self.winfo_rooty() + 60))
        except Exception:
            pass
        c = dict(self.cfg)
        ent = {}
        rows = [
            ("adb 路径 (留空=自动查找)", "adb_path"),
            ("无线调试 · 配对地址 IP:端口  [一次性]", "pair_addr"),
            ("无线调试 · 配对码  [一次性]", "pair_code"),
            ("无线调试 · 连接地址 IP:端口  [每次]", "conn_addr"),
            ("默认 IP (固定预填用)", "default_ip"),
            ("adb 串号 serial (多设备时指定, 可留空)", "serial"),
            ("优先遥控器 IP (多设备时默认选它, 可留空)", "preferred_ip"),
            ("日志存储目录", "storage_dir"),
            ("打杆动作识别阈值 (PWM)", "action_thresh"),
            ("实时监视 RC 通道 (1-16, 逗号分隔, 如 1,3)", "rc_channels"),
            ("实时监视电机通道 (SERVO1-16, 逗号分隔, 如 1,3)", "servo_channels"),
        ]
        for i, (lab, key) in enumerate(rows):
            ttk.Label(d, text=lab).grid(row=i, column=0, sticky="e", padx=8, pady=3)
            v = tk.StringVar(value=str(c.get(key, "")))
            ent[key] = v
            ttk.Entry(d, textvariable=v, width=44).grid(row=i, column=1, sticky="w", padx=6, pady=3)
        ttk.Label(d, text="提示: 遥控器『无线调试』主界面顶部有【IP地址和端口】；\n"
                          "首次需在『使用配对码配对设备』弹窗里获取配对地址与配对码。\n"
                          "通道配置: RC 通道填 CH 编号, 电机通道填 SERVO 编号, 逗号分隔。",
                  foreground="#666", justify="left").grid(row=len(rows), column=0,
                                                          columnspan=2, sticky="w",
                                                          padx=8, pady=(6, 0))

        def cur_cfg():
            cc = dict(c)
            for k in ent:
                v = ent[k].get()
                cc[k] = int(v) if k == "action_thresh" and v.strip().isdigit() else v
            return cc

        box = ttk.Frame(d)
        box.grid(row=len(rows) + 1, column=0, columnspan=2, sticky="w", padx=8, pady=8)
        self._dlg_out = tk.StringVar(value="")
        ttk.Button(box, text="配对(ADB)", command=lambda: self._settings_cmd("pair")).pack(side="left")
        ttk.Button(box, text="连接(ADB)", command=lambda: self._settings_cmd("connect")).pack(side="left", padx=6)
        ttk.Button(box, text="列出设备", command=lambda: self._settings_cmd("devices")).pack(side="left", padx=6)

        def ok():
            cc = cur_cfg()
            self.cfg.update(cc)
            save_cfg(self.cfg)
            d.destroy()
            messagebox.showinfo(APP_NAME, "设置已保存。")

        ttk.Button(box, text="保存", command=ok).pack(side="left", padx=6)
        ttk.Button(box, text="关闭", command=d.destroy).pack(side="left", padx=6)
        ttk.Label(d, textvariable=self._dlg_out, foreground="#06c", wraplength=520,
                  justify="left").grid(row=len(rows) + 2, column=0, columnspan=2,
                                       sticky="w", padx=8, pady=(0, 8))
        self._dlg_cfg = cur_cfg
        self._dlg_win = d

    def _settings_cmd(self, what):
        cc = self._dlg_cfg()
        self.cfg.update(cc)
        save_cfg(self.cfg)

        def work():
            if what == "pair":
                code, out = adb_pair(cc)
            elif what == "connect":
                code, out = adb_connect(cc)
            else:
                code, out = adb_devices(cc)
            return "rc=%s\n%s" % (code, (out or "").strip()[:600])

        def done(txt):
            try:
                self._dlg_out.set(txt)
            except Exception:
                pass

        def runner():
            r = work()
            self.after(0, lambda: done(r))

        threading.Thread(target=runner, daemon=True).start()


# ============================================================ 入口
def probe(addr, sec=6.0):
    cfg = load_cfg()
    q = queue.Queue()
    feed = LiveFeed(addr, cfg, q)
    feed.start()
    t0 = time.time()
    while time.time() - t0 < sec:
        if feed.error:
            print("错误:", feed.error)
            feed.stop()
            return 1
        time.sleep(0.3)
    with feed.lock:
        n_rc = sum(len(feed.buf[k]) for k in feed.buf if k.startswith("rc"))
        n_out = sum(len(feed.buf[k]) for k in feed.buf if k.startswith("s"))
        n_pid = sum(len(v) for v in feed.pid_hist.values())
        n_att = len(feed.att)
        n_hud = len(feed.hud)
        n_p = len(feed.params)
        hb = feed.n_hb
        mode = feed.mode
    print("OK  RC=%d  OUT=%d  PID_TUNING=%d (原始%d)  ATTITUDE=%d  VFR_HUD=%d  PARAM=%d  HB=%d  mode=%s"
          % (n_rc, n_out, n_pid, feed.n_pid_raw, n_att, n_hud, n_p, hb, mode))
    print("    轴: %s" % (", ".join("axis%d=%d条" % (k, len(v))
                                    for k, v in sorted(feed.pid_hist.items())) or "(无)"))
    print("    其他: NED=%d  GPS=%d  EKF=%d  POS=%d  MCUR=%d"
          % (len(feed.vel), len(feed.gps), len(feed.ekf), len(feed.pos), len(feed.mcur)))
    print("    航线: %d 点  %s"
          % (len(feed.mission),
             " ".join("#%d%s" % (m[0], MISSION_CMD.get(m[3], "/%d" % m[3]))
                      for m in feed.mission[:10]) or "(无)"))
    if feed.pos:
        p = feed.pos[-1]
        print("    位置: %.7f, %.7f  航向 %.1f°" % (p[1], p[2], p[6] / 100.0))
    print("    提速: %s" % (feed.boost_note or "(未提速)"))
    feed.stop()
    feed.close_csv()
    return 0


def uitest(sec=16.0):
    """界面自检：真开窗口，遍历所有页面 + 跑一次诊断，打印结果后自动关闭"""
    root = tk.Tk()
    root.title("uitest")
    try:
        root.geometry("1200x860")
    except Exception:
        pass
    cfg = load_cfg()
    view = MainView(root, cfg)
    view.pack(fill="both", expand=True)
    res = {}

    def on_err(exc, tb):
        errlog("Tk回调异常: %s" % (exc,))
        res.setdefault("tk_errors", []).append(str(exc))

    root.report_callback_exception = on_err

    def step_switch():
        for k in ("mon", "pid", "diag", "param"):
            try:
                view.switch(k)
                root.update()
                res[k] = "ok"
            except Exception as e:
                res[k] = "FAIL %r" % (e,)

    def step_diag():
        v = view.views["diag"]
        try:
            v.run_diag()
            root.update()
            res["diag_run"] = "ok (报告 %d 字)" % len(v.last_report)
        except Exception as e:
            res["diag_run"] = "FAIL %r" % (e,)
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(os.path.join(LOG_DIR, "uitest_diag.txt"), "w", encoding="utf-8") as f:
                f.write(v.last_report)
        except Exception:
            pass
        f = view.feed
        if f is not None:
            with f.lock:
                res["feed"] = ("rc=%d out=%d pid=%d att=%d hud=%d param=%d hb=%d mode=%s"
                               % (sum(len(f.buf[k]) for k in f.buf if k.startswith("rc")),
                                  sum(len(f.buf[k]) for k in f.buf if k.startswith("s")),
                                  sum(len(x) for x in f.pid_hist.values()),
                                  len(f.att), len(f.hud), len(f.params), f.n_hb, f.mode))
            res["feed_err"] = f.error

    def step_zoom():
        try:
            pv = view.views["pid"]
            for _f, _c, p in pv._plots.values():
                p.set_zoom(1.0, 5.0)
                p.redraw()
            root.update()
            res["pid_zoom"] = "ok"
        except Exception as e:
            res["pid_zoom"] = "FAIL %r" % (e,)

    root.after(1200, step_switch)
    root.after(4000, step_zoom)
    root.after(int(sec * 1000) - 800, step_diag)
    root.after(int(sec * 1000), root.destroy)
    root.mainloop()
    try:
        view.shutdown()
    except Exception:
        pass
    print("---- 界面自检结果 ----")
    for k in ("mon", "pid", "diag", "param", "pid_zoom", "diag_run", "feed", "feed_err"):
        if k in res:
            print("  %-10s %s" % (k, res[k]))
    if res.get("tk_errors"):
        print("  Tk异常  : %s" % res["tk_errors"])
    try:
        with open(os.path.join(LOG_DIR, "usv_err.log"), "r", encoding="utf-8") as f:
            tail = f.read().strip().splitlines()[-8:]
        if tail:
            print("  usv_err.log 末尾:")
            for ln in tail:
                print("    " + ln)
    except Exception:
        pass


def main():
    root = tk.Tk()
    root.title("%s v%s" % (APP_NAME, VERSION))
    try:
        root.geometry("1200x860")
    except Exception:
        pass
    for ic in ("usv_icon.ico", "g12_icon.ico"):
        p = os.path.join(HERE, ic)
        if os.path.exists(p) and sys.platform.startswith("win"):
            try:
                root.iconbitmap(p)
                break
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
    cfg = load_cfg()
    view = MainView(root, cfg)
    view.pack(fill="both", expand=True)

    def on_close():
        view.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    argv = sys.argv[1:]
    if argv and argv[0] == "--probe":
        _c = load_cfg()
        _a = argv[1] if len(argv) > 1 else \
            (_c.get("live_addr") or "%s:%s" % (_c.get("default_ip"), _c.get("mav_port", 5760)))
        sys.exit(probe(_a, float(argv[2]) if len(argv) > 2 else 6.0))
    elif argv and argv[0] == "--addr":
        _c = load_cfg()
        _c["live_addr"] = argv[1]
        save_cfg(_c)
        print("已保存 live_addr =", argv[1])
        sys.exit(0)
    elif argv and argv[0] == "--uitest":
        uitest(float(argv[1]) if len(argv) > 1 else 16.0)
        sys.exit(0)
    elif argv and argv[0] == "--scan":
        _c = load_cfg()
        _p = int(argv[1]) if len(argv) > 1 else int(_c.get("mav_port", 5760))
        _deep = len(argv) > 2 and argv[2] == "deep"
        print("本机候选网段:", local_subnets())
        print("ipconfig 所有网卡 IPv4:", _ipconfig_ipv4())
        print("开始扫描 (deep=%s) …" % _deep)
        _h = scan_lan(_p, [], deep=_deep)
        print("确认是 MAVLink 桥的地址 (%d 个):" % len(_h))
        for x in _h:
            print("   %s:%d" % (x, _p))
        sys.exit(0)
    else:
        main()
