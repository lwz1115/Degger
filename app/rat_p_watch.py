#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rat_p_watch.py —— 只读监视 RAT(转向角速率)环 PID_TUNING, 验证"改 P 立即生效"

用法(Windows):
    python rat_p_watch.py                 # 默认 USB COM14
    python rat_p_watch.py COM15           # 指定串口
    python rat_p_watch.py tcp:192.168.10.162:5760   # 经遥控器桥

功能(全程只读, 不写飞控):
  · 实时打印 [时间] 模式 desired achieved P I D FF
  · 每 2 秒读一次 ATC_STR_RAT_P/I/D/MAX, 参数一变就打印醒目提示(标注改值时刻)
  · 误差足够大时给出"有效增益 ≈ P输出/(desired-achieved)", 便于对比参数值
Ctrl+C 结束。
"""
import sys
import time
from pymavlink import mavutil

DEV = sys.argv[1] if len(sys.argv) > 1 else "COM14"
AXIS = 5                      # 5 = Steering(转向)
WATCH = ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
         "ATC_STR_RAT_FF", "ATC_STR_RAT_MAX"]
READ_PARAM_EVERY = 2.0        # 秒

MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER",
         6: "FOLLOW", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 15: "GUIDED"}


def connect(dev):
    if dev.startswith("tcp:"):
        return mavutil.mavlink_connection(dev, timeout=2)
    return mavutil.mavlink_connection(dev, baud=115200, timeout=1)


def main():
    m = connect(DEV)
    print("连接 %s ... (只读监视, Ctrl+C 结束)" % DEV)
    try:
        m.wait_heartbeat(timeout=8)
    except Exception:
        print("未收到心跳, 检查连接/端口")
    print("%-12s %-8s %8s %8s %9s %8s %8s %8s   | 有效增益~P/(err)"
          % ("time", "mode", "desired", "achiev", "P_out", "I_out", "D_out", "FF_out"))

    mode = -1
    last_params = {}
    t_param = 0.0
    try:
        while True:
            now = time.time()
            # --- 周期读参数, 检测变化 ---
            if now - t_param >= READ_PARAM_EVERY:
                t_param = now
                for name in WATCH:
                    try:
                        m.mav.param_request_read_send(1, 1, name.encode(), -1)
                    except Exception:
                        pass
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            ty = msg.get_type()
            if ty == "BAD_DATA" or ty == "UNKNOWN":
                continue
            if ty == "HEARTBEAT":
                cm = getattr(msg, "custom_mode", None)
                if cm is not None and cm != mode:
                    mode = cm
                    print("  >> 模式切换 -> %s(%s)" % (MODES.get(mode, "?"), mode))
            elif ty == "PARAM_VALUE":
                nm = msg.param_id.replace("\x00", "").strip()
                if nm in WATCH:
                    v = msg.param_value
                    old = last_params.get(nm)
                    last_params[nm] = v
                    if old is not None and abs(v - old) > max(1e-6, abs(old) * 1e-5):
                        print("  ** 参数变化 %-16s %s -> %s" % (nm, ("%.6g" % old), ("%.6g" % v)))
            elif ty == "PID_TUNING" and getattr(msg, "axis", -1) == AXIS:
                des, ach = msg.desired, msg.achieved
                p, i, d, ff = msg.P, msg.I, msg.D, msg.FF
                err = des - ach
                gain = (p / err) if abs(err) > 0.5 else None   # 误差太小不算
                tm = time.strftime("%H:%M:%S")
                gtxt = ("%+9.4f" % gain) if gain is not None else "        -"
                print("%-12s %-8s %8.2f %8.2f %9.4f %8.4f %8.4f %8.3f   | %s"
                      % (tm, MODES.get(mode, str(mode)), des, ach, p, i, d, ff, gtxt))
    except KeyboardInterrupt:
        print("\n结束。")


if __name__ == "__main__":
    main()
