# -*- coding: utf-8 -*-
"""临时测试：能否用 SET_MESSAGE_INTERVAL 让飞控开始发 NAV_CONTROLLER_OUTPUT。
只发流速率请求，不改任何持久参数；结束前恢复默认。"""
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)

MSG = mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT
print("NAV_CONTROLLER_OUTPUT 消息 ID =", MSG)


def set_interval(us):
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            MSG, us, 0, 0, 0, 0, 0)


# 1) 默认状态：6 秒内收到几条
t0 = time.time()
n = 0
last = None
while time.time() - t0 < 6:
    msg = m.recv_match(type="NAV_CONTROLLER_OUTPUT", blocking=True, timeout=1)
    if msg:
        n += 1
        last = msg
print("请求前 6 秒收到: %d 条" % n)

# 2) 请求 5 Hz
set_interval(200000)
t0 = time.time()
n2 = 0
ack = None
while time.time() - t0 < 8:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    ty = msg.get_type()
    if ty == "NAV_CONTROLLER_OUTPUT":
        n2 += 1
        last = msg
    elif ty == "COMMAND_ACK":
        ack = msg
print("请求后 8 秒收到: %d 条  (%.2f Hz)" % (n2, n2 / 8.0))
if ack is not None:
    print("COMMAND_ACK: cmd=%s result=%s" % (ack.command, ack.result))
else:
    print("COMMAND_ACK: 未收到")

if last is not None:
    print("样例字段:")
    for f in ("nav_roll", "nav_pitch", "nav_bearing", "target_bearing",
              "wp_dist", "xtrack_error"):
        print("   %-16s = %s" % (f, getattr(last, f, None)))

# 3) 恢复默认
set_interval(-1)
print("已发送 interval=-1 恢复默认(重启也会恢复)")
