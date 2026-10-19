# -*- coding: utf-8 -*-
"""全量参数审计：拉取飞控全部参数，按「影响转向/航迹精度」的类别分组列出。
只读。"""
import sys
import time

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import usv_debug as U          # 导入即启用 pymavlink 加固补丁
from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.162:5760"
m = mavutil.mavlink_connection("tcp:" + ADDR, timeout=3)
m.wait_heartbeat(timeout=8)

allp = {}
m.mav.param_request_list_send(1, 1)
t0 = time.time()
while time.time() - t0 < 40:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg is None:
        continue
    nm = msg.param_id.replace("\x00", "").strip()
    allp[nm] = msg.param_value
print("拉取到参数总数 =", len(allp))
print()

GROUPS = [
    ("转向内环 ATC_STR_RAT", ["ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
                              "ATC_STR_RAT_FF", "ATC_STR_RAT_FLTT", "ATC_STR_RAT_FLTD",
                              "ATC_STR_RAT_FLTE", "ATC_STR_RAT_IMAX", "ATC_STR_RAT_MAX",
                              "ATC_STR_RAT_SMAX"]),
    ("外环/角度", ["ATC_STR_ANG_P", "ATC_STR_ACC_MAX", "ATC_STR_RATE_MAX",
                   "ATC_STR_ANG_RATE", "ATC_STR_ANG_P"]),
    ("速度环 ATC_SPEED", ["ATC_SPEED_P", "ATC_SPEED_I", "ATC_SPEED_D", "ATC_SPEED_FF",
                          "ATC_SPEED_IMAX", "ATC_SPEED_FILT", "ATC_SPEED_FLTT",
                          "ATC_SPEED_FLTD", "ATC_SPEED_FF_MAX", "ATC_SPEED_P_MAX"]),
    ("★导航层 NAVL1 / WP", ["NAVL1_PERIOD", "NAVL1_DAMPING", "NAVL1_XTRACK_I",
                            "WP_SPEED", "WP_ACCEL", "WP_JERK", "WP_RADIUS",
                            "WP_OVERSHOOT", "WP_PIVOT_ANGLE", "WP_PIVOT_RATE",
                            "WP_TURN_PCT", "WP_LOITER_RAD", "TURN_RADIUS",
                            "ACRO_TURN_RATE", "LOIT_SPEED", "RTL_SPEED",
                            "RTL_LOIT_TIME", "MIS_TOTAL"]),
    ("★★混控/电机(差速船命门)", ["MOT_STR_THR_MIX", "MOT_SLEWRATE", "MOT_THR_MIN",
                                 "MOT_THR_MAX", "MOT_PWM_TYPE", "MOT_PWM_MIN",
                                 "MOT_PWM_MAX", "MOT_PWM_FREQ", "MOT_SPIN_MIN",
                                 "MOT_SPIN_ARM", "MOT_OPTIONS", "MOT_SAFE_DISARM",
                                 "MOT_BOOST_SCALE", "MOT_VEC_ANGLEMAX"]),
    ("★★伺服行程/中位/反向", ["SERVO1_FUNCTION", "SERVO1_MIN", "SERVO1_TRIM",
                               "SERVO1_MAX", "SERVO1_REVERSED",
                               "SERVO2_FUNCTION", "SERVO2_MIN", "SERVO2_TRIM",
                               "SERVO2_MAX", "SERVO2_REVERSED",
                               "SERVO3_FUNCTION", "SERVO3_MIN", "SERVO3_TRIM",
                               "SERVO3_MAX", "SERVO3_REVERSED",
                               "SERVO4_FUNCTION", "SERVO4_MIN", "SERVO4_TRIM",
                               "SERVO4_MAX", "SERVO4_REVERSED"]),
    ("★传感器/估计器", ["INS_GYRO_FILTER", "INS_ACCEL_FILTER", "INS_USE", "INS_USE2",
                        "INS_USE3", "AHRS_EKF_TYPE", "EK3_ENABLE", "EK3_IMU_MASK",
                        "EK3_GBIAS_P_NSE", "EK3_ABIAS_P_NSE", "EK3_ACC_P_NSE",
                        "EK3_YAW_I_GATE", "EK3_MAG_CAL", "EK3_SRC1_YAW",
                        "EK3_SRC1_POSXY", "GPS_TYPE", "GPS_HDOP_GOOD",
                        "GPS_GNSS_MODE"]),
    ("★★磁罗盘(航向精度命门)", ["COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",
                                 "COMPASS_DEC", "COMPASS_OFS_X", "COMPASS_OFS_Y",
                                 "COMPASS_OFS_Z", "COMPASS_DIA_X", "COMPASS_ODI_X",
                                 "COMPASS_MOTCT", "COMPASS_MOT_X", "COMPASS_EXTERNAL",
                                 "COMPASS_LEARN", "COMPASS_AUTODEC",
                                 "COMPASS_CAL_FIT"]),
    ("其他/保护", ["ARMING_CHECK", "FS_ACTION", "FS_TIMEOUT", "FS_THR_ENABLE",
                   "FENCE_ENABLE", "CRUISE_SPEED", "CRUISE_THROTTLE",
                   "GCS_PID_MASK", "SCR_ENABLE", "LOG_BITMASK"]),
]

for title, names in GROUPS:
    print("=== %s ===" % title)
    miss = []
    for nm in names:
        if nm in allp:
            print("   %-24s = %g" % (nm, allp[nm]))
        else:
            miss.append(nm)
    if miss:
        print("   (不存在: %s)" % ", ".join(miss))
    print()
