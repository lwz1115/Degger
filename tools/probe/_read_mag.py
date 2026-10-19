# -*- coding: utf-8 -*-
import sys, time
from pymavlink import mavutil
addr = sys.argv[1]
NAMES = ["COMPASS_USE","COMPASS_USE2","COMPASS_USE3","COMPASS_DEV_ID",
         "COMPASS_DEV_ID2","COMPASS_DEV_ID3","COMPASS_PRIO1_ID","COMPASS_PRIO2_ID",
         "COMPASS_PRIO3_ID","COMPASS_OFS_X","COMPASS_OFS_X2","COMPASS_OFS_X3",
         "COMPASS_DIA_X","COMPASS_DIA_X2","COMPASS_DIA_X3","COMPASS_MOT_X",
         "COMPASS_MOT_X2","COMPASS_MOT_X3","COMPASS_EXTERNAL",
         "COMPASS_EXTERN2","COMPASS_EXTERN3","COMPASS_ORIENT","COMPASS_ORIENT2",
         "COMPASS_ORIENT3","MAG_ENABLE","COMPASS_CAL_FIT","COMPASS_TYPEMASK",
         "EK3_SRC1_YAW","EK3_SRC1_POSXY","AHRS_EKF_TYPE","ARMING_MAGTHRESH",
         "ARMING_ACCTHRESH","ARMING_OPTIONS","ARMING_SKIPCHK","INS_GYROFFS_X",
         "LOG_BACKEND_TYPE","LOG_BITMASK","SCHED_LOOP_RATE"]
want=set(NAMES); got={}
m = mavutil.mavlink_connection("tcp:"+addr, source_system=255)
if not m.wait_heartbeat(timeout=10): print("无 HEARTBEAT"); sys.exit(1)
t0=time.time(); last=0
while time.time()-t0 < 30 and len(got) < len(NAMES):
    if time.time()-t0-last > 4.0:
        last=time.time()-t0
        for nm in NAMES:
            if nm not in got:
                try: m.mav.param_request_read_send(1,1,nm.encode(),-1)
                except Exception: pass
        time.sleep(0.05)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
    if msg is None: continue
    nm = msg.param_id
    if isinstance(nm,bytes): nm = nm.decode(errors="replace")
    nm = nm.replace("\x00","").strip()
    if nm in want and nm not in got: got[nm]=float(msg.param_value)
for nm in NAMES:
    print("   %-20s = %s" % (nm, got.get(nm,"(无此参数)")))
