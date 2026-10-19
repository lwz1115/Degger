import sys, time
from pymavlink import mavutil
NAMES = ["PSC_POS_P","PSC_POS_P_DEFAULT","PSC_VEL_P","PSC_VEL_I","PSC_VEL_D",
         "PSC_VEL_IMAX","PSC_VEL_FLTE","PSC_VEL_FLTD","PSC_VEL_FF",
         "PSC_ACCZ_P","PSC_ACCZ_I","PSC_ACCZ_D","PSC_JERK_XY","PSC_JERK_Z",
         "ATC_TURN_MAX_G","TURN_RADIUS","WP_RADIUS","WP_OVERSHOOT",
         "WP_SPEED","WP_ACCEL","WP_JERK","WP_PIVOT_ANGLE","WP_PIVOT_RATE",
         "WP_PIVOT_DELAY"]
addr = sys.argv[1]
want=set(NAMES); got={}
m = mavutil.mavlink_connection("tcp:"+addr, source_system=255)
if not m.wait_heartbeat(timeout=8): print("无心跳"); sys.exit(1)
t0=time.time(); last=0
while time.time()-t0 < 35 and len(got)<len(NAMES):
    if time.time()-t0-last > 3.0:
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
print("--- 位置控制器(外环, S型走位的嫌疑) ---")
for nm in ("PSC_POS_P","PSC_VEL_P","PSC_VEL_I","PSC_VEL_D","PSC_VEL_IMAX",
           "PSC_VEL_FLTE","PSC_VEL_FLTD","PSC_VEL_FF"):
    print("   %-18s = %s" % (nm, got.get(nm,"(未取到)")))
print("--- 导航几何 ---")
for nm in ("ATC_TURN_MAX_G","TURN_RADIUS","WP_RADIUS","WP_OVERSHOOT",
           "WP_SPEED","WP_ACCEL","WP_JERK","WP_PIVOT_ANGLE","WP_PIVOT_RATE","WP_PIVOT_DELAY"):
    print("   %-18s = %s" % (nm, got.get(nm,"(未取到)")))
