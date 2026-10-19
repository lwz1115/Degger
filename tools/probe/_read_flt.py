import sys, time
from pymavlink import mavutil
addr = sys.argv[1]
NAMES = ["ATC_STR_RAT_P","ATC_STR_RAT_I","ATC_STR_RAT_D","ATC_STR_RAT_FF",
         "ATC_STR_RAT_IMAX","ATC_STR_RAT_FLTD","ATC_STR_RAT_FLTE","ATC_STR_RAT_FLTT",
         "ATC_STR_RAT_SMAX","ATC_STR_RAT_MAX","ATC_STR_RAT_PDMX","ATC_STR_RAT_NEF",
         "ATC_STR_ANG_P","ATC_STR_ACC_MAX","ATC_STR_DEC_MAX","MOT_STR_THR_MIX"]
want=set(NAMES); got={}
m = mavutil.mavlink_connection("tcp:"+addr, source_system=255)
if not m.wait_heartbeat(timeout=8): print("无心跳(船可能已断电)"); sys.exit(1)
t0=time.time(); last=0
while time.time()-t0 < 25 and len(got)<len(NAMES):
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
for nm in NAMES:
    print("   %-20s = %s" % (nm, got.get(nm,"(未取到)")))
