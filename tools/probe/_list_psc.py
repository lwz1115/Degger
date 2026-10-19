import sys, time
from pymavlink import mavutil
addr = sys.argv[1]
pfx = (sys.argv[2] if len(sys.argv) > 2 else "PSC_").split(",")
m = mavutil.mavlink_connection("tcp:"+addr, source_system=255)
if not m.wait_heartbeat(timeout=8): print("无心跳"); sys.exit(1)
m.mav.param_request_list_send(1, 1)
found = {}
t0 = time.time(); expect = None
while time.time()-t0 < 90:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
    if msg is None: break
    nm = msg.param_id
    if isinstance(nm, bytes): nm = nm.decode(errors="replace")
    nm = nm.replace("\x00", "").strip()
    if msg.param_count: expect = int(msg.param_count)
    if any(nm.startswith(p) for p in pfx):
        found[nm] = (float(msg.param_value), int(msg.param_index), int(msg.param_count))
    if expect and msg.param_index == expect - 1: break
print("飞控参数总数 = %s, 匹配 %s 前缀的参数共 %d 个:" % (expect, "/".join(pfx), len(found)))
for nm in sorted(found):
    v, idx, cnt = found[nm]
    print("   %-18s = %-12g  (索引 %d/%d)" % (nm, v, idx, cnt))
