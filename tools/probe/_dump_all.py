# -*- coding: utf-8 -*-
"""全量拉取飞控参数(只读), 存 JSON, 并与指定快照做全量对比"""
import sys, time, json, os
from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "192.168.43.138:5760"
out = sys.argv[2] if len(sys.argv) > 2 else "Logs/param_snapshot_now.json"
ref = sys.argv[3] if len(sys.argv) > 3 else "Logs/param_snapshot_live.json"

m = mavutil.mavlink_connection("tcp:" + addr, source_system=255)
if not m.wait_heartbeat(timeout=10):
    print("无 HEARTBEAT")
    sys.exit(1)
print("已连接，开始拉取全部参数 ...")
m.mav.param_request_list_send(1, 1)

got = {}
t0 = time.time()
last_progress = 0.0
expect = None
while time.time() - t0 < 90:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
    if msg is None:
        break
    nm = msg.param_id
    if isinstance(nm, bytes):
        nm = nm.decode(errors="replace")
    nm = nm.replace("\x00", "").strip()
    if not nm:
        continue
    got[nm] = float(msg.param_value)
    if msg.param_count:
        expect = int(msg.param_count)
    now = time.time() - t0
    if now - last_progress > 10:
        last_progress = now
        print("   ... %d / %s" % (len(got), expect))
    if expect and len(got) >= expect:
        break

print("拿到 %d 个参数 (预期 %s)" % (len(got), expect))
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as f:
    json.dump({"FORMAT_VERSION": "dump_all", "params": got}, f,
              ensure_ascii=False, indent=1)
print("已保存:", out)

if not os.path.exists(ref):
    sys.exit(0)

d = json.load(open(ref, encoding="utf-8"))
old = d.get("params", d)
if isinstance(old, list):
    old = {x["name"]: x["value"] for x in old}

diff = []
for k in sorted(set(old) | set(got)):
    a, b = old.get(k), got.get(k)
    if a is None:
        diff.append((k, None, b))
    elif b is None:
        diff.append((k, a, None))
    elif abs(a - b) > max(1e-6, abs(a) * 1e-6):
        diff.append((k, a, b))

print("\n" + "=" * 64)
print(" 与快照 %s 的差异: %d 个参数" % (ref, len(diff)))
print("=" * 64)
for k, a, b in diff:
    if a is None:
        print("   [新增] %-22s %s" % (k, b))
    elif b is None:
        print("   [消失] %-22s 原=%s" % (k, a))
    else:
        pct = (b - a) / abs(a) * 100 if a else float("inf")
        print("   %-22s %-14g -> %-14g (%+.1f%%)" % (k, a, b, pct))
