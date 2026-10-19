#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit test for the new multi-device discovery logic (no GUI, no adb needed)."""
import sys

sys.path.insert(0, r"C:\Users\Administrator\Desktop\G12Debugger\app")
import usv_debug as U  # noqa: E402

cfgs = dict(U.DEFAULT_CFG)
cfgs["adb_path"] = r"D:\study_sofeware\SDK\platform-tools\adb.exe"
cfgs["mav_port"] = 5760

HDR = "List of devices attached\n"
CASES = [
    ("A. 两个在线设备, 没记住选择 -> 应返回空 addr + 2 候选",
     HDR + "192.168.10.162:43283  device product:a model:G12_NEW device:x transport_id:1\n"
           "192.168.10.171:38211  device product:b model:G12_OLD device:y transport_id:2\n",
     ""),
    ("B. 两个在线设备, 记住了 162 -> 应直接返回 162",
     HDR + "192.168.10.162:43283  device product:a model:G12_NEW device:x transport_id:1\n"
           "192.168.10.171:38211  device product:b model:G12_OLD device:y transport_id:2\n",
     "192.168.10.162"),
    ("C. 两个在线设备, 记住的 199 不在列表 -> 应返回空 addr + 2 候选",
     HDR + "192.168.10.162:43283  device product:a model:G12_NEW device:x transport_id:1\n"
           "192.168.10.171:38211  device product:b model:G12_OLD device:y transport_id:2\n",
     "192.168.10.199"),
    ("D. 一个在线 + 一个掉线 -> 应直接返回在线那个",
     HDR + "192.168.10.162:43283  device product:a model:G12_NEW device:x transport_id:1\n"
           "192.168.10.171:38211  offline\n",
     ""),
    ("E. 全部掉线 -> 应返回空 addr + 2 候选(让人选后重连)",
     HDR + "192.168.10.162:43283  offline\n192.168.10.171:38211  offline\n",
     ""),
    ("F. 同一 IP 两条(重连产生) -> 应去重成 1 条, 在线优先",
     HDR + "192.168.10.162:43283  offline\n"
           "192.168.10.162:99001  device product:a model:G12_NEW device:x transport_id:1\n",
     ""),
]

fail = 0
for title, fake, pref in CASES:
    U.adb_devices = lambda cfg, _o=fake: (0, _o)
    addr, note, cands = U.discover_live_addr(cfgs, prefer_ip=pref)
    print("=" * 74)
    print(title)
    print("   -> addr=%r" % addr)
    print("   -> note=%s" % note)
    print("   -> candidates=%s" % [(c["ip"], c["state"], c["name"]) for c in cands])

# 额外: 解析字段是否完整
print("=" * 74)
U.adb_devices = lambda cfg: (0, CASES[0][1])
devs = U.list_rc_devices(cfgs)
print("list_rc_devices 解析结果:")
for d in devs:
    print("   ", d)
assert all(d["ip"] and d["port"] and d["name"] for d in devs), "字段解析不全"
print("OK: ip/port/model 全部解析成功")
print("\nALL TESTS RAN")
